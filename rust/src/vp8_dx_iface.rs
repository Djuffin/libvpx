//! VP8 decoder adapter — the [`Decoder`](crate::codec::Decoder)
//! implementation and the private `Vp8AlgPriv` state it owns.
//! Translated from `vp8/vp8_dx_iface.c`; public types come from
//! `crate::vpx_api`.

#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]

use core::ffi::c_void;
use core::ptr;

use crate::onyxd_if::{
    vp8_create_decoder_instances, vp8_remove_decoder_instances, vp8dx_get_raw_frame,
    vp8dx_get_reference, vp8dx_set_reference,
};
use crate::types::{
    FragmentData, FrameBuffers, FrameView, VP8_BORDER_IN_PIXELS, MAX_PARTITIONS,
    PlaneRef, Vp8PpFlags, Vp8dComp, Vp8dConfig, VpxInternalErrorInfo, Yv12BufferConfig,
};
use crate::vpx_api::*;
use crate::vpx_codec::vpx_internal_error;

// ===========================================================================
// Local types — adapter-private, not part of the public libvpx API.
// ===========================================================================

/// Minimal build: CONFIG_POSTPROC=0, CONFIG_ERROR_CONCEALMENT=0,
/// CONFIG_MULTITHREAD=0.
pub const CONFIG_POSTPROC: i32 = 0;
pub const CONFIG_ERROR_CONCEALMENT: i32 = 0;
pub const CONFIG_MULTITHREAD: i32 = 0;

/// Expansion of `VP8_CAP_POSTPROC` (`vp8_dx_iface.c:34`).
pub const VP8_CAP_POSTPROC: VpxCodecCaps = 0;
/// Expansion of `VP8_CAP_ERROR_CONCEALMENT` (`vp8_dx_iface.c:35-36`).
pub const VP8_CAP_ERROR_CONCEALMENT: VpxCodecCaps = 0;

/// VP8 carries no extra fields — `vp8_stream_info_t` is an alias.
/// (`vp8_dx_iface.c:38`)
pub type Vp8StreamInfo = VpxCodecStreamInfo;

/// `vpx_decrypt_cb` (`vpx/vp8dx.h`).
pub type VpxDecryptCb = Option<
    unsafe extern "C" fn(decrypt_state: *mut c_void, input: *const u8, output: *mut u8, count: i32),
>;

/// `vpx_decrypt_init` (`vpx/vp8dx.h:174`).
#[repr(C)]
pub struct VpxDecryptInit {
    pub decrypt_cb: VpxDecryptCb,
    pub decrypt_state: *mut c_void,
}

/// `vp8_postproc_cfg_t` (`vpx/vp8.h`).
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct Vp8PostprocCfg {
    pub post_proc_flag: i32,
    pub deblocking_level: i32,
    pub noise_level: i32,
}

/// Postproc flag bits.
pub const VP8_DEBLOCK: i32 = 0x1;
pub const VP8_DEMACROBLOCK: i32 = 0x2;
pub const VP8_MFQE: i32 = 0x10;

/// `vpx_ref_frame_t` (`vpx/vp8.h`).
#[repr(C)]
pub struct VpxRefFrame {
    pub frame_type: i32,
    pub img: VpxImage,
}

/// Control IDs from `vpx/vp8.h`, `vpx/vp8dx.h`. Used by
/// `vpx_codec_control_` to map a legacy ctrl_id onto a typed
/// [`crate::codec::ControlCmd`] variant.
pub const VP8_SET_REFERENCE: i32 = 1;
pub const VP8_COPY_REFERENCE: i32 = 2;
pub const VP8_SET_POSTPROC: i32 = 3;
pub const VP8D_GET_LAST_REF_UPDATES: i32 = 9;
pub const VP8D_GET_FRAME_CORRUPTED: i32 = 10;
pub const VP8D_GET_LAST_REF_USED: i32 = 11;
pub const VPXD_GET_LAST_QUANTIZER: i32 = 12;
pub const VPXD_SET_DECRYPTOR: i32 = 13;

/// VP8-decoder private state (`vp8_dx_iface.c:44-64`'s
/// `vpx_codec_alg_priv_t`). Owned by [`Vp8Decoder`].
pub struct Vp8AlgPriv<'a> {
    pub base: VpxCodecPriv,
    pub cfg: VpxCodecDecCfg,
    pub si: Vp8StreamInfo,
    pub decoder_init: i32,
    // CONFIG_MULTITHREAD-only `restart_threads` omitted in minimal build.
    pub postproc_cfg_set: i32,
    pub postproc_cfg: Vp8PostprocCfg,
    pub img: VpxImage,
    pub img_setup: i32,
    pub yv12_frame_buffers: FrameBuffers<'a>,
    pub allocator: Option<std::sync::Arc<dyn crate::api::VideoFrameAllocator>>,
    pub user_priv: *mut c_void,
    pub fragments: FragmentData,
}

use crate::onyxd_if::{
    vp8dx_get_quantizer, vp8dx_receive_compressed_data, vp8dx_references_buffer,
};

use crate::alloccommon::vp8_alloc_frame_buffers;
use crate::mbpitch::vp8_build_block_doffsets;
use crate::rtcd::vp8_rtcd;
use crate::vpx_dsp_rtcd::vpx_dsp_rtcd;
use crate::vpx_scale_rtcd::vpx_scale_rtcd;

use crate::types::{
    ALTREF_FRAME, GOLDEN_FRAME, LAST_FRAME, VP8_ALTR_FRAME, VP8_GOLD_FRAME, VP8_LAST_FRAME,
};

// ===========================================================================
// `vp8_dx_iface.c` static helpers
// ===========================================================================

/// `vp8_peek_si_internal` — `vp8/vp8_dx_iface.c:127`.
unsafe fn vp8_peek_si_internal(
    data: *const u8,
    data_sz: u32,
    si: *mut VpxCodecStreamInfo,
) -> VpxCodecErr {
    let mut res: VpxCodecErr = VPX_CODEC_OK;

    if data.is_null() {
        return VPX_CODEC_INVALID_PARAM;
    }

    // `if (data + data_sz <= data)` — wrap-around guard.
    if (data as usize).wrapping_add(data_sz as usize) <= data as usize {
        res = VPX_CODEC_INVALID_PARAM;
    } else {
        // Parse uncompressed part of key frame header.
        //  3 bytes:- including version, frame type and an offset
        //  3 bytes:- sync code (0x9d, 0x01, 0x2a)
        //  4 bytes:- including image width and height in the lowest 14 bits
        //            of each 2-byte value.
        let clear: *const u8 = data;
        (*si).is_kf = 0;

        if data_sz >= 10 && (*clear.add(0) & 0x01) == 0 {
            // I-Frame
            (*si).is_kf = 1;

            // vet via sync code
            if *clear.add(3) != 0x9d || *clear.add(4) != 0x01 || *clear.add(5) != 0x2a {
                return VPX_CODEC_UNSUP_BITSTREAM;
            }

            (*si).w = ((*clear.add(6) as u32) | ((*clear.add(7) as u32) << 8)) & 0x3fff;
            (*si).h = ((*clear.add(8) as u32) | ((*clear.add(9) as u32) << 8)) & 0x3fff;

            if !((*si).h != 0 && (*si).w != 0) {
                (*si).w = 0;
                (*si).h = 0;
                res = VPX_CODEC_CORRUPT_FRAME;
            }
        } else {
            res = VPX_CODEC_UNSUP_BITSTREAM;
        }
    }

    res
}

/// `vp8_peek_si` — `vp8/vp8_dx_iface.c:178`.
pub unsafe fn vp8_peek_si(
    data: *const u8,
    data_sz: u32,
    si: *mut VpxCodecStreamInfo,
) -> VpxCodecErr {
    vp8_peek_si_internal(data, data_sz, si)
}

/// `vp8_get_si` — `vp8/vp8_dx_iface.c:183`.
pub unsafe fn vp8_get_si(ctx: &Vp8AlgPriv<'static>, si: *mut VpxCodecStreamInfo) -> VpxCodecErr {
    let sz: u32 = if (*si).sz as usize >= core::mem::size_of::<Vp8StreamInfo>() {
        core::mem::size_of::<Vp8StreamInfo>() as u32
    } else {
        core::mem::size_of::<VpxCodecStreamInfo>() as u32
    };

    ptr::copy_nonoverlapping(
        &ctx.si as *const Vp8StreamInfo as *const u8,
        si as *mut u8,
        sz as usize,
    );
    (*si).sz = sz;

    VPX_CODEC_OK
}

/// `update_error_state` — `vp8/vp8_dx_iface.c:199`. Returns the error
/// code; the variadic detail-message channel is not carried.
fn update_error_state(error: &VpxInternalErrorInfo) -> VpxCodecErr {
    error.error_code
}

/// `yuvconfig2image` — `vp8/vp8_dx_iface.c:210`.
fn yuvconfig2image(img: &mut VpxImage, view: &FrameView, user_priv: *mut c_void) {
    // vpx_img_wrap() doesn't allow specifying independent strides for
    // the Y, U, and V planes, nor other alignment adjustments that
    // might be representable by a YV12_BUFFER_CONFIG, so we just
    // initialize all the fields. The dimensions here are the cropped
    // (display) ones — the C source overwrites the descriptor's
    // y_width/y_height with pc->Width/Height before this conversion.
    img.fmt = VPX_IMG_FMT_I420;
    img.w = view.y_stride as u32;
    img.h = ((view.display_height + 2 * VP8_BORDER_IN_PIXELS + 15) & !15) as u32;
    img.d_w = view.display_width as u32;
    img.r_w = view.display_width as u32;
    img.d_h = view.display_height as u32;
    img.r_h = view.display_height as u32;
    img.x_chroma_shift = 1;
    img.y_chroma_shift = 1;
    img.planes[VPX_PLANE_Y] = view.y_buffer;
    img.planes[VPX_PLANE_U] = view.u_buffer;
    img.planes[VPX_PLANE_V] = view.v_buffer;
    img.planes[VPX_PLANE_ALPHA] = ptr::null_mut();
    img.stride[VPX_PLANE_Y] = view.y_stride;
    img.stride[VPX_PLANE_U] = view.uv_stride;
    img.stride[VPX_PLANE_V] = view.uv_stride;
    img.stride[VPX_PLANE_ALPHA] = view.y_stride;
    img.bit_depth = 8;
    img.bps = 12;
    img.user_priv = user_priv;
    img.img_data = view.buffer_alloc;
    img.img_data_owner = 0;
    img.self_allocd = 0;
}

/// `update_fragments` — `vp8/vp8_dx_iface.c:239`.
fn update_fragments(
    ctx: &mut Vp8AlgPriv<'static>,
    data: *const u8,
    data_sz: u32,
    res: &mut VpxCodecErr,
) -> i32 {
    *res = VPX_CODEC_OK;

    if ctx.fragments.count == 0 {
        // New frame, reset fragment pointers and sizes
        ctx.fragments.ptrs.fill(ptr::null());
        ctx.fragments.sizes.fill(0);
    }

    // Flush signal in fragment mode but no fragments were accumulated yet.
    // Nothing to decode; treat as a no-op.
    if ctx.fragments.enabled != 0
        && data.is_null()
        && data_sz == 0
        && ctx.fragments.count == 0
    {
        return 0;
    }

    if ctx.fragments.enabled != 0 && !(data.is_null() && data_sz == 0) {
        // Store a pointer to this fragment and return. We haven't
        // received the complete frame yet, so we will wait with decoding.
        if ctx.fragments.count as usize >= MAX_PARTITIONS {
            ctx.fragments.count = 0;
            *res = VPX_CODEC_INVALID_PARAM;
            return -1;
        }
        ctx.fragments.ptrs[ctx.fragments.count as usize] = data;
        ctx.fragments.sizes[ctx.fragments.count as usize] = data_sz;
        ctx.fragments.count += 1;
        return 0;
    }

    if ctx.fragments.enabled == 0 && data.is_null() && data_sz == 0 {
        return 0;
    }

    if ctx.fragments.enabled == 0 {
        ctx.fragments.ptrs[0] = data;
        ctx.fragments.sizes[0] = data_sz;
        ctx.fragments.count = 1;
    }

    1
}

/// `vp8_decode` — `vp8/vp8_dx_iface.c:285`.
pub unsafe fn vp8_decode(
    ctx: *mut Vp8AlgPriv<'static>,
    data: *const u8,
    data_sz: u32,
) -> VpxCodecErr {
    // `user_priv` is already on `(*ctx).user_priv` — populated by
    // `Vp8Decoder::set_user_priv` before this call. Read by
    // `vp8_get_frame` → `yuvconfig2image` → `(*img).user_priv`.
    let mut res: VpxCodecErr;
    let mut resolution_change: u32 = 0;
    let w: u32;
    let h: u32;

    if (*ctx).fragments.enabled == 0 && data.is_null() && data_sz == 0 {
        return VPX_CODEC_OK;
    }

    // Update the input fragment data
    let mut res_local: VpxCodecErr = VPX_CODEC_OK;
    if update_fragments(&mut *ctx, data, data_sz, &mut res_local) <= 0 {
        return res_local;
    }

    // Determine the stream parameters. Note that we rely on peek_si to
    // validate that we have a buffer that does not wrap around the top
    // of the heap.
    w = (*ctx).si.w;
    h = (*ctx).si.h;

    res = vp8_peek_si_internal(
        (*ctx).fragments.ptrs[0],
        (*ctx).fragments.sizes[0],
        &mut (*ctx).si,
    );

    if res == VPX_CODEC_UNSUP_BITSTREAM && (*ctx).si.is_kf == 0 {
        // the peek function returns an error for non keyframes, however for
        // this case, it is not an error
        res = VPX_CODEC_OK;
    }

    if (*ctx).decoder_init == 0 && (*ctx).si.is_kf == 0 {
        res = VPX_CODEC_UNSUP_BITSTREAM;
    }
    if res == VPX_CODEC_OK
        && (*ctx).decoder_init != 0
        && w == 0
        && h == 0
        && (*ctx).si.h == 0
        && (*ctx).si.w == 0
    {
        let pbi = (*ctx).yv12_frame_buffers.pbi_ptr();
        assert!(!pbi.is_null());
        let _ = vpx_internal_error::<()>(&mut (*pbi).common.error, VPX_CODEC_CORRUPT_FRAME);
        return VPX_CODEC_CORRUPT_FRAME;
    }

    if (*ctx).si.h != h || (*ctx).si.w != w {
        resolution_change = 1;
    }

    // CONFIG_MULTITHREAD block omitted (minimal build).

    // Initialize the decoder instance on the first frame
    if res == VPX_CODEC_OK && (*ctx).decoder_init == 0 {
        let mut oxcf: Vp8dConfig = Vp8dConfig::default();

        oxcf.width = (*ctx).si.w as i32;
        oxcf.height = (*ctx).si.h as i32;
        oxcf.version = 9;
        oxcf.postprocess = 0;
        oxcf.max_threads = (*ctx).cfg.threads as i32;
        oxcf.error_concealment =
            (((*ctx).base.init_flags & VPX_CODEC_USE_ERROR_CONCEALMENT) != 0) as i32;

        // If postprocessing was enabled by the application and a
        // configuration has not been provided, default it.
        if (*ctx).postproc_cfg_set == 0 && ((*ctx).base.init_flags & VPX_CODEC_USE_POSTPROC) != 0 {
            (*ctx).postproc_cfg.post_proc_flag = VP8_DEBLOCK | VP8_DEMACROBLOCK | VP8_MFQE;
            (*ctx).postproc_cfg.deblocking_level = 4;
            (*ctx).postproc_cfg.noise_level = 0;
        }

        let rc = vp8_create_decoder_instances(&mut (*ctx).yv12_frame_buffers, &oxcf);
        res = if rc == VPX_CODEC_OK as i32 {
            VPX_CODEC_OK
        } else {
            VPX_CODEC_ERROR
        };
        if res == VPX_CODEC_OK {
            (*ctx).decoder_init = 1;
        } else {
            // on failure clear the cached resolution to ensure a full
            // reallocation is attempted on resync.
            (*ctx).si.w = 0;
            (*ctx).si.h = 0;
        }
    }

    if res == VPX_CODEC_OK {
        let pbi = (*ctx).yv12_frame_buffers.pbi_ptr();
        let pc = &mut (*pbi).common as *mut crate::types::Vp8Common;
        if resolution_change != 0 {
            (*pc).width = (*ctx).si.w as i32;
            (*pc).height = (*ctx).si.h as i32;
            if vp8_decode_resolution_change(&mut *pbi, w, h, (*ctx).allocator.as_deref()).is_err() {
                res = update_error_state(&(*pbi).common.error);
                (*ctx).fragments.count = 0;
                return res;
            }

            // required to get past the first get_free_fb() call
            (*pbi).common.fb_idx_ref_cnt[0] = 0;
        }

        // update the pbi fragment data
        (*pbi).fragments = (*ctx).fragments;
        if vp8dx_receive_compressed_data(&mut *pbi).is_err() {
            (*pc).yv12_fb[(*pc).lst_fb_idx as usize].corrupted = 1;
            if (*pc).fb_idx_ref_cnt[(*pc).new_fb_idx as usize] > 0 {
                (*pc).fb_idx_ref_cnt[(*pc).new_fb_idx as usize] -= 1;
            }
            res = update_error_state(&(*pbi).common.error);
        }

        // get ready for the next series of fragments
        (*ctx).fragments.count = 0;
    }

    res
}

/// Resolution-change branch of `vp8_decode` (`vp8/vp8_dx_iface.c:402`).
/// Returns `Err` if any width/height validation or `vp8_alloc_frame_buffers`
/// fails.
unsafe fn vp8_decode_resolution_change(
    pbi: &mut Vp8dComp<'static>,
    w: u32,
    h: u32,
    allocator: Option<&dyn crate::api::VideoFrameAllocator>,
) -> VpxResult<()> {
    if pbi.common.width <= 0 {
        pbi.common.width = w as i32;
        return vpx_internal_error(&mut pbi.common.error, VPX_CODEC_CORRUPT_FRAME);
    }

    if pbi.common.height <= 0 {
        pbi.common.height = h as i32;
        return vpx_internal_error(&mut pbi.common.error, VPX_CODEC_CORRUPT_FRAME);
    }

    let (cw, ch) = (pbi.common.width, pbi.common.height);
    if let Some(allocator) = allocator {
        crate::alloccommon::vp8_de_alloc_frame_buffers(&mut pbi.common);

        let mut width = cw;
        let mut height = ch;
        if (width & 0xf) != 0 { width += 16 - (width & 0xf); }
        if (height & 0xf) != 0 { height += 16 - (height & 0xf); }

        for i in 0..crate::types::NUM_YV12_BUFFERS {
            match crate::yv12config::vp8_yv12_alloc_external_frame_buffer(
                &mut pbi.common.yv12_fb[i],
                width,
                height,
                VP8_BORDER_IN_PIXELS,
                allocator
            ) {
                Ok(_) => {}
                Err(e) => {
                    pbi.latest_alloc_error = Some(e);
                    crate::alloccommon::vp8_de_alloc_frame_buffers(&mut pbi.common);
                    return vpx_internal_error(&mut pbi.common.error, VPX_CODEC_MEM_ERROR);
                }
            }
        }

        pbi.common.new_fb_idx = 0;
        pbi.common.lst_fb_idx = 1;
        pbi.common.gld_fb_idx = 2;
        pbi.common.alt_fb_idx = 3;

        pbi.common.fb_idx_ref_cnt[0] = 1;
        pbi.common.fb_idx_ref_cnt[1] = 1;
        pbi.common.fb_idx_ref_cnt[2] = 1;
        pbi.common.fb_idx_ref_cnt[3] = 1;

        match crate::yv12config::vp8_yv12_alloc_external_frame_buffer(
            &mut pbi.common.temp_scale_frame,
            width,
            16,
            VP8_BORDER_IN_PIXELS,
            allocator
        ) {
            Ok(_) => {}
            Err(e) => {
                pbi.latest_alloc_error = Some(e);
                crate::alloccommon::vp8_de_alloc_frame_buffers(&mut pbi.common);
                return vpx_internal_error(&mut pbi.common.error, VPX_CODEC_MEM_ERROR);
            }
        }

        pbi.common.mb_rows = height >> 4;
        pbi.common.mb_cols = width >> 4;
        pbi.common.mbs = pbi.common.mb_rows * pbi.common.mb_cols;
        pbi.common.mode_info_stride = pbi.common.mb_cols + 1;
        let count = ((pbi.common.mb_cols + 1) * (pbi.common.mb_rows + 1)) as usize;
        pbi.common.mip = Some(Box::<[crate::types::ModeInfo]>::new_zeroed_slice(count).assume_init());

        pbi.common.above_context = Some(
            vec![crate::types::EntropyContextPlanes::default(); pbi.common.mb_cols as usize].into_boxed_slice(),
        );
    } else {
        if vp8_alloc_frame_buffers(&mut pbi.common, cw, ch) != 0 {
            return vpx_internal_error(&mut pbi.common.error, VPX_CODEC_MEM_ERROR);
        }
    }

    // xd->pre = pc->yv12_fb[pc->lst_fb_idx];
    // xd->dst = pc->yv12_fb[pc->new_fb_idx];
    // pre/dst are slim plane views: copy plane bases + strides only.
    let lst = pbi.common.lst_fb_idx as usize;
    let new = pbi.common.new_fb_idx as usize;
    pbi.mb.pre = PlaneRef::of(&pbi.common.yv12_fb[lst]);
    pbi.mb.dst = PlaneRef::of(&pbi.common.yv12_fb[new]);

    vp8_build_block_doffsets(&mut pbi.mb);

    // CONFIG_ERROR_CONCEALMENT / CONFIG_MULTITHREAD blocks
    // omitted in the minimal build.
    Ok(())
}

pub fn vp8_get_frame<'a>(
    ctx: &'a mut Vp8AlgPriv<'static>,
    iter: &mut VpxCodecIter,
) -> Option<&'a mut VpxImage> {
    // iter acts as a flip flop, so an image is only returned on the first
    // call to get_frame.
    if iter.is_null() && ctx.yv12_frame_buffers.pbi.is_some() {
        let mut flags: Vp8PpFlags = Vp8PpFlags::default();

        if (ctx.base.init_flags & VPX_CODEC_USE_POSTPROC) != 0 {
            flags.post_proc_flag = ctx.postproc_cfg.post_proc_flag;
            flags.deblocking_level = ctx.postproc_cfg.deblocking_level;
            flags.noise_level = ctx.postproc_cfg.noise_level;
        }

        let pbi = ctx
            .yv12_frame_buffers
            .pbi
            .as_deref_mut()
            .expect("pbi present (guarded by pbi.is_some() above)");
        if let Some(view) = vp8dx_get_raw_frame(pbi, &mut flags) {
            yuvconfig2image(&mut ctx.img, &view, ctx.user_priv);

            let img_ref = &mut ctx.img;
            *iter = img_ref as *mut VpxImage as *const core::ffi::c_void;
            return Some(img_ref);
        }
    }

    None
}

/// `image2yuvconfig` — `vp8/vp8_dx_iface.c:560`.
fn image2yuvconfig(img: &VpxImage, yv12: &mut Yv12BufferConfig) -> VpxCodecErr {
    let y_w = img.d_w as i32;
    let y_h = img.d_h as i32;
    let uv_w = (img.d_w as i32 + 1) / 2;
    let uv_h = (img.d_h as i32 + 1) / 2;
    let res: VpxCodecErr = VPX_CODEC_OK;

    yv12.y_crop_width = y_w;
    yv12.y_crop_height = y_h;
    yv12.y_width = y_w;
    yv12.y_height = y_h;
    yv12.uv_crop_width = uv_w;
    yv12.uv_crop_height = uv_h;
    yv12.uv_width = uv_w;
    yv12.uv_height = uv_h;

    let y_stride = img.stride[VPX_PLANE_Y];
    let uv_stride = img.stride[VPX_PLANE_U];
    yv12.y_stride = y_stride;
    yv12.uv_stride = uv_stride;

    let border = (y_stride - img.d_w as i32) / 2;
    yv12.border = border;
    let b = border / 2;

    // Plane regions span plane+border, with the data pointer stepped back
    // from the caller's visible origin to the region base. This mirrors
    // the C contract that a bordered caller buffer has the surrounding
    // slack (for border == 0 the region simply starts at the origin).
    // SAFETY: the caller guarantees a buffer with the implied border.
    unsafe {
        yv12.y_region = Yv12BufferConfig::plane_region_from_origin(
            img.planes[VPX_PLANE_Y],
            (border * y_stride + border) as usize,
            ((y_h + 2 * border) * y_stride) as usize,
        );
        yv12.u_region = Yv12BufferConfig::plane_region_from_origin(
            img.planes[VPX_PLANE_U],
            (b * uv_stride + b) as usize,
            ((uv_h + 2 * b) * uv_stride) as usize,
        );
        yv12.v_region = Yv12BufferConfig::plane_region_from_origin(
            img.planes[VPX_PLANE_V],
            (b * uv_stride + b) as usize,
            ((uv_h + 2 * b) * uv_stride) as usize,
        );
    }
    yv12.alpha_region = None;
    res
}

// ===========================================================================
// Codec descriptor — `{ name, abi_version, caps }` metadata only.
// Dispatch lives on the `Decoder` trait impl below.
// ===========================================================================

pub static mut VPX_CODEC_VP8_DX_ALGO: VpxCodecIface = VpxCodecIface {
    name: "WebM Project VP8 Decoder",
    abi_version: VPX_CODEC_INTERNAL_ABI_VERSION,
    caps: VPX_CODEC_CAP_DECODER
        | VP8_CAP_POSTPROC
        | VP8_CAP_ERROR_CONCEALMENT
        | VPX_CODEC_CAP_INPUT_FRAGMENTS,
};

/// VP8 decoder iface descriptor.
pub fn vpx_codec_vp8_dx() -> &'static VpxCodecIface {
    unsafe { &*ptr::addr_of!(VPX_CODEC_VP8_DX_ALGO) }
}

// ===========================================================================
// `Decoder` trait implementation.
// ===========================================================================

use crate::codec::{ControlCmd, Decoder, Error, Image};

/// Decoder handle owning a boxed [`Vp8AlgPriv`].
///
/// `iter` mirrors the C `vpx_codec_iter_t` flip-flop: reset to null on
/// each `decode()` call, advanced by `get_frame()`. Lets the trait
/// `get_frame` return `Some` on the first call after a decode and
/// `None` thereafter without exposing the iter to callers.
pub struct Vp8Decoder {
    priv_: Box<Vp8AlgPriv<'static>>,
    iter: VpxCodecIter,
}

impl Vp8Decoder {
    /// Construct a new VP8 decoder over a freshly-zeroed `Vp8AlgPriv`.
    pub fn new(init_flags: VpxCodecFlags) -> Result<Self, Error> {
        // SAFETY: all fields of `Vp8AlgPriv` are zero-init valid —
        // primitive ints, raw pointers (null), arrays of the same, and
        // `Option<Box<dyn FnMut + 'static>>` which uses null-pointer
        // optimization on the data pointer (zero ↦ None).
        let mut priv_: Box<Vp8AlgPriv<'static>> = Box::new(unsafe { core::mem::zeroed() });

        vp8_rtcd();
        vpx_dsp_rtcd();
        vpx_scale_rtcd();

        priv_.base.init_flags = init_flags;
        priv_.si.sz = core::mem::size_of::<Vp8StreamInfo>() as u32;
        priv_.fragments.enabled = ((init_flags & VPX_CODEC_USE_INPUT_FRAGMENTS) != 0) as i32;

        Ok(Vp8Decoder {
            priv_,
            iter: ptr::null(),
        })
    }

    /// Store the caller's decoder configuration. Used by
    /// `vpx_codec_dec_init_ver` when a `cfg` is supplied.
    pub fn set_cfg(&mut self, cfg: VpxCodecDecCfg) {
        self.priv_.cfg = cfg;
    }
}

impl Drop for Vp8Decoder {
    fn drop(&mut self) {
        // Release the YV12 frame buffer pool and the inner Vp8dComp
        // instance. The Box drop that follows reclaims the
        // `Vp8AlgPriv` shell.
        vp8_remove_decoder_instances(&mut self.priv_.yv12_frame_buffers);
    }
}

impl Decoder for Vp8Decoder {
    fn set_user_priv(&mut self, user_priv: *mut c_void) {
        self.priv_.user_priv = user_priv;
    }

    fn decode(&mut self, data: &[u8], _deadline: core::time::Duration) -> Result<(), Error> {
        unsafe {
            let (ptr, len) = if data.is_empty() {
                (ptr::null(), 0u32)
            } else {
                (data.as_ptr(), data.len() as u32)
            };
            // Reset the iter so the next get_frame() reports the
            // newly-decoded image instead of replaying the previous one.
            self.iter = core::ptr::null();
            let err = vp8_decode(&raw mut *self.priv_, ptr, len);
            if err == VPX_CODEC_OK {
                Ok(())
            } else {
                Err(err)
            }
        }
    }

    fn get_frame(&mut self) -> Option<&Image> {
        vp8_get_frame(&mut self.priv_, &mut self.iter).map(|img| &*img)
    }

    fn control(&mut self, cmd: ControlCmd<'_>) -> Result<(), Error> {
        unsafe {
            let ctx: &mut Vp8AlgPriv<'static> = &mut self.priv_;
            match cmd {
                ControlCmd::SetReference(frame) => {
                    let mut sd: Yv12BufferConfig = core::mem::zeroed();
                    image2yuvconfig(&frame.img, &mut sd);
                    let pbi = match ctx.yv12_frame_buffers.pbi.as_deref_mut() {
                        Some(b) => b,
                        None => return Err(VPX_CODEC_CORRUPT_FRAME),
                    };
                    vp8dx_set_reference(pbi, frame.frame_type, &mut sd)
                }
                ControlCmd::CopyReference(frame) => {
                    let mut sd: Yv12BufferConfig = core::mem::zeroed();
                    image2yuvconfig(&frame.img, &mut sd);
                    let pbi = match ctx.yv12_frame_buffers.pbi.as_deref_mut() {
                        Some(b) => b,
                        None => return Err(VPX_CODEC_CORRUPT_FRAME),
                    };
                    vp8dx_get_reference(pbi, frame.frame_type, &mut sd)
                }
                ControlCmd::SetPostproc(_cfg) => {
                    // CONFIG_POSTPROC=0 in the minimal build.
                    Err(VPX_CODEC_INCAPABLE)
                }
                ControlCmd::GetLastRefUpdates(out) => {
                    let pbi = ctx.yv12_frame_buffers.pbi_ptr();
                    if pbi.is_null() {
                        return Err(VPX_CODEC_CORRUPT_FRAME);
                    }
                    *out = (*pbi).common.refresh_alt_ref_frame * VP8_ALTR_FRAME
                        + (*pbi).common.refresh_golden_frame * VP8_GOLD_FRAME
                        + (*pbi).common.refresh_last_frame * VP8_LAST_FRAME;
                    Ok(())
                }
                ControlCmd::GetFrameCorrupted(out) => {
                    let pbi = ctx.yv12_frame_buffers.pbi_ptr();
                    if pbi.is_null() {
                        return Err(VPX_CODEC_INVALID_PARAM);
                    }
                    let idx = (*pbi).common.frame_to_show_idx;
                    if idx < 0 {
                        return Err(VPX_CODEC_ERROR);
                    }
                    *out = (*pbi).common.yv12_fb[idx as usize].corrupted;
                    Ok(())
                }
                ControlCmd::GetLastRefUsed(out) => {
                    let pbi = ctx.yv12_frame_buffers.pbi_ptr();
                    if pbi.is_null() {
                        return Err(VPX_CODEC_CORRUPT_FRAME);
                    }
                    let oci = &mut (*pbi).common;
                    *out = (if vp8dx_references_buffer(oci, ALTREF_FRAME as i32) != 0 {
                        VP8_ALTR_FRAME
                    } else {
                        0
                    }) | (if vp8dx_references_buffer(oci, GOLDEN_FRAME as i32) != 0 {
                        VP8_GOLD_FRAME
                    } else {
                        0
                    }) | (if vp8dx_references_buffer(oci, LAST_FRAME as i32) != 0 {
                        VP8_LAST_FRAME
                    } else {
                        0
                    });
                    Ok(())
                }
                ControlCmd::GetLastQuantizer(out) => {
                    let pbi = match ctx.yv12_frame_buffers.pbi.as_deref() {
                        Some(b) => b,
                        None => return Err(VPX_CODEC_CORRUPT_FRAME),
                    };
                    *out = vp8dx_get_quantizer(pbi);
                    Ok(())
                }
                ControlCmd::SetDecryptor(_init) => {
                    // Bytestream decryption was removed from this build; the
                    // control is kept for ABI shape but reports unsupported.
                    Err(VPX_CODEC_INCAPABLE)
                }
            }
        }
    }

    fn peek_stream_info(data: &[u8]) -> Result<crate::codec::StreamInfo, Error> {
        unsafe {
            let mut si: VpxCodecStreamInfo = core::mem::zeroed();
            si.sz = core::mem::size_of::<VpxCodecStreamInfo>() as u32;
            let res = vp8_peek_si(data.as_ptr(), data.len() as u32, &mut si);
            if res == VPX_CODEC_OK {
                Ok(si)
            } else {
                Err(res)
            }
        }
    }

    fn stream_info(&self) -> Result<crate::codec::StreamInfo, Error> {
        unsafe {
            let mut si: VpxCodecStreamInfo = core::mem::zeroed();
            si.sz = core::mem::size_of::<VpxCodecStreamInfo>() as u32;
            let res = vp8_get_si(&self.priv_, &mut si);
            if res == VPX_CODEC_OK {
                Ok(si)
            } else {
                Err(res)
            }
        }
    }
}

// ===========================================================================
// Unified API VideoFrame implementation.
// ===========================================================================

pub struct PublishedFrame {
    buffer: std::sync::Arc<dyn crate::api::FrameBuffer>,
    y_crop_width: i32,
    y_crop_height: i32,
    y_stride: i32,
    uv_crop_width: i32,
    uv_crop_height: i32,
    uv_stride: i32,
    border: i32,
}

impl PublishedFrame {
    pub fn new(ybf: &crate::types::Yv12BufferConfig, display_width: i32, display_height: i32) -> Self {
        Self {
            buffer: ybf.ext_buffer.clone().expect("external frame buffer must be present"),
            y_crop_width: display_width,
            y_crop_height: display_height,
            y_stride: ybf.y_stride,
            uv_crop_width: (display_width + 1) / 2,
            uv_crop_height: (display_height + 1) / 2,
            uv_stride: ybf.uv_stride,
            border: ybf.border,
        }
    }
}

impl crate::api::VideoFrame for PublishedFrame {
    fn plane(&self, plane: crate::api::VideoPlane) -> Option<crate::api::PlaneView<'_>> {
        let (w, h, stride) = match plane {
            crate::api::VideoPlane::Y => (self.y_crop_width, self.y_crop_height, self.y_stride),
            crate::api::VideoPlane::U => (self.uv_crop_width, self.uv_crop_height, self.uv_stride),
            crate::api::VideoPlane::V => (self.uv_crop_width, self.uv_crop_height, self.uv_stride),
            _ => return None,
        };

        let ptr = self.buffer.plane_ptr(plane)?;
        let border = self.border;
        let b = if plane == crate::api::VideoPlane::Y { border } else { border / 2 };
        let origin = b * stride + b;
        let visible_bytes = (h.saturating_sub(1)) * stride + w;

        let slice_ref = unsafe { ptr.as_ref() };
        let data = &slice_ref[(origin as usize)..(origin as usize + visible_bytes as usize)];

        Some(crate::api::PlaneView {
            plane,
            data,
            stride: stride as usize,
            width: w as usize,
            height: h as usize,
        })
    }

    fn planes(&self) -> [Option<crate::api::PlaneView<'_>>; 4] {
        [
            self.plane(crate::api::VideoPlane::Y),
            self.plane(crate::api::VideoPlane::U),
            self.plane(crate::api::VideoPlane::V),
            None,
        ]
    }
}

// ===========================================================================
// Unified API VideoDecoder wrapper.
// ===========================================================================

pub struct Vp8VideoDecoder {
    decoder: Vp8Decoder,
    allocator: std::sync::Arc<dyn crate::api::VideoFrameAllocator>,
    callbacks: std::sync::Arc<dyn crate::api::VideoDecoderCallbacks>,
    out_queue: std::collections::VecDeque<crate::api::DecodedPicture>,
    pending_opaque: Option<Box<dyn std::any::Any + Send>>,
    last_format: Option<crate::api::StreamFormat>,
}

impl Vp8VideoDecoder {
    pub fn new(
        _config: crate::api::DecoderConfig,
        allocator: std::sync::Arc<dyn crate::api::VideoFrameAllocator>,
        callbacks: std::sync::Arc<dyn crate::api::VideoDecoderCallbacks>,
    ) -> Result<Self, crate::api::DecoderError> {
        let mut decoder = Vp8Decoder::new(0).map_err(|e| crate::api::DecoderError::InitializationFailed(format!("{e:?}")))?;
        decoder.priv_.allocator = Some(allocator.clone());

        Ok(Self {
            decoder,
            allocator,
            callbacks,
            out_queue: std::collections::VecDeque::new(),
            pending_opaque: None,
            last_format: None,
        })
    }
}
impl crate::api::VideoDecoder for Vp8VideoDecoder {
    fn decode(&mut self, packet: crate::api::EncodedPacket) -> Result<(), crate::api::DecoderError> {
        let crate::api::EncodedPacket { data, opaque } = packet;
        let bytes = (*data).as_ref();
        self.pending_opaque = opaque;

        if let Err(e) = self.decoder.decode(bytes, core::time::Duration::ZERO) {
            if e == crate::vpx_api::VpxCodecErr::VPX_CODEC_MEM_ERROR {
                let priv_ref = &mut *self.decoder.priv_;
                if let Some(pbi) = priv_ref.yv12_frame_buffers.pbi.as_mut() {
                    if let Some(alloc_err) = pbi.latest_alloc_error.take() {
                        return Err(crate::api::DecoderError::Alloc(alloc_err));
                    }
                }
                return Err(crate::api::DecoderError::Alloc(crate::api::AllocError::OutOfMemory));
            }
            return Err(crate::api::DecoderError::MisformedData(format!("{e:?}")));
        }

        let priv_ref = &mut *self.decoder.priv_;
        let mut iter = core::ptr::null();
        
        if let Some(_img) = vp8_get_frame(priv_ref, &mut iter) {
            // Retrieve the pbi safely using Option and Box reference:
            let pbi = priv_ref.yv12_frame_buffers.pbi.as_ref().expect("pbi missing");
            let ybf = &pbi.common.yv12_fb[pbi.common.new_fb_idx as usize];

            let format = crate::api::StreamFormat {
                codec: crate::api::Codec::VP8,
                coded_width: ybf.y_width as usize,
                coded_height: ybf.y_height as usize,
                crop_left: 0,
                crop_top: 0,
                display_width: pbi.common.width as usize,
                display_height: pbi.common.height as usize,
                color_space: Some(crate::api::ColorSpace {
                    primaries: crate::api::ColorPrimaries::Unspecified,
                    transfer: crate::api::TransferCharacteristics::Unspecified,
                    matrix: crate::api::MatrixCoefficients::Unspecified,
                    range: crate::api::ColorRange::Limited,
                }),
                pixel_format: crate::api::PixelFormat::I420,
                bit_depth: 8,
            };

            if self.last_format.as_ref() != Some(&format) {
                self.last_format = Some(format.clone());
                self.callbacks.on_format_changed(format.clone());
            }

            let frame: std::sync::Arc<dyn crate::api::VideoFrame> = std::sync::Arc::new(PublishedFrame::new(ybf, pbi.common.width, pbi.common.height));
            self.out_queue.push_back(crate::api::DecodedPicture {
                frame,
                format,
                opaque: self.pending_opaque.take(),
            });

            self.callbacks.on_picture_available();
        }

        Ok(())
    }

    fn get_picture(&mut self) -> Result<Option<crate::api::DecodedPicture>, crate::api::DecoderError> {
        Ok(self.out_queue.pop_front())
    }

    fn flush(&mut self, mode: crate::api::FlushMode) -> Result<(), crate::api::DecoderError> {
        match mode {
            crate::api::FlushMode::Discard => {
                self.out_queue.clear();
                self.pending_opaque = None;
            }
            crate::api::FlushMode::Drain => {}
        }
        Ok(())
    }

    fn control(&mut self, cmd: &mut crate::api::ControlCmd) -> Result<(), crate::api::DecoderError> {
        if let Some(c) = cmd.downcast_ref::<crate::api::Vp8SetReference>() {
            let ref_frame = VpxRefFrame {
                frame_type: c.frame_type,
                img: c.img,
            };
            self.decoder.control(crate::codec::ControlCmd::SetReference(&ref_frame))
                .map_err(|e| crate::api::DecoderError::Fatal(format!("{e:?}")))?;
            return Ok(());
        }
        if let Some(c) = cmd.downcast_ref::<crate::api::Vp8CopyReference>() {
            let mut ref_frame = VpxRefFrame {
                frame_type: c.frame_type,
                img: c.img.get(),
            };
            self.decoder.control(crate::codec::ControlCmd::CopyReference(&mut ref_frame))
                .map_err(|e| crate::api::DecoderError::Fatal(format!("{e:?}")))?;
            c.img.set(ref_frame.img);
            return Ok(());
        }
        if let Some(c) = cmd.downcast_ref::<crate::api::Vp8GetLastRefUpdates>() {
            let mut val = 0;
            self.decoder.control(crate::codec::ControlCmd::GetLastRefUpdates(&mut val))
                .map_err(|e| crate::api::DecoderError::Fatal(format!("{e:?}")))?;
            c.out.set(val);
            return Ok(());
        }
        if let Some(c) = cmd.downcast_ref::<crate::api::Vp8GetFrameCorrupted>() {
            let mut val = 0;
            self.decoder.control(crate::codec::ControlCmd::GetFrameCorrupted(&mut val))
                .map_err(|e| crate::api::DecoderError::Fatal(format!("{e:?}")))?;
            c.out.set(val);
            return Ok(());
        }
        if let Some(c) = cmd.downcast_ref::<crate::api::Vp8GetLastRefUsed>() {
            let mut val = 0;
            self.decoder.control(crate::codec::ControlCmd::GetLastRefUsed(&mut val))
                .map_err(|e| crate::api::DecoderError::Fatal(format!("{e:?}")))?;
            c.out.set(val);
            return Ok(());
        }
        if let Some(c) = cmd.downcast_ref::<crate::api::Vp8GetLastQuantizer>() {
            let mut val = 0;
            self.decoder.control(crate::codec::ControlCmd::GetLastQuantizer(&mut val))
                .map_err(|e| crate::api::DecoderError::Fatal(format!("{e:?}")))?;
            c.out.set(val);
            return Ok(());
        }

        Err(crate::api::DecoderError::FeatureNotSupported(
            "unknown control payload for VP8 Decoder".into()
        ))
    }
}

unsafe impl Send for Vp8VideoDecoder {}
unsafe impl Sync for Vp8VideoDecoder {}



