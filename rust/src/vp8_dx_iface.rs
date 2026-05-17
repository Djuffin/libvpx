//! Literal Rust translation of `vp8/vp8_dx_iface.c` — the VP8 decoder's
//! `vpx_codec_iface_t` adapter. See documentation/vp8_files/vp8_dx_iface.md.
//!
//! Public types and constants come from `crate::vpx_api`. Concrete
//! adapter-private structs (`Vp8AlgPriv`, frame-buffer wrapper, etc.)
//! stay local since they describe state internal to this translation
//! unit.

#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]

use core::ffi::c_void;
use core::ptr;

use crate::types::{FragmentData, FrameBuffers, Vp8dConfig, Vp8dComp, Vp8PpFlags, Yv12BufferConfig, VpxInternalErrorInfo, MAX_PARTITIONS, MAX_FB_MT_DEC, VP8_BORDER_IN_PIXELS};
use crate::vpx_api::*;
use crate::vpx_codec::vpx_internal_error;
use crate::onyxd_if::{
    vp8_create_decoder_instances, vp8_remove_decoder_instances, vp8dx_get_raw_frame,
    vp8dx_get_reference, vp8dx_set_reference,
};

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

/// `NELEMENTS` (`vp8_dx_iface.c:42`).
#[inline]
pub const fn n_elements<T, const N: usize>(_x: &[T; N]) -> i32 {
    N as i32
}

/// `mem_seg_id_t` (`vp8_dx_iface.c:41`).
#[repr(i32)]
#[derive(Copy, Clone)]
pub enum MemSegId {
    Vp8SegAlgPriv = 256,
    Vp8SegMax,
}

/// VP8 carries no extra fields — `vp8_stream_info_t` is an alias.
/// (`vp8_dx_iface.c:38`)
pub type Vp8StreamInfo = VpxCodecStreamInfo;

/// `vpx_decrypt_cb` (`vpx/vp8dx.h`).
pub type VpxDecryptCb = Option<
    unsafe extern "C" fn(
        decrypt_state: *mut c_void,
        input: *const u8,
        output: *mut u8,
        count: i32,
    ),
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

/// Reference-frame bitmap constants (`vpx/vp8.h`).
pub const VP8_LAST_FRAME: i32 = 1;
pub const VP8_GOLD_FRAME: i32 = 2;
pub const VP8_ALTR_FRAME: i32 = 4;

/// `MV_REFERENCE_FRAME` values consumed by `vp8dx_references_buffer`
/// (`blockd.h`).
pub const INTRA_FRAME: i32 = 0;
pub const LAST_FRAME: i32 = 1;
pub const GOLDEN_FRAME: i32 = 2;
pub const ALTREF_FRAME: i32 = 3;

/// Control IDs used by `vp8_ctf_maps` (`vpx/vp8.h`, `vpx/vp8dx.h`).
pub const VP8_SET_REFERENCE: i32 = 1;
pub const VP8_COPY_REFERENCE: i32 = 2;
pub const VP8_SET_POSTPROC: i32 = 3;
pub const VP8D_GET_LAST_REF_UPDATES: i32 = 9;
pub const VP8D_GET_FRAME_CORRUPTED: i32 = 10;
pub const VP8D_GET_LAST_REF_USED: i32 = 11;
pub const VPXD_GET_LAST_QUANTIZER: i32 = 12;
pub const VPXD_SET_DECRYPTOR: i32 = 13;

/// `va_list` placeholder — varargs do not have a stable Rust ABI; the
/// FFI shim that bridges into this module deals with the platform's
/// `va_list`. Local to this translation unit.
#[repr(C)]
pub struct VaList {
    pub raw: *mut c_void,
}

impl VaList {
    pub unsafe fn arg_ptr<T>(&mut self) -> *mut T {
        self.raw as *mut T
    }
    pub unsafe fn arg_i32(&mut self) -> i32 {
        self.raw as i32
    }
}

// Local iface-shaped types matching the iface populated by the static
// `VPX_CODEC_VP8_DX_ALGO`. They are distinct from the canonical types
// in `vpx_api` because the function-pointer signatures here use plain
// `unsafe fn` rather than `unsafe extern "C" fn` — the VP8 decoder
// implementations are translated as ordinary Rust functions and never
// cross the C ABI directly. A future cleanup pass can collapse these
// into the canonical types once trampolines are added.

pub type Vp8DxCtrlFn = unsafe fn(*mut Vp8AlgPriv<'static>, VaList) -> VpxCodecErr;

#[repr(C)]
pub struct Vp8DxCtrlFnMap {
    pub ctrl_id: i32,
    pub fn_: Option<Vp8DxCtrlFn>,
}

pub type Vp8DxInitFn =
    unsafe fn(*mut VpxCodecCtx, *mut VpxCodecPrivEncMrCfg) -> VpxCodecErr;
pub type Vp8DxDestroyFn = unsafe fn(*mut Vp8AlgPriv<'static>) -> VpxCodecErr;
pub type Vp8DxPeekSiFn =
    unsafe fn(*const u8, u32, *mut VpxCodecStreamInfo) -> VpxCodecErr;
pub type Vp8DxGetSiFn =
    unsafe fn(*mut Vp8AlgPriv<'static>, *mut VpxCodecStreamInfo) -> VpxCodecErr;
pub type Vp8DxDecodeFn =
    unsafe fn(*mut Vp8AlgPriv<'static>, *const u8, u32, *mut c_void) -> VpxCodecErr;
pub type Vp8DxFrameGetFn =
    unsafe fn(*mut Vp8AlgPriv<'static>, *mut VpxCodecIter) -> *mut VpxImage;
pub type Vp8DxSetFbFn = unsafe fn() -> VpxCodecErr;

#[repr(C)]
pub struct Vp8DxIfaceDec {
    pub peek_si: Option<Vp8DxPeekSiFn>,
    pub get_si: Option<Vp8DxGetSiFn>,
    pub decode: Option<Vp8DxDecodeFn>,
    pub frame_get: Option<Vp8DxFrameGetFn>,
    pub set_fb_fn: Option<Vp8DxSetFbFn>,
}

#[repr(C)]
pub struct Vp8DxIfaceEnc {
    pub cfg_map_count: i32,
    pub cfg_maps: *mut c_void,
    pub encode: *mut c_void,
    pub get_cx_data: *mut c_void,
    pub cfg_set: *mut c_void,
    pub get_global_headers: *mut c_void,
    pub get_preview_frame: *mut c_void,
    pub mr_get_mem_loc: *mut c_void,
    pub mr_free_mem_loc: *mut c_void,
}

/// VP8-decoder-specific `vpx_codec_iface_t` layout — function-pointer
/// types use the local plain-Rust shapes above. Distinct from
/// `vpx_api::VpxCodecIface` (which uses `extern "C"` signatures).
#[repr(C)]
pub struct Vp8DxIface {
    pub name: *const u8,
    pub abi_version: i32,
    pub caps: VpxCodecCaps,
    pub init: Option<Vp8DxInitFn>,
    pub destroy: Option<Vp8DxDestroyFn>,
    pub ctrl_maps: *mut Vp8DxCtrlFnMap,
    pub dec: Vp8DxIfaceDec,
    pub enc: Vp8DxIfaceEnc,
}

unsafe impl Sync for Vp8DxIface {}

/// `vpx_codec_alg_priv_t` for the VP8 decoder (`vp8_dx_iface.c:44-64`).
/// Concrete adapter-private state — distinct from `vpx_api::VpxCodecAlgPriv`
/// (the opaque pointer type used in the iface vtable).
#[repr(C)]
pub struct Vp8AlgPriv<'a> {
    pub base: VpxCodecPriv,
    pub cfg: VpxCodecDecCfg,
    pub si: Vp8StreamInfo,
    pub decoder_init: i32,
    // CONFIG_MULTITHREAD-only `restart_threads` omitted in minimal build.
    pub postproc_cfg_set: i32,
    pub postproc_cfg: Vp8PostprocCfg,
    pub decrypt_cb: VpxDecryptCb,
    pub decrypt_state: *mut c_void,
    pub img: VpxImage,
    pub img_setup: i32,
    pub yv12_frame_buffers: FrameBuffers<'a>,
    pub user_priv: *mut c_void,
    pub fragments: FragmentData,
}

// ===========================================================================
// External Rust dependencies (functions defined in sibling source files,
// translated separately).
// ===========================================================================

use crate::vpx_mem::{vpx_calloc, vpx_free};

// From `vp8/decoder/onyxd_if.rs`. The `*_inner` adapter calls below
// import the new-style `VpxResult`-returning helpers directly.
use crate::onyxd_if::{
    vp8dx_get_quantizer, vp8dx_receive_compressed_data, vp8dx_references_buffer,
};

use crate::alloccommon::vp8_alloc_frame_buffers;
use crate::mbpitch::vp8_build_block_doffsets;
use crate::rtcd::vp8_rtcd;
use crate::vpx_dsp_rtcd::vpx_dsp_rtcd;
use crate::vpx_scale_rtcd::vpx_scale_rtcd;

use crate::vpx_ports::vpx_clear_system_state;

// ===========================================================================
// Helpers
// ===========================================================================

/// `VPXMIN` macro.
#[inline]
fn vpx_min<T: PartialOrd>(a: T, b: T) -> T {
    if a < b {
        a
    } else {
        b
    }
}

/// `vp8_zero` macro — memset a struct to zero.
#[inline]
unsafe fn vp8_zero<T>(t: &mut T) {
    ptr::write_bytes(t as *mut T, 0, 1);
}

// ===========================================================================
// `vp8_dx_iface.c` static helpers
// ===========================================================================

/// `vp8_init_ctx` — `vp8/vp8_dx_iface.c:66`.
unsafe fn vp8_init_ctx(ctx: *mut VpxCodecCtx) -> i32 {
    let priv_ =
        vpx_calloc(1, core::mem::size_of::<Vp8AlgPriv<'static>>()) as *mut Vp8AlgPriv<'static>;
    if priv_.is_null() {
        return 1;
    }

    (*ctx).priv_ = priv_ as *mut VpxCodecPriv;
    (*(*ctx).priv_).init_flags = (*ctx).init_flags;

    (*priv_).si.sz = core::mem::size_of::<Vp8StreamInfo>() as u32;
    (*priv_).decrypt_cb = None;
    (*priv_).decrypt_state = ptr::null_mut();

    if !(*ctx).config.dec.is_null() {
        // Update the reference to the config structure to an internal copy.
        (*priv_).cfg = *(*ctx).config.dec;
        (*ctx).config.dec = &mut (*priv_).cfg;
    }

    0
}

/// `vp8_init` — `vp8/vp8_dx_iface.c:87`. Vtable `init` slot.
pub unsafe fn vp8_init(
    ctx: *mut VpxCodecCtx,
    data: *mut VpxCodecPrivEncMrCfg,
) -> VpxCodecErr {
    let res: VpxCodecErr = VPX_CODEC_OK;
    let _ = data;

    vp8_rtcd();
    vpx_dsp_rtcd();
    vpx_scale_rtcd();

    // This function only allocates space for the vpx_codec_alg_priv_t
    // structure. More memory may be required at the time the stream
    // information becomes known.
    if (*ctx).priv_.is_null() {
        if vp8_init_ctx(ctx) != 0 {
            return VPX_CODEC_MEM_ERROR;
        }

        let priv_ = (*ctx).priv_ as *mut Vp8AlgPriv<'static>;

        // initialize number of fragments to zero
        (*priv_).fragments.count = 0;
        // is input fragments enabled?
        (*priv_).fragments.enabled =
            (((*priv_).base.init_flags & VPX_CODEC_USE_INPUT_FRAGMENTS) != 0) as i32;

        // post processing level initialized to do nothing
    }

    res
}

/// `vp8_destroy` — `vp8/vp8_dx_iface.c:119`. Vtable `destroy` slot.
pub unsafe fn vp8_destroy(ctx: *mut Vp8AlgPriv<'static>) -> VpxCodecErr {
    vp8_remove_decoder_instances(&mut (*ctx).yv12_frame_buffers);

    vpx_free(ctx as *mut c_void);

    VPX_CODEC_OK
}

/// `vp8_peek_si_internal` — `vp8/vp8_dx_iface.c:127`.
unsafe fn vp8_peek_si_internal(
    data: *const u8,
    data_sz: u32,
    si: *mut VpxCodecStreamInfo,
    decrypt_cb: VpxDecryptCb,
    decrypt_state: *mut c_void,
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
        let mut clear_buffer: [u8; 10] = [0; 10];
        let mut clear: *const u8 = data;
        if let Some(cb) = decrypt_cb {
            let n = vpx_min(clear_buffer.len() as u32, data_sz);
            cb(decrypt_state, data, clear_buffer.as_mut_ptr(), n as i32);
            clear = clear_buffer.as_ptr();
        }
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

/// `vp8_peek_si` — `vp8/vp8_dx_iface.c:178`. Vtable `dec.peek_si` slot.
pub unsafe fn vp8_peek_si(
    data: *const u8,
    data_sz: u32,
    si: *mut VpxCodecStreamInfo,
) -> VpxCodecErr {
    vp8_peek_si_internal(data, data_sz, si, None, ptr::null_mut())
}

/// `vp8_get_si` — `vp8/vp8_dx_iface.c:183`. Vtable `dec.get_si` slot.
pub unsafe fn vp8_get_si(
    ctx: *mut Vp8AlgPriv<'static>,
    si: *mut VpxCodecStreamInfo,
) -> VpxCodecErr {
    let sz: u32;

    if (*si).sz as usize >= core::mem::size_of::<Vp8StreamInfo>() {
        sz = core::mem::size_of::<Vp8StreamInfo>() as u32;
    } else {
        sz = core::mem::size_of::<VpxCodecStreamInfo>() as u32;
    }

    ptr::copy_nonoverlapping(
        &(*ctx).si as *const Vp8StreamInfo as *const u8,
        si as *mut u8,
        sz as usize,
    );
    (*si).sz = sz;

    VPX_CODEC_OK
}

/// `update_error_state` — `vp8/vp8_dx_iface.c:199`.
///
/// The C source also propagated a formatted `err_detail` string out of
/// `VpxInternalErrorInfo`; that field is gone in the Rust port, so we
/// simply clear `err_detail` and return the error code.
unsafe fn update_error_state(
    ctx: *mut Vp8AlgPriv<'static>,
    error: *const VpxInternalErrorInfo,
) -> VpxCodecErr {
    let code: VpxCodecErr = (*error).error_code;

    if code != VPX_CODEC_OK {
        (*ctx).base.err_detail = ptr::null();
    }

    code
}

/// `yuvconfig2image` — `vp8/vp8_dx_iface.c:210`.
unsafe fn yuvconfig2image(
    img: *mut VpxImage,
    yv12: *const Yv12BufferConfig,
    user_priv: *mut c_void,
) {
    // vpx_img_wrap() doesn't allow specifying independent strides for
    // the Y, U, and V planes, nor other alignment adjustments that
    // might be representable by a YV12_BUFFER_CONFIG, so we just
    // initialize all the fields.
    (*img).fmt = VPX_IMG_FMT_I420;
    (*img).w = (*yv12).y_stride as u32;
    (*img).h = (((*yv12).y_height + 2 * VP8_BORDER_IN_PIXELS + 15) & !15) as u32;
    (*img).d_w = (*yv12).y_width as u32;
    (*img).r_w = (*yv12).y_width as u32;
    (*img).d_h = (*yv12).y_height as u32;
    (*img).r_h = (*yv12).y_height as u32;
    (*img).x_chroma_shift = 1;
    (*img).y_chroma_shift = 1;
    (*img).planes[VPX_PLANE_Y] = (*yv12).y_buffer;
    (*img).planes[VPX_PLANE_U] = (*yv12).u_buffer;
    (*img).planes[VPX_PLANE_V] = (*yv12).v_buffer;
    (*img).planes[VPX_PLANE_ALPHA] = ptr::null_mut();
    (*img).stride[VPX_PLANE_Y] = (*yv12).y_stride;
    (*img).stride[VPX_PLANE_U] = (*yv12).uv_stride;
    (*img).stride[VPX_PLANE_V] = (*yv12).uv_stride;
    (*img).stride[VPX_PLANE_ALPHA] = (*yv12).y_stride;
    (*img).bit_depth = 8;
    (*img).bps = 12;
    (*img).user_priv = user_priv;
    (*img).img_data = (*yv12).buffer_alloc;
    (*img).img_data_owner = 0;
    (*img).self_allocd = 0;
}

/// `update_fragments` — `vp8/vp8_dx_iface.c:239`.
unsafe fn update_fragments(
    ctx: *mut Vp8AlgPriv<'static>,
    data: *const u8,
    data_sz: u32,
    res: *mut VpxCodecErr,
) -> i32 {
    *res = VPX_CODEC_OK;

    if (*ctx).fragments.count == 0 {
        // New frame, reset fragment pointers and sizes
        ptr::write_bytes(
            (*ctx).fragments.ptrs.as_mut_ptr(),
            0,
            (*ctx).fragments.ptrs.len(),
        );
        ptr::write_bytes(
            (*ctx).fragments.sizes.as_mut_ptr(),
            0,
            (*ctx).fragments.sizes.len(),
        );
    }

    // Flush signal in fragment mode but no fragments were accumulated yet.
    // Nothing to decode; treat as a no-op.
    if (*ctx).fragments.enabled != 0
        && data.is_null()
        && data_sz == 0
        && (*ctx).fragments.count == 0
    {
        return 0;
    }

    if (*ctx).fragments.enabled != 0 && !(data.is_null() && data_sz == 0) {
        // Store a pointer to this fragment and return. We haven't
        // received the complete frame yet, so we will wait with decoding.
        if (*ctx).fragments.count as usize >= MAX_PARTITIONS {
            (*ctx).fragments.count = 0;
            *res = VPX_CODEC_INVALID_PARAM;
            return -1;
        }
        (*ctx).fragments.ptrs[(*ctx).fragments.count as usize] = data;
        (*ctx).fragments.sizes[(*ctx).fragments.count as usize] = data_sz;
        (*ctx).fragments.count += 1;
        return 0;
    }

    if (*ctx).fragments.enabled == 0 && data.is_null() && data_sz == 0 {
        return 0;
    }

    if (*ctx).fragments.enabled == 0 {
        (*ctx).fragments.ptrs[0] = data;
        (*ctx).fragments.sizes[0] = data_sz;
        (*ctx).fragments.count = 1;
    }

    1
}

/// `vp8_decode` — `vp8/vp8_dx_iface.c:285`. Vtable `dec.decode` slot.
pub unsafe fn vp8_decode(
    ctx: *mut Vp8AlgPriv<'static>,
    data: *const u8,
    data_sz: u32,
    user_priv: *mut c_void,
) -> VpxCodecErr {
    let mut res: VpxCodecErr;
    let mut resolution_change: u32 = 0;
    let w: u32;
    let h: u32;

    if (*ctx).fragments.enabled == 0 && data.is_null() && data_sz == 0 {
        return VPX_CODEC_OK;
    }

    // Update the input fragment data
    let mut res_local: VpxCodecErr = VPX_CODEC_OK;
    if update_fragments(ctx, data, data_sz, &mut res_local) <= 0 {
        return res_local;
    }
    res = res_local;

    // Determine the stream parameters. Note that we rely on peek_si to
    // validate that we have a buffer that does not wrap around the top
    // of the heap.
    w = (*ctx).si.w;
    h = (*ctx).si.h;

    res = vp8_peek_si_internal(
        (*ctx).fragments.ptrs[0],
        (*ctx).fragments.sizes[0],
        &mut (*ctx).si,
        (*ctx).decrypt_cb,
        (*ctx).decrypt_state,
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
        let pbi = (*ctx).yv12_frame_buffers.pbi[0];
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
        if (*ctx).postproc_cfg_set == 0
            && ((*ctx).base.init_flags & VPX_CODEC_USE_POSTPROC) != 0
        {
            (*ctx).postproc_cfg.post_proc_flag = VP8_DEBLOCK | VP8_DEMACROBLOCK | VP8_MFQE;
            (*ctx).postproc_cfg.deblocking_level = 4;
            (*ctx).postproc_cfg.noise_level = 0;
        }

        let rc = vp8_create_decoder_instances(&mut (*ctx).yv12_frame_buffers, &mut oxcf);
        res = if rc == VPX_CODEC_OK as i32 { VPX_CODEC_OK } else { VPX_CODEC_ERROR };
        if res == VPX_CODEC_OK {
            (*ctx).decoder_init = 1;
        } else {
            // on failure clear the cached resolution to ensure a full
            // reallocation is attempted on resync.
            (*ctx).si.w = 0;
            (*ctx).si.h = 0;
        }
    }

    // Set these even if already initialized.  The caller may have changed the
    // decrypt config between frames.
    if (*ctx).decoder_init != 0 {
        let pbi = (*ctx).yv12_frame_buffers.pbi[0];
        (*pbi).decrypt_cb = None; // FFI decryption callback wiring lives at the
                                  // FFI boundary; the Rust core does not consume
                                  // the C-style callback directly.
        let _ = (*ctx).decrypt_cb;
        (*pbi).decrypt_state = (*ctx).decrypt_state;
    }

    if res == VPX_CODEC_OK {
        let pbi = (*ctx).yv12_frame_buffers.pbi[0];
        let pc = &mut (*pbi).common as *mut crate::types::Vp8Common;
        if resolution_change != 0 {
            (*pc).width = (*ctx).si.w as i32;
            (*pc).height = (*ctx).si.h as i32;
            match vp8_decode_resolution_change(pbi, w, h) {
                Ok(()) => {}
                Err(_) => {
                    res = update_error_state(ctx, &(*pbi).common.error);
                    (*ctx).fragments.count = 0;
                    return res;
                }
            }

            // required to get past the first get_free_fb() call
            (*pbi).common.fb_idx_ref_cnt[0] = 0;
        }

        // update the pbi fragment data
        (*pbi).fragments = (*ctx).fragments;
        (*ctx).user_priv = user_priv;
        if let Err(_) = vp8dx_receive_compressed_data(pbi) {
            (*pc).yv12_fb[(*pc).lst_fb_idx as usize].corrupted = 1;
            if (*pc).fb_idx_ref_cnt[(*pc).new_fb_idx as usize] > 0 {
                (*pc).fb_idx_ref_cnt[(*pc).new_fb_idx as usize] -= 1;
            }
            res = update_error_state(ctx, &(*pbi).common.error);
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
    pbi: *mut Vp8dComp<'static>,
    w: u32,
    h: u32,
) -> VpxResult<()> {
    let pc = &mut (*pbi).common as *mut crate::types::Vp8Common;
    let xd = &mut (*pbi).mb as *mut crate::types::Macroblockd;

    if (*pc).width <= 0 {
        (*pc).width = w as i32;
        return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
    }

    if (*pc).height <= 0 {
        (*pc).height = h as i32;
        return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
    }

    if vp8_alloc_frame_buffers(pc, (*pc).width, (*pc).height) != 0 {
        return vpx_internal_error(&mut (*pc).error, VPX_CODEC_MEM_ERROR);
    }

    // xd->pre = pc->yv12_fb[pc->lst_fb_idx];
    ptr::copy_nonoverlapping(
        &(*pc).yv12_fb[(*pc).lst_fb_idx as usize] as *const Yv12BufferConfig,
        &mut (*xd).pre as *mut Yv12BufferConfig,
        1,
    );
    // xd->dst = pc->yv12_fb[pc->new_fb_idx];
    ptr::copy_nonoverlapping(
        &(*pc).yv12_fb[(*pc).new_fb_idx as usize] as *const Yv12BufferConfig,
        &mut (*xd).dst as *mut Yv12BufferConfig,
        1,
    );

    vp8_build_block_doffsets(&mut (*pbi).mb);

    // CONFIG_ERROR_CONCEALMENT / CONFIG_MULTITHREAD blocks
    // omitted in the minimal build.
    Ok(())
}

/// `vp8_get_frame` — `vp8/vp8_dx_iface.c:531`. Vtable `dec.get_frame` slot.
pub unsafe fn vp8_get_frame(
    ctx: *mut Vp8AlgPriv<'static>,
    iter: *mut VpxCodecIter,
) -> *mut VpxImage {
    let mut img: *mut VpxImage = ptr::null_mut();

    // iter acts as a flip flop, so an image is only returned on the first
    // call to get_frame.
    if (*iter).is_null() && !(*ctx).yv12_frame_buffers.pbi[0].is_null() {
        let mut sd: Yv12BufferConfig = core::mem::zeroed();
        let mut flags: Vp8PpFlags = Vp8PpFlags::default();
        vp8_zero(&mut flags);

        if ((*ctx).base.init_flags & VPX_CODEC_USE_POSTPROC) != 0 {
            flags.post_proc_flag = (*ctx).postproc_cfg.post_proc_flag;
            flags.deblocking_level = (*ctx).postproc_cfg.deblocking_level;
            flags.noise_level = (*ctx).postproc_cfg.noise_level;
        }

        if vp8dx_get_raw_frame((*ctx).yv12_frame_buffers.pbi[0], &mut sd, &mut flags) == 0 {
            yuvconfig2image(&mut (*ctx).img, &sd, (*ctx).user_priv);

            img = &mut (*ctx).img;
            *iter = img as *mut c_void;
        }
    }

    img
}

/// `image2yuvconfig` — `vp8/vp8_dx_iface.c:560`.
unsafe fn image2yuvconfig(img: *const VpxImage, yv12: *mut Yv12BufferConfig) -> VpxCodecErr {
    let y_w = (*img).d_w as i32;
    let y_h = (*img).d_h as i32;
    let uv_w = ((*img).d_w as i32 + 1) / 2;
    let uv_h = ((*img).d_h as i32 + 1) / 2;
    let res: VpxCodecErr = VPX_CODEC_OK;
    (*yv12).y_buffer = (*img).planes[VPX_PLANE_Y];
    (*yv12).u_buffer = (*img).planes[VPX_PLANE_U];
    (*yv12).v_buffer = (*img).planes[VPX_PLANE_V];

    (*yv12).y_crop_width = y_w;
    (*yv12).y_crop_height = y_h;
    (*yv12).y_width = y_w;
    (*yv12).y_height = y_h;
    (*yv12).uv_crop_width = uv_w;
    (*yv12).uv_crop_height = uv_h;
    (*yv12).uv_width = uv_w;
    (*yv12).uv_height = uv_h;

    (*yv12).y_stride = (*img).stride[VPX_PLANE_Y];
    (*yv12).uv_stride = (*img).stride[VPX_PLANE_U];

    (*yv12).border = ((*img).stride[VPX_PLANE_Y] - (*img).d_w as i32) / 2;
    res
}

/// `vp8_set_reference` — `vp8/vp8_dx_iface.c:587`. Control callback.
pub unsafe fn vp8_set_reference(
    ctx: *mut Vp8AlgPriv<'static>,
    mut args: VaList,
) -> VpxCodecErr {
    let data: *mut VpxRefFrame = args.arg_ptr::<VpxRefFrame>();

    if !data.is_null() {
        let frame: *mut VpxRefFrame = data;
        let mut sd: Yv12BufferConfig = core::mem::zeroed();

        image2yuvconfig(&(*frame).img, &mut sd);

        if (*ctx).yv12_frame_buffers.pbi[0].is_null() {
            return VPX_CODEC_CORRUPT_FRAME;
        }

        match vp8dx_set_reference(
            (*ctx).yv12_frame_buffers.pbi[0],
            (*frame).frame_type,
            &mut sd,
        ) {
            Ok(()) => VPX_CODEC_OK,
            Err(e) => e,
        }
    } else {
        VPX_CODEC_INVALID_PARAM
    }
}

/// `vp8_get_reference` — `vp8/vp8_dx_iface.c:606`. Control callback.
pub unsafe fn vp8_get_reference(
    ctx: *mut Vp8AlgPriv<'static>,
    mut args: VaList,
) -> VpxCodecErr {
    let data: *mut VpxRefFrame = args.arg_ptr::<VpxRefFrame>();

    if !data.is_null() {
        let frame: *mut VpxRefFrame = data;
        let mut sd: Yv12BufferConfig = core::mem::zeroed();

        image2yuvconfig(&(*frame).img, &mut sd);

        if (*ctx).yv12_frame_buffers.pbi[0].is_null() {
            return VPX_CODEC_CORRUPT_FRAME;
        }

        match vp8dx_get_reference(
            (*ctx).yv12_frame_buffers.pbi[0],
            (*frame).frame_type,
            &mut sd,
        ) {
            Ok(()) => VPX_CODEC_OK,
            Err(e) => e,
        }
    } else {
        VPX_CODEC_INVALID_PARAM
    }
}

/// `vp8_get_quantizer` — `vp8/vp8_dx_iface.c:625`. Control callback.
pub unsafe fn vp8_get_quantizer(
    ctx: *mut Vp8AlgPriv<'static>,
    mut args: VaList,
) -> VpxCodecErr {
    let arg: *mut i32 = args.arg_ptr::<i32>();
    let pbi = (*ctx).yv12_frame_buffers.pbi[0];
    if arg.is_null() {
        return VPX_CODEC_INVALID_PARAM;
    }
    if pbi.is_null() {
        return VPX_CODEC_CORRUPT_FRAME;
    }
    *arg = vp8dx_get_quantizer(pbi);
    VPX_CODEC_OK
}

/// `vp8_set_postproc` — `vp8/vp8_dx_iface.c:635`. Control callback.
pub unsafe fn vp8_set_postproc(
    _ctx: *mut Vp8AlgPriv<'static>,
    _args: VaList,
) -> VpxCodecErr {
    // CONFIG_POSTPROC is 0 in the minimal build.
    VPX_CODEC_INCAPABLE
}

/// `vp8_get_last_ref_updates` — `vp8/vp8_dx_iface.c:655`. Control callback.
pub unsafe fn vp8_get_last_ref_updates(
    ctx: *mut Vp8AlgPriv<'static>,
    mut args: VaList,
) -> VpxCodecErr {
    let update_info: *mut i32 = args.arg_ptr::<i32>();

    if !update_info.is_null() {
        let pbi = (*ctx).yv12_frame_buffers.pbi[0];
        if pbi.is_null() {
            return VPX_CODEC_CORRUPT_FRAME;
        }

        *update_info = (*pbi).common.refresh_alt_ref_frame * VP8_ALTR_FRAME
            + (*pbi).common.refresh_golden_frame * VP8_GOLD_FRAME
            + (*pbi).common.refresh_last_frame * VP8_LAST_FRAME;

        VPX_CODEC_OK
    } else {
        VPX_CODEC_INVALID_PARAM
    }
}

/// `vp8_get_last_ref_frame` — `vp8/vp8_dx_iface.c:673`. Control callback.
pub unsafe fn vp8_get_last_ref_frame(
    ctx: *mut Vp8AlgPriv<'static>,
    mut args: VaList,
) -> VpxCodecErr {
    let ref_info: *mut i32 = args.arg_ptr::<i32>();

    if !ref_info.is_null() {
        let pbi = (*ctx).yv12_frame_buffers.pbi[0];
        if !pbi.is_null() {
            let oci = &mut (*pbi).common as *mut crate::types::Vp8Common;
            *ref_info = (if vp8dx_references_buffer(oci, ALTREF_FRAME) != 0 {
                VP8_ALTR_FRAME
            } else {
                0
            }) | (if vp8dx_references_buffer(oci, GOLDEN_FRAME) != 0 {
                VP8_GOLD_FRAME
            } else {
                0
            }) | (if vp8dx_references_buffer(oci, LAST_FRAME) != 0 {
                VP8_LAST_FRAME
            } else {
                0
            });
            VPX_CODEC_OK
        } else {
            VPX_CODEC_CORRUPT_FRAME
        }
    } else {
        VPX_CODEC_INVALID_PARAM
    }
}

/// `vp8_get_frame_corrupted` — `vp8/vp8_dx_iface.c:694`. Control callback.
pub unsafe fn vp8_get_frame_corrupted(
    ctx: *mut Vp8AlgPriv<'static>,
    mut args: VaList,
) -> VpxCodecErr {
    let corrupted: *mut i32 = args.arg_ptr::<i32>();
    let pbi = (*ctx).yv12_frame_buffers.pbi[0];

    if !corrupted.is_null() && !pbi.is_null() {
        let frame = (*pbi).common.frame_to_show as *const Yv12BufferConfig;
        if frame.is_null() {
            return VPX_CODEC_ERROR;
        }
        *corrupted = (*frame).corrupted;
        VPX_CODEC_OK
    } else {
        VPX_CODEC_INVALID_PARAM
    }
}

/// `vp8_set_decryptor` — `vp8/vp8_dx_iface.c:709`. Control callback.
pub unsafe fn vp8_set_decryptor(
    ctx: *mut Vp8AlgPriv<'static>,
    mut args: VaList,
) -> VpxCodecErr {
    let init: *mut VpxDecryptInit = args.arg_ptr::<VpxDecryptInit>();

    if !init.is_null() {
        (*ctx).decrypt_cb = (*init).decrypt_cb;
        (*ctx).decrypt_state = (*init).decrypt_state;
    } else {
        (*ctx).decrypt_cb = None;
        (*ctx).decrypt_state = ptr::null_mut();
    }
    VPX_CODEC_OK
}

// ===========================================================================
// Vtable and control map — `vp8_dx_iface.c:723-765`
// ===========================================================================

/// `vp8_ctf_maps` — `vp8/vp8_dx_iface.c:723`.
pub static mut VP8_CTF_MAPS: [Vp8DxCtrlFnMap; 9] = [
    Vp8DxCtrlFnMap { ctrl_id: VP8_SET_REFERENCE, fn_: Some(vp8_set_reference) },
    Vp8DxCtrlFnMap { ctrl_id: VP8_COPY_REFERENCE, fn_: Some(vp8_get_reference) },
    Vp8DxCtrlFnMap { ctrl_id: VP8_SET_POSTPROC, fn_: Some(vp8_set_postproc) },
    Vp8DxCtrlFnMap { ctrl_id: VP8D_GET_LAST_REF_UPDATES, fn_: Some(vp8_get_last_ref_updates) },
    Vp8DxCtrlFnMap { ctrl_id: VP8D_GET_FRAME_CORRUPTED, fn_: Some(vp8_get_frame_corrupted) },
    Vp8DxCtrlFnMap { ctrl_id: VP8D_GET_LAST_REF_USED, fn_: Some(vp8_get_last_ref_frame) },
    Vp8DxCtrlFnMap { ctrl_id: VPXD_GET_LAST_QUANTIZER, fn_: Some(vp8_get_quantizer) },
    Vp8DxCtrlFnMap { ctrl_id: VPXD_SET_DECRYPTOR, fn_: Some(vp8_set_decryptor) },
    Vp8DxCtrlFnMap { ctrl_id: -1, fn_: None },
];

/// `vpx_codec_vp8_dx_algo` — `vp8/vp8_dx_iface.c:738-765`.
pub static mut VPX_CODEC_VP8_DX_ALGO: Vp8DxIface = Vp8DxIface {
    name: b"WebM Project VP8 Decoder\0".as_ptr(),
    abi_version: VPX_CODEC_INTERNAL_ABI_VERSION,
    caps: VPX_CODEC_CAP_DECODER
        | VP8_CAP_POSTPROC
        | VP8_CAP_ERROR_CONCEALMENT
        | VPX_CODEC_CAP_INPUT_FRAGMENTS,
    init: Some(vp8_init),
    destroy: Some(vp8_destroy),
    ctrl_maps: ptr::null_mut(), // populated by `vpx_codec_vp8_dx` lazily
    dec: Vp8DxIfaceDec {
        peek_si: Some(vp8_peek_si),
        get_si: Some(vp8_get_si),
        decode: Some(vp8_decode),
        frame_get: Some(vp8_get_frame),
        set_fb_fn: None,
    },
    enc: Vp8DxIfaceEnc {
        cfg_map_count: 0,
        cfg_maps: ptr::null_mut(),
        encode: ptr::null_mut(),
        get_cx_data: ptr::null_mut(),
        cfg_set: ptr::null_mut(),
        get_global_headers: ptr::null_mut(),
        get_preview_frame: ptr::null_mut(),
        mr_get_mem_loc: ptr::null_mut(),
        mr_free_mem_loc: ptr::null_mut(),
    },
};

/// `vpx_codec_vp8_dx` — `vp8/vp8_dx_iface.c:738` (via `CODEC_INTERFACE`).
///
/// Returns a pointer into the static `Vp8DxIface` block. Callers that
/// expect a canonical `vpx_api::VpxCodecIface*` should cast — the two
/// layouts overlap in the leading fields (`name`, `abi_version`,
/// `caps`) but the function-pointer slots differ.
pub unsafe fn vpx_codec_vp8_dx() -> *mut Vp8DxIface {
    if VPX_CODEC_VP8_DX_ALGO.ctrl_maps.is_null() {
        VPX_CODEC_VP8_DX_ALGO.ctrl_maps = VP8_CTF_MAPS.as_mut_ptr();
    }
    &raw mut VPX_CODEC_VP8_DX_ALGO
}
