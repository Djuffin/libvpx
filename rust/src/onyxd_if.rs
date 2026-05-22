//! `vp8/decoder/onyxd_if.c` — VP8 decoder instance lifecycle and frame driver.
//!
//! Literal Rust translation of `vp8/decoder/onyxd_if.c`. Function names,
//! control flow, and pointer arithmetic mirror the C source verbatim.
//! All bodies are `unsafe` because the work is built on raw pointers
//! and FFI-shaped state owned by [`Vp8dComp`].
//!
//! Build assumptions (the minimal `vp8_only` configuration documented
//! in `documentation/vp8_files.md`):
//!   - `CONFIG_POSTPROC = 0`
//!   - `CONFIG_ERROR_CONCEALMENT = 0`
//!   - `CONFIG_MULTITHREAD = 0`
//!
//! Anything gated by one of those switches in the C source is omitted,
//! mirroring the pruning already applied to [`Vp8dComp`] / [`Vp8Common`]
//! in `rust/src/types.rs`.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]


use crate::types::{
    FrameBuffers, FrameView, NUM_YV12_BUFFERS, Vp8Common, Vp8PpFlags,
    Vp8dComp, Vp8dConfig, VpxResult, Yv12BufferConfig,
};

// ===========================================================================
// Inline types and constants (kept here to avoid editing `types.rs`).
// ===========================================================================

/// `vpx_codec_err_t` — public libvpx error code (`vpx/vpx_codec.h`). Only
/// the two values produced by this file are spelled out; the rest are
/// passed through opaquely.
use crate::vpx_api::{VPX_CODEC_ERROR, VPX_CODEC_OK};

use crate::types::{
    ALTREF_FRAME, GOLDEN_FRAME, INTRA_FRAME, LAST_FRAME, VP8_ALTR_FRAME, VP8_GOLD_FRAME,
    VP8_LAST_FRAME,
};

/// `enum vpx_ref_frame_type` (`vpx/vp8.h`). Bitmask values used by the
/// `VP8_COPY_REFERENCE` / `VP8_SET_REFERENCE` control codes.
pub type VpxRefFrameType = i32;

// ===========================================================================
// Cross-translation-unit dependencies.
// ===========================================================================

use crate::decodeframe::{vp8_decode_frame, vp8cx_init_de_quantizer};
use crate::vpx_codec::vpx_internal_error;

use crate::alloccommon::{vp8_create_common, vp8_remove_common};
use crate::reconintra::vp8_init_intra_predictors;
use crate::vp8_loopfilter::vp8_loop_filter_init;
use crate::vpx_dsp_rtcd::vpx_dsp_rtcd;
use crate::vpx_ports::{once, vpx_clear_system_state};
use crate::vpx_scale_rtcd::vp8_yv12_copy_frame;

// ===========================================================================
// Static helpers
// ===========================================================================

/// `static void initialize_dec(void)` — `vp8/decoder/onyxd_if.c:48`.
///
/// Process-wide one-shot init. Invoked through `once()` from
/// [`create_decompressor`]. The C source has a `volatile int init_done`
/// guard that is defensive — the real serialization happens in
/// `once()`, so we drop the dead inner check. Remains `unsafe fn` to
/// satisfy `once()`'s `unsafe fn` signature and because
/// `vp8_init_intra_predictors` is itself `unsafe`.
unsafe fn initialize_dec() {
    vpx_dsp_rtcd();
    vp8_init_intra_predictors();
}

/// `static void remove_decompressor(VP8D_COMP *)` — `vp8/decoder/onyxd_if.c:58`.
/// `static struct VP8D_COMP *create_decompressor(VP8D_CONFIG *)` —
/// `vp8/decoder/onyxd_if.c:66`. Returns `None` if initialization fails;
/// the half-initialized instance is torn down before return.
fn create_decompressor(oxcf: &Vp8dConfig) -> Option<Box<Vp8dComp<'static>>> {
    // Allocate the outer shell zero-initialised on the heap. The C
    // source uses `vpx_memalign(32, sizeof(VP8D_COMP)) + memset(0, ...)`;
    // `Box::new_zeroed` is the same byte pattern. The function-pointer
    // fields on `Macroblockd` (subpixel_predict*) are non-nullable and
    // thus zero-init is technically UB until they are written in
    // `init_frame`; this matches the literal-transliteration policy
    // the rest of the port follows.
    //
    // SAFETY of `assume_init`: every field of `Vp8dComp` is either
    // `Copy`/POD or `Option<…>` whose all-zero bit pattern is the
    // `None` discriminant — except for the non-nullable subpixel-
    // predict function pointers on `Macroblockd`, which are written
    // before first use by `init_frame`. We mirror the C source's
    // policy here.
    let mut pbi: Box<Vp8dComp<'static>> =
        unsafe { Box::<Vp8dComp<'static>>::new_zeroed().assume_init() };

    match create_decompressor_inner(&mut pbi, oxcf) {
        Ok(()) => Some(pbi),
        Err(_) => {
            vp8_remove_common(&mut pbi.common);
            None
        }
    }
}

/// Body of [`create_decompressor`].
fn create_decompressor_inner(
    pbi: &mut Vp8dComp<'static>,
    oxcf: &Vp8dConfig,
) -> VpxResult<()> {
    vp8_create_common(&mut pbi.common);

    pbi.common.current_video_frame = 0;
    pbi.ready_for_new_data = 1;

    // vp8cx_init_de_quantizer() is first called here. Add check in
    // frame_init_dequantizer() to avoid unnecessary calling of
    // vp8cx_init_de_quantizer() for every frame.
    vp8cx_init_de_quantizer(&mut pbi.common);

    vp8_loop_filter_init(&mut pbi.common);

    // CONFIG_ERROR_CONCEALMENT is disabled on this build.
    let _ = oxcf;
    pbi.ec_enabled = 0;

    // Error concealment is activated after a key frame has been decoded
    // without errors when error concealment is enabled.
    pbi.ec_active = 0;

    pbi.decoded_key_frame = 0;

    // Independent partitions is activated when a frame updates the token
    // probability table to have equal probabilities over the PREV_COEF
    // context.
    pbi.independent_partitions = 0;

    // SAFETY: one-time RTCD dispatch-table init (still an `unsafe fn`).
    unsafe { once(initialize_dec); }

    Ok(())
}

/// `static int get_free_fb(VP8_COMMON *)` — `vp8/decoder/onyxd_if.c:193`.
fn get_free_fb(cm: &mut Vp8Common) -> i32 {
    let i = (0..NUM_YV12_BUFFERS as i32)
        .find(|&i| cm.fb_idx_ref_cnt[i as usize] == 0)
        .unwrap_or(NUM_YV12_BUFFERS as i32);

    debug_assert!(i < NUM_YV12_BUFFERS as i32);
    cm.fb_idx_ref_cnt[i as usize] = 1;
    i
}

/// `static void ref_cnt_fb(int *buf, int *idx, int new_idx)` —
/// `vp8/decoder/onyxd_if.c:204`.
fn ref_cnt_fb(buf: &mut [i32; NUM_YV12_BUFFERS], idx: &mut i32, new_idx: i32) {
    if buf[*idx as usize] > 0 {
        buf[*idx as usize] -= 1;
    }

    *idx = new_idx;

    buf[new_idx as usize] += 1;
}

/// `static int swap_frame_buffers(VP8_COMMON *)` —
/// `vp8/decoder/onyxd_if.c:213`.
///
/// If any buffer copy / swapping is signalled it should be done here.
fn swap_frame_buffers(cm: &mut Vp8Common) -> i32 {
    let mut err: i32 = 0;

    // The alternate reference frame or golden frame can be updated using
    // the new, last, or golden/alt ref frame. If it is updated using the
    // newly decoded frame it is a refresh. An update using the last or
    // golden/alt ref frame is a copy.
    if cm.copy_buffer_to_arf != 0 {
        let mut new_fb: i32 = 0;

        if cm.copy_buffer_to_arf == 1 {
            new_fb = cm.lst_fb_idx;
        } else if cm.copy_buffer_to_arf == 2 {
            new_fb = cm.gld_fb_idx;
        } else {
            err = -1;
        }

        ref_cnt_fb(&mut cm.fb_idx_ref_cnt, &mut cm.alt_fb_idx, new_fb);
    }

    if cm.copy_buffer_to_gf != 0 {
        let mut new_fb: i32 = 0;

        if cm.copy_buffer_to_gf == 1 {
            new_fb = cm.lst_fb_idx;
        } else if cm.copy_buffer_to_gf == 2 {
            new_fb = cm.alt_fb_idx;
        } else {
            err = -1;
        }

        ref_cnt_fb(&mut cm.fb_idx_ref_cnt, &mut cm.gld_fb_idx, new_fb);
    }

    if cm.refresh_golden_frame != 0 {
        ref_cnt_fb(&mut cm.fb_idx_ref_cnt, &mut cm.gld_fb_idx, cm.new_fb_idx);
    }

    if cm.refresh_alt_ref_frame != 0 {
        ref_cnt_fb(&mut cm.fb_idx_ref_cnt, &mut cm.alt_fb_idx, cm.new_fb_idx);
    }

    if cm.refresh_last_frame != 0 {
        ref_cnt_fb(&mut cm.fb_idx_ref_cnt, &mut cm.lst_fb_idx, cm.new_fb_idx);

        cm.frame_to_show_idx = cm.lst_fb_idx;
    } else {
        cm.frame_to_show_idx = cm.new_fb_idx;
    }

    cm.fb_idx_ref_cnt[cm.new_fb_idx as usize] -= 1;

    err
}

/// `static int check_fragments_for_errors(VP8D_COMP *)` —
/// `vp8/decoder/onyxd_if.c:270`.
fn check_fragments_for_errors(pbi: &mut Vp8dComp<'static>) -> i32 {
    if pbi.ec_active != 0 || pbi.fragments.count > 1 || pbi.fragments.sizes[0] != 0 {
        return 1;
    }

    let cm = &mut pbi.common;

    // If error concealment is disabled we won't signal missing frames
    // to the decoder.
    if cm.fb_idx_ref_cnt[cm.lst_fb_idx as usize] > 1 {
        // The last reference shares buffer with another reference
        // buffer. Move it to its own buffer before setting it as
        // corrupt, otherwise we will make multiple buffers corrupt.
        let prev_idx = cm.lst_fb_idx;
        cm.fb_idx_ref_cnt[prev_idx as usize] -= 1;
        cm.lst_fb_idx = get_free_fb(cm);
        // SAFETY: prev_idx and lst_fb_idx are distinct (get_free_fb
        // returned an unallocated slot, and prev_idx is still
        // refcounted), so the two raw projections don't alias.
        let src: *const Yv12BufferConfig = &cm.yv12_fb[prev_idx as usize];
        let dst: *mut Yv12BufferConfig = &mut cm.yv12_fb[cm.lst_fb_idx as usize];
        unsafe { vp8_yv12_copy_frame(src, dst); }
    }
    // Mark only the last buffer as corrupted — we don't know which
    // references the missing frame(s) would have updated.
    cm.yv12_fb[cm.lst_fb_idx as usize].corrupted = 1;

    // Signal that we have no frame to show.
    cm.show_frame = 0;

    0
}

// ===========================================================================
// Public functions
// ===========================================================================

/// `vp8dx_get_reference` — `vp8/decoder/onyxd_if.c:123`.
pub unsafe fn vp8dx_get_reference(
    pbi: &mut Vp8dComp<'static>,
    ref_frame_flag: VpxRefFrameType,
    sd: *mut Yv12BufferConfig,
) -> VpxResult<()> {
    let ref_fb_idx: i32 = if ref_frame_flag == VP8_LAST_FRAME {
        pbi.common.lst_fb_idx
    } else if ref_frame_flag == VP8_GOLD_FRAME {
        pbi.common.gld_fb_idx
    } else if ref_frame_flag == VP8_ALTR_FRAME {
        pbi.common.alt_fb_idx
    } else {
        return vpx_internal_error(&mut pbi.common.error, VPX_CODEC_ERROR);
    };

    let slot: *mut Yv12BufferConfig =
        &mut pbi.common.yv12_fb[ref_fb_idx as usize] as *mut Yv12BufferConfig;
    if (*slot).y_height != (*sd).y_height
        || (*slot).y_width != (*sd).y_width
        || (*slot).uv_height != (*sd).uv_height
        || (*slot).uv_width != (*sd).uv_width
    {
        return vpx_internal_error(&mut pbi.common.error, VPX_CODEC_ERROR);
    }
    vp8_yv12_copy_frame(slot, sd);
    Ok(())
}

/// `vp8dx_set_reference` — `vp8/decoder/onyxd_if.c:153`.
pub unsafe fn vp8dx_set_reference(
    pbi: &mut Vp8dComp<'static>,
    ref_frame_flag: VpxRefFrameType,
    sd: *mut Yv12BufferConfig,
) -> VpxResult<()> {
    // Current index value for the targeted slot (copied out so the
    // dim-check + the later disjoint-field borrows don't conflict).
    let cur_idx: i32 = if ref_frame_flag == VP8_LAST_FRAME {
        pbi.common.lst_fb_idx
    } else if ref_frame_flag == VP8_GOLD_FRAME {
        pbi.common.gld_fb_idx
    } else if ref_frame_flag == VP8_ALTR_FRAME {
        pbi.common.alt_fb_idx
    } else {
        return vpx_internal_error(&mut pbi.common.error, VPX_CODEC_ERROR);
    };

    let slot: *const Yv12BufferConfig =
        &pbi.common.yv12_fb[cur_idx as usize] as *const Yv12BufferConfig;
    if (*slot).y_height != (*sd).y_height
        || (*slot).y_width != (*sd).y_width
        || (*slot).uv_height != (*sd).uv_height
        || (*slot).uv_width != (*sd).uv_width
    {
        return vpx_internal_error(&mut pbi.common.error, VPX_CODEC_ERROR);
    }
    // Find an empty frame buffer.
    let free_fb = get_free_fb(&mut pbi.common);
    // Decrease fb_idx_ref_cnt since it will be increased again in
    // ref_cnt_fb() below.
    pbi.common.fb_idx_ref_cnt[free_fb as usize] -= 1;

    // Manage the reference counters. `fb_idx_ref_cnt` and the targeted
    // index field are disjoint fields of `common`, so they coexist as
    // `&mut`. ref_cnt_fb sets `*idx = free_fb`, so the copy destination
    // below is `yv12_fb[free_fb]`.
    {
        let ref_fb_ptr: &mut i32 = if ref_frame_flag == VP8_LAST_FRAME {
            &mut pbi.common.lst_fb_idx
        } else if ref_frame_flag == VP8_GOLD_FRAME {
            &mut pbi.common.gld_fb_idx
        } else {
            &mut pbi.common.alt_fb_idx
        };
        ref_cnt_fb(&mut pbi.common.fb_idx_ref_cnt, ref_fb_ptr, free_fb);
    }
    vp8_yv12_copy_frame(
        sd,
        &mut pbi.common.yv12_fb[free_fb as usize] as *mut Yv12BufferConfig,
    );
    Ok(())
}

/// `vp8dx_receive_compressed_data` — `vp8/decoder/onyxd_if.c:305`.
pub fn vp8dx_receive_compressed_data(pbi: &mut Vp8dComp<'static>) -> VpxResult<()> {
    pbi.common.error.error_code = VPX_CODEC_OK;

    let frag_status = check_fragments_for_errors(pbi);
    if frag_status <= 0 {
        // No fragments to decode (C signals this with `return 0` /
        // `return -1` without throwing). Mirror by reporting success —
        // callers check `common.show_frame` and `error_code` separately.
        return Ok(());
    }

    pbi.common.new_fb_idx = get_free_fb(&mut pbi.common);

    // setup reference frames for vp8_decode_frame
    pbi.dec_fb_ref_idx[INTRA_FRAME] = pbi.common.new_fb_idx;
    pbi.dec_fb_ref_idx[LAST_FRAME] = pbi.common.lst_fb_idx;
    pbi.dec_fb_ref_idx[GOLDEN_FRAME] = pbi.common.gld_fb_idx;
    pbi.dec_fb_ref_idx[ALTREF_FRAME] = pbi.common.alt_fb_idx;

    if let Err(e) = vp8_decode_frame(pbi) {
        // Drop the just-allocated new_fb refcount.
        let new_idx = pbi.common.new_fb_idx as usize;
        if pbi.common.fb_idx_ref_cnt[new_idx] > 0 {
            pbi.common.fb_idx_ref_cnt[new_idx] -= 1;
        }
        // C source has a post-longjmp error_code copyback block that is
        // unreachable in C (longjmp unwinds past it). `vpx_internal_error`
        // already wrote the canonical code into `common.error.error_code`
        // before returning Err — we just propagate.
        pbi.common.error.error_code = e;
        vpx_clear_system_state();
        return Err(e);
    }

    if swap_frame_buffers(&mut pbi.common) != 0 {
        pbi.common.error.error_code = VPX_CODEC_ERROR;
        vpx_clear_system_state();
        return Err(VPX_CODEC_ERROR);
    }

    vpx_clear_system_state();

    if pbi.common.show_frame != 0 {
        pbi.common.current_video_frame += 1;
    }

    pbi.ready_for_new_data = 0;
    vpx_clear_system_state();
    Ok(())
}

/// `vp8dx_get_raw_frame` — `vp8/decoder/onyxd_if.c:376`.

pub fn vp8dx_get_raw_frame(
    pbi: &mut Vp8dComp<'static>,
    flags: *mut Vp8PpFlags,
) -> Option<FrameView> {
    if pbi.ready_for_new_data == 1 {
        return None;
    }

    // ie no raw frame to show!!!
    if pbi.common.show_frame == 0 {
        return None;
    }

    pbi.ready_for_new_data = 1;

    // CONFIG_POSTPROC is disabled — cast flags to void as the C source does.
    let _ = flags;

    // A non-owning view that shares plane buffers with the decoder's
    // frame_to_show until the next call to vp8dx_receive_compressed_data.
    // The visible dimensions are the cropped ones, not the slot's
    // 16-aligned width/height.
    let view = if pbi.common.frame_to_show_idx >= 0 {
        let slot = &pbi.common.yv12_fb[pbi.common.frame_to_show_idx as usize];
        Some(FrameView {
            y_buffer: slot.y_buffer(),
            u_buffer: slot.u_buffer(),
            v_buffer: slot.v_buffer(),
            // Slab base, surfaced as the output image's `img_data`.
            buffer_alloc: slot
                .owning_buffer
                .as_ref()
                .map_or(core::ptr::null_mut(), |s| s.as_ptr() as *mut u8),
            y_stride: slot.y_stride,
            uv_stride: slot.uv_stride,
            display_width: pbi.common.width,
            display_height: pbi.common.height,
        })
    } else {
        None
    };

    vpx_clear_system_state();
    view
}

/// `vp8dx_references_buffer` — `vp8/decoder/onyxd_if.c:411`.
///
/// Linear scan over the mode-info grid checking whether any macroblock
/// referenced `ref_frame`.
pub fn vp8dx_references_buffer(oci: &Vp8Common, ref_frame: i32) -> i32 {
    for row in 0..oci.mb_rows {
        for mi in oci.mi_row(row) {
            if mi.mbmi.ref_frame as i32 == ref_frame {
                return 1;
            }
        }
    }
    0
}

/// `vp8_create_decoder_instances` — `vp8/decoder/onyxd_if.c:424`.
///
/// `create_decompressor` is a safe wrapper that encapsulates the
/// `assume_init` / raw-pointer dance internally; the boundary here is
/// safe because the only caller side-effect on success is
/// `fb.pbi = Some(box)`, which is a plain field write.
pub fn vp8_create_decoder_instances(
    fb: &mut FrameBuffers<'static>,
    oxcf: &Vp8dConfig,
) -> i32 {
    // decoder instance for single thread mode
    match create_decompressor(oxcf) {
        Some(b) => {
            fb.pbi = Some(b);
            // CONFIG_MULTITHREAD branch omitted on this build.
            VPX_CODEC_OK as i32
        }
        None => VPX_CODEC_ERROR as i32,
    }
}

/// `vp8_remove_decoder_instances` — `vp8/decoder/onyxd_if.c:446`.
///
/// `take()`-ing the `Box` makes a double-call safe by construction (the
/// second call sees `None`).
pub fn vp8_remove_decoder_instances(fb: &mut FrameBuffers<'static>) -> i32 {
    match fb.pbi.take() {
        Some(mut b) => {
            vp8_remove_common(&mut b.common);
            // Outer shell freed by Box drop here.
            VPX_CODEC_OK as i32
        }
        None => VPX_CODEC_ERROR as i32,
    }
}

/// `vp8dx_get_quantizer` — `vp8/decoder/onyxd_if.c:460`.

pub fn vp8dx_get_quantizer(pbi: &Vp8dComp<'static>) -> i32 {
    pbi.common.base_qindex
}
