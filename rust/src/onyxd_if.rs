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

use core::ffi::c_void;
use core::ptr;

use crate::types::{
    FragmentData, FrameBuffers, MbModeInfo, MbPredictionMode, ModeInfo, MvReferenceFrame, Vp8Common,
    Vp8dComp, Vp8dConfig, Vp8PpFlags, VpxResult, Yv12BufferConfig, MAX_FB_MT_DEC, NUM_YV12_BUFFERS,
};

// ===========================================================================
// Inline types and constants (kept here to avoid editing `types.rs`).
// ===========================================================================

/// `vpx_codec_err_t` — public libvpx error code (`vpx/vpx_codec.h`). Only
/// the two values produced by this file are spelled out; the rest are
/// passed through opaquely.
use crate::vpx_api::{VpxCodecErr, VPX_CODEC_ERROR, VPX_CODEC_OK};

/// `enum vpx_ref_frame_type` (`vpx/vp8.h`). Bitmask values used by the
/// `VP8_COPY_REFERENCE` / `VP8_SET_REFERENCE` control codes.
pub type VpxRefFrameType = i32;
pub const VP8_LAST_FRAME: VpxRefFrameType = 1;
pub const VP8_GOLD_FRAME: VpxRefFrameType = 2;
pub const VP8_ALTR_FRAME: VpxRefFrameType = 4;

/// `INTRA_FRAME`/`LAST_FRAME`/`GOLDEN_FRAME`/`ALTREF_FRAME` slot indices
/// into [`Vp8dComp::dec_fb_ref`] (`vp8/common/blockd.h`).
pub const INTRA_FRAME: usize = MvReferenceFrame::Intra as usize;
pub const LAST_FRAME: usize = MvReferenceFrame::Last as usize;
pub const GOLDEN_FRAME: usize = MvReferenceFrame::Golden as usize;
pub const ALTREF_FRAME: usize = MvReferenceFrame::Altref as usize;

// ===========================================================================
// Cross-translation-unit dependencies.
// ===========================================================================

use crate::decodeframe::{vp8_decode_frame, vp8cx_init_de_quantizer};
use crate::vpx_codec::vpx_internal_error;
use crate::vpx_mem::{vpx_free, vpx_memalign};

use crate::alloccommon::{vp8_create_common, vp8_remove_common};
use crate::mbpitch::vp8_setup_block_dptrs;
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
/// [`create_decompressor`]; the `volatile` guard is defensive — the
/// real serialization happens in `once()`.
unsafe fn initialize_dec() {
    static mut INIT_DONE: i32 = 0;

    if INIT_DONE == 0 {
        vpx_dsp_rtcd();
        vp8_init_intra_predictors();
        INIT_DONE = 1;
    }
}

/// `static void remove_decompressor(VP8D_COMP *)` — `vp8/decoder/onyxd_if.c:58`.
unsafe fn remove_decompressor(pbi: *mut Vp8dComp<'static>) {
    vp8_remove_common(&mut (*pbi).common as *mut Vp8Common);
    vpx_free(pbi as *mut c_void);
}

/// `static struct VP8D_COMP *create_decompressor(VP8D_CONFIG *)` —
/// `vp8/decoder/onyxd_if.c:66`. On `Err` the half-initialized instance
/// is torn down via [`remove_decompressor`].
unsafe fn create_decompressor(oxcf: *mut Vp8dConfig) -> *mut Vp8dComp<'static> {
    let pbi = vpx_memalign(32, core::mem::size_of::<Vp8dComp<'static>>())
        as *mut Vp8dComp<'static>;

    if pbi.is_null() {
        return ptr::null_mut();
    }

    ptr::write_bytes(pbi as *mut u8, 0, core::mem::size_of::<Vp8dComp<'static>>());

    match create_decompressor_inner(pbi, oxcf) {
        Ok(()) => pbi,
        Err(_) => {
            remove_decompressor(pbi);
            ptr::null_mut()
        }
    }
}

/// Body of [`create_decompressor`].
unsafe fn create_decompressor_inner(
    pbi: *mut Vp8dComp<'static>,
    oxcf: *mut Vp8dConfig,
) -> VpxResult<()> {
    vp8_create_common(&mut (*pbi).common as *mut Vp8Common);

    (*pbi).common.current_video_frame = 0;
    (*pbi).ready_for_new_data = 1;

    // vp8cx_init_de_quantizer() is first called here. Add check in
    // frame_init_dequantizer() to avoid unnecessary calling of
    // vp8cx_init_de_quantizer() for every frame.
    vp8cx_init_de_quantizer(pbi);

    vp8_loop_filter_init(&mut (*pbi).common as *mut Vp8Common);

    // CONFIG_ERROR_CONCEALMENT is disabled on this build.
    let _ = oxcf;
    (*pbi).ec_enabled = 0;

    // Error concealment is activated after a key frame has been decoded
    // without errors when error concealment is enabled.
    (*pbi).ec_active = 0;

    (*pbi).decoded_key_frame = 0;

    // Independent partitions is activated when a frame updates the token
    // probability table to have equal probabilities over the PREV_COEF
    // context.
    (*pbi).independent_partitions = 0;

    vp8_setup_block_dptrs(&mut (*pbi).mb as *mut crate::types::Macroblockd);

    once(initialize_dec);

    Ok(())
}

/// `static int get_free_fb(VP8_COMMON *)` — `vp8/decoder/onyxd_if.c:193`.
unsafe fn get_free_fb(cm: *mut Vp8Common) -> i32 {
    let mut i: i32 = 0;
    while i < NUM_YV12_BUFFERS as i32 {
        if (*cm).fb_idx_ref_cnt[i as usize] == 0 {
            break;
        }
        i += 1;
    }

    debug_assert!(i < NUM_YV12_BUFFERS as i32);
    (*cm).fb_idx_ref_cnt[i as usize] = 1;
    i
}

/// `static void ref_cnt_fb(int *buf, int *idx, int new_idx)` —
/// `vp8/decoder/onyxd_if.c:204`.
unsafe fn ref_cnt_fb(buf: *mut i32, idx: *mut i32, new_idx: i32) {
    if *buf.offset(*idx as isize) > 0 {
        *buf.offset(*idx as isize) -= 1;
    }

    *idx = new_idx;

    *buf.offset(new_idx as isize) += 1;
}

/// `static int swap_frame_buffers(VP8_COMMON *)` —
/// `vp8/decoder/onyxd_if.c:213`.
///
/// If any buffer copy / swapping is signalled it should be done here.
unsafe fn swap_frame_buffers(cm: *mut Vp8Common) -> i32 {
    let mut err: i32 = 0;

    // The alternate reference frame or golden frame can be updated using
    // the new, last, or golden/alt ref frame. If it is updated using the
    // newly decoded frame it is a refresh. An update using the last or
    // golden/alt ref frame is a copy.
    if (*cm).copy_buffer_to_arf != 0 {
        let mut new_fb: i32 = 0;

        if (*cm).copy_buffer_to_arf == 1 {
            new_fb = (*cm).lst_fb_idx;
        } else if (*cm).copy_buffer_to_arf == 2 {
            new_fb = (*cm).gld_fb_idx;
        } else {
            err = -1;
        }

        ref_cnt_fb(
            (*cm).fb_idx_ref_cnt.as_mut_ptr(),
            &mut (*cm).alt_fb_idx,
            new_fb,
        );
    }

    if (*cm).copy_buffer_to_gf != 0 {
        let mut new_fb: i32 = 0;

        if (*cm).copy_buffer_to_gf == 1 {
            new_fb = (*cm).lst_fb_idx;
        } else if (*cm).copy_buffer_to_gf == 2 {
            new_fb = (*cm).alt_fb_idx;
        } else {
            err = -1;
        }

        ref_cnt_fb(
            (*cm).fb_idx_ref_cnt.as_mut_ptr(),
            &mut (*cm).gld_fb_idx,
            new_fb,
        );
    }

    if (*cm).refresh_golden_frame != 0 {
        ref_cnt_fb(
            (*cm).fb_idx_ref_cnt.as_mut_ptr(),
            &mut (*cm).gld_fb_idx,
            (*cm).new_fb_idx,
        );
    }

    if (*cm).refresh_alt_ref_frame != 0 {
        ref_cnt_fb(
            (*cm).fb_idx_ref_cnt.as_mut_ptr(),
            &mut (*cm).alt_fb_idx,
            (*cm).new_fb_idx,
        );
    }

    if (*cm).refresh_last_frame != 0 {
        ref_cnt_fb(
            (*cm).fb_idx_ref_cnt.as_mut_ptr(),
            &mut (*cm).lst_fb_idx,
            (*cm).new_fb_idx,
        );

        (*cm).frame_to_show = &mut (*cm).yv12_fb[(*cm).lst_fb_idx as usize]
            as *mut Yv12BufferConfig;
    } else {
        (*cm).frame_to_show = &mut (*cm).yv12_fb[(*cm).new_fb_idx as usize]
            as *mut Yv12BufferConfig;
    }

    (*cm).fb_idx_ref_cnt[(*cm).new_fb_idx as usize] -= 1;

    err
}

/// `static int check_fragments_for_errors(VP8D_COMP *)` —
/// `vp8/decoder/onyxd_if.c:270`.
unsafe fn check_fragments_for_errors(pbi: *mut Vp8dComp<'static>) -> i32 {
    let fragments: *mut FragmentData = &mut (*pbi).fragments;
    if (*pbi).ec_active == 0
        && (*fragments).count <= 1
        && (*fragments).sizes[0] == 0
    {
        let cm: *mut Vp8Common = &mut (*pbi).common;

        // If error concealment is disabled we won't signal missing frames
        // to the decoder.
        if (*cm).fb_idx_ref_cnt[(*cm).lst_fb_idx as usize] > 1 {
            // The last reference shares buffer with another reference
            // buffer. Move it to its own buffer before setting it as
            // corrupt, otherwise we will make multiple buffers corrupt.
            let prev_idx = (*cm).lst_fb_idx;
            (*cm).fb_idx_ref_cnt[prev_idx as usize] -= 1;
            (*cm).lst_fb_idx = get_free_fb(cm);
            vp8_yv12_copy_frame(
                &(*cm).yv12_fb[prev_idx as usize] as *const Yv12BufferConfig,
                &mut (*cm).yv12_fb[(*cm).lst_fb_idx as usize] as *mut Yv12BufferConfig,
            );
        }
        // This is used to signal that we are missing frames. We do not
        // know if the missing frame(s) was supposed to update any of the
        // reference buffers, but we act conservative and mark only the
        // last buffer as corrupted.
        (*cm).yv12_fb[(*cm).lst_fb_idx as usize].corrupted = 1;

        // Signal that we have no frame to show.
        (*cm).show_frame = 0;

        // Nothing more to do.
        return 0;
    }

    1
}

// ===========================================================================
// Public functions
// ===========================================================================

/// `vp8dx_get_reference` — `vp8/decoder/onyxd_if.c:123`.
pub unsafe fn vp8dx_get_reference(
    pbi: *mut Vp8dComp<'static>,
    ref_frame_flag: VpxRefFrameType,
    sd: *mut Yv12BufferConfig,
) -> VpxResult<()> {
    let cm: *mut Vp8Common = &mut (*pbi).common;
    let ref_fb_idx: i32;

    if ref_frame_flag == VP8_LAST_FRAME {
        ref_fb_idx = (*cm).lst_fb_idx;
    } else if ref_frame_flag == VP8_GOLD_FRAME {
        ref_fb_idx = (*cm).gld_fb_idx;
    } else if ref_frame_flag == VP8_ALTR_FRAME {
        ref_fb_idx = (*cm).alt_fb_idx;
    } else {
        return vpx_internal_error(&mut (*pbi).common.error, VPX_CODEC_ERROR);
    }

    let slot: *mut Yv12BufferConfig =
        &mut (*cm).yv12_fb[ref_fb_idx as usize] as *mut Yv12BufferConfig;
    if (*slot).y_height != (*sd).y_height
        || (*slot).y_width != (*sd).y_width
        || (*slot).uv_height != (*sd).uv_height
        || (*slot).uv_width != (*sd).uv_width
    {
        return vpx_internal_error(&mut (*pbi).common.error, VPX_CODEC_ERROR);
    }
    vp8_yv12_copy_frame(slot, sd);
    Ok(())
}

/// `vp8dx_set_reference` — `vp8/decoder/onyxd_if.c:153`.
pub unsafe fn vp8dx_set_reference(
    pbi: *mut Vp8dComp<'static>,
    ref_frame_flag: VpxRefFrameType,
    sd: *mut Yv12BufferConfig,
) -> VpxResult<()> {
    let cm: *mut Vp8Common = &mut (*pbi).common;
    let ref_fb_ptr: *mut i32;
    let free_fb: i32;

    if ref_frame_flag == VP8_LAST_FRAME {
        ref_fb_ptr = &mut (*cm).lst_fb_idx;
    } else if ref_frame_flag == VP8_GOLD_FRAME {
        ref_fb_ptr = &mut (*cm).gld_fb_idx;
    } else if ref_frame_flag == VP8_ALTR_FRAME {
        ref_fb_ptr = &mut (*cm).alt_fb_idx;
    } else {
        return vpx_internal_error(&mut (*pbi).common.error, VPX_CODEC_ERROR);
    }

    let slot: *mut Yv12BufferConfig =
        &mut (*cm).yv12_fb[*ref_fb_ptr as usize] as *mut Yv12BufferConfig;
    if (*slot).y_height != (*sd).y_height
        || (*slot).y_width != (*sd).y_width
        || (*slot).uv_height != (*sd).uv_height
        || (*slot).uv_width != (*sd).uv_width
    {
        return vpx_internal_error(&mut (*pbi).common.error, VPX_CODEC_ERROR);
    }
    // Find an empty frame buffer.
    free_fb = get_free_fb(cm);
    // Decrease fb_idx_ref_cnt since it will be increased again in
    // ref_cnt_fb() below.
    (*cm).fb_idx_ref_cnt[free_fb as usize] -= 1;

    // Manage the reference counters and copy image.
    ref_cnt_fb((*cm).fb_idx_ref_cnt.as_mut_ptr(), ref_fb_ptr, free_fb);
    vp8_yv12_copy_frame(
        sd,
        &mut (*cm).yv12_fb[*ref_fb_ptr as usize] as *mut Yv12BufferConfig,
    );
    Ok(())
}

/// `vp8dx_receive_compressed_data` — `vp8/decoder/onyxd_if.c:305`.
pub unsafe fn vp8dx_receive_compressed_data(pbi: *mut Vp8dComp<'static>) -> VpxResult<()> {
    let cm: *mut Vp8Common = &mut (*pbi).common;

    (*pbi).common.error.error_code = VPX_CODEC_OK;

    let frag_status = check_fragments_for_errors(pbi);
    if frag_status <= 0 {
        // No fragments to decode (the C source signals this with
        // `return 0` / `return -1` *without* throwing). We mirror that
        // by reporting success — the caller checks `(*cm).show_frame`
        // and `error_code` separately.
        return Ok(());
    }

    (*cm).new_fb_idx = get_free_fb(cm);

    // setup reference frames for vp8_decode_frame
    (*pbi).dec_fb_ref[INTRA_FRAME] =
        &mut (*cm).yv12_fb[(*cm).new_fb_idx as usize] as *mut Yv12BufferConfig;
    (*pbi).dec_fb_ref[LAST_FRAME] =
        &mut (*cm).yv12_fb[(*cm).lst_fb_idx as usize] as *mut Yv12BufferConfig;
    (*pbi).dec_fb_ref[GOLDEN_FRAME] =
        &mut (*cm).yv12_fb[(*cm).gld_fb_idx as usize] as *mut Yv12BufferConfig;
    (*pbi).dec_fb_ref[ALTREF_FRAME] =
        &mut (*cm).yv12_fb[(*cm).alt_fb_idx as usize] as *mut Yv12BufferConfig;

    if let Err(e) = vp8_decode_frame(pbi) {
        // Drop the just-allocated new_fb refcount and propagate the
        // per-MB error_code up to the common error info.
        if (*cm).fb_idx_ref_cnt[(*cm).new_fb_idx as usize] > 0 {
            (*cm).fb_idx_ref_cnt[(*cm).new_fb_idx as usize] -= 1;
        }

        (*pbi).common.error.error_code = VPX_CODEC_ERROR;
        if (*pbi).mb.error_info.error_code != VPX_CODEC_OK {
            (*pbi).common.error.error_code = (*pbi).mb.error_info.error_code;
        }
        // goto decode_exit;
        vpx_clear_system_state();
        return Err(e);
    }

    if swap_frame_buffers(cm) != 0 {
        (*pbi).common.error.error_code = VPX_CODEC_ERROR;
        // goto decode_exit;
        vpx_clear_system_state();
        return Err(VPX_CODEC_ERROR);
    }

    vpx_clear_system_state();

    if (*cm).show_frame != 0 {
        (*cm).current_video_frame += 1;
        (*cm).show_frame_mi = (*cm).mi;
    }

    // CONFIG_ERROR_CONCEALMENT block omitted on this build.

    (*pbi).ready_for_new_data = 0;

    // decode_exit:
    vpx_clear_system_state();
    Ok(())
}

/// `vp8dx_get_raw_frame` — `vp8/decoder/onyxd_if.c:376`.
#[no_mangle]
pub unsafe extern "C" fn vp8dx_get_raw_frame(
    pbi: *mut Vp8dComp<'static>,
    sd: *mut Yv12BufferConfig,
    flags: *mut Vp8PpFlags,
) -> i32 {
    let mut ret: i32 = -1;

    if (*pbi).ready_for_new_data == 1 {
        return ret;
    }

    // ie no raw frame to show!!!
    if (*pbi).common.show_frame == 0 {
        return ret;
    }

    (*pbi).ready_for_new_data = 1;

    // CONFIG_POSTPROC is disabled — cast flags to void as the C source does.
    let _ = flags;

    if !(*pbi).common.frame_to_show.is_null() {
        // Shallow descriptor copy — *sd shares plane buffers with the
        // decoder's frame_to_show until the next call to
        // vp8dx_receive_compressed_data.
        ptr::copy_nonoverlapping((*pbi).common.frame_to_show, sd, 1);
        (*sd).y_width = (*pbi).common.width;
        (*sd).y_height = (*pbi).common.height;
        (*sd).uv_height = (*pbi).common.height / 2;
        ret = 0;
    } else {
        ret = -1;
    }

    vpx_clear_system_state();
    ret
}

/// `vp8dx_references_buffer` — `vp8/decoder/onyxd_if.c:411`.
///
/// Linear scan over the mode-info grid (`oci->mi`) checking whether any
/// macroblock referenced `ref_frame`. The trailing `mi = mi.add(1)` past
/// each row skips the sentinel column at `mode_info_stride - 1`.
#[no_mangle]
pub unsafe extern "C" fn vp8dx_references_buffer(
    oci: *mut Vp8Common,
    ref_frame: i32,
) -> i32 {
    let mut mi: *const ModeInfo = (*oci).mi as *const ModeInfo;

    let mut mb_row: i32 = 0;
    while mb_row < (*oci).mb_rows {
        let mut mb_col: i32 = 0;
        while mb_col < (*oci).mb_cols {
            let mbmi: *const MbModeInfo = &(*mi).mbmi;
            if (*mbmi).ref_frame as i32 == ref_frame {
                return 1;
            }
            mb_col += 1;
            mi = mi.add(1);
        }
        mi = mi.add(1);
        mb_row += 1;
    }
    let _ = MbPredictionMode::DcPred; // keep MbPredictionMode use exercised
    0
}

/// `vp8_create_decoder_instances` — `vp8/decoder/onyxd_if.c:424`.
#[no_mangle]
pub unsafe extern "C" fn vp8_create_decoder_instances(
    fb: *mut FrameBuffers<'static>,
    oxcf: *mut Vp8dConfig,
) -> i32 {
    // decoder instance for single thread mode
    (*fb).pbi[0] = create_decompressor(oxcf);
    if (*fb).pbi[0].is_null() {
        return VPX_CODEC_ERROR as i32;
    }

    // CONFIG_MULTITHREAD branch omitted on this build.
    VPX_CODEC_OK as i32
}

/// `vp8_remove_decoder_instances` — `vp8/decoder/onyxd_if.c:446`.
#[no_mangle]
pub unsafe extern "C" fn vp8_remove_decoder_instances(fb: *mut FrameBuffers<'static>) -> i32 {
    let pbi: *mut Vp8dComp<'static> = (*fb).pbi[0];

    if pbi.is_null() {
        return VPX_CODEC_ERROR as i32;
    }

    // decoder instance for single thread mode
    remove_decompressor(pbi);
    (*fb).pbi[0] = ptr::null_mut();
    VPX_CODEC_OK as i32
}

/// `vp8dx_get_quantizer` — `vp8/decoder/onyxd_if.c:460`.
#[no_mangle]
pub unsafe extern "C" fn vp8dx_get_quantizer(pbi: *const Vp8dComp<'static>) -> i32 {
    (*pbi).common.base_qindex
}
