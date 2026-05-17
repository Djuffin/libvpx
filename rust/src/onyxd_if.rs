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
    FragmentData, MbModeInfo, MbPredictionMode, ModeInfo, MvReferenceFrame, Vp8Common,
    Vp8dComp, Vp8dConfig, Yv12BufferConfig, NUM_YV12_BUFFERS,
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

/// `vp8_ppflags_t` (`vp8/common/ppflags.h`) — opaque on the
/// `--disable-postproc` build path; the parameter is only cast to
/// `void` and never dereferenced by code below.
#[repr(C)]
pub struct Vp8PpFlags {
    _opaque: [u8; 0],
}

/// `struct frame_buffers` (`vp8/decoder/onyxd_int.h:50-57`). In the
/// single-threaded build only slot 0 is ever populated.
pub const MAX_FB_MT_DEC: usize = 32;

#[repr(C)]
pub struct FrameBuffers<'a> {
    pub pbi: [*mut Vp8dComp<'a>; MAX_FB_MT_DEC],
}

// ===========================================================================
// `extern "Rust"` cross-translation-unit dependencies.
//
// These all live in sibling files (`alloccommon.rs`, `loopfilter.rs`,
// `decodeframe.rs`, `mbpitch.rs`, `reconintra.rs`, `yv12extend.rs`,
// `quant_common.rs`, error-info shims) that have not been translated
// yet. They are declared `extern "Rust"` so the compiler is happy until
// those sibling modules land.
// ===========================================================================

use crate::vpx_codec::vpx_internal_error;
use crate::vpx_mem::{vpx_free, vpx_memalign};

extern "Rust" {
    fn vpx_dsp_rtcd();
    fn vpx_clear_system_state();

    /// One-shot initializer primitive (`vpx_ports/vpx_once.h`).
    fn once(func: unsafe extern "Rust" fn());

    fn vp8_create_common(cm: *mut Vp8Common);
    fn vp8_remove_common(cm: *mut Vp8Common);

    fn vp8_init_intra_predictors();
    fn vp8_init_loop_filter(cm: *mut Vp8Common);
    fn vp8_loop_filter_init(cm: *mut Vp8Common);

    fn vp8cx_init_de_quantizer(pbi: *mut Vp8dComp<'static>);
    fn vp8_setup_block_dptrs(mb: *mut crate::types::Macroblockd);
    fn vp8_decode_frame(pbi: *mut Vp8dComp<'static>) -> i32;

    fn vp8_yv12_copy_frame(src: *const Yv12BufferConfig, dst: *mut Yv12BufferConfig);

    /// `setjmp` shim against the `jmp_buf` embedded inside
    /// [`crate::types::VpxInternalErrorInfo`]. Returns 0 on the
    /// initial call, non-zero when reached via `longjmp`.
    fn vpx_setjmp(jmp: *mut u8) -> i32;
}

// ===========================================================================
// Static helpers
// ===========================================================================

/// `static void initialize_dec(void)` — `vp8/decoder/onyxd_if.c:48`.
///
/// Process-wide one-shot init. Invoked through `once()` from
/// [`create_decompressor`]; the `volatile` guard is defensive — the
/// real serialization happens in `once()`.
unsafe extern "Rust" fn initialize_dec() {
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
/// `vp8/decoder/onyxd_if.c:66`.
unsafe fn create_decompressor(oxcf: *mut Vp8dConfig) -> *mut Vp8dComp<'static> {
    let pbi = vpx_memalign(32, core::mem::size_of::<Vp8dComp<'static>>())
        as *mut Vp8dComp<'static>;

    if pbi.is_null() {
        return ptr::null_mut();
    }

    ptr::write_bytes(pbi as *mut u8, 0, core::mem::size_of::<Vp8dComp<'static>>());

    if vpx_setjmp(&mut (*pbi).common.error.jmp[0] as *mut u8) != 0 {
        (*pbi).common.error.setjmp = 0;
        remove_decompressor(pbi);
        return ptr::null_mut();
    }

    (*pbi).common.error.setjmp = 1;

    vp8_create_common(&mut (*pbi).common as *mut Vp8Common);

    (*pbi).common.current_video_frame = 0;
    (*pbi).ready_for_new_data = 1;

    // vp8cx_init_de_quantizer() is first called here. Add check in
    // frame_init_dequantizer() to avoid unnecessary calling of
    // vp8cx_init_de_quantizer() for every frame.
    vp8cx_init_de_quantizer(pbi);

    vp8_loop_filter_init(&mut (*pbi).common as *mut Vp8Common);

    (*pbi).common.error.setjmp = 0;

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

    pbi
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
#[no_mangle]
pub unsafe extern "C" fn vp8dx_get_reference(
    pbi: *mut Vp8dComp<'static>,
    ref_frame_flag: VpxRefFrameType,
    sd: *mut Yv12BufferConfig,
) -> VpxCodecErr {
    let cm: *mut Vp8Common = &mut (*pbi).common;
    let ref_fb_idx: i32;

    if ref_frame_flag == VP8_LAST_FRAME {
        ref_fb_idx = (*cm).lst_fb_idx;
    } else if ref_frame_flag == VP8_GOLD_FRAME {
        ref_fb_idx = (*cm).gld_fb_idx;
    } else if ref_frame_flag == VP8_ALTR_FRAME {
        ref_fb_idx = (*cm).alt_fb_idx;
    } else {
        vpx_internal_error(
            &mut (*pbi).common.error,
            VPX_CODEC_ERROR,
            c"Invalid reference frame".as_ptr(),
        );
        return (*pbi).common.error.error_code;
    }

    let slot: *mut Yv12BufferConfig =
        &mut (*cm).yv12_fb[ref_fb_idx as usize] as *mut Yv12BufferConfig;
    if (*slot).y_height != (*sd).y_height
        || (*slot).y_width != (*sd).y_width
        || (*slot).uv_height != (*sd).uv_height
        || (*slot).uv_width != (*sd).uv_width
    {
        vpx_internal_error(
            &mut (*pbi).common.error,
            VPX_CODEC_ERROR,
            c"Incorrect buffer dimensions".as_ptr(),
        );
    } else {
        vp8_yv12_copy_frame(slot, sd);
    }

    (*pbi).common.error.error_code
}

/// `vp8dx_set_reference` — `vp8/decoder/onyxd_if.c:153`.
#[no_mangle]
pub unsafe extern "C" fn vp8dx_set_reference(
    pbi: *mut Vp8dComp<'static>,
    ref_frame_flag: VpxRefFrameType,
    sd: *mut Yv12BufferConfig,
) -> VpxCodecErr {
    let cm: *mut Vp8Common = &mut (*pbi).common;
    let mut ref_fb_ptr: *mut i32 = ptr::null_mut();
    let free_fb: i32;

    if ref_frame_flag == VP8_LAST_FRAME {
        ref_fb_ptr = &mut (*cm).lst_fb_idx;
    } else if ref_frame_flag == VP8_GOLD_FRAME {
        ref_fb_ptr = &mut (*cm).gld_fb_idx;
    } else if ref_frame_flag == VP8_ALTR_FRAME {
        ref_fb_ptr = &mut (*cm).alt_fb_idx;
    } else {
        vpx_internal_error(
            &mut (*pbi).common.error,
            VPX_CODEC_ERROR,
            c"Invalid reference frame".as_ptr(),
        );
        return (*pbi).common.error.error_code;
    }

    let slot: *mut Yv12BufferConfig =
        &mut (*cm).yv12_fb[*ref_fb_ptr as usize] as *mut Yv12BufferConfig;
    if (*slot).y_height != (*sd).y_height
        || (*slot).y_width != (*sd).y_width
        || (*slot).uv_height != (*sd).uv_height
        || (*slot).uv_width != (*sd).uv_width
    {
        vpx_internal_error(
            &mut (*pbi).common.error,
            VPX_CODEC_ERROR,
            c"Incorrect buffer dimensions".as_ptr(),
        );
    } else {
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
    }

    (*pbi).common.error.error_code
}

/// `vp8dx_receive_compressed_data` — `vp8/decoder/onyxd_if.c:305`.
#[no_mangle]
pub unsafe extern "C" fn vp8dx_receive_compressed_data(pbi: *mut Vp8dComp<'static>) -> i32 {
    let cm: *mut Vp8Common = &mut (*pbi).common;
    let mut retcode: i32 = -1;

    (*pbi).common.error.error_code = VPX_CODEC_OK;

    retcode = check_fragments_for_errors(pbi);
    if retcode <= 0 {
        return retcode;
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

    retcode = vp8_decode_frame(pbi);

    if retcode < 0 {
        if (*cm).fb_idx_ref_cnt[(*cm).new_fb_idx as usize] > 0 {
            (*cm).fb_idx_ref_cnt[(*cm).new_fb_idx as usize] -= 1;
        }

        (*pbi).common.error.error_code = VPX_CODEC_ERROR;
        // Propagate the error info.
        if (*pbi).mb.error_info.error_code != VPX_CODEC_OK {
            (*pbi).common.error.error_code = (*pbi).mb.error_info.error_code;
            ptr::copy_nonoverlapping(
                (*pbi).mb.error_info.detail.as_ptr(),
                (*pbi).common.error.detail.as_mut_ptr(),
                (*pbi).mb.error_info.detail.len(),
            );
        }
        // goto decode_exit;
        vpx_clear_system_state();
        return retcode;
    }

    if swap_frame_buffers(cm) != 0 {
        (*pbi).common.error.error_code = VPX_CODEC_ERROR;
        // goto decode_exit;
        vpx_clear_system_state();
        return retcode;
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
    retcode
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
