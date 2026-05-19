//! Literal Rust translation of `vp8/common/alloccommon.c`.
//!
//! Owns the construction, dimension-driven (re)allocation, and teardown of
//! every heap-resident region attached to a [`Vp8Common`] instance. Mirrors
//! the C control flow verbatim — each public entry point corresponds 1:1 to
//! the C prototype in `vp8/common/alloccommon.h`.
//!
//! The minimal build targeted by `rust_types.md` does not compile
//! `CONFIG_POSTPROC` or `CONFIG_ERROR_CONCEALMENT`, so the gated blocks
//! from the C source are intentionally omitted here.

#![allow(dead_code)]

use core::ffi::c_void;
use core::ptr;

use crate::types::{
    ClampType, EntropyContextPlanes, LoopFilterType, MAX_REF_FRAMES, ModeInfo, NUM_YV12_BUFFERS,
    TokenPartition, VP8_BORDER_IN_PIXELS, Vp8Common,
};

// ---------------------------------------------------------------------------
// Cross-translation-unit dependencies.
// ---------------------------------------------------------------------------

use crate::entropymode::{vp8_default_bmode_probs, vp8_init_mbmode_probs};
use crate::vpx_mem::{vpx_calloc, vpx_free};
use crate::yv12config::{vp8_yv12_alloc_frame_buffer, vp8_yv12_de_alloc_frame_buffer};

// ---------------------------------------------------------------------------
// Public entry points (mirrors `alloccommon.h:20-24`).
// ---------------------------------------------------------------------------

/// `vp8_de_alloc_frame_buffers` — free all heap-resident regions reachable
/// from `oci`. NULL-safe and idempotent.
///
/// Source: `vp8/common/alloccommon.c:22`.
pub unsafe fn vp8_de_alloc_frame_buffers(oci: *mut Vp8Common) {
    for i in 0..NUM_YV12_BUFFERS {
        vp8_yv12_de_alloc_frame_buffer(&mut (*oci).yv12_fb[i]);
        (*oci).fb_idx_ref_cnt[i] = 0;
    }

    vp8_yv12_de_alloc_frame_buffer(&mut (*oci).temp_scale_frame);

    // CONFIG_POSTPROC block omitted (minimal build).

    vpx_free((*oci).above_context as *mut c_void);
    vpx_free((*oci).mip as *mut c_void);

    // CONFIG_ERROR_CONCEALMENT block omitted (minimal build).

    (*oci).above_context = ptr::null_mut();
    (*oci).mip = ptr::null_mut();
    (*oci).mi = ptr::null_mut();
    (*oci).frame_to_show_idx = -1;
}

/// `vp8_alloc_frame_buffers` — (re)allocate every heap region attached to
/// `oci` keyed by `width`/`height`. Returns 0 on success, 1 on failure
/// (matching the C convention).
///
/// Source: `vp8/common/alloccommon.c:59`.
pub unsafe fn vp8_alloc_frame_buffers(oci: *mut Vp8Common, mut width: i32, mut height: i32) -> i32 {
    vp8_de_alloc_frame_buffers(oci);

    /* our internal buffers are always multiples of 16 */
    if (width & 0xf) != 0 {
        width += 16 - (width & 0xf);
    }

    if (height & 0xf) != 0 {
        height += 16 - (height & 0xf);
    }

    for i in 0..NUM_YV12_BUFFERS {
        if vp8_yv12_alloc_frame_buffer(&mut (*oci).yv12_fb[i], width, height, VP8_BORDER_IN_PIXELS)
            < 0
        {
            // goto allocation_fail
            vp8_de_alloc_frame_buffers(oci);
            return 1;
        }
    }

    (*oci).new_fb_idx = 0;
    (*oci).lst_fb_idx = 1;
    (*oci).gld_fb_idx = 2;
    (*oci).alt_fb_idx = 3;

    (*oci).fb_idx_ref_cnt[0] = 1;
    (*oci).fb_idx_ref_cnt[1] = 1;
    (*oci).fb_idx_ref_cnt[2] = 1;
    (*oci).fb_idx_ref_cnt[3] = 1;

    if vp8_yv12_alloc_frame_buffer(
        &mut (*oci).temp_scale_frame,
        width,
        16,
        VP8_BORDER_IN_PIXELS,
    ) < 0
    {
        vp8_de_alloc_frame_buffers(oci);
        return 1;
    }

    (*oci).mb_rows = height >> 4;
    (*oci).mb_cols = width >> 4;
    (*oci).mbs = (*oci).mb_rows * (*oci).mb_cols;
    (*oci).mode_info_stride = (*oci).mb_cols + 1;
    (*oci).mip = vpx_calloc(
        (((*oci).mb_cols + 1) * ((*oci).mb_rows + 1)) as usize,
        core::mem::size_of::<ModeInfo>(),
    ) as *mut ModeInfo;

    if (*oci).mip.is_null() {
        vp8_de_alloc_frame_buffers(oci);
        return 1;
    }

    (*oci).mi = (*oci).mip.offset(((*oci).mode_info_stride + 1) as isize);

    /* Allocation of previous mode info will be done in vp8_decode_frame()
     * as it is a decoder only data */

    (*oci).above_context = vpx_calloc(
        core::mem::size_of::<EntropyContextPlanes>() * (*oci).mb_cols as usize,
        1,
    ) as *mut EntropyContextPlanes;

    if (*oci).above_context.is_null() {
        vp8_de_alloc_frame_buffers(oci);
        return 1;
    }

    // CONFIG_POSTPROC block omitted (minimal build).

    0
}

/// `vp8_setup_version` — decode the 3-bit frame-tag `version` field into
/// the four derived profile flags on `cm`.
///
/// Source: `vp8/common/alloccommon.c:134`.
pub unsafe fn vp8_setup_version(cm: *mut Vp8Common) {
    match (*cm).version {
        0 => {
            (*cm).no_lpf = 0;
            (*cm).filter_type = LoopFilterType::Normal;
            (*cm).use_bilinear_mc_filter = 0;
            (*cm).full_pixel = 0;
        }
        1 => {
            (*cm).no_lpf = 0;
            (*cm).filter_type = LoopFilterType::Simple;
            (*cm).use_bilinear_mc_filter = 1;
            (*cm).full_pixel = 0;
        }
        2 => {
            (*cm).no_lpf = 1;
            (*cm).filter_type = LoopFilterType::Normal;
            (*cm).use_bilinear_mc_filter = 1;
            (*cm).full_pixel = 0;
        }
        3 => {
            (*cm).no_lpf = 1;
            (*cm).filter_type = LoopFilterType::Simple;
            (*cm).use_bilinear_mc_filter = 1;
            (*cm).full_pixel = 1;
        }
        _ => {
            /*4,5,6,7 are reserved for future use*/
            (*cm).no_lpf = 0;
            (*cm).filter_type = LoopFilterType::Normal;
            (*cm).use_bilinear_mc_filter = 0;
            (*cm).full_pixel = 0;
        }
    }
}

/// `vp8_create_common` — one-time per-instance initialization. Sets up
/// RTCD function-pointer tables, seeds default entropy probability
/// tables, and writes the profile/sign-bias defaults expected before any
/// frame tag has been parsed.
///
/// Source: `vp8/common/alloccommon.c:169`.
pub unsafe fn vp8_create_common(oci: *mut Vp8Common) {
    // `vp8_machine_specific_config` in the C source either runs a CPU
    // probe (multithread builds) or is a no-op (single-thread). With
    // `--disable-multithread` it's a no-op, so the call is omitted.

    vp8_init_mbmode_probs(&mut *oci);
    vp8_default_bmode_probs(&mut (*oci).fc.bmode_prob);

    (*oci).mb_no_coeff_skip = 1;
    (*oci).no_lpf = 0;
    (*oci).filter_type = LoopFilterType::Normal;
    (*oci).use_bilinear_mc_filter = 0;
    (*oci).full_pixel = 0;
    (*oci).multi_token_partition = TokenPartition::One;
    (*oci).clamp_type = ClampType::Required;

    /* Initialize reference frame sign bias structure to defaults */
    ptr::write_bytes((*oci).ref_frame_sign_bias.as_mut_ptr(), 0, MAX_REF_FRAMES);

    /* Default disable buffer to buffer copying */
    (*oci).copy_buffer_to_gf = 0;
    (*oci).copy_buffer_to_arf = 0;

    /* No frame ready until the first decode completes. zero-init of
     * `vpx_calloc` would otherwise leave this at the valid index 0. */
    (*oci).frame_to_show_idx = -1;
}

/// `vp8_remove_common` — instance teardown. Forwards to the frame-buffer
/// deallocator; does not free `oci` itself (the parent `Vp8dComp` owns it).
///
/// Source: `vp8/common/alloccommon.c:191`.
pub unsafe fn vp8_remove_common(oci: *mut Vp8Common) {
    vp8_de_alloc_frame_buffers(oci);
}
