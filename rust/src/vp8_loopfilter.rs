//! Loop-filter driver — literal translation of `vp8/common/vp8_loopfilter.c`.
//!
//! This file does no pixel work itself. It:
//!   1. Builds and refreshes the `loop_filter_info_n` LUT
//!      (`vp8_loop_filter_init`, `vp8_loop_filter_update_sharpness`,
//!      `vp8_loop_filter_frame_init`).
//!   2. Walks the frame in raster order and dispatches per-edge
//!      kernels declared in `loopfilter_filters.c`
//!      (`vp8_loop_filter_frame`, `vp8_loop_filter_row_normal`,
//!      `vp8_loop_filter_row_simple`, `vp8_loop_filter_frame_yonly`,
//!      `vp8_loop_filter_partial_frame`).
//!
//! See `documentation/vp8_files/vp8_loopfilter.md` for the full prose
//! walkthrough and `documentation/vp8_technical_overview.md` §11.

#![allow(non_snake_case)]
#![allow(clippy::too_many_arguments)]

use core::ptr;

use crate::types::{
    FrameType, LoopFilterInfo, LoopFilterInfoN, Macroblockd,
    MbPredictionMode, ModeInfo, Vp8Common, INTRA_FRAME, MAX_LOOP_FILTER,
    MAX_MB_SEGMENTS, MAX_REF_FRAMES,
};

// ---------------------------------------------------------------------------
// C #defines / enum values needed locally.
// ---------------------------------------------------------------------------

/// `SEGMENT_ABSDATA` (`vp8/common/blockd.h:39`).
const SEGMENT_ABSDATA: u8 = 1;

/// `MB_LVL_ALT_LF` — index into `segment_feature_data` for the alt-LF
/// feature. Matches the `MbLevelFeature::AltLf` discriminant.
const MB_LVL_ALT_LF: usize = 1;

// ---------------------------------------------------------------------------
// extern dependencies (translated in other modules).
//
// The RTCD layer would normally route `vp8_loop_filter_mbv` etc. to the
// best SIMD variant; on `--target=generic-gnu` it resolves to the `_c`
// reference implementation in `loopfilter_filters.c`. We call the `_c`
// symbols directly.
// ---------------------------------------------------------------------------

use crate::loopfilter_filters::{
    vp8_loop_filter_bh_c, vp8_loop_filter_bhs_c, vp8_loop_filter_bv_c, vp8_loop_filter_bvs_c,
    vp8_loop_filter_mbh_c, vp8_loop_filter_mbv_c, vp8_loop_filter_simple_horizontal_edge_c,
    vp8_loop_filter_simple_vertical_edge_c,
};

// ---------------------------------------------------------------------------
// RTCD alias shims — let the body code read like the C source.
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn vp8_loop_filter_mbv(
    y_ptr: *mut u8,
    u_ptr: *mut u8,
    v_ptr: *mut u8,
    y_stride: i32,
    uv_stride: i32,
    lfi: *mut LoopFilterInfo,
) {
    vp8_loop_filter_mbv_c(y_ptr, u_ptr, v_ptr, y_stride, uv_stride, lfi)
}

#[inline(always)]
unsafe fn vp8_loop_filter_bv(
    y_ptr: *mut u8,
    u_ptr: *mut u8,
    v_ptr: *mut u8,
    y_stride: i32,
    uv_stride: i32,
    lfi: *mut LoopFilterInfo,
) {
    vp8_loop_filter_bv_c(y_ptr, u_ptr, v_ptr, y_stride, uv_stride, lfi)
}

#[inline(always)]
unsafe fn vp8_loop_filter_mbh(
    y_ptr: *mut u8,
    u_ptr: *mut u8,
    v_ptr: *mut u8,
    y_stride: i32,
    uv_stride: i32,
    lfi: *mut LoopFilterInfo,
) {
    vp8_loop_filter_mbh_c(y_ptr, u_ptr, v_ptr, y_stride, uv_stride, lfi)
}

#[inline(always)]
unsafe fn vp8_loop_filter_bh(
    y_ptr: *mut u8,
    u_ptr: *mut u8,
    v_ptr: *mut u8,
    y_stride: i32,
    uv_stride: i32,
    lfi: *mut LoopFilterInfo,
) {
    vp8_loop_filter_bh_c(y_ptr, u_ptr, v_ptr, y_stride, uv_stride, lfi)
}

#[inline(always)]
unsafe fn vp8_loop_filter_simple_mbv(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    vp8_loop_filter_simple_vertical_edge_c(y_ptr, y_stride, blimit)
}

#[inline(always)]
unsafe fn vp8_loop_filter_simple_bv(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    vp8_loop_filter_bvs_c(y_ptr, y_stride, blimit)
}

#[inline(always)]
unsafe fn vp8_loop_filter_simple_mbh(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    vp8_loop_filter_simple_horizontal_edge_c(y_ptr, y_stride, blimit)
}

#[inline(always)]
unsafe fn vp8_loop_filter_simple_bh(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    vp8_loop_filter_bhs_c(y_ptr, y_stride, blimit)
}


// ---------------------------------------------------------------------------
// `lf_init_lut` — static helper (vp8/common/vp8_loopfilter.c:17).
// ---------------------------------------------------------------------------

unsafe fn lf_init_lut(lfi: *mut LoopFilterInfoN) {
    for filt_lvl in 0..=MAX_LOOP_FILTER as usize {
        if filt_lvl >= 40 {
            (*lfi).hev_thr_lut[FrameType::Key as usize][filt_lvl] = 2;
            (*lfi).hev_thr_lut[FrameType::Inter as usize][filt_lvl] = 3;
        } else if filt_lvl >= 20 {
            (*lfi).hev_thr_lut[FrameType::Key as usize][filt_lvl] = 1;
            (*lfi).hev_thr_lut[FrameType::Inter as usize][filt_lvl] = 2;
        } else if filt_lvl >= 15 {
            (*lfi).hev_thr_lut[FrameType::Key as usize][filt_lvl] = 1;
            (*lfi).hev_thr_lut[FrameType::Inter as usize][filt_lvl] = 1;
        } else {
            (*lfi).hev_thr_lut[FrameType::Key as usize][filt_lvl] = 0;
            (*lfi).hev_thr_lut[FrameType::Inter as usize][filt_lvl] = 0;
        }
    }

    (*lfi).mode_lf_lut[MbPredictionMode::DcPred as usize] = 1;
    (*lfi).mode_lf_lut[MbPredictionMode::VPred as usize] = 1;
    (*lfi).mode_lf_lut[MbPredictionMode::HPred as usize] = 1;
    (*lfi).mode_lf_lut[MbPredictionMode::TmPred as usize] = 1;
    (*lfi).mode_lf_lut[MbPredictionMode::BPred as usize] = 0;

    (*lfi).mode_lf_lut[MbPredictionMode::ZeroMv as usize] = 1;
    (*lfi).mode_lf_lut[MbPredictionMode::NearestMv as usize] = 2;
    (*lfi).mode_lf_lut[MbPredictionMode::NearMv as usize] = 2;
    (*lfi).mode_lf_lut[MbPredictionMode::NewMv as usize] = 2;
    (*lfi).mode_lf_lut[MbPredictionMode::SplitMv as usize] = 3;
}

// ---------------------------------------------------------------------------
// Public functions
// ---------------------------------------------------------------------------

/// `vp8_loop_filter_update_sharpness` (vp8/common/vp8_loopfilter.c:49).
///
/// For each possible value of `filter_level` (0..=63), fills out the
/// three per-strength byte vectors (`lim`, `blim`, `mblim`) in `lfi`
/// using the current sharpness level. See RFC 6386 §15.4.
pub unsafe fn vp8_loop_filter_update_sharpness(
    lfi: *mut LoopFilterInfoN,
    sharpness_lvl: i32,
) {
    /* For each possible value for the loop filter fill out limits */
    for i in 0..=MAX_LOOP_FILTER as usize {
        let filt_lvl: i32 = i as i32;

        /* Set loop filter paramaeters that control sharpness. */
        let mut block_inside_limit: i32 = filt_lvl >> ((sharpness_lvl > 0) as i32);
        block_inside_limit >>= (sharpness_lvl > 4) as i32;

        if sharpness_lvl > 0 && block_inside_limit > (9 - sharpness_lvl) {
            block_inside_limit = 9 - sharpness_lvl;
        }

        if block_inside_limit < 1 {
            block_inside_limit = 1;
        }

        (*lfi).lim[i].fill(block_inside_limit as u8);
        (*lfi).blim[i].fill((2 * filt_lvl + block_inside_limit) as u8);
        (*lfi).mblim[i].fill((2 * (filt_lvl + 2) + block_inside_limit) as u8);
    }
}

/// `vp8_loop_filter_init` (vp8/common/vp8_loopfilter.c:77).
///
/// One-time setup at `VP8_COMMON` allocation: populates the sharpness-
/// dependent threshold tables, the mode/hev-threshold LUTs, and the
/// four broadcast `hev_thr` vectors.
pub unsafe fn vp8_loop_filter_init(cm: *mut Vp8Common) {
    let lfi: *mut LoopFilterInfoN = &mut (*cm).lf_info;

    /* init limits for given sharpness*/
    vp8_loop_filter_update_sharpness(lfi, (*cm).sharpness_level);
    (*cm).last_sharpness_level = (*cm).sharpness_level;

    /* init LUT for lvl  and hev thr picking */
    lf_init_lut(lfi);

    /* init hev threshold const vectors */
    for i in 0..4usize {
        (*lfi).hev_thr[i].fill(i as u8);
    }
}

/// `vp8_loop_filter_frame_init` (vp8/common/vp8_loopfilter.c:94).
///
/// Per-frame derivation of the `lvl[seg][ref][mode]` per-MB strength
/// table. Refreshes the sharpness tables if `sharpness_level` changed.
pub unsafe fn vp8_loop_filter_frame_init(
    cm: *mut Vp8Common,
    mbd: *mut Macroblockd,
    default_filt_lvl: i32,
) {
    let lfi: *mut LoopFilterInfoN = &mut (*cm).lf_info;

    /* update limits if sharpness has changed */
    if (*cm).last_sharpness_level != (*cm).sharpness_level {
        vp8_loop_filter_update_sharpness(lfi, (*cm).sharpness_level);
        (*cm).last_sharpness_level = (*cm).sharpness_level;
    }

    for seg in 0..MAX_MB_SEGMENTS as usize {
        let mut lvl_seg: i32 = default_filt_lvl;
        let mut lvl_ref: i32;
        let mut lvl_mode: i32;

        /* Note the baseline filter values for each segment */
        if (*mbd).segmentation_enabled != 0 {
            if (*mbd).mb_segment_abs_delta == SEGMENT_ABSDATA {
                lvl_seg = (*mbd).segment_feature_data[MB_LVL_ALT_LF][seg] as i32;
            } else {
                /* Delta Value */
                lvl_seg += (*mbd).segment_feature_data[MB_LVL_ALT_LF][seg] as i32;
            }
            lvl_seg = lvl_seg.clamp(0, 63);
        }

        if (*mbd).mode_ref_lf_delta_enabled == 0 {
            /* we could get rid of this if we assume that deltas are set to
             * zero when not in use; encoder always uses deltas
             */
            for row in (*lfi).lvl[seg].iter_mut() {
                row.fill(lvl_seg as u8);
            }
            continue;
        }

        /* INTRA_FRAME */
        let intra_ref = INTRA_FRAME as usize;

        /* Apply delta for reference frame */
        lvl_ref = lvl_seg + (*mbd).ref_lf_deltas[intra_ref] as i32;

        /* Apply delta for Intra modes */
        /* mode = 0: B_PRED — only the split mode BPRED has a further special case */
        lvl_mode = lvl_ref + (*mbd).mode_lf_deltas[0] as i32;
        /* clamp */
        lvl_mode = lvl_mode.clamp(0, 63);

        (*lfi).lvl[seg][intra_ref][0] = lvl_mode as u8;

        /* mode = 1: all the rest of Intra modes — clamp */
        lvl_mode = lvl_ref.clamp(0, 63);
        (*lfi).lvl[seg][intra_ref][1] = lvl_mode as u8;

        /* LAST, GOLDEN, ALT */
        for r#ref in 1..MAX_REF_FRAMES as usize {
            /* Apply delta for reference frame */
            lvl_ref = lvl_seg + (*mbd).ref_lf_deltas[r#ref] as i32;

            /* Apply delta for Inter modes */
            for mode in 1..4usize {
                lvl_mode = lvl_ref + (*mbd).mode_lf_deltas[mode] as i32;
                /* clamp */
                lvl_mode = lvl_mode.clamp(0, 63);

                (*lfi).lvl[seg][r#ref][mode] = lvl_mode as u8;
            }
        }
    }
}

/// `vp8_loop_filter_row_normal` (vp8/common/vp8_loopfilter.c:167).
///
/// Row-granular normal-filter walker — used by the threaded build
/// (`vp8/decoder/threading.c`). One MB-row's worth of edge dispatches.
pub unsafe fn vp8_loop_filter_row_normal(
    cm: *mut Vp8Common,
    mut mode_info_context: *mut ModeInfo,
    mb_row: i32,
    post_ystride: i32,
    post_uvstride: i32,
    mut y_ptr: *mut u8,
    mut u_ptr: *mut u8,
    mut v_ptr: *mut u8,
) {
    let lfi_n: *mut LoopFilterInfoN = &mut (*cm).lf_info;
    let mut lfi = LoopFilterInfo {
        mblim: ptr::null(),
        blim: ptr::null(),
        lim: ptr::null(),
        hev_thr: ptr::null(),
    };
    let frame_type: FrameType = (*cm).frame_type;

    for mb_col in 0..(*cm).mb_cols {
        let skip_lf: bool = (*mode_info_context).mbmi.mode != MbPredictionMode::BPred
            && (*mode_info_context).mbmi.mode != MbPredictionMode::SplitMv
            && (*mode_info_context).mbmi.mb_skip_coeff;

        let mode_index = (*lfi_n).mode_lf_lut[(*mode_info_context).mbmi.mode as usize] as usize;
        let seg = (*mode_info_context).mbmi.segment_id as usize;
        let ref_frame = (*mode_info_context).mbmi.ref_frame as usize;

        let filter_level = (*lfi_n).lvl[seg][ref_frame][mode_index] as usize;

        if filter_level != 0 {
            let hev_index = (*lfi_n).hev_thr_lut[frame_type as usize][filter_level] as usize;
            lfi.mblim = (*lfi_n).mblim[filter_level].as_ptr();
            lfi.blim = (*lfi_n).blim[filter_level].as_ptr();
            lfi.lim = (*lfi_n).lim[filter_level].as_ptr();
            lfi.hev_thr = (*lfi_n).hev_thr[hev_index].as_ptr();

            if mb_col > 0 {
                vp8_loop_filter_mbv(
                    y_ptr, u_ptr, v_ptr, post_ystride, post_uvstride, &mut lfi,
                );
            }

            if !skip_lf {
                vp8_loop_filter_bv(
                    y_ptr, u_ptr, v_ptr, post_ystride, post_uvstride, &mut lfi,
                );
            }

            /* don't apply across umv border */
            if mb_row > 0 {
                vp8_loop_filter_mbh(
                    y_ptr, u_ptr, v_ptr, post_ystride, post_uvstride, &mut lfi,
                );
            }

            if !skip_lf {
                vp8_loop_filter_bh(
                    y_ptr, u_ptr, v_ptr, post_ystride, post_uvstride, &mut lfi,
                );
            }
        }

        y_ptr = y_ptr.offset(16);
        u_ptr = u_ptr.offset(8);
        v_ptr = v_ptr.offset(8);

        mode_info_context = mode_info_context.offset(1); /* step to next MB */
    }
}

/// `vp8_loop_filter_row_simple` (vp8/common/vp8_loopfilter.c:221).
///
/// Row-granular simple-filter walker — luma-only, no `hev_thr`.
pub unsafe fn vp8_loop_filter_row_simple(
    cm: *mut Vp8Common,
    mut mode_info_context: *mut ModeInfo,
    mb_row: i32,
    post_ystride: i32,
    mut y_ptr: *mut u8,
) {
    let lfi_n: *mut LoopFilterInfoN = &mut (*cm).lf_info;

    for mb_col in 0..(*cm).mb_cols {
        let skip_lf: bool = (*mode_info_context).mbmi.mode != MbPredictionMode::BPred
            && (*mode_info_context).mbmi.mode != MbPredictionMode::SplitMv
            && (*mode_info_context).mbmi.mb_skip_coeff;

        let mode_index = (*lfi_n).mode_lf_lut[(*mode_info_context).mbmi.mode as usize] as usize;
        let seg = (*mode_info_context).mbmi.segment_id as usize;
        let ref_frame = (*mode_info_context).mbmi.ref_frame as usize;

        let filter_level = (*lfi_n).lvl[seg][ref_frame][mode_index] as usize;

        if filter_level != 0 {
            if mb_col > 0 {
                vp8_loop_filter_simple_mbv(
                    y_ptr,
                    post_ystride,
                    (*lfi_n).mblim[filter_level].as_ptr(),
                );
            }

            if !skip_lf {
                vp8_loop_filter_simple_bv(
                    y_ptr,
                    post_ystride,
                    (*lfi_n).blim[filter_level].as_ptr(),
                );
            }

            /* don't apply across umv border */
            if mb_row > 0 {
                vp8_loop_filter_simple_mbh(
                    y_ptr,
                    post_ystride,
                    (*lfi_n).mblim[filter_level].as_ptr(),
                );
            }

            if !skip_lf {
                vp8_loop_filter_simple_bh(
                    y_ptr,
                    post_ystride,
                    (*lfi_n).blim[filter_level].as_ptr(),
                );
            }
        }

        y_ptr = y_ptr.offset(16);

        mode_info_context = mode_info_context.offset(1); /* step to next MB */
    }
}

