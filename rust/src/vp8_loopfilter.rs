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
    FrameType, LoopFilterInfo, LoopFilterInfoN, LoopFilterType, Macroblockd,
    MbPredictionMode, ModeInfo, Vp8Common, MAX_LOOP_FILTER, MAX_MB_SEGMENTS,
    MAX_REF_FRAMES, SIMD_WIDTH,
};

// ---------------------------------------------------------------------------
// C #defines / enum values needed locally.
// ---------------------------------------------------------------------------

/// `SEGMENT_ABSDATA` (`vp8/common/blockd.h:39`).
const SEGMENT_ABSDATA: u8 = 1;

/// `MB_LVL_ALT_LF` — index into `segment_feature_data` for the alt-LF
/// feature. Matches the `MbLevelFeature::AltLf` discriminant.
const MB_LVL_ALT_LF: usize = 1;

/// `INTRA_FRAME` ref-frame id (`vp8/common/blockd.h:134`).
const INTRA_FRAME: usize = 0;

/// `PARTIAL_FRAME_FRACTION` (`vp8/common/loopfilter.h:25`).
const PARTIAL_FRAME_FRACTION: i32 = 8;

// ---------------------------------------------------------------------------
// extern dependencies (translated in other modules).
//
// The RTCD layer would normally route `vp8_loop_filter_mbv` etc. to the
// best SIMD variant; on `--target=generic-gnu` it resolves to the `_c`
// reference implementation in `loopfilter_filters.c`. We call the `_c`
// symbols directly.
// ---------------------------------------------------------------------------

unsafe extern "Rust" {
    /// `vp8_loop_filter_mbv_c` (vp8/common/loopfilter_filters.c:321).
    fn vp8_loop_filter_mbv_c(
        y_ptr: *mut u8,
        u_ptr: *mut u8,
        v_ptr: *mut u8,
        y_stride: i32,
        uv_stride: i32,
        lfi: *mut LoopFilterInfo,
    );

    /// `vp8_loop_filter_bv_c` (vp8/common/loopfilter_filters.c:371).
    fn vp8_loop_filter_bv_c(
        y_ptr: *mut u8,
        u_ptr: *mut u8,
        v_ptr: *mut u8,
        y_stride: i32,
        uv_stride: i32,
        lfi: *mut LoopFilterInfo,
    );

    /// `vp8_loop_filter_mbh_c` (vp8/common/loopfilter_filters.c:303).
    fn vp8_loop_filter_mbh_c(
        y_ptr: *mut u8,
        u_ptr: *mut u8,
        v_ptr: *mut u8,
        y_stride: i32,
        uv_stride: i32,
        lfi: *mut LoopFilterInfo,
    );

    /// `vp8_loop_filter_bh_c` (vp8/common/loopfilter_filters.c:339).
    fn vp8_loop_filter_bh_c(
        y_ptr: *mut u8,
        u_ptr: *mut u8,
        v_ptr: *mut u8,
        y_stride: i32,
        uv_stride: i32,
        lfi: *mut LoopFilterInfo,
    );

    /// `vp8_loop_filter_simple_vertical_edge_c`
    /// (vp8/common/loopfilter_filters.c:289). Aliased by RTCD as
    /// `vp8_loop_filter_simple_mbv`.
    fn vp8_loop_filter_simple_vertical_edge_c(
        y_ptr: *mut u8,
        y_stride: i32,
        blimit: *const u8,
    );

    /// `vp8_loop_filter_bvs_c` (vp8/common/loopfilter_filters.c:392).
    /// Aliased by RTCD as `vp8_loop_filter_simple_bv`.
    fn vp8_loop_filter_bvs_c(y_ptr: *mut u8, y_stride: i32, blimit: *const u8);

    /// `vp8_loop_filter_simple_horizontal_edge_c`
    /// (vp8/common/loopfilter_filters.c:273). Aliased by RTCD as
    /// `vp8_loop_filter_simple_mbh`.
    fn vp8_loop_filter_simple_horizontal_edge_c(
        y_ptr: *mut u8,
        y_stride: i32,
        blimit: *const u8,
    );

    /// `vp8_loop_filter_bhs_c` (vp8/common/loopfilter_filters.c:360).
    /// Aliased by RTCD as `vp8_loop_filter_simple_bh`.
    fn vp8_loop_filter_bhs_c(y_ptr: *mut u8, y_stride: i32, blimit: *const u8);
}

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
    unsafe { vp8_loop_filter_mbv_c(y_ptr, u_ptr, v_ptr, y_stride, uv_stride, lfi) }
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
    unsafe { vp8_loop_filter_bv_c(y_ptr, u_ptr, v_ptr, y_stride, uv_stride, lfi) }
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
    unsafe { vp8_loop_filter_mbh_c(y_ptr, u_ptr, v_ptr, y_stride, uv_stride, lfi) }
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
    unsafe { vp8_loop_filter_bh_c(y_ptr, u_ptr, v_ptr, y_stride, uv_stride, lfi) }
}

#[inline(always)]
unsafe fn vp8_loop_filter_simple_mbv(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    unsafe { vp8_loop_filter_simple_vertical_edge_c(y_ptr, y_stride, blimit) }
}

#[inline(always)]
unsafe fn vp8_loop_filter_simple_bv(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    unsafe { vp8_loop_filter_bvs_c(y_ptr, y_stride, blimit) }
}

#[inline(always)]
unsafe fn vp8_loop_filter_simple_mbh(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    unsafe { vp8_loop_filter_simple_horizontal_edge_c(y_ptr, y_stride, blimit) }
}

#[inline(always)]
unsafe fn vp8_loop_filter_simple_bh(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    unsafe { vp8_loop_filter_bhs_c(y_ptr, y_stride, blimit) }
}

// ---------------------------------------------------------------------------
// `memset` shim (libc `memset(p, v, n)` semantics on bytes).
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn memset_bytes(p: *mut u8, v: u8, n: usize) {
    unsafe { ptr::write_bytes(p, v, n) }
}

// ---------------------------------------------------------------------------
// `lf_init_lut` — static helper (vp8/common/vp8_loopfilter.c:17).
// ---------------------------------------------------------------------------

unsafe fn lf_init_lut(lfi: *mut LoopFilterInfoN) {
    unsafe {
        let mut filt_lvl: i32;

        filt_lvl = 0;
        while filt_lvl <= MAX_LOOP_FILTER as i32 {
            if filt_lvl >= 40 {
                (*lfi).hev_thr_lut[FrameType::Key as usize][filt_lvl as usize] = 2;
                (*lfi).hev_thr_lut[FrameType::Inter as usize][filt_lvl as usize] = 3;
            } else if filt_lvl >= 20 {
                (*lfi).hev_thr_lut[FrameType::Key as usize][filt_lvl as usize] = 1;
                (*lfi).hev_thr_lut[FrameType::Inter as usize][filt_lvl as usize] = 2;
            } else if filt_lvl >= 15 {
                (*lfi).hev_thr_lut[FrameType::Key as usize][filt_lvl as usize] = 1;
                (*lfi).hev_thr_lut[FrameType::Inter as usize][filt_lvl as usize] = 1;
            } else {
                (*lfi).hev_thr_lut[FrameType::Key as usize][filt_lvl as usize] = 0;
                (*lfi).hev_thr_lut[FrameType::Inter as usize][filt_lvl as usize] = 0;
            }
            filt_lvl += 1;
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
    unsafe {
        let mut i: i32;

        /* For each possible value for the loop filter fill out limits */
        i = 0;
        while i <= MAX_LOOP_FILTER as i32 {
            let filt_lvl: i32 = i;
            let mut block_inside_limit: i32 = 0;

            /* Set loop filter paramaeters that control sharpness. */
            block_inside_limit = filt_lvl >> ((sharpness_lvl > 0) as i32);
            block_inside_limit = block_inside_limit >> ((sharpness_lvl > 4) as i32);

            if sharpness_lvl > 0 {
                if block_inside_limit > (9 - sharpness_lvl) {
                    block_inside_limit = 9 - sharpness_lvl;
                }
            }

            if block_inside_limit < 1 {
                block_inside_limit = 1;
            }

            memset_bytes(
                (*lfi).lim[i as usize].as_mut_ptr(),
                block_inside_limit as u8,
                SIMD_WIDTH,
            );
            memset_bytes(
                (*lfi).blim[i as usize].as_mut_ptr(),
                (2 * filt_lvl + block_inside_limit) as u8,
                SIMD_WIDTH,
            );
            memset_bytes(
                (*lfi).mblim[i as usize].as_mut_ptr(),
                (2 * (filt_lvl + 2) + block_inside_limit) as u8,
                SIMD_WIDTH,
            );

            i += 1;
        }
    }
}

/// `vp8_loop_filter_init` (vp8/common/vp8_loopfilter.c:77).
///
/// One-time setup at `VP8_COMMON` allocation: populates the sharpness-
/// dependent threshold tables, the mode/hev-threshold LUTs, and the
/// four broadcast `hev_thr` vectors.
pub unsafe fn vp8_loop_filter_init(cm: *mut Vp8Common) {
    unsafe {
        let lfi: *mut LoopFilterInfoN = &mut (*cm).lf_info;
        let mut i: i32;

        /* init limits for given sharpness*/
        vp8_loop_filter_update_sharpness(lfi, (*cm).sharpness_level);
        (*cm).last_sharpness_level = (*cm).sharpness_level;

        /* init LUT for lvl  and hev thr picking */
        lf_init_lut(lfi);

        /* init hev threshold const vectors */
        i = 0;
        while i < 4 {
            memset_bytes((*lfi).hev_thr[i as usize].as_mut_ptr(), i as u8, SIMD_WIDTH);
            i += 1;
        }
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
    unsafe {
        let mut seg: i32;
        let mut r#ref: i32;
        let mut mode: i32;

        let lfi: *mut LoopFilterInfoN = &mut (*cm).lf_info;

        /* update limits if sharpness has changed */
        if (*cm).last_sharpness_level != (*cm).sharpness_level {
            vp8_loop_filter_update_sharpness(lfi, (*cm).sharpness_level);
            (*cm).last_sharpness_level = (*cm).sharpness_level;
        }

        seg = 0;
        while seg < MAX_MB_SEGMENTS as i32 {
            let mut lvl_seg: i32 = default_filt_lvl;
            let mut lvl_ref: i32;
            let mut lvl_mode: i32;

            /* Note the baseline filter values for each segment */
            if (*mbd).segmentation_enabled != 0 {
                if (*mbd).mb_segment_abs_delta == SEGMENT_ABSDATA {
                    lvl_seg =
                        (*mbd).segment_feature_data[MB_LVL_ALT_LF][seg as usize] as i32;
                } else {
                    /* Delta Value */
                    lvl_seg += (*mbd).segment_feature_data[MB_LVL_ALT_LF][seg as usize]
                        as i32;
                }
                lvl_seg = if lvl_seg > 0 {
                    if lvl_seg > 63 {
                        63
                    } else {
                        lvl_seg
                    }
                } else {
                    0
                };
            }

            if (*mbd).mode_ref_lf_delta_enabled == 0 {
                /* we could get rid of this if we assume that deltas are set to
                 * zero when not in use; encoder always uses deltas
                 */
                memset_bytes(
                    (*lfi).lvl[seg as usize][0].as_mut_ptr(),
                    lvl_seg as u8,
                    4 * 4,
                );
                seg += 1;
                continue;
            }

            /* INTRA_FRAME */
            r#ref = INTRA_FRAME as i32;

            /* Apply delta for reference frame */
            lvl_ref = lvl_seg + (*mbd).ref_lf_deltas[r#ref as usize] as i32;

            /* Apply delta for Intra modes */
            mode = 0; /* B_PRED */
            /* Only the split mode BPRED has a further special case */
            lvl_mode = lvl_ref + (*mbd).mode_lf_deltas[mode as usize] as i32;
            /* clamp */
            lvl_mode = if lvl_mode > 0 {
                if lvl_mode > 63 {
                    63
                } else {
                    lvl_mode
                }
            } else {
                0
            };

            (*lfi).lvl[seg as usize][r#ref as usize][mode as usize] = lvl_mode as u8;

            mode = 1; /* all the rest of Intra modes */
            /* clamp */
            lvl_mode = if lvl_ref > 0 {
                if lvl_ref > 63 {
                    63
                } else {
                    lvl_ref
                }
            } else {
                0
            };
            (*lfi).lvl[seg as usize][r#ref as usize][mode as usize] = lvl_mode as u8;

            /* LAST, GOLDEN, ALT */
            r#ref = 1;
            while r#ref < MAX_REF_FRAMES as i32 {
                /* Apply delta for reference frame */
                lvl_ref = lvl_seg + (*mbd).ref_lf_deltas[r#ref as usize] as i32;

                /* Apply delta for Inter modes */
                mode = 1;
                while mode < 4 {
                    lvl_mode = lvl_ref + (*mbd).mode_lf_deltas[mode as usize] as i32;
                    /* clamp */
                    lvl_mode = if lvl_mode > 0 {
                        if lvl_mode > 63 {
                            63
                        } else {
                            lvl_mode
                        }
                    } else {
                        0
                    };

                    (*lfi).lvl[seg as usize][r#ref as usize][mode as usize] =
                        lvl_mode as u8;
                    mode += 1;
                }
                r#ref += 1;
            }

            seg += 1;
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
    unsafe {
        let mut mb_col: i32;
        let mut filter_level: i32;
        let lfi_n: *mut LoopFilterInfoN = &mut (*cm).lf_info;
        let mut lfi = LoopFilterInfo {
            mblim: ptr::null(),
            blim: ptr::null(),
            lim: ptr::null(),
            hev_thr: ptr::null(),
        };
        let frame_type: FrameType = (*cm).frame_type;

        mb_col = 0;
        while mb_col < (*cm).mb_cols {
            let skip_lf: bool = (*mode_info_context).mbmi.mode != MbPredictionMode::BPred
                && (*mode_info_context).mbmi.mode != MbPredictionMode::SplitMv
                && (*mode_info_context).mbmi.mb_skip_coeff;

            let mode_index: i32 =
                (*lfi_n).mode_lf_lut[(*mode_info_context).mbmi.mode as usize] as i32;
            let seg: i32 = (*mode_info_context).mbmi.segment_id as i32;
            let ref_frame: i32 = (*mode_info_context).mbmi.ref_frame as i32;

            filter_level =
                (*lfi_n).lvl[seg as usize][ref_frame as usize][mode_index as usize] as i32;

            if filter_level != 0 {
                let hev_index: i32 =
                    (*lfi_n).hev_thr_lut[frame_type as usize][filter_level as usize] as i32;
                lfi.mblim = (*lfi_n).mblim[filter_level as usize].as_ptr();
                lfi.blim = (*lfi_n).blim[filter_level as usize].as_ptr();
                lfi.lim = (*lfi_n).lim[filter_level as usize].as_ptr();
                lfi.hev_thr = (*lfi_n).hev_thr[hev_index as usize].as_ptr();

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
            mb_col += 1;
        }
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
    unsafe {
        let mut mb_col: i32;
        let mut filter_level: i32;
        let lfi_n: *mut LoopFilterInfoN = &mut (*cm).lf_info;

        mb_col = 0;
        while mb_col < (*cm).mb_cols {
            let skip_lf: bool = (*mode_info_context).mbmi.mode != MbPredictionMode::BPred
                && (*mode_info_context).mbmi.mode != MbPredictionMode::SplitMv
                && (*mode_info_context).mbmi.mb_skip_coeff;

            let mode_index: i32 =
                (*lfi_n).mode_lf_lut[(*mode_info_context).mbmi.mode as usize] as i32;
            let seg: i32 = (*mode_info_context).mbmi.segment_id as i32;
            let ref_frame: i32 = (*mode_info_context).mbmi.ref_frame as i32;

            filter_level =
                (*lfi_n).lvl[seg as usize][ref_frame as usize][mode_index as usize] as i32;

            if filter_level != 0 {
                if mb_col > 0 {
                    vp8_loop_filter_simple_mbv(
                        y_ptr,
                        post_ystride,
                        (*lfi_n).mblim[filter_level as usize].as_ptr(),
                    );
                }

                if !skip_lf {
                    vp8_loop_filter_simple_bv(
                        y_ptr,
                        post_ystride,
                        (*lfi_n).blim[filter_level as usize].as_ptr(),
                    );
                }

                /* don't apply across umv border */
                if mb_row > 0 {
                    vp8_loop_filter_simple_mbh(
                        y_ptr,
                        post_ystride,
                        (*lfi_n).mblim[filter_level as usize].as_ptr(),
                    );
                }

                if !skip_lf {
                    vp8_loop_filter_simple_bh(
                        y_ptr,
                        post_ystride,
                        (*lfi_n).blim[filter_level as usize].as_ptr(),
                    );
                }
            }

            y_ptr = y_ptr.offset(16);

            mode_info_context = mode_info_context.offset(1); /* step to next MB */
            mb_col += 1;
        }
    }
}

/// `vp8_loop_filter_frame` (vp8/common/vp8_loopfilter.c:263).
///
/// Main per-frame raster walker. Two structurally-identical halves
/// (normal vs simple), chosen by `cm->filter_type`.
pub unsafe fn vp8_loop_filter_frame(
    cm: *mut Vp8Common,
    mbd: *mut Macroblockd,
    frame_type_arg: i32,
) {
    unsafe {
        let post = (*cm).frame_to_show;
        let lfi_n: *mut LoopFilterInfoN = &mut (*cm).lf_info;
        let mut lfi = LoopFilterInfo {
            mblim: ptr::null(),
            blim: ptr::null(),
            lim: ptr::null(),
            hev_thr: ptr::null(),
        };

        let mut mb_row: i32;
        let mut mb_col: i32;
        let mb_rows: i32 = (*cm).mb_rows;
        let mb_cols: i32 = (*cm).mb_cols;

        let mut filter_level: i32;

        let mut y_ptr: *mut u8;
        let mut u_ptr: *mut u8;
        let mut v_ptr: *mut u8;

        /* Point at base of Mb MODE_INFO list */
        let mut mode_info_context: *mut ModeInfo = (*cm).mi;
        let post_y_stride: i32 = (*post).y_stride;
        let post_uv_stride: i32 = (*post).uv_stride;

        // `frame_type` from the FRAME_TYPE enum, used to index `hev_thr_lut`.
        // Recover the enum from the raw int parameter passed in (matches the C
        // signature `int frame_type`).
        let frame_type: FrameType = if frame_type_arg == FrameType::Key as i32 {
            FrameType::Key
        } else {
            FrameType::Inter
        };

        /* Initialize the loop filter for this frame. */
        vp8_loop_filter_frame_init(cm, mbd, (*cm).filter_level);

        /* Set up the buffer pointers */
        y_ptr = (*post).y_buffer;
        u_ptr = (*post).u_buffer;
        v_ptr = (*post).v_buffer;

        /* vp8_filter each macro block */
        if (*cm).filter_type == LoopFilterType::Normal {
            mb_row = 0;
            while mb_row < mb_rows {
                mb_col = 0;
                while mb_col < mb_cols {
                    let skip_lf: bool = (*mode_info_context).mbmi.mode
                        != MbPredictionMode::BPred
                        && (*mode_info_context).mbmi.mode != MbPredictionMode::SplitMv
                        && (*mode_info_context).mbmi.mb_skip_coeff;

                    let mode_index: i32 = (*lfi_n).mode_lf_lut
                        [(*mode_info_context).mbmi.mode as usize]
                        as i32;
                    let seg: i32 = (*mode_info_context).mbmi.segment_id as i32;
                    let ref_frame: i32 = (*mode_info_context).mbmi.ref_frame as i32;

                    filter_level = (*lfi_n).lvl[seg as usize][ref_frame as usize]
                        [mode_index as usize]
                        as i32;

                    if filter_level != 0 {
                        let hev_index: i32 = (*lfi_n).hev_thr_lut[frame_type as usize]
                            [filter_level as usize]
                            as i32;
                        lfi.mblim = (*lfi_n).mblim[filter_level as usize].as_ptr();
                        lfi.blim = (*lfi_n).blim[filter_level as usize].as_ptr();
                        lfi.lim = (*lfi_n).lim[filter_level as usize].as_ptr();
                        lfi.hev_thr = (*lfi_n).hev_thr[hev_index as usize].as_ptr();

                        if mb_col > 0 {
                            vp8_loop_filter_mbv(
                                y_ptr,
                                u_ptr,
                                v_ptr,
                                post_y_stride,
                                post_uv_stride,
                                &mut lfi,
                            );
                        }

                        if !skip_lf {
                            vp8_loop_filter_bv(
                                y_ptr,
                                u_ptr,
                                v_ptr,
                                post_y_stride,
                                post_uv_stride,
                                &mut lfi,
                            );
                        }

                        /* don't apply across umv border */
                        if mb_row > 0 {
                            vp8_loop_filter_mbh(
                                y_ptr,
                                u_ptr,
                                v_ptr,
                                post_y_stride,
                                post_uv_stride,
                                &mut lfi,
                            );
                        }

                        if !skip_lf {
                            vp8_loop_filter_bh(
                                y_ptr,
                                u_ptr,
                                v_ptr,
                                post_y_stride,
                                post_uv_stride,
                                &mut lfi,
                            );
                        }
                    }

                    y_ptr = y_ptr.offset(16);
                    u_ptr = u_ptr.offset(8);
                    v_ptr = v_ptr.offset(8);

                    mode_info_context = mode_info_context.offset(1); /* step to next MB */
                    mb_col += 1;
                }
                y_ptr = y_ptr.offset((post_y_stride * 16 - (*post).y_width) as isize);
                u_ptr = u_ptr.offset((post_uv_stride * 8 - (*post).uv_width) as isize);
                v_ptr = v_ptr.offset((post_uv_stride * 8 - (*post).uv_width) as isize);

                mode_info_context = mode_info_context.offset(1); /* Skip border mb */
                mb_row += 1;
            }
        } else {
            /* SIMPLE_LOOPFILTER */
            mb_row = 0;
            while mb_row < mb_rows {
                mb_col = 0;
                while mb_col < mb_cols {
                    let skip_lf: bool = (*mode_info_context).mbmi.mode
                        != MbPredictionMode::BPred
                        && (*mode_info_context).mbmi.mode != MbPredictionMode::SplitMv
                        && (*mode_info_context).mbmi.mb_skip_coeff;

                    let mode_index: i32 = (*lfi_n).mode_lf_lut
                        [(*mode_info_context).mbmi.mode as usize]
                        as i32;
                    let seg: i32 = (*mode_info_context).mbmi.segment_id as i32;
                    let ref_frame: i32 = (*mode_info_context).mbmi.ref_frame as i32;

                    filter_level = (*lfi_n).lvl[seg as usize][ref_frame as usize]
                        [mode_index as usize]
                        as i32;
                    if filter_level != 0 {
                        let mblim: *const u8 =
                            (*lfi_n).mblim[filter_level as usize].as_ptr();
                        let blim: *const u8 =
                            (*lfi_n).blim[filter_level as usize].as_ptr();

                        if mb_col > 0 {
                            vp8_loop_filter_simple_mbv(y_ptr, post_y_stride, mblim);
                        }

                        if !skip_lf {
                            vp8_loop_filter_simple_bv(y_ptr, post_y_stride, blim);
                        }

                        /* don't apply across umv border */
                        if mb_row > 0 {
                            vp8_loop_filter_simple_mbh(y_ptr, post_y_stride, mblim);
                        }

                        if !skip_lf {
                            vp8_loop_filter_simple_bh(y_ptr, post_y_stride, blim);
                        }
                    }

                    y_ptr = y_ptr.offset(16);
                    u_ptr = u_ptr.offset(8);
                    v_ptr = v_ptr.offset(8);

                    mode_info_context = mode_info_context.offset(1); /* step to next MB */
                    mb_col += 1;
                }
                y_ptr = y_ptr.offset((post_y_stride * 16 - (*post).y_width) as isize);
                u_ptr = u_ptr.offset((post_uv_stride * 8 - (*post).uv_width) as isize);
                v_ptr = v_ptr.offset((post_uv_stride * 8 - (*post).uv_width) as isize);

                mode_info_context = mode_info_context.offset(1); /* Skip border mb */
                mb_row += 1;
            }
        }
    }
}

/// `vp8_loop_filter_frame_yonly` (vp8/common/vp8_loopfilter.c:384).
///
/// Luma-only raster walk used by the encoder's filter-level search.
/// The decoder never calls it but it is built as part of the unit.
pub unsafe fn vp8_loop_filter_frame_yonly(
    cm: *mut Vp8Common,
    mbd: *mut Macroblockd,
    default_filt_lvl: i32,
) {
    unsafe {
        let post = (*cm).frame_to_show;

        let mut y_ptr: *mut u8;
        let mut mb_row: i32;
        let mut mb_col: i32;

        let lfi_n: *mut LoopFilterInfoN = &mut (*cm).lf_info;
        let mut lfi = LoopFilterInfo {
            mblim: ptr::null(),
            blim: ptr::null(),
            lim: ptr::null(),
            hev_thr: ptr::null(),
        };

        let mut filter_level: i32;
        let frame_type: FrameType = (*cm).frame_type;

        /* Point at base of Mb MODE_INFO list */
        let mut mode_info_context: *mut ModeInfo = (*cm).mi;

        // #if 0 default_filt_lvl == 0 short-circuit — omitted as in C.

        /* Initialize the loop filter for this frame. */
        vp8_loop_filter_frame_init(cm, mbd, default_filt_lvl);

        /* Set up the buffer pointers */
        y_ptr = (*post).y_buffer;

        /* vp8_filter each macro block */
        mb_row = 0;
        while mb_row < (*cm).mb_rows {
            mb_col = 0;
            while mb_col < (*cm).mb_cols {
                let skip_lf: bool = (*mode_info_context).mbmi.mode
                    != MbPredictionMode::BPred
                    && (*mode_info_context).mbmi.mode != MbPredictionMode::SplitMv
                    && (*mode_info_context).mbmi.mb_skip_coeff;

                let mode_index: i32 =
                    (*lfi_n).mode_lf_lut[(*mode_info_context).mbmi.mode as usize] as i32;
                let seg: i32 = (*mode_info_context).mbmi.segment_id as i32;
                let ref_frame: i32 = (*mode_info_context).mbmi.ref_frame as i32;

                filter_level = (*lfi_n).lvl[seg as usize][ref_frame as usize]
                    [mode_index as usize]
                    as i32;

                if filter_level != 0 {
                    if (*cm).filter_type == LoopFilterType::Normal {
                        let hev_index: i32 = (*lfi_n).hev_thr_lut[frame_type as usize]
                            [filter_level as usize]
                            as i32;
                        lfi.mblim = (*lfi_n).mblim[filter_level as usize].as_ptr();
                        lfi.blim = (*lfi_n).blim[filter_level as usize].as_ptr();
                        lfi.lim = (*lfi_n).lim[filter_level as usize].as_ptr();
                        lfi.hev_thr = (*lfi_n).hev_thr[hev_index as usize].as_ptr();

                        if mb_col > 0 {
                            vp8_loop_filter_mbv(
                                y_ptr,
                                ptr::null_mut(),
                                ptr::null_mut(),
                                (*post).y_stride,
                                0,
                                &mut lfi,
                            );
                        }

                        if !skip_lf {
                            vp8_loop_filter_bv(
                                y_ptr,
                                ptr::null_mut(),
                                ptr::null_mut(),
                                (*post).y_stride,
                                0,
                                &mut lfi,
                            );
                        }

                        /* don't apply across umv border */
                        if mb_row > 0 {
                            vp8_loop_filter_mbh(
                                y_ptr,
                                ptr::null_mut(),
                                ptr::null_mut(),
                                (*post).y_stride,
                                0,
                                &mut lfi,
                            );
                        }

                        if !skip_lf {
                            vp8_loop_filter_bh(
                                y_ptr,
                                ptr::null_mut(),
                                ptr::null_mut(),
                                (*post).y_stride,
                                0,
                                &mut lfi,
                            );
                        }
                    } else {
                        if mb_col > 0 {
                            vp8_loop_filter_simple_mbv(
                                y_ptr,
                                (*post).y_stride,
                                (*lfi_n).mblim[filter_level as usize].as_ptr(),
                            );
                        }

                        if !skip_lf {
                            vp8_loop_filter_simple_bv(
                                y_ptr,
                                (*post).y_stride,
                                (*lfi_n).blim[filter_level as usize].as_ptr(),
                            );
                        }

                        /* don't apply across umv border */
                        if mb_row > 0 {
                            vp8_loop_filter_simple_mbh(
                                y_ptr,
                                (*post).y_stride,
                                (*lfi_n).mblim[filter_level as usize].as_ptr(),
                            );
                        }

                        if !skip_lf {
                            vp8_loop_filter_simple_bh(
                                y_ptr,
                                (*post).y_stride,
                                (*lfi_n).blim[filter_level as usize].as_ptr(),
                            );
                        }
                    }
                }

                y_ptr = y_ptr.offset(16);
                mode_info_context = mode_info_context.offset(1); /* step to next MB */
                mb_col += 1;
            }

            y_ptr = y_ptr.offset(((*post).y_stride * 16 - (*post).y_width) as isize);
            mode_info_context = mode_info_context.offset(1); /* Skip border mb */
            mb_row += 1;
        }
    }
}

/// `vp8_loop_filter_partial_frame` (vp8/common/vp8_loopfilter.c:474).
///
/// Encoder utility: filters only ~1/8 of the frame, centered around
/// the vertical middle. The decoder never invokes it.
pub unsafe fn vp8_loop_filter_partial_frame(
    cm: *mut Vp8Common,
    mbd: *mut Macroblockd,
    default_filt_lvl: i32,
) {
    unsafe {
        let post = (*cm).frame_to_show;

        let mut y_ptr: *mut u8;
        let mut mb_row: i32;
        let mut mb_col: i32;
        let mb_cols: i32 = (*post).y_width >> 4;
        let mb_rows: i32 = (*post).y_height >> 4;

        let mut linestocopy: i32;

        let lfi_n: *mut LoopFilterInfoN = &mut (*cm).lf_info;
        let mut lfi = LoopFilterInfo {
            mblim: ptr::null(),
            blim: ptr::null(),
            lim: ptr::null(),
            hev_thr: ptr::null(),
        };

        let mut filter_level: i32;
        let frame_type: FrameType = (*cm).frame_type;

        let mut mode_info_context: *mut ModeInfo;

        // #if 0 default_filt_lvl == 0 short-circuit — omitted as in C.

        /* Initialize the loop filter for this frame. */
        vp8_loop_filter_frame_init(cm, mbd, default_filt_lvl);

        /* number of MB rows to use in partial filtering */
        linestocopy = mb_rows / PARTIAL_FRAME_FRACTION;
        linestocopy = if linestocopy != 0 {
            linestocopy << 4
        } else {
            16
        }; /* 16 lines per MB */

        /* Set up the buffer pointers; partial image starts at ~middle of frame */
        y_ptr = (*post)
            .y_buffer
            .offset((((*post).y_height >> 5) * 16 * (*post).y_stride) as isize);
        mode_info_context =
            (*cm).mi.offset((((*post).y_height >> 5) * (mb_cols + 1)) as isize);

        /* vp8_filter each macro block */
        mb_row = 0;
        while mb_row < (linestocopy >> 4) {
            mb_col = 0;
            while mb_col < mb_cols {
                let skip_lf: bool = (*mode_info_context).mbmi.mode
                    != MbPredictionMode::BPred
                    && (*mode_info_context).mbmi.mode != MbPredictionMode::SplitMv
                    && (*mode_info_context).mbmi.mb_skip_coeff;

                let mode_index: i32 =
                    (*lfi_n).mode_lf_lut[(*mode_info_context).mbmi.mode as usize] as i32;
                let seg: i32 = (*mode_info_context).mbmi.segment_id as i32;
                let ref_frame: i32 = (*mode_info_context).mbmi.ref_frame as i32;

                filter_level = (*lfi_n).lvl[seg as usize][ref_frame as usize]
                    [mode_index as usize]
                    as i32;

                if filter_level != 0 {
                    if (*cm).filter_type == LoopFilterType::Normal {
                        let hev_index: i32 = (*lfi_n).hev_thr_lut[frame_type as usize]
                            [filter_level as usize]
                            as i32;
                        lfi.mblim = (*lfi_n).mblim[filter_level as usize].as_ptr();
                        lfi.blim = (*lfi_n).blim[filter_level as usize].as_ptr();
                        lfi.lim = (*lfi_n).lim[filter_level as usize].as_ptr();
                        lfi.hev_thr = (*lfi_n).hev_thr[hev_index as usize].as_ptr();

                        if mb_col > 0 {
                            vp8_loop_filter_mbv(
                                y_ptr,
                                ptr::null_mut(),
                                ptr::null_mut(),
                                (*post).y_stride,
                                0,
                                &mut lfi,
                            );
                        }

                        if !skip_lf {
                            vp8_loop_filter_bv(
                                y_ptr,
                                ptr::null_mut(),
                                ptr::null_mut(),
                                (*post).y_stride,
                                0,
                                &mut lfi,
                            );
                        }

                        vp8_loop_filter_mbh(
                            y_ptr,
                            ptr::null_mut(),
                            ptr::null_mut(),
                            (*post).y_stride,
                            0,
                            &mut lfi,
                        );

                        if !skip_lf {
                            vp8_loop_filter_bh(
                                y_ptr,
                                ptr::null_mut(),
                                ptr::null_mut(),
                                (*post).y_stride,
                                0,
                                &mut lfi,
                            );
                        }
                    } else {
                        if mb_col > 0 {
                            vp8_loop_filter_simple_mbv(
                                y_ptr,
                                (*post).y_stride,
                                (*lfi_n).mblim[filter_level as usize].as_ptr(),
                            );
                        }

                        if !skip_lf {
                            vp8_loop_filter_simple_bv(
                                y_ptr,
                                (*post).y_stride,
                                (*lfi_n).blim[filter_level as usize].as_ptr(),
                            );
                        }

                        vp8_loop_filter_simple_mbh(
                            y_ptr,
                            (*post).y_stride,
                            (*lfi_n).mblim[filter_level as usize].as_ptr(),
                        );

                        if !skip_lf {
                            vp8_loop_filter_simple_bh(
                                y_ptr,
                                (*post).y_stride,
                                (*lfi_n).blim[filter_level as usize].as_ptr(),
                            );
                        }
                    }
                }

                y_ptr = y_ptr.offset(16);
                mode_info_context = mode_info_context.offset(1); /* step to next MB */
                mb_col += 1;
            }

            y_ptr = y_ptr.offset(((*post).y_stride * 16 - (*post).y_width) as isize);
            mode_info_context = mode_info_context.offset(1); /* Skip border mb */
            mb_row += 1;
        }
    }
}
