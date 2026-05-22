//! Whole-macroblock intra prediction (`vp8/common/reconintra.c`).
//!
//! Three public entry points:
//!   * [`vp8_build_intra_predictors_mby_s`] — 16x16 luma predictor.
//!   * [`vp8_build_intra_predictors_mbuv_s`] — 8x8 chroma (U and V).
//!   * [`vp8_init_intra_predictors`] — once-per-process dispatch-table
//!     populator.
//!
//! The pixel kernels live in `vpx_dsp/intrapred.c` and are reached by raw
//! function pointer through the `pred` / `dc_pred` tables.

#![allow(non_upper_case_globals)]

use crate::types::{Macroblockd, MbPredictionMode, ModeInfo};

use crate::reconintra4x4::vp8_init_intra4x4_predictors_internal;
use crate::vpx_dsp_rtcd::{
    vpx_dc_128_predictor_8x8, vpx_dc_128_predictor_16x16, vpx_dc_left_predictor_8x8,
    vpx_dc_left_predictor_16x16, vpx_dc_predictor_8x8, vpx_dc_predictor_16x16,
    vpx_dc_top_predictor_8x8, vpx_dc_top_predictor_16x16, vpx_h_predictor_8x8,
    vpx_h_predictor_16x16, vpx_tm_predictor_8x8, vpx_tm_predictor_16x16, vpx_v_predictor_8x8,
    vpx_v_predictor_16x16,
};
use crate::vpx_ports::once;

/// Size token used to index the `pred` / `dc_pred` tables.
/// Mirrors the anonymous `enum { SIZE_16, SIZE_8, NUM_SIZES }` in
/// `reconintra.c`.
const SIZE_16: usize = 0;
const SIZE_8: usize = 1;
const NUM_SIZES: usize = 2;

/// `intra_pred_fn` typedef — kernel signature shared by every concrete
/// VP8 intra predictor in `vpx_dsp/intrapred.c`.
type IntraPredFn =
    unsafe extern "C" fn(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);

/// `static intra_pred_fn pred[4][NUM_SIZES]`.
/// Slot `[DC_PRED][*]` is intentionally left as `None`; the build
/// functions route `DC_PRED` through `DC_PRED_TBL` instead.
static mut pred: [[Option<IntraPredFn>; NUM_SIZES]; 4] = [[None; NUM_SIZES]; 4];

/// `static intra_pred_fn dc_pred[2][2][NUM_SIZES]`.
/// Indexed `[left_available][up_available][size]`.
static mut dc_pred: [[[Option<IntraPredFn>; NUM_SIZES]; 2]; 2] = [[[None; NUM_SIZES]; 2]; 2];

/// `vp8_init_intra_predictors_internal` (vp8/common/reconintra.c:32).
///
/// Populates both dispatch tables for the two whole-MB sizes (16 and 8)
/// and chains into the per-4x4 initializer.
unsafe fn vp8_init_intra_predictors_internal() {
    // INIT_SIZE(16);
    pred[MbPredictionMode::VPred as usize][SIZE_16] = Some(vpx_v_predictor_16x16);
    pred[MbPredictionMode::HPred as usize][SIZE_16] = Some(vpx_h_predictor_16x16);
    pred[MbPredictionMode::TmPred as usize][SIZE_16] = Some(vpx_tm_predictor_16x16);

    dc_pred[0][0][SIZE_16] = Some(vpx_dc_128_predictor_16x16);
    dc_pred[0][1][SIZE_16] = Some(vpx_dc_top_predictor_16x16);
    dc_pred[1][0][SIZE_16] = Some(vpx_dc_left_predictor_16x16);
    dc_pred[1][1][SIZE_16] = Some(vpx_dc_predictor_16x16);

    // INIT_SIZE(8);
    pred[MbPredictionMode::VPred as usize][SIZE_8] = Some(vpx_v_predictor_8x8);
    pred[MbPredictionMode::HPred as usize][SIZE_8] = Some(vpx_h_predictor_8x8);
    pred[MbPredictionMode::TmPred as usize][SIZE_8] = Some(vpx_tm_predictor_8x8);

    dc_pred[0][0][SIZE_8] = Some(vpx_dc_128_predictor_8x8);
    dc_pred[0][1][SIZE_8] = Some(vpx_dc_top_predictor_8x8);
    dc_pred[1][0][SIZE_8] = Some(vpx_dc_left_predictor_8x8);
    dc_pred[1][1][SIZE_8] = Some(vpx_dc_predictor_8x8);

    vp8_init_intra4x4_predictors_internal();
}

/// `vp8_build_intra_predictors_mby_s` (vp8/common/reconintra.c:48).
///
/// Writes the 16x16 luma intra predictor into `ypred_ptr` for the
/// macroblock described by `x`. Caller must have set
/// `x->left_available` / `x->up_available` and `x->mode_info_context`.
pub fn vp8_build_intra_predictors_mby_s(
    x: &Macroblockd,
    mi: &ModeInfo,
    yabove_row: *mut u8,
    yleft: *mut u8,
    left_stride: i32,
    ypred_ptr: *mut u8,
    y_stride: i32,
) {
    let mode: MbPredictionMode = mi.mbmi.mode;
    // DECLARE_ALIGNED(16, uint8_t, yleft_col[16])
    #[repr(align(16))]
    struct Aligned16([u8; 16]);
    let mut yleft_col_buf = Aligned16([0u8; 16]);
    let yleft_col: *mut u8 = yleft_col_buf.0.as_mut_ptr();

    // SAFETY: `yleft` is the dst left-neighbor column pointer; we read 16
    // bytes at `(0..16)*left_stride`, which stays within the dst plane.
    // The dispatch tables are populated before this is ever called.
    unsafe {
        for i in 0..16i32 {
            *yleft_col.offset(i as isize) = *yleft.offset((i * left_stride) as isize);
        }

        let fn_: IntraPredFn = if mode == MbPredictionMode::DcPred {
            dc_pred[x.left_available as usize][x.up_available as usize][SIZE_16].unwrap()
        } else {
            pred[mode as usize][SIZE_16].unwrap()
        };

        fn_(ypred_ptr, y_stride as isize, yabove_row, yleft_col);
    }
}

/// `vp8_build_intra_predictors_mbuv_s` (vp8/common/reconintra.c:69).
///
/// Writes the 8x8 U and V intra predictors into `upred_ptr` /
/// `vpred_ptr`. U and V always share the single `uv_mode` selector
/// per RFC 6386 §13.4.
pub fn vp8_build_intra_predictors_mbuv_s(
    x: &Macroblockd,
    mi: &ModeInfo,
    uabove_row: *mut u8,
    vabove_row: *mut u8,
    uleft: *mut u8,
    vleft: *mut u8,
    left_stride: i32,
    upred_ptr: *mut u8,
    vpred_ptr: *mut u8,
    pred_stride: i32,
) {
    let uvmode: MbPredictionMode = mi.mbmi.uv_mode;
    // 16 bytes (not 8): the C source reserves 16 under `#if HAVE_VSX` so
    // VSX kernels can load full 128-bit vectors.
    let mut uleft_col: [u8; 16] = [0; 16];
    let mut vleft_col: [u8; 16] = [0; 16];

    // SAFETY: uleft/vleft are dst-plane left-neighbor column pointers;
    // we read 8 stride-spaced bytes from each. The predictor dispatch
    // table is initialized at decoder startup.
    unsafe {
        for i in 0..8i32 {
            uleft_col[i as usize] = *uleft.offset((i * left_stride) as isize);
            vleft_col[i as usize] = *vleft.offset((i * left_stride) as isize);
        }

        let fn_: IntraPredFn = if uvmode == MbPredictionMode::DcPred {
            dc_pred[x.left_available as usize][x.up_available as usize][SIZE_8].unwrap()
        } else {
            pred[uvmode as usize][SIZE_8].unwrap()
        };

        fn_(
            upred_ptr,
            pred_stride as isize,
            uabove_row,
            uleft_col.as_ptr(),
        );
        fn_(
            vpred_ptr,
            pred_stride as isize,
            vabove_row,
            vleft_col.as_ptr(),
        );
    }
}

/// `vp8_init_intra_predictors` (vp8/common/reconintra.c:102).
///
/// Process-wide one-shot dispatch-table populator.
pub unsafe fn vp8_init_intra_predictors() {
    once(vp8_init_intra_predictors_internal);
}
