//! Whole-macroblock intra prediction (`vp8/common/reconintra.c`).
//!
//! Literal Rust transliteration of libvpx's `reconintra.c`. Three public
//! entry points:
//!   * [`vp8_build_intra_predictors_mby_s`] — 16x16 luma predictor.
//!   * [`vp8_build_intra_predictors_mbuv_s`] — 8x8 chroma (U and V).
//!   * [`vp8_init_intra_predictors`] — once-per-process dispatch-table
//!     populator.
//!
//! The actual pixel kernels live in `vpx_dsp/intrapred.c` and are
//! reached by raw function pointer through the `pred` / `dc_pred`
//! tables. Those kernels are declared `extern "Rust"` until their host
//! module is translated.

#![allow(non_upper_case_globals)]

use crate::types::{Macroblockd, MbPredictionMode};

// ---------------------------------------------------------------------------
// extern dependencies (translated in other modules)
// ---------------------------------------------------------------------------

unsafe extern "Rust" {
    /// One-shot initializer primitive (`vpx_ports/vpx_once.h`).
    fn once(func: unsafe extern "Rust" fn());

    /// Per-4x4 intra predictor dispatch-table populator
    /// (`vp8/common/reconintra4x4.c`). The whole-MB initializer chains
    /// into this so a single `once()` covers both.
    fn vp8_init_intra4x4_predictors_internal();

    // VP8 intra prediction kernels — generated through `vpx_dsp_rtcd.h`
    // from `vpx_dsp/intrapred.c`. All share the `IntraPredFn` shape.
    fn vpx_v_predictor_16x16(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_h_predictor_16x16(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_tm_predictor_16x16(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_dc_predictor_16x16(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_dc_top_predictor_16x16(
        dst: *mut u8,
        stride: isize,
        above: *const u8,
        left: *const u8,
    );
    fn vpx_dc_left_predictor_16x16(
        dst: *mut u8,
        stride: isize,
        above: *const u8,
        left: *const u8,
    );
    fn vpx_dc_128_predictor_16x16(
        dst: *mut u8,
        stride: isize,
        above: *const u8,
        left: *const u8,
    );

    fn vpx_v_predictor_8x8(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_h_predictor_8x8(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_tm_predictor_8x8(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_dc_predictor_8x8(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_dc_top_predictor_8x8(
        dst: *mut u8,
        stride: isize,
        above: *const u8,
        left: *const u8,
    );
    fn vpx_dc_left_predictor_8x8(
        dst: *mut u8,
        stride: isize,
        above: *const u8,
        left: *const u8,
    );
    fn vpx_dc_128_predictor_8x8(
        dst: *mut u8,
        stride: isize,
        above: *const u8,
        left: *const u8,
    );
}

// ---------------------------------------------------------------------------
// File-local enums and dispatch tables
// ---------------------------------------------------------------------------

/// Size token used to index the `pred` / `dc_pred` tables.
/// Mirrors the anonymous `enum { SIZE_16, SIZE_8, NUM_SIZES }` in
/// `reconintra.c`.
const SIZE_16: usize = 0;
const SIZE_8: usize = 1;
const NUM_SIZES: usize = 2;

/// `intra_pred_fn` typedef — kernel signature shared by every concrete
/// VP8 intra predictor in `vpx_dsp/intrapred.c`.
type IntraPredFn =
    unsafe extern "Rust" fn(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);

/// `static intra_pred_fn pred[4][NUM_SIZES]`.
/// Slot `[DC_PRED][*]` is intentionally left as `None`; the build
/// functions route `DC_PRED` through `DC_PRED_TBL` instead.
static mut pred: [[Option<IntraPredFn>; NUM_SIZES]; 4] =
    [[None; NUM_SIZES]; 4];

/// `static intra_pred_fn dc_pred[2][2][NUM_SIZES]`.
/// Indexed `[left_available][up_available][size]`.
static mut dc_pred: [[[Option<IntraPredFn>; NUM_SIZES]; 2]; 2] =
    [[[None; NUM_SIZES]; 2]; 2];

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

/// `static void vp8_init_intra_predictors_internal(void)`
/// (vp8/common/reconintra.c:32).
///
/// Populates both file-local dispatch tables for the two whole-MB sizes
/// (16 and 8) and chains into the per-4x4 initializer so a single
/// `once()` call covers both layers of VP8 intra prediction.
unsafe extern "Rust" fn vp8_init_intra_predictors_internal() {
    unsafe {
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
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

/// `vp8_build_intra_predictors_mby_s` (vp8/common/reconintra.c:48).
///
/// Writes the 16x16 luma intra predictor into `ypred_ptr` for the
/// macroblock described by `x`. Caller must have set
/// `x->left_available` / `x->up_available` and `x->mode_info_context`.
pub unsafe fn vp8_build_intra_predictors_mby_s(
    x: *mut Macroblockd,
    yabove_row: *mut u8,
    yleft: *mut u8,
    left_stride: i32,
    ypred_ptr: *mut u8,
    y_stride: i32,
) {
    unsafe {
        let mode: MbPredictionMode = (*(*x).mode_info_context).mbmi.mode;
        // DECLARE_ALIGNED(16, uint8_t, yleft_col[16])
        #[repr(align(16))]
        struct Aligned16([u8; 16]);
        let mut yleft_col_buf = Aligned16([0u8; 16]);
        let yleft_col: *mut u8 = yleft_col_buf.0.as_mut_ptr();
        let mut i: i32;
        let fn_: IntraPredFn;

        i = 0;
        while i < 16 {
            *yleft_col.offset(i as isize) = *yleft.offset((i * left_stride) as isize);
            i += 1;
        }

        if mode == MbPredictionMode::DcPred {
            fn_ = dc_pred[(*x).left_available as usize][(*x).up_available as usize][SIZE_16]
                .unwrap();
        } else {
            fn_ = pred[mode as usize][SIZE_16].unwrap();
        }

        fn_(ypred_ptr, y_stride as isize, yabove_row, yleft_col);
    }
}

/// `vp8_build_intra_predictors_mbuv_s` (vp8/common/reconintra.c:69).
///
/// Writes the 8x8 U and V intra predictors into `upred_ptr` /
/// `vpred_ptr`. U and V always share the single `uv_mode` selector
/// per RFC 6386 §13.4.
pub unsafe fn vp8_build_intra_predictors_mbuv_s(
    x: *mut Macroblockd,
    uabove_row: *mut u8,
    vabove_row: *mut u8,
    uleft: *mut u8,
    vleft: *mut u8,
    left_stride: i32,
    upred_ptr: *mut u8,
    vpred_ptr: *mut u8,
    pred_stride: i32,
) {
    unsafe {
        let uvmode: MbPredictionMode = (*(*x).mode_info_context).mbmi.uv_mode;
        // The C source uses `#if HAVE_VSX` to reserve 16 bytes on PowerPC
        // VSX builds (which load full 128-bit vectors). We unconditionally
        // reserve 16 bytes — minor stack overhead, no UB on any backend.
        let mut uleft_col: [u8; 16] = [0; 16];
        let mut vleft_col: [u8; 16] = [0; 16];
        let mut i: i32;
        let fn_: IntraPredFn;

        i = 0;
        while i < 8 {
            uleft_col[i as usize] = *uleft.offset((i * left_stride) as isize);
            vleft_col[i as usize] = *vleft.offset((i * left_stride) as isize);
            i += 1;
        }

        if uvmode == MbPredictionMode::DcPred {
            fn_ = dc_pred[(*x).left_available as usize][(*x).up_available as usize][SIZE_8]
                .unwrap();
        } else {
            fn_ = pred[uvmode as usize][SIZE_8].unwrap();
        }

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
/// Process-wide one-shot dispatch-table populator. Wraps the internal
/// initializer in `once()` so multiple decoder instances and threads
/// share the same populated tables safely.
pub unsafe fn vp8_init_intra_predictors() {
    unsafe {
        once(vp8_init_intra_predictors_internal);
    }
}
