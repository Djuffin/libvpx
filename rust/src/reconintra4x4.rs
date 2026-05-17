//! `vp8/common/reconintra4x4.c` — per-4x4 luma intra prediction (`B_PRED`).
//!
//! Literal Rust transliteration of libvpx's `reconintra4x4.c` plus the
//! `intra_prediction_down_copy` helper that lives in `reconintra4x4.h`.
//! See `documentation/vp8_files/reconintra4x4.md` and RFC 6386 §12.2.
//!
//! The per-mode 4x4 intra-prediction kernels themselves live in
//! `vpx_dsp/intrapred.c` and are reached via the existing C symbols
//! (`vpx_*_predictor_4x4`); they are declared `extern "C"` here.

#![allow(non_upper_case_globals)]

use core::ptr::copy_nonoverlapping;

use crate::types::{BPredictionMode, Macroblockd};

// ---------------------------------------------------------------------------
// `intra_pred_fn` (reconintra4x4.c:21).
//
// The common signature shared by every per-mode 4x4 intra-prediction
// kernel exported by `vpx_dsp`.
// ---------------------------------------------------------------------------
type IntraPredFn = unsafe extern "C" fn(
    dst: *mut u8,
    stride: isize, /* ptrdiff_t */
    above: *const u8,
    left: *const u8,
);

// ---------------------------------------------------------------------------
// Foreign per-mode kernels (vpx_dsp/intrapred.c). Resolved at link time via
// the RTCD layer to whatever the host CPU supports.
// ---------------------------------------------------------------------------
extern "C" {
    fn vpx_dc_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_tm_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_ve_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_he_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_d45e_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_d135_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_d117_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_d63e_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_d153_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
    fn vpx_d207_predictor_4x4(dst: *mut u8, stride: isize, above: *const u8, left: *const u8);
}

// ---------------------------------------------------------------------------
// `static intra_pred_fn pred[10];` (reconintra4x4.c:24).
//
// File-private dispatch table indexed by `B_PREDICTION_MODE`. Written
// exactly once by `vp8_init_intra4x4_predictors_internal` (under
// `vpx_once`) and read many times from the per-MB decode loop.
// ---------------------------------------------------------------------------
static mut pred: [Option<IntraPredFn>; 10] = [None; 10];

/// `vp8_init_intra4x4_predictors_internal` (reconintra4x4.c:26).
///
/// Populates the `pred[]` dispatch table with the per-mode 4x4
/// predictor kernels from `vpx_dsp`. Called once per process from
/// `vp8_init_intra_predictors_internal` (wrapped in `vpx_once`).
pub unsafe fn vp8_init_intra4x4_predictors_internal() {
    pred[BPredictionMode::DcPred as usize] = Some(vpx_dc_predictor_4x4);
    pred[BPredictionMode::TmPred as usize] = Some(vpx_tm_predictor_4x4);
    pred[BPredictionMode::VePred as usize] = Some(vpx_ve_predictor_4x4);
    pred[BPredictionMode::HePred as usize] = Some(vpx_he_predictor_4x4);
    pred[BPredictionMode::LdPred as usize] = Some(vpx_d45e_predictor_4x4);
    pred[BPredictionMode::RdPred as usize] = Some(vpx_d135_predictor_4x4);
    pred[BPredictionMode::VrPred as usize] = Some(vpx_d117_predictor_4x4);
    pred[BPredictionMode::VlPred as usize] = Some(vpx_d63e_predictor_4x4);
    pred[BPredictionMode::HdPred as usize] = Some(vpx_d153_predictor_4x4);
    pred[BPredictionMode::HuPred as usize] = Some(vpx_d207_predictor_4x4);
}

/// `intra_prediction_down_copy` (reconintra4x4.h:19).
///
/// Pre-stages above-right samples for the right-most column of 4x4
/// sub-blocks inside the current MB by replicating the four bytes at
/// `above_right_src` downward into the three interior rows of the
/// above-right slot. Called once per `B_PRED` MB, right before the
/// per-sub-block dispatch loop.
pub unsafe fn intra_prediction_down_copy(
    xd: *mut Macroblockd,
    above_right_src: *mut u8,
) {
    let dst_stride: i32 = (*xd).dst.y_stride;
    let above_right_dst: *mut u8 = (*xd).dst.y_buffer.offset(-(dst_stride as isize)).offset(16);

    let src_ptr: *mut u32 = above_right_src as *mut u32;
    let dst_ptr0: *mut u32 =
        above_right_dst.offset((4 * dst_stride) as isize) as *mut u32;
    let dst_ptr1: *mut u32 =
        above_right_dst.offset((8 * dst_stride) as isize) as *mut u32;
    let dst_ptr2: *mut u32 =
        above_right_dst.offset((12 * dst_stride) as isize) as *mut u32;

    *dst_ptr0 = *src_ptr;
    *dst_ptr1 = *src_ptr;
    *dst_ptr2 = *src_ptr;
}

/// `vp8_intra4x4_predict` (reconintra4x4.c:39).
///
/// Assembles the L-shaped neighbour samples for one 4x4 luma sub-block
/// — the contiguous `Above[-1..7]` and `Left[0..3]` — into local
/// stack buffers and invokes the per-mode kernel that was wired up by
/// `vp8_init_intra4x4_predictors_internal`. Stateless: consults
/// neither `MACROBLOCKD` nor the frame.
pub unsafe fn vp8_intra4x4_predict(
    above: *mut u8,
    yleft: *mut u8,
    left_stride: i32,
    b_mode: BPredictionMode,
    dst: *mut u8,
    dst_stride: i32,
    top_left: u8,
) {
    /* Power PC implementation uses "vec_vsx_ld" to read 16 bytes from
       Above (aka, Aboveb + 4). Play it safe by reserving enough stack
       space here. Similary for "Left". */
    // Generic (non-VSX) build: 12 bytes is enough.
    let mut aboveb: [u8; 12] = [0; 12];
    let above_buf: *mut u8 = aboveb.as_mut_ptr().offset(4);
    // Generic (non-NEON, non-VSX) build: Left[4].
    let mut left: [u8; 4] = [0; 4];

    left[0] = *yleft.offset(0);
    left[1] = *yleft.offset(left_stride as isize);
    left[2] = *yleft.offset((2 * left_stride) as isize);
    left[3] = *yleft.offset((3 * left_stride) as isize);
    copy_nonoverlapping(above, above_buf, 8);
    *above_buf.offset(-1) = top_left;

    (pred[b_mode as usize].unwrap())(dst, dst_stride as isize, above_buf, left.as_ptr());
}
