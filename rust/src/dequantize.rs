//! Inverse-quantization primitives — literal translation of
//! `vp8/common/dequantize.c`.
//!
//! Two reference C kernels live here:
//!   * [`vp8_dequantize_b_c`] — standalone dequant for the Y2 block.
//!   * [`vp8_dequant_idct_add_c`] — fused dequant + IDCT + accumulate.
//!
//! Both are dispatched in the C build through `vp8_rtcd.h`; the SIMD
//! variants must remain bit-exact with these references.

#![allow(non_snake_case)]

use core::ptr;

use crate::types::Blockd;

// ---------------------------------------------------------------------------
// extern dependencies (translated in other modules)
// ---------------------------------------------------------------------------

extern "Rust" {
    /// `vp8_short_idct4x4llm_c` (vp8/common/idctllm.c) — reference 4x4
    /// inverse DCT + accumulate. Reads coefficients from `input`,
    /// reads the predictor from `pred` (stride `pitch`), writes the
    /// reconstructed pels to `dst` (stride `stride`).
    fn vp8_short_idct4x4llm_c(
        input: *mut i16,
        pred: *mut u8,
        pitch: i32,
        dst: *mut u8,
        stride: i32,
    );
}

// ---------------------------------------------------------------------------
// Public kernels
// ---------------------------------------------------------------------------

/// `vp8_dequantize_b_c` — vp8/common/dequantize.c:16.
///
/// Multiplies the 16 quantized coefficients held in `d->qcoeff` by the
/// 16-element dequantizer table `DQC`, writing the result to
/// `d->dqcoeff`. Used only for the Y2 second-order block (the rest of
/// the MB uses the fused [`vp8_dequant_idct_add_c`] form).
pub unsafe fn vp8_dequantize_b_c(d: *mut Blockd, DQC: *mut i16) {
    let mut i: i32;
    let DQ: *mut i16 = (*d).dqcoeff;
    let Q: *mut i16 = (*d).qcoeff;

    i = 0;
    while i < 16 {
        // `Q[i] * DQC[i]` is promoted to `int` in C and truncated back
        // to `short` on store — match with a wrapping i32 multiply.
        let prod = (*Q.offset(i as isize) as i32)
            .wrapping_mul(*DQC.offset(i as isize) as i32);
        *DQ.offset(i as isize) = prod as i16;
        i += 1;
    }
}

/// `vp8_dequant_idct_add_c` — vp8/common/dequantize.c:26.
///
/// Fused dequant + 4x4 inverse DCT + accumulate. Scales `input[0..15]`
/// in place by `dq[0..15]`, runs `vp8_short_idct4x4llm_c` to add the
/// residual onto `dest` (stride `stride`), then zeros the 16-short
/// `input` buffer (32 bytes) so the next pass over this MB starts
/// from a clean slate.
pub unsafe fn vp8_dequant_idct_add_c(
    input: *mut i16,
    dq: *mut i16,
    dest: *mut u8,
    stride: i32,
) {
    let mut i: i32;

    i = 0;
    while i < 16 {
        let prod = (*dq.offset(i as isize) as i32)
            .wrapping_mul(*input.offset(i as isize) as i32);
        *input.offset(i as isize) = prod as i16;
        i += 1;
    }

    vp8_short_idct4x4llm_c(input, dest, stride, dest, stride);

    ptr::write_bytes(input as *mut u8, 0, 32);
}
