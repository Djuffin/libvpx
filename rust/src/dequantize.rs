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

use crate::idctllm::vp8_short_idct4x4llm_c;

// ---------------------------------------------------------------------------
// Public kernels
// ---------------------------------------------------------------------------

/// `vp8_dequantize_b_c` — vp8/common/dequantize.c:16.
///
/// Multiplies the 16 quantized coefficients in `qcoeff` by the
/// 16-element dequantizer table `DQC`, writing the result to
/// `dqcoeff`. Used only for the Y2 second-order block (the rest of
/// the MB uses the fused [`vp8_dequant_idct_add_c`] form).
///
/// C signature is `vp8_dequantize_b_c(BLOCKD *d, short *DQC)`; the
/// `BLOCKD` argument's only purpose was to carry `d->qcoeff` and
/// `d->dqcoeff` — which in this port are just `xd.{qcoeff,dqcoeff} +
/// 24 * 16` (the Y2 block's slot). We take them directly to avoid the
/// `Blockd` indirection.
pub unsafe fn vp8_dequantize_b_c(qcoeff: *mut i16, dqcoeff: *mut i16, DQC: *mut i16) {
    for i in 0..16isize {
        // `Q[i] * DQC[i]` is promoted to `int` in C and truncated back
        // to `short` on store — match with a wrapping i32 multiply.
        let prod = (*qcoeff.offset(i) as i32).wrapping_mul(*DQC.offset(i) as i32);
        *dqcoeff.offset(i) = prod as i16;
    }
}

/// `vp8_dequant_idct_add_c` — vp8/common/dequantize.c:26.
///
/// Fused dequant + 4x4 inverse DCT + accumulate. Scales `input[0..15]`
/// in place by `dq[0..15]`, runs `vp8_short_idct4x4llm_c` to add the
/// residual onto `dest` (stride `stride`), then zeros the 16-short
/// `input` buffer (32 bytes) so the next pass over this MB starts
/// from a clean slate.
pub unsafe fn vp8_dequant_idct_add_c(input: *mut i16, dq: *mut i16, dest: *mut u8, stride: i32) {
    for i in 0..16isize {
        let prod = (*dq.offset(i) as i32).wrapping_mul(*input.offset(i) as i32);
        *input.offset(i) = prod as i16;
    }

    vp8_short_idct4x4llm_c(input, dest, stride, dest, stride);

    ptr::write_bytes(input as *mut u8, 0, 32);
}
