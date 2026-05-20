//! 4x4 inverse transforms — literal translation of
//! `vp8/common/idctllm.c`.
//!
//! Four reference C kernels live here:
//!   * [`vp8_short_idct4x4llm_c`] — full 4x4 IDCT + clipped accumulate
//!     into a predictor. RFC 6386 §14.2 + §14.4.
//!   * [`vp8_dc_only_idct_add_c`] — DC-only IDCT-add fast path.
//!   * [`vp8_short_inv_walsh4x4_c`] — full inverse WHT for the Y2 block,
//!     scattering recovered DCs into `xd->qcoeff`. RFC 6386 §14.3.
//!   * [`vp8_short_inv_walsh4x4_1_c`] — DC-only WHT fast path.
//!
//! All four are dispatched in the C build through `vp8_rtcd.h`; the SIMD
//! variants must remain bit-exact with these references.
//!
//! Notes (from the C source):
//!
//! This implementation makes use of 16 bit fixed point verio of two
//! multiply constants:
//!         1.   sqrt(2) * cos (pi/8)
//!         2.   sqrt(2) * sin (pi/8)
//! Becuase the first constant is bigger than 1, to maintain the same 16
//! bit fixed point precision as the second one, we use a trick of
//!         x * a = x + x*(a-1)
//! so
//!         x * sqrt(2) * cos (pi/8) = x + x * (sqrt(2) *cos(pi/8)-1).

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

// ---------------------------------------------------------------------------
// Normative multiply constants (RFC 6386 §14.2).
// ---------------------------------------------------------------------------

/// `round((cos(π/8) · √2 − 1) · 2^16)`. Encoded as `a-1` because
/// `cos(π/8)·√2 ≈ 1.3066` overflows a Q16 representation; reconstruct
/// the full multiplication as `x + ((x * cospi8sqrt2minus1) >> 16)`.
const cospi8sqrt2minus1: i32 = 20091;
/// `round(sin(π/8) · √2 · 2^16)`. Used directly as
/// `(x * sinpi8sqrt2) >> 16`.
const sinpi8sqrt2: i32 = 35468;

// ---------------------------------------------------------------------------
// Full 4x4 IDCT + predictor summation.
// ---------------------------------------------------------------------------

/// `vp8_short_idct4x4llm_c` — reference 4x4 inverse DCT followed by the
/// clip-and-add accumulate of RFC 6386 §14.4.

pub extern "C" fn vp8_short_idct4x4llm_c(
    input: *mut i16,
    pred_ptr: *mut u8,
    pred_stride: i32,
    dst_ptr: *mut u8,
    dst_stride: i32,
) {
    // SAFETY: input is a 16-i16 coefficient block; pred_ptr/dst_ptr point
    // to 4x4 pixel regions in (possibly identical) Yv12 planes.
    unsafe {
    // Build a bounded view of the input coefficients (fixed 16 shorts).
    // We materialize the predictor into a local 4x4 buffer before
    // touching `dst` because callers commonly pass the same buffer as
    // both `pred_ptr` and `dst_ptr` (in-place IDCT-add); constructing
    // overlapping `&[u8]` and `&mut [u8]` slices over the same memory
    // would violate Rust's aliasing rules.
    let input: &[i16; 16] = &*(input as *const [i16; 16]);

    let mut pred: [u8; 16] = [0; 16];
    for r in 0..4 {
        let row = pred_ptr.offset((r as isize) * (pred_stride as isize));
        for c in 0..4 {
            pred[r * 4 + c] = *row.offset(c as isize);
        }
    }

    let mut output: [i16; 16] = [0; 16];

    // Column pass: input[col + 4*row] -> output[col + 4*row].
    for col in 0..4 {
        let i0 = input[col] as i32;
        let i4 = input[col + 4] as i32;
        let i8_ = input[col + 8] as i32;
        let i12 = input[col + 12] as i32;

        let a1 = i0 + i8_;
        let b1 = i0 - i8_;

        let temp1 = (i4 * sinpi8sqrt2) >> 16;
        let temp2 = i12 + ((i12 * cospi8sqrt2minus1) >> 16);
        let c1 = temp1 - temp2;

        let temp1 = i4 + ((i4 * cospi8sqrt2minus1) >> 16);
        let temp2 = (i12 * sinpi8sqrt2) >> 16;
        let d1 = temp1 + temp2;

        output[col] = (a1 + d1) as i16;
        output[col + 12] = (a1 - d1) as i16;
        output[col + 4] = (b1 + c1) as i16;
        output[col + 8] = (b1 - c1) as i16;
    }

    // Row pass: in-place on `output`.
    for row in 0..4 {
        let base = row * 4;
        let r0 = output[base] as i32;
        let r1 = output[base + 1] as i32;
        let r2 = output[base + 2] as i32;
        let r3 = output[base + 3] as i32;

        let a1 = r0 + r2;
        let b1 = r0 - r2;

        let temp1 = (r1 * sinpi8sqrt2) >> 16;
        let temp2 = r3 + ((r3 * cospi8sqrt2minus1) >> 16);
        let c1 = temp1 - temp2;

        let temp1 = r1 + ((r1 * cospi8sqrt2minus1) >> 16);
        let temp2 = (r3 * sinpi8sqrt2) >> 16;
        let d1 = temp1 + temp2;

        output[base] = ((a1 + d1 + 4) >> 3) as i16;
        output[base + 3] = ((a1 - d1 + 4) >> 3) as i16;
        output[base + 1] = ((b1 + c1 + 4) >> 3) as i16;
        output[base + 2] = ((b1 - c1 + 4) >> 3) as i16;
    }

    // Clip-and-add into `dst` from the saved predictor.
    for r in 0..4 {
        let row = dst_ptr.offset((r as isize) * (dst_stride as isize));
        for c in 0..4 {
            let a = output[r * 4 + c] as i32 + pred[r * 4 + c] as i32;
            *row.offset(c as isize) = a.clamp(0, 255) as u8;
        }
    }
    }
}

// ---------------------------------------------------------------------------
// DC-only IDCT-add fast path.
// ---------------------------------------------------------------------------

/// `vp8_dc_only_idct_add_c` — fast path taken when only the DC
/// coefficient is non-zero (`eobs[block] == 1`). The DC value is
/// pre-multiplied by `dq[0]` by the caller.

pub extern "C" fn vp8_dc_only_idct_add_c(
    input_dc: i16,
    pred_ptr: *mut u8,
    pred_stride: i32,
    dst_ptr: *mut u8,
    dst_stride: i32,
) {
    let a1: i32 = ((input_dc as i32) + 4) >> 3;

    // SAFETY: pred_ptr and dst_ptr point to 4x4 pixel regions reachable
    // at offsets (0..4)*stride + (0..4). Callers may pass pred==dst.
    unsafe {
        for r in 0..4 {
            let pred_row = pred_ptr.offset((r as isize) * (pred_stride as isize));
            let dst_row = dst_ptr.offset((r as isize) * (dst_stride as isize));
            let mut row: [u8; 4] = [0; 4];
            for c in 0..4 {
                row[c] = *pred_row.offset(c as isize);
            }
            for c in 0..4 {
                let a = a1 + row[c] as i32;
                *dst_row.offset(c as isize) = a.clamp(0, 255) as u8;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Full inverse Walsh–Hadamard for the Y2 block (RFC 6386 §14.3).
// ---------------------------------------------------------------------------

/// `vp8_short_inv_walsh4x4_c` — inverse WHT for the macroblock Y2 block,
/// scattering the 16 recovered luma DCs into the DC slot of each Y
/// residual block (`mb_dqcoeff[i * 16]`).

pub extern "C" fn vp8_short_inv_walsh4x4_c(input: *mut i16, mb_dqcoeff: *mut i16) {
    // SAFETY: input is a 16-i16 block (the Y2 dqcoeff slot); mb_dqcoeff
    // is qcoeff[0..16*16] addressed at stride 16.
    unsafe {
    // `input` is a fixed 16-short block; snapshot it into a local
    // [i16; 16] up front and operate purely on safe arrays.
    let input: &[i16; 16] = &*(input as *const [i16; 16]);
    let mut output: [i16; 16] = [0; 16];

    // Column pass.
    for col in 0..4 {
        let i0 = input[col] as i32;
        let i4 = input[col + 4] as i32;
        let i8_ = input[col + 8] as i32;
        let i12 = input[col + 12] as i32;

        let a1 = i0 + i12;
        let b1 = i4 + i8_;
        let c1 = i4 - i8_;
        let d1 = i0 - i12;

        output[col] = (a1 + b1) as i16;
        output[col + 4] = (c1 + d1) as i16;
        output[col + 8] = (a1 - b1) as i16;
        output[col + 12] = (d1 - c1) as i16;
    }

    // Row pass, in-place.
    for row in 0..4 {
        let base = row * 4;
        let r0 = output[base] as i32;
        let r1 = output[base + 1] as i32;
        let r2 = output[base + 2] as i32;
        let r3 = output[base + 3] as i32;

        let a1 = r0 + r3;
        let b1 = r1 + r2;
        let c1 = r1 - r2;
        let d1 = r0 - r3;

        let a2 = a1 + b1;
        let b2 = c1 + d1;
        let c2 = a1 - b1;
        let d2 = d1 - c1;

        output[base] = ((a2 + 3) >> 3) as i16;
        output[base + 1] = ((b2 + 3) >> 3) as i16;
        output[base + 2] = ((c2 + 3) >> 3) as i16;
        output[base + 3] = ((d2 + 3) >> 3) as i16;
    }

    // Scatter the 16 recovered DCs into the DC slot of each Y residual
    // block (stride 16 shorts).
    for i in 0..16 {
        *mb_dqcoeff.offset((i * 16) as isize) = output[i as usize];
    }
    }
}

// ---------------------------------------------------------------------------
// DC-only inverse Walsh–Hadamard.
// ---------------------------------------------------------------------------

/// `vp8_short_inv_walsh4x4_1_c` — fast path when only the WHT DC is
/// non-zero; every Y block gets the same recovered DC.

pub extern "C" fn vp8_short_inv_walsh4x4_1_c(input: *mut i16, mb_dqcoeff: *mut i16) {
    // SAFETY: input points to the Y2 dqcoeff DC slot; mb_dqcoeff[0..16*16]
    // is the qcoeff array addressed at stride 16.
    unsafe {
        let a1: i32 = ((*input as i32) + 3) >> 3;
        for i in 0..16 {
            *mb_dqcoeff.offset((i * 16) as isize) = a1 as i16;
        }
    }
}
