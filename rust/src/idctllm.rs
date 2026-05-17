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
#[no_mangle]
pub unsafe extern "C" fn vp8_short_idct4x4llm_c(
    input: *mut i16,
    mut pred_ptr: *mut u8,
    pred_stride: i32,
    mut dst_ptr: *mut u8,
    dst_stride: i32,
) {
    let mut a1: i32;
    let mut b1: i32;
    let mut c1: i32;
    let mut d1: i32;
    let mut output: [i16; 16] = [0; 16];
    let mut ip: *mut i16 = input;
    let mut op: *mut i16 = output.as_mut_ptr();
    let mut temp1: i32;
    let mut temp2: i32;
    let shortpitch: i32 = 4;

    let mut i: i32 = 0;
    while i < 4 {
        a1 = (*ip.offset(0) as i32) + (*ip.offset(8) as i32);
        b1 = (*ip.offset(0) as i32) - (*ip.offset(8) as i32);

        temp1 = ((*ip.offset(4) as i32) * sinpi8sqrt2) >> 16;
        temp2 = (*ip.offset(12) as i32)
            + (((*ip.offset(12) as i32) * cospi8sqrt2minus1) >> 16);
        c1 = temp1 - temp2;

        temp1 = (*ip.offset(4) as i32)
            + (((*ip.offset(4) as i32) * cospi8sqrt2minus1) >> 16);
        temp2 = ((*ip.offset(12) as i32) * sinpi8sqrt2) >> 16;
        d1 = temp1 + temp2;

        *op.offset((shortpitch * 0) as isize) = (a1 + d1) as i16;
        *op.offset((shortpitch * 3) as isize) = (a1 - d1) as i16;

        *op.offset((shortpitch * 1) as isize) = (b1 + c1) as i16;
        *op.offset((shortpitch * 2) as isize) = (b1 - c1) as i16;

        ip = ip.offset(1);
        op = op.offset(1);
        i += 1;
    }

    ip = output.as_mut_ptr();
    op = output.as_mut_ptr();

    let mut i: i32 = 0;
    while i < 4 {
        a1 = (*ip.offset(0) as i32) + (*ip.offset(2) as i32);
        b1 = (*ip.offset(0) as i32) - (*ip.offset(2) as i32);

        temp1 = ((*ip.offset(1) as i32) * sinpi8sqrt2) >> 16;
        temp2 = (*ip.offset(3) as i32)
            + (((*ip.offset(3) as i32) * cospi8sqrt2minus1) >> 16);
        c1 = temp1 - temp2;

        temp1 = (*ip.offset(1) as i32)
            + (((*ip.offset(1) as i32) * cospi8sqrt2minus1) >> 16);
        temp2 = ((*ip.offset(3) as i32) * sinpi8sqrt2) >> 16;
        d1 = temp1 + temp2;

        *op.offset(0) = ((a1 + d1 + 4) >> 3) as i16;
        *op.offset(3) = ((a1 - d1 + 4) >> 3) as i16;

        *op.offset(1) = ((b1 + c1 + 4) >> 3) as i16;
        *op.offset(2) = ((b1 - c1 + 4) >> 3) as i16;

        ip = ip.offset(shortpitch as isize);
        op = op.offset(shortpitch as isize);
        i += 1;
    }

    let mut ip: *mut i16 = output.as_mut_ptr();
    let mut r: i32 = 0;
    while r < 4 {
        let mut c: i32 = 0;
        while c < 4 {
            let mut a: i32 =
                (*ip.offset(c as isize) as i32) + (*pred_ptr.offset(c as isize) as i32);

            if a < 0 {
                a = 0;
            }

            if a > 255 {
                a = 255;
            }

            *dst_ptr.offset(c as isize) = a as u8;
            c += 1;
        }
        ip = ip.offset(4);
        dst_ptr = dst_ptr.offset(dst_stride as isize);
        pred_ptr = pred_ptr.offset(pred_stride as isize);
        r += 1;
    }
}

// ---------------------------------------------------------------------------
// DC-only IDCT-add fast path.
// ---------------------------------------------------------------------------

/// `vp8_dc_only_idct_add_c` — fast path taken when only the DC
/// coefficient is non-zero (`eobs[block] == 1`). The DC value is
/// pre-multiplied by `dq[0]` by the caller.
#[no_mangle]
pub unsafe extern "C" fn vp8_dc_only_idct_add_c(
    input_dc: i16,
    mut pred_ptr: *mut u8,
    pred_stride: i32,
    mut dst_ptr: *mut u8,
    dst_stride: i32,
) {
    let a1: i32 = ((input_dc as i32) + 4) >> 3;

    let mut r: i32 = 0;
    while r < 4 {
        let mut c: i32 = 0;
        while c < 4 {
            let mut a: i32 = a1 + (*pred_ptr.offset(c as isize) as i32);

            if a < 0 {
                a = 0;
            }

            if a > 255 {
                a = 255;
            }

            *dst_ptr.offset(c as isize) = a as u8;
            c += 1;
        }

        dst_ptr = dst_ptr.offset(dst_stride as isize);
        pred_ptr = pred_ptr.offset(pred_stride as isize);
        r += 1;
    }
}

// ---------------------------------------------------------------------------
// Full inverse Walsh–Hadamard for the Y2 block (RFC 6386 §14.3).
// ---------------------------------------------------------------------------

/// `vp8_short_inv_walsh4x4_c` — inverse WHT for the macroblock Y2 block,
/// scattering the 16 recovered luma DCs into the DC slot of each Y
/// residual block (`mb_dqcoeff[i * 16]`).
#[no_mangle]
pub unsafe extern "C" fn vp8_short_inv_walsh4x4_c(
    input: *mut i16,
    mb_dqcoeff: *mut i16,
) {
    let mut output: [i16; 16] = [0; 16];
    let mut a1: i32;
    let mut b1: i32;
    let mut c1: i32;
    let mut d1: i32;
    let mut a2: i32;
    let mut b2: i32;
    let mut c2: i32;
    let mut d2: i32;
    let mut ip: *mut i16 = input;
    let mut op: *mut i16 = output.as_mut_ptr();

    let mut i: i32 = 0;
    while i < 4 {
        a1 = (*ip.offset(0) as i32) + (*ip.offset(12) as i32);
        b1 = (*ip.offset(4) as i32) + (*ip.offset(8) as i32);
        c1 = (*ip.offset(4) as i32) - (*ip.offset(8) as i32);
        d1 = (*ip.offset(0) as i32) - (*ip.offset(12) as i32);

        *op.offset(0) = (a1 + b1) as i16;
        *op.offset(4) = (c1 + d1) as i16;
        *op.offset(8) = (a1 - b1) as i16;
        *op.offset(12) = (d1 - c1) as i16;
        ip = ip.offset(1);
        op = op.offset(1);
        i += 1;
    }

    ip = output.as_mut_ptr();
    op = output.as_mut_ptr();

    let mut i: i32 = 0;
    while i < 4 {
        a1 = (*ip.offset(0) as i32) + (*ip.offset(3) as i32);
        b1 = (*ip.offset(1) as i32) + (*ip.offset(2) as i32);
        c1 = (*ip.offset(1) as i32) - (*ip.offset(2) as i32);
        d1 = (*ip.offset(0) as i32) - (*ip.offset(3) as i32);

        a2 = a1 + b1;
        b2 = c1 + d1;
        c2 = a1 - b1;
        d2 = d1 - c1;

        *op.offset(0) = ((a2 + 3) >> 3) as i16;
        *op.offset(1) = ((b2 + 3) >> 3) as i16;
        *op.offset(2) = ((c2 + 3) >> 3) as i16;
        *op.offset(3) = ((d2 + 3) >> 3) as i16;

        ip = ip.offset(4);
        op = op.offset(4);
        i += 1;
    }

    let mut i: i32 = 0;
    while i < 16 {
        *mb_dqcoeff.offset((i * 16) as isize) = output[i as usize];
        i += 1;
    }
}

// ---------------------------------------------------------------------------
// DC-only inverse Walsh–Hadamard.
// ---------------------------------------------------------------------------

/// `vp8_short_inv_walsh4x4_1_c` — fast path when only the WHT DC is
/// non-zero; every Y block gets the same recovered DC.
#[no_mangle]
pub unsafe extern "C" fn vp8_short_inv_walsh4x4_1_c(
    input: *mut i16,
    mb_dqcoeff: *mut i16,
) {
    let a1: i32 = ((*input.offset(0) as i32) + 3) >> 3;

    let mut i: i32 = 0;
    while i < 16 {
        *mb_dqcoeff.offset((i * 16) as isize) = a1 as i16;
        i += 1;
    }
}
