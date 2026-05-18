//! Generic C intra-prediction kernels (`vpx_dsp/intrapred.c`).
//!
//! Literal Rust transliteration of libvpx's reference C intra-prediction
//! kernels. Every function is a `_c` reference entry point that on a
//! SIMD-capable target would be replaced at startup by a NEON/SSE2/MSA
//! variant via the RTCD dispatch table; on the `generic-gnu`
//! configuration these functions are what actually runs.
//!
//! The C file uses two layers of preprocessor macros
//! (`intra_pred_sized` / `intra_pred_allsizes` / `intra_pred_no_4x4`) to
//! materialize each `(mode, size)` pair as a separately-linked symbol.
//! Rust has no equivalent; each variant is written out as its own
//! function below.
//!
//! `CONFIG_VP9_HIGHBITDEPTH` is off in the minimal VP8 build, so the
//! `highbd_*` ladder from the C file is omitted (it is `#if`'d out
//! upstream too).

#![allow(non_snake_case)]
#![allow(dead_code)]

use core::ptr;

// ---------------------------------------------------------------------------
// `#define DST(x, y) dst[(x) + (y) * stride]`
// `#define AVG3(a, b, c) (((a) + 2 * (b) + (c) + 2) >> 2)`
// `#define AVG2(a, b) (((a) + (b) + 1) >> 1)`
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn dst_set(dst: *mut u8, stride: isize, x: isize, y: isize, v: u8) {
    *dst.offset(x + y * stride) = v;
}

#[inline(always)]
fn avg3(a: i32, b: i32, c: i32) -> u8 {
    ((a + 2 * b + c + 2) >> 2) as u8
}

#[inline(always)]
fn avg2(a: i32, b: i32) -> u8 {
    ((a + b + 1) >> 1) as u8
}

/// `clip_pixel` from `vpx_dsp/vpx_dsp_common.h`.
#[inline(always)]
fn clip_pixel(val: i32) -> u8 {
    if val > 255 {
        255
    } else if val < 0 {
        0
    } else {
        val as u8
    }
}

// ===========================================================================
// Kernel templates (the `static INLINE *_predictor` family in the C file).
// `bs` (block-side) is a runtime parameter here; the per-size wrappers
// below pin it to a compile-time constant.
// ===========================================================================

unsafe fn d117_predictor(
    mut dst: *mut u8,
    stride: isize,
    bs: i32,
    above: *const u8,
    left: *const u8,
) {
    // first row
    for c in 0..bs {
        *dst.offset(c as isize) =
            avg2(*above.offset(c as isize - 1) as i32, *above.offset(c as isize) as i32);
    }
    dst = dst.offset(stride);

    // second row
    *dst.offset(0) = avg3(
        *left.offset(0) as i32,
        *above.offset(-1) as i32,
        *above.offset(0) as i32,
    );
    for c in 1..bs {
        *dst.offset(c as isize) = avg3(
            *above.offset(c as isize - 2) as i32,
            *above.offset(c as isize - 1) as i32,
            *above.offset(c as isize) as i32,
        );
    }
    dst = dst.offset(stride);

    // the rest of first col
    *dst.offset(0) = avg3(
        *above.offset(-1) as i32,
        *left.offset(0) as i32,
        *left.offset(1) as i32,
    );
    for r in 3..bs {
        *dst.offset((r - 2) as isize * stride) = avg3(
            *left.offset((r - 3) as isize) as i32,
            *left.offset((r - 2) as isize) as i32,
            *left.offset((r - 1) as isize) as i32,
        );
    }

    // the rest of the block
    for _r in 2..bs {
        for c in 1..bs {
            *dst.offset(c as isize) = *dst.offset(-2 * stride + c as isize - 1);
        }
        dst = dst.offset(stride);
    }
}

unsafe fn d135_predictor(
    dst: *mut u8,
    stride: isize,
    bs: i32,
    above: *const u8,
    left: *const u8,
) {
    // outer border from bottom-left to top-right; max size 32+32-1 = 63,
    // padded to 69 to mirror the C `#if __GNUC__ == 4 ...` branch.
    let mut border = [0u8; 69];

    // dst(bs, bs - 2)[0], i.e., border starting at bottom-left
    for i in 0..(bs - 2) {
        border[i as usize] = avg3(
            *left.offset((bs - 3 - i) as isize) as i32,
            *left.offset((bs - 2 - i) as isize) as i32,
            *left.offset((bs - 1 - i) as isize) as i32,
        );
    }
    border[(bs - 2) as usize] = avg3(
        *above.offset(-1) as i32,
        *left.offset(0) as i32,
        *left.offset(1) as i32,
    );
    border[(bs - 1) as usize] = avg3(
        *left.offset(0) as i32,
        *above.offset(-1) as i32,
        *above.offset(0) as i32,
    );
    border[bs as usize] = avg3(
        *above.offset(-1) as i32,
        *above.offset(0) as i32,
        *above.offset(1) as i32,
    );
    // dst[0][2, size), i.e., remaining top border ascending
    for i in 0..(bs - 2) {
        border[(bs + 1 + i) as usize] = avg3(
            *above.offset(i as isize) as i32,
            *above.offset(i as isize + 1) as i32,
            *above.offset(i as isize + 2) as i32,
        );
    }

    for i in 0..bs {
        ptr::copy_nonoverlapping(
            border.as_ptr().offset((bs - 1 - i) as isize),
            dst.offset(i as isize * stride),
            bs as usize,
        );
    }
}

unsafe fn d153_predictor(
    mut dst: *mut u8,
    stride: isize,
    bs: i32,
    above: *const u8,
    left: *const u8,
) {
    *dst.offset(0) = avg2(*above.offset(-1) as i32, *left.offset(0) as i32);
    for r in 1..bs {
        *dst.offset(r as isize * stride) =
            avg2(*left.offset((r - 1) as isize) as i32, *left.offset(r as isize) as i32);
    }
    dst = dst.offset(1);

    *dst.offset(0) = avg3(
        *left.offset(0) as i32,
        *above.offset(-1) as i32,
        *above.offset(0) as i32,
    );
    *dst.offset(stride) = avg3(
        *above.offset(-1) as i32,
        *left.offset(0) as i32,
        *left.offset(1) as i32,
    );
    for r in 2..bs {
        *dst.offset(r as isize * stride) = avg3(
            *left.offset((r - 2) as isize) as i32,
            *left.offset((r - 1) as isize) as i32,
            *left.offset(r as isize) as i32,
        );
    }
    dst = dst.offset(1);

    for c in 0..(bs - 2) {
        *dst.offset(c as isize) = avg3(
            *above.offset(c as isize - 1) as i32,
            *above.offset(c as isize) as i32,
            *above.offset(c as isize + 1) as i32,
        );
    }
    dst = dst.offset(stride);

    for _r in 1..bs {
        for c in 0..(bs - 2) {
            *dst.offset(c as isize) = *dst.offset(-stride + c as isize - 2);
        }
        dst = dst.offset(stride);
    }
}

unsafe fn v_predictor(
    mut dst: *mut u8,
    stride: isize,
    bs: i32,
    above: *const u8,
    _left: *const u8,
) {
    for _r in 0..bs {
        ptr::copy_nonoverlapping(above, dst, bs as usize);
        dst = dst.offset(stride);
    }
}

unsafe fn h_predictor(
    mut dst: *mut u8,
    stride: isize,
    bs: i32,
    _above: *const u8,
    left: *const u8,
) {
    for r in 0..bs {
        ptr::write_bytes(dst, *left.offset(r as isize), bs as usize);
        dst = dst.offset(stride);
    }
}

unsafe fn tm_predictor(
    mut dst: *mut u8,
    stride: isize,
    bs: i32,
    above: *const u8,
    left: *const u8,
) {
    let ytop_left = *above.offset(-1) as i32;
    for r in 0..bs {
        for c in 0..bs {
            *dst.offset(c as isize) = clip_pixel(
                *left.offset(r as isize) as i32 + *above.offset(c as isize) as i32 - ytop_left,
            );
        }
        dst = dst.offset(stride);
    }
}

unsafe fn dc_128_predictor(
    mut dst: *mut u8,
    stride: isize,
    bs: i32,
    _above: *const u8,
    _left: *const u8,
) {
    for _r in 0..bs {
        ptr::write_bytes(dst, 128u8, bs as usize);
        dst = dst.offset(stride);
    }
}

unsafe fn dc_left_predictor(
    mut dst: *mut u8,
    stride: isize,
    bs: i32,
    _above: *const u8,
    left: *const u8,
) {
    let mut sum: i32 = 0;
    for i in 0..bs {
        sum += *left.offset(i as isize) as i32;
    }
    let expected_dc = ((sum + (bs >> 1)) / bs) as u8;

    for _r in 0..bs {
        ptr::write_bytes(dst, expected_dc, bs as usize);
        dst = dst.offset(stride);
    }
}

unsafe fn dc_top_predictor(
    mut dst: *mut u8,
    stride: isize,
    bs: i32,
    above: *const u8,
    _left: *const u8,
) {
    let mut sum: i32 = 0;
    for i in 0..bs {
        sum += *above.offset(i as isize) as i32;
    }
    let expected_dc = ((sum + (bs >> 1)) / bs) as u8;

    for _r in 0..bs {
        ptr::write_bytes(dst, expected_dc, bs as usize);
        dst = dst.offset(stride);
    }
}

unsafe fn dc_predictor(
    mut dst: *mut u8,
    stride: isize,
    bs: i32,
    above: *const u8,
    left: *const u8,
) {
    let count = 2 * bs;
    let mut sum: i32 = 0;
    for i in 0..bs {
        sum += *above.offset(i as isize) as i32;
        sum += *left.offset(i as isize) as i32;
    }
    let expected_dc = ((sum + (count >> 1)) / count) as u8;

    for _r in 0..bs {
        ptr::write_bytes(dst, expected_dc, bs as usize);
        dst = dst.offset(stride);
    }
}

// ===========================================================================
// Hand-unrolled 4x4 entry points.
// ===========================================================================

#[no_mangle]
pub unsafe extern "C" fn vpx_he_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    above: *const u8,
    left: *const u8,
) {
    let H = *above.offset(-1) as i32;
    let I = *left.offset(0) as i32;
    let J = *left.offset(1) as i32;
    let K = *left.offset(2) as i32;
    let L = *left.offset(3) as i32;

    ptr::write_bytes(dst.offset(stride * 0), avg3(H, I, J), 4);
    ptr::write_bytes(dst.offset(stride * 1), avg3(I, J, K), 4);
    ptr::write_bytes(dst.offset(stride * 2), avg3(J, K, L), 4);
    ptr::write_bytes(dst.offset(stride * 3), avg3(K, L, L), 4);
}

#[no_mangle]
pub unsafe extern "C" fn vpx_ve_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    above: *const u8,
    _left: *const u8,
) {
    let H = *above.offset(-1) as i32;
    let I = *above.offset(0) as i32;
    let J = *above.offset(1) as i32;
    let K = *above.offset(2) as i32;
    let L = *above.offset(3) as i32;
    let M = *above.offset(4) as i32;

    *dst.offset(0) = avg3(H, I, J);
    *dst.offset(1) = avg3(I, J, K);
    *dst.offset(2) = avg3(J, K, L);
    *dst.offset(3) = avg3(K, L, M);
    ptr::copy_nonoverlapping(dst, dst.offset(stride * 1), 4);
    ptr::copy_nonoverlapping(dst, dst.offset(stride * 2), 4);
    ptr::copy_nonoverlapping(dst, dst.offset(stride * 3), 4);
}

#[no_mangle]
pub unsafe extern "C" fn vpx_d207_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    _above: *const u8,
    left: *const u8,
) {
    let I = *left.offset(0) as i32;
    let J = *left.offset(1) as i32;
    let K = *left.offset(2) as i32;
    let L = *left.offset(3) as i32;

    let v00 = avg2(I, J);
    dst_set(dst, stride, 0, 0, v00);
    let v20_01 = avg2(J, K);
    dst_set(dst, stride, 2, 0, v20_01);
    dst_set(dst, stride, 0, 1, v20_01);
    let v21_02 = avg2(K, L);
    dst_set(dst, stride, 2, 1, v21_02);
    dst_set(dst, stride, 0, 2, v21_02);
    let v10 = avg3(I, J, K);
    dst_set(dst, stride, 1, 0, v10);
    let v30_11 = avg3(J, K, L);
    dst_set(dst, stride, 3, 0, v30_11);
    dst_set(dst, stride, 1, 1, v30_11);
    let v31_12 = avg3(K, L, L);
    dst_set(dst, stride, 3, 1, v31_12);
    dst_set(dst, stride, 1, 2, v31_12);
    let vL = L as u8;
    dst_set(dst, stride, 3, 2, vL);
    dst_set(dst, stride, 2, 2, vL);
    dst_set(dst, stride, 0, 3, vL);
    dst_set(dst, stride, 1, 3, vL);
    dst_set(dst, stride, 2, 3, vL);
    dst_set(dst, stride, 3, 3, vL);
}

#[no_mangle]
pub unsafe extern "C" fn vpx_d63_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    above: *const u8,
    _left: *const u8,
) {
    let A = *above.offset(0) as i32;
    let B = *above.offset(1) as i32;
    let C = *above.offset(2) as i32;
    let D = *above.offset(3) as i32;
    let E = *above.offset(4) as i32;
    let F = *above.offset(5) as i32;
    let G = *above.offset(6) as i32;

    dst_set(dst, stride, 0, 0, avg2(A, B));
    let v = avg2(B, C);
    dst_set(dst, stride, 1, 0, v);
    dst_set(dst, stride, 0, 2, v);
    let v = avg2(C, D);
    dst_set(dst, stride, 2, 0, v);
    dst_set(dst, stride, 1, 2, v);
    let v = avg2(D, E);
    dst_set(dst, stride, 3, 0, v);
    dst_set(dst, stride, 2, 2, v);
    dst_set(dst, stride, 3, 2, avg2(E, F)); // differs from vp8

    dst_set(dst, stride, 0, 1, avg3(A, B, C));
    let v = avg3(B, C, D);
    dst_set(dst, stride, 1, 1, v);
    dst_set(dst, stride, 0, 3, v);
    let v = avg3(C, D, E);
    dst_set(dst, stride, 2, 1, v);
    dst_set(dst, stride, 1, 3, v);
    let v = avg3(D, E, F);
    dst_set(dst, stride, 3, 1, v);
    dst_set(dst, stride, 2, 3, v);
    dst_set(dst, stride, 3, 3, avg3(E, F, G)); // differs from vp8
}

#[no_mangle]
pub unsafe extern "C" fn vpx_d63e_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    above: *const u8,
    _left: *const u8,
) {
    let A = *above.offset(0) as i32;
    let B = *above.offset(1) as i32;
    let C = *above.offset(2) as i32;
    let D = *above.offset(3) as i32;
    let E = *above.offset(4) as i32;
    let F = *above.offset(5) as i32;
    let G = *above.offset(6) as i32;
    let H = *above.offset(7) as i32;

    dst_set(dst, stride, 0, 0, avg2(A, B));
    let v = avg2(B, C);
    dst_set(dst, stride, 1, 0, v);
    dst_set(dst, stride, 0, 2, v);
    let v = avg2(C, D);
    dst_set(dst, stride, 2, 0, v);
    dst_set(dst, stride, 1, 2, v);
    let v = avg2(D, E);
    dst_set(dst, stride, 3, 0, v);
    dst_set(dst, stride, 2, 2, v);
    dst_set(dst, stride, 3, 2, avg3(E, F, G));

    dst_set(dst, stride, 0, 1, avg3(A, B, C));
    let v = avg3(B, C, D);
    dst_set(dst, stride, 1, 1, v);
    dst_set(dst, stride, 0, 3, v);
    let v = avg3(C, D, E);
    dst_set(dst, stride, 2, 1, v);
    dst_set(dst, stride, 1, 3, v);
    let v = avg3(D, E, F);
    dst_set(dst, stride, 3, 1, v);
    dst_set(dst, stride, 2, 3, v);
    dst_set(dst, stride, 3, 3, avg3(F, G, H));
}

#[no_mangle]
pub unsafe extern "C" fn vpx_d45_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    above: *const u8,
    _left: *const u8,
) {
    let A = *above.offset(0) as i32;
    let B = *above.offset(1) as i32;
    let C = *above.offset(2) as i32;
    let D = *above.offset(3) as i32;
    let E = *above.offset(4) as i32;
    let F = *above.offset(5) as i32;
    let G = *above.offset(6) as i32;
    let H = *above.offset(7) as i32;

    dst_set(dst, stride, 0, 0, avg3(A, B, C));
    let v = avg3(B, C, D);
    dst_set(dst, stride, 1, 0, v);
    dst_set(dst, stride, 0, 1, v);
    let v = avg3(C, D, E);
    dst_set(dst, stride, 2, 0, v);
    dst_set(dst, stride, 1, 1, v);
    dst_set(dst, stride, 0, 2, v);
    let v = avg3(D, E, F);
    dst_set(dst, stride, 3, 0, v);
    dst_set(dst, stride, 2, 1, v);
    dst_set(dst, stride, 1, 2, v);
    dst_set(dst, stride, 0, 3, v);
    let v = avg3(E, F, G);
    dst_set(dst, stride, 3, 1, v);
    dst_set(dst, stride, 2, 2, v);
    dst_set(dst, stride, 1, 3, v);
    let v = avg3(F, G, H);
    dst_set(dst, stride, 3, 2, v);
    dst_set(dst, stride, 2, 3, v);
    dst_set(dst, stride, 3, 3, H as u8); // differs from vp8
}

#[no_mangle]
pub unsafe extern "C" fn vpx_d45e_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    above: *const u8,
    _left: *const u8,
) {
    let A = *above.offset(0) as i32;
    let B = *above.offset(1) as i32;
    let C = *above.offset(2) as i32;
    let D = *above.offset(3) as i32;
    let E = *above.offset(4) as i32;
    let F = *above.offset(5) as i32;
    let G = *above.offset(6) as i32;
    let H = *above.offset(7) as i32;

    dst_set(dst, stride, 0, 0, avg3(A, B, C));
    let v = avg3(B, C, D);
    dst_set(dst, stride, 1, 0, v);
    dst_set(dst, stride, 0, 1, v);
    let v = avg3(C, D, E);
    dst_set(dst, stride, 2, 0, v);
    dst_set(dst, stride, 1, 1, v);
    dst_set(dst, stride, 0, 2, v);
    let v = avg3(D, E, F);
    dst_set(dst, stride, 3, 0, v);
    dst_set(dst, stride, 2, 1, v);
    dst_set(dst, stride, 1, 2, v);
    dst_set(dst, stride, 0, 3, v);
    let v = avg3(E, F, G);
    dst_set(dst, stride, 3, 1, v);
    dst_set(dst, stride, 2, 2, v);
    dst_set(dst, stride, 1, 3, v);
    let v = avg3(F, G, H);
    dst_set(dst, stride, 3, 2, v);
    dst_set(dst, stride, 2, 3, v);
    dst_set(dst, stride, 3, 3, avg3(G, H, H));
}

#[no_mangle]
pub unsafe extern "C" fn vpx_d117_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    above: *const u8,
    left: *const u8,
) {
    let I = *left.offset(0) as i32;
    let J = *left.offset(1) as i32;
    let K = *left.offset(2) as i32;
    let X = *above.offset(-1) as i32;
    let A = *above.offset(0) as i32;
    let B = *above.offset(1) as i32;
    let C = *above.offset(2) as i32;
    let D = *above.offset(3) as i32;

    let v = avg2(X, A);
    dst_set(dst, stride, 0, 0, v);
    dst_set(dst, stride, 1, 2, v);
    let v = avg2(A, B);
    dst_set(dst, stride, 1, 0, v);
    dst_set(dst, stride, 2, 2, v);
    let v = avg2(B, C);
    dst_set(dst, stride, 2, 0, v);
    dst_set(dst, stride, 3, 2, v);
    dst_set(dst, stride, 3, 0, avg2(C, D));

    dst_set(dst, stride, 0, 3, avg3(K, J, I));
    dst_set(dst, stride, 0, 2, avg3(J, I, X));
    let v = avg3(I, X, A);
    dst_set(dst, stride, 0, 1, v);
    dst_set(dst, stride, 1, 3, v);
    let v = avg3(X, A, B);
    dst_set(dst, stride, 1, 1, v);
    dst_set(dst, stride, 2, 3, v);
    let v = avg3(A, B, C);
    dst_set(dst, stride, 2, 1, v);
    dst_set(dst, stride, 3, 3, v);
    dst_set(dst, stride, 3, 1, avg3(B, C, D));
}

#[no_mangle]
pub unsafe extern "C" fn vpx_d135_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    above: *const u8,
    left: *const u8,
) {
    let I = *left.offset(0) as i32;
    let J = *left.offset(1) as i32;
    let K = *left.offset(2) as i32;
    let L = *left.offset(3) as i32;
    let X = *above.offset(-1) as i32;
    let A = *above.offset(0) as i32;
    let B = *above.offset(1) as i32;
    let C = *above.offset(2) as i32;
    let D = *above.offset(3) as i32;

    dst_set(dst, stride, 0, 3, avg3(J, K, L));
    let v = avg3(I, J, K);
    dst_set(dst, stride, 1, 3, v);
    dst_set(dst, stride, 0, 2, v);
    let v = avg3(X, I, J);
    dst_set(dst, stride, 2, 3, v);
    dst_set(dst, stride, 1, 2, v);
    dst_set(dst, stride, 0, 1, v);
    let v = avg3(A, X, I);
    dst_set(dst, stride, 3, 3, v);
    dst_set(dst, stride, 2, 2, v);
    dst_set(dst, stride, 1, 1, v);
    dst_set(dst, stride, 0, 0, v);
    let v = avg3(B, A, X);
    dst_set(dst, stride, 3, 2, v);
    dst_set(dst, stride, 2, 1, v);
    dst_set(dst, stride, 1, 0, v);
    let v = avg3(C, B, A);
    dst_set(dst, stride, 3, 1, v);
    dst_set(dst, stride, 2, 0, v);
    dst_set(dst, stride, 3, 0, avg3(D, C, B));
}

#[no_mangle]
pub unsafe extern "C" fn vpx_d153_predictor_4x4_c(
    dst: *mut u8,
    stride: isize,
    above: *const u8,
    left: *const u8,
) {
    let I = *left.offset(0) as i32;
    let J = *left.offset(1) as i32;
    let K = *left.offset(2) as i32;
    let L = *left.offset(3) as i32;
    let X = *above.offset(-1) as i32;
    let A = *above.offset(0) as i32;
    let B = *above.offset(1) as i32;
    let C = *above.offset(2) as i32;

    let v = avg2(I, X);
    dst_set(dst, stride, 0, 0, v);
    dst_set(dst, stride, 2, 1, v);
    let v = avg2(J, I);
    dst_set(dst, stride, 0, 1, v);
    dst_set(dst, stride, 2, 2, v);
    let v = avg2(K, J);
    dst_set(dst, stride, 0, 2, v);
    dst_set(dst, stride, 2, 3, v);
    dst_set(dst, stride, 0, 3, avg2(L, K));

    dst_set(dst, stride, 3, 0, avg3(A, B, C));
    dst_set(dst, stride, 2, 0, avg3(X, A, B));
    let v = avg3(I, X, A);
    dst_set(dst, stride, 1, 0, v);
    dst_set(dst, stride, 3, 1, v);
    let v = avg3(J, I, X);
    dst_set(dst, stride, 1, 1, v);
    dst_set(dst, stride, 3, 2, v);
    let v = avg3(K, J, I);
    dst_set(dst, stride, 1, 2, v);
    dst_set(dst, stride, 3, 3, v);
    dst_set(dst, stride, 1, 3, avg3(L, K, J));
}

// ===========================================================================
// Macro-generated per-size wrappers.
//
// In C these are emitted by `intra_pred_no_4x4(d207)` / `intra_pred_no_4x4(d63)`
// / `intra_pred_no_4x4(d45)` / `intra_pred_no_4x4(d117)` /
// `intra_pred_no_4x4(d135)` / `intra_pred_no_4x4(d153)` (sizes 8/16/32) and
// `intra_pred_allsizes(...)` for v/h/tm/dc_128/dc_left/dc_top/dc (sizes
// 4/8/16/32). Each one pins `bs` to a compile-time constant and forwards.
// ===========================================================================

macro_rules! intra_pred_sized_rs {
    ($name:ident, $tpl:ident, $size:expr) => {
        #[no_mangle]
        pub unsafe extern "C" fn $name(
            dst: *mut u8,
            stride: isize,
            above: *const u8,
            left: *const u8,
        ) {
            $tpl(dst, stride, $size, above, left);
        }
    };
}

// d117 — sizes 8, 16
intra_pred_sized_rs!(vpx_d117_predictor_8x8_c, d117_predictor, 8);
intra_pred_sized_rs!(vpx_d117_predictor_16x16_c, d117_predictor, 16);

// d135 — sizes 8, 16
intra_pred_sized_rs!(vpx_d135_predictor_8x8_c, d135_predictor, 8);
intra_pred_sized_rs!(vpx_d135_predictor_16x16_c, d135_predictor, 16);

// d153 — sizes 8, 16
intra_pred_sized_rs!(vpx_d153_predictor_8x8_c, d153_predictor, 8);
intra_pred_sized_rs!(vpx_d153_predictor_16x16_c, d153_predictor, 16);

// v — sizes 4, 8, 16
intra_pred_sized_rs!(vpx_v_predictor_4x4_c, v_predictor, 4);
intra_pred_sized_rs!(vpx_v_predictor_8x8_c, v_predictor, 8);
intra_pred_sized_rs!(vpx_v_predictor_16x16_c, v_predictor, 16);

// h — sizes 4, 8, 16
intra_pred_sized_rs!(vpx_h_predictor_4x4_c, h_predictor, 4);
intra_pred_sized_rs!(vpx_h_predictor_8x8_c, h_predictor, 8);
intra_pred_sized_rs!(vpx_h_predictor_16x16_c, h_predictor, 16);

// tm — sizes 4, 8, 16
intra_pred_sized_rs!(vpx_tm_predictor_4x4_c, tm_predictor, 4);
intra_pred_sized_rs!(vpx_tm_predictor_8x8_c, tm_predictor, 8);
intra_pred_sized_rs!(vpx_tm_predictor_16x16_c, tm_predictor, 16);

// dc_128 — sizes 4, 8, 16
intra_pred_sized_rs!(vpx_dc_128_predictor_4x4_c, dc_128_predictor, 4);
intra_pred_sized_rs!(vpx_dc_128_predictor_8x8_c, dc_128_predictor, 8);
intra_pred_sized_rs!(vpx_dc_128_predictor_16x16_c, dc_128_predictor, 16);

// dc_left — sizes 4, 8, 16
intra_pred_sized_rs!(vpx_dc_left_predictor_4x4_c, dc_left_predictor, 4);
intra_pred_sized_rs!(vpx_dc_left_predictor_8x8_c, dc_left_predictor, 8);
intra_pred_sized_rs!(vpx_dc_left_predictor_16x16_c, dc_left_predictor, 16);

// dc_top — sizes 4, 8, 16
intra_pred_sized_rs!(vpx_dc_top_predictor_4x4_c, dc_top_predictor, 4);
intra_pred_sized_rs!(vpx_dc_top_predictor_8x8_c, dc_top_predictor, 8);
intra_pred_sized_rs!(vpx_dc_top_predictor_16x16_c, dc_top_predictor, 16);

// dc — sizes 4, 8, 16
intra_pred_sized_rs!(vpx_dc_predictor_4x4_c, dc_predictor, 4);
intra_pred_sized_rs!(vpx_dc_predictor_8x8_c, dc_predictor, 8);
intra_pred_sized_rs!(vpx_dc_predictor_16x16_c, dc_predictor, 16);
