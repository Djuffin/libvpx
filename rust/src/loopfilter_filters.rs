//! Literal Rust transliteration of `vp8/common/loopfilter_filters.c`.
//!
//! The inner-loop pixel kernels for the VP8 deblocking filter. See
//! `documentation/vp8_files/loopfilter_filters.md` for design notes.
//!
//! These are the `_c` reference implementations dispatched via
//! `vp8_rtcd.h` when SIMD is unavailable. All per-pixel arithmetic is
//! done in `i8` (signed char, range `-128..127`) by XOR'ing input bytes
//! with `0x80`, so overflow on differences stays within byte range and
//! is saturated cheaply via [`vp8_signed_char_clamp`].

#![allow(non_snake_case)]
#![allow(clippy::missing_safety_doc)]

use crate::types::LoopFilterInfo;

/// `typedef unsigned char uc;` — `loopfilter_filters.c:15`.
#[allow(dead_code)]
type Uc = u8;

// ===========================================================================
// Arithmetic helper
// ===========================================================================

/// Saturating `int -> signed char` cast. `loopfilter_filters.c:17`.
#[inline]
fn vp8_signed_char_clamp(t: i32) -> i8 {
    t.clamp(i8::MIN as i32, i8::MAX as i32) as i8
}

// ===========================================================================
// Edge classification helpers
// ===========================================================================

/// `vp8_filter_mask` — should we apply any filter at all
/// (11111111 yes, 00000000 no). `loopfilter_filters.c:24`.
#[inline]
fn vp8_filter_mask(
    limit: Uc,
    blimit: Uc,
    p3: Uc,
    p2: Uc,
    p1: Uc,
    p0: Uc,
    q0: Uc,
    q1: Uc,
    q2: Uc,
    q3: Uc,
) -> i8 {
    let limit = limit as i32;
    let blimit = blimit as i32;
    let p3 = p3 as i32;
    let p2 = p2 as i32;
    let p1 = p1 as i32;
    let p0 = p0 as i32;
    let q0 = q0 as i32;
    let q1 = q1 as i32;
    let q2 = q2 as i32;
    let q3 = q3 as i32;

    let mut mask: i8 = 0;
    mask |= ((p3 - p2).abs() > limit) as i8;
    mask |= ((p2 - p1).abs() > limit) as i8;
    mask |= ((p1 - p0).abs() > limit) as i8;
    mask |= ((q1 - q0).abs() > limit) as i8;
    mask |= ((q2 - q1).abs() > limit) as i8;
    mask |= ((q3 - q2).abs() > limit) as i8;
    mask |= ((p0 - q0).abs() * 2 + (p1 - q1).abs() / 2 > blimit) as i8;
    mask.wrapping_sub(1)
}

/// `vp8_hevmask` — high-edge-variance flag (11111111 yes, 00000000 no).
/// `loopfilter_filters.c:38`.
#[inline]
fn vp8_hevmask(thresh: Uc, p1: Uc, p0: Uc, q0: Uc, q1: Uc) -> i8 {
    let thresh = thresh as i32;
    let p1 = p1 as i32;
    let p0 = p0 as i32;
    let q0 = q0 as i32;
    let q1 = q1 as i32;

    let mut hev: i8 = 0;
    hev |= (((p1 - p0).abs() > thresh) as i8).wrapping_mul(-1);
    hev |= (((q1 - q0).abs() > thresh) as i8).wrapping_mul(-1);
    hev
}

// ===========================================================================
// Normal loop filter — 4-tap inner kernel
// ===========================================================================

/// `vp8_filter` — 4-tap inner filter. `loopfilter_filters.c:45`.
///
/// Reads the four pixels straddling the edge (`p1`, `p0`, `q0`, `q1`)
/// and returns their filtered replacements.
#[inline]
fn vp8_filter(mask: i8, hev: Uc, p1_in: u8, p0_in: u8, q0_in: u8, q1_in: u8) -> (u8, u8, u8, u8) {
    let ps1: i8 = (p1_in ^ 0x80) as i8;
    let ps0: i8 = (p0_in ^ 0x80) as i8;
    let qs0: i8 = (q0_in ^ 0x80) as i8;
    let qs1: i8 = (q1_in ^ 0x80) as i8;

    /* add outer taps if we have high edge variance */
    let mut filter_value: i8 = vp8_signed_char_clamp(ps1 as i32 - qs1 as i32);
    filter_value &= hev as i8;

    /* inner taps */
    filter_value = vp8_signed_char_clamp(filter_value as i32 + 3 * (qs0 as i32 - ps0 as i32));
    filter_value &= mask;

    /* save bottom 3 bits so that we round one side +4 and the other +3
     * if it equals 4 we'll set it to adjust by -1 to account for the fact
     * we'd round it by 3 the other way
     */
    let mut Filter1: i8 = vp8_signed_char_clamp(filter_value as i32 + 4);
    let mut Filter2: i8 = vp8_signed_char_clamp(filter_value as i32 + 3);
    Filter1 >>= 3;
    Filter2 >>= 3;
    let mut u: i8 = vp8_signed_char_clamp(qs0 as i32 - Filter1 as i32);
    let oq0 = (u as u8) ^ 0x80;
    u = vp8_signed_char_clamp(ps0 as i32 + Filter2 as i32);
    let op0 = (u as u8) ^ 0x80;
    filter_value = Filter1;

    /* outer tap adjustments */
    filter_value = filter_value.wrapping_add(1);
    filter_value >>= 1;
    filter_value &= !(hev as i8);

    u = vp8_signed_char_clamp(qs1 as i32 - filter_value as i32);
    let oq1 = (u as u8) ^ 0x80;
    u = vp8_signed_char_clamp(ps1 as i32 + filter_value as i32);
    let op1 = (u as u8) ^ 0x80;
    (op1, op0, oq0, oq1)
}

/// `loop_filter_horizontal_edge_c`. `loopfilter_filters.c:90`.
#[inline]
unsafe fn loop_filter_horizontal_edge_c(
    s: *mut u8,
    p: i32,
    blimit: *const u8,
    limit: *const u8,
    thresh: *const u8,
    count: i32,
) {
    let mut hev: i32; /* high edge variance */
    let mut mask: i8;
    let mut i: i32 = 0;

    /* loop filter designed to work using chars so that we can make maximum use
     * of 8 bit simd instructions.
     */
    let mut s = s;
    loop {
        mask = vp8_filter_mask(
            *limit.add(0),
            *blimit.add(0),
            *s.offset(-4 * p as isize),
            *s.offset(-3 * p as isize),
            *s.offset(-2 * p as isize),
            *s.offset(-1 * p as isize),
            *s.offset(0 * p as isize),
            *s.offset(1 * p as isize),
            *s.offset(2 * p as isize),
            *s.offset(3 * p as isize),
        );

        hev = vp8_hevmask(
            *thresh.add(0),
            *s.offset(-2 * p as isize),
            *s.offset(-1 * p as isize),
            *s.offset(0 * p as isize),
            *s.offset(1 * p as isize),
        ) as i32;

        let pm2 = s.offset(-2 * p as isize);
        let pm1 = s.offset(-1 * p as isize);
        let pp1 = s.offset(1 * p as isize);
        let (np1, np0, nq0, nq1) = vp8_filter(mask, hev as u8, *pm2, *pm1, *s, *pp1);
        *pm2 = np1;
        *pm1 = np0;
        *s = nq0;
        *pp1 = nq1;

        s = s.add(1);
        i += 1;
        if i >= count * 8 {
            break;
        }
    }
}

/// `loop_filter_vertical_edge_c`. `loopfilter_filters.c:114`.
#[inline]
unsafe fn loop_filter_vertical_edge_c(
    s: *mut u8,
    p: i32,
    blimit: *const u8,
    limit: *const u8,
    thresh: *const u8,
    count: i32,
) {
    let mut hev: i32; /* high edge variance */
    let mut mask: i8;
    let mut i: i32 = 0;

    let mut s = s;
    loop {
        mask = vp8_filter_mask(
            *limit.add(0),
            *blimit.add(0),
            *s.offset(-4),
            *s.offset(-3),
            *s.offset(-2),
            *s.offset(-1),
            *s.offset(0),
            *s.offset(1),
            *s.offset(2),
            *s.offset(3),
        );

        hev = vp8_hevmask(
            *thresh.add(0),
            *s.offset(-2),
            *s.offset(-1),
            *s.offset(0),
            *s.offset(1),
        ) as i32;

        let pm2 = s.offset(-2);
        let pm1 = s.offset(-1);
        let pp1 = s.offset(1);
        let (np1, np0, nq0, nq1) = vp8_filter(mask, hev as u8, *pm2, *pm1, *s, *pp1);
        *pm2 = np1;
        *pm1 = np0;
        *s = nq0;
        *pp1 = nq1;

        s = s.offset(p as isize);
        i += 1;
        if i >= count * 8 {
            break;
        }
    }
}

// ===========================================================================
// Normal loop filter — 7-pixel macroblock-edge kernel
// ===========================================================================

/// `vp8_mbfilter` — 7-pixel macroblock-edge filter.
/// `loopfilter_filters.c:138`.
///
/// Reads the six pixels straddling the macroblock edge and returns their
/// filtered replacements in scan order (`p2`, `p1`, `p0`, `q0`, `q1`, `q2`).
#[inline]
fn vp8_mbfilter(
    mask: i8,
    hev: Uc,
    p2_in: u8,
    p1_in: u8,
    p0_in: u8,
    q0_in: u8,
    q1_in: u8,
    q2_in: u8,
) -> (u8, u8, u8, u8, u8, u8) {
    let mut s: i8;
    let mut u: i8;
    let mut filter_value: i8;
    let mut Filter1: i8;
    let mut Filter2: i8;
    let ps2: i8 = (p2_in ^ 0x80) as i8;
    let ps1: i8 = (p1_in ^ 0x80) as i8;
    let mut ps0: i8 = (p0_in ^ 0x80) as i8;
    let mut qs0: i8 = (q0_in ^ 0x80) as i8;
    let qs1: i8 = (q1_in ^ 0x80) as i8;
    let qs2: i8 = (q2_in ^ 0x80) as i8;

    /* add outer taps if we have high edge variance */
    filter_value = vp8_signed_char_clamp(ps1 as i32 - qs1 as i32);
    filter_value = vp8_signed_char_clamp(filter_value as i32 + 3 * (qs0 as i32 - ps0 as i32));
    filter_value &= mask;

    Filter2 = filter_value;
    Filter2 &= hev as i8;

    /* save bottom 3 bits so that we round one side +4 and the other +3 */
    Filter1 = vp8_signed_char_clamp(Filter2 as i32 + 4);
    Filter2 = vp8_signed_char_clamp(Filter2 as i32 + 3);
    Filter1 >>= 3;
    Filter2 >>= 3;
    qs0 = vp8_signed_char_clamp(qs0 as i32 - Filter1 as i32);
    ps0 = vp8_signed_char_clamp(ps0 as i32 + Filter2 as i32);

    /* only apply wider filter if not high edge variance */
    filter_value &= !(hev as i8);
    Filter2 = filter_value;

    /* roughly 3/7th difference across boundary */
    u = vp8_signed_char_clamp((63 + Filter2 as i32 * 27) >> 7);
    s = vp8_signed_char_clamp(qs0 as i32 - u as i32);
    let oq0 = (s as u8) ^ 0x80;
    s = vp8_signed_char_clamp(ps0 as i32 + u as i32);
    let op0 = (s as u8) ^ 0x80;

    /* roughly 2/7th difference across boundary */
    u = vp8_signed_char_clamp((63 + Filter2 as i32 * 18) >> 7);
    s = vp8_signed_char_clamp(qs1 as i32 - u as i32);
    let oq1 = (s as u8) ^ 0x80;
    s = vp8_signed_char_clamp(ps1 as i32 + u as i32);
    let op1 = (s as u8) ^ 0x80;

    /* roughly 1/7th difference across boundary */
    u = vp8_signed_char_clamp((63 + Filter2 as i32 * 9) >> 7);
    s = vp8_signed_char_clamp(qs2 as i32 - u as i32);
    let oq2 = (s as u8) ^ 0x80;
    s = vp8_signed_char_clamp(ps2 as i32 + u as i32);
    let op2 = (s as u8) ^ 0x80;
    (op2, op1, op0, oq0, oq1, oq2)
}

/// `mbloop_filter_horizontal_edge_c`. `loopfilter_filters.c:191`.
#[inline]
unsafe fn mbloop_filter_horizontal_edge_c(
    s: *mut u8,
    p: i32,
    blimit: *const u8,
    limit: *const u8,
    thresh: *const u8,
    count: i32,
) {
    let mut hev: i8; /* high edge variance */
    let mut mask: i8;
    let mut i: i32 = 0;

    let mut s = s;
    loop {
        mask = vp8_filter_mask(
            *limit.add(0),
            *blimit.add(0),
            *s.offset(-4 * p as isize),
            *s.offset(-3 * p as isize),
            *s.offset(-2 * p as isize),
            *s.offset(-1 * p as isize),
            *s.offset(0 * p as isize),
            *s.offset(1 * p as isize),
            *s.offset(2 * p as isize),
            *s.offset(3 * p as isize),
        );

        hev = vp8_hevmask(
            *thresh.add(0),
            *s.offset(-2 * p as isize),
            *s.offset(-1 * p as isize),
            *s.offset(0 * p as isize),
            *s.offset(1 * p as isize),
        );

        let pm3 = s.offset(-3 * p as isize);
        let pm2 = s.offset(-2 * p as isize);
        let pm1 = s.offset(-1 * p as isize);
        let pp1 = s.offset(1 * p as isize);
        let pp2 = s.offset(2 * p as isize);
        let (np2, np1, np0, nq0, nq1, nq2) =
            vp8_mbfilter(mask, hev as u8, *pm3, *pm2, *pm1, *s, *pp1, *pp2);
        *pm3 = np2;
        *pm2 = np1;
        *pm1 = np0;
        *s = nq0;
        *pp1 = nq1;
        *pp2 = nq2;

        s = s.add(1);
        i += 1;
        if i >= count * 8 {
            break;
        }
    }
}

/// `mbloop_filter_vertical_edge_c`. `loopfilter_filters.c:216`.
#[inline]
unsafe fn mbloop_filter_vertical_edge_c(
    s: *mut u8,
    p: i32,
    blimit: *const u8,
    limit: *const u8,
    thresh: *const u8,
    count: i32,
) {
    let mut hev: i8; /* high edge variance */
    let mut mask: i8;
    let mut i: i32 = 0;

    let mut s = s;
    loop {
        mask = vp8_filter_mask(
            *limit.add(0),
            *blimit.add(0),
            *s.offset(-4),
            *s.offset(-3),
            *s.offset(-2),
            *s.offset(-1),
            *s.offset(0),
            *s.offset(1),
            *s.offset(2),
            *s.offset(3),
        );

        hev = vp8_hevmask(
            *thresh.add(0),
            *s.offset(-2),
            *s.offset(-1),
            *s.offset(0),
            *s.offset(1),
        );

        let pm3 = s.offset(-3);
        let pm2 = s.offset(-2);
        let pm1 = s.offset(-1);
        let pp1 = s.offset(1);
        let pp2 = s.offset(2);
        let (np2, np1, np0, nq0, nq1, nq2) =
            vp8_mbfilter(mask, hev as u8, *pm3, *pm2, *pm1, *s, *pp1, *pp2);
        *pm3 = np2;
        *pm2 = np1;
        *pm1 = np0;
        *s = nq0;
        *pp1 = nq1;
        *pp2 = nq2;

        s = s.offset(p as isize);
        i += 1;
        if i >= count * 8 {
            break;
        }
    }
}

// ===========================================================================
// Simple loop filter
// ===========================================================================

/// `vp8_simple_filter_mask` — simplified per-edge predicate.
/// `loopfilter_filters.c:238`.
#[inline]
fn vp8_simple_filter_mask(blimit: Uc, p1: Uc, p0: Uc, q0: Uc, q1: Uc) -> i8 {
    /* Why does this cause problems for win32?
     * error C2143: syntax error : missing ';' before 'type'
     *  (void) limit;
     */
    let blimit = blimit as i32;
    let p1 = p1 as i32;
    let p0 = p0 as i32;
    let q0 = q0 as i32;
    let q1 = q1 as i32;
    let cond = ((p0 - q0).abs() * 2 + (p1 - q1).abs() / 2 <= blimit) as i8;
    cond.wrapping_mul(-1)
}

/// `vp8_simple_filter`. `loopfilter_filters.c:248`.
///
/// Reads the four pixels straddling the edge and returns the two filtered
/// inner pixels (`p0_new`, `q0_new`). The outer two (`p1`, `q1`) are
/// unchanged by this kernel.
#[inline]
fn vp8_simple_filter(mask: i8, p1_in: u8, p0_in: u8, q0_in: u8, q1_in: u8) -> (u8, u8) {
    let mut filter_value: i8;
    let mut Filter1: i8;
    let mut Filter2: i8;
    let p1: i8 = (p1_in ^ 0x80) as i8;
    let p0: i8 = (p0_in ^ 0x80) as i8;
    let q0: i8 = (q0_in ^ 0x80) as i8;
    let q1: i8 = (q1_in ^ 0x80) as i8;
    let mut u: i8;

    filter_value = vp8_signed_char_clamp(p1 as i32 - q1 as i32);
    filter_value = vp8_signed_char_clamp(filter_value as i32 + 3 * (q0 as i32 - p0 as i32));
    filter_value &= mask;

    /* save bottom 3 bits so that we round one side +4 and the other +3 */
    Filter1 = vp8_signed_char_clamp(filter_value as i32 + 4);
    Filter1 >>= 3;
    u = vp8_signed_char_clamp(q0 as i32 - Filter1 as i32);
    let oq0 = (u as u8) ^ 0x80;

    Filter2 = vp8_signed_char_clamp(filter_value as i32 + 3);
    Filter2 >>= 3;
    u = vp8_signed_char_clamp(p0 as i32 + Filter2 as i32);
    let op0 = (u as u8) ^ 0x80;
    (op0, oq0)
}

// ===========================================================================
// Public entry points
// ===========================================================================

/// `vp8_loop_filter_simple_horizontal_edge_c`. `loopfilter_filters.c:273`.
///
/// # Safety
/// Caller must ensure `y_ptr` and the 4-row neighbourhood around it lie
/// within an allocated plane.
pub unsafe fn vp8_loop_filter_simple_horizontal_edge_c(
    y_ptr: *mut u8,
    y_stride: i32,
    blimit: *const u8,
) {
    let mut mask: i8;
    let mut i: i32 = 0;

    let mut y_ptr = y_ptr;
    loop {
        mask = vp8_simple_filter_mask(
            *blimit.add(0),
            *y_ptr.offset(-2 * y_stride as isize),
            *y_ptr.offset(-1 * y_stride as isize),
            *y_ptr.offset(0 * y_stride as isize),
            *y_ptr.offset(1 * y_stride as isize),
        );
        let pm1 = y_ptr.offset(-1 * y_stride as isize);
        let (np0, nq0) = vp8_simple_filter(
            mask,
            *y_ptr.offset(-2 * y_stride as isize),
            *pm1,
            *y_ptr,
            *y_ptr.offset(1 * y_stride as isize),
        );
        *pm1 = np0;
        *y_ptr = nq0;
        y_ptr = y_ptr.add(1);
        i += 1;
        if i >= 16 {
            break;
        }
    }
}

/// `vp8_loop_filter_simple_vertical_edge_c`. `loopfilter_filters.c:289`.
///
/// # Safety
/// Caller must ensure `y_ptr` and the 4-column neighbourhood around it
/// lie within an allocated plane.
pub unsafe fn vp8_loop_filter_simple_vertical_edge_c(
    y_ptr: *mut u8,
    y_stride: i32,
    blimit: *const u8,
) {
    let mut mask: i8;
    let mut i: i32 = 0;

    let mut y_ptr = y_ptr;
    loop {
        mask = vp8_simple_filter_mask(
            *blimit.add(0),
            *y_ptr.offset(-2),
            *y_ptr.offset(-1),
            *y_ptr.offset(0),
            *y_ptr.offset(1),
        );
        let pm1 = y_ptr.offset(-1);
        let (np0, nq0) =
            vp8_simple_filter(mask, *y_ptr.offset(-2), *pm1, *y_ptr, *y_ptr.offset(1));
        *pm1 = np0;
        *y_ptr = nq0;
        y_ptr = y_ptr.offset(y_stride as isize);
        i += 1;
        if i >= 16 {
            break;
        }
    }
}

/// `vp8_loop_filter_mbh_c` — horizontal MB filtering.
/// `loopfilter_filters.c:303`.
///
/// # Safety
/// `lfi` must point to a valid `LoopFilterInfo`. `u_ptr`/`v_ptr` may be
/// null to skip the chroma planes.
pub unsafe fn vp8_loop_filter_mbh_c(
    y_ptr: *mut u8,
    u_ptr: *mut u8,
    v_ptr: *mut u8,
    y_stride: i32,
    uv_stride: i32,
    lfi: *mut LoopFilterInfo,
) {
    mbloop_filter_horizontal_edge_c(y_ptr, y_stride, (*lfi).mblim, (*lfi).lim, (*lfi).hev_thr, 2);

    if !u_ptr.is_null() {
        mbloop_filter_horizontal_edge_c(
            u_ptr,
            uv_stride,
            (*lfi).mblim,
            (*lfi).lim,
            (*lfi).hev_thr,
            1,
        );
    }

    if !v_ptr.is_null() {
        mbloop_filter_horizontal_edge_c(
            v_ptr,
            uv_stride,
            (*lfi).mblim,
            (*lfi).lim,
            (*lfi).hev_thr,
            1,
        );
    }
}

/// `vp8_loop_filter_mbv_c` — vertical MB filtering.
/// `loopfilter_filters.c:321`.
///
/// # Safety
/// `lfi` must point to a valid `LoopFilterInfo`. `u_ptr`/`v_ptr` may be
/// null.
pub unsafe fn vp8_loop_filter_mbv_c(
    y_ptr: *mut u8,
    u_ptr: *mut u8,
    v_ptr: *mut u8,
    y_stride: i32,
    uv_stride: i32,
    lfi: *mut LoopFilterInfo,
) {
    mbloop_filter_vertical_edge_c(y_ptr, y_stride, (*lfi).mblim, (*lfi).lim, (*lfi).hev_thr, 2);

    if !u_ptr.is_null() {
        mbloop_filter_vertical_edge_c(
            u_ptr,
            uv_stride,
            (*lfi).mblim,
            (*lfi).lim,
            (*lfi).hev_thr,
            1,
        );
    }

    if !v_ptr.is_null() {
        mbloop_filter_vertical_edge_c(
            v_ptr,
            uv_stride,
            (*lfi).mblim,
            (*lfi).lim,
            (*lfi).hev_thr,
            1,
        );
    }
}

/// `vp8_loop_filter_bh_c` — horizontal B filtering.
/// `loopfilter_filters.c:339`.
///
/// # Safety
/// `lfi` must point to a valid `LoopFilterInfo`. `u_ptr`/`v_ptr` may be
/// null.
pub unsafe fn vp8_loop_filter_bh_c(
    y_ptr: *mut u8,
    u_ptr: *mut u8,
    v_ptr: *mut u8,
    y_stride: i32,
    uv_stride: i32,
    lfi: *mut LoopFilterInfo,
) {
    loop_filter_horizontal_edge_c(
        y_ptr.offset(4 * y_stride as isize),
        y_stride,
        (*lfi).blim,
        (*lfi).lim,
        (*lfi).hev_thr,
        2,
    );
    loop_filter_horizontal_edge_c(
        y_ptr.offset(8 * y_stride as isize),
        y_stride,
        (*lfi).blim,
        (*lfi).lim,
        (*lfi).hev_thr,
        2,
    );
    loop_filter_horizontal_edge_c(
        y_ptr.offset(12 * y_stride as isize),
        y_stride,
        (*lfi).blim,
        (*lfi).lim,
        (*lfi).hev_thr,
        2,
    );

    if !u_ptr.is_null() {
        loop_filter_horizontal_edge_c(
            u_ptr.offset(4 * uv_stride as isize),
            uv_stride,
            (*lfi).blim,
            (*lfi).lim,
            (*lfi).hev_thr,
            1,
        );
    }

    if !v_ptr.is_null() {
        loop_filter_horizontal_edge_c(
            v_ptr.offset(4 * uv_stride as isize),
            uv_stride,
            (*lfi).blim,
            (*lfi).lim,
            (*lfi).hev_thr,
            1,
        );
    }
}

/// `vp8_loop_filter_bhs_c`. `loopfilter_filters.c:360`.
///
/// # Safety
/// `blimit` must point to at least one byte.
pub unsafe fn vp8_loop_filter_bhs_c(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    vp8_loop_filter_simple_horizontal_edge_c(y_ptr.offset(4 * y_stride as isize), y_stride, blimit);
    vp8_loop_filter_simple_horizontal_edge_c(y_ptr.offset(8 * y_stride as isize), y_stride, blimit);
    vp8_loop_filter_simple_horizontal_edge_c(
        y_ptr.offset(12 * y_stride as isize),
        y_stride,
        blimit,
    );
}

/// `vp8_loop_filter_bv_c` — vertical B filtering.
/// `loopfilter_filters.c:371`.
///
/// # Safety
/// `lfi` must point to a valid `LoopFilterInfo`. `u_ptr`/`v_ptr` may be
/// null.
pub unsafe fn vp8_loop_filter_bv_c(
    y_ptr: *mut u8,
    u_ptr: *mut u8,
    v_ptr: *mut u8,
    y_stride: i32,
    uv_stride: i32,
    lfi: *mut LoopFilterInfo,
) {
    loop_filter_vertical_edge_c(
        y_ptr.offset(4),
        y_stride,
        (*lfi).blim,
        (*lfi).lim,
        (*lfi).hev_thr,
        2,
    );
    loop_filter_vertical_edge_c(
        y_ptr.offset(8),
        y_stride,
        (*lfi).blim,
        (*lfi).lim,
        (*lfi).hev_thr,
        2,
    );
    loop_filter_vertical_edge_c(
        y_ptr.offset(12),
        y_stride,
        (*lfi).blim,
        (*lfi).lim,
        (*lfi).hev_thr,
        2,
    );

    if !u_ptr.is_null() {
        loop_filter_vertical_edge_c(
            u_ptr.offset(4),
            uv_stride,
            (*lfi).blim,
            (*lfi).lim,
            (*lfi).hev_thr,
            1,
        );
    }

    if !v_ptr.is_null() {
        loop_filter_vertical_edge_c(
            v_ptr.offset(4),
            uv_stride,
            (*lfi).blim,
            (*lfi).lim,
            (*lfi).hev_thr,
            1,
        );
    }
}

/// `vp8_loop_filter_bvs_c`. `loopfilter_filters.c:392`.
///
/// # Safety
/// `blimit` must point to at least one byte.
pub unsafe fn vp8_loop_filter_bvs_c(y_ptr: *mut u8, y_stride: i32, blimit: *const u8) {
    vp8_loop_filter_simple_vertical_edge_c(y_ptr.offset(4), y_stride, blimit);
    vp8_loop_filter_simple_vertical_edge_c(y_ptr.offset(8), y_stride, blimit);
    vp8_loop_filter_simple_vertical_edge_c(y_ptr.offset(12), y_stride, blimit);
}
