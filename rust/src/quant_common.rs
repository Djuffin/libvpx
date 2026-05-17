//! VP8 quantizer step-size lookups.
//!
//! Literal translation of `vp8/common/quant_common.c`. Six tiny
//! accessors map a quantizer index (QI, 0..127) plus a signed delta to
//! the integer dequant step used by the IDCT path. The backing tables
//! `DC_QLOOKUP` / `AC_QLOOKUP` live in [`crate::tables`]; this module
//! only carries the clamp + per-channel correction logic specified by
//! RFC 6386 §9.6.
//!
//! See `documentation/vp8_files/quant_common.md` for the per-function
//! rationale (Y2 DC × 2, Y2 AC × 155 % via a 16-bit fixed-point
//! multiply with a floor of 8, chroma DC ceiling of 132, etc.).

#![allow(non_snake_case)]

use crate::tables::{AC_QLOOKUP, DC_QLOOKUP};

/// Clamp `QIndex + Delta` to `[0, 127]`, matching the saturating
/// arithmetic mandated by RFC 6386 §9.6.
#[inline]
fn clamp_qindex(QIndex: i32, Delta: i32) -> usize {
    let mut q = QIndex + Delta;
    if q > 127 {
        q = 127;
    } else if q < 0 {
        q = 0;
    }
    q as usize
}

/// `vp8_dc_quant` — luma Y plane DC step size.
/// Source: `vp8/common/quant_common.c:37`.
pub fn vp8_dc_quant(QIndex: i32, Delta: i32) -> i32 {
    let q = clamp_qindex(QIndex, Delta);
    DC_QLOOKUP[q]
}

/// `vp8_dc2quant` — Y2 (Walsh-Hadamard) plane DC step size.
/// Source: `vp8/common/quant_common.c:52`.
pub fn vp8_dc2quant(QIndex: i32, Delta: i32) -> i32 {
    let q = clamp_qindex(QIndex, Delta);
    DC_QLOOKUP[q] * 2
}

/// `vp8_dc_uv_quant` — chroma UV plane DC step size, capped at 132.
/// Source: `vp8/common/quant_common.c:66`.
pub fn vp8_dc_uv_quant(QIndex: i32, Delta: i32) -> i32 {
    let q = clamp_qindex(QIndex, Delta);
    let mut retval = DC_QLOOKUP[q];
    if retval > 132 {
        retval = 132;
    }
    retval
}

/// `vp8_ac_yquant` — luma Y plane AC step size (no delta; Y AC is the
/// reference channel for all other deltas).
/// Source: `vp8/common/quant_common.c:84`.
pub fn vp8_ac_yquant(QIndex: i32) -> i32 {
    let mut q = QIndex;
    if q > 127 {
        q = 127;
    } else if q < 0 {
        q = 0;
    }
    AC_QLOOKUP[q as usize]
}

/// `vp8_ac2quant` — Y2 (Walsh-Hadamard) plane AC step size,
/// `ac * 155 / 100` floored at 8.
///
/// The `* 101581 >> 16` is bit-exact with `x * 155 / 100` for every
/// `x in [0..284]` (the full range of `AC_QLOOKUP`) — see the comment
/// in the C source.
///
/// Source: `vp8/common/quant_common.c:97`.
pub fn vp8_ac2quant(QIndex: i32, Delta: i32) -> i32 {
    let q = clamp_qindex(QIndex, Delta);
    let mut retval = (AC_QLOOKUP[q] * 101581) >> 16;
    if retval < 8 {
        retval = 8;
    }
    retval
}

/// `vp8_ac_uv_quant` — chroma UV plane AC step size.
/// Source: `vp8/common/quant_common.c:117`.
pub fn vp8_ac_uv_quant(QIndex: i32, Delta: i32) -> i32 {
    let q = clamp_qindex(QIndex, Delta);
    AC_QLOOKUP[q]
}
