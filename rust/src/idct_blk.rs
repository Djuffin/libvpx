//! Per-block IDCT dispatch over a macroblock — literal translation of
//! `vp8/common/idct_blk.c`.
//!
//! Two reference C dispatchers live here:
//!   * [`vp8_dequant_idct_add_y_block_c`] — walks the 16 Y blocks of
//!     a macroblock in raster order.
//!   * [`vp8_dequant_idct_add_uv_block_c`] — walks the 4 U then 4 V
//!     chroma blocks.
//!
//! Each block is dispatched per its `eob` index to either the full
//! dequant+IDCT+add kernel ([`vp8_dequant_idct_add_c`]) or the
//! DC-only shortcut ([`vp8_dc_only_idct_add_c`]). See
//! `documentation/vp8_files/idct_blk.md` for the exact contract.

#![allow(non_snake_case)]

use core::ptr;

use crate::dequantize::vp8_dequant_idct_add_c;
use crate::idctllm::vp8_dc_only_idct_add_c;

// ---------------------------------------------------------------------------
// Public kernels
// ---------------------------------------------------------------------------

/// `vp8_dequant_idct_add_y_block_c` — vp8/common/idct_blk.c:15.
///
/// Walks the 16 4x4 Y blocks of one macroblock in raster order.
/// For each block, dispatches to the full IDCT path when `eob > 1`,
/// or to the DC-only shortcut otherwise. The shortcut path also
/// clears `q[0]` and `q[1]` so the per-frame `qcoeff[]` clear is
/// maintained.
pub unsafe fn vp8_dequant_idct_add_y_block_c(
    mut q: *mut i16,
    dq: *mut i16,
    mut dst: *mut u8,
    stride: i32,
    mut eobs: *mut i8,
) {
    let mut i: i32;
    let mut j: i32;

    i = 0;
    while i < 4 {
        j = 0;
        while j < 4 {
            let eob = *eobs;
            eobs = eobs.offset(1);
            if eob > 1 {
                vp8_dequant_idct_add_c(q, dq, dst, stride);
            } else {
                vp8_dc_only_idct_add_c(
                    ((*q.offset(0) as i32).wrapping_mul(*dq.offset(0) as i32)) as i16,
                    dst,
                    stride,
                    dst,
                    stride,
                );
                ptr::write_bytes(q as *mut u8, 0, 2 * core::mem::size_of::<i16>());
            }

            q = q.offset(16);
            dst = dst.offset(4);
            j += 1;
        }

        dst = dst.offset((4 * stride - 16) as isize);
        i += 1;
    }
}

/// `vp8_dequant_idct_add_uv_block_c` — vp8/common/idct_blk.c:36.
///
/// Walks the 4 U then 4 V chroma blocks in 2x2 raster order per
/// plane. `q` and `eobs` step contiguously through U-then-V; `dst_u`
/// and `dst_v` index the two chroma planes (sharing `stride`).
pub unsafe fn vp8_dequant_idct_add_uv_block_c(
    mut q: *mut i16,
    dq: *mut i16,
    mut dst_u: *mut u8,
    mut dst_v: *mut u8,
    stride: i32,
    mut eobs: *mut i8,
) {
    let mut i: i32;
    let mut j: i32;

    i = 0;
    while i < 2 {
        j = 0;
        while j < 2 {
            let eob = *eobs;
            eobs = eobs.offset(1);
            if eob > 1 {
                vp8_dequant_idct_add_c(q, dq, dst_u, stride);
            } else {
                vp8_dc_only_idct_add_c(
                    ((*q.offset(0) as i32).wrapping_mul(*dq.offset(0) as i32)) as i16,
                    dst_u,
                    stride,
                    dst_u,
                    stride,
                );
                ptr::write_bytes(q as *mut u8, 0, 2 * core::mem::size_of::<i16>());
            }

            q = q.offset(16);
            dst_u = dst_u.offset(4);
            j += 1;
        }

        dst_u = dst_u.offset((4 * stride - 8) as isize);
        i += 1;
    }

    i = 0;
    while i < 2 {
        j = 0;
        while j < 2 {
            let eob = *eobs;
            eobs = eobs.offset(1);
            if eob > 1 {
                vp8_dequant_idct_add_c(q, dq, dst_v, stride);
            } else {
                vp8_dc_only_idct_add_c(
                    ((*q.offset(0) as i32).wrapping_mul(*dq.offset(0) as i32)) as i16,
                    dst_v,
                    stride,
                    dst_v,
                    stride,
                );
                ptr::write_bytes(q as *mut u8, 0, 2 * core::mem::size_of::<i16>());
            }

            q = q.offset(16);
            dst_v = dst_v.offset(4);
            j += 1;
        }

        dst_v = dst_v.offset((4 * stride - 8) as isize);
        i += 1;
    }
}
