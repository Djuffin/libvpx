//! Per-macroblock block-descriptor wiring.
//!
//! Literal translation of `vp8/common/mbpitch.c`. Sets up the pointer
//! and offset fields of `MACROBLOCKD::block[0..25]` so that downstream
//! passes (token decode, dequantize, IDCT, intra/inter prediction, and
//! reconstruction) can address each 4x4 sub-block uniformly via the
//! `BLOCKD` array.

use crate::types::Macroblockd;

/// `vp8_setup_block_dptrs` — wires per-block scratch pointers
/// (`predictor`, `qcoeff`, `dqcoeff`, `eob`) into the parent
/// `MACROBLOCKD`'s flat backing arrays.
///
/// Source: `vp8/common/mbpitch.c:13`.
pub unsafe fn vp8_setup_block_dptrs(x: *mut Macroblockd) {
    for r in 0..4i32 {
        for c in 0..4i32 {
            (*x).block[(r * 4 + c) as usize].predictor = (*x)
                .predictor
                .as_mut_ptr()
                .offset((r * 4 * 16 + c * 4) as isize);
        }
    }

    for r in 0..2i32 {
        for c in 0..2i32 {
            (*x).block[(16 + r * 2 + c) as usize].predictor = (*x)
                .predictor
                .as_mut_ptr()
                .offset((256 + r * 4 * 8 + c * 4) as isize);
        }
    }

    for r in 0..2i32 {
        for c in 0..2i32 {
            (*x).block[(20 + r * 2 + c) as usize].predictor = (*x)
                .predictor
                .as_mut_ptr()
                .offset((320 + r * 4 * 8 + c * 4) as isize);
        }
    }

    for r in 0..25i32 {
        (*x).block[r as usize].qcoeff = (*x).qcoeff.as_mut_ptr().offset((r * 16) as isize);
        (*x).block[r as usize].dqcoeff = (*x).dqcoeff.as_mut_ptr().offset((r * 16) as isize);
        (*x).block[r as usize].eob = (*x).eobs.as_mut_ptr().offset(r as isize);
    }
}

/// `vp8_build_block_doffsets` — wires the destination-frame
/// (`x->dst`) byte offsets into `BLOCKD::offset`. Must be re-invoked
/// whenever `dst.y_stride` or `dst.uv_stride` changes.
///
/// Source: `vp8/common/mbpitch.c:43`.
pub unsafe fn vp8_build_block_doffsets(x: *mut Macroblockd) {
    // y blocks
    for block in 0..16i32 {
        (*x).block[block as usize].offset = (block >> 2) * 4 * (*x).dst.y_stride + (block & 3) * 4;
    }

    // U and V blocks
    for block in 16..20i32 {
        let off = ((block - 16) >> 1) * 4 * (*x).dst.uv_stride + (block & 1) * 4;
        (*x).block[block as usize].offset = off;
        (*x).block[(block + 4) as usize].offset = off;
    }
}
