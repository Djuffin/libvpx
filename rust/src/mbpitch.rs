//! Per-macroblock block-descriptor wiring.
//!
//! Literal translation of `vp8/common/mbpitch.c`. Sets up the
//! `offset` field of `MACROBLOCKD::block[0..25]` so that downstream
//! passes (intra/inter prediction, reconstruction) can address each
//! 4x4 sub-block uniformly via the `BLOCKD` array.
//!
//! The C source also has `vp8_setup_block_dptrs` which wires
//! `qcoeff`/`dqcoeff`/`predictor`/`eob` pointer aliases per block.
//! All five fields are encoder-only convenience caches into
//! `Macroblockd`'s flat arrays; the decoder-only Rust port omits both
//! the fields and the setup function.

use crate::types::Macroblockd;

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
