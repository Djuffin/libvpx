//! `vp8/common/blockd.c` — block-to-context index tables.
//!
//! Two 25-byte read-only arrays, `vp8_block2left` and `vp8_block2above`, map a
//! 4x4 sub-block index `b in [0,24]` to the index of the corresponding
//! left/above neighbour entropy context. The values live in `tables.rs`; this
//! module re-exports them under their original C names.

#![allow(dead_code)]
#![allow(non_upper_case_globals)]

pub use crate::tables::{VP8_BLOCK2ABOVE as vp8_block2above, VP8_BLOCK2LEFT as vp8_block2left};
