//! `vp8/common/blockd.c` — block-to-context index tables.
//!
//! The original C translation unit contributes only two 25-byte read-only
//! arrays, `vp8_block2left` and `vp8_block2above`, that map a 4x4 sub-block
//! index `b in [0,24]` to the byte offset inside an `ENTROPY_CONTEXT_PLANES`
//! struct holding the corresponding left/above neighbour "any non-zero
//! coefficient" flag. See `documentation/vp8_files/blockd.md` for the full
//! derivation; the values themselves live in `tables.rs` because they are
//! pure bitstream/struct-layout constants.
//!
//! This module re-exports those tables under their original C names so that
//! call sites translated from libvpx can be transliterated verbatim.

#![allow(dead_code)]
#![allow(non_upper_case_globals)]

pub use crate::tables::{VP8_BLOCK2ABOVE as vp8_block2above, VP8_BLOCK2LEFT as vp8_block2left};
