//! Header-only inline wrappers from `vp8/decoder/treereader.h`.
//!
//! Every entity in the C header is either a typedef, a `#define` macro,
//! or a `static INLINE` function — so nothing here corresponds to an
//! `.o` in the C build, but every name is referenced from
//! `decodemv.c` / `decodeframe.c` and must exist as a Rust symbol.

use crate::dboolhuff::{vp8_decode_value, vp8dx_decode_bool};
use crate::tables::{PROB_HALF, Prob, TreeIndex};
use crate::types::BoolDecoder;

/// `typedef BOOL_DECODER vp8_reader;` — the bitstream-side bool decoder.
pub type vp8_reader<'a> = BoolDecoder<'a>;

/// `#define vp8_read vp8dx_decode_bool` — read one binary symbol against
/// an 8-bit probability.
#[inline]
pub fn vp8_read(r: &mut vp8_reader<'_>, prob: i32) -> i32 {
    vp8dx_decode_bool(r, prob)
}

/// `#define vp8_read_literal vp8_decode_value` — read `bits` raw bits
/// (each at prob = 128).
#[inline]
pub fn vp8_read_literal(r: &mut vp8_reader<'_>, bits: i32) -> i32 {
    vp8_decode_value(r, bits)
}

/// `#define vp8_read_bit(R) vp8_read(R, vp8_prob_half)` — one fair coin
/// flip.
#[inline]
pub fn vp8_read_bit(r: &mut vp8_reader<'_>) -> i32 {
    vp8_read(r, PROB_HALF as i32)
}

/// `static INLINE int vp8_treed_read(...)` (treereader.h:30). Walks a
/// libvpx tree: positive indices are jumps within the tree array,
/// non-positive indices are terminal leaves whose value is `-i`.
#[inline]
pub fn vp8_treed_read(r: &mut vp8_reader<'_>, t: *const TreeIndex, p: *const Prob) -> i32 {
    let mut i: TreeIndex = 0;
    // SAFETY: `t` / `p` are generated tree/probability tables; the tree
    // is well-formed so `i` stays within bounds as we walk to a leaf.
    unsafe {
        loop {
            let bit = vp8_read(r, *p.offset((i >> 1) as isize) as i32);
            i = *t.offset((i as isize) + (bit as isize));
            if i <= 0 {
                return -(i as i32);
            }
        }
    }
}
