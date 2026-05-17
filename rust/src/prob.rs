//! Probability arithmetic and the renormalisation table.
//!
//! Literal Rust transliteration of `vpx_dsp/prob.c` and the inline
//! helpers in `vpx_dsp/prob.h`. See `documentation/vp8_files/prob.md`
//! for the prose walkthrough.
//!
//! In a VP8-only decoder build only `vpx_norm[256]` is actually
//! reached at run time (and even then via VP8's own `vp8_norm`
//! mirror in `vp8/decoder/dboolhuff.c`). `vpx_tree_merge_probs` and
//! the probability-arithmetic helpers are link-reachable but unused
//! by the VP8 decoder; they ship here because `vpx_dsp/` is shared
//! across VP8/VP9 and their encoders.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use crate::tables::{Prob, TreeIndex};

/// `vpx_prob` from `prob.h` — 8-bit probability in `[1, 255]`.
pub type VpxProb = Prob;

/// `vpx_tree_index` from `prob.h` — signed tree node index.
pub type VpxTreeIndex = TreeIndex;

/// `MAX_PROB` (`prob.h`).
pub const MAX_PROB: i32 = 255;

/// `vpx_prob_half` (`prob.h`).
pub const vpx_prob_half: VpxProb = 128;

/// `MODE_MV_COUNT_SAT` (`prob.h`).
pub const MODE_MV_COUNT_SAT: u32 = 20;

/// `vpx_complement(x)` macro (`prob.h`).
#[inline]
pub fn vpx_complement(x: i32) -> i32 {
    255 - x
}

/// `TREE_SIZE(leaf_count)` macro (`prob.h`).
#[inline]
pub const fn TREE_SIZE(leaf_count: usize) -> usize {
    2 * leaf_count - 2
}

// ---------------------------------------------------------------------------
// vpx_norm[256] — the bool-decoder renormalisation table.
// ---------------------------------------------------------------------------

/// `vpx_norm` (`prob.c`) — renormalisation lookup. `vpx_norm[r]` is
/// the number of left shifts required to drive an 8-bit `range`
/// register back into the canonical `[128, 256)` interval. See
/// `prob.md` for the full table semantics.
///
/// In the C source this is declared `DECLARE_ALIGNED(16, …)`; the
/// alignment is purely a cache-line hint and has no effect on
/// correctness. Rust's default array placement is fine.
#[rustfmt::skip]
pub static vpx_norm: [u8; 256] = [
    0, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];

// ---------------------------------------------------------------------------
// Inline helpers from prob.h.
// ---------------------------------------------------------------------------

/// `get_prob(num, den)` (`prob.h`) — frequency-to-probability with
/// branchless `[1, 255]` clamping.
#[inline]
pub fn get_prob(num: u32, den: u32) -> VpxProb {
    debug_assert!(den != 0);
    let p: i32 = (((num as u64) * 256 + ((den >> 1) as u64)) / (den as u64)) as i32;
    // (p > 255) ? 255 : (p < 1) ? 1 : p;
    let clipped_prob: i32 = p | ((255 - p) >> 23) | ((p == 0) as i32);
    clipped_prob as VpxProb
}

/// `get_binary_prob(n0, n1)` (`prob.h`).
#[inline]
pub fn get_binary_prob(n0: u32, n1: u32) -> VpxProb {
    let den = n0 + n1;
    if den == 0 {
        return 128u8;
    }
    get_prob(n0, den)
}

/// `ROUND_POWER_OF_TWO(x, 8)` from `vpx_dsp_common.h`, specialised
/// for the use site below: `(x + (1 << (n - 1))) >> n`.
#[inline]
fn round_power_of_two(value: i32, n: i32) -> i32 {
    (value + (1 << (n - 1))) >> n
}

/// `weighted_prob(prob1, prob2, factor)` (`prob.h`). Assumes both
/// probabilities are already in `[1, 255]`.
#[inline]
pub fn weighted_prob(prob1: i32, prob2: i32, factor: i32) -> VpxProb {
    round_power_of_two(prob1 * (256 - factor) + prob2 * factor, 8) as VpxProb
}

/// `VPXMIN` macro from `vpx_dsp_common.h`.
#[inline]
fn vpxmin(a: u32, b: u32) -> u32 {
    if a < b {
        a
    } else {
        b
    }
}

/// `merge_probs(pre_prob, ct, count_sat, max_update_factor)` (`prob.h`).
#[inline]
pub fn merge_probs(
    pre_prob: VpxProb,
    ct: &[u32; 2],
    count_sat: u32,
    max_update_factor: u32,
) -> VpxProb {
    let prob: VpxProb = get_binary_prob(ct[0], ct[1]);
    let count: u32 = vpxmin(ct[0] + ct[1], count_sat);
    let factor: u32 = max_update_factor * count / count_sat;
    weighted_prob(pre_prob as i32, prob as i32, factor as i32)
}

/// `count_to_update_factor` (`prob.h`) —
/// `MODE_MV_MAX_UPDATE_FACTOR (128) * count / MODE_MV_COUNT_SAT`.
#[rustfmt::skip]
static count_to_update_factor: [i32; (MODE_MV_COUNT_SAT + 1) as usize] = [
    0,  6,  12, 19, 25, 32,  38,  44,  51,  57, 64,
    70, 76, 83, 89, 96, 102, 108, 115, 121, 128,
];

/// `mode_mv_merge_probs(pre_prob, ct)` (`prob.h`).
#[inline]
pub fn mode_mv_merge_probs(pre_prob: VpxProb, ct: &[u32; 2]) -> VpxProb {
    let den: u32 = ct[0] + ct[1];
    if den == 0 {
        pre_prob
    } else {
        let count: u32 = vpxmin(den, MODE_MV_COUNT_SAT);
        let factor: i32 = count_to_update_factor[count as usize];
        let prob: VpxProb = get_prob(ct[0], den);
        weighted_prob(pre_prob as i32, prob as i32, factor)
    }
}

// ---------------------------------------------------------------------------
// vpx_tree_merge_probs and its recursive worker.
// ---------------------------------------------------------------------------

/// `tree_merge_probs_impl` (`prob.c`) — post-order DFS worker.
///
/// Pointer-heavy C: `tree`, `pre_probs`, `counts`, and `probs` are
/// raw pointers walked by signed/unsigned index. Translated as
/// `unsafe` to keep the bookkeeping byte-for-byte identical with the
/// original.
unsafe fn tree_merge_probs_impl(
    i: u32,
    tree: *const VpxTreeIndex,
    pre_probs: *const VpxProb,
    counts: *const u32,
    probs: *mut VpxProb,
) -> u32 {
    let l: i32 = *tree.offset(i as isize) as i32;
    let left_count: u32 = if l <= 0 {
        *counts.offset((-l) as isize)
    } else {
        tree_merge_probs_impl(l as u32, tree, pre_probs, counts, probs)
    };
    let r: i32 = *tree.offset((i + 1) as isize) as i32;
    let right_count: u32 = if r <= 0 {
        *counts.offset((-r) as isize)
    } else {
        tree_merge_probs_impl(r as u32, tree, pre_probs, counts, probs)
    };
    let ct: [u32; 2] = [left_count, right_count];
    *probs.offset((i >> 1) as isize) =
        mode_mv_merge_probs(*pre_probs.offset((i >> 1) as isize), &ct);
    left_count + right_count
}

/// `vpx_tree_merge_probs` (`prob.c`) — public trampoline. Starts the
/// recursion at node 0 and discards the aggregate count returned by
/// the worker.
///
/// VP8-only builds never reach this function (VP8 transmits
/// probability updates explicitly in the frame header instead of
/// performing backward adaptation); it is kept for symbol parity
/// with the C library.
pub unsafe fn vpx_tree_merge_probs(
    tree: *const VpxTreeIndex,
    pre_probs: *const VpxProb,
    counts: *const u32,
    probs: *mut VpxProb,
) {
    tree_merge_probs_impl(0, tree, pre_probs, counts, probs);
}
