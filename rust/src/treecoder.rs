//! Generic binary tree coder utility.
//!
//! Literal Rust transliteration of `vp8/common/treecoder.c`. See
//! `documentation/vp8_files/treecoder.md` for an exhaustive prose
//! description of the tree-as-`i8[]` encoding and the algorithms below.
//!
//! This file is the *generic* helper that sits behind every tree-coded
//! VP8 syntax element. It contains no wire-format I/O; the on-wire
//! walker is `vp8_treed_read` in `treereader.h`. What lives here:
//!
//!  * `tree2tok` / `vp8_tokens_from_tree` / `vp8_tokens_from_tree_offset`
//!    — derive the leaf-symbol → (codeword, length) table from a tree.
//!  * `branch_counts` — fold a per-leaf event histogram into per-node
//!    `(left, right)` branch counts.
//!  * `vp8_tree_probs_from_distribution` — turn those counts into the
//!    8-bit per-node probabilities used on the wire.
//!
//! In the pure-decoder build `vp8_tree_probs_from_distribution` is dead
//! code, and `vp8_tokens_from_tree` is called exactly once (from
//! `vp8_coef_tree_initialize` in `entropy.c`). They ship in `common/`
//! because the tree-array format is shared with the encoder.

#![allow(dead_code)]
#![allow(non_snake_case)]

use crate::tables::{Prob, Token, TreeIndex, PROB_HALF};

/// `tree2tok` — recursive DFS that fills `p[-leaf]` with the
/// (codeword, length) pair implied by the shape of `t`.
///
/// `p` is indexed by *leaf symbol value*, hence the `p[-j]` stores in
/// the C source. `i` is the byte offset of the next child slot to
/// inspect; `v` is the codeword built so far (MSB-first); `L` is its
/// length. See `treecoder.md` § "tree2tok" for the full derivation.
unsafe fn tree2tok(p: *mut Token, t: *const TreeIndex, mut i: i32, mut v: i32, mut L: i32) {
    v += v;
    L += 1;

    loop {
        let j: TreeIndex = unsafe { *t.offset(i as isize) };
        i += 1;

        if j <= 0 {
            unsafe {
                let slot = p.offset(-(j as isize));
                (*slot).value = v;
                (*slot).len = L;
            }
        } else {
            unsafe {
                tree2tok(p, t, j as i32, v, L);
            }
        }

        v += 1;
        if v & 1 == 0 {
            break;
        }
    }
}

/// `vp8_tokens_from_tree` — public wrapper that starts the recursion at
/// the root. Assumes the symbol alphabet starts at 0.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vp8_tokens_from_tree(p: *mut Token, t: *const TreeIndex) {
    unsafe {
        tree2tok(p, t, 0, 0, 0);
    }
}

/// `vp8_tokens_from_tree_offset` — like `vp8_tokens_from_tree` but
/// shifts `p` so that leaves whose symbol value starts at `offset`
/// still land in the caller's array.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vp8_tokens_from_tree_offset(
    p: *mut Token,
    t: *const TreeIndex,
    offset: i32,
) {
    unsafe {
        tree2tok(p.offset(-(offset as isize)), t, 0, 0, 0);
    }
}

/// `branch_counts` — for each interior node `j`, accumulate the number
/// of times the encoder went left vs right by replaying every leaf's
/// codeword through the tree.
///
/// `tok` must already describe `tree` (i.e. the caller has first run
/// `vp8_tokens_from_tree(tok, tree)`). Pre-zeroes `branch_ct`.
unsafe fn branch_counts(
    n: i32,
    tok: *const Token,
    tree: *const TreeIndex,
    branch_ct: *mut [u32; 2],
    num_events: *const u32,
) {
    let tree_len = n - 1;
    let mut t: i32 = 0;

    debug_assert!(tree_len != 0);

    loop {
        unsafe {
            (*branch_ct.offset(t as isize))[0] = 0;
            (*branch_ct.offset(t as isize))[1] = 0;
        }
        t += 1;
        if t >= tree_len {
            break;
        }
    }

    t = 0;

    loop {
        let mut L: i32 = unsafe { (*tok.offset(t as isize)).len };
        let enc: i32 = unsafe { (*tok.offset(t as isize)).value };
        let ct: u32 = unsafe { *num_events.offset(t as isize) };

        let mut i: TreeIndex = 0;

        loop {
            L -= 1;
            let b: i32 = (enc >> L) & 1;
            let j: i32 = (i as i32) >> 1;
            debug_assert!(j < tree_len && 0 <= L);

            unsafe {
                (*branch_ct.offset(j as isize))[b as usize] += ct;
                i = *tree.offset((i as i32 + b) as isize);
            }

            if i <= 0 {
                break;
            }
        }

        debug_assert!(L == 0);

        t += 1;
        if t >= n {
            break;
        }
    }
}

/// `vp8_tree_probs_from_distribution` — turn a per-leaf event histogram
/// into per-interior-node 8-bit probabilities for the wire.
///
/// Also writes the raw `(left, right)` counts into `branch_ct` so the
/// encoder can later weigh the bit-cost of sending a probability
/// update against the cost of keeping the previous frame's value.
///
/// `Pfactor` scales the numerator (conventionally 255 for the 8-bit
/// bool coder); `Round` controls rounding direction.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vp8_tree_probs_from_distribution(
    n: i32,
    tok: *const Token,
    tree: *const TreeIndex,
    probs: *mut Prob,
    branch_ct: *mut [u32; 2],
    num_events: *const u32,
    Pfactor: u32,
    Round: i32,
) {
    let tree_len = n - 1;
    let mut t: i32 = 0;

    unsafe {
        branch_counts(n, tok, tree, branch_ct, num_events);
    }

    loop {
        let c = unsafe { &*branch_ct.offset(t as isize) };
        let tot: u32 = c[0] + c[1];

        if tot != 0 {
            let p: u32 = (((c[0] as u64) * (Pfactor as u64))
                + (if Round != 0 { (tot >> 1) as u64 } else { 0 })) as u32
                / tot;
            unsafe {
                *probs.offset(t as isize) = if p < 256 {
                    if p != 0 { p as Prob } else { 1 }
                } else {
                    255
                };
            }
        } else {
            unsafe {
                *probs.offset(t as isize) = PROB_HALF;
            }
        }

        t += 1;
        if t >= tree_len {
            break;
        }
    }
}
