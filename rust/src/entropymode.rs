//! `vp8/common/entropymode.c` — mode-probability defaults, tree shapes,
//! and the `vp8_mv_cont` neighbour-MV classifier.
//!
//! The trees, probability tables, and MB-split layouts referenced here
//! are defined in [`crate::tables`]. This module re-exports them under
//! the original C identifiers and provides the three functions from the
//! original `.c` file:
//!
//! - [`vp8_mv_cont`] — five-way classification of `(left_mv, above_mv)`
//!   used to pick a row of `vp8_sub_mv_ref_prob2` when decoding a
//!   SPLITMV sub-block;
//! - [`vp8_init_mbmode_probs`] — reset the three mutable per-frame
//!   mode-probability tables on `VP8_COMMON::fc` to their defaults;
//! - [`vp8_default_bmode_probs`] — copy the default 4x4-intra-mode
//!   probabilities into a caller-supplied buffer.

#![allow(dead_code)]
#![allow(non_upper_case_globals)]

use crate::tables::{
    Prob, SUB_MV_REF_PROB, SUBMVREF_COUNT, TreeIndex, VP8_BINTRAMODES, VP8_BMODE_PROB,
    VP8_BMODE_TREE, VP8_KF_BMODE_PROB, VP8_KF_UV_MODE_PROB, VP8_KF_YMODE_PROB, VP8_KF_YMODE_TREE,
    VP8_MBSPLIT_TREE, VP8_MBSPLITS, VP8_MV_REF_TREE, VP8_NUMMBSPLITS, VP8_SMALL_MVTREE,
    VP8_SUB_MV_REF_TREE, VP8_SUBMVREFS, VP8_UV_MODE_PROB, VP8_UV_MODE_TREE, VP8_UV_MODES,
    VP8_YMODE_PROB, VP8_YMODE_TREE, VP8_YMODES,
};
use crate::types::{Mv, Vp8Common};

// ===========================================================================
// `sumvfref_t` enum (entropymode.h:21-27)
// ===========================================================================

/// `sumvfref_t` — five-way classification of `(left_mv, above_mv)` used
/// to select a row of [`vp8_sub_mv_ref_prob2`]. RFC 6386 §16.3.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(i32)]
pub enum SumvfrefT {
    SubmvrefNormal = 0,
    SubmvrefLeftZed = 1,
    SubmvrefAboveZed = 2,
    SubmvrefLeftAboveSame = 3,
    SubmvrefLeftAboveZed = 4,
}

// Aliases matching the C enumerators verbatim.
pub const SUBMVREF_NORMAL: i32 = SumvfrefT::SubmvrefNormal as i32;
pub const SUBMVREF_LEFT_ZED: i32 = SumvfrefT::SubmvrefLeftZed as i32;
pub const SUBMVREF_ABOVE_ZED: i32 = SumvfrefT::SubmvrefAboveZed as i32;
pub const SUBMVREF_LEFT_ABOVE_SAME: i32 = SumvfrefT::SubmvrefLeftAboveSame as i32;
pub const SUBMVREF_LEFT_ABOVE_ZED: i32 = SumvfrefT::SubmvrefLeftAboveZed as i32;

// ===========================================================================
// Re-exports under the original C names. All storage lives in
// `crate::tables`.
// ===========================================================================

/// `vp8_mbsplit` typedef (entropymode.h:29).
pub type vp8_mbsplit = [i32; 16];

/// `VP8_NUMMBSPLITS` (entropymode.h:31).
pub const VP8_NUMMBSPLITS_C: usize = VP8_NUMMBSPLITS;

/// `vp8_mbsplits[VP8_NUMMBSPLITS]` (entropymode.c:45).
pub const vp8_mbsplits: [vp8_mbsplit; VP8_NUMMBSPLITS] = VP8_MBSPLITS;

/// `SUBMVREF_COUNT` (entropymode.h:40).
pub const SUBMVREF_COUNT_C: usize = SUBMVREF_COUNT;

/// `vp8_bmode_tree[18]` (entropymode.c:58).
pub const vp8_bmode_tree: [TreeIndex; 18] = VP8_BMODE_TREE;

/// `vp8_ymode_tree[8]` (entropymode.c:74).
pub const vp8_ymode_tree: [TreeIndex; 8] = VP8_YMODE_TREE;

/// `vp8_kf_ymode_tree[8]` (entropymode.c:78).
pub const vp8_kf_ymode_tree: [TreeIndex; 8] = VP8_KF_YMODE_TREE;

/// `vp8_uv_mode_tree[6]` (entropymode.c:82).
pub const vp8_uv_mode_tree: [TreeIndex; 6] = VP8_UV_MODE_TREE;

/// `vp8_mbsplit_tree[6]` (entropymode.c:85).
pub const vp8_mbsplit_tree: [TreeIndex; 6] = VP8_MBSPLIT_TREE;

/// `vp8_mv_ref_tree[8]` (entropymode.c:87).
pub const vp8_mv_ref_tree: [TreeIndex; 8] = VP8_MV_REF_TREE;

/// `vp8_sub_mv_ref_tree[6]` (entropymode.c:90).
pub const vp8_sub_mv_ref_tree: [TreeIndex; 6] = VP8_SUB_MV_REF_TREE;

/// `vp8_small_mvtree[14]` (entropymode.c:93).
pub const vp8_small_mvtree: [TreeIndex; 14] = VP8_SMALL_MVTREE;

// ---- Key-frame default mode probability tables. ----

/// `vp8_kf_bmode_prob[VP8_BINTRAMODES][VP8_BINTRAMODES][VP8_BINTRAMODES - 1]`
/// (vp8_entropymodedata.h).
pub const vp8_kf_bmode_prob: [[[Prob; VP8_BINTRAMODES - 1]; VP8_BINTRAMODES]; VP8_BINTRAMODES] =
    VP8_KF_BMODE_PROB;

/// `vp8_kf_uv_mode_prob[VP8_UV_MODES - 1]` (vp8_entropymodedata.h).
pub const vp8_kf_uv_mode_prob: [Prob; VP8_UV_MODES - 1] = VP8_KF_UV_MODE_PROB;

/// `vp8_kf_ymode_prob[VP8_YMODES - 1]` (vp8_entropymodedata.h).
pub const vp8_kf_ymode_prob: [Prob; VP8_YMODES - 1] = VP8_KF_YMODE_PROB;

// ---- Inter-frame defaults consumed by `vp8_init_mbmode_probs`. ----

/// `vp8_ymode_prob[VP8_YMODES - 1]` (vp8_entropymodedata.h).
pub const vp8_ymode_prob: [Prob; VP8_YMODES - 1] = VP8_YMODE_PROB;

/// `vp8_uv_mode_prob[VP8_UV_MODES - 1]` (vp8_entropymodedata.h).
pub const vp8_uv_mode_prob: [Prob; VP8_UV_MODES - 1] = VP8_UV_MODE_PROB;

/// `vp8_bmode_prob[VP8_BINTRAMODES - 1]` (vp8_entropymodedata.h).
pub const vp8_bmode_prob: [Prob; VP8_BINTRAMODES - 1] = VP8_BMODE_PROB;

/// File-local default sub-MV-ref probability vector (entropymode.c:35).
/// Copied into `cm->fc.sub_mv_ref_prob` by [`vp8_init_mbmode_probs`].
const sub_mv_ref_prob: [Prob; VP8_SUBMVREFS - 1] = SUB_MV_REF_PROB;

// ===========================================================================
// Functions
// ===========================================================================

/// Pack an [`Mv`] like the C `int_mv` union's `as_int` field: low 16
/// bits = `row`, high 16 bits = `col`. Used only for equality / zero
/// comparisons.
#[inline]
fn mv_as_int(m: &Mv) -> u32 {
    ((m.col as u16 as u32) << 16) | (m.row as u16 as u32)
}

/// `vp8_mv_cont` (entropymode.c:19-33). Classify the
/// `(left_mv, above_mv)` neighbour pair into one of five sub-MV-ref
/// context categories.
#[inline]
pub fn vp8_mv_cont(l: &Mv, a: &Mv) -> i32 {
    let l_as_int = mv_as_int(l);
    let a_as_int = mv_as_int(a);

    let lez = (l_as_int == 0) as i32;
    let aez = (a_as_int == 0) as i32;
    let lea = (l_as_int == a_as_int) as i32;

    if lea != 0 && lez != 0 {
        return SUBMVREF_LEFT_ABOVE_ZED;
    }

    if lea != 0 {
        return SUBMVREF_LEFT_ABOVE_SAME;
    }

    if aez != 0 {
        return SUBMVREF_ABOVE_ZED;
    }

    if lez != 0 {
        return SUBMVREF_LEFT_ZED;
    }

    SUBMVREF_NORMAL
}

/// `vp8_init_mbmode_probs` (entropymode.c:96-100). Reset the three
/// mutable per-frame mode-probability tables on `VP8_COMMON::fc` to
/// their inter-frame defaults.
pub fn vp8_init_mbmode_probs(x: &mut Vp8Common) {
    x.fc.ymode_prob.copy_from_slice(&vp8_ymode_prob);
    x.fc.uv_mode_prob.copy_from_slice(&vp8_uv_mode_prob);
    x.fc.sub_mv_ref_prob.copy_from_slice(&sub_mv_ref_prob);
}

/// `vp8_default_bmode_probs` (entropymode.c:102-104). Copy the 9
/// default inter-frame 4x4-intra-mode probabilities into the caller's
/// buffer.
pub fn vp8_default_bmode_probs(dest: &mut [Prob; VP8_BINTRAMODES - 1]) {
    dest.copy_from_slice(&vp8_bmode_prob);
}
