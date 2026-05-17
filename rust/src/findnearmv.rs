//! `vp8/common/findnearmv.c` — spatial MV predictor for inter macroblocks.
//!
//! Literal translation of the canonical near-MV predictor shared between
//! encoder and decoder. Given the three spatial neighbours (above, left,
//! above-left) of the current macroblock, [`vp8_find_near_mvs`] collects
//! the candidate MVs, scores them with the fixed RFC 6386 §16 weights
//! {2, 2, 1}, ranks the top two into the "nearest" and "near" slots, and
//! returns the four-element count vector that the mode entropy decoder
//! consumes via [`vp8_mv_ref_probs`]. See
//! `documentation/vp8_files/findnearmv.md` for the full derivation.

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use crate::tables::{Prob, VP8_MODE_CONTEXTS};
use crate::types::{Macroblockd, MbPredictionMode, ModeInfo, Mv, MvReferenceFrame};

// ---------------------------------------------------------------------------
// `int_mv` helpers
// ---------------------------------------------------------------------------
//
// The C source uses `int_mv`, a union of `as_int` (uint32_t) and
// `as_mv` (MV { int16_t row, col }). The Rust translation in
// `types.rs` collapses this to a single `Mv` struct; the `as_int`
// view is reconstructed here with a pair of bit-cast helpers so the
// equality tests and copies in the original C body transliterate
// directly.

#[inline]
fn mv_as_int(m: Mv) -> u32 {
    // little-endian layout matching libvpx's union on supported targets:
    // bits 0..16 = row, bits 16..32 = col.
    (m.row as u16 as u32) | ((m.col as u16 as u32) << 16)
}

#[inline]
fn mv_from_int(v: u32) -> Mv {
    Mv {
        row: (v & 0xFFFF) as i16,
        col: ((v >> 16) & 0xFFFF) as i16,
    }
}

// ---------------------------------------------------------------------------
// Static data
// ---------------------------------------------------------------------------

/// `vp8_mbsplit_offset` — vp8/common/findnearmv.c:13.
///
/// For each of the four `SPLITMV` partitionings (MB_2_HORIZ, MB_2_VERT,
/// MB_4_QUART, MB_16_4x4), the 4x4-block raster indices where each
/// sub-partition begins. Trailing entries beyond the row's meaningful
/// count are zero padding.
#[rustfmt::skip]
pub static vp8_mbsplit_offset: [[u8; 16]; 4] = [
    [0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 2, 8, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
];

// ---------------------------------------------------------------------------
// `mv_bias` / `vp8_clamp_mv2` — from findnearmv.h
// ---------------------------------------------------------------------------

/// `mv_bias` — vp8/common/findnearmv.h:24.
///
/// Negates `mvp` in place if the neighbour reference's sign-bias disagrees
/// with the current MB's reference. After the call both vectors live in
/// the same temporal frame and equality tests are meaningful.
#[inline]
unsafe fn mv_bias(
    refmb_ref_frame_sign_bias: i32,
    refframe: i32,
    mvp: *mut Mv,
    ref_frame_sign_bias: *const i32,
) {
    if refmb_ref_frame_sign_bias != *ref_frame_sign_bias.offset(refframe as isize) {
        (*mvp).row = ((*mvp).row as i32 * -1) as i16;
        (*mvp).col = ((*mvp).col as i32 * -1) as i16;
    }
}

const LEFT_TOP_MARGIN: i32 = 16 << 3;
const RIGHT_BOTTOM_MARGIN: i32 = 16 << 3;

/// `vp8_clamp_mv2` — vp8/common/findnearmv.h:34.
///
/// Clamps `mv` so motion-comp never reads farther than half an MB outside
/// the current macroblock's allowable window.
#[inline]
unsafe fn vp8_clamp_mv2(mv: *mut Mv, xd: *const Macroblockd) {
    if ((*mv).col as i32) < ((*xd).mb_to_left_edge - LEFT_TOP_MARGIN) {
        (*mv).col = ((*xd).mb_to_left_edge - LEFT_TOP_MARGIN) as i16;
    } else if ((*mv).col as i32) > (*xd).mb_to_right_edge + RIGHT_BOTTOM_MARGIN {
        (*mv).col = ((*xd).mb_to_right_edge + RIGHT_BOTTOM_MARGIN) as i16;
    }

    if ((*mv).row as i32) < ((*xd).mb_to_top_edge - LEFT_TOP_MARGIN) {
        (*mv).row = ((*xd).mb_to_top_edge - LEFT_TOP_MARGIN) as i16;
    } else if ((*mv).row as i32) > (*xd).mb_to_bottom_edge + RIGHT_BOTTOM_MARGIN {
        (*mv).row = ((*xd).mb_to_bottom_edge + RIGHT_BOTTOM_MARGIN) as i16;
    }
}

// ---------------------------------------------------------------------------
// `vp8_find_near_mvs`
// ---------------------------------------------------------------------------

// CNT_INTRA / CNT_NEAREST / CNT_NEAR / CNT_SPLITMV — local enum in C.
const CNT_INTRA: usize = 0;
const CNT_NEAREST: usize = 1;
const CNT_NEAR: usize = 2;
const CNT_SPLITMV: usize = 3;

/// `vp8_find_near_mvs` — vp8/common/findnearmv.c:23.
///
/// Predict motion vectors using those from already-decoded nearby blocks.
/// Note that we only consider one 4x4 sub-block from each candidate 16x16
/// macroblock.
pub unsafe fn vp8_find_near_mvs(
    xd: *mut Macroblockd,
    here: *const ModeInfo,
    nearest: *mut Mv,
    nearby: *mut Mv,
    best_mv: *mut Mv,
    near_mv_ref_cnts: *mut i32,
    refframe: i32,
    ref_frame_sign_bias: *mut i32,
) {
    let above: *const ModeInfo = here.offset(-((*xd).mode_info_stride as isize));
    let left: *const ModeInfo = here.offset(-1);
    let aboveleft: *const ModeInfo = above.offset(-1);
    let mut near_mvs: [Mv; 4] = [Mv { row: 0, col: 0 }; 4];
    // `mv` walks `near_mvs`; `cntx` walks `near_mv_ref_cnts`.
    let mut mv: *mut Mv = near_mvs.as_mut_ptr();
    let mut cntx: *mut i32 = near_mv_ref_cnts;

    /* Zero accumulators */
    // near_mvs[0..3] zeroed (entry 3 left untouched, matching the C).
    near_mvs[0] = Mv { row: 0, col: 0 };
    near_mvs[1] = Mv { row: 0, col: 0 };
    near_mvs[2] = Mv { row: 0, col: 0 };
    *near_mv_ref_cnts.offset(0) = 0;
    *near_mv_ref_cnts.offset(1) = 0;
    *near_mv_ref_cnts.offset(2) = 0;
    *near_mv_ref_cnts.offset(3) = 0;

    /* Process above */
    if (*above).mbmi.ref_frame != MvReferenceFrame::Intra {
        if mv_as_int((*above).mbmi.mv) != 0 {
            mv = mv.offset(1);
            *mv = (*above).mbmi.mv;
            mv_bias(
                *ref_frame_sign_bias.offset((*above).mbmi.ref_frame as isize),
                refframe,
                mv,
                ref_frame_sign_bias,
            );
            cntx = cntx.offset(1);
        }

        *cntx += 2;
    }

    /* Process left */
    if (*left).mbmi.ref_frame != MvReferenceFrame::Intra {
        if mv_as_int((*left).mbmi.mv) != 0 {
            let mut this_mv: Mv = (*left).mbmi.mv;

            mv_bias(
                *ref_frame_sign_bias.offset((*left).mbmi.ref_frame as isize),
                refframe,
                &mut this_mv,
                ref_frame_sign_bias,
            );

            if mv_as_int(this_mv) != mv_as_int(*mv) {
                mv = mv.offset(1);
                *mv = this_mv;
                cntx = cntx.offset(1);
            }

            *cntx += 2;
        } else {
            *near_mv_ref_cnts.offset(CNT_INTRA as isize) += 2;
        }
    }

    /* Process above left */
    if (*aboveleft).mbmi.ref_frame != MvReferenceFrame::Intra {
        if mv_as_int((*aboveleft).mbmi.mv) != 0 {
            let mut this_mv: Mv = (*aboveleft).mbmi.mv;

            mv_bias(
                *ref_frame_sign_bias.offset((*aboveleft).mbmi.ref_frame as isize),
                refframe,
                &mut this_mv,
                ref_frame_sign_bias,
            );

            if mv_as_int(this_mv) != mv_as_int(*mv) {
                mv = mv.offset(1);
                *mv = this_mv;
                cntx = cntx.offset(1);
            }

            *cntx += 1;
        } else {
            *near_mv_ref_cnts.offset(CNT_INTRA as isize) += 1;
        }
    }

    /* If we have three distinct MV's ... */
    if *near_mv_ref_cnts.offset(CNT_SPLITMV as isize) != 0 {
        /* See if above-left MV can be merged with NEAREST */
        if mv_as_int(*mv) == mv_as_int(near_mvs[CNT_NEAREST]) {
            *near_mv_ref_cnts.offset(CNT_NEAREST as isize) += 1;
        }
    }

    *near_mv_ref_cnts.offset(CNT_SPLITMV as isize) =
        (((*above).mbmi.mode == MbPredictionMode::SplitMv) as i32
            + ((*left).mbmi.mode == MbPredictionMode::SplitMv) as i32)
            * 2
            + ((*aboveleft).mbmi.mode == MbPredictionMode::SplitMv) as i32;

    /* Swap near and nearest if necessary */
    if *near_mv_ref_cnts.offset(CNT_NEAR as isize)
        > *near_mv_ref_cnts.offset(CNT_NEAREST as isize)
    {
        let mut tmp: i32;
        tmp = *near_mv_ref_cnts.offset(CNT_NEAREST as isize);
        *near_mv_ref_cnts.offset(CNT_NEAREST as isize) =
            *near_mv_ref_cnts.offset(CNT_NEAR as isize);
        *near_mv_ref_cnts.offset(CNT_NEAR as isize) = tmp;
        tmp = mv_as_int(near_mvs[CNT_NEAREST]) as i32;
        near_mvs[CNT_NEAREST] = near_mvs[CNT_NEAR];
        near_mvs[CNT_NEAR] = mv_from_int(tmp as u32);
    }

    /* Use near_mvs[0] to store the "best" MV */
    if *near_mv_ref_cnts.offset(CNT_NEAREST as isize)
        >= *near_mv_ref_cnts.offset(CNT_INTRA as isize)
    {
        near_mvs[CNT_INTRA] = near_mvs[CNT_NEAREST];
    }

    /* Set up return values */
    *best_mv = near_mvs[0];
    *nearest = near_mvs[CNT_NEAREST];
    *nearby = near_mvs[CNT_NEAR];
}

// ---------------------------------------------------------------------------
// `invert_and_clamp_mvs`
// ---------------------------------------------------------------------------

/// `invert_and_clamp_mvs` — vp8/common/findnearmv.c:124 (static helper).
///
/// Componentwise negate `src` into `inv` and clamp both to the current
/// MB's allowable MV window.
unsafe fn invert_and_clamp_mvs(inv: *mut Mv, src: *mut Mv, xd: *mut Macroblockd) {
    (*inv).row = ((*src).row as i32 * -1) as i16;
    (*inv).col = ((*src).col as i32 * -1) as i16;
    vp8_clamp_mv2(inv, xd);
    vp8_clamp_mv2(src, xd);
}

// ---------------------------------------------------------------------------
// `vp8_find_near_mvs_bias`
// ---------------------------------------------------------------------------

// Mirrors of the C `MB_MODE_COUNT` constant and the `NEARESTMV` / `NEARMV`
// enumerator indices used as row selectors into `mode_mv_sb`.
const MB_MODE_COUNT: usize = crate::types::MB_MODE_COUNT;
const NEARESTMV: usize = MbPredictionMode::NearestMv as usize;
const NEARMV: usize = MbPredictionMode::NearMv as usize;

/// `vp8_find_near_mvs_bias` — vp8/common/findnearmv.c:131.
///
/// Wrapper used by the encoder's RD loop: calls [`vp8_find_near_mvs`] for
/// the requested `refframe`, deposits the three result MVs into the
/// `mode_mv_sb[sign_bias]` column, then fills the opposite sign-bias
/// column with the negated/clamped versions. Returns the sign-bias used.
///
/// `mode_mv_sb` points at `int_mv mode_mv_sb[2][MB_MODE_COUNT]`,
/// laid out row-major; `best_mv_sb` points at `int_mv best_mv_sb[2]`.
pub unsafe fn vp8_find_near_mvs_bias(
    xd: *mut Macroblockd,
    here: *const ModeInfo,
    mode_mv_sb: *mut [Mv; MB_MODE_COUNT],
    best_mv_sb: *mut Mv,
    cnt: *mut i32,
    refframe: i32,
    ref_frame_sign_bias: *mut i32,
) -> i32 {
    let sign_bias: i32 = *ref_frame_sign_bias.offset(refframe as isize);

    vp8_find_near_mvs(
        xd,
        here,
        (*mode_mv_sb.offset(sign_bias as isize))
            .as_mut_ptr()
            .add(NEARESTMV),
        (*mode_mv_sb.offset(sign_bias as isize))
            .as_mut_ptr()
            .add(NEARMV),
        best_mv_sb.offset(sign_bias as isize),
        cnt,
        refframe,
        ref_frame_sign_bias,
    );

    invert_and_clamp_mvs(
        (*mode_mv_sb.offset((!sign_bias & 1) as isize))
            .as_mut_ptr()
            .add(NEARESTMV),
        (*mode_mv_sb.offset(sign_bias as isize))
            .as_mut_ptr()
            .add(NEARESTMV),
        xd,
    );
    invert_and_clamp_mvs(
        (*mode_mv_sb.offset((!sign_bias & 1) as isize))
            .as_mut_ptr()
            .add(NEARMV),
        (*mode_mv_sb.offset(sign_bias as isize))
            .as_mut_ptr()
            .add(NEARMV),
        xd,
    );
    invert_and_clamp_mvs(
        best_mv_sb.offset((!sign_bias & 1) as isize),
        best_mv_sb.offset(sign_bias as isize),
        xd,
    );

    sign_bias
}

// ---------------------------------------------------------------------------
// `vp8_mv_ref_probs`
// ---------------------------------------------------------------------------

/// `vp8_mv_ref_probs` — vp8/common/findnearmv.c:150.
///
/// Map the four-slot context vector produced by [`vp8_find_near_mvs`] into
/// the four-byte probability vector for the MV-reference tree. Returns
/// `p` for call-chaining (mirrors the C return).
pub unsafe fn vp8_mv_ref_probs(p: *mut Prob, near_mv_ref_ct: *const i32) -> *mut Prob {
    *p.offset(0) = VP8_MODE_CONTEXTS[*near_mv_ref_ct.offset(0) as usize][0] as Prob;
    *p.offset(1) = VP8_MODE_CONTEXTS[*near_mv_ref_ct.offset(1) as usize][1] as Prob;
    *p.offset(2) = VP8_MODE_CONTEXTS[*near_mv_ref_ct.offset(2) as usize][2] as Prob;
    *p.offset(3) = VP8_MODE_CONTEXTS[*near_mv_ref_ct.offset(3) as usize][3] as Prob;
    /* p[3] = vp8_mode_contexts[near_mv_ref_ct[1] + near_mv_ref_ct[2] +
                             near_mv_ref_ct[3]][3]; */
    p
}
