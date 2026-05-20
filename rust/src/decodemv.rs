//! `vp8/decoder/decodemv.c` — parse modes, segment IDs and motion vectors.
//!
//! Literal Rust transliteration of `decodemv.c`. The single externally
//! visible entry point is [`vp8_decode_mode_mvs`]; everything else is a
//! private helper used by it. See `documentation/vp8_files/decodemv.md`
//! for the algorithmic rationale.
//!
//! Pointer / unsafe conventions match `dboolhuff.rs`:
//!   - the bool decoder is addressed through a `*mut BoolDecoder`;
//!   - the `MODE_INFO` grid is walked with raw pointer arithmetic to
//!     mirror the C `mi - mis`, `mi - 1`, `mi - mis - 1` neighbour
//!     lookups (column −1 / row −1 sentinels are set up elsewhere);
//!   - helper functions translated from `findnearmv.h` /
//!     `entropymv.h` / `treereader.h` are imported from their host
//!     modules at the top.

#![allow(dead_code)]
#![allow(non_upper_case_globals)]
#![allow(clippy::missing_safety_doc)]
#![allow(clippy::too_many_arguments)]

use crate::tables::{
    MV_LONG_WIDTH, MVP_BITS, Prob, TreeIndex, VP8_BMODE_TREE, VP8_KF_BMODE_PROB,
    VP8_KF_UV_MODE_PROB, VP8_KF_YMODE_PROB, VP8_KF_YMODE_TREE, VP8_MODE_CONTEXTS,
    VP8_MV_UPDATE_PROBS, VP8_SMALL_MVTREE, VP8_SUBMVREFS, VP8_UV_MODE_TREE, VP8_YMODE_TREE,
};
use crate::types::{
    BModeInfo, BPredictionMode, FrameType, MAX_REF_FRAMES, Macroblockd, MbModeInfo, MbPredictionMode,
    ModeInfo, Mv, MvReferenceFrame, Vp8Common, Vp8Reader, Vp8dComp,
};

// ===========================================================================
// Local constants mirroring `vp8/common/entropymv.h` indexing scheme.
// ===========================================================================

/// `mvpis_short` — index of the "long vs short" prob in `MV_CONTEXT.prob`.
const MVPIS_SHORT: usize = 0;
/// `MVPsign` — index of the sign prob in `MV_CONTEXT.prob`.
const MVP_SIGN: usize = 1;
/// `MVPshort` — base index of the short-tree probs in `MV_CONTEXT.prob`.
const MVP_SHORT: usize = 2;
/// `mvlong_width` — number of long-magnitude bit probabilities.
const MVLONG_WIDTH: usize = MV_LONG_WIDTH;
/// `MVPcount` — total per-component MV probabilities (== 19).
const MVP_COUNT: usize = MVP_BITS + MV_LONG_WIDTH;

/// `LEFT_TOP_MARGIN` (findnearmv.h:32).
const LEFT_TOP_MARGIN: i32 = 16 << 3;
/// `RIGHT_BOTTOM_MARGIN` (findnearmv.h:33).
const RIGHT_BOTTOM_MARGIN: i32 = 16 << 3;

// ===========================================================================
// `treereader.h` wrappers — re-exported from `treereader.rs`.
// ===========================================================================

use crate::treereader::{
    vp8_read, vp8_read_bit, vp8_read_literal, vp8_treed_read as vp8_treed_read_raw,
};

/// Slice-friendly wrapper around [`vp8_treed_read`] — most local call
/// sites pass a static array reference.
#[inline]
unsafe fn vp8_treed_read(r: &mut Vp8Reader<'_>, t: &[TreeIndex], p: *const Prob) -> i32 {
    vp8_treed_read_raw(r, t.as_ptr(), p)
}

// ===========================================================================
// `int_mv` accessors.
//
// `types.rs` collapses the C `int_mv` union into a single `Mv` struct, so
// the `.as_int` view used pervasively by `decodemv.c` for fast 32-bit
// compares is provided here as bit-cast helpers. `Mv` is `#[repr(C)]
// { i16, i16 }` and is therefore layout-compatible with `u32`.
// ===========================================================================

use crate::types::{mv_as_int, mv_from_int};

#[inline]
fn int_as_mv(v: u32) -> Mv {
    mv_from_int(v)
}

// ===========================================================================
// findnearmv.h inline helpers reproduced here. They are static inline in C
// and used only by this translation unit; keeping them local avoids a
// premature dependency on a translation of findnearmv.h.
// ===========================================================================

/// `mv_bias` (`findnearmv.h:24`).
#[inline]
fn mv_bias(
    refmb_ref_frame_sign_bias: i32,
    refframe: MvReferenceFrame,
    mvp: &mut Mv,
    ref_frame_sign_bias: &[i32; MAX_REF_FRAMES],
) {
    if refmb_ref_frame_sign_bias != ref_frame_sign_bias[refframe as usize] {
        mvp.row = -mvp.row;
        mvp.col = -mvp.col;
    }
}

/// `vp8_clamp_mv2` (`findnearmv.h:34`).
#[inline]
fn vp8_clamp_mv2(
    mv: &mut Mv,
    mb_to_left_edge: i32,
    mb_to_right_edge: i32,
    mb_to_top_edge: i32,
    mb_to_bottom_edge: i32,
) {
    let left = mb_to_left_edge - LEFT_TOP_MARGIN;
    let right = mb_to_right_edge + RIGHT_BOTTOM_MARGIN;
    let top = mb_to_top_edge - LEFT_TOP_MARGIN;
    let bottom = mb_to_bottom_edge + RIGHT_BOTTOM_MARGIN;

    mv.col = (mv.col as i32).clamp(left, right) as i16;
    mv.row = (mv.row as i32).clamp(top, bottom) as i16;
}

/// `vp8_check_mv_bounds` (`findnearmv.h:60`).
#[inline]
fn vp8_check_mv_bounds(
    mv: &Mv,
    mb_to_left_edge: i32,
    mb_to_right_edge: i32,
    mb_to_top_edge: i32,
    mb_to_bottom_edge: i32,
) -> u32 {
    let mut need_to_clamp: u32 = ((mv.col as i32) < mb_to_left_edge) as u32;
    need_to_clamp |= ((mv.col as i32) > mb_to_right_edge) as u32;
    need_to_clamp |= ((mv.row as i32) < mb_to_top_edge) as u32;
    need_to_clamp |= ((mv.row as i32) > mb_to_bottom_edge) as u32;
    need_to_clamp
}

/// `vp8_mbsplit_offset` (findnearmv.c:13) — first 4x4 block index of
/// each subset, per split shape. Local copy because `findnearmv.c` is
/// not yet translated.
const VP8_MBSPLIT_OFFSET: [[u8; 16]; 4] = [
    [0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 2, 8, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
];

/// `above_block_mode` (`findnearmv.h:128`).
#[inline]
fn above_block_mode(pc: &Vp8Common, mb_row: i32, mb_col: i32, mi: &ModeInfo, b: i32) -> BPredictionMode {
    if (b >> 2) == 0 {
        // On top edge, get from MB above us
        let above = pc.mi_above(mb_row, mb_col);
        return match above.mbmi.mode {
            MbPredictionMode::BPred => match above.bmi[(b + 12) as usize] {
                BModeInfo::Intra(m) => m,
                _ => BPredictionMode::DcPred,
            },
            MbPredictionMode::VPred => BPredictionMode::VePred,
            MbPredictionMode::HPred => BPredictionMode::HePred,
            MbPredictionMode::TmPred => BPredictionMode::TmPred,
            _ => BPredictionMode::DcPred,
        };
    }

    match mi.bmi[(b - 4) as usize] {
        BModeInfo::Intra(m) => m,
        _ => BPredictionMode::DcPred,
    }
}

/// `left_block_mode` (`findnearmv.h:110`).
#[inline]
fn left_block_mode(pc: &Vp8Common, mb_row: i32, mb_col: i32, mi: &ModeInfo, b: i32) -> BPredictionMode {
    if (b & 3) == 0 {
        // On L edge, get from MB to left of us
        let left = pc.mi_left(mb_row, mb_col);
        return match left.mbmi.mode {
            MbPredictionMode::BPred => match left.bmi[(b + 3) as usize] {
                BModeInfo::Intra(m) => m,
                _ => BPredictionMode::DcPred,
            },
            MbPredictionMode::VPred => BPredictionMode::VePred,
            MbPredictionMode::HPred => BPredictionMode::HePred,
            MbPredictionMode::TmPred => BPredictionMode::TmPred,
            _ => BPredictionMode::DcPred,
        };
    }

    match mi.bmi[(b - 1) as usize] {
        BModeInfo::Intra(m) => m,
        _ => BPredictionMode::DcPred,
    }
}

// ===========================================================================
// `BModeInfo` accessor helpers — `union b_mode_info`'s two views.
// ===========================================================================

#[inline]
fn bmi_mv_as_int(bmi: BModeInfo) -> u32 {
    match bmi {
        BModeInfo::Mv(m) => mv_as_int(m),
        // The C union allows either view to be read; under SPLITMV the
        // entry holds an MV. The Intra path is unreachable here but we
        // return 0 to mirror "raw bits of the union".
        BModeInfo::Intra(_) => 0,
    }
}

// ===========================================================================
// Tree-decoding wrappers (decodemv.c:18..40).
// ===========================================================================

/// `read_bmode` (decodemv.c:18).
unsafe fn read_bmode(bc: &mut Vp8Reader<'_>, p: *const Prob) -> BPredictionMode {
    let i = vp8_treed_read(bc, &VP8_BMODE_TREE, p);
    // Maps to BPredictionMode::DcPred..HuPred (0..9).
    core::mem::transmute::<u8, BPredictionMode>(i as u8)
}

/// `read_ymode` (decodemv.c:24).
unsafe fn read_ymode(bc: &mut Vp8Reader<'_>, p: *const Prob) -> MbPredictionMode {
    let i = vp8_treed_read(bc, &VP8_YMODE_TREE, p);
    core::mem::transmute::<u8, MbPredictionMode>(i as u8)
}

/// `read_kf_ymode` (decodemv.c:30).
unsafe fn read_kf_ymode(bc: &mut Vp8Reader<'_>, p: *const Prob) -> MbPredictionMode {
    let i = vp8_treed_read(bc, &VP8_KF_YMODE_TREE, p);
    core::mem::transmute::<u8, MbPredictionMode>(i as u8)
}

/// `read_uv_mode` (decodemv.c:36).
unsafe fn read_uv_mode(bc: &mut Vp8Reader<'_>, p: *const Prob) -> MbPredictionMode {
    let i = vp8_treed_read(bc, &VP8_UV_MODE_TREE, p);
    core::mem::transmute::<u8, MbPredictionMode>(i as u8)
}

// ===========================================================================
// `read_kf_modes` (decodemv.c:42).
// ===========================================================================

unsafe fn read_kf_modes(pbi: *mut Vp8dComp<'_>, mi: *mut ModeInfo, mb_row: i32, mb_col: i32) {
    let bc = &mut (*pbi).mbc[8];

    (*mi).mbmi.ref_frame = MvReferenceFrame::Intra;
    (*mi).mbmi.mode = read_kf_ymode(bc, VP8_KF_YMODE_PROB.as_ptr());

    if (*mi).mbmi.mode == MbPredictionMode::BPred {
        (*mi).mbmi.is_4x4 = true;

        for i in 0..16i32 {
            let a = above_block_mode(&(*pbi).common, mb_row, mb_col, &*mi, i);
            let l = left_block_mode(&(*pbi).common, mb_row, mb_col, &*mi, i);

            let m = read_bmode(bc, VP8_KF_BMODE_PROB[a as usize][l as usize].as_ptr());
            (*mi).bmi[i as usize] = BModeInfo::Intra(m);
        }
    }

    (*mi).mbmi.uv_mode = read_uv_mode(bc, VP8_KF_UV_MODE_PROB.as_ptr());
}

// ===========================================================================
// `read_mvcomponent` (decodemv.c:64).
// ===========================================================================

unsafe fn read_mvcomponent(r: &mut Vp8Reader<'_>, mvc: *const Prob) -> i32 {
    // The C code casts MV_CONTEXT* to vp8_prob* — `mvc` here is the
    // resulting flat probability array.
    let p: *const Prob = mvc;
    let mut x: i32 = 0;

    if vp8_read(r, *p.add(MVPIS_SHORT) as i32) != 0 {
        /* Large */
        for i in 0..3i32 {
            x += vp8_read(r, *p.add(MVP_BITS + i as usize) as i32) << i;
        }

        /* Skip bit 3, which is sometimes implicit */
        for i in (4..MVLONG_WIDTH as i32).rev() {
            x += vp8_read(r, *p.add(MVP_BITS + i as usize) as i32) << i;
        }

        if (x & 0xFFF0) == 0 || vp8_read(r, *p.add(MVP_BITS + 3) as i32) != 0 {
            x += 8;
        }
    } else {
        /* small */
        x = vp8_treed_read(r, &VP8_SMALL_MVTREE, p.add(MVP_SHORT));
    }

    if x != 0 && vp8_read(r, *p.add(MVP_SIGN) as i32) != 0 {
        x = -x;
    }

    x
}

// ===========================================================================
// `read_mv` (decodemv.c:91).
// ===========================================================================

unsafe fn read_mv(r: &mut Vp8Reader<'_>, mv: *mut Mv, mvc: *const Prob) {
    // `mvc` points at the row-component prob vector; `mvc + MVPcount`
    // is the col-component vector (the equivalent of `++mvc` over an
    // array of `MV_CONTEXT`).
    (*mv).row = (read_mvcomponent(r, mvc) * 2) as i16;
    (*mv).col = (read_mvcomponent(r, mvc.add(MVP_COUNT)) * 2) as i16;
}

// ===========================================================================
// `read_mvcontexts` (decodemv.c:96).
// ===========================================================================

unsafe fn read_mvcontexts(bc: &mut Vp8Reader<'_>, mvc: *mut Prob) {
    // `mvc` is the flat probability array of the two MV_CONTEXT records
    // (length 2 * MVP_COUNT).
    for i in 0..2usize {
        let mut up: *const Prob = VP8_MV_UPDATE_PROBS[i].prob.as_ptr();
        let mut p: *mut Prob = mvc.add(i * MVP_COUNT);
        let pstop: *mut Prob = p.add(MVP_COUNT);

        loop {
            if vp8_read(bc, *up as i32) != 0 {
                up = up.add(1);
                let x: Prob = vp8_read_literal(bc, 7) as Prob;
                *p = if x != 0 { x << 1 } else { 1 };
            } else {
                up = up.add(1);
            }
            p = p.add(1);
            if p >= pstop {
                break;
            }
        }
    }
}

// ===========================================================================
// SPLITMV helper tables (decodemv.c:114..120).
// ===========================================================================

static MBSPLIT_FILL_COUNT: [u8; 4] = [8, 8, 4, 1];
static MBSPLIT_FILL_OFFSET: [[u8; 16]; 4] = [
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    [0, 1, 4, 5, 8, 9, 12, 13, 2, 3, 6, 7, 10, 11, 14, 15],
    [0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15],
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
];

// ===========================================================================
// `mb_mode_mv_init` (decodemv.c:122).
// ===========================================================================

unsafe fn mb_mode_mv_init(pbi: *mut Vp8dComp<'_>) {
    let bc = &mut (*pbi).mbc[8];
    let mvc: *mut Prob = (*pbi).common.fc.mvc.as_mut_ptr() as *mut Prob;

    // (CONFIG_ERROR_CONCEALMENT branch omitted — minimal build.)

    /* Read the mb_no_coeff_skip flag */
    (*pbi).common.mb_no_coeff_skip = vp8_read_bit(bc);

    (*pbi).prob_skip_false = 0;
    if (*pbi).common.mb_no_coeff_skip != 0 {
        (*pbi).prob_skip_false = vp8_read_literal(bc, 8) as Prob;
    }

    if (*pbi).common.frame_type != FrameType::Key {
        (*pbi).prob_intra = vp8_read_literal(bc, 8) as Prob;
        (*pbi).prob_last = vp8_read_literal(bc, 8) as Prob;
        (*pbi).prob_gf = vp8_read_literal(bc, 8) as Prob;

        if vp8_read_bit(bc) != 0 {
            for i in 0..4usize {
                (*pbi).common.fc.ymode_prob[i] = vp8_read_literal(bc, 8) as Prob;
            }
        }

        if vp8_read_bit(bc) != 0 {
            for i in 0..3usize {
                (*pbi).common.fc.uv_mode_prob[i] = vp8_read_literal(bc, 8) as Prob;
            }
        }

        read_mvcontexts(bc, mvc);
    }
}

// ===========================================================================
// `vp8_sub_mv_ref_prob3` (decodemv.c:165) — file-local table.
// ===========================================================================

/// `vp8_sub_mv_ref_prob3` — 8-entry deduplicated sub-MV-ref probability
/// table indexed by `(aez<<2)|(lez<<1)|lea`. RFC 6386 §16.4.
pub const VP8_SUB_MV_REF_PROB3: [[Prob; VP8_SUBMVREFS - 1]; 8] = [
    [147, 136, 18], /* SUBMVREF_NORMAL          */
    [223, 1, 34],   /* SUBMVREF_LEFT_ABOVE_SAME */
    [106, 145, 1],  /* SUBMVREF_LEFT_ZED        */
    [208, 1, 1],    /* SUBMVREF_LEFT_ABOVE_ZED  */
    [179, 121, 1],  /* SUBMVREF_ABOVE_ZED       */
    [223, 1, 34],   /* SUBMVREF_LEFT_ABOVE_SAME */
    [179, 121, 1],  /* SUBMVREF_ABOVE_ZED       */
    [208, 1, 1],    /* SUBMVREF_LEFT_ABOVE_ZED  */
];

// ===========================================================================
// `get_sub_mv_ref_prob` (decodemv.c:176).
// ===========================================================================

fn get_sub_mv_ref_prob(left: u32, above: u32) -> *const Prob {
    let lez = (left == 0) as usize;
    let aez = (above == 0) as usize;
    let lea = (left == above) as usize;

    VP8_SUB_MV_REF_PROB3[(aez << 2) | (lez << 1) | lea].as_ptr()
}

// ===========================================================================
// `decode_split_mv` (decodemv.c:188).
// ===========================================================================

unsafe fn decode_split_mv(
    bc: &mut Vp8Reader<'_>,
    mi: *mut ModeInfo,
    left_mb: *const ModeInfo,
    above_mb: *const ModeInfo,
    mbmi: *mut MbModeInfo,
    best_mv: Mv,
    mvc: *mut crate::tables::MvContext,
    mb_to_left_edge: i32,
    mb_to_right_edge: i32,
    mb_to_top_edge: i32,
    mb_to_bottom_edge: i32,
) {
    let mut s: i32; /* split configuration (16x8, 8x16, 8x8, 4x4) */
    /* number of partitions in the split configuration */
    let mut num_p: i32;

    s = 3;
    num_p = 16;
    if vp8_read(bc, 110) != 0 {
        s = 2;
        num_p = 4;
        if vp8_read(bc, 111) != 0 {
            s = vp8_read(bc, 150);
            num_p = 2;
        }
    }

    for j in 0..num_p
    /* for each subset j */
    {
        let leftmv: u32;
        let abovemv: u32;
        let mut blockmv: u32;
        let k: i32; /* first block in subset j */

        let prob: *const Prob;
        k = VP8_MBSPLIT_OFFSET[s as usize][j as usize] as i32;

        if (k & 3) == 0 {
            /* On L edge, get from MB to left of us */
            if (*left_mb).mbmi.mode != MbPredictionMode::SplitMv {
                leftmv = mv_as_int((*left_mb).mbmi.mv);
            } else {
                leftmv = bmi_mv_as_int((*left_mb).bmi[(k + 4 - 1) as usize]);
            }
        } else {
            leftmv = bmi_mv_as_int((*mi).bmi[(k - 1) as usize]);
        }

        if (k >> 2) == 0 {
            /* On top edge, get from MB above us */
            if (*above_mb).mbmi.mode != MbPredictionMode::SplitMv {
                abovemv = mv_as_int((*above_mb).mbmi.mv);
            } else {
                abovemv = bmi_mv_as_int((*above_mb).bmi[(k + 16 - 4) as usize]);
            }
        } else {
            abovemv = bmi_mv_as_int((*mi).bmi[(k - 4) as usize]);
        }

        prob = get_sub_mv_ref_prob(leftmv, abovemv);

        if vp8_read(bc, *prob.add(0) as i32) != 0 {
            if vp8_read(bc, *prob.add(1) as i32) != 0 {
                blockmv = 0;
                if vp8_read(bc, *prob.add(2) as i32) != 0 {
                    let mvc_row: *const Prob = (*mvc.add(0)).prob.as_ptr();
                    let mvc_col: *const Prob = (*mvc.add(1)).prob.as_ptr();
                    let mut tmp = Mv { row: 0, col: 0 };
                    tmp.row = (read_mvcomponent(bc, mvc_row) * 2) as i16;
                    tmp.row = tmp.row.wrapping_add(best_mv.row);
                    tmp.col = (read_mvcomponent(bc, mvc_col) * 2) as i16;
                    tmp.col = tmp.col.wrapping_add(best_mv.col);
                    blockmv = mv_as_int(tmp);
                }
            } else {
                blockmv = abovemv;
            }
        } else {
            blockmv = leftmv;
        }

        let blockmv_as_mv = int_as_mv(blockmv);
        (*mbmi).need_to_clamp_mvs = (*mbmi).need_to_clamp_mvs
            || vp8_check_mv_bounds(
                &blockmv_as_mv,
                mb_to_left_edge,
                mb_to_right_edge,
                mb_to_top_edge,
                mb_to_bottom_edge,
            ) != 0;

        {
            /* Fill (uniform) modes, mvs of jth subset.
            Must do it here because ensuing subsets can
            refer back to us via "left" or "above". */
            let mut fill_offset: *const u8;
            let mut fill_count: u32 = MBSPLIT_FILL_COUNT[s as usize] as u32;

            fill_offset = MBSPLIT_FILL_OFFSET[s as usize]
                .as_ptr()
                .add((j as u8 as usize) * (MBSPLIT_FILL_COUNT[s as usize] as usize));

            loop {
                let idx = *fill_offset as usize;
                (*mi).bmi[idx] = BModeInfo::Mv(int_as_mv(blockmv));
                fill_offset = fill_offset.add(1);
                fill_count -= 1;
                if fill_count == 0 {
                    break;
                }
            }
        }
    }

    (*mbmi).partitioning = s as u8;
}

// ===========================================================================
// `read_mb_modes_mv` (decodemv.c:284).
// ===========================================================================

unsafe fn read_mb_modes_mv(pbi: *mut Vp8dComp<'_>, mi: *mut ModeInfo, mbmi: *mut MbModeInfo) {
    let bc = &mut (*pbi).mbc[8];

    // ref_frame = (MV_REFERENCE_FRAME)vp8_read(bc, pbi->prob_intra);
    let rf = vp8_read(bc, (*pbi).prob_intra as i32);
    (*mbmi).ref_frame = match rf {
        0 => MvReferenceFrame::Intra,
        1 => MvReferenceFrame::Last,
        2 => MvReferenceFrame::Golden,
        _ => MvReferenceFrame::Altref,
    };

    if (*mbmi).ref_frame != MvReferenceFrame::Intra {
        /* inter MB */
        const CNT_INTRA: usize = 0;
        const CNT_NEAREST: usize = 1;
        const CNT_NEAR: usize = 2;
        const CNT_SPLITMV: usize = 3;

        let mut cnt: [i32; 4] = [0; 4];
        let mut cntx_idx: usize = 0;
        let mut near_mvs: [Mv; 4] = [Mv { row: 0, col: 0 }; 4];
        let mut nmv_idx: usize = 0;
        let mis = (*pbi).mb.mode_info_stride;
        let above: *const ModeInfo = mi.offset(-(mis as isize));
        let left: *const ModeInfo = mi.offset(-1);
        let aboveleft: *const ModeInfo = above.offset(-1);
        let ref_frame_sign_bias: &[i32; MAX_REF_FRAMES] = &(*pbi).common.ref_frame_sign_bias;

        (*mbmi).need_to_clamp_mvs = false;

        if vp8_read(bc, (*pbi).prob_last as i32) != 0 {
            let v = 2 + vp8_read(bc, (*pbi).prob_gf as i32);
            (*mbmi).ref_frame = match v {
                2 => MvReferenceFrame::Golden,
                _ => MvReferenceFrame::Altref,
            };
        }

        /* Zero accumulators */
        // (already zero from initialisation above; mirror the C zeroing.)
        near_mvs[0] = Mv { row: 0, col: 0 };
        near_mvs[1] = Mv { row: 0, col: 0 };
        near_mvs[2] = Mv { row: 0, col: 0 };
        cnt[0] = 0;
        cnt[1] = 0;
        cnt[2] = 0;
        cnt[3] = 0;

        /* Process above */
        if (*above).mbmi.ref_frame != MvReferenceFrame::Intra {
            if mv_as_int((*above).mbmi.mv) != 0 {
                nmv_idx += 1;
                near_mvs[nmv_idx] = (*above).mbmi.mv;
                mv_bias(
                    ref_frame_sign_bias[(*above).mbmi.ref_frame as usize],
                    (*mbmi).ref_frame,
                    &mut near_mvs[nmv_idx],
                    ref_frame_sign_bias,
                );
                cntx_idx += 1;
            }
            cnt[cntx_idx] += 2;
        }

        /* Process left */
        if (*left).mbmi.ref_frame != MvReferenceFrame::Intra {
            if mv_as_int((*left).mbmi.mv) != 0 {
                let mut this_mv: Mv = (*left).mbmi.mv;
                mv_bias(
                    ref_frame_sign_bias[(*left).mbmi.ref_frame as usize],
                    (*mbmi).ref_frame,
                    &mut this_mv,
                    ref_frame_sign_bias,
                );

                if mv_as_int(this_mv) != mv_as_int(near_mvs[nmv_idx]) {
                    nmv_idx += 1;
                    near_mvs[nmv_idx] = this_mv;
                    cntx_idx += 1;
                }
                cnt[cntx_idx] += 2;
            } else {
                cnt[CNT_INTRA] += 2;
            }
        }

        /* Process above left */
        if (*aboveleft).mbmi.ref_frame != MvReferenceFrame::Intra {
            if mv_as_int((*aboveleft).mbmi.mv) != 0 {
                let mut this_mv: Mv = (*aboveleft).mbmi.mv;
                mv_bias(
                    ref_frame_sign_bias[(*aboveleft).mbmi.ref_frame as usize],
                    (*mbmi).ref_frame,
                    &mut this_mv,
                    ref_frame_sign_bias,
                );

                if mv_as_int(this_mv) != mv_as_int(near_mvs[nmv_idx]) {
                    nmv_idx += 1;
                    near_mvs[nmv_idx] = this_mv;
                    cntx_idx += 1;
                }
                cnt[cntx_idx] += 1;
            } else {
                cnt[CNT_INTRA] += 1;
            }
        }

        if vp8_read(bc, VP8_MODE_CONTEXTS[cnt[CNT_INTRA] as usize][0]) != 0 {
            /* If we have three distinct MV's ... */
            /* See if above-left MV can be merged with NEAREST */
            cnt[CNT_NEAREST] += ((cnt[CNT_SPLITMV] > 0) as i32)
                & ((mv_as_int(near_mvs[nmv_idx]) == mv_as_int(near_mvs[CNT_NEAREST])) as i32);

            /* Swap near and nearest if necessary */
            if cnt[CNT_NEAR] > cnt[CNT_NEAREST] {
                let tmp_c = cnt[CNT_NEAREST];
                cnt[CNT_NEAREST] = cnt[CNT_NEAR];
                cnt[CNT_NEAR] = tmp_c;
                let tmp_mv_int = mv_as_int(near_mvs[CNT_NEAREST]);
                near_mvs[CNT_NEAREST] = near_mvs[CNT_NEAR];
                near_mvs[CNT_NEAR] = int_as_mv(tmp_mv_int);
            }

            if vp8_read(bc, VP8_MODE_CONTEXTS[cnt[CNT_NEAREST] as usize][1]) != 0 {
                if vp8_read(bc, VP8_MODE_CONTEXTS[cnt[CNT_NEAR] as usize][2]) != 0 {
                    let mut mb_to_top_edge: i32;
                    let mut mb_to_bottom_edge: i32;
                    let mut mb_to_left_edge: i32;
                    let mut mb_to_right_edge: i32;
                    let mvc: *mut crate::tables::MvContext = (*pbi).common.fc.mvc.as_mut_ptr();
                    let near_index: usize;

                    mb_to_top_edge = (*pbi).mb.mb_to_top_edge;
                    mb_to_bottom_edge = (*pbi).mb.mb_to_bottom_edge;
                    mb_to_top_edge -= LEFT_TOP_MARGIN;
                    mb_to_bottom_edge += RIGHT_BOTTOM_MARGIN;
                    mb_to_right_edge = (*pbi).mb.mb_to_right_edge;
                    mb_to_right_edge += RIGHT_BOTTOM_MARGIN;
                    mb_to_left_edge = (*pbi).mb.mb_to_left_edge;
                    mb_to_left_edge -= LEFT_TOP_MARGIN;

                    /* Use near_mvs[0] to store the "best" MV */
                    near_index = CNT_INTRA + ((cnt[CNT_NEAREST] >= cnt[CNT_INTRA]) as usize);

                    vp8_clamp_mv2(
                        &mut near_mvs[near_index],
                        (*pbi).mb.mb_to_left_edge,
                        (*pbi).mb.mb_to_right_edge,
                        (*pbi).mb.mb_to_top_edge,
                        (*pbi).mb.mb_to_bottom_edge,
                    );

                    cnt[CNT_SPLITMV] = (((*above).mbmi.mode == MbPredictionMode::SplitMv) as i32
                        + ((*left).mbmi.mode == MbPredictionMode::SplitMv) as i32)
                        * 2
                        + ((*aboveleft).mbmi.mode == MbPredictionMode::SplitMv) as i32;

                    if vp8_read(bc, VP8_MODE_CONTEXTS[cnt[CNT_SPLITMV] as usize][3]) != 0 {
                        decode_split_mv(
                            bc,
                            mi,
                            left,
                            above,
                            mbmi,
                            near_mvs[near_index],
                            mvc,
                            mb_to_left_edge,
                            mb_to_right_edge,
                            mb_to_top_edge,
                            mb_to_bottom_edge,
                        );
                        (*mbmi).mv = int_as_mv(bmi_mv_as_int((*mi).bmi[15]));
                        (*mbmi).mode = MbPredictionMode::SplitMv;
                        (*mbmi).is_4x4 = true;
                    } else {
                        let mbmi_mv: *mut Mv = &mut (*mbmi).mv as *mut Mv;
                        read_mv(bc, mbmi_mv, mvc as *const Prob);
                        (*mbmi_mv).row = (*mbmi_mv).row.wrapping_add(near_mvs[near_index].row);
                        (*mbmi_mv).col = (*mbmi_mv).col.wrapping_add(near_mvs[near_index].col);

                        /* Don't need to check this on NEARMV and NEARESTMV
                         * modes since those modes clamp the MV. The NEWMV mode
                         * does not, so signal to the prediction stage whether
                         * special handling may be required.
                         */
                        (*mbmi).need_to_clamp_mvs = vp8_check_mv_bounds(
                            &*mbmi_mv,
                            mb_to_left_edge,
                            mb_to_right_edge,
                            mb_to_top_edge,
                            mb_to_bottom_edge,
                        ) != 0;
                        (*mbmi).mode = MbPredictionMode::NewMv;
                    }
                } else {
                    (*mbmi).mode = MbPredictionMode::NearMv;
                    (*mbmi).mv = near_mvs[CNT_NEAR];
                    vp8_clamp_mv2(
                        &mut (*mbmi).mv,
                        (*pbi).mb.mb_to_left_edge,
                        (*pbi).mb.mb_to_right_edge,
                        (*pbi).mb.mb_to_top_edge,
                        (*pbi).mb.mb_to_bottom_edge,
                    );
                }
            } else {
                (*mbmi).mode = MbPredictionMode::NearestMv;
                (*mbmi).mv = near_mvs[CNT_NEAREST];
                vp8_clamp_mv2(
                    &mut (*mbmi).mv,
                    (*pbi).mb.mb_to_left_edge,
                    (*pbi).mb.mb_to_right_edge,
                    (*pbi).mb.mb_to_top_edge,
                    (*pbi).mb.mb_to_bottom_edge,
                );
            }
        } else {
            (*mbmi).mode = MbPredictionMode::ZeroMv;
            (*mbmi).mv = Mv { row: 0, col: 0 };
        }

        // (CONFIG_ERROR_CONCEALMENT branch omitted — minimal build.)
    } else {
        /* required for left and above block mv */
        (*mbmi).mv = Mv { row: 0, col: 0 };

        /* MB is intra coded */
        let ym = read_ymode(bc, (*pbi).common.fc.ymode_prob.as_ptr());
        (*mbmi).mode = ym;
        if ym == MbPredictionMode::BPred {
            (*mbmi).is_4x4 = true;
            for j in 0..16usize {
                let m = read_bmode(bc, (*pbi).common.fc.bmode_prob.as_ptr());
                (*mi).bmi[j] = BModeInfo::Intra(m);
            }
        }

        (*mbmi).uv_mode = read_uv_mode(bc, (*pbi).common.fc.uv_mode_prob.as_ptr());
    }
}

// ===========================================================================
// `read_mb_features` (decodemv.c:475).
// ===========================================================================

fn read_mb_features(r: &mut Vp8Reader<'_>, mi: &mut MbModeInfo, x: &Macroblockd) {
    /* Is segmentation enabled */
    if x.segmentation_enabled != 0 && x.update_mb_segmentation_map != 0 {
        /* If so then read the segment id. */
        if vp8_read(r, x.mb_segment_tree_probs[0] as i32) != 0 {
            mi.segment_id = (2 + vp8_read(r, x.mb_segment_tree_probs[2] as i32)) as u8;
        } else {
            mi.segment_id = vp8_read(r, x.mb_segment_tree_probs[1] as i32) as u8;
        }
    }
}

// ===========================================================================
// `decode_mb_mode_mvs` (decodemv.c:489).
// ===========================================================================

unsafe fn decode_mb_mode_mvs(pbi: *mut Vp8dComp<'_>, mi: *mut ModeInfo, mb_row: i32, mb_col: i32) {
    /* Read the Macroblock segmentation map if it is being updated explicitly
     * this frame (reset to 0 above by default)
     * By default on a key frame reset all MBs to segment 0
     */
    if (*pbi).mb.update_mb_segmentation_map != 0 {
        read_mb_features(
            &mut (*pbi).mbc[8],
            &mut (*mi).mbmi,
            &(*pbi).mb,
        );
    } else if (*pbi).common.frame_type == FrameType::Key {
        (*mi).mbmi.segment_id = 0;
    }

    /* Read the macroblock coeff skip flag if this feature is in use,
     * else default to 0 */
    if (*pbi).common.mb_no_coeff_skip != 0 {
        (*mi).mbmi.mb_skip_coeff =
            vp8_read(&mut (*pbi).mbc[8], (*pbi).prob_skip_false as i32) != 0;
    } else {
        (*mi).mbmi.mb_skip_coeff = false;
    }

    (*mi).mbmi.is_4x4 = false;
    if (*pbi).common.frame_type == FrameType::Key {
        read_kf_modes(pbi, mi, mb_row, mb_col);
    } else {
        read_mb_modes_mv(pbi, mi, &mut (*mi).mbmi as *mut MbModeInfo);
    }
}

// ===========================================================================
// `vp8_decode_mode_mvs` (decodemv.c:516) — sole external entry point.
// ===========================================================================

/// `vp8_decode_mode_mvs` — parse all per-MB modes / MVs / segment IDs
/// for the current frame, writing into `pbi->common.mi[]`.
///
/// Source: `vp8/decoder/decodemv.c:516`.
pub unsafe fn vp8_decode_mode_mvs(pbi: *mut Vp8dComp<'_>) {
    mb_mode_mv_init(pbi);

    (*pbi).mb.mb_to_top_edge = 0;
    (*pbi).mb.mb_to_bottom_edge = (((*pbi).common.mb_rows - 1) * 16) << 3;
    let mb_to_right_edge_start: i32 = (((*pbi).common.mb_cols - 1) * 16) << 3;

    for mb_row in 0..(*pbi).common.mb_rows {
        (*pbi).mb.mb_to_left_edge = 0;
        (*pbi).mb.mb_to_right_edge = mb_to_right_edge_start;

        for mb_col in 0..(*pbi).common.mb_cols {
            // Address the current MB cell through the safe accessor;
            // demote to raw `*mut ModeInfo` for the still-raw-ptr-shaped
            // sub-call chain.
            let mi: *mut ModeInfo = (*pbi).common.mi_mut(mb_row, mb_col);
            decode_mb_mode_mvs(pbi, mi, mb_row, mb_col);

            // (CONFIG_ERROR_CONCEALMENT branch omitted — minimal build.)

            (*pbi).mb.mb_to_left_edge -= 16 << 3;
            (*pbi).mb.mb_to_right_edge -= 16 << 3;
        }
        (*pbi).mb.mb_to_top_edge -= 16 << 3;
        (*pbi).mb.mb_to_bottom_edge -= 16 << 3;
    }
}
