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
    Prob, TreeIndex, MVP_BITS, MV_LONG_WIDTH, VP8_BMODE_TREE, VP8_KF_BMODE_PROB,
    VP8_KF_UV_MODE_PROB, VP8_KF_YMODE_PROB, VP8_KF_YMODE_TREE, VP8_MODE_CONTEXTS,
    VP8_MV_UPDATE_PROBS, VP8_SMALL_MVTREE, VP8_SUBMVREFS, VP8_UV_MODE_TREE,
    VP8_YMODE_TREE,
};
use crate::types::{
    BModeInfo, BPredictionMode, BoolDecoder, FrameType, Macroblockd, MbModeInfo,
    MbPredictionMode, ModeInfo, Mv, MvReferenceFrame, Vp8Reader, Vp8dComp,
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

/// `vp8_prob_half` — used by `vp8_read_bit`. Mirrors `treecoder.h`.
const VP8_PROB_HALF: i32 = 128;

use crate::dboolhuff::{vp8_decode_value, vp8dx_bool_error, vp8dx_decode_bool};

// ===========================================================================
// Local re-implementations of trivial `treereader.h` wrappers / helpers.
// ===========================================================================

/// `vp8_read` (`treereader.h:24`) — `#define vp8_read vp8dx_decode_bool`.
#[inline]
unsafe fn vp8_read(r: *mut Vp8Reader<'_>, probability: i32) -> i32 {
    vp8dx_decode_bool(r, probability)
}

/// `vp8_read_literal` (`treereader.h:25`) — `#define vp8_read_literal
/// vp8_decode_value`.
#[inline]
unsafe fn vp8_read_literal(r: *mut Vp8Reader<'_>, bits: i32) -> i32 {
    vp8_decode_value(r, bits)
}

/// `vp8_read_bit` (`treereader.h:26`) — `vp8_read(R, vp8_prob_half)`.
#[inline]
unsafe fn vp8_read_bit(r: *mut Vp8Reader<'_>) -> i32 {
    vp8_read(r, VP8_PROB_HALF)
}

/// `vp8_treed_read` (`treereader.h:30`) — walk a tree-coded value.
#[inline]
unsafe fn vp8_treed_read(
    r: *mut Vp8Reader<'_>,
    t: &[TreeIndex],
    p: *const Prob,
) -> i32 {
    let mut i: TreeIndex = 0;
    loop {
        let idx = (i as usize).wrapping_add(vp8_read(r, *p.add((i >> 1) as usize) as i32) as usize);
        i = t[idx];
        if i <= 0 {
            break;
        }
    }
    -i as i32
}

// ===========================================================================
// `int_mv` accessors.
//
// `types.rs` collapses the C `int_mv` union into a single `Mv` struct, so
// the `.as_int` view used pervasively by `decodemv.c` for fast 32-bit
// compares is provided here as bit-cast helpers. `Mv` is `#[repr(C)]
// { i16, i16 }` and is therefore layout-compatible with `u32`.
// ===========================================================================

#[inline]
fn mv_as_int(m: Mv) -> u32 {
    // Bit-cast Mv -> u32. Safe because of #[repr(C)] and matching size.
    unsafe { core::mem::transmute::<Mv, u32>(m) }
}

#[inline]
fn int_as_mv(v: u32) -> Mv {
    unsafe { core::mem::transmute::<u32, Mv>(v) }
}

// ===========================================================================
// findnearmv.h inline helpers reproduced here. They are static inline in C
// and used only by this translation unit; keeping them local avoids a
// premature dependency on a translation of findnearmv.h.
// ===========================================================================

/// `mv_bias` (`findnearmv.h:24`).
#[inline]
unsafe fn mv_bias(
    refmb_ref_frame_sign_bias: i32,
    refframe: MvReferenceFrame,
    mvp: *mut Mv,
    ref_frame_sign_bias: *const i32,
) {
    if refmb_ref_frame_sign_bias != *ref_frame_sign_bias.add(refframe as usize) {
        (*mvp).row = -(*mvp).row;
        (*mvp).col = -(*mvp).col;
    }
}

/// `vp8_clamp_mv2` (`findnearmv.h:34`).
#[inline]
unsafe fn vp8_clamp_mv2(mv: *mut Mv, xd: *const Macroblockd) {
    let left = (*xd).mb_to_left_edge - LEFT_TOP_MARGIN;
    let right = (*xd).mb_to_right_edge + RIGHT_BOTTOM_MARGIN;
    let top = (*xd).mb_to_top_edge - LEFT_TOP_MARGIN;
    let bottom = (*xd).mb_to_bottom_edge + RIGHT_BOTTOM_MARGIN;

    if ((*mv).col as i32) < left {
        (*mv).col = left as i16;
    } else if ((*mv).col as i32) > right {
        (*mv).col = right as i16;
    }

    if ((*mv).row as i32) < top {
        (*mv).row = top as i16;
    } else if ((*mv).row as i32) > bottom {
        (*mv).row = bottom as i16;
    }
}

/// `vp8_check_mv_bounds` (`findnearmv.h:60`).
#[inline]
unsafe fn vp8_check_mv_bounds(
    mv: *const Mv,
    mb_to_left_edge: i32,
    mb_to_right_edge: i32,
    mb_to_top_edge: i32,
    mb_to_bottom_edge: i32,
) -> u32 {
    let mut need_to_clamp: u32 = (((*mv).col as i32) < mb_to_left_edge) as u32;
    need_to_clamp |= (((*mv).col as i32) > mb_to_right_edge) as u32;
    need_to_clamp |= (((*mv).row as i32) < mb_to_top_edge) as u32;
    need_to_clamp |= (((*mv).row as i32) > mb_to_bottom_edge) as u32;
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
unsafe fn above_block_mode(cur_mb: *const ModeInfo, b: i32, mi_stride: i32) -> BPredictionMode {
    if (b >> 2) == 0 {
        // On top edge, get from MB above us
        let cur_mb = cur_mb.offset(-(mi_stride as isize));
        match (*cur_mb).mbmi.mode {
            MbPredictionMode::BPred => {
                let bmi = (*cur_mb).bmi[(b + 12) as usize];
                if let BModeInfo::Intra(m) = bmi {
                    return m;
                }
                return BPredictionMode::DcPred;
            }
            MbPredictionMode::DcPred => return BPredictionMode::DcPred,
            MbPredictionMode::VPred => return BPredictionMode::VePred,
            MbPredictionMode::HPred => return BPredictionMode::HePred,
            MbPredictionMode::TmPred => return BPredictionMode::TmPred,
            _ => return BPredictionMode::DcPred,
        }
    }

    let bmi = (*cur_mb).bmi[(b - 4) as usize];
    if let BModeInfo::Intra(m) = bmi {
        m
    } else {
        BPredictionMode::DcPred
    }
}

/// `left_block_mode` (`findnearmv.h:110`).
#[inline]
unsafe fn left_block_mode(cur_mb: *const ModeInfo, b: i32) -> BPredictionMode {
    if (b & 3) == 0 {
        // On L edge, get from MB to left of us
        let cur_mb = cur_mb.offset(-1);
        match (*cur_mb).mbmi.mode {
            MbPredictionMode::BPred => {
                let bmi = (*cur_mb).bmi[(b + 3) as usize];
                if let BModeInfo::Intra(m) = bmi {
                    return m;
                }
                return BPredictionMode::DcPred;
            }
            MbPredictionMode::DcPred => return BPredictionMode::DcPred,
            MbPredictionMode::VPred => return BPredictionMode::VePred,
            MbPredictionMode::HPred => return BPredictionMode::HePred,
            MbPredictionMode::TmPred => return BPredictionMode::TmPred,
            _ => return BPredictionMode::DcPred,
        }
    }

    let bmi = (*cur_mb).bmi[(b - 1) as usize];
    if let BModeInfo::Intra(m) = bmi {
        m
    } else {
        BPredictionMode::DcPred
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
unsafe fn read_bmode(bc: *mut Vp8Reader<'_>, p: *const Prob) -> BPredictionMode {
    let i = vp8_treed_read(bc, &VP8_BMODE_TREE, p);
    // Maps to BPredictionMode::DcPred..HuPred (0..9).
    core::mem::transmute::<u8, BPredictionMode>(i as u8)
}

/// `read_ymode` (decodemv.c:24).
unsafe fn read_ymode(bc: *mut Vp8Reader<'_>, p: *const Prob) -> MbPredictionMode {
    let i = vp8_treed_read(bc, &VP8_YMODE_TREE, p);
    core::mem::transmute::<u8, MbPredictionMode>(i as u8)
}

/// `read_kf_ymode` (decodemv.c:30).
unsafe fn read_kf_ymode(bc: *mut Vp8Reader<'_>, p: *const Prob) -> MbPredictionMode {
    let i = vp8_treed_read(bc, &VP8_KF_YMODE_TREE, p);
    core::mem::transmute::<u8, MbPredictionMode>(i as u8)
}

/// `read_uv_mode` (decodemv.c:36).
unsafe fn read_uv_mode(bc: *mut Vp8Reader<'_>, p: *const Prob) -> MbPredictionMode {
    let i = vp8_treed_read(bc, &VP8_UV_MODE_TREE, p);
    core::mem::transmute::<u8, MbPredictionMode>(i as u8)
}

// ===========================================================================
// `read_kf_modes` (decodemv.c:42).
// ===========================================================================

unsafe fn read_kf_modes(pbi: *mut Vp8dComp<'_>, mi: *mut ModeInfo) {
    let bc: *mut Vp8Reader = &mut (*pbi).mbc[8] as *mut _;
    let mis = (*pbi).common.mode_info_stride;

    (*mi).mbmi.ref_frame = MvReferenceFrame::Intra;
    (*mi).mbmi.mode = read_kf_ymode(bc, VP8_KF_YMODE_PROB.as_ptr());

    if (*mi).mbmi.mode == MbPredictionMode::BPred {
        let mut i: i32 = 0;
        (*mi).mbmi.is_4x4 = true;

        loop {
            let a = above_block_mode(mi as *const ModeInfo, i, mis);
            let l = left_block_mode(mi as *const ModeInfo, i);

            let m = read_bmode(
                bc,
                VP8_KF_BMODE_PROB[a as usize][l as usize].as_ptr(),
            );
            (*mi).bmi[i as usize] = BModeInfo::Intra(m);

            i += 1;
            if i >= 16 {
                break;
            }
        }
    }

    (*mi).mbmi.uv_mode = read_uv_mode(bc, VP8_KF_UV_MODE_PROB.as_ptr());
}

// ===========================================================================
// `read_mvcomponent` (decodemv.c:64).
// ===========================================================================

unsafe fn read_mvcomponent(r: *mut Vp8Reader<'_>, mvc: *const Prob) -> i32 {
    // The C code casts MV_CONTEXT* to vp8_prob* — `mvc` here is the
    // resulting flat probability array.
    let p: *const Prob = mvc;
    let mut x: i32 = 0;

    if vp8_read(r, *p.add(MVPIS_SHORT) as i32) != 0 {
        /* Large */
        let mut i: i32 = 0;

        loop {
            x += vp8_read(r, *p.add(MVP_BITS + i as usize) as i32) << i;
            i += 1;
            if i >= 3 {
                break;
            }
        }

        i = MVLONG_WIDTH as i32 - 1; /* Skip bit 3, which is sometimes implicit */

        loop {
            x += vp8_read(r, *p.add(MVP_BITS + i as usize) as i32) << i;
            i -= 1;
            if i <= 3 {
                break;
            }
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

unsafe fn read_mv(r: *mut Vp8Reader<'_>, mv: *mut Mv, mvc: *const Prob) {
    // `mvc` points at the row-component prob vector; `mvc + MVPcount`
    // is the col-component vector (the equivalent of `++mvc` over an
    // array of `MV_CONTEXT`).
    (*mv).row = (read_mvcomponent(r, mvc) * 2) as i16;
    (*mv).col = (read_mvcomponent(r, mvc.add(MVP_COUNT)) * 2) as i16;
}

// ===========================================================================
// `read_mvcontexts` (decodemv.c:96).
// ===========================================================================

unsafe fn read_mvcontexts(bc: *mut Vp8Reader<'_>, mvc: *mut Prob) {
    // `mvc` is the flat probability array of the two MV_CONTEXT records
    // (length 2 * MVP_COUNT).
    let mut i: i32 = 0;

    loop {
        let mut up: *const Prob = VP8_MV_UPDATE_PROBS[i as usize].prob.as_ptr();
        let mut p: *mut Prob = mvc.add((i as usize) * MVP_COUNT);
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
        i += 1;
        if i >= 2 {
            break;
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
    let bc: *mut Vp8Reader = &mut (*pbi).mbc[8] as *mut _;
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
            let mut i: i32 = 0;

            loop {
                (*pbi).common.fc.ymode_prob[i as usize] =
                    vp8_read_literal(bc, 8) as Prob;
                i += 1;
                if i >= 4 {
                    break;
                }
            }
        }

        if vp8_read_bit(bc) != 0 {
            let mut i: i32 = 0;

            loop {
                (*pbi).common.fc.uv_mode_prob[i as usize] =
                    vp8_read_literal(bc, 8) as Prob;
                i += 1;
                if i >= 3 {
                    break;
                }
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

unsafe fn get_sub_mv_ref_prob(left: u32, above: u32) -> *const Prob {
    let lez = (left == 0) as usize;
    let aez = (above == 0) as usize;
    let lea = (left == above) as usize;

    VP8_SUB_MV_REF_PROB3[(aez << 2) | (lez << 1) | lea].as_ptr()
}

// ===========================================================================
// `decode_split_mv` (decodemv.c:188).
// ===========================================================================

unsafe fn decode_split_mv(
    bc: *mut Vp8Reader<'_>,
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
    let mut j: i32 = 0;

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

    loop /* for each subset j */
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
                    let mvc_row: *const Prob =
                        (*mvc.add(0)).prob.as_ptr();
                    let mvc_col: *const Prob =
                        (*mvc.add(1)).prob.as_ptr();
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
                &blockmv_as_mv as *const Mv,
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

        j += 1;
        if j >= num_p {
            break;
        }
    }

    (*mbmi).partitioning = s as u8;
}

// ===========================================================================
// `read_mb_modes_mv` (decodemv.c:284).
// ===========================================================================

unsafe fn read_mb_modes_mv(
    pbi: *mut Vp8dComp<'_>,
    mi: *mut ModeInfo,
    mbmi: *mut MbModeInfo,
) {
    let bc: *mut Vp8Reader = &mut (*pbi).mbc[8] as *mut _;

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
        let ref_frame_sign_bias: *const i32 = (*pbi).common.ref_frame_sign_bias.as_ptr();

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
                    *ref_frame_sign_bias.add((*above).mbmi.ref_frame as usize),
                    (*mbmi).ref_frame,
                    &mut near_mvs[nmv_idx] as *mut Mv,
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
                    *ref_frame_sign_bias.add((*left).mbmi.ref_frame as usize),
                    (*mbmi).ref_frame,
                    &mut this_mv as *mut Mv,
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
                    *ref_frame_sign_bias.add((*aboveleft).mbmi.ref_frame as usize),
                    (*mbmi).ref_frame,
                    &mut this_mv as *mut Mv,
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
                & ((mv_as_int(near_mvs[nmv_idx]) == mv_as_int(near_mvs[CNT_NEAREST]))
                    as i32);

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
                    let mvc: *mut crate::tables::MvContext =
                        (*pbi).common.fc.mvc.as_mut_ptr();
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
                    near_index = CNT_INTRA
                        + ((cnt[CNT_NEAREST] >= cnt[CNT_INTRA]) as usize);

                    vp8_clamp_mv2(&mut near_mvs[near_index] as *mut Mv, &(*pbi).mb);

                    cnt[CNT_SPLITMV] =
                        (((*above).mbmi.mode == MbPredictionMode::SplitMv) as i32
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
                            mbmi_mv,
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
                    vp8_clamp_mv2(&mut (*mbmi).mv as *mut Mv, &(*pbi).mb);
                }
            } else {
                (*mbmi).mode = MbPredictionMode::NearestMv;
                (*mbmi).mv = near_mvs[CNT_NEAREST];
                vp8_clamp_mv2(&mut (*mbmi).mv as *mut Mv, &(*pbi).mb);
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
            let mut j: i32 = 0;
            (*mbmi).is_4x4 = true;
            loop {
                let m = read_bmode(bc, (*pbi).common.fc.bmode_prob.as_ptr());
                (*mi).bmi[j as usize] = BModeInfo::Intra(m);
                j += 1;
                if j >= 16 {
                    break;
                }
            }
        }

        (*mbmi).uv_mode = read_uv_mode(bc, (*pbi).common.fc.uv_mode_prob.as_ptr());
    }
}

// ===========================================================================
// `read_mb_features` (decodemv.c:475).
// ===========================================================================

unsafe fn read_mb_features(
    r: *mut Vp8Reader<'_>,
    mi: *mut MbModeInfo,
    x: *mut Macroblockd,
) {
    /* Is segmentation enabled */
    if (*x).segmentation_enabled != 0 && (*x).update_mb_segmentation_map != 0 {
        /* If so then read the segment id. */
        if vp8_read(r, (*x).mb_segment_tree_probs[0] as i32) != 0 {
            (*mi).segment_id =
                (2 + vp8_read(r, (*x).mb_segment_tree_probs[2] as i32)) as u8;
        } else {
            (*mi).segment_id =
                vp8_read(r, (*x).mb_segment_tree_probs[1] as i32) as u8;
        }
    }
}

// ===========================================================================
// `decode_mb_mode_mvs` (decodemv.c:489).
// ===========================================================================

unsafe fn decode_mb_mode_mvs(pbi: *mut Vp8dComp<'_>, mi: *mut ModeInfo) {
    /* Read the Macroblock segmentation map if it is being updated explicitly
     * this frame (reset to 0 above by default)
     * By default on a key frame reset all MBs to segment 0
     */
    if (*pbi).mb.update_mb_segmentation_map != 0 {
        read_mb_features(
            &mut (*pbi).mbc[8] as *mut _,
            &mut (*mi).mbmi as *mut MbModeInfo,
            &mut (*pbi).mb as *mut Macroblockd,
        );
    } else if (*pbi).common.frame_type == FrameType::Key {
        (*mi).mbmi.segment_id = 0;
    }

    /* Read the macroblock coeff skip flag if this feature is in use,
     * else default to 0 */
    if (*pbi).common.mb_no_coeff_skip != 0 {
        (*mi).mbmi.mb_skip_coeff =
            vp8_read(&mut (*pbi).mbc[8] as *mut _, (*pbi).prob_skip_false as i32) != 0;
    } else {
        (*mi).mbmi.mb_skip_coeff = false;
    }

    (*mi).mbmi.is_4x4 = false;
    if (*pbi).common.frame_type == FrameType::Key {
        read_kf_modes(pbi, mi);
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
    let mut mi: *mut ModeInfo = (*pbi).common.mi;
    let mut mb_row: i32 = -1;
    let mb_to_right_edge_start: i32;

    mb_mode_mv_init(pbi);

    (*pbi).mb.mb_to_top_edge = 0;
    (*pbi).mb.mb_to_bottom_edge = (((*pbi).common.mb_rows - 1) * 16) << 3;
    mb_to_right_edge_start = (((*pbi).common.mb_cols - 1) * 16) << 3;

    loop {
        mb_row += 1;
        if mb_row >= (*pbi).common.mb_rows {
            break;
        }
        let mut mb_col: i32 = -1;

        (*pbi).mb.mb_to_left_edge = 0;
        (*pbi).mb.mb_to_right_edge = mb_to_right_edge_start;

        loop {
            mb_col += 1;
            if mb_col >= (*pbi).common.mb_cols {
                break;
            }

            decode_mb_mode_mvs(pbi, mi);

            // (CONFIG_ERROR_CONCEALMENT branch omitted — minimal build.)

            (*pbi).mb.mb_to_left_edge -= 16 << 3;
            (*pbi).mb.mb_to_right_edge -= 16 << 3;
            mi = mi.add(1); /* next macroblock */
        }
        (*pbi).mb.mb_to_top_edge -= 16 << 3;
        (*pbi).mb.mb_to_bottom_edge -= 16 << 3;

        mi = mi.add(1); /* skip left predictor each row */
    }
}
