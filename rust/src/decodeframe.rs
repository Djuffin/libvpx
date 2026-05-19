//! Literal Rust transliteration of `vp8/decoder/decodeframe.c`.
//!
//! This is the frame-level driver of the VP8 decoder. It parses the
//! uncompressed frame tag, opens the residual partition, parses the
//! compressed header, lays out token partitions, then walks the
//! macroblock grid in raster order calling into the per-MB predictors,
//! token decoder and IDCT adders defined in sibling files.
//!
//! All functions here mirror the C control-flow as closely as possible;
//! pointer-heavy code uses raw `*mut T` / `*const T` and `unsafe`.

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]

use core::ffi::{c_int, c_uint};
use core::ptr;

use crate::tables::{
    BLOCK_TYPES, COEF_BANDS, ENTROPY_NODES, MAXQ, PREV_COEF_CONTEXTS, Prob, VP8_COEF_UPDATE_PROBS,
    VP8_DEFAULT_MV_CONTEXT, VP8_MB_FEATURE_DATA_BITS,
};
use crate::types::{
    Blockd, ClampType, EntropyContextPlanes, FrameType, LoopFilterType, MAX_MB_SEGMENTS,
    MAX_MODE_LF_DELTAS, MAX_REF_FRAMES, MAX_REF_LF_DELTAS, MB_FEATURE_TREE_PROBS, MB_LVL_MAX,
    Macroblockd, MbLevelFeature, MbPredictionMode, ModeInfo, MvReferenceFrame,
    TokenPartition, Vp8Common, Vp8Reader, Vp8dComp, VpxResult, Yv12BufferConfig,
};

// ---------------------------------------------------------------------------
// Local enums/constants shadowing the C ones used in this file.
// ---------------------------------------------------------------------------

/// C `SEGMENT_ABSDATA` / `SEGMENT_DELTADATA` (`blockd.h`).
const SEGMENT_ABSDATA: u8 = 1;
const SEGMENT_DELTADATA: u8 = 0;

/// `MB_LVL_ALT_Q` (`blockd.h`).
const MB_LVL_ALT_Q: usize = MbLevelFeature::AltQ as usize;
/// `MB_LVL_ALT_LF` (`blockd.h`).
const MB_LVL_ALT_LF: usize = MbLevelFeature::AltLf as usize;

/// C `INTRA_FRAME` / `LAST_FRAME` / `GOLDEN_FRAME` / `ALTREF_FRAME`
/// (`blockd.h`).
const INTRA_FRAME: usize = MvReferenceFrame::Intra as usize;
const LAST_FRAME: usize = MvReferenceFrame::Last as usize;
const GOLDEN_FRAME: usize = MvReferenceFrame::Golden as usize;
const ALTREF_FRAME: usize = MvReferenceFrame::Altref as usize;

/// C `KEY_FRAME` / `INTER_FRAME` (`blockd.h`).
const KEY_FRAME: FrameType = FrameType::Key;
const INTER_FRAME: FrameType = FrameType::Inter;

/// C `B_PRED`, `DC_PRED`, `SPLITMV` from `MB_PREDICTION_MODE`.
const B_PRED: MbPredictionMode = MbPredictionMode::BPred;
const DC_PRED: MbPredictionMode = MbPredictionMode::DcPred;
const SPLITMV: MbPredictionMode = MbPredictionMode::SplitMv;

/// C `NORMAL_LOOPFILTER` from `LOOPFILTERTYPE`.
const NORMAL_LOOPFILTER: LoopFilterType = LoopFilterType::Normal;

/// C `ONE_PARTITION` from `TOKEN_PARTITION`.
const ONE_PARTITION: TokenPartition = TokenPartition::One;

use crate::vpx_api::{VPX_CODEC_CORRUPT_FRAME, VPX_CODEC_MEM_ERROR, VPX_CODEC_UNSUP_BITSTREAM};

// ---------------------------------------------------------------------------
// External functions implemented in sibling translation units.
// ---------------------------------------------------------------------------

use crate::alloccommon::vp8_setup_version;
use crate::decodemv::vp8_decode_mode_mvs;
use crate::detokenize::{vp8_decode_mb_tokens, vp8_reset_mb_tokens_context};
use crate::entropy::vp8_default_coef_probs;
use crate::entropymode::vp8_init_mbmode_probs;
use crate::extend::vp8_extend_mb_row;
use crate::quant_common::{
    vp8_ac_uv_quant, vp8_ac_yquant, vp8_ac2quant, vp8_dc_quant, vp8_dc_uv_quant, vp8_dc2quant,
};
use crate::reconinter::vp8_build_inter_predictors_mb;
use crate::reconintra::{vp8_build_intra_predictors_mbuv_s, vp8_build_intra_predictors_mby_s};
use crate::reconintra4x4::{intra_prediction_down_copy, vp8_intra4x4_predict};
use crate::setupintrarecon::{setup_intra_recon_left, vp8_setup_intra_recon_top_line};
use crate::vp8_loopfilter::{
    vp8_loop_filter_frame_init, vp8_loop_filter_row_normal, vp8_loop_filter_row_simple,
};

use crate::dboolhuff::{vp8dx_bool_error, vp8dx_start_decode};
use crate::treereader::{vp8_read, vp8_read_bit, vp8_read_literal};
use crate::vp8_rtcd::{
    vp8_dc_only_idct_add, vp8_dequant_idct_add, vp8_dequant_idct_add_uv_block,
    vp8_dequant_idct_add_y_block, vp8_dequantize_b, vp8_short_inv_walsh4x4,
    vp8_short_inv_walsh4x4_1,
};
use crate::vpx_codec::vpx_internal_error;

// ---------------------------------------------------------------------------
// vp8cx_init_de_quantizer — decodeframe.c:42
// ---------------------------------------------------------------------------

/// `vp8cx_init_de_quantizer` (vp8/decoder/decodeframe.c:42).
pub unsafe fn vp8cx_init_de_quantizer(pbi: *mut Vp8dComp<'static>) {
    let mut Q: c_int;
    let pc: *mut Vp8Common = &mut (*pbi).common;

    Q = 0;
    while Q < crate::tables::QINDEX_RANGE as c_int {
        (*pc).y1_dequant[Q as usize][0] = vp8_dc_quant(Q, (*pc).y1dc_delta_q) as i16;
        (*pc).y2_dequant[Q as usize][0] = vp8_dc2quant(Q, (*pc).y2dc_delta_q) as i16;
        (*pc).uv_dequant[Q as usize][0] = vp8_dc_uv_quant(Q, (*pc).uvdc_delta_q) as i16;

        (*pc).y1_dequant[Q as usize][1] = vp8_ac_yquant(Q) as i16;
        (*pc).y2_dequant[Q as usize][1] = vp8_ac2quant(Q, (*pc).y2ac_delta_q) as i16;
        (*pc).uv_dequant[Q as usize][1] = vp8_ac_uv_quant(Q, (*pc).uvac_delta_q) as i16;

        Q += 1;
    }
}

// ---------------------------------------------------------------------------
// vp8_mb_init_dequantizer — decodeframe.c:57
// ---------------------------------------------------------------------------

/// `vp8_mb_init_dequantizer` (vp8/decoder/decodeframe.c:57).
///
/// §3 split-borrow pilot: this function originally took
/// `(pbi: *mut Vp8dComp, xd: *mut Macroblockd)`. It now takes disjoint
/// references against the two fields that pointed at — `&Vp8Common`
/// (read-only) for the dequant tables and base qindex, and
/// `&mut Macroblockd` for the per-MB dequant arrays. The single
/// remaining `unsafe` deref is `mb.mode_info_context`, which points
/// into `common.mip` and is therefore an alias we can't express as a
/// safe reborrow until the MI grid itself is converted.
pub fn vp8_mb_init_dequantizer(pc: &Vp8Common, mb: &mut Macroblockd) {
    // SAFETY: `mode_info_context` is a kernel-internal cursor into
    // `common.mip`; the deref is sound by the same invariant that the
    // rest of the kernel relies on.
    let segment_id = unsafe { (*mb.mode_info_context).mbmi.segment_id as usize };

    /* Decide whether to use the default or alternate baseline Q value. */
    let qi: usize = (if mb.segmentation_enabled != 0 {
        let q = if mb.mb_segment_abs_delta == SEGMENT_ABSDATA {
            mb.segment_feature_data[MB_LVL_ALT_Q][segment_id] as c_int
        } else {
            pc.base_qindex + mb.segment_feature_data[MB_LVL_ALT_Q][segment_id] as c_int
        };
        q.clamp(0, MAXQ as c_int)
    } else {
        pc.base_qindex
    }) as usize;

    /* Set up the macroblock dequant constants */
    mb.dequant_y1_dc[0] = 1;
    mb.dequant_y1[0] = pc.y1_dequant[qi][0];
    mb.dequant_y2[0] = pc.y2_dequant[qi][0];
    mb.dequant_uv[0] = pc.uv_dequant[qi][0];

    for i in 1..16 {
        let ac = pc.y1_dequant[qi][1];
        mb.dequant_y1_dc[i] = ac;
        mb.dequant_y1[i] = ac;
        mb.dequant_y2[i] = pc.y2_dequant[qi][1];
        mb.dequant_uv[i] = pc.uv_dequant[qi][1];
    }
}

// ---------------------------------------------------------------------------
// decode_macroblock — decodeframe.c:94
// ---------------------------------------------------------------------------

/// `decode_macroblock` (vp8/decoder/decodeframe.c:94). Static helper.
unsafe fn decode_macroblock(
    pbi: *mut Vp8dComp<'static>,
    xd: *mut Macroblockd,
    mb_col: c_int,
    bc: *mut Vp8Reader<'static>,
) {
    let mode: MbPredictionMode;

    if (*(*xd).mode_info_context).mbmi.mb_skip_coeff {
        vp8_reset_mb_tokens_context(pbi, xd, mb_col);
    } else if vp8dx_bool_error(bc) == 0 {
        let eobtotal: c_int = vp8_decode_mb_tokens(pbi, xd, mb_col, bc);

        /* Special case:  Force the loopfilter to skip when eobtotal is zero */
        (*(*xd).mode_info_context).mbmi.mb_skip_coeff = eobtotal == 0;
    }

    mode = (*(*xd).mode_info_context).mbmi.mode;

    if (*xd).segmentation_enabled != 0 {
        vp8_mb_init_dequantizer(&(*pbi).common, &mut *xd);
    }

    /* do prediction */
    if (*(*xd).mode_info_context).mbmi.ref_frame == MvReferenceFrame::Intra {
        vp8_build_intra_predictors_mbuv_s(
            xd,
            (*xd).recon_above[1],
            (*xd).recon_above[2],
            (*xd).recon_left[1],
            (*xd).recon_left[2],
            (*xd).recon_left_stride[1],
            (*xd).dst.u_buffer,
            (*xd).dst.v_buffer,
            (*xd).dst.uv_stride,
        );

        if mode != B_PRED {
            vp8_build_intra_predictors_mby_s(
                xd,
                (*xd).recon_above[0],
                (*xd).recon_left[0],
                (*xd).recon_left_stride[0],
                (*xd).dst.y_buffer,
                (*xd).dst.y_stride,
            );
        } else {
            let DQC: *mut i16 = (*xd).dequant_y1.as_mut_ptr();
            let dst_stride: c_int = (*xd).dst.y_stride;

            /* clear out residual eob info */
            if (*(*xd).mode_info_context).mbmi.mb_skip_coeff {
                ptr::write_bytes((*xd).eobs.as_mut_ptr(), 0, 25);
            }

            intra_prediction_down_copy(xd, (*xd).recon_above[0].add(16));

            for i in 0..16 {
                let b: *mut Blockd = &mut (*xd).block[i as usize];
                let dst: *mut u8 = (*xd).dst.y_buffer.offset((*b).offset as isize);
                // Extract the 4x4 intra mode from the BModeInfo enum at this
                // sub-block slot. In C this is `bmi[i].as_mode` — a plain
                // `B_PREDICTION_MODE`.
                let b_mode_val: crate::types::BPredictionMode =
                    match (*(*xd).mode_info_context).bmi[i as usize] {
                        crate::types::BModeInfo::Intra(m) => m,
                        // SPLITMV path stores an Mv here; in B_PRED context this
                        // branch should be unreachable, but mirror C's behaviour
                        // (which would just read garbage from the union) by
                        // treating it as DC_PRED.
                        crate::types::BModeInfo::Mv(_) => crate::types::BPredictionMode::DcPred,
                    };
                let above: *mut u8 = dst.offset(-(dst_stride as isize));
                let yleft: *mut u8 = dst.offset(-1);
                let left_stride: c_int = dst_stride;
                let top_left: u8 = *above.offset(-1);

                vp8_intra4x4_predict(
                    above,
                    yleft,
                    left_stride,
                    b_mode_val,
                    dst,
                    dst_stride,
                    top_left,
                );

                if (*xd).eobs[i as usize] != 0 {
                    if (*xd).eobs[i as usize] > 1 {
                        vp8_dequant_idct_add((*b).qcoeff, DQC, dst, dst_stride);
                    } else {
                        let q0 = *(*b).qcoeff;
                        let dqc0 = *DQC;
                        vp8_dc_only_idct_add(
                            (q0 as i32 * dqc0 as i32) as i16,
                            dst,
                            dst_stride,
                            dst,
                            dst_stride,
                        );
                        ptr::write_bytes(
                            (*b).qcoeff as *mut u8,
                            0,
                            2 * core::mem::size_of::<i16>(),
                        );
                    }
                }
            }
        }
    } else {
        vp8_build_inter_predictors_mb(xd);
    }

    if !(*(*xd).mode_info_context).mbmi.mb_skip_coeff {
        /* dequantization and idct */
        if mode != B_PRED {
            let mut DQC: *mut i16 = (*xd).dequant_y1.as_mut_ptr();

            if mode != SPLITMV {
                let b: *mut Blockd = &mut (*xd).block[24];

                /* do 2nd order transform on the dc block */
                if (*xd).eobs[24] > 1 {
                    vp8_dequantize_b(b, (*xd).dequant_y2.as_mut_ptr());

                    vp8_short_inv_walsh4x4((*b).dqcoeff, (*xd).qcoeff.as_mut_ptr());
                    ptr::write_bytes((*b).qcoeff as *mut u8, 0, 16 * core::mem::size_of::<i16>());
                } else {
                    let q0 = *(*b).qcoeff;
                    let dq0 = (*xd).dequant_y2[0];
                    *(*b).dqcoeff = (q0 as i32 * dq0 as i32) as i16;
                    vp8_short_inv_walsh4x4_1((*b).dqcoeff, (*xd).qcoeff.as_mut_ptr());
                    ptr::write_bytes((*b).qcoeff as *mut u8, 0, 2 * core::mem::size_of::<i16>());
                }

                /* override the dc dequant constant in order to preserve the
                 * dc components
                 */
                DQC = (*xd).dequant_y1_dc.as_mut_ptr();
            }

            vp8_dequant_idct_add_y_block(
                (*xd).qcoeff.as_mut_ptr(),
                DQC,
                (*xd).dst.y_buffer,
                (*xd).dst.y_stride,
                (*xd).eobs.as_mut_ptr(),
            );
        }

        vp8_dequant_idct_add_uv_block(
            (*xd).qcoeff.as_mut_ptr().add(16 * 16),
            (*xd).dequant_uv.as_mut_ptr(),
            (*xd).dst.u_buffer,
            (*xd).dst.v_buffer,
            (*xd).dst.uv_stride,
            (*xd).eobs.as_mut_ptr().add(16),
        );
    }
}

// ---------------------------------------------------------------------------
// get_delta_q — decodeframe.c:235
// ---------------------------------------------------------------------------

/// `get_delta_q` (vp8/decoder/decodeframe.c:235). Static helper.
unsafe fn get_delta_q(bc: *mut Vp8Reader<'static>, prev: c_int, q_update: *mut c_int) -> c_int {
    let mut ret_val: c_int = 0;

    if vp8_read_bit(bc) != 0 {
        ret_val = vp8_read_literal(bc, 4);

        if vp8_read_bit(bc) != 0 {
            ret_val = -ret_val;
        }
    }

    /* Trigger a quantizer update if the delta-q value has changed */
    if ret_val != prev {
        *q_update = 1;
    }

    ret_val
}

// ---------------------------------------------------------------------------
// yv12_extend_frame_top_c — decodeframe.c:255
// ---------------------------------------------------------------------------

/// `yv12_extend_frame_top_c` (vp8/decoder/decodeframe.c:255). Static helper.
unsafe fn yv12_extend_frame_top_c(ybf: *mut Yv12BufferConfig) {
    let mut i: c_int;
    let mut src_ptr1: *mut u8;
    let mut dest_ptr1: *mut u8;

    let mut Border: c_uint;
    let mut plane_stride: c_int;

    /* Y Plane */
    Border = (*ybf).border as c_uint;
    plane_stride = (*ybf).y_stride;
    src_ptr1 = (*ybf).y_buffer.offset(-(Border as isize));
    dest_ptr1 = src_ptr1.offset(-((Border as isize) * (plane_stride as isize)));

    i = 0;
    while i < Border as c_int {
        ptr::copy_nonoverlapping(src_ptr1, dest_ptr1, plane_stride as usize);
        dest_ptr1 = dest_ptr1.offset(plane_stride as isize);
        i += 1;
    }

    /* U Plane */
    plane_stride = (*ybf).uv_stride;
    Border /= 2;
    src_ptr1 = (*ybf).u_buffer.offset(-(Border as isize));
    dest_ptr1 = src_ptr1.offset(-((Border as isize) * (plane_stride as isize)));

    i = 0;
    while i < Border as c_int {
        ptr::copy_nonoverlapping(src_ptr1, dest_ptr1, plane_stride as usize);
        dest_ptr1 = dest_ptr1.offset(plane_stride as isize);
        i += 1;
    }

    /* V Plane */
    src_ptr1 = (*ybf).v_buffer.offset(-(Border as isize));
    dest_ptr1 = src_ptr1.offset(-((Border as isize) * (plane_stride as isize)));

    i = 0;
    while i < Border as c_int {
        ptr::copy_nonoverlapping(src_ptr1, dest_ptr1, plane_stride as usize);
        dest_ptr1 = dest_ptr1.offset(plane_stride as isize);
        i += 1;
    }
}

// ---------------------------------------------------------------------------
// yv12_extend_frame_bottom_c — decodeframe.c:302
// ---------------------------------------------------------------------------

/// `yv12_extend_frame_bottom_c` (vp8/decoder/decodeframe.c:302). Static helper.
unsafe fn yv12_extend_frame_bottom_c(ybf: *mut Yv12BufferConfig) {
    let mut i: c_int;
    let mut src_ptr1: *mut u8;
    let mut src_ptr2: *mut u8;
    let mut dest_ptr2: *mut u8;

    let mut Border: c_uint;
    let mut plane_stride: c_int;
    let mut plane_height: c_int;

    /* Y Plane */
    Border = (*ybf).border as c_uint;
    plane_stride = (*ybf).y_stride;
    plane_height = (*ybf).y_height;

    src_ptr1 = (*ybf).y_buffer.offset(-(Border as isize));
    src_ptr2 = src_ptr1
        .offset((plane_height as isize) * (plane_stride as isize))
        .offset(-(plane_stride as isize));
    dest_ptr2 = src_ptr2.offset(plane_stride as isize);

    i = 0;
    while i < Border as c_int {
        ptr::copy_nonoverlapping(src_ptr2, dest_ptr2, plane_stride as usize);
        dest_ptr2 = dest_ptr2.offset(plane_stride as isize);
        i += 1;
    }

    /* U Plane */
    plane_stride = (*ybf).uv_stride;
    plane_height = (*ybf).uv_height;
    Border /= 2;

    src_ptr1 = (*ybf).u_buffer.offset(-(Border as isize));
    src_ptr2 = src_ptr1
        .offset((plane_height as isize) * (plane_stride as isize))
        .offset(-(plane_stride as isize));
    dest_ptr2 = src_ptr2.offset(plane_stride as isize);

    i = 0;
    while i < Border as c_int {
        ptr::copy_nonoverlapping(src_ptr2, dest_ptr2, plane_stride as usize);
        dest_ptr2 = dest_ptr2.offset(plane_stride as isize);
        i += 1;
    }

    /* V Plane */
    src_ptr1 = (*ybf).v_buffer.offset(-(Border as isize));
    src_ptr2 = src_ptr1
        .offset((plane_height as isize) * (plane_stride as isize))
        .offset(-(plane_stride as isize));
    dest_ptr2 = src_ptr2.offset(plane_stride as isize);

    i = 0;
    while i < Border as c_int {
        ptr::copy_nonoverlapping(src_ptr2, dest_ptr2, plane_stride as usize);
        dest_ptr2 = dest_ptr2.offset(plane_stride as isize);
        i += 1;
    }
}

// ---------------------------------------------------------------------------
// yv12_extend_frame_left_right_c — decodeframe.c:357
// ---------------------------------------------------------------------------

/// `yv12_extend_frame_left_right_c` (vp8/decoder/decodeframe.c:357). Static helper.
unsafe fn yv12_extend_frame_left_right_c(
    ybf: *mut Yv12BufferConfig,
    y_src: *mut u8,
    u_src: *mut u8,
    v_src: *mut u8,
) {
    let mut i: c_int;
    let mut src_ptr1: *mut u8;
    let mut src_ptr2: *mut u8;
    let mut dest_ptr1: *mut u8;
    let mut dest_ptr2: *mut u8;

    let mut Border: c_uint;
    let mut plane_stride: c_int;
    let mut plane_height: c_int;
    let mut plane_width: c_int;

    /* Y Plane */
    Border = (*ybf).border as c_uint;
    plane_stride = (*ybf).y_stride;
    plane_height = 16;
    plane_width = (*ybf).y_width;

    /* copy the left and right most columns out */
    src_ptr1 = y_src;
    src_ptr2 = src_ptr1.offset((plane_width - 1) as isize);
    dest_ptr1 = src_ptr1.offset(-(Border as isize));
    dest_ptr2 = src_ptr2.offset(1);

    i = 0;
    while i < plane_height {
        ptr::write_bytes(dest_ptr1, *src_ptr1, Border as usize);
        ptr::write_bytes(dest_ptr2, *src_ptr2, Border as usize);
        src_ptr1 = src_ptr1.offset(plane_stride as isize);
        src_ptr2 = src_ptr2.offset(plane_stride as isize);
        dest_ptr1 = dest_ptr1.offset(plane_stride as isize);
        dest_ptr2 = dest_ptr2.offset(plane_stride as isize);
        i += 1;
    }

    /* U Plane */
    plane_stride = (*ybf).uv_stride;
    plane_height = 8;
    plane_width = (*ybf).uv_width;
    Border /= 2;

    src_ptr1 = u_src;
    src_ptr2 = src_ptr1.offset((plane_width - 1) as isize);
    dest_ptr1 = src_ptr1.offset(-(Border as isize));
    dest_ptr2 = src_ptr2.offset(1);

    i = 0;
    while i < plane_height {
        ptr::write_bytes(dest_ptr1, *src_ptr1, Border as usize);
        ptr::write_bytes(dest_ptr2, *src_ptr2, Border as usize);
        src_ptr1 = src_ptr1.offset(plane_stride as isize);
        src_ptr2 = src_ptr2.offset(plane_stride as isize);
        dest_ptr1 = dest_ptr1.offset(plane_stride as isize);
        dest_ptr2 = dest_ptr2.offset(plane_stride as isize);
        i += 1;
    }

    /* V Plane */
    src_ptr1 = v_src;
    src_ptr2 = src_ptr1.offset((plane_width - 1) as isize);
    dest_ptr1 = src_ptr1.offset(-(Border as isize));
    dest_ptr2 = src_ptr2.offset(1);

    i = 0;
    while i < plane_height {
        ptr::write_bytes(dest_ptr1, *src_ptr1, Border as usize);
        ptr::write_bytes(dest_ptr2, *src_ptr2, Border as usize);
        src_ptr1 = src_ptr1.offset(plane_stride as isize);
        src_ptr2 = src_ptr2.offset(plane_stride as isize);
        dest_ptr1 = dest_ptr1.offset(plane_stride as isize);
        dest_ptr2 = dest_ptr2.offset(plane_stride as isize);
        i += 1;
    }
}

// ---------------------------------------------------------------------------
// decode_mb_rows — decodeframe.c:436
// ---------------------------------------------------------------------------

/// `decode_mb_rows` (vp8/decoder/decodeframe.c:436). Static helper.
unsafe fn decode_mb_rows(pbi: *mut Vp8dComp<'static>) {
    let pc: *mut Vp8Common = &mut (*pbi).common;
    let xd: *mut Macroblockd = &mut (*pbi).mb;

    let mut lf_mic: *mut ModeInfo = (*xd).mode_info_context;

    let mut ibc: c_int = 0;
    let num_part: c_int = 1 << ((*pc).multi_token_partition as c_int);

    let mut recon_yoffset: c_int;
    let mut recon_uvoffset: c_int;
    let mut mb_row: c_int;
    let mut mb_col: c_int;

    let yv12_fb_new: *mut Yv12BufferConfig =
        &mut (*pbi).common.yv12_fb[(*pbi).dec_fb_ref_idx[INTRA_FRAME] as usize];

    let recon_y_stride: c_int = (*yv12_fb_new).y_stride;
    let recon_uv_stride: c_int = (*yv12_fb_new).uv_stride;

    let mut ref_buffer: [[*mut u8; 3]; MAX_REF_FRAMES] = [[ptr::null_mut(); 3]; MAX_REF_FRAMES];
    let mut dst_buffer: [*mut u8; 3] = [ptr::null_mut(); 3];
    let mut lf_dst: [*mut u8; 3] = [ptr::null_mut(); 3];
    let mut eb_dst: [*mut u8; 3] = [ptr::null_mut(); 3];
    let mut i: c_int;
    let mut ref_fb_corrupted: [c_int; MAX_REF_FRAMES] = [0; MAX_REF_FRAMES];

    ref_fb_corrupted[INTRA_FRAME] = 0;

    i = 1;
    while i < MAX_REF_FRAMES as c_int {
        let this_fb: *mut Yv12BufferConfig =
            &mut (*pbi).common.yv12_fb[(*pbi).dec_fb_ref_idx[i as usize] as usize];

        ref_buffer[i as usize][0] = (*this_fb).y_buffer;
        ref_buffer[i as usize][1] = (*this_fb).u_buffer;
        ref_buffer[i as usize][2] = (*this_fb).v_buffer;

        ref_fb_corrupted[i as usize] = (*this_fb).corrupted;
        i += 1;
    }

    /* Set up the buffer pointers */
    dst_buffer[0] = (*yv12_fb_new).y_buffer;
    lf_dst[0] = dst_buffer[0];
    eb_dst[0] = dst_buffer[0];
    dst_buffer[1] = (*yv12_fb_new).u_buffer;
    lf_dst[1] = dst_buffer[1];
    eb_dst[1] = dst_buffer[1];
    dst_buffer[2] = (*yv12_fb_new).v_buffer;
    lf_dst[2] = dst_buffer[2];
    eb_dst[2] = dst_buffer[2];

    (*xd).up_available = false;

    /* Initialize the loop filter for this frame. */
    if (*pc).filter_level != 0 {
        vp8_loop_filter_frame_init(pc, xd, (*pc).filter_level);
    }

    vp8_setup_intra_recon_top_line(yv12_fb_new);

    /* Decode the individual macro block */
    mb_row = 0;
    while mb_row < (*pc).mb_rows {
        // Pick the bool reader for this row: cycle through the N token
        // partitions when multi-partition, else always the lone reader.
        let bc: *mut Vp8Reader<'static> = if num_part > 1 {
            let p = &mut (*pbi).mbc[ibc as usize] as *mut Vp8Reader<'static>;
            ibc += 1;
            if ibc == num_part {
                ibc = 0;
            }
            p
        } else {
            &mut (*pbi).mbc[0] as *mut Vp8Reader<'static>
        };

        recon_yoffset = mb_row * recon_y_stride * 16;
        recon_uvoffset = mb_row * recon_uv_stride * 8;

        /* reset contexts */
        ptr::write_bytes(
            &mut (*pc).left_context as *mut _ as *mut u8,
            0,
            core::mem::size_of::<EntropyContextPlanes>(),
        );

        (*xd).left_available = false;

        (*xd).mb_to_top_edge = -((mb_row * 16) << 3);
        (*xd).mb_to_bottom_edge = ((*pc).mb_rows - 1 - mb_row) * 16 << 3;

        (*xd).recon_above[0] = dst_buffer[0].offset(recon_yoffset as isize);
        (*xd).recon_above[1] = dst_buffer[1].offset(recon_uvoffset as isize);
        (*xd).recon_above[2] = dst_buffer[2].offset(recon_uvoffset as isize);

        (*xd).recon_left[0] = (*xd).recon_above[0].offset(-1);
        (*xd).recon_left[1] = (*xd).recon_above[1].offset(-1);
        (*xd).recon_left[2] = (*xd).recon_above[2].offset(-1);

        (*xd).recon_above[0] = (*xd).recon_above[0].offset(-((*xd).dst.y_stride as isize));
        (*xd).recon_above[1] = (*xd).recon_above[1].offset(-((*xd).dst.uv_stride as isize));
        (*xd).recon_above[2] = (*xd).recon_above[2].offset(-((*xd).dst.uv_stride as isize));

        (*xd).recon_left_stride[0] = (*xd).dst.y_stride;
        (*xd).recon_left_stride[1] = (*xd).dst.uv_stride;

        setup_intra_recon_left(
            (*xd).recon_left[0],
            (*xd).recon_left[1],
            (*xd).recon_left[2],
            (*xd).dst.y_stride,
            (*xd).dst.uv_stride,
        );

        mb_col = 0;
        while mb_col < (*pc).mb_cols {
            /* Distance of Mb to the various image edges. */
            (*xd).mb_to_left_edge = -((mb_col * 16) << 3);
            (*xd).mb_to_right_edge = ((*pc).mb_cols - 1 - mb_col) * 16 << 3;

            (*xd).dst.y_buffer = dst_buffer[0].offset(recon_yoffset as isize);
            (*xd).dst.u_buffer = dst_buffer[1].offset(recon_uvoffset as isize);
            (*xd).dst.v_buffer = dst_buffer[2].offset(recon_uvoffset as isize);

            if (*(*xd).mode_info_context).mbmi.ref_frame as u8 >= LAST_FRAME as u8 {
                let ref_idx = (*(*xd).mode_info_context).mbmi.ref_frame as usize;
                (*xd).pre.y_buffer = ref_buffer[ref_idx][0].offset(recon_yoffset as isize);
                (*xd).pre.u_buffer = ref_buffer[ref_idx][1].offset(recon_uvoffset as isize);
                (*xd).pre.v_buffer = ref_buffer[ref_idx][2].offset(recon_uvoffset as isize);
            } else {
                // ref_frame is INTRA_FRAME, pre buffer should not be used.
                (*xd).pre.y_buffer = ptr::null_mut();
                (*xd).pre.u_buffer = ptr::null_mut();
                (*xd).pre.v_buffer = ptr::null_mut();
            }

            /* propagate errors from reference frames */
            (*xd).corrupted |= ref_fb_corrupted[(*(*xd).mode_info_context).mbmi.ref_frame as usize];

            decode_macroblock(pbi, xd, mb_col, bc);

            (*xd).left_available = true;

            /* check if the boolean decoder has suffered an error */
            (*xd).corrupted |= vp8dx_bool_error(bc);

            (*xd).recon_above[0] = (*xd).recon_above[0].add(16);
            (*xd).recon_above[1] = (*xd).recon_above[1].add(8);
            (*xd).recon_above[2] = (*xd).recon_above[2].add(8);
            (*xd).recon_left[0] = (*xd).recon_left[0].add(16);
            (*xd).recon_left[1] = (*xd).recon_left[1].add(8);
            (*xd).recon_left[2] = (*xd).recon_left[2].add(8);

            recon_yoffset += 16;
            recon_uvoffset += 8;

            (*xd).mode_info_context = (*xd).mode_info_context.add(1); /* next mb */

            mb_col += 1;
        }

        /* adjust to the next row of mbs */
        vp8_extend_mb_row(
            yv12_fb_new,
            (*xd).dst.y_buffer.add(16),
            (*xd).dst.u_buffer.add(8),
            (*xd).dst.v_buffer.add(8),
        );

        (*xd).mode_info_context = (*xd).mode_info_context.add(1); /* skip prediction column */
        (*xd).up_available = true;

        if (*pc).filter_level != 0 {
            if mb_row > 0 {
                if (*pc).filter_type == NORMAL_LOOPFILTER {
                    vp8_loop_filter_row_normal(
                        pc,
                        lf_mic,
                        mb_row - 1,
                        recon_y_stride,
                        recon_uv_stride,
                        lf_dst[0],
                        lf_dst[1],
                        lf_dst[2],
                    );
                } else {
                    vp8_loop_filter_row_simple(pc, lf_mic, mb_row - 1, recon_y_stride, lf_dst[0]);
                }
                if mb_row > 1 {
                    yv12_extend_frame_left_right_c(yv12_fb_new, eb_dst[0], eb_dst[1], eb_dst[2]);

                    eb_dst[0] = eb_dst[0].offset((recon_y_stride * 16) as isize);
                    eb_dst[1] = eb_dst[1].offset((recon_uv_stride * 8) as isize);
                    eb_dst[2] = eb_dst[2].offset((recon_uv_stride * 8) as isize);
                }

                lf_dst[0] = lf_dst[0].offset((recon_y_stride * 16) as isize);
                lf_dst[1] = lf_dst[1].offset((recon_uv_stride * 8) as isize);
                lf_dst[2] = lf_dst[2].offset((recon_uv_stride * 8) as isize);
                lf_mic = lf_mic.offset((*pc).mb_cols as isize);
                lf_mic = lf_mic.offset(1); /* Skip border mb */
            }
        } else {
            if mb_row > 0 {
                yv12_extend_frame_left_right_c(yv12_fb_new, eb_dst[0], eb_dst[1], eb_dst[2]);
                eb_dst[0] = eb_dst[0].offset((recon_y_stride * 16) as isize);
                eb_dst[1] = eb_dst[1].offset((recon_uv_stride * 8) as isize);
                eb_dst[2] = eb_dst[2].offset((recon_uv_stride * 8) as isize);
            }
        }

        mb_row += 1;
    }

    if (*pc).filter_level != 0 {
        if (*pc).filter_type == NORMAL_LOOPFILTER {
            vp8_loop_filter_row_normal(
                pc,
                lf_mic,
                mb_row - 1,
                recon_y_stride,
                recon_uv_stride,
                lf_dst[0],
                lf_dst[1],
                lf_dst[2],
            );
        } else {
            vp8_loop_filter_row_simple(pc, lf_mic, mb_row - 1, recon_y_stride, lf_dst[0]);
        }

        yv12_extend_frame_left_right_c(yv12_fb_new, eb_dst[0], eb_dst[1], eb_dst[2]);
        eb_dst[0] = eb_dst[0].offset((recon_y_stride * 16) as isize);
        eb_dst[1] = eb_dst[1].offset((recon_uv_stride * 8) as isize);
        eb_dst[2] = eb_dst[2].offset((recon_uv_stride * 8) as isize);
    }
    yv12_extend_frame_left_right_c(yv12_fb_new, eb_dst[0], eb_dst[1], eb_dst[2]);
    yv12_extend_frame_top_c(yv12_fb_new);
    yv12_extend_frame_bottom_c(yv12_fb_new);
}

// ---------------------------------------------------------------------------
// read_partition_size — decodeframe.c:663
// ---------------------------------------------------------------------------

/// `read_partition_size` (vp8/decoder/decodeframe.c:663). Static helper.
unsafe fn read_partition_size(pbi: *mut Vp8dComp<'static>, cx_size_in: *const u8) -> c_uint {
    let mut temp: [u8; 3] = [0; 3];
    let mut cx_size: *const u8 = cx_size_in;
    if let Some(cb) = (*pbi).decrypt.as_mut() {
        let src = core::slice::from_raw_parts(cx_size, 3);
        cb(src, &mut temp);
        cx_size = temp.as_ptr();
    }
    (*cx_size.add(0) as c_uint)
        + ((*cx_size.add(1) as c_uint) << 8)
        + ((*cx_size.add(2) as c_uint) << 16)
}

// ---------------------------------------------------------------------------
// read_is_valid — decodeframe.c:673
// ---------------------------------------------------------------------------

/// `read_is_valid` (vp8/decoder/decodeframe.c:673). Static helper.
unsafe fn read_is_valid(start: *const u8, len: usize, end: *const u8) -> c_int {
    let valid = len != 0 && end > start && len <= (end as usize).wrapping_sub(start as usize);
    valid as c_int
}

// ---------------------------------------------------------------------------
// read_available_partition_size — decodeframe.c:678
// ---------------------------------------------------------------------------

/// `read_available_partition_size` (vp8/decoder/decodeframe.c:678).
/// Static helper.
unsafe fn read_available_partition_size(
    pbi: *mut Vp8dComp<'static>,
    token_part_sizes: *const u8,
    fragment_start: *const u8,
    first_fragment_end: *const u8,
    fragment_end: *const u8,
    i: c_int,
    num_part: c_int,
) -> VpxResult<c_uint> {
    let pc: *mut Vp8Common = &mut (*pbi).common;
    let partition_size_ptr: *const u8 = token_part_sizes.offset((i * 3) as isize);
    let mut partition_size: c_uint;
    let bytes_left: isize = (fragment_end as isize) - (fragment_start as isize);
    if bytes_left < 0 {
        return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
    }
    /* Calculate the length of this partition. */
    if i < num_part - 1 {
        if read_is_valid(partition_size_ptr, 3, first_fragment_end) != 0 {
            partition_size = read_partition_size(pbi, partition_size_ptr);
        } else if (*pbi).ec_active != 0 {
            partition_size = bytes_left as c_uint;
        } else {
            return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
        }
    } else {
        partition_size = bytes_left as c_uint;
    }

    /* Validate the calculated partition length. */
    if read_is_valid(fragment_start, partition_size as usize, fragment_end) == 0 {
        if (*pbi).ec_active != 0 {
            partition_size = bytes_left as c_uint;
        } else {
            return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
        }
    }
    Ok(partition_size)
}

// ---------------------------------------------------------------------------
// setup_token_decoder — decodeframe.c:728
// ---------------------------------------------------------------------------

/// `setup_token_decoder` (vp8/decoder/decodeframe.c:728). Static helper.
unsafe fn setup_token_decoder(
    pbi: *mut Vp8dComp<'static>,
    token_part_sizes: *const u8,
) -> VpxResult<()> {
    let mut bool_decoder: *mut Vp8Reader<'static> = &mut (*pbi).mbc[0] as *mut Vp8Reader<'static>;
    let mut partition_idx: c_uint;
    let mut fragment_idx: c_uint;
    let num_token_partitions: c_uint;
    let first_fragment_end: *const u8 =
        (*pbi).fragments.ptrs[0].offset((*pbi).fragments.sizes[0] as isize);

    let mbc8: *mut Vp8Reader<'static> = &mut (*pbi).mbc[8] as *mut Vp8Reader<'static>;
    let multi_token_partition_val: c_int = vp8_read_literal(mbc8, 2);
    let multi_token_partition: TokenPartition = match multi_token_partition_val & 0x3 {
        0 => TokenPartition::One,
        1 => TokenPartition::Two,
        2 => TokenPartition::Four,
        _ => TokenPartition::Eight,
    };
    if vp8dx_bool_error(mbc8) == 0 {
        (*pbi).common.multi_token_partition = multi_token_partition;
    }
    num_token_partitions = 1u32 << ((*pbi).common.multi_token_partition as c_int);

    /* Walk the fragments and split each one into one-per-partition chunks. */
    fragment_idx = 0;
    while fragment_idx < (*pbi).fragments.count {
        let mut fragment_size: c_uint = (*pbi).fragments.sizes[fragment_idx as usize];
        let fragment_end: *const u8 =
            (*pbi).fragments.ptrs[fragment_idx as usize].offset(fragment_size as isize);
        /* Special case for handling the first partition since we have already
         * read its size. */
        if fragment_idx == 0 {
            /* Size of first partition + token partition sizes element */
            let ext_first_part_size: isize = (token_part_sizes as isize)
                - ((*pbi).fragments.ptrs[0] as isize)
                + (3 * (num_token_partitions as isize - 1));
            if (fragment_size as isize) < ext_first_part_size {
                return vpx_internal_error(&mut (*pbi).common.error, VPX_CODEC_CORRUPT_FRAME);
            }
            fragment_size = (fragment_size as isize - ext_first_part_size) as c_uint;
            if fragment_size > 0 {
                (*pbi).fragments.sizes[0] = ext_first_part_size as c_uint;
                /* The fragment contains an additional partition. */
                fragment_idx += 1;
                (*pbi).fragments.ptrs[fragment_idx as usize] =
                    (*pbi).fragments.ptrs[0].offset((*pbi).fragments.sizes[0] as isize);
            }
        }
        /* Split the chunk into partitions read from the bitstream */
        while fragment_size > 0 {
            let partition_size: c_uint = read_available_partition_size(
                pbi,
                token_part_sizes,
                (*pbi).fragments.ptrs[fragment_idx as usize],
                first_fragment_end,
                fragment_end,
                fragment_idx as c_int - 1,
                num_token_partitions as c_int,
            )?;
            (*pbi).fragments.sizes[fragment_idx as usize] = partition_size;
            if fragment_size < partition_size {
                return vpx_internal_error(&mut (*pbi).common.error, VPX_CODEC_CORRUPT_FRAME);
            }
            fragment_size -= partition_size;
            debug_assert!(fragment_idx <= num_token_partitions);
            if fragment_size > 0 {
                /* The fragment contains an additional partition. */
                fragment_idx += 1;
                (*pbi).fragments.ptrs[fragment_idx as usize] = (*pbi).fragments.ptrs
                    [(fragment_idx - 1) as usize]
                    .offset(partition_size as isize);
            }
        }
        fragment_idx += 1;
    }

    (*pbi).fragments.count = num_token_partitions + 1;

    partition_idx = 1;
    while partition_idx < (*pbi).fragments.count {
        if vp8dx_start_decode(
            bool_decoder,
            (*pbi).fragments.ptrs[partition_idx as usize],
            (*pbi).fragments.sizes[partition_idx as usize],
            (*pbi).decrypt.as_deref_mut(),
        ) != 0
        {
            return vpx_internal_error(&mut (*pbi).common.error, VPX_CODEC_MEM_ERROR);
        }

        bool_decoder = bool_decoder.add(1);
        partition_idx += 1;
    }

    // CONFIG_MULTITHREAD branch is intentionally omitted (minimal build).
    Ok(())
}

// ---------------------------------------------------------------------------
// init_frame — decodeframe.c:818
// ---------------------------------------------------------------------------

/// `init_frame` (vp8/decoder/decodeframe.c:818). Static helper.
unsafe fn init_frame(pbi: *mut Vp8dComp<'static>) {
    let pc: *mut Vp8Common = &mut (*pbi).common;
    let xd: *mut Macroblockd = &mut (*pbi).mb;

    if (*pc).frame_type == KEY_FRAME {
        /* Various keyframe initializations */
        (*pc).fc.mvc = VP8_DEFAULT_MV_CONTEXT;

        vp8_init_mbmode_probs(&mut *pc);

        vp8_default_coef_probs(pc);

        /* reset the segment feature data */
        ptr::write_bytes(
            (*xd).segment_feature_data.as_mut_ptr() as *mut u8,
            0,
            core::mem::size_of_val(&(*xd).segment_feature_data),
        );
        (*xd).mb_segment_abs_delta = SEGMENT_DELTADATA;

        /* reset the mode ref deltas for loop filter */
        ptr::write_bytes(
            (*xd).ref_lf_deltas.as_mut_ptr() as *mut u8,
            0,
            core::mem::size_of_val(&(*xd).ref_lf_deltas),
        );
        ptr::write_bytes(
            (*xd).mode_lf_deltas.as_mut_ptr() as *mut u8,
            0,
            core::mem::size_of_val(&(*xd).mode_lf_deltas),
        );

        /* All buffers are implicitly updated on key frames. */
        (*pc).refresh_golden_frame = 1;
        (*pc).refresh_alt_ref_frame = 1;
        (*pc).copy_buffer_to_gf = 0;
        (*pc).copy_buffer_to_arf = 0;

        /* Sign bias for Golden/Altref is meaningless on a key frame. */
        (*pc).ref_frame_sign_bias[GOLDEN_FRAME] = 0;
        (*pc).ref_frame_sign_bias[ALTREF_FRAME] = 0;
    } else {
        /* To enable choice of different interpolation filters */
        use crate::filter::{
            vp8_bilinear_predict4x4_c, vp8_bilinear_predict8x4_c, vp8_bilinear_predict8x8_c,
            vp8_bilinear_predict16x16_c, vp8_sixtap_predict4x4_c, vp8_sixtap_predict8x4_c,
            vp8_sixtap_predict8x8_c, vp8_sixtap_predict16x16_c,
        };
        if (*pc).use_bilinear_mc_filter == 0 {
            (*xd).subpixel_predict = vp8_sixtap_predict4x4_c;
            (*xd).subpixel_predict8x4 = vp8_sixtap_predict8x4_c;
            (*xd).subpixel_predict8x8 = vp8_sixtap_predict8x8_c;
            (*xd).subpixel_predict16x16 = vp8_sixtap_predict16x16_c;
        } else {
            (*xd).subpixel_predict = vp8_bilinear_predict4x4_c;
            (*xd).subpixel_predict8x4 = vp8_bilinear_predict8x4_c;
            (*xd).subpixel_predict8x8 = vp8_bilinear_predict8x8_c;
            (*xd).subpixel_predict16x16 = vp8_bilinear_predict16x16_c;
        }

        // Minimal build: CONFIG_ERROR_CONCEALMENT is off, so the
        // decoded_key_frame/ec_enabled/ec_active toggle is also off.
    }

    (*xd).mode_info_context = (*pc).mi_base_ptr();
    (*xd).frame_type = (*pc).frame_type;
    (*(*xd).mode_info_context).mbmi.mode = DC_PRED;
    (*xd).mode_info_stride = (*pc).mode_info_stride;
    (*xd).corrupted = 0; /* init without corruption */

    (*xd).fullpixel_mask = !0;
    if (*pc).full_pixel != 0 {
        (*xd).fullpixel_mask = !7;
    }
}

// ---------------------------------------------------------------------------
// vp8_decode_frame — decodeframe.c:879
// ---------------------------------------------------------------------------

/// `vp8_decode_frame` (vp8/decoder/decodeframe.c:879). Public entry point.
pub unsafe fn vp8_decode_frame(pbi: *mut Vp8dComp<'static>) -> VpxResult<()> {
    let bc: *mut Vp8Reader<'static> = &mut (*pbi).mbc[8] as *mut Vp8Reader<'static>;
    let pc: *mut Vp8Common = &mut (*pbi).common;
    let xd: *mut Macroblockd = &mut (*pbi).mb;
    let mut data: *const u8 = (*pbi).fragments.ptrs[0];
    let data_sz: c_uint = (*pbi).fragments.sizes[0];
    let data_end: *const u8 = data.offset(data_sz as isize);
    let first_partition_length_in_bytes: c_int;

    let mut i: c_int;
    let mut j: c_int;
    let mut k: c_int;
    let mut l: c_int;
    let mb_feature_data_bits: *const i32 = VP8_MB_FEATURE_DATA_BITS.as_ptr();
    let mut corrupt_tokens: c_int = 0;
    let prev_independent_partitions: c_int = (*pbi).independent_partitions;

    let yv12_fb_new: *mut Yv12BufferConfig =
        &mut (*pbi).common.yv12_fb[(*pbi).dec_fb_ref_idx[INTRA_FRAME] as usize];

    /* start with no corruption of current frame */
    (*xd).corrupted = 0;
    (*yv12_fb_new).corrupted = 0;

    if (data_end as isize) - (data as isize) < 3 {
        if (*pbi).ec_active == 0 {
            return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
        }

        /* Declare the missing frame as an inter frame. */
        (*pc).frame_type = INTER_FRAME;
        (*pc).version = 0;
        (*pc).show_frame = 1;
        first_partition_length_in_bytes = 0;
    } else {
        let mut clear_buffer: [u8; 10] = [0; 10];
        let mut clear: *const u8 = data;
        if let Some(cb) = (*pbi).decrypt.as_mut() {
            let n = core::cmp::min(clear_buffer.len(), data_sz as usize);
            let src = core::slice::from_raw_parts(data, n);
            cb(src, &mut clear_buffer[..n]);
            clear = clear_buffer.as_ptr();
        }

        (*pc).frame_type = if (*clear.add(0) & 1) == 0 {
            FrameType::Key
        } else {
            FrameType::Inter
        };
        (*pc).version = ((*clear.add(0) >> 1) & 7) as i32;
        (*pc).show_frame = ((*clear.add(0) >> 4) & 1) as i32;
        first_partition_length_in_bytes = (((*clear.add(0) as c_int)
            | ((*clear.add(1) as c_int) << 8)
            | ((*clear.add(2) as c_int) << 16))
            >> 5) as c_int;

        if (*pbi).ec_active == 0 && first_partition_length_in_bytes == 0 {
            return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
        }

        data = data.add(3);
        clear = clear.add(3);

        vp8_setup_version(&mut *pc);

        if (*pc).frame_type == KEY_FRAME {
            if (data_end as isize) - (data as isize) >= 7 {
                /* vet via sync code */
                if *clear.add(0) != 0x9d || *clear.add(1) != 0x01 || *clear.add(2) != 0x2a {
                    return vpx_internal_error(&mut (*pc).error, VPX_CODEC_UNSUP_BITSTREAM);
                }

                (*pc).width = ((*clear.add(3) as c_int) | ((*clear.add(4) as c_int) << 8)) & 0x3fff;
                (*pc).horiz_scale = (*clear.add(4) >> 6) as c_int;
                (*pc).height =
                    ((*clear.add(5) as c_int) | ((*clear.add(6) as c_int) << 8)) & 0x3fff;
                (*pc).vert_scale = (*clear.add(6) >> 6) as c_int;
                data = data.add(7);
            } else if (*pbi).ec_active == 0 {
                return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
            } else {
                /* Error concealment is active, clear the frame. */
                data = data_end;
            }
        } else {
            // The C `xd->pre = *yv12_fb_new; xd->dst = *yv12_fb_new;` does a
            // full struct copy of the YV12 config. `Yv12BufferConfig` is not
            // `Copy` (it owns raw pointers), so use ptr::copy to mirror the
            // C semantics byte-for-byte.
            ptr::copy_nonoverlapping(yv12_fb_new, &mut (*xd).pre, 1);
            ptr::copy_nonoverlapping(yv12_fb_new, &mut (*xd).dst, 1);
        }
    }
    if (*pbi).decoded_key_frame == 0 && (*pc).frame_type != KEY_FRAME {
        // C source returns -1 here without populating common.error;
        // the caller maps that to VPX_CODEC_ERROR.
        return Err(crate::vpx_api::VPX_CODEC_ERROR);
    }

    if (*pbi).ec_active == 0
        && ((data_end as isize) - (data as isize)) < first_partition_length_in_bytes as isize
    {
        return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
    }

    init_frame(pbi);

    if vp8dx_start_decode(
        bc,
        data,
        ((data_end as isize) - (data as isize)) as c_uint,
        (*pbi).decrypt.as_deref_mut(),
    ) != 0
    {
        return vpx_internal_error(&mut (*pc).error, VPX_CODEC_MEM_ERROR);
    }
    if (*pc).frame_type == KEY_FRAME {
        let _ = vp8_read_bit(bc); // colorspace
        (*pc).clamp_type = if vp8_read_bit(bc) == 0 {
            ClampType::Required
        } else {
            ClampType::NotRequired
        };
    }

    /* Is segmentation enabled */
    (*xd).segmentation_enabled = vp8_read_bit(bc) as u8;

    if (*xd).segmentation_enabled != 0 {
        /* segmentation map update flag */
        (*xd).update_mb_segmentation_map = vp8_read_bit(bc) as u8;
        (*xd).update_mb_segmentation_data = vp8_read_bit(bc) as u8;

        if (*xd).update_mb_segmentation_data != 0 {
            (*xd).mb_segment_abs_delta = vp8_read_bit(bc) as u8;

            ptr::write_bytes(
                (*xd).segment_feature_data.as_mut_ptr() as *mut u8,
                0,
                core::mem::size_of_val(&(*xd).segment_feature_data),
            );

            /* For each segmentation feature (Quant and loop filter level) */
            i = 0;
            while i < MB_LVL_MAX as c_int {
                j = 0;
                while j < MAX_MB_SEGMENTS as c_int {
                    /* Frame level data */
                    if vp8_read_bit(bc) != 0 {
                        (*xd).segment_feature_data[i as usize][j as usize] =
                            vp8_read_literal(bc, *mb_feature_data_bits.offset(i as isize)) as i8;

                        if vp8_read_bit(bc) != 0 {
                            (*xd).segment_feature_data[i as usize][j as usize] =
                                -(*xd).segment_feature_data[i as usize][j as usize];
                        }
                    } else {
                        (*xd).segment_feature_data[i as usize][j as usize] = 0;
                    }
                    j += 1;
                }
                i += 1;
            }
        }

        if (*xd).update_mb_segmentation_map != 0 {
            /* Which macro block level features are enabled */
            ptr::write_bytes(
                (*xd).mb_segment_tree_probs.as_mut_ptr() as *mut u8,
                255,
                core::mem::size_of_val(&(*xd).mb_segment_tree_probs),
            );

            /* Read probs used to decode segment id per macroblock. */
            i = 0;
            while i < MB_FEATURE_TREE_PROBS as c_int {
                if vp8_read_bit(bc) != 0 {
                    (*xd).mb_segment_tree_probs[i as usize] = vp8_read_literal(bc, 8) as Prob;
                }
                i += 1;
            }
        }
    } else {
        /* No segmentation updates on this frame */
        (*xd).update_mb_segmentation_map = 0;
        (*xd).update_mb_segmentation_data = 0;
    }

    /* Read the loop filter level and type */
    (*pc).filter_type = if vp8_read_bit(bc) == 0 {
        LoopFilterType::Normal
    } else {
        LoopFilterType::Simple
    };
    (*pc).filter_level = vp8_read_literal(bc, 6);
    (*pc).sharpness_level = vp8_read_literal(bc, 3);

    /* Read in loop filter deltas applied at the MB level. */
    (*xd).mode_ref_lf_delta_update = 0;
    (*xd).mode_ref_lf_delta_enabled = vp8_read_bit(bc) as u8;

    if (*xd).mode_ref_lf_delta_enabled != 0 {
        /* Do the deltas need to be updated */
        (*xd).mode_ref_lf_delta_update = vp8_read_bit(bc) as u8;

        if (*xd).mode_ref_lf_delta_update != 0 {
            /* Send update */
            i = 0;
            while i < MAX_REF_LF_DELTAS as c_int {
                if vp8_read_bit(bc) != 0 {
                    (*xd).ref_lf_deltas[i as usize] = vp8_read_literal(bc, 6) as i8;

                    if vp8_read_bit(bc) != 0 {
                        /* Apply sign */
                        (*xd).ref_lf_deltas[i as usize] = -(*xd).ref_lf_deltas[i as usize];
                    }
                }
                i += 1;
            }

            /* Send update */
            i = 0;
            while i < MAX_MODE_LF_DELTAS as c_int {
                if vp8_read_bit(bc) != 0 {
                    (*xd).mode_lf_deltas[i as usize] = vp8_read_literal(bc, 6) as i8;

                    if vp8_read_bit(bc) != 0 {
                        /* Apply sign */
                        (*xd).mode_lf_deltas[i as usize] = -(*xd).mode_lf_deltas[i as usize];
                    }
                }
                i += 1;
            }
        }
    }

    setup_token_decoder(pbi, data.offset(first_partition_length_in_bytes as isize))?;

    /* Read the default quantizers. */
    {
        let Q: c_int;
        let mut q_update: c_int;

        Q = vp8_read_literal(bc, 7); /* AC 1st order Q = default */
        (*pc).base_qindex = Q;
        q_update = 0;
        (*pc).y1dc_delta_q = get_delta_q(bc, (*pc).y1dc_delta_q, &mut q_update);
        (*pc).y2dc_delta_q = get_delta_q(bc, (*pc).y2dc_delta_q, &mut q_update);
        (*pc).y2ac_delta_q = get_delta_q(bc, (*pc).y2ac_delta_q, &mut q_update);
        (*pc).uvdc_delta_q = get_delta_q(bc, (*pc).uvdc_delta_q, &mut q_update);
        (*pc).uvac_delta_q = get_delta_q(bc, (*pc).uvac_delta_q, &mut q_update);

        if q_update != 0 {
            vp8cx_init_de_quantizer(pbi);
        }

        /* MB level dequantizer setup */
        vp8_mb_init_dequantizer(&(*pbi).common, &mut (*pbi).mb);
    }

    /* Determine if GF/ARF buffers should be updated and how. */
    if (*pc).frame_type != KEY_FRAME {
        /* GF/ARF refresh flags */
        (*pc).refresh_golden_frame = vp8_read_bit(bc);
        (*pc).refresh_alt_ref_frame = vp8_read_bit(bc);

        /* Buffer to buffer copy flags. */
        (*pc).copy_buffer_to_gf = 0;

        if (*pc).refresh_golden_frame == 0 {
            (*pc).copy_buffer_to_gf = vp8_read_literal(bc, 2);
        }

        (*pc).copy_buffer_to_arf = 0;

        if (*pc).refresh_alt_ref_frame == 0 {
            (*pc).copy_buffer_to_arf = vp8_read_literal(bc, 2);
        }

        (*pc).ref_frame_sign_bias[GOLDEN_FRAME] = vp8_read_bit(bc);
        (*pc).ref_frame_sign_bias[ALTREF_FRAME] = vp8_read_bit(bc);
    }

    (*pc).refresh_entropy_probs = vp8_read_bit(bc);
    if (*pc).refresh_entropy_probs == 0 {
        (*pc).lfc = (*pc).fc;
    }

    (*pc).refresh_last_frame = if (*pc).frame_type == KEY_FRAME {
        1
    } else {
        vp8_read_bit(bc)
    };

    {
        (*pbi).independent_partitions = 1;

        /* read coef probability tree */
        i = 0;
        while i < BLOCK_TYPES as c_int {
            j = 0;
            while j < COEF_BANDS as c_int {
                k = 0;
                while k < PREV_COEF_CONTEXTS as c_int {
                    l = 0;
                    while l < ENTROPY_NODES as c_int {
                        let p: *mut Prob = (*pc).fc.coef_probs[i as usize][j as usize][k as usize]
                            .as_mut_ptr()
                            .offset(l as isize);

                        if vp8_read(
                            bc,
                            VP8_COEF_UPDATE_PROBS[i as usize][j as usize][k as usize][l as usize]
                                as c_int,
                        ) != 0
                        {
                            *p = vp8_read_literal(bc, 8) as Prob;
                        }
                        if k > 0
                            && *p
                                != (*pc).fc.coef_probs[i as usize][j as usize][(k - 1) as usize]
                                    [l as usize]
                        {
                            (*pbi).independent_partitions = 0;
                        }
                        l += 1;
                    }
                    k += 1;
                }
                j += 1;
            }
            i += 1;
        }
    }

    /* clear out the coeff buffer */
    ptr::write_bytes(
        (*xd).qcoeff.as_mut_ptr() as *mut u8,
        0,
        core::mem::size_of_val(&(*xd).qcoeff),
    );

    vp8_decode_mode_mvs(pbi);

    ptr::write_bytes(
        (*pc).above_context.as_deref_mut().unwrap().as_mut_ptr() as *mut u8,
        0,
        core::mem::size_of::<EntropyContextPlanes>() * (*pc).mb_cols as usize,
    );
    (*pbi).frame_corrupt_residual = 0;

    {
        decode_mb_rows(pbi);
        corrupt_tokens |= (*xd).corrupted;
    }

    /* Collect information about decoder corruption. */
    /* 1. Check first boolean decoder for errors. */
    (*yv12_fb_new).corrupted = vp8dx_bool_error(bc);
    /* 2. Check the macroblock information */
    (*yv12_fb_new).corrupted |= corrupt_tokens;

    if (*pbi).decoded_key_frame == 0 {
        if (*pc).frame_type == KEY_FRAME && (*yv12_fb_new).corrupted == 0 {
            (*pbi).decoded_key_frame = 1;
        } else {
            return vpx_internal_error(&mut (*pbi).common.error, VPX_CODEC_CORRUPT_FRAME);
        }
    }

    if (*pc).refresh_entropy_probs == 0 {
        (*pc).fc = (*pc).lfc;
        (*pbi).independent_partitions = prev_independent_partitions;
    }

    Ok(())
}
