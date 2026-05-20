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
    BLOCK_TYPES, COEF_BANDS, ENTROPY_NODES, MAXQ, PREV_COEF_CONTEXTS, Prob, QINDEX_RANGE,
    VP8_COEF_UPDATE_PROBS, VP8_DEFAULT_MV_CONTEXT, VP8_MB_FEATURE_DATA_BITS,
};
use crate::types::{
    ClampType, EntropyContextPlanes, FrameContext, FrameType, LoopFilterType, MAX_MB_SEGMENTS,
    MAX_MODE_LF_DELTAS, MAX_REF_FRAMES, MAX_REF_LF_DELTAS, MB_FEATURE_TREE_PROBS, MB_LVL_MAX,
    Macroblockd, MbLevelFeature, MbPredictionMode, ModeInfo, MvReferenceFrame, TokenPartition,
    Vp8Common, Vp8Reader, Vp8dComp, VpxResult, Yv12BufferConfig,
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
pub fn vp8cx_init_de_quantizer(pc: &mut Vp8Common) {
    let mut Q: c_int;

    Q = 0;
    while Q < crate::tables::QINDEX_RANGE as c_int {
        pc.y1_dequant[Q as usize][0] = vp8_dc_quant(Q, pc.y1dc_delta_q) as i16;
        pc.y2_dequant[Q as usize][0] = vp8_dc2quant(Q, pc.y2dc_delta_q) as i16;
        pc.uv_dequant[Q as usize][0] = vp8_dc_uv_quant(Q, pc.uvdc_delta_q) as i16;

        pc.y1_dequant[Q as usize][1] = vp8_ac_yquant(Q) as i16;
        pc.y2_dequant[Q as usize][1] = vp8_ac2quant(Q, pc.y2ac_delta_q) as i16;
        pc.uv_dequant[Q as usize][1] = vp8_ac_uv_quant(Q, pc.uvac_delta_q) as i16;

        Q += 1;
    }
}

// ---------------------------------------------------------------------------
// vp8_mb_init_dequantizer — decodeframe.c:57
// ---------------------------------------------------------------------------

/// `vp8_mb_init_dequantizer` (vp8/decoder/decodeframe.c:57).
pub fn vp8_mb_init_dequantizer(
    y1_dequant: &[[i16; 2]; QINDEX_RANGE],
    y2_dequant: &[[i16; 2]; QINDEX_RANGE],
    uv_dequant: &[[i16; 2]; QINDEX_RANGE],
    base_qindex: c_int,
    mb: &mut Macroblockd,
    mi: &ModeInfo,
) {
    let segment_id = mi.mbmi.segment_id as usize;

    /* Decide whether to use the default or alternate baseline Q value. */
    let qi: usize = (if mb.segmentation_enabled != 0 {
        let q = if mb.mb_segment_abs_delta == SEGMENT_ABSDATA {
            mb.segment_feature_data[MB_LVL_ALT_Q][segment_id] as c_int
        } else {
            base_qindex + mb.segment_feature_data[MB_LVL_ALT_Q][segment_id] as c_int
        };
        q.clamp(0, MAXQ as c_int)
    } else {
        base_qindex
    }) as usize;

    /* Set up the macroblock dequant constants */
    mb.dequant_y1_dc[0] = 1;
    mb.dequant_y1[0] = y1_dequant[qi][0];
    mb.dequant_y2[0] = y2_dequant[qi][0];
    mb.dequant_uv[0] = uv_dequant[qi][0];

    for i in 1..16 {
        let ac = y1_dequant[qi][1];
        mb.dequant_y1_dc[i] = ac;
        mb.dequant_y1[i] = ac;
        mb.dequant_y2[i] = y2_dequant[qi][1];
        mb.dequant_uv[i] = uv_dequant[qi][1];
    }
}

// ---------------------------------------------------------------------------
// decode_macroblock — decodeframe.c:94
// ---------------------------------------------------------------------------

/// `decode_macroblock` (vp8/decoder/decodeframe.c:94). Static helper.
fn decode_macroblock(
    fc: &FrameContext,
    above_slot: &mut EntropyContextPlanes,
    left_context: &mut EntropyContextPlanes,
    y1_dequant: &[[i16; 2]; QINDEX_RANGE],
    y2_dequant: &[[i16; 2]; QINDEX_RANGE],
    uv_dequant: &[[i16; 2]; QINDEX_RANGE],
    base_qindex: c_int,
    xd: &mut Macroblockd,
    mi: &mut ModeInfo,
    bc: &mut Vp8Reader<'static>,
) {
    let mode: MbPredictionMode;

    if mi.mbmi.mb_skip_coeff {
        vp8_reset_mb_tokens_context(above_slot, left_context, mi);
    } else if vp8dx_bool_error(bc) == 0 {
        let eobtotal: c_int =
            vp8_decode_mb_tokens(above_slot, left_context, fc, xd, mi, bc);

        /* Special case:  Force the loopfilter to skip when eobtotal is zero */
        mi.mbmi.mb_skip_coeff = eobtotal == 0;
    }

    mode = mi.mbmi.mode;

    if xd.segmentation_enabled != 0 {
        vp8_mb_init_dequantizer(y1_dequant, y2_dequant, uv_dequant, base_qindex, xd, mi);
    }

    /* do prediction */
    if mi.mbmi.ref_frame == MvReferenceFrame::Intra {
        let y_stride: isize = xd.dst.y_stride as isize;
        let uv_stride: isize = xd.dst.uv_stride as isize;
        // SAFETY: plane-pointer arithmetic for neighbor rows reaches
        // valid pixels in xd.dst (yabove = y_buffer - y_stride; yleft =
        // y_buffer - 1). The predictor calls themselves are safe fn.
        let (uabove, vabove, uleft, vleft, yabove, yleft) = unsafe {
            (
                xd.dst.u_buffer.offset(-uv_stride),
                xd.dst.v_buffer.offset(-uv_stride),
                xd.dst.u_buffer.offset(-1),
                xd.dst.v_buffer.offset(-1),
                xd.dst.y_buffer.offset(-y_stride),
                xd.dst.y_buffer.offset(-1),
            )
        };
        vp8_build_intra_predictors_mbuv_s(
            xd, mi, uabove, vabove, uleft, vleft,
            xd.dst.uv_stride, xd.dst.u_buffer, xd.dst.v_buffer, xd.dst.uv_stride,
        );

        if mode != B_PRED {
            vp8_build_intra_predictors_mby_s(
                xd, mi, yabove, yleft, xd.dst.y_stride, xd.dst.y_buffer, xd.dst.y_stride,
            );
        } else {
            let dst_stride: c_int = xd.dst.y_stride;

            /* clear out residual eob info */
            if mi.mbmi.mb_skip_coeff {
                xd.eobs.fill(0);
            }

            // SAFETY: above_right_src reaches the above row of the dst plane.
            let above_right = unsafe { xd.dst.y_buffer.offset(-y_stride).add(16) };
            intra_prediction_down_copy(xd, above_right);

            for i in 0..16usize {
                let b_mode_val: crate::types::BPredictionMode = match mi.bmi[i] {
                    crate::types::BModeInfo::Intra(m) => m,
                    crate::types::BModeInfo::Mv(_) => crate::types::BPredictionMode::DcPred,
                };
                let b_offset = xd.block[i].offset as isize;
                // SAFETY: per-sub-block pixel pointers stay within the
                // dst luma plane; qcoeff/DQC index well-defined 16-coeff
                // ranges of xd.qcoeff/xd.dequant_y1.
                let (dst, above, yleft, top_left, qcoeff, DQC) = unsafe {
                    let dst = xd.dst.y_buffer.offset(b_offset);
                    let above = dst.offset(-(dst_stride as isize));
                    (
                        dst,
                        above,
                        dst.offset(-1),
                        *above.offset(-1),
                        xd.qcoeff.as_mut_ptr().add(i * 16),
                        xd.dequant_y1.as_mut_ptr(),
                    )
                };
                vp8_intra4x4_predict(above, yleft, dst_stride, b_mode_val, dst, dst_stride, top_left);

                if xd.eobs[i] != 0 {
                    if xd.eobs[i] > 1 {
                        vp8_dequant_idct_add(qcoeff, DQC, dst, dst_stride);
                    } else {
                        // SAFETY: qcoeff/DQC[0] reads are 1 i16 each.
                        let (q0, dqc0) = unsafe { (*qcoeff, *DQC) };
                        vp8_dc_only_idct_add(
                            (q0 as i32 * dqc0 as i32) as i16,
                            dst, dst_stride, dst, dst_stride,
                        );
                        // SAFETY: clear 2 i16 entries of qcoeff.
                        unsafe {
                            ptr::write_bytes(qcoeff as *mut u8, 0, 2 * core::mem::size_of::<i16>());
                        }
                    }
                }
            }
        }
    } else {
        vp8_build_inter_predictors_mb(xd, mi);
    }

    if !mi.mbmi.mb_skip_coeff {
        /* dequantization and idct */
        if mode != B_PRED {
            // SAFETY: qcoeff/dqcoeff/dequant raw pointers are derived
            // from xd's owned arrays at well-defined block offsets.
            let mut DQC: *mut i16 = xd.dequant_y1.as_mut_ptr();

            if mode != SPLITMV {
                let (y2_qcoeff, y2_dqcoeff, dequant_y2) = unsafe {
                    (
                        xd.qcoeff.as_mut_ptr().add(24 * 16),
                        xd.dqcoeff.as_mut_ptr().add(24 * 16),
                        xd.dequant_y2.as_mut_ptr(),
                    )
                };

                /* do 2nd order transform on the dc block */
                if xd.eobs[24] > 1 {
                    vp8_dequantize_b(y2_qcoeff, y2_dqcoeff, dequant_y2);
                    vp8_short_inv_walsh4x4(y2_dqcoeff, xd.qcoeff.as_mut_ptr());
                    // SAFETY: clear 16 i16 entries at y2 slot.
                    unsafe {
                        ptr::write_bytes(y2_qcoeff as *mut u8, 0, 16 * core::mem::size_of::<i16>());
                    }
                } else {
                    // SAFETY: read 1 i16 from the y2 qcoeff slot.
                    let q0 = unsafe { *y2_qcoeff };
                    let dq0 = xd.dequant_y2[0];
                    // SAFETY: write 1 i16 at the y2 dqcoeff slot.
                    unsafe { *y2_dqcoeff = (q0 as i32 * dq0 as i32) as i16; }
                    vp8_short_inv_walsh4x4_1(y2_dqcoeff, xd.qcoeff.as_mut_ptr());
                    // SAFETY: clear 2 i16 entries.
                    unsafe {
                        ptr::write_bytes(y2_qcoeff as *mut u8, 0, 2 * core::mem::size_of::<i16>());
                    }
                }

                /* override the dc dequant constant in order to preserve the
                 * dc components
                 */
                DQC = xd.dequant_y1_dc.as_mut_ptr();
            }

            vp8_dequant_idct_add_y_block(
                xd.qcoeff.as_mut_ptr(),
                DQC,
                xd.dst.y_buffer,
                xd.dst.y_stride,
                xd.eobs.as_mut_ptr(),
            );
        }

        // SAFETY: chroma qcoeff/eobs at known offsets; uv buffers from xd.dst.
        let (uv_q, uv_dq, uv_eobs) = unsafe {
            (
                xd.qcoeff.as_mut_ptr().add(16 * 16),
                xd.dequant_uv.as_mut_ptr(),
                xd.eobs.as_mut_ptr().add(16),
            )
        };
        vp8_dequant_idct_add_uv_block(
            uv_q, uv_dq, xd.dst.u_buffer, xd.dst.v_buffer, xd.dst.uv_stride, uv_eobs,
        );
    }
}

// ---------------------------------------------------------------------------
// get_delta_q — decodeframe.c:235
// ---------------------------------------------------------------------------

/// `get_delta_q` (vp8/decoder/decodeframe.c:235). Static helper.
fn get_delta_q(bc: &mut Vp8Reader<'static>, prev: c_int, q_update: &mut c_int) -> c_int {
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
fn decode_mb_rows(pbi: &mut Vp8dComp<'static>) {
    let num_part: c_int = 1 << (pbi.common.multi_token_partition as c_int);
    let mut ibc: c_int = 0;

    let new_idx = pbi.dec_fb_ref_idx[INTRA_FRAME] as usize;

    // Snapshot reference frame plane pointers + corrupted flags up front.
    // These come from yv12_fb slots distinct from the new (output) slot.
    let mut ref_buffer: [[*mut u8; 3]; MAX_REF_FRAMES] = [[ptr::null_mut(); 3]; MAX_REF_FRAMES];
    let mut ref_fb_corrupted: [c_int; MAX_REF_FRAMES] = [0; MAX_REF_FRAMES];
    for i in 1..MAX_REF_FRAMES {
        let this_fb = &pbi.common.yv12_fb[pbi.dec_fb_ref_idx[i] as usize];
        ref_buffer[i][0] = this_fb.y_buffer;
        ref_buffer[i][1] = this_fb.u_buffer;
        ref_buffer[i][2] = this_fb.v_buffer;
        ref_fb_corrupted[i] = this_fb.corrupted;
    }

    // Snapshot the new frame's plane state. We keep a raw pointer to the
    // Yv12BufferConfig for sub-calls that take *mut, and capture plane
    // pointers/strides as locals.
    let yv12_fb_new: *mut Yv12BufferConfig = &mut pbi.common.yv12_fb[new_idx];
    let (recon_y_stride, recon_uv_stride, dst_y, dst_u, dst_v) = {
        let yv12 = &pbi.common.yv12_fb[new_idx];
        (yv12.y_stride, yv12.uv_stride, yv12.y_buffer, yv12.u_buffer, yv12.v_buffer)
    };
    let dst_buffer: [*mut u8; 3] = [dst_y, dst_u, dst_v];
    let mut lf_dst: [*mut u8; 3] = [dst_y, dst_u, dst_v];
    let mut eb_dst: [*mut u8; 3] = [dst_y, dst_u, dst_v];

    pbi.mb.up_available = false;

    /* Initialize the loop filter for this frame. */
    if pbi.common.filter_level != 0 {
        let filter_level = pbi.common.filter_level;
        vp8_loop_filter_frame_init(&mut pbi.common, &pbi.mb, filter_level);
    }

    // SAFETY: yv12_fb_new points to the live new-frame slot in pbi.common.
    unsafe { vp8_setup_intra_recon_top_line(yv12_fb_new); }

    let mb_rows = pbi.common.mb_rows;
    let mb_cols = pbi.common.mb_cols;

    /* Decode the individual macro block */
    let mut mb_row: c_int = 0;
    while mb_row < mb_rows {
        // Pick the bool reader for this row: cycle through N token
        // partitions when multi-partition, else always the lone reader.
        let bc_idx: usize = if num_part > 1 {
            let cur = ibc as usize;
            ibc += 1;
            if ibc == num_part {
                ibc = 0;
            }
            cur
        } else {
            0
        };

        let mut recon_yoffset: c_int = mb_row * recon_y_stride * 16;
        let mut recon_uvoffset: c_int = mb_row * recon_uv_stride * 8;

        /* reset contexts */
        pbi.common.left_context = EntropyContextPlanes::default();

        pbi.mb.left_available = false;
        pbi.mb.mb_to_top_edge = -((mb_row * 16) << 3);
        pbi.mb.mb_to_bottom_edge = (mb_rows - 1 - mb_row) * 16 << 3;

        // SAFETY: dst_buffer pointers reach the new-frame plane allocation;
        // the -1 offsets compute the left-column scratch location used by
        // intra-prediction. setup_intra_recon_left writes 16 byte stripes.
        unsafe {
            let y_row_base = dst_buffer[0].offset(recon_yoffset as isize);
            let u_row_base = dst_buffer[1].offset(recon_uvoffset as isize);
            let v_row_base = dst_buffer[2].offset(recon_uvoffset as isize);
            setup_intra_recon_left(
                y_row_base.offset(-1),
                u_row_base.offset(-1),
                v_row_base.offset(-1),
                recon_y_stride,
                recon_uv_stride,
            );
        }

        let mut mb_col: c_int = 0;
        while mb_col < mb_cols {
            /* Distance of Mb to the various image edges. */
            pbi.mb.mb_to_left_edge = -((mb_col * 16) << 3);
            pbi.mb.mb_to_right_edge = (mb_cols - 1 - mb_col) * 16 << 3;

            // SAFETY: dst_buffer pointers + recon_yoffset stay within the
            // new-frame plane allocation.
            unsafe {
                pbi.mb.dst.y_buffer = dst_buffer[0].offset(recon_yoffset as isize);
                pbi.mb.dst.u_buffer = dst_buffer[1].offset(recon_uvoffset as isize);
                pbi.mb.dst.v_buffer = dst_buffer[2].offset(recon_uvoffset as isize);
            }

            // Current MB's MI cell, borrowed once as a direct field path
            // (`common.mip`, not the `mi_mut` method) and held across the
            // rest of the loop body. The intervening code touches only
            // `pbi.mb` and disjoint `pbi.common` sub-fields, so this `&mut`
            // coexists with all of them — and reusing it for the ref_frame
            // read avoids a second bounds-checked grid index per MB.
            let mi: &mut ModeInfo = {
                let stride = pbi.common.mode_info_stride as usize;
                let idx = ((mb_row + 1) as usize) * stride + ((mb_col + 1) as usize);
                &mut pbi.common.mip.as_deref_mut().expect("MI grid not allocated")[idx]
            };
            let ref_frame = mi.mbmi.ref_frame;

            if ref_frame as u8 >= LAST_FRAME as u8 {
                let ref_idx = ref_frame as usize;
                // SAFETY: ref_buffer points into a distinct yv12_fb slot.
                unsafe {
                    pbi.mb.pre.y_buffer = ref_buffer[ref_idx][0].offset(recon_yoffset as isize);
                    pbi.mb.pre.u_buffer = ref_buffer[ref_idx][1].offset(recon_uvoffset as isize);
                    pbi.mb.pre.v_buffer = ref_buffer[ref_idx][2].offset(recon_uvoffset as isize);
                }
            } else {
                pbi.mb.pre.y_buffer = ptr::null_mut();
                pbi.mb.pre.u_buffer = ptr::null_mut();
                pbi.mb.pre.v_buffer = ptr::null_mut();
            }

            /* propagate errors from reference frames */
            pbi.mb.corrupted |= ref_fb_corrupted[ref_frame as usize];

            // Field-disjoint borrows for decode_macroblock: every source
            // is a distinct field path off `pbi`, so the `&mut` into the
            // current MI cell (`common.mip`, taken above) coexists with the
            // `common.above_context` / `left_context` / `fc` / dequant
            // borrows and with `pbi.mb` / `pbi.mbc[bc_idx]`.
            let above_slot = &mut pbi
                .common
                .above_context
                .as_deref_mut()
                .expect("above_context allocated")[mb_col as usize];
            let left_context = &mut pbi.common.left_context;
            let fc = &pbi.common.fc;
            let y1_dq = &pbi.common.y1_dequant;
            let y2_dq = &pbi.common.y2_dequant;
            let uv_dq = &pbi.common.uv_dequant;
            let base_qi = pbi.common.base_qindex;
            let mb = &mut pbi.mb;
            let bc = &mut pbi.mbc[bc_idx];

            decode_macroblock(
                fc,
                above_slot,
                left_context,
                y1_dq,
                y2_dq,
                uv_dq,
                base_qi,
                mb,
                mi,
                bc,
            );

            pbi.mb.left_available = true;

            /* check if the boolean decoder has suffered an error */
            pbi.mb.corrupted |= vp8dx_bool_error(&pbi.mbc[bc_idx]);

            recon_yoffset += 16;
            recon_uvoffset += 8;

            mb_col += 1;
        }

        // SAFETY: dst.{y,u,v}_buffer point to live plane memory.
        unsafe {
            vp8_extend_mb_row(
                yv12_fb_new,
                pbi.mb.dst.y_buffer.add(16),
                pbi.mb.dst.u_buffer.add(8),
                pbi.mb.dst.v_buffer.add(8),
            );
        }

        pbi.mb.up_available = true;

        if pbi.common.filter_level != 0 {
            if mb_row > 0 {
                // SAFETY: lf_dst/eb_dst pointers walk the live new-frame
                // planes; loop-filter routines operate within the allocation.
                unsafe {
                    if pbi.common.filter_type == NORMAL_LOOPFILTER {
                        vp8_loop_filter_row_normal(
                            &mut pbi.common,
                            mb_row - 1,
                            recon_y_stride,
                            recon_uv_stride,
                            lf_dst[0],
                            lf_dst[1],
                            lf_dst[2],
                        );
                    } else {
                        vp8_loop_filter_row_simple(
                            &mut pbi.common,
                            mb_row - 1,
                            recon_y_stride,
                            lf_dst[0],
                        );
                    }
                    if mb_row > 1 {
                        yv12_extend_frame_left_right_c(
                            yv12_fb_new,
                            eb_dst[0],
                            eb_dst[1],
                            eb_dst[2],
                        );

                        eb_dst[0] = eb_dst[0].offset((recon_y_stride * 16) as isize);
                        eb_dst[1] = eb_dst[1].offset((recon_uv_stride * 8) as isize);
                        eb_dst[2] = eb_dst[2].offset((recon_uv_stride * 8) as isize);
                    }

                    lf_dst[0] = lf_dst[0].offset((recon_y_stride * 16) as isize);
                    lf_dst[1] = lf_dst[1].offset((recon_uv_stride * 8) as isize);
                    lf_dst[2] = lf_dst[2].offset((recon_uv_stride * 8) as isize);
                }
            }
        } else if mb_row > 0 {
            // SAFETY: extend the previous row's edge pixels.
            unsafe {
                yv12_extend_frame_left_right_c(yv12_fb_new, eb_dst[0], eb_dst[1], eb_dst[2]);
                eb_dst[0] = eb_dst[0].offset((recon_y_stride * 16) as isize);
                eb_dst[1] = eb_dst[1].offset((recon_uv_stride * 8) as isize);
                eb_dst[2] = eb_dst[2].offset((recon_uv_stride * 8) as isize);
            }
        }

        mb_row += 1;
    }

    if pbi.common.filter_level != 0 {
        // SAFETY: final row's loop-filter + edge-extend on the live plane.
        unsafe {
            if pbi.common.filter_type == NORMAL_LOOPFILTER {
                vp8_loop_filter_row_normal(
                    &mut pbi.common,
                    mb_row - 1,
                    recon_y_stride,
                    recon_uv_stride,
                    lf_dst[0],
                    lf_dst[1],
                    lf_dst[2],
                );
            } else {
                vp8_loop_filter_row_simple(
                    &mut pbi.common,
                    mb_row - 1,
                    recon_y_stride,
                    lf_dst[0],
                );
            }

            yv12_extend_frame_left_right_c(yv12_fb_new, eb_dst[0], eb_dst[1], eb_dst[2]);
            eb_dst[0] = eb_dst[0].offset((recon_y_stride * 16) as isize);
            eb_dst[1] = eb_dst[1].offset((recon_uv_stride * 8) as isize);
            eb_dst[2] = eb_dst[2].offset((recon_uv_stride * 8) as isize);
        }
    }
    // SAFETY: extend remaining edges + top/bottom borders.
    unsafe {
        yv12_extend_frame_left_right_c(yv12_fb_new, eb_dst[0], eb_dst[1], eb_dst[2]);
        yv12_extend_frame_top_c(yv12_fb_new);
        yv12_extend_frame_bottom_c(yv12_fb_new);
    }
}

// ---------------------------------------------------------------------------
// read_partition_size — decodeframe.c:663
// ---------------------------------------------------------------------------

/// `read_partition_size` (vp8/decoder/decodeframe.c:663). Static helper.
unsafe fn read_partition_size(cx_size: *const u8) -> c_uint {
    (*cx_size.add(0) as c_uint)
        + ((*cx_size.add(1) as c_uint) << 8)
        + ((*cx_size.add(2) as c_uint) << 16)
}

// ---------------------------------------------------------------------------
// read_is_valid — decodeframe.c:673
// ---------------------------------------------------------------------------

/// `read_is_valid` (vp8/decoder/decodeframe.c:673). Static helper.
///
/// Body performs only pointer comparisons / address arithmetic (no
/// dereferences), so it is safe to call from safe contexts.
fn read_is_valid(start: *const u8, len: usize, end: *const u8) -> c_int {
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
            partition_size = read_partition_size(partition_size_ptr);
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

    let multi_token_partition_val: c_int = vp8_read_literal(&mut (*pbi).mbc[8], 2);
    let multi_token_partition: TokenPartition = match multi_token_partition_val & 0x3 {
        0 => TokenPartition::One,
        1 => TokenPartition::Two,
        2 => TokenPartition::Four,
        _ => TokenPartition::Eight,
    };
    if vp8dx_bool_error(&(*pbi).mbc[8]) == 0 {
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
            &mut *bool_decoder,
            (*pbi).fragments.ptrs[partition_idx as usize],
            (*pbi).fragments.sizes[partition_idx as usize],
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
///
/// `common` and `mb` are disjoint fields of `pbi`, so all accesses go
/// through direct field paths — no raw-pointer aliases needed.
fn init_frame(pbi: &mut Vp8dComp<'static>) {
    if pbi.common.frame_type == KEY_FRAME {
        /* Various keyframe initializations */
        pbi.common.fc.mvc = VP8_DEFAULT_MV_CONTEXT;

        vp8_init_mbmode_probs(&mut pbi.common);

        vp8_default_coef_probs(&mut pbi.common);

        /* reset the segment feature data */
        pbi.mb.segment_feature_data = Default::default();
        pbi.mb.mb_segment_abs_delta = SEGMENT_DELTADATA;

        /* reset the mode ref deltas for loop filter */
        pbi.mb.ref_lf_deltas = Default::default();
        pbi.mb.mode_lf_deltas = Default::default();

        /* All buffers are implicitly updated on key frames. */
        pbi.common.refresh_golden_frame = 1;
        pbi.common.refresh_alt_ref_frame = 1;
        pbi.common.copy_buffer_to_gf = 0;
        pbi.common.copy_buffer_to_arf = 0;

        /* Sign bias for Golden/Altref is meaningless on a key frame. */
        pbi.common.ref_frame_sign_bias[GOLDEN_FRAME] = 0;
        pbi.common.ref_frame_sign_bias[ALTREF_FRAME] = 0;
    } else {
        /* To enable choice of different interpolation filters */
        use crate::filter::{
            vp8_bilinear_predict4x4_c, vp8_bilinear_predict8x4_c, vp8_bilinear_predict8x8_c,
            vp8_bilinear_predict16x16_c, vp8_sixtap_predict4x4_c, vp8_sixtap_predict8x4_c,
            vp8_sixtap_predict8x8_c, vp8_sixtap_predict16x16_c,
        };
        if pbi.common.use_bilinear_mc_filter == 0 {
            pbi.mb.subpixel_predict = vp8_sixtap_predict4x4_c;
            pbi.mb.subpixel_predict8x4 = vp8_sixtap_predict8x4_c;
            pbi.mb.subpixel_predict8x8 = vp8_sixtap_predict8x8_c;
            pbi.mb.subpixel_predict16x16 = vp8_sixtap_predict16x16_c;
        } else {
            pbi.mb.subpixel_predict = vp8_bilinear_predict4x4_c;
            pbi.mb.subpixel_predict8x4 = vp8_bilinear_predict8x4_c;
            pbi.mb.subpixel_predict8x8 = vp8_bilinear_predict8x8_c;
            pbi.mb.subpixel_predict16x16 = vp8_bilinear_predict16x16_c;
        }

        // Minimal build: CONFIG_ERROR_CONCEALMENT is off, so the
        // decoded_key_frame/ec_enabled/ec_active toggle is also off.
    }

    pbi.mb.frame_type = pbi.common.frame_type;
    pbi.common.mi_mut(0, 0).mbmi.mode = DC_PRED;
    pbi.mb.mode_info_stride = pbi.common.mode_info_stride;
    pbi.mb.corrupted = 0; /* init without corruption */

    pbi.mb.fullpixel_mask = !0;
    if pbi.common.full_pixel != 0 {
        pbi.mb.fullpixel_mask = !7;
    }
}

// ---------------------------------------------------------------------------
// vp8_decode_frame — decodeframe.c:879
// ---------------------------------------------------------------------------

/// `vp8_decode_frame` (vp8/decoder/decodeframe.c:879). Public entry point.
pub fn vp8_decode_frame(pbi: &mut Vp8dComp<'static>) -> VpxResult<()> {
    let mut data: *const u8 = pbi.fragments.ptrs[0];
    let data_sz: c_uint = pbi.fragments.sizes[0];
    // SAFETY: data_sz bytes are guaranteed valid past `data` by the caller.
    let data_end: *const u8 = unsafe { data.offset(data_sz as isize) };
    let first_partition_length_in_bytes: c_int;

    let mut corrupt_tokens: c_int = 0;
    let prev_independent_partitions: c_int = pbi.independent_partitions;

    let new_idx = pbi.dec_fb_ref_idx[INTRA_FRAME] as usize;

    /* start with no corruption of current frame */
    pbi.mb.corrupted = 0;
    pbi.common.yv12_fb[new_idx].corrupted = 0;

    if (data_end as isize) - (data as isize) < 3 {
        if pbi.ec_active == 0 {
            // SAFETY: vpx_internal_error writes an error code and returns.
            return unsafe { vpx_internal_error(&mut pbi.common.error, VPX_CODEC_CORRUPT_FRAME) };
        }

        /* Declare the missing frame as an inter frame. */
        pbi.common.frame_type = INTER_FRAME;
        pbi.common.version = 0;
        pbi.common.show_frame = 1;
        first_partition_length_in_bytes = 0;
    } else {
        // Header byte parsing — raw byte reads from `data`.
        let clear: *const u8 = data;

        // SAFETY: clear[0..3] guaranteed valid (we checked data_end-data >= 3).
        let (b0, b1, b2) = unsafe { (*clear.add(0), *clear.add(1), *clear.add(2)) };

        pbi.common.frame_type = if (b0 & 1) == 0 { FrameType::Key } else { FrameType::Inter };
        pbi.common.version = ((b0 >> 1) & 7) as i32;
        pbi.common.show_frame = ((b0 >> 4) & 1) as i32;
        first_partition_length_in_bytes =
            (((b0 as c_int) | ((b1 as c_int) << 8) | ((b2 as c_int) << 16)) >> 5) as c_int;

        if pbi.ec_active == 0 && first_partition_length_in_bytes == 0 {
            return unsafe { vpx_internal_error(&mut pbi.common.error, VPX_CODEC_CORRUPT_FRAME) };
        }

        // SAFETY: advance past the 3 header bytes we just consumed.
        data = unsafe { data.add(3) };

        vp8_setup_version(&mut pbi.common);

        if pbi.common.frame_type == KEY_FRAME {
            if (data_end as isize) - (data as isize) >= 7 {
                // Sync code + width/height live at clear[3..9] (the
                // top-of-function `clear` was *not* advanced when we
                // advanced `data`).
                // SAFETY: at least 10 bytes were copied into clear_buffer
                // (or `data_sz` >= 10); the if-condition above ensures
                // bytes 3..9 are valid past `data`.
                let (s0, s1, s2, w0, w1, h0, h1) = unsafe {
                    (
                        *clear.add(3), *clear.add(4), *clear.add(5),
                        *clear.add(6), *clear.add(7), *clear.add(8), *clear.add(9),
                    )
                };
                if s0 != 0x9d || s1 != 0x01 || s2 != 0x2a {
                    return unsafe {
                        vpx_internal_error(&mut pbi.common.error, VPX_CODEC_UNSUP_BITSTREAM)
                    };
                }

                pbi.common.width = ((w0 as c_int) | ((w1 as c_int) << 8)) & 0x3fff;
                pbi.common.horiz_scale = (w1 >> 6) as c_int;
                pbi.common.height = ((h0 as c_int) | ((h1 as c_int) << 8)) & 0x3fff;
                pbi.common.vert_scale = (h1 >> 6) as c_int;
                // SAFETY: 7 bytes confirmed available.
                data = unsafe { data.add(7) };
            } else if pbi.ec_active == 0 {
                return unsafe {
                    vpx_internal_error(&mut pbi.common.error, VPX_CODEC_CORRUPT_FRAME)
                };
            } else {
                /* Error concealment is active, clear the frame. */
                data = data_end;
            }
        } else {
            // C does `xd->pre = *yv12_fb_new; xd->dst = *yv12_fb_new;` —
            // a full struct copy. Yv12BufferConfig is not Copy (it owns
            // raw plane pointers), so mirror with ptr::copy.
            // SAFETY: yv12_fb[new_idx] is live and distinct from xd.pre/xd.dst.
            unsafe {
                let src: *const Yv12BufferConfig = &pbi.common.yv12_fb[new_idx];
                ptr::copy_nonoverlapping(src, &mut pbi.mb.pre, 1);
                ptr::copy_nonoverlapping(src, &mut pbi.mb.dst, 1);
            }
        }
    }
    if pbi.decoded_key_frame == 0 && pbi.common.frame_type != KEY_FRAME {
        return Err(crate::vpx_api::VPX_CODEC_ERROR);
    }

    if pbi.ec_active == 0
        && ((data_end as isize) - (data as isize)) < first_partition_length_in_bytes as isize
    {
        return unsafe { vpx_internal_error(&mut pbi.common.error, VPX_CODEC_CORRUPT_FRAME) };
    }

    init_frame(pbi);

    let data_remaining = ((data_end as isize) - (data as isize)) as c_uint;
    let start_rc = vp8dx_start_decode(&mut pbi.mbc[8], data, data_remaining);
    if start_rc != 0 {
        return unsafe { vpx_internal_error(&mut pbi.common.error, VPX_CODEC_MEM_ERROR) };
    }

    // Bool-reader-driven header parsing. `bc` borrows pbi.mbc[8]; the
    // writes target disjoint fields (pbi.mb / pbi.common), so they
    // coexist under field-disjoint borrows.
    {
        let bc = &mut pbi.mbc[8];
        if pbi.common.frame_type == KEY_FRAME {
            let _ = vp8_read_bit(bc); // colorspace
            pbi.common.clamp_type = if vp8_read_bit(bc) == 0 {
                ClampType::Required
            } else {
                ClampType::NotRequired
            };
        }

        /* Is segmentation enabled */
        pbi.mb.segmentation_enabled = vp8_read_bit(bc) as u8;

        if pbi.mb.segmentation_enabled != 0 {
            pbi.mb.update_mb_segmentation_map = vp8_read_bit(bc) as u8;
            pbi.mb.update_mb_segmentation_data = vp8_read_bit(bc) as u8;

            if pbi.mb.update_mb_segmentation_data != 0 {
                pbi.mb.mb_segment_abs_delta = vp8_read_bit(bc) as u8;
                pbi.mb.segment_feature_data = Default::default();

                /* For each segmentation feature (Quant and loop filter level) */
                for i in 0..MB_LVL_MAX {
                    for j in 0..MAX_MB_SEGMENTS {
                        if vp8_read_bit(bc) != 0 {
                            let v = vp8_read_literal(bc, VP8_MB_FEATURE_DATA_BITS[i]) as i8;
                            pbi.mb.segment_feature_data[i][j] =
                                if vp8_read_bit(bc) != 0 { -v } else { v };
                        } else {
                            pbi.mb.segment_feature_data[i][j] = 0;
                        }
                    }
                }
            }

            if pbi.mb.update_mb_segmentation_map != 0 {
                pbi.mb.mb_segment_tree_probs = [255; MB_FEATURE_TREE_PROBS];

                /* Read probs used to decode segment id per macroblock. */
                for i in 0..MB_FEATURE_TREE_PROBS {
                    if vp8_read_bit(bc) != 0 {
                        pbi.mb.mb_segment_tree_probs[i] = vp8_read_literal(bc, 8) as Prob;
                    }
                }
            }
        } else {
            pbi.mb.update_mb_segmentation_map = 0;
            pbi.mb.update_mb_segmentation_data = 0;
        }

        /* Read the loop filter level and type */
        pbi.common.filter_type = if vp8_read_bit(bc) == 0 {
            LoopFilterType::Normal
        } else {
            LoopFilterType::Simple
        };
        pbi.common.filter_level = vp8_read_literal(bc, 6);
        pbi.common.sharpness_level = vp8_read_literal(bc, 3);

        /* Read in loop filter deltas applied at the MB level. */
        pbi.mb.mode_ref_lf_delta_update = 0;
        pbi.mb.mode_ref_lf_delta_enabled = vp8_read_bit(bc) as u8;

        if pbi.mb.mode_ref_lf_delta_enabled != 0 {
            pbi.mb.mode_ref_lf_delta_update = vp8_read_bit(bc) as u8;

            if pbi.mb.mode_ref_lf_delta_update != 0 {
                for i in 0..MAX_REF_LF_DELTAS {
                    if vp8_read_bit(bc) != 0 {
                        let v = vp8_read_literal(bc, 6) as i8;
                        pbi.mb.ref_lf_deltas[i] = if vp8_read_bit(bc) != 0 { -v } else { v };
                    }
                }

                for i in 0..MAX_MODE_LF_DELTAS {
                    if vp8_read_bit(bc) != 0 {
                        let v = vp8_read_literal(bc, 6) as i8;
                        pbi.mb.mode_lf_deltas[i] = if vp8_read_bit(bc) != 0 { -v } else { v };
                    }
                }
            }
        }
    }

    // SAFETY: setup_token_decoder takes *mut Vp8dComp; data offset is
    // pre-validated against first_partition_length_in_bytes.
    unsafe { setup_token_decoder(pbi, data.offset(first_partition_length_in_bytes as isize))?; }

    /* Read the default quantizers. */
    {
        // bool-reader literal reads + delta-Q deltas; `bc` borrows
        // pbi.mbc[8], updates target disjoint pbi.common fields.
        let q_update = {
            let bc = &mut pbi.mbc[8];
            pbi.common.base_qindex = vp8_read_literal(bc, 7);
            let mut q_upd: c_int = 0;
            pbi.common.y1dc_delta_q = get_delta_q(bc, pbi.common.y1dc_delta_q, &mut q_upd);
            pbi.common.y2dc_delta_q = get_delta_q(bc, pbi.common.y2dc_delta_q, &mut q_upd);
            pbi.common.y2ac_delta_q = get_delta_q(bc, pbi.common.y2ac_delta_q, &mut q_upd);
            pbi.common.uvdc_delta_q = get_delta_q(bc, pbi.common.uvdc_delta_q, &mut q_upd);
            pbi.common.uvac_delta_q = get_delta_q(bc, pbi.common.uvac_delta_q, &mut q_upd);
            q_upd
        };

        if q_update != 0 {
            vp8cx_init_de_quantizer(&mut pbi.common);
        }

        /* MB level dequantizer setup */
        let common = &pbi.common;
        let mi = common.mi(0, 0);
        vp8_mb_init_dequantizer(
            &common.y1_dequant,
            &common.y2_dequant,
            &common.uv_dequant,
            common.base_qindex,
            &mut pbi.mb,
            mi,
        );
    }

    // More bool-reader-driven header parsing; `bc` borrows pbi.mbc[8].
    {
        let bc = &mut pbi.mbc[8];
        /* Determine if GF/ARF buffers should be updated and how. */
        if pbi.common.frame_type != KEY_FRAME {
            pbi.common.refresh_golden_frame = vp8_read_bit(bc);
            pbi.common.refresh_alt_ref_frame = vp8_read_bit(bc);

            pbi.common.copy_buffer_to_gf = 0;
            if pbi.common.refresh_golden_frame == 0 {
                pbi.common.copy_buffer_to_gf = vp8_read_literal(bc, 2);
            }

            pbi.common.copy_buffer_to_arf = 0;
            if pbi.common.refresh_alt_ref_frame == 0 {
                pbi.common.copy_buffer_to_arf = vp8_read_literal(bc, 2);
            }

            pbi.common.ref_frame_sign_bias[GOLDEN_FRAME] = vp8_read_bit(bc);
            pbi.common.ref_frame_sign_bias[ALTREF_FRAME] = vp8_read_bit(bc);
        }

        pbi.common.refresh_entropy_probs = vp8_read_bit(bc);
        if pbi.common.refresh_entropy_probs == 0 {
            pbi.common.lfc = pbi.common.fc;
        }

        pbi.common.refresh_last_frame = if pbi.common.frame_type == KEY_FRAME {
            1
        } else {
            vp8_read_bit(bc)
        };

        /* Read coef probability tree. */
        pbi.independent_partitions = 1;
        for i in 0..BLOCK_TYPES {
            for j in 0..COEF_BANDS {
                for k in 0..PREV_COEF_CONTEXTS {
                    for l in 0..ENTROPY_NODES {
                        if vp8_read(bc, VP8_COEF_UPDATE_PROBS[i][j][k][l] as c_int) != 0 {
                            pbi.common.fc.coef_probs[i][j][k][l] = vp8_read_literal(bc, 8) as Prob;
                        }
                        if k > 0
                            && pbi.common.fc.coef_probs[i][j][k][l]
                                != pbi.common.fc.coef_probs[i][j][k - 1][l]
                        {
                            pbi.independent_partitions = 0;
                        }
                    }
                }
            }
        }
    }

    /* clear out the coeff buffer */
    pbi.mb.qcoeff.fill(0);

    vp8_decode_mode_mvs(pbi);

    /* Reset above_context for the upcoming row walk. */
    {
        let above = pbi.common.above_context.as_deref_mut().expect("above_context allocated");
        let mb_cols = pbi.common.mb_cols as usize;
        for slot in &mut above[..mb_cols] {
            *slot = EntropyContextPlanes::default();
        }
    }
    pbi.frame_corrupt_residual = 0;

    decode_mb_rows(pbi);
    corrupt_tokens |= pbi.mb.corrupted;

    /* Collect information about decoder corruption. */
    {
        let bc_ref = &pbi.mbc[8];
        let new_corrupt = vp8dx_bool_error(bc_ref) | corrupt_tokens;
        pbi.common.yv12_fb[new_idx].corrupted = new_corrupt;
    }

    if pbi.decoded_key_frame == 0 {
        if pbi.common.frame_type == KEY_FRAME && pbi.common.yv12_fb[new_idx].corrupted == 0 {
            pbi.decoded_key_frame = 1;
        } else {
            return unsafe { vpx_internal_error(&mut pbi.common.error, VPX_CODEC_CORRUPT_FRAME) };
        }
    }

    if pbi.common.refresh_entropy_probs == 0 {
        pbi.common.fc = pbi.common.lfc;
        pbi.independent_partitions = prev_independent_partitions;
    }

    Ok(())
}
