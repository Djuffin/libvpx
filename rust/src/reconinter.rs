//! `vp8/common/reconinter.c` — inter-prediction reconstruction.
//!
//! Literal Rust translation of `vp8/common/reconinter.c`. Materialises
//! the per-macroblock inter predictor samples — either via a plain
//! rectangular copy (integer-MV fast path) or by dispatching one of the
//! installed sub-pel kernels — into the destination YV12 plane. See
//! `documentation/vp8_files/reconinter.md` for the narrative walk-through.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]

use crate::types::{BModeInfo, Blockd, Macroblockd, MbPredictionMode, ModeInfo, Mv, SubpixFn};

// ===========================================================================
// External dependencies (sub-pel filter kernels — defined in `filter.c`
// and dispatched here through `MACROBLOCKD::subpixel_predict*`).
// The function pointers themselves live in [`Macroblockd`], so there is
// nothing extra to import for them. The plain-copy kernels are static
// helpers in this same file (the `_c` variants); when the libvpx RTCD
// dispatcher would select a SIMD specialisation, we always pick the
// portable `_c` body in the Rust port.
// ===========================================================================

// ===========================================================================
// `int_mv` helpers
// ===========================================================================

/// Mirror the C `int_mv` union's `as_int` field for the equality /
/// "any fractional bit set" tests below. The exact bit layout does not
/// matter — only that it is a bijection on `(row, col)` pairs — so use
/// the same packing as `entropymode::mv_as_int`.
#[inline]
fn bmi_as_int(b: &BModeInfo) -> u32 {
    match *b {
        BModeInfo::Mv(m) => ((m.col as u16 as u32) << 16) | (m.row as u16 as u32),
        BModeInfo::Intra(_) => 0,
    }
}

/// Borrow the [`Mv`] payload of a `BModeInfo::Mv` variant. In the C
/// source this is the trivial `bmi.mv.as_mv` projection; we assume the
/// caller only touches `bmi.mv` when the parent MB is in SPLITMV (i.e.
/// the variant is `Mv`), which mirrors the C invariant.
#[inline]
fn bmi_mv_mut(b: &mut BModeInfo) -> &mut Mv {
    if let BModeInfo::Intra(_) = b {
        // The C union doesn't distinguish; in practice the SPLITMV
        // path always installs an `Mv` variant before reading. Coerce
        // by overwriting with a zero MV and re-borrowing.
        *b = BModeInfo::Mv(Mv { row: 0, col: 0 });
    }
    match b {
        BModeInfo::Mv(m) => m,
        BModeInfo::Intra(_) => unreachable!(),
    }
}

/// Read the `Mv` payload (zero if the variant is `Intra`, matching the
/// implicit zero-init the C union has on a fresh `BLOCKD`).
#[inline]
fn bmi_mv(b: &BModeInfo) -> Mv {
    match *b {
        BModeInfo::Mv(m) => m,
        BModeInfo::Intra(_) => Mv { row: 0, col: 0 },
    }
}

// ===========================================================================
// Plain-copy kernels (`vp8_copy_mem*_c`)
// ===========================================================================

/// `vp8_copy_mem16x16_c` — straight 16-byte-wide rectangular copy.
///
/// Source: `vp8/common/reconinter.c:23`.
pub unsafe fn vp8_copy_mem16x16_c(
    mut src: *mut u8,
    src_stride: i32,
    mut dst: *mut u8,
    dst_stride: i32,
) {
    for _ in 0..16 {
        core::ptr::copy_nonoverlapping(src, dst, 16);

        src = src.offset(src_stride as isize);
        dst = dst.offset(dst_stride as isize);
    }
}

/// `vp8_copy_mem8x8_c` — straight 8-byte-wide × 8-row copy (chroma in
/// 16×16-MV mode, or one luma quadrant in coarse SPLITMV).
///
/// Source: `vp8/common/reconinter.c:35`.
pub unsafe fn vp8_copy_mem8x8_c(
    mut src: *mut u8,
    src_stride: i32,
    mut dst: *mut u8,
    dst_stride: i32,
) {
    for _ in 0..8 {
        core::ptr::copy_nonoverlapping(src, dst, 8);

        src = src.offset(src_stride as isize);
        dst = dst.offset(dst_stride as isize);
    }
}

/// `vp8_copy_mem8x4_c` — 8-byte-wide × 4-row copy (fused
/// horizontally-adjacent 4×4 pair under SPLITMV).
///
/// Source: `vp8/common/reconinter.c:47`.
pub unsafe fn vp8_copy_mem8x4_c(
    mut src: *mut u8,
    src_stride: i32,
    mut dst: *mut u8,
    dst_stride: i32,
) {
    for _ in 0..4 {
        core::ptr::copy_nonoverlapping(src, dst, 8);

        src = src.offset(src_stride as isize);
        dst = dst.offset(dst_stride as isize);
    }
}

// Internal wrappers that mirror the RTCD-dispatched symbols without the
// `_c` suffix. The Rust port always uses the portable body.
#[inline]
unsafe fn vp8_copy_mem16x16(src: *mut u8, src_stride: i32, dst: *mut u8, dst_stride: i32) {
    vp8_copy_mem16x16_c(src, src_stride, dst, dst_stride);
}

#[inline]
unsafe fn vp8_copy_mem8x8(src: *mut u8, src_stride: i32, dst: *mut u8, dst_stride: i32) {
    vp8_copy_mem8x8_c(src, src_stride, dst, dst_stride);
}

#[inline]
unsafe fn vp8_copy_mem8x4(src: *mut u8, src_stride: i32, dst: *mut u8, dst_stride: i32) {
    vp8_copy_mem8x4_c(src, src_stride, dst, dst_stride);
}

/// `build_inter_predictors4b` — predict an 8×8 quadrant of luma under a
/// coarse SPLITMV partition.
///
/// Source: `vp8/common/reconinter.c:82`.
unsafe fn build_inter_predictors4b(
    subpixel_predict8x8: SubpixFn,
    d: &Blockd,
    dst: *mut u8,
    dst_stride: i32,
    base_pre: *mut u8,
    pre_stride: i32,
) {
    let mv = bmi_mv(&d.bmi);
    let ptr: *mut u8 = base_pre
        .offset(d.offset as isize)
        .offset(((mv.row as i32 >> 3) * pre_stride) as isize)
        .offset((mv.col as i32 >> 3) as isize);

    if (mv.row as i32 & 7) != 0 || (mv.col as i32 & 7) != 0 {
        subpixel_predict8x8(
            ptr,
            pre_stride,
            mv.col as i32 & 7,
            mv.row as i32 & 7,
            dst,
            dst_stride,
        );
    } else {
        vp8_copy_mem8x8(ptr, pre_stride, dst, dst_stride);
    }
}

/// `build_inter_predictors2b` — predict an 8×4 fused pair of adjacent
/// 4×4 sub-blocks under SPLITMV.
///
/// Source: `vp8/common/reconinter.c:97`.
unsafe fn build_inter_predictors2b(
    subpixel_predict8x4: SubpixFn,
    d: &Blockd,
    dst: *mut u8,
    dst_stride: i32,
    base_pre: *mut u8,
    pre_stride: i32,
) {
    let mv = bmi_mv(&d.bmi);
    let ptr: *mut u8 = base_pre
        .offset(d.offset as isize)
        .offset(((mv.row as i32 >> 3) * pre_stride) as isize)
        .offset((mv.col as i32 >> 3) as isize);

    if (mv.row as i32 & 7) != 0 || (mv.col as i32 & 7) != 0 {
        subpixel_predict8x4(
            ptr,
            pre_stride,
            mv.col as i32 & 7,
            mv.row as i32 & 7,
            dst,
            dst_stride,
        );
    } else {
        vp8_copy_mem8x4(ptr, pre_stride, dst, dst_stride);
    }
}

/// `build_inter_predictors_b` (static lowercase) — true 4×4 predict
/// into the destination YV12 plane.
///
/// Source: `vp8/common/reconinter.c:112`.
unsafe fn build_inter_predictors_b(
    d: &Blockd,
    mut dst: *mut u8,
    dst_stride: i32,
    base_pre: *mut u8,
    pre_stride: i32,
    sppf: SubpixFn,
) {
    let mut ptr: *mut u8;
    let mv = bmi_mv(&d.bmi);
    ptr = base_pre
        .offset(d.offset as isize)
        .offset(((mv.row as i32 >> 3) * pre_stride) as isize)
        .offset((mv.col as i32 >> 3) as isize);

    if (mv.row as i32 & 7) != 0 || (mv.col as i32 & 7) != 0 {
        sppf(
            ptr,
            pre_stride,
            mv.col as i32 & 7,
            mv.row as i32 & 7,
            dst,
            dst_stride,
        );
    } else {
        for _ in 0..4 {
            *dst.offset(0) = *ptr.offset(0);
            *dst.offset(1) = *ptr.offset(1);
            *dst.offset(2) = *ptr.offset(2);
            *dst.offset(3) = *ptr.offset(3);
            dst = dst.offset(dst_stride as isize);
            ptr = ptr.offset(pre_stride as isize);
        }
    }
}

// ===========================================================================
// MV clamping at the picture border
// ===========================================================================

/// `clamp_mv_to_umv_border` — luma MV clamp.
///
/// Source: `vp8/common/reconinter.c:257`.
fn clamp_mv_to_umv_border(
    mv: &mut Mv,
    mb_to_left_edge: i32,
    mb_to_right_edge: i32,
    mb_to_top_edge: i32,
    mb_to_bottom_edge: i32,
) {
    /* If the MV points so far into the UMV border that no visible pixels
     * are used for reconstruction, the subpel part of the MV can be
     * discarded and the MV limited to 16 pixels with equivalent results.
     *
     * This limit kicks in at 19 pixels for the top and left edges, for
     * the 16 pixels plus 3 taps right of the central pixel when subpel
     * filtering. The bottom and right edges use 16 pixels plus 2 pixels
     * left of the central pixel when filtering.
     */
    if (mv.col as i32) < (mb_to_left_edge - (19 << 3)) {
        mv.col = (mb_to_left_edge - (16 << 3)) as i16;
    } else if (mv.col as i32) > mb_to_right_edge + (18 << 3) {
        mv.col = (mb_to_right_edge + (16 << 3)) as i16;
    }

    if (mv.row as i32) < (mb_to_top_edge - (19 << 3)) {
        mv.row = (mb_to_top_edge - (16 << 3)) as i16;
    } else if (mv.row as i32) > mb_to_bottom_edge + (18 << 3) {
        mv.row = (mb_to_bottom_edge + (16 << 3)) as i16;
    }
}

/// `clamp_uvmv_to_umv_border` — chroma MV clamp (works on an
/// already-derived chroma MV; thresholds still expressed in luma units).
///
/// Source: `vp8/common/reconinter.c:281`.
fn clamp_uvmv_to_umv_border(
    mv: &mut Mv,
    mb_to_left_edge: i32,
    mb_to_right_edge: i32,
    mb_to_top_edge: i32,
    mb_to_bottom_edge: i32,
) {
    mv.col = if 2 * (mv.col as i32) < (mb_to_left_edge - (19 << 3)) {
        ((mb_to_left_edge - (16 << 3)) >> 1) as i16
    } else {
        mv.col
    };
    mv.col = if 2 * (mv.col as i32) > mb_to_right_edge + (18 << 3) {
        ((mb_to_right_edge + (16 << 3)) >> 1) as i16
    } else {
        mv.col
    };

    mv.row = if 2 * (mv.row as i32) < (mb_to_top_edge - (19 << 3)) {
        ((mb_to_top_edge - (16 << 3)) >> 1) as i16
    } else {
        mv.row
    };
    mv.row = if 2 * (mv.row as i32) > mb_to_bottom_edge + (18 << 3) {
        ((mb_to_bottom_edge + (16 << 3)) >> 1) as i16
    } else {
        mv.row
    };
}

// ===========================================================================
// 16×16 main entry — `vp8_build_inter16x16_predictors_mb`
// ===========================================================================

/// `vp8_build_inter16x16_predictors_mb` — the production decoder path
/// for non-SPLITMV inter macroblocks.
///
/// Source: `vp8/common/reconinter.c:297`.
pub unsafe fn vp8_build_inter16x16_predictors_mb(
    x: &Macroblockd,
    mi: &ModeInfo,
    dst_y: *mut u8,
    dst_u: *mut u8,
    dst_v: *mut u8,
    dst_ystride: i32,
    dst_uvstride: i32,
) {
    let offset: i32;
    let ptr: *mut u8;
    let uptr: *mut u8;
    let vptr: *mut u8;

    // `int_mv` modelled as a plain `Mv` plus a packed-int snapshot for
    // the `as_int & 0x00070007` test.
    let mut _16x16mv: Mv;

    let ptr_base: *mut u8 = x.pre.y_buffer;
    let mut pre_stride: i32 = x.pre.y_stride;

    _16x16mv = mi.mbmi.mv;

    if mi.mbmi.need_to_clamp_mvs {
        clamp_mv_to_umv_border(
            &mut _16x16mv,
            x.mb_to_left_edge,
            x.mb_to_right_edge,
            x.mb_to_top_edge,
            x.mb_to_bottom_edge,
        );
    }

    ptr = ptr_base
        .offset(((_16x16mv.row as i32 >> 3) * pre_stride) as isize)
        .offset((_16x16mv.col as i32 >> 3) as isize);

    // C tests `_16x16mv.as_int & 0x00070007`; mirror by packing into the
    // same layout `entropymode::mv_as_int` uses (col<<16 | row).
    let mut as_int: u32 = ((_16x16mv.col as u16 as u32) << 16) | (_16x16mv.row as u16 as u32);

    if (as_int & 0x0007_0007) != 0 {
        (x.subpixel_predict16x16)(
            ptr,
            pre_stride,
            _16x16mv.col as i32 & 7,
            _16x16mv.row as i32 & 7,
            dst_y,
            dst_ystride,
        );
    } else {
        vp8_copy_mem16x16(ptr, pre_stride, dst_y, dst_ystride);
    }

    /* calc uv motion vectors */
    let row_i: i32 = _16x16mv.row as i32
        + (1 | ((_16x16mv.row as i32) >> (core::mem::size_of::<i32>() as i32 * 8 - 1)));
    let col_i: i32 = _16x16mv.col as i32
        + (1 | ((_16x16mv.col as i32) >> (core::mem::size_of::<i32>() as i32 * 8 - 1)));
    let row_i: i32 = (row_i / 2) & x.fullpixel_mask;
    let col_i: i32 = (col_i / 2) & x.fullpixel_mask;
    _16x16mv.row = row_i as i16;
    _16x16mv.col = col_i as i16;

    if 2 * (_16x16mv.col as i32) < (x.mb_to_left_edge - (19 << 3))
        || 2 * (_16x16mv.col as i32) > x.mb_to_right_edge + (18 << 3)
        || 2 * (_16x16mv.row as i32) < (x.mb_to_top_edge - (19 << 3))
        || 2 * (_16x16mv.row as i32) > x.mb_to_bottom_edge + (18 << 3)
    {
        return;
    }

    pre_stride >>= 1;
    offset = (_16x16mv.row as i32 >> 3) * pre_stride + (_16x16mv.col as i32 >> 3);
    uptr = x.pre.u_buffer.offset(offset as isize);
    vptr = x.pre.v_buffer.offset(offset as isize);

    // Recompute `as_int` after the chroma-derivation mutated row/col.
    as_int = ((_16x16mv.col as u16 as u32) << 16) | (_16x16mv.row as u16 as u32);

    if (as_int & 0x0007_0007) != 0 {
        (x.subpixel_predict8x8)(
            uptr,
            pre_stride,
            _16x16mv.col as i32 & 7,
            _16x16mv.row as i32 & 7,
            dst_u,
            dst_uvstride,
        );
        (x.subpixel_predict8x8)(
            vptr,
            pre_stride,
            _16x16mv.col as i32 & 7,
            _16x16mv.row as i32 & 7,
            dst_v,
            dst_uvstride,
        );
    } else {
        vp8_copy_mem8x8(uptr, pre_stride, dst_u, dst_uvstride);
        vp8_copy_mem8x8(vptr, pre_stride, dst_v, dst_uvstride);
    }
}

// ===========================================================================
// SPLITMV path
// ===========================================================================

/// `build_inter4x4_predictors_mb` — drive sub-block predictors for a
/// SPLITMV macroblock into the destination YV12.
///
/// Source: `vp8/common/reconinter.c:359`.
unsafe fn build_inter4x4_predictors_mb(x: &mut Macroblockd, mi: &ModeInfo) {
    // Snapshot fn pointers + plane bases up front. After this, x is held
    // for `block[]` access only, avoiding borrow conflicts with the
    // raw pixel pointers we pass to sub-calls.
    let sp8x8 = x.subpixel_predict8x8;
    let sp8x4 = x.subpixel_predict8x4;
    let sp4x4 = x.subpixel_predict;
    let dst_y_buffer: *mut u8 = x.dst.y_buffer;
    let pre_y_buffer: *mut u8 = x.pre.y_buffer;
    let dst_u_buffer: *mut u8 = x.dst.u_buffer;
    let pre_u_buffer: *mut u8 = x.pre.u_buffer;
    let dst_v_buffer: *mut u8 = x.dst.v_buffer;
    let pre_v_buffer: *mut u8 = x.pre.v_buffer;
    let y_stride: i32 = x.dst.y_stride;
    let uv_stride: i32 = x.dst.uv_stride;
    let l = x.mb_to_left_edge;
    let r = x.mb_to_right_edge;
    let t = x.mb_to_top_edge;
    let bot = x.mb_to_bottom_edge;

    if mi.mbmi.partitioning < 3 {
        x.block[0].bmi = mi.bmi[0];
        x.block[2].bmi = mi.bmi[2];
        x.block[8].bmi = mi.bmi[8];
        x.block[10].bmi = mi.bmi[10];
        if mi.mbmi.need_to_clamp_mvs {
            clamp_mv_to_umv_border(&mut *bmi_mv_mut(&mut x.block[0].bmi), l, r, t, bot);
            clamp_mv_to_umv_border(&mut *bmi_mv_mut(&mut x.block[2].bmi), l, r, t, bot);
            clamp_mv_to_umv_border(&mut *bmi_mv_mut(&mut x.block[8].bmi), l, r, t, bot);
            clamp_mv_to_umv_border(&mut *bmi_mv_mut(&mut x.block[10].bmi), l, r, t, bot);
        }

        for idx in [0usize, 2, 8, 10] {
            let b = &x.block[idx];
            build_inter_predictors4b(
                sp8x8,
                b,
                dst_y_buffer.offset(b.offset as isize),
                y_stride,
                pre_y_buffer,
                y_stride,
            );
        }
    } else {
        for i in (0..16usize).step_by(2) {
            x.block[i].bmi = mi.bmi[i];
            x.block[i + 1].bmi = mi.bmi[i + 1];
            if mi.mbmi.need_to_clamp_mvs {
                clamp_mv_to_umv_border(&mut *bmi_mv_mut(&mut x.block[i].bmi), l, r, t, bot);
                clamp_mv_to_umv_border(&mut *bmi_mv_mut(&mut x.block[i + 1].bmi), l, r, t, bot);
            }

            let d0 = &x.block[i];
            let d1 = &x.block[i + 1];
            if bmi_as_int(&d0.bmi) == bmi_as_int(&d1.bmi) {
                build_inter_predictors2b(
                    sp8x4,
                    d0,
                    dst_y_buffer.offset(d0.offset as isize),
                    y_stride,
                    pre_y_buffer,
                    y_stride,
                );
            } else {
                build_inter_predictors_b(
                    d0,
                    dst_y_buffer.offset(d0.offset as isize),
                    y_stride,
                    pre_y_buffer,
                    y_stride,
                    sp4x4,
                );
                build_inter_predictors_b(
                    d1,
                    dst_y_buffer.offset(d1.offset as isize),
                    y_stride,
                    pre_y_buffer,
                    y_stride,
                    sp4x4,
                );
            }
        }
    }
    for i in (16..20usize).step_by(2) {
        let d0 = &x.block[i];
        let d1 = &x.block[i + 1];

        /* Note: uv mvs already clamped in build_4x4uvmvs() */

        if bmi_as_int(&d0.bmi) == bmi_as_int(&d1.bmi) {
            build_inter_predictors2b(
                sp8x4,
                d0,
                dst_u_buffer.offset(d0.offset as isize),
                uv_stride,
                pre_u_buffer,
                uv_stride,
            );
        } else {
            build_inter_predictors_b(
                d0,
                dst_u_buffer.offset(d0.offset as isize),
                uv_stride,
                pre_u_buffer,
                uv_stride,
                sp4x4,
            );
            build_inter_predictors_b(
                d1,
                dst_u_buffer.offset(d1.offset as isize),
                uv_stride,
                pre_u_buffer,
                uv_stride,
                sp4x4,
            );
        }
    }

    for i in (20..24usize).step_by(2) {
        let d0 = &x.block[i];
        let d1 = &x.block[i + 1];

        /* Note: uv mvs already clamped in build_4x4uvmvs() */

        if bmi_as_int(&d0.bmi) == bmi_as_int(&d1.bmi) {
            build_inter_predictors2b(
                sp8x4,
                d0,
                dst_v_buffer.offset(d0.offset as isize),
                uv_stride,
                pre_v_buffer,
                uv_stride,
            );
        } else {
            build_inter_predictors_b(
                d0,
                dst_v_buffer.offset(d0.offset as isize),
                uv_stride,
                pre_v_buffer,
                uv_stride,
                sp4x4,
            );
            build_inter_predictors_b(
                d1,
                dst_v_buffer.offset(d1.offset as isize),
                uv_stride,
                pre_v_buffer,
                uv_stride,
                sp4x4,
            );
        }
    }
}

/// `build_4x4uvmvs` — derive the four chroma 4×4 MVs from the sixteen
/// luma 4×4 MVs for a SPLITMV macroblock, with sign-aware-rounded
/// averaging and full-pixel-mode quantisation.
///
/// Source: `vp8/common/reconinter.c:456`.
fn build_4x4uvmvs(x: &mut Macroblockd, mi: &ModeInfo) {
    let fullpixel_mask = x.fullpixel_mask;
    let l = x.mb_to_left_edge;
    let r = x.mb_to_right_edge;
    let t = x.mb_to_top_edge;
    let bot = x.mb_to_bottom_edge;

    for i in 0..2i32 {
        for j in 0..2i32 {
            let yoffset: i32 = i * 8 + j * 2;
            let uoffset: i32 = 16 + i * 2 + j;
            let voffset: i32 = 20 + i * 2 + j;

            let mut temp: i32;

            // `mode_info_context->bmi[k].mv.as_mv.row` -> via bmi_mv helper.
            temp = bmi_mv(&mi.bmi[(yoffset + 0) as usize]).row as i32
                + bmi_mv(&mi.bmi[(yoffset + 1) as usize]).row as i32
                + bmi_mv(&mi.bmi[(yoffset + 4) as usize]).row as i32
                + bmi_mv(&mi.bmi[(yoffset + 5) as usize]).row as i32;

            temp += 4 + ((temp >> (core::mem::size_of::<i32>() as i32 * 8 - 1)) * 8);

            let new_row = ((temp / 8) & fullpixel_mask) as i16;

            temp = bmi_mv(&mi.bmi[(yoffset + 0) as usize]).col as i32
                + bmi_mv(&mi.bmi[(yoffset + 1) as usize]).col as i32
                + bmi_mv(&mi.bmi[(yoffset + 4) as usize]).col as i32
                + bmi_mv(&mi.bmi[(yoffset + 5) as usize]).col as i32;

            temp += 4 + ((temp >> (core::mem::size_of::<i32>() as i32 * 8 - 1)) * 8);

            let new_col = ((temp / 8) & fullpixel_mask) as i16;

            x.block[uoffset as usize].bmi = BModeInfo::Mv(Mv {
                row: new_row,
                col: new_col,
            });

            if mi.mbmi.need_to_clamp_mvs {
                let mv_ref: &mut Mv = bmi_mv_mut(&mut x.block[uoffset as usize].bmi);
                clamp_uvmv_to_umv_border(mv_ref, l, r, t, bot);
            }

            let u_mv = bmi_mv(&x.block[uoffset as usize].bmi);
            x.block[voffset as usize].bmi = BModeInfo::Mv(u_mv);
        }
    }
}

/// `vp8_build_inter_predictors_mb` — top-level dispatcher invoked once
/// per inter macroblock from `decodeframe.c`.
///
/// Source: `vp8/common/reconinter.c:494`.
pub fn vp8_build_inter_predictors_mb(xd: &mut Macroblockd, mi: &ModeInfo) {
    if mi.mbmi.mode != MbPredictionMode::SplitMv {
        let dst_y = xd.dst.y_buffer;
        let dst_u = xd.dst.u_buffer;
        let dst_v = xd.dst.v_buffer;
        let y_stride = xd.dst.y_stride;
        let uv_stride = xd.dst.uv_stride;
        // SAFETY: dst plane pointers come from xd.dst (live Yv12 frame);
        // the 16x16 predictor reads ref-frame planes within their bounds.
        unsafe {
            vp8_build_inter16x16_predictors_mb(&*xd, mi, dst_y, dst_u, dst_v, y_stride, uv_stride);
        }
    } else {
        build_4x4uvmvs(xd, mi);
        // SAFETY: SPLITMV 4x4 sub-block dispatcher walks per-sub-block
        // motion vectors into ref-frame pixel ranges; xd carries the
        // dst/pre plane state.
        unsafe { build_inter4x4_predictors_mb(xd, mi); }
    }
}
