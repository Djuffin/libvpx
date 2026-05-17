//! Frame-border extension (`vp8/common/extend.c`).
//!
//! Literal Rust transliteration of libvpx's `extend.c`. Three public
//! entry points sit on top of a single file-local helper
//! `copy_and_extend_plane`. The decoder only ever calls
//! [`vp8_extend_mb_row`]; the other two are encoder-side conveniences.

#![allow(clippy::too_many_arguments)]

use core::ptr::{copy_nonoverlapping, write_bytes};

use crate::types::Yv12BufferConfig;

/// Static helper `copy_and_extend_plane` (vp8/common/extend.c:14).
///
/// Copies an `h`-row by `w`-column rectangle from `s` (pitch `sp`) into
/// `d` (pitch `dp`), then replicates the picture's edges outward into
/// the configured `et`/`el`/`eb`/`er` border widths. `interleave_step`
/// is `2` for NV12-interleaved chroma reads, `1` otherwise.
unsafe fn copy_and_extend_plane(
    s: *mut u8,       /* source */
    sp: i32,          /* source pitch */
    d: *mut u8,       /* destination */
    dp: i32,          /* destination pitch */
    h: i32,           /* height */
    w: i32,           /* width */
    et: i32,          /* extend top border */
    el: i32,          /* extend left border */
    eb: i32,          /* extend bottom border */
    er: i32,          /* extend right border */
    mut interleave_step: i32, /* step between pixels of the current plane */
) {
    unsafe {
        let mut src_ptr1: *mut u8;
        let mut src_ptr2: *mut u8;
        let mut dest_ptr1: *mut u8;
        let mut dest_ptr2: *mut u8;
        let linesize: i32;

        if interleave_step < 1 {
            interleave_step = 1;
        }

        /* copy the left and right most columns out */
        src_ptr1 = s;
        src_ptr2 = s.offset(((w - 1) * interleave_step) as isize);
        dest_ptr1 = d.offset(-(el as isize));
        dest_ptr2 = d.offset(w as isize);

        let mut i = 0;
        while i < h {
            write_bytes(dest_ptr1, *src_ptr1.offset(0), el as usize);
            if interleave_step == 1 {
                copy_nonoverlapping(src_ptr1, dest_ptr1.offset(el as isize), w as usize);
            } else {
                let mut j = 0;
                while j < w {
                    *dest_ptr1.offset((el + j) as isize) =
                        *src_ptr1.offset((interleave_step * j) as isize);
                    j += 1;
                }
            }
            write_bytes(dest_ptr2, *src_ptr2.offset(0), er as usize);
            src_ptr1 = src_ptr1.offset(sp as isize);
            src_ptr2 = src_ptr2.offset(sp as isize);
            dest_ptr1 = dest_ptr1.offset(dp as isize);
            dest_ptr2 = dest_ptr2.offset(dp as isize);
            i += 1;
        }

        /* Now copy the top and bottom lines into each line of the respective
         * borders
         */
        src_ptr1 = d.offset(-(el as isize));
        src_ptr2 = d.offset((dp * (h - 1)) as isize).offset(-(el as isize));
        dest_ptr1 = d.offset((dp * (-et)) as isize).offset(-(el as isize));
        dest_ptr2 = d.offset((dp * h) as isize).offset(-(el as isize));
        linesize = el + er + w;

        let mut i = 0;
        while i < et {
            copy_nonoverlapping(src_ptr1, dest_ptr1, linesize as usize);
            dest_ptr1 = dest_ptr1.offset(dp as isize);
            i += 1;
        }

        let mut i = 0;
        while i < eb {
            copy_nonoverlapping(src_ptr2, dest_ptr2, linesize as usize);
            dest_ptr2 = dest_ptr2.offset(dp as isize);
            i += 1;
        }
    }
}

/// `vp8_copy_and_extend_frame` (vp8/common/extend.c:75).
///
/// Copies the entire `src` picture into `dst` and fills `dst`'s border.
/// Encoder-side path; the decoder uses `vp8_yv12_extend_frame_borders_c`
/// instead.
pub unsafe fn vp8_copy_and_extend_frame(
    src: *mut Yv12BufferConfig,
    dst: *mut Yv12BufferConfig,
) {
    unsafe {
        let mut et: i32 = (*dst).border;
        let mut el: i32 = (*dst).border;
        let mut eb: i32 = (*dst).border + (*dst).y_height - (*src).y_height;
        let mut er: i32 = (*dst).border + (*dst).y_width - (*src).y_width;

        // detect nv12 colorspace
        let chroma_step: i32 = if (*src).v_buffer.offset_from((*src).u_buffer) == 1 {
            2
        } else {
            1
        };

        copy_and_extend_plane(
            (*src).y_buffer,
            (*src).y_stride,
            (*dst).y_buffer,
            (*dst).y_stride,
            (*src).y_height,
            (*src).y_width,
            et,
            el,
            eb,
            er,
            1,
        );

        et = (*dst).border >> 1;
        el = (*dst).border >> 1;
        eb = ((*dst).border >> 1) + (*dst).uv_height - (*src).uv_height;
        er = ((*dst).border >> 1) + (*dst).uv_width - (*src).uv_width;

        copy_and_extend_plane(
            (*src).u_buffer,
            (*src).uv_stride,
            (*dst).u_buffer,
            (*dst).uv_stride,
            (*src).uv_height,
            (*src).uv_width,
            et,
            el,
            eb,
            er,
            chroma_step,
        );

        copy_and_extend_plane(
            (*src).v_buffer,
            (*src).uv_stride,
            (*dst).v_buffer,
            (*dst).uv_stride,
            (*src).uv_height,
            (*src).uv_width,
            et,
            el,
            eb,
            er,
            chroma_step,
        );
    }
}

/// `vp8_copy_and_extend_frame_with_rect` (vp8/common/extend.c:103).
///
/// Copies a sub-rectangle of `src` into `dst` and extends only the
/// borders on the sides of `dst` that the sub-rectangle actually
/// touches. Encoder-side only.
pub unsafe fn vp8_copy_and_extend_frame_with_rect(
    src: *mut Yv12BufferConfig,
    dst: *mut Yv12BufferConfig,
    srcy: i32,
    srcx: i32,
    mut srch: i32,
    mut srcw: i32,
) {
    unsafe {
        let mut et: i32 = (*dst).border;
        let mut el: i32 = (*dst).border;
        let mut eb: i32 = (*dst).border + (*dst).y_height - (*src).y_height;
        let mut er: i32 = (*dst).border + (*dst).y_width - (*src).y_width;
        let src_y_offset: i32 = srcy * (*src).y_stride + srcx;
        let dst_y_offset: i32 = srcy * (*dst).y_stride + srcx;
        let src_uv_offset: i32 = ((srcy * (*src).uv_stride) >> 1) + (srcx >> 1);
        let dst_uv_offset: i32 = ((srcy * (*dst).uv_stride) >> 1) + (srcx >> 1);
        // detect nv12 colorspace
        let chroma_step: i32 = if (*src).v_buffer.offset_from((*src).u_buffer) == 1 {
            2
        } else {
            1
        };

        /* If the side is not touching the bounder then don't extend. */
        if srcy != 0 {
            et = 0;
        }
        if srcx != 0 {
            el = 0;
        }
        if srcy + srch != (*src).y_height {
            eb = 0;
        }
        if srcx + srcw != (*src).y_width {
            er = 0;
        }

        copy_and_extend_plane(
            (*src).y_buffer.offset(src_y_offset as isize),
            (*src).y_stride,
            (*dst).y_buffer.offset(dst_y_offset as isize),
            (*dst).y_stride,
            srch,
            srcw,
            et,
            el,
            eb,
            er,
            1,
        );

        et = (et + 1) >> 1;
        el = (el + 1) >> 1;
        eb = (eb + 1) >> 1;
        er = (er + 1) >> 1;
        srch = (srch + 1) >> 1;
        srcw = (srcw + 1) >> 1;

        copy_and_extend_plane(
            (*src).u_buffer.offset(src_uv_offset as isize),
            (*src).uv_stride,
            (*dst).u_buffer.offset(dst_uv_offset as isize),
            (*dst).uv_stride,
            srch,
            srcw,
            et,
            el,
            eb,
            er,
            chroma_step,
        );

        copy_and_extend_plane(
            (*src).v_buffer.offset(src_uv_offset as isize),
            (*src).uv_stride,
            (*dst).v_buffer.offset(dst_uv_offset as isize),
            (*dst).uv_stride,
            srch,
            srcw,
            et,
            el,
            eb,
            er,
            chroma_step,
        );
    }
}

/// `vp8_extend_mb_row` (vp8/common/extend.c:144).
///
/// Replicates one column of edge samples into the right side of the
/// most-recently-decoded macroblock row so that the next MB row's
/// intra-prediction can read 4 pixels to the right of the current
/// column. Called from `decode_mb_rows` at the end of every MB row.
///
/// Note the extension is only for the last row, for intra prediction
/// purpose.
pub unsafe fn vp8_extend_mb_row(
    ybf: *mut Yv12BufferConfig,
    mut y_ptr: *mut u8,
    mut u_ptr: *mut u8,
    mut v_ptr: *mut u8,
) {
    unsafe {
        y_ptr = y_ptr.offset(((*ybf).y_stride * 14) as isize);
        u_ptr = u_ptr.offset(((*ybf).uv_stride * 6) as isize);
        v_ptr = v_ptr.offset(((*ybf).uv_stride * 6) as isize);

        let mut i = 0;
        while i < 4 {
            *y_ptr.offset(i as isize) = *y_ptr.offset(-1);
            *u_ptr.offset(i as isize) = *u_ptr.offset(-1);
            *v_ptr.offset(i as isize) = *v_ptr.offset(-1);
            i += 1;
        }

        y_ptr = y_ptr.offset((*ybf).y_stride as isize);
        u_ptr = u_ptr.offset((*ybf).uv_stride as isize);
        v_ptr = v_ptr.offset((*ybf).uv_stride as isize);

        let mut i = 0;
        while i < 4 {
            *y_ptr.offset(i as isize) = *y_ptr.offset(-1);
            *u_ptr.offset(i as isize) = *u_ptr.offset(-1);
            *v_ptr.offset(i as isize) = *v_ptr.offset(-1);
            i += 1;
        }
    }
}
