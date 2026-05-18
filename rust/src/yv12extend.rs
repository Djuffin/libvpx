//! YV12-aware border-extension and whole-frame-copy.
//!
//! Literal Rust port of `vpx_scale/generic/yv12extend.c`. Mirrors C
//! control flow exactly. The VP9/HBD (`CONFIG_VP9`,
//! `CONFIG_VP9_HIGHBITDEPTH`) blocks are omitted — VP8-only build.
//!
//! See `documentation/vp8_files/yv12extend.md` and
//! `documentation/vp8_technical_overview.md` §10.3.

#![allow(non_snake_case)]

use core::ptr;

use crate::types::Yv12BufferConfig;

/// `extend_plane` (yv12extend.c:22) — replicate the four edges of a
/// `width × height` sub-region of one plane outward into the
/// surrounding border bytes. `src` points at the visible top-left
/// pixel (i.e. `buffer_alloc + border * stride + border`); the border
/// is addressed by negative / past-the-end offsets relative to `src`.
///
/// Two-phase: column fill first (so the corner squares pick up the
/// correct edge value), then row copy.
unsafe fn extend_plane(
    src: *mut u8,
    src_stride: i32,
    width: i32,
    height: i32,
    extend_top: i32,
    extend_left: i32,
    extend_bottom: i32,
    extend_right: i32,
) {
    let linesize = extend_left + extend_right + width;

    /* copy the left and right most columns out */
    let mut src_ptr1: *mut u8 = src;
    let mut src_ptr2: *mut u8 = src.offset((width - 1) as isize);
    let mut dst_ptr1: *mut u8 = src.offset(-(extend_left as isize));
    let mut dst_ptr2: *mut u8 = src.offset(width as isize);

    for _ in 0..height {
        ptr::write_bytes(dst_ptr1, *src_ptr1, extend_left as usize);
        ptr::write_bytes(dst_ptr2, *src_ptr2, extend_right as usize);
        src_ptr1 = src_ptr1.offset(src_stride as isize);
        src_ptr2 = src_ptr2.offset(src_stride as isize);
        dst_ptr1 = dst_ptr1.offset(src_stride as isize);
        dst_ptr2 = dst_ptr2.offset(src_stride as isize);
    }

    /* Now copy the top and bottom lines into each line of the respective
     * borders
     */
    src_ptr1 = src.offset(-(extend_left as isize));
    src_ptr2 = src
        .offset((src_stride * (height - 1)) as isize)
        .offset(-(extend_left as isize));
    dst_ptr1 = src
        .offset((src_stride * -extend_top) as isize)
        .offset(-(extend_left as isize));
    dst_ptr2 = src
        .offset((src_stride * height) as isize)
        .offset(-(extend_left as isize));

    for _ in 0..extend_top {
        ptr::copy_nonoverlapping(src_ptr1, dst_ptr1, linesize as usize);
        dst_ptr1 = dst_ptr1.offset(src_stride as isize);
    }

    for _ in 0..extend_bottom {
        ptr::copy_nonoverlapping(src_ptr2, dst_ptr2, linesize as usize);
        dst_ptr2 = dst_ptr2.offset(src_stride as isize);
    }
}

/// `vp8_yv12_extend_frame_borders_c` (yv12extend.c:105). VP8 public
/// entry point: extends Y, U and V of one `YV12_BUFFER_CONFIG` outward
/// by `border` and `border / 2` pixels respectively.
///
/// # Safety
/// `ybf` must point to a valid, fully-initialised `Yv12BufferConfig`
/// whose plane allocations include the surrounding border pixels (as
/// guaranteed by libvpx's frame-buffer allocator).
pub unsafe fn vp8_yv12_extend_frame_borders_c(ybf: *mut Yv12BufferConfig) {
    let uv_border = (*ybf).border / 2;

    assert!((*ybf).border % 2 == 0);
    assert!((*ybf).y_height - (*ybf).y_crop_height < 16);
    assert!((*ybf).y_width - (*ybf).y_crop_width < 16);
    assert!((*ybf).y_height - (*ybf).y_crop_height >= 0);
    assert!((*ybf).y_width - (*ybf).y_crop_width >= 0);

    extend_plane(
        (*ybf).y_buffer,
        (*ybf).y_stride,
        (*ybf).y_crop_width,
        (*ybf).y_crop_height,
        (*ybf).border,
        (*ybf).border,
        (*ybf).border + (*ybf).y_height - (*ybf).y_crop_height,
        (*ybf).border + (*ybf).y_width - (*ybf).y_crop_width,
    );

    extend_plane(
        (*ybf).u_buffer,
        (*ybf).uv_stride,
        (*ybf).uv_crop_width,
        (*ybf).uv_crop_height,
        uv_border,
        uv_border,
        uv_border + (*ybf).uv_height - (*ybf).uv_crop_height,
        uv_border + (*ybf).uv_width - (*ybf).uv_crop_width,
    );

    extend_plane(
        (*ybf).v_buffer,
        (*ybf).uv_stride,
        (*ybf).uv_crop_width,
        (*ybf).uv_crop_height,
        uv_border,
        uv_border,
        uv_border + (*ybf).uv_height - (*ybf).uv_crop_height,
        uv_border + (*ybf).uv_width - (*ybf).uv_crop_width,
    );
}

/// `vp8_yv12_copy_frame_c` (yv12extend.c:193). Whole-frame deep copy
/// of the three planes from `src_ybc` into `dst_ybc`, followed by
/// border extension of the destination.
///
/// # Safety
/// Both pointers must reference valid `Yv12BufferConfig`s with
/// matching `y_width`/`y_height` / `uv_width`/`uv_height` and properly
/// allocated planes (including border slack).
pub unsafe fn vp8_yv12_copy_frame_c(
    src_ybc: *const Yv12BufferConfig,
    dst_ybc: *mut Yv12BufferConfig,
) {
    let mut src: *const u8 = (*src_ybc).y_buffer;
    let mut dst: *mut u8 = (*dst_ybc).y_buffer;

    // #if 0 block (disabled assertions) elided.

    for _ in 0..(*src_ybc).y_height {
        ptr::copy_nonoverlapping(src, dst, (*src_ybc).y_width as usize);
        src = src.offset((*src_ybc).y_stride as isize);
        dst = dst.offset((*dst_ybc).y_stride as isize);
    }

    src = (*src_ybc).u_buffer;
    dst = (*dst_ybc).u_buffer;

    for _ in 0..(*src_ybc).uv_height {
        ptr::copy_nonoverlapping(src, dst, (*src_ybc).uv_width as usize);
        src = src.offset((*src_ybc).uv_stride as isize);
        dst = dst.offset((*dst_ybc).uv_stride as isize);
    }

    src = (*src_ybc).v_buffer;
    dst = (*dst_ybc).v_buffer;

    for _ in 0..(*src_ybc).uv_height {
        ptr::copy_nonoverlapping(src, dst, (*src_ybc).uv_width as usize);
        src = src.offset((*src_ybc).uv_stride as isize);
        dst = dst.offset((*dst_ybc).uv_stride as isize);
    }

    vp8_yv12_extend_frame_borders_c(dst_ybc);
}

/// `vpx_yv12_copy_y_c` (yv12extend.c:311). Copies only the luma plane
/// from `src_ybc` into `dst_ybc`. Does *not* extend borders.
///
/// # Safety
/// Both pointers must reference valid `Yv12BufferConfig`s with
/// matching luma dimensions.
pub unsafe fn vpx_yv12_copy_y_c(src_ybc: *const Yv12BufferConfig, dst_ybc: *mut Yv12BufferConfig) {
    let mut src: *const u8 = (*src_ybc).y_buffer;
    let mut dst: *mut u8 = (*dst_ybc).y_buffer;

    for _ in 0..(*src_ybc).y_height {
        ptr::copy_nonoverlapping(src, dst, (*src_ybc).y_width as usize);
        src = src.offset((*src_ybc).y_stride as isize);
        dst = dst.offset((*dst_ybc).y_stride as isize);
    }
}
