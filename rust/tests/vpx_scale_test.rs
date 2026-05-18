#![allow(unsafe_op_in_unsafe_fn)]
//! Port of `test/vpx_scale_test.cc` + `test/vpx_scale_test.h` to Rust
//! integration tests.
//!
//! Two tests, each parameterized over (W, H) ∈ kSizesToTest × itself:
//!   - `ExtendBorder` — `vp8_yv12_extend_frame_borders_c` replicates the
//!     32-px guard band; reference impl does the same in pure Rust;
//!     compare whole `buffer_alloc`.
//!   - `CopyFrame`    — `vp8_yv12_copy_frame_c` copies + extends borders
//!     into `dst_img_`; compare against the reference impl.
//!
//! SIMD variants from the C suite are dropped — only the `_c` reference
//! lives in our crate. We use the 7-size list (skip `16383`) to keep
//! peak allocation reasonable.

use std::mem;

use vp8_decoder_rs::types::{VP8_BORDER_IN_PIXELS, Yv12BufferConfig};
use vp8_decoder_rs::yv12config::{vp8_yv12_alloc_frame_buffer, vp8_yv12_de_alloc_frame_buffer};
use vp8_decoder_rs::yv12extend::{vp8_yv12_copy_frame_c, vp8_yv12_extend_frame_borders_c};

const SIZES_TO_TEST: [i32; 7] = [1, 15, 33, 145, 512, 1025, 3840];

const BUF_FILLER: u8 = 123;
const BUF_MAX: u8 = BUF_FILLER - 1;

// --- Yv12 helpers (mirror `VpxScaleBase`) ----------------------------

unsafe fn reset_image(img: *mut Yv12BufferConfig, width: i32, height: i32) {
    // memset(img, 0, sizeof(*img))
    std::ptr::write_bytes(img as *mut u8, 0, mem::size_of::<Yv12BufferConfig>());
    let rc = vp8_yv12_alloc_frame_buffer(img, width, height, VP8_BORDER_IN_PIXELS);
    assert_eq!(rc, 0, "alloc failed for {width}x{height}");
    std::ptr::write_bytes((*img).buffer_alloc, BUF_FILLER, (*img).frame_size);
}

unsafe fn fill_plane(buf: *mut u8, width: i32, height: i32, stride: i32) {
    for y in 0..height {
        for x in 0..width {
            let v = ((x + width * y) as u32 % BUF_MAX as u32) as u8;
            *buf.offset((x + y * stride) as isize) = v;
        }
    }
}

unsafe fn reset_images(
    img: *mut Yv12BufferConfig,
    ref_img: *mut Yv12BufferConfig,
    dst_img: *mut Yv12BufferConfig,
    width: i32,
    height: i32,
) {
    reset_image(img, width, height);
    reset_image(ref_img, width, height);
    reset_image(dst_img, width, height);

    fill_plane(
        (*img).y_buffer,
        (*img).y_crop_width,
        (*img).y_crop_height,
        (*img).y_stride,
    );
    fill_plane(
        (*img).u_buffer,
        (*img).uv_crop_width,
        (*img).uv_crop_height,
        (*img).uv_stride,
    );
    fill_plane(
        (*img).v_buffer,
        (*img).uv_crop_width,
        (*img).uv_crop_height,
        (*img).uv_stride,
    );
}

unsafe fn dealloc_images(
    img: *mut Yv12BufferConfig,
    ref_img: *mut Yv12BufferConfig,
    dst_img: *mut Yv12BufferConfig,
) {
    vp8_yv12_de_alloc_frame_buffer(img);
    vp8_yv12_de_alloc_frame_buffer(ref_img);
    vp8_yv12_de_alloc_frame_buffer(dst_img);
}

/// Reference border-extend (`ExtendPlane` from vpx_scale_test.h:115).
unsafe fn extend_plane(
    buf: *mut u8,
    crop_width: i32,
    crop_height: i32,
    width: i32,
    height: i32,
    stride: i32,
    padding: i32,
) {
    let padding = padding as isize;
    let stride = stride as isize;
    let crop_width = crop_width as isize;
    let crop_height = crop_height as isize;

    let mut left = buf.offset(-padding);
    let mut right = buf.offset(crop_width);
    let right_extend = (padding + (width as isize - crop_width)) as usize;
    let bottom_extend = (padding + (height as isize - crop_height)) as usize;

    for _ in 0..crop_height {
        std::ptr::write_bytes(left, *left.offset(padding), padding as usize);
        std::ptr::write_bytes(right, *right.offset(-1), right_extend);
        left = left.offset(stride);
        right = right.offset(stride);
    }

    let left = buf.offset(-padding);
    let mut top = left.offset(-(stride * padding));
    let extend_width = (padding + crop_width + right_extend as isize) as usize;

    for _ in 0..padding {
        std::ptr::copy_nonoverlapping(left, top, extend_width);
        top = top.offset(stride);
    }

    let mut bottom = left.offset(crop_height * stride);
    let last_row = left.offset((crop_height - 1) * stride);
    for _ in 0..bottom_extend {
        std::ptr::copy_nonoverlapping(last_row, bottom, extend_width);
        bottom = bottom.offset(stride);
    }
}

unsafe fn reference_extend_border(ref_img: *mut Yv12BufferConfig) {
    extend_plane(
        (*ref_img).y_buffer,
        (*ref_img).y_crop_width,
        (*ref_img).y_crop_height,
        (*ref_img).y_width,
        (*ref_img).y_height,
        (*ref_img).y_stride,
        (*ref_img).border,
    );
    extend_plane(
        (*ref_img).u_buffer,
        (*ref_img).uv_crop_width,
        (*ref_img).uv_crop_height,
        (*ref_img).uv_width,
        (*ref_img).uv_height,
        (*ref_img).uv_stride,
        (*ref_img).border / 2,
    );
    extend_plane(
        (*ref_img).v_buffer,
        (*ref_img).uv_crop_width,
        (*ref_img).uv_crop_height,
        (*ref_img).uv_width,
        (*ref_img).uv_height,
        (*ref_img).uv_stride,
        (*ref_img).border / 2,
    );
}

unsafe fn reference_copy_frame(img: *const Yv12BufferConfig, ref_img: *mut Yv12BufferConfig) {
    assert_eq!((*ref_img).frame_size, (*img).frame_size);
    for y in 0..(*img).y_crop_height {
        for x in 0..(*img).y_crop_width {
            let dst_off = (x + y * (*ref_img).y_stride) as isize;
            let src_off = (x + y * (*img).y_stride) as isize;
            *(*ref_img).y_buffer.offset(dst_off) = *(*img).y_buffer.offset(src_off);
        }
    }
    for y in 0..(*img).uv_crop_height {
        for x in 0..(*img).uv_crop_width {
            let dst_off = (x + y * (*ref_img).uv_stride) as isize;
            let src_off = (x + y * (*img).uv_stride) as isize;
            *(*ref_img).u_buffer.offset(dst_off) = *(*img).u_buffer.offset(src_off);
            *(*ref_img).v_buffer.offset(dst_off) = *(*img).v_buffer.offset(src_off);
        }
    }
    reference_extend_border(ref_img);
}

unsafe fn compare_images(ref_img: *const Yv12BufferConfig, actual: *const Yv12BufferConfig) {
    assert_eq!((*ref_img).frame_size, (*actual).frame_size);
    let a = std::slice::from_raw_parts((*ref_img).buffer_alloc, (*ref_img).frame_size);
    let b = std::slice::from_raw_parts((*actual).buffer_alloc, (*actual).frame_size);
    assert_eq!(a, b, "buffer_alloc mismatch");
}

unsafe fn new_yv12() -> Yv12BufferConfig {
    mem::zeroed()
}

// --- Tests --------------------------------------------------------

#[test]
fn extend_border() {
    unsafe {
        let mut img: Yv12BufferConfig = new_yv12();
        let mut ref_img: Yv12BufferConfig = new_yv12();
        let mut dst_img: Yv12BufferConfig = new_yv12();

        for &h in &SIZES_TO_TEST {
            for &w in &SIZES_TO_TEST {
                reset_images(&mut img, &mut ref_img, &mut dst_img, w, h);
                reference_copy_frame(&img, &mut ref_img);
                vp8_yv12_extend_frame_borders_c(&mut img);
                compare_images(&ref_img, &img);
                dealloc_images(&mut img, &mut ref_img, &mut dst_img);
            }
        }
    }
}

#[test]
fn copy_frame() {
    unsafe {
        let mut img: Yv12BufferConfig = new_yv12();
        let mut ref_img: Yv12BufferConfig = new_yv12();
        let mut dst_img: Yv12BufferConfig = new_yv12();

        for &h in &SIZES_TO_TEST {
            for &w in &SIZES_TO_TEST {
                reset_images(&mut img, &mut ref_img, &mut dst_img, w, h);
                reference_copy_frame(&img, &mut ref_img);
                vp8_yv12_copy_frame_c(&img, &mut dst_img);
                compare_images(&ref_img, &dst_img);
                dealloc_images(&mut img, &mut ref_img, &mut dst_img);
            }
        }
    }
}
