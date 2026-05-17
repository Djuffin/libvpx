//! Sub-pixel interpolation filters — literal translation of
//! `vp8/common/filter.c`.
//!
//! Two reference families live here:
//!   * 6-tap luma sub-pel filter (`vp8_sixtap_predict*_c`).
//!   * 2-tap bilinear filter (`vp8_bilinear_predict*_c`).
//!
//! Both are dispatched in the C build through `vp8_rtcd.h`; the SIMD
//! variants must remain bit-exact with these references. The two
//! coefficient tables (`vp8_bilinear_filters`, `vp8_sub_pel_filters`)
//! live in [`crate::tables`] and are re-exported below under their
//! original C names so call sites can use them unchanged.

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use crate::tables::{VP8_BILINEAR_FILTERS, VP8_FILTER_SHIFT, VP8_FILTER_WEIGHT, VP8_SUB_PEL_FILTERS};

// ---------------------------------------------------------------------------
// Re-exports of the tap tables under their original C identifiers.
// ---------------------------------------------------------------------------

/// `vp8_bilinear_filters` — 2-tap bilinear filter coefficients. RFC 6386 §6.5.2.
pub const vp8_bilinear_filters: [[i16; 2]; 8] = VP8_BILINEAR_FILTERS;

/// `vp8_sub_pel_filters` — 6-tap sub-pixel filter coefficients. RFC 6386 §6.5.1.
pub const vp8_sub_pel_filters: [[i16; 6]; 8] = VP8_SUB_PEL_FILTERS;

// ---------------------------------------------------------------------------
// 6-tap helpers
// ---------------------------------------------------------------------------

/// `filter_block2d_first_pass` — vp8/common/filter.c:33.
///
/// 1-D 6-tap convolution; `pixel_step` selects horizontal (1) or
/// vertical (= `src_pixels_per_line`) direction. Emits 32-bit
/// intermediates clamped to `[0, 255]`.
unsafe fn filter_block2d_first_pass(
    mut src_ptr: *mut u8,
    mut output_ptr: *mut i32,
    src_pixels_per_line: u32,
    pixel_step: u32,
    output_height: u32,
    output_width: u32,
    vp8_filter: *const i16,
) {
    let mut i: u32;
    let mut j: u32;
    let mut Temp: i32;

    i = 0;
    while i < output_height {
        j = 0;
        while j < output_width {
            Temp = (*src_ptr.offset(-2 * pixel_step as isize) as i32)
                * (*vp8_filter.offset(0) as i32)
                + (*src_ptr.offset(-1 * pixel_step as isize) as i32)
                    * (*vp8_filter.offset(1) as i32)
                + (*src_ptr.offset(0) as i32) * (*vp8_filter.offset(2) as i32)
                + (*src_ptr.offset(pixel_step as isize) as i32)
                    * (*vp8_filter.offset(3) as i32)
                + (*src_ptr.offset(2 * pixel_step as isize) as i32)
                    * (*vp8_filter.offset(4) as i32)
                + (*src_ptr.offset(3 * pixel_step as isize) as i32)
                    * (*vp8_filter.offset(5) as i32)
                + (VP8_FILTER_WEIGHT >> 1); /* Rounding */

            /* Normalize back to 0-255 */
            Temp = Temp >> VP8_FILTER_SHIFT;

            if Temp < 0 {
                Temp = 0;
            } else if Temp > 255 {
                Temp = 255;
            }

            *output_ptr.offset(j as isize) = Temp;
            src_ptr = src_ptr.offset(1);
            j += 1;
        }

        /* Next row... */
        src_ptr = src_ptr.offset((src_pixels_per_line - output_width) as isize);
        output_ptr = output_ptr.offset(output_width as isize);
        i += 1;
    }
}

/// `filter_block2d_second_pass` — vp8/common/filter.c:71.
///
/// Mirror of [`filter_block2d_first_pass`] but consumes a 32-bit
/// intermediate and writes 8-bit pels with `output_pitch` stride.
unsafe fn filter_block2d_second_pass(
    mut src_ptr: *mut i32,
    mut output_ptr: *mut u8,
    output_pitch: i32,
    src_pixels_per_line: u32,
    pixel_step: u32,
    output_height: u32,
    output_width: u32,
    vp8_filter: *const i16,
) {
    let mut i: u32;
    let mut j: u32;
    let mut Temp: i32;

    i = 0;
    while i < output_height {
        j = 0;
        while j < output_width {
            /* Apply filter */
            Temp = (*src_ptr.offset(-2 * pixel_step as isize)) * (*vp8_filter.offset(0) as i32)
                + (*src_ptr.offset(-1 * pixel_step as isize)) * (*vp8_filter.offset(1) as i32)
                + (*src_ptr.offset(0)) * (*vp8_filter.offset(2) as i32)
                + (*src_ptr.offset(pixel_step as isize)) * (*vp8_filter.offset(3) as i32)
                + (*src_ptr.offset(2 * pixel_step as isize)) * (*vp8_filter.offset(4) as i32)
                + (*src_ptr.offset(3 * pixel_step as isize)) * (*vp8_filter.offset(5) as i32)
                + (VP8_FILTER_WEIGHT >> 1); /* Rounding */

            /* Normalize back to 0-255 */
            Temp = Temp >> VP8_FILTER_SHIFT;

            if Temp < 0 {
                Temp = 0;
            } else if Temp > 255 {
                Temp = 255;
            }

            *output_ptr.offset(j as isize) = Temp as u8;
            src_ptr = src_ptr.offset(1);
            j += 1;
        }

        /* Start next row */
        src_ptr = src_ptr.offset((src_pixels_per_line - output_width) as isize);
        output_ptr = output_ptr.offset(output_pitch as isize);
        i += 1;
    }
}

/// `filter_block2d` — vp8/common/filter.c:111. The 4x4 driver.
unsafe fn filter_block2d(
    src_ptr: *mut u8,
    output_ptr: *mut u8,
    src_pixels_per_line: u32,
    output_pitch: i32,
    HFilter: *const i16,
    VFilter: *const i16,
) {
    let mut FData: [i32; 9 * 4] = [0; 9 * 4]; /* Temp data buffer used in filtering */

    /* First filter 1-D horizontally... */
    filter_block2d_first_pass(
        src_ptr.offset(-(2 * src_pixels_per_line as isize)),
        FData.as_mut_ptr(),
        src_pixels_per_line,
        1,
        9,
        4,
        HFilter,
    );

    /* then filter verticaly... */
    filter_block2d_second_pass(
        FData.as_mut_ptr().offset(8),
        output_ptr,
        output_pitch,
        4,
        4,
        4,
        4,
        VFilter,
    );
}

// ---------------------------------------------------------------------------
// 6-tap public entry points
// ---------------------------------------------------------------------------

/// `vp8_sixtap_predict4x4_c` — vp8/common/filter.c:125.
pub unsafe fn vp8_sixtap_predict4x4_c(
    src_ptr: *mut u8,
    src_pixels_per_line: i32,
    xoffset: i32,
    yoffset: i32,
    dst_ptr: *mut u8,
    dst_pitch: i32,
) {
    let HFilter: *const i16;
    let VFilter: *const i16;

    HFilter = vp8_sub_pel_filters[xoffset as usize].as_ptr(); /* 6 tap */
    VFilter = vp8_sub_pel_filters[yoffset as usize].as_ptr(); /* 6 tap */

    filter_block2d(
        src_ptr,
        dst_ptr,
        src_pixels_per_line as u32,
        dst_pitch,
        HFilter,
        VFilter,
    );
}

/// `vp8_sixtap_predict8x8_c` — vp8/common/filter.c:137.
pub unsafe fn vp8_sixtap_predict8x8_c(
    src_ptr: *mut u8,
    src_pixels_per_line: i32,
    xoffset: i32,
    yoffset: i32,
    dst_ptr: *mut u8,
    dst_pitch: i32,
) {
    let HFilter: *const i16;
    let VFilter: *const i16;
    let mut FData: [i32; 13 * 16] = [0; 13 * 16]; /* Temp data buffer used in filtering */

    HFilter = vp8_sub_pel_filters[xoffset as usize].as_ptr(); /* 6 tap */
    VFilter = vp8_sub_pel_filters[yoffset as usize].as_ptr(); /* 6 tap */

    /* First filter 1-D horizontally... */
    filter_block2d_first_pass(
        src_ptr.offset(-(2 * src_pixels_per_line as isize)),
        FData.as_mut_ptr(),
        src_pixels_per_line as u32,
        1,
        13,
        8,
        HFilter,
    );

    /* then filter verticaly... */
    filter_block2d_second_pass(
        FData.as_mut_ptr().offset(16),
        dst_ptr,
        dst_pitch,
        8,
        8,
        8,
        8,
        VFilter,
    );
}

/// `vp8_sixtap_predict8x4_c` — vp8/common/filter.c:156.
pub unsafe fn vp8_sixtap_predict8x4_c(
    src_ptr: *mut u8,
    src_pixels_per_line: i32,
    xoffset: i32,
    yoffset: i32,
    dst_ptr: *mut u8,
    dst_pitch: i32,
) {
    let HFilter: *const i16;
    let VFilter: *const i16;
    let mut FData: [i32; 13 * 16] = [0; 13 * 16]; /* Temp data buffer used in filtering */

    HFilter = vp8_sub_pel_filters[xoffset as usize].as_ptr(); /* 6 tap */
    VFilter = vp8_sub_pel_filters[yoffset as usize].as_ptr(); /* 6 tap */

    /* First filter 1-D horizontally... */
    filter_block2d_first_pass(
        src_ptr.offset(-(2 * src_pixels_per_line as isize)),
        FData.as_mut_ptr(),
        src_pixels_per_line as u32,
        1,
        9,
        8,
        HFilter,
    );

    /* then filter verticaly... */
    filter_block2d_second_pass(
        FData.as_mut_ptr().offset(16),
        dst_ptr,
        dst_pitch,
        8,
        8,
        4,
        8,
        VFilter,
    );
}

/// `vp8_sixtap_predict16x16_c` — vp8/common/filter.c:175.
pub unsafe fn vp8_sixtap_predict16x16_c(
    src_ptr: *mut u8,
    src_pixels_per_line: i32,
    xoffset: i32,
    yoffset: i32,
    dst_ptr: *mut u8,
    dst_pitch: i32,
) {
    let HFilter: *const i16;
    let VFilter: *const i16;
    let mut FData: [i32; 21 * 24] = [0; 21 * 24]; /* Temp data buffer used in filtering */

    HFilter = vp8_sub_pel_filters[xoffset as usize].as_ptr(); /* 6 tap */
    VFilter = vp8_sub_pel_filters[yoffset as usize].as_ptr(); /* 6 tap */

    /* First filter 1-D horizontally... */
    filter_block2d_first_pass(
        src_ptr.offset(-(2 * src_pixels_per_line as isize)),
        FData.as_mut_ptr(),
        src_pixels_per_line as u32,
        1,
        21,
        16,
        HFilter,
    );

    /* then filter verticaly... */
    filter_block2d_second_pass(
        FData.as_mut_ptr().offset(32),
        dst_ptr,
        dst_pitch,
        16,
        16,
        16,
        16,
        VFilter,
    );
}

// ---------------------------------------------------------------------------
// Bilinear helpers
// ---------------------------------------------------------------------------

/// `filter_block2d_bil_first_pass` — vp8/common/filter.c:216.
///
/// 1-D 2-tap bilinear pass with `u16` intermediates. No clamp needed
/// (taps sum to `VP8_FILTER_WEIGHT` and are non-negative).
unsafe fn filter_block2d_bil_first_pass(
    mut src_ptr: *mut u8,
    mut dst_ptr: *mut u16,
    src_stride: u32,
    height: u32,
    width: u32,
    vp8_filter: *const i16,
) {
    let mut i: u32;
    let mut j: u32;

    i = 0;
    while i < height {
        j = 0;
        while j < width {
            /* Apply bilinear filter */
            *dst_ptr.offset(j as isize) = (((*src_ptr.offset(0) as i32)
                * (*vp8_filter.offset(0) as i32)
                + (*src_ptr.offset(1) as i32) * (*vp8_filter.offset(1) as i32)
                + (VP8_FILTER_WEIGHT / 2))
                >> VP8_FILTER_SHIFT) as u16;
            src_ptr = src_ptr.offset(1);
            j += 1;
        }

        /* Next row... */
        src_ptr = src_ptr.offset((src_stride - width) as isize);
        dst_ptr = dst_ptr.offset(width as isize);
        i += 1;
    }
}

/// `filter_block2d_bil_second_pass` — vp8/common/filter.c:261.
///
/// Vertical bilinear pass; the neighbour-row offset is `width` because
/// the intermediate is laid out as contiguous `width`-wide rows.
unsafe fn filter_block2d_bil_second_pass(
    mut src_ptr: *mut u16,
    mut dst_ptr: *mut u8,
    dst_pitch: i32,
    height: u32,
    width: u32,
    vp8_filter: *const i16,
) {
    let mut i: u32;
    let mut j: u32;
    let mut Temp: i32;

    i = 0;
    while i < height {
        j = 0;
        while j < width {
            /* Apply filter */
            Temp = (*src_ptr.offset(0) as i32) * (*vp8_filter.offset(0) as i32)
                + (*src_ptr.offset(width as isize) as i32) * (*vp8_filter.offset(1) as i32)
                + (VP8_FILTER_WEIGHT / 2);
            *dst_ptr.offset(j as isize) = ((Temp >> VP8_FILTER_SHIFT) as u32) as u8;
            src_ptr = src_ptr.offset(1);
            j += 1;
        }

        /* Next row... */
        dst_ptr = dst_ptr.offset(dst_pitch as isize);
        i += 1;
    }
}

/// `filter_block2d_bil` — vp8/common/filter.c:307. Bilinear driver.
unsafe fn filter_block2d_bil(
    src_ptr: *mut u8,
    dst_ptr: *mut u8,
    src_pitch: u32,
    dst_pitch: u32,
    HFilter: *const i16,
    VFilter: *const i16,
    Width: i32,
    Height: i32,
) {
    let mut FData: [u16; 17 * 16] = [0; 17 * 16]; /* Temp data buffer used in filtering */

    /* First filter 1-D horizontally... */
    filter_block2d_bil_first_pass(
        src_ptr,
        FData.as_mut_ptr(),
        src_pitch,
        (Height + 1) as u32,
        Width as u32,
        HFilter,
    );

    /* then 1-D vertically... */
    filter_block2d_bil_second_pass(
        FData.as_mut_ptr(),
        dst_ptr,
        dst_pitch as i32,
        Height as u32,
        Width as u32,
        VFilter,
    );
}

// ---------------------------------------------------------------------------
// Bilinear public entry points
// ---------------------------------------------------------------------------

/// `vp8_bilinear_predict4x4_c` — vp8/common/filter.c:322.
pub unsafe fn vp8_bilinear_predict4x4_c(
    src_ptr: *mut u8,
    src_pixels_per_line: i32,
    xoffset: i32,
    yoffset: i32,
    dst_ptr: *mut u8,
    dst_pitch: i32,
) {
    let HFilter: *const i16;
    let VFilter: *const i16;

    // This represents a copy and is not required to be handled by optimizations.
    assert!((xoffset | yoffset) != 0);

    HFilter = vp8_bilinear_filters[xoffset as usize].as_ptr();
    VFilter = vp8_bilinear_filters[yoffset as usize].as_ptr();
    filter_block2d_bil(
        src_ptr,
        dst_ptr,
        src_pixels_per_line as u32,
        dst_pitch as u32,
        HFilter,
        VFilter,
        4,
        4,
    );
}

/// `vp8_bilinear_predict8x8_c` — vp8/common/filter.c:337.
pub unsafe fn vp8_bilinear_predict8x8_c(
    src_ptr: *mut u8,
    src_pixels_per_line: i32,
    xoffset: i32,
    yoffset: i32,
    dst_ptr: *mut u8,
    dst_pitch: i32,
) {
    let HFilter: *const i16;
    let VFilter: *const i16;

    assert!((xoffset | yoffset) != 0);

    HFilter = vp8_bilinear_filters[xoffset as usize].as_ptr();
    VFilter = vp8_bilinear_filters[yoffset as usize].as_ptr();

    filter_block2d_bil(
        src_ptr,
        dst_ptr,
        src_pixels_per_line as u32,
        dst_pitch as u32,
        HFilter,
        VFilter,
        8,
        8,
    );
}

/// `vp8_bilinear_predict8x4_c` — vp8/common/filter.c:352.
pub unsafe fn vp8_bilinear_predict8x4_c(
    src_ptr: *mut u8,
    src_pixels_per_line: i32,
    xoffset: i32,
    yoffset: i32,
    dst_ptr: *mut u8,
    dst_pitch: i32,
) {
    let HFilter: *const i16;
    let VFilter: *const i16;

    assert!((xoffset | yoffset) != 0);

    HFilter = vp8_bilinear_filters[xoffset as usize].as_ptr();
    VFilter = vp8_bilinear_filters[yoffset as usize].as_ptr();

    filter_block2d_bil(
        src_ptr,
        dst_ptr,
        src_pixels_per_line as u32,
        dst_pitch as u32,
        HFilter,
        VFilter,
        8,
        4,
    );
}

/// `vp8_bilinear_predict16x16_c` — vp8/common/filter.c:367.
pub unsafe fn vp8_bilinear_predict16x16_c(
    src_ptr: *mut u8,
    src_pixels_per_line: i32,
    xoffset: i32,
    yoffset: i32,
    dst_ptr: *mut u8,
    dst_pitch: i32,
) {
    let HFilter: *const i16;
    let VFilter: *const i16;

    assert!((xoffset | yoffset) != 0);

    HFilter = vp8_bilinear_filters[xoffset as usize].as_ptr();
    VFilter = vp8_bilinear_filters[yoffset as usize].as_ptr();

    filter_block2d_bil(
        src_ptr,
        dst_ptr,
        src_pixels_per_line as u32,
        dst_pitch as u32,
        HFilter,
        VFilter,
        16,
        16,
    );
}
