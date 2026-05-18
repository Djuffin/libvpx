//! Literal translation of `vpx/src/vpx_image.c` — the `vpx_image_t`
//! lifecycle (`vpx_img_alloc`, `vpx_img_wrap`, `vpx_img_set_rect`,
//! `vpx_img_flip`, `vpx_img_free`).
//!
//! Mirrors the C control flow verbatim. Public types (`vpx_image_t`,
//! `vpx_img_fmt_t`, the `VPX_IMG_FMT_*` / `VPX_PLANE_*` constants)
//! live in `crate::vpx_api`.

#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use core::ffi::c_void;
use core::ptr;

use crate::vpx_api::*;

// ===========================================================================
// External allocator hooks (provided by `vpx_mem/vpx_mem.c`).
// ===========================================================================

use crate::vpx_mem::{vpx_free, vpx_memalign};

unsafe extern "C" {
    fn calloc(nmemb: usize, size: usize) -> *mut c_void;
    fn free(ptr: *mut c_void);
}

// ===========================================================================
// Implementation.
// ===========================================================================

/// Mirrors `INT_MAX` from `<limits.h>` (32-bit signed maximum).
const INT_MAX_U64: u64 = i32::MAX as u64;
/// Mirrors `UINT_MAX` from `<limits.h>` (32-bit unsigned maximum).
const UINT_MAX: u32 = u32::MAX;

/// `static int is_valid_img_fmt(vpx_img_fmt_t fmt)` — admission test
/// for the format enum.
fn is_valid_img_fmt(fmt: vpx_img_fmt_t) -> bool {
    matches!(
        fmt,
        VPX_IMG_FMT_YV12
            | VPX_IMG_FMT_I420
            | VPX_IMG_FMT_I422
            | VPX_IMG_FMT_I444
            | VPX_IMG_FMT_I440
            | VPX_IMG_FMT_NV12
            | VPX_IMG_FMT_I42016
            | VPX_IMG_FMT_I42216
            | VPX_IMG_FMT_I44416
            | VPX_IMG_FMT_I44016
    )
}

/// `static vpx_image_t *img_alloc_helper(...)` — common construction
/// routine shared by `vpx_img_alloc` and `vpx_img_wrap`.
unsafe fn img_alloc_helper(
    mut img: *mut vpx_image_t,
    fmt: vpx_img_fmt_t,
    d_w: u32,
    d_h: u32,
    mut buf_align: u32,
    mut stride_align: u32,
    img_data: *mut u8,
) -> *mut vpx_image_t {
    'fail: {
        if !img.is_null() {
            ptr::write_bytes(img as *mut u8, 0, core::mem::size_of::<vpx_image_t>());
        }

        if !is_valid_img_fmt(fmt) {
            break 'fail;
        }

        /* Impose maximum values on input parameters so that this function can
         * perform arithmetic operations without worrying about overflows.
         */
        if d_w > 0x08000000 || d_h > 0x08000000 || buf_align > 65536 || stride_align > 65536 {
            break 'fail;
        }

        /* Treat align==0 like align==1 */
        if buf_align == 0 {
            buf_align = 1;
        }

        /* Validate alignment (must be power of 2) */
        if buf_align & (buf_align - 1) != 0 {
            break 'fail;
        }

        /* Treat align==0 like align==1 */
        if stride_align == 0 {
            stride_align = 1;
        }

        /* Validate alignment (must be power of 2) */
        if stride_align & (stride_align - 1) != 0 {
            break 'fail;
        }

        /* Get sample size for this format */
        let bps: u32 = match fmt {
            VPX_IMG_FMT_I420 | VPX_IMG_FMT_YV12 | VPX_IMG_FMT_NV12 => 12,
            VPX_IMG_FMT_I422 | VPX_IMG_FMT_I440 => 16,
            VPX_IMG_FMT_I444 => 24,
            VPX_IMG_FMT_I42016 => 24,
            VPX_IMG_FMT_I42216 | VPX_IMG_FMT_I44016 => 32,
            VPX_IMG_FMT_I44416 => 48,
            _ => 16,
        };

        /* Get chroma shift values for this format */
        // For VPX_IMG_FMT_NV12, xcs needs to be 0 such that UV data is all
        // read at once.
        let xcs: u32 = match fmt {
            VPX_IMG_FMT_I420 | VPX_IMG_FMT_YV12 | VPX_IMG_FMT_I422 | VPX_IMG_FMT_I42016
            | VPX_IMG_FMT_I42216 => 1,
            _ => 0,
        };

        let ycs: u32 = match fmt {
            VPX_IMG_FMT_I420 | VPX_IMG_FMT_NV12 | VPX_IMG_FMT_I440 | VPX_IMG_FMT_YV12
            | VPX_IMG_FMT_I42016 | VPX_IMG_FMT_I44016 => 1,
            _ => 0,
        };

        /* Calculate storage sizes. */
        let (w, h) = if !img_data.is_null() {
            /* If the buffer was allocated externally, the width and height
             * shouldn't be adjusted. */
            (d_w, d_h)
        } else {
            /* Calculate storage sizes given the chroma subsampling */
            let x_align = (1u32 << xcs) - 1;
            let w = (d_w + x_align) & !x_align;
            debug_assert!(d_w <= w);
            let y_align = (1u32 << ycs) - 1;
            let h = (d_h + y_align) & !y_align;
            debug_assert!(d_h <= h);
            (w, h)
        };

        let mut s: u64 = if (fmt & VPX_IMG_FMT_PLANAR) != 0 {
            w as u64
        } else {
            (bps as u64) * (w as u64) / 8
        };
        if (fmt & VPX_IMG_FMT_HIGHBITDEPTH) != 0 {
            s *= 2;
        }
        s = (s + stride_align as u64 - 1) & !((stride_align as u64) - 1);
        if s > INT_MAX_U64 {
            break 'fail;
        }
        let stride_in_bytes: i32 = s as i32;
        if (fmt & VPX_IMG_FMT_HIGHBITDEPTH) != 0 {
            s /= 2;
        }

        /* Allocate the new image */
        if img.is_null() {
            img = calloc(1, core::mem::size_of::<vpx_image_t>()) as *mut vpx_image_t;

            if img.is_null() {
                break 'fail;
            }

            (*img).self_allocd = 1;
        }

        (*img).img_data = img_data;

        if img_data.is_null() {
            let alloc_size: u64 = if (fmt & VPX_IMG_FMT_PLANAR) != 0 {
                (h as u64) * s * (bps as u64) / 8
            } else {
                (h as u64) * s
            };

            if alloc_size != (alloc_size as usize) as u64 {
                break 'fail;
            }

            (*img).img_data = vpx_memalign(buf_align as usize, alloc_size as usize) as *mut u8;
            (*img).img_data_owner = 1;
        }

        if (*img).img_data.is_null() {
            break 'fail;
        }

        (*img).fmt = fmt;
        (*img).bit_depth = if (fmt & VPX_IMG_FMT_HIGHBITDEPTH) != 0 {
            16
        } else {
            8
        };
        (*img).w = w;
        (*img).h = h;
        (*img).x_chroma_shift = xcs;
        (*img).y_chroma_shift = ycs;
        (*img).bps = bps as i32;

        /* Calculate strides */
        (*img).stride[VPX_PLANE_Y] = stride_in_bytes;
        (*img).stride[VPX_PLANE_ALPHA] = stride_in_bytes;
        (*img).stride[VPX_PLANE_U] = stride_in_bytes >> xcs;
        (*img).stride[VPX_PLANE_V] = stride_in_bytes >> xcs;

        /* Default viewport to entire image. (This vpx_img_set_rect call
         * always succeeds.) */
        let ret = vpx_img_set_rect(img, 0, 0, d_w, d_h);
        debug_assert!(ret == 0);
        return img;
    }

    // fail:
    vpx_img_free(img);
    ptr::null_mut()
}

/// `vpx_image_t *vpx_img_alloc(...)` — allocate-and-own constructor.

pub unsafe fn vpx_img_alloc(
    img: *mut vpx_image_t,
    fmt: vpx_img_fmt_t,
    d_w: u32,
    d_h: u32,
    align: u32,
) -> *mut vpx_image_t {
    img_alloc_helper(img, fmt, d_w, d_h, align, align, ptr::null_mut())
}

/// `vpx_image_t *vpx_img_wrap(...)` — wrap caller-supplied pixels.

pub unsafe fn vpx_img_wrap(
    img: *mut vpx_image_t,
    fmt: vpx_img_fmt_t,
    d_w: u32,
    d_h: u32,
    stride_align: u32,
    img_data: *mut u8,
) -> *mut vpx_image_t {
    /* Set buf_align = 1. It is ignored by img_alloc_helper because img_data is
     * not NULL. */
    img_alloc_helper(img, fmt, d_w, d_h, 1, stride_align, img_data)
}

/// `int vpx_img_set_rect(...)` — install the visible viewport.

pub unsafe fn vpx_img_set_rect(img: *mut vpx_image_t, x: u32, y: u32, w: u32, h: u32) -> i32 {
    if x <= UINT_MAX - w && x + w <= (*img).w && y <= UINT_MAX - h && y + h <= (*img).h {
        (*img).d_w = w;
        (*img).d_h = h;

        /* Calculate plane pointers */
        if ((*img).fmt & VPX_IMG_FMT_PLANAR) == 0 {
            (*img).planes[VPX_PLANE_PACKED] = (*img).img_data.add(
                (x as usize) * ((*img).bps as usize) / 8
                    + (y as usize) * ((*img).stride[VPX_PLANE_PACKED] as usize),
            );
        } else {
            let bytes_per_sample: i32 = if ((*img).fmt & VPX_IMG_FMT_HIGHBITDEPTH) != 0 {
                2
            } else {
                1
            };
            let mut data: *mut u8 = (*img).img_data;

            if ((*img).fmt & VPX_IMG_FMT_HAS_ALPHA) != 0 {
                (*img).planes[VPX_PLANE_ALPHA] = data.add(
                    (x as usize) * (bytes_per_sample as usize)
                        + (y as usize) * ((*img).stride[VPX_PLANE_ALPHA] as usize),
                );
                data = data.add(((*img).h as usize) * ((*img).stride[VPX_PLANE_ALPHA] as usize));
            }

            (*img).planes[VPX_PLANE_Y] = data.add(
                (x as usize) * (bytes_per_sample as usize)
                    + (y as usize) * ((*img).stride[VPX_PLANE_Y] as usize),
            );
            data = data.add(((*img).h as usize) * ((*img).stride[VPX_PLANE_Y] as usize));

            let uv_x: u32 = x >> (*img).x_chroma_shift;
            let uv_y: u32 = y >> (*img).y_chroma_shift;
            if (*img).fmt == VPX_IMG_FMT_NV12 {
                (*img).planes[VPX_PLANE_U] = data
                    .add((uv_x as usize) + (uv_y as usize) * ((*img).stride[VPX_PLANE_U] as usize));
                (*img).planes[VPX_PLANE_V] = (*img).planes[VPX_PLANE_U].add(1);
            } else if ((*img).fmt & VPX_IMG_FMT_UV_FLIP) == 0 {
                (*img).planes[VPX_PLANE_U] = data.add(
                    (uv_x as usize) * (bytes_per_sample as usize)
                        + (uv_y as usize) * ((*img).stride[VPX_PLANE_U] as usize),
                );
                data = data.add(
                    (((*img).h >> (*img).y_chroma_shift) as usize)
                        * ((*img).stride[VPX_PLANE_U] as usize),
                );
                (*img).planes[VPX_PLANE_V] = data.add(
                    (uv_x as usize) * (bytes_per_sample as usize)
                        + (uv_y as usize) * ((*img).stride[VPX_PLANE_V] as usize),
                );
            } else {
                (*img).planes[VPX_PLANE_V] = data.add(
                    (uv_x as usize) * (bytes_per_sample as usize)
                        + (uv_y as usize) * ((*img).stride[VPX_PLANE_V] as usize),
                );
                data = data.add(
                    (((*img).h >> (*img).y_chroma_shift) as usize)
                        * ((*img).stride[VPX_PLANE_V] as usize),
                );
                (*img).planes[VPX_PLANE_U] = data.add(
                    (uv_x as usize) * (bytes_per_sample as usize)
                        + (uv_y as usize) * ((*img).stride[VPX_PLANE_U] as usize),
                );
            }
        }
        return 0;
    }
    -1
}

/// `void vpx_img_flip(vpx_image_t *img)` — present the image upside-down.

pub unsafe fn vpx_img_flip(img: *mut vpx_image_t) {
    /* Note: In the calculation pointer adjustment calculation, we want the
     * rhs to be promoted to a signed type. Section 6.3.1.8 of the ISO C99
     * standard indicates that if the adjustment parameter is unsigned, the
     * stride parameter will be promoted to unsigned, causing errors when
     * the lhs is a larger type than the rhs.
     */
    (*img).planes[VPX_PLANE_Y] = (*img).planes[VPX_PLANE_Y]
        .offset(((*img).d_h as i32 - 1) as isize * (*img).stride[VPX_PLANE_Y] as isize);
    (*img).stride[VPX_PLANE_Y] = -(*img).stride[VPX_PLANE_Y];

    (*img).planes[VPX_PLANE_U] = (*img).planes[VPX_PLANE_U].offset(
        (((*img).d_h >> (*img).y_chroma_shift) as i32 - 1) as isize
            * (*img).stride[VPX_PLANE_U] as isize,
    );
    (*img).stride[VPX_PLANE_U] = -(*img).stride[VPX_PLANE_U];

    (*img).planes[VPX_PLANE_V] = (*img).planes[VPX_PLANE_V].offset(
        (((*img).d_h >> (*img).y_chroma_shift) as i32 - 1) as isize
            * (*img).stride[VPX_PLANE_V] as isize,
    );
    (*img).stride[VPX_PLANE_V] = -(*img).stride[VPX_PLANE_V];

    (*img).planes[VPX_PLANE_ALPHA] = (*img).planes[VPX_PLANE_ALPHA]
        .offset(((*img).d_h as i32 - 1) as isize * (*img).stride[VPX_PLANE_ALPHA] as isize);
    (*img).stride[VPX_PLANE_ALPHA] = -(*img).stride[VPX_PLANE_ALPHA];
}

/// `void vpx_img_free(vpx_image_t *img)` — coordinated tear-down.

pub unsafe fn vpx_img_free(img: *mut vpx_image_t) {
    if !img.is_null() {
        if !(*img).img_data.is_null() && (*img).img_data_owner != 0 {
            vpx_free((*img).img_data as *mut c_void);
        }

        if (*img).self_allocd != 0 {
            free(img as *mut c_void);
        }
    }
}
