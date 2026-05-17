//! `vpx_util/vpx_write_yuv_frame.c` — debug YUV dump helper.
//!
//! Literal Rust transliteration of the single `vpx_write_yuv_frame`
//! function. The C source body is gated behind four `OUTPUT_YUV_*`
//! compile-time macros (`OUTPUT_YUV_SRC`, `OUTPUT_YUV_DENOISED`,
//! `OUTPUT_YUV_SKINMAP`, `OUTPUT_YUV_SVC_SRC`). None are ever defined
//! by `./configure`; each is a single-line `#define` that a developer
//! adds locally while hunting a bug. The Rust port mirrors that with
//! Cargo features of the same names — without any of them, the body
//! reduces to a no-op stub and the function exists purely as a symbol
//! for call sites to bind against.
//!
//! See `documentation/vp8_files/vpx_write_yuv_frame.md`.

#![allow(non_snake_case)]
#![allow(non_camel_case_types)]

use crate::types::Yv12BufferConfig;

// `FILE *` stand-in. We do not have a libc dependency wired into this
// crate yet; treat the stream as an opaque pointer. The body never
// dereferences it directly — it is forwarded straight to `fwrite`.
pub type FILE = core::ffi::c_void;

// `size_t fwrite(const void *ptr, size_t size, size_t nmemb,
//                FILE *stream);`
//
// Stubbed FFI declaration so the dump path links against the system
// libc. Only compiled when one of the OUTPUT_YUV_* features is active;
// without them the function body never references `fwrite` and the
// extern block can be elided entirely (still kept here for clarity).
#[cfg(any(
    feature = "OUTPUT_YUV_SRC",
    feature = "OUTPUT_YUV_DENOISED",
    feature = "OUTPUT_YUV_SKINMAP",
    feature = "OUTPUT_YUV_SVC_SRC",
))]
extern "C" {
    fn fwrite(
        ptr: *const core::ffi::c_void,
        size: usize,
        nmemb: usize,
        stream: *mut FILE,
    ) -> usize;
}

/// `void vpx_write_yuv_frame(FILE *yuv_file, YV12_BUFFER_CONFIG *s);`
///
/// Append the Y, then U, then V planes of `*s` to `yuv_file`, one row
/// at a time, in raster order. No header is written; downstream tools
/// are expected to know `y_width`/`uv_width`/`y_crop_height` out of
/// band (typically encoded into the filename, e.g. `dump_640x480.yuv`).
///
/// Preconditions (debug-quality, unchecked):
///   - `yuv_file` is open for writing in binary mode.
///   - `s->y_buffer`, `s->u_buffer`, `s->v_buffer` point at the
///     visible (0, 0) pixel of their plane (past the border).
///   - `s->y_crop_height >= 1` and `s->uv_crop_height >= 1`. The
///     do/while loops would underflow if a plane were zero-height.
///
/// # Safety
///
/// `yuv_file` must be a valid `FILE *` (or null when the body
/// compiles to a no-op). `s` must be a valid pointer to a fully
/// populated `Yv12BufferConfig` whose plane buffers cover at least
/// `y_stride * y_crop_height` (luma) and `uv_stride * uv_crop_height`
/// (chroma) bytes.
#[no_mangle]
pub unsafe extern "C" fn vpx_write_yuv_frame(
    yuv_file: *mut FILE,
    s: *mut Yv12BufferConfig,
) {
    #[cfg(any(
        feature = "OUTPUT_YUV_SRC",
        feature = "OUTPUT_YUV_DENOISED",
        feature = "OUTPUT_YUV_SKINMAP",
        feature = "OUTPUT_YUV_SVC_SRC",
    ))]
    {
        // unsigned char *src = s->y_buffer;
        // int h = s->y_crop_height;
        let mut src: *mut u8 = (*s).y_buffer;
        let mut h: i32 = (*s).y_crop_height;

        // do { fwrite(src, s->y_width, 1, yuv_file); src += s->y_stride; }
        // while (--h);
        loop {
            fwrite(
                src as *const core::ffi::c_void,
                (*s).y_width as usize,
                1,
                yuv_file,
            );
            src = src.offset((*s).y_stride as isize);
            h -= 1;
            if h == 0 {
                break;
            }
        }

        // src = s->u_buffer;
        // h = s->uv_crop_height;
        src = (*s).u_buffer;
        h = (*s).uv_crop_height;

        loop {
            fwrite(
                src as *const core::ffi::c_void,
                (*s).uv_width as usize,
                1,
                yuv_file,
            );
            src = src.offset((*s).uv_stride as isize);
            h -= 1;
            if h == 0 {
                break;
            }
        }

        // src = s->v_buffer;
        // h = s->uv_crop_height;
        src = (*s).v_buffer;
        h = (*s).uv_crop_height;

        loop {
            fwrite(
                src as *const core::ffi::c_void,
                (*s).uv_width as usize,
                1,
                yuv_file,
            );
            src = src.offset((*s).uv_stride as isize);
            h -= 1;
            if h == 0 {
                break;
            }
        }
    }

    #[cfg(not(any(
        feature = "OUTPUT_YUV_SRC",
        feature = "OUTPUT_YUV_DENOISED",
        feature = "OUTPUT_YUV_SKINMAP",
        feature = "OUTPUT_YUV_SVC_SRC",
    )))]
    {
        // (void)yuv_file;
        // (void)s;
        let _ = yuv_file;
        let _ = s;
    }
}
