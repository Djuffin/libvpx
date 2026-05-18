//! Literal Rust translation of `vpx_scale/generic/yv12config.c`.
//!
//! Allocator, deallocator, and (re)allocator for [`Yv12BufferConfig`] — the
//! single contiguous YV12 frame-buffer slab used everywhere in the VP8
//! decoder for reference pictures and reconstruction targets. The three
//! VP8 entry points (`vp8_yv12_alloc_frame_buffer`,
//! `vp8_yv12_realloc_frame_buffer`, `vp8_yv12_de_alloc_frame_buffer`)
//! mirror the C control flow verbatim: a single `vpx_memalign(32, ...)`
//! produces the slab, and `y_buffer` / `u_buffer` / `v_buffer` are
//! computed as offsets into it so that index `[0]` is the top-left
//! displayable pixel and the border lives at negative offsets.
//!
//! Geometry invariants preserved from the C source:
//!   - `aligned_width  = (width  + 15) & !15`
//!   - `aligned_height = (height + 15) & !15`
//!   - `y_stride  = (aligned_width + 2*border + 31) & !31`
//!   - `uv_stride = y_stride / 2`
//!   - `border` must be a multiple of 32 (else `-3`).
//!
//! The VP9 entry points (`vpx_alloc_frame_buffer`, `vpx_realloc_frame_buffer`,
//! `vpx_free_frame_buffer`) are gated by `CONFIG_VP9` in the C source and
//! are intentionally omitted from this VP8-only translation.

#![allow(dead_code)]

use core::ffi::c_void;
use core::ptr;

use crate::types::Yv12BufferConfig;

// ---------------------------------------------------------------------------
// Cross-translation-unit dependencies.
// ---------------------------------------------------------------------------

use crate::vpx_mem::{vpx_free, vpx_memalign};

// ---------------------------------------------------------------------------
// `vp8_yv12_de_alloc_frame_buffer` — release a YV12 buffer and zero the
// `YV12_BUFFER_CONFIG`.
//
// Returns 0 on success, -1 if `ybf` is null.
// ---------------------------------------------------------------------------

/// # Safety
/// `ybf` must be null or point to a valid, writable `Yv12BufferConfig`.
/// If `buffer_alloc_sz > 0`, `buffer_alloc` must point to memory
/// previously returned by `vpx_memalign`.

pub unsafe fn vp8_yv12_de_alloc_frame_buffer(ybf: *mut Yv12BufferConfig) -> i32 {
    if !ybf.is_null() {
        // If libvpx is using frame buffer callbacks then buffer_alloc_sz
        // must not be set.
        if (*ybf).buffer_alloc_sz > 0 {
            vpx_free((*ybf).buffer_alloc as *mut c_void);
        }

        // buffer_alloc isn't accessed by most functions. Rather y_buffer,
        // u_buffer and v_buffer point to buffer_alloc and are used. Clear
        // out all of this so that a freed pointer isn't inadvertently used.
        ptr::write_bytes(
            ybf as *mut u8,
            0u8,
            core::mem::size_of::<Yv12BufferConfig>(),
        );
    } else {
        return -1;
    }

    0
}

// ---------------------------------------------------------------------------
// `vp8_yv12_realloc_frame_buffer` — size, allocate (if needed), and lay
// out the planes.
//
// Returns 0 on success, -1 on OOM / undersized existing buffer, -2 if
// `ybf` is null, -3 if `border` is not a multiple of 32.
// ---------------------------------------------------------------------------

/// # Safety
/// `ybf` must be null or point to a valid, writable `Yv12BufferConfig`
/// whose `buffer_alloc` (if non-null) is either NULL or a
/// `vpx_memalign`-produced allocation of at least `buffer_alloc_sz` bytes.

pub unsafe fn vp8_yv12_realloc_frame_buffer(
    ybf: *mut Yv12BufferConfig,
    width: i32,
    height: i32,
    border: i32,
) -> i32 {
    if !ybf.is_null() {
        let aligned_width: i32 = (width + 15) & !15;
        let aligned_height: i32 = (height + 15) & !15;
        let y_stride: i32 = ((aligned_width + 2 * border) + 31) & !31;
        let yplane_size: i32 = (aligned_height + 2 * border) * y_stride;
        let uv_width: i32 = aligned_width >> 1;
        let uv_height: i32 = aligned_height >> 1;
        // There is currently a bunch of code which assumes
        // uv_stride == y_stride/2, so enforce this here.
        let uv_stride: i32 = y_stride >> 1;
        let uvplane_size: i32 = (uv_height + border) * uv_stride;
        let frame_size: usize = (yplane_size + 2 * uvplane_size) as usize;

        if (*ybf).buffer_alloc.is_null() {
            (*ybf).buffer_alloc = vpx_memalign(32, frame_size) as *mut u8;
            if (*ybf).buffer_alloc.is_null() {
                (*ybf).buffer_alloc_sz = 0;
                return -1;
            }
            // msan-only zeroing block from the C source omitted (no
            // __has_feature(memory_sanitizer) gating in Rust here).
            (*ybf).buffer_alloc_sz = frame_size;
        }

        if (*ybf).buffer_alloc_sz < frame_size {
            return -1;
        }

        // Only support allocating buffers that have a border that's a
        // multiple of 32. The border restriction is required to get 16-byte
        // alignment of the start of the chroma rows without introducing an
        // arbitrary gap between planes, which would break the semantics of
        // things like vpx_img_set_rect().
        if border & 0x1f != 0 {
            return -3;
        }

        (*ybf).y_crop_width = width;
        (*ybf).y_crop_height = height;
        (*ybf).y_width = aligned_width;
        (*ybf).y_height = aligned_height;
        (*ybf).y_stride = y_stride;

        (*ybf).uv_crop_width = (width + 1) / 2;
        (*ybf).uv_crop_height = (height + 1) / 2;
        (*ybf).uv_width = uv_width;
        (*ybf).uv_height = uv_height;
        (*ybf).uv_stride = uv_stride;

        (*ybf).alpha_width = 0;
        (*ybf).alpha_height = 0;
        (*ybf).alpha_stride = 0;

        (*ybf).border = border;
        (*ybf).frame_size = frame_size;

        (*ybf).y_buffer = (*ybf)
            .buffer_alloc
            .offset((border * y_stride) as isize)
            .offset(border as isize);
        (*ybf).u_buffer = (*ybf)
            .buffer_alloc
            .offset(yplane_size as isize)
            .offset((border / 2 * uv_stride) as isize)
            .offset((border / 2) as isize);
        (*ybf).v_buffer = (*ybf)
            .buffer_alloc
            .offset(yplane_size as isize)
            .offset(uvplane_size as isize)
            .offset((border / 2 * uv_stride) as isize)
            .offset((border / 2) as isize);
        (*ybf).alpha_buffer = ptr::null_mut();

        (*ybf).corrupted = 0; // assume not currupted by errors
        return 0;
    }
    -2
}

// ---------------------------------------------------------------------------
// `vp8_yv12_alloc_frame_buffer` — fresh allocation: free any existing
// buffer, then realloc from scratch at the requested geometry.
//
// Returns -2 if `ybf` is null, otherwise the result of `realloc`.
// ---------------------------------------------------------------------------

/// # Safety
/// Same requirements as [`vp8_yv12_de_alloc_frame_buffer`] and
/// [`vp8_yv12_realloc_frame_buffer`].

pub unsafe fn vp8_yv12_alloc_frame_buffer(
    ybf: *mut Yv12BufferConfig,
    width: i32,
    height: i32,
    border: i32,
) -> i32 {
    if !ybf.is_null() {
        vp8_yv12_de_alloc_frame_buffer(ybf);
        return vp8_yv12_realloc_frame_buffer(ybf, width, height, border);
    }
    -2
}

// ---------------------------------------------------------------------------
// VP9 entry points (`vpx_alloc_frame_buffer`, `vpx_realloc_frame_buffer`,
// `vpx_free_frame_buffer`) are guarded by `#if CONFIG_VP9` in the C
// source and are not part of the VP8 decoder build. They are
// intentionally not translated; per-translation-task instructions
// retain their C names if/when they are later added.
// ---------------------------------------------------------------------------
