//! Translation of `vpx_scale/generic/yv12config.c`.
//!
//! Allocator, deallocator, and (re)allocator for [`Yv12BufferConfig`] — the
//! single contiguous YV12 frame-buffer slab used everywhere in the VP8
//! decoder for reference pictures and reconstruction targets. The three
//! VP8 entry points (`vp8_yv12_alloc_frame_buffer`,
//! `vp8_yv12_realloc_frame_buffer`, `vp8_yv12_de_alloc_frame_buffer`) take
//! `&mut Yv12BufferConfig`. The slab is an owned `Box<[u8]>`; the Y/U/V
//! plane regions are subslices of it, each spanning its plane plus border,
//! so the visible origin sits `border` rows/cols in from the region start.
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

use core::ptr::NonNull;

use crate::types::Yv12BufferConfig;

// ---------------------------------------------------------------------------
// `vp8_yv12_de_alloc_frame_buffer` — release a YV12 buffer and reset the
// `YV12_BUFFER_CONFIG` to an empty config.
// ---------------------------------------------------------------------------

pub fn vp8_yv12_de_alloc_frame_buffer(ybf: &mut Yv12BufferConfig) {
    *ybf = Yv12BufferConfig::default();
}

// ---------------------------------------------------------------------------
// `vp8_yv12_realloc_frame_buffer` — size, allocate (if needed), and lay
// out the planes.
//
// Returns 0 on success, -1 on undersized existing buffer, -3 if `border`
// is not a multiple of 32.
// ---------------------------------------------------------------------------

pub fn vp8_yv12_realloc_frame_buffer(
    ybf: &mut Yv12BufferConfig,
    width: i32,
    height: i32,
    border: i32,
) -> i32 {
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

    if ybf.owning_buffer.is_none() {
        ybf.owning_buffer = Some(vec![0u8; frame_size].into_boxed_slice());
    }

    if ybf.owning_buffer.as_ref().map_or(0, |s| s.len()) < frame_size {
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

    ybf.y_crop_width = width;
    ybf.y_crop_height = height;
    ybf.y_width = aligned_width;
    ybf.y_height = aligned_height;
    ybf.y_stride = y_stride;

    ybf.uv_crop_width = (width + 1) / 2;
    ybf.uv_crop_height = (height + 1) / 2;
    ybf.uv_width = uv_width;
    ybf.uv_height = uv_height;
    ybf.uv_stride = uv_stride;

    ybf.alpha_width = 0;
    ybf.alpha_height = 0;
    ybf.alpha_stride = 0;

    ybf.border = border;
    ybf.frame_size = frame_size;

    // Each plane region is a contiguous subslice of the slab; the visible
    // origin sits `border` rows/cols in from its start.
    let (yp, uvp) = (yplane_size as usize, uvplane_size as usize);
    let (y, u, v) = {
        let buf = ybf.owning_buffer.as_mut().unwrap();
        let y = NonNull::from(&mut buf[..yp]);
        let u = NonNull::from(&mut buf[yp..yp + uvp]);
        let v = NonNull::from(&mut buf[yp + uvp..yp + 2 * uvp]);
        (y, u, v)
    };
    ybf.y_region = Some(y);
    ybf.u_region = Some(u);
    ybf.v_region = Some(v);
    ybf.alpha_region = None;

    ybf.corrupted = 0; // assume not currupted by errors
    0
}

// ---------------------------------------------------------------------------
// `vp8_yv12_alloc_frame_buffer` — fresh allocation: free any existing
// buffer, then realloc from scratch at the requested geometry.
// ---------------------------------------------------------------------------

pub fn vp8_yv12_alloc_frame_buffer(
    ybf: &mut Yv12BufferConfig,
    width: i32,
    height: i32,
    border: i32,
) -> i32 {
    vp8_yv12_de_alloc_frame_buffer(ybf);
    vp8_yv12_realloc_frame_buffer(ybf, width, height, border)
}

// ---------------------------------------------------------------------------
// VP9 entry points (`vpx_alloc_frame_buffer`, `vpx_realloc_frame_buffer`,
// `vpx_free_frame_buffer`) are guarded by `#if CONFIG_VP9` in the C
// source and are intentionally omitted from this VP8-only translation.
// ---------------------------------------------------------------------------

pub fn vp8_yv12_alloc_external_frame_buffer(
    ybf: &mut Yv12BufferConfig,
    width: i32,
    height: i32,
    border: i32,
    allocator: &dyn crate::api::VideoFrameAllocator,
) -> Result<(), crate::api::AllocError> {
    vp8_yv12_de_alloc_frame_buffer(ybf);

    let aligned_width: i32 = (width + 15) & !15;
    let aligned_height: i32 = (height + 15) & !15;
    let y_stride: i32 = ((aligned_width + 2 * border) + 31) & !31;
    let yplane_size: i32 = (aligned_height + 2 * border) * y_stride;
    let uv_width: i32 = aligned_width >> 1;
    let uv_height: i32 = aligned_height >> 1;
    let uv_stride: i32 = y_stride >> 1;
    let uvplane_size: i32 = (uv_height + border) * uv_stride;
    let frame_size: usize = (yplane_size + 2 * uvplane_size) as usize;

    if border & 0x1f != 0 {
        return Err(crate::api::AllocError::UnsupportedAlignment);
    }

    let req = crate::api::BufferAllocation {
        planes: [
            Some(crate::api::PlaneAllocation {
                plane: crate::api::VideoPlane::Y,
                size_bytes: yplane_size as usize,
                alignment: 32,
            }),
            Some(crate::api::PlaneAllocation {
                plane: crate::api::VideoPlane::U,
                size_bytes: uvplane_size as usize,
                alignment: 32,
            }),
            Some(crate::api::PlaneAllocation {
                plane: crate::api::VideoPlane::V,
                size_bytes: uvplane_size as usize,
                alignment: 32,
            }),
            None,
        ],
    };

    let buffer = allocator.alloc_frame(&req)?;
    let ext_buffer: std::sync::Arc<dyn crate::api::FrameBuffer> = std::sync::Arc::from(buffer);

    ybf.y_crop_width = width;
    ybf.y_crop_height = height;
    ybf.y_width = aligned_width;
    ybf.y_height = aligned_height;
    ybf.y_stride = y_stride;

    ybf.uv_crop_width = (width + 1) / 2;
    ybf.uv_crop_height = (height + 1) / 2;
    ybf.uv_width = uv_width;
    ybf.uv_height = uv_height;
    ybf.uv_stride = uv_stride;

    ybf.alpha_width = 0;
    ybf.alpha_height = 0;
    ybf.alpha_stride = 0;

    ybf.border = border;
    ybf.frame_size = frame_size;

    let y_ptr = ext_buffer.plane_ptr(crate::api::VideoPlane::Y).ok_or(crate::api::AllocError::OutOfMemory)?;
    let u_ptr = ext_buffer.plane_ptr(crate::api::VideoPlane::U).ok_or(crate::api::AllocError::OutOfMemory)?;
    let v_ptr = ext_buffer.plane_ptr(crate::api::VideoPlane::V).ok_or(crate::api::AllocError::OutOfMemory)?;

    // SAFETY: the allocator guarantees that the returned pointers are valid and aligned.
    unsafe {
        ybf.y_region = Some(NonNull::new(std::slice::from_raw_parts_mut(y_ptr.as_ptr(), yplane_size as usize)).unwrap());
        ybf.u_region = Some(NonNull::new(std::slice::from_raw_parts_mut(u_ptr.as_ptr(), uvplane_size as usize)).unwrap());
        ybf.v_region = Some(NonNull::new(std::slice::from_raw_parts_mut(v_ptr.as_ptr(), uvplane_size as usize)).unwrap());
    }
    ybf.alpha_region = None;
    ybf.ext_buffer = Some(ext_buffer);

    ybf.corrupted = 0;
    Ok(())
}

