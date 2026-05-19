//! YV12 buffer pointer swap (`vp8/common/swapyv12buffer.c`).
//!
//! Literal Rust transliteration of libvpx's `swapyv12buffer.c`. One
//! public entry point, [`vp8_swap_yv12_buffer`], performs an in-place
//! pointer exchange on the four heap-pointer fields of two
//! [`Yv12BufferConfig`] descriptors (`buffer_alloc`, `y_buffer`,
//! `u_buffer`, `v_buffer`). No pixel data is moved, no dimension or
//! stride field is touched — see `documentation/vp8_files/swapyv12buffer.md`
//! for the invariants relied upon by the caller.

use crate::types::Yv12BufferConfig;

/// `vp8_swap_yv12_buffer` (vp8/common/swapyv12buffer.c:13).
///
/// Exchanges the four heap pointers (`buffer_alloc`, `y_buffer`,
/// `u_buffer`, `v_buffer`) of `new_frame` and `last_frame` using a
/// single stack-local scratch. Width/height/stride/border and the
/// `corrupted` flag are intentionally left in place; the caller is
/// expected to have allocated both buffers with identical dimensions.
/// The `&mut` signature statically prevents aliasing the two frames.
pub fn vp8_swap_yv12_buffer(new_frame: &mut Yv12BufferConfig, last_frame: &mut Yv12BufferConfig) {
    core::mem::swap(&mut last_frame.buffer_alloc, &mut new_frame.buffer_alloc);
    core::mem::swap(&mut last_frame.y_buffer, &mut new_frame.y_buffer);
    core::mem::swap(&mut last_frame.u_buffer, &mut new_frame.u_buffer);
    core::mem::swap(&mut last_frame.v_buffer, &mut new_frame.v_buffer);
}
