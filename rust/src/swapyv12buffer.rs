//! YV12 buffer pointer swap (`vp8/common/swapyv12buffer.c`).
//!
//! One public entry point, [`vp8_swap_yv12_buffer`], swaps the owned
//! backing buffer and plane regions of two [`Yv12BufferConfig`]
//! descriptors. No pixel data is moved and no dimension or stride field
//! is touched.

use crate::types::Yv12BufferConfig;

/// `vp8_swap_yv12_buffer` (vp8/common/swapyv12buffer.c:13).
///
/// Exchanges the `owning_buffer` and the `y`/`u`/`v_region` fields of
/// `new_frame` and `last_frame`. Width/height/stride/border and the
/// dimension fields are left in place; the caller is expected to have
/// allocated both buffers with identical dimensions.
pub fn vp8_swap_yv12_buffer(new_frame: &mut Yv12BufferConfig, last_frame: &mut Yv12BufferConfig) {
    // The owned buffer and the regions pointing into it move together.
    core::mem::swap(&mut last_frame.owning_buffer, &mut new_frame.owning_buffer);
    core::mem::swap(&mut last_frame.y_region, &mut new_frame.y_region);
    core::mem::swap(&mut last_frame.u_region, &mut new_frame.u_region);
    core::mem::swap(&mut last_frame.v_region, &mut new_frame.v_region);
}
