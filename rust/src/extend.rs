//! Frame-border extension (`vp8/common/extend.c`).
//!
//! Literal Rust transliteration of libvpx's `extend.c`. The decoder
//! only ever calls [`vp8_extend_mb_row`]; encoder-side helpers were
//! pruned in the dead-code cleanup.

#![allow(clippy::too_many_arguments)]

use crate::types::Yv12BufferConfig;

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
    y_ptr = y_ptr.offset(((*ybf).y_stride * 14) as isize);
    u_ptr = u_ptr.offset(((*ybf).uv_stride * 6) as isize);
    v_ptr = v_ptr.offset(((*ybf).uv_stride * 6) as isize);

    for i in 0..4 {
        *y_ptr.offset(i) = *y_ptr.offset(-1);
        *u_ptr.offset(i) = *u_ptr.offset(-1);
        *v_ptr.offset(i) = *v_ptr.offset(-1);
    }

    y_ptr = y_ptr.offset((*ybf).y_stride as isize);
    u_ptr = u_ptr.offset((*ybf).uv_stride as isize);
    v_ptr = v_ptr.offset((*ybf).uv_stride as isize);

    for i in 0..4 {
        *y_ptr.offset(i) = *y_ptr.offset(-1);
        *u_ptr.offset(i) = *u_ptr.offset(-1);
        *v_ptr.offset(i) = *v_ptr.offset(-1);
    }
}
