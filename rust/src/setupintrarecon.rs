//! Intra-prediction border seeding (`vp8/common/setupintrarecon.c`).
//!
//! Literal Rust transliteration of libvpx's `setupintrarecon.c` /
//! `setupintrarecon.h`. The decoder uses
//! [`vp8_setup_intra_recon_top_line`] (called once per frame) plus the
//! `static INLINE` helper [`setup_intra_recon_left`] from the header
//! (called per MB row). The full-frame encoder-side `vp8_setup_intra_recon`
//! was pruned in the dead-code cleanup.
//!
//! Both routines write the synthetic boundary samples that VP8 intra
//! prediction reads when no real neighbour exists: `127` along the row
//! above each plane (and the above-left corner) and `129` down the
//! column to the left of each plane. The asymmetry is engineered so
//! that every intra mode degenerates to the neutral grey value `128`
//! when fed only synthetic samples — see
//! `documentation/vp8_files/setupintrarecon.md` for the derivation.

use core::ptr::write_bytes;

use crate::tables::{INTRA_RECON_ABOVE_SEED, INTRA_RECON_LEFT_SEED};
use crate::types::Yv12BufferConfig;

/// `vp8_setup_intra_recon_top_line` (vp8/common/setupintrarecon.c:34).
///
/// Seeds only the above row (and above-left corner / right-reach
/// padding) for each of the three planes. Called once per frame by
/// the decoder; the per-MB-row left-column seed is handled by
/// [`setup_intra_recon_left`].
pub unsafe fn vp8_setup_intra_recon_top_line(ybf: *mut Yv12BufferConfig) {
    write_bytes(
        (*ybf).y_buffer().offset(-1 - (*ybf).y_stride as isize),
        INTRA_RECON_ABOVE_SEED,
        ((*ybf).y_width + 5) as usize,
    );
    write_bytes(
        (*ybf).u_buffer().offset(-1 - (*ybf).uv_stride as isize),
        INTRA_RECON_ABOVE_SEED,
        ((*ybf).uv_width + 5) as usize,
    );
    write_bytes(
        (*ybf).v_buffer().offset(-1 - (*ybf).uv_stride as isize),
        INTRA_RECON_ABOVE_SEED,
        ((*ybf).uv_width + 5) as usize,
    );
}

/// `setup_intra_recon_left` (vp8/common/setupintrarecon.h:23,
/// `static INLINE`).
///
/// Writes the `129` left-column seed for one MB row: 16 rows of
/// luma + 8 rows each of U and V. Callers pre-offset the pointers
/// to address the byte one column to the left of the leftmost MB
/// of the current row.
#[inline]
pub unsafe fn setup_intra_recon_left(
    y_buffer: *mut u8,
    u_buffer: *mut u8,
    v_buffer: *mut u8,
    y_stride: i32,
    uv_stride: i32,
) {
    for i in 0..16 {
        *y_buffer.offset((y_stride * i) as isize) = INTRA_RECON_LEFT_SEED;
    }

    for i in 0..8 {
        *u_buffer.offset((uv_stride * i) as isize) = INTRA_RECON_LEFT_SEED;
    }

    for i in 0..8 {
        *v_buffer.offset((uv_stride * i) as isize) = INTRA_RECON_LEFT_SEED;
    }
}
