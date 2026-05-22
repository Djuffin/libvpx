//! `vp8/common/entropy.c` — coefficient-token entropy data.
//!
//! Every table referenced here lives in [`crate::tables`] and is
//! re-exported under its original C name (`pub use crate::tables::X as
//! vp8_x`) so call-sites keep using the libvpx identifiers.

#![allow(dead_code)]
#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]

use crate::types::Vp8Common;

// ---------------------------------------------------------------------------
// Coefficient-token alphabet (`entropy.h:23–34`).
// ---------------------------------------------------------------------------

/// `ZERO_TOKEN` — magnitude 0. RFC 6386 §13.2.
pub const ZERO_TOKEN: i32 = 0;
/// `ONE_TOKEN` — magnitude 1.
pub const ONE_TOKEN: i32 = 1;
/// `TWO_TOKEN` — magnitude 2.
pub const TWO_TOKEN: i32 = 2;
/// `THREE_TOKEN` — magnitude 3.
pub const THREE_TOKEN: i32 = 3;
/// `FOUR_TOKEN` — magnitude 4.
pub const FOUR_TOKEN: i32 = 4;
/// `DCT_VAL_CATEGORY1` — magnitudes 5–6 (1 extra bit).
pub const DCT_VAL_CATEGORY1: i32 = 5;
/// `DCT_VAL_CATEGORY2` — magnitudes 7–10 (2 extra bits).
pub const DCT_VAL_CATEGORY2: i32 = 6;
/// `DCT_VAL_CATEGORY3` — magnitudes 11–18 (3 extra bits).
pub const DCT_VAL_CATEGORY3: i32 = 7;
/// `DCT_VAL_CATEGORY4` — magnitudes 19–34 (4 extra bits).
pub const DCT_VAL_CATEGORY4: i32 = 8;
/// `DCT_VAL_CATEGORY5` — magnitudes 35–66 (5 extra bits).
pub const DCT_VAL_CATEGORY5: i32 = 9;
/// `DCT_VAL_CATEGORY6` — magnitudes 67+ (11 extra bits).
pub const DCT_VAL_CATEGORY6: i32 = 10;
/// `DCT_EOB_TOKEN` — end-of-block sentinel.
pub const DCT_EOB_TOKEN: i32 = 11;

// Re-exports of bitstream-fixed constants declared in `crate::tables`.

pub use crate::tables::DCT_MAX_VALUE;
pub use crate::tables::{
    BLOCK_TYPES, COEF_BANDS, ENTROPY_NODES, MAX_ENTROPY_TOKENS, PREV_COEF_CONTEXTS,
};

// ---------------------------------------------------------------------------
// Re-exports of the tables defined in `entropy.c`.
// ---------------------------------------------------------------------------

/// `vp8_coef_bands[16]` — zig-zag position → coefficient band.
pub use crate::tables::VP8_COEF_BANDS as vp8_coef_bands;
/// `vp8_coef_tree[22]` — coefficient-token decoding tree.
pub use crate::tables::VP8_COEF_TREE as vp8_coef_tree;
/// `vp8_default_inv_zig_zag[16]` — inverse zig-zag (raster → scan + 1).
pub use crate::tables::VP8_DEFAULT_INV_ZIG_ZAG as vp8_default_inv_zig_zag;
/// `vp8_default_zig_zag_mask[16]` — bit mask form of the inverse zig-zag.
pub use crate::tables::VP8_DEFAULT_ZIG_ZAG_MASK as vp8_default_zig_zag_mask;
/// `vp8_default_zig_zag1d[16]` — forward zig-zag (scan → raster).
pub use crate::tables::VP8_DEFAULT_ZIG_ZAG1D as vp8_default_zig_zag1d;
/// `vp8_extra_bits[12]` — per-token extra-bit dispatch table.
pub use crate::tables::VP8_EXTRA_BITS as vp8_extra_bits;
/// `vp8_mb_feature_data_bits[MB_LVL_MAX]` — bit-widths of segment features.
pub use crate::tables::VP8_MB_FEATURE_DATA_BITS as vp8_mb_feature_data_bits;
/// `vp8_norm[256]` — bool-decoder renormalisation LUT.
pub use crate::tables::VP8_NORM as vp8_norm;
/// `vp8_prev_token_class[12]` — token value → previous-coefficient context.
pub use crate::tables::VP8_PREV_TOKEN_CLASS as vp8_prev_token_class;

/// `default_coef_probs` — static initial coefficient-probability cube
/// (`default_coef_probs.h`).
pub use crate::tables::DEFAULT_COEF_PROBS as default_coef_probs;

// ---------------------------------------------------------------------------
// Functions.
// ---------------------------------------------------------------------------

/// `vp8_default_coef_probs` (`entropy.c:145`) — reset the frame-context
/// coefficient probs to their RFC 6386 default values.
pub fn vp8_default_coef_probs(pc: &mut Vp8Common) {
    pc.fc.coef_probs = default_coef_probs;
}
