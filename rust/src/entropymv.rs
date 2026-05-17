//! `vp8/common/entropymv.c` — default MV-component probability tables.
//!
//! The original translation unit contains nothing but two constant
//! `MV_CONTEXT[2]` arrays: `vp8_mv_update_probs` (the prior used to
//! entropy-code per-frame MV-probability updates) and
//! `vp8_default_mv_context` (the initial MV-component probabilities a
//! fresh decoder / key-frame reset starts from). Both tables live in
//! `crate::tables` (where the rest of the precomputed VP8 constants
//! sit); this module simply re-exports them under their original C
//! names so downstream code that mirrors the C sources can refer to
//! them verbatim. See `documentation/vp8_files/entropymv.md` and
//! RFC 6386 §17.1–§17.2 for the layout and semantics.

#![allow(non_upper_case_globals)]

pub use crate::tables::VP8_DEFAULT_MV_CONTEXT as vp8_default_mv_context;
pub use crate::tables::VP8_MV_UPDATE_PROBS as vp8_mv_update_probs;
