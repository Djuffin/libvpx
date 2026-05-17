//! `vp8/common/modecont.c` — default mode-context probability table for
//! inter macroblocks.
//!
//! The C translation unit holds a single read-only constant,
//! `vp8_mode_contexts[6][4]`: the byte-sized mv-ref tree probabilities
//! indexed by `[near_mv_ref_ct slot][mv_ref_tree split]`. See
//! `documentation/vp8_files/modecont.md` and RFC 6386 §16.3.
//!
//! The table itself already lives in `tables.rs` as
//! [`VP8_MODE_CONTEXTS`]; this module re-exports it under the original
//! C name so call sites can use the libvpx spelling unchanged.

pub use crate::tables::VP8_MODE_CONTEXTS as vp8_mode_contexts;
