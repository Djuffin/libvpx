//! vp8-decoder-rs — Rust port of the libvpx VP8 decoder.
//!
//! Module layout mirrors the C source tree (`vp8/common/`, `vp8/decoder/`,
//! `vpx/src/`, `vpx_dsp/`, `vpx_mem/`, `vpx_scale/`, `vpx_util/`). Each
//! module is a literal transliteration of one `.c` file; cross-file
//! references are stitched via `extern "Rust"` declarations until the
//! whole tree is wired up.

#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_assignments)]
#![allow(unused_mut)]
#![allow(unused_unsafe)]
#![allow(static_mut_refs)]

// ---- Foundations (no inter-module deps) ----
pub mod tables;
pub mod types;

// ---- vp8/common/ ----
pub mod alloccommon;
pub mod blockd;
pub mod dequantize;
pub mod entropy;
pub mod entropymode;
pub mod entropymv;
pub mod extend;
pub mod filter;
pub mod findnearmv;
pub mod idct_blk;
pub mod idctllm;
pub mod loopfilter_filters;
pub mod mbpitch;
pub mod modecont;
pub mod quant_common;
pub mod reconinter;
pub mod reconintra;
pub mod reconintra4x4;
pub mod rtcd;
pub mod setupintrarecon;
pub mod swapyv12buffer;
pub mod systemdependent;
pub mod treecoder;
pub mod vp8_loopfilter;

// ---- vp8/decoder/ ----
pub mod dboolhuff;
pub mod decodeframe;
pub mod decodemv;
pub mod detokenize;
pub mod onyxd_if;
pub mod treereader;

// ---- vp8_only/ generated RTCD ----
pub mod vp8_rtcd;

// ---- vp8/ (codec interface) ----
pub mod vp8_dx_iface;

// ---- vpx/src/ (public API dispatcher) ----
pub mod vpx_api;
pub mod vpx_codec;
pub mod vpx_decoder;
pub mod vpx_encoder;
pub mod vpx_image;

// ---- vpx_dsp/ ----
pub mod intrapred;
pub mod prob;
pub mod vpx_dsp_rtcd;

// ---- vpx_mem/ ----
pub mod vpx_mem;

// ---- vpx_scale/ ----
pub mod vpx_scale_rtcd;
pub mod yv12config;
pub mod yv12extend;

// ---- vpx_util/ ----
pub mod vpx_thread;
pub mod vpx_write_yuv_frame;

// ---- vpx_ports/ (header-only shims) ----
pub mod vpx_ports;

// ---- generated ----
pub mod vpx_config;
