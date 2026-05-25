//! vp8-decoder-rs — Rust port of the libvpx VP8 decoder.
//!
//! The kernel modules (vp8/common/, vp8/decoder/, vpx_dsp/, vpx_mem/,
//! vpx_scale/, vpx_util/) mirror the libvpx C source layout, one file
//! per module.
//!
//! The public API surface ([`codec`], [`vpx_api`], [`vpx_codec`],
//! [`vpx_decoder`], [`vp8_dx_iface`]) is shaped around the
//! [`Decoder`](codec::Decoder) trait.

#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(static_mut_refs)]
#![allow(unsafe_op_in_unsafe_fn)]

// ---- Foundations (no inter-module deps) ----
pub mod api;
pub mod codec;
pub mod tables;
pub mod types;
pub mod vp8_cx_stub;
pub mod vp9_dx_stub;
#[cfg(test)]
pub mod api_tests;


// ---- vp8/common/ ----
pub mod alloccommon;
pub mod blockd;
pub mod dequantize;
pub mod entropy;
pub mod entropymode;
pub mod entropymv;
pub mod extend;
pub mod filter;
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
pub mod vpx_dsp_rtcd;

// ---- vpx_mem/ ----
pub mod vpx_mem;

// ---- vpx_scale/ ----
pub mod vpx_scale_rtcd;
pub mod yv12config;
pub mod yv12extend;

// ---- vpx_util/ ----
pub mod vpx_thread;

// ---- vpx_ports/ (header-only shims) ----
pub mod vpx_ports;
