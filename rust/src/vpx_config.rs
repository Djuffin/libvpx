// Translation of vp8_only/vpx_config.c
//
// Original C source contains exactly two declarations:
//   static const char* const cfg = "...";
//   const char *vpx_codec_build_config(void) {return cfg;}
//
// The configuration string itself already lives in `tables.rs` as
// `VPX_CODEC_BUILD_CONFIG`; this module re-exports it and provides the
// public accessor that mirrors the C ABI entry point.

pub use crate::tables::VPX_CODEC_BUILD_CONFIG;

/// Return the build configuration.
///
/// Returns a printable string containing an encoded version of the build
/// configuration. This may be useful to vpx support.
///
/// Faithful translation of:
/// ```c
/// const char *vpx_codec_build_config(void) {return cfg;}
/// ```
pub fn vpx_codec_build_config() -> &'static str {
    VPX_CODEC_BUILD_CONFIG
}
