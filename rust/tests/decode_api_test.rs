#![allow(unsafe_op_in_unsafe_fn)]
//! Port of `test/decode_api_test.cc` (gtest) to Rust integration
//! tests. VP9-specific cases are dropped (no VP9 in this crate).

use core::mem::MaybeUninit;

use vp8_decoder_rs::vp8_dx_iface::vpx_codec_vp8_dx;
use vp8_decoder_rs::vpx_api::{
    VPX_CODEC_CAP_HIGHBITDEPTH, VPX_CODEC_INCAPABLE, VPX_CODEC_INVALID_PARAM, VPX_CODEC_OK,
    VPX_CODEC_UNSUP_BITSTREAM, VPX_CODEC_USE_ERROR_CONCEALMENT, VPX_CODEC_USE_INPUT_FRAGMENTS,
    VPX_DECODER_ABI_VERSION, VpxCodecErr, VpxCodecFlags, VpxCodecIface, vpx_codec_ctx_t,
    vpx_codec_dec_cfg_t, vpx_codec_dec_init_ver, vpx_codec_decode, vpx_codec_destroy,
    vpx_codec_error, vpx_codec_error_detail, vpx_codec_get_caps,
};

unsafe fn dec_init(
    ctx: &mut vpx_codec_ctx_t,
    iface: Option<&'static VpxCodecIface>,
    flags: VpxCodecFlags,
) -> VpxCodecErr {
    vpx_codec_dec_init_ver(ctx, iface, None, flags, VPX_DECODER_ABI_VERSION)
}

/// C: `TEST(DecodeAPI, InvalidParams)` — null-pointer arm only. The
/// iface-loop arm is in `invalid_params_via_iface` below.
///
/// The C test also passes `NULL` for `ctx` to `dec_init` / `decode` /
/// `destroy`. Those arms are gone here: the Rust entry points take
/// `&mut VpxCodecCtx`, which can't be null, so a null-`ctx` call is
/// unrepresentable (same reason the null+nonzero `&[u8]` combos
/// dropped out). Only the null-`iface` arm and the `Option<&ctx>`
/// error queries remain exercisable.
#[test]
fn invalid_params_null_ptrs() {
    let mut dec_storage = MaybeUninit::<vpx_codec_ctx_t>::zeroed();

    unsafe {
        // Valid ctx, null iface → INVALID_PARAM.
        assert_eq!(
            dec_init(dec_storage.assume_init_mut(), None, 0),
            VPX_CODEC_INVALID_PARAM
        );

        // Error queries still accept `None` and return a fallback
        // description.
        assert!(!vpx_codec_error(None).is_empty());
        assert!(!vpx_codec_error_detail(None).is_empty());
    }
}

/// C: `TEST(DecodeAPI, HighBitDepthCapability)`.
#[test]
fn high_bit_depth_capability() {
    let vp8_iface = vpx_codec_vp8_dx();
    assert_eq!(
        vpx_codec_get_caps(Some(vp8_iface)) & VPX_CODEC_CAP_HIGHBITDEPTH,
        0,
        "VP8 must not advertise HighBitDepth capability"
    );
}

/// C: `TEST(DecodeAPI, InvalidParams)` — iface-loop arm. C iterates
/// `kCodecs[]` which, in this build, contains only VP8.
#[test]
fn invalid_params_via_iface() {
    unsafe {
        let iface = vpx_codec_vp8_dx();
        let mut dec = MaybeUninit::<vpx_codec_ctx_t>::zeroed();
        let buf = [0u8; 1];

        assert_eq!(dec_init(dec.assume_init_mut(), Some(iface), 0), VPX_CODEC_OK);
        assert_eq!(
            vpx_codec_decode(dec.assume_init_mut(), &buf, core::ptr::null_mut(), 0),
            VPX_CODEC_UNSUP_BITSTREAM
        );
        // Empty buffer is the only "no data" shape now (the null+nonzero
        // and nonzero+null combos are unrepresentable through `&[u8]`).
        assert_eq!(
            vpx_codec_decode(dec.assume_init_mut(), &[], core::ptr::null_mut(), 0),
            VPX_CODEC_OK
        );
        assert_eq!(vpx_codec_destroy(dec.assume_init_mut()), VPX_CODEC_OK);
    }
}

/// C: `TEST(DecodeAPI, OptionalParams)` — CONFIG_ERROR_CONCEALMENT is 0
/// in the minimal Rust build, so init with the flag must return INCAPABLE.
#[test]
fn optional_params() {
    unsafe {
        let iface = vpx_codec_vp8_dx();
        let mut dec = MaybeUninit::<vpx_codec_ctx_t>::zeroed();

        assert_eq!(
            dec_init(
                dec.assume_init_mut(),
                Some(iface),
                VPX_CODEC_USE_ERROR_CONCEALMENT
            ),
            VPX_CODEC_INCAPABLE
        );
    }
}

/// C: `TEST(DecodeAPI, Vp8FlushWithNoFragments)`.
#[test]
fn vp8_flush_with_no_fragments() {
    unsafe {
        let iface = vpx_codec_vp8_dx();
        let mut dec = MaybeUninit::<vpx_codec_ctx_t>::zeroed();
        let cfg = vpx_codec_dec_cfg_t {
            threads: 1,
            w: 0,
            h: 0,
        };
        let flags: VpxCodecFlags = VPX_CODEC_USE_INPUT_FRAGMENTS;

        assert_eq!(
            vpx_codec_dec_init_ver(
                dec.assume_init_mut(),
                Some(iface),
                Some(&cfg),
                flags,
                VPX_DECODER_ABI_VERSION
            ),
            VPX_CODEC_OK
        );
        assert_eq!(
            vpx_codec_decode(dec.assume_init_mut(), &[], core::ptr::null_mut(), 0),
            VPX_CODEC_OK
        );
        assert_eq!(vpx_codec_destroy(dec.assume_init_mut()), VPX_CODEC_OK);
    }
}
