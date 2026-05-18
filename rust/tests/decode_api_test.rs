#![allow(unsafe_op_in_unsafe_fn)]
//! Port of `test/decode_api_test.cc` (gtest) to Rust integration
//! tests. VP9-specific cases are dropped (no VP9 in this crate).

use core::mem::MaybeUninit;
use core::ptr;

use vp8_decoder_rs::vp8_dx_iface::vpx_codec_vp8_dx;
use vp8_decoder_rs::vpx_api::{
    vpx_codec_ctx_t, vpx_codec_dec_cfg_t, vpx_codec_dec_init_ver, vpx_codec_decode,
    vpx_codec_destroy, vpx_codec_error, vpx_codec_error_detail, vpx_codec_get_caps,
    vpx_codec_iface_t, VpxCodecCaps, VpxCodecErr, VpxCodecFlags, VPX_CODEC_CAP_HIGHBITDEPTH,
    VPX_CODEC_INCAPABLE, VPX_CODEC_INVALID_PARAM, VPX_CODEC_OK, VPX_CODEC_UNSUP_BITSTREAM,
    VPX_CODEC_USE_ERROR_CONCEALMENT, VPX_CODEC_USE_INPUT_FRAGMENTS, VPX_DECODER_ABI_VERSION,
};

unsafe fn dec_init(
    ctx: *mut vpx_codec_ctx_t,
    iface: *mut vpx_codec_iface_t,
    flags: VpxCodecFlags,
) -> VpxCodecErr {
    vpx_codec_dec_init_ver(ctx, iface, ptr::null_mut(), flags, VPX_DECODER_ABI_VERSION)
}

/// C: `TEST(DecodeAPI, InvalidParams)` — null-pointer arm only. The
/// iface-loop arm is in `invalid_params_via_iface` below.
#[test]
fn invalid_params_null_ptrs() {
    let mut buf = [0u8; 1];
    let mut dec = MaybeUninit::<vpx_codec_ctx_t>::uninit();

    unsafe {
        assert_eq!(dec_init(ptr::null_mut(), ptr::null_mut(), 0), VPX_CODEC_INVALID_PARAM);
        assert_eq!(dec_init(dec.as_mut_ptr(), ptr::null_mut(), 0), VPX_CODEC_INVALID_PARAM);

        assert_eq!(
            vpx_codec_decode(ptr::null_mut(), ptr::null(), 0, ptr::null_mut(), 0),
            VPX_CODEC_INVALID_PARAM
        );
        assert_eq!(
            vpx_codec_decode(ptr::null_mut(), buf.as_mut_ptr(), 0, ptr::null_mut(), 0),
            VPX_CODEC_INVALID_PARAM
        );
        assert_eq!(
            vpx_codec_decode(
                ptr::null_mut(),
                buf.as_mut_ptr(),
                buf.len() as u32,
                ptr::null_mut(),
                0
            ),
            VPX_CODEC_INVALID_PARAM
        );
        assert_eq!(
            vpx_codec_decode(
                ptr::null_mut(),
                ptr::null(),
                buf.len() as u32,
                ptr::null_mut(),
                0
            ),
            VPX_CODEC_INVALID_PARAM
        );
        assert_eq!(vpx_codec_destroy(ptr::null_mut()), VPX_CODEC_INVALID_PARAM);

        // vpx_codec_error(null) returns "<invalid interface>" / generic message,
        // never null.
        assert!(!vpx_codec_error(ptr::null()).is_null());
        // vpx_codec_error_detail(null) returns null.
        assert!(vpx_codec_error_detail(ptr::null()).is_null());
    }
}

/// C: `TEST(DecodeAPI, HighBitDepthCapability)`.
#[test]
fn high_bit_depth_capability() {
    let vp8_iface = unsafe { vpx_codec_vp8_dx() };
    let vp8_caps: VpxCodecCaps =
        unsafe { vpx_codec_get_caps(vp8_iface as *mut vpx_codec_iface_t) };
    assert_eq!(vp8_caps & VPX_CODEC_CAP_HIGHBITDEPTH, 0,
        "VP8 must not advertise HighBitDepth capability");
}

/// C: `TEST(DecodeAPI, InvalidParams)` — iface-loop arm. C iterates
/// `kCodecs[]` which, in this build, contains only VP8.
#[test]
fn invalid_params_via_iface() {
    unsafe {
        let iface = vpx_codec_vp8_dx() as *mut vpx_codec_iface_t;
        let mut dec = MaybeUninit::<vpx_codec_ctx_t>::uninit();
        let buf = [0u8; 1];

        assert_eq!(dec_init(ptr::null_mut(), iface, 0), VPX_CODEC_INVALID_PARAM);
        assert_eq!(dec_init(dec.as_mut_ptr(), iface, 0), VPX_CODEC_OK);
        assert_eq!(
            vpx_codec_decode(dec.as_mut_ptr(), buf.as_ptr(), 1, ptr::null_mut(), 0),
            VPX_CODEC_UNSUP_BITSTREAM
        );
        assert_eq!(
            vpx_codec_decode(dec.as_mut_ptr(), ptr::null(), 1, ptr::null_mut(), 0),
            VPX_CODEC_INVALID_PARAM
        );
        assert_eq!(
            vpx_codec_decode(dec.as_mut_ptr(), buf.as_ptr(), 0, ptr::null_mut(), 0),
            VPX_CODEC_INVALID_PARAM
        );
        assert_eq!(vpx_codec_destroy(dec.as_mut_ptr()), VPX_CODEC_OK);
    }
}

/// C: `TEST(DecodeAPI, OptionalParams)` — CONFIG_ERROR_CONCEALMENT is 0
/// in the minimal Rust build, so init with the flag must return INCAPABLE.
#[test]
fn optional_params() {
    unsafe {
        let iface = vpx_codec_vp8_dx() as *mut vpx_codec_iface_t;
        let mut dec = MaybeUninit::<vpx_codec_ctx_t>::uninit();

        assert_eq!(
            dec_init(dec.as_mut_ptr(), iface, VPX_CODEC_USE_ERROR_CONCEALMENT),
            VPX_CODEC_INCAPABLE
        );
    }
}

/// C: `TEST(DecodeAPI, Vp8FlushWithNoFragments)`.
#[test]
fn vp8_flush_with_no_fragments() {
    unsafe {
        let iface = vpx_codec_vp8_dx() as *mut vpx_codec_iface_t;
        let mut dec = MaybeUninit::<vpx_codec_ctx_t>::uninit();
        let cfg = vpx_codec_dec_cfg_t { threads: 1, w: 0, h: 0 };
        let flags: VpxCodecFlags = VPX_CODEC_USE_INPUT_FRAGMENTS;

        assert_eq!(
            vpx_codec_dec_init_ver(
                dec.as_mut_ptr(),
                iface,
                &cfg,
                flags,
                VPX_DECODER_ABI_VERSION
            ),
            VPX_CODEC_OK
        );
        assert_eq!(
            vpx_codec_decode(dec.as_mut_ptr(), ptr::null(), 0, ptr::null_mut(), 0),
            VPX_CODEC_OK
        );
        assert_eq!(vpx_codec_destroy(dec.as_mut_ptr()), VPX_CODEC_OK);
    }
}
