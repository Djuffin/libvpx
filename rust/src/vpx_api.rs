//! Consolidated public API surface — types and constants spanning
//! the `vpx/vpx_codec.h`, `vpx/vpx_decoder.h`, `vpx/vpx_encoder.h`,
//! `vpx/vpx_image.h`, and `vpx/internal/vpx_codec_internal.h`
//! headers. Re-exports the entry points from the consumer modules
//! (`vpx_codec`, `vpx_decoder`, `vpx_encoder`, `vpx_image`,
//! `vp8_dx_iface`) so callers can `use crate::vpx_api::*;`.
//!
//! Lowercase aliases (`vpx_codec_err_t`, `vpx_image_t`, …) exist
//! alongside the CamelCase Rust spellings for naming symmetry with
//! the original C headers.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use core::ffi::{c_int, c_long, c_uint, c_ulong, c_void};

// ===========================================================================
// `vpx_codec_err_t` (`vpx/vpx_codec.h:93`).
// ===========================================================================

pub use crate::types::{
    VPX_CODEC_ABI_MISMATCH, VPX_CODEC_CORRUPT_FRAME, VPX_CODEC_ERROR, VPX_CODEC_INCAPABLE,
    VPX_CODEC_INVALID_PARAM, VPX_CODEC_LIST_END, VPX_CODEC_MEM_ERROR, VPX_CODEC_OK,
    VPX_CODEC_UNSUP_BITSTREAM, VPX_CODEC_UNSUP_FEATURE, VpxCodecErr, VpxResult,
};

pub type vpx_codec_err_t = VpxCodecErr;

// ===========================================================================
// `vpx_codec_caps_t` / `vpx_codec_flags_t` (`vpx/vpx_codec.h:155-170`).
// ===========================================================================

pub type VpxCodecCaps = c_long;
pub type vpx_codec_caps_t = VpxCodecCaps;

pub type VpxCodecFlags = c_long;
pub type vpx_codec_flags_t = VpxCodecFlags;

pub type VpxCodecIter = *const c_void;
pub type vpx_codec_iter_t = VpxCodecIter;

// Capability bits (vpx_codec.h, vpx_decoder.h, vpx_encoder.h).
pub const VPX_CODEC_CAP_DECODER: VpxCodecCaps = 0x1;
pub const VPX_CODEC_CAP_ENCODER: VpxCodecCaps = 0x2;
pub const VPX_CODEC_CAP_HIGHBITDEPTH: VpxCodecCaps = 0x4;

pub const VPX_CODEC_CAP_PUT_SLICE: VpxCodecCaps = 0x10000;
pub const VPX_CODEC_CAP_PUT_FRAME: VpxCodecCaps = 0x20000;
pub const VPX_CODEC_CAP_POSTPROC: VpxCodecCaps = 0x40000;
pub const VPX_CODEC_CAP_ERROR_CONCEALMENT: VpxCodecCaps = 0x80000;
pub const VPX_CODEC_CAP_INPUT_FRAGMENTS: VpxCodecCaps = 0x100000;
pub const VPX_CODEC_CAP_FRAME_THREADING: VpxCodecCaps = 0x200000;
pub const VPX_CODEC_CAP_EXTERNAL_FRAME_BUFFER: VpxCodecCaps = 0x400000;

pub const VPX_CODEC_CAP_PSNR: VpxCodecCaps = 0x10000;
pub const VPX_CODEC_CAP_OUTPUT_PARTITION: VpxCodecCaps = 0x20000;

// Init-flag bits (vpx_decoder.h, vpx_encoder.h).
pub const VPX_CODEC_USE_POSTPROC: VpxCodecFlags = 0x10000;
pub const VPX_CODEC_USE_ERROR_CONCEALMENT: VpxCodecFlags = 0x20000;
pub const VPX_CODEC_USE_INPUT_FRAGMENTS: VpxCodecFlags = 0x40000;
pub const VPX_CODEC_USE_FRAME_THREADING: VpxCodecFlags = 0x80000;
pub const VPX_CODEC_USE_PSNR: VpxCodecFlags = 0x10000;
pub const VPX_CODEC_USE_OUTPUT_PARTITION: VpxCodecFlags = 0x20000;
pub const VPX_CODEC_USE_HIGHBITDEPTH: VpxCodecFlags = 0x40000;

// ===========================================================================
// ABI version constants.
// ===========================================================================

pub const VPX_IMAGE_ABI_VERSION: c_int = 5;
pub const VPX_CODEC_ABI_VERSION: c_int = 4 + VPX_IMAGE_ABI_VERSION;
pub const VPX_DECODER_ABI_VERSION: c_int = 3 + VPX_CODEC_ABI_VERSION;
pub const VPX_EXT_RATECTRL_ABI_VERSION: c_int = 1;
pub const VPX_ENCODER_ABI_VERSION: c_int =
    18 + VPX_CODEC_ABI_VERSION + VPX_EXT_RATECTRL_ABI_VERSION;
pub const VPX_CODEC_INTERNAL_ABI_VERSION: c_int = 5;

// ===========================================================================
// `vpx_image_t` (`vpx/vpx_image.h`).
// ===========================================================================

pub const VPX_IMG_FMT_PLANAR: i32 = 0x100;
pub const VPX_IMG_FMT_UV_FLIP: i32 = 0x200;
pub const VPX_IMG_FMT_HAS_ALPHA: i32 = 0x400;
pub const VPX_IMG_FMT_HIGHBITDEPTH: i32 = 0x800;

/// `vpx_img_fmt_t` is a bit-OR-able set of `VPX_IMG_FMT_*` flag values
/// plus a small index, so we expose it as a transparent `i32`.
pub type VpxImgFmt = i32;
pub type vpx_img_fmt_t = VpxImgFmt;

pub const VPX_IMG_FMT_NONE: vpx_img_fmt_t = 0;
pub const VPX_IMG_FMT_YV12: vpx_img_fmt_t = VPX_IMG_FMT_PLANAR | VPX_IMG_FMT_UV_FLIP | 1;
pub const VPX_IMG_FMT_I420: vpx_img_fmt_t = VPX_IMG_FMT_PLANAR | 2;
pub const VPX_IMG_FMT_I422: vpx_img_fmt_t = VPX_IMG_FMT_PLANAR | 5;
pub const VPX_IMG_FMT_I444: vpx_img_fmt_t = VPX_IMG_FMT_PLANAR | 6;
pub const VPX_IMG_FMT_I440: vpx_img_fmt_t = VPX_IMG_FMT_PLANAR | 7;
pub const VPX_IMG_FMT_NV12: vpx_img_fmt_t = VPX_IMG_FMT_PLANAR | 9;
pub const VPX_IMG_FMT_I42016: vpx_img_fmt_t = VPX_IMG_FMT_I420 | VPX_IMG_FMT_HIGHBITDEPTH;
pub const VPX_IMG_FMT_I42216: vpx_img_fmt_t = VPX_IMG_FMT_I422 | VPX_IMG_FMT_HIGHBITDEPTH;
pub const VPX_IMG_FMT_I44416: vpx_img_fmt_t = VPX_IMG_FMT_I444 | VPX_IMG_FMT_HIGHBITDEPTH;
pub const VPX_IMG_FMT_I44016: vpx_img_fmt_t = VPX_IMG_FMT_I440 | VPX_IMG_FMT_HIGHBITDEPTH;

pub type VpxColorSpace = u32;
pub type vpx_color_space_t = VpxColorSpace;
pub const VPX_CS_UNKNOWN: vpx_color_space_t = 0;
pub const VPX_CS_BT_601: vpx_color_space_t = 1;
pub const VPX_CS_BT_709: vpx_color_space_t = 2;
pub const VPX_CS_SMPTE_170: vpx_color_space_t = 3;
pub const VPX_CS_SMPTE_240: vpx_color_space_t = 4;
pub const VPX_CS_BT_2020: vpx_color_space_t = 5;
pub const VPX_CS_RESERVED: vpx_color_space_t = 6;
pub const VPX_CS_SRGB: vpx_color_space_t = 7;

pub type VpxColorRange = u32;
pub type vpx_color_range_t = VpxColorRange;
pub const VPX_CR_STUDIO_RANGE: vpx_color_range_t = 0;
pub const VPX_CR_FULL_RANGE: vpx_color_range_t = 1;

pub const VPX_PLANE_PACKED: usize = 0;
pub const VPX_PLANE_Y: usize = 0;
pub const VPX_PLANE_U: usize = 1;
pub const VPX_PLANE_V: usize = 2;
pub const VPX_PLANE_ALPHA: usize = 3;

/// `vpx_image_t` (`vpx/vpx_image.h:76`). Full struct body — the
/// canonical layout shared by every consumer module.
#[repr(C)]
pub struct VpxImage {
    pub fmt: vpx_img_fmt_t,
    pub cs: vpx_color_space_t,
    pub range: vpx_color_range_t,

    pub w: c_uint,
    pub h: c_uint,
    pub bit_depth: c_uint,

    pub d_w: c_uint,
    pub d_h: c_uint,

    pub r_w: c_uint,
    pub r_h: c_uint,

    pub x_chroma_shift: c_uint,
    pub y_chroma_shift: c_uint,

    pub planes: [*mut u8; 4],
    pub stride: [c_int; 4],

    pub bps: c_int,

    pub user_priv: *mut c_void,

    pub img_data: *mut u8,
    pub img_data_owner: c_int,
    pub self_allocd: c_int,

    pub fb_priv: *mut c_void,
}

pub type vpx_image_t = VpxImage;

#[repr(C)]
#[derive(Copy, Clone)]
pub struct VpxImageRect {
    pub x: c_uint,
    pub y: c_uint,
    pub w: c_uint,
    pub h: c_uint,
}

pub type vpx_image_rect_t = VpxImageRect;

/// `vpx_bit_depth_t` (`vpx/vpx_codec.h:220`).
#[repr(C)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum VpxBitDepth {
    VPX_BITS_8 = 8,
    VPX_BITS_10 = 10,
    VPX_BITS_12 = 12,
}
pub use VpxBitDepth::*;
pub type vpx_bit_depth_t = VpxBitDepth;

// ===========================================================================
// `vpx_codec_frame_buffer_t` (`vpx/vpx_frame_buffer.h`). Opaque to the
// dispatcher.
// ===========================================================================

#[repr(C)]
pub struct VpxCodecFrameBuffer {
    _opaque: [u8; 0],
}
pub type vpx_codec_frame_buffer_t = VpxCodecFrameBuffer;

pub type VpxGetFrameBufferCbFnT = Option<
    unsafe extern "C" fn(
        priv_: *mut c_void,
        min_size: usize,
        fb: *mut VpxCodecFrameBuffer,
    ) -> c_int,
>;
pub type vpx_get_frame_buffer_cb_fn_t = VpxGetFrameBufferCbFnT;

pub type VpxReleaseFrameBufferCbFnT =
    Option<unsafe extern "C" fn(priv_: *mut c_void, fb: *mut VpxCodecFrameBuffer) -> c_int>;
pub type vpx_release_frame_buffer_cb_fn_t = VpxReleaseFrameBufferCbFnT;

// ===========================================================================
// Decoder-side callback signatures (`vpx/vpx_decoder.h`).
// ===========================================================================

pub type VpxCodecPutFrameCbFnT =
    Option<unsafe extern "C" fn(user_priv: *mut c_void, img: *const VpxImage)>;
pub type vpx_codec_put_frame_cb_fn_t = VpxCodecPutFrameCbFnT;

pub type VpxCodecPutSliceCbFnT = Option<
    unsafe extern "C" fn(
        user_priv: *mut c_void,
        img: *const VpxImage,
        valid: *const VpxImageRect,
        update: *const VpxImageRect,
    ),
>;
pub type vpx_codec_put_slice_cb_fn_t = VpxCodecPutSliceCbFnT;

// ===========================================================================
// Decoder / encoder configuration (`vpx/vpx_decoder.h`,
// `vpx/vpx_encoder.h`).
// ===========================================================================

/// `vpx_codec_dec_cfg_t` (`vpx/vpx_decoder.h:106`).
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct VpxCodecDecCfg {
    pub threads: c_uint,
    pub w: c_uint,
    pub h: c_uint,
}
pub type vpx_codec_dec_cfg_t = VpxCodecDecCfg;

/// `vpx_codec_stream_info_t` (`vpx/vpx_decoder.h:88`).
#[repr(C)]
#[derive(Copy, Clone)]
pub struct VpxCodecStreamInfo {
    pub sz: c_uint,
    pub w: c_uint,
    pub h: c_uint,
    pub is_kf: c_uint,
}
pub type vpx_codec_stream_info_t = VpxCodecStreamInfo;

/// `vpx_codec_enc_cfg_t` — opaque to the dispatcher; codec-specific
/// translation units carry the full layout.
#[repr(C)]
pub struct VpxCodecEncCfg {
    _opaque: [u8; 0],
}
pub type vpx_codec_enc_cfg_t = VpxCodecEncCfg;

// ===========================================================================
// Encoder support types (`vpx/vpx_encoder.h`).
// ===========================================================================

pub type VpxCodecPts = i64;
pub type vpx_codec_pts_t = VpxCodecPts;

pub type VpxEncFrameFlags = c_long;
pub type vpx_enc_frame_flags_t = VpxEncFrameFlags;

pub type VpxEncDeadline = c_ulong;
pub type vpx_enc_deadline_t = VpxEncDeadline;

pub type VpxCodecFrameFlags = u32;
pub type vpx_codec_frame_flags_t = VpxCodecFrameFlags;

pub const VPX_CODEC_CX_FRAME_PKT: c_int = 0;

#[repr(C)]
#[derive(Copy, Clone)]
pub struct VpxFixedBuf {
    pub buf: *mut c_void,
    pub sz: usize,
}
pub type vpx_fixed_buf_t = VpxFixedBuf;

#[repr(C)]
#[derive(Copy, Clone)]
pub struct VpxRational {
    pub num: c_int,
    pub den: c_int,
}
pub type vpx_rational_t = VpxRational;

/// `vpx_codec_cx_pkt::data` union (mirror of the relevant arms). The
/// padding is sized to fit the C union (128 bytes).
#[repr(C)]
pub union VpxCodecCxPktData {
    pub raw: VpxFixedBuf,
    pub pad: [u8; 128],
}

#[repr(C)]
pub struct VpxCodecCxPkt {
    pub kind: c_int,
    pub data: VpxCodecCxPktData,
}
pub type vpx_codec_cx_pkt_t = VpxCodecCxPkt;

/// `vpx_codec_pkt_list` (`vpx/internal/vpx_codec_internal.h:399`).
#[repr(C)]
pub struct VpxCodecPktList {
    pub cnt: c_uint,
    pub max: c_uint,
    /// Trailing `vpx_codec_cx_pkt pkts[1]` in C.
    pub pkts: [VpxCodecCxPkt; 1],
}
pub type vpx_codec_pkt_list_t = VpxCodecPktList;

// ===========================================================================
// Internal codec-ABI types (`vpx/internal/vpx_codec_internal.h`).
// ===========================================================================

/// `vpx_codec_alg_priv_t` (opaque to the dispatcher).
#[repr(C)]
pub struct VpxCodecAlgPriv {
    _opaque: [u8; 0],
}
pub type vpx_codec_alg_priv_t = VpxCodecAlgPriv;

/// `vpx_codec_priv_enc_mr_cfg_t` (`vpx/internal/vpx_codec_internal.h:364`).
#[repr(C)]
pub struct VpxCodecPrivEncMrCfg {
    pub mr_total_resolutions: c_uint,
    pub mr_encoder_id: c_uint,
    pub mr_down_sampling_factor: VpxRational,
    pub mr_low_res_mode_info: *mut c_void,
}
pub type vpx_codec_priv_enc_mr_cfg_t = VpxCodecPrivEncMrCfg;

/// `vpx_codec_iface_t` (`vpx/internal/vpx_codec_internal.h`). The
/// dispatcher only inspects `name`, `abi_version`, and `caps`; the
/// full vtable lives behind the `Decoder` trait object now.
pub struct VpxCodecIface {
    pub name: &'static str,
    pub abi_version: c_int,
    pub caps: VpxCodecCaps,
}
pub type vpx_codec_iface_t = VpxCodecIface;

unsafe impl Sync for VpxCodecIface {}

/// `vpx_codec_priv_cb_pair_t` (`vpx_codec_internal.h:329`). The two
/// callback pointers share storage via a C union — we use a single
/// `*mut c_void` slot here because both callbacks are pointer-sized.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct VpxCodecPrivCbPair {
    pub u: *mut c_void,
    pub user_priv: *mut c_void,
}
pub type vpx_codec_priv_cb_pair_t = VpxCodecPrivCbPair;

/// `vpx_codec_priv_t::dec` sub-struct.
#[repr(C)]
pub struct VpxCodecPrivDec {
    pub put_frame_cb: VpxCodecPrivCbPair,
    pub put_slice_cb: VpxCodecPrivCbPair,
}

/// `vpx_codec_priv_t::enc` sub-struct.
#[repr(C)]
pub struct VpxCodecPrivEnc {
    pub cx_data_dst_buf: VpxFixedBuf,
    pub cx_data_pad_before: c_uint,
    pub cx_data_pad_after: c_uint,
    pub cx_data_pkt: VpxCodecCxPkt,
    pub total_encoders: c_uint,
}

/// `struct vpx_codec_priv` (`vpx_codec_internal.h:345`).
#[repr(C)]
pub struct VpxCodecPriv {
    pub init_flags: VpxCodecFlags,
    pub dec: VpxCodecPrivDec,
    pub enc: VpxCodecPrivEnc,
}
pub type vpx_codec_priv_t = VpxCodecPriv;

/// `vpx_codec_ctx_t` (`vpx/vpx_codec.h:200`).
pub struct VpxCodecCtx {
    pub name: Option<&'static str>,
    /// Iface descriptor; `None` until `vpx_codec_dec_init_ver` binds
    /// one. The `static mut VPX_CODEC_VP8_DX_ALGO` outlives any
    /// `VpxCodecCtx`, so the `'static` lifetime is sound.
    pub iface: Option<&'static VpxCodecIface>,
    pub err: VpxCodecErr,
    pub init_flags: VpxCodecFlags,
    // No `config` field. In libvpx, `vpx_codec_ctx_t::config` is a
    // pointer-aliasing union used only as the generic-dispatcher →
    // codec-`init(ctx)` hand-off channel for the caller's `cfg`, which
    // the codec then copies inward and re-points at its internal copy
    // (`vp8/vp8_dx_iface.c:78-81`). This port hands `cfg` to
    // `Vp8Decoder::new` / `set_cfg` directly as a typed parameter, so
    // the channel — and the field — carry nothing anyone reads back.
    //
    // No `priv_` field either. In libvpx, `vpx_codec_ctx_t::priv` is
    // both the dispatch handle (every entry recovers the alg-priv from
    // it) and the "initialized?" sentinel. Dispatch here goes through
    // `trait_obj`, so the only surviving role — the init check — is
    // already answered by `trait_obj.is_some()`.
    /// Boxed [`crate::codec::Decoder`] trait object. `None` before
    /// `vpx_codec_dec_init_ver` succeeds.
    pub trait_obj: Option<Box<dyn crate::codec::Decoder + 'static>>,
}
pub type vpx_codec_ctx_t = VpxCodecCtx;

// ===========================================================================
// Re-export of all public functions from the five consumer modules.
// ===========================================================================

pub use crate::vpx_codec::{
    vpx_codec_control_, vpx_codec_destroy, vpx_codec_err_to_string, vpx_codec_error,
    vpx_codec_error_detail, vpx_codec_get_caps, vpx_codec_iface_name, vpx_codec_version,
    vpx_codec_version_extra_str, vpx_codec_version_str, vpx_internal_error,
};

pub use crate::vpx_decoder::{
    vpx_codec_dec_init_ver, vpx_codec_decode, vpx_codec_get_frame, vpx_codec_get_stream_info,
    vpx_codec_peek_stream_info, vpx_codec_register_put_frame_cb, vpx_codec_register_put_slice_cb,
    vpx_codec_set_frame_buffer_functions,
};

pub use crate::vpx_encoder::{
    vpx_codec_enc_config_default, vpx_codec_enc_config_set, vpx_codec_enc_init_multi_ver,
    vpx_codec_enc_init_ver, vpx_codec_encode, vpx_codec_get_cx_data, vpx_codec_get_global_headers,
    vpx_codec_get_preview_frame, vpx_codec_pkt_list_add, vpx_codec_pkt_list_get,
    vpx_codec_set_cx_data_buf,
};

pub use crate::vpx_image::{
    vpx_img_alloc, vpx_img_flip, vpx_img_free, vpx_img_set_rect, vpx_img_wrap,
};

pub use crate::vp8_dx_iface::vpx_codec_vp8_dx;
