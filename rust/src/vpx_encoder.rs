//! Encoder API stub.
//!
//! libvpx build is `--disable-vp8-encoder --disable-vp9`, so no encoder
//! implementation exists. The public `vpx_codec_enc_*` functions are
//! retained as stable entry points but always return
//! `VPX_CODEC_INCAPABLE`. A real `Encoder` surface lives at
//! `crate::codec::Encoder`.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(unused_variables)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::{c_int, c_ulong};
use core::ptr;

use crate::vpx_api::*;


pub unsafe fn vpx_codec_enc_init_ver(
    _ctx: *mut VpxCodecCtx,
    _iface: *mut VpxCodecIface,
    _cfg: *const VpxCodecEncCfg,
    _flags: VpxCodecFlags,
    _ver: c_int,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}


pub unsafe fn vpx_codec_enc_init_multi_ver(
    _ctx: *mut VpxCodecCtx,
    _iface: *mut VpxCodecIface,
    _cfg: *const VpxCodecEncCfg,
    _num_enc: c_int,
    _flags: VpxCodecFlags,
    _dsf: *mut VpxRational,
    _ver: c_int,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}


pub unsafe fn vpx_codec_enc_config_default(
    _iface: *mut VpxCodecIface,
    _cfg: *mut VpxCodecEncCfg,
    _usage: c_int,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}


pub unsafe fn vpx_codec_enc_config_set(
    _ctx: *mut VpxCodecCtx,
    _cfg: *const VpxCodecEncCfg,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}


pub unsafe fn vpx_codec_encode(
    _ctx: *mut VpxCodecCtx,
    _img: *const VpxImage,
    _pts: VpxCodecPts,
    _duration: c_ulong,
    _flags: VpxEncFrameFlags,
    _deadline: VpxEncDeadline,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}


pub unsafe fn vpx_codec_get_cx_data(
    _ctx: *mut VpxCodecCtx,
    _iter: *mut VpxCodecIter,
) -> *const VpxCodecCxPkt {
    ptr::null()
}


pub unsafe fn vpx_codec_get_global_headers(
    _ctx: *mut VpxCodecCtx,
) -> *mut VpxFixedBuf {
    ptr::null_mut()
}


pub unsafe fn vpx_codec_get_preview_frame(
    _ctx: *mut VpxCodecCtx,
) -> *mut VpxImage {
    ptr::null_mut()
}


pub unsafe fn vpx_codec_set_cx_data_buf(
    _ctx: *mut VpxCodecCtx,
    _buf: *const VpxFixedBuf,
    _pad_before: c_int,
    _pad_after: c_int,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}


pub unsafe fn vpx_codec_pkt_list_add(
    _list: *mut VpxCodecPktList,
    _pkt: *const VpxCodecCxPkt,
) -> c_int {
    -1
}


pub unsafe fn vpx_codec_pkt_list_get(
    _list: *mut VpxCodecPktList,
    _iter: *mut VpxCodecIter,
) -> *const VpxCodecCxPkt {
    ptr::null()
}
