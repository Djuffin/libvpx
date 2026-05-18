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

use crate::vpx_api::*;

pub fn vpx_codec_enc_init_ver(
    _ctx: Option<&mut VpxCodecCtx>,
    _iface: Option<&VpxCodecIface>,
    _cfg: Option<&VpxCodecEncCfg>,
    _flags: VpxCodecFlags,
    _ver: c_int,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}

pub fn vpx_codec_enc_init_multi_ver(
    _ctx: Option<&mut VpxCodecCtx>,
    _iface: Option<&VpxCodecIface>,
    _cfg: Option<&VpxCodecEncCfg>,
    _num_enc: c_int,
    _flags: VpxCodecFlags,
    _dsf: Option<&mut VpxRational>,
    _ver: c_int,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}

pub fn vpx_codec_enc_config_default(
    _iface: Option<&VpxCodecIface>,
    _cfg: Option<&mut VpxCodecEncCfg>,
    _usage: c_int,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}

pub fn vpx_codec_enc_config_set(
    _ctx: Option<&mut VpxCodecCtx>,
    _cfg: Option<&VpxCodecEncCfg>,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}

pub fn vpx_codec_encode(
    _ctx: Option<&mut VpxCodecCtx>,
    _img: Option<&VpxImage>,
    _pts: VpxCodecPts,
    _duration: c_ulong,
    _flags: VpxEncFrameFlags,
    _deadline: VpxEncDeadline,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}

pub fn vpx_codec_get_cx_data<'a>(
    _ctx: Option<&'a mut VpxCodecCtx>,
    _iter: &mut VpxCodecIter,
) -> Option<&'a VpxCodecCxPkt> {
    None
}

pub fn vpx_codec_get_global_headers(
    _ctx: Option<&mut VpxCodecCtx>,
) -> Option<&VpxFixedBuf> {
    None
}

pub fn vpx_codec_get_preview_frame(
    _ctx: Option<&mut VpxCodecCtx>,
) -> Option<&VpxImage> {
    None
}

pub fn vpx_codec_set_cx_data_buf(
    _ctx: Option<&mut VpxCodecCtx>,
    _buf: Option<&VpxFixedBuf>,
    _pad_before: c_int,
    _pad_after: c_int,
) -> VpxCodecErr {
    VPX_CODEC_INCAPABLE
}

pub fn vpx_codec_pkt_list_add(
    _list: Option<&mut VpxCodecPktList>,
    _pkt: Option<&VpxCodecCxPkt>,
) -> c_int {
    -1
}

pub fn vpx_codec_pkt_list_get<'a>(
    _list: Option<&'a VpxCodecPktList>,
    _iter: &mut VpxCodecIter,
) -> Option<&'a VpxCodecCxPkt> {
    None
}
