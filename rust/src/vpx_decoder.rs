//! Decoder-side public-API dispatcher. Each entry point validates
//! arguments + capability bits, then dispatches through the
//! `Decoder` trait stashed on `VpxCodecCtx::trait_obj`.
//!
//! Public types and constants live in `crate::vpx_api`.

#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use core::ffi::c_void;

use crate::codec::Decoder;
use crate::vp8_dx_iface::Vp8Decoder;
use crate::vpx_api::*;
use crate::vpx_codec::vpx_codec_destroy;

/// Stash `var` into `ctx.err` and return it.
#[inline]
fn save_status(ctx: &mut VpxCodecCtx, var: VpxCodecErr) -> VpxCodecErr {
    ctx.err = var;
    var
}

/// `vpx_codec_dec_init_ver` — bind a context to an algorithm.
pub fn vpx_codec_dec_init_ver(
    ctx: &mut VpxCodecCtx,
    iface: Option<&'static VpxCodecIface>,
    cfg: Option<&VpxCodecDecCfg>,
    flags: VpxCodecFlags,
    ver: i32,
) -> VpxCodecErr {
    if ver != VPX_DECODER_ABI_VERSION {
        return save_status(ctx, VPX_CODEC_ABI_MISMATCH);
    }
    let Some(iface) = iface else {
        ctx.err = VPX_CODEC_INVALID_PARAM;
        return VPX_CODEC_INVALID_PARAM;
    };

    // Each entry pairs a USE_* flag with the CAP_* bit the iface must
    // advertise to honor it. Empty intersection => INCAPABLE.
    const CAP_REQUIRED: &[(VpxCodecFlags, VpxCodecCaps)] = &[
        (VPX_CODEC_USE_POSTPROC, VPX_CODEC_CAP_POSTPROC),
        (
            VPX_CODEC_USE_ERROR_CONCEALMENT,
            VPX_CODEC_CAP_ERROR_CONCEALMENT,
        ),
        (VPX_CODEC_USE_INPUT_FRAGMENTS, VPX_CODEC_CAP_INPUT_FRAGMENTS),
    ];

    let validation_err = if iface.abi_version != VPX_CODEC_INTERNAL_ABI_VERSION {
        Some(VPX_CODEC_ABI_MISMATCH)
    } else if (iface.caps & VPX_CODEC_CAP_DECODER) == 0
        || CAP_REQUIRED
            .iter()
            .any(|&(flag, cap)| (flags & flag) != 0 && (iface.caps & cap) == 0)
    {
        Some(VPX_CODEC_INCAPABLE)
    } else {
        None
    };
    if let Some(e) = validation_err {
        ctx.err = e;
        return e;
    }

    // Reset the context to its known-initial state.
    ctx.iface = Some(iface);
    ctx.name = Some(iface.name);
    ctx.init_flags = flags;
    ctx.trait_obj = None;

    // VP8 is currently the only supported algorithm.
    match Vp8Decoder::new(flags) {
        Ok(mut dec) => {
            if let Some(c) = cfg {
                dec.set_cfg(*c);
            }
            ctx.trait_obj = Some(Box::new(dec));
            ctx.err = VPX_CODEC_OK;
            VPX_CODEC_OK
        }
        Err(e) => {
            ctx.err = e;
            vpx_codec_destroy(ctx);
            e
        }
    }
}

/// `vpx_codec_peek_stream_info` — parse without committing. `iface` is
/// only validated; dispatch uses `Decoder::peek_stream_info`.
pub fn vpx_codec_peek_stream_info(
    iface: Option<&VpxCodecIface>,
    data: &[u8],
    si: Option<&mut VpxCodecStreamInfo>,
) -> VpxCodecErr {
    if iface.is_none() || data.is_empty() {
        return VPX_CODEC_INVALID_PARAM;
    }
    let Some(si) = si else {
        return VPX_CODEC_INVALID_PARAM;
    };
    if (si.sz as usize) < core::mem::size_of::<VpxCodecStreamInfo>() {
        return VPX_CODEC_INVALID_PARAM;
    }
    si.w = 0;
    si.h = 0;

    match Vp8Decoder::peek_stream_info(data) {
        Ok(out) => {
            *si = out;
            VPX_CODEC_OK
        }
        Err(e) => e,
    }
}

/// `vpx_codec_get_stream_info` — query an active context.
pub fn vpx_codec_get_stream_info(
    ctx: &mut VpxCodecCtx,
    si: Option<&mut VpxCodecStreamInfo>,
) -> VpxCodecErr {
    let Some(si) = si else {
        ctx.err = VPX_CODEC_INVALID_PARAM;
        return VPX_CODEC_INVALID_PARAM;
    };
    if (si.sz as usize) < core::mem::size_of::<VpxCodecStreamInfo>() {
        ctx.err = VPX_CODEC_INVALID_PARAM;
        return VPX_CODEC_INVALID_PARAM;
    }
    if ctx.iface.is_none() || ctx.trait_obj.is_none() {
        ctx.err = VPX_CODEC_ERROR;
        return VPX_CODEC_ERROR;
    }
    si.w = 0;
    si.h = 0;

    let res = match ctx.trait_obj.as_ref().unwrap().stream_info() {
        Ok(out) => {
            *si = out;
            VPX_CODEC_OK
        }
        Err(e) => e,
    };
    ctx.err = res;
    res
}

/// `vpx_codec_decode` — feed encoded bytes in. `data` empty means
/// flush. `user_priv` is stashed on the next emitted [`VpxImage`]'s
/// `user_priv` field for per-frame caller tagging.
pub fn vpx_codec_decode(
    ctx: &mut VpxCodecCtx,
    data: &[u8],
    user_priv: *mut c_void,
    _deadline: i64,
) -> VpxCodecErr {
    if ctx.iface.is_none() || ctx.trait_obj.is_none() {
        ctx.err = VPX_CODEC_ERROR;
        return VPX_CODEC_ERROR;
    }
    let dec = ctx.trait_obj.as_mut().unwrap();
    dec.set_user_priv(user_priv);
    let res = match dec.decode(data, core::time::Duration::ZERO) {
        Ok(()) => VPX_CODEC_OK,
        Err(e) => e,
    };
    ctx.err = res;
    res
}

/// `vpx_codec_get_frame` — drain decoded pictures. The user's `iter`
/// is mirrored to the trait's internal iter: first call after a decode
/// returns the new image and toggles iter; subsequent calls return
/// `None`.
pub fn vpx_codec_get_frame<'a>(
    ctx: &'a mut VpxCodecCtx,
    iter: &mut VpxCodecIter,
) -> Option<&'a VpxImage> {
    if ctx.iface.is_none() || ctx.trait_obj.is_none() {
        return None;
    }
    if !iter.is_null() {
        // Caller's iter already advanced past the single VP8 output.
        return None;
    }

    let image = ctx.trait_obj.as_mut().unwrap().get_frame()?;
    *iter = image as *const VpxImage as *const c_void;
    Some(image)
}

/// `vpx_codec_register_put_frame_cb`. The VP8 build lacks
/// `VPX_CODEC_CAP_PUT_FRAME`, so this always returns `INCAPABLE`.
pub fn vpx_codec_register_put_frame_cb(
    ctx: &mut VpxCodecCtx,
    cb: VpxCodecPutFrameCbFnT,
    _user_priv: *mut c_void,
) -> VpxCodecErr {
    if cb.is_none() {
        return save_status(ctx, VPX_CODEC_INVALID_PARAM);
    }
    save_status(ctx, VPX_CODEC_INCAPABLE)
}

/// `vpx_codec_register_put_slice_cb`. The VP8 build lacks
/// `VPX_CODEC_CAP_PUT_SLICE`, so this always returns `INCAPABLE`.
pub fn vpx_codec_register_put_slice_cb(
    ctx: &mut VpxCodecCtx,
    cb: VpxCodecPutSliceCbFnT,
    _user_priv: *mut c_void,
) -> VpxCodecErr {
    if cb.is_none() {
        return save_status(ctx, VPX_CODEC_INVALID_PARAM);
    }
    save_status(ctx, VPX_CODEC_INCAPABLE)
}

/// `vpx_codec_set_frame_buffer_functions`. The VP8 build lacks
/// `VPX_CODEC_CAP_EXTERNAL_FRAME_BUFFER`, so this always returns
/// `INCAPABLE`.
pub fn vpx_codec_set_frame_buffer_functions(
    ctx: &mut VpxCodecCtx,
    cb_get: VpxGetFrameBufferCbFnT,
    cb_release: VpxReleaseFrameBufferCbFnT,
    _cb_priv: *mut c_void,
) -> VpxCodecErr {
    if cb_get.is_none() || cb_release.is_none() {
        return save_status(ctx, VPX_CODEC_INVALID_PARAM);
    }
    save_status(ctx, VPX_CODEC_INCAPABLE)
}
