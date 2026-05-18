//! Decoder-side public-API dispatcher. Each entry point validates
//! arguments + capability bits, then dispatches through the
//! `Decoder` trait stashed on `VpxCodecCtx::trait_obj`.
//!
//! Public types and constants live in `crate::vpx_api`.

#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use core::ffi::c_void;
use core::ptr;

use crate::vpx_api::*;
use crate::vpx_codec::vpx_codec_destroy;
use crate::vp8_dx_iface::Vp8Decoder;
use crate::codec::Decoder;

/// Stash `var` into `ctx.err` (if `ctx` is `Some`) and return it.
#[inline]
fn save_status(ctx: Option<&mut VpxCodecCtx>, var: VpxCodecErr) -> VpxCodecErr {
    if let Some(c) = ctx {
        c.err = var;
    }
    var
}

/// `vpx_codec_dec_init_ver` — bind a context to an algorithm.
pub unsafe fn vpx_codec_dec_init_ver(
    ctx: Option<&mut VpxCodecCtx>,
    iface: Option<&'static VpxCodecIface>,
    cfg: Option<&VpxCodecDecCfg>,
    flags: VpxCodecFlags,
    ver: i32,
) -> VpxCodecErr {
    if ver != VPX_DECODER_ABI_VERSION {
        return save_status(ctx, VPX_CODEC_ABI_MISMATCH);
    }
    let Some(ctx) = ctx else { return VPX_CODEC_INVALID_PARAM };
    let Some(iface) = iface else {
        ctx.err = VPX_CODEC_INVALID_PARAM;
        return VPX_CODEC_INVALID_PARAM;
    };

    // Each entry pairs a USE_* flag with the CAP_* bit the iface must
    // advertise to honor it. Empty intersection => INCAPABLE.
    const CAP_REQUIRED: &[(VpxCodecFlags, VpxCodecCaps)] = &[
        (VPX_CODEC_USE_POSTPROC, VPX_CODEC_CAP_POSTPROC),
        (VPX_CODEC_USE_ERROR_CONCEALMENT, VPX_CODEC_CAP_ERROR_CONCEALMENT),
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

    // Reset the slot to known-initial state. The struct is no longer
    // `#[repr(C)]` (it holds a `Box<dyn Decoder>` field), so we can't
    // memset it blindly. Assign each field instead.
    ctx.iface = Some(iface);
    ctx.name = iface.name;
    ctx.priv_ = ptr::null_mut();
    ctx.err_detail = ptr::null();
    ctx.init_flags = flags;
    ctx.config.dec = cfg.map(|c| c as *const VpxCodecDecCfg).unwrap_or(ptr::null());
    ctx.trait_obj = None;

    // VP8 is currently the only algorithm; when VP9 lands a small
    // per-algo registry will choose between constructors.
    match Vp8Decoder::new(flags) {
        Ok(mut dec) => {
            let priv_ptr = dec.as_ptr();
            if let Some(c) = cfg {
                (*priv_ptr).cfg = *c;
            }
            // priv_ is a non-null sentinel for initialized-state
            // checks elsewhere; it points at the same Vp8AlgPriv the
            // boxed trait object owns.
            ctx.priv_ = priv_ptr as *mut VpxCodecPriv;
            ctx.trait_obj = Some(Box::new(dec));
            ctx.err = VPX_CODEC_OK;
            VPX_CODEC_OK
        }
        Err(e) => {
            ctx.err = e;
            vpx_codec_destroy(Some(ctx));
            e
        }
    }
}

/// `vpx_codec_peek_stream_info` — parse without committing. The
/// `iface` parameter is kept only for null-check + param validation;
/// dispatch uses `Decoder::peek_stream_info`.
pub fn vpx_codec_peek_stream_info(
    iface: Option<&VpxCodecIface>,
    data: &[u8],
    si: Option<&mut VpxCodecStreamInfo>,
) -> VpxCodecErr {
    if iface.is_none() || data.is_empty() {
        return VPX_CODEC_INVALID_PARAM;
    }
    let Some(si) = si else { return VPX_CODEC_INVALID_PARAM };
    if (si.sz as usize) < core::mem::size_of::<VpxCodecStreamInfo>() {
        return VPX_CODEC_INVALID_PARAM;
    }
    // Set default/unknown values
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
pub unsafe fn vpx_codec_get_stream_info(
    ctx: Option<&mut VpxCodecCtx>,
    si: Option<&mut VpxCodecStreamInfo>,
) -> VpxCodecErr {
    let Some(ctx) = ctx else { return VPX_CODEC_INVALID_PARAM };
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
/// flush. `_user_priv` is ignored (the per-frame opaque-pointer
/// tagging feature was dropped during the trait refactor).
pub fn vpx_codec_decode(
    ctx: Option<&mut VpxCodecCtx>,
    data: &[u8],
    _user_priv: *mut c_void,
    _deadline: i64,
) -> VpxCodecErr {
    let Some(ctx) = ctx else { return VPX_CODEC_INVALID_PARAM };
    if ctx.iface.is_none() || ctx.trait_obj.is_none() {
        ctx.err = VPX_CODEC_ERROR;
        return VPX_CODEC_ERROR;
    }
    let res = match ctx
        .trait_obj
        .as_mut()
        .unwrap()
        .decode(data, core::time::Duration::ZERO)
    {
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
    ctx: Option<&'a mut VpxCodecCtx>,
    iter: &mut VpxCodecIter,
) -> Option<&'a VpxImage> {
    let ctx = ctx?;
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
    ctx: Option<&mut VpxCodecCtx>,
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
    ctx: Option<&mut VpxCodecCtx>,
    cb: VpxCodecPutSliceCbFnT,
    _user_priv: *mut c_void,
) -> VpxCodecErr {
    if cb.is_none() {
        return save_status(ctx, VPX_CODEC_INVALID_PARAM);
    }
    save_status(ctx, VPX_CODEC_INCAPABLE)
}

/// `vpx_codec_set_frame_buffer_functions`. The VP8 build lacks
/// `VPX_CODEC_CAP_EXTERNAL_FRAME_BUFFER`; the trait-based
/// `FrameBufferAllocator` hook in `crate::codec` is the future
/// replacement.
pub fn vpx_codec_set_frame_buffer_functions(
    ctx: Option<&mut VpxCodecCtx>,
    cb_get: VpxGetFrameBufferCbFnT,
    cb_release: VpxReleaseFrameBufferCbFnT,
    _cb_priv: *mut c_void,
) -> VpxCodecErr {
    if cb_get.is_none() || cb_release.is_none() {
        return save_status(ctx, VPX_CODEC_INVALID_PARAM);
    }
    save_status(ctx, VPX_CODEC_INCAPABLE)
}
