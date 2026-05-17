//! Literal Rust transliteration of `vpx/src/vpx_decoder.c`.
//!
//! Decoder-side public-API dispatcher: every entry point validates its
//! arguments, validates the bound algorithm's capabilities, then routes
//! through `ctx->iface->dec.<slot>(...)` (or, for the put-frame /
//! put-slice callback registrations, mutates `ctx->priv` directly).
//!
//! Public types and constants live in `crate::vpx_api`.

#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use core::ffi::{c_uint, c_void};
use core::ptr;

use crate::vpx_api::*;
use crate::vpx_codec::vpx_codec_destroy;

// ===========================================================================
// Helpers (translation of file-static helpers in `vpx_decoder.c`).
// ===========================================================================

/// Returns truthy in the C sense — every non-OK error code is `true`,
/// matching the `if (res) { ... }` idiom in the C source.
#[inline]
fn err_is_set(res: VpxCodecErr) -> bool {
    res as i32 != 0
}

/// `SAVE_STATUS(ctx, var)` — write-through error reporting. Mirrors the
/// C macro: stash `var` into `ctx->err` iff `ctx` is non-NULL, then
/// return `var`.
#[inline]
unsafe fn save_status(ctx: *mut VpxCodecCtx, var: VpxCodecErr) -> VpxCodecErr {
    if !ctx.is_null() {
        (*ctx).err = var;
    }
    var
}

/// `get_alg_priv` — cast `ctx->priv` (a `*mut vpx_codec_priv`) down to
/// the opaque, codec-private `*mut vpx_codec_alg_priv_t`. Safe by
/// contract: every algorithm allocates `vpx_codec_alg_priv_t` such that
/// its first bytes are a `vpx_codec_priv` header.
#[inline]
unsafe fn get_alg_priv(ctx: *mut VpxCodecCtx) -> *mut VpxCodecAlgPriv {
    (*ctx).priv_ as *mut VpxCodecAlgPriv
}

// ===========================================================================
// Public-API dispatcher functions (literal C→Rust transliteration).
// ===========================================================================

/// `vpx_codec_dec_init_ver` — bind a context to an algorithm.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_dec_init_ver(
    ctx: *mut VpxCodecCtx,
    iface: *mut VpxCodecIface,
    cfg: *const VpxCodecDecCfg,
    flags: VpxCodecFlags,
    ver: i32,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ver != VPX_DECODER_ABI_VERSION {
        res = VPX_CODEC_ABI_MISMATCH;
    } else if ctx.is_null() || iface.is_null() {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*iface).abi_version != VPX_CODEC_INTERNAL_ABI_VERSION {
        res = VPX_CODEC_ABI_MISMATCH;
    } else if (flags & VPX_CODEC_USE_POSTPROC) != 0
        && ((*iface).caps & VPX_CODEC_CAP_POSTPROC) == 0
    {
        res = VPX_CODEC_INCAPABLE;
    } else if (flags & VPX_CODEC_USE_ERROR_CONCEALMENT) != 0
        && ((*iface).caps & VPX_CODEC_CAP_ERROR_CONCEALMENT) == 0
    {
        res = VPX_CODEC_INCAPABLE;
    } else if (flags & VPX_CODEC_USE_INPUT_FRAGMENTS) != 0
        && ((*iface).caps & VPX_CODEC_CAP_INPUT_FRAGMENTS) == 0
    {
        res = VPX_CODEC_INCAPABLE;
    } else if ((*iface).caps & VPX_CODEC_CAP_DECODER) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else {
        // memset(ctx, 0, sizeof(*ctx));
        ptr::write_bytes(ctx as *mut u8, 0, core::mem::size_of::<VpxCodecCtx>());
        (*ctx).iface = iface;
        (*ctx).name = (*iface).name;
        (*ctx).priv_ = ptr::null_mut();
        (*ctx).init_flags = flags;
        (*ctx).config.dec = cfg;

        let init_res = ((*(*ctx).iface).init.expect("iface->init"))(ctx, ptr::null_mut());
        res = init_res;
        if err_is_set(res) {
            (*ctx).err_detail = if !(*ctx).priv_.is_null() {
                (*(*ctx).priv_).err_detail
            } else {
                ptr::null()
            };
            vpx_codec_destroy(ctx);
        }
    }

    save_status(ctx, res)
}

/// `vpx_codec_peek_stream_info` — parse without committing.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_peek_stream_info(
    iface: *mut VpxCodecIface,
    data: *const u8,
    data_sz: c_uint,
    si: *mut VpxCodecStreamInfo,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if iface.is_null()
        || data.is_null()
        || data_sz == 0
        || si.is_null()
        || ((*si).sz as usize) < core::mem::size_of::<VpxCodecStreamInfo>()
    {
        res = VPX_CODEC_INVALID_PARAM;
    } else {
        // Set default/unknown values
        (*si).w = 0;
        (*si).h = 0;

        res = ((*iface).dec.peek_si.expect("iface->dec.peek_si"))(data, data_sz, si);
    }

    res
}

/// `vpx_codec_get_stream_info` — query an active context.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_get_stream_info(
    ctx: *mut VpxCodecCtx,
    si: *mut VpxCodecStreamInfo,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null()
        || si.is_null()
        || ((*si).sz as usize) < core::mem::size_of::<VpxCodecStreamInfo>()
    {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
        res = VPX_CODEC_ERROR;
    } else {
        // Set default/unknown values
        (*si).w = 0;
        (*si).h = 0;

        res = ((*(*ctx).iface).dec.get_si.expect("iface->dec.get_si"))(get_alg_priv(ctx), si);
    }

    save_status(ctx, res)
}

/// `vpx_codec_decode` — feed encoded bytes in.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_decode(
    ctx: *mut VpxCodecCtx,
    data: *const u8,
    data_sz: c_uint,
    user_priv: *mut c_void,
    deadline: i64,
) -> VpxCodecErr {
    let res: VpxCodecErr;
    let _ = deadline;

    // Sanity checks; NULL data ptr allowed if data_sz is 0 too.
    if ctx.is_null()
        || (data.is_null() && data_sz != 0)
        || (!data.is_null() && data_sz == 0)
    {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
        res = VPX_CODEC_ERROR;
    } else {
        res = ((*(*ctx).iface).dec.decode.expect("iface->dec.decode"))(
            get_alg_priv(ctx),
            data,
            data_sz,
            user_priv,
        );
    }

    save_status(ctx, res)
}

/// `vpx_codec_get_frame` — drain decoded pictures.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_get_frame(
    ctx: *mut VpxCodecCtx,
    iter: *mut VpxCodecIter,
) -> *mut VpxImage {
    let img: *mut VpxImage;

    if ctx.is_null()
        || iter.is_null()
        || (*ctx).iface.is_null()
        || (*ctx).priv_.is_null()
    {
        img = ptr::null_mut();
    } else {
        img = ((*(*ctx).iface).dec.get_frame.expect("iface->dec.get_frame"))(
            get_alg_priv(ctx),
            iter,
        );
    }

    img
}

/// `vpx_codec_register_put_frame_cb`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_register_put_frame_cb(
    ctx: *mut VpxCodecCtx,
    cb: VpxCodecPutFrameCbFnT,
    user_priv: *mut c_void,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null() || cb.is_none() {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
        res = VPX_CODEC_ERROR;
    } else if ((*(*ctx).iface).caps & VPX_CODEC_CAP_PUT_FRAME) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else {
        // ctx->priv->dec.put_frame_cb.u.put_frame = cb;
        (*(*ctx).priv_).dec.put_frame_cb.u =
            core::mem::transmute::<VpxCodecPutFrameCbFnT, *mut c_void>(cb);
        (*(*ctx).priv_).dec.put_frame_cb.user_priv = user_priv;
        res = VPX_CODEC_OK;
    }

    save_status(ctx, res)
}

/// `vpx_codec_register_put_slice_cb`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_register_put_slice_cb(
    ctx: *mut VpxCodecCtx,
    cb: VpxCodecPutSliceCbFnT,
    user_priv: *mut c_void,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null() || cb.is_none() {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
        res = VPX_CODEC_ERROR;
    } else if ((*(*ctx).iface).caps & VPX_CODEC_CAP_PUT_SLICE) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else {
        // ctx->priv->dec.put_slice_cb.u.put_slice = cb;
        (*(*ctx).priv_).dec.put_slice_cb.u =
            core::mem::transmute::<VpxCodecPutSliceCbFnT, *mut c_void>(cb);
        (*(*ctx).priv_).dec.put_slice_cb.user_priv = user_priv;
        res = VPX_CODEC_OK;
    }

    save_status(ctx, res)
}

/// `vpx_codec_set_frame_buffer_functions` — external frame-buffer
/// registration.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_set_frame_buffer_functions(
    ctx: *mut VpxCodecCtx,
    cb_get: VpxGetFrameBufferCbFnT,
    cb_release: VpxReleaseFrameBufferCbFnT,
    cb_priv: *mut c_void,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null() || cb_get.is_none() || cb_release.is_none() {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
        res = VPX_CODEC_ERROR;
    } else if ((*(*ctx).iface).caps & VPX_CODEC_CAP_EXTERNAL_FRAME_BUFFER) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else {
        res = ((*(*ctx).iface).dec.set_fb_fn.expect("iface->dec.set_fb_fn"))(
            get_alg_priv(ctx),
            cb_get,
            cb_release,
            cb_priv,
        );
    }

    save_status(ctx, res)
}
