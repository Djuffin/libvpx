//! Decoder-side public-API dispatcher. Each entry point validates
//! arguments + capability bits, then dispatches through the
//! `Decoder` trait stashed on `VpxCodecCtx::trait_obj`.
//!
//! Public types and constants live in `crate::vpx_api`.

#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use core::ffi::{c_uint, c_void};
use core::ptr;

use crate::vpx_api::*;
use crate::vpx_codec::vpx_codec_destroy;
use crate::vp8_dx_iface::Vp8Decoder;
use crate::codec::Decoder;

// ===========================================================================
// Helpers
// ===========================================================================

/// Stash `var` into `ctx->err` iff `ctx` is non-NULL, then return `var`.
#[inline]
unsafe fn save_status(ctx: *mut VpxCodecCtx, var: VpxCodecErr) -> VpxCodecErr {
    if !ctx.is_null() {
        (*ctx).err = var;
    }
    var
}

// ===========================================================================
// Public-API dispatcher functions (literal C→Rust transliteration).
// ===========================================================================

/// `vpx_codec_dec_init_ver` — bind a context to an algorithm.

pub unsafe fn vpx_codec_dec_init_ver(
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

        // VP8 is currently the only algorithm; when VP9 lands a small
        // per-algo registry will choose between constructors.
        match Vp8Decoder::new(flags) {
            Ok(dec) => {
                // Mirror the threads config into the priv_ if cfg
                // was supplied (preserves the legacy iface-init
                // behavior of copying cfg into Vp8AlgPriv).
                if !cfg.is_null() {
                    (*dec.as_ptr()).cfg = *cfg;
                }
                // priv_ is set as a non-null sentinel for
                // initialized-state checks elsewhere; it points at the
                // same Vp8AlgPriv that the trait object owns.
                (*ctx).priv_ = dec.as_ptr() as *mut VpxCodecPriv;
                let boxed = Box::new(dec);
                (*ctx).trait_obj = Box::into_raw(boxed) as *mut c_void;
                res = VPX_CODEC_OK;
            }
            Err(e) => {
                res = e;
                vpx_codec_destroy(ctx);
            }
        }
    }

    save_status(ctx, res)
}

/// `vpx_codec_peek_stream_info` — parse without committing. The
/// `iface` parameter is kept only for null-check + param validation;
/// dispatch uses `Decoder::peek_stream_info`.

pub unsafe fn vpx_codec_peek_stream_info(
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

        let slice = core::slice::from_raw_parts(data, data_sz as usize);
        match Vp8Decoder::peek_stream_info(slice) {
            Ok(out) => {
                *si = out;
                res = VPX_CODEC_OK;
            }
            Err(e) => {
                res = e;
            }
        }
    }

    res
}

/// `vpx_codec_get_stream_info` — query an active context.

pub unsafe fn vpx_codec_get_stream_info(
    ctx: *mut VpxCodecCtx,
    si: *mut VpxCodecStreamInfo,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null()
        || si.is_null()
        || ((*si).sz as usize) < core::mem::size_of::<VpxCodecStreamInfo>()
    {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).trait_obj.is_null() {
        res = VPX_CODEC_ERROR;
    } else {
        // Set default/unknown values
        (*si).w = 0;
        (*si).h = 0;

        let dec = &*((*ctx).trait_obj as *const Vp8Decoder);
        res = match dec.stream_info() {
            Ok(out) => {
                *si = out;
                VPX_CODEC_OK
            }
            Err(e) => e,
        };
    }

    save_status(ctx, res)
}

/// `vpx_codec_decode` — feed encoded bytes in.

pub unsafe fn vpx_codec_decode(
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
    } else if (*ctx).iface.is_null() || (*ctx).trait_obj.is_null() {
        res = VPX_CODEC_ERROR;
    } else {
        let dec = &mut *((*ctx).trait_obj as *mut Vp8Decoder);
        dec.set_user_priv(user_priv);
        let slice = if data.is_null() {
            &[][..]
        } else {
            core::slice::from_raw_parts(data, data_sz as usize)
        };
        res = match dec.decode(slice, core::time::Duration::ZERO) {
            Ok(()) => VPX_CODEC_OK,
            Err(e) => e,
        };
    }

    save_status(ctx, res)
}

/// `vpx_codec_get_frame` — drain decoded pictures. The user's `iter`
/// is mirrored to the trait's internal iter: first call after a decode
/// returns the new image and toggles iter; subsequent calls return null.

pub unsafe fn vpx_codec_get_frame(
    ctx: *mut VpxCodecCtx,
    iter: *mut VpxCodecIter,
) -> *mut VpxImage {
    let img: *mut VpxImage;

    if ctx.is_null()
        || iter.is_null()
        || (*ctx).iface.is_null()
        || (*ctx).trait_obj.is_null()
    {
        img = ptr::null_mut();
    } else if !(*iter).is_null() {
        // Caller's iter already advanced past the single VP8 output —
        // no more frames in the queue.
        img = ptr::null_mut();
    } else {
        let dec = &mut *((*ctx).trait_obj as *mut Vp8Decoder);
        img = match dec.get_frame() {
            Some(image) => {
                // Mark user's iter as advanced for C-side drain loops.
                *iter = image as *const VpxImage as *const c_void;
                image as *const VpxImage as *mut VpxImage
            }
            None => ptr::null_mut(),
        };
    }

    img
}

/// `vpx_codec_register_put_frame_cb`. The VP8 build lacks
/// `VPX_CODEC_CAP_PUT_FRAME`, so this always returns `INCAPABLE`.

pub unsafe fn vpx_codec_register_put_frame_cb(
    ctx: *mut VpxCodecCtx,
    cb: VpxCodecPutFrameCbFnT,
    _user_priv: *mut c_void,
) -> VpxCodecErr {
    let res: VpxCodecErr;
    if ctx.is_null() || cb.is_none() {
        res = VPX_CODEC_INVALID_PARAM;
    } else {
        res = VPX_CODEC_INCAPABLE;
    }
    save_status(ctx, res)
}

/// `vpx_codec_register_put_slice_cb`. The VP8 build lacks
/// `VPX_CODEC_CAP_PUT_SLICE`, so this always returns `INCAPABLE`.

pub unsafe fn vpx_codec_register_put_slice_cb(
    ctx: *mut VpxCodecCtx,
    cb: VpxCodecPutSliceCbFnT,
    _user_priv: *mut c_void,
) -> VpxCodecErr {
    let res: VpxCodecErr;
    if ctx.is_null() || cb.is_none() {
        res = VPX_CODEC_INVALID_PARAM;
    } else {
        res = VPX_CODEC_INCAPABLE;
    }
    save_status(ctx, res)
}

/// `vpx_codec_set_frame_buffer_functions`. The VP8 build lacks
/// `VPX_CODEC_CAP_EXTERNAL_FRAME_BUFFER`; the trait-based
/// `FrameBufferAllocator` hook in `crate::codec` is the future
/// replacement.

pub unsafe fn vpx_codec_set_frame_buffer_functions(
    ctx: *mut VpxCodecCtx,
    cb_get: VpxGetFrameBufferCbFnT,
    cb_release: VpxReleaseFrameBufferCbFnT,
    _cb_priv: *mut c_void,
) -> VpxCodecErr {
    let res: VpxCodecErr;
    if ctx.is_null() || cb_get.is_none() || cb_release.is_none() {
        res = VPX_CODEC_INVALID_PARAM;
    } else {
        res = VPX_CODEC_INCAPABLE;
    }
    save_status(ctx, res)
}
