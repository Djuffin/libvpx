//! Codec-agnostic public API dispatcher (`vpx_codec_*` entry points).
//!
//! Dispatch routes through the `Decoder` trait stashed on
//! `VpxCodecCtx::trait_obj`. Function signatures take
//! `Option<&mut VpxCodecCtx>` etc. so the null-pointer arms of the
//! original C API are still expressible as `None`.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::{c_int, c_void};
use core::ptr;

use crate::vpx_api::*;
use crate::types::{VpxInternalErrorInfo, VpxResult};
use crate::vp8_dx_iface::{
    VpxDecryptInit, VpxRefFrame, Vp8PostprocCfg,
    VP8_SET_REFERENCE, VP8_COPY_REFERENCE, VP8_SET_POSTPROC,
    VP8D_GET_LAST_REF_UPDATES, VP8D_GET_FRAME_CORRUPTED, VP8D_GET_LAST_REF_USED,
    VPXD_GET_LAST_QUANTIZER, VPXD_SET_DECRYPTOR,
};
use crate::codec::ControlCmd;

/// `VERSION_PACKED` from `vpx_version.h`. Packed as
/// `major<<16 | minor<<8 | patch`.
pub const VERSION_PACKED: c_int = (1 << 16) | (15 << 8) | 0;

/// `VERSION_STRING_NOSP` from `vpx_version.h`.
pub const VERSION_STRING_NOSP: &[u8] = b"v1.15.0\0";

/// `VERSION_EXTRA` from `vpx_version.h`.
pub const VERSION_EXTRA: &[u8] = b"\0";

/// Stash `var` into `ctx.err` (if `ctx` is `Some`) and return it.
/// Rust spelling of libvpx's `SAVE_STATUS` macro.
#[inline]
fn SAVE_STATUS(ctx: Option<&mut VpxCodecCtx>, var: VpxCodecErr) -> VpxCodecErr {
    if let Some(c) = ctx {
        c.err = var;
    }
    var
}

pub fn vpx_codec_version() -> c_int {
    VERSION_PACKED
}

pub fn vpx_codec_version_str() -> *const core::ffi::c_char {
    VERSION_STRING_NOSP.as_ptr() as *const core::ffi::c_char
}

pub fn vpx_codec_version_extra_str() -> *const core::ffi::c_char {
    VERSION_EXTRA.as_ptr() as *const core::ffi::c_char
}

pub fn vpx_codec_iface_name(
    iface: Option<&VpxCodecIface>,
) -> *const core::ffi::c_char {
    match iface {
        Some(i) => i.name,
        None => b"<invalid interface>\0".as_ptr() as *const core::ffi::c_char,
    }
}

pub fn vpx_codec_err_to_string(err: VpxCodecErr) -> *const core::ffi::c_char {
    let s: &[u8] = match err {
        VPX_CODEC_OK => b"Success\0",
        VPX_CODEC_ERROR => b"Unspecified internal error\0",
        VPX_CODEC_MEM_ERROR => b"Memory allocation error\0",
        VPX_CODEC_ABI_MISMATCH => b"ABI version mismatch\0",
        VPX_CODEC_INCAPABLE => b"Codec does not implement requested capability\0",
        VPX_CODEC_UNSUP_BITSTREAM => b"Bitstream not supported by this decoder\0",
        VPX_CODEC_UNSUP_FEATURE => {
            b"Bitstream required feature not supported by this decoder\0"
        }
        VPX_CODEC_CORRUPT_FRAME => b"Corrupt frame detected\0",
        VPX_CODEC_INVALID_PARAM => b"Invalid parameter\0",
        VPX_CODEC_LIST_END => b"End of iterated list\0",
    };
    s.as_ptr() as *const core::ffi::c_char
}

pub fn vpx_codec_error(
    ctx: Option<&VpxCodecCtx>,
) -> *const core::ffi::c_char {
    match ctx {
        Some(c) => vpx_codec_err_to_string(c.err),
        None => vpx_codec_err_to_string(VPX_CODEC_INVALID_PARAM),
    }
}

pub unsafe fn vpx_codec_error_detail(
    ctx: Option<&VpxCodecCtx>,
) -> *const core::ffi::c_char {
    let Some(c) = ctx else { return ptr::null() };
    if c.err == VPX_CODEC_OK {
        return ptr::null();
    }
    if !c.priv_.is_null() {
        (*c.priv_).err_detail
    } else {
        c.err_detail
    }
}

/// `vpx_codec_destroy`. Drops the boxed [`Decoder`] trait object,
/// which in turn runs `Vp8Decoder::Drop` → `vp8_destroy` → `vpx_free`.
pub fn vpx_codec_destroy(ctx: Option<&mut VpxCodecCtx>) -> VpxCodecErr {
    let Some(c) = ctx else { return VPX_CODEC_INVALID_PARAM };

    if c.iface.is_none() || c.priv_.is_null() {
        c.err = VPX_CODEC_ERROR;
        return VPX_CODEC_ERROR;
    }

    let _ = c.trait_obj.take(); // Drops the Box.
    c.iface = None;
    c.name = ptr::null();
    c.priv_ = ptr::null_mut();
    c.err = VPX_CODEC_OK;
    VPX_CODEC_OK
}

pub fn vpx_codec_get_caps(iface: Option<&VpxCodecIface>) -> VpxCodecCaps {
    iface.map(|i| i.caps).unwrap_or(0)
}

/// `vpx_codec_control_`. Accepts the C-style `(ctrl_id, ap)` pair,
/// maps it to a typed [`ControlCmd`], and dispatches via the trait.
/// `ap` stays raw — it's a caller-owned, caller-typed payload.
pub unsafe fn vpx_codec_control_(
    ctx: Option<&mut VpxCodecCtx>,
    ctrl_id: c_int,
    ap: *mut c_void,
) -> VpxCodecErr {
    let Some(c) = ctx else { return VPX_CODEC_INVALID_PARAM };

    if ctrl_id == 0 {
        c.err = VPX_CODEC_INVALID_PARAM;
        return VPX_CODEC_INVALID_PARAM;
    }
    if c.iface.is_none() || c.trait_obj.is_none() {
        c.err = VPX_CODEC_ERROR;
        return VPX_CODEC_ERROR;
    }

    // Build the typed ControlCmd from the legacy (ctrl_id, ap) pair.
    // Ap is null-checked per-arm; the SET_DECRYPTOR arm accepts null
    // (means "clear").
    let cmd: Result<ControlCmd<'_>, VpxCodecErr> = match ctrl_id {
        VP8_SET_REFERENCE if !ap.is_null() => {
            Ok(ControlCmd::SetReference(&*(ap as *const VpxRefFrame)))
        }
        VP8_COPY_REFERENCE if !ap.is_null() => {
            Ok(ControlCmd::CopyReference(&mut *(ap as *mut VpxRefFrame)))
        }
        VP8_SET_POSTPROC if !ap.is_null() => {
            Ok(ControlCmd::SetPostproc(*(ap as *const Vp8PostprocCfg)))
        }
        VP8D_GET_LAST_REF_UPDATES if !ap.is_null() => {
            Ok(ControlCmd::GetLastRefUpdates(&mut *(ap as *mut i32)))
        }
        VP8D_GET_FRAME_CORRUPTED if !ap.is_null() => {
            Ok(ControlCmd::GetFrameCorrupted(&mut *(ap as *mut i32)))
        }
        VP8D_GET_LAST_REF_USED if !ap.is_null() => {
            Ok(ControlCmd::GetLastRefUsed(&mut *(ap as *mut i32)))
        }
        VPXD_GET_LAST_QUANTIZER if !ap.is_null() => {
            Ok(ControlCmd::GetLastQuantizer(&mut *(ap as *mut i32)))
        }
        VPXD_SET_DECRYPTOR => {
            let init = if ap.is_null() {
                None
            } else {
                Some(&*(ap as *const VpxDecryptInit))
            };
            Ok(ControlCmd::SetDecryptor(init))
        }
        VP8_SET_REFERENCE
        | VP8_COPY_REFERENCE
        | VP8_SET_POSTPROC
        | VP8D_GET_LAST_REF_UPDATES
        | VP8D_GET_FRAME_CORRUPTED
        | VP8D_GET_LAST_REF_USED
        | VPXD_GET_LAST_QUANTIZER => Err(VPX_CODEC_INVALID_PARAM),
        _ => Err(VPX_CODEC_INCAPABLE),
    };

    let res = match cmd {
        Ok(cmd) => match c.trait_obj.as_mut().unwrap().control(cmd) {
            Ok(()) => VPX_CODEC_OK,
            Err(e) => e,
        },
        Err(e) => e,
    };
    c.err = res;
    res
}

/// Records `error` into `info` and returns it as `Err`.
#[inline]
pub unsafe fn vpx_internal_error<T>(
    info: *mut VpxInternalErrorInfo,
    error: VpxCodecErr,
) -> VpxResult<T> {
    (*info).error_code = error;
    Err(error)
}
