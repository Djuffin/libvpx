//! Codec-agnostic public API dispatcher (`vpx_codec_*` entry points).
//!
//! Bodies remain `unsafe` because they manipulate raw pointers shaped
//! like the original C public API. Dispatch goes through the
//! `Decoder` trait stashed on `VpxCodecCtx::trait_obj`.
//!
//! Public types and constants live in `crate::vpx_api`.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::{c_int, c_void};
use core::ptr;

use crate::vpx_api::*;
use crate::types::{VpxInternalErrorInfo, VpxResult};
use crate::vp8_dx_iface::{
    Vp8Decoder, VpxDecryptInit, VpxRefFrame, Vp8PostprocCfg,
    VP8_SET_REFERENCE, VP8_COPY_REFERENCE, VP8_SET_POSTPROC,
    VP8D_GET_LAST_REF_UPDATES, VP8D_GET_FRAME_CORRUPTED, VP8D_GET_LAST_REF_USED,
    VPXD_GET_LAST_QUANTIZER, VPXD_SET_DECRYPTOR,
};
use crate::codec::{ControlCmd, Decoder};

// ===========================================================================
// `vpx_version.h` macros (generated at configure time). Stubs for the
// minimal build.
// ===========================================================================

/// `VERSION_PACKED` from `vpx_version.h`. Packed as
/// `major<<16 | minor<<8 | patch`.
pub const VERSION_PACKED: c_int = (1 << 16) | (15 << 8) | 0;

/// `VERSION_STRING_NOSP` from `vpx_version.h`.
pub const VERSION_STRING_NOSP: &[u8] = b"v1.15.0\0";

/// `VERSION_EXTRA` from `vpx_version.h`.
pub const VERSION_EXTRA: &[u8] = b"\0";

/// Stash `var` into `ctx->err` (if `ctx` non-null) and return it.
/// Rust spelling of libvpx's `SAVE_STATUS` macro.
#[inline]
unsafe fn SAVE_STATUS(ctx: *mut VpxCodecCtx, var: VpxCodecErr) -> VpxCodecErr {
    if !ctx.is_null() {
        (*ctx).err = var;
        var
    } else {
        var
    }
}

// ===========================================================================
// Version queries (vpx_codec.c:24–28).
// ===========================================================================

/// `vpx_codec_version` (vpx_codec.c:24).

pub fn vpx_codec_version() -> c_int {
    VERSION_PACKED
}

/// `vpx_codec_version_str` (vpx_codec.c:26).

pub fn vpx_codec_version_str() -> *const core::ffi::c_char {
    VERSION_STRING_NOSP.as_ptr() as *const core::ffi::c_char
}

/// `vpx_codec_version_extra_str` (vpx_codec.c:28).

pub fn vpx_codec_version_extra_str() -> *const core::ffi::c_char {
    VERSION_EXTRA.as_ptr() as *const core::ffi::c_char
}

// ===========================================================================
// Names, capabilities, and string translation (vpx_codec.c:30–64).
// ===========================================================================

/// `vpx_codec_iface_name` (vpx_codec.c:30).

pub unsafe fn vpx_codec_iface_name(
    iface: *mut VpxCodecIface,
) -> *const core::ffi::c_char {
    if !iface.is_null() {
        (*iface).name
    } else {
        b"<invalid interface>\0".as_ptr() as *const core::ffi::c_char
    }
}

/// `vpx_codec_err_to_string` (vpx_codec.c:34).

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

/// `vpx_codec_error` (vpx_codec.c:54).

pub unsafe fn vpx_codec_error(
    ctx: *const VpxCodecCtx,
) -> *const core::ffi::c_char {
    if !ctx.is_null() {
        vpx_codec_err_to_string((*ctx).err)
    } else {
        vpx_codec_err_to_string(VPX_CODEC_INVALID_PARAM)
    }
}

/// `vpx_codec_error_detail` (vpx_codec.c:59).

pub unsafe fn vpx_codec_error_detail(
    ctx: *const VpxCodecCtx,
) -> *const core::ffi::c_char {
    if !ctx.is_null() && (*ctx).err != VPX_CODEC_OK {
        if !(*ctx).priv_.is_null() {
            return (*(*ctx).priv_).err_detail;
        } else {
            return (*ctx).err_detail;
        }
    }

    ptr::null()
}

// ===========================================================================
// Destructor (vpx_codec.c:66–83).
// ===========================================================================

/// `vpx_codec_destroy` (vpx_codec.c:66). Ownership of `Vp8AlgPriv`
/// lives in the `Vp8Decoder` boxed at `(*ctx).trait_obj`. Dropping
/// the box runs `vp8_destroy` → `vpx_free`.

pub unsafe fn vpx_codec_destroy(ctx: *mut VpxCodecCtx) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null() {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
        res = VPX_CODEC_ERROR;
    } else {
        if !(*ctx).trait_obj.is_null() {
            // Reclaim the Box and drop it. Drop on Vp8Decoder calls
            // vp8_destroy which frees the underlying Vp8AlgPriv.
            let dec = Box::from_raw((*ctx).trait_obj as *mut Vp8Decoder);
            drop(dec);
            (*ctx).trait_obj = ptr::null_mut();
        }

        (*ctx).iface = ptr::null_mut();
        (*ctx).name = ptr::null();
        (*ctx).priv_ = ptr::null_mut();
        res = VPX_CODEC_OK;
    }

    SAVE_STATUS(ctx, res)
}

// ===========================================================================
// Capabilities query (vpx_codec.c:85–87).
// ===========================================================================

/// `vpx_codec_get_caps` (vpx_codec.c:85).

pub unsafe fn vpx_codec_get_caps(iface: *mut VpxCodecIface) -> VpxCodecCaps {
    if !iface.is_null() {
        (*iface).caps
    } else {
        0
    }
}

/// `vpx_codec_control_`. Accepts the C-style `(ctrl_id, ap)` pair,
/// maps it to a typed [`ControlCmd`], and dispatches via the trait.
pub unsafe fn vpx_codec_control_(
    ctx: *mut VpxCodecCtx,
    ctrl_id: c_int,
    ap: *mut c_void,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null() || ctrl_id == 0 {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).trait_obj.is_null() {
        res = VPX_CODEC_ERROR;
    } else {
        // Map the legacy ctrl_id + opaque payload to a typed
        // `ControlCmd` and dispatch through the trait.
        let dec = &mut *((*ctx).trait_obj as *mut Vp8Decoder);
        let cmd = match ctrl_id {
            VP8_SET_REFERENCE => {
                if ap.is_null() {
                    return SAVE_STATUS(ctx, VPX_CODEC_INVALID_PARAM);
                }
                ControlCmd::SetReference(&*(ap as *const VpxRefFrame))
            }
            VP8_COPY_REFERENCE => {
                if ap.is_null() {
                    return SAVE_STATUS(ctx, VPX_CODEC_INVALID_PARAM);
                }
                ControlCmd::CopyReference(&mut *(ap as *mut VpxRefFrame))
            }
            VP8_SET_POSTPROC => {
                if ap.is_null() {
                    return SAVE_STATUS(ctx, VPX_CODEC_INVALID_PARAM);
                }
                ControlCmd::SetPostproc(*(ap as *const Vp8PostprocCfg))
            }
            VP8D_GET_LAST_REF_UPDATES => {
                if ap.is_null() {
                    return SAVE_STATUS(ctx, VPX_CODEC_INVALID_PARAM);
                }
                ControlCmd::GetLastRefUpdates(&mut *(ap as *mut i32))
            }
            VP8D_GET_FRAME_CORRUPTED => {
                if ap.is_null() {
                    return SAVE_STATUS(ctx, VPX_CODEC_INVALID_PARAM);
                }
                ControlCmd::GetFrameCorrupted(&mut *(ap as *mut i32))
            }
            VP8D_GET_LAST_REF_USED => {
                if ap.is_null() {
                    return SAVE_STATUS(ctx, VPX_CODEC_INVALID_PARAM);
                }
                ControlCmd::GetLastRefUsed(&mut *(ap as *mut i32))
            }
            VPXD_GET_LAST_QUANTIZER => {
                if ap.is_null() {
                    return SAVE_STATUS(ctx, VPX_CODEC_INVALID_PARAM);
                }
                ControlCmd::GetLastQuantizer(&mut *(ap as *mut i32))
            }
            VPXD_SET_DECRYPTOR => {
                let init = if ap.is_null() {
                    None
                } else {
                    Some(&*(ap as *const VpxDecryptInit))
                };
                ControlCmd::SetDecryptor(init)
            }
            _ => return SAVE_STATUS(ctx, VPX_CODEC_INCAPABLE),
        };
        res = match dec.control(cmd) {
            Ok(()) => VPX_CODEC_OK,
            Err(e) => e,
        };
    }

    SAVE_STATUS(ctx, res)
}

// ===========================================================================
// Internal error path (vpx_codec.c:116–134).
// ===========================================================================

/// Records `error` into `info` and returns it as `Err`.
#[inline]
pub unsafe fn vpx_internal_error<T>(
    info: *mut VpxInternalErrorInfo,
    error: VpxCodecErr,
) -> VpxResult<T> {
    (*info).error_code = error;
    Err(error)
}
