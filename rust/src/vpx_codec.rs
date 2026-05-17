//! `vpx/src/vpx_codec.c` — codec-agnostic public API dispatcher.
//!
//! Literal Rust translation of `vpx/src/vpx_codec.c`. Function names,
//! control flow, and pointer arithmetic mirror the C source verbatim.
//! All bodies are `unsafe` because they manipulate raw pointers shaped
//! like the C public API.
//!
//! Public types and constants live in `crate::vpx_api`; this file
//! only carries function bodies.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::{c_int, c_void};
use core::ptr;

use crate::vpx_api::*;
use crate::types::{VpxInternalErrorInfo, VpxResult};

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

// ===========================================================================
// `SAVE_STATUS` macro (vpx_codec.c:22).
//
// Original C: `#define SAVE_STATUS(ctx, var) (ctx ? (ctx->err = var) : var)`
// Stashes the return value into `ctx->err` while also returning it.
// ===========================================================================

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
#[unsafe(no_mangle)]
pub extern "C" fn vpx_codec_version() -> c_int {
    VERSION_PACKED
}

/// `vpx_codec_version_str` (vpx_codec.c:26).
#[unsafe(no_mangle)]
pub extern "C" fn vpx_codec_version_str() -> *const core::ffi::c_char {
    VERSION_STRING_NOSP.as_ptr() as *const core::ffi::c_char
}

/// `vpx_codec_version_extra_str` (vpx_codec.c:28).
#[unsafe(no_mangle)]
pub extern "C" fn vpx_codec_version_extra_str() -> *const core::ffi::c_char {
    VERSION_EXTRA.as_ptr() as *const core::ffi::c_char
}

// ===========================================================================
// Names, capabilities, and string translation (vpx_codec.c:30–64).
// ===========================================================================

/// `vpx_codec_iface_name` (vpx_codec.c:30).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_iface_name(
    iface: *mut VpxCodecIface,
) -> *const core::ffi::c_char {
    if !iface.is_null() {
        (*iface).name
    } else {
        b"<invalid interface>\0".as_ptr() as *const core::ffi::c_char
    }
}

/// `vpx_codec_err_to_string` (vpx_codec.c:34).
#[unsafe(no_mangle)]
pub extern "C" fn vpx_codec_err_to_string(err: VpxCodecErr) -> *const core::ffi::c_char {
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
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_error(
    ctx: *const VpxCodecCtx,
) -> *const core::ffi::c_char {
    if !ctx.is_null() {
        vpx_codec_err_to_string((*ctx).err)
    } else {
        vpx_codec_err_to_string(VPX_CODEC_INVALID_PARAM)
    }
}

/// `vpx_codec_error_detail` (vpx_codec.c:59).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_error_detail(
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

/// `vpx_codec_destroy` (vpx_codec.c:66).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_destroy(ctx: *mut VpxCodecCtx) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null() {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
        res = VPX_CODEC_ERROR;
    } else {
        if let Some(destroy) = (*(*ctx).iface).destroy {
            destroy((*ctx).priv_ as *mut VpxCodecAlgPriv);
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
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_get_caps(iface: *mut VpxCodecIface) -> VpxCodecCaps {
    if !iface.is_null() {
        (*iface).caps
    } else {
        0
    }
}

// ===========================================================================
// Control trampoline (vpx_codec.c:89–114).
// ===========================================================================

/// `vpx_codec_control_` (vpx_codec.c:89).
///
/// The C source uses C varargs (`...` / `va_list`) to forward an
/// arbitrarily typed payload to the per-codec handler. Rust does not
/// have stable variadic function support, so this translation accepts a
/// pre-built `va_list`-shaped pointer (opaque `*mut c_void`) from the
/// caller. A real binding would either declare this as a true C-variadic
/// `extern "C"` function (nightly only) or call through a per-control-ID
/// shim that already extracted the typed payload.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_control_(
    ctx: *mut VpxCodecCtx,
    ctrl_id: c_int,
    ap: *mut c_void,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null() || ctrl_id == 0 {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null()
        || (*ctx).priv_.is_null()
        || (*(*ctx).iface).ctrl_maps.is_null()
    {
        res = VPX_CODEC_ERROR;
    } else {
        let mut local_res = VPX_CODEC_INCAPABLE;

        let mut entry: *mut VpxCodecCtrlFnMap = (*(*ctx).iface).ctrl_maps;
        while (*entry).fn_.is_some() {
            if (*entry).ctrl_id == 0 || (*entry).ctrl_id == ctrl_id {
                // C: va_start(ap, ctrl_id);
                //    res = entry->fn(priv, ap);
                //    va_end(ap);
                let f = (*entry).fn_.unwrap();
                local_res = f((*ctx).priv_ as *mut VpxCodecAlgPriv, ap);
                break;
            }
            entry = entry.add(1);
        }

        res = local_res;
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
