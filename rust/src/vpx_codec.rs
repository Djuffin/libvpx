//! Codec-agnostic public API dispatcher (`vpx_codec_*` entry points).
//!
//! Dispatch routes through the `Decoder` trait stashed on
//! `VpxCodecCtx::trait_obj`. Entry points take `&mut VpxCodecCtx`
//! directly — a reference can't be null, so the C API's
//! null-`ctx` → `INVALID_PARAM` arm is unrepresentable here. The
//! error-query helpers keep `Option<&VpxCodecCtx>` because a `None`
//! there is meaningful (returns a fallback description, matching
//! C's `vpx_codec_error(NULL)`).

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::{c_int, c_void};

use crate::codec::ControlCmd;
use crate::types::{VpxInternalErrorInfo, VpxResult};
use crate::vp8_dx_iface::{
    VP8_COPY_REFERENCE, VP8_SET_POSTPROC, VP8_SET_REFERENCE, VP8D_GET_FRAME_CORRUPTED,
    VP8D_GET_LAST_REF_UPDATES, VP8D_GET_LAST_REF_USED, VPXD_GET_LAST_QUANTIZER, VPXD_SET_DECRYPTOR,
    Vp8PostprocCfg, VpxDecryptInit, VpxRefFrame,
};
use crate::vpx_api::*;

/// `VERSION_PACKED` from `vpx_version.h`. Packed as
/// `major<<16 | minor<<8 | patch`.
pub const VERSION_PACKED: c_int = (1 << 16) | (15 << 8) | 0;

/// `VERSION_STRING_NOSP` from `vpx_version.h`.
pub const VERSION_STRING_NOSP: &str = "v1.15.0";

/// `VERSION_EXTRA` from `vpx_version.h`.
pub const VERSION_EXTRA: &str = "";

pub fn vpx_codec_version() -> c_int {
    VERSION_PACKED
}

pub fn vpx_codec_version_str() -> &'static str {
    VERSION_STRING_NOSP
}

pub fn vpx_codec_version_extra_str() -> &'static str {
    VERSION_EXTRA
}

pub fn vpx_codec_iface_name(iface: Option<&VpxCodecIface>) -> &'static str {
    iface.map_or("<invalid interface>", |i| i.name)
}

pub fn vpx_codec_err_to_string(err: VpxCodecErr) -> &'static str {
    match err {
        VPX_CODEC_OK => "Success",
        VPX_CODEC_ERROR => "Unspecified internal error",
        VPX_CODEC_MEM_ERROR => "Memory allocation error",
        VPX_CODEC_ABI_MISMATCH => "ABI version mismatch",
        VPX_CODEC_INCAPABLE => "Codec does not implement requested capability",
        VPX_CODEC_UNSUP_BITSTREAM => "Bitstream not supported by this decoder",
        VPX_CODEC_UNSUP_FEATURE => "Bitstream required feature not supported by this decoder",
        VPX_CODEC_CORRUPT_FRAME => "Corrupt frame detected",
        VPX_CODEC_INVALID_PARAM => "Invalid parameter",
        VPX_CODEC_LIST_END => "End of iterated list",
    }
}

pub fn vpx_codec_error(ctx: Option<&VpxCodecCtx>) -> &'static str {
    vpx_codec_err_to_string(ctx.map_or(VPX_CODEC_INVALID_PARAM, |c| c.err))
}

/// Returns a description of the most recent error on `ctx`. The
/// variadic detail-formatter from libvpx was dropped during the Rust
/// port (see `translation_summary.md` §4.1) so the "detail" is just
/// the error-code description — identical to [`vpx_codec_error`].
/// Kept as a separate entry point for API symmetry.
pub fn vpx_codec_error_detail(ctx: Option<&VpxCodecCtx>) -> &'static str {
    vpx_codec_error(ctx)
}

/// `vpx_codec_destroy`. Drops the boxed [`Decoder`] trait object,
/// which in turn runs `Vp8Decoder::Drop` (frees the YV12 frame buffer
/// pool and the inner `Vp8dComp` instances; the `Box` drop reclaims
/// the `Vp8AlgPriv` shell).
pub fn vpx_codec_destroy(c: &mut VpxCodecCtx) -> VpxCodecErr {
    if c.iface.is_none() || c.trait_obj.is_none() {
        c.err = VPX_CODEC_ERROR;
        return VPX_CODEC_ERROR;
    }

    let _ = c.trait_obj.take(); // Drops the Box.
    c.iface = None;
    c.name = None;
    c.err = VPX_CODEC_OK;
    VPX_CODEC_OK
}

pub fn vpx_codec_get_caps(iface: Option<&VpxCodecIface>) -> VpxCodecCaps {
    iface.map_or(0, |i| i.caps)
}

/// `vpx_codec_control_`. Accepts the C-style `(ctrl_id, ap)` pair,
/// maps it to a typed [`ControlCmd`], and dispatches via the trait.
/// `ap` stays raw — it's a caller-owned, caller-typed payload.
pub unsafe fn vpx_codec_control_(
    c: &mut VpxCodecCtx,
    ctrl_id: c_int,
    ap: *mut c_void,
) -> VpxCodecErr {
    let res = if ctrl_id == 0 {
        VPX_CODEC_INVALID_PARAM
    } else if c.iface.is_none() || c.trait_obj.is_none() {
        VPX_CODEC_ERROR
    } else {
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

        match cmd {
            Ok(cmd) => match c.trait_obj.as_mut().unwrap().control(cmd) {
                Ok(()) => VPX_CODEC_OK,
                Err(e) => e,
            },
            Err(e) => e,
        }
    };
    c.err = res;
    res
}

/// Records `error` into `info` and returns it as `Err`.
#[inline]
pub fn vpx_internal_error<T>(info: &mut VpxInternalErrorInfo, error: VpxCodecErr) -> VpxResult<T> {
    info.error_code = error;
    Err(error)
}
