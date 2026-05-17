//! `vpx/src/vpx_encoder.c` — public encoder dispatcher.
//!
//! Literal Rust translation of the thin C wrapper that sits between an
//! application's `vpx_codec_enc_*` calls and the concrete encoder
//! implementation buried inside a `vpx_codec_iface_t`. See
//! `documentation/vp8_files/vpx_encoder.md`.
//!
//! Public types and constants live in `crate::vpx_api`. Function bodies
//! remain here and are re-exported via the API module.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::{c_int, c_ulong, c_void};
use core::ptr;

use crate::vpx_api::*;
use crate::vpx_codec::vpx_codec_destroy;

// ---------------------------------------------------------------------------
// Compile-time toggles, mirroring `vpx_config.h`.
// ---------------------------------------------------------------------------

/// `CONFIG_MULTI_RES_ENCODING` — disabled in the minimal build.
pub const CONFIG_MULTI_RES_ENCODING: bool = false;

// ===========================================================================
// External helpers
// ===========================================================================

unsafe extern "C" {
    fn memcpy(dst: *mut c_void, src: *const c_void, n: usize) -> *mut c_void;
}

// ===========================================================================
// SAVE_STATUS — record the error on the context, return it too.
// ===========================================================================

/// C macro `SAVE_STATUS(ctx, var) ((ctx) ? ((ctx)->err = (var)) : (var))`.
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
// get_alg_priv — cast `ctx->priv` to the algorithm-private type.
// ===========================================================================

#[inline]
unsafe fn get_alg_priv(ctx: *mut VpxCodecCtx) -> *mut VpxCodecAlgPriv {
    (*ctx).priv_ as *mut VpxCodecAlgPriv
}

// ===========================================================================
// vpx_codec_enc_init_ver
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_enc_init_ver(
    ctx: *mut VpxCodecCtx,
    iface: *mut VpxCodecIface,
    cfg: *const VpxCodecEncCfg,
    flags: VpxCodecFlags,
    ver: i32,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ver != VPX_ENCODER_ABI_VERSION {
        res = VPX_CODEC_ABI_MISMATCH;
    } else if ctx.is_null() || iface.is_null() || cfg.is_null() {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*iface).abi_version != VPX_CODEC_INTERNAL_ABI_VERSION {
        res = VPX_CODEC_ABI_MISMATCH;
    } else if ((*iface).caps & VPX_CODEC_CAP_ENCODER) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else if (flags & VPX_CODEC_USE_PSNR) != 0 && ((*iface).caps & VPX_CODEC_CAP_PSNR) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else if (flags & VPX_CODEC_USE_OUTPUT_PARTITION) != 0
        && ((*iface).caps & VPX_CODEC_CAP_OUTPUT_PARTITION) == 0
    {
        res = VPX_CODEC_INCAPABLE;
    } else {
        (*ctx).iface = iface;
        (*ctx).name = (*iface).name;
        (*ctx).priv_ = ptr::null_mut();
        (*ctx).init_flags = flags;
        (*ctx).config.enc = cfg;
        let r = ((*(*ctx).iface).init.expect("iface->init"))(ctx, ptr::null_mut());

        if r != VPX_CODEC_OK {
            // IMPORTANT: ctx->priv->err_detail must be null or point to a
            // string that remains valid after ctx->priv is destroyed, such as
            // a C string literal. This makes it safe to call
            // vpx_codec_error_detail() after vpx_codec_enc_init_ver() failed.
            (*ctx).err_detail = if !(*ctx).priv_.is_null() {
                (*(*ctx).priv_).err_detail
            } else {
                ptr::null()
            };
            vpx_codec_destroy(ctx);
        }
        res = r;
    }

    SAVE_STATUS(ctx, res)
}

// ===========================================================================
// vpx_codec_enc_init_multi_ver
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_enc_init_multi_ver(
    mut ctx: *mut VpxCodecCtx,
    iface: *mut VpxCodecIface,
    mut cfg: *const VpxCodecEncCfg,
    num_enc: i32,
    flags: VpxCodecFlags,
    mut dsf: *const VpxRational,
    ver: i32,
) -> VpxCodecErr {
    let mut res: VpxCodecErr = VPX_CODEC_OK;

    if ver != VPX_ENCODER_ABI_VERSION {
        res = VPX_CODEC_ABI_MISMATCH;
    } else if ctx.is_null()
        || iface.is_null()
        || cfg.is_null()
        || (num_enc > 16 || num_enc < 1)
        || dsf.is_null()
    {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*iface).abi_version != VPX_CODEC_INTERNAL_ABI_VERSION {
        res = VPX_CODEC_ABI_MISMATCH;
    } else if ((*iface).caps & VPX_CODEC_CAP_ENCODER) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else if (flags & VPX_CODEC_USE_PSNR) != 0 && ((*iface).caps & VPX_CODEC_CAP_PSNR) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else if (flags & VPX_CODEC_USE_OUTPUT_PARTITION) != 0
        && ((*iface).caps & VPX_CODEC_CAP_OUTPUT_PARTITION) == 0
    {
        res = VPX_CODEC_INCAPABLE;
    } else {
        let mut i: i32;
        #[allow(unused_assignments, unused_variables)]
        let mut mem_loc_owned: i32 = 0;
        let mut mem_loc: *mut c_void = ptr::null_mut();

        if (*iface).enc.mr_get_mem_loc.is_none() {
            return VPX_CODEC_INCAPABLE;
        }

        res = ((*iface).enc.mr_get_mem_loc.unwrap())(cfg, &mut mem_loc);
        if res == VPX_CODEC_OK {
            i = 0;
            while i < num_enc {
                // Validate down-sampling factor.
                if (*dsf).num < 1 || (*dsf).num > 4096 || (*dsf).den < 1 || (*dsf).den > (*dsf).num
                {
                    res = VPX_CODEC_INVALID_PARAM;
                } else {
                    let mut mr_cfg = VpxCodecPrivEncMrCfg {
                        mr_total_resolutions: num_enc as u32,
                        mr_encoder_id: (num_enc - 1 - i) as u32,
                        mr_down_sampling_factor: *dsf,
                        mr_low_res_mode_info: mem_loc,
                    };

                    (*ctx).iface = iface;
                    (*ctx).name = (*iface).name;
                    (*ctx).priv_ = ptr::null_mut();
                    (*ctx).init_flags = flags;
                    (*ctx).config.enc = cfg;
                    // ctx takes ownership of mr_cfg.mr_low_res_mode_info if
                    // and only if this call succeeds. The first ctx entry in
                    // the array is responsible for freeing the memory.
                    res = ((*(*ctx).iface).init.expect("iface->init"))(ctx, &mut mr_cfg);
                }

                if res != VPX_CODEC_OK {
                    let error_detail = if !(*ctx).priv_.is_null() {
                        (*(*ctx).priv_).err_detail
                    } else {
                        ptr::null()
                    };
                    // Destroy current ctx
                    (*ctx).err_detail = error_detail;
                    vpx_codec_destroy(ctx);

                    // Destroy already allocated high-level ctx
                    while i != 0 {
                        ctx = ctx.offset(-1);
                        (*ctx).err_detail = error_detail;
                        vpx_codec_destroy(ctx);
                        i -= 1;
                    }
                    if CONFIG_MULTI_RES_ENCODING {
                        if mem_loc_owned == 0 {
                            debug_assert!(!mem_loc.is_null());
                            ((*iface).enc.mr_free_mem_loc.unwrap())(mem_loc);
                        }
                    }
                    return SAVE_STATUS(ctx, res);
                }
                if CONFIG_MULTI_RES_ENCODING {
                    mem_loc_owned = 1;
                }
                ctx = ctx.offset(1);
                cfg = cfg.offset(1);
                dsf = dsf.offset(1);
                i += 1;
            }
            ctx = ctx.offset(-1);
        }
    }

    SAVE_STATUS(ctx, res)
}

// ===========================================================================
// vpx_codec_enc_config_default
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_enc_config_default(
    iface: *mut VpxCodecIface,
    cfg: *mut VpxCodecEncCfg,
    usage: u32,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if iface.is_null() || cfg.is_null() || usage != 0 {
        res = VPX_CODEC_INVALID_PARAM;
    } else if ((*iface).caps & VPX_CODEC_CAP_ENCODER) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else {
        debug_assert!((*iface).enc.cfg_map_count == 1);
        // *cfg = iface->enc.cfg_maps->cfg;
        let src = &(*(*iface).enc.cfg_maps).cfg as *const VpxCodecEncCfgValue as *const c_void;
        memcpy(
            cfg as *mut c_void,
            src,
            core::mem::size_of::<VpxCodecEncCfgValue>(),
        );
        res = VPX_CODEC_OK;
    }

    res
}

// ===========================================================================
// FLOATING_POINT_INIT / FLOATING_POINT_RESTORE
// ===========================================================================

#[inline]
unsafe fn FLOATING_POINT_INIT() {}

#[inline]
unsafe fn FLOATING_POINT_RESTORE() {}

// ===========================================================================
// vpx_codec_encode
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_encode(
    mut ctx: *mut VpxCodecCtx,
    mut img: *const VpxImage,
    pts: VpxCodecPts,
    duration: c_ulong,
    flags: VpxEncFrameFlags,
    deadline: VpxEncDeadline,
) -> VpxCodecErr {
    let mut res: VpxCodecErr = VPX_CODEC_OK;

    if ctx.is_null() || (!img.is_null() && duration == 0) {
        res = VPX_CODEC_INVALID_PARAM;
    } else if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
        res = VPX_CODEC_ERROR;
    } else if ((*(*ctx).iface).caps & VPX_CODEC_CAP_ENCODER) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else if core::mem::size_of::<c_ulong>() > core::mem::size_of::<u32>()
        && (duration as u64 > u32::MAX as u64 || deadline as u64 > u32::MAX as u64)
    {
        res = VPX_CODEC_INVALID_PARAM;
    } else {
        let num_enc: u32 = (*(*ctx).priv_).enc.total_encoders;

        // Execute in a normalized floating point environment, if the
        // platform requires it.
        FLOATING_POINT_INIT();

        if num_enc == 1 {
            res = ((*(*ctx).iface).enc.encode.unwrap())(
                get_alg_priv(ctx),
                img,
                pts,
                duration,
                flags,
                deadline,
            );
        } else {
            // Multi-resolution encoding: encode multi-levels in reverse
            // order.
            let mut i: i32;

            ctx = ctx.offset((num_enc - 1) as isize);
            if !img.is_null() {
                img = img.offset((num_enc - 1) as isize);
            }

            i = (num_enc - 1) as i32;
            while i >= 0 {
                res = ((*(*ctx).iface).enc.encode.unwrap())(
                    get_alg_priv(ctx),
                    img,
                    pts,
                    duration,
                    flags,
                    deadline,
                );
                if res != VPX_CODEC_OK {
                    break;
                }

                ctx = ctx.offset(-1);
                if !img.is_null() {
                    img = img.offset(-1);
                }
                i -= 1;
            }
            ctx = ctx.offset(1);
        }

        FLOATING_POINT_RESTORE();
    }

    SAVE_STATUS(ctx, res)
}

// ===========================================================================
// vpx_codec_get_cx_data
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_get_cx_data(
    ctx: *mut VpxCodecCtx,
    iter: *mut VpxCodecIter,
) -> *const VpxCodecCxPkt {
    let mut pkt: *const VpxCodecCxPkt = ptr::null();

    if !ctx.is_null() {
        if iter.is_null() {
            (*ctx).err = VPX_CODEC_INVALID_PARAM;
        } else if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
            (*ctx).err = VPX_CODEC_ERROR;
        } else if ((*(*ctx).iface).caps & VPX_CODEC_CAP_ENCODER) == 0 {
            (*ctx).err = VPX_CODEC_INCAPABLE;
        } else {
            pkt = ((*(*ctx).iface).enc.get_cx_data.unwrap())(get_alg_priv(ctx), iter);
        }
    }

    if !pkt.is_null() && (*pkt).kind == VPX_CODEC_CX_FRAME_PKT {
        // If the application has specified a destination area for the
        // compressed data, and the codec has not placed the data there,
        // and it fits, copy it.
        let priv_: *mut VpxCodecPriv = (*ctx).priv_;
        let dst_buf: *mut core::ffi::c_char =
            (*priv_).enc.cx_data_dst_buf.buf as *mut core::ffi::c_char;

        let raw = (*pkt).data.raw;
        if !dst_buf.is_null()
            && raw.buf != dst_buf as *mut c_void
            && raw.sz + (*priv_).enc.cx_data_pad_before as usize
                + (*priv_).enc.cx_data_pad_after as usize
                <= (*priv_).enc.cx_data_dst_buf.sz
        {
            let modified_pkt: *mut VpxCodecCxPkt = &mut (*priv_).enc.cx_data_pkt;

            memcpy(
                dst_buf.add((*priv_).enc.cx_data_pad_before as usize) as *mut c_void,
                raw.buf as *const c_void,
                raw.sz,
            );
            ptr::copy_nonoverlapping(pkt, modified_pkt, 1);
            (*modified_pkt).data.raw.buf = dst_buf as *mut c_void;
            (*modified_pkt).data.raw.sz +=
                (*priv_).enc.cx_data_pad_before as usize + (*priv_).enc.cx_data_pad_after as usize;
            pkt = modified_pkt;
        }

        let raw2 = (*pkt).data.raw;
        if dst_buf as *mut c_void == raw2.buf {
            (*priv_).enc.cx_data_dst_buf.buf = dst_buf.add(raw2.sz) as *mut c_void;
            (*priv_).enc.cx_data_dst_buf.sz -= raw2.sz;
        }
    }

    pkt
}

// ===========================================================================
// vpx_codec_set_cx_data_buf
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_set_cx_data_buf(
    ctx: *mut VpxCodecCtx,
    buf: *const VpxFixedBuf,
    pad_before: u32,
    pad_after: u32,
) -> VpxCodecErr {
    if ctx.is_null() || (*ctx).priv_.is_null() {
        return VPX_CODEC_INVALID_PARAM;
    }

    if !buf.is_null() {
        (*(*ctx).priv_).enc.cx_data_dst_buf = *buf;
        (*(*ctx).priv_).enc.cx_data_pad_before = pad_before;
        (*(*ctx).priv_).enc.cx_data_pad_after = pad_after;
    } else {
        (*(*ctx).priv_).enc.cx_data_dst_buf.buf = ptr::null_mut();
        (*(*ctx).priv_).enc.cx_data_dst_buf.sz = 0;
        (*(*ctx).priv_).enc.cx_data_pad_before = 0;
        (*(*ctx).priv_).enc.cx_data_pad_after = 0;
    }

    VPX_CODEC_OK
}

// ===========================================================================
// vpx_codec_get_preview_frame
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_get_preview_frame(ctx: *mut VpxCodecCtx) -> *const VpxImage {
    let mut img: *mut VpxImage = ptr::null_mut();

    if !ctx.is_null() {
        if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
            (*ctx).err = VPX_CODEC_ERROR;
        } else if ((*(*ctx).iface).caps & VPX_CODEC_CAP_ENCODER) == 0 {
            (*ctx).err = VPX_CODEC_INCAPABLE;
        } else if (*(*ctx).iface).enc.get_preview.is_none() {
            (*ctx).err = VPX_CODEC_INCAPABLE;
        } else {
            img = ((*(*ctx).iface).enc.get_preview.unwrap())(get_alg_priv(ctx));
        }
    }

    img
}

// ===========================================================================
// vpx_codec_get_global_headers
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_get_global_headers(ctx: *mut VpxCodecCtx) -> *mut VpxFixedBuf {
    let mut buf: *mut VpxFixedBuf = ptr::null_mut();

    if !ctx.is_null() {
        if (*ctx).iface.is_null() || (*ctx).priv_.is_null() {
            (*ctx).err = VPX_CODEC_ERROR;
        } else if ((*(*ctx).iface).caps & VPX_CODEC_CAP_ENCODER) == 0 {
            (*ctx).err = VPX_CODEC_INCAPABLE;
        } else if (*(*ctx).iface).enc.get_glob_hdrs.is_none() {
            (*ctx).err = VPX_CODEC_INCAPABLE;
        } else {
            buf = ((*(*ctx).iface).enc.get_glob_hdrs.unwrap())(get_alg_priv(ctx));
        }
    }

    buf
}

// ===========================================================================
// vpx_codec_enc_config_set
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_enc_config_set(
    ctx: *mut VpxCodecCtx,
    cfg: *const VpxCodecEncCfg,
) -> VpxCodecErr {
    let res: VpxCodecErr;

    if ctx.is_null() || (*ctx).iface.is_null() || (*ctx).priv_.is_null() || cfg.is_null() {
        res = VPX_CODEC_INVALID_PARAM;
    } else if ((*(*ctx).iface).caps & VPX_CODEC_CAP_ENCODER) == 0 {
        res = VPX_CODEC_INCAPABLE;
    } else {
        res = ((*(*ctx).iface).enc.cfg_set.unwrap())(get_alg_priv(ctx), cfg);
    }

    SAVE_STATUS(ctx, res)
}

// ===========================================================================
// vpx_codec_pkt_list_add
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_pkt_list_add(
    list: *mut VpxCodecPktList,
    pkt: *const VpxCodecCxPkt,
) -> c_int {
    if (*list).cnt < (*list).max {
        // list->pkts[list->cnt++] = *pkt;
        let slot = (*list).pkts.as_mut_ptr().add((*list).cnt as usize);
        ptr::copy_nonoverlapping(pkt, slot, 1);
        (*list).cnt += 1;
        return 0;
    }

    1
}

// ===========================================================================
// vpx_codec_pkt_list_get
// ===========================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn vpx_codec_pkt_list_get(
    list: *mut VpxCodecPktList,
    iter: *mut VpxCodecIter,
) -> *const VpxCodecCxPkt {
    let mut pkt: *const VpxCodecCxPkt;

    if (*iter).is_null() {
        *iter = (*list).pkts.as_ptr() as VpxCodecIter;
    }

    pkt = *iter as *const VpxCodecCxPkt;

    let base = (*list).pkts.as_ptr();
    let offset = pkt.offset_from(base) as usize;
    if offset < (*list).cnt as usize {
        *iter = pkt.add(1) as VpxCodecIter;
    } else {
        pkt = ptr::null();
    }

    pkt
}
