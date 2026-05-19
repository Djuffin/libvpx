//! `vp8/decoder/detokenize.c` — residual coefficient parsing.
//!
//! Literal C-to-Rust transliteration of the per-MB token decoder. The
//! two public entry points (`vp8_reset_mb_tokens_context`,
//! `vp8_decode_mb_tokens`) mirror their C counterparts byte-for-byte;
//! `GetCoeffs` and `GetSigned` remain private file-static helpers.
//!
//! Control flow, indexing arithmetic, and the asymmetric Y2 context
//! handling are preserved exactly as in the C source. See
//! `documentation/vp8_files/detokenize.md` for the line-by-line rationale.

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use crate::tables::Prob;
use crate::types::{
    BD_VALUE_BITS, BdValue, BoolDecoder, EntropyContext, EntropyContextPlanes, FrameContext,
    Macroblockd, Vp8dComp,
};

// ===========================================================================
// Extern declarations for symbols defined in other translation units.
// ===========================================================================

use crate::dboolhuff::{vp8dx_bool_decoder_fill, vp8dx_decode_bool};

// ===========================================================================
// File-scope tables — direct ports of `detokenize.c:35-47`.
// ===========================================================================

/// `kBands[16 + 1]` (`detokenize.c:35-38`) — RFC 6386 §13.3 band-remap
/// table, with a 17th sentinel entry.
static kBands: [u8; 16 + 1] = [
    0, 1, 2, 3, 6, 4, 5, 6, 6, 6, 6, 6, 6, 6, 6, 7, 0, /* extra entry as sentinel */
];

/// `kCat3` (`detokenize.c:40`) — DCT_VAL_CATEGORY3 extra-bit probabilities.
static kCat3: [u8; 4] = [173, 148, 140, 0];
/// `kCat4` (`detokenize.c:41`) — DCT_VAL_CATEGORY4 extra-bit probabilities.
static kCat4: [u8; 5] = [176, 155, 140, 135, 0];
/// `kCat5` (`detokenize.c:42`) — DCT_VAL_CATEGORY5 extra-bit probabilities.
static kCat5: [u8; 6] = [180, 157, 141, 134, 130, 0];
/// `kCat6` (`detokenize.c:43-44`) — DCT_VAL_CATEGORY6 extra-bit probabilities.
static kCat6: [u8; 12] = [254, 254, 243, 230, 196, 177, 153, 140, 133, 130, 129, 0];

/// `kCat3456[]` (`detokenize.c:45`) — pointer-indexed view of the four
/// category tables. Indexed by `cat = 2*bit1 + bit0`. Wrapped in a
/// `Sync` newtype because raw pointers are `!Sync` by default; the
/// pointees are `'static` data, so this is sound.
#[repr(transparent)]
struct CatPtrs([*const u8; 4]);
unsafe impl Sync for CatPtrs {}

static kCat3456: CatPtrs = CatPtrs([
    kCat3.as_ptr(),
    kCat4.as_ptr(),
    kCat5.as_ptr(),
    kCat6.as_ptr(),
]);

/// `kZigzag[16]` (`detokenize.c:46-47`) — inverse zig-zag table.
static kZigzag: [u8; 16] = [0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15];

const NUM_PROBAS: usize = 11;
const NUM_CTX: usize = 3;

/// `ProbaArray` (`detokenize.c:54`) — pointer-to-array such that
/// `prob[band][ctx][node]` indexes naturally. The underlying storage is
/// `FRAME_CONTEXT.coef_probs[block_type]`, a `[[[Prob; 11]; 3]; 8]`.
type ProbaArray = *const [[Prob; NUM_PROBAS]; NUM_CTX];

// ===========================================================================
// `VP8GetBit` — thin macro rename (`detokenize.c:49`).
// ===========================================================================

/// `VP8GetBit` (`detokenize.c:49`) — alias macro for `vp8dx_decode_bool`.
#[inline(always)]
unsafe fn VP8GetBit(br: *mut BoolDecoder<'static>, probability: i32) -> i32 {
    vp8dx_decode_bool(br, probability)
}

// ===========================================================================
// `GetSigned` — sign-bit folded into a streamlined bool decoder
// (`detokenize.c:58-79`).
// ===========================================================================

/// `GetSigned` (`detokenize.c:58-79`) — decode one equiprobable sign bit
/// and return `+/- value_to_sign`. Renormalization is open-coded
/// (probability is always 128, so the shift is always 1).
///
/// With corrupt / fuzzed streams the calculation of `br->value` may
/// overflow (b/148271109); we use `wrapping_*` to mirror the
/// `VPX_NO_UNSIGNED_OVERFLOW_CHECK` attribute on the C source.
unsafe fn GetSigned(br: *mut BoolDecoder<'static>, value_to_sign: i32) -> i32 {
    let split: i32 = ((*br).range as i32 + 1) >> 1;
    let bigsplit: BdValue = (split as BdValue) << (BD_VALUE_BITS - 8);
    let v: i32;

    if (*br).count < 0 {
        vp8dx_bool_decoder_fill(br);
    }

    if (*br).value < bigsplit {
        (*br).range = split as u32;
        v = value_to_sign;
    } else {
        (*br).range = (*br).range - split as u32;
        (*br).value = (*br).value.wrapping_sub(bigsplit);
        v = -value_to_sign;
    }
    (*br).range = (*br).range.wrapping_add((*br).range);
    (*br).value = (*br).value.wrapping_add((*br).value);
    (*br).count -= 1;

    v
}

// ===========================================================================
// `GetCoeffs` — one-block coefficient decoder (`detokenize.c:84-140`).
// ===========================================================================

/// `GetCoeffs` (`detokenize.c:84-140`) — decode all coefficients of one
/// 4x4 block. Scatters magnitudes through `kZigzag` into `out[0..16]`
/// and returns the zig-zag position of the last non-zero coefficient
/// plus one (0 if the block has no coefficients).
unsafe fn GetCoeffs(
    br: *mut BoolDecoder<'static>,
    prob: ProbaArray,
    ctx: i32,
    mut n: i32,
    out: *mut i16,
) -> i32 {
    // `p = prob[n][ctx]` — pointer to a row of NUM_PROBAS probabilities.
    let mut p: *const Prob = (*prob.offset(n as isize))[ctx as usize].as_ptr();
    if VP8GetBit(br, *p.offset(0) as i32) == 0 {
        /* first EOB is more a 'CBP' bit. */
        return 0;
    }
    loop {
        n += 1;
        if VP8GetBit(br, *p.offset(1) as i32) == 0 {
            p = (*prob.offset(kBands[n as usize] as isize))[0].as_ptr();
        } else {
            /* non zero coeff */
            let v: i32;
            let j: i32;
            if VP8GetBit(br, *p.offset(2) as i32) == 0 {
                p = (*prob.offset(kBands[n as usize] as isize))[1].as_ptr();
                v = 1;
            } else {
                if VP8GetBit(br, *p.offset(3) as i32) == 0 {
                    if VP8GetBit(br, *p.offset(4) as i32) == 0 {
                        v = 2;
                    } else {
                        v = 3 + VP8GetBit(br, *p.offset(5) as i32);
                    }
                } else {
                    if VP8GetBit(br, *p.offset(6) as i32) == 0 {
                        if VP8GetBit(br, *p.offset(7) as i32) == 0 {
                            v = 5 + VP8GetBit(br, 159);
                        } else {
                            let mut vv = 7 + 2 * VP8GetBit(br, 165);
                            vv += VP8GetBit(br, 145);
                            v = vv;
                        }
                    } else {
                        let mut tab: *const u8;
                        let bit1 = VP8GetBit(br, *p.offset(8) as i32);
                        let bit0 = VP8GetBit(br, *p.offset(9 + bit1 as isize) as i32);
                        let cat = (2 * bit1 + bit0) as usize;
                        let mut vv: i32 = 0;
                        tab = kCat3456.0[cat];
                        while *tab != 0 {
                            vv += vv + VP8GetBit(br, *tab as i32);
                            tab = tab.offset(1);
                        }
                        vv += 3 + (8 << cat);
                        v = vv;
                    }
                }
                p = (*prob.offset(kBands[n as usize] as isize))[2].as_ptr();
            }
            j = kZigzag[(n - 1) as usize] as i32;

            *out.offset(j as isize) = GetSigned(br, v) as i16;

            if n == 16 || VP8GetBit(br, *p.offset(0) as i32) == 0 {
                /* EOB */
                return n;
            }
        }
        if n == 16 {
            return 16;
        }
    }
}

// ===========================================================================
// `vp8_reset_mb_tokens_context` — zero the entropy context for a skip MB.
// ===========================================================================

/// `vp8_reset_mb_tokens_context` (`detokenize.c:18-29`).
///
/// Clears the first 8 entropy-context bytes (Y, U, V) of both the above
/// and left contexts. The 9th byte (Y2) is only cleared when the MB
/// uses the second-order transform (`!is_4x4`); B_PRED MBs intentionally
/// preserve whatever Y2 context the previous MB left there.
pub unsafe fn vp8_reset_mb_tokens_context(
    dx: *mut Vp8dComp<'static>,
    x: *mut Macroblockd,
    mb_col: i32,
) {
    let a_ctx: *mut EntropyContext = &mut (*dx).common.above_context.as_deref_mut().unwrap()
        [mb_col as usize] as *mut _ as *mut EntropyContext;
    let l_ctx: *mut EntropyContext = &mut (*dx).common.left_context as *mut _ as *mut EntropyContext;

    core::ptr::write_bytes(a_ctx, 0u8, core::mem::size_of::<EntropyContextPlanes>() - 1);
    core::ptr::write_bytes(l_ctx, 0u8, core::mem::size_of::<EntropyContextPlanes>() - 1);

    /* Clear entropy contexts for Y2 blocks */
    if !(*(*x).mode_info_context).mbmi.is_4x4 {
        *a_ctx.offset(8) = 0;
        *l_ctx.offset(8) = 0;
    }
}

// ===========================================================================
// `vp8_decode_mb_tokens` — the per-MB driver (`detokenize.c:142-210`).
// ===========================================================================

/// `vp8_decode_mb_tokens` (`detokenize.c:142-210`).
///
/// Orchestrates exactly 24 or 25 calls to `GetCoeffs` (Y2 if present,
/// then 16 Y, then 8 UV), threading entropy contexts and the
/// per-block `eobs` array. Returns `eobtotal` — the sum of every
/// block's eob, with the Y2 adjustment described in the doc.
pub unsafe fn vp8_decode_mb_tokens(
    dx: *mut Vp8dComp<'static>,
    x: *mut Macroblockd,
    mb_col: i32,
) -> i32 {
    let bc: *mut BoolDecoder<'static> = (*x).current_bc as *mut BoolDecoder<'static>;
    let fc = &(*dx).common.fc as *const FrameContext;
    let eobs: *mut i8 = (*x).eobs.as_mut_ptr();

    let mut nonzeros: i32;
    let mut eobtotal: i32 = 0;

    let mut coef_probs: ProbaArray;
    let mut a_ctx: *mut EntropyContext = &mut (*dx).common.above_context.as_deref_mut().unwrap()
        [mb_col as usize] as *mut _ as *mut EntropyContext;
    let mut l_ctx: *mut EntropyContext =
        &mut (*dx).common.left_context as *mut _ as *mut EntropyContext;
    let mut a: *mut EntropyContext;
    let mut l: *mut EntropyContext;
    let skip_dc: i32;

    let mut qcoeff_ptr: *mut i16 = (*x).qcoeff.as_mut_ptr();

    if !(*(*x).mode_info_context).mbmi.is_4x4 {
        a = a_ctx.offset(8);
        l = l_ctx.offset(8);

        coef_probs = (*fc).coef_probs[1].as_ptr() as ProbaArray;

        nonzeros = GetCoeffs(
            bc,
            coef_probs,
            (*a + *l) as i32,
            0,
            qcoeff_ptr.offset(24 * 16),
        );
        *a = (nonzeros > 0) as EntropyContext;
        *l = (nonzeros > 0) as EntropyContext;

        *eobs.offset(24) = nonzeros as i8;
        eobtotal += nonzeros - 16;

        coef_probs = (*fc).coef_probs[0].as_ptr() as ProbaArray;
        skip_dc = 1;
    } else {
        coef_probs = (*fc).coef_probs[3].as_ptr() as ProbaArray;
        skip_dc = 0;
    }

    for i in 0..16i32 {
        a = a_ctx.offset((i & 3) as isize);
        l = l_ctx.offset(((i & 0xc) >> 2) as isize);

        nonzeros = GetCoeffs(bc, coef_probs, (*a + *l) as i32, skip_dc, qcoeff_ptr);
        *a = (nonzeros > 0) as EntropyContext;
        *l = (nonzeros > 0) as EntropyContext;

        nonzeros += skip_dc;
        *eobs.offset(i as isize) = nonzeros as i8;
        eobtotal += nonzeros;
        qcoeff_ptr = qcoeff_ptr.offset(16);
    }

    coef_probs = (*fc).coef_probs[2].as_ptr() as ProbaArray;

    a_ctx = a_ctx.offset(4);
    l_ctx = l_ctx.offset(4);
    for i in 16..24i32 {
        a = a_ctx.offset((((i > 19) as i32) << 1) as isize + (i & 1) as isize);
        l = l_ctx.offset((((i > 19) as i32) << 1) as isize + ((i & 3) > 1) as isize);

        nonzeros = GetCoeffs(bc, coef_probs, (*a + *l) as i32, 0, qcoeff_ptr);
        *a = (nonzeros > 0) as EntropyContext;
        *l = (nonzeros > 0) as EntropyContext;

        *eobs.offset(i as isize) = nonzeros as i8;
        eobtotal += nonzeros;
        qcoeff_ptr = qcoeff_ptr.offset(16);
    }

    eobtotal
}
