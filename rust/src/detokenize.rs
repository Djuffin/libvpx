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

use crate::tables::{COEF_BANDS, Prob};
use crate::types::{
    BD_VALUE_BITS, BdValue, BoolDecoder, ECTX_UV, ECTX_Y2, EntropyContext, EntropyContextPlanes,
    FrameContext, Macroblockd, ModeInfo,
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

/// `kCat3456[]` (`detokenize.c:45`) — view of the four category tables,
/// indexed by `cat = 2*bit1 + bit0`. Each table ends in a `0` sentinel.
static kCat3456: [&[u8]; 4] = [&kCat3, &kCat4, &kCat5, &kCat6];

/// `kZigzag[16]` (`detokenize.c:46-47`) — inverse zig-zag table.
static kZigzag: [u8; 16] = [0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15];

const NUM_PROBAS: usize = 11;
const NUM_CTX: usize = 3;

/// `ProbaArray` (`detokenize.c:54`) — one block type's coefficient
/// probabilities, indexed `prob[band][ctx][node]`. Borrowed directly from
/// `FRAME_CONTEXT.coef_probs[block_type]` (a `[[[Prob; 11]; 3]; 8]`).
type ProbaArray<'a> = &'a [[[Prob; NUM_PROBAS]; NUM_CTX]; COEF_BANDS];

// ===========================================================================
// `VP8GetBit` — thin macro rename (`detokenize.c:49`).
// ===========================================================================

/// `VP8GetBit` (`detokenize.c:49`) — alias macro for `vp8dx_decode_bool`.
#[inline(always)]
fn VP8GetBit(br: &mut BoolDecoder<'static>, probability: i32) -> i32 {
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
fn GetSigned(br: &mut BoolDecoder<'static>, value_to_sign: i32) -> i32 {
    let split: i32 = (br.range as i32 + 1) >> 1;
    let bigsplit: BdValue = (split as BdValue) << (BD_VALUE_BITS - 8);
    let v: i32;

    if br.count < 0 {
        vp8dx_bool_decoder_fill(br);
    }

    if br.value < bigsplit {
        br.range = split as u32;
        v = value_to_sign;
    } else {
        br.range = br.range - split as u32;
        br.value = br.value.wrapping_sub(bigsplit);
        v = -value_to_sign;
    }
    br.range = br.range.wrapping_add(br.range);
    br.value = br.value.wrapping_add(br.value);
    br.count -= 1;

    v
}

// ===========================================================================
// `GetCoeffs` — one-block coefficient decoder (`detokenize.c:84-140`).
// ===========================================================================

/// `GetCoeffs` (`detokenize.c:84-140`) — decode all coefficients of one
/// 4x4 block. Scatters magnitudes through `kZigzag` into `out[0..16]`
/// and returns the zig-zag position of the last non-zero coefficient
/// plus one (0 if the block has no coefficients).
fn GetCoeffs(
    br: &mut BoolDecoder<'static>,
    prob: ProbaArray,
    ctx: i32,
    mut n: i32,
    out: &mut [i16],
) -> i32 {
    // `p = prob[band][ctx]` — a row of NUM_PROBAS token-tree probabilities.
    let mut p: &[Prob; NUM_PROBAS] = &prob[n as usize][ctx as usize];
    if VP8GetBit(br, p[0] as i32) == 0 {
        /* first EOB is more a 'CBP' bit. */
        return 0;
    }
    loop {
        n += 1;
        if VP8GetBit(br, p[1] as i32) == 0 {
            p = &prob[kBands[n as usize] as usize][0];
        } else {
            /* non zero coeff */
            let v: i32;
            let j: usize;
            if VP8GetBit(br, p[2] as i32) == 0 {
                p = &prob[kBands[n as usize] as usize][1];
                v = 1;
            } else {
                if VP8GetBit(br, p[3] as i32) == 0 {
                    if VP8GetBit(br, p[4] as i32) == 0 {
                        v = 2;
                    } else {
                        v = 3 + VP8GetBit(br, p[5] as i32);
                    }
                } else {
                    if VP8GetBit(br, p[6] as i32) == 0 {
                        if VP8GetBit(br, p[7] as i32) == 0 {
                            v = 5 + VP8GetBit(br, 159);
                        } else {
                            let mut vv = 7 + 2 * VP8GetBit(br, 165);
                            vv += VP8GetBit(br, 145);
                            v = vv;
                        }
                    } else {
                        let bit1 = VP8GetBit(br, p[8] as i32);
                        let bit0 = VP8GetBit(br, p[9 + bit1 as usize] as i32);
                        let cat = (2 * bit1 + bit0) as usize;
                        let mut vv: i32 = 0;
                        for &t in kCat3456[cat] {
                            if t == 0 {
                                break;
                            }
                            vv += vv + VP8GetBit(br, t as i32);
                        }
                        vv += 3 + (8 << cat);
                        v = vv;
                    }
                }
                p = &prob[kBands[n as usize] as usize][2];
            }
            j = kZigzag[(n - 1) as usize] as usize;

            out[j] = GetSigned(br, v) as i16;

            if n == 16 || VP8GetBit(br, p[0] as i32) == 0 {
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
pub fn vp8_reset_mb_tokens_context(
    above_slot: &mut EntropyContextPlanes,
    left_context: &mut EntropyContextPlanes,
    mi: &ModeInfo,
) {
    // Always clear Y/U/V (bytes 0..ECTX_Y2); also clear Y2 (byte ECTX_Y2)
    // unless this is a B_PRED MB, which preserves the previous Y2 context.
    let n = if mi.mbmi.is_4x4 { ECTX_Y2 } else { ECTX_Y2 + 1 };
    above_slot.ctx[..n].fill(0);
    left_context.ctx[..n].fill(0);
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
pub fn vp8_decode_mb_tokens(
    above_slot: &mut EntropyContextPlanes,
    left_context: &mut EntropyContextPlanes,
    fc: &FrameContext,
    mb: &mut Macroblockd,
    mi: &ModeInfo,
    bc: &mut BoolDecoder<'static>,
) -> i32 {
    let mut nonzeros: i32;
    let mut eobtotal: i32 = 0;

    let mut coef_probs: ProbaArray;
    // Entropy-context cursors are bounds-checked indices into the flat
    // `[i8; 9]` planes (Y at ECTX_Y1, U/V at ECTX_UV, Y2 at ECTX_Y2). The
    // C code walked these as raw `ENTROPY_CONTEXT *` with `.offset()`; the
    // index arithmetic below is the exact equivalent.
    let a = &mut above_slot.ctx;
    let l = &mut left_context.ctx;
    let skip_dc: i32;

    // `qcoeff`/`eobs` are disjoint fields of `mb`; the per-block coefficient
    // slice is `qcoeff[block * 16 .. block * 16 + 16]`.
    let qcoeff = &mut mb.qcoeff;
    let eobs = &mut mb.eobs;

    if !mi.mbmi.is_4x4 {
        coef_probs = &fc.coef_probs[1];

        nonzeros = GetCoeffs(
            bc,
            coef_probs,
            (a[ECTX_Y2] + l[ECTX_Y2]) as i32,
            0,
            &mut qcoeff[24 * 16..24 * 16 + 16],
        );
        a[ECTX_Y2] = (nonzeros > 0) as EntropyContext;
        l[ECTX_Y2] = (nonzeros > 0) as EntropyContext;

        eobs[24] = nonzeros as i8;
        eobtotal += nonzeros - 16;

        coef_probs = &fc.coef_probs[0];
        skip_dc = 1;
    } else {
        coef_probs = &fc.coef_probs[3];
        skip_dc = 0;
    }

    for i in 0..16usize {
        let ai = i & 3;
        let li = (i & 0xc) >> 2;

        nonzeros = GetCoeffs(
            bc,
            coef_probs,
            (a[ai] + l[li]) as i32,
            skip_dc,
            &mut qcoeff[i * 16..i * 16 + 16],
        );
        a[ai] = (nonzeros > 0) as EntropyContext;
        l[li] = (nonzeros > 0) as EntropyContext;

        nonzeros += skip_dc;
        eobs[i] = nonzeros as i8;
        eobtotal += nonzeros;
    }

    coef_probs = &fc.coef_probs[2];

    // UV blocks address the U/V region at base offset ECTX_UV.
    for i in 16..24usize {
        let ai = ECTX_UV + (((i > 19) as usize) << 1) + (i & 1);
        let li = ECTX_UV + (((i > 19) as usize) << 1) + ((i & 3) > 1) as usize;

        nonzeros = GetCoeffs(
            bc,
            coef_probs,
            (a[ai] + l[li]) as i32,
            0,
            &mut qcoeff[i * 16..i * 16 + 16],
        );
        a[ai] = (nonzeros > 0) as EntropyContext;
        l[li] = (nonzeros > 0) as EntropyContext;

        eobs[i] = nonzeros as i8;
        eobtotal += nonzeros;
    }

    eobtotal
}
