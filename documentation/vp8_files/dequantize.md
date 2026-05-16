# `vp8/common/dequantize.c` — inverse quantization primitives

This file holds two of the smallest, most-called primitives in the
VP8 decoder: the reference C implementations of *inverse quantization*
for a single 4x4 block. The whole compilation unit is 38 lines and
contains exactly two functions, but its placement in the pipeline and
the conventions it obeys deserve a careful look — every faster SIMD
variant (NEON, MSA, MMI, the discontinued MMX path) is judged
bit-exact against the code in this file via `vp8_rtcd.h`'s
function-pointer table.

## Role in the decoder

VP8 transmits each 4x4 residual block as 16 quantized DCT coefficients
in zigzag order (RFC 6386 §13). The decoder reconstructs samples in
three discrete steps per block:

  1. **Detokenize.** `vp8_decode_mb_tokens` (vp8/decoder/detokenize.c)
     walks the per-coefficient token tree and scatters the recovered
     magnitudes into `BLOCKD::qcoeff[16]` (in *natural* row-major
     order — the zigzag has already been undone via
     `kZigzag[]`). It also fills `eobs[i]` with the count of
     non-zero coefficients in block `i`, used downstream as a
     fast-path discriminator.

  2. **Inverse quantization.** Multiply each `qcoeff[k]` by the
     matching dequantizer scale `dq[k]`, producing the integer
     transform-domain coefficients `dqcoeff[k]`. The scales come from
     RFC 6386 §14.1's `dc_qlookup[]` / `ac_qlookup[]` tables (in
     libvpx these live in `vp8/common/quant_common.c`, accessed via
     the `vp8_dc_quant` / `vp8_ac_quant` family declared in
     `quant_common.h:22-27`). Per-frame the decoder fans them out
     into four 16-short arrays held on `MACROBLOCKD`:

         dequant_y1     /* Y AC scales, with normal Y DC scale at [0] */
         dequant_y1_dc  /* same as dequant_y1 but [0] is the second-order DC scale */
         dequant_y2     /* the Y2 (DC-of-DCs) block's DC and AC scales */
         dequant_uv     /* chroma scales */

     (declarations: blockd.h:215-218).

  3. **Inverse transform + accumulate.** Run `vp8_short_idct4x4llm`
     to convert the dequantized coefficients into a 4x4 residual,
     then add that residual to the already-formed predictor in `dst`
     and clip to `[0,255]`.

`dequantize.c` provides the kernels for steps (2) and (2+3). The
splitting matters: the Y2 (second-order DC) block needs step (2)
*standalone* — its dequantized coefficients are fed to a
Walsh–Hadamard (`vp8_short_inv_walsh4x4`) rather than the DCT, and
its outputs are then scattered back into the DC slots of the 16 Y
AC blocks before *those* go through step (2+3). For everything else
— the 16 Y AC blocks and the 8 chroma blocks — the dequant and the
IDCT happen together, fused into one routine for cache friendliness.

Both functions are reference C only. They are reachable from the
real decode path through the RTCD shim — see `vp8_rtcd.h:64-74` in
the configured build tree:

    void vp8_dequant_idct_add_c(short *input, short *dq, unsigned char *dest, int stride);
    #define vp8_dequant_idct_add vp8_dequant_idct_add_c
    void vp8_dequantize_b_c(struct blockd*, short *DQC);
    #define vp8_dequantize_b vp8_dequantize_b_c

(those `#define`s flip to function pointers when SIMD is built in;
the `rtcd_defs.pl` entries are at vp8/common/rtcd_defs.pl:43-47.)

## The two definitions

### `vp8_dequantize_b_c` — standalone dequantize for the Y2 block

**What.** Multiplies the 16 quantized coefficients of a single 4x4
block by the 16-element dequantizer table, writing the result to a
*separate* output buffer.

    void vp8_dequantize_b_c(BLOCKD *d, short *DQC) {           /* dequantize.c:16 */
      int i;
      short *DQ = d->dqcoeff;
      short *Q  = d->qcoeff;
      for (i = 0; i < 16; ++i) {
        DQ[i] = Q[i] * DQC[i];
      }
    }

**Why standalone.** Of the 25 blocks of a VP8 macroblock, exactly
one — block 24, the Y2 second-order DC block — does not get an IDCT
applied to its dequantized coefficients. Instead, the 16 dequantized
DCs go through `vp8_short_inv_walsh4x4`, and the resulting 16 values
are then sprinkled back into position 0 of each of the 16 Y AC
blocks' `qcoeff[]` arrays (RFC 6386 §14.3, also `decodeframe.c:209`).
Because of that, dequantization and transform must be separated for
Y2 — there is no fused `dequant_walsh_add_c`. Compare the calls in
`decodeframe.c:208-217`:

    if (xd->eobs[24] > 1) {
      vp8_dequantize_b(b, xd->dequant_y2);
      vp8_short_inv_walsh4x4(&b->dqcoeff[0], xd->qcoeff);   /* scatter into Y DCs */
      memset(b->qcoeff, 0, 16 * sizeof(b->qcoeff[0]));
    } else {
      b->dqcoeff[0] = (short)(b->qcoeff[0] * xd->dequant_y2[0]);
      vp8_short_inv_walsh4x4_1(&b->dqcoeff[0], xd->qcoeff);
      memset(b->qcoeff, 0, 2 * sizeof(b->qcoeff[0]));
    }

The non-fused form also explains the function's signature: it takes
a `BLOCKD *` rather than two raw pointers, because the source and
destination are both reachable from the descriptor (`qcoeff` and
`dqcoeff` — blockd.h:194-195) and Y2 has its own per-block buffers
(`MACROBLOCKD::block[24]` indexes into the 25th 16-coefficient
slice of the 400-short shared `qcoeff[400]` and `dqcoeff[400]`
arenas — blockd.h:211-212, 221).

**Invariants.**
  * `d->qcoeff` and `d->dqcoeff` are 16-aligned 16-element regions
    (alignment comes from `DECLARE_ALIGNED(16, …)` in blockd.h:211-212).
  * `DQC` is exactly the 16-entry `dequant_y2[16]` of the parent
    `MACROBLOCKD` (blockd.h:217). Element 0 is the Y2 DC scale
    (`vp8_dc2quant`), elements 1..15 are the Y2 AC scale
    (`vp8_ac2quant`) replicated.
  * The product `Q[i] * DQC[i]` fits in 16 bits by spec —
    coefficient magnitudes after dequant cannot exceed the working
    range of the Walsh, so `short` is safe. (cat6 magnitudes ±2114
    × the maximum Y2 AC scale also fit in `int`, which is the
    promoted type before the truncating store; the truncation is
    deterministic two's-complement.)
  * Unlike `vp8_dequant_idct_add_c` below, `qcoeff` is **not**
    cleared in this function. The caller is responsible (see the
    `memset(b->qcoeff, 0, 16 * sizeof(...))` immediately after the
    call at `decodeframe.c:212`).

**How used.** Called exactly once per macroblock that has an `MB`-
level Y2 block (i.e. every macroblock whose prediction mode is not
`B_PRED` or `SPLITMV`). The call sites are `decodeframe.c:209` and
`threading.c:226`.

### `vp8_dequant_idct_add_c` — fused dequant + IDCT + accumulate

**What.** For one Y AC block or one chroma 4x4 block: scale
coefficients by `dq[]` *in place* (overwriting the original
quantized values), run the 4x4 IDCT, accumulate the resulting
residual onto `dest` (which already holds the intra or inter
predictor), and finally zero the input buffer so the next pass over
this MB sees a clean slate.

    void vp8_dequant_idct_add_c(short *input, short *dq,        /* dequantize.c:26 */
                                unsigned char *dest, int stride) {
      int i;
      for (i = 0; i < 16; ++i) {
        input[i] = dq[i] * input[i];
      }
      vp8_short_idct4x4llm_c(input, dest, stride, dest, stride);
      memset(input, 0, 32);
    }

**Why fused.** Once the second-order Walsh has been run (or for blocks
that never had one), the rest of the macroblock dequant/IDCT pass
is mechanical: for each 4x4 block, multiply by the right `dq[]`,
inverse-transform, add to the predictor. Fusing the multiply into
the same function that calls the IDCT avoids a round-trip to memory
for the 16 transform coefficients between two functions — important
because this is hot code (24 calls per non-skipped macroblock, before
SIMD speedups). It is also the natural unit for the SIMD variants:
on NEON, a load-multiply-IDCT-store can stay entirely in vector
registers.

**Invariants.**
  * `input` points to 16 shorts that already hold the *quantized*
    coefficients in natural (un-zigzagged) row-major order — that
    is, the output of `GetCoeffs` (detokenize.c:84).
  * `dq` points to a 16-short dequantizer table. For Y blocks
    *with* an active Y2 the caller passes `dequant_y1_dc`, in which
    case position 0 has been overridden by `xd->dequant_y2[0]` so
    that the IDCT sees a consistent DC scale (`decodeframe.c:222`,
    comment "override the dc dequant constant in order to preserve
    the dc components"). For Y blocks *without* an active Y2, the
    caller passes `dequant_y1` (which has the ordinary Y DC scale
    at position 0). For chroma it is `dequant_uv`.
  * `dest` already contains the predictor (intra prediction filled
    it for intra MBs, motion compensation for inter MBs); the IDCT
    will add residual on top and saturate.
  * `stride` is the Y/U/V plane stride of `dst` in the current
    `YV12_BUFFER_CONFIG`, *not* the 4x4 block's own stride.
  * Two output buffers are passed to `vp8_short_idct4x4llm_c` as
    the same pointer — VP8's IDCT reads the predictor from one
    pointer and writes back to another, but here read-modify-write
    happens in place.
  * Caller pre-condition for entry: `eobs[i] > 1`. The
    `eobs[i] == 1` (DC-only) and `eobs[i] == 0` (all-zero) cases
    are handled by `vp8_dc_only_idct_add` and a no-op respectively
    in the dispatcher above this function — see `idct_blk.c:21-25`.
  * Post-condition: `input[0..15]` is all zeros after the call. The
    `memset(input, 0, 32)` is two bytes per short × 16 shorts. This
    matters because the same 400-short `qcoeff` arena is reused for
    every macroblock in the frame, and `vp8_decode_mb_tokens` only
    writes the *non-zero* coefficients it actually decoded; the
    remaining positions are expected to be zero from the previous
    pass. Clearing here keeps that invariant.

**How used.** Two call sites in the C reference path, both inside
`vp8/common/idct_blk.c`:

  * `vp8_dequant_idct_add_y_block_c` (idct_blk.c:15) — iterates the
    16 Y AC blocks of one MB; calls `vp8_dequant_idct_add_c` for each
    block whose `eobs > 1`, with the appropriate `DQC` per the rules
    above.
  * `vp8_dequant_idct_add_uv_block_c` (idct_blk.c:36) — same shape
    but 4 + 4 chroma blocks, with `dequant_uv`.

There is also a direct call from the intra-4x4 path
(`decodeframe.c:180`, `threading.c:197`) for the `B_PRED` mode,
where dequant + IDCT + accumulate is applied per-block immediately
after the per-block intra prediction, before the next sub-block's
prediction is computed.

## Summary

  * The compilation unit is intentionally tiny: it defines the two
    fundamental dequant kernels and nothing else.
  * `vp8_dequantize_b_c` is the *non-fused* form, used only for the
    Y2 second-order block, whose downstream is the Walsh and a
    DC-scatter rather than an IDCT.
  * `vp8_dequant_idct_add_c` is the *fused* form, used for every
    other 4x4 block. It owns three responsibilities (dequant, IDCT,
    accumulate) and one cleanup (zero the input) because all of
    them are part of the per-block pipeline and benefit from being
    SIMD-fusible into a single routine.
  * The data shape, alignment, ordering (natural, not zigzag) and
    `eobs` discrimination are all enforced by the callers; this
    file's contract is purely "multiply and transform what I'm
    given".
  * Both functions are dispatched through RTCD; faster
    architecture-specific implementations must be bit-exact with
    these references.
