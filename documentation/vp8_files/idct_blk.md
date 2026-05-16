# `vp8/common/idct_blk.c` — per-block IDCT dispatch over a macroblock

`idct_blk.c` is the thin dispatch layer that, after coefficients have been
dequantized and end-of-block indices recorded, walks the 4×4 transform
blocks of a macroblock and decides — block by block — *which* inverse
transform kernel to call and *whether to call one at all*. It contains
exactly two externally visible functions:

```c
void vp8_dequant_idct_add_y_block_c (short *q, short *dq,
                                     unsigned char *dst,   int stride,
                                     char *eobs);
void vp8_dequant_idct_add_uv_block_c(short *q, short *dq,
                                     unsigned char *dst_u,
                                     unsigned char *dst_v, int stride,
                                     char *eobs);
```

Both are reference C implementations of RTCD entry points declared in
`vp8/common/rtcd_defs.pl`:

```
add_proto qw/void vp8_dequant_idct_add_y_block/,
  "short *q, short *dq, unsigned char *dst, int stride, char *eobs";
specialize qw/vp8_dequant_idct_add_y_block neon dspr2 msa mmi lsx/, "$sse2_asm";
```

— so on a real CPU these `*_c` versions are replaced wholesale by NEON,
SSE2, etc. through `vp8_rtcd.h`. The C versions are what the generic
build links, what the spec and the unit tests check against, and what
this document describes.

The file's only includes are `vpx_config.h`, `vp8_rtcd.h` (for the
prototypes of the kernels it dispatches to: `vp8_dequant_idct_add_c`,
`vp8_dc_only_idct_add_c`), and `vpx_mem/vpx_mem.h` (for `memset`).

## Role in the decoder

VP8 uses a single transform size — 4×4 — for every residual block. A
macroblock therefore has 24 4×4 transform blocks (16 luma + 4 U + 4 V),
plus an optional 25th "Y2" block whose 16 coefficients are themselves the
DCs of the 16 luma blocks reorganised into another 4×4 grid and
transformed once more with a Walsh–Hadamard. (See §4.2 of the technical
overview.) After detokenisation, each of the 25 blocks has

- 16 signed-short quantized coefficients packed into the per-MB
  `qcoeff[400]` array, and
- one `char` *end-of-block index* (`eob`) packed into `eobs[25]` —
  the position one past the last non-zero coefficient seen in the
  zigzag scan, in the range `[0, 16]`.

The decoder reaches the IDCT stage having (a) inverted the Walsh on
block 24 if present (`vp8_inverse_transform_mby` in `invtrans.h`
scatters the resulting 16 DCs back into `qcoeff[i*16]` for `i=0..15`),
and (b) built the predictor into the destination YV12 buffer using
intra- or inter-prediction. What remains is to dequantize, inverse-DCT,
and *add* each block's residual on top of the predictor pixels already
sitting in `dst`. The two functions in this file are exactly that loop.

Crucially they branch per block on `eob`:

```
eob >  1 :  call full 4×4 IDCT  (vp8_dequant_idct_add)
eob == 1 :  call the 1-coefficient DC-only shortcut (vp8_dc_only_idct_add)
eob == 0 :  do nothing — predictor stays as-is — block is all zeros
```

That three-way decision is the whole point of the file: a full IDCT
butterfly is ≈ 30 multiplies and 60 adds; the DC-only path is one
multiply and a constant; the skip path is zero work. On natural video
the great majority of high-frequency 4×4 blocks have `eob ≤ 1`, so this
dispatch — invoked 24 times per macroblock, thousands of times per
frame — is a significant fraction of total decode time. The dispatcher
also has the side effect of *clearing* the input coefficient buffer
ahead of the next frame so the decoder need not memset 400 shorts
explicitly.

Note that the Walsh inversion for block 24 is *not* done here; it
happens earlier, in `vp8_inverse_transform_mby` (invtrans.h), which is
the caller of `vp8_dequant_idct_add_y_block`. By the time this file
runs, block 24's contribution has already been folded into the DC
positions of blocks 0..15 in `qcoeff`. So this file only ever sees
"plain" 4×4 IDCTs and DC-only shortcuts; it never invokes the WHT
kernels directly.

## End-of-block (eob) semantics

`eob` is the number of coefficients the entropy decoder placed before
emitting `DCT_EOB_TOKEN` (detokenize.c). Concretely:

- `eob == 0` means the very first token was `DCT_EOB_TOKEN`. The
  block is all zeros after dequantization, and the IDCT output is the
  all-zero matrix. Adding zero to the predictor is a no-op.
- `eob == 1` means exactly one non-zero coefficient was decoded, and
  because the scan is zigzag and starts at position 0, *that
  coefficient is the DC*. The 4×4 IDCT of a DC-only input is itself
  another constant matrix (every output pixel equals `(dc + 4) >> 3`
  in VP8's exact fixed-point form), so a full butterfly is wasteful.
- `eob > 1` means at least one AC coefficient exists; the general
  IDCT must run.

There is a subtlety hidden behind the `eob == 1` test: the entropy
decoder's "EOB" position is in *zigzag* coordinates, but the DC
coefficient happens to be position 0 of the zigzag *and* position 0 in
natural row-major order. So `qcoeff[0]` really is the DC regardless of
representation, which is why the DC-only path reads `q[0]` directly.

When Y2 is in use (the usual non-`B_PRED`, non-`SPLITMV` case), the
entropy decoder decodes the per-Y blocks starting at *band 1* — i.e.,
their DC slot is left as whatever the Walsh inversion wrote into it,
and `eob` for those blocks reflects only AC. `invtrans.h` performs an
explicit `eob_adjust` step before invoking this file:

```c
static void eob_adjust(char *eobs, short *diff) {
  /* eob adjust.... the idct can only skip if both the dc and eob are zero */
  int js;
  for (js = 0; js < 16; ++js) {
    if ((eobs[js] == 0) && (diff[0] != 0)) eobs[js]++;
    diff += 16;
  }
}
```

This bumps `eobs[i]` from 0 to 1 for any Y block whose entropy-coded AC
was empty but which received a non-zero DC from the Walsh pass — so
that the dispatcher in this file takes the DC-only branch instead of
the skip branch. The invariant the dispatcher relies on, therefore, is:
**`eob == 0` ⇒ all 16 coefficients (DC included) are zero**.

## The luma block: `vp8_dequant_idct_add_y_block_c`

The luma function processes the 16 4×4 Y blocks of a macroblock in
raster order, four blocks per row, four rows per MB:

```c
void vp8_dequant_idct_add_y_block_c(short *q, short *dq, unsigned char *dst,
                                    int stride, char *eobs) {
  int i, j;
  for (i = 0; i < 4; ++i) {
    for (j = 0; j < 4; ++j) {
      if (*eobs++ > 1) {
        vp8_dequant_idct_add_c(q, dq, dst, stride);
      } else {
        vp8_dc_only_idct_add_c(q[0] * dq[0], dst, stride, dst, stride);
        memset(q, 0, 2 * sizeof(q[0]));
      }
      q   += 16;
      dst +=  4;
    }
    dst += 4 * stride - 16;
  }
}
```

**What it does.** It walks `eobs[0..15]`, the per-block EOB indices for
the 16 Y blocks (the 17th–25th entries of the parent `eobs[25]` array
belong to U, V and Y2, and are stepped over by the caller's pointer
arithmetic). For each block it dispatches to a full IDCT or to the
DC-only shortcut.

**Why two arguments collapse into one comparison.** The original three
cases (`> 1`, `== 1`, `== 0`) are folded into a binary branch on
`*eobs++ > 1`. The `else` arm handles both `eob == 1` (real DC-only)
*and* `eob == 0` (all zeros). For the latter, `q[0]` is zero, so
`vp8_dc_only_idct_add_c` adds a zero constant to the predictor — that
is, it copies the predictor through, which is the correct no-op. This
trades one comparison and one (cheap) DC-add for the simpler control
flow.

**Why `memset(q, 0, 2 * sizeof(q[0]))`.** Whenever the DC-only branch
is taken, the function explicitly zeros the *first two* shorts of the
current block's coefficient slot. This is the invariant that lets the
caller skip an explicit `memset(qcoeff, 0, ...)` between frames: any
block that was hit by the DC-only branch has its `q[0]` (the DC the
shortcut consumed) and `q[1]` (which can carry a stray value when the
2nd-order Walsh inversion ran) reset to zero. The full-IDCT path
(`vp8_dequant_idct_add_c`) is itself responsible for clearing its own
16 coefficients after consuming them. Together these two clears
guarantee that, on entry to the *next* macroblock, the entire
`qcoeff[]` region this MB touched is back to zero — a precondition for
the entropy decoder to write only the non-zero coefficients into
specific positions while leaving the others at zero. Only two shorts
are cleared in the DC-only path (not all 16) because only those two are
the ones that could have been non-zero on entry: the entropy decoder
emitted `EOB` immediately (so the AC positions are zero from the prior
frame's clear) and the DC came either from a single token or from the
Walsh scatter (which only writes position 0); position 1 is included
defensively because the Walsh pass writes consecutive memory and a
sloppy AC of 0 is cheaper to overwrite than to test.

**Pointer arithmetic and the raster walk.**

- `q += 16` advances the coefficient pointer by one block (16 shorts).
- `dst += 4` advances the destination pointer by one 4×4 block to the
  right.
- `dst += 4 * stride - 16` at the end of each row jumps down four rows
  in the YV12 plane and back to the leftmost block column. The `- 16`
  exactly undoes the four `+= 4` increments that happened during the
  row.

So `stride` is the destination plane's row pitch in bytes (`y_stride`),
and `dst` enters the function pointing at the top-left pixel of the
macroblock's luma area.

**Invariants on input.**

- `q` points at a 256-short region (16 blocks × 16 coeffs) laid out
  contiguously in block-raster order; this is `xd->qcoeff` advanced to
  the Y portion. After the function returns, every short this MB
  touched is zero again (either by the DC-only `memset` above or by
  the full-IDCT kernel's internal clear).
- `dq` is a 16-short dequantizer vector. *The same `dq` is reused for
  all 16 blocks*; the per-position scaling happens inside
  `vp8_dequant_idct_add_c` (`out[k] = q[k] * dq[k]`). For blocks 0..15
  the caller passes either `xd->dequant_y1` (B_PRED / SPLITMV path,
  full AC+DC dequant per block) or `xd->dequant_y1_dc` (Y2 path, AC
  positions and the DC position separately scaled — the DC entry of
  this table is the Y2 DC quantiser, applied because the Walsh inverse
  reconstructed *un*scaled DCs). The decision lives in
  `vp8_inverse_transform_mby` (invtrans.h:48–51).
- `eobs` is a 16-char array of EOB indices, *post* `eob_adjust`.

**How it is used.** Called from `vp8_inverse_transform_mby`
(invtrans.h) for every macroblock that uses whole-MB luma prediction
(`mbmi.mode != SPLITMV`); the prior call is either the Walsh inversion
(if Y2 is in use) followed by `eob_adjust`, or nothing (Y2 not used).
There is one site per frame loop and the function runs exactly
`mb_rows * mb_cols` times.

## The chroma block: `vp8_dequant_idct_add_uv_block_c`

The chroma function processes 8 4×4 blocks — four U then four V — in a
2×2 raster within each plane:

```c
void vp8_dequant_idct_add_uv_block_c(short *q, short *dq, unsigned char *dst_u,
                                     unsigned char *dst_v, int stride,
                                     char *eobs) {
  int i, j;
  for (i = 0; i < 2; ++i) {                              /* U plane */
    for (j = 0; j < 2; ++j) {
      if (*eobs++ > 1) {
        vp8_dequant_idct_add_c(q, dq, dst_u, stride);
      } else {
        vp8_dc_only_idct_add_c(q[0] * dq[0], dst_u, stride, dst_u, stride);
        memset(q, 0, 2 * sizeof(q[0]));
      }
      q     += 16;
      dst_u +=  4;
    }
    dst_u += 4 * stride - 8;
  }

  for (i = 0; i < 2; ++i) {                              /* V plane */
    /* …mirror of the above, with dst_v… */
  }
}
```

**What it does.** Two consecutive 2×2 raster walks — first over U
blocks 16..19 of the macroblock, then over V blocks 20..23 — issuing
the same `> 1 / else` dispatch per block. Both halves share the same
`q` pointer (advanced contiguously through `qcoeff[16*16 .. 16*24]`)
and the same `dq` (the chroma dequantizer `xd->dequant_uv`, since U and
V share quantizers in VP8).

**Why it takes two `dst_*` pointers and one `stride`.** U and V planes
in `YV12_BUFFER_CONFIG` have identical strides (`uv_stride`) but live in
separate base buffers; passing both pointers and one stride is the
exact match for that storage layout. The end-of-row adjustment uses
`- 8` (not `- 16`) because chroma planes are half resolution: an 8×8
chroma MB region is two 4×4 blocks wide, so two `+= 4` increments must
be undone.

**Why the U and V halves are not folded into one loop.** The chroma
EOBs occupy `eobs[16..23]` of the parent `eobs[25]` array, contiguously
U-then-V; the `eobs++` postfix increment relies on that contiguity. The
`q` pointer is likewise contiguous. So conceptually a single 8-block
loop would suffice — except for `dst_u` vs `dst_v`. Splitting the loop
hard-codes the U/V plane switch at the loop boundary instead of
branching on `i < 4` inside; this matches what hand-written SIMD
specializations want anyway, since each half maps onto a 2×2 register
tile.

**Invariants on input.** Same as the luma case, with the substitutions
4×4 → 2×2, 16 → 8, and one `dq` vector for both chroma planes. After
the function returns, every chroma coefficient short this MB touched is
zero again.

**How it is used.** Called immediately after
`vp8_dequant_idct_add_y_block` in the per-MB decode loop
(`vp8_decode_macroblock` in decodeframe.c), with `q` pointing at
`xd->qcoeff + 16*16`, `dq` at `xd->dequant_uv`, `dst_u` and `dst_v` at
the MB's chroma positions in the current frame's YV12 buffer, `stride`
= `dst.uv_stride`, and `eobs` = `xd->eobs + 16`. Runs once per
macroblock, also `mb_rows * mb_cols` times per frame.

## Why this file is so small

Everything that *could* be parameterised in the dispatch — the kernel
chosen, the per-coefficient dequant scale, the kernel's exact integer
arithmetic — is delegated:

- The full IDCT is `vp8_dequant_idct_add` (a separate RTCD'd entry,
  reference C in `dequantize.c`, which itself calls
  `vp8_short_idct4x4llm` in `idctllm.c`).
- The DC-only fast path is `vp8_dc_only_idct_add` (reference C in
  `idctllm.c`).
- The Walsh inversion that feeds the Y path is
  `vp8_short_inv_walsh4x4` / `vp8_short_inv_walsh4x4_1` (reference C
  also in `idctllm.c`), invoked from `invtrans.h` before this file
  runs.

What `idct_blk.c` contributes that none of the kernels can is the
**loop shape** — the exact raster walk over the 16 Y or 8 UV blocks of
one macroblock, the pointer arithmetic that follows the YV12 layout,
the per-block `eob` test, and the side-effect `memset` that keeps
`qcoeff[]` zero between frames. Because that loop shape is the unit on
which SIMD specializations are written, the file's surface area is
exactly two functions wide — and on a CPU with NEON/SSE2/MSA/MMI/LSX
support, neither of these two C implementations is ever called. They
remain in the tree as the spec-equivalent reference, the fallback for
unknown architectures, and the documentation of intent.
