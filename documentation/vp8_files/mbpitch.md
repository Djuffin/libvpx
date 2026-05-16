# `vp8/common/mbpitch.c` — per-macroblock block-descriptor wiring

## Role in the decoder

A VP8 macroblock is, internally, not one object but a *flotilla* of
twenty-five 4x4 blocks travelling together: sixteen luma blocks, four U
chroma, four V chroma, and one phantom "Y2" block that holds the DC
coefficients of the sixteen luma blocks after a second-order Walsh
transform. The per-macroblock working context, `MACROBLOCKD` (see
`blockd.h`, struct definition lines 209–302), holds the *bulk* storage
for every per-MB intermediate quantity in one place — a flat predictor
buffer, two flat coefficient buffers, and an end-of-block counter
array — and then exposes a parallel array `BLOCKD block[25]` whose entries
are little descriptors carrying *pointers into* those flat buffers.

`mbpitch.c` is the one-time wiring file for that array. It contains two
functions, both invoked once per `MACROBLOCKD` instance (or once per
destination buffer), that pre-compute the per-subblock pointer offsets
and destination-frame offsets and store them in `block[0..24]`. After
this wiring is performed, every subsequent pass — token decode,
dequantize, IDCT, Walsh second-order pass, intra-prediction, reconstruction —
addresses the contents of the macroblock through `xd->block[i].predictor`,
`xd->block[i].qcoeff`, `xd->block[i].dqcoeff`, `xd->block[i].eob`, and
`xd->block[i].offset`, never through the flat backing buffers directly.

The file is small (about forty lines of code, no headers besides
`blockd.h`) and entirely consists of address arithmetic. But the
arithmetic encodes the canonical *block ordering* of VP8 — luma in raster
order, then U, then V, then the Y2 second-order block as index 24 — and
the *strides* peculiar to each plane: 16 pixels for luma (one whole
predictor row of an MB), 8 pixels for chroma, and an implicit "the Y2
block isn't a region of the predictor at all" for index 24. Anything
that later reads `block[i].predictor` is silently relying on the layout
this file imposes.

## The backing arrays in `MACROBLOCKD`

Before we look at the wiring functions themselves, it is worth recalling
the shape of the four flat buffers they index into. From `blockd.h`
(lines 210–221):

```c
DECLARE_ALIGNED(16, unsigned char, predictor[384]);
DECLARE_ALIGNED(16, short, qcoeff[400]);
DECLARE_ALIGNED(16, short, dqcoeff[400]);
DECLARE_ALIGNED(16, char, eobs[25]);
...
BLOCKD block[25];
```

- `predictor[384]` is exactly one macroblock's worth of intra/inter
  predicted samples: 16x16 = 256 bytes for Y, then 8x8 = 64 bytes for U,
  then 8x8 = 64 bytes for V, total 384. The layout is **plane-major**
  and within each plane the predictor for an MB is stored as a *flat
  contiguous block* whose stride equals the MB width in that plane
  (16 for luma, 8 for chroma).
- `qcoeff[400]` and `dqcoeff[400]` hold 25 * 16 = 400 quantized
  coefficients each. The block ordering is the canonical VP8 ordering:
  blocks 0..15 are the sixteen luma 4x4s in raster order, 16..19 are
  the U blocks, 20..23 are the V blocks, and 24 is the Y2 second-order
  Walsh block. Each block has a fixed 16-entry slot — there is no
  packing.
- `eobs[25]` is one end-of-block index per block, in the same ordering.

`BLOCKD` itself (`blockd.h` lines 193–203) is just a bundle of pointers
into those flat arrays plus a per-block `offset` (used for addressing
into the *destination frame buffer*, not the predictor scratch) and a
per-block mode-info `bmi` union:

```c
typedef struct blockd {
  short *qcoeff;
  short *dqcoeff;
  unsigned char *predictor;
  short *dequant;
  int offset;
  char *eob;
  union b_mode_info bmi;
} BLOCKD;
```

The job of `mbpitch.c` is to fill in `qcoeff`, `dqcoeff`, `predictor`,
`eob`, and `offset` for each of the 25 entries. The `dequant` pointer is
**not** set here — it is wired up elsewhere (in `vp8_setup_block_dequant`,
which selects among `dequant_y1`, `dequant_y2`, and `dequant_uv` per
plane), and `bmi` is filled in by `decodemv.c` during mode parsing.

## The 25-block layout in practice

The fixed block-index assignment used throughout the VP8 decoder is:

| Index range | Plane | Sub-shape  | Count |
|-------------|-------|------------|-------|
| 0..15       | Y     | 4x4        | 16    |
| 16..19      | U     | 4x4        | 4     |
| 20..23      | V     | 4x4        | 4     |
| 24          | Y2    | 4x4 (DCs)  | 1     |

Within luma, blocks are in *raster order*: index `r * 4 + c` is the
(r,c)-th 4x4 cell of the macroblock, with `(0,0)` in the upper-left and
`(3,3)` in the lower-right. Within U and V, blocks are in raster order
of a 2x2 grid: index `16 + r*2 + c` (U) and `20 + r*2 + c` (V) for
`r,c ∈ {0,1}`. The Y2 block (index 24) has no spatial position — it is
a virtual block whose 16 "samples" are the DC coefficients of the
sixteen Y blocks routed through a Walsh–Hadamard transform; see
`§4.2` and `§9.6` of the technical overview.

## `vp8_setup_block_dptrs` — wiring the per-MB scratch pointers

```c
void vp8_setup_block_dptrs(MACROBLOCKD *x) { ... }
```

This function is called once per `MACROBLOCKD` instance, immediately
after the structure is allocated, and never again — the pointers it
writes are *internal* (they all point inside `x` itself, into the four
flat arrays described above) and therefore remain valid for the entire
lifetime of `x`. It populates four of the seven fields of every
`BLOCKD`: `predictor`, `qcoeff`, `dqcoeff`, and `eob`.

The function is structured as four loops, one per "region" of the
predictor buffer plus a final flat loop over all 25 entries.

### Luma predictor pointers (blocks 0..15)

```c
for (r = 0; r < 4; ++r) {
  for (c = 0; c < 4; ++c) {
    x->block[r * 4 + c].predictor = x->predictor + r * 4 * 16 + c * 4;
  }
}
```

The luma plane occupies bytes `[0, 256)` of `predictor[384]`, laid out
as a 16x16 byte image with stride 16. For luma block `(r, c)`, the
upper-left sample of its 4x4 sub-region sits at row `4*r`, column `4*c`,
so its offset within the predictor is `(4*r)*16 + 4*c`, which is exactly
what the expression `r * 4 * 16 + c * 4` computes.

**Why** this layout: the 16x16 luma predictor is built as a single
contiguous block by the intra-16x16 predictors (`vpx_dc_predictor_16x16_c`
and friends, see overview §7) or by the inter motion-compensation kernels
(see §8). Both write into `xd->predictor` *as if it were one MB-wide
image of stride 16*. Indexing the 4x4 sub-blocks at stride 16 is what
makes that work transparently for the residual-add pass downstream.

**Invariant**: `block[i].predictor` for `i ∈ [0, 16)` points inside the
range `[x->predictor, x->predictor + 256)`, and the sixteen 4x4 windows
tile that region exactly.

### U predictor pointers (blocks 16..19)

```c
for (r = 0; r < 2; ++r) {
  for (c = 0; c < 2; ++c) {
    x->block[16 + r * 2 + c].predictor =
        x->predictor + 256 + r * 4 * 8 + c * 4;
  }
}
```

U occupies bytes `[256, 320)`, an 8x8 byte image with stride 8. Block
`16 + r*2 + c` is the (r,c) 4x4 cell of that 8x8 image, so its base
offset is `256 + (4*r)*8 + 4*c`. The `+ 256` is the per-plane base; the
stride argument inside the multiplication is now `8`, not `16`, because
chroma is 2:0:0-subsampled in each dimension and so the per-MB chroma
extent is half of luma.

**Why** the stride differs: chroma is downsampled 2:1 in both axes, so
an 8x8 chroma image corresponds to the same MB area as a 16x16 luma
image. The same logical "step down to the next row of sub-blocks" is
`4 * stride` either way, but the absolute number of bytes per row of
the underlying plane is plane-dependent. Hard-coding `16` and `8`
correctly here is what lets the intra/inter chroma predictors (e.g.
`vpx_dc_predictor_8x8_c`) write into a flat 8x8 region and still have
the per-4x4 IDCT pass see its own quadrant at the right pointer.

### V predictor pointers (blocks 20..23)

```c
for (r = 0; r < 2; ++r) {
  for (c = 0; c < 2; ++c) {
    x->block[20 + r * 2 + c].predictor =
        x->predictor + 320 + r * 4 * 8 + c * 4;
  }
}
```

V occupies the remaining 64 bytes, `[320, 384)`, identical in shape to U.
The only difference from the U loop is the base index (20 instead of 16)
and the base offset (320 instead of 256).

**Note**: index 24 — the Y2 block — is *not* assigned a predictor
pointer. That is deliberate. Y2 has no spatial-domain reconstruction:
its sixteen output samples are not pixels to be added to a predictor
but DC coefficients to be scattered back into the sixteen Y blocks (see
overview §4.2 and §9.6). Leaving `block[24].predictor` uninitialized is
safe because no code path dereferences it; the IDCT-block dispatcher
treats Y2 as a coefficient-domain transform only.

### Coefficient and EOB pointers (blocks 0..24)

```c
for (r = 0; r < 25; ++r) {
  x->block[r].qcoeff  = x->qcoeff  + r * 16;
  x->block[r].dqcoeff = x->dqcoeff + r * 16;
  x->block[r].eob     = x->eobs    + r;
}
```

This loop runs over *all* 25 entries, including Y2 (which crucially does
have qcoeff/dqcoeff/eob storage — its 16 Walsh coefficients live in
`qcoeff[24*16 .. 24*16+16)`, i.e. `qcoeff[384..400)`). Each block gets
a 16-coefficient slot, and `eob` is a pointer to a single `char` in the
shared `eobs[25]` array.

**Why per-block `eob` is stored as a pointer rather than an inline byte**:
the dequant / IDCT dispatchers in `idct_blk.c` consume `xd->eobs[]` as a
flat array, while the token-parsing path in `detokenize.c` writes
through `block[i].eob`. Both paths must see the same byte, so the
indirection — `block[i].eob = &xd->eobs[i]` — gives the token parser an
addressable per-block slot without imposing a separate scan to copy the
values back into the flat array.

**Invariants** after the function returns:
- `block[i].qcoeff   == xd->qcoeff  + 16*i` for all `i ∈ [0, 25)`.
- `block[i].dqcoeff  == xd->dqcoeff + 16*i` for all `i ∈ [0, 25)`.
- `block[i].eob      == &xd->eobs[i]` for all `i ∈ [0, 25)`.
- `block[i].predictor` is set for `i ∈ [0, 24)`, undefined for `i == 24`.

## `vp8_build_block_doffsets` — wiring the destination-frame offsets

```c
void vp8_build_block_doffsets(MACROBLOCKD *x) { ... }
```

Where `vp8_setup_block_dptrs` wires pointers into the *MB's own scratch
buffer*, `vp8_build_block_doffsets` wires offsets into the *destination
YV12 frame buffer* (`x->dst`, a `YV12_BUFFER_CONFIG`). The result is
stored in the integer field `BLOCKD::offset`. Downstream code adds this
offset to a per-MB base pointer (`x->dst.y_buffer + mb_row * 16 *
y_stride + mb_col * 16` for luma, similar for chroma) to land on the
top-left destination sample for each 4x4 sub-block.

This function must be re-invoked whenever `x->dst.y_stride` or
`x->dst.uv_stride` could have changed — in practice, after every
`vp8_setup_intra_recon` / dimension change. The offsets are stride-
dependent, hence cannot be computed once at structure-creation time.

### Luma destination offsets (blocks 0..15)

```c
for (block = 0; block < 16; ++block) /* y blocks */
{
  x->block[block].offset =
      (block >> 2) * 4 * x->dst.y_stride + (block & 3) * 4;
}
```

Here `block >> 2` is the 4x4 row index (0..3) and `block & 3` is the
column index (0..3) — the same raster decomposition as in
`vp8_setup_block_dptrs`. The vertical step is `4 * y_stride` (four rows
of the destination image), the horizontal step is `4` (four samples).

**Why** this differs from the predictor wiring: the *destination* image
has stride `y_stride`, which depends on the frame's allocated buffer
width (with border padding), not 16. The two strides — 16 in the
predictor scratch, `y_stride` in the destination — are why both
`predictor` and `offset` exist as separate fields on `BLOCKD`: the IDCT
pass reads from a 4x4 window at stride 16 inside the scratch and writes
to a 4x4 window at stride `y_stride` inside the frame.

### Chroma destination offsets (blocks 16..23) — the dual-write trick

```c
for (block = 16; block < 20; ++block) /* U and V blocks */
{
  x->block[block + 4].offset = x->block[block].offset =
      ((block - 16) >> 1) * 4 * x->dst.uv_stride + (block & 1) * 4;
}
```

This loop is the subtlest line in the file. It loops only over `block ∈
[16, 20)` — the four U indices — but the assignment is *chained*: each
iteration writes the same offset to both `block[block].offset` (the U
block) and `block[block + 4].offset` (the corresponding V block).

The arithmetic uses `((block - 16) >> 1)` for the chroma row (0 or 1)
and `(block & 1)` for the chroma column (0 or 1) — a 2x2 raster of 4x4
cells, addressed at stride `uv_stride`.

**Why** U and V share the offset: in libvpx's YV12 layout, U and V have
their own *base pointers* (`x->dst.u_buffer`, `x->dst.v_buffer`) but
**identical strides** (`x->dst.uv_stride`). The offset from each plane's
base to a given chroma 4x4 cell is therefore the same byte count for U
and V. Consumers (e.g. `vp8_dequant_idct_add_uv_block`) compute U and V
destinations as `u_buffer + offset` and `v_buffer + offset` from the
same `offset`, so storing it once per chroma quadrant — under two
different block indices — saves the duplicate computation.

**Note**: index 24 (Y2) is again skipped here. Y2 is purely a
coefficient-domain object; it has no destination location in the frame.

## How the wiring is consumed downstream

After `vp8_setup_block_dptrs` and `vp8_build_block_doffsets` have run,
the rest of the decoder treats a macroblock as a uniform array of 25
`BLOCKD` records. Three representative consumers:

1. **Token parsing** (`vp8/decoder/detokenize.c`): walks blocks in
   coefficient order (Y2 first if present, then 16 Y, then 4 U, then 4 V),
   writing into `*block[i].qcoeff` and `*block[i].eob`. Because every
   block's `qcoeff` was wired into the flat `xd->qcoeff` array,
   `detokenize` does not need to know that the storage is contiguous —
   yet `idct_blk` later iterates the flat array sequentially without
   touching `BLOCKD`.
2. **Dequantize + IDCT** (`vp8/common/idct_blk.c`, `idctllm.c`): reads
   `block[i].dqcoeff` and writes back into the predictor at
   `block[i].predictor`, then the inverse-residual-add pass writes the
   result to `dst + block[i].offset`.
3. **Intra prediction** (`vp8/common/reconintra4x4.c`): for the `B_PRED`
   mode, predicts into `block[i].predictor` per 4x4 cell using the
   already-reconstructed neighbouring samples, again relying on the
   stride-16 layout set up here.

## Summary

`mbpitch.c` is structural glue: forty lines of address arithmetic that
make the rest of the decoder oblivious to the precise layout of the
macroblock's flat scratch buffers and to the stride of the destination
frame. The two functions enforce three layout decisions on which the
entire VP8 decode pipeline depends:

1. The canonical 25-block index order — 16 Y (raster), 4 U (raster),
   4 V (raster), 1 Y2.
2. The plane-major stride-16/stride-8 layout of the 384-byte predictor
   scratch, with Y2 absent.
3. The dual U/V offset sharing exploited by the chroma reconstruction
   path.

Changing any of these would require touching nearly every other file
in `vp8/common/` and `vp8/decoder/` — they are not invariants of this
one file but of the whole codec.
