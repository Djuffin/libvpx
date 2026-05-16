# `vp8/common/setupintrarecon.c` — seeding the intra-prediction border

## Role in the decoder

VP8 intra prediction is, by construction, a function of the samples that
*surround* a block: the row immediately above it, the column immediately
to its left, and the single pixel at the above-left corner. Each of the
ten 4x4 intra modes and each of the four MB-level modes (`DC_PRED`,
`V_PRED`, `H_PRED`, `TM_PRED`) reads from one or more of those
neighbours. The VP8 technical overview summarises this in §7.2:

> VP8 intra modes read one row of samples above the block (extended one
> block to the right for `B_LD_PRED`), one column to the left, and the
> single pixel above-left. At the picture boundary those samples are
> *not* clipped — they are replaced by constants:
>
> - `127` above (vp8/common/setupintrarecon.c:18),
> - `129` to the left (line 20),
> - `127` at the top-left corner.

At the very first macroblock row of the frame there is no above-row to
read; at the leftmost column of every row there is no left-column to
read. The bitstream spec resolves this by *defining* the missing
samples to be the constants `127` (above) and `129` (left). It is
`vp8/common/setupintrarecon.c` that materialises those constants in
memory, by writing them into the byte immediately above and the byte
immediately to the left of each plane in the freshly-allocated YV12
buffer. From that point on, the intra predictors do not need to test
whether they sit on a picture edge — the boundary samples already hold
the right values, and a plain pointer dereference produces the
specified result.

The file is tiny — two non-static functions plus one `static INLINE`
helper in the header — but it is on the hot path of every single
keyframe and of every intra-coded macroblock in every inter frame. The
two callers in the decoder are `vp8_decode_frame` in
`vp8/decoder/decodeframe.c:483` (single-thread / row-loop entry, calls
the *top-line* variant once per frame and the *left* helper once per MB
row), and the multi-thread equivalent in
`vp8/decoder/threading.c:357,881`. In the encoder, the full
`vp8_setup_intra_recon` is called on the newly-reconstructed buffer
before encoding starts (`vp8/encoder/encodeframe.c:605`,
`vp8/encoder/firstpass.c:527`).

The file's listing in the decoder build inventory (`vp8_files.md`,
section A, "vp8/common/") simply reads:

```
setupintrarecon.c          intra-prediction border init
```

That single line is the entire job: write the border seeds. The
remaining sections of this document explain the geometry of the writes,
the choice of the two magic numbers, and why each of the four MB-level
intra modes degenerates to a neutral grey prediction when fed those
seeds.

## The shape of a YV12 buffer

The functions in this file are method-like extensions of
`YV12_BUFFER_CONFIG`, the frame-buffer descriptor defined in
`vpx_scale/yv12config.h:29`. Only six of its members matter here:

```c
typedef struct yv12_buffer_config {
  int y_width;
  int y_height;
  int y_stride;

  int uv_width;
  int uv_height;
  int uv_stride;

  uint8_t *y_buffer;
  uint8_t *u_buffer;
  uint8_t *v_buffer;
  ...
} YV12_BUFFER_CONFIG;
```

`y_buffer`, `u_buffer`, `v_buffer` point at the *top-left visible
pixel* of each plane. The allocation function
(`vp8_yv12_alloc_frame_buffer`, declared at line 69 of the same
header) reserves a border of `VP8BORDERINPIXELS = 32` (line 23 of
`yv12config.h`) on every side, so the bytes at `y_buffer - 1`,
`y_buffer - y_stride`, `y_buffer - 1 - y_stride`, `y_buffer + y_width`
and so on are all valid, writeable memory belonging to the same
allocation. This is the property that lets `setupintrarecon.c`
unconditionally write *one row above* and *one column to the left* of
the active picture without any boundary check.

The chroma planes are 4:2:0, so `uv_width = y_width / 2` and
`uv_height = y_height / 2`, and the same border-extension argument
applies with `uv_stride` in place of `y_stride`.

## `vp8_setup_intra_recon` — seed every row and the top line

```c
void vp8_setup_intra_recon(YV12_BUFFER_CONFIG *ybf) {
  int i;

  /* set up frame new frame for intra coded blocks */
  memset(ybf->y_buffer - 1 - ybf->y_stride, 127, ybf->y_width + 5);
  for (i = 0; i < ybf->y_height; ++i) {
    ybf->y_buffer[ybf->y_stride * i - 1] = (unsigned char)129;
  }

  memset(ybf->u_buffer - 1 - ybf->uv_stride, 127, ybf->uv_width + 5);
  for (i = 0; i < ybf->uv_height; ++i) {
    ybf->u_buffer[ybf->uv_stride * i - 1] = (unsigned char)129;
  }

  memset(ybf->v_buffer - 1 - ybf->uv_stride, 127, ybf->uv_width + 5);
  for (i = 0; i < ybf->uv_height; ++i) {
    ybf->v_buffer[ybf->uv_stride * i - 1] = (unsigned char)129;
  }
}
```

**What.** For each of the three planes, this writes `127` across the
row of pixels immediately above the visible area, and writes `129`
into the byte immediately to the left of every visible row. The same
pattern is repeated for Y, U, V with the appropriate stride and
dimensions.

**Why.** This is the *full* seed: every row of the picture gets its
own left-edge seed byte. The full seed is what the encoder and the
encoder's first-pass analyser need, because the encoder freely
visits MBs out of raster order (e.g. trial-encoding, RDO),
and so cannot rely on the just-reconstructed left neighbour having
already been produced. The single-threaded decoder, by contrast,
decodes strictly in raster order and *overwrites* the left-edge byte
with the rightmost reconstructed pixel of each MB as it goes — so it
only needs the seed to be correct for the leftmost MB of each row.
That economy is realised by the cheaper variants below.

**Invariants.**
- `ybf->y_buffer - 1 - ybf->y_stride` is a valid address; the YV12
  allocator guarantees at least one row of border above and one
  pixel of border to the left.
- `ybf->y_width + 5` covers: the above-left corner (1 byte), the full
  above row (`y_width` bytes), plus 4 padding bytes to the right of
  the active area. The four right-side bytes exist so that
  `B_LD_PRED` (a 4x4 intra mode that reads four samples *to the right*
  of its block's above-row) can do so even for the rightmost 4x4
  column of the rightmost MB. The geometry of `B_LD_PRED`'s reach is
  spelled out in the technical overview, §7.3:

  > VP8 intra modes read one row of samples above the block (extended
  > one block to the right for `B_LD_PRED`) …

- The "+5" therefore decomposes as: 1 above-left corner + `y_width`
  above-row + 4 extra for the right edge.
- `ybf->y_buffer[ybf->y_stride * i - 1]` is the byte one column to the
  left of row `i` — again valid because of the 32-pixel left border.
- Y and UV are independent; nothing depends on the order in which the
  three planes are processed.

**How used.** The encoder calls this once per frame before macroblock
encoding (`vp8/encoder/encodeframe.c:605`) and once per frame in the
first pass (`vp8/encoder/firstpass.c:527`). The decoder does **not**
call this routine in the minimal build; it uses the two cheaper
variants below.

## `vp8_setup_intra_recon_top_line` — seed only the top border

```c
void vp8_setup_intra_recon_top_line(YV12_BUFFER_CONFIG *ybf) {
  memset(ybf->y_buffer - 1 - ybf->y_stride, 127, ybf->y_width + 5);
  memset(ybf->u_buffer - 1 - ybf->uv_stride, 127, ybf->uv_width + 5);
  memset(ybf->v_buffer - 1 - ybf->uv_stride, 127, ybf->uv_width + 5);
}
```

**What.** Exactly the three `memset(…, 127, …)` writes from
`vp8_setup_intra_recon`, with the per-row left-column loops omitted.

**Why.** The decoder is a strict raster walker. Once the first row of
MBs has been decoded and reconstructed, the *next* row's "above row" is
no longer a synthetic seed: it is the *bottom row* of the just-finished
MB row, written there by the reconstruction step. Only the very first
MB row needs a synthetic above-row. So the decoder seeds the top
border once per frame, and leaves the per-row left-edge seeding to be
done lazily, one row at a time, inside the row loop (see
`setup_intra_recon_left` below). This saves `y_height + 2 * uv_height`
single-byte stores per frame compared to `vp8_setup_intra_recon`.

**Invariants.** Same as the `memset` calls in `vp8_setup_intra_recon`.
The "+5" trailing bytes again exist for `B_LD_PRED`'s right-reach on
the top MB row.

**How used.** Called once per frame, immediately before the macroblock
decode loop begins:

- `vp8/decoder/decodeframe.c:483` — single-thread / row-of-MBs entry.
- `vp8/decoder/threading.c:881` — multithread row-MT path.

## `setup_intra_recon_left` — seed the left column of one MB row

Declared `static INLINE` in `vp8/common/setupintrarecon.h:23`:

```c
static INLINE void setup_intra_recon_left(unsigned char *y_buffer,
                                          unsigned char *u_buffer,
                                          unsigned char *v_buffer, int y_stride,
                                          int uv_stride) {
  int i;

  for (i = 0; i < 16; ++i) y_buffer[y_stride * i] = (unsigned char)129;

  for (i = 0; i < 8; ++i) u_buffer[uv_stride * i] = (unsigned char)129;

  for (i = 0; i < 8; ++i) v_buffer[uv_stride * i] = (unsigned char)129;
}
```

**What.** Writes the `129` left-column seed for the **16 pixel rows of
a single luma MB** and the corresponding **8 pixel rows of each chroma
MB**. The pointers passed in are pre-offset to address the column
*just left* of the leftmost MB of the current MB row — see the caller
below.

**Why.** This is the row-loop counterpart of
`vp8_setup_intra_recon_top_line`. The decoder seeds the top border
once per frame and the left border once per MB row, deferring each
write until the row that needs it. This amortises the writes inside
the loop where they are most cache-friendly (the bytes are written
right before they are read by the first MB of the row's intra
predictor).

The reason only the leftmost MB needs seeding is the same as before:
after the first MB of the row is decoded and its reconstruction is
written into the frame buffer, the byte one column to the left of the
*second* MB is exactly the rightmost pixel of the *first* MB, which is
genuine reconstructed data. No seed is required there or anywhere
further right in the row.

**Invariants.**
- The caller must pass pointers to the leftmost-byte-of-each-row of
  the current MB row, i.e. `xd->recon_above[plane] - 1` after
  `recon_above` has been set to the start of the row's destination.
- The 16 (luma) and 8 (chroma) loop counts match the MB dimensions in
  4:2:0; this routine writes exactly one MB's worth of left-edge
  samples per call.

**How used.** Called once per MB row, inside the row loop in
`vp8/decoder/decodeframe.c:522` (and the threaded version in
`vp8/decoder/threading.c:357`). The pointer arithmetic that prepares
its arguments is worth quoting in full:

```c
xd->recon_above[0] = dst_buffer[0] + recon_yoffset;
xd->recon_above[1] = dst_buffer[1] + recon_uvoffset;
xd->recon_above[2] = dst_buffer[2] + recon_uvoffset;

xd->recon_left[0] = xd->recon_above[0] - 1;
xd->recon_left[1] = xd->recon_above[1] - 1;
xd->recon_left[2] = xd->recon_above[2] - 1;
...
setup_intra_recon_left(xd->recon_left[0], xd->recon_left[1],
                       xd->recon_left[2], xd->dst.y_stride,
                       xd->dst.uv_stride);
```

The pointer `xd->recon_left[0]` thus addresses the byte one column to
the left of the topmost pixel of the row's leftmost luma MB. Writing
`129` at `[0, y_stride, 2*y_stride, …, 15*y_stride]` from there
fills the entire leftmost-MB left-column seed.

## Why `127` above and `129` to the left

The numbers look arbitrary; they are not. They are chosen so that, in
the absence of any real neighbour data, each of the four MB-level
intra modes produces the *neutral grey* value `128` — the midpoint of
the 8-bit luma range. The technical overview, §7.2, states the rule
plainly:

> The asymmetry (127/129) ensures that for DC-only neighbors the
> average rounds to 128 — the neutral grey VP8 uses when nothing is
> decoded yet.

Walking through the four modes, treating "the top-left MB" as the
worst case (no above row, no left column, no above-left pixel):

**`DC_PRED` — average of the above row and the left column.** For a
16x16 block, with 16 above samples all `127` and 16 left samples all
`129`, the DC value is computed (in `vpx_dsp/intrapred.c`) as the
sum of all 32 samples plus a rounding constant of 16, divided by 32:

```
(16 * 127 + 16 * 129 + 16) / 32
  = (2032 + 2064 + 16) / 32
  = 4112 / 32
  = 128.5  →  truncates to 128
```

The two seeds were chosen precisely so that their *mean* is 128 — and
because the integer division rounds toward zero of a half-integer, the
choice `(127, 129)` lands on 128 rather than the alternative `(128,
128)` would have. (`(128, 128)` would also yield 128, of course, but
the asymmetric pair has additional virtues for `TM_PRED` and `H_PRED`,
described below.)

**`V_PRED` — replicate the above row downward.** Every column of the
predictor takes the value of the seed byte directly above it: `127`.
That is *one* below grey. This is a deliberate, tiny bias: when the
top edge is synthetic, the predictor errs slightly dark rather than
randomly. The same logic applies to chroma.

**`H_PRED` — replicate the left column rightward.** Every row of the
predictor takes the value of the seed byte directly to its left:
`129`. That is *one* above grey. Symmetric bias in the opposite
direction. So `V_PRED` and `H_PRED` differ by exactly two; their
*average* (which is what `DC_PRED` would compute if it sampled one row
and one column) is again `128`.

**`TM_PRED` — "TrueMotion": `p[i,j] = L[i] + A[j] − TL`.** With
`L[i] = 129`, `A[j] = 127`, and the top-left corner pixel `TL = 127`
(it sits inside the `memset(..., 127, ...)` region), the formula
becomes:

```
p[i,j] = 129 + 127 − 127 = 129
```

If instead the seeds had been `(128, 128, 128)`, `TM_PRED` would have
emitted `128` everywhere — which is fine. With the actual `(129, 127,
127)`, it emits `129` everywhere — one above grey. The asymmetry
costs nothing here; it is `DC_PRED`'s rounding that drives the choice.

**`B_PRED` (per-4x4 intra modes).** Each of the ten 4x4 modes
(`B_DC_PRED`, `B_TM_PRED`, `B_VE_PRED`, `B_HE_PRED`, `B_LD_PRED`,
`B_RD_PRED`, `B_VR_PRED`, `B_VL_PRED`, `B_HD_PRED`, `B_HU_PRED`)
reads at most one row of four above samples (eight for `B_LD_PRED`),
one column of four left samples, and the above-left pixel. The
arithmetic in each predictor is a small fixed combination of those
inputs with rounding-to-nearest. The `(127, 129, 127)` seeds were
chosen so that *all* of them either emit `128` or are within one of
it when fed only seeds. This is why the four trailing bytes of the
top-row `memset` (the "+5" past `y_width`) also get the `127` value:
they participate in `B_LD_PRED`'s eight-sample above-row.

The take-away: the seed values **127 / 129 / 127** are the unique
8-bit triple (under the constraint "the above and left seeds differ by
two and average to 128") that makes every VP8 intra predictor
degenerate to the neutral grey `128 ± 1` when fed only synthetic
boundary samples. The decoder therefore needs no special case for the
top-left MB of a keyframe: it runs the same `vp8_build_intra_predictors_*`
dispatch tables as every other MB, and the right values come out
automatically.

## Why this file does so little

A simpler implementation would set the entire YV12 border to `128`
everywhere. That would also produce a neutral grey, but it would lose
the `(127, 129)` asymmetry, and `DC_PRED` in particular would no
longer have a guaranteed integer result — the rounding behaviour
depends on the choice of seeds. The current scheme also lets the
decoder distinguish between "above border" and "left border" if it
ever needs to (it currently does not, but the encoder's RDO does, in
trial-encode paths that re-seed mid-frame).

The other reason for keeping the file tiny is that the work happens
exactly *once* per frame for the top line and *once per MB row* for
the left column, in straight-line memory writes that the hardware
prefetcher handles well. There is no SIMD version because there is
nothing to vectorise: the cost is dominated by the two `memset`s, and
those are already platform-optimised inside libc.

## Summary

`vp8/common/setupintrarecon.c` exists to make a single property of the
VP8 intra-prediction subsystem true: *the byte one row above and one
column to the left of every reconstructed pixel always holds a
defined value*. At a picture edge that value is synthetic — `127`
above, `129` to the left, `127` at the corner — and is engineered so
that all five intra modes (the four MB-level modes plus `B_PRED`'s
ten 4x4 modes) produce the neutral grey value `128` when fed only
synthetic samples. Everywhere else the value is real reconstructed
data, written there by the IDCT/reconstruction step of the preceding
MB. The two `memset`-based seeding routines plus the inline
left-column helper are the entire mechanism.
