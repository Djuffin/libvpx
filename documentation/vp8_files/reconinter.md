# `vp8/common/reconinter.c` — inter-prediction reconstruction

`reconinter.c` is the file that, for every macroblock VP8 decides to
predict from a previous frame, *materialises the prediction samples* —
that is, copies (or sub-pel-filters) the right rectangle out of the
reference frame and lays it down into the output plane the residual is
about to be added to. It is one of the two "reconstruction" units in
`vp8/common/` (its sibling being `reconintra*.c` for spatial
prediction). Although it lives under `common/` and a few of its entry
points are encoder-only, the central code path is what a pure VP8
decoder runs for every inter-coded macroblock of every frame.

## Role in the decoder

A VP8 macroblock that is not intra-coded carries one of four
inter-prediction modes (`NEARESTMV`, `NEARMV`, `NEWMV`, `SPLITMV`; see
`blockd.h:73`). The first three describe a single 16×16 motion vector
that applies to the whole luma block; `SPLITMV` describes a partition
of the luma 16×16 into 2, 4, 8 or 16 sub-blocks each with its own
motion vector. Whichever mode the bitstream chose, the decoder reaches
the same one-line dispatch into this file:

```c
void vp8_build_inter_predictors_mb(MACROBLOCKD *xd) {
  if (xd->mode_info_context->mbmi.mode != SPLITMV) {
    vp8_build_inter16x16_predictors_mb(xd, …);
  } else {
    build_4x4uvmvs(xd);
    build_inter4x4_predictors_mb(xd);
  }
}
```

(`vp8/common/reconinter.c:494–503`.)

Three things happen inside the dispatch, every time:

1. **Motion-vector decomposition.** A VP8 motion vector is stored at
   1/8-pel precision in a 16-bit pair `(row, col)`. The integer part
   (`>> 3`) is the offset to add to the reference-frame pointer; the
   fractional part (`& 7`) selects which sub-pel filter phase to apply.

2. **Reference-sample fetch.** The reference pointer is obtained from
   `MACROBLOCKD.pre` (a YV12 view of LAST / GOLDEN / ALTREF that the
   higher-level frame loop has already chosen).

3. **Splat into the prediction buffer.** Either a fast plain-copy (when
   the MV is integer-aligned on a full pixel) or one of the four
   sub-pel "predict" kernels (`subpixel_predict4x4`, `…8x4`, `…8x8`,
   `…16x16`) writes the predicted samples into the destination plane —
   in the decoder, directly into the `dst` YV12 frame the residual
   will be added to; in the encoder, into the temporary `predictor[]`
   scratch buffer inside `MACROBLOCKD`.

The chroma planes are 2:1 subsampled, so for the 16×16 case they are
predicted with an 8×8 MV derived from the luma MV by halving; for the
4×4-split case the four 4×4 luma MVs covering a chroma 4×4 are
averaged. Both derivations are sign-aware (rounded toward zero in a
specific way explained below) and both are then re-quantised to the
codec's "full-pixel" lattice when the active segment runs in full-pixel
MV mode. This file owns both reductions.

VP8's MVs are allowed to point a few pixels outside the picture; the
border is extended by `vpx_scale/yv12extend.c` to a fixed width
(`VP8BORDERINPIXELS = 32`) precisely so the unconditional fetch in this
file does not need to bounds-check on the fast path. When an MV points
*so* far outside that no in-picture samples would contribute, a clamp
routine in this file (`clamp_mv_to_umv_border`) snaps the MV to a
representative on the border before the fetch.

The rest of this document walks every definition in the file in
narrative order.

---

## Plain-copy kernels

When the motion vector is integer-aligned (fractional row and col are
both zero), the prediction is literally a rectangular copy from the
reference plane to the destination plane. The file defines three such
copies, sized to match the three block geometries VP8 ever needs.

### `vp8_copy_mem16x16_c`

```c
void vp8_copy_mem16x16_c(unsigned char *src, int src_stride,
                         unsigned char *dst, int dst_stride) {
  for (int r = 0; r < 16; ++r) {
    memcpy(dst, src, 16);
    src += src_stride; dst += dst_stride;
  }
}
```

(`vp8/common/reconinter.c:23–33`.) Sixteen 16-byte memcpys laid out for
the compiler to recognise. The `_c` suffix marks this as the portable
fallback published through `vp8_rtcd.h`; the build replaces it with a
NEON / SSE2 / DSPR2 / MSA / MMI / LSX implementation when one is
available (see `vp8/common/rtcd_defs.pl:133–141`). The decoder calls it
through the symbol `vp8_copy_mem16x16` (without the `_c`), which the
RTCD dispatcher points at the chosen variant during
`vpx_codec_dec_init_ver()`.

**Why it exists separately from the sub-pel kernels.** The integer-MV
case is statistically frequent and bypasses the sub-pel filter
entirely; even on the C path, replacing a six-tap convolution with a
straight `memcpy` is a large win. The dispatch site in
`vp8_build_inter16x16_predictors_mb` (`reconinter.c:319–324`) explicitly
checks whether *any* of the fractional bits are set and branches to
this kernel when they are all zero.

**Invariant.** `src` must be a valid pointer into the
border-extended reference frame, i.e. it may sit inside the
`VP8BORDERINPIXELS`-wide border that `yv12extend.c` filled — the kernel
itself does no bounds checking.

### `vp8_copy_mem8x8_c` and `vp8_copy_mem8x4_c`

```c
void vp8_copy_mem8x8_c(unsigned char *src, int src_stride,
                       unsigned char *dst, int dst_stride);
void vp8_copy_mem8x4_c(unsigned char *src, int src_stride,
                       unsigned char *dst, int dst_stride);
```

(`vp8/common/reconinter.c:35–57`.) The 8×8 variant is used for chroma
planes in 16×16-MV mode and for each "4b" (four-4×4-block) luma
quadrant in the `partitioning < 3` cases of `SPLITMV`. The 8×4 variant
is used to amortise across two horizontally-adjacent 4×4 sub-blocks
that share the same MV (the so-called "2b" path). Same RTCD treatment
as the 16×16 kernel.

**Why 8×4 exists.** The bitstream cost of MV-coding inside `SPLITMV` is
high, so the common case is that adjacent 4×4 blocks within the chosen
partition repeat the same MV. When two horizontally adjacent 4×4s carry
identical MVs, the decoder fuses their copies into one 8×4 call rather
than running the predict step twice. The fusion check is the comparison
`d0->bmi.mv.as_int == d1->bmi.mv.as_int` you see repeatedly throughout
the file.

---

## 4×4 sub-block predict, with sub-pel branch

The file has four near-identical helpers that wrap "predict one
sub-block": one public (`vp8_build_inter_predictors_b`, written into
`MACROBLOCKD.predictor[]` at a fixed stride of `pitch`), three private
(`build_inter_predictors4b`, `build_inter_predictors2b`,
`build_inter_predictors_b`, all writing into the destination YV12).
They share the same skeleton:

```c
ptr = base_pre + d->offset
      + (d->bmi.mv.as_mv.row >> 3) * pre_stride
      + (d->bmi.mv.as_mv.col >> 3);

if (d->bmi.mv.as_mv.row & 7 || d->bmi.mv.as_mv.col & 7) {
  sppf(ptr, pre_stride, col & 7, row & 7, dst, dst_stride);
} else {
  /* plain copy */
}
```

(See, for example, `reconinter.c:59–80`.) Three things to notice.

* `d->offset` is the precomputed byte offset of *this* sub-block's
  top-left pixel within a generic YV12 plane (set up once per frame in
  `mbpitch.c`). Add the reference base pointer and the integer MV and
  you have the source rectangle.

* The fractional bits select the sub-pel filter phase. VP8 sub-pel
  filters are 1/8-pel; `xoffset` and `yoffset` are 0..7 (see the
  prototype on `blockd.h:205`). Phase 0 is identity; the early branch
  diverts that case to the plain-copy path. Phases 1..7 do the work.

* The sub-pel kernel is dispatched through a function pointer rather
  than a static call. That pointer is `MACROBLOCKD.subpixel_predict*`
  (`blockd.h:284–287`), which `vp8_setup_intra_recon`-adjacent setup
  initialises to either the six-tap "sixtap" family or the two-tap
  "bilinear" family, depending on `xd->mode_info_context->mbmi.mode`
  policy chosen by the encoder. (For SPLITMV the standard sixtap is
  always used.)

### `vp8_build_inter_predictors_b`

```c
void vp8_build_inter_predictors_b(BLOCKD *d, int pitch,
                                  unsigned char *base_pre,
                                  int pre_stride, vp8_subpix_fn_t sppf);
```

(`reconinter.c:59–80`.) Writes a single 4×4 prediction into
`d->predictor` (the per-block scratch field on `BLOCKD`, into which all
predictors used to land in older versions of this code; `d->predictor`
in turn aliases into the `MACROBLOCKD.predictor[384]` mega-buffer
declared at `blockd.h:210`). The caller passes the desired stride
(`pitch`) and the sub-pel kernel (`sppf`). Used inside this file by
`vp8_build_inter4x4_predictors_mbuv` (encoder-only path) and exported
via `reconinter.h:26` for the encoder's RD loop.

**Why a separately exposed entry.** The encoder needs to compute
predictions into the scratch buffer (`x->predictor[]`) for rate-
distortion search before committing them to the output frame. The
decoder never calls this function; in the decoder the equivalent work
goes through the *static* `build_inter_predictors_b` below (note the
lower-case-static naming) which writes straight into the destination
YV12.

### `build_inter_predictors4b` (static)

```c
static void build_inter_predictors4b(MACROBLOCKD *x, BLOCKD *d,
                                     unsigned char *dst, int dst_stride,
                                     unsigned char *base_pre, int pre_stride);
```

(`reconinter.c:82–95`.) Treats a quartet of 4×4 sub-blocks as one 8×8
unit: dispatches `subpixel_predict8x8` (sixtap or bilinear, whichever
is installed in the `MACROBLOCKD`) or `vp8_copy_mem8x8`. Used by the
"coarse" `SPLITMV` partitionings — `partitioning < 3` — where the 16×16
luma block was split into 2 or 4 macro-partitions that each happen to
contain four 4×4s sharing one MV. The reason for the larger kernel
isn't correctness (four invocations of the 4×4 kernel would produce
the same samples); it's throughput: an 8×8 sub-pel convolution
amortises the horizontal/vertical filter setup across more output
samples than four independent 4×4 convolutions.

### `build_inter_predictors2b` (static)

```c
static void build_inter_predictors2b(MACROBLOCKD *x, BLOCKD *d,
                                     unsigned char *dst, int dst_stride,
                                     unsigned char *base_pre, int pre_stride);
```

(`reconinter.c:97–110`.) Same idea, dispatched to
`subpixel_predict8x4` / `vp8_copy_mem8x4`. Used whenever two
horizontally adjacent 4×4 sub-blocks share an MV — the
`d0->bmi.mv.as_int == d1->bmi.mv.as_int` fusion mentioned earlier.
This is the most-used helper for `SPLITMV` chroma, where the four U
(and four V) sub-blocks are visited as two pairs.

### `build_inter_predictors_b` (static, lower case)

```c
static void build_inter_predictors_b(BLOCKD *d, unsigned char *dst,
                                     int dst_stride, unsigned char *base_pre,
                                     int pre_stride, vp8_subpix_fn_t sppf);
```

(`reconinter.c:112–133`.) The "true 4×4" predict, used when a pair of
adjacent 4×4s do not share their MV and have to be filtered
individually; identical body to the public `vp8_build_inter_predictors_b`
above except that it writes into the destination plane at a caller-
provided stride rather than into `BLOCKD.predictor` at a fixed
`pitch`. It is the workhorse of `partitioning == 3` (fully
4×4-partitioned) `SPLITMV` blocks.

---

## Encoder-only helpers

The next three functions are tagged `/*encoder only*/` in the source.
The decoder never reaches them; they exist in this file because the
encoder shares the prediction logic with the decoder and it was
convenient to keep all of inter-prediction in one translation unit.
For a fork that strips the encoder these can be excised, but they
compile cleanly and harmlessly without an encoder linked in.

### `vp8_build_inter16x16_predictors_mbuv`

```c
void vp8_build_inter16x16_predictors_mbuv(MACROBLOCKD *x);
```

(`reconinter.c:136–167`.) Predicts the chroma planes of a 16×16-MV
macroblock into the `predictor[256]` and `predictor[320]` slots of
`MACROBLOCKD.predictor` (a single 384-byte aligned array sized
`16*16 + 2*8*8`). The exact same chroma-MV derivation it performs is
inlined into the production path `vp8_build_inter16x16_predictors_mb`
below; the encoder needs a separate entry because its RD loop wants the
chroma prediction written into scratch rather than into the destination
frame.

### `vp8_build_inter4x4_predictors_mbuv`

```c
void vp8_build_inter4x4_predictors_mbuv(MACROBLOCKD *x);
```

(`reconinter.c:170–235`.) Same idea for `SPLITMV` chroma: it derives
the four 8×8 chroma MVs from the sixteen luma MVs (the averaging
algorithm explained below), then drives `build_inter_predictors2b` /
`vp8_build_inter_predictors_b` to write into each U/V `BLOCKD`'s own
`predictor` field. Decoder uses the in-place equivalent
`build_4x4uvmvs` + `build_inter4x4_predictors_mb` further down.

### `vp8_build_inter16x16_predictors_mby`

```c
void vp8_build_inter16x16_predictors_mby(MACROBLOCKD *x,
                                         unsigned char *dst_y, int dst_ystride);
```

(`reconinter.c:238–255`.) The luma-only variant of the 16×16 inter
predictor, with no MV clamping. The encoder calls it during motion
search where the candidate MV has already been clipped by the search
itself. Note the absence of `need_to_clamp_mvs` handling and the
absence of any chroma work; otherwise the body is identical to the
luma half of `vp8_build_inter16x16_predictors_mb`.

---

## MV clamping at the picture border

The two clamp routines exist because VP8 lets motion vectors point
quite far outside the picture, but only up to the edge of the border
extension that `vpx_scale/yv12extend.c` has materialised. If an MV
exceeds that, the result of the unbounded fetch in the predict kernels
would read past the allocated buffer. The clamp routines snap such
extreme MVs to a representative point on the border, exploiting the
fact that beyond a certain threshold the prediction is identically the
border pixel anyway.

### `clamp_mv_to_umv_border` (luma)

```c
static void clamp_mv_to_umv_border(MV *mv, const MACROBLOCKD *xd) {
  if (mv->col < (xd->mb_to_left_edge   - (19 << 3))) { … }
  else if (mv->col > xd->mb_to_right_edge + (18 << 3)) { … }
  if (mv->row < (xd->mb_to_top_edge    - (19 << 3))) { … }
  else if (mv->row > xd->mb_to_bottom_edge + (18 << 3)) { … }
}
```

(`reconinter.c:257–278`.) The numbers in this function are best
understood by counting pixels at the source-fetch site:

* `mb_to_left_edge` etc. are signed pixel-times-eight distances (the
  same 1/8-pel units MVs use) from the current macroblock's top-left
  corner to each frame edge; they're set up per macroblock in
  `mbpitch.c`.

* "19 << 3" is 19 pixels in 1/8-pel units. The six-tap sub-pel filter
  taps `t-2, t-1, t, t+1, t+2, t+3` around each output centre. A 16-wide
  output block centred at MV-implied column `c` therefore reads source
  columns `c-2 .. c+19` (i.e. 16+3 = 19 pixels right of the central
  column). If `mv->col < mb_to_left_edge - (19 << 3)` then *no* output
  tap of the 16-wide block can land on a real in-picture pixel —
  everything is in the left border. In that case the predicted block
  is uniformly the leftmost-column samples replicated, which is also
  exactly what a 16-pixel-clamped MV would fetch. So clamp to
  `mb_to_left_edge - (16 << 3)` and proceed.

* "18 << 3" on the right edge accounts for the asymmetry of the
  six-tap filter (two left taps, three right taps relative to the
  central pixel; on the right edge the binding direction reverses).

The comment block at the top of the function says exactly this. The
clamp is conditional on `mbmi.need_to_clamp_mvs`, a flag the decoder
sets during MV parsing when the MV is detected to be near the border
(see `decodemv.c`); for the overwhelming majority of macroblocks the
clamp is skipped entirely.

### `clamp_uvmv_to_umv_border` (chroma)

```c
static void clamp_uvmv_to_umv_border(MV *mv, const MACROBLOCKD *xd);
```

(`reconinter.c:281–295`.) Same shape, but operates on a chroma MV that
has already been derived from one or more luma MVs and halved. Note
that the thresholds are still expressed in luma-edge units: each chroma
column corresponds to two luma columns, so the chroma MV is doubled on
both sides of the comparison. The clamped value, however, is written
out at chroma scale (`>> 1`). Used only on the encoder's
`build_4x4uvmvs` path; the in-decoder 16×16 path inlines an equivalent
out-of-bounds check that simply *skips* the chroma fetch rather than
clamping (see below).

---

## The 16×16 main entry — `vp8_build_inter16x16_predictors_mb`

```c
void vp8_build_inter16x16_predictors_mb(MACROBLOCKD *x,
                                        unsigned char *dst_y,
                                        unsigned char *dst_u,
                                        unsigned char *dst_v,
                                        int dst_ystride, int dst_uvstride);
```

(`reconinter.c:297–357`.) This is the function the decoder runs for
every non-SPLITMV inter macroblock; everything else in the file either
feeds it or handles the SPLITMV alternative. Five things happen, in
order.

**1. MV fetch and optional clamp.** The single 16×16 MV is read from
`mode_info_context->mbmi.mv`, then conditionally clamped:

```c
_16x16mv.as_int = x->mode_info_context->mbmi.mv.as_int;
if (x->mode_info_context->mbmi.need_to_clamp_mvs)
  clamp_mv_to_umv_border(&_16x16mv.as_mv, x);
```

The `as_int / as_mv` union (defined on `mv.h`) lets the decoder copy
the (row, col) pair in a single 32-bit load.

**2. Luma source fetch.** `ptr_base + (row >> 3) * pre_stride + (col >> 3)`
is the byte address of the top-left source sample.

**3. Luma predict.** The fractional-part test is a single 32-bit mask
against `0x00070007` — both row and col fractional bits in one shot:

```c
if (_16x16mv.as_int & 0x00070007) {
  x->subpixel_predict16x16(ptr, pre_stride,
                           _16x16mv.as_mv.col & 7, _16x16mv.as_mv.row & 7,
                           dst_y, dst_ystride);
} else {
  vp8_copy_mem16x16(ptr, pre_stride, dst_y, dst_ystride);
}
```

(`reconinter.c:319–324`.) The bitwise OR over both axes in one
comparison is the kind of micro-optimisation that pays off because this
branch runs once per macroblock per frame and is, by far, the hot path
of inter decoding.

**4. Chroma MV derivation.** The luma MV is halved with a *sign-aware
rounding* dance:

```c
_16x16mv.as_mv.row += 1 | (_16x16mv.as_mv.row >> (sizeof(int)*CHAR_BIT - 1));
_16x16mv.as_mv.col += 1 | (_16x16mv.as_mv.col >> (sizeof(int)*CHAR_BIT - 1));
_16x16mv.as_mv.row /= 2;
_16x16mv.as_mv.col /= 2;
_16x16mv.as_mv.row &= x->fullpixel_mask;
_16x16mv.as_mv.col &= x->fullpixel_mask;
```

(`reconinter.c:327–334`.) The unusual expression
`1 | (mv >> (sizeof(int)*CHAR_BIT - 1))` evaluates to `+1` for
non-negative inputs and `-1` for negative inputs (because the
arithmetic right shift of a negative int is `-1` (`0xFF…FF`), whose OR
with `1` is still `-1`; for a non-negative input the shift is `0`).
Adding that to the MV before halving rounds *away from zero*, which is
the standard "biased toward larger magnitude" rounding VP8 mandates
for chroma MV derivation (RFC 6386 §13.6). It is written this way to
avoid the branch a more conventional `(mv >= 0 ? mv + 1 : mv - 1) / 2`
would compile to.

The subsequent `& x->fullpixel_mask` quantises the derived chroma MV
to full pixels when the current segment is in full-pixel-MV mode
(`fullpixel_mask = ~7`) and is a no-op (`fullpixel_mask = ~0`)
otherwise. The mask is segment-level state set up in `decodeframe.c`.

**5. Chroma out-of-bounds skip, then chroma predict.** Rather than
clamping, the chroma path simply *bails*:

```c
if (2 * _16x16mv.as_mv.col < (x->mb_to_left_edge   - (19 << 3)) || …) {
  return;
}
```

(`reconinter.c:336–341`.) Doubling the chroma MV puts it back into
luma units for the comparison against the same thresholds the luma
clamp uses. If the chroma MV would land entirely outside the picture
the function leaves `dst_u` / `dst_v` untouched. (The decoder relies
on these buffers having sensible prior contents — for an inter MB they
were initialised to whatever was left from a previous frame; the
residual adds onto them.) This is a long-standing libvpx behaviour
that matches the reference decoder.

Finally, with `pre_stride >>= 1` (chroma stride is half of luma), the
same `0x00070007` test selects between `subpixel_predict8x8` and
`vp8_copy_mem8x8`, called once each for U and V.

---

## The SPLITMV path

When the macroblock-level mode is `SPLITMV`, every 4×4 sub-block can
carry its own MV. The chroma MVs aren't transmitted; they're derived
from the luma MVs. Two static helpers split this work in the
decoder.

### `build_4x4uvmvs` (static)

```c
static void build_4x4uvmvs(MACROBLOCKD *x);
```

(`reconinter.c:456–492`.) Iterates the four 4×4 chroma sub-blocks (the
2×2 grid of chroma 4×4s) and for each derives an MV by averaging the
four luma 4×4 MVs that overlap it. Concretely, a U sub-block at chroma
grid position `(i, j)` covers luma sub-block indices
`yoffset, yoffset+1, yoffset+4, yoffset+5` where
`yoffset = i*8 + j*2`. The implementation:

```c
temp = sum-of-four-luma-rows;
temp += 4 + ((temp >> (sizeof(temp)*CHAR_BIT - 1)) * 8);
x->block[uoffset].bmi.mv.as_mv.row = (temp / 8) & x->fullpixel_mask;
```

For non-negative `temp` the second line adds `4` (so the integer divide
by 8 becomes a round-to-nearest with ties going *up*); for negative
`temp` it adds `4 + (-1)*8 = -4` (so the divide rounds toward more
negative, away from zero). Same sign-aware-rounding pattern as the
16×16 chroma derivation, but here the bias is `±4` because the divisor
is 8 rather than 2. The full-pixel mask is then applied, and the V
sub-block reuses the U sub-block's MV verbatim
(`block[voffset].bmi.mv.as_int = block[uoffset].bmi.mv.as_int`) since
U and V always share motion in VP8.

The clamp (`clamp_uvmv_to_umv_border`) is called on the resulting
chroma MV when the MB needed clamping, so the chroma path inside
`build_inter4x4_predictors_mb` can omit the clamp comment-block notes:
"uv mvs already clamped in build_4x4uvmvs()".

### `build_inter4x4_predictors_mb` (static)

```c
static void build_inter4x4_predictors_mb(MACROBLOCKD *x);
```

(`reconinter.c:359–454`.) The driver that actually splats sub-block
predictions into the destination YV12 for a `SPLITMV` macroblock.
Three loops:

*Luma.* Branches on `mbmi.partitioning`:

* `partitioning < 3` — coarse partitionings (2 or 4 macro-partitions,
  each a multiple of 8×8). Only four BLOCKDs (indices 0, 2, 8, 10) are
  the corners of 8×8 quadrants; each is predicted with the 8×8 helper
  `build_inter_predictors4b`. Their MVs are individually clamped if
  the MB asked for it.

* `partitioning == 3` — full 4×4 split. A pair-loop walks
  `i = 0, 2, 4, …, 14`; for each pair (`d0`, `d1`) of horizontally
  adjacent 4×4s, if their MVs are identical (`as_int` equal) it dispatches
  one 8×4 fused predict (`build_inter_predictors2b`), otherwise two
  individual 4×4 predicts (`build_inter_predictors_b`). Each `BLOCKD`'s
  `bmi` is first refreshed from the per-MB `mode_info_context->bmi[i]`
  (where the bitstream parser put it) and then optionally clamped.

*Chroma U.* Iterates `i = 16, 18`. Same fusion pattern: same MV →
`build_inter_predictors2b`; otherwise two 4×4 predicts. MVs were
already pre-clamped by `build_4x4uvmvs`, so no clamp here.

*Chroma V.* Iterates `i = 20, 22`. Identical to U.

The result, when this function returns, is that the destination luma
and chroma planes of the current macroblock hold the full inter
prediction — ready for the dequant+IDCT pass to add residuals on top
(see `vp8/common/idct_blk.c` and `idctllm.c`).

---

## Dispatcher — `vp8_build_inter_predictors_mb`

```c
void vp8_build_inter_predictors_mb(MACROBLOCKD *xd) {
  if (xd->mode_info_context->mbmi.mode != SPLITMV) {
    vp8_build_inter16x16_predictors_mb(xd, xd->dst.y_buffer, xd->dst.u_buffer,
                                       xd->dst.v_buffer, xd->dst.y_stride,
                                       xd->dst.uv_stride);
  } else {
    build_4x4uvmvs(xd);
    build_inter4x4_predictors_mb(xd);
  }
}
```

(`reconinter.c:494–503`.) The whole file boils down to this two-line
choice: derive-and-write 16×16, or derive-uv-MVs then write
sub-block-by-sub-block. This is the function called from the per-MB
loop in `decodeframe.c` for every inter macroblock. Both branches end
with the prediction already laid into `xd->dst`, which is the
contemporary frame's YV12 reconstruction buffer the residual will be
added to.

---

## Putting it together — fast path versus slow path

The "fast path" through this file, taken for the vast majority of
inter MBs in real-world video, is:

1. Mode is not `SPLITMV`.
2. `need_to_clamp_mvs` is false (the MV is well inside the picture).
3. The 16×16 MV's fractional part is zero (integer-aligned).
4. The derived chroma MV's fractional part is also zero.

Cost: one 16×16 memcpy-strip plus two 8×8 memcpy-strips, all
RTCD-dispatched to SIMD when available. No function-pointer call into
a sub-pel kernel, no border clamp.

The "slow path" the other branches accommodate:

* **Sub-pel motion.** Six-tap (or, by encoder choice, bilinear)
  convolution dispatched through `xd->subpixel_predict*`.
* **Border-touching motion.** `clamp_mv_to_umv_border` /
  `clamp_uvmv_to_umv_border` snap the MV to a representative on the
  edge; the chroma half of `vp8_build_inter16x16_predictors_mb`
  optimises further by skipping the chroma fetch entirely when the
  derived chroma MV would be wholly out of bounds.
* **Per-sub-block motion (`SPLITMV`).** `build_4x4uvmvs` derives four
  chroma MVs from sixteen luma MVs by sign-aware-rounded averaging;
  `build_inter4x4_predictors_mb` walks the partition tree, fusing
  adjacent same-MV 4×4 pairs into one 8×4 call when possible
  (`build_inter_predictors2b`), and grouping coarser partitionings into
  8×8 calls (`build_inter_predictors4b`) when the partition lets it.

What this file does *not* do: parse motion vectors (that is
`decodemv.c`), allocate or border-extend reference frames (that is
`alloccommon.c` / `vpx_scale/yv12extend.c`), or supply the actual
sub-pel filter taps (those live in `vp8/common/filter.c` and the
SIMD-specialised translations of the four sixtap/bilinear predict
kernels). It is the glue that turns "an MV plus a reference frame" into
"the right rectangle of samples in the right place in the output
frame", and nothing more.
