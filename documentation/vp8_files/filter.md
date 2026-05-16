# `vp8/common/filter.c` — sub-pixel interpolation for inter prediction

## Role in the decoder

Motion vectors in VP8 are stored at 1/8-pel precision (see
`vp8_technical_overview.md` §8.2), but the inter-prediction kernels need
to resample a reference frame at a *fractional* offset to produce the
predicted block before residual addition. `filter.c` provides the C
reference implementations of those resamplers, and the two filter
coefficient tables that drive every SIMD specialisation as well.

In the per-MB inter path (`reconinter.c`), each block's MV is split into
an integer `(mv_row, mv_col)` part and a fractional 3-bit phase
`(xoffset, yoffset) = (mv.col & 7, mv.row & 7)`. The integer part walks
the source pointer to the upper-left of the support region; the
fractional part indexes into one of the two tables defined here. The
predictor itself is then dispatched through a function-pointer field on
`MACROBLOCKD` (blockd.h:284–287):

```c
vp8_subpix_fn_t subpixel_predict;       /*  4x4 */
vp8_subpix_fn_t subpixel_predict8x4;
vp8_subpix_fn_t subpixel_predict8x8;
vp8_subpix_fn_t subpixel_predict16x16;
```

Each of those eight slots (four sizes × two filter families) is
populated either by the C reference defined in this file, or by an
arch-specific override published in `vp8_rtcd.h` (see §14 of the
technical overview). The technical-overview-level summary is "6-tap
horizontal then vertical", and that *is* what this file implements;
this document fills in the parts the overview elides — the coefficient
tables and their RFC 6386 origin, the rounding/normalisation
convention, the choice of separable order, and the read-ahead /
border-extension contract.

Two families of filters live here:

1. **Six-tap sub-pixel filters** (`vp8_sub_pel_filters`, `vp8_sixtap_predict*`),
   the default luma resampler from a key/inter frame whose
   `version == 0`. This is the path that gives VP8 most of its quality
   advantage over earlier MPEG/H.263-style codecs that use bilinear MC.

2. **Bilinear filters** (`vp8_bilinear_filters`, `vp8_bilinear_predict*`),
   a cheaper 2-tap fallback selected for chroma in all profiles, and
   for luma when the frame's `version` field is 1, 2, or 3 (see
   technical overview §16.4 — the "version → filter" table).

Both families are *separable*: the 2-D filter is realised as a 1-D
horizontal pass into an intermediate buffer followed by a 1-D vertical
pass into the destination. The same coefficient table is reused for
both passes because the filter is symmetric in the two spatial axes.

---

## Header surface (`filter.h`)

The header is tiny — it exposes only the two coefficient tables, the
normalisation constants, and the macroblock-edge sentinel:

```c
#define BLOCK_HEIGHT_WIDTH 4
#define VP8_FILTER_WEIGHT  128
#define VP8_FILTER_SHIFT     7
```

`VP8_FILTER_WEIGHT` is the gain that every filter row sums to (or that
the bilinear taps must sum to exactly); `VP8_FILTER_SHIFT` is its base-2
log, which is therefore the right shift applied after multiplication to
return to the 8-bit pixel domain. The rounding offset used everywhere
in this file is `VP8_FILTER_WEIGHT >> 1 = 64`, i.e. half the divisor —
ordinary "round-half-up" for non-negative quotients.

`BLOCK_HEIGHT_WIDTH = 4` is unused in this translation unit but is
exposed because callers (e.g. `reconinter.c`) want a named constant
when they iterate sub-blocks; it sits in this header for historical
reasons.

The two tables are declared with `DECLARE_ALIGNED(16, …)` so that they
live on a 16-byte boundary; this is a precondition of the SSE2/NEON
specialisations that index into them with aligned loads, not a need of
the C reference.

---

## The two coefficient tables

### `vp8_bilinear_filters[8][2]` — chroma / "simple" luma resampler

```c
DECLARE_ALIGNED(16, const short, vp8_bilinear_filters[8][2]) = {
  { 128, 0 }, { 112, 16 }, { 96, 32 }, { 80, 48 },
  { 64, 64 }, { 48, 80 },  { 32, 96 }, { 16, 112 }
};
```

**What it is.** Eight rows of two `int16_t` taps. Row index `p` ∈ {0..7}
is the *sub-pel phase* — the fractional part of the motion vector
component, multiplied by 8 (so 1/8-pel resolution). The two taps weight
the sample at integer position `0` and the sample at position `+1`,
respectively.

**Why these specific numbers.** For phase `p`, the desired bilinear
interpolant is `((8-p)/8) · s[0] + (p/8) · s[1]`. Multiplying by
`VP8_FILTER_WEIGHT = 128` yields `(128 - 16p)·s[0] + 16p·s[1]`, which
is exactly the table: row 0 is `{128, 0}` (no interpolation needed),
row 4 is `{64, 64}` (the midpoint), and so on by 16 per step. The
invariant `taps[0] + taps[1] == 128` holds in every row, ensuring DC
preservation.

**Invariants.**
- Every row sums to `VP8_FILTER_WEIGHT == 128`.
- All taps are non-negative; therefore the post-shift result is
  trivially in `[0, 255]` and the bilinear pass does **not** need a
  clamp. (Compare with the six-tap path, which does need one.)
- Row 0 is the identity. Callers that detect "no fractional offset"
  before dispatching should never *need* this row, and the
  `vp8_bilinear_predict*_c` wrappers assert exactly that:

  ```c
  // This represents a copy and is not required to be handled by optimizations.
  assert((xoffset | yoffset) != 0);
  ```

  (The assertion is *or* of both axes — an integer-pel offset on one
  axis combined with a fractional on the other is fine. What is
  forbidden is the all-zero phase, because that would not need this
  function at all.)

**Where it comes from.** RFC 6386 §6.5.2, "Bilinear filters".

### `vp8_sub_pel_filters[8][6]` — luma 6-tap resampler

```c
DECLARE_ALIGNED(16, const short, vp8_sub_pel_filters[8][6]) = {
  { 0,   0, 128,   0,   0, 0 },   /* phase 0 — identity */
  { 0,  -6, 123,  12,  -1, 0 },
  { 2, -11, 108,  36,  -8, 1 },   /* New 1/4 pel 6 tap filter */
  { 0,  -9,  93,  50,  -6, 0 },
  { 3, -16,  77,  77, -16, 3 },   /* New 1/2 pel 6 tap filter */
  { 0,  -6,  50,  93,  -9, 0 },
  { 1,  -8,  36, 108, -11, 2 },   /* New 1/4 pel 6 tap filter */
  { 0,  -1,  12, 123,  -6, 0 },
};
```

**What it is.** Eight rows, six taps each, indexed by the same 1/8-pel
phase as the bilinear table. Tap `k` weights the source sample at
relative offset `k - 2`, so the support spans positions
`{-2, -1, 0, +1, +2, +3}` around the integer-pel sample — two pixels
to the left/above, three to the right/below. This asymmetry is why the
table's *non-identity* rows are not mirror-symmetric: phase `p` and
phase `8-p` are mirror images, but each individual row is not
symmetric about its centre.

**Why six taps and these specific coefficients.** RFC 6386 §6.5.1
defines two 6-tap filters: an alpha-0.5 **bicubic** for 1/4-pel phases
(rows 2, 4, 6) and a milder filter for the 1/8-pel "off-grid" phases
(rows 1, 3, 5, 7). The in-code comments label the bicubic rows
explicitly ("New 1/4 pel 6 tap filter", "New 1/2 pel 6 tap filter").
The "alpha = -0.5" remark on row 0 is a historical artefact: row 0 is
the identity, but the off-grid rows (1, 3, 5, 7) are derived as if
extending the same bicubic kernel to 1/8-pel.

**Invariants.**
- Every row sums to `VP8_FILTER_WEIGHT == 128`. Verify row 2 by
  hand: `2 − 11 + 108 + 36 − 8 + 1 = 128`. Row 4: `3 − 16 + 77 + 77 −
  16 + 3 = 128`. This invariant is what `VP8_FILTER_SHIFT = 7` is
  paired with: shifting a sum-of-128-weighted-pixels right by 7
  restores the 8-bit range.
- Taps are signed and small-magnitude **negative side-lobes** appear at
  positions ±1, ±2 from centre. Those negative lobes are what give
  the 6-tap filter its sharpening property — and what makes
  intermediate values able to fall outside `[0, 255]`, hence the
  explicit clamp in `filter_block2d_first_pass` (see below).
- Phase 4 (`{3, -16, 77, 77, -16, 3}`) is centro-symmetric, so the
  half-pel interpolant is unbiased.
- Phases `p` and `8-p` are mirror images of each other in tap order.

**Why row 0 is `{0,0,128,0,0,0}` rather than special-cased.** The C
reference still costs six multiplies for a copy if it lands in this
function, but the dispatcher in `reconinter.c` checks for full-pel MVs
before calling, so the identity row is reached only by oblique paths
(such as one axis integer and the other fractional under the
separable decomposition — the identity *does* run on the unused axis
of a 1-D pass).

---

## The 1-D primitive: `filter_block2d_first_pass`

```c
static void filter_block2d_first_pass(unsigned char *src_ptr, int *output_ptr,
                                      unsigned int src_pixels_per_line,
                                      unsigned int pixel_step,
                                      unsigned int output_height,
                                      unsigned int output_width,
                                      const short *vp8_filter);
```

**What it does.** A single direction-agnostic 6-tap convolution. The
`pixel_step` argument is what makes it direction-agnostic: when called
with `pixel_step == 1` it convolves horizontally (taps reach
`src_ptr[-2]..src_ptr[+3]`); when called with `pixel_step ==
src_pixels_per_line` (not used in `filter_block2d` here but available
to callers) it would convolve vertically. In `filter.c` the horizontal
pass always uses `pixel_step == 1` and the vertical pass is split off
into `filter_block2d_second_pass` because the source for the second
pass is `int*` (signed 32-bit), not `unsigned char*`.

**Why a separable formulation.** The 2-D 6×6 filter would be 36 MACs
per output sample; the separable form is 6 + 6 = 12. The cost of
separability is a per-sample intermediate (the `int` output of the
first pass), and the requirement that the first pass produce *more*
rows than the final output because the vertical pass also needs ±2 / +3
neighbours in the intermediate.

**Inner loop.**

```c
Temp = ((int)src_ptr[-2*pixel_step] * vp8_filter[0])
     + ((int)src_ptr[-1*pixel_step] * vp8_filter[1])
     + ((int)src_ptr[ 0            ] * vp8_filter[2])
     + ((int)src_ptr[ 1*pixel_step] * vp8_filter[3])
     + ((int)src_ptr[ 2*pixel_step] * vp8_filter[4])
     + ((int)src_ptr[ 3*pixel_step] * vp8_filter[5])
     + (VP8_FILTER_WEIGHT >> 1);          /* +64 round-up */
Temp = Temp >> VP8_FILTER_SHIFT;          /* /128 */
if      (Temp <   0) Temp =   0;
else if (Temp > 255) Temp = 255;
output_ptr[j] = Temp;
```

**Why the clamp.** The 6-tap kernel has *negative* side-lobes (−16,
−11, −9, …). For a high-frequency input the convolution can ring
beyond `[0, 255]`. Without the clamp the intermediate could be e.g.
−40 or 295, and the vertical pass would then see those values, then
clamp *again*. The double clamp matches the RFC's pixel-domain
semantics: the intermediate is treated as if it were an 8-bit pixel
even though stored as 32-bit. (The 32-bit storage is purely an
SSE/register-allocation convenience.)

**Invariants and contract.**
- `src_ptr` is the *top-left of the output region*, not of the
  read region. The function reaches back 2 rows / columns and forward
  3 — see how callers pre-decrement by `2 * src_pixels_per_line`
  before invoking (e.g. line 117). The implication is that the YV12
  reference frame *must* have at least 2-pixel border above/left and
  3-pixel border below/right. In libvpx that border is established by
  `vpx_scale/generic/yv12extend.c` (border-extend hooks invoked from
  `decodeframe.c` at end-of-frame).
- `output_ptr` is `int*` — the next pass consumes 32-bit values.
- `output_height` and `output_width` are the *intermediate* dimensions,
  not the final ones (see the call-site analysis below).

## `filter_block2d_second_pass`

```c
static void filter_block2d_second_pass(int *src_ptr, unsigned char *output_ptr,
                                       int output_pitch,
                                       unsigned int src_pixels_per_line,
                                       unsigned int pixel_step,
                                       unsigned int output_height,
                                       unsigned int output_width,
                                       const short *vp8_filter);
```

**What it does.** Structurally identical to `first_pass`, except that
the source is `int*` (the intermediate buffer) and the destination is
`unsigned char*`, the final pixel-domain output. The convolution
formula, rounding, shift, and clamp are byte-for-byte the same; the
only difference is the output cast and an `output_pitch` parameter so
that the final block can be written into a non-contiguous frame
buffer.

**Why two functions instead of one templated.** The C reference
predates extensive templating in this code base; the duplication is
also what makes the two passes independently replaceable by
hand-written assembly without needing to share an intermediate format
contract beyond "32-bit signed".

## The 4x4 driver: `filter_block2d`

```c
static void filter_block2d(unsigned char *src_ptr, unsigned char *output_ptr,
                           unsigned int src_pixels_per_line, int output_pitch,
                           const short *HFilter, const short *VFilter) {
  int FData[9 * 4];   /* Temp data buffer used in filtering */

  /* First filter 1-D horizontally... */
  filter_block2d_first_pass(src_ptr - (2 * src_pixels_per_line), FData,
                            src_pixels_per_line, 1, 9, 4, HFilter);

  /* then filter verticaly... */
  filter_block2d_second_pass(FData + 8, output_ptr, output_pitch, 4, 4, 4, 4,
                             VFilter);
}
```

This is the **archetype** of the size-specific predictors below; tracing
it carefully illuminates them all.

- `int FData[9 * 4]` is the intermediate. Why 9 rows for a 4x4 output?
  The vertical pass at each output row needs taps at relative
  positions `{-2..+3}`, i.e. 6 input rows; for 4 output rows that is
  `4 + 6 - 1 = 9`. Width is 4 because the horizontal pass already
  emitted exactly `output_width = 4`-wide rows.
- The source pre-decrement `src_ptr - 2 * src_pixels_per_line` skips
  *up* two rows so that the horizontal pass starts reading at the row
  that the vertical pass will eventually reach back to.
- `FData + 8` skips the first 2 rows of intermediate (`2 * 4`-wide
  rows = 8 ints) so that the vertical pass treats the third row as
  "centre" of its taps and reaches back into the leading 2 rows for
  the `-2`, `-1` taps.
- The final two `4, 4` arguments to the second pass are
  `output_height, output_width` — the actual 4x4 output rectangle.

Why this skip arithmetic and not "pass the intermediate origin and let
the second pass also decrement"? Because the second pass's
`pixel_step = src_pixels_per_line = 4` already encodes the vertical
stride within `FData`, and giving it a `+8`-shifted source pointer is
equivalent to telling it "your top output row is at intermediate row
2" — which is the row whose convolution support cleanly fits in
`[0..8]`.

## The size-specialised six-tap entry points

There are four exposed C entry points, one per supported block size:

| Function                          | Output | Intermediate size       |
|-----------------------------------|--------|--------------------------|
| `vp8_sixtap_predict4x4_c`         | 4x4    | `FData[9 * 4]`           |
| `vp8_sixtap_predict8x4_c`         | 8x4    | `FData[13 * 16]` (9 used)|
| `vp8_sixtap_predict8x8_c`         | 8x8    | `FData[13 * 16]`         |
| `vp8_sixtap_predict16x16_c`       | 16x16  | `FData[21 * 24]`         |

The 4x4 path delegates to `filter_block2d` (above). The other three
inline the same horizontal-then-vertical pattern with the dimensions
adjusted:

- **8x8**: 13 intermediate rows (`8 + 6 - 1 = 13`), 8 wide; skip
  `2 * 8 = 16` ints into the buffer for the vertical pass.
- **8x4**: 9 intermediate rows, 8 wide; output is `4` high but skip is
  still `16`.
- **16x16**: 21 intermediate rows, 16 wide; skip `2 * 16 = 32`.

Note that `vp8_sixtap_predict8x4_c` allocates the same generous
`FData[13 * 16]` as the 8x8 — it only *uses* 9 rows but reserves space
shaped for the larger sibling so that the same stack frame can be
hoisted by the compiler. The 16x16 path needs `21 * 24` because — even
though only 16 columns are written — the alignment of the intermediate
keeps width = 24 in some legacy callers; here the C reference uses 24
defensively to match the SIMD layout (see comment on the alloc
inside `vp8_sixtap_predict16x16_c`).

**Why these four sizes and no others.** VP8 inter prediction operates
at MB granularity (16x16 luma + two 8x8 chroma) and, under
`SPLIT_MV`, at sub-block granularity of either 16x8/8x16 (handled as
two 16x8 or 8x16 = 8x4 + 4x4 calls) or 4x4. The 8x4 form covers the
chroma half-height case that arises when a 16x8 luma split is mapped
to chroma at 4:2:0.

**Why luma vs chroma get the same C function.** The function only sees
a pointer and a pitch; whether the caller is reconstructing Y or U/V
is invisible. The selection between "use 6-tap" vs "use bilinear"
happens in `reconinter.c` via the per-MACROBLOCKD function pointers,
not here.

---

## The bilinear path

The bilinear functions (`filter_block2d_bil*`) mirror the six-tap ones
in structure but are simpler because:

1. Only 2 taps per pass, so no negative side-lobes and no clamp.
2. The intermediate fits in `unsigned short` (max value of any single
   tap sum is `255 * 128 = 32640 < 65536`).
3. No source pre-decrement: the support is `{+0, +1}` in each axis, so
   `src_ptr` is the top-left of *both* the read region and the output
   region, and no negative offsets exist.

### `filter_block2d_bil_first_pass`

```c
dst_ptr[j] =
    (((int)src_ptr[0] * vp8_filter[0]) +
     ((int)src_ptr[1] * vp8_filter[1]) +
     (VP8_FILTER_WEIGHT / 2)) >> VP8_FILTER_SHIFT;
```

**What.** A 1-D 2-tap convolution emitting `unsigned short`. The
function comment states "Two filter taps should sum to
VP8_FILTER_WEIGHT" — and indeed every row of `vp8_bilinear_filters`
satisfies that. The result is therefore guaranteed in `[0, 255]`
(convex combination of two byte values, plus rounding), so the
post-shift goes straight into a `uint16`. **No clamp.**

**Why `unsigned short` intermediate.** It is enough precision because
the intermediate value is already a (rounded) 8-bit pixel — unlike the
six-tap intermediate, which can have legitimately out-of-range
representations of an in-range result.

### `filter_block2d_bil_second_pass`

```c
Temp = ((int)src_ptr[0]     * vp8_filter[0]) +
       ((int)src_ptr[width] * vp8_filter[1]) +
       (VP8_FILTER_WEIGHT / 2);
dst_ptr[j] = (unsigned int)(Temp >> VP8_FILTER_SHIFT);
```

**Stride encoding.** Note that the vertical neighbour is read at
offset `width`, not `pixel_step`. The bilinear intermediate buffer is
contiguous (`width`-wide rows packed in `FData`), so the vertical
stride is simply `width`. There is no `src_pixels_per_line - width`
back-fill at the end of each row because the loop only advances
`src_ptr++` and explicitly *doesn't* re-anchor at row end — the
intermediate is laid out so that this works out (the increments of
`src_ptr` over `width` columns plus the implicit overlap give the
right next-row anchor without explicit fix-up).

### `filter_block2d_bil` driver

```c
static void filter_block2d_bil(unsigned char *src_ptr, unsigned char *dst_ptr,
                               unsigned int src_pitch, unsigned int dst_pitch,
                               const short *HFilter, const short *VFilter,
                               int Width, int Height) {
  unsigned short FData[17 * 16];   /* Temp data buffer used in filtering */

  filter_block2d_bil_first_pass(src_ptr, FData, src_pitch, Height + 1, Width,
                                HFilter);
  filter_block2d_bil_second_pass(FData, dst_ptr, dst_pitch, Height, Width,
                                 VFilter);
}
```

`Height + 1` rows of intermediate are needed because the vertical
2-tap pass needs one row *below* each output row. The fixed
`unsigned short FData[17 * 16]` handles the maximum case (`Height +
1 = 17`, `Width = 16`); smaller block sizes simply leave the tail of
the buffer untouched. The driver is parameterised on `Width` and
`Height` directly, so unlike the six-tap path, the four entry points
below collapse to one body with size constants:

```c
vp8_bilinear_predict4x4_c   →  filter_block2d_bil(..., 4, 4)
vp8_bilinear_predict8x4_c   →  filter_block2d_bil(..., 8, 4)
vp8_bilinear_predict8x8_c   →  filter_block2d_bil(..., 8, 8)
vp8_bilinear_predict16x16_c →  filter_block2d_bil(..., 16, 16)
```

Each wrapper picks `HFilter = vp8_bilinear_filters[xoffset]`,
`VFilter = vp8_bilinear_filters[yoffset]`, and asserts
`(xoffset | yoffset) != 0` — the bilinear path is never legally called
with both fractional offsets zero, because that would be an integer
copy that the SIMD specialisations are explicitly excused from
handling. (The C reference would still do the right thing because row
0 of the table is the identity, but the assertion documents the
contract for parity with the SIMD implementations.)

---

## Putting it together — the read region and border contract

For a 16x16 luma block with both fractional phases non-zero, the
six-tap path reads:

- Vertically: 2 rows above the output, 3 below ⇒ 21 source rows.
- Horizontally: 2 cols left, 3 right ⇒ 19 source cols.

This is the largest contiguous read region in the decoder hot path,
and it is the reason the YV12 frame buffer carries a border (32 bytes
on each side in libvpx's default configuration — see
`vpx_scale/generic/yv12config.c`). When an MV places the support
region across the picture edge, the border-extension pass over the
*reference* frame (done once, at end-of-frame, by
`vp8_yv12_extend_frame_borders` triggered from `decodeframe.c`'s frame
finalisation) has already replicated the edge pixels far enough out
that no bounds check is needed inside `filter_block2d_first_pass`.
This is the invariant referenced obliquely in §8 of the technical
overview when it mentions "Clamp the MV (using `vp8_clamp_mv2`)" —
that clamp ensures the read region never exceeds what the border can
provide.

The bilinear path's read region is `(Width + 1) × (Height + 1)` and
needs only a 1-pixel border on the right/bottom. Since the YV12
allocator's border easily accommodates this, the bilinear path imposes
no additional constraint.

---

## RTCD registration (`vp8_rtcd.h`)

Each `_c` function in this file appears in
`vp8/common/rtcd_defs.pl` and is published through
`vp8_rtcd.h` (generated). For example:

```perl
add_proto qw/void vp8_sixtap_predict16x16/,
   "unsigned char *src_ptr, int src_pixels_per_line,
    int xoffset, int yoffset, unsigned char *dst_ptr, int dst_pitch";
specialize qw/vp8_sixtap_predict16x16 neon dspr2 msa mmi lsx/,
           "$sse2_asm", "$ssse3_asm";
```

In a `generic-gnu` build all `specialize` slots collapse to the C
reference, so the symbol `vp8_sixtap_predict16x16` is `#define`-d to
`vp8_sixtap_predict16x16_c` in the generated header. On an x86 build
the RTCD init in `vp8/common/rtcd.c` replaces the pointer with the
SSE2 or SSSE3 specialisation at startup, and `filter.c`'s functions
become reference-only fallbacks used by the unit tests. The tables
`vp8_sub_pel_filters` and `vp8_bilinear_filters`, by contrast, are
used by *every* implementation — the SSE2/SSSE3/NEON kernels broadcast
columns of these tables into vector registers (see e.g.
`x86/bilinear_filter_sse2.c:40–41` and `x86/subpixel_ssse3.asm:1506`,
which defines its own re-laid-out `vp8_bilinear_filters_ssse3`).

---

## Cross-references

- `vp8/common/blockd.h:205, 284–287` — the `vp8_subpix_fn_t` typedef
  and the four function-pointer slots on `MACROBLOCKD`.
- `vp8/common/reconinter.c:60, 114, 297` — call sites for inter
  prediction; `mv.col & 7` / `mv.row & 7` derive `xoffset` / `yoffset`.
- `vp8/common/findnearmv.h:34` — `vp8_clamp_mv2`, the MV clamp that
  makes the read-region contract safe.
- `vpx_scale/generic/yv12extend.c` — establishes the per-frame border
  that the six-tap reach (−2 / +3) is allowed to step into.
- RFC 6386 §6.5.1 (6-tap), §6.5.2 (bilinear) — bitstream-level
  definition of the filter coefficients reproduced in this file.
- `vp8_technical_overview.md` §8.4 (Luma reconstruction), §16.4
  (version → filter selection table).
