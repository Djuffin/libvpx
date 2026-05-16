# `vp8/common/extend.c` — Frame-border extension

## Role in the decoder

VP8 is a motion-compensated codec. When a macroblock in frame *N* is
predicted from a region of a reference frame, the bitstream is free to
supply a motion vector that points outside the visible picture, and the
sub-pel interpolation filter is then free to read several more pixels
*beyond* that out-of-picture region. Without preparation, both of these
would step off the end of the reference frame's pixel array and read
either garbage or memory that does not belong to the codec.

libvpx solves this the standard way: every frame buffer is allocated
with an opaque "border" of extra pixels on all four sides, and after a
frame is reconstructed those border pixels are *filled with edge
replication* — every pixel in the top border is a copy of the top row,
every pixel in the left border is a copy of the leftmost column, and so
on, with the corners filled by replication of the corner sample. The
decoder can then perform sub-pel interpolation on any motion vector
whose *taps* stay within `[-border, picture+border)` without a single
boundary check in the hot path. The border is the codec's way of paying
once at frame-output time so that the inter-prediction inner loop in
`reconinter.c` / `filter.c` can be branch-free.

`vp8/common/extend.c` is the VP8-specific implementation of that
"replicate the edge into the border" operation. It exposes three entry
points (see `vp8/common/extend.h`):

```c
void vp8_copy_and_extend_frame          (YV12_BUFFER_CONFIG *src, YV12_BUFFER_CONFIG *dst);
void vp8_copy_and_extend_frame_with_rect(YV12_BUFFER_CONFIG *src, YV12_BUFFER_CONFIG *dst,
                                         int srcy, int srcx, int srch, int srcw);
void vp8_extend_mb_row                  (YV12_BUFFER_CONFIG *ybf,
                                         unsigned char *YPtr,
                                         unsigned char *UPtr,
                                         unsigned char *VPtr);
```

The first two are encoder-side conveniences (they copy a raw input
picture into an internal `YV12_BUFFER_CONFIG` *and* extend its border in
one pass; they are called from `vp8/encoder/lookahead.c` and similar).
The third one is the only entry point reached by the *decoder*: it is
called from `decode_mb_rows()` in `vp8/decoder/decodeframe.c:599` and
from the matching threaded path in `vp8/decoder/threading.c:565`, once
per macroblock row of the current frame, to perform a *very small*
local-only border extension on the right edge of the destination
buffer. That micro-extension exists so that intra prediction for the
*next* column of macroblocks — which reads "the rightmost decoded
samples" as its `LEFT` predictor — can read 4 pixels even when those 4
pixels would normally lie beyond the current decode front. (Full
border extension at end-of-frame in the decoder is performed by
`vp8_yv12_extend_frame_borders_c()` in `vpx_scale/generic/yv12extend.c`,
not by this file. The static helper here, `copy_and_extend_plane`, is
the VP8 encoder's twin of that routine.)

### The geometry

Every `YV12_BUFFER_CONFIG` (defined in `vpx_scale/yv12config.h:29`)
carries a `border` field that records the number of *extra* luma pixels
around each side of the picture. For VP8 the value is fixed:

```c
/* vpx_scale/yv12config.h:23 */
#define VP8BORDERINPIXELS 32
```

All decoder frame buffers (LAST, GOLDEN, ALTREF, the in-progress
reconstruction, and the post-processing scratch) are allocated with
`border = VP8BORDERINPIXELS = 32` — see `vp8/common/alloccommon.c:71`,
`alloccommon.c:87`, `alloccommon.c:112`. 32 is a deliberate over-spec:
the worst-case sub-pel filter in VP8 is the 6-tap luma filter
`vp8_sub_pel_filters[8][6]` (`vp8/common/filter.c:20`), which at the
extreme of a 16×16 prediction block reads up to ~21 pixels beyond a MV
that already points slightly outside the picture. A 16-pixel border
would be tight; 32 leaves comfortable slack — enough that even the
encoder's wide motion search (which uses `mv_col_min = -(... +
(VP8BORDERINPIXELS - 16))`, see `vp8/encoder/encodeframe.c:374`) can
roam without overrun.

For the chroma planes, which are sub-sampled 4:2:0, the border is
implicitly half as wide: every place this file divides `dst->border` by
two with `>> 1` is consistent with `uv_width = y_width / 2`. There is
no separate `uv_border` field — the convention is "luma border / 2".

```
              ┌─────────────────────────────────────────────┐
              │            top border (et rows)             │
              ├──────┬──────────────────────────────┬───────┤
              │      │                              │       │
              │ left │      visible picture         │ right │
              │ (el) │     w × h samples            │ (er)  │
              │      │                              │       │
              ├──────┴──────────────────────────────┴───────┤
              │           bottom border (eb rows)           │
              └─────────────────────────────────────────────┘
```

The `dst->border` member tells the allocator how many extra pixels were
reserved. `et` (extend-top) and `el` (extend-left) always equal
`dst->border`. The `eb` (extend-bottom) and `er` (extend-right) values
are subtler: when the displayed picture is smaller than the macroblock
grid (e.g. 854×480 inside a 864×480 buffer), the routine has to extend
all the way to the *allocated* buffer edge, so it uses
`dst->border + dst->y_height - src->y_height` and the matching width
formula. That keeps the border *outside the allocated buffer's right
edge* properly populated even when the displayed picture leaves a few
unused columns inside the buffer.

---

## Code, in narrative order

### `copy_and_extend_plane` — the per-plane work-horse

This file-static function is the engine that all three public entry
points sit on top of. Its job is: given a source plane (luma or
chroma), copy every sample into a destination plane, then replicate
the picture's edges outward into the four configured border widths.

```c
/* vp8/common/extend.c:14 */
static void copy_and_extend_plane(
    unsigned char *s,  int sp,
    unsigned char *d,  int dp,
    int h, int w,
    int et, int el, int eb, int er,
    int interleave_step);
```

**What.** Reads from `s` (pitch `sp`), writes to `d` (pitch `dp`),
copying an `h`-row by `w`-column rectangle. After the copy, it extends
`et` rows above, `eb` below, `el` columns left and `er` columns right
by edge replication. The pointer arithmetic assumes that the caller's
`d` is the top-left of the *visible* destination region — i.e. that the
border lies in the address ranges `[d - el, d)` (left),
`[d + w, d + w + er)` (right), `[d + dp*(-et), d)` (top) — which is
also the convention used everywhere else in libvpx (`y_buffer` and
friends in `YV12_BUFFER_CONFIG` always point at the picture, not at the
allocation).

**Why.** Sub-pel interpolation in `reconinter.c` will eventually read
`src_ptr[xoffset_in_taps][yoffset_in_taps]` for taps that overhang the
picture. If those overhanging samples are not edge-replicated, the
prediction is wrong (and can crash if it reaches unmapped memory). Edge
replication is the standard cure: the value of any pixel "outside" the
picture is defined to be the value of the nearest pixel "inside".

**Invariants.**
* `interleave_step >= 1` (the function defensively normalises a 0 or
  negative argument to 1; this matters because NV12 chroma is
  interleaved, see below).
* `el + er + w` is the destination-line length used for the top/bottom
  fill — i.e. the top/bottom rows are filled by `memcpy`ing the already
  edge-extended first/last *visible row including its already-filled
  left/right border*. That choice means the order of operations
  matters: left/right must run first, then top/bottom can copy
  whole-line, which is what the code does.
* No bounds check on the destination: the caller must have allocated at
  least `(et + h + eb) * dp + el + w + er` bytes around the picture.
  This is guaranteed when `d` came from a `YV12_BUFFER_CONFIG`
  allocated by `vp8_yv12_alloc_frame_buffer` with the right `border`.

**How it works.** Pass one handles the left and right strips, row by
row. For each of the `h` source rows, the leftmost sample
(`src_ptr1[0]`) is `memset` into the `el`-wide left border; the *row
itself* is copied (with `memcpy` in the dense case, or with a stride-
`interleave_step` loop for NV12 chroma); the rightmost sample
(`src_ptr2[0]`, with `src_ptr2 = s + (w - 1) * interleave_step`) is
`memset` into the `er`-wide right border. After this pass, the
"visible-rows + left/right borders" region of `d` is fully populated,
forming `h` rows each of width `el + w + er`.

```c
/* vp8/common/extend.c:33 */
src_ptr1 = s;
src_ptr2 = s + (w - 1) * interleave_step;
dest_ptr1 = d - el;
dest_ptr2 = d + w;
for (i = 0; i < h; ++i) {
  memset(dest_ptr1, src_ptr1[0], el);
  if (interleave_step == 1) {
    memcpy(dest_ptr1 + el, src_ptr1, w);
  } else {
    for (j = 0; j < w; j++) dest_ptr1[el + j] = src_ptr1[interleave_step * j];
  }
  memset(dest_ptr2, src_ptr2[0], er);
  src_ptr1 += sp;  src_ptr2 += sp;
  dest_ptr1 += dp; dest_ptr2 += dp;
}
```

Pass two handles top and bottom. It re-aims its source pointers at the
*already-extended* first and last rows of `d` (so the corners come out
right "for free"), and then `memcpy`s a `linesize = el + er + w` block
into each of the `et` rows above and `eb` rows below.

```c
/* vp8/common/extend.c:58 */
src_ptr1  = d - el;
src_ptr2  = d + dp * (h - 1) - el;
dest_ptr1 = d + dp * (-et) - el;
dest_ptr2 = d + dp * (h)   - el;
linesize  = el + er + w;
for (i = 0; i < et; ++i) { memcpy(dest_ptr1, src_ptr1, linesize); dest_ptr1 += dp; }
for (i = 0; i < eb; ++i) { memcpy(dest_ptr2, src_ptr2, linesize); dest_ptr2 += dp; }
```

The corner regions are a direct consequence of these two passes
composing: the *first* pass writes the four corner left/right strips of
the top and bottom row's `memset` of the single corner pixel value, and
the *second* pass then replicates those corner strips upward and
downward. The net effect is that every corner cell holds the value of
the corresponding picture-corner sample — exactly what edge replication
demands.

**The `interleave_step` quirk.** In ordinary I420 / YV12 the U and V
planes are stored separately, contiguous in memory, and `memcpy` works.
But libvpx also accepts an NV12-shaped input (chroma `UV` interleaved
in a single plane), and the heuristic `chroma_step = src->v_buffer -
src->u_buffer == 1 ? 2 : 1` in `vp8_copy_and_extend_frame` detects that
case: if the V pointer is exactly one byte after the U pointer, the two
chroma "planes" must be interleaved, so reading every other byte is
required. The slow per-byte loop is only used in that NV12 path; the
common case takes the `memcpy` branch.

---

### `vp8_copy_and_extend_frame` — full-picture copy-with-extend

```c
/* vp8/common/extend.c:75 */
void vp8_copy_and_extend_frame(YV12_BUFFER_CONFIG *src,
                               YV12_BUFFER_CONFIG *dst);
```

**What.** Copies the entire `src` picture into `dst` and fills `dst`'s
border. Y first, then U, then V — each through one call to
`copy_and_extend_plane`.

**Why.** This is the *encoder* path: it is invoked from
`vp8/encoder/lookahead.c:139` to ingest a raw input frame into the
lookahead queue's internal buffer, and from `vp8/encoder/onyx_if.c`
when ref slots need a fresh extended copy. The decoder does not call
this; the decoder uses `vp8_yv12_extend_frame_borders_c()` instead,
which extends *in place* (no copy) because the reconstruction has
already been written to the destination during macroblock decoding.

**Invariants.**
* `src` and `dst` must agree on the *visible* picture dimensions up to
  the alignment slack: `dst->y_height >= src->y_height`,
  `dst->y_width  >= src->y_width`.
* `dst->border` must be the as-allocated border (`VP8BORDERINPIXELS =
  32` in libvpx's allocator).
* The chroma border is implicitly `dst->border >> 1`.

**How it works.** It computes the four extension widths and calls the
plane helper three times. For luma:

```c
int et = dst->border;
int el = dst->border;
int eb = dst->border + dst->y_height - src->y_height;
int er = dst->border + dst->y_width  - src->y_width;
```

The `eb`/`er` formulas are the geometry note from the previous
section: extend not just by `border`, but by `border + slack`, so that
the *allocated* buffer's right/bottom borders are populated even when
the visible picture has been slightly cropped relative to the
allocation. For chroma the same formulas appear but with everything
shifted right by one (chroma is 2× sub-sampled).

The NV12 detection (`chroma_step`) is passed as the `interleave_step`
argument only for the chroma calls; luma always uses step 1.

---

### `vp8_copy_and_extend_frame_with_rect` — partial-frame variant

```c
/* vp8/common/extend.c:103 */
void vp8_copy_and_extend_frame_with_rect(YV12_BUFFER_CONFIG *src,
                                         YV12_BUFFER_CONFIG *dst,
                                         int srcy, int srcx,
                                         int srch, int srcw);
```

**What.** Copies a rectangular sub-region `(srcx, srcy, srcw, srch)` of
`src` into the matching location in `dst`, and *only* extends the
borders on the sides of `dst` that the sub-rectangle actually touches.

**Why.** The encoder's lookahead queue (`vp8/encoder/lookahead.c:129`)
uses this when an application supplies sliced-into-strips inputs: the
strip in the interior of the picture must not have its top/bottom
border re-filled (those samples belong to the previous/next strip and
are written by other calls), but the *outer* border on the picture-edge
side of the strip must be filled. The four conditional zero-outs are
exactly that logic:

```c
/* vp8/common/extend.c:117 */
if (srcy) et = 0;                              /* not the top strip */
if (srcx) el = 0;                              /* not the left strip */
if (srcy + srch != src->y_height) eb = 0;      /* not the bottom strip */
if (srcx + srcw != src->y_width)  er = 0;      /* not the right strip */
```

**Invariants.**
* The sub-rectangle must lie inside `src` and inside `dst`'s allocated
  area at the same coordinates.
* `srcx`, `srcy`, `srcw`, `srch` are *luma* coordinates. The chroma
  call below divides them by two with `(x + 1) >> 1`, the standard
  rounding for 4:2:0 sub-sampling: it errs on the side of "include the
  edge chroma sample" when the rectangle has odd alignment.
* The chroma `et`/`el`/`eb`/`er` are halved with `(x + 1) >> 1`, again
  rounding *up* so that a zero stays zero but a non-zero stays
  non-zero — preserving the "do extend / don't extend" decision made by
  the conditionals above.

**How it works.** The starting source and destination offsets are
computed in bytes:

```c
int src_y_offset  = srcy * src->y_stride + srcx;
int dst_y_offset  = srcy * dst->y_stride + srcx;
int src_uv_offset = ((srcy * src->uv_stride) >> 1) + (srcx >> 1);
int dst_uv_offset = ((srcy * dst->uv_stride) >> 1) + (srcx >> 1);
```

Then `copy_and_extend_plane` is called once per plane with the
adjusted offsets and the conditionally-zeroed extension widths.

The decoder never invokes this routine — it is encoder-side only.

---

### `vp8_extend_mb_row` — the in-loop intra-prediction patch

```c
/* vp8/common/extend.c:144 */
void vp8_extend_mb_row(YV12_BUFFER_CONFIG *ybf,
                       unsigned char *YPtr,
                       unsigned char *UPtr,
                       unsigned char *VPtr);
```

This is the *only* function from `extend.c` that the minimal decoder
build ever calls.

**What.** Replicates a single column of edge samples into the right
side of the most-recently-decoded macroblock row, in just enough places
that the next pass can read four pixels to the right of the current
column without falling off the edge of the buffer.

**Why.** The caller (`vp8_decode_frame()`, `decodeframe.c:599`) invokes
this *at the end of every macroblock row*, immediately after the last
macroblock in that row has been reconstructed. The pointer arguments
are addresses just past the last column of the just-decoded row:
`xd->dst.y_buffer + 16`, `xd->dst.u_buffer + 8`, `xd->dst.v_buffer +
8`. The function fills 4 luma pixels and 4 chroma pixels (in two
adjacent rows of each plane) with the rightmost decoded value. That
patch is what intra prediction needs when a macroblock on the next row
reads its `TOP_RIGHT` neighbour — without it, that read would sample
random buffer memory beyond the picture edge.

The comment in the source is terse but exact:

```c
/* vp8/common/extend.c:143 */
/* note the extension is only for the last row, for intra prediction purpose */
```

**Invariants.**
* `YPtr` must point to the address one luma macroblock to the right of
  the rightmost decoded macroblock in the current row (offset `+16`).
* `UPtr`, `VPtr` must point to the address one chroma macroblock to the
  right (offset `+8`, since chroma macroblocks are 8 wide).
* The constants 14, 6, 6 inside the function are the *vertical* offsets
  to the last-but-one row of the macroblock — i.e. lines 14 and 15 of
  luma, lines 6 and 7 of chroma — measured in `y_stride` /
  `uv_stride` units. Those are precisely the rows that the upcoming
  next-row intra prediction will use as `TOP` / `TOP_RIGHT`.

**How it works.**

```c
YPtr += ybf->y_stride * 14;
UPtr += ybf->uv_stride * 6;
VPtr += ybf->uv_stride * 6;

for (i = 0; i < 4; ++i) {
  YPtr[i] = YPtr[-1];        /* replicate the last decoded column */
  UPtr[i] = UPtr[-1];
  VPtr[i] = VPtr[-1];
}

YPtr += ybf->y_stride;       /* next row */
UPtr += ybf->uv_stride;
VPtr += ybf->uv_stride;

for (i = 0; i < 4; ++i) {
  YPtr[i] = YPtr[-1];
  UPtr[i] = UPtr[-1];
  VPtr[i] = VPtr[-1];
}
```

Two rows, four pixels each, three planes — that is the entire patch.
It is doing manually what `copy_and_extend_plane` does row-by-row, but
on a single MB-edge column instead of the full picture; the macroblock
loop in `decode_mb_rows` runs raster-order, so this two-row,
four-column patch is exactly what's needed at the end of each MB row
to make the *next* MB row's intra-predictors safe.

Full-picture border extension at end-of-frame in the decoder is
performed elsewhere — `vp8_yv12_extend_frame_borders_c()` in
`vpx_scale/generic/yv12extend.c` — once the entire frame has been
reconstructed, so that the *next* frame's inter-prediction can read
samples beyond the picture edge. The split is intentional:

* This file's `vp8_extend_mb_row` is the small, surgical, *intra-frame*
  extension done during decode.
* `yv12extend.c`'s routine is the large, full-perimeter, *inter-frame*
  extension done after decode (and is structurally identical to the
  static `copy_and_extend_plane` helper here — they are siblings, one
  copy-and-extend, one extend-in-place).

---

## How this interacts with reconinter and the sub-pel taps

VP8's luma sub-pel filter is six taps:

```c
/* vp8/common/filter.h:25 */
extern DECLARE_ALIGNED(16, const short, vp8_sub_pel_filters[8][6]);
```

For a 16-pixel-wide prediction block, the worst case is: motion vector
points one pixel inside the picture's left edge, fractional component
selects a sub-pel position whose filter footprint reaches 2 taps to the
left and 3 to the right of the integer position. The interpolator
therefore needs samples at offsets `[-2, -1, 0, 1, 2, 3]` from the
integer source — i.e. up to 2 pixels to the left of the (already
out-of-picture) MV target. Add the bilinear chroma extension (2 taps)
and the additional MV "out-of-picture by N" allowance that the
bitstream legally permits in VP8 (clamped during MV parse, but
generously), and one arrives at the empirical "32 is comfortable" number
that `VP8BORDERINPIXELS` codifies.

Because the border is *populated by edge replication* before any
inter-prediction reads it, the predictor in `vp8/common/reconinter.c`
(via `vp8_sixtap_predict16x16` and friends in `filter.c`) can issue
straight pointer-arithmetic reads with no boundary tests at all. That
is the whole reason this file exists: it converts a memory-safety
problem into a one-time `memset`/`memcpy` per frame, so the
inter-prediction inner loop stays branch-free.

The intra-prediction case (handled by `vp8_extend_mb_row`) is dual:
intra predictors in `reconintra.c` / `reconintra4x4.c` read up to 4
pixels above and to the right of the current macroblock. Inside the
picture those pixels come from previously-decoded macroblocks; at the
right edge of the picture they would come from off-buffer memory, so
the patch in this file replicates the last decoded column into them
just in time.
