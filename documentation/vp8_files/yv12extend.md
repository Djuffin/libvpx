# yv12extend.c

`vpx_scale/generic/yv12extend.c` is the YV12-aware border-extension and
whole-frame-copy module. It contains the C reference implementations of
three RTCD entry points that operate on the libvpx-wide
`YV12_BUFFER_CONFIG` struct rather than on raw `uint8_t *` planes:

```
vp8_yv12_extend_frame_borders_c    (line 105)
vp8_yv12_copy_frame_c              (line 193)
vpx_yv12_copy_y_c                  (line 311)
```

plus, under `CONFIG_VP9`, two extra extenders
(`vpx_extend_frame_borders_c`, `vpx_extend_frame_inner_borders_c`) and a
VP9-flavoured copy. In a pure VP8 build those VP9 symbols are still
compiled (the file is unconditional) but never called, and the
`CONFIG_VP9_HIGHBITDEPTH` paths are dead.

## Role in the decoder

This file runs once per decoded frame, at the very end of the
reconstruction loop, after the entropy-decoded residuals have been
added to the predictor and the loop filter has finished. Its job is to
take the freshly written reconstructed buffer and replicate its
outermost row and column outward into the 32-pixel `border` region that
surrounds every YV12 plane, so that the next frame's inter-prediction
sub-pel filters can sample past the picture edge without bounds checks.

The VP8 technical overview describes the rule like this
(`documentation/vp8_technical_overview.md`, §10.3):

> After every decoded frame, the reconstructed buffer's 32-pixel border
> is replicated outward by `vp8_yv12_extend_frame_borders`
> (yv12extend.c:105) … This is what allows inter prediction's 6-tap
> filters to read any MV within the clamped range without per-pixel
> boundary checks — the borders are guaranteed valid.

The companion file `vp8/common/extend.c` solves the same problem at a
lower level — it works on individual `uint8_t *` plane pointers and is
called from MB-level setup (e.g. cropped-frame padding). `yv12extend.c`
sits one level up: it knows about the `YV12_BUFFER_CONFIG` container
and dispatches the per-plane work, including the bookkeeping that comes
from chroma subsampling (the U/V planes are half-resolution in both
dimensions, so their border is half the luma border).

The other two responsibilities of this file — full-frame copy
(`vp8_yv12_copy_frame_c`) and luma-only copy (`vpx_yv12_copy_y_c`) —
are used by reference-frame management paths that need a real pixel
copy rather than a refcount bump (see §10.2 of the overview). The copy
helper always finishes by re-extending the destination's borders, so a
caller never has to think about that step separately.

The single header that defines the container, `vpx_scale/yv12config.h`,
gives the relevant constants:

```c
#define VP8BORDERINPIXELS 32       /* yv12config.h:23 */
```

and the YV12 struct itself (`yv12config.h:29`) carries both visible
(`*_crop_width/height`) and padded (`y_width/y_height`,
`uv_width/uv_height`) plane sizes, the strides, the three plane base
pointers, and the `border` field that this file consults.

## File anatomy

```c
#include <assert.h>
#include "./vpx_config.h"
#include "./vpx_scale_rtcd.h"
#include "vpx/vpx_integer.h"
#include "vpx_mem/vpx_mem.h"
#include "vpx_ports/mem.h"
#include "vpx_scale/yv12config.h"
#if CONFIG_VP9_HIGHBITDEPTH
#include "vp9/common/vp9_common.h"
#endif
```

The dependency surface is small and quite revealing: only
`yv12config.h` for the buffer layout, `vpx_mem` (for `vpx_memset16`,
used only on the high-bit-depth path), `vpx_scale_rtcd.h` for the
function-pointer table this file populates, and `vpx_ports/mem.h` for
the `CONVERT_TO_SHORTPTR` macro that bridges packed `uint16_t` planes
masquerading as `uint8_t *`. There is no codec-state header: the file
is a pure container utility.

The remainder of the file is one static low-level worker, optionally
its high-bit-depth twin, then the VP8 and VP9 entry points layered on
top.

### `extend_plane` — the workhorse (lines 22–60)

```c
static void extend_plane(uint8_t *const src, int src_stride, int width,
                         int height, int extend_top, int extend_left,
                         int extend_bottom, int extend_right) {
```

**What.** Replicates the four edges of one rectangular `width × height`
sub-region of a plane outward by the four `extend_*` amounts, writing
into the surrounding border bytes of the same backing allocation. The
caller passes `src` already pointing at pixel (0,0) of the *visible*
plane — that is, the post-offset address `buffer_alloc + border*stride
+ border` (see overview §10.1). The destination border is therefore
addressed by *negative* offsets relative to `src`.

**Why.** This is the geometric primitive that makes the rest of the
file trivial. Splitting the four sides into a single sweep keeps the
logic in one place; the VP8 and VP9 entry points only differ in *how
much* to extend each side and *which* plane to pass in. Implementing
the work in cropped-aware coordinates (rather than padded) is what
lets the function handle frames whose visible width/height is not a
multiple of 16 in a single uniform pass.

**Invariants.**

- `src` points to the visible top-left pixel; pre-existing border
  memory of size at least `extend_left`/`extend_right` (per side) is
  reachable at negative / right-of-end addresses.
- `src_stride ≥ extend_left + width + extend_right`, so the linewise
  copies cannot overshoot.
- `extend_top` and `extend_bottom` may legitimately differ when the
  visible (cropped) height is less than the padded height — the
  callers add `padded - cropped` to the bottom extension to fill that
  difference too. Likewise for `extend_right`. This is how the file
  handles padded-but-uncropped slack uniformly with true border
  extension.

**How it works.** Two phases:

```c
/* copy the left and right most columns out */
uint8_t *src_ptr1 = src;
uint8_t *src_ptr2 = src + width - 1;
uint8_t *dst_ptr1 = src - extend_left;
uint8_t *dst_ptr2 = src + width;

for (i = 0; i < height; ++i) {
  memset(dst_ptr1, src_ptr1[0], extend_left);
  memset(dst_ptr2, src_ptr2[0], extend_right);
  src_ptr1 += src_stride;
  src_ptr2 += src_stride;
  dst_ptr1 += src_stride;
  dst_ptr2 += src_stride;
}
```

The first phase walks every visible row and fills the left and right
border strips with the value of the row's first and last visible
pixel respectively. `memset` is the right primitive precisely because
"replicate-one-byte-N-times" is exactly what edge clamping means.

```c
/* Now copy the top and bottom lines into each line of the respective
 * borders
 */
src_ptr1 = src - extend_left;
src_ptr2 = src + src_stride * (height - 1) - extend_left;
dst_ptr1 = src + src_stride * -extend_top - extend_left;
dst_ptr2 = src + src_stride * height - extend_left;

for (i = 0; i < extend_top; ++i) {
  memcpy(dst_ptr1, src_ptr1, linesize);
  dst_ptr1 += src_stride;
}

for (i = 0; i < extend_bottom; ++i) {
  memcpy(dst_ptr2, src_ptr2, linesize);
  dst_ptr2 += src_stride;
}
```

The second phase walks the top and bottom border rows and copies the
adjacent visible *scanline* into each. The crucial detail is the
ordering: the column fill runs *first*, so by the time the row-copy
starts, both `src_ptr1` (the topmost visible row) and `src_ptr2` (the
bottommost) already contain the correct left/right border values for
their own row. Copying `linesize = extend_left + width + extend_right`
bytes from those rows therefore propagates the corner regions
correctly — the top-left, top-right, bottom-left, bottom-right
"corner squares" all end up filled with the corresponding extreme
pixel value, which is exactly what nearest-edge clamping prescribes.
If the two phases ran in the opposite order the corner squares would
contain whatever uninitialised garbage the column-fill phase had not
yet overwritten.

**How used.** Called three times per frame from
`vp8_yv12_extend_frame_borders_c` (Y, U, V), and three times per frame
from VP9's `extend_frame` (the same three planes, with a
high-bit-depth variant routed to `extend_plane_high`).

### `extend_plane_high` (lines 62–103, CONFIG_VP9_HIGHBITDEPTH only)

The 10/12-bit twin of `extend_plane`. Same algorithm, two
modifications:

- The plane pointer `src8` arrives as a `uint8_t *` for ABI uniformity
  but actually points at packed `uint16_t` samples; `CONVERT_TO_SHORTPTR`
  reinterprets it.
- The column-fill is `vpx_memset16` (a 16-bit-wide `memset` from
  `vpx_mem`) instead of plain `memset`; the row-copy is still
  `memcpy`, scaled by `sizeof(uint16_t)`.

In a VP8-only build this function is `#ifdef`'d out completely. It is
documented here for completeness because the surrounding control flow
in `extend_frame` references it.

### `vp8_yv12_extend_frame_borders_c` (lines 105–128)

```c
void vp8_yv12_extend_frame_borders_c(YV12_BUFFER_CONFIG *ybf) {
  const int uv_border = ybf->border / 2;

  assert(ybf->border % 2 == 0);
  assert(ybf->y_height - ybf->y_crop_height < 16);
  assert(ybf->y_width  - ybf->y_crop_width  < 16);
  assert(ybf->y_height - ybf->y_crop_height >= 0);
  assert(ybf->y_width  - ybf->y_crop_width  >= 0);
```

**What.** The VP8-side public entry point. Extends Y, U and V of one
`YV12_BUFFER_CONFIG` outward by `border` and `border/2` pixels
respectively. Registered through the RTCD machinery so callers invoke
it as `vp8_yv12_extend_frame_borders(...)` and the dispatch resolves
to the C function on `generic-gnu` (and to SIMD versions on
arch-enabled builds).

**Why halve the border for U/V.** VP8 is 4:2:0: chroma planes have
half the resolution of luma in both dimensions. To preserve the same
*spatial* reach in pixel units the chroma border must be exactly half
the luma border. With `VP8BORDERINPIXELS = 32`, that gives a 16-pixel
chroma border, and at the chroma sample rate that 16-pixel border
covers the same 32-luma-pixel extent that the luma border does. The
chroma sub-pel filter (and the luma 6-tap filter) thus require exactly
the same amount of off-edge data, expressed in their own sample grid.
This is why every call passes `uv_border = ybf->border / 2`, and why
the very first assertion insists `ybf->border` be even — odd borders
would lose a chroma pixel and the assumption breaks down.

**Invariants verified by the asserts.**

- `ybf->border % 2 == 0` — required so the chroma half-border is exact.
- `0 ≤ y_height - y_crop_height < 16` and same for width — VP8 pads
  the reconstructed frame to a multiple of 16 (MB-aligned), but the
  visible (`crop`) dimensions are the encoded `Width`/`Height` from
  the uncompressed header. The padding is therefore in `[0, 15]`. The
  asserts catch a malformed buffer where the padded size is smaller
  than the visible size (negative) or larger than the next MB boundary
  (≥ 16).

**How used.** Called by the VP8 decoder once per frame, after
reconstruction and loop filtering finish, from
`vp8_loopfilter_frame`'s caller chain in the per-frame driver. It is
also called by `vp8_yv12_copy_frame_c` immediately after a full-frame
copy so the destination's borders are valid.

**The three extension calls.** All three follow the same idiom:

```c
extend_plane(ybf->y_buffer, ybf->y_stride, ybf->y_crop_width,
             ybf->y_crop_height, ybf->border, ybf->border,
             ybf->border + ybf->y_height - ybf->y_crop_height,
             ybf->border + ybf->y_width  - ybf->y_crop_width);
```

The first four arguments describe the visible plane: the (cropped)
width and height at the cropped origin. The last four are the four
side extensions. Note the asymmetry: `extend_top` and `extend_left`
are exactly `border`, while `extend_bottom` and `extend_right` are
`border + padding_slack`. This rolls two distinct jobs into one pass:
(1) fill the real `border`-pixel collar that motion compensation will
read into, and (2) clean up the right/bottom *intra-frame* padding
strip between the cropped picture and the MB-aligned padded picture.
Without the padded-slack term those rows/columns of the padded
allocation would hold whatever was left over from a previous frame —
not a correctness issue for the motion-compensation read (which only
ever sees the post-extension data) but it would defeat the loop's
purpose of presenting a clean "infinite extrapolation" view of the
visible image.

The U and V calls are identical except for using `uv_border`,
`uv_stride`, `uv_crop_*`, and `uv_*` (padded) — exactly halved both
in border and dimensions.

### `extend_frame` and `vpx_extend_frame_borders_c` (lines 131–178, VP9 only)

```c
static void extend_frame(YV12_BUFFER_CONFIG *const ybf, int ext_size) {
  const int c_w  = ybf->uv_crop_width;
  const int c_h  = ybf->uv_crop_height;
  const int ss_x = ybf->uv_width < ybf->y_width;
  const int ss_y = ybf->uv_height < ybf->y_height;
  const int c_et = ext_size >> ss_y;
  const int c_el = ext_size >> ss_x;
  ...
}
```

**What.** A more general extender used by VP9 (which supports 4:2:0,
4:2:2, 4:4:0 and 4:4:4 chroma layouts and optional high bit depth).
Instead of unconditionally halving the chroma border, it derives the
subsampling factor by comparing `uv_*` to `y_*` and right-shifts the
extension size accordingly: `ss_x` is 1 only when U/V is narrower
than Y; `ss_y` is 1 only when U/V is shorter. The chroma extension
then becomes `ext_size >> ss_y` (top/bottom) and `ext_size >> ss_x`
(left/right). For VP8's fixed 4:2:0 this would also produce `>> 1`,
but the runtime check makes the code reusable.

`vpx_extend_frame_borders_c` is the public face: it just calls
`extend_frame(ybf, ybf->border)`. `vpx_extend_frame_inner_borders_c`
calls it with the smaller of `ybf->border` and `VP9INNERBORDERINPIXELS
= 96`, used by VP9 encoder paths that only need a smaller reachable
window.

**Why this exists alongside the VP8 entry point.** Historical: VP8
predates the more flexible chroma model, has a fixed half-border
convention, and ships its own simpler entry point. The two coexist in
this single file because they share `extend_plane`. In a VP8-only
build (`CONFIG_VP9` undefined) the entire block from line 131 to 187
is omitted.

### `memcpy_short_addr` (lines 181–185, VP9 + HBD only)

Tiny shim that does a `memcpy` between two `uint8_t *` arguments that
actually point at `uint16_t` data. Used by the high-bit-depth copy
path below. Excluded from VP8 builds.

### `vp8_yv12_copy_frame_c` (lines 193–232)

```c
void vp8_yv12_copy_frame_c(const YV12_BUFFER_CONFIG *src_ybc,
                           YV12_BUFFER_CONFIG *dst_ybc) {
  ...
  for (row = 0; row < src_ybc->y_height; ++row) {
    memcpy(dst, src, src_ybc->y_width);
    src += src_ybc->y_stride;
    dst += dst_ybc->y_stride;
  }
  ...
  vp8_yv12_extend_frame_borders_c(dst_ybc);
}
```

**What.** Whole-frame deep copy of the three planes from one YV12
buffer into another, followed by border extension of the destination.

**Why.** The reference-frame pool described in §10.2 of the overview
normally avoids copies by manipulating reference counts: when the
frame header asks for "GOLDEN ← LAST" the implementation rewrites
indices and bumps a refcount instead of moving pixels. There are
nevertheless paths — most prominently `copy_buffer_to_arf` /
`copy_buffer_to_gf` cases where the source slot is *also* being
overwritten this frame — that require a true copy so the old contents
survive. `vp8_yv12_copy_frame_c` is that copy.

The trailing `vp8_yv12_extend_frame_borders_c(dst_ybc)` call is the
load-bearing detail: this function copies only the in-picture pixels
(`width × height`, not stride × height), so the destination's border
contains whatever was previously there. Callers store these copies
back into the reference-frame pool where the next frame's motion
compensation will read past the edge; the borders must therefore be
re-extended before the buffer can be used as a reference. Folding the
extend into the copy means no caller can forget.

**Invariants.** The function assumes (but does not assert; see the
`#if 0` block) that source and destination have identical
`y_width`/`y_height`. It does not assume identical strides — the loop
advances by each buffer's own stride — so the two buffers can have
been allocated with different alignment. The padded dimensions
(`y_height`, `uv_height`) drive the loop, not the cropped ones,
because the destination will then be border-extended over the entire
padded region anyway.

**Comment from the source about the disabled assertions** (lines 199–
205):

```c
#if 0
  /* These assertions are valid in the codec, but the libvpx-tester uses
   * this code slightly differently.
   */
  assert(src_ybc->y_width == dst_ybc->y_width);
  assert(src_ybc->y_height == dst_ybc->y_height);
#endif
```

The assertions are correct for production decoder use; they are
disabled only to keep the libvpx-tester harness — which abuses this
function to copy between mismatched buffers — from crashing.

**How used.** Inside the VP8 decoder, see the reference-buffer
management around `vp8_yv12_copy_frame` in
`vp8/common/swapyv12buffer.c` and the swap logic in
`vp8/decoder/onyxd_if.c` (§10.2 of the overview).

### `vpx_yv12_copy_frame_c` (lines 235–308, VP9 only)

The VP9-side copy with optional high-bit-depth dispatch. Same shape
as the VP8 version but ends by calling `vpx_extend_frame_borders_c`
instead of the VP8 entry point, so it uses the runtime-subsampling
extender. Not compiled into a VP8-only build.

### `vpx_yv12_copy_y_c` (lines 311–335)

```c
void vpx_yv12_copy_y_c(const YV12_BUFFER_CONFIG *src_ybc,
                       YV12_BUFFER_CONFIG *dst_ybc) {
```

**What.** Copies only the luma plane from one buffer to another, with
optional high-bit-depth path. Does *not* extend borders afterwards.

**Why and how used.** Used by callers that want to preserve only the
Y plane (denoising / metric paths in the encoder; analysis tools).
Although the symbol is exported through the RTCD table and therefore
linked into every build, no path in the VP8 decoder calls it; it is
present here because the file is shared with VP9 and the encoder.
Mentioned for completeness — a VP8-decoder-only fork could delete it.

## Relationship to `vp8/common/extend.c`

`vp8/common/extend.c` provides a parallel set of functions
(`vp8_copy_and_extend_frame`, `vp8_copy_and_extend_frame_with_rect`)
that operate at the same conceptual layer but take `uint8_t *`
pointers explicitly and are used for partial / rectangular extension
during MB-level setup. `yv12extend.c` is the higher-level partner:
it accepts the opaque `YV12_BUFFER_CONFIG` container, walks its three
planes, computes the chroma border by halving the luma border, and
hides the asymmetry between "real border" and "padded slack" inside
its `extend_*` arguments. Decoder code calls `yv12extend.c`'s
top-level entry points; `extend.c` is reserved for narrower
intra-frame patching.

## Why this matters for the decoder pipeline

The "every MV is in-bounds" guarantee of the VP8 motion-compensation
inner loop — see overview §8.6 and §10.3 — is funded entirely by this
file. MV clamping at decode time (in `decodemv.c` via
`vp8_check_mv_bounds`) ensures the requested filter taps stay within
the 32-pixel border; the actual *existence* of valid pixel data inside
that border, ready to feed the 6-tap sub-pel filter, is what
`vp8_yv12_extend_frame_borders_c` provides at the end of each frame.
The two are a contract: clamp guarantees no read past `border`, extend
guarantees every read within `border` returns the edge-clamped value
the reference model expects. Without the extension step the
motion-compensation kernels would have to insert per-pixel boundary
checks, costing several percent of decode time on every inter MB.
