# `vpx_scale/generic/yv12config.c` — YV12 frame-buffer allocation

## Role in the decoder

Every reference picture and every reconstruction target that the VP8
decoder works with is a `YV12_BUFFER_CONFIG`: a triple of plane pointers
(Y, U, V) into a single contiguous slab of memory, garnished with
strides, widths, heights, and a border. The structure is declared in
`vpx_scale/yv12config.h:29` and used throughout the decoder via the
`yv12_fb[NUM_YV12_BUFFERS]` array carried inside `VP8_COMMON`
(see the technical overview, §10.1). `yv12config.c` is the
allocator/deallocator for those buffers — the only place in the
decoder where the raw pixel storage is sized, laid out, and freed.

There are three responsibilities concentrated in this small file:

1. **Lay out the planes inside one allocation.** A single
   `vpx_memalign(32, …)` returns a 32-byte-aligned block big enough
   to hold a bordered Y plane followed by two bordered U and V
   planes. `y_buffer`, `u_buffer`, `v_buffer` are then offsets into
   that one slab — every other function in the decoder treats them
   as if they were independent allocations, but free/alloc always
   operate on `buffer_alloc` as a unit.

2. **Honor stride and border invariants.** The Y stride is rounded up
   so that `(aligned_width + 2*border + 31) & ~31` — a multiple of
   32 — and the U/V stride is exactly `y_stride / 2`. The border
   itself must be a multiple of 32. Together these guarantees keep
   every row aligned to 16 bytes and let the subsampled chroma
   strides be derived by a simple right shift rather than recomputed
   from the U/V width independently. The 32-pixel border is what
   makes unclipped motion-vector reads legal (overview §8 explains
   why the loop filter and sub-pel interpolation can read pixels
   outside the picture rectangle); see `vp8/common/extend.c` and
   `vpx_scale/generic/yv12extend.c` for the code that replicates
   edge pixels into that border after each frame is reconstructed.

3. **Support an external-frame-buffer callback path.** The VP9 entry
   point (compiled only when `CONFIG_VP9` is on) lets the embedder
   own the memory: instead of calling `vpx_memalign`, the allocator
   calls back into user code with the required size and uses the
   buffer the caller supplies. The VP8 decoder does not use that
   path — `vp8_yv12_alloc_frame_buffer` always allocates internally
   via `vpx_memalign` — but the same file holds both
   implementations.

In the verified minimal VP8-decoder build (`vp8_files.md`, §A) only
the three VP8-flavored entry points (`vp8_yv12_alloc_frame_buffer`,
`vp8_yv12_realloc_frame_buffer`, `vp8_yv12_de_alloc_frame_buffer`)
are actually linked; the `vpx_*` variants compile out behind
`#if CONFIG_VP9`. The discussion below covers both because the file
holds both, but the reader interested only in VP8 can stop reading
once the alpha-buffer assignment is explained.

## Includes and dependencies

```c
#include <assert.h>
#include <limits.h>
#include <stdint.h>

#include "vpx_scale/yv12config.h"
#include "vpx_mem/vpx_mem.h"
#include "vpx_ports/mem.h"

#if defined(VPX_MAX_ALLOCABLE_MEMORY)
#include "vp9/common/vp9_onyxc_int.h"
#endif  // VPX_MAX_ALLOCABLE_MEMORY
```

`yv12config.h` declares both the struct layout and the prototypes of
every function defined here. `vpx_mem.h` supplies `vpx_memalign` and
`vpx_free`; `vpx_ports/mem.h` is pulled in for `CONVERT_TO_BYTEPTR`,
which is used only in the VP9 high-bit-depth branch. The
`VPX_MAX_ALLOCABLE_MEMORY` block is a security/robustness cap on a
single VP9 allocation (and reaches into `vp9_onyxc_int.h` purely for
the `REF_FRAMES` constant). None of that is active in a VP8-only build.

## The alignment macro

```c
#define yv12_align_addr(addr, align) \
  (void *)(((size_t)(addr) + ((align) - 1)) & (size_t)-(align))
```

### `yv12_align_addr` — round a pointer up to an alignment

**What.** A standard "next multiple of `align`" pointer-arithmetic
macro. `(addr + align - 1) & -align` masks off the low bits, where
`align` is required to be a power of two.

**Why.** It is used only by the VP9 path, in two ways: to align the
externally provided callback buffer to 32 bytes
(`yv12config.c:224`), and to push each plane pointer up to the
caller-specified `byte_alignment` (`yv12config.c:279–287`). The VP8
path does not need this because (a) `vpx_memalign(32, …)` already
returns a 32-byte-aligned base, and (b) the VP8 border is required
to be a multiple of 32, so adding `border * y_stride + border` to a
32-aligned base lands you on another 32-aligned address.

**Invariant.** `align` must be a non-zero power of two. The cast to
`size_t` and the unary `-` are the trick that turns "round down to
the previous multiple of a power of two" into a bitmask; the
addition of `align - 1` first turns it into "round up."

## The VP8 entry points

The three VP8 functions form an obvious cycle: `alloc` first calls
`de_alloc` then `realloc`; `realloc` does the actual work and is
also called directly when the caller wants to grow an existing
buffer without freeing it first.

### `vp8_yv12_de_alloc_frame_buffer` — release a YV12 buffer

```c
int vp8_yv12_de_alloc_frame_buffer(YV12_BUFFER_CONFIG *ybf) {
  if (ybf) {
    // If libvpx is using frame buffer callbacks then buffer_alloc_sz must
    // not be set.
    if (ybf->buffer_alloc_sz > 0) {
      vpx_free(ybf->buffer_alloc);
    }

    /* buffer_alloc isn't accessed by most functions.  Rather y_buffer,
      u_buffer and v_buffer point to buffer_alloc and are used.  Clear out
      all of this so that a freed pointer isn't inadvertently used */
    memset(ybf, 0, sizeof(YV12_BUFFER_CONFIG));
  } else {
    return -1;
  }
  return 0;
}
```

**What.** Frees the underlying allocation and zeroes the whole
`YV12_BUFFER_CONFIG`.

**Why the `buffer_alloc_sz > 0` guard.** When external frame buffers
are in use (VP9 only), `buffer_alloc` points into memory owned by
the caller and was never produced by `vpx_memalign`. The convention
this file establishes is that **`buffer_alloc_sz == 0` means "do not
free; the buffer is borrowed"**, while a positive size means
libvpx-owned. The comment in the source spells this out. In the VP8
path `buffer_alloc_sz` is always set whenever `buffer_alloc` is, so
the free always happens.

**Why the full `memset`.** The pointers `y_buffer`, `u_buffer`,
`v_buffer` are aliases into the freed `buffer_alloc`. Leaving them
populated after the free would arm a use-after-free trap for any
caller that reused the slot without re-allocating. Zeroing the
whole struct makes a missed re-allocation manifest as an obvious
null-pointer dereference rather than as memory corruption.

**Return value.** `0` on success, `-1` if `ybf == NULL`. This is the
file's universal convention: negative on failure, zero on success.

### `vp8_yv12_realloc_frame_buffer` — size, allocate, and lay out the planes

This is the core of the file. It computes the geometry, possibly
allocates, and writes every field of the `YV12_BUFFER_CONFIG`.

#### The size arithmetic

```c
int aligned_width  = (width + 15) & ~15;
int aligned_height = (height + 15) & ~15;
int y_stride       = ((aligned_width + 2 * border) + 31) & ~31;
int yplane_size    = (aligned_height + 2 * border) * y_stride;
int uv_width       = aligned_width >> 1;
int uv_height      = aligned_height >> 1;
/** There is currently a bunch of code which assumes
 *  uv_stride == y_stride/2, so enforce this here. */
int uv_stride      = y_stride >> 1;
int uvplane_size   = (uv_height + border) * uv_stride;
const size_t frame_size = yplane_size + 2 * uvplane_size;
```

Step by step:

* `aligned_width / aligned_height` round the displayed resolution
  up to a multiple of 16. **Why 16?** A VP8 macroblock is 16×16, and
  the decoder works exclusively in MB units; padding the picture out
  to a whole number of MBs means the per-MB loops never need a
  partial-tile path. The discarded pad pixels live in the bottom and
  right margin and are not displayed (`y_crop_width`/`y_crop_height`
  remember the true picture rectangle).

* `y_stride` is `aligned_width + 2 * border`, then rounded up to a
  multiple of 32. The `2 * border` accounts for the left and right
  border strips; the `& ~31` rounding makes every row 32-byte
  aligned, which matches the alignment required by AVX2 and several
  ARM NEON helpers (and is comfortably more than the 16 bytes needed
  by SSE2). For a 1920-wide frame with the standard
  `VP8BORDERINPIXELS = 32` border, this comes to
  `1920 + 64 = 1984` already a multiple of 32; for non-multiple
  widths the round-up adds at most 31 bytes per row.

* `yplane_size = (aligned_height + 2*border) * y_stride` is the
  full bordered Y plane: top + bottom border rows folded in.

* The chroma planes are 4:2:0 subsampled (each chroma sample
  represents a 2×2 block of luma), so `uv_width` and `uv_height`
  are exactly half of `aligned_width` and `aligned_height`. The
  border around chroma is exactly half the luma border, and the
  comment makes the **invariant `uv_stride == y_stride / 2`**
  explicit. This is a load-bearing assumption: code throughout
  the VP8 reconstructor — for instance the per-MB pointer
  arithmetic in `mbpitch.c` and the sub-pel motion-compensation
  paths in `reconinter.c` — derives `uv_stride` by shifting
  `y_stride` rather than reading it from the struct. Breaking the
  invariant would silently corrupt chroma reads. Note also that
  `uvplane_size` uses `(uv_height + border) * uv_stride`, not
  `(uv_height + 2*border_uv) * uv_stride` — i.e., it accounts for
  `border` rows of *chroma* slack, which is twice the strictly
  needed `border/2` chroma rows top + bottom. The extra rows
  are harmless padding and keep the arithmetic simple.

* `frame_size = yplane_size + 2 * uvplane_size` is the total slab.
  The layout, end to end, is

  ```
  ┌─────────────── yplane_size ───────────────┐
  │ bordered Y plane                          │
  ├─────────────── uvplane_size ──────────────┤
  │ bordered U plane                          │
  ├─────────────── uvplane_size ──────────────┤
  │ bordered V plane                          │
  └───────────────────────────────────────────┘
  ```

#### The allocation, with a callback-buffer carve-out

```c
if (!ybf->buffer_alloc) {
  ybf->buffer_alloc = (uint8_t *)vpx_memalign(32, frame_size);
  if (!ybf->buffer_alloc) {
    ybf->buffer_alloc_sz = 0;
    return -1;
  }
  ...
  ybf->buffer_alloc_sz = frame_size;
}

if (ybf->buffer_alloc_sz < frame_size) return -1;
```

`vpx_memalign(32, …)` is libvpx's `aligned_alloc` analogue. The
allocator only runs when `buffer_alloc` is currently null — i.e.,
the first time, or after a `de_alloc`. If the buffer already
exists and is at least `frame_size`, it is reused; that is the
"realloc smaller or equal" fast path. **If the existing buffer is
too small, this function fails with `-1`.** Growing a buffer
therefore requires the caller to free first (`vp8_yv12_alloc_frame_buffer`
does exactly that). The msan-only `memset` is a workaround so that
sanitizer builds do not flag reads of unallocated border pixels.

#### The border-multiple-of-32 check

```c
if (border & 0x1f) return -3;
```

**Why 32.** The comment in the source is the canonical answer:

> Only support allocating buffers that have a border that's a
> multiple of 32. The border restriction is required to get
> 16-byte alignment of the start of the chroma rows without
> introducing an arbitrary gap between planes, which would break
> the semantics of things like `vpx_img_set_rect()`.

If `border` were e.g. 24, then `u_buffer = buffer_alloc +
yplane_size + (12 * uv_stride) + 12` would land on an unaligned
byte, and the only way to fix it would be to insert padding
between Y and U — which would mean the slab is no longer a
contiguous `WxH I420` image, and helpers that hand the slab to
external consumers would have to lie about the layout. The VP8
default border `VP8BORDERINPIXELS` (yv12config.h:23) is 32; the
VP9 decode-time default `VP9_DEC_BORDER_IN_PIXELS` is also 32.
Returning `-3` for a bad border distinguishes this caller error
from the OOM case (`-1`) and the null-arg case (`-2`).

#### Field assignment

```c
ybf->y_crop_width  = width;             ybf->y_crop_height  = height;
ybf->y_width       = aligned_width;     ybf->y_height       = aligned_height;
ybf->y_stride      = y_stride;

ybf->uv_crop_width = (width + 1) / 2;   ybf->uv_crop_height = (height + 1) / 2;
ybf->uv_width      = uv_width;          ybf->uv_height      = uv_height;
ybf->uv_stride     = uv_stride;

ybf->alpha_width = 0;
ybf->alpha_height = 0;
ybf->alpha_stride = 0;

ybf->border = border;
ybf->frame_size = frame_size;
```

The `*_crop_*` fields hold the **true** displayable picture
rectangle; the `*_width`/`*_height` fields hold the
MB-aligned-and-padded *coded* rectangle. `(width + 1) / 2` is the
classic ceiling-divide for chroma crop: when `width` is odd, chroma
gets one extra column to cover the lonely luma sample. The alpha
plane is unsupported by VP8 (and largely vestigial across the
codebase — VP8 streams have no alpha channel), so the three alpha
fields are zeroed. `frame_size` is recorded so that callers can
size dumps and copies without recomputing.

#### Computing the plane pointers

```c
ybf->y_buffer = ybf->buffer_alloc + (border * y_stride) + border;
ybf->u_buffer =
    ybf->buffer_alloc + yplane_size + (border / 2 * uv_stride) + border / 2;
ybf->v_buffer = ybf->buffer_alloc + yplane_size + uvplane_size +
                (border / 2 * uv_stride) + border / 2;
ybf->alpha_buffer = NULL;

ybf->corrupted = 0; /* assume not currupted by errors */
return 0;
```

The Y plane starts `border` rows down (`border * y_stride`) and
`border` columns in (`+ border`), so that `y_buffer[0]` is the
*top-left displayable pixel* and `y_buffer[-1]`, `y_buffer[-y_stride]`,
etc., are valid border-extended pixels. The same offset trick, with
the half-border, positions `u_buffer` and `v_buffer` inside their
own planes within the same slab. **The picture origin (0,0) of each
plane is always the top-left of the displayable region; the border
lives at negative coordinates relative to that origin.** This is
exactly what the loop filter and sub-pel interpolators rely on
when they unconditionally read up to ±4 pixels (luma) or ±2
(chroma) outside the picture (overview §8, §11).

`corrupted = 0` is the optimistic init; the bitstream parser will
set it to 1 if a corrupt frame is detected
(`onyxd_if.c` / `decodeframe.c`).

### `vp8_yv12_alloc_frame_buffer` — fresh allocation

```c
int vp8_yv12_alloc_frame_buffer(YV12_BUFFER_CONFIG *ybf, int width, int height,
                                int border) {
  if (ybf) {
    vp8_yv12_de_alloc_frame_buffer(ybf);
    return vp8_yv12_realloc_frame_buffer(ybf, width, height, border);
  }
  return -2;
}
```

**What.** Free whatever is there, then allocate from scratch at the
new geometry.

**Why the two-step.** `realloc` will not grow an existing buffer
that is too small; it returns `-1` and leaves the old buffer in
place. Calling `de_alloc` first unconditionally nulls
`buffer_alloc`, which puts `realloc` on its allocate-from-scratch
path. This is the function the rest of the decoder calls on
resolution change (e.g. from `vp8_alloc_frame_buffers` in
`vp8/common/alloccommon.c`).

**Return convention.** `-2` for null arg (matches `realloc`),
otherwise whatever `realloc` returned. Note the asymmetry with
`de_alloc`, which returns `-1` for null arg. The codes are not
consistent across the three functions, but callers all check for
`!= 0`.

## The VP9 entry points (conditional)

The remainder of the file is wrapped in `#if CONFIG_VP9` and is not
compiled in the VP8-only build described in `vp8_files.md`. It is
discussed briefly for completeness; readers focused on the VP8
decoder may skip ahead to "summary."

### `vpx_free_frame_buffer` — VP9 deallocator

Structurally identical to `vp8_yv12_de_alloc_frame_buffer`: it
frees `buffer_alloc` if `buffer_alloc_sz > 0` (i.e., libvpx-owned,
not borrowed from a callback) and zeros the struct. The name is
codec-agnostic because VP9 frame buffers can be shared across
many in-flight frames via a reference-counted pool.

### `vpx_realloc_frame_buffer` — VP9 allocator with extras

The VP9 version of `realloc` takes the same `width`, `height`,
`border` as VP8, plus:

* `ss_x`, `ss_y` — chroma subsampling shifts (0 or 1 along each
  axis), so it supports 4:4:4, 4:4:0, 4:2:2 and 4:2:0. The VP8
  function hard-codes 4:2:0 (`uv_width = aligned_width >> 1`).
* `use_highbitdepth` — when set, every pixel occupies 16 bits
  rather than 8, so the slab is allocated at `2 * frame_size`
  and the plane pointers are cast through `CONVERT_TO_BYTEPTR`
  (which encodes the high-bit-depth indication into the pointer).
* `byte_alignment` — caller-specified alignment of plane starts,
  power-of-two from 32 to 1024 (header comment, yv12config.h:82).
  Implemented via the `yv12_align_addr` macro and adds
  `byte_alignment` worth of slack to each plane size for the
  worst-case alignment cost.
* `fb`, `cb`, `cb_priv` — the external-frame-buffer callback path.

The macroblock size is 8 for VP9 (versus 16 for VP8), so width and
height are rounded up to multiples of 8: `(width + 7) & ~7`. The
Y stride is rounded to 32 just as in VP8.

#### The external-frame-buffer callback path

The callback mechanism lets the application — typically a hardware
pipeline that wants the decoded pixels to land in a DMA-able
buffer it has already allocated — supply the storage:

```c
if (cb != NULL) {
  const int align_addr_extra_size = 31;
  const uint64_t external_frame_size = frame_size + align_addr_extra_size;
  assert(fb != NULL);
  if (external_frame_size != (size_t)external_frame_size) return -1;
  if (cb(cb_priv, (size_t)external_frame_size, fb) < 0) return -1;
  if (fb->data == NULL || fb->size < external_frame_size) return -1;
  ybf->buffer_alloc = (uint8_t *)yv12_align_addr(fb->data, 32);
  ...
}
```

The callback receives the size in bytes (with 31 extra so the
realigned pointer always fits), populates `fb->data` and
`fb->size`, and returns 0 on success. Libvpx then aligns the
returned pointer to 32 bytes and uses it directly. **Crucially,
`buffer_alloc_sz` is left at zero** (the assignment inside the
`else if` branch is skipped), so `vpx_free_frame_buffer` will
later free nothing — the application retains ownership of the
storage. The reverse path, registering the callback, is
`vpx_codec_set_frame_buffer_functions` (not in this file).

#### Memory caps

```c
#if defined(VPX_MAX_ALLOCABLE_MEMORY)
if (frame_size > VPX_MAX_ALLOCABLE_MEMORY / REF_FRAMES) return -1;
#endif
```

When built with a memory cap, the per-frame allocation is bounded
to `VPX_MAX_ALLOCABLE_MEMORY / REF_FRAMES`, because the decoder
will hold up to `REF_FRAMES` such buffers concurrently. This is a
hardening measure against pathologically large dimensions in
attacker-controlled streams. Together with the `CONFIG_SIZE_LIMIT`
check (`width > DECODE_WIDTH_LIMIT || height > DECODE_HEIGHT_LIMIT`)
at the top of the function, these are the only checks against
hostile resolution declarations.

The `if (frame_size > SIZE_MAX)` check below it is for 32-bit
platforms where `uint64_t` (used for `frame_size`) is wider than
`size_t`.

### `vpx_alloc_frame_buffer` — VP9 fresh allocation

Mirror of `vp8_yv12_alloc_frame_buffer`: `vpx_free_frame_buffer`,
then `vpx_realloc_frame_buffer` with no callback. Used by VP9
internal-allocation flows.

## Summary

`yv12config.c` is a deliberately small file with three jobs: pick
sizes that satisfy MB-multiple, stride-32, border-multiple-of-32,
and `uv_stride == y_stride/2`; allocate one 32-byte-aligned slab;
and produce plane pointers that sit at the (0,0) of the displayable
picture with the border at negative offsets. Everything else in the
decoder — sub-pel interpolation, the loop filter, border extension
in `yv12extend.c`, the macroblock pointer scaffolding in `mbpitch.c`,
and the picture-to-`vpx_image_t` wrapping in `vp8_dx_iface.c` —
relies on those invariants being held by this file. The VP9
callback path lets an embedder substitute its own storage without
changing any of those geometric guarantees.
