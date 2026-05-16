# `vpx/src/vpx_image.c` — the `vpx_image_t` lifecycle

## Role in the decoder

The VP8 decoder is an internal machine that operates exclusively on its own
`YV12_BUFFER_CONFIG` frame buffers (allocated by `vp8/common/alloccommon.c`
and configured by `vpx_scale/generic/yv12config.c`). The public C API, on the
other hand, hands reconstructed frames to the caller in a small, format-agnostic
descriptor called `vpx_image_t`. The file `vpx/src/vpx_image.c` is the
implementation of that descriptor. It contains the handful of functions —
`vpx_img_alloc`, `vpx_img_wrap`, `vpx_img_set_rect`, `vpx_img_flip`, and
`vpx_img_free` — that the public header `vpx/vpx_image.h` advertises to
applications.

In the decoder build (see `vp8_files.md`, section A), `vpx_image.c` is one of
only four files compiled from `vpx/src/`. The codec dispatcher (`vpx_codec.c`)
and the decoder entry points (`vpx_decoder.c`) ferry calls in and out;
`vpx_image.c` describes the *shape* of the data those calls move. The VP8
decode loop never invokes anything in this file directly. Instead, when the
application calls `vpx_codec_get_frame` and the decoder is ready to surface a
finished picture, the codec-specific glue inside `vp8_dx_iface.c` populates a
preallocated `vpx_image_t` whose `planes[]` and `stride[]` point straight into
the internal `YV12_BUFFER_CONFIG`. So the role of `vpx_image.c` divides cleanly
in two:

1. **Allocate or wrap** a `vpx_image_t` plus (optionally) its pixel storage,
   computing plane strides and per-format alignment.
2. **Crop** the visible viewport via `vpx_img_set_rect`, which rewrites the
   `planes[]` pointers to point at the top-left pixel of a sub-rectangle.

The two distinct construction routines — `vpx_img_alloc` (allocates pixels)
versus `vpx_img_wrap` (borrows pixels from the caller) — are the central
distinction this file embodies. They share a single private helper,
`img_alloc_helper`, that hides every fiddly piece of arithmetic.

A consequence worth stating up front: `vpx_image.c` has no dependency on any
VP8-specific data structure. It only knows about formats, bytes, and strides.
That is what lets the same descriptor type serve VP8 decode, VP9 decode, and
both encoders.

## Headers and helpers

The file pulls in only standard library headers (`assert.h`, `limits.h`,
`stdlib.h`, `string.h`) plus three vpx headers: `vpx/vpx_image.h` for the
descriptor type and format enum, `vpx/vpx_integer.h` for `uint64_t` / `uint8_t`
typedefs, and `vpx_mem/vpx_mem.h` for `vpx_memalign` and `vpx_free`. The use of
the vpx allocator (rather than plain `malloc`/`free`) is important: image pixel
storage must satisfy a caller-specified power-of-two alignment that frequently
exceeds the natural `malloc` alignment, so `vpx_memalign` is the only viable
allocator for the pixel buffer.

### `is_valid_img_fmt` — admission test for the format enum

```c
static int is_valid_img_fmt(vpx_img_fmt_t fmt) {
  switch (fmt) {
    case VPX_IMG_FMT_YV12:
    case VPX_IMG_FMT_I420:
    ...
    case VPX_IMG_FMT_I44016: return 1;
    default: return 0;
  }
}
```

**What.** A static predicate that returns 1 for every format that
`img_alloc_helper` knows how to handle, and 0 otherwise.

**Why.** `vpx_img_fmt_t` is an `enum` whose values are bit-packed: the bottom
nibble is an arbitrary numeric tag and the high bits (`VPX_IMG_FMT_PLANAR`,
`VPX_IMG_FMT_UV_FLIP`, `VPX_IMG_FMT_HAS_ALPHA`, `VPX_IMG_FMT_HIGHBITDEPTH`) are
attribute flags ORed in. Because C lets the caller cast any integer through the
enum, the helper cannot assume the value passed in is a member it actually
recognises. This whitelist is the first thing `img_alloc_helper` consults, and
a `default` return of 0 makes additions to the format enum fail loudly here
rather than crashing later inside the per-format `bps`/chroma-shift switches.

**Invariant.** Exactly the formats listed return 1; in particular,
`VPX_IMG_FMT_NONE`, packed-RGB sentinels (none exist in this build), and any
unknown bit pattern return 0.

**How used.** Sole caller is `img_alloc_helper`, immediately after zeroing the
descriptor:

```c
if (!is_valid_img_fmt(fmt)) goto fail;
```

### `img_alloc_helper` — the common construction routine

This is the workhorse of the file. Both `vpx_img_alloc` and `vpx_img_wrap` are
one-line wrappers over it, distinguished by whether they pass `img_data == NULL`
(allocate-and-own) or `img_data != NULL` (wrap an external buffer). The two
also differ in how they pass alignment, as discussed below. The helper performs
six logical steps; we walk through them in source order.

**Step 1 — zero the descriptor and validate inputs.**

```c
if (img != NULL) memset(img, 0, sizeof(vpx_image_t));

if (!is_valid_img_fmt(fmt)) goto fail;

if (d_w > 0x08000000 || d_h > 0x08000000 || buf_align > 65536 ||
    stride_align > 65536) {
  goto fail;
}
```

The `memset` clears all `private` fields (`img_data_owner`, `self_allocd`,
`user_priv`, `fb_priv`, etc.) so that the cleanup path `fail:`→`vpx_img_free`
sees a consistent state regardless of whether the caller passed in a stack
descriptor or whether the helper later allocated one. The dimension cap of
2^27 and alignment cap of 65536 are not artistic choices — they are picked
specifically so that all the multiplications that follow stay below the 64-bit
range with room to spare, so the code never has to test for `uint64_t` overflow.

**Step 2 — normalize and validate alignment.**

```c
if (!buf_align) buf_align = 1;
if (buf_align & (buf_align - 1)) goto fail;
if (!stride_align) stride_align = 1;
if (stride_align & (stride_align - 1)) goto fail;
```

Two alignments matter: `buf_align` for the base address of the pixel buffer,
and `stride_align` for the byte length of one luma row. Zero is silently
upgraded to 1 (no alignment); anything that isn't a power of two is rejected.

**Step 3 — translate format to `bps`, `xcs`, `ycs`.** Three back-to-back
switches map the format enum to:

- `bps`: bits per sample of the *composite* pixel (12 for 4:2:0, 16 for 4:2:2
  and 4:4:0, 24 for 4:4:4 or 4:2:0/16-bit, 32 for 4:2:2/16-bit and 4:4:0/16-bit,
  48 for 4:4:4/16-bit).
- `xcs`, `ycs`: horizontal and vertical chroma shifts. A `1` here means the
  chroma plane is half-resolution along that axis. The comment specifically
  notes that `VPX_IMG_FMT_NV12` deliberately has `xcs = 0` even though it is
  4:2:0 sampled — because in NV12 the U and V samples are *interleaved* in
  the same plane, so the byte stride is the same as luma, not half.

**Step 4 — round storage dimensions up to the chroma alignment.**

```c
if (img_data) {
  w = d_w;
  h = d_h;
} else {
  align = (1 << xcs) - 1;
  w = (d_w + align) & ~align;
  align = (1 << ycs) - 1;
  h = (d_h + align) & ~align;
}
```

This is the first place where alloc and wrap diverge. When the helper is going
to allocate the buffer itself, it pads `d_w` up to a multiple of `1 << xcs`
(2 for any 4:2:0/4:2:2 format) and similarly for height, so that the chroma
planes are an exact integer number of samples wide and tall. When the caller
brought their own buffer (`img_data != NULL`), the helper trusts whatever
dimensions the caller passed — it has no way to enlarge a buffer it didn't
allocate.

**Step 5 — compute `stride_in_bytes`.**

```c
s = (fmt & VPX_IMG_FMT_PLANAR) ? w : (uint64_t)bps * w / 8;
s = (fmt & VPX_IMG_FMT_HIGHBITDEPTH) ? s * 2 : s;
s = (s + stride_align - 1) & ~((uint64_t)stride_align - 1);
if (s > INT_MAX) goto fail;
stride_in_bytes = (int)s;
s = (fmt & VPX_IMG_FMT_HIGHBITDEPTH) ? s / 2 : s;
```

For planar formats the luma stride is one byte per sample, so it starts at
`w`; for hypothetical packed formats (none in this enum but the code is
defensive) it would be `bps * w / 8`. High-bit-depth formats then double the
byte stride because each sample occupies two bytes. Finally the byte stride is
rounded up to `stride_align`. The bound check against `INT_MAX` matters because
the public `stride[]` field of `vpx_image_t` is `int`, not `unsigned`.

After the rounding, the helper undoes the high-bit-depth doubling on `s`. The
reason is that the post-rounding `s` is now used a few lines later as
"samples per row" when computing the total allocation size — pixel arithmetic
is done in samples, byte arithmetic in `stride_in_bytes`.

**Step 6 — allocate the descriptor (if needed), allocate the pixel buffer
(if needed), and populate the descriptor.**

```c
if (!img) {
  img = (vpx_image_t *)calloc(1, sizeof(vpx_image_t));
  if (!img) goto fail;
  img->self_allocd = 1;
}

img->img_data = img_data;

if (!img_data) {
  uint64_t alloc_size;
  alloc_size = (fmt & VPX_IMG_FMT_PLANAR) ? (uint64_t)h * s * bps / 8
                                          : (uint64_t)h * s;
  if (alloc_size != (size_t)alloc_size) goto fail;
  img->img_data = (uint8_t *)vpx_memalign(buf_align, (size_t)alloc_size);
  img->img_data_owner = 1;
}
```

The two boolean fields `self_allocd` and `img_data_owner` are crucial: they
remember, for the eventual `vpx_img_free`, whether the descriptor was heap-
allocated here (and therefore must be `free`d) and whether the pixel buffer was
allocated here (and therefore must be `vpx_free`d). If the caller passed both
a stack descriptor and an external buffer, both flags remain 0 and
`vpx_img_free` is effectively a no-op.

The composite-`bps` formula `h * s * bps / 8` works because for planar formats
`s` is in samples (after the divide-back on the previous step) and `bps` is the
total bits per pixel across all planes. So `h * s` is the luma plane in samples
and multiplying by `bps / 8` and dividing again expands that to the full
multi-plane byte count.

**Step 7 — fill in metadata and per-plane strides.**

```c
img->fmt = fmt;
img->bit_depth = (fmt & VPX_IMG_FMT_HIGHBITDEPTH) ? 16 : 8;
img->w = w; img->h = h;
img->x_chroma_shift = xcs; img->y_chroma_shift = ycs;
img->bps = bps;

img->stride[VPX_PLANE_Y] = img->stride[VPX_PLANE_ALPHA] = stride_in_bytes;
img->stride[VPX_PLANE_U] = img->stride[VPX_PLANE_V] = stride_in_bytes >> xcs;
```

`stride[VPX_PLANE_Y]` (which aliases `stride[VPX_PLANE_PACKED]` — both are
index `0`) and the alpha plane use the full luma stride. The two chroma planes
use the stride right-shifted by `xcs`: half-stride for 4:2:0 / 4:2:2, full for
4:4:4 and NV12.

**Step 8 — set the default viewport.** The helper finishes by calling
`vpx_img_set_rect(img, 0, 0, d_w, d_h)`. This populates `img->planes[*]` to
point at the top-left of each plane, sets `d_w`/`d_h`, and never fails for the
trivial rectangle (hence the `assert(ret == 0)`).

**The `fail` path.** Any error jumps to `fail:`, where `vpx_img_free(img)` is
called. Because `memset` happened at the very top and the two ownership flags
are only set after their respective allocations succeed, `vpx_img_free` always
sees a coherent set of flags: it frees only what was actually allocated.

### `vpx_img_alloc` — allocate-and-own

```c
vpx_image_t *vpx_img_alloc(vpx_image_t *img, vpx_img_fmt_t fmt,
                           unsigned int d_w, unsigned int d_h,
                           unsigned int align) {
  return img_alloc_helper(img, fmt, d_w, d_h, align, align, NULL);
}
```

**What.** Public allocator: returns a fully usable `vpx_image_t` plus a freshly
`vpx_memalign`'d pixel buffer.

**Why a single `align`?** The public API exposes only one alignment parameter.
The helper takes two: `buf_align` for the base address and `stride_align` for
the per-row byte length. `vpx_img_alloc` passes the same value for both, which
means: every row begins at the same alignment as the base, so the address of
pixel `(0, y)` for any `y` shares the alignment of pixel `(0, 0)`. That is
exactly what SIMD load instructions need.

**Invariants on return.** The returned descriptor satisfies:
`img_data != NULL`, `img_data_owner == 1`, `self_allocd == 1` iff the caller
passed `img == NULL`, and the four `planes[]` pointers reference the
`img_data` block.

**How used.** Sample applications, test harnesses, and any callers that want
the library to manage pixel storage. It is *not* called from inside the VP8
decoder core itself — VP8 manages its frame buffers separately.

### `vpx_img_wrap` — borrow caller-supplied pixels

```c
vpx_image_t *vpx_img_wrap(vpx_image_t *img, vpx_img_fmt_t fmt, unsigned int d_w,
                          unsigned int d_h, unsigned int stride_align,
                          unsigned char *img_data) {
  /* Set buf_align = 1. It is ignored by img_alloc_helper because img_data is
   * not NULL. */
  return img_alloc_helper(img, fmt, d_w, d_h, 1, stride_align, img_data);
}
```

**What.** Public wrapper: returns a `vpx_image_t` whose pixel storage was
allocated by the caller and is owned by the caller.

**Why no `buf_align`?** Since the helper will not allocate the pixel buffer,
the base-address alignment is whatever the caller already chose. Passing `1`
documents the intent: there is no allocation, so no alignment is applied.
`stride_align` *is* still meaningful — it controls the rounding-up of the
per-row byte stride that gets stored in `img->stride[]`. The caller's buffer
must therefore be at least as large as the helper computes; the header
documents the exact formula and even suggests a two-call protocol where
`vpx_img_wrap` is first called with a dummy buffer to learn the required size,
then again with the real allocation.

**Critical distinction from `vpx_img_alloc`.** When pixel storage comes from
outside, `img_alloc_helper` does *not* round up `d_w`/`d_h` to the chroma
alignment — it uses them verbatim. The caller is responsible for ensuring the
external buffer has the right dimensions. After this call, `img_data_owner`
is 0, so `vpx_img_free` will not touch the pixel buffer.

**How used.** This is the primary glue between the VP8 decoder's internal
`YV12_BUFFER_CONFIG` and the public API. `vp8_dx_iface.c` constructs a
`vpx_image_t` that wraps the decoder's reconstruction buffer, then hands it to
the application. The application sees a normal `vpx_image_t`; under the hood,
its pixels live in the decoder's own frame buffer.

### `vpx_img_set_rect` — install the visible viewport

```c
int vpx_img_set_rect(vpx_image_t *img, unsigned int x, unsigned int y,
                     unsigned int w, unsigned int h);
```

**What.** Sets `img->d_w`, `img->d_h`, and the four `planes[]` pointers so they
reference the top-left pixel of the sub-rectangle (`x`, `y`, `w`, `h`) within
the larger stored image (`img->w` × `img->h`).

**Why.** A decoder routinely produces a frame whose *stored* dimensions are
larger than its *displayed* dimensions: 16×16 macroblock alignment, chroma
subsampling, and border extension all conspire to push stored sizes up. The
caller still wants to receive a descriptor that, when iterated `0 .. d_w-1` by
`0 .. d_h-1`, hits exactly the displayable pixels. `set_rect` is the
mechanism: rather than copy data, it just offsets the plane pointers.

**Bounds and overflow check.**

```c
if (x <= UINT_MAX - w && x + w <= img->w && y <= UINT_MAX - h &&
    y + h <= img->h) { ... return 0; }
return -1;
```

The `x <= UINT_MAX - w` and `y <= UINT_MAX - h` clauses guard against the
addition wrapping in the comparisons that follow. If anything is out of range
the function returns `-1` without modifying the descriptor.

**Pointer arithmetic — packed case.** For non-planar formats it is a single
expression: skip `y` rows of stride and `x * bps / 8` bytes into the row.

**Pointer arithmetic — planar case.** Walk the planes in their on-disk order:

- If the format has an alpha plane, it comes first; advance `data` by
  `h * stride[A]` after handling it.
- The Y plane follows; advance `data` by `h * stride[Y]`.
- For NV12, the U pointer is at `(uv_x, uv_y)` in the interleaved UV plane,
  and the V pointer is `U + 1` (next byte in the same row).
- Otherwise the two chroma planes occupy `(h >> y_chroma_shift)` rows each.
  The U-first vs V-first ordering is selected by the `VPX_IMG_FMT_UV_FLIP`
  flag, which distinguishes YV12 (V first) from I420 (U first).

`uv_x` and `uv_y` are the chroma-plane coordinates derived by right-shifting
`x` and `y` by the chroma shifts. `bytes_per_sample` is 1 or 2 depending on
the high-bit-depth flag.

**Invariant.** On success, every `planes[i]` points to pixel (x, y) of plane
*i*, the strides are unchanged, and `d_w`/`d_h` reflect the new viewport.
A subsequent `vpx_img_set_rect(img, 0, 0, img->w, img->h)` always resets the
descriptor to "whole frame visible" — that is the call the constructor makes.

**How used.** `img_alloc_helper` calls it at the end of construction to
initialise the plane pointers. The codec layer (`vp8_dx_iface.c`) calls it
when wrapping an internal frame buffer whose stored dimensions exceed the
display dimensions, so that the application's view of the image is cropped to
the visible area.

### `vpx_img_flip` — present the image upside-down

```c
void vpx_img_flip(vpx_image_t *img) {
  img->planes[VPX_PLANE_Y] += (signed)(img->d_h - 1) * img->stride[VPX_PLANE_Y];
  img->stride[VPX_PLANE_Y] = -img->stride[VPX_PLANE_Y];
  ...
}
```

**What.** Advances each plane pointer to the *last* row of pixels and negates
the corresponding stride, so that iterating `0 .. d_h-1` walks the rows from
bottom to top.

**Why.** Some display surfaces (BMP, GL textures with origin-at-bottom) expect
bottom-up data. `vpx_img_flip` lets the caller present such a surface without
copying.

**The cast comment.** The leading comment is genuinely important. Without the
`(signed)` cast, `img->d_h - 1` is `unsigned`, and the C usual arithmetic
conversions would then promote `img->stride[...]` (an `int`) to `unsigned` as
well; on a platform where `unsigned` is narrower than the pointer-arithmetic
type, the resulting value could wrap before being added to the pointer. The
explicit cast forces the multiplication to be signed and large enough.

**Invariant after flip.** `stride[i]` is negated and `planes[i]` is moved by
`(d_h - 1) * old_stride`. A second `vpx_img_flip` restores the original state
(it is its own inverse).

**Decoder relevance.** Not used by the VP8 decoder itself; provided for
applications that want a flipped view of the decoded frame.

### `vpx_img_free` — coordinated tear-down

```c
void vpx_img_free(vpx_image_t *img) {
  if (img) {
    if (img->img_data && img->img_data_owner) vpx_free(img->img_data);
    if (img->self_allocd) free(img);
  }
}
```

**What.** Releases anything that was allocated by `img_alloc_helper`.

**Why two flags?** The descriptor and its pixel buffer have independent
ownership. The four combinations are:

| Construction call                                   | `self_allocd` | `img_data_owner` |
|-----------------------------------------------------|---------------|------------------|
| `vpx_img_alloc(NULL, ...)` — both heap-allocated    | 1             | 1                |
| `vpx_img_alloc(&stack_img, ...)` — desc on stack    | 0             | 1                |
| `vpx_img_wrap(NULL, ..., my_buf)` — desc on heap    | 1             | 0                |
| `vpx_img_wrap(&stack_img, ..., my_buf)` — neither   | 0             | 0                |

`vpx_img_free` must do the right thing in every case. Note that it uses
`vpx_free` (matched with `vpx_memalign`) for the pixel buffer but plain
`free` (matched with `calloc`) for the descriptor.

**Robustness.** Safe to call on a `NULL` pointer, and safe to call as part of
the `fail:` path inside `img_alloc_helper` because `memset` guarantees both
flags start at 0.

**Why no `vpx_img_realloc`?** The library deliberately doesn't expose one.
Resizing requires recomputing strides, plane offsets, and allocation sizes —
exactly what `vpx_img_alloc` already does. The intended pattern is
`vpx_img_free` followed by `vpx_img_alloc`.

## Putting it together — how the decoder uses this file

In a typical decode loop:

1. The application calls `vpx_codec_decode` to push compressed VP8 data into
   the decoder. None of `vpx_image.c` is involved.
2. The application calls `vpx_codec_get_frame`, which iterates pictures the
   decoder has finished reconstructing. The VP8-specific glue in
   `vp8_dx_iface.c` constructs (or reuses) a `vpx_image_t` via `vpx_img_wrap`,
   pointing it at the internal `YV12_BUFFER_CONFIG` of the just-finished
   frame, and then calls `vpx_img_set_rect` to crop to the displayable
   dimensions encoded in the bitstream.
3. The application reads `image->planes[0..2]` and `image->stride[0..2]` to
   copy or display the frame.
4. When the codec instance is destroyed, the internal frame buffers are freed
   by `alloccommon.c`; the wrapping `vpx_image_t` is freed by `vpx_img_free`,
   which does *not* touch the pixel storage (`img_data_owner == 0`).

The `vpx_img_alloc` path, by contrast, is what test harnesses and tools (e.g.
`vpxdec` reading raw YUV from disk for re-encode tests) use when they need
working frame storage but have no codec to draw it from.

That clean separation — *the codec owns the pixels, the wrapper owns the
metadata* — is the entire reason `vpx_image.c` exists as a separate file with
no codec dependencies.
