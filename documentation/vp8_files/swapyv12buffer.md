# `vp8/common/swapyv12buffer.c` — pointer-swap helpers for YV12 frame buffers

This is the smallest non-trivial `.c` file in the VP8 codebase: a single
function, twenty lines of body, no branches, no allocations, no
arithmetic. It exists to do one thing, and to do it without copying any
pixels.

```
$ wc -l vp8/common/swapyv12buffer.c
33 vp8/common/swapyv12buffer.c
```

The function it defines, `vp8_swap_yv12_buffer`, exchanges the heap
pointers of two `YV12_BUFFER_CONFIG` structs in place. After the call,
each struct still describes a valid YV12 picture — only the storage
underneath each label has changed places. Nothing in the picture is
read, nothing is written.

## Role in the decoder

In the VP8 model described in section 4 of `vp8_technical_overview.md`,
inter-prediction draws from up to three reference frames per macroblock:
`LAST_FRAME`, `GOLDEN_FRAME`, and `ALTREF_FRAME`. After each frame is
reconstructed, the bitstream's per-frame `refresh_last_frame`,
`refresh_golden_frame`, and `refresh_alt_ref_frame` bits decide which
of the three slots should adopt the just-decoded picture as their new
contents. That decision is what the overview calls "reference-frame
rotation."

A naïve implementation would memcpy the new frame's pixel planes into
the slot being refreshed. For a 1080p Y plane alone that is roughly
2 MB of data per refresh, and a single inter frame can trigger up to
three refreshes. The libvpx design avoids the copies entirely by
keeping the heap allocations stationary and exchanging only the
descriptor *pointers* that name them. `vp8_swap_yv12_buffer` is the
mechanical primitive for that exchange.

A subtle point that needs flagging up front: in the **decoder**
(`vp8/decoder/onyxd_if.c:215-263`), reference rotation is actually
implemented through a different mechanism — a bank of `MAX_REF_FRAMES`
buffers in `cm->yv12_fb[]` plus an array of small integer indices
(`cm->lst_fb_idx`, `cm->gld_fb_idx`, `cm->alt_fb_idx`) governed by
reference counts in `cm->fb_idx_ref_cnt`. The helper called there is
`ref_cnt_fb`, which rewrites an index and bumps/decrements counts; no
`YV12_BUFFER_CONFIG` fields move at all. The advantage of that scheme
is that several slots can share a buffer without copying, the
disadvantage is the indirection.

The encoder (`vp8/encoder/firstpass.c:817`) takes the simpler route and
calls `vp8_swap_yv12_buffer` directly:

```c
/* swap frame pointers so last frame refers to the frame we just
 * compressed
 */
vp8_swap_yv12_buffer(lst_yv12, new_yv12);
vp8_yv12_extend_frame_borders(lst_yv12);
```

This file is therefore on the **decoder** build list (per
`vp8_files.md` section A, `vp8/common/`) only because `vp8/common/`
is shared between encoder and decoder and the build doesn't split
the directory. In a pure decoder fork the file could be removed; in
the canonical libvpx tree it is built but never linked against any
decode-side caller. The documentation that follows is honest about
this: the function's specification is decoder-relevant (it formalizes
what reference-frame rotation *means*), but its observable use lives
in the encoder.

## Headers and the dependency surface

The include block at `swapyv12buffer.c:11` reaches into exactly one
thing:

```c
#include "swapyv12buffer.h"
```

The header in turn (`swapyv12buffer.h:14`) pulls in
`vpx_scale/yv12config.h`, which is the canonical home of
`YV12_BUFFER_CONFIG` (`yv12config.h:29-65`). That struct is the *only*
data type touched by this translation unit. No DSP primitives, no
common-state pointers, no codec-instance pointers — the helper is
deliberately decoupled from `VP8_COMMON` and `VP8D_COMP` so that it
can live in `common/` and be called from either pipeline.

### The `YV12_BUFFER_CONFIG` shape (relevant subset)

Out of the roughly thirty fields in `YV12_BUFFER_CONFIG`
(`yv12config.h:29-65`), `vp8_swap_yv12_buffer` touches only four:

| Field            | Type            | Meaning                                                 |
|------------------|-----------------|---------------------------------------------------------|
| `buffer_alloc`   | `uint8_t *`     | Base pointer of the single heap region holding all planes plus the extended border. Returned by `vpx_memalign` in `vp8_yv12_realloc_frame_buffer`. This is the address that `free()` ultimately needs. |
| `y_buffer`       | `uint8_t *`     | Pointer to the top-left of the *cropped* Y plane (`y_width × y_height`), inside `buffer_alloc` and past the top/left border. |
| `u_buffer`       | `uint8_t *`     | Pointer to the top-left of the U plane (`uv_width × uv_height`), 4:2:0 subsampled. |
| `v_buffer`       | `uint8_t *`     | Pointer to the top-left of the V plane. |

Everything else in the struct — the widths, heights, strides, border
size, subsampling factors, bit depth, color space, `corrupted` flag —
is **not** exchanged. The implicit invariant exploited by the
implementation is that the two buffers being swapped are dimensionally
identical. Otherwise the un-swapped `y_stride`/`y_width`/`y_height`
fields would describe storage that no longer matches the planes
they now address.

That invariant is enforced extrinsically: `vp8_alloc_frame_buffers`
(see `alloccommon.md`) allocates every reference slot from the same
`(width, height, border)` triple, so any two slots in `cm->yv12_fb[]`
are interchangeable by construction. If the picture is resized
mid-stream, all reference frames are reallocated together before any
swap can occur.

## The function

### `vp8_swap_yv12_buffer` — three-line pointer exchange repeated four times

```c
void vp8_swap_yv12_buffer(YV12_BUFFER_CONFIG *new_frame,
                          YV12_BUFFER_CONFIG *last_frame) {
  unsigned char *temp;

  temp = last_frame->buffer_alloc;
  last_frame->buffer_alloc = new_frame->buffer_alloc;
  new_frame->buffer_alloc = temp;

  temp = last_frame->y_buffer;
  last_frame->y_buffer = new_frame->y_buffer;
  new_frame->y_buffer = temp;

  temp = last_frame->u_buffer;
  last_frame->u_buffer = new_frame->u_buffer;
  new_frame->u_buffer = temp;

  temp = last_frame->v_buffer;
  last_frame->v_buffer = new_frame->v_buffer;
  new_frame->v_buffer = temp;
}
```

**What it does.** Performs the classic three-statement pointer swap on
each of the four pointer fields enumerated in the previous section,
using a single stack-local scratch (`temp`). The result is that
`new_frame` now describes the pixel storage that `last_frame`
previously named, and vice versa.

**Why a swap and not a copy.** The whole point of the helper is to
avoid touching the picture data. A 1920×1080 4:2:0 picture, even
without the extended border, is 3 MB of memory; copying it costs both
bandwidth and L2 footprint. The swap is O(1) and pure register
traffic. From the caller's perspective the rotation looks identical:
after `vp8_swap_yv12_buffer(lst_yv12, new_yv12)`, dereferencing
`lst_yv12->y_buffer` returns the pixels of what was just compressed,
which is exactly what `LAST_FRAME` is supposed to mean for the next
frame.

**Why all four pointers.** Three of the four (`y_buffer`, `u_buffer`,
`v_buffer`) are the working pointers — they are what intra/inter
reconstruction will index off of next time around. The fourth,
`buffer_alloc`, is the *deallocation handle*: it must travel with the
storage it owns, because eventually `vpx_free(ybf->buffer_alloc)`
inside `vp8_yv12_de_alloc_frame_buffer` is what releases the region.
If `buffer_alloc` were left behind in the swap, the next
deallocation pass would `free()` the wrong base address — almost
certainly a pointer into the middle of the *other* buffer's heap
region — and corrupt the allocator. Swapping all four pointers
together keeps every YV12 struct internally consistent: each
descriptor still owns, and can still free, the bytes its planes
point into.

**What it deliberately does not swap.** No dimension or stride field
moves. This is correct precisely because the implementation assumes
the two structs are dimensionally identical (same `y_width`,
`y_height`, `y_stride`, `uv_*`, `border`, `subsampling_x/y`,
`bit_depth`). If they were not, the swap would silently corrupt the
descriptors — the planes after the swap would still be addressed
with the strides and crops of the *un-swapped* struct, leading to
out-of-bounds reads on the very next access. The function does not
assert this invariant; it is the caller's responsibility. In
practice the only caller is `vp8_first_pass` after
`vp8_alloc_frame_buffers` has produced a uniformly-sized bank, so
the invariant holds by construction.

Also unswapped: `corrupted`. This is intentional. The flag is a
property of the *content* (was this picture decoded from a damaged
bitstream?), not of the storage. When you swap `last_frame` with a
freshly produced `new_frame`, you want `last_frame->corrupted` to
reflect the corruption status of what is now in last_frame's slot —
i.e. of the just-produced frame, whose `corrupted` value lived in
`new_frame->corrupted` *before* the swap. Yet the code does not move
that bit. The reason is that the encoder's `firstpass.c` caller has
no notion of stream corruption — it is producing first-pass
statistics from raw input — so the `corrupted` flag is not consulted
on the swap path. The decoder, which *does* consult `corrupted`,
uses the indexed/ref-counted scheme in `onyxd_if.c` instead and
never calls this helper. The bug would only surface if a future
caller mixed paradigms.

**Invariants summary.**

1. *Precondition.* The two argument buffers were allocated with
   identical `(width, height, border)`, identical
   `(subsampling_x, subsampling_y)`, and identical `bit_depth`. In
   the encoder build, that holds because the reference bank is
   created by a single `vp8_alloc_frame_buffers` call.
2. *Postcondition.* Each `YV12_BUFFER_CONFIG` still has internal
   consistency between its four heap pointers — i.e.
   `y_buffer`, `u_buffer`, `v_buffer` all point into the region
   beginning at `buffer_alloc`, with the same offsets as before the
   swap. No double-free can arise as long as each descriptor is
   freed exactly once and via its own `buffer_alloc`.
3. *Aliasing.* The function is **not** safe when
   `new_frame == last_frame`. The three-statement swap of a pointer
   with itself would still terminate with the same value, but no
   caller has any reason to ask for this case, and the function
   does not check.

**How it is used.** Exactly one call site exists in the entire
codebase (`vp8/encoder/firstpass.c:817`). The pattern there is the
canonical use:

```c
vp8_swap_yv12_buffer(lst_yv12, new_yv12);
vp8_yv12_extend_frame_borders(lst_yv12);
```

After the swap, `lst_yv12` describes the storage of the
just-compressed frame and is then handed to
`vp8_yv12_extend_frame_borders` to fill its extended-border region
(so that subsequent sub-pixel inter-prediction can read past the
image edges without bounds checks). The companion `new_yv12` now
holds what used to be the last frame; the encoder will overwrite it
in the next iteration without caring about its current contents.

The argument naming in the prototype (`new_frame`, `last_frame`)
documents this intent, even though the function is symmetric and
treats the two operands identically. Reading the call site, "swap
the new frame into the last-frame slot" reads naturally.

## Closing notes

`swapyv12buffer.c` is a useful object lesson in how libvpx separates
two distinct concepts that the VP8 spec conflates into a single
"reference frame":

- the **logical role** of a slot (`LAST_FRAME`, `GOLDEN_FRAME`,
  `ALTREF_FRAME`), which is what the bitstream's refresh bits
  manipulate;
- the **physical storage** of a YV12 picture, which is what
  `YV12_BUFFER_CONFIG` describes.

A reference rotation is, formally, just a permutation that re-binds
logical roles to physical storage. `vp8_swap_yv12_buffer` realizes
that permutation by exchanging descriptor pointers, which is the
encoder's chosen representation. The decoder reaches the same
mathematical outcome by permuting integer indices into a fixed bank
(`onyxd_if.c`'s `ref_cnt_fb`), which adds reference counting at the
cost of one indirection. Either way, no pixels move; that is the
property worth remembering when reading the rest of the codec.
