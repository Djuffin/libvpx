# `vp8/common/blockd.c` — block-to-context index tables

`blockd.c` is a deliberately tiny translation unit. After the two lines of
`#include` it contributes nothing to the decoder but two 25-byte read-only
arrays:

```c
const unsigned char vp8_block2left [25] = { … };
const unsigned char vp8_block2above[25] = { … };
```

Yet these arrays are *load-bearing*: every coefficient block in every
macroblock is detokenised against an above/left entropy context whose
position is looked up through them, and they encode — once, as plain data
— the geometric layout of the 25 sub-blocks that VP8 carves a macroblock
into. The file's job is to *exist as the single definition site* for that
table so that both the encoder and the decoder link against the same
copy. Its companion header `blockd.h` does the much heavier work of
declaring `MACROBLOCKD`, `BLOCKD`, the mode enums, and the entropy
context types; this `.c` file only realises the two `extern` arrays
declared on lines 58–59 of that header.

## Role in the decoder

A VP8 macroblock is 16×16 luma plus two 8×8 chroma planes, but the
codec *always* works on 4×4 transform blocks. That gives 16 luma blocks
(indices 0..15, raster order within the MB), 4 U blocks (16..19), 4 V
blocks (20..23) and — when the MB uses the 16×16 second-order Walsh
transform path — one 4×4 "Y2" DC block at index 24. The number 25 in
both array sizes is exactly that: `16 + 4 + 4 + 1`.

VP8's residual entropy coder (RFC 6386 §13.3 "Token Probabilities" and
the calling convention in §13.2) selects a token-tree probability table
using a 2-bit context, formed from the "has at least one non-zero
coefficient" flag of the immediately above and immediately left
neighbouring 4×4 blocks *of the same plane and the same DCT-coefficient
type*. That neighbour flag is the `ENTROPY_CONTEXT`. To make the
decoder fast, libvpx does not walk the macroblock grid to find the
neighbour at decode time; it pre-flattens each plane's neighbour state
into a small linear array stored in `ENTROPY_CONTEXT_PLANES`
(`blockd.h:51`):

```c
typedef struct {
  ENTROPY_CONTEXT y1[4];   /* 4 cols of Y  */
  ENTROPY_CONTEXT u [2];   /* 2 cols of U  */
  ENTROPY_CONTEXT v [2];   /* 2 cols of V  */
  ENTROPY_CONTEXT y2;      /* 1 scalar Y2  */
} ENTROPY_CONTEXT_PLANES;
```

`above_context` and `left_context` on the `MACROBLOCKD` are pointers
into per-frame strips of these structures (one struct per MB column for
the above strip, one per MB row for the left strip). Within a single
`ENTROPY_CONTEXT_PLANES` the four luma slots map to the four *columns*
of the MB (for the above strip) or the four *rows* (for the left
strip), the two chroma slots cover the two columns/rows of the 2×2
chroma block grid, and the `y2` scalar covers the second-order block.

The two tables defined in this file are *the maps from a sub-block
index `b ∈ [0,24]` to the offset (in `ENTROPY_CONTEXT` units) inside an
`ENTROPY_CONTEXT_PLANES` structure that holds the corresponding
above-neighbour or left-neighbour flag*. Code that follows the
"canonical" calling convention writes:

```c
ENTROPY_CONTEXT *a = (ENTROPY_CONTEXT *)x->above_context + vp8_block2above[b];
ENTROPY_CONTEXT *l = (ENTROPY_CONTEXT *)x->left_context  + vp8_block2left [b];
```

— a single array lookup replaces a row/column computation. Every
encoder call-site listed in `vp8/encoder/encodemb.c`, `rdopt.c`, and
`tokenize.c` uses exactly this idiom. The minimal-decoder build does
not link any of those (`vp8/encoder/` is excluded per `vp8_files.md`,
section E), and `vp8/decoder/detokenize.c` inlines the bit-twiddling
equivalents directly:

```c
a = a_ctx + (i & 3);              /* luma column                */
l = l_ctx + ((i & 0xc) >> 2);     /* luma row                   */
```

That is what `vp8_block2above` and `vp8_block2left` compute for
`i ∈ [0,15]`; the decoder simply chose to emit the constant in code
rather than read the table. The tables still ship in the decoder
binary because they are unconditional non-static globals — the linker
keeps them in case another translation unit (any encoder or rate-control
helper not part of the minimal build) takes their address. They cost 50
bytes of `.rodata` and zero cycles.

The casts `(ENTROPY_CONTEXT *)x->above_context` and
`… x->left_context` are deliberate: the table indices treat the
`ENTROPY_CONTEXT_PLANES` struct as if it were a flat 9-byte
`ENTROPY_CONTEXT[9]` array laid out in the order `[Y0 Y1 Y2 Y3 U0 U1 V0
V1 Y2]`. Because the struct contains only `char`-typed members and is
declared in that exact order with no padding, the cast is well-defined
and the table offsets `0..8` map cleanly to the nine slots. This is
the silent invariant the tables encode.

## The two tables

### `vp8_block2left[25]`

```c
const unsigned char vp8_block2left[25] = { 0, 0, 0, 0, 1, 1, 1, 1, 2,
                                           2, 2, 2, 3, 3, 3, 3, 4, 4,
                                           5, 5, 6, 6, 7, 7, 8 };
```

For sub-block `b`, `vp8_block2left[b]` is the byte offset (in
`ENTROPY_CONTEXT` units, i.e. bytes) from the start of the *left*
strip's `ENTROPY_CONTEXT_PLANES` to the entry that records whether the
block immediately to the left of `b`'s row had any non-zero
coefficients. Concretely:

* `b ∈ [0,15]` — luma. The four entries `b/4 = 0,1,2,3` index into the
  `y1[4]` array of the left-neighbour `ENTROPY_CONTEXT_PLANES`. Since
  the left strip stores per-row context (the leftmost column of the MB
  to our left, projected onto the rows of the *current* MB), the
  relevant slot is the *row* of block `b` within its MB: row
  `b >> 2 = b/4`. Hence the four-fold repeat `0 0 0 0 1 1 1 1 …`.
* `b ∈ [16,19]` — U plane (2×2 block grid). The two left entries are
  `u[0]` and `u[1]` at offsets `4` and `5`. Block `b`'s row within the
  chroma 2×2 grid is `((b-16) >> 1)`, giving the pattern `4,4,5,5`.
* `b ∈ [20,23]` — V plane. Offsets `6` and `7` in the planes struct.
  Same row-within-2×2 logic, pattern `6,6,7,7`.
* `b = 24` — the second-order Y2 DC block. There is only one such
  block per MB, so its left context is the single `y2` scalar at offset
  `8`.

The table is therefore the static answer to "given that the
`ENTROPY_CONTEXT_PLANES` layout is `[y1[0..3] u[0..1] v[0..1] y2]`,
which slot do I read when I need the left-neighbour flag for sub-block
`b`?". It is purely a function of the struct layout and the
sub-block-to-row mapping inside a macroblock — no probability or
bitstream data goes into it. The invariant is fragile only if
`ENTROPY_CONTEXT_PLANES`'s member order or padding changes; nothing
checks this at runtime.

### `vp8_block2above[25]`

```c
const unsigned char vp8_block2above[25] = { 0, 1, 2, 3, 0, 1, 2, 3, 0,
                                            1, 2, 3, 0, 1, 2, 3, 4, 5,
                                            4, 5, 6, 7, 6, 7, 8 };
```

Same scheme, but for the *above* strip. The above strip stores per-MB
context indexed by *column*, so the relevant slot for sub-block `b` is
the column of `b` within its MB: `b & 3` for luma, `(b-16) & 1` for
chroma. Hence:

* `b ∈ [0,15]` — luma. The repeating run `0 1 2 3 0 1 2 3 …` is exactly
  `b mod 4`, the column index inside the 4×4 luma sub-block grid.
* `b ∈ [16,19]` — U: column within the 2×2 U grid is `(b-16) & 1`,
  giving offsets `4, 5, 4, 5` into the `u[0..1]` part of the struct.
* `b ∈ [20,23]` — V: same logic on `v[0..1]`, offsets `6, 7, 6, 7`.
* `b = 24` — Y2 second-order block: offset `8`, the scalar `y2` slot.

Note the diagonal symmetry between the two tables: the luma block of
index `b` has row `b/4` (selecting its *left* neighbour) and column
`b mod 4` (selecting its *above* neighbour) — exactly what one would
write by hand for a 4×4 sub-grid stored in raster order. The tables
are a serialised form of that arithmetic, and the encoder paths in
`tokenize.c`, `encodemb.c` and `rdopt.c` use them in the canonical
`A + vp8_block2above[b], L + vp8_block2left[b]` shape directly visible
in those files.

### RFC 6386 origin

The RFC does not specify these tables verbatim — they are a libvpx
implementation device — but they correspond directly to the
"position-dependent context" rules in:

* §13.2 "Token-coding token-tree" — selection of the per-block
  coefficient probability table from above/left non-zero flags.
* §13.3 "Token probabilities" — definition of the
  `(plane_type, band, prev_coef_context)` triple under which token
  probabilities live.
* §12.1 "Macroblock structure" — fixes the 16 + 4 + 4 + 1 = 25 sub-block
  enumeration that the index `b` ranges over.

The values themselves are derivable arithmetically (`b mod 4`,
`b div 4`, with offsets for U/V/Y2); they are tabulated only for speed
and clarity at the call site.

## What is *not* in this file

The header `blockd.h` declares a great deal of additional machinery
that lives in other translation units:

* `MACROBLOCKD`, `BLOCKD`, `MODE_INFO`, `MB_MODE_INFO` — instantiated
  inside `VP8D_COMP` (see `onyxd_int.h`).
* `MB_PREDICTION_MODE`, `B_PREDICTION_MODE`, `MV_REFERENCE_FRAME`,
  `FRAME_TYPE` — bitstream enums used pervasively.
* `vp8_build_block_doffsets()` and `vp8_setup_block_dptrs()` — pointer
  setup for the 25 `BLOCKD` entries; *defined in*
  `vp8/common/mbpitch.c`, not here, despite being declared at the
  bottom of `blockd.h`.

`blockd.c` itself contains no functions, no `#ifdef`s, no static state,
and no initialiser logic. It is essentially a data file written in C.
That austerity is the right architectural choice for a piece of code
that defines a fundamental coordinate convention used by both the
encoder and decoder: there is nothing here to get wrong at runtime, and
the layout assumption is documented by sitting one screen away from the
struct it indexes.
