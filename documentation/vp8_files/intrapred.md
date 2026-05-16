# `vpx_dsp/intrapred.c` — generic C intra-prediction kernels

`intrapred.c` is the shared DSP library of reference C intra-prediction
kernels used by both VP9 and the minimal VP8 decoder build. Every
function it exports is a `_c` "reference" entry point registered through
the run-time CPU dispatch (RTCD) system: on a SIMD-capable target a NEON,
SSE2 or MSA variant is patched in at startup; on the `generic-gnu`
configuration these C functions are what actually runs. They are also
what the unit tests treat as ground truth.

The file is laid out in three layers, from inside out:

1. A small set of `static INLINE` *kernel templates* taking a `bs`
   (block-side) parameter at the bottom of the file. There is one
   template per directional / DC / TM mode. They handle the 8×8, 16×16
   and 32×32 sizes uniformly.
2. A handful of explicit, hand-unrolled 4×4 implementations
   (`vpx_he_predictor_4x4_c`, `vpx_ve_predictor_4x4_c`,
   `vpx_d207_predictor_4x4_c`, `vpx_d63_predictor_4x4_c`,
   `vpx_d63e_predictor_4x4_c`, `vpx_d45_predictor_4x4_c`,
   `vpx_d45e_predictor_4x4_c`, `vpx_d117_predictor_4x4_c`,
   `vpx_d135_predictor_4x4_c`, `vpx_d153_predictor_4x4_c`). These
   bypass the loop overhead of the templated bodies and, for `d45`,
   `d63` and `d135`, encode subtly different boundary behaviour that
   VP9's bitstream demands.
3. A pair of preprocessor generators — `intra_pred_sized(type, size)`
   and `intra_pred_allsizes(type)` / `intra_pred_no_4x4(type)` — that
   expand each templated body into the actual `vpx_<type>_predictor_NxN_c`
   externally linked symbols expected by `vpx_dsp_rtcd.h` (the table
   that the dispatcher reads).

Throughout the file three two-line helpers are used:

```c
#define DST(x, y)    dst[(x) + (y) * stride]
#define AVG3(a,b,c)  (((a) + 2 * (b) + (c) + 2) >> 2)
#define AVG2(a,b)    (((a) + (b) + 1) >> 1)
```

`AVG3` is a 1-2-1 binomial filter rounded to nearest (the standard
"smoothed three-tap" used everywhere in the VPx intra predictors);
`AVG2` is a rounded 2-tap average; `DST` is plain row-major addressing
into the destination plane.

## Role in the decoder

VP9 uses ten intra directional modes (the eight diagonals plus pure
vertical and pure horizontal), the DC family (`DC_PRED`,
`DC_LEFT_PRED`, `DC_TOP_PRED`, `DC_128_PRED`), and the
"TrueMotion / Paeth" mode (`TM_PRED`). Each mode exists in four block
sizes (4, 8, 16, 32), and when `CONFIG_VP9_HIGHBITDEPTH` is on, in a
parallel `uint16_t` family with a `bd` (bit-depth) parameter. The
combinatorial blow-up — modes × sizes × bit depths — is the reason this
file is so macro-heavy: the macros generate the dozens of leaf symbols
mechanically rather than by hand.

In the VP8-decoder slice (the build described in `vp8_files.md`) the
full table of symbols is *compiled* — VP8 cannot easily strip it because
`vpx_dsp_rtcd.h` declares all of them unconditionally — but only a
narrow subset is actually *called* at run time. VP8 uses only:

- `vpx_v_predictor_{4,8,16}x{4,8,16}_c`
- `vpx_h_predictor_{4,8,16}x{4,8,16}_c`
- `vpx_tm_predictor_{4,8,16}x{4,8,16}_c`
- `vpx_dc_predictor_{4,8,16}x{4,8,16}_c`
  and the three boundary variants `dc_top`, `dc_left`, `dc_128`
- For `B_PRED` sub-blocks: `vpx_ve_predictor_4x4_c`,
  `vpx_he_predictor_4x4_c`, plus the diagonal-4×4 family
  (`d45`, `d135`, `d117`, `d153`, `d207`, `d63`).

The 32×32 sizes and the high-bit-depth ladder are dead code in a VP8
build, but they are emitted so that the same object file can satisfy
both VP8 and VP9 linkage. See `reconintra.c` / `reconintra4x4.c` for the
VP8-side wrappers that map VP8's `MB_PREDICTION_MODE` and
`B_PREDICTION_MODE` enums onto the predictors above.

### Boundary samples — the abstract calling convention

Every predictor in this file is called with the same five arguments:

```c
void vpx_<mode>_predictor_<n>x<n>_c(uint8_t *dst,
                                    ptrdiff_t stride,
                                    const uint8_t *above,
                                    const uint8_t *left);
```

`dst` is the top-left of the block being synthesised; `stride` is the
distance (in bytes) between two destination rows. `above` and `left` are
pointers into the *reconstructed* neighbour samples. By convention:

- `above[0 .. bs-1]` are the `bs` samples immediately above the block,
  in raster left-to-right order.
- `above[bs .. 2*bs-1]` are the `bs` "above-right" samples (the row
  continuing past the top-right corner). Several diagonal modes
  (`d45`, `d63`) read into this range; the caller must populate it,
  even if at the frame boundary it duplicates `above[bs-1]`.
- `above[-1]` is the single sample at the top-left corner, sometimes
  called `X` or `TL` in the code.
- `left[0 .. bs-1]` are the `bs` samples immediately to the left of the
  block, top-to-bottom.

When a side of the block sits on the frame boundary, VP8 (see
`vp8/common/setupintrarecon.c`) writes the constants `127` above and
`129` left into that one-pixel strip at frame allocation time, so the
predictor can always read the samples unconditionally — there is no
"left available?" or "above available?" branch inside any of the
kernels in this file. The branching happens one layer up, in the
function-pointer table that selects which of `dc`, `dc_top`, `dc_left`
or `dc_128` to call.

The `(void)above` / `(void)left` casts at the top of many kernels
document which of the two boundaries the mode does not consume: e.g.
`d207_predictor` reads only `left[]`, `d63_predictor` only `above[]`,
`v_predictor` only `above[]`, `h_predictor` only `left[]`, and
`dc_128_predictor` neither.

## The diagonal kernel templates

The six diagonal modes — `d207`, `d45`, `d63`, `d117`, `d135`, `d153`
— share the same calling convention but synthesise pixels along six
different propagation angles. Their names encode the angle in degrees
measured counter-clockwise from horizontal-right: 207° points down-left
(into the picture from the left edge), 45° points up-right (along the
anti-diagonal of `above[]`), 63° is a shallow diagonal up-right, 117° is
a steep diagonal up-left, 135° is the main diagonal (down-right from
the top-left corner), and 153° is a shallow down-right from the left
edge. The kernels combine `AVG2` and `AVG3` smoothing along that
direction.

### `d207_predictor` — 207° (down-left, left-only)

```c
static INLINE void d207_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                                  const uint8_t *above, const uint8_t *left);
```

`d207` is the *only* directional kernel that reads neither `above[]` nor
`above[-1]`; it builds the block entirely from the left column. It first
fills column 0 with `AVG2(left[r], left[r+1])` (a 2-tap smoothing of
adjacent left samples), then column 1 with the 1-2-1-smoothed
`AVG3(left[r], left[r+1], left[r+2])`. Beyond column 1 there is no new
information: the kernel propagates the pattern along the 207° diagonal
by copying `dst[(r+1) * stride + c-2]` into `dst[r * stride + c]`, i.e.
shifting up by one row and left by two columns. The bottom row
degenerates to a constant `left[bs-1]`.

The *why*: this lets the kernel produce the entire block from one
column of neighbours, which is exactly the situation when the predictor
points towards the (unavailable) bottom-left of the frame. The output
preserves the smoothness of the left column without inventing samples
to the right.

Invariants: `dst[(bs-1) * stride + c] = left[bs-1]` for all `c`; the
2D pattern is shift-invariant along (+2 columns, -1 row).

### `d63_predictor` — 63° (above-only, shallow up-right)

```c
static INLINE void d63_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                                 const uint8_t *above, const uint8_t *left);
```

The dual of `d207`. The first two rows are filled directly from
`above[]`: row 0 with `AVG2(above[c], above[c+1])`, row 1 with
`AVG3(above[c], above[c+1], above[c+2])`. Then a tight loop copies row
`r/2` into row `r` and row `r/2 + 1` (the second-row pattern) into row
`r+1`, shifted left by `r/2` columns each iteration, and pads the right
edge with `above[bs-1]`. The `(r >> 1)` indexing realises the 63°
slope: every two rows the source row moves one column to the right.

`d63` reads up to `above[2*bs - 1]`, which is why the caller must
populate the above-right region.

### `d45_predictor` — 45° (above-only, exact anti-diagonal)

```c
static INLINE void d45_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                                 const uint8_t *above, const uint8_t *left);
```

The clean 45°-anti-diagonal case. Row 0 is the smoothed top boundary
`AVG3(above[x], above[x+1], above[x+2])` with the last column pinned to
`above_right = above[bs-1]`. Every subsequent row is row 0 shifted left
by one column, with the freshly exposed right-edge cells filled with
`above_right`. The implementation expresses this with `memcpy` of
contracting size and `memset` of growing size, which keeps the inner
loop branchless and SIMD-friendly:

```c
for (x = 1, size = bs - 2; x < bs; ++x, --size) {
  memcpy(dst, dst_row0 + x, size);
  memset(dst + size, above_right, x + 1);
  dst += stride;
}
```

The last `bs - 1` cells of the bottom-right triangle are all
`above_right`, matching the textbook 45° propagation when the source
strip is finite.

### `d117_predictor` — 117° (steep down-right, above + above-left + a little left)

```c
static INLINE void d117_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                                  const uint8_t *above, const uint8_t *left);
```

This mode propagates down-right at a steep angle (close to vertical).
The kernel separates the work into four phases: row 0 uses `AVG2` of
adjacent above samples (smoothed vertical), row 1 uses `AVG3` of three
consecutive above samples, the leftmost column for rows 2..bs-1 walks
into `left[]` with `AVG3` of three consecutive lefts, and the rest of
the block is filled with a "look two rows up, one column left" copy:
`dst[c] = dst[-2*stride + c - 1]`. The "-2 rows, -1 column" stride
encodes the 117° slope exactly.

Note the negative indices `above[-1]`, `above[c-1]`, `above[c-2]`,
`above[-1]` — the kernel relies on `above` and `left` being safely
readable one past their starts (the corner sample `above[-1]`) and the
caller having populated that overlap.

### `d135_predictor` — 135° (main diagonal, both boundaries)

```c
static INLINE void d135_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                                  const uint8_t *above, const uint8_t *left);
```

The principal-diagonal predictor. Rather than computing each cell
directly, the kernel first builds a 1-D `border[]` of length
`2 * bs - 1` running from the bottom-left of the left column,
counter-clockwise through the top-left corner, to the top-right of the
above row, with every sample replaced by its `AVG3` smoothing along
that contour. Then the block is laid out by:

```c
for (i = 0; i < bs; ++i)
  memcpy(dst + i * stride, border + bs - 1 - i, bs);
```

Row 0 reads `border[bs-1 .. 2*bs-2]` (the smoothed top), row `bs-1`
reads `border[0 .. bs-1]` (the smoothed left). The shift by one
`border` index per row is exactly the 135° propagation. The GCC 4.8-era
`#if` enlarges the on-stack `border[]` to 69 bytes (instead of the
arithmetically sufficient 63) to silence a spurious `-Warray-bounds`
warning; functionally either size works for `bs ≤ 32`.

The kernel needs *both* `above[]` and `left[]`, and `above[-1]` (the
top-left corner) appears explicitly in three contour entries.

### `d153_predictor` — 153° (shallow down-right, left-dominant)

```c
static INLINE void d153_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                                  const uint8_t *above, const uint8_t *left);
```

The mirror of `d117` across the diagonal: shallow down-right rather
than steep. Column 0 is `AVG2(left[r-1], left[r])` (smoothed left edge),
column 1 is `AVG3` of three consecutive lefts; row 0 (from column 2 on)
is `AVG3` of three consecutive aboves. The rest of the block is filled
by `dst[c] = dst[-stride + c - 2]` — look one row up, two columns left
— which is the inverse of the slope used in `d117`.

## Cardinal and DC kernel templates

### `v_predictor` — vertical (replicate above row)

```c
static INLINE void v_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                               const uint8_t *above, const uint8_t *left);
```

The simplest kernel in the file: `memcpy(dst, above, bs)` for every one
of the `bs` rows. No averaging; every column of the block becomes a
constant equal to the corresponding above sample. `left[]` is ignored.
This is what VP8 calls `V_PRED` at the MB level and `B_VE_PRED` at the
4×4 level (modulo a 1-2-1 smoothing in the 4×4 case; see
`vpx_ve_predictor_4x4_c` below).

### `h_predictor` — horizontal (replicate left column)

```c
static INLINE void h_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                               const uint8_t *above, const uint8_t *left);
```

Dual of `v_predictor`: each row of the destination is `memset` to
`left[r]`. `above[]` is ignored. Used as `H_PRED` / `B_HE_PRED` in
VP8 (again, with smoothing in the 4×4 case).

### `tm_predictor` — TrueMotion / Paeth (both boundaries + corner)

```c
static INLINE void tm_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                                const uint8_t *above, const uint8_t *left);
```

VP8/VP9's only directionally-blended mode that uses the top-left corner
sample arithmetically:

```c
dst[r][c] = clip_pixel(left[r] + above[c] - above[-1]);
```

The intuition is that `above[-1]` is the "base" colour shared by both
edges; `above[c] - above[-1]` is the column's deviation from the base;
`left[r]` carries the per-row offset. Adding the two and re-adding the
base gives an estimate of `dst[r][c]` that is exact for any image whose
samples decompose additively as row + column. The clip to `[0, 255]`
is what makes this "TrueMotion" rather than pure Paeth, and is the
reason it must be a per-pixel kernel rather than a `memcpy`.

### The DC family

VP8/VP9 use four DC variants depending on which boundaries are
available. All four fill the destination with a single constant; they
differ only in how that constant is computed:

```c
static INLINE void dc_128_predictor (…)  { value = 128 }
static INLINE void dc_left_predictor(…)  { value = round(mean(left)) }
static INLINE void dc_top_predictor (…)  { value = round(mean(above)) }
static INLINE void dc_predictor     (…)  { value = round(mean(above ++ left)) }
```

All four end in the same `for (r = 0; r < bs; r++) memset(dst, v, bs)`
loop. The rounding offset `(bs >> 1)` (or `(count >> 1)` for the
two-sided mean) implements round-half-up division by the population
size.

The reason VP8 has four variants is that at the picture's top-left
corner *neither* boundary is real, at the top edge only `left[]` is, at
the left edge only `above[]` is, and elsewhere both are. The caller —
not the kernel — picks which of the four to invoke based on the MB's
position; the kernels themselves never inspect availability. `dc_128`
uses the neutral grey 128, which falls out arithmetically when both
edges are the synthetic `127`/`129` boundary samples
(`(127+129)/2 = 128`).

## The hand-unrolled 4×4 entry points

The 4×4 sizes of `he`, `ve`, `d207`, `d63`, `d63e`, `d45`, `d45e`,
`d117`, `d135`, `d153` are defined directly as `vpx_..._predictor_4x4_c`
rather than via `intra_pred_sized`. There are three reasons.

1. *Performance.* At 4×4, the inner loop of a generic templated body
   is only 16 destination cells; the loop and index-arithmetic
   overhead dominate the useful work. A flat unroll using the `DST(x,y)`
   macro is both smaller and easier for the C compiler to schedule.
2. *VP9 vs VP8 boundary semantics.* `vpx_d45_predictor_4x4_c`,
   `vpx_d63_predictor_4x4_c` and `vpx_d135_predictor_4x4_c` include
   the comments `// differs from vp8`. VP9 has a slightly different
   treatment of the right and bottom edges of these 4×4 directional
   modes than VP8 does; the parallel `_e` variants
   (`vpx_d45e_predictor_4x4_c`, `vpx_d63e_predictor_4x4_c`) supply the
   *VP8*-compatible behaviour (the `e` stands for "edge-extended"). The
   8×8 / 16×16 / 32×32 templated `d45` / `d63` bodies, by contrast,
   are VP9-only and so do not need the second flavour. The VP8-side
   wrapper in `vp8/common/reconintra4x4.c` chooses the right entry
   point.
3. *Direct addressability.* The 4×4 bodies use named locals — `A` =
   `above[0]`, `B` = `above[1]`, …, `I` = `left[0]`, `J` = `left[1]`,
   …, `X` = `above[-1]` — which match the letter conventions used in
   the VP8/VP9 specs and make these functions trivially diff-able
   against the standard.

### `vpx_he_predictor_4x4_c` and `vpx_ve_predictor_4x4_c`

The horizontal-edge and vertical-edge VP8 modes (`B_HE_PRED`,
`B_VE_PRED`). Unlike the plain `h_predictor` / `v_predictor` templates,
these *do* smooth the boundary with `AVG3` before replicating:

- `he` fills row `r` with `AVG3(left[r-1], left[r], left[r+1])`
  (folded at the bottom: row 3 uses `AVG3(K, L, L)`), then `memset`s
  the four columns of that row to the result.
- `ve` smooths the four above samples once with `AVG3` plus the
  top-left `H = above[-1]` and the right-extension `M = above[4]`,
  computes the four-cell top row, then `memcpy`s it down to rows 1–3.

The 1-2-1 smoothing is what visually distinguishes `B_HE_PRED` /
`B_VE_PRED` from MB-level `H_PRED` / `V_PRED`: at the larger sizes the
extra averaging is not used.

### The 4×4 diagonal entries

`vpx_d207_predictor_4x4_c`, `vpx_d63_predictor_4x4_c`,
`vpx_d63e_predictor_4x4_c`, `vpx_d45_predictor_4x4_c`,
`vpx_d45e_predictor_4x4_c`, `vpx_d117_predictor_4x4_c`,
`vpx_d135_predictor_4x4_c`, `vpx_d153_predictor_4x4_c` are
straight-line transcripts of the per-cell formulas of the corresponding
diagonal modes. The repeated assignments of the form
`DST(0, 0) = DST(2, 1) = AVG2(I, X)` express *both* that the cell value
is `AVG2(I, X)` and that the same value appears at multiple coordinates
of the block (the propagation along the mode's diagonal that the larger
templated bodies do with a `memcpy + stride` loop).

The two `_e` variants — `vpx_d63e_predictor_4x4_c` and
`vpx_d45e_predictor_4x4_c` — change only the last few `DST(...)`
assignments compared to their non-`_e` siblings. Those assignments
correspond to the right and bottom edges, which is where VP8 and VP9
disagree:

- `vpx_d45_predictor_4x4_c`:  `DST(3, 3) = H;`
- `vpx_d45e_predictor_4x4_c`: `DST(3, 3) = AVG3(G, H, H);`
- `vpx_d63_predictor_4x4_c`:  `DST(3, 2) = AVG2(E, F); DST(3, 3) = AVG3(E, F, G);`
- `vpx_d63e_predictor_4x4_c`: `DST(3, 2) = AVG3(E, F, G); DST(3, 3) = AVG3(F, G, H);`

VP9 reaches less far into the above-right strip than VP8 does and so
falls back to a constant edge; the `_e` versions extend the average
through one more sample.

## High-bit-depth duplicates (`#if CONFIG_VP9_HIGHBITDEPTH`)

When configured for 10- or 12-bit decoding, the entire ladder of
templates and 4×4 unrolls is duplicated with `highbd_` prefixes,
`uint16_t *` pointers, an additional `int bd` (bit depth) parameter,
and `vpx_memset16` in place of `memset`. The DC-128 variant
parameterises the constant: `vpx_memset16(dst, 128 << (bd - 8), bs)`
so that the neutral mid-grey scales correctly with bit depth (`512`
at 10-bit, `2048` at 12-bit). The `tm_predictor` clip becomes
`clip_pixel_highbd(..., bd)` to clip into `[0, (1<<bd) - 1]`.

Everything else is mechanically identical to the 8-bit code; the
duplication exists because (a) the public API uses different pointer
types for the two paths, and (b) the SIMD overrides also come in two
flavours, so the dispatch table must too.

In a VP8-decoder-only build with `CONFIG_VP9_HIGHBITDEPTH=0` (the
default for the minimal slice described in `vp8_files.md`), this entire
half of the file is `#if`'d out and never compiles.

## The macro generators

### `intra_pred_sized(type, size)`

```c
#define intra_pred_sized(type, size)                        \
  void vpx_##type##_predictor_##size##x##size##_c(          \
      uint8_t *dst, ptrdiff_t stride, const uint8_t *above, \
      const uint8_t *left) {                                \
    type##_predictor(dst, stride, size, above, left);       \
  }
```

This is the one-shot "wrapper factory" that materialises an
externally-linked symbol `vpx_<type>_predictor_<size>x<size>_c` and
forwards into the corresponding `<type>_predictor` static inline
template. The wrapper exists because the RTCD layer wants a fixed
signature per (mode, size) pair — no `int bs` parameter — so it can
store function pointers of a homogeneous type in
`vpx_dsp_rtcd.h`. The inline template, in contrast, takes `bs` so it
can be shared across sizes. The macro is therefore the glue: it
*pins* `bs` to a compile-time constant `size`, which lets the optimiser
fully unroll the inner loops of the template when it specialises.

The `INLINE` annotation on each template, combined with the fixed
`size` substitution in the wrapper, is what makes the generated
`vpx_v_predictor_8x8_c` essentially equivalent to a hand-written
8-row, 8-byte `memcpy` loop with no overhead.

### `intra_pred_highbd_sized(type, size)`

Same idea for the high-bit-depth side: emits
`vpx_highbd_<type>_predictor_<size>x<size>_c` forwarding into
`highbd_<type>_predictor`. Guarded by `#if CONFIG_VP9_HIGHBITDEPTH`.

### `intra_pred_allsizes(type)` and `intra_pred_no_4x4(type)`

```c
#define intra_pred_allsizes(type)   \
  intra_pred_sized(type, 4)         \
  intra_pred_sized(type, 8)         \
  intra_pred_sized(type, 16)        \
  intra_pred_sized(type, 32)        \
  …high-bit-depth siblings…

#define intra_pred_no_4x4(type)     \
  intra_pred_sized(type, 8)         \
  intra_pred_sized(type, 16)        \
  intra_pred_sized(type, 32)        \
  …high-bit-depth siblings…
```

`intra_pred_allsizes` is used for `v`, `h`, `tm`, `dc`, `dc_top`,
`dc_left`, `dc_128`: modes for which the templated body is the right
implementation at *every* size, 4×4 included. `intra_pred_no_4x4` is
used for the six directional modes `d207`, `d63`, `d45`, `d117`,
`d135`, `d153`, because at 4×4 the hand-unrolled and possibly
VP8-edge-extended versions earlier in the file already supply the
symbol. If `intra_pred_allsizes(d45)` were used instead, the linker
would see two definitions of `vpx_d45_predictor_4x4_c`.

The final block of the file is, accordingly, just thirteen macro
invocations:

```c
intra_pred_no_4x4(d207)
intra_pred_no_4x4(d63)
intra_pred_no_4x4(d45)
intra_pred_no_4x4(d117)
intra_pred_no_4x4(d135)
intra_pred_no_4x4(d153)
intra_pred_allsizes(v)
intra_pred_allsizes(h)
intra_pred_allsizes(tm)
intra_pred_allsizes(dc_128)
intra_pred_allsizes(dc_left)
intra_pred_allsizes(dc_top)
intra_pred_allsizes(dc)
```

These thirteen lines expand (with high-bit-depth on) to roughly
13 × 8 = 104 externally linked C functions, plus the ten hand-unrolled
4×4 entries earlier in the file. With high-bit-depth off, just over
half of those; with `CONFIG_VP9=0` and no SIMD, the VP8 decoder
*declares* all of them through `vpx_dsp_rtcd.h` (the RTCD header has no
notion of "which codec needs this") but only invokes a small VP8-mode
subset at runtime. The unused ones survive only as link-time symbols
and add a few kilobytes to the final binary; see
`vp8_files.md` § "Things you can DELETE entirely" for a discussion of
why this file is still kept in a stripped VP8-only build.

## A note on the absent `intrapred_common.h`

The task brief mentions `vpx_dsp/intrapred_common.h` as optional
reading. As of this tree no such file exists in `vpx_dsp/`; the
`AVG2`/`AVG3`/`DST` helpers and the per-mode templates all live
directly inside `intrapred.c`. In upstream VPx history a refactor split
the highbd halves out into a separate file, but the current revision
keeps everything in one translation unit — the only common header that
matters at compile time is `vpx_dsp/vpx_dsp_common.h` (for `clip_pixel`
/ `clip_pixel_highbd`) and the generated `vpx_dsp_rtcd.h` (for the
declarations the macros must match).
