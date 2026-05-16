# `vp8/common/loopfilter_filters.c` — the inner-loop pixel kernels

## Role in the decoder

This file is the bottom of the VP8 deblocking stack. The driver in
`vp8/common/vp8_loopfilter.c` walks the decoded frame in macroblock raster
order, decides for each MB which of its four sides (top, left, plus three
internal vertical and three internal horizontal sub-block seams) need
filtering, and for each chosen edge calls one of the entry points exported
here:

```
vp8_loop_filter_mbv_c   /  vp8_loop_filter_mbh_c    — MB outer edge (vertical / horizontal)
vp8_loop_filter_bv_c    /  vp8_loop_filter_bh_c     — internal sub-block edges
vp8_loop_filter_simple_horizontal_edge_c
vp8_loop_filter_simple_vertical_edge_c              — the "simple" loop filter
vp8_loop_filter_bvs_c   /  vp8_loop_filter_bhs_c    — internal edges, simple variant
```

These names are the `_c` reference implementations that the runtime CPU
dispatcher (`vp8_rtcd.h`) hangs off of: when SIMD is enabled the same
function pointer (`vp8_loop_filter_mbv` etc.) is rebound to a SSE2/NEON/MSA
kernel; in a pure-C, `generic-gnu` decoder build the `_c` versions are what
actually run. The dispatch table is declared in
`vp8/common/rtcd_defs.pl`:

```
add_proto qw/void vp8_loop_filter_mbv/, "unsigned char *y_ptr, unsigned char *u_ptr,
            unsigned char *v_ptr, int y_stride, int uv_stride, struct loop_filter_info *lfi";
specialize qw/vp8_loop_filter_mbv neon dspr2 msa mmi lsx/, "$sse2_asm";
```

Everything below the entry points is `static` and exists solely to be
called by them. There are exactly three filter "shapes":

- a 4-tap inner filter (`vp8_filter`) used on internal sub-block edges of
  the normal loop filter, and on MB-edges when the inner pixels look
  high-variance (in which case the wider filter is suppressed and only
  the inner 4-tap part is applied);
- a 6-tap-input, 7-pixel-touching macroblock filter (`vp8_mbfilter`) used
  on MB edges in the normal loop filter when the inner pixels are smooth;
- a 4-pixel "simple" filter (`vp8_simple_filter`) which only touches
  `p1, p0, q0, q1` and is used end-to-end by the `SIMPLE_LOOPFILTER`
  type (luma only, no chroma, no wider variant).

All of this implements RFC 6386 §15 ("Loop Filter"). The mask-and-clamp
discipline below maps directly to the pseudo-code in §15.2 ("Common Filter
Computations"), §15.3 ("Normal Loop Filter"), and §15.4 ("Simple Loop
Filter") of that document. The arithmetic is normative: VP8 mandates this
exact integer rounding (the decoder is *not* allowed to pick a different
clamp boundary or rounding direction), because the deblocked pixels are
written back into the reference frame and feed the next inter-predicted
frame's pixel arithmetic.

The file uses three carefully chosen tricks throughout: (a) all per-pixel
arithmetic is done in `signed char` (range `-128..127`) by XOR'ing the
input bytes with `0x80`, so that overflow on a difference computation
stays within the signed-byte range and can be saturated cheaply; (b) all
masks are full-width (`0x00` or `0xFF`) so they can be `AND`'d with
filter values without a branch; (c) every intermediate that could exceed
`[-128, 127]` is funnelled through `vp8_signed_char_clamp` so that the
spec's saturating semantics are honored even on hosts whose `int` is
wider than 8 bits.

---

## Header layout and external surface

The file includes only `loopfilter.h` (for the public types) and
`onyxc_int.h` (for completeness — not strictly used in this file but
common to the driver). It then defines a one-line shorthand:

```c
typedef unsigned char uc;
```

That `uc` keeps the inner-filter signatures readable in 80-column
formatting and has no other purpose; treat it as `unsigned char`
everywhere.

The header `loopfilter.h` defines the `loop_filter_info` struct that the
driver fills out per (MB, edge) and hands to these kernels:

```c
typedef struct loop_filter_info {
  const unsigned char *mblim;    /* outer-MB-edge limit table */
  const unsigned char *blim;     /* internal-block-edge limit table */
  const unsigned char *lim;      /* inter-pixel limit table */
  const unsigned char *hev_thr;  /* high-edge-variance threshold table */
} loop_filter_info;
```

These four pointers are vectors of length `SIMD_WIDTH` (1 on ARM, 16 on
x86), all elements equal to the same scalar — so a SIMD kernel can load
them as a broadcast register. The C kernels in this file just read
element `[0]`. The actual numeric derivation of those values lives in
`vp8_loop_filter_update_sharpness` (vp8/common/vp8_loopfilter.c:49) and
is described in the technical overview, §11.2.

---

## The arithmetic helper

### `vp8_signed_char_clamp` — saturating `int → signed char`

```c
static signed char vp8_signed_char_clamp(int t) {
  t = (t < -128 ? -128 : t);
  t = (t > 127 ? 127 : t);
  return (signed char)t;
}
```

This is VP8's portable stand-in for the saturating 8-bit add/subtract
that x86 (`PADDSB`, `PSUBSB`), ARM (`SQADD`/`SQSUB` on byte lanes), and
MIPS DSP all provide as a single instruction. Every place where a
filter-difference could exceed signed-byte range is wrapped in a call
to this function. The promise it makes to its callers is:

- input: any `int` (the caller has already widened);
- output: the same value if it fits in `[-128, 127]`; otherwise `±128`/
  `±127`;
- **invariant**: the result is bit-identical to what a hardware
  saturating-byte ALU instruction would produce. Spec conformance
  depends on that: RFC 6386 §15.2 specifies the same `clamp255` /
  `clamp128` semantics.

Important consequence: because the SIMD code paths use the hardware
saturating instructions directly, the C and SIMD kernels must produce
the same pixel output bit-for-bit. The C path's correctness is verified
by libvpx's unit tests (`test/loopfilter_test.cc`).

---

## Edge classification: `mask` and `hev`

VP8 makes two independent yes/no decisions per edge: *should I filter
this edge at all?* (the `mask`), and *is the boundary itself a real
high-contrast feature I should preserve?* (the high-edge-variance
flag `hev`). Both decisions are represented as full-width `signed char`
values — `0xFF` means yes, `0x00` means no — so subsequent code can fold
them into the filter value with a single `&`.

### `vp8_filter_mask` — the per-edge "do we filter?" predicate

```c
static signed char vp8_filter_mask(uc limit, uc blimit, uc p3, uc p2, uc p1,
                                   uc p0, uc q0, uc q1, uc q2, uc q3) {
  signed char mask = 0;
  mask |= (abs(p3 - p2) > limit);
  mask |= (abs(p2 - p1) > limit);
  mask |= (abs(p1 - p0) > limit);
  mask |= (abs(q1 - q0) > limit);
  mask |= (abs(q2 - q1) > limit);
  mask |= (abs(q3 - q2) > limit);
  mask |= (abs(p0 - q0) * 2 + abs(p1 - q1) / 2 > blimit);
  return mask - 1;
}
```

The function inspects 8 pixels across the boundary (`p3 p2 p1 p0 | q0 q1
q2 q3`, with `p0`/`q0` being the two samples that straddle the edge) and
applies two rules, taken straight from RFC 6386 §15.2:

1. **Interior-flatness test** (`limit`): each of the six interior step
   differences `|pi - pi-1|`, `|qi - qi-1|` must be small. A large step
   somewhere on either side means the region is genuinely textured, not
   a smooth block with a blocking artifact, and we should leave it
   alone.
2. **Cross-boundary jump test** (`blimit`): the combined statistic
   `2·|p0-q0| + ⌊|p1-q1|/2⌋` must be below `blimit`. This is the
   spec's measure of how "blocking-like" the edge is, weighting the
   direct cross-boundary delta heavier than the next-out pair. Below the
   threshold we treat the jump as a coding artifact and filter; above
   it we treat it as a true image edge and leave it.

Note the careful encoding trick at the end. The body accumulates `1`
into `mask` whenever a test *fails* (i.e. the condition that should
*suppress* filtering is true). After the OR-chain, `mask` is `0` if every
test passed and `1` if at least one failed. Returning `mask - 1` then
gives `0 - 1 = -1 = 0xFF` ("filter") or `1 - 1 = 0` ("don't filter").
That branchless `0xFF` / `0x00` encoding is what lets the actual filter
kernels write `filter_value &= mask;` to gate themselves without
introducing per-pixel branches that would defeat SIMD parallelism.

The invariant assumed by every caller is that the inputs really are
adjacent pixels in scan order across the edge being tested. The caller
either steps `s` by `±p` for a horizontal edge or by `±1` for a vertical
edge; see the loops in §"Edge-walking wrappers" below.

### `vp8_hevmask` — high-edge-variance flag

```c
static signed char vp8_hevmask(uc thresh, uc p1, uc p0, uc q0, uc q1) {
  signed char hev = 0;
  hev |= (abs(p1 - p0) > thresh) * -1;
  hev |= (abs(q1 - q0) > thresh) * -1;
  return hev;
}
```

`hev` is `0xFF` if *either* side of the edge already has a sharp local
contrast (the step from `p1` to `p0` or from `q0` to `q1` exceeds
`thresh`), and `0x00` otherwise. The driver computes `thresh` from
`filter_level` plus a key-frame/inter-frame distinction
(`hev_thr_lut[frame_type][filter_level]` in
vp8/common/vp8_loopfilter.c:189, then indexed into `hev_thr[0..3]`).

`hev` does *not* gate whether filtering happens — `mask` does. `hev`
selects *which* of the two normal-filter variants to apply:

- when `hev = 0xFF` (a real local edge sits next to the boundary), the
  normal loop filter only adjusts the innermost pair `(p0, q0)` with the
  4-tap kernel — touching `p1`/`q1` would smear the legitimate sharp
  feature;
- when `hev = 0x00` (the neighbourhood is smooth), the normal filter is
  allowed to touch outwards (in `vp8_mbfilter`: `p2..q2`; in `vp8_filter`
  on an internal block edge: `p1..q1`).

This is RFC 6386 §15.3's `hev` selector, and it is the reason the normal
loop filter is described as adaptive: the *strength* depends on
`filter_level`, but the *footprint* depends on `hev`.

---

## The normal loop filter

The "normal" loop filter is the default and the only one that touches
chroma. It has two width variants, the 4-tap inner filter
(`vp8_filter`) and the 7-pixel macroblock filter (`vp8_mbfilter`). The
driver picks one or the other depending on whether the edge is *internal*
to an MB (use `vp8_filter` via `loop_filter_*_edge_c`) or is *between*
MBs (use `vp8_mbfilter` via `mbloop_filter_*_edge_c`).

### `vp8_filter` — the 4-tap inner filter

Used on internal block edges (where only 4 pixels around the edge can
be touched anyway, because the adjacent block's interior must be left
alone), and used on MB edges *when `hev` is set* (i.e. the wider 7-pixel
filter is suppressed and only this 4-tap part runs).

```c
static void vp8_filter(signed char mask, uc hev, uc *op1, uc *op0, uc *oq0,
                       uc *oq1) {
  signed char ps0, qs0;
  signed char ps1, qs1;
  signed char filter_value, Filter1, Filter2;
  signed char u;

  ps1 = (signed char)*op1 ^ 0x80;
  ps0 = (signed char)*op0 ^ 0x80;
  qs0 = (signed char)*oq0 ^ 0x80;
  qs1 = (signed char)*oq1 ^ 0x80;
  ...
```

The first four lines are the trademark *bias trick* of VP8's deblocker.
The pixels arrive as unsigned `[0, 255]`, but every subsequent step is
written as if they were signed `[-128, 127]`: a XOR with `0x80` (which
on `signed char` is bitwise-equivalent to subtracting 128) recenters
them at zero. The final stores XOR back with `0x80` to undo the bias.

The deep reason for this bias is hinted at by the file's own comments
("loop filter designed to work using chars so that we can make maximum
use of 8 bit simd instructions"). With pixels recentered, *all* the
interesting filter quantities — differences like `ps1 - qs1` and
weighted combinations like `3*(qs0 - ps0)` — naturally live near zero
and fit into a signed byte after one saturating clamp. On a SIMD
implementation this means a single `PSUBSB`/`PADDSB` does what a C
compiler would have to do in multiple widening steps. The math is the
common "`c = p - q + 128`" identity rewritten as the more symmetric
`c = (p^0x80) - (q^0x80)`; the two are arithmetically equal mod 256.

The body then assembles the filter value in stages:

```c
  /* add outer taps if we have high edge variance */
  filter_value = vp8_signed_char_clamp(ps1 - qs1);
  filter_value &= hev;

  /* inner taps */
  filter_value = vp8_signed_char_clamp(filter_value + 3 * (qs0 - ps0));
  filter_value &= mask;
```

The two contributions correspond exactly to RFC 6386 §15.3's
`filter = clip(ps1 - qs1) + 3·(qs0 - ps0)`. The outer-tap part
`(ps1 - qs1)` is only included when `hev` is set; the inner-tap part
`3·(qs0 - ps0)` is always included. After both are summed and clamped,
`filter_value &= mask` zeroes everything if this edge failed
`vp8_filter_mask` — making the whole subsequent kernel a no-op on
no-filter edges without a branch.

```c
  /* save bottom 3 bits so that we round one side +4 and the other +3
   * if it equals 4 we'll set it to adjust by -1 to account for the fact
   * we'd round it by 3 the other way
   */
  Filter1 = vp8_signed_char_clamp(filter_value + 4);
  Filter2 = vp8_signed_char_clamp(filter_value + 3);
  Filter1 >>= 3;
  Filter2 >>= 3;
  u = vp8_signed_char_clamp(qs0 - Filter1);
  *oq0 = u ^ 0x80;
  u = vp8_signed_char_clamp(ps0 + Filter2);
  *op0 = u ^ 0x80;
```

The filter value is divided by 8 (`>> 3`) before being subtracted from
`q0` and added to `p0`. The comment in the source explains the asymmetry
between `+4` and `+3`: VP8 requires that the two sides of the edge see
an effective change of *equal magnitude* but *opposite sign*, even when
the unrounded change is a half-integer. Rounding both sides the same way
would introduce a 1-LSB drift across the boundary; rounding `+4` on one
side and `+3` on the other averages out, except in the degenerate
`filter_value = 4` case where the comment notes the `-1` adjustment is
baked in. This is exactly RFC 6386 §15.2's `Filter1 = (filter+4)>>3,
Filter2 = (filter+3)>>3` distinction.

```c
  filter_value = Filter1;

  /* outer tap adjustments */
  filter_value += 1;
  filter_value >>= 1;
  filter_value &= ~hev;

  u = vp8_signed_char_clamp(qs1 - filter_value);
  *oq1 = u ^ 0x80;
  u = vp8_signed_char_clamp(ps1 + filter_value);
  *op1 = u ^ 0x80;
}
```

The outer-tap pixels `p1` and `q1` get an adjustment of `(Filter1+1)/2`
— half the magnitude applied to `p0`/`q0`, with rounding-half-up. The
`&= ~hev` is the other half of the `hev` selector: when `hev` is set
the outer adjustment is zeroed (we already took the `hev` branch by
including the outer taps in the *input* filter value; we must not also
modify the outer pixels in the output, or we'd double-count). When
`hev` is clear, the outer taps were excluded from `filter_value`, so
modifying `p1`/`q1` here actually broadens the filter's footprint.

The invariants for `vp8_filter` are:

- `op1, op0, oq0, oq1` must point to four adjacent pixels across the
  edge in scan order — adjacent in either rows (for horizontal edges)
  or columns (for vertical edges);
- `mask` must be either `0x00` or `0xFF` (it would be a logical bug to
  pass a value outside this set; the code AND's with it directly);
- `hev` must be either `0x00` or `0xFF` for the same reason.

### `vp8_mbfilter` — the wider 7-pixel macroblock filter

This is the wider filter that runs on edges *between* macroblocks (top
and left MB edges), and only when `hev = 0` so we have reason to believe
the boundary is smooth on both sides. It touches `p2`, `p1`, `p0`,
`q0`, `q1`, `q2` (six pixels), and reads but does not write `p3`/`q3`
via the mask test.

```c
static void vp8_mbfilter(signed char mask, uc hev, uc *op2, uc *op1, uc *op0,
                         uc *oq0, uc *oq1, uc *oq2) {
  signed char s, u;
  signed char filter_value, Filter1, Filter2;
  signed char ps2 = (signed char)*op2 ^ 0x80;
  ...
```

Again the six inputs are bias-shifted into signed range. The first
phase computes the same 4-tap filter value as `vp8_filter`, but here it
is always applied to `(p0, q0)`:

```c
  filter_value = vp8_signed_char_clamp(ps1 - qs1);
  filter_value = vp8_signed_char_clamp(filter_value + 3 * (qs0 - ps0));
  filter_value &= mask;

  Filter2 = filter_value;
  Filter2 &= hev;

  /* save bottom 3 bits so that we round one side +4 and the other +3 */
  Filter1 = vp8_signed_char_clamp(Filter2 + 4);
  Filter2 = vp8_signed_char_clamp(Filter2 + 3);
  Filter1 >>= 3;
  Filter2 >>= 3;
  qs0 = vp8_signed_char_clamp(qs0 - Filter1);
  ps0 = vp8_signed_char_clamp(ps0 + Filter2);
```

The trick here is that the *first* update of `(p0, q0)` is only done
when `hev` is set (note `Filter2 &= hev` — when `hev = 0`, `Filter2 = 0`
and the `±Filter1/Filter2` writes do nothing). When `hev` is clear, the
wide filter further down handles `p0`/`q0` instead, with smoother
weights. When `hev` is set, this is the *only* update — the wide filter
below will be entirely zeroed.

```c
  /* only apply wider filter if not high edge variance */
  filter_value &= ~hev;
  Filter2 = filter_value;

  /* roughly 3/7th difference across boundary */
  u = vp8_signed_char_clamp((63 + Filter2 * 27) >> 7);
  s = vp8_signed_char_clamp(qs0 - u);
  *oq0 = s ^ 0x80;
  s = vp8_signed_char_clamp(ps0 + u);
  *op0 = s ^ 0x80;

  /* roughly 2/7th difference across boundary */
  u = vp8_signed_char_clamp((63 + Filter2 * 18) >> 7);
  s = vp8_signed_char_clamp(qs1 - u);
  *oq1 = s ^ 0x80;
  s = vp8_signed_char_clamp(ps1 + u);
  *op1 = s ^ 0x80;

  /* roughly 1/7th difference across boundary */
  u = vp8_signed_char_clamp((63 + Filter2 * 9) >> 7);
  s = vp8_signed_char_clamp(qs2 - u);
  *oq2 = s ^ 0x80;
  s = vp8_signed_char_clamp(ps2 + u);
  *op2 = s ^ 0x80;
}
```

The wide filter applies a tapered correction `27 : 18 : 9` (the comments
describe this as "roughly 3/7th, 2/7th, 1/7th" — and `27/128`, `18/128`,
`9/128` are indeed close to `3/7`, `2/7`, `1/7`). The `+ 63` and `>> 7`
form an unsigned divide-by-128 with round-to-nearest; together with the
saturating clamps at every step this gives a well-defined, bit-exact
fixed-point approximation. The coefficient ratio `3:2:1` is what
distributes the boundary correction over three pixels on each side
instead of one.

The reason this filter is reserved for MB edges (and not used on
internal block edges) is that internal edges only have one block of
"flat" pixels available on each side — the next 4-pixel block over may
have its own residual we don't want to smear into. MB-edges are the
seams the human eye is most likely to perceive as blocking, so they get
the broader, smoother treatment when the surrounding pixels look flat
enough to support it.

Invariants for `vp8_mbfilter` mirror those of `vp8_filter`: six adjacent
pixels in scan order; `mask`, `hev` are full-width 0x00 / 0xFF; outputs
are stored back through the same pointers.

---

## The simple loop filter

VP8 supports a `SIMPLE_LOOPFILTER` mode (header bit, see RFC 6386 §15.1)
in which only luma is filtered, only the simplest 4-pixel filter is
applied, and the mask test is reduced. It exists for low-end profiles
where the encoder traded subjective quality for a near-zero deblocking
cost. The decoder is required to support both modes regardless.

### `vp8_simple_filter_mask` — the simplified predicate

```c
static signed char vp8_simple_filter_mask(uc blimit, uc p1, uc p0, uc q0,
                                          uc q1) {
  signed char mask = (abs(p0 - q0) * 2 + abs(p1 - q1) / 2 <= blimit) * -1;
  return mask;
}
```

Only the cross-boundary test remains — the interior-flatness checks of
the normal mask are dropped entirely. The condition is the *negation* of
the corresponding test in `vp8_filter_mask` (`<=` rather than `>`),
and the result is multiplied by `-1` so that "should filter" again
becomes `0xFF`. The funny-looking comment about `void limit` is
explained inline: declaring an unused parameter to silence a warning
caused a MSVC parse error, so the parameter is simply omitted. Callers
only pass `blimit`.

### `vp8_simple_filter` — the simplified filter

```c
static void vp8_simple_filter(signed char mask, uc *op1, uc *op0, uc *oq0,
                              uc *oq1) {
  signed char filter_value, Filter1, Filter2;
  signed char p1 = (signed char)*op1 ^ 0x80;
  signed char p0 = (signed char)*op0 ^ 0x80;
  signed char q0 = (signed char)*oq0 ^ 0x80;
  signed char q1 = (signed char)*oq1 ^ 0x80;
  signed char u;

  filter_value = vp8_signed_char_clamp(p1 - q1);
  filter_value = vp8_signed_char_clamp(filter_value + 3 * (q0 - p0));
  filter_value &= mask;

  /* save bottom 3 bits so that we round one side +4 and the other +3 */
  Filter1 = vp8_signed_char_clamp(filter_value + 4);
  Filter1 >>= 3;
  u = vp8_signed_char_clamp(q0 - Filter1);
  *oq0 = u ^ 0x80;

  Filter2 = vp8_signed_char_clamp(filter_value + 3);
  Filter2 >>= 3;
  u = vp8_signed_char_clamp(p0 + Filter2);
  *op0 = u ^ 0x80;
}
```

This is the same first-phase computation as `vp8_filter`, but the outer
taps (`p1`, `q1`) are read for the filter-value computation only and
never written. Note that there is also no `hev` selector — the
simple filter unconditionally uses the outer-tap contribution in
`filter_value`. The `+4`/`+3` rounding trick is the same.

The simple filter is luma-only by spec, and the driver enforces this:
its entry points (below) take only `y_ptr` and `y_stride`, never
chroma pointers.

---

## Edge-walking wrappers

Each of the four kernel-flavor / orientation combinations needs an inner
loop that walks `count * 8` pixels along the edge and re-runs the
appropriate per-position computation. All four wrappers are essentially
the same loop with different pointer arithmetic.

### `loop_filter_horizontal_edge_c` and `loop_filter_vertical_edge_c`

Both walk an 8- or 16-pixel edge (the `count * 8` controls length:
`count = 2` for a 16-luma edge, `count = 1` for an 8-chroma edge).

```c
static void loop_filter_horizontal_edge_c(unsigned char *s, int p, /* pitch */
                                          const unsigned char *blimit,
                                          const unsigned char *limit,
                                          const unsigned char *thresh,
                                          int count) {
  ...
  do {
    mask = vp8_filter_mask(limit[0], blimit[0], s[-4 * p], s[-3 * p], s[-2 * p],
                           s[-1 * p], s[0 * p], s[1 * p], s[2 * p], s[3 * p]);
    hev = vp8_hevmask(thresh[0], s[-2 * p], s[-1 * p], s[0 * p], s[1 * p]);
    vp8_filter(mask, hev, s - 2 * p, s - 1 * p, s, s + 1 * p);
    ++s;
  } while (++i < count * 8);
}
```

For the horizontal edge, `p` is the row pitch, and the pixels on the two
sides of the edge are `s[-1*p]` (= p0) and `s[0*p]` (= q0). Moving along
the edge is `++s` (one column to the right per iteration). The vertical
sibling is the same loop with `p` and `1` swapped: pixels are at `s[-1]`
and `s[0]`, the inter-pixel step is `s += p`.

`mask` and `hev` are recomputed per column because the surrounding
8 pixels change with each step; the limit/threshold *parameters* don't
change, but they are still read on every iteration via `limit[0]` /
`blimit[0]` / `thresh[0]` so that SIMD ports loading a broadcast vector
can use the same signature.

### `mbloop_filter_horizontal_edge_c` and `mbloop_filter_vertical_edge_c`

Identical structure, but the inner call is `vp8_mbfilter` and reaches
one pixel further out:

```c
vp8_mbfilter(mask, hev, s - 3 * p, s - 2 * p, s - 1 * p, s, s + 1 * p,
             s + 2 * p);
```

Same `count * 8` length convention.

---

## Public entry points

These functions are what `vp8_loopfilter.c` actually calls per edge,
via the RTCD trampoline. Their job is to coordinate three calls — luma
plus two chroma planes — and to step the luma pointer to each of the
three internal sub-block seams when needed.

### MB-edge entries: `vp8_loop_filter_mbh_c` / `vp8_loop_filter_mbv_c`

```c
void vp8_loop_filter_mbh_c(unsigned char *y_ptr, unsigned char *u_ptr,
                           unsigned char *v_ptr, int y_stride, int uv_stride,
                           loop_filter_info *lfi) {
  mbloop_filter_horizontal_edge_c(y_ptr, y_stride, lfi->mblim, lfi->lim,
                                  lfi->hev_thr, 2);
  if (u_ptr) {
    mbloop_filter_horizontal_edge_c(u_ptr, uv_stride, lfi->mblim, lfi->lim,
                                    lfi->hev_thr, 1);
  }
  if (v_ptr) {
    mbloop_filter_horizontal_edge_c(v_ptr, uv_stride, lfi->mblim, lfi->lim,
                                    lfi->hev_thr, 1);
  }
}
```

Luma gets `count = 2` (16 pixels of edge length); chroma gets `count = 1`
(8 pixels). The `if (u_ptr)` / `if (v_ptr)` guards let the driver pass
`NULL` for the chroma planes if it only wants to filter Y — used by
`vp8_loop_filter_frame_yonly` for example. Note that the *strength*
parameters used here are `lfi->mblim` (the wider MB-edge tolerance) and
`lfi->lim` (the interior tolerance). `vp8_loop_filter_mbv_c` is the
same with `mbloop_filter_vertical_edge_c`.

### Internal-edge entries: `vp8_loop_filter_bh_c` / `vp8_loop_filter_bv_c`

```c
void vp8_loop_filter_bh_c(unsigned char *y_ptr, unsigned char *u_ptr,
                          unsigned char *v_ptr, int y_stride, int uv_stride,
                          loop_filter_info *lfi) {
  loop_filter_horizontal_edge_c(y_ptr + 4 * y_stride, y_stride, lfi->blim,
                                lfi->lim, lfi->hev_thr, 2);
  loop_filter_horizontal_edge_c(y_ptr + 8 * y_stride, y_stride, lfi->blim,
                                lfi->lim, lfi->hev_thr, 2);
  loop_filter_horizontal_edge_c(y_ptr + 12 * y_stride, y_stride, lfi->blim,
                                lfi->lim, lfi->hev_thr, 2);
  if (u_ptr) {
    loop_filter_horizontal_edge_c(u_ptr + 4 * uv_stride, uv_stride, lfi->blim,
                                  lfi->lim, lfi->hev_thr, 1);
  }
  if (v_ptr) {
    loop_filter_horizontal_edge_c(v_ptr + 4 * uv_stride, uv_stride, lfi->blim,
                                  lfi->lim, lfi->hev_thr, 1);
  }
}
```

Three luma calls walk down the macroblock at rows 4, 8, and 12 — the
three horizontal seams between the four 4-pixel-tall sub-blocks of a
16x16 luma MB. Chroma is 8x8, so it has only *one* internal seam at row
4 (the other "seam" inside an 8x8 chroma block is between two 4x4 chroma
blocks, but the chroma plane is half-size, so a single 4-row offset
gets us there). The strength parameter is `lfi->blim` (the narrower
internal-edge tolerance), not `lfi->mblim`. `vp8_loop_filter_bv_c` is
the column-wise mirror.

### Simple-filter top-level entries

```c
void vp8_loop_filter_simple_horizontal_edge_c(unsigned char *y_ptr,
                                              int y_stride,
                                              const unsigned char *blimit) {
  ...
  do {
    mask = vp8_simple_filter_mask(blimit[0], y_ptr[-2 * y_stride],
                                  y_ptr[-1 * y_stride], y_ptr[0 * y_stride],
                                  y_ptr[1 * y_stride]);
    vp8_simple_filter(mask, y_ptr - 2 * y_stride, y_ptr - 1 * y_stride, y_ptr,
                      y_ptr + 1 * y_stride);
    ++y_ptr;
  } while (++i < 16);
}
```

These two functions (`_horizontal_edge_c` and `_vertical_edge_c`)
combine the role of a `count = 2` walker and an edge entry point: they
hard-code 16 iterations and only handle luma. The driver invokes them
directly for the simple-loop-filter MB-edge case. Note that the driver
also re-uses these functions as `vp8_loop_filter_simple_mbv` /
`vp8_loop_filter_simple_mbh` via aliasing in `rtcd_defs.pl`:

```
$vp8_loop_filter_simple_mbv_c = vp8_loop_filter_simple_vertical_edge_c;
$vp8_loop_filter_simple_mbh_c = vp8_loop_filter_simple_horizontal_edge_c;
```

The internal-seam entries `vp8_loop_filter_bhs_c` / `vp8_loop_filter_bvs_c`
just call the per-edge function three times at offsets 4, 8, 12 — same
shape as `_bh_c`/`_bv_c` but luma-only and without the chroma branches.

```c
void vp8_loop_filter_bhs_c(unsigned char *y_ptr, int y_stride,
                           const unsigned char *blimit) {
  vp8_loop_filter_simple_horizontal_edge_c(y_ptr + 4 * y_stride, y_stride,
                                           blimit);
  vp8_loop_filter_simple_horizontal_edge_c(y_ptr + 8 * y_stride, y_stride,
                                           blimit);
  vp8_loop_filter_simple_horizontal_edge_c(y_ptr + 12 * y_stride, y_stride,
                                           blimit);
}
```

---

## How a call site exercises this file

For a single normal-filter MB at `(mb_row, mb_col)` with all edges
active, `vp8_loop_filter_row_normal` (vp8_loopfilter.c:167) executes,
in order:

```
vp8_loop_filter_mbv(...)   →  this file: vp8_loop_filter_mbv_c
                                  → mbloop_filter_vertical_edge_c    (luma, count=2)
                                  → mbloop_filter_vertical_edge_c    (u,    count=1)
                                  → mbloop_filter_vertical_edge_c    (v,    count=1)

vp8_loop_filter_bv(...)    →  this file: vp8_loop_filter_bv_c
                                  → loop_filter_vertical_edge_c × 3  (luma rows 4,8,12)
                                  → loop_filter_vertical_edge_c      (u)
                                  → loop_filter_vertical_edge_c      (v)

vp8_loop_filter_mbh(...)   →  vp8_loop_filter_mbh_c
                                  → mbloop_filter_horizontal_edge_c × 3
vp8_loop_filter_bh(...)    →  vp8_loop_filter_bh_c
                                  → loop_filter_horizontal_edge_c × {3 luma, 1+1 chroma}
```

Each edge walker invokes `vp8_filter_mask` + `vp8_hevmask` + (one of
`vp8_filter`/`vp8_mbfilter`/`vp8_simple_filter`) per pixel position along
the edge. All values funnel through `vp8_signed_char_clamp` at every
step where overflow is possible, so that the C path produces output
byte-identical to the saturating-SIMD ports.

The order of vertical-then-horizontal across a frame is what makes this
an *in-loop* deblocker: pixels written by a horizontal-edge filter at
MB *m* are read by the vertical-edge filter at MB *m+1* on the next
row, and (more importantly) by the inter-prediction stage of the *next
frame*. Bit-exact behaviour of the kernels in this file is therefore
load-bearing for every subsequent inter-coded frame's reconstruction —
the reason every single saturate, round, and mask in here is normative.
