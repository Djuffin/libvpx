# `vp8/common/quant_common.c` — Quantizer step-size lookup tables

## Role in the decoder

VP8 carries quantization in the bitstream as a single 7-bit *quantizer
index* (QI) in the range 0–127, optionally adjusted per-segment and
per-channel by signed deltas. The decoder, however, does not multiply
by a QI — it must multiply each dequantized coefficient by an integer
*step size* (the "dequant" value) in pixel-domain units. The mapping
from QI to step size is a fixed, normative pair of tables baked into
the VP8 specification (RFC 6386 §9.6, "Dequantization"). Those tables,
together with the six tiny accessor functions that index them, are the
entire content of this file.

There are six accessor functions because VP8 has three transform-coded
plane types (luma Y, chroma UV, and the Walsh-transformed DC plane
called Y2), and each has its own DC and AC step size. Each accessor
takes a QI and a signed delta, clamps the result to 0–127, indexes one
of the two underlying tables (`dc_qlookup` for DC, `ac_qlookup` for
AC), and applies the small per-channel correction that the standard
requires (Y2 DC is doubled, Y2 AC is multiplied by 155 %, UV DC is
clamped to 132, etc.). The result is the integer that the IDCT path
will multiply each dequantized coefficient by.

This file is shared between the encoder and the decoder. Within the
decoder-only build enumerated in `vp8_files.md`, it is called from
exactly one place: `vp8cx_init_de_quantizer()` in
`vp8/decoder/decodeframe.c`, which is invoked once per frame to
rebuild the `VP8_COMMON::{Y1,Y2,UV}dequant[QINDEX_RANGE][2]` cache
tables before the macroblock loop begins. Per-macroblock dequantization
then reads from that cache by `QIndex` (possibly altered by
segmentation) — see `vp8_mb_init_dequantizer()`, also in
`decodeframe.c`. The functions here are therefore on the *setup*
path, not the inner loop; they run 128 × 6 = 768 times per frame.

```
   bitstream QI ─┐
                 │   (per-channel
   delta_q  ────►├── + clamp + lookup) ─► step size (short) ─► dequant[QI][0|1]
   {0..127}      │                                              │
                 │                                              ▼
                 │                                       IDCT × coeff
```

The remainder of this document walks the file in source order: first
the two backing tables, then the six wrappers.

---

## The two backing tables

### `dc_qlookup[QINDEX_RANGE]` — DC step sizes

```c
static const int dc_qlookup[QINDEX_RANGE] = {
  4,   5,   6,   7,   8,   9,   10,  10,  11,  12,  13,  14,  15,  16,  17,
  17,  18,  19,  20,  20,  21,  21,  22,  22,  23,  23,  24,  25,  25,  26,
  ...
  138, 140, 143, 145, 148, 151, 154, 157,
};
```

**What.** A 128-entry table mapping each legal QI value (0..127, where
`QINDEX_RANGE = MAXQ + 1 = 128`; see `onyxc_int.h:32-34`) to the
integer DC step size that should be used for the luma plane and (with
later post-processing) for chroma. Values run from 4 at QI = 0 (highest
quality, finest quantization) to 157 at QI = 127 (lowest quality,
coarsest). Units are pixel-domain coefficient quanta — i.e., a
de-quantized coefficient equals the parsed coefficient times this
number.

**Why.** VP8 separates DC and AC step sizes because the human visual
system is more sensitive to errors in low spatial frequencies (DC and
near-DC) than in high. Quantizing DC less aggressively at any given QI
preserves overall brightness fidelity. Look at the slope of the two
tables: at QI = 127 the DC step is 157 but the AC step (below) is
284 — DC is quantized roughly 1.8× finer at the high-QI end. The exact
numbers are normative; encoders and decoders must agree on them
bit-for-bit, so they appear verbatim in RFC 6386 §9.6, "Dequantization
Tables" (also reproduced in libvpx's `vp9_quantize.c` ancestor and in
the VP8 reference decoder).

**Invariants.** The table is `static const`, has exactly
`QINDEX_RANGE == 128` entries, is monotonically non-decreasing, and
its values fit easily in a `short` (max 157). It is read-only and has
no thread-affinity concerns. The decoder never modifies it.

**How used.** All three "DC" wrappers (`vp8_dc_quant`,
`vp8_dc2quant`, `vp8_dc_uv_quant`) read from it after their clamp.
The wrappers handle the per-channel scaling.

### `ac_qlookup[QINDEX_RANGE]` — AC step sizes

```c
static const int ac_qlookup[QINDEX_RANGE] = {
  4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,
  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,
  ...
  249, 254, 259, 264, 269, 274, 279, 284,
};
```

**What.** A second 128-entry table giving the *AC* step size for each
QI. AC step sizes run from 4 at QI = 0 to 284 at QI = 127 — almost
twice as coarse as DC at the high end, as discussed above.

**Why.** Same rationale as `dc_qlookup`, except that AC is allowed to
be coarser. RFC 6386 §9.6 defines this table separately. Both tables
start at 4 because a step of 0 or 1 would be lossless or near-lossless
(no quantization at all) and was not deemed worth representing in the
7-bit index space.

**Invariants.** Same shape and properties as `dc_qlookup`: 128
entries, monotonic non-decreasing, `static const`, fits in `short`.

**How used.** All three "AC" wrappers (`vp8_ac_yquant`,
`vp8_ac2quant`, `vp8_ac_uv_quant`) read from it. The Y2 wrapper
additionally multiplies by 155 % (see `vp8_ac2quant` below).

---

## The clamping idiom

Every wrapper begins with the same five lines:

```c
QIndex = QIndex + Delta;

if (QIndex > 127) {
  QIndex = 127;
} else if (QIndex < 0) {
  QIndex = 0;
}
```

(`vp8_ac_yquant` is the lone exception — it has no `Delta` parameter
because the Y AC plane is the *reference* channel from which all other
deltas are measured; see below.)

The clamp is required by the standard. VP8 transmits the base QI as a
7-bit field, but each of the five derived channels carries an
*additional* signed 4-bit delta (with a sign bit, so the range is
roughly ±15). Segmentation can also alter the effective QI on a
per-macroblock basis. The sum of `base_qindex + delta + segment_delta`
can therefore overflow 0..127 in either direction. RFC 6386 §9.6
mandates that the index be saturated, not wrapped — clamping to 127 at
the top and to 0 at the bottom. The wrappers here implement that
clamp before the table lookup, guaranteeing in-bounds array access and
matching the spec's `clamp(QI, 0, 127)` semantics exactly.

Note that the post-clamp `QIndex` is recomputed locally; the caller's
QI is not mutated (the argument is passed by value).

---

## The six accessor wrappers

The six functions partition along two axes: **DC vs. AC** (which
underlying table) × **{Y, Y2, UV}** (which per-channel post-processing).
This gives the 2 × 3 = 6 entry points declared in `quant_common.h`.

The decoder calls all six in a single loop, once per frame, populating
the per-channel cache:

```c
/* decodeframe.c:46-54 */
for (Q = 0; Q < QINDEX_RANGE; ++Q) {
  pc->Y1dequant[Q][0] = (short)vp8_dc_quant   (Q, pc->y1dc_delta_q);
  pc->Y2dequant[Q][0] = (short)vp8_dc2quant   (Q, pc->y2dc_delta_q);
  pc->UVdequant[Q][0] = (short)vp8_dc_uv_quant(Q, pc->uvdc_delta_q);
  pc->Y1dequant[Q][1] = (short)vp8_ac_yquant  (Q);
  pc->Y2dequant[Q][1] = (short)vp8_ac2quant   (Q, pc->y2ac_delta_q);
  pc->UVdequant[Q][1] = (short)vp8_ac_uv_quant(Q, pc->uvac_delta_q);
}
```

Note the index convention: `[Q][0]` is DC, `[Q][1]` is AC. That
encoding is consumed by the IDCT path (`idct_blk.c`) and by
`vp8_mb_init_dequantizer()`, which broadcasts the AC value across all
non-zero positions of the 16-entry per-MB `dequant_y1[]`, `dequant_y2[]`,
`dequant_uv[]` arrays declared in `blockd.h:215-218`. DC sits alone at
position 0; positions 1..15 all get AC.

### `vp8_dc_quant(int QIndex, int Delta)` — luma Y DC

```c
int vp8_dc_quant(int QIndex, int Delta) {
  int retval;
  QIndex = QIndex + Delta;
  if (QIndex > 127) { QIndex = 127; }
  else if (QIndex < 0) { QIndex = 0; }
  retval = dc_qlookup[QIndex];
  return retval;
}
```

**What.** Returns the luma Y plane's DC step size: a plain lookup into
`dc_qlookup` after clamping. The `Delta` parameter is the
`y1dc_delta_q` value parsed from the frame header.

**Why.** Y is the "primary" channel. Its DC step is taken straight
from the table without any per-channel correction — every other DC
wrapper is defined in terms of this one with some adjustment.
RFC 6386 §9.6 specifies `y1dc_delta_q` as a signed delta added to
`y_ac_qi` (the base index) before the lookup.

**Invariants.** Result is in `[4, 157]` (the range of `dc_qlookup`).

**How used.** Populates `Y1dequant[Q][0]` for each Q.

### `vp8_dc2quant(int QIndex, int Delta)` — Y2 (Walsh) DC

```c
retval = dc_qlookup[QIndex] * 2;
```

**What.** Returns the DC step size for the **Y2** plane — the 4×4
plane of DC coefficients of the sixteen luma 4×4 blocks of a
macroblock, which VP8 transforms a *second* time with a 4×4
Walsh-Hadamard transform (RFC 6386 §13.3). The step size is exactly
`dc_qlookup[QI] * 2`. The `Delta` is the parsed `y2dc_delta_q`.

**Why the ×2?** The Walsh-Hadamard transform that produces Y2
coefficients is *not* normalized to unit gain — it scales the input
by a factor of 4 in each dimension (the forward transform sums four
values without dividing). The inverse transform partially undoes this
in `idctllm.c` (the WHT inverse shifts by 3), leaving an overall
residual factor of 2 between the Y2 coefficient domain and the Y4×4
DC domain. Doubling the dequant step on the wire is the standard's
way of compensating: it pre-multiplies the inverse-quantized
coefficient so the subsequent unweighted WHT and the eventual addition
of the WHT outputs as DC inputs to the 4×4 IDCTs end up at the right
magnitude. RFC 6386 §9.6 specifies the doubling explicitly; the libvpx
implementation mirrors it here. (Side note: the doubling implies that
the maximum Y2 DC step is 2 × 157 = 314, which still fits in a
`short`.)

**Invariants.** Result is in `[8, 314]`.

**How used.** Populates `Y2dequant[Q][0]`.

### `vp8_dc_uv_quant(int QIndex, int Delta)` — chroma UV DC

```c
retval = dc_qlookup[QIndex];
if (retval > 132) retval = 132;
```

**What.** Returns the chroma DC step size. Looks the QI up in
`dc_qlookup` and then clamps the *result* (not the index) to 132. The
`Delta` is `uvdc_delta_q`.

**Why the 132 ceiling?** Chroma quality matters less to perception
than luma, but only up to a point: pushing chroma DC step beyond ~132
introduces visible color bleed and chroma banding that becomes a
dominant artifact even when luma is acceptable. The VP8 designers
capped chroma DC quantization at the QI value whose `dc_qlookup`
entry is approximately 132 (which corresponds to roughly QI ≈ 117).
Above that, regardless of how much the encoder requested via QI plus
delta, the decoder will *use* a step of 132. This is a hard normative
clamp from RFC 6386 §9.6, and it asymmetrically caps coarseness only:
the floor of the table (4 at QI = 0) is unaffected.

**Invariants.** Result is in `[4, 132]`.

**How used.** Populates `UVdequant[Q][0]`.

### `vp8_ac_yquant(int QIndex)` — luma Y AC

```c
int vp8_ac_yquant(int QIndex) {
  int retval;
  if (QIndex > 127) { QIndex = 127; }
  else if (QIndex < 0) { QIndex = 0; }
  retval = ac_qlookup[QIndex];
  return retval;
}
```

**What.** Returns the luma Y plane's AC step size. Plain lookup into
`ac_qlookup` after clamping. Note: **no `Delta` parameter** — the Y
AC step is the reference channel.

**Why no delta?** The Y AC step is defined to be the *base* QI for
the frame; all other channels' deltas are signed offsets from this
base, parsed separately and applied by the other five wrappers above.
Equivalently: the value transmitted on the wire as `y_ac_qi` (RFC 6386
§9.6) is the QI, with no further adjustment. Were there a
`y1ac_delta_q` it would always be zero and add nothing — so VP8
defines the format such that the delta is implicit-zero and the
function takes no delta argument. This is why the call site is
`vp8_ac_yquant(Q)` and not `vp8_ac_yquant(Q, 0)`.

**Invariants.** Result is in `[4, 284]`.

**How used.** Populates `Y1dequant[Q][1]`. Also used by the encoder's
rate-distortion code (`rdopt.c`, `encodeframe.c`, `firstpass.c`) as a
proxy for "quantization strength."

### `vp8_ac2quant(int QIndex, int Delta)` — Y2 (Walsh) AC

```c
/* For all x in [0..284], x*155/100 is bitwise equal to (x*101581) >> 16.
 * The smallest precision for that is '(x*6349) >> 12' but 16 is a good
 * word size. */
retval = (ac_qlookup[QIndex] * 101581) >> 16;
if (retval < 8) retval = 8;
```

**What.** Returns the Y2 AC step size: `ac_qlookup[QI] * 155 / 100`,
floored at 8. `Delta` is `y2ac_delta_q`.

**Why ×155 %?** Same rationale as the ×2 on Y2 DC, but a different
constant. The Walsh-Hadamard inverse transform's AC components see a
different residual gain after the integer shifts in `idctllm.c`'s WHT
implementation, and RFC 6386 §9.6 fixes the appropriate compensation
at 155 % (i.e., `qY2_AC = qAC × 155 / 100`). The comment in the source
documents the *implementation* of this scale factor: rather than
divide by 100 (which is slow and could vary in rounding behavior
across platforms), libvpx replaces the operation with a 16-bit fixed
point multiply. The constant 101581 is chosen so that for every input
in the table's actual range (`x ∈ [4, 284]`),
`(x * 101581) >> 16 == x * 155 / 100` exactly. The smaller, 12-bit
form `(x * 6349) >> 12` would also work, but using a 16-bit shift is
preferred because it matches the native word size on common targets
and avoids carry/overflow surprises. This is a small, contained
performance/portability trick; the spec-defined value is 155 %.

**Why the floor at 8?** When QI is near 0, `ac_qlookup[QI]` is itself
4 or 5, and 4 × 1.55 = 6.2 rounds to 6 — but RFC 6386 requires the Y2
AC step to be at least 8. Without the floor, very low-QI streams
would produce dequant values that under-scale the WHT output, which
in turn would lose precision (a step of 1 means the IDCT has nothing
to work with after the WHT inverse shifts). The `if (retval < 8)
retval = 8;` enforces the standard's minimum.

**Invariants.** Result is in `[8, 440]` (since 284 × 155 / 100 = 440).

**How used.** Populates `Y2dequant[Q][1]`.

### `vp8_ac_uv_quant(int QIndex, int Delta)` — chroma UV AC

```c
retval = ac_qlookup[QIndex];
```

**What.** Returns the chroma AC step size: plain `ac_qlookup`
read-back after clamping. `Delta` is `uvac_delta_q`.

**Why no per-channel correction here?** Unlike chroma DC, the chroma
*AC* path has no spec-mandated clamp or scale. RFC 6386 §9.6 defines
`qUV_AC = ac_qlookup[clamp(QI + uvac_delta_q, 0, 127)]` — identical
in form to the Y AC equation. The only difference from `vp8_ac_yquant`
is that here the caller may supply a non-zero delta. Conceptually one
could remove this function entirely and call `vp8_ac_yquant(QIndex +
Delta)` after clamping; libvpx keeps it as a separate symbol for
parallelism with the other five and for symmetry in the cache-fill
loop above.

**Invariants.** Result is in `[4, 284]`.

**How used.** Populates `UVdequant[Q][1]`.

---

## Summary table

| Function              | Source table | Per-channel transform | Floor | Ceiling |
|-----------------------|--------------|-----------------------|-------|---------|
| `vp8_dc_quant`        | `dc_qlookup` | identity              | —     | —       |
| `vp8_dc2quant`        | `dc_qlookup` | × 2                   | —     | —       |
| `vp8_dc_uv_quant`     | `dc_qlookup` | identity              | —     | 132     |
| `vp8_ac_yquant`       | `ac_qlookup` | identity (no delta)   | —     | —       |
| `vp8_ac2quant`        | `ac_qlookup` | × 155 % (via shift)   | 8     | —       |
| `vp8_ac_uv_quant`     | `ac_qlookup` | identity              | —     | —       |

All six begin by clamping `QIndex + Delta` to `[0, 127]`. The clamps
on the *result* (`> 132` for chroma DC, `< 8` for Y2 AC) are in
addition to that. Every value returned fits comfortably in a `short`,
which is why the call sites in `decodeframe.c` cast the `int` return
to `short` before storing it in the `Y1dequant`/`Y2dequant`/`UVdequant`
arrays declared in `onyxc_int.h:65-67`.

This file is roughly 130 lines of code and a normative dataset; it
contains no logic that varies at runtime beyond the clamps. The
heavy lifting it enables happens downstream in `dequantize.c` and
`idct_blk.c`, which multiply the per-MB `xd->dequant_*[]` arrays
(populated from the cache built here) into the parsed coefficients
before transforming back to the pixel domain.
