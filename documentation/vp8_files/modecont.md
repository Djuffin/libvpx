# `vp8/common/modecont.c` — default mode-context probability table for inter MBs

This is one of the smallest files in the entire VP8 decoder: a single
translation unit holding *one* statically-initialised constant,
`vp8_mode_contexts[6][4]`. Its 24 byte-sized probability values feed
every macroblock-mode decision made for an inter-coded macroblock in
an inter (P) frame. Despite being just twenty-seven lines (license
header included), the table is load-bearing: it is the *only* source
of `mv_ref` probabilities the decoder ever sees, because VP8's bitstream
does not retransmit or update these values per frame. RFC 6386 §16.3
calls this exact mapping a *table indexed by the "near MV context"*.

The contents in full (lines 13-26):

```c
const int vp8_mode_contexts[6][4] = {
  { /* 0 */
    7, 1, 1, 143 },
  { /* 1 */
    14, 18, 14, 107 },
  { /* 2 */
    135, 64, 57, 68 },
  { /* 3 */
    60, 56, 128, 65 },
  { /* 4 */
    159, 134, 128, 34 },
  { /* 5 */
    234, 188, 128, 28 },
};
```

The header (`modecont.h:18`) merely re-exports it:

```c
extern const int vp8_mode_contexts[6][4];
```

The remainder of this document explains why the table has shape
[6][4], what each cell means, how the decoder converts spatial
neighbourhood data into an index pair `(row, column)`, and the path
back to the corresponding section of RFC 6386.

## Role in the decoder

A VP8 frame is either a **key frame** (intra only, RFC 6386 §10–11) or
an **inter frame** (P frame, RFC 6386 §16). On an inter frame, every
macroblock carries one of these *Y-modes* (RFC 6386 §16.2,
`blockd.h:69-76`):

```
DC_PRED, V_PRED, H_PRED, TM_PRED,       (intra Y modes — copied from the key-frame set)
NEARESTMV, NEARMV, ZEROMV, NEWMV, SPLITMV   (inter Y modes)
```

The macroblock first opts in or out of intra coding (via the
per-frame `prob_intra` byte transmitted in the frame header); having
chosen inter, it then chooses *which* of the five inter modes to use.
Because intra/inter is decided earlier in the same MB header, the
five-way inter choice is in practice a *four-way* one over
`{NEARESTMV, NEARMV, ZEROMV, NEWMV, SPLITMV}` minus what has already
been decoded — and that four-way decision is the one driven by this
file.

`vp8_mv_ref_tree` (defined in `entropymode.c:87-88`) gives the binary
tree the decoder walks:

```c
const vp8_tree_index vp8_mv_ref_tree[8] = { -ZEROMV,   2, -NEARESTMV, 4,
                                            -NEARMV,   6, -NEWMV,     -SPLITMV };
```

That is four internal nodes, four probabilities. Each probability is
read out of `vp8_mode_contexts` at decode time and fed to the bool
decoder. The four columns of the table correspond *one-to-one* with
the four split decisions in that tree, in the order:

| Column | Tree split                              | Outcome on bit 0 / bit 1                    |
|--------|------------------------------------------|---------------------------------------------|
| 0      | root                                    | 0 -> ZEROMV ; 1 -> next split (NEAREST/NEAR/NEW/SPLIT) |
| 1      | "NEAREST vs (NEAR or NEW or SPLIT)"     | 0 -> NEARESTMV ; 1 -> next split            |
| 2      | "NEAR vs (NEW or SPLIT)"                | 0 -> NEARMV ; 1 -> next split               |
| 3      | "NEW vs SPLIT"                          | 0 -> NEWMV ; 1 -> SPLITMV                   |

This mapping is what makes `vp8_mode_contexts` a 4-column table:
**one column per internal node of the mv_ref tree**, regardless of
context.

The six rows correspond to the six possible values of the *near-MV
neighbour score* — a small integer the decoder computes from the
above, left, and above-left macroblock neighbours. RFC 6386 §16.3 calls
this score the "near MV context"; libvpx calls it `near_mv_ref_ct[]`.
Because the per-column scoring caps at 5 (see "How the table is
indexed" below), exactly six rows are needed: `near_mv_ref_ct[i] in
[0..5]`.

The result, then, is a table whose layout is *not* "row = mode, column
= probability", but rather "**row = neighbourhood-strength bucket for
this column's decision, column = which mv_ref-tree split**". Every
column has its own independent context-to-probability curve, and the
four columns are looked up independently per macroblock.

## What each cell means quantitatively

Probabilities in libvpx are stored as `vp8_prob` — an 8-bit unsigned
quantity where `p/256` is the probability of the decoded bit being
**zero** (RFC 6386 §7.1, `vpx_dsp/prob.h`). The bool decoder
`vp8_read(bc, p)` returns `0` with probability `p/256`. Examining the
table with that convention in mind:

* **Column 0 — "is this MB ZEROMV?"** Row 0 holds 7 (≈ 2.7% chance of
  bit=0, i.e. ≈ 97% chance the MB is *not* zeromv). Row 5 holds 234
  (≈ 91.4% chance of bit=0, i.e. very likely ZEROMV). The
  monotonic-ascending pattern across rows 0..5 in column 0 reflects
  the intuition: *the stronger the surrounding evidence that nearby
  blocks moved together, the more likely THIS block is a stationary
  copy from the reference frame using a zero MV.* (More precisely:
  rows are ordered by the same `near_mv_ref_ct[0]` that already
  determined how much intra-coded neighbour weight surrounds the MB —
  see the indexing rule below.)

* **Column 1 — "is this MB NEARESTMV?"** Same monotonic increase
  (1 .. 188) — stronger neighbour-NEAREST evidence biases toward
  picking NEARESTMV outright instead of further refinement.

* **Column 2 — "is this MB NEARMV?"** Rises (1, 14, 57, 128, 128,
  128) and then plateaus at 128 (the neutral value, 50/50). Above
  row 2 the decoder neither prefers nor avoids NEARMV — the
  signalled bit is essentially uniform.

* **Column 3 — "is this MB NEWMV vs SPLITMV?"** *Decreasing* with
  row (143, 107, 68, 65, 34, 28). The row index here is wired to
  `cnt[CNT_SPLITMV]` (the count of neighbours that themselves used
  SPLITMV), so the more split-coded neighbours, the lower the
  probability of bit=0 — i.e. the **more likely the current MB is
  SPLITMV too**. This is the only column whose meaning makes the
  monotonic *descent* the "correct" intuition.

These exact byte values were not arrived at by training during normal
operation — they are bitstream constants frozen by RFC 6386 and never
adapted by the decoder. The matching encoder file would be free to
re-derive them empirically per corpus, but the decoder must use the
table verbatim or it will desynchronize from the encoder's bool
arithmetic.

### Invariants

* The table is `const`; nothing in the decoder modifies it.
* Shape is fixed at `[6][4]` (so the header declares it with explicit
  inner dimension `[4]`, allowing the compiler to lay out the
  pointer-arithmetic without runtime sizes).
* Values inhabit the range `[1, 255]`. The bool decoder requires
  `p != 0` and `p != 256`; libvpx clamps elsewhere but here the
  authoring of the table itself respects the invariant (smallest is
  1, largest is 234).
* The `int` storage type is wasteful (one byte would do), but
  cosmetic — the array is read once and the values immediately get
  passed to `vp8_read()` which takes an `int` anyway.

## How the table is indexed at decode time

Two call sites use `vp8_mode_contexts` — one in shared code (the
encoder path uses it as well) and one private to the decoder.

### Indirect via `vp8_mv_ref_probs` (findnearmv.c:150-159)

```c
vp8_prob *vp8_mv_ref_probs(vp8_prob p[VP8_MVREFS - 1],
                           const int near_mv_ref_ct[4]) {
  p[0] = vp8_mode_contexts[near_mv_ref_ct[0]][0];
  p[1] = vp8_mode_contexts[near_mv_ref_ct[1]][1];
  p[2] = vp8_mode_contexts[near_mv_ref_ct[2]][2];
  p[3] = vp8_mode_contexts[near_mv_ref_ct[3]][3];
  return p;
}
```

This is the *canonical* lookup pattern, and it is what reveals the
geometry of the table most clearly: column `j` is indexed by
neighbour-score slot `j`. Each of the four columns is looked up with
*its own row index*. The four scores `near_mv_ref_ct[0..3]` are
filled in by `vp8_find_near_mvs` (findnearmv.c:14-122) — they are not
indexed by mode but by the four enumerated *count slots* used during
neighbour processing:

```c
enum { CNT_INTRA, CNT_NEAREST, CNT_NEAR, CNT_SPLITMV };
```

So the row index for column 0 (ZEROMV probability) comes from
`CNT_INTRA` (how many neighbours were intra-coded); column 1
(NEARESTMV probability) from `CNT_NEAREST` (the dominant-neighbour-MV
weight); column 2 (NEARMV probability) from `CNT_NEAR` (the
second-dominant); column 3 (NEWMV/SPLITMV probability) from
`CNT_SPLITMV` (how many neighbours themselves used SPLITMV).

The accumulator update in `vp8_find_near_mvs` (findnearmv.c:39-89)
weights the *above* neighbour by 2, *left* by 2, *above-left* by 1.
Therefore the maximum any single `CNT_*` slot can reach is **5**
(0..2 from above, 0..2 from left, 0..1 from above-left), which is
exactly why the table has **6 rows** (indices 0..5).

### Direct inline (decodemv.c:363-408)

The decoder's `read_mb_modes_mv` does *not* call
`vp8_mv_ref_probs`. Instead it splices the same four lookups directly
into its tree walk, interleaved with the read calls, so that mid-walk
state (specifically the near/nearest swap on lines 369-378) can be
applied to the row index before column 2 is consulted:

```c
if (vp8_read(bc, vp8_mode_contexts[cnt[CNT_INTRA]][0])) {       // ZEROMV split
  /* ... swap CNT_NEAREST/CNT_NEAR if NEAR>NEAREST ... */
  if (vp8_read(bc, vp8_mode_contexts[cnt[CNT_NEAREST]][1])) {   // NEARESTMV split
    if (vp8_read(bc, vp8_mode_contexts[cnt[CNT_NEAR]][2])) {    // NEARMV split
      /* ... compute cnt[CNT_SPLITMV] from neighbour modes ... */
      if (vp8_read(bc, vp8_mode_contexts[cnt[CNT_SPLITMV]][3])) {
        /* SPLITMV decoded */
```

The inlining is performance-driven: the function consumes one bool
decode per node and the table lookups compile down to a single
`movzx` per probability. The semantics are identical to a
`vp8_mv_ref_probs`-then-`vp8_treed_read` pair; merging them just
avoids materialising the 4-byte temp.

There is a commented-out alternative in `findnearmv.c:156-157`
suggesting that column 3 was once experimented with as
`vp8_mode_contexts[ near_mv_ref_ct[1] + near_mv_ref_ct[2] +
near_mv_ref_ct[3] ][3]` — i.e. row indexed by the *combined*
non-intra-neighbour score. That experiment did not ship: the spec
freezes the slot-only indexing shown above.

## Trace to RFC 6386

RFC 6386 §16.3 ("Mode and Motion Vector Contexts") specifies exactly
this table. The RFC arrives at it via the `find_near_mvs` procedure
(RFC 6386 §16.4) which produces a 4-element "context" vector — there
called `cnt[]` — using the very same above/left/above-left weighting
of 2/2/1 and the same four indices `CNT_INTRA / CNT_NEAREST /
CNT_NEAR / CNT_SPLITMV`. The RFC then states (paraphrasing) that the
four probabilities driving the mv-ref tree are picked column-by-column
from a 6×4 array of constants. The body of that array, listed
verbatim in the RFC, matches `vp8_mode_contexts` byte-for-byte.

In other words, the entire content of this libvpx file is a literal
transcription of one normative table in the VP8 specification. There
is no derivation, no tuning, no per-frame adaptation, no
endian-dependence, and no SIMD-specialised copy elsewhere in the tree
— the table appears in source exactly once, lives in the read-only
section after compilation, and is read by both the decoder
(decodemv.c) and (via `vp8_mv_ref_probs`) the encoder.

## Why this layout, and not the alternative

A naïve reader would expect a "mode probability table" to be indexed
`[context][mode]` and contain `VP8_MVREFS = 5` columns — one
probability per mode. VP8 does not do that, for two reasons:

1. The mv_ref *tree* is binary, so only `VP8_MVREFS - 1 = 4`
   probabilities are needed (one per internal node), not five —
   hence 4 columns, not 5.
2. The four columns each have their own independent notion of
   "context strength", drawn from different fields of the
   neighbourhood-count vector (intra-ness for column 0,
   nearest-vote-weight for column 1, etc.). They share a row axis
   only by coincidence of dimension: each column gets its own row
   index when looked up. The 6×4 array is therefore really *four
   independent 6-entry context-probability curves*, laid out together
   for cache locality.

That column-independence is also what makes the table so resilient:
the encoder could in principle retune any one column without
disturbing the others, provided it ships matching values in its own
build. In practice neither libvpx nor the spec ever do.

## Build and link footprint

`modecont.c` is listed in the verified VP8-decoder file roster
(`documentation/vp8_files.md`, section A, `vp8/common/` line 59) and
its header in section B (`vp8/common/` line 149). The compilation
unit produces a single read-only symbol; no functions, no relocations
into writable memory, no RTCD entry. It is included unconditionally
by every libvpx build because the symbol is referenced both from the
encoder (which is gated out in a decoder-only build) and from
`findnearmv.c` / `decodemv.c` (which are part of the decoder). On a
decoder-only build the encoder's reference goes away but the decoder's
two callers keep `vp8_mode_contexts` live, so the file remains
mandatory.

The header `modecont.h` does nothing more than forward-declare the
array with the correct shape, allowing TU's that touch it to avoid
pulling in `entropy.h` transitively. (The `.c` file itself includes
`entropy.h` only for legacy reasons — none of `entropy.h`'s contents
are actually referenced here. The include is harmless; removing it
would be a one-line cleanup with no other consequence.)
