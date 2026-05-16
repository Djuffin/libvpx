# `vp8/common/entropymv.c` — Default MV-component probability tables

This file is one of the smallest in the libvpx VP8 codec — fewer than
fifty lines once the boilerplate is stripped — and it contains nothing
but data: two pairs of constant probability vectors that the arithmetic
decoder consults whenever it has to read a motion-vector component out
of an inter-frame bitstream. Despite its size it is load-bearing. Every
non-key frame begins with an optional MV-context update step, and that
step is parsed against the probabilities defined here; every motion
vector reconstructed from the wire is then decoded against the table
that step has just (re)populated. If either table were wrong the decoder
would silently produce gibberish motion fields.

The tables are direct transcriptions of the constants given in
[RFC 6386 §17.2](../rfc6386.txt) (the VP8 specification, "Probability
Updates" and "Motion Vector Decoding"). The file ships only the *default*
values; per-frame updates are read by `read_mvcontexts()` in
`vp8/decoder/decodemv.c` and mutate `pbi->common.fc.mvc[]` in place. The
defaults remain immutable and serve two distinct purposes: they are the
initial state of a fresh decoder/key-frame reset, *and* they are the
prior against which on-the-wire updates are themselves coded.

## Role in the decoder

A VP8 motion vector is a pair (`row`, `col`) of signed `short` values
stored in quarter-pel units, but is coded on the wire in half-pel units
(see `read_mv()` in `decodemv.c`, where each decoded component is
doubled before being stored in the `MV` struct). Each component is
coded *independently*, with two distinct probability sets — one for row
deltas, one for column deltas — because horizontal and vertical motion
statistics in natural video differ markedly (panning, scanlines, etc.).

The decoder therefore needs:

  1. A pair (row-context, column-context) of probability vectors used
     to actually decode an MV component. These live in
     `vp8_default_mv_context[2]`.
  2. A pair (row-context, column-context) of *update* probabilities,
     each entry telling the decoder how likely the corresponding entry
     of (1) is to be re-transmitted at the head of the frame. These
     live in `vp8_mv_update_probs[2]`.

Both pairs share the same `MV_CONTEXT` layout — a flat 19-entry
`vp8_prob` array indexed by the `mvpis_short`, `MVPsign`, `MVPshort`,
`MVPbits` enum from `entropymv.h`. That uniformity is what makes
`read_mvcontexts()` in `decodemv.c` able to walk both arrays in
lockstep:

```c
const vp8_prob *up = vp8_mv_update_probs[i].prob;
vp8_prob *p = (vp8_prob *)(mvc + i);
vp8_prob *const pstop = p + MVPcount;
do {
  if (vp8_read(bc, *up++)) {              /* update flag */
    const vp8_prob x = (vp8_prob)vp8_read_literal(bc, 7);
    *p = x ? x << 1 : 1;                  /* new 8-bit prob */
  }
} while (++p < pstop);
```

That is the only consumer of `vp8_mv_update_probs`. The result — the
mutated `mvc[]` — is then consumed by `read_mvcomponent()` (also in
`decodemv.c`), which is the inner loop that turns wire bits into a
signed magnitude.

## The `MV_CONTEXT` layout

Before looking at the numbers it pays to be precise about what each
entry of the 19-element vector *means*. The layout is fixed by the
enum at `entropymv.h:30–37`:

```c
mvpis_short = 0,                          /* p[0]:    is short? */
MVPsign,                                  /* p[1]:    sign of non-zero */
MVPshort,                                 /* p[2..8]: short-tree branches */
MVPbits = MVPshort + mvnum_short - 1,     /* p[9..18]: long-bit probs */
MVPcount = MVPbits + mvlong_width         /* total = 19 */
```

with `mvnum_short = 8` and `mvlong_width = 10`. Concretely:

| Index    | Symbolic name      | Meaning                                                            |
|----------|--------------------|--------------------------------------------------------------------|
| 0        | `mvpis_short`      | P(magnitude ≥ 8). 0 → "short", read 3-bit tree; 1 → "long".        |
| 1        | `MVPsign`          | P(sign bit = 1), read only when magnitude ≠ 0.                     |
| 2..8     | `MVPshort + 0..6`  | Seven internal nodes of the 8-leaf `vp8_small_mvtree`.             |
| 9..11    | `MVPbits + 0..2`   | Long-MV bits 0, 1, 2.                                              |
| 12       | `MVPbits + 3`      | Long-MV "bit 3 implicit?" probability — the trick described below. |
| 13..18   | `MVPbits + 4..9`   | Long-MV bits 4, 5, 6, 7, 8, 9 (the upper magnitude bits).          |

The struct itself is declared in `entropymv.h:39–41`:

```c
typedef struct mv_context {
  vp8_prob prob[MVPcount];  /* often come in row, col pairs */
} MV_CONTEXT;
```

Note the comment: every place in the codec that uses an `MV_CONTEXT`
actually uses a `MV_CONTEXT[2]` — index 0 is the row component, index 1
is the column. This file's two extern arrays therefore each carry 38
probabilities in total (19 × 2).

## Sign vs. magnitude, short vs. long: the bitstream tree

`read_mvcomponent()` (decodemv.c:64–89) is the canonical reader for an
MV component and shows exactly how the table is walked. The flow is:

```
                       ┌── p[mvpis_short]
                       │
              ┌────────┴────────┐
        0 (short)              1 (long)
              │                    │
       vp8_small_mvtree           read 9 bits using
       (uses p[MVPshort+0..6])    p[MVPbits+0..9],
       → magnitude 0..7           with bit 3 implicit when
              │                    bits 4..9 are zero
              └────────┬───────────┘
                       │
                  magnitude ≠ 0?
                       │
              ┌────────┴────────┐
             yes               no
              │                 │
         p[MVPsign]           done
              │
        ± magnitude
```

This shape matches RFC 6386 §17.1 ("Coding of Each Component"). Two
features deserve commentary because the table layout is built around
them:

  * **Sign is coded last and only conditionally.** A zero MV component
    has no sign, so `MVPsign` is consulted only when the magnitude turns
    out to be non-zero. The default value `128 = vp8_prob_half` reflects
    the natural symmetry: positive and negative motion are *a priori*
    equally likely. The reference encoder typically leaves this entry
    untouched, and indeed both rows of `vp8_default_mv_context` carry
    128 in slot 1.

  * **Long MVs share the "bit 3" probability with the short-vs-long
    decision.** Reading the loop in `read_mvcomponent()`:

    ```c
    do { x += vp8_read(r, p[MVPbits + i]) << i; } while (++i < 3);
    i = mvlong_width - 1;
    do { x += vp8_read(r, p[MVPbits + i]) << i; } while (--i > 3);
    if (!(x & 0xFFF0) || vp8_read(r, p[MVPbits + 3])) x += 8;
    ```

    Bits 0–2 are read first (low end of the magnitude), then bits 9..4
    (high end). Bit 3 is *skipped* and read only at the end, and only
    conditionally: if any of the high bits is non-zero the decoder can
    skip transmitting bit 3 because in that case the magnitude is
    already ≥ 16 and the "+ 8" implied by bit 3 is implicit (the
    bitstream guarantees the long-path magnitude is at least 8). This
    saves one bit on the majority of long MVs while leaving the short
    path (magnitudes 0–7) handled entirely by `vp8_small_mvtree`. The
    indexing in the table is what it is precisely so that `p[MVPbits + 3]`
    is reachable as the same probability whether it gates an explicit
    bit-3 read or short-circuits one.

The 7-entry short-tree segment `p[MVPshort .. MVPshort+6]` parameterises
the binary tree `vp8_small_mvtree` defined in `entropymode.c:93`:

```c
const vp8_tree_index vp8_small_mvtree[14] = { 2,  8,  4,  6,  -0, -1, -2,
                                              -3, 10, 12, -4, -5, -6, -7 };
```

This is a balanced tree over the 8 leaves {0,1,2,3,4,5,6,7}; each pair
of `vp8_tree_index` slots is one internal node (per the contract in
`treecoder.h:37–43`). Seven internal nodes ⇒ seven probabilities ⇒
exactly the seven slots reserved at `MVPshort..MVPshort+6`.

## `vp8_default_mv_context` — the initial component probabilities

These are the probabilities that the freshly-constructed entropy
context starts out with, and that subsequent on-the-wire updates patch
into. They are also what a key-frame resets to. The array is declared
extern in `entropymv.h:43`. The body (`entropymv.c:30–47`) keeps the
row-component and column-component initialisers side by side so the
asymmetry between them is visible at a glance:

```c
const MV_CONTEXT vp8_default_mv_context[2] = {
  { { /* row */
      162,                                            /* is short */
      128,                                            /* sign */
      225, 146, 172, 147, 214,  39, 156,              /* short tree */
      128, 129, 132,  75, 145, 178, 206, 239, 254, 254 /* long bits */
  } },
  { { /* same for column */
      164,
      128,
      204, 170, 119, 235, 140, 230, 228,
      128, 130, 130,  74, 148, 180, 203, 236, 254, 254
  } }
};
```

Reading these in light of the layout above:

* **Slot 0 (`mvpis_short` = 162, 164).** A `vp8_prob` is interpreted as
  P(bit = 0) on the wire, with `255` meaning "almost certainly 0" and
  `0` meaning "almost certainly 1". Values near 160 say that the
  short-path branch is taken roughly 63% of the time by default. Most
  motion in natural content is small, so the short path dominates; row
  and column are nearly equiprobable here.

* **Slot 1 (`MVPsign` = 128, 128).** Exactly even odds, as discussed
  above. The encoder is free to update this if it sees a bias, but
  defaults to symmetric.

* **Slots 2..8 (short tree).** Seven probabilities, one per internal
  node of `vp8_small_mvtree`. These are *not* P(magnitude = k); they
  are P(go-left) at each branch of the tree. The asymmetry between row
  (225, 146, 172, 147, 214, 39, 156) and column (204, 170, 119, 235,
  140, 230, 228) reflects empirical statistics in the VP8 training
  corpus: vertical motion is dominated by very small magnitudes
  (P(left at root) = 225/256 ≈ 88% for the row tree), while horizontal
  motion has a heavier shoulder. The figures match RFC 6386 §17.2 and
  must not be changed independently of the spec.

* **Slots 9..18 (long magnitude bits).** Ten probabilities, one per
  binary digit of a 9-bit-plus-implicit-bit-3 long magnitude. They are
  *almost* monotone-increasing toward 254 — meaning "as the magnitude
  bit becomes more significant, it is more and more likely to be zero"
  — except for the dip at bit 3 (`p[MVPbits+3] = 75` for row, `74` for
  column). That dip is exactly what makes the "bit 3 sometimes
  implicit" trick worthwhile: when bits 4..9 are zero, the decoder
  reads this probability, and a low value (75/256 ≈ 29%) means bit 3
  is overwhelmingly likely to be set. The encoder exploited the
  asymmetry: long MVs are rare, and when they do occur their
  magnitudes cluster around 8–15 (i.e. bit 3 = 1, bits 4..9 = 0). The
  decoder must mirror this convention to the bit because the encoder
  used these exact priors when emitting the wire form.

* **Invariants.** Every entry is in `[1, 255]`; `vp8_prob` of 0 is
  illegal (it would cause the arithmetic decoder to divide by zero).
  The total count is `MVPcount = 19` per component, `2 × 19 = 38` per
  `MV_CONTEXT[2]`, fixed by the enum in `entropymv.h`. Any change to
  these enums would require regenerating the constants from the
  reference encoder and re-validating against the spec.

* **How used.** `vp8_init_frame()` /
  `vp8_setup_intra_recon_top_line()` and friends in `alloccommon.c` /
  `decodeframe.c` copy `vp8_default_mv_context` into `cm->fc.mvc[]`
  whenever the frame-context is reset (key frames, decoder
  initialisation). From that moment on, `read_mvcomponent()` reads
  every motion-vector component against `cm->fc.mvc[]`, *not* against
  the default array directly. The default array is only ever read, not
  written.

## `vp8_mv_update_probs` — probabilities for the per-frame updates

The second table (`entropymv.c:14–28`) governs the *meta*-channel: it
is the prior used to entropy-code the optional, per-frame update of
the table above. Inter-frames may carry, for each of the 38 MV-context
slots, either a single "no change" bit (cost: one fractional bit) or a
"no change" bit followed by a new 7-bit literal (cost: one fractional
bit + 7 bits). The update probability for slot `k` is exactly
`vp8_mv_update_probs[component].prob[k]`:

```c
const MV_CONTEXT vp8_mv_update_probs[2] = {
  { { 237, 246, 253, 253, 254, 254, 254, 254, 254,
      254, 254, 254, 254, 254, 250, 250, 252, 254, 254 } },
  { { 231, 243, 245, 253, 254, 254, 254, 254, 254,
      254, 254, 254, 254, 254, 251, 251, 254, 254, 254 } }
};
```

Every entry is in the range 231..254 — that is, very close to 255 —
which encodes the prior "this slot is overwhelmingly unlikely to need
updating in any given frame". This is precisely the design goal of the
update channel: in the common case the decoder reads 38 bits, all of
them zeros, decoded against probabilities very close to 1, so the
arithmetic-coded cost of "no updates this frame" is a small fraction
of a bit. When a slot *does* need updating — a scene change, a sudden
shift in motion statistics — the seven-bit literal carries the new
value (with the special case that the literal `0` is mapped to actual
probability `1`, never `0`, to satisfy the arithmetic decoder's
non-zero requirement; this is the `*p = x ? x << 1 : 1` line in
`read_mvcontexts()`).

* **Indexing.** Identical to the indexing of `vp8_default_mv_context`:
  slot `k` of the update array gates an update of slot `k` of the
  component array. Row and column have independent update probabilities
  for the same reason they have independent component probabilities —
  the statistics differ.

* **Notable values.** Slots 14, 15 (corresponding to `MVPbits+5` and
  `MVPbits+6`, i.e. the top-end long-MV bits) are slightly less than
  the bulk of the array (250–252 vs 254) for both row and column.
  These bits are the ones whose default values are themselves close to
  254 — saturated to "almost certainly zero" — so when very large
  motions do appear in the source, those entries are statistically the
  most likely to need recalibration. The skew is small but real, and
  copied verbatim from RFC 6386.

* **Invariants.** As with the default context, every entry is in
  `[1, 255]`; the array size is `MVPcount = 19`; the outer dimension
  is `2` (row, column). The order of read in `read_mvcontexts()` is
  fixed: outer loop over component `i ∈ {0, 1}`, inner loop over slot
  `p ∈ [0, MVPcount)` in ascending order — anything else would
  desynchronise the bitstream.

* **How used.** Read once, on every inter-frame, by `read_mvcontexts()`
  at `decodemv.c:96–112`, invoked from `mb_mode_mv_init()` at
  `decodemv.c:161`. Never copied, never mutated. Key frames skip the
  update step entirely (the calling site is gated on `frame_type !=
  KEY_FRAME`); they instead reset `cm->fc.mvc[]` to the defaults.

## Why this file is a separate translation unit

A reader familiar with the rest of `vp8/common/` will notice that
several other modules also carry tables of `vp8_prob`. Mode-context
probabilities live in `modecont.c`, coefficient-update probabilities
in `coefupdateprobs.h`, intra-mode probabilities in `entropymode.c`.
`entropymv.c` is intentionally kept separate from `entropymode.c`
because the MV statistical model is conceptually orthogonal: it deals
exclusively with inter-prediction motion, whereas `entropymode.c`
deals with prediction-mode selection. The header `entropymv.h`
correspondingly declares only what is needed to consume these two
tables and is included only by code that decodes (or in the encoder
build, encodes) motion vectors. Keeping the partition minimal means
that a fork stripping the MV decoder out — for, say, an
intra-frames-only application — would be a single-file change. In the
shipped decoder, however, every inter-frame depends on this file.
