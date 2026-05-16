# `vp8/common/findnearmv.c`

## Role in the decoder

VP8 codes inter-frame macroblock motion vectors *differentially*. The
bitstream never carries an absolute MV in isolation; instead, for every
inter-coded macroblock it carries one of four mode symbols — `NEARESTMV`,
`NEARMV`, `ZEROMV`, `NEWMV` (plus `SPLITMV`, the per-sub-block escape) —
and the actual MV is reconstructed from a *predictor* that both the
encoder and decoder derive deterministically from the already-decoded
neighborhood. `findnearmv.c` is the predictor-search machinery: given
the current macroblock and its three spatial neighbors (above, left,
above-left), it (1) collects the candidate MVs, (2) weights them by how
plausible each one is, (3) sorts the top two into a "nearest" and a
"near" slot, (4) chooses a single "best" MV used as the prediction
center for `NEWMV`, and (5) returns a four-element context vector
`cnt[]` whose entries directly index the entropy probability table used
to arithmetic-decode the mode symbol.

The score is computed with a fixed weighting that the VP8 specification
(RFC 6386, section 16) calls out explicitly:

> the above and left neighbors each contribute 2 to the score of their
> MV; the above-left neighbor contributes 1.

After scoring, the candidate with the highest count becomes `NEARESTMV`,
the runner-up becomes `NEARMV`, and `cnt[CNT_INTRA]` accumulates the
weight of neighbors that are *intra* (or zero-MV inter, which counts as
intra for prediction purposes). The four counts indexed `[INTRA,
NEAREST, NEAR, SPLITMV]` are then passed to `vp8_mv_ref_probs()` which
returns the four-element probability vector that drives the
`MV_REF`-tree bool decoder.

The decoder side (`vp8/decoder/decodemv.c`) inlines this same algorithm
verbatim around line 270; the routines in this file are the canonical,
encoder-shared implementation and the reference for the inlined copy.
Both must agree bit-exactly — that is the meaning of "predictor".

The file depends on a small surface of common types:

- `int_mv` / `MV` (from `mv.h`) — a 32-bit union of a `(row, col)` pair
  of `short`s, designed for one-instruction equality tests and copies.
- `MODE_INFO`, `MB_MODE_INFO`, `MV_REFERENCE_FRAME`, `SPLITMV`,
  `INTRA_FRAME`, `MB_MODE_COUNT` (from `blockd.h`) — the per-macroblock
  decoded state stored in the `mode_info` grid.
- `vp8_mode_contexts[6][4]` (from `modecont.c`/`.h`) — the 6×4 byte table
  of mode-tree probabilities, indexed by score and tree-node.

---

## The sub-block index table

### `vp8_mbsplit_offset[4][16]`

```c
const unsigned char vp8_mbsplit_offset[4][16] = {
  { 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 2, 8, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }
};
```

**What it is.** A lookup giving the 4×4-block index (0..15, raster
within a macroblock) where each of the *split sub-partitions* begins.
The four rows correspond to the four `SPLITMV` partitionings the VP8
bitstream supports:

- row 0 — `MB_2_HORIZ`  — two 16×8 halves (top half starts at block 0,
  bottom half starts at block 8).
- row 1 — `MB_2_VERT`   — two 8×16 halves (left at 0, right at 2).
- row 2 — `MB_4_QUART`  — four 8×8 quadrants (starts 0, 2, 8, 10).
- row 3 — `MB_16_4x4`   — all sixteen 4×4 blocks individually.

**Why.** When `SPLITMV` is signalled, each sub-partition carries its
own MV. The decoder needs to know, for sub-partition *k*, *which*
4×4 block to start writing the MV into; the unused trailing entries are
zeroed (defensively never read in those rows).

**Invariants.** Indices are in [0, 15]. The number of meaningful
entries per row is {2, 2, 4, 16}; the rest are padding.

**How used.** It is `extern`-declared in `findnearmv.h` and read by the
sub-block MV machinery in `decodemv.c` and the corresponding encoder
file. It lives here mostly out of historical co-location with the
near-MV predictor; it has nothing to do with the predictor itself.

---

## The neighbor sampling pattern

The predictor inspects exactly three neighbors of the current
macroblock:

```
       +---------+---------+
       |  AL     |  A      |
       | above-  | above   |
       |  left   |         |
       +---------+---------+
       |  L      | here    |
       | left    | current |
       +---------+---------+
```

Computed at function entry as pointer arithmetic over the
`mode_info_context` raster (whose stride is `mb_cols + 1` — the +1
makes the left-of-column-zero neighbor land on a sentinel row of
`INTRA_FRAME` records, so no boundary check is needed):

```c
const MODE_INFO *above     = here - xd->mode_info_stride;
const MODE_INFO *left      = here - 1;
const MODE_INFO *aboveleft = above - 1;
```

Crucially: VP8 considers **only the macroblock-level MV** of each
neighbor (`mbmi.mv`), even if that neighbor was itself coded with
`SPLITMV`. This is the comment "we only consider one 4×4 subblock from
each candidate 16×16 macroblock" at the top of `vp8_find_near_mvs` —
when a neighbor is `SPLITMV`, the encoder stores in `mbmi.mv` the MV
of the *last* sub-block (block 15), which is what gets sampled here.

The three neighbors contribute to scoring with fixed weights:

| Neighbor    | Weight |
|-------------|--------|
| above       | 2      |
| left        | 2      |
| above-left  | 1      |

---

## `vp8_find_near_mvs`

```c
void vp8_find_near_mvs(MACROBLOCKD *xd, const MODE_INFO *here,
                       int_mv *nearest, int_mv *nearby, int_mv *best_mv,
                       int near_mv_ref_cnts[4],
                       int refframe, int *ref_frame_sign_bias);
```

**What it does.** The core predictor. Walks the three neighbors,
accumulates a four-slot count array `near_mv_ref_cnts[]` indexed by

```c
enum { CNT_INTRA, CNT_NEAREST, CNT_NEAR, CNT_SPLITMV };
```

and a parallel four-slot MV array `near_mvs[]`, ranks the candidates,
and writes three MV outputs (`nearest`, `nearby`, `best_mv`) plus the
count vector (which the caller will feed straight into
`vp8_mv_ref_probs`).

**Why.** This is the function that implements the "predict the
inter-MB mode from spatial neighbors" rule of section 16 of RFC 6386.
The mode decoder needs both the probability context (to read the mode
bit-tree) and the candidate MVs themselves (to *use* once a mode is
chosen).

### The `cnt[]` array semantics

The decoder maintains two intertwined arrays of length 4:

```
near_mvs[CNT_INTRA]    ← unused as an MV (acts as a 0 slot used later
                          to hold the "best" MV for NEWMV addition)
near_mvs[CNT_NEAREST]  ← first distinct non-zero neighbor MV seen
near_mvs[CNT_NEAR]     ← second distinct non-zero neighbor MV seen
near_mvs[CNT_SPLITMV]  ← never actually filled with an MV; the slot
                          is repurposed as a count

cnt[CNT_INTRA]   = sum of weights of neighbors that are INTRA or
                   zero-MV inter (i.e. "no useful prediction")
cnt[CNT_NEAREST] = total weight backing the MV in near_mvs[CNT_NEAREST]
cnt[CNT_NEAR]    = total weight backing the MV in near_mvs[CNT_NEAR]
cnt[CNT_SPLITMV] = synthesised at the end from SPLITMV neighbor count
                   (NOT a "third MV" count, in spite of the field name)
```

The encoding is dense: `cntx` is a pointer that walks `near_mv_ref_cnts`
in step with `mv` walking `near_mvs`. When a neighbor's MV equals the
one already at `*mv`, only the count is bumped; when it differs, both
pointers advance and a new slot is opened.

### The reference-frame sign-bias re-orientation

A neighbor MV is only directly comparable to the current MB's prospective
MV if both point through the same reference frame *with the same
temporal direction*. VP8 keeps a per-reference-slot `sign_bias` flag
(set when the reference is temporally *after* the current frame, so the
MV must be interpreted with flipped sign). The helper

```c
static INLINE void mv_bias(int refmb_ref_frame_sign_bias, int refframe,
                           int_mv *mvp, const int *ref_frame_sign_bias) {
  if (refmb_ref_frame_sign_bias != ref_frame_sign_bias[refframe]) {
    mvp->as_mv.row *= -1;
    mvp->as_mv.col *= -1;
  }
}
```

(declared in `findnearmv.h`) is invoked on every neighbor MV before any
comparison: if the neighbor's reference has a different sign-bias than
the current MB's reference, the MV is *negated in place* so that, after
the call, both vectors live in the same temporal frame and equality
tests are meaningful.

This is why the encoder/decoder pass `refframe` and the array
`ref_frame_sign_bias` into the predictor: predictor sharing only makes
sense within a consistent sign-bias frame.

### The three neighbor-processing blocks

**Above.** If the neighbor is inter (`ref_frame != INTRA_FRAME`):

```c
if (above->mbmi.mv.as_int) {                      /* non-zero MV */
    (++mv)->as_int = above->mbmi.mv.as_int;        /* open NEAREST slot */
    mv_bias(...);                                  /* sign-flip if needed */
    ++cntx;                                        /* advance count pointer */
}
*cntx += 2;                                        /* add weight 2 */
```

If the above MB had a zero MV, no slot is opened (the count is still
bumped, but `cntx` is still pointing at `cnt[CNT_INTRA]` because it
hasn't been advanced — *so a zero-MV above MB contributes its weight to
the INTRA bucket*). This is the asymmetry that "intra" really means
"unhelpful for prediction": a zero MV pointing at the previous frame
is folded in with intra-coded neighbors.

**Left.** Identical to above, but with an extra equality test
*before* opening a slot:

```c
if (this_mv.as_int != mv->as_int) {
    (++mv)->as_int = this_mv.as_int;               /* open NEAR slot */
    ++cntx;
}
*cntx += 2;                                        /* +2 to whatever slot */
```

If the left MV (post sign-bias) equals what's already in the NEAREST
slot, then it adds its weight *to the existing slot*, not to a new one
— that is how identical neighbor MVs accumulate score. Also note the
explicit `else` arm: an intra/zero left contributes to `CNT_INTRA`
*indexed by name*, because at this point `cntx` may already be past it.

**Above-left.** Same shape, but the contribution is weight 1, not 2:

```c
*cntx += 1;     /* above-left weighs half as much */
```

### Post-pass adjustments

Two subtle fix-ups follow.

**Merge AL into NEAREST when there are three distinct neighbor MVs.**

```c
if (near_mv_ref_cnts[CNT_SPLITMV]) {
    if (mv->as_int == near_mvs[CNT_NEAREST].as_int)
        near_mv_ref_cnts[CNT_NEAREST] += 1;
}
```

This is conditioned on `cnt[CNT_SPLITMV]` being non-zero at this point
— but at this point `cnt[CNT_SPLITMV]` still holds the count that was
deposited there when the loop opened the *third* slot. (Remember `cntx`
walked off the end into slot 3 only if all three neighbors were inter
*and* all three MVs were distinct.) So the condition really means "we
ended up using all four slots", and what the code is doing is: if the
above-left MV (now sitting in slot 3) coincidentally matches the MV
sitting in slot 1 (NEAREST), credit NEAREST with the extra +1. (The
companion `decodemv.c` inlining does the same merge differently — it
ORs `cnt[CNT_SPLITMV] > 0` against the equality.)

**Recompute `cnt[CNT_SPLITMV]` as the SPLITMV-neighbor weight.**

```c
near_mv_ref_cnts[CNT_SPLITMV] =
    ((above->mbmi.mode == SPLITMV) + (left->mbmi.mode == SPLITMV)) * 2
    + (aboveleft->mbmi.mode == SPLITMV);
```

This *overwrites* the previous transient meaning of slot 3. From here
on, `cnt[CNT_SPLITMV]` is the same kind of score as the others, but
counting only neighbors that themselves chose `SPLITMV`. This is the
context that the mode decoder will use to decide whether the current MB
is `SPLITMV`.

### Sorting near vs. nearest

```c
if (near_mv_ref_cnts[CNT_NEAR] > near_mv_ref_cnts[CNT_NEAREST]) {
    /* swap slots 1 and 2 in both arrays */
}
```

The neighbors were visited above → left → above-left and slots were
opened in *first-seen* order. After scoring, the slot with the *higher*
count must be the NEAREST one (that is the definition of nearest:
"the MV that the most neighbor-weight votes for"). One unconditional
compare-and-swap restores that invariant.

### Choosing `best_mv`

```c
if (near_mv_ref_cnts[CNT_NEAREST] >= near_mv_ref_cnts[CNT_INTRA])
    near_mvs[CNT_INTRA] = near_mvs[CNT_NEAREST];
best_mv->as_int = near_mvs[0].as_int;
```

`near_mvs[CNT_INTRA]` (slot 0) was initialized to zero and never
otherwise touched. If the NEAREST slot *outweighs* the INTRA bucket, we
overwrite slot 0 with the NEAREST MV; otherwise slot 0 stays at zero.
`best_mv` is then read from slot 0. The semantic is: "the MV around
which `NEWMV` will be coded as a residual" — and it is the all-zeros
vector exactly when the neighborhood was dominated by intra/zero
neighbors.

### Outputs

```c
best_mv->as_int = near_mvs[0].as_int;
nearest->as_int = near_mvs[CNT_NEAREST].as_int;
nearby->as_int  = near_mvs[CNT_NEAR].as_int;
```

The caller now has, in canonical units within the current MB's
reference-sign-bias frame:

- the `NEAREST` candidate (used directly as the MV if mode == NEARESTMV),
- the `NEAR` candidate (used directly as the MV if mode == NEARMV),
- the `best_mv` predictor (used as the additive base for NEWMV), and
- the four-slot score vector to feed into the mode entropy decoder.

**Invariants.**
- `cnt[CNT_NEAREST] >= cnt[CNT_NEAR]` on exit (post-swap).
- `near_mvs[CNT_NEAREST]` and `near_mvs[CNT_NEAR]` are distinct MVs *if
  both slots were ever opened*; otherwise the unopened slot stays at
  zero.
- All MVs are expressed in the current MB's reference sign-bias
  convention.

---

## `invert_and_clamp_mvs`

```c
static void invert_and_clamp_mvs(int_mv *inv, int_mv *src, MACROBLOCKD *xd);
```

**What it does.** Negates `src` into `inv` and clamps both to the
current MB's allowable MV window.

**Why.** VP8 has only two *sign-bias classes* of references (those whose
sign-bias agrees with the current MB and those whose disagrees). The
predictor is computed once in the current MB's convention; if the encoder
later wants to test a candidate reference frame with the opposite
sign-bias, the cheapest thing to do is negate the predictors instead of
re-running the predictor search. `invert_and_clamp_mvs` is the
componentwise primitive that pairs up the two views.

**Invariants.** `inv` and `src` after the call are componentwise
negatives, both inside `[mb_to_left_edge - 128, mb_to_right_edge + 128]`
(and similarly for rows). The 128 (= `16 << 3` = 16 pixels in 1/8-pel
units) is the half-MB safety margin that prevents motion-comp reads
beyond a single MB outside the frame.

**How used.** Only called by `vp8_find_near_mvs_bias` below.

---

## `vp8_find_near_mvs_bias`

```c
int vp8_find_near_mvs_bias(MACROBLOCKD *xd, const MODE_INFO *here,
                           int_mv mode_mv_sb[2][MB_MODE_COUNT],
                           int_mv best_mv_sb[2], int cnt[4],
                           int refframe, int *ref_frame_sign_bias);
```

**What it does.** Calls `vp8_find_near_mvs` once for the current
`refframe`, deposits the three result MVs into the `mode_mv_sb[sign_bias]`
column at the `NEARESTMV` and `NEARMV` rows (and `best_mv_sb[sign_bias]`),
then fills the *other* sign-bias column with the negated/clamped versions
via `invert_and_clamp_mvs`. Returns the sign-bias that was used.

**Why.** The encoder's rate-distortion search evaluates each candidate
reference frame in turn but reuses the same neighborhood predictor
context (`cnt[]`); only the sign-bias-relative interpretation of the MV
changes. This wrapper computes both interpretations once so that the
RD loop can index `mode_mv_sb[sign_bias_of_candidate_ref][mode]` without
re-running the search.

**Invariants on output.**
- `mode_mv_sb[s][NEARESTMV]` and `mode_mv_sb[!s][NEARESTMV]` are
  componentwise negatives (likewise for NEARMV and `best_mv_sb`).
- All MVs are clamped to the current MB's allowable window.

**How used.** Encoder-only paths (`pickinter.c`, `rdopt.c`); the decoder
calls `vp8_find_near_mvs` directly because it only ever knows one
`refframe` at a time. The function is built into the decoder library
nonetheless because `findnearmv.c` is shared object code.

---

## `vp8_mv_ref_probs`

```c
vp8_prob *vp8_mv_ref_probs(vp8_prob p[VP8_MVREFS - 1],
                           const int near_mv_ref_ct[4]);
```

(`VP8_MVREFS == 5`, defined in `blockd.h` as
`1 + SPLITMV - NEARESTMV`, i.e. the count of inter modes
{NEARESTMV, NEARMV, ZEROMV, NEWMV, SPLITMV}. The probability vector has
4 entries because the mode-tree has 4 internal nodes.)

**What it does.** Maps the four-slot context vector produced by
`vp8_find_near_mvs` into a four-byte probability vector indexed by the
fixed 6×4 table:

```c
p[0] = vp8_mode_contexts[near_mv_ref_ct[0]][0];   /* INTRA-vs-inter node */
p[1] = vp8_mode_contexts[near_mv_ref_ct[1]][1];   /* NEAREST node */
p[2] = vp8_mode_contexts[near_mv_ref_ct[2]][2];   /* NEAR node */
p[3] = vp8_mode_contexts[near_mv_ref_ct[3]][3];   /* SPLIT node */
```

**Why.** This is the *binding* of context to entropy. The four count
slots are not arbitrary: each one selects the row of `vp8_mode_contexts`
to use for *the corresponding internal node* of the MV reference tree.
The tree has four internal binary decisions

```
                 [0]
                /   \
               /     \
            inter   intra (sub-tree: ZEROMV vs. ...)
             / \
          [1]   ...
           etc.
```

so node *i* gets its probability from row `cnt[i]` of the table — that
is the entire point of the layered count semantics. The commented-out
alternative at the bottom of the function is a historical experiment
(combining counts before lookup); the live code uses one count per
node.

**Invariants.** Each `cnt[i]` is in [0, 5] (the table has 6 rows) by
construction of `vp8_find_near_mvs` — the maximum any slot can collect
is `2 + 2 + 1 = 5`.

**How used.** Both the decoder (`vp8/decoder/decodemv.c`, where the
context table is indexed directly without going through this wrapper)
and the encoder bitstream writer (`vp8/encoder/bitstream.c`) use the
same `vp8_mode_contexts` table to encode and decode the four-bit mode
tree. The decoder reads the tree as four sequential `vp8_read` calls,
each gated by the row selected by its `cnt[i]`.

---

## Why this file is in `common/`

`findnearmv.c` is built into both the encoder and the decoder. Although
the *decoder* inlines a hand-tuned copy of the algorithm in
`decodemv.c` for speed, this canonical version is what the encoder
calls (often, per RD candidate) and what the inlined decoder version is
verified against. The single shared definition of the score weights
{2, 2, 1}, the sign-bias re-orientation, the slot-opening discipline,
and the entropy-context mapping is the contract that makes encoder and
decoder produce bit-identical predictions for every macroblock — which
is, ultimately, the only reason differential MV coding works at all.
