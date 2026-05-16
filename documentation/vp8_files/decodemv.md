# `vp8/decoder/decodemv.c` — parsing modes, segment IDs, and motion vectors

## Role in the decoder

After the uncompressed frame tag, the frame-level header, the segmentation
and loop-filter sub-headers, the quantizer indices, the ref-buffer refresh
flags and the coefficient-probability updates have all been consumed from
the first ("residual") partition by `decodeframe.c`, the decoder's
attention turns to the per-macroblock metadata: which reference frame
each MB uses, what motion vectors it carries, which intra mode it picks,
whether its coefficients are all-zero, and what segment it belongs to.
This file is the entire owner of that responsibility.

It corresponds to RFC 6386 §10 (segment IDs), §11 (key-frame MB
prediction), §16 (inter-frame MB prediction) and §17 (motion-vector
coding); the cross-reference table in `vp8_technical_overview.md`
(§16.16) maps each of these RFC sections back to the line ranges in this
single source file. The technical overview's call-tree diagram lists
the file's public entry point as one of the four leaves of
`vp8_decode_frame`:

```
vp8_decode_frame  →  vp8_decode_mode_mvs  (decodemv.c)
                  →  decode_mb_rows       (decodeframe.c)
                  →  vp8_decode_mb_tokens (detokenize.c)
                  →  vp8_loop_filter_frame
```

`vp8_decode_mode_mvs` runs to completion **before** any residual or
loop-filter work begins for the frame; downstream stages read the
populated `MODE_INFO` array but never write to it. This separation is
what allows multi-threaded builds to fan out token decoding and
reconstruction over MB-rows without coordination on the mode parser.

All reads happen out of `pbi->mbc[8]` — the ninth boolean decoder. The
first eight (`mbc[0..7]`) hold the per-token partitions consumed by
`detokenize.c`; `mbc[8]` is by convention the residual partition that
this file alone walks.

## What this file produces

For every macroblock the file fills a `MODE_INFO` record at
`pbi->common.mi[row*mode_info_stride + col]`:

```c
typedef struct modeinfo {
  MB_MODE_INFO mbmi;
  union b_mode_info bmi[16];
} MODE_INFO;
```
(blockd.h:156–159)

`mbmi` carries the MB-wide decisions (mode, uv_mode, ref_frame,
single MV, partitioning, mb_skip_coeff, need_to_clamp_mvs, segment_id,
is_4x4); `bmi[16]` carries the per-4×4 information that is only
populated when the MB is `B_PRED` (then each entry holds a
`B_PREDICTION_MODE`) or `SPLITMV` (then each entry holds an MV). The
union member `is_4x4` flags which interpretation is in force so that
downstream code (token decoding, intra reconstruction, MV-aware loop
filtering) can branch without re-parsing the mode.

## File layout, narrative order

The file is organized declaration-order from the leaves of the parse
tree (tiny tree-decoding wrappers) up to the top-level driver
`vp8_decode_mode_mvs`. The walkthrough below proceeds in the *opposite*
direction — top-down — because that mirrors how the bitstream actually
unfolds and how the technical overview presents the syntax.

## The top-level driver

### `void vp8_decode_mode_mvs(VP8D_COMP *pbi)`

This is the file's only externally-visible function (declared in
`decodemv.h:20`). It loops over every macroblock in raster order, in
two nested while-loops with `mb_row` outside and `mb_col` inside, and
calls `decode_mb_mode_mvs` once per MB.

Before entering the loop it calls `mb_mode_mv_init` to read the
"remaining frame header" fields (`mb_no_coeff_skip`, `prob_skip_false`,
`prob_intra`, `prob_last`, `prob_gf`, optional Y/UV-mode probability
updates and the MV-context update sub-stream). It also primes the
"distance to the four edges of the frame" trackers that the per-MB
parser uses to clamp motion vectors:

```c
pbi->mb.mb_to_top_edge = 0;
pbi->mb.mb_to_bottom_edge = ((pbi->common.mb_rows - 1) * 16) << 3;
mb_to_right_edge_start    = ((pbi->common.mb_cols - 1) * 16) << 3;
```
(decodemv.c:523–525)

The `<< 3` converts the bound from pixels to **1/8-pel units** —
the unit MVs are stored in (see RFC 6386 §17). These bounds are then
maintained incrementally inside the loop: after each MB they are
decremented by `(16 << 3)` (one MB at 1/8-pel granularity), so that
`mb_to_left_edge` is always "1/8-pel distance from current MB's left
edge to the frame's left edge" and the analogous interpretation for
the other three.

The `mi` pointer walks `pbi->common.mi`, advancing by one each MB and
**skipping one extra slot at the end of each row** (`mi++;` after the
column loop — decodemv.c:560). This skip accounts for the "left
predictor" sentinel column: `mode_info_stride = mb_cols + 1`, with
column −1 holding a sentinel `MODE_INFO` that supplies the "neighbor
to the left" for the first MB of the row. The same sentinel pattern
is used for row −1; cleanup of those sentinels happens in
`decodeframe.c` and `alloccommon.c`.

Under `CONFIG_ERROR_CONCEALMENT`, every iteration of the inner loop
checks `vp8dx_bool_error(&pbi->mbc[8])` and, if the boolean decoder
has detected stream corruption, records the MB number into
`pbi->mvs_corrupt_from_mb` and aborts immediately (decodemv.c:543–550).
This is what later code uses to decide which MVs to substitute when
concealing damaged macroblocks. The field is initialised to `UINT_MAX`
inside `mb_mode_mv_init` so that "no corruption found" means "every MB
number is less than the marker".

## Frame-header tail: `mb_mode_mv_init`

### `static void mb_mode_mv_init(VP8D_COMP *pbi)`

This function consumes the last few frame-header fields — those that
sit at the very end of the residual partition's header but that
logically belong to the *per-MB* parser because the values are used
nowhere else. The technical overview (§16.12) describes the exact
bitstream syntax. The function reads, in order:

1. **`mb_no_coeff_skip`** (1 bit). If true, every MB carries a
   `mb_skip_coeff` flag; if false, the flag is absent and all MBs are
   assumed to have coefficients.
2. **`prob_skip_false`** (8-bit literal, only if `mb_no_coeff_skip`).
3. For inter-frames only:
   - **`prob_intra`**, **`prob_last`**, **`prob_gf`** — three 8-bit
     literals that parameterise the reference-frame decision tree.
   - Optional Y-mode probability update: one update flag followed by
     four 8-bit literals overwriting `fc.ymode_prob[0..3]`.
   - Optional UV-mode probability update: one update flag followed by
     three 8-bit literals overwriting `fc.uv_mode_prob[0..2]`.
   - The MV-context update sub-stream, handled by `read_mvcontexts`.

The 7-bit raw / "non-zero or fall-back-to-1" mapping used for MV
probabilities (handled in `read_mvcontexts` below) is the canonical
VP8 quirk; the Y/UV probabilities here use the simpler 8-bit literal
form. None of these are conditional on each other in the YMODE/UVMODE
update blocks — once the gate bit says "update", *all* values in that
table are rewritten.

A few invariants worth pinning down:

- The function reads from `&pbi->mbc[8]`, the same boolean decoder
  used for the per-MB syntax. The header and per-MB syntax thus
  share one continuous bit-stream, with no alignment in between.
- `pbi->common.fc` ("frame context") is the table that this function
  may overwrite. `decodeframe.c` clones `pc->fc` into `pc->lfc` before
  invoking the residual-partition parser and restores it afterwards,
  so any overwrite here is scoped to the current frame.
- `prob_skip_false` is **zeroed** when `mb_no_coeff_skip == 0`, not
  left untouched. That zero is meaningful: it tells
  `decode_mb_mode_mvs` not to even *read* the skip flag.

## Per-MB driver: `decode_mb_mode_mvs`

### `static void decode_mb_mode_mvs(VP8D_COMP *pbi, MODE_INFO *mi)`

Called once per MB. In order:

1. **Segment ID.** If the frame's `update_mb_segmentation_map` bit is
   set, call `read_mb_features` to decode a new segment ID; on key
   frames where the map is not being updated, reset `segment_id` to 0;
   otherwise leave the inherited value alone (the reset of all IDs to
   0 mentioned in the function comment happens at frame-allocation
   time in `decodeframe.c`).
2. **`mb_skip_coeff`.** Read a single bool with probability
   `prob_skip_false` if `mb_no_coeff_skip` is set; otherwise default
   to 0.
3. **Modes & MVs.** Dispatch to `read_kf_modes` on key frames or
   `read_mb_modes_mv` on inter-frames. `is_4x4` is reset to 0 at this
   point because it is only set to 1 inside those callees when the
   mode is `B_PRED` or `SPLITMV`.

The sequencing matters: every code path below assumes that segment_id
and mb_skip_coeff have already landed by the time mode parsing begins,
even though none of the mode-tree probabilities depend on those
fields. The reason for the strict order is bitstream conformance, not
computational dependency.

## Segmentation map: `read_mb_features`

### `static void read_mb_features(vp8_reader *r, MB_MODE_INFO *mi, MACROBLOCKD *x)`

VP8 supports four segments (RFC 6386 §10). When the encoder chooses to
re-signal the per-MB segment map this frame, each MB picks one of
those four IDs via a 3-leaf binary tree coded with three probabilities
stored in `x->mb_segment_tree_probs[3]`:

```c
if (vp8_read(r, x->mb_segment_tree_probs[0])) {
  mi->segment_id = 2 + vp8_read(r, x->mb_segment_tree_probs[2]);
} else {
  mi->segment_id =     vp8_read(r, x->mb_segment_tree_probs[1]);
}
```
(decodemv.c:478–485)

The first bit splits {0,1} from {2,3}; the second bit picks within the
half. This is a hand-unrolled `vp8_treed_read` over a 3-node tree —
small enough that the inlined version is more readable than calling
the generic helper. The function is a no-op (segment_id is left
unchanged) when segmentation is disabled or the map is being
inherited from the previous frame.

## Key-frame MB parsing: `read_kf_modes`

### `static void read_kf_modes(VP8D_COMP *pbi, MODE_INFO *mi)`

A key-frame MB is unconditionally intra. The function:

1. Sets `mbmi.ref_frame = INTRA_FRAME`.
2. Reads the Y mode from a fixed 5-leaf tree
   (`vp8_kf_ymode_tree` indexed by `vp8_kf_ymode_prob`).
3. If the Y mode is `B_PRED`, sets `is_4x4 = 1` and decodes 16
   per-4×4 sub-modes. The probability table for each sub-mode is
   selected by the *neighbor sub-mode pair*:

```c
const B_PREDICTION_MODE A = above_block_mode(mi, i, mis);
const B_PREDICTION_MODE L = left_block_mode(mi, i);
mi->bmi[i].as_mode = read_bmode(bc, vp8_kf_bmode_prob[A][L]);
```
(decodemv.c:54–57)

`vp8_kf_bmode_prob[10][10][9]` is one of the largest static tables in
the codec (a 9-probability tree per (A,L) pair). The helpers
`above_block_mode` and `left_block_mode` are in `findnearmv.h`; when
the relevant neighbor MB is not in `B_PRED`, those helpers map the
MB-level mode into the B-mode equivalent (`DC_PRED → B_DC_PRED`, etc.)
so the table is always defined.

4. Reads the UV mode (`vp8_kf_uv_mode_tree`, 4 leaves).

The key-frame path uses *frame-invariant* probability tables (the
`vp8_kf_*_prob` constants in `vp8_entropymodedata.h`); inter-frame MBs
use the per-frame-updated `fc.ymode_prob` / `fc.uv_mode_prob` /
`fc.bmode_prob` tables that `mb_mode_mv_init` may rewrite. This is the
key distinction between key-frame and inter-frame intra parsing —
inter-frame intra is "intra with the inter-frame context", not "intra
with the key-frame context".

## Inter-frame MB parsing: `read_mb_modes_mv`

### `static void read_mb_modes_mv(VP8D_COMP *pbi, MODE_INFO *mi, MB_MODE_INFO *mbmi)`

The single largest function in the file. The outline is

```
ref_frame = vp8_read(bc, prob_intra);
if (ref_frame) {
    /* inter MB: pick reference, build MV predictors,
       decode mode + delta */
} else {
    /* intra MB: same as key-frame MB but with inter-frame probs */
}
```

### Inter sub-path

1. **Reference frame.** A single bit gates "this MB is intra (0)
   vs. inter (1)". If inter, two more bits select within
   {LAST_FRAME, GOLDEN_FRAME, ALTREF_FRAME}, gated by `prob_last`
   and (if not last) `prob_gf`. Note the unusual encoding: the
   initial `ref_frame` is 1 (= `LAST_FRAME`) and is **replaced** with
   2 or 3 only if `vp8_read(bc, prob_last)` returns true.

2. **MV-predictor accumulation.** A 4-bin histogram `cnt[]` and a
   ring of three candidate MVs `near_mvs[1..3]` are built by visiting
   the *above*, *left*, and *above-left* neighbors in that order
   (decodemv.c:311–361). Each neighbor contributes a count of 2, 2,
   and 1 respectively to the appropriate bin (one of CNT_INTRA,
   CNT_NEAREST, CNT_NEAR, CNT_SPLITMV) depending on whether its MV
   matches one already seen, and whether the neighbor was intra-coded.
   The `mv_bias` helper (findnearmv.h:24) negates the candidate if the
   neighbor's reference has the opposite sign-bias from the current
   reference — this is VP8's way of approximating bidirectional MVs.

   Note the pointer-arithmetic idiom `(++nmv)->as_int = ...` paired
   with `++cntx`: both pointers advance only when a *new* MV is
   accumulated, so the slot at `near_mvs[CNT_NEAREST]` is the
   most-frequent distinct neighbor MV, `near_mvs[CNT_NEAR]` the
   second-most-frequent, and `near_mvs[CNT_SPLITMV]` the third
   (despite the name; the slot is reused). The first slot
   `near_mvs[0]` is intentionally left zero (later overwritten as the
   "best" MV — see step 4 below).

3. **Mode decision.** Up to four sequential bool reads, each with a
   probability indexed by the count of the relevant bin:

```c
if (vp8_read(bc, vp8_mode_contexts[cnt[CNT_INTRA]   ][0])) {
    if (vp8_read(bc, vp8_mode_contexts[cnt[CNT_NEAREST]][1])) {
        if (vp8_read(bc, vp8_mode_contexts[cnt[CNT_NEAR]   ][2])) {
            /* one more bit decides SPLITMV vs NEWMV */
        } else  mbmi.mode = NEARMV;
    } else      mbmi.mode = NEARESTMV;
} else          mbmi.mode = ZEROMV;
```
(decodemv.c:363–444, condensed)

   The condensed comment "If we have three distinct MV's ... See if
   above-left MV can be merged with NEAREST" (decodemv.c:364) refers
   to the merging step at line 366:

```c
cnt[CNT_NEAREST] += ((cnt[CNT_SPLITMV] > 0) &
                     (nmv->as_int == near_mvs[CNT_NEAREST].as_int));
```

   This nudges the count if the third distinct MV happens to coincide
   with the first; the swap immediately below ensures
   `near_mvs[CNT_NEAREST]` always holds the actually-most-frequent MV
   even after this merger.

4. **Best-MV selection.** `near_index = CNT_INTRA + (cnt[CNT_NEAREST]
   >= cnt[CNT_INTRA])` picks slot 0 (the zero-initialised entry, so
   the "best" is implicitly (0,0)) when NEAREST is rarer than intra,
   and slot 1 (the nearest non-zero neighbor MV) otherwise. The
   selected best is then clamped to the frame boundary via
   `vp8_clamp_mv2` and used as the predictor for NEWMV.

5. **NEWMV vs SPLITMV branch.** Once it's decided this MB is neither
   ZERO nor NEAREST nor NEAR, a final bool — with probability indexed
   by the count of neighbors-that-were-SPLITMV — picks SPLITMV or
   NEWMV. NEWMV reads a `read_mv` delta and adds it to the best MV;
   SPLITMV hands off to `decode_split_mv`. Either way `mbmi.is_4x4`
   becomes 1 for SPLITMV (because per-4x4 MVs were written into
   `bmi[]`); NEARMV / NEARESTMV / NEWMV leave it 0.

   `mbmi.mv` after `decode_split_mv` is overwritten with
   `mi->bmi[15].mv.as_int` (decodemv.c:412). That choice (the MV of
   the bottom-right 4×4 block) becomes the "summary" MV for any
   downstream code that wants a single MV per MB without unpacking
   the split — chiefly the loop-filter context computation.

6. **Frame-boundary tracking.** Both NEWMV and SPLITMV paths can
   produce MVs that point outside the padded reference. The flag
   `mbmi.need_to_clamp_mvs` is set by `vp8_check_mv_bounds`
   (findnearmv.h:60) when this happens, and the inter-prediction
   stage uses that flag to decide whether to do the extra clamping
   `vp8_clamp_mv2` would normally apply. NEAREST/NEAR don't need this
   because they were already clamped to the visible frame boundary by
   the `vp8_clamp_mv2` call above; ZERO is trivially in-bounds.

7. **Error-concealment fill.** Under `CONFIG_ERROR_CONCEALMENT`, if
   the mode is not SPLITMV, the MB's `bmi[0..15].mv.as_int` are
   replicated from `mbmi.mv.as_int` so that later concealment code
   that walks `bmi[]` unconditionally finds valid per-4x4 MVs. The
   chained assignment (decodemv.c:448–455) is hand-unrolled rather
   than a loop because the compiler is free to vectorise it as a
   broadcast store.

### Intra sub-path

Identical in shape to `read_kf_modes` but using the inter-frame
context's probability tables (`fc.ymode_prob`, `fc.uv_mode_prob`,
`fc.bmode_prob`) and **without** the `[A][L]` 2-D conditioning on
neighbor B-modes: inter-frame B_PRED uses a single flat
`bmode_prob[9]` table, not the 10×10 conditional table. The implicit
assumption is that B_PRED is so rare in inter frames that conditional
modeling is not worth the table size and update cost.

The single-line if-with-assignment idiom

```c
if ((mbmi->mode = read_ymode(bc, pbi->common.fc.ymode_prob)) == B_PRED) {
```
(decodemv.c:463)

is the only place in the file where parse-and-test are fused; the
rest of the file prefers separate statements.

## SPLITMV: `decode_split_mv`

### `static void decode_split_mv(...)`

The most intricate function in the file. It handles the case where a
single inter MB is fractured into 2, 4 or 16 sub-blocks, each with
its own MV. Reading the function from top to bottom:

1. **Split shape.** Three sequential bool reads with **literal**
   probabilities — these magic numbers are normative and come directly
   from RFC 6386 §16.4 / §17.2:

```c
s = 3;                          /* default: 4x4, 16 sub-blocks */
num_p = 16;
if (vp8_read(bc, 110)) {
  s = 2; num_p = 4;             /* 8x8 quadrants */
  if (vp8_read(bc, 111)) {
    s = vp8_read(bc, 150);      /* 0 = 16x8 rows, 1 = 8x16 cols */
    num_p = 2;
  }
}
```
(decodemv.c:199–208)

   The values 110, 111, 150 are not derived from any probability
   table — they are fixed in the spec. The chosen index `s` is later
   stored as `mbmi->partitioning`.

2. **Per-subset loop.** For each of `num_p` subsets:

   - **`k = vp8_mbsplit_offset[s][j]`** — the index of the first 4×4
     block belonging to subset `j` under partition shape `s`. This
     table lives in `vp8/common/findnearmv.c:13`. The shapes "16×8"
     and "8×16" have only 2 subsets but each subset spans 8 blocks;
     "8×8" has 4 subsets spanning 4 blocks each; "4×4" has 16 subsets
     spanning 1 block each.

   - **Left neighbor MV.** If `k` is on the MB's left column
     (`k & 3 == 0`), look outside the MB: use the left MB's
     `mbmi.mv` if it's not SPLITMV, else its per-4x4 MV at offset
     `k + 4 - 1`. The `+ 4 - 1` is "step over one column boundary
     from outside the MB, into its rightmost column". Otherwise,
     use the MV at `mi->bmi[k - 1]` — the subset to our left
     *within this same MB*, already filled in by a previous loop
     iteration.

   - **Above neighbor MV.** Symmetric: if `k >> 2 == 0` (top row),
     look at the above MB; else `mi->bmi[k - 4]`.

   - **Sub-mode decision.** A 4-way tree gated by three
     probabilities from `vp8_sub_mv_ref_prob3[]` (the table
     defined and indexed in `get_sub_mv_ref_prob`, see below). The
     four sub-modes are:

```
LEFT4X4    use left neighbor's MV
ABOVE4X4   use above neighbor's MV
ZERO4X4    (0, 0)
NEW4X4     read an MV delta with read_mvcomponent, add to best_mv
```

   - **Bounds check.** `vp8_check_mv_bounds` against the same
     four `mb_to_*_edge` values used in `read_mb_modes_mv`, but
     extended outward by `LEFT_TOP_MARGIN` and `RIGHT_BOTTOM_MARGIN`
     (= 128 1/8-pel units = 16 pels) — this is the padding budget
     of the reference frame's border extension, beyond which the
     MV would need explicit clamping.

   - **Fill the subset.** Each subset of 1, 2, 4 or 8 4×4 blocks
     gets the same MV; the layout within the MB is given by
     `mbsplit_fill_offset[s][...]` (see below). The fill must happen
     *now*, before processing the next subset, because the next
     subset may select this one's MV via the "above" or "left"
     neighbor lookup above. This is exactly the comment at
     decodemv.c:264–266.

3. **Mode/MV finalisation.** After the loop, `mbmi->partitioning = s`
   is recorded so that downstream code knows the split shape without
   needing to re-derive it from the per-block MVs.

### Why expand `(k & 3) == 0` for the left edge?

The 16 sub-blocks of an MB are indexed 0..15 in raster scan: rows
of 4. Block `k`'s row is `k >> 2` and column is `k & 3`. The left-edge
test `!(k & 3)` is "column 0", and the top-edge test `!(k >> 2)` is
"row 0". The expressions `k + 4 - 1` and `k + 16 - 4` are "the block
in the same row, but in the *rightmost column* of the MB-to-the-left"
and "the block in the same column, but in the *bottom row* of the
MB-above", respectively — i.e., the immediate neighbor across the
boundary.

## SPLITMV helper tables

### `static const unsigned char mbsplit_fill_count[4]`

```c
static const unsigned char mbsplit_fill_count[4] = { 8, 8, 4, 1 };
```
(decodemv.c:114)

The number of 4×4 blocks per subset for each split shape:

| `s` | Shape  | Subsets | Blocks/subset |
|-----|--------|---------|---------------|
| 0   | 16×8   | 2       | 8             |
| 1   | 8×16   | 2       | 8             |
| 2   | 8×8    | 4       | 4             |
| 3   | 4×4    | 16      | 1             |

Indexed by `s`. The product `num_p * fill_count[s]` is always 16,
which is the constraint the splits must satisfy by construction.

### `static const unsigned char mbsplit_fill_offset[4][16]`

```c
{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },  /* s=0 16x8 */
{ 0, 1, 4, 5, 8, 9, 12, 13, 2, 3, 6, 7, 10, 11, 14, 15 },  /* s=1 8x16 */
{ 0, 1, 4, 5, 2, 3, 6, 7, 8, 9, 12, 13, 10, 11, 14, 15 },  /* s=2 8x8  */
{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },  /* s=3 4x4  */
```
(decodemv.c:115–120)

For each split shape, a flat list of 4×4 block indices grouped by
subset. Reading `mbsplit_fill_offset[s][j*fill_count[s] .. j*fill_count[s] + fill_count[s] - 1]`
gives the block indices of subset `j` in raster order *within that
subset's bounding box*.

For example, with `s = 1` (8×16) and `j = 0` (the left 8×16 half),
the offsets `{0, 1, 4, 5, 8, 9, 12, 13}` cover the two leftmost
columns of 4×4 blocks (i.e., the 8×16 region on the left side of
the MB), in raster scan.

For `s = 0` (16×8) and `s = 3` (4×4), the offset list is the trivial
identity `{0..15}` because the subsets are naturally contiguous in
raster order: 16×8 partitions the MB into a top half and a bottom
half (blocks 0..7 then 8..15), and 4×4 makes each block its own
subset.

These two tables encode the fill geometry purely in offsets, which
keeps `decode_split_mv`'s fill loop branch-free:

```c
fill_offset = &mbsplit_fill_offset[s][j * mbsplit_fill_count[s]];
do {
  mi->bmi[*fill_offset].mv.as_int = blockmv.as_int;
  fill_offset++;
} while (--fill_count);
```
(decodemv.c:270–276)

This pair `(mbsplit_fill_count, mbsplit_fill_offset)` is functionally
related to `vp8_mbsplit_offset[4][16]` in `findnearmv.c:13`, which
maps "subset index `j`" to "first block within that subset". The
two tables together let the parser keep two different traversal
orders: `vp8_mbsplit_offset` for "which block is the canonical
representative of subset j" (used for neighbor lookups) and
`mbsplit_fill_offset` for "which blocks belong to subset j" (used
for fill).

## SPLITMV per-subset probability context

### `const vp8_prob vp8_sub_mv_ref_prob3[8][VP8_SUBMVREFS - 1]`

```c
const vp8_prob vp8_sub_mv_ref_prob3[8][VP8_SUBMVREFS - 1] = {
  { 147, 136, 18 }, /* SUBMVREF_NORMAL          */
  { 223, 1, 34 },   /* SUBMVREF_LEFT_ABOVE_SAME */
  { 106, 145, 1 },  /* SUBMVREF_LEFT_ZED        */
  { 208, 1, 1 },    /* SUBMVREF_LEFT_ABOVE_ZED  */
  { 179, 121, 1 },  /* SUBMVREF_ABOVE_ZED       */
  { 223, 1, 34 },   /* duplicate of row 1       */
  { 179, 121, 1 },  /* duplicate of row 4       */
  { 208, 1, 1 }     /* duplicate of row 3       */
};
```
(decodemv.c:165–174)

`VP8_SUBMVREFS = 4` (blockd.h:122), so each row holds three
probabilities — one for each non-leaf node of the 4-leaf tree
{LEFT4X4, ABOVE4X4, ZERO4X4, NEW4X4}.

The table is indexed by a 3-bit code packed from
`(aez << 2) | (lez << 1) | lea` (see `get_sub_mv_ref_prob` below):
`lez` = "left MV is zero", `aez` = "above MV is zero",
`lea` = "left MV equals above MV". Only 5 of the 8 combinations are
semantically distinct; the table is padded to 8 entries (with rows
5, 6, 7 duplicating rows 1, 4, 3) so the index can be computed in
O(1) bit-tricks rather than via the `if-else` cascade in
`entropymode.c:21–33` (which is the encoder's lookup path).

The non-deduplicated form `vp8_sub_mv_ref_prob2[SUBMVREF_COUNT][3]`
in `entropymode.c:37` is the canonical form referenced by the RFC;
this file's `vp8_sub_mv_ref_prob3` is purely a decoder-side
optimisation that trades 3 × 3 = 9 bytes of padding for branch-free
indexing. The numbers themselves are the canonical RFC 6386 §16.4
"sub_mv_ref_prob" values.

Why the `_prob3` suffix? `vp8_sub_mv_ref_prob` (the original
encoder-style table) used a single 3-tuple regardless of neighbors,
`_prob2` is the 5-context version in `entropymode.c`, and `_prob3` is
this 8-context bit-tricks-indexable variant. All three exist in
parallel in the source tree.

### `static const vp8_prob *get_sub_mv_ref_prob(uint32_t left, uint32_t above)`

A three-line bit-trick lookup into `vp8_sub_mv_ref_prob3`:

```c
int lez = (left == 0);
int aez = (above == 0);
int lea = (left == above);
return vp8_sub_mv_ref_prob3[(aez << 2) | (lez << 1) | lea];
```
(decodemv.c:178–185)

The `lea` test captures the "same-MV" context implicitly — when
`left == above == 0`, both `lez` and `aez` are 1 *and* `lea` is 1,
so the index is `0b111 = 7` (which the duplicate-padding maps to
SUBMVREF_LEFT_ABOVE_ZED, the same semantics as index 3). Treating
the MVs as opaque `uint32_t` (the `as_int` view of `int_mv`) makes
"same MV" a single 32-bit equality test rather than four 16-bit
compares.

## MV-context probability updates: `read_mvcontexts`

### `static void read_mvcontexts(vp8_reader *bc, MV_CONTEXT *mvc)`

Called once per inter-frame from `mb_mode_mv_init`. Walks the two
`MV_CONTEXT` records (one for row, one for col), each holding
`MVPcount = 19` probabilities. For each probability, the per-bit
"should I update this prob?" flag is gated by the corresponding entry
of `vp8_mv_update_probs[c][j]` (the table is so heavily skewed toward
"no update" — most entries are 254 — that the update block typically
costs only ~38 bits per frame):

```c
if (vp8_read(bc, *up++)) {
  const vp8_prob x = vp8_read_literal(bc, 7);
  *p = x ? x << 1 : 1;
}
```
(decodemv.c:105–108)

The "7-bit raw, then `x << 1` or `1` if `x` is zero" mapping is the
VP8 quirk noted in the technical overview (§16.12). It avoids
encoding a probability of 0 (which would render some branches
unreachable) and maps the 128 raw values to {1, 2, 4, 6, ..., 254},
so the 7-bit field uses one bit for the "low-bit must be 0" structure
and reserves zero as the "actually 1" escape.

The function takes a `MV_CONTEXT *` and casts each row to a flat
`vp8_prob *` array for sequential walking. The cast is legal because
`MV_CONTEXT` is defined (entropymv.h:39–41) as exactly
`typedef struct mv_context { vp8_prob prob[MVPcount]; } MV_CONTEXT;`
— a single-field struct that is layout-compatible with its only
member.

## MV component coding: `read_mv` and `read_mvcomponent`

### `static int read_mvcomponent(vp8_reader *r, const MV_CONTEXT *mvc)`

Decodes one MV component (row or col) and returns its value in
**1/4-pel units**. The structure is "short path vs long path":

```c
if (vp8_read(r, p[mvpis_short])) {  /* long, |x| ≥ 8 */
    /* bits 0,1,2 — bottom up                          */
    for (i = 0; i < 3; i++) x += vp8_read(r, p[MVPbits+i]) << i;
    /* bits 9..4 — TOP DOWN                            */
    for (i = mvlong_width-1; i > 3; i--)
        x += vp8_read(r, p[MVPbits+i]) << i;
    /* bit 3 is implicit-1 unless any higher bit was 0 */
    if (!(x & 0xFFF0) || vp8_read(r, p[MVPbits+3])) x += 8;
} else {                            /* short, |x| < 8 */
    x = vp8_treed_read(r, vp8_small_mvtree, p + MVPshort);
}
if (x && vp8_read(r, p[MVPsign])) x = -x;
return x;
```
(decodemv.c:64–88, condensed)

Several subtleties:

- **Bit order.** Low bits (0–2) are read bottom-up, but high bits
  (4–9) are read **top-down**. This top-down order is what makes the
  "implicit bit 3" trick work: by the time we'd otherwise read bit 3,
  we already know whether any higher bit is set (via the running
  `x`). If yes, bit 3 is sent explicitly; if no, bit 3 is implicitly 1
  (because the long path requires |x| ≥ 8, so bit 3 *must* be 1 when
  no higher bit is). The bit gets added unconditionally via `x += 8`;
  the explicit read only happens when high bits *are* zero, in which
  case bit 3 itself decides the magnitude.

- **The 0xFFF0 mask.** Tests "any bit ≥ 4 is set". This works because
  the magnitude can be at most `mvlong_width = 10` bits wide (511 for
  the high bits + 8 from the implicit bit), so bit 9 fits inside a
  16-bit mask. The numeric guard limits MV components to
  `mv_max = 1023` (entropymv.h:21).

- **Sign is suppressed for x == 0.** Saves one wasted bit per
  zero MV component, even though MV components of exactly 0 in the
  long path are technically possible (they'd require all 10 magnitude
  bits to be 0, which is forbidden by the "long ⇒ |x| ≥ 8" invariant,
  so this is a safety net).

- **`mvc` is treated as a flat array via the cast
  `(const vp8_prob *)mvc`**. The `mvpis_short`, `MVPsign`,
  `MVPshort`, `MVPbits` constants are *offsets into that flat array*,
  not field names — see `entropymv.h:20–37` for the indexing scheme.

### `static void read_mv(vp8_reader *r, MV *mv, const MV_CONTEXT *mvc)`

Calls `read_mvcomponent` twice and multiplies each result by 2 to
convert from 1/4-pel to 1/8-pel:

```c
mv->row = (short)(read_mvcomponent(r,   mvc) * 2);
mv->col = (short)(read_mvcomponent(r, ++mvc) * 2);
```
(decodemv.c:92–94)

The `++mvc` advances from the row-component context to the
col-component context — `mvc` is an array of length 2 in
`pbi->common.fc.mvc[2]`. The increment is in the function-call
argument expression, which is fine because the comma sequencing of
the two statements gives a sequence point between them. The "store
as 1/8-pel" convention applies everywhere a finished MV lives in the
codec (`int_mv`, `mbmi.mv`, `bmi[].mv`); the only places 1/4-pel
units appear are inside `decode_split_mv`'s `blockmv.as_mv.row =
read_mvcomponent(...) * 2` (decodemv.c:247) and inside
`read_mvcomponent` itself.

## Tree-decoding wrappers

These four helpers all have the same shape: read a tree-coded value
and return it cast to the appropriate mode enum. They exist purely to
give the four cases distinct, type-safe entry points; each is just
two lines of code on top of `vp8_treed_read`:

### `static B_PREDICTION_MODE read_bmode(vp8_reader *bc, const vp8_prob *p)`

Reads a 4×4 sub-block intra mode (`vp8_bmode_tree`, 10 leaves
B_DC_PRED..B_HU_PRED). Used both for key-frame B_PRED MBs (with
neighbor-conditioned probabilities) and for inter-frame B_PRED MBs
(with the unconditional `fc.bmode_prob`).

### `static MB_PREDICTION_MODE read_ymode(vp8_reader *bc, const vp8_prob *p)`

Reads an inter-frame Y mode (`vp8_ymode_tree`, 5 leaves
DC_PRED, V_PRED, H_PRED, TM_PRED, B_PRED). Inter-frames-only —
`MB_PREDICTION_MODE` enumerates 10 values total (with five more for
inter modes NEARESTMV..SPLITMV), but the tree only covers the intra
half because this is the intra path of an inter-frame MB.

### `static MB_PREDICTION_MODE read_kf_ymode(vp8_reader *bc, const vp8_prob *p)`

The key-frame counterpart. Uses `vp8_kf_ymode_tree`, which has the
same 5 leaves but a different tree shape (the order of nodes is
chosen for typical key-frame mode distributions, different from
inter-frame intra).

### `static MB_PREDICTION_MODE read_uv_mode(vp8_reader *bc, const vp8_prob *p)`

Reads the UV chroma mode (`vp8_uv_mode_tree`, 4 leaves DC_PRED,
V_PRED, H_PRED, TM_PRED). The same tree is used for key-frame and
inter-frame UV; the difference between the two is the probability
table (`vp8_kf_uv_mode_prob` vs `fc.uv_mode_prob`).

These four wrappers are the file's smallest, most-duplicated
function pattern. They are not factored further because each cast
makes the calling site's intent (B-mode vs MB-mode) unambiguous —
the typedef'd enum type is the function's only documentation that
the caller is reading "a 4x4 sub-mode" vs "an MB-level mode". A
combined `read_tree(vp8_reader*, vp8_tree, const vp8_prob*)` helper
would have been one type cast longer at each call site and would
have hidden which sub-grammar is being parsed.

## Cross-references to the rest of the decoder

- The output `MODE_INFO` array is consumed by:
  - `vp8_decode_mb_tokens` (`detokenize.c`) — uses `mbmi.mode`,
    `mbmi.is_4x4`, `mbmi.mb_skip_coeff`.
  - `vp8_build_inter*_predictors_mb` (`reconinter.c`) — uses
    `mbmi.ref_frame`, `mbmi.mv`, `mbmi.partitioning`,
    `mbmi.need_to_clamp_mvs`, and the `bmi[].mv` array for SPLITMV.
  - `vp8_build_intra_predictors_*` (`reconintra.c`, `reconintra4x4.c`)
    — uses `mbmi.mode`, `mbmi.uv_mode`, and `bmi[].as_mode` for
    B_PRED.
  - `vp8_loop_filter_frame` (`vp8_loopfilter.c`) — uses `mbmi.mode`,
    `mbmi.ref_frame`, `mbmi.mv`, `mbmi.mb_skip_coeff` for filter-level
    selection and edge classification.

- The inputs read in this file:
  - `pbi->common.fc.mvc[2]`, `fc.ymode_prob[4]`, `fc.uv_mode_prob[3]`,
    `fc.bmode_prob[9]` — frame-context tables, possibly updated by
    `mb_mode_mv_init` itself.
  - `pbi->common.mode_info_stride`, `pbi->common.mb_rows`,
    `pbi->common.mb_cols`, `pbi->common.frame_type`,
    `pbi->common.ref_frame_sign_bias[4]`.
  - `pbi->mb.mb_to_*_edge`, `pbi->mb.segmentation_enabled`,
    `pbi->mb.update_mb_segmentation_map`,
    `pbi->mb.mb_segment_tree_probs[3]`.

- The static probability tables used:
  - `vp8_kf_ymode_prob`, `vp8_kf_uv_mode_prob`,
    `vp8_kf_bmode_prob[10][10][9]` from `vp8_entropymodedata.h` for
    key frames.
  - `vp8_mv_update_probs[2]` from `entropymv.h:43` for the MV
    probability-update sub-stream.
  - `vp8_mode_contexts[6][4]` from `modecont.c:13` for the inter-mode
    decision tree (referenced from `read_mb_modes_mv` via a literal
    lookup, not imported here).
  - `vp8_sub_mv_ref_prob3[8][3]` — defined locally in this file
    (decodemv.c:165) and unused outside it.
  - `vp8_bmode_tree`, `vp8_ymode_tree`, `vp8_kf_ymode_tree`,
    `vp8_uv_mode_tree`, `vp8_small_mvtree` from `entropymode.h`.

- The two local tables `mbsplit_fill_count` and
  `mbsplit_fill_offset` are referenced *only* by `decode_split_mv`
  and are static-scoped to this translation unit.

## Design observations

The file is unusually disciplined for the VP8 codebase: every
function fits on a screen except `read_mb_modes_mv` (which is
already broken into the visually distinct above/left/aboveleft
neighbor-accumulation blocks), there is no SIMD intrusion, no
platform-specific code, no preprocessor branching besides
`CONFIG_ERROR_CONCEALMENT`. This reflects the structural reality
that mode/MV parsing is fundamentally serial across MBs (each MB's
parse may depend on its neighbors) and inherently scalar (each
probability and bit is consumed one at a time by the arithmetic
decoder).

The boolean decoder `mbc[8]` is used by reference everywhere; nothing
in this file ever allocates, copies or destroys it. State lives in
`VP8D_COMP` for the parser side, in `MODE_INFO` for the parsed
output, and in `MV_CONTEXT` for the rolling probability tables. The
parser is therefore conceptually pure: same input bitstream + same
initial `fc` → same `MODE_INFO` array. That property is what
`decoderthreading.c` (in EC-disabled builds, absent) relies on to
parallelise the residual partition across rows.
