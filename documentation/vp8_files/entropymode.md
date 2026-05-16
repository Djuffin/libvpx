# `vp8/common/entropymode.c` — Mode probabilities and the trees that decode them

## Role in the decoder

VP8's entropy coder (the "bool decoder" of `dboolhuff.c`) is a binary
arithmetic coder: every primitive operation extracts one bit, biased by a
single 8-bit probability `p` in `[0..255]`. But the *symbols* the bitstream
actually carries — "what intra mode is this macroblock?", "is this MV
ZEROMV, NEAREST, NEAR, NEW, or SPLIT?", "which of the four 16-way
partitionings does this MB use?" — are not single bits. They are drawn
from small alphabets of size 4–10.

The libvpx solution, inherited verbatim from RFC 6386 §8, is the
**tree-coded** symbol: each alphabet is fixed at compile time as a small
binary tree, and a symbol is encoded as the sequence of left/right
choices along the root-to-leaf path. Each *internal node* of the tree
owns one probability; that probability is the bias of the bool decoder
when at that node.

`entropymode.c` is the file that holds these trees and their default
probability tables for every mode decision the decoder has to make:

- the per-4x4 intra-prediction mode (`B_PRED` sub-modes, alphabet of 10);
- the macroblock-level luma intra mode (alphabet of 5, two flavours: one
  for keyframes, one for inter frames);
- the chroma intra mode (alphabet of 4);
- the inter MB's MV-reference mode (alphabet of 5: `ZEROMV`, `NEAREST`,
  `NEAR`, `NEW`, `SPLITMV`);
- the four-way `mb_split` partition pattern selector;
- the sub-MV reference mode for each piece of a split MB (alphabet of 4);
- the small-MV magnitude tree shared with `entropymv.c`.

Together with `entropy.c` (residual-coefficient probabilities) and
`entropymv.c` (motion-vector component probabilities), it constitutes
the third pillar of VP8's arithmetic-coded payload. The encoder-side
companion of every table in this file is the matching `vp8_*_encodings[]`
array in `vp8_entropymodedata.h` — a precomputed Huffman-style code-word
listing that the encoder can index in O(1) to traverse the same tree
that the decoder walks bit-by-bit.

The file's surface is small — about a hundred lines of code, mostly
*table literals* — but its content is normative: the constants here are
exactly the numbers prescribed by RFC 6386 §11.2, §16.1, §16.2, §17.1,
and the tree shapes are exactly those of §11.2 (Fig. 11.1).

The rest of this document walks the file top-to-bottom, explaining
*what* each definition is, *why* it has the shape it has, the
invariants it relies on, and which downstream call site consumes it.

---

## Headers and table generation

```c
#define USE_PREBUILT_TABLES

#include "entropymode.h"
#include "entropy.h"
#include "vpx_mem/vpx_mem.h"

#include "vp8_entropymodedata.h"
```

`USE_PREBUILT_TABLES` is a vestigial switch from the early VP8
reference: when defined, the decoder relies on the precomputed
constants in `vp8_entropymodedata.h` rather than rebuilding them at
startup. The libvpx production decoder always uses prebuilt tables;
`vp8_entropymodedata.h` is included *as a C source fragment* (it
defines the storage of `vp8_bmode_encodings[]`, `vp8_ymode_prob[]`,
`vp8_kf_bmode_prob[][][]`, and so on). That is why the include sits
inside `entropymode.c` and not in the header — those arrays must have
exactly one storage definition in the translation unit, and this file
is the canonical owner.

The pull of `entropy.h` and `vpx_mem.h` is needed only for the
`memcpy()` in `vp8_init_mbmode_probs()` and for the `vp8_prob` type
forwarding; the bulk of this file does not actually need them.

---

## Sub-MV reference context: `vp8_mv_cont`

```c
int vp8_mv_cont(const int_mv *l, const int_mv *a) {
  int lez = (l->as_int == 0);
  int aez = (a->as_int == 0);
  int lea = (l->as_int == a->as_int);

  if (lea && lez) return SUBMVREF_LEFT_ABOVE_ZED;
  if (lea)        return SUBMVREF_LEFT_ABOVE_SAME;
  if (aez)        return SUBMVREF_ABOVE_ZED;
  if (lez)        return SUBMVREF_LEFT_ZED;
  return SUBMVREF_NORMAL;
}
```

### What
A pure helper that classifies the *pair* `(left MV, above MV)` into one of
the five context categories declared in `entropymode.h`:

```c
typedef enum {
  SUBMVREF_NORMAL,
  SUBMVREF_LEFT_ZED,
  SUBMVREF_ABOVE_ZED,
  SUBMVREF_LEFT_ABOVE_SAME,
  SUBMVREF_LEFT_ABOVE_ZED
} sumvfref_t;
```

### Why
When an MB chooses `SPLITMV`, each sub-block must pick its own
sub-MV-reference mode from `{LEFT4X4, ABOVE4X4, ZERO4X4, NEW4X4}`. The
*probability* with which those four choices appear is not stationary —
it depends strongly on whether the two neighbouring MVs are zero or
identical. RFC 6386 §16.3 therefore conditions the per-symbol
probability table on this 5-way classification. `vp8_mv_cont()` is
exactly that classifier.

### Invariants
- Comparison is on `int_mv::as_int`, the 32-bit packed form of the MV
  (row + column in one word); equality of the packed form is exactly
  the geometric equality of the two vectors.
- The five return values are checked in a strict priority order:
  `LEFT_ABOVE_ZED` (both zero and equal) wins over `LEFT_ABOVE_SAME`
  (both equal); the disjoint single-zero cases come next; otherwise
  `NORMAL`. The order matters because the predicates `lez && aez && lea`
  collapse together (a zero MV equals a zero MV), and the highest-priority
  return value yields the most specific probability table.
- The function is `pure`: it touches no global state and produces no
  side effects.

### How used
Called only from `decodemv.c` while decoding a `SPLITMV` macroblock —
actually it has been superseded there by the more compact lookup
`vp8_sub_mv_ref_prob3[(aez<<2)|(lez<<1)|lea]` (decodemv.c:165–183),
but the function and its companion table `vp8_sub_mv_ref_prob2[][..]`
are kept for the encoder path and for ABI stability.

---

## Default sub-MV-ref probabilities

```c
static const vp8_prob sub_mv_ref_prob[VP8_SUBMVREFS - 1] = { 180, 162, 25 };

const vp8_prob vp8_sub_mv_ref_prob2[SUBMVREF_COUNT][VP8_SUBMVREFS - 1] = {
  { 147, 136,  18 },   /* SUBMVREF_NORMAL          */
  { 106, 145,   1 },   /* SUBMVREF_LEFT_ZED        */
  { 179, 121,   1 },   /* SUBMVREF_ABOVE_ZED       */
  { 223,   1,  34 },   /* SUBMVREF_LEFT_ABOVE_SAME */
  { 208,   1,   1 }    /* SUBMVREF_LEFT_ABOVE_ZED  */
};
```

### What
Two tables giving the three internal-node probabilities of
`vp8_sub_mv_ref_tree` (described below).

- `sub_mv_ref_prob` is the *unconditional* default copied into
  `cm->fc.sub_mv_ref_prob[]` by `vp8_init_mbmode_probs()`.
- `vp8_sub_mv_ref_prob2` is the *context-keyed* table indexed by the
  category returned by `vp8_mv_cont()`. Each row is the probability
  triple to use when traversing the sub-MV-ref tree for sub-blocks with
  that particular neighbour pattern.

### Why
The alphabet is `{LEFT4X4, ABOVE4X4, ZERO4X4, NEW4X4}` (4 symbols, so
`VP8_SUBMVREFS - 1 = 3` internal-node probabilities). Numerically, the
columns mean:

| Col | Internal node | Branch-0 leads to | Branch-1 leads to     |
|-----|---------------|-------------------|-----------------------|
| 0   | root          | `LEFT4X4`         | rest of tree          |
| 1   | second-level  | `ABOVE4X4`        | tail                  |
| 2   | third-level   | `ZERO4X4`         | `NEW4X4`              |

A row like `{ 223, 1, 34 }` (for `SUBMVREF_LEFT_ABOVE_SAME`) tells the
arithmetic decoder "when the left and above MVs are identical, expect
the sub-block to copy the left one with probability 223/256, and if it
doesn't, the second choice (ABOVE) is *essentially guaranteed* (1/256
for the alternative)". This row is therefore the formal encoding of
the intuition "if the two neighbours agree, the split is almost
certainly a degenerate one that takes one of them".

### Invariants
- The row order is exactly the integer order of the `sumvfref_t` enum.
- The constants match RFC 6386 §16.3 (Table 30) byte-for-byte.
- `sub_mv_ref_prob` is `static`; it is the *initial* value that
  `vp8_init_mbmode_probs()` copies into the per-frame `FRAME_CONTEXT`,
  and from there it may be updated by the bitstream (rarely — there's
  no in-stream update path for sub-MV-ref in VP8, so once set per
  sequence it is constant).

### How used
The context-keyed `vp8_sub_mv_ref_prob2[ctx]` row is the `p` argument
to `vp8_treed_read(bc, vp8_sub_mv_ref_tree, p)` in decodemv.c when a
SPLITMV sub-block is being parsed.

---

## Static MB-split patterns: `vp8_mbsplits`, `vp8_mbsplit_count`, `vp8_mbsplit_probs`

```c
const vp8_mbsplit vp8_mbsplits[VP8_NUMMBSPLITS] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1 },  /* 0: top/bottom halves */
  { 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1 },  /* 1: left/right halves */
  { 0, 0, 1, 1, 0, 0, 1, 1, 2, 2, 3, 3, 2, 2, 3, 3 },  /* 2: four 8x8 quadrants */
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15 }   /* 3: full 4x4 split    */
};

const int vp8_mbsplit_count[VP8_NUMMBSPLITS] = { 2, 2, 4, 16 };

const vp8_prob vp8_mbsplit_probs[VP8_NUMMBSPLITS - 1] = { 110, 111, 150 };
```

### What
The geometric vocabulary for `SPLITMV`. When a macroblock chooses
`SPLITMV`, the bitstream first picks one of four *partition patterns*
(the index into `vp8_mbsplits`) and then transmits one MV per *piece*
of that partition.

- `vp8_mbsplits[p]` is a 16-element label vector: entry `k` (in 4x4
  raster order inside the MB) tells which piece (`0..vp8_mbsplit_count[p]-1`)
  block `k` belongs to.
- `vp8_mbsplit_count[p]` is the cardinality of that pattern's piece set.
- `vp8_mbsplit_probs` is the three-probability table for the
  partition-pattern tree (`vp8_mbsplit_tree`).

### Why
Encoding the four split patterns this way — as fixed label vectors
rather than as a free per-block partition — lets the encoder spend just
2–3 bits selecting one of the four canonical shapes (halves, quarters,
or full atomization) and then exactly `count` MVs. The geometric layout
is identical to RFC 6386 §16.2, Fig. 16.

The probability triple `{110, 111, 150}` is biased fairly evenly,
slightly favouring the four-quadrant split (pattern 2) — empirically
the most common.

### Invariants
- `vp8_mbsplit_count[p] == 1 + max(vp8_mbsplits[p][0..15])`.
- `vp8_mbsplits[p][k]` is the raster-order block index `k` inside the
  4x4 grid `(k%4, k/4)`. The 16 entries are stored in row-major order.
- `VP8_NUMMBSPLITS == 4`; the probability vector has length 3 because
  the tree has 4 leaves and therefore 3 internal nodes.

### How used
After `vp8_mbsplit_tree` selects the pattern, `decodemv.c` reads
`vp8_mbsplit_count[p]` motion vectors and stores them into the
appropriate `BLOCKD::bmi` slots. `reconinter.c` later uses
`vp8_mbsplits[p]` to know which MV applies to which 4x4 block.

---

## The intra-4x4 mode tree: `vp8_bmode_tree`

```c
const vp8_tree_index vp8_bmode_tree[18] = /* INTRAMODECONTEXTNODE value */
    {
      -B_DC_PRED, 2,          /* 0 = DC_NODE */
      -B_TM_PRED, 4,          /* 1 = TM_NODE */
      -B_VE_PRED, 6,          /* 2 = VE_NODE */
      8,          12,         /* 3 = COM_NODE */
      -B_HE_PRED, 10,         /* 4 = HE_NODE */
      -B_RD_PRED, -B_VR_PRED, /* 5 = RD_NODE */
      -B_LD_PRED, 14,         /* 6 = LD_NODE */
      -B_VL_PRED, 16,         /* 7 = VL_NODE */
      -B_HD_PRED, -B_HU_PRED  /* 8 = HD_NODE */
    };
```

### What
A binary decoding tree for the 10-symbol alphabet
`{B_DC_PRED, B_TM_PRED, B_VE_PRED, B_HE_PRED, B_RD_PRED, B_VR_PRED, B_LD_PRED, B_VL_PRED, B_HD_PRED, B_HU_PRED}`,
laid out in the flat-array form documented in `treecoder.h`:

> Each node of the tree is a pair of `vp8_tree_index`. Array index
> often references a corresponding probability table. Index <= 0 means
> done encoding/decoding and value = -Index. Index > 0 means need
> another bit, specification at index. Nonnegative indices are always
> even; processing begins at node 0.

So a tree of `n` leaves needs `2(n-1)` indices and `n-1` internal-node
probabilities. Here, n = 10 ⇒ 18 indices, 9 probabilities. The 9
labels in the right-hand comments — `DC_NODE`, `TM_NODE`, `VE_NODE`,
…, `HD_NODE` — are the *node names* used in the original RFC text and
they double as the indices into the probability table.

### Why
This is the literal Fig. 11.1 of RFC 6386, rendered into a 1-D array.
Each pair of entries describes one internal node: the first element is
"where to go if the decoded bit is 0", the second is "where to go if
the decoded bit is 1". A negative value `-X` is a leaf carrying the
intra-block mode `X`; a positive value is the *index* (in the same
array) of the next internal node.

### Trace: walking the tree

`vp8_treed_read` (treereader.h:30–38) is the canonical traversal:

```c
static INLINE int vp8_treed_read(vp8_reader *r, vp8_tree t,
                                 const vp8_prob *p) {
  vp8_tree_index i = 0;
  while ((i = t[i + vp8_read(r, p[i >> 1])]) > 0) {}
  return -i;
}
```

To decode a 4x4 intra mode at, say, the `RD_NODE`:

1. Start at array offset `i = 0`. Probability slot is `p[0]` (the
   `DC_NODE` probability).
2. `vp8_read(r, p[0])` returns 0 or 1. If 0, `i = t[0+0] = -B_DC_PRED`,
   loop terminates, return `B_DC_PRED`. If 1, `i = t[0+1] = 2`, continue.
3. At `i = 2` (the `TM_NODE`), probability slot is `p[1]`. Read again.
   0 → leaf `B_TM_PRED`. 1 → `i = 4`.
4. At `i = 4` (`VE_NODE`), use `p[2]`. 0 → `B_VE_PRED`. 1 → `i = 6`.
5. At `i = 6` (`COM_NODE`), use `p[3]`. **Both children are still
   positive** (8 and 12) — this is the only fully-internal node in the
   tree. 0 → `i = 8`; 1 → `i = 12`.
6. At `i = 8` (`HE_NODE`), use `p[4]`. 0 → `B_HE_PRED`. 1 → `i = 10`.
7. At `i = 10` (`RD_NODE`), use `p[5]`. 0 → `B_RD_PRED`; 1 → `B_VR_PRED`.
   Loop terminates.
8. The right subtree (`LD_NODE`, `VL_NODE`, `HD_NODE` at offsets 12,
   14, 16, with probabilities `p[6]`, `p[7]`, `p[8]`) is symmetric.

The crucial relation is `p[i >> 1]`: because internal node offsets are
always even (the invariant in `treecoder.h`), dividing by 2 gives the
sequential probability index. So tree array offset `2k` is the
probability `p[k]`, for `k = 0..n-2`.

### Invariants
- All non-leaf entries are even non-negative integers; the `i >> 1`
  trick depends on it.
- All leaf values are non-positive; `-i` is the symbol returned, so
  a leaf `-0` (used in `vp8_mbsplit_tree`) decodes as symbol 0.
- The tree shape (in particular the depth of each symbol) determines
  the bit-length of each codeword; together with the probabilities it
  determines the average per-symbol bit cost.

### How used
- In `decodemv.c::read_bmode()` (line 19): one call to
  `vp8_treed_read(bc, vp8_bmode_tree, p)` per 4x4 block of a `B_PRED`
  macroblock, where `p` is the row of `vp8_kf_bmode_prob[A][L]`
  selected by the modes of the *above* and *left* 4x4 neighbours
  (keyframes) or the constant `vp8_bmode_prob` (inter frames).
- The 9 node-name comments mirror an old enumeration
  `INTRAMODECONTEXTNODES` that has been deleted from the header; the
  layout remains identical to that defunct enum, which is why the
  comment is preserved verbatim ("Array indices are identical to
  previously-existing INTRAMODECONTEXTNODES.").

---

## Macroblock-level intra mode trees: `vp8_ymode_tree`, `vp8_kf_ymode_tree`, `vp8_uv_mode_tree`

```c
const vp8_tree_index vp8_ymode_tree[8] = {
  -DC_PRED, 2, 4, 6, -V_PRED, -H_PRED, -TM_PRED, -B_PRED
};

const vp8_tree_index vp8_kf_ymode_tree[8] = {
  -B_PRED, 2, 4, 6, -DC_PRED, -V_PRED, -H_PRED, -TM_PRED
};

const vp8_tree_index vp8_uv_mode_tree[6] = {
  -DC_PRED, 2, -V_PRED, 4, -H_PRED, -TM_PRED
};
```

### What
Three coding trees for MB-level intra modes:

- **`vp8_ymode_tree`** — luma mode for an *inter* frame's intra MB.
  Alphabet `{DC_PRED, V_PRED, H_PRED, TM_PRED, B_PRED}` (`VP8_YMODES = 5`).
- **`vp8_kf_ymode_tree`** — luma mode for a *keyframe* MB. Same alphabet,
  different tree shape.
- **`vp8_uv_mode_tree`** — chroma mode. Alphabet
  `{DC_PRED, V_PRED, H_PRED, TM_PRED}` (`VP8_UV_MODES = 4`).

### Why
A glance at the layout shows the asymmetry. In inter frames most intra
MBs are `DC_PRED` (the cheapest, flattest predictor), so `vp8_ymode_tree`
gives `DC_PRED` the shortest codeword (1 bit, leaf at index 0). On
keyframes the picture is wildly different: a fresh-start frame has no
temporal redundancy and the most-used luma mode is the per-4x4-block
`B_PRED`, so `vp8_kf_ymode_tree` puts `B_PRED` at the 1-bit leaf and
shoves `DC_PRED` deeper.

These are not just different probability values — they are different
*tree shapes*. The placement of the symbol in the tree fixes its
codeword length; the probability merely fine-tunes the arithmetic
coder's compression of that fixed-length symbol. RFC 6386 §16.1
specifies both layouts.

`vp8_uv_mode_tree` is the same shape as the inter `vp8_ymode_tree`
truncated to four symbols (`B_PRED` does not exist for chroma — VP8
never sub-splits the 8x8 chroma planes).

### Invariants
- Tree length is `2 * (alphabet - 1)` (8 for the two 5-symbol luma
  trees, 6 for the 4-symbol chroma tree).
- The corresponding probability tables (`vp8_ymode_prob`,
  `vp8_kf_ymode_prob`, `vp8_uv_mode_prob`, `vp8_kf_uv_mode_prob`,
  defined in `vp8_entropymodedata.h`) have length `alphabet - 1`.

### How used
- `decodemv.c::read_ymode()` decodes an inter MB's luma mode with
  `vp8_ymode_tree` + the (potentially bitstream-updated)
  `cm->fc.ymode_prob`.
- `decodemv.c::read_kf_ymode()` uses `vp8_kf_ymode_tree` +
  the static `vp8_kf_ymode_prob`.
- `decodemv.c::read_uv_mode()` uses `vp8_uv_mode_tree`.
- A symmetric pair `read_kf_uv_mode()` uses the same tree with
  `vp8_kf_uv_mode_prob`.

---

## The MB-split partition tree: `vp8_mbsplit_tree`

```c
const vp8_tree_index vp8_mbsplit_tree[6] = { -3, 2, -2, 4, -0, -1 };
```

### What
Decoding tree for the four split patterns of `vp8_mbsplits`. Alphabet of
4 symbols ⇒ 6-element tree, 3 internal-node probabilities (supplied by
`vp8_mbsplit_probs`).

### Why
Leaf labelling: `-0`, `-1`, `-2`, `-3` decode as patterns `0..3`. Read
through the tree:

| Internal node     | Prob       | Branch 0       | Branch 1       |
|-------------------|------------|----------------|----------------|
| root (offset 0)   | probs[0]=110 | leaf `3` (full 4x4) | next (offset 2) |
| second (offset 2) | probs[1]=111 | leaf `2` (four 8x8) | next (offset 4) |
| third (offset 4)  | probs[2]=150 | leaf `0` (halves) | leaf `1` (verticals) |

So pattern 3 (the most expensive, full atomization) gets the shortest
1-bit codeword — a counter-intuitive choice that pays off because
`SPLITMV` is itself the rarest MB-level mode (chosen only when the
others fail). The third probability `150` slightly favours horizontal
halves over vertical, reflecting empirical content statistics.

### How used
Read by `decodemv.c` whenever the MB's mode resolves to `SPLITMV`,
using a private static probability vector that the keyframe-side branch
of the decoder hands in.

---

## The MV-ref mode tree: `vp8_mv_ref_tree`

```c
const vp8_tree_index vp8_mv_ref_tree[8] = {
  -ZEROMV, 2, -NEARESTMV, 4, -NEARMV, 6, -NEWMV, -SPLITMV
};
```

### What
The 5-symbol alphabet for an inter MB's MV-reference mode:
`{ZEROMV, NEARESTMV, NEARMV, NEWMV, SPLITMV}`. The numerical enum order
is `{NEARESTMV=0, NEARMV=1, ZEROMV=2, NEWMV=3, SPLITMV=4}`
(blockd.h:72–76); this is **not** the same order as the tree.

### Why
The tree puts `ZEROMV` at depth 1 (the shortest codeword) — the single
most common inter MV for slow-motion content. `SPLITMV`, the most
expensive choice, sits at the bottom (depth 4, sharing it with `NEWMV`).

The four probabilities driving this tree
(`mv_ref_p[0..3]` in the bitstream) are derived per-MB at decode time
from a 4x4×4 lookup table keyed by the *MV-ref classification* of the
left and above MBs. The defaults live in `modecont.c`; this file only
supplies the tree shape.

### How used
`decodemv.c::read_mv_ref()` calls `vp8_treed_read(bc, vp8_mv_ref_tree,
mv_ref_p)` once per inter MB. The returned value is one of the
`MB_PREDICTION_MODE` constants directly (the leaf labels `-ZEROMV`,
`-NEARESTMV`, etc., already encode the enum values).

---

## The sub-MV-ref tree: `vp8_sub_mv_ref_tree`

```c
const vp8_tree_index vp8_sub_mv_ref_tree[6] = {
  -LEFT4X4, 2, -ABOVE4X4, 4, -ZERO4X4, -NEW4X4
};
```

### What
A 4-symbol tree for the per-sub-block MV-reference mode in a `SPLITMV`
MB.

### Why
Inside a SPLITMV macroblock each piece can either copy its left/above
neighbour's MV, use a zero MV, or transmit a new MV explicitly.
`LEFT4X4` gets the 1-bit codeword because in practice the same MV
applies to a contiguous chunk of the partition — choosing "copy left"
is the default that triggers the streak. `NEW4X4` is at the bottom,
because actually transmitting a fresh MV is the most expensive choice
in both bits *and* in the subsequent MV-component encoding.

### How used
`decodemv.c` walks this tree for every piece of every SPLITMV MB,
using the row of `vp8_sub_mv_ref_prob3[]` (the table in `decodemv.c`
that has superseded `vp8_sub_mv_ref_prob2[]` here) selected by
`vp8_mv_cont()`'s context.

---

## The small-MV tree: `vp8_small_mvtree`

```c
const vp8_tree_index vp8_small_mvtree[14] = {
  2,  8,  4,  6,  -0, -1, -2, -3, 10, 12, -4, -5, -6, -7
};
```

### What
A balanced 8-leaf tree used by `entropymv.c` to decode the magnitude
of an MV component whose absolute value is in `[0..7]`. The
alphabet is `{0, 1, 2, 3, 4, 5, 6, 7}`; the leaves carry exactly those
integers (encoded as `-0, -1, …, -7`).

### Why
Belongs in `entropymode.c` historically because the MV-magnitude
decoder shares the `vp8_treed_read` infrastructure and benefits from
co-locating with the other tree tables — even though, strictly, it is
an *MV* table. The tree is fully balanced (depth 3 for every leaf) so
that the *probabilities* alone decide the relative cost: the
distribution of small-MV magnitudes is fairly flat and a balanced tree
is near-optimal.

### How used
`decodemv.c::read_mvcomponent()` (line 83 region) calls
`vp8_treed_read(r, vp8_small_mvtree, p + MVPshort)` whenever an MV
component is being decoded as "short" (under 8 in magnitude).

---

## Per-frame defaults: `vp8_init_mbmode_probs`

```c
void vp8_init_mbmode_probs(VP8_COMMON *x) {
  memcpy(x->fc.ymode_prob,    vp8_ymode_prob,    sizeof(vp8_ymode_prob));
  memcpy(x->fc.uv_mode_prob,  vp8_uv_mode_prob,  sizeof(vp8_uv_mode_prob));
  memcpy(x->fc.sub_mv_ref_prob, sub_mv_ref_prob, sizeof(sub_mv_ref_prob));
}
```

### What
Resets the three mutable, per-frame-context mode-probability tables on
`VP8_COMMON::fc` (`fc` = "frame context") to their default values from
the spec.

### Why
VP8 supports a small amount of probability adaptation: the bitstream
header for an inter frame may carry partial updates to `ymode_prob`,
`uv_mode_prob` and (via the `prob_skip_false` field) the skip-coeff
probability. To start from a known state at every keyframe, the decoder
calls `vp8_init_mbmode_probs()` on `VP8_COMMON` once, restoring all
three tables to the canonical defaults.

The three constants copied in are:

- `vp8_ymode_prob   = { 112, 86, 140, 37 }` — inter-frame luma intra
  mode (4 probabilities for 5 symbols).
- `vp8_uv_mode_prob = { 162, 101, 204 }` — chroma intra mode (3 of 4).
- `sub_mv_ref_prob  = { 180, 162,  25 }` — the unconditional baseline
  for sub-MV-ref (overridden in practice by the context-specific rows
  of `vp8_sub_mv_ref_prob2`).

### Invariants
- The sizes match exactly: `sizeof(vp8_ymode_prob) == (VP8_YMODES-1)`,
  etc., so the `memcpy` cannot overflow.
- The `sub_mv_ref_prob` source is `static` in this file; the only way
  to "see" it from the rest of the decoder is via this function.

### How used
- `alloccommon.c:172` — first-time init when `VP8_COMMON` is allocated.
- `decodeframe.c:826` — on every keyframe, just before the per-frame
  probability update loop runs.

---

## Per-frame intra-4x4 defaults: `vp8_default_bmode_probs`

```c
void vp8_default_bmode_probs(vp8_prob dest[VP8_BINTRAMODES - 1]) {
  memcpy(dest, vp8_bmode_prob, sizeof(vp8_bmode_prob));
}
```

### What
Copies the 9 default 4x4-intra-mode probabilities (the constant
`vp8_bmode_prob = { 120, 90, 79, 133, 87, 85, 80, 111, 151 }` from
`vp8_entropymodedata.h`) into the caller's buffer.

### Why
`vp8_bmode_prob` is the *inter-frame, context-free* probability vector
for the 4x4 intra-block mode. The keyframe path instead uses the much
larger 3-D table `vp8_kf_bmode_prob[A][L][i]` keyed on the above (`A`)
and left (`L`) neighbour modes — that table also lives in
`vp8_entropymodedata.h` and is accessed directly (no copy required,
since the keyframe table is read-only and never adapted).

### How used
- `alloccommon.c:173` — once per `VP8_COMMON`, target buffer is
  `oci->fc.bmode_prob`.

The wrapper exists as a function (instead of `extern const`) so that
the caller does not need to know the storage class of `vp8_bmode_prob`
— a tiny encapsulation borrowed from the original reference encoder.

---

## What's *not* here

The header `entropymode.h` declares three larger tables that this file
does **not** define:

- `vp8_kf_default_bmode_counts[10][10][10]` — old encoder-side
  histogram bootstrap; its definition was moved out of the decoder
  build long ago.
- `vp8_kf_bmode_prob[10][10][9]` — the per-(above,left) keyframe
  intra-4x4 probability cube; defined in `vp8_entropymodedata.h`.
- `vp8_kf_uv_mode_prob`, `vp8_kf_ymode_prob` — the keyframe MB-mode
  probabilities; also in `vp8_entropymodedata.h`.

Likewise, all the `vp8_*_encodings[]` arrays declared in
`entropymode.h` are *encoder*-side codeword tables and live entirely
in the included `vp8_entropymodedata.h`. The decoder never reads them
— it only walks the trees.

---

## Summary table

| Tree                  | Leaves | Probability table                 | Decoded symbol               | Decoded by              |
|-----------------------|--------|-----------------------------------|------------------------------|-------------------------|
| `vp8_bmode_tree`      | 10     | `vp8_(kf_)bmode_prob[A][L]` / `fc.bmode_prob` | `B_PRED` sub-mode (per 4x4)  | `decodemv.c:19`         |
| `vp8_ymode_tree`      | 5      | `fc.ymode_prob`                   | Inter-frame luma intra mode  | `decodemv.c:25`         |
| `vp8_kf_ymode_tree`   | 5      | `vp8_kf_ymode_prob`               | Keyframe luma intra mode     | `decodemv.c:31`         |
| `vp8_uv_mode_tree`    | 4      | `fc.uv_mode_prob` / `vp8_kf_uv_mode_prob` | Chroma intra mode    | `decodemv.c:37`         |
| `vp8_mv_ref_tree`     | 5      | `mv_ref_p[4]` (context-derived)   | MV-ref mode (`ZEROMV` … `SPLITMV`) | `decodemv.c::read_mv_ref` |
| `vp8_sub_mv_ref_tree` | 4      | `vp8_sub_mv_ref_prob3[ctx]`       | Sub-MV-ref mode              | `decodemv.c:241`        |
| `vp8_mbsplit_tree`    | 4      | `vp8_mbsplit_probs`               | Split-pattern index          | `decodemv.c`            |
| `vp8_small_mvtree`    | 8      | `p + MVPshort`                    | Small MV magnitude (0–7)     | `decodemv.c:83`         |

Every one of these trees, every leaf labelling, every default
probability vector is part of the VP8 bitstream spec; changing a
single byte breaks bitstream compatibility. That is why this file is
predominantly a *table of constants*, with only the trivial
`vp8_mv_cont` classifier and two `memcpy` resetters as actual code.
