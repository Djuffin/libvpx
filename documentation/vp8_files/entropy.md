# `vp8/common/entropy.c` — coefficient-token entropy data

## Role in the decoder

`entropy.c` is the static-data file that defines the **coefficient-token
entropy tables and the small constants the arithmetic coder needs to
walk them**. It contains no algorithmic code beyond a single one-line
helper (`vp8_default_coef_probs`) — every symbol in the translation
unit is a `const` table whose values are fixed by RFC 6386. The
companion header `entropy.h` exports just enough of this state for the
detokenizer (`vp8/decoder/detokenize.c`) and the header parser
(`vp8/decoder/decodeframe.c`) to interpret the residual section of a
VP8 bitstream.

The file is the **single source of truth** for:

1. The arithmetic-decoder renormalisation LUT (`vp8_norm`).
2. The coefficient-token alphabet's tree shape (`vp8_coef_tree`,
   `vp8_coef_encodings`, `vp8_extra_bits`).
3. The **context indexing tables** that map a 4×4 zig-zag position
   onto one of eight "coefficient bands" (`vp8_coef_bands`) and a
   token value onto one of three "previous-token classes"
   (`vp8_prev_token_class`).
4. The forward and inverse zig-zag permutations
   (`vp8_default_zig_zag1d`, `vp8_default_inv_zig_zag`,
   `vp8_default_zig_zag_mask`).
5. The default coefficient-probability table loaded into every
   `VP8_COMMON` at keyframe time (`default_coef_probs`, brought in via
   `default_coef_probs.h`).
6. The probabilities used to gate per-frame updates of that table
   (`vp8_coef_update_probs`, brought in via `coefupdateprobs.h`).
7. Two segmentation-data widths used by the header parser
   (`vp8_mb_feature_data_bits`).

Everything in this file is shared between encoder and decoder — it lives
in `vp8/common/` precisely because both halves of the codec must agree
on these tables byte-for-byte for a stream to be decodable. In the
decoder-only build we are documenting, the file is roughly a "ROM
chip": it is read but never modified at runtime (with the single
exception of `vp8_default_coef_probs`, which is a copy, not a mutation).

Two siblings round out the data set:

- `default_coef_probs.h` (included near the bottom): a 1,056-byte
  table of initial probabilities for the 11 internal nodes of the
  token tree, indexed by `[block_type][band][prev_token_context]`.
- `coefupdateprobs.h` (included at the top): a 1,056-byte table of
  per-node "update flag" probabilities, used by the compressed header
  parser to decide whether each individual probability is being
  overridden for the current frame.

The rest of this document walks the file top-to-bottom, taking each
definition in turn and giving it the prose treatment that RFC 6386
sections 11 and 13 spend many pages on.

---

## 1. Arithmetic-decoder renormalisation: `vp8_norm`

```c
DECLARE_ALIGNED(16, const unsigned char, vp8_norm[256]) = {
  0, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, …
};
```

### What it is

A 256-entry byte table mapping an 8-bit value `r` to the number of
leading zeros of `r` viewed as an unsigned 8-bit integer, with the
exception that `vp8_norm[0] == 0` (not 8).

| `r`            | `vp8_norm[r]` |
|----------------|---------------|
| 0              | 0 (sentinel)  |
| 1              | 7             |
| 2–3            | 6             |
| 4–7            | 5             |
| 8–15           | 4             |
| 16–31          | 3             |
| 32–63          | 2             |
| 64–127         | 1             |
| 128–255        | 0             |

### Why it is here

Every per-bit decode of VP8's binary arithmetic coder ends with a
*renormalisation* step that shifts `range` (and the residual `value`)
left until `range`'s top bit is back at position 7 — i.e., until
`range ≥ 128`. A naïve loop would issue 0–7 dependent shifts per bit.
With this LUT, the decoder computes the shift count in one load and
applies it as a single `<<` (see `vp8dx_decode_bool` in
`vp8/decoder/dboolhuff.h`):

```c
shift  = vp8_norm[range];
range <<= shift;
value <<= shift;
```

The arithmetic coder guarantees `range > 0` after a successful split,
so the `vp8_norm[0]` entry is never read in normal operation; setting
it to `0` makes the table robust against zero inputs without forcing
a branch.

### Invariants and use

- 256 bytes, naturally 16-byte aligned for SIMD-friendly loads.
- Declared `extern` in `vp8/decoder/dboolhuff.h` (line 46), not in
  `entropy.h` — historically it belongs to the bool decoder but is
  defined here because `entropy.c` is the common-side TU that owns
  RFC-6386 static data.
- Read on every coefficient-bit, mode-bit, and MV-bit decoded — by far
  the hottest table in the file.

---

## 2. Coefficient-band coarsening: `vp8_coef_bands`

```c
DECLARE_ALIGNED(16, const unsigned char,
                vp8_coef_bands[16]) = { 0, 1, 2, 3, 6, 4, 5, 6,
                                        6, 6, 6, 6, 6, 6, 6, 7 };
```

### What it is

A 16-entry table indexed by **zig-zag position** (0–15) that returns a
**coefficient band** in 0–7. The bands are the second axis of the
coefficient probability cube; they collapse the 16 positions inside a
4×4 transform into 8 statistical equivalence classes.

| Zig-zag idx | Band | Meaning                                |
|-------------|------|----------------------------------------|
| 0           | 0    | DC                                     |
| 1           | 1    | first low-frequency AC                 |
| 2           | 2    | second low-frequency AC                |
| 3           | 3    | third                                  |
| 4           | 6    | "leftover" position grouped with 6     |
| 5           | 4    | fourth band                            |
| 6           | 5    | fifth band                             |
| 7           | 6    | sixth band                             |
| 8–14        | 6    | mid-to-high-frequency cluster          |
| 15          | 7    | the high-frequency corner              |

### Why this particular permutation

The mapping is the one specified by RFC 6386 §13.3 (table from Figure
9 of the spec). It is **not** monotone in zig-zag order: position 4
jumps to band 6 because, after the early low-frequency bands, the
remaining 11 zig-zag positions cluster naturally into one fat
"mid-band" (band 6) and a single high-frequency tail (band 7,
position 15 only). Bands 4 and 5 catch the off-diagonal positions
(5 and 6) that statistically differ from both the leading low-frequency
group and the bulk of the mid-frequency cluster.

The bands let VP8 reuse one probability vector across many zig-zag
positions while preserving the strong statistical contrast between
the DC, the first few AC, and the high-frequency tail.

### How it is used

Each per-coefficient probability lookup in the decoder is of the form
`fc->coef_probs[block_type][band][prev_ctx][node]`, where
`band = vp8_coef_bands[n]` and `n` is the current zig-zag index. The
detokenizer keeps its own private copy of this table
(`kBands[]` in `detokenize.c:35`) to avoid the cross-TU load on every
coefficient — but the values are identical to `vp8_coef_bands`, plus a
17th sentinel entry to simplify the inner loop's bounds.

### Invariants

- 16 bytes, 16-byte aligned.
- Values are bounded by `COEF_BANDS - 1 == 7` (`entropy.h:67`).
- Read-only; never updated by the bitstream.

---

## 3. Previous-token class: `vp8_prev_token_class`

```c
DECLARE_ALIGNED(16, const unsigned char,
                vp8_prev_token_class[MAX_ENTROPY_TOKENS]) = {
  0, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0
};
```

### What it is

A 12-entry table indexed by **token value** (one of the 12
`*_TOKEN` constants defined in `entropy.h`) that classifies the
previously-decoded token into one of three "complexity contexts":

| Token                            | Value | Class |
|----------------------------------|------:|------:|
| `ZERO_TOKEN`                     | 0     | 0     |
| `ONE_TOKEN`                      | 1     | 1     |
| `TWO_TOKEN` … `DCT_VAL_CATEGORY6`| 2–10  | 2     |
| `DCT_EOB_TOKEN`                  | 11    | 0     |

### Why classify like this

The third axis of the coefficient probability cube
(`PREV_COEF_CONTEXTS = 3`) records "how complex was the neighbourhood
of this coefficient?" Before the first coefficient, the neighbourhood
is the **count of nonzero coefficients in the spatial neighbours
above and to the left** (one of {0, 1, 2}). After the first
coefficient, the neighbourhood is collapsed to "what was the magnitude
of the *previously decoded* coefficient in this same block?" — exactly
the three classes here: it was zero, it was ±1, or it was bigger.
EOB collapses back to class 0 because it can only occur after a
non-zero coefficient and so the next block's first coefficient will use
the spatial-neighbour count anyway; but inside this block there are no
more coefficients, so the EOB entry is only an extra-bit placeholder.

The header for `entropy.h` (lines 70–85) spells out the philosophy in
the source: "the intuitive meaning of this measure changes as
coefficients are decoded… this shift in meaning is perfectly OK because
our context depends also on the coefficient band."

### Invariants and use

- 12 bytes, 16-byte aligned to share a cache line with the band table.
- Indexed by `*_TOKEN`; the array exactly covers `[0,
  MAX_ENTROPY_TOKENS)`.
- The decoder uses it only when the next coefficient's `prev_ctx`
  depends on the value just decoded. In the inlined fast path of
  `GetCoeffs` (`detokenize.c`) the equivalent is hand-unrolled —
  `p = prob[band][0]` after a zero, `[1]` after a one, `[2]` after a
  larger token — but the values match `vp8_prev_token_class`.

---

## 4. Zig-zag permutations

### 4.1 `vp8_default_zig_zag1d`

```c
DECLARE_ALIGNED(16, const int, vp8_default_zig_zag1d[16]) = {
  0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15,
};
```

The forward zig-zag: `vp8_default_zig_zag1d[n]` is the **raster-order
position** of the coefficient transmitted at zig-zag slot `n`. Visualised
on the 4×4 grid:

```
  0   1   5   6
  2   4   7  12
  3   8  11  13
  9  10  14  15
```

(`5` at raster index 4 means "the coefficient sent fifth ends up at row
1 col 0", etc.) This is the standard zig-zag order of RFC 6386 §13.2
that diagonally walks from DC outward, biasing the bitstream to send
the energy-concentrated low frequencies first so that EOB can arrive
early.

### 4.2 `vp8_default_inv_zig_zag`

```c
DECLARE_ALIGNED(16, const short, vp8_default_inv_zig_zag[16]) =
    { 1, 2, 6, 7, 3, 5, 8, 13, 4, 9, 12, 14, 10, 11, 15, 16 };
```

The **inverse** map, with a +1 offset: `vp8_default_inv_zig_zag[r] - 1`
is the zig-zag slot of raster position `r`. The +1 is so a value of 0
can be reserved as "not present" by callers that share this table with
EOB tracking. In the decoder we are documenting, this table is exported
but not used (it is referenced from the encoder's coefficient-cost
machinery only). It is kept here so that encoder and decoder remain
build-symmetric.

### 4.3 `vp8_default_zig_zag_mask`

```c
DECLARE_ALIGNED(16, const short, vp8_default_zig_zag_mask[16]) = {
  1, 2, 32, 64, 4, 16, 128, 4096, 8, 256, 2048, 8192, 512, 1024, 16384, -32768
};
```

A bitmask version: `vp8_default_zig_zag_mask[r] = 1 << zig_zag_slot(r)`,
stored as a `short` so the high-frequency entry at zig-zag slot 15
becomes `1<<15 = 0x8000`, which is `-32768` in signed 16-bit. The
generator in the surrounding comment shows the construction:

```c
for (i = 0; i < 16; ++i)
    vp8_default_zig_zag_mask[vp8_default_zig_zag1d[i]] = 1 << i;
```

The mask form lets the encoder OR together a bitset of "which raster
positions are nonzero" and then test "is the run up to zig-zag slot
`k` empty?" with a single `&`. Like the inverse table, this is
encoder-only in practice; the decoder build keeps it for symmetry.

### Invariants

- All three tables are 16-element, 16-byte aligned.
- They are a permutation of {0..15} (the `_mask` is a permutation of
  the 16 distinct powers of two).
- They never change at runtime — the only "scan order" VP8 knows is
  this zig-zag.

---

## 5. Segmentation-feature widths: `vp8_mb_feature_data_bits`

```c
const int vp8_mb_feature_data_bits[MB_LVL_MAX] = { 7, 6 };
```

A two-entry table giving the **bit width of the magnitude field** of
each of VP8's two per-segment features in the bitstream:

| Index           | Feature                         | Width |
|-----------------|---------------------------------|-------|
| `MB_LVL_ALT_Q`  | Per-segment quantizer delta     | 7 bits|
| `MB_LVL_ALT_LF` | Per-segment loop-filter delta   | 6 bits|

Each value is transmitted in the segmentation update section of the
compressed header as a sign-magnitude pair: a magnitude of the width
specified here, followed (if non-zero) by a sign bit. The header
parser at `vp8/decoder/decodeframe.c:889` indexes into this table
inside the per-segment loop, so the widths are not hard-coded at the
parsing call site. The values follow RFC 6386 §9.3 / §10.

The fact that this lives in `entropy.c` rather than (say)
`entropymode.c` is a historical accident — it is grouped with
"static numeric constants of the bitstream" rather than with "mode
probability tables."

---

## 6. The coefficient-token tree

### 6.1 The tree: `vp8_coef_tree`

```c
const vp8_tree_index vp8_coef_tree[22] = {
  -DCT_EOB_TOKEN, 2,                       /* 0 = EOB        */
  -ZERO_TOKEN,    4,                       /* 1 = ZERO       */
  -ONE_TOKEN,     6,                       /* 2 = ONE        */
   8, 12,                                  /* 3 = LOW_VAL    */
  -TWO_TOKEN,    10,                       /* 4 = TWO        */
  -THREE_TOKEN, -FOUR_TOKEN,               /* 5 = THREE      */
  14, 16,                                  /* 6 = HIGH_LOW   */
  -DCT_VAL_CATEGORY1, -DCT_VAL_CATEGORY2,  /* 7 = CAT_ONE    */
  18, 20,                                  /* 8 = CAT_THREEFOUR */
  -DCT_VAL_CATEGORY3, -DCT_VAL_CATEGORY4,  /* 9 = CAT_THREE  */
  -DCT_VAL_CATEGORY5, -DCT_VAL_CATEGORY6   /* 10 = CAT_FIVE  */
};
```

This is the **token alphabet's binary decoding tree**, encoded in
libvpx's standard tree-code form (see `treecoder.h:37`): an int8 array
where each pair is one node. A negative entry means "stop, emit
symbol `-value`"; a non-negative entry is the index of the next node
in the same array. Walking always begins at index 0.

A worked walk for "decode a TWO":

1. Start at node 0. Read bit, prob `p[0]`. Bit=1 → go to index `2`.
2. Node at index 2 = `[-ONE_TOKEN, 6]`. Bit=1 → go to index `6`.
   Wait — to reach TWO we need bit=0 at the ONE node first. Reread:
   Node 1 (entries at idx 2,3): `[-ONE_TOKEN, 6]`, but read at node
   #1 with prob `p[1]`. Bit=0 → emit ONE; Bit=1 → go to index `6`
   (node 3, the LOW_VAL split).
3. From the LOW_VAL split, bit=0 selects index `8` (TWO/THREE
   subtree), bit=1 selects index `12` (HIGH_LOW subtree).
4. At node 4 (entries 8,9): `[-TWO_TOKEN, 10]`. Bit=0 → emit TWO.

The node indices in the comments (`/* 1 = ZERO */`, …) are the
**node numbers** that the probability array is indexed by (i.e.,
`i >> 1` where `i` is the byte offset into the tree). RFC 6386 §13.5
shows this same tree in Figure 12.

The 11 internal nodes are exactly `ENTROPY_NODES = 11`
(`entropy.h:37`), which sets the innermost dimension of every
coefficient-probability table in the file.

### 6.2 The pre-computed encodings: `vp8_coef_encodings`

```c
vp8_token vp8_coef_encodings[MAX_ENTROPY_TOKENS] = {
  { 2, 2 },  { 6, 3 },   { 28, 5 },  { 58, 6 },  { 59, 6 },  { 60, 6 },
  { 61, 6 }, { 124, 7 }, { 125, 7 }, { 126, 7 }, { 127, 7 }, { 0, 1 }
};
```

The same tree, inverted: `vp8_coef_encodings[t]` is `{bits, len}`
giving the bit-pattern and length the encoder would write for token
`t`. It is generated by `vp8_tokens_from_tree(vp8_coef_encodings,
vp8_coef_tree)` (the comment above the table notes this — it's only
materialised for the encoder).

For the decoder build this table is technically unused. It is kept for
build symmetry with the encoder and for tooling that wants to map
token values back to bit-lengths (e.g., RD-cost approximations).

`MAX_ENTROPY_TOKENS = 12` because the alphabet is `{ZERO, ONE, TWO,
THREE, FOUR, CAT1, CAT2, CAT3, CAT4, CAT5, CAT6, EOB}`.

### 6.3 Extra-bit trees: `Pcat1`–`Pcat6`, `cat1`–`cat6`

```c
static const vp8_prob Pcat1[] = { 159 };
static const vp8_prob Pcat2[] = { 165, 145 };
static const vp8_prob Pcat3[] = { 173, 148, 140 };
static const vp8_prob Pcat4[] = { 176, 155, 140, 135 };
static const vp8_prob Pcat5[] = { 180, 157, 141, 134, 130 };
static const vp8_prob Pcat6[] = { 254, 254, 243, 230, 196, 177,
                                  153, 140, 133, 130, 129 };
```

Tokens 0..4 transmit their exact magnitudes (0, 1, 2, 3, 4) directly.
Tokens 5..10 are **range categories**: each represents an interval of
absolute values, and the exact value within the interval is sent as
"extra bits". For example, `DCT_VAL_CATEGORY1` covers magnitudes 5–6
(one extra bit, base 5), `DCT_VAL_CATEGORY6` covers 67–2048
(eleven extra bits, base 67).

The extra bits are themselves arithmetic-coded with **fixed
probabilities** (constant per category, see comment in source line 93).
They do not adapt — knowing only the category, the conditional
distribution of the residual magnitude is approximated by these probs,
which were tuned offline. Concretely:

| Category | Range    | Extra bits | Prob table |
|----------|----------|------------|-----------:|
| 1        | 5–6      | 1          | `Pcat1`    |
| 2        | 7–10     | 2          | `Pcat2`    |
| 3        | 11–18    | 3          | `Pcat3`    |
| 4        | 19–34    | 4          | `Pcat4`    |
| 5        | 35–66    | 5          | `Pcat5`    |
| 6        | 67–2048  | 11         | `Pcat6`    |

The numbers come from RFC 6386 §13.2, Table 13-5.

Each category also has a **tree** for these extra bits. The trees are
trivial binary chains:

```c
static const vp8_tree_index cat1[2]  = { 0, 0 };
static const vp8_tree_index cat2[4]  = { 2, 2, 0, 0 };
static const vp8_tree_index cat3[6]  = { 2, 2, 4, 4, 0, 0 };
…
static const vp8_tree_index cat6[22] = { 2, 2, … 20, 20, 0, 0 };
```

The shape is: read bit using `Pcat[k][0]` to choose the high bit;
proceed to next node, read with `Pcat[k][1]`; and so on for `Len`
bits, MSB first. The surrounding comment in `entropy.c` shows the
exact `init_bit_tree()` generator. A leaf of value 0 means "done,
return accumulator".

### 6.4 The dispatch table: `vp8_extra_bits`

```c
const vp8_extra_bit_struct vp8_extra_bits[12] = {
  { 0, 0, 0, 0 },         { 0, 0, 0, 1 },          { 0, 0, 0, 2 },
  { 0, 0, 0, 3 },         { 0, 0, 0, 4 },          { cat1, Pcat1, 1, 5 },
  { cat2, Pcat2, 2, 7 },  { cat3, Pcat3, 3, 11 },  { cat4, Pcat4, 4, 19 },
  { cat5, Pcat5, 5, 35 }, { cat6, Pcat6, 11, 67 }, { 0, 0, 0, 0 }
};
```

One row per token (including EOB at index 11), holding
`{tree, prob, Len, base_val}`:

- `tree`, `prob`: pointers into the `cat*` and `Pcat*` tables (NULL
  for tokens that have no extra bits).
- `Len`: number of extra bits.
- `base_val`: the value to add to the decoded extra-bit accumulator
  to obtain the final magnitude. For tokens 0..4 this is simply the
  token's literal value (0..4); for the categories it is the
  inclusive lower bound of the range.

The detokenizer in `detokenize.c` does not call through this table —
it inlines a hand-rolled equivalent (`GetCoeffs`, see lines 100–124)
with the same constants (`159`, `165`, `145`, …) hard-coded for
maximum throughput on the very hot path. So `vp8_extra_bits` exists
chiefly for encoder use and for reference implementations (e.g.,
slow-path decoders, fuzz harnesses) that prefer to walk the table.

The duplication is deliberate: the canonical numerical values live
here, and the inner-loop copy in `detokenize.c` is kept consistent
by code review. Any change to one must be mirrored in the other.

### Why this entire tree shape

The token tree was designed so that the most common tokens (ZERO and
EOB) are at depth 1–2, while the rarest (the high-category values
that encode large magnitudes) are at depth 5–6. Combined with the
adaptive probabilities, this gives a near-Huffman bit-length for the
common values and a fallback range-code for the rare ones, without
the trouble of an actual Huffman code (which would require
recomputing tables per frame).

---

## 7. The coefficient-probability cube: `default_coef_probs`

```c
#include "default_coef_probs.h"

void vp8_default_coef_probs(VP8_COMMON *pc) {
  memcpy(pc->fc.coef_probs, default_coef_probs, sizeof(default_coef_probs));
}
```

### What `default_coef_probs` is

The default initial probability table for every internal node of the
coefficient tree, in every context. It is a four-dimensional array:

```c
static const vp8_prob default_coef_probs
    [BLOCK_TYPES][COEF_BANDS][PREV_COEF_CONTEXTS][ENTROPY_NODES];
```

with dimensions `4 × 8 × 3 × 11 = 1,056` bytes. Each axis indexes:

| Axis | Constant            | Range | Meaning                                                          |
|-----:|---------------------|------:|------------------------------------------------------------------|
| 1    | `BLOCK_TYPES`       | 4     | block class (see below)                                          |
| 2    | `COEF_BANDS`        | 8     | output of `vp8_coef_bands[zig-zag idx]`                          |
| 3    | `PREV_COEF_CONTEXTS`| 3     | spatial-neighbour count or previous-token class                  |
| 4    | `ENTROPY_NODES`     | 11    | one entry per internal node of `vp8_coef_tree`                   |

**Block types (entropy.h:60):**

| Type | Meaning                                                                                                      |
|------|--------------------------------------------------------------------------------------------------------------|
| 0    | Luma AC coefficients of a Y block whose DC was extracted into a Y2 block (so this block's "DC" slot is skipped). |
| 1    | The single Y2 (Walsh) block's coefficients.                                                                  |
| 2    | Chroma (U or V) coefficients — these are full 4×4 blocks with their own DC.                                  |
| 3    | Luma coefficients of a Y block whose DC was *not* extracted (the `B_PRED` / `SPLIT_MV` case).                |

The detokenizer's `vp8_decode_mb_tokens` selects the block type per
block: `coef_probs = fc->coef_probs[1]` for Y2, `[0]` for Y-with-Y2,
`[3]` for Y-no-Y2, `[2]` for UV (`detokenize.c:165–193`).

**Coefficient bands** (axis 2) are produced via `vp8_coef_bands` as
described in §2.

**Previous-coef contexts** (axis 3) are produced as described in §3.

**Entropy nodes** (axis 4) are exactly the 11 internal nodes of
`vp8_coef_tree`. The semantics of each probability are:

| Node | Probability that the bit is 0 (i.e., …) |
|------|-----------------------------------------|
| 0    | `Pr(EOB)` — first split, EOB vs continue |
| 1    | `Pr(ZERO   | not EOB)`                  |
| 2    | `Pr(ONE    | nonzero)`                  |
| 3    | `Pr(LOW    | not ONE)` — TWO/THREE/FOUR vs CAT_*  |
| 4    | `Pr(TWO    | LOW)`                      |
| 5    | `Pr(THREE  | not TWO, in LOW)`          |
| 6    | `Pr(CAT1or2| HIGH)`                     |
| 7    | `Pr(CAT1   | CAT1/CAT2)`                |
| 8    | `Pr(CAT3or4| not CAT1/CAT2)`            |
| 9    | `Pr(CAT3   | CAT3/CAT4)`                |
| 10   | `Pr(CAT5   | CAT5/CAT6)`                |

This direct correspondence with `vp8_coef_tree`'s node numbering is the
whole reason the entropy nodes are exactly 11.

### Why these particular numbers

The default table is the result of offline training on a corpus and is
specified verbatim in RFC 6386 §13.5 (the table that begins
"For Block Type 0, Coefficient Band 0…"). It serves as the initial
state of `pc->fc.coef_probs` at every keyframe; the compressed header
of each frame can then update individual entries with
`vp8_coef_update_probs` (§8).

A few features of the data worth noting:

- **Band 0 of block type 0 is all 128s** (lines 24–26 of
  `default_coef_probs.h`). This is dead state: block type 0 is "luma
  AC", whose DC slot is skipped (the DC went into Y2), so band 0
  is never indexed for this block type. 128 is the maximum-entropy
  fallback that would behave neutrally if it were ever read.
- **Band 7 (the high-frequency tail) frequently has 128 in the
  trailing nodes**, because once you have seen a high-frequency
  category-token, the chance of seeing another in the same block is
  so low that the table never needs to refine past the first few
  nodes.
- The values for `Pr(ZERO)` (node 1) are generally **high** (often
  > 200) because most coefficients in a natural-image transform are
  zero.

### What `vp8_default_coef_probs(VP8_COMMON *)` does

The one-line function copies `default_coef_probs` into
`pc->fc.coef_probs`. It is called from `decodeframe.c:828` whenever
the decoder needs to reset entropy state — namely on every keyframe
and on any frame where the encoder has chosen not to persist its
previous-frame probability updates (`refresh_entropy_probs == 0`,
in which case the working table `fc.coef_probs` is restored before the
next frame from the last saved copy in `lfc.coef_probs`; the keyframe
codepath restores from the static defaults instead).

The `memcpy` is the only **write** in the entire file. Everything else
is `const`.

---

## 8. Per-frame update gating: `vp8_coef_update_probs`

```c
const vp8_prob vp8_coef_update_probs
    [BLOCK_TYPES][COEF_BANDS][PREV_COEF_CONTEXTS][ENTROPY_NODES]
    = { … };          /* coefupdateprobs.h */
```

### What it is

A second 4-D array with the **same shape** as `default_coef_probs`.
Each entry is the probability that, in the current frame's compressed
header, the corresponding entry of `fc.coef_probs` will be
**overridden** with a fresh 8-bit value.

### Why a separate table

Most of the 1,056 coefficient probabilities rarely benefit from
per-frame retuning, so the encoder usually wants to leave them alone.
A naïve flag-per-entry would cost ~1,056 bits per frame just to say
"no change" for everything. Instead, the encoder writes one bit per
entry but **arithmetic-coded with this very biased probability**, so
that the common "no change" case costs nearly zero bits. Looking at
the values in `coefupdateprobs.h`:

- The vast majority of entries are `255` (== `1 - 1/256`), meaning
  "almost certainly no update". An arithmetic 0 against probability
  255 costs essentially nothing.
- Entries that *are* commonly updated have lower values (`176`, `223`,
  `186`, …) — these are the parts of the table the encoder found
  worth retuning per frame on the training corpus.

### How it is used

In `decodeframe.c:1175–1187` the parser walks the entire 4-D index
space and, for each entry, does:

```c
if (vp8_read(bc, vp8_coef_update_probs[i][j][k][l])) {
  pc->fc.coef_probs[i][j][k][l] = (vp8_prob)vp8_read_literal(bc, 8);
}
```

In a steady-state interframe with no real probability changes, that
loop transmits ~1,056 bits "0", each at probability 255 — under 10
bytes total. This is the mechanism that gives VP8 its frame-by-frame
entropy adaptivity at almost no bitstream cost.

### Invariants

- Same dimensions as `default_coef_probs`, so the parser can use the
  same nested-loop index.
- Values fixed by RFC 6386 §13.5; never modified at runtime.

---

## 9. What is *not* in this file

For orientation, it is worth listing what `entropy.c` does **not**
contain, since the file's name might suggest otherwise:

- **The bool decoder itself** (`vp8/decoder/dboolhuff.[ch]`) — only
  the `vp8_norm` LUT lives here.
- **Mode-tree probabilities** (intra mode, inter mode, MV partition,
  segment ID, etc.) — these live in
  `vp8/common/entropymode.c` and `vp8/common/vp8_entropymodedata.h`.
- **MV probabilities** — in `vp8/common/entropymv.c`.
- **The tree-walker** — generic, in `vp8/common/treecoder.c`.

The file is, in spirit, "the constants annex to RFC 6386 §13" in C —
nothing more, nothing less.
