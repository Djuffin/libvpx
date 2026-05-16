# `vpx_dsp/prob.c` — probability arithmetic and the renormalisation table

## Role in the decoder

VP8's bitstream is, almost in its entirety, encoded by a binary
arithmetic coder (see the technical overview, [§5.1 "The arithmetic
decoder"](../vp8_technical_overview.md#51-the-arithmetic-decoder)).
Every coded bit costs one comparison against a single byte-sized
probability — an `uint8_t` in the range `[1, 255]` representing the
likelihood (out of 256) that the next bit is **zero**. The whole
codec — the bool decoder, the tree-code walker, the coefficient
parser, the MV parser — speaks this currency, the `vpx_prob`.

`prob.c` is the small library that

1. owns the **renormalisation lookup table** `vpx_norm[256]` used by
   the arithmetic engine on every coded bit to determine how many
   times its internal `range` must be doubled, and
2. provides the **probability-arithmetic helpers** for converting
   raw `[count_of_zeros, count_of_ones]` frequency tallies into
   `vpx_prob` bytes, optionally blending them with a prior probability
   along a tree-structured prior (Bayesian-style "backward update").

The shared header `vpx_dsp/prob.h` carries the inline portion of this
library; `prob.c` carries the two things that cannot be inlined: a
256-byte read-only table, and one recursive tree-walker.

### What VP8 actually uses

This file is on the VP8 decoder's mandatory file list (`vp8_files.md`,
section A), but VP8 only uses **one** of the two facilities provided:

| Symbol                          | Used by VP8 decoder?                                          |
|---------------------------------|---------------------------------------------------------------|
| `vpx_norm[256]` (the table)     | Yes — indirectly, through `vp8_norm` (see note below).        |
| `vpx_tree_merge_probs` + friends| No. These exist for the **encoder** and for the VP9 decoder's "backward adaptation" pass at the end of each non-keyframe. |

Why is the entire file built then? Because `vpx_dsp/` is a single
translation-unit library shared by VP8, VP9, and their encoders.
`prob.c` lives there so that all four consumers can share the
`vpx_norm` table and the inline probability-arithmetic primitives.
Pulling it apart per codec would duplicate code; leaving the
unreachable functions in for a decoder-only build costs a few hundred
bytes of `.text` and is the cleanest tradeoff.

A subtle bookkeeping note: `vp8/decoder/dboolhuff.[ch]` declares its
**own** copy of the renormalisation table as `vp8_norm[256]`, defined
in `vp8/decoder/dboolhuff.c`. The two tables are byte-for-byte
identical — they encode the same mathematical function, namely
`norm(r) = the smallest k such that (r << k) >= 128`. VP8 uses
`vp8_norm`; the table in `prob.c` is the version used by the
`vpx_dsp/bitreader.h` arithmetic engine (which VP9 uses) and by the
encoder's `bitwriter`. Both tables coexist in a typical libvpx build,
indexed by independent name; the contents must always match.

The technical overview's [§5.1](../vp8_technical_overview.md#51-the-arithmetic-decoder)
shows exactly where the lookup happens in the inner loop:

```c
shift = vpx_norm[(unsigned char)range];   /* bitreader.h:101 */
range <<= shift;
value <<= shift;
count  -= shift;
```

That single table lookup is what makes the arithmetic engine fast: it
replaces a leading-zero count and a conditional loop with one byte
fetch.

---

## Header preamble

```c
#include "./prob.h"
```

The only include. `prob.h` itself pulls in `vpx_config.h`,
`vpx_dsp_common.h`, and `vpx_ports/mem.h` (for `DECLARE_ALIGNED`), so
the `.c` file has no further dependencies — it is a self-contained
data + algorithm unit with no I/O, no allocation, no error paths, and
no platform conditionals.

---

## The renormalisation table — `vpx_norm[256]`

```c
const uint8_t vpx_norm[256] = {
  0, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, …
  …
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
  0, 0, 0, 0, 0, 0, …, 0
};
```

### What it is

For every possible 8-bit value of the bool decoder's `range`
register, `vpx_norm[range]` gives the number of left-shifts required
to drive `range` back into the canonical half-open interval
`[128, 256)`. Concretely:

| `range` value | `vpx_norm[range]` | Explanation                          |
|---------------|-------------------|--------------------------------------|
| `0`           | `0`               | Never occurs in practice; range is always >= 1. |
| `1`           | `7`               | `1 << 7 = 128` — exactly one bit short of 256. |
| `2–3`         | `6`               | `range << 6` lands in `[128, 192)`.  |
| `4–7`         | `5`               | …                                   |
| `8–15`        | `4`               | …                                   |
| `16–31`       | `3`               | …                                   |
| `32–63`       | `2`               | …                                   |
| `64–127`      | `1`               | One shift suffices.                  |
| `128–255`     | `0`               | Already normalised; no shift needed. |

In one sentence: `vpx_norm[r] = clz8(r)`, the number of leading
zero bits in the 8-bit representation of `r` (with `r == 0` mapped to
`0` rather than the mathematically correct `8`, since the engine
never normalises a zero range).

### Why it exists

The arithmetic coder maintains `range` in a sub-octet representation
to maximise coding efficiency: after every coded bit, `range` may
have shrunk to as little as 128 + 1 = 129 (no shift) or as much as
255 (no shift), or down to anywhere in `[1, 127]` (needs shifts).
Renormalisation could be expressed as a `while (range < 128) { range
<<= 1; … }` loop, but coded bits are the hottest path in the decoder:
each pull of the loop, on average, runs barely a few instructions.
The branch in a `while` loop would mispredict on most coded bits.

A single byte-indexed lookup replaces the loop with one memory access
and one branchless shift. On every architecture libvpx targets,
that table fits in a single cache line (it is `DECLARE_ALIGNED(16,
…)` in the header — see below — to ensure it doesn't straddle two
lines), and the entire table is read so frequently that it tends to
live in L1 for the duration of a decode.

### Invariants

- `vpx_norm[0] == 0`. The engine guarantees `range >= 1`, so this
  slot is unreachable; the value is chosen to make accidental reads
  fail loudly (zero shift = no progress = infinite loop, immediately
  visible in testing).
- For all `r` in `[1, 255]`: `(r << vpx_norm[r]) ∈ [128, 256)`.
- `vpx_norm[r]` is monotonically non-increasing in `r` for `r >= 1`.
- The table is exactly 256 bytes; no padding, no escape values.

### Alignment

The header declares it with `DECLARE_ALIGNED(16, extern const
uint8_t, vpx_norm[256])` (`prob.h:100`). The `.c` definition does
not repeat the alignment attribute (only the extern declaration
carries it), but GCC and Clang both honour the attribute at the
definition site through the matching declaration. The 16-byte
alignment is overkill for scalar access but cheap, and it guarantees
the entire table sits within two adjacent 16-byte SIMD-friendly
chunks — handy for any future vectorised reader.

### How it is used

`vpx_norm` has three call-sites in a full libvpx build:

1. **`vpx_dsp/bitreader.h:101`** — the VP9 / encoder bool decoder's
   `vpx_read()` renormalisation step.
2. **`vpx_dsp/bitwriter.h:79`** — the symmetric step in the encoder's
   `vpx_write()`.
3. **`vp9/decoder/vp9_detokenize.c:63, 86`** — a special-cased copy
   of the inner loop for coefficient decoding.

In a **decoder-only VP8 build** the table is technically present but
none of these three sites is compiled in. VP8's own
`vp8/decoder/dboolhuff.{c,h}` carries its own identical table named
`vp8_norm`. The dead copy in `prob.c` survives because the
build system doesn't slice `vpx_dsp/` by codec.

---

## The probability-arithmetic helpers (defined in `prob.h`)

Although the file under study is `prob.c`, the bulk of the
probability-arithmetic API is `static INLINE` in `prob.h`. The one
function in `prob.c` (`vpx_tree_merge_probs`) is the tree-walker that
sits on top of them. To make the rest of this document standalone we
document each helper as if it were defined here, citing `prob.h` for
the source.

The whole stack is built on one act of estimation: given that a
specific binary event has happened `n0` times for outcome 0 and `n1`
times for outcome 1 across the previous frame, what 8-bit probability
should the next frame's arithmetic coder use for that event? The
answer is a maximum-likelihood estimate `p ≈ n0 / (n0 + n1)`,
rendered into the `[1, 255]` byte range, then **blended** with the
previous frame's probability to dampen statistical noise.

### `get_prob(num, den)` — frequency-to-probability conversion

```c
static INLINE vpx_prob get_prob(unsigned int num, unsigned int den) {
  assert(den != 0);
  {
    const int p = (int)(((uint64_t)num * 256 + (den >> 1)) / den);
    // (p > 255) ? 255 : (p < 1) ? 1 : p;
    const int clipped_prob = p | ((255 - p) >> 23) | (p == 0);
    return (vpx_prob)clipped_prob;
  }
}
```

**What.** Rounds `(num / den) * 256` to the nearest integer, then
clamps the result into `[1, 255]`.

**Why.** A `vpx_prob` value of 0 or 256 would make the arithmetic
coder allocate zero range to one of the two outcomes, after which
that outcome can never be coded. The wire format avoids this by
reserving 0 (interpreted as "use parent / default" in some update
contexts) and capping at 255. Probabilities of exactly 0.5 use
`vpx_prob_half = 128`.

The branchless clipping trick `p | ((255 - p) >> 23) | (p == 0)`
deserves a second look:

- `(255 - p) >> 23` is `0` when `p <= 255` and `0x1FFFFFFF…` (a value
  with the top bits set) when `p > 255`. Bit-ORing this with `p`
  saturates `p` to at least `255` when it overflows.
- `(p == 0)` is `1` when `p == 0` and `0` otherwise. Bit-ORing it
  with `p` ensures the final value is at least `1`.

The two ORs commute and together produce the same result as the
commented-out three-way clamp, without branches.

**Invariants.** `den > 0` (assertion). Output ∈ `[1, 255]`.

**How used.** Called by `get_binary_prob` (below) and by
`mode_mv_merge_probs` (`prob.h:92`).

### `get_binary_prob(n0, n1)` — the typical caller-facing form

```c
static INLINE vpx_prob get_binary_prob(unsigned int n0, unsigned int n1) {
  const unsigned int den = n0 + n1;
  if (den == 0) return 128u;
  return get_prob(n0, den);
}
```

**What.** Converts a 2-element frequency tally `[n0, n1]` into the
probability of the **zero** outcome.

**Why.** The vast majority of probability updates in the
codec come in the form "I observed `n0` zeros and `n1` ones; what
probability should I assign to a zero?" Spelling out the addition
`n0 + n1` at every call-site would be both noisy and a place to
introduce overflow bugs.

**Invariants.** No precondition on `n0`/`n1` (both may be zero —
this is the explicit special case). Output ∈ `[1, 255]` if at least
one observation was made, exactly `128` if there were no observations
(maximum-entropy default).

**How used.** Called by `merge_probs` (the generic blender) and by
`tree_merge_probs_impl` — i.e., this is the per-node probability
estimator used when walking a tree-coded distribution.

### `weighted_prob(prob1, prob2, factor)` — convex blend of two probabilities

```c
static INLINE vpx_prob weighted_prob(int prob1, int prob2, int factor) {
  return ROUND_POWER_OF_TWO(prob1 * (256 - factor) + prob2 * factor, 8);
}
```

**What.** Returns `((256 - factor) * prob1 + factor * prob2) / 256`,
rounded to nearest, with `factor` ∈ `[0, 256]` controlling the mix
(`factor = 0` returns `prob1`, `factor = 256` returns `prob2`,
`factor = 128` is the midpoint).

**Why.** Probability adaptation across frames is intentionally
**damped**. The encoder does not jump straight from "previous
frame's estimate" to "this frame's count-based estimate" because a
single short frame can produce a wildly skewed sample. Instead it
linearly interpolates with a confidence-weighted factor — more
observations means more trust in the new estimate. This function is
the workhorse of that interpolation.

The `ROUND_POWER_OF_TWO(x, 8)` macro (from `vpx_dsp_common.h`)
computes `(x + 128) >> 8`, i.e., divide by 256 with proper rounding.

**Invariants.** The header comment says it all: "This function
assumes prob1 and prob2 are already within `[1,255]` range." Because
`factor` ∈ `[0, 256]` and both inputs are in `[1, 255]`, the convex
combination stays in `[1, 255]`; **the result therefore needs no
clamping** and is safe to feed straight back into the arithmetic
coder.

**How used.** Called by `merge_probs` and by `mode_mv_merge_probs`;
hence indirectly by every backward-adaptation step in the encoder
and the VP9 decoder.

### `merge_probs(pre_prob, ct, count_sat, max_update_factor)` — the generic Bayesian blend

```c
static INLINE vpx_prob merge_probs(vpx_prob pre_prob, const unsigned int ct[2],
                                   unsigned int count_sat,
                                   unsigned int max_update_factor) {
  const vpx_prob prob = get_binary_prob(ct[0], ct[1]);
  const unsigned int count = VPXMIN(ct[0] + ct[1], count_sat);
  const unsigned int factor = max_update_factor * count / count_sat;
  return weighted_prob(pre_prob, prob, factor);
}
```

**What.** Combines a prior probability `pre_prob` with the
count-based estimate from `ct[]`, with the mixing weight scaling
linearly with the number of observations up to a saturation limit
`count_sat`.

**Why.** A clean piece of empirical Bayes. With zero observations
the function returns `pre_prob` unchanged (because `count = 0` ⇒
`factor = 0` ⇒ `weighted_prob` returns `pre_prob`). At
`count == count_sat` it returns
`weighted_prob(pre_prob, prob, max_update_factor)`, which gives the
new estimate weight `max_update_factor / 256`. Tunable per call-site:

- Large `max_update_factor` → fast adaptation.
- Large `count_sat` → adaptation requires more evidence before
  reaching full speed.

**Invariants.** `count_sat > 0` (assumed; no assert). `ct` is a
2-element array. `pre_prob ∈ [1, 255]` (enforced upstream by
construction). Output ∈ `[1, 255]`.

**How used.** Not called by the in-tree VP8 decoder; used by the
encoder where it tunes adaptation speed per syntax element.

### `count_to_update_factor[]` — the precomputed blend schedule

```c
// MODE_MV_MAX_UPDATE_FACTOR (128) * count / MODE_MV_COUNT_SAT;
static const int count_to_update_factor[MODE_MV_COUNT_SAT + 1] = {
  0,  6,  12, 19, 25, 32,  38,  44,  51,  57, 64,
  70, 76, 83, 89, 96, 102, 108, 115, 121, 128
};
```

**What.** A 21-entry lookup table giving the mixing factor used by
`mode_mv_merge_probs` for each observation count from 0 through
`MODE_MV_COUNT_SAT = 20`. Each entry is
`round(128 * count / 20)`.

**Why.** Mode and motion-vector probabilities are blended with a
fixed adaptation speed: `MODE_MV_MAX_UPDATE_FACTOR = 128` (i.e., a
50/50 mix once saturated, equally weighting prior and observation).
The header comment spells out the formula; the table avoids one
integer division per probability per frame.

**Invariants.** `count_to_update_factor[0] == 0`,
`count_to_update_factor[20] == 128`, monotonically increasing,
length `MODE_MV_COUNT_SAT + 1`.

### `mode_mv_merge_probs(pre_prob, ct)` — the specialised blend for modes/MVs

```c
static INLINE vpx_prob mode_mv_merge_probs(vpx_prob pre_prob,
                                           const unsigned int ct[2]) {
  const unsigned int den = ct[0] + ct[1];
  if (den == 0) {
    return pre_prob;
  } else {
    const unsigned int count = VPXMIN(den, MODE_MV_COUNT_SAT);
    const unsigned int factor = count_to_update_factor[count];
    const vpx_prob prob = get_prob(ct[0], den);
    return weighted_prob(pre_prob, prob, factor);
  }
}
```

**What.** A specialisation of `merge_probs` with `count_sat = 20`
and `max_update_factor = 128`, using the precomputed table above to
sidestep the division.

**Why.** This is the per-node primitive that drives the recursive
tree merge (next section). It is called many times per frame, so
two optimisations were applied: (i) the explicit `if (den == 0)`
early-out skips the work entirely for cells with no observations,
and (ii) the lookup table replaces an integer divide.

**Invariants.** `pre_prob ∈ [1, 255]`. `ct` is a 2-element array.
`MODE_MV_COUNT_SAT = 20` ⇒ the lookup table has exactly 21 slots
and `count` is always in range.

**How used.** Called pointwise by `tree_merge_probs_impl` (next
section) for each non-leaf node in a tree; also called directly by
VP9's `vp9_entropymode.c` and `vp9_entropymv.c` for non-tree
probabilities (single bits with no child structure: e.g. `skip`,
`intra_inter`, MV sign).

---

## The tree merger — `vpx_tree_merge_probs`

The remaining content of `prob.c` is one externally-visible function
and one static recursive helper. Together they perform a depth-first
walk of a libvpx tree-code description, blending the entire tree of
probabilities in a single pass.

### Background — what is a `vpx_tree`?

From `prob.h`:

```c
typedef int8_t vpx_tree_index;
typedef const vpx_tree_index vpx_tree[];

#define TREE_SIZE(leaf_count) (2 * (leaf_count) - 2)
```

> We build coding trees compactly in arrays.
> Each node of the tree is a pair of `vpx_tree_index`es.
> Array index often references a corresponding probability table.
> Index `<= 0` means done encoding/decoding and value = `-Index`,
> Index `> 0` means need another bit, specification at index.
> Nonnegative indices are always even; processing begins at node 0.

So a tree is a flat array of `int8_t` pairs. Each pair is one
internal node; the two entries are its left and right children
respectively. A non-positive child entry `-leaf` terminates the
walk with leaf index `leaf`; a positive child entry is the array
offset of the child internal node.

The probabilities for such a tree live in a parallel array, one
`vpx_prob` per internal node, indexed by `(node_offset >> 1)`.
This is the indexing convention used throughout this file and
throughout the rest of libvpx's entropy code (see the technical
overview, [§5.2 "Tree codes"](../vp8_technical_overview.md#52-tree-codes)).

### `tree_merge_probs_impl` — the recursive worker

```c
static unsigned int tree_merge_probs_impl(unsigned int i,
                                          const vpx_tree_index *tree,
                                          const vpx_prob *pre_probs,
                                          const unsigned int *counts,
                                          vpx_prob *probs) {
  const int l = tree[i];
  const unsigned int left_count =
      (l <= 0) ? counts[-l]
               : tree_merge_probs_impl(l, tree, pre_probs, counts, probs);
  const int r = tree[i + 1];
  const unsigned int right_count =
      (r <= 0) ? counts[-r]
               : tree_merge_probs_impl(r, tree, pre_probs, counts, probs);
  const unsigned int ct[2] = { left_count, right_count };
  probs[i >> 1] = mode_mv_merge_probs(pre_probs[i >> 1], ct);
  return left_count + right_count;
}
```

**What.** A post-order traversal of the tree rooted at array
offset `i`. For each internal node it (a) recursively obtains the
total observation counts for its left and right sub-trees, (b) feeds
those two totals into `mode_mv_merge_probs` together with the prior
probability for this node, and (c) writes the blended result into
`probs[i >> 1]`. It returns the sum of all leaf counts under this
node, so the parent can in turn aggregate.

**Why post-order.** The probability at an internal node is the
probability of taking the **left** child at that node. To estimate
it from observations we need to know how many times the bitstream
passed through the left subtree versus the right subtree. Those
sums must therefore be computed bottom-up before the parent's
probability can be evaluated. The function elegantly fuses
"aggregate counts" and "blend probability" into a single recursion.

The leaf base case is `l <= 0` (resp. `r <= 0`): the corresponding
count is taken from `counts[-l]`, indexed by the symbol number that
the leaf represents.

**Invariants.**

- `tree[]` is well-formed: every positive entry points to an even
  index within the array, and every recursion terminates.
- `pre_probs[]` and `probs[]` have at least
  `TREE_SIZE(leaf_count) / 2 = leaf_count - 1` slots.
- `counts[]` has one slot per leaf (i.e., per symbol); `counts[k]`
  is the number of times the encoder produced symbol `k` in the
  previous frame's collected statistics.
- For any leaf index `k`, `-k` (a non-positive `vpx_tree_index`)
  must fit in `int8_t`, so trees with more than 128 leaves cannot
  use this representation. In practice the largest VP9 trees have
  ~10 leaves, well within range.
- Recursion depth is bounded by the tree's height. VP9's trees
  have height ≤ ~5, so stack usage is trivial.

**How used.** Strictly the implementation backing
`vpx_tree_merge_probs`; never called directly.

### `vpx_tree_merge_probs` — the public entry point

```c
void vpx_tree_merge_probs(const vpx_tree_index *tree, const vpx_prob *pre_probs,
                          const unsigned int *counts, vpx_prob *probs) {
  tree_merge_probs_impl(0, tree, pre_probs, counts, probs);
}
```

**What.** Trivial trampoline: starts the recursion at node 0 (the
tree root, per the convention in `prob.h`) and discards the returned
total count (which is only useful as an internal accumulator).

**Why a separate non-static function.** Two reasons: (i) it pins
the recursion entry to offset `0`, removing the need for every
caller to remember the convention; (ii) it gives the linker a
single externally visible symbol for callers in `vp9/common/` to
bind to, while keeping the recursive worker `static` (so the
compiler may freely inline it within this translation unit if it
chooses).

**Invariants.** Same as `tree_merge_probs_impl`. The function has
no return value; its effect is entirely a write into `probs[]`.

**How used.** VP9 only. Concretely:

- `vp9/common/vp9_entropymode.c:361, 365, 369, 373, 378` — backward
  adaptation of inter-mode, Y-mode, UV-mode, partition, and
  intra-inter probabilities at the end of every non-keyframe.
- `vp9/common/vp9_entropymv.c:161, 170, 172, 179, 182` — backward
  adaptation of MV joint, class, class0, and fractional-pel
  probabilities.

In the **VP8** decoder build this function is link-reachable (it is
not `static`) but is never called: VP8 does not perform per-frame
backward probability adaptation in the way VP9 does. VP8's
probability updates are explicitly transmitted in the frame header
and applied directly by `decodeframe.c` (see the technical overview,
[§16.10–16.11](../vp8_technical_overview.md#1611-coefficient-probability-updates-rfc-99-134)).
The function is dead weight in a VP8-only build, kept only because
`vpx_dsp/` is not sliced per codec.

---

## Summary of what this file contributes to a VP8 decode

1. **`vpx_norm[256]`** — the 256-byte renormalisation lookup
   referenced by `vpx_dsp/bitreader.h`. The VP8 decoder uses an
   independent identical copy in `vp8/decoder/dboolhuff.c` named
   `vp8_norm`; both must agree byte-for-byte.

2. **`vpx_tree_merge_probs`** — a tree-walking probability blender
   used by VP9's backward-adaptation pass. **Not called** in a
   VP8-only build; present because the file is shared across
   codecs.

3. **Inline helpers in `prob.h`** (`get_prob`, `get_binary_prob`,
   `weighted_prob`, `merge_probs`, `mode_mv_merge_probs`) — the
   probability arithmetic library. Again, called only by VP9 and
   the encoders in the in-tree build; the VP8 decoder's
   probability-update path reads literal 8-bit probability bytes
   straight from the bitstream and so never invokes these.

The file is a hundred lines of code, of which one hundred bytes is
the table and one short function is the only non-data content. Its
out-sized importance comes not from the lines it contains but from
the fact that the table it ships is the inner-loop hot data of every
arithmetic decoder in libvpx.
