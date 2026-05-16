# `vp8/common/treecoder.c` — generic binary tree coder utility

## Role in the decoder

VP8 codes almost every non-coefficient syntax element — Y/UV intra
modes, partition types, segment IDs, motion-vector magnitudes,
mb-skip flags, and the DCT-coefficient token alphabet itself — as a
small **prefix code over a binary tree**. The wire format is "starting
at the root, at each interior node read one bool with a node-specific
probability; go left on 0, right on 1; when you reach a leaf, emit its
symbol." Decoding such a code on the wire is performed by
`vp8_treed_read` in `vp8/decoder/treereader.h` (overview
[§5.2](../vp8_technical_overview.md#52-tree-codes)). The trees
themselves are not data structures with pointers — they are encoded as
small `int8_t` arrays, one per syntax element, defined in
`entropy.c`, `entropymode.c`, `entropymv.c`, and `vp8_entropymodedata.h`.

`treecoder.c` is the *generic utility* that sits behind that scheme.
It has nothing to do with decoding a wire bitstream directly; it
contains the metadata transformations that let the encoder and decoder
share a single, declarative description of each tree:

1. Given a tree and an array of leaf events (frequency counts), compute
   the eleven (or seven, or three, …) interior-node probabilities that
   minimise the expected bit cost — `vp8_tree_probs_from_distribution`.
2. Given a tree, build a lookup table mapping leaf-symbol → (bit-string,
   length) so the encoder can emit a symbol in one branch-free table
   read — `vp8_tokens_from_tree`.

In a pure-decoder build the first function is dead code (it is only
used to recompute prob tables when the encoder decides to push an
update on the wire), and the second is called exactly once per process
lifetime, from `vp8_coef_tree_initialize` in `entropy.c:84` — but the
file ships in the common library because the tree-data format itself
is shared with the encoder, and the comment block in `treecoder.h`
that documents the array layout is read by every contributor who needs
to extend or debug the prob-update path. We treat it here as the
authoritative description of the tree encoding.

The technical overview does not call out this file by name but its
output formats are used in
[§5.2 (tree codes)](../vp8_technical_overview.md#52-tree-codes),
[§9.1 (coefficient token tree)](../vp8_technical_overview.md#91-coefficient-token-tree),
and the mode-tree references throughout §7 and §8.

`treecoder.c` is small: three static helpers (one private), two
exported functions, no global state, no allocations. The whole file
fits on one screen. Its weight is in the **convention** it documents,
not the code.

---

## The tree-as-`int8_t[]` encoding

Quoting the header (`treecoder.h:37`):

```c
/* We build coding trees compactly in arrays.
   Each node of the tree is a pair of vp8_tree_indices.
   Array index often references a corresponding probability table.
   Index <= 0 means done encoding/decoding and value = -Index,
   Index > 0 means need another bit, specification at index.
   Nonnegative indices are always even;  processing begins at node 0. */
```

So a tree of `n` leaves is an array of exactly `2*(n-1)` signed
bytes (`typedef signed char vp8_tree_index`). The `n-1` interior nodes
each occupy two consecutive entries: the *left child* (taken on bool=0)
followed by the *right child* (taken on bool=1). A child slot holds
either:

- A **positive even integer** — the byte offset of the next interior
  node within the same array; or
- A **non-positive integer** — `-symbol`, where `symbol` is the leaf
  value to emit.

Because positive indices are byte offsets to interior nodes and
interior nodes are pairs, "nonnegative indices are always even" is an
invariant — and it is exactly what lets the prob index `i >> 1` in
`treed_read` and in `branch_counts` below address a per-node
probability table indexed `[0..n-2]`.

The canonical example is `vp8_coef_tree[22]` (`entropy.c:70`),
quoted in the overview:

```c
const vp8_tree_index vp8_coef_tree[22] = {
  -DCT_EOB_TOKEN,  2,     /* root: bit 0 ⇒ EOB,   bit 1 ⇒ next      */
  -ZERO_TOKEN,     4,
  -ONE_TOKEN,      6,
   8, 12,
  -TWO_TOKEN,      10,
  -THREE_TOKEN,   -FOUR_TOKEN,
  14, 16,                 /* high-category sub-tree                 */
  -DCT_VAL_CATEGORY1,  -DCT_VAL_CATEGORY2,
  18, 20,
  -DCT_VAL_CATEGORY3,  -DCT_VAL_CATEGORY4,
  -DCT_VAL_CATEGORY5,  -DCT_VAL_CATEGORY6,
};
```

Twelve leaves, eleven interior nodes, twenty-two entries. Index 0 is
the root; reading bool 0 yields `-DCT_EOB_TOKEN` (leaf), reading bool 1
moves to offset 2; from offset 2, bool 0 yields `-ZERO_TOKEN`, bool 1
moves to offset 4; and so on.

The shape of the tree is purely the *Huffman shape over the symbol
alphabet* — there is no `vp8_prob` baked into it. The probabilities
are a separate `vp8_prob probs[n-1]` array passed alongside it at
decode time, indexed by interior-node number `i >> 1`. This separation
is what makes the whole scheme so cheap to maintain: you can change a
tree's shape without touching the decoder; you can change a tree's
probabilities (and they do change every frame for many trees, via
prob-update syntax in the frame header) without touching the tree.
Recovering the leaf-to-codeword mapping that this shape implies is the
job of `vp8_tokens_from_tree`; recovering the optimal probabilities
for the interior nodes from a sample distribution is the job of
`vp8_tree_probs_from_distribution`.

The `vp8_treed_read` walker (`treereader.h:30`) consumes the tree in
five trivial lines:

```c
vp8_tree_index i = 0;
while ((i = t[i + vp8_read(r, p[i >> 1])]) > 0) {
}
return -i;
```

That loop is the entire decoder side of every tree-coded element. The
arithmetic decoder underneath returns 0 or 1; we add it to `i` to pick
the left or right child slot; if the slot is positive we keep going,
otherwise we negate and return. Everything in `treecoder.c` exists to
make that one loop legitimate.

---

## `tree2tok` — leaf-symbol → (bit-string, length) table builder

```c
static void tree2tok(struct vp8_token_struct *const p, vp8_tree t,
                     int i, int v, int L) {
  v += v;
  ++L;

  do {
    const vp8_tree_index j = t[i++];

    if (j <= 0) {
      p[-j].value = v;
      p[-j].Len = L;
    } else {
      tree2tok(p, t, j, v, L);
    }
  } while (++v & 1);
}
```

A small recursive depth-first walk. The state is:

- `i` — index of the *next child slot to examine* in the tree array.
- `v` — the bitstring read so far on the path from the root, packed
  high-bit-first into an `int`.
- `L` — the depth of the current node (= length of `v`).

At every call site we are about to inspect *two* sibling slots starting
at position `i`. The first thing the function does is `v += v; ++L;`,
appending a "0" bit to `v` and incrementing the depth — this prepares
the LSB so that the do-while loop can flip it from 0 to 1 at the end of
each pass via `++v & 1`. After the first iteration the bit is 1, so the
condition becomes 0 and the loop terminates after exactly two passes:
once for the left child, once for the right child.

For each slot the body distinguishes the two cases of the encoding:

- **Leaf (`j <= 0`)**: write the codeword `v` and its length `L` into
  output slot `p[-j]`. The negation un-does the tree's convention so
  that `-j` is the leaf's *symbol value*, not an offset.
- **Interior (`j > 0`)**: recurse into the subtree at offset `j`,
  passing the bitstring built so far.

The output array `p` is indexed by *leaf symbol value*, so a tree of
twelve symbols (the coef tree) produces twelve `vp8_token_struct`
entries `{ value, Len }`. From any leaf symbol the encoder can now,
in O(1), emit the right sequence of bools at the right probabilities
by reading off `Len` bits from `value` MSB-first and calling
`vp8_write` once per bit.

### Why a recursion bound matters here

The recursion depth is the tree's height, not its size. VP8 trees are
small (the deepest is the coef tree at eight levels), and there are no
user-controllable inputs to this function — all trees are compile-time
constants in `entropy.c`, `entropymode.c`, etc. So the stack usage
is fixed by the source.

### Invariants

- On entry to `tree2tok(p, t, i, v, L)`, `t[i]` and `t[i+1]` are a
  sibling pair of child slots — guaranteed by the
  "nonnegative indices are always even" rule of the tree encoding.
- On exit, every leaf reachable from offset `i` has been written into
  `p[-leaf]`. There is no return value; the entire effect is on `*p`.
- The output table is *unordered* with respect to tree-walking; it is
  indexed by leaf value, so the caller does not have to know which
  leaves are short or long.

### `vp8_tokens_from_tree` and `vp8_tokens_from_tree_offset`

```c
void vp8_tokens_from_tree(struct vp8_token_struct *p, vp8_tree t) {
  tree2tok(p, t, 0, 0, 0);
}

void vp8_tokens_from_tree_offset(struct vp8_token_struct *p, vp8_tree t,
                                 int offset) {
  tree2tok(p - offset, t, 0, 0, 0);
}
```

Trivial public wrappers that start the recursion at the root.
`vp8_tokens_from_tree` assumes the symbol alphabet starts at zero;
`vp8_tokens_from_tree_offset` allows a non-zero base symbol value by
backing `p` up so that the eventual `p[-j]` stores still land in the
caller's array. This is used for trees whose leaves are an enum whose
first member is not 0 — e.g., the prediction-mode trees, which start at
`DC_PRED` = 0 but where some adjacent trees omit one mode.

Inside the minimal VP8 decoder build this function has exactly one
caller: `vp8_coef_tree_initialize` (`entropy.c:84–87`), invoked once
from `initialize_dec` to fill `vp8_coef_encodings[12]`. The decoder
itself never consumes that table — the encoder does — but the
initialiser ships in `common/` and so does this file.

---

## `branch_counts` — fold leaf frequencies into per-node counts

```c
static void branch_counts(int n,
                          vp8_token tok[/* n */], vp8_tree tree,
                          unsigned int branch_ct[/* n-1 */][2],
                          const unsigned int num_events[/* n */]) {
  const int tree_len = n - 1;
  int t = 0;

  assert(tree_len);

  do {
    branch_ct[t][0] = branch_ct[t][1] = 0;
  } while (++t < tree_len);

  t = 0;

  do {
    int L = tok[t].Len;
    const int enc = tok[t].value;
    const unsigned int ct = num_events[t];

    vp8_tree_index i = 0;

    do {
      const int b = (enc >> --L) & 1;
      const int j = i >> 1;
      assert(j < tree_len && 0 <= L);

      branch_ct[j][b] += ct;
      i = tree[i + b];
    } while (i > 0);

    assert(!L);
  } while (++t < n);
}
```

This is the bridge from "I saw symbol *s* this many times" to "I went
left from node *k* this many times, right from node *k* that many
times" — i.e., from a *leaf distribution* over the alphabet to a
*Bernoulli distribution at each interior node*. The Bernoulli
parameters are exactly the per-node probabilities the wire format
needs.

The algorithm is the obvious one: for each leaf symbol `t`, look up its
codeword `(value, Len)` in the `tok` table (which was filled by
`vp8_tokens_from_tree`), then *replay* that codeword bit-by-bit
through the tree, adding `num_events[t]` to whichever branch counter we
took at each step.

Walking bit-by-bit:

- `L` starts at the codeword length and is *pre-decremented* each
  iteration, so `(enc >> --L) & 1` reads bit `L-1` first — the MSB,
  matching the order in which `tree2tok` packed it.
- `i` is the current child-slot index in the tree array; initially 0
  (root's left slot).
- `j = i >> 1` is the interior-node number, used to index
  `branch_ct[n-1][2]`. The "nonnegative indices are always even"
  invariant from the header is what makes `i >> 1` an exact, lossless
  conversion.
- `b = 0 or 1` selects which counter to bump and which child slot to
  read next (`tree[i + b]`).
- Loop terminates when we land on a non-positive child (a leaf) — which
  should happen exactly when `L` hits 0, asserted on exit.

The function pre-zeroes the `branch_ct` table; callers therefore do
not have to clear it themselves.

### Invariants and assumptions

- `n >= 2`. The leading `assert(tree_len)` catches the degenerate
  one-symbol alphabet (which would have nothing to code anyway).
- `tok[]` must already describe `tree` — i.e., the caller has run
  `vp8_tokens_from_tree(tok, tree)` first.
- Codewords are read MSB-first; this is consistent with
  `tree2tok`'s `v += v` packing scheme.
- After replaying a codeword, all of its bits are consumed and we land
  on a leaf — the trailing `assert(!L)` is a sanity check on the
  consistency of `tok` and `tree`.

This function is private; its only client is the next one.

---

## `vp8_tree_probs_from_distribution` — distribution → wire probabilities

```c
void vp8_tree_probs_from_distribution(int n,
                                      vp8_token tok[/* n */], vp8_tree tree,
                                      vp8_prob probs[/* n-1 */],
                                      unsigned int branch_ct[/* n-1 */][2],
                                      const unsigned int num_events[/* n */],
                                      unsigned int Pfactor, int Round) {
  const int tree_len = n - 1;
  int t = 0;

  branch_counts(n, tok, tree, branch_ct, num_events);

  do {
    const unsigned int *const c = branch_ct[t];
    const unsigned int tot = c[0] + c[1];

    if (tot) {
      const unsigned int p =
          (unsigned int)(((uint64_t)c[0] * Pfactor) + (Round ? tot >> 1 : 0)) /
          tot;
      probs[t] = p < 256 ? (p ? p : 1) : 255;
    } else {
      probs[t] = vp8_prob_half;
    }
  } while (++t < tree_len);
}
```

The headline function of the file. It takes a per-symbol histogram and
produces:

- `probs[t]` — for each interior node `t`, the probability that the
  arithmetic-coded bool at that node is **0** (= "go left"), scaled to
  the range needed by the wire format.
- `branch_ct[t][2]` — for each interior node `t`, the raw `(left,
  right)` event counts. The encoder needs these when deciding whether
  to send a probability *update* on the wire: if the gain from the
  updated `probs[t]` versus the previous frame's `probs[t]`, weighted
  by `branch_ct[t][0] + branch_ct[t][1]`, exceeds the 8-bit cost of
  the update itself, send the update; otherwise keep the old prob.

The two parameters `Pfactor` and `Round` parameterise the scale:

- `Pfactor` — the numerator multiplier. For the 8-bit boolean coder
  this is conventionally 255 (max representable prob).
- `Round` — if non-zero, add `tot/2` before the division, i.e., round
  to nearest instead of truncating toward zero. This is a quality knob
  the encoder controls.

The per-node formula:

```
p = (c[0] * Pfactor + maybe-round) / tot
probs[t] = clamp(p, 1, 255)
```

Two corner cases get special treatment:

1. `tot == 0` — no events ever passed through this node. The data give
   us nothing to estimate from, so we default to `vp8_prob_half` (=
   128). This makes the node maximally uncertain in either direction
   and yields a balanced 8-bit cost regardless of how it is later
   coded — neutral both as a starting point and as a fallback.
2. `p == 0` after the division — the data say "this branch was *never*
   taken", but we cannot emit prob = 0 on the wire (some arithmetic
   coder implementations cannot represent it, and a future event would
   then be uncodable). We clamp to 1. Symmetrically `p >= 256` clamps
   to 255. The comment `"agree w/old version for now"` records that
   this exact clamp shape is reproduced from the reference encoder,
   not derived.

The `uint64_t` cast in the numerator is necessary because
`c[0] * Pfactor` can overflow 32 bits if the encoder has accumulated
counts over a long sequence and `Pfactor` is 255: the product is up to
`2^32 × 255`. Promoting to 64 bits before the divide avoids that.

### Invariants

- `n >= 2` (inherited from `branch_counts`).
- Output `probs[t] ∈ [1, 255]` always — never 0, never 256.
- `branch_ct[t][0] + branch_ct[t][1]` equals the total number of
  symbols whose codeword visits node `t`. Summed across all leaves
  *reachable through `t`*, this is `Σ_{s ∈ leaves(t)} num_events[s]`.
- The `branch_ct[]` returned is well-defined regardless of whether the
  probability was clamped; the caller's update-cost computation reads
  the raw counts, not the clamped probability.

### How it is used

In a *pure decoder* build this function is not called — the decoder
receives `probs[]` directly on the wire (default tables in
`default_coef_probs.h`, mode/MV tables in `entropy{mode,mv}.c`, plus
per-frame updates parsed by `decodeframe.c`). The function exists so
that:

- The encoder, after each frame, can call it with the per-node event
  counts it accumulated while *encoding* that frame, get the
  probability table that would have minimised that frame's bit cost,
  and decide which (if any) per-node probability updates to send.
- The encoder's training tools (offline) can call it on a corpus to
  derive the default tables that ship in `default_coef_probs.h` and
  friends in the first place.

The "tree-shape and prob-table are independent" property is what makes
this useful: changing the shape of a tree (say, splitting one frequent
leaf into two sub-categories) only requires re-running this function on
existing histograms to obtain a refreshed prob table — no decoder
change is needed beyond linking the new `vp8_tree_index[]` constant.

---

## `vp8bc_tree_probs_from_distribution` — declared, not defined

The header declares a variant:

```c
void vp8bc_tree_probs_from_distribution(int n, vp8_token tok[/* n */],
                                        vp8_tree tree,
                                        vp8_prob probs[/* n-1 */],
                                        unsigned int branch_ct[/* n-1 */][2],
                                        const unsigned int num_events[/* n */],
                                        c_bool_coder_spec *s);
```

intended to derive prob-table widths from a `bool_coder_spec` rather
than from the hard-wired 8-bit `Pfactor` of the main entry point.
There is no definition in `treecoder.c` in the current tree — the
generalisation was apparently never finished, and no caller exists.
The declaration is retained in the header for source-compatibility
with downstream forks that *did* implement it.

---

## What this file is not

A few things one might expect to find here and which are deliberately
absent:

- **No probability-update decoder.** Per-frame updates to the
  mode/MV/coef probability tables are parsed inline in
  `decodeframe.c` and `decodemv.c`, using `vp8_read` directly. They
  do not pass through this file.
- **No tree-walker for decoding.** That is `vp8_treed_read` in
  `vp8/decoder/treereader.h`, quoted above — a five-line static
  inline whose efficiency depends on the bool decoder, not on
  anything in `treecoder.c`.
- **No tree definitions.** The actual `vp8_tree_index[]` arrays for
  the coef, intra mode, inter mode, segment, MV, and partition trees
  live in `entropy.c`, `entropymode.c`, `entropymv.c`, and
  `vp8_entropymodedata.h`. `treecoder.c` is purely the algorithm; the
  data is elsewhere.
- **No allocations, no globals, no state.** Every output buffer is
  caller-supplied; every function is pure given its inputs.

That separation — *one* canonical encoding of "a binary prefix tree"
plus *one* trivial walker, *one* helper that turns a histogram into
optimal probs, and a forest of small constant tables — is what allows
the libvpx VP8 codec to add or tune a syntax element with at most one
edit per concern, instead of carrying a hand-coded if/else ladder per
tree.
