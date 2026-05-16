# `vp8/decoder/detokenize.c` — Residual Coefficient Parsing

## Role in the decoder

This is the smallest of the five files in `vp8/decoder/` and one of the
most performance-critical: it is invoked once per non-skipped macroblock
to decode every quantized DCT coefficient of that macroblock from the
arithmetic-coded ("boolean coder") bitstream into the per-MB
`qcoeff[400]` array. After this file is done, `vp8_dequantize_b` /
`vp8_short_inv_walsh4x4` / `vp8_dequant_idct_add` (none of which live
here) take those integers, multiply by the per-plane dequantizer, and
run the inverse transforms.

Step 5 of the per-MB pipeline in
[vp8_technical_overview.md §9](../vp8_technical_overview.md#9-residuals-tokens-dequant-idct-walsh)
("Residuals") is precisely what this file implements. The macroblock
driver — `decode_macroblock` at `vp8/decoder/decodeframe.c:94` —
either calls `vp8_reset_mb_tokens_context` (when `mb_skip_coeff` is set)
to zero the entropy context for the skipped MB, or `vp8_decode_mb_tokens`
to actually parse coefficients:

```
decode_macroblock                            decodeframe.c:104-112
  if (xd->mode_info_context->mbmi.mb_skip_coeff)
    vp8_reset_mb_tokens_context(xd);
  else if (!vp8dx_bool_error(xd->current_bc))
    eobtotal = vp8_decode_mb_tokens(pbi, xd);
```

The same two functions are called from `threading.c:100-103` in the
multithreaded decoder. They are the file's only two public entry
points (declared in `detokenize.h:20-21`).

The file is intentionally a single self-contained translation unit:
the only state it touches is (a) the per-MB `MACROBLOCKD` (read &
write), (b) the per-frame `FRAME_CONTEXT.coef_probs[4][8][3][11]` table
(read only), and (c) the bool decoder behind `x->current_bc`. The
algorithm follows RFC 6386 §13.2–§13.3 ("Decoding of Token Tree" and
"Decoding the DCT Coefficients") almost line-for-line.

## The static tables that drive token decoding

The four file-scope arrays at `detokenize.c:35-47` encode the three
RFC 6386 lookup tables that the inner loop needs. They are intentionally
`static const uint8_t` so the compiler can keep them in `.rodata` and
inline-fold accesses.

### `kBands` — coefficient-band remapping (`detokenize.c:35-38`)

```c
static const uint8_t kBands[16 + 1] = {
  0, 1, 2, 3, 6, 4, 5, 6, 6,
  6, 6, 6, 6, 6, 6, 7, 0 /* extra entry as sentinel */
};
```

VP8 has 16 DCT coefficient positions per 4×4 block, but only 8
coefficient-probability "bands" (`COEF_BANDS = 8`, see
`vp8/common/entropy.h:67`). The middle dimension of
`FRAME_CONTEXT.coef_probs[BLOCK_TYPES][COEF_BANDS][PREV_COEF_CONTEXTS][ENTROPY_NODES]`
(`onyxc_int.h:45`) is indexed by this remapped band number, not by raw
zig-zag position. The mapping is the table from RFC 6386 §13.3
(`coef_bands[]`).

The 17th entry is a deliberate sentinel: in the inner loop (see
`GetCoeffs` below), after decoding a coefficient at position `n` the
code uses `prob[kBands[n]][...]` to fetch the probability context for
the *next* position. When `n` is already 16 the loop should bail out
on its own, but the sentinel guarantees that even if a path reads
`kBands[16]` momentarily (e.g. a speculative load or an off-by-one in
a future refactor) the index does not run off the end of `.rodata`.

A duplicate of this table also lives in
`vp8/common/entropy.c:23` as `vp8_coef_bands[]`. The decoder keeps its
own private copy here because the inner loop is hot and a local symbol
gives the compiler the freedom to fold the address.

### `kCat3..kCat6` and `kCat3456[]` — extra-bit probabilities (`detokenize.c:40-45`)

VP8 represents large coefficients with a token (one of
`DCT_VAL_CATEGORY3..6`, see `entropy.h:30-33`) followed by "extra bits"
that refine the magnitude. Each extra bit has its own fixed probability
(taken straight from RFC 6386 §13.2 Table on "Probabilities of Extra
Bits"). `kCat3`, `kCat4`, `kCat5`, `kCat6` are exactly those
tables — null-terminated so the magnitude-accumulation loop in
`GetCoeffs` can use `for (tab = kCat3456[cat]; *tab; ++tab)`.

The unified pointer-array `kCat3456[]` is indexed by a 2-bit category
number `cat ∈ {0,1,2,3}` corresponding to `DCT_VAL_CATEGORY{3,4,5,6}`.
The two bits are themselves bool-decoded (`p[8]`, `p[9 + bit1]`) and
combined via `cat = 2*bit1 + bit0` — see `detokenize.c:116-118`. The
base values added back (`v += 3 + (8 << cat)` at `detokenize.c:123`)
recover the magnitudes 11, 19, 35, 67 — the category lower bounds
from `entropy.h:30-33`.

For categories 1 and 2 the extra-bit count is small (1 and 2 bits
respectively) and their probabilities (159, 165, 145) are hard-coded
inline at `detokenize.c:109-112` rather than living in a table.

### `kZigzag` — inverse zig-zag (`detokenize.c:46-47`)

```c
static const uint8_t kZigzag[16] = { 0, 1,  4,  8,  5, 2,  3,  6,
                                     9, 12, 13, 10, 7, 11, 14, 15 };
```

Coefficients arrive in zig-zag order — the bitstream sends them
roughly low-frequency-first so that the typical EOB occurs early — but
the IDCT consumes them in raster order. `kZigzag[n]` is the raster-order
position of the coefficient that arrived at zig-zag step `n`. The line
`out[j] = GetSigned(br, v)` at `detokenize.c:128-130` is the only place
this is used: it scatters each decoded magnitude back into its raster
slot in `qcoeff[16]`.

This is the same table as `vp8_default_zig_zag1d[]` in
`vp8/common/entropy.c`. It is duplicated locally for the same reason
as `kBands` — to keep the hot inner loop's symbols local.

## Local macros and typedefs

### `VP8GetBit` (`detokenize.c:49`)

```c
#define VP8GetBit vp8dx_decode_bool
```

Pure renaming. `vp8dx_decode_bool` (defined `static inline` in
`dboolhuff.h:54-91`) is the arithmetic-coder primitive that decodes one
bit against an 8-bit probability. The macro exists only to make the
many call sites in `GetCoeffs` short; there is no behavioral wrapping.

### `NUM_PROBAS = 11`, `NUM_CTX = 3` (`detokenize.c:50-51`)

The two inner dimensions of `coef_probs[block_type][band][ctx][node]`.
`NUM_CTX` is the previous-token complexity (0 / 1 / >1 — see
`PREV_COEF_CONTEXTS = 3` in `entropy.h:87`). `NUM_PROBAS` is the number
of internal nodes of the coefficient tree the arithmetic decoder walks
(`ENTROPY_NODES = 11`, `entropy.h:37`). They are redefined locally
rather than reused so this file does not have to pull in `entropy.h`
just for two integer constants.

### `ProbaArray` (`detokenize.c:54`)

```c
typedef const uint8_t (*ProbaArray)[NUM_CTX][NUM_PROBAS];
```

The actual type of `FRAME_CONTEXT.coef_probs[i]` for a fixed
block-type `i`: a pointer-to-array such that `prob[band][ctx][node]`
indexes naturally. The comment "for const-casting" is slightly
misleading — there is no `const`-cast here; the typedef simply gives
the inner loop a short, accurate name for the 3-D slice it walks.

## How a single block is decoded — `GetCoeffs`

`GetCoeffs` at `detokenize.c:84-140` is the implementation of RFC 6386
§13.3. It decodes one 4×4 block, scatters the magnitudes into the
caller's `int16_t out[16]`, and returns the *zig-zag* position of the
last non-zero coefficient plus one — i.e., what VP8 calls `eob`.
Returning 0 means the block has no coefficients at all (in coding
terms, the first EOB bit was 1, which the RFC explicitly calls out as
serving "more as a CBP bit").

The arguments deserve a note:

- `prob` is the 3-D slice `coef_probs[block_type]` — picked by the
  caller depending on which of the four block types (Y-no-DC, Y2,
  UV, Y-with-DC; see `entropy.h:60`) is in flight.
- `ctx` is the initial previous-token-complexity context, derived from
  the neighbor's nonzero flag (see `vp8_decode_mb_tokens` below). On
  the second and subsequent iterations it is implicit in which `prob[][ctx][]`
  row was selected on the previous iteration.
- `n` is the starting zig-zag position. For block type 1 (the Y plane
  whose DC has been routed through Y2's Walsh) and similarly for the
  Y-with-DC bookkeeping, the DC position is decoded separately; this
  function is asked to skip it by passing `n = 1` (`skip_dc`).

The control flow follows VP8's coefficient tree. After the initial
EOB-vs-non-EOB test (`p[0]`), each iteration:

1. tries `p[1]` (EOB at the next position) — if 0, advance `n` and
   continue with the "zero token" probability row `prob[kBands[n]][0]`;
2. otherwise reads bits against `p[2..9]` to identify the token's
   *category* (one of ZERO / ONE / TWO / THREE / FOUR /
   DCT_VAL_CATEGORY1..6) and assemble the magnitude `v`;
3. for the category branches DCT_VAL_CATEGORY1..6, reads the appropriate
   number of extra bits, then for categories 3..6 walks the
   `kCat3456[cat]` extra-bit-probability table to accumulate the residual
   magnitude;
4. calls `GetSigned` to attach the sign bit, and writes
   `out[kZigzag[n - 1]] = GetSigned(br, v)`;
5. terminates if either the next EOB bit (`p[0]` on the post-token
   probability row) is set, or `n == 16`.

The double `n == 16` check (`detokenize.c:132` *and* `:136`) is not
redundant. The first short-circuits before issuing an EOB read when the
block is already full (saving one bit-decode for the maximum-length
case). The second handles the case where we left the "current
coefficient" branch via the `kBands[]` zero-token path without setting a
coefficient.

`prob[kBands[n]][...]` is the only point where `kBands` is used. The
subscript `[0]`, `[1]`, or `[2]` selects the previous-token-complexity
row for the *next* iteration, based on the magnitude just decoded:
zero ⇒ row 0, magnitude 1 ⇒ row 1, magnitude >1 ⇒ row 2. This is the
"context updates as we go" mechanism described in `entropy.h:70-84`.

The function is **never** marked `inline`; making it inline would not
help since it has exactly one caller (`vp8_decode_mb_tokens`), and the
compiler already inlines it in optimized builds. Keeping it as a named
function makes profiles legible.

## Sign bit + arithmetic-decoder fold — `GetSigned`

`GetSigned` at `detokenize.c:58-79` is a streamlined `vp8dx_decode_bool`
with two simplifications: (a) the split is always at the midpoint
(`split = (range + 1) >> 1`, i.e. probability 128 — sign bits are
equiprobable), and (b) the bit it decodes is interpreted as a sign,
returning either `+value_to_sign` or `-value_to_sign` directly.

```c
if (br->value < bigsplit) {
  br->range = split;
  v = value_to_sign;          // positive
} else {
  br->range = br->range - split;
  br->value = br->value - bigsplit;
  v = -value_to_sign;         // negative
}
```

The renormalization (`range += range; value += value; count--`) is
done in line — note that, unlike `vp8dx_decode_bool`, this code does
*not* use the `vp8_norm[]` shift table. That is safe because at
probability 128 a single decoded bit consumes exactly one range bit,
so the renormalization always needs precisely a single left shift.

The attribute `VPX_NO_UNSIGNED_OVERFLOW_CHECK` (defined in
`vpx_ports/compiler_attributes.h:35`) suppresses UBSan's unsigned-
overflow warning. The accompanying comment cites b/148271109: with a
**corrupt or fuzzed** bitstream, `br->value` can legitimately wrap
around. The decoder will still produce an undefined-but-bounded result
that the surrounding error-detection logic catches; we just don't
want UBSan crashing the fuzzer. The function is otherwise standards-
conformant.

A subtle invariant: `GetSigned` *always* consumes one bit from `br`,
even if `value_to_sign == 0`. The current caller never passes zero
(it only invokes `GetSigned` after `GetCoeffs` has decided the
coefficient is nonzero), so this is harmless.

## Resetting the per-MB context — `vp8_reset_mb_tokens_context`

`vp8_reset_mb_tokens_context` at `detokenize.c:18-29` is the cheap
counterpart to `vp8_decode_mb_tokens`: when the macroblock is signaled
skipped (`mbmi.mb_skip_coeff`), no tokens are coded for it at all, so
the entropy-context bytes — the per-plane "did the neighbor have
nonzero coefficients?" flags that feed `(*a + *l)` in the decoder —
must be cleared to 0 so subsequent MBs do not predict against stale
state.

The context lives in a struct of nine bytes:

```c
typedef struct {
  ENTROPY_CONTEXT y1[4];   // 4 Y subblocks per row/col
  ENTROPY_CONTEXT u[2];
  ENTROPY_CONTEXT v[2];
  ENTROPY_CONTEXT y2;      // the 9th byte
} ENTROPY_CONTEXT_PLANES;   // blockd.h:50-56
```

Notice the unusual `sizeof(ENTROPY_CONTEXT_PLANES) - 1` length passed
to `memset` at `detokenize.c:22-23`. This deliberately clears the
first 8 bytes (the four Y, two U, two V context flags) but **leaves
the 9th byte — `y2` — alone**. The `if (!x->mode_info_context->mbmi.is_4x4)`
block at `:26-28` then clears `a_ctx[8] = l_ctx[8] = 0` only when the MB
uses the second-order Y2 transform (i.e., it is *not* in B_PRED mode).
The reason: when a MB *is* B_PRED (i.e. is_4x4), it has no Y2 block, so
its `y2` context cell must remain whatever the previous MB left there —
otherwise the next non-B_PRED MB to the right or below would see a
spuriously cleared neighbor.

This invariant is the file's single most subtle line and is the
mirror of the asymmetric write in `vp8_decode_mb_tokens` where the Y2
context is only updated when `!is_4x4`.

The casts `(ENTROPY_CONTEXT *)x->above_context` etc. flatten the
9-byte struct into an addressable byte array so that `[0..8]` indexing
works uniformly. `above_context` / `left_context` are themselves
`ENTROPY_CONTEXT_PLANES *` (see `blockd.h:240-241`) — the cast is
required because C does not let you point a `char*` directly into the
named fields of a different struct type even when the layout permits
it.

## The macroblock-level driver — `vp8_decode_mb_tokens`

`vp8_decode_mb_tokens` at `detokenize.c:142-210` is what
`decode_macroblock` calls. Its job is to orchestrate exactly 24 or 25
calls to `GetCoeffs` — one per 4×4 block in the macroblock — picking
the right `coef_probs[]` slice, the right starting position, and the
right `(above + left)` context for each call, and to thread the EOB
count back to the caller via the return value (`eobtotal`) and the
per-block `x->eobs[25]` array.

The MB's 25 coefficient blocks are laid out, in `qcoeff[400]`, as:

```
   [0..15]    16 Y subblocks      (4x4 each, 16 coeffs)
   [16..19]   4 U subblocks
   [20..23]   4 V subblocks
   [24]       the Y2 (Walsh) DC block — 16 coeffs of DC-of-Y
```

(See `blockd.h:211-213`.) Three different coefficient-probability
slices are used:

- `fc->coef_probs[0]` — block type "Y no DC": Y subblocks whose DC has
  been routed through Y2 (i.e., when `is_4x4 == 0`). Decoded with
  `skip_dc = 1`, so `GetCoeffs` starts at zig-zag position 1.
- `fc->coef_probs[1]` — block type "Y2": the single Walsh-transformed
  DC block at `qcoeff_ptr + 24*16`.
- `fc->coef_probs[2]` — block type "UV".
- `fc->coef_probs[3]` — block type "Y with DC": the Y subblocks when
  no Y2 is used (B_PRED). Decoded with `skip_dc = 0`.

The order in which the function visits the blocks matches the
bitstream's interleaving (RFC 6386 §13):

1. **Y2 first, if present.** `:161-178` — if `!is_4x4`, decode the
   Walsh block into `qcoeff_ptr + 24*16` (slot 24), update
   `a_ctx[8] / l_ctx[8]` from its eob, and stash 25's eob in
   `eobs[24]`. The peculiar `eobtotal += nonzeros - 16` deserves
   attention: when the Y2 block has *any* nonzero coefficients the
   subsequent IDCT-skip optimization needs to know the Y2 contributes,
   but the 16 Y-no-DC subblocks each *also* have an implicit DC from
   Y2 that we should not double-count toward `eobtotal`. Subtracting
   16 corrects for the 16 DC coefficients we will *not* be reading
   in the Y subblock loop below.
2. **The 16 Y subblocks.** `:180-191` — at iteration `i`, the above
   context cell is `a_ctx + (i & 3)` (the column within the MB) and
   the left context cell is `l_ctx + ((i & 0xc) >> 2)` (the row). The
   bit-manipulation reads cleanly: blocks are visited in raster order
   `(row, col)`, so `i & 3` is the column and `(i & 0xc) >> 2` is the
   row. Each call advances `qcoeff_ptr += 16` to the next 4×4 block's
   slot.
3. **The 8 UV subblocks.** `:195-207` — `a_ctx += 4; l_ctx += 4;`
   skips past the Y portion of the `ENTROPY_CONTEXT_PLANES` struct
   into the `u[2], v[2]` cells, then for `i ∈ [16,24)`:
   `a_ctx + ((i > 19) << 1) + (i & 1)`,
   `l_ctx + ((i > 19) << 1) + ((i & 3) > 1)`.
   The `(i > 19)` term jumps from U-context (offsets 0,1) to V-context
   (offsets 2,3) at block 20. The remaining arithmetic picks the
   right column/row within the 2×2 layout of each chroma plane.

The two writes to context happen once per call:

```c
*a = *l = (nonzeros > 0);
```

That is, both the above and the left context cells for the just-
decoded block get set to "1 if there was anything", "0 otherwise" — a
single bit broadcast into both directions because the next MB (or the
block to the right within this MB) will read this cell as its own
neighbor. After the loop, `above_context` and `left_context` between
them hold exactly the boundary conditions for the next MB.

`eobtotal` returns the sum of every block's `eob` (with the Y2
adjustment described above). `decode_macroblock` uses it to force
`mbmi.mb_skip_coeff = (eobtotal == 0)` (`decodeframe.c:111`), which in
turn instructs the loop filter to skip this MB. So the return value
is not just a sanity check — it directly controls a later pipeline
stage.

### Invariants and gotchas worth highlighting

- The function never reads `vp8dx_bool_error(bc)`. The caller is
  responsible for that check (`decodeframe.c:106`). If the bool
  decoder has run off the end of its partition,
  `vp8_decode_mb_tokens` will happily decode garbage; the corrupted
  values are bounded by the integer types and will be caught downstream.
- The function assumes `qcoeff[400]` was zeroed before it ran. It only
  *writes* the coefficient positions that turn out to be nonzero;
  every other slot of every block is left untouched. `decode_macroblock`
  satisfies this invariant via a separate `memset` path used when a
  macroblock is skipped (see `decodeframe.c:134, 141`). The
  zig-zag-scattered writes through `out[kZigzag[n-1]]` are the only
  ones — everything else is presumed already-zero.
- The Y2 / Y-no-DC pairing is fragile: if the caller passes a MB with
  `is_4x4 == 1` it must use `coef_probs[3]` and `skip_dc = 0`; passing
  `is_4x4 == 0` must use `coef_probs[0]` and `skip_dc = 1`. The
  function bakes this in at `:165, 173-177`, so it is impossible to
  call wrong from outside — but anyone refactoring should preserve the
  coupling.
- `eobs[24]` is set only in the `!is_4x4` branch. In B_PRED MBs the
  IDCT path explicitly clears `eobs[24]` elsewhere
  (`decodeframe.c:162`), so the omission is intentional.
