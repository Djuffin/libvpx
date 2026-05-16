# `vp8/decoder/dboolhuff.c` — the VP8 boolean arithmetic decoder

## Role in the decoder

Every non-trivial syntax element in a VP8 bitstream — frame‑level
flags, segment IDs, modes, motion vectors, the per-coefficient tokens,
and even most of the per‑frame probability updates themselves — is
written by a binary arithmetic coder. `dboolhuff.c` and its companion
header `dboolhuff.h` implement the **decoder side** of that coder.
It is the lowest layer of the entropy stack: literally every bit that
isn't part of the 3‑byte uncompressed frame tag is read through this
file.

The technical overview places this code in
[§5.1 "The arithmetic decoder"](../vp8_technical_overview.md#51-the-arithmetic-decoder).
It is the engine on which the higher‑level helpers sit:

- `vp8_decode_value()` (defined in `dboolhuff.h`) reads `n` literal
  bits by running the engine `n` times at p = 128.
- `vp8_treed_read()` (defined in `vp8/decoder/treereader.h`) walks the
  tree‑coded mode / MV / coefficient tables described in
  [§5.2](../vp8_technical_overview.md#52-tree-codes) one bool at a
  time.
- `vp8_decode_frame()` (`decodeframe.c`) calls `vp8dx_start_decode()`
  to open the **residual / first partition** after the uncompressed
  header, and again to open each of the 1..8 token partitions
  (`setup_token_decoder`, see overview §6.3). The decoder context
  therefore holds up to nine concurrently live `BOOL_DECODER`s in
  `VP8D_COMP::mbc[MAX_PARTITIONS]`.

The name "dboolhuff" is historical: VP8's predecessor (VP7) coded some
syntax elements with a small Huffman fallback, hence "bool **+**
Huff". In VP8 the Huffman path is gone; only the boolean coder
remains, but the filename stuck.

The wire format implemented here is normatively specified in
RFC 6386 §7 "Boolean Entropy Decoder", and the renormalisation
algorithm matches the reference C in the appendix to that RFC.

`dboolhuff.c` itself is intentionally tiny — only two non‑inline
functions. The hot path (`vp8dx_decode_bool`) is defined `static
inline` in the header so that the compiler can inline it into the
per‑token loop in `detokenize.c` and the per‑node loop in
`treereader.h`; if it weren't inlined, the cost of one function call
per coded bit would dominate decode time.

## The `BOOL_DECODER` state object

```c
typedef struct {
  const unsigned char *user_buffer_end;
  const unsigned char *user_buffer;
  VP8_BD_VALUE         value;
  int                  count;
  unsigned int         range;
  vpx_decrypt_cb       decrypt_cb;
  void                *decrypt_state;
} BOOL_DECODER;            /* dboolhuff.h:36 */
```

Strictly speaking the struct is defined in the header, not in the `.c`
file, but the bookkeeping discipline lives in `dboolhuff.c` and it is
impossible to explain the implementation without it.

The five state words map directly onto the variables in the RFC's
reference implementation:

- `range` — the current width of the arithmetic interval, kept in
  `[128, 255]` after every renormalisation. It is initialised to 255
  in `vp8dx_start_decode`.
- `value` — a left‑aligned window of upcoming bits. Its **top 8 bits**
  represent the offset inside the current interval; the rest is
  look‑ahead that has been pulled from the input stream but not yet
  consumed. `VP8_BD_VALUE` is `size_t`
  (`typedef size_t VP8_BD_VALUE;` in `dboolhuff.h:27`), so on a 64‑bit
  host the look‑ahead window is 56 bits wide, allowing up to seven
  bytes of refill per pass.
- `count` — number of usable look‑ahead bits in `value`, **biased by
  −8**. The bias is what lets the hot path test `if (count < 0)` to
  decide whether to refill. When the buffer is full, `count == 8 ×
  (sizeof(value) − 1)`; when the algorithmic byte at the top has been
  fully drained, `count` drops below zero and the next decode triggers
  a refill. `vp8dx_bool_error()` exploits this same encoding to detect
  reads past end of stream (see below).
- `user_buffer` / `user_buffer_end` — the half‑open `[user_buffer,
  user_buffer_end)` slice of the caller‑owned partition that has not
  yet been pulled into `value`. `vp8dx_bool_decoder_fill` advances
  `user_buffer` as it refills.
- `decrypt_cb` / `decrypt_state` — optional per‑partition decryption
  hook (`VPXD_SET_DECRYPTOR` control). When non‑null, every refill
  copies the next few bytes through `decrypt_cb` before they enter the
  coder. This is what lets a DRM layer hand the decoder an encrypted
  blob and have it lazily decrypted byte‑by‑byte rather than in one
  shot up front.

### Why `size_t` for `VP8_BD_VALUE`?

VP8 was designed to be fast both on 32‑bit ARM and on 64‑bit x86 /
ARM. Sizing `value` to the host pointer width means the renormalise /
refill loop runs at full word width on both, with no `#ifdef`. The
derived `VP8_BD_VALUE_SIZE` macro,

```c
#define VP8_BD_VALUE_SIZE ((int)sizeof(VP8_BD_VALUE) * CHAR_BIT)
                                                       /* dboolhuff.h:29 */
```

is the only place where the actual width is named, and every shift
constant in the implementation derives from it. The encoder/decoder
round‑trip is bit‑exact regardless of word width because the
arithmetic only depends on the top 8 bits of `value`.

### Why the `VP8_LOTS_OF_BITS` sentinel?

```c
#define VP8_LOTS_OF_BITS (0x40000000)     /* dboolhuff.h:34 */
```

When the input partition has been exhausted, the refill code adds
`VP8_LOTS_OF_BITS` to `count` (rather than feeding more bytes) so that
subsequent `vp8dx_decode_bool` calls keep finding "enough" buffered
bits and never re‑enter the refill path. This means the bit‑decoder
itself has **no branches** for end‑of‑stream — out‑of‑bounds reads
simply return whatever bits happen to be in `value`. Detection of
having gone past the end is deferred to `vp8dx_bool_error()`, which
the caller polls. The header comment notes:

> *"This is meant to be a large, positive constant that can still be
> efficiently loaded as an immediate (on platforms like ARM, for
> example). Even relatively modest values like 100 would work fine."*

`0x40000000` is convenient because it fits in an ARM `mov` immediate
yet is still distinguishable from any legitimate `count`.

## `vp8dx_start_decode` — opening a partition

```c
int vp8dx_start_decode(BOOL_DECODER *br, const unsigned char *source,
                       unsigned int source_sz, vpx_decrypt_cb decrypt_cb,
                       void *decrypt_state);
                                          /* dboolhuff.c:15 */
```

This is the constructor. Its only job is to seed the `BOOL_DECODER`
with the canonical initial state mandated by RFC 6386 §7 — `range =
255`, `value = 0`, eight bits worth of empty buffer — record the
partition slice, and prime the look‑ahead with one full call to
`vp8dx_bool_decoder_fill`.

Two subtleties are encoded in the function:

1. **Null buffer tolerance.** `vp8_decode_frame` and friends sometimes
   pass an empty partition (for example a token partition whose size
   computed out to zero on a corrupt stream). The early return on
   `source_sz && !source` reports an error only for the genuinely
   inconsistent case; `(source = NULL, source_sz = 0)` is accepted and
   produces an inert decoder. The comment explains:

   > *"To simplify calling code this function can be called with
   > |source| == null and |source_sz| == 0. This and
   > `vp8dx_bool_decoder_fill()` are essentially no‑ops in this case."*

2. **UBSan‑clean pointer arithmetic.** The ternary

   ```c
   br->user_buffer_end = source ? source + source_sz : source;
                                          /* dboolhuff.c:24 */
   ```

   avoids forming `NULL + 0`. That expression has defined behaviour by
   the letter of C (`size_t` zero added to a null pointer yields a
   null pointer), but UndefinedBehaviorSanitizer's pointer‑overflow
   check flags it. Keeping the build clean under UBSan is a project
   policy and this idiom appears in several other libvpx files for the
   same reason.

The return value is `0` on success, `1` on the `(NULL, nonzero)`
inconsistency. Callers in `decodeframe.c` propagate the failure
through `vpx_internal_error()` / `longjmp` so that a corrupted stream
tears down cleanly rather than continuing into the decode loop with
garbage state.

## `vp8dx_bool_decoder_fill` — refilling the look‑ahead window

```c
void vp8dx_bool_decoder_fill(BOOL_DECODER *br);   /* dboolhuff.c:38 */
```

This is the slow path. The hot path (`vp8dx_decode_bool`, in the
header) shifts bits out of `value` and decrements `count` as it goes;
when `count` finally goes negative it calls here to top the window
back up with one byte per iteration.

The function reads as a single carefully arranged loop, but it is
doing three things:

### 1. Compute where the next byte should land

```c
int shift     = VP8_BD_VALUE_SIZE - CHAR_BIT - (count + CHAR_BIT);
                                          /* dboolhuff.c:42 */
```

`shift` is the bit‑position inside `value` at which the next input
byte should be ORed. With `count` biased by −8, the algebra collapses
to "top of `value` minus 16, minus however many bits are currently
buffered". The variable goes down by 8 with each byte we shovel in
and the loop stops when there's no longer room for another whole
byte.

### 2. Decide how many bytes we can actually refill

```c
size_t bytes_left = br->user_buffer_end - bufptr;
size_t bits_left  = bytes_left * CHAR_BIT;
int x = shift + CHAR_BIT - (int)bits_left;
                                          /* dboolhuff.c:43-45 */
```

`x` is the **deficit** in bits: how many bits *short* the input buffer
is of being able to top the window up completely. If `x` is
non‑positive there is enough input; the inner loop will run normally
until `shift < 0`. If `x > 0` we are about to read past the end of
the partition; the function still consumes whatever bytes remain, and
then adds `VP8_LOTS_OF_BITS` to `count` to mark the decoder as
permanently "full" of phantom zero bits from here on:

```c
if (x >= 0) {
  count    += VP8_LOTS_OF_BITS;
  loop_end  = x;
}
                                          /* dboolhuff.c:55-58 */
```

The `loop_end` adjustment makes the inner loop stop early at the
exact byte boundary where the input runs out, instead of running off
the end of `user_buffer`. The combination of the `VP8_LOTS_OF_BITS`
bump and the early loop exit is what makes `vp8dx_decode_bool`
branch‑free with respect to EOF — a corrupt or truncated stream still
produces *some* output bits (effectively zeros), and the caller
detects the situation later via `vp8dx_bool_error()`.

### 3. Optionally decrypt en route

```c
unsigned char decrypted[sizeof(VP8_BD_VALUE) + 1];
if (br->decrypt_cb) {
  size_t n = VPXMIN(sizeof(decrypted), bytes_left);
  br->decrypt_cb(br->decrypt_state, bufptr, decrypted, (int)n);
  bufptr = decrypted;
}
                                          /* dboolhuff.c:47-53 */
```

If a decrypt callback is installed, the next few bytes are read into
a small stack buffer through the callback before being shifted into
`value`. The size of `decrypted` is `sizeof(VP8_BD_VALUE) + 1`, which
on a 64‑bit host is 9 — comfortably more than the maximum number of
bytes a single refill will consume. Note that `br->user_buffer` (the
real input pointer) is advanced by the loop just as if no decryption
had happened; only the byte fetched at `*bufptr` is rerouted through
the stack buffer. This is what lets the decryptor be both per‑byte
lazy and stateless across refills.

### Why the loop is structured the way it is

The naive form would be a single `while (count < VP8_BD_VALUE_SIZE -
CHAR_BIT && bufptr < end)` loop. The branch‑pair the file actually
uses (outer "is there enough input?" decision, then a `while (shift
>= loop_end)` inner loop) is equivalent but lets the compiler hoist
the bound check out of the loop body, and gives the EOF case its own
straight‑line path. On hot decode paths every saved branch matters.

### Invariants on entry / exit

- **On entry:** `br->count < 0`, i.e. the hot path has just dropped
  the last fully‑buffered byte. `br->value` is left‑aligned, with
  `count + 8` valid bits remaining at the top.
- **On exit:** either `br->count >=
  VP8_BD_VALUE_SIZE − CHAR_BIT` (window full to within one byte) or
  `br->count >= VP8_LOTS_OF_BITS` (EOF latched). The hot path treats
  both cases identically.

The function does **not** touch `br->range`; renormalisation is
entirely the responsibility of `vp8dx_decode_bool`.

## What's *not* in this `.c` file — but logically belongs to it

The bool decoder is split across `dboolhuff.c` and `dboolhuff.h`. For
completeness, the inline pieces that the `.c` file collaborates with
are:

- **`vp8dx_decode_bool(br, probability)`** — the per‑bit hot path
  (`dboolhuff.h:54`). It computes `split = 1 + (((range − 1) × prob)
  >> 8)`, compares `value` against `split << (VP8_BD_VALUE_SIZE − 8)`
  to pick the branch, then renormalises `range` back up by consulting
  the `vp8_norm[256]` leading‑zero LUT (defined in
  `vp8/common/entropy.c:18` and forward‑declared via
  `DECLARE_ALIGNED(16, extern const unsigned char, vp8_norm[256]);` at
  `dboolhuff.h:46`). Whenever `count` falls below zero,
  `vp8dx_bool_decoder_fill` (this file) is called. The `static
  VPX_NO_UNSIGNED_SHIFT_CHECK` attribute on the inline function
  suppresses a UBSan unsigned‑shift warning for the
  `value <<= shift; range <<= shift` step — UBSan complains because
  `shift` can be zero (when `range` is already in the top byte and
  `vp8_norm[range] == 0`), and the project would rather paper over
  the false positive than reorganise the hot path.

- **`vp8_decode_value(br, bits)`** — read `bits` literal bits at
  probability 128 (`dboolhuff.h:93`). Used for sizes, deltas, raw
  magnitudes, etc.

- **`vp8dx_bool_error(br)`** — EOF / corruption check
  (`dboolhuff.h:104`). The test
  `count > VP8_BD_VALUE_SIZE && count < VP8_LOTS_OF_BITS` is the
  caller‑side decoder of the `VP8_LOTS_OF_BITS` sentinel: if `count`
  ever rises above the size of the buffer for any reason **other**
  than EOF being latched, somebody has decoded bits that don't exist.
  The header comment is unusually long because the encoding is so
  subtle; the gist is that the only legal large values of `count` are
  the `VP8_LOTS_OF_BITS + k` ones produced by `vp8dx_bool_decoder_fill`
  when it ran out of input, so any *other* large value is proof of an
  inconsistency. Callers (`decodeframe.c`, `detokenize.c`,
  `decodemv.c`) poll this after each MB and abort the frame with
  `VPX_CODEC_CORRUPT_FRAME` if it ever returns 1.

## Cross‑references

- The companion `vp8_norm[256]` renormalisation table lives at
  `vp8/common/entropy.c:18` and is described briefly in the technical
  overview §5.1.
- `BOOL_DECODER` is aliased as `vp8_reader` in
  `vp8/common/treecoder.h`; that's why the higher‑level code calls
  `vp8_read(r, p)` rather than `vp8dx_decode_bool(r, p)` — the macro
  hides the longer name.
- Nine bool decoders live inside each `VP8D_COMP` as
  `mbc[MAX_PARTITIONS]` (`vp8/decoder/onyxd_int.h`): index 8 is the
  residual/first partition opened immediately after the uncompressed
  header, and indices `0..N−1` (with `N = 1 << multi_token_partition`)
  carry the coefficient tokens. Each one is initialised through
  `vp8dx_start_decode` and drained by `detokenize.c`'s per‑MB token
  loop.
- The optional decryptor callback hook is wired up by the
  `VPXD_SET_DECRYPTOR` control (see technical overview §2.2) and
  stored on `VP8D_COMP::decrypt_cb / decrypt_state`; those are copied
  into each `BOOL_DECODER` at partition setup time.
