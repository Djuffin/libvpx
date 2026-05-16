# `vpx_mem/vpx_mem.c` — the codec's memory front door

## Role in the decoder

Everything the VP8 decoder allocates — the YV12 frame buffers, the
`MODE_INFO` grids, the per-thread scratch rows, the codec's private
`VP8D_COMP` state, even the small `vpx_codec_alg_priv_t` handed back
across the public API boundary — passes through the four entry points
defined in this file:

```c
void *vpx_memalign(size_t align, size_t size);
void *vpx_malloc(size_t size);
void *vpx_calloc(size_t num, size_t size);
void  vpx_free (void *memblk);
```

Internally these are thin wrappers around the platform's `malloc` and
`free`. They add three things that the bare C runtime does not give
you portably:

1. **Alignment.** Callers can ask for an arbitrary power-of-two
   alignment (and `vpx_malloc` always returns at least 16 bytes on
   x86-64 — the default `2 * sizeof(void*)`). This is needed because
   most of the DSP kernels — IDCT, loop filter, sub-pixel filter,
   reconstruction — assume their input buffers are aligned to a SIMD
   register width, and `posix_memalign` / `_aligned_malloc` is not
   universally available.

2. **Integer-overflow checks.** `nmemb * size` is computed in 64-bit
   arithmetic and refused if it would wrap on 32-bit `size_t` or if it
   exceeds `VPX_MAX_ALLOCABLE_MEMORY` (1 TiB on 64-bit, ~2 GiB on
   32-bit). The decoder parses arbitrary, possibly-malicious bitstreams
   and computes buffer sizes from them; without this guard a crafted
   stream could trigger an underflow that allocates a tiny buffer and
   then writes gigabytes into it.

3. **One choke-point for the whole codec.** Because every other VP8 (and
   VP9) source file calls `vpx_malloc`/`vpx_free` rather than `malloc`
   /`free` directly, swapping in a custom allocator, an arena, or an
   instrumentation hook is a single-file change. Embedded ports that
   need to live inside a fixed-size pool only need to rewrite this
   file.

The implementation is small — under 90 lines — and very mechanical:
over-allocate, round the user-visible pointer up to the requested
alignment, and stash the original `malloc` pointer in the word *just
before* it so `vpx_free` can recover it. Everything else in this file
is housekeeping for that one trick.

## The over-allocation trick

Standard `malloc` returns a pointer aligned to a default suitable for
any built-in type (typically 8 or 16 bytes), which is not enough when
the SIMD code wants 32-byte alignment for an AVX load. The textbook
workaround is:

- ask `malloc` for `size + align - 1 + sizeof(size_t)` bytes,
- skip past `sizeof(size_t)` bytes,
- round the resulting address up to the next multiple of `align`,
- save the original `malloc` return value in the `sizeof(size_t)`-byte
  slot immediately preceding the rounded address,
- return the rounded address to the caller.

`vpx_free` reverses this: subtract `sizeof(size_t)` from the user's
pointer to get the slot, read the original `malloc` address back out,
and `free` it.

The "stash" slot is what the header file calls
`ADDRESS_STORAGE_SIZE`:

```c
// include/vpx_mem_intrnl.h
#define ADDRESS_STORAGE_SIZE sizeof(size_t)
```

and the rounding step is the `align_addr` macro from the same header:

```c
#define align_addr(addr, align) \
  (void *)(((size_t)(addr) + ((align) - 1)) & ~(size_t)((align) - 1))
```

Both are the standard "round-up-to-power-of-two" idiom written with
the bit-mask form (`addr + align - 1) & ~(align - 1)`). It assumes
`align` is a power of two — there is no runtime check, but every caller
in the codec passes a power-of-two constant (16, 32, occasionally
`sizeof(void*)`).

With these two definitions in mind, the rest of `vpx_mem.c` reads as
straightforward bookkeeping.

## The maximum-allocation cap

Before any other code runs, the file picks a ceiling on what
`check_size_argument_overflow` will let through:

```c
#if !defined(VPX_MAX_ALLOCABLE_MEMORY)
#if SIZE_MAX > (1ULL << 40)
#define VPX_MAX_ALLOCABLE_MEMORY (1ULL << 40)
#else
// For 32-bit targets keep this below INT_MAX to avoid valgrind warnings.
#define VPX_MAX_ALLOCABLE_MEMORY ((1ULL << 31) - (1 << 16))
#endif
#endif
```

The cap is **1 TiB on 64-bit targets** (where `SIZE_MAX` is
`2^64 − 1`) and **just under 2 GiB on 32-bit targets** (specifically
`INT_MAX − 65535`). Two motivations sit behind the limits:

- *64-bit:* 1 TiB is far larger than any legitimate decoder allocation
  (a typical 4K frame is ~12 MiB), so anything beyond it is almost
  certainly a parsed bitstream field being multiplied into nonsense.
- *32-bit:* the cap is kept strictly below `INT_MAX` because Valgrind
  on 32-bit Linux emits noisy warnings for allocations whose size, when
  cast to a signed int, would be negative. Subtracting `1 << 16`
  leaves headroom so that downstream code (e.g. `n + border`) cannot
  push the request back across the threshold.

A build that needs different limits can predefine `VPX_MAX_ALLOCABLE_MEMORY`
on the compiler command line; the `#if !defined` guard preserves the
override.

## `check_size_argument_overflow` — the safety gate

```c
// Returns 0 in case of overflow of nmemb * size.
static int check_size_argument_overflow(uint64_t nmemb, uint64_t size) {
  const uint64_t total_size = nmemb * size;
  if (nmemb == 0) return 1;
  if (size > VPX_MAX_ALLOCABLE_MEMORY / nmemb) return 0;
  if (total_size != (size_t)total_size) return 0;

  return 1;
}
```

The function returns **1 if `nmemb * size` is safe to allocate, 0 if
it is not**. The three lines after the `nmemb == 0` early-out are the
actual checks:

- `size > VPX_MAX_ALLOCABLE_MEMORY / nmemb` is the canonical
  multiplication-overflow guard written in division form: it asks
  "is the product going to exceed our cap?" without ever computing the
  product itself.
- `total_size != (size_t)total_size` catches the case where the cap
  is wider than `size_t` (i.e. on a 32-bit `size_t` with a 64-bit
  `uint64_t`); the round-trip through `size_t` loses the high bits if
  the value does not fit, and the inequality then fires.
- The `nmemb == 0` early return is required because the division
  `VPX_MAX_ALLOCABLE_MEMORY / nmemb` would otherwise divide by zero.
  A zero-element allocation is also semantically harmless: ISO C lets
  `malloc(0)` return either `NULL` or a freeable non-null pointer, and
  the codec never calls `vpx_calloc(0, …)` deliberately.

Note the parameters are **`uint64_t`**, not `size_t`. Inputs are
narrower (`size_t`) but the intermediate `total_size` and the
comparison against `VPX_MAX_ALLOCABLE_MEMORY` need 64-bit width on
32-bit hosts to detect a 32-bit `size_t` wrap. Callers that pass a
single value (`vpx_memalign`) use `nmemb == 1` so the multiplication
collapses to a single bounds check.

## `get_malloc_address_location`, `set_actual_malloc_address`, `get_actual_malloc_address` — the stash

```c
static size_t *get_malloc_address_location(void *const mem) {
  return ((size_t *)mem) - 1;
}
```

Given the pointer that `vpx_memalign` returned to the caller, this
walks **one `size_t`-sized slot backwards** to find the hidden header
where the original `malloc` address lives. All three of the
`*_malloc_address` helpers are thin wrappers over this same offset.

```c
static void set_actual_malloc_address(void *const mem,
                                      const void *const malloc_addr) {
  size_t *const malloc_addr_location = get_malloc_address_location(mem);
  *malloc_addr_location = (size_t)malloc_addr;
}

static void *get_actual_malloc_address(void *const mem) {
  size_t *const malloc_addr_location = get_malloc_address_location(mem);
  return (void *)(*malloc_addr_location);
}
```

`set_actual_malloc_address` is called once from `vpx_memalign` after
the alignment round-up, to record where the underlying `malloc` block
started. `get_actual_malloc_address` is called once from `vpx_free` to
recover it.

The address is round-tripped through `size_t`, not stored as a `void
*`. This is deliberate: on every modern platform supported by libvpx,
`size_t` is wide enough to hold a pointer, and using `size_t`
sidesteps strict-aliasing concerns about reading and writing a
`void *` through a non-`void *` lvalue.

**Invariant:** the slot at `mem - sizeof(size_t)` must be writeable.
This is guaranteed because `vpx_memalign` allocated
`align - 1 + ADDRESS_STORAGE_SIZE` extra bytes and then shifted the
user pointer *forward* by at least `ADDRESS_STORAGE_SIZE` (it computes
`addr + ADDRESS_STORAGE_SIZE` before rounding up). There is therefore
always at least one `size_t` of headroom before the returned pointer
and still inside the `malloc` block.

## `get_aligned_malloc_size` — sizing the request

```c
static uint64_t get_aligned_malloc_size(size_t size, size_t align) {
  return (uint64_t)size + align - 1 + ADDRESS_STORAGE_SIZE;
}
```

The total number of bytes to ask `malloc` for, decomposed:

- `size` — what the caller wants;
- `align - 1` — worst-case slack between the address `malloc` returns
  and the next multiple of `align` we will round up to;
- `ADDRESS_STORAGE_SIZE` — the header slot for stashing the original
  `malloc` pointer.

The return type is **`uint64_t`**, not `size_t`, so that the addition
itself cannot wrap silently on a 32-bit host before
`check_size_argument_overflow` gets a chance to see it. The caller
(`vpx_memalign`) feeds this `uint64_t` directly into
`check_size_argument_overflow(1, aligned_size)`; only if that check
passes is the value cast back to `size_t` for the actual `malloc`
call.

## `vpx_memalign` — the workhorse

```c
void *vpx_memalign(size_t align, size_t size) {
  void *x = NULL, *addr;
  const uint64_t aligned_size = get_aligned_malloc_size(size, align);
  if (!check_size_argument_overflow(1, aligned_size)) return NULL;

  addr = malloc((size_t)aligned_size);
  if (addr) {
    x = align_addr((unsigned char *)addr + ADDRESS_STORAGE_SIZE, align);
    set_actual_malloc_address(x, addr);
  }
  return x;
}
```

This is the function the whole module exists to provide. Reading the
body line by line:

1. `get_aligned_malloc_size` computes the padded request as a
   `uint64_t`.
2. `check_size_argument_overflow(1, aligned_size)` rejects the
   allocation if the padded request would exceed
   `VPX_MAX_ALLOCABLE_MEMORY` or would not fit in `size_t`. On
   failure the function returns `NULL`, matching the contract of
   `malloc`.
3. `malloc((size_t)aligned_size)` is the actual platform allocation.
   If it fails (`addr == NULL`) the function falls through to the
   `return x` at the bottom, where `x` is still its initial `NULL` —
   again matching `malloc`'s failure convention.
4. `addr + ADDRESS_STORAGE_SIZE` skips past the header slot;
   `align_addr(…, align)` rounds the resulting address up to the next
   multiple of `align`. The cast to `unsigned char *` is required to
   make the `+ ADDRESS_STORAGE_SIZE` pointer arithmetic legal — `void *`
   arithmetic is a GNU extension, not standard C.
5. `set_actual_malloc_address(x, addr)` writes `addr` into the slot
   immediately before `x`. Because `x >= addr + ADDRESS_STORAGE_SIZE`
   (the round-up only ever increases the address), the slot is
   guaranteed to be inside the `malloc` block.

**Used by:** YV12 frame buffers explicitly request 32-byte alignment
(`vpx_memalign(32, …)` in `vpx_scale/generic/yv12config.c`); everything
else goes through `vpx_malloc` / `vpx_calloc`, which call this
function with `DEFAULT_ALIGNMENT`.

**Invariants:**

- The returned pointer is aligned to at least `align` bytes (assuming
  `align` is a power of two).
- The `size_t` immediately preceding the returned pointer holds the
  original `malloc` return value, and must not be touched by the
  caller.
- The caller-visible region `[x, x + size)` lies entirely within the
  `malloc` block `[addr, addr + aligned_size)`.

## `vpx_malloc` — default-aligned convenience

```c
void *vpx_malloc(size_t size) { return vpx_memalign(DEFAULT_ALIGNMENT, size); }
```

A one-liner that fixes `align` to `DEFAULT_ALIGNMENT`. From the
internal header:

```c
#ifndef DEFAULT_ALIGNMENT
#if defined(VXWORKS)
#define DEFAULT_ALIGNMENT 32
#else
#define DEFAULT_ALIGNMENT (2 * sizeof(void *)) /* NOLINT */
#endif
#endif
```

So **`vpx_malloc` guarantees 16-byte alignment on any 64-bit host**
(`2 * sizeof(void *) == 16`), **8-byte alignment on 32-bit hosts**, and
32 bytes on VxWorks (a real-time OS whose DMA engines sometimes
require it). 16 bytes is the SSE register width and the smallest
alignment that an `__m128i` load tolerates; the choice means casual
callers (e.g. `vpx_calloc(num, sizeof(struct …))`) do not have to
think about whether the result is SIMD-safe.

The macro is overridable on the compiler command line for ports that
need stricter alignment. A user-provided override is preserved by the
`#ifndef` guard, allowing platform integrators to tune it without
touching the source.

## `vpx_calloc` — zero-initialised allocation

```c
void *vpx_calloc(size_t num, size_t size) {
  void *x;
  if (!check_size_argument_overflow(num, size)) return NULL;

  x = vpx_malloc(num * size);
  if (x) memset(x, 0, num * size);
  return x;
}
```

The libc `calloc` is *not* used. Why? Because libc `calloc` would
return an 8- or 16-byte-aligned pointer that does not carry the hidden
"original `malloc` address" header, so `vpx_free` would dereference
the wrong word and corrupt the heap. Every `vpx_*` allocator must
produce pointers whose stash slot is valid; routing through
`vpx_malloc` enforces that.

The function performs the overflow check **with the caller's actual
`num` and `size`**, *before* multiplying them, so a crafted
`num × size = 0` (i.e. either factor zero) or wrap is rejected at the
top. After that, the call to `vpx_malloc(num * size)` is safe — the
multiplication on this line cannot overflow because
`check_size_argument_overflow` just proved it stays under
`VPX_MAX_ALLOCABLE_MEMORY` and within `size_t`.

The `memset` is unconditional only when `x != NULL`; allocation
failure returns `NULL` straight through to the caller, again matching
libc `calloc`'s contract.

**Used by:** virtually every persistent structure in the codec. The
`MODE_INFO` grid (`cm->mip`, `cm->prev_mip`), the per-frame
probability tables, the `VP8D_COMP` instance, and the
`vpx_codec_alg_priv_t` handed back from `vpx_codec_dec_init` are all
allocated with `vpx_calloc` so they start in a defined state.

## `vpx_free` — symmetric release

```c
void vpx_free(void *memblk) {
  if (memblk) {
    void *addr = get_actual_malloc_address(memblk);
    free(addr);
  }
}
```

The `if (memblk)` guard makes `vpx_free(NULL)` a no-op, matching the
ISO C guarantee for `free(NULL)`. The codec relies on this in many
places: cleanup paths blindly call `vpx_free` on every pointer in a
struct after `memset(struct, 0)`, without first checking whether the
field was ever allocated.

Once a non-`NULL` pointer is in hand the rest is symmetric with
`vpx_memalign`: walk back one `size_t` to find the stashed original
`malloc` address, hand *that* address to libc `free`. The user's
pointer (`memblk`) is never freed directly — it points into the middle
of a `malloc` block, and freeing it would corrupt the heap.

**Invariant:** `memblk` must have been returned by one of the `vpx_*`
allocators, never by libc `malloc`/`calloc`/`realloc` directly. There
is no defence against violating this — the stash slot would contain
garbage and `free` would be called on a wild address.

## Note: no `vpx_realloc`

Despite the prompt's mention of `vpx_realloc`, **there is no
`vpx_realloc` in this file** (nor in `vpx_mem.h`). The codec does not
need one: every buffer that might grow (frame buffers, mode-info grid)
is reallocated by the explicit two-step
"`vpx_free` old, `vpx_calloc` new" pattern that you can see in
`vp8/common/alloccommon.c` and `vpx_scale/generic/yv12config.c`. A
generic realloc would have to know which alignment to preserve and
would still have to round-trip through the hidden header, so the
codebase simply does not provide one.

## Note: `vpx_memset16`

The header `vpx_mem.h` exposes a fifth symbol, `vpx_memset16`, but it
is declared `static INLINE` and is **only compiled when
`CONFIG_VP9_HIGHBITDEPTH` is set**. In a VP8-decoder build it is
absent. It is not defined in `vpx_mem.c`.

## Summary of the contract

| Function          | Returns                                       | Failure mode |
|-------------------|-----------------------------------------------|--------------|
| `vpx_memalign(a,s)` | pointer aligned to `a`, valid for `s` bytes | `NULL` on overflow or OOM |
| `vpx_malloc(s)`     | pointer aligned to `DEFAULT_ALIGNMENT`      | `NULL` on overflow or OOM |
| `vpx_calloc(n,s)`   | zero-filled, aligned to `DEFAULT_ALIGNMENT` | `NULL` on overflow or OOM |
| `vpx_free(p)`       | (void), no-op on `NULL`                     | undefined if `p` not from `vpx_*` |

Every pointer returned by the first three carries an invisible
`sizeof(size_t)`-byte prefix containing the original `malloc` address;
`vpx_free` is the only legal way to release them. Around that single
mechanism the file builds two safety properties — alignment up to any
power of two, and refusal to allocate beyond a per-target cap — that
together let the rest of libvpx treat memory as a single, opaque,
SIMD-friendly resource.
