# vpx_encoder.c — the public encoder dispatcher

`vpx/src/vpx_encoder.c` is the thin C wrapper that sits between an
application's `vpx_codec_enc_*` calls and the concrete encoder
implementation buried inside a `vpx_codec_iface_t` (for VP8 that would
be `vpx_codec_vp8_cx_algo` in `vp8/vp8_cx_iface.c`; for VP9,
`vpx_codec_vp9_cx_algo`). Its job is bookkeeping — version handshakes,
NULL-pointer checks, dispatching through a function-pointer table, and
the small amount of generic glue (multi-resolution iteration, floating
point precision normalization, output-buffer relocation) that doesn't
belong inside any single codec.

## Role in the decoder

A decoder-only build of libvpx still links this file. Section A of
`vp8_files.md` lists `vpx/src/vpx_encoder.c` among the 47 mandatory
object files in a `--disable-vp8-encoder` build, with the qualifier
"built unconditionally; no-op without an encoder". That phrasing is
exact: the translation unit always compiles and always links, but every
one of its functions checks `iface->caps & VPX_CODEC_CAP_ENCODER` and
returns `VPX_CODEC_INCAPABLE` when the bit is clear. So the symbols are
present in `libvpx.a` and the application can call them, but they do
nothing useful on a decoder-only build.

Why ship it at all? Two reasons, both structural rather than
functional.

First, **transitive header inclusion**. `vpx/internal/vpx_codec_internal.h`
is the central interface that every codec (decoder included) must
implement, and at lines 47–48 it pulls in both `../vpx_decoder.h` and
`../vpx_encoder.h` unconditionally. That means the `vpx_codec_iface_t`
struct itself (defined in `vpx_codec_internal.h`) embeds a
`struct vpx_codec_enc_iface enc` member whose function-pointer types
(`vpx_codec_encode_fn_t`, `vpx_codec_get_cx_data_fn_t`, etc.) are
declared in `vpx_encoder.h`. A pure decoder build still needs those
type declarations just to lay out the interface struct correctly —
which is precisely why the doc note in `vp8_files.md` flags
`vpx_encoder.h`, `vpx_ext_ratectrl.h`, and `vpx_tpl.h` as
non-deletable in a decoder fork.

Second, **API symmetry**. Applications link against a single `libvpx`
and discover at runtime whether the codec they instantiated has
encoder support, by calling `vpx_codec_enc_init_ver()` and checking the
return code. Removing the `vpx_codec_enc_*` symbols would force
applications to use conditional compilation, which the libvpx ABI has
historically avoided.

So this file is, for a decoder, a façade returning polite refusals.
The narrative below explains every definition assuming the encoder
*is* present (the VP8 encoder case), and notes wherever the
decoder-only path diverges.

## What this file deals with

Everything here operates on three handles:

- `vpx_codec_ctx_t *ctx` — the application-visible context. Holds an
  `iface` pointer (the algorithm's vtable) and a `priv` pointer (the
  algorithm's per-instance state).
- `vpx_codec_iface_t *iface` — the vtable. Its `enc` sub-struct
  carries the seven function pointers (`encode`, `get_cx_data`,
  `cfg_set`, `cfg_maps`, `get_glob_hdrs`, `get_preview`,
  `mr_get_mem_loc`/`mr_free_mem_loc`) that this file dispatches into.
- `vpx_codec_alg_priv_t *priv` — opaque to this file; recovered via
  the static helper `get_alg_priv` for passing down to algorithm code.

### `SAVE_STATUS` — record the error on the context, return it too

```c
#define SAVE_STATUS(ctx, var) ((ctx) ? ((ctx)->err = (var)) : (var))
```

A one-line ternary that does two things at once. If `ctx` is non-NULL,
it writes the result into `ctx->err` so that a later
`vpx_codec_error(ctx)` or `vpx_codec_error_detail(ctx)` call can recover
a textual description. Either way, the macro evaluates to the error
code so it can be `return`ed from the calling function.

The NULL guard matters because some failure modes (most obviously, the
caller passing `ctx == NULL` to `vpx_codec_enc_init_ver`) cannot write
into a context that doesn't exist; the macro silently degrades to "just
return the code". This is the file's only macro of substance, and it
appears at the bottom of nearly every public function.

### `get_alg_priv` — cast `ctx->priv` to the algorithm-private type

```c
static vpx_codec_alg_priv_t *get_alg_priv(vpx_codec_ctx_t *ctx) {
  return (vpx_codec_alg_priv_t *)ctx->priv;
}
```

`vpx_codec_priv_t` (in `vpx_codec_internal.h`) is the *generic* part
of the per-instance state; the algorithm's actual private struct
extends it. The convention is that the algorithm allocates its own
struct whose first member is `vpx_codec_priv_t`, and then casts
freely. This helper exists purely to centralize the cast and keep the
dispatch sites readable. It is not exported.

### `vpx_codec_enc_init_ver` — set up an encoder instance

This is the entrypoint the `vpx_codec_enc_init` convenience macro
expands to (the macro appends `VPX_ENCODER_ABI_VERSION` automatically).
The "what" is described in `vpx_encoder.h`; the "why" of the checks is
what's interesting here.

The function performs five validation steps in a strict order, encoded
as an `if / else if / else if / .../ else` chain so that the *first*
failing predicate wins and no further checks run:

```c
if (ver != VPX_ENCODER_ABI_VERSION)
  res = VPX_CODEC_ABI_MISMATCH;
else if (!ctx || !iface || !cfg)
  res = VPX_CODEC_INVALID_PARAM;
else if (iface->abi_version != VPX_CODEC_INTERNAL_ABI_VERSION)
  res = VPX_CODEC_ABI_MISMATCH;
else if (!(iface->caps & VPX_CODEC_CAP_ENCODER))
  res = VPX_CODEC_INCAPABLE;
...
```

Two distinct ABI versions are checked. `VPX_ENCODER_ABI_VERSION` (in
`vpx_encoder.h`) is what an application sees — it bundles
`VPX_CODEC_ABI_VERSION` and `VPX_EXT_RATECTRL_ABI_VERSION` so a single
integer captures every external structure layout the application
depends on. `VPX_CODEC_INTERNAL_ABI_VERSION` (currently `5`, in
`vpx_codec_internal.h`) is the *interface-implementer* ABI — bumped
whenever the layout of the `vpx_codec_iface_t` vtable itself changes.
The two are independent: a new field added to `vpx_codec_enc_cfg_t`
breaks the former but not the latter.

The `VPX_CODEC_CAP_ENCODER` check is the gate that makes a decoder-only
build well-behaved. A VP8 decoder interface (`vpx_codec_vp8_dx_algo`)
will have `VPX_CODEC_CAP_DECODER` set but not `VPX_CODEC_CAP_ENCODER`;
calling `vpx_codec_enc_init_ver` against it returns `VPX_CODEC_INCAPABLE`
cleanly without dereferencing the encoder vtable.

After validation, the function populates `ctx`, then delegates to
`iface->init(ctx, NULL)` — the same function pointer used by
`vpx_codec_dec_init`, distinguished only by the second argument (NULL
here means single-encoder, no multi-res). The `NULL` is the
`vpx_codec_priv_enc_mr_cfg_t *` slot, occupied below by
`vpx_codec_enc_init_multi_ver`.

The failure path deserves attention. There is a careful comment:

```c
// IMPORTANT: ctx->priv->err_detail must be null or point to a string
// that remains valid after ctx->priv is destroyed, such as a C string
// literal. This makes it safe to call vpx_codec_error_detail() after
// vpx_codec_enc_init_ver() failed.
```

The invariant is real: `vpx_codec_destroy` frees `ctx->priv`, so the
detail string captured into `ctx->err_detail` before destruction must
have *static* lifetime. Algorithms that use `vpx_internal_error` with
`vsnprintf` must take care to copy or replace such strings with literals
before the destroy. The wrapper's job is just to hoist the pointer up
one level so the user can still call `vpx_codec_error_detail()` on the
context after a failed init.

### `vpx_codec_enc_init_multi_ver` — multi-resolution variant

Same shape as the single-encoder init, but iterates over an *array* of
`num_enc` contexts, configs, and downsampling factors. This is the
mechanism behind libvpx's spatial multi-resolution encoding: the
application allocates `num_enc` contexts side-by-side, each one
representing a different scale, and they share a single shared
`mem_loc` buffer holding cross-resolution mode info.

The header documents that this is *only supported by VP8* — for VP9 the
proper SVC API is used instead. Inside the function this manifests as a
hard requirement that `iface->enc.mr_get_mem_loc` be non-NULL:

```c
if (iface->enc.mr_get_mem_loc == NULL) return VPX_CODEC_INCAPABLE;
```

The shared buffer pattern is the load-bearing complexity. Ownership
flows like this:

1. `mr_get_mem_loc(cfg, &mem_loc)` allocates the shared memory.
2. Each per-encoder `init` is called with a `vpx_codec_priv_enc_mr_cfg_t`
   carrying `mem_loc` and the encoder's id within the group (numbered
   in reverse, `num_enc - 1 - i`, so encoder 0 is highest resolution).
3. The comment captures the ownership rule:

   ```c
   // ctx takes ownership of mr_cfg.mr_low_res_mode_info if and only if
   // this call succeeds. The first ctx entry in the array is
   // responsible for freeing the memory.
   ```

   So on success, the first context (the one populated first, at
   `i == 0`) owns the shared buffer and will free it during its
   `vpx_codec_destroy`. The `mem_loc_owned` flag, gated by
   `CONFIG_MULTI_RES_ENCODING`, tracks whether any context has taken
   ownership yet — if `init` fails before the first context completes,
   the wrapper itself calls `iface->enc.mr_free_mem_loc(mem_loc)` to
   prevent a leak.

4. On failure mid-stream, the wrapper walks backward through the
   already-initialized contexts calling `vpx_codec_destroy` on each,
   carrying the same `error_detail` pointer forward so every
   destroyed context surfaces the originating failure.

The `num_enc > 16 || num_enc < 1` guard is a hard upper bound — there
is no internal limit baked into the data structures, but the API caps
it as a sanity measure (compare with `VPX_MAX_LAYERS == 12`).

The downsampling factor range check `dsf->num < 1 || dsf->num > 4096`
keeps the numerator within a window that won't overflow when multiplied
by typical pixel dimensions, and `dsf->den > dsf->num` is forbidden
because the multi-res scheme only downsamples (denominator can't exceed
numerator). The check is inside the loop because each encoder has its
own dsf entry.

### `vpx_codec_enc_config_default` vs `vpx_codec_enc_config_set`

These two are an asymmetric pair worth contrasting.

`vpx_codec_enc_config_default` produces a fresh, prefilled
`vpx_codec_enc_cfg_t` by copying from the algorithm's static
`cfg_maps`:

```c
assert(iface->enc.cfg_map_count == 1);
*cfg = iface->enc.cfg_maps->cfg;
res = VPX_CODEC_OK;
```

The `assert` says: present-day libvpx only ever ships one cfg map per
encoder, even though the data structure (`vpx_codec_enc_cfg_map_t`,
in `vpx_codec_internal.h`) is an array indexed by "usage". The
deprecated `usage` parameter is required to be zero by the public
header — these two restrictions together mean the function is really
"give me the only available default". The `cfg_maps[0].cfg` is a
static struct compiled into the algorithm's translation unit (e.g.
`vp8_usage_cfg_map` in `vp8_cx_iface.c`), so the copy is to caller
memory and the caller is free to mutate it. This function is callable
*without* a context — it just inspects the iface.

`vpx_codec_enc_config_set` is the inverse: it pushes a (possibly
mutated) config into an already-initialized context, mid-stream:

```c
res = ctx->iface->enc.cfg_set(get_alg_priv(ctx), cfg);
```

This requires `ctx->priv` (so the context must have been init'd) and
delegates entirely to the codec's `cfg_set` callback, which is
responsible for validating that the new config is compatible with the
current state. Used for things like changing target bitrate
mid-stream.

Both functions share the `VPX_CODEC_CAP_ENCODER` gate; neither
performs ABI version checks (the assumption being that the
context already passed those checks at init time, and the
default-getter doesn't have a context to bind to anyway).

### `FLOATING_POINT_INIT` / `FLOATING_POINT_RESTORE` — x87 precision pin

```c
#if VPX_ARCH_X86 || VPX_ARCH_X86_64
#include "vpx_ports/x86.h"
#define FLOATING_POINT_INIT() \
  do {                        \
  unsigned short x87_orig_mode = x87_set_double_precision()
#define FLOATING_POINT_RESTORE()       \
  x87_set_control_word(x87_orig_mode); \
  }                                    \
  while (0)
#else
static void FLOATING_POINT_INIT(void) {}
static void FLOATING_POINT_RESTORE(void) {}
#endif
```

The deliberately-unbalanced macros open a `do { ... }` block at INIT
and close it at RESTORE. They must therefore appear paired in the
*same* lexical scope (and they do, bracketing the encode dispatch).
The trick declares `x87_orig_mode` as a local in the do-block so it
naturally lives just long enough to be passed back to
`x87_set_control_word`.

The "why" is in the comment at the top of the block: on x87, FPU
operations internally use 80-bit precision even when the operands are
64-bit `double`s. The SSE unit, by contrast, uses 64-bit precision
strictly. Code paths that mix x87 and SSE floats can produce slightly
different results depending on register allocation choices. The
encoder's rate-distortion math is sensitive to this — a one-bit
quantizer flip can move the bitstream — so the wrapper forces the x87
unit to 64-bit precision for the duration of the encode call, then
restores the caller's control word on exit. On every non-x86 platform
the macros expand to no-op static functions.

### `vpx_codec_encode` — submit one frame

This is the workhorse. After validation (the standard NULL,
caps-bit, and 32-bit-overflow checks), it bracket-calls
`FLOATING_POINT_INIT`/`RESTORE` around either a single
`iface->enc.encode` call or a reverse-order loop over the
multi-resolution context array.

Two non-obvious checks deserve mention.

```c
if (!ctx || (img && !duration))
  res = VPX_CODEC_INVALID_PARAM;
```

`img && !duration` means: if you're submitting a real frame
(non-NULL `img`), you must give it a positive duration. A NULL `img`
with `duration == 0` is the flush sentinel and is allowed.

```c
#if ULONG_MAX > UINT32_MAX
else if (duration > UINT32_MAX || deadline > UINT32_MAX)
  res = VPX_CODEC_INVALID_PARAM;
#endif
```

`duration` and `deadline` are typed `unsigned long`, which is 32 bits on
LLP64 (Windows) but 64 on LP64 (Linux/macOS). The codec internals
truncate to 32 bits, so on platforms where the ABI permits a wider
value, the wrapper actively rejects it rather than silently
truncating. The matching note appears in the public header docs for
`vpx_codec_set_cx_data_buf`.

The multi-res loop walks contexts in reverse — highest-numbered (which,
by the convention set up in `vpx_codec_enc_init_multi_ver`, is the
*lowest* resolution) first, descending to ctx[0] (highest resolution).
The rationale is that the low-resolution encoder pass produces the
mode info that the higher-resolution passes consume; the dependency
graph forces the order. The pointer arithmetic `ctx += num_enc - 1;
if (img) img += num_enc - 1;` assumes the caller has supplied
parallel arrays of the same length, which is the contract documented
in `vpx_codec_enc_init_multi_ver`.

After the loop the wrapper carefully resets `ctx` back to `ctx[0]` so
that `SAVE_STATUS(ctx, res)` records the error on the application's
base pointer, not the iterated tail.

### `vpx_codec_get_cx_data` — pull output packets out

The iterator pattern is canonical libvpx. The caller initializes an
opaque `vpx_codec_iter_t iter = NULL;` and calls in a loop until the
function returns NULL:

```c
const vpx_codec_cx_pkt_t *pkt;
vpx_codec_iter_t iter = NULL;
while ((pkt = vpx_codec_get_cx_data(ctx, &iter))) {
    /* dispatch on pkt->kind */
}
```

The iterator state is owned and interpreted by the algorithm —
`vpx_codec_pkt_list_get` (see below) demonstrates the typical
implementation, treating `*iter` as a `vpx_codec_cx_pkt_t *` walked
forward through a pre-populated array.

What this wrapper adds on top of the simple iface delegation is the
*output-buffer relocation* logic in the second half. If the
application has called `vpx_codec_set_cx_data_buf` to nominate a
destination buffer, and the algorithm emits a packet whose data lives
elsewhere, the wrapper will copy it into the nominated buffer
(respecting `pad_before` and `pad_after`) provided the data fits:

```c
if (dst_buf && pkt->data.raw.buf != dst_buf &&
    pkt->data.raw.sz + priv->enc.cx_data_pad_before +
            priv->enc.cx_data_pad_after <=
        priv->enc.cx_data_dst_buf.sz) {
  ...
  *modified_pkt = *pkt;
  modified_pkt->data.raw.buf = dst_buf;
  modified_pkt->data.raw.sz +=
      priv->enc.cx_data_pad_before + priv->enc.cx_data_pad_after;
  pkt = modified_pkt;
}
```

The packet returned to the caller is then `&priv->enc.cx_data_pkt`,
which is a per-context scratch slot (`vpx_codec_cx_pkt_t cx_data_pkt`
in `vpx_codec_priv`). The caller sees the new pointer and size; the
codec's internal buffer is unchanged. This explains the API note in
`vpx_encoder.h` that the returned buffer is only valid until the next
`vpx_codec_*` call — `cx_data_pkt` is reused for each packet.

When relocation succeeds, the wrapper bumps the destination cursor:

```c
if (dst_buf == pkt->data.raw.buf) {
  priv->enc.cx_data_dst_buf.buf = dst_buf + pkt->data.raw.sz;
  priv->enc.cx_data_dst_buf.sz -= pkt->data.raw.sz;
}
```

So successive `get_cx_data` calls within a single encode round-trip
append into the same application-supplied buffer until it fills, at
which point relocation silently stops and packets revert to pointing
at internal storage. The application must call
`vpx_codec_set_cx_data_buf` again to reset.

The `if (dst_buf == pkt->data.raw.buf)` predicate handles both cases —
the just-relocated case (where the modified packet's buf equals
`dst_buf` by construction) and the case where the codec happened to
write directly into `dst_buf` itself.

### `vpx_codec_set_cx_data_buf` — register output destination

A pure setter on the `vpx_codec_priv`'s `enc` sub-struct. It either
records the buffer/padding values or, if `buf == NULL`, clears them
back to zero. There is one invariant flagged in the public header:

> Applications MUSTNOT call this function during iteration of
> vpx_codec_get_cx_data().

The wrapper does not enforce this — it would require keeping
iteration state outside the iterator. The consequence of violating
the rule is that mid-iteration packets may end up split across the
old and new buffers in confusing ways.

### `vpx_codec_get_preview_frame` and `vpx_codec_get_global_headers`

Two near-identical thin dispatchers. Both are conditional capabilities:
the iface may legitimately leave `get_preview` or `get_glob_hdrs` as
NULL function pointers, in which case the wrapper returns
`VPX_CODEC_INCAPABLE` via `ctx->err`. (Notice these two return data
pointers, not error codes, so the only way to communicate failure is
to set `ctx->err` and return NULL.)

For VP8 specifically, the public docs note that
`vpx_codec_get_global_headers` is "Unsupported" — the VP8 iface leaves
the function pointer NULL and the dispatcher's NULL-check is the path
that converts that into the API-visible "incapable" response.

### `vpx_codec_pkt_list_add` and `vpx_codec_pkt_list_get`

These are utility helpers exported for algorithm implementers, not
exposed in the public `vpx_encoder.h`. The header that declares them
is `vpx_codec_internal.h`, which also provides the convenience macros
`vpx_codec_pkt_list_decl(n)` and `vpx_codec_pkt_list_init(m)` for
declaring a fixed-size packet list inline in an algorithm's private
struct.

`pkt_list_add` is bounded-append: returns 0 on success, 1 on overflow
(when `cnt == max`). The struct uses the classic C99 "struct hack"
single-element trailing array (`vpx_codec_cx_pkt pkts[1]` in the
declaration, padded out by the union-based `_decl` macro), so the
list has a fixed compile-time capacity per encoder.

`pkt_list_get` implements the iterator pattern from the algorithm side.
On the first call (`*iter == NULL`) it seeds the iterator with the
list head; on each subsequent call it advances by one and returns
NULL once `(pkt - list->pkts) >= list->cnt`. The cast through
`vpx_codec_iter_t` (a `const void *`) is the abstraction that lets
each algorithm use a different iteration scheme without the public
API needing to know.

## Summary

The whole file is around 390 lines of glue. Every function follows the
same pattern: validate cheaply, gate on capabilities, dispatch through
an iface function pointer, record status via `SAVE_STATUS`. The
non-trivial pieces are concentrated in three places — the ownership
dance of multi-resolution init, the floating-point precision pin
around `vpx_codec_encode`, and the optional output-buffer relocation in
`vpx_codec_get_cx_data`. Everything else is a controlled refusal,
which is exactly what makes the file safe to link into a decoder-only
build of libvpx without doing harm.
