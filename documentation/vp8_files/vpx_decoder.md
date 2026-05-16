# `vpx/src/vpx_decoder.c` — The decoder-side public API dispatcher

## Role in the decoder

libvpx is organised around a thin, codec-agnostic public surface (the
`vpx_codec_*` family) sitting on top of one or more codec-specific
implementations. For VP8 decoding, the per-codec implementation lives in
`vp8/vp8_dx_iface.c`, which exports a `vpx_codec_iface_t` describing that
algorithm. The file documented here, `vpx/src/vpx_decoder.c`, is the other
end of that bridge: it is the **dispatcher** that every application talks to
when it decodes a VP8 (or VP9) bitstream.

Each function in this file follows a single, deliberately monotonous
pattern:

1. validate the arguments handed in by the caller (a defensive courtesy,
   since the public API is the only place where untrusted pointers can
   enter the codec);
2. validate that the context is in a state where the requested operation
   is meaningful (initialised, with a populated `priv`);
3. validate that the algorithm bound to that context actually advertises
   the capability being requested;
4. route the call through the function pointer slot
   `ctx->iface->dec.<slot>(...)` (or, for the few entry points that touch
   `vpx_codec_priv` directly, manipulate that structure in place);
5. record the resulting `vpx_codec_err_t` into `ctx->err` via
   `SAVE_STATUS` so that the caller can later retrieve it with
   `vpx_codec_error()` / `vpx_codec_error_detail()` without re-passing the
   return value.

Why is the dispatcher so thin? Because the public ABI must be stable
across libvpx releases while the codec internals are free to churn. By
forcing every entry point through a vtable (`vpx_codec_iface_t`), libvpx
can add codecs, retire codecs, and reshape codec-private state without
breaking compiled applications. `vpx_decoder.c` is therefore mostly
*plumbing* — its real job is to make sure that, by the time control
reaches the VP8 decoder implementation, the inputs are sane and the
codec is the right kind of codec to do the work.

The entry points in this file map one-to-one onto the public decoder API
declared in `vpx/vpx_decoder.h`:

| Public entry point                          | Dispatches to                       |
|---------------------------------------------|-------------------------------------|
| `vpx_codec_dec_init_ver`                    | `iface->init`                       |
| `vpx_codec_peek_stream_info`                | `iface->dec.peek_si`                |
| `vpx_codec_get_stream_info`                 | `iface->dec.get_si`                 |
| `vpx_codec_decode`                          | `iface->dec.decode`                 |
| `vpx_codec_get_frame`                       | `iface->dec.get_frame`              |
| `vpx_codec_register_put_frame_cb`           | mutates `ctx->priv->dec.put_frame_cb` |
| `vpx_codec_register_put_slice_cb`           | mutates `ctx->priv->dec.put_slice_cb` |
| `vpx_codec_set_frame_buffer_functions`      | `iface->dec.set_fb_fn`              |

Destruction (`vpx_codec_destroy`) and error retrieval (`vpx_codec_error`,
`vpx_codec_error_detail`, `vpx_codec_error_to_string`) live in the sister
file `vpx/src/vpx_codec.c` — they are common to encoders and decoders.

## Preliminaries — the include and the two helpers

The file opens with the bare minimum needed to act as a router:

```c
#include <string.h>
#include "vpx/internal/vpx_codec_internal.h"
```

A single `string.h` is pulled in for `memset`, used once during context
initialisation. `vpx_codec_internal.h` is the contract this file enforces:
it defines `vpx_codec_iface_t` (the vtable), `vpx_codec_priv` (the
per-instance state the algorithm allocates and the dispatcher peeks into
for callback storage), the `vpx_codec_*_fn_t` typedefs that name the
function-pointer slots, and the magic constant
`VPX_CODEC_INTERNAL_ABI_VERSION`. Including the *internal* header is what
makes `vpx_decoder.c` part of libvpx rather than just another consumer
of the public API.

### `SAVE_STATUS` — write-through error reporting

```c
#define SAVE_STATUS(ctx, var) (ctx ? (ctx->err = var) : var)
```

**What.** A two-argument macro that copies `var` into `ctx->err` if `ctx`
is non-NULL, then evaluates to `var` either way.

**Why.** Every dispatcher function below ends with
`return SAVE_STATUS(ctx, res);`. The macro has two purposes that the bare
`return res;` would not serve:

  * It mirrors the error into `ctx->err` so that an application can call
    `vpx_codec_error(ctx)` *much later* — long after the original return
    value has been discarded — and still recover the most recent failure
    code. This is exactly what `vpx_codec_error_to_string` and
    `vpx_codec_error_detail` then consume.
  * It guards against `ctx == NULL`. If the caller handed in a null
    context (the very first thing the function detected as invalid), the
    macro must not dereference it; it just propagates `res` upward. The
    ternary is the entire reason this is a macro and not a function — a
    function call cannot conditionally elide an assignment to a member.

**Invariant.** After any dispatcher call that returns through
`SAVE_STATUS`, the state observable through `vpx_codec_error(ctx)` is
*exactly* the value just returned. Errors are never silently swallowed
or upgraded.

**Where used.** Every entry point in this file except
`vpx_codec_peek_stream_info` (which has no context yet — see below) and
`vpx_codec_get_frame` (which returns `vpx_image_t *`, not an error
code).

### `get_alg_priv` — the cast across the public/private boundary

```c
static vpx_codec_alg_priv_t *get_alg_priv(vpx_codec_ctx_t *ctx) {
  return (vpx_codec_alg_priv_t *)ctx->priv;
}
```

**What.** A one-line static helper that casts `ctx->priv` (declared as a
pointer to the public `vpx_codec_priv`) down to the opaque,
codec-private `vpx_codec_alg_priv_t`.

**Why.** The trick the comment in `vpx_codec_internal.h` describes is
that an algorithm's per-instance struct begins with a `vpx_codec_priv`
header. The dispatcher's view of that struct is the header; the
algorithm's view is the full struct. The cast is therefore safe by
contract: every codec promises to allocate `vpx_codec_alg_priv_t` such
that its first bytes are a `vpx_codec_priv`. By isolating the cast in
one helper, the dispatcher functions stay readable and the assumption
is documented in exactly one place.

## The lifecycle: initialisation

### `vpx_codec_dec_init_ver` — bind a context to an algorithm

The application begins by declaring an opaque `vpx_codec_ctx_t` on the
stack (or the heap) and calling this function — usually through the
convenience macro `vpx_codec_dec_init`, which fixes the trailing `ver`
argument to the current ABI version.

The body is essentially one large `if`/`else if` chain whose only purpose
is to refuse to proceed if any precondition is violated. Reading it from
top to bottom:

```c
if (ver != VPX_DECODER_ABI_VERSION)
  res = VPX_CODEC_ABI_MISMATCH;
else if (!ctx || !iface)
  res = VPX_CODEC_INVALID_PARAM;
else if (iface->abi_version != VPX_CODEC_INTERNAL_ABI_VERSION)
  res = VPX_CODEC_ABI_MISMATCH;
else if ((flags & VPX_CODEC_USE_POSTPROC) &&
         !(iface->caps & VPX_CODEC_CAP_POSTPROC))
  res = VPX_CODEC_INCAPABLE;
...
```

These checks deserve enumeration because they are the file's most
important defensive surface:

1. **Caller-side ABI check** (`ver != VPX_DECODER_ABI_VERSION`). The
   `ver` argument is baked into the application at compile time by the
   `vpx_codec_dec_init` macro, which expands to the
   `VPX_DECODER_ABI_VERSION` defined in the *header* the application
   compiled against. If the running `libvpx.so` defines a different
   value, the application was linked against an incompatible header and
   must be rebuilt. This check protects the application from layout
   skew in `vpx_codec_dec_cfg_t`, `vpx_codec_ctx_t`, and the public
   decoder ABI generally.

2. **Null-pointer check** (`!ctx || !iface`). Without a context to
   initialise and an algorithm to install in it, there is nothing to do.

3. **Implementation-side ABI check** (`iface->abi_version !=
   VPX_CODEC_INTERNAL_ABI_VERSION`). The algorithm's vtable was compiled
   against an *internal* ABI version. If that does not match the
   dispatcher's expectation, the vtable layout may have shifted and the
   function pointer slots cannot be trusted. This is a separate ABI
   from the public one: the public ABI insulates the application from
   the library, the internal ABI insulates the dispatcher from the
   codec implementation. Both must match.

4. **Capability negotiation** (the three `flags & VPX_CODEC_USE_*` blocks
   against `iface->caps & VPX_CODEC_CAP_*`). The application requests
   features at initialisation time via `flags`; the algorithm advertises
   which features it supports via `caps`. If the application asks for
   post-processing and the linked-in decoder lacks it (e.g. a build
   compiled with `--disable-postproc`), the request is refused with
   `VPX_CODEC_INCAPABLE` rather than silently ignored. The three checks
   here cover the three init-time-required capabilities: postproc, error
   concealment, and input fragments. (Frame threading is a VP9 concern
   and is checked elsewhere.)

5. **Decoder-vs-encoder** (`!(iface->caps & VPX_CODEC_CAP_DECODER)`).
   `vpx_codec_iface_t` is the same struct type used by both encoders and
   decoders, so a caller could in principle hand `vpx_codec_dec_init`
   an encoder's interface. This last guard catches that.

Only when every gate is passed does the function commit:

```c
memset(ctx, 0, sizeof(*ctx));
ctx->iface = iface;
ctx->name = iface->name;
ctx->priv = NULL;
ctx->init_flags = flags;
ctx->config.dec = cfg;

res = ctx->iface->init(ctx, NULL);
```

The `memset` zeros the entire public context, including the `err` field
that subsequent calls will populate. The four field assignments wire the
context to the algorithm and record the initialisation parameters. Then
the algorithm's `init` slot is invoked, with `data == NULL` (the second
argument is reserved for multi-resolution encoder initialisation and is
unused for decoders). It is the algorithm's `init` that actually allocates
`ctx->priv` — until it returns successfully, `ctx->priv` is NULL.

The failure cleanup is interesting:

```c
if (res) {
  ctx->err_detail = ctx->priv ? ctx->priv->err_detail : NULL;
  vpx_codec_destroy(ctx);
}
```

If `init` fails *after* it has already allocated `ctx->priv`, that
storage must be reclaimed; `vpx_codec_destroy` (defined in `vpx_codec.c`)
calls back through `iface->destroy` to do so. Before destroying, the
detail string is hoisted from the about-to-be-freed `priv` to the
context's own `err_detail` slot so the caller can still query a
meaningful description after the failure.

**Invariants on success.** `ctx->iface != NULL`, `ctx->priv != NULL`,
`ctx->name == iface->name`. These three are precisely what every
*subsequent* dispatcher function checks for to decide whether the context
has been initialised.

**Why this is the only entry point that touches `ctx->iface` and
`ctx->priv` directly.** Every other function in this file *reads*
those fields to validate state but never *writes* them. Binding a
context to a codec is exactly what initialisation means; once bound, the
binding is immutable until destruction.

## Stream introspection

### `vpx_codec_peek_stream_info` — parse without committing

```c
vpx_codec_err_t vpx_codec_peek_stream_info(vpx_codec_iface_t *iface,
                                           const uint8_t *data,
                                           unsigned int data_sz,
                                           vpx_codec_stream_info_t *si);
```

**What.** Examines a raw bitstream buffer and fills in
`vpx_codec_stream_info_t` (width, height, key-frame bit) *without
constructing a decoder instance*.

**Why.** Containers like WebM hand the application a single packet at a
time; before the application even decides whether to instantiate a
decoder, it may want to look at the first packet to confirm the format
or to learn the frame dimensions. By taking `iface` instead of `ctx`,
this entry point lets the caller use the algorithm's parser without
paying for codec instance allocation.

**Argument validation.** Four pointers (`iface`, `data`, `si`) must be
non-NULL, `data_sz` must be non-zero, and `si->sz` must be at least the
size of the public struct. The last check is the standard libvpx
convention for forward-compatible structs: callers initialise `sz` to
`sizeof(*si)`, and the library refuses to write into a buffer that's
smaller than what it was built to populate.

**Notable absence of `SAVE_STATUS`.** This function takes no `ctx`, so
there is nowhere to stash the error. The return value is the only
channel.

**Dispatch.** `si->w` and `si->h` are pre-cleared so the algorithm can
leave them at 0 if it cannot determine them, then `iface->dec.peek_si`
is invoked. For VP8, this lands in `vp8_peek_si` in `vp8_dx_iface.c`,
which decodes the three-byte uncompressed VP8 header.

### `vpx_codec_get_stream_info` — query an active context

```c
vpx_codec_err_t vpx_codec_get_stream_info(vpx_codec_ctx_t *ctx,
                                          vpx_codec_stream_info_t *si);
```

**What.** The same idea as `peek_stream_info`, but for a context that
has already decoded (or at least started decoding) a stream — it
returns information about *the stream that has been parsed*, not about
an arbitrary buffer.

**Why split this from `peek`?** Because once a frame has been decoded,
the codec instance already knows the answers and has them cached in
its private state. Asking the algorithm to re-parse the bitstream would
be wasteful and would also require the application to retain the
original buffer. The `get_si` slot reads from `vpx_codec_alg_priv_t`
instead.

**Validation differences.** This entry point checks the standard
"context is initialised" invariant (`ctx->iface && ctx->priv`), and
returns `VPX_CODEC_ERROR` (not `INVALID_PARAM`) if the context exists
but was never successfully `dec_init`ed.

**Dispatch.** `iface->dec.get_si(get_alg_priv(ctx), si)` — the helper
cast is used here because the algorithm's slot operates on the private
struct.

## The hot path: decoding and frame retrieval

### `vpx_codec_decode` — feed encoded bytes in

```c
vpx_codec_err_t vpx_codec_decode(vpx_codec_ctx_t *ctx, const uint8_t *data,
                                 unsigned int data_sz, void *user_priv,
                                 long deadline);
```

**What.** Submits one compressed VP8 frame (or one fragment thereof, if
`VPX_CODEC_USE_INPUT_FRAGMENTS` was set at init) to the codec.

**Why the parameters.** `user_priv` is an opaque cookie the application
attaches to this packet; when a decoded image becomes available through
`vpx_codec_get_frame`, the codec will carry that cookie along on
`vpx_image_t::user_priv` so the application can pair the picture with
the application-side timing/metadata it owns. `deadline` was an
intent-of-quality control inherited from on2's earlier codecs; the
header explicitly documents that VP8 ignores it ("always pass 0"), and
the function body cements this with a `(void)deadline;` to silence the
unused-parameter warning.

**The peculiar pointer/size test.**

```c
if (!ctx || (!data && data_sz) || (data && !data_sz))
  res = VPX_CODEC_INVALID_PARAM;
```

`data == NULL && data_sz == 0` is allowed *on purpose*: it is how a
client signals end-of-stream when fragments are enabled, which causes
the algorithm to invoke `put_frame` for any frame whose final fragment
has been delivered. The disallowed combinations are the mismatches:
non-NULL pointer but zero size, or NULL pointer with non-zero size —
both of which would indicate a caller bug.

**Dispatch.** Standard "initialised?" check, then
`ctx->iface->dec.decode(get_alg_priv(ctx), data, data_sz, user_priv)`.
For VP8 this delegates to the per-frame decoder driven from
`vp8/decoder/decodeframe.c`.

**Invariant.** On success, the list of "frames ready for display" inside
the algorithm has been updated; the application must drain it with
`vpx_codec_get_frame` before the next `vpx_codec_decode` call. The
public header makes this explicit ("the list of available frames …
remains valid until the next call to `vpx_codec_decode`").

### `vpx_codec_get_frame` — drain decoded pictures

```c
vpx_image_t *vpx_codec_get_frame(vpx_codec_ctx_t *ctx, vpx_codec_iter_t *iter);
```

**What.** An iterator over the images that became displayable as a
result of the most recent `vpx_codec_decode`. The caller initialises an
opaque `vpx_codec_iter_t` to NULL and calls this function repeatedly
until it returns NULL.

**Why an iterator and not a single return?** VP9 super-frames may
contain multiple displayable frames; the iterator interface is the
common shape used by both codecs. For VP8 the inner loop almost always
runs at most once, but the API does not assume that.

**Why no `SAVE_STATUS`?** Because the return type is `vpx_image_t *`,
not `vpx_codec_err_t`. The only failure mode is "not ready" or "context
invalid", and the API expresses both by returning NULL. There is no
error code to record.

**Validation.** Defends against null `ctx`/`iter` and against a context
that hasn't been initialised; otherwise routes straight to
`ctx->iface->dec.get_frame(get_alg_priv(ctx), iter)`.

## Callback registration — direct manipulation of `vpx_codec_priv`

The next three functions are the unusual ones: instead of dispatching
through `iface->dec.<slot>`, two of them write into `ctx->priv` in place,
and the third dispatches but only after consulting capability bits.

This split is principled. The `put_frame_cb` and `put_slice_cb` storage
lives in the *generic* `vpx_codec_priv` struct (defined in
`vpx_codec_internal.h` and shared by every codec), so the dispatcher can
manipulate it without going through the algorithm — the algorithm will
later read the same slots when it has an image to publish. External
frame buffer registration, on the other hand, is implementation-specific
state, so the algorithm must be involved.

### `vpx_codec_register_put_frame_cb`

```c
vpx_codec_err_t vpx_codec_register_put_frame_cb(vpx_codec_ctx_t *ctx,
                                                vpx_codec_put_frame_cb_fn_t cb,
                                                void *user_priv);
```

**What.** Records `cb` and `user_priv` in
`ctx->priv->dec.put_frame_cb`. When the algorithm later finishes a
frame and chooses the push model, it will invoke
`put_frame_cb.u.put_frame(put_frame_cb.user_priv, img)`.

**Why a capability check.** Not every codec build invokes
`put_frame_cb`. The dispatcher refuses registration with
`VPX_CODEC_INCAPABLE` unless `iface->caps & VPX_CODEC_CAP_PUT_FRAME` —
without that, the callback would be installed but never fired, leaving
the application waiting forever.

**Invariants.** After success, the union member `u.put_frame` (not
`u.put_slice`) holds the function pointer. The two callback kinds share
the same storage via a union (see `vpx_codec_priv_cb_pair_t` in
`vpx_codec_internal.h`); the codec knows which to call based on which
capability it advertises.

**Notable on VP8.** The VP8 decoder does *not* advertise
`VPX_CODEC_CAP_PUT_FRAME` (see `vp8_dx_iface.c`), so this entry point
always returns `VPX_CODEC_INCAPABLE` for VP8 contexts. VP8 applications
use the pull model via `vpx_codec_get_frame` instead. The dispatcher
exists for the *interface*, not because every codec needs it.

### `vpx_codec_register_put_slice_cb`

The exact same structure as `register_put_frame_cb`, but for slice-level
callbacks and gated on `VPX_CODEC_CAP_PUT_SLICE`. Slice callbacks fire
during decoding to notify the application about partially decoded image
regions, with `vpx_image_rect_t` parameters describing the valid and
updated rectangles. Again, VP8 in libvpx does not advertise this, so the
registration is informational for VP9 and future codecs.

The two functions are deliberately near-identical: pasting them is
preferable to factoring out the four-line body, because the union field
selection (`u.put_frame` vs `u.put_slice`) and the capability bit are
both type-specific, and templating around either would require either
macros or function pointer indirection that would harm clarity.

### `vpx_codec_set_frame_buffer_functions` — external frame-buffer registration

```c
vpx_codec_err_t vpx_codec_set_frame_buffer_functions(
    vpx_codec_ctx_t *ctx, vpx_get_frame_buffer_cb_fn_t cb_get,
    vpx_release_frame_buffer_cb_fn_t cb_release, void *cb_priv);
```

**What.** Hands the algorithm a pair of callbacks: `cb_get` is invoked
whenever the codec needs a new frame buffer to write a decoded picture
into; `cb_release` is invoked when libvpx no longer references that
buffer. `cb_priv` is the application cookie passed to both.

**Why externalise buffer allocation?** Players that integrate with GPU
texture pools, zero-copy pipelines, or pre-allocated arenas often need
to dictate where decoded frames land. Without this hook, libvpx would
own the YV12 buffer and the application would have to `memcpy` it out;
with it, the application supplies the storage directly.

**Why gated on a capability.** Only the VP9 implementation supports
this in libvpx — the documentation on the header function explicitly
notes "Currently this only works with VP9." The VP8 decoder does not
advertise `VPX_CODEC_CAP_EXTERNAL_FRAME_BUFFER`, so VP8 callers will
receive `VPX_CODEC_INCAPABLE`. The reason this entry point dispatches
through `iface->dec.set_fb_fn` instead of writing into `ctx->priv` (as
the put-frame/put-slice callbacks do) is that the registered callbacks
have to interpose on the codec's *internal* buffer-pool allocator,
which lives in `vpx_codec_alg_priv_t` — out of reach of the generic
dispatcher.

**Invariant noted in the header.** The application "must" call this
before the first `vpx_codec_decode`. The dispatcher does not enforce
the ordering — it cannot tell what state the algorithm is in — but the
documented contract is that mid-stream changes to the buffer-allocator
policy are not supported.

## Threading and concurrency notes

`vpx_decoder.c` itself contains no locks. The dispatcher is reentrant:
two threads can simultaneously call `vpx_codec_decode` on two distinct
`vpx_codec_ctx_t` objects with no interaction. Concurrency on a *single*
context, however, is the algorithm's responsibility — and for libvpx
built with `--disable-multithread`, the header on `vpx_codec_dec_init`
explicitly warns that even `vpx_codec_dec_init_ver` itself must be
guarded by an external lock if called from multiple threads, because
parts of the algorithm `init` path may touch process-global RTCD state.

## Postproc and frame-buffer-callback registration paths — summary

The two configuration paths that the task brief calls out specifically
both flow through `vpx_codec_dec_init_ver` but split immediately:

* **Postproc** is set up entirely at init time. The application passes
  `VPX_CODEC_USE_POSTPROC` in `flags`; the dispatcher rejects the
  request with `VPX_CODEC_INCAPABLE` if `iface->caps` does not include
  `VPX_CODEC_CAP_POSTPROC` (i.e. the build was made with
  `--disable-postproc`); otherwise the flag is stored in
  `ctx->init_flags` for the algorithm to read during its `init`. The
  postproc *configuration* (deblock level, noise level, etc.) is
  applied later via `vpx_codec_control`, dispatched out of
  `vpx_codec.c`, not from this file.

* **External frame buffer callbacks** are *not* set up at init time.
  Initialisation does not touch them at all; the application invokes
  `vpx_codec_set_frame_buffer_functions` after `vpx_codec_dec_init`
  returns and before the first `vpx_codec_decode`. The dispatcher
  performs argument and capability checks here and then delegates to
  the algorithm's `set_fb_fn` slot, which is the only place the actual
  hookup happens. The `put_frame_cb` and `put_slice_cb` registrations
  are different again: they are stored in `ctx->priv` by the dispatcher
  itself, bypassing the algorithm entirely.

Together these three paths illustrate the design rule of the file:
every public knob is exposed via one of three mechanisms — init flags
(checked here, consumed by the algorithm), direct storage in
`vpx_codec_priv` (written by this dispatcher, read by the algorithm),
or dispatch through a function pointer in `iface->dec` (where the
algorithm holds the state and the dispatcher is purely a validator).
This three-way split is what keeps `vpx_decoder.c` to fewer than 200
lines while still providing the entire decoder-side public API.
