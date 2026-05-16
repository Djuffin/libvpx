# `vp8/vp8_dx_iface.c` — the VP8 decoder's `vpx_codec_iface_t` adapter

## Role in the decoder

This single source file is the seam between the public, codec-agnostic
libvpx API (`vpx/vpx_codec.h`, `vpx/vpx_decoder.h`, dispatched by
`vpx/src/vpx_codec.c` and `vpx/src/vpx_decoder.c`) and the VP8-specific
decoder core in `vp8/decoder/onyxd_if.c`. The big-picture pipeline
diagram in
[`vp8_technical_overview.md` §1](../vp8_technical_overview.md#1-big-picture-pipeline)
shows where it sits: every call the application makes goes through
`vpx_codec_*()` dispatchers, those dispatchers chase function pointers
stored in a `vpx_codec_iface_t`, and **the VP8 decoder's
`vpx_codec_iface_t` instance is defined here**, at the bottom of this
file (`vp8/vp8_dx_iface.c:738-765`).

The file therefore plays three roles at once:

1. **It implements the vtable**. Every slot in the
   `vpx_codec_iface_t` struct (see
   `vpx/internal/vpx_codec_internal.h:293-326`) — init, destroy,
   ctrl_maps, peek_si, get_si, decode, get_frame — is filled in here
   with a `vp8_*` function defined in this file.
2. **It owns the decoder's per-instance "front-end" state**
   (`vpx_codec_alg_priv_t`, `vp8/vp8_dx_iface.c:44-64`). This is the
   block of memory pointed to by `ctx->priv` in every
   `vpx_codec_ctx_t` opened for VP8 decoding; it nests the public
   `vpx_codec_priv_t` as its first member, with VP8-specific bookkeeping
   tacked on afterwards.
3. **It translates between the public surface and the core.** The public
   API speaks `vpx_image_t`, `vpx_ref_frame_t`, varargs control codes,
   and per-decoder configuration. The core (`VP8D_COMP`,
   `YV12_BUFFER_CONFIG`, `vp8_ppflags_t`, …) speaks raw frame buffers
   and decoder structures. This file is the only place in the VP8
   decoder build where those two vocabularies meet.

Lifecycle as seen from inside this file:

- `vp8_init`  — first call after `vpx_codec_dec_init_ver()`; allocates
  the `vpx_codec_alg_priv_t`, kicks RTCD tables, but **does not yet
  allocate `VP8D_COMP` or any frame buffers** — that is deferred to the
  first decode call because VP8 carries width/height in the keyframe
  itself.
- `vp8_decode` — does the deferred allocation on the first keyframe,
  reallocates on resolution change, then delegates the actual bitstream
  parse + reconstruction to `vp8dx_receive_compressed_data` in
  `onyxd_if.c`.
- `vp8_get_frame` — wraps the just-decoded `frame_to_show` in a
  `vpx_image_t` and hands it to the application.
- `vp8_destroy` — calls `vp8_remove_decoder_instances` and frees the
  priv block.

The detailed lifecycle diagram lives in
[`vp8_technical_overview.md` §2.3](../vp8_technical_overview.md#23-lifecycle);
this document describes the file that implements every line of that
diagram.

---

## Preamble and dependencies

The includes at the top (`vp8/vp8_dx_iface.c:11-33`) reflect the file's
"adapter" position. From the public API it pulls in
`vpx/vpx_decoder.h`, `vpx/vp8dx.h` (for VP8-specific control IDs and
`vpx_decrypt_init`), and the private interface contract in
`vpx/internal/vpx_codec_internal.h`. From the decoder core it pulls
`common/onyxd.h` (the `VP8D_CONFIG` struct passed to
`vp8_create_decoder_instances`), `decoder/onyxd_int.h` (the full
`VP8D_COMP` and `FRAGMENT_DATA` definitions), and the three RTCD shims
(`vp8_rtcd.h`, `vpx_dsp_rtcd.h`, `vpx_scale_rtcd.h`) whose initializers
must fire before any decoder DSP is called. `decoder/error_concealment.h`
is gated behind `CONFIG_ERROR_CONCEALMENT` because the file holds an EC
allocation branch deep inside `vp8_decode`.

### `VP8_CAP_POSTPROC` and `VP8_CAP_ERROR_CONCEALMENT` (macros, `vp8_dx_iface.c:34-36`)

```c
#define VP8_CAP_POSTPROC (CONFIG_POSTPROC ? VPX_CODEC_CAP_POSTPROC : 0)
#define VP8_CAP_ERROR_CONCEALMENT \
  (CONFIG_ERROR_CONCEALMENT ? VPX_CODEC_CAP_ERROR_CONCEALMENT : 0)
```

These two macros expand to the corresponding public capability bit if
the build was configured with that feature, and to `0` otherwise. They
are OR'd into the `caps` field of the `vpx_codec_iface_t` instance at
the bottom of the file (`vp8_dx_iface.c:741-742`). A typical
`--disable-postproc --disable-error-concealment` build (the one
`vp8_files.md` was written to describe) reports
`VPX_CODEC_CAP_DECODER | VPX_CODEC_CAP_INPUT_FRAGMENTS` and nothing
else, which lets callers query
`vpx_codec_get_caps(iface) & VPX_CODEC_CAP_POSTPROC` and discover at
runtime that post-processing was compiled out — a much better outcome
than letting `VP8_SET_POSTPROC` fail mysteriously later.

### `vp8_stream_info_t` (typedef, `vp8_dx_iface.c:38`)

```c
typedef vpx_codec_stream_info_t vp8_stream_info_t;
```

A simple rename. VP8 carries no fields beyond what the generic
`vpx_codec_stream_info_t` (just `sz`, `w`, `h`, `is_kf`) can hold, so
the "VP8-specific stream info" is structurally identical to the
generic one. The alias survives because `vp8_get_si` (below) uses it
to compute how many bytes to copy out, distinguishing the
caller-allocated public type from any possible larger VP8-private
extension — a forward-compatibility hook that was never exercised but
costs nothing.

### `mem_seg_id_t` (enum, `vp8_dx_iface.c:41`)

```c
typedef enum { VP8_SEG_ALG_PRIV = 256, VP8_SEG_MAX } mem_seg_id_t;
```

A legacy artifact from an older libvpx memory-tracking scheme where
each subsystem owned a numbered "memory segment." Nothing in this file
or in any other file in the minimal decoder build actually references
these identifiers — the enum is dead code preserved only because
removing it would touch the ABI. Worth knowing only so that you do not
go searching for code that uses it.

### `NELEMENTS` (macro, `vp8_dx_iface.c:42`)

```c
#define NELEMENTS(x) ((int)(sizeof(x) / sizeof((x)[0])))
```

The standard "compile-time array length" macro, cast to `int` so it
participates cleanly in signed arithmetic. Defined here but, again, not
actually referenced inside the file as it currently stands; it is
inherited from a version that walked control-fn maps with explicit
length counts.

---

## The per-instance front-end state

### `struct vpx_codec_alg_priv` (`vp8_dx_iface.c:44-64`)

This is the heart of the file: the structure pointed to by `ctx->priv`
on every initialized `vpx_codec_ctx_t` for VP8 decoding. The
declaration is the **algorithm-specific definition** of the opaque
type `vpx_codec_alg_priv_t` that the internal API forward-declares in
`vpx/internal/vpx_codec_internal.h:67`. Each control-callback and
vtable slot below receives a `vpx_codec_alg_priv_t *` and immediately
treats it as a pointer to *this* concrete struct — the contract is
that the algorithm that filled in the `init` slot is the one
responsible for the layout of `priv`.

Field roles, in order:

- `vpx_codec_priv_t base` — must be first, because the public layer
  casts `ctx->priv` to `vpx_codec_priv_t *` whenever it manipulates
  fields like `err_detail` or `init_flags` (see
  `vpx/internal/vpx_codec_internal.h:345-359`). C "first-member-aliasing"
  is what makes the cast legal.
- `vpx_codec_dec_cfg_t cfg` — an internal copy of the caller's
  decoder configuration. `vp8_init_ctx` makes the copy and rebinds
  `ctx->config.dec` to point at this copy so the application's
  configuration buffer can go out of scope safely.
- `vp8_stream_info_t si` — the most-recently-seen stream info (width,
  height, is_kf flag). Updated on every frame by `vp8_peek_si_internal`
  inside `vp8_decode`. Cleared (`w = h = 0`) on hard errors so that the
  next decode call sees "no known dimensions" and forces full
  reallocation.
- `int decoder_init` — a one-shot flag: 0 before the first successful
  `vp8_create_decoder_instances`, 1 thereafter. This is the latch that
  defers `VP8D_COMP` construction until the first keyframe is in hand.
- `int restart_threads` (multithread builds only) — set by
  `vp8_decode`'s error path when a multithreaded decode crashes and
  all worker threads have been torn down. The next decode call sees the
  flag and respawns the worker pool before parsing.
- `int postproc_cfg_set` / `vp8_postproc_cfg_t postproc_cfg` — last
  values supplied via `VP8_SET_POSTPROC`. Default flags are filled in
  on the first decode if the application asked for postproc at init
  time but never sent a config.
- `vpx_decrypt_cb decrypt_cb` / `void *decrypt_state` — the
  byte-stream decryption callback installed via `VPXD_SET_DECRYPTOR`
  and the opaque state pointer that goes with it. Wired through to
  `VP8D_COMP::decrypt_cb` once `pbi[0]` exists.
- `vpx_image_t img` — the `vpx_image_t` *handle* that `vp8_get_frame`
  hands back to the application. Living here (rather than being newly
  allocated each frame) means the same pointer is valid for the
  lifetime of the decoder; `vp8_get_frame` just refills its fields.
- `int img_setup` — declared but unused in the current code; another
  vestige.
- `struct frame_buffers yv12_frame_buffers` — owns the array of
  `VP8D_COMP *pbi[MAX_FB_MT_DEC]` (see `onyxd_int.h:50-57`). In the
  single-threaded build only `pbi[0]` is populated; the array exists
  for frame-parallel decoding modes that this minimal build does not
  use. `vp8_create_decoder_instances` populates `pbi[0]`;
  `vp8_remove_decoder_instances` tears it back down.
- `void *user_priv` — opaque pointer the application passed into
  `vpx_codec_decode`. Copied into the `vpx_image_t::user_priv` on the
  way out, so callers can attach a user cookie to every decoded frame.
- `FRAGMENT_DATA fragments` — input-fragment accumulator (see
  `onyxd_int.h:41-46`). If the caller enabled
  `VPX_CODEC_USE_INPUT_FRAGMENTS`, one frame's payload can be fed in
  pieces; `fragments.ptrs[]` and `fragments.sizes[]` collect them
  until `vp8_decode` is called with the "flush" signal `(NULL, 0)`.

The struct is allocated by `vpx_calloc` (so all flags start at 0) and
freed in one shot by `vp8_destroy`; ownership of the deeper
`VP8D_COMP` allocations is delegated to the
`vp8_create_decoder_instances` / `vp8_remove_decoder_instances` pair.

---

## Construction and teardown

### `vp8_init_ctx` (`vp8_dx_iface.c:66-85`)

Internal helper that allocates the `vpx_codec_alg_priv_t`, hooks it
into `ctx->priv`, initializes the most fragile fields by hand
(`si.sz` must be set so that downstream "size-stamped" copies work,
`decrypt_cb` defaults to "no decryptor"), and, if the caller supplied
a decoder config, makes an internal copy. The last step rebinds
`ctx->config.dec` to point at the internal copy: the **invariant** is
that `ctx->config.dec`, after `init` returns, points at memory owned
by libvpx, not at the application's stack-allocated configuration
buffer.

Returns `1` on allocation failure (caller will translate to
`VPX_CODEC_MEM_ERROR`), `0` on success.

### `vp8_init` (`vp8_dx_iface.c:87-117`)

This is the function stored in the `init` slot of the vtable; it is
what `vpx_codec_dec_init_ver()` ultimately invokes. The function has
two responsibilities:

1. **Bring up the RTCD dispatch tables** unconditionally, by calling
   `vp8_rtcd()`, `vpx_dsp_rtcd()`, `vpx_scale_rtcd()`. Each of these
   is itself guarded by `vpx_once` (see
   `vp8_technical_overview.md` §14.2) so calling them on every new
   context is safe — only the first call does work. Doing it here
   guarantees that even applications that never call decode (e.g.,
   peek-only consumers) get correctly populated function-pointer
   tables.
2. **Allocate the priv block** via `vp8_init_ctx`, but only if it has
   not been allocated already. The "already?" check exists because some
   multi-resolution encoder code paths construct a `vpx_codec_ctx_t`
   with `ctx->priv` pre-populated and then call `init` to finalize
   it; the decoder never goes down that path, but the test is cheap.

After `vp8_init_ctx`, the function copies the `VPX_CODEC_USE_INPUT_FRAGMENTS`
init flag into `priv->fragments.enabled` so the rest of the file can
just consult that boolean. The `data` argument (the multi-resolution
encoder config) is unused for the decoder and explicitly ignored with
`(void)data`.

**Notable invariant**: this function does *not* allocate `VP8D_COMP`
or any frame buffers. VP8 keyframes carry the picture dimensions, so
the decoder cannot allocate buffers of the right size until the first
keyframe is seen. That happens inside `vp8_decode`.

### `vp8_destroy` (`vp8_dx_iface.c:119-125`)

The vtable's `destroy` slot. Tears down the decoder instances (which
internally walks `pbi[0..MAX_FB_MT_DEC]`, releases all YV12 buffers,
joins any worker threads, and frees the `VP8D_COMP`) and then frees
the priv block itself. The function unconditionally returns
`VPX_CODEC_OK`; the underlying free routines have no failure mode.

The signature takes `vpx_codec_alg_priv_t *` rather than
`vpx_codec_ctx_t *` because the public dispatcher
(`vpx_codec_destroy`) has already extracted the priv pointer when it
makes this call.

---

## Stream-info probing

### `vp8_peek_si_internal` (`vp8_dx_iface.c:127-176`)

Parses just enough of a frame's first ten bytes to determine whether
it is a key frame and, if so, what its dimensions are. This is the
implementation behind `vpx_codec_peek_stream_info()` (when called
without an attached decoder) and is also invoked once per frame from
inside `vp8_decode` (so dimension changes are detected before the
heavyweight parser runs).

The format it parses is described in RFC 6386 §9.1 ("Uncompressed
Data Chunk") and summarized in the inline comment
(`vp8_dx_iface.c:139-144`): 3 frame-tag bytes, a 3-byte sync code
(`0x9d 0x01 0x2a`) that must match, and 4 bytes carrying the 14-bit
width/height in little-endian half-words. Bit 0 of the first byte is
the **inverse** key-frame flag: 0 means key frame, 1 means inter
frame.

Three behaviors deserve attention:

- **Decryption support**. If the caller has installed a
  `vpx_decrypt_cb`, the function decrypts up to ten bytes into a
  local `clear_buffer[10]` and parses out of that. This is the only
  place outside the bool decoder that consults the decryption
  callback, and is why the function carries the `decrypt_cb` /
  `decrypt_state` parameters that `vp8_peek_si` (below) hides.
- **Non-key-frame handling**. If bit 0 is set, the function returns
  `VPX_CODEC_UNSUP_BITSTREAM` *and* sets `si->is_kf = 0`. The caller
  in `vp8_decode` knows to interpret that combination as "this is an
  inter frame, no resize possible, carry on."
- **Buffer-wrap guard** at line 136: `if (data + data_sz <= data)`
  detects pointer arithmetic that would wrap around the address space
  (i.e. someone passing a buffer that ostensibly extends past
  `UINTPTR_MAX`). The check is paranoid because the result feeds
  directly into the boolean decoder's pointer arithmetic.

### `vp8_peek_si` (`vp8_dx_iface.c:178-181`)

A trivial wrapper that fills in `NULL` for the decryption callbacks
and forwards. It is what occupies the `dec.peek_si` slot of the
vtable. Splitting the implementation in two lets `vp8_decode`
re-use the same parser while supplying a real decryption callback.

### `vp8_get_si` (`vp8_dx_iface.c:183-197`)

The vtable's `dec.get_si` slot. Copies the most-recently-recorded
stream info out of the priv block into the caller's buffer, using
**size-stamped copying**: the caller's `si->sz` is consulted to decide
how many bytes to copy. This way the caller's struct can be smaller or
larger than libvpx's internal one and still get only the fields it has
room for. After the copy, `si->sz` is overwritten with the actual
number of bytes written, so the application can detect a truncation.

This function is purely a read of cached state — it never re-parses the
bitstream — so it works only after at least one `vp8_decode` call has
populated `ctx->si`.

---

## Internal plumbing used by `vp8_decode` and the control callbacks

### `update_error_state` (`vp8_dx_iface.c:199-208`)

When a decode hits a fatal error, the parsers fill in
`pbi->common.error.error_code` and `.detail` (see
`vp8_technical_overview.md` §15) and longjmp back to the
`setjmp` site inside `vp8_decode`. This helper then copies the
error-detail string pointer up into `ctx->base.err_detail` so that
`vpx_codec_error_detail()` returns it to the application, and propagates
the error code back as the return value. The detail pointer is set only
when `has_detail` is true; otherwise it is cleared to `NULL` so a stale
message from an earlier frame cannot bleed through.

### `yuvconfig2image` (`vp8_dx_iface.c:210-237`)

Translates the decoder-internal frame format
(`YV12_BUFFER_CONFIG`, defined in `vpx_scale/yv12config.h`) into a
public `vpx_image_t`. The function hand-fills every field rather than
calling `vpx_img_wrap`, because `vpx_img_wrap` cannot represent
**independent strides for the Y, U, and V planes** and cannot describe
the asymmetric border layout that VP8's reconstruction buffers use.

A few subtle bits:

- `img->w` is set to the **stride**, not the display width, while
  `img->d_w` carries the display width. This is the standard
  `vpx_image_t` convention for "image dimensions" vs. "displayable
  dimensions."
- `img->h` is computed as `(y_height + 2*VP8BORDERINPIXELS + 15) & ~15`
  — i.e., the display height plus top and bottom borders (32 pixels
  each, per `VP8BORDERINPIXELS` in `vp8_technical_overview.md` §10),
  rounded up to a 16-row multiple. This matches the actual allocated
  height of the buffer, so a consumer that walks all `img->h` rows of
  `img->planes[Y]` reads only valid memory.
- `img->img_data_owner = 0` and `img->self_allocd = 0` mark the image
  as a **view** into memory owned by the decoder. Callers must not
  call `vpx_img_free` on it; they receive only a borrowed handle whose
  contents become invalid the next time `vp8_decode` runs.
- `img->user_priv` is set to the per-decode user-cookie that the
  caller passed into `vpx_codec_decode`. This is the mechanism for
  per-frame metadata flow.

### `update_fragments` (`vp8_dx_iface.c:239-283`)

Implements the input-fragment state machine. Three cases drive its
return value:

1. **Fragment mode off, normal data**: copy the (pointer, size) into
   `fragments.ptrs[0] / sizes[0]` with `count = 1` and return 1 so the
   caller proceeds to decode immediately. This is the default path for
   conventional callers that hand whole frames to `vpx_codec_decode`.
2. **Fragment mode off, flush signal `(NULL, 0)`**: return 0 — there
   is nothing buffered, nothing to do.
3. **Fragment mode on, data**: append to the partition arrays, return
   0 (so `vp8_decode` returns without decoding) until the caller
   eventually invokes decode with `(NULL, 0)`. On that flush call the
   function falls through past every `if`, returning 1 and allowing
   `vp8_decode` to use the accumulated fragments. A `MAX_PARTITIONS`
   cap (9; the maximum legal partition count plus the prediction
   partition) protects against runaway accumulation.

The `volatile vpx_codec_err_t *res` out-parameter mirrors the
`volatile` discipline that surrounds the `setjmp` regions inside
`vp8_decode` — the compiler must not optimize stores into `*res` away
just because they precede a `return 0`. The `volatile` qualifier on
locals across `setjmp` boundaries is what the C standard requires for
correctness.

The "new frame" path at `vp8_dx_iface.c:244-248` zeros the partition
arrays whenever `count == 0`, ensuring no stale fragment pointers from
a previous frame can leak forward.

---

## The decode call

### `vp8_decode` (`vp8_dx_iface.c:285-529`)

The single largest function in the file. It occupies the vtable's
`dec.decode` slot. Conceptually it is six phases:

**Phase 1 — Fragment buffering** (`:292-297`). If fragment mode is
enabled and this call is a partial-frame accumulator, `update_fragments`
returns 0 and `vp8_decode` returns early without decoding. Only when
the caller signals "frame complete" does control reach phase 2. (The
short-circuit at line 292 is for the simpler "fragment mode off and
caller passed `(NULL, 0)`" case, which means "no data" and is a
no-op.)

**Phase 2 — Stream-info refresh and resolution-change detection**
(`:299-327`). Save the previous `si.w` / `si.h` into the locals
`w` / `h`, then re-parse the first 10 bytes of the (now-complete)
frame with `vp8_peek_si_internal`. If the frame is an inter frame the
peek returns `UNSUP_BITSTREAM`; this is fine and is rewritten to
`VPX_CODEC_OK` at line 309. The decoder refuses to consume a non-key
frame before any decoder instance has been initialized
(`!decoder_init && !si.is_kf` → `UNSUP_BITSTREAM`, line 315) — this
is the rule that callers must feed a key frame first. A subtle
defensive check at line 316 catches a bug-shaped corner case: the
decoder has been initialized once but is now seeing a peeked frame
whose dimensions came back as zero; that is treated as a corrupted
frame and a detail string is recorded.

If the parsed dimensions differ from the cached ones, the
`resolution_change` flag is set so phase 5 will reallocate.

**Phase 3 — Thread restart** (`:329-348`, `#if CONFIG_MULTITHREAD`).
If a prior decode crashed and tore down the worker pool, this block
re-spawns the workers before parsing. Guarded by its own `setjmp` so
that an allocation failure during thread creation does not leak.

**Phase 4 — First-frame allocation** (`:349-381`). On the very first
successful peek, build a `VP8D_CONFIG` from the freshly known
dimensions and call `vp8_create_decoder_instances`, which allocates
`VP8D_COMP` for `pbi[0]`. The `error_concealment` field is wired
straight from the init flag the caller passed to
`vpx_codec_dec_init_ver`. If no postproc config was ever supplied by
the application, a reasonable default is filled in here. On failure,
the cached dimensions are zeroed so that a later "resync" call sees
"no dimensions known" and re-attempts allocation.

**Phase 5 — Reconfigure decrypt callback** (`:383-389`). Every call,
even after the first, re-pushes the application's current decrypt
callback into `pbi[0]`. The caller is allowed to change the decryptor
between frames, and `vp8_decode` honors that by simply re-pushing.

**Phase 6 — Resolution change + actual decode** (`:391-526`). All
inside `if (!res)`, gated by no error from phases 2-4. The
resolution-change branch (`:394-486`) runs only when the parsed
dimensions differ from the cached ones: it overwrites
`pc->Width` / `pc->Height`, wraps the reallocation in a `setjmp` (so
that `vpx_internal_error` from inside `vp8_alloc_frame_buffers` can
longjmp back here and return -1), validates that the new dimensions
are positive, optionally tears down and re-allocates multithreaded
worker temp buffers, and calls `vp8_alloc_frame_buffers` to size the
YV12 array. Block-offset tables in every per-thread `MACROBLOCKD` are
rebuilt by `vp8_build_block_doffsets`. Under `CONFIG_ERROR_CONCEALMENT`,
an additional `MODE_INFO` grid for the previous frame is allocated
here, since EC needs to remember the last frame's mode/MV decisions.

After the resolution branch, the function arms `setjmp` once more
(line 488) and calls `vp8dx_receive_compressed_data` (line 519),
which is the entry point into the decoder core defined in
`onyxd_if.c`. That call returns either successfully or via longjmp;
both cases are handled. On longjmp, the function marks the LAST
reference buffer corrupted (a conservative assumption — we do not know
which buffers the missing frame would have updated), decrements the
new-frame refcount, and remembers (in `restart_threads`) that the
worker pool needs to be respawned on the next call.

Finally, `fragments.count` is reset to zero so the next call starts a
fresh accumulation, and the `setjmp` guard is disarmed.

**Invariants and gotchas worth keeping in mind**:

- Local variables that are read after a `setjmp` (`res`,
  `resolution_change`, `w`, `h`) are declared `volatile`. Without
  that, the C standard permits the compiler to keep them in registers
  across the jump and the post-jump read may see indeterminate values.
- `setjmp`'s `pbi->common.error.setjmp` companion flag is paired
  manually: set to 1 just before entering an unsafe region, set to 0
  immediately after a safe exit. `vpx_internal_error` consults that
  flag to decide whether to longjmp or to fall through to a normal
  return.
- The function returns -1 (an out-of-band value not in
  `vpx_codec_err_t`) on one specific path (line 410) — when
  reallocation fails during a resolution change. This matches the
  return value of `vp8dx_receive_compressed_data` for the same kind
  of failure and is what the caller in `vpx_codec_decode` expects to
  translate into an error.
- The fragment-mode flush `(data=NULL, data_sz=0)` is essential and
  is the *only* way to drive decoding when input-fragments are
  enabled. Conventional callers that always pass complete frames need
  never see it.

---

## Frame retrieval

### `vp8_get_frame` (`vp8_dx_iface.c:531-558`)

The vtable's `dec.get_frame` slot. The contract from
`vpx/internal/vpx_codec_internal.h:219` is that this function is an
**iterator**: the caller initializes `*iter` to `NULL` and calls
repeatedly until the function returns `NULL`. VP8 produces at most one
displayable frame per `vp8_decode` call, so the implementation uses
`*iter` as a one-shot flag: on the first call (`*iter == NULL`) it
wraps `frame_to_show` in `ctx->img`, stores a non-null cookie in
`*iter`, and returns the image; on any subsequent call (`*iter != NULL`)
it returns `NULL`.

The actual frame extraction is delegated to
`vp8dx_get_raw_frame`, which decides what to show (it may apply
postprocessing if compiled in) and returns the pointer in `sd`. If
nothing is ready (because the decoded frame was a "show_frame = 0"
internal-only reference update), it returns nonzero and the function
returns `NULL` without setting `*iter`, so a polite caller will simply
move on.

Note the postproc-flag plumbing on lines 543-547: each call to
`vp8_get_frame` re-pulls the postproc settings out of the priv struct
into a stack `vp8_ppflags_t` and hands them to `vp8dx_get_raw_frame`.
This permits the caller to change postproc settings on the fly via
`VP8_SET_POSTPROC` and have them take effect on the very next
`get_frame`.

### `image2yuvconfig` (`vp8_dx_iface.c:560-585`)

The inverse of `yuvconfig2image`: takes a caller-supplied
`vpx_image_t *` and fills a `YV12_BUFFER_CONFIG *` referring to the
same memory. Used by `vp8_set_reference` and `vp8_get_reference` so
that the caller can hand in an arbitrary external image as a
replacement reference buffer (or as the destination of a reference
copy).

Two assumptions deserve attention:

- It hard-codes 4:2:0 chroma layout (`uv_w = (d_w + 1) / 2`,
  `uv_h = (d_h + 1) / 2`) and that the UV planes share a stride
  (`uv_stride = img->stride[U]`). VP8 is itself I420-only, so this is
  always correct for VP8 callers.
- It computes `border = (stride - d_w) / 2`, i.e. the symmetric
  horizontal border embedded in the stride. This is what
  `yuvconfig2image` produces, so a round trip is lossless; an image
  not produced by `yuvconfig2image` may have a `border` value the
  decoder did not expect.

The function always returns `VPX_CODEC_OK`; the return type exists for
forward compatibility with possible later validation.

---

## Control callbacks

Each function below has the signature
`vpx_codec_err_t (vpx_codec_alg_priv_t *, va_list)` required by the
`vpx_codec_control_fn_t` typedef
(`vpx/internal/vpx_codec_internal.h:160`). The application-visible
control IDs are declared in `vpx/vp8.h` and `vpx/vp8dx.h` along with
type-checking macros (`VPX_CTRL_USE_TYPE`) that the C compiler enforces
at the call site; here we just `va_arg` the data back out.

### `vp8_set_reference` (`vp8_dx_iface.c:587-604`)

Handles `VP8_SET_REFERENCE`. The application supplies a
`vpx_ref_frame_t` containing a `frame_type` (LAST/GOLDEN/ALTREF) and
a `vpx_image_t` whose contents should be *copied into* the decoder's
named reference slot. The helper `image2yuvconfig` produces a
matching `YV12_BUFFER_CONFIG`, and the real work happens inside
`vp8dx_set_reference` (declared in `common/onyxd.h:52-54`,
implemented in `onyxd_if.c`).

**Gotcha**: if `pbi[0]` is still `NULL` (no decode has run yet), the
function returns `VPX_CODEC_CORRUPT_FRAME`. That error code is a
misnomer here — there is no frame at all yet — but it is the closest
existing code for "no decoder instance exists." Tests that rely on
the exact code must be aware of this.

### `vp8_get_reference` (`vp8_dx_iface.c:606-623`)

Handles `VP8_COPY_REFERENCE`. Same shape as `vp8_set_reference`, but
the direction is reversed: the decoder copies one of its reference
slots **out into** the caller-provided `vpx_image_t`. Useful for
external scrubbers and for tests that want to introspect what the
decoder has cached.

### `vp8_get_quantizer` (`vp8_dx_iface.c:625-633`)

Handles `VPXD_GET_LAST_QUANTIZER`. Returns the base AC quantizer of
the most recently decoded frame (via `vp8dx_get_quantizer`). The
control id is shared with VP9 but uses the same callback name across
codecs. Returns `VPX_CODEC_INVALID_PARAM` on a null output pointer,
`VPX_CODEC_CORRUPT_FRAME` if there is no decoder yet, otherwise
writes the quantizer through the supplied `int *`.

### `vp8_set_postproc` (`vp8_dx_iface.c:635-653`)

Handles `VP8_SET_POSTPROC`. Under `CONFIG_POSTPROC`, copies the
caller's `vp8_postproc_cfg_t` into the priv block and sets
`postproc_cfg_set = 1` so the first-frame default-fill in `vp8_decode`
will not overwrite it. Compiled out (returns `VPX_CODEC_INCAPABLE`)
when postproc is disabled. This is the runtime symmetry that the
`VP8_CAP_POSTPROC` capability bit advertises at compile time.

### `vp8_get_last_ref_updates` (`vp8_dx_iface.c:655-671`)

Handles `VP8D_GET_LAST_REF_UPDATES`. The decoder writes back a bitmap
of which of `{VP8_LAST_FRAME, VP8_GOLD_FRAME, VP8_ALTR_FRAME}` were
refreshed by the last frame, derived from `refresh_last_frame`,
`refresh_golden_frame`, and `refresh_alt_ref_frame` in `VP8_COMMON`.
Used by external clients implementing reference-frame management on
top of libvpx (e.g., scalable-video adapters that need to know which
buffer the latest frame just overwrote).

### `vp8_get_last_ref_frame` (`vp8_dx_iface.c:673-692`)

Handles `VP8D_GET_LAST_REF_USED`. Returns a bitmap of which reference
slots the last frame **read from** (as opposed to wrote to). Uses
`vp8dx_references_buffer` which checks whether the slot index used by
each named reference (LAST/GOLDEN/ALTREF) maps to a buffer that was
actually consulted by any MB in the frame.

### `vp8_get_frame_corrupted` (`vp8_dx_iface.c:694-707`)

Handles `VP8D_GET_FRAME_CORRUPTED`. Looks at the
`YV12_BUFFER_CONFIG::corrupted` flag of `cm->frame_to_show`. That flag
is set by the decoder core (in error-concealment builds) or by the
error path in `vp8_decode` itself (which marks LAST as corrupted on
longjmp). If `frame_to_show` is null because no frame has yet been
displayed, the function returns `VPX_CODEC_ERROR` — distinct from
`VPX_CODEC_INVALID_PARAM`, which is reserved for "caller passed NULL."

### `vp8_set_decryptor` (`vp8_dx_iface.c:709-721`)

Handles `VPXD_SET_DECRYPTOR`. The caller supplies a
`vpx_decrypt_init` struct (defined in `vpx/vp8dx.h:174`) carrying a
function pointer and an opaque state. This function copies both into
the priv block, where `vp8_decode` later forwards them to `pbi[0]`
on every decode call. The actual decryption happens inside the bool
decoder (`vp8dx_bool_decoder_fill` consults the callback on every
buffer refill) and inside `vp8_peek_si_internal`.

A `NULL` `vpx_decrypt_init *` is allowed and clears both fields — the
documented way to *uninstall* a decryptor mid-stream. The function
always returns `VPX_CODEC_OK`.

---

## The control map and the vtable

### `vp8_ctf_maps` (`vp8_dx_iface.c:723-733`)

The table that maps integer control IDs to the eight callbacks above.
The dispatcher `vpx_codec_control_()` in `vpx/src/vpx_codec.c` walks
this array linearly looking for a matching `ctrl_id`, then invokes the
associated function with the application's `va_list`. The terminating
sentinel `{ -1, NULL }` is what tells the dispatcher it has reached
the end (the contract is documented at
`vpx/internal/vpx_codec_internal.h:174-177`).

The table is `static`, so it is not visible outside this translation
unit; only the `vp8_ctf_maps` pointer in the vtable below exposes it.

### `vpx_codec_vp8_dx_algo` and `vpx_codec_vp8_dx()` (`vp8_dx_iface.c:735-765`)

```c
#ifndef VERSION_STRING
#define VERSION_STRING
#endif
CODEC_INTERFACE(vpx_codec_vp8_dx) = { … };
```

The macro `CODEC_INTERFACE(id)` (defined in
`vpx/internal/vpx_codec_internal.h:390-392`) expands to **two**
declarations:

```c
vpx_codec_iface_t *vpx_codec_vp8_dx(void) { return &vpx_codec_vp8_dx_algo; }
vpx_codec_iface_t  vpx_codec_vp8_dx_algo;
```

That is, both a data symbol (`_algo`) and a getter function that
returns its address. The getter exists because some build environments
(dynamic loading, language bindings) can resolve function symbols more
robustly than data symbols (the comment in `vpx_codec_internal.h:381-389`
attributes this to issue #169). Applications are expected to call
`vpx_codec_vp8_dx()` and pass the result into
`vpx_codec_dec_init_ver()`; direct references to `vpx_codec_vp8_dx_algo`
do work but are not the supported surface.

The `VERSION_STRING` macro is defined at build time (from
`vpx_version.h`) to append a "v1.2.3-…" tag to the human-readable
name; when absent, `#define VERSION_STRING` empty makes the
concatenation a no-op so the build always succeeds.

Wiring of each slot to the functions above:

| `vpx_codec_iface_t` field | Value here                       | Defined at file:line |
|---------------------------|----------------------------------|----------------------|
| `name`                    | `"WebM Project VP8 Decoder" VERSION_STRING` | `:739` |
| `abi_version`             | `VPX_CODEC_INTERNAL_ABI_VERSION` (=5) | `:740` |
| `caps`                    | `VPX_CODEC_CAP_DECODER \| VP8_CAP_POSTPROC \| VP8_CAP_ERROR_CONCEALMENT \| VPX_CODEC_CAP_INPUT_FRAGMENTS` | `:741-742` |
| `init`                    | `vp8_init`                       | `:87-117` |
| `destroy`                 | `vp8_destroy`                    | `:119-125` |
| `ctrl_maps`               | `vp8_ctf_maps`                   | `:723-733` |
| `dec.peek_si`             | `vp8_peek_si`                    | `:178-181` |
| `dec.get_si`              | `vp8_get_si`                     | `:183-197` |
| `dec.decode`              | `vp8_decode`                     | `:285-529` |
| `dec.get_frame`           | `vp8_get_frame`                  | `:531-558` |
| `dec.set_fb_fn`           | `NULL`                           | `:752` |
| `enc.*` (all eight slots) | `0`/`NULL`                       | `:754-764` |

`dec.set_fb_fn` is `NULL` because VP8 (unlike VP9) does not implement
the external-frame-buffer callback protocol; the public dispatcher in
`vpx/src/vpx_decoder.c` detects the `NULL` and returns
`VPX_CODEC_ERROR` if the application tries to use it. The eight
encoder slots are zero because, well, this is the decoder.

That is the full surface the VP8 decoder exposes to the libvpx public
API: ten function pointers, one capability mask, one control-map array,
one name string. Everything else in the decoder is reached transitively
from there.
