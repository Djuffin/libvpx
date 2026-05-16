# `vp8/decoder/onyxd_if.c` — Decoder Instance Lifecycle and Frame Driver

## Role in the decoder

Within the pipeline laid out in
[vp8_technical_overview.md, section 1 ("Big-picture pipeline")](../vp8_technical_overview.md#1-big-picture-pipeline)
this translation unit sits exactly one layer below the public C API
(`vp8/vp8_dx_iface.c`) and one layer above the actual bit-crunching
work in `decodeframe.c`, `decodemv.c`, `detokenize.c` and `dboolhuff.c`.
Its job description is narrow but load-bearing:

* **Allocate and tear down a `VP8D_COMP`** — the decoder's per-instance
  state struct defined in `vp8/decoder/onyxd_int.h:59`.
* **Own the small fleet of YV12 frame buffers** (`NUM_YV12_BUFFERS == 4`,
  see `vp8/common/onyxc_int.h:36`) and the reference-count vector that
  governs the LAST / GOLDEN / ALTREF / new-frame slot rotation described
  in the overview's
  [section 10 ("Decoded picture buffer & reference management")](../vp8_technical_overview.md#10-decoded-picture-buffer--reference-management).
* **Drive a single frame through `vp8_decode_frame()`** — the boundary
  call into `decodeframe.c` — wrapping it with buffer acquisition,
  buffer rotation, x87/SSE state hygiene, and error propagation.
* Provide the **reference-frame copy/inject helpers**
  (`vp8dx_get_reference`, `vp8dx_set_reference`) that back the
  `VP8_COPY_REFERENCE` / `VP8_SET_REFERENCE` control codes on the public
  surface.
* Expose three small **introspection helpers**
  (`vp8dx_get_quantizer`, `vp8dx_references_buffer`,
  `vp8dx_get_raw_frame`).

Everything here is glue code in the literal sense: it shuffles pointers,
manages a four-slot DPB, and hands control to the next layer. No
bitstream parsing, no DSP, no motion compensation — those live in
sibling files. Read this file when you want to understand *what runs
around* `vp8_decode_frame`, not what runs inside it.

The public function names follow the historical libvpx convention:
`vp8dx_*` for symbols originally exposed to the integrating
application (now reached via the `vpx_codec_*` shim in
`vp8/vp8_dx_iface.c`), `vp8_*` for internal-but-cross-file helpers
(`vp8_create_decoder_instances`, `vp8_remove_decoder_instances`).

---

## Headers and dependencies

The opening block at `vp8/decoder/onyxd_if.c:11-42` pulls in everything
needed for the four concerns listed above:

* `onyxc_int.h` — the `VP8_COMMON` decoder/encoder-shared state.
* `onyxd.h` — the public `VP8D_CONFIG` and the `vp8dx_*` prototypes
  this file implements.
* `onyxd_int.h` — the private `VP8D_COMP` struct, `FRAGMENT_DATA`,
  `MAX_FB_MT_DEC`, and the `vp8_create_decoder_instances` prototype.
* `alloccommon.h`, `loopfilter.h`, `swapyv12buffer.h`,
  `quant_common.h`, `reconintra.h` — the subsystems that have to be
  initialised when a decompressor is created.
* `vpx_scale/vpx_scale.h` — needed even on a `--disable-spatial-resampling`
  build (see `documentation/vp8_files.md` section B); the header is
  unconditionally included even though its body is mostly guarded out.
* `vpx_ports/system_state.h` — `vpx_clear_system_state()` hygiene.
* `vpx_ports/vpx_once.h` — the `once()` one-shot initializer used to
  fire RTCD and intra-predictor table setup exactly once per process.
* `vpx_ports/vpx_timer.h` — included for ABI compatibility but unused
  in this file as written.
* `detokenize.h` — only for the public function exposed by detokenize
  that this file does *not* call directly; kept because the header
  chain transitively pulls fields used by `VP8D_COMP`.

The two `extern`/forward declarations at `vp8/decoder/onyxd_if.c:44-46`
hoist symbols that are either defined elsewhere
(`vp8_init_loop_filter` lives in `vp8_loopfilter.c`) or later in this
file (`get_free_fb`, `ref_cnt_fb`).

---

## Static helpers

### `static void initialize_dec(void)` — process-wide one-shot init

Defined at `vp8/decoder/onyxd_if.c:48-56`. Two things must happen
exactly once per process before any decode work runs:

1. `vpx_dsp_rtcd()` — populate the `vpx_dsp_rtcd_defs.pl`-generated
   function-pointer table with CPU-dispatched DSP kernels.
2. `vp8_init_intra_predictors()` — fill the intra-predictor function
   table used by `reconintra*.c`.

The function uses a *plain* `static volatile int init_done` guard, not
a mutex, because it is itself only ever called through
`once(initialize_dec)` from `create_decompressor` (line 118), and
`once()` (defined in `vpx_ports/vpx_once.h`) is the portable
single-execution primitive that *does* take the lock. The `volatile`
inside the function is therefore defensive belt-and-suspenders against
older compilers reordering the assignment past the calls; the real
serialization happens one level up.

### `static void remove_decompressor(VP8D_COMP *pbi)` — tear-down

Defined at `vp8/decoder/onyxd_if.c:58-64`. Mirrors
`create_decompressor`. Three steps:

1. If error concealment was compiled in, drop the overlap lists used
   to reconstruct missing MV partitions.
2. `vp8_remove_common(&pbi->common)` — free the YV12 frame buffers,
   the mode-info grid, the segment map, and all other buffers owned by
   `VP8_COMMON` (see `vp8/common/alloccommon.c`).
3. `vpx_free(pbi)` — release the aligned allocation itself.

Gotcha: this function is called from two error paths inside
`create_decompressor` (one via `setjmp`, one not), and from
`vp8_remove_decoder_instances`. After it runs the caller must NULL the
pointer; this file does so explicitly at line 456.

### `static struct VP8D_COMP *create_decompressor(VP8D_CONFIG *oxcf)` — instance constructor

Defined at `vp8/decoder/onyxd_if.c:66-121`. This is the only place a
`VP8D_COMP` is ever allocated. The construction sequence is worth
walking line-by-line because every step encodes a startup invariant:

* `vpx_memalign(32, sizeof(VP8D_COMP))` (line 67) — 32-byte alignment
  is required because the struct holds an embedded `MACROBLOCKD` and
  `VP8_COMMON` (declared with `DECLARE_ALIGNED(16, …)` in
  `vp8/decoder/onyxd_int.h:60,64`) plus internal arrays that some SIMD
  kernels load with aligned moves.
* `memset(pbi, 0, sizeof(VP8D_COMP))` (line 71) — zeroes the entire
  struct so that the partial-failure path below (which calls
  `remove_decompressor`) can safely traverse pointers that may not
  have been assigned yet.
* `setjmp(pbi->common.error.jmp)` (line 73) — installs the long-jump
  landing pad used by `vpx_internal_error`. While `setjmp` is armed
  any internal-error call from the helpers invoked below
  (`vp8_create_common`, `vp8cx_init_de_quantizer`,
  `vp8_loop_filter_init`) unwinds back here, where the partial
  instance is freed and `NULL` is returned. Note that `0` (not
  `NULL`) is returned on line 76 — historical libvpx style.
* `pbi->common.error.setjmp = 1` (line 79) immediately followed by
  `... = 0` (line 94) brackets the failure-handled region. Outside the
  brackets, `vpx_internal_error` will set the error code but will
  *not* `longjmp` — important, because once `create_decompressor`
  returns successfully the stack frame containing the `jmp_buf` is
  gone.
* `vp8_create_common` allocates the YV12 reference buffers and the
  mode-info grid.
* `vp8cx_init_de_quantizer(pbi)` (line 90) — initial fill of the
  dequant tables. The comment above the call explains why this is
  done at construction time *and* in `frame_init_dequantizer`: a
  guard in the per-frame path skips recomputation when the quantizer
  hasn't changed, but that guard would never fire on the first frame
  without this priming call.
* `vp8_loop_filter_init(&pbi->common)` — precomputes the loop-filter
  threshold tables (the "lfi" array).
* `pbi->ec_enabled` is set from `oxcf->error_concealment` only when
  error concealment is compiled in; otherwise `oxcf` is cast to
  `void` (line 100) to silence unused-parameter warnings and
  `ec_enabled` is forced to 0.
* `pbi->ec_active = 0` (line 106) is distinct from `ec_enabled`: EC is
  only *activated* after a keyframe has been decoded cleanly, even
  when it has been *enabled* by the caller.
* `pbi->independent_partitions = 0` (line 114). This flag becomes 1
  later if and only if a frame updates the token-probability table
  with equal probabilities across the `PREV_COEF` context — a special
  case that lets a multi-threaded decoder decode token partitions
  independently. It is established here because the decode loop reads
  it before any frame has set it.
* `vp8_setup_block_dptrs(&pbi->mb)` (line 116) — wires the
  `MACROBLOCKD`'s per-block descriptor pointers into the macroblock's
  contiguous coefficient/predictor storage.
* `once(initialize_dec)` (line 118) — fires the process-wide
  initialization described above. Placed *after* per-instance setup
  so the very first thread can still safely race a second thread
  starting a second decoder.

Returns `pbi` on success; `NULL` (literal `0`) on failure via either
the `setjmp` path or the initial `vpx_memalign` failure.

### `static int get_free_fb(VP8_COMMON *cm)` — frame-buffer slot allocator

Defined at `vp8/decoder/onyxd_if.c:193-202`. Linear scan over the
four-slot DPB looking for the first slot whose reference count is
zero, marks it as held with `fb_idx_ref_cnt[i] = 1`, returns the
slot index. The `assert(i < NUM_YV12_BUFFERS)` (line 199) encodes the
invariant that the protocol described in
[overview section 10](../vp8_technical_overview.md#10-decoded-picture-buffer--reference-management)
guarantees: with four slots and at most three live references
(LAST, GOLDEN, ALTREF) at any time, a free slot for the
just-decoded frame is *always* available. If the assert ever trips,
something has leaked a reference — typically a bug in
`swap_frame_buffers` or `ref_cnt_fb`.

### `static void ref_cnt_fb(int *buf, int *idx, int new_idx)` — reseat one reference

Defined at `vp8/decoder/onyxd_if.c:204-210`. Atomically (in the
single-threaded sense) moves the reference at `*idx` to `new_idx`:
decrement the old slot's refcount, retarget `*idx`, increment the
new slot. The `if (buf[*idx] > 0)` guard handles the first-frame
case where the slot starts at zero. This is the only primitive used
to mutate `fb_idx_ref_cnt` outside of `get_free_fb` and the explicit
post-decode decrement in `swap_frame_buffers` line 265 — keeping
the surface tiny is important because the four-slot accounting is
unforgiving.

### `static int swap_frame_buffers(VP8_COMMON *cm)` — DPB rotation after decode

Defined at `vp8/decoder/onyxd_if.c:213-268`. Called once per
successfully-decoded frame from `vp8dx_receive_compressed_data`. It
turns the flags parsed from the frame header
(`copy_buffer_to_arf`, `copy_buffer_to_gf`, `refresh_golden_frame`,
`refresh_alt_ref_frame`, `refresh_last_frame`) into pointer
reseating on the four-slot DPB. The five rules implemented here are
exactly the reference-update semantics specified in RFC 6386
section 9.10:

* `copy_buffer_to_arf` (lines 221-233) — 0 = no-op, 1 = copy LAST
  into the ALTREF slot, 2 = copy GOLDEN/ALTREF source into the
  ALTREF slot. "Copy" here means refcount manipulation, not a pixel
  copy: the ALTREF index simply points at the LAST or GOLDEN slot.
  A value other than 0/1/2 is a malformed stream — `err = -1`.
* `copy_buffer_to_gf` (lines 235-247) — mirror logic for the
  GOLDEN slot, with 1 = LAST, 2 = ALTREF.
* `refresh_golden_frame`, `refresh_alt_ref_frame` (lines 249-255) —
  point the named slot at the just-decoded `new_fb_idx`.
* `refresh_last_frame` (lines 257-263) — if LAST is refreshed,
  `frame_to_show` (the pointer the application will subsequently
  read) is set to point at the new LAST; otherwise it points at the
  freshly-decoded buffer (which will not be retained beyond this
  frame).
* `cm->fb_idx_ref_cnt[cm->new_fb_idx]--` (line 265) — drops the
  refcount that `get_free_fb` set when it handed out `new_fb_idx`.
  After the rotations above, if `new_fb_idx` was promoted to LAST /
  GOLDEN / ALTREF its count is still ≥ 1; if it was not promoted
  (because none of the refresh flags pointed at it) but
  `refresh_last_frame` is false, the count drops to zero and the
  slot becomes immediately reusable.

The function returns `-1` if either `copy_buffer_to_*` field has an
out-of-range value; the caller treats this as a hard decode error.

Invariant after this function returns: every live reference
(LAST, GOLDEN, ALTREF) corresponds to a slot whose
`fb_idx_ref_cnt > 0`, and the slot pointed to by `frame_to_show` is
the one the application should display.

### `static int check_fragments_for_errors(VP8D_COMP *pbi)` — handle empty input

Defined at `vp8/decoder/onyxd_if.c:270-303`. Called at the very top
of `vp8dx_receive_compressed_data`. Distinguishes three input shapes:

* Normal input — `fragments.count >= 1` and `sizes[0] > 0` — returns
  1, decode proceeds.
* Empty input with error concealment off — the function returns 0
  (no error, but no frame to show) after marking the LAST reference
  as corrupted and clearing `show_frame`. The corruption mark is
  important so that any future frame that motion-compensates from
  LAST will inherit the corruption flag and the application can
  decide what to do.
* The same empty input with EC on — returns 1 to let
  `vp8_decode_frame` attempt concealment from the surrounding
  context.

The subtlety in lines 278-287 deserves attention: if the LAST slot
is currently shared with another reference (refcount > 1, e.g.
because the previous frame copied LAST into GOLDEN), marking the
existing buffer as corrupted would propagate corruption to that
other reference too. The function first allocates a fresh slot via
`get_free_fb`, copies LAST's pixels into it, retargets `lst_fb_idx`,
and only *then* sets `corrupted = 1`. This preserves the other
reference's clean state.

Return value: 0 = nothing to do (early exit ok), 1 = continue,
no negative values are produced by this helper.

---

## Public functions

### `vpx_codec_err_t vp8dx_get_reference(VP8D_COMP *, enum vpx_ref_frame_type, YV12_BUFFER_CONFIG *)` — export a reference frame

Defined at `vp8/decoder/onyxd_if.c:123-151`. Resolves
`ref_frame_flag` (one of `VP8_LAST_FRAME`, `VP8_GOLD_FRAME`,
`VP8_ALTR_FRAME` — declared in `vpx/vp8.h`) into the corresponding
slot index and copies the pixels into the caller-supplied
`YV12_BUFFER_CONFIG`. Dimensional mismatch (the caller's buffer must
be exactly as big as the internal buffer in all four
luma/chroma dimensions) is a `VPX_CODEC_ERROR` reported through
`vpx_internal_error`. Reached via the `VP8_COPY_REFERENCE` codec
control implemented at `vp8/vp8_dx_iface.c:618`.

Note that this function performs an actual pixel copy via
`vp8_yv12_copy_frame` — it is not zero-copy. The caller owns the
destination buffer.

### `vpx_codec_err_t vp8dx_set_reference(VP8D_COMP *, enum vpx_ref_frame_type, YV12_BUFFER_CONFIG *)` — inject a reference frame

Defined at `vp8/decoder/onyxd_if.c:153-191`. Symmetric to
`vp8dx_get_reference` but mutates the DPB: it acquires a fresh slot
via `get_free_fb`, then immediately decrements that slot's count
(line 183) because `ref_cnt_fb` on line 186 will increment it again
when it reseats the chosen reference index onto the new slot. (The
decrement-then-increment dance is required because `get_free_fb`
unconditionally marks its return slot as held with count 1; without
the decrement on line 183, the count would end at 2 after the
`ref_cnt_fb` call, leaking a reference.) Finally
`vp8_yv12_copy_frame` writes the caller's pixels into the slot now
pointed at by `*ref_fb_ptr`. Reached via the `VP8_SET_REFERENCE`
codec control at `vp8/vp8_dx_iface.c:599`.

This is the canonical primitive for "hand the decoder a synthetic
reference frame" — used by tests, by RTC stacks that want to recover
from a lost reference, and by the conformance suite.

### `int vp8dx_receive_compressed_data(VP8D_COMP *pbi)` — the per-frame driver

Defined at `vp8/decoder/onyxd_if.c:305-375`. The heart of this file.
Walks one frame from compressed bytes to a reconstructed YV12 in the
DPB. The steps (in code order) are:

1. `pbi->common.error.error_code = VPX_CODEC_OK` (line 309) — clear
   any sticky error from a previous frame.
2. `check_fragments_for_errors(pbi)` (line 311) — early-exit on
   empty input or signal that EC should run.
3. `cm->new_fb_idx = get_free_fb(cm)` (line 314) — reserve the slot
   the decoded frame will land in.
4. Populate `pbi->dec_fb_ref[]` (lines 317-320) — the array the
   inter-prediction machinery in `decodeframe.c` and
   `reconinter.c` dereferences when motion-compensating. The four
   slots are INTRA (the new frame itself; intra-block predictors
   read their own partial reconstruction), LAST, GOLDEN, ALTREF.
5. `vp8_decode_frame(pbi)` (line 322) — hand off to `decodeframe.c`.
   On failure, drop the refcount that step 3 set on `new_fb_idx`
   (lines 325-327), and propagate any structured error info from
   the macroblock decoder (`pbi->mb.error_info`, lines 331-335) into
   the common error struct so the integrator sees a meaningful
   `vpx_codec_error_detail`. Note the `goto decode_exit` (line 336)
   — the function still wants to run `vpx_clear_system_state()` on
   the way out.
6. `swap_frame_buffers(cm)` (line 339) — apply the refresh-flag
   rules.
7. `vpx_clear_system_state()` (line 344) — restores the FPU/SSE
   control word to its caller-visible state. The VP8 IDCT and
   loop-filter kernels may have changed MMX/MXCSR; clearing here
   is part of libvpx's coexistence contract with hosting
   applications.
8. If the decoded frame is a "show" frame (`cm->show_frame`),
   increment `current_video_frame` and snapshot the mode-info grid
   into `show_frame_mi` (lines 346-349) so that
   `vp8dx_references_buffer` (below) can inspect what the just-shown
   frame referred to.
9. Under `CONFIG_ERROR_CONCEALMENT` (lines 351-368) swap the
   current and previous mode-info grids and propagate segment IDs
   forward into the new grid, so the next frame's EC has the prior
   frame's modes/MVs handy.
10. `pbi->ready_for_new_data = 0` — the application must now read
    out the frame via `vp8dx_get_raw_frame` before pushing more
    bytes.

Returns 0 on success, negative on hard decode error. Reached from
`vp8/vp8_dx_iface.c:519` inside `vp8_decode`.

Gotcha: the function performs `vpx_clear_system_state()` twice on the
success path (line 344 *and* via `decode_exit` line 373). This is
intentional defensive practice — the second call is cheap and
guarantees that any partial state inserted between line 344 and the
return is also flushed.

### `int vp8dx_get_raw_frame(VP8D_COMP *pbi, YV12_BUFFER_CONFIG *sd, vp8_ppflags_t *flags)` — hand the show frame to the caller

Defined at `vp8/decoder/onyxd_if.c:376-405`. The companion to
`vp8dx_receive_compressed_data`: copies the *descriptor* (not the
pixels) of `pbi->common.frame_to_show` into `*sd`. Two preconditions
gate the call:

* `ready_for_new_data` must be 0 — that is, the previous decode
  must have actually decoded *something*. Otherwise the function
  returns `-1` with no work done.
* `show_frame` must be 1 — the just-decoded frame must be marked for
  display. (VP8 supports non-shown frames, notably as
  ALTREF-construction frames; those return `-1` here.)

Once both hold, `ready_for_new_data` is flipped back to 1 so the next
`vp8dx_receive_compressed_data` call is allowed. The `#if
CONFIG_POSTPROC` branch (line 387) delegates to
`vp8_post_proc_frame`, which can substitute a postprocessed copy
governed by `flags`; in the decoder-only build path documented in
[vp8_files.md](../vp8_files.md), this branch is compiled out, `flags`
is cast to `void`, and the function does a shallow structure
assignment (`*sd = *pbi->common.frame_to_show`, line 393) followed by
a fix-up of the width/height fields. The shallow assignment is
*not* a pixel copy — `sd` ends up pointing at the same plane buffers
the decoder still owns. The caller (`vp8/vp8_dx_iface.c:549`) treats
the descriptor as borrowed for the duration of the
`vpx_codec_get_frame` iteration.

Width fix-ups (lines 394-396): `frame_to_show->y_width` etc. are
the *padded* dimensions (rounded up to a 16-pixel macroblock grid);
overwriting them here with the *visible* `common.Width`/`Height`
gives the application the cropped picture size.

Final `vpx_clear_system_state()` (line 403) — same hygiene rationale
as in `vp8dx_receive_compressed_data`.

### `int vp8dx_references_buffer(VP8_COMMON *oci, int ref_frame)` — has the last frame motion-compensated from `ref_frame`?

Defined at `vp8/decoder/onyxd_if.c:411-422`. Linear scan over the
mode-info grid (`oci->mi`) checking whether any macroblock's
`mbmi.ref_frame` equals the queried reference (LAST / GOLDEN /
ALTREF). Returns 1 on the first hit, 0 if no macroblock referenced
it. Driven by the public control `VP8D_GET_LAST_REF_USED` at
`vp8/vp8_dx_iface.c:682-684`, where it is invoked three times in a
row to build a bitmask reporting which references the just-decoded
frame actually depended on. This lets an RTC application drop the
bits for unused references.

The comment at lines 407-410 acknowledges that this function isn't
strictly decoder-specific — an encoder knows the reference usage
without scanning — but lives here because the decoder is the one
asked.

The trailing `mi++` on line 419 is a quirk of the mode-info grid
layout: there is one extra column past each row used as a sentinel
(see `VP8_COMMON::mode_info_stride` semantics in
`vp8/common/onyxc_int.h`), so the iterator increments past it
between rows.

### `int vp8_create_decoder_instances(struct frame_buffers *fb, VP8D_CONFIG *oxcf)` — instantiate the decoder fleet

Defined at `vp8/decoder/onyxd_if.c:424-444`. The single entry point
used by `vp8/vp8_dx_iface.c:372` to bring a decoder up. In the
single-threaded build it does exactly one thing: call
`create_decompressor(oxcf)` and store the result in `fb->pbi[0]`.
The `struct frame_buffers` declared in
`vp8/decoder/onyxd_int.h:50-57` carries an array of up to
`MAX_FB_MT_DEC == 32` decoder instances; in the documented build
only slot 0 is ever populated. Under `CONFIG_MULTITHREAD` (lines
429-442) it additionally arms a `setjmp` landing pad and calls
`vp8_decoder_create_threads` (`threading.c`) to spin up worker
threads.

Returns `VPX_CODEC_OK` (0) on success, `VPX_CODEC_ERROR` if
`create_decompressor` returns NULL.

### `int vp8_remove_decoder_instances(struct frame_buffers *fb)` — symmetric teardown

Defined at `vp8/decoder/onyxd_if.c:446-458`. Tears down what
`vp8_create_decoder_instances` set up: under MT, joins the worker
threads via `vp8_decoder_remove_threads`; in both modes, calls
`remove_decompressor` on `fb->pbi[0]` and NULLs the slot. Returns
`VPX_CODEC_ERROR` if `pbi[0]` was already NULL — a defensive check
that lets the iface layer call this function from arbitrary error
paths without tracking whether construction had completed.

### `int vp8dx_get_quantizer(const VP8D_COMP *pbi)` — base Q index of the last frame

Defined at `vp8/decoder/onyxd_if.c:460-462`. One-liner returning
`pbi->common.base_qindex`. The base Q index is the unsegmented
quantizer parsed from the frame header (RFC 6386 §9.6); 0 = highest
quality, 127 = lowest. Surfaced through the `VP8D_GET_LAST_QUANTIZER`
control at `vp8/vp8_dx_iface.c:631` for adaptive-bitrate clients
that want to react to source-side quantizer changes.

---

## How the rest of the decoder uses this file

Three classes of callers, all routed through `vp8/vp8_dx_iface.c`:

| Caller (iface layer)                  | Function in this file                  |
|---------------------------------------|----------------------------------------|
| `vp8_init` → `decoder_init`           | `vp8_create_decoder_instances`         |
| `vp8_destroy`                         | `vp8_remove_decoder_instances`         |
| `vp8_decode`                          | `vp8dx_receive_compressed_data`        |
| `vp8_get_frame` (the iterator)        | `vp8dx_get_raw_frame`                  |
| `vp8_set_reference` ctl               | `vp8dx_set_reference`                  |
| `vp8_get_reference` ctl               | `vp8dx_get_reference`                  |
| `vp8_get_quantizer` ctl               | `vp8dx_get_quantizer`                  |
| `vp8_get_last_ref_used` ctl           | `vp8dx_references_buffer`              |

Internally, the file's static helpers form a small closed system:
`get_free_fb` and `ref_cnt_fb` are the only refcount mutators (apart
from one explicit decrement in `swap_frame_buffers` line 265 and one
in the error path on line 326). Any future change to the DPB rules
should preserve the invariant stated in the
[overview](../vp8_technical_overview.md#10-decoded-picture-buffer--reference-management):
the sum of `fb_idx_ref_cnt[]` equals the number of live references
plus 1 for an in-flight `new_fb_idx`, and never exceeds
`NUM_YV12_BUFFERS`.
