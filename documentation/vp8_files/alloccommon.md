# `vp8/common/alloccommon.c` — lifecycle of the shared `VP8_COMMON` state

This 192-line file is the smallest piece of the VP8 decoder that you cannot
do without: it owns the construction, dimension-driven (re)allocation, and
teardown of every memory region attached to the per-instance
`VP8_COMMON` block. There are only five non-static functions, all listed
in `alloccommon.h:20-24`, and together they constitute the entire memory
contract between the codec's API layer (`vp8/vp8_dx_iface.c`) and the
frame-level driver (`vp8/decoder/decodeframe.c`).

## Role in the decoder

Section 3 of `vp8_technical_overview.md` introduces `VP8_COMMON`
(`onyxc_int.h:62`) as the per-instance, per-sequence container that
holds the reference picture buffers, the per-macroblock mode grid, the
above-row entropy context, the loop-filter parameters, and the two
`FRAME_CONTEXT` probability snapshots (`lfc`, `fc`). All of those
fields are *referenced* by many other compilation units, but only
`alloccommon.c` actually allocates and frees the ones that live on the
heap.

In the lifecycle diagram of section 2.3 of the overview, this file
implements the leaves marked `vp8_create_common`,
`vp8_alloc_frame_buffers`, and `vp8_remove_common`:

```
create_decompressor()  →  vp8_create_common(&pbi->common)         /* once */
vp8_decode() (1st call, or on dim change)
                       →  vp8_alloc_frame_buffers(&pc, W, H)       /* per-resize */
vp8_remove_decoder_instances() → vp8_remove_common(&pbi->common)   /* once */
```

Because VP8 carries the picture dimensions in the keyframe payload
rather than in any out-of-band configuration, allocation cannot happen
at `vpx_codec_dec_init_ver()` time. The decoder discovers `Width` and
`Height` from the 3-byte frame tag and the keyframe extension
(`decodeframe.c` parses them; see overview §6.1), then routes through
`vp8_dx_iface.c:433` to call `vp8_alloc_frame_buffers`. The single
choke-point design here matters: the API layer can resize the decoder
simply by re-invoking this one function — the helper itself begins by
calling its own inverse, so it is safe to call repeatedly without
leaking.

There is exactly one non-trivial invariant that runs through the whole
file: every allocation path either succeeds completely or rolls itself
back to the same state as `vp8_de_alloc_frame_buffers`. That symmetry
is what makes the resize/teardown logic in `vp8_dx_iface.c` and
`onyxd_if.c` short enough to inspect by eye.

## Headers and the dependency surface

The include block at `alloccommon.c:11-18` is itself worth a sentence
each, because it tells you exactly which subsystems this file reaches
into:

| Include                  | What it brings in                                                              |
|--------------------------|---------------------------------------------------------------------------------|
| `vpx_config.h`           | `CONFIG_POSTPROC`, `CONFIG_ERROR_CONCEALMENT` switches that gate optional fields. |
| `alloccommon.h`          | The five public prototypes this file implements.                                |
| `blockd.h`               | `MODE_INFO`, `ENTROPY_CONTEXT_PLANES`, `LOOPFILTERTYPE` enumerators.            |
| `vpx_mem/vpx_mem.h`      | `vpx_calloc`, `vpx_memalign`, `vpx_free` — the only allocator used here.        |
| `onyxc_int.h`            | `VP8_COMMON` itself, plus `NUM_YV12_BUFFERS` (4) and `MAX_PARTITIONS` (9).      |
| `findnearmv.h`           | Indirectly pulled in for transitive types; nothing from it is called here.      |
| `entropymode.h`          | `vp8_init_mbmode_probs`, `vp8_default_bmode_probs` (entropy-table initializers). |
| `systemdependent.h`      | `vp8_machine_specific_config` (RTCD entry point).                               |

`yv12config.h` is reached transitively (via `onyxc_int.h` →
`vpx_scale_rtcd.h` paths in practice); it supplies the
`VP8BORDERINPIXELS` macro (= 32, `vpx_scale/yv12config.h:23`) and the
`YV12_BUFFER_CONFIG` type, plus the two helpers
`vp8_yv12_alloc_frame_buffer` / `vp8_yv12_de_alloc_frame_buffer`
called below.

## `vp8_de_alloc_frame_buffers` — the dual of every allocation in the file

```c
void vp8_de_alloc_frame_buffers(VP8_COMMON *oci);     /* alloccommon.c:22 */
```

This is the canonical teardown routine for `VP8_COMMON`'s heap-resident
fields. It is written so that calling it on a freshly zeroed structure
is harmless: every `vpx_free` accepts `NULL`, and every
`vp8_yv12_de_alloc_frame_buffer` checks its argument internally before
touching anything.

It walks through, in order:

1. **The four-slot picture buffer pool** (`oci->yv12_fb[0..3]`,
   `alloccommon.c:24-27`). Each slot is released, and the matching
   reference-count cell `fb_idx_ref_cnt[i]` is forced back to `0`. The
   pool is the decoded-picture buffer (DPB) discussed in section 10 of
   the overview; the `fb_idx_ref_cnt[]` array is what lets the
   `frame_to_show` slot be held by the application *and* by the
   reference rotation simultaneously.

2. `temp_scale_frame` (`alloccommon.c:29`) — a scratch buffer used by
   the optional spatial resampler path. It is allocated unconditionally
   below (`alloccommon.c:86-89`) because the size of the allocation is
   trivially small; even on `--disable-spatial-resampling` builds it is
   created and destroyed but never used.

3. The `CONFIG_POSTPROC` block (`alloccommon.c:30-42`) — releases the
   post-processing frame buffer, the optional secondary post-proc
   buffer (and only if it was lazily promoted via
   `post_proc_buffer_int_used`), the SIMD-aligned `pp_limits_buffer`,
   and the `generated_noise` pool. The `memset(&oci->postproc_state, 0,
   …)` at line 41 is what guarantees that a subsequent
   `vp8_alloc_frame_buffers` followed by post-proc reuse starts from a
   reproducible state — without it, stale pointers in
   `postproc_state` would be dereferenced.

4. The above-row entropy context `above_context` (`alloccommon.c:44`).
   This is one `ENTROPY_CONTEXT_PLANES` per macroblock column
   (`blockd.h:51-56`); see overview §5 and §9 — it caches the
   coefficient-band entropy contexts produced by the macroblock above
   the current row so the bool decoder can pick the right probability
   table.

5. The mode-info grid `mip` (`alloccommon.c:45`). Because `mi`
   (`onyxc_int.h:121`) is just `mip + mode_info_stride + 1`, freeing
   the base also frees the visible window — but only `mip` is the real
   allocation. `mi`, `show_frame_mi`, and `frame_to_show` are
   *reset to NULL* below (`alloccommon.c:52-56`) so that subsequent
   code does not chase dangling pointers.

6. The `CONFIG_ERROR_CONCEALMENT`-gated `prev_mip` /
   `prev_mi` (`alloccommon.c:46-50`). In the minimal build targeted by
   the overview, those fields do not even exist on `VP8_COMMON` (see
   `onyxc_int.h:122-125`). Note that the matching *allocation* lives
   not here but in `vp8_dx_iface.c` — only the teardown is centralized.

**Why it is structured this way.** Every other function in this file
needs to be able to undo a partial allocation. By making the teardown
idempotent and NULL-safe, the allocator can simply `goto allocation_fail`
at any point and recover without intermediate book-keeping.

**Gotcha.** The function does *not* zero out scalar bookkeeping fields
like `mb_rows`, `mb_cols`, or `mode_info_stride`. They will retain the
values from the previous successful allocation. Callers that need a
truly fresh `VP8_COMMON` use `vp8_create_common` instead.

## `vp8_alloc_frame_buffers` — the (re)allocator keyed by frame dimensions

```c
int vp8_alloc_frame_buffers(VP8_COMMON *oci, int width, int height);  /* alloccommon.c:59 */
```

This is the only function in the file that takes nontrivial arguments,
and it is responsible for the entire heap footprint of `VP8_COMMON`.

It opens by calling `vp8_de_alloc_frame_buffers(oci)` unconditionally
(`alloccommon.c:62`). This is what makes it safe to call on resize —
the decoder simply *re-enters* this function with the new dimensions
and the old allocations evaporate first. The dual roles "first
allocation" and "resize" are therefore one code path.

Next, the dimensions are rounded up to the next multiple of 16
(`alloccommon.c:64-67`). VP8's coding unit is the 16×16 macroblock; the
internal buffers must hold an integer number of MBs even when the
display width/height is not aligned. The encoder mirrors this on the
write side.

```c
if ((width  & 0xf) != 0) width  += 16 - (width  & 0xf);
if ((height & 0xf) != 0) height += 16 - (height & 0xf);
```

Then the four reference-pool slots are allocated
(`alloccommon.c:69-74`):

```c
for (i = 0; i < NUM_YV12_BUFFERS; ++i) {
  if (vp8_yv12_alloc_frame_buffer(&oci->yv12_fb[i], width, height,
                                  VP8BORDERINPIXELS) < 0)
    goto allocation_fail;
}
```

`VP8BORDERINPIXELS` (= 32) is the per-side guard band that
`vp8_yv12_alloc_frame_buffer` reserves around every plane. The guard
band is what lets motion-compensated inter prediction read past the
visible frame edge without bounds checks (see overview §8).

The four DPB slot indices are then assigned the *initial* identity
permutation (`alloccommon.c:76-79`):

```c
oci->new_fb_idx = 0;     /* slot that the next frame will be written into */
oci->lst_fb_idx = 1;     /* "LAST" reference                              */
oci->gld_fb_idx = 2;     /* "GOLDEN" reference                            */
oci->alt_fb_idx = 3;     /* "ALTREF" reference                            */
```

and all four `fb_idx_ref_cnt[]` entries are forced to `1`
(`alloccommon.c:81-84`). The non-zero reference counts here matter:
they tell the swap logic in `swapyv12buffer.c` that every slot is
"live" from the start, so the very first frame's reference rotation
(see overview §10) does not free a buffer that decoding still depends
on.

`temp_scale_frame` is allocated with `height = 16`
(`alloccommon.c:86-89`) — only a single row of macroblocks, because
the spatial resampler operates a strip at a time when it is enabled.

Then the mode-info grid (`alloccommon.c:91-100`):

```c
oci->mb_rows = height >> 4;
oci->mb_cols = width  >> 4;
oci->MBs = oci->mb_rows * oci->mb_cols;
oci->mode_info_stride = oci->mb_cols + 1;
oci->mip = vpx_calloc((oci->mb_cols + 1) * (oci->mb_rows + 1),
                      sizeof(MODE_INFO));
if (!oci->mip) goto allocation_fail;
oci->mi = oci->mip + oci->mode_info_stride + 1;
```

The allocation is `(mb_cols+1) × (mb_rows+1)` and `mi` is offset by
`(stride+1)` to point at the *upper-left visible* macroblock. The
extra row at the top and the extra column on the left are sentinel
slots: they let neighbor lookups like `mi[-1]`, `mi[-stride]`, and
`mi[-stride-1]` (used everywhere in intra prediction and entropy
context computation) read without a bounds check. Overview §3.2
expands on this layout. The `vpx_calloc` zero-fills the sentinels,
which is exactly what the neighbor logic expects for absent
macroblocks.

The comment at lines 102-103 — "Allocation of previous mode info will
be done in `vp8_decode_frame()` as it is a decoder only data" — is the
explanation for why this file does not allocate `prev_mip` even when
`CONFIG_ERROR_CONCEALMENT` is on: that field is conceptually part of
the decoder, not of the shared common state, and its allocation is
deferred to a context where the EC subsystem can choose whether to do
it lazily.

The above-row entropy context (`alloccommon.c:105-108`) is allocated
as one `ENTROPY_CONTEXT_PLANES` per column. Note the bytes-per-element
argument swap in `vpx_calloc(size, 1)` — the file consistently uses
this form to mean "allocate `size` bytes, zeroed".

The `CONFIG_POSTPROC` block (`alloccommon.c:110-125`) creates a
fifth picture-sized buffer for the post-processor and a SIMD-aligned
`pp_limits_buffer`. The width formula `24 * ((mb_cols + 1) & ~1)` is
worth reading carefully: the per-column post-proc state takes 24
bytes, `mb_cols + 1` rounds the column count up to include the right
sentinel column, and `& ~1` aligns it to an even number so that
SSE2/AVX loads on the post-proc fast paths can read the next 16-byte
chunk safely. The buffer is `memalign(16, …)` rather than
`vpx_calloc` precisely for that reason.

Finally, the rollback label (`alloccommon.c:129-131`):

```c
allocation_fail:
  vp8_de_alloc_frame_buffers(oci);
  return 1;
```

Returning `1` on failure (and `0` on success) is the convention the
API layer relies on; `vp8_dx_iface.c:433` propagates the non-zero
return as `VPX_CODEC_MEM_ERROR`. Because the rollback re-runs the
full teardown, the caller is guaranteed that `VP8_COMMON` ends up in
the same NULL-pointer state regardless of which allocation actually
failed — no half-initialized state ever survives.

**Gotchas.**

* The function is *not* reentrant and *not* thread-safe; the API layer
  serializes around it. With `CONFIG_MULTITHREAD`, callers must ensure
  the worker pool is quiesced before invoking.

* `mb_rows`, `mb_cols`, `MBs`, `mode_info_stride` get updated to match
  the rounded-up dimensions — *not* the visible `Width`/`Height`. The
  caller (`vp8_dx_iface.c`) holds the visible dimensions separately.

* The function silently rounds the input width and height up; it does
  not refuse degenerate sizes. Bounds checking happens earlier in
  `vp8_peek_si_internal`.

## `vp8_setup_version` — bitstream-version → profile-bit decoder

```c
void vp8_setup_version(VP8_COMMON *cm);   /* alloccommon.c:134 */
```

The 3-bit `version` field in the VP8 frame tag (RFC 6386 §9.1; see
overview §6.1) is *not* a forward-compatibility version number in the
usual sense — it is a packed selector for three independent profile
bits: deblock-on/off, loop-filter type (normal vs simple), and
motion-compensation filter (6-tap vs bilinear). This helper decodes
the four officially defined values into the four scalar fields on
`VP8_COMMON`:

| `version` | `no_lpf` | `filter_type`       | `use_bilinear_mc_filter` | `full_pixel` |
|-----------|----------|---------------------|--------------------------|--------------|
| 0         | 0        | `NORMAL_LOOPFILTER` | 0                        | 0            |
| 1         | 0        | `SIMPLE_LOOPFILTER` | 1                        | 0            |
| 2         | 1        | `NORMAL_LOOPFILTER` | 1                        | 0            |
| 3         | 1        | `SIMPLE_LOOPFILTER` | 1                        | 1            |
| 4-7       | (same as version 0)                                              |

The `default:` branch (`alloccommon.c:160-167`) is conservative — it
falls back to "version 0" behaviour for the reserved values 4..7 rather
than erroring out, matching the RFC's recommendation. `full_pixel = 1`
in version 3 means motion vectors are restricted to integer pixel
positions; the sub-pel filter pipeline in `vp8/common/filter.c` short-
circuits accordingly.

Called from `vp8_decode_frame` (`decodeframe.c:935`) immediately after
the `version` field has been parsed; the values it writes are then read
on every macroblock by the loop filter (`vp8_loopfilter.c`), the
motion-compensation dispatcher (`reconinter.c`), and the sub-pel filter
tables (`filter.c`).

**Invariant.** `cm->version` itself is left untouched — only the four
derived flags are written. This lets later code that wants to inspect
the raw value (e.g. some test harnesses) still find it.

## `vp8_create_common` — one-time per-instance initialization

```c
void vp8_create_common(VP8_COMMON *oci);   /* alloccommon.c:169 */
```

Called exactly once per decoder instance from `create_decompressor`
(`onyxd_if.c:81`), *before* any frame has been seen. It is the
mirror image of `vp8_remove_common` and is responsible for two
classes of setup that have nothing to do with frame-dimension-sized
heap regions:

1. **RTCD initialization** via `vp8_machine_specific_config(oci)`
   (`alloccommon.c:170`). This populates the function-pointer
   tables that the runtime-CPU-dispatch layer routes through (see
   overview §14). On the generic-C build it is essentially a no-op
   set of identity assignments; on x86 / ARM it probes CPU
   features and selects SIMD kernels.

2. **Default probability tables** — `vp8_init_mbmode_probs(oci)`
   (entropymode.c) seeds `oci->fc.ymode_prob`, `oci->fc.uv_mode_prob`,
   and `oci->fc.sub_mv_ref_prob` with the RFC 6386 §11.2/§16.2
   default values; `vp8_default_bmode_probs(oci->fc.bmode_prob)`
   does the same for the 4×4 sub-mode tree. These defaults are what
   keyframes start from before any per-frame probability updates are
   applied.

It then writes the **profile defaults** that a freshly created decoder
expects to see before the first frame tag is parsed
(`alloccommon.c:175-181`):

```c
oci->mb_no_coeff_skip      = 1;
oci->no_lpf                = 0;
oci->filter_type           = NORMAL_LOOPFILTER;
oci->use_bilinear_mc_filter = 0;
oci->full_pixel            = 0;
oci->multi_token_partition = ONE_PARTITION;
oci->clamp_type            = RECON_CLAMP_REQUIRED;
```

These are identical to the values that `vp8_setup_version` would write
for `version = 0`, which is the correct "before-we-know-anything"
initial state. `multi_token_partition = ONE_PARTITION` (= 0; see
`onyxc_int.h:51`) declares that until proven otherwise the bitstream
has a single token partition; this becomes important on the very first
frame, where the partition count is read from the compressed header.
`clamp_type = RECON_CLAMP_REQUIRED` mandates clipping of reconstructed
samples to `[0, 255]` — the safe default.

`memset(oci->ref_frame_sign_bias, 0, …)` (`alloccommon.c:184`) zeroes
the three-element MV sign-bias array (LAST/GOLDEN/ALTREF). These flags
control sign reversal of MV predictors when the reference frame's
canonical time direction differs; until any frame is decoded they are
all "forward".

The last two writes (`alloccommon.c:187-188`) disable the
buffer-to-buffer copies that VP8 supports (copy LAST → GOLDEN, or ARF
→ GOLDEN, etc.). The decoder will re-enable them frame-by-frame from
the compressed header.

**Invariant.** This function touches *only* fields that are scalars or
small inline arrays — it allocates nothing on the heap. The heap-side
counterpart is `vp8_alloc_frame_buffers`. Splitting the two means a
decoder instance can exist before its dimensions are known.

**How the rest of the decoder uses it.** Exactly one call site, in
`onyxd_if.c:81`, inside `create_decompressor`. No frame-decode path
calls it.

## `vp8_remove_common` — instance teardown

```c
void vp8_remove_common(VP8_COMMON *oci);    /* alloccommon.c:191 */
```

A one-liner: it forwards to `vp8_de_alloc_frame_buffers`. The
asymmetry with `vp8_create_common` (which does much more than just
zeroing) is deliberate: the fields that `vp8_create_common` writes are
either inline storage or function-pointer tables that need no cleanup,
so the only thing teardown actually has to do is release the heap.

Called from `remove_decompressor` (`onyxd_if.c:62`) during
`vpx_codec_destroy()`.

**Gotcha.** This function does not free `VP8_COMMON` itself. The
struct is embedded inside `VP8D_COMP` (`onyxd_int.h:59`), which is
freed one level up by `vp8_remove_decoder_instances`. The split lets
the API layer reuse the same `VP8_COMMON` storage if it ever needed
to reset the decoder without destroying it — currently no caller does
this, but the abstraction is preserved.

---

That is the complete contract of `alloccommon.c`: a single allocator
keyed by frame dimensions, its NULL-safe inverse, a tiny
profile-bit decoder, and a paired pre-allocation / post-deallocation
initializer for the per-instance state that does not depend on
dimensions. Every other compilation unit in the decoder treats these
five entry points as the only legitimate way to mutate the lifetime of
`VP8_COMMON`'s heap regions.
