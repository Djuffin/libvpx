# `vp8/common/vp8_loopfilter.c` — the loop-filter driver

This file is the top-level driver of VP8's in-loop deblocking filter.
It does not, itself, touch a single pixel. Its job, instead, is the
bookkeeping that every pixel-touching kernel will depend on: turning
the small bundle of bitstream-controlled parameters (the frame's
`filter_level`, `sharpness_level`, `filter_type`, per-segment
`segment_feature_data`, per-ref-frame `ref_lf_deltas`, per-mode
`mode_lf_deltas`) into the dense lookup tables `loop_filter_info_n`,
then walking the frame in raster order, picking the per-macroblock
filter strength out of those tables, and dispatching one of two
families of edge kernels — *normal* or *simple* — at every
macroblock boundary and every sub-block boundary that the bitstream
permits.

The grand structure of the file therefore breaks into three layers:

  1. **Per-build initialization** — `vp8_loop_filter_init`, called
     once when a `VP8_COMMON` is allocated; pre-computes the parts
     of `loop_filter_info_n` that depend on nothing but the sharpness
     level and a handful of compile-time constants.

  2. **Per-frame initialization** — `vp8_loop_filter_frame_init`,
     called at the top of every decoded frame; combines the frame's
     `default_filt_lvl` (also called `cm->filter_level`) with the
     bitstream's segment / ref / mode deltas and clamps the result
     into a single 4×4×4 array `lfi->lvl[seg][ref][mode]`, indexed
     in the inner loop by three fields of `MB_MODE_INFO`.

  3. **Per-frame application** — `vp8_loop_filter_frame` and its
     two row-granular siblings (`vp8_loop_filter_row_normal`,
     `vp8_loop_filter_row_simple`, used by the threaded build) plus
     two specialty variants (`vp8_loop_filter_frame_yonly`,
     `vp8_loop_filter_partial_frame`). Each walks the chosen MB
     range, looks the filter strength up in the prepared table, and
     calls into the `vp8_loop_filter_mbv` / `_bv` / `_mbh` / `_bh`
     edge kernels declared in `vp8_rtcd.h` and implemented (for the
     C reference path) in `loopfilter_filters.c`.

The file is the libvpx realization of **RFC 6386 §15** ("Loop
Filter"). The numeric formulae in `vp8_loop_filter_update_sharpness`,
the per-mode/per-ref delta logic in `vp8_loop_filter_frame_init`, and
the four-edge raster walk in `vp8_loop_filter_frame` correspond
one-for-one to §15.2–§15.5 of that document. The technical-overview
notes the same mapping in its bitstream-correspondence table
(`documentation/vp8_technical_overview.md:2025–2026`):

```
| §15. Loop filter                                 | vp8_loopfilter.c | 263–382 |
| §15.4 LF control-parameter derivation            | vp8_loopfilter.c |  49–75  |
```

## Role in the decoder

VP8 is a block-based codec: residuals are coded on 4×4 blocks and
predictions on 4×4 or 16×16 blocks. The block boundaries are
visible to the eye as "blocking" artifacts, and the loop filter
exists to soften those boundaries before the reconstructed frame is
either displayed or used as a reference for later inter-prediction.
It is *in-loop* — i.e. the filtered samples are what later frames
will motion-compensate from — so the filter is not optional: every
decoder must apply it bit-exactly the same way the encoder did, or
drift accumulates.

The filter is applied once per frame, late, after every macroblock
of the frame has been reconstructed (intra-predicted or
inter-predicted, residual added, clipped). Concretely, the call site
is `decodeframe.c`'s `decode_mb_rows`, which invokes
`vp8_loop_filter_frame(cm, &mb, cm->frame_type)` immediately after
the last MB row has been reconstructed and before the frame buffer
is swapped into the reference-buffer ring. The driver here is what
that call enters.

The pixel-level kernels live in a sibling translation unit,
`vp8/common/loopfilter_filters.c`. The driver communicates with them
exclusively through the function-pointer table built by `vp8_rtcd.h`
(`vp8_loop_filter_mbv`, `_bv`, `_mbh`, `_bh` for the normal
filter; `vp8_loop_filter_simple_mbv`, `_bv`, `_mbh`, `_bh` for the
simple filter — see `vp8/common/rtcd_defs.pl:58–109`). Each of
those table slots is filled by `rtcd.c::vp8_rtcd()` at startup with
the best implementation the host CPU supports (NEON, SSE2, MSA,
MMI, LSX, DSPR2, or the C reference). The driver is wholly
ignorant of which one will fire.

Two parameters from the frame header are decisive for the driver
and worth introducing before the code:

  * `filter_type` (LOOPFILTERTYPE, two values: `NORMAL_LOOPFILTER=0`,
    `SIMPLE_LOOPFILTER=1`; declared at `loopfilter.h:27`). Selects
    one of two filter families. The simple filter only touches luma
    and uses a 4-tap kernel, so it is faster but lower quality; the
    normal filter touches Y, U, V, uses a longer kernel with a
    high-edge-variance test, and is the default.

  * `filter_level` (range 0..63, the "global" filter strength
    for the frame; per-MB strength is `filter_level` adjusted by
    segment / ref / mode deltas and re-clamped to 0..63). If after
    adjustment the per-MB strength is zero, that MB is not filtered
    at all.

Every function below is structured around these two switches.

## Header dependencies

```c
#include "vpx_config.h"
#include "vp8_rtcd.h"
#include "loopfilter.h"
#include "onyxc_int.h"
#include "vpx_mem/vpx_mem.h"
```

Only five. `vpx_config.h` is the `./configure` output and gives
`VPX_ARCH_ARM` (used by `loopfilter.h` to pick `SIMD_WIDTH = 1` on
ARM, 16 elsewhere — see below). `vp8_rtcd.h` is the generated
function-pointer table for the edge kernels. `loopfilter.h` brings
in `loop_filter_info_n` (the dense LUT struct), `loop_filter_info`
(the four-pointer "tear-off" actually passed to the normal kernels),
`MAX_LOOP_FILTER = 63`, `SIMD_WIDTH`, and the two enumeration
values of `LOOPFILTERTYPE`. `onyxc_int.h` brings in `VP8_COMMON`,
which holds the LUT (`cm->lf_info`), the bitstream-controlled fields
(`cm->filter_level`, `cm->filter_type`, `cm->sharpness_level`,
`cm->last_sharpness_level`), and the frame buffer pointer
(`cm->frame_to_show`). `vpx_mem.h` is included for completeness;
the only memory helper actually used in the file is the libc
`memset` referenced through `<string.h>` (transitively pulled in).

## The data structure being filled in: `loop_filter_info_n`

Every function in this file either populates or reads
`loop_filter_info_n`, which is defined at `loopfilter.h:38–49`:

```c
typedef struct {
  DECLARE_ALIGNED(SIMD_WIDTH, unsigned char,
                  mblim[MAX_LOOP_FILTER + 1][SIMD_WIDTH]);
  DECLARE_ALIGNED(SIMD_WIDTH, unsigned char,
                  blim [MAX_LOOP_FILTER + 1][SIMD_WIDTH]);
  DECLARE_ALIGNED(SIMD_WIDTH, unsigned char,
                  lim  [MAX_LOOP_FILTER + 1][SIMD_WIDTH]);
  DECLARE_ALIGNED(SIMD_WIDTH, unsigned char, hev_thr[4][SIMD_WIDTH]);
  unsigned char lvl[4][4][4];
  unsigned char hev_thr_lut[2][MAX_LOOP_FILTER + 1];
  unsigned char mode_lf_lut[10];
} loop_filter_info_n;
```

A reader who internalizes these seven fields can essentially read the
rest of the file as glue. Their roles:

  * `mblim[lvl][...]` — the MB-edge "limit" byte for each global
    `filter_level` in 0..63. Each entry is replicated `SIMD_WIDTH`
    times so that the SIMD kernels can load one aligned vector and
    use it without broadcasting. `mblim` is consulted at the
    macroblock-boundary edges only (16 pixels wide on luma, 8 on
    chroma); compared to `blim` it is more permissive (the boundary
    can be flatter and the filter still fires).

  * `blim[lvl][...]` — same shape, but for the *sub-block*
    interior edges (three vertical and three horizontal per MB).

  * `lim[lvl][...]`  — the *inner* limit used in the
    `|p_n - p_{n-1}| < lim` per-sample masking test that both
    kernels (MB and sub-block edges) consult to disable filtering at
    samples that look like genuine image edges rather than blocking
    artifacts.

  * `hev_thr[i][...]` — four threshold vectors (`i = 0..3`)
    containing the broadcast constant `i`. Used as the "high edge
    variance" threshold in the normal filter only: when a sample's
    local variance is above `hev_thr`, the kernel switches from a
    long filter to a short filter to avoid blurring real edges.

  * `lvl[seg][ref][mode]` — the *per-MB* filter strength. Indexed
    by `segment_id` (0..3), `ref_frame` (0=INTRA, 1=LAST,
    2=GOLDEN, 3=ALT), and a small mode bucket (0..3, picked from
    `mode_lf_lut` below).

  * `hev_thr_lut[frame_type][filter_level]` — picks one of the four
    `hev_thr` rows as a function of (a) whether this is a key frame
    or an inter frame and (b) the magnitude of the per-MB filter
    level. Built once by `lf_init_lut`.

  * `mode_lf_lut[mode]` — coalesces the ten possible
    `MB_PREDICTION_MODE` values (the four intra modes `DC_PRED`,
    `V_PRED`, `H_PRED`, `TM_PRED`, the intra `B_PRED`, and the
    five inter modes `ZEROMV`, `NEARESTMV`, `NEARMV`, `NEWMV`,
    `SPLITMV`) into one of four indices used to look up `lvl`.
    Built once by `lf_init_lut`.

The alignment annotation `DECLARE_ALIGNED(SIMD_WIDTH, ...)` exists
because `mblim`, `blim`, `lim`, `hev_thr` are loaded directly as
SIMD vectors by the assembly kernels (16-byte aligned loads on x86,
NEON-friendly on ARM). On ARM the compiled code is scalar — see
`loopfilter.h:29–33` — so `SIMD_WIDTH` collapses to 1 and the LUT
becomes a single byte per level. On every other architecture
`SIMD_WIDTH = 16` and each entry is a broadcast 16-byte vector.
This is the cleanest example in libvpx of "store the constant in
the form the kernel wants it" — the driver pays once at frame
boundaries, the kernels never pay.

`lvl[4][4][4]` is *not* aligned. It is read scalar-only, in the
inner loop of the four frame walkers below.

## The driver-side tear-off: `loop_filter_info`

The normal kernels (`vp8_loop_filter_mbv` etc.) do not take
`loop_filter_info_n*` — they take a four-pointer struct
`loop_filter_info`, declared at `loopfilter.h:51–56`:

```c
typedef struct loop_filter_info {
  const unsigned char *mblim;
  const unsigned char *blim;
  const unsigned char *lim;
  const unsigned char *hev_thr;
} loop_filter_info;
```

This is a tear-off built per-MB inside the walker: each pointer is
set to the row of the corresponding `loop_filter_info_n` member
that matches the current MB's filter level (and `hev_thr` is
chosen via the `hev_thr_lut`). The frame walkers below all build
the same four-line tear-off. The simple kernels do not need a
struct at all — they take just `mblim` or `blim` directly.

---

## `lf_init_lut` — one-time mode and hev-threshold lookup tables

```c
static void lf_init_lut(loop_filter_info_n *lfi) {
  int filt_lvl;

  for (filt_lvl = 0; filt_lvl <= MAX_LOOP_FILTER; ++filt_lvl) {
    if (filt_lvl >= 40) {
      lfi->hev_thr_lut[KEY_FRAME][filt_lvl] = 2;
      lfi->hev_thr_lut[INTER_FRAME][filt_lvl] = 3;
    } else if (filt_lvl >= 20) {
      lfi->hev_thr_lut[KEY_FRAME][filt_lvl] = 1;
      lfi->hev_thr_lut[INTER_FRAME][filt_lvl] = 2;
    } else if (filt_lvl >= 15) {
      lfi->hev_thr_lut[KEY_FRAME][filt_lvl] = 1;
      lfi->hev_thr_lut[INTER_FRAME][filt_lvl] = 1;
    } else {
      lfi->hev_thr_lut[KEY_FRAME][filt_lvl] = 0;
      lfi->hev_thr_lut[INTER_FRAME][filt_lvl] = 0;
    }
  }
  ...
```

What this routine builds is two small tables that the per-MB inner
loop will need to consult: `hev_thr_lut` and `mode_lf_lut`. Neither
table depends on anything that changes from frame to frame (or even
from session to session) — they are pure constants — so they are
built once, at `VP8_COMMON` allocation time, by `vp8_loop_filter_init`
calling this routine.

*Why* the staircase shape on `filt_lvl`: the high-edge-variance
threshold controls the trade-off between *filter strength* and
*edge preservation*. When `filter_level` is small (the encoder has
asked for gentle filtering), the codec also wants to preserve
real edges, so `hev_thr` is 0 — even a tiny luminance step is
considered an edge and shortens the kernel. As `filter_level`
grows the threshold raises in three steps. The two columns
(KEY_FRAME vs. INTER_FRAME) differ because key frames don't have a
predictor with blocking artifacts of its own, so they tolerate
slightly less filtering. The thresholds (15, 20, 40) and the
values (0,1,2,3) are exactly the constants given in RFC 6386
§15.2.

The second half of the routine builds `mode_lf_lut`:

```c
  lfi->mode_lf_lut[DC_PRED] = 1;
  lfi->mode_lf_lut[V_PRED]  = 1;
  lfi->mode_lf_lut[H_PRED]  = 1;
  lfi->mode_lf_lut[TM_PRED] = 1;
  lfi->mode_lf_lut[B_PRED]  = 0;

  lfi->mode_lf_lut[ZEROMV]    = 1;
  lfi->mode_lf_lut[NEARESTMV] = 2;
  lfi->mode_lf_lut[NEARMV]    = 2;
  lfi->mode_lf_lut[NEWMV]     = 2;
  lfi->mode_lf_lut[SPLITMV]   = 3;
}
```

This collapses the ten `MB_PREDICTION_MODE` values into four
buckets, which form the third index into `lvl[seg][ref][mode]`.
The buckets correspond to RFC 6386's notion of how "split-y" a
macroblock is:

  * **0 = `B_PRED`**     — intra split mode (16 separate 4×4
    intra predictions). The filter must treat sub-block edges
    aggressively because each predictor is independent.
  * **1 = `DC/V/H/TM_PRED`, `ZEROMV`** — flat 16×16 prediction,
    no internal sub-block discontinuities. Gentler filtering at
    sub-block edges is appropriate, but MB edges still need it.
  * **2 = `NEARESTMV`, `NEARMV`, `NEWMV`** — inter, single 16×16
    motion vector but non-zero. Slightly stronger.
  * **3 = `SPLITMV`**    — inter split, up to 16 independent MVs.
    Most aggressive, like `B_PRED` but for inter.

The three buckets at indices 1, 2, 3 will receive an additional
**mode delta** in `vp8_loop_filter_frame_init` (see below); the
filter strength they end up with reflects both the bucket index
and the bitstream's `mode_lf_deltas[mode]` adjustment.

**Invariants.** Both tables are filled exhaustively (the `for` loop
covers every legal `filt_lvl` 0..63; the named-constant initializers
cover every legal `MB_PREDICTION_MODE`). Nothing else in the file
writes to `hev_thr_lut` or `mode_lf_lut` — they are read-only after
init.

---

## `vp8_loop_filter_update_sharpness` — derivation of `lim`, `blim`, `mblim`

```c
void vp8_loop_filter_update_sharpness(loop_filter_info_n *lfi,
                                      int sharpness_lvl) {
  int i;

  for (i = 0; i <= MAX_LOOP_FILTER; ++i) {
    int filt_lvl = i;
    int block_inside_limit = 0;

    block_inside_limit = filt_lvl >> (sharpness_lvl > 0);
    block_inside_limit = block_inside_limit >> (sharpness_lvl > 4);

    if (sharpness_lvl > 0) {
      if (block_inside_limit > (9 - sharpness_lvl)) {
        block_inside_limit = (9 - sharpness_lvl);
      }
    }

    if (block_inside_limit < 1) block_inside_limit = 1;

    memset(lfi->lim  [i], block_inside_limit,                       SIMD_WIDTH);
    memset(lfi->blim [i], (2 * filt_lvl       + block_inside_limit), SIMD_WIDTH);
    memset(lfi->mblim[i], (2 * (filt_lvl + 2) + block_inside_limit), SIMD_WIDTH);
  }
}
```

This is the encoded form of RFC 6386 §15.4 "Loop Filter
Control Parameter Derivation." Its sole job is, for *every* possible
`filter_level` 0..63, to compute the three byte constants
(`lim`, `blim`, `mblim`) the kernels need, and to fan each out
into a `SIMD_WIDTH`-byte vector so SIMD can load it aligned.

Reading it bottom-up: the kernel's per-sample test is something like
"this 8-pixel column straddles an edge that is at most `mblim`
wide overall, with neighbouring pixels at most `lim` apart". Hence
the natural relationship:

    mblim = 2*(filter_level + 2) + block_inside_limit   (MB edge)
    blim  = 2* filter_level      + block_inside_limit   (sub-block edge)
    lim   = block_inside_limit                          (per-sample)

The MB-edge limit is offset by +2 (becomes +4 after the doubling) to
allow the filter to fire across slightly wider apparent
discontinuities at MB seams, where blocking is worst.

The interesting machinery is `block_inside_limit`. Its derivation:

    block_inside_limit = filt_lvl
                        >> (sharpness_lvl > 0)   /* halve if sharpness>0 */
                        >> (sharpness_lvl > 4);  /* halve again if >4   */
    if (sharpness_lvl > 0)
        clamp block_inside_limit to (9 - sharpness_lvl)
    if (block_inside_limit < 1)
        block_inside_limit = 1;

`sharpness_lvl` (0..7, read from the uncompressed header — see
`decodeframe.c` for the bit layout) is the encoder's *preserve
edges* knob:

  * `sharpness = 0` — no division, no clamp. `lim` grows linearly
    with `filter_level`. The filter is permissive (will fire on
    relatively bumpy edges).
  * `sharpness > 0` — divide once. Filter becomes less permissive
    (only smoother edges qualify).
  * `sharpness > 4` — divide twice. Even more conservative.
  * Always clamped to at least 1 so that the per-sample test never
    becomes degenerate.
  * Capped at `9 - sharpness_lvl` whenever `sharpness > 0` so that
    the maximum aggressiveness is bounded; `lim` will never exceed
    9, 8, 7, 6, 5, 4, 3, 2 for sharpness levels 1..7 respectively.

**Why call this out as a separate function.** The sharpness level
*does* change at frame boundaries (it's in the frame header, and
real encoders adjust it across scene changes). But it changes
**rarely**: keeping `last_sharpness_level` on `VP8_COMMON` lets
`vp8_loop_filter_frame_init` skip this 64-iteration rebuild
whenever the value is unchanged from the previous frame. This is
small but real — it would otherwise be 64 iterations × 3 memsets
of 16 bytes apiece on every frame.

**Invariants.** After this routine returns, for every `i` in
0..MAX_LOOP_FILTER:

  * `lfi->lim[i][k]   == block_inside_limit_for(i, sharpness)`
    for k in 0..SIMD_WIDTH-1 (broadcast).
  * `lfi->blim[i][k]  == 2*i + above`.
  * `lfi->mblim[i][k] == 2*(i+2) + above`.

These satisfy `mblim > blim >= 2*i + 1` for all `i >= 0`, so
the more-permissive ordering needed by the kernels is preserved
by construction.

---

## `vp8_loop_filter_init` — one-time setup at codec creation

```c
void vp8_loop_filter_init(VP8_COMMON *cm) {
  loop_filter_info_n *lfi = &cm->lf_info;
  int i;

  vp8_loop_filter_update_sharpness(lfi, cm->sharpness_level);
  cm->last_sharpness_level = cm->sharpness_level;

  lf_init_lut(lfi);

  for (i = 0; i < 4; ++i) {
    memset(lfi->hev_thr[i], i, SIMD_WIDTH);
  }
}
```

This is the entry point called when a `VP8_COMMON` is allocated
(from `alloccommon.c::vp8_create_common`). It performs the
*build-once* work:

  1. Populate the level-dependent tables for the initial sharpness
     setting. (`cm->sharpness_level` is zero at allocation, but the
     code is symmetric and handles whatever the field happens to be
     when called.)

  2. Initialize `last_sharpness_level` to match, so that the next
     `vp8_loop_filter_frame_init` will *not* redo the
     update-sharpness work unless the bitstream changes the
     sharpness value.

  3. Build the constant tables `hev_thr_lut` and `mode_lf_lut`.

  4. Broadcast the four `hev_thr` rows: row `i` is the byte `i`
     replicated `SIMD_WIDTH` times. The kernels will be passed
     `lfi->hev_thr[i]` for whatever `i` the `hev_thr_lut` returned
     for the current per-MB filter level + frame type, and they'll
     load it as a constant vector.

The function returns void and is, per construction, infallible — no
mallocs, no I/O.

---

## `vp8_loop_filter_frame_init` — per-frame derivation of `lvl[seg][ref][mode]`

```c
void vp8_loop_filter_frame_init(VP8_COMMON *cm, MACROBLOCKD *mbd,
                                int default_filt_lvl) {
  int seg, ref, mode;
  loop_filter_info_n *lfi = &cm->lf_info;

  if (cm->last_sharpness_level != cm->sharpness_level) {
    vp8_loop_filter_update_sharpness(lfi, cm->sharpness_level);
    cm->last_sharpness_level = cm->sharpness_level;
  }
  ...
```

Called once per frame, before walking macroblocks. Two parts.

### Part 1 — refresh sharpness if it changed

A trivial change-detector. The cost of `update_sharpness` is small
but `lvl[][][]` computation below is run unconditionally for the
whole 4×4×4 table, so the sharpness check is the only place where a
true optimization happens. Note that the *default* filter level
(`default_filt_lvl`, which is `cm->filter_level` in normal calls)
is *not* cached — it changes per frame and is consumed directly in
Part 2.

### Part 2 — derive `lvl[seg][ref][mode]`

For each of the four possible **segments** (0..3) the function
computes one byte `lvl[seg][ref][mode]` for every combination of
**ref-frame** (0=INTRA, 1=LAST, 2=GOLDEN, 3=ALT) and **mode bucket**
(0..3, the buckets defined by `mode_lf_lut`). That's at most 64
bytes per frame — trivial to recompute.

The derivation has three nested stages.

**Stage A: per-segment base level.** A copy of the frame default:

```c
    int lvl_seg = default_filt_lvl;
    ...
    if (mbd->segmentation_enabled) {
      if (mbd->mb_segment_abs_delta == SEGMENT_ABSDATA) {
        lvl_seg = mbd->segment_feature_data[MB_LVL_ALT_LF][seg];
      } else { /* Delta Value */
        lvl_seg += mbd->segment_feature_data[MB_LVL_ALT_LF][seg];
      }
      lvl_seg = (lvl_seg > 0) ? ((lvl_seg > 63) ? 63 : lvl_seg) : 0;
    }
```

VP8 supports up to four segments per frame (`MAX_MB_SEGMENTS = 4`,
`blockd.h:32`); each segment can carry its own alternate filter
level. The bitstream (`decodemv.c::read_mb_features`) tags each MB
with a `segment_id`, and the frame header carries the four-element
`segment_feature_data[MB_LVL_ALT_LF][0..3]`. The `mb_lf_adjust`
mechanism — controlled by `mbd->mb_segment_abs_delta` — has two
modes:

  * `SEGMENT_ABSDATA` (=1) — the per-segment value replaces the
    frame default outright.
  * delta (=0)             — the per-segment value is *added* to
    the frame default.

The result is clamped to [0, 63] (the legal `filter_level`
range, equal to `MAX_LOOP_FILTER`).

If segmentation is *not* enabled, all four segments use the frame
default, and the per-segment ref/mode derivation below is run with
that same `lvl_seg`.

**Stage B: short-circuit when ref/mode deltas are disabled.**

```c
    if (!mbd->mode_ref_lf_delta_enabled) {
      memset(lfi->lvl[seg][0], lvl_seg, 4 * 4);
      continue;
    }
```

If the frame header didn't enable mode/ref-frame adjustments, the
whole 4×4 ref×mode subtable for this segment is filled with the
segment's base level via a single 16-byte `memset`, and we skip the
rest of the derivation. This is the common case for short or
machine-generated streams. The comment in the source is honest
about the imperfection:

> /* we could get rid of this if we assume that deltas are set to
>  * zero when not in use; encoder always uses deltas */

i.e. the encoder side never writes streams with the deltas-disabled
flag set, so this fast path mostly handles third-party encoders.

**Stage C: ref-frame and mode deltas.** When deltas *are* enabled,
the function builds `lvl[seg][ref][mode]` ref-by-ref:

```c
    /* INTRA_FRAME */
    ref = INTRA_FRAME;
    lvl_ref  = lvl_seg + mbd->ref_lf_deltas[ref];

    mode = 0; /* B_PRED */
    lvl_mode = lvl_ref + mbd->mode_lf_deltas[mode];
    lvl_mode = clamp(lvl_mode, 0, 63);
    lfi->lvl[seg][ref][mode] = lvl_mode;

    mode = 1; /* all the rest of Intra modes */
    lvl_mode = clamp(lvl_ref, 0, 63);
    lfi->lvl[seg][ref][mode] = lvl_mode;

    for (ref = 1; ref < MAX_REF_FRAMES; ++ref) {
      lvl_ref = lvl_seg + mbd->ref_lf_deltas[ref];
      for (mode = 1; mode < 4; ++mode) {
        lvl_mode = lvl_ref + mbd->mode_lf_deltas[mode];
        lvl_mode = clamp(lvl_mode, 0, 63);
        lfi->lvl[seg][ref][mode] = lvl_mode;
      }
    }
```

A reader has to know what `ref_lf_deltas` and `mode_lf_deltas` are.
Both arrays live on `MACROBLOCKD` (`blockd.h:272, 276`):

  * `ref_lf_deltas[4]` — signed adjustments indexed by ref frame:
    [INTRA, LAST, GOLDEN, ALT]. The encoder can, for example, ask
    that intra-coded macroblocks be filtered more strongly than
    last-frame-predicted ones (the most common configuration is
    intra +ref delta of +1 or +2).
  * `mode_lf_deltas[4]` — signed adjustments indexed by the
    *mode bucket* defined by `mode_lf_lut`: [B_PRED, ZEROMV,
    NEW/NEAR/NEAREST, SPLITMV]. The encoder can ask, for instance,
    that split-mode MBs (lots of sub-block discontinuities) be
    filtered more strongly.

These deltas are also bitstream-encoded with a sign bit and
applied additively. Both arrays are populated when the loop-filter
update segment of the frame header is parsed (in `decodeframe.c`),
and may be left over from a prior frame when the header says
"keep previous deltas."

The intra ref (ref=0) is special-cased before the inter loop
because intra has only two relevant mode buckets:

  * **mode 0 = `B_PRED`** — gets the mode delta added.
  * **mode 1 = everything else intra (`DC/V/H/TM_PRED`)** — does
    not get a mode delta. RFC 6386 §15.3 says only `B_PRED` gets
    the BPRED delta; the other intra modes receive ref delta only.

For inter refs (LAST, GOLDEN, ALT), all three inter mode buckets
(1=ZEROMV, 2=NEAR/NEAREST/NEW, 3=SPLITMV) get the corresponding
`mode_lf_delta`. The 0 slot of those rows in `lvl[seg][ref][]` is
left uninitialized, because no inter MB ever maps to mode-bucket 0
(only `B_PRED` does, and `B_PRED` implies intra).

Every result is clamped to [0, 63] independently, so additions that
would otherwise overflow into negative or above-63 values are
handled.

**Invariants on exit.** For every `(seg, ref, mode)` that the
walker will ever index with: `lvl[seg][ref][mode]` is a valid byte
in 0..63. When that byte is 0, the walker will skip the MB entirely
(no filtering — see below).

---

## `vp8_loop_filter_frame` — the main raster walker

The work-horse. Called once per frame from `decoded_mb_rows` after
all reconstruction is done. Two structural halves: a normal-filter
loop and a simple-filter loop, selected on `cm->filter_type`. They
are nearly identical apart from the inner kernel calls; the file
keeps them as two parallel loops rather than a function-pointer
dispatch in the inner loop, since the branch hoists out and the
fewer indirect calls measurably help on small/in-order cores.

The setup is straightforward:

```c
  vp8_loop_filter_frame_init(cm, mbd, cm->filter_level);

  y_ptr = post->y_buffer;
  u_ptr = post->u_buffer;
  v_ptr = post->v_buffer;
```

`post = cm->frame_to_show` is the newly-reconstructed YV12 frame
buffer. After the walk it will be swapped into the reference ring.
`mode_info_context = cm->mi` points at the first per-MB `MODE_INFO`
record; the file walks it with `++mode_info_context` per MB and
`++mode_info_context` once per row to skip the **border column**
that VP8 maintains as a fictional MB at every row boundary (see
`alloccommon.c` for the `(mb_cols + 1)` allocation pattern).

### The per-MB pattern, common to both halves

At every MB the loop does this:

```c
  int skip_lf = (mode_info_context->mbmi.mode != B_PRED &&
                 mode_info_context->mbmi.mode != SPLITMV &&
                 mode_info_context->mbmi.mb_skip_coeff);

  const int mode_index = lfi_n->mode_lf_lut[mode_info_context->mbmi.mode];
  const int seg        = mode_info_context->mbmi.segment_id;
  const int ref_frame  = mode_info_context->mbmi.ref_frame;

  filter_level = lfi_n->lvl[seg][ref_frame][mode_index];
```

The three lookups read from `MB_MODE_INFO` and use the small LUTs
built earlier. `filter_level` is now in 0..63.

`skip_lf` is **the** sub-block-filter gating predicate. It says:

> if this MB has no coded residual (mb_skip_coeff=1) AND is not
> B_PRED AND is not SPLITMV, then there is no sub-block-edge
> discontinuity inside the MB worth filtering.

The two exclusions are necessary because both `B_PRED` and
`SPLITMV` build the 16×16 MB from 16 independent sub-blocks (intra
4×4 predictors for the former, distinct motion vectors for the
latter); even with zero residual, the sub-block joins can show as
visible seams. For every other mode, a residual-free MB is
internally smooth — only its MB-edges with neighbouring MBs
need filtering.

The next gate, `if (filter_level)`, is the master gate: a zero
per-MB filter level disables *all* edges for this MB.

When both gates pass, the normal filter builds the four-pointer
tear-off:

```c
  const int hev_index = lfi_n->hev_thr_lut[frame_type][filter_level];
  lfi.mblim   = lfi_n->mblim  [filter_level];
  lfi.blim    = lfi_n->blim   [filter_level];
  lfi.lim     = lfi_n->lim    [filter_level];
  lfi.hev_thr = lfi_n->hev_thr[hev_index];
```

…and dispatches four edges, in a specific raster-friendly order:

```c
  if (mb_col > 0)   vp8_loop_filter_mbv(...);   /* 1: left MB edge        */
  if (!skip_lf)     vp8_loop_filter_bv (...);   /* 2: 3 internal V edges  */
  if (mb_row > 0)   vp8_loop_filter_mbh(...);   /* 3: top MB edge         */
  if (!skip_lf)     vp8_loop_filter_bh (...);   /* 4: 3 internal H edges  */
```

The first three lines hide subtle correctness properties:

  * **Why left then top (not the other way).** Vertical edges
    overwrite columns *to the left* of the current MB; horizontal
    edges overwrite rows *above*. If we filtered the top edge
    first, the left-edge filter for the next column would
    immediately load samples that the top-edge filter had just
    written, producing a different result from the encoder's
    bottom-then-side order. The encoder commits to left-then-top
    and the decoder matches.

  * **Why guard with `mb_col > 0` and `mb_row > 0`.** Frame
    boundaries have no "previous" MB on the other side of the
    edge — filtering there would read into the buffer extension
    (which exists but contains undefined data for the loop
    filter's purposes). The unfiltered-margin (UMV) border is a
    deliberate decoder concept: see `extend.c`.

  * **Why the internal edges are gated on `!skip_lf`.** Same
    argument as for `skip_lf`: if the MB has no internal
    discontinuities, both `_bv` and `_bh` are unnecessary work.
    The kernels themselves don't know about MB-level skip; the
    decision is made here.

After the four-edge dispatch, the buffer pointers advance:

```c
  y_ptr += 16; u_ptr += 8; v_ptr += 8;
  mode_info_context++;
```

(`+16` for luma is the MB stride; `+8` for chroma is the MB stride
at 4:2:0.)

After each MB row, the pointers wrap to the next row and skip the
extra border MB:

```c
  y_ptr += post_y_stride * 16 - post->y_width;
  u_ptr += post_uv_stride * 8 - post->uv_width;
  v_ptr += post_uv_stride * 8 - post->uv_width;
  mode_info_context++;   /* Skip border mb */
```

### Simple filter half

The simple-filter loop has exactly the same MB iteration, same
`skip_lf` gating, same master `filter_level` gate. The only
differences are:

  * No `loop_filter_info` tear-off — the kernels take pointers
    directly.
  * No `hev_thr` (the simple filter doesn't do the
    high-edge-variance test).
  * No `lim` (the simple filter uses only `mblim`/`blim`).
  * No chroma (the simple filter is luma-only). The U/V pointers
    are *still* advanced inside the inner loop, however, even
    though the simple half never reads from them — this is a
    quirk of the code (note the `u_ptr += 8; v_ptr += 8;` in the
    simple half too).

The four kernel calls are the simple-variant analogues:

```c
  if (mb_col > 0) vp8_loop_filter_simple_mbv(y_ptr, post_y_stride, mblim);
  if (!skip_lf)   vp8_loop_filter_simple_bv (y_ptr, post_y_stride, blim);
  if (mb_row > 0) vp8_loop_filter_simple_mbh(y_ptr, post_y_stride, mblim);
  if (!skip_lf)   vp8_loop_filter_simple_bh (y_ptr, post_y_stride, blim);
```

---

## `vp8_loop_filter_row_normal` and `vp8_loop_filter_row_simple`

These are **row-granular** versions of the inner body of
`vp8_loop_filter_frame`. They take a `mode_info_context` already
advanced to the start of the row to filter, the row's `mb_row`
index (so the `mb_row > 0` guard works), and pre-incremented
buffer pointers. They do the body of the inner loop for `mb_cols`
columns and return.

These exist for the threaded build (`vp8/decoder/threading.c`):
worker threads compute one MB row apiece, and they invoke these
helpers rather than the full-frame walker. The minimal decoder
build (`--disable-multithread`) compiles them but never calls them.

The logic is identical to the inner body described above — same
`skip_lf` predicate, same kernel calls — and they are simply
factored copies. The two are kept separate for the same reason
the main walker has two parallel halves: avoid an indirect call
on the filter_type in the inner loop.

The `_simple` variant has the same `skip_lf` test as the normal
one despite the `B_PRED`/`SPLITMV` exclusion logic only mattering
for chroma — at the simple level the predicate happens to also be
the right condition for *luma* sub-block filtering, since the
encoder won't have coded distinct sub-block predictors except in
those two modes.

---

## `vp8_loop_filter_frame_yonly` — luma-only filtering

```c
void vp8_loop_filter_frame_yonly(VP8_COMMON *cm, MACROBLOCKD *mbd,
                                 int default_filt_lvl) {
  ...
  if (filter_level) {
    if (cm->filter_type == NORMAL_LOOPFILTER) {
      ...
      vp8_loop_filter_mbv(y_ptr, 0, 0, post->y_stride, 0, &lfi);
      ...
    } else {
      vp8_loop_filter_simple_mbv(y_ptr, post->y_stride,
                                 lfi_n->mblim[filter_level]);
      ...
    }
  }
  ...
}
```

The same raster walk as `vp8_loop_filter_frame`, but only the
luma plane is filtered. The chroma pointers are not passed at all
to the normal kernels; the literal `0`s in the call sites are the
`u_ptr`, `v_ptr`, and `uv_stride` arguments. The normal kernels
have an internal check that skips chroma when `u_ptr == NULL`.

The function is used by tooling that needs to deblock a luma
plane in isolation (e.g. the encoder's filter-level search). The
decoder build does not call it. It is included in the
decoder-only build because the file is compiled as a unit and
the symbol is exported.

---

## `vp8_loop_filter_partial_frame` — partial-frame filtering

The third specialty walker. It filters only a fraction of the
frame, centered roughly around the vertical middle:

```c
  linestocopy = mb_rows / PARTIAL_FRAME_FRACTION;
  linestocopy = linestocopy ? linestocopy << 4 : 16; /* 16 lines per MB */

  y_ptr = post->y_buffer + ((post->y_height >> 5) * 16) * post->y_stride;
  mode_info_context = cm->mi + (post->y_height >> 5) * (mb_cols + 1);
```

`PARTIAL_FRAME_FRACTION` is fixed at 8 (`loopfilter.h:25`), so this
filters at most one-eighth of the frame's MB rows. The starting
offset places the partial region in the middle vertically: the
expression `(post->y_height >> 5) * 16` is `(height / 32) * 16`,
which lands at approximately the frame's center (more precisely:
half-way through the rows, rounded down to a 16-pixel boundary).

This is, again, an encoder utility: the rate-control search uses it
to evaluate the cost of different `filter_level` choices on a
representative slice of the frame without the cost of filtering the
whole thing. The decoder never invokes it. One small structural
difference from the full walker is worth noting: the partial walker
*does not* guard `vp8_loop_filter_mbh` with `mb_row > 0`, because
its first row is by construction in the middle of the frame, never
at row 0.

---

## Filter dispatch summary (the punchline)

For a reader who wants the whole file's behavior in one diagram:

```
                          cm->filter_type ?
                            /          \
                  NORMAL_LOOPFILTER     SIMPLE_LOOPFILTER
                       │                       │
            ┌──────────┴──────────┐  ┌─────────┴─────────┐
            │ vp8_loop_filter_mbv │  │ vp8_loop_filter_  │
            │  + mb-edge limits   │  │  simple_mbv       │
            │  + hev_thr          │  │  + mb-edge limit  │
            │  Y + U + V          │  │  only Y           │
            └─────────────────────┘  └───────────────────┘
                       │                       │
              ... bv / mbh / bh ...    ... simple_bv/mbh/bh ...
                                       (no chroma, no hev_thr)
```

Per-MB strength is `lfi->lvl[seg][ref][mode_bucket]`, built once
per frame by `vp8_loop_filter_frame_init` as the clamped sum
`default_filt_lvl + ref_lf_deltas[ref] + mode_lf_deltas[mode]`
(plus the segment override at the top). The per-level
byte-constants `mblim/blim/lim/hev_thr` are precomputed for every
strength 0..63 once per *session* by `vp8_loop_filter_init` and
refreshed only when the bitstream changes the sharpness.

Cross-reference to the technical overview:

  * `documentation/vp8_technical_overview.md:1016` — §11
    "Deblocking (loop filter)" — derives the same equations and
    shows the four-edge raster order graphically.
  * `documentation/vp8_technical_overview.md:2025–2026` — bitstream
    correspondence: this file == RFC 6386 §15 and §15.4.
  * `vp8/common/loopfilter_filters.c` — the kernels this driver
    calls into.
  * `vp8/common/loopfilter.h` — `loop_filter_info_n`,
    `loop_filter_info`, `MAX_LOOP_FILTER`, `SIMD_WIDTH`,
    `LOOPFILTERTYPE`.
  * `vp8/common/onyxc_int.h:128–134` — the seven `VP8_COMMON`
    fields this file reads and writes (`filter_type`, `lf_info`,
    `filter_level`, `last_sharpness_level`, `sharpness_level`,
    `frame_type`, `frame_to_show`).
  * `vp8/common/blockd.h:254–276` — the `MACROBLOCKD` fields that
    carry the per-segment / per-ref / per-mode adjustments
    (`mb_segment_abs_delta`, `segment_feature_data`,
    `mode_ref_lf_delta_enabled`, `ref_lf_deltas`, `mode_lf_deltas`).
  * `vp8/common/rtcd_defs.pl:58–109` — the eight edge-kernel slots
    in the RTCD table that the driver dispatches through.
