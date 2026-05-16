# `vp8/common/reconintra.c` — whole-macroblock intra prediction

This is one of the smallest non-trivial translation units in the VP8
decoder: roughly a hundred lines, four functions, two static dispatch
tables. Its job is to take a macroblock that has been signalled as
intra-coded with one of the four *macroblock-level* intra modes
(`DC_PRED`, `V_PRED`, `H_PRED`, `TM_PRED`) and synthesise the 16x16
luma predictor plus the two 8x8 chroma predictors into the destination
YV12 buffer. The actual pixel kernels do not live here; they live in
`vpx_dsp/intrapred.c` and are reached through the run-time CPU dispatch
(`vp8_rtcd.h` / `vpx_dsp_rtcd.h`). `reconintra.c`'s contribution is the
*glue*: it (a) builds and caches a pair of function-pointer tables, (b)
gathers the boundary samples that the kernels read, and (c) selects the
right kernel for the requested mode and the current macroblock's
"left/above available" flags.

The fifth VP8 macroblock mode, `B_PRED`, is *not* served here — it
decomposes the luma plane into sixteen 4x4 sub-blocks each with its
own sub-mode, and is handled by the neighbouring
`vp8/common/reconintra4x4.c`. The two files share an initialiser:
`vp8_init_intra_predictors_internal()` below calls
`vp8_init_intra4x4_predictors_internal()` so that a single `once()`
call at decoder start-up wires up both dispatch tables.

## Role in the decoder

VP8 (RFC 6386 §12) defines two layers of intra prediction. At the
*macroblock* layer the luma 16x16 block and each of the two 8x8 chroma
blocks pick one of four whole-block modes:

> "DC_PRED, V_PRED, H_PRED, TM_PRED" — `vp8/common/blockd.h:66-69`

```c
typedef enum {
  DC_PRED, /* average of above and left pixels */
  V_PRED,  /* vertical prediction              */
  H_PRED,  /* horizontal prediction            */
  TM_PRED, /* Truemotion prediction            */
  B_PRED,  /* block based prediction, each block has its own prediction mode */
  ...
} MB_PREDICTION_MODE;
```

The luma mode is `mbmi.mode`; the chroma mode (used for both U and V)
is the independent `mbmi.uv_mode`. When `mbmi.mode == B_PRED` the
luma path falls through to the per-4x4 predictor in
`reconintra4x4.c`; the chroma path is always one of the four
whole-block modes regardless of whether luma is `B_PRED`.

`reconintra.c` is therefore on the critical decode path for every
intra MB:

```
decodeframe.c:147   if (xd->mode_info_context->mbmi.ref_frame == INTRA_FRAME) {
                      vp8_build_intra_predictors_mbuv_s(...);  /* this file */
decodeframe.c:153     if (mode != B_PRED)
                        vp8_build_intra_predictors_mby_s(...); /* this file */
                      else
                        /* 4x4 path in reconintra4x4.c */
```

After this file has filled `xd->dst.{y,u,v}_buffer` with the
predictor, the IDCT path (`idct_blk.c`) adds the dequantised residual
on top, and the loop filter (`vp8_loopfilter.c`) eventually smooths
block edges. Until that point the predictor lives directly in the
*output* frame buffer — VP8's decoder never materialises a separate
"predictor" temporary the way some other codecs do — so the bytes this
file writes are the bytes the next macroblock will read as *its*
left/above neighbours.

## Dispatch tables

```c
typedef void (*intra_pred_fn)(uint8_t *dst, ptrdiff_t stride,
                              const uint8_t *above, const uint8_t *left);

static intra_pred_fn pred[4][NUM_SIZES];
static intra_pred_fn dc_pred[2][2][NUM_SIZES];
```

### `intra_pred_fn` — the kernel signature

Every concrete VP8 intra kernel in `vpx_dsp/intrapred.c` has the same
four-argument shape: a destination pointer, the destination row stride,
a pointer to the row of samples *above* the block (with `above[-1]`
holding the top-left pixel), and a pointer to a contiguous column of
samples *left* of the block (`left[0]` is the topmost left-neighbour).
Hoisting that signature into a typedef lets `reconintra.c` index two
flat function-pointer tables instead of writing a `switch` per mode.

*Why a typedef and a function pointer instead of a `switch`?* Because
the four modes are dispatched once per macroblock — hundreds of
thousands of times per second at HD frame rates — and because each
mode has *up to four* concrete kernels (scalar C plus SSE2/NEON/MSA
specialisations chosen at run time by `vp8_rtcd.h`). A table lookup
amortises the RTCD choice across the whole stream and removes a
predictable but pointless branch from the hot loop.

### `pred[4][NUM_SIZES]` — non-DC modes

The `pred` table is indexed by `MB_PREDICTION_MODE` (values 0..3, the
four whole-block modes) and by a size token. `NUM_SIZES` is 2: the
luma path uses `SIZE_16` (16x16), the chroma path `SIZE_8` (8x8).
Slot `[DC_PRED][*]` is **never read** — DC has its own three-by-three
table because the kernel choice depends on neighbour availability. The
table is left zero-initialised in that slot; the dispatch in the two
build functions explicitly routes `DC_PRED` to `dc_pred[...]` instead.

### `dc_pred[2][2][NUM_SIZES]` — DC variants by availability

```
dc_pred[left_available][up_available][size]
                    = vpx_dc_*_predictor_NxN
```

RFC 6386 §12.2 makes DC prediction a four-way decision: if both the
above row and the left column exist (interior macroblock), the
predictor is `(sum(above) + sum(left) + N) / (2N)`; if only the above
row exists (first column of macroblocks), it is `(sum(above) + N/2) /
N`; if only the left column exists (first row), it is symmetric; and
if *neither* exists (the very first MB of the picture), the value is
the neutral grey constant 128. Encoding all four cases as a 2x2x2
table lets the dispatcher pick the right kernel with two array indexes
and no further branching:

| `left_avail` | `up_avail` | kernel chosen           |
|--------------|-----------|--------------------------|
| 0            | 0         | `vpx_dc_128_predictor`   |
| 0            | 1         | `vpx_dc_top_predictor`   |
| 1            | 0         | `vpx_dc_left_predictor`  |
| 1            | 1         | `vpx_dc_predictor`       |

The two flags come straight from `MACROBLOCKD::up_available` and
`MACROBLOCKD::left_available` (`vp8/common/blockd.h:232-233`), which
the frame-level driver sets to 1 once it has crossed the corresponding
picture edge.

## Initialisation

### `vp8_init_intra_predictors_internal()` — populate the tables

```c
static void vp8_init_intra_predictors_internal(void) {
#define INIT_SIZE(sz)                                           \
  pred[V_PRED][SIZE_##sz] = vpx_v_predictor_##sz##x##sz;        \
  pred[H_PRED][SIZE_##sz] = vpx_h_predictor_##sz##x##sz;        \
  pred[TM_PRED][SIZE_##sz] = vpx_tm_predictor_##sz##x##sz;      \
                                                                \
  dc_pred[0][0][SIZE_##sz] = vpx_dc_128_predictor_##sz##x##sz;  \
  dc_pred[0][1][SIZE_##sz] = vpx_dc_top_predictor_##sz##x##sz;  \
  dc_pred[1][0][SIZE_##sz] = vpx_dc_left_predictor_##sz##x##sz; \
  dc_pred[1][1][SIZE_##sz] = vpx_dc_predictor_##sz##x##sz

  INIT_SIZE(16);
  INIT_SIZE(8);
  vp8_init_intra4x4_predictors_internal();
}
```

The macro expansion is mechanical: for each of the two block sizes,
fill three slots of `pred` with the V/H/TM kernels and four slots of
`dc_pred` with the DC variants. The names on the right-hand side
(`vpx_v_predictor_16x16` etc.) are *not* the C kernel directly: they
are macros generated by `vp8_rtcd.h`/`vpx_dsp_rtcd.h` (`build/make/
rtcd.pl` reads `vpx_dsp/vpx_dsp_rtcd_defs.pl`) that resolve to the
function pointer chosen at run time after `vpx_dsp_rtcd()` has probed
the CPU. So the dispatch tables in this file have *two* levels of
indirection: mode → C function symbol, and (inside that symbol) C
function symbol → architecture-specific implementation. By the time
control reaches `fn(dst, stride, above, left)` both have been
resolved.

The trailing call to `vp8_init_intra4x4_predictors_internal()`
(`reconintra4x4.c`) is the bridge for the `B_PRED` path: a single
initialiser entry point covers both whole-MB and per-4x4 intra.

*Invariant:* the table slot `pred[DC_PRED][*]` is left as a null
pointer and must never be dereferenced; the two build functions
guarantee this by routing `DC_PRED` through `dc_pred` instead. Slots
above `TM_PRED` (`B_PRED`, `NEARESTMV`, ...) are also unreachable
because `B_PRED` is filtered out by the caller and inter modes never
reach these functions.

### `vp8_init_intra_predictors()` — once-per-process wrapper

```c
void vp8_init_intra_predictors(void) {
  once(vp8_init_intra_predictors_internal);
}
```

`once()` is the libvpx-wide thread-safe "run exactly once" gate
(`vpx_ports/vpx_once.h`). The internal initialiser writes to two
file-scope tables that *all* decoder instances in the process share,
so it must be both idempotent and race-free. Wrapping the populator
in `once()` makes it safe to call from every `vp8_create_decoder_instances`
without re-doing the work, and also without two threads racing to fill
the same slots with the same pointers (which would be harmless in
practice but would still trip a race detector).

This is the function called from the public entry points:

```
vp8/decoder/onyxd_if.c:53   vp8_init_intra_predictors();
vp8/encoder/onyx_if.c:412   vp8_init_intra_predictors();
```

## Border samples — how the predictors get their inputs

Before discussing the two build functions, it is worth being explicit
about *where* the above row and the left column come from, because
that is the load-bearing detail of VP8 intra prediction.

The two pointers passed to each kernel are conceptually:

* `above` — a pointer to the row of *already reconstructed* samples
  one line above the destination block. `above[0..N-1]` are the N
  pixels directly above the block; `above[-1]` is the top-left pixel
  (TM mode needs it). For a 16x16 luma block, N=16; for an 8x8 chroma
  block, N=8.
* `left` — a pointer to N already-reconstructed samples to the left,
  *laid out contiguously*. `left[0]` is the leftmost top sample,
  `left[N-1]` the leftmost bottom sample.

The above row is naturally contiguous in the YV12 frame buffer because
rows are stored linearly: the caller can pass `yabove_row` directly as
`dst - dst_stride`. The left column is *not* contiguous — successive
left-neighbour pixels are `stride` bytes apart — so the caller copies
them into a small stack buffer first. That copy is exactly what the
loops in `vp8_build_intra_predictors_mby_s` and `_mbuv_s` do.

At picture boundaries the data the kernels would read does not exist.
RFC 6386 §12.2 specifies that for the first row of macroblocks the
above row shall be treated as a constant 127, and that for the first
column of macroblocks the left column shall be 129; the top-left
corner is 127. libvpx implements this in `vp8/common/setupintrarecon.c`,
not here, by *physically* writing 127 to the byte preceding row 0 of
each plane and 129 to the byte before each row:

```c
/* vp8/common/setupintrarecon.c:18-21 */
memset(ybf->y_buffer - 1 - ybf->y_stride, 127, ybf->y_width + 5);
for (i = 0; i < ybf->y_height; ++i) {
  ybf->y_buffer[ybf->y_stride * i - 1] = (unsigned char)129;
}
```

The `127` / `129` asymmetry is deliberate: for a fully-unavailable
corner, DC prediction would average them and round to 128 — the
neutral grey value used as the prior for everything VP8 has not yet
decoded. By baking these sentinels into the buffer *before* decoding
starts, `reconintra.c` can be entirely oblivious to the picture edge:
it just reads `yabove_row[0..15]` and `yleft[i*stride]` without any
edge tests, and the right values come back automatically.

The frame driver in `decodeframe.c` re-establishes the per-MB
neighbour pointers (`xd->recon_above[]`, `xd->recon_left[]`) for each
MB before calling into this file:

```
decodeframe.c:506-516   xd->recon_above[0] = dst_buffer[0] + recon_yoffset;
                        ...
                        xd->recon_left[0]  = xd->recon_above[0] - 1;
                        xd->recon_above[0] -= xd->dst.y_stride;
```

So `xd->recon_above[0]` always points one row above `xd->dst.y_buffer`
and `xd->recon_left[0]` points one column to the left of
`xd->dst.y_buffer`; analogously for chroma at index 1 (U) and 2 (V).
At the picture boundary those addresses land on the 127/129 sentinel
bytes; in the interior they land on samples that earlier macroblocks
have already reconstructed *and loop-filtered into the same buffer*.

Note the `left_stride` parameter the build functions receive: for
normal in-frame MBs it equals the destination stride (samples really
are one row apart), but for the special left-column case the frame
driver may set it to 1 so that a *contiguous* sentinel column
(`recon_left = synthetic 129-buffer`) can be reused. The functions do
not need to know which case they are in — they just dereference
`yleft[i * left_stride]`.

## The two build functions

### `vp8_build_intra_predictors_mby_s` — 16x16 luma predictor

```c
void vp8_build_intra_predictors_mby_s(MACROBLOCKD *x, unsigned char *yabove_row,
                                      unsigned char *yleft, int left_stride,
                                      unsigned char *ypred_ptr, int y_stride) {
  MB_PREDICTION_MODE mode = x->mode_info_context->mbmi.mode;
  DECLARE_ALIGNED(16, uint8_t, yleft_col[16]);
  int i;
  intra_pred_fn fn;

  for (i = 0; i < 16; ++i) {
    yleft_col[i] = yleft[i * left_stride];
  }

  if (mode == DC_PRED) {
    fn = dc_pred[x->left_available][x->up_available][SIZE_16];
  } else {
    fn = pred[mode][SIZE_16];
  }

  fn(ypred_ptr, y_stride, yabove_row, yleft_col);
}
```

The function does three things, in order:

1. **Linearise the left column.** The caller hands in `yleft`, a
   pointer to a sample, plus `left_stride` (sample-to-sample byte
   distance for that column). The 16 left-neighbour samples are
   gathered into a 16-aligned stack array `yleft_col[16]`. The 16-byte
   alignment matters because the SSE2 / NEON specialisations read
   16-byte vectors out of this buffer; without `DECLARE_ALIGNED(16,
   ...)` the kernel would have to fall back to an unaligned load. The
   above row, in contrast, is *already* linear in memory, so the
   caller's `yabove_row` is passed through verbatim.

2. **Pick the kernel.** The four-mode dispatch is a single lookup. If
   the requested mode is `DC_PRED` the table is `dc_pred`, indexed by
   the two availability flags; otherwise it is `pred[mode]`. Either
   way, the SIZE_16 slice selects the 16x16 specialisation.

3. **Call it.** The kernel writes 16x16 samples into `ypred_ptr` with
   row stride `y_stride`.

*Why is the function suffixed `_s`?* The historical libvpx convention
distinguished `_s` ("stride", or "scattered") variants that take an
explicit `left_stride` from older variants that assumed the
predictor was being written to a contiguous 16-byte-pitch temporary.
The non-`_s` variants have long since been removed; the `_s` suffix
is now vestigial but kept for ABI stability across the encoder and
decoder build.

*Invariant on the mode argument.* `mode` must be one of
`{DC_PRED, V_PRED, H_PRED, TM_PRED}`. `B_PRED` is filtered out by the
caller (`decodeframe.c:153 if (mode != B_PRED)`) because it has its
own 4x4-grain reconstruction loop; inter modes never reach this code
because the caller guards on `mbmi.ref_frame == INTRA_FRAME`. If
`B_PRED` ever slipped through, `pred[B_PRED][SIZE_16]` would be NULL
and the call would crash — there is no defensive `assert` here, the
correctness of `decodeframe.c` is part of the precondition.

### `vp8_build_intra_predictors_mbuv_s` — 8x8 chroma predictor (U and V)

```c
void vp8_build_intra_predictors_mbuv_s(
    MACROBLOCKD *x, unsigned char *uabove_row, unsigned char *vabove_row,
    unsigned char *uleft, unsigned char *vleft, int left_stride,
    unsigned char *upred_ptr, unsigned char *vpred_ptr, int pred_stride) {
  MB_PREDICTION_MODE uvmode = x->mode_info_context->mbmi.uv_mode;
#if HAVE_VSX
  unsigned char uleft_col[16];
  unsigned char vleft_col[16];
#else
  unsigned char uleft_col[8];
  unsigned char vleft_col[8];
#endif
  ...
  for (i = 0; i < 8; ++i) {
    uleft_col[i] = uleft[i * left_stride];
    vleft_col[i] = vleft[i * left_stride];
  }

  if (uvmode == DC_PRED) {
    fn = dc_pred[x->left_available][x->up_available][SIZE_8];
  } else {
    fn = pred[uvmode][SIZE_8];
  }

  fn(upred_ptr, pred_stride, uabove_row, uleft_col);
  fn(vpred_ptr, pred_stride, vabove_row, vleft_col);
}
```

This is the chroma twin of the luma function. The structure is
identical — gather left columns into linear stack buffers, select the
kernel by mode (and availability for DC), invoke it — except that:

* it operates on two planes (U and V) in succession, using the *same*
  kernel for both. RFC 6386 makes chroma share a single `uv_mode`
  selector, so U and V always use the same predictor type with the
  same border conditions; the two calls differ only in their source
  borders and destination pointers.
* the block size is 8x8 (`SIZE_8`), so the gather loop runs 8 times
  and the kernel writes 64 samples per plane.
* the dispatch table indexing is identical to the luma function. The
  *same* `dc_pred[2][2][SIZE_8]` and `pred[mode][SIZE_8]` slots are
  consulted for both U and V — the chroma table is not separate.

#### The `HAVE_VSX` stack-size pad

```c
#if HAVE_VSX
  /* Power PC implementation uses "vec_vsx_ld" to read 16 bytes from
     uleft_col and vleft_col. Play it safe by reserving enough stack
     space here. */
  unsigned char uleft_col[16];
  unsigned char vleft_col[16];
#else
  unsigned char uleft_col[8];
  unsigned char vleft_col[8];
#endif
```

The PowerPC VSX kernels in `vpx_dsp/ppc/intrapred_vsx.c` use
`vec_vsx_ld`, an instruction that loads a full 128-bit vector even
when only 64 bits are wanted. Without this guard the loader would
read 8 bytes of stack past the end of an 8-byte buffer — almost
certainly harmless in practice but formally undefined behaviour and a
valgrind/ASan trip-wire. Reserving 16 bytes on VSX builds and 8
elsewhere is the minimal portable fix: the gather loop still writes
only 8 bytes; the kernel reads 16 and ignores the top 8.

*Why not unconditionally allocate 16?* Stack hygiene is cheap to get
right and `_mbuv_s` is on the hot path; the extra 16 bytes per call
would not actually matter, but the `#if` makes the intent visible —
"these eight padding bytes exist specifically because of VSX, nothing
else uses them" — and keeps the non-VSX build byte-identical to
older revisions.

#### Invariant: U and V are always coded with the same mode

This is a property of the VP8 bitstream, not of this file: RFC 6386
§13.4 only ever reads a single `uv_mode` per macroblock. The function
relies on this by issuing both `fn(upred_ptr, ...)` and `fn(vpred_ptr,
...)` with the same `fn`. If a future codec generation wanted
separate U and V modes it would not be enough to add a second
dispatch — the API itself would need a new argument.

## How the four mode kernels work

The implementations live in `vpx_dsp/intrapred.c` (search for
`intra_pred_sized(v, 16)`, `intra_pred_sized(h, 16)`,
`intra_pred_sized(tm, 16)`, `intra_pred_sized(dc, 16)`, plus the
`dc_top` / `dc_left` / `dc_128` variants). They are exactly the
RFC 6386 §12.2 definitions, written as straight C:

### `vpx_v_predictor_NxN` — `V_PRED`, vertical replication

```c
static INLINE void v_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                               const uint8_t *above, const uint8_t *left) {
  int r; (void)left;
  for (r = 0; r < bs; r++) { memcpy(dst, above, bs); dst += stride; }
}
```

Each row of the predicted block is a copy of the above-row sample
vector. The left column is unused. RFC 6386 §12.2: "The first row
above the block is replicated downward to form the prediction."

### `vpx_h_predictor_NxN` — `H_PRED`, horizontal replication

```c
static INLINE void h_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                               const uint8_t *above, const uint8_t *left) {
  int r; (void)above;
  for (r = 0; r < bs; r++) { memset(dst, left[r], bs); dst += stride; }
}
```

Each row of the predicted block is the single left-column sample
`left[r]` broadcast across `bs` columns. The above row is unused.
RFC 6386 §12.2: "The first column left of the block is replicated to
the right."

### `vpx_tm_predictor_NxN` — `TM_PRED`, "True Motion"

```c
static INLINE void tm_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                                const uint8_t *above, const uint8_t *left) {
  int r, c;
  int ytop_left = above[-1];
  for (r = 0; r < bs; r++) {
    for (c = 0; c < bs; c++)
      dst[c] = clip_pixel(left[r] + above[c] - ytop_left);
    dst += stride;
  }
}
```

True Motion (an On2/VP3 invention preserved into VP8) approximates a
slowly-varying gradient: each pixel is `left[r] + above[c] - TL`,
clipped to 0..255. Intuitively, `above[c] - TL` is the horizontal
delta the top row implies at column `c`, and adding it to the
left-column sample at row `r` propagates that delta down. It is the
only mode that reads the corner sample `above[-1]`; all the others
treat the corner as part of either the row or the column.

### DC family — `vpx_dc_predictor_NxN` and the three variants

The fully-available variant averages all 2N border samples:

```c
static INLINE void dc_predictor(uint8_t *dst, ptrdiff_t stride, int bs,
                                const uint8_t *above, const uint8_t *left) {
  int i, r, expected_dc, sum = 0;
  const int count = 2 * bs;
  for (i = 0; i < bs; i++) { sum += above[i]; sum += left[i]; }
  expected_dc = (sum + (count >> 1)) / count;
  for (r = 0; r < bs; r++) { memset(dst, expected_dc, bs); dst += stride; }
}
```

`vpx_dc_top_predictor_NxN` and `vpx_dc_left_predictor_NxN` are the
same shape with `count = bs` and the sum taken over a single border.
`vpx_dc_128_predictor_NxN` simply fills with the constant 128 — used
when neither neighbour exists, i.e. the very first macroblock of the
picture. The four kernels correspond one-for-one to the four cells of
the `dc_pred[left_avail][up_avail][size]` table.

It is worth noticing that even `vpx_dc_128_predictor` could be
replaced by a `memset(128)` in the dispatcher; the dispatch table is
deliberately *uniform* (same `intra_pred_fn` signature, same call
sequence) so that the calling code is identical regardless of which
DC variant fires. The cost of one extra indirect call is dwarfed by
the cost of the prediction itself.

## Tracing back to RFC 6386

| RFC 6386 reference | This file                          |
|--------------------|------------------------------------|
| §12.1 *Intra Prediction Modes — Luma*   | `pred[mode][SIZE_16]` dispatch in `vp8_build_intra_predictors_mby_s` |
| §12.1 *Intra Prediction Modes — Chroma* | dispatch on `uv_mode` in `vp8_build_intra_predictors_mbuv_s`         |
| §12.2 "DC_PRED" with 4-way border handling | `dc_pred[left_available][up_available][...]` table |
| §12.2 "V_PRED" replicate above row    | `vpx_v_predictor_NxN` (called via `pred[V_PRED][...]`) |
| §12.2 "H_PRED" replicate left column  | `vpx_h_predictor_NxN` (called via `pred[H_PRED][...]`) |
| §12.2 "TM_PRED" `L[i] + A[j] - TL`    | `vpx_tm_predictor_NxN` (called via `pred[TM_PRED][...]`) |
| §12.2 boundary fill (127/129/127)     | implemented in `vp8/common/setupintrarecon.c`; consumed transparently here |
| §11.5 "B_PRED" per-4x4 sub-modes      | *not in this file*; see `vp8/common/reconintra4x4.c`                  |

The combination "RFC-defined kernel set" plus "borders pre-filled at
buffer allocation time" plus "dispatch table chosen at process start"
is what makes `reconintra.c` so terse: there is no logic to spend
words on, only the right `fn(dst, stride, above, left)` to find. Once
that one line is on the screen, the entire whole-MB intra-prediction
mechanism of VP8 has been delivered.

## Files referenced

- `/home/eugene/projects/libvpx/vp8/common/reconintra.c` — this file
- `/home/eugene/projects/libvpx/vp8/common/reconintra.h` — public prototypes
- `/home/eugene/projects/libvpx/vp8/common/reconintra4x4.c` — `B_PRED` path; shares the once-only initialiser
- `/home/eugene/projects/libvpx/vp8/common/setupintrarecon.c` — writes the 127/129 sentinel borders
- `/home/eugene/projects/libvpx/vp8/common/blockd.h` — `MB_PREDICTION_MODE`, `MACROBLOCKD::{up,left}_available`, `recon_{above,left}[]`
- `/home/eugene/projects/libvpx/vp8/decoder/decodeframe.c` — caller for the decode path (lines ~146-156, 506-523)
- `/home/eugene/projects/libvpx/vp8/decoder/onyxd_if.c` — calls `vp8_init_intra_predictors()`
- `/home/eugene/projects/libvpx/vpx_dsp/intrapred.c` — concrete C kernels (`v_predictor`, `h_predictor`, `tm_predictor`, `dc_predictor` and variants)
- `/home/eugene/projects/libvpx/vpx_dsp/vpx_dsp_rtcd_defs.pl` — RTCD declarations for the kernels (lines 116-186)
- `/home/eugene/projects/libvpx/vpx_ports/vpx_once.h` — `once()` thread-safe-init helper
