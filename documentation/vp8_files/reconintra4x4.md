# `vp8/common/reconintra4x4.c` — per-4×4 luma intra prediction (`B_PRED`)

This file is the entire C-language realisation of VP8's *per-subblock*
intra-prediction path. Where `reconintra.c` builds one 16×16 luma
predictor (or 8×8 chroma predictor) at MB granularity, this file is what
runs when the macroblock chose `mbmi.mode == B_PRED` — the mode in which
every one of the sixteen 4×4 luma sub-blocks carries its *own*
prediction mode, drawn from a richer 10-entry alphabet of directional /
DC patterns.

The translation unit is only 76 lines and contains exactly three things:

- a function-pointer table `pred[10]` keyed by `B_PREDICTION_MODE`,
- the initialiser `vp8_init_intra4x4_predictors_internal()` that wires
  each mode to a kernel exported by `vpx_dsp/intrapred.c`,
- the dispatcher `vp8_intra4x4_predict()` that assembles the boundary
  samples for one 4×4 block and invokes the kernel.

The actual pixel arithmetic for every mode lives in `vpx_dsp`; this file
is the *glue* between (a) VP8's per-block decode loop in
`vp8/decoder/decodeframe.c` and `vp8/decoder/threading.c` and (b) the
sized, generic intra-prediction kernels that VP9 also uses. A companion
`static INLINE` helper, `intra_prediction_down_copy()`, lives in the
header (`reconintra4x4.h:19`) because it is required at every MB but is
trivial enough to inline — its role in the dataflow is discussed below.

## Role in the decoder

VP8 always transforms residuals on a 4×4 grid (RFC 6386 §13, §14), and
in the simplest intra modes (`DC_PRED`, `V_PRED`, `H_PRED`, `TM_PRED`)
the *prediction* is performed once on the whole 16×16 luma block before
the residuals are added. `B_PRED` is the alternative: prediction is
performed *one 4×4 block at a time*, in raster order within the
macroblock, with each block's predictor allowed to read the
*already-reconstructed* samples of its neighbours. This is what makes
`B_PRED` powerful — fine-grained directional prediction can follow the
local structure of an edge — and also what dictates the order of
operations in the decoder loop:

```c
for (i = 0; i < 16; ++i) {
  BLOCKD *b = &xd->block[i];
  unsigned char *dst       = xd->dst.y_buffer + b->offset;
  B_PREDICTION_MODE b_mode = xd->mode_info_context->bmi[i].as_mode;
  unsigned char *Above     = dst - dst_stride;
  unsigned char *yleft     = dst - 1;
  unsigned char top_left   = Above[-1];

  vp8_intra4x4_predict(Above, yleft, dst_stride, b_mode, dst,
                       dst_stride, top_left);
  /* … dequantise & add residual into dst in place … */
}
```
(`vp8/decoder/decodeframe.c:166`)

Each iteration predicts the i-th 4×4 sub-block from samples that
include the *output* of all previous iterations — the
just-reconstructed neighbours from the left, above, above-left, and
above-right. That feedback loop is the whole point of `B_PRED`, and the
present file is the engine that closes it.

The function `vp8_intra4x4_predict` itself is *stateless*: it consults
neither `MACROBLOCKD` nor the frame; it takes raw pointers into the
reconstructed luma buffer plus a mode enum, and writes a 4×4 prediction
into `dst`. All the address arithmetic that locates the four-neighbour
sample sets is the caller's responsibility — and is identical between
the single-thread and threaded decoders (compare
`decodeframe.c:175` with `threading.c:192`).

## Boundary-sample geometry

Each 4×4 intra-prediction mode in VP8 reads from at most fourteen
neighbouring samples laid out in an L-shape:

```
            P  A0 A1 A2 A3   A4 A5 A6 A7
            L0 . . . .
            L1 . . . .
            L2 . . . .
            L3 . . . .
```

where `P` is the single *top-left* sample, `A0..A3` are the four
samples directly above the block, `A4..A7` are the four samples that
sit above the *right* neighbour, and `L0..L3` are the four samples
directly to the left. VP8 spells those out in RFC 6386 §12.2 as
`P`, `A0..A7`, `L0..L3`. Different modes need different subsets:

- DC mode needs all four `A` and all four `L`.
- The "easy" directional modes `VE` and `HE` need only the immediate
  edge plus a couple of corner samples.
- The diagonal modes `LD`, `RD`, `VR`, `VL`, `HD`, `HU` each need
  some combination of `A0..A7`, `L0..L3`, and `P`. In particular
  `LD` ("left-down", a.k.a. mode 4) and `VL` ("vertical-left",
  mode 7) read **all eight** above-samples, including the four that
  belong to the macroblock's right-side neighbour.

That last point is what creates the famous chicken-and-egg of `B_PRED`:
the samples `A4..A7` of sub-block 3 (the top-right 4×4 of the MB) live
in the *right-hand neighbouring macroblock*, which on a normal raster
scan has not been processed yet at the *right edge of the picture*, and
which — even mid-row — would in principle have to wait its turn. VP8
escapes the trap with two complementary tricks:

### Above-right replication via `intra_prediction_down_copy`

When the predictor for the four sub-blocks in the *right-most column*
of an MB needs samples `A4..A7`, those samples sit one row above the
block — i.e. in the bottom row of the macroblock that is diagonally
above-right of the current MB. That MB has already been decoded
(raster order: above-right is decoded one row earlier and one column
later, hence available before our row begins). The samples we need are
therefore concretely available *at* `xd->dst.y_buffer - dst_stride +
16`, the byte just above the right edge of the current MB.

What the inline helper `intra_prediction_down_copy()`
(`reconintra4x4.h:19`) does is replicate those four bytes *downward*
into the same column of `Above` for the three rows of sub-blocks that
sit further down inside the MB:

```c
static INLINE void intra_prediction_down_copy(MACROBLOCKD *xd,
                                              unsigned char *above_right_src) {
  int dst_stride = xd->dst.y_stride;
  unsigned char *above_right_dst = xd->dst.y_buffer - dst_stride + 16;

  unsigned int *src_ptr  = (unsigned int *)above_right_src;
  unsigned int *dst_ptr0 = (unsigned int *)(above_right_dst + 4  * dst_stride);
  unsigned int *dst_ptr1 = (unsigned int *)(above_right_dst + 8  * dst_stride);
  unsigned int *dst_ptr2 = (unsigned int *)(above_right_dst + 12 * dst_stride);

  *dst_ptr0 = *src_ptr;
  *dst_ptr1 = *src_ptr;
  *dst_ptr2 = *src_ptr;
}
```

The geometry: `above_right_dst` is the four-byte slot that sits one
scanline above the byte at column 16 of the current MB. We copy the
same four bytes into the slots `+4`, `+8`, and `+12` rows further
down — i.e. one scanline above sub-blocks 7, 11, and 15 (the bottom of
each of the four sub-block *rows* of the MB). This pre-stages the
"above-right" neighbourhood for every sub-block in the right-most
column, *before* the per-sub-block loop begins.

Why a copy rather than a pointer fix-up? Because the *kernel*
unconditionally reads `Above[0..7]` from the row above the dst — the
kernel does not know it is on the right edge. The `Above[4..7]` slots
in those interior rows would otherwise contain samples from the wrong
macroblock (the right neighbour, whose contents are *not* a valid
prediction source — they belong to a future decode step). The copy
replaces them with the only meaningful proxy: a horizontal extension
of the above-right neighbour's bottom row. RFC 6386 §12.2 spells out
this convention as "samples beyond the right of the block are taken as
copies of the rightmost above-right sample" — diagonal-down propagated
all the way to the bottom of the MB.

The call lives at `decodeframe.c:164` and `threading.c:159`, run
exactly once per `B_PRED` MB right before the per-sub-block loop:

```c
intra_prediction_down_copy(xd, xd->recon_above[0] + 16);
```

The argument `xd->recon_above[0] + 16` is the source — the four bytes
of the above-right MB's bottom row. The helper writes the three
"interior" destination rows; the top-most row already contains those
bytes because they *are* the above-right MB's bottom row.

### Left samples come from the per-sub-block reconstruction

The other half of the chicken-and-egg disappears because of the
in-place reconstruction in the decoder loop above: when sub-block
`i = 5` (second row, second column) needs `L0..L3` from sub-block
`i = 4` (second row, first column), those samples *are* in `dst - 1`
because sub-block 4 has already had its residual added. The caller's
`yleft = dst - 1` pointer therefore always points into reconstructed
data, and `Above = dst - dst_stride` does the same for the row above.

## Definitions in this file

### `intra_pred_fn` — kernel function-pointer type

```c
typedef void (*intra_pred_fn)(uint8_t *dst, ptrdiff_t stride,
                              const uint8_t *above, const uint8_t *left);
```

**What.** The common signature shared by all ten per-mode kernels.
The same signature is used across VP8 *and* VP9; it is the contract
exported by `vpx_dsp/intrapred.c`.

**Why.** The four pointer-arguments are the minimal information a
predictor needs to write a 4×4 block: where the output goes
(`dst`/`stride`), and how to reach the L-shaped neighbour samples
(`above`, `left`). The `above` pointer is one-past-`top_left` so that
the kernel can index `above[-1]` to read the corner sample and
`above[0..7]` to read the row. The `left` pointer is the column read
top-to-bottom, with `left[0]` being the row aligned with `dst`'s first
row.

**Invariants.** `above` must point to memory in which `above[-1]`
through `above[7]` are addressable, and `left` to memory in which
`left[0..3]` are addressable (kernels on NEON / VSX may read more — see
`Left[8]` / `Left[16]` below). The kernels never write outside the 4×4
patch starting at `dst`.

**How used.** Every entry of the `pred[]` table has this type, and the
dispatcher calls `pred[b_mode](dst, dst_stride, Above, Left)` once per
sub-block.

### `pred[10]` — dispatch table indexed by `B_PREDICTION_MODE`

```c
static intra_pred_fn pred[10];
```

**What.** A file-private array of ten function pointers, one per
intra-only `B_PREDICTION_MODE` enum value (the enumeration in
`blockd.h:98` lists ten such values — `B_DC_PRED` through `B_HU_PRED`
— followed by four *inter-only* sub-modes `LEFT4X4 / ABOVE4X4 /
ZERO4X4 / NEW4X4` that are never indexed here; `VP8_BINTRAMODES` is
defined as `B_HU_PRED + 1 == 10` in `blockd.h:121` to capture exactly
that count).

**Why.** The table flattens what would otherwise be a ten-way `switch`
into one indirect call. Beyond avoiding the branch, this is also the
*indirection point at which CPU-specific kernels are wired in* — the
`vpx_dsp_rtcd` runtime-CPU-detection layer rewrites the
`vpx_*_predictor_4x4` symbols to NEON / SSE2 / VSX implementations on
the first call, and `vp8_init_intra4x4_predictors_internal()` then
captures the chosen pointer here. After init, the per-block call site
incurs nothing more than a load+jump.

**Invariants.** Only the first ten enum values are valid indices.
`pred[]` is written exactly once (by the init function below) and read
many times from per-MB code; the table is not protected by a lock
because the initialiser is itself called under `vpx_once`
(`reconintra.c:43`).

**How used.** `vp8_intra4x4_predict` performs `pred[b_mode](…)` at the
end of its body.

### `vp8_init_intra4x4_predictors_internal()` — one-shot wiring

```c
void vp8_init_intra4x4_predictors_internal(void) {
  pred[B_DC_PRED] = vpx_dc_predictor_4x4;
  pred[B_TM_PRED] = vpx_tm_predictor_4x4;
  pred[B_VE_PRED] = vpx_ve_predictor_4x4;
  pred[B_HE_PRED] = vpx_he_predictor_4x4;
  pred[B_LD_PRED] = vpx_d45e_predictor_4x4;
  pred[B_RD_PRED] = vpx_d135_predictor_4x4;
  pred[B_VR_PRED] = vpx_d117_predictor_4x4;
  pred[B_VL_PRED] = vpx_d63e_predictor_4x4;
  pred[B_HD_PRED] = vpx_d153_predictor_4x4;
  pred[B_HU_PRED] = vpx_d207_predictor_4x4;
}
```

**What.** Populates the dispatch table.

**Why.** The kernels themselves are in `vpx_dsp` because VP9 needs
them too; the *mapping* from VP8's enum to the (otherwise
codec-neutral) kernel names is VP8-specific and lives here. Note the
two-step naming convention used by `vpx_dsp`: directional modes are
named by their *angle in degrees from the horizontal* (45, 63, 117,
135, 153, 207). VP8's mode aliases (`LD`, `VL`, `VR`, `RD`, `HD`,
`HU`) decode to those same angles — see the per-mode subsections
below.

**Invariants.** Called exactly once per process. The function is
invoked from `vp8_init_intra_predictors_internal()` in
`reconintra.c:43`, which is itself wrapped in `vpx_once`. After this
returns, `pred[]` is constant for the program's lifetime.

**How used.** Called from `vp8_create_common`'s init path; the
per-block dispatcher assumes the table is populated.

A subtle implementation note: VP8 mode `B_LD_PRED` could in principle
map to either `vpx_d45_predictor_4x4` or `vpx_d45e_predictor_4x4`, and
`B_VL_PRED` to either `vpx_d63_predictor_4x4` or `vpx_d63e_predictor_4x4`.
The "e" variants (`d45e`, `d63e`) are the *VP8-bitstream-conformant*
edge-handling — they extend the final sample with `AVG3(G, H, H)` /
`AVG3(F, G, H)` rather than VP9's `= H` / `AVG2(E, F)`. Comments in
`intrapred.c` mark this: `// differs from vp8` at lines 313, 319, 364.
This is one of the rare places where VP8 and VP9 share an algorithm
*almost but not quite* identically, and the choice of `d45e` /
`d63e` here is the bit that keeps decoded pixels bit-exact with the
spec.

### `vp8_intra4x4_predict()` — the dispatcher

```c
void vp8_intra4x4_predict(unsigned char *above, unsigned char *yleft,
                          int left_stride, B_PREDICTION_MODE b_mode,
                          unsigned char *dst, int dst_stride,
                          unsigned char top_left);
```

**What.** Given pointers to the row above and the column to the left
of one 4×4 sub-block, plus the top-left corner sample and the chosen
intra mode, predict the 4×4 patch into `dst`.

**Why.** The function exists to *normalise* the boundary-sample layout
into the form the `vpx_dsp` kernels expect. Two normalisations
matter:

1. The kernels read `above[-1]` for the corner, but the caller may
   have constructed `Above = dst - dst_stride`, in which case
   `Above[-1]` would be the byte one column to the left of the
   top-left of the block — which is *not* the same pixel as the
   top-left corner (because of the in-place reconstruction order:
   the top-left corner of a sub-block in the *interior* of an MB
   comes from the bottom-right pixel of the diagonally-prior
   sub-block, while the byte at `dst - dst_stride - 1` could be
   stale data near picture borders). The caller therefore passes
   the corner sample explicitly as `top_left`, and the dispatcher
   stores it into the buffer slot that becomes `Above[-1]`.

2. The kernels read `left[0..3]` as four *contiguous* bytes, but in
   the frame buffer the left column is *strided* (one byte per
   scanline). The dispatcher copies the four samples into a local
   contiguous `Left[]` array.

**Invariants.** The caller guarantees that `above[0..7]` are
readable — the eight above-samples; on the right edge of an MB this
is only true because `intra_prediction_down_copy` has run.
`yleft[0]`, `yleft[left_stride]`, `yleft[2*left_stride]`, and
`yleft[3*left_stride]` must all be readable; on the left edge of the
picture they read the `129`-filled border installed by
`vp8_setup_intra_recon` (see `setupintrarecon.c:20`). The buffer
`Aboveb[]` has slot indices `-4..7` relative to `Above`, all writable.

**How used.** Called sixteen times per `B_PRED` macroblock from the
decoder's per-sub-block loop, once per 4×4 block in raster order, in
between the dequantise/IDCT/add step of the previous block and the
same step of the current block.

The body breaks into three phases:

```c
#if HAVE_VSX
  unsigned char Aboveb[20];
#else
  unsigned char Aboveb[12];
#endif
  unsigned char *Above = Aboveb + 4;
```

**Phase 1 — allocate a local buffer.** `Aboveb` is the backing
storage; `Above` is the pointer the kernel is given, offset by 4 so
that the kernel's `above[-1]` (top-left), `above[0..7]` (the row
above), and (on VSX) `above[8..15]` (overread slack) all sit within
`Aboveb`. On generic builds 12 bytes suffices: `[-4..-1]` are
spare-but-aligned, `[0..7]` hold the row. On VSX, 20 bytes are
reserved because `vec_vsx_ld` may read a full 16-byte vector starting
at `Above`; the extra 8 bytes past the end of the "real" data are
overread but unused, and the file's comment (`reconintra4x4.c:43–45`)
flags this explicitly.

```c
#if HAVE_NEON
  unsigned char Left[8];
  #if VPX_WITH_ASAN
    vp8_zero_array(Left, 8);
  #endif
#elif HAVE_VSX
  unsigned char Left[16];
#else
  unsigned char Left[4];
#endif
```

The same overread accommodation applies to `Left`. NEON's
narrow-load idioms cannot load 4 bytes (they load 8 or 16); VSX wants
a full vector. ASan-instrumented NEON builds explicitly zero the
overread slack because the optimiser cannot prove the read is unused.
On generic builds, `Left[4]` is all that's needed.

```c
Left[0] = yleft[0];
Left[1] = yleft[left_stride];
Left[2] = yleft[2 * left_stride];
Left[3] = yleft[3 * left_stride];
memcpy(Above, above, 8);
Above[-1] = top_left;
```

**Phase 2 — gather neighbour samples.** The four strided left samples
are copied into the contiguous `Left[]`. The eight contiguous above
samples (`A0..A7` in the RFC's notation) are copied with `memcpy`.
Finally the explicitly-passed `top_left` is stored at `Above[-1]`,
overwriting whatever happened to be in `Aboveb[3]` (this is the slot
just before `Above[0]`).

The eight bytes copied — not four — is the silent commitment to the
fact that modes `B_LD_PRED` and `B_VL_PRED` need `A4..A7` and that
the caller has already arranged for those bytes to contain valid
above-right samples (via `intra_prediction_down_copy` at the start
of the MB).

```c
pred[b_mode](dst, dst_stride, Above, Left);
```

**Phase 3 — invoke the kernel.** Single indirect call. The kernel
writes 16 bytes (4 rows × 4 bytes) at `dst[r*dst_stride + c]`.

## The ten `B_PRED` sub-modes

Each entry traces VP8's enum name → angle → kernel → the geometry of
what it computes. The angles are measured from the positive
horizontal axis (so 0° / 180° are horizontal and 90° is vertical),
and the names of the underlying `vpx_dsp` kernels embed these angles
directly (`d135` = 135°, etc.). The reference is RFC 6386 §12.2.

### `B_DC_PRED` — DC mode (mode 0)

Kernel: `vpx_dc_predictor_4x4`.

All sixteen output samples are set to a single constant: the rounded
average of all four above-samples `A0..A3` and all four left-samples
`L0..L3`:

```
DC = (A0+A1+A2+A3 + L0+L1+L2+L3 + 4) >> 3
```

This is the fallback mode used when no clear directional structure is
visible in the neighbours. Distinct from VP8's MB-level `DC_PRED`,
the 4×4 variant *always* averages over both edges; it does not have
`DC_TOP`, `DC_LEFT`, `DC_128` variants — because at this granularity
neighbours are always available, courtesy of the `127` / `129`
boundary fill installed at frame allocation (RFC 6386 §12.1 last
paragraph).

### `B_TM_PRED` — TrueMotion mode (mode 1)

Kernel: `vpx_tm_predictor_4x4`.

TrueMotion was Duck's name (inherited from VP3/VP7) for the "planar"
predictor: each output sample `dst(r,c) = clip(L_r + A_c - P)` where
`P` is the top-left corner. The intuition is that a smooth gradient
between the top-left corner, the above edge, and the left edge can be
extrapolated bilinearly into the block. It works well on smooth
shaded regions.

### `B_VE_PRED` — vertical (with three-tap smoothing), mode 2

Kernel: `vpx_ve_predictor_4x4` (vpx_dsp/intrapred.c:264).

Computes a single one-dimensional row by three-tap averaging of the
above-samples (with `H = above[-1] = top_left`):

```
row[0] = AVG3(H, I, J);          // I=A0, J=A1
row[1] = AVG3(I, J, K);          // K=A2
row[2] = AVG3(J, K, L);          // L=A3
row[3] = AVG3(K, L, M);          // M=A4
```

then replicates that row down through all four rows of the block.
Compared to a literal "copy above-row down" predictor (VP9's
`V_PRED`), the 3-tap smoothing damps high-frequency aliasing along
the vertical direction.

### `B_HE_PRED` — horizontal (with three-tap smoothing), mode 3

Kernel: `vpx_he_predictor_4x4` (intrapred.c:250).

Dual of `B_VE_PRED`: a single column is built by 3-tap averaging of
the left-samples (with `H = above[-1]`):

```
col[0] = AVG3(H,  L0, L1);
col[1] = AVG3(L0, L1, L2);
col[2] = AVG3(L1, L2, L3);
col[3] = AVG3(L2, L3, L3);       // edge: duplicate L3
```

then replicated across all four columns.

### `B_LD_PRED` — diagonal "left-down", 45°, mode 4

Kernel: `vpx_d45e_predictor_4x4` (intrapred.c:367).

A 45°-from-horizontal diagonal predictor: all eight above-samples
`A0..A7` participate; the diagonal stripes run from upper-right to
lower-left. Each output is a 3-tap average along the
diagonal — e.g. `dst(0,0) = AVG3(A0, A1, A2)`, `dst(0,1) = dst(1,0) =
AVG3(A1, A2, A3)`, and so on, with the bottom-right corner clamped
to `AVG3(G, H, H)`. This mode is *the* reason
`intra_prediction_down_copy` exists: it is the one that reads
`A4..A7`.

### `B_RD_PRED` — diagonal "right-down", 135°, mode 5

Kernel: `vpx_d135_predictor_4x4` (intrapred.c:411).

A 135°-from-horizontal predictor: diagonal stripes run from
upper-left to lower-right. Requires `L0..L3`, `P` (= `above[-1]`),
and `A0..A3` — the entire L-shape but not the above-right. The
samples along the anti-diagonal `dst(0,0), dst(1,1), dst(2,2),
dst(3,3)` are all equal to `AVG3(A0, P, L0)`, and the eight other
diagonals are each filled with the matching 3-tap average.

### `B_VR_PRED` — vertical-right, 117°, mode 6

Kernel: `vpx_d117_predictor_4x4` (intrapred.c:388).

A nearly-vertical predictor at 117° (i.e. tilted 27° to the right of
vertical). The even rows use 2-tap averages of `P, A0, A1, A2, A3`
across the row; the odd rows use 3-tap averages mixing in the
left-column. The block is filled with stripes that descend
predominantly downward but lean rightward.

### `B_VL_PRED` — vertical-left, 63°, mode 7

Kernel: `vpx_d63e_predictor_4x4` (intrapred.c:322).

A nearly-vertical predictor at 63° (tilted 27° to the left of
vertical). Uses only above-samples `A0..A7`; like `B_LD_PRED` it
relies on the down-copied above-right pixels. The `e` suffix on the
kernel name flags the VP8-spec-conformant edge handling that
distinguishes it from VP9's `vpx_d63_predictor_4x4`.

### `B_HD_PRED` — horizontal-down, 153°, mode 8

Kernel: `vpx_d153_predictor_4x4` (intrapred.c:432).

A nearly-horizontal predictor at 153° (tilted 27° below horizontal).
Uses `L0..L3`, `P`, `A0..A2`. Even columns use 2-tap averages of
the left column; odd columns use 3-tap averages mixing in `P` and
the above-row.

### `B_HU_PRED` — horizontal-up, 207°, mode 9

Kernel: `vpx_d207_predictor_4x4` (intrapred.c:283).

A nearly-horizontal predictor at 207° (i.e. mirror of 153°, tilted
27° *above* horizontal toward the lower-left). Uses only `L0..L3`,
because the predictor runs from the left column outward and downward;
the entire bottom row is filled with `L3` (the bottom-most
left-sample), reflecting that the diagonal has "run off" the
left-column source data.

## The shape of the dispatch as a whole

Reading the file end-to-end, the discipline is:

1. **Once per process:** `vp8_init_intra4x4_predictors_internal()`
   captures function pointers from the RTCD-resolved `vpx_dsp`
   kernels into `pred[]`.
2. **Once per MB (in `B_PRED` only):** the decoder calls
   `intra_prediction_down_copy()` to pre-stage above-right samples for
   the right-edge sub-blocks (helper in `reconintra4x4.h`).
3. **Sixteen times per MB:** the per-sub-block decoder loop calls
   `vp8_intra4x4_predict()`, which copies the L-shape into a local
   buffer, then invokes `pred[b_mode]` to write the 4×4 prediction.
4. **In between each call:** the decoder dequantises that block's
   coefficients, runs the 4×4 IDCT, and adds the residual into the
   prediction *in place*, so that the next call's `Above` /
   `yleft` pointers read the just-reconstructed neighbours.

That last property — reconstruction *between* predictions of
neighbouring sub-blocks — is what makes `B_PRED` a genuine
finer-grained intra mode, and what makes this file's API
deliberately minimal: it owns no state, it depends on no `MACROBLOCKD`
field beyond what the caller has already dereferenced, and it can
therefore live unchanged in the multi-threaded decoder
(`vp8/decoder/threading.c:192`) and in the encoder's mode-search code
(`vp8/encoder/rdopt.c:553`, `pickinter.c:196`, `encodeintra.c:54`)
without modification.
