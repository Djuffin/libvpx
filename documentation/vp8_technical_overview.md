# libvpx VP8 Decoder — Technical Overview

This document describes the architecture of the VP8 decoder as it is
implemented inside libvpx, restricted to the set of source files
enumerated in `vp8_files.md` (a minimal, pure-C, single-threaded,
no-postproc, no-error-concealment, decoder-only build). All file paths
are relative to the repository root. All line numbers refer to the tree
as of this writing.

The reader is assumed to have a working knowledge of the VP8 bitstream
format (RFC 6386), but not of libvpx's internal layering.

---

## Table of contents

1. [Big-picture pipeline](#1-big-picture-pipeline)
2. [Decoder API](#2-decoder-api)
3. [Key data structures](#3-key-data-structures)
4. [Block partitioning](#4-block-partitioning)
5. [Entropy coding (bool decoder, tree codes)](#5-entropy-coding-bool-decoder-tree-codes)
6. [Frame header & frame partitions](#6-frame-header--frame-partitions)
7. [Intra-prediction](#7-intra-prediction)
8. [Inter-prediction](#8-inter-prediction)
9. [Residuals (tokens, dequant, IDCT, Walsh)](#9-residuals-tokens-dequant-idct-walsh)
10. [Decoded picture buffer & reference management](#10-decoded-picture-buffer--reference-management)
11. [Deblocking (loop filter)](#11-deblocking-loop-filter)
12. [Memory management](#12-memory-management)
13. [Threading](#13-threading)
14. [Runtime CPU dispatch (RTCD)](#14-runtime-cpu-dispatch-rtcd)
15. [Error handling](#15-error-handling)
16. [Bitstream layout (wire format)](#16-bitstream-layout-wire-format)

---

## 1. Big-picture pipeline

A single decoded frame walks through the following stages, in order:

```
caller buffer
   │
   ▼
vpx_codec_decode()  ─►  vp8_decode()  (vp8_dx_iface.c)
   │                                          │
   │                            vp8dx_receive_compressed_data()
   │                                          │
   │                                          ▼
   │                              vp8_decode_frame()  (decodeframe.c)
   │                                          │
   │  ┌────────────── per-frame ──────────────┤
   │  │  1. parse uncompressed header (frame tag, dims, lf params, …)
   │  │  2. open arithmetic decoder for the residual / "first" partition
   │  │  3. parse segmentation, loop-filter, quantizer, MV/coef-prob updates
   │  │  4. carve token partitions (1..8), one bool decoder per partition
   │  │  5. macroblock decode loop (raster order):
   │  │        ├── decode_mb_mode_mv()            (decodemv.c)
   │  │        ├── decode_mb_tokens()             (detokenize.c)
   │  │        ├── intra OR inter predictors      (reconintra*, reconinter*)
   │  │        ├── dequant + IDCT + accumulate    (idct_blk, idctllm)
   │  │        └── (if multithreaded) deblock row
   │  │  6. in-loop deblocking (whole frame in single-thread builds)
   │  │  7. border extension on the reconstructed YV12
   │  │  8. swap_frame_buffers() — rotate LAST/GOLDEN/ALTREF slots
   │  └──────────────────────────────────────────┘
   │                                          │
   ▼                                          ▼
vpx_codec_get_frame()  ◄────── frame_to_show wrapped in vpx_image_t
```

Everything inside the per-frame block runs on the calling thread in a
`--disable-multithread` build.

---

## 2. Decoder API

### 2.1 Public surface

The caller uses six entry points from `vpx/vpx_decoder.h` (which
re-exports `vpx/vpx_codec.h`):

| Function                          | Purpose                                   |
|-----------------------------------|-------------------------------------------|
| `vpx_codec_vp8_dx()`              | Return the VP8 decoder vtable.            |
| `vpx_codec_dec_init_ver()`        | Construct a decoder context.              |
| `vpx_codec_peek_stream_info()`    | Probe one frame's header without decoding.|
| `vpx_codec_decode()`              | Feed one access unit into the decoder.    |
| `vpx_codec_get_frame()`           | Iterate decoded frames.                   |
| `vpx_codec_destroy()`             | Tear down.                                |

`vpx_codec_vp8_dx()` (vp8/vp8_dx_iface.c:744) returns
`&vpx_codec_vp8_dx_algo`, a static `vpx_codec_iface_t` whose
function-pointer table (vp8/vp8_dx_iface.c:738–765) is:

```c
{
  .name      = "WebM Project VP8 Decoder",
  .caps      = VPX_CODEC_CAP_DECODER
             | VP8_CAP_POSTPROC               /* = VPX_CODEC_CAP_POSTPROC
                                                  iff CONFIG_POSTPROC,
                                                  else 0                  */
             | VP8_CAP_ERROR_CONCEALMENT      /* = VPX_CODEC_CAP_ERROR_CONCEALMENT
                                                  iff CONFIG_ERROR_CONCEALMENT,
                                                  else 0                  */
             | VPX_CODEC_CAP_INPUT_FRAGMENTS,
  .init      = vp8_init,
  .destroy   = vp8_destroy,
  .ctrl_maps = vp8_ctf_maps,
  .dec       = { vp8_peek_si, vp8_get_si, vp8_decode, vp8_get_frame, /*…*/ },
}
```

`VP8_CAP_POSTPROC` and `VP8_CAP_ERROR_CONCEALMENT` are macros that
collapse to 0 when the relevant `CONFIG_*` is off (vp8_dx_iface.c:34–36).
In the minimal `--disable-postproc --disable-error-concealment` build
the runtime caps reduce to
`VPX_CODEC_CAP_DECODER | VPX_CODEC_CAP_INPUT_FRAGMENTS`.

The dispatcher in `vpx/src/vpx_codec.c` and `vpx/src/vpx_decoder.c`
takes any `vpx_codec_ctx_t *` and forwards through `ctx->iface->...`,
so VP8 and VP9 share the same caller-facing API.

### 2.2 Control IDs

Decoder-specific knobs go through `vpx_codec_control()`, which walks
`vp8_ctf_maps[]` (vp8/vp8_dx_iface.c:723–732):

| Control ID                          | Action                              |
|-------------------------------------|-------------------------------------|
| `VP8_SET_REFERENCE` / `VP8_COPY_REFERENCE` | Replace or read a ref slot.   |
| `VP8_SET_POSTPROC`                  | Post-processing flags (compiled out here). |
| `VP8D_GET_LAST_REF_UPDATES`         | Which of LAST/GOLDEN/ALT was refreshed. |
| `VP8D_GET_FRAME_CORRUPTED`          | Was the last frame corrupted?       |
| `VP8D_GET_LAST_REF_USED`            | Which references the last frame used.   |
| `VPXD_GET_LAST_QUANTIZER`           | Base quantizer of last frame.       |
| `VPXD_SET_DECRYPTOR`                | Install a per-byte decrypt callback used by the bool decoder. |

### 2.3 Lifecycle

```
vpx_codec_dec_init_ver()
  └─► iface->init()  =  vp8_init()                       [vp8_dx_iface.c]
         └─► allocate vpx_codec_alg_priv_t (ctx->priv)
             (does NOT yet allocate VP8D_COMP / VP8_COMMON;
              those wait until the first vp8_decode() call so that
              the frame dimensions are known)

vpx_codec_decode()
  └─► iface->dec.decode()  =  vp8_decode()
         ├─► (1st time)  vp8_create_decoder_instances()
         │                 └─► create_decompressor()
         │                       ├─► allocate VP8D_COMP
         │                       ├─► setjmp(common.error.jmp)
         │                       ├─► once(initialize_dec)   ← RTCD init
         │                       └─► vp8_create_common()
         ├─► vp8_peek_si_internal()   – parse frame tag, maybe resize
         ├─► vp8_alloc_frame_buffers() – on dimension change
         └─► vp8dx_receive_compressed_data() → vp8_decode_frame()

vpx_codec_get_frame()
  └─► iface->dec.get_frame()  =  vp8_get_frame()
         └─► yuvconfig2image(cm->frame_to_show, …) → vpx_image_t *

vpx_codec_destroy()
  └─► iface->destroy()  =  vp8_destroy()
         └─► vp8_remove_decoder_instances() + free priv
```

The first `vp8_decode()` call is special: it allocates everything
because VP8 carries its width/height in the keyframe payload, so
allocation cannot happen at `init` time.

---

## 3. Key data structures

VP8's state is layered in three nested structures:

```
vpx_codec_alg_priv_t        (vp8/vp8_dx_iface.c:44)
   │
   └── frame_buffers.pbi[]  (array because the threaded "frame-MT" build
                             can decode multiple frames in flight; the
                             single-thread minimal build uses pbi[0])
            │
            ▼
       VP8D_COMP             (vp8/decoder/onyxd_int.h:59)
         ├── MACROBLOCKD  mb               (per-MB working context)
         ├── VP8_COMMON   common           (per-frame, per-sequence state)
         ├── vp8_reader   mbc[MAX_PARTITIONS]   (bool decoders, MAX=9)
         ├── FRAGMENT_DATA fragments       (input partition pointers/sizes)
         ├── VP8D_CONFIG  oxcf             (resolution, postproc bits, …)
         └── decrypt_cb / decrypt_state    (optional bytestream decryptor)
            │
            ▼
       VP8_COMMON           (vp8/common/onyxc_int.h:62)
         ├── width / height / horiz_scale / vert_scale
         ├── mb_rows / mb_cols / mode_info_stride (= mb_cols + 1)
         ├── YV12_BUFFER_CONFIG yv12_fb[NUM_YV12_BUFFERS]   (NUM=4)
         ├── int new_fb_idx, lst_fb_idx, gld_fb_idx, alt_fb_idx
         ├── int fb_idx_ref_cnt[NUM_YV12_BUFFERS]
         ├── MODE_INFO  *mip / *mi / *prev_mi / *prev_mip
         ├── FRAME_CONTEXT  lfc, fc           (entropy probs: prev / current)
         ├── loop_filter_info_n lf_info
         ├── filter_level / sharpness_level / filter_type / …
         ├── refresh_last_frame / refresh_golden_frame / refresh_alt_ref_frame
         ├── copy_buffer_to_gf / copy_buffer_to_arf
         ├── multi_token_partition (0..3 → 1,2,4,8 token partitions)
         └── vpx_internal_error_info error    (setjmp/longjmp target)
```

### 3.1 MACROBLOCKD

`MACROBLOCKD` (vp8/common/blockd.h, around line 209) is the working
state of one macroblock being decoded:

- `BLOCKD block[25]` — 16 Y + 4 U + 4 V + 1 Y2; see §4.
- `MODE_INFO *mode_info_context` — points at the current MB's slot in
  the `mi` grid. Indexing `mode_info_context[-1]` reaches the left
  neighbor, `mode_info_context[-mode_info_stride]` the one above.
- `dst.y_buffer / u_buffer / v_buffer` — destination pointers into the
  current YV12 frame, advanced as the raster walks.
- `pre.y_buffer / u_buffer / v_buffer` — source pointers into the
  selected reference YV12 buffer (for inter MBs).
- `predictor[384]` — scratch area for the per-MB predictor before
  residuals are added (16x16 Y + 8x8 U + 8x8 V = 384 bytes).
- `qcoeff[400] / dqcoeff[400] / eobs[25]` — per-block coefficient
  storage (400 = 25 × 16 coefficients).
- Per-edge distance fields `mb_to_left/right/top/bottom_edge` (in
  1/8-pel units) used by MV clamping.
- Segmentation: `mb_segment_tree_probs[3]`,
  `segment_feature_data[2][4]`, `mb_segment_abs_delta`.
- Loop-filter delta storage `ref_lf_deltas[4]`, `mode_lf_deltas[4]`
  (see §11).

### 3.2 MODE_INFO grid

`MODE_INFO` (vp8/common/blockd.h:156) is the bitstream-side per-MB
record:

```c
typedef struct modeinfo {
  MB_MODE_INFO     mbmi;       /* MB-level mode (mode, uv_mode, ref_frame,
                                  mv, partitioning, segment_id,
                                  mb_skip_coeff, need_to_clamp_mvs, … )   */
  union b_mode_info bmi[16];   /* per-4x4 sub-mode/MV, used only when
                                  mbmi.mode is B_PRED or SPLITMV          */
} MODE_INFO;
```

The grid is allocated as `(mb_cols + 1) × (mb_rows + 1)` and `mi` is
offset by `(stride + 1)` so that the entries with negative row or column
indices are valid sentinel slots. Neighbor lookups
(`mi[-1]`, `mi[-stride]`, `mi[-stride-1]`) therefore never need bounds
checks (alloccommon.c:94–100).

`prev_mi` / `prev_mip` (previous frame's grid, used by error
concealment) are declared **only** under `#if CONFIG_ERROR_CONCEALMENT`
(onyxc_int.h:122–124); the matching deallocation lives at
alloccommon.c:46–49, also gated by `CONFIG_ERROR_CONCEALMENT`, and
allocation happens in `vp8_dx_iface.c` on the EC path. The minimal
`--disable-error-concealment` build targeted by this document does
not even have these fields on `VP8_COMMON`.

### 3.3 BLOCKD

`BLOCKD` (vp8/common/blockd.h:193) is the per-4x4-block working
context inside the MB. Each of the 25 entries carries:

- `qcoeff` / `dqcoeff` — `short *` pointers (set up by `mbpitch.c`)
  pointing into the per-MB coefficient arrays at the right 16-coefficient
  slot.
- `eob` — `char *` pointer to the block's end-of-block index inside
  the per-MB `eobs[25]` array.
- `predictor` — `unsigned char *` pointer into the MB's predictor
  scratch.
- `offset` — pixel offset from the MB's top-left corner inside the
  destination plane (precomputed by `vp8_build_block_doffsets`,
  vp8/common/mbpitch.c:43).
- `bmi` — reference back to the relevant `bmi[]` entry (used when the
  MB uses per-4x4 modes).

---

## 4. Block partitioning

### 4.1 Macroblock decomposition

VP8 only knows one block size for the transform stage: 4x4. A 16x16
macroblock is statically decomposed into the 25 4x4 blocks indexed by
`BLOCKD[0..24]`:

```
Luma (16x16, blocks 0..15, raster order inside the MB)
   ┌────┬────┬────┬────┐
   │  0 │  1 │  2 │  3 │
   ├────┼────┼────┼────┤
   │  4 │  5 │  6 │  7 │
   ├────┼────┼────┼────┤
   │  8 │  9 │ 10 │ 11 │
   ├────┼────┼────┼────┤
   │ 12 │ 13 │ 14 │ 15 │
   └────┴────┴────┴────┘

U (8x8, blocks 16..19)       V (8x8, blocks 20..23)
   ┌────┬────┐                 ┌────┬────┐
   │ 16 │ 17 │                 │ 20 │ 21 │
   ├────┼────┤                 ├────┼────┤
   │ 18 │ 19 │                 │ 22 │ 23 │
   └────┴────┘                 └────┴────┘

Y2 (DC-only, block 24): a 4x4 transform whose 16 input
samples are the DCs of Y blocks 0..15.
```

`vp8_block2left[25]` and `vp8_block2above[25]` (vp8/common/blockd.c:14)
encode each block's position inside the MB and are used for
neighbor-mode lookups during entropy coding.

### 4.2 Y2 — the second-order Walsh transform

When the MB is **not** using `B_PRED` (i.e., its luma is predicted as a
single 16x16 block) **and** the inter mode is not `SPLITMV`, the DCs of
the 16 Y blocks form a 4x4 array that is further transformed with a
Walsh–Hadamard (the so-called "Y2" or "DC" block).

- During encoding: each Y block's DC is replaced by 0; the 16 DCs are
  routed through an extra Walsh; that 4x4 set of WHT coefficients is
  what gets coded into block 24.
- During decoding: block 24 is decoded first; if its EOB is non-empty,
  `vp8_short_inv_walsh4x4` (vp8/common/idctllm.c:127) runs the inverse
  Walsh into 16 dequantized DCs; those are scattered back into
  `dqcoeff[i*16]` for `i = 0..15` before the per-Y-block IDCTs run
  (invtrans.h:36).

Block 24 is also separately quantized (with the `y2dc_delta` /
`y2ac_delta` factors — see §9.4).

### 4.3 Frame partitioning

Spatially the picture is just `mb_rows × mb_cols` macroblocks with no
slicing. The "partitions" in VP8 are a **bitstream** concept, not a
spatial one (see §6).

---

## 5. Entropy coding (bool decoder, tree codes)

### 5.1 The arithmetic decoder

VP8 uses a binary arithmetic coder; per-bit probabilities are 8-bit
integers in `[1, 255]`. The decoder lives in vp8/decoder/dboolhuff.[ch]:

```c
typedef struct {
  const unsigned char *user_buffer_end;
  const unsigned char *user_buffer;
  VP8_BD_VALUE         value;
  int                  count;
  unsigned int         range;
  vpx_decrypt_cb       decrypt_cb;
  void                *decrypt_state;
} BOOL_DECODER;                          /* dboolhuff.h:36 */
```

`VP8_BD_VALUE` is `size_t` (dboolhuff.h:27) — 64 bits on a 64-bit host,
so the decoder loads up to 7 bytes of look-ahead into `value` and shifts
them out as it normalises.

Per-bit decode (inline `vp8dx_decode_bool` in dboolhuff.h:54–91):

```c
split    = 1 + (((range - 1) * prob) >> 8);
bigsplit = (VP8_BD_VALUE)split << (VP8_BD_VALUE_SIZE - 8);
if (value >= bigsplit) {
    bit    = 1;
    range -= split;
    value -= bigsplit;
} else {
    bit    = 0;
    range  = split;
}
shift  = vp8_norm[range];                /* leading-zero LUT */
range <<= shift;
value <<= shift;
count  -= shift;
if (count < 0) vp8dx_bool_decoder_fill();    /* refill from user_buffer */
```

`vp8_norm[256]` is a 256-entry lookup with the number of leading zeros
needed to renormalise `range` back to the top of the byte
(vp8/common/entropy.c:18).

Helpers built on top:
- `vp8_decode_value(br, n)` — read `n` literal bits at p=128 (uniform).
- `vp8_treed_read(br, tree, probs)` — walk a tree code (§5.2).

The optional decrypt callback (`vpx_decrypt_cb`) is invoked inside
`vp8dx_bool_decoder_fill` to lazily decrypt input bytes as they enter
the decoder, supporting DRM use cases.

### 5.2 Tree codes

Most non-coefficient syntax elements (modes, partition types, MV trees,
…) are coded with the libvpx tree-code format: an `int8_t` array where
positive entries point to the next node (within the same array),
negative entries are leaf-symbol values:

```c
const vp8_tree_index vp8_coef_tree[22] = {        /* entropy.c:70 */
  -DCT_EOB_TOKEN,  2,     /* root: bit 0 ⇒ EOB,   bit 1 ⇒ next      */
  -ZERO_TOKEN,     4,
  -ONE_TOKEN,      6,
   8, 12,                 /* node 3: LOW vs HIGH                    */
  -TWO_TOKEN,      10,
  -THREE_TOKEN,    -FOUR_TOKEN,
  14, 16,                 /* high-category sub-tree                 */
  -DCT_VAL_CATEGORY1,  -DCT_VAL_CATEGORY2,
  18, 20,
  -DCT_VAL_CATEGORY3,  -DCT_VAL_CATEGORY4,
  -DCT_VAL_CATEGORY5,  -DCT_VAL_CATEGORY6,
};
```

`vp8_treed_read` (vp8/decoder/treereader.h:30) walks this:

```c
i = 0;
while ((i = t[i + vp8_read(r, p[i >> 1])]) > 0) { /* keep walking */ }
return -i;       /* leaf */
```

Probabilities `p[]` are addressed by **node number** (`i >> 1`) so the
caller passes one probability per non-leaf node. All mode/MV/coef
tables in `vp8/common/entropymode.[ch]` and `entropy.[ch]` follow this
convention.

---

## 6. Frame header & frame partitions

### 6.1 Uncompressed header

`vp8_decode_frame` (vp8/decoder/decodeframe.c:879) starts by parsing the
3-byte frame tag (RFC 6386 §9.1):

```
byte 0   bit 0     frame_type      (0 = keyframe, 1 = interframe)
         bits 1-3  version
         bit  4    show_frame
         bits 5-7  first_partition_length_in_bytes  bits 0-2
byte 1            first_partition_length_in_bytes  bits 3-10
byte 2            first_partition_length_in_bytes  bits 11-18

(keyframe only, immediately following the 3-byte frame tag:)
byte 3-5          sync code  0x9d 0x01 0x2a
byte 6 + byte 7   16-bit width  (low 14 bits) + 2-bit horiz_scale
byte 8 + byte 9   16-bit height (low 14 bits) + 2-bit vert_scale
```

The decoder records `pc->Width`, `pc->Height`, `pc->horiz_scale`,
`pc->vert_scale`. A change triggers reallocation of MB grids and frame
buffers (`vp8_alloc_frame_buffers`).

### 6.2 Compressed header (residual / "first" partition)

After the uncompressed header, the **first partition** is opened with
`vp8dx_start_decode` and used to parse everything except per-MB
coefficients:

1. Color space + clamp type (keyframe only).
2. Segmentation enable / update / per-segment data and tree probs
   (decodeframe.c ~ lines 987–1034).
3. Loop-filter type, level, sharpness, and mode/ref deltas
   (decodeframe.c:1037–1074) — see §11.
4. `multi_token_partition` (2 bits): log2 of the number of token
   partitions (so 1, 2, 4, or 8). The first-partition decoder is the
   "residual" partition; the others carry only coefficients
   (decodeframe.c:742).
5. Quantizer base index + DC/AC per-component deltas
   (`y1dc_delta_q`, `y2dc_delta_q`, `y2ac_delta_q`, `uvdc_delta_q`,
   `uvac_delta_q`).
6. Reference-frame refresh flags (`refresh_last`,
   `refresh_golden_frame`, `refresh_alt_ref_frame`) and the
   `copy_buffer_to_gf` / `copy_buffer_to_arf` selectors.
7. `refresh_entropy_probs` — whether the per-frame coefficient
   probability updates are persistent.
8. Coefficient-probability updates: for every node of the coefficient
   tree at every context, a 1-bit "update?" with fixed prob; if 1,
   read a literal 8-bit new probability (decodeframe.c:1175–1187).
9. `mb_no_coeff_skip` flag + the corresponding `prob_skip_false`.
10. MV-probability updates (entropy-update of the MV trees).

### 6.3 Token partitions

`setup_token_decoder` (decodeframe.c:728) splits the remaining bytes
into `1 << multi_token_partition` token partitions. Sizes for all but
the last are encoded as 3-byte little-endian fields in the residual
partition; the last partition's size is implicit. Each partition is
initialized as an independent `vp8_reader`:

```
pbi->mbc[8]                   ← residual / first partition
pbi->mbc[0 .. N-1]            ← N token partitions
```

The libvpx decoder stores the residual partition's bool reader in the
last slot of `pbi->mbc[]` and the N token partitions in `pbi->mbc[0..N-1]`.
MB rows are assigned to token partitions in a round-robin fashion: row
`r` reads its coefficients from `mbc[r % N]`. That layout is exactly
what enables row-level multi-threading: one worker per partition, no
synchronisation needed on coefficient decoding because the
per-partition bool decoders are disjoint.

The total partition count fits in `MAX_PARTITIONS = 9`
(vp8/common/onyxc_int.h:38): up to 8 token partitions plus the residual
partition in `mbc[8]`.

---

## 7. Intra-prediction

### 7.1 Prediction modes

Macroblock-level intra modes (vp8/common/blockd.h:65–79):

```c
typedef enum {
  DC_PRED,     /* mean of above-row & left-column            */
  V_PRED,      /* replicate above row                        */
  H_PRED,      /* replicate left column                      */
  TM_PRED,     /* "TrueMotion": p[i,j] = L[i] + A[j] - TL    */
  B_PRED,      /* per-4x4 sub-modes (luma only)              */
  NEAREST_MV, NEAR_MV, ZERO_MV, NEW_MV, SPLIT_MV
} MB_PREDICTION_MODE;
```

Per-4x4 luma sub-modes used under `B_PRED` (10 intra modes plus 4
inter-only sub-modes, vp8/common/blockd.h:98-119):

```
B_DC_PRED   B_TM_PRED
B_VE_PRED   B_HE_PRED
B_LD_PRED   B_RD_PRED   B_VR_PRED   B_VL_PRED   B_HD_PRED   B_HU_PRED
```

The four MB-level modes also apply to the 8x8 chroma planes, selected
independently by `mbmi.uv_mode`.

### 7.2 Boundary samples

VP8 intra modes read one row of samples above the block (extended
one block to the right for `B_LD_PRED`), one column to the left, and
the single pixel above-left. At the picture boundary those samples are
*not* clipped — they are replaced by constants:

- `127` above (vp8/common/setupintrarecon.c:18),
- `129` to the left (line 20),
- `127` at the top-left corner.

This is what `vp8_setup_intra_recon` writes into the byte that
precedes each row of every plane at frame allocation time. The
asymmetry (127/129) ensures that for DC-only neighbors the average
rounds to 128 — the neutral grey VP8 uses when nothing is decoded yet.

### 7.3 Dispatch

For MB-level intra:

```
vp8_build_intra_predictors_mby_s()    16x16 Y predictor → predictor[]
vp8_build_intra_predictors_mbuv_s()   8x8 U+V predictors → predictor[]
```

These dispatch through function-pointer tables indexed by mode (and by
"left-available × above-available" for DC variants). The actual
predictors live in `vpx_dsp/intrapred.c` — generic-C implementations
generated by the `intra_pred_allsizes` macro (intrapred.c:963–989):
e.g. `vpx_dc_predictor_16x16_c`, `vpx_dc_top_predictor_16x16_c`,
`vpx_dc_left_predictor_16x16_c`, `vpx_dc_128_predictor_16x16_c`,
`vpx_v_predictor_16x16_c`, `vpx_h_predictor_16x16_c`,
`vpx_tm_predictor_16x16_c`.

For `B_PRED`, the decoder reads per-4x4 modes one at a time and calls
`vp8_intra4x4_predict` (vp8/common/reconintra4x4.c:39) per block:

```c
unsigned char Aboveb[12];          /* TL + 11 above (some modes need
                                      4 extra to the right)            */
Aboveb[3]    = top_left;
Above        = Aboveb + 4;
memcpy(Above, above_row, 8);
Left[0..3]   = left_col[0, stride, 2*stride, 3*stride];
pred[b_mode](dst, dst_stride, Above, Left);
```

Order matters: each 4x4 block is decoded, predicted, **and
reconstructed** (residual added in place) before the next 4x4 block is
predicted, because the next prediction will read the just-reconstructed
samples as its neighbors.

### 7.4 Entropy context for B_PRED modes

The mode of each 4x4 block is itself entropy-coded with a context
formed by the left and above neighbors' modes:

```c
A = above_block_mode(mi, i, mi_stride);   /* findnearmv.h:128 */
L = left_block_mode(mi,  i);              /* findnearmv.h:110 */
mi->bmi[i].as_mode = read_bmode(bc, vp8_kf_bmode_prob[A][L]);
```

(decodemv.c:54–57). When the neighbor MB has a different MB-level
mode, that mode is mapped to its B-mode equivalent (`DC_PRED → B_DC_PRED`,
etc.) so that the conditional probability table
`vp8_kf_bmode_prob[10][10][9]` (entropymode.h:74) is always
well-defined.

---

## 8. Inter-prediction

### 8.1 References and MB inter modes

Each inter MB selects a reference via `mbmi.ref_frame ∈
{LAST_FRAME, GOLDEN_FRAME, ALTREF_FRAME}` and a motion mode
(blockd.h:72–76):

| Mode          | Meaning                                              |
|---------------|------------------------------------------------------|
| `NEAREST_MV`  | Use the most-frequent neighbor MV; no MV delta sent. |
| `NEAR_MV`     | Use the second-most-frequent neighbor MV; no delta.  |
| `ZERO_MV`     | (0, 0).                                              |
| `NEW_MV`      | Decode an MV delta added to the best-near predictor. |
| `SPLIT_MV`    | Split the MB into 2/4/8/16 sub-blocks with own MVs.  |

`vp8_find_near_mvs` (vp8/common/findnearmv.c:23) computes the
`nearest / near / best` predictors and a histogram of neighbor MV
classes (intra / nearest / near / splitmv) used to derive the
`mv_ref_p[]` probabilities through which the inter mode itself is
decoded (`vp8_mv_ref_probs`, findnearmv.c:150, looking into
`vp8_mode_contexts`, modecont.c:13–26).

The neighbors considered are: the MB above, the MB to the left, and
the MB above-left, in that order, with **sign-bias correction**: if
the candidate reference has opposite sign-bias from the current
reference, the candidate MV is negated before contributing to the
histogram (findnearmv.h:24). This compensates for the fact that an MV
toward LAST and an MV toward GOLDEN normally point in opposite
temporal directions.

### 8.2 MV coding

Components are coded by `read_mvcomponent` (vp8/decoder/decodemv.c:64),
which returns a value in 1/4-pel units:

```c
if (vp8_read(bc, mvc[mvpis_short])) {            /* long path: magnitude ≥ 8 */
    v = 0;
    for (i = 0; i < 3; i++)
        v += vp8_read(bc, mvc[MVPbits+i]) << i;  /* low bits */
    for (i = 9; i >= 4; i--)
        v += vp8_read(bc, mvc[MVPbits+i]) << i;  /* high bits, top-down */
    /* fill bit 3 only if any higher bit is set */
} else {                                         /* short path: magnitude < 8 */
    v = vp8_treed_read(bc, vp8_small_mvtree, &mvc[MVPshort]);
}
if (v) {
    if (vp8_read(bc, mvc[MVPsign])) v = -v;
}
return v;               /* 1/4-pel units */
```

`read_mv` (decodemv.c:91) calls `read_mvcomponent` twice and stores
`row` and `col` as `read_mvcomponent(...) * 2`, so the value that lands
in `MV.row / MV.col` is in 1/8-pel units.

VP8 MVs are stored in 1/8-pel precision (a shift of 3 from full-pel),
even though luma sub-pixel filters only need 1/4-pel and chroma needs
1/8-pel — the extra bit ends up at zero for luma and gets used when MV
averaging is applied to chroma under `SPLIT_MV`.

The probability table is `MV_CONTEXT mvc[2]` (one per component), each
with `MVPcount = 19` probabilities (`MVPbits[10]`,
`MVPshort[8]`-tree, `MVPsign`, `MVPis_short`).

### 8.3 SPLIT_MV partitioning

`mbmi.partitioning` (0..3) picks one of 4 sub-block layouts:

```
 0  16x8 (two rows)         1  8x16 (two cols)
 2  8x8  (four quadrants)   3  4x4  (sixteen sub-blocks)
```

`vp8_mbsplit_offset[4][16]` (findnearmv.c:13) maps each partition's
sub-block index to the BLOCKD index inside the MB. Each sub-block can
have its own MV; in the `4x4` partition each of the 16 Y blocks has
its own MV.

### 8.4 Luma reconstruction

`vp8_build_inter16x16_predictors_mb` (reconinter.c:297) handles
non-SPLIT MBs:

1. Clamp the MV (using `vp8_clamp_mv2`, findnearmv.h:34) so that the
   6-tap filter stays inside the padded reference.
2. Convert the MV's full-pel and fractional-pel parts (luma is 1/4-pel
   precision — bits 0–2 of the MV component are the phase × 2, since
   storage is 1/8-pel).
3. Dispatch:

   ```c
   x->subpixel_predict16x16(pre_ptr + offset, pre_stride,
                            xoffset, yoffset, dst, dst_stride);
   ```

   `subpixel_predict16x16` is one of:
   - `vp8_sixtap_predict16x16` — separable 6-tap horizontal then
     vertical (vp8/common/filter.c). Filter taps in
     `vp8_sub_pel_filters[8][6]` (filter.c:20–31). Intermediate
     precision is 32-bit; the post-shift is `>> VP8_FILTER_SHIFT`
     (= 7) with rounding (`+64`).
   - `vp8_bilinear_predict16x16` — 2-tap (`vp8_bilinear_filters[8][2]`,
     filter.c:15) used when both fractional offsets are simple.

   Integer-MV blocks skip the filter and copy directly.

For `SPLIT_MV`, `build_inter4x4_predictors_mb` issues per-sub-block
calls to `vp8_sixtap_predict4x4` (or 8x4 / 8x8 variants for the
larger split shapes), each with its own MV.

### 8.5 Chroma reconstruction

Chroma MVs are derived from luma MVs:

- **Non-SPLIT** path (reconinter.c:316–334): the luma MV's two
  components are simply halved (with rounding-toward-zero), yielding
  the chroma MV in 1/8-pel precision over the chroma plane (8 chroma
  samples cover 16 luma samples).
- **SPLIT_MV** path (`build_4x4uvmvs`, reconinter.c:456–492): each of
  the 4 chroma 4x4 blocks corresponds to a 2x2 group of luma 4x4
  blocks. Its MV is the average of those 4 luma MVs, computed via
  the integer trick (reconinter.c:472, 481):

  ```c
  temp  = luma[0].row + luma[1].row + luma[2].row + luma[3].row;
  temp += 4 + ((temp >> (sizeof(temp) * CHAR_BIT - 1)) * 8);  /* see below */
  cmv.row = temp / 8;
  /* and the analogous block for the column */
  ```

  The arithmetic-right-shift `temp >> 31` produces `0` when
  `temp >= 0` (including `temp == 0`) and `-1` when `temp < 0`. So
  the bias term collapses to `+4` for non-negative sums and `-4` for
  negative sums — i.e., rounding "toward zero by 1/8 pel":

  ```c
  /* equivalent pseudo-code: */
  cmv.row = (temp + (temp >= 0 ? 4 : -4)) / 8;
  ```

  (Note: `>= 0`, not `> 0` — zero rounds the same way as positives.)
  This averaging rule is one of VP8's quirky design choices and is
  semantically part of the bitstream — both encoder and decoder must
  use exactly the same formula.

### 8.6 MV clamping

If the integer part of the MV would cause the 6-tap filter to read
beyond the YV12 frame's 32-pixel border, the MV is clamped at MV-decode
time and `mbmi.need_to_clamp_mvs` is set (decodemv.c:259, via
`vp8_check_mv_bounds`, findnearmv.h:60). The reconstruction path
relies on the borders pre-extended on the reference frame
(see §10.3) — no per-pixel clamping happens in the inner filter loop.

---

## 9. Residuals (tokens, dequant, IDCT, Walsh)

### 9.1 Coefficient token tree

VP8 codes each block's coefficients in zigzag order:

```c
static const int kZigzag[16] =                       /* detokenize.c:46 */
  { 0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15 };
```

Each non-trivial position emits one of 12 tokens
(vp8/common/entropy.h:21–34):

```
DCT_EOB_TOKEN  = end-of-block (terminate this block)
ZERO_TOKEN     = 0
ONE_TOKEN      = ±1            (extra bit: sign)
TWO_TOKEN      = ±2            (extra bit: sign)
THREE_TOKEN    = ±3            (extra bit: sign)
FOUR_TOKEN     = ±4            (extra bit: sign)
DCT_VAL_CATEGORY1..6           (extra bits: a few magnitude bits + sign)
```

Each category covers a magnitude range; the magnitude is decoded as
`base + extra_bits` where `base` and the bit count are fixed per
category (cat6 has base 67 and 11 extra bits, so its magnitude spans
±[67, …, 2114]).

### 9.2 Coefficient context

The probabilities for the token tree are indexed in four dimensions
(`coef_probs[BLOCK_TYPES][COEF_BANDS][PREV_COEF_CONTEXTS][ENTROPY_NODES]`,
entropy.h ~ line 92):

| Axis                    | Range | Meaning                                       |
|-------------------------|-------|-----------------------------------------------|
| `BLOCK_TYPES`           | 4     | 0=Y-after-Y2 AC, 1=Y2, 2=UV, 3=Y-no-Y2        |
| `COEF_BANDS`            | 8     | Position in zigzag, grouped by `vp8_coef_bands[16]` |
| `PREV_COEF_CONTEXTS`    | 3     | Magnitude class of the previous coef: 0 (none), 1 (=1), 2 (>1) |
| `ENTROPY_NODES`         | 11    | One probability per non-leaf node of the 12-symbol coef tree |

Tables: defaults in `default_coef_probs.h:20` (loaded by
`vp8_default_coef_probs`, entropy.c:145), per-frame updates gated by
`vp8_coef_update_probs[...]` (decodeframe.c:1175). Updates are
restorable: if `refresh_entropy_probs == 0`, the decoder writes them
to `pc->fc` for the current frame but restores from `pc->lfc` at
the end of the frame.

### 9.3 Per-MB decode and skip

`vp8_decode_mb_tokens` (detokenize.c) is the per-MB driver. If
`mb_no_coeff_skip == 1` for the frame and the per-MB `mb_skip_coeff`
bit is set, the entire 25-block coefficient pass is bypassed and all
`eobs` are zeroed (decodeframe.c). Otherwise it iterates:

```
if (has_Y2(MB))
    decode block 24 (Y2)        starting band 0   block_type = 1
for y in 0..15
    decode Y block y            starting band 1*  block_type = 0 (if has_Y2)
                                                  block_type = 3 (else)
for uv in 16..23
    decode UV block uv          starting band 0   block_type = 2
```

\* The Y blocks start at band 1 when Y2 is present because the DC
coefficient has already been handled by the Y2 block.

For each block, `GetCoeffs` (detokenize.c:84) walks the coefficient
tree, picks the right `coef_probs[type][band][prev_ctx]` for each
coefficient, places the magnitude (with sign) at
`qcoeff[ kZigzag[n-1] ]`, updates `prev_ctx`, and stops at the first
EOB token (or at position 16).

### 9.4 Dequantization

Per-frame, the decoder derives quantizer tables from a base index
(`y1_q`) plus per-component deltas:

```
y1dc, y1ac    Y luma  (DC and AC quantisers)
y2dc, y2ac    Y2      (the DC-of-DC block)
uvdc, uvac    Chroma
```

If segmentation is on, each segment carries its own
`alt_q[]` (either absolute or delta from `y1_q`), reapplied per MB
based on `mbmi.segment_id`. The conversion from "quantiser index" to
the actual integer scale factor is the AC/DC tables in
`vp8/common/quant_common.c:37–130` — flat for DC, bit-saturating for
AC (`vp8_ac_quant`).

For the residual pass, `vp8_dequant_idct_add_y_block` and `_uv_block`
(vp8/common/idct_blk.c) iterate per 4x4 block:

```c
if (eobs[i] > 1)
    vp8_dequant_idct_add(qcoeff, dq, dst, stride);    /* full IDCT */
else if (eobs[i] == 1)
    vp8_dc_only_idct_add(qcoeff[0]*dq[0], dst, stride, dst, stride);
/* else: nothing — block is all zeros */
```

`vp8_dequant_idct_add` scales `qcoeff[k] * dq[k]` in place into
`dqcoeff`, runs the 4x4 IDCT, then accumulates into the predictor.

### 9.5 4x4 IDCT

`vp8_short_idct4x4llm` (vp8/common/idctllm.c:29–103) is VP8's exact
4x4 inverse-transform definition — two passes of butterflies with
two specific 16-bit fixed-point constants:

```
cospi8sqrt2minus1 = 20091      /* = round((cos(π/8)·√2 - 1) · 65536)      */
sinpi8sqrt2       = 35468      /* = round( sin(π/8)·√2          · 65536)  */
```

Both row and column passes share the same butterfly. The final values
are `>> 3` with `+4` rounding and clipped to `[0, 255]` after
adding to the predictor (idctllm.c:78–82). The exact integer
arithmetic is normative — the spec specifies these constants.

For DC-only blocks (`eobs == 1`), `vp8_dc_only_idct_add` skips the
butterfly: it just spreads `(dc + 4) >> 3` across all 16 samples.

### 9.6 Walsh second-order transform

Y2 uses a Walsh–Hadamard rather than the IDCT because all its inputs
are themselves DCs. `vp8_short_inv_walsh4x4` (idctllm.c:127–175) is a
straight Hadamard with a `+3` rounding `>> 3`. Its 16 outputs are
distributed into `mb->block[i].dqcoeff[0]` for `i = 0..15`
(idctllm.c:172–174), seeding the DCs of all 16 Y IDCTs.

---

## 10. Decoded picture buffer & reference management

### 10.1 YV12_BUFFER_CONFIG

Each frame slot is a `YV12_BUFFER_CONFIG` (vpx_scale/yv12config.h:29):

```
buffer_alloc  ── single contiguous allocation, 32-byte aligned
buffer_alloc_sz

y_width    /  y_height                     padded, multiple of 16
y_crop_w   /  y_crop_h                     visible (Width / Height from header)
y_stride                                   ≥ y_width + 2*border, aligned to 32
y_buffer   = buffer_alloc + border*y_stride + border
                                           ── i.e., (border, border) is (0,0)

uv_width   /  uv_height   /  uv_crop_w  /  uv_crop_h     half of luma
uv_stride                                  ≥ uv_width + 2*border, aligned
u_buffer   /  v_buffer                     placed contiguously after Y

border                                     VP8BORDERINPIXELS = 32
```

Allocation lives in `vp8_yv12_alloc_frame_buffer`
(vpx_scale/generic/yv12config.c:124, which delegates to the realloc
helper at line 51). The single call to `vpx_memalign(32, …)`
(yv12config.c:67) returns one block; everything else is offset
arithmetic.

### 10.2 The 4-slot reference pool

`VP8_COMMON` keeps:

```c
#define NUM_YV12_BUFFERS  4                    /* onyxc_int.h:36 */
YV12_BUFFER_CONFIG  yv12_fb[NUM_YV12_BUFFERS];
int                 fb_idx_ref_cnt[NUM_YV12_BUFFERS];
int                 new_fb_idx, lst_fb_idx, gld_fb_idx, alt_fb_idx;
```

`new_fb_idx` always points to the slot the decoder is writing into.
The other three indices are the current LAST / GOLDEN / ALTREF
references. The reference counts let a single physical buffer back
multiple logical references when the frame header says
"GOLDEN ← LAST" (no copy needed — same slot, refcount bumped).

`swap_frame_buffers` (vp8/decoder/onyxd_if.c:213) — called from
`vp8dx_receive_compressed_data` after decode — applies the four
flag-driven operations in this order (lines 221–263):

```
if copy_buffer_to_arf  : alt_fb_idx  ← (lst | gld)
if copy_buffer_to_gf   : gld_fb_idx  ← (lst | alt)
if refresh_golden_frame: gld_fb_idx  ← new_fb_idx
if refresh_alt_ref_frame: alt_fb_idx ← new_fb_idx
if refresh_last_frame  : lst_fb_idx  ← new_fb_idx;  frame_to_show = LAST
else                                              : frame_to_show = NEW
```

All four reassignments go through `ref_cnt_fb`
(onyxd_if.c:204), which decrements the old slot's refcount, increments
the new slot's, and updates the index. No pixel data is copied.

After this, the decoder picks a fresh `new_fb_idx` for the next frame
by scanning `fb_idx_ref_cnt[]` for the slot with count 1 (= owned only
by the pool itself).

### 10.3 Border extension

After every decoded frame, the reconstructed buffer's 32-pixel border
is replicated outward by `vp8_yv12_extend_frame_borders`
(yv12extend.c:105): `extend_plane` (line 22) fills the top and bottom
rows with copies of the edge scanlines and the left and right border
columns with `memset` of the edge column's pixel value.

This is what allows inter prediction's 6-tap filters to read any MV
within the clamped range without per-pixel boundary checks — the
borders are guaranteed valid.

### 10.4 The user-facing image

`vpx_image_t` is the caller's view. `yuvconfig2image`
(vp8_dx_iface.c) takes the `YV12_BUFFER_CONFIG` of `cm->frame_to_show`
and sets `planes[VPX_PLANE_Y|U|V]` to `y_buffer/u_buffer/v_buffer`,
`stride[]` to the corresponding strides, and `fmt = VPX_IMG_FMT_I420`,
with `d_w/d_h` = visible (cropped) size and `w/h` = padded size. No
data is copied; the image points into the live YV12 buffer, so
the caller must consume it before the next `vpx_codec_decode()`
(or copy it out).

### 10.5 External frame buffers

`vpx_frame_buffer.h` defines a `get_fb_cb / release_fb_cb` interface
that VP9 uses to let the caller supply frame memory. The VP8 decoder
**does not** plumb these callbacks through — it always allocates
internally (the `cb == NULL` path in `vp8_yv12_realloc_frame_buffer`).
This is a quiet, longstanding asymmetry between VP8 and VP9.

---

## 11. Deblocking (loop filter)

### 11.1 Frame-level controls

Parsed from the compressed header (decodeframe.c:1037–1074):

| Field                          | Bits | Meaning                              |
|--------------------------------|------|--------------------------------------|
| `filter_type`                  | 1    | 0 = NORMAL, 1 = SIMPLE               |
| `filter_level`                 | 6    | Base strength (0..63; 0 disables LF) |
| `sharpness_level`              | 3    | 0..7                                 |
| `mode_ref_lf_delta_enabled`    | 1    | Per-MB delta adjustment on/off       |
| `mode_ref_lf_delta_update`     | 1    | Updates to deltas this frame         |
| `ref_lf_deltas[4]`             | 6/signed | Delta for INTRA/LAST/GOLDEN/ALT  |
| `mode_lf_deltas[4]`            | 6/signed | Delta for ZERO/NEW/NEAREST/NEAR/B/SPLITMV (grouped) |

Stored in `VP8_COMMON` (the base settings) and `MACROBLOCKD` (the
deltas). Deltas survive across frames unless explicitly updated.

### 11.2 Precomputed per-MB strength table

Before the per-MB loop, `vp8_loop_filter_frame_init`
(vp8/common/vp8_loopfilter.c:94–165) fills out `loop_filter_info_n`
(vp8/common/loopfilter.h:38):

```c
typedef struct {
  unsigned char mblim   [MAX_LOOP_FILTER+1][SIMD_WIDTH]; /* MB-edge limit */
  unsigned char blim    [MAX_LOOP_FILTER+1][SIMD_WIDTH]; /* sub-edge limit*/
  unsigned char lim     [MAX_LOOP_FILTER+1][SIMD_WIDTH]; /* inner limit   */
  unsigned char hev_thr [4][SIMD_WIDTH];                 /* HEV threshold */
  unsigned char lvl     [4][4][4];      /* [seg][ref][mode] → final level */
  unsigned char hev_thr_lut[2][MAX_LOOP_FILTER+1];      /* keyframe LUT   */
  unsigned char mode_lf_lut[10];        /* mode → delta-table index       */
} loop_filter_info_n;
```

The thresholds (`mblim`, `blim`, `lim`, `hev_thr`) are derived from
`filter_level` and `sharpness_level` in
`vp8_loop_filter_update_sharpness` (vp8_loopfilter.c:49–75):

```
mblim = 2*(filter_level + 2) + lim_inner
blim  = 2* filter_level      + lim_inner
lim   = filter_level >> (sharpness>0) >> (sharpness>4)
        clamped to [1, 9 - sharpness]
hev_thr ∈ {0,1,2,3} from a 2-entry keyframe LUT indexed by
         filter_level thresholds {15, 20, 40}.
```

The 4×4×4 `lvl[seg][ref][mode]` table caches the final per-MB
filter level (post-delta) so the inner loop just indexes it.

### 11.3 Per-edge kernels

`vp8/common/loopfilter_filters.c` carries the C reference kernels:

| Kernel                          | Edge type            | Length |
|---------------------------------|----------------------|--------|
| `mbloop_filter_horizontal_edge_c` (line 191) | MB top edge        | 16 px (luma) / 8 (chroma) |
| `mbloop_filter_vertical_edge_c`   (line 216) | MB left edge       | 16 / 8 |
| `loop_filter_horizontal_edge_c`   (line 90)  | Internal block edge| 16 / 8 |
| `loop_filter_vertical_edge_c`     (line 114) | Internal block edge| 16 / 8 |
| `vp8_simple_filter*`                         | SIMPLE_LOOPFILTER variants — only p0/q0 adjusted |

The innermost arithmetic uses `vp8_filter_mask` (line 24) to gate
filtering when neighbor pixel differences exceed the limits, and
`vp8_hevmask` (line 38) to switch between the "narrow" 4-tap filter
(`vp8_filter`, line 45) and the "wide" filter (`vp8_mbfilter`,
line 138) that adjusts 3 pairs of pixels on either side of an MB
boundary with weights `3/7, 2/7, 1/7`.

All arithmetic is in `int8_t` after a 128-shift, with explicit
saturation through `vp8_signed_char_clamp` (line 17).

### 11.4 Skip rule

For each internal sub-block edge:

```
skip_lf = (mbmi.mode != B_PRED && mbmi.mode != SPLIT_MV
           && mbmi.mb_skip_coeff != 0);
```

This is the VP8 spec rule: an inter-coded MB with no residuals has no
new high-frequency content to deblock internally, so the internal
4x4-block edges are skipped. MB-boundary edges (mbv, mbh) are always
applied when `filter_level > 0`.

### 11.5 Ordering

`vp8_loop_filter_frame` (vp8_loopfilter.c:263) walks MBs in raster
order. For each MB, in this order:

1. Left MB edge — `vp8_loop_filter_mbv` (if `mb_col > 0`).
2. Three internal vertical edges — `vp8_loop_filter_bv` (unless
   `skip_lf`).
3. Top MB edge — `vp8_loop_filter_mbh` (if `mb_row > 0`).
4. Three internal horizontal edges — `vp8_loop_filter_bh` (unless
   `skip_lf`).

Luma and chroma are filtered together inside each kernel. Output
pixels in the same row are referenced again when the next MB's left
edge is filtered, but the pixel rewrites are local enough that this
single-pass raster order is sound.

---

## 12. Memory management

### 12.1 Allocator wrappers

`vpx_mem/vpx_mem.c` wraps the platform allocator with alignment
support:

- `vpx_memalign(align, size)` — returns a pointer aligned to `align`.
  Implementation: over-allocate by `align - 1 + sizeof(size_t)`,
  store the original `malloc` pointer in the word just below the
  returned pointer, return the rounded-up address (vpx_mem.c:57).
- `vpx_malloc(size)` — `vpx_memalign(DEFAULT_ALIGNMENT, size)`. On
  this build, `DEFAULT_ALIGNMENT = 2 * sizeof(void*)` (16 on x86-64).
- `vpx_calloc(num, size)` — `vpx_malloc` + `memset(0)`.
- `vpx_free(p)` — recover the original pointer from the stash and
  `free()` it (handles `NULL` gracefully).

All multi-byte buffers in libvpx go through these wrappers, which
means there is exactly one place to instrument (or replace with a
custom allocator) for the entire codec.

### 12.2 Where allocations live

| Where                                                  | Owner       | Lifetime |
|--------------------------------------------------------|-------------|----------|
| `vpx_codec_alg_priv_t`                                 | dispatcher  | init→destroy |
| `VP8D_COMP` (one per concurrent in-flight frame)       | `pbi[]`     | init→destroy |
| `mip` / `prev_mip` (MODE_INFO grids)                   | VP8_COMMON  | resize → resize/destroy |
| `yv12_fb[0..3]` (frame buffers + 32-px border)         | VP8_COMMON  | resize → resize/destroy |
| Per-frame entropy probabilities                        | VP8_COMMON  | inline                |
| Scratch / per-thread row buffers (MT only)             | per worker  | per-frame             |

The big-ticket allocations are the YV12 frame buffers (≈
`1.5 × w × h` bytes each, × 4 slots) and the MODE_INFO grid
(≈ `sizeof(MODE_INFO) × (mb_cols+1) × (mb_rows+1)` × 2 for `mi` and
`prev_mi`). They are reallocated on every resolution change and freed
at decoder destruction.

### 12.3 Alignment

Frame buffers are 32-byte aligned to support SIMD loads (`vpx_memalign(32, …)`
at yv12config.c:67). In this minimal C-only build the alignment is
unnecessary but harmless. `MACROBLOCKD` and `VP8_COMMON` are themselves
declared `DECLARE_ALIGNED(16, …)` inside `VP8D_COMP`
(onyxd_int.h:60, 64) for the same reason.

---

## 13. Threading

### 13.1 Conceptual model (what threading.c **would** do)

With `--enable-multithread`, libvpx's VP8 decoder uses **row-based
parallelism**:

- One main thread parses the residual partition (modes, MVs, frame
  header) for an MB row.
- N worker threads each take one of the token partitions and decode
  coefficients + reconstruct pixels for the MB rows assigned to that
  partition (round-robin by `row % N`).
- Synchronization is via atomic counters
  (`mt_current_mb_col[mb_row]`, vp8/decoder/threading.c:309). A worker
  decoding row `r`, column `c` waits until the worker on row `r-1`
  has progressed past column `c + small_lookahead` — enough that the
  current MB's deblocking can safely run on the previous row.
- Number of partitions caps parallelism: with 8 token partitions you
  can use up to 8 workers (decodeframe.c:808–809).
- The worker pool itself is the generic `VPxWorker` API in
  `vpx_util/vpx_thread.[ch]` (used by both VP8 and VP9).

### 13.2 In the minimal `--disable-multithread` build

The CONFIG_MULTITHREAD-guarded code in `vp8/decoder/threading.c`, the
threaded paths in `vp8_dx_iface.c`, and the worker-pool implementation
in `vpx_util/vpx_thread.c` are compiled to nothing. The frame decode
runs entirely on the calling thread. The `mbc[1..N]` token-partition
bool decoders are still parsed serially in the same thread; row-based
sync is just absent.

`vpx_thread.c` is still compiled (it appears in the §A file list of
`vp8_files.md`) but the work-distribution code path is `#if`'d out;
what remains is essentially stubs. The doc explicitly notes
`vpx_thread.c` is "safely deletable in a fork."

### 13.3 pthread abstraction

`vpx_util/vpx_pthread.h` is a thin platform shim:

- On POSIX: `#include <pthread.h>`; libvpx's `pthread_t`,
  `pthread_mutex_t`, `pthread_cond_t` are aliases for the system
  types.
- On Windows: typedefs map to Win32 `HANDLE`, `CRITICAL_SECTION`,
  `CONDITION_VARIABLE`; small inline wrappers translate
  `pthread_create / join / mutex_lock / cond_wait` into the Windows
  equivalents.

There is **no thread pool reuse across frames** in VP8 — workers are
created in `vp8_decoder_create_threads` (threading.c) once per
decoder instance and live until decoder destruction.

### 13.4 Per-frame "frame-MT" mode

In some configurations libvpx also runs an outer pipeline: while
worker threads finish decoding frame N, the main thread starts
parsing the header of frame N+1 (the `pbi[MAX_FB_MT_DEC]` array
exists for this). With `--disable-multithread` only `pbi[0]` is used.

---

## 14. Runtime CPU dispatch (RTCD)

### 14.1 Pattern

Each module has a Perl script that emits a header full of function
pointers. At runtime, a small `<module>_rtcd()` function picks the
best variant for the host CPU.

| Module      | Perl script                    | Init function       |
|-------------|--------------------------------|---------------------|
| `vp8`       | `vp8/common/rtcd_defs.pl`      | `vp8_rtcd()`        |
| `vpx_dsp`   | `vpx_dsp/vpx_dsp_rtcd_defs.pl` | `vpx_dsp_rtcd()`    |
| `vpx_scale` | `vpx_scale/vpx_scale_rtcd.pl`  | `vpx_scale_rtcd()`  |

Generated headers (`vp8_rtcd.h`, `vpx_dsp_rtcd.h`, `vpx_scale_rtcd.h`)
contain declarations like:

```c
extern void (*vp8_sixtap_predict16x16)(uint8_t *src, int src_stride,
                                       int xoffset, int yoffset,
                                       uint8_t *dst, int dst_stride);
RTCD_EXTERN void vp8_sixtap_predict16x16_c(/* … */);
```

with a default `#define vp8_sixtap_predict16x16 vp8_sixtap_predict16x16_c`
when no SIMD variants exist for the target. On `--target=generic-gnu`
the generated `setup_rtcd_internal` is empty (`{ }`), so the function
pointers remain bound to their `_c` reference implementations.

### 14.2 One-time init

`vpx_once` (vpx_ports/vpx_once.h:94 on POSIX, 40 on Win32) guards
all three `rtcd()` calls with `pthread_once` (POSIX) or an
`InterlockedCompareExchange` state machine (Windows), so that
concurrent decoder contexts initialize the function-pointer tables
exactly once.

All three RTCD-init functions are invoked unconditionally at decoder
construction, in `vp8_init` (vp8/vp8_dx_iface.c:92–94):

```c
static vpx_codec_err_t vp8_init(vpx_codec_ctx_t *ctx, …) {
  …
  vp8_rtcd();
  vpx_dsp_rtcd();
  vpx_scale_rtcd();
  …
}
```

Later, `create_decompressor` (vp8/decoder/onyxd_if.c:118) calls
`once(initialize_dec)`, where `initialize_dec`
(vp8/decoder/onyxd_if.c:48):

```c
static void initialize_dec(void) {
  if (!init_done) {
    vpx_dsp_rtcd();
    vp8_init_intra_predictors();          /* fill the dispatch table     */
    init_done = 1;
  }
}
```

re-calls `vpx_dsp_rtcd()` — harmless, because each `<module>_rtcd()`
body is itself wrapped in `once(setup_rtcd_internal)`, so the second
call is a no-op.

`vp8_machine_specific_config` (vp8/common/generic/systemdependent.c:63)
is **not** part of the RTCD init chain. On the generic-gnu target it
only records `ctx->processor_core_count` for would-be threading
(under `CONFIG_MULTITHREAD`); with multithread disabled it is a
complete no-op (`(void)ctx;`).

### 14.3 `vpx_clear_system_state`

`vpx_ports/system_state.h:20–24` is a macro that on x86/x64+MMX builds
expands to a function call that emits `emms` to flush MMX state and
clear x87 floating-point exception flags. On `generic-gnu` it is a
no-op. The decoder calls it after each frame (and on error paths) so
that callers using MMX/SSE don't inherit a dirty FPU.

### 14.4 Compiler-attribute shims

`vpx_ports/compiler_attributes.h` carries portable annotation macros:

- `VPX_NO_UNSIGNED_OVERFLOW_CHECK` — suppress UBSan's
  `unsigned-integer-overflow` checker on a few hot paths that rely on
  defined unsigned wraparound (e.g., in the bool decoder
  normalization).
- `VPX_NO_UNSIGNED_SHIFT_CHECK` — same idea for shifts.

VP8 intentionally relies on C's defined unsigned wraparound; these
macros are how it stays clean under sanitizer builds.

---

## 15. Error handling

The decoder uses `setjmp` / `longjmp` to propagate fatal bitstream
errors out of deeply nested parsers without poisoning every function
with an error-return code.

`vpx_internal_error_info` (vpx/internal/vpx_codec_internal.h):

```c
struct vpx_internal_error_info {
  vpx_codec_err_t error_code;
  int             has_detail;
  char            detail[80];
  int             setjmp;
  jmp_buf         jmp;
};
```

is embedded in `VP8_COMMON.error`. The flow:

1. **Setup**: `create_decompressor` (onyxd_if.c:73) does
   `if (setjmp(pbi->common.error.jmp)) { … return -1; }` and sets
   `pbi->common.error.setjmp = 1`.
2. **Per-decode**: `vp8_decode` (vp8_dx_iface.c) wraps the call in
   the same `setjmp` so a longjmp lands back in the dispatcher.
3. **Trigger**: parsers call
   `vpx_internal_error(&pc->error, VPX_CODEC_CORRUPT_FRAME, "fmt", …)`
   which sets `error_code`, formats `detail`, and longjmps if
   `setjmp == 1`. Otherwise it returns normally and the caller checks
   `error_code`.
4. **Capture**: `update_error_state(ctx, &pbi->common.error)` copies
   the error into `ctx->priv->err_detail` so that
   `vpx_codec_error_detail(ctx)` works for the application.
5. **Frame marking**: `corrupted` bit gets set on the YV12 buffer so
   that `VP8D_GET_FRAME_CORRUPTED` reflects it for the latest frame.

This is also why the bool decoder is a header-inlined function: a
truncated bitstream is detected at refill time
(`vp8dx_bool_decoder_fill` sees `user_buffer >= user_buffer_end`),
and it raises `VPX_CODEC_CORRUPT_FRAME` via the same longjmp path
without unwinding any state explicitly.

---

## 16. Bitstream layout (wire format)

This chapter walks the VP8 bitstream byte-by-byte and, for each field,
gives:

- the **RFC 6386** section (the authoritative normative spec; a copy
  lives at `rfc6386.txt`),
- the **libvpx parser line** that reads it,
- the encoding (raw bits, bool-coded with constant prob, bool-coded
  tree, …) and any width / range / default information.

A VP8 access unit (one decodable frame, including reference-update
side effects) is a contiguous byte string laid out as follows:

```
┌──────────────────────────────────────────────────────────────────────┐
│   Frame tag                          (3 bytes,  raw)                 │
│   Key-frame extras                   (7 bytes,  raw) ─ key frames    │
│ ┌──────────────────────────────────────────────────────────────────┐ │
│ │ Residual / "first" partition    (variable, bool-coded)           │ │
│ │   ├ color space / clamp_type           (key frames only)         │ │
│ │   ├ segmentation block                                           │ │
│ │   ├ loop-filter block                                            │ │
│ │   ├ log2(num token partitions)                                   │ │
│ │   ├ token-partition size table   (NOT bool-coded; raw 3-byte LE) │ │
│ │   ├ quantizer block                                              │ │
│ │   ├ reference-buffer flags                                       │ │
│ │   ├ refresh_entropy_probs + refresh_last_frame                   │ │
│ │   ├ coefficient probability updates                              │ │
│ │   ├ mb_no_coeff_skip + prob_skip_false                           │ │
│ │   ├ prob_intra/prob_last/prob_gf  (inter frames only)            │ │
│ │   ├ Y/UV mode probability updates (inter frames only)            │ │
│ │   ├ MV probability updates        (inter frames only)            │ │
│ │   └ per-MB modes/MVs/skips        (mb_rows × mb_cols times)      │ │
│ └──────────────────────────────────────────────────────────────────┘ │
│ ┌──────────────────────────────────────────────────────────────────┐ │
│ │ Token partition 0   (bool-coded)   ─┐                            │ │
│ │ Token partition 1   (bool-coded)    │  N = 1, 2, 4, or 8         │ │
│ │       …                             │  MB row r reads tokens     │ │
│ │ Token partition N-1 (bool-coded)   ─┘  from partition (r % N)    │ │
│ └──────────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
```

A few conventions before we dig in:

- **`vp8_read_bit(bc)`** — read one bit at prob 128 (50/50).
- **`vp8_read_literal(bc, n)`** — read `n` raw bits, each at prob 128,
  most-significant-bit first. Used for fixed-width literals.
- **`vp8_read(bc, p)`** — read one bit with custom 8-bit prob `p`.
- **`vp8_treed_read(bc, tree, probs)`** — walk a tree code (see §5.2).
- The libvpx decoder caches **the residual partition** at
  `pbi->mbc[8]` (vp8/decoder/decodeframe.c:880) and the
  **token partitions** at `pbi->mbc[0..N-1]` (decodeframe.c:1079,
  setup_token_decoder:730). Storage size is `MAX_PARTITIONS = 9`
  (vp8/common/onyxc_int.h:38).

All field offsets given below are within the residual partition unless
explicitly marked as "raw" (i.e., outside the bool decoder).

### 16.1 Frame tag — 3 raw bytes (RFC 6386 §9.1)

```
byte 0   bit 0     frame_type                       0 = KEY, 1 = INTER
         bits 1-3  version                          0..3 (filter / subpixel variants)
         bit  4    show_frame                       1 = display this frame
         bits 5-7  first_partition_length_in_bytes[bits 0-2]
byte 1            first_partition_length_in_bytes[bits 3-10]
byte 2            first_partition_length_in_bytes[bits 11-18]
```

`first_partition_length_in_bytes` (19 bits) is the **byte length of
the residual partition only** (header parsing, modes, MVs, …) — *not*
including the token partitions that follow.

Parser (decodeframe.c:921–925):

```c
pc->frame_type    = (FRAME_TYPE)(clear[0] & 1);
pc->version       = (clear[0] >> 1) & 7;
pc->show_frame    = (clear[0] >> 4) & 1;
first_partition_length_in_bytes =
    (clear[0] | (clear[1] << 8) | (clear[2] << 16)) >> 5;
```

`version` selects between the four filter / sub-pixel pairs (§18.4 of
RFC 6386 / vp8_setup_version, decodeframe.c:935). Versions ≥ 4 are
reserved.

### 16.2 Key-frame extras — 7 raw bytes (key frames only, RFC §9.1)

Immediately after the frame tag on key frames:

```
byte 3-5          sync code, exactly  0x9d 0x01 0x2a
byte 6 lo + byte 7 lo-6   Width   (14 bits, little-endian, in pixels)
byte 7 hi-2               horiz_scale (2 bits;  0 = 1× scale, …)
byte 8 lo + byte 9 lo-6   Height  (14 bits, little-endian)
byte 9 hi-2               vert_scale  (2 bits)
```

Parser (decodeframe.c:943–951):

```c
if (clear[0] != 0x9d || clear[1] != 0x01 || clear[2] != 0x2a)
    /* error: invalid sync code */;
pc->Width       = (clear[3] | (clear[4] << 8)) & 0x3fff;
pc->horiz_scale =  clear[4] >> 6;
pc->Height      = (clear[5] | (clear[6] << 8)) & 0x3fff;
pc->vert_scale  =  clear[6] >> 6;
```

The `scale` fields are advisory display hints — VP8 itself decodes at
the coded dimensions and leaves rescaling to the application. The
14-bit width/height cap dimensions at 16383.

### 16.3 Opening the residual partition (RFC §7, §9)

```c
vp8dx_start_decode(&pbi->mbc[8], data, data_end - data, …);
              /* decodeframe.c:976 */
```

From here on, all reads are through the arithmetic (bool) decoder
described in §5.1. The first `first_partition_length_in_bytes` of the
payload are the input to this decoder; whatever comes after is the
token-partition area (parsed in §16.6 below).

### 16.4 Color-space & clamping (key frames only, RFC §9.2)

```c
if (pc->frame_type == KEY_FRAME) {
    (void)vp8_read_bit(bc);                    /* "color_space"; must be 0 */
    pc->clamp_type = (CLAMP_TYPE)vp8_read_bit(bc);
}
/* decodeframe.c:981-984 */
```

`color_space` is reserved (must be 0; YUV in BT.601-like range).
`clamp_type` selects whether the decoder clamps decoded samples to
[0, 255] (the only allowed mode in practice).

### 16.5 Segmentation block (RFC §9.3, §10)

```
1 bit   segmentation_enabled
if segmentation_enabled:
    1 bit   update_mb_segmentation_map
    1 bit   update_mb_segmentation_data
    if update_mb_segmentation_data:
        1 bit   segment_feature_mode      (0 = delta, 1 = absolute)
        for each feature f in {ALT_Q, ALT_LF}:        /* MB_LVL_MAX = 2 */
            for each segment s in 0..3:               /* MAX_MB_SEGMENTS = 4 */
                1 bit   present?
                if present:
                    n bits   magnitude     n = 7 (ALT_Q) or 6 (ALT_LF)
                    1 bit    sign
    if update_mb_segmentation_map:
        for i in 0..2:                                /* MB_FEATURE_TREE_PROBS = 3 */
            1 bit   probability_present?
            if present:
                8 bits  probability
            else:
                probability defaults to 255
```

Parser: decodeframe.c:987–1034. Constants:
`MB_LVL_MAX = 2`, `MAX_MB_SEGMENTS = 4`, `MB_FEATURE_TREE_PROBS = 3`
(vp8/common/blockd.h:31–32, 85). The per-feature magnitude widths are
`vp8_mb_feature_data_bits[2] = { 7, 6 }` (entropy.c:65).

The 3 segment-tree probabilities decode a 4-segment 2-bit ID per MB
via a tiny custom tree (read_mb_features, decodemv.c:475):

```
                p[0]
                /  \
             0 /    \ 1
              p[1]   p[2]
              / \    / \
           seg0 seg1 seg2 seg3
```

### 16.6 Loop-filter block (RFC §9.4)

```
1 bit       filter_type              0 = NORMAL, 1 = SIMPLE
6 bits      filter_level             0..63       (0 disables LF)
3 bits      sharpness_level          0..7
1 bit       mode_ref_lf_delta_enabled
if enabled:
    1 bit   mode_ref_lf_delta_update
    if update:
        for i in 0..3:                              /* MAX_REF_LF_DELTAS = 4 */
            1 bit  present?
            if present:
                6 bits  magnitude
                1 bit   sign
        for i in 0..3:                              /* MAX_MODE_LF_DELTAS = 4 */
            1 bit  present?
            if present:
                6 bits  magnitude
                1 bit   sign
```

Parser: decodeframe.c:1037–1075. The ordering matters: `ref_lf_deltas`
covers (INTRA, LAST, GOLDEN, ALT); `mode_lf_deltas` covers
(B_PRED-ish, ZERO_MV-ish, MV-ish, SPLITMV-ish) — see the comment at
blockd.h:271–276. Both delta arrays are *persistent across frames*
unless updated, so a single update flag stays in force for the rest of
the GOP.

### 16.7 Token-partition count — 2 bits (RFC §9.5)

```
2 bits   multi_token_partition    ∈ {0,1,2,3}
                                   ⇒ num_token_partitions = 1 << val ∈ {1,2,4,8}
```

Parser (vp8/decoder/decodeframe.c:738–742, inside
`setup_token_decoder`):

```c
TOKEN_PARTITION multi_token_partition =
    (TOKEN_PARTITION)vp8_read_literal(&pbi->mbc[8], 2);
num_token_partitions = 1 << pbi->common.multi_token_partition;
```

Note that these 2 bits **are bool-coded** (they appear inside the
residual partition), even though everything else about partition
layout lives outside the bool stream.

### 16.8 Token-partition size table — raw, *outside* the bool decoder

Immediately after the last byte of the residual partition (i.e., at
byte offset `frame_tag_size + first_partition_length_in_bytes` of the
frame), there is a raw block of `3 × (num_token_partitions − 1)` bytes
giving the byte lengths of the first `N−1` token partitions, each as
a 3-byte little-endian unsigned integer. The length of the *last*
token partition is implicit (it occupies the rest of the frame).

```
size_of_partition_0   3 raw bytes (LE)
size_of_partition_1   3 raw bytes (LE)
…
size_of_partition_(N-2) 3 raw bytes (LE)
[no entry for partition N-1: implicit]

partition_0           size_of_partition_0       bytes (bool-coded)
partition_1           size_of_partition_1       bytes (bool-coded)
…
partition_(N-1)       (remainder)               bytes (bool-coded)
```

The size table is not entropy-coded so a multi-threaded decoder can
locate every partition without first walking the bool stream.

The libvpx side of this lives in `read_available_partition_size`
(decodeframe.c:684) and the carving loop in `setup_token_decoder`
(decodeframe.c:728–806). Token partitions are opened with
`vp8dx_start_decode` into `pbi->mbc[0..N-1]` (decodeframe.c:794–804).

### 16.9 Quantizer block (RFC §9.6)

Back in the residual partition's bool stream:

```
7 bits         y_ac_qi                  base Y AC quantiser index, 0..127
get_delta_q()  y1dc_delta_q             Y plane DC delta
get_delta_q()  y2dc_delta_q             Y2 DC delta
get_delta_q()  y2ac_delta_q             Y2 AC delta
get_delta_q()  uvdc_delta_q             chroma DC delta
get_delta_q()  uvac_delta_q             chroma AC delta
```

with `get_delta_q` (decodeframe.c:235, paraphrased):

```c
if (vp8_read_bit(bc)) {           /* delta present? */
    int v = vp8_read_literal(bc, 4);
    if (vp8_read_bit(bc)) v = -v; /* sign */
    return v;
}
return 0;                         /* delta absent — keep previous value */
```

Parser: decodeframe.c:1081–1098. Each present-flag is 1 bit; if set,
4 bits of magnitude + 1 bit of sign follow. Otherwise the previous
frame's value persists.

### 16.10 Reference-buffer flags — inter frames only (RFC §9.7, §9.8)

On key frames all three reference slots are implicitly refreshed.
On inter frames:

```
1 bit         refresh_golden_frame
1 bit         refresh_alt_ref_frame
if !refresh_golden_frame:
    2 bits    copy_buffer_to_gf       ∈ {0,1,2}:
                                       0 = no copy
                                       1 = GOLDEN ← LAST
                                       2 = GOLDEN ← ALT
if !refresh_alt_ref_frame:
    2 bits    copy_buffer_to_arf      ∈ {0,1,2}:
                                       0 = no copy
                                       1 = ALT ← LAST
                                       2 = ALT ← GOLDEN
1 bit         sign_bias[GOLDEN]
1 bit         sign_bias[ALTREF]
```

Parser: decodeframe.c:1104–1147.

Then, both inter and key frames:

```
1 bit         refresh_entropy_probs
1 bit         refresh_last_frame              (only on inter frames;
                                               implicitly 1 on key)
```

Parser: decodeframe.c:1149–1166.

If `refresh_entropy_probs == 0`, the parser stashes `pc->fc` into
`pc->lfc` (line 1157) before applying the per-frame coefficient and
MV-prob updates, and at the end of the frame restores `pc->fc` from
`pc->lfc` (line 1246). The on-wire updates therefore affect only the
current frame.

### 16.11 Coefficient probability updates (RFC §9.9, §13.4)

For every node of every coefficient context — a 4-D table of shape
`[BLOCK_TYPES=4][COEF_BANDS=8][PREV_COEF_CONTEXTS=3][ENTROPY_NODES=11]` =
1056 nodes — one update flag with a fixed prob, followed by an 8-bit
literal if updated:

```
for i in 0..3:                    /* BLOCK_TYPES */
    for j in 0..7:                /* COEF_BANDS */
        for k in 0..2:            /* PREV_COEF_CONTEXTS */
            for l in 0..10:       /* ENTROPY_NODES */
                vp8_read(bc, vp8_coef_update_probs[i][j][k][l]):
                    if 1: prob[i][j][k][l] = vp8_read_literal(bc, 8)
```

Parser: decodeframe.c:1172–1187. The update-flag probabilities are
the table `vp8_coef_update_probs` (coefupdateprobs.h:21), almost all
set to 252 — i.e., "almost always 0", so the update block is usually
nearly empty.

### 16.12 mb_no_coeff_skip and predictor probabilities (RFC §9.10–9.11)

The "remaining frame-header" parsing lives in `mb_mode_mv_init`
(decodemv.c:122–163), called once before the per-MB loop:

```
1 bit       mb_no_coeff_skip                   (frame-wide: any MB allowed to skip?)
if mb_no_coeff_skip:
    8 bits  prob_skip_false                    prob that mb_skip_coeff = 0

if frame_type == INTER_FRAME:
    8 bits  prob_intra                         prob that an MB is intra-coded
    8 bits  prob_last                          prob LAST | not-LAST
    8 bits  prob_gf                            prob GOLDEN | not-GOLDEN  (given not LAST)

    1 bit   update_ymode_probs?
    if update:
        for i in 0..3:
            8 bits  ymode_prob[i]              4 non-leaf nodes of ymode tree

    1 bit   update_uv_mode_probs?
    if update:
        for i in 0..2:
            8 bits  uv_mode_prob[i]            3 non-leaf nodes of uv tree

    /* MV-component prob updates: */
    for c in 0..1:                              /* row, col */
        for j in 0..MVPcount-1:                 /* MVPcount = 19 */
            vp8_read(bc, vp8_mv_update_probs[c][j]):
                if 1:
                    7 bits  prob_raw
                    component_prob[c][j] = prob_raw ? (prob_raw << 1) : 1
```

Constants: `MVPcount = 19` (entropymv.h:36), comprising
`MVPis_short(1) + MVPsign(1) + MVPshort(7-node tree) + MVPbits(10)`
(entropymv.h:31–36). The 7-bit raw probability followed by a
"non-zero or fall-back-to-1" mapping is a VP8-specific quirk: it
ensures a probability of 0 cannot be coded (which would make some
branches undecodable).

### 16.13 Per-macroblock syntax

After the header, the residual partition encodes, for every
macroblock in raster order, the modes / refs / MVs / skip flag. The
coefficients themselves live in the round-robin token partitions
(§16.14) so the parser swaps decoders depending on the field.

Driver: `vp8_decode_mode_mvs` → `decode_mb_mode_mvs`
(decodemv.c:516, 489).

Per MB, in order (decodemv.c:489–513):

```
read_mb_features(&pbi->mbc[8], mbmi, mb)            /* §16.5 segment ID */
if mb_no_coeff_skip:
    1 bit  mb_skip_coeff       coded with prob_skip_false
                                (decodemv.c:503)
if frame_type == KEY_FRAME:
    read_kf_modes()                                 /* §16.13.1 */
else:
    read_mb_modes_mv()                              /* §16.13.2 */
```

#### 16.13.1 Key-frame MBs (RFC §11)

```c
/* read_kf_modes, decodemv.c:42-62                                     */
mbmi.ref_frame = INTRA_FRAME;
mbmi.mode = read_kf_ymode(bc, vp8_kf_ymode_prob);    /* tree-coded Y mode  */

if (mbmi.mode == B_PRED) {
    for i in 0..15:
        A = above_block_mode(mi, i, mis);            /* §7.4 */
        L = left_block_mode(mi, i);
        mi->bmi[i].as_mode =
            read_bmode(bc, vp8_kf_bmode_prob[A][L]); /* per-4x4 tree */
}
mbmi.uv_mode = read_uv_mode(bc, vp8_kf_uv_mode_prob);
```

`vp8_kf_ymode_tree` codes 5 leaves (DC, V, H, TM, B_PRED).
`vp8_kf_uv_mode_tree` codes 4 leaves (DC, V, H, TM). Probability
tables `vp8_kf_*_prob` are fixed constants for key frames; inter
frames use the per-frame-updated `pc->fc.ymode_prob` and
`pc->fc.uv_mode_prob` instead.

#### 16.13.2 Inter-frame MBs (RFC §16)

```c
/* read_mb_modes_mv, decodemv.c:284-473                                  */
mbmi.ref_frame = vp8_read(bc, prob_intra);           /* 0 = intra, 1 = inter */

if (mbmi.ref_frame) {        /* inter-predicted                           */
    /* pick reference: ref_frame is already 1 (= LAST_FRAME) here.
       If prob_last reads 1, replace it with GOLDEN (2) or ALTREF (3): */
    if (vp8_read(bc, prob_last)) {
        mbmi.ref_frame = 2 + vp8_read(bc, prob_gf);  /* 2=GOLDEN, 3=ALTREF */
    }
    /* else: mbmi.ref_frame stays 1 = LAST_FRAME                         */

    /* compute (nearest, near, best) MV predictors and a 4-bin count
       of neighbor MV classes (vp8_find_near_mvs) — see §8.1           */

    /* mode coded via up to four sequential bool reads, with
       probabilities indexed by the count histogram (vp8_mode_contexts): */
    if (vp8_read(bc, mode_ctx[cnt[CNT_INTRA]   ][0]) == 0) {
        mbmi.mode = ZEROMV;  mbmi.mv = 0;
    } else if (vp8_read(bc, mode_ctx[cnt[CNT_NEAREST]][1]) == 0) {
        mbmi.mode = NEARESTMV; mbmi.mv = nearest_mv;
    } else if (vp8_read(bc, mode_ctx[cnt[CNT_NEAR]   ][2]) == 0) {
        mbmi.mode = NEARMV;    mbmi.mv = near_mv;
    } else if (vp8_read(bc, mode_ctx[cnt[CNT_SPLITMV]][3])) {
        /* SPLITMV: see §16.13.3                                          */
        decode_split_mv(bc, mi, …);
        mbmi.mode = SPLITMV;
    } else {
        mbmi.mode = NEWMV;
        read_mv(bc, &delta, mvc);                    /* §16.14 */
        mbmi.mv = best_mv + delta;
    }
}
else {                       /* intra MB: same fields as key-frame MB    */
    mbmi.mv = 0;
    mbmi.mode = read_ymode(bc, pc->fc.ymode_prob);
    if (mbmi.mode == B_PRED) {
        for i in 0..15:
            mi->bmi[i].as_mode = read_bmode(bc, pc->fc.bmode_prob);
            /* note: inter-frame B_PRED uses unconditional probs,
               not the [A][L] table key frames use                       */
    }
    mbmi.uv_mode = read_uv_mode(bc, pc->fc.uv_mode_prob);
}
```

The mode_context table `vp8_mode_contexts[6][4]` (modecont.c:13) has
6 rows indexed by the neighbor count for that class (0..5+), and 4
columns: one per "stage" of the if-else above. This is how
"unanimous neighbors say NEAREST" makes NEAREST much cheaper to
signal than NEW.

#### 16.13.3 SPLIT_MV partition (RFC §16.4)

`decode_split_mv` (decodemv.c:188–282) first picks a split shape with
three bool reads using **literal** probabilities (these magic numbers
are normative, decodemv.c:201–207):

```c
if (vp8_read(bc, 110)) {
    if (vp8_read(bc, 111)) {
        s = vp8_read(bc, 150);    /* 0 = 16x8, 1 = 8x16 */
        num_p = 2;
    } else {
        s = 2;                    /* 8x8 quadrants */
        num_p = 4;
    }
} else {
    s = 3;                        /* 4x4 — every sub-block its own MV */
    num_p = 16;
}
```

Then, for each of the `num_p` sub-partitions, a 4-way sub-MV mode is
decoded using a context built from the LEFT/ABOVE neighbor's MV:

```c
const vp8_prob *prob = get_sub_mv_ref_prob(left_mv, above_mv);

if (vp8_read(bc, prob[0])) {
    if (vp8_read(bc, prob[1])) {
        if (vp8_read(bc, prob[2])) {
            blockmv.row = read_mvcomponent(bc, &mvc[0]) * 2;
            blockmv.col = read_mvcomponent(bc, &mvc[1]) * 2;
            blockmv += best_mv;                       /* NEW4x4 */
        } else  blockmv = 0;                          /* ZERO4x4 */
    } else      blockmv = above_mv;                   /* ABOVE4x4 */
} else          blockmv = left_mv;                    /* LEFT4x4 */
```

The fixed probability tables for the 8 distinct (LEFT-zero,
ABOVE-zero, LEFT-equals-ABOVE) contexts are
`vp8_sub_mv_ref_prob3[8][3]` (decodemv.c:165).

### 16.14 MV component coding (RFC §17.1)

`read_mvcomponent` (decodemv.c:64–89) reads one MV component (row or
column). The output is in 1/8-pel units (`* 2` is applied in
`read_mv`, decodemv.c:91–94, so the component itself returns 1/4-pel
units multiplied by two — see RFC §17.1):

```c
if (vp8_read(r, p[mvpis_short])) {          /* "long" path, magnitude ≥ 8 */
    x = 0;
    for (i = 0; i < 3; i++)                 /* bits 0,1,2                  */
        x += vp8_read(r, p[MVPbits + i]) << i;
    for (i = mvlong_width - 1; i > 3; i--)  /* bits 9..4 (top-down!)       */
        x += vp8_read(r, p[MVPbits + i]) << i;
    /* bit 3 only if any higher bit is set: */
    if (!(x & 0xFFF0) || vp8_read(r, p[MVPbits + 3])) x += 8;
}
else {                                       /* "short" path, magnitude < 8 */
    x = vp8_treed_read(r, vp8_small_mvtree, p + MVPshort);
}
if (x && vp8_read(r, p[MVPsign])) x = -x;
return x;                                    /* 1/4-pel units               */
```

Notes:
- The two short-vs-long branches give VP8 a Golomb-like code: small
  magnitudes use a flat 8-way tree code; larger ones use up to 10
  independently-coded magnitude bits.
- Bits 4..9 are read top-down so the dependence on bit 3's
  "implicit-1" rule is correctly honoured.
- `mvlong_width = 10`, `mvnum_short = 8`, `MVPshort = 2`,
  `MVPbits = 9`, `MVPcount = 19` (entropymv.h:24–36).

### 16.15 Tokens: per-MB coefficient bitstream (RFC §13)

Tokens live in the round-robin token partitions, not the residual
partition. `vp8_decode_mb_tokens` (vp8/decoder/detokenize.c) iterates
the 25 4x4 blocks of one MB in this order (see §9.3):

```
if has_Y2(mb):                  /* mbmi.mode != B_PRED and mbmi.mode != SPLIT_MV */
    decode block 24 (Y2)        block_type = 1   first_coef_band = 0
for y in 0..15:
    decode Y block y            block_type = 0   first_coef_band = 1  (if has_Y2)
                                 block_type = 3   first_coef_band = 0  (else)
for uv in 16..23:
    decode UV block uv          block_type = 2   first_coef_band = 0
```

Per block, the inner loop in `GetCoeffs` (detokenize.c:84) walks
the coefficient tree `vp8_coef_tree` (entropy.c:70) at every
zig-zag position:

```
For coefficient position n = first_coef_band, first_coef_band+1, …, 15:
    probs = coef_probs[block_type][vp8_coef_bands[n]][prev_coef_context]

    token = treed_read(coef_tree, probs):
        DCT_EOB_TOKEN  → end this block; remaining coefs are 0
        ZERO_TOKEN     → 0;     prev_coef_context := 0;
        ONE_TOKEN      → ±1;    extra bit = sign;   prev_coef_context := 1
        TWO_TOKEN      → ±2;    extra bit = sign;   prev_coef_context := 2
        THREE_TOKEN    → ±3;    extra bit = sign;   prev_coef_context := 2
        FOUR_TOKEN     → ±4;    extra bit = sign;   prev_coef_context := 2
        DCT_VAL_CATEGORY1 → 5..6,            +1 magnitude bit + sign,  ctx := 2
        DCT_VAL_CATEGORY2 → 7..10,           +2 magnitude bits + sign, ctx := 2
        DCT_VAL_CATEGORY3 → 11..18,          +3 magnitude bits + sign, ctx := 2
        DCT_VAL_CATEGORY4 → 19..34,          +4 magnitude bits + sign, ctx := 2
        DCT_VAL_CATEGORY5 → 35..66,          +5 magnitude bits + sign, ctx := 2
        DCT_VAL_CATEGORY6 → 67..2114,       +11 magnitude bits + sign, ctx := 2

    qcoeff[ kZigzag[n] ] = signed magnitude
```

The "extra-bit" trees for each category have fixed probabilities;
they're stored as small binary trees (cat1..cat6 in entropy.c:127),
not as `vp8_read_literal` calls — each bit has its own probability.
This is how VP8 squeezes large magnitudes without forcing them into a
flat 11-bit literal.

`prev_coef_context` (0 / 1 / 2) feeds the next coefficient's
probability lookup, so consecutive ±1 coefficients are cheaper to
code than alternating large and small ones.

EOB token at position 0 means "block has no coefficients at all"; if
the MB-level `mb_skip_coeff` was 1, this whole loop is skipped and
the EOB / entropy state is reset via `vp8_reset_mb_tokens_context`
(decodeframe.c:104-105). For B_PRED MBs, EOBs are additionally cleared
explicitly with `memset(xd->eobs, 0, 25)` at decodeframe.c:162.
`mb_skip_coeff` itself is read at decodemv.c:503.

### 16.16 Cross-reference table

| RFC 6386 section                                | libvpx file               | Line(s) |
|--------------------------------------------------|---------------------------|---------|
| §7. Boolean Entropy Decoder                      | vp8/decoder/dboolhuff.h   | 54–91   |
| §8. Tree Coding                                  | vp8/decoder/treereader.h  | 30–39   |
| §9.1 Uncompressed Data Chunk — frame tag         | decodeframe.c             | 921–925 |
| §9.1 Uncompressed — sync code, W×H, scales       | decodeframe.c             | 943–951 |
| §9.2 Color space & clamp_type                    | decodeframe.c             | 981–984 |
| §9.3 Segmentation                                | decodeframe.c             | 987–1034|
| §9.4 Loop-filter header                          | decodeframe.c             | 1037–1075|
| §9.5 Token partition count                       | decodeframe.c             | 738–742 |
| §9.5 Token partition size table (raw)            | decodeframe.c             | 684, 752–789 |
| §9.6 Quantizer indices                           | decodeframe.c             | 1081–1098|
| §9.7 Refresh GF/ARF + buffer-copy flags          | decodeframe.c             | 1104–1147|
| §9.8 Refresh last + sign bias + entropy update   | decodeframe.c             | 1145–1166|
| §9.9 / §13.4 Coef prob updates                   | decodeframe.c             | 1172–1187|
| §9.10 / §9.11 Remaining header (skip, mode, MV)  | decodemv.c                | 122–163 |
| §10. Segment-based feature adjustments           | decodemv.c                | 475–487 |
| §11. Key-frame MB prediction                     | decodemv.c                | 42–62   |
| §13. DCT coefficient decoding                    | detokenize.c              | 84–210  |
| §13.5 Default token prob table                   | vp8/common/default_coef_probs.h | 20–* |
| §14.1 Dequantization                             | vp8/common/quant_common.c | 37–130  |
| §14.2 IDCT                                       | vp8/common/idctllm.c      | 29–103  |
| §14.3 WHT inversion                              | vp8/common/idctllm.c      | 127–175 |
| §14.5 Predictor + residue summation              | vp8/common/idct_blk.c     | 15–34   |
| §15. Loop filter                                 | vp8/common/vp8_loopfilter.c | 263–382 |
| §15.4 LF control-parameter derivation            | vp8/common/vp8_loopfilter.c | 49–75 |
| §16. Inter-frame MB prediction                   | decodemv.c                | 284–473 |
| §16.4 SPLIT_MV split-shape tree                  | decodemv.c                | 199–207 |
| §17.1 MV component coding                        | decodemv.c                | 64–89   |
| §17.2 MV probability updates                     | decodemv.c                | 96–112  |
| §18.3 Sub-pixel interpolation                    | vp8/common/filter.c       | 20–192  |

### 16.17 Practical gotchas

A handful of things bite implementers of VP8 parsers because the spec
text is subtle:

1. **The token-partition size table is *raw*, not bool-coded.** A
   first-time reader naturally expects it to follow `multi_token_partition`
   in the bool stream. It does not. Those 3-byte sizes are at fixed
   byte offsets inside the input buffer.
2. **`first_partition_length_in_bytes` covers the *residual*
   partition only** (header + modes + MVs), not the token partitions.
   The size table sits *after* it; the token partitions sit after the
   size table.
3. **The bool decoder is little-endian by accident.** The arithmetic
   coder pulls input bytes most-significant-bit first into `value`,
   so the bitstream is morally MSB-first. But the multi-byte literals
   embedded outside the bool decoder (width, height,
   `first_partition_length_in_bytes`, partition sizes) are
   little-endian. Easy to mix up.
4. **`prob = 0` is forbidden.** The MV-prob update path encodes 7-bit
   probabilities then doubles them, falling back to 1 when 0 — see
   decodemv.c:108. This ensures the bool decoder never gets `prob = 0`
   (which would make some branches require infinite bits).
5. **Inter-frame B_PRED uses unconditional bmode probs.** Key frames
   use the 3-D conditional table `vp8_kf_bmode_prob[A][L]` indexed by
   neighbor modes (decodemv.c:57). Inter frames just use
   `pc->fc.bmode_prob[9]` directly (decodemv.c:467). Two different
   tables — easy to confuse.
6. **MV magnitude bits 4..9 are read top-down**, not bottom-up
   (decodemv.c:77–79). Bit 3 is also conditional on whether any
   higher bit is set (line 81). The result *looks* like a 10-bit
   literal but isn't.
7. **`refresh_entropy_probs == 0` snapshots `pc->fc` early, restores
   at end-of-frame** (decodeframe.c:1157, 1246). A naïve implementation
   that just applied the prob updates and forgot to roll back would
   silently corrupt all subsequent frames.
8. **`mb_skip_coeff` short-circuits the entire 25-block coefficient
   pass** via `vp8_reset_mb_tokens_context` (decodeframe.c:104-105);
   B_PRED additionally calls `memset(xd->eobs, 0, 25)` at
   decodeframe.c:162. The internal sub-block loop-filter edges are
   then skipped only if the MB is also *not* B_PRED and *not* SPLITMV
   (§11.4 here / RFC §15.4).
9. **Chroma MV under SPLITMV** is the arithmetic mean of four luma
   sub-block MVs with VP8's specific signed-rounding rule
   (reconinter.c:467–483) — *not* (luma_mv >> 1) as one might
   reasonably expect. Encoders must use exactly the same averaging
   formula or the chroma planes will diverge.
10. **The "version" field toggles four independent flags**, not a
    single "progressively simpler" knob. From `vp8_setup_version`
    (alloccommon.c:134):

    | version | sub-pel filter | loop filter | LF disabled | full-pel MC |
    |---------|----------------|-------------|-------------|-------------|
    | 0       | 6-tap (bicubic)| NORMAL      | no          | no          |
    | 1       | bilinear       | SIMPLE      | no          | no          |
    | 2       | bilinear       | NORMAL      | **yes**     | no          |
    | 3       | bilinear       | SIMPLE      | yes         | **yes**     |
    | 4-7     | reserved (treated as v0 by libvpx)                       |

    A decoder that assumes "v0 = 6-tap, vN = bilinear" alone will
    mis-decode v2/v3 because it will still run the loop filter on
    streams that disabled it, and it will still do fractional-pel MC
    on v3 streams that asked for full-pel.

---

## Appendix: File-to-section map

Every source file actually consulted while writing this document,
grouped by directory, with the sections that cite it.

### Public API (`vpx/`, `vpx/src/`, `vpx/internal/`)

| File                                       | Sections                       |
|--------------------------------------------|--------------------------------|
| `vpx/vpx_codec.h`, `vpx_decoder.h`         | §2                             |
| `vpx/vp8dx.h`                              | §2                             |
| `vpx/vpx_image.h`, `vpx_frame_buffer.h`    | §2, §10                        |
| `vpx/internal/vpx_codec_internal.h`        | §2, §15                        |
| `vpx/src/vpx_codec.c`, `vpx_decoder.c`     | §2                             |
| `vpx/src/vpx_image.c`                      | §10                            |

### VP8 codec interface & decoder driver (`vp8/`, `vp8/decoder/`)

| File                                       | Sections                       |
|--------------------------------------------|--------------------------------|
| `vp8/vp8_dx_iface.c`                       | §2, §10, §15                   |
| `vp8/decoder/onyxd_if.c`                   | §2, §10, §13, §14, §15         |
| `vp8/decoder/onyxd_int.h`                  | §3, §6                         |
| `vp8/decoder/decodeframe.c`                | §1, §6, §9, §11, §16           |
| `vp8/decoder/decodemv.c`                   | §7, §8, §16                    |
| `vp8/decoder/detokenize.c`, `detokenize.h` | §9, §16                        |
| `vp8/decoder/dboolhuff.c`, `dboolhuff.h`   | §5, §16                        |
| `vp8/decoder/treereader.h`                 | §5, §16                        |

### VP8 common code (`vp8/common/`)

| File                                       | Sections                       |
|--------------------------------------------|--------------------------------|
| `vp8/common/onyxc_int.h`                   | §3, §6, §10                    |
| `vp8/common/blockd.h`, `blockd.c`          | §3, §4, §7, §8, §11, §16       |
| `vp8/common/mv.h`                          | §3, §8                         |
| `vp8/common/mbpitch.c`                     | §3, §4                         |
| `vp8/common/alloccommon.c`                 | §3, §10, §12, §14              |
| `vp8/common/setupintrarecon.c`             | §7                             |
| `vp8/common/reconintra.c`, `reconintra.h`  | §7                             |
| `vp8/common/reconintra4x4.c`, `reconintra4x4.h` | §7                        |
| `vp8/common/reconinter.c`, `reconinter.h`  | §8                             |
| `vp8/common/findnearmv.c`, `findnearmv.h`  | §8, §16                        |
| `vp8/common/filter.c`, `filter.h`          | §8                             |
| `vp8/common/modecont.c`, `modecont.h`      | §8, §16                        |
| `vp8/common/entropy.c`, `entropy.h`        | §5, §9, §16                    |
| `vp8/common/entropymode.h`                 | §7, §16                        |
| `vp8/common/entropymv.h`                   | §8, §16                        |
| `vp8/common/default_coef_probs.h`          | §9, §16                        |
| `vp8/common/coefupdateprobs.h`             | §9, §16                        |
| `vp8/common/quant_common.c`                | §9, §16                        |
| `vp8/common/idctllm.c`                     | §4, §9, §16                    |
| `vp8/common/idct_blk.c`                    | §9, §16                        |
| `vp8/common/invtrans.h`                    | §4, §9                         |
| `vp8/common/vp8_loopfilter.c`              | §11, §16                       |
| `vp8/common/loopfilter_filters.c`          | §11                            |
| `vp8/common/loopfilter.h`                  | §11                            |
| `vp8/common/swapyv12buffer.c`              | §10                            |
| `vp8/common/extend.c`                      | §10                            |
| `vp8/common/treecoder.c`, `treecoder.h`    | §5, §16                        |
| `vp8/common/rtcd.c`                        | §14                            |
| `vp8/common/rtcd_defs.pl`                  | §7, §8, §11, §14               |
| `vp8/common/generic/systemdependent.c`     | §13, §14                       |
| `vp8/common/threading.h`                   | §13                            |

### Cross-codec utilities (`vpx_dsp/`, `vpx_mem/`, `vpx_scale/`, `vpx_util/`, `vpx_ports/`)

| File                                       | Sections                       |
|--------------------------------------------|--------------------------------|
| `vpx_dsp/intrapred.c`                      | §7                             |
| `vpx_dsp/bitreader_buffer.c`, `.h`         | §5, §6                         |
| `vpx_dsp/bitreader.c`, `.h`                | §5                             |
| `vpx_dsp/prob.c`, `prob.h`                 | §5                             |
| `vpx_dsp/vpx_dsp_rtcd_defs.pl`             | §7, §8, §14                    |
| `vpx_mem/vpx_mem.c`, `vpx_mem.h`           | §12                            |
| `vpx_scale/yv12config.h`                   | §10                            |
| `vpx_scale/generic/yv12config.c`           | §10, §12                       |
| `vpx_scale/generic/yv12extend.c`           | §8, §10                        |
| `vpx_util/vpx_thread.c`, `vpx_thread.h`    | §13                            |
| `vpx_util/vpx_pthread.h`                   | §13                            |
| `vpx_ports/vpx_once.h`                     | §14                            |
| `vpx_ports/system_state.h`                 | §14                            |
| `vpx_ports/compiler_attributes.h`          | §14                            |

