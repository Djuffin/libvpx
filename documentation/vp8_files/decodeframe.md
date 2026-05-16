# `vp8/decoder/decodeframe.c` — the frame-level driver

## Role in the decoder

`decodeframe.c` is the orchestrator. Everything else in `vp8/decoder/`
specialises: `dboolhuff.c` does arithmetic decoding, `decodemv.c` parses
modes and motion vectors, `detokenize.c` parses residual coefficients,
`onyxd_if.c` manages the decoder instance. This file's job is to **stitch
them together for one frame**: parse the uncompressed frame tag, open the
residual ("first") partition, parse the compressed header end-to-end, lay
out and open the token partitions, then walk the macroblock grid in
raster order — for each MB pulling tokens from the right round-robin
partition, dispatching intra or inter prediction, dequantising, inverse-
transforming the residual, adding it to the predictor, and, one row
behind, running the loop filter.

`vp8_decode_frame` is the single public entry point of this file (and is
declared in `vp8/decoder/onyxd_int.h:133`). It is called from
`vp8_decode_frame_wrapper` / the public `vpx_codec_decode` path in
`onyxd_if.c` once per access unit. By the time it returns, the new
reconstructed picture sits in `pbi->dec_fb_ref[INTRA_FRAME]` (the
`new_fb_idx` slot of the 4-buffer pool described in §10.2 of
`vp8_technical_overview.md`) with its 32-pixel border replicated and is
ready for `vp8_show_frame_wrapper` to publish it to the user.

The file also owns the two **dequantizer-bootstrap** helpers
(`vp8cx_init_de_quantizer`, `vp8_mb_init_dequantizer`) that are reused
elsewhere in the decoder (including by `decodemv.c` indirectly through
the per-frame setup) — they are the only non-`static` symbols besides
`vp8_decode_frame`.

The remaining functions are all `static` helpers — frame-tag parsing
(`get_delta_q`, `read_partition_size`, `read_is_valid`,
`read_available_partition_size`, `setup_token_decoder`), state
initialisation (`init_frame`), the per-MB worker (`decode_macroblock`),
the row-by-row driver (`decode_mb_rows`), and three local C reference
implementations of YV12 border extension
(`yv12_extend_frame_top_c`, `yv12_extend_frame_bottom_c`,
`yv12_extend_frame_left_right_c`). They are documented below in the
order that an actual decode visits them.

---

## Bootstrapping the dequantiser

### `vp8cx_init_de_quantizer`

The 7-bit `base_qindex` parsed at decodeframe.c:1085 is just a table
index; the actual dequantisation multipliers come from RFC 6386 §9.6 /
Table 17 (Y-AC, Y-DC, Y2-DC, Y2-AC, UV-DC, UV-AC). This function
materialises **all six rows of all 128 entries** into
`pc->Y1dequant[]`, `pc->Y2dequant[]`, `pc->UVdequant[]` (declared
`DECLARE_ALIGNED(16, short, …[QINDEX_RANGE][2])` in
`onyxc_int.h:65–67`):

```c
pc->Y1dequant[Q][0] = (short)vp8_dc_quant   (Q, pc->y1dc_delta_q);
pc->Y2dequant[Q][0] = (short)vp8_dc2quant   (Q, pc->y2dc_delta_q);
pc->UVdequant[Q][0] = (short)vp8_dc_uv_quant(Q, pc->uvdc_delta_q);
pc->Y1dequant[Q][1] = (short)vp8_ac_yquant  (Q);
pc->Y2dequant[Q][1] = (short)vp8_ac2quant   (Q, pc->y2ac_delta_q);
pc->UVdequant[Q][1] = (short)vp8_ac_uv_quant(Q, pc->uvac_delta_q);
```
(decodeframe.c:47–53)

The `[0]` slot is the DC multiplier for that plane, the `[1]` slot is
the AC multiplier; the AC for luma has no per-component delta in
VP8 (so `vp8_ac_yquant` takes no `Delta` argument), all others do. The
six `vp8_*quant` functions live in `vp8/common/quant_common.c` and just
index the RFC tables clamped to `[0, MAXQ]`.

This function is only ever called **when one of the five delta-Q values
in the header actually changes** — `vp8_decode_frame` accumulates a
`q_update` flag through five `get_delta_q` calls and only rebuilds the
table when set (decodeframe.c:1083–1094). Re-running it for every
frame would be wasteful since the deltas are persistent across frames
unless re-signalled.

### `vp8_mb_init_dequantizer`

Picks the right `QIndex` row for one macroblock and copies the 16-entry
DC/AC pattern into the four scratch arrays the per-MB reconstruction
kernels read (`xd->dequant_y1`, `xd->dequant_y1_dc`, `xd->dequant_y2`,
`xd->dequant_uv`, each `DECLARE_ALIGNED(16, short, …[16])` in
`blockd.h:215–218`). When segmentation is on, `QIndex` is the value
chosen for this MB's segment — either absolute (`SEGMENT_ABSDATA`) or
`base_qindex + delta` (`SEGMENT_DELTADATA`) — clamped to `[0, MAXQ]`:

```c
QIndex = (QIndex >= 0) ? ((QIndex <= MAXQ) ? QIndex : MAXQ) : 0;
```
(decodeframe.c:75)

Note the asymmetry the code creates at line 82: `xd->dequant_y1_dc[0]
= 1`. The 16-entry `dequant_y1_dc[]` differs from `dequant_y1[]` only
in slot 0 (the DC term), and only when the macroblock has a 2nd-order
Y2 block — see §9.5 of the overview. Setting the DC multiplier to 1
lets `vp8_dequant_idct_add_y_block` re-use a single inverse-transform
kernel for both AC-only and AC+DC cases by feeding it the already-
walsh-inverted DC residual in qcoeff slot 0 (which is in pixel-domain
units after `vp8_short_inv_walsh4x4` and must not be multiplied
again). The override at decodeframe.c:222 (`DQC = xd->dequant_y1_dc`)
is the consumer.

For the frame-level case, `vp8_decode_frame` calls this once at line
1097 (to set the default-segment dequant for any MBs that don't have
segmentation enabled), and `decode_macroblock` calls it again only
when `xd->segmentation_enabled` is true (decodeframe.c:116) — that's
the per-MB cost of segmentation.

---

## Compressed-header helpers

### `get_delta_q`

VP8's quantiser deltas (`y1dc`, `y2dc`, `y2ac`, `uvdc`, `uvac`) are
optional sign-magnitude fields with this layout (RFC 6386 §9.6):

```c
if (vp8_read_bit(bc)) {
    ret_val = vp8_read_literal(bc, 4);
    if (vp8_read_bit(bc)) ret_val = -ret_val;
}
```
(decodeframe.c:238–242)

So: 1 bit "present?", and if set 4 bits of magnitude plus a sign bit.
If absent the previous frame's value persists. The function compares
the freshly read value against the supplied `prev` and sets
`*q_update = 1` if they differ; the caller uses that to decide whether
to rebuild the dequantizer tables (the OR-accumulator pattern at
decodeframe.c:1088–1094). The "delta" semantics are why
`y1dc_delta_q` etc. live on `VP8_COMMON` and not on the frame: they
must carry across.

### `read_partition_size`

Reads a single 24-bit little-endian length from the token-partition
size table (the raw, non-bool-coded block that lives between the
residual partition and the first token partition — RFC 6386 §9.5).
Calls out through `pbi->decrypt_cb` if the bitstream is encrypted
(`vpx_decrypt_cb` is the application-provided AES-CTR-style hook from
`vpx/vp8dx.h`):

```c
if (pbi->decrypt_cb) {
    pbi->decrypt_cb(pbi->decrypt_state, cx_size, temp, 3);
    cx_size = temp;
}
return cx_size[0] + (cx_size[1] << 8) + (cx_size[2] << 16);
```
(decodeframe.c:666–670)

The three-byte unsigned LE format caps a single token partition at 16
MiB. Note that the decrypt callback decrypts into a 3-byte scratch
buffer `temp[3]` on the caller's stack — the only place a 3-byte
buffer is enough — keeping the API uniform with the
`pbi->fragments.ptrs[]` arena which is fed encrypted.

### `read_is_valid`

A one-line bounds check used wherever the parser is about to read `len`
bytes starting at `start` from a buffer that ends at `end`:

```c
return len != 0 && end > start && len <= (size_t)(end - start);
```
(decodeframe.c:675)

The `len != 0` clause rejects zero-length partitions outright. The
ordering — `end > start` before the subtraction — avoids signed
overflow on the `ptrdiff_t` that would otherwise be negative.

### `read_available_partition_size`

Compute the byte length of the `i`-th token partition and validate it
against the fragment we are currently carving. For all but the last
partition the length comes from the size table; the last partition's
length is implicit (the remainder of the fragment). Two failure modes
are distinguished:

1. The 3-byte entry in the size table itself is past the fragment end.
2. The partition described by a valid size entry overflows the
   fragment.

In ordinary playback either is a corrupt-frame error
(`VPX_CODEC_CORRUPT_FRAME` via `vpx_internal_error`, which longjmps
out via `pc->error.jmp` — see §15 of the overview). With error
concealment active (`pbi->ec_active`) we instead clip to the bytes
that *are* available and continue, letting the bool decoder discover
the truncation later via `vp8dx_bool_error`.

Note also the negative `bytes_left` guard at the top
(decodeframe.c:687) — if a previous loop iteration miscounted, this
fires before we start indexing the size table.

### `setup_token_decoder`

This is the carving loop. The caller passes `token_part_sizes`, a
pointer to the first byte *after* the residual partition (which is
also the first byte of the size table). The job:

1. Read the 2-bit `multi_token_partition` from the residual partition's
   bool decoder at `pbi->mbc[8]` (the residual partition lives at slot
   8 of `pbi->mbc[MAX_PARTITIONS=9]` — see `onyxc_int.h:38` and
   `onyxd_int.h:67`). `num_token_partitions = 1 << val ∈ {1,2,4,8}`.
2. Re-shape `pbi->fragments` (a list of `{ptr, size}` buffer chunks
   describing the original input, see `onyxd_int.h:41–46`) so that
   each fragment corresponds to exactly one partition. In normal
   single-packet decoding there is one input fragment; the loop
   splits it into `num_token_partitions + 1` (one for the residual
   + N for tokens). With WebM/RTP-style multi-fragment input each
   fragment may already span partition boundaries and the loop walks
   them.
3. Open each token partition with `vp8dx_start_decode` into
   `pbi->mbc[0..N-1]` (decodeframe.c:794–803), failing with
   `VPX_CODEC_MEM_ERROR` if the bool decoder can't initialise (it
   only ever fails if `source_sz < 1`, since `BOOL_DECODER` is a
   plain struct with no dynamic allocation).
4. Clamp `pbi->decoding_thread_count` so it never exceeds either
   `num_token_partitions - 1` or `mb_rows - 1`. This is the only
   place that enforces the threading invariants and it is gated on
   `CONFIG_MULTITHREAD`.

The special-case path at decodeframe.c:752–769 deals with "the
first fragment contains both the residual partition data we already
parsed *and* the size table *and* the start of partition 0" — we have
to account for `3 * (num_token_partitions - 1)` bytes of size table
in the fragment-size bookkeeping before treating the rest as
partition data.

The residual partition's bool decoder at `pbi->mbc[8]` was already
opened by `vp8_decode_frame` (decodeframe.c:976) before this is
called; this function reuses it to read those 2 layout bits and then
ignores it. The asymmetric storage (residual at the *highest* index,
tokens at low indices) lets `vp8_decode_mb_tokens` switch decoders
purely with `xd->current_bc = &pbi->mbc[ibc]` indexed by row mod N
without ever colliding with the residual decoder.

---

## Frame-state initialisation

### `init_frame`

Called once per frame from `vp8_decode_frame` (decodeframe.c:974)
after the frame tag has been parsed (so `pc->frame_type` is known) and
before the compressed header bool stream is opened. The key-frame
branch resets *everything* the bitstream might otherwise leave stale:

- Copy `vp8_default_mv_context` (entropymv.c:30) into `pc->fc.mvc` —
  the per-component MV probability table.
- Initialise `pc->fc.ymode_prob`, `uv_mode_prob`, `bmode_prob`,
  `sub_mv_ref_prob` from the hard-coded defaults
  (`vp8_init_mbmode_probs`, entropymode.c).
- Reset all 1056 coefficient-update probabilities to RFC defaults
  (`vp8_default_coef_probs`, entropy.c:145).
- Zero `xd->segment_feature_data[]`, the loop-filter ref/mode deltas,
  and set `mb_segment_abs_delta = SEGMENT_DELTADATA`.
- Mark all three reference buffers for refresh
  (`refresh_golden_frame = refresh_alt_ref_frame = 1`), skip the
  buffer copies, and clear the `ref_frame_sign_bias[GOLDEN|ALTREF]`
  fields (sign bias is meaningless on a key frame because those
  references are about to be overwritten anyway).

The inter branch is much smaller. It picks the **sub-pel
interpolator** based on `pc->use_bilinear_mc_filter` (set by
`vp8_setup_version` from the `version` bits in the frame tag —
versions 0–1 select 6-tap, 2–3 select bilinear). The 4 function
pointers `subpixel_predict{,8x4,8x8,16x16}` are written into
`MACROBLOCKD` and then dispatched by `vp8_build_inter_predictors_mb`
(reconinter.c). The other inter-frame work — re-enabling error
concealment after the first key frame is decoded — is one cheap
toggle.

Finally, both branches set `xd->left_context = &pc->left_context`,
plant `xd->mode_info_context = pc->mi` at the top-left visible MB
(see §3.2 of the overview for the MODE_INFO grid layout), reset
`xd->corrupted`, and choose `xd->fullpixel_mask` (= ~0 normally, ~7
when `pc->full_pixel` is set — that bit, in turn, can only be set by
the version field; it forces 4-bit MV-quarter-pel rounding to
integer-pel before reconstruction).

---

## YV12 border extension (C reference)

The three `yv12_extend_frame_*_c` functions are local C implementations
of border-extension copies that the decoder uses to keep the
reconstructed buffer's outer border valid for the next frame's inter
prediction. They are *not* the same as `vp8_yv12_extend_frame_borders`
(which lives in `vpx_scale/generic/yv12extend.c` and is called only at
the very end of multithreaded paths) — these are interleaved with the
per-row decode loop so the loop filter and the border copy can overlap.

A `YV12_BUFFER_CONFIG` carries `border` (a power-of-two count of pixels
of padding around the active picture, typically 32) plus separate
`{y,u,v}_buffer`, `{y,uv}_stride`, `{y,uv}_height`, `{y,uv}_width`.
The chroma border is implicitly `border / 2` because the chroma plane
is 4:2:0 subsampled. All three helpers honour that.

### `yv12_extend_frame_top_c`

Copies the topmost scanline of each plane (including its already-
extended left and right horizontal padding — note `src_ptr1 =
y_buffer - Border` at decodeframe.c:268) up by `Border` rows, plane by
plane. Run only once per frame, *after* the last MB row has been
decoded, the deblocker has finished, and the left/right padding has
been replicated (decodeframe.c:659).

### `yv12_extend_frame_bottom_c`

Symmetric to the above, but copies the bottommost scanline downward by
`Border` rows. Also run once per frame at the end (decodeframe.c:660).

### `yv12_extend_frame_left_right_c`

Extends one **strip** of MB rows (16 luma rows / 8 chroma rows)
horizontally. Unlike the top/bottom helpers, which take only the
buffer, this one takes explicit `y_src`/`u_src`/`v_src` pointers
addressing the first pixel of the strip to be extended. It writes
`Border` copies of the leftmost pixel into the left border and
`Border` copies of the rightmost pixel into the right border, for
each of the 16 (or 8) rows in the strip:

```c
memset(dest_ptr1, src_ptr1[0], Border);
memset(dest_ptr2, src_ptr2[0], Border);
```
(decodeframe.c:385–386)

The driver in `decode_mb_rows` runs this **two rows behind** the
decode cursor (one row behind the loop filter, which is one row behind
decode) so that the strip being extended is finalised: the loop
filter has already processed it, no later MB can write back to it,
and inter prediction in subsequent frames can sample arbitrary MVs in
the clamped range without bounds checking.

This is the "border extension after deblocking" arrangement described
in §10.3 of the overview, hand-inlined here to avoid an extra full-
frame pass.

---

## Per-row driver

### `decode_mb_rows`

The main loop. After all header parsing is done, this function walks
the macroblock grid in raster order. It is shaped by four cross-
cutting concerns that all have to march in lockstep:

1. **Token-partition round-robin.** With `num_token_partitions = N`,
   MB row `r` reads tokens from partition `r mod N`. The pointer
   `xd->current_bc = &pbi->mbc[ibc]` is updated at the start of each
   row, `ibc` cycling `0..N-1`:

   ```c
   if (num_part > 1) {
       xd->current_bc = &pbi->mbc[ibc];
       ibc++;
       if (ibc == num_part) ibc = 0;
   }
   ```
   (decodeframe.c:487–492)

   With `N = 1` there is just `pbi->mbc[0]` and the assignment is
   skipped (it is set once at the end of `vp8_decode_frame`,
   line 1079).

2. **Entropy / reconstruction context.** `xd->above_context` is a
   pointer along `pc->above_context`, advanced one column per MB
   (line 595). `xd->left_context` is the single
   `ENTROPY_CONTEXT_PLANES` on `VP8_COMMON`, zeroed at the start of
   each row (line 499). `recon_above[]` / `recon_left[]` hold the
   pixel pointers feeding the intra predictors and are shuffled in
   tandem (lines 506–524 set them up for the start of a row;
   lines 583–588 advance them per MB).

3. **Edge distances in 1/8-pel.** `mb_to_{left,right,top,bottom}_edge`
   are computed in 1/8-pel units (a.k.a. "Q3.3" — the shifts at
   lines 503, 504, 531, 532 are `<< 3`) because MVs are stored in
   1/8-pel and the MV-clamping path in `findnearmv.c` does its
   comparisons directly against these.

4. **Loop filter / border extension lag.** The loop filter for row
   `r` needs row `r+1`'s top edge to filter the horizontal edge
   between them, so it runs *after* row `r+1`'s MBs are reconstructed
   (i.e., one row behind decode). The left/right border extension
   for row `r` runs *after* the loop filter has finished with row
   `r`, i.e., two rows behind decode. Three matched cursors
   `lf_dst[]` / `lf_mic` (loop filter), `eb_dst[]` (extension), and
   `dst_buffer[]` (decode) advance independently. The closing
   block at lines 642–660 flushes the lag at end-of-frame.

The per-MB body (lines 526–596) is itself just bookkeeping for the
edge distances, predictor pointers, and reference-frame pointers,
followed by a single call to `decode_macroblock` (line 575). Inter
frames use the pre-built `ref_buffer[ref][0..2]` arrays — collected
once at the top of the function (lines 463–471) so the inner loop
doesn't keep redoing the indirection through `pbi->dec_fb_ref[]`.

The corrupted-frame propagation deserves a note: `xd->corrupted` is
OR-accumulated with the reference frame's `corrupted` flag (line
573) **before** decoding the MB, and with the bool decoder's error
flag (line 581) **after**. So once any partition glitches, or once
we are decoding atop a corrupted reference, the flag stays set for
the rest of the frame, eventually surfacing at decodeframe.c:1231.

When error concealment is enabled, the inner block at lines 534–554
detects "intra MB whose coefficients are corrupt" *before* calling
the macroblock decoder, and replaces the MB's MVs with neighbour-
interpolated values via `vp8_interpolate_motion` — that way the
predictor used inside `decode_macroblock` will at least be a
sensible inter-prediction rather than a garbled intra mash.

---

## Per-macroblock worker

### `decode_macroblock`

Decodes one macroblock end-to-end: tokens → predictor → residual →
add. The control flow is dense but always proceeds in the same five
phases:

**1. Token decode.** If the MB is marked `mb_skip_coeff` (no
non-zero coefficients in any of its 25 blocks), only the entropy
context needs resetting:

```c
if (xd->mode_info_context->mbmi.mb_skip_coeff) {
    vp8_reset_mb_tokens_context(xd);
}
```
(decodeframe.c:104–105)

Otherwise `vp8_decode_mb_tokens` (detokenize.c) walks the token tree
for the Y2 block (if present), then the 16 Y blocks, then the 8
chroma blocks, populating `xd->qcoeff[]` (a 400-short flat array
covering all 25 blocks — `blockd.h:211`) and `xd->eobs[]` (the
end-of-block index for each, 25 bytes). The returned `eobtotal` is
used at line 111 to *retroactively* set `mb_skip_coeff` if every
block turned out empty — that lets the deblocker skip the MB on its
fast path.

**2. Per-segment dequant.** If segmentation is enabled, the
quantiser may have changed per-MB; `vp8_mb_init_dequantizer`
(decodeframe.c:116) rebuilds `xd->dequant_*[]`.

**3. Optional error-concealment short-circuit.** The
`CONFIG_ERROR_CONCEALMENT` block at lines 118–144 detects two
failure cases — corrupt residual carrying over from a previous MB
(when partitions are not independent), and post-this-point corrupt
modes/MVs (`mvs_corrupt_from_mb`) — zeros the coefficient buffer and
the EOBs, sets `corruption_detected = 1`, and lets the predictor
run; then the early return at line 195 skips the residual add. The
reconstruction is "predictor only, no residual" — coarse, but better
than visible garbage.

**4. Prediction.** Three branches:

- **Intra, non-B_PRED:** `vp8_build_intra_predictors_mbuv_s` (UV) +
  `vp8_build_intra_predictors_mby_s` (Y). The "_s" suffix means
  "self" — they write directly to `xd->dst.{y,u,v}_buffer` instead
  of to the separate `predictor[]` staging buffer used by the
  encoder. The Y predictor uses `xd->recon_above[0]` /
  `xd->recon_left[0]` for its top/left samples; these were just
  shuffled by `decode_mb_rows` so they point at the just-decoded
  pixels of the MB above / to the left.

- **Intra, B_PRED:** the predictor is *per 4×4 block* (16 sub-
  blocks, each with its own mode chosen from `vp8_intra4x4_predict`'s
  10 modes). Crucially this loop **also adds the residual block-by-
  block** rather than as a single MB-wide pass:

  ```c
  if (xd->eobs[i]) {
      if (xd->eobs[i] > 1)
          vp8_dequant_idct_add(b->qcoeff, DQC, dst, dst_stride);
      else
          vp8_dc_only_idct_add(b->qcoeff[0] * DQC[0], dst, …);
  }
  ```
  (decodeframe.c:178–186)

  This is because B_PRED's neighbour samples for block `i+1` are the
  *reconstructed* samples of block `i` (not just the predictor),
  which forces a sequential prediction-then-add per block. The
  DC-only fast path at line 182 skips the full 4×4 IDCT when only
  the DC coefficient is non-zero. The `intra_prediction_down_copy`
  call at line 164 (defined in `vp8/common/reconintra4x4.h`)
  duplicates row 3 of the MB above into row 4 — needed for diagonal
  predictors `B_LD_PRED` and `B_VL_PRED` which reach 4 pixels past
  the MB-above's right edge.

- **Inter:** `vp8_build_inter_predictors_mb` (reconinter.c) reads
  through `xd->pre.{y,u,v}_buffer` (set up by `decode_mb_rows` from
  `ref_buffer[mbmi.ref_frame]`), applies the chosen sub-pel filter
  via `xd->subpixel_predict*`, and writes the predictor to
  `xd->dst.*_buffer`.

**5. Residual add.** Skipped entirely if `mb_skip_coeff` is set.
Otherwise:

- **Non-B_PRED.** If the MB is not SPLITMV, it carries a 2nd-order
  Y2 DC block. The decoder dequantises Y2 (`vp8_dequantize_b`),
  inverse-Walsh-transforms it (`vp8_short_inv_walsh4x4`) writing the
  16 reconstructed DCs into slots 0, 16, 32, … of `xd->qcoeff` (the
  DC of each Y block), and then switches the Y dequant to
  `xd->dequant_y1_dc` (which has `[0] = 1` — see
  `vp8_mb_init_dequantizer` above) so that the per-block IDCT adder
  treats those slots as already-pixel-valued. The DC-only fast
  variant `vp8_short_inv_walsh4x4_1` handles the case where Y2 had
  only the DC term.
  Then `vp8_dequant_idct_add_y_block` adds the 16 Y blocks at once.

- **Always:** `vp8_dequant_idct_add_uv_block` adds the 8 chroma
  blocks (`xd->qcoeff + 16*16` is the start of the chroma block
  coefficients in the flat 400-short array).

This is the only function in the file whose behaviour shape is
controlled by *all four* of: intra/inter, B_PRED vs not, SPLITMV vs
not, and segmentation on/off. Every other macroblock detail flows
out from `decode_mb_rows`'s setup.

---

## The orchestrator

### `vp8_decode_frame`

The public entry point. The structure mirrors the frame format chapter
of `vp8_technical_overview.md` (§16) section-by-section — that
document doubles as the line-by-line commentary for this function.
Here we describe the **non-parsing decisions** the function makes.

The function opens by snapshotting `pbi->independent_partitions` into
`prev_independent_partitions` (line 891) and clearing
`xd->corrupted`. The independent-partitions flag is recomputed during
coefficient-probability update parsing (lines 1181–1183): a frame is
"partition-independent" iff every coefficient probability ends up the
same in `PREV_COEF_CONTEXTS` slot 0 and 1 (and 1 and 2). If true and
the entropy probabilities aren't being refreshed, the flag is rolled
back at line 1247 — independent_partitions has to track the *probably*
shared state, not the transient state of a non-refreshed frame.

The 3-byte minimum-frame check at line 899 is the first error
boundary. A frame shorter than its mandatory frame tag is either an
error or, with EC active, becomes a synthesised "missing" inter frame
(`frame_type = INTER_FRAME`, `show_frame = 1`, zero-length partition).
The decoder will go on to estimate everything; the resulting picture
will be the previous frame nudged by the EC's MV interpolation.

The `data + 3` / `clear + 3` advance at lines 932–933 is the only
place where the parser pointer leaves the encrypted (`data`) and the
plaintext (`clear`) views in sync — both must advance together because
all subsequent **raw** reads happen via the plaintext `clear` pointer
into the 10-byte `clear_buffer[]` scratch decrypted at line 917.
After this point only the **bool-coded** reads happen (which take care
of decryption internally through `pbi->decrypt_cb` plumbed into
`vp8dx_start_decode` at line 977).

The "must start with a complete key frame" guard at line 965 is the
gate that distinguishes a recovered stream from an unstarted one. The
decoder cannot meaningfully reconstruct any inter frame until a key
frame's pixels are in the reference buffers; the early `return -1`
is the codec-API contract.

The bulk of the function from line 981 to line 1187 is straight
header parsing — colour space, segmentation, loop filter, token-
partition layout, quantiser, reference flags, entropy-refresh flag,
and the 1056 coefficient probabilities. Each block follows the same
shape: 1-bit "present?", optional payload, optional EC fallback when
the bit is lost mid-frame.

After `vp8_decode_mode_mvs` parses the per-MB modes/MVs (line 1193,
in decodemv.c), `pc->above_context` is zeroed (the row-zero token
context for the upcoming token-decode walk), and we dispatch:

```c
if (vpx_atomic_load_acquire(&pbi->b_multithreaded_rd) &&
    pc->multi_token_partition != ONE_PARTITION) {
    if (vp8mt_decode_mb_rows(pbi, xd)) { … error … }
    vp8_yv12_extend_frame_borders(yv12_fb_new);
    …
} else {
    decode_mb_rows(pbi);
}
```
(decodeframe.c:1207–1225)

The MT path lives in `threading.c`; the single-threaded path is
`decode_mb_rows` above. The MT path does its own end-of-frame border
extension because it doesn't have the row-by-row hand-extension that
`decode_mb_rows` does (it can't — rows finish out-of-order).

The final block (lines 1227–1248) collects corruption information
into the frame's `corrupted` field — both the residual decoder's
error state and the per-MB accumulator. If the very first key frame
is corrupted, we refuse to consider it decoded (line 1237 throws
`VPX_CODEC_CORRUPT_FRAME`), because every subsequent inter frame
would predict from garbage. If `refresh_entropy_probs` was zero, the
probability table is rolled back to the start-of-frame copy
(`pc->lfc`) so the next frame inherits the un-updated state.

The `PACKET_TESTING` blocks at lines 250–253 and 1250–1258 are
development hooks: when defined, the decoder dumps every input frame
to `decompressor.VP8` with a 4-byte length prefix, for replay by a
test harness. They are not part of the normal build.

---

## Cross-references

- **`vp8/decoder/decodemv.c`** — `vp8_decode_mode_mvs` (called at
  line 1193) parses the per-MB modes and MVs that `decode_macroblock`
  later consumes via `xd->mode_info_context->mbmi`.
- **`vp8/decoder/detokenize.c`** — `vp8_decode_mb_tokens` (line 108)
  and `vp8_reset_mb_tokens_context` (line 105) populate `xd->qcoeff`
  and `xd->eobs`.
- **`vp8/decoder/dboolhuff.c`** — `vp8dx_start_decode` (lines 796,
  976) initialises each partition's `BOOL_DECODER`; `vp8_read*`
  primitives are used throughout.
- **`vp8/common/quant_common.c`** — the six `vp8_*quant` functions
  consumed by `vp8cx_init_de_quantizer`.
- **`vp8/common/reconinter.c`** — `vp8_build_inter_predictors_mb`
  (line 190).
- **`vp8/common/reconintra.c` / `reconintra4x4.c`** —
  `vp8_build_intra_predictors_mby_s`, `vp8_build_intra_predictors_mbuv_s`,
  `vp8_intra4x4_predict`, and the inlined `intra_prediction_down_copy`.
- **`vp8/common/idct_blk.c` / `idctllm.c`** — `vp8_dequant_idct_add`,
  `vp8_dequant_idct_add_y_block`, `vp8_dequant_idct_add_uv_block`,
  `vp8_dc_only_idct_add`, `vp8_short_inv_walsh4x4{,_1}`.
- **`vp8/common/vp8_loopfilter.c`** — `vp8_loop_filter_frame_init`,
  `vp8_loop_filter_row_normal`, `vp8_loop_filter_row_simple` called
  from `decode_mb_rows`.
- **`vp8/common/entropy.c`** — `vp8_mb_feature_data_bits = {7, 6}`
  controls the magnitude width of `Q` vs `LF` segment deltas at line
  1006; `vp8_default_coef_probs` is the key-frame reset called from
  `init_frame`.
- **`vp8/common/entropymv.c`** — `vp8_default_mv_context` copied into
  `pc->fc.mvc` at line 824 on key frames.
- **`vp8/common/coefupdateprobs.h`** — `vp8_coef_update_probs[][][][]`
  is the per-node "should I update?" probability table read at line
  1178; almost all entries are 252, so the update block is usually
  near-empty.
- **`vp8/common/extend.h`** — `vp8_extend_mb_row` is the alternative
  border extension used per-row, distinct from the three local C
  helpers above.
- **`vp8/decoder/onyxd_if.c`** — owns `VP8D_COMP`'s lifetime,
  including `pbi->fragments`, and is the only caller of
  `vp8_decode_frame`.

The result of one successful `vp8_decode_frame` invocation is a
fully-reconstructed `YV12_BUFFER_CONFIG` at
`pbi->dec_fb_ref[INTRA_FRAME]` with border extended; `onyxd_if.c`
then runs `swap_frame_buffers` to rotate the reference pool
(LAST/GOLDEN/ALTREF) according to the refresh and copy flags this
file decoded into `pc->refresh_*` and `pc->copy_buffer_to_*`.
