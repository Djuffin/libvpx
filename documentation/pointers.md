# VP8 Decoder: Raw Pointers to Higher-Level Structures

This document provides context and a conversion-complexity guesstimate for the various raw pointers (`*mut T` / `*const T`) used in the VP8 decoder kernel to alias or share higher-level data structures (e.g., macroblocks, frame buffers, and contexts).

Pointers used exclusively for pixel, coefficient, or bitstream access (such as `y_buffer: *mut u8`, `qcoeff: *mut i16`, `eob: *mut i8`, or `buffer: *const u8`) are excluded from this analysis.

---

## 0. Completed refactors

The following raw pointers have already been replaced. They are listed here so future readers see what is already done and which patterns are precedent.

| Was | Now | Notes |
|---|---|---|
| `Vp8Common.frame_to_show: *mut Yv12BufferConfig` | `frame_to_show_idx: i32` (`-1` = none) | Plain index into `yv12_fb[]`. Explicit `-1` init in `vp8_create_common` because `vpx_calloc`'s zero would otherwise be a valid index. |
| `Vp8dComp.dec_fb_ref: [*mut Yv12BufferConfig; 4]` | `dec_fb_ref_idx: [i32; 4]` | Indexed by `MvReferenceFrame` (INTRA/LAST/GOLDEN/ALTREF). Reads go `pc.yv12_fb[dec_fb_ref_idx[r] as usize]`. |
| `FrameBuffers.pbi: [*mut Vp8dComp<'a>; 32]` | `pbi: Option<Box<Vp8dComp<'a>>>` | Single owned slot. Allocated via `Box::<T>::new_zeroed().assume_init()` (same byte pattern as the old `vpx_memalign + write_bytes(0)`; preserves the same UB-by-letter-of-spec for the non-nullable `SubpixFn` fields, which are overwritten in `init_frame`). Drop of the Box replaces the manual `vpx_free`. Kernel call sites use the `pbi_ptr()` helper to recover a raw pointer where still required. |
| `Vp8Common.show_frame_mi: *mut ModeInfo` | (deleted) | Was written per frame but never read in the minimal build (C source uses it for postproc / MFQE, both disabled). Three sites removed. |
| `Macroblockd.left_context: *mut EntropyContextPlanes` | (deleted) | The field always aliased `&mut common.left_context`; carried no state of its own. Read sites in `detokenize.rs` now reach the context via the existing `dx: *mut Vp8dComp` parameter (`vp8_reset_mb_tokens_context` gained a matching `dx` arg for symmetry with `vp8_decode_mb_tokens`). The per-row zeroing site in `decodeframe.rs` writes directly through `&mut (*pc).left_context`. **MT note**: in libvpx's `--enable-multithread` build, each worker thread rebinds `xd->left_context` to a stack-local `mb_row_left_context` (see `threading.c:598`). That C pattern is a workaround for C's lack of ownership types. In a Rust MT port each row-worker would own its own `Macroblockd` (the borrow checker forbids sharing `&mut Macroblockd` across threads), so the natural fix is to make `left_context` an inline `EntropyContextPlanes` field on `Macroblockd` rather than restoring the pointer. Deleting the pointer here did not foreclose that path. |
| `Vp8Common.above_context: *mut EntropyContextPlanes` | `Option<Box<[EntropyContextPlanes]>>` | `vpx_calloc(mb_cols * sizeof, 1)` → `vec![Default::default(); mb_cols].into_boxed_slice()`. `Option<Box<>>` zero-niche means the zero-initialised `Vp8dComp` shell produces `None` for the field automatically; alloccommon assigns `Some(box)` when frame dimensions are known. The explicit `vpx_free` call is replaced by `(*oci).above_context = None` (Box's Drop runs). OOM behaviour shifts from `is_null()` failure-path to Rust's allocator-abort. Bench delta vs. baseline: no measurable change (p > 0.05). |
| `Macroblockd.above_context: *mut EntropyContextPlanes` (cursor) | (deleted) | The cursor was a state-free walker — its value was always `pc.above_context.as_mut_ptr().add(mb_col)`. The dead `mb_idx` counter variable in the row-decode loop (only fed to an unused `_mb_idx` parameter) was removed alongside. `decode_macroblock` now receives `mb_col` directly; `vp8_reset_mb_tokens_context` and `vp8_decode_mb_tokens` each gained a `mb_col: i32` arg and index into `(*dx).common.above_context.as_deref_mut().unwrap()[mb_col as usize]` at use sites. Bench delta vs. baseline: no measurable change (p ≈ 0.06, trending slightly faster — likely the elided per-MB pointer advance). |
| `Macroblockd.current_bc: *mut c_void` | (deleted) | The field was a type-erased pointer to the per-row bool reader (one of `Vp8dComp.mbc[0..N-1]`). It is now computed as a local `bc: *mut Vp8Reader<'static>` at row-start in the outer decode loop and threaded as an explicit argument to `decode_macroblock` (gains `bc: *mut Vp8Reader<'static>`) and `vp8_decode_mb_tokens` (same). Three read sites (`decodeframe.rs` post-decode error check; `detokenize.rs` first line of the per-MB token driver) consume the param. The frame-init write at `decodeframe.rs:1313` was redundant with the per-row computation and was deleted. `Macroblockd` now carries **zero lifetime-bearing raw pointers**, removing the original motivation for the `*mut c_void` erasure. Bench delta vs. baseline: no measurable change (p > 0.9 on both vectors). |
| `Vp8Common.mip: *mut ModeInfo` | `Option<Box<[ModeInfo]>>` | `vpx_calloc((mb_cols+1)*(mb_rows+1), sizeof)` → `Box::<[ModeInfo]>::new_zeroed_slice(count).assume_init()`. Byte-identical layout (every `ModeInfo` field has a valid zero bit pattern — `MbPredictionMode::DcPred=0`, `MvReferenceFrame::Intra=0`, `BModeInfo`'s `Intra` variant has discriminant 0 and `BPredictionMode::DcPred=0`). The Box's Drop replaces the manual `vpx_free` + null-pointer reset. Bench delta vs. baseline: 480p flat (p=0.61), 720p +0.5% at p=0.03 (marginal — at the noise floor for this bench; the helper fires ≤2 times per frame). |
| `Vp8Common.mi: *mut ModeInfo` | (deleted) | The field was just `mip.offset(stride + 1)` — a convenience pointer at the first visible MB slot. Replaced by a method `Vp8Common::mi_base_ptr(&mut self) -> *mut ModeInfo` that computes the same value on demand. Three reader sites (`decodeframe.rs:1053` cursor init, `decodemv.rs:828` mode-decoding traversal start, `onyxd_if.rs:474` references-buffer scan) updated to call the helper. The negative-offset neighbour reads from `mode_info_context` still land in valid memory because the Box owns `(mb_cols+1)*(mb_rows+1)` entries. |
| `Macroblockd.mode_info_context: *mut ModeInfo` | (deleted) | The "boss-level" remaining pointer. Replaced by a safe accessor API on `Vp8Common`: `mi(row, col)` / `mi_mut(row, col)` (single cell), `mi_left` / `mi_above` / `mi_above_left` (typed neighbour reads using the slab's top-row + left-column padding so the negative offsets land in valid memory), and `mi_row(row)` / `mi_row_mut(row)` (`mb_cols`-long slice hoisted once per row for hot loops). 57 access sites converted across 5 files. Seven kernel functions gained `mi: &ModeInfo` / `&mut ModeInfo` parameters (`vp8_reset_mb_tokens_context`, `vp8_decode_mb_tokens`, `vp8_mb_init_dequantizer`, `vp8_build_intra_predictors_mby_s`, `_mbuv_s`, `vp8_build_inter_predictors_mb` + its 3 internal sub-callees, `decode_macroblock`). The loop-filter row functions (`vp8_loop_filter_row_normal` / `_simple`) lost their `mode_info_context: *mut ModeInfo` parameter and now read each MB via `cm.mi_row(mb_row)` inside the body. **Bench delta vs. baseline: +1.62% (480p), +1.73% (720p)** — the honest structural cost of the conversion: `Option::expect` (one branch per row) + slice bounds-checked indexing (one compare+branch per MB) where the C source used a single cursor `add(1)` per MB. All MI-grid access is now safe Rust — zero `unsafe { get_unchecked(_mut) }` shortcuts. |
| `Macroblockd.recon_above: [*mut u8; 3]`, `recon_left: [*mut u8; 3]`, `recon_left_stride: [i32; 2]` | (deleted, all three together) | These fields were a cached/walking copy of pointer offsets already encoded by `xd.dst.{y,u,v}_buffer` and the YV12 strides: `recon_above[i] ≡ dst.{y,u,v}_buffer.offset(-stride)` and `recon_left[i] ≡ dst.{y,u,v}_buffer.offset(-1)`. Per-row setup (~12 lines) and per-MB advance (6 lines of `.add(16)`/`.add(8)`) were deleted; intra-predictor call sites in `decode_macroblock` now compute the offsets inline at the point of use; `setup_intra_recon_left` at row-init computes the left-column pointers from `dst_buffer + recon_yoffset` locally. **Bench delta: this refactor *recovered* most of the MI-grid regression** — 720p went from +1.73% to no measurable change (p=0.49), 480p from +1.77% to +0.94% (p=0.04, marginal). The win comes from eliminating the unconditional per-MB advance that fired for inter MBs too, even though only intra MBs read the values. |
| `Blockd.qcoeff: *mut i16`, `Blockd.dqcoeff: *mut i16`, `Blockd.predictor: *mut u8`, `Blockd.dequant: *mut i16`, `Blockd.eob: *mut i8` | (all five deleted) | All five `Blockd` pointer fields were aliases of either `Macroblockd.{qcoeff,dqcoeff,eobs}` at fixed offset `block_idx * 16` or the encoder-only `Macroblockd.predictor[384]` scratch buffer. **`predictor`, `dequant`, and `eob` are encoder-only state** in the C source (used by `vp8/encoder/{loongarch/vp8_quantize_lsx, rdopt}.c`) — set by `vp8_setup_block_dptrs` for ABI parity but never read in a decoder-only build. Verified by greps: VP9 doesn't include `vp8/common/blockd.h`, so these fields are truly VP8-encoder-specific. **Phase A** deleted the three encoder-only fields plus the `Macroblockd.predictor: [u8; 384]` backing buffer (~1.2 KB saved per `Vp8dComp`) and the dead `vp8_build_inter_predictors_b` function. **Phase B** deleted `qcoeff` and `dqcoeff` (live but redundant) — ~6 call sites in `decodeframe.rs` now compute `xd.qcoeff.as_mut_ptr().add(i * 16)` directly; `vp8_dequantize_b_c` gained an explicit `(qcoeff, dqcoeff, DQC)` signature instead of `(d: *mut Blockd, DQC)`. With all five pointer fields gone, `vp8_setup_block_dptrs` is empty and was deleted along with its caller line in `create_decompressor_inner`. Final `Blockd` shape: `{ offset: i32, bmi: BModeInfo }` — ~12 bytes, zero raw pointers, from ~48 bytes previously. **Bench delta: marginal +0.7-0.8% regression** (B_PRED's 16-iteration sub-block loop now does `.add(i * 16)` per iteration instead of reading a cached field; cumulative effect on intra-heavy content). |

**Every kernel struct is now free of raw-pointer fields under this document's tracking.** `Vp8Common` (every slab — `yv12_fb`, `above_context`, `mip` — is owned through safe Rust types); `Macroblockd` (only the embedded `pre`/`dst: Yv12BufferConfig` plane pointers remain, which are pure pixel-data access — explicitly excluded); `Blockd` (down to `{ offset: i32, bmi: BModeInfo }`). The remaining raw-pointer surface in the codebase consists of pixel-data access (excluded by scope) and function-arg shape (§1 below).

All were verified bit-exact against the libvpx C reference via the 62-vector conformance suite.

### Companion `unsafe` relaxations enabled by these refactors

| Function | Before | After |
|---|---|---|
| `vp8dx_get_quantizer` | `pub unsafe fn(_: *const Vp8dComp)` | `pub fn(_: &Vp8dComp)` |
| `vp8_remove_decoder_instances` | `pub unsafe fn(_: *mut FrameBuffers)` | `pub fn(_: &mut FrameBuffers)` — `Option::take` consumes the Box so double-call is harmless; the `remove_decompressor` helper was inlined |
| `vp8_create_decoder_instances` | `pub unsafe fn(_: *mut FrameBuffers, _: *mut Vp8dConfig)` | `pub fn(_: &mut FrameBuffers, _: &Vp8dConfig)` with one internal `unsafe { create_decompressor(...) }` block |
| `vp8_setup_version` | `pub unsafe fn(_: *mut Vp8Common)` | `pub fn(_: &mut Vp8Common)` — body became fully safe (pure field assignments) |
| `vp8_create_common` | `pub unsafe fn(_: *mut Vp8Common)` | `pub fn(_: &mut Vp8Common)` — body fully safe; `ptr::write_bytes(...)` zeroing replaced with `.fill(0)` on the typed array |
| `vp8_remove_common` | `pub unsafe fn(_: *mut Vp8Common)` | `pub fn(_: &mut Vp8Common)` — body fully safe (calls the now-safe `vp8_de_alloc_frame_buffers`) |
| `vp8_de_alloc_frame_buffers` | `pub unsafe fn(_: *mut Vp8Common)` | `pub fn(_: &mut Vp8Common)` — body has one internal `unsafe` block around `vp8_yv12_de_alloc_frame_buffer` + `vpx_free` |
| `vp8_alloc_frame_buffers` | `pub unsafe fn(_: *mut Vp8Common, ...)` | `pub fn(_: &mut Vp8Common, ...)` — two small internal `unsafe` blocks around `vp8_yv12_alloc_frame_buffer` calls (the `vpx_calloc` and `mip.offset(...)` blocks went away when the MI grid became a `Box<[ModeInfo]>`) |
| `vp8_mb_init_dequantizer` *(§3 split-borrow pilot)* | `pub unsafe fn(_: *mut Vp8dComp, _: *mut Macroblockd)` | `pub fn(_: &Vp8Common, _: &mut Macroblockd)` — body almost fully safe; one internal `unsafe { (*mb.mode_info_context).mbmi.segment_id }` line for the remaining MI-grid cursor deref. Caller pattern: `vp8_mb_init_dequantizer(&(*pbi).common, &mut *xd)`. Validates that per-function disjoint-borrow conversion works in this codebase. |

### Split-borrow refactor — Phase 1 (reconinter / reconintra / reconintra4x4)

Phase 1 of the §1 split-borrow refactor: convert leaf kernel functions taking `xd: *mut Macroblockd` to take `&[mut] Macroblockd` or drop the param entirely. 4 more `unsafe fn` markers removed; 7 function signatures relaxed to `&[mut] Macroblockd`.

| Function | Before | After |
|---|---|---|
| `clamp_mv_to_umv_border` / `clamp_uvmv_to_umv_border` | `unsafe fn(_: &mut Mv, _: *const Macroblockd)` | `fn(_: &mut Mv, l: i32, r: i32, t: i32, bot: i32)` — pass the 4 edge i32s directly; body fully safe (zero unsafe ops) |
| `vp8_build_intra_predictors_mby_s` / `_mbuv_s` | `pub unsafe fn(x: *mut Macroblockd, ...)` | `pub unsafe fn(x: &Macroblockd, ...)` — still `unsafe fn` due to raw pixel ptrs + static-mut dispatch tables, but the Macroblockd access is type-checked |
| `intra_prediction_down_copy` | `pub unsafe fn(xd: *mut Macroblockd, ...)` | `pub fn(xd: &Macroblockd, ...)` — body has one internal `unsafe { }` for pixel ptr offsets |
| `build_inter_predictors4b` / `2b` | `unsafe fn(x: *mut Macroblockd, d: *mut Blockd, ...)` | `unsafe fn(subpixel_predict_*: SubpixFn, d: &Blockd, ...)` — dropped `x` param entirely; takes `SubpixFn` directly |
| `build_inter_predictors_b` | `unsafe fn(d: *mut Blockd, ...)` | `unsafe fn(d: &Blockd, ...)` |
| `vp8_build_inter16x16_predictors_mb` | `pub unsafe fn(x: *mut Macroblockd, ...)` | `pub unsafe fn(x: &Macroblockd, ...)` |
| `build_4x4uvmvs` | `unsafe fn(x: *mut Macroblockd, mi: &ModeInfo)` | `fn(x: &mut Macroblockd, mi: &ModeInfo)` — fully safe `fn`; edges captured up-front to avoid borrow conflict with `bmi_mv_mut` |
| `build_inter4x4_predictors_mb` | `unsafe fn(x: *mut Macroblockd, mi: &ModeInfo)` | `unsafe fn(x: &mut Macroblockd, mi: &ModeInfo)` — Tier C orchestrator; full rewrite to snapshot `SubpixFn`/plane-base ptrs up front, then sub-call with `&x.block[i]` shared borrows |
| `vp8_build_inter_predictors_mb` | `pub unsafe fn(xd: *mut Macroblockd, mi: &ModeInfo)` | `pub unsafe fn(xd: &mut Macroblockd, mi: &ModeInfo)` |

**Bench delta vs. baseline**: 480p -0.42% (no change p=0.06), 720p **-1.01% improvement** (p<0.01). The cumulative trajectory (MI-grid + recon_* + Blockd + Phase 1) now lands **at or below baseline**. The Phase 1 gain came from eliminating `(*x).subpixel_predict*` function-pointer derefs through a raw pointer — passing the `SubpixFn` value directly let the compiler optimize better.

Caller pattern at `decode_macroblock` boundaries: `&*xd` / `&mut *xd` to bridge from the still-raw `xd: *mut Macroblockd` to the new safe signatures. The bridge is zero-cost and uses the established pattern from earlier refactors.

### Split-borrow refactor — Phase 2a + 2b (decodemv.rs leaf helpers)

5 more leaf functions in `decodemv.rs` converted to safe signatures. Phase 2b finally exercises the `Vp8Common::mi_above`/`mi_left` accessors that were created during the MI-grid removal but unused in the kernel hot path until now.

| Function | Before | After |
|---|---|---|
| `mv_bias` | `unsafe fn(_: i32, _: MvReferenceFrame, mvp: *mut Mv, ref_frame_sign_bias: *const i32)` | `fn(_: i32, _: MvReferenceFrame, mvp: &mut Mv, ref_frame_sign_bias: &[i32; MAX_REF_FRAMES])` — body fully safe |
| `vp8_clamp_mv2` | `unsafe fn(mv: *mut Mv, xd: *const Macroblockd)` | `fn(mv: &mut Mv, l: i32, r: i32, t: i32, bot: i32)` — body fully safe; 3 call sites unrolled `xd` into the 4 edges |
| `vp8_check_mv_bounds` | `unsafe fn(mv: *const Mv, ...)` | `fn(mv: &Mv, ...)` — body fully safe |
| `above_block_mode` | `unsafe fn(cur_mb: *const ModeInfo, b, mi_stride)` | `fn(pc: &Vp8Common, mb_row, mb_col, mi: &ModeInfo, b)` — uses `pc.mi_above(mb_row, mb_col)` for the neighbor read; body fully safe |
| `left_block_mode` | `unsafe fn(cur_mb: *const ModeInfo, b)` | `fn(pc: &Vp8Common, mb_row, mb_col, mi: &ModeInfo, b)` — uses `pc.mi_left(mb_row, mb_col)`; body fully safe |

The neighbor-accessor migration cascaded `(mb_row, mb_col)` through `read_kf_modes` and `decode_mb_mode_mvs` so the values reach `above_block_mode`/`left_block_mode`. `vp8_decode_mode_mvs`'s previously-unused `_mb_row`/`_mb_col` loop variables are now live.

**Bench delta vs. baseline**: 480p **−1.20% improvement** (p<0.01), 720p **−1.12% improvement** (p<0.01). Both benches are now ~1% faster than baseline. The neighbor-accessor migration (`pc.mi_above` / `mi_left`) compiles to faster code than the C-style negative-offset cursor it replaced — LLVM can prove `idx < slab.len()` from the `mb_row < mb_rows` / `mb_col < mb_cols` invariants, eliding the bounds check entirely. The earlier MI-grid regression has been fully recovered AND beaten.

### Split-borrow refactor — Phase 2c (vp8_decode_mode_mvs + read_mb_features)

| Function | Change |
|---|---|
| `vp8_decode_mode_mvs` | The raw `mi: *mut ModeInfo` cursor that walked the MI grid is gone. Each iteration of the inner loop now obtains `mi` fresh from `pc.mi_mut(mb_row, mb_col)`. The `mi.add(1)` per-MB advance and `mi.add(1)` row-end skip-padding are deleted. Function itself stays `unsafe fn` (still derefs `*mut Vp8dComp`). |
| `Vp8Common::mi_base_ptr` | **Deleted** — no remaining callers after the cursor elimination. |
| `read_mb_features` | `unsafe fn(r: *mut Vp8Reader, mi: *mut MbModeInfo, x: *mut Macroblockd)` → `fn(r: *mut Vp8Reader, mi: &mut MbModeInfo, x: &Macroblockd)`. Body has one localized `unsafe { ... }` block around the three `vp8_read` calls (bool-reader API still raw). |

**No raw MI cursors remain anywhere in the codebase.** Every neighbour read goes through `pc.mi_above` / `mi_left` / `mi_above_left`; every current-MB access through `pc.mi_mut`. The Box-owned `Option<Box<[ModeInfo]>>` slab is reached exclusively via safe accessors with bounds-checked indexing that LLVM elides at the call sites' loop bounds.

**Bench delta vs. baseline**: 480p **−1.35% improvement** (p<0.01), 720p **−1.62% improvement** (p<0.01). The cumulative trajectory now sits comfortably below baseline.

The kernel callers that still hold raw `*mut Vp8Common` cross into these safe APIs via `&mut *pc` at the call site — keeping the per-frame decoder loop raw-pointer-shaped while everything from `Vp8Common`-level alloc/teardown upward is type-checked.

### Split-borrow refactor — Phase 3 (outer drivers)

| Function | Change |
|---|---|
| `vp8_mb_init_dequantizer` | `(pc: &Vp8Common, ...)` → `(y1_dequant, y2_dequant, uv_dequant, base_qindex, mb, mi)`. Decomposed to take the specific `Vp8Common` fields it touches instead of the aggregate. |
| `vp8_reset_mb_tokens_context` | `unsafe fn(dx: *mut Vp8dComp, mi, mb_col)` → `fn(above_slot: &mut EntropyContextPlanes, left_context: &mut EntropyContextPlanes, mi: &ModeInfo)`. Above/left slots are resolved at the caller. |
| `vp8_decode_mb_tokens` | `unsafe fn(dx, xd, mi, mb_col, bc)` → `fn(above_slot, left_context, fc, mb, mi, bc: *mut BoolDecoder)`. Aggregate-pointer params replaced with field-disjoint references; only the bool-reader stays raw. |
| `vp8dx_bool_error` | `unsafe fn(br: *mut BoolDecoder)` → `fn(br: &BoolDecoder)`. Pure shared-state predicate. |
| `decode_macroblock` | `unsafe fn(pbi, xd, mi, mb_col, bc)` → `fn(fc, above_slot, left_context, y1_dequant, y2_dequant, uv_dequant, base_qindex, xd: &mut Macroblockd, mi: &mut ModeInfo, bc: &mut Vp8Reader<'static>)`. 10 args, all field-disjoint borrows. Body uses small internal `unsafe` blocks for pixel-pointer arithmetic and inter-predictor dispatch. |
| `decode_mb_rows` | `unsafe fn(pbi: *mut)` → `fn(pbi: &mut Vp8dComp<'static>)`. The row driver materializes ref-frame plane snapshots up front, then walks MBs through safe field accesses on `pbi`. Pixel arithmetic and loop-filter row callees stay in small internal `unsafe` blocks. |
| `vp8_decode_frame` | `unsafe fn(*mut Vp8dComp)` → `fn(pbi: &mut Vp8dComp<'static>)`. Body uses `pbi.foo` field access throughout, with localized `unsafe { }` blocks scoped to: (a) header-byte pointer arithmetic, (b) the bool-reader-driven segmentation/loop-filter/quantizer/refresh/coef-probs parsing clusters, (c) FFI-shaped sub-calls (`init_frame`, `vp8dx_start_decode`, `setup_token_decoder`, `vp8cx_init_de_quantizer`, `vp8_decode_mode_mvs`, `vpx_internal_error`). |
| `vp8dx_receive_compressed_data` | `unsafe fn` → `fn(pbi: &mut Vp8dComp<'static>)`. Body is fully safe field access — only `vp8_yv12_copy_frame` (pixel copy) is in a small `unsafe` block. **Public API is now callable from safe Rust.** |
| `check_fragments_for_errors` | `unsafe fn(*mut Vp8dComp)` → `fn(&mut Vp8dComp)`. Body uses safe field access; the one `vp8_yv12_copy_frame` call sits in a scoped unsafe block. |

**Key technique**: when sub-functions take individual `Vp8Common` fields (above_context slot, left_context, fc, dequant tables, base_qindex), Rust's field-disjoint borrow checker lets the caller hold *all of them simultaneously* alongside `&mut pbi.mb` and the current MI cell — even though they all originate from `pbi.common`. This is what unlocks safe `decode_macroblock`.

**Bench delta vs. baseline**: 480p **−1.87% improvement** (p<0.01), 720p **−1.87% improvement** (p<0.01). Phase 3 *speeds the decoder up* — likely from removing raw-pointer dereferences (which suppress aliasing-based optimizations) in the per-MB hot path and frame-header parsing.

### Pixel-kernel safety push (predictors + IDCT)

Pushed the `unsafe` *down into* the pixel kernels so callers don't wear it:

| Function | Change |
|---|---|
| `vp8_build_intra_predictors_mby_s` / `_mbuv_s` | `unsafe fn` → `fn`; internal `unsafe` for the left-column gather + dispatch-table call. |
| `vp8_intra4x4_predict` | `unsafe fn` → `fn`; internal `unsafe`. |
| `vp8_build_inter_predictors_mb` | `unsafe fn` → `fn`; internal `unsafe` for the still-unsafe sub-dispatchers. |
| `vp8_dequantize_b_c`, `vp8_dequant_idct_add_c`, `vp8_dequant_idct_add_y_block_c`, `vp8_dequant_idct_add_uv_block_c` | `unsafe fn` → `fn`; internal `unsafe`. |
| `vp8_dc_only_idct_add_c`, `vp8_short_idct4x4llm_c`, `vp8_short_inv_walsh4x4_c`, `vp8_short_inv_walsh4x4_1_c` | `unsafe extern "C" fn` → `extern "C" fn` (RTCD-dispatch-compatible, dropped `unsafe`); internal `unsafe`. |

`decode_macroblock` lost all five large `unsafe { }` blocks — the remaining scoped unsafe is just pixel-pointer offset arithmetic (`xd.dst.y_buffer.offset(...)`, `xd.qcoeff.as_mut_ptr().add(...)`). Kernel calls are now plain safe-fn calls. Conformance held; bench 480p −2.0%, 720p −1.7%.

### Phase 4 — Bool-reader API

| Function | Change |
|---|---|
| `vp8dx_decode_bool`, `vp8_decode_value` | `unsafe fn(*mut BoolDecoder)` → `fn(&mut BoolDecoder)`. **Fully safe** — no internal unsafe. |
| `vp8_read` / `vp8_read_bit` / `vp8_read_literal` | `unsafe fn(*mut)` → `fn(&mut)`. Fully safe. |
| `vp8_treed_read` | `fn(&mut BoolDecoder, *const TreeIndex, *const Prob)`; internal `unsafe` for the table walk. |
| `vp8dx_bool_decoder_fill` | signature `fn(&mut BoolDecoder)`; internal `unsafe` for the byte-walk + decrypt detour. |
| `vp8dx_start_decode` | signature `fn(&mut BoolDecoder, ...)`; internal `unsafe` for `from_raw_parts`. |
| `VP8GetBit`, `GetSigned` | `unsafe fn(*mut)` → `fn(&mut)`. Fully safe (GetSigned operates on `br.range/value/count` directly). |
| `GetCoeffs`, `vp8_decode_mb_tokens` | bool-reader arg `*mut` → `&mut BoolDecoder`; keep internal `unsafe` for coefficient/entropy-context pointer arithmetic. |

Cascade cleanups: the three big bool-reader `unsafe { }` blocks in `vp8_decode_frame` (segmentation / loop-filter-deltas / refresh+coef-probs) became plain safe blocks, each scoped to a fresh `let bc = &mut pbi.mbc[8];` — field-disjoint from the `pbi.mb`/`pbi.common` writes they interleave with. `decodemv.rs` leaf readers (`read_bmode`/`read_ymode`/`read_mvcomponent`/`read_mv`/`read_mvcontexts`/`decode_split_mv`/`read_mb_features`) now take `&mut Vp8Reader`; the `bc as *mut _` coercion in `decode_macroblock` is gone.

**Bench delta vs. baseline**: 480p **−1.7%**, 720p **−2.8%**. 125/125 conformance + idct/predict/decode-api suites pass.

What's left of the original §1 raw-pointer surface:
- **FFI-shaped sub-calls** (`init_frame`, `setup_token_decoder`, `vp8cx_init_de_quantizer`, `vp8_decode_mode_mvs`, `vp8_loop_filter_*`) — still take `*mut Vp8dComp` / `*mut Vp8Common`. Called inside scoped `unsafe { }`.
- **`vp8dx_start_decode` lifetime escape hatch** — its `'a` unifies `BoolDecoder<'a>` with the decrypt-callback lifetime; the two callers route through a raw `*mut Vp8dComp` to dodge a borrow-checker false positive (documented at each site).
- **`GetCoeffs` / `vp8_decode_mb_tokens` / `decodemv.rs` prob-pointer arithmetic** — `*const Prob` / `*mut EntropyContext` table walking stays in scoped `unsafe`; a follow-on pass could index these safely.
- **Loop filter row callees / pixel kernels** — intentionally raw-ptr-shaped (pixel-side, doc-excluded).

---

## 1. Function Arguments Across the Kernel

### `*mut Vp8dComp`, `*mut Macroblockd`, `*mut Vp8Common`, etc.
*   **Context:** Almost every internal kernel function takes these structures as raw mutable pointers (e.g., `vp8_decode_frame(pbi: *mut Vp8dComp)`).
*   **Conversion Complexity: VERY HIGH (in aggregate).** The C source heavily aliases these structures. It is extremely common for a function to be passed both `pbi` and `xd` (where `xd` is a pointer to `pbi.mb`). Rust's borrow checker strictly forbids aliasing mutable references (`&mut`). Converting these function signatures to use `&mut` would require a massive refactoring to "split borrows" — modifying functions to only accept the specific fields they need (e.g., passing `&mut pbi.mb` and `&mut pbi.common` separately) rather than passing the entire god-object context around.

This is the last remaining structural raw-pointer category. With every kernel-aliasing field gone from §0, the function-arg surface is what's left: most internal kernel functions still take `*mut Vp8dComp` / `*mut Macroblockd` even though the *fields* they reach through are now all type-safe.

### §1 — empirical findings after the MI-grid conversion

The original VERY HIGH rating was for the all-at-once cross-cutting conversion. A pilot (`vp8_mb_init_dequantizer`) and then the full MI-grid removal converted ~9 kernel functions to take typed `mi: &ModeInfo` / `&mut ModeInfo` parameters in addition to (or instead of) their raw `*mut Macroblockd` ones. Observations:

- **Per-function conversion is LOW-MEDIUM** — mechanical, scope-bounded. The pattern is: identify what the function reads from `xd.mode_info_context`, replace with a `mi` parameter, update call sites to pass `pc.mi_mut(mb_row, mb_col)` or a per-row slice's element.
- **Mixing safe-signature and raw-pointer functions works.** The reborrow pattern `(&(*pbi).common, &mut *xd, mi)` at the boundary lets converted functions live alongside un-converted ones with zero ABI compatibility issues.
- **Borrow-checker conflicts are real but manageable.** Where the body needed both a current-MB borrow and a neighbour read, the read-then-write idiom (read into local, then mutate) resolved them. No site required a `split_at_mut` or interior-mutability cell.
- **There is a measurable perf cost — and partly recoverable.** Cumulative trajectory from baseline: MI-grid conversion added +1.6-1.7%; `recon_*` removal recovered most of it (720p to flat, 480p to +0.94%); `Blockd` cleanup added a small +0.7-0.8% (the `.add(i * 16)` arithmetic in B_PRED's 16-iteration sub-block loop). Final cumulative delta: **~+0.7-0.85% at both resolutions**, statistically significant but small. The honest framing: the kernel's whole structural raw-pointer surface was retired for under 1% steady-state cost.
- **Functions can still be `unsafe fn` internally** while taking typed reference parameters. The kernel functions that take `mi: &ModeInfo` typically also still take `xd: *mut Macroblockd` because the surrounding state is raw-pointer-shaped. A future pass could push the `xd` parameter to `&mut Macroblockd` in the same way.

Making functions like `decode_macroblock` *fully* safe (no `unsafe {}` blocks inside, no raw pointer parameters) remains gated on the embedded `pre`/`dst` YV12 plane pointers — pixel-side state that this document deliberately excludes from its tracking.
