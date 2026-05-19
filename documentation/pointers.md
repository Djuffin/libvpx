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
| `vp8_alloc_frame_buffers` | `pub unsafe fn(_: *mut Vp8Common, ...)` | `pub fn(_: &mut Vp8Common, ...)` — three small internal `unsafe` blocks around `vp8_yv12_alloc_frame_buffer`, `vpx_calloc`, and `mip.offset(...)` |

The kernel callers that still hold raw `*mut Vp8Common` cross into these safe APIs via `&mut *pc` at the call site — keeping the per-frame decoder loop raw-pointer-shaped while everything from `Vp8Common`-level alloc/teardown upward is type-checked.

---

## 1. Inside `Macroblockd` (Per-MB working state)

### `mode_info_context: *mut ModeInfo`
*   **Context:** Points to the current macroblock's `ModeInfo` entry within the frame-wide `mi` grid (which is owned by `Vp8Common`). It is used extensively for neighbor lookups via pointer arithmetic (e.g., `mode_info_context[-1]` for the left neighbor, `mode_info_context[-stride]` for the above neighbor).
*   **Conversion Complexity: HIGH.** Rust references (`&mut ModeInfo`) cannot be negatively indexed. To remove this pointer, the codebase would need to pass a slice of the entire grid (`&mut [ModeInfo]`) along with the current `(row, col)` index to every function that inspects neighbors. This would require rewriting almost every motion-vector and intra-prediction mode decoding function, significantly altering their signatures and adding bounds checks.

### `recon_above: [*mut u8; 3]` / `recon_left: [*mut u8; 3]`
*   **Context:** Per-plane (Y/U/V) above-row and left-column edge pointers, set up once per MB to alias into the destination `Yv12BufferConfig`'s planes. The intra predictors read neighbor samples through these pointers when generating the predictor for the current block. They straddle the "pixel access" exclusion of this document — they resolve to pixels but are structural anchors (one per plane per MB) rather than buffers walked sample-by-sample.
*   **Conversion Complexity: MEDIUM.** Each entry could become a `(plane: u8, offset: isize)` pair indexing into the embedded `dst: Yv12BufferConfig`. The touch is broad (every intra predictor call site dereferences these) but mechanical, and the lifetime is bounded by the embedding `Macroblockd` so no lifetime-parameter cascade is needed.

---

## 2. Inside `Vp8Common` (Per-sequence/frame state)

### `mip: *mut ModeInfo`, `mi: *mut ModeInfo`
*   **Context:** `mip` points to the base heap allocation of the `ModeInfo` grid (which includes padding for top/left borders). `mi` is an offset view into `mip` pointing to the first visible macroblock (allowing negative indexing to hit the padding).
*   **Conversion Complexity: HIGH.** The grid is allocated via `vpx_calloc` and aliased heavily. Replacing this requires changing the raw allocation to a `Vec<ModeInfo>` or `Box<[ModeInfo]>`. Because Rust slices do not support negative indexing, the concept of `mi` as an offset pointer would need to be replaced by a custom 2D grid abstraction that safely encapsulates the border padding and provides safe `get(row, col)` methods.

---

## 3. Function Arguments Across the Kernel

### `*mut Vp8dComp`, `*mut Macroblockd`, `*mut Vp8Common`, etc.
*   **Context:** Almost every internal kernel function takes these structures as raw mutable pointers (e.g., `vp8_decode_frame(pbi: *mut Vp8dComp)`).
*   **Conversion Complexity: VERY HIGH (in aggregate).** The C source heavily aliases these structures. It is extremely common for a function to be passed both `pbi` and `xd` (where `xd` is a pointer to `pbi.mb`). Rust's borrow checker strictly forbids aliasing mutable references (`&mut`). Converting these function signatures to use `&mut` would require a massive refactoring to "split borrows" — modifying functions to only accept the specific fields they need (e.g., passing `&mut pbi.mb` and `&mut pbi.common` separately) rather than passing the entire god-object context around.

This refactor is the cross-cutting prerequisite for most of the §1 and §2 conversions: once functions stop taking `*mut Vp8dComp` and instead take disjoint `&mut` views, several of the "MEDIUM-HIGH" entries above drop to LOW because their access pattern can be expressed as a safe `&mut` against state that's no longer aliased.
