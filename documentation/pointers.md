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

All were verified bit-exact against the libvpx C reference via the 62-vector conformance suite.

### Companion `unsafe` relaxations enabled by these refactors

| Function | Before | After |
|---|---|---|
| `vp8dx_get_quantizer` | `pub unsafe fn(_: *const Vp8dComp)` | `pub fn(_: &Vp8dComp)` |
| `vp8_remove_decoder_instances` | `pub unsafe fn(_: *mut FrameBuffers)` | `pub fn(_: &mut FrameBuffers)` — `Option::take` consumes the Box so double-call is harmless; the `remove_decompressor` helper was inlined |
| `vp8_create_decoder_instances` | `pub unsafe fn(_: *mut FrameBuffers, _: *mut Vp8dConfig)` | `pub fn(_: &mut FrameBuffers, _: &Vp8dConfig)` with one internal `unsafe { create_decompressor(...) }` block |

---

## 1. Inside `Macroblockd` (Per-MB working state)

### `mode_info_context: *mut ModeInfo`
*   **Context:** Points to the current macroblock's `ModeInfo` entry within the frame-wide `mi` grid (which is owned by `Vp8Common`). It is used extensively for neighbor lookups via pointer arithmetic (e.g., `mode_info_context[-1]` for the left neighbor, `mode_info_context[-stride]` for the above neighbor).
*   **Conversion Complexity: HIGH.** Rust references (`&mut ModeInfo`) cannot be negatively indexed. To remove this pointer, the codebase would need to pass a slice of the entire grid (`&mut [ModeInfo]`) along with the current `(row, col)` index to every function that inspects neighbors. This would require rewriting almost every motion-vector and intra-prediction mode decoding function, significantly altering their signatures and adding bounds checks.

### `above_context: *mut EntropyContextPlanes`
*   **Context:** Points to the current MB column's slot in the `above_context` array (owned by `Vp8Common`). It advances linearly as the decoder processes a row of macroblocks.
*   **Conversion Complexity: MEDIUM-HIGH.** Similar to `mode_info_context`, it is an interior pointer into an array owned elsewhere. Converting it would mean explicitly passing the relevant slice or index per macroblock instead of keeping a crawling pointer inside the state struct.

### `current_bc: *mut c_void`
*   **Context:** A type-erased pointer to the currently-active `BoolDecoder` (`Vp8Reader`) for the current macroblock row. Reset per row at `decodeframe.rs:629` (round-robin across token partitions) and cast back to `*mut Vp8Reader<'static>` at every read site (`detokenize.rs:233`, `decodeframe.rs:182, 707`). It is type-erased because `Vp8Reader` has a lifetime parameter (`Vp8Reader<'a>`), and the port sought to avoid adding lifetimes to the `Macroblockd` aggregate.
*   **Conversion Complexity: HIGH.** Converting this to a safe `Option<&mut BoolDecoder<'a>>` requires adding the `'a` lifetime to `Macroblockd` (`Vp8dComp<'a>` already carries one — the real touch point is `Macroblockd`, which is embedded by value inside `Vp8dComp`). Additionally, the bool decoders live in `Vp8dComp.mbc[]` while `Macroblockd` is also reached through `&mut Vp8dComp`; Rust's borrow checker forbids holding a `&mut BoolDecoder` inside `Macroblockd` while the array still belongs to the same owning aggregate. Resolution would require either an interior-mutability cell, a split-borrow refactor, or routing `current_bc` as an explicit argument rather than as state.

### `recon_above: [*mut u8; 3]` / `recon_left: [*mut u8; 3]`
*   **Context:** Per-plane (Y/U/V) above-row and left-column edge pointers, set up once per MB to alias into the destination `Yv12BufferConfig`'s planes. The intra predictors read neighbor samples through these pointers when generating the predictor for the current block. They straddle the "pixel access" exclusion of this document — they resolve to pixels but are structural anchors (one per plane per MB) rather than buffers walked sample-by-sample.
*   **Conversion Complexity: MEDIUM.** Each entry could become a `(plane: u8, offset: isize)` pair indexing into the embedded `dst: Yv12BufferConfig`. The touch is broad (every intra predictor call site dereferences these) but mechanical, and the lifetime is bounded by the embedding `Macroblockd` so no lifetime-parameter cascade is needed.

---

## 2. Inside `Vp8Common` (Per-sequence/frame state)

### `mip: *mut ModeInfo`, `mi: *mut ModeInfo`
*   **Context:** `mip` points to the base heap allocation of the `ModeInfo` grid (which includes padding for top/left borders). `mi` is an offset view into `mip` pointing to the first visible macroblock (allowing negative indexing to hit the padding).
*   **Conversion Complexity: HIGH.** The grid is allocated via `vpx_calloc` and aliased heavily. Replacing this requires changing the raw allocation to a `Vec<ModeInfo>` or `Box<[ModeInfo]>`. Because Rust slices do not support negative indexing, the concept of `mi` as an offset pointer would need to be replaced by a custom 2D grid abstraction that safely encapsulates the border padding and provides safe `get(row, col)` methods.

### `above_context: *mut EntropyContextPlanes`
*   **Context:** Points to the heap allocation for the above-row entropy contexts (allocated via `vpx_calloc`).
*   **Conversion Complexity: MEDIUM-HIGH.** Could be converted to a `Box<[EntropyContextPlanes]>` or `Vec`. However, all accesses would need to be updated to use safe slice indexing (`above_context[col]`), and the pointer arithmetic currently used to walk the context would need to be rewritten into iterator or index-based loops.

---

## 3. Function Arguments Across the Kernel

### `*mut Vp8dComp`, `*mut Macroblockd`, `*mut Vp8Common`, etc.
*   **Context:** Almost every internal kernel function takes these structures as raw mutable pointers (e.g., `vp8_decode_frame(pbi: *mut Vp8dComp)`).
*   **Conversion Complexity: VERY HIGH (in aggregate).** The C source heavily aliases these structures. It is extremely common for a function to be passed both `pbi` and `xd` (where `xd` is a pointer to `pbi.mb`). Rust's borrow checker strictly forbids aliasing mutable references (`&mut`). Converting these function signatures to use `&mut` would require a massive refactoring to "split borrows" — modifying functions to only accept the specific fields they need (e.g., passing `&mut pbi.mb` and `&mut pbi.common` separately) rather than passing the entire god-object context around.

This refactor is the cross-cutting prerequisite for most of the §1 and §2 conversions: once functions stop taking `*mut Vp8dComp` and instead take disjoint `&mut` views, several of the "MEDIUM-HIGH" entries above drop to LOW because their access pattern can be expressed as a safe `&mut` against state that's no longer aliased.
