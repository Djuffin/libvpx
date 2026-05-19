# VP8 Decoder: Raw Pointers to Higher-Level Structures

This document provides context and a conversion-complexity guesstimate for the various raw pointers (`*mut T` / `*const T`) used in the VP8 decoder kernel to alias or share higher-level data structures (e.g., macroblocks, frame buffers, and contexts).

Pointers used exclusively for pixel, coefficient, or bitstream access (such as `y_buffer: *mut u8`, `qcoeff: *mut i16`, `eob: *mut i8`, or `buffer: *const u8`) are excluded from this analysis.

---

## 1. Inside `Macroblockd` (Per-MB working state)

### `mode_info_context: *mut ModeInfo`
*   **Context:** Points to the current macroblock's `ModeInfo` entry within the frame-wide `mi` grid (which is owned by `Vp8Common`). It is used extensively for neighbor lookups via pointer arithmetic (e.g., `mode_info_context[-1]` for the left neighbor, `mode_info_context[-stride]` for the above neighbor).
*   **Conversion Complexity: HIGH.** Rust references (`&mut ModeInfo`) cannot be negatively indexed. To remove this pointer, the codebase would need to pass a slice of the entire grid (`&mut [ModeInfo]`) along with the current `(row, col)` index to every function that inspects neighbors. This would require rewriting almost every motion-vector and intra-prediction mode decoding function, significantly altering their signatures and adding bounds checks.

### `above_context: *mut EntropyContextPlanes`
*   **Context:** Points to the current MB column's slot in the `above_context` array (owned by `Vp8Common`). It advances linearly as the decoder processes a row of macroblocks.
*   **Conversion Complexity: MEDIUM-HIGH.** Similar to `mode_info_context`, it is an interior pointer into an array owned elsewhere. Converting it would mean explicitly passing the relevant slice or index per macroblock instead of keeping a crawling pointer inside the state struct.

### `left_context: *mut EntropyContextPlanes`
*   **Context:** Points to the left-column entropy context (which usually points directly to `common.left_context`).
*   **Conversion Complexity: MEDIUM.** It generally points to a single struct rather than crawling an array. However, replacing it with a `&mut EntropyContextPlanes` would require tying `Macroblockd`'s lifetime to `Vp8Common` via a lifetime parameter (`Macroblockd<'a>`), which would ripple up to `Vp8dComp`.

### `current_bc: *mut c_void`
*   **Context:** A type-erased pointer to the currently-active `BoolDecoder` (`Vp8Reader`) for the current macroblock row. It is type-erased because `Vp8Reader` has a lifetime parameter (`Vp8Reader<'a>`), and the port sought to avoid adding lifetimes to the dense `Macroblockd` aggregate.
*   **Conversion Complexity: HIGH.** Converting this to a safe `Option<&mut BoolDecoder<'a>>` requires adding the `'a` lifetime to `Macroblockd`, which cascades to `Vp8dComp`. Additionally, mutable borrow rules would make it very difficult to store a `&mut` to the bool decoder inside `Macroblockd` while simultaneously holding the array of bool decoders in `Vp8dComp`.

---

## 2. Inside `Vp8Common` (Per-sequence/frame state)

### `frame_to_show: *mut Yv12BufferConfig`
*   **Context:** Points to one of the slots in the `yv12_fb` array that represents the frame ready to be displayed.
*   **Conversion Complexity: LOW.** This is a classic self-referential pointer. It can be easily converted to a `usize` index or an enum representing the slot index in the `yv12_fb` array. Accesses would just become `common.yv12_fb[common.frame_to_show_idx]`.

### `mip: *mut ModeInfo`, `mi: *mut ModeInfo`, `show_frame_mi: *mut ModeInfo`
*   **Context:** `mip` points to the base heap allocation of the `ModeInfo` grid (which includes padding for top/left borders). `mi` is an offset view into `mip` pointing to the first visible macroblock (allowing negative indexing to hit the padding). `show_frame_mi` points to the grid for the frame currently being shown.
*   **Conversion Complexity: HIGH.** The grid is allocated via `vpx_calloc` and aliased heavily. Replacing this requires changing the raw allocation to a `Vec<ModeInfo>` or `Box<[ModeInfo]>`. Because Rust slices do not support negative indexing, the concept of `mi` as an offset pointer would need to be replaced by a custom 2D grid abstraction that safely encapsulates the border padding and provides safe `get(row, col)` methods.

### `above_context: *mut EntropyContextPlanes`
*   **Context:** Points to the heap allocation for the above-row entropy contexts (allocated via `vpx_calloc`).
*   **Conversion Complexity: MEDIUM-HIGH.** Could be converted to a `Box<[EntropyContextPlanes]>` or `Vec`. However, all accesses would need to be updated to use safe slice indexing (`above_context[col]`), and the pointer arithmetic currently used to walk the context would need to be rewritten into iterator or index-based loops.

---

## 3. Inside `Vp8dComp` (Top-level decoder instance)

### `dec_fb_ref: [*mut Yv12BufferConfig; NUM_YV12_BUFFERS]`
*   **Context:** An array of 4 pointers referencing the DPB slots (LAST, GOLDEN, ALTREF, NEW). Each entry points back into the `common.yv12_fb` array.
*   **Conversion Complexity: LOW.** Like `frame_to_show`, this is a self-referential array of pointers. It can be easily replaced with an array of indices `[usize; 4]` pointing to the corresponding slots in `common.yv12_fb`. Accesses would change from `(*pbi).dec_fb_ref[INTRA_FRAME]` to `(*pbi).common.yv12_fb[(*pbi).dec_fb_idx[INTRA_FRAME]]`.

---

## 4. Function Arguments Across the Kernel

### `*mut Vp8dComp`, `*mut Macroblockd`, `*mut Vp8Common`, etc.
*   **Context:** Almost every internal kernel function takes these structures as raw mutable pointers (e.g., `vp8_decode_frame(pbi: *mut Vp8dComp)`).
*   **Conversion Complexity: VERY HIGH (in aggregate).** The C source heavily aliases these structures. It is extremely common for a function to be passed both `pbi` and `xd` (where `xd` is a pointer to `pbi.mb`). Rust's borrow checker strictly forbids aliasing mutable references (`&mut`). Converting these function signatures to use `&mut` would require a massive refactoring to "split borrows" — modifying functions to only accept the specific fields they need (e.g., passing `&mut pbi.mb` and `&mut pbi.common` separately) rather than passing the entire god-object context around.
