# Plan: de-raw-pointer `Yv12BufferConfig`

Staged plan to give the YV12 frame-buffer slab RAII ownership and remove
whole-struct pointer-carrying copies of `Yv12BufferConfig`, **without
breaking the external "caller provides their own buffer" contract** and
without regressing the bit-exact decode path.

This is the §15 ("things that remain idiomatically un-Rust") kernel
frontier — larger and more perf-sensitive than the API-layer pointer
cleanups. It is broken into stages that the compiler can enforce in
order.

---

## Goal / non-goals

- **Goal:** the YV12 backing slab owns itself via RAII (no manual
  `vpx_memalign` / `vpx_free`, no `ptr::write_bytes` scrub, no
  double-free / leak expressible), and no `Yv12BufferConfig` is ever
  bytewise-copied with live plane pointers.
- **Non-goal (core):** converting per-pixel plane access from raw
  pointers to slices. That is the perf-sensitive part, isolated into the
  final optional, bench-gated stage (Stage 4).
- **Invariants after every stage:** `cargo build --all-targets` clean
  (0 warnings); full test suite green incl. all 62 bit-exact MD5 vectors
  **and** the SET/COPY reference round-trip; `cargo bench` within noise
  of the Stage-0 baseline.

---

## Current architecture (verified facts)

- DPB: `Vp8Common.yv12_fb[NUM_YV12_BUFFERS = 4]: [Yv12BufferConfig; 4]`,
  ref-counted by `fb_idx_ref_cnt[4]` + indices
  `new/lst/gld/alt_fb_idx`. `frame_to_show` is **already an index**
  (`frame_to_show_idx`) — precedent for the index model
  (`onyxd_if.rs:221/223/426/430`).
- The slab: `buffer_alloc: *mut u8` (+ `buffer_alloc_sz`) is one
  `vpx_memalign(32, frame_size)` block; `y/u/v/alpha_buffer` alias into
  it at `border*stride + border`. Freed manually in
  `vp8_yv12_de_alloc_frame_buffer` (guarded by `buffer_alloc_sz > 0`),
  struct scrubbed with `ptr::write_bytes`.
- `mb.pre` / `mb.dst: Yv12BufferConfig` get a **bytewise copy** of a DPB
  slot (`decodeframe.rs:1149-1150`), then per-MB the y/u/v bases are
  overwritten (`decodeframe.rs:644-668`). Only `{y,u,v}_buffer` +
  `{y,uv}_stride` are ever read off them (reconinter.rs:343-448, etc.) —
  i.e. `mb.pre`/`mb.dst` act as a small per-MB *plane view*, not a real
  frame buffer.

### Two kinds of `Yv12BufferConfig` (the ownership invariant)

This distinction is load-bearing and the type system must encode it:

1. **Owned** — the DPB slots `yv12_fb[]`. `buffer_alloc` non-null; owns
   the slab; must be freed exactly once.
2. **Borrowed / aliasing** — built by `image2yuvconfig`
   (`vp8_dx_iface.rs:515`) for `VP8_SET_REFERENCE` / `VP8_COPY_REFERENCE`,
   and the future external-output-buffer slot. `y/u/v_buffer` point at
   **caller memory the decoder must never free**; `buffer_alloc == null`,
   `buffer_alloc_sz == 0`.

Today the two are distinguished only by the `buffer_alloc_sz > 0` guard.
The RAII rework must replace that runtime convention with a typed one.

### External-memory mechanisms (verified, must be preserved)

| Mechanism | Implemented? | Caller memory is… |
|---|---|---|
| Provide a reference frame (`VP8_SET_REFERENCE`) | yes | aliased transiently by `image2yuvconfig`, then **deep-copied** into a decoder slot (`vp8_yv12_copy_frame`) |
| Read a reference out (`VP8_COPY_REFERENCE`) | yes | caller buffer is the **copy destination** |
| Get decoded frame (`vpx_codec_get_frame`) | yes | **zero-copy alias** into decoder YV12; valid until next `decode()` |
| Provide an output buffer to decode *into* (`vpx_codec_set_frame_buffer_functions`) | **no — returns `INCAPABLE`** | n/a (the `FrameBufferAllocator` trait is defined but unwired, §14) |

Guard test: `tests/reference_control_test.rs` exercises the SET→COPY
round-trip with caller-owned buffers and asserts bit-exact pixels.

---

## Stages (dependency order)

### Stage 0 — Safety net + baseline
- `cargo bench -- --save-baseline yv12_pre`.
- Gate suite: `test_vector_test` (62 MD5), `invalid_file_test`,
  `vp8_decrypt_test`, `vpx_scale_test`, `decode_api_test`,
  `codec_trait_smoke`, **`reference_control_test`**.
- No code change.

### Stage 1 — Slim `mb.pre`/`mb.dst` to a plane view *(kills the internal struct copy)*
- Replace `Macroblockd.pre`/`dst: Yv12BufferConfig` with a small
  `PlaneRef { y, u, v: *mut u8, y_stride, uv_stride }` (still raw
  pointers — pure refactor, no ownership/alloc change).
- Delete the `ptr::copy_nonoverlapping(src, &mut pbi.mb.pre/dst, 1)` at
  `decodeframe.rs:1149-1150`; set strides once + y/u/v bases per-MB
  (already happening at `decodeframe.rs:644-668`).
- **Blast radius:** decodeframe (33), reconinter (18), reconintra4x4 (3),
  mbpitch (2), vp8_dx_iface (2).
- **Why first / safe:** removes one of the two struct-copy blockers,
  touches no allocation → MD5 + bench must be flat. Independently
  valuable.

### Stage 2 — Borrowed view for `get_reference` / `get_frame` output *(kills the external struct copy)*
- `onyxd_if.rs:431` (`ptr::copy_nonoverlapping(&yv12_fb[idx], sd, 1)`)
  copies a whole config into the caller's `sd`. Replace with a
  non-owning view (`&Yv12BufferConfig` or a `FrameView { planes,
  strides, dims, corrupted }`) so the caller reads pixels without
  owning the slab.
- **Blast radius:** onyxd_if, vp8_dx_iface (`yuvconfig2image` /
  `vp8_get_frame`), `codec::Decoder::get_frame`.
- **Why safe:** output pixels identical → MD5 unaffected; covered by
  `decode_api_test`, `codec_trait_smoke`, every vector, and the
  zero-copy aliasing of `vpx_codec_get_frame` is unchanged.

### Stage 3 — RAII the slab
- With no `Yv12BufferConfig` bytewise-copied anymore, replace
  `buffer_alloc: *mut u8` + `buffer_alloc_sz` with an owned
  **`Option<OwnedSlab>`** (32-byte-aligned `std::alloc` allocation,
  freed in `Drop`):
  - `Some(slab)` for DPB slots (owned).
  - **`None` for every caller-derived config** (`image2yuvconfig`) and
    the future external-output-buffer slot. This is the typed
    replacement for the `buffer_alloc_sz > 0` guard, and is
    forward-compatible with external output buffers (a decode-target
    slot may carry borrowed planes + `None`).
- `vp8_yv12_de_alloc_frame_buffer` stops calling `vpx_free` +
  `ptr::write_bytes`; realloc becomes `slab = Some(OwnedSlab::new(...))`.
  `y/u/v_buffer` stay raw `*mut u8` into the owned slab.
- The owned field makes the struct non-`Copy` / non-memcpy-able —
  **compiler-enforced**, which is why Stages 1–2 must land first.
- **Blast radius:** yv12config.rs, alloccommon.rs, onyxd_if
  (`vp8_remove_decoder_instances` / pool teardown), swapyv12buffer.
- **Gate:** MD5 + reference round-trip + a leak/double-free check (ASAN
  or valgrind if available; otherwise a Drop-count assertion).
  `vpx_memalign` / `vpx_free` drop out of this path.

### Stage 4 — *(optional, deferred, bench-gated)* planes as slices
- Convert `y/u/v_buffer` to a view that works for **both** owned slab
  memory and **borrowed external** memory — i.e. `{ base, stride }`,
  **not** a slab-relative offset (a slab-relative model would break
  SET/COPY reference, whose planes live in caller memory with caller
  stride). Access pixels via `get_unchecked` to avoid the bounds-check
  tax in the hot kernels (intrapred, filter, reconinter,
  loopfilter_filters, idct_blk, yv12extend).
- Done **per-kernel**, each diffed against `yv12_pre`; revert any kernel
  that regresses beyond an agreed threshold. May legitimately stay
  deferred per §15.

---

## Risks & mitigations

- **Alignment:** `OwnedSlab` must guarantee 32-byte alignment
  (`Layout::from_size_align(n, 32)`); plane-offset math and any future
  SIMD assume it.
- **`#[repr(C)]`:** may stay, but the struct must never again be
  bytewise-copied; the owned field enforces this.
- **Borrowed-config soundness:** caller-derived configs must carry
  `None` for the slab so `Drop` never frees foreign memory. This is the
  single most important correctness point and is guarded by
  `reference_control_test`.
- **Rollback:** one commit per stage → bisectable; each stage
  independently green.

## Verification protocol (every stage)

`cargo build --all-targets` (0 warnings) → `cargo test` (full suite incl.
reference round-trip) → `cargo bench --bench decode -- --baseline
yv12_pre` (within noise).
