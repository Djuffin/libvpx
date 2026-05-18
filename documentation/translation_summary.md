# VP8 Decoder C → Rust Translation Summary

This document catalogs every meaningful difference between the libvpx
C VP8 decoder and the Rust port at `rust/`. The Rust port is a
**literal transliteration** intended for behavioral parity, not an
idiomatic rewrite — but C constructs that don't exist in Rust forced a
handful of deliberate deviations. They're enumerated here.

**Verification baseline.** All 62 VP8 conformance test vectors
(`vp80-00-comprehensive-*.ivf` etc.) decode to bit-exact MD5 match with
libvpx C, across all 29-frame sequences. 158 ported tests pass, zero
failures, zero ignored. `cargo build` reports 0 errors / 0 warnings.

---

## 1. Translation scope

The Rust port targets the **single-arch, decoder-only, no-postproc,
no-error-concealment, no-multithread** libvpx build, configured as:

```
--target=generic-gnu
--disable-vp9 --disable-vp8-encoder
--disable-postproc --disable-error-concealment --disable-multithread
--disable-spatial-resampling
```

That trims libvpx to **44 `.c` files** (see `documentation/vp8_files.md`).
Every one of those is translated.

Code paths gated by `CONFIG_POSTPROC`, `CONFIG_ERROR_CONCEALMENT`,
`CONFIG_MULTITHREAD`, `CONFIG_VP9_HIGHBITDEPTH`, or arch-specific SIMD
(`HAVE_NEON`, `HAVE_SSE2`, `HAVE_AVX2`, etc.) are **omitted entirely** in
the Rust port. Each such omission is documented at the call site that
would have housed the conditional code.

**~18,400 lines of Rust source** across 51 modules + 8 integration
test files. Roughly 1.5× the C line count, mostly due to:
- Explicit `unsafe { ... }` blocks at call sites.
- Per-field type annotations on struct literals.
- Doc comments tracing every public function back to its C source line.

---

## 2. Module structure

The Rust crate mirrors the C source tree one-to-one:

| C source | Rust module |
|---|---|
| `vp8/common/<name>.c` | `rust/src/<name>.rs` |
| `vp8/decoder/<name>.c` | `rust/src/<name>.rs` |
| `vp8/vp8_dx_iface.c` | `rust/src/vp8_dx_iface.rs` |
| `vpx/src/<name>.c` | `rust/src/<name>.rs` |
| `vpx_dsp/<name>.c` | `rust/src/<name>.rs` |
| `vpx_mem/vpx_mem.c` | `rust/src/vpx_mem.rs` |
| `vpx_scale/generic/<name>.c` | `rust/src/<name>.rs` |
| `vpx_util/<name>.c` | `rust/src/<name>.rs` |

In addition, **6 new modules** exist that have no `.c` analog. They
correspond to C **headers** that aren't compiled to `.o` files but are
load-bearing:

| New Rust module | Translated from | Purpose |
|---|---|---|
| `rust/src/types.rs` | many `.h` files | All shared struct / enum / typedef declarations |
| `rust/src/tables.rs` | many `.c` files (data only) | Every `const` table from VP8 / RFC 6386 |
| `rust/src/vpx_api.rs` | `vpx/*.h` | Public API types + re-exports |
| `rust/src/vp8_rtcd.rs` | `vp8_only/vp8_rtcd.h` (generated) | `#define X X_c` alias layer |
| `rust/src/vpx_ports.rs` | `vpx_ports/{system_state,vpx_once}.h` | `once()` + `vpx_clear_system_state()` shims |
| `rust/src/treereader.rs` | `vp8/decoder/treereader.h` (header-only) | `vp8_read`, `vp8_read_literal`, `vp8_treed_read` |

The `vpx_dsp_rtcd.rs` and `vpx_scale_rtcd.rs` modules each absorb their
respective RTCD `#define X X_c` aliases in addition to their original
init shim.

### 2.1 Data tables hoisted to a single module

Every `const` table in libvpx — RFC 6386 probabilities, IDCT cosines,
sub-pel filter taps, zig-zag scan orders, intra-mode trees, etc. — is
extracted into `rust/src/tables.rs`. **59 tables**, byte-for-byte
verified against the C source by `rust/scripts/verify_tables.py`.

C source files like `entropy.c`, `entropymode.c`, `quant_common.c`, and
`filter.c` therefore became thin Rust shims that re-export the
canonical table names back from `tables.rs` (`pub use crate::tables::X
as vp8_x;`) and translate only their few non-data functions.

---

## 3. Type system mapping

### 3.1 Primitive aliases

| C | Rust |
|---|---|
| `short` | `i16` |
| `unsigned short` | `u16` |
| `int` | `i32` (or `c_int` at FFI surfaces) |
| `unsigned int` | `u32` (or `c_uint`) |
| `long` | `i64` (or `c_long`; libvpx assumes LP64) |
| `size_t` | `usize` |
| `ptrdiff_t` | `isize` |
| `char` | `i8` (libvpx treats `char` as signed) |
| `unsigned char` | `u8` |
| `void *` | `*mut c_void` |
| `vpx_codec_err_t` (typedef of an `enum`) | `VpxCodecErr` (real Rust `#[repr(C)]` enum) |

### 3.2 Pointer types

VP8's internal data structures are **dense with raw pointers** — `BLOCKD`
contains four pointers into the parent `MACROBLOCKD`'s scratch arrays;
`MACROBLOCKD` contains pointers into the frame's MI grid; etc. We kept
these as **`*mut T` raw pointers** for layout parity, accepting that
every access is `unsafe`. Borrow-checker-friendly remodeling would have
required rewriting essentially every function and would not have
preserved the bit-exact decoder behavior.

### 3.3 `union` translation

The C `union b_mode_info` (`B_PREDICTION_MODE as_mode` / `int_mv mv`,
4 bytes) becomes a tagged enum:

```rust
pub enum BModeInfo {
    Intra(BPredictionMode),
    Mv(Mv),
}
```

This is **larger than the C union** (8 vs 4 bytes per element ×16
elements ×many MBs). Documented and accepted: the per-MB working set
grows by ~64 bytes, with no observable behavioral effect because
nothing in the decoder relies on the union's exact byte size.

The C `int_mv` union (`uint32_t as_int` / `MV as_mv`) collapses to a
plain `Mv` struct in Rust. Helper inlines (`mv_as_int`, `mv_from_int`)
mimic the C `.as_int` view for sites that need the 4-byte fast-equality
trick. Two small adapter functions in `findnearmv.rs` / `decodemv.rs` /
`reconinter.rs` bridge between the two views.

### 3.4 Enum variant ordering

Every `#[repr(u8)]` enum (`MbPredictionMode`, `BPredictionMode`,
`MvReferenceFrame`, etc.) is laid out in **C-enum order** so that
`SomeEnum::Variant as u8` matches the integer value the decoder
expects when indexing tree-decoder probability tables or RFC-6386
mode lookup tables.

### 3.5 Forced `#[repr(C, align(16))]`

`Macroblockd` and `Vp8Common` are 16-byte aligned to match the C
`DECLARE_ALIGNED(16, ...)` macros. The original C alignment was
required for SIMD intrinsics; we preserve it because the buffer
layouts are public-API-visible (the `vpx_image_t` wrapper exposes
plane pointers directly into these structs).

---

## 4. Error handling — `setjmp`/`longjmp` → `Result<T, VpxCodecErr>`

The single largest divergence. libvpx C uses `setjmp`/`longjmp` as a
C-style exception mechanism:

```c
// outermost trampoline:
if (setjmp(pbi->common.error.jmp)) {
    pbi->common.error.setjmp = 0;
    // cleanup, return error
}
pbi->common.error.setjmp = 1;
... call deep code that may longjmp ...
pbi->common.error.setjmp = 0;

// deep code (anywhere in the decoder):
vpx_internal_error(&pc->error, VPX_CODEC_CORRUPT_FRAME,
                   "Truncated packet");
// → sets info->error_code, then longjmp(info->jmp, code) — never returns
```

Rust has no `longjmp` and Rust's borrow checker objects to skipping
destructor execution. The port replaces all of it with **idiomatic
`Result` + `?`**:

```rust
pub type VpxResult<T> = Result<T, VpxCodecErr>;

#[inline]
pub unsafe fn vpx_internal_error<T>(
    info: *mut VpxInternalErrorInfo,
    error: VpxCodecErr,
) -> VpxResult<T> {
    (*info).error_code = error;
    Err(error)
}

// at a throw site:
return vpx_internal_error(&mut (*pc).error, VPX_CODEC_CORRUPT_FRAME);
```

### 4.1 What was dropped

- The `setjmp` + `jmp_buf` machinery (no Rust analog needed).
- The `setjmp: i32` flag on `vpx_internal_error_info` (no "armed?"
  state — Result is always live).
- The 80-byte `detail[80]` formatted-message buffer + `has_detail`
  flag. The public `vpx_codec_error_detail()` C API now returns
  `nullptr`. Acceptable for the prototype; if the API surface must
  match exactly later, attach a `Cow<'static, str>` to the error
  struct.
- The variadic `vpx_internal_error(info, err, fmt, ...)` signature —
  Rust doesn't expose stable variadic dispatch.

### 4.2 What that touched

| File | Change |
|---|---|
| `types.rs::VpxInternalErrorInfo` | Stripped to `error_code: VpxCodecErr` only |
| `vpx_codec.rs::vpx_internal_error` | 4-line `Result` constructor |
| `decodeframe.rs` | 13 throw sites converted; `vp8_decode_frame` + ~9 helpers return `VpxResult<...>` |
| `onyxd_if.rs` | 4 throw sites; `vp8dx_get/set_reference` and `vp8dx_receive_compressed_data` return `VpxResult` |
| `vp8_dx_iface.rs` | 5 throw sites; the 2 `setjmp`-guarded blocks in `vp8_decode` rewritten as `match` arms |

### 4.3 The four trampolines

The C source has 4 `setjmp` sites for the decoder. Each became an
explicit `Err` arm in Rust:

| C site | Rust equivalent |
|---|---|
| `create_decompressor` (onyxd_if.c:73) | `match create_decompressor_inner() { Err(_) => remove_decompressor; return null }` |
| `vp8_create_decoder_instances` (onyxd_if.c:430) | (cleanup lives in `create_decompressor`) |
| `vp8_decode` reso-change (vp8_dx_iface.c:402) | `match vp8_decode_resolution_change() { Err(_) => clear fragments; update_error_state }` |
| `vp8_decode` main (vp8_dx_iface.c:488) | `if let Err(_) = vp8dx_receive_compressed_data { mark fb corrupt; refcount--; update_error_state }` |

### 4.4 Bug exposed in the process

The C source unconditionally writes:
```c
pbi->common.error.error_code = VPX_CODEC_ERROR;
if (pbi->mb.error_info.error_code != 0) {
    pbi->common.error.error_code = pbi->mb.error_info.error_code;
}
```
*after* `vp8_decode_frame` returns -1 in `vp8dx_receive_compressed_data`.
That code is **unreachable in C** — `vpx_internal_error`'s `longjmp`
unwinds past it. The literal Rust translation made it reachable, and
it clobbered the correct `VPX_CODEC_CORRUPT_FRAME` value with
`VPX_CODEC_ERROR`. **Fix**: propagate the `Err(e)` value directly.
Surfaced when porting `invalid_file_test.cc`.

---

## 5. RTCD (Run-Time CPU Dispatch) layer

libvpx uses a generated header (`vp8_rtcd.h`, `vpx_dsp_rtcd.h`,
`vpx_scale_rtcd.h`) full of `#define X X_c` aliases at configure time
that, on SIMD-enabled builds, become function-pointer reads
(`X = pick_at_runtime(X_c, X_sse2, X_neon, ...)`).

The Rust port translates only the `_c` (C reference) variants of every
dispatchable kernel. The alias layer becomes a `pub use … as …` block:

```rust
// rust/src/vp8_rtcd.rs
pub use crate::idctllm::{
    vp8_short_idct4x4llm_c as vp8_short_idct4x4llm,
    vp8_short_inv_walsh4x4_c as vp8_short_inv_walsh4x4,
    ...
};
```

Same pattern in `vpx_dsp_rtcd.rs` (intra-predictors, ≈30 aliases) and
`vpx_scale_rtcd.rs` (YV12 buffer ops, 3 aliases).

The `vp8_rtcd()` / `vpx_dsp_rtcd()` / `vpx_scale_rtcd()` init shims
are still present (each `pub extern "C" fn` with a `std::sync::Once`
guard, no-op body) to preserve symbol-level FFI compatibility.

If SIMD lands later, these alias modules become the runtime-dispatch
selection points.

---

## 6. ABI considerations

### 6.1 `extern "C"` on every public symbol

Every function that the C public API exposes is `pub unsafe extern
"C" fn` with `#[unsafe(no_mangle)]`. This includes `vpx_codec_decode`,
`vpx_codec_destroy`, `vpx_img_alloc`, every `_c` kernel, every iface
vtable entry, etc.

### 6.2 The `Vp8DxIface` collapse

Originally, the agent that translated `vp8_dx_iface.c` invented a
*local* `Vp8DxIface` struct whose function-pointer slots used Rust ABI
(`Option<unsafe fn(...)>`). The canonical `VpxCodecIface` in
`vpx_api.rs` (which `vpx_codec_dec_init` consumes) uses C ABI
(`Option<unsafe extern "C" fn(...)>`). Two structs with the same shape
but incompatible fn-pointer ABI — dispatch through the vtable failed.

Fixed by introducing **14 `extern "C"` trampolines** in
`vp8_dx_iface.rs` and switching `VPX_CODEC_VP8_DX_ALGO` to the
canonical `VpxCodecIface` shape:

```rust
unsafe extern "C" fn vp8_destroy_c(ctx: *mut VpxCodecAlgPriv) -> VpxCodecErr {
    vp8_destroy(ctx as *mut Vp8AlgPriv<'static>)
}
```

Each trampoline does two jobs:
1. ABI switch (Rust → C).
2. Pointer-type cast (opaque `*mut VpxCodecAlgPriv` →
   concrete `*mut Vp8AlgPriv<'static>`).

Same pattern as how the C source does `(vpx_codec_alg_priv_t *) ctx`
casts internally. The local `Vp8DxIface` / `Vp8DxCtrlFnMap` / 8
fn-pointer typedefs were deleted.

### 6.3 No `extern "Rust"` blocks remain

Early in the translation, the per-`.c`-file subagents used `extern
"Rust" { fn name(...); }` forward declarations to call functions that
hadn't been translated yet. Once every module landed, **all such
blocks were eliminated** in favor of plain `use crate::module::name;`
imports. The compiler now enforces signature agreement.

A side-effect of this cleanup was finding three latent type-drift bugs
(`vp8_init_mbmode_probs` parameter type, `vp8_default_bmode_probs`
slice vs raw, `vp8_intra4x4_predict` enum vs `c_int`) that the loose
`extern` decls had been hiding.

---

## 7. Memory management

`vpx_mem.c`'s allocator wrappers (`vpx_malloc`, `vpx_calloc`,
`vpx_realloc`, `vpx_memalign`, `vpx_free`) became thin shims over
`std::alloc::{alloc, alloc_zeroed, realloc, dealloc, Layout}`.

One deviation: C `vpx_malloc` stashes the original pointer **one
machine word before** the returned aligned pointer (for `free`'s use).
The Rust port stashes **two** `usize` words — the original pointer
*and* the original `Layout` — because `std::alloc::dealloc` requires
the `Layout` back. Header size constant changes accordingly. Callers
are oblivious.

No `Vec`, no `Box<[T]>`, no `String` is used inside the decoder hot
path. The pool of YV12 frame buffers and the MI grid are still raw
`vpx_memalign`-allocated slabs, accessed via `*mut Yv12BufferConfig` /
`*mut ModeInfo` pointers, exactly like C.

`Box<dyn FnMut>` shows up in two narrow places:
- `BoolDecoder<'a>::decrypt` — the per-decoder decryption callback.
- The bridge in `decodeframe.rs::bridge_decrypt_cb` — a fresh
  `Box<dyn FnMut>` is built per `vp8dx_start_decode` call, wrapping
  the FFI `vpx_decrypt_cb` fn pointer.

---

## 8. Bool decoder lifetime

The C `BOOL_DECODER` carries raw `(user_buffer, user_buffer_end)`
pointers. The Rust port retypes it with a borrowed slice:

```rust
pub struct BoolDecoder<'a> {
    pub buffer: &'a [u8],
    pub pos: usize,
    pub value: BdValue,
    pub count: i32,
    pub range: u32,
    pub decrypt: Option<DecryptCb<'a>>,
}
```

This catches a class of fragmented-input bugs at compile time: the
input partition slice must outlive the decoder. The lifetime parameter
is uniformly `'static` throughout the codebase because the decoder is
embedded in `Vp8dComp<'static>` (the longest-lived owner), so the
borrow checker doesn't actually restrict behavior — but the
*signature* is more honest.

`vp8dx_start_decode` takes `decrypt_cb: Option<DecryptCb<'a>>` (a
`Box` closure). The bridge described in §7 builds a fresh closure
from the FFI `vpx_decrypt_cb` pointer stored on `Vp8dComp::decrypt_cb`.

---

## 9. The `Result` cascade through `vp8_decode_frame`

Removing setjmp meant making every function on the throw path
`Result`-returning. The signature changes:

| Function | C return | Rust return |
|---|---|---|
| `vp8_decode_frame` | `int` (0 = OK, -1 = err) | `VpxResult<()>` |
| `vp8dx_receive_compressed_data` | `int` | `VpxResult<()>` |
| `setup_token_decoder` | `void` (throws) | `VpxResult<()>` |
| `read_partition_size` | `unsigned int` | `c_uint` (no Err — never throws) |
| `read_available_partition_size` | `unsigned int` | `VpxResult<c_uint>` |
| `decode_mb_rows` | `void` | `VpxResult<()>` |
| `vp8dx_get_reference` | `vpx_codec_err_t` (in-band) | `VpxResult<()>` |
| `vp8dx_set_reference` | `vpx_codec_err_t` (in-band) | `VpxResult<()>` |

10 signature changes total. None propagate further outward — the FFI
boundary at `vp8_decode` collapses `Ok(())` → `VPX_CODEC_OK as i32`,
`Err(e)` → `e as i32`.

---

## 10. Edition / lint configuration

`Cargo.toml` pins `edition = "2021"` rather than 2024 because:

- Edition 2024 enables `unsafe_op_in_unsafe_fn` by default. The
  literal port is built on `unsafe fn` bodies full of raw pointer
  dereferences; wrapping each individual `*p`, `*q.add(i)`, etc., in
  `unsafe { ... }` adds ~4000 mechanical edits with no behavioral
  change. The 2021 edition keeps the older "implicit unsafe body"
  semantics, matching the C source structure.
- All other 2024 changes (panic vs abort defaults, `Future` send
  bounds, etc.) are irrelevant to this codebase.

A handful of crate-wide `#![allow(...)]` lints quiet noise that's
intrinsic to the literal-translation approach:

```rust
#![allow(non_snake_case)]          // C names like `vp8_dc2quant`
#![allow(non_camel_case_types)]    // typedefs like vpx_codec_err_t
#![allow(non_upper_case_globals)]  // const names like vp8_default_mv_context
#![allow(dead_code)]               // many helpers unused until later phases
#![allow(unused_imports)]
#![allow(unused_variables)]        // `(void)param;` in C → unused arg in Rust
#![allow(unused_assignments)]      // C-style init then conditional reassign
#![allow(unused_mut)]
#![allow(unused_unsafe)]
#![allow(static_mut_refs)]         // RTCD tables are `static mut`
```

---

## 11. Tables: generation pipeline

`rust/src/tables.rs` (~40 KB, 59 entries) was **not hand-written**.
The script `rust/scripts/extract_tables.py` lexes each table from the
C source, resolves enum-symbol references (e.g. `-DCT_EOB_TOKEN`)
through a small `CONSTS` map, and emits Rust literals with the same
nested shape. The companion `rust/scripts/verify_tables.py`
re-parses both sides and asserts element-wise equality across all 59
tables (1056-entry `DEFAULT_COEF_PROBS` 4-D table down to 1-byte
`MAX_PROB`).

To regenerate after a libvpx update:
```bash
python3 rust/scripts/extract_tables.py     # writes rust/src/tables.rs
python3 rust/scripts/verify_tables.py      # byte-for-byte check vs C
```

---

## 12. Tests

The Rust port ships **8 integration test files** under `rust/tests/`,
one per VP8-relevant C test in `test/*.cc`:

| Rust test file | C source | Tests | Notes |
|---|---|---|---|
| `vpx_image_test.rs` | `test/vpx_image_test.cc` | 5 | Image alloc/wrap/format validation |
| `idct_test.rs` | `test/idct_test.cc` | 4 | 4x4 IDCT correctness |
| `predict_test.rs` | `test/predict_test.cc` | 11 | Sub-pel filter random + preset data |
| `decode_api_test.rs` | `test/decode_api_test.cc` | 5 | Public API null guards + iface dispatch |
| `invalid_file_test.rs` | `test/invalid_file_test.cc` | 5 | Corrupt-bitstream error-code matching |
| `vpx_scale_test.rs` | `test/vpx_scale_test.cc` | 2 (49 size combos each) | YV12 border extension + frame copy |
| `vp8_decrypt_test.rs` | `test/vp8_decrypt_test.cc` | 1 | DRM bytestream decryption callback |
| `test_vector_test.rs` | `test/test_vector_test.cc` | 1 + 62 + 62 | Full conformance suite |

**158 tests pass, 0 failures, 0 ignored.** The conformance suite
(`test_vector_test.rs`) decodes every frame of all 62 official VP8
test vectors, MD5-comparing each frame's output against the
canonical `.md5` files. Total runtime: ~2.3 seconds for all 62
vectors end-to-end.

### 12.1 Tests deliberately not ported

- `test_vector_test.cc`'s `VP8MultiThreaded` instantiation (62
  vectors × 7 thread counts). The decoder build is single-threaded.
- `vp8_boolcoder_test.cc`, `vp8_fragments_test.cc`. Round-trip tests
  that need the VP8 encoder.
- `add_noise_test.cc`, `pp_filter_test.cc`. Post-processing tests
  with `--disable-postproc`.

### 12.2 Test data hardcoded path

`test_vector_test.rs`, `invalid_file_test.rs`, `vp8_decrypt_test.rs`
all read from a hardcoded `/home/eugene/projects/libvpx/vp8_only/`.
The C suite reads `LIBVPX_TEST_DATA_PATH` env var; the Rust port
should match that eventually.

### 12.3 IVF FOURCC quirk surfaced

Five of the 62 `vp80-03-segmentation-*` vectors have FOURCC `"I420"`
in their IVF file header rather than `"VP80"`. The actual frame
payloads are VP8 and decode correctly. The Rust `IvfReader` checks
only the `"DKIF"` magic; it skips the FOURCC field, which would
otherwise reject those files. Documented at `IvfReader::open`.

---

## 13. Bugs found while porting

| # | Bug | Surfaced via | Fix location |
|---|---|---|---|
| 1 | `init_frame` left `subpixel_predict*` function pointers uninitialized for inter frames — `subagent stubbed them with "TODO: link once filter translations exist"` | First inter frame SIGSEGV in `test_vector_test::full_vector_001` | `decodeframe.rs::init_frame`: assign sixtap or bilinear fn pointers per `pc->use_bilinear_mc_filter` |
| 2 | `vp8dx_receive_compressed_data` overwrote the correct error code with `VPX_CODEC_ERROR` because the post-longjmp cleanup block from C became reachable in Rust | `invalid_file_test` reported `VPX_CODEC_ERROR` where `VPX_CODEC_CORRUPT_FRAME` was expected | `onyxd_if.rs::vp8dx_receive_compressed_data`: drop the unreachable-in-C overwrite |
| 3 | Decryption callback never reached the bool decoder — `(*pbi).decrypt_cb = None;` was hardcoded at the FFI boundary | `vp8_decrypt_test` failed | Three-place fix: retype `Vp8dComp::decrypt_cb` to match the FFI shape, copy from AlgPriv at frame init, build a `Box<dyn FnMut>` adapter per `vp8dx_start_decode` call |
| 4 | `Vp8DxIface` had Rust-ABI fn pointers while `VpxCodecIface` had C-ABI — dispatch through the vtable was a no-op / type error | `decode_api_test::invalid_params_via_iface` couldn't even compile | Collapse `Vp8DxIface` into `VpxCodecIface`; add 14 `extern "C"` trampolines |
| 5 | 9 `extern "Rust" { fn ... }` declaration blocks across modules — each module independently re-declared its callees with potentially-drifting signatures | Spotted via `clashing_extern_declarations` lint after the iface unification | Replace all with `use crate::module::name;` imports — compiler enforces signatures |
| 6 | `IntraPredFn` type alias in `reconintra.rs` was `unsafe extern "Rust" fn` but the kernels in `intrapred.rs` are `unsafe extern "C" fn` | Link error during `cargo test` | Switch the alias to `extern "C"` |
| 7 | `reconintra4x4.rs` had a bare `extern "C" { fn vpx_*_predictor_4x4(...) }` block referencing the un-suffixed RTCD names that were never defined | Link error | Same RTCD-alias pattern: add 10 `pub use … as …` lines to `vpx_dsp_rtcd.rs` and import from there |
| 8 | `vp8dx_get_quantizer`'s `extern "Rust"` decl in `vp8_dx_iface.rs` was `*mut Vp8dComp` while the definition is `*const Vp8dComp` — silent ABI mismatch hidden by the loose extern declaration | Surfaced by replacing the extern with a real `use` import | Used the correct `*const` type |

All 8 bugs were artifacts of the literal-translation process; none
existed in the libvpx C source.

---

## 14. Things deliberately omitted

| C feature | Why omitted | Re-enable cost |
|---|---|---|
| VP8 encoder (`vp8/encoder/`) | `--disable-vp8-encoder` build target | Out of scope |
| VP9 codec (`vp9/`) | `--disable-vp9` | Out of scope |
| Post-processing (`vp8/common/postproc.c`, `mfqe`, etc.) | `--disable-postproc` | Tractable; ~3 source files, no decoder dependency |
| Error concealment | `--disable-error-concealment` | Tractable; isolated `#if`'d branches throughout decoder |
| Multi-threading | `--disable-multithread` | Significant; requires reasoning about per-MB-row workers, fragment partition routing, mutex/cv translation |
| Spatial resampling | `--disable-spatial-resampling` | Standalone helpers — porting unblocks the scaler tests in `vpx_scale_test.cc`'s `ResetScaleImages` path |
| SIMD (NEON, SSE2, AVX2, ...) | `--target=generic-gnu` | Per-kernel; each `_c` kernel has 1-4 SIMD variants that would replace it at runtime via RTCD |
| `setjmp`/`longjmp` | Rust has no longjmp; replaced with `Result` | N/A — the new model is strictly better |
| Variadic `vpx_internal_error(..., fmt, ...)` formatted messages | Rust doesn't expose stable variadic dispatch | Cheap; attach `Cow<'static, str>` to `VpxError` and use `format!()` at throw sites |
| `vpx_codec_error_detail()` returning the formatted detail | Detail was dropped with the variadic formatter | Same as above |

---

## 15. Things that remain idiomatically un-Rust

The literal-translation rule means several constructs stay un-idiomatic:

- **Raw pointers everywhere.** No `&`/`&mut`, no `Box<T>`, no `Vec<T>`
  inside the hot path. Every method on `Macroblockd`, `Vp8dComp`, etc.
  is `unsafe fn` and dereferences raw pointers.
- **`while i < N { ... i += 1; }`** instead of `for i in 0..N`. The
  raw loop matches the C source's induction-variable shape and makes
  side-by-side diffing easier.
- **C-style early returns.** Where the C says `goto cleanup;`, the Rust
  port repeats the cleanup at each early-return site. (One exception:
  the four trampoline functions, where the cleanup lives in a single
  `Err(_) => { … }` arm.)
- **`pub static mut`** for the iface vtable. Required to match
  `vpx_codec_vp8_dx()`'s C semantics (returns a pointer to a global
  iface struct).
- **`#[no_mangle]`** on every public function the C API exposes — even
  if no current consumer links against it from C, the symbol names
  match libvpx exactly.

A rewrite that prioritized Rust idioms (typed slices, builder
patterns, RAII, `Result` chaining, iterator-style loops) is possible
but is explicitly out of scope for the v1 port. The current shape
exists to make line-by-line comparison with `vp8/` straightforward
and to make any bug bisectable against the libvpx C source.

---

## 16. Practical reading guide

When something in the Rust port looks weird, the cause is usually one
of these (in order of likelihood):

1. **C-to-Rust transliteration of an idiomatic-C construct.** Check
   the doc comment above the function — it cites the C source line.
2. **The function returns `VpxResult<T>` because the C version threw
   via `longjmp`.** See §4.
3. **A `pub use ... as ...` alias is doing what a `#define` did in C.**
   See §5 (RTCD).
4. **A `*mut T` field stores what would naturally be `&mut T` or
   `Box<T>` in Rust.** Required for layout / lifetime parity with C.
   See §3.2.
5. **A function is `pub unsafe extern "C" fn` even though no C caller
   exists yet.** Required for ABI compatibility with consumers that
   may link via `#[no_mangle]` symbol resolution. See §6.

Anywhere that diverges from those five patterns is documented inline.
