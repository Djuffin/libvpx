# VP8 Decoder C → Rust Translation Summary

This document catalogs every meaningful difference between the libvpx
C VP8 decoder and the Rust port at `rust/`.

The **decoder kernel** (`decodeframe.rs`, `decodemv.rs`, `reconinter.rs`,
`reconintra.rs`, `vp8_loopfilter.rs`, `loopfilter_filters.rs`,
`intrapred.rs`, `filter.rs`, `idctllm.rs`, `dboolhuff.rs`, `detokenize.rs`,
and friends) is a **literal transliteration** intended for behavioral
parity with libvpx — function names, control flow, and pointer arithmetic
mirror the C source.

The **API/adapter layer** (`crate::codec`, `vpx_api`, `vpx_codec`,
`vpx_decoder`, `vp8_dx_iface`'s outer `Vp8Decoder`) has been **reshaped
around an idiomatic `Decoder` trait**; the original C-shape vtable and
`#[no_mangle]` / `extern "C"` decorators were removed once C-caller
compatibility was abandoned. See §6 for the full story.

**Verification baseline.** All 62 VP8 conformance test vectors
(`vp80-00-comprehensive-*.ivf` etc.) decode to bit-exact MD5 match with
libvpx C, across all 29-frame sequences. **162 ported tests pass**, zero
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

**~18,500 lines of Rust source** across 54 modules + 9 integration
test files. Roughly 1.5× the C line count, mostly due to:
- Explicit `unsafe { ... }` blocks at call sites in the kernel.
- Per-field type annotations on struct literals.
- Doc comments tracing every public function back to its C source line.

---

## 2. Module structure

The Rust crate mirrors the C source tree one-to-one for the decoder
kernel:

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

In addition, **9 new modules** exist that have no `.c` analog. They
correspond either to C **headers** that aren't compiled to `.o` files
but are load-bearing, or to the post-port trait-based API surface:

| Rust module | Origin | Purpose |
|---|---|---|
| `rust/src/types.rs` | many `.h` files | All shared struct / enum / typedef declarations |
| `rust/src/tables.rs` | many `.c` files (data only) | Every `const` table from VP8 / RFC 6386 |
| `rust/src/vpx_api.rs` | `vpx/*.h` | Public API types + re-exports |
| `rust/src/vp8_rtcd.rs` | `vp8_only/vp8_rtcd.h` (generated) | `#define X X_c` alias layer |
| `rust/src/vpx_ports.rs` | `vpx_ports/{system_state,vpx_once}.h` | `once()` + `vpx_clear_system_state()` shims |
| `rust/src/treereader.rs` | `vp8/decoder/treereader.h` (header-only) | `vp8_read`, `vp8_read_literal`, `vp8_treed_read` |
| `rust/src/codec.rs` | new (post-port) | Idiomatic `Decoder` / `Encoder` / `ControlCmd` traits |
| `rust/src/vp8_cx_stub.rs` | new (post-port) | `Vp8Encoder` placeholder — confirms trait shape compiles |
| `rust/src/vp9_dx_stub.rs` | new (post-port) | `Vp9Decoder` placeholder — same |

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

The **API/adapter layer is different** — see §6. The post-port API
surface uses `&mut T` / `&[u8]` / `Option<Box<dyn Decoder>>` and stays
fully on the safe-Rust side of the line. The crossing from safe API to
unsafe kernel happens at the `Vp8Decoder` trait impl boundary.

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

`Macroblockd`, `Vp8Common`, and most kernel structs are 16-byte aligned
to match the C `DECLARE_ALIGNED(16, ...)` macros. SIMD intrinsics
require this alignment for `_mm_load_si128` / `_mm_store_si128`; the
plane-pointer aliasing into `Yv12BufferConfig` buffers also depends on
it.

**Exception:** `Vp8dComp<'a>` and `Vp8AlgPriv<'a>` were retitled from
`#[repr(C)]` to `#[repr(align(16))]` (and plain `#[repr(Rust)]`
respectively) to hold `Option<Box<dyn FnMut(&[u8], &mut [u8]) + 'static>>`
fields (see §7). Layout-stable kernel structs are untouched.

---

## 4. Error handling — `setjmp`/`longjmp` → `Result<T, VpxCodecErr>`

The single largest divergence in the decoder kernel. libvpx C uses
`setjmp`/`longjmp` as a C-style exception mechanism:

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
pub fn vpx_internal_error<T>(
    info: &mut VpxInternalErrorInfo,
    error: VpxCodecErr,
) -> VpxResult<T> {
    info.error_code = error;
    Err(error)
}

// at a throw site:
return vpx_internal_error(&mut pc.error, VPX_CODEC_CORRUPT_FRAME);
```

`info` is a plain `&mut` rather than `*mut`: every throw site already
holds a `&mut …error`, so the function is safe and the `unsafe { … }`
wrappers around it are gone.

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

### 4.3 Bug exposed in the process

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
are still present (each a `pub fn` with a `std::sync::Once` guard,
no-op body). Called once at decoder construction.

The **kernel functions themselves** (`intrapred.rs`, `filter.rs`,
`idctllm.rs`, `reconintra.rs`) retain `pub unsafe extern "C" fn`
signatures so that future SIMD slot-in via libvpx's hand-rolled
`.asm`/`.S` files stays possible — the assembly assumes C ABI, and
mixing assembly kernels into the same RTCD dispatch table as Rust
kernels requires shared ABI. Pure-Rust SIMD intrinsics would not need
this; see §15 for the trade-off.

---

## 6. API surface evolution: from C-ABI vtable to `Decoder` trait

This section replaces the original "ABI considerations" section. After
the initial port, the public API was reshaped through six phases. The
result is an idiomatic Rust surface; the C-ABI scaffolding is gone.

### 6.1 What used to be there

The first cut of the port preserved drop-in `libvpx.so` compatibility:

- Every public entry point was `pub unsafe extern "C" fn` with
  `#[unsafe(no_mangle)]` so external linkers could resolve the symbols
  by their libvpx C names.
- The `VpxCodecIface` struct held **14 function-pointer slots**
  (`init`, `destroy`, `peek_si`, `get_si`, `decode`, `get_frame`,
  `set_fb_fn`, plus 8 control-ID thunks, plus the encoder slots) all
  typed `Option<unsafe extern "C" fn(...)>`.
- A static `VPX_CODEC_VP8_DX_ALGO: VpxCodecIface` carried the VP8
  decoder's slot bindings; `vpx_codec_vp8_dx()` returned a `*mut
  VpxCodecIface` to it.
- 14 `extern "C"` trampoline functions (`vp8_init_c`, `vp8_destroy_c`,
  …) in `vp8_dx_iface.rs` bridged Rust-ABI internal helpers to the
  C-ABI slots, doing the `*mut VpxCodecAlgPriv` → `*mut Vp8AlgPriv`
  downcast at each entry.
- Dispatch in `vpx_codec_decode` went
  `(*ctx).iface->dec.decode.unwrap()(get_alg_priv(ctx), data, sz, ...)`.

### 6.2 What replaced it

A small **trait surface** at `crate::codec`:

```rust
pub trait Decoder {
    fn decode(&mut self, data: &[u8], deadline: Duration) -> Result<(), Error>;
    fn get_frame(&mut self) -> Option<&Image>;
    fn control(&mut self, cmd: ControlCmd<'_>) -> Result<(), Error>;
    fn peek_stream_info(data: &[u8]) -> Result<StreamInfo, Error>
        where Self: Sized;
    fn stream_info(&self) -> Result<StreamInfo, Error>;
    fn flush(&mut self) -> Result<(), Error> { self.decode(&[], Duration::ZERO) }
}

pub trait Encoder { ... }    // stub for future VP9/VP8 encoder work

#[non_exhaustive]
pub enum ControlCmd<'a> {
    SetReference(&'a VpxRefFrame),
    CopyReference(&'a mut VpxRefFrame),
    SetPostproc(Vp8PostprocCfg),
    GetLastRefUpdates(&'a mut i32),
    GetFrameCorrupted(&'a mut i32),
    GetLastRefUsed(&'a mut i32),
    GetLastQuantizer(&'a mut i32),
    SetDecryptor(Option<&'a VpxDecryptInit>),
}
```

`Vp8Decoder` implements `Decoder`; it owns a `Box<Vp8AlgPriv<'static>>`
(see §7) and a per-frame `iter` cursor mirroring `vpx_codec_iter_t`.

The original `VpxCodecIface` shrunk from a 14-slot vtable to a 3-field
descriptor:

```rust
pub struct VpxCodecIface {
    pub name: *const c_char,
    pub abi_version: c_int,
    pub caps: VpxCodecCaps,
}
```

— purely metadata used for capability checks. **All 14 trampolines, the
fn-pointer typedefs, and `VP8_CTF_MAPS` were deleted.**

### 6.3 Public C-API entry points reshaped

The libvpx-named entry points (`vpx_codec_dec_init_ver`,
`vpx_codec_decode`, `vpx_codec_get_frame`, `vpx_codec_destroy`,
`vpx_codec_control_`, etc.) survive — but with idiomatic Rust
signatures, no longer C-ABI:

```rust
// Before:
pub unsafe extern "C" fn vpx_codec_decode(
    ctx: *mut VpxCodecCtx,
    data: *const u8,
    data_sz: c_uint,
    user_priv: *mut c_void,
    deadline: i64,
) -> VpxCodecErr

// After:
pub fn vpx_codec_decode(
    ctx: &mut VpxCodecCtx,
    data: &[u8],
    _user_priv: *mut c_void,
    _deadline: i64,
) -> VpxCodecErr
```

Inside, dispatch goes through the trait:
`ctx.trait_obj.as_mut().unwrap().decode(data, Duration::ZERO)`.

- `#[unsafe(no_mangle)]` removed from every public function.
- `extern "C"` removed from public API; kept on RTCD-table-pointed
  kernel `_c` functions for SIMD future-compatibility.
- The `ctx` parameter is a plain `&mut VpxCodecCtx`, not
  `Option<&mut …>`: a reference can't be null, so the C API's
  null-`ctx` → `INVALID_PARAM` arm is unrepresentable (the same way
  the null-buffer + nonzero-length `&[u8]` combos are). This applies
  to every `ctx`-mutating entry point (`dec_init_ver`, `decode`,
  `get_frame`, `get_stream_info`, `destroy`, `control_`, the cb
  registrars). The error-query helpers (`vpx_codec_error`,
  `vpx_codec_error_detail`) keep `Option<&VpxCodecCtx>` because a
  `None` there is meaningful — it returns a fallback description,
  matching C's `vpx_codec_error(NULL)`.

### 6.4 `VpxCodecCtx.trait_obj`

`VpxCodecCtx` gained a `trait_obj: Option<Box<dyn Decoder + 'static>>`
field. The `iface` field changed to `Option<&'static VpxCodecIface>`.
The struct is no longer `#[repr(C)]` (a `Box<dyn Trait>` field is not
FFI-safe), but no external C consumer was ever wired up.

`vpx_codec_destroy` collapses to `let _ = ctx.trait_obj.take()` —
`Box`'s `Drop` runs `Vp8Decoder::Drop` which releases the YV12 pool +
inner `Vp8dComp` instances; the `Box` itself reclaims the
`Vp8AlgPriv` shell.

---

## 7. Memory management

### 7.1 Internal allocators

`vpx_mem.c`'s allocator wrappers (`vpx_malloc`, `vpx_calloc`,
`vpx_realloc`, `vpx_memalign`, `vpx_free`) are thin shims over
`std::alloc::{alloc, alloc_zeroed, realloc, dealloc, Layout}`.

One deviation: C `vpx_malloc` stashes the original pointer **one
machine word before** the returned aligned pointer (for `free`'s use).
The Rust port stashes **two** `usize` words — the original pointer
*and* the original `Layout` — because `std::alloc::dealloc` requires
the `Layout` back. Header size constant changes accordingly. Callers
are oblivious.

### 7.2 Box / std::alloc / vpx_calloc split

After the trait refactor, the allocator landscape is:

| Allocation | Allocator | Freed by |
|---|---|---|
| `Vp8AlgPriv` (outer shell, ~few KiB) | `Box::new(zeroed())` → `std::alloc` | `Box` drop |
| `Vp8dComp` instances + YV12 frame buffer pool + MI grid | `vpx_calloc` / `vpx_memalign` | `vp8_remove_decoder_instances` (called from `Vp8Decoder::Drop` before Box drop) |
| Decryption-callback closure | `Box::new(closure)` | `Box` drop (inside `Vp8AlgPriv.decrypt`) |
| `Box<dyn Decoder>` (the trait object) | `Box::new(Vp8Decoder)` | `Box` drop |
| YV12 plane buffers (`buffer_alloc: *mut u8`) | `vpx_memalign` | `vpx_free` |
| MI grid (`Vp8Common.mip`) | `vpx_calloc` | `vpx_free` |

Inner slabs are freed **before** the outer `Box` shell drop runs, so
no cross-allocator free occurs. The kernel data structures (YV12
buffers, MI grid, residual scratch) remain raw-pointer-managed exactly
like C; only the outer aggregate moved to `Box`.

### 7.3 Decryption callback storage (`Box<dyn FnMut>`)

`Vp8AlgPriv.decrypt: Option<Box<dyn FnMut(&[u8], &mut [u8]) + 'static>>`
holds the per-codec decryption closure, built **once** when the
`VPXD_SET_DECRYPTOR` control runs. Each frame:

1. `vp8_decode` moves the `Box` from `Vp8AlgPriv.decrypt` to
   `Vp8dComp.decrypt` (a single `Option::take` swap).
2. Bool-decoder partitions borrow it via
   `(*pbi).decrypt.as_deref_mut()` — type alias `DecryptCbMut<'a>`.
3. After the frame, the `Box` moves back.

This replaces the pre-refactor pattern where a fresh `Box<dyn FnMut>`
was allocated per `vp8dx_start_decode` call.

Type aliases at `types.rs`:

```rust
pub type DecryptFn = dyn FnMut(&[u8], &mut [u8]) + 'static;
pub type DecryptCb = Box<DecryptFn>;            // owned form
pub type DecryptCbMut<'a> = &'a mut DecryptFn;  // borrowed form
```

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
    pub decrypt: Option<DecryptCbMut<'a>>,
}
```

This catches a class of fragmented-input bugs at compile time: the
input partition slice must outlive the decoder. The lifetime parameter
is uniformly `'static` throughout the codebase because the decoder is
embedded in `Vp8dComp<'static>` (the longest-lived owner), so the
borrow checker doesn't actually restrict behavior — but the
*signature* is more honest.

`vp8dx_start_decode` takes `Option<DecryptCbMut<'a>>`, a borrow into
`Vp8dComp.decrypt`. The bool decoder accesses the closure through the
borrow during refill; the underlying `Box` lives on `Vp8AlgPriv` (see §7).

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

10 signature changes total. None propagate further outward — the trait
boundary at `Vp8Decoder::decode` collapses `Ok(())` → `Ok(())`,
`Err(e)` → `Err(e)` directly.

---

## 10. Edition / lint configuration

`Cargo.toml` pins `edition = "2024"`. Crate-wide lint allows:

```rust
#![allow(non_snake_case)]              // C names like `vp8_dc2quant`
#![allow(non_camel_case_types)]        // typedefs like vpx_codec_err_t
#![allow(non_upper_case_globals)]      // const names like vp8_default_mv_context
#![allow(static_mut_refs)]             // RTCD tables are `static mut`
#![allow(unsafe_op_in_unsafe_fn)]      // see below
```

Edition 2024 enables `unsafe_op_in_unsafe_fn` by default — every
unsafe operation inside an `unsafe fn` body must be wrapped in its own
`unsafe { ... }` block. The decoder kernel is built on `unsafe fn`
bodies full of raw pointer dereferences (~4000 sites); wrapping each
individually has no behavioral payoff and would mostly add noise. The
crate-wide `#![allow(unsafe_op_in_unsafe_fn)]` preserves the old
"implicit unsafe body" semantics.

The lints that used to be in this list (`dead_code`, `unused_imports`,
`unused_variables`, `unused_assignments`, `unused_mut`,
`unused_unsafe`) have been removed; the codebase now compiles cleanly
with the default warning level. Test files individually carry
`#![allow(unsafe_op_in_unsafe_fn)]` since they define `unsafe fn`
helpers.

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

The Rust port ships **9 integration test files** under `rust/tests/`:

| Rust test file | C source | Tests | Notes |
|---|---|---|---|
| `vpx_image_test.rs` | `test/vpx_image_test.cc` | 5 | Image alloc/wrap/format validation |
| `idct_test.rs` | `test/idct_test.cc` | 4 | 4x4 IDCT correctness |
| `predict_test.rs` | `test/predict_test.cc` | 11 | Sub-pel filter random + preset data |
| `decode_api_test.rs` | `test/decode_api_test.cc` | 5 | Public API null/None guards + iface dispatch |
| `invalid_file_test.rs` | `test/invalid_file_test.cc` | 5 | Corrupt-bitstream error-code matching |
| `vpx_scale_test.rs` | `test/vpx_scale_test.cc` | 2 (49 size combos each) | YV12 border extension + frame copy |
| `vp8_decrypt_test.rs` | `test/vp8_decrypt_test.cc` | 1 | DRM bytestream decryption callback |
| `test_vector_test.rs` | `test/test_vector_test.cc` | 1 + 62 + 62 | Full conformance suite |
| `codec_trait_smoke.rs` | new (post-port) | 2 | Trait-API decode of comp-001 keyframe |

**162 tests pass, 0 failures, 0 ignored.** The conformance suite
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
| 1 | `init_frame` left `subpixel_predict*` function pointers uninitialized for inter frames — subagent stubbed them with "TODO: link once filter translations exist" | First inter frame SIGSEGV in `test_vector_test::full_vector_001` | `decodeframe.rs::init_frame`: assign sixtap or bilinear fn pointers per `pc->use_bilinear_mc_filter` |
| 2 | `vp8dx_receive_compressed_data` overwrote the correct error code with `VPX_CODEC_ERROR` because the post-longjmp cleanup block from C became reachable in Rust | `invalid_file_test` reported `VPX_CODEC_ERROR` where `VPX_CODEC_CORRUPT_FRAME` was expected | `onyxd_if.rs::vp8dx_receive_compressed_data`: drop the unreachable-in-C overwrite |
| 3 | Decryption callback never reached the bool decoder — `(*pbi).decrypt_cb = None;` was hardcoded at the FFI boundary | `vp8_decrypt_test` failed | Initially: retype `Vp8dComp::decrypt_cb` to match the FFI shape, copy from AlgPriv at frame init, build a `Box<dyn FnMut>` adapter per call. Later simplified — see §7.3. |
| 4 | `Vp8DxIface` had Rust-ABI fn pointers while `VpxCodecIface` had C-ABI — dispatch through the vtable was a no-op / type error | `decode_api_test::invalid_params_via_iface` couldn't even compile | Collapsed `Vp8DxIface` into `VpxCodecIface`; added 14 `extern "C"` trampolines. Later: the whole vtable was deleted in favor of the trait surface (§6). |
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
| VP8 encoder (`vp8/encoder/`) | `--disable-vp8-encoder` build target | Out of scope; `crate::codec::Encoder` trait shape is ready (`Vp8Encoder` stub) |
| VP9 codec (`vp9/`) | `--disable-vp9` | Out of scope; `crate::codec::Decoder` trait validated via `Vp9Decoder` stub |
| Post-processing (`vp8/common/postproc.c`, `mfqe`, etc.) | `--disable-postproc` | Tractable; ~3 source files, no decoder dependency |
| Error concealment | `--disable-error-concealment` | Tractable; isolated `#if`'d branches throughout decoder |
| Multi-threading | `--disable-multithread` | Significant; requires reasoning about per-MB-row workers, fragment partition routing, mutex/cv translation. `vpx_thread.rs` is a single-threaded shim |
| Spatial resampling | `--disable-spatial-resampling` | Standalone helpers — porting unblocks the scaler tests in `vpx_scale_test.cc`'s `ResetScaleImages` path |
| SIMD (NEON, SSE2, AVX2, ...) | `--target=generic-gnu` | Per-kernel; each `_c` kernel has 1-4 SIMD variants that would replace it at runtime via RTCD. Kernel `extern "C" fn` annotations preserved to keep this path open |
| `setjmp`/`longjmp` | Rust has no longjmp; replaced with `Result` | N/A — the new model is strictly better |
| Variadic `vpx_internal_error(..., fmt, ...)` formatted messages | Rust doesn't expose stable variadic dispatch | Cheap; attach `Cow<'static, str>` to `VpxError` and use `format!()` at throw sites |
| `vpx_codec_error_detail()` returning the formatted detail | Detail was dropped with the variadic formatter | Same as above |
| External frame-buffer registration (`vpx_codec_set_frame_buffer_functions`) | The C-ABI cb pair was dropped during the refactor. `crate::codec::FrameBufferAllocator` trait is defined but not yet wired | Wire the trait at decoder construction time |
| `*mut c_void user_priv` per-frame tagging | Dropped during trait refactor — the trait `decode` signature has no user_priv | Add an explicit `tag: Option<u64>` parameter or HashMap on the caller side |

---

## 15. Things that remain idiomatically un-Rust

The literal-translation rule still applies to the **decoder kernel**.
Several constructs stay un-idiomatic for layout/lifetime parity with
the C source:

- **Raw pointers throughout the kernel.** `Macroblockd`, `Vp8Common`,
  `Vp8dComp`, `Blockd`, `Yv12BufferConfig` all carry `*mut T` / `*const T`
  fields that alias into shared scratch arrays. Methods on these
  structs are `unsafe fn` and dereference raw pointers. (~250 occurrences.)
- **Pointer arithmetic in kernels.** `intrapred.rs`, `filter.rs`,
  `loopfilter_filters.rs`, `reconinter.rs`, `idctllm.rs` use `*p.add(i)`,
  `*p.offset(stride * j)`, etc. Going through `&[u8]` would add bounds
  checks on every sample access (~250 more occurrences; gated on a
  Criterion bench harness before any conversion).
- **`while i < N { ... i += 1; }`** instead of `for i in 0..N`. The
  raw loop matches the C source's induction-variable shape and makes
  side-by-side diffing easier.
- **C-style early returns.** Where the C says `goto cleanup;`, the Rust
  port repeats the cleanup at each early-return site.
- **`pub static mut`** for the iface metadata descriptor. Could be
  `const`, but the original C let users mutate it (e.g.,
  `vpx_set_worker_interface`); the field is kept for shape parity.
- **`*mut c_void`** survives in genuine FFI boundaries: `VpxImage.user_priv`,
  `VpxDecryptInit.decrypt_state`, `VPxWorker.data1/data2`,
  `VpxCodecCtx.priv_` sentinel. These intentionally type-erase
  caller-supplied state. (~75 occurrences.)

A rewrite that prioritized Rust idioms throughout (typed slices, owned
buffers, RAII, iterator-style loops, no raw pointer arithmetic) is
possible for the kernel layer as well, but is gated on a benchmark
harness — the bounds-check tax could measurably regress decode
throughput, and the bit-exact MD5 baseline must keep passing.

The **API/adapter layer** does NOT have these constraints — see §6.

---

## 16. Practical reading guide

When something in the Rust port looks weird, the cause is usually one
of these (in order of likelihood):

1. **C-to-Rust transliteration of an idiomatic-C construct in a kernel
   file.** Check the doc comment above the function — it cites the C
   source line.
2. **The function returns `VpxResult<T>` because the C version threw
   via `longjmp`.** See §4.
3. **A `pub use ... as ...` alias is doing what a `#define` did in C.**
   See §5 (RTCD).
4. **A `*mut T` field stores what would naturally be `&mut T` or
   `Box<T>` in Rust.** Required for layout / lifetime parity with C in
   the kernel. See §3.2 and §15.
5. **An `unsafe extern "C" fn` kernel function** — kept that ABI to
   preserve the libvpx-asm slot-in path. See §5.

The **API layer** (`crate::codec`, `vpx_api`, `vpx_codec`,
`vpx_decoder`, the outer `Vp8Decoder`) is safe-Rust idiomatic and does
not match any of patterns 1, 4, or 5. If you see raw pointers there,
they're either at the FFI boundary (callback state) or a sentinel
into the kernel — both documented inline.
