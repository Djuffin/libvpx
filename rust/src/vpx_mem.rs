//! `vpx_mem/vpx_mem.c` — the codec's memory front door.
//!
//! Literal transliteration of the four public entry points
//! (`vpx_memalign`, `vpx_malloc`, `vpx_calloc`, `vpx_free`) plus the
//! five `static` helpers (`check_size_argument_overflow`,
//! `get_malloc_address_location`, `get_aligned_malloc_size`,
//! `set_actual_malloc_address`, `get_actual_malloc_address`).
//!
//! The C code over-allocates and stashes the original `malloc` pointer
//! in the `sizeof(size_t)` slot immediately preceding the returned
//! address, so `free` can recover it. Rust's `std::alloc::dealloc`
//! additionally requires the original [`Layout`] (size + align), so the
//! header here holds **two** `usize` words: the original allocation
//! pointer and the original allocation size. The header slot grows from
//! `sizeof(size_t)` to `2 * sizeof(size_t)`, which is wider than the C
//! original but invisible to callers.
//!
//! There is no `vpx_realloc` in the C source — the codec uses the
//! explicit `vpx_free` + `vpx_calloc` pattern instead — so none is
//! provided here either.

#![allow(dead_code)]
#![allow(non_upper_case_globals)]

use core::ffi::c_void;
use core::ptr;
use std::alloc::{alloc, dealloc, Layout};

// ===========================================================================
// vpx-mem-specific types.
// ===========================================================================

/// Mirrors C `size_t` on the target. On every platform supported by
/// libvpx, `usize` and `size_t` have the same width.
#[allow(non_camel_case_types)]
pub type size_t = usize;

// ===========================================================================
// Constants (mirrors `include/vpx_mem_intrnl.h` + the
// `VPX_MAX_ALLOCABLE_MEMORY` block at the top of `vpx_mem.c`).
// ===========================================================================

/// `ADDRESS_STORAGE_SIZE` from `include/vpx_mem_intrnl.h`. In C this is
/// `sizeof(size_t)`; here we widen it to two `usize` words so the
/// stash can hold the original pointer **and** the original
/// allocation size (Rust's `dealloc` needs the `Layout` back).
pub const ADDRESS_STORAGE_SIZE: size_t = 2 * core::mem::size_of::<size_t>();

/// `DEFAULT_ALIGNMENT` from `include/vpx_mem_intrnl.h`:
/// `2 * sizeof(void *)` on non-VxWorks targets. That is 16 on 64-bit
/// hosts, 8 on 32-bit hosts — the smallest alignment that an SSE
/// `__m128i` load tolerates.
pub const DEFAULT_ALIGNMENT: size_t = 2 * core::mem::size_of::<*mut c_void>();

/// `VPX_MAX_ALLOCABLE_MEMORY` ceiling — 1 TiB on 64-bit, just under
/// 2 GiB on 32-bit. See `vpx_mem.c` lines 19-26.
pub const VPX_MAX_ALLOCABLE_MEMORY: u64 = {
    // `usize::MAX > (1ULL << 40)` <=> 64-bit host.
    if (usize::MAX as u64) > (1u64 << 40) {
        1u64 << 40
    } else {
        (1u64 << 31) - (1u64 << 16)
    }
};

// ===========================================================================
// `align_addr` macro (from `include/vpx_mem_intrnl.h`).
// ===========================================================================

/// Round `addr` up to the next multiple of `align`. `align` must be a
/// power of two (no runtime check — matches the C macro).
#[inline]
fn align_addr(addr: *mut u8, align: size_t) -> *mut u8 {
    let a = addr as size_t;
    let mask = align - 1;
    ((a + mask) & !mask) as *mut u8
}

// ===========================================================================
// Static helpers.
// ===========================================================================

/// `check_size_argument_overflow` — returns `false` (0) on overflow,
/// `true` (1) otherwise.
fn check_size_argument_overflow(nmemb: u64, size: u64) -> bool {
    let total_size: u64 = nmemb.wrapping_mul(size);
    if nmemb == 0 {
        return true;
    }
    if size > VPX_MAX_ALLOCABLE_MEMORY / nmemb {
        return false;
    }
    // Catches the case where the 64-bit product does not fit in `size_t`.
    if total_size != (total_size as size_t) as u64 {
        return false;
    }
    true
}

/// `get_malloc_address_location` — return the address of the stash slot
/// (one `ADDRESS_STORAGE_SIZE`-sized step before `mem`).
#[inline]
unsafe fn get_malloc_address_location(mem: *mut c_void) -> *mut size_t {
    // Two-word header: `[orig_ptr, orig_size]`. Step back two `usize`s.
    (mem as *mut size_t).offset(-2)
}

/// `get_aligned_malloc_size` — total bytes to request from the
/// underlying allocator.
#[inline]
fn get_aligned_malloc_size(size: size_t, align: size_t) -> u64 {
    (size as u64) + (align as u64) - 1 + (ADDRESS_STORAGE_SIZE as u64)
}

/// `set_actual_malloc_address` — write the original allocation pointer
/// (and its size, which Rust needs to free) into the stash slot.
#[inline]
unsafe fn set_actual_malloc_address(
    mem: *mut c_void,
    malloc_addr: *const c_void,
    alloc_size: size_t,
) {
    let slot = get_malloc_address_location(mem);
    // Word 0: original pointer.  Word 1: original allocation size.
    *slot = malloc_addr as size_t;
    *slot.offset(1) = alloc_size;
}

/// `get_actual_malloc_address` — recover the original allocation
/// pointer from the stash slot. The returned tuple's second member is
/// the size used by the original `Layout`, needed for `dealloc`.
#[inline]
unsafe fn get_actual_malloc_address(mem: *mut c_void) -> (*mut c_void, size_t) {
    let slot = get_malloc_address_location(mem);
    let addr = *slot as *mut c_void;
    let sz = *slot.offset(1);
    (addr, sz)
}

// ===========================================================================
// Public entry points.
// ===========================================================================

/// `void *vpx_memalign(size_t align, size_t size);`
///
/// Returns a pointer aligned to at least `align` bytes (`align` must be
/// a power of two), valid for `size` bytes of access, or `NULL` on
/// overflow / OOM.

pub unsafe fn vpx_memalign(align: size_t, size: size_t) -> *mut c_void {
    let mut x: *mut c_void = ptr::null_mut();
    let aligned_size: u64 = get_aligned_malloc_size(size, align);
    if !check_size_argument_overflow(1, aligned_size) {
        return ptr::null_mut();
    }

    let total = aligned_size as size_t;
    // Rust's allocator demands a non-zero size and a valid Layout. The
    // C runtime is more forgiving (`malloc(0)` may return NULL or a
    // freeable pointer); on this path `total >= ADDRESS_STORAGE_SIZE`,
    // so it is always > 0.
    let layout = match Layout::from_size_align(total, core::mem::size_of::<size_t>()) {
        Ok(l) => l,
        Err(_) => return ptr::null_mut(),
    };
    let addr = alloc(layout) as *mut c_void;
    if !addr.is_null() {
        // Skip past the header slot, then round up to `align`.
        let after_header = (addr as *mut u8).add(ADDRESS_STORAGE_SIZE);
        x = align_addr(after_header, align) as *mut c_void;
        set_actual_malloc_address(x, addr, total);
    }
    x
}

/// `void *vpx_malloc(size_t size);` — `vpx_memalign(DEFAULT_ALIGNMENT, size)`.

pub unsafe fn vpx_malloc(size: size_t) -> *mut c_void {
    vpx_memalign(DEFAULT_ALIGNMENT, size)
}

/// `void *vpx_calloc(size_t num, size_t size);` — zero-initialised
/// allocation. Routes through `vpx_malloc` so the stash header is set
/// up correctly (libc `calloc` would not be `vpx_free`-safe).

pub unsafe fn vpx_calloc(num: size_t, size: size_t) -> *mut c_void {
    if !check_size_argument_overflow(num as u64, size as u64) {
        return ptr::null_mut();
    }

    let total = num * size;
    let x = vpx_malloc(total);
    if !x.is_null() {
        ptr::write_bytes(x as *mut u8, 0, total);
    }
    x
}

/// `void vpx_free(void *memblk);` — symmetric release. No-op on
/// `NULL`. `memblk` must have been returned by one of the `vpx_*`
/// allocators; passing anything else is undefined behaviour.

pub unsafe fn vpx_free(memblk: *mut c_void) {
    if !memblk.is_null() {
        let (addr, total) = get_actual_malloc_address(memblk);
        // Reconstruct the same Layout used by `vpx_memalign`.
        let layout =
            Layout::from_size_align_unchecked(total, core::mem::size_of::<size_t>());
        dealloc(addr as *mut u8, layout);
    }
}
