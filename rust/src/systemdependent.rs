//! Literal Rust translation of `vp8/common/generic/systemdependent.c`.
//!
//! This translation unit historically performed two jobs:
//!   1. Run-time CPU-feature detection (now moved to `vpx_ports/*_cpudetect.c`).
//!   2. Installation of architecture-specific SIMD function pointers (now
//!      handled by the generated `setup_rtcd_internal()` invoked through
//!      `vp8_rtcd()` in `vp8/common/rtcd.c`).
//!
//! After those migrations the file's only residual responsibility is to
//! populate `VP8_COMMON::processor_core_count` from a host CPU-count probe
//! when `CONFIG_MULTITHREAD` is enabled.
//!
//! The minimal build targeted here (`--disable-multithread`) compiles the
//! entire body of `vp8_machine_specific_config` out: it becomes a no-op
//! that simply discards its argument. The `VP8_COMMON` struct in
//! `types.rs` likewise omits the `processor_core_count` field (gated by
//! `CONFIG_MULTITHREAD` in the C source), so there is nothing to write
//! to even if we wanted to.
//!
//! The RTCD function-pointer table is owned by a separate translation
//! unit and is not touched from here — see the file-level comment in
//! `documentation/vp8_files/systemdependent.md`.

#![allow(dead_code)]

use crate::types::Vp8Common;

// ---------------------------------------------------------------------------
// `static int get_cpu_count(void)` — the core-count probe.
//
// Only compiled when `CONFIG_MULTITHREAD` is set in the C build. The
// minimal Rust target is single-threaded so this helper is omitted; it
// is sketched here as a `#[cfg]`-gated stub for parity with the C
// source and to document the intended behavior should the multithreaded
// build ever be brought online.
// ---------------------------------------------------------------------------

#[cfg(feature = "multithread")]
fn get_cpu_count() -> i32 {
    // TODO: port the POSIX `sysconf(_SC_NPROCESSORS_ONLN)` /
    // Win32 `GetNativeSystemInfo` probe. Default fallback is 16; the
    // final result is clamped to a minimum of 1.
    let core_count: i32 = 16;
    if core_count > 0 {
        core_count
    } else {
        1
    }
}

// ---------------------------------------------------------------------------
// `void vp8_machine_specific_config(VP8_COMMON *ctx)` — public entry point.
//
// Called once per `VP8_COMMON` instance from `vp8_create_common()`
// (alloccommon.c). In the minimal single-threaded build this is a
// deliberate no-op; the parameter is intentionally unused, mirroring
// the C `(void)ctx;` cast that silences the unused-parameter warning.
// ---------------------------------------------------------------------------

pub unsafe fn vp8_machine_specific_config(ctx: *mut Vp8Common) {
    #[cfg(feature = "multithread")]
    {
        // (*ctx).processor_core_count = get_cpu_count();
        // TODO: enable once `Vp8Common::processor_core_count` is added
        // under a `multithread` cfg gate in `types.rs`.
        let _ = ctx;
    }
    #[cfg(not(feature = "multithread"))]
    {
        let _ = ctx;
    }
}
