//! `vp8/common/rtcd.c` — once-only initialiser for run-time CPU dispatch.
//!
//! In the C source this file is the single translation unit that
//! `#define RTCD_C` before including the generated `vp8_rtcd.h`. That
//! designates this TU as the home of the function-pointer table and of
//! the (generated) `setup_rtcd_internal` symbol. The entire visible
//! body is:
//!
//! ```c
//! void vp8_rtcd(void) { once(setup_rtcd_internal); }
//! ```
//!
//! On the verified `generic-gnu` decoder build, every dispatched kernel
//! collapses to a `#define foo vp8_foo_c` and `setup_rtcd_internal` has
//! an empty body — so this translation unit is, effectively, a no-op
//! init shim. We mirror that here.

#![allow(dead_code)]

// TODO: `std::sync::Once` is the natural Rust analogue of libvpx's
// `once()` helper (`vpx_ports/vpx_once.h`). Pulled in even though the
// generic-build `setup_rtcd_internal` is empty, so the call shape
// matches the C source verbatim.
use std::sync::Once;

static RTCD_ONCE: Once = Once::new();

/// Generated into `vp8_rtcd.h` under `#ifdef RTCD_C`. On the
/// `generic-gnu` build the body is empty — every kernel is a
/// `#define foo vp8_foo_c` alias and there is no function-pointer
/// table to populate. SIMD targets would populate it from
/// `x86_simd_caps()` / `arm_cpu_caps()` / etc. here.
fn setup_rtcd_internal() {
    // intentionally empty — matches the generic-build expansion of
    // `setup_rtcd_internal` in the generated `vp8_rtcd.h`.
}

/// `vp8_rtcd` — public entry point of the VP8 RTCD subsystem.
///
/// Idempotent and safe to call from any thread. Mirrors
/// `void vp8_rtcd(void) { once(setup_rtcd_internal); }` from
/// `vp8/common/rtcd.c:15`.
#[no_mangle]
pub extern "C" fn vp8_rtcd() {
    RTCD_ONCE.call_once(setup_rtcd_internal);
}
