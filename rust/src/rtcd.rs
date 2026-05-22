//! `vp8/common/rtcd.c` — once-only initialiser for run-time CPU dispatch.
//!
//! On the `generic-gnu` decoder build, every dispatched kernel collapses
//! to a `#define foo vp8_foo_c` alias and `setup_rtcd_internal` has an
//! empty body, so this is a no-op init shim.

#![allow(dead_code)]

use std::sync::Once;

static RTCD_ONCE: Once = Once::new();

/// From `vp8_rtcd.h` (`#ifdef RTCD_C`). Empty on the `generic-gnu` build:
/// every kernel is a `#define foo vp8_foo_c` alias, so there is no
/// function-pointer table to populate. SIMD targets would populate it
/// here from `x86_simd_caps()` / `arm_cpu_caps()` / etc.
fn setup_rtcd_internal() {}

/// Public entry point of the VP8 RTCD subsystem. Idempotent and
/// thread-safe. Mirrors `void vp8_rtcd(void) { once(setup_rtcd_internal); }`
/// from `vp8/common/rtcd.c`.

pub fn vp8_rtcd() {
    RTCD_ONCE.call_once(setup_rtcd_internal);
}
