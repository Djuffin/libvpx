//! Translations of header-only shims from `vpx_ports/`.
//!
//! Two entities from `vpx_ports/*.h` are referenced widely by the
//! decoder but do not live in any `.c` file:
//!
//! - `vpx_clear_system_state()` — `vpx_ports/system_state.h`. Empties
//!   the x87 FPU state after MMX use. On non-x86/MMX builds the C
//!   header `#define`s it to nothing, so this is a no-op.
//!
//! - `once(func)` — `vpx_ports/vpx_once.h`. Runs `func` exactly once
//!   across the program's lifetime, thread-safely. Rust's
//!   [`std::sync::Once`] is the exact semantic match.

use std::collections::HashMap;
use std::sync::{Mutex, Once, OnceLock};

/// `vpx_clear_system_state()` — no-op on non-x86/MMX builds. RFC 6386
/// is silent; this is purely an `emms`-style FPU reset for SIMD paths.
#[inline]
pub fn vpx_clear_system_state() {}

/// `once(func)` — one-shot initialiser primitive from
/// `vpx_ports/vpx_once.h`.
///
/// In C the `once` macro stamps out a per-translation-unit lock and
/// calls `func` under it. Here a thread-safe map keyed by the function
/// pointer address holds a separate `Once` per distinct `func`.
///
/// Safety: `func` must be safe to call from any thread.
pub unsafe fn once(func: unsafe fn()) {
    static GUARDS: OnceLock<Mutex<HashMap<usize, &'static Once>>> = OnceLock::new();
    let map = GUARDS.get_or_init(|| Mutex::new(HashMap::new()));

    let key = func as usize;
    let once_ref: &'static Once = {
        let mut guard = map.lock().unwrap();
        guard
            .entry(key)
            .or_insert_with(|| Box::leak(Box::new(Once::new())))
    };

    once_ref.call_once(|| func());
}
