//! Translations of header-only shims from `vpx_ports/`.
//!
//! Two entities from `vpx_ports/*.h` are referenced widely by the
//! decoder but do not live in any `.c` file:
//!
//! - `vpx_clear_system_state()` — `vpx_ports/system_state.h`. Empties
//!   the x87 FPU state after MMX use. On non-x86/MMX builds the C
//!   header `#define`s it to nothing. We always compile that variant.
//!
//! - `once(func)` — `vpx_ports/vpx_once.h`. Runs `func` exactly once
//!   across the program's lifetime, thread-safely. Rust's
//!   [`std::sync::Once`] is the exact semantic match.

use std::sync::Once;

/// `vpx_clear_system_state()` — no-op on non-x86/MMX builds. RFC 6386
/// is silent; this is purely an `emms`-style FPU reset for SIMD paths.
#[inline]
pub fn vpx_clear_system_state() {}

/// `once(func)` — one-shot initialiser primitive from
/// `vpx_ports/vpx_once.h`.
///
/// In C the `once` macro stamps out a per-translation-unit lock and
/// calls `func` under it. Rust's `Once` does the same thing more
/// directly, but we need a *separate* `Once` per call site (the C
/// version's lock is per-translation-unit, but each `.c` file calls
/// `once(somefunc)` for at most one `somefunc`). Modelling that by
/// having the caller own their own `Once` would require touching every
/// call site, so instead we use a thread-safe map keyed by the function
/// pointer address. For the decoder this map only ever has 2-3 entries
/// (`initialize_dec`, `vp8_init_intra_predictors_internal`,
/// `vp8_init_intra4x4_predictors_internal`), so the cost is trivial.
///
/// Safety: `func` must be safe to call from any thread.
pub unsafe fn once(func: unsafe fn()) {
    use std::collections::HashMap;
    use std::sync::Mutex;
    use std::sync::OnceLock;

    static GUARDS: OnceLock<Mutex<HashMap<usize, &'static Once>>> = OnceLock::new();
    let map = GUARDS.get_or_init(|| Mutex::new(HashMap::new()));

    let key = func as usize;
    let once_ref: &'static Once = {
        let mut guard = map.lock().unwrap();
        *guard.entry(key).or_insert_with(|| Box::leak(Box::new(Once::new())))
    };

    once_ref.call_once(|| func());
}
