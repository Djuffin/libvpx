//! `VPxWorker` worker-thread vtable.
//!
//! `CONFIG_MULTITHREAD = 0` for the supported build, so this is a
//! single-thread shim: `reset()` flips a flag, `launch()` calls
//! `execute()` inline, `end()` is a no-op beyond resetting `status_`.
//!
//! Ported from `vpx_util/vpx_thread.{c,h}`, which libvpx in turn derived
//! from libwebp's `src/utils/thread.c`.

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::{c_char, c_int, c_void};
use core::ptr;

/// `CONFIG_MULTITHREAD` from `vpx_config.h`. Hard-coded to `0` for the
/// vp8_only configuration this crate targets.
pub const CONFIG_MULTITHREAD: c_int = 0;

/// `VPxWorkerStatus` — three-state lifecycle of the worker object.
/// The numeric ordering `NOT_OK < OK < WORKING` is significant; both
/// `reset()` (`status_ < OK`) and `sync()` (`status_ <= OK`) compare
/// values, so the variants must stay in this order.
pub type VPxWorkerStatus = c_int;
pub const VPX_WORKER_STATUS_NOT_OK: VPxWorkerStatus = 0;
pub const VPX_WORKER_STATUS_OK: VPxWorkerStatus = 1;
pub const VPX_WORKER_STATUS_WORKING: VPxWorkerStatus = 2;

/// `VPxWorkerHook` — user-supplied function the worker calls. Two opaque
/// pointers, returns nonzero on success and zero on error.
pub type VPxWorkerHook = Option<unsafe fn(*mut c_void, *mut c_void) -> c_int>;

/// `VPxWorkerImpl` — opaque platform-state placeholder (would hold
/// `pthread_mutex_t`/`pthread_cond_t`/`pthread_t` in a threaded build).
#[repr(C)]
pub struct VPxWorkerImpl {
    _private: [u8; 0],
}

/// `VPxWorker` — public synchronisation object. Field names match the
/// C struct verbatim.
#[repr(C)]
pub struct VPxWorker {
    /// Owned by the worker; NULL until `reset` succeeds, NULL again
    /// after `end`.
    pub impl_: *mut VPxWorkerImpl,
    pub status_: VPxWorkerStatus,
    /// Optional debugger label. Must outlive the worker; libvpx
    /// recommends <= 15 characters.
    pub thread_name: *const c_char,
    /// Function to call on `launch`/`execute`.
    pub hook: VPxWorkerHook,
    /// First opaque argument to `hook`.
    pub data1: *mut c_void,
    /// Second opaque argument to `hook`.
    pub data2: *mut c_void,
    /// Latching error bit; cleared only by `reset`.
    pub had_error: c_int,
}

/// `VPxWorkerInterface` — vtable of the six methods libvpx call sites
/// dispatch through. Every entry must be non-NULL;
/// `vpx_set_worker_interface` validates that.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct VPxWorkerInterface {
    pub init: Option<unsafe fn(worker: &mut VPxWorker)>,
    pub reset: Option<unsafe fn(worker: &mut VPxWorker) -> c_int>,
    pub sync: Option<unsafe fn(worker: &mut VPxWorker) -> c_int>,
    pub launch: Option<unsafe fn(worker: &mut VPxWorker)>,
    pub execute: Option<unsafe fn(worker: &mut VPxWorker)>,
    pub end: Option<unsafe fn(worker: &mut VPxWorker)>,
}

// ===========================================================================
// `#if CONFIG_MULTITHREAD` block — never compiled in the single-thread build.
// ===========================================================================

#[cfg(any())]
unsafe fn thread_loop(_ptr: *mut c_void) -> *mut c_void {
    ptr::null_mut()
}

#[cfg(any())]
unsafe fn change_state(_worker: &mut VPxWorker, _new_status: VPxWorkerStatus) {}

// ===========================================================================
// The vtable methods (`CONFIG_MULTITHREAD == 0` build).
// ===========================================================================

/// Zero the struct and set `status_` to `NOT_OK`. After this call the
/// worker is safe to pass to `end()` even if `reset()` is never called.
unsafe fn init(worker: &mut VPxWorker) {
    // C: memset(worker, 0, sizeof(*worker)).
    worker.impl_ = ptr::null_mut();
    worker.status_ = VPX_WORKER_STATUS_NOT_OK;
    worker.thread_name = ptr::null();
    worker.hook = None;
    worker.data1 = ptr::null_mut();
    worker.data2 = ptr::null_mut();
    worker.had_error = 0;
}

/// Wait for the worker to finish; return `!had_error`. Nothing to wait
/// for in the single-thread build.
unsafe fn sync(worker: &mut VPxWorker) -> c_int {
    debug_assert!(worker.status_ <= VPX_WORKER_STATUS_OK);
    (worker.had_error == 0) as c_int
}

/// Flip `status_` to `OK` and clear `had_error`.
unsafe fn reset(worker: &mut VPxWorker) -> c_int {
    let mut ok: c_int = 1;
    worker.had_error = 0;
    if worker.status_ < VPX_WORKER_STATUS_OK {
        worker.status_ = VPX_WORKER_STATUS_OK;
    } else if worker.status_ > VPX_WORKER_STATUS_OK {
        ok = sync(worker);
    }
    debug_assert!(ok == 0 || worker.status_ == VPX_WORKER_STATUS_OK);
    ok
}

/// Run the hook on whichever thread calls this. OR the negated return
/// value into `had_error`.
unsafe fn execute(worker: &mut VPxWorker) {
    if let Some(hook) = worker.hook {
        let rc = hook(worker.data1, worker.data2);
        worker.had_error |= (rc == 0) as c_int;
    }
}

/// Kick the worker: call `execute()` inline.
unsafe fn launch(worker: &mut VPxWorker) {
    execute(worker);
}

/// Terminate the worker; only resets `status_`.
unsafe fn end(worker: &mut VPxWorker) {
    worker.status_ = VPX_WORKER_STATUS_NOT_OK;
    debug_assert!(worker.impl_.is_null());
    debug_assert!(worker.status_ == VPX_WORKER_STATUS_NOT_OK);
}

// ===========================================================================
// `g_worker_interface` — the default vtable. `vpx_set_worker_interface`
// overwrites it. The header warns the setter "is not thread-safe" and
// must be called before any workers exist.
// ===========================================================================

static mut g_worker_interface: VPxWorkerInterface = VPxWorkerInterface {
    init: Some(init),
    reset: Some(reset),
    sync: Some(sync),
    launch: Some(launch),
    execute: Some(execute),
    end: Some(end),
};

/// Validate that every entry of `winterface` is non-NULL, then copy
/// the struct into the global. Returns 1 on success, 0 on invalid
/// input.
pub fn vpx_set_worker_interface(winterface: Option<&VPxWorkerInterface>) -> c_int {
    let Some(w) = winterface else { return 0 };
    if w.init.is_none()
        || w.reset.is_none()
        || w.sync.is_none()
        || w.launch.is_none()
        || w.execute.is_none()
        || w.end.is_none()
    {
        return 0;
    }
    unsafe { g_worker_interface = *w };
    1
}

/// Returns the (possibly-overridden) default vtable.
pub fn vpx_get_worker_interface() -> &'static VPxWorkerInterface {
    unsafe { &*ptr::addr_of!(g_worker_interface) }
}
