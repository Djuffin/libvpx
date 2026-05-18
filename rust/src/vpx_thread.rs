//! `VPxWorker` worker-thread vtable.
//!
//! `CONFIG_MULTITHREAD = 0` for the supported build, so this is a
//! single-thread shim: `reset()` flips a flag, `launch()` calls
//! `execute()` inline, `end()` is essentially a no-op. The pthread
//! paths from the original `vpx_util/vpx_thread.c` are kept as
//! commented scaffolding for a future multi-thread port.
//!
//! Original C source (per libvpx's `vpx_thread.h`):
//!   <https://chromium.googlesource.com/webm/libwebp>

#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]
#![allow(clippy::missing_safety_doc)]

use core::ffi::{c_char, c_int, c_void};
use core::ptr;

// ===========================================================================
// Build switch — mirrors `#if CONFIG_MULTITHREAD` in the C source.
// In the minimal vp8_only build this is zero (single-thread), so all the
// pthread code below is dead in the same way it is in C.
// ===========================================================================

/// `CONFIG_MULTITHREAD` from `vpx_config.h`. Hard-coded to `0` for the
/// vp8_only configuration this crate targets.
pub const CONFIG_MULTITHREAD: c_int = 0;

// ===========================================================================
// Inline types (kept here to avoid editing `types.rs`).
// Mirrors `vpx_util/vpx_thread.h`.
// ===========================================================================

/// `VPxWorkerStatus` — three-state lifecycle of the worker object.
/// The numeric ordering `NOT_OK < OK < WORKING` is significant; both
/// `reset()` (`status_ < OK`) and `sync()` (`status_ <= OK`) compare
/// values, so the variants must stay in this order.
pub type VPxWorkerStatus = c_int;
pub const VPX_WORKER_STATUS_NOT_OK: VPxWorkerStatus = 0;
pub const VPX_WORKER_STATUS_OK: VPxWorkerStatus = 1;
pub const VPX_WORKER_STATUS_WORKING: VPxWorkerStatus = 2;

/// `VPxWorkerHook` — function the worker thread calls. Two opaque
/// pointers, returns nonzero on success and zero on error.
pub type VPxWorkerHook = Option<unsafe fn(*mut c_void, *mut c_void) -> c_int>;

/// `VPxWorkerImpl` — platform-dependent state (pthread handles in the
/// C source). Forward-declared in the C header as
/// `typedef struct VPxWorkerImpl VPxWorkerImpl;` and fully defined only
/// under `#if CONFIG_MULTITHREAD`. Kept opaque here — the field layout
/// would be `pthread_mutex_t mutex_; pthread_cond_t condition_; pthread_t thread_;`
/// in the threaded build. In the single-thread build the type is never
/// instantiated, mirroring the C source.
#[repr(C)]
pub struct VPxWorkerImpl {
    _private: [u8; 0],
}

/// `VPxWorker` — public synchronisation object. Field names match the
/// C struct verbatim (including the trailing underscores on `impl_` and
/// `status_`).
#[repr(C)]
pub struct VPxWorker {
    /// `VPxWorkerImpl *impl_;` — owned by the worker; NULL until `reset`
    /// succeeds, NULL again after `end`.
    pub impl_: *mut VPxWorkerImpl,
    /// `VPxWorkerStatus status_;` — current lifecycle state.
    pub status_: VPxWorkerStatus,
    /// `const char *thread_name;` — optional debugger label. Must
    /// outlive the worker; libvpx recommends <= 15 characters.
    pub thread_name: *const c_char,
    /// `VPxWorkerHook hook;` — function to call on `launch`/`execute`.
    pub hook: VPxWorkerHook,
    /// `void *data1;` — first opaque argument to `hook`.
    pub data1: *mut c_void,
    /// `void *data2;` — second opaque argument to `hook`.
    pub data2: *mut c_void,
    /// `int had_error;` — latching error bit; cleared only by `reset`.
    pub had_error: c_int,
}

/// `VPxWorkerInterface` — vtable of the six methods libvpx call sites
/// dispatch through. Every entry must be non-NULL; `vpx_set_worker_interface`
/// validates that.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct VPxWorkerInterface {
    pub init: Option<unsafe fn(worker: *mut VPxWorker)>,
    pub reset: Option<unsafe fn(worker: *mut VPxWorker) -> c_int>,
    pub sync: Option<unsafe fn(worker: *mut VPxWorker) -> c_int>,
    pub launch: Option<unsafe fn(worker: *mut VPxWorker)>,
    pub execute: Option<unsafe fn(worker: *mut VPxWorker)>,
    pub end: Option<unsafe fn(worker: *mut VPxWorker)>,
}

// ===========================================================================
// External helpers (vpx_mem) — declared here so the file is self-contained.
// In the C source these come from `vpx_mem/vpx_mem.h`. Only referenced by
// the multithreaded code paths that are dead in this build.
// ===========================================================================

// ===========================================================================
// `#if CONFIG_MULTITHREAD` block — the pthread-backed implementation.
// Translated literally for fidelity but never invoked in the single-thread
// build (the `CONFIG_MULTITHREAD == 0` arms below take over).
// ===========================================================================

/// Forward declaration mirroring
/// `static void execute(VPxWorker *const worker);` in the C source.
/// Defined further down.
unsafe fn execute_fwd(worker: *mut VPxWorker) {
    execute(worker);
}

/// `static THREADFN thread_loop(void *ptr)` — the body the OS thread
/// runs in the threaded build. Punted to a stub: pthread primitives are
/// not modelled here. With `CONFIG_MULTITHREAD = 0` no thread is ever
/// spawned, so this is never reached.
#[cfg(any())] // never compiled — placeholder for the threaded build
unsafe fn thread_loop(_ptr: *mut c_void) -> *mut c_void {
    // Body in C:
    //   pthread_mutex_lock(&worker->impl_->mutex_);
    //   for (;;) {
    //     while (worker->status_ == VPX_WORKER_STATUS_OK)
    //       pthread_cond_wait(&worker->impl_->condition_, &worker->impl_->mutex_);
    //     if (worker->status_ == VPX_WORKER_STATUS_WORKING) {
    //       pthread_mutex_unlock(&worker->impl_->mutex_);
    //       execute(worker);
    //       pthread_mutex_lock(&worker->impl_->mutex_);
    //       assert(worker->status_ == VPX_WORKER_STATUS_WORKING);
    //       worker->status_ = VPX_WORKER_STATUS_OK;
    //       pthread_cond_signal(&worker->impl_->condition_);
    //     } else {
    //       assert(worker->status_ == VPX_WORKER_STATUS_NOT_OK);
    //       break;
    //     }
    //   }
    //   pthread_mutex_unlock(&worker->impl_->mutex_);
    //   return THREAD_EXIT_SUCCESS;
    ptr::null_mut()
}

/// `static void change_state(VPxWorker *const worker, VPxWorkerStatus new_status)`
/// — main-thread side of the rendezvous. Punted: requires pthread mutex
/// and condition variable. Never called in the single-thread build.
#[cfg(any())]
unsafe fn change_state(_worker: *mut VPxWorker, _new_status: VPxWorkerStatus) {
    // if (worker->impl_ == NULL) return;
    // pthread_mutex_lock(&worker->impl_->mutex_);
    // if (worker->status_ >= VPX_WORKER_STATUS_OK) {
    //   while (worker->status_ != VPX_WORKER_STATUS_OK)
    //     pthread_cond_wait(&worker->impl_->condition_, &worker->impl_->mutex_);
    //   if (new_status != VPX_WORKER_STATUS_OK) {
    //     worker->status_ = new_status;
    //     pthread_cond_signal(&worker->impl_->condition_);
    //   }
    // }
    // pthread_mutex_unlock(&worker->impl_->mutex_);
}

// ===========================================================================
// The vtable methods — these have an unconditional definition in the C
// source. Branches gated by `#if CONFIG_MULTITHREAD` collapse to the
// single-thread form in this build.
// ===========================================================================

/// `static void init(VPxWorker *const worker)` — zero the struct and
/// explicitly set `status_` to `NOT_OK`. After this call the worker is
/// safe to pass to `end()` even if `reset()` is never called.
unsafe fn init(worker: *mut VPxWorker) {
    // memset(worker, 0, sizeof(*worker));
    ptr::write_bytes(worker, 0u8, 1);
    // worker->status_ = VPX_WORKER_STATUS_NOT_OK;
    (*worker).status_ = VPX_WORKER_STATUS_NOT_OK;
}

/// `static int sync(VPxWorker *const worker)` — wait for the worker to
/// finish; return `!had_error`. In the single-thread build there is
/// nothing to wait for (`launch()` ran synchronously), so only the
/// assert and the return remain.
unsafe fn sync(worker: *mut VPxWorker) -> c_int {
    if CONFIG_MULTITHREAD != 0 {
        // change_state(worker, VPX_WORKER_STATUS_OK);
        // (unreachable in this build)
    }
    debug_assert!((*worker).status_ <= VPX_WORKER_STATUS_OK);
    ((*worker).had_error == 0) as c_int
}

/// `static int reset(VPxWorker *const worker)` — allocate the impl and
/// spawn the OS thread (threaded build), or just flip `status_` to
/// `OK` (single-thread build). `had_error` is cleared either way —
/// `reset` is the only operation that does this.
unsafe fn reset(worker: *mut VPxWorker) -> c_int {
    let mut ok: c_int = 1;
    (*worker).had_error = 0;
    if (*worker).status_ < VPX_WORKER_STATUS_OK {
        if CONFIG_MULTITHREAD != 0 {
            // Threaded path — kept here in comment form, never executed
            // in the vp8_only single-thread build:
            //
            //   worker->impl_ = (VPxWorkerImpl *)vpx_calloc(1, sizeof(*worker->impl_));
            //   if (worker->impl_ == NULL) return 0;
            //   if (pthread_mutex_init(&worker->impl_->mutex_, NULL)) goto Error;
            //   if (pthread_cond_init(&worker->impl_->condition_, NULL)) {
            //     pthread_mutex_destroy(&worker->impl_->mutex_);
            //     goto Error;
            //   }
            //   pthread_mutex_lock(&worker->impl_->mutex_);
            //   ok = !pthread_create(&worker->impl_->thread_, NULL, thread_loop, worker);
            //   if (ok) worker->status_ = VPX_WORKER_STATUS_OK;
            //   pthread_mutex_unlock(&worker->impl_->mutex_);
            //   if (!ok) {
            //     pthread_mutex_destroy(&worker->impl_->mutex_);
            //     pthread_cond_destroy(&worker->impl_->condition_);
            //   Error:
            //     vpx_free(worker->impl_);
            //     worker->impl_ = NULL;
            //     return 0;
            //   }
            //
        } else {
            (*worker).status_ = VPX_WORKER_STATUS_OK;
        }
    } else if (*worker).status_ > VPX_WORKER_STATUS_OK {
        ok = sync(worker);
    }
    debug_assert!(ok == 0 || (*worker).status_ == VPX_WORKER_STATUS_OK);
    ok
}

/// `static void execute(VPxWorker *const worker)` — run the hook on
/// whichever thread calls this. OR the negated return value into
/// `had_error` (hook returns true on success, `had_error` is true on
/// failure).
unsafe fn execute(worker: *mut VPxWorker) {
    if let Some(hook) = (*worker).hook {
        let rc = hook((*worker).data1, (*worker).data2);
        (*worker).had_error |= (rc == 0) as c_int;
    }
}

/// `static void launch(VPxWorker *const worker)` — kick the worker.
/// In the threaded build this transitions state to `WORKING` and
/// signals the condvar; in the single-thread build it is literally
/// `execute()`.
unsafe fn launch(worker: *mut VPxWorker) {
    if CONFIG_MULTITHREAD != 0 {
        // change_state(worker, VPX_WORKER_STATUS_WORKING);
    } else {
        execute(worker);
    }
}

/// `static void end(VPxWorker *const worker)` — terminate the worker.
/// In the threaded build joins the OS thread and frees the impl block;
/// in the single-thread build only resets `status_` to `NOT_OK` and
/// asserts `impl_` was never allocated.
unsafe fn end(worker: *mut VPxWorker) {
    if CONFIG_MULTITHREAD != 0 {
        // if (worker->impl_ != NULL) {
        //   change_state(worker, VPX_WORKER_STATUS_NOT_OK);
        //   pthread_join(worker->impl_->thread_, NULL);
        //   pthread_mutex_destroy(&worker->impl_->mutex_);
        //   pthread_cond_destroy(&worker->impl_->condition_);
        //   vpx_free(worker->impl_);
        //   worker->impl_ = NULL;
        // }
    } else {
        (*worker).status_ = VPX_WORKER_STATUS_NOT_OK;
        debug_assert!((*worker).impl_.is_null());
    }
    debug_assert!((*worker).status_ == VPX_WORKER_STATUS_NOT_OK);
}

// ===========================================================================
// `g_worker_interface` — the default vtable. Mutable in C
// (`vpx_set_worker_interface` overwrites it); use a `static mut` here.
// The header warns the setter "is not thread-safe" and must be called
// before any workers exist, so the absence of any synchronisation
// matches the C semantics.
// ===========================================================================

/// `static VPxWorkerInterface g_worker_interface = { init, reset, sync,
///                                                   launch, execute, end };`
static mut g_worker_interface: VPxWorkerInterface = VPxWorkerInterface {
    init: Some(init),
    reset: Some(reset),
    sync: Some(sync),
    launch: Some(launch),
    execute: Some(execute),
    end: Some(end),
};

/// `int vpx_set_worker_interface(const VPxWorkerInterface *const winterface)`.
/// Validates that every entry of `winterface` is non-NULL, then copies
/// the struct into the global. Returns 1 on success, 0 on invalid input.

pub unsafe fn vpx_set_worker_interface(
    winterface: *const VPxWorkerInterface,
) -> c_int {
    if winterface.is_null()
        || (*winterface).init.is_none()
        || (*winterface).reset.is_none()
        || (*winterface).sync.is_none()
        || (*winterface).launch.is_none()
        || (*winterface).execute.is_none()
        || (*winterface).end.is_none()
    {
        return 0;
    }
    g_worker_interface = *winterface;
    1
}

/// `const VPxWorkerInterface *vpx_get_worker_interface(void)`.
/// Returns a pointer to the (possibly-overridden) default vtable.

pub unsafe fn vpx_get_worker_interface() -> *const VPxWorkerInterface {
    ptr::addr_of!(g_worker_interface)
}
