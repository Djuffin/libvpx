# `vpx_util/vpx_thread.c` — the generic worker-thread pool

`vpx_thread.c` is a small, self-contained library that turns the
amorphous pthread primitives (`pthread_create`, `pthread_join`,
`pthread_mutex_*`, `pthread_cond_*`) into a single, well-typed object
called a `VPxWorker`. A worker is a long-lived thread that sits idle on
a condition variable until the main thread asks it to run a *hook*
function exactly once, after which it goes back to sleep. The same
worker can be re-launched arbitrarily many times; only `reset()` ever
calls `pthread_create`, and only `end()` ever calls `pthread_join`.

The file is a direct descendant of WebP's
[`src/utils/thread.c`](https://chromium.googlesource.com/webm/libwebp/+/refs/heads/main/src/utils/thread_utils.c)
— the copyright header explicitly cites the origin:

```c
// Original source:
//  https://chromium.googlesource.com/webm/libwebp
```

It has been adopted unchanged in spirit by both VP8 and VP9 inside
libvpx; the same `VPxWorkerInterface` vtable backs row-based threading
in the VP8 decoder (`vp8/decoder/threading.c`) and tile-row threading in
the VP9 codec.

## Role in the decoder

VP8's threaded decode path (described in §13 of the technical overview)
spawns one worker per token partition. Each worker decodes a horizontal
strip of macroblocks and synchronises with the worker on the row above
via a shared `mt_current_mb_col[]` array. The mechanics of the rendezvous
— "wake up, run a hook, signal completion, sleep again" — are not coded
in `threading.c` itself. They live here, behind a six-function vtable:

| Method      | Purpose                                                |
|-------------|--------------------------------------------------------|
| `init`      | Zero the struct; mark the worker `NOT_OK`.             |
| `reset`     | Allocate the impl, create the OS thread, mark `OK`.    |
| `launch`    | Wake the thread; it will run `hook(data1, data2)`.     |
| `execute`   | Run `hook` synchronously on the calling thread (bypass)|
| `sync`      | Block until the launched hook has finished.            |
| `end`       | Tell the thread to exit; join it; free the impl.       |

The interface is published in
`vpx_util/vpx_thread.h` as the `VPxWorkerInterface` struct and reached
through `vpx_get_worker_interface()`; the table itself
(`g_worker_interface`) is overridable at runtime via
`vpx_set_worker_interface()` so a host application can substitute, for
instance, a fibre-based or task-stealing back end.

### Why this file is compiled even with `--disable-multithread`

The build verified in `vp8_files.md` was configured with
`--disable-multithread`, yet `vpx_thread.c` still appears in the §A list
of the 47 mandatory `.c` files. The reason is that the file participates
in the build unconditionally:

* `vpx_util/vpx_util.mk` lists `vpx_thread.c` without an `ifeq` guard.
* The file itself wraps only its *implementation* in
  `#if CONFIG_MULTITHREAD`. The six public-vtable functions (`init`,
  `reset`, `sync`, `launch`, `execute`, `end`) plus `g_worker_interface`
  and `vpx_set_worker_interface` / `vpx_get_worker_interface` are
  always emitted.
* `onyxd_if.c` and `vp8_dx_iface.c` call `vpx_get_worker_interface()`
  unconditionally during decoder lifecycle setup; resolving that symbol
  at link time requires the file to be present.

When `CONFIG_MULTITHREAD == 0`, the public functions degenerate to a
trivial single-thread implementation: `launch()` simply calls
`execute()` inline, `reset()` flips a flag, and `end()` is a no-op. No
pthread symbol is referenced, so the binary has no libpthread
dependency. This is why `vp8_files.md` flags `vpx_thread.{c,h}` and
`vpx_pthread.h` as "safely deletable in a fork" once you commit to a
single-thread build: the code compiles but no thread is ever spawned.

---

## The data model

### `VPxWorker` — the public synchronisation object

Declared in `vpx_thread.h`:

```c
typedef struct {
  VPxWorkerImpl *impl_;
  VPxWorkerStatus status_;
  const char *thread_name;
  VPxWorkerHook hook;
  void *data1;
  void *data2;
  int had_error;
} VPxWorker;
```

The caller owns the struct (typically as an array element inside the
codec context) and treats every field but `hook`, `data1`, `data2`, and
`thread_name` as opaque. The contract is:

* `init()` must be called first. It zeroes everything; `status_` becomes
  `VPX_WORKER_STATUS_NOT_OK` (the enum's `0` value, so the `memset` and
  the explicit assignment in `init()` agree).
* Before each `launch()` / `execute()`, the caller writes `hook`,
  `data1`, `data2`. Those three may be changed only when `status_ <=
  VPX_WORKER_STATUS_OK`, i.e. when no work is in flight. The header
  explicitly warns: "hook/data1/data2 values can be changed at any time
  before calling this function, but not be changed afterward until the
  next call to Sync()."
* `had_error` is *latching*: every time the hook returns false, the bit
  is ORed in; only `reset()` clears it. The reason is composability —
  decode of an entire row may launch dozens of hooks but the caller only
  wants to know "did any of them fail?" at sync time.
* `thread_name` is optional and used only to label the OS thread in a
  debugger. It must outlive the worker (the lifetime requirement is
  spelled out in the header).

### `VPxWorkerImpl` — the private platform-dependent state

```c
struct VPxWorkerImpl {
  pthread_mutex_t mutex_;
  pthread_cond_t condition_;
  pthread_t thread_;
};
```

The split between `VPxWorker` (public) and `VPxWorkerImpl` (private)
serves two purposes:

1. It hides `pthread.h` from the rest of libvpx. Only `vpx_thread.c`
   and `vpx_pthread.h` include the system header; everywhere else uses
   the opaque `VPxWorkerImpl *`. This is what lets the no-thread build
   omit the include path entirely.
2. It allocates the OS resources lazily. The `impl_` pointer is `NULL`
   until `reset()` succeeds in calling `vpx_calloc`, and is set back to
   `NULL` by `end()`. Code that wants to know "is this worker live?"
   needs only `worker->impl_ != NULL` — a check `change_state()` performs
   on its first line, defensively bailing out if the worker never came
   up.

The struct is forward-declared in the header as
`typedef struct VPxWorkerImpl VPxWorkerImpl;` and fully defined here
under `#if CONFIG_MULTITHREAD`. In the no-thread build the type has no
definition because it is never instantiated.

### `VPxWorkerStatus` — the three-state lifecycle

```c
typedef enum {
  VPX_WORKER_STATUS_NOT_OK = 0,  // object is unusable
  VPX_WORKER_STATUS_OK,          // ready to work
  VPX_WORKER_STATUS_WORKING      // busy finishing the current task
} VPxWorkerStatus;
```

The whole locking protocol revolves around this enum. The ordering
`NOT_OK < OK < WORKING` is used numerically in `reset()` (`if
(worker->status_ < VPX_WORKER_STATUS_OK)`) and `sync()` (`assert(...
<= VPX_WORKER_STATUS_OK)`), so the values cannot be reordered.

The transitions are:

```
                init()                  reset()                 launch()
   (nothing) ─────────► NOT_OK ────────────────────► OK ──────────────────► WORKING
                          ▲                          │ ▲                     │
                          │           end()          │ │   sync() / hook done│
                          └──────────────────────────┘ └─────────────────────┘
```

`NOT_OK` is both the *initial* and the *terminal* state. `reset()` is
re-entrant: if called when the worker is already running (`status_ >
VPX_WORKER_STATUS_OK`), it just delegates to `sync()` to drain the
in-flight job and returns success without re-creating the thread.

---

## The control protocol

The four state-changing operations (`reset`, `launch`, `sync`, `end`)
are implemented on top of a single mutex / condition-variable pair.
Everything funnels through one helper, `change_state`, which captures
the locking discipline in one place.

### `change_state` — the main-thread side of the rendezvous

```c
static void change_state(VPxWorker *const worker, VPxWorkerStatus new_status) {
  if (worker->impl_ == NULL) return;
  pthread_mutex_lock(&worker->impl_->mutex_);
  if (worker->status_ >= VPX_WORKER_STATUS_OK) {
    while (worker->status_ != VPX_WORKER_STATUS_OK) {
      pthread_cond_wait(&worker->impl_->condition_, &worker->impl_->mutex_);
    }
    if (new_status != VPX_WORKER_STATUS_OK) {
      worker->status_ = new_status;
      pthread_cond_signal(&worker->impl_->condition_);
    }
  }
  pthread_mutex_unlock(&worker->impl_->mutex_);
}
```

The function is the *only* place the main thread is allowed to touch
`status_`. The protocol it implements is:

1. **Bail-out for dead workers.** If `impl_` is `NULL`, the worker never
   came up (or has already been torn down). Re-checking `status_` first
   would be a data race, so the function exits before locking. This is
   the comment "No-op when attempting to change state on a thread that
   didn't come up."

2. **Drain any in-flight job.** While `status_ == VPX_WORKER_STATUS_WORKING`,
   wait. The worker thread is the only entity that may transition
   `WORKING → OK`, and it does so just before signalling the condition
   variable in `thread_loop`. The `while` loop is essential against
   spurious wakeups.

3. **Publish the new state.** If the caller asked for `OK` (the
   `sync()` case) we are already there — nothing more to do. Otherwise
   (`WORKING` for `launch()`, `NOT_OK` for `end()`), assign and signal.
   The worker thread is in `pthread_cond_wait` at this point; the
   signal moves it to whichever branch its outer loop selects.

Note the asymmetry: the worker thread's `while (status_ == OK)` loop in
`thread_loop` matches the main thread's `while (status_ != OK)` loop
here. They are mutually exclusive — exactly one party is asleep on the
condition variable at any time.

### `thread_loop` — the worker side of the rendezvous

`thread_loop` is the function passed to `pthread_create`. It runs
forever (or until `end()` is called) and structurally mirrors
`change_state`:

```c
pthread_mutex_lock(&worker->impl_->mutex_);
for (;;) {
  while (worker->status_ == VPX_WORKER_STATUS_OK) {
    pthread_cond_wait(&worker->impl_->condition_, &worker->impl_->mutex_);
  }
  if (worker->status_ == VPX_WORKER_STATUS_WORKING) {
    pthread_mutex_unlock(&worker->impl_->mutex_);
    execute(worker);
    pthread_mutex_lock(&worker->impl_->mutex_);
    assert(worker->status_ == VPX_WORKER_STATUS_WORKING);
    worker->status_ = VPX_WORKER_STATUS_OK;
    pthread_cond_signal(&worker->impl_->condition_);
  } else {
    assert(worker->status_ == VPX_WORKER_STATUS_NOT_OK);
    break;
  }
}
```

There are two subtleties worth dwelling on:

**Releasing the lock around `execute()`.** The hook is potentially
long-running (decoding a whole MB row), and holding the mutex across it
would block any concurrent `sync()` from the main thread until the work
is *already* done — defeating the point of waiting. The lock is
therefore dropped while the hook runs. The author's in-source comment
explains why this is safe:

> When `worker->status_` is `VPX_WORKER_STATUS_WORKING`, the main thread
> doesn't change `worker->status_` and will wait until the worker
> changes `worker->status_` to `VPX_WORKER_STATUS_OK`. See
> `change_state()`. So the worker can safely call `execute()` without
> holding `worker->impl_->mutex_`. When the worker reacquires
> `worker->impl_->mutex_`, `worker->status_` must still be
> `VPX_WORKER_STATUS_WORKING`.

The invariant being relied on is precisely the `while (status_ !=
VPX_WORKER_STATUS_OK)` wait in `change_state` — no one will mutate the
field while the worker holds it at `WORKING`. The `assert` after the
re-lock checks that the invariant held.

**Termination signal.** `end()` transitions the state straight from
`OK` to `NOT_OK`. When the worker wakes up, its outer `while` exits
(`status_ != OK`), the `if (status_ == WORKING)` is false, and the
`else` branch breaks out of the loop. `pthread_join` in `end()` then
completes.

### Optional thread naming

The first thirty lines of `thread_loop` are a `pthread_setname_np`
incantation that gives the OS thread a human-readable name for the
debugger if `worker->thread_name` is set. The platform variations are
illustrative of the portability minefield this whole file insulates the
rest of the code from:

* On Apple, `pthread_setname_np` takes a single string and renames the
  *current* thread; the buffer must be at most 63 bytes.
* On glibc (excluding GNU/Hurd, hence `!defined(__GNU__)`) and on
  Android (Bionic), the function takes `(pthread_t, const char *)` and
  the name plus its NUL must fit in 16 bytes — Linux's
  `comm` field is fixed-size.
* Everywhere else, the block is empty.

The `#define _GNU_SOURCE` at the very top of the file (placed before
*any* include, as the comment insists) is what makes glibc expose
`pthread_setname_np` from `<pthread.h>`.

---

## The vtable methods

The seven static functions below this point are exactly the entries of
the default `g_worker_interface` table. Each one is small and
straightforward in isolation; their value lies in how they compose into
the protocol above.

### `init` — zero the struct

```c
static void init(VPxWorker *const worker) {
  memset(worker, 0, sizeof(*worker));
  worker->status_ = VPX_WORKER_STATUS_NOT_OK;
}
```

The `memset` makes the worker safe to call `end()` on even if `reset()`
never succeeded (because `impl_` is now `NULL`). The explicit assignment
to `NOT_OK` is redundant given the enum's `0` value, but it documents
intent and is robust against future reordering of the enum.

### `reset` — allocate the impl and spawn the thread

The function distinguishes three input states:

* `status_ < OK` (i.e. `NOT_OK`, freshly initialised): allocate
  `impl_`, init the mutex and condvar, create the OS thread. On any
  failure, cascade the cleanup through the `Error:` label and return
  zero. The `pthread_mutex_lock` taken before `pthread_create` is
  deliberate: it ensures the worker thread cannot reach
  `pthread_cond_wait` before the main thread has finished assigning
  `status_ = VPX_WORKER_STATUS_OK`. Without that, the worker might
  observe `status_ == NOT_OK` on entry and immediately exit.
* `status_ == OK`: nothing to do.
* `status_ > OK` (i.e. `WORKING`, a reuse case): drain the prior job
  with `sync()`. After this branch the worker is guaranteed to be `OK`
  again, ready for a new `launch`.

The `had_error` field is cleared unconditionally at the top — `reset`
is the only operation that does this. The `assert(!ok || (status_ ==
OK))` at the bottom captures the post-condition: on success the worker
is exactly in the `OK` state.

In the `--disable-multithread` build the entire pthread block is
preprocessed out and the function reduces to:

```c
worker->status_ = VPX_WORKER_STATUS_OK;
```

with `impl_` left as `NULL`. The state machine still progresses, but
nothing happens off the calling thread.

### `execute` — run the hook in the current thread

```c
static void execute(VPxWorker *const worker) {
  if (worker->hook != NULL) {
    worker->had_error |= !worker->hook(worker->data1, worker->data2);
  }
}
```

This is the body of the worker. The hook is invoked with the two opaque
pointers the caller supplied. The return value is *negated* before being
ORed into `had_error`, because the hook contract is "true on success,
false on error" but `had_error` is "true if any error has occurred."
The null check exists so that an idle worker — one whose hook has been
cleared — can be safely launched without crashing.

`execute` is the only vtable entry that is called from *both* threads:
the worker thread invokes it from `thread_loop`, and the main thread
may invoke it directly when the caller wants to bypass the thread.
That dual-use is the reason it is a public method on the vtable rather
than a private helper.

### `launch` — kick the worker

```c
static void launch(VPxWorker *const worker) {
#if CONFIG_MULTITHREAD
  change_state(worker, VPX_WORKER_STATUS_WORKING);
#else
  execute(worker);
#endif
}
```

In the threaded build, transitioning to `WORKING` is the wake-up signal
the worker is waiting for. In the no-thread build, `launch()` is
literally `execute()` — the work happens synchronously on the caller's
thread. From the caller's perspective the two builds are
indistinguishable in correctness; only the latency differs.

### `sync` — wait for the worker to finish

```c
static int sync(VPxWorker *const worker) {
#if CONFIG_MULTITHREAD
  change_state(worker, VPX_WORKER_STATUS_OK);
#endif
  assert(worker->status_ <= VPX_WORKER_STATUS_OK);
  return !worker->had_error;
}
```

`sync` blocks until the worker has returned to `OK`. After the call the
caller is free to mutate `hook`, `data1`, `data2` again. The return
value is `1` if every hook since the last `reset` succeeded, `0`
otherwise — this is the only way the caller observes hook failures.
The asymmetry is intentional: `launch` is fire-and-forget, errors
materialise only at `sync` time.

In the no-thread build there is nothing to wait for — `launch()` ran
the hook synchronously already — so the function reduces to the
`return !had_error` line.

### `end` — terminate the worker

```c
static void end(VPxWorker *const worker) {
#if CONFIG_MULTITHREAD
  if (worker->impl_ != NULL) {
    change_state(worker, VPX_WORKER_STATUS_NOT_OK);
    pthread_join(worker->impl_->thread_, NULL);
    pthread_mutex_destroy(&worker->impl_->mutex_);
    pthread_cond_destroy(&worker->impl_->condition_);
    vpx_free(worker->impl_);
    worker->impl_ = NULL;
  }
#else
  worker->status_ = VPX_WORKER_STATUS_NOT_OK;
  assert(worker->impl_ == NULL);
#endif
  assert(worker->status_ == VPX_WORKER_STATUS_NOT_OK);
}
```

The order is fixed and necessary:

1. `change_state(..., NOT_OK)` wakes the worker; it observes the new
   state, breaks out of the `for(;;)` loop, and returns from
   `thread_loop`.
2. `pthread_join` collects the now-finished thread.
3. The mutex and condvar are destroyed only *after* the join, because
   destroying a condvar a thread is currently waiting on is undefined.
4. The impl block is freed and the pointer cleared so that a subsequent
   `end()` is a no-op (`if (impl_ != NULL)`).

The `impl_ == NULL` invariant at the end leaves the worker in a state
where `init()` could legally be called again to start the cycle over.

---

## The interface table

### `g_worker_interface` and the dispatch indirection

```c
static VPxWorkerInterface g_worker_interface = { init,   reset,   sync,
                                                 launch, execute, end };
```

This is the default vtable. Every libvpx call site fetches it through
`vpx_get_worker_interface()` rather than calling `init`, `reset`, … by
name, so an embedder can substitute its own dispatcher:

```c
int vpx_set_worker_interface(const VPxWorkerInterface *const winterface) {
  if (winterface == NULL || winterface->init == NULL ||
      winterface->reset == NULL || winterface->sync == NULL ||
      winterface->launch == NULL || winterface->execute == NULL ||
      winterface->end == NULL) {
    return 0;
  }
  g_worker_interface = *winterface;
  return 1;
}
```

The header warns that the setter "is not thread-safe" and must be called
before any worker exists. The reason is obvious in hindsight: the global
is read without a barrier from every codec instance, and replacing it
mid-decode would race against any in-flight `sync()` or `launch()`.

The contents of the supplied struct are *copied*, not referenced, so the
caller may free its prototype immediately. The all-non-NULL check is the
only validation performed — there is no way to provide partial
overrides.

`vpx_get_worker_interface()` is a one-line accessor:

```c
const VPxWorkerInterface *vpx_get_worker_interface(void) {
  return &g_worker_interface;
}
```

Decoder lifecycle code (e.g. `vp8_create_decoder_instances` in
`vp8/decoder/onyxd_if.c` and the threaded paths in
`vp8/decoder/threading.c`) fetches the pointer once, caches it in the
local context, and dispatches every `worker->reset()` / `launch()` /
`sync()` through it. In a `--disable-multithread` build the pointer is
still fetched but the operations through it never spawn a thread.

---

## Summary

`vpx_thread.c` is libvpx's answer to the question "how do you wrap
pthread without infecting the rest of the codebase with `<pthread.h>`?"
Its three principal abstractions — the `VPxWorker` struct, the
three-state lifecycle, and the `VPxWorkerInterface` vtable — together
allow:

* row-based multithreaded VP8 decode (the `VPxWorker` per token
  partition in `vp8/decoder/threading.c`),
* tile-row threading in VP9,
* a no-thread build that *uses the same call sites* but inlines the
  work, and
* host-supplied alternative threading back ends via
  `vpx_set_worker_interface`.

Reading the file in order, the WebP heritage is everywhere visible:
the same `THREADFN` typedef, the same `change_state` helper, even the
same locking discipline around `execute()`. What libvpx adds is the
`thread_name` field and its platform-specific
`pthread_setname_np` handling. The hot path — `launch` / hook / `sync`
— is a textbook condition-variable rendezvous, and the only
non-obvious invariant (releasing the mutex around the hook) is
documented inline. It is rare for a file this small to be quite so
load-bearing.
