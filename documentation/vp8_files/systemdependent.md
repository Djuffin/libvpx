# `vp8/common/generic/systemdependent.c` — machine-specific configuration hook

## Role in the decoder

In the libvpx tree this file lives at a curiously specific path:
`vp8/common/generic/systemdependent.c`. The directory name `generic/` is
the giveaway — historically VP8 expected one `systemdependent.c` per CPU
architecture (an `arm/`, `x86/`, `ppc/` variant), each filling in a
common entry point that performed (a) run-time CPU-feature detection,
and (b) installation of the architecture-appropriate SIMD function
pointers. Only the `generic/` variant survives today: every other
architecture in the tree (ARM, x86, PPC, MIPS, LoongArch) builds the
exact same file. There is no `vp8/common/arm/systemdependent.c`, no
`vp8/common/x86/systemdependent.c`. Verify:

```
$ find vp8 -name systemdependent.c
vp8/common/generic/systemdependent.c
```

What changed? CPU-feature detection and SIMD dispatch were factored out
of VP8's per-codec machinery into shared infrastructure:

  * **Feature detection** moved to `vpx_ports/{arm,aarch32,aarch64,x86,
    ppc,mips,loongarch}_cpudetect.c`, exposing `arm_cpu_caps()`,
    `x86_simd_caps()`, etc.
  * **Function-pointer installation** moved to a generated dispatcher,
    `setup_rtcd_internal()` (declared in the generated `vp8_rtcd.h`),
    invoked through the one-shot wrapper `vp8_rtcd()` in
    `vp8/common/rtcd.c`. RTCD = "run-time CPU dispatch."

After that migration, the only machine-dependent thing left for VP8
itself to do at `VP8_COMMON` construction time was to figure out **how
many CPU cores the host has**, so multi-threaded decode/encode can size
its worker pool sensibly. That is precisely — and only — what this file
does today. The "generic" path therefore turns out to be all the path
that VP8 needs, and the architecture branches in the `#include` block
near the top are vestigial.

The single entry point `vp8_machine_specific_config()` is called once
per `VP8_COMMON` instance, from `vp8_create_common()` in
`vp8/common/alloccommon.c`:

```c
void vp8_create_common(VP8_COMMON *oci) {
  vp8_machine_specific_config(oci);
  ...
```

In a decoder-only, single-threaded build (e.g. `--disable-multithread`),
the function compiles down to a no-op that just discards its argument;
the file is still pulled into the build because the call site is
unconditional and the declaration in `systemdependent.h` is part of the
common ABI.

This documentation walks the file top-to-bottom: the architecture
include block, the CPU-count helper, and the public entry point.

## The architecture-specific include block

```c
#include "vpx_config.h"
#include "vp8_rtcd.h"
#if VPX_ARCH_ARM
#include "vpx_ports/arm.h"
#elif VPX_ARCH_X86 || VPX_ARCH_X86_64
#include "vpx_ports/x86.h"
#elif VPX_ARCH_PPC
#include "vpx_ports/ppc.h"
#elif VPX_ARCH_MIPS
#include "vpx_ports/mips.h"
#elif VPX_ARCH_LOONGARCH
#include "vpx_ports/loongarch.h"
#endif
#include "vp8/common/onyxc_int.h"
#include "vp8/common/systemdependent.h"
```

The first two unconditional includes are functional: `vpx_config.h`
brings in the build-time feature macros (`CONFIG_MULTITHREAD`,
`HAVE_UNISTD_H`, the `VPX_ARCH_*` selectors), and `vp8_rtcd.h` is the
generated header that declares the dispatchable VP8 function pointers
plus `setup_rtcd_internal`. **Why include `vp8_rtcd.h` if this file
never calls `vp8_rtcd()`?** Because earlier revisions of the file did,
and removing the include is one of those small cleanups that is easy
to forget; the include is currently inert here.

The `#if VPX_ARCH_*` cascade is similarly vestigial. Each per-arch
header — e.g. `vpx_ports/x86.h` — exposes capability bit masks (`HAS_SSE2`,
`HAS_AVX2`, …) and the corresponding `*_simd_caps()` query. **Why pull
them in if we never reference any of those symbols in this translation
unit?** Two plausible reasons preserved by inertia: (1) callers used to
need symbols like `x86_simd_caps()` to populate a CPU-features field on
`VP8_COMMON`; (2) leaving the includes ensures the per-arch port header
is at least syntactically compiled on every build, catching breakage
early. Functionally, none of these headers is consulted by the code
below.

The trailing two includes are load-bearing: `onyxc_int.h` defines the
`VP8_COMMON` (a.k.a. `VP8Common`) struct so that the field
`processor_core_count` is in scope, and `systemdependent.h` provides
the prototype that this file implements.

### `#if CONFIG_MULTITHREAD` headers

```c
#if CONFIG_MULTITHREAD
#if HAVE_UNISTD_H
#include <unistd.h>
#elif defined(_WIN32)
#include <windows.h>
typedef void(WINAPI *PGNSI)(LPSYSTEM_INFO);
#endif
#endif
```

These platform headers are needed exclusively by `get_cpu_count()`
below, and they are pulled in only when multi-threading is enabled in
the build configuration. On POSIX hosts `unistd.h` declares `sysconf()`
and the `_SC_NPROCESSORS_ONLN` / `_SC_NPROC_ONLN` selectors. On Windows
`windows.h` declares `GetNativeSystemInfo()` and `SYSTEM_INFO`.

**The `PGNSI` typedef.** This is a leftover. Originally VP8 used
`GetProcAddress` to look up `GetNativeSystemInfo` at run time (it did
not exist on pre-XP Windows), and `PGNSI` was the function-pointer type
of the lookup result. The current code unconditionally calls
`GetNativeSystemInfo()` directly — see the `#if _WIN32_WINNT < 0x0501`
guard below — so `PGNSI` is now unused. The typedef remains for historic
reasons; the compiler does not warn about an unused `typedef`.

## `static int get_cpu_count(void)` — the core-count probe

```c
#if CONFIG_MULTITHREAD
static int get_cpu_count(void) {
  int core_count = 16;
  ...
  return core_count > 0 ? core_count : 1;
}
#endif
```

**What it does.** Returns the number of online logical CPUs on the
host, defaulting to 16 if no platform-specific probe is available, and
clamped to a minimum of 1.

**Why it exists.** VP8's worker-pool sizing reads `VP8Common::
processor_core_count` to decide how many decode/encode threads to spawn.
The decoder side does:

```c
/* vp8/decoder/threading.c */
if (core_count > pbi->common.processor_core_count) {
  core_count = pbi->common.processor_core_count;
}
```

and the encoder side uses it analogously in `ethreading.c`. Without an
accurate CPU count, the multi-threaded paths either over-subscribe
(spawning more workers than cores, causing contention) or starve.

**The default-of-16 fallback.** If both the POSIX and Win32 branches
are compiled out — i.e. some exotic platform where neither
`HAVE_UNISTD_H` nor `_WIN32` is defined and the `/* other platforms */`
comment is the only thing taken — `core_count` keeps its initial value
of 16. The choice of 16 over, say, 1 or 4 is a pragmatic guess: a
modest over-estimate for a server-class machine, but better than
disabling concurrency outright. The minimum-of-1 clamp at the return
ensures we never hand back a zero or negative count (defensive against
a misbehaving `sysconf()`).

**Invariants.**

  * Return value is always `>= 1`.
  * Function is **only defined** when `CONFIG_MULTITHREAD` is set; the
    function-pointer field it ultimately fills, `processor_core_count`,
    is similarly guarded in the struct definition (see `onyxc_int.h`).
    Pure single-threaded decoder builds compile this function out
    entirely.
  * No side effects beyond the syscall(s) it performs; safe to call
    from any thread (no global state).

**The POSIX branch.**

```c
#if HAVE_UNISTD_H
#if defined(_SC_NPROCESSORS_ONLN)
  core_count = (int)sysconf(_SC_NPROCESSORS_ONLN);
#elif defined(_SC_NPROC_ONLN)
  core_count = (int)sysconf(_SC_NPROC_ONLN);
#endif
```

`_SC_NPROCESSORS_ONLN` is the POSIX-2008 / Linux / *BSD / macOS spelling
of "logical CPUs currently online." `_SC_NPROC_ONLN` is the older IRIX
spelling, kept as a fallback for ancient Unixes. The cast to `int` is
necessary because `sysconf()` returns `long`; the result is
`processor_core_count`'s type, `int`, in the common struct.

**The Win32 branch.**

```c
#elif defined(_WIN32)
  {
#if _WIN32_WINNT < 0x0501
#error _WIN32_WINNT must target Windows XP or newer.
#endif
    SYSTEM_INFO sysinfo;
    GetNativeSystemInfo(&sysinfo);
    core_count = (int)sysinfo.dwNumberOfProcessors;
  }
```

The `_WIN32_WINNT` guard documents the minimum supported Windows
release: XP introduced `GetNativeSystemInfo()`, which (unlike the older
`GetSystemInfo()`) returns the native processor count even when the
caller is a 32-bit process running under WOW64 on a 64-bit host. The
`#error` directive makes the requirement a hard compile failure rather
than a confusing runtime symbol-resolution error. `dwNumberOfProcessors`
counts logical processors, matching the POSIX semantics.

## `void vp8_machine_specific_config(VP8_COMMON *ctx)` — public entry point

```c
void vp8_machine_specific_config(VP8_COMMON *ctx) {
#if CONFIG_MULTITHREAD
  ctx->processor_core_count = get_cpu_count();
#else
  (void)ctx;
#endif /* CONFIG_MULTITHREAD */
}
```

**What it does.** Populates the `processor_core_count` field on the
caller-supplied `VP8_COMMON`. That is the entirety of the body in a
multi-threaded build; in a single-threaded build the function is a
deliberate no-op (the `(void)ctx;` silences an "unused parameter"
warning).

**Why a function at all, rather than inlining the probe at the call
site?** Two reasons:

  1. **Architectural placeholder.** Even though the body is currently
     trivial, the function is the agreed extension point for any
     future per-host configuration that the VP8 codec instance might
     need to know — independent of (and complementary to) the global
     RTCD dispatch table. Keeping the indirection makes it cheap to
     add new fields later without touching every call site.
  2. **Build-time hiding.** The single-threaded path strips out
     `unistd.h`/`windows.h` and the `get_cpu_count()` helper
     completely, so callers do not have to wrap each invocation in
     `#if CONFIG_MULTITHREAD`.

**How it is used.** Called exactly once per codec instance, from
`vp8_create_common()` in `alloccommon.c`, which is itself invoked from
the encoder and decoder lifecycle entry points (`vp8_create_compressor`
on the encoder side; `vp8_create_decoder_instances` on the decoder
side). By the time the first frame is decoded or encoded,
`processor_core_count` is set and the threading layer can rely on it.

**Invariants and contract.**

  * `ctx` must be non-null and point to a valid (zero-initialised is
    fine) `VP8_COMMON`. The function does not allocate; it only writes.
  * Idempotent: calling twice on the same `VP8_COMMON` produces the
    same result.
  * Thread-safe in the sense that `get_cpu_count()` itself has no
    shared mutable state, but the typical pattern is single-threaded
    setup on the codec-creation thread before any workers are spawned.

**Where the dispatched SIMD setup actually happens.** Notably *not*
here. The companion file `vp8/common/rtcd.c` defines
`vp8_rtcd()`, a one-shot wrapper around the generated
`setup_rtcd_internal()` that installs the architecture-appropriate
function pointers (NEON, SSE2, AVX2, MSA, …). That call is made from
the codec-iface initialisation path (`vp8_dx_iface.c` /
`vp8_cx_iface.c`), independently of `vp8_machine_specific_config`. The
two used to be coupled; today they are not. This file is the residue
of that decoupling.
