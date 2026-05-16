# `vpx_dsp/vpx_dsp_rtcd.c` — the DSP-layer once-only RTCD initialiser

## Role in the decoder

If `vp8/common/rtcd.c` is the load-bearing one-liner for the *VP8*
function-pointer table, then `vpx_dsp/vpx_dsp_rtcd.c` is the load-bearing
one-liner for the *cross-codec* function-pointer table that VP8 and VP9
share. The two files are siblings — character-for-character almost
identical — and exist for the same reason: each provides the single
translation unit into which a generated header is allowed to materialise
the static body of its `setup_rtcd_internal()` function. Three such
files exist in the tree, one per RTCD table:

```
vp8/common/rtcd.c          -> setup_rtcd_internal for vp8_rtcd.h
vpx_dsp/vpx_dsp_rtcd.c     -> setup_rtcd_internal for vpx_dsp_rtcd.h   (this file)
vpx_scale/vpx_scale_rtcd.c -> setup_rtcd_internal for vpx_scale_rtcd.h
```

The whole file, including copyright, is sixteen lines; stripped it is
five. Reproduced verbatim:

```c
#include "./vpx_config.h"
#define RTCD_C
#include "./vpx_dsp_rtcd.h"
#include "vpx_ports/vpx_once.h"

void vpx_dsp_rtcd(void) { once(setup_rtcd_internal); }
```

`RTCD` is libvpx's term for *Run-Time CPU Dispatch*: at process start,
the library asks the host what SIMD extensions it has (cpuid, getauxval,
IsProcessorFeaturePresent, …) and writes the best available
implementation of each accelerable kernel into a global pointer table.
Subsequent calls into the DSP layer dereference that table. This file's
contribution is to (a) own the only definition of that table for the
DSP layer, and (b) ensure the population routine runs exactly once,
even if multiple threads race to create the first codec instance.

### The `RTCD_EXTERN` / `RTCD_C` / `setup_rtcd` convention

The generated `vpx_dsp_rtcd.h` uses a small preprocessor protocol to
arrange that exactly one translation unit owns each pointer in the
dispatch table. Lifting the relevant excerpt from the verified
single-arch generated header (`vp8_only/vpx_dsp_rtcd.h:14-19,206-214`):

```c
#ifdef RTCD_C
#define RTCD_EXTERN
#else
#define RTCD_EXTERN extern
#endif

/* ... ~200 prototype lines ... */

void vpx_dsp_rtcd(void);

#include "vpx_config.h"

#ifdef RTCD_C
static void setup_rtcd_internal(void)
{
}
#endif
```

The mechanism has three moving parts and they all interlock:

  * `RTCD_EXTERN` is the macro that decorates each pointer in the table.
    In all translation units *except* the one that defines `RTCD_C` it
    expands to `extern`, leaving the symbol as a mere declaration. In
    the one translation unit that *does* define `RTCD_C` (this file) it
    expands to nothing, giving the symbol a single tentative definition
    — exactly what the C linker needs.
  * `RTCD_C` is the per-table marker that picks the owner translation
    unit. The DSP table's owner is `vpx_dsp/vpx_dsp_rtcd.c`. The VP8
    table's owner is `vp8/common/rtcd.c`. The scale table's owner is
    `vpx_scale/vpx_scale_rtcd.c`. Each defines `RTCD_C` *before* it
    includes its corresponding generated header — see the body of this
    file, line 11 / line 12.
  * `setup_rtcd_internal` is the per-table population routine that the
    generated header emits as a `static` function under
    `#ifdef RTCD_C`. The `static` qualifier means each owner translation
    unit gets its own private copy with its own private name-scope; the
    `#ifdef RTCD_C` guard means *only* the owner gets a body, so the
    symbol cannot accidentally exist in two places.

On the verified single-arch decoder build (`generic-gnu`, no SIMD), the
body of `setup_rtcd_internal()` is empty: every dispatchable kernel in
the header collapses to a `#define` alias of its `_c` implementation,
so there is nothing to wire up at runtime. An excerpt from the
generated `vpx_dsp_rtcd.h` shows the degenerate case
(`vp8_only/vpx_dsp_rtcd.h:38-39`):

```c
void vpx_d117_predictor_16x16_c(uint8_t *dst, ptrdiff_t stride,
                                const uint8_t *above, const uint8_t *left);
#define vpx_d117_predictor_16x16 vpx_d117_predictor_16x16_c
```

On a SIMD-enabled build (say `x86_64-linux-gcc`), the same kernel
appears as an `RTCD_EXTERN` pointer, and the `setup_rtcd_internal`
body queries `x86_simd_caps()` once and writes the chosen
implementation into the pointer. The shape is exactly that documented
for `vp8_rtcd.c`; the DSP variant simply has a much wider table
(intra-predictors, inverse transforms, convolve filters, SAD/variance
for the encoder, …).

### Why `vpx_once` and not the body inlined

The body of `vpx_dsp_rtcd()` could in principle be

```c
void vpx_dsp_rtcd(void) { setup_rtcd_internal(); }
```

— and on a single-threaded build with idempotent population it would be
correct. It is not written that way because libvpx wants the same
source line to compile correctly on Windows (`InterlockedCompareExchange`
loop), POSIX (`pthread_once`), and bare-metal (a `static volatile int`
flag), and to do so *without* this file ever mentioning a synchronisation
primitive. The selection of which `once` implementation to use is made
inside `vpx_ports/vpx_once.h:40-115`:

```c
#if CONFIG_MULTITHREAD && defined(_WIN32)
  /* InterlockedCompareExchange version */
#elif CONFIG_MULTITHREAD && HAVE_PTHREAD_H
  /* pthread_once version */
#else
  /* no-op static-flag version */
#endif
```

Three properties of `once()` deserve to be highlighted because they
shape why this file looks the way it does:

  1. **It is `static`.** Each translation unit that includes
     `vpx_once.h` gets its own private `once` with its own private lock
     state. The header comment makes this explicit
     (`vpx_ports/vpx_once.h:16-37`): "*These functions use static locks,
     and can only be used with one common argument per compilation unit.*"
     `vpx_dsp_rtcd.c` obeys the rule by calling `once` with exactly one
     argument (`setup_rtcd_internal`) and never doing anything else in
     the file. `vp8/common/rtcd.c` has its own copy of `once` with its
     own lock and its own one argument. Likewise `vpx_scale_rtcd.c`.
     None of the three can starve another.
  2. **The no-op fallback relies on RTCD idempotency.** The header
     comment is worth quoting (`vpx_ports/vpx_once.h:101-103`): "*_rtcd()
     is idempotent, so as long as your platform provides atomic
     loads/stores of pointers no synchronization is strictly necessary.*"
     That is to say: even if two threads raced through the no-op
     `once()` and both ran `setup_rtcd_internal`, both would write the
     same pointer values to the same locations, and the table would end
     up correct. The synchronisation is a courtesy to sanitizers and to
     formal C semantics, not a functional necessity.
  3. **It blocks subsequent callers until the first call completes.**
     On Win32 by a `Sleep(0)` busy-wait on the state variable; on POSIX
     by `pthread_once`'s built-in wait. After `vpx_dsp_rtcd()` returns
     to *any* caller, the DSP table is fully populated and visible — no
     caller needs to re-check or re-acquire anything.

## The single function: `vpx_dsp_rtcd`

```c
void vpx_dsp_rtcd(void) { once(setup_rtcd_internal); }
```

### What it is

The public entry point of the cross-codec DSP RTCD subsystem. Its
prototype is emitted by the generated header
(`vpx_dsp_rtcd.h`, around line 206 in the verified build), so any
translation unit that includes `vpx_dsp_rtcd.h` can call it; its
definition lives here and only here in the whole library.

### What it does

It delegates to `once`, which guarantees that `setup_rtcd_internal` is
called exactly one time over the program's lifetime, regardless of how
many threads enter `vpx_dsp_rtcd()` and regardless of how many times
each thread enters it. After the first call returns, all of the DSP
pointers — intra-predictors of every block size and direction, inverse
transforms, convolve filters, SAD, variance, postproc helpers,
quantize/dequantize, and so on — are bound to their best available
implementations for the host.

### Why this shape

The reasoning mirrors the one in `vp8/common/rtcd.c` and is worth
re-stating because it is the entire point of why this file exists at
all:

  1. **`setup_rtcd_internal` is generated into the header, not the .c
     file.** The whole reason this .c file exists is to provide the one
     translation unit into which the header's `#ifdef RTCD_C`-guarded
     definition gets emitted. Inlining the body into `vpx_dsp_rtcd.c`
     would force a copy of the generated boilerplate into the source
     tree, defeating the code-generation strategy that lets libvpx
     describe its dispatch table once (in `vpx_dsp/vpx_dsp_rtcd_defs.pl`)
     and have it instantiated correctly for every target.
  2. **`once` must enclose the call site.** The synchronisation
     primitive selection lives in `vpx_ports/vpx_once.h` and depends on
     `CONFIG_MULTITHREAD` and the host's thread library. Keeping that
     selection in the header and keeping `vpx_dsp_rtcd()` ignorant of it
     is what allows libvpx to compile unchanged for glibc Linux, musl
     Linux, Apple, Android (bionic), Windows, and bare-metal SDKs.
  3. **The per-file `once` has its own lock.** Already discussed above
     in "Why `vpx_once` and not the body inlined" — the key fact is
     that `vpx_dsp_rtcd.c` gets its own `once` separate from the one in
     `vp8/common/rtcd.c`, so the three RTCD tables can all be
     initialised independently.

### Invariants

  * **Safe to call from any thread at any time after `main()` (or the
    DLL load-time initialisers) begins.** The synchronisation inside
    `once()` makes the first-call / subsequent-call distinction
    transparent to callers.
  * **Idempotent.** Multiple calls have the same effect as one. The
    no-op fallback in `vpx_once.h:107-114` makes this assumption
    explicit; both real implementations preserve the property by
    short-circuiting subsequent calls.
  * **Must be called before any of the dispatched DSP kernels are
    invoked.** On builds with no specialisations (every `vpx_foo` is a
    `#define` alias to `vpx_foo_c`), this is trivially satisfied; on
    SIMD builds, calling, say, `vpx_d117_predictor_16x16(...)` before
    `vpx_dsp_rtcd()` would dereference an uninitialised function
    pointer.
  * **Must not be called from a signal handler.** `pthread_once` is
    async-signal-safe per POSIX, but the Win32 implementation uses
    `Sleep(0)` to yield, which is not. The codec APIs that ultimately
    call this function are not signal-safe either, so the constraint
    is normally invisible.
  * **No globals are read other than via `once`'s internal state.**
    `vpx_dsp_rtcd()` itself reads nothing, writes nothing (directly),
    and returns nothing. All state mutation happens inside
    `setup_rtcd_internal`, and that state — the DSP function-pointer
    table — is initialised by `setup_rtcd_internal` and never written
    again.

### How it is used

The DSP RTCD initialiser is called by every codec-instance creation
path that might subsequently call into the DSP layer. From a search of
the tree (`grep -n "vpx_dsp_rtcd(" vp8 vpx vpx_dsp`):

```
vp8/vp8_dx_iface.c:93        vpx_dsp_rtcd();
vp8/vp8_cx_iface.c:698       vpx_dsp_rtcd();
vp8/decoder/onyxd_if.c:52    vpx_dsp_rtcd();
vp8/encoder/onyx_if.c:411    vpx_dsp_rtcd();
```

(VP9 has equivalent call sites; only the VP8 ones are listed because
this documentation is for the VP8 decoder.) Every entry point that can
plausibly be the first to touch the DSP table makes the call. Because
`once` short-circuits after the first successful invocation, the
redundancy is free — the codec lifecycle paths do not need to track
which of them was first, and a future entry point can be added without
threading through any flag.

In a typical decoder run, the call chain that triggers the *first*
invocation is:

```
vpx_codec_dec_init_ver         (vpx/src/vpx_decoder.c)
  -> vp8_init                  (vp8/vp8_dx_iface.c:93)
       vpx_dsp_rtcd()          <-- first call; once() winning thread runs setup_rtcd_internal
```

The DSP table is now live for the rest of the process. Every
subsequent codec-create call (decoder or encoder, VP8 or VP9) will hit
the short-circuit path in `once()` and return immediately.

## How `setup_rtcd_internal` interacts with `vpx_once` and `vpx_dsp_rtcd.h`

Putting the four lines back together with the rest of the system, the
first call to `vpx_dsp_rtcd()` triggers this chain:

  1. Some caller — typically `vp8_init` in `vp8_dx_iface.c` for the
     decoder — calls `vpx_dsp_rtcd()`.
  2. `vpx_dsp_rtcd()` calls `once(setup_rtcd_internal)`. The function
     `once` is resolved at *link* time to the `static` copy that lives
     in this translation unit; the function pointer
     `setup_rtcd_internal` is resolved at *compile* time to the
     `static` definition emitted by the included `vpx_dsp_rtcd.h` (line
     11 of this file, the `#define RTCD_C`, is what causes it to exist
     at all).
  3. On the first such call ever made by the process, `once()` wins
     its synchronisation primitive (pthread_once / Win32
     InterlockedCompareExchange loop / static-flag set) and invokes
     `setup_rtcd_internal`.
  4. `setup_rtcd_internal` runs through its generated assignment
     blocks. On the verified `generic-gnu` decoder build the body is
     empty; on a SIMD build it queries `x86_simd_caps()` /
     `arm_cpu_caps()` / `mips_cpu_caps()` / `ppc_simd_caps()` once each
     and assigns the chosen pointers into the `RTCD_EXTERN` variables
     declared in the header.
  5. `once()` returns; `vpx_dsp_rtcd()` returns; the caller continues
     with whatever it was doing. On every subsequent call to
     `vpx_dsp_rtcd()` — from this thread, a sibling, or a totally
     unrelated codec instance created hours later — `once()`
     short-circuits and returns immediately.

### Why `RTCD_C` is `#define`d *before* the include

```c
#define RTCD_C
#include "./vpx_dsp_rtcd.h"
```

The order is load-bearing. `vpx_dsp_rtcd.h` checks `RTCD_C` to decide
two things: (a) whether `RTCD_EXTERN` expands to `extern` or to
nothing — i.e. whether the pointer table is *declared* or *defined* in
this translation unit; and (b) whether `setup_rtcd_internal` is emitted
as a `static` definition or omitted entirely. If `RTCD_C` were defined
*after* the include, both decisions would resolve the wrong way and
`setup_rtcd_internal` would be referenced from `vpx_dsp_rtcd()` but
never defined — a link-time undefined reference. The single line
`#define RTCD_C` therefore performs the central act of this file:
designating *this* compilation unit as the home of the DSP pointer
table and its initialiser.

### Why the include is `"./vpx_dsp_rtcd.h"` not `"vpx_dsp_rtcd.h"`

The leading `./` forces the compiler's "current directory of the
including file" lookup rule. The header is *generated* into the build
directory (`vp8_only/vpx_dsp_rtcd.h`, `build_debug/vpx_dsp_rtcd.h`,
…), not committed to the source tree, and the build system arranges
for the build directory to be on the include search path such that the
`./` prefix resolves to it. The convention is shared verbatim with
`vp8/common/rtcd.c` and `vpx_scale/vpx_scale_rtcd.c` and is the libvpx
shorthand for "this header is generated next to the .c file."

### What the include of `vpx_config.h` accomplishes here

`vpx_config.h` is itself a generated header that records every
`CONFIG_*` and `HAVE_*` flag selected at configure time. This file
references none of those flags directly, but `vpx_dsp_rtcd.h` does
(its forward declarations are guarded by `CONFIG_VP9_ENCODER` — see
`vp8_only/vpx_dsp_rtcd.h:28-31` — and its `setup_rtcd_internal` body,
on SIMD builds, is written in terms of `HAVE_SSE2`, `HAVE_NEON`, …),
and so does `vpx_once.h` (`CONFIG_MULTITHREAD`, `HAVE_PTHREAD_H`).
Including `vpx_config.h` first guarantees those macros are in scope
before either of the other two headers gets a chance to look at them.
This is the "config first" preamble pattern used in nearly every .c
file in libvpx.

## A note on the table's contents

The DSP RTCD table is much wider than the VP8 one. The generated
header for the verified single-arch decoder build is 220 lines and
contains roughly 90 prototypes; the encoder build is a few hundred
prototypes wider still. For the VP8 *decoder* specifically, the
contents that matter most are:

  * **Intra-predictors** — the full set of directional and non-
    directional predictors at sizes 4x4, 8x8, 16x16, 32x32. These are
    the kernels implemented in `vpx_dsp/intrapred.c` and dispatched
    here. VP8's decoder uses the 4x4 and 16x16 subsets exclusively
    (8x8 and 32x32 are VP9's).
  * **Forward declarations for VP9 transforms** that the build still
    compiles in even on a VP8-only configuration, but that the VP8
    decoder never calls. These cost nothing at link time (every
    `_c`-only entry is a `#define` alias, not a pointer) and exist
    purely because the DSP layer is intentionally cross-codec.

On a tuned SIMD build, each prototype becomes a `cpuid`-selected
pointer assigned by `setup_rtcd_internal`. On the verified
`generic-gnu` build, every one collapses to a direct `#define` and
`setup_rtcd_internal` has nothing to do. In both extremes, the price
paid at the call site of `vpx_dsp_rtcd()` is the same: one
synchronised function call, exactly once per process. That is the
entire reason this five-line file exists.
