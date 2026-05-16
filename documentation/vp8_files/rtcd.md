# `vp8/common/rtcd.c` — the once-only initialiser for run-time CPU dispatch

`rtcd.c` is the smallest non-trivial translation unit in the VP8
decoder. Counting blank lines and the boilerplate copyright header, it
is sixteen lines long; stripped of those it is exactly two lines of
code. Yet it is the single point at which the entire SIMD-acceleration
strategy of libvpx hangs together. Every kernel that has a NEON, SSE2,
SSSE3, AVX2, MSA, MMI or LSX implementation — the sixtap and bilinear
sub-pixel filters, the 4x4 IDCT, the loop filter inner kernels, the
copy_mem helpers, the dequant/IDCT fusion — is reached through a
function-pointer table that is populated exactly once, the first time a
VP8 codec instance is created, by the call this file makes.

```c
#include "./vpx_config.h"
#define RTCD_C
#include "./vp8_rtcd.h"
#include "vpx_ports/vpx_once.h"

void vp8_rtcd(void) { once(setup_rtcd_internal); }
```

There is no other code in the file. What is interesting is everything
that does *not* appear here but is nevertheless brought into existence
by this one source line — a textbook case of work pushed entirely into
the build system and the preprocessor.

## Role in the decoder

`RTCD` stands for *Run-Time CPU Dispatch*. The acronym is libvpx's own,
and it names the project's solution to a recurring problem: a single
shipped binary must run on a Pentium III (SSE) and on a Tiger Lake
(AVX2); on an ARMv7-a phone (NEON optional) and on an Apple M-series
core (NEON mandatory). The strategy is the well-known one of
function-pointer-per-kernel:

  * For every accelerable primitive `foo`, libvpx declares the C
    reference implementation `foo_c` and zero or more architecture
    specialisations `foo_sse2`, `foo_neon`, `foo_msa`, …
  * It declares a public symbol `foo` that is either a `#define` alias
    to the C version (if no faster version is possible on this build's
    target) or a function pointer of the matching signature (if at
    least one specialisation exists).
  * On first use, the program asks the OS / hardware which extensions
    are present (`cpuid` on x86, `getauxval(AT_HWCAP)` on Linux/ARM,
    `IsProcessorFeaturePresent` on Win32, etc.) and assigns each
    pointer to the best available implementation.

`rtcd.c` is the *VP8-side* initialiser for that last step. It exists in
identical form in two sister files — `vpx_dsp/vpx_dsp_rtcd.c` for the
DSP layer and `vpx_scale/vpx_scale_rtcd.c` for the YV12 scaler — and
each of those, like this one, is reduced to a single statement that
delegates almost everything to a generated header.

### Where the table itself lives

The table is `vp8_rtcd.h`, produced at configure/build time by
`build/make/rtcd.pl` from the per-codec DSL file
`vp8/common/rtcd_defs.pl`. The decoder build sees something like
`vp8_only/vp8_rtcd.h` or `build_debug/vp8_rtcd.h`, depending on the
out-of-tree build dir; both are structurally identical:

```c
// vp8_rtcd.h, abridged
#ifdef RTCD_C
#define RTCD_EXTERN
#else
#define RTCD_EXTERN extern
#endif

void vp8_short_idct4x4llm_c(short *input, unsigned char *pred_ptr,
                            int pred_stride, unsigned char *dst_ptr,
                            int dst_stride);
#define vp8_short_idct4x4llm vp8_short_idct4x4llm_c

/* ... ~30 more prototypes ... */

void vp8_rtcd(void);

#ifdef RTCD_C
static void setup_rtcd_internal(void)
{
}
#endif
```

Two things to note about that excerpt. First, on this particular target
(`generic-gnu`, the verified single-arch decoder build) every kernel
collapses to `#define foo vp8_short_…_c` — there is no function pointer
because there is no second implementation to choose between, and the
preprocessor resolves the dispatch at compile time. Second,
`setup_rtcd_internal` is *defined inside the header* but guarded by
`#ifdef RTCD_C`. That is the only definition site for the symbol; it
exists as a `static` function in whatever .c file defines `RTCD_C`
before including the header. There is exactly one such file in the
decoder build: `rtcd.c`, by means of the `#define RTCD_C` on its line
11. That is what makes line 11 load-bearing despite looking like a
stylistic flourish.

On a build with SIMD enabled (say `x86_64-linux-gcc`), the generated
header instead contains, for each accelerable kernel, blocks shaped
like:

```c
void vp8_short_idct4x4llm_c(...);
void vp8_short_idct4x4llm_sse2(...);
RTCD_EXTERN void (*vp8_short_idct4x4llm)(short *, unsigned char *, int,
                                         unsigned char *, int);

static void setup_rtcd_internal(void)
{
    int flags = x86_simd_caps();
    vp8_short_idct4x4llm = vp8_short_idct4x4llm_c;
    if (flags & HAS_SSE2) vp8_short_idct4x4llm = vp8_short_idct4x4llm_sse2;
    /* ... one such block per kernel ... */
}
```

The pointer is `RTCD_EXTERN` — i.e. `extern` everywhere *except* in
`rtcd.c` (where `RTCD_C` is defined and `RTCD_EXTERN` collapses to
nothing), giving the pointer exactly one definition in the program.
That is the same trick used by the GNU `errno.h` family and by most
header-as-table libraries; libvpx's contribution is to generate it from
a single DSL.

### Why "once" matters

`vp8_rtcd()` is called by the codec instance lifecycle (`vp8_create_compressor`,
`vp8_dx_create_compressor`, and analogues elsewhere) at the *start* of
every new encoder/decoder instance. Without protection, two threads
spinning up a decoder simultaneously would each race to overwrite the
same pointer table — harmless on architectures where pointer stores
are atomic, but undefined behaviour in C and visibly racy under
sanitizers. The `once()` helper from `vpx_ports/vpx_once.h` makes the
first call winner-takes-all and blocks all subsequent callers until the
table is populated. After that the call is, by intent, free: callers
need not even check whether initialisation has happened.

## The single function: `vp8_rtcd`

```c
void vp8_rtcd(void) { once(setup_rtcd_internal); }
```

### What it is

The public entry point of the VP8 RTCD subsystem. Its prototype is
emitted by the generated header (`vp8_rtcd.h:121`), so any compilation
unit that includes `./vp8_rtcd.h` can call it; its definition lives
here, and only here, in the whole codec.

### What it does

It delegates to the inline helper `once`, which guarantees that
`setup_rtcd_internal` is called exactly one time in the program's
lifetime, regardless of how many threads enter `vp8_rtcd()` or how many
times each thread enters it. After the first call returns,
`setup_rtcd_internal` has finished executing on *some* thread and the
function-pointer table is fully initialised.

### Why this shape

A reader might reasonably ask: why not put the body of
`setup_rtcd_internal` directly into `vp8_rtcd()` and drop the helper?
There are three reasons, in increasing order of importance:

  1. `setup_rtcd_internal` is *generated* into the header. The whole
     reason this .c file exists is to provide the one translation unit
     into which the header's static definition gets emitted. Putting
     anything substantive into `rtcd.c` would force a copy of that
     generated boilerplate into the source tree, defeating the
     code-generation strategy.

  2. The `once()` wrapper has to *enclose* the call site, because the
     synchronization primitive (whether `pthread_once`, the Win32
     `InterlockedCompareExchange` loop, or the single-threaded
     `static volatile int done` flag) is selected by
     `vpx_once.h:40-114` based on `CONFIG_MULTITHREAD` and the host's
     thread primitives. Keeping that selection in `vpx_once.h` and
     keeping `vp8_rtcd()` blissfully ignorant of it is what allows
     libvpx to be built for environments as different as Windows, glibc
     Linux, musl Linux, Apple, Android (bionic) and bare-metal SDKs
     without touching a line of `rtcd.c`.

  3. `once()` is a *file-static* function (it is `static void once(...)`
     in `vpx_once.h:51, 96, 107`). Each translation unit that includes
     `vpx_once.h` gets its own copy of the lock state. That is exactly
     what is wanted here: `vp8_rtcd.c`'s `once()` has its own lock,
     `vpx_dsp_rtcd.c`'s `once()` has another, `vpx_scale_rtcd.c`'s has a
     third. None can starve another. The header comment makes the
     restriction explicit (`vpx_once.h:16-37`): "These functions use
     static locks, and can only be used with one common argument per
     compilation unit." `rtcd.c` obeys the rule by calling `once` with
     exactly one argument and never doing anything else.

### Invariants

  * `vp8_rtcd()` is **safe to call from any thread at any time** after
    the program has started. The synchronization within `once()` makes
    the first-call/subsequent-call distinction transparent.
  * `vp8_rtcd()` is **idempotent**. The C-only fallback in
    `vpx_once.h:101-115` notes this explicitly: "*_rtcd() is
    idempotent, so as long as your platform provides atomic
    loads/stores of pointers no synchronization is strictly
    necessary*". The fallback is therefore not even truly
    "synchronised" — it relies on the fact that `setup_rtcd_internal`
    only ever writes the same pointer values to the same locations, so
    a hypothetical second initialiser running concurrently with the
    first would not be observably different from a single
    initialisation.
  * `vp8_rtcd()` **must be called before any of the dispatched kernels
    are invoked**. On builds with no specialisations (i.e. when every
    `vp8_foo` is a `#define` alias to `vp8_foo_c`), this is trivially
    satisfied; on SIMD builds, calling `vp8_short_idct4x4llm(...)`
    before `vp8_rtcd()` would dereference an uninitialised function
    pointer. The codec creation paths uphold this invariant by calling
    `vp8_rtcd()` from `vp8_create_common` / `vp8_create_compressor` and
    their decoder analogues.
  * It **must not be called from a signal handler**: `pthread_once` is
    async-signal-safe per POSIX, but the Win32 implementation uses
    `Sleep(0)` to yield (`vpx_once.h:82`), and that is not. The codec
    APIs are not signal-safe either, so this constraint is normally
    invisible.

### How it is used

The caller in the decoder is, per `vp8/common/generic/systemdependent.c`
and the codec-create paths in `vp8_dx_iface.c` / `onyxd_if.c`, simply:

```c
vp8_rtcd();
vpx_dsp_rtcd();
vpx_scale_rtcd();
```

These three calls together populate every function-pointer table the
VP8 decoder will ever consult. After they return, the IDCT, sub-pixel
filter, loop filter, copy-mem, intra-predictor, bitreader-MULx and YV12
border-extension primitives are all bound to their best available
implementations for the host. The dispatch cost from that moment on is
a single indirect call per kernel invocation — on a current x86 with
correctly-predicted indirect branches, statistically free.

## How `setup_rtcd_internal` interacts with `vpx_once` and `vp8_rtcd.h`

Putting the pieces together, the request "decode a VP8 frame for the
first time" causes the following chain of events:

  1. The caller (`vpx_codec_dec_init_ver` -> `vp8_init_decoder` ->
     ultimately `vp8_create_common`) invokes `vp8_rtcd()`.
  2. `vp8_rtcd()` calls `once(setup_rtcd_internal)`. The function
     pointer is resolved at link time to the `static` `once` in
     `rtcd.c`'s translation unit — which is one specific copy with one
     specific lock state.
  3. On the first such call ever, `once()` acquires its lock (or wins
     the `InterlockedCompareExchange`, or sets `done = 1`, depending on
     the build) and invokes `setup_rtcd_internal`.
  4. `setup_rtcd_internal` is the symbol that the generated
     `vp8_rtcd.h` defined under `#ifdef RTCD_C`. Because `rtcd.c` is
     the only translation unit with `RTCD_C` defined, this is the
     *only* place in the program where `setup_rtcd_internal` exists as
     a defined symbol. (If a second .c file ever defined `RTCD_C`, the
     linker would either succeed silently — `setup_rtcd_internal` is
     `static`, after all — or, more annoyingly, two pointer-table
     definitions would clash on the non-static `RTCD_EXTERN` lines.
     Both outcomes are bug-shaped; the convention is that exactly one
     .c file per RTCD table defines `RTCD_C`, and that file does
     nothing else.)
  5. `setup_rtcd_internal` runs through the generated assignment
     blocks. On the verified `generic-gnu` decoder build, the body is
     empty (`vp8_rtcd.h:127-128`); on a SIMD build it queries
     `x86_simd_caps()` / `arm_cpu_caps()` / `mips_cpu_caps()` /
     `ppc_simd_caps()` once each and writes the chosen pointers.
  6. `once()` returns; `vp8_rtcd()` returns; the caller continues with
     decoder setup. On every subsequent call to `vp8_rtcd()` — whether
     from this thread, a sibling, or a totally unrelated decoder
     instance created hours later — `once()` short-circuits and
     returns immediately.

### Why `RTCD_C` is `#define`d *before* the include

```c
#define RTCD_C
#include "./vp8_rtcd.h"
```

The order matters. `vp8_rtcd.h` checks for `RTCD_C` to decide whether
to (a) `extern` the pointer table or define it, and (b) emit
`setup_rtcd_internal` as a static definition or omit it. If `RTCD_C`
were defined *after* the include, both decisions would resolve the
wrong way and `setup_rtcd_internal` would be referenced from
`vp8_rtcd()` but never defined — a link-time error. The single line
`#define RTCD_C` therefore performs the central act of this file:
designating *this* compilation unit as the home of the pointer table
and its initialiser.

### Why the include is `"./vp8_rtcd.h"` not `"vp8_rtcd.h"`

The leading `./` forces the compiler's "current directory of the
including file" rule to be used unambiguously. In libvpx,
`vp8_rtcd.h` is *generated* into the build directory, not committed to
the source tree, and the build system arranges to set the include path
so that "current directory" resolves to that build directory. The
explicit `./` prefix is the libvpx convention for "this header is
generated next to the .c file"; it is used identically in
`vpx_dsp_rtcd.c` and `vpx_scale_rtcd.c`, and matches the include style
in every consumer of these tables across the tree.

### What the include of `vpx_config.h` accomplishes here

`vpx_config.h` is itself a generated header that records every
`CONFIG_*` / `HAVE_*` flag selected at configure time. `rtcd.c` does
not reference any of those flags directly, but `vp8_rtcd.h` does (the
generated `setup_rtcd_internal` is written in terms of `HAVE_SSE2`,
`HAVE_NEON`, etc.), and so does `vpx_once.h` (`CONFIG_MULTITHREAD`,
`HAVE_PTHREAD_H`). Including `vpx_config.h` first guarantees those
macros are in scope before either of the other two headers gets a
chance to look at them — the same "config first" preamble pattern used
in nearly every .c file in libvpx.

## A note on the table's *contents*

For the VP8 decoder specifically, the pointer-dispatched kernels are
the bandwidth-critical inner loops of pixel reconstruction. The
verified single-arch decoder list (from `vp8_only/vp8_rtcd.h`) is:

  * **Inter-prediction sub-pixel filters** — `vp8_sixtap_predict16x16`,
    `…8x8`, `…8x4`, `…4x4` and the matching `vp8_bilinear_predict*`
    quartet. Eight primitives.
  * **Block copy helpers** — `vp8_copy_mem16x16`, `…8x8`, `…8x4`.
    Three primitives.
  * **Dequant + IDCT fusion** — `vp8_dequant_idct_add`,
    `vp8_dequant_idct_add_y_block`, `vp8_dequant_idct_add_uv_block`,
    `vp8_dc_only_idct_add`, `vp8_dequantize_b`. Five primitives.
  * **IDCT and Walsh** — `vp8_short_idct4x4llm`,
    `vp8_short_inv_walsh4x4`, `vp8_short_inv_walsh4x4_1`. Three
    primitives.
  * **Loop filter** — `vp8_loop_filter_bh/bv/mbh/mbv` (normal),
    `vp8_loop_filter_simple_bh/bv/mbh/mbv` (simple). Eight primitives.

That is roughly two-and-a-half dozen dispatched functions for the
decoder alone; the encoder build adds another two dozen (`…walsh4x4`,
`temporal_filter_apply`, the encoder-specific variance/SAD tables that
in fact come from `vpx_dsp_rtcd.h`, and so on). On a tuned SIMD build,
every one of those becomes a `cpuid`-selected pointer; on the verified
`generic-gnu` build, every one collapses to a direct `#define` and
`setup_rtcd_internal` has nothing to do. In both extremes, the price
paid at the call site of `vp8_rtcd()` is the same: one synchronised
function call, exactly once per process. That is the entire point.
