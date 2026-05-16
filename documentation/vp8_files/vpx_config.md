# `vp8_only/vpx_config.c` — the build-configuration witness

Of all the files compiled into a libvpx VP8 decoder, `vpx_config.c` is
the only one whose entire contents are dictated by the command line the
user typed at configure time, and the only one whose payload is a single
human-readable string. It carries no functions of consequence, no data
structures, no algorithms. It exists so that a running program — or
a support engineer staring at a crash report — can ask the library a
single question: "How were you built?"

Its companion `vpx_config.h` answers a different but related question
("Which features are on?") in machine-readable form via a hundred-odd
`#define`s. Between them, the two files constitute libvpx's complete
self-description.

```c
/* Copyright … (BSD-style license header) … */
#include "vpx/vpx_codec.h"
static const char* const cfg = "--target=generic-gnu --disable-vp9 --disable-vp8-encoder --disable-postproc --disable-error-concealment --disable-multithread --disable-spatial-resampling --disable-examples --disable-tools --disable-docs --enable-unit-tests --enable-debug";
const char *vpx_codec_build_config(void) {return cfg;}
```

That is the entire file: a license header, one `#include`, one static
string, one function. Three lines of code.

---

## Role in the decoder

`vpx_config.c` plays no part in decoding a single bit of a VP8 frame.
It is not on any hot path; it is not even on any cold path that the
decoder pipeline visits during normal operation. Its sole reason for
existing is to back the public API entry point
`vpx_codec_build_config()`, declared in `vpx/vpx_codec.h`:

```c
/*!\brief Return the build configuration
 *
 * Returns a printable string containing an encoded version of the build
 * configuration. This may be useful to vpx support.
 *
 */
const char *vpx_codec_build_config(void);
```

A caller — typically a test harness, a `--version`-style CLI flag in an
application that links libvpx, or a bug-report collection script —
invokes this function and prints the result. The string it returns is,
verbatim, the argument list passed to `./configure` when the library was
built. In the `vp8_only` build at hand, that list is the one shown
above: a `generic-gnu` target with VP9, the VP8 encoder, postproc,
error concealment, multithreading, spatial resampling, examples, tools,
and docs all disabled, and the unit tests plus debug build enabled.

This file is the *prose* face of the configuration. Its sibling
`vp8_only/vpx_config.h` is the *propositional* face: each configure
switch in the string above is reflected as a `#define CONFIG_xxx 0/1`
or `#define HAVE_xxx 0/1` macro that the rest of the source tree
consults at compile time. For example:

* `--disable-vp9`         -> `#define CONFIG_VP9 0` (line 76 of `vp8_only/vpx_config.h`),
  which also forces `CONFIG_VP9_ENCODER`, `CONFIG_VP9_DECODER`,
  `CONFIG_VP9_POSTPROC`, `CONFIG_VP9_TEMPORAL_DENOISING`, and
  `CONFIG_VP9_HIGHBITDEPTH` to zero (lines 73-74, 68, 96, 98).
* `--disable-vp8-encoder` -> `#define CONFIG_VP8_ENCODER 0` (line 71),
  with `CONFIG_VP8_DECODER 1` (line 72) left untouched because the
  symmetric `--disable-vp8-decoder` was not given.
* `--disable-postproc`    -> `#define CONFIG_POSTPROC 0` (line 67) and,
  by the same configure logic, `CONFIG_POSTPROC_VISUALIZER 0` (line 87).
* `--disable-multithread` -> `#define CONFIG_MULTITHREAD 0` (line 69).
* `--enable-debug`        -> `#define CONFIG_DEBUG 1` (line 54).

The two files therefore are not redundant. `vpx_config.h` lets the
compiler take or shed code per the configuration; `vpx_config.c` lets a
human ascertain, post-hoc and at run-time, what configuration that was.
They are produced by the same shell logic but consumed by different
audiences.

---

## What this translation unit contains

### `#include "vpx/vpx_codec.h"`

Pulled in for one reason: it is where `vpx_codec_build_config` is
declared (`vpx/vpx_codec.h:288`). Including the public header here lets
the compiler verify that the definition in this file matches the
declared signature — a small but real safeguard against ABI skew
between header and implementation.

The header drags in considerable extra material (`vpx_image.h`,
`vpx_integer.h`, the entire `vpx_codec_iface_t` machinery) that this
file does not use. None of it costs anything at run time, and the
compile-time overhead is invisible because the configure-generated
files are recompiled rarely.

### `static const char* const cfg = "..."`

The payload. A file-scope, doubly-`const` pointer to a string literal.
The double `const` is deliberate: the *pointer* is constant (cannot be
reassigned at runtime) and what it *points to* is also constant
(string-literal storage, typically placed in `.rodata` by GCC and
Clang). `static` keeps the symbol out of the link namespace, so the
choice of variable name (`cfg`) has no effect on anyone else's build.

The string's content is the verbatim `$CONFIGURE_ARGS` value the
`configure` script captured the moment it began processing. In the
`vp8_only` build:

```
--target=generic-gnu --disable-vp9 --disable-vp8-encoder
--disable-postproc --disable-error-concealment --disable-multithread
--disable-spatial-resampling --disable-examples --disable-tools
--disable-docs --enable-unit-tests --enable-debug
```

(reflowed for readability; the actual stored form is a single line, no
embedded newlines, with whatever quoting the shell stripped at the time
the user invoked `./configure`).

Invariants worth noting:

* The string is **never** modified after program load. Multiple decoder
  instances, multiple threads, multiple `dlopen` users — all see the
  same bytes.
* The string **may be empty** if libvpx was configured by running
  `./configure` with no arguments. The `vp8_only` build is not such a
  case; the unconfigured `build_debug/vpx_config.c` in this same tree
  is more typical of a `--target=…`-only invocation.
* The string contains exactly what the user typed and nothing more — it
  is not normalized, not deduplicated, and does not record defaults
  that were left implicit. A `--enable-debug` that was actually
  redundant (because debug is the platform default) will still be
  echoed back. Conversely, an option that the user did **not** name on
  the command line will not appear, even if `configure` enabled it
  internally as a side-effect of another switch.

### `const char *vpx_codec_build_config(void) {return cfg;}`

The public accessor. Three tokens of body. There is no synchronization,
no allocation, no copying: the returned pointer is the same `cfg`
pointer every call, and the caller must not free it or modify what it
points to (the API contract in `vpx/vpx_codec.h` does not spell this
out, but the `const` return type and the string-literal backing
together make the discipline unmissable).

This is the entire public surface of `vpx_config.c`. The translation
unit defines no other symbols.

---

## How it is generated

The file is produced by the last six lines of the top-level
`configure` script:

```sh
CONFIGURE_ARGS="$@"
process "$@"
print_webm_license ${BUILD_PFX}vpx_config.c "/*" " */"
cat <<EOF >> ${BUILD_PFX}vpx_config.c
#include "vpx/vpx_codec.h"
static const char* const cfg = "$CONFIGURE_ARGS";
const char *vpx_codec_build_config(void) {return cfg;}
EOF
```

(`configure:841-848`). Two things stand out.

First, `CONFIGURE_ARGS` is captured **before** `process` runs, so the
string really is the original command line — not the post-processing
result. If the user wrote `--disable-vp9` and `process` later concluded
that this also implies turning off five other knobs, the string still
says only `--disable-vp9`. The `vpx_config.h` macros tell you the
*consequences*; the `vpx_config.c` string tells you the *cause*.

Second, the script writes the file by simple shell heredoc concatenation.
There is no escape pass for shell metacharacters. If your configure
invocation contained a literal double-quote in an argument value, the
resulting C source would be syntactically invalid. In practice this
never happens because the supported configure flags are all of the form
`--name` or `--name=value` with `value` being either a path, an
identifier, or a small fixed enumeration.

The path component `${BUILD_PFX}` resolves to the build directory
prefix (here, `vp8_only/`), which is what places the generated file at
`/home/eugene/projects/libvpx/vp8_only/vpx_config.c`. The same configure
logic emits sibling files into the same directory:
`vpx_config.h`, `vpx_version.h`, and the three RTCD headers
(`vp8_rtcd.h`, `vpx_dsp_rtcd.h`, `vpx_scale_rtcd.h`); see the
"Generated files" table in `vp8_files.md` section C.

---

## Reproducibility and the case for pre-committing

Because `vpx_config.c` is regenerated on every configure run, the
following situations all produce different bytes in this file even when
the resulting decoder behaves identically:

* Re-running `./configure` with the same arguments but from a different
  working directory whose path appears in `--prefix=...`.
* Re-running with the arguments in a different order
  (`--disable-vp9 --disable-postproc` versus
  `--disable-postproc --disable-vp9`).
* Re-running with redundant flags appended
  (`--enable-debug --enable-debug` is preserved verbatim).
* Two developers who agreed on a feature set but typed it differently.

For a bit-reproducible build, this means `vpx_config.c` must be treated
either as a build artifact (regenerated and excluded from any hash that
is supposed to be reproducible across machines) or as a frozen source
file checked into the fork. The libvpx upstream chooses the former —
the file is emitted into the build directory and is not under
`git`-tracking — which is appropriate for a library that supports
arbitrary configure invocations.

A single-architecture fork that wants reproducible binaries can take
the latter route: run `./configure` once with the desired arguments,
copy `vpx_config.c` and `vpx_config.h` (and the RTCD headers) into the
source tree, delete the `configure` script, and let the Makefile (or a
hand-written CMake build) treat those files as ordinary sources. The
`vp8_files.md` doc walks through exactly this scenario at section C:

> For a single-arch fork you can pre-generate them once and check them
> in, eliminating the Perl dependency at build time.

The cost of doing so is small: a developer who wants to change the
configuration must edit `vpx_config.h` by hand (or re-run a pinned
configure invocation), and the build no longer adapts to a new compiler
or platform without manual intervention. The benefit is that the build
ceases to depend on Perl, on a working shell, and on `./configure`'s
two thousand-line probing logic, and the resulting object files become
bit-identical across machines.

For the present `vp8_only` build, which is itself a single-purpose
configuration kept around to verify a minimal decoder set, the latter
approach would be a natural next step: the configuration is fixed,
the target is fixed, and the cost of re-running `./configure` whenever
the source tree moves is unnecessary churn.

---

## Why the design is the shape it is

A reader might reasonably wonder why this information is shipped as a
runtime string at all rather than, say, a `#define` in `vpx_config.h`
that any tool could grep for at build time. There are two reasons.

The first is API stability. `vpx_codec_build_config()` is part of the
public ABI: it has been there since the earliest libvpx releases (the
license header in this file is dated 2011, when libvpx was first
WebM-licensed), and the WebM project commits to keeping it callable
forever. A user-space application that wants to log the library's
configuration cannot read a header file at run time; it must call a
function. Concentrating the implementation in one tiny translation
unit, and one tiny string, makes the cost of that commitment vanishingly
small.

The second is that the string and the macros serve different consumers.
The macros in `vpx_config.h` are read by the compiler, which needs
boolean answers it can branch on. The string in `vpx_config.c` is read
by humans, who want to see the original switches in the form they were
typed. Trying to reconstitute the string from the macros would be
lossy — many configure switches expand to several macros, several
macros may collapse to the same switch — and at any rate would require
the application to link in a substantial table-driven reverse mapping.
A literal echo of the command line is the smallest, most honest
representation.

---

## Cross-references

* The accessor declaration: `vpx/vpx_codec.h:288`.
* The generating logic: `configure:841-848`.
* The companion configuration header: `vp8_only/vpx_config.h`.
* The configure switches enabled here, in their broader build context:
  `documentation/vp8_files.md` (section header block lines 7-15).
* The RTCD initialiser that, like this file, is "shipped boilerplate
  around a generated artifact":
  `documentation/vp8_files/rtcd.md`.
