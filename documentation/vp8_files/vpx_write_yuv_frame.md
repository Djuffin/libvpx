# `vpx_util/vpx_write_yuv_frame.c` — debug YUV dump helper

## Role in the decoder

This translation unit contributes exactly one function — `vpx_write_yuv_frame` — to the libvpx build. Its job is to take a fully reconstructed (or partially processed) frame held in a `YV12_BUFFER_CONFIG` and append it, in raw planar YV12 byte order, to a caller-supplied `FILE *`. It is a *developer's diagnostic tap*: by sprinkling calls into the codec one can dump the source frame, the denoiser output, the skin-detection mask, or per-layer SVC inputs to disk and then inspect them with any tool that understands the headerless YV12 raw format (e.g. ffplay, mediainfo, custom Python scripts).

The function is gated behind a quartet of compile-time macros — `OUTPUT_YUV_SRC`, `OUTPUT_YUV_DENOISED`, `OUTPUT_YUV_SKINMAP`, `OUTPUT_YUV_SVC_SRC`. None of these are ever defined by `./configure`; each is a single-line `#define` that an engineer adds locally while hunting a bug. Without one of them defined, the entire body of the function compiles to a pair of `(void)` casts and the linker can fold it away, but the symbol itself is still exported so the call sites scattered through the encoder need not also be conditionally compiled.

In the verified minimal **VP8 decoder build** of `vp8_files.md`, this file is listed under *"compiled but unused"*: nothing in the decoder data-path ever calls `vpx_write_yuv_frame`. All four `OUTPUT_YUV_*` switches live in encoder-side code (denoiser, skin map, SVC source dump, raw source dump). The file is therefore explicitly flagged as **safely deletable in a decoder-only fork**:

> ```
> vpx_util/vpx_write_yuv_frame.c   debug YUV dump helper (safely deletable in a fork)
> ```
> *— `documentation/vp8_files.md`*

Keep this file in mind purely as a courtesy hook for downstream debugging; the contract is "I will faithfully serialise the visible pixels of a YV12 buffer to a stream, ignoring the invisible border."

## Includes

```c
#include "vpx_dsp/skin_detection.h"
#include "vpx_util/vpx_write_yuv_frame.h"
```

The `skin_detection.h` include is somewhat surprising — this file does not invoke any skin-detection API. It is pulled in because one of the gating macros, `OUTPUT_YUV_SKINMAP`, is conceptually paired with the skin detector: when a developer compiles with `OUTPUT_YUV_SKINMAP`, the skin detector writes a per-block mask into a YV12 buffer and then calls `vpx_write_yuv_frame` to serialise it. The include guards against the macro being defined in only one of the two translation units; including the header here surfaces any prototype mismatch at the dump site. Functionally for the dump itself it is a no-op include.

The own-header include (`vpx_util/vpx_write_yuv_frame.h`) does the usual job: it pulls in `<stdio.h>` for `FILE *` and `fwrite`, and `vpx_scale/yv12config.h` for the `YV12_BUFFER_CONFIG` definition. So the `.c` file does not need to repeat either include.

## The `YV12_BUFFER_CONFIG` fields we depend on

To understand the loop bodies it is worth re-reading the four fields per plane that the function touches (from `vpx_scale/yv12config.h`):

```c
int y_width;        int y_height;
int y_crop_width;   int y_crop_height;
int y_stride;
int uv_width;       int uv_height;
int uv_crop_width;  int uv_crop_height;
int uv_stride;
uint8_t *y_buffer;
uint8_t *u_buffer;
uint8_t *v_buffer;
```

A YV12 buffer in libvpx is allocated with a border of `VP8BORDERINPIXELS == 32` (for VP8) extra pixels on every side, so motion compensation can sample outside the picture without conditional branches. The struct therefore stores both a *padded* dimension (`y_width`, the width allocated, rounded up to a multiple of 16) and a *cropped* dimension (`y_crop_width`/`y_crop_height`, the original picture size). The base pointer `y_buffer` skips the top and left border, pointing at pixel (0, 0) of the visible frame; consecutive rows are `y_stride` bytes apart, and `y_stride > y_width` because of the right/left border that surrounds each row.

The dump function uses `y_width` as the per-row *write count* and `y_crop_height` as the number of rows. This is a deliberate, slightly asymmetric choice: it dumps a width that has been rounded up to a coding multiple but a height that has been cropped to the original picture height. The chroma planes follow the same pattern. The justification is that the *width* in the YV12 raw format must match what downstream tools assume the stride to be (a multiple of 16 for 4:2:0 alignment, so they can scan two chroma samples per four luma samples), while the *height* is the user-visible row count.

## `void vpx_write_yuv_frame(FILE *yuv_file, YV12_BUFFER_CONFIG *s)`

The single function in the file. Signature mirrors the header:

```c
void vpx_write_yuv_frame(FILE *yuv_file, YV12_BUFFER_CONFIG *s);
```

**What.** Append the Y, then U, then V planes of `*s` to the open file `yuv_file`, one row at a time, in raster order. No header is written; the caller is responsible for noting the frame dimensions externally (e.g. by encoding them in the filename — the common convention is `dump_640x480.yuv`).

**Why this exists.** Codec bring-up and debugging routinely require comparing two visually identical pipelines pixel-for-pixel. A raw YV12 dump is the cheapest serialisation: no compression, no colour-space conversion, no I/O abstraction. Once an engineer suspects, say, that the denoiser is corrupting frame N, they `#define OUTPUT_YUV_DENOISED` at the top of the denoiser file, rebuild, run, and `ffplay -f rawvideo -pixel_format yv12 -video_size WxH dump.yuv` to inspect the results frame by frame.

**Invariants assumed.**

1. `yuv_file` is open for writing in binary mode; the function makes no effort to check `ferror` or `fwrite`'s return value. A short write is silently dropped.
2. `s->y_buffer`, `s->u_buffer`, `s->v_buffer` all point at the (0, 0) pixel of their respective planes, *past* the border. This is the standard libvpx convention established by `vp8_yv12_alloc_frame_buffer`.
3. `s->y_crop_height >= 1` and `s->uv_crop_height >= 1`. The function uses do/while loops, which would underflow the row counter and run for 4 GiB of rows if a plane were zero-height. In practice no real frame ever has a zero-height plane, but the lack of a guard means this is *not* a generic utility — it is a debug hook with debug-quality preconditions.
4. The `OUTPUT_YUV_*` macros' state is consistent across all translation units that share the dump file. There is no global header that defines them, so it is the developer's job to define them in a single common location (typically a fresh `#define` in the top of the file that will *call* `vpx_write_yuv_frame`).

**How used.** Search call sites with `grep -r vpx_write_yuv_frame` in the encoder tree; you will find them in the VP8 and VP9 denoiser, the skin-map producer, the SVC source-frame logger, and the raw-source dump in the top-level encoder driver. Each call is wrapped in `#ifdef OUTPUT_YUV_*` so the function itself only needs to be callable, not enabled.

### The conditional-compile gate

```c
#if defined(OUTPUT_YUV_SRC) || defined(OUTPUT_YUV_DENOISED) || \
    defined(OUTPUT_YUV_SKINMAP) || defined(OUTPUT_YUV_SVC_SRC)
```

The entire writing body is wrapped in this `#if`. If *any* of the four macros is defined, the body is compiled. If none is defined, the `#else` branch runs:

```c
#else
  (void)yuv_file;
  (void)s;
#endif
```

The `(void)` casts suppress the `-Wunused-parameter` warning that would otherwise fire on the no-op build. Note that the *symbol* `vpx_write_yuv_frame` is always emitted — even without any macro, the function exists, just as an empty stub. This is what lets unconditional call-site wrappers like

```c
#ifdef OUTPUT_YUV_SRC
  vpx_write_yuv_frame(yuv_file, &cpi->raw_source_frame);
#endif
```

…compile cleanly without forcing the same `#ifdef` discipline on every translation unit, *and* what makes the file always show up in the build's `.o` list (the reason the `vp8_files.md` inventory mentions it among the 47 mandatory objects despite it being functionally dead code in the decoder).

### Walking the luma plane

```c
unsigned char *src = s->y_buffer;
int h = s->y_crop_height;

do {
  fwrite(src, s->y_width, 1, yuv_file);
  src += s->y_stride;
} while (--h);
```

The structure of all three plane loops is identical, so it is worth dissecting this one in detail.

`src` starts at the top-left visible luma pixel. Each iteration writes `s->y_width` bytes — note the **third argument to `fwrite` is `1`**, meaning "one block of `y_width` bytes", not "`y_width` blocks of one byte". The two are observationally identical (`fwrite` returns the number of blocks written), but using a single block makes a short write detectable as "0 returned" rather than "some smaller integer". The function ignores the return regardless.

After writing the row, `src` advances by `s->y_stride`, **not** by `s->y_width`. This is the border-aware stride step — it skips over the right-edge border padding of the current row and the left-edge border padding of the next row in one jump. Failing to do this would interleave border garbage into the dump.

The loop counts down using `do {...} while (--h)`. This is a deliberate idiom in libvpx: a `do/while` saves one branch compared to `for` and works correctly for `h >= 1`. It does *not* work for `h == 0` — the loop would run 2^32 times — but as noted in the invariants, that case never occurs for a real frame.

### Walking the U (Cb) plane

```c
src = s->u_buffer;
h = s->uv_crop_height;

do {
  fwrite(src, s->uv_width, 1, yuv_file);
  src += s->uv_stride;
} while (--h);
```

Same structure, but on the U plane. The stride and width fields switch to their `uv_*` counterparts. In 4:2:0 (the only sub-sampling VP8 supports) `uv_width = y_width / 2` and `uv_crop_height = y_crop_height / 2`, so this loop writes a quarter as many bytes as the luma loop. `u_buffer` points to (0, 0) of the chroma plane, past its (smaller) border.

### Walking the V (Cr) plane

```c
src = s->v_buffer;
h = s->uv_crop_height;

do {
  fwrite(src, s->uv_width, 1, yuv_file);
  src += s->uv_stride;
} while (--h);
```

Identical to the U loop, but on the V plane. The Y-then-U-then-V ordering is what makes the dump *YV12* in libvpx's sense — historically the "YV12" four-CC actually means *Y, V, U* (V before U), but libvpx, ffmpeg's `yuv420p`, and most modern tools all use Y, U, V order. When loading the dump back with `ffplay -pixel_format yv12` you are getting the latter convention. If you misname the pixel format as the *original* fourcc YV12, the colours will be swapped — a classic gotcha when sharing dumps between teams.

## What this file is *not*

It is worth noting what is conspicuously absent, because a reader scanning the file might expect more:

- **No header writing.** Unlike a Y4M dump, there is no per-stream or per-frame magic. To replay the file you must know `y_width`, `uv_width`, and `y_crop_height` from elsewhere. The conventional libvpx workaround is to embed the resolution in the filename and rely on the developer's discipline.
- **No high-bit-depth path.** The function treats every pixel as 8-bit by reading `unsigned char *`. If `bit_depth > 8` and `flags & YV12_FLAG_HIGHBITDEPTH`, the buffer actually holds 16-bit samples (twice the bytes), and this function will dump exactly half of them with byte-pair endian artifacts. Correct dumping of HBD frames would require a parallel path that scales the row byte counts by 2 — but since the OUTPUT macros are debug-only and the original developers only ever wired them up for 8-bit experiments, no one added that branch.
- **No error handling.** Disk full? Permission denied? `fwrite` returns 0 and the function moves on. Acceptable for a debug-only artefact; would be a bug in production code.
- **No threading guard.** Two threads calling `vpx_write_yuv_frame` on the same `FILE *` will interleave their bytes catastrophically. Multi-threaded debug dumps must use one file per thread.

Each of these absences is justified by the file's narrow remit: a hand-rolled, define-to-enable, debug-only YV12 serialiser for a single-threaded developer staring at frame data.

## Deletion guidance for a decoder-only fork

The accompanying `vp8_files.md` is explicit:

> If you also accept the "compiled-but-unused" surprises listed in section A:
> ```
> vpx_util/vpx_write_yuv_frame.{c,h}
> ```
> You can hand-delete these in a fork; the build system pulls them in unconditionally, so just trimming the Makefile entries and `.c` files is enough — the resulting `libvpx.a` will be a few KB smaller.

To delete safely:

1. Remove the entries from `vpx_util/vpx_util.mk`.
2. Delete `vpx_util/vpx_write_yuv_frame.c` and `vpx_util/vpx_write_yuv_frame.h`.
3. Grep the remaining tree for `vpx_write_yuv_frame` — in a pure VP8 decoder build the result is empty, confirming that no live call site is broken.

The `skin_detection.h` include here also has zero functional effect on the decoder; if you simultaneously drop `vpx_dsp/skin_detection.{c,h}` as the same document recommends, you avoid even the cross-include relationship and the deletion is fully clean.
