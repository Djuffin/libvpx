//! VP8 decoder performance benchmark.
//!
//! Each bench reads an IVF file into memory once (outside the timed
//! loop) and times one full init → decode-all-frames → destroy cycle
//! through the public API. Test streams are generated on-demand via
//! ffmpeg (libvpx VP8 encoder) into `target/` and cached between runs.
//!
//! Requires `ffmpeg` on `PATH` with `libvpx` enabled (Ubuntu:
//! `apt install ffmpeg`).
//!
//! Run all benches with `cargo bench`. Run one with
//! `cargo bench --bench decode -- mandelbrot_720p`.
#![allow(unsafe_op_in_unsafe_fn)]

use core::mem::MaybeUninit;
use core::ptr;
use std::fs;
use std::io::{BufReader, Read};
use std::path::{Path, PathBuf};
use std::process::Command;

use criterion::{Criterion, criterion_group, criterion_main};
use std::hint::black_box;

use vp8_decoder_rs::vp8_dx_iface::vpx_codec_vp8_dx;
use vp8_decoder_rs::vpx_api::{
    VPX_CODEC_OK, VPX_DECODER_ABI_VERSION, vpx_codec_ctx_t, vpx_codec_dec_init_ver,
    vpx_codec_decode, vpx_codec_destroy, vpx_codec_get_frame,
};

// ---------------------------------------------------------------------
// Test-stream generation via ffmpeg
// ---------------------------------------------------------------------

/// Run ffmpeg to encode a mandelbrot test pattern to a VP8 IVF stream.
/// No-op if `path` already exists.
fn generate_vp8_stream(path: &Path, size: &str, duration_secs: u32, bitrate: &str) {
    if path.exists() {
        return;
    }

    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }

    println!(
        "Generating VP8 benchmark stream {} ({size}, {duration_secs}s, {bitrate})...",
        path.display()
    );

    let status = Command::new("ffmpeg")
        .args([
            "-y",
            "-loglevel", "error",
            "-f", "lavfi",
            "-i", &format!("mandelbrot=size={size}:rate=30"),
            "-t", &duration_secs.to_string(),
            "-c:v", "libvpx",
            "-b:v", bitrate,
            "-deadline", "good",
            "-cpu-used", "1",
            "-pix_fmt", "yuv420p",
            path.to_str().expect("non-UTF8 path"),
        ])
        .status()
        .expect("Failed to spawn ffmpeg — install ffmpeg with libvpx enabled");

    assert!(
        status.success(),
        "ffmpeg failed to generate {}",
        path.display()
    );
}

// ---------------------------------------------------------------------
// IVF slurp + decode driver
// ---------------------------------------------------------------------

/// Read a whole IVF file into a list of compressed-frame buffers so the
/// per-iteration loop doesn't pay for file IO or IVF parsing.
fn slurp_ivf(path: &Path) -> Vec<Vec<u8>> {
    let mut reader = BufReader::new(fs::File::open(path).expect("open ivf"));
    let mut hdr = [0u8; 32];
    reader.read_exact(&mut hdr).expect("read ivf header");
    assert_eq!(&hdr[0..4], b"DKIF", "{path:?} missing DKIF magic");

    let mut frames = Vec::new();
    loop {
        let mut fhdr = [0u8; 12];
        if reader.read_exact(&mut fhdr).is_err() {
            break;
        }
        let size = u32::from_le_bytes([fhdr[0], fhdr[1], fhdr[2], fhdr[3]]) as usize;
        let mut buf = vec![0u8; size];
        reader.read_exact(&mut buf).expect("read ivf frame");
        frames.push(buf);
    }
    frames
}

/// init → decode every packet (draining every emitted image) → destroy.
/// Decoder is constructed fresh each iteration; for the dozens-to-hundreds
/// of frames per vector here, init overhead is well below 1% of the
/// measurement.
unsafe fn decode_all(packets: &[Vec<u8>]) {
    let iface = vpx_codec_vp8_dx();
    let mut dec = MaybeUninit::<vpx_codec_ctx_t>::zeroed();
    let init_res = vpx_codec_dec_init_ver(
        dec.assume_init_mut(),
        Some(iface),
        None,
        0,
        VPX_DECODER_ABI_VERSION,
    );
    assert_eq!(init_res, VPX_CODEC_OK);
    let mut dec = dec.assume_init();

    for packet in packets {
        let res = vpx_codec_decode(&mut dec, packet, ptr::null_mut(), 0);
        assert_eq!(res, VPX_CODEC_OK);

        let mut iter: *const core::ffi::c_void = ptr::null();
        while let Some(img) = vpx_codec_get_frame(&mut dec, &mut iter) {
            black_box(img);
        }
    }

    assert_eq!(vpx_codec_destroy(&mut dec), VPX_CODEC_OK);
}

fn bench_decoder(b: &mut criterion::Bencher, packets: &[Vec<u8>]) {
    b.iter(|| unsafe { decode_all(black_box(packets)) });
}

// ---------------------------------------------------------------------
// Bench definitions
// ---------------------------------------------------------------------

fn target_dir() -> PathBuf {
    // Cargo invokes the bench binary from the crate root; benches/decode.rs
    // sits at CARGO_MANIFEST_DIR. Cache generated streams in target/ so
    // they survive `cargo clean -p` of the bench artifact alone.
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("target")
}

pub fn decoding_benchmark(c: &mut Criterion) {
    let target = target_dir();

    // 720p, 3 seconds, 2 Mbps.
    let p720 = target.join("bench_mandelbrot_720p.ivf");
    generate_vp8_stream(&p720, "1280x720", 3, "2M");

    // 480p, 2 seconds, 1 Mbps.
    let p480 = target.join("bench_mandelbrot_480p.ivf");
    generate_vp8_stream(&p480, "854x480", 2, "1M");

    let mut group = c.benchmark_group("decode");
    group.sample_size(20);

    let packets_480 = slurp_ivf(&p480);
    group.bench_function("mandelbrot_480p", |b| bench_decoder(b, &packets_480));

    let packets_720 = slurp_ivf(&p720);
    group.bench_function("mandelbrot_720p", |b| bench_decoder(b, &packets_720));

    group.finish();
}

criterion_group!(benches, decoding_benchmark);
criterion_main!(benches);
