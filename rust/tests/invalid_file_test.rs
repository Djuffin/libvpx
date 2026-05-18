#![allow(unsafe_op_in_unsafe_fn)]
//! Port of `test/invalid_file_test.cc` to Rust integration tests.
//!
//! For each corrupt VP8 IVF, decode every frame and verify the returned
//! `vpx_codec_err_t` matches the canonical `.res` companion file
//! (one integer per frame, `0` = OK, `7` = `VPX_CODEC_CORRUPT_FRAME`).
//!
//! Both `InvalidFileTest` and `InvalidFileInvalidPeekTest` collapse to
//! the same body — the only difference in the C suite is that the
//! peek-test class overrides `HandlePeekResult` to no-op, which is
//! implicit here (we never call peek).
//!
//! Test data is discovered at build time by `build.rs`. If discovery
//! fails the tests skip themselves rather than failing.

use core::mem::MaybeUninit;
use core::ptr;
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::PathBuf;

use vp8_decoder_rs::vp8_dx_iface::vpx_codec_vp8_dx;
use vp8_decoder_rs::vpx_api::{
    vpx_codec_ctx_t, vpx_codec_dec_init_ver, vpx_codec_decode, vpx_codec_destroy,
    vpx_codec_iface_t, VpxCodecErr, VPX_CODEC_CORRUPT_FRAME, VPX_CODEC_OK,
    VPX_DECODER_ABI_VERSION,
};

const TEST_DATA_DIR: &str = env!("VP8_TEST_DATA_DIR");

fn test_data_dir(test_name: &str) -> Option<PathBuf> {
    if TEST_DATA_DIR.is_empty() {
        eprintln!(
            "skipping {test_name}: VP8 test data not available \
             (set LIBVPX_TEST_DATA_PATH or ensure network access at build time)"
        );
        return None;
    }
    Some(PathBuf::from(TEST_DATA_DIR))
}

// --- IVF reader (same shape as test_vector_test) -------------------------

struct IvfReader {
    inner: BufReader<File>,
}

impl IvfReader {
    fn open(path: &PathBuf) -> std::io::Result<Self> {
        let mut inner = BufReader::new(File::open(path)?);
        let mut hdr = [0u8; 32];
        inner.read_exact(&mut hdr)?;
        assert_eq!(&hdr[0..4], b"DKIF", "{path:?} missing DKIF magic");
        Ok(Self { inner })
    }
    fn next_frame(&mut self) -> Option<Vec<u8>> {
        let mut hdr = [0u8; 12];
        if self.inner.read_exact(&mut hdr).is_err() {
            return None;
        }
        let size = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]) as usize;
        let mut buf = vec![0u8; size];
        self.inner.read_exact(&mut buf).ok()?;
        Some(buf)
    }
}

/// Read a `.res` file. Each non-empty line is one decimal integer
/// (the expected `vpx_codec_err_t` for that frame).
fn read_res_file(path: &PathBuf) -> Vec<VpxCodecErr> {
    let f = File::open(path).expect("open .res file");
    BufReader::new(f)
        .lines()
        .filter_map(|l| l.ok())
        .filter(|l| !l.trim().is_empty())
        .map(|l| match l.trim().parse::<i32>().expect("parse .res code") {
            0 => VPX_CODEC_OK,
            7 => VPX_CODEC_CORRUPT_FRAME,
            other => panic!("unknown expected error code {other} in {path:?}"),
        })
        .collect()
}

unsafe fn run_invalid_file(name: &str) {
    let Some(dir) = test_data_dir(name) else { return };
    let ivf_path = dir.join(name);
    let res_path = dir.join(format!("{name}.res"));
    let expected = read_res_file(&res_path);

    let mut reader = IvfReader::open(&ivf_path).expect("open ivf");

    let iface = vpx_codec_vp8_dx() as *mut vpx_codec_iface_t;
    let mut dec_uninit = MaybeUninit::<vpx_codec_ctx_t>::uninit();
    assert_eq!(
        vpx_codec_dec_init_ver(
            dec_uninit.as_mut_ptr(),
            iface,
            ptr::null(),
            0,
            VPX_DECODER_ABI_VERSION,
        ),
        VPX_CODEC_OK,
        "{name}: dec_init failed",
    );
    let mut dec = dec_uninit.assume_init();

    let mut frame_no = 0usize;
    while let Some(packet) = reader.next_frame() {
        assert!(
            frame_no < expected.len(),
            "{name}: more input frames than .res lines",
        );
        let res = vpx_codec_decode(
            &mut dec,
            packet.as_ptr(),
            packet.len() as u32,
            ptr::null_mut(),
            0,
        );
        assert_eq!(
            res, expected[frame_no],
            "{name} frame {frame_no}: decoder returned {res:?}, .res expected {:?}",
            expected[frame_no],
        );
        frame_no += 1;
    }
    assert_eq!(
        frame_no,
        expected.len(),
        "{name}: decoded {frame_no} frames, .res has {} entries",
        expected.len(),
    );

    assert_eq!(vpx_codec_destroy(&mut dec), VPX_CODEC_OK);
}

// `InvalidFileTest` — C: `kVP8InvalidFileTests`.
#[test] fn invalid_bug_1443()                { unsafe { run_invalid_file("invalid-bug-1443.ivf") } }
#[test] fn invalid_bug_148271109()           { unsafe { run_invalid_file("invalid-bug-148271109.ivf") } }
#[test] fn invalid_token_partition()         { unsafe { run_invalid_file("invalid-token-partition.ivf") } }
#[test] fn invalid_comprehensive_s17661()    { unsafe { run_invalid_file("invalid-vp80-00-comprehensive-s17661_r01-05_b6-.ivf") } }

// `InvalidFileInvalidPeekTest` — C: `kVP8InvalidPeekTests`. Same body;
// distinction in the C suite is that peek-result handling is overridden
// to no-op. Implicit in this port (no separate peek call).
#[test] fn invalid_peek_2kf_0x6()            { unsafe { run_invalid_file("invalid-vp80-00-comprehensive-018.ivf.2kf_0x6.ivf") } }
