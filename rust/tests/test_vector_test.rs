#![allow(unsafe_op_in_unsafe_fn)]
//! Port of `test/test_vector_test.cc` to Rust integration tests.
//!
//! Iterates the 62 VP8 conformance vectors, decodes every frame through
//! the public API, and verifies the per-frame MD5 against the canonical
//! `<filename>.md5` companion file.
//!
//! Test data is discovered at build time by `build.rs`: it honours
//! `$LIBVPX_TEST_DATA_PATH`, probes a few conventional locations, and
//! falls back to downloading from the WebM project storage bucket. If
//! none of that works the tests skip themselves at runtime rather than
//! failing.

use core::mem::MaybeUninit;
use core::ptr;
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::PathBuf;

use md5::{Digest, Md5};

use vp8_decoder_rs::vp8_dx_iface::vpx_codec_vp8_dx;
use vp8_decoder_rs::vpx_api::{
    vpx_codec_ctx_t, vpx_codec_dec_init_ver, vpx_codec_decode, vpx_codec_destroy,
    vpx_codec_get_frame, vpx_codec_iface_t, vpx_image_t, VPX_CODEC_OK,
    VPX_DECODER_ABI_VERSION, VPX_IMG_FMT_HIGHBITDEPTH,
};

/// Build-time-discovered test data location (see `build.rs`). Empty
/// string if neither the local probe nor the download fallback
/// succeeded — tests skip silently in that case.
const TEST_DATA_DIR: &str = env!("VP8_TEST_DATA_DIR");

/// Returns `Some(dir)` if test data is available, otherwise prints a
/// skip notice and returns `None`. Callers `return` on `None`.
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

/// The 62 VP8 conformance test vectors (`test/test_vectors.cc:18-51`).
#[rustfmt::skip]
const VP8_TEST_VECTORS: &[&str] = &[
    "vp80-00-comprehensive-001.ivf", "vp80-00-comprehensive-002.ivf",
    "vp80-00-comprehensive-003.ivf", "vp80-00-comprehensive-004.ivf",
    "vp80-00-comprehensive-005.ivf", "vp80-00-comprehensive-006.ivf",
    "vp80-00-comprehensive-007.ivf", "vp80-00-comprehensive-008.ivf",
    "vp80-00-comprehensive-009.ivf", "vp80-00-comprehensive-010.ivf",
    "vp80-00-comprehensive-011.ivf", "vp80-00-comprehensive-012.ivf",
    "vp80-00-comprehensive-013.ivf", "vp80-00-comprehensive-014.ivf",
    "vp80-00-comprehensive-015.ivf", "vp80-00-comprehensive-016.ivf",
    "vp80-00-comprehensive-017.ivf", "vp80-00-comprehensive-018.ivf",
    "vp80-01-intra-1400.ivf",        "vp80-01-intra-1411.ivf",
    "vp80-01-intra-1416.ivf",        "vp80-01-intra-1417.ivf",
    "vp80-02-inter-1402.ivf",        "vp80-02-inter-1412.ivf",
    "vp80-02-inter-1418.ivf",        "vp80-02-inter-1424.ivf",
    "vp80-03-segmentation-01.ivf",   "vp80-03-segmentation-02.ivf",
    "vp80-03-segmentation-03.ivf",   "vp80-03-segmentation-04.ivf",
    "vp80-03-segmentation-1401.ivf", "vp80-03-segmentation-1403.ivf",
    "vp80-03-segmentation-1407.ivf", "vp80-03-segmentation-1408.ivf",
    "vp80-03-segmentation-1409.ivf", "vp80-03-segmentation-1410.ivf",
    "vp80-03-segmentation-1413.ivf", "vp80-03-segmentation-1414.ivf",
    "vp80-03-segmentation-1415.ivf", "vp80-03-segmentation-1425.ivf",
    "vp80-03-segmentation-1426.ivf", "vp80-03-segmentation-1427.ivf",
    "vp80-03-segmentation-1432.ivf", "vp80-03-segmentation-1435.ivf",
    "vp80-03-segmentation-1436.ivf", "vp80-03-segmentation-1437.ivf",
    "vp80-03-segmentation-1441.ivf", "vp80-03-segmentation-1442.ivf",
    "vp80-04-partitions-1404.ivf",   "vp80-04-partitions-1405.ivf",
    "vp80-04-partitions-1406.ivf",   "vp80-05-sharpness-1428.ivf",
    "vp80-05-sharpness-1429.ivf",    "vp80-05-sharpness-1430.ivf",
    "vp80-05-sharpness-1431.ivf",    "vp80-05-sharpness-1433.ivf",
    "vp80-05-sharpness-1434.ivf",    "vp80-05-sharpness-1438.ivf",
    "vp80-05-sharpness-1439.ivf",    "vp80-05-sharpness-1440.ivf",
    "vp80-05-sharpness-1443.ivf",    "vp80-06-smallsize.ivf",
];

#[test]
fn vector_list_has_62_entries() {
    assert_eq!(VP8_TEST_VECTORS.len(), 62);
}

// ---------------------------------------------------------------------
// IVF parser
// ---------------------------------------------------------------------

/// Minimal IVF reader. `vp80*.ivf` files use 32-byte file headers and
/// 12-byte per-frame headers (4-byte LE size + 8-byte LE pts).
struct IvfReader {
    inner: BufReader<File>,
}

impl IvfReader {
    fn open(path: &PathBuf) -> std::io::Result<Self> {
        let mut inner = BufReader::new(File::open(path)?);
        // Skip the 32-byte file header. We don't need the contents.
        let mut hdr = [0u8; 32];
        inner.read_exact(&mut hdr)?;
        assert_eq!(&hdr[0..4], b"DKIF", "{path:?} missing DKIF magic");
        Ok(Self { inner })
    }

    /// Read the next packet, or `None` at EOF.
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

// ---------------------------------------------------------------------
// MD5 helper (mirrors `test/md5_helper.h::MD5::Add(vpx_image_t*)`)
// ---------------------------------------------------------------------

unsafe fn md5_of_image(img: &vpx_image_t) -> String {
    let mut hasher = Md5::new();
    let highbd = (img.fmt & VPX_IMG_FMT_HIGHBITDEPTH) != 0;
    let bytes_per_sample = if highbd { 2 } else { 1 };

    for plane in 0..3 {
        let h = if plane == 0 {
            img.d_h as usize
        } else {
            ((img.d_h + img.y_chroma_shift) >> img.y_chroma_shift) as usize
        };
        let w = if plane == 0 {
            (img.d_w as usize) * bytes_per_sample
        } else {
            (((img.d_w + img.x_chroma_shift) >> img.x_chroma_shift) as usize) * bytes_per_sample
        };

        let mut buf: *const u8 = img.planes[plane];
        for _ in 0..h {
            let row = core::slice::from_raw_parts(buf, w);
            hasher.update(row);
            buf = buf.add(img.stride[plane] as usize);
        }
    }

    let digest = hasher.finalize();
    let mut s = String::with_capacity(32);
    for b in digest {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

// ---------------------------------------------------------------------
// .md5 file reader — each line: `<32 hex chars>  <basename>.i420`
// ---------------------------------------------------------------------

fn read_md5_lines(path: &PathBuf) -> Vec<String> {
    let f = File::open(path).expect("open .md5 file");
    BufReader::new(f)
        .lines()
        .map(|l| {
            let l = l.expect("read .md5 line");
            l.split_whitespace().next().expect("md5 hex").to_owned()
        })
        .collect()
}

// ---------------------------------------------------------------------
// Decode-and-compare driver
// ---------------------------------------------------------------------

unsafe fn init_dec() -> vpx_codec_ctx_t {
    let iface = vpx_codec_vp8_dx() as *mut vpx_codec_iface_t;
    let mut dec_uninit = MaybeUninit::<vpx_codec_ctx_t>::uninit();
    let init_res = vpx_codec_dec_init_ver(
        dec_uninit.as_mut_ptr(),
        iface,
        ptr::null(),
        0,
        VPX_DECODER_ABI_VERSION,
    );
    assert_eq!(init_res, VPX_CODEC_OK, "dec_init failed");
    dec_uninit.assume_init()
}

/// Decode at most `max_packets` IVF packets from `name`, asserting
/// per-frame MD5 matches for every *displayed* frame that surfaces.
///
/// `max_packets = Some(1)` decodes the first IVF packet — that packet
/// might be a normal keyframe (1 image out, MD5 verified) or an
/// invisible keyframe (0 images out, only "didn't crash" verified).
/// `max_packets = None` decodes the whole file.
unsafe fn run_one_vector(name: &str, max_packets: Option<usize>) {
    let Some(dir) = test_data_dir(name) else { return };
    let ivf_path = dir.join(name);
    let md5_path = dir.join(format!("{name}.md5"));
    let expected = read_md5_lines(&md5_path);

    let mut reader = IvfReader::open(&ivf_path).expect("open ivf");
    let mut dec = init_dec();

    let mut frame_no = 0usize;
    let mut packets_decoded = 0usize;
    while let Some(packet) = reader.next_frame() {
        if matches!(max_packets, Some(limit) if packets_decoded >= limit) {
            break;
        }
        let res = vpx_codec_decode(
            &mut dec,
            packet.as_ptr(),
            packet.len() as u32,
            ptr::null_mut(),
            0,
        );
        assert_eq!(res, VPX_CODEC_OK,
            "vpx_codec_decode failed on {name} packet {packets_decoded}");
        packets_decoded += 1;

        let mut iter: *const core::ffi::c_void = ptr::null();
        let mut img = vpx_codec_get_frame(&mut dec, &mut iter);
        while !img.is_null() {
            assert!(frame_no < expected.len(),
                "{name}: more decoded frames than md5 lines");
            let got = md5_of_image(&*img);
            assert_eq!(got, expected[frame_no],
                "{name}: md5 mismatch at frame {frame_no}");
            frame_no += 1;
            img = vpx_codec_get_frame(&mut dec, &mut iter);
        }
    }

    if max_packets.is_none() {
        assert_eq!(frame_no, expected.len(),
            "{name}: decoded {frame_no} frames, md5 file has {}", expected.len());
    }

    assert_eq!(vpx_codec_destroy(&mut dec), VPX_CODEC_OK);
}

/// Generates one `#[test] fn keyframe_NNN()` per VP8 conformance vector.
/// Each test decodes only frame 0 (the keyframe) and asserts the MD5
/// matches the canonical libvpx output. Inter-frame decoding is broken
/// (see `full_vector_001`), so we stop after the keyframe.
///
/// Vectors known to fail at the keyframe stage are tagged with
/// `#[ignore]` plus a one-line bug summary so the rest of the suite
/// stays green.
macro_rules! keyframe_tests {
    ($($(#[$attr:meta])* $id:ident => $vector:literal),* $(,)?) => {
        $(
            #[test]
            $(#[$attr])*
            fn $id() {
                unsafe { run_one_vector($vector, Some(1)) }
            }
        )*
    };
}

keyframe_tests! {
    keyframe_001 => "vp80-00-comprehensive-001.ivf",
    keyframe_002 => "vp80-00-comprehensive-002.ivf",
    keyframe_003 => "vp80-00-comprehensive-003.ivf",
    keyframe_004 => "vp80-00-comprehensive-004.ivf",
    keyframe_005 => "vp80-00-comprehensive-005.ivf",
    keyframe_006 => "vp80-00-comprehensive-006.ivf",
    keyframe_007 => "vp80-00-comprehensive-007.ivf",
    keyframe_008 => "vp80-00-comprehensive-008.ivf",
    keyframe_009 => "vp80-00-comprehensive-009.ivf",
    keyframe_010 => "vp80-00-comprehensive-010.ivf",
    keyframe_011 => "vp80-00-comprehensive-011.ivf",
    keyframe_012 => "vp80-00-comprehensive-012.ivf",
    keyframe_013 => "vp80-00-comprehensive-013.ivf",
    keyframe_014 => "vp80-00-comprehensive-014.ivf",
    keyframe_015 => "vp80-00-comprehensive-015.ivf",
    keyframe_016 => "vp80-00-comprehensive-016.ivf",
    keyframe_017 => "vp80-00-comprehensive-017.ivf",
    keyframe_018 => "vp80-00-comprehensive-018.ivf",
    keyframe_intra_1400 => "vp80-01-intra-1400.ivf",
    keyframe_intra_1411 => "vp80-01-intra-1411.ivf",
    keyframe_intra_1416 => "vp80-01-intra-1416.ivf",
    keyframe_intra_1417 => "vp80-01-intra-1417.ivf",
    keyframe_inter_1402 => "vp80-02-inter-1402.ivf",
    keyframe_inter_1412 => "vp80-02-inter-1412.ivf",
    keyframe_inter_1418 => "vp80-02-inter-1418.ivf",
    keyframe_inter_1424 => "vp80-02-inter-1424.ivf",
    keyframe_seg_01 => "vp80-03-segmentation-01.ivf",
    keyframe_seg_02 => "vp80-03-segmentation-02.ivf",
    keyframe_seg_03 => "vp80-03-segmentation-03.ivf",
    keyframe_seg_04 => "vp80-03-segmentation-04.ivf",
    keyframe_seg_1401 => "vp80-03-segmentation-1401.ivf",
    keyframe_seg_1403 => "vp80-03-segmentation-1403.ivf",
    keyframe_seg_1407 => "vp80-03-segmentation-1407.ivf",
    keyframe_seg_1408 => "vp80-03-segmentation-1408.ivf",
    keyframe_seg_1409 => "vp80-03-segmentation-1409.ivf",
    keyframe_seg_1410 => "vp80-03-segmentation-1410.ivf",
    keyframe_seg_1413 => "vp80-03-segmentation-1413.ivf",
    keyframe_seg_1414 => "vp80-03-segmentation-1414.ivf",
    keyframe_seg_1415 => "vp80-03-segmentation-1415.ivf",
    keyframe_seg_1425 => "vp80-03-segmentation-1425.ivf",
    keyframe_seg_1426 => "vp80-03-segmentation-1426.ivf",
    keyframe_seg_1427 => "vp80-03-segmentation-1427.ivf",
    keyframe_seg_1432 => "vp80-03-segmentation-1432.ivf",
    keyframe_seg_1435 => "vp80-03-segmentation-1435.ivf",
    keyframe_seg_1436 => "vp80-03-segmentation-1436.ivf",
    keyframe_seg_1437 => "vp80-03-segmentation-1437.ivf",
    keyframe_seg_1441 => "vp80-03-segmentation-1441.ivf",
    keyframe_seg_1442 => "vp80-03-segmentation-1442.ivf",
    keyframe_part_1404 => "vp80-04-partitions-1404.ivf",
    keyframe_part_1405 => "vp80-04-partitions-1405.ivf",
    keyframe_part_1406 => "vp80-04-partitions-1406.ivf",
    keyframe_sharp_1428 => "vp80-05-sharpness-1428.ivf",
    keyframe_sharp_1429 => "vp80-05-sharpness-1429.ivf",
    keyframe_sharp_1430 => "vp80-05-sharpness-1430.ivf",
    keyframe_sharp_1431 => "vp80-05-sharpness-1431.ivf",
    keyframe_sharp_1433 => "vp80-05-sharpness-1433.ivf",
    keyframe_sharp_1434 => "vp80-05-sharpness-1434.ivf",
    keyframe_sharp_1438 => "vp80-05-sharpness-1438.ivf",
    keyframe_sharp_1439 => "vp80-05-sharpness-1439.ivf",
    keyframe_sharp_1440 => "vp80-05-sharpness-1440.ivf",
    keyframe_sharp_1443 => "vp80-05-sharpness-1443.ivf",
    keyframe_smallsize => "vp80-06-smallsize.ivf",
}

/// Generates one full-decode test per vector. Each decodes EVERY frame
/// (keyframe + all inter frames) and verifies the per-frame MD5.
macro_rules! full_vector_tests {
    ($($(#[$attr:meta])* $id:ident => $vector:literal),* $(,)?) => {
        $(
            #[test]
            $(#[$attr])*
            fn $id() {
                unsafe { run_one_vector($vector, None) }
            }
        )*
    };
}

full_vector_tests! {
    full_001 => "vp80-00-comprehensive-001.ivf",
    full_002 => "vp80-00-comprehensive-002.ivf",
    full_003 => "vp80-00-comprehensive-003.ivf",
    full_004 => "vp80-00-comprehensive-004.ivf",
    full_005 => "vp80-00-comprehensive-005.ivf",
    full_006 => "vp80-00-comprehensive-006.ivf",
    full_007 => "vp80-00-comprehensive-007.ivf",
    full_008 => "vp80-00-comprehensive-008.ivf",
    full_009 => "vp80-00-comprehensive-009.ivf",
    full_010 => "vp80-00-comprehensive-010.ivf",
    full_011 => "vp80-00-comprehensive-011.ivf",
    full_012 => "vp80-00-comprehensive-012.ivf",
    full_013 => "vp80-00-comprehensive-013.ivf",
    full_014 => "vp80-00-comprehensive-014.ivf",
    full_015 => "vp80-00-comprehensive-015.ivf",
    full_016 => "vp80-00-comprehensive-016.ivf",
    full_017 => "vp80-00-comprehensive-017.ivf",
    full_018 => "vp80-00-comprehensive-018.ivf",
    full_intra_1400 => "vp80-01-intra-1400.ivf",
    full_intra_1411 => "vp80-01-intra-1411.ivf",
    full_intra_1416 => "vp80-01-intra-1416.ivf",
    full_intra_1417 => "vp80-01-intra-1417.ivf",
    full_inter_1402 => "vp80-02-inter-1402.ivf",
    full_inter_1412 => "vp80-02-inter-1412.ivf",
    full_inter_1418 => "vp80-02-inter-1418.ivf",
    full_inter_1424 => "vp80-02-inter-1424.ivf",
    full_seg_01 => "vp80-03-segmentation-01.ivf",
    full_seg_02 => "vp80-03-segmentation-02.ivf",
    full_seg_03 => "vp80-03-segmentation-03.ivf",
    full_seg_04 => "vp80-03-segmentation-04.ivf",
    full_seg_1401 => "vp80-03-segmentation-1401.ivf",
    full_seg_1403 => "vp80-03-segmentation-1403.ivf",
    full_seg_1407 => "vp80-03-segmentation-1407.ivf",
    full_seg_1408 => "vp80-03-segmentation-1408.ivf",
    full_seg_1409 => "vp80-03-segmentation-1409.ivf",
    full_seg_1410 => "vp80-03-segmentation-1410.ivf",
    full_seg_1413 => "vp80-03-segmentation-1413.ivf",
    full_seg_1414 => "vp80-03-segmentation-1414.ivf",
    full_seg_1415 => "vp80-03-segmentation-1415.ivf",
    full_seg_1425 => "vp80-03-segmentation-1425.ivf",
    full_seg_1426 => "vp80-03-segmentation-1426.ivf",
    full_seg_1427 => "vp80-03-segmentation-1427.ivf",
    full_seg_1432 => "vp80-03-segmentation-1432.ivf",
    full_seg_1435 => "vp80-03-segmentation-1435.ivf",
    full_seg_1436 => "vp80-03-segmentation-1436.ivf",
    full_seg_1437 => "vp80-03-segmentation-1437.ivf",
    full_seg_1441 => "vp80-03-segmentation-1441.ivf",
    full_seg_1442 => "vp80-03-segmentation-1442.ivf",
    full_part_1404 => "vp80-04-partitions-1404.ivf",
    full_part_1405 => "vp80-04-partitions-1405.ivf",
    full_part_1406 => "vp80-04-partitions-1406.ivf",
    full_sharp_1428 => "vp80-05-sharpness-1428.ivf",
    full_sharp_1429 => "vp80-05-sharpness-1429.ivf",
    full_sharp_1430 => "vp80-05-sharpness-1430.ivf",
    full_sharp_1431 => "vp80-05-sharpness-1431.ivf",
    full_sharp_1433 => "vp80-05-sharpness-1433.ivf",
    full_sharp_1434 => "vp80-05-sharpness-1434.ivf",
    full_sharp_1438 => "vp80-05-sharpness-1438.ivf",
    full_sharp_1439 => "vp80-05-sharpness-1439.ivf",
    full_sharp_1440 => "vp80-05-sharpness-1440.ivf",
    full_sharp_1443 => "vp80-05-sharpness-1443.ivf",
    full_smallsize => "vp80-06-smallsize.ivf",
}
