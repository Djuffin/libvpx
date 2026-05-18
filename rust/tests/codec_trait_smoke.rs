//! Smoke test for the `Decoder` trait. Decodes the first packet of
//! the first conformance vector through the trait API and confirms
//! the per-frame MD5 matches the canonical one.

use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::PathBuf;
use std::time::Duration;

use md5::{Digest, Md5};

use vp8_decoder_rs::codec::Decoder;
use vp8_decoder_rs::vp8_dx_iface::Vp8Decoder;
use vp8_decoder_rs::vpx_api::{vpx_image_t, VPX_IMG_FMT_HIGHBITDEPTH};

const TEST_DATA_DIR: &str = env!("VP8_TEST_DATA_DIR");

fn data_dir() -> Option<PathBuf> {
    if TEST_DATA_DIR.is_empty() {
        return None;
    }
    Some(PathBuf::from(TEST_DATA_DIR))
}

fn md5_of_image(img: &vpx_image_t) -> String {
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
        unsafe {
            for _ in 0..h {
                let row = core::slice::from_raw_parts(buf, w);
                hasher.update(row);
                buf = buf.add(img.stride[plane] as usize);
            }
        }
    }

    let digest = hasher.finalize();
    let mut s = String::with_capacity(32);
    for b in digest {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

fn first_md5(path: &PathBuf) -> String {
    let f = File::open(path).expect("open .md5 file");
    BufReader::new(f)
        .lines()
        .next()
        .expect("at least one md5 line")
        .expect("read line")
        .split_whitespace()
        .next()
        .expect("md5 hex")
        .to_owned()
}

fn read_ivf_first_packet(path: &PathBuf) -> Vec<u8> {
    let mut f = BufReader::new(File::open(path).expect("open ivf"));
    let mut hdr = [0u8; 32];
    f.read_exact(&mut hdr).expect("read ivf header");
    assert_eq!(&hdr[0..4], b"DKIF");
    let mut frame_hdr = [0u8; 12];
    f.read_exact(&mut frame_hdr).expect("read frame header");
    let size = u32::from_le_bytes([frame_hdr[0], frame_hdr[1], frame_hdr[2], frame_hdr[3]]) as usize;
    let mut buf = vec![0u8; size];
    f.read_exact(&mut buf).expect("read frame");
    buf
}

/// Decode the first keyframe of `comprehensive-001` through the trait
/// and verify the MD5 matches the canonical one.
#[test]
fn trait_decodes_first_keyframe() {
    let Some(dir) = data_dir() else {
        eprintln!("skipping: VP8 test data not available");
        return;
    };
    let name = "vp80-00-comprehensive-001.ivf";
    let packet = read_ivf_first_packet(&dir.join(name));
    let expected = first_md5(&dir.join(format!("{name}.md5")));

    let mut decoder: Box<dyn Decoder> =
        Box::new(Vp8Decoder::new(0).expect("decoder init"));

    decoder.decode(&packet, Duration::ZERO).expect("decode");
    let img = decoder.get_frame().expect("got frame");
    let got = md5_of_image(img);
    assert_eq!(got, expected, "trait-path MD5 mismatch");
}

/// `set_user_priv` round-trips an opaque pointer to the next emitted
/// `Image.user_priv` field.
#[test]
fn user_priv_round_trip() {
    let Some(dir) = data_dir() else {
        eprintln!("skipping: VP8 test data not available");
        return;
    };
    let packet = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));

    let mut decoder: Box<dyn Decoder> =
        Box::new(Vp8Decoder::new(0).expect("decoder init"));

    let tag: usize = 0xDEAD_BEEF;
    decoder.set_user_priv(tag as *mut core::ffi::c_void);
    decoder.decode(&packet, Duration::ZERO).expect("decode");
    let img = decoder.get_frame().expect("got frame");
    assert_eq!(img.user_priv as usize, tag, "user_priv did not round-trip");
}

/// Sanity check: peek_stream_info returns plausible width/height.
#[test]
fn peek_stream_info_keyframe() {
    let Some(dir) = data_dir() else {
        eprintln!("skipping: VP8 test data not available");
        return;
    };
    let packet = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));
    let si = Vp8Decoder::peek_stream_info(&packet).expect("peek");
    assert!(si.w > 0 && si.h > 0);
    assert_eq!(si.is_kf, 1);
}
