use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use md5::{Digest, Md5};

use vp8_decoder_rs::api::*;

const TEST_DATA_DIR: &str = env!("VP8_TEST_DATA_DIR");

fn data_dir() -> Option<PathBuf> {
    if TEST_DATA_DIR.is_empty() {
        return None;
    }
    Some(PathBuf::from(TEST_DATA_DIR))
}

fn md5_of_frame(frame: &dyn VideoFrame) -> String {
    let mut hasher = Md5::new();
    let planes = frame.planes();

    for plane_idx in 0..3 {
        let plane_view = planes[plane_idx].as_ref().expect("planar plane must be present");
        let stride = plane_view.stride;
        let w = plane_view.width;
        let h = plane_view.height;
        
        let mut offset = 0;
        for _ in 0..h {
            let row = &plane_view.data[offset..offset + w];
            hasher.update(row);
            offset += stride;
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
    let size =
        u32::from_le_bytes([frame_hdr[0], frame_hdr[1], frame_hdr[2], frame_hdr[3]]) as usize;
    let mut buf = vec![0u8; size];
    f.read_exact(&mut buf).expect("read frame");
    buf
}

struct TestCallbacks {
    picture_available: Mutex<usize>,
    format_changed: Mutex<Option<StreamFormat>>,
}

impl VideoDecoderCallbacks for TestCallbacks {
    fn on_picture_available(&self) {
        let mut count = self.picture_available.lock().unwrap();
        *count += 1;
    }

    fn on_format_changed(&self, format: StreamFormat) {
        let mut fmt = self.format_changed.lock().unwrap();
        *fmt = Some(format);
    }
}

#[test]
fn unified_api_decodes_first_keyframe() {
    let Some(dir) = data_dir() else {
        eprintln!("skipping: VP8 test data not available");
        return;
    };
    let name = "vp80-00-comprehensive-001.ivf";
    let packet_data = read_ivf_first_packet(&dir.join(name));
    let expected = first_md5(&dir.join(format!("{name}.md5")));

    let callbacks = Arc::new(TestCallbacks {
        picture_available: Mutex::new(0),
        format_changed: Mutex::new(None),
    });

    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, Arc::new(DefaultAllocator), callbacks.clone())
        .expect("create_decoder failed");

    let opaque_tag: usize = 0x1234_5678;
    let packet = EncodedPacket {
        data: Arc::new(packet_data),
        opaque: Some(Box::new(opaque_tag)),
    };

    decoder.decode(packet).expect("decode failed");

    // Verify callbacks were invoked correctly
    let format_opt = callbacks.format_changed.lock().unwrap().clone();
    assert!(format_opt.is_some(), "on_format_changed was not triggered");
    let format = format_opt.unwrap();
    assert_eq!(format.codec, Codec::VP8);
    assert_eq!(format.display_width, 176);
    assert_eq!(format.display_height, 144);

    let pic_count = *callbacks.picture_available.lock().unwrap();
    assert_eq!(pic_count, 1, "on_picture_available was not triggered exactly once");

    // Retrieve picture and assert on checksum and opaque metadata propagation
    let decoded_pic = decoder.get_picture().expect("get_picture failed")
        .expect("no decoded picture returned");

    let tag_ref = decoded_pic.opaque.expect("propagated opaque tag missing")
        .downcast_ref::<usize>().cloned();
    assert_eq!(tag_ref, Some(opaque_tag), "opaque tag was corrupted/missing");

    let got_checksum = md5_of_frame(decoded_pic.frame.as_ref());
    assert_eq!(got_checksum, expected, "Unified-API-path decoded MD5 mismatch");
}
