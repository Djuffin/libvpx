use std::sync::Arc;

use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::default_allocator::DefaultAllocator;
use crate::api::packet::EncodedPacket;
use crate::api::VideoFrame;

use super::support::{test_data_dir, read_ivf_first_packet, read_md5_lines, CountingCallbacks};
use md5::{Digest, Md5};

fn md5_of_frame(frame: &dyn VideoFrame) -> String {
    let mut hasher = Md5::new();
    let planes = frame.planes();

    for plane_idx in 0..3 {
        let plane_view = planes[plane_idx].as_ref().expect("plane must be present");
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

#[test]
fn decode_first_keyframe_and_verify_md5() {
    let Some(dir) = test_data_dir("data_path_test") else { return; };
    let name = "vp80-00-comprehensive-001.ivf";
    let packet_data = read_ivf_first_packet(&dir.join(name));
    let expected = read_md5_lines(&dir.join(format!("{name}.md5")));

    let callbacks = CountingCallbacks::shared();
    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, Arc::new(DefaultAllocator), callbacks)
        .expect("create");

    let packet = EncodedPacket {
        data: Arc::new(packet_data),
        opaque: None,
    };

    decoder.decode(packet).expect("decode");
    
    let pic = decoder.get_picture().expect("get_picture")
        .expect("expected one decoded picture");

    let got = md5_of_frame(pic.frame.as_ref());
    assert_eq!(got, expected[0], "conformance MD5 mismatch on data_path test");
}
