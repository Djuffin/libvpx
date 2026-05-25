use std::sync::Arc;

use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::default_allocator::DefaultAllocator;
use crate::api::packet::EncodedPacket;
use crate::api::VideoPlane;

use super::support::{test_data_dir, read_ivf_first_packet, CountingCallbacks};

#[test]
fn plane_geometries_match_stream() {
    let Some(dir) = test_data_dir("frame_test") else { return; };
    let packet_data = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));

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
        .expect("picture available");

    let y_plane = pic.frame.plane(VideoPlane::Y).expect("Y plane");
    assert_eq!(y_plane.width, 176);
    assert_eq!(y_plane.height, 144);

    let u_plane = pic.frame.plane(VideoPlane::U).expect("U plane");
    assert_eq!(u_plane.width, 88);
    assert_eq!(u_plane.height, 72);

    let v_plane = pic.frame.plane(VideoPlane::V).expect("V plane");
    assert_eq!(v_plane.width, 88);
    assert_eq!(v_plane.height, 72);
}
