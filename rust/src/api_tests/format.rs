use std::sync::Arc;

use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::default_allocator::DefaultAllocator;
use crate::api::packet::EncodedPacket;

use super::support::{test_data_dir, read_ivf_first_packet, CountingCallbacks};

#[test]
fn format_signaled_accurately() {
    let Some(dir) = test_data_dir("format_test") else { return; };
    let packet_data = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));

    let callbacks = CountingCallbacks::shared();
    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, Arc::new(DefaultAllocator), callbacks.clone())
        .expect("create");

    let packet = EncodedPacket {
        data: Arc::new(packet_data),
        opaque: None,
    };

    decoder.decode(packet).expect("decode");
    
    let pic = decoder.get_picture().expect("get_picture")
        .expect("picture must be available");

    assert_eq!(pic.format.display_width, 176);
    assert_eq!(pic.format.display_height, 144);
    assert!(pic.format.color_space.is_some());
}
