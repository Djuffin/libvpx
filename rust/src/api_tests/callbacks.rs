use std::sync::Arc;

use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::default_allocator::DefaultAllocator;
use crate::api::packet::EncodedPacket;

use super::support::{test_data_dir, read_ivf_first_packet, CountingCallbacks};

#[test]
fn callbacks_fire_on_keyframe() {
    let Some(dir) = test_data_dir("callbacks_test") else { return; };
    let packet_data = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));

    let callbacks = CountingCallbacks::shared();
    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, Arc::new(DefaultAllocator), callbacks.clone())
        .expect("create");

    assert_eq!(callbacks.picture_callbacks(), 0);
    assert_eq!(callbacks.format_change_count(), 0);

    let packet = EncodedPacket {
        data: Arc::new(packet_data),
        opaque: None,
    };

    decoder.decode(packet).expect("decode");

    assert_eq!(callbacks.format_change_count(), 1, "format must change on first keyframe");
    assert_eq!(callbacks.picture_callbacks(), 1, "keyframe must emit exactly one frame available");

    let format = callbacks.last_format.lock().unwrap().clone().unwrap();
    assert_eq!(format.display_width, 176);
    assert_eq!(format.display_height, 144);
}
