use std::sync::Arc;

use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::default_allocator::DefaultAllocator;
use crate::api::decoder::FlushMode;
use crate::api::packet::EncodedPacket;

use super::support::{test_data_dir, read_ivf_first_packet, CountingCallbacks};

#[test]
fn flush_discard_clears_queues() {
    let Some(dir) = test_data_dir("flush_test") else { return; };
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
    
    // Perform fast discard
    decoder.flush(FlushMode::Discard).expect("flush");

    // Output queue should be empty now
    let pic = decoder.get_picture().expect("get_picture");
    assert!(pic.is_none(), "output queue must be empty after Discard flush");
}
