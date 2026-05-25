use std::sync::Arc;

use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::default_allocator::DefaultAllocator;
use crate::api::packet::EncodedPacket;

use super::support::{test_data_dir, read_ivf_first_packet, CountingCallbacks};

#[test]
fn opaque_tag_propagates_correctly() {
    let Some(dir) = test_data_dir("opaque_test") else { return; };
    let packet_data = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));

    let callbacks = CountingCallbacks::shared();
    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, Arc::new(DefaultAllocator), callbacks)
        .expect("create");

    let opaque_tag: usize = 0x55AA_EEFF;
    let packet = EncodedPacket {
        data: Arc::new(packet_data),
        opaque: Some(Box::new(opaque_tag)),
    };

    decoder.decode(packet).expect("decode");
    
    let pic = decoder.get_picture().expect("get_picture")
        .expect("picture available");

    let tag = pic.opaque.expect("missing opaque tag")
        .downcast_ref::<usize>().cloned();
    assert_eq!(tag, Some(opaque_tag), "opaque tag corrupted during decode");
}
