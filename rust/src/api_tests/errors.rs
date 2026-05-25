use std::sync::Arc;

use crate::api::callbacks::DecoderError;
use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::default_allocator::DefaultAllocator;
use crate::api::packet::EncodedPacket;

use super::support::{test_data_dir, CountingCallbacks};

#[test]
fn reports_corrupted_bitstream_error() {
    let Some(_dir) = test_data_dir("errors_test") else { return; };
    
    // Corrupt/bogus 10-byte payload that is not a keyframe
    let bogus_data = vec![0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a];

    let callbacks = CountingCallbacks::shared();
    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, Arc::new(DefaultAllocator), callbacks)
        .expect("create");

    let packet = EncodedPacket {
        data: Arc::new(bogus_data),
        opaque: None,
    };

    let err = decoder.decode(packet).unwrap_err();
    assert!(
        matches!(err, DecoderError::MisformedData(_) | DecoderError::OutOfRange(_)),
        "Expected corrupt bitstream error, got {err:?}"
    );
}
