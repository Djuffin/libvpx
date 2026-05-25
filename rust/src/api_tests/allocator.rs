use std::sync::Arc;

use crate::api::callbacks::DecoderError;
use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::frame::AllocError;
use crate::api::packet::EncodedPacket;

use super::support::{test_data_dir, read_ivf_first_packet, CountingCallbacks, TrackingAllocator};

#[test]
fn propagates_allocator_oom_failure() {
    let Some(dir) = test_data_dir("oom_test") else { return; };
    let packet_data = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));

    let callbacks = CountingCallbacks::shared();
    let allocator = TrackingAllocator::new();
    allocator.set_failure(AllocError::OutOfMemory);

    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, allocator.clone(), callbacks)
        .expect("create");

    let packet = EncodedPacket {
        data: Arc::new(packet_data),
        opaque: None,
    };

    let err = decoder.decode(packet).unwrap_err();
    assert!(
        matches!(err, DecoderError::Alloc(AllocError::OutOfMemory) | DecoderError::Fatal(_)),
        "Expected OOM error, got {err:?}"
    );
}

#[test]
fn propagates_allocator_unsupported_alignment_failure() {
    let Some(dir) = test_data_dir("alignment_test") else { return; };
    let packet_data = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));

    let callbacks = CountingCallbacks::shared();
    let allocator = TrackingAllocator::new();
    allocator.set_failure(AllocError::UnsupportedAlignment);

    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, allocator.clone(), callbacks)
        .expect("create");

    let packet = EncodedPacket {
        data: Arc::new(packet_data),
        opaque: None,
    };

    let err = decoder.decode(packet).unwrap_err();
    assert!(
        matches!(err, DecoderError::Alloc(AllocError::UnsupportedAlignment) | DecoderError::Fatal(_)),
        "Expected alignment error, got {err:?}"
    );
}
