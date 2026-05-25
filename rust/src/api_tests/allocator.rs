use std::sync::Arc;

use crate::api::callbacks::DecoderError;
use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::frame::AllocError;
use crate::api::packet::EncodedPacket;

use super::support::{test_data_dir, read_ivf_first_packet, CountingCallbacks, TrackingAllocator, IvfReader};

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

#[test]
fn recycles_dropped_buffers() {
    let Some(dir) = test_data_dir("pool_test") else { return; };
    let name = "vp80-00-comprehensive-001.ivf";
    let mut reader = IvfReader::open(&dir.join(name)).expect("open ivf");

    let callbacks = CountingCallbacks::shared();
    let allocator = super::support::PoolAllocator::new();

    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, allocator.clone(), callbacks)
        .expect("create");

    // Decode 5 frames, dropping each completed picture immediately:
    for _ in 0..5 {
        if let Some(packet_data) = reader.next_frame() {
            let packet = EncodedPacket {
                data: Arc::new(packet_data),
                opaque: None,
            };
            decoder.decode(packet).expect("decode failed");

            // Retrieve and immediately drop/discard the picture:
            let pic = decoder.get_picture().expect("get_picture failed");
            assert!(pic.is_some());
            // pic goes out of scope here and drops, recycling its buffer!
        }
    }

    // The DPB slots (4 slots) + temp scale frame (1 slot) + output staging (1 slot)
    // mean the allocator count should stabilize to no more than 6 fresh allocations total!
    let initial_allocs = allocator.count();
    assert!(initial_allocs > 0 && initial_allocs <= 6, "expected small fresh allocation count, got {initial_allocs}");

    // Decode 3 more frames. Since we recycle, no new fresh allocations should be triggered!
    for _ in 0..3 {
        if let Some(packet_data) = reader.next_frame() {
            let packet = EncodedPacket {
                data: Arc::new(packet_data),
                opaque: None,
            };
            decoder.decode(packet).expect("decode failed");
            let _pic = decoder.get_picture().expect("get_picture failed");
        }
    }

    let final_allocs = allocator.count();
    assert_eq!(final_allocs, initial_allocs, "Memory pool failed: fresh allocations occurred instead of recycling!");
}

