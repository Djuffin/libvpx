use std::sync::Arc;

use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::default_allocator::DefaultAllocator;

use super::support::CountingCallbacks;

#[test]
fn output_queue_starts_empty() {
    let callbacks = CountingCallbacks::shared();
    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, Arc::new(DefaultAllocator), callbacks)
        .expect("create");

    let pic = decoder.get_picture().expect("get_picture");
    assert!(pic.is_none(), "output queue must be empty on fresh initialization");
}
