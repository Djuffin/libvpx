use std::sync::Arc;

use crate::api::callbacks::DecoderError;
use crate::api::config::{Codec, DecoderConfig, LatencyMode};
use crate::api::create_decoder;
use crate::api::DefaultAllocator;

use super::support::CountingCallbacks;

#[test]
fn create_decoder_with_default_config_succeeds() {
    let result = create_decoder(
        DecoderConfig::new(Codec::VP8),
        Arc::new(DefaultAllocator),
        CountingCallbacks::shared(),
    );
    assert!(result.is_ok());
}

#[test]
fn create_decoder_low_latency_accepted() {
    let config = DecoderConfig::new(Codec::VP8).with_latency_mode(LatencyMode::LowLatency);
    let result = create_decoder(config, Arc::new(DefaultAllocator), CountingCallbacks::shared());
    assert!(result.is_ok(), "LowLatency must be accepted even if not honored specially");
}

#[test]
fn decoder_config_builder_chain_works() {
    let config = DecoderConfig::new(Codec::VP8)
        .with_latency_mode(LatencyMode::Throughput);
    assert_eq!(config.codec, Codec::VP8);
    assert_eq!(config.latency_mode, LatencyMode::Throughput);
}

#[test]
fn create_decoder_returns_box_dyn_video_decoder() {
    let decoder = create_decoder(
        DecoderConfig::new(Codec::VP8),
        Arc::new(DefaultAllocator),
        CountingCallbacks::shared(),
    )
    .expect("create");
    let mut decoder = decoder;
    assert!(decoder.get_picture().unwrap().is_none());
}

#[test]
fn create_decoder_unsupported_codecs_table() {
    for codec in [Codec::H264, Codec::VP9, Codec::AV1, Codec::AV2] {
        let result = create_decoder(
            DecoderConfig::new(codec),
            Arc::new(DefaultAllocator),
            CountingCallbacks::shared(),
        );
        let err = result.map(|_| ()).unwrap_err();
        assert!(
            matches!(err, DecoderError::FeatureNotSupported(_)),
            "{codec:?} should return FeatureNotSupported, got {err:?}",
        );
    }
}
