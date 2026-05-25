use std::any::Any;
use std::sync::Arc;

use super::callbacks::{DecoderError, VideoDecoderCallbacks};
use super::config::{Codec, DecoderConfig};
use super::frame::VideoFrameAllocator;
use super::packet::{DecodedPicture, EncodedPacket};

/// Modes for flushing the decoder pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlushMode {
    /// Fast discard: instantly clears input/output queues and DPB.
    /// In-flight thread work is allowed to finish but its results are
    /// discarded. Used immediately when seeking in a video player.
    Discard,

    /// Drain pipeline: forces DPB to release all remaining frames to
    /// the output queue. Does NOT stop the decoder from accepting new
    /// inputs afterwards. Used at End of Stream (EOS) or sequence
    /// boundaries.
    Drain,
}

/// Codec-specific control payload.
pub type ControlCmd = dyn Any;

/// Codec-agnostic software video decoder interface.
pub trait VideoDecoder: Send {
    /// Submit an encoded packet to the decoder's input queue.
    fn decode(&mut self, packet: EncodedPacket) -> Result<(), DecoderError>;

    /// Pull the next decoded picture from the output queue, or
    /// `Ok(None)` if the queue is empty.
    fn get_picture(&mut self) -> Result<Option<DecodedPicture>, DecoderError>;

    /// Flushes the decoder pipeline according to the specified `FlushMode`.
    fn flush(&mut self, mode: FlushMode) -> Result<(), DecoderError>;

    /// Dispatch a codec-specific command.
    fn control(&mut self, cmd: &mut ControlCmd) -> Result<(), DecoderError>;
}

/// The primary entry point to instantiate a software video decoder.
pub fn create_decoder(
    config: DecoderConfig,
    allocator: Arc<dyn VideoFrameAllocator>,
    callback: Arc<dyn VideoDecoderCallbacks>,
) -> Result<Box<dyn VideoDecoder>, DecoderError> {
    match config.codec {
        Codec::VP8 => {
            Ok(Box::new(crate::vp8_dx_iface::Vp8VideoDecoder::new(
                config, allocator, callback,
            )?))
        }
        other => Err(DecoderError::FeatureNotSupported(format!(
            "codec {other:?} is not supported by this decoder library"
        ))),
    }
}
