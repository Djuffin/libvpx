//! Codec-agnostic video decoder API.

pub mod callbacks;
pub mod color;
pub mod config;
pub mod decoder;
pub mod default_allocator;
pub mod format;
pub mod frame;
pub mod packet;

pub use callbacks::{DecoderError, VideoDecoderCallbacks};
pub use decoder::{create_decoder, ControlCmd, FlushMode, VideoDecoder};
pub use default_allocator::DefaultAllocator;
pub use color::{
    ColorPrimaries, ColorRange, ColorSpace, MatrixCoefficients, PixelFormat,
    TransferCharacteristics, VideoPlane,
};
pub use config::{Codec, DecoderConfig, LatencyMode};
pub use format::StreamFormat;
pub use frame::{
    AllocError, BufferAllocation, FrameBuffer, PlaneAllocation, PlaneView, VideoFrame,
    VideoFrameAllocator,
};
pub use packet::{DecodedPicture, EncodedPacket};

// ===========================================================================
// VP8 Codec-Specific Control Commands
// ===========================================================================

pub struct Vp8SetReference {
    pub frame_type: i32,
    pub img: crate::vpx_api::VpxImage,
}

pub struct Vp8CopyReference {
    pub frame_type: i32,
    pub img: std::cell::Cell<crate::vpx_api::VpxImage>,
}

pub struct Vp8GetLastRefUpdates {
    pub out: std::cell::Cell<i32>,
}

pub struct Vp8GetFrameCorrupted {
    pub out: std::cell::Cell<i32>,
}

pub struct Vp8GetLastRefUsed {
    pub out: std::cell::Cell<i32>,
}

pub struct Vp8GetLastQuantizer {
    pub out: std::cell::Cell<i32>,
}

