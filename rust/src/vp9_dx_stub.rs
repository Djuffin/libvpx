//! VP9 decoder placeholder.
//!
//! Validates that the `Decoder` trait shape can express VP9 without
//! changes. Every method returns `Err(Error::Incapable)` — a real
//! port would replace these bodies with calls into the VP9 stack.

use core::time::Duration;

use crate::codec::{ControlCmd, Decoder, Error, Image, StreamInfo};
use crate::vpx_api::{VPX_CODEC_INCAPABLE, VpxCodecStreamInfo};

/// Phantom VP9 decoder. No state; constructible to confirm the trait
/// signature accepts it.
pub struct Vp9Decoder;

impl Vp9Decoder {
    pub fn new() -> Self {
        Vp9Decoder
    }
}

impl Default for Vp9Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder for Vp9Decoder {
    fn decode(&mut self, _data: &[u8], _deadline: Duration) -> Result<(), Error> {
        Err(VPX_CODEC_INCAPABLE)
    }

    fn get_frame(&mut self) -> Option<&Image> {
        None
    }

    fn control(&mut self, _cmd: ControlCmd<'_>) -> Result<(), Error> {
        Err(VPX_CODEC_INCAPABLE)
    }

    fn peek_stream_info(_data: &[u8]) -> Result<StreamInfo, Error> {
        Err(VPX_CODEC_INCAPABLE)
    }

    fn stream_info(&self) -> Result<StreamInfo, Error> {
        // Return a zero StreamInfo just to exercise the path.
        Ok(VpxCodecStreamInfo {
            sz: 0,
            w: 0,
            h: 0,
            is_kf: 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Trait object safety: `Box<dyn Decoder>` must accept this impl.
    #[test]
    fn vp9_stub_is_object_safe() {
        let _: Box<dyn Decoder> = Box::new(Vp9Decoder::new());
    }
}
