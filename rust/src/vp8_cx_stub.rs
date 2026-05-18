//! VP8 encoder placeholder.
//!
//! Confirms the `Encoder` trait shape compiles against a concrete
//! implementor. The libvpx build is `--disable-vp8-encoder`; a real
//! port replaces each body.

use crate::codec::{ControlCmd, Encoder, Error, Image};
use crate::vpx_api::VPX_CODEC_INCAPABLE;

pub struct Vp8Encoder;

impl Vp8Encoder {
    pub fn new() -> Self {
        Vp8Encoder
    }
}

impl Default for Vp8Encoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Encoder for Vp8Encoder {
    fn encode(
        &mut self,
        _img: &Image,
        _pts: i64,
        _duration: u64,
        _flags: u32,
    ) -> Result<(), Error> {
        Err(VPX_CODEC_INCAPABLE)
    }

    fn get_cx_data(&mut self) -> Option<&[u8]> {
        None
    }

    fn control(&mut self, _cmd: ControlCmd<'_>) -> Result<(), Error> {
        Err(VPX_CODEC_INCAPABLE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Trait object safety: `Box<dyn Encoder>` must accept this impl.
    #[test]
    fn vp8_encoder_stub_is_object_safe() {
        let _: Box<dyn Encoder> = Box::new(Vp8Encoder::new());
    }
}
