//! Idiomatic Rust codec interface.
//!
//! Two top-level traits: [`Decoder`] and [`Encoder`]. A codec may
//! implement either or both. There is intentionally no `Codec`
//! supertrait — `decode` and `encode` have no common signature.

use core::time::Duration;

use crate::vp8_dx_iface::{Vp8PostprocCfg, VpxDecryptInit, VpxRefFrame};

// Re-exports under idiomatic short names.
pub use crate::vpx_api::{
    VpxCodecErr as Error, VpxCodecStreamInfo as StreamInfo, VpxImage as Image,
};

/// Frame-buffer allocator hook. Codecs that need externally-supplied
/// frame buffers accept any implementor via a builder.
///
/// The default implementation in the decoder pulls buffers from an
/// internal heap pool — equivalent to the libvpx behavior when no
/// `vpx_codec_set_frame_buffer_functions` is called.
pub trait FrameBufferAllocator {
    /// Acquire a frame buffer of at least `min_size` bytes. Returns
    /// `Err(Error::MemError)` on failure.
    fn get(&mut self, min_size: usize) -> Result<FrameBufferHandle, Error>;

    /// Release a previously-acquired buffer.
    fn release(&mut self, handle: FrameBufferHandle) -> Result<(), Error>;
}

/// Handle to a frame buffer issued by a [`FrameBufferAllocator`].
///
/// Wraps the C `vpx_codec_frame_buffer_t` shape (data pointer + size +
/// private slot). Fields are `pub` so allocator implementations can
/// populate them directly; ownership of the underlying allocation
/// remains with the allocator.
#[derive(Copy, Clone)]
pub struct FrameBufferHandle {
    pub data: *mut u8,
    pub size: usize,
    pub priv_: *mut core::ffi::c_void,
}

/// Decoder control commands. Replaces the libvpx `vpx_codec_control_`
/// + `va_list` dispatch with a typed enum.
///
/// `#[non_exhaustive]` so that adding VP9-specific controls in the
/// future does not break user `match` arms.
#[non_exhaustive]
pub enum ControlCmd<'a> {
    /// Replace a reference frame (last / golden / altref) with the
    /// supplied image. `VP8_SET_REFERENCE` in C.
    SetReference(&'a VpxRefFrame),

    /// Copy a reference frame out to the supplied image buffer.
    /// `VP8_COPY_REFERENCE` in C.
    CopyReference(&'a mut VpxRefFrame),

    /// Apply post-processing config. `VP8_SET_POSTPROC` in C. The
    /// minimal build returns `Err(Error::Incapable)` for this.
    SetPostproc(Vp8PostprocCfg),

    /// Output: bitmask of which references the last frame refreshed.
    /// `VP8D_GET_LAST_REF_UPDATES` in C.
    GetLastRefUpdates(&'a mut i32),

    /// Output: 1 if the last decoded frame was corrupt.
    /// `VP8D_GET_FRAME_CORRUPTED` in C.
    GetFrameCorrupted(&'a mut i32),

    /// Output: bitmask of which references the last frame used.
    /// `VP8D_GET_LAST_REF_USED` in C.
    GetLastRefUsed(&'a mut i32),

    /// Output: last-frame quantizer value. `VPXD_GET_LAST_QUANTIZER` in C.
    GetLastQuantizer(&'a mut i32),

    /// Install (or clear) the per-frame decryption callback.
    /// `VPXD_SET_DECRYPTOR` in C. `None` clears.
    SetDecryptor(Option<&'a VpxDecryptInit>),
}

/// Decoder trait. One frame in, zero-or-one frame out.
///
/// Stateful — implementors maintain reference buffers and probability
/// tables across calls.
pub trait Decoder {
    /// Submit a compressed frame to the decoder. May internally
    /// produce an image retrievable via [`Decoder::get_frame`].
    ///
    /// `data` may be empty to signal end-of-stream / flush.
    /// `deadline` is advisory — VP8 always decodes fully and ignores
    /// the value; provided for VP9 compatibility.
    fn decode(&mut self, data: &[u8], deadline: Duration) -> Result<(), Error>;

    /// Retrieve the next decoded image, if any. Returns `None` when
    /// no frame is available.
    ///
    /// The returned reference is valid until the next `decode` call.
    fn get_frame(&mut self) -> Option<&Image>;

    /// Dispatch a typed control command.
    fn control(&mut self, cmd: ControlCmd<'_>) -> Result<(), Error>;

    /// Probe a key-frame header for stream parameters without
    /// committing to decode. Static — does not need a decoder
    /// instance. `Self: Sized` so it does not interfere with object
    /// safety.
    fn peek_stream_info(data: &[u8]) -> Result<StreamInfo, Error>
    where
        Self: Sized;

    /// Stream parameters of the most-recently-decoded frame.
    fn stream_info(&self) -> Result<StreamInfo, Error>;

    /// Flush any buffered fragments. Default implementation submits
    /// an empty buffer.
    fn flush(&mut self) -> Result<(), Error> {
        self.decode(&[], Duration::ZERO)
    }
}

/// Encoder trait. VP8/VP9 encoder implementations are out-of-scope
/// for the current build (`--disable-vp8-encoder`, `--disable-vp9`);
/// the trait exists for shape validation against future `Vp8Encoder` /
/// `Vp9Encoder` ports.
pub trait Encoder {
    /// Submit an uncompressed image for encoding.
    fn encode(
        &mut self,
        img: &Image,
        pts: i64,
        duration: u64,
        flags: u32,
    ) -> Result<(), Error>;

    /// Retrieve the next compressed packet, if any.
    fn get_cx_data(&mut self) -> Option<&[u8]>;

    /// Dispatch a typed control command.
    fn control(&mut self, cmd: ControlCmd<'_>) -> Result<(), Error>;
}
