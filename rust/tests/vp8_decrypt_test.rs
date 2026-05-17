//! Port of `test/vp8_decrypt_test.cc` to a Rust integration test.
//!
//! C: `TEST(TestDecrypt, DecryptWorksVp8)`. Decode the first frame of
//! `vp80-00-comprehensive-001.ivf` plain, then XOR-encrypt the second
//! frame with a fixed 16-byte key, install a decryption callback via
//! `VPXD_SET_DECRYPTOR`, and verify the second frame still decodes
//! cleanly (callback is invoked on every byte the decoder reads).

use core::ffi::c_void;
use core::mem::MaybeUninit;
use core::ptr;
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::PathBuf;

use vp8_decoder_rs::vp8_dx_iface::{VpxDecryptInit, VPXD_SET_DECRYPTOR};
use vp8_decoder_rs::vp8_dx_iface::vpx_codec_vp8_dx;
use vp8_decoder_rs::vpx_api::{
    vpx_codec_control_, vpx_codec_ctx_t, vpx_codec_dec_init_ver, vpx_codec_decode,
    vpx_codec_destroy, vpx_codec_iface_t, VPX_CODEC_OK, VPX_DECODER_ABI_VERSION,
};

const TEST_DATA_DIR: &str = "/home/eugene/projects/libvpx/vp8_only";

/// XOR key used by the test (C: `test_key`).
#[rustfmt::skip]
const TEST_KEY: [u8; 16] = [
    0x01, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78,
    0x89, 0x9a, 0xab, 0xbc, 0xcd, 0xde, 0xef, 0xf0,
];

fn encrypt_buffer(src: &[u8], dst: &mut [u8], offset: usize) {
    for i in 0..src.len() {
        dst[i] = src[i] ^ TEST_KEY[(offset + i) & 15];
    }
}

/// C: `test_decrypt_cb`. Called by the decoder for every contiguous
/// chunk it reads from the bytestream. `decrypt_state` carries a
/// pointer to byte 0 of the encrypted buffer so we can recover the
/// byte offset of `input` for keying.
unsafe extern "C" fn test_decrypt_cb(
    decrypt_state: *mut c_void,
    input: *const u8,
    output: *mut u8,
    count: i32,
) {
    let base = decrypt_state as *const u8;
    let offset = input.offset_from(base) as usize;
    let n = count as usize;
    let in_slice = core::slice::from_raw_parts(input, n);
    let out_slice = core::slice::from_raw_parts_mut(output, n);
    encrypt_buffer(in_slice, out_slice, offset);
}

/// Minimal IVF reader (one helper at a time — easier than sharing a
/// crate-internal module).
struct Ivf {
    inner: BufReader<File>,
}
impl Ivf {
    fn open(p: &PathBuf) -> Self {
        let mut inner = BufReader::new(File::open(p).expect("open ivf"));
        let mut hdr = [0u8; 32];
        inner.read_exact(&mut hdr).expect("ivf hdr");
        Self { inner }
    }
    fn next_packet(&mut self) -> Option<Vec<u8>> {
        let mut h = [0u8; 12];
        if self.inner.read_exact(&mut h).is_err() {
            return None;
        }
        let n = u32::from_le_bytes([h[0], h[1], h[2], h[3]]) as usize;
        let mut buf = vec![0u8; n];
        self.inner.read_exact(&mut buf).ok()?;
        Some(buf)
    }
}

#[test]
fn decrypt_works_vp8() {
    unsafe {
        let path = PathBuf::from(TEST_DATA_DIR).join("vp80-00-comprehensive-001.ivf");
        let mut video = Ivf::open(&path);

        let iface = vpx_codec_vp8_dx() as *mut vpx_codec_iface_t;
        let mut dec_uninit = MaybeUninit::<vpx_codec_ctx_t>::uninit();
        assert_eq!(
            vpx_codec_dec_init_ver(
                dec_uninit.as_mut_ptr(),
                iface,
                ptr::null(),
                0,
                VPX_DECODER_ABI_VERSION,
            ),
            VPX_CODEC_OK,
        );
        let mut dec = dec_uninit.assume_init();

        // Frame 0: plain decode (sanity).
        let frame0 = video.next_packet().expect("frame 0");
        assert_eq!(
            vpx_codec_decode(
                &mut dec,
                frame0.as_ptr(),
                frame0.len() as u32,
                ptr::null_mut(),
                0
            ),
            VPX_CODEC_OK,
        );

        // Frame 1: encrypt, install decryptor, decode through the cb.
        let frame1 = video.next_packet().expect("frame 1");
        let mut encrypted = vec![0u8; frame1.len()];
        encrypt_buffer(&frame1, &mut encrypted, 0);

        // The C `decrypt_state` doubles as the pointer marking "byte 0"
        // of the original buffer; the cb subtracts that pointer from
        // each `input` to get the byte offset into the encrypted stream.
        let mut di = VpxDecryptInit {
            decrypt_cb: Some(test_decrypt_cb),
            decrypt_state: encrypted.as_mut_ptr() as *mut c_void,
        };
        assert_eq!(
            vpx_codec_control_(
                &mut dec,
                VPXD_SET_DECRYPTOR,
                &mut di as *mut VpxDecryptInit as *mut c_void,
            ),
            VPX_CODEC_OK,
        );

        assert_eq!(
            vpx_codec_decode(
                &mut dec,
                encrypted.as_ptr(),
                encrypted.len() as u32,
                ptr::null_mut(),
                0
            ),
            VPX_CODEC_OK,
            "encrypted frame failed to decode through cb",
        );

        assert_eq!(vpx_codec_destroy(&mut dec), VPX_CODEC_OK);
    }
}
