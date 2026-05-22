#![allow(unsafe_op_in_unsafe_fn)]
//! Round-trip test for caller-supplied reference frames via
//! `VP8_SET_REFERENCE` / `VP8_COPY_REFERENCE`.
//!
//! This guards the external "caller provides their own buffer" contract:
//! the decoder must accept a frame whose plane memory the caller owns
//! (SET), copy it into a decoder-owned DPB slot, and later copy it back
//! out into another caller-owned buffer (COPY). Both directions go
//! through `image2yuvconfig`, which *aliases* the caller's planes into a
//! transient `Yv12BufferConfig` whose `owning_buffer` is `None` — the
//! borrowed, never-freed config kind. If `Yv12BufferConfig` were made to
//! own its slab unconditionally, this test would fail (double-free /
//! wrong copy).

use core::ffi::c_void;
use core::mem::MaybeUninit;
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::PathBuf;

use vp8_decoder_rs::types::VP8_LAST_FRAME;
use vp8_decoder_rs::vp8_dx_iface::{
    VP8_COPY_REFERENCE, VP8_SET_REFERENCE, VpxRefFrame, vpx_codec_vp8_dx,
};
use vp8_decoder_rs::vpx_api::{
    VPX_CODEC_OK, VPX_DECODER_ABI_VERSION, VPX_IMG_FMT_I420, VPX_PLANE_U, VPX_PLANE_V, VPX_PLANE_Y,
    VpxImage, vpx_codec_control_, vpx_codec_ctx_t, vpx_codec_dec_init_ver, vpx_codec_decode,
    vpx_codec_destroy, vpx_codec_get_frame,
};

const TEST_DATA_DIR: &str = env!("VP8_TEST_DATA_DIR");

fn data_dir() -> Option<PathBuf> {
    if TEST_DATA_DIR.is_empty() {
        return None;
    }
    Some(PathBuf::from(TEST_DATA_DIR))
}

fn read_ivf_first_packet(path: &PathBuf) -> Vec<u8> {
    let mut f = BufReader::new(File::open(path).expect("open ivf"));
    let mut hdr = [0u8; 32];
    f.read_exact(&mut hdr).expect("read ivf header");
    assert_eq!(&hdr[0..4], b"DKIF");
    let mut frame_hdr = [0u8; 12];
    f.read_exact(&mut frame_hdr).expect("read frame header");
    let size =
        u32::from_le_bytes([frame_hdr[0], frame_hdr[1], frame_hdr[2], frame_hdr[3]]) as usize;
    let mut buf = vec![0u8; size];
    f.read_exact(&mut buf).expect("read frame");
    buf
}

/// Build a caller-owned `VpxImage` over the supplied plane pointers,
/// with `stride == width` so `image2yuvconfig` computes `border == 0`
/// (and the post-copy border extension is a no-op on these buffers).
unsafe fn make_image(
    d_w: usize,
    d_h: usize,
    uv_w: usize,
    y: *mut u8,
    u: *mut u8,
    v: *mut u8,
) -> VpxImage {
    let mut img: VpxImage = core::mem::zeroed();
    img.fmt = VPX_IMG_FMT_I420;
    img.d_w = d_w as u32;
    img.d_h = d_h as u32;
    img.w = d_w as u32;
    img.h = d_h as u32;
    img.x_chroma_shift = 1;
    img.y_chroma_shift = 1;
    img.planes[VPX_PLANE_Y] = y;
    img.planes[VPX_PLANE_U] = u;
    img.planes[VPX_PLANE_V] = v;
    img.stride[VPX_PLANE_Y] = d_w as i32;
    img.stride[VPX_PLANE_U] = uv_w as i32;
    img.stride[VPX_PLANE_V] = uv_w as i32;
    img
}

/// Decode a keyframe (to allocate the DPB at known dims), install a
/// caller-owned frame as LAST via `VP8_SET_REFERENCE`, read it back via
/// `VP8_COPY_REFERENCE`, and assert the pixels round-trip.
#[test]
fn set_and_copy_reference_round_trip() {
    let Some(dir) = data_dir() else {
        eprintln!("skipping: VP8 test data not available");
        return;
    };
    let packet = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));

    unsafe {
        let iface = vpx_codec_vp8_dx();
        let mut ctx_storage = MaybeUninit::<vpx_codec_ctx_t>::zeroed();
        assert_eq!(
            vpx_codec_dec_init_ver(
                ctx_storage.assume_init_mut(),
                Some(iface),
                None,
                0,
                VPX_DECODER_ABI_VERSION,
            ),
            VPX_CODEC_OK,
        );
        let ctx = ctx_storage.assume_init_mut();

        assert_eq!(
            vpx_codec_decode(ctx, &packet, core::ptr::null_mut(), 0),
            VPX_CODEC_OK,
        );

        // Learn the decoded (16-aligned) plane dimensions; scope the
        // borrow of `ctx` from get_frame so the &mut control calls below
        // are free to reborrow.
        let (d_w, d_h) = {
            let mut iter: *const c_void = core::ptr::null();
            let img = vpx_codec_get_frame(ctx, &mut iter).expect("decoded frame");
            (img.d_w as usize, img.d_h as usize)
        };
        let uv_w = (d_w + 1) / 2;
        let uv_h = (d_h + 1) / 2;

        // Caller-owned source frame, filled with a deterministic pattern.
        let mut src_y = vec![0u8; d_h * d_w];
        let mut src_u = vec![0u8; uv_h * uv_w];
        let mut src_v = vec![0u8; uv_h * uv_w];
        for (i, p) in src_y.iter_mut().enumerate() {
            *p = (i & 0xff) as u8;
        }
        for (i, p) in src_u.iter_mut().enumerate() {
            *p = ((i * 3) & 0xff) as u8;
        }
        for (i, p) in src_v.iter_mut().enumerate() {
            *p = ((i * 7) & 0xff) as u8;
        }

        let src_img = make_image(
            d_w,
            d_h,
            uv_w,
            src_y.as_mut_ptr(),
            src_u.as_mut_ptr(),
            src_v.as_mut_ptr(),
        );
        let mut set_ref = VpxRefFrame {
            frame_type: VP8_LAST_FRAME,
            img: src_img,
        };
        assert_eq!(
            vpx_codec_control_(
                ctx,
                VP8_SET_REFERENCE,
                &mut set_ref as *mut VpxRefFrame as *mut c_void,
            ),
            VPX_CODEC_OK,
            "VP8_SET_REFERENCE with a caller-owned buffer failed",
        );

        // Read LAST back into a distinct caller-owned frame.
        let mut dst_y = vec![0u8; d_h * d_w];
        let mut dst_u = vec![0u8; uv_h * uv_w];
        let mut dst_v = vec![0u8; uv_h * uv_w];
        let dst_img = make_image(
            d_w,
            d_h,
            uv_w,
            dst_y.as_mut_ptr(),
            dst_u.as_mut_ptr(),
            dst_v.as_mut_ptr(),
        );
        let mut copy_ref = VpxRefFrame {
            frame_type: VP8_LAST_FRAME,
            img: dst_img,
        };
        assert_eq!(
            vpx_codec_control_(
                ctx,
                VP8_COPY_REFERENCE,
                &mut copy_ref as *mut VpxRefFrame as *mut c_void,
            ),
            VPX_CODEC_OK,
            "VP8_COPY_REFERENCE into a caller-owned buffer failed",
        );

        assert_eq!(dst_y, src_y, "Y plane did not round-trip SET→COPY");
        assert_eq!(dst_u, src_u, "U plane did not round-trip SET→COPY");
        assert_eq!(dst_v, src_v, "V plane did not round-trip SET→COPY");

        assert_eq!(vpx_codec_destroy(ctx), VPX_CODEC_OK);
    }
}
