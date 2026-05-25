use std::sync::Arc;

use crate::api::config::{Codec, DecoderConfig};
use crate::api::create_decoder;
use crate::api::default_allocator::DefaultAllocator;
use crate::api::packet::EncodedPacket;
use crate::api::*;
use crate::vpx_api::{VPX_IMG_FMT_I420, VPX_PLANE_U, VPX_PLANE_V, VPX_PLANE_Y, VpxImage};

use super::support::{test_data_dir, read_ivf_first_packet, CountingCallbacks};

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

#[test]
fn set_and_copy_reference_round_trip() {
    let Some(dir) = test_data_dir("control_test") else { return; };
    let packet_data = read_ivf_first_packet(&dir.join("vp80-00-comprehensive-001.ivf"));

    let callbacks = CountingCallbacks::shared();
    let config = DecoderConfig::new(Codec::VP8);
    let mut decoder = create_decoder(config, Arc::new(DefaultAllocator), callbacks)
        .expect("create");

    // Decode keyframe to instantiate internal buffers and dimensions
    let packet = EncodedPacket {
        data: Arc::new(packet_data),
        opaque: None,
    };
    decoder.decode(packet).expect("decode");

    let pic = decoder.get_picture().expect("get_picture")
        .expect("decoded picture");
    
    let d_w = pic.format.display_width;
    let d_h = pic.format.display_height;
    let uv_w = (d_w + 1) / 2;
    let uv_h = (d_h + 1) / 2;

    // 1. Create a deterministic pattern in a caller-owned source image
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

    unsafe {
        let src_img = make_image(
            d_w,
            d_h,
            uv_w,
            src_y.as_mut_ptr(),
            src_u.as_mut_ptr(),
            src_v.as_mut_ptr(),
        );

        // 2. Set the reference frame LAST to our custom image
        let mut set_ref = Vp8SetReference {
            frame_type: 1, // VP8_LAST_FRAME is 1
            img: src_img,
        };
        decoder.control(&mut set_ref).expect("Vp8SetReference failed");

        // 3. Copy it back out into a distinct caller-owned destination image
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

        let mut copy_ref = Vp8CopyReference {
            frame_type: 1,
            img: std::cell::Cell::new(dst_img),
        };
        decoder.control(&mut copy_ref).expect("Vp8CopyReference failed");

        let final_dst = copy_ref.img.get();
        assert_eq!(final_dst.planes[VPX_PLANE_Y], dst_img.planes[VPX_PLANE_Y]);

        assert_eq!(dst_y, src_y, "Y plane mismatch after Vp8SetReference -> Vp8CopyReference roundtrip");
        assert_eq!(dst_u, src_u, "U plane mismatch");
        assert_eq!(dst_v, src_v, "V plane mismatch");
    }

    // 4. Query other frame metrics using codec controls
    let mut q_cmd = Vp8GetLastQuantizer { out: std::cell::Cell::new(0) };
    decoder.control(&mut q_cmd).expect("Vp8GetLastQuantizer failed");
    let QP = q_cmd.out.get();
    assert!(QP > 0, "QP should be positive, got {QP}");

    let mut c_cmd = Vp8GetFrameCorrupted { out: std::cell::Cell::new(0) };
    decoder.control(&mut c_cmd).expect("Vp8GetFrameCorrupted failed");
    let corrupted = c_cmd.out.get();
    assert_eq!(corrupted, 0, "keyframe must not be corrupted");
}
