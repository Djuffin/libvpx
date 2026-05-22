//! Port of `test/vpx_image_test.cc` (gtest) to Rust integration tests.
//!
//! Each `#[test]` mirrors one `TEST(VpxImageTest, ...)` from the C suite.

use core::mem::MaybeUninit;
use core::ptr;
use std::ffi::c_uint;

use vp8_decoder_rs::vpx_api::{
    VPX_IMG_FMT_I420, VPX_IMG_FMT_I444, VPX_IMG_FMT_I42016, VPX_IMG_FMT_NONE, VPX_IMG_FMT_NV12,
    VPX_IMG_FMT_YV12, VPX_PLANE_U, VPX_PLANE_V, VPX_PLANE_Y, vpx_image_t, vpx_img_alloc,
    vpx_img_free, vpx_img_set_rect, vpx_img_wrap,
};

/// C: `TEST(VpxImageTest, VpxImgWrapInvalidAlign)`.
#[test]
fn vpx_img_wrap_invalid_align() {
    const W: usize = 128;
    const H: usize = 128;
    let mut buf = vec![0u8; W * H * 3];

    let mut img: MaybeUninit<vpx_image_t> = MaybeUninit::uninit();
    // Stamp junk into the two fields the implementation must not read on failure.
    let img_ptr = img.as_mut_ptr();
    unsafe {
        (*img_ptr).img_data = b"\0" as *const u8 as *mut u8;
        (*img_ptr).img_data_owner = 1;
    }

    // `align = 31` is not a power of two → `vpx_img_wrap` must return null.
    let align: c_uint = 31;
    let r = unsafe {
        vpx_img_wrap(
            img_ptr,
            VPX_IMG_FMT_I444,
            W as c_uint,
            H as c_uint,
            align,
            buf.as_mut_ptr(),
        )
    };
    assert!(
        r.is_null(),
        "vpx_img_wrap should fail on non-power-of-2 align"
    );
}

/// C: `TEST(VpxImageTest, VpxImgSetRectOverflow)`.
#[test]
fn vpx_img_set_rect_overflow() {
    const W: c_uint = 128;
    const H: c_uint = 128;
    let mut buf = vec![0u8; (W * H * 3) as usize];

    let mut img: MaybeUninit<vpx_image_t> = MaybeUninit::uninit();
    let img_ptr = img.as_mut_ptr();
    let align: c_uint = 32;
    let wrapped = unsafe { vpx_img_wrap(img_ptr, VPX_IMG_FMT_I444, W, H, align, buf.as_mut_ptr()) };
    assert_eq!(wrapped, img_ptr, "vpx_img_wrap should succeed");

    assert_eq!(unsafe { vpx_img_set_rect(img_ptr, 0, 0, W, H) }, 0);
    // `-1` cast to `c_uint` becomes `UINT_MAX` → overflow inside set_rect.
    let neg1 = u32::MAX as c_uint;
    assert_ne!(
        unsafe { vpx_img_set_rect(img_ptr, neg1, neg1, W, H) },
        0,
        "vpx_img_set_rect should reject overflowing offsets"
    );
}

/// C: `TEST(VpxImageTest, VpxImgAllocNone)`.
#[test]
fn vpx_img_alloc_none() {
    let mut img: MaybeUninit<vpx_image_t> = MaybeUninit::uninit();
    let r = unsafe { vpx_img_alloc(img.as_mut_ptr(), VPX_IMG_FMT_NONE, 128, 128, 32) };
    assert!(r.is_null(), "alloc with VPX_IMG_FMT_NONE must fail");
}

/// C: `TEST(VpxImageTest, VpxImgAllocNv12)`.
#[test]
fn vpx_img_alloc_nv12() {
    let mut img: MaybeUninit<vpx_image_t> = MaybeUninit::uninit();
    let img_ptr = img.as_mut_ptr();
    let r = unsafe { vpx_img_alloc(img_ptr, VPX_IMG_FMT_NV12, 128, 128, 32) };
    assert_eq!(r, img_ptr, "NV12 alloc should succeed");

    unsafe {
        // NV12 packs U and V interleaved into one plane → both share stride.
        assert_eq!(
            (*img_ptr).stride[VPX_PLANE_U],
            (*img_ptr).stride[VPX_PLANE_Y]
        );
        assert_eq!(
            (*img_ptr).stride[VPX_PLANE_V],
            (*img_ptr).stride[VPX_PLANE_U]
        );
        // V plane pointer is U + 1 (interleaved layout).
        assert_eq!(
            (*img_ptr).planes[VPX_PLANE_V],
            (*img_ptr).planes[VPX_PLANE_U].add(1)
        );
        vpx_img_free(img_ptr);
    }
}

/// C: `TEST(VpxImageTest, VpxImgAllocHugeWidth)`.
#[test]
fn vpx_img_alloc_huge_width() {
    unsafe {
        // The stride (0x80000000 * 2) would overflow unsigned int.
        let img = vpx_img_alloc(ptr::null_mut(), VPX_IMG_FMT_I42016, 0x8000_0000, 1, 1);
        assert!(img.is_null());

        // The stride (0x80000000) would overflow int.
        let img = vpx_img_alloc(ptr::null_mut(), VPX_IMG_FMT_I420, 0x8000_0000, 1, 1);
        assert!(img.is_null());

        // The aligned width (UINT_MAX + 1) would overflow unsigned int.
        let img = vpx_img_alloc(ptr::null_mut(), VPX_IMG_FMT_I420, u32::MAX, 1, 1);
        assert!(img.is_null());

        // Remaining cases may succeed or fail; just check we don't crash.
        let img = vpx_img_alloc(ptr::null_mut(), VPX_IMG_FMT_I420, 0x7fff_fffe, 1, 1);
        if !img.is_null() {
            vpx_img_free(img);
        }

        let img = vpx_img_alloc(ptr::null_mut(), VPX_IMG_FMT_I420, 285_245_883, 64, 1);
        if !img.is_null() {
            vpx_img_free(img);
        }

        let img = vpx_img_alloc(ptr::null_mut(), VPX_IMG_FMT_NV12, 285_245_883, 64, 1);
        if !img.is_null() {
            vpx_img_free(img);
        }

        let img = vpx_img_alloc(ptr::null_mut(), VPX_IMG_FMT_YV12, 285_245_883, 64, 1);
        if !img.is_null() {
            vpx_img_free(img);
        }

        let img = vpx_img_alloc(ptr::null_mut(), VPX_IMG_FMT_I42016, 65536, 2, 1);
        if !img.is_null() {
            let y = (*img).planes[VPX_PLANE_Y] as *mut u16;
            y.write(0);
            y.add(((*img).d_w - 1) as usize).write(0);
            vpx_img_free(img);
        }

        let img = vpx_img_alloc(ptr::null_mut(), VPX_IMG_FMT_I42016, 285_245_883, 2, 1);
        if !img.is_null() {
            let y = (*img).planes[VPX_PLANE_Y] as *mut u16;
            y.write(0);
            y.add(((*img).d_w - 1) as usize).write(0);
            vpx_img_free(img);
        }
    }
}
