//! Port of `test/predict_test.cc` (gtest) to Rust integration tests.
//!
//! Only the C-reference sub-pel kernels exist in our build. The C suite
//! parameterizes over (width, height, fn) — we use one `#[test]` per
//! variant. SIMD `INSTANTIATE_TEST_SUITE_P` blocks are dropped.

use std::os::raw::c_int;

use vp8_decoder_rs::filter::{
    vp8_bilinear_predict4x4_c, vp8_bilinear_predict8x4_c, vp8_bilinear_predict8x8_c,
    vp8_bilinear_predict16x16_c, vp8_sixtap_predict4x4_c, vp8_sixtap_predict8x4_c,
    vp8_sixtap_predict8x8_c, vp8_sixtap_predict16x16_c,
};

type PredictFn = unsafe extern "C" fn(
    src: *mut u8,
    src_pixels_per_line: c_int,
    xoffset: c_int,
    yoffset: c_int,
    dst: *mut u8,
    dst_pitch: c_int,
);

// Six-tap filters need 5 extra pixels outside of the 16x16 macroblock.
const SRC_STRIDE: usize = 21;
const SRC_SIZE: usize = SRC_STRIDE * SRC_STRIDE;
const BORDER: usize = 16;
const BORDER_FILL: u8 = 128;

/// Deterministic LCG (Numerical Recipes), seeded so two test runs
/// produce the same byte stream. Mirrors libvpx's `ACMRandom` use of a
/// fixed `DeterministicSeed`.
struct LCG(u32);

impl LCG {
    fn new() -> Self {
        Self(0xA5A5_A5A5)
    }
    fn rand_u8(&mut self) -> u8 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (self.0 >> 24) as u8
    }
}

/// Padded destination buffer. Active region is `width` × `height`,
/// surrounded by `BORDER` cells of `BORDER_FILL`.
struct PaddedDst {
    buf: Vec<u8>,
    stride: usize,
    width: usize,
    height: usize,
}

impl PaddedDst {
    fn new(width: usize, height: usize) -> Self {
        let stride = BORDER + width + BORDER;
        let total = stride * (BORDER + height + BORDER);
        Self {
            buf: vec![BORDER_FILL; total],
            stride,
            width,
            height,
        }
    }

    fn dst_ptr(&mut self) -> *mut u8 {
        // Safe: bounds-checked indexing + `slice::as_mut_ptr`. The raw
        // pointer is only handed to the C-ABI kernel.
        let off = BORDER * self.stride + BORDER;
        self.buf[off..].as_mut_ptr()
    }

    fn dst_stride(&self) -> c_int {
        self.stride as c_int
    }

    /// Raw pointer `off` bytes past the active top-left corner, used to
    /// exercise unaligned destinations. Safe: bounds-checked indexing.
    fn dst_ptr_offset(&mut self, off: usize) -> *mut u8 {
        let base = BORDER * self.stride + BORDER + off;
        self.buf[base..].as_mut_ptr()
    }

    /// Reset every byte to `BORDER_FILL`.
    fn reset(&mut self) {
        self.buf.iter_mut().for_each(|b| *b = BORDER_FILL);
    }

    /// Confirm every byte outside the active region is still BORDER_FILL.
    fn check_border(&self) -> bool {
        for y in 0..(BORDER + self.height + BORDER) {
            for x in 0..self.stride {
                let in_active = y >= BORDER
                    && y < BORDER + self.height
                    && x >= BORDER
                    && x < BORDER + self.width;
                if !in_active && self.buf[y * self.stride + x] != BORDER_FILL {
                    return false;
                }
            }
        }
        true
    }

    /// Row pointer inside the active region.
    fn row(&self, y: usize) -> &[u8] {
        let row_start = (BORDER + y) * self.stride + BORDER;
        &self.buf[row_start..row_start + self.width]
    }
}

/// `TestWithRandomData(reference)` from the C suite. Since our build
/// only has the C reference variants the "reference" and "UUT" are the
/// same function — the test still has value as a smoke / border check.
fn test_with_random_data(width: usize, height: usize, predict: PredictFn) {
    let mut rng = LCG::new();
    let mut padded = PaddedDst::new(width, height);
    // Plain dst_c output (no border padding) for the side-by-side compare.
    let mut dst_c = vec![0u8; 16 * 16];
    let mut src = vec![0u8; SRC_SIZE];

    for xoffset in 0..8 {
        for yoffset in 0..8 {
            if xoffset == 0 && yoffset == 0 {
                continue; // pure copy, not handled by the filter
            }
            for b in src.iter_mut() {
                *b = rng.rand_u8();
            }
            padded.reset();
            dst_c.iter_mut().for_each(|b| *b = 0);

            let src_base = src[SRC_STRIDE * 2 + 2..].as_mut_ptr();

            // Reference (= same function in our build).
            unsafe {
                predict(
                    src_base,
                    SRC_STRIDE as c_int,
                    xoffset,
                    yoffset,
                    dst_c.as_mut_ptr(),
                    16,
                );
            }
            // UUT writes into the padded destination.
            unsafe {
                predict(
                    src_base,
                    SRC_STRIDE as c_int,
                    xoffset,
                    yoffset,
                    padded.dst_ptr(),
                    padded.dst_stride(),
                );
            }

            for y in 0..height {
                let a = padded.row(y);
                let b = &dst_c[y * 16..y * 16 + width];
                assert_eq!(
                    a, b,
                    "row {y} differs at xoffset={xoffset}, yoffset={yoffset}"
                );
            }
            assert!(
                padded.check_border(),
                "border corrupted at xoffset={xoffset}, yoffset={yoffset}"
            );
        }
    }
}

/// `TestWithUnalignedDst(reference)` — only the 4x4 variants must handle
/// unaligned destination pointers. Caller passes width == height == 4.
fn test_with_unaligned_dst(width: usize, height: usize, predict: PredictFn) {
    assert_eq!((width, height), (4, 4));
    let mut rng = LCG::new();
    let mut padded = PaddedDst::new(width, height);
    let mut dst_c = vec![0u8; 16 * 16];
    let mut src = vec![0u8; SRC_SIZE];

    for xoffset in 0..8 {
        for yoffset in 0..8 {
            if xoffset == 0 && yoffset == 0 {
                continue;
            }
            for b in src.iter_mut() {
                *b = rng.rand_u8();
            }
            let src_base = src[SRC_STRIDE * 2 + 2..].as_mut_ptr();
            unsafe {
                predict(
                    src_base,
                    SRC_STRIDE as c_int,
                    xoffset,
                    yoffset,
                    dst_c.as_mut_ptr(),
                    16,
                );
            }
            for i in 1..4 {
                padded.reset();
                let dst_off = padded.dst_ptr_offset(i);
                let dst_stride = (padded.dst_stride() as usize) + i;
                unsafe {
                    predict(
                        src_base,
                        SRC_STRIDE as c_int,
                        xoffset,
                        yoffset,
                        dst_off,
                        dst_stride as c_int,
                    );
                }
                // The kernel writes rows at dst_off + y * dst_stride;
                // mirror that addressing when reading back.
                let dst_off_idx = BORDER * padded.stride + BORDER + i;
                for y in 0..height {
                    let row_start = dst_off_idx + y * dst_stride;
                    let a = &padded.buf[row_start..row_start + width];
                    let b = &dst_c[y * 16..y * 16 + width];
                    assert_eq!(
                        a, b,
                        "unaligned i={i} xoffset={xoffset} yoffset={yoffset} row={y}",
                    );
                }
            }
        }
    }
}

// ------------------------------------------------------------------
// SixtapPredict variants — TestWithRandomData
// ------------------------------------------------------------------

#[test]
fn sixtap_random_16x16() {
    test_with_random_data(16, 16, vp8_sixtap_predict16x16_c);
}
#[test]
fn sixtap_random_8x8() {
    test_with_random_data(8, 8, vp8_sixtap_predict8x8_c);
}
#[test]
fn sixtap_random_8x4() {
    test_with_random_data(8, 4, vp8_sixtap_predict8x4_c);
}
#[test]
fn sixtap_random_4x4() {
    test_with_random_data(4, 4, vp8_sixtap_predict4x4_c);
}

#[test]
fn sixtap_unaligned_4x4() {
    test_with_unaligned_dst(4, 4, vp8_sixtap_predict4x4_c);
}

// ------------------------------------------------------------------
// BilinearPredict variants — TestWithRandomData
// ------------------------------------------------------------------

#[test]
fn bilinear_random_16x16() {
    test_with_random_data(16, 16, vp8_bilinear_predict16x16_c);
}
#[test]
fn bilinear_random_8x8() {
    test_with_random_data(8, 8, vp8_bilinear_predict8x8_c);
}
#[test]
fn bilinear_random_8x4() {
    test_with_random_data(8, 4, vp8_bilinear_predict8x4_c);
}
#[test]
fn bilinear_random_4x4() {
    test_with_random_data(4, 4, vp8_bilinear_predict4x4_c);
}

#[test]
fn bilinear_unaligned_4x4() {
    test_with_unaligned_dst(4, 4, vp8_bilinear_predict4x4_c);
}

// ------------------------------------------------------------------
// SixtapPredict 16x16 — TestWithPresetData
// ------------------------------------------------------------------

#[rustfmt::skip]
const PRESET_INPUT: [u8; SRC_SIZE] = [
    184, 4,   191, 82,  92,  41,  0,   1,   226, 236, 172, 20,  182, 42,  226,
    177, 79,  94,  77,  179, 203, 206, 198, 22,  192, 19,  75,  17,  192, 44,
    233, 120, 48,  168, 203, 141, 210, 203, 143, 180, 184, 59,  201, 110, 102,
    171, 32,  182, 10,  109, 105, 213, 60,  47,  236, 253, 67,  55,  14,  3,
    99,  247, 124, 148, 159, 71,  34,  114, 19,  177, 38,  203, 237, 239, 58,
    83,  155, 91,  10,  166, 201, 115, 124, 5,   163, 104, 2,   231, 160, 16,
    234, 4,   8,   103, 153, 167, 174, 187, 26,  193, 109, 64,  141, 90,  48,
    200, 174, 204, 36,  184, 114, 237, 43,  238, 242, 207, 86,  245, 182, 247,
    6,   161, 251, 14,  8,   148, 182, 182, 79,  208, 120, 188, 17,  6,   23,
    65,  206, 197, 13,  242, 126, 128, 224, 170, 110, 211, 121, 197, 200, 47,
    188, 207, 208, 184, 221, 216, 76,  148, 143, 156, 100, 8,   89,  117, 14,
    112, 183, 221, 54,  197, 208, 180, 69,  176, 94,  180, 131, 215, 121, 76,
    7,   54,  28,  216, 238, 249, 176, 58,  142, 64,  215, 242, 72,  49,  104,
    87,  161, 32,  52,  216, 230, 4,   141, 44,  181, 235, 224, 57,  195, 89,
    134, 203, 144, 162, 163, 126, 156, 84,  185, 42,  148, 145, 29,  221, 194,
    134, 52,  100, 166, 105, 60,  140, 110, 201, 184, 35,  181, 153, 93,  121,
    243, 227, 68,  131, 134, 232, 2,   35,  60,  187, 77,  209, 76,  106, 174,
    15,  241, 227, 115, 151, 77,  175, 36,  187, 121, 221, 223, 47,  118, 61,
    168, 105, 32,  237, 236, 167, 213, 238, 202, 17,  170, 24,  226, 247, 131,
    145, 6,   116, 117, 121, 11,  194, 41,  48,  126, 162, 13,  93,  209, 131,
    154, 122, 237, 187, 103, 217, 99,  60,  200, 45,  78,  115, 69,  49,  106,
    200, 194, 112, 60,  56,  234, 72,  251, 19,  120, 121, 182, 134, 215, 135,
    10,  114, 2,   247, 46,  105, 209, 145, 165, 153, 191, 243, 12,  5,   36,
    119, 206, 231, 231, 11,  32,  209, 83,  27,  229, 204, 149, 155, 83,  109,
    35,  93,  223, 37,  84,  14,  142, 37,  160, 52,  191, 96,  40,  204, 101,
    77,  67,  52,  53,  43,  63,  85,  253, 147, 113, 226, 96,  6,   125, 179,
    115, 161, 17,  83,  198, 101, 98,  85,  139, 3,   137, 75,  99,  178, 23,
    201, 255, 91,  253, 52,  134, 60,  138, 131, 208, 251, 101, 48,  2,   227,
    228, 118, 132, 245, 202, 75,  91,  44,  160, 231, 47,  41,  50,  147, 220,
    74,  92,  219, 165, 89,  16,
];

#[rustfmt::skip]
const PRESET_EXPECTED: [u8; 256] = [
    117, 102, 74,  135, 42,  98,  175, 206, 70,  73,  222, 197, 50,  24,  39,
    49,  38,  105, 90,  47,  169, 40,  171, 215, 200, 73,  109, 141, 53,  85,
    177, 164, 79,  208, 124, 89,  212, 18,  81,  145, 151, 164, 217, 153, 91,
    154, 102, 102, 159, 75,  164, 152, 136, 51,  213, 219, 186, 116, 193, 224,
    186, 36,  231, 208, 84,  211, 155, 167, 35,  59,  42,  76,  216, 149, 73,
    201, 78,  149, 184, 100, 96,  196, 189, 198, 188, 235, 195, 117, 129, 120,
    129, 49,  25,  133, 113, 69,  221, 114, 70,  143, 99,  157, 108, 189, 140,
    78,  6,   55,  65,  240, 255, 245, 184, 72,  90,  100, 116, 131, 39,  60,
    234, 167, 33,  160, 88,  185, 200, 157, 159, 176, 127, 151, 138, 102, 168,
    106, 170, 86,  82,  219, 189, 76,  33,  115, 197, 106, 96,  198, 136, 97,
    141, 237, 151, 98,  137, 191, 185, 2,   57,  95,  142, 91,  255, 185, 97,
    137, 76,  162, 94,  173, 131, 193, 161, 81,  106, 72,  135, 222, 234, 137,
    66,  137, 106, 243, 210, 147, 95,  15,  137, 110, 85,  66,  16,  96,  167,
    147, 150, 173, 203, 140, 118, 196, 84,  147, 160, 19,  95,  101, 123, 74,
    132, 202, 82,  166, 12,  131, 166, 189, 170, 159, 85,  79,  66,  57,  152,
    132, 203, 194, 0,   1,   56,  146, 180, 224, 156, 28,  83,  181, 79,  76,
    80,  46,  160, 175, 59,  106, 43,  87,  75,  136, 85,  189, 46,  71,  200,
    90,
];

#[test]
fn sixtap_preset_16x16() {
    let mut padded = PaddedDst::new(16, 16);
    let mut input = PRESET_INPUT;
    let src_base = input[SRC_STRIDE * 2 + 2..].as_mut_ptr();

    unsafe {
        vp8_sixtap_predict16x16_c(
            src_base,
            SRC_STRIDE as c_int,
            2,
            2,
            padded.dst_ptr(),
            padded.dst_stride(),
        );
    }

    for y in 0..16 {
        let got = padded.row(y);
        let want = &PRESET_EXPECTED[y * 16..y * 16 + 16];
        assert_eq!(got, want, "row {y} differs");
    }
}
