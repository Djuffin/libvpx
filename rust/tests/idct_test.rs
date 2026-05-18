//! Port of `test/idct_test.cc` (gtest) to Rust integration tests.
//!
//! Only the C-reference 4x4 IDCT kernel is built into our crate
//! (`vp8_short_idct4x4llm_c`), so we drop the SIMD `INSTANTIATE_TEST_SUITE_P`
//! variants and run the 4 test bodies directly.

use vp8_decoder_rs::idctllm::vp8_short_idct4x4llm_c;

const SIZE: usize = 4;
const PAD_BYTE_U8: u8 = 0xAA;
const PAD_VAL_I16: i16 = 0x5A5A;

/// Strided buffer used for `predict` / `output` (`Buffer<uint8_t>(4, 4, 3)`
/// in the C test). The active 4×4 region is surrounded by `PAD` padding
/// cells filled with a sentinel so we can detect out-of-bounds writes.
struct Buf<T: Copy + PartialEq + std::fmt::Debug> {
    data: Vec<T>,
    pad: usize,
    stride: usize,
}

impl<T: Copy + PartialEq + std::fmt::Debug> Buf<T> {
    fn new(pad: usize, pad_val: T) -> Self {
        let stride = SIZE + 2 * pad;
        Self {
            data: vec![pad_val; stride * stride],
            pad,
            stride,
        }
    }

    fn top_left(&mut self) -> *mut T {
        unsafe {
            self.data
                .as_mut_ptr()
                .add(self.pad * self.stride + self.pad)
        }
    }

    fn stride(&self) -> i32 {
        self.stride as i32
    }

    fn set(&mut self, val: T) {
        for y in 0..SIZE {
            for x in 0..SIZE {
                self.data[(self.pad + y) * self.stride + (self.pad + x)] = val;
            }
        }
    }

    fn at(&self, x: usize, y: usize) -> T {
        self.data[(self.pad + y) * self.stride + (self.pad + x)]
    }

    fn check_values(&self, val: T) -> bool {
        (0..SIZE).all(|y| (0..SIZE).all(|x| self.at(x, y) == val))
    }

    fn check_padding(&self, pad_val: T) -> bool {
        for y in 0..self.stride {
            for x in 0..self.stride {
                let in_data =
                    y >= self.pad && y < self.pad + SIZE && x >= self.pad && x < self.pad + SIZE;
                if !in_data && self.data[y * self.stride + x] != pad_val {
                    return false;
                }
            }
        }
        true
    }
}

// In the C test `input` uses padding=0 — the IDCT reads it as a flat 16
// element array, so the "stride" must equal the width.
fn make_input() -> Buf<i16> {
    Buf::<i16>::new(0, PAD_VAL_I16)
}
fn make_u8() -> Buf<u8> {
    Buf::<u8>::new(3, PAD_BYTE_U8)
}

/// C: `TEST_P(IDCTTest, TestAllZeros)`.
#[test]
fn test_all_zeros() {
    let mut input = make_input();
    let mut predict = make_u8();
    let mut output = make_u8();
    input.set(0);
    predict.set(0);
    output.set(0);

    unsafe {
        vp8_short_idct4x4llm_c(
            input.top_left(),
            predict.top_left(),
            predict.stride(),
            output.top_left(),
            output.stride(),
        );
    }

    assert!(input.check_values(0));
    assert!(input.check_padding(PAD_VAL_I16));
    assert!(output.check_values(0));
    assert!(output.check_padding(PAD_BYTE_U8));
}

/// C: `TEST_P(IDCTTest, TestAllOnes)`.
#[test]
fn test_all_ones() {
    let mut input = make_input();
    let mut predict = make_u8();
    let mut output = make_u8();
    input.set(0);
    // input[0] = 4 → IDCT output is uniform 1 → predict 0 + 1 = 1.
    unsafe { input.top_left().write(4) };
    predict.set(0);
    output.set(0);

    unsafe {
        vp8_short_idct4x4llm_c(
            input.top_left(),
            predict.top_left(),
            predict.stride(),
            output.top_left(),
            output.stride(),
        );
    }

    assert!(output.check_values(1));
    assert!(output.check_padding(PAD_BYTE_U8));
}

/// C: `TEST_P(IDCTTest, TestAddOne)`.
#[test]
fn test_add_one() {
    let mut input = make_input();
    let mut predict = make_u8();
    let mut output = make_u8();
    input.set(0);
    unsafe { input.top_left().write(4) };
    output.set(0);

    // predict[y][x] = y * 4 + x → 0..15.
    for y in 0..SIZE {
        for x in 0..SIZE {
            unsafe {
                predict
                    .top_left()
                    .add(y * predict.stride() as usize + x)
                    .write((y * 4 + x) as u8);
            }
        }
    }

    unsafe {
        vp8_short_idct4x4llm_c(
            input.top_left(),
            predict.top_left(),
            predict.stride(),
            output.top_left(),
            output.stride(),
        );
    }

    for y in 0..SIZE {
        for x in 0..SIZE {
            let got = output.at(x, y);
            assert_eq!(
                1 + (y * 4 + x) as u8,
                got,
                "output[{}][{}] = {}, expected {}",
                y,
                x,
                got,
                1 + (y * 4 + x) as u8
            );
        }
    }
    assert!(output.check_padding(PAD_BYTE_U8));
}

/// C: `TEST_P(IDCTTest, TestWithData)`.
#[test]
fn test_with_data() {
    let mut input = make_input();
    let mut predict = make_u8();
    let mut output = make_u8();
    predict.set(0);

    for y in 0..SIZE {
        for x in 0..SIZE {
            unsafe {
                input
                    .top_left()
                    .add(y * input.stride() as usize + x)
                    .write((y * 4 + x) as i16);
            }
        }
    }

    unsafe {
        vp8_short_idct4x4llm_c(
            input.top_left(),
            predict.top_left(),
            predict.stride(),
            output.top_left(),
            output.stride(),
        );
    }

    for y in 0..SIZE {
        for x in 0..SIZE {
            let expected: u8 = match y * 4 + x {
                0 => 11,
                2 | 5 | 8 => 3,
                10 => 1,
                _ => 0,
            };
            let got = output.at(x, y);
            assert_eq!(
                expected, got,
                "output[{}][{}] = {}, expected {}",
                y, x, got, expected
            );
        }
    }
    assert!(output.check_padding(PAD_BYTE_U8));
}
