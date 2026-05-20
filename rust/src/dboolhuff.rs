//! VP8 boolean arithmetic decoder.
//!
//! Literal translation of `vp8/decoder/dboolhuff.c` and the inline
//! helpers from `vp8/decoder/dboolhuff.h`. See `documentation/vp8_files/
//! dboolhuff.md` for the design / invariants.
//!
//! The Rust [`BoolDecoder`] in `types.rs` replaces the C pair
//! `(user_buffer, user_buffer_end)` with a borrowed slice plus a `pos`
//! cursor. The translation below mirrors the C control flow exactly,
//! addressing the slice as if it were a `(buf, buf + sz)` pair: the
//! "current pointer" is `buffer.as_ptr().add(pos)` and the "end" is
//! `buffer.as_ptr().add(buffer.len())`.

#![allow(dead_code)]
#![allow(clippy::missing_safety_doc)]

use crate::tables::VP8_NORM;
use crate::types::{BD_VALUE_BITS, BdValue, BoolDecoder, DecryptCbMut, VP8_LOTS_OF_BITS};

// ---------------------------------------------------------------------------
// Local constants mirroring the C `#define`s in `dboolhuff.h`.
// ---------------------------------------------------------------------------

/// `CHAR_BIT` — bits per byte. C `<limits.h>` constant; always 8 on the
/// targets libvpx supports.
const CHAR_BIT: i32 = 8;

/// `VP8_BD_VALUE_SIZE` — bit width of [`BdValue`]. The C macro derives
/// this from `sizeof(VP8_BD_VALUE) * CHAR_BIT`.
const VP8_BD_VALUE_SIZE: i32 = BD_VALUE_BITS as i32;

// ---------------------------------------------------------------------------
// vp8dx_start_decode (dboolhuff.c:15)
// ---------------------------------------------------------------------------

/// `vp8dx_start_decode` — open / seed a [`BoolDecoder`] over a partition.
///
/// Source: `vp8/decoder/dboolhuff.c:15`.
///
/// In libvpx C the decryptor is a `(callback, state)` pair. In the Rust
/// port the callback is owned by the decoder as `Option<DecryptCb>`; the
/// caller hands it in here and we install it before priming the buffer.
/// Returns `0` on success, `1` if `source_sz != 0 && source.is_null()`
/// (UBSan-clean equivalent of the C `if (source_sz && !source)` check).
pub fn vp8dx_start_decode<'a>(
    br: &mut BoolDecoder<'a>,
    source: *const u8,
    source_sz: u32,
    decrypt_cb: Option<DecryptCbMut<'a>>,
) -> i32 {
    if source_sz != 0 && source.is_null() {
        return 1;
    }

    // To simplify calling code this function can be called with |source| == null
    // and |source_sz| == 0. This and vp8dx_bool_decoder_fill() are essentially
    // no-ops in this case.
    // Work around a ubsan warning with a ternary to avoid adding 0 to null.
    // SAFETY: caller guarantees `source` points to `source_sz` valid bytes
    // that outlive `'a` (the partition buffer), unless source is null.
    let buffer: &'a [u8] = if source.is_null() {
        &[]
    } else {
        unsafe { core::slice::from_raw_parts(source, source_sz as usize) }
    };

    br.buffer = buffer;
    br.pos = 0;
    br.value = 0;
    br.count = -8;
    br.range = 255;
    br.decrypt = decrypt_cb;

    // Populate the buffer.
    vp8dx_bool_decoder_fill(br);

    0
}

// ---------------------------------------------------------------------------
// vp8dx_bool_decoder_fill (dboolhuff.c:38)
// ---------------------------------------------------------------------------

/// `vp8dx_bool_decoder_fill` — refill the look-ahead window.
///
/// Source: `vp8/decoder/dboolhuff.c:38`.
///
/// Called from [`vp8dx_decode_bool`] when `count` drops below zero. See
/// the file-level doc and `documentation/vp8_files/dboolhuff.md` for the
/// invariants.
pub fn vp8dx_bool_decoder_fill(br: &mut BoolDecoder<'_>) {
    // Stand-in for `const unsigned char *bufptr = br->user_buffer;`. We
    // keep a usize cursor into `buffer` rather than a raw pointer so the
    // optional decrypt path can swap the source out for a stack buffer
    // without aliasing the borrowed slice.
    let buffer_ptr = br.buffer.as_ptr();
    let buffer_len = br.buffer.len();

    let mut value: BdValue = br.value;
    let mut count: i32 = br.count;
    let mut shift: i32 = VP8_BD_VALUE_SIZE - CHAR_BIT - (count + CHAR_BIT);
    let mut loop_end: i32 = 0;
    let mut decrypted: [u8; core::mem::size_of::<BdValue>() + 1] =
        [0; core::mem::size_of::<BdValue>() + 1];

    // SAFETY: the byte cursor `bufptr` walks `br.buffer[pos..]`; the
    // optional decryptor reroutes it to a stack scratch buffer. All
    // dereferences stay within `buffer_len - pos` bytes (or the
    // decrypted scratch for `n` bytes). `buffer_end` is one-past-the-end,
    // valid to form but never dereferenced.
    unsafe {
        let buffer_end = buffer_ptr.add(buffer_len);
        let mut bufptr: *const u8 = buffer_ptr.add(br.pos);

        let bytes_left: usize = (buffer_end as usize).wrapping_sub(bufptr as usize);
        let bits_left: usize = bytes_left * CHAR_BIT as usize;
        let x: i32 = shift + CHAR_BIT - (bits_left as i32);

        if let Some(cb) = br.decrypt.as_mut() {
            // VPXMIN(sizeof(decrypted), bytes_left)
            let n: usize = decrypted.len().min(bytes_left);
            let src_slice = core::slice::from_raw_parts(bufptr, n);
            cb(src_slice, &mut decrypted[..n]);
            bufptr = decrypted.as_ptr();
        }

        if x >= 0 {
            count += VP8_LOTS_OF_BITS;
            loop_end = x;
        }

        if x < 0 || bits_left != 0 {
            while shift >= loop_end {
                count += CHAR_BIT;
                value |= (*bufptr as BdValue) << shift;
                bufptr = bufptr.add(1);
                // Mirror `++br->user_buffer` — advance the real cursor even
                // when the optional decryptor rerouted `bufptr` to the stack
                // buffer.
                br.pos += 1;
                shift -= CHAR_BIT;
            }
        }
    }

    br.value = value;
    br.count = count;
}

// ---------------------------------------------------------------------------
// vp8dx_decode_bool (dboolhuff.h:54) — `static inline` in C, the per-bit
// hot path. Translated as a regular `pub unsafe fn`; later phases may
// mark it `#[inline]` once the call-site shape is settled.
// ---------------------------------------------------------------------------

/// `vp8dx_decode_bool` — decode one binary symbol at the given probability.
///
/// Source: `vp8/decoder/dboolhuff.h:54`.
pub fn vp8dx_decode_bool(br: &mut BoolDecoder<'_>, probability: i32) -> i32 {
    let mut bit: u32 = 0;

    let split: u32 = 1 + (((br.range - 1) * probability as u32) >> 8);

    if br.count < 0 {
        vp8dx_bool_decoder_fill(br);
    }

    let mut value_local: BdValue = br.value;
    let mut count: i32 = br.count;

    let bigsplit: BdValue = (split as BdValue) << (VP8_BD_VALUE_SIZE - 8);

    let mut range: u32 = split;

    if value_local >= bigsplit {
        range = br.range - split;
        value_local -= bigsplit;
        bit = 1;
    }

    let shift: u8 = VP8_NORM[(range as u8) as usize];
    range <<= shift;
    value_local <<= shift;
    count -= shift as i32;

    br.value = value_local;
    br.count = count;
    br.range = range;

    bit as i32
}

// ---------------------------------------------------------------------------
// vp8_decode_value (dboolhuff.h:93)
// ---------------------------------------------------------------------------

/// `vp8_decode_value` — read `bits` literal bits at probability 128.
///
/// Source: `vp8/decoder/dboolhuff.h:93`.
pub fn vp8_decode_value(br: &mut BoolDecoder<'_>, bits: i32) -> i32 {
    let mut z: i32 = 0;
    let mut bit: i32 = bits - 1;
    while bit >= 0 {
        z |= vp8dx_decode_bool(br, 0x80) << bit;
        bit -= 1;
    }
    z
}

// ---------------------------------------------------------------------------
// vp8dx_bool_error (dboolhuff.h:104)
// ---------------------------------------------------------------------------

/// `vp8dx_bool_error` — non-zero iff the decoder has read past EOF.
///
/// Source: `vp8/decoder/dboolhuff.h:104`.
pub fn vp8dx_bool_error(br: &BoolDecoder<'_>) -> i32 {
    // Check if we have reached the end of the buffer.
    //
    // Variable 'count' stores the number of bits in the 'value' buffer, minus
    // 8. The top byte is part of the algorithm, and the remainder is buffered
    // to be shifted into it. So if count == 8, the top 16 bits of 'value' are
    // occupied, 8 for the algorithm and 8 in the buffer.
    //
    // When reading a byte from the user's buffer, count is filled with 8 and
    // one byte is filled into the value buffer. When we reach the end of the
    // data, count is additionally filled with VP8_LOTS_OF_BITS. So when
    // count == VP8_LOTS_OF_BITS - 1, the user's data has been exhausted.
    if br.count > VP8_BD_VALUE_SIZE && br.count < VP8_LOTS_OF_BITS {
        // We have tried to decode bits after the end of stream was encountered.
        return 1;
    }

    // No error.
    0
}
