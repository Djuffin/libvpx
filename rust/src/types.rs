//! VP8 decoder backbone types.
//!
//! Literal Rust translations of the structs/enums/aliases that hold the
//! decoder's state. This module declares the *shape* of those types only
//! — there is intentionally no behavior here. Method bodies, allocation
//! helpers, FFI shims and the bool-decoder hot path land in later phases.
//!
//! Layout decisions follow `documentation/translation_summary.md` §3:
//!   - direct C transliteration with raw `*mut u8` plane pointers,
//!   - `#[repr(C)]` (and `align(16)` for the working MB) wherever bit
//!     layout matters for compatibility with SIMD/assembly later,
//!   - the `int_mv` union collapses to a single `Mv` (both halves of
//!     the C union are addressed via helper methods to be added later),
//!   - `union b_mode_info` becomes a tagged enum `BModeInfo`. This is
//!     larger than the C union (8 vs 4 bytes) — accepted, see the design
//!     doc.
//!
//! Build targets the minimal libvpx configuration (`--disable-postproc
//! --disable-error-concealment --disable-multithread`), so per-frame
//! state for those features is omitted (matches the conditional
//! `#if`-guarded fields in `onyxc_int.h` / `onyxd_int.h`).

#![allow(dead_code)]


use crate::tables::{
    BLOCK_TYPES, COEF_BANDS, ENTROPY_NODES, MvContext, PREV_COEF_CONTEXTS, Prob, QINDEX_RANGE,
    VP8_BINTRAMODES, VP8_SUBMVREFS, VP8_UV_MODES, VP8_YMODES,
};

// ===========================================================================
// Decoder-wide constants (mirrors `onyxc_int.h` / `loopfilter.h` /
// `yv12config.h` / `blockd.h`).
// ===========================================================================

/// `MINQ` — minimum quantizer index. RFC 6386 §9.6.
pub const MIN_Q: i32 = 0;
/// `MAX_REF_FRAMES` — sign-bias and refcount arrays are 4 wide
/// (INTRA, LAST, GOLDEN, ALTREF). RFC 6386 §16.
pub const MAX_REF_FRAMES: usize = 4;
/// `NUM_YV12_BUFFERS` — slots in the reference picture pool. RFC 6386 §16.
pub const NUM_YV12_BUFFERS: usize = 4;
/// `MAX_PARTITIONS` — 8 token partitions + 1 residual partition.
/// RFC 6386 §9.5.
pub const MAX_PARTITIONS: usize = 9;
/// `MB_FEATURE_TREE_PROBS` — number of segment-id tree probabilities.
pub const MB_FEATURE_TREE_PROBS: usize = 3;
/// `MAX_MB_SEGMENTS` — segments per frame. RFC 6386 §10.
pub const MAX_MB_SEGMENTS: usize = 4;
/// `MAX_REF_LF_DELTAS` — reference-frame loop-filter deltas (INTRA/LAST/GF/ARF).
pub const MAX_REF_LF_DELTAS: usize = 4;
/// `MAX_MODE_LF_DELTAS` — mode-class loop-filter deltas (BPRED/ZERO/MV/SPLIT).
pub const MAX_MODE_LF_DELTAS: usize = 4;
/// `MAX_LOOP_FILTER` — largest legal filter strength. RFC 6386 §15.
pub const MAX_LOOP_FILTER: usize = 63;
/// SIMD lane width used to pad the per-strength threshold rows in
/// `LoopFilterInfoN`. Matches the x86 build (`SIMD_WIDTH = 16`); ARM
/// builds use 1 in libvpx — we standardize on 16.
pub const SIMD_WIDTH: usize = 16;
/// `VP8BORDERINPIXELS` — border each YV12 plane is padded with.
pub const VP8_BORDER_IN_PIXELS: i32 = 32;
/// `VP8_HEADER_SIZE` — bytes consumed by the uncompressed frame tag
/// (3 bytes for the minimal build).
pub const VP8_HEADER_SIZE: usize = 3;

// ===========================================================================
// Atoms
// ===========================================================================

/// `MV` (`mv.h`) — sub-pel motion vector, 1/8-pel units.
/// RFC 6386 §17.
#[derive(Copy, Clone, Default, PartialEq, Eq)]
#[repr(C)]
pub struct Mv {
    pub row: i16,
    pub col: i16,
}

/// Pack an [`Mv`] the way the C `int_mv` union exposes its `as_int`
/// field: low 16 bits = `row`, high 16 bits = `col`. Used for cheap
/// equality / zero compares.
#[inline]
pub fn mv_as_int(m: Mv) -> u32 {
    ((m.col as u16 as u32) << 16) | (m.row as u16 as u32)
}

/// Inverse of [`mv_as_int`].
#[inline]
pub fn mv_from_int(v: u32) -> Mv {
    Mv {
        row: v as u16 as i16,
        col: (v >> 16) as u16 as i16,
    }
}

/// `POS` (`blockd.h`).
#[derive(Copy, Clone)]
#[repr(C)]
pub struct Pos {
    pub r: i32,
    pub c: i32,
}

/// `FRAME_TYPE` (`blockd.h`). RFC 6386 §9.1.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum FrameType {
    Key = 0,
    Inter = 1,
}

/// `MB_PREDICTION_MODE` (`blockd.h`). Listed in C-enum order so the
/// `as u8` cast matches the bitstream-side integer values used by the
/// mode-decoding trees (`VP8_YMODE_TREE`, `VP8_KF_YMODE_TREE`,
/// `VP8_MV_REF_TREE`). RFC 6386 §11 (intra) / §16 (inter).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum MbPredictionMode {
    DcPred = 0,
    VPred,
    HPred,
    TmPred,
    BPred,
    NearestMv,
    NearMv,
    ZeroMv,
    NewMv,
    SplitMv,
}

/// Sentinel for the C `MB_MODE_COUNT` (one past `SplitMv`).
pub const MB_MODE_COUNT: usize = 10;

/// `B_PREDICTION_MODE` (`blockd.h`). The first 10 variants are the
/// intra-4x4 modes; the last 4 are the SPLITMV sub-block reference
/// modes (`LEFT4X4` … `NEW4X4`). Note: the `Mv` variant of
/// [`BModeInfo`] carries the sub-block MV for SPLITMV — these enum
/// tags are only used by `VP8_SUB_MV_REF_TREE` parsing, not stored in
/// `bmi[i].as_mode`.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum BPredictionMode {
    DcPred = 0,
    TmPred,
    VePred,
    HePred,
    LdPred,
    RdPred,
    VrPred,
    VlPred,
    HdPred,
    HuPred,
    Left4x4,
    Above4x4,
    Zero4x4,
    New4x4,
}

/// Sentinel for `B_MODE_COUNT` (one past `New4x4`).
pub const B_MODE_COUNT: usize = 14;

/// `MV_REFERENCE_FRAME` (`blockd.h`). RFC 6386 §16.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum MvReferenceFrame {
    Intra = 0,
    Last = 1,
    Golden = 2,
    Altref = 3,
}

/// Slot indices that match [`MvReferenceFrame`] discriminants, used for
/// `usize` array indexing into per-frame reference-buffer state.
pub const INTRA_FRAME: usize = 0;
pub const LAST_FRAME: usize = 1;
pub const GOLDEN_FRAME: usize = 2;
pub const ALTREF_FRAME: usize = 3;

/// Reference-frame bitmap constants (`vpx/vp8.h`).
pub const VP8_LAST_FRAME: i32 = 1;
pub const VP8_GOLD_FRAME: i32 = 2;
pub const VP8_ALTR_FRAME: i32 = 4;

/// `MB_LVL_FEATURES` (`blockd.h`) — segment-feature kinds. RFC 6386 §10.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum MbLevelFeature {
    AltQ = 0,
    AltLf = 1,
}

/// `MB_LVL_MAX` — number of MB-level features (alt-Q, alt-LF).
/// Matches the length of `tables::VP8_MB_FEATURE_DATA_BITS`. RFC 6386 §10.
pub const MB_LVL_MAX: usize = 2;

/// `LOOPFILTERTYPE` (`loopfilter.h`). RFC 6386 §15.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum LoopFilterType {
    Normal = 0,
    Simple = 1,
}

/// `CLAMP_TYPE` (`onyxc_int.h`). Bit 1 of the keyframe color-space byte.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum ClampType {
    Required = 0,
    NotRequired = 1,
}

/// `TOKEN_PARTITION` (`onyxc_int.h`) — log2 of the number of token
/// partitions. RFC 6386 §9.5.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum TokenPartition {
    One = 0,
    Two = 1,
    Four = 2,
    Eight = 3,
}

// ===========================================================================
// `union b_mode_info`
// ===========================================================================

/// `union b_mode_info` (`blockd.h`). When the parent MB uses `B_PRED`
/// the entry holds the 4x4 intra mode; under `SplitMv` it holds a
/// sub-block motion vector. RFC 6386 §11.5 / §17.4.
#[derive(Copy, Clone)]
pub enum BModeInfo {
    Intra(BPredictionMode),
    Mv(Mv),
}

// ===========================================================================
// Entropy context (per-plane, per-row carry of "any-coeff" flags)
// ===========================================================================

/// `ENTROPY_CONTEXT` typedef (`blockd.h`).
pub type EntropyContext = i8;

/// `ENTROPY_CONTEXT_PLANES` (`blockd.h`) — the 9-byte "has any nonzero
/// coefficient" carry written by `vp8_propagate_nnz` for the MB that
/// just finished decoding. RFC 6386 §13.3.
#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct EntropyContextPlanes {
    pub y1: [EntropyContext; 4],
    pub u: [EntropyContext; 2],
    pub v: [EntropyContext; 2],
    pub y2: EntropyContext,
}

// ===========================================================================
// `MB_MODE_INFO` / `MODE_INFO`
// ===========================================================================

/// `MB_MODE_INFO` (`blockd.h`). One per macroblock in the MI grid.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct MbModeInfo {
    /// Intra or inter mode for this MB.
    pub mode: MbPredictionMode,
    /// Chroma intra mode (independent of `mode`). RFC 6386 §11.2 / §16.1.
    pub uv_mode: MbPredictionMode,
    /// Active reference frame for inter modes; `Intra` for intra MBs.
    pub ref_frame: MvReferenceFrame,
    /// True iff this MB is split into 4x4 sub-blocks (B_PRED or SPLITMV).
    pub is_4x4: bool,
    /// MB-level motion vector. RFC 6386 §16, §17.
    pub mv: Mv,
    /// SPLITMV partition selection (index into `VP8_MBSPLITS`). RFC 6386 §17.3.
    pub partitioning: u8,
    /// Skip-residual flag for this MB. RFC 6386 §13.1.
    pub mb_skip_coeff: bool,
    /// True if an MV component had to be clamped to the picture edge.
    /// RFC 6386 §16.6.
    pub need_to_clamp_mvs: bool,
    /// Active segmentation id (0..3). RFC 6386 §10.
    pub segment_id: u8,
}

/// `MODE_INFO` (`blockd.h`). One per MB-grid slot. `bmi` is only
/// meaningful when `mbmi.mode == BPred` or `SplitMv`.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ModeInfo {
    pub mbmi: MbModeInfo,
    pub bmi: [BModeInfo; 16],
}

// ===========================================================================
// `YV12_BUFFER_CONFIG`
// ===========================================================================

/// `YV12_BUFFER_CONFIG` (`yv12config.h`) — one slot in the DPB.
/// Single contiguous `buffer_alloc`, with `y_buffer/u_buffer/v_buffer`
/// offset into it by `border * stride + border`. RFC 6386 §16
/// (decoded picture buffer); libvpx-specific bookkeeping otherwise.
#[repr(C)]
pub struct Yv12BufferConfig {
    pub y_width: i32,
    pub y_height: i32,
    pub y_crop_width: i32,
    pub y_crop_height: i32,
    pub y_stride: i32,

    pub uv_width: i32,
    pub uv_height: i32,
    pub uv_crop_width: i32,
    pub uv_crop_height: i32,
    pub uv_stride: i32,

    // Alpha planes — VP8 does not produce alpha but the buffer config
    // is shared with VP9 and image scaling code, so the fields exist.
    pub alpha_width: i32,
    pub alpha_height: i32,
    pub alpha_stride: i32,

    pub y_buffer: *mut u8,
    pub u_buffer: *mut u8,
    pub v_buffer: *mut u8,
    pub alpha_buffer: *mut u8,

    pub buffer_alloc: *mut u8,
    pub buffer_alloc_sz: usize,
    pub border: i32,
    pub frame_size: usize,

    pub subsampling_x: i32,
    pub subsampling_y: i32,
    pub bit_depth: u32,
    /// Color-space tag (placeholder; `vpx_color_space_t` enum lives in
    /// the public `vpx` API and is translated separately).
    pub color_space: i32,
    pub color_range: i32,
    pub render_width: i32,
    pub render_height: i32,

    pub corrupted: i32,
    pub flags: i32,
}

// ===========================================================================
// Bool decoder
// ===========================================================================

/// `VP8_BD_VALUE` — bit-buffer register. Holds up to ~7 bytes of
/// look-ahead on a 64-bit host. `dboolhuff.h`.
pub type BdValue = usize;

/// `VP8_BD_VALUE_SIZE` — bit-width of [`BdValue`].
pub const BD_VALUE_BITS: u32 = (core::mem::size_of::<BdValue>() * 8) as u32;

/// `VP8_LOTS_OF_BITS` — sentinel added to `count` once the input is
/// exhausted, to keep `vp8dx_bool_error` cheap.
pub const VP8_LOTS_OF_BITS: i32 = 0x4000_0000;

/// Bytestream decryption callback (DRM hook). Invoked by the refill
/// path before bytes enter `value`. The C signature is
/// `void cb(void *state, const u8 *in, u8 *out, int count)`; the
/// Rust port wraps the `state` capture into the closure.
pub type DecryptFn = dyn FnMut(&[u8], &mut [u8]) + 'static;

/// Owned form, stored on `Vp8AlgPriv.decrypt` and `Vp8dComp.decrypt`.
/// Built once at `VPXD_SET_DECRYPTOR` time.
pub type DecryptCb = Box<DecryptFn>;

/// Borrowed form, handed to the bool decoder per partition.
pub type DecryptCbMut<'a> = &'a mut DecryptFn;

/// `BOOL_DECODER` (`dboolhuff.h`). The libvpx C struct keeps
/// `(user_buffer, user_buffer_end)` plus a separate cursor inside
/// `value`. We replace those with a borrowed slice plus an offset; the
/// `'a` lifetime ties the decoder to the underlying partition bytes.
pub struct BoolDecoder<'a> {
    pub buffer: &'a [u8],
    pub pos: usize,
    pub value: BdValue,
    pub count: i32,
    pub range: u32,
    /// Borrow of the codec's persistent decryption closure. None when
    /// no decryptor is installed.
    pub decrypt: Option<DecryptCbMut<'a>>,
}

/// `vp8_reader` typedef alias in `treereader.h` / `dboolhuff.h`.
pub type Vp8Reader<'a> = BoolDecoder<'a>;

// ===========================================================================
// `BLOCKD` / `MACROBLOCKD`
// ===========================================================================

/// `BLOCKD` (`blockd.h`) — per-4x4-block working context. Each entry's
/// pointer fields are set up once by `vp8_setup_block_dptrs`
/// (`mbpitch.c`) to alias into the parent [`Macroblockd`]'s scratch
/// arrays.
#[repr(C)]
pub struct Blockd {
    pub qcoeff: *mut i16,
    pub dqcoeff: *mut i16,
    pub predictor: *mut u8,
    pub dequant: *mut i16,
    /// Pixel offset from the MB's top-left into the destination plane.
    pub offset: i32,
    pub eob: *mut i8,
    pub bmi: BModeInfo,
}

/// Sub-pixel predictor function type. Matches the C signature
/// `vp8_subpix_fn_t`; one entry per supported block size lives on
/// [`Macroblockd`].
pub type SubpixFn = unsafe extern "C" fn(
    src: *mut u8,
    src_stride: i32,
    xoff: i32,
    yoff: i32,
    dst: *mut u8,
    dst_stride: i32,
);

/// `vpx_codec_err_t` (`vpx/vpx_codec.h`) — public algorithm return code.
/// `#[repr(C)]` keeps the integer ABI compatible with the C enum.
#[repr(C)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum VpxCodecErr {
    VPX_CODEC_OK = 0,
    VPX_CODEC_ERROR = 1,
    VPX_CODEC_MEM_ERROR = 2,
    VPX_CODEC_ABI_MISMATCH = 3,
    VPX_CODEC_INCAPABLE = 4,
    VPX_CODEC_UNSUP_BITSTREAM = 5,
    VPX_CODEC_UNSUP_FEATURE = 6,
    VPX_CODEC_CORRUPT_FRAME = 7,
    VPX_CODEC_INVALID_PARAM = 8,
    VPX_CODEC_LIST_END = 9,
}

pub use VpxCodecErr::*;

/// Pervasive result type for fallible decoder operations.
pub type VpxResult<T> = Result<T, VpxCodecErr>;

/// Last-error code stash, embedded in [`Vp8Common`] and [`Macroblockd`].
/// Inspectable after a failed `VpxResult`.
#[repr(C)]
pub struct VpxInternalErrorInfo {
    pub error_code: VpxCodecErr,
}

/// `MACROBLOCKD` (`blockd.h`) — the working state for one macroblock
/// during decode. Heavy alignment (`align(16)`) because libvpx's
/// reference SIMD reads `predictor` / `qcoeff` / `dqcoeff` / `eobs`
/// directly as vector registers. RFC 6386 §12–§14 (per-MB pipeline).
#[repr(C, align(16))]
pub struct Macroblockd {
    pub predictor: [u8; 384],
    pub qcoeff: [i16; 400],
    pub dqcoeff: [i16; 400],
    pub eobs: [i8; 25],

    pub dequant_y1: [i16; 16],
    pub dequant_y1_dc: [i16; 16],
    pub dequant_y2: [i16; 16],
    pub dequant_uv: [i16; 16],

    /// 16 Y + 4 U + 4 V + 1 Y2 = 25 blocks. The pointer fields inside
    /// each `Blockd` alias the arrays above.
    pub block: [Blockd; 25],
    /// Mask used to round MVs to full-pel when `full_pixel` is set.
    pub fullpixel_mask: i32,

    /// Source frame (selected reference) descriptor.
    pub pre: Yv12BufferConfig,
    /// Destination (current frame) descriptor.
    pub dst: Yv12BufferConfig,

    pub mode_info_stride: i32,

    pub frame_type: FrameType,
    pub up_available: bool,
    pub left_available: bool,

    /// Per-plane (Y, U, V) above-row reconstruction edge pointers.
    pub recon_above: [*mut u8; 3],
    /// Per-plane left-column reconstruction edge pointers.
    pub recon_left: [*mut u8; 3],
    pub recon_left_stride: [i32; 2],

    pub segmentation_enabled: u8,
    pub update_mb_segmentation_map: u8,
    pub update_mb_segmentation_data: u8,
    pub mb_segment_abs_delta: u8,
    pub mb_segment_tree_probs: [Prob; MB_FEATURE_TREE_PROBS],
    /// `[feature][segment]` — alt-Q / alt-LF values per segment.
    pub segment_feature_data: [[i8; MAX_MB_SEGMENTS]; 2],

    pub mode_ref_lf_delta_enabled: u8,
    pub mode_ref_lf_delta_update: u8,
    /// Last persisted ref-frame LF deltas (INTRA/LAST/GF/ARF).
    pub last_ref_lf_deltas: [i8; MAX_REF_LF_DELTAS],
    pub ref_lf_deltas: [i8; MAX_REF_LF_DELTAS],
    /// Mode-class LF deltas (BPRED, ZERO_MV, MV, SPLIT).
    pub last_mode_lf_deltas: [i8; MAX_MODE_LF_DELTAS],
    pub mode_lf_deltas: [i8; MAX_MODE_LF_DELTAS],

    /// 1/8-pel distance from this MB to each frame edge, used by MV
    /// clamping. RFC 6386 §16.6.
    pub mb_to_left_edge: i32,
    pub mb_to_right_edge: i32,
    pub mb_to_top_edge: i32,
    pub mb_to_bottom_edge: i32,

    pub subpixel_predict: SubpixFn,
    pub subpixel_predict8x4: SubpixFn,
    pub subpixel_predict8x8: SubpixFn,
    pub subpixel_predict16x16: SubpixFn,

    pub corrupted: i32,

    pub error_info: VpxInternalErrorInfo,
}

// ===========================================================================
// Entropy probability context
// ===========================================================================

/// `FRAME_CONTEXT` (`onyxc_int.h`) — the running entropy state for one
/// frame. `lfc` snapshots the previous frame's probs; `fc` is updated
/// in place during decode and (optionally) persisted on success.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FrameContext {
    pub bmode_prob: [Prob; VP8_BINTRAMODES - 1],
    pub ymode_prob: [Prob; VP8_YMODES - 1],
    pub uv_mode_prob: [Prob; VP8_UV_MODES - 1],
    pub sub_mv_ref_prob: [Prob; VP8_SUBMVREFS - 1],
    pub coef_probs: [[[[Prob; ENTROPY_NODES]; PREV_COEF_CONTEXTS]; COEF_BANDS]; BLOCK_TYPES],
    /// `[0] = row`, `[1] = col` MV component probs.
    pub mvc: [MvContext; 2],
}

// ===========================================================================
// Loop-filter precomputed thresholds
// ===========================================================================

/// `loop_filter_info_n` (`loopfilter.h`). Pre-derived per-strength
/// threshold tables; `lvl[seg][ref][mode]` caches the post-delta filter
/// level used by the per-MB inner loop. RFC 6386 §15.
#[repr(C)]
pub struct LoopFilterInfoN {
    pub mblim: [[u8; SIMD_WIDTH]; MAX_LOOP_FILTER + 1],
    pub blim: [[u8; SIMD_WIDTH]; MAX_LOOP_FILTER + 1],
    pub lim: [[u8; SIMD_WIDTH]; MAX_LOOP_FILTER + 1],
    pub hev_thr: [[u8; SIMD_WIDTH]; 4],
    pub lvl: [[[u8; 4]; 4]; 4],
    pub hev_thr_lut: [[u8; MAX_LOOP_FILTER + 1]; 2],
    pub mode_lf_lut: [u8; 10],
}

/// `loop_filter_info` (`loopfilter.h`) — borrowed view into one
/// strength's rows; passed by value into the per-edge filter kernels.
#[repr(C)]
pub struct LoopFilterInfo {
    pub mblim: *const u8,
    pub blim: *const u8,
    pub lim: *const u8,
    pub hev_thr: *const u8,
}

// ===========================================================================
// `VP8_HEADER` — uncompressed frame tag
// ===========================================================================

/// `VP8_HEADER` (`header.h`) — 24-bit uncompressed frame tag. RFC 6386 §9.1.
#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct Vp8Header {
    pub frame_type: u8,
    pub version: u8,
    pub show_frame: u8,
    /// 19-bit field; we store it as `u32` for simplicity.
    pub first_partition_length_in_bytes: u32,
}

// ===========================================================================
// `VP8_COMMON` — per-sequence + per-frame decoder state
// ===========================================================================

/// `VP8_COMMON` (`onyxc_int.h`). Holds all state that survives between
/// MB decodes within a frame, plus the DPB and reference indices that
/// survive between frames.
///
/// Fields gated by `CONFIG_POSTPROC`, `CONFIG_ERROR_CONCEALMENT`, and
/// `CONFIG_MULTITHREAD` in the C source are omitted to match the
/// minimal build (`vp8_only`).
#[repr(C, align(16))]
pub struct Vp8Common {
    pub error: VpxInternalErrorInfo,

    /// Dequantizer step pairs (DC, AC) for each plane, indexed by qindex.
    pub y1_dequant: [[i16; 2]; QINDEX_RANGE],
    pub y2_dequant: [[i16; 2]; QINDEX_RANGE],
    pub uv_dequant: [[i16; 2]; QINDEX_RANGE],

    pub width: i32,
    pub height: i32,
    pub horiz_scale: i32,
    pub vert_scale: i32,

    pub clamp_type: ClampType,

    /// Index of the slot in `yv12_fb` that should be returned to the
    /// user as the current output frame, or `-1` when no frame is ready.
    /// Updated by `swap_frame_buffers` (`onyxd_if.c`).
    pub frame_to_show_idx: i32,

    /// 4-slot reference picture pool.
    pub yv12_fb: [Yv12BufferConfig; NUM_YV12_BUFFERS],
    /// Refcount per slot in `yv12_fb`.
    pub fb_idx_ref_cnt: [i32; NUM_YV12_BUFFERS],
    pub new_fb_idx: i32,
    pub lst_fb_idx: i32,
    pub gld_fb_idx: i32,
    pub alt_fb_idx: i32,

    /// Scratch frame for `vp8_scale_post_processing_frame`. Allocated
    /// lazily; included because the C struct does not gate it.
    pub temp_scale_frame: Yv12BufferConfig,

    pub last_frame_type: FrameType,
    pub frame_type: FrameType,

    pub show_frame: i32,

    pub frame_flags: i32,
    /// Total MB count (`mb_rows * mb_cols`).
    pub mbs: i32,
    pub mb_rows: i32,
    pub mb_cols: i32,
    pub mode_info_stride: i32,

    pub mb_no_coeff_skip: i32,
    pub no_lpf: i32,
    pub use_bilinear_mc_filter: i32,
    pub full_pixel: i32,

    pub base_qindex: i32,
    pub y1dc_delta_q: i32,
    pub y2dc_delta_q: i32,
    pub y2ac_delta_q: i32,
    pub uvdc_delta_q: i32,
    pub uvac_delta_q: i32,

    /// MI-grid base allocation (with the one-row top and one-column
    /// left border). RFC 6386 §11.5 (neighbour-aware key-frame intra
    /// prediction needs negative indices to be valid). `None` until
    /// `vp8_alloc_frame_buffers` runs. Access via [`Vp8Common::mi`],
    /// [`Vp8Common::mi_mut`], [`Vp8Common::mi_left`] / `mi_above` /
    /// `mi_above_left`, or the per-row slice accessors
    /// [`Vp8Common::mi_row`] / [`Vp8Common::mi_row_mut`].
    pub mip: Option<Box<[ModeInfo]>>,

    pub filter_type: LoopFilterType,
    pub lf_info: LoopFilterInfoN,
    pub filter_level: i32,
    pub last_sharpness_level: i32,
    pub sharpness_level: i32,

    pub refresh_last_frame: i32,
    pub refresh_golden_frame: i32,
    pub refresh_alt_ref_frame: i32,
    pub copy_buffer_to_gf: i32,
    pub copy_buffer_to_arf: i32,
    pub refresh_entropy_probs: i32,
    /// `[INTRA..ALTREF]` — flips the MV sign for backward predictions.
    pub ref_frame_sign_bias: [i32; MAX_REF_FRAMES],

    /// Above-row entropy context (one slot per MB column). `None`
    /// until the first `vp8_alloc_frame_buffers` call.
    /// `Option<Box<[T]>>` is zero-niche so the zero-init shell produced
    /// by `Box::<Vp8dComp>::new_zeroed` leaves this as `None`. Indexed
    /// per MB by `mb_col` at the access sites in `detokenize.rs`.
    pub above_context: Option<Box<[EntropyContextPlanes]>>,
    /// Single rolling left-column context (one MB tall).
    pub left_context: EntropyContextPlanes,

    /// Last frame's entropy probabilities (backup for backward update).
    pub lfc: FrameContext,
    /// Live entropy probabilities for the frame being decoded.
    pub fc: FrameContext,

    pub current_video_frame: u32,
    pub version: i32,
    pub multi_token_partition: TokenPartition,
}

impl Vp8Common {
    /// Raw pointer to the first visible MB slot in the MI grid —
    /// equivalent to the former `mi: *mut ModeInfo` field, derived as
    /// `mip.as_mut_ptr().add(stride + 1)`. Returns null when the grid
    /// hasn't been allocated yet.
    ///
    /// The returned pointer supports the kernel's negative-offset
    /// neighbour reads (`mi.offset(-1)`, `mi.offset(-stride)`,
    /// `mi.offset(-stride - 1)`) because the corresponding entries
    /// live in the top-row / left-column padding of the underlying
    /// allocation.
    pub fn mi_base_ptr(&mut self) -> *mut ModeInfo {
        let stride = self.mode_info_stride as usize;
        match self.mip.as_deref_mut() {
            Some(slab) => unsafe { slab.as_mut_ptr().add(stride + 1) },
            None => core::ptr::null_mut(),
        }
    }

    /// Linear index of cell `(row, col)` inside the MI slab. Both
    /// arguments may be `-1` (top/left padding); the slab is sized
    /// `(mb_cols+1)*(mb_rows+1)` for exactly this purpose, so the
    /// padding cells are real (zero-initialised) entries.
    #[inline]
    fn mi_linear_index(&self, row: i32, col: i32) -> usize {
        let stride = self.mode_info_stride as usize;
        ((row + 1) as usize) * stride + ((col + 1) as usize)
    }

    /// Shared reference to the `(row, col)` cell.
    #[inline]
    pub fn mi(&self, row: i32, col: i32) -> &ModeInfo {
        let idx = self.mi_linear_index(row, col);
        &self.mip.as_deref().expect("MI grid not allocated")[idx]
    }

    /// Unique reference to the `(row, col)` cell.
    #[inline]
    pub fn mi_mut(&mut self, row: i32, col: i32) -> &mut ModeInfo {
        let idx = self.mi_linear_index(row, col);
        &mut self.mip.as_deref_mut().expect("MI grid not allocated")[idx]
    }

    /// Shared references to the neighbour cells. Always valid (the
    /// padding row+column at the top and left make `(-1, c)` /
    /// `(r, -1)` / `(-1, -1)` real entries).
    #[inline] pub fn mi_left(&self, row: i32, col: i32) -> &ModeInfo { self.mi(row, col - 1) }
    #[inline] pub fn mi_above(&self, row: i32, col: i32) -> &ModeInfo { self.mi(row - 1, col) }
    #[inline] pub fn mi_above_left(&self, row: i32, col: i32) -> &ModeInfo {
        self.mi(row - 1, col - 1)
    }

    /// `mb_cols`-long slice covering the visible MBs of `row`.
    /// Hoists the multiply out of per-MB hot loops.
    #[inline]
    pub fn mi_row_mut(&mut self, row: i32) -> &mut [ModeInfo] {
        let stride = self.mode_info_stride as usize;
        let start = ((row + 1) as usize) * stride + 1;
        let mb_cols = self.mb_cols as usize;
        &mut self.mip.as_deref_mut().expect("MI grid not allocated")[start..start + mb_cols]
    }

    /// Shared variant of [`Vp8Common::mi_row_mut`].
    #[inline]
    pub fn mi_row(&self, row: i32) -> &[ModeInfo] {
        let stride = self.mode_info_stride as usize;
        let start = ((row + 1) as usize) * stride + 1;
        let mb_cols = self.mb_cols as usize;
        &self.mip.as_deref().expect("MI grid not allocated")[start..start + mb_cols]
    }
}

// ===========================================================================
// Decoder entry-point structs
// ===========================================================================

/// `VP8D_CONFIG` (`onyxd.h`) — decoder configuration passed at create
/// time. The minimal build still carries the field (`max_threads`,
/// `postprocess`, `error_concealment`); they are read but ignored.
#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct Vp8dConfig {
    pub width: i32,
    pub height: i32,
    pub version: i32,
    pub postprocess: i32,
    pub max_threads: i32,
    pub error_concealment: i32,
}

/// `FRAGMENT_DATA` (`onyxd_int.h`) — bytestream fragments that make up
/// one frame. The decoder concatenates them lazily via the
/// `vp8_reader` refill path.
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FragmentData {
    pub enabled: i32,
    pub count: u32,
    pub ptrs: [*const u8; MAX_PARTITIONS],
    pub sizes: [u32; MAX_PARTITIONS],
}

/// `VP8D_COMP` (`onyxd_int.h`) — the top-level decoder instance.
///
/// Holds the live `Macroblockd` working state, the `Vp8Common`
/// per-frame state, and one bool decoder per partition. Fields gated
/// by `CONFIG_MULTITHREAD` / `CONFIG_ERROR_CONCEALMENT` are omitted.
// `#[repr(C)]` dropped: the `decrypt` field stores a `Box<dyn FnMut>`
// (a fat pointer with Rust-defined layout). `#[repr(align(16))]` is
// retained for the embedded `Macroblockd` SIMD alignment.
#[repr(align(16))]
pub struct Vp8dComp<'a> {
    pub mb: Macroblockd,

    /// Indices into `common.yv12_fb` for the four DPB slots, keyed by
    /// `MvReferenceFrame` (INTRA=current/new, LAST, GOLDEN, ALTREF).
    /// `-1` means "no slot bound" (not currently emitted, but allowed by
    /// the type). Set per frame at `onyxd_if.rs:vp8dx_receive_compressed_data`.
    pub dec_fb_ref_idx: [i32; NUM_YV12_BUFFERS],

    pub common: Vp8Common,

    /// `mbc[8]` is the residual / "first" partition; `mbc[0..N-1]`
    /// hold the N token partitions, where `N = 1 << multi_token_partition`.
    pub mbc: [Vp8Reader<'a>; MAX_PARTITIONS],

    pub oxcf: Vp8dConfig,

    pub fragments: FragmentData,

    pub ready_for_new_data: i32,

    /// Frame-level inter-mode tree probabilities (re-derived from
    /// `vp8_mode_contexts` each frame). RFC 6386 §16.3.
    pub prob_intra: Prob,
    pub prob_last: Prob,
    pub prob_gf: Prob,
    /// `prob_skip_false` — probability of an MB *not* having
    /// `mb_skip_coeff` set. RFC 6386 §13.1.
    pub prob_skip_false: Prob,

    pub ec_enabled: i32,
    pub ec_active: i32,
    pub decoded_key_frame: i32,
    pub independent_partitions: i32,
    pub frame_corrupt_residual: i32,

    /// Optional bytestream decryption callback. Owned for the
    /// duration of a single `vp8_decode` call: moved in from the
    /// parent `Vp8AlgPriv.decrypt` at the start of each frame, taken
    /// back at the end. Bool-decoder partitions borrow from this slot.
    pub decrypt: Option<DecryptCb>,
}

// ===========================================================================
// Per-frame and runtime post-process control structs (`vp8/common/ppflags.h`,
// `vp8/decoder/onyxd_int.h`). Unified here so every consumer agrees on
// layout.
// ===========================================================================

/// `vp8_ppflags_t` (`vp8/common/ppflags.h`) — runtime post-processing
/// flags handed to the decoder per frame. Layout is part of the public
/// API even though the minimal build never branches on the contents.
#[repr(C)]
#[derive(Copy, Clone, Default)]
pub struct Vp8PpFlags {
    pub post_proc_flag: i32,
    pub deblocking_level: i32,
    pub noise_level: i32,
    pub display_ref_frame_flag: i32,
    pub display_mb_modes_flag: i32,
    pub display_b_modes_flag: i32,
    pub display_mv_flag: i32,
}

/// `struct frame_buffers` (`vp8/decoder/onyxd_int.h:50-57`).
///
/// The C source carries a `pbi[MAX_FB_MT_DEC]` array (32 slots) for the
/// frame-parallel multithread mode. The minimal `vp8_only` build only
/// ever populates slot 0, so the Rust port collapses the array to a
/// single owned slot. Reinstating frame-parallel MT later would require
/// restoring the array (or using a `Vec`).
pub struct FrameBuffers<'a> {
    pub pbi: Option<Box<Vp8dComp<'a>>>,
}

impl<'a> FrameBuffers<'a> {
    /// Return the inner `Vp8dComp` as a raw mutable pointer for the
    /// kernel call sites that still take `*mut Vp8dComp`. Returns null
    /// when no decoder is bound.
    #[inline]
    pub fn pbi_ptr(&mut self) -> *mut Vp8dComp<'a> {
        match self.pbi.as_deref_mut() {
            Some(b) => b as *mut Vp8dComp<'a>,
            None => core::ptr::null_mut(),
        }
    }
}
