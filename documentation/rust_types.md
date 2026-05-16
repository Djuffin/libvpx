# VP8 decoder backbone types — Rust sketches

Concrete Rust translations of the backbone types identified in the
header scan. Buffers, plane storage, and self-referential structs all
use the **raw `*mut u8` everywhere** model — a direct C
transliteration. Every method body that touches a pointer field is
`unsafe`; the goal is bit-identical behavior to the C original.

---

## Atoms

```rust
#[derive(Copy, Clone, Default, PartialEq, Eq)]
#[repr(C)]
pub struct Mv {
    pub row: i16,
    pub col: i16,
}

// The C `int_mv` union — used so MV equality/copy is a single 32-bit op.
// In Rust we just derive Copy + Eq on Mv; #[repr(C)] keeps the layout.
// If you specifically need the `as_int` view (e.g. for FFI), expose it:
impl Mv {
    #[inline] pub fn as_int(self) -> u32 { unsafe { core::mem::transmute(self) } }
    #[inline] pub fn from_int(i: u32) -> Self { unsafe { core::mem::transmute(i) } }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct Pos { pub r: i32, pub c: i32 }

#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameType { Key = 0, Inter = 1 }

#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum MbPredictionMode {
    DcPred, VPred, HPred, TmPred, BPred,
    NearestMv, NearMv, ZeroMv, NewMv, SplitMv,
}

#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum BPredictionMode {
    DcPred, TmPred, VePred, HePred,
    LdPred, RdPred, VrPred, VlPred, HdPred, HuPred,
    // 4x4 sub-MV reference modes (only used inside SPLITMV b_mode_info)
    Left4x4, Above4x4, Zero4x4, New4x4,
}

#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum MvReferenceFrame {
    Intra = 0, Last = 1, Golden = 2, Altref = 3,
}
```

## `union b_mode_info`

The C union is `B_PREDICTION_MODE` *or* `int_mv` depending on whether
the parent MB is `B_PRED` (intra) or `SPLITMV` (inter). In Rust this
is exactly what `enum` was made for:

```rust
#[derive(Copy, Clone)]
pub enum BModeInfo {
    Intra(BPredictionMode),
    Mv(Mv),
}
```

If you care about size parity with the C `union` (which is 4 bytes),
wrap a `MaybeUninit<u32>` and discriminate externally — but
`BModeInfo` above is already 4 bytes thanks to niche optimization on
the enum tag.

## Entropy context

```rust
pub type EntropyContext = i8;

#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct EntropyContextPlanes {
    pub y1: [EntropyContext; 4],
    pub u:  [EntropyContext; 2],
    pub v:  [EntropyContext; 2],
    pub y2: EntropyContext,
}
```

## `MB_MODE_INFO` / `MODE_INFO`

```rust
#[derive(Copy, Clone)]
#[repr(C)]
pub struct MbModeInfo {
    pub mode:               MbPredictionMode,
    pub uv_mode:            MbPredictionMode,
    pub ref_frame:          MvReferenceFrame,
    pub is_4x4:             bool,
    pub mv:                 Mv,
    pub partitioning:       u8,
    pub mb_skip_coeff:      bool,
    pub need_to_clamp_mvs:  bool,
    pub segment_id:         u8,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ModeInfo {
    pub mbmi: MbModeInfo,
    pub bmi:  [BModeInfo; 16],
}
```

## `YV12_BUFFER_CONFIG`

Direct C transliteration with raw pointers. Stores raw `*mut u8`
plane pointers + `int` strides and a separately-owned `buffer_alloc`:

```rust
#[repr(C)]
pub struct YV12BufferConfig {
    pub y_width:  i32, pub y_height: i32, pub y_stride: i32,
    pub uv_width: i32, pub uv_height: i32, pub uv_stride: i32,
    pub y_buffer: *mut u8,
    pub u_buffer: *mut u8,
    pub v_buffer: *mut u8,
    pub buffer_alloc:    *mut u8,
    pub buffer_alloc_sz: usize,
    pub border: i32,
    pub frame_size: usize,
    pub corrupted: bool,
    // ... omitting alpha + render fields for brevity
}
```

## `BOOL_DECODER`

```rust
pub type BdValue = usize;
pub const BD_VALUE_BITS: u32 = (core::mem::size_of::<BdValue>() * 8) as u32;
pub const LOTS_OF_BITS:  i32 = 0x4000_0000;

pub struct BoolDecoder<'a> {
    buffer:        &'a [u8],   // replaces (user_buffer, user_buffer_end)
    pos:           usize,
    pub value:     BdValue,
    pub count:     i32,
    pub range:     u32,
    // Decryption callback is rarely used — model as Option:
    decrypt: Option<DecryptCb<'a>>,
}

pub type DecryptCb<'a> = Box<dyn FnMut(&[u8], &mut [u8]) + 'a>;

impl<'a> BoolDecoder<'a> {
    pub fn new(buf: &'a [u8]) -> Self { /* sets value/range/count from first bytes */ }
    pub fn read_bool(&mut self, prob: u8) -> bool { /* hot path */ }
    pub fn read_literal(&mut self, bits: u32) -> u32 { /* loop of read_bool(128) */ }
    pub fn read_signed_literal(&mut self, bits: u32) -> i32 { /* ... */ }
    pub fn has_error(&self) -> bool { self.pos > self.buffer.len() + LOTS_OF_BITS as usize }
}
```

The lifetime tie `BoolDecoder<'a>` is genuine improvement over C —
the borrow checker enforces that the input partition slice outlives
the decoder. This catches a class of fragmented-input bugs that exist
in libvpx today.

## `BLOCKD`

Holds raw pointers into its parent `MACROBLOCKD`'s arrays (`qcoeff`,
`dqcoeff`, `predictor`, `dequant`). Matches C 1:1; self-referential
is fine because every access is `unsafe`:

```rust
#[repr(C)]
pub struct Blockd {
    pub qcoeff:    *mut i16,
    pub dqcoeff:   *mut i16,
    pub predictor: *mut u8,
    pub dequant:   *mut i16,
    pub offset:    i32,
    pub eob:       *mut i8,
    pub bmi:       BModeInfo,
}
```

## `MACROBLOCKD`

Raw pointers, direct C transliteration:

```rust
#[repr(C, align(16))]
pub struct Macroblockd {
    pub predictor: [u8;  384],
    pub qcoeff:    [i16; 400],
    pub dqcoeff:   [i16; 400],
    pub eobs:      [i8;  25],

    pub dequant_y1:    [i16; 16],
    pub dequant_y1_dc: [i16; 16],
    pub dequant_y2:    [i16; 16],
    pub dequant_uv:    [i16; 16],

    pub block: [Blockd; 25],       // self-referencing pointers into above arrays
    pub fullpixel_mask: i32,

    pub pre: YV12BufferConfig,     // by-value descriptor (raw ptr inside)
    pub dst: YV12BufferConfig,

    pub mode_info_context: *mut ModeInfo,
    pub mode_info_stride:  i32,

    pub frame_type:    FrameType,
    pub up_available:  bool,
    pub left_available:bool,

    pub recon_above:  [*mut u8; 3],
    pub recon_left:   [*mut u8; 3],
    pub recon_left_stride: [i32; 2],

    pub above_context: *mut EntropyContextPlanes,
    pub left_context:  *mut EntropyContextPlanes,

    pub segmentation_enabled:         u8,
    pub update_mb_segmentation_map:   u8,
    pub update_mb_segmentation_data:  u8,
    pub mb_segment_abs_delta:         u8,
    pub mb_segment_tree_probs:        [u8; 3],
    pub segment_feature_data:         [[i8; 4]; 2],

    pub mode_ref_lf_delta_enabled: u8,
    pub mode_ref_lf_delta_update:  u8,
    pub last_ref_lf_deltas:  [i8; 4],
    pub ref_lf_deltas:       [i8; 4],
    pub last_mode_lf_deltas: [i8; 4],
    pub mode_lf_deltas:      [i8; 4],

    pub mb_to_left_edge:   i32,
    pub mb_to_right_edge:  i32,
    pub mb_to_top_edge:    i32,
    pub mb_to_bottom_edge: i32,

    pub subpixel_predict:       SubpixFn,   // fn pointer
    pub subpixel_predict8x4:    SubpixFn,
    pub subpixel_predict8x8:    SubpixFn,
    pub subpixel_predict16x16:  SubpixFn,

    pub current_bc: *mut core::ffi::c_void,   // erased BoolDecoder
    pub corrupted:  i32,
}

pub type SubpixFn = unsafe extern "C" fn(
    src: *mut u8, src_stride: i32,
    xoff: i32, yoff: i32,
    dst: *mut u8, dst_stride: i32,
);
```

Honest, ugly, fast to write. Every method body is `unsafe`. Almost
guaranteed bit-identical to C if you transliterate carefully.

## `FRAME_CONTEXT`

```rust
pub struct FrameContext {
    pub bmode_prob:      [Prob; VP8_BINTRAMODES - 1],
    pub ymode_prob:      [Prob; VP8_YMODES - 1],
    pub uv_mode_prob:    [Prob; VP8_UV_MODES - 1],
    pub sub_mv_ref_prob: [Prob; VP8_SUBMVREFS - 1],
    pub coef_probs:      [[[[Prob; ENTROPY_NODES]; PREV_COEF_CONTEXTS]; COEF_BANDS]; BLOCK_TYPES],
    pub mvc: [MvContext; 2],
}

pub type Prob = u8;

#[derive(Copy, Clone)]
pub struct MvContext {
    pub probs: [Prob; 19],   // sign + short-tree(7) + long-bit-probs(10) + short-prob(1)
}
```

## `VP8_COMMON`

The big one. Field-by-field, kept honest to the C original:

```rust
pub struct Vp8Common {
    pub error: vpx_internal_error_info,    // FFI shim type

    pub y1_dequant: [[i16; 2]; QINDEX_RANGE],   // 128 * 2
    pub y2_dequant: [[i16; 2]; QINDEX_RANGE],
    pub uv_dequant: [[i16; 2]; QINDEX_RANGE],

    pub width:  u32,
    pub height: u32,
    pub horiz_scale: u8,
    pub vert_scale:  u8,
    pub clamp_type:  ClampType,

    // The DPB: 4 buffers + ref counts + active indices.
    pub yv12_fb:        [YV12BufferConfig; NUM_YV12_BUFFERS],
    pub fb_idx_ref_cnt: [i32;               NUM_YV12_BUFFERS],
    pub new_fb_idx:  i8,
    pub lst_fb_idx:  i8,
    pub gld_fb_idx:  i8,
    pub alt_fb_idx:  i8,
    pub frame_to_show: i8,    // index into yv12_fb

    pub last_frame_type: FrameType,
    pub frame_type:      FrameType,
    pub show_frame:      bool,

    pub mbs:      u32,
    pub mb_rows:  u32,
    pub mb_cols:  u32,
    pub mode_info_stride: u32,

    pub mb_no_coeff_skip:     bool,
    pub no_lpf:               bool,
    pub use_bilinear_mc_filter: bool,
    pub full_pixel:           bool,

    pub base_qindex:    u8,
    pub y1dc_delta_q:   i8,
    pub y2dc_delta_q:   i8,
    pub y2ac_delta_q:   i8,
    pub uvdc_delta_q:   i8,
    pub uvac_delta_q:   i8,

    // The MI grid. `mip` is the raw allocation (with border row/col);
    // `mi` is the offset into the first visible MB.
    pub mip: Box<[ModeInfo]>,
    pub mi_offset: usize,
    pub show_frame_mi_offset: usize,

    pub filter_type: LoopFilterType,
    pub lf_info:     LoopFilterInfoN,
    pub filter_level:       u8,
    pub last_sharpness_level: u8,
    pub sharpness_level:    u8,

    pub refresh_last_frame:    bool,
    pub refresh_golden_frame:  bool,
    pub refresh_alt_ref_frame: bool,
    pub copy_buffer_to_gf:     u8,
    pub copy_buffer_to_arf:    u8,
    pub refresh_entropy_probs: bool,
    pub ref_frame_sign_bias: [bool; MAX_REF_FRAMES],

    // Entropy context for the MB row currently being decoded.
    pub above_context: Box<[EntropyContextPlanes]>,   // mb_cols long
    pub left_context:  EntropyContextPlanes,

    pub lfc: FrameContext,   // last frame's probs (saved for backward update)
    pub fc:  FrameContext,   // this frame's probs

    pub current_video_frame: u32,
    pub version: u8,
    pub multi_token_partition: TokenPartition,
}
```
