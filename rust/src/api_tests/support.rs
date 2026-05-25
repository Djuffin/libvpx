use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufRead, BufReader, Read};
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::ptr::NonNull;
use std::sync::{Arc, Mutex};

use crate::api::*;

const TEST_DATA_DIR: &str = env!("VP8_TEST_DATA_DIR");

pub fn test_data_dir(test_name: &str) -> Option<PathBuf> {
    if TEST_DATA_DIR.is_empty() {
        eprintln!(
            "skipping {test_name}: VP8 test data not available \
             (set LIBVPX_TEST_DATA_PATH or ensure network access at build time)"
        );
        return None;
    }
    Some(PathBuf::from(TEST_DATA_DIR))
}

pub struct IvfReader {
    inner: BufReader<File>,
}

impl IvfReader {
    pub fn open(path: &PathBuf) -> std::io::Result<Self> {
        let mut inner = BufReader::new(File::open(path)?);
        let mut hdr = [0u8; 32];
        inner.read_exact(&mut hdr)?;
        assert_eq!(&hdr[0..4], b"DKIF", "{path:?} missing DKIF magic");
        Ok(Self { inner })
    }

    pub fn next_frame(&mut self) -> Option<Vec<u8>> {
        let mut hdr = [0u8; 12];
        if self.inner.read_exact(&mut hdr).is_err() {
            return None;
        }
        let size = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]) as usize;
        let mut buf = vec![0u8; size];
        self.inner.read_exact(&mut buf).ok()?;
        Some(buf)
    }
}

pub fn read_ivf_first_packet(path: &PathBuf) -> Vec<u8> {
    let mut reader = IvfReader::open(path).expect("open ivf");
    reader.next_frame().expect("read first frame")
}

pub fn read_md5_lines(path: &PathBuf) -> Vec<String> {
    let f = File::open(path).expect("open .md5 file");
    BufReader::new(f)
        .lines()
        .map(|l| {
            let l = l.expect("read .md5 line");
            l.split_whitespace().next().expect("md5 hex").to_owned()
        })
        .collect()
}

#[derive(Default)]
pub struct CountingCallbacks {
    pub pictures: AtomicUsize,
    pub format_changes: AtomicUsize,
    pub last_format: Mutex<Option<StreamFormat>>,
}

impl VideoDecoderCallbacks for CountingCallbacks {
    fn on_picture_available(&self) {
        self.pictures.fetch_add(1, Ordering::Relaxed);
    }
    fn on_format_changed(&self, format: StreamFormat) {
        self.format_changes.fetch_add(1, Ordering::Relaxed);
        *self.last_format.lock().unwrap() = Some(format);
    }
}

impl CountingCallbacks {
    pub fn shared() -> Arc<Self> {
        Arc::new(Self::default())
    }

    pub fn picture_callbacks(&self) -> usize {
        self.pictures.load(Ordering::Relaxed)
    }

    pub fn format_change_count(&self) -> usize {
        self.format_changes.load(Ordering::Relaxed)
    }
}

pub struct TrackingAllocator {
    inner: DefaultAllocator,
    pub alloc_count: AtomicUsize,
    pub fail_with: Mutex<Option<AllocError>>,
    pub last_request: Mutex<Option<BufferAllocation>>,
}

impl TrackingAllocator {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            inner: DefaultAllocator,
            alloc_count: AtomicUsize::new(0),
            fail_with: Mutex::new(None),
            last_request: Mutex::new(None),
        })
    }

    pub fn set_failure(&self, err: AllocError) {
        *self.fail_with.lock().unwrap() = Some(err);
    }

    pub fn count(&self) -> usize {
        self.alloc_count.load(Ordering::Relaxed)
    }
}

impl VideoFrameAllocator for TrackingAllocator {
    fn alloc_frame(
        &self,
        alloc: &BufferAllocation,
    ) -> Result<Box<dyn FrameBuffer>, AllocError> {
        self.alloc_count.fetch_add(1, Ordering::Relaxed);
        *self.last_request.lock().unwrap() = Some(*alloc);
        if let Some(err) = self.fail_with.lock().unwrap().clone() {
            return Err(err);
        }
        self.inner.alloc_frame(alloc)
    }
}

struct PooledFrameBuffer {
    inner: Option<Box<dyn FrameBuffer>>,
    pool: Arc<Mutex<VecDeque<Box<dyn FrameBuffer>>>>,
}

impl FrameBuffer for PooledFrameBuffer {
    fn plane_ptr(&self, plane: VideoPlane) -> Option<NonNull<[u8]>> {
        self.inner.as_ref().and_then(|b| b.plane_ptr(plane))
    }
}

impl Drop for PooledFrameBuffer {
    fn drop(&mut self) {
        if let Some(buf) = self.inner.take() {
            let mut list = self.pool.lock().unwrap();
            list.push_back(buf);
        }
    }
}

pub struct PoolAllocator {
    inner: DefaultAllocator,
    pub alloc_count: AtomicUsize,
    pool: Arc<Mutex<VecDeque<Box<dyn FrameBuffer>>>>,
}

impl PoolAllocator {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            inner: DefaultAllocator,
            alloc_count: AtomicUsize::new(0),
            pool: Arc::new(Mutex::new(VecDeque::new())),
        })
    }

    pub fn count(&self) -> usize {
        self.alloc_count.load(Ordering::Relaxed)
    }
}

impl VideoFrameAllocator for PoolAllocator {
    fn alloc_frame(
        &self,
        alloc: &BufferAllocation,
    ) -> Result<Box<dyn FrameBuffer>, AllocError> {
        let mut list = self.pool.lock().unwrap();

        // Scan the pool for any recycled buffer that is large enough:
        let found_idx = list.iter().position(|buf| {
            for plane_alloc in alloc.planes.iter().flatten() {
                if let Some(slice) = buf.plane_ptr(plane_alloc.plane) {
                    if slice.len() < plane_alloc.size_bytes {
                        return false;
                    }
                } else {
                    return false;
                }
            }
            true
        });

        let inner_buf = if let Some(idx) = found_idx {
            list.remove(idx).unwrap()
        } else {
            // Cache miss: allocate fresh memory
            self.alloc_count.fetch_add(1, Ordering::Relaxed);
            self.inner.alloc_frame(alloc)?
        };

        Ok(Box::new(PooledFrameBuffer {
            inner: Some(inner_buf),
            pool: self.pool.clone(),
        }))
    }
}
