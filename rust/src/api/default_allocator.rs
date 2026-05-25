use std::alloc::{self, Layout};
use std::ptr::NonNull;

use super::color::VideoPlane;
use super::frame::{
    AllocError, BufferAllocation, FrameBuffer, PlaneAllocation, VideoFrameAllocator,
};

/// A stock `VideoFrameAllocator` that backs each plane with an
/// aligned heap allocation. Provided for callers who don't need a
/// pool or custom storage.
#[derive(Default, Clone, Copy)]
pub struct DefaultAllocator;

impl DefaultAllocator {
    pub fn new() -> Self {
        Self
    }
}

impl VideoFrameAllocator for DefaultAllocator {
    fn alloc_frame(
        &self,
        alloc: &BufferAllocation,
    ) -> Result<Box<dyn FrameBuffer>, AllocError> {
        let mut buffers: [Option<PlaneBuffer>; 4] = [None, None, None, None];
        for (slot, plane_alloc) in buffers.iter_mut().zip(alloc.planes.iter()) {
            if let Some(pa) = plane_alloc {
                *slot = Some(PlaneBuffer::new(pa)?);
            }
        }
        Ok(Box::new(DefaultFrameBuffer { buffers }))
    }
}

struct DefaultFrameBuffer {
    buffers: [Option<PlaneBuffer>; 4],
}

impl FrameBuffer for DefaultFrameBuffer {
    fn plane_ptr(&self, plane: VideoPlane) -> Option<NonNull<[u8]>> {
        self.buffers.iter().flatten()
            .find(|b| b.plane == plane)
            .map(|b| {
                let slice_ptr = std::ptr::slice_from_raw_parts_mut(b.ptr.as_ptr(), b.layout.size());
                NonNull::new(slice_ptr).unwrap()
            })
    }
}

struct PlaneBuffer {
    plane: VideoPlane,
    ptr: NonNull<u8>,
    layout: Layout,
}

impl PlaneBuffer {
    fn new(req: &PlaneAllocation) -> Result<Self, AllocError> {
        if req.size_bytes == 0 {
            return Err(AllocError::OutOfMemory);
        }
        let layout = Layout::from_size_align(req.size_bytes, req.alignment)
            .map_err(|_| AllocError::UnsupportedAlignment)?;
        let raw = unsafe { alloc::alloc_zeroed(layout) };
        let ptr = NonNull::new(raw).ok_or(AllocError::OutOfMemory)?;
        Ok(Self { plane: req.plane, ptr, layout })
    }
}

impl Drop for PlaneBuffer {
    fn drop(&mut self) {
        unsafe { alloc::dealloc(self.ptr.as_ptr(), self.layout) };
    }
}

unsafe impl Send for PlaneBuffer {}
unsafe impl Sync for PlaneBuffer {}
