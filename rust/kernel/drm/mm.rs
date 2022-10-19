// SPDX-License-Identifier: GPL-2.0 OR MIT
#![allow(missing_docs)]

//! DRM MM range allocator
//!
//! C header: [`include/linux/drm/drm_mm.h`](../../../../include/linux/drm/drm_mm.h)

use crate::{
    bindings,
    sync::{smutex::Mutex, Arc, UniqueArc},
    to_result, Opaque, Result,
};

use alloc::boxed::Box;

use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
    pin::Pin,
};

pub type Node<T> = Pin<Box<NodeData<T>>>;

struct MmInner(Opaque<bindings::drm_mm>);

pub struct NodeData<T> {
    node: bindings::drm_mm_node,
    mm: Arc<Mutex<MmInner>>,
    valid: bool,
    inner: T,
}

unsafe impl<T: Send> Send for NodeData<T> {}
unsafe impl<T: Sync> Sync for NodeData<T> {}

#[repr(u32)]
pub enum InsertMode {
    Best = bindings::drm_mm_insert_mode_DRM_MM_INSERT_BEST,
    Low = bindings::drm_mm_insert_mode_DRM_MM_INSERT_LOW,
    High = bindings::drm_mm_insert_mode_DRM_MM_INSERT_HIGH,
    Evict = bindings::drm_mm_insert_mode_DRM_MM_INSERT_EVICT,
}

impl<T> NodeData<T> {
    pub fn color(&self) -> usize {
        self.node.color as usize
    }
    pub fn start(&self) -> u64 {
        self.node.start
    }
    pub fn size(&self) -> u64 {
        self.node.size
    }
}

impl<T> Deref for NodeData<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T> DerefMut for NodeData<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T> Drop for NodeData<T> {
    fn drop(&mut self) {
        if self.valid {
            unsafe {
                let _guard = self.mm.lock();
                bindings::drm_mm_remove_node(&mut self.node);
            }
        }
    }
}

pub struct Allocator<T> {
    mm: Arc<Mutex<MmInner>>,
    _p: PhantomData<T>,
}

impl<T> Allocator<T> {
    pub fn new(start: u64, size: u64) -> Result<Allocator<T>> {
        let mm: UniqueArc<Mutex<MmInner>> =
            UniqueArc::try_new(Mutex::new(MmInner(Opaque::uninit())))?;

        unsafe {
            bindings::drm_mm_init(mm.lock().0.get(), start, size);
        }

        Ok(Allocator {
            mm: Pin::from(mm).into(),
            _p: PhantomData,
        })
    }

    pub fn insert_node(&mut self, node: T, size: u64) -> Result<Node<T>> {
        self.insert_node_generic(node, size, 0, 0, InsertMode::Best)
    }

    pub fn insert_node_generic(
        &mut self,
        node: T,
        size: u64,
        alignment: u64,
        color: usize,
        mode: InsertMode,
    ) -> Result<Node<T>> {
        self.insert_node_in_range(node, size, alignment, color, 0, u64::MAX, mode)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn insert_node_in_range(
        &mut self,
        node: T,
        size: u64,
        alignment: u64,
        color: usize,
        start: u64,
        end: u64,
        mode: InsertMode,
    ) -> Result<Node<T>> {
        let mut mm_node = Box::try_new(NodeData {
            node: unsafe { core::mem::zeroed() },
            valid: false,
            inner: node,
            mm: self.mm.clone(),
        })?;

        to_result(unsafe {
            bindings::drm_mm_insert_node_in_range(
                self.mm.lock().0.get(),
                &mut mm_node.node,
                size,
                alignment,
                color as core::ffi::c_ulong,
                start,
                end,
                mode as u32,
            )
        })?;

        mm_node.valid = true;

        Ok(Pin::from(mm_node))
    }
}

impl Drop for MmInner {
    fn drop(&mut self) {
        unsafe {
            bindings::drm_mm_takedown(self.0.get());
        }
    }
}

unsafe impl Send for MmInner {}
