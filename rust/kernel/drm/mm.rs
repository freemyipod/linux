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

pub type Node<A, T> = Pin<Box<NodeData<A, T>>>;

pub trait AllocInner<T> {
    fn drop_object(&mut self, _start: u64, _size: u64, _color: usize, _object: &mut T) {}
}

impl<T> AllocInner<T> for () {}

struct MmInner<A: AllocInner<T>, T>(Opaque<bindings::drm_mm>, A, PhantomData<T>);

pub struct NodeData<A: AllocInner<T>, T> {
    node: bindings::drm_mm_node,
    mm: Arc<Mutex<MmInner<A, T>>>,
    valid: bool,
    inner: T,
}

unsafe impl<A: Send + AllocInner<T>, T: Send> Send for NodeData<A, T> {}
unsafe impl<A: Send + AllocInner<T>, T: Sync> Sync for NodeData<A, T> {}

#[repr(u32)]
pub enum InsertMode {
    Best = bindings::drm_mm_insert_mode_DRM_MM_INSERT_BEST,
    Low = bindings::drm_mm_insert_mode_DRM_MM_INSERT_LOW,
    High = bindings::drm_mm_insert_mode_DRM_MM_INSERT_HIGH,
    Evict = bindings::drm_mm_insert_mode_DRM_MM_INSERT_EVICT,
}

impl<A: AllocInner<T>, T> NodeData<A, T> {
    pub fn color(&self) -> usize {
        self.node.color as usize
    }
    pub fn start(&self) -> u64 {
        self.node.start
    }
    pub fn size(&self) -> u64 {
        self.node.size
    }
    pub fn with_inner<RetVal>(&self, cb: impl FnOnce(&mut A) -> RetVal) -> RetVal {
        let mut l = self.mm.lock();
        cb(&mut l.1)
    }
}

impl<A: AllocInner<T>, T> Deref for NodeData<A, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<A: AllocInner<T>, T> DerefMut for NodeData<A, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<A: AllocInner<T>, T> Drop for NodeData<A, T> {
    fn drop(&mut self) {
        if self.valid {
            let mut guard = self.mm.lock();

            guard
                .1
                .drop_object(self.start(), self.size(), self.color(), &mut self.inner);
            unsafe { bindings::drm_mm_remove_node(&mut self.node) };
        }
    }
}

pub struct Allocator<A: AllocInner<T>, T> {
    mm: Arc<Mutex<MmInner<A, T>>>,
    _p: PhantomData<T>,
}

impl<A: AllocInner<T>, T> Allocator<A, T> {
    pub fn new(start: u64, size: u64, inner: A) -> Result<Allocator<A, T>> {
        let mm: UniqueArc<Mutex<MmInner<A, T>>> =
            UniqueArc::try_new(Mutex::new(MmInner(Opaque::uninit(), inner, PhantomData)))?;

        unsafe {
            bindings::drm_mm_init(mm.lock().0.get(), start, size);
        }

        Ok(Allocator {
            mm: Pin::from(mm).into(),
            _p: PhantomData,
        })
    }

    pub fn insert_node(&mut self, node: T, size: u64) -> Result<Node<A, T>> {
        self.insert_node_generic(node, size, 0, 0, InsertMode::Best)
    }

    pub fn insert_node_generic(
        &mut self,
        node: T,
        size: u64,
        alignment: u64,
        color: usize,
        mode: InsertMode,
    ) -> Result<Node<A, T>> {
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
    ) -> Result<Node<A, T>> {
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

    #[allow(clippy::too_many_arguments)]
    pub fn reserve_node(
        &mut self,
        node: T,
        start: u64,
        size: u64,
        color: usize,
    ) -> Result<Node<A, T>> {
        let mut mm_node = Box::try_new(NodeData {
            node: unsafe { core::mem::zeroed() },
            valid: false,
            inner: node,
            mm: self.mm.clone(),
        })?;

        mm_node.node.start = start;
        mm_node.node.size = size;
        mm_node.node.color = color as core::ffi::c_ulong;

        to_result(unsafe {
            bindings::drm_mm_reserve_node(self.mm.lock().0.get(), &mut mm_node.node)
        })?;

        mm_node.valid = true;

        Ok(Pin::from(mm_node))
    }

    pub fn with_inner<RetVal>(&self, cb: impl FnOnce(&mut A) -> RetVal) -> RetVal {
        let mut l = self.mm.lock();
        cb(&mut l.1)
    }
}

impl<A: AllocInner<T>, T> Drop for MmInner<A, T> {
    fn drop(&mut self) {
        unsafe {
            bindings::drm_mm_takedown(self.0.get());
        }
    }
}

unsafe impl<A: Send + AllocInner<T>, T> Send for MmInner<A, T> {}
