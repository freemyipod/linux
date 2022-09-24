// SPDX-License-Identifier: GPL-2.0 OR MIT
#![allow(missing_docs)]

//! DRM MM range allocator
//!
//! C header: [`include/linux/drm/drm_mm.h`](../../../../include/linux/drm/drm_mm.h)

use crate::{bindings, to_result, Opaque, Result};

use alloc::boxed::Box;

use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
    pin::Pin,
};

pub type Node<T> = Pin<Box<NodeData<T>>>;

pub struct NodeData<T> {
    node: bindings::drm_mm_node,
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
        // SAFETY: TODO: Make sure self outlives the Allocator<Self>
        unsafe {
            bindings::drm_mm_remove_node(&mut self.node);
        }
    }
}

pub struct Allocator<T> {
    mm: Pin<Box<Opaque<bindings::drm_mm>>>,
    _p: PhantomData<T>,
}

impl<T> Allocator<T> {
    pub fn new(start: u64, size: u64) -> Result<Allocator<T>> {
        let mm: Box<Opaque<bindings::drm_mm>> = Box::try_new(Opaque::uninit())?;

        unsafe {
            bindings::drm_mm_init(mm.get(), start, size);
        }

        Ok(Allocator {
            mm: Pin::from(mm),
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
            inner: node,
        })?;

        to_result(unsafe {
            bindings::drm_mm_insert_node_in_range(
                self.mm.get(),
                &mut mm_node.node,
                size,
                alignment,
                color as core::ffi::c_ulong,
                start,
                end,
                mode as u32,
            )
        })?;

        Ok(Pin::from(mm_node))
    }
}

impl<T> Drop for Allocator<T> {
    fn drop(&mut self) {
        unsafe {
            bindings::drm_mm_takedown(self.mm.get());
        }
    }
}

unsafe impl<T> Send for Allocator<T> {}
