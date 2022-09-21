// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM device
//!
//! C header: [`include/linux/drm/drm_device.h`](../../../../include/linux/drm/drm_device.h)

use crate::{bindings, device, drm, types::PointerWrapper};
use core::marker::PhantomData;

pub struct Device<T: drm::drv::Driver> {
    // Type invariant: ptr must be a valid and initialized drm_device
    pub(super) ptr: *mut bindings::drm_device,
    _p: PhantomData<T>,
}

impl<T: drm::drv::Driver> Device<T> {
    pub(crate) fn from_raw(raw: *mut bindings::drm_device) -> Device<T> {
        Device {
            ptr: raw,
            _p: PhantomData,
        }
    }

    pub(crate) fn raw(&self) -> *const bindings::drm_device {
        self.ptr
    }

    pub(crate) fn raw_mut(&mut self) -> *mut bindings::drm_device {
        self.ptr
    }

    pub fn data(&self) -> <T::Data as PointerWrapper>::Borrowed<'_> {
        unsafe { T::Data::borrow((*self.ptr).dev_private) }
    }
}

impl<T: drm::drv::Driver> Drop for Device<T> {
    fn drop(&mut self) {
        // SAFETY: By the type invariants, we know that `self` owns a reference, so it is safe to
        // relinquish it now.
        unsafe { bindings::drm_dev_put(self.ptr) };
    }
}

impl<T: drm::drv::Driver> Clone for Device<T> {
    fn clone(&self) -> Self {
        unsafe { bindings::drm_dev_get(self.ptr) };
        Device::from_raw(self.ptr)
    }
}

// SAFETY: `Device` only holds a pointer to a C device, which is safe to be used from any thread.
unsafe impl<T: drm::drv::Driver> Send for Device<T> {}

// SAFETY: `Device` only holds a pointer to a C device, references to which are safe to be used
// from any thread.
unsafe impl<T: drm::drv::Driver> Sync for Device<T> {}

// Make drm::Device work for dev_info!() and friends
unsafe impl<T: drm::drv::Driver> device::RawDevice for Device<T> {
    fn raw_device(&self) -> *mut bindings::device {
        // SAFETY: ptr must be valid per the type invariant
        unsafe { (*self.ptr).dev }
    }
}
