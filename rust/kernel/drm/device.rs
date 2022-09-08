// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM device
//!
//! C header: [`include/linux/drm/drm_device.h`](../../../../include/linux/drm/drm_device.h)

use crate::{bindings, device};

pub struct Device {
    // Type invariant: ptr must be a valid and initialized drm_device
    pub(crate) ptr: *mut bindings::drm_device,
}

impl Device {
    pub(crate) fn raw(&self) -> *const bindings::drm_device {
        self.ptr
    }

    pub(crate) fn raw_mut(&mut self) -> *mut bindings::drm_device {
        self.ptr
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        // SAFETY: By the type invariants, we know that `self` owns a reference, so it is safe to
        // relinquish it now.
        unsafe { bindings::drm_dev_put(self.ptr) };
    }
}

impl Clone for Device {
    fn clone(&self) -> Self {
        unsafe { bindings::drm_dev_get(self.ptr) };
        Device { ptr: self.ptr }
    }
}

// SAFETY: `Device` only holds a pointer to a C device, which is safe to be used from any thread.
unsafe impl Send for Device {}

// SAFETY: `Device` only holds a pointer to a C device, references to which are safe to be used
// from any thread.
unsafe impl Sync for Device {}

// Make drm::Device work for dev_info!() and friends
unsafe impl device::RawDevice for Device {
    fn raw_device(&self) -> *mut bindings::device {
        // SAFETY: ptr must be valid per the type invariant
        unsafe { (*self.ptr).dev }
    }
}
