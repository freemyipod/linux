// SPDX-License-Identifier: GPL-2.0 OR MIT
#![allow(missing_docs)]

//! DRM GEM API
//!
//! C header: [`include/linux/drm/drm_gem.h`](../../../../include/linux/drm/drm_gem.h)

#[cfg(CONFIG_DRM_GEM_SHMEM_HELPER)]
pub mod shmem;

use alloc::boxed::Box;

use crate::{
    bindings,
    drm::{device, drv, file, private},
    prelude::*,
    to_result, Result,
};
use core::{mem, mem::ManuallyDrop, ops::Deref, ops::DerefMut};

/// GEM object functions
pub trait BaseDriverObject<T: BaseObject>: Sync + Send + Sized {
    fn new(dev: &device::Device<T::Driver>, size: usize) -> Result<Self>;
}

pub trait IntoGEMObject: Sized + private::Sealed {
    type Driver: drv::Driver;

    fn gem_obj(&self) -> &bindings::drm_gem_object;
}

pub trait BaseObject: IntoGEMObject {
    fn size(&self) -> usize;
    fn reference(&self) -> ObjectRef<Self>;
    fn create_handle(
        &self,
        file: &file::File<<<Self as IntoGEMObject>::Driver as drv::Driver>::File>,
    ) -> Result<u32>;
    fn lookup_handle(
        file: &file::File<<<Self as IntoGEMObject>::Driver as drv::Driver>::File>,
        handle: u32,
    ) -> Result<ObjectRef<Self>>;
    fn create_mmap_offset(&self) -> Result<u64>;
}

#[repr(C)]
pub struct Object<T: DriverObject> {
    obj: bindings::drm_gem_object,
    dev: ManuallyDrop<device::Device<T::Driver>>,
    inner: T,
}

pub trait DriverObject: BaseDriverObject<Object<Self>> {
    type Driver: drv::Driver;
}

pub struct ObjectRef<T: IntoGEMObject> {
    // Invariant: the pointer is valid and initialized, and this ObjectRef owns a reference to it
    ptr: *const T,
}

pub struct UniqueObjectRef<T: IntoGEMObject> {
    // Invariant: the pointer is valid and initialized, and this ObjectRef owns the only reference to it
    ptr: *mut T,
}

unsafe extern "C" fn free_callback<T: DriverObject>(obj: *mut bindings::drm_gem_object) {
    // SAFETY: All of our objects are Object<T>.
    let this = crate::container_of!(obj, Object<T>, obj) as *mut Object<T>;

    // SAFETY: The pointer we got has to be valid
    unsafe { bindings::drm_gem_object_release(obj) };

    // SAFETY: All of our objects are allocated via Box<>, and we're in the
    // free callback which guarantees this object has zero remaining references,
    // so we can drop it
    unsafe { Box::from_raw(this) };
}

impl<T: DriverObject> IntoGEMObject for Object<T> {
    type Driver = T::Driver;

    fn gem_obj(&self) -> &bindings::drm_gem_object {
        &self.obj
    }
}

impl<T: IntoGEMObject> BaseObject for T {
    fn size(&self) -> usize {
        self.gem_obj().size
    }

    fn reference(&self) -> ObjectRef<Self> {
        // SAFETY: Having a reference to an Object implies holding a GEM reference
        unsafe {
            bindings::drm_gem_object_get(self.gem_obj() as *const _ as *mut _);
        }
        ObjectRef {
            ptr: self as *const _,
        }
    }

    fn create_handle(
        &self,
        file: &file::File<<<Self as IntoGEMObject>::Driver as drv::Driver>::File>,
    ) -> Result<u32> {
        let mut handle: u32 = 0;
        to_result(unsafe {
            bindings::drm_gem_handle_create(
                file.raw() as *mut _,
                self.gem_obj() as *const _ as *mut _,
                &mut handle,
            )
        })?;
        Ok(handle)
    }

    fn lookup_handle(
        file: &file::File<<<Self as IntoGEMObject>::Driver as drv::Driver>::File>,
        handle: u32,
    ) -> Result<ObjectRef<Self>> {
        let ptr = unsafe { bindings::drm_gem_object_lookup(file.raw() as *mut _, handle) };

        if ptr.is_null() {
            Err(ENOENT)
        } else {
            Ok(ObjectRef {
                ptr: ptr as *const _,
            })
        }
    }

    fn create_mmap_offset(&self) -> Result<u64> {
        to_result(unsafe {
            // TODO: is this threadsafe?
            bindings::drm_gem_create_mmap_offset(self.gem_obj() as *const _ as *mut _)
        })?;
        Ok(unsafe {
            bindings::drm_vma_node_offset_addr(&self.gem_obj().vma_node as *const _ as *mut _)
        })
    }
}

impl<T: DriverObject> private::Sealed for Object<T> {}

impl<T: DriverObject> drv::AllocImpl for Object<T> {
    const ALLOC_OPS: drv::AllocOps = drv::AllocOps {
        gem_create_object: None,
        prime_handle_to_fd: Some(bindings::drm_gem_prime_handle_to_fd),
        prime_fd_to_handle: Some(bindings::drm_gem_prime_fd_to_handle),
        gem_prime_import: None,
        gem_prime_import_sg_table: None,
        gem_prime_mmap: Some(bindings::drm_gem_prime_mmap),
        dumb_create: None,
        dumb_map_offset: None,
        dumb_destroy: None,
    };
}

impl<T: DriverObject> Object<T> {
    pub const SIZE: usize = mem::size_of::<Self>();

    const OBJECT_FUNCS: bindings::drm_gem_object_funcs = bindings::drm_gem_object_funcs {
        free: Some(free_callback::<T>),
        open: None,
        close: None,
        print_info: None,
        export: None,
        pin: None,
        unpin: None,
        get_sg_table: None,
        vmap: None,
        vunmap: None,
        mmap: None,
        vm_ops: core::ptr::null_mut(),
    };

    pub fn new(dev: &device::Device<T::Driver>, size: usize) -> Result<UniqueObjectRef<Self>> {
        let mut obj: Box<Self> = Box::try_new(Self {
            // SAFETY: This struct is expected to be zero-initialized
            obj: unsafe { mem::zeroed() },
            // SAFETY: The drm subsystem guarantees that the drm_device will live as long as
            // the GEM object lives, so we can conjure a reference out of thin air.
            dev: ManuallyDrop::new(unsafe { device::Device::from_raw(dev.ptr) }),
            inner: T::new(dev, size)?,
        })?;

        obj.obj.funcs = &Self::OBJECT_FUNCS;
        to_result(unsafe {
            bindings::drm_gem_object_init(dev.raw() as *mut _, &mut obj.obj, size)
        })?;

        let obj_ref = UniqueObjectRef {
            ptr: Box::leak(obj),
        };

        Ok(obj_ref)
    }

    pub fn dev(&self) -> &device::Device<T::Driver> {
        &self.dev
    }
}

impl<T: IntoGEMObject> Clone for ObjectRef<T> {
    fn clone(&self) -> Self {
        self.reference()
    }
}

impl<T: IntoGEMObject> Drop for ObjectRef<T> {
    fn drop(&mut self) {
        // SAFETY: Having an ObjectRef implies holding a GEM reference.
        // The free callback will take care of deallocation.
        unsafe {
            bindings::drm_gem_object_put((*self.ptr).gem_obj() as *const _ as *mut _);
        }
    }
}

impl<T: IntoGEMObject> Drop for UniqueObjectRef<T> {
    fn drop(&mut self) {
        // SAFETY: Having a UniqueObjectRef implies holding a GEM
        // reference. The free callback will take care of deallocation.
        unsafe {
            bindings::drm_gem_object_put((*self.ptr).gem_obj() as *const _ as *mut _);
        }
    }
}

impl<T: DriverObject> Deref for Object<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T: DriverObject> DerefMut for Object<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T: IntoGEMObject> Deref for ObjectRef<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        // SAFETY: The pointer is valid per the invariant
        unsafe { &*self.ptr }
    }
}

impl<T: IntoGEMObject> Deref for UniqueObjectRef<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        // SAFETY: The pointer is valid per the invariant
        unsafe { &*self.ptr }
    }
}

impl<T: IntoGEMObject> DerefMut for UniqueObjectRef<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: The pointer is valid per the invariant
        unsafe { &mut *self.ptr }
    }
}

impl<T: IntoGEMObject> UniqueObjectRef<T> {
    pub fn into_ref(self) -> ObjectRef<T> {
        let ptr = self.ptr as *const _;
        core::mem::forget(self);

        ObjectRef { ptr }
    }
}

pub(super) fn create_fops() -> bindings::file_operations {
    bindings::file_operations {
        owner: core::ptr::null_mut(),
        open: Some(bindings::drm_open),
        release: Some(bindings::drm_release),
        unlocked_ioctl: Some(bindings::drm_ioctl),
        #[cfg(CONFIG_COMPAT)]
        compat_ioctl: Some(bindings::drm_compat_ioctl),
        #[cfg(not(CONFIG_COMPAT))]
        compat_ioctl: None,
        poll: Some(bindings::drm_poll),
        read: Some(bindings::drm_read),
        llseek: Some(bindings::noop_llseek),
        mmap: Some(bindings::drm_gem_mmap),
        ..Default::default()
    }
}
