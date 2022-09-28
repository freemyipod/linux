// SPDX-License-Identifier: GPL-2.0
#![allow(missing_docs)]

//! DRM GEM shmem helpers
//!
//! C header: [`include/linux/drm/drm_gem_shmem_helper.h`](../../../../include/linux/drm/drm_gem_shmem_helper.h)

use crate::drm::{device, drv, gem, private};
use crate::{
    error::{from_kernel_err_ptr, to_result},
    prelude::*,
};
use core::{
    marker::PhantomData,
    mem,
    mem::{ManuallyDrop, MaybeUninit},
    ops::{Deref, DerefMut},
    ptr::addr_of_mut,
    slice,
};

use gem::BaseObject;

#[repr(C)]
pub struct Object<T: DriverObject> {
    obj: bindings::drm_gem_shmem_object,
    dev: ManuallyDrop<device::Device<T::Driver>>,
    inner: T,
}

pub trait DriverObject: gem::BaseDriverObject<Object<Self>> {
    type Driver: drv::Driver;
}

impl<T: DriverObject> private::Sealed for Object<T> {}

impl<T: DriverObject> gem::IntoGEMObject for Object<T> {
    type Driver = T::Driver;

    fn gem_obj(&self) -> &bindings::drm_gem_object {
        &self.obj.base
    }
}

unsafe extern "C" fn gem_create_object<T: DriverObject>(
    raw_dev: *mut bindings::drm_device,
    size: usize,
) -> *mut bindings::drm_gem_object {
    // SAFETY: GEM ensures the device lives as long as its objects live,
    // so we can conjure up a reference from thin air and never drop it.
    let dev = ManuallyDrop::new(unsafe { device::Device::from_raw(raw_dev) });

    let inner = match T::new(&*dev, size) {
        Ok(v) => v,
        Err(e) => return e.to_ptr(),
    };

    let p = unsafe {
        bindings::krealloc(
            core::ptr::null(),
            Object::<T>::SIZE,
            bindings::GFP_KERNEL | bindings::__GFP_ZERO,
        ) as *mut Object<T>
    };

    if p.is_null() {
        return ENOMEM.to_ptr();
    }

    // SAFETY: p is valid as long as the alloc succeeded
    unsafe {
        addr_of_mut!((*p).dev).write(dev);
        addr_of_mut!((*p).inner).write(inner);
    }

    // SAFETY: drm_gem_shmem_object is safe to zero-init, and
    // the rest of Object has been initialized
    let new: &mut Object<T> = unsafe { &mut *(p as *mut _) };

    new.obj.base.funcs = &Object::<T>::VTABLE;
    &mut new.obj.base
}

unsafe extern "C" fn free_callback<T: DriverObject>(obj: *mut bindings::drm_gem_object) {
    // SAFETY: All of our objects are Object<T>.
    let p = crate::container_of!(obj, Object<T>, obj) as *mut Object<T>;

    // SAFETY: p is never used after this
    unsafe {
        core::ptr::drop_in_place(p);
    }

    // SAFETY: This pointer has to be valid, since p is valid
    unsafe {
        bindings::drm_gem_shmem_free(&mut (*p).obj);
    }
}

impl<T: DriverObject> drv::AllocImpl for Object<T> {
    const ALLOC_OPS: drv::AllocOps = drv::AllocOps {
        gem_create_object: Some(gem_create_object::<T>),
        prime_handle_to_fd: Some(bindings::drm_gem_prime_handle_to_fd),
        prime_fd_to_handle: Some(bindings::drm_gem_prime_fd_to_handle),
        gem_prime_import: None,
        gem_prime_import_sg_table: Some(bindings::drm_gem_shmem_prime_import_sg_table),
        gem_prime_mmap: Some(bindings::drm_gem_prime_mmap),
        dumb_create: Some(bindings::drm_gem_shmem_dumb_create),
        dumb_map_offset: None,
        dumb_destroy: None,
    };
}

// FIXME: This is terrible and I don't know how to avoid it
#[cfg(CONFIG_NUMA)]
macro_rules! vm_numa_fields {
    ( $($field:ident: $val:expr),* $(,)? ) => {
        bindings::vm_operations_struct {
            $( $field: $val ),*,
            set_policy: None,
            get_policy: None,
        }
    }
}

#[cfg(not(CONFIG_NUMA))]
macro_rules! vm_numa_fields {
    ( $($field:ident: $val:expr),* $(,)? ) => {
        bindings::vm_operations_struct {
            $( $field: $val ),*
        }
    }
}

const SHMEM_VM_OPS: bindings::vm_operations_struct = vm_numa_fields! {
    open: Some(bindings::drm_gem_shmem_vm_open),
    close: Some(bindings::drm_gem_shmem_vm_close),
    may_split: None,
    mremap: None,
    mprotect: None,
    fault: Some(bindings::drm_gem_shmem_fault),
    huge_fault: None,
    map_pages: None,
    pagesize: None,
    page_mkwrite: None,
    pfn_mkwrite: None,
    access: None,
    name: None,
    find_special_page: None,
};

pub struct VMap<T: DriverObject> {
    map: bindings::iosys_map,
    owner: gem::ObjectRef<Object<T>>,
}

impl<T: DriverObject> VMap<T> {
    pub fn as_ptr(&self) -> *const core::ffi::c_void {
        // SAFETY: The shmem helpers always return non-iomem maps
        unsafe { self.map.__bindgen_anon_1.vaddr }
    }

    pub fn as_mut_ptr(&mut self) -> *mut core::ffi::c_void {
        // SAFETY: The shmem helpers always return non-iomem maps
        unsafe { self.map.__bindgen_anon_1.vaddr }
    }

    pub fn as_slice(&self) -> &[u8] {
        // SAFETY: The vmap maps valid memory up to the owner size
        unsafe { slice::from_raw_parts(self.as_ptr() as *const u8, self.owner.size()) }
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        // SAFETY: The vmap maps valid memory up to the owner size
        unsafe { slice::from_raw_parts_mut(self.as_mut_ptr() as *mut u8, self.owner.size()) }
    }

    pub fn owner(&self) -> &gem::ObjectRef<Object<T>> {
        &self.owner
    }
}

impl<T: DriverObject> Drop for VMap<T> {
    fn drop(&mut self) {
        // SAFETY: This function is thread-safe
        unsafe {
            bindings::drm_gem_shmem_vunmap(self.owner.mut_shmem(), &mut self.map);
        }
    }
}

#[repr(transparent)]
pub struct SGEntry(bindings::scatterlist);

impl SGEntry {
    pub fn dma_address(&self) -> usize {
        (unsafe { bindings::sg_dma_address(&self.0) }) as usize
    }

    pub fn dma_len(&self) -> usize {
        (unsafe { bindings::sg_dma_len(&self.0) }) as usize
    }
}

pub struct SGTable<T: DriverObject> {
    sgt: *const bindings::sg_table,
    _owner: gem::ObjectRef<Object<T>>,
}

pub struct SGTableIter<'a> {
    sg: *mut bindings::scatterlist,
    left: usize,
    _p: PhantomData<&'a ()>,
}

impl<'a> Iterator for SGTableIter<'a> {
    type Item = &'a SGEntry;

    fn next(&mut self) -> Option<Self::Item> {
        if self.left == 0 {
            None
        } else {
            let sg = self.sg;
            self.sg = unsafe { bindings::sg_next(self.sg) };
            self.left -= 1;
            Some(unsafe { &(*(sg as *const SGEntry)) })
        }
    }
}

impl<T: DriverObject> SGTable<T> {
    pub fn iter(&'_ self) -> SGTableIter<'_> {
        SGTableIter {
            left: unsafe { (*self.sgt).nents } as usize,
            sg: unsafe { (*self.sgt).sgl },
            _p: PhantomData,
        }
    }
}

impl<'a, T: DriverObject> IntoIterator for &'a SGTable<T> {
    type Item = &'a SGEntry;
    type IntoIter = SGTableIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<T: DriverObject> Object<T> {
    const SIZE: usize = mem::size_of::<Self>();
    const VTABLE: bindings::drm_gem_object_funcs = bindings::drm_gem_object_funcs {
        free: Some(free_callback::<T>),
        open: None,
        close: None,
        print_info: Some(bindings::drm_gem_shmem_object_print_info),
        export: None,
        pin: Some(bindings::drm_gem_shmem_object_pin),
        unpin: Some(bindings::drm_gem_shmem_object_unpin),
        get_sg_table: Some(bindings::drm_gem_shmem_object_get_sg_table),
        vmap: Some(bindings::drm_gem_shmem_object_vmap),
        vunmap: Some(bindings::drm_gem_shmem_object_vunmap),
        mmap: Some(bindings::drm_gem_shmem_object_mmap),
        vm_ops: &SHMEM_VM_OPS,
    };

    // SAFETY: Must only be used with DRM functions that are thread-safe
    unsafe fn mut_shmem(&self) -> *mut bindings::drm_gem_shmem_object {
        &self.obj as *const _ as *mut _
    }

    pub fn new(dev: &device::Device<T::Driver>, size: usize) -> Result<gem::UniqueObjectRef<Self>> {
        // SAFETY: This function can be called as long as the ALLOC_OPS are set properly
        // for this driver, and the gem_create_object is called.
        let p = unsafe { bindings::drm_gem_shmem_create(dev.raw() as *mut _, size) };
        let p = crate::container_of!(p, Object<T>, obj) as *mut _;

        // SAFETY: The gem_create_object callback ensures this is a valid Object<T>,
        // so we can take a unique reference to it.
        let obj_ref = gem::UniqueObjectRef { ptr: p };

        Ok(obj_ref)
    }

    pub fn dev(&self) -> &device::Device<T::Driver> {
        &self.dev
    }

    pub fn sg_table(&self) -> Result<SGTable<T>> {
        let sgt = from_kernel_err_ptr(unsafe {
            bindings::drm_gem_shmem_get_pages_sgt(self.mut_shmem())
        })?;

        Ok(SGTable {
            sgt,
            _owner: self.reference(),
        })
    }

    pub fn vmap(&self) -> Result<VMap<T>> {
        let mut map: MaybeUninit<bindings::iosys_map> = MaybeUninit::uninit();

        // SAFETY: drm_gem_shmem_vmap is thread-safe
        to_result(unsafe { bindings::drm_gem_shmem_vmap(self.mut_shmem(), map.as_mut_ptr()) })?;

        // SAFETY: if drm_gem_shmem_vmap did not fail, map is initialized now
        let map = unsafe { map.assume_init() };

        Ok(VMap {
            map,
            owner: self.reference(),
        })
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
