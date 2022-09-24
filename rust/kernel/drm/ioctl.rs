// SPDX-License-Identifier: GPL-2.0 OR MIT
#![allow(missing_docs)]
#![allow(non_snake_case)]

//! DRM IOCTL definition
//!
//! C header: [`include/linux/drm/drm_ioctl.h`](../../../../include/linux/drm/drm_ioctl.h)

use crate::ioctl;

const BASE: u32 = bindings::DRM_IOCTL_BASE as u32;

pub const fn IO(nr: u32) -> u32 {
    ioctl::_IO(BASE, nr)
}
pub const fn IOR<T>(nr: u32) -> u32 {
    ioctl::_IOR::<T>(BASE, nr)
}
pub const fn IOW<T>(nr: u32) -> u32 {
    ioctl::_IOW::<T>(BASE, nr)
}
pub const fn IOWR<T>(nr: u32) -> u32 {
    ioctl::_IOWR::<T>(BASE, nr)
}

pub type DRMIOCTLDescriptor = bindings::drm_ioctl_desc;

pub const AUTH: u32 = bindings::drm_ioctl_flags_DRM_AUTH;
pub const MASTER: u32 = bindings::drm_ioctl_flags_DRM_MASTER;
pub const ROOT_ONLY: u32 = bindings::drm_ioctl_flags_DRM_ROOT_ONLY;
pub const UNLOCKED: u32 = bindings::drm_ioctl_flags_DRM_UNLOCKED;
pub const RENDER_ALLOW: u32 = bindings::drm_ioctl_flags_DRM_RENDER_ALLOW;

#[macro_export]
macro_rules! declare_drm_ioctls {
    ( $(($cmd:ident, $struct:ident, $flags:expr, $func:expr)),* $(,)? ) => {
        const IOCTLS: &'static [$crate::drm::ioctl::DRMIOCTLDescriptor] = {
            const _:() = {
                use $crate::bindings::*;
                let i: u32 = $crate::bindings::DRM_COMMAND_BASE;
                // Assert that all the IOCTLs are in the right order and there are no gaps,
                // and that the sizeof of the specified type is correct.
                $(
                    let cmd: u32 = $crate::macros::concat_idents!(DRM_IOCTL_, $cmd);
                    ::core::assert!(i == $crate::ioctl::_IOC_NR(cmd));
                    ::core::assert!(core::mem::size_of::<$crate::bindings::$struct>() == $crate::ioctl::_IOC_SIZE(cmd));
                    let i: u32 = i + 1;
                )*
            };

            let ioctls = &[$(
                $crate::bindings::drm_ioctl_desc {
                    // TODO: nicer solution to this?
                    cmd: {
                        use $crate::bindings::*;
                        $crate::macros::concat_idents!(DRM_IOCTL_, $cmd) as u32
                    },
                    func: {
                        #[allow(non_snake_case)]
                        unsafe extern "C" fn $cmd(
                                raw_dev: *mut $crate::bindings::drm_device,
                                raw_data: *mut ::core::ffi::c_void,
                                raw_file_priv: *mut $crate::bindings::drm_file,
                        ) -> core::ffi::c_int {
                            // SAFETY: We never drop this, and the DRM core ensures the device lives while
                            // callbacks are being called
                            let dev = ::core::mem::ManuallyDrop::new(unsafe {
                                $crate::drm::device::Device::from_raw(raw_dev)
                            });
                            // SAFETY: This is just the ioctl argument, which hopefully has the right type
                            // (we've done our best checking the size).
                            let data = unsafe { &mut *(raw_data as *mut $crate::bindings::$struct) };
                            // SAFETY: This is just the DRM file structure
                            let file = unsafe { $crate::drm::file::File::from_raw(raw_file_priv) };

                            match $func(&*dev, data, &file) {
                                Err(e) => e.to_kernel_errno(),
                                Ok(i) => i.try_into().unwrap_or(ERANGE.to_kernel_errno()),
                            }
                        }
                        Some($cmd)
                    },
                    flags: $flags,
                    name: $crate::c_str!(::core::stringify!($cmd)).as_char_ptr(),
                }
            ),*];
            ioctls
        };
    };
}
