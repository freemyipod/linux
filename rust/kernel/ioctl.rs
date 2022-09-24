// SPDX-License-Identifier: GPL-2.0
#![allow(missing_docs)]
#![allow(non_snake_case)]

//! ioctl() number definitions
//!
//! C header: [`include/asm-generic/ioctl.h`](../../../../include/asm-generic/ioctl.h)

const fn _IOC(dir: u32, ty: u32, nr: u32, size: usize) -> u32 {
    core::assert!(dir <= bindings::_IOC_DIRMASK);
    core::assert!(ty <= bindings::_IOC_TYPEMASK);
    core::assert!(nr <= bindings::_IOC_NRMASK);
    core::assert!(size <= (bindings::_IOC_SIZEMASK as usize));

    (dir << bindings::_IOC_DIRSHIFT)
        | (ty << bindings::_IOC_TYPESHIFT)
        | (nr << bindings::_IOC_NRSHIFT)
        | ((size as u32) << bindings::_IOC_SIZESHIFT)
}

pub const fn _IO(ty: u32, nr: u32) -> u32 {
    _IOC(bindings::_IOC_NONE, ty, nr, 0)
}
pub const fn _IOR<T>(ty: u32, nr: u32) -> u32 {
    _IOC(bindings::_IOC_READ, ty, nr, core::mem::size_of::<T>())
}
pub const fn _IOW<T>(ty: u32, nr: u32) -> u32 {
    _IOC(bindings::_IOC_WRITE, ty, nr, core::mem::size_of::<T>())
}
pub const fn _IOWR<T>(ty: u32, nr: u32) -> u32 {
    _IOC(
        bindings::_IOC_READ | bindings::_IOC_WRITE,
        ty,
        nr,
        core::mem::size_of::<T>(),
    )
}

pub const fn _IOC_DIR(nr: u32) -> u32 {
    (nr >> bindings::_IOC_DIRSHIFT) & bindings::_IOC_DIRMASK
}
pub const fn _IOC_TYPE(nr: u32) -> u32 {
    (nr >> bindings::_IOC_TYPESHIFT) & bindings::_IOC_TYPEMASK
}
pub const fn _IOC_NR(nr: u32) -> u32 {
    (nr >> bindings::_IOC_NRSHIFT) & bindings::_IOC_NRMASK
}
pub const fn _IOC_SIZE(nr: u32) -> usize {
    ((nr >> bindings::_IOC_SIZESHIFT) & bindings::_IOC_SIZEMASK) as usize
}
