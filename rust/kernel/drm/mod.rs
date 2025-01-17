// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM subsystem

pub mod device;
pub mod drv;
pub mod file;
pub mod gem;
pub mod ioctl;
pub mod kms;
pub mod mm;

pub(crate) mod private {
    pub trait Sealed {}
}
