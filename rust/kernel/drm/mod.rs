// SPDX-License-Identifier: GPL-2.0 OR MIT

//! DRM subsystem

pub mod device;

pub(crate) mod private {
    pub trait Sealed {}
}
