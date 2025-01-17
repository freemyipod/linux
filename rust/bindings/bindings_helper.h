/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Header that contains the code (mostly headers) for which Rust bindings
 * will be automatically generated by `bindgen`.
 *
 * Sorted alphabetically.
 */

#include <kunit/test.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_mm.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/amba/bus.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/errname.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fs_parser.h>
#include <linux/gpio/driver.h>
#include <linux/hw_random.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/netfilter_arp.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/random.h>
#include <linux/security.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <uapi/asm-generic/ioctl.h>
#include <uapi/linux/android/binder.h>

/* `bindgen` gets confused at certain things. */
const gfp_t BINDINGS_GFP_KERNEL = GFP_KERNEL;
const gfp_t BINDINGS___GFP_ZERO = __GFP_ZERO;
const __poll_t BINDINGS_EPOLLIN = EPOLLIN;
const __poll_t BINDINGS_EPOLLOUT = EPOLLOUT;
const __poll_t BINDINGS_EPOLLERR = EPOLLERR;
const __poll_t BINDINGS_EPOLLHUP = EPOLLHUP;

const loff_t BINDINGS_MAX_LFS_FILESIZE = MAX_LFS_FILESIZE;
