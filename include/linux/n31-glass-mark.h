/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * N31 LCD-only bring-up witness — no serial, no U-Boot vidconsole.
 * pr_alert() + console=tty0 → readable on fbcon once DRM is up.
 */
#ifndef _LINUX_N31_GLASS_MARK_H
#define _LINUX_N31_GLASS_MARK_H

#include <linux/printk.h>

#define n31_glass_mark(tag) \
	pr_alert("N31>> %s\n", (tag))

#endif /* _LINUX_N31_GLASS_MARK_H */
