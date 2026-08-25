/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * S5L8740 FMSS → FTL block layer export (Whimory read path).
 * Consumed by ftl-s5l8740.ko; implemented by fmss-s5l8740.ko.
 *
 * Logical disk: 4096-byte sectors, LPN = sector >> 2 (4 sectors / 16 KiB page).
 * Build the dense L2V via sysfs l2v_build (or fmss_ftl_build_map); sector 0 is
 * served from a carved *UOKJIHC BPB when present. Unmapped sectors return -ENOENT.
 */
#ifndef FMSS_S5L8740_API_H
#define FMSS_S5L8740_API_H

#include <linux/device.h>
#include <linux/types.h>

/* Apple RetailOS FAT32 on N31 (4096-byte logical sectors). Override via module param. */
#define FMSS_FTL_SECTOR_SIZE	4096U
#define FMSS_FTL_SECTORS_PER_LPN 4U
#define FMSS_FTL_DEFAULT_CAPACITY 3856968U

bool fmss_ftl_present(void);
struct device *fmss_ftl_device(void);
unsigned int fmss_ftl_lpn_count(void);
int fmss_ftl_build_map(unsigned int max_lpn);
int fmss_ftl_read_sector(u64 logical_sector, void *buf);

#endif /* FMSS_S5L8740_API_H */
