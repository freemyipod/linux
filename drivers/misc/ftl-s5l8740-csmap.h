/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef FTL_S5L8740_CSMAP_H
#define FTL_S5L8740_CSMAP_H

#include <linux/device.h>
#include <linux/types.h>

#define N31_FTL_DISK_NAME	"s5l8740-ftl"	/* FAT alias (compat) */
#define N31_IPOD_DISK_NAME	"s5l8740-ipod"	/* user FAT volume */
#define N31_FW_DISK_NAME	"s5l8740-firmware"

struct n31_lba_map_entry {
	u8 ce;
	u8 cau;
	u16 block;
	u8 page;
	u8 slot;
	u8 type;
	u64 weave;
	u32 fmss_lba;
	bool present;
};

struct n31_ftl_cs;

int ftl_s5l8740_csmap_init(struct device *dev);
void ftl_s5l8740_csmap_exit(struct device *dev);

int n31_ftl_read_fmss_lba(struct n31_ftl_cs *ftl, u32 fmss_lba, void *dst);
int n31_ftl_read_disk_lba(struct n31_ftl_cs *ftl, u32 disk_lba, void *dst);

/*
 * After Whimory CXT→BTOC→L2V recover: bind csmap disks to L2V_Search
 * (no full hash import — avoids multi-million node RAM).
 */
int n31_ftl_cs_bind_whimory(void);
bool n31_ftl_cs_disk_registered(void);
bool n31_ftl_cs_whimory_backed(void);

/* Implemented in ftl-s5l8740-core.c (same module). */
int whimory_sftl_recover_cs(void);
int whimory_cxt_dump(unsigned int max_vbas);
/* The FPart system objects, and the SysCfg identity inside one. */
int whimory_fpart_objects_show(char *buf, size_t len);
int whimory_syscfg_show(char *buf, size_t len);
int whimory_touch_cal_show(char *buf, size_t len);
size_t whimory_syscfg_touch_cal(const u8 **out);
int whimory_cxt_candidate(u32 fat_base);
bool whimory_l2v_ready(void);
int whimory_read_fmss_lba(u32 lba, void *buf);
int whimory_range_walk(int (*fn)(u32 start, u32 len, u32 vba, u64 weave,
				 void *ctx),
		       void *ctx);
int whimory_l2v_search_phys(u32 lba, u8 *ce, u8 *cau, u16 *blk, u8 *page,
			    u8 *slot, u64 *weave);

/* Physical NAND string scan (ignores L2V). Returns hit count. */
int whimory_phys_string_scan(unsigned int max_blocks);

#endif /* FTL_S5L8740_CSMAP_H */
