/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * S5L8740 FMSS FIL export — raw PPN page I/O for the Whimory stack.
 *
 * fmss-s5l8740.ko owns the controller. whimory / ftl-s5l8740.ko owns
 * FPart, VFL, SFTL, L2V, and the block device.
 *
 * fmss_ftl_read_sector() is a compatibility hook for apple-nimbus.ko.
 * After Whimory opens successfully it registers the real LBA reader.
 */
#ifndef FMSS_S5L8740_API_H
#define FMSS_S5L8740_API_H

#include <linux/device.h>
#include <linux/types.h>

#define FMSS_FTL_SECTOR_SIZE		4096U
#define FMSS_FTL_SECTORS_PER_LPN	4U
#define FMSS_FTL_DEFAULT_CAPACITY	3856968U

#define S5L8740_FMSS_MAX_CE		2U
#define S5L8740_FMSS_MAX_CAU		2U
#define S5L8740_FMSS_PAGE_SIZE		16384U
#define S5L8740_FMSS_META_SIZE		64U	/* 4 × 16-byte SFTL slots */

struct s5l8740_fmss_geom {
	u32 num_ce;
	u32 num_cau;
	u32 blocks_per_cau;
	u32 pages_per_block;
	u32 pages_per_block_slc;
	u32 page_size;
	u32 vfl_tail;
	u32 page_bits;
	u32 block_bits;
	u32 cau_bits;
	u32 caus_per_channel;
	u32 dev_id;	/* FIL selector 101 analogue */
	u32 geom_104;	/* FIL selector 104 analogue */
	u32 geom_105;	/* FIL selector 105 analogue */
	u32 geom_135;	/* FIL selector 135 analogue */
	bool from_param_page;
};

bool fmss_ftl_present(void);
struct device *fmss_ftl_device(void);
unsigned int fmss_ftl_lpn_count(void);
int fmss_ftl_build_map(unsigned int max_lpn);
int fmss_ftl_read_sector(u64 logical_sector, void *buf);

u32 s5l8740_fmss_fil_get_info(u32 selector);
int s5l8740_fmss_available(void);
int s5l8740_fmss_hw_init(void);
int s5l8740_fmss_query_geometry(struct s5l8740_fmss_geom *g);
int s5l8740_fmss_page_read(unsigned int ce, unsigned int cau,
			   unsigned int block, unsigned int page,
			   unsigned int slc, unsigned int chunks,
			   void *data, size_t data_len,
			   void *meta, size_t meta_len);
int s5l8740_fmss_nand_reset(void);
void s5l8740_fmss_register_ftl_read(int (*fn)(u64 lba, void *buf));

#endif /* FMSS_S5L8740_API_H */
