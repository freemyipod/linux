/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * S5L8740 NAND controller (FMSS/FIL) — raw PPN page I/O for the Whimory / CS-map stack.
 *
 * Terminology (FTL map):
 * fmss_lba — logical LBA from 16-byte CS metadata
 * disk_lba — exported FAT volume LBA (Linux/VFAT)
 * fat_base_lba — fmss_lba of the FAT32 BPB (disk_lba 0)
 * physical record — ce/cau/block/page/slot/weave/type/lba
 * CS page — 4×4096 data + 4×16 meta (rec=4112)
 *
 * disk_lba 0 → fat_base_lba (proven candidate 49279).
 * target_fmss_lba = fat_base_lba + disk_lba
 *
 * nand-s5l8740.ko owns the controller. ftl-s5l8740.ko owns map / block.
 */
#ifndef NAND_S5L8740_H
#define NAND_S5L8740_H

#include <linux/device.h>
#include <linux/types.h>

#define NAND_FTL_SECTOR_SIZE		4096U
#define NAND_FTL_SECTORS_PER_LPN	4U
#define NAND_FTL_DEFAULT_CAPACITY	3856968U

#define S5L8740_NAND_MAX_CE		2U
#define S5L8740_NAND_MAX_CAU		2U
#define S5L8740_NAND_PAGE_SIZE		16384U
#define S5L8740_NAND_META_SIZE		64U	/* 4 × 16-byte SFTL slots */
#define S5L8740_NAND_SLOTS_PER_PAGE	4U
#define S5L8740_NAND_SLOT_DATA		4096U
#define S5L8740_NAND_SLOT_META		16U
#define S5L8740_NAND_REC_BYTES		4112U	/* 4096 data + 16 meta */

/* FTL map names (aliases of the above). */
#define N31_DATA_SLOTS			S5L8740_NAND_SLOTS_PER_PAGE
#define N31_DATA_SLOT_SIZE		S5L8740_NAND_SLOT_DATA
#define N31_META_SLOT_SIZE		S5L8740_NAND_SLOT_META
#define N31_CS_REC_SIZE			S5L8740_NAND_REC_BYTES

/* PPN DATA spare (CS META stream): glass-proven with rec=4112. */
#define S5L8740_NAND_META_TYPE_DATA	0x01u
#define S5L8740_NAND_META_TYPE_DATA2	0x02u

struct s5l8740_nand_geom {
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

/* Decoded 16-byte CS metadata slot (fmss_lba lives in.lba). */
struct s5l8740_meta_decoded {
	u8 type;
	u8 flags;
	u64 weave;	/* weaveSeq from meta bytes; width-limited */
	u32 lba;	/* fmss_lba — NOT disk_lba */
	bool valid;
	bool blank;
};

/* One CS physical page: four data + four meta records (rec=4112). */
struct s5l8740_cs_page {
	u8 data[N31_DATA_SLOTS][N31_DATA_SLOT_SIZE];
	u8 meta_raw[N31_DATA_SLOTS][N31_META_SLOT_SIZE];
	struct s5l8740_meta_decoded meta[N31_DATA_SLOTS];
};

/* Legacy alias used by older call sites / Whimory. */
struct s5l8740_nand_slot_meta {
	u8 type;
	u8 flags;
	u64 weave48;
	u32 lba;
	u8 aux[4];
	bool data_like;
};

bool nand_ftl_present(void);
struct device *nand_ftl_device(void);
unsigned int nand_ftl_lpn_count(void);
int nand_ftl_build_map(unsigned int max_lpn);
int nand_ftl_read_sector(u64 logical_sector, void *buf);

u32 s5l8740_nand_fil_get_info(u32 selector);
int s5l8740_nand_available(void);
int s5l8740_nand_meta_transport_ok(void);
int s5l8740_nand_hw_init(void);
int s5l8740_nand_query_geometry(struct s5l8740_nand_geom *g);

/*
 * CS physical read (FTL map ABI):
 * always slot=0, span=4, rec=4112, command-list CS DMA
 * fills 4 data + 4 meta slots; no lba_map ingest
 * Requires dma_dry=0 and dma_armed=1 (one-shot friendly).
 */
int s5l8740_nand_cs_phys_read_slot0(u8 ce, u8 cau, u16 block, u8 page,
				    struct s5l8740_cs_page *out);
int s5l8740_nand_cs_phys_read_span(u8 ce, u8 cau, u16 block, u8 page,
				   struct s5l8740_cs_page *out, unsigned int span);
int s5l8740_nand_cs_phys_read_slc(u8 ce, u8 cau, u16 block, u8 page, u8 slc,
				  struct s5l8740_cs_page *out, unsigned int span);
int s5l8740_nand_cs_phys_read(u8 ce, u8 cau, u16 block, u8 page,
			      struct s5l8740_cs_page *out);

/* Hold dma_armed across a multi-page CS scan; restores prior dry/one_shot. */
int s5l8740_nand_dma_session_begin(void);
void s5l8740_nand_dma_session_end(void);

void s5l8740_nand_meta_decode(const u8 *m16,
			      struct s5l8740_meta_decoded *out);

/* Legacy decode into slot_meta (aux + data_like). */
void s5l8740_nand_meta_decode_legacy(const u8 *m16,
				     struct s5l8740_nand_slot_meta *out);

static inline bool n31_meta_is_data_record(const struct s5l8740_meta_decoded *m)
{
	if (!m || !m->valid || m->blank)
		return false;
	return m->type == S5L8740_NAND_META_TYPE_DATA ||
	       m->type == S5L8740_NAND_META_TYPE_DATA2;
}

static inline bool n31_weave_newer(u64 a, u64 b)
{
	return a > b;
}

/*
 * Among four decoded meta records, pick newest weave claiming @fmss_lba
 * with type in {0x01,0x02}. Returns slot 0..3 or -ENOENT.
 */
int s5l8740_nand_meta_pick_lba(const struct s5l8740_cs_page *page,
			       u32 fmss_lba);

int s5l8740_nand_page_read(unsigned int ce, unsigned int cau,
			   unsigned int block, unsigned int page,
			   unsigned int slc, unsigned int chunks,
			   void *data, size_t data_len,
			   void *meta, size_t meta_len);
int s5l8740_nand_reset(void);
void s5l8740_nand_register_ftl_read(int (*fn)(u64 lba, void *buf));

#endif /* NAND_S5L8740_H */
