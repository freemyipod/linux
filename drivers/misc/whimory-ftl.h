/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Classic Whimory VFL/FTL structs (freemyipod Nano 2G FTL wiki).
 * N31 uses PPN/SFTL (wrmx / L2V) but the same spare type codes and
 * mount flow (VFL → ftlctrlblocks → FTL cxt 0x43 → map pages 0x44) apply
 * as a family reference for Stage C.
 *
 * Endianness: all multi-byte fields are little-endian on flash.
 */
#ifndef WHIMORY_FTL_H
#define WHIMORY_FTL_H

#include <linux/types.h>

/* Spare type codes (meta[9] / user.type) */
#define WMR_SPARE_DATA		0x40
#define WMR_SPARE_DATA_LAST	0x41
#define WMR_SPARE_FTL_CXT	0x43
#define WMR_SPARE_BLOCK_MAP	0x44
#define WMR_SPARE_ERASECTR	0x46
#define WMR_SPARE_UNCLEAN	0x47
#define WMR_SPARE_VFL_CXT	0x80
/* PPN/SFTL (N31) */
#define WMR_SPARE_VFLCXT_PPN	0x20
#define WMR_SPARE_SFTL_CXT	0x1F

#define WMR_DEVICEINFOSIGN	"DEVICEINFOSIGN"

/* User-data spare (types 0x40 / 0x41) — first 0xC bytes + ECC follow */
struct wmr_spare_user {
	__le32 lpn;
	__le32 usn;
	u8 field_8;
	u8 type;
	u8 eccmark;
	u8 field_B;
} __packed;

/* Meta spare (FTL/VFL context, block map, …) */
struct wmr_spare_meta {
	__le32 usn;
	__le16 idx;
	u8 field_6;
	u8 field_7;
	u8 field_8;
	u8 type;
	u8 eccmark;
	u8 field_B;
} __packed;

/*
 * VFL context — Nano2G size; N31 wrmx header is a different layout but
 * still carries FTL ctrl block hints in family ports.
 */
struct wmr_vfl_cxt {
	__le32 usn;
	__le16 ftlctrlblocks[3];
	u8 field_A[2];
	__le32 updatecount;
	__le16 activecxtblock;
	__le16 nextcxtpage;
	u8 field_14[4];
	__le16 field_18;
	__le16 spareused;
	__le16 firstspare;
	__le16 sparecount;
	__le16 remaptable[0x334];
	u8 bbt[0x11A];
	__le16 vflcxtblocks[4];
	__le16 scheduledstart;
	u8 field_7AC[0x4C];
	__le32 checksum1;
	__le32 checksum2;
} __packed;

/* FTL context (0x28C used by freemyipod on read) */
struct wmr_ftl_cxt {
	__le32 usn;
	__le32 nextblockusn;
	__le16 freecount;
	__le16 nextfreeidx;
	__le16 swapcounter;
	__le16 blockpool[0x14];
	__le16 field_36;
	__le32 ftl_map_pages[8];
	u8 field_58[0x28];
	__le32 ftl_erasectr_pages[8];
	u8 field_A0[0x70];
	__le32 ftl_map_ptr;
	__le32 ftl_erasectr_ptr;
	__le32 ftl_log_ptr;
	__le32 erasedirty;
	__le16 field_120;
	__le16 ftlctrlblocks[3];
	__le32 ftlctrlpage;
	__le32 clean_flag;
	u8 field_130[0x15C];
} __packed;

/* Classic lPage → vPage using block map (u16 vBlock per lBlock). */
static inline u32 wmr_lpage_to_vpage(u32 lpage, u32 pages_per_block,
				     const u16 *map, u32 map_entries)
{
	u32 lblock = lpage / pages_per_block;
	u32 page = lpage % pages_per_block;
	u16 vblock;

	if (lblock >= map_entries || !map)
		return ~0u;
	vblock = map[lblock];
	if (!vblock || vblock == 0xffff)
		return ~0u;
	return (u32)vblock * pages_per_block + page;
}


#endif /* WHIMORY_FTL_H */
