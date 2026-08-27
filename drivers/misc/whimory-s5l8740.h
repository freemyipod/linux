/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * N31 Whimory PPN / SFTL / L2V — in-kernel structures.
 *
 * On-flash constants from OSOS 1.0.2 (sub_111B0C / sub_1122FC / sub_E8CA0 /
 * sub_428694 / sub_56AB3C / sub_56C328). Runtime L2V bytes are allocated
 * here; they are not present in the static OSOS dump.
 */
#ifndef WHIMORY_S5L8740_H
#define WHIMORY_S5L8740_H

#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "fmss-s5l8740-api.h"

#define WHIMORY_SIG_SIZE		0x600
#define WHIMORY_SIG_MAGIC		0x776d7278u	/* "xrmw" LE — payload, not raw page+0 */
#define WHIMORY_SIG_TYPE		0xC101u		/* FPart special type; op80/op20 */
#define WHIMORY_SIG_MAGIC_WRMX		0x786d7277u	/* "wrmx" LE — VFL CXT, not FPart sig */
#define WHIMORY_SIG_MAGIC_REV		0x78726d77u	/* "wmrx" byte-reversed hunt */

/* OSOS fpart_read_special_copy_4F1420 / locate_4EBBDC */
#define FPART_SPECIAL_TAG		0x30u
#define FPART_SPECIAL_CLASS		1u
#define FPART_SPECIAL_CLASS_MASK	0x3fu
#define FPART_SPECIAL_HDR		0x80u	/* chunk0 payload starts here */
#define FPART_SPECIAL_LEN_OFF		0x24u
#define FPART_SPECIAL_GEN_OFF		0x28u
#define FPART_SPECIAL_TABLE_BYTES	0x2d0u	/* ctx+112; 120 × 6 */
#define FPART_SPECIAL_MAX_ENTRIES	(FPART_SPECIAL_TABLE_BYTES / 6)
#define FPART_ASSIGN_MAX_PAIRS		8u
#define FPART_SPECIAL_TYPE_C104		0xC104u
#define FPART_SPECIAL_TYPE_C105		0xC105u

struct fpart_special_entry {
	u16 bank;
	u16 block;
	u16 type_word;
} __packed;

struct whimory_fpart {
	struct fpart_special_entry table[FPART_SPECIAL_MAX_ENTRIES];
	u16 count;
	bool scanned;
	unsigned int slot_logs;
};

#define WHIMORY_LBA_SIZE		4096U
#define WHIMORY_MIN_NODEPOOL_BYTES	0x80000
#define WHIMORY_L2V_NODE_SIZE		64
#define WHIMORY_L2V_ROOT_SPAN		0x8000
#define WHIMORY_L2V_ROOT_REC_SIZE	6
#define WHIMORY_L2V_INVALID_ROOT	0xffff
#define WHIMORY_META_SIZE		16
#define WHIMORY_VBAS_PER_PAGE		4
#define WHIMORY_PAGES_PER_SB		128
#define WHIMORY_DATA_PAGES_PER_SB	127
#define WHIMORY_BTOC_PAGE		127
/* s_g_vbas_per_sb includes the BTOC page: addr_to_vba(sb, vfl+76-1)
 * is the last VBA of page 127 (sub_56B77C). */
#define WHIMORY_VBAS_PER_SB		(WHIMORY_PAGES_PER_SB * \
					 WHIMORY_VBAS_PER_PAGE)
#define WHIMORY_DATA_VBAS_PER_SB	(WHIMORY_DATA_PAGES_PER_SB * \
					 WHIMORY_VBAS_PER_PAGE)

#define WHIMORY_META_TYPE_DATA		0x01
#define WHIMORY_META_TYPE_DATA2		0x02
#define WHIMORY_META_TYPE_BTOC		0x1c
#define WHIMORY_META_TYPE_SFTL_CXT	0x1f
#define WHIMORY_META_TYPE_VFL_CXT	0x20

#define WHIMORY_SB_EMPTY		0
#define WHIMORY_SB_CLOSED		1
#define WHIMORY_SB_OPEN			2
#define WHIMORY_SB_CXT			7	/* s_cxt_diff.c type 7 */

#define WHIMORY_SB_UNKNOWN		3

#define WHIMORY_CXT_MAX_SB		32
#define WHIMORY_CXT_TAG_BASE		1
#define WHIMORY_CXT_TAG_STATS		2
#define WHIMORY_CXT_TAG_L2V		4
#define WHIMORY_CXT_TAG_END		255
#define WHIMORY_CXT_TAG_CLEAN		0xff
#define WHIMORY_CXT_CONTIG_SPAN		0xfffffff0u
#define WHIMORY_FIL_META_BYTES		16	/* FIL GetInfo(105); sub_12ED9C */

/* BTOC / META tokens (sub_5688C4). Occupy VBA stream, not user L2V. */
#define WHIMORY_LBA_HOLE		0xFFFF0000u
#define WHIMORY_LBA_LIST		0xFFFF0001u
#define WHIMORY_LBA_DELETED		0xFFFFFF00u
#define WHIMORY_LBA_BLANK		0xFFFFFFFFu

/* VFL_GetParam selector: num_superblocks (WhimoryBoot.c sub_130158). */
#define WHIMORY_VFL_PARAM_NUM_SB	0x02000100u
#define WHIMORY_VFL_SPARE_FREE		0xC070u	/* sub_4EABA4 / 3D26D8 */
#define WHIMORY_VFL_CXT_HDR		0x200u	/* sub_4EB02C */
#define WHIMORY_VFL_SPARE_STRIDE	32u	/* sub_4EB098 */
#define WHIMORY_BTOC_SLOTS		6	/* s_btoc.c sub_56863C */
#define WHIMORY_BTOC_OPEN		2
#define WHIMORY_GC_ZONE_MIN		16	/* s_gc.c sub_56A328 */
#define WHIMORY_L2V_FINDFRAG_WIN	32	/* L2V_FindFrag.c */
#define WHIMORY_L2V_MIN_FREE		0x22u	/* sub_3F8958: free > 0x21 */
#define WHIMORY_L2V_UPDATE_REPACK	0xC8u	/* sub_E8EA8 */

struct whimory;

struct whimory_geometry {
	u32 num_ce;
	u32 num_cau;
	u32 blocks_per_cau;
	u32 user_blocks;
	u32 pages_per_block;
	u32 page_size;
	u32 vfl_tail;
	u32 dev_id;
	u32 geom_104;
	u32 geom_105;
	u32 geom_135;
};

struct whimory_signature {
	u8 raw[WHIMORY_SIG_SIZE];
	u32 version;
	u32 ftl_major;
	u32 ftl_minor;
	u32 vfl_major;
	u32 vfl_minor;
	u32 fpart_major;
	u32 fpart_minor;
	u32 sig_geom;
	u32 flags_or_open;	/* +0x20 VFL/open arg */
	u32 fpart_arg;		/* +0x2c */
	u32 extra_arg;		/* +0x30 */
};

struct whimory_meta {
	u8 type;
	u8 flags;
	u8 unk02[6];
	__le32 lba;
	u8 unk0c[4];
} __packed;

/* s_btoc.c sub_567E3C / s_cxt_diff.c sub_5694F0 — LE u32×4 */
struct whimory_bte {
	__le32 weave_seq_add;
	__le32 aux;
	__le32 lba;
	__le32 span;
} __packed;

struct whimory_leaf {
	u32 vba;
	u32 span;
};

struct whimory_l2v {
	u32 num_roots;
	u32 nodepool_bytes;
	u32 nodes_used;
	u8 bits_vba;
	u8 spanbits_vba;
	u8 bits_nodeidx;
	u8 spanbits_nodeidx;
	u32 invalid_vba;
	u32 sentinel_vba;
	u32 updates;		/* 0x8D100B0 */
	u32 gen;		/* 0x8D100B4 */
	u32 frag_count;		/* L2V_FindFrag */
	u32 frag_max;
	u32 free_head;		/* L2V_Mem.c 0x8D100DC; 0xffffffff empty */
	u32 free_count;		/* 0x8D100E0 */
	u8 *root;		/* 6 bytes × numRoots */
	u8 *nodes;
	struct whimory_leaf *leaf_scratch;
};

struct whimory_range {
	struct rb_node rb;
	u32 start;
	u32 len;
	u32 vba;
	u64 weave;
};

struct whimory_vfl {
	u32 *remap[S5L8740_FMSS_MAX_CAU];
	u16 *cxt_u16[S5L8740_FMSS_MAX_CAU];
	u32 ctx_ce[S5L8740_FMSS_MAX_CAU];
	u32 ctx_block[S5L8740_FMSS_MAX_CAU];
	u32 remap_count;
	u32 cxt_u16_len;
	u32 cxt_loc_count;
	u32 ctx_hits;
	u32 spare_applied;
	u32 bitmap_loaded;
	u32 bank_stride;	/* 0x8D0D0F0; N31 = 1 byte/VBN */
	u8 *bank_mask;		/* [blocks_per_cau] bank bitmask; sub_3D1438 */
	u16 cached_vbn;
	u8 cached_n;
	u8 cached_banks[S5L8740_FMSS_MAX_CAU];
};

struct whimory_sb {
	u16 ce;
	u16 cau;
	u16 block;
	u8 kind;
	u64 weave;
};

struct whimory_sftl {
	u32 vba_factor_a;
	u32 vba_factor_b;
	u32 nodepool_bytes;
	u32 vbas_per_page;
	u32 vbas_per_sb;
	u32 pages_per_sb;
	u32 num_sb;
	u32 user_blocks;
	u8 *btoc_page;
	u8 *data_page;
	u8 *meta_page;
	struct whimory_sb *sbs;
	u32 mapped_roots;
	u32 mapped_lbas;
	u32 btoc_sbs;
	u32 open_sbs;
	u32 empty_sbs;
	u32 cxt_sbs;
	u32 btoc_recs;
	u32 range_nodes;
	u32 cxt_bases;
	u32 token_hole;
	u32 token_list;
	u32 token_list_applied;
	u32 max_pages_per_btoc;
	u32 gc_zone_size;
	u8 *gc_data;
	u8 *gc_meta;
	u32 *btoc_lba[WHIMORY_BTOC_OPEN];
	bool cxt_loaded;
	bool packed_ok;
	u32 cxt_blocks_seen;
	u32 cxt_records_seen;
	u32 cxt_l2v_updates;
	u32 btoc_pages_read;
	u32 btoc_pages_valid;
	u32 btoc_entries_seen;
	u32 btoc_l2v_updates;
	u32 btoc_token_ffff0000;
	u32 btoc_token_ffffff00;
	u32 btoc_token_ffffffff;
	u32 btoc_holelist_ffff0001;
	u32 open_slots_seen;
	u32 open_slots_valid_meta;
	u32 open_l2v_updates;
	u32 l2v_update_calls;
	u32 l2v_unmap_calls;
	u32 l2v_repack_roots;
	u32 meta0_hits;
	u32 btoc_dumps_left;
	u32 unknown_sbs;
	u64 claim_weave;
};

struct whimory_cxt_base {
	u32 sb;
	u64 weave;
};

struct whimory_fpart_ops {
	u32 major;
	u32 (*minor)(struct whimory *w);
	int (*init)(struct whimory *w);
	int (*read_special)(struct whimory *w, u32 type, u8 *buf, size_t len);
	int (*read_signature)(struct whimory *w, u8 *buf, size_t len);
};

struct whimory_vfl_ops {
	u32 major;
	u32 (*minor)(struct whimory *w);
	int (*init)(struct whimory *w);
	int (*open)(struct whimory *w);
	u32 (*get_param)(struct whimory *w, u32 selector);
	int (*read_vba)(struct whimory *w, u32 vba, u32 count,
			void *data, struct whimory_meta *meta);
};

struct whimory_ftl_ops {
	u32 major;
	u32 (*minor)(struct whimory *w);
	int (*init)(struct whimory *w);
	int (*open)(struct whimory *w);
	int (*read_lba)(struct whimory *w, u32 lba, void *buf, bool allow_blank);
};

struct whimory {
	struct device *dev;
	struct whimory_geometry geom;
	struct whimory_signature sig;
	struct whimory_l2v l2v;
	struct whimory_vfl vfl;
	struct whimory_sftl sftl;
	struct rb_root ranges;
	struct whimory_fpart fpart_ctx;
	const struct whimory_fpart_ops *fpart;
	const struct whimory_vfl_ops *vfl_ops;
	const struct whimory_ftl_ops *ftl;
	u64 total_4k_sectors;
	u8 *bounce;
	struct mutex bounce_lock;
	struct mutex tree_lock;
	struct gendisk *disk;
	struct gendisk *ipod_disk;
	struct platform_device *pdev;
	u32 lba0_vba;
	u64 cxt_base_weave;
	struct whimory_cxt_base cxt[WHIMORY_CXT_MAX_SB];
	u32 n_cxt;
	u32 cxt_next_lba;
	bool cxt_lba_valid;
	bool fil_ok;
	bool sig_ok;
	bool vfl_ok;
	bool ftl_ok;
	bool l2v_ok;
	bool lba0_ok;
	bool oracle_used;
	char status[512];
};

static inline u32 whimory_sig32(const u8 *sig, unsigned int off)
{
	return get_unaligned_le32(sig + off);
}

#endif /* WHIMORY_S5L8740_H */
