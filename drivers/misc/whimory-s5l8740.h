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

#include "nand-s5l8740.h"

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
/* meta[0] of every FPart system object; the type word at meta+2 sorts them. */
#define FPART_META_TYPE_SPECIAL	0x30u
/* Enumerated on the glass: c101 signature, c105 config/serial, c104 context. */
#define FPART_TYPE_SIGNATURE		0xc101u
#define FPART_TYPE_VFL_CXT		0xc104u
#define FPART_TYPE_CONFIG		0xc105u
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

/* Pages held in the sequential read-ahead window (16 KiB each). */
#define WHIMORY_RC_SLOTS		8

/* Legacy per-block classify prefetch window, in 4 KiB slots. */
#define WHIMORY_PF_SLOTS		16
#define WHIMORY_NUM_CE_MAX		2
#define WHIMORY_VBAS_PER_PAGE		4
#define WHIMORY_PAGES_PER_SB		128
#define WHIMORY_DATA_PAGES_PER_SB	127
#define WHIMORY_BTOC_PAGE		127
/*
 * VBAs per superblock includes the block table of contents page: the last
 * VBA a superblock can address falls on page 127.
 */
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
#define WHIMORY_CXT_TAG_SB		3
#define WHIMORY_CXT_TAG_L2V		4
#define WHIMORY_CXT_TAG_USERSEQ		5
#define WHIMORY_CXT_TAG_READS		6
/* 0xff is both "nothing written here" and the record-stream terminator. */
#define WHIMORY_CXT_TAG_END		255
#define WHIMORY_CXT_TAG_CLEAN		0xff
#define WHIMORY_CXT_CONTIG_SPAN		0xfffffff0u
/* TREE hole sentinel: consumes logical space, maps nothing. */
#define WHIMORY_CXT_VBA_HOLE		0x7fffffu
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
	/*
	 * Which producer put this range here: 1 BTOC, 2 open rebuild,
	 * 3 CXT seed, 4 list token. Diagnostic only, and the reason it
	 * exists is that a wrong mapping looks identical whoever wrote
	 * it -- the CXT record for the failing LBAs parses correctly,
	 * so the question is who overwrote it afterwards.
	 */
	u8 src;
};

struct whimory_vfl {
	u32 *remap[S5L8740_NAND_MAX_CAU];
	u16 *cxt_u16[S5L8740_NAND_MAX_CAU];
	u32 ctx_ce[S5L8740_NAND_MAX_CAU];
	u32 ctx_block[S5L8740_NAND_MAX_CAU];
	u32 remap_count;
	u32 cxt_u16_len;
	u32 cxt_loc_count;
	u32 ctx_hits;
	u32 spare_applied;
	u32 bitmap_loaded;
	/* Block status table from the 0xc104 object: one bit per block. */
	u8 *blk_status;
	u32 blk_status_bad;
	u32 blk_status_bank;
	u32 blk_status_block;
	u32 blk_status_mismatch;
	bool syscfg_ok;
	u32 bank_stride;	/* 0x8D0D0F0; N31 = 1 byte/VBN */
	u8 *bank_mask;		/* [blocks_per_cau] bank bitmask; sub_3D1438 */
	u16 cached_vbn;
	u8 cached_n;
	u8 cached_banks[S5L8740_NAND_MAX_CAU];
};

/*
 * Apple SysCfg, the 0xc105 FPart object.
 *
 * A header naming the entry count, then that many records. Each record
 * is a four-character tag followed by its value, and the tag is stored
 * byte-reversed -- "SrNm" sits in the page as 6d 4e 72 53. Record sizes
 * are not uniform, so the parser finds the tags and takes each value as
 * the bytes up to the next one rather than assuming a stride.
 *
 * Read off the glass: 11 entries, SrNm FwId HwId HwVr SwVr MLB# CNTB
 * MtCl Mod# Regn BMac.
 */
/*
 * The touch calibration block, as an offset into the SysCfg payload.
 *
 * It sits at 0x1dc0 from the start of the page, and the payload starts at
 * FPART_SPECIAL_HDR, so the payload-relative offset is 0x80 less. Using
 * the page offset here started the block 0x80 bytes late and cut its
 * header off.
 */
#define N31_TOUCH_CAL_OFF		(0x1dc0u - FPART_SPECIAL_HDR)

/* Bytes stock copies from the touch calibration descriptor (sub_564). */
#define N31_TOUCH_CAL_LEN		0x560u
/* apple-grape's calibration window inside that blob. */
#define N31_TOUCH_CAL_CAL_OFF	350u
#define N31_TOUCH_CAL_CAL_LEN	0x200u

/* Candidate tags recorded from one SysCfg section. */
#define N31_SYSCFG_MAX_CAND	64u

struct whimory_syscfg {
	bool valid;
	u32 entries;
	char serial[24];	/* SrNm */
	char model[24];	/* Mod# */
	char mlb[32];		/* MLB#, the logic board */
	char sw_ver[24];	/* SwVr */
	char cnt_b[24];	/* CNTB */
	char mt_cl[24];	/* MtCl */
	u8 mac[6];		/* BMac */
	bool mac_ok;
	u32 region;		/* Regn */
	u32 hw_ver;		/* HwVr */
	u32 fw_id;		/* FwId */
	/* touch calibration touchscreen calibration; raw, for apple-grape. */
	/*
	 * The touch calibration blob, verbatim and whole.
	 *
	 * Stock takes the descriptor at 0x2202FE18 (magic 0x53797349, data
	 * pointer at +4) and copies 0x560 bytes from it; apple-grape then
	 * reads its calibration window at +350 for 0x200. So the consumer
	 * needs 862 bytes at minimum and the container is 1376, which is
	 * what U-Boot republishes via /chosen apple,n31-touch_cal-addr and
	 * apple,n31-touch_cal-size.
	 *
	 * A 1024-byte buffer could not hold it.
	 */
	u8 touch_cal[N31_TOUCH_CAL_LEN];
	u32 touch_cal_len;
	int touch_cal_magic_off;	/* offset of the container magic, -1 if absent */

	/*
	 * The whole SysCfg section, verbatim, plus every 4-byte-aligned
	 * printable group found in it.
	 *
	 * The parser only decodes the eleven tags it knows. Everything else
	 * in the section is invisible, and this device carries records
	 * nobody has identified yet -- so keep the raw bytes and a list of
	 * candidate tags, and let them be read out rather than guessed at.
	 *
	 * Tags are stored byte-reversed on disk ("SrNm" is 6d 4e 72 53), so
	 * cand[].tag holds the un-reversed text.
	 */
	u8 *raw;
	u32 raw_len;
	struct {
		char tag[5];
		u32 off;
		u32 len;
		bool known;
	} cand[N31_SYSCFG_MAX_CAND];
	u32 n_cand;
};

struct whimory_sb {
	u16 ce;
	u16 cau;
	u16 block;
	u8 kind;
	u64 weave;	/* page 0 -- the OLDEST content in the superblock */
	u64 weave_max;	/* newest content we have actually seen a weave for */
	u8 weave_max_p127;	/* weave_max came from page 127, not page 0 */
};

/* Compact CXT superblock identity; see whimory_cxt_index_build(). */
struct whimory_cxt_sb_id {
	u16 ce;
	u16 cau;
	u16 block;
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
	/*
	 * Last NAND page read, cached across calls.
	 *
	 * A NAND page is 16 KiB and an LBA is 4 KiB, so four consecutive
	 * LBAs live in one page. The block layer hands this driver one 4 KiB
	 * LBA at a time, and n31_vfl_read_vba() reads a whole page to copy
	 * 4 KiB out of it -- so a sequential read pulls the SAME page off the
	 * media four times. Measured before this: 520 KB/s and 7.7 ms per
	 * 4 KiB, on a device where a directory scan is hundreds of reads and
	 * playback has 341 ms of buffer.
	 *
	 * The media is read-only here (whimory_submit_bio_range rejects
	 * writes outright), so nothing can invalidate this behind our back
	 * and the cache needs no coherence beyond being dropped on teardown.
	 */
	u8 *page_cache;
	u8 page_cache_spare[S5L8740_NAND_META_SIZE];
	u32 pc_ce, pc_cau, pc_pblock, pc_page;
	bool page_cache_valid;
	struct s5l8740_cs_page *cs_page; /* CS span4 scratch for recover/read */

	/*
	 * Batched page-0 prefetch window for the classify scan. pf_data is
	 * 16 * 4 KiB and pf_meta 16 * 16 B, allocated at recover and freed
	 * with it -- the scan is the only thing that reads whole ranges of
	 * page 0 in order, so nothing outside it needs the window.
	 */
	u8 *pf_data;
	u8 *pf_meta;
	unsigned int pf_ce;
	unsigned int pf_cau;
	unsigned int pf_first;
	unsigned int pf_count;
	unsigned int pf_kicks;
	unsigned int pf_hits;
	bool pf_valid;
	bool pf_failed;
	bool pf_checked;

	/*
	 * Read-ahead window: S5L8740_NAND_PAGE_BATCH_MAX consecutive physical
	 * pages of one block, filled by a single batched kick.
	 *
	 * This supersedes the one-page cross-call cache for sequential reads.
	 * Four LBAs share a 16 KiB page, so the old cache turned four block
	 * layer calls into one NAND read; the window turns thirty-two into
	 * one, which is a whole 128 KiB readahead request per kick.
	 */
	u8 *rc_data;
	u8 *rc_meta;
	u8 *rc_stage;
	u8 *rc_stage_meta;
	struct {
		unsigned int ce;
		unsigned int cau;
		unsigned int pblock;
		unsigned int page;
		bool valid;
	} rc_key[WHIMORY_RC_SLOTS];
	unsigned int rc_count;
	unsigned int rc_fills;
	unsigned int rc_hits;
	unsigned int rc_misses;
	unsigned int rc_fails;
	unsigned int scan_kicks;

	/*
	 * Which (ce, cau) banks each virtual block actually spans.
	 *
	 * A superblock is not always all four banks. The VFL keeps a bitmap
	 * of the banks that carry a given VBN -- s_vfl.c sub_3D1438 counts
	 * the set bits of one stride-sized row per VBN -- and every address
	 * the FTL builds is dense over *those* banks:
	 *
	 *	vba = (vbn * max_banks * pages_per_sb
	 *	       + page * nbanks + bank_ofs) * vbas_per_page + slot
	 *
	 * so a superblock holds nbanks * pages_per_sb * vbas_per_page
	 * addresses (sub_4EFE0C), not the full four banks' worth. Decoding a
	 * three-bank superblock as four puts the read on the wrong page of
	 * the wrong bank, which is the "sftl lba mismatch" against a page
	 * written long before the checkpoint.
	 *
	 * One byte per virtual block, bit b set for bank b = ce * num_cau +
	 * cau. Built during classify from which banks carry a real record at
	 * page 0; an all-zero entry means "not classified", and the decode
	 * falls back to all banks so an unscanned region behaves as before.
	 */
	u8 *sb_bank_mask;
	u32 sb_bank_blocks;	/* entries allocated */
	u32 sb_bank_known;	/* vblocks with at least one bank */
	u32 sb_bank_partial;	/* vblocks with fewer banks than the maximum */
	u32 sb_bank_hist[S5L8740_NAND_MAX_CE * S5L8740_NAND_MAX_CAU + 1];
	u32 sb_bank_overflow;	/* CXT offsets past the derived superblock end */

	struct whimory_sb *sbs;
	u32 mapped_roots;
	u32 mapped_lbas;
	u32 btoc_sbs;
	u32 open_sbs;
	u32 empty_sbs;
	u32 fast_empty_hits;	/* settled by the one-record probe */
	u32 fast_slot0_hits;	/* non-empty blocks settled from slot 0 */
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
	u32 *btoc_map;		/* per-VBA LBA decoded from one BTOC page */
	bool cxt_loaded;
	bool packed_ok;
	u32 cxt_blocks_seen;
	u32 cxt_records_seen;
	u32 cxt_l2v_updates;
	u32 cxt_hole_entries;
	u32 cxt_xlate_fail;
	/* CXT extents checked against the page's own metadata before use. */
	u32 cxt_meta_confirmed;
	u32 cxt_meta_mismatch;
	u32 cxt_confirm_pages;
	u32 cxt_confirm_unreadable;
	/* Extents a stale checkpoint lost, rebuilt from page metadata. */
	u32 cxt_repair_pages;
	u32 cxt_repair_slots;
	u32 diff_replayed_sbs;
	u32 diff_skipped_sbs;
	u32 diff_open_kept;	/* open SBs the page-0 weave would have skipped */
	u32 open_truncated;	/* open SBs dropped by max_open_sbs */
	u32 weave_newer;	/* SBs at or after the checkpoint */
	u32 weave_older;	/* SBs the checkpoint provably covers */
	u32 weave_none;		/* SBs with no usable weave at all */
	u32 open_pages_read;	/* pages read rebuilding open SBs */
	u32 cxt_hole_lbas;	/* logical space the holes cover */
	u32 cxt_ext_nospc;	/* extents dropped, table full */
	u32 cxt_records_lost;	/* records after an aborted parse */
	u32 cxt_sb_empty;	/* CXT SBs that yielded nothing */
	u32 btoc_blank;		/* BTOC pages that were erased */
	u32 btoc_be_bte;	/* claimed by the big-endian BTE parser */
	u32 btoc_be_lpn;	/* claimed by the big-endian LPN parser */
	u32 btoc_le_bte;	/* claimed by the little-endian BTE parser */
	u32 btoc_le_bte_hdr8;	/* same, after an 8-byte page header */
	u32 btoc_unclaimed;	/* no parser recognised the page */
	u32 btoc_fb_sbs;	/* closed SBs rebuilt from per-page meta */
	u32 btoc_fb_pages;	/* pages that cost */
	u32 btoc_fb_hits;	/* mappings it recovered */
	u32 unk_fb_sbs;		/* unclassified SBs rebuilt from meta */
	u32 unk_fb_pages;
	u32 unk_fb_hits;
	u32 bad_map_logged;	/* bounded explanations for bad reads */
	u32 cxt_dumped;		/* pairs printed by the record dump */
	u32 cxt_hdr_skipped;	/* records whose header said nothing follows */
	u32 cxt_hdr_bad;	/* records whose header was not a CONTIG marker */
	u32 cxt_resyncs;	/* TREE records that re-anchored the LBA cursor */
	u32 btoc_pages_read;
	u32 btoc_pages_valid;
	u32 btoc_entries_seen;
	u32 btoc_l2v_updates;
	u32 btoc_token_ffff0000;
	u32 btoc_token_ffffff00;
	u32 btoc_token_ffffffff;
	u32 btoc_holelist_ffff0001;
	/* Classified BTOC/open entry counters (recovery observability). */
	u32 btoc_unmap_entries;	/* LIST tokens applied or attempted */
	u32 btoc_hole_entries;	/* HOLE / erased physical spans */
	u32 btoc_unknown_entries;
	u32 open_unmap_entries;
	u32 open_skipped_zero;
	u32 open_overrides_closed;
	u32 open_rejected_stale;
	u32 open_unknown_order;
	u32 stale_mapping_rejected;
	u32 btoc_meta_mismatch;
	u32 btoc_meta_confirmed;
	u32 btoc_skipped_zero;
	u32 btoc_confirm_pages;
	u32 btoc_confirm_capped;
	u32 btoc_confirm_budget_stop;
	u32 string_hit_itunesdb;
	u32 string_hit_f00;
	u32 string_hit_apps;
	u32 string_hit_mp3;
	u32 string_hit_m4a;
	u32 string_hit_ipod_control;
	u32 string_hit_music;
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
	/* 0=none, 1=BTOC/closed, 2=open, 3=CXT, 4=LIST-unmap */
	u8 claim_source;
	unsigned long confirm_start_jiffies;
	/* Recover-time replay accounting (widen staging / OOM guard). */
	u32 range_budget_stop;
	u32 btoc_verified;
	u32 map_gen;		/* bumped on every map mutation */
	u32 search_cache_hits;
	u32 search_cache_misses;
	struct whimory_cxt_sb_id cxt_idx[WHIMORY_CXT_MAX_SB];
	unsigned int n_cxt_idx;
};

struct whimory_cxt_base {
	u32 sb;
	u64 weave;
};

/*
 * One contiguous (lba, span) -> vba mapping decoded from a CXT TREE record.
 * Phase 3 keeps these in a candidate map, separate from the live interval
 * map, so the CXT decode can be validated without disturbing a working disk.
 */
/*
 * weave is the checkpoint generation this extent came from, and it is not
 * optional bookkeeping.
 *
 * The candidate map is the union of every CXT superblock, and several
 * generations describe the same logical ranges. Without a per-extent weave
 * the seed had nothing to arbitrate with: it stamped every extent with one
 * value, so the newest-wins rule in whimory_range_update() could never fire
 * between two seed extents and whichever entry the sort placed last won.
 * sort() is heapsort and unstable, so that was decided by the heap, and the
 * map came out different on every boot from identical flash.
 */
struct whimory_cxt_extent {
	u32 lba;
	u32 span;
	u32 vba;
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
	struct whimory_syscfg	syscfg;
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
	u32 bad_vba;		/* what the map answered for a failing read */
	u32 bad_span;
	u64 cxt_base_weave;
	/*
	 * The weave the checkpoint's own pages ran out to.
	 *
	 * s_cxt.c:81 sets sftl.write.weaveSeq to
	 *
	 *	cxt->load.baseWeaveSeq + cxt->save.num_sb * s_g_vbas_per_sb + 1
	 *
	 * before the diff runs, so every write after the checkpoint carries a
	 * weave at or above this. Weaves between the base and here belong to
	 * the checkpoint superblocks themselves, not to post-checkpoint user
	 * data, and the skip rule has to draw its line here rather than at
	 * the base.
	 */
	u64 cxt_top_weave;
	u32 cxt_save_num_sb;	/* first word of the BASE record payload */
	struct whimory_cxt_extent *cxt_ext;	/* candidate map (Phase 3) */
	u32 n_cxt_ext;
	u32 max_cxt_ext;
	/* Longest span in cxt_ext; bounds the backward walk in cxt_lookup. */
	u32 cxt_ext_max_span;
	u64 cxt_ext_weave;			/* base weave it came from */
	u32 cxt_ext_sb;
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
	bool l2v_defer_pack;	/* pack once after replay, not per update */
	unsigned long progress_jiffies;	/* rate-limits recover progress */
	/*
	 * Machine-readable mirror of the recover progress lines. The dev_info
	 * output suits a human reading a console; a progress bar needs the
	 * numbers without parsing prose, so the same call sites publish here.
	 */
	const char *prog_phase;
	unsigned int prog_cur;
	unsigned int prog_total;
	/* L2V_Search sequential hint; see whimory_l2v_search(). */
	u32 search_start;
	u32 search_len;
	u32 search_vba;
	u32 search_gen;
	bool search_valid;
	char status[512];
};

static inline u32 whimory_sig32(const u8 *sig, unsigned int off)
{
	return get_unaligned_le32(sig + off);
}

const char *whimory_recovery_state_name(void);

#endif /* WHIMORY_S5L8740_H */
