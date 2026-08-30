// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 FTL CS LBA map and read-only VFAT block front-end (N31).
 *
 * Builds a sparse LBA→physical cache from CS page metadata (4096+16 × 4
 * slots per page), compresses it into dual L2V/V2L vector tables, and
 * registers read-only disks after FAT-critical validation:
 *   /dev/s5l8740-ipod      — user FAT (disk_lba 0 @ fat_base_lba)
 *   /dev/s5l8740-ftl       — same FAT (compat alias)
 *   /dev/s5l8740-firmware  — pre-FAT fmss range, if mapped content found
 *
 * Authoritative map after `ftl_sftl_recover`: Whimory CXT→BTOC→L2V_Update
 * (CS META), with reads via L2V_Search. `ftl_map_build` (blk 62–66) is a
 * debug fallback only — not a journal replay.
 *
 * Every data return re-validates the on-media metadata LBA. Read-only:
 * no program, erase, or GC.
 */
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/hashtable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "nand-s5l8740.h"
#include "ftl-s5l8740-csmap.h"
#include "ftl-s5l8740-vecmap.h"

#define N31_FAT_BASE_DEFAULT		49279u
#define N31_FAT_TOTAL_DEFAULT		3856968u
#define N31_FMSS_LBA_MAX		(N31_FAT_BASE_DEFAULT + \
					 N31_FAT_TOTAL_DEFAULT + 65536u)
#define N31_BPB_CANDIDATES_MAX		16
#define N31_MAP_HASH_BITS		12
#define N31_EXTENT_MAX			2048
#define N31_PAGES_PER_BLOCK		128

#define N31_PHYS_KEY_INVALID		0xffffffffu

/* Sparse hash + extents for construction; L2V/V2L preferred on read. */

struct n31_map_entry {
	u32 fmss_lba;
	u32 phys_key;
	u64 weave;	/* full 48-bit weaveSeq — do not truncate */
	u8 type;
	u8 valid;
};

struct n31_map_node {
	struct hlist_node hnode;
	struct n31_map_entry e;
};

struct n31_lba_extent {
	u32 start_lba;
	u32 len;
	u32 start_phys_key;
	u64 weave_first;
	u8 type;
};

struct n31_fat_layout {
	u16 bytes_per_sector;
	u8 sectors_per_cluster;
	u16 reserved_sectors;
	u8 num_fats;
	u32 fat_size_32;
	u32 root_cluster;
	u16 fsinfo_sector;
	u16 backup_boot_sector;
	u16 ext_flags;		/* BPB_ExtFlags @ 0x28 */
	u32 total_sectors;
	u32 fat_start;
	u32 data_start;
	u32 root_dir_lba;
	u32 fsinfo_lba;
	u32 backup_boot_lba;
	bool valid;
};

enum n31_slice_kind {
	N31_SLICE_IPOD = 0,
	N31_SLICE_FTL_ALIAS,
	N31_SLICE_FIRMWARE,
};

struct n31_ftl_cs;

struct n31_ftl_slice {
	struct n31_ftl_cs *ftl;
	struct gendisk *gd;
	u32 base_fmss;
	u32 nsectors;
	enum n31_slice_kind kind;
};

struct n31_ftl_cs {
	struct device *dev;
	struct mutex lock;

	DECLARE_HASHTABLE(map, N31_MAP_HASH_BITS);
	unsigned int map_entries;
	unsigned int map_updates;
	unsigned int map_skips;
	unsigned int map_collisions;
	unsigned int map_pages;
	unsigned int map_data_recs;
	unsigned int newer_replacements;
	u32 lba_min, lba_max;
	bool map_built;

	struct n31_vecmap vec;
	char vec_log[384];

	struct n31_lba_extent *extents;
	unsigned int extent_count;
	unsigned int extent_largest;
	u32 open_ext_lba;
	u32 open_ext_phys;
	u32 open_ext_start_lba;
	u32 open_ext_start_phys;
	u64 open_ext_weave;
	u8 open_ext_type;
	u32 open_ext_len;

	u32 fat_base_lba;
	bool fat_base_valid;
	bool fat_base_autodetect;
	u32 fat_total_sectors;
	struct n31_fat_layout layout;
	u8 bpb_sector[N31_DATA_SLOT_SIZE];
	bool bpb_cached;

	u32 bpb_candidates[N31_BPB_CANDIDATES_MAX];
	u64 bpb_cand_weave[N31_BPB_CANDIDATES_MAX];
	u32 bpb_cand_total[N31_BPB_CANDIDATES_MAX];
	char bpb_cand_oem[N31_BPB_CANDIDATES_MAX][9];
	u8 bpb_cand_sector[N31_BPB_CANDIDATES_MAX][N31_DATA_SLOT_SIZE];
	unsigned int bpb_ncand;
	unsigned int fat_crit_ok_n;
	unsigned int fat_crit_need_n;

	u32 fw_base_fmss;
	u32 fw_nsectors;
	bool fw_valid;
	unsigned int fw_mapped;
	unsigned int fw_magic_hits;
	char fw_log[320];

	u8 last_sector[N31_DATA_SLOT_SIZE];
	u32 last_fmss_lba;
	u32 last_disk_lba;
	int last_ret;
	char last_log[768];
	char extents_log[512];
	char range_log[512];
	char layout_log[512];
	char bpb_log[384];

	unsigned int range_ok;
	unsigned int range_fail;
	unsigned int range_miss;
	unsigned int demand_scans;
	unsigned int read_miss_count;

	bool disk0_ok;
	bool fat_critical_ok;
	bool enable_gate_ok;
	bool block_enable;
	bool dma_session_held;
	bool whimory_backed; /* L2V_Search via Whimory recover */

	/* FAT semantic validation (beyond crit sector reads). */
	u32 fat_sem_root_chain_len;
	u32 fat_sem_root_entries;
	u32 fat_sem_music_dirs;
	u32 fat_sem_fat0_fat1_diff;
	bool fat_sem_itunesdb;
	bool fat_sem_apps;
	bool fat_sem_nanoapps;
	char fat_sem_log[512];
	char string_scan_log[384];

	struct n31_ftl_slice ipod;
	struct n31_ftl_slice ftl_alias;
	struct n31_ftl_slice firmware;
	u8 *bounce;
};

static int n31_ftl_find_bpb(struct n31_ftl_cs *ftl);
static int n31_ftl_select_bpb(struct n31_ftl_cs *ftl);
static int n31_validate_fat_critical(struct n31_ftl_cs *ftl);
static void n31_fat_semantic_validate(struct n31_ftl_cs *ftl);
static bool n31_fat_first_sector_ok(struct n31_ftl_cs *ftl);
static int n31_ftl_register_disk(struct n31_ftl_cs *ftl);
static void n31_ftl_unregister_disk(struct n31_ftl_cs *ftl);
static int n31_ftl_apply_bpb(struct n31_ftl_cs *ftl, u32 fmss_lba,
			     u32 total, const u8 *sector);
static bool n31_bpb_looks_valid(const u8 *d, u32 *total_out);

static struct n31_ftl_cs *n31_ftl;

/*
 * A read miss costs nine L2V probes plus ten console lines. VFAT retries a
 * failing directory cluster, so an unmapped chain used to bury the log and
 * slow the mount. Describe the first few, then count silently.
 */
static unsigned int read_miss_diag_max = 3;
module_param(read_miss_diag_max, uint, 0644);
MODULE_PARM_DESC(read_miss_diag_max,
		 "Read misses to describe in full before going quiet (0=all)");

static bool ftl_block_enable = true;
module_param(ftl_block_enable, bool, 0644);
MODULE_PARM_DESC(ftl_block_enable,
		 "Register RO ipod/firmware disks after FAT-critical gates (default Y)");

static bool ftl_demand_scan;
module_param(ftl_demand_scan, bool, 0644);
MODULE_PARM_DESC(ftl_demand_scan,
		 "On map miss, scan a nearby block for the LBA (default N; unsafe under mount I/O)");

static int fw_start_lba = -1;
module_param(fw_start_lba, int, 0644);
MODULE_PARM_DESC(fw_start_lba,
		 "Firmware slice start fmss_lba (-1 = auto below fat_base)");

static int fw_nsectors = -1;
module_param(fw_nsectors, int, 0644);
MODULE_PARM_DESC(fw_nsectors,
		 "Firmware slice length in 4096-byte sectors (-1 = auto)");

static bool fw_force;
module_param(fw_force, bool, 0644);
MODULE_PARM_DESC(fw_force,
		 "Register firmware disk even without magic/mapped hits (default N)");

/* -------------------- packed physical key -------------------- */

static u32 n31_phys_pack(u8 ce, u8 cau, u16 blk, u8 page, u8 slot)
{
	return ((u32)(ce & 0x3) << 30) |
	       ((u32)(cau & 0x3) << 28) |
	       ((u32)(blk & 0x3fff) << 14) |
	       ((u32)(page & 0xff) << 6) |
	       ((u32)(slot & 0x3));
}

static void n31_phys_unpack(u32 key, u8 *ce, u8 *cau, u16 *blk,
			    u8 *page, u8 *slot)
{
	*ce = (key >> 30) & 0x3;
	*cau = (key >> 28) & 0x3;
	*blk = (key >> 14) & 0x3fff;
	*page = (key >> 6) & 0xff;
	*slot = key & 0x3;
}

/* Advance one CS data slot: slot→…→3 → next page slot0 → next block. */
static u32 n31_phys_succ(u32 key)
{
	u8 ce, cau, page, slot;
	u16 blk;

	if (key == N31_PHYS_KEY_INVALID)
		return N31_PHYS_KEY_INVALID;
	n31_phys_unpack(key, &ce, &cau, &blk, &page, &slot);
	if (slot < 3)
		return n31_phys_pack(ce, cau, blk, page, slot + 1);
	if (page + 1 < N31_PAGES_PER_BLOCK)
		return n31_phys_pack(ce, cau, blk, page + 1, 0);
	return n31_phys_pack(ce, cau, blk + 1, 0, 0);
}

static u32 n31_phys_pred(u32 key)
{
	u8 ce, cau, page, slot;
	u16 blk;

	if (key == N31_PHYS_KEY_INVALID)
		return N31_PHYS_KEY_INVALID;
	n31_phys_unpack(key, &ce, &cau, &blk, &page, &slot);
	if (slot > 0)
		return n31_phys_pack(ce, cau, blk, page, slot - 1);
	if (page > 0)
		return n31_phys_pack(ce, cau, blk, page - 1, 3);
	if (blk > 0)
		return n31_phys_pack(ce, cau, blk - 1,
				     N31_PAGES_PER_BLOCK - 1, 3);
	return N31_PHYS_KEY_INVALID;
}

static u32 n31_phys_advance(u32 key, u32 delta)
{
	while (delta--) {
		key = n31_phys_succ(key);
		if (key == N31_PHYS_KEY_INVALID)
			break;
	}
	return key;
}

static void n31_entry_from_phys(struct n31_map_entry *e, u32 fmss_lba,
				u32 phys_key, u64 weave, u8 type)
{
	e->fmss_lba = fmss_lba;
	e->phys_key = phys_key;
	e->weave = weave;
	e->type = type;
	e->valid = 1;
}

static void n31_entry_to_legacy(const struct n31_map_entry *e,
				struct n31_lba_map_entry *out)
{
	u8 ce, cau, page, slot;
	u16 blk;

	memset(out, 0, sizeof(*out));
	if (!e || !e->valid)
		return;
	n31_phys_unpack(e->phys_key, &ce, &cau, &blk, &page, &slot);
	out->ce = ce;
	out->cau = cau;
	out->block = blk;
	out->page = page;
	out->slot = slot;
	out->type = e->type;
	out->weave = e->weave;
	out->fmss_lba = e->fmss_lba;
	out->present = true;
}

/* -------------------- BPB / layout -------------------- */

static bool n31_bpb_looks_valid(const u8 *d, u32 *total_out)
{
	u16 bps, root_ents, fat16;
	u32 total32, total16, fat32;
	u8 spc, fats;

	if (!d)
		return false;
	if (d[0] != 0xeb && d[0] != 0xe9)
		return false;
	if (d[0] == 0xeb && d[2] != 0x90)
		return false;
	bps = get_unaligned_le16(d + 11);
	spc = d[13];
	fats = d[16];
	root_ents = get_unaligned_le16(d + 17);
	fat16 = get_unaligned_le16(d + 22);
	total16 = get_unaligned_le16(d + 19);
	total32 = get_unaligned_le32(d + 32);
	fat32 = get_unaligned_le32(d + 36);
	if (bps != 4096 || spc != 4 || fats == 0 || fats > 2)
		return false;
	if (!total32)
		total32 = total16;
	if (!total32 || total32 < 1000u || total32 > 16u * 1024u * 1024u)
		return false;
	/*
	 * FAT32: root_ents==0 and FATsz16==0 → require FATsz32.
	 * Reject MSDOS5.0-style stubs that advertise FAT32 in the type
	 * string but leave FATsz32=0 (not a mountable volume).
	 */
	if (root_ents == 0 && fat16 == 0 && fat32 == 0)
		return false;
	if (total_out)
		*total_out = total32;
	return true;
}

static int n31_parse_bpb(const u8 *d, struct n31_fat_layout *L)
{
	u32 total = 0;

	memset(L, 0, sizeof(*L));
	if (!n31_bpb_looks_valid(d, &total))
		return -EINVAL;

	L->bytes_per_sector = get_unaligned_le16(d + 11);
	L->sectors_per_cluster = d[13];
	L->reserved_sectors = get_unaligned_le16(d + 14);
	L->num_fats = d[16];
	L->total_sectors = total;
	L->fat_size_32 = get_unaligned_le32(d + 36);
	L->ext_flags = get_unaligned_le16(d + 0x28);
	L->root_cluster = get_unaligned_le32(d + 44);
	L->fsinfo_sector = get_unaligned_le16(d + 48);
	L->backup_boot_sector = get_unaligned_le16(d + 50);

	L->fat_start = L->reserved_sectors;
	L->data_start = L->reserved_sectors +
			(u32)L->num_fats * L->fat_size_32;
	if (L->root_cluster >= 2)
		L->root_dir_lba = L->data_start +
			(L->root_cluster - 2) * L->sectors_per_cluster;
	else
		L->root_dir_lba = L->data_start;
	L->fsinfo_lba = L->fsinfo_sector;
	L->backup_boot_lba = L->backup_boot_sector;
	L->valid = true;
	return 0;
}

/* -------------------- sparse map + extents -------------------- */

static void n31_extent_close(struct n31_ftl_cs *ftl)
{
	struct n31_lba_extent *ex;

	if (ftl->open_ext_len == 0)
		return;
	if (ftl->extent_count >= N31_EXTENT_MAX) {
		ftl->open_ext_len = 0;
		return;
	}
	if (!ftl->extents) {
		ftl->extents = kcalloc(N31_EXTENT_MAX,
				       sizeof(*ftl->extents), GFP_KERNEL);
		if (!ftl->extents) {
			ftl->open_ext_len = 0;
			return;
		}
	}
	ex = &ftl->extents[ftl->extent_count++];
	ex->start_lba = ftl->open_ext_start_lba;
	ex->len = ftl->open_ext_len;
	ex->start_phys_key = ftl->open_ext_start_phys;
	ex->weave_first = ftl->open_ext_weave;
	ex->type = ftl->open_ext_type;
	if (ex->len > ftl->extent_largest)
		ftl->extent_largest = ex->len;
	ftl->open_ext_len = 0;
}

static void n31_extent_feed(struct n31_ftl_cs *ftl, u32 fmss_lba,
			    u32 phys_key, u64 weave, u8 type)
{
	if (ftl->open_ext_len == 0) {
		ftl->open_ext_start_lba = fmss_lba;
		ftl->open_ext_start_phys = phys_key;
		ftl->open_ext_lba = fmss_lba;
		ftl->open_ext_phys = phys_key;
		ftl->open_ext_weave = weave;
		ftl->open_ext_type = type;
		ftl->open_ext_len = 1;
		return;
	}

	if (fmss_lba == ftl->open_ext_lba + 1 &&
	    phys_key == n31_phys_succ(ftl->open_ext_phys) &&
	    (type == ftl->open_ext_type ||
	     (type <= 2 && ftl->open_ext_type <= 2))) {
		ftl->open_ext_lba = fmss_lba;
		ftl->open_ext_phys = phys_key;
		ftl->open_ext_len++;
		return;
	}

	n31_extent_close(ftl);
	ftl->open_ext_start_lba = fmss_lba;
	ftl->open_ext_start_phys = phys_key;
	ftl->open_ext_lba = fmss_lba;
	ftl->open_ext_phys = phys_key;
	ftl->open_ext_weave = weave;
	ftl->open_ext_type = type;
	ftl->open_ext_len = 1;
}

static void n31_extents_reset(struct n31_ftl_cs *ftl)
{
	n31_extent_close(ftl);
	ftl->extent_count = 0;
	ftl->extent_largest = 0;
	ftl->open_ext_len = 0;
}

static int n31_extent_lookup(struct n31_ftl_cs *ftl, u32 fmss_lba,
			     struct n31_map_entry *out)
{
	unsigned int i;

	for (i = 0; i < ftl->extent_count; i++) {
		struct n31_lba_extent *ex = &ftl->extents[i];
		u32 off;

		if (fmss_lba < ex->start_lba ||
		    fmss_lba >= ex->start_lba + ex->len)
			continue;
		off = fmss_lba - ex->start_lba;
		n31_entry_from_phys(out, fmss_lba,
				    n31_phys_advance(ex->start_phys_key, off),
				    ex->weave_first, ex->type);
		return 0;
	}
	return -ENOENT;
}

static void n31_map_free(struct n31_ftl_cs *ftl)
{
	unsigned int bkt;
	struct n31_map_node *n;
	struct hlist_node *tmp;

	hash_for_each_safe(ftl->map, bkt, tmp, n, hnode) {
		hash_del(&n->hnode);
		kfree(n);
	}
	ftl->map_entries = 0;
	ftl->map_updates = 0;
	ftl->map_skips = 0;
	ftl->map_collisions = 0;
	ftl->map_pages = 0;
	ftl->map_data_recs = 0;
	ftl->newer_replacements = 0;
	ftl->lba_min = ~0u;
	ftl->lba_max = 0;
	ftl->map_built = false;
	ftl->disk0_ok = false;
	ftl->fat_critical_ok = false;
	ftl->enable_gate_ok = false;
	n31_extents_reset(ftl);
	kfree(ftl->extents);
	ftl->extents = NULL;
	n31_vecmap_free(&ftl->vec);
	ftl->vec_log[0] = '\0';
	ftl->fw_valid = false;
	ftl->fw_mapped = 0;
	ftl->fw_magic_hits = 0;
	ftl->fw_nsectors = 0;
	ftl->fw_log[0] = '\0';
}

static struct n31_map_node *n31_map_find(struct n31_ftl_cs *ftl, u32 fmss_lba)
{
	struct n31_map_node *n;

	hash_for_each_possible(ftl->map, n, hnode, fmss_lba) {
		if (n->e.valid && n->e.fmss_lba == fmss_lba)
			return n;
	}
	return NULL;
}

static void n31_map_ingest(struct n31_ftl_cs *ftl, u8 ce, u8 cau,
			   u16 block, u8 page, u8 slot,
			   const struct s5l8740_meta_decoded *m)
{
	struct n31_map_node *n;
	u32 phys;
	u64 weave;

	if (!n31_meta_is_data_record(m)) {
		ftl->map_skips++;
		return;
	}
	if (m->lba >= N31_FMSS_LBA_MAX) {
		ftl->map_skips++;
		return;
	}

	phys = n31_phys_pack(ce, cau, block, page, slot);
	weave = m->weave;
	ftl->map_data_recs++;
	if (m->lba < ftl->lba_min)
		ftl->lba_min = m->lba;
	if (m->lba > ftl->lba_max)
		ftl->lba_max = m->lba;

	n = n31_map_find(ftl, m->lba);
	if (!n) {
		n = kzalloc(sizeof(*n), GFP_KERNEL);
		if (!n) {
			ftl->map_skips++;
			return;
		}
		n31_entry_from_phys(&n->e, m->lba, phys, weave, m->type);
		hash_add(ftl->map, &n->hnode, m->lba);
		ftl->map_entries++;
		n31_extent_feed(ftl, m->lba, phys, weave, m->type);
		return;
	}

	if (weave == n->e.weave && n->e.phys_key != phys) {
		ftl->map_collisions++;
		dev_dbg(ftl->dev,
			"lba=%u weave collision old_phys=%08x new_phys=%08x\n",
			m->lba, n->e.phys_key, phys);
		return;
	}
	if (!n31_weave_newer(weave, n->e.weave)) {
		/* Stale or equal — never feed extents from losers. */
		ftl->map_skips++;
		return;
	}

	ftl->newer_replacements++;
	ftl->map_updates++;
	n31_entry_from_phys(&n->e, m->lba, phys, weave, m->type);
	n31_extent_feed(ftl, m->lba, phys, weave, m->type);
}

static int n31_scan_page(struct n31_ftl_cs *ftl, u8 ce, u8 cau,
			 u16 block, u8 page)
{
	struct s5l8740_cs_page *pg;
	int ret, s;

	pg = kzalloc(sizeof(*pg), GFP_KERNEL);
	if (!pg)
		return -ENOMEM;
	ret = s5l8740_nand_cs_phys_read(ce, cau, block, page, pg);
	if (!ret) {
		ftl->map_pages++;
		for (s = 0; s < N31_DATA_SLOTS; s++)
			n31_map_ingest(ftl, ce, cau, block, page, s,
				       &pg->meta[s]);
	}
	kfree(pg);
	return ret;
}

/*
 * Compress newest-weave hash entries into L2V/V2L. Pair scratch is freed
 * after build; the hash remains available for misses and rebuilds.
 */
static int n31_vecmap_rebuild_from_hash(struct n31_ftl_cs *ftl)
{
	struct n31_vec_pair *pairs;
	struct n31_map_node *n;
	unsigned int bkt, i = 0, nent;
	int ret;
	u8 ce, cau, page, slot;
	u16 blk;

	nent = ftl->map_entries;
	if (!nent) {
		n31_vecmap_free(&ftl->vec);
		scnprintf(ftl->vec_log, sizeof(ftl->vec_log),
			  "ready=0 pairs=0\n");
		return 0;
	}

	pairs = vmalloc(array_size(nent, sizeof(*pairs)));
	if (!pairs) {
		scnprintf(ftl->vec_log, sizeof(ftl->vec_log),
			  "ready=0 err=-ENOMEM pairs=%u\n", nent);
		return -ENOMEM;
	}

	hash_for_each(ftl->map, bkt, n, hnode) {
		if (i >= nent)
			break;
		n31_phys_unpack(n->e.phys_key, &ce, &cau, &blk, &page, &slot);
		pairs[i].l = n->e.fmss_lba;
		pairs[i].p = n31_phys_to_ordinal(ce, cau, blk, page, slot);
		pairs[i].weave = n->e.weave;
		i++;
	}

	ret = n31_vecmap_build(&ftl->vec, pairs, i);
	vfree(pairs);
	if (ret) {
		scnprintf(ftl->vec_log, sizeof(ftl->vec_log),
			  "ready=0 err=%d pairs=%u\n", ret, i);
		return ret;
	}

	scnprintf(ftl->vec_log, sizeof(ftl->vec_log),
		  "ready=1 pairs=%u l_base=%u l_count=%u p_base=%u p_count=%u "
		  "l2v_groups=%u v2l_groups=%u compact_ok=%u esc=%u miss=%u "
		  "l2v_esc_n=%u v2l_esc_n=%u\n",
		  i, ftl->vec.l_base, ftl->vec.l_count,
		  ftl->vec.p_base, ftl->vec.p_count,
		  ftl->vec.l2v_groups, ftl->vec.v2l_groups,
		  ftl->vec.compact_ok, ftl->vec.compact_esc,
		  ftl->vec.compact_miss, ftl->vec.l2v_esc_n,
		  ftl->vec.v2l_esc_n);
	return 0;
}

static void n31_scan_finish_stats(struct n31_ftl_cs *ftl, const char *tag,
				  unsigned int pages_ok,
				  unsigned int pages_fail,
				  bool rebuild_vec)
{
	int vret = 0;

	n31_extent_close(ftl);
	ftl->map_built = ftl->map_entries > 0;
	if (rebuild_vec)
		vret = n31_vecmap_rebuild_from_hash(ftl);
	scnprintf(ftl->last_log, sizeof(ftl->last_log),
		  "%s pages_ok=%u pages_fail=%u records=%u "
		  "lba_min=%u lba_max=%u extents=%u largest=%u "
		  "collisions=%u replacements=%u has_fat_base=%d "
		  "vec_ready=%d vec_ret=%d\n",
		  tag, pages_ok, pages_fail, ftl->map_data_recs,
		  ftl->lba_min == ~0u ? 0 : ftl->lba_min, ftl->lba_max,
		  ftl->extent_count, ftl->extent_largest,
		  ftl->map_collisions, ftl->newer_replacements,
		  n31_map_find(ftl, ftl->fat_base_lba) ? 1 : 0,
		  ftl->vec.ready ? 1 : 0, vret);
	scnprintf(ftl->extents_log, sizeof(ftl->extents_log),
		  "extent_count=%u largest=%u open_len=%u\n",
		  ftl->extent_count, ftl->extent_largest, ftl->open_ext_len);
	dev_dbg(ftl->dev, "%s", ftl->last_log);
	if (rebuild_vec && ftl->vec_log[0])
		dev_dbg(ftl->dev, "vecmap %s", ftl->vec_log);
}

/*
 * Scan CE/CAU × blk_lo..blk_hi × pages. Appends unless @reset.
 * Full vector rebuild only when @rebuild_vec (not on demand I/O fills).
 */
static int n31_scan_block_window(struct n31_ftl_cs *ftl, u8 ce, u8 cau,
				 u16 blk_lo, u16 blk_hi, bool reset,
				 bool rebuild_vec)
{
	u16 blk;
	u8 pg;
	unsigned int pages_ok = 0, pages_fail = 0;
	int ret;

	if (reset) {
		n31_map_free(ftl);
		hash_init(ftl->map);
		ftl->lba_min = ~0u;
		ftl->lba_max = 0;
	}

	for (blk = blk_lo; blk <= blk_hi; blk++) {
		for (pg = 0; pg < N31_PAGES_PER_BLOCK; pg++) {
			ret = n31_scan_page(ftl, ce, cau, blk, pg);
			if (ret)
				pages_fail++;
			else
				pages_ok++;
			cond_resched();
		}
	}
	n31_scan_finish_stats(ftl, "scan_window", pages_ok, pages_fail,
			      rebuild_vec);
	return 0;
}

/* Scan blk_lo..blk_hi on every CE/CAU; reset once; compress vectors once. */
static int n31_scan_banks_window(struct n31_ftl_cs *ftl, u16 blk_lo,
				 u16 blk_hi, bool reset)
{
	u8 ce, cau;
	bool do_reset = reset;

	for (ce = 0; ce < N31_VEC_NUM_CE; ce++) {
		for (cau = 0; cau < N31_VEC_NUM_CAU; cau++) {
			bool last = (ce == N31_VEC_NUM_CE - 1) &&
				    (cau == N31_VEC_NUM_CAU - 1);

			n31_scan_block_window(ftl, ce, cau, blk_lo, blk_hi,
					     do_reset, last);
			do_reset = false;
		}
	}
	return 0;
}

static int n31_map_lookup_hint(struct n31_ftl_cs *ftl, u32 fmss_lba,
			       struct n31_map_entry *out)
{
	struct n31_map_node *n;

	n = n31_map_find(ftl, fmss_lba);
	if (n) {
		*out = n->e;
		return 0;
	}
	return n31_extent_lookup(ftl, fmss_lba, out);
}

/*
 * Optional nearby-block fill for map misses. Must not run under block I/O
 * with a multi-block window or vector rebuild — that softlocks the system.
 */
static int n31_demand_scan_for(struct n31_ftl_cs *ftl, u32 fmss_lba)
{
	struct n31_map_entry hint;
	u8 ce = 0, cau = 0;
	u16 blk = 63;
	u8 page = 88, slot;
	int ret;
	u16 lo, hi;

	ftl->demand_scans++;
	if (fmss_lba > 0 &&
	    !n31_map_lookup_hint(ftl, fmss_lba - 1, &hint)) {
		n31_phys_unpack(hint.phys_key, &ce, &cau, &blk, &page, &slot);
	} else if (!n31_map_lookup_hint(ftl, fmss_lba + 1, &hint)) {
		n31_phys_unpack(hint.phys_key, &ce, &cau, &blk, &page, &slot);
	} else if (!n31_map_lookup_hint(ftl, ftl->fat_base_lba, &hint)) {
		n31_phys_unpack(hint.phys_key, &ce, &cau, &blk, &page, &slot);
	}
	(void)slot;

	dev_dbg(ftl->dev,
		"demand_scan lba=%u ce=%u cau=%u blk=%u\n",
		fmss_lba, ce, cau, blk);

	lo = blk;
	hi = blk;

	/* Home bank first, then the other CE/CAU at the same block. */
	ret = n31_scan_block_window(ftl, ce, cau, lo, hi, false, false);
	if (n31_map_find(ftl, fmss_lba) ||
	    !n31_extent_lookup(ftl, fmss_lba, &hint))
		return 0;
	{
		u8 tce, tcau;

		for (tce = 0; tce < N31_VEC_NUM_CE; tce++) {
			for (tcau = 0; tcau < N31_VEC_NUM_CAU; tcau++) {
				if (tce == ce && tcau == cau)
					continue;
				n31_scan_block_window(ftl, tce, tcau, lo, hi,
						      false, false);
				if (n31_map_find(ftl, fmss_lba) ||
				    !n31_extent_lookup(ftl, fmss_lba, &hint))
					return 0;
			}
		}
	}
	return ret ? ret : -ENOENT;
}

static void n31_map_ingest_page(struct n31_ftl_cs *ftl, u8 ce, u8 cau,
				u16 block, u8 page,
				const struct s5l8740_cs_page *pg)
{
	int s;

	for (s = 0; s < N31_DATA_SLOTS; s++)
		n31_map_ingest(ftl, ce, cau, block, page, s, &pg->meta[s]);
}

/*
 * One-page fill from a mapped neighbour's CS page. Safe under block I/O.
 * Also tries the predicted successor/predecessor slot's page when distinct.
 */
static int n31_neighbor_probe(struct n31_ftl_cs *ftl, u32 fmss_lba,
			      struct n31_map_entry *out)
{
	struct n31_map_entry hint;
	struct s5l8740_cs_page *page;
	u32 try_key[4];
	unsigned int ntry = 0, i, j;
	u8 ce, cau, pg, sl;
	u16 blk;
	int slot, ret;

	if (fmss_lba > 0 &&
	    !n31_map_lookup_hint(ftl, fmss_lba - 1, &hint)) {
		try_key[ntry++] = hint.phys_key;
		try_key[ntry++] = n31_phys_succ(hint.phys_key);
	}
	if (!n31_map_lookup_hint(ftl, fmss_lba + 1, &hint)) {
		try_key[ntry++] = hint.phys_key;
		try_key[ntry++] = n31_phys_pred(hint.phys_key);
	}
	if (!ntry)
		return -ENOENT;

	page = kzalloc(sizeof(*page), GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	for (i = 0; i < ntry; i++) {
		if (try_key[i] == N31_PHYS_KEY_INVALID)
			continue;
		/* Skip duplicate pages already tried. */
		for (j = 0; j < i; j++) {
			u8 ce2, cau2, pg2, sl2;
			u16 blk2;

			if (try_key[j] == N31_PHYS_KEY_INVALID)
				continue;
			n31_phys_unpack(try_key[i], &ce, &cau, &blk, &pg, &sl);
			n31_phys_unpack(try_key[j], &ce2, &cau2, &blk2, &pg2,
					&sl2);
			if (ce == ce2 && cau == cau2 && blk == blk2 &&
			    pg == pg2)
				goto next;
		}
		n31_phys_unpack(try_key[i], &ce, &cau, &blk, &pg, &sl);
		(void)sl;
		ret = s5l8740_nand_cs_phys_read(ce, cau, blk, pg, page);
		if (ret)
			continue;
		n31_map_ingest_page(ftl, ce, cau, blk, pg, page);
		slot = s5l8740_nand_meta_pick_lba(page, fmss_lba);
		if (slot >= 0) {
			kfree(page);
			return n31_map_lookup_hint(ftl, fmss_lba, out);
		}
next:
		;
	}
	kfree(page);
	return -ENOENT;
}

static int n31_ftl_read_fmss_lba_flags(struct n31_ftl_cs *ftl, u32 fmss_lba,
				       void *dst, bool allow_demand)
{
	struct n31_map_entry e;
	struct n31_lba_map_entry leg;
	struct s5l8740_cs_page *page;
	int slot, ret;
	u8 ce, cau, pg, sl;
	u16 blk;
	u32 p_ord;
	bool have_hint = false;

	if (!ftl || !dst)
		return -EINVAL;

	/* Whimory L2V is authoritative after CXT→BTOC recover. */
	if (ftl->whimory_backed && whimory_l2v_ready()) {
		dev_dbg(ftl->dev,
			"read disk? via L2V_Search fmss_lba=%u\n",
			fmss_lba);
		ret = whimory_read_fmss_lba(fmss_lba, dst);
		if (!ret)
			return 0;
		/* Fall through to hash/vec if Search miss (sparse holes). */
	}

	if (ftl->vec.ready) {
		ret = n31_vecmap_lookup(&ftl->vec, fmss_lba, &p_ord);
		if (!ret) {
			n31_ordinal_to_phys(p_ord, &ce, &cau, &blk, &pg, &sl);
			n31_entry_from_phys(&e, fmss_lba,
					    n31_phys_pack(ce, cau, blk, pg, sl),
					    0, 0x01);
			have_hint = true;
		} else if (ret == -EUCLEAN) {
			return -EUCLEAN;
		}
	}

	if (!have_hint) {
		ret = n31_map_lookup_hint(ftl, fmss_lba, &e);
		if (ret)
			ret = n31_neighbor_probe(ftl, fmss_lba, &e);
		if (ret && allow_demand && ftl_demand_scan) {
			ret = n31_demand_scan_for(ftl, fmss_lba);
			if (!ret)
				ret = n31_map_lookup_hint(ftl, fmss_lba, &e);
		}
		if (ret)
			return -ENOENT;
	}

	n31_entry_to_legacy(&e, &leg);
	ce = leg.ce;
	cau = leg.cau;
	blk = leg.block;
	pg = leg.page;

	page = kzalloc(sizeof(*page), GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	ret = s5l8740_nand_cs_phys_read(ce, cau, blk, pg, page);
	if (ret) {
		ret = -EIO;
		goto out;
	}

	n31_map_ingest_page(ftl, ce, cau, blk, pg, page);

	slot = s5l8740_nand_meta_pick_lba(page, fmss_lba);
	if (slot < 0) {
		ret = -EUCLEAN;
		goto out;
	}
	n31_phys_unpack(e.phys_key, &ce, &cau, &blk, &pg, &sl);
	if ((u8)slot != sl)
		dev_dbg(ftl->dev,
			"lba=%u map_slot=%u meta_slot=%u\n",
			fmss_lba, sl, slot);

	memcpy(dst, page->data[slot], N31_DATA_SLOT_SIZE);
	ret = 0;
out:
	kfree(page);
	return ret;
}

int n31_ftl_read_fmss_lba(struct n31_ftl_cs *ftl, u32 fmss_lba, void *dst)
{
	return n31_ftl_read_fmss_lba_flags(ftl, fmss_lba, dst, true);
}

static int n31_ftl_read_disk_lba_flags(struct n31_ftl_cs *ftl, u32 disk_lba,
				       void *dst, bool allow_demand)
{
	u32 fmss_lba;

	if (!ftl || !dst)
		return -EINVAL;
	if (!ftl->fat_base_valid)
		return -ENODEV;
	if (disk_lba >= ftl->fat_total_sectors)
		return -ERANGE;

	fmss_lba = ftl->fat_base_lba + disk_lba;
	dev_dbg(ftl->dev,
		"disk_lba=%u -> fmss_lba=%u (fat_base=%u)\n",
		disk_lba, fmss_lba, ftl->fat_base_lba);
	return n31_ftl_read_fmss_lba_flags(ftl, fmss_lba, dst, allow_demand);
}

int n31_ftl_read_disk_lba(struct n31_ftl_cs *ftl, u32 disk_lba, void *dst)
{
	return n31_ftl_read_disk_lba_flags(ftl, disk_lba, dst, true);
}

static int n31_ftl_apply_bpb(struct n31_ftl_cs *ftl, u32 fmss_lba,
			     u32 total, const u8 *sector)
{
	if (!sector)
		return -EINVAL;
	ftl->fat_base_lba = fmss_lba;
	ftl->fat_total_sectors = total;
	ftl->fat_base_valid = true;
	memcpy(ftl->bpb_sector, sector, N31_DATA_SLOT_SIZE);
	n31_parse_bpb(sector, &ftl->layout);
	ftl->bpb_cached = true;
	return 0;
}

static int n31_ftl_find_bpb(struct n31_ftl_cs *ftl)
{
	unsigned int bkt;
	struct n31_map_node *n;
	u8 *buf;
	int ret, sess;
	unsigned int found = 0;
	char cand_log[512];
	unsigned int cand_n = 0;

	buf = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY) {
		kfree(buf);
		return sess;
	}

	mutex_lock(&ftl->lock);
	ftl->bpb_ncand = 0;
	ftl->fat_base_valid = false;
	ftl->bpb_cached = false;
	cand_log[0] = '\0';

	hash_for_each(ftl->map, bkt, n, hnode) {
		u32 total = 0;
		unsigned int i;

		if (!n->e.valid)
			continue;
		if (n->e.type != S5L8740_NAND_META_TYPE_DATA &&
		    n->e.type != S5L8740_NAND_META_TYPE_DATA2)
			continue;
		ret = n31_ftl_read_fmss_lba(ftl, n->e.fmss_lba, buf);
		if (ret)
			continue;
		if (!n31_bpb_looks_valid(buf, &total))
			continue;

		/* Dedup same fmss_lba (keep higher weave). */
		for (i = 0; i < ftl->bpb_ncand; i++) {
			if (ftl->bpb_candidates[i] != n->e.fmss_lba)
				continue;
			if (n31_weave_newer(n->e.weave, ftl->bpb_cand_weave[i])) {
				ftl->bpb_cand_weave[i] = n->e.weave;
				ftl->bpb_cand_total[i] = total;
				memcpy(ftl->bpb_cand_oem[i], buf + 3, 8);
				ftl->bpb_cand_oem[i][8] = '\0';
				memcpy(ftl->bpb_cand_sector[i], buf,
				       N31_DATA_SLOT_SIZE);
			}
			goto next;
		}
		if (ftl->bpb_ncand < N31_BPB_CANDIDATES_MAX) {
			i = ftl->bpb_ncand++;
			ftl->bpb_candidates[i] = n->e.fmss_lba;
			ftl->bpb_cand_weave[i] = n->e.weave;
			ftl->bpb_cand_total[i] = total;
			memcpy(ftl->bpb_cand_oem[i], buf + 3, 8);
			ftl->bpb_cand_oem[i][8] = '\0';
			memcpy(ftl->bpb_cand_sector[i], buf, N31_DATA_SLOT_SIZE);
		}
		found++;
next:
		;
	}

	for (bkt = 0; bkt < ftl->bpb_ncand && cand_n < sizeof(cand_log) - 96;
	     bkt++)
		cand_n += scnprintf(cand_log + cand_n, sizeof(cand_log) - cand_n,
				    " cand%u fmss=%u weave=%012llx oem='%.8s' "
				    "total=%u\n",
				    bkt + 1, ftl->bpb_candidates[bkt],
				    (unsigned long long)ftl->bpb_cand_weave[bkt],
				    ftl->bpb_cand_oem[bkt],
				    ftl->bpb_cand_total[bkt]);

	scnprintf(ftl->bpb_log, sizeof(ftl->bpb_log),
		  "bpb_scan candidates=%u\n%s", ftl->bpb_ncand, cand_log);
	scnprintf(ftl->last_log, sizeof(ftl->last_log), "%s", ftl->bpb_log);
	dev_info(ftl->dev, "%s", ftl->last_log);
	mutex_unlock(&ftl->lock);

	if (sess == 0)
		s5l8740_nand_dma_session_end();
	kfree(buf);
	return found || ftl->bpb_ncand ? 0 : -ENOENT;
}

/*
 * Newest weave first among candidates that pass FAT-critical. A slightly
 * older *UOKJIHC volume that mounts beats a newer BPB with a broken FAT.
 */
static int n31_ftl_select_bpb(struct n31_ftl_cs *ftl)
{
	unsigned int i, j;
	int best = -1;
	unsigned int best_ok = 0;
	bool best_apple = false;
	u64 best_weave = 0;

	if (!ftl->bpb_ncand)
		return -ENOENT;

	/* Insertion-sort candidates by weave descending (n is tiny). */
	for (i = 1; i < ftl->bpb_ncand; i++) {
		u32 fmss = ftl->bpb_candidates[i];
		u64 weave = ftl->bpb_cand_weave[i];
		u32 total = ftl->bpb_cand_total[i];
		char oem[9];
		u8 sector[N31_DATA_SLOT_SIZE];

		memcpy(oem, ftl->bpb_cand_oem[i], 9);
		memcpy(sector, ftl->bpb_cand_sector[i], N31_DATA_SLOT_SIZE);
		j = i;
		while (j > 0 &&
		       n31_weave_newer(weave, ftl->bpb_cand_weave[j - 1])) {
			ftl->bpb_candidates[j] = ftl->bpb_candidates[j - 1];
			ftl->bpb_cand_weave[j] = ftl->bpb_cand_weave[j - 1];
			ftl->bpb_cand_total[j] = ftl->bpb_cand_total[j - 1];
			memcpy(ftl->bpb_cand_oem[j], ftl->bpb_cand_oem[j - 1],
			       9);
			memcpy(ftl->bpb_cand_sector[j],
			       ftl->bpb_cand_sector[j - 1],
			       N31_DATA_SLOT_SIZE);
			j--;
		}
		ftl->bpb_candidates[j] = fmss;
		ftl->bpb_cand_weave[j] = weave;
		ftl->bpb_cand_total[j] = total;
		memcpy(ftl->bpb_cand_oem[j], oem, 9);
		memcpy(ftl->bpb_cand_sector[j], sector, N31_DATA_SLOT_SIZE);
	}

	for (i = 0; i < ftl->bpb_ncand; i++) {
		bool apple, fatsig;
		int vret;

		n31_ftl_apply_bpb(ftl, ftl->bpb_candidates[i],
				  ftl->bpb_cand_total[i],
				  ftl->bpb_cand_sector[i]);
		vret = n31_validate_fat_critical(ftl);
		apple = !memcmp(ftl->bpb_cand_oem[i], "*UOKJIHC", 8);
		fatsig = n31_fat_first_sector_ok(ftl);

		dev_info(ftl->dev,
			 "bpb_try fmss=%u weave=%012llx oem='%.8s' "
			 "crit=%u/%u ret=%d\n",
			 ftl->bpb_candidates[i],
			 (unsigned long long)ftl->bpb_cand_weave[i],
			 ftl->bpb_cand_oem[i], ftl->fat_crit_ok_n,
			 ftl->fat_crit_need_n, vret);

		dev_info(ftl->dev, "bpb_try fmss=%u fatsig=%d\n",
			 ftl->bpb_candidates[i], fatsig);

		/*
		 * A candidate whose first FAT sector is not a FAT is the wrong
		 * volume base, however well its critical sectors read. That is
		 * the entire failure this check exists for, so it gates both
		 * the early-out below and the scoring after it.
		 */
		if (!fatsig)
			continue;

		/* Perfect critical set: newest weave wins immediately. */
		if (!vret && ftl->fat_crit_ok_n == ftl->fat_crit_need_n &&
		    ftl->fat_crit_need_n > 0) {
			best = i;
			break;
		}

		if (ftl->fat_crit_ok_n < 3)
			continue;
		if (best < 0 || ftl->fat_crit_ok_n > best_ok ||
		    (ftl->fat_crit_ok_n == best_ok && apple && !best_apple) ||
		    (ftl->fat_crit_ok_n == best_ok && apple == best_apple &&
		     n31_weave_newer(ftl->bpb_cand_weave[i], best_weave))) {
			best = i;
			best_ok = ftl->fat_crit_ok_n;
			best_apple = apple;
			best_weave = ftl->bpb_cand_weave[i];
		}
	}

	if (best < 0)
		best = 0;

	n31_ftl_apply_bpb(ftl, ftl->bpb_candidates[best],
			  ftl->bpb_cand_total[best],
			  ftl->bpb_cand_sector[best]);
	n31_validate_fat_critical(ftl);

	scnprintf(ftl->bpb_log, sizeof(ftl->bpb_log),
		  "fat_base_lba=%u valid=%d total=%u candidates=%u "
		  "oem='%.8s' weave=%012llx ext_flags=0x%04x "
		  "active_fat=%u mirror=%s selected=%u crit=%u/%u "
		  "itunesdb=%d music_dirs=%u fat1_disk_lba=%u\n",
		  ftl->fat_base_lba, ftl->fat_base_valid, ftl->fat_total_sectors,
		  ftl->bpb_ncand, ftl->bpb_cand_oem[best],
		  (unsigned long long)ftl->bpb_cand_weave[best],
		  ftl->layout.ext_flags, ftl->layout.ext_flags & 0xF,
		  (ftl->layout.ext_flags & 0x80) ? "off" : "on",
		  best + 1, ftl->fat_crit_ok_n, ftl->fat_crit_need_n,
		  ftl->fat_sem_itunesdb, ftl->fat_sem_music_dirs,
		  ftl->layout.fat_start + ftl->layout.fat_size_32);
	dev_info(ftl->dev, "%s", ftl->bpb_log);
	for (i = 0; i < ftl->bpb_ncand; i++)
		dev_info(ftl->dev,
			 "BPB_CAND #%u fmss_lba=%u weave=%012llx oem='%.8s' "
			 "total=%u selected=%s reason=%s\n",
			 i + 1, ftl->bpb_candidates[i],
			 (unsigned long long)ftl->bpb_cand_weave[i],
			 ftl->bpb_cand_oem[i], ftl->bpb_cand_total[i],
			 i == (unsigned int)best ? "yes" : "no",
			 i == (unsigned int)best ?
				"newest_valid_high_crit" : "not_selected");
	return ftl->fat_critical_ok ? 0 : -EAGAIN;
}

static int n31_read_disk_checked(struct n31_ftl_cs *ftl, u32 disk_lba,
				 u8 *buf, const char *tag)
{
	int ret = n31_ftl_read_disk_lba(ftl, disk_lba, buf);

	if (ret)
		dev_warn_ratelimited(ftl->dev,
				     "FAT-critical %s disk_lba=%u fail %d\n",
				     tag, disk_lba, ret);
	else
		dev_dbg(ftl->dev, "FAT-critical %s disk_lba=%u OK\n",
			tag, disk_lba);
	return ret;
}

/*
 * Semantic FAT checks beyond fat_critical N/N sector readability.
 * Walk root cluster chain, count dir entries, search for known iPod names.
 * FAT1 starts at reserved + fat_size32 (e.g. 32+942=974), not disk_lba=33.
 */
/*
 * Does the first FAT sector actually look like the start of a FAT?
 *
 * n31_validate_fat_critical() only checks that its nine critical sectors
 * *read*. It never looks at what came back, so a BPB whose volume base is
 * wrong still scores a perfect 9/9 as long as the shifted sectors happen to
 * be mapped -- which they were:
 *
 *   BPB_CAND #1 fmss_lba=49285 weave=..ad crit=9/9 selected
 *   BPB_CAND #2 fmss_lba=49279 weave=..ac
 *
 * Six sectors apart, and the FAT belongs to the older one. Selecting #1 put
 * the whole volume six sectors out: sector 0 still read as a valid BPB and
 * sector 1 as a valid FSInfo, because those are static and were found by
 * content, but the FAT region was offset. Reading it back showed a perfectly
 * well-formed FAT32 chain stepping 0x400 per 4096-byte sector and starting
 * six sectors early -- so vfat walked into the middle of the table and got
 * "invalid cluster chain".
 *
 * Every FAT32 begins entry 0 with the media descriptor in the low byte and
 * the FAT32 12-bit-wide EOC nibbles above it: F8 FF FF 0F. That single test
 * separates the two candidates, and nothing that is genuinely the first FAT
 * sector can fail it.
 */
static bool n31_fat_first_sector_ok(struct n31_ftl_cs *ftl)
{
	struct n31_fat_layout *L = &ftl->layout;
	u8 *buf;
	bool ok = false;

	if (!ftl->fat_base_valid || !L->fat_start)
		return false;
	buf = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
	if (!buf)
		return false;
	if (!n31_ftl_read_disk_lba(ftl, L->fat_start, buf)) {
		u32 e0 = get_unaligned_le32(buf);
		u32 e1 = get_unaligned_le32(buf + 4);

		/*
		 * Entry 0 low byte is the media descriptor and matches the
		 * one in the BPB at offset 0x15; the rest of entry 0 and all
		 * of entry 1 are set. Mask
		 * to 28 bits -- FAT32 entries carry only the low 28.
		 */
		ok = (e0 & 0xff) == ftl->bpb_sector[0x15] &&
		     (e0 & 0x0fffff00) == 0x0fffff00 &&
		     (e1 & 0x0fffffff) == 0x0fffffff;
	}
	kfree(buf);
	return ok;
}

static void n31_fat_semantic_validate(struct n31_ftl_cs *ftl)
{
	struct n31_fat_layout *L = &ftl->layout;
	u8 *buf, *fat0 = NULL, *fat1 = NULL;
	u32 cluster, chain_len = 0, entries = 0, music_dirs = 0;
	u32 fat_diff = 0, i, max_chain = 64, max_fat_cmp = 4;
	bool itunesdb = false, apps = false, nanoapps = false;
	int ret;

	ftl->fat_sem_root_chain_len = 0;
	ftl->fat_sem_root_entries = 0;
	ftl->fat_sem_music_dirs = 0;
	ftl->fat_sem_fat0_fat1_diff = 0;
	ftl->fat_sem_itunesdb = false;
	ftl->fat_sem_apps = false;
	ftl->fat_sem_nanoapps = false;
	ftl->fat_sem_log[0] = '\0';

	if (!ftl->fat_base_valid || L->sectors_per_cluster == 0 ||
	    L->root_cluster < 2)
		return;

	buf = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
	if (!buf)
		return;

	/* Compare first few FAT0 vs FAT1 sectors (mirror check). */
	if (L->num_fats >= 2 && L->fat_size_32) {
		fat0 = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
		fat1 = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
		if (fat0 && fat1) {
			u32 n = min(max_fat_cmp, L->fat_size_32);

			for (i = 0; i < n; i++) {
				u32 d0 = L->fat_start + i;
				u32 d1 = L->fat_start + L->fat_size_32 + i;

				if (n31_ftl_read_disk_lba(ftl, d0, fat0) ||
				    n31_ftl_read_disk_lba(ftl, d1, fat1))
					break;
				if (memcmp(fat0, fat1, N31_DATA_SLOT_SIZE))
					fat_diff++;
			}
		}
		kfree(fat0);
		kfree(fat1);
	}

	cluster = L->root_cluster;
	while (cluster >= 2 && cluster < 0x0ffffff8 && chain_len < max_chain) {
		u32 disk_lba = L->data_start +
			(cluster - 2) * L->sectors_per_cluster;
		u32 s;

		for (s = 0; s < L->sectors_per_cluster; s++) {
			unsigned int off;

			ret = n31_ftl_read_disk_lba(ftl, disk_lba + s, buf);
			if (ret)
				goto done;
			for (off = 0; off + 32 <= N31_DATA_SLOT_SIZE; off += 32) {
				const u8 *ent = buf + off;
				char name[13];
				unsigned int n;

				if (ent[0] == 0x00)
					goto chain_done;
				if (ent[0] == 0xe5 || (ent[11] & 0x0f) == 0x0f)
					continue;
				entries++;
				for (n = 0; n < 11; n++)
					name[n] = ent[n] == ' ' ? '\0' : ent[n];
				name[11] = '\0';
				if (strnstr(name, "MUSIC", 11) ||
				    (ent[11] & 0x10)) {
					if (strnstr((const char *)ent, "MUSIC", 11) ||
					    strnstr(name, "F00", 11) ||
					    strnstr(name, "F01", 11) ||
					    strnstr(name, "F02", 11))
						music_dirs++;
				}
			}
			if (strnstr((const char *)buf, "iTunesDB",
				    N31_DATA_SLOT_SIZE) ||
			    strnstr((const char *)buf, "ITUNESDB",
				    N31_DATA_SLOT_SIZE))
				itunesdb = true;
			if (strnstr((const char *)buf, "iPod_Control",
				    N31_DATA_SLOT_SIZE) ||
			    strnstr((const char *)buf, "IPOD_CON",
				    N31_DATA_SLOT_SIZE))
				entries++; /* ensure root hit is counted */
			if (strnstr((const char *)buf, "NanoApps",
				    N31_DATA_SLOT_SIZE) ||
			    strnstr((const char *)buf, "NANOAPPS",
				    N31_DATA_SLOT_SIZE))
				nanoapps = true;
			if (strnstr((const char *)buf, "Apps",
				    N31_DATA_SLOT_SIZE))
				apps = true;
			if (strnstr((const char *)buf, "Music",
				    N31_DATA_SLOT_SIZE) ||
			    strnstr((const char *)buf, "MUSIC",
				    N31_DATA_SLOT_SIZE))
				music_dirs++;
			if (strnstr((const char *)buf, "F00",
				    N31_DATA_SLOT_SIZE) ||
			    strnstr((const char *)buf, "F01",
				    N31_DATA_SLOT_SIZE))
				music_dirs++;
		}

		/* Next cluster from active FAT (FAT0). */
		{
			u32 fat_off = cluster * 4;
			u32 fat_sec = L->fat_start + (fat_off / N31_DATA_SLOT_SIZE);
			u32 fat_ent_off = fat_off % N31_DATA_SLOT_SIZE;

			ret = n31_ftl_read_disk_lba(ftl, fat_sec, buf);
			if (ret)
				break;
			cluster = get_unaligned_le32(buf + fat_ent_off) &
				  0x0fffffffu;
		}
		chain_len++;
	}
chain_done:
done:
	ftl->fat_sem_root_chain_len = chain_len;
	ftl->fat_sem_root_entries = entries;
	ftl->fat_sem_music_dirs = music_dirs;
	ftl->fat_sem_fat0_fat1_diff = fat_diff;
	ftl->fat_sem_itunesdb = itunesdb;
	ftl->fat_sem_apps = apps;
	ftl->fat_sem_nanoapps = nanoapps;
	scnprintf(ftl->fat_sem_log, sizeof(ftl->fat_sem_log),
		  "fat_semantic fat_base=%u root_chain_len=%u root_entries=%u "
		  "music_dirs=%u itunesdb=%d apps=%d nanoapps=%d "
		  "fat0_fat1_diff=%u fat1_start_disk_lba=%u\n",
		  ftl->fat_base_lba, chain_len, entries, music_dirs,
		  itunesdb, apps, nanoapps, fat_diff,
		  L->fat_start + L->fat_size_32);
	dev_info(ftl->dev, "%s", ftl->fat_sem_log);
	kfree(buf);
}

/*
 * Validate BPB, FSInfo, FAT, and root-directory sectors before registering
 * the block device.
 */
static int n31_validate_fat_critical(struct n31_ftl_cs *ftl)
{
	u8 *buf;
	u32 total = 0;
	int ret, sess;
	struct n31_fat_layout *L;
	unsigned int ok = 0, need = 0;

	if (!ftl->fat_base_valid)
		return -ENODEV;

	buf = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY) {
		kfree(buf);
		return sess;
	}

	mutex_lock(&ftl->lock);
	ftl->fat_critical_ok = false;
	ftl->enable_gate_ok = false;
	ftl->disk0_ok = false;

	ret = n31_ftl_read_disk_lba(ftl, 0, buf);
	ftl->last_ret = ret;
	ftl->last_disk_lba = 0;
	ftl->last_fmss_lba = ftl->fat_base_lba;
	if (ret || !n31_bpb_looks_valid(buf, &total)) {
		scnprintf(ftl->last_log, sizeof(ftl->last_log),
			  "disk_lba0 BPB fail ret=%d\n", ret);
		goto out;
	}
	memcpy(ftl->last_sector, buf, N31_DATA_SLOT_SIZE);
	memcpy(ftl->bpb_sector, buf, N31_DATA_SLOT_SIZE);
	ftl->bpb_cached = true;
	ftl->disk0_ok = true;
	ftl->fat_total_sectors = total;
	n31_parse_bpb(buf, &ftl->layout);
	L = &ftl->layout;

	scnprintf(ftl->layout_log, sizeof(ftl->layout_log),
		  "bps=%u spc=%u reserved=%u fats=%u fat_size32=%u "
		  "root_cluster=%u fsinfo=%u backup=%u total=%u "
		  "ext_flags=0x%04x active_fat=%u mirror=%s\n"
		  "fat_start=%u fat1_start=%u data_start=%u root_dir_lba=%u "
		  "fsinfo_lba=%u backup_boot_lba=%u\n",
		  L->bytes_per_sector, L->sectors_per_cluster,
		  L->reserved_sectors, L->num_fats, L->fat_size_32,
		  L->root_cluster, L->fsinfo_sector, L->backup_boot_sector,
		  L->total_sectors, L->ext_flags, L->ext_flags & 0xF,
		  (L->ext_flags & 0x80) ? "off" : "on",
		  L->fat_start,
		  L->fat_start + L->fat_size_32,
		  L->data_start, L->root_dir_lba, L->fsinfo_lba,
		  L->backup_boot_lba);
	scnprintf(ftl->bpb_log, sizeof(ftl->bpb_log),
		  "oem='%.8s' jump=%02x%02x%02x %s",
		  buf + 3, buf[0], buf[1], buf[2], ftl->layout_log);

	need = 0;
	ok = 0;
#define CRIT(lba, tag) do { \
	need++; \
	if ((lba) < ftl->fat_total_sectors && \
	    !n31_read_disk_checked(ftl, (lba), buf, (tag))) \
		ok++; \
} while (0)

	CRIT(0, "BPB");
	if (L->fsinfo_lba)
		CRIT(L->fsinfo_lba, "FSInfo");
	if (L->backup_boot_lba && L->backup_boot_lba != L->fsinfo_lba)
		CRIT(L->backup_boot_lba, "backup_BPB");
	CRIT(L->fat_start, "FAT0");
	if (L->fat_start + 1 < ftl->fat_total_sectors)
		CRIT(L->fat_start + 1, "FAT1");
	CRIT(L->data_start, "root0");
	if (L->sectors_per_cluster >= 2)
		CRIT(L->data_start + 1, "root1");
	if (L->sectors_per_cluster >= 3)
		CRIT(L->data_start + 2, "root2");
	if (L->sectors_per_cluster >= 4)
		CRIT(L->data_start + 3, "root3");
#undef CRIT

	/* Whimory-backed maps must nearly pass; 5/9 must not register. */
	if (ftl->whimory_backed)
		ftl->fat_critical_ok = (ok >= 8 && ftl->disk0_ok);
	else
		ftl->fat_critical_ok = (ok >= 3 && ftl->disk0_ok &&
					n31_map_find(ftl, ftl->fat_base_lba));
	ftl->enable_gate_ok = ftl->fat_critical_ok;
	ftl->fat_crit_ok_n = ok;
	ftl->fat_crit_need_n = need;
	scnprintf(ftl->last_log, sizeof(ftl->last_log),
		  "fat_critical ok=%u/%u gate=%d fat_base=%u\n%s",
		  ok, need, ftl->enable_gate_ok, ftl->fat_base_lba,
		  ftl->layout_log);
	dev_info(ftl->dev, "%s", ftl->last_log);
	if (ftl->fat_critical_ok)
		n31_fat_semantic_validate(ftl);
	ret = ftl->fat_critical_ok ? 0 : -EAGAIN;
out:
	mutex_unlock(&ftl->lock);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	kfree(buf);
	return ret;
}

static int n31_read_disk_range(struct n31_ftl_cs *ftl, u32 start, u32 count)
{
	u8 *buf;
	u32 i;
	int ret, sess;
	u32 prev_phys = N31_PHYS_KEY_INVALID;

	if (!count)
		return -EINVAL;
	buf = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY) {
		kfree(buf);
		return sess;
	}

	mutex_lock(&ftl->lock);
	ftl->range_ok = 0;
	ftl->range_fail = 0;
	ftl->range_miss = 0;
	for (i = 0; i < count; i++) {
		u32 disk_lba = start + i;
		u32 fmss_lba;
		struct n31_map_entry e;

		if (!ftl->fat_base_valid) {
			ftl->range_fail++;
			break;
		}
		fmss_lba = ftl->fat_base_lba + disk_lba;
		if (n31_map_lookup_hint(ftl, fmss_lba, &e))
			ftl->range_miss++;
		ret = n31_ftl_read_disk_lba(ftl, disk_lba, buf);
		if (ret) {
			ftl->range_fail++;
			dev_dbg(ftl->dev,
				"range miss disk_lba=%u ret=%d\n",
				disk_lba, ret);
			continue;
		}
		ftl->range_ok++;
		if (!n31_map_lookup_hint(ftl, fmss_lba, &e)) {
			if (prev_phys != N31_PHYS_KEY_INVALID &&
			    e.phys_key != n31_phys_succ(prev_phys) &&
			    e.phys_key != prev_phys)
				dev_dbg(ftl->dev,
					"range gap disk_lba=%u phys=%08x prev=%08x\n",
					disk_lba, e.phys_key, prev_phys);
			prev_phys = e.phys_key;
		}
	}
	scnprintf(ftl->range_log, sizeof(ftl->range_log),
		  "range start=%u count=%u ok=%u fail=%u pre_miss=%u "
		  "demand_scans=%u\n",
		  start, count, ftl->range_ok, ftl->range_fail,
		  ftl->range_miss, ftl->demand_scans);
	scnprintf(ftl->last_log, sizeof(ftl->last_log), "%s", ftl->range_log);
	dev_dbg(ftl->dev, "%s", ftl->last_log);
	mutex_unlock(&ftl->lock);

	if (sess == 0)
		s5l8740_nand_dma_session_end();
	kfree(buf);
	return ftl->range_fail ? -EIO : 0;
}

/* -------------------- block device -------------------- */

static bool n31_looks_like_firmware(const u8 *d)
{
	u32 total;

	if (!d)
		return false;
	/* Classic Apple IMG1 / 8900 header */
	if (d[0] == '8' && d[1] == '9' && d[2] == '0' && d[3] == '0')
		return true;
	if (d[0] == 'I' && d[1] == 'm' && d[2] == 'g')
		return true;
	/* WinPod-style volume marker */
	if (!memcmp(d + 0x100, "[hi]", 4))
		return true;
	/* Nested FAT inside the pre-user range */
	if (n31_bpb_looks_valid(d, &total) && total > 0)
		return true;
	return false;
}

static void n31_firmware_probe(struct n31_ftl_cs *ftl)
{
	unsigned int bkt;
	struct n31_map_node *n;
	u32 min_l = ~0u, max_l = 0;
	unsigned int mapped = 0, magic = 0, sampled = 0;
	u8 *buf;

	ftl->fw_valid = false;
	ftl->fw_mapped = 0;
	ftl->fw_magic_hits = 0;
	ftl->fw_base_fmss = 0;
	ftl->fw_nsectors = 0;
	ftl->fw_log[0] = '\0';

	if (!ftl->fat_base_valid || !ftl->fat_base_lba)
		return;

	hash_for_each(ftl->map, bkt, n, hnode) {
		u32 l = n->e.fmss_lba;

		if (l >= ftl->fat_base_lba)
			continue;
		mapped++;
		if (l < min_l)
			min_l = l;
		if (l > max_l)
			max_l = l;
	}
	ftl->fw_mapped = mapped;

	buf = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
	if (buf) {
		hash_for_each(ftl->map, bkt, n, hnode) {
			u32 l = n->e.fmss_lba;

			if (l >= ftl->fat_base_lba)
				continue;
			if (sampled >= 24)
				break;
			sampled++;
			if (!n31_ftl_read_fmss_lba_flags(ftl, l, buf, false) &&
			    n31_looks_like_firmware(buf))
				magic++;
		}
		kfree(buf);
	}
	ftl->fw_magic_hits = magic;

	if (fw_start_lba >= 0)
		ftl->fw_base_fmss = (u32)fw_start_lba;
	else
		ftl->fw_base_fmss = 0;

	if (fw_nsectors > 0)
		ftl->fw_nsectors = (u32)fw_nsectors;
	else if (ftl->fat_base_lba > ftl->fw_base_fmss)
		ftl->fw_nsectors = ftl->fat_base_lba - ftl->fw_base_fmss;
	else
		ftl->fw_nsectors = 0;

	ftl->fw_valid = fw_force || magic > 0 || mapped >= 4;
	if (!ftl->fw_nsectors)
		ftl->fw_valid = false;

	scnprintf(ftl->fw_log, sizeof(ftl->fw_log),
		  "fw_valid=%d base=%u nsectors=%u mapped=%u magic=%u "
		  "min_lba=%u max_lba=%u force=%d\n",
		  ftl->fw_valid, ftl->fw_base_fmss, ftl->fw_nsectors,
		  mapped, magic,
		  mapped ? min_l : 0, mapped ? max_l : 0, fw_force);
	dev_info(ftl->dev, "%s", ftl->fw_log);
}

/*
 * VFAT directory bread failures land here as L2V misses. Print address-space
 * math + neighbor map presence so we can tell "not scanned yet" from corruption.
 */
static void n31_log_read_miss(struct n31_ftl_cs *ftl, struct n31_ftl_slice *sl,
			      u32 fmss_lba, u32 disk_lba, int ret)
{
	struct n31_fat_layout *L = &ftl->layout;
	u32 cluster = 0;
	int i;
	u8 ce = 0, cau = 0, page = 0, slot = 0;
	u16 blk = 0;
	u64 weave = 0;
	int phys_ret;

	ftl->read_miss_count++;
	if (read_miss_diag_max && ftl->read_miss_count > read_miss_diag_max)
		return;
	if (L->valid && L->sectors_per_cluster &&
	    disk_lba >= L->data_start) {
		cluster = ((disk_lba - L->data_start) /
			   L->sectors_per_cluster) + 2;
	}

	dev_err_ratelimited(ftl->dev,
		"read miss %s fmss_lba=%u disk_lba=%u ret=%d "
		"fat_base=%u data_start=%u spc=%u cluster~=%u miss_n=%u\n",
		sl->gd ? sl->gd->disk_name : "?",
		fmss_lba, disk_lba, ret,
		ftl->fat_base_lba, L->data_start, L->sectors_per_cluster,
		cluster, ftl->read_miss_count);

	for (i = -4; i <= 4; i++) {
		u32 n = fmss_lba + i;
		u8 nce = 0, ncau = 0, npg = 0, nslot = 0;
		u16 nblk = 0;
		u64 nw = 0;
		int nr;

		if ((int)fmss_lba + i < 0)
			continue;
		nr = whimory_l2v_search_phys(n, &nce, &ncau, &nblk, &npg,
					     &nslot, &nw);
		if (!nr)
			dev_err_ratelimited(ftl->dev,
				"  neighbor fmss_lba=%u MAPPED ce=%u cau=%u "
				"blk=%u pg=%u slot=%u weave=%012llx\n",
				n, nce, ncau, nblk, npg, nslot,
				(unsigned long long)nw);
		else if (i == 0)
			dev_err_ratelimited(ftl->dev,
				"  neighbor fmss_lba=%u UNMAPPED ret=%d\n",
				n, nr);
	}

	phys_ret = whimory_l2v_search_phys(fmss_lba, &ce, &cau, &blk, &page,
					   &slot, &weave);
	if (!phys_ret)
		dev_err_ratelimited(ftl->dev,
			"  L2V suddenly mapped after miss? ce=%u cau=%u blk=%u "
			"pg=%u slot=%u\n",
			ce, cau, blk, page, slot);
	(void)phys_ret;
}

static void n31_ftl_submit_bio(struct bio *bio)
{
	struct n31_ftl_slice *sl = bio->bi_bdev->bd_disk->private_data;
	struct n31_ftl_cs *ftl;
	struct bvec_iter iter;
	struct bio_vec bvec;
	sector_t sector = bio->bi_iter.bi_sector;
	int ret = 0;

	if (!sl || !sl->ftl || !sl->nsectors) {
		bio_io_error(bio);
		return;
	}
	ftl = sl->ftl;
	if (sl->kind != N31_SLICE_FIRMWARE && !ftl->enable_gate_ok) {
		bio_io_error(bio);
		return;
	}
	if (op_is_write(bio_op(bio)) || bio_op(bio) == REQ_OP_DISCARD ||
	    bio_op(bio) == REQ_OP_WRITE_ZEROES) {
		bio->bi_status = BLK_STS_IOERR;
		bio_endio(bio);
		return;
	}
	if ((sector & 7) != 0) {
		bio_io_error(bio);
		return;
	}

	bio_for_each_segment(bvec, bio, iter) {
		u8 *dst = kmap_local_page(bvec.bv_page) + bvec.bv_offset;
		unsigned int done = 0;

		while (done < bvec.bv_len) {
			u32 off = (u32)(sector >> 3);
			u32 fmss_lba;
			unsigned int n = min_t(unsigned int,
					       bvec.bv_len - done,
					       N31_DATA_SLOT_SIZE);

			if (off >= sl->nsectors) {
				ret = -ERANGE;
				kunmap_local(dst);
				goto done;
			}
			fmss_lba = sl->base_fmss + off;
			mutex_lock(&ftl->lock);
			ret = n31_ftl_read_fmss_lba_flags(ftl, fmss_lba,
							  ftl->bounce, false);
			if (!ret)
				memcpy(dst + done, ftl->bounce, n);
			else
				n31_log_read_miss(ftl, sl, fmss_lba, off, ret);
			mutex_unlock(&ftl->lock);
			if (ret) {
				kunmap_local(dst);
				goto done;
			}
			done += n;
			sector += n / 512;
		}
		kunmap_local(dst);
	}
done:
	if (ret)
		bio_io_error(bio);
	else
		bio_endio(bio);
}

static const struct block_device_operations n31_ftl_bd_ops = {
	.owner = THIS_MODULE,
	.submit_bio = n31_ftl_submit_bio,
};

static int n31_slice_register(struct n31_ftl_slice *sl, const char *name,
			      u32 base_fmss, u32 nsectors,
			      enum n31_slice_kind kind)
{
	struct queue_limits lim = {
		.logical_block_size = N31_DATA_SLOT_SIZE,
		.physical_block_size = N31_DATA_SLOT_SIZE,
		.io_min = N31_DATA_SLOT_SIZE,
	};
	struct gendisk *gd;
	int ret;

	if (!sl || !sl->ftl || !nsectors)
		return -EINVAL;
	if (sl->gd)
		return 0;

	gd = blk_alloc_disk(&lim, NUMA_NO_NODE);
	if (IS_ERR(gd))
		return PTR_ERR(gd);

	sl->base_fmss = base_fmss;
	sl->nsectors = nsectors;
	sl->kind = kind;
	sl->gd = gd;

	gd->first_minor = 0;
	gd->flags = GENHD_FL_NO_PART;
	gd->fops = &n31_ftl_bd_ops;
	gd->private_data = sl;
	snprintf(gd->disk_name, DISK_NAME_LEN, "%s", name);
	set_capacity(gd, (sector_t)nsectors * 8);
	set_disk_ro(gd, 1);
	ret = add_disk(gd);
	if (ret) {
		put_disk(gd);
		sl->gd = NULL;
		return ret;
	}
	dev_info(sl->ftl->dev,
		 "/dev/%s read-only, %u × 4096-byte sectors, base_fmss=%u\n",
		 name, nsectors, base_fmss);
	return 0;
}

static void n31_slice_unregister(struct n31_ftl_slice *sl)
{
	if (!sl || !sl->gd)
		return;
	del_gendisk(sl->gd);
	put_disk(sl->gd);
	sl->gd = NULL;
	sl->nsectors = 0;
}

static void n31_ftl_unregister_disk(struct n31_ftl_cs *ftl);

static int n31_ftl_register_disk(struct n31_ftl_cs *ftl)
{
	int ret;

	if (!ftl_block_enable && !ftl->block_enable)
		return -EPERM;
	if (!ftl->enable_gate_ok || !ftl->fat_critical_ok)
		return -EAGAIN;
	if (ftl->ipod.gd)
		return 0;

	if (!ftl->dma_session_held) {
		ret = s5l8740_nand_dma_session_begin();
		if (ret && ret != -EBUSY)
			return ret;
		ftl->dma_session_held = (ret == 0);
	}

	ftl->ipod.ftl = ftl;
	ftl->ftl_alias.ftl = ftl;
	ftl->firmware.ftl = ftl;

	n31_firmware_probe(ftl);

	ret = n31_slice_register(&ftl->ipod, N31_IPOD_DISK_NAME,
				 ftl->fat_base_lba, ftl->fat_total_sectors,
				 N31_SLICE_IPOD);
	if (ret)
		goto fail;

	ret = n31_slice_register(&ftl->ftl_alias, N31_FTL_DISK_NAME,
				 ftl->fat_base_lba, ftl->fat_total_sectors,
				 N31_SLICE_FTL_ALIAS);
	if (ret)
		goto fail;

	if (ftl->fw_valid) {
		ret = n31_slice_register(&ftl->firmware, N31_FW_DISK_NAME,
					 ftl->fw_base_fmss, ftl->fw_nsectors,
					 N31_SLICE_FIRMWARE);
		if (ret)
			dev_warn(ftl->dev,
				 "firmware disk register failed %d\n", ret);
	}
	return 0;

fail:
	n31_ftl_unregister_disk(ftl);
	return ret;
}

static void n31_ftl_unregister_disk(struct n31_ftl_cs *ftl)
{
	if (!ftl)
		return;
	n31_slice_unregister(&ftl->firmware);
	n31_slice_unregister(&ftl->ftl_alias);
	n31_slice_unregister(&ftl->ipod);
	if (ftl->dma_session_held) {
		s5l8740_nand_dma_session_end();
		ftl->dma_session_held = false;
	}
}

/* -------------------- sysfs -------------------- */

static ssize_t ftl_map_stats_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	return sysfs_emit(buf,
			  "built=%d entries=%u pages=%u valid_records=%u "
			  "lba_min=%u lba_max=%u extents=%u largest=%u "
			  "duplicates=%u newer=%u has_49279=%d "
			  "demand_scans=%u\n%s",
			  ftl->map_built, ftl->map_entries, ftl->map_pages,
			  ftl->map_data_recs,
			  ftl->lba_min == ~0u ? 0 : ftl->lba_min, ftl->lba_max,
			  ftl->extent_count, ftl->extent_largest,
			  ftl->map_collisions, ftl->newer_replacements,
			  n31_map_find(ftl, N31_FAT_BASE_DEFAULT) ? 1 : 0,
			  ftl->demand_scans, ftl->last_log);
}
static DEVICE_ATTR_RO(ftl_map_stats);

static ssize_t ftl_extents_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	unsigned int i, n = 0;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	n = sysfs_emit(buf, "%s", ftl->extents_log);
	for (i = 0; i < ftl->extent_count && n < PAGE_SIZE - 80; i++) {
		struct n31_lba_extent *ex = &ftl->extents[i];

		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "ex%u start_lba=%u len=%u phys=%08x type=%02x\n",
			       i, ex->start_lba, ex->len, ex->start_phys_key,
			       ex->type);
	}
	return n;
}
static DEVICE_ATTR_RO(ftl_extents);

/* echo "CE CAU BLK_LO BLK_HI" > ftl_scan_block_window */
static ssize_t ftl_scan_block_window_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	unsigned int ce, cau, lo, hi;
	int nf, sess, ret;

	if (!ftl)
		return -ENODEV;
	nf = sscanf(buf, "%u %u %u %u", &ce, &cau, &lo, &hi);
	if (nf < 4)
		return -EINVAL;
	if (hi < lo)
		return -EINVAL;
	if (hi - lo > 32)
		return -E2BIG; /* glass safety */

	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY)
		return sess;
	mutex_lock(&ftl->lock);
	ret = n31_scan_block_window(ftl, ce, cau, lo, hi, true, true);
	mutex_unlock(&ftl->lock);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_scan_block_window);

static ssize_t ftl_map_build_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	unsigned int stage = 1;
	int sess, ret;
	u16 lo, hi;

	if (!ftl)
		return -ENODEV;
	if (sscanf(buf, "%u", &stage) < 1)
		stage = 1;
	/* stage1 = blk63; stage2 = 62..66; all CE/CAU. stage3 refused. */
	if (stage >= 3)
		return -EPERM;
	lo = (stage <= 1) ? 63 : 62;
	hi = (stage <= 1) ? 63 : 66;

	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY)
		return sess;
	mutex_lock(&ftl->lock);
	ret = n31_scan_banks_window(ftl, lo, hi, true);
	mutex_unlock(&ftl->lock);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_map_build);

static ssize_t ftl_fat_base_lba_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	return sysfs_emit(buf,
			  "fat_base_lba=%u valid=%d total=%u disk0_ok=%d "
			  "fat_critical_ok=%d gate_ok=%d\n",
			  ftl->fat_base_lba, ftl->fat_base_valid,
			  ftl->fat_total_sectors, ftl->disk0_ok,
			  ftl->fat_critical_ok, ftl->enable_gate_ok);
}

static ssize_t ftl_fat_base_lba_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	u32 v;

	if (!ftl || kstrtou32(buf, 0, &v))
		return -EINVAL;
	mutex_lock(&ftl->lock);
	ftl->fat_base_lba = v;
	ftl->fat_base_valid = true;
	ftl->fat_base_autodetect = false;
	ftl->disk0_ok = false;
	ftl->fat_critical_ok = false;
	ftl->enable_gate_ok = false;
	mutex_unlock(&ftl->lock);
	return count;
}
static DEVICE_ATTR_RW(ftl_fat_base_lba);

static ssize_t ftl_bpb_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	return sysfs_emit(buf, "%s", ftl->bpb_log[0] ? ftl->bpb_log : "none\n");
}
static DEVICE_ATTR_RO(ftl_bpb);

static ssize_t ftl_layout_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	return sysfs_emit(buf, "%s",
			  ftl->layout_log[0] ? ftl->layout_log : "none\n");
}
static DEVICE_ATTR_RO(ftl_layout);

static ssize_t ftl_find_bpb_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	int ret;

	if (!ftl)
		return -ENODEV;
	ftl->fat_base_autodetect = true;
	ret = n31_ftl_find_bpb(ftl);
	if (!ret)
		ret = n31_ftl_select_bpb(ftl);
	if (!ret && ftl_block_enable) {
		ftl->block_enable = true;
		ret = n31_ftl_register_disk(ftl);
	}
	return (ret && ret != -ENOENT && ret != -EAGAIN) ? ret : count;
}
static DEVICE_ATTR_WO(ftl_find_bpb);

static ssize_t ftl_read_last_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	size_t n = N31_DATA_SLOT_SIZE;

	if (!ftl)
		return -ENODEV;
	if (n > PAGE_SIZE)
		n = PAGE_SIZE;
	memcpy(buf, ftl->last_sector, n);
	return n;
}
static DEVICE_ATTR_RO(ftl_read_last);

static ssize_t ftl_read_fmss_lba_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	u32 fmss_lba;
	int ret, sess;

	if (!ftl || kstrtou32(buf, 0, &fmss_lba))
		return -EINVAL;
	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY)
		return sess;
	mutex_lock(&ftl->lock);
	ret = n31_ftl_read_fmss_lba(ftl, fmss_lba, ftl->last_sector);
	ftl->last_ret = ret;
	ftl->last_fmss_lba = fmss_lba;
	ftl->last_disk_lba = ~0u;
	scnprintf(ftl->last_log, sizeof(ftl->last_log),
		  "ftl_read_fmss_lba=%u ret=%d first8=%*ph\n",
		  fmss_lba, ret, 8, ftl->last_sector);
	dev_dbg(ftl->dev, "%s", ftl->last_log);
	mutex_unlock(&ftl->lock);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_read_fmss_lba);

static ssize_t ftl_read_disk_lba_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	u32 disk_lba;
	int ret, sess;

	if (!ftl || kstrtou32(buf, 0, &disk_lba))
		return -EINVAL;
	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY)
		return sess;
	mutex_lock(&ftl->lock);
	ret = n31_ftl_read_disk_lba(ftl, disk_lba, ftl->last_sector);
	ftl->last_ret = ret;
	ftl->last_disk_lba = disk_lba;
	ftl->last_fmss_lba = ftl->fat_base_valid ?
			     ftl->fat_base_lba + disk_lba : 0;
	scnprintf(ftl->last_log, sizeof(ftl->last_log),
		  "ftl_read_disk_lba=%u fmss_lba=%u ret=%d first8=%*ph\n",
		  disk_lba, ftl->last_fmss_lba, ret, 8, ftl->last_sector);
	dev_dbg(ftl->dev, "%s", ftl->last_log);
	mutex_unlock(&ftl->lock);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_read_disk_lba);

/* echo "START COUNT" > ftl_read_disk_range */
static ssize_t ftl_read_disk_range_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	unsigned int start, n;
	int ret;

	if (!ftl)
		return -ENODEV;
	if (sscanf(buf, "%u %u", &start, &n) < 2)
		return -EINVAL;
	if (n > 512)
		return -E2BIG;
	ret = n31_read_disk_range(ftl, start, n);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_read_disk_range);

static ssize_t ftl_read_range_stats_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	return sysfs_emit(buf, "%s",
			  ftl->range_log[0] ? ftl->range_log : "none\n");
}
static DEVICE_ATTR_RO(ftl_read_range_stats);

static ssize_t ftl_enable_block_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	unsigned int v = 0;
	int ret;

	if (!ftl || kstrtouint(buf, 0, &v))
		return -EINVAL;
	if (!v) {
		n31_ftl_unregister_disk(ftl);
		ftl->block_enable = false;
		return count;
	}
	ret = n31_validate_fat_critical(ftl);
	if (ret)
		return ret;
	ftl->block_enable = true;
	ret = n31_ftl_register_disk(ftl);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_enable_block);

static ssize_t ftl_vec_stats_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	struct n31_vecmap *v;
	u32 p = N31_INVALID_P;
	int cross = -ENOENT;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	v = &ftl->vec;
	if (v->ready)
		cross = n31_vecmap_lookup(v, N31_FAT_BASE_DEFAULT, &p);
	return sysfs_emit(buf, "%scross_49279_ret=%d p=%u\n",
			  ftl->vec_log[0] ? ftl->vec_log : "ready=0\n",
			  cross, p);
}
static DEVICE_ATTR_RO(ftl_vec_stats);

/* echo 1 > ftl_vec_build — recompress from current sparse hash */
static ssize_t ftl_vec_build_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	int ret;

	if (!ftl)
		return -ENODEV;
	mutex_lock(&ftl->lock);
	ret = n31_vecmap_rebuild_from_hash(ftl);
	mutex_unlock(&ftl->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_vec_build);

static ssize_t ftl_firmware_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	return sysfs_emit(buf, "%s",
			  ftl->fw_log[0] ? ftl->fw_log : "fw_valid=0\n");
}
static DEVICE_ATTR_RO(ftl_firmware);

struct n31_bpb_walk_ctx {
	struct n31_ftl_cs *ftl;
	u8 *buf;
	unsigned int tried;
	unsigned int max_try;
};

static void n31_ftl_note_bpb_cand(struct n31_ftl_cs *ftl, u32 fmss_lba,
				  u64 weave, const u8 *buf, u32 total)
{
	unsigned int i;

	for (i = 0; i < ftl->bpb_ncand; i++) {
		if (ftl->bpb_candidates[i] != fmss_lba)
			continue;
		if (n31_weave_newer(weave, ftl->bpb_cand_weave[i])) {
			ftl->bpb_cand_weave[i] = weave;
			ftl->bpb_cand_total[i] = total;
			memcpy(ftl->bpb_cand_oem[i], buf + 3, 8);
			ftl->bpb_cand_oem[i][8] = '\0';
			memcpy(ftl->bpb_cand_sector[i], buf,
			       N31_DATA_SLOT_SIZE);
		}
		return;
	}
	if (ftl->bpb_ncand >= N31_BPB_CANDIDATES_MAX)
		return;
	i = ftl->bpb_ncand++;
	ftl->bpb_candidates[i] = fmss_lba;
	ftl->bpb_cand_weave[i] = weave;
	ftl->bpb_cand_total[i] = total;
	memcpy(ftl->bpb_cand_oem[i], buf + 3, 8);
	ftl->bpb_cand_oem[i][8] = '\0';
	memcpy(ftl->bpb_cand_sector[i], buf, N31_DATA_SLOT_SIZE);
}

static int n31_bpb_walk_fn(u32 start, u32 len, u32 vba, u64 weave, void *opaque)
{
	struct n31_bpb_walk_ctx *c = opaque;
	u32 try_lba[2];
	unsigned int ntry = 0, i;
	u32 total = 0;
	int ret;

	(void)vba;
	if (!c || !c->ftl || !c->buf || !len)
		return 0;
	if (c->tried >= c->max_try)
		return 1; /* stop walk */

	try_lba[ntry++] = start;
	if (start < N31_FAT_BASE_DEFAULT &&
	    start + len > N31_FAT_BASE_DEFAULT)
		try_lba[ntry++] = N31_FAT_BASE_DEFAULT;

	for (i = 0; i < ntry; i++) {
		if (c->tried >= c->max_try)
			break;
		c->tried++;
		ret = whimory_read_fmss_lba(try_lba[i], c->buf);
		if (ret)
			continue;
		if (!n31_bpb_looks_valid(c->buf, &total))
			continue;
		mutex_lock(&c->ftl->lock);
		n31_ftl_note_bpb_cand(c->ftl, try_lba[i], weave, c->buf,
				      total);
		mutex_unlock(&c->ftl->lock);
	}
	return 0;
}

bool n31_ftl_cs_whimory_backed(void)
{
	return n31_ftl && n31_ftl->whimory_backed;
}

/* True once /dev/s5l8740-ipod is live; a rebuild under it is destructive. */
bool n31_ftl_cs_disk_registered(void)
{
	return n31_ftl && n31_ftl->ipod.gd;
}
EXPORT_SYMBOL_GPL(n31_ftl_cs_disk_registered);

int n31_ftl_cs_bind_whimory(void)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	struct n31_bpb_walk_ctx ctx;
	u8 *buf;
	u8 ce, cau, page, slot;
	u16 blk;
	u64 weave = 0;
	u32 total = 0;
	int ret, sess;
	static const u32 probes[] = {
		N31_FAT_BASE_DEFAULT, 49279u, 49285u, 0u
	};
	unsigned int i;

	if (!ftl)
		return -ENODEV;
	if (!whimory_l2v_ready())
		return -ENODEV;

	buf = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	sess = s5l8740_nand_dma_session_begin();
	mutex_lock(&ftl->lock);
	ftl->whimory_backed = true;
	ftl->map_built = true;
	ftl->bpb_ncand = 0;
	ftl->fat_base_valid = false;
	mutex_unlock(&ftl->lock);

	/* Prefer known BPB LBA probes via L2V_Search. */
	for (i = 0; i < ARRAY_SIZE(probes); i++) {
		ret = whimory_read_fmss_lba(probes[i], buf);
		if (ret)
			continue;
		if (!n31_bpb_looks_valid(buf, &total))
			continue;
		weave = 0;
		whimory_l2v_search_phys(probes[i], &ce, &cau, &blk, &page,
					&slot, &weave);
		mutex_lock(&ftl->lock);
		n31_ftl_note_bpb_cand(ftl, probes[i], weave, buf, total);
		mutex_unlock(&ftl->lock);
	}

	ctx.ftl = ftl;
	ctx.buf = buf;
	ctx.tried = 0;
	ctx.max_try = 512;
	whimory_range_walk(n31_bpb_walk_fn, &ctx);

	mutex_lock(&ftl->lock);
	if (!ftl->bpb_ncand) {
		dev_err(ftl->dev,
			"whimory bind: no BPB found in L2V ranges\n");
		ftl->whimory_backed = false;
		mutex_unlock(&ftl->lock);
		ret = -ENOENT;
		goto out_sess;
	}
	/* Snapshot ncand; select_bpb/validate take the lock themselves. */
	mutex_unlock(&ftl->lock);

	ret = n31_ftl_select_bpb(ftl);
	if (ret) {
		mutex_lock(&ftl->lock);
		ftl->whimory_backed = false;
		mutex_unlock(&ftl->lock);
		goto out_sess;
	}

	/* Self-check: Search(fat_base) must resolve. */
	ret = whimory_l2v_search_phys(ftl->fat_base_lba, &ce, &cau, &blk,
				      &page, &slot, &weave);
	dev_info(ftl->dev,
		 "whimory_bind fat_base=%u search=%d phys=%u/%u/%u/%u/%u "
		 "weave=%012llx cand=%u\n",
		 ftl->fat_base_lba, ret, ce, cau, blk, page, slot,
		 (unsigned long long)weave, ftl->bpb_ncand);
	if (ret) {
		mutex_lock(&ftl->lock);
		ftl->whimory_backed = false;
		mutex_unlock(&ftl->lock);
		goto out_sess;
	}

	if (ftl->enable_gate_ok && ftl_block_enable) {
		mutex_lock(&ftl->lock);
		n31_ftl_unregister_disk(ftl);
		ret = n31_ftl_register_disk(ftl);
		mutex_unlock(&ftl->lock);
	} else {
		ret = ftl->enable_gate_ok ? 0 : -EAGAIN;
	}

	scnprintf(ftl->last_log, sizeof(ftl->last_log),
		  "whimory_bind ok backed=%d fat_base=%u gate=%d disk=%d\n",
		  ftl->whimory_backed, ftl->fat_base_lba, ftl->enable_gate_ok,
		  ret);
	dev_info(ftl->dev, "%s", ftl->last_log);
out_sess:
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	kfree(buf);
	return ret;
}

/* echo 1 > ftl_sftl_recover — CXT→BTOC→L2V on CS META, then bind disks */
static ssize_t ftl_sftl_recover_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned int v = 1;
	int ret;

	(void)dev;
	(void)attr;
	if (sscanf(buf, "%u", &v) >= 1 && !v)
		return count;

	ret = whimory_sftl_recover_cs();
	if (ret)
		return ret;
	/*
	 * Re-binding a disk that is already registered clears the BPB
	 * candidates and leaves the gendisk at capacity 0, so a live mount
	 * starts failing every read. Recovery that was a no-op must not
	 * disturb the disk it just declined to rebuild.
	 */
	if (n31_ftl_cs_disk_registered())
		return count;
	ret = n31_ftl_cs_bind_whimory();
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_sftl_recover);

/* echo <vbas> > ftl_cxt_dump — report CXT bases/tags; never touches L2V */
static ssize_t ftl_cxt_dump_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int v = 0;
	int ret;

	(void)dev;
	(void)attr;
	if (sscanf(buf, "%u", &v) < 1)
		v = 0;
	ret = whimory_cxt_dump(v);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_cxt_dump);

/*
 * echo [fat_base] > ftl_cxt_candidate — build the CXT TREE map into a
 * separate candidate array, diff it against the live brute-force map, and
 * validate the FAT-critical sectors through it. Never mutates L2V.
 */
static ssize_t ftl_cxt_candidate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	unsigned int v = 0;
	int ret;

	(void)dev;
	(void)attr;
	if (sscanf(buf, "%u", &v) < 1 || !v)
		v = ftl ? ftl->fat_base_lba : 0;
	ret = whimory_cxt_candidate(v);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(ftl_cxt_candidate);

static ssize_t ftl_finishline_status_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	return sysfs_emit(buf,
			  "map_built=%d whimory_backed=%d entries=%u extents=%u "
			  "vec_ready=%d fat_base_lba=%u valid=%d disk0_ok=%d "
			  "fat_critical_ok=%d gate_ok=%d "
			  "ipod=%s ftl_alias=%s firmware=%s\n"
			  "last_ret=%d last_fmss=%u last_disk=%u\n%s%s%s",
			  ftl->map_built, ftl->whimory_backed ? 1 : 0,
			  ftl->map_entries, ftl->extent_count,
			  ftl->vec.ready ? 1 : 0,
			  ftl->fat_base_lba, ftl->fat_base_valid, ftl->disk0_ok,
			  ftl->fat_critical_ok, ftl->enable_gate_ok,
			  ftl->ipod.gd ? ftl->ipod.gd->disk_name : "(none)",
			  ftl->ftl_alias.gd ? ftl->ftl_alias.gd->disk_name :
					      "(none)",
			  ftl->firmware.gd ? ftl->firmware.gd->disk_name :
					     "(none)",
			  ftl->last_ret, ftl->last_fmss_lba, ftl->last_disk_lba,
			  ftl->last_log,
			  ftl->vec_log,
			  ftl->fw_log);
}
static DEVICE_ATTR_RO(ftl_finishline_status);

static ssize_t ftl_fat_semantic_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	return sysfs_emit(buf, "%s",
			  ftl->fat_sem_log[0] ? ftl->fat_sem_log :
						"none\n");
}
static DEVICE_ATTR_RO(ftl_fat_semantic);

static ssize_t ftl_phys_string_scan_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	unsigned int blocks = 0;
	int hits;

	if (kstrtouint(buf, 0, &blocks))
		blocks = 0;
	hits = whimory_phys_string_scan(blocks);
	if (n31_ftl)
		scnprintf(n31_ftl->string_scan_log,
			  sizeof(n31_ftl->string_scan_log),
			  "phys_string_scan blocks=%u hits=%d\n", blocks, hits);
	return hits < 0 ? hits : count;
}
static DEVICE_ATTR_WO(ftl_phys_string_scan);

static ssize_t ftl_logical_string_scan_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct n31_ftl_cs *ftl = n31_ftl;
	u8 *sec;
	u32 i, nsectors = 4096, hits = 0;
	int ret, sess;
	static const char *const needles[] = {
		"iTunesDB", "F00", "F01", "F02", "iPod_Control", "Music",
		"Apps", "NanoApps", ".mp3", ".m4a",
	};

	if (!ftl || !ftl->fat_base_valid)
		return -ENODEV;
	if (kstrtouint(buf, 0, &nsectors))
		nsectors = 4096;
	if (nsectors > ftl->fat_total_sectors)
		nsectors = ftl->fat_total_sectors;
	sec = kmalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
	if (!sec)
		return -ENOMEM;
	sess = s5l8740_nand_dma_session_begin();
	dev_info(ftl->dev,
		 "LOGICAL_STRING_SCAN start disk_lbas=0..%u via L2V\n",
		 nsectors);
	for (i = 0; i < nsectors; i++) {
		unsigned int ni;

		ret = n31_ftl_read_disk_lba(ftl, i, sec);
		if (ret)
			continue;
		for (ni = 0; ni < ARRAY_SIZE(needles); ni++) {
			if (!strnstr((const char *)sec, needles[ni],
				     N31_DATA_SLOT_SIZE))
				continue;
			hits++;
			if (hits <= 64)
				dev_info(ftl->dev,
					 "LOGICAL_STRING hit=%s disk_lba=%u "
					 "fmss_lba=%u\n",
					 needles[ni], i,
					 ftl->fat_base_lba + i);
			break;
		}
		if ((i & 0xff) == 0)
			cond_resched();
	}
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	scnprintf(ftl->string_scan_log, sizeof(ftl->string_scan_log),
		  "logical_string_scan disk_lbas=%u hits=%u\n", nsectors, hits);
	dev_info(ftl->dev, "%s", ftl->string_scan_log);
	kfree(sec);
	return count;
}
static DEVICE_ATTR_WO(ftl_logical_string_scan);

static ssize_t ftl_string_scan_log_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return sysfs_emit(buf, "no ftl\n");
	return sysfs_emit(buf, "%s",
			  ftl->string_scan_log[0] ? ftl->string_scan_log :
						    "none\n");
}
static DEVICE_ATTR_RO(ftl_string_scan_log);

static struct attribute *n31_ftl_finish_attrs[] = {
	&dev_attr_ftl_sftl_recover.attr,
	&dev_attr_ftl_cxt_dump.attr,
	&dev_attr_ftl_cxt_candidate.attr,
	&dev_attr_ftl_map_build.attr,
	&dev_attr_ftl_scan_block_window.attr,
	&dev_attr_ftl_map_stats.attr,
	&dev_attr_ftl_extents.attr,
	&dev_attr_ftl_vec_stats.attr,
	&dev_attr_ftl_vec_build.attr,
	&dev_attr_ftl_firmware.attr,
	&dev_attr_ftl_find_bpb.attr,
	&dev_attr_ftl_fat_base_lba.attr,
	&dev_attr_ftl_bpb.attr,
	&dev_attr_ftl_layout.attr,
	&dev_attr_ftl_fat_semantic.attr,
	&dev_attr_ftl_phys_string_scan.attr,
	&dev_attr_ftl_logical_string_scan.attr,
	&dev_attr_ftl_string_scan_log.attr,
	&dev_attr_ftl_read_fmss_lba.attr,
	&dev_attr_ftl_read_disk_lba.attr,
	&dev_attr_ftl_read_disk_range.attr,
	&dev_attr_ftl_read_range_stats.attr,
	&dev_attr_ftl_read_last.attr,
	&dev_attr_ftl_enable_block.attr,
	&dev_attr_ftl_finishline_status.attr,
	NULL,
};

static const struct attribute_group n31_ftl_finish_group = {
	.attrs = n31_ftl_finish_attrs,
};

int ftl_s5l8740_csmap_init(struct device *dev)
{
	struct n31_ftl_cs *ftl;
	int ret;

	if (!dev)
		return -EINVAL;
	if (n31_ftl)
		return -EBUSY;

	ftl = kzalloc(sizeof(*ftl), GFP_KERNEL);
	if (!ftl)
		return -ENOMEM;
	ftl->bounce = kzalloc(N31_DATA_SLOT_SIZE, GFP_KERNEL);
	if (!ftl->bounce) {
		kfree(ftl);
		return -ENOMEM;
	}
	ftl->dev = dev;
	mutex_init(&ftl->lock);
	hash_init(ftl->map);
	ftl->lba_min = ~0u;
	ftl->fat_base_lba = N31_FAT_BASE_DEFAULT;
	ftl->fat_total_sectors = N31_FAT_TOTAL_DEFAULT;
	ftl->fat_base_autodetect = true;

	ret = sysfs_create_group(&dev->kobj, &n31_ftl_finish_group);
	if (ret) {
		kfree(ftl->bounce);
		kfree(ftl);
		return ret;
	}
	n31_ftl = ftl;
	dev_info(dev,
		 "CS map ready (scan → find_bpb → /dev/%s + /dev/%s)\n",
		 N31_IPOD_DISK_NAME, N31_FW_DISK_NAME);
	return 0;
}

void ftl_s5l8740_csmap_exit(struct device *dev)
{
	struct n31_ftl_cs *ftl = n31_ftl;

	if (!ftl)
		return;
	n31_ftl_unregister_disk(ftl);
	if (dev)
		sysfs_remove_group(&dev->kobj, &n31_ftl_finish_group);
	mutex_lock(&ftl->lock);
	n31_map_free(ftl);
	mutex_unlock(&ftl->lock);
	kfree(ftl->bounce);
	kfree(ftl);
	n31_ftl = NULL;
}
