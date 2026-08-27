// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 Whimory FTL — read-only block path (N31).
 *
 * Layers: NAND FIL → FPart → VFL → FTL/L2V → optional /dev/s5l8740-ftl.
 * The CS-map front-end in ftl-s5l8740-csmap.c is the preferred RO disk path.
 * This module retains the classic Whimory open/boot helpers.
 *
 * The block device is registered only after FAT-critical validation succeeds.
 * Empty or inconsistent maps never expose a disk.
 */
#include <linux/bio.h>
#include <linux/bitops.h>
#include <linux/blkdev.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/stdarg.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "whimory-s5l8740.h"
#include "ftl-s5l8740-csmap.h"

#define FTL_DISK_NAME		"s5l8740-ftl"
#define FTL_IPOD_NAME		"s5l8740-ipod"

#define WHIMORY_ORACLE_SIG	"apple/n31-whimory-sig.bin"
#define WHIMORY_ORACLE_ROOT	"apple/n31-whimory-l2v-root.bin"
#define WHIMORY_ORACLE_NODES	"apple/n31-whimory-l2v-nodes.bin"
#define WHIMORY_ORACLE_GLOBALS	"apple/n31-whimory-l2v-globals.bin"

#define WHIMORY_SPECIAL_LBA	0xFFFF0000u

static bool import_l2v_oracle;
module_param(import_l2v_oracle, bool, 0644);
MODULE_PARM_DESC(import_l2v_oracle,
		 "Load L2V root/nodes/globals from /lib/firmware/apple/");

static unsigned int max_open_sbs = 16;
module_param(max_open_sbs, uint, 0644);
MODULE_PARM_DESC(max_open_sbs,
		 "Max open superblocks to META-rebuild (0 = all; default 16)");

static unsigned int scan_blocks = 256;
module_param(scan_blocks, uint, 0644);
MODULE_PARM_DESC(scan_blocks,
		 "User blocks per CE/CAU to classify (0 = all; default 256)");

static unsigned int meta0_scan_sbs = 4;
module_param(meta0_scan_sbs, uint, 0644);
MODULE_PARM_DESC(meta0_scan_sbs,
		 "Closed SBs to full-scan for META lba=0 after classify (0=skip extra)");

static bool allow_sigless_debug;
module_param(allow_sigless_debug, bool, 0644);
MODULE_PARM_DESC(allow_sigless_debug,
		 "If true, classify/recover without xrmw (default N — NAND wedge / fake META)");

static unsigned int sig_scan_blocks;
module_param(sig_scan_blocks, uint, 0644);
MODULE_PARM_DESC(sig_scan_blocks,
		 "FPart assignment scan: tail blocks (0 = vfl_tail)");

static unsigned int fpart_assign_pages = 1;
module_param(fpart_assign_pages, uint, 0644);
MODULE_PARM_DESC(fpart_assign_pages,
		 "Pages per tail block to scan for META 0x30 assignment (default 1 = page0)");

static bool sig_brute_scan;
module_param(sig_brute_scan, bool, 0644);
MODULE_PARM_DESC(sig_brute_scan,
		 "META 0x30 page0 brute (default N — PIO spare is not Sogeti)");

static bool payload_magic_scan;
module_param(payload_magic_scan, bool, 0644);
MODULE_PARM_DESC(payload_magic_scan,
		 "Debug-only data xrmw/wrmx hunt (default N — FPart uses META 0x30)");

/* Kept so existing insmod lines do not fail. Recovery always runs at probe. */
static bool ftl_auto_map __maybe_unused;
module_param(ftl_auto_map, bool, 0644);
MODULE_PARM_DESC(ftl_auto_map, "ignored; Whimory always recovers at insmod");

static bool fpart_auto_scan __maybe_unused;
module_param(fpart_auto_scan, bool, 0644);
MODULE_PARM_DESC(fpart_auto_scan, "ignored; host slices created after LBA0");

static struct whimory *whimory_dev;
static struct platform_device *ftl_pdev;

static void whimory_l2v_find_frag(struct whimory *w);
static void whimory_l2v_free_tree(struct whimory *w, u32 node_idx, u32 root_idx);
static int whimory_l2v_update_packed(struct whimory *w, u32 ridx, u32 off,
				     u32 span, u32 vba);
static int n31_sftl_read_lba(struct whimory *w, u32 lba, void *buf,
			     bool allow_blank);

static u64 whimory_weave48(const u8 *m)
{
	return (u64)get_unaligned_le16(m + 2) |
	       ((u64)get_unaligned_le32(m + 4) << 16);
}

/*
 * CS span4/rec4112 page read — real 4× META (glass-proven). Used for
 * classify / BTOC / open-SB / CXT / VBA reads when meta_dma_read=0.
 */
static int whimory_cs_read_page(struct whimory *w, unsigned int ce,
				unsigned int cau, unsigned int block,
				unsigned int page, void *data, size_t data_len,
				void *meta, size_t meta_len)
{
	struct s5l8740_cs_page *csp;
	unsigned int s;
	int ret;

	if (!w || !data || data_len < S5L8740_NAND_PAGE_SIZE)
		return -EINVAL;
	csp = w->sftl.cs_page;
	if (!csp)
		return -ENOMEM;

	ret = s5l8740_nand_cs_phys_read((u8)ce, (u8)cau, (u16)block, (u8)page,
					csp);
	if (ret)
		return ret;

	for (s = 0; s < S5L8740_NAND_SLOTS_PER_PAGE; s++)
		memcpy((u8 *)data + s * S5L8740_NAND_SLOT_DATA,
		       csp->data[s], S5L8740_NAND_SLOT_DATA);

	if (meta && meta_len) {
		size_t copy = min_t(size_t, meta_len, S5L8740_NAND_META_SIZE);

		memset(meta, 0xff, meta_len);
		for (s = 0; s < S5L8740_NAND_SLOTS_PER_PAGE &&
			     (s + 1) * WHIMORY_META_SIZE <= copy; s++)
			memcpy((u8 *)meta + s * WHIMORY_META_SIZE,
			       csp->meta_raw[s], WHIMORY_META_SIZE);
	}
	return 0;
}

static bool whimory_meta_any_btoc(const u8 *meta64)
{
	unsigned int s;

	if (!meta64)
		return false;
	for (s = 0; s < WHIMORY_VBAS_PER_PAGE; s++) {
		if (meta64[s * WHIMORY_META_SIZE] == WHIMORY_META_TYPE_BTOC)
			return true;
	}
	return false;
}

static bool whimory_meta_slot0_or_any_cxt(const u8 *meta64)
{
	unsigned int s;

	if (!meta64)
		return false;
	if (meta64[0] == WHIMORY_META_TYPE_SFTL_CXT)
		return true;
	for (s = 1; s < WHIMORY_VBAS_PER_PAGE; s++) {
		if (meta64[s * WHIMORY_META_SIZE] == WHIMORY_META_TYPE_SFTL_CXT)
			return true;
	}
	return false;
}

static bool whimory_page_blank(const u8 *p, unsigned int n)
{
	unsigned int i;
	u8 all_ff = 0xff, all_00 = 0;

	if (!p || !n)
		return true;
	for (i = 0; i < n; i++) {
		all_ff &= p[i];
		all_00 |= p[i];
	}
	return all_ff == 0xff || all_00 == 0;
}

static bool whimory_meta_erased(const u8 *m, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (m[i] != 0xff)
			return false;
	}
	return true;
}

static bool whimory_meta_is_user_data(const struct whimory_meta *m)
{
	return m->type == WHIMORY_META_TYPE_DATA ||
	       m->type == WHIMORY_META_TYPE_DATA2;
}

static bool whimory_meta_is_cxt_base(const u8 *m, u32 vba_ofs)
{
	return m[0] == WHIMORY_META_TYPE_SFTL_CXT &&
	       m[1] == WHIMORY_CXT_TAG_BASE &&
	       vba_ofs == 0;
}

static bool whimory_meta_is_btoc(const u8 *m)
{
	return m[0] == WHIMORY_META_TYPE_BTOC;
}

static bool whimory_meta_is_data_raw(const u8 *m)
{
	return m[0] == WHIMORY_META_TYPE_DATA ||
	       m[0] == WHIMORY_META_TYPE_DATA2;
}

static bool whimory_special_lba(u32 lba)
{
	return (lba & 0xFFFF0000u) == WHIMORY_SPECIAL_LBA ||
	       lba == WHIMORY_LBA_BLANK || lba == WHIMORY_LBA_DELETED;
}

static u32 whimory_vfl_phys(struct whimory *w, u32 cau, u32 virt)
{
	/*
 *: PBN = VBN (identity over blocks_per_cau).
 * The u16 table at CXT +0x200 is a VFL CXT copy journal
 * : value = index | (gen<<15), 0xC070 = free), not
 * virt→phys. Failed user blocks keep the same VBN and switch
 * CAU via the bank bitmap.
 */
	if (cau >= w->geom.num_cau || !w->vfl.remap[cau])
		return virt;
	if (virt >= w->geom.blocks_per_cau)
		return virt;
	return w->vfl.remap[cau][virt];
}

/*: banks that participate in this VBN. */
static u32 whimory_vfl_banks_in_vbn(struct whimory *w, u32 vbn, u8 *out,
				    u32 out_max)
{
	u8 mask;
	u32 n = 0, b;

	if (w->vfl.cached_vbn == (u16)vbn && w->vfl.cached_n) {
		n = min_t(u32, w->vfl.cached_n, out_max);
		if (out)
			memcpy(out, w->vfl.cached_banks, n);
		return w->vfl.cached_n;
	}
	mask = 0;
	if (w->vfl.bank_mask && vbn < w->geom.blocks_per_cau) {
		u32 stride = w->vfl.bank_stride ? w->vfl.bank_stride : 1;
		const u8 *row = w->vfl.bank_mask + stride * vbn;
		u32 b;

		for (b = 0; b < w->geom.num_cau && b < 8; b++) {
			u32 bi = b >> 3;

			if (bi < stride && (row[bi] & (1u << (b & 7))))
				mask |= (u8)(1u << b);
		}
	}
	if (!mask)
		mask = (1u << w->geom.num_cau) - 1;
	for (b = 0; b < w->geom.num_cau && b < 8; b++) {
		if (!(mask & (1u << b)))
			continue;
		if (out && n < out_max)
			out[n] = (u8)b;
		if (n < S5L8740_NAND_MAX_CAU)
			w->vfl.cached_banks[n] = (u8)b;
		n++;
	}
	w->vfl.cached_vbn = (u16)vbn;
	w->vfl.cached_n = (u8)n;
	return n;
}

static u32 whimory_vfl_bank(struct whimory *w, u32 cau, u32 vblock)
{
	u8 banks[S5L8740_NAND_MAX_CAU];
	u32 n, i;

	n = whimory_vfl_banks_in_vbn(w, vblock, banks, ARRAY_SIZE(banks));
	if (!n)
		return cau;
	for (i = 0; i < n; i++) {
		if (banks[i] == (u8)cau)
			return cau;
	}
	return banks[0];
}

static u32 whimory_vfl_virt(struct whimory *w, u32 cau, u32 phys)
{
	u32 i, n;

	if (cau >= w->geom.num_cau || !w->vfl.remap[cau])
		return phys;
	n = w->geom.blocks_per_cau;
	for (i = 0; i < n; i++) {
		if (w->vfl.remap[cau][i] == phys)
			return i;
	}
	return phys;
}

static u32 s_g_addr_to_vba(const struct whimory *w, u32 sb, u32 ofs)
{
	return sb * w->sftl.vbas_per_sb + ofs;
}

static u32 s_g_vba_to_sb(const struct whimory *w, u32 vba)
{
	if (!w->sftl.vbas_per_sb)
		return 0;
	return vba / w->sftl.vbas_per_sb;
}

static u32 s_g_vba_to_ofs(const struct whimory *w, u32 vba)
{
	if (!w->sftl.vbas_per_sb)
		return 0;
	return vba % w->sftl.vbas_per_sb;
}

static u32 whimory_sb_index(const struct whimory *w, u32 ce, u32 cau,
			    u32 vblock)
{
	return (ce * w->geom.num_cau + cau) * w->sftl.user_blocks + vblock;
}

static u32 whimory_pack_vba(const struct whimory *w, u32 ce, u32 cau,
			    u32 vblock, u32 page, u32 slot)
{
	u32 sb = whimory_sb_index(w, ce, cau, vblock);
	u32 ofs = page * w->sftl.vbas_per_page + slot;

	return s_g_addr_to_vba(w, sb, ofs);
}

static int whimory_unpack_vba(const struct whimory *w, u32 vba,
			      u32 *ce, u32 *cau, u32 *vblock,
			      u32 *page, u32 *slot)
{
	u32 sb, ofs, per_ce;

	if (!w->sftl.vbas_per_sb || !w->sftl.vbas_per_page ||
	    !w->sftl.user_blocks)
		return -EINVAL;
	sb = s_g_vba_to_sb(w, vba);
	ofs = s_g_vba_to_ofs(w, vba);
	*page = ofs / w->sftl.vbas_per_page;
	*slot = ofs % w->sftl.vbas_per_page;
	per_ce = w->geom.num_cau * w->sftl.user_blocks;
	if (!per_ce)
		return -EINVAL;
	*ce = sb / per_ce;
	sb %= per_ce;
	*cau = sb / w->sftl.user_blocks;
	*vblock = sb % w->sftl.user_blocks;
	if (*ce >= w->geom.num_ce || *cau >= w->geom.num_cau)
		return -ERANGE;
	if (*page >= w->sftl.pages_per_sb)
		return -ERANGE;
	return 0;
}

static void whimory_set_status(struct whimory *w, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(w->status, sizeof(w->status), fmt, ap);
	va_end(ap);
}

/* ------------------------------------------------------------------ */
/* Interval map: weave-order LBA→VBA, then packed into the L2V tree. */
/* ------------------------------------------------------------------ */

static struct whimory_range *whimory_range_find(struct rb_root *root, u32 lba)
{
	struct rb_node *n = root->rb_node;

	while (n) {
		struct whimory_range *r = rb_entry(n, struct whimory_range, rb);

		if (lba < r->start)
			n = n->rb_left;
		else if (lba >= r->start + r->len)
			n = n->rb_right;
		else
			return r;
	}
	return NULL;
}

static int whimory_range_link(struct rb_root *root, struct whimory_range *n)
{
	struct rb_node **link = &root->rb_node, *parent = NULL;

	while (*link) {
		struct whimory_range *r = rb_entry(*link, struct whimory_range,
						   rb);

		parent = *link;
		if (n->start < r->start)
			link = &(*link)->rb_left;
		else
			link = &(*link)->rb_right;
	}
	rb_link_node(&n->rb, parent, link);
	rb_insert_color(&n->rb, root);
	return 0;
}

static int whimory_range_split(struct whimory *w, struct whimory_range *r,
			       u32 at)
{
	struct whimory_range *right;
	u32 left_len;

	if (at <= r->start || at >= r->start + r->len)
		return 0;
	right = kzalloc(sizeof(*right), GFP_KERNEL);
	if (!right)
		return -ENOMEM;
	left_len = at - r->start;
	right->start = at;
	right->len = r->len - left_len;
	right->vba = r->vba + left_len;
	right->weave = r->weave;
	r->len = left_len;
	whimory_range_link(&w->ranges, right);
	w->sftl.range_nodes++;
	return 0;
}

static void whimory_range_erase(struct whimory *w, struct whimory_range *r)
{
	rb_erase(&r->rb, &w->ranges);
	kfree(r);
	if (w->sftl.range_nodes)
		w->sftl.range_nodes--;
}

static int whimory_range_insert_new(struct whimory *w, u32 start, u32 len,
				    u32 vba)
{
	struct whimory_range *n;

	if (!len)
		return 0;
	n = kzalloc(sizeof(*n), GFP_KERNEL);
	if (!n)
		return -ENOMEM;
	n->start = start;
	n->len = len;
	n->vba = vba;
	n->weave = w->sftl.claim_weave;
	whimory_range_link(&w->ranges, n);
	w->sftl.range_nodes++;
	return 0;
}

static void whimory_range_coalesce_at(struct whimory *w, u32 start)
{
	struct whimory_range *r, *prev, *next;
	struct rb_node *p, *q;

	r = whimory_range_find(&w->ranges, start);
	if (!r)
		return;
	p = rb_prev(&r->rb);
	if (p) {
		prev = rb_entry(p, struct whimory_range, rb);
		if (prev->start + prev->len == r->start &&
		    prev->vba + prev->len == r->vba &&
		    prev->weave == r->weave) {
			prev->len += r->len;
			whimory_range_erase(w, r);
			r = prev;
		}
	}
	q = rb_next(&r->rb);
	if (q) {
		next = rb_entry(q, struct whimory_range, rb);
		if (r->start + r->len == next->start &&
		    r->vba + r->len == next->vba &&
		    r->weave == next->weave) {
			r->len += next->len;
			whimory_range_erase(w, next);
		}
	}
}

static int whimory_range_update(struct whimory *w, u32 lba, u32 span, u32 vba)
{
	u32 end = lba + span;
	struct whimory_range *hit;
	struct rb_node *node, *next;
	int ret;

	if (!span || whimory_special_lba(lba))
		return 0;

	{
		u32 end = lba + span;
		struct rb_node *node = rb_first(&w->ranges);

		while (node) {
			struct whimory_range *r = rb_entry(node,
							  struct whimory_range,
							  rb);
			u32 r_end = r->start + r->len;

			if (r->start >= end)
				break;
			if (r_end > lba && r->start < end &&
			    r->weave > w->sftl.claim_weave)
				return 0;
			node = rb_next(node);
		}
	}

	hit = whimory_range_find(&w->ranges, lba);
	if (hit) {
		ret = whimory_range_split(w, hit, lba);
		if (ret)
			return ret;
	}
	if (end) {
		hit = whimory_range_find(&w->ranges, end - 1);
		if (hit && hit->start < end) {
			ret = whimory_range_split(w, hit, end);
			if (ret)
				return ret;
		}
	}

	node = rb_first(&w->ranges);
	while (node) {
		struct whimory_range *r = rb_entry(node, struct whimory_range,
						   rb);

		next = rb_next(node);
		if (r->start >= end)
			break;
		if (r->start >= lba && r->start + r->len <= end)
			whimory_range_erase(w, r);
		node = next;
	}

	ret = whimory_range_insert_new(w, lba, span, vba);
	if (ret)
		return ret;
	whimory_range_coalesce_at(w, lba);
	return 0;
}

/*
 *L2V_Update.c: split at 0x8000 root boundaries, then insert.
 * The interval map is the RO observable of the live tree.
 */
static int whimory_l2v_update(struct whimory *w, u32 lba, u32 span, u32 vba)
{
	w->sftl.l2v_update_calls++;
	if (vba >= w->l2v.invalid_vba)
		w->sftl.l2v_unmap_calls++;
	while (span) {
		u32 chunk = WHIMORY_L2V_ROOT_SPAN -
			    (lba & (WHIMORY_L2V_ROOT_SPAN - 1));
		int ret;

		if (chunk > span)
			chunk = span;
		if (w->l2v.root && w->l2v.num_roots) {
			u32 ridx = lba >> 15;
			u8 *rec;
			u16 ver, node_idx;

			if (ridx < w->l2v.num_roots) {
				rec = w->l2v.root + ridx * WHIMORY_L2V_ROOT_REC_SIZE;
				ver = get_unaligned_le16(rec + 4);
				if (ver == 0xffff)
					ver = 0;
				put_unaligned_le16(ver + 1, rec + 4);
				/*
 *: whole-root unmap (off=0,
 * span=0x8000, vba=invalid) frees the tree.
 */
				if (!(lba & 0x7fff) &&
				    chunk == WHIMORY_L2V_ROOT_SPAN &&
				    vba >= w->l2v.invalid_vba) {
					node_idx = get_unaligned_le16(rec);
					if (node_idx != WHIMORY_L2V_INVALID_ROOT)
						whimory_l2v_free_tree(w,
								      node_idx,
								      ridx);
					put_unaligned_le16(
						WHIMORY_L2V_INVALID_ROOT, rec);
				}
			}
			w->l2v.updates++;
			w->l2v.gen++;
			if (w->l2v.updates >= WHIMORY_L2V_UPDATE_REPACK)
				w->l2v.updates = 0;
		}
		ret = whimory_range_update(w, lba, chunk, vba);
		if (ret)
			return ret;
		if (w->l2v.root && w->l2v.num_roots) {
			u32 ridx = lba >> 15;
			bool whole_unmap = !(lba & 0x7fff) &&
				chunk == WHIMORY_L2V_ROOT_SPAN &&
				vba >= w->l2v.invalid_vba;

			if (ridx < w->l2v.num_roots && !whole_unmap) {
				ret = whimory_l2v_update_packed(w, ridx,
						lba & 0x7fff, chunk, vba);
				if (ret)
					dev_dbg(w->dev,
						"L2V packed update r=%u %d\n",
						ridx, ret);
			}
		}
		span -= chunk;
		lba += chunk;
		if (vba < w->l2v.invalid_vba)
			vba += chunk;
	}
	return 0;
}

static void whimory_range_free(struct whimory *w)
{
	struct rb_node *n;

	while ((n = rb_first(&w->ranges))) {
		struct whimory_range *r = rb_entry(n, struct whimory_range, rb);

		whimory_range_erase(w, r);
	}
	w->ranges = RB_ROOT;
	w->sftl.range_nodes = 0;
}

/* ------------------------------------------------------------------ */
/* L2V init / lookup / tree pack , */
/* ------------------------------------------------------------------ */

static void whimory_l2v_free(struct whimory *w)
{
	kvfree(w->l2v.root);
	kvfree(w->l2v.nodes);
	kvfree(w->l2v.leaf_scratch);
	w->l2v.root = NULL;
	w->l2v.nodes = NULL;
	w->l2v.leaf_scratch = NULL;
	w->l2v.num_roots = 0;
	w->l2v.nodepool_bytes = 0;
	w->l2v.free_head = WHIMORY_L2V_INVALID_ROOT;
	w->l2v.free_count = 0;
}

/* L2V_Mem.c— intrusive free list in node[0]. */
static void whimory_l2v_mem_free(struct whimory_l2v *l2v, u32 idx)
{
	u8 *node;
	u32 n = l2v->nodepool_bytes / WHIMORY_L2V_NODE_SIZE;

	if (!l2v->nodes || idx >= n)
		return;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	put_unaligned_le32(l2v->free_head, node);
	l2v->free_head = idx;
	l2v->free_count++;
}

static void whimory_l2v_mem_reset(struct whimory_l2v *l2v)
{
	u32 n = l2v->nodepool_bytes / WHIMORY_L2V_NODE_SIZE;
	s32 j;

	l2v->free_head = WHIMORY_L2V_INVALID_ROOT;
	l2v->free_count = 0;
	l2v->nodes_used = 0;
	if (!l2v->nodes || !n)
		return;
	for (j = (s32)n - 1; j >= 0; j--)
		whimory_l2v_mem_free(l2v, (u32)j);
}

static u32 whimory_l2v_alloc_node(struct whimory_l2v *l2v)
{
	u32 idx, next;
	u8 *node;
	u32 n = l2v->nodepool_bytes / WHIMORY_L2V_NODE_SIZE;

	idx = l2v->free_head;
	if (idx == WHIMORY_L2V_INVALID_ROOT || !l2v->free_count || idx >= n)
		return WHIMORY_L2V_INVALID_ROOT;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	next = get_unaligned_le32(node);
	l2v->free_head = next;
	l2v->free_count--;
	memset(node, 0, WHIMORY_L2V_NODE_SIZE);
	if (idx + 1 > l2v->nodes_used)
		l2v->nodes_used = idx + 1;
	return idx;
}

static int whimory_l2v_init(struct whimory *w, u32 max_lba,
			    u32 vba_factor_a, u32 vba_factor_b,
			    u32 nodepool_bytes)
{
	struct whimory_l2v *l2v = &w->l2v;
	u64 prod;

	whimory_l2v_free(w);

	if (nodepool_bytes < WHIMORY_MIN_NODEPOOL_BYTES)
		nodepool_bytes = WHIMORY_MIN_NODEPOOL_BYTES;

	prod = 2ull * vba_factor_a * vba_factor_b;
	if (prod < 2)
		return -EINVAL;
	l2v->bits_vba = fls64(prod - 1) - 1;
	if (!l2v->bits_vba || l2v->bits_vba > 30)
		return -EINVAL;
	l2v->spanbits_vba = 30 - l2v->bits_vba;

	prod = 2ull * (nodepool_bytes >> 6);
	if (prod < 2)
		return -EINVAL;
	l2v->bits_nodeidx = fls64(prod - 1) - 1;
	if (!l2v->bits_nodeidx || l2v->bits_nodeidx > 30)
		return -EINVAL;
	l2v->spanbits_nodeidx = 30 - l2v->bits_nodeidx;

	l2v->sentinel_vba = (1u << l2v->bits_vba) - 1;
	l2v->invalid_vba = l2v->sentinel_vba;
	l2v->num_roots = (max_lba >> 15) + 1;
	l2v->nodepool_bytes = nodepool_bytes;
	l2v->nodes_used = 0;
	l2v->updates = 0;
	l2v->gen = 0;
	l2v->frag_count = 0;
	l2v->frag_max = 0;
	l2v->free_head = WHIMORY_L2V_INVALID_ROOT;
	l2v->free_count = 0;

	l2v->root = kvzalloc(WHIMORY_L2V_ROOT_REC_SIZE * l2v->num_roots,
			     GFP_KERNEL);
	if (!l2v->root)
		return -ENOMEM;
	l2v->nodes = kvzalloc(nodepool_bytes, GFP_KERNEL);
	if (!l2v->nodes) {
		kvfree(l2v->root);
		l2v->root = NULL;
		return -ENOMEM;
	}
	l2v->leaf_scratch = kvcalloc(WHIMORY_L2V_ROOT_SPAN,
				     sizeof(*l2v->leaf_scratch), GFP_KERNEL);
	if (!l2v->leaf_scratch) {
		kvfree(l2v->nodes);
		kvfree(l2v->root);
		l2v->nodes = NULL;
		l2v->root = NULL;
		return -ENOMEM;
	}
	memset(l2v->root, 0xff, WHIMORY_L2V_ROOT_REC_SIZE * l2v->num_roots);
	memset(l2v->nodes, 0, nodepool_bytes);
	whimory_l2v_mem_reset(l2v);

	dev_info(w->dev,
		 "L2V init roots=%u nodepool=0x%x bits_vba=%u/%u bits_node=%u/%u invalid=0x%x\n",
		 l2v->num_roots, l2v->nodepool_bytes,
		 l2v->bits_vba, l2v->spanbits_vba,
		 l2v->bits_nodeidx, l2v->spanbits_nodeidx,
		 l2v->invalid_vba);
	return 0;
}

static int whimory_l2v_encode(struct whimory_l2v *l2v, u8 *node,
			      u32 *front, u32 *back, bool is_node,
			      u32 value, u32 span)
{
	u8 value_bits = is_node ? l2v->bits_nodeidx : l2v->bits_vba;
	u8 span_bits = is_node ? l2v->spanbits_nodeidx : l2v->spanbits_vba;
	u32 span_m1, span_mask, value_mask, e, need;
	bool has_ext;

	if (!span || !value_bits)
		return -EINVAL;
	span_m1 = span - 1;
	span_mask = span_bits ? ((1u << span_bits) - 1) : 0;
	value_mask = (1u << value_bits) - 1;
	if (value > value_mask)
		return -EINVAL;
	has_ext = span_m1 > span_mask;
	if (has_ext && span_bits && (span_m1 >> span_bits) > 0xffffu)
		return -E2BIG;
	need = 4 + (has_ext ? 2 : 0);
	if (*front + need > *back)
		return -ENOSPC;
	e = (is_node ? 1u : 0u) | (has_ext ? 2u : 0u);
	e |= (value & value_mask) << 2;
	e |= (span_m1 & span_mask) << (value_bits + 2);
	put_unaligned_le32(e, node + *front);
	*front += 4;
	if (has_ext) {
		*back -= 2;
		put_unaligned_le16((u16)(span_m1 >> span_bits),
				   node + *back);
	}
	return 0;
}

static u32 whimory_leaf_span_sum(const struct whimory_leaf *l, u32 n)
{
	u32 i, s = 0;

	for (i = 0; i < n; i++)
		s += l[i].span;
	return s;
}

static int whimory_l2v_pack_leaves(struct whimory *w,
				   const struct whimory_leaf *leaves, u32 n,
				   u32 *idx_out);

static int whimory_l2v_pack_parent(struct whimory *w, u32 left, u32 span_l,
				   u32 right, u32 span_r, u32 *idx_out)
{
	struct whimory_l2v *l2v = &w->l2v;
	u8 *node;
	u32 idx, front, back;
	int ret;

	idx = whimory_l2v_alloc_node(l2v);
	if (idx == WHIMORY_L2V_INVALID_ROOT)
		return -ENOMEM;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	front = 0;
	back = WHIMORY_L2V_NODE_SIZE;
	ret = whimory_l2v_encode(l2v, node, &front, &back, true, left, span_l);
	if (!ret)
		ret = whimory_l2v_encode(l2v, node, &front, &back, true,
					 right, span_r);
	if (ret) {
		memset(node, 0xff, WHIMORY_L2V_NODE_SIZE);
		return ret;
	}
	if (front <= back - 4)
		put_unaligned_le32(0xffffffff, node + front);
	*idx_out = idx;
	return 0;
}

static int whimory_l2v_pack_leaves(struct whimory *w,
				   const struct whimory_leaf *leaves, u32 n,
				   u32 *idx_out)
{
	struct whimory_l2v *l2v = &w->l2v;
	u8 *node;
	u32 idx, front, back, i, mid, left, right, span_l, span_r;
	int ret;

	if (!n)
		return -EINVAL;

	idx = whimory_l2v_alloc_node(l2v);
	if (idx == WHIMORY_L2V_INVALID_ROOT)
		return -ENOMEM;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	front = 0;
	back = WHIMORY_L2V_NODE_SIZE;
	for (i = 0; i < n; i++) {
		ret = whimory_l2v_encode(l2v, node, &front, &back, false,
					 leaves[i].vba, leaves[i].span);
		if (ret)
			break;
	}
	if (!ret) {
		if (front <= back - 4)
			put_unaligned_le32(0xffffffff, node + front);
		*idx_out = idx;
		return 0;
	}
	memset(node, 0xff, WHIMORY_L2V_NODE_SIZE);
	if (l2v->nodes_used == idx + 1)
		l2v->nodes_used = idx;

	if (n == 1) {
		struct whimory_leaf half[2];
		u32 s0, s1;

		s0 = leaves[0].span / 2;
		s1 = leaves[0].span - s0;
		if (!s0 || !s1)
			return -EINVAL;
		half[0].vba = leaves[0].vba;
		half[0].span = s0;
		half[1].vba = leaves[0].vba + s0;
		half[1].span = s1;
		ret = whimory_l2v_pack_leaves(w, half, 1, &left);
		if (ret)
			return ret;
		ret = whimory_l2v_pack_leaves(w, half + 1, 1, &right);
		if (ret)
			return ret;
		return whimory_l2v_pack_parent(w, left, s0, right, s1, idx_out);
	}

	mid = n / 2;
	if (!mid)
		mid = 1;
	ret = whimory_l2v_pack_leaves(w, leaves, mid, &left);
	if (ret)
		return ret;
	ret = whimory_l2v_pack_leaves(w, leaves + mid, n - mid, &right);
	if (ret)
		return ret;
	span_l = whimory_leaf_span_sum(leaves, mid);
	span_r = whimory_leaf_span_sum(leaves + mid, n - mid);
	return whimory_l2v_pack_parent(w, left, span_l, right, span_r, idx_out);
}

static u32 whimory_l2v_collect_root(struct whimory *w, u32 ridx,
				    struct whimory_leaf *leaves)
{
	struct whimory_l2v *l2v = &w->l2v;
	u32 base = ridx * WHIMORY_L2V_ROOT_SPAN;
	u32 win_end = base + WHIMORY_L2V_ROOT_SPAN;
	u32 cursor = base, nleaf = 0;
	struct rb_node *n;

	for (n = rb_first(&w->ranges); n; n = rb_next(n)) {
		struct whimory_range *rg = rb_entry(n, struct whimory_range, rb);
		u32 s, e, vba, span;

		if (rg->start >= win_end)
			break;
		e = rg->start + rg->len;
		if (e <= base)
			continue;
		s = max(rg->start, base);
		e = min(e, win_end);
		if (s >= e)
			continue;
		vba = rg->vba + (s - rg->start);
		if (s > cursor) {
			leaves[nleaf].vba = l2v->invalid_vba;
			leaves[nleaf].span = s - cursor;
			nleaf++;
		}
		span = e - s;
		leaves[nleaf].vba = vba;
		leaves[nleaf].span = span;
		nleaf++;
		cursor = e;
	}
	if (nleaf && cursor < win_end) {
		leaves[nleaf].vba = l2v->invalid_vba;
		leaves[nleaf].span = win_end - cursor;
		nleaf++;
	}
	return nleaf;
}

/*analogue: free this root's tree, pack from the interval map. */
static int whimory_l2v_pack_root(struct whimory *w, u32 ridx)
{
	struct whimory_l2v *l2v = &w->l2v;
	struct whimory_leaf *leaves = l2v->leaf_scratch;
	u8 *rec;
	u16 ver, node_old;
	u32 nleaf, node_idx, used0;
	int ret;

	if (!leaves || ridx >= l2v->num_roots)
		return -EINVAL;
	rec = l2v->root + ridx * WHIMORY_L2V_ROOT_REC_SIZE;
	nleaf = whimory_l2v_collect_root(w, ridx, leaves);
	node_old = get_unaligned_le16(rec);
	if (node_old != WHIMORY_L2V_INVALID_ROOT)
		whimory_l2v_free_tree(w, node_old, ridx);
	if (!nleaf) {
		put_unaligned_le16(WHIMORY_L2V_INVALID_ROOT, rec);
		put_unaligned_le16(0, rec + 2);
		return 0;
	}
	w->sftl.l2v_repack_roots++;
	used0 = l2v->nodes_used;
	ret = whimory_l2v_pack_leaves(w, leaves, nleaf, &node_idx);
	if (ret) {
		put_unaligned_le16(WHIMORY_L2V_INVALID_ROOT, rec);
		put_unaligned_le16(0, rec + 2);
		return ret;
	}
	ver = get_unaligned_le16(rec + 4);
	if (ver == 0xffff)
		ver = 1;
	put_unaligned_le16((u16)node_idx, rec);
	put_unaligned_le16((u16)min_t(u32, l2v->nodes_used - used0, 0xffff),
			   rec + 2);
	put_unaligned_le16(ver, rec + 4);
	return 0;
}

/*: first insert into an empty root — one node, up to 3 leaves. */
static int whimory_l2v_grow_empty(struct whimory *w, u32 ridx, u32 off,
				  u32 span, u32 vba)
{
	struct whimory_l2v *l2v = &w->l2v;
	u8 *node, *rec;
	u32 idx, front = 0, back = WHIMORY_L2V_NODE_SIZE;
	u32 old = l2v->invalid_vba;
	int ret;

	if (off + span > WHIMORY_L2V_ROOT_SPAN)
		return -EINVAL;
	idx = whimory_l2v_alloc_node(l2v);
	if (idx == WHIMORY_L2V_INVALID_ROOT)
		return -ENOMEM;
	node = l2v->nodes + idx * WHIMORY_L2V_NODE_SIZE;
	if (off) {
		ret = whimory_l2v_encode(l2v, node, &front, &back, false, old,
					 off);
		if (ret)
			goto fail;
	}
	ret = whimory_l2v_encode(l2v, node, &front, &back, false, vba, span);
	if (ret)
		goto fail;
	if (off + span < WHIMORY_L2V_ROOT_SPAN) {
		u32 tail = WHIMORY_L2V_ROOT_SPAN - off - span;
		u32 tail_vba = old;

		if (old < l2v->invalid_vba)
			tail_vba = old + off + span;
		ret = whimory_l2v_encode(l2v, node, &front, &back, false,
					 tail_vba, tail);
		if (ret)
			goto fail;
	}
	if (front < back)
		memset(node + front, 0xff, back - front);
	rec = l2v->root + ridx * WHIMORY_L2V_ROOT_REC_SIZE;
	put_unaligned_le16((u16)idx, rec);
	put_unaligned_le16(1, rec + 2);
	return 0;
fail:
	whimory_l2v_mem_free(l2v, idx);
	return ret;
}

static int whimory_l2v_update_packed(struct whimory *w, u32 ridx, u32 off,
				     u32 span, u32 vba)
{
	u16 node_idx;
	u8 *rec;

	if (!w->l2v.root || ridx >= w->l2v.num_roots || !span)
		return 0;
	rec = w->l2v.root + ridx * WHIMORY_L2V_ROOT_REC_SIZE;
	node_idx = get_unaligned_le16(rec);
	if (node_idx == WHIMORY_L2V_INVALID_ROOT)
		return whimory_l2v_grow_empty(w, ridx, off, span, vba);
	return whimory_l2v_pack_root(w, ridx);
}

static int whimory_l2v_build_from_ranges(struct whimory *w)
{
	struct whimory_l2v *l2v = &w->l2v;
	u32 ridx, mapped_roots = 0, mapped_lbas = 0;
	int ret = 0;
	struct rb_node *n;

	if (!l2v->root || !l2v->nodes || !l2v->leaf_scratch)
		return -ENODEV;

	{
		u16 *vers;
		u32 i;

		vers = kvmalloc_array(l2v->num_roots, sizeof(u16), GFP_KERNEL);
		if (!vers)
			return -ENOMEM;
		for (i = 0; i < l2v->num_roots; i++)
			vers[i] = get_unaligned_le16(
				l2v->root + i * WHIMORY_L2V_ROOT_REC_SIZE + 4);
		for (i = 0; i < l2v->num_roots; i++) {
			u8 *rec = l2v->root + i * WHIMORY_L2V_ROOT_REC_SIZE;

			put_unaligned_le16(WHIMORY_L2V_INVALID_ROOT, rec);
			put_unaligned_le16(0, rec + 2);
			put_unaligned_le16(vers[i], rec + 4);
		}
		kvfree(vers);
		whimory_l2v_mem_reset(l2v);
	}

	for (ridx = 0; ridx < l2v->num_roots; ridx++) {
		ret = whimory_l2v_pack_root(w, ridx);
		if (ret)
			break;
		if (get_unaligned_le16(l2v->root +
				       ridx * WHIMORY_L2V_ROOT_REC_SIZE) !=
		    WHIMORY_L2V_INVALID_ROOT)
			mapped_roots++;
	}
	for (n = rb_first(&w->ranges); n; n = rb_next(n)) {
		struct whimory_range *rg = rb_entry(n, struct whimory_range, rb);

		if (rg->vba < l2v->invalid_vba)
			mapped_lbas += rg->len;
	}
	w->sftl.mapped_roots = mapped_roots;
	w->sftl.mapped_lbas = mapped_lbas;
	if (ret)
		return ret;
	whimory_l2v_find_frag(w);
	if (l2v->free_count < WHIMORY_L2V_MIN_FREE)
		dev_warn(w->dev, "L2V free %u < %u after pack\n",
			 l2v->free_count, WHIMORY_L2V_MIN_FREE);
	dev_info(w->dev,
		 "L2V recovery OK mapped_roots=%u mapped_lbas=%u nodes_used=%u range_nodes=%u frag=%u/%u free=%u\n",
		 mapped_roots, mapped_lbas, l2v->nodes_used,
		 w->sftl.range_nodes, l2v->frag_count, l2v->frag_max,
		 l2v->free_count);
	return mapped_roots ? 0 : -ENOENT;
}

/* L2V_FindFrag.c— walk leaves, record fragment stats. */
static void whimory_l2v_find_frag_node(struct whimory *w, u32 node_idx,
				       u32 *count, u32 *maxspan, int depth)
{
	struct whimory_l2v *l2v = &w->l2v;
	const u8 *node;
	u32 front = 0, back = WHIMORY_L2V_NODE_SIZE;

	if (depth > WHIMORY_L2V_FINDFRAG_WIN ||
	    (node_idx + 1) * WHIMORY_L2V_NODE_SIZE >
	    l2v->nodepool_bytes)
		return;
	node = l2v->nodes + node_idx * WHIMORY_L2V_NODE_SIZE;
	while (front + 4 <= back) {
		u32 e = get_unaligned_le32(node + front);
		bool is_node, has_ext;
		u32 value_bits, span_bits, value, span_minus1, span;

		if (e == 0xffffffff)
			break;
		is_node = e & 1;
		has_ext = e & 2;
		value_bits = is_node ? l2v->bits_nodeidx : l2v->bits_vba;
		span_bits = is_node ? l2v->spanbits_nodeidx : l2v->spanbits_vba;
		value = (e >> 2) & ((1u << value_bits) - 1);
		span_minus1 = span_bits ?
			((e >> (value_bits + 2)) & ((1u << span_bits) - 1)) : 0;
		if (has_ext) {
			back -= 2;
			if (front + 4 > back)
				break;
			span_minus1 += (u32)get_unaligned_le16(node + back) <<
				       span_bits;
		}
		span = span_minus1 + 1;
		if (is_node)
			whimory_l2v_find_frag_node(w, value, count, maxspan,
						   depth + 1);
		else {
			(*count)++;
			if (span > *maxspan)
				*maxspan = span;
		}
		front += 4;
	}
}

static void whimory_l2v_free_tree(struct whimory *w, u32 node_idx, u32 root_idx)
{
	struct whimory_l2v *l2v = &w->l2v;
	const u8 *node;
	u32 front = 0, back = WHIMORY_L2V_NODE_SIZE;
	u8 *rec;

	if ((node_idx + 1) * WHIMORY_L2V_NODE_SIZE > l2v->nodepool_bytes)
		return;
	node = l2v->nodes + node_idx * WHIMORY_L2V_NODE_SIZE;
	while (front + 4 <= back) {
		u32 e = get_unaligned_le32(node + front);
		bool is_node, has_ext;
		u32 value_bits, value;

		if (e == 0xffffffff)
			break;
		is_node = e & 1;
		has_ext = e & 2;
		value_bits = is_node ? l2v->bits_nodeidx : l2v->bits_vba;
		value = (e >> 2) & ((1u << value_bits) - 1);
		if (has_ext) {
			back -= 2;
			if (front + 4 > back)
				break;
		}
		if (is_node)
			whimory_l2v_free_tree(w, value, root_idx);
		front += 4;
	}
	whimory_l2v_mem_free(l2v, node_idx);
	if (root_idx < l2v->num_roots) {
		rec = l2v->root + root_idx * WHIMORY_L2V_ROOT_REC_SIZE;
		{
			u16 n_nodes = get_unaligned_le16(rec + 2);

			if (n_nodes && n_nodes != 0xffff)
				put_unaligned_le16(n_nodes - 1, rec + 2);
		}
	}
}

static void whimory_l2v_find_frag(struct whimory *w)
{
	struct whimory_l2v *l2v = &w->l2v;
	u32 ridx, count = 0, maxspan = 0;

	if (!l2v->root || !l2v->nodes)
		return;
	for (ridx = 0; ridx < l2v->num_roots; ridx++) {
		u32 node_idx = get_unaligned_le16(
			l2v->root + ridx * WHIMORY_L2V_ROOT_REC_SIZE);

		if (node_idx == WHIMORY_L2V_INVALID_ROOT)
			continue;
		whimory_l2v_find_frag_node(w, node_idx, &count, &maxspan, 0);
	}
	l2v->frag_count = count;
	l2v->frag_max = maxspan;
}

static int whimory_l2v_lookup(struct whimory *w, u32 lba,
			      u32 *vba_out, u32 *span_out)
{
	struct whimory_l2v *l2v = &w->l2v;
	u32 root_idx = lba >> 15;
	u32 target = lba & 0x7fff;
	u32 consumed;
	u32 node_idx;
	int depth;

	if (!l2v->root || !l2v->nodes)
		return -ENODEV;
	if (root_idx >= l2v->num_roots)
		return -ERANGE;

	node_idx = get_unaligned_le16(l2v->root + 6 * root_idx);
	if (node_idx == WHIMORY_L2V_INVALID_ROOT)
		return -ENOENT;

	for (depth = 0; depth < 32; depth++) {
		const u8 *node;
		u32 front = 0;
		u32 back = WHIMORY_L2V_NODE_SIZE;

		if ((node_idx + 1) * WHIMORY_L2V_NODE_SIZE >
		    l2v->nodepool_bytes)
			return -EINVAL;
		node = l2v->nodes + node_idx * WHIMORY_L2V_NODE_SIZE;
		consumed = 0;

		while (front + 4 <= back) {
			u32 e = get_unaligned_le32(node + front);
			bool is_node, has_ext;
			u32 value_bits, span_bits, value_mask, value;
			u32 span_minus1, span;

			if (e == 0xffffffff)
				break;
			is_node = e & 1;
			has_ext = e & 2;
			if (is_node) {
				value_bits = l2v->bits_nodeidx;
				span_bits = l2v->spanbits_nodeidx;
			} else {
				value_bits = l2v->bits_vba;
				span_bits = l2v->spanbits_vba;
			}
			value_mask = value_bits ? ((1u << value_bits) - 1) : 0;
			value = (e >> 2) & value_mask;
			span_minus1 = e >> (value_bits + 2);
			if (span_bits)
				span_minus1 &= (1u << span_bits) - 1;
			else
				span_minus1 = 0;
			if (has_ext) {
				back -= 2;
				if (front + 4 > back)
					return -EINVAL;
				span_minus1 +=
					(u32)get_unaligned_le16(node + back) <<
					span_bits;
			}
			span = span_minus1 + 1;
			if (!span)
				return -EINVAL;
			if (target < consumed + span) {
				u32 delta = target - consumed;

				if (is_node) {
					node_idx = value;
					goto next_level;
				}
				if (value >= l2v->invalid_vba)
					return -ENOENT;
				*vba_out = value + delta;
				*span_out = span - delta;
				return 0;
			}
			consumed += span;
			front += 4;
		}
		return -EINVAL;
next_level:
		continue;
	}
	return -ELOOP;
}

/* Prefer the interval map (L2V_Update result); packed tree is for Search. */
static int whimory_l2v_search(struct whimory *w, u32 lba,
			      u32 *vba_out, u32 *span_out)
{
	struct whimory_range *r = whimory_range_find(&w->ranges, lba);

	if (r) {
		u32 delta = lba - r->start;

		*vba_out = r->vba + delta;
		*span_out = r->len - delta;
		return 0;
	}
	return whimory_l2v_lookup(w, lba, vba_out, span_out);
}

/* ------------------------------------------------------------------ */
/* FIL */
/* ------------------------------------------------------------------ */

static int whimory_fil_init(struct whimory *w)
{
	struct s5l8740_nand_geom g;
	int ret;

	ret = s5l8740_nand_hw_init();
	if (ret)
		return ret;
	ret = s5l8740_nand_query_geometry(&g);
	if (ret)
		return ret;
	if (!g.dev_id)
		return -ENODEV;

	w->geom.num_ce = g.num_ce;
	w->geom.num_cau = g.num_cau;
	w->geom.blocks_per_cau = g.blocks_per_cau;
	w->geom.pages_per_block = g.pages_per_block;
	w->geom.page_size = g.page_size;
	w->geom.vfl_tail = g.vfl_tail;
	w->geom.user_blocks = g.blocks_per_cau - g.vfl_tail;
	w->geom.dev_id = s5l8740_nand_fil_get_info(101);
	w->geom.geom_104 = s5l8740_nand_fil_get_info(104);
	w->geom.geom_105 = s5l8740_nand_fil_get_info(105);
	w->geom.geom_135 = s5l8740_nand_fil_get_info(135);
	if (!w->geom.dev_id)
		return -ENODEV;
	if (w->geom.geom_104 && w->geom.geom_104 != w->geom.page_size) {
		dev_err(w->dev,
			"FIL GetInfo(104)=%u != page_size=%u\n",
			w->geom.geom_104, w->geom.page_size);
		return -EINVAL;
	}
	if (w->geom.geom_105 && w->geom.geom_105 != WHIMORY_FIL_META_BYTES) {
		dev_err(w->dev, "FIL GetInfo(105)=%u != %u\n",
			w->geom.geom_105, WHIMORY_FIL_META_BYTES);
		return -EINVAL;
	}

	dev_info(w->dev,
		 "FIL_Init OK dev_id=%u g104=%u g105=%u g135=%u ce=%u cau=%u blocks=%u user=%u param=%d\n",
		 w->geom.dev_id, w->geom.geom_104, w->geom.geom_105,
		 w->geom.geom_135, w->geom.num_ce, w->geom.num_cau,
		 w->geom.blocks_per_cau, w->geom.user_blocks,
		 g.from_param_page);
	w->fil_ok = true;
	return 0;
}

/* ------------------------------------------------------------------ */
/* FPart — signature from media (or oracle firmware file) */
/* ------------------------------------------------------------------ */

/*
 *— FPart signature is NOT a user-page hunt.
 * OSOS: memset(sig, 0xA5, 0x600) then _fpart->op80(sig, 0x600, 0xC101).
 * READ ONLY — never AllocateSpecialBlock / WriteSpecial / erase.
 * Validate: magic 0x776d7278, ver<=6, +0x34 == FIL GetInfo(101).
 */
static void whimory_log_sig_fields(struct whimory *w, const u8 *s,
				   const char *why)
{
	u32 magic = whimory_sig32(s, 0x00);
	u32 ver = whimory_sig32(s, 0x08);
	u32 ftl_m = whimory_sig32(s, 0x0c);
	u32 ftl_n = whimory_sig32(s, 0x10);
	u32 vfl_m = whimory_sig32(s, 0x18);
	u32 vfl_n = whimory_sig32(s, 0x1c);
	u32 vfl_arg = whimory_sig32(s, 0x20);
	u32 fpt_m = whimory_sig32(s, 0x24);
	u32 fpt_n = whimory_sig32(s, 0x28);
	u32 fpt_a = whimory_sig32(s, 0x2c);
	u32 extra = whimory_sig32(s, 0x30);
	u32 geom = whimory_sig32(s, 0x34);
	u32 cfg = whimory_sig32(s, 0xb8);

	dev_info(w->dev,
		 "WHIMORY_SIG %s magic=%08x ver=%u ftl=%u.%u vfl=%u.%u fpart=%u.%u geom=%u fil101=%u vfl_arg=%u fpart_arg=%u extra=%u cfg_b8=%u first32=%32ph\n",
		 why, magic, ver, ftl_m, ftl_n, vfl_m, vfl_n, fpt_m, fpt_n,
		 geom, w->geom.dev_id, vfl_arg, fpt_a, extra, cfg, s);
}

/* OSOSchecks — not the old ver>=1 / major<=16 heuristic. */
static int whimory_validate_signature(struct whimory *w, const u8 *sig)
{
	u32 magic = whimory_sig32(sig, 0x00);
	u32 ver = whimory_sig32(sig, 0x08);
	u32 geom = whimory_sig32(sig, 0x34);

	whimory_log_sig_fields(w, sig, "validate");
	if (magic != WHIMORY_SIG_MAGIC) {
		dev_info(w->dev,
			 "FPART_SIG_READ reject: magic=%08x want=776d7278\n",
			 magic);
		return -EINVAL;
	}
	if (ver > 6) {
		dev_info(w->dev,
			 "FPART_SIG_READ reject: version=%u > 6\n", ver);
		return -EINVAL;
	}
	if (geom != w->geom.dev_id) {
		dev_info(w->dev,
			 "FPART_SIG_READ reject: geom=%u != FIL GetInfo(101)=%u\n",
			 geom, w->geom.dev_id);
		return -EINVAL;
	}
	return 0;
}

static int whimory_parse_signature(struct whimory *w, const u8 *s)
{
	int ret;

	ret = whimory_validate_signature(w, s);
	if (ret)
		return ret;
	memcpy(w->sig.raw, s, WHIMORY_SIG_SIZE);
	w->sig.version = whimory_sig32(s, 0x08);
	w->sig.ftl_major = whimory_sig32(s, 0x0c);
	w->sig.ftl_minor = whimory_sig32(s, 0x10);
	w->sig.vfl_major = whimory_sig32(s, 0x18);
	w->sig.vfl_minor = whimory_sig32(s, 0x1c);
	w->sig.fpart_major = whimory_sig32(s, 0x24);
	w->sig.fpart_minor = whimory_sig32(s, 0x28);
	w->sig.sig_geom = whimory_sig32(s, 0x34);
	w->sig.flags_or_open = whimory_sig32(s, 0x20);
	w->sig.fpart_arg = whimory_sig32(s, 0x2c);
	w->sig.extra_arg = whimory_sig32(s, 0x30);
	w->sig_ok = true;
	dev_info(w->dev,
		 "Whimory sig OK ver=%u fpart=%u.%u vfl=%u.%u ftl=%u.%u geom=%u vfl_arg=%u fpart_arg=%u extra=%u\n",
		 w->sig.version, w->sig.fpart_major, w->sig.fpart_minor,
		 w->sig.vfl_major, w->sig.vfl_minor,
		 w->sig.ftl_major, w->sig.ftl_minor, w->sig.sig_geom,
		 w->sig.flags_or_open, w->sig.fpart_arg, w->sig.extra_arg);
	return 0;
}

static int n31_fpart_init(struct whimory *w)
{
	if (!w->fil_ok)
		return -ENODEV;
	memset(w->fpart_ctx.table, 0xff, sizeof(w->fpart_ctx.table));
	w->fpart_ctx.count = 0;
	w->fpart_ctx.scanned = false;
	return 0;
}

static u32 n31_fpart_minor(struct whimory *w)
{
	return w->sig.fpart_minor;
}

static u16 fpart_num_banks(const struct whimory *w)
{
	return w->geom.num_ce * w->geom.num_cau;
}

static void fpart_bank_to_ce_cau(const struct whimory *w, u16 bank,
				 unsigned int *ce, unsigned int *cau)
{
	u16 ncau = w->geom.num_cau ? w->geom.num_cau : 1;

	*ce = bank / ncau;
	*cau = bank % ncau;
}

static bool fpart_type_class1(u16 type_word)
{
	return ((type_word >> 8) & FPART_SPECIAL_CLASS_MASK) ==
	       FPART_SPECIAL_CLASS;
}

static bool fpart_meta_special(const u8 *meta, u8 want_chunk, u16 *type_out);
static bool fpart_meta_is_assign(const u8 *meta, u16 *type_out);
static bool fpart_has_xrmw(const u8 *page);

/*
 *op=1 analogue. Special objects often live on SLC; try SLC
 * then MLC. Full 16 KiB data + 64B META; special uses first 16 META bytes.
 */
static int fpart_fil_read_page(struct whimory *w, u16 bank, u32 block,
			       u32 page, void *data, u8 *meta)
{
	unsigned int ce, cau, i;
	int last = -EIO;
	const unsigned int slc_order[2] = { 1, 0 };

	fpart_bank_to_ce_cau(w, bank, &ce, &cau);
	if (ce >= w->geom.num_ce || cau >= w->geom.num_cau ||
	    block >= w->geom.blocks_per_cau ||
	    page >= w->geom.pages_per_block)
		return -EINVAL;

	for (i = 0; i < 2; i++) {
		int ret;

		ret = s5l8740_nand_page_read(ce, cau, block, page, slc_order[i],
					     16, data, w->geom.page_size,
					     meta, S5L8740_NAND_META_SIZE);
		if (ret)
			continue;
		last = 0;
		if (fpart_meta_special(meta, 0, NULL) ||
		    fpart_meta_is_assign(meta, NULL))
			return 0;
		if (payload_magic_scan && fpart_has_xrmw(data))
			return 0;
	}
	return last;
}

/*— 16-byte META copy. LE type_word at +2 (RE). */
static bool fpart_meta_special(const u8 *meta, u8 want_chunk, u16 *type_out)
{
	unsigned int slot;

	if (!meta)
		return false;
	for (slot = 0; slot < 4; slot++) {
		const u8 *m = meta + slot * WHIMORY_META_SIZE;

		if (m[0] != FPART_SPECIAL_TAG)
			continue;
		if (m[1] != want_chunk)
			continue;
		if (type_out)
			*type_out = get_unaligned_le16(m + 2);
		return true;
	}
	return false;
}

/*
 * Scanner: META tag 0x30 and class 1. Chunk-0 assignment pages use m[1]==0
 * with class in type_word[15:8]. Avoid treat payload magic as a hit.
 */
static bool fpart_meta_is_assign(const u8 *meta, u16 *type_out)
{
	unsigned int slot;

	if (!meta)
		return false;
	for (slot = 0; slot < 4; slot++) {
		const u8 *m = meta + slot * WHIMORY_META_SIZE;
		u16 tw;

		if (m[0] != FPART_SPECIAL_TAG)
			continue;
		tw = get_unaligned_le16(m + 2);
		if ((m[1] & FPART_SPECIAL_CLASS_MASK) == FPART_SPECIAL_CLASS ||
		    (m[1] == 0 && fpart_type_class1(tw))) {
			if (type_out)
				*type_out = tw;
			return true;
		}
	}
	return false;
}

static int fpart_meta_special_slot(const u8 *meta, u8 want_chunk, u16 *type_out)
{
	unsigned int slot;

	if (!meta)
		return -1;
	for (slot = 0; slot < 4; slot++) {
		const u8 *m = meta + slot * WHIMORY_META_SIZE;

		if (m[0] != FPART_SPECIAL_TAG)
			continue;
		if (m[1] != want_chunk)
			continue;
		if (type_out)
			*type_out = get_unaligned_le16(m + 2);
		return (int)slot;
	}
	return -1;
}

static bool fpart_meta_interesting(const u8 *m)
{
	return m && (m[0] == FPART_SPECIAL_TAG ||
		     m[0] == WHIMORY_META_TYPE_VFL_CXT ||
		     m[0] == WHIMORY_META_TYPE_SFTL_CXT ||
		     m[0] == WHIMORY_META_TYPE_BTOC);
}

static u32 fpart_word_at(const u8 *page, unsigned int off)
{
	return get_unaligned_le32(page + off);
}

static bool fpart_has_xrmw(const u8 *page)
{
	u32 a = fpart_word_at(page, 0);
	u32 b = fpart_word_at(page, FPART_SPECIAL_HDR);

	return a == WHIMORY_SIG_MAGIC || b == WHIMORY_SIG_MAGIC;
}

static bool fpart_has_wrmx(const u8 *page)
{
	u32 a = fpart_word_at(page, 0);
	u32 b = fpart_word_at(page, FPART_SPECIAL_HDR);

	return a == WHIMORY_SIG_MAGIC_WRMX || b == WHIMORY_SIG_MAGIC_WRMX;
}

/*
 * Cache insert: sorted by type_word, then bank, then block (OSOS
 * fpart_special_cache_add_pairs). Table size 0x2d0 / 6 = 120.
 */
static int fpart_cache_add(struct whimory *w, u16 bank, u16 block,
			   u16 type_word)
{
	struct fpart_special_entry *t = w->fpart_ctx.table;
	u16 n = w->fpart_ctx.count;
	u16 i, j;

	if (n >= FPART_SPECIAL_MAX_ENTRIES)
		return -ENOSPC;

	for (i = 0; i < n; i++) {
		if (t[i].type_word == type_word && t[i].bank == bank &&
		    t[i].block == block)
			return 0;
		if (t[i].type_word > type_word)
			break;
		if (t[i].type_word == type_word && t[i].bank > bank)
			break;
		if (t[i].type_word == type_word && t[i].bank == bank &&
		    t[i].block > block)
			break;
	}
	for (j = n; j > i; j--)
		t[j] = t[j - 1];
	t[i].bank = bank;
	t[i].block = block;
	t[i].type_word = type_word;
	w->fpart_ctx.count = n + 1;
	return 1;
}

/* Assignment page DATA: up to eight u16 bank, u16 block; bank==0xffff ends. */
static bool fpart_data_is_assignment(const u8 *data, u16 nbanks, u32 nblk)
{
	unsigned int i, valid = 0;

	for (i = 0; i < FPART_ASSIGN_MAX_PAIRS; i++) {
		u16 bank = get_unaligned_le16(data + i * 4);
		u16 block = get_unaligned_le16(data + i * 4 + 2);

		if (bank == 0xffff)
			return i == 0 || valid > 0;
		if (bank >= nbanks || block >= nblk)
			return false;
		valid++;
	}
	return valid > 0;
}

static unsigned int fpart_ingest_pairs(struct whimory *w, const u8 *data,
				       u16 type_word)
{
	unsigned int i, added = 0;
	u16 nbanks = fpart_num_banks(w);

	for (i = 0; i < FPART_ASSIGN_MAX_PAIRS; i++) {
		u16 bank = get_unaligned_le16(data + i * 4);
		u16 block = get_unaligned_le16(data + i * 4 + 2);

		if (bank == 0xffff)
			break;
		if (bank >= nbanks || block >= w->geom.blocks_per_cau) {
			dev_info(w->dev,
				 "FPART_ASSIGN skip pair bank=%u block=%u (banks=%u blocks=%u)\n",
				 bank, block, nbanks, w->geom.blocks_per_cau);
			continue;
		}
		if (fpart_cache_add(w, bank, block, type_word) > 0) {
			dev_info(w->dev,
				 "FPART_ASSIGN_ADD bank=%u block=%u type=0x%04x\n",
				 bank, block, type_word);
			added++;
		}
	}
	return added;
}

static bool fpart_find_in_cache(struct whimory *w, u16 *index, u16 type)
{
	u8 low = type & 0xff;
	u16 i;

	for (i = 0; i < w->fpart_ctx.count; i++) {
		if ((w->fpart_ctx.table[i].type_word & 0xff) == low) {
			*index = i;
			return true;
		}
	}
	return false;
}

static u16 fpart_count_special_copies(struct whimory *w, u16 type_word)
{
	u8 low = type_word & 0xff;
	u16 n = 0, i;

	for (i = 0; i < w->fpart_ctx.count; i++) {
		if ((w->fpart_ctx.table[i].type_word & 0xff) == low)
			n++;
	}
	return n;
}

static int fpart_scan_region(struct whimory *w, u16 type,
			     u32 block_lo, u32 block_hi,
			     unsigned int page_lo, unsigned int page_hi,
			     bool *matched)
{
	u8 *page;
	u8 meta[S5L8740_NAND_META_SIZE];
	u16 bank, nbanks = fpart_num_banks(w);
	u32 b, p;
	int ret, reads = 0, tag30 = 0, xrmw = 0, wrmx = 0, fail = 0;
	unsigned int sample = 0;
	u32 hist[256];

	page = kvmalloc(w->geom.page_size, GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	memset(hist, 0, sizeof(hist));

	if (page_hi >= w->geom.pages_per_block)
		page_hi = w->geom.pages_per_block - 1;

	s5l8740_nand_reset();

	for (bank = 0; bank < nbanks; bank++) {
		for (b = block_hi; b > block_lo; b--) {
			u32 blk = b - 1;

			for (p = page_lo; p <= page_hi; p++) {
				u16 type_word = 0;
				unsigned int ce, cau, pairs;
				u32 obj_len;
				bool special, magic;

				cond_resched();
				ret = fpart_fil_read_page(w, bank, blk, p,
							  page, meta);
				reads++;
				if (ret) {
					fail++;
					continue;
				}
				hist[meta[0]]++;
				if (sample < 12) {
					fpart_bank_to_ce_cau(w, bank, &ce, &cau);
					dev_info(w->dev,
						 "FPART_META_SAMPLE n=%u bank=%u ce=%u cau=%u blk=%u pg=%u meta=%16ph data00=%32ph data80=%32ph\n",
						 sample, bank, ce, cau, blk, p,
						 meta, page,
						 page + FPART_SPECIAL_HDR);
					sample++;
				}
				{
					unsigned int s, interesting = 0;

					for (s = 0; s < 4; s++)
						if (fpart_meta_interesting(meta +
									    s * WHIMORY_META_SIZE))
							interesting++;
					if (interesting && w->fpart_ctx.slot_logs < 48) {
						fpart_bank_to_ce_cau(w, bank, &ce, &cau);
						for (s = 0; s < 4; s++) {
							const u8 *m = meta + s * WHIMORY_META_SIZE;
							const u8 *d = page + s * WHIMORY_LBA_SIZE;

							if (!fpart_meta_interesting(m) &&
							    s != 0)
								continue;
							dev_info(w->dev,
								 "FPART_SLOTS n=%u bank=%u ce=%u cau=%u blk=%u pg=%u slot=%u type=%02x chunk=%02x tw=0x%04x meta=%16ph data=%32ph\n",
								 w->fpart_ctx.slot_logs,
								 bank, ce, cau, blk, p, s,
								 m[0], m[1],
								 get_unaligned_le16(m + 2),
								 m, d);
						}
						w->fpart_ctx.slot_logs++;
					}
				}
				special = fpart_meta_is_assign(meta, &type_word);
				if (!special)
					special = fpart_meta_special(meta, 0,
								     &type_word);
				magic = fpart_has_xrmw(page);
				if (fpart_has_wrmx(page))
					wrmx++;
				if (magic)
					xrmw++;
				if (!special)
					continue;
				tag30++;
				fpart_bank_to_ce_cau(w, bank, &ce, &cau);
				dev_info(w->dev,
					 "FPART_ASSIGN_SCAN bank=%u ce=%u cau=%u block=%u page=%u slot=%d type_word=0x%04x blank=%d m0=%16ph m1=%16ph m2=%16ph m3=%16ph data00=%32ph data80=%32ph\n",
					 bank, ce, cau, blk, p,
					 fpart_meta_special_slot(meta, 0, NULL),
					 type_word,
					 whimory_page_blank(page, 256),
					 meta, meta + 16, meta + 32, meta + 48,
					 page, page + FPART_SPECIAL_HDR);
				if (whimory_page_blank(page, 256)) {
					dev_info(w->dev,
						 "FPART_ASSIGN skip blank data type_word=0x%04x\n",
						 type_word);
					continue;
				}
				if (fpart_data_is_assignment(page, nbanks,
							     w->geom.blocks_per_cau)) {
					pairs = fpart_ingest_pairs(w, page,
								   type_word);
					dev_info(w->dev,
						 "FPART_ASSIGN_PAGE type_word=0x%04x pairs=%u count=%u\n",
						 type_word, pairs,
						 w->fpart_ctx.count);
				} else {
					obj_len = get_unaligned_le32(page +
							FPART_SPECIAL_LEN_OFF);
					if (magic ||
					    (obj_len && obj_len != 0xffffffffu &&
					     obj_len < 0x100000u)) {
						if (fpart_cache_add(w, bank,
								    blk,
								    type_word) > 0)
							dev_info(w->dev,
								 "FPART_ASSIGN_ADD bank=%u block=%u type=0x%04x (object chunk0)\n",
								 bank, blk,
								 type_word);
					}
				}
				if ((type_word & 0xff) == (type & 0xff))
					*matched = true;
			}
		}
	}
	dev_info(w->dev,
		 "FPART_SCAN blk[%u,%u) pages[%u,%u] reads=%d fail=%d tag30=%d xrmw=%d wrmx=%d entries=%u matched=%d meta0_top=%02x/%u %02x/%u %02x/%u %02x/%u\n",
		 block_lo, block_hi, page_lo, page_hi, reads, fail, tag30, xrmw,
		 wrmx, w->fpart_ctx.count, *matched,
		 0xff, hist[0xff], 0x00, hist[0], 0x30, hist[0x30], 0x20,
		 hist[0x20]);
	kvfree(page);
	return 0;
}

/*
 * fpart_locate_special_4EBBDC: cache by low byte, else scan tail assignment
 * pages (META 0x30 chunk 0). scanned=true after a full miss so we do not
 * rescan.op=4 bitmap is not ported — every tail block is read.
 */
static bool fpart_locate_special(struct whimory *w, u16 *index, u16 type)
{
	u32 tail, start, nblk = w->geom.blocks_per_cau;
	unsigned int npg = fpart_assign_pages ? fpart_assign_pages : 1;
	bool matched = false;

	if (fpart_find_in_cache(w, index, type))
		return true;
	if (w->fpart_ctx.scanned)
		return false;

	tail = sig_scan_blocks ? sig_scan_blocks : w->geom.vfl_tail;
	if (!tail)
		tail = 128;
	if (tail > nblk)
		tail = nblk;
	start = nblk - tail;

	dev_info(w->dev,
		 "FPART_LOCATE type=0x%04x tail=%u start=%u pages=0..%u banks=%u\n",
		 type, tail, start, npg - 1, fpart_num_banks(w));

	if (fpart_scan_region(w, type, start, nblk, 0, npg - 1, &matched))
		return false;

	if (!matched && sig_brute_scan && start) {
		dev_info(w->dev,
			 "FPART_LOCATE brute remaining blk[0,%u) (debug)\n",
			 start);
		fpart_scan_region(w, type, 0, start, 0, 0, &matched);
	}

	w->fpart_ctx.scanned = true;
	if (!fpart_find_in_cache(w, index, type)) {
		dev_info(w->dev,
			 "FPART_LOCATE type=0x%04x miss entries=%u\n",
			 type, w->fpart_ctx.count);
		return false;
	}
	return true;
}

/*
 * fpart_read_special_copy_4F1420.
 * Chunk 0: META 30 <chunk> type_word; object_len @+0x24, gen @+0x28,
 * payload @+0x80. Later chunks: payload @+0, dst off = page_size*chunk-128.
 */
static int fpart_read_special_copy(struct whimory *w, u8 *dst, u32 dst_len,
				   u16 entry_i, u32 *gen_out)
{
	struct fpart_special_entry *e;
	u8 *page;
	u8 meta[S5L8740_NAND_META_SIZE];
	u32 page_size, chunk_count = 1, copy_slots, chunk, slot;
	u32 object_len = 0, copy_len = 0, generation = 0;
	int ret = -ENOENT;

	if (entry_i >= w->fpart_ctx.count)
		return -EINVAL;
	e = &w->fpart_ctx.table[entry_i];
	page_size = w->geom.page_size;
	copy_slots = w->geom.pages_per_block;
	if (!page_size || !copy_slots)
		return -EINVAL;

	page = kvmalloc(page_size, GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	for (chunk = 0; chunk < chunk_count; chunk++) {
		bool got = false;

		for (slot = 0; slot < copy_slots; slot++) {
			u32 pg = chunk + slot * chunk_count;
			u16 meta_type = 0;

			if (pg >= w->geom.pages_per_block)
				break;
			cond_resched();
			ret = fpart_fil_read_page(w, e->bank, e->block, pg,
						  page, meta);
			if (ret)
				continue;
			if (!fpart_meta_special(meta, chunk, &meta_type))
				continue;
			if (meta_type != e->type_word)
				continue;
			got = true;

			if (chunk == 0) {
				object_len = get_unaligned_le32(page +
								FPART_SPECIAL_LEN_OFF);
				generation = get_unaligned_le32(page +
								FPART_SPECIAL_GEN_OFF);
				if (!object_len || object_len == 0xffffffffu) {
					got = false;
					continue;
				}
				copy_len = min(object_len, dst_len ? dst_len :
					       object_len);
				chunk_count = DIV_ROUND_UP(copy_len +
							   FPART_SPECIAL_HDR,
							   page_size);
				if (!chunk_count)
					chunk_count = 1;
				copy_slots = w->geom.pages_per_block /
					     chunk_count;
				if (!copy_slots)
					copy_slots = 1;
				dev_info(w->dev,
					 "FPART_SPECIAL_COPY entry=%u bank=%u block=%u type_word=0x%04x chunk0 page=%u meta=%16ph object_len=0x%x gen=%u raw00=%32ph raw80=%32ph\n",
					 entry_i, e->bank, e->block,
					 e->type_word, pg, meta, object_len,
					 generation, page,
					 page + FPART_SPECIAL_HDR);
				if (dst && dst_len) {
					u32 n = min(page_size - FPART_SPECIAL_HDR,
						    copy_len);

					n = min(n, dst_len);
					memcpy(dst, page + FPART_SPECIAL_HDR, n);
				}
			} else if (dst && dst_len) {
				u32 dst_off = page_size * chunk - FPART_SPECIAL_HDR;
				u32 n;

				if (dst_off >= dst_len || dst_off >= copy_len)
					break;
				n = min(page_size, copy_len - dst_off);
				n = min(n, dst_len - dst_off);
				memcpy(dst + dst_off, page, n);
			}
			break;
		}
		if (!got) {
			ret = -ENOENT;
			goto out;
		}
	}
	if (gen_out)
		*gen_out = generation;
	if (dst && dst_len)
		dev_info(w->dev,
			 "FPART_SPECIAL_PAYLOAD entry=%u first32=%32ph\n",
			 entry_i, dst);
	ret = 0;
out:
	kvfree(page);
	return ret;
}

/* Newest generation among contiguous low-byte copies (4F12DC / 4EB428). */
static int fpart_read_special_by_index(struct whimory *w, u8 *dst, u32 len,
				       u16 index)
{
	u16 type_word, copies, i, best_i = 0;
	bool have = false;
	u32 best_gen = 0;

	if (index >= w->fpart_ctx.count)
		return -EINVAL;
	type_word = w->fpart_ctx.table[index].type_word;
	copies = fpart_count_special_copies(w, type_word);
	dev_info(w->dev,
		 "FPART_READ_SPECIAL type=0x%04x index=%u copies=%u class=%u\n",
		 type_word, index, copies, (type_word >> 8) &
		 FPART_SPECIAL_CLASS_MASK);

	for (i = 0; i < copies; i++) {
		u16 entry_i = index + i;
		u32 gen = 0;
		int ok;

		if (entry_i >= w->fpart_ctx.count)
			break;
		if ((w->fpart_ctx.table[entry_i].type_word & 0xff) !=
		    (type_word & 0xff))
			break;
		dev_info(w->dev,
			 "FPART_READ_SPECIAL copy=%u bank=%u block=%u type_word=0x%04x\n",
			 i, w->fpart_ctx.table[entry_i].bank,
			 w->fpart_ctx.table[entry_i].block,
			 w->fpart_ctx.table[entry_i].type_word);
		ok = fpart_read_special_copy(w, have ? NULL : dst,
					     have ? 0 : len, entry_i, &gen);
		if (ok)
			continue;
		if (!have) {
			have = true;
			best_gen = gen;
			best_i = entry_i;
			continue;
		}
		if (gen > best_gen) {
			if (!fpart_read_special_copy(w, dst, len, entry_i,
						     &gen)) {
				best_gen = gen;
				best_i = entry_i;
			} else if (fpart_read_special_copy(w, dst, len,
							   best_i, NULL)) {
				return -EIO;
			}
		}
	}
	if (!have)
		return -ENOENT;
	dev_info(w->dev,
		 "FPART_READ_SPECIAL selected entry=%u gen=%u\n",
		 best_i, best_gen);
	return 0;
}

/*
 * fpart_read_special_common_4EEB68 / vtable +80.
 * Do NOT expect xrmw at raw page offset 0 — payload is after the 0x80 header.
 */
static int whimory_fpart_read_special(struct whimory *w, u32 type, u8 *buf,
				      size_t len)
{
	u16 index = (u16)type;
	u16 type_word;

	if (!buf || !len || type > 0xffff)
		return -EINVAL;

	memset(buf, 0xa5, len);
	if (!fpart_locate_special(w, &index, (u16)type))
		return -ENOENT;

	type_word = w->fpart_ctx.table[index].type_word;
	dev_info(w->dev,
		 "FPART_LOCATE type=0x%04x index=%u entry bank=%u block=%u type_word=0x%04x class=%u\n",
		 type, index, w->fpart_ctx.table[index].bank,
		 w->fpart_ctx.table[index].block, type_word,
		 (type_word >> 8) & FPART_SPECIAL_CLASS_MASK);
	if (!fpart_type_class1(type_word))
		return -EINVAL;

	return fpart_read_special_by_index(w, buf, len, index);
}

static int n31_fpart_read_signature(struct whimory *w, u8 *buf, size_t len)
{
	return whimory_fpart_read_special(w, WHIMORY_SIG_TYPE, buf, len);
}

static const struct whimory_fpart_ops n31_ppn_fpart_ops = {
	.major = 0,
	.minor = n31_fpart_minor,
	.init = n31_fpart_init,
	.read_special = whimory_fpart_read_special,
	.read_signature = n31_fpart_read_signature,
};

static void whimory_dump256(struct whimory *w, const char *tag, const u8 *p)
{
	unsigned int i;

	for (i = 0; i < 256; i += 32)
		dev_info(w->dev, "%s +0x%02x: %32ph\n", tag, i, p + i);
}

static int whimory_payload_read_page(struct whimory *w, u16 bank, u32 block,
				     u32 page, void *data, unsigned int *slc_out)
{
	unsigned int ce, cau, i;
	const unsigned int slc_order[2] = { 1, 0 };
	int last = -EIO;

	fpart_bank_to_ce_cau(w, bank, &ce, &cau);
	if (ce >= w->geom.num_ce || cau >= w->geom.num_cau ||
	    block >= w->geom.blocks_per_cau ||
	    page >= w->geom.pages_per_block)
		return -EINVAL;

	for (i = 0; i < 2; i++) {
		int ret;

		ret = s5l8740_nand_page_read(ce, cau, block, page, slc_order[i],
					     16, data, w->geom.page_size,
					     NULL, 0);
		if (ret) {
			last = ret;
			continue;
		}
		if (slc_out)
			*slc_out = slc_order[i];
		/* One successful FIL read is enough — do not MLC-retry blanks. */
		return 0;
	}
	return last;
}

static int whimory_payload_check_hit(struct whimory *w, u16 bank, u32 blk,
				     u32 pg, unsigned int slc, const u8 *page)
{
	unsigned int ce, cau, i;
	const unsigned int offs[2] = { 0, FPART_SPECIAL_HDR };

	fpart_bank_to_ce_cau(w, bank, &ce, &cau);
	for (i = 0; i < 2; i++) {
		unsigned int off = offs[i];
		u32 mag;
		const char *kind;

		if (off + 4 > w->geom.page_size)
			continue;
		mag = get_unaligned_le32(page + off);
		if (mag == WHIMORY_SIG_MAGIC)
			kind = "xrmw";
		else if (mag == WHIMORY_SIG_MAGIC_WRMX)
			kind = "wrmx";
		else
			continue;
		dev_info(w->dev,
			 "PAYLOAD_MAGIC hit kind=%s off=0x%x bank=%u ce=%u cau=%u blk=%u pg=%u slc=%u\n",
			 kind, off, bank, ce, cau, blk, pg, slc);
		whimory_dump256(w, "PAYLOAD_MAGIC data00", page);
		whimory_dump256(w, "PAYLOAD_MAGIC data80", page + FPART_SPECIAL_HDR);
		if (mag == WHIMORY_SIG_MAGIC &&
		    off + WHIMORY_SIG_SIZE <= w->geom.page_size &&
		    !whimory_parse_signature(w, page + off))
			dev_info(w->dev, "PAYLOAD_MAGIC parsed xrmw as signature\n");
		return 1;
	}
	return 0;
}

static int whimory_payload_scan_range(struct whimory *w, void *page,
				      u32 block_lo, u32 block_hi,
				      unsigned int page_lo, unsigned int page_hi,
				      const char *why, int *reads)
{
	u16 bank, nbanks = fpart_num_banks(w);
	u32 b, p;

	if (page_hi >= w->geom.pages_per_block)
		page_hi = w->geom.pages_per_block - 1;
	if (block_hi > w->geom.blocks_per_cau)
		block_hi = w->geom.blocks_per_cau;
	if (block_lo >= block_hi)
		return 0;

	dev_info(w->dev,
		 "PAYLOAD_SCAN %s blk[%u,%u) pages[%u,%u] banks=%u\n",
		 why, block_lo, block_hi, page_lo, page_hi, nbanks);

	for (bank = 0; bank < nbanks; bank++) {
		for (b = block_hi; b > block_lo; b--) {
			u32 blk = b - 1;

			for (p = page_lo; p <= page_hi; p++) {
				unsigned int slc = 0;
				int ret;

				cond_resched();
				ret = whimory_payload_read_page(w, bank, blk, p,
								page, &slc);
				(*reads)++;
				if (!(*reads % 512))
					dev_info(w->dev,
						 "PAYLOAD_SCAN %s progress reads=%d blk=%u pg=%u\n",
						 why, *reads, blk, p);
				if (ret)
					continue;
				if (whimory_payload_check_hit(w, bank, blk, p,
							      slc, page))
					return 1;
			}
		}
	}
	return 0;
}

/*
 * Data-only xrmw/wrmx hunt. PIO last_spare is not Sogeti META. Abort on
 * first hit. No classify / L2V.
 */
static int whimory_payload_magic_scan(struct whimory *w)
{
	void *page;
	int reads = 0, hit;
	u32 nblk = w->geom.blocks_per_cau;
	u32 user = w->geom.user_blocks;
	u32 around = 1461;

	if (!user || user > nblk)
		user = nblk > 128 ? nblk - 128 : nblk;
	if (around >= nblk)
		around = nblk / 2;

	page = kvmalloc(w->geom.page_size, GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	s5l8740_nand_reset();
	hit = whimory_payload_scan_range(w, page, user, nblk, 0,
					 w->geom.pages_per_block - 1,
					 "tail", &reads);
	if (!hit)
		hit = whimory_payload_scan_range(w, page, 0, user, 0, 0,
						 "user-pg0", &reads);
	if (!hit && around + 1 < nblk)
		hit = whimory_payload_scan_range(w, page, around, around + 1, 0,
						 w->geom.pages_per_block - 1,
						 "blk1461", &reads);
	dev_info(w->dev, "PAYLOAD_SCAN done hit=%d reads=%d sig=%d\n",
		 hit, reads, w->sig_ok);
	kvfree(page);
	return hit;
}

static int whimory_read_signature(struct whimory *w)
{
	int ret;

	w->fpart = &n31_ppn_fpart_ops;
	ret = w->fpart->init(w);
	if (ret)
		return ret;
	if (payload_magic_scan)
		whimory_payload_magic_scan(w);
	ret = w->fpart->read_signature(w, w->sig.raw, WHIMORY_SIG_SIZE);
	if (ret) {
		dev_warn(w->dev,
			 "FPart special 0xC101 miss (%d). sig=0 is not a native open.\n",
			 ret);
		if (!allow_sigless_debug) {
			dev_err(w->dev,
				"allow_sigless_debug=0: refusing VFL/FTL without signature\n");
			return ret;
		}
		dev_warn(w->dev,
			 "allow_sigless_debug=1: classify/recover anyway (research only)\n");
		return 0;
	}
	return whimory_parse_signature(w, w->sig.raw);
}

static u32 n31_vfl_minor(struct whimory *w)
{
	return w->sig.vfl_minor;
}

static u32 n31_sftl_minor(struct whimory *w)
{
	return w->sig.ftl_minor;
}

/* ------------------------------------------------------------------ */
/* VFL */
/* ------------------------------------------------------------------ */

static int n31_vfl_init(struct whimory *w)
{
	unsigned int cau, i, n, cxt_len;

	n = w->geom.blocks_per_cau;
	if (!n)
		return -EINVAL;
	cxt_len = 16;
	w->vfl.cxt_u16_len = cxt_len;
	w->vfl.bank_stride = 1;
	w->vfl.bank_mask = kvmalloc(n, GFP_KERNEL);
	if (!w->vfl.bank_mask)
		return -ENOMEM;
	memset(w->vfl.bank_mask, (1u << min_t(u32, w->geom.num_cau, 8)) - 1, n);
	w->vfl.cached_vbn = 0xffff;
	w->vfl.cached_n = 0;
	for (cau = 0; cau < w->geom.num_cau; cau++) {
		w->vfl.remap[cau] = kvmalloc_array(n, sizeof(u32), GFP_KERNEL);
		w->vfl.cxt_u16[cau] = kvmalloc_array(cxt_len, sizeof(u16),
						     GFP_KERNEL);
		if (!w->vfl.remap[cau] || !w->vfl.cxt_u16[cau])
			return -ENOMEM;
		for (i = 0; i < n; i++)
			w->vfl.remap[cau][i] = i;
		for (i = 0; i < cxt_len; i++)
			w->vfl.cxt_u16[cau][i] = 0xffff;
		w->vfl.ctx_block[cau] = ~0u;
	}
	return 0;
}

static int n31_vfl_ingest_ctx(struct whimory *w, unsigned int ce,
			      unsigned int cau, unsigned int block,
			      const u8 *page, unsigned int page_len,
			      const u8 *meta)
{
	unsigned int i, loc = 0;
	const u8 *tab;
	bool magic_ok, type_ok;

	if (!page || page_len < 0x200)
		return 0;
	magic_ok = (page[0] == 'w' && page[1] == 'r' && page[2] == 'm' &&
		    page[3] == 'x') ||
		   (page[0] == 'x' && page[1] == 'r' && page[2] == 'm' &&
		    page[3] == 'w') ||
		   (page[FPART_SPECIAL_HDR] == 'w' &&
		    page[FPART_SPECIAL_HDR + 1] == 'r' &&
		    page[FPART_SPECIAL_HDR + 2] == 'm' &&
		    page[FPART_SPECIAL_HDR + 3] == 'x') ||
		   (page[FPART_SPECIAL_HDR] == 'x' &&
		    page[FPART_SPECIAL_HDR + 1] == 'r' &&
		    page[FPART_SPECIAL_HDR + 2] == 'm' &&
		    page[FPART_SPECIAL_HDR + 3] == 'w');
	type_ok = meta && meta[0] == WHIMORY_META_TYPE_VFL_CXT;
	if (!magic_ok && !type_ok)
		return 0;
	if (cau >= w->geom.num_cau || !w->vfl.remap[cau])
		return 0;
	w->vfl.ctx_ce[cau] = ce;
	w->vfl.ctx_block[cau] = block;

	/*
 *: memcpy(cxt_copies, data+0x100, 4 * num_copies).
 * Each record is {le16 phys_block, u8 bank, u8 flags} — VFL CXT
 * copy locations in the tail, not a user virt→phys table.
 * Live glass: first u32 is often 0x827 (block 2087).
 */
	tab = page + 0x100;
	for (i = 0; i < 64 && 0x100 + 4 * (i + 1) <= 0x200; i++) {
		u16 blk = get_unaligned_le16(tab + i * 4);
		u8 bank = tab[i * 4 + 2];

		if (!blk)
			break;
		if (blk < w->geom.blocks_per_cau && bank < w->geom.num_cau)
			loc++;
	}
	w->vfl.cxt_loc_count += loc;

	/*: per-bank u16 CXT copy journal at +0x200 + 32*bank */
	if (page_len >= WHIMORY_VFL_CXT_HDR +
	    WHIMORY_VFL_SPARE_STRIDE * w->geom.num_cau + 2) {
		unsigned int b, j, n16 = w->vfl.cxt_u16_len;

		for (b = 0; b < w->geom.num_cau; b++) {
			if (!w->vfl.cxt_u16[b])
				continue;
			for (j = 0; j < n16; j++) {
				const u8 *src = page + WHIMORY_VFL_CXT_HDR +
						WHIMORY_VFL_SPARE_STRIDE * b +
						2 * j;
				u16 v;

				if (src + 2 > page + page_len)
					break;
				v = get_unaligned_le16(src);
				w->vfl.cxt_u16[b][j] = v;
				if (v != 0xffff && v != WHIMORY_VFL_SPARE_FREE)
					w->vfl.spare_applied++;
			}
		}
	}

	/*
 *bitmap: one byte per VBN (stride 0x8D0D0F0 = 1 on N31),
 * bit = bank. Not in the 0x200 header / spare journal. Try the
 * remainder of this CXT page; reject if any byte has bits outside
 * num_cau (would be unrelated payload).
 */
	{
		unsigned int off = WHIMORY_VFL_CXT_HDR +
				   WHIMORY_VFL_SPARE_STRIDE * w->geom.num_cau;
		u8 def = (1u << min_t(u32, w->geom.num_cau, 8)) - 1;
		unsigned int i, nblk = w->geom.blocks_per_cau;

		if (w->vfl.bank_mask && page_len >= off + nblk && nblk) {
			const u8 *src = page + off;
			bool ok = true;

			for (i = 0; i < nblk; i++) {
				if (src[i] & ~def) {
					ok = false;
					break;
				}
			}
			if (ok) {
				memcpy(w->vfl.bank_mask, src, nblk);
				w->vfl.bitmap_loaded = 1;
				w->vfl.cached_vbn = 0xffff;
			}
		}
	}

	/*
 * User VBN→PBN is identity over blocks_per_cau :
 * vbn < mcxt.dev.blocks_per_cau). Failed-block replacement lives
 * in the u16 tables, not in a 256-entry slice of +0x100.
 */
	w->vfl.remap_count = w->geom.blocks_per_cau;
	dev_info(w->dev,
		 "VFL ingest ce=%u cau=%u blk=%u magic=%d type20=%d cxt_loc=%u identity=%u\n",
		 ce, cau, block, magic_ok, type_ok, loc,
		 w->geom.blocks_per_cau);
	return loc || type_ok || magic_ok;
}

static int n31_vfl_open(struct whimory *w)
{
	u8 *page;
	u8 meta[S5L8740_NAND_META_SIZE];
	unsigned int ce, cau, b, start, pg, slc;
	int hits = 0;

	page = kvmalloc(S5L8740_NAND_PAGE_SIZE, GFP_KERNEL);
	if (!page)
		return -ENOMEM;
	start = w->geom.blocks_per_cau - w->geom.vfl_tail;
	s5l8740_nand_reset();
	for (ce = 0; ce < w->geom.num_ce; ce++) {
		for (cau = 0; cau < w->geom.num_cau; cau++) {
			for (b = start; b < w->geom.blocks_per_cau; b++) {
				for (pg = 0; pg < 8; pg++) {
					int got = -EIO;

					cond_resched();
					for (slc = 0; slc < 2; slc++) {
						got = s5l8740_nand_page_read(ce,
							cau, b, pg, slc, 16,
							page,
							S5L8740_NAND_PAGE_SIZE,
							meta, sizeof(meta));
						if (!got)
							break;
					}
					if (got)
						continue;
					if (n31_vfl_ingest_ctx(w, ce, cau, b,
							       page,
							       S5L8740_NAND_PAGE_SIZE,
							       meta))
						hits++;
				}
			}
		}
	}
	kvfree(page);
	w->vfl.ctx_hits = hits;
	w->vfl_ok = true;
	dev_info(w->dev,
		 "VFL_Open OK ctx_hits=%u remap_ents=%u cxt_loc=%u bitmap=%u spare=%u\n",
		 hits, w->vfl.remap_count, w->vfl.cxt_loc_count,
		 w->vfl.bitmap_loaded, w->vfl.spare_applied);
	return 0;
}

static u32 n31_vfl_get_param(struct whimory *w, u32 selector)
{
	switch (selector) {
	case WHIMORY_VFL_PARAM_NUM_SB:
		return w->geom.num_ce * w->geom.num_cau * w->geom.user_blocks;
	default:
		return 0;
	}
}

static int n31_vfl_read_vba(struct whimory *w, u32 vba, u32 count,
			    void *data, struct whimory_meta *meta)
{
	u32 i, ce, cau, vblock, page, slot, pblock;
	u32 last_ce = ~0u, last_cau = ~0u, last_pblock = ~0u, last_page = ~0u;
	u8 *pagebuf;
	u8 spare[S5L8740_NAND_META_SIZE];
	int ret;

	if (!count || count > WHIMORY_VBAS_PER_PAGE)
		return -EINVAL;
	pagebuf = w->sftl.data_page;
	if (!pagebuf)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		u8 *dst = (u8 *)data + i * WHIMORY_LBA_SIZE;

		ret = whimory_unpack_vba(w, vba + i, &ce, &cau, &vblock,
					 &page, &slot);
		if (ret)
			return ret;
		if (page >= w->sftl.pages_per_sb ||
		    slot >= w->sftl.vbas_per_page)
			return -ERANGE;
		cau = whimory_vfl_bank(w, cau, vblock);
		pblock = whimory_vfl_phys(w, cau, vblock);
		if (ce != last_ce || cau != last_cau || pblock != last_pblock ||
		    page != last_page) {
			ret = whimory_cs_read_page(w, ce, cau, pblock, page,
						   pagebuf,
						   S5L8740_NAND_PAGE_SIZE,
						   spare, sizeof(spare));
			if (ret)
				return ret;
			last_ce = ce;
			last_cau = cau;
			last_pblock = pblock;
			last_page = page;
		}
		memcpy(dst, pagebuf + slot * WHIMORY_LBA_SIZE, WHIMORY_LBA_SIZE);
		if (meta && i == 0) {
			memset(meta, 0xff, sizeof(*meta));
			if (sizeof(spare) >= (slot + 1) * WHIMORY_META_SIZE)
				memcpy(meta, spare + slot * WHIMORY_META_SIZE,
				       sizeof(*meta));
		}
	}
	return 0;
}

static const struct whimory_vfl_ops n31_vfl_ops = {
	.major = 0,
	.minor = n31_vfl_minor,
	.init = n31_vfl_init,
	.open = n31_vfl_open,
	.get_param = n31_vfl_get_param,
	.read_vba = n31_vfl_read_vba,
};

static int whimory_vfl_open(struct whimory *w)
{
	int ret;

	ret = w->vfl_ops->init(w);
	if (ret) {
		dev_err(w->dev, "VFL_Init failed: %d\n", ret);
		return ret;
	}
	ret = w->vfl_ops->open(w);
	if (ret) {
		dev_err(w->dev, "VFL_Open failed: %d\n", ret);
		return ret;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* SFTL recovery — classify SBs, replay BTOC/META by weave */
/* ------------------------------------------------------------------ */

/*: FFFF0001 payload is {count, [lba,span]...} → unmap. */
static int whimory_sftl_apply_list(struct whimory *w, u32 vba)
{
	u8 *buf;
	u32 count, i, lba, span;
	struct whimory_meta meta;
	int ret;

	buf = w->sftl.gc_data;
	if (!buf)
		buf = w->sftl.data_page;
	if (!buf || !w->vfl_ops || !w->vfl_ops->read_vba)
		return -ENOMEM;
	ret = w->vfl_ops->read_vba(w, vba, 1, buf, &meta);
	if (ret)
		return ret;
	count = get_unaligned_le32(buf);
	if (!count || count > (WHIMORY_LBA_SIZE - 4) / 8)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		lba = get_unaligned_le32(buf + 4 + 8 * i);
		span = get_unaligned_le32(buf + 8 + 8 * i);
		if (!span || whimory_special_lba(lba))
			break;
		ret = whimory_l2v_update(w, lba, span, w->l2v.invalid_vba);
		if (ret)
			return ret;
		w->sftl.token_list_applied++;
	}
	return 0;
}

static bool whimory_btoc_looks_be_lpn(const u8 *page)
{
	u32 a = get_unaligned_be32(page);
	u32 b = get_unaligned_be32(page + 4);
	u32 c = get_unaligned_be32(page + 8);
	u32 d = get_unaligned_be32(page + 12);
	unsigned int i, ok = 0;

	if (a < 0x01000000u && b == a + 1 && c == b + 1 && d == c + 1)
		return true;
	if (a == 0 && (b == 1 || b == WHIMORY_VBAS_PER_PAGE) &&
	    c == b + (b == 1 ? 1 : WHIMORY_VBAS_PER_PAGE))
		return true;
	for (i = 0; i < 16; i++) {
		u32 v = get_unaligned_be32(page + i * 4);

		if (v != 0xffffffff && v < 0x01000000u)
			ok++;
	}
	return ok >= 12;
}

/* Live N31 SFTL BTOC: 16-byte BE records, span in last byte (fmss glass). */
static bool whimory_btoc_looks_be_bte(const u8 *page)
{
	u32 weave0, lba0, lba1;
	u32 span0, span1;

	if (whimory_page_blank(page, 64))
		return false;
	weave0 = get_unaligned_be32(page);
	lba0 = get_unaligned_be32(page + 8);
	span0 = page[15];
	if (weave0 != 0 || !span0 || span0 > WHIMORY_DATA_VBAS_PER_SB ||
	    lba0 >= 0x01000000u)
		return false;
	lba1 = get_unaligned_be32(page + 16 + 8);
	span1 = page[16 + 15];
	if (!span1 || span1 > WHIMORY_DATA_VBAS_PER_SB || lba1 >= 0x01000000u)
		return false;
	if (lba1 != lba0 + span0 && lba1 + span1 != lba0 &&
	    (lba1 < lba0 || lba1 > lba0 + span0 + 8))
		return false;
	return true;
}

static bool whimory_btoc_parse_be_lpn(struct whimory *w, const u8 *page,
				      unsigned int len, unsigned int ce,
				      unsigned int cau, unsigned int vblock)
{
	unsigned int i, n, hit = 0, valid = 0;
	bool page_gran;

	n = min_t(unsigned int, len / 4, WHIMORY_DATA_VBAS_PER_SB);
	for (i = 0; i < n; i++) {
		u32 lpn = get_unaligned_be32(page + i * 4);

		if (lpn == 0xffffffff)
			break;
		valid++;
	}
	page_gran = valid > 0 && valid <= WHIMORY_DATA_PAGES_PER_SB;
	if (w->sftl.btoc_pages_valid < 5)
		dev_info(w->dev,
			 "BTOC_BE_LPN valid=%u %s ce=%u cau=%u vblock=%u\n",
			 valid,
			 page_gran ? "page-granularity x4" : "slot-granularity",
			 ce, cau, vblock);

	if (page_gran)
		n = min(valid, (unsigned int)WHIMORY_DATA_PAGES_PER_SB);
	for (i = 0; i < n; i++) {
		u32 lpn = get_unaligned_be32(page + i * 4);
		u32 vba;
		unsigned int slot, pg;

		w->sftl.btoc_entries_seen++;
		if (lpn == 0xffffffff || lpn == WHIMORY_LBA_BLANK)
			continue;
		if (whimory_special_lba(lpn)) {
			w->sftl.token_hole++;
			continue;
		}
		if (lpn >= 0x01000000u)
			continue;
		if (page_gran) {
			for (slot = 0; slot < WHIMORY_VBAS_PER_PAGE; slot++) {
				vba = whimory_pack_vba(w, ce, cau, vblock, i,
						       slot);
				if (whimory_l2v_update(w,
						       lpn * WHIMORY_VBAS_PER_PAGE +
						       slot, 1, vba))
					return hit > 0;
				w->sftl.btoc_l2v_updates++;
				w->sftl.btoc_recs++;
				hit++;
			}
		} else {
			pg = i / w->sftl.vbas_per_page;
			slot = i % w->sftl.vbas_per_page;
			vba = whimory_pack_vba(w, ce, cau, vblock, pg, slot);
			if (whimory_l2v_update(w, lpn, 1, vba))
				break;
			w->sftl.btoc_l2v_updates++;
			w->sftl.btoc_recs++;
			hit++;
		}
	}
	return hit > 0;
}

static bool whimory_btoc_parse_be_bte(struct whimory *w, const u8 *page,
				      unsigned int len, unsigned int ce,
				      unsigned int cau, unsigned int vblock)
{
	unsigned int i, recs, vba_ofs = 0, hit = 0;

	recs = len / 16;
	for (i = 0; i < recs; i++) {
		const u8 *r = page + i * 16;
		u32 lba = get_unaligned_be32(r + 8);
		u32 span = r[15];
		u32 vba;
		int upd;

		w->sftl.btoc_entries_seen++;
		if (!span)
			break;
		if (whimory_special_lba(lba)) {
			if (lba == WHIMORY_LBA_LIST)
				w->sftl.btoc_holelist_ffff0001++;
			else if (lba == WHIMORY_LBA_HOLE)
				w->sftl.btoc_token_ffff0000++;
			else if (lba == WHIMORY_LBA_DELETED)
				w->sftl.btoc_token_ffffff00++;
			else if (lba == WHIMORY_LBA_BLANK)
				w->sftl.btoc_token_ffffffff++;
			w->sftl.token_hole++;
			if (vba_ofs + span > WHIMORY_VBAS_PER_SB)
				break;
			vba_ofs += span;
			continue;
		}
		if (span > WHIMORY_DATA_VBAS_PER_SB || lba >= 0x01000000u)
			break;
		if (vba_ofs + span > WHIMORY_DATA_VBAS_PER_SB)
			break;
		vba = whimory_pack_vba(w, ce, cau, vblock,
				       vba_ofs / w->sftl.vbas_per_page,
				       vba_ofs % w->sftl.vbas_per_page);
		upd = whimory_l2v_update(w, lba, span, vba);
		if (upd)
			break;
		w->sftl.btoc_l2v_updates++;
		vba_ofs += span;
		hit++;
		w->sftl.btoc_recs++;
	}
	return hit > 0;
}

static bool whimory_btoc_parse_bte(struct whimory *w, const u8 *page,
				   unsigned int len, unsigned int ce,
				   unsigned int cau, unsigned int vblock)
{
	unsigned int i, recs, vba_ofs = 0, hit = 0;

	if (len < sizeof(struct whimory_bte) || whimory_page_blank(page, 64))
		return false;

	recs = len / sizeof(struct whimory_bte);
	if (le32_to_cpu(((const struct whimory_bte *)page)->weave_seq_add))
		dev_dbg(w->dev, "BTOC weaveSeqAdd[0] != 0\n");

	for (i = 0; i < recs; i++) {
		const struct whimory_bte *bte =
			(const struct whimory_bte *)(page + i * sizeof(*bte));
		u32 lba = le32_to_cpu(bte->lba);
		u32 span = le32_to_cpu(bte->span);
		u32 vba;
		int upd;

		w->sftl.btoc_entries_seen++;
		if (!span)
			break;
		if (whimory_special_lba(lba)) {
			if (lba == WHIMORY_LBA_LIST) {
				vba = whimory_pack_vba(w, ce, cau, vblock,
						       vba_ofs / w->sftl.vbas_per_page,
						       vba_ofs % w->sftl.vbas_per_page);
				if (whimory_sftl_apply_list(w, vba))
					dev_warn(w->dev,
						 "list token vba=%u failed\n",
						 vba);
				w->sftl.token_list++;
				w->sftl.btoc_holelist_ffff0001++;
			} else if (lba == WHIMORY_LBA_HOLE) {
				w->sftl.btoc_token_ffff0000++;
				w->sftl.token_hole++;
			} else if (lba == WHIMORY_LBA_DELETED) {
				w->sftl.btoc_token_ffffff00++;
				w->sftl.token_hole++;
			} else if (lba == WHIMORY_LBA_BLANK) {
				w->sftl.btoc_token_ffffffff++;
				w->sftl.token_hole++;
			} else
				w->sftl.token_hole++;
			if (vba_ofs + span > WHIMORY_VBAS_PER_SB)
				break;
			vba_ofs += span;
			continue;
		}
		if (span > WHIMORY_DATA_VBAS_PER_SB)
			break;
		if (lba >= 0x01000000u)
			break;
		if (vba_ofs + span > WHIMORY_DATA_VBAS_PER_SB)
			break;
		vba = whimory_pack_vba(w, ce, cau, vblock,
				       vba_ofs / w->sftl.vbas_per_page,
				       vba_ofs % w->sftl.vbas_per_page);
		upd = whimory_l2v_update(w, lba, span, vba);
		if (upd)
			break;
		w->sftl.btoc_l2v_updates++;
		vba_ofs += span;
		hit++;
		w->sftl.btoc_recs++;
	}
	return hit > 0;
}

static int whimory_ingest_btoc_page(struct whimory *w, unsigned int ce,
				    unsigned int cau, unsigned int vblock,
				    const u8 *page, unsigned int len)
{
	const char *verdict = "NONE";
	int hit = 0;

	if (whimory_page_blank(page, 64))
		return 0;
	if (whimory_btoc_looks_be_bte(page)) {
		if (whimory_btoc_parse_be_bte(w, page, len, ce, cau, vblock)) {
			verdict = "BE_BTE";
			hit = 1;
		}
	}
	if (!hit && whimory_btoc_looks_be_lpn(page)) {
		if (whimory_btoc_parse_be_lpn(w, page, len, ce, cau, vblock)) {
			verdict = "BE_LPN_ARRAY";
			hit = 1;
		}
	}
	if (!hit && whimory_btoc_parse_bte(w, page, len, ce, cau, vblock)) {
		verdict = "LE_BTE";
		hit = 1;
	}
	if (w->sftl.btoc_pages_read <= 8)
		dev_info(w->dev,
			 "BTOC_VERDICT ce=%u cau=%u vblock=%u %s first32=%32ph\n",
			 ce, cau, vblock, verdict, page);
	return hit;
}

static int whimory_rebuild_open_sb(struct whimory *w, struct whimory_sb *sb)
{
	unsigned int pg, slot, vblock;
	u8 *data = w->sftl.data_page;
	u8 spare[S5L8740_NAND_META_SIZE];
	int ret, hits = 0;

	vblock = whimory_vfl_virt(w, sb->cau, sb->block);
	for (pg = 0; pg < WHIMORY_DATA_PAGES_PER_SB; pg++) {
		ret = whimory_cs_read_page(w, sb->ce, sb->cau, sb->block, pg,
					   data, S5L8740_NAND_PAGE_SIZE,
					   spare, sizeof(spare));
		if (ret)
			break;
		if (whimory_page_blank(data, 64) &&
		    whimory_page_blank(spare, 16))
			break;
		for (slot = 0; slot < WHIMORY_VBAS_PER_PAGE; slot++) {
			const u8 *m = spare + slot * WHIMORY_META_SIZE;
			u32 lba, vba;

			w->sftl.open_slots_seen++;
			if (m[0] != WHIMORY_META_TYPE_DATA &&
			    m[0] != WHIMORY_META_TYPE_DATA2)
				continue;
			if (m[1] & 0x02)
				continue;
			if (whimory_meta_erased(m, WHIMORY_META_SIZE))
				continue;
			lba = get_unaligned_le32(m + 8);
			w->sftl.open_slots_valid_meta++;
			if (whimory_special_lba(lba) || lba >= 0x01000000u)
				continue;
			if (lba == 0 && w->sftl.open_l2v_updates < 8)
				dev_info(w->dev,
					 "OPEN_META_SCAN lba=0 ce=%u cau=%u blk=%u page=%u slot=%u type=%02x flags=%02x first64=%32ph\n",
					 sb->ce, sb->cau, sb->block, pg, slot,
					 m[0], m[1], data + slot * WHIMORY_LBA_SIZE);
			vba = whimory_pack_vba(w, sb->ce, sb->cau, vblock, pg,
					       slot);
			w->sftl.claim_weave = whimory_weave48(m);
			if (whimory_l2v_update(w, lba, 1, vba)) {
				w->sftl.claim_weave = 0;
				return -ENOMEM;
			}
			w->sftl.claim_weave = 0;
			w->sftl.open_l2v_updates++;
			hits++;
		}
	}
	return hits;
}

static int whimory_sb_cmp(const void *a, const void *b)
{
	const struct whimory_sb *sa = a, *sb = b;

	if (sa->weave < sb->weave)
		return -1;
	if (sa->weave > sb->weave)
		return 1;
	if (sa->ce != sb->ce)
		return sa->ce < sb->ce ? -1 : 1;
	if (sa->cau != sb->cau)
		return sa->cau < sb->cau ? -1 : 1;
	if (sa->block != sb->block)
		return sa->block < sb->block ? -1 : 1;
	return 0;
}

/*analogue: CXT SB VBAs are not L2V_Update'd. */
static bool whimory_vba_is_cxt(struct whimory *w, u32 vba)
{
	u32 ce, cau, vblock, page, slot, phys, i;

	if (whimory_unpack_vba(w, vba, &ce, &cau, &vblock, &page, &slot))
		return false;
	cau = whimory_vfl_bank(w, cau, vblock);
	phys = whimory_vfl_phys(w, cau, vblock);
	if (!w->sftl.sbs)
		return false;
	for (i = 0; i < w->sftl.num_sb; i++) {
		struct whimory_sb *sb = &w->sftl.sbs[i];

		if (sb->kind == WHIMORY_SB_CXT && sb->ce == ce &&
		    sb->cau == cau && sb->block == phys)
			return true;
	}
	return false;
}

static int whimory_cxt_add_base(struct whimory *w, u32 sb, u64 weave)
{
	int i;

	if (w->n_cxt >= WHIMORY_CXT_MAX_SB)
		return -ENOSPC;
	for (i = w->n_cxt; i > 0; i--) {
		if (w->cxt[i - 1].weave >= weave)
			break;
		w->cxt[i] = w->cxt[i - 1];
	}
	w->cxt[i].sb = sb;
	w->cxt[i].weave = weave;
	w->n_cxt++;
	w->sftl.cxt_bases = w->n_cxt;
	return 0;
}

static int whimory_cxt_load_contig(struct whimory *w, const u8 *data,
				   unsigned int len)
{
	u32 lba, span, vba, i, n;

	if (len < 16)
		return 0;
	n = len / 8;
	lba = get_unaligned_le32(data);
	span = get_unaligned_le32(data + 4);
	if (span == 0xffffffff)
		return 0;
	if (span != WHIMORY_CXT_CONTIG_SPAN)
		return -EINVAL;
	if (w->cxt_lba_valid && lba != w->cxt_next_lba) {
		dev_err(w->dev,
			"cxt lba not consecutive want=%u got=%u\n",
			w->cxt_next_lba, lba);
		return -EINVAL;
	}
	w->cxt_lba_valid = true;
	for (i = 1; i < n; i++) {
		vba = get_unaligned_le32(data + 8 * i);
		span = get_unaligned_le32(data + 8 * i + 4);
		if (vba == 0xffffffff || !span)
			break;
		w->sftl.cxt_records_seen++;
		if (vba < w->l2v.invalid_vba && !whimory_vba_is_cxt(w, vba)) {
			if (whimory_l2v_update(w, lba, span, vba))
				return -ENOMEM;
			w->sftl.cxt_l2v_updates++;
		}
		lba += span;
	}
	w->cxt_next_lba = lba;
	return 0;
}

static int whimory_cxt_handle_vba(struct whimory *w, const u8 *data,
				  const u8 *meta)
{
	u8 tag;

	if (meta[0] != WHIMORY_META_TYPE_SFTL_CXT)
		return 0;
	tag = meta[1];
	if (tag == WHIMORY_CXT_TAG_END)
		return 1;
	if (tag != WHIMORY_CXT_TAG_L2V)
		return 0;
	return whimory_cxt_load_contig(w, data, WHIMORY_LBA_SIZE);
}

static int whimory_cxt_load_sb(struct whimory *w, u32 sb_idx)
{
	struct whimory_sftl *s = &w->sftl;
	u32 ce, cau, vblock, page, slot, ofs, vba, pblock;
	u32 last_ce = ~0u, last_cau = ~0u, last_pblock = ~0u, last_page = ~0u;
	u32 zone, n, i;
	u8 *data, *gmeta;
	u8 spare[S5L8740_NAND_META_SIZE];
	int ret, done = 0;

	if (sb_idx >= s->num_sb)
		return -EINVAL;
	w->sftl.cxt_blocks_seen++;
	zone = s->gc_zone_size;
	data = s->gc_data;
	gmeta = s->gc_meta;
	if (!zone || !data || !gmeta || zone % s->vbas_per_page)
		return -ENOMEM;

	w->cxt_lba_valid = false;
	w->cxt_next_lba = 0;
	/*: VFL_Read in chunks of sftl.gc.zoneSize into ED7C/ED80. */
	for (ofs = 0; ofs < s->vbas_per_sb && !done; ofs += zone) {
		n = min(zone, s->vbas_per_sb - ofs);
		for (i = 0; i < n; i++) {
			vba = s_g_addr_to_vba(w, sb_idx, ofs + i);
			ret = whimory_unpack_vba(w, vba, &ce, &cau, &vblock,
						 &page, &slot);
			if (ret)
				return ret;
			cau = whimory_vfl_bank(w, cau, vblock);
			pblock = whimory_vfl_phys(w, cau, vblock);
			if (ce != last_ce || cau != last_cau ||
			    pblock != last_pblock || page != last_page) {
				ret = whimory_cs_read_page(w, ce, cau, pblock,
							   page, s->data_page,
							   S5L8740_NAND_PAGE_SIZE,
							   spare,
							   sizeof(spare));
				if (ret)
					return ret;
				last_ce = ce;
				last_cau = cau;
				last_pblock = pblock;
				last_page = page;
			}
			memcpy(data + i * WHIMORY_LBA_SIZE,
			       s->data_page + slot * WHIMORY_LBA_SIZE,
			       WHIMORY_LBA_SIZE);
			if (sizeof(spare) >= (slot + 1) * WHIMORY_META_SIZE)
				memcpy(gmeta + i * WHIMORY_META_SIZE,
				       spare + slot * WHIMORY_META_SIZE,
				       WHIMORY_META_SIZE);
			else
				memset(gmeta + i * WHIMORY_META_SIZE, 0xff,
				       WHIMORY_META_SIZE);
		}
		for (i = 0; i < n; i++) {
			ret = whimory_cxt_handle_vba(w,
						     data + i * WHIMORY_LBA_SIZE,
						     gmeta + i * WHIMORY_META_SIZE);
			if (ret < 0)
				return ret;
			if (ret > 0) {
				done = 1;
				break;
			}
		}
	}
	return 0;
}

static int whimory_cxt_load(struct whimory *w)
{
	unsigned int i;
	int ret, loaded = 0;

	for (i = 0; i < w->n_cxt; i++) {
		u32 sb = w->cxt[i].sb;

		dev_info(w->dev, "s_cxt_load base sb=%u weave=%llu\n",
			 sb, w->cxt[i].weave);
		w->sftl.claim_weave = w->cxt[i].weave;
		ret = whimory_cxt_load_sb(w, sb);
		w->sftl.claim_weave = 0;
		if (ret) {
			dev_warn(w->dev, "cxt sb=%u failed %d\n", sb, ret);
			continue;
		}
		w->cxt_base_weave = w->cxt[i].weave;
		loaded = 1;
		break;
	}
	w->sftl.cxt_loaded = loaded;
	return 0;
}

static void whimory_note_meta0(struct whimory *w, unsigned int ce,
			       unsigned int cau, unsigned int block,
			       unsigned int page, const u8 *data, const u8 *meta)
{
	unsigned int slot;

	if (!data || !meta)
		return;
	for (slot = 0; slot < WHIMORY_VBAS_PER_PAGE; slot++) {
		const u8 *m = meta + slot * WHIMORY_META_SIZE;
		const u8 *d = data + slot * WHIMORY_LBA_SIZE;
		u32 lba = get_unaligned_le32(m + 8);
		u32 vba, vblock;
		u16 bps;

		if (lba != 0)
			continue;
		if (m[0] != WHIMORY_META_TYPE_DATA &&
		    m[0] != WHIMORY_META_TYPE_DATA2)
			continue;
		w->sftl.meta0_hits++;
		if (w->sftl.meta0_hits > 24)
			continue;
		vblock = whimory_vfl_virt(w, cau, block);
		vba = whimory_pack_vba(w, ce, cau, vblock, page, slot);
		bps = get_unaligned_le16(d + 11);
		dev_info(w->dev,
			 "META0_HIT vba=%u ce=%u cau=%u blk=%u page=%u slot=%u type=%02x first64=%32ph %32ph bps=%u\n",
			 vba, ce, cau, block, page, slot, m[0], d, d + 32, bps);
	}
}

static void whimory_dump_btoc_page(struct whimory *w, const struct whimory_sb *sb,
				   u32 vblock, const u8 *page, const u8 *meta)
{
	u32 be0 = get_unaligned_be32(page);
	u32 be1 = get_unaligned_be32(page + 4);
	u32 be2 = get_unaligned_be32(page + 8);
	bool lpn = whimory_btoc_looks_be_lpn(page);

	dev_info(w->dev,
		 "BTOC_DUMP sb_ce=%u cau=%u blk=%u vblock=%u page=%u first32=%32ph meta0=%16ph be=%u %u %u%s\n",
		 sb->ce, sb->cau, sb->block, vblock, WHIMORY_BTOC_PAGE,
		 page, meta, be0, be1, be2,
		 lpn ? " (BE LPN array)" : "");
}

static void whimory_print_recovery_stats(struct whimory *w)
{
	struct whimory_sftl *s = &w->sftl;

	dev_info(w->dev,
		 "RECOVERY_STATS:\n"
		 "  fpart_sig=%u vfl_ctx_hits=%u vfl_cxt_loc=%u vfl_bitmap=%u\n"
		 "  classified_empty=%u classified_closed=%u classified_open=%u classified_cxt=%u classified_unknown=%u\n"
		 "  cxt_blocks_seen=%u cxt_records_seen=%u cxt_l2v_updates=%u\n"
		 "  btoc_pages_read=%u btoc_pages_valid=%u btoc_entries_seen=%u btoc_l2v_updates=%u\n"
		 "  btoc_token_ffff0000=%u btoc_token_ffffff00=%u btoc_token_ffffffff=%u btoc_holelist_ffff0001=%u\n"
		 "  open_slots_seen=%u open_slots_valid_meta=%u open_l2v_updates=%u\n"
		 "  l2v_update_calls=%u l2v_unmap_calls=%u l2v_repack_roots=%u mapped_lbas=%u mapped_roots=%u meta0_hits=%u\n",
		 w->sig_ok, w->vfl.ctx_hits, w->vfl.cxt_loc_count,
		 w->vfl.bitmap_loaded,
		 s->empty_sbs, s->btoc_sbs, s->open_sbs, s->cxt_sbs,
		 s->unknown_sbs,
		 s->cxt_blocks_seen, s->cxt_records_seen, s->cxt_l2v_updates,
		 s->btoc_pages_read, s->btoc_pages_valid, s->btoc_entries_seen,
		 s->btoc_l2v_updates,
		 s->btoc_token_ffff0000, s->btoc_token_ffffff00,
		 s->btoc_token_ffffffff, s->btoc_holelist_ffff0001,
		 s->open_slots_seen, s->open_slots_valid_meta,
		 s->open_l2v_updates,
		 s->l2v_update_calls, s->l2v_unmap_calls, s->l2v_repack_roots,
		 s->mapped_lbas, s->mapped_roots, s->meta0_hits);
}

static void whimory_scan_closed_meta0(struct whimory *w, unsigned int nsb)
{
	unsigned int i, pg, scanned = 0, cap;
	u8 spare[S5L8740_NAND_META_SIZE];
	u8 *data = w->sftl.data_page;

	cap = meta0_scan_sbs;
	if (!cap || !data)
		return;
	dev_info(w->dev,
		 "META0_SCAN deeper closed SBs=%u pages 0..%u (independent of L2V)\n",
		 cap, WHIMORY_DATA_PAGES_PER_SB - 1);
	for (i = 0; i < nsb && scanned < cap; i++) {
		struct whimory_sb *sb = &w->sftl.sbs[i];

		if (sb->kind != WHIMORY_SB_CLOSED)
			continue;
		scanned++;
		for (pg = 0; pg < WHIMORY_DATA_PAGES_PER_SB; pg++) {
			int ret;

			ret = whimory_cs_read_page(w, sb->ce, sb->cau, sb->block,
						   pg, data,
						   S5L8740_NAND_PAGE_SIZE,
						   spare, sizeof(spare));
			if (ret)
				break;
			whimory_note_meta0(w, sb->ce, sb->cau, sb->block, pg,
					   data, spare);
			if (!(pg & 0x1f))
				cond_resched();
		}
	}
}

static void whimory_dump_vba_page(struct whimory *w, u32 vba)
{
	u32 ce, cau, vblock, page, slot, pblock;
	u8 spare[S5L8740_NAND_META_SIZE];
	u8 *data = w->sftl.data_page;
	int ret;

	if (!data)
		return;
	if (whimory_unpack_vba(w, vba, &ce, &cau, &vblock, &page, &slot)) {
		dev_warn(w->dev, "BAD_VBA unpack failed vba=%u\n", vba);
		return;
	}
	cau = whimory_vfl_bank(w, cau, vblock);
	pblock = whimory_vfl_phys(w, cau, vblock);
	dev_info(w->dev,
		 "BAD_VBA vba=%u sb=%u ofs=%u -> ce=%u cau=%u vblock=%u pbn=%u page=%u map_slot=%u\n",
		 vba, s_g_vba_to_sb(w, vba), s_g_vba_to_ofs(w, vba),
		 ce, cau, vblock, pblock, page, slot);
	ret = whimory_cs_read_page(w, ce, cau, pblock, page, data,
				   S5L8740_NAND_PAGE_SIZE, spare,
				   sizeof(spare));
	if (ret) {
		dev_warn(w->dev, "BAD_VBA page read %d\n", ret);
		return;
	}
	for (slot = 0; slot < WHIMORY_VBAS_PER_PAGE; slot++) {
		const u8 *m = spare + slot * WHIMORY_META_SIZE;
		const u8 *d = data + slot * WHIMORY_LBA_SIZE;
		u32 meta_lba = get_unaligned_le32(m + 8);
		u16 bps = get_unaligned_le16(d + 11);

		dev_info(w->dev,
			 "BAD_VBA slot=%u type=%02x flags=%02x meta_lba=%u bps=%u first64=%32ph %32ph meta=%16ph\n",
			 slot, m[0], m[1], meta_lba, bps, d, d + 32, m);
	}
}

static int whimory_sftl_recover_l2v_from_media(struct whimory *w)
{
	struct whimory_sftl *s = &w->sftl;
	unsigned int ce, cau, b, nscan, nsb = 0, i, open_done = 0;
	u8 meta0[S5L8740_NAND_META_SIZE];
	u8 meta127[S5L8740_NAND_META_SIZE];
	u8 *p127;
	int ret;

	nscan = scan_blocks ? scan_blocks : s->user_blocks;
	if (nscan > s->user_blocks)
		nscan = s->user_blocks;

	p127 = s->btoc_page;
	if (!p127)
		return -ENOMEM;
	s->btoc_dumps_left = 5;

	dev_info(w->dev, "SFTL classify scan ce=%u cau=%u blocks=%u\n",
		 w->geom.num_ce, w->geom.num_cau, nscan);

	for (ce = 0; ce < w->geom.num_ce; ce++) {
		for (cau = 0; cau < w->geom.num_cau; cau++) {
			for (b = 0; b < nscan; b++) {
				struct whimory_sb *sb;
				int r0, r127;

				if (nsb >= s->num_sb)
					goto classify_done;
				if ((b & 0x7f) == 0 && cau == 0 && ce == 0)
					dev_info(w->dev,
						 "SFTL classify ce=%u cau=%u blk=%u/%u nsb=%u\n",
						 ce, cau, b, nscan, nsb);
				r0 = whimory_cs_read_page(w, ce, cau, b, 0,
							  w->sftl.data_page,
							  S5L8740_NAND_PAGE_SIZE,
							  meta0, sizeof(meta0));
				r127 = whimory_cs_read_page(w, ce, cau, b,
							    WHIMORY_BTOC_PAGE,
							    p127,
							    S5L8740_NAND_PAGE_SIZE,
							    meta127,
							    sizeof(meta127));
				if (!r0)
					whimory_note_meta0(w, ce, cau, b, 0,
							   w->sftl.data_page,
							   meta0);
				if (r0 && r127)
					continue;
				if ((!r0 && whimory_page_blank(w->sftl.data_page, 64) &&
				     whimory_meta_erased(meta0, 16)) &&
				    (r127 || (whimory_page_blank(p127, 64) &&
					      whimory_meta_erased(meta127, 16)))) {
					s->empty_sbs++;
					continue;
				}
				sb = &s->sbs[nsb];
				sb->ce = ce;
				sb->cau = cau;
				sb->block = b;
				sb->weave = 0;
				if (!r0 && (whimory_meta_is_data_raw(meta0) ||
					    meta0[0] == WHIMORY_META_TYPE_SFTL_CXT))
					sb->weave = whimory_weave48(meta0);
				if (!r0 && whimory_meta_is_cxt_base(meta0, 0)) {
					u32 vblock = whimory_vfl_virt(w, cau, b);
					u32 sb_idx = whimory_sb_index(w, ce, cau,
								      vblock);

					sb->kind = WHIMORY_SB_CXT;
					s->cxt_sbs++;
					whimory_cxt_add_base(w, sb_idx, sb->weave);
				} else if (!r0 && whimory_meta_slot0_or_any_cxt(meta0)) {
					sb->kind = WHIMORY_SB_CXT;
					s->cxt_sbs++;
				} else if (!r127 && whimory_meta_any_btoc(meta127)) {
					sb->kind = WHIMORY_SB_CLOSED;
					s->btoc_sbs++;
				} else if ((!r0 && whimory_meta_is_data_raw(meta0)) ||
					   (!r127 && whimory_meta_is_data_raw(meta127))) {
					sb->kind = WHIMORY_SB_OPEN;
					s->open_sbs++;
				} else {
					s->unknown_sbs++;
					continue;
				}
				nsb++;
			}
		}
	}
classify_done:
	sort(s->sbs, nsb, sizeof(s->sbs[0]), whimory_sb_cmp, NULL);
	dev_info(w->dev,
		 "SFTL classified nsb=%u closed=%u open=%u cxt=%u empty=%u unknown=%u\n",
		 nsb, s->btoc_sbs, s->open_sbs, s->cxt_sbs, s->empty_sbs,
		 s->unknown_sbs);

	ret = whimory_cxt_load(w);
	if (ret)
		dev_warn(w->dev, "s_cxt_load %d; continuing with BTOC replay\n",
			 ret);
	if (s->cxt_loaded)
		dev_info(w->dev, "s_cxt_load OK bases=%u weave=%llu mapped=%u\n",
			 w->n_cxt, w->cxt_base_weave, s->range_nodes);

	for (i = 0; i < nsb; i++) {
		struct whimory_sb *sb = &s->sbs[i];
		u32 vblock = whimory_vfl_virt(w, sb->cau, sb->block);

		if (sb->kind == WHIMORY_SB_CXT)
			continue;
		if (s->cxt_loaded && sb->weave && sb->weave < w->cxt_base_weave)
			continue;
		if (sb->kind == WHIMORY_SB_CLOSED) {
			int ingested;

			ret = whimory_cs_read_page(w, sb->ce, sb->cau, sb->block,
						   WHIMORY_BTOC_PAGE,
						   s->btoc_page,
						   S5L8740_NAND_PAGE_SIZE,
						   meta127, sizeof(meta127));
			if (ret)
				continue;
			s->btoc_pages_read++;
			if (s->btoc_dumps_left &&
			    (s->btoc_pages_read <= 2 ||
			     whimory_btoc_looks_be_lpn(s->btoc_page))) {
				whimory_dump_btoc_page(w, sb, vblock,
						       s->btoc_page, meta127);
				s->btoc_dumps_left--;
			}
			s->claim_weave = sb->weave;
			ingested = whimory_ingest_btoc_page(w, sb->ce, sb->cau,
							    vblock, s->btoc_page,
							    S5L8740_NAND_PAGE_SIZE);
			s->claim_weave = 0;
			if (ingested)
				s->btoc_pages_valid++;
		} else if (sb->kind == WHIMORY_SB_OPEN) {
			if (max_open_sbs && open_done >= max_open_sbs)
				continue;
			ret = whimory_rebuild_open_sb(w, sb);
			if (ret > 0)
				open_done++;
			else if (ret < 0)
				return ret;
		}
	}

	ret = whimory_l2v_build_from_ranges(w);
	if (ret && s->range_nodes) {
		dev_warn(w->dev,
			 "L2V pack %d; using interval map (%u ranges)\n",
			 ret, s->range_nodes);
		ret = 0;
	} else if (ret) {
		return ret;
	} else {
		s->packed_ok = true;
	}
	w->l2v_ok = true;
	whimory_scan_closed_meta0(w, nsb);
	whimory_print_recovery_stats(w);
	return 0;
}

static int whimory_sftl_alloc(struct whimory *w)
{
	struct whimory_sftl *s = &w->sftl;
	u32 nsb;

	s->vbas_per_page = WHIMORY_VBAS_PER_PAGE;
	s->pages_per_sb = WHIMORY_PAGES_PER_SB;
	s->vbas_per_sb = WHIMORY_VBAS_PER_SB;
	s->user_blocks = w->geom.user_blocks;
	nsb = w->geom.num_ce * w->geom.num_cau * s->user_blocks;
	if (w->vfl_ops && w->vfl_ops->get_param) {
		u32 p = w->vfl_ops->get_param(w, WHIMORY_VFL_PARAM_NUM_SB);

		if (p)
			nsb = p;
	}
	s->num_sb = nsb;
	s->vba_factor_a = nsb;
	s->vba_factor_b = s->vbas_per_sb;
	s->nodepool_bytes = WHIMORY_MIN_NODEPOOL_BYTES;

	s->btoc_page = kvmalloc(S5L8740_NAND_PAGE_SIZE, GFP_KERNEL);
	s->data_page = kvmalloc(S5L8740_NAND_PAGE_SIZE, GFP_KERNEL);
	s->meta_page = kvmalloc(WHIMORY_META_SIZE * WHIMORY_VBAS_PER_PAGE *
				(WHIMORY_DATA_PAGES_PER_SB + 1), GFP_KERNEL);
	s->cs_page = kvmalloc(sizeof(*s->cs_page), GFP_KERNEL);
	s->sbs = kvcalloc(nsb, sizeof(*s->sbs), GFP_KERNEL);
	if (!s->btoc_page || !s->data_page || !s->meta_page || !s->cs_page ||
	    !s->sbs)
		return -ENOMEM;

	/*
 *: max_pages_per_btoc =
 * div(page_bytes + 16 * vbas_per_sb - 1, page_bytes) + 1
 * 16×512 BTE bytes fit in a 16KiB NAND page → 1; OSOS adds 1 → 2.
 */
	{
		u32 page_bytes = w->geom.page_size ?
				 w->geom.page_size : S5L8740_NAND_PAGE_SIZE;
		u32 i;

		s->max_pages_per_btoc =
			(page_bytes + 16 * s->vbas_per_sb - 1) / page_bytes + 1;
		if (!s->max_pages_per_btoc)
			return -EINVAL;
		for (i = 0; i < WHIMORY_BTOC_OPEN; i++) {
			s->btoc_lba[i] = kvmalloc_array(s->vbas_per_sb,
							sizeof(u32),
							GFP_KERNEL);
			if (!s->btoc_lba[i])
				return -ENOMEM;
			memset(s->btoc_lba[i], 0xff,
			       s->vbas_per_sb * sizeof(u32));
		}
	}

	/*
 *: zoneSize starts at 0x8D0EC98 * vbas_per_page and
 * doubles until >= 16. Minimum from the loop is 16; must be a
 * multiple of vbas_per_page. CXT load reads this
 * many VBAs into gc_data / gc_meta.
 */
	s->gc_zone_size = WHIMORY_GC_ZONE_MIN;
	if (s->gc_zone_size % s->vbas_per_page)
		return -EINVAL;
	s->gc_data = kvmalloc((size_t)WHIMORY_LBA_SIZE * s->gc_zone_size,
			      GFP_KERNEL);
	s->gc_meta = kvmalloc((size_t)WHIMORY_META_SIZE * s->gc_zone_size,
			      GFP_KERNEL);
	if (!s->gc_data || !s->gc_meta)
		return -ENOMEM;
	/*
 *full-size FTL: num_superblocks * user VBAs per SB.
 * BTOC page is not host LBA space (DATA_VBAS_PER_SB).
 */
	{
		u64 cap = (u64)nsb * WHIMORY_DATA_VBAS_PER_SB;

		w->total_4k_sectors = cap ? cap : NAND_FTL_DEFAULT_CAPACITY;
	}
	return 0;
}

static int whimory_oracle_load(struct whimory *w)
{
	const struct firmware *gfw = NULL, *rfw = NULL, *nfw = NULL, *sfw = NULL;
	int ret;

	ret = request_firmware(&sfw, WHIMORY_ORACLE_SIG, w->dev);
	if (!ret && sfw && sfw->size >= WHIMORY_SIG_SIZE) {
		ret = whimory_parse_signature(w, sfw->data);
		if (ret)
			dev_err(w->dev, "oracle signature invalid: %d\n", ret);
	}
	if (sfw)
		release_firmware(sfw);

	ret = request_firmware(&gfw, WHIMORY_ORACLE_GLOBALS, w->dev);
	if (ret) {
		dev_err(w->dev, "oracle globals missing: %d\n", ret);
		return ret;
	}
	if (gfw->size < 28) {
		release_firmware(gfw);
		return -EINVAL;
	}
	{
		u32 num_roots = get_unaligned_le32(gfw->data + 0);
		u32 nodepool = get_unaligned_le32(gfw->data + 4);
		u32 max_lba;

		if (gfw->size >= 32)
			max_lba = get_unaligned_le32(gfw->data + 28);
		else
			max_lba = (u32)w->total_4k_sectors;
		if (!num_roots || !nodepool)
			ret = -EINVAL;
		else
			ret = whimory_l2v_init(w, max_lba ? max_lba :
					       (u32)w->total_4k_sectors,
					       w->sftl.vba_factor_a,
					       w->sftl.vba_factor_b,
					       nodepool);
		if (!ret && gfw->size >= 12) {
			w->l2v.bits_vba = gfw->data[8];
			w->l2v.spanbits_vba = gfw->data[9];
			w->l2v.bits_nodeidx = gfw->data[10];
			w->l2v.spanbits_nodeidx = gfw->data[11];
			if (gfw->size >= 16)
				w->l2v.invalid_vba =
					get_unaligned_le32(gfw->data + 12);
			w->l2v.sentinel_vba = w->l2v.invalid_vba;
			if (num_roots && num_roots != w->l2v.num_roots)
				dev_warn(w->dev,
					 "oracle num_roots=%u init=%u\n",
					 num_roots, w->l2v.num_roots);
		}
	}
	release_firmware(gfw);
	if (ret)
		return ret;

	ret = request_firmware(&rfw, WHIMORY_ORACLE_ROOT, w->dev);
	if (ret)
		return ret;
	if (rfw->size < WHIMORY_L2V_ROOT_REC_SIZE * w->l2v.num_roots) {
		release_firmware(rfw);
		return -EINVAL;
	}
	memcpy(w->l2v.root, rfw->data,
	       WHIMORY_L2V_ROOT_REC_SIZE * w->l2v.num_roots);
	release_firmware(rfw);

	ret = request_firmware(&nfw, WHIMORY_ORACLE_NODES, w->dev);
	if (ret)
		return ret;
	if (nfw->size < w->l2v.nodepool_bytes) {
		release_firmware(nfw);
		return -EINVAL;
	}
	memcpy(w->l2v.nodes, nfw->data, w->l2v.nodepool_bytes);
	release_firmware(nfw);

	w->oracle_used = true;
	w->l2v_ok = true;
	whimory_l2v_find_frag(w);
	dev_info(w->dev,
		 "L2V oracle loaded roots=%u nodes=0x%x frag=%u/%u\n",
		 w->l2v.num_roots, w->l2v.nodepool_bytes,
		 w->l2v.frag_count, w->l2v.frag_max);
	return 0;
}

static int n31_sftl_init(struct whimory *w)
{
	if (!w->vfl_ok)
		return -ENODEV;
	w->sftl.vbas_per_page = WHIMORY_VBAS_PER_PAGE;
	w->sftl.pages_per_sb = WHIMORY_PAGES_PER_SB;
	w->sftl.vbas_per_sb = WHIMORY_VBAS_PER_SB;
	w->sftl.user_blocks = w->geom.user_blocks;
	if (!w->sftl.user_blocks || !w->sftl.vbas_per_sb)
		return -EINVAL;
	return 0;
}

static int whimory_l2v_selftest(struct whimory *w)
{
	u32 vba = ~0u, span = 0;
	int fail = 0;

	if (whimory_l2v_update(w, 0, 1, 100) ||
	    whimory_l2v_search(w, 0, &vba, &span) || vba != 100)
		fail++;
	if (whimory_l2v_update(w, 1, 10, 101) ||
	    whimory_l2v_search(w, 5, &vba, &span) || vba != 105)
		fail++;
	if (whimory_l2v_update(w, 0x7fff, 4, 200) ||
	    whimory_l2v_search(w, 0x7fff, &vba, &span) || vba != 200)
		fail++;
	if (whimory_l2v_search(w, 0x8000, &vba, &span) || vba != 201)
		fail++;
	whimory_range_free(w);
	if (w->l2v.root && w->l2v.num_roots)
		memset(w->l2v.root, 0xff,
		       WHIMORY_L2V_ROOT_REC_SIZE * w->l2v.num_roots);
	whimory_l2v_mem_reset(&w->l2v);
	w->sftl.l2v_update_calls = 0;
	w->sftl.l2v_unmap_calls = 0;
	w->sftl.l2v_repack_roots = 0;
	dev_info(w->dev, "L2V_SELFTEST %s\n", fail ? "FAIL" : "OK");
	return fail ? -EINVAL : 0;
}

static int n31_sftl_open(struct whimory *w)
{
	int ret;
	int sess;

	/*
 * OSOS FTL_Open:BTOC (6 slots / 2 open LBA maps),
 *GC zone,block tables,SB
 * state, nodepool ≥ 0x80000,L2V_Init, then s_boot.
 */
	ret = whimory_sftl_alloc(w);
	if (ret)
		return ret;

	if (import_l2v_oracle) {
		ret = whimory_oracle_load(w);
		if (ret) {
			dev_err(w->dev, "L2V oracle load failed: %d\n", ret);
			return ret;
		}
		return 0;
	}

	ret = whimory_l2v_init(w, (u32)w->total_4k_sectors,
			       w->sftl.vba_factor_a, w->sftl.vba_factor_b,
			       w->sftl.nodepool_bytes);
	if (ret)
		return ret;
	whimory_l2v_selftest(w);

	sess = s5l8740_nand_dma_session_begin();
	if (sess && sess != -EBUSY) {
		dev_warn(w->dev, "SFTL recover: DMA session %d\n", sess);
		/* Continue — cs_phys_read may still one-shot arm. */
	}
	ret = whimory_sftl_recover_l2v_from_media(w);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	if (ret)
		return ret;
	return 0;
}

static const struct whimory_ftl_ops n31_sftl_ops = {
	.major = 0,
	.minor = n31_sftl_minor,
	.init = n31_sftl_init,
	.open = n31_sftl_open,
	.read_lba = n31_sftl_read_lba,
};

static int whimory_select_ops(struct whimory *w)
{
	/*
 * OSOS dispatches VFL/FTL by signature major through a table that
 * is not named in the static dump. N31 media is PPN VFL + SFTL;
 * those are the only ops this module implements. Log the majors
 * from the signature (when present) and bind the N31 ops.
 */
	w->vfl_ops = &n31_vfl_ops;
	w->ftl = &n31_sftl_ops;
	if (w->sig_ok) {
		dev_info(w->dev,
			 "VFL_SELECT major=%u minor=%u arg=%u\n",
			 w->sig.vfl_major, w->sig.vfl_minor,
			 w->sig.flags_or_open);
		dev_info(w->dev,
			 "FTL_SELECT major=%u minor=%u\n",
			 w->sig.ftl_major, w->sig.ftl_minor);
		dev_info(w->dev,
			 "ops bound fpart=%u.%u vfl=%u.%u ftl=%u.%u\n",
			 w->sig.fpart_major, w->sig.fpart_minor,
			 w->sig.vfl_major, w->sig.vfl_minor,
			 w->sig.ftl_major, w->sig.ftl_minor);
	} else {
		dev_warn(w->dev,
			 "VFL_SELECT/FTL_SELECT skipped: sig=0, binding N31 PPN+SFTL fallback\n");
	}
	return 0;
}

static int whimory_ftl_open(struct whimory *w)
{
	int ret;

	ret = w->ftl->init(w);
	if (ret) {
		dev_err(w->dev, "FTL_Init failed: %d\n", ret);
		return ret;
	}
	ret = w->ftl->open(w);
	if (ret) {
		dev_err(w->dev, "FTL_Open failed: %d\n", ret);
		return ret;
	}
	w->ftl_ok = true;
	dev_info(w->dev, "FTL_Open OK\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Read path */
/* ------------------------------------------------------------------ */

static int whimory_validate_meta(struct whimory *w,
				 const struct whimory_meta *m,
				 u32 expected_lba)
{
	u32 meta_lba = le32_to_cpu(m->lba);

	if (!whimory_meta_is_user_data(m)) {
		dev_err(w->dev,
			"sftl non-data meta want=0x%x type=%02x flags=%02x lba=0x%x\n",
			expected_lba, m->type, m->flags, meta_lba);
		return -EIO;
	}

	if (meta_lba != expected_lba) {
		dev_err(w->dev,
			"sftl lba mismatch want=0x%x meta=0x%x type=%02x flags=%02x\n",
			expected_lba, meta_lba, m->type, m->flags);
		return -EIO;
	}

	if (m->flags & 0x02) {
		dev_err(w->dev,
			"sftl uECC flag lba=0x%x type=%02x flags=%02x\n",
			expected_lba, m->type, m->flags);
		return -EIO;
	}

	return 0;
}

static int n31_sftl_read_lba(struct whimory *w, u32 lba, void *buf,
			     bool allow_blank)
{
	struct whimory_meta meta;
	u32 vba = 0, span = 0;
	int ret;

	if (!w->l2v_ok)
		return -ENODEV;
	ret = whimory_l2v_search(w, lba, &vba, &span);
	if (ret) {
		if (allow_blank && ret == -ENOENT) {
			memset(buf, 0xff, WHIMORY_LBA_SIZE);
			return 0;
		}
		return ret;
	}
	if (lba == 0)
		w->lba0_vba = vba;
	if (vba >= w->l2v.invalid_vba) {
		if (!allow_blank)
			return -ENOENT;
		memset(buf, 0xff, WHIMORY_LBA_SIZE);
		return 0;
	}
	if (lba == 0) {
		dev_info(w->dev, "L2V lookup LBA0 -> VBA=%u span=%u\n",
			 vba, span);
		whimory_dump_vba_page(w, vba);
	}
	ret = w->vfl_ops->read_vba(w, vba, 1, buf, &meta);
	if (ret)
		return ret;
	ret = whimory_validate_meta(w, &meta, lba);
	if (!ret)
		dev_dbg(w->dev, "meta OK lba=%u vba=%u type=%02x\n",
			lba, vba, meta.type);
	return ret;
}

static int whimory_read_lba_4k(struct whimory *w, u32 lba, void *buf)
{
	return n31_sftl_read_lba(w, lba, buf, true);
}

static int whimory_ftl_read_hook(u64 lba, void *buf)
{
	struct whimory *w = whimory_dev;

	if (!w || !w->l2v_ok)
		return -ENODEV;
	if (lba >= w->total_4k_sectors)
		return -ERANGE;
	return whimory_read_lba_4k(w, (u32)lba, buf);
}

static int whimory_check_lba0(struct whimory *w)
{
	u8 *buf;
	int ret;
	u16 bps;
	u32 total32, rootclus, serial;

	buf = kzalloc(WHIMORY_LBA_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = n31_sftl_read_lba(w, 0, buf, false);
	if (ret) {
		dev_err(w->dev, "LBA0 read failed: %d\n", ret);
		goto out;
	}

	dev_info(w->dev, "LBA0 first32=%32ph\n", buf);
	dev_info(w->dev, "LBA0 next32=%32ph\n", buf + 32);

	bps = get_unaligned_le16(buf + 11);
	total32 = get_unaligned_le32(buf + 32);
	rootclus = get_unaligned_le32(buf + 44);
	serial = get_unaligned_le32(buf + 67);
	dev_info(w->dev,
		 "BPB bytes_per_sector=%u sectors_per_cluster=%u total32=%u rootclus=%u serial=%08x label=%.11s\n",
		 bps, buf[13], total32, rootclus, serial, buf + 71);

	if (bps != 4096) {
		ret = -EINVAL;
		goto out;
	}
	if (buf[0] != 0xeb && buf[0] != 0xe9) {
		ret = -EINVAL;
		goto out;
	}
	dev_info(w->dev, "meta OK lba=0\n");
	w->lba0_ok = true;
	if (total32)
		w->total_4k_sectors = total32;
out:
	kfree(buf);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Block device */
/* ------------------------------------------------------------------ */

static void whimory_submit_bio_range(struct bio *bio, u64 start_4k,
				     u64 n_4k)
{
	struct whimory *w = whimory_dev;
	struct bvec_iter iter;
	struct bio_vec bvec;
	sector_t sector = bio->bi_iter.bi_sector;
	int ret = 0;

	if (!w || !w->lba0_ok) {
		bio_io_error(bio);
		return;
	}
	if (op_is_write(bio_op(bio))) {
		bio_io_error(bio);
		return;
	}

	bio_for_each_segment(bvec, bio, iter) {
		u8 *dst = kmap_local_page(bvec.bv_page) + bvec.bv_offset;
		unsigned int done_bytes = 0;

		while (done_bytes < bvec.bv_len) {
			u32 lba4k = (u32)(start_4k + (sector >> 3));
			unsigned int off = (sector & 7) * 512;
			unsigned int n = min_t(unsigned int,
					       bvec.bv_len - done_bytes,
					       4096 - off);

			if ((u64)lba4k >= start_4k + n_4k ||
			    lba4k >= w->total_4k_sectors) {
				ret = -EIO;
				kunmap_local(dst);
				goto done;
			}
			mutex_lock(&w->bounce_lock);
			ret = whimory_read_lba_4k(w, lba4k, w->bounce);
			if (!ret)
				memcpy(dst + done_bytes, w->bounce + off, n);
			mutex_unlock(&w->bounce_lock);
			if (ret) {
				kunmap_local(dst);
				goto done;
			}
			done_bytes += n;
			sector += n >> 9;
		}
		kunmap_local(dst);
	}
done:
	if (ret)
		bio_io_error(bio);
	else
		bio_endio(bio);
}

static void whimory_submit_bio(struct bio *bio)
{
	struct whimory *w = whimory_dev;

	whimory_submit_bio_range(bio, 0, w ? w->total_4k_sectors : 0);
}

static void whimory_ipod_submit_bio(struct bio *bio)
{
	struct whimory *w = whimory_dev;

	whimory_submit_bio_range(bio, 0, w ? w->total_4k_sectors : 0);
}

static const struct block_device_operations whimory_bd_ops = {
	.owner = THIS_MODULE,
	.submit_bio = whimory_submit_bio,
};

static const struct block_device_operations whimory_ipod_ops = {
	.owner = THIS_MODULE,
	.submit_bio = whimory_ipod_submit_bio,
};

static struct gendisk *whimory_alloc_disk(struct whimory *w, const char *name,
					  const struct block_device_operations *ops)
{
	struct queue_limits lim = {
		.logical_block_size = WHIMORY_LBA_SIZE,
		.physical_block_size = WHIMORY_LBA_SIZE,
	};
	struct gendisk *gd;
	int ret;

	gd = blk_alloc_disk(&lim, NUMA_NO_NODE);
	if (IS_ERR(gd))
		return gd;
	gd->first_minor = 0;
	gd->flags = GENHD_FL_NO_PART;
	gd->fops = ops;
	gd->private_data = w;
	snprintf(gd->disk_name, DISK_NAME_LEN, "%s", name);
	set_capacity(gd, w->total_4k_sectors * (WHIMORY_LBA_SIZE / 512));
	set_disk_ro(gd, 1);
	ret = add_disk(gd);
	if (ret) {
		put_disk(gd);
		return ERR_PTR(ret);
	}
	return gd;
}

static int whimory_register_disk(struct whimory *w)
{
	struct gendisk *gd;

	if (!w->sig_ok) {
		dev_warn(w->dev,
			 "sig=0: native PASS requires FPart 0xC101 xrmw signature\n");
		if (!allow_sigless_debug)
			return -ENODEV;
	}
	if (!w->vfl_ok || !w->ftl_ok || !w->l2v_ok || !w->lba0_ok)
		return -ENODEV;

	gd = whimory_alloc_disk(w, FTL_DISK_NAME, &whimory_bd_ops);
	if (IS_ERR(gd))
		return PTR_ERR(gd);
	w->disk = gd;
	gd = whimory_alloc_disk(w, FTL_IPOD_NAME, &whimory_ipod_ops);
	if (!IS_ERR(gd))
		w->ipod_disk = gd;
	s5l8740_nand_register_ftl_read(whimory_ftl_read_hook);
	dev_info(w->dev,
		 "/dev/%s registered read-only (%llu x %uB)\n",
		 FTL_DISK_NAME, w->total_4k_sectors, WHIMORY_LBA_SIZE);
	return 0;
}

static void whimory_unregister_disk(struct whimory *w)
{
	s5l8740_nand_register_ftl_read(NULL);
	if (w->ipod_disk) {
		del_gendisk(w->ipod_disk);
		put_disk(w->ipod_disk);
		w->ipod_disk = NULL;
	}
	if (w->disk) {
		del_gendisk(w->disk);
		put_disk(w->disk);
		w->disk = NULL;
	}
}

static ssize_t whimory_status_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct whimory *w = whimory_dev;
	int meta_ok = s5l8740_nand_meta_transport_ok();

	if (!w)
		return sysfs_emit(buf, "no device\n");
	return sysfs_emit(buf,
			  "fil=%d sig=%d vfl=%d ftl=%d l2v=%d lba0=%d oracle=%d\n"
			  "meta_transport=%s cs_dma_safe=%d pio_meta_trusted=0 "
			  "disk_gate=%s\n"
			  "mapped_roots=%u mapped_lbas=%u btoc_sbs=%u open_sbs=%u cxt_sbs=%u empty=%u recs=%u cxt_loaded=%d packed=%d\n"
			  "lba0_vba=%u cap=%llu vbas_per_sb=%u hole=%u list=%u\n"
			  "spare_applied=%u bitmap=%u frag=%u/%u gc_zone=%u btoc_pages=%u updates=%u gen=%u free=%u list_unmapped=%u\n%s\n",
			  w->fil_ok, w->sig_ok, w->vfl_ok, w->ftl_ok,
			  w->l2v_ok, w->lba0_ok, w->oracle_used,
			  meta_ok ? "enabled" : "disabled",
			  meta_ok ? 1 : 0,
			  w->disk ? "registered" :
			  (meta_ok ? "blocked_open" : "blocked_cs_phys_only"),
			  w->sftl.mapped_roots, w->sftl.mapped_lbas,
			  w->sftl.btoc_sbs, w->sftl.open_sbs, w->sftl.cxt_sbs,
			  w->sftl.empty_sbs, w->sftl.btoc_recs,
			  w->sftl.cxt_loaded, w->sftl.packed_ok,
			  w->lba0_vba, w->total_4k_sectors, w->sftl.vbas_per_sb,
			  w->sftl.token_hole, w->sftl.token_list,
			  w->vfl.spare_applied, w->vfl.bitmap_loaded,
			  w->l2v.frag_count, w->l2v.frag_max,
			  w->sftl.gc_zone_size, w->sftl.max_pages_per_btoc,
			  w->l2v.updates, w->l2v.gen, w->l2v.free_count,
			  w->sftl.token_list_applied,
			  w->status);
}
static DEVICE_ATTR_RO(whimory_status);

static struct attribute *ftl_attrs[] = {
	&dev_attr_whimory_status.attr,
	NULL,
};
static const struct attribute_group ftl_attr_group = {
	.attrs = ftl_attrs,
};

static void whimory_free(struct whimory *w)
{
	unsigned int cau;

	if (!w)
		return;
	whimory_unregister_disk(w);
	whimory_range_free(w);
	whimory_l2v_free(w);
	kvfree(w->sftl.btoc_page);
	kvfree(w->sftl.data_page);
	kvfree(w->sftl.meta_page);
	kvfree(w->sftl.cs_page);
	kvfree(w->sftl.sbs);
	kvfree(w->sftl.gc_data);
	kvfree(w->sftl.gc_meta);
	kvfree(w->vfl.bank_mask);
	for (cau = 0; cau < WHIMORY_BTOC_OPEN; cau++)
		kvfree(w->sftl.btoc_lba[cau]);
	for (cau = 0; cau < S5L8740_NAND_MAX_CAU; cau++) {
		kvfree(w->vfl.remap[cau]);
		kvfree(w->vfl.cxt_u16[cau]);
	}
	kfree(w->bounce);
	kfree(w);
}

static int whimory_open_stack(struct whimory *w)
{
	int ret;

	ret = whimory_fil_init(w);
	if (ret) {
		whimory_set_status(w, "FIL_Init failed %d", ret);
		return ret;
	}

	/*
	 * Without CS metadata DMA, classic Whimory open cannot validate
	 * META via page_read. Recover is available via CS phys reads:
	 * echo 1 > .../ftl_sftl_recover  (binds csmap disks to L2V).
	 */
	if (!s5l8740_nand_meta_transport_ok()) {
		whimory_set_status(w,
				   "CS metadata DMA disabled; "
				   "use ftl_sftl_recover (CS META path) "
				   "or meta_dma_read=1");
		pr_info("s5l8740-ftl: Whimory auto-open deferred "
			"(meta_dma_read=0); run ftl_sftl_recover for "
			"CXT→BTOC→L2V on CS META\n");
		return -EOPNOTSUPP;
	}

	ret = whimory_read_signature(w);
	if (ret) {
		whimory_set_status(w, "signature failed %d", ret);
		return ret;
	}
	ret = whimory_select_ops(w);
	if (ret)
		return ret;
	ret = whimory_vfl_open(w);
	if (ret) {
		whimory_set_status(w, "VFL_Open failed %d", ret);
		return ret;
	}
	ret = whimory_ftl_open(w);
	if (ret) {
		whimory_set_status(w, "FTL_Open failed %d", ret);
		return ret;
	}
	ret = whimory_check_lba0(w);
	if (ret) {
		whimory_set_status(w, "LBA0 check failed %d", ret);
		return ret;
	}
	ret = whimory_register_disk(w);
	if (ret) {
		whimory_set_status(w, "disk register failed %d", ret);
		return ret;
	}
	whimory_set_status(w, "ready");
	return 0;
}

bool whimory_l2v_ready(void)
{
	return whimory_dev && whimory_dev->l2v_ok;
}

int whimory_read_fmss_lba(u32 lba, void *buf)
{
	struct whimory *w = whimory_dev;
	int sess, ret;

	if (!w || !buf)
		return -EINVAL;
	if (!w->l2v_ok || !w->ftl || !w->ftl->read_lba)
		return -ENODEV;
	sess = s5l8740_nand_dma_session_begin();
	mutex_lock(&w->tree_lock);
	ret = w->ftl->read_lba(w, lba, buf, false);
	mutex_unlock(&w->tree_lock);
	if (sess == 0)
		s5l8740_nand_dma_session_end();
	return ret;
}

int whimory_range_walk(int (*fn)(u32 start, u32 len, u32 vba, u64 weave,
				 void *ctx),
		       void *ctx)
{
	struct whimory *w = whimory_dev;
	struct rb_node *n;
	struct whimory_range *snap;
	unsigned int i, count = 0;
	int ret = 0;

	if (!w || !fn)
		return -EINVAL;

	mutex_lock(&w->tree_lock);
	count = w->sftl.range_nodes;
	if (!count) {
		mutex_unlock(&w->tree_lock);
		return 0;
	}
	snap = kvmalloc_array(count, sizeof(*snap), GFP_KERNEL);
	if (!snap) {
		mutex_unlock(&w->tree_lock);
		return -ENOMEM;
	}
	i = 0;
	for (n = rb_first(&w->ranges); n && i < count; n = rb_next(n)) {
		struct whimory_range *r = rb_entry(n, struct whimory_range, rb);

		snap[i].start = r->start;
		snap[i].len = r->len;
		snap[i].vba = r->vba;
		snap[i].weave = r->weave;
		i++;
	}
	count = i;
	mutex_unlock(&w->tree_lock);

	for (i = 0; i < count; i++) {
		ret = fn(snap[i].start, snap[i].len, snap[i].vba,
			 snap[i].weave, ctx);
		if (ret)
			break;
	}
	kvfree(snap);
	return ret;
}

int whimory_l2v_search_phys(u32 lba, u8 *ce, u8 *cau, u16 *blk, u8 *page,
			    u8 *slot, u64 *weave)
{
	struct whimory *w = whimory_dev;
	u32 vba = ~0u, span = 0, vce, vcau, vblock, vpage, vslot, pblock;
	struct whimory_range *r;
	int ret;

	if (!w || !w->l2v_ok)
		return -ENODEV;
	mutex_lock(&w->tree_lock);
	ret = whimory_l2v_search(w, lba, &vba, &span);
	if (ret || vba >= w->l2v.invalid_vba) {
		mutex_unlock(&w->tree_lock);
		return ret ? ret : -ENOENT;
	}
	r = whimory_range_find(&w->ranges, lba);
	if (weave)
		*weave = r ? r->weave : 0;
	ret = whimory_unpack_vba(w, vba, &vce, &vcau, &vblock, &vpage, &vslot);
	if (ret) {
		mutex_unlock(&w->tree_lock);
		return ret;
	}
	vcau = whimory_vfl_bank(w, vcau, vblock);
	pblock = whimory_vfl_phys(w, vcau, vblock);
	mutex_unlock(&w->tree_lock);
	if (ce)
		*ce = (u8)vce;
	if (cau)
		*cau = (u8)vcau;
	if (blk)
		*blk = (u16)pblock;
	if (page)
		*page = (u8)vpage;
	if (slot)
		*slot = (u8)vslot;
	return 0;
}

int whimory_sftl_recover_cs(void)
{
	struct whimory *w = whimory_dev;
	int ret, sess;

	if (!w)
		return -ENODEV;

	ret = whimory_fil_init(w);
	if (ret) {
		whimory_set_status(w, "FIL_Init failed %d", ret);
		return ret;
	}

	ret = whimory_read_signature(w);
	if (ret) {
		dev_warn(w->dev,
			 "signature %d; CS recover continues (identity VFL)\n",
			 ret);
	}

	ret = whimory_select_ops(w);
	if (ret)
		return ret;

	if (!w->vfl_ok) {
		ret = whimory_vfl_open(w);
		if (ret) {
			whimory_set_status(w, "VFL_Open failed %d", ret);
			return ret;
		}
	}

	if (!w->ftl_ok) {
		ret = whimory_ftl_open(w);
		if (ret) {
			whimory_set_status(w, "FTL_Open/recover failed %d",
					   ret);
			return ret;
		}
	} else {
		/* Re-run recover on CS META (clear prior L2V). */
		whimory_range_free(w);
		if (w->l2v.root && w->l2v.num_roots)
			memset(w->l2v.root, 0xff,
			       WHIMORY_L2V_ROOT_REC_SIZE * w->l2v.num_roots);
		whimory_l2v_mem_reset(&w->l2v);
		w->l2v_ok = false;
		w->sftl.cxt_loaded = false;
		w->sftl.packed_ok = false;
		w->n_cxt = 0;
		w->cxt_base_weave = 0;
		w->sftl.btoc_sbs = 0;
		w->sftl.open_sbs = 0;
		w->sftl.empty_sbs = 0;
		w->sftl.cxt_sbs = 0;
		w->sftl.unknown_sbs = 0;
		w->sftl.btoc_pages_read = 0;
		w->sftl.btoc_pages_valid = 0;
		w->sftl.btoc_entries_seen = 0;
		w->sftl.btoc_l2v_updates = 0;
		w->sftl.open_slots_seen = 0;
		w->sftl.open_slots_valid_meta = 0;
		w->sftl.open_l2v_updates = 0;
		w->sftl.range_nodes = 0;
		w->sftl.cxt_l2v_updates = 0;

		sess = s5l8740_nand_dma_session_begin();
		if (sess && sess != -EBUSY)
			dev_warn(w->dev, "re-recover DMA session %d\n", sess);
		ret = whimory_sftl_recover_l2v_from_media(w);
		if (sess == 0)
			s5l8740_nand_dma_session_end();
		if (ret) {
			whimory_set_status(w, "re-recover failed %d", ret);
			return ret;
		}
	}

	if (!w->l2v_ok) {
		whimory_set_status(w, "recover OK but l2v_ok=0");
		return -EIO;
	}
	whimory_set_status(w,
			   "CS recover OK mapped_ranges=%u btoc=%u open=%u",
			   w->sftl.range_nodes, w->sftl.btoc_pages_valid,
			   w->sftl.open_l2v_updates);
	dev_info(w->dev, "%s\n", w->status);
	return 0;
}

static int __init ftl_init(void)
{
	struct whimory *w;
	int ret;

	if (!s5l8740_nand_available()) {
		pr_err("s5l8740-ftl: load nand_s5l8740 first\n");
		return -ENODEV;
	}

	w = kzalloc(sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;
	w->dev = nand_ftl_device();
	mutex_init(&w->bounce_lock);
	mutex_init(&w->tree_lock);
	w->ranges = RB_ROOT;
	w->bounce = kzalloc(WHIMORY_LBA_SIZE, GFP_KERNEL);
	if (!w->bounce) {
		kfree(w);
		return -ENOMEM;
	}
	w->total_4k_sectors = NAND_FTL_DEFAULT_CAPACITY;
	whimory_dev = w;

	ftl_pdev = platform_device_register_simple("s5l8740-ftl", -1, NULL, 0);
	if (IS_ERR(ftl_pdev)) {
		ret = PTR_ERR(ftl_pdev);
		ftl_pdev = NULL;
		whimory_free(w);
		whimory_dev = NULL;
		return ret;
	}
	w->pdev = ftl_pdev;
	w->dev = &ftl_pdev->dev;
	ret = sysfs_create_group(&ftl_pdev->dev.kobj, &ftl_attr_group);
	if (ret) {
		platform_device_unregister(ftl_pdev);
		whimory_free(w);
		whimory_dev = NULL;
		ftl_pdev = NULL;
		return ret;
	}

	ret = ftl_s5l8740_csmap_init(&ftl_pdev->dev);
	if (ret)
		dev_warn(&ftl_pdev->dev, "CS map init failed %d\n", ret);

	ret = whimory_open_stack(w);
	if (ret) {
		dev_err(w->dev,
			"Whimory open failed (%d) — NOT registering /dev/%s (fil=%d sig=%d vfl=%d ftl=%d l2v=%d lba0=%d)\n",
			ret, FTL_DISK_NAME, w->fil_ok, w->sig_ok, w->vfl_ok,
			w->ftl_ok, w->l2v_ok, w->lba0_ok);
		/*
 * Keep the platform device so sysfs status is visible.
 * The block disk is absent until LBA0 works.
 */
		return 0;
	}
	return 0;
}

static void __exit ftl_exit(void)
{
	struct whimory *w = whimory_dev;

	if (ftl_pdev) {
		ftl_s5l8740_csmap_exit(&ftl_pdev->dev);
		sysfs_remove_group(&ftl_pdev->dev.kobj, &ftl_attr_group);
		platform_device_unregister(ftl_pdev);
		ftl_pdev = NULL;
	}
	whimory_dev = NULL;
	whimory_free(w);
}

module_init(ftl_init);
module_exit(ftl_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("S5L8740 Whimory PPN SFTL read-only block driver");
MODULE_AUTHOR("n31");
MODULE_SOFTDEP("pre: nand_s5l8740");
MODULE_FIRMWARE(WHIMORY_ORACLE_SIG);
MODULE_FIRMWARE(WHIMORY_ORACLE_ROOT);
MODULE_FIRMWARE(WHIMORY_ORACLE_NODES);
MODULE_FIRMWARE(WHIMORY_ORACLE_GLOBALS);
