/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef FTL_S5L8740_VECMAP_H
#define FTL_S5L8740_VECMAP_H

#include <linux/types.h>

#define N31_VEC_GROUP_SHIFT		8	/* 256 entries per group */
#define N31_VEC_GROUP_SIZE		(1u << N31_VEC_GROUP_SHIFT)
#define N31_VEC_MISS			127	/* unknown / missing */
#define N31_VEC_ESC			(-128)	/* escape to full table */
#define N31_INVALID_P			0xffffffffu
#define N31_INVALID_L			0xffffffffu

#define N31_VEC_NUM_CE			2u
#define N31_VEC_NUM_CAU			2u
#define N31_VEC_BLOCKS_PER_CAU		2088u
#define N31_VEC_PAGES_PER_BLOCK		128u
#define N31_VEC_SLOTS_PER_PAGE		4u

struct n31_vec_pair {
	u32 l;		/* fmss_lba */
	u32 p;		/* physical record ordinal */
	u32 weave;
};

struct n31_vec_escape {
	u32 key;
	u32 value;
	u32 weave;
};

struct n31_vecmap {
	/* L2V: L -> P */
	u32 l_base;
	u32 l_count;
	u32 *l2v_base_p;	/* per group */
	s8 *l2v_delta;		/* per L index */
	u32 l2v_groups;

	/* V2L: P -> L */
	u32 p_base;
	u32 p_count;
	u32 *v2l_base_l;
	s8 *v2l_delta;
	u32 v2l_groups;

	struct n31_vec_escape *l2v_esc;
	unsigned int l2v_esc_n;
	unsigned int l2v_esc_cap;
	struct n31_vec_escape *v2l_esc;
	unsigned int v2l_esc_n;
	unsigned int v2l_esc_cap;

	unsigned int compact_ok;
	unsigned int compact_esc;
	unsigned int compact_miss;
	bool ready;
};

static inline u32 n31_phys_to_ordinal(u8 ce, u8 cau, u16 blk, u8 page, u8 slot)
{
	u32 bank = (u32)ce * N31_VEC_NUM_CAU + cau;
	u32 page_i = (u32)blk * N31_VEC_PAGES_PER_BLOCK + page;

	return ((bank * N31_VEC_BLOCKS_PER_CAU *
		 N31_VEC_PAGES_PER_BLOCK + page_i) *
		N31_VEC_SLOTS_PER_PAGE) + (slot & 3);
}

static inline void n31_ordinal_to_phys(u32 p, u8 *ce, u8 *cau, u16 *blk,
				       u8 *page, u8 *slot)
{
	u32 slot_i = p % N31_VEC_SLOTS_PER_PAGE;
	u32 page_i = p / N31_VEC_SLOTS_PER_PAGE;
	u32 pages_per_bank = N31_VEC_BLOCKS_PER_CAU * N31_VEC_PAGES_PER_BLOCK;
	u32 bank = page_i / pages_per_bank;
	u32 rem = page_i % pages_per_bank;

	*slot = (u8)slot_i;
	*page = (u8)(rem % N31_VEC_PAGES_PER_BLOCK);
	*blk = (u16)(rem / N31_VEC_PAGES_PER_BLOCK);
	*cau = (u8)(bank % N31_VEC_NUM_CAU);
	*ce = (u8)(bank / N31_VEC_NUM_CAU);
}

void n31_vecmap_init(struct n31_vecmap *v);
void n31_vecmap_free(struct n31_vecmap *v);

/*
 * Build L2V/V2L from newest-weave pairs (caller already chose weave).
 * Allocates delta tables covering [l_min..l_max] and [p_min..p_max].
 */
int n31_vecmap_build(struct n31_vecmap *v, const struct n31_vec_pair *pairs,
		     unsigned int n);

int n31_vecmap_lookup(const struct n31_vecmap *v, u32 lba, u32 *p_out);
u32 n31_vecmap_v2l(const struct n31_vecmap *v, u32 p);

#endif /* FTL_S5L8740_VECMAP_H */
