// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dual vector LBA maps for S5L8740 FTL.
 *
 * L2V maps logical LBA → physical record ordinal; V2L maps the reverse.
 * Each axis uses a u32 group anchor (256 entries) plus an s8 residual.
 * Sentinels: 127 = missing, -128 = sparse escape. Callers must still
 * validate on-media metadata after a successful cross-check.
 */
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#include "ftl-s5l8740-vecmap.h"

void n31_vecmap_init(struct n31_vecmap *v)
{
	memset(v, 0, sizeof(*v));
}

void n31_vecmap_free(struct n31_vecmap *v)
{
	if (!v)
		return;
	vfree(v->l2v_base_p);
	vfree(v->l2v_delta);
	vfree(v->v2l_base_l);
	vfree(v->v2l_delta);
	kfree(v->l2v_esc);
	kfree(v->v2l_esc);
	memset(v, 0, sizeof(*v));
}

static int n31_s32_cmp(const void *a, const void *b)
{
	s32 da = *(const s32 *)a;
	s32 db = *(const s32 *)b;

	return da < db ? -1 : da > db ? 1 : 0;
}

static s32 n31_median_s32(s32 *tmp, unsigned int n)
{
	if (!n)
		return 0;
	sort(tmp, n, sizeof(*tmp), n31_s32_cmp, NULL);
	return tmp[n / 2];
}

static int n31_esc_add(struct n31_vec_escape **arr, unsigned int *n,
		       unsigned int *cap, u32 key, u32 value, u32 weave)
{
	struct n31_vec_escape *e;
	unsigned int i;

	for (i = 0; i < *n; i++) {
		if ((*arr)[i].key == key) {
			(*arr)[i].value = value;
			(*arr)[i].weave = weave;
			return 0;
		}
	}
	if (*n >= *cap) {
		unsigned int ncap = *cap ? *cap * 2 : 64;
		struct n31_vec_escape *na =
			krealloc(*arr, ncap * sizeof(*e), GFP_KERNEL);

		if (!na)
			return -ENOMEM;
		*arr = na;
		*cap = ncap;
	}
	e = &(*arr)[(*n)++];
	e->key = key;
	e->value = value;
	e->weave = weave;
	return 0;
}

static int n31_esc_find(const struct n31_vec_escape *arr, unsigned int n,
			u32 key, u32 *value)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (arr[i].key == key) {
			*value = arr[i].value;
			return 0;
		}
	}
	return -ENOENT;
}

static int n31_compress_axis(u32 *base_out, s8 *delta_out, u32 count,
			     u32 groups, bool is_l2v,
			     const struct n31_vec_pair *pairs, unsigned int np,
			     u32 key_base,
			     struct n31_vec_escape **esc, unsigned int *esc_n,
			     unsigned int *esc_cap,
			     unsigned int *ok, unsigned int *escapes,
			     unsigned int *miss)
{
	s32 *scratch;
	unsigned int g;

	scratch = kmalloc_array(N31_VEC_GROUP_SIZE, sizeof(*scratch),
				GFP_KERNEL);
	if (!scratch)
		return -ENOMEM;

	for (g = 0; g < groups; g++) {
		u32 gbase = g << N31_VEC_GROUP_SHIFT;
		unsigned int nobs = 0;
		unsigned int i;
		s32 anchor;
		u32 idx;

		for (i = 0; i < np && nobs < N31_VEC_GROUP_SIZE; i++) {
			u32 key = is_l2v ? pairs[i].l : pairs[i].p;
			u32 val = is_l2v ? pairs[i].p : pairs[i].l;

			if (key < key_base)
				continue;
			idx = key - key_base;
			if ((idx >> N31_VEC_GROUP_SHIFT) != g)
				continue;
			scratch[nobs++] = (s32)val - (s32)(idx &
					  (N31_VEC_GROUP_SIZE - 1));
		}

		if (!nobs) {
			base_out[g] = 0;
			for (idx = 0; idx < N31_VEC_GROUP_SIZE &&
			     gbase + idx < count; idx++) {
				delta_out[gbase + idx] = N31_VEC_MISS;
				(*miss)++;
			}
			continue;
		}

		anchor = n31_median_s32(scratch, nobs);
		base_out[g] = (u32)anchor;

		/* Fill miss first, then overwrite observed. */
		for (idx = 0; idx < N31_VEC_GROUP_SIZE &&
		     gbase + idx < count; idx++)
			delta_out[gbase + idx] = N31_VEC_MISS;

		for (i = 0; i < np; i++) {
			u32 key = is_l2v ? pairs[i].l : pairs[i].p;
			u32 val = is_l2v ? pairs[i].p : pairs[i].l;
			s32 expected, delta;
			u32 off;

			if (key < key_base)
				continue;
			off = key - key_base;
			if ((off >> N31_VEC_GROUP_SHIFT) != g)
				continue;
			if (off >= count)
				continue;

			expected = (s32)anchor + (s32)(off & (N31_VEC_GROUP_SIZE - 1));
			delta = (s32)val - expected;
			if (delta >= -127 && delta <= 126) {
				delta_out[off] = (s8)delta;
				(*ok)++;
			} else {
				delta_out[off] = N31_VEC_ESC;
				if (n31_esc_add(esc, esc_n, esc_cap, key, val,
						pairs[i].weave)) {
					kfree(scratch);
					return -ENOMEM;
				}
				(*escapes)++;
			}
		}

		for (idx = 0; idx < N31_VEC_GROUP_SIZE &&
		     gbase + idx < count; idx++) {
			if (delta_out[gbase + idx] == N31_VEC_MISS)
				(*miss)++;
		}
	}

	kfree(scratch);
	return 0;
}

int n31_vecmap_build(struct n31_vecmap *v, const struct n31_vec_pair *pairs,
		     unsigned int n)
{
	u32 l_min = ~0u, l_max = 0, p_min = ~0u, p_max = 0;
	unsigned int i;
	int ret;

	n31_vecmap_free(v);
	n31_vecmap_init(v);

	if (!pairs || !n)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		if (pairs[i].l < l_min)
			l_min = pairs[i].l;
		if (pairs[i].l > l_max)
			l_max = pairs[i].l;
		if (pairs[i].p < p_min)
			p_min = pairs[i].p;
		if (pairs[i].p > p_max)
			p_max = pairs[i].p;
	}

	/* Align bases down to group boundary for clean indexing. */
	v->l_base = l_min & ~(N31_VEC_GROUP_SIZE - 1);
	v->p_base = p_min & ~(N31_VEC_GROUP_SIZE - 1);
	v->l_count = l_max - v->l_base + 1;
	v->p_count = p_max - v->p_base + 1;
	/* Pad counts to full groups. */
	v->l_count = (v->l_count + N31_VEC_GROUP_SIZE - 1) &
		     ~(N31_VEC_GROUP_SIZE - 1);
	v->p_count = (v->p_count + N31_VEC_GROUP_SIZE - 1) &
		     ~(N31_VEC_GROUP_SIZE - 1);
	v->l2v_groups = v->l_count >> N31_VEC_GROUP_SHIFT;
	v->v2l_groups = v->p_count >> N31_VEC_GROUP_SHIFT;

	v->l2v_base_p = vmalloc(array_size(v->l2v_groups, sizeof(u32)));
	v->l2v_delta = vmalloc(v->l_count);
	v->v2l_base_l = vmalloc(array_size(v->v2l_groups, sizeof(u32)));
	v->v2l_delta = vmalloc(v->p_count);
	if (!v->l2v_base_p || !v->l2v_delta || !v->v2l_base_l || !v->v2l_delta) {
		n31_vecmap_free(v);
		return -ENOMEM;
	}
	memset(v->l2v_delta, N31_VEC_MISS, v->l_count);
	memset(v->v2l_delta, N31_VEC_MISS, v->p_count);

	ret = n31_compress_axis(v->l2v_base_p, v->l2v_delta, v->l_count,
				v->l2v_groups, true, pairs, n, v->l_base,
				&v->l2v_esc, &v->l2v_esc_n, &v->l2v_esc_cap,
				&v->compact_ok, &v->compact_esc,
				&v->compact_miss);
	if (ret) {
		n31_vecmap_free(v);
		return ret;
	}

	ret = n31_compress_axis(v->v2l_base_l, v->v2l_delta, v->p_count,
				v->v2l_groups, false, pairs, n, v->p_base,
				&v->v2l_esc, &v->v2l_esc_n, &v->v2l_esc_cap,
				&v->compact_ok, &v->compact_esc,
				&v->compact_miss);
	if (ret) {
		n31_vecmap_free(v);
		return ret;
	}

	v->ready = true;
	return 0;
}

static u32 n31_vec_predict(u32 base, s8 delta, u32 idx_in_group)
{
	return (u32)((s32)base + (s32)idx_in_group + (s32)delta);
}

int n31_vecmap_lookup(const struct n31_vecmap *v, u32 lba, u32 *p_out)
{
	u32 off, group, idx, p, back;
	s8 d;

	if (!v || !v->ready || !p_out)
		return -EINVAL;
	if (lba < v->l_base || lba >= v->l_base + v->l_count)
		return -ENOENT;

	off = lba - v->l_base;
	d = v->l2v_delta[off];
	if (d == N31_VEC_MISS)
		return -ENOENT;
	if (d == N31_VEC_ESC) {
		if (n31_esc_find(v->l2v_esc, v->l2v_esc_n, lba, &p))
			return -ENOENT;
	} else {
		group = off >> N31_VEC_GROUP_SHIFT;
		idx = off & (N31_VEC_GROUP_SIZE - 1);
		p = n31_vec_predict(v->l2v_base_p[group], d, idx);
	}

	/* Cross-check V2L(P) == L */
	back = n31_vecmap_v2l(v, p);
	if (back != lba)
		return -EUCLEAN;

	*p_out = p;
	return 0;
}

u32 n31_vecmap_v2l(const struct n31_vecmap *v, u32 p)
{
	u32 off, group, idx, l;
	s8 d;

	if (!v || !v->ready)
		return N31_INVALID_L;
	if (p < v->p_base || p >= v->p_base + v->p_count)
		return N31_INVALID_L;

	off = p - v->p_base;
	d = v->v2l_delta[off];
	if (d == N31_VEC_MISS)
		return N31_INVALID_L;
	if (d == N31_VEC_ESC) {
		if (n31_esc_find(v->v2l_esc, v->v2l_esc_n, p, &l))
			return N31_INVALID_L;
		return l;
	}
	group = off >> N31_VEC_GROUP_SHIFT;
	idx = off & (N31_VEC_GROUP_SIZE - 1);
	return n31_vec_predict(v->v2l_base_l[group], d, idx);
}
