/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * N31 sample-rate table — OSOS sub_D34C0 (osos.dec.bin.ida.c).
 *
 * RetailOS local music observed CLKDIV=272 → 44100 (code 10).
 * 48000 (code 12, CLKDIV 250) stays in the table for ALSA requests;
 * do not assume 48 kHz. Unspecified rate → N31_RATE_DEFAULT.
 */
#ifndef N31_AUDIO_RATES_H
#define N31_AUDIO_RATES_H

#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/types.h>
#include <sound/pcm.h>

/*
 * Every rate the hardware actually has a divider for. Both the codec and
 * the IIS DAI advertise exactly this set: they used to advertise only
 * 44.1/48 while the table below carried nine entries, so 8/11.025/12/16/
 * 22.05/24/32 kHz streams were refused by ALSA despite the silicon
 * supporting them.
 */
#define N31_RATE_MASK	(SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_12000 | SNDRV_PCM_RATE_24000)

#define N31_RATE_DEFAULT	44100u

struct n31_rate_cfg {
	unsigned int rate;
	u8 cs42_rate_code;
	u16 clkdiv;
};

static const struct n31_rate_cfg n31_rates[] = {
	{  8000,  1, 1500 },
	{ 11025,  2, 1088 },
	{ 12000,  4, 1000 },
	{ 16000,  5,  750 },
	{ 22050,  6,  544 },
	{ 24000,  8,  500 },
	{ 32000,  9,  375 },
	{ 44100, 10,  272 },
	{ 48000, 12,  250 },
};

static inline const struct n31_rate_cfg *n31_find_rate(unsigned int rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(n31_rates); i++)
		if (n31_rates[i].rate == rate)
			return &n31_rates[i];
	return NULL;
}

/* Use the requested OSOS rate, else RetailOS 44.1 kHz. */
static inline unsigned int n31_pick_rate(unsigned int rate)
{
	if (rate && n31_find_rate(rate))
		return rate;
	return N31_RATE_DEFAULT;
}

/*
 * One resolver, used by BOTH the codec and the IIS driver.
 *
 * They used to resolve independently: the codec fell back to
 * N31_RATE_DEFAULT for anything it did not recognise while the IIS side
 * refused the stream outright. For an out-of-table rate that meant the
 * codec was programmed for 44.1 kHz while the clock divider was left on
 * whatever the caller asked for -- the two halves of one link configured
 * for different rates, which is silence or noise rather than an error.
 *
 * So: resolve once, here, and let both sides call this. An exact match
 * wins; otherwise pick the nearest supported rate by relative distance,
 * which keeps the substitution predictable (96000 -> 48000, 5512 ->
 * 8000) instead of collapsing everything onto the default.
 */
static inline unsigned int n31_resolve_rate(unsigned int rate)
{
	unsigned int i, best = N31_RATE_DEFAULT;
	u64 best_err = U64_MAX;

	if (!rate)
		return N31_RATE_DEFAULT;
	if (n31_find_rate(rate))
		return rate;

	for (i = 0; i < ARRAY_SIZE(n31_rates); i++) {
		unsigned int r = n31_rates[i].rate;
		u64 err = (r > rate) ? (u64)(r - rate) : (u64)(rate - r);

		/* Scale so the choice is proportional, not absolute. */
		err = div64_u64(err * 100000ULL, r);
		if (err < best_err) {
			best_err = err;
			best = r;
		}
	}
	return best;
}

/*
 * True when the codec must run its sample-rate converter rather than
 * clocking the DAC straight off the ASP. OSOS takes the SRC arm for
 * every rate except 48 kHz (rate code 12) -- see sub_183138, where the
 * non-48 arm programs 0x121/0x122 and drops 0x10B/0x10C to 4/0x33.
 */
static inline bool n31_rate_uses_src(unsigned int rate)
{
	const struct n31_rate_cfg *r = n31_find_rate(n31_resolve_rate(rate));

	return r && r->cs42_rate_code != 12;
}

/* Exact 1 kHz period group: rate / gcd(rate, 1000) frames. */
static inline unsigned int n31_tone_period_frames(unsigned int rate)
{
	unsigned int a, b, t;

	rate = n31_pick_rate(rate);
	a = rate;
	b = 1000;
	while (b) {
		t = a % b;
		a = b;
		b = t;
	}
	return rate / a;
}

/* 256-point sine, peak ≈ 0.7 * 32767. 1 kHz via DDS at any table rate. */
static const s16 n31_sin256[256] = {
	0, 563, 1125, 1687, 2248, 2808, 3366, 3921,
	4475, 5026, 5573, 6118, 6658, 7195, 7727, 8255,
	8778, 9295, 9807, 10313, 10812, 11306, 11792, 12271,
	12743, 13207, 13664, 14112, 14551, 14982, 15404, 15816,
	16219, 16612, 16995, 17368, 17731, 18082, 18423, 18753,
	19071, 19378, 19674, 19957, 20229, 20488, 20735, 20969,
	21191, 21400, 21596, 21779, 21949, 22106, 22250, 22380,
	22496, 22599, 22689, 22765, 22827, 22875, 22909, 22930,
	22937, 22930, 22909, 22875, 22827, 22765, 22689, 22599,
	22496, 22380, 22250, 22106, 21949, 21779, 21596, 21400,
	21191, 20969, 20735, 20488, 20229, 19957, 19674, 19378,
	19071, 18753, 18423, 18082, 17731, 17368, 16995, 16612,
	16219, 15816, 15404, 14982, 14551, 14112, 13664, 13207,
	12743, 12271, 11792, 11306, 10812, 10313, 9807, 9295,
	8778, 8255, 7727, 7195, 6658, 6118, 5573, 5026,
	4475, 3921, 3366, 2808, 2248, 1687, 1125, 563,
	0, -563, -1125, -1687, -2248, -2808, -3366, -3921,
	-4475, -5026, -5573, -6118, -6658, -7195, -7727, -8255,
	-8778, -9295, -9807, -10313, -10812, -11306, -11792, -12271,
	-12743, -13207, -13664, -14112, -14551, -14982, -15404, -15816,
	-16219, -16612, -16995, -17368, -17731, -18082, -18423, -18753,
	-19071, -19378, -19674, -19957, -20229, -20488, -20735, -20969,
	-21191, -21400, -21596, -21779, -21949, -22106, -22250, -22380,
	-22496, -22599, -22689, -22765, -22827, -22875, -22909, -22930,
	-22937, -22930, -22909, -22875, -22827, -22765, -22689, -22599,
	-22496, -22380, -22250, -22106, -21949, -21779, -21596, -21400,
	-21191, -20969, -20735, -20488, -20229, -19957, -19674, -19378,
	-19071, -18753, -18423, -18082, -17731, -17368, -16995, -16612,
	-16219, -15816, -15404, -14982, -14551, -14112, -13664, -13207,
	-12743, -12271, -11792, -11306, -10812, -10313, -9807, -9295,
	-8778, -8255, -7727, -7195, -6658, -6118, -5573, -5026,
	-4475, -3921, -3366, -2808, -2248, -1687, -1125, -563,
};

static inline s16 n31_tone_s16(unsigned int sample, unsigned int rate)
{
	u32 idx;

	rate = n31_pick_rate(rate);
	idx = (u32)div_u64((u64)sample * 1000ull * 256ull, rate);
	return n31_sin256[idx & 255];
}

#endif /* N31_AUDIO_RATES_H */
