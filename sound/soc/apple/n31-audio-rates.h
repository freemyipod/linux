/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Sample rates shared by the N31 codec and IIS drivers.
 *
 * The table, the rate codes and the clock dividers are all taken from the
 * iPod nano 7G stock firmware. docs-internal/n7g-audio/N31-AUDIO-STOCK-MAP.md
 * records where each value comes from and why the dividers are what they are.
 */
#ifndef N31_AUDIO_RATES_H
#define N31_AUDIO_RATES_H

#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/types.h>
#include <sound/pcm.h>

/*
 * The nine rates the codec has rate codes for -- stock's set exactly.
 *
 * 12 MHz divides exactly to the 48 kHz family only; 44.1 kHz and its
 * relatives land on a fractional divider (12e6/272 = 44117.65, 400 ppm
 * fast). Stock ships them anyway and clocks the DAC straight off the ASP at
 * those rates, so the error stays a pitch offset rather than becoming
 * converter slip. Advertising a narrower set than stock would push
 * conversion into the application for no benefit.
 */
#define N31_RATE_MASK	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
			 SNDRV_PCM_RATE_12000 | SNDRV_PCM_RATE_16000 | \
			 SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_24000 | \
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			 SNDRV_PCM_RATE_48000)

#define N31_RATE_DEFAULT	44100u

struct n31_rate_cfg {
	unsigned int rate;
	u8 cs42_rate_code;
	u16 clkdiv;
};

/*
 * Rate, codec rate code, IIS clock divider.
 *
 * Every divider is exactly 12000000/rate. Stock reads its master clock from
 * a software word that is initialised to 12 MHz and, in this firmware, never
 * written again -- the one function that could change it is called once,
 * with a mask that excludes that field. So the 6 MHz case, in which stock
 * halves every divider, is unreachable and is not implemented here.
 */
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

/* Forward declaration; the ceiling rule is documented at the definition. */
static inline unsigned int n31_resolve_rate(unsigned int rate);

/*
 * Take the rate as asked when it is in the table, otherwise snap it.
 *
 * Stock has no default-rate fallback -- it always ceilings. N31_RATE_DEFAULT
 * is only the ALSA-facing default, not a substitute for an unrecognised rate.
 */
static inline unsigned int n31_pick_rate(unsigned int rate)
{
	if (rate && n31_find_rate(rate))
		return rate;
	return n31_resolve_rate(rate);
}

/*
 * One resolver, used by both the codec and the IIS driver.
 *
 * Both sides have to resolve identically. If they do not, an out-of-table
 * rate leaves the codec programmed for one rate and the clock divider set for
 * another, and two halves of one link configured differently produce silence
 * or noise rather than an error.
 *
 * The rule is stock's: a ceiling, clamped at both ends, applied before the
 * rate is looked up. It is not nearest-match, and the two differ for any
 * request that falls between two tabled rates and is closer to the lower one
 * -- 12500 resolves to 16000, not 12000. Note that zero resolves to 8000.
 *
 * Because the result is always a tabled rate, the lookup below cannot fail.
 */
static inline unsigned int n31_resolve_rate(unsigned int rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(n31_rates); i++)
		if (n31_rates[i].rate >= rate)
			return n31_rates[i].rate;

	return n31_rates[ARRAY_SIZE(n31_rates) - 1].rate;
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

/*
 * Implemented by s5l8740-i2s.c: the codec clock gate, CLKCON+0x0C bit 15.
 * Clear to run, set to gate. Declared here so both drivers agree on the
 * signature -- cs42l81-spi.c reaches it through __symbol_get, which cannot
 * type-check anything.
 */
void s5l8740_codec_clk_gate(bool on);

/*
 * Implemented by s5l8740-i2s.c: the codec clock divider, CLKCON+0x0C bits
 * 3:0, held as div-1. Stock selects 2 at a 12 MHz master clock. Declared
 * here for the same reason as the gate above.
 */
void s5l8740_codec_clk_divider(unsigned int div);

/*
 * Codec entry points the IIS driver drives. Declared here rather than in each
 * .c so both sides agree on the signatures and the definitions do not trip
 * -Wmissing-prototypes.
 */
int cs42l81_play_prepare(void);
int cs42l81_play_start(void);
int cs42l81_play_stop(void);
int cs42l81_pre_iis_start(void);
int cs42l81_post_iis_start(void);
int cs42l81_set_clock_role(bool drive);
int cs42l81_asp_hold_light(void);
int cs42l81_get_audio_path_mode(void);
void cs42l81_schedule_post_iis(void);
void cs42l81_cancel_post_iis(void);

/*
 * Accessory and headphone-remote message channel, on the codec's FIFO
 * registers. Not used by the audio path.
 */
int cs42l81_mbox_send(u8 *buf, size_t buf_size);
int cs42l81_mbox_recv(u8 *buf, size_t buf_size);

#endif /* N31_AUDIO_RATES_H */
