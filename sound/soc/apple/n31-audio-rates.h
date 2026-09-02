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
 * The nine rates sub_D34C0 has codes for. This is stock's set exactly.
 *
 * 12 MHz divides exactly to the 48 kHz family only; 44.1 and its relatives
 * land on a fractional divider (12e6/272 = 44117.65). Stock ships them
 * anyway and engages the codec's sample rate converter to absorb the
 * difference -- see cs42l81-spi.c. Advertising a narrower set than stock
 * pushes conversion into the application for no benefit.
 */
#define N31_RATE_MASK	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | 			 SNDRV_PCM_RATE_12000 | SNDRV_PCM_RATE_16000 | 			 SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_24000 | 			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | 			 SNDRV_PCM_RATE_48000)

#define N31_RATE_DEFAULT	44100u

struct n31_rate_cfg {
	unsigned int rate;
	u8 cs42_rate_code;
	u16 clkdiv;
};

/*
 * Read out of sub_D34C0, not inferred. Each arm sets v10 (the codec rate
 * code) and v11 (the divider) and falls into LABEL_23:
 *
 *      8000 -> 10 = 1,  v11 = 1500      24000 -> 8,  500
 *     11025 -> 2,  1088                 32000 -> 9,  375
 *     12000 -> 4,  1000                 44100 -> 10, 272
 *     16000 -> 5,  750                  48000 -> 12, 250
 *     22050 -> 6,  544
 *
 * LABEL_23 then does
 *
 *     if (MEMORY[0x892A02C] == 6000)  v11 >>= 1;
 *     sub_4F716(v6, v11);
 *
 * so the dividers here are the 12 MHz source case. The 6 MHz arm is not
 * implemented: it is unreachable on N31, so the halving at sub_D34C0 is
 * dead code.
 *
 * 0x892A02C is a 32-bit static inside the OS image -- file offset 0x92A02C,
 * bytes E0 2E 00 00 -- initialised to 12000, and field 0 of a nine-field
 * audio config block. It is not a hardware register.
 *
 * Exactly one store to it exists in the image:
 *
 *	080D3712  ldr.w  r8, [pc, #0x120]   ; r8 = 0x892A02C
 *	080D373A  str.w  r1, [r8]
 *
 * That instruction sits in the function at 0x0D3700, which the .c export
 * omits entirely (the banner list jumps 000D34C0 -> 000D3838). It is the
 * setter whose bit 0 selects the clock field. Its one and only call site is
 *
 *	08413E5A  movw r1, #0x386
 *	08413E60  bl   #0x800121c
 *
 * and 0x386 is bits 1,2,7,8,9 -- bit 0 CLEAR. So the sole writer is invoked
 * exactly once and deliberately excludes this field. The bootloader image
 * contains no reference to 0x892A02C and no occurrence of 12000 or 6000.
 *
 * The same word also picks the SoC codec divider, at 0x080D32BE -- a read the
 * decompiler dropped: r0 - 12000, and div = 2 when equal, else 4. One word
 * drives both halves consistently, which is why a divider of 2 here makes
 * these dividers correct.
 *
 * There is therefore nothing for a Linux driver to read at runtime: this is
 * RetailOS software state in its RAM, not a register. Every divider in the
 * table is exactly 12000000 / rate and always will be.
 *
 * LABEL_13 (return 19) is unreachable: sub_D34C0 snaps the rate through
 * sub_191E76 before the switch runs, so stock always substitutes rather
 * than rejecting. n31_resolve_rate() below applies the same ceiling rule.
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

/* Use the requested OSOS rate, else RetailOS 44.1 kHz. */
static inline unsigned int n31_resolve_rate(unsigned int rate);

/*
 * Stock has no "default rate" fallback -- it ceilings. N31_RATE_DEFAULT is
 * kept only as the ALSA-facing default; it is no longer a substitute for an
 * unrecognised rate.
 */
static inline unsigned int n31_pick_rate(unsigned int rate)
{
	if (rate && n31_find_rate(rate))
		return rate;
	return n31_resolve_rate(rate);
}

/*
 * One resolver, used by BOTH the codec and the IIS driver.
 *
 * Both sides must resolve identically. If they do not -- for example a
 * codec that falls back to N31_RATE_DEFAULT while the IIS side refuses the
 * stream -- an out-of-table rate leaves the codec programmed for one rate
 * and the clock divider set for another. Two halves of one link configured
 * differently produce silence or noise rather than an error.
 *
 * So: resolve once, here, and let both sides call this. An exact match
 * wins; otherwise pick the nearest supported rate by relative distance,
 * which keeps the substitution predictable (96000 -> 48000, 5512 ->
 * 8000) instead of collapsing everything onto the default.
 */
/*
 * Stock's rate snap: sub_191E76 (EA 0x08191E76), called as the very first act
 * of sub_D34C0 -- "*a1 = sub_191E76(*a1)" -- before the rate switch runs.
 *
 * It is a pure CEILING with clamps at both ends:
 *
 *	<= 8000   -> 8000        > 22050 -> 24000
 *	>  8000   -> 11025       > 24000 -> 32000
 *	> 11025   -> 12000       > 32000 -> 44100
 *	> 12000   -> 16000       > 44100 -> 48000
 *	> 16000   -> 22050
 *
 * Because its output is always one of the nine tabled rates, LABEL_13 in
 * sub_D34C0 -- the "return 19" arm -- is UNREACHABLE.
 *
 * Stock substitutes silently and always; the rule is a ceiling, not
 * nearest-by-relative-error. The two differ for any request that falls
 * between two tabled rates and is closer to the lower one: nearest gives
 * 12000 for 12500 where stock gives 16000, and 32000 for 33000 where stock
 * gives 44100.
 *
 * Note 0 maps to 8000, not to 44100 -- stock's first test is "a1 > 0x1F40",
 * so zero falls through to the 8000 clamp.
 */
static inline unsigned int n31_resolve_rate(unsigned int rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(n31_rates); i++)
		if (n31_rates[i].rate >= rate)
			return n31_rates[i].rate;

	return n31_rates[ARRAY_SIZE(n31_rates) - 1].rate;
}

/*
 * True when the codec must run its sample-rate converter rather than
 * clocking the DAC straight off the ASP.
 *
 * 48 kHz is the special case, not 44.1. sub_183138 branches on the
 * rate code and only code 12 gets the native arm:
 *
 *     if (a1 == 12) {
 *         sub_43CDB4(267, 8);        0x10B = 0x08
 *         sub_43CDB4(268, 9);        0x10C = 0x09
 *         sub_42A5D6(305, 1, 1);     0x131 bit0 = 1
 *     } else {
 *         sub_43CDB4(289, 8);        0x121 = 0x08
 *         sub_43CDB4(290, 9);        0x122 = 0x09
 *         sub_42A5D6(304, 15, a1);   0x130 & 0x0F = code
 *         sub_42A5D6(305, 1, 0);     0x131 bit0 = 0
 *         sub_43CDB4(267, 4);        0x10B = 0x04
 *         sub_43CDB4(268, 51);       0x10C = 0x33
 *     }
 *
 * so 44.1 kHz runs through the SRC. cs42l81_set_rate_long() mirrors
 * both arms, and the three writes before the branch -- 0x00E & 0xC0 =
 * 0xC0, 0x00F & 0x0F = code, 0x12F = code | (code << 4) -- and the
 * three after -- 0x00E & 0xC0 = 0x40, sub_400330(64), 0x220 bit5 --
 * are there too, in stock's order.
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

/*
 * Implemented by s5l8740-i2s.c. This is stock's sub_41CBD8 case 9: the
 * codec clock gate is CLKCON+0x0C bit 15, cleared to run and set to gate.
 * Declared here so both drivers agree on the signature -- cs42l81-spi.c
 * reaches it through __symbol_get, which cannot type-check anything.
 */
void s5l8740_codec_clk_gate(bool on);

/*
 * RetailOS sub_345D28(9, 0, div): CLKCON+0x0C bits 3:0 hold the codec clock
 * divider as div-1. Stock selects 2 at a 12 MHz MCLK. Declared here so both
 * drivers agree on the signature; cs42l81-spi.c reaches it through
 * __symbol_get, which cannot type-check anything.
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
 * Accessory / headphone-remote message channel on the codec's FIFO
 * registers (RetailOS sub_15A50C / sub_19A838). Not used by the audio path.
 */
int cs42l81_mbox_send(u8 *buf, size_t buf_size);
int cs42l81_mbox_recv(u8 *buf, size_t buf_size);

#endif /* N31_AUDIO_RATES_H */
