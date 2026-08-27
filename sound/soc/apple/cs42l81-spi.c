// SPDX-License-Identifier: GPL-2.0-only
/*
 * CS42L81 / Apple 338S1146 — SPI0 control path (N31 RetailOS-matched)
 *
 * Framing (sub_43CDB4 / sub_43CDFA):
 *   write: 0x6C, reg_hi, reg_lo, 0, data
 *   read:  0x6D, reg_hi, reg_lo, 0, 0xFF  (rx data on last byte)
 *
 * Bring-up (confirmed corpus):
 *   read 0x227; write 0xC96F=0x0E|0x1E; 0x9901 unlock A5 then 0;
 *   0xC81F=0xFF; 0xC85F=0x0F
 *
 * RetailOS user volume is an integer 0..256 (debug HUD, NVRAM key 11/179
 * clamped in 37138, Vol+/- in 1BB754/1BB874). That value is a CoreAudio
 * VolumeScalar (256 = unity), not a CS42 mixer byte.
 *
 * 0x403/0x404 are HP mixer tap indices from 174E7C / 440AA4(udiv, 160):
 *   play 5706F4(1): L=(0+159)/160+2 = 2, R=+1 = 1
 *   play 5706F4(4): L=(160+159)/160+2 = 3, R=+1 = 2
 * Analog 0x527 is mute 0xFF / unmute 0x60 (F141C). Do not unlock 9901
 * after mixer.
 *
 * Play graph: RetailOS sub_570620. If 8A8FB58 is set → sub_5707D8 static
 * blast; else dynamic 5706F4 → 174E7C → 174E38 → 165BD4 → 0x401 latch.
 * Do not assume tap index == mixer slot index.
 *
 * 10B4EA / 1042FC are RTOS SVC domain 33 (enter/exit), NOT a CS42 SPI page.
 * 43DF0A / 43DF04 are domain 24 around SPI. Linux: c->lock + logged no-op.
 *
 * ASoC DAI cs42l81-hifi: analog on hw_params. IIS serializer is the CPU DAI.
 *
 * CS42L42/L83 (I2C, paged 8-bit regmap, snd_soc_cs42l42) are a newer Cirrus
 * line with nearly identical maps — chip ID + MCLK_CTL defaults differ. N31
 * CS42L81 / 338S1146 is SPI-framed 16-bit addresses (see N31-102-CS42-REG-CORPUS);
 * do not drop in cs42l42.c wholesale.
 *
 * Cirrus bring-up notes:
 *   Reset mutes outputs (0x527=0xFF); unmute 0x60 on play.
 *   0x2F bit6: glass shows 0x40 idle (no IIS), 0x00 while BCLK/LRCLK run.
 *   Treat bit6 as LOS / no-sync when asp_bit6_is_los=1 (default): asp_lock
 *   succeeds when bit6 is CLEAR. Legacy asp_bit6_is_los=0 waits for bit6 set.
 *   ASP lock after IIS clocks (414FAE), not before.
 *   I2S slave NB_NF 16-bit; no DAPM graph — path is register audio_on().
 *
 * ARTP routes (CoreAudio debug: "BT-%s, HP-%s, USB-%s, MB-%s"):
 *   HP = headphone jack (CS42 IIS0). MB = mainboard/dock internal route,
 *   not MikeyBus and not a feed into the 3.5 mm jack. USB/BT are separate
 *   digital paths (Lightning/Tristar, BCM2078 A2DP). N31 Linux: HP only.
 *
 * Play/stop: sub_42D364(1)=F141C(1)+570620; stop=42D364(0).
 *
 * HPDET / jack (p5-hpdet): OPEN — no CONFIRMED_N31 GPIO ID or plug status
 * bit. Family schematics show an HPDET pin; N31 RESET/IRQ/HPDET GPIOs are
 * explicitly unmapped. Do not wire ALSA Jack/Switch or mute-on-unplug until
 * glass diffs close docs/N31-HPDET-OPEN.md. Headphones plugged on glass =
 * detect=1 baseline (N31-GLASS-AUDIO-CAPTURE.md).
 */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/kallsyms.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "n31-audio-rates.h"

#define CS42L81_USER_VOL_MAX	256
#define CS42L81_VOL_STEP	16	/* ~16 presses full 0..256 range */

/*
 * Play rate for D34C0/183138. 0 = follow PCM hw_params (OSOS 44100 if none).
 * RetailOS local music is 44100. Do not force 48000.
 */
static unsigned int play_rate;
module_param(play_rate, uint, 0644);
MODULE_PARM_DESC(play_rate, "CS42 D34C0 rate; 0=follow PCM (default, OSOS 44100)");

/*
 * graph_mode: RetailOS sub_570620 play-graph selector.
 *   0 = static sub_5707D8 (DEFAULT — music path when 8A8FB58 non-NULL)
 *   1 / 3 / 4 = dynamic 5706F4→174E7C→174E38→165BD4
 *
 * RE (osos.dec.bin): BSS at 0x892A05D/05E/060/061 and dword table at
 * 0x892A068 have ZERO writers and ZERO ROM init. 174E7C therefore always
 * sees table_v=0 and idx=0 → accum_l/r=0. Mode 4 cannot grow non-zero
 * slot gains from firmware recovery; inventing table words would not be
 * RetailOS. Music uses the hardcoded 5707D8 image (gains 01 E0).
 */
static int graph_mode;
module_param(graph_mode, int, 0644);
MODULE_PARM_DESC(graph_mode,
		 "CS42 play graph: 0=static 5707D8 (default); 1/3/4=dynamic");

/* -1 = use computed 174E7C taps; else force 0x403/0x404 after compute. */
static int graph_tap_l_override = -1;
static int graph_tap_r_override = -1;
module_param(graph_tap_l_override, int, 0644);
module_param(graph_tap_r_override, int, 0644);
MODULE_PARM_DESC(graph_tap_l_override, "Override 0x403 tap (-1=computed)");
MODULE_PARM_DESC(graph_tap_r_override, "Override 0x404 tap (-1=computed)");

/*
 * Debug only: after dynamic/static graph, force slot at 0x410 to 02 01 E0.
 * Default off — tap!=slot; do not use as the primary fix.
 */
static bool graph_slot2_gain;
module_param(graph_slot2_gain, bool, 0644);
MODULE_PARM_DESC(graph_slot2_gain,
		 "DEBUG: bake 0x410=02 01 E0 after graph (default 0)");

/*
 * 174E7C table word at 0x892A068[idx_l_hi]. Recovered value is 0 (BSS).
 * Module param kept only for deliberate experiments — not a RetailOS value.
 */
static unsigned int graph_table_v; /* RE: always 0 */
module_param(graph_table_v, uint, 0644);
MODULE_PARM_DESC(graph_table_v,
		 "174E7C table[idx] override (RE default 0; do not invent)");
/*
 * audio_route: 0=HP jack (default). USB/BT/MB are CoreAudio ARTP names for
 * alternate sinks — no RE-backed CS42 mux to HP on N31 yet.
 */
static int audio_route;
module_param(audio_route, int, 0644);
MODULE_PARM_DESC(audio_route, "0=HP jack (default); USB/BT/MB unimplemented");

/* 570620 gates on 0x8925CF4==1 (headset ready). */
static bool force_headset;
module_param(force_headset, bool, 0644);
MODULE_PARM_DESC(force_headset, "1=skip headset-ready gate (glass bring-up)");

static unsigned int jack_poll_ms = 500;
module_param(jack_poll_ms, uint, 0644);
MODULE_PARM_DESC(jack_poll_ms, "MikeyBus/HSDET poll period ms (0=off)");

/*
 * audio_path_mode (i2s trigger also reads via cs42l81_get_audio_path_mode):
 *   0 = legacy: play graph folded into codec prepare (debug only)
 *   1 = RetailOS: F141C+570620 before DMA/TXCOM (default)
 *   2 = RetailOS: DMA/TXCOM before F141C+570620
 */
static int audio_path_mode = 1;
module_param(audio_path_mode, int, 0644);
MODULE_PARM_DESC(audio_path_mode, "0=legacy soup; 1=play before IIS; 2=play after IIS");

/* 1 = redo D3280 prepare on every play_prepare (no stale brought_up). */
static bool force_full_prepare = true;
module_param(force_full_prepare, bool, 0644);
MODULE_PARM_DESC(force_full_prepare, "1=re-run codec prepare every session (default)");

/*
 * RetailOS sub_10B4EA → SVC domain 33 enter; sub_1042FC → domain 33 exit.
 * NOT a CS42 SPI page. Linux: log + marker under c->lock (already held).
 * Legacy alias allow_no_page33 kept so old glass scripts still load.
 */
static bool allow_no_page33 = true;
module_param(allow_no_page33, bool, 0644);
MODULE_PARM_DESC(allow_no_page33,
		 "legacy alias; domain 33 is always a no-op (never blocks graph)");

static bool dump_regs;
module_param(dump_regs, bool, 0644);
MODULE_PARM_DESC(dump_regs, "1=verbose CS42 FINAL + 5707D8 verify dumps");

/*
 * post_iis_401_rmw=1 (default): force 401&3=2 after ASP (legacy glass).
 * =0: leave static 5707D8 latch 401=0x12 alone (A/B if silent).
 */
static bool post_iis_401_rmw = true;
module_param(post_iis_401_rmw, bool, 0644);
MODULE_PARM_DESC(post_iis_401_rmw, "1=post_iis 401&3=2 (default); 0=keep graph 0x12");

/* D3280(4) RE writes C96F=0x0E. Glass sometimes needed 0x1E — A/B. */
static int c96f_final = 0x0e;
module_param(c96f_final, int, 0644);
MODULE_PARM_DESC(c96f_final, "D3280(4) final 0xC96F (default 0x0E RE; try 0x1E)");

/* Dynamic 570620: force 41F944 gate pass (route_present=1, busy=0). */
static bool graph_force_gate = true;
module_param(graph_force_gate, bool, 0644);
MODULE_PARM_DESC(graph_force_gate, "1=force 570620 gate pass (default bring-up)");

/*
 * Recovered OSOS graph BSS layout at 0x892A05C (Thumb base for 174E7C /
 * 165BD4 / 5706F4). All idx_* and table[] are never stored in OSOS — left
 * zero. Only 5706F4 writes count_l/r and base_l/r.
 */
static const u32 cs42_osos_graph_table_892a068[] = {
	/* table[0] at 0x892A068 — only entry 174E7C can hit with idx_hi==0 */
	0x00000000,
};

struct cs42_graph_state {
	u8 count_l;
	u8 count_r;
	u8 idx_l_lo;	/* 0x892A05D */
	u8 idx_l_hi;	/* 0x892A05E */
	u8 idx_r_lo;	/* 0x892A060 */
	u8 idx_r_hi;	/* 0x892A061 */
	u16 base_l;	/* 0x892A062 */
	u16 base_r;	/* 0x892A064 */
	u16 accum_l;	/* 0x8AE4E50 */
	u16 accum_r;	/* 0x8AE4E54 */
	u8 tap_l;	/* 0x8AE4E4C → 0x403 */
	u8 tap_r;	/* 0x8AE4E4D → 0x404 */
	u8 status_528;
	int mode;
};

struct cs42l81 {
	struct spi_device *spi;
	struct mutex lock;
	unsigned int user_vol;
	unsigned int rate;	/* last D34C0 rate; ASP reprogram uses this */
	bool dai_mute;
	bool input_handler_reg;
	struct snd_soc_component *component;
	struct input_handler input_handler;
	struct work_struct vol_work;
	struct delayed_work asp_post_work;
	struct delayed_work jack_work;
	atomic_t vol_steps;
	bool jack_poll_active;
	bool jack_last_present;
	bool route_playing;	/* 892A058 mirror */
	bool codec_prepared;
	bool play_started;
	int graph_domain;	/* RetailOS SVC domain 33 held (not SPI page) */
	struct cs42_graph_state graph;
};

struct cs42_regval {
	u16 reg;
	u8 val;
};

static struct cs42l81 *cs42l81_dev;

static unsigned int cs42_pick_rate(struct cs42l81 *c, unsigned int rate)
{
	if (play_rate)
		return n31_pick_rate(play_rate);
	if (rate)
		return n31_pick_rate(rate);
	if (c && c->rate)
		return n31_pick_rate(c->rate);
	return N31_RATE_DEFAULT;
}

/*
 * 0x2F bit6 polarity: glass idle (no IIS) reads 0x40; running IIS reads 0x00.
 * Default asp_bit6_is_los=1 → bit6 set = loss/no-sync, clear = ASP locked.
 */
static bool asp_bit6_is_los = true;
module_param(asp_bit6_is_los, bool, 0644);
MODULE_PARM_DESC(asp_bit6_is_los,
		 "1=bit6 is LOS flag (clear=synced); 0=legacy bit6-set=synced");

/*
 * asp_gate_unmute=0 (default): post_iis always forces HP unmute; 0x2F is telemetry.
 * asp_gate_unmute=1: legacy — unmute only when asp probe reports synced.
 */
static bool asp_gate_unmute;
module_param(asp_gate_unmute, bool, 0644);
MODULE_PARM_DESC(asp_gate_unmute, "1=gate unmute on 0x2F probe; 0=force unmute (default)");

/*
 * of_asp_slave=1: clear 0x0F bit7 before IIS (CS42L73-family ASP slave test).
 * Default 0 keeps RetailOS D3280(3) pad-drive write.
 */
static bool of_asp_slave;
module_param(of_asp_slave, bool, 0644);
MODULE_PARM_DESC(of_asp_slave, "1=force CS42 0x0F bit7=0 (ASP slave) before IIS");

static bool cs42l81_asp_synced(u8 r2f)
{
	if (asp_bit6_is_los)
		return !(r2f & 0x40);
	return !!(r2f & 0x40);
}

int cs42l81_post_iis_start(void);
int cs42l81_play_stop(void);
int cs42l81_play_start(void);
int cs42l81_play_prepare(void);
int cs42l81_pre_iis_start(void);
int cs42l81_get_audio_path_mode(void);
void cs42l81_schedule_post_iis(void);
void cs42l81_cancel_post_iis(void);

static int cs42l81_write(struct cs42l81 *c, u16 reg, u8 val);
static int cs42l81_apply_user_vol(struct cs42l81 *c);
static void cs42l81_log_start_state(struct cs42l81 *c, const char *tag);
static void cs42l81_push_pcm_q8(unsigned int vol);

static int cs42l81_write(struct cs42l81 *c, u16 reg, u8 val)
{
	u8 tx[5] = {
		0x6c,
		(reg >> 8) & 0xff,
		reg & 0xff,
		0x00,
		val,
	};
	struct spi_transfer t = { .tx_buf = tx, .len = 5 };
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(c->spi, &m);
}

static int cs42l81_read(struct cs42l81 *c, u16 reg, u8 *val)
{
	u8 tx[5] = {
		0x6d,
		(reg >> 8) & 0xff,
		reg & 0xff,
		0x00,
		0xff,
	};
	u8 rx[5] = { 0 };
	struct spi_transfer t = { .tx_buf = tx, .rx_buf = rx, .len = 5 };
	struct spi_message m;
	int ret;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(c->spi, &m);
	if (ret)
		return ret;
	*val = rx[4];
	return 0;
}

static int cs42l81_rmw(struct cs42l81 *c, u16 reg, u8 mask, u8 val)
{
	u8 cur = 0;
	int ret = cs42l81_read(c, reg, &cur);

	if (ret)
		return ret;
	return cs42l81_write(c, reg, (cur & ~mask) | (val & mask));
}

static int cs42l81_bringup(struct cs42l81 *c)
{
	u8 st = 0;
	int ret;

	ret = cs42l81_read(c, 0x0227, &st);
	dev_info(&c->spi->dev, "CS42 status 0x227 = 0x%02x (ret=%d)\n", st, ret);

	/* rail / backpower-ish — prefer safe idle 0x0E over 0x1E */
	ret = cs42l81_write(c, 0xc96f, 0x0e);
	if (ret)
		return ret;

	/* unlock-like */
	cs42l81_write(c, 0x9901, 0xa5);
	cs42l81_write(c, 0x9901, 0x00);

	cs42l81_write(c, 0xc81f, 0xff);
	cs42l81_write(c, 0xc85f, 0x0f);

	cs42l81_read(c, 0x0219, &st);
	dev_info(&c->spi->dev, "CS42 companion 0x219 = 0x%02x\n", st);
	return 0;
}

/*
 * RetailOS sub_5707D8 — literal SPI table (Hex-Rays). Not a loop guess.
 * Preceded by 0x006/529/52A/533/534; ends with msleep(100)+0x500+read 528.
 */
static const struct cs42_regval cs42_static_5707d8[] = {
	{ 0x0006, 0x24 },
	{ 0x0529, 0x2c },
	{ 0x052a, 0x2c },
	{ 0x0533, 0x2c },
	{ 0x0534, 0x2c },

	{ 0x0400, 0x04 },
	{ 0x0401, 0x10 },
	{ 0x0402, 0x00 },
	{ 0x0403, 0x09 },
	{ 0x0404, 0x08 },
	{ 0x0405, 0x00 },
	{ 0x0406, 0x00 },

	{ 0x0407, 0x00 }, { 0x0408, 0x01 }, { 0x0409, 0xe0 },
	{ 0x040a, 0x01 }, { 0x040b, 0x01 }, { 0x040c, 0xe0 },
	{ 0x040d, 0xfe }, { 0x040e, 0x00 }, { 0x040f, 0xa0 },

	{ 0x0410, 0x02 }, { 0x0411, 0x00 }, { 0x0412, 0x00 },
	{ 0x0413, 0x03 }, { 0x0414, 0x00 }, { 0x0415, 0x00 },
	{ 0x0416, 0x04 }, { 0x0417, 0x00 }, { 0x0418, 0x00 },
	{ 0x0419, 0x05 }, { 0x041a, 0x00 }, { 0x041b, 0x00 },
	{ 0x041c, 0x06 }, { 0x041d, 0x00 }, { 0x041e, 0x00 },
	{ 0x041f, 0x07 }, { 0x0420, 0x00 }, { 0x0421, 0x00 },
	{ 0x0422, 0x08 }, { 0x0423, 0x00 }, { 0x0424, 0x00 },
	{ 0x0425, 0x09 }, { 0x0426, 0x00 }, { 0x0427, 0x00 },

	{ 0x0428, 0x0a }, { 0x0429, 0x01 }, { 0x042a, 0xe0 },
	{ 0x042b, 0x0b }, { 0x042c, 0x01 }, { 0x042d, 0xe0 },
	{ 0x042e, 0xff }, { 0x042f, 0x00 }, { 0x0430, 0xa0 },

	{ 0x0431, 0x0c }, { 0x0432, 0x00 }, { 0x0433, 0x00 },
	{ 0x0434, 0x0d }, { 0x0435, 0x00 }, { 0x0436, 0x00 },
	{ 0x0437, 0x0e }, { 0x0438, 0x00 }, { 0x0439, 0x00 },
	{ 0x043a, 0x0f }, { 0x043b, 0x00 }, { 0x043c, 0x00 },
	{ 0x043d, 0x10 }, { 0x043e, 0x00 }, { 0x043f, 0x00 },
	{ 0x0440, 0x11 }, { 0x0441, 0x00 }, { 0x0442, 0x00 },
	{ 0x0443, 0x12 }, { 0x0444, 0x00 }, { 0x0445, 0x00 },
	{ 0x0446, 0x13 }, { 0x0447, 0x00 }, { 0x0448, 0x00 },

	{ 0x0400, 0x04 },
	{ 0x0401, 0x12 },
};

/*
 * RetailOS RTOS domains (SVC 0x46) — NOT CS42 hardware pages.
 *   10B4EA / 43DD18(33) = graph critical-section enter
 *   1042FC / 43DDA0(33) = graph critical-section exit/status
 *   43DF0A / 43DD18(24) = SPI critical-section enter
 *   43DF04 / 43DDA0(24) = SPI critical-section exit
 * Linux: caller already holds c->lock; these are markers + logs only.
 */
static void cs42_domain33_enter(struct cs42l81 *c)
{
	if (c->graph_domain == 33)
		return;
	c->graph_domain = 33;
	(void)allow_no_page33; /* legacy param; domain 33 never blocks */
	dev_info_once(&c->spi->dev,
		      "RetailOS domain 33 enter (10B4EA→SVC 0x46; Linux no-op under lock)\n");
	if (dump_regs)
		dev_info(&c->spi->dev, "domain33 enter\n");
}

static void cs42_domain33_exit(struct cs42l81 *c)
{
	if (c->graph_domain != 33)
		return;
	c->graph_domain = 0;
	if (dump_regs)
		dev_info(&c->spi->dev, "domain33 exit (1042FC)\n");
}

static int cs42_graph_begin(struct cs42l81 *c, int page)
{
	/* page arg kept for call-site clarity; only 33 is used by RE. */
	if (page != 33 && dump_regs)
		dev_info(&c->spi->dev, "domain enter id=%d (expected 33)\n", page);
	cs42_domain33_enter(c);
	return 0;
}

static int cs42_graph_end(struct cs42l81 *c, int page)
{
	(void)page;
	cs42_domain33_exit(c);
	return 0;
}

static int cs42_write_table(struct cs42l81 *c, const struct cs42_regval *t,
			    unsigned int n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = cs42l81_write(c, t[i].reg, t[i].val);
		if (ret)
			return ret;
	}
	return 0;
}

/* Read back last-write-wins expected values for 0x400..0x448 + key regs. */
static void cs42_verify_5707d8(struct cs42l81 *c)
{
	static const struct cs42_regval expect[] = {
		{ 0x0006, 0x24 },
		{ 0x0529, 0x2c }, { 0x052a, 0x2c },
		{ 0x0533, 0x2c }, { 0x0534, 0x2c },
		{ 0x0400, 0x04 }, { 0x0401, 0x12 },
		{ 0x0402, 0x00 }, { 0x0403, 0x09 }, { 0x0404, 0x08 },
		{ 0x0405, 0x00 }, { 0x0406, 0x00 },
		{ 0x0407, 0x00 }, { 0x0408, 0x01 }, { 0x0409, 0xe0 },
		{ 0x040a, 0x01 }, { 0x040b, 0x01 }, { 0x040c, 0xe0 },
		{ 0x0428, 0x0a }, { 0x0429, 0x01 }, { 0x042a, 0xe0 },
		{ 0x0500, 0x05 },
	};
	unsigned int i, mism = 0;
	u8 v;

	for (i = 0; i < ARRAY_SIZE(expect); i++) {
		if (cs42l81_read(c, expect[i].reg, &v))
			continue;
		if (v != expect[i].val) {
			dev_warn(&c->spi->dev,
				 "5707D8 MISMATCH 0x%03x got=%02x want=%02x\n",
				 expect[i].reg, v, expect[i].val);
			mism++;
		}
	}
	dev_info(&c->spi->dev, "5707D8 verify: %u mismatches (0=good)\n", mism);
}

static void cs42_log_final_state(struct cs42l81 *c, const char *tag)
{
	static const u16 regs[] = {
		0x006, 0x007, 0x00e, 0x00f,
		0x075, 0x074, 0x07b, 0x07c,
		0x201, 0x203, 0x204, 0x205, 0x206, 0x207,
		0x219, 0x220, 0x223, 0x224, 0x225, 0x227, 0x229,
		0x400, 0x401, 0x402, 0x403, 0x404, 0x405, 0x406,
		0x500, 0x527, 0x528, 0x54f,
		0xc81f, 0xc85f, 0xc96f,
	};
	char line[128];
	unsigned int i, n = 0;
	u8 v;

	dev_info(&c->spi->dev, "CS42 FINAL %s:\n", tag);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		if (cs42l81_read(c, regs[i], &v))
			continue;
		n += scnprintf(line + n, sizeof(line) - n, "%03x=%02x ",
			       regs[i], v);
		if (n > 90 || i + 1 == ARRAY_SIZE(regs)) {
			dev_info(&c->spi->dev, "  %s\n", line);
			n = 0;
			line[0] = '\0';
		}
	}
}

/* OSOS sub_5706F4(mode) — route counts / bases into graph state. */
static int cs42_5706f4_route_state(struct cs42l81 *c, int mode)
{
	struct cs42_graph_state *g = &c->graph;

	memset(g, 0, sizeof(*g));
	g->mode = mode;

	switch (mode) {
	case 1:
		g->count_l = 0;
		g->count_r = 0;
		g->base_l = 0;
		g->base_r = 0;
		break;
	case 3:
		g->count_l = 0;
		g->count_r = 0;
		g->base_l = 160;
		g->base_r = 160;
		break;
	case 4:
		g->count_l = 2;
		g->count_r = 0;
		g->base_l = 160;
		g->base_r = 160;
		break;
	default:
		dev_err(&c->spi->dev, "unsupported RE graph mode %d\n", mode);
		return -EINVAL;
	}

	/* idx_* not assigned in visible 5706F4; BSS-style zero + log. */
	dev_info(&c->spi->dev,
		 "CS42 5706F4: mode=%d count_l=%u count_r=%u base_l=%u base_r=%u\n",
		 mode, g->count_l, g->count_r, g->base_l, g->base_r);
	return 0;
}

/* OSOS sub_440AA4(a,b) — plain unsigned divide for tap math. */
static u32 cs42_440aa4_udiv(u32 num, u32 den)
{
	if (!den)
		return 0;
	return num / den;
}

/* OSOS sub_174E7C — compute tap_l / tap_r (+ optional accum). */
static int cs42_174e7c_compute_taps(struct cs42l81 *c)
{
	struct cs42_graph_state *g = &c->graph;
	u32 table_v;

	/*
	 * OSOS: v0 = *(u32 *)(0x892A068 + 4 * idx_l_hi).
	 * Recovered: table[0]==0 and idx_hi never written → always 0.
	 * graph_table_v overrides only for lab experiments.
	 */
	if (graph_table_v)
		table_v = graph_table_v;
	else if (g->idx_l_hi < ARRAY_SIZE(cs42_osos_graph_table_892a068))
		table_v = cs42_osos_graph_table_892a068[g->idx_l_hi];
	else
		table_v = 0;

	if (g->count_l)
		g->accum_l = (g->idx_l_lo + 1) * table_v;
	g->tap_l = cs42_440aa4_udiv(g->base_l + g->accum_l * g->count_l + 159,
				    160) + 2;

	if (g->count_r)
		g->accum_r = (g->idx_r_lo + 1) * table_v;
	g->tap_r = cs42_440aa4_udiv(g->base_r + g->accum_r * g->count_r + 159,
				    160) + 1;

	if (graph_tap_l_override >= 0 && graph_tap_l_override <= 0xff)
		g->tap_l = graph_tap_l_override;
	if (graph_tap_r_override >= 0 && graph_tap_r_override <= 0xff)
		g->tap_r = graph_tap_r_override;

	dev_info(&c->spi->dev,
		 "CS42 174E7C: tap_l=%u tap_r=%u accum_l=%u accum_r=%u table_v=%u (RE BSS)\n",
		 g->tap_l, g->tap_r, g->accum_l, g->accum_r, table_v);
	return 0;
}

/* OSOS sub_174E38(side) — program 0x529+ / 0x533+ from idx nibbles. */
static int cs42_174e38_program_range(struct cs42l81 *c, int side)
{
	struct cs42_graph_state *g = &c->graph;
	u16 reg;
	u8 count;
	u8 val;
	int i, ret;

	if (side) {
		reg = 0x533;
		count = g->count_r;
		val = g->idx_r_hi | (g->idx_r_lo << 4);
	} else {
		reg = 0x529;
		count = g->count_l;
		val = g->idx_l_hi | (g->idx_l_lo << 4);
	}

	for (i = 0; i < count; i++) {
		ret = cs42l81_rmw(c, reg + i, 0x3f, val);
		if (ret)
			return ret;
	}

	dev_info(&c->spi->dev,
		 "CS42 174E38 side=%d reg=0x%03x count=%u val=0x%02x\n",
		 side, reg, count, val);
	return 0;
}

static int cs42_write_slot3(struct cs42l81 *c, u16 reg, u8 a, u8 b, u8 cbyte)
{
	int ret;

	ret = cs42l81_write(c, reg + 0, a);
	if (ret)
		return ret;
	ret = cs42l81_write(c, reg + 1, b);
	if (ret)
		return ret;
	return cs42l81_write(c, reg + 2, cbyte);
}

/* OSOS sub_165BD4(side) — dynamic 11-slot triple writer. */
static int cs42_165bd4_build_slots(struct cs42l81 *c, int side)
{
	struct cs42_graph_state *g = &c->graph;
	u8 count;
	u8 source;
	u16 base_reg;
	u16 base_val;
	u16 gain;
	u8 term;
	int i, ret;

	if (side) {
		count = g->count_r;
		source = 10;
		base_reg = 0x428;
		base_val = g->base_r;
		gain = g->accum_r;
		term = 0xff;
	} else {
		count = g->count_l;
		source = 0;
		base_reg = 0x407;
		base_val = g->base_l;
		gain = g->accum_l;
		term = 0xfe;
	}

	for (i = 0; i < 11; i++) {
		u16 reg = base_reg + 3 * i;
		u8 a, b, cc;

		if (i < count) {
			a = source++;
			b = gain >> 8;
			cc = gain & 0xff;
		} else if (i == count && base_val) {
			a = term;
			b = base_val >> 8;
			cc = base_val & 0xff;
		} else if (i == 10 && !base_val) {
			a = term;
			b = 0;
			cc = 0;
		} else {
			a = source++;
			b = 0;
			cc = 0;
		}

		ret = cs42_write_slot3(c, reg, a, b, cc);
		if (ret)
			return ret;
	}

	dev_info(&c->spi->dev,
		 "CS42 165BD4 side=%d base=0x%03x count=%u base_val=%u gain=%u\n",
		 side, base_reg, count, base_val, gain);
	return 0;
}

static void cs42_log_graph_snapshot(struct cs42l81 *c, const char *tag)
{
	struct cs42_graph_state *g = &c->graph;
	u8 v;
	u16 r;
	char line[96];
	int n, i;

	if (!dump_regs)
		return;

	dev_info(&c->spi->dev,
		 "CS42 GRAPH %s: mode=%d tap_l=%u tap_r=%u status528=%02x\n",
		 tag, g->mode, g->tap_l, g->tap_r, g->status_528);

	n = 0;
	for (r = 0x400; r <= 0x406; r++) {
		if (cs42l81_read(c, r, &v))
			break;
		n += scnprintf(line + n, sizeof(line) - n, "%03x=%02x ", r, v);
	}
	dev_info(&c->spi->dev, "CS42 GRAPH hdr: %s\n", line);

	for (i = 0; i < 22; i++) {
		u16 base = 0x407 + 3 * i;

		if (cs42l81_read(c, base, &v))
			break;
		n = scnprintf(line, sizeof(line), "%03x:", base);
		n += scnprintf(line + n, sizeof(line) - n, " %02x", v);
		if (!cs42l81_read(c, base + 1, &v))
			n += scnprintf(line + n, sizeof(line) - n, " %02x", v);
		if (!cs42l81_read(c, base + 2, &v))
			n += scnprintf(line + n, sizeof(line) - n, " %02x", v);
		dev_info(&c->spi->dev, "CS42 GRAPH slot%02d %s\n", i, line);
	}

	n = 0;
	for (r = 0x529; r <= 0x534; r++) {
		if (cs42l81_read(c, r, &v))
			break;
		n += scnprintf(line + n, sizeof(line) - n, "%03x=%02x ", r, v);
	}
	dev_info(&c->spi->dev, "CS42 GRAPH 529..: %s\n", line);

	if (!cs42l81_read(c, 0x54f, &v))
		dev_info(&c->spi->dev, "CS42 GRAPH 54f=%02x\n", v);
}

/* RetailOS sub_5707D8 — exact static graph (literal table + verify). */
static int cs42_build_play_graph_static(struct cs42l81 *c)
{
	int ret;

	ret = cs42_graph_begin(c, 33);
	if (ret)
		return ret;

	ret = cs42_write_table(c, cs42_static_5707d8,
			       ARRAY_SIZE(cs42_static_5707d8));
	if (ret)
		goto out;

	msleep(100); /* sub_43E006(100) → RTOS sleep */
	ret = cs42l81_write(c, 0x0500, 0x05);
	if (ret)
		goto out;
	cs42l81_read(c, 0x0528, &c->graph.status_528);
	c->graph.mode = 0;
	c->graph.tap_l = 0x09;
	c->graph.tap_r = 0x08;

	cs42_verify_5707d8(c);
	cs42_log_graph_snapshot(c, "post_5707D8");

out:
	cs42_graph_end(c, 33);
	if (!ret)
		dev_info(&c->spi->dev,
			 "CS42 5707D8 static exact: taps=9/8 status528=%02x\n",
			 c->graph.status_528);
	return ret;
}

/*
 * RetailOS sub_570620 dynamic branch (8A8FB58 == NULL):
 *   10B4EA → 5706F4 → 174E7C → 174E38×2 → 54F/401/402..406 → 165BD4×2
 *   → 401 bit1=2 → sleep 100 → read 528 → 1042FC
 */
static int cs42_build_play_graph_retailos(struct cs42l81 *c, int route_mode)
{
	int ret;

	if (route_mode == 0)
		return cs42_build_play_graph_static(c);

	dev_info(&c->spi->dev,
		 "CS42 570620 dynamic mode=%d (BSS table/idx=0) force_gate=%d\n",
		 route_mode, graph_force_gate);

	/*
	 * RetailOS 41F944 gate: if route_present!=1 or busy, 570620 calls
	 * 42D364(0) and returns 17. Bring-up forces pass.
	 */
	if (!graph_force_gate) {
		dev_warn(&c->spi->dev,
			 "graph_force_gate=0 — dynamic path may tear down via 42D364(0)\n");
	}

	ret = cs42_graph_begin(c, 33);
	if (ret)
		return ret;

	ret = cs42_5706f4_route_state(c, route_mode);
	if (ret)
		return ret;

	ret = cs42_174e7c_compute_taps(c);
	if (ret)
		return ret;

	ret = cs42_174e38_program_range(c, 0);
	if (ret)
		return ret;
	ret = cs42_174e38_program_range(c, 1);
	if (ret)
		return ret;

	ret = cs42l81_rmw(c, 0x054f, 0xf0, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0401, 0x01, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0401, 0x02, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0402, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0403, c->graph.tap_l);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0404, c->graph.tap_r);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0405, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0406, 0x00);
	if (ret)
		return ret;

	ret = cs42_165bd4_build_slots(c, 0);
	if (ret)
		return ret;
	ret = cs42_165bd4_build_slots(c, 1);
	if (ret)
		return ret;

	ret = cs42l81_rmw(c, 0x0401, 0x02, 0x02);
	if (ret)
		return ret;

	msleep(100);

	ret = cs42l81_read(c, 0x0528, &c->graph.status_528);
	if (ret)
		return ret;

	ret = cs42_graph_end(c, 33);
	if (ret)
		return ret;

	dev_info(&c->spi->dev,
		 "CS42 graph built: mode=%d tap_l=%u tap_r=%u status528=%02x\n",
		 route_mode, c->graph.tap_l, c->graph.tap_r, c->graph.status_528);
	return 0;
}

static void cs42_maybe_debug_slot2_gain(struct cs42l81 *c)
{
	if (!graph_slot2_gain)
		return;
	cs42l81_write(c, 0x0410, 0x02);
	cs42l81_write(c, 0x0411, 0x01);
	cs42l81_write(c, 0x0412, 0xe0);
	dev_info(&c->spi->dev, "CS42 DEBUG graph_slot2_gain baked at 0x410\n");
}


/* OSOS sub_F141C — HP analog mute via 0x527 (sub_43CDB4 reg 1319). */
static int cs42_f141c(struct cs42l81 *c, int on)
{
	return cs42l81_write(c, 0x0527, on ? 0x60 : 0xff);
}

/*
 * Play-side F141C(1): 527=0x60 before 570620. Final 401&3=2 at post_iis.
 * Stop-side F141C(0): 527=0xFF; 401 bit0=1 handled in 42D364(0).
 */
static int cs42_f141c_play_unmute(struct cs42l81 *c, bool play)
{
	if (play)
		return cs42_f141c(c, 1);
	return cs42_f141c(c, 0);
}

/* OSOS sub_F1444 — meter soft-ramp pulse on 0x51E/0x523 bit5. */
static void cs42_f1444(struct cs42l81 *c)
{
	cs42l81_rmw(c, 0x051e, 0x20, 0x20);
	cs42l81_rmw(c, 0x051e, 0x20, 0x00);
	cs42l81_rmw(c, 0x0523, 0x20, 0x20);
	cs42l81_rmw(c, 0x0523, 0x20, 0x00);
}

/*
 * OSOS sub_42D364(0) stop path:
 *   F141C(0); 401 bit0=1; 401 bit1=0; F1444; (500E14 = CoreAudio state)
 */
static int cs42_42d364_stop(struct cs42l81 *c)
{
	int ret;

	ret = cs42_f141c(c, 0);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0401, 0x01, 0x01);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0401, 0x02, 0x00);
	if (ret)
		return ret;
	cs42_f1444(c);
	c->route_playing = false;
	c->play_started = false;
	dev_info_ratelimited(&c->spi->dev,
			     "42D364(0) stop: 527=FF 401 bit0=1 bit1=0 F1444\n");
	return 0;
}

/* ASP lock + final play latch: F141C(1); optional 401&3=2. */
static int cs42_play_unmute(struct cs42l81 *c)
{
	int ret;

	ret = cs42_f141c_play_unmute(c, true);
	if (ret)
		return ret;
	if (post_iis_401_rmw) {
		ret = cs42l81_rmw(c, 0x0401, 0x03, 0x02);
		if (ret)
			return ret;
	}
	c->route_playing = true;
	return 0;
}

/*
 * OSOS sub_570620(1) play graph — static 5707D8 (mode 0) or dynamic path.
 * Caller must have already done F141C(1) per 42D364(1).
 */
static int cs42_570620_play_graph(struct cs42l81 *c, int mode)
{
	int ret;

	if (audio_route != 0) {
		dev_warn(&c->spi->dev,
			 "audio_route=%d (USB/BT/MB) — forcing HP graph\n",
			 audio_route);
	}

	ret = cs42_build_play_graph_retailos(c, graph_mode);
	if (ret)
		return ret;
	cs42_maybe_debug_slot2_gain(c);
	dev_info(&c->spi->dev,
		 "CS42 570620(play): graph=%s taps=%u/%u latched (mode param=%d)\n",
		 graph_mode == 0 ? "static" : "dynamic",
		 c->graph.tap_l, c->graph.tap_r, mode);
	return 0;
}

static bool cs42_headset_ready(void);

/*
 * Do not fold into codec prepare — this is the play lifecycle latch.
 */
static int cs42_retailos_play_start(struct cs42l81 *c)
{
	int ret;

	if (!cs42_headset_ready()) {
		dev_warn(&c->spi->dev,
			 "headset not ready (8925CF4) — RetailOS would 42D364(0)\n");
		if (!force_headset)
			return -ENODEV;
	}

	ret = cs42_f141c_play_unmute(c, true);
	if (ret)
		return ret;

	ret = cs42_570620_play_graph(c, 1);
	if (ret)
		return ret;

	cs42_log_graph_snapshot(c, "post_play_start");
	cs42l81_log_start_state(c, "play_start");
	if (dump_regs)
		cs42_log_final_state(c, "play_start");
	c->play_started = true;
	c->route_playing = true;
	dev_info(&c->spi->dev, "CS42 RetailOS play_start complete\n");
	return 0;
}

static int cs42_retailos_play_stop(struct cs42l81 *c)
{
	int ret;

	ret = cs42_42d364_stop(c);
	if (!ret)
		cs42l81_log_start_state(c, "play_stop");
	return ret;
}

static bool cs42_headset_ready(void)
{
	int (*ready)(void);
	int r;

	if (force_headset)
		return true;
	ready = (int (*)(void))__symbol_get("apple_mikeybus_headset_ready");
	if (!ready) {
		ready = (int (*)(void))__symbol_get("apple_mikeybus_jack_present");
		if (!ready)
			return true; /* no mikey module — analog HP path */
		r = ready();
		__symbol_put("apple_mikeybus_jack_present");
		if (r < 0)
			return true; /* loaded but unbound (uart2 disabled) */
		return r > 0;
	}
	r = ready();
	__symbol_put("apple_mikeybus_headset_ready");
	/*
	 * -ENODEV: module loaded, serdev never probed (uart2 status=disabled).
	 * That is not "open circuit". Blocking DAI here is the -19 bug.
	 * 0: resistor task measured open circuit.
	 * 1: identified accessory or force_plugged / unmeasured-ready.
	 */
	if (r < 0)
		return true;
	return r > 0;
}

/* RE D3280(3)/audio_on HSDET pulse — tip/ring sense + 0x0B type read. */
static void cs42_hsdet_pulse(struct cs42l81 *c)
{
	u8 r220 = 0, r2f = 0, r0b = 0, r08 = 0, r09 = 0;
	unsigned int j;

	cs42l81_rmw(c, 0x0073, 0xc3, 0x00);
	cs42l81_rmw(c, 0x0073, 0xc0, 0xc0);
	cs42l81_rmw(c, 0x0079, 0x60, 0x00);
	cs42l81_read(c, 0x0220, &r220);
	cs42l81_rmw(c, 0x0220, 0x40, 0x40);
	msleep(1);
	cs42l81_rmw(c, 0x0009, 0xc0, 0xc0);
	for (j = 0; j < 3; j++) {
		msleep(1);
		cs42l81_read(c, 0x002f, &r2f);
		if (r2f & 0x40)
			break;
	}
	cs42l81_read(c, 0x000b, &r0b);
	cs42l81_rmw(c, 0x0009, 0xc0, 0x80);
	cs42l81_rmw(c, 0x0220, 0x40, r220 & 0x40);
	cs42l81_read(c, 0x0008, &r08);
	cs42l81_read(c, 0x0009, &r09);
	dev_info(&c->spi->dev,
		 "HSDET 0x0B=0x%02x type=%u 0x2F=0x%02x 0x08=0x%02x 0x09=0x%02x\n",
		 r0b, r0b & 3, r2f, r08, r09);
}

static void cs42_jack_poll_stop(struct cs42l81 *c)
{
	c->jack_poll_active = false;
	cancel_delayed_work_sync(&c->jack_work);
}

static void cs42_jack_workfn(struct work_struct *work)
{
	struct cs42l81 *c = container_of(work, struct cs42l81, jack_work.work);
	bool present;
	int jack;

	mutex_lock(&c->lock);
	if (!c->jack_poll_active)
		goto out_unlock;

	jack = -ENODEV;
	{
		int (*jp)(void) = (int (*)(void))
			__symbol_get("apple_mikeybus_jack_present");

		if (jp) {
			jack = jp();
			__symbol_put("apple_mikeybus_jack_present");
		}
	}
	present = force_headset || jack != 0;
	if (jack == 0 && c->route_playing) {
		dev_info(&c->spi->dev, "jack unplug -> 42D364(0)\n");
		cs42_42d364_stop(c);
		cs42_hsdet_pulse(c);
	} else if (jack > 0 && !c->jack_last_present && !c->route_playing) {
		dev_info(&c->spi->dev, "jack plug -> re-arm HSDET\n");
		cs42_hsdet_pulse(c);
	}
	c->jack_last_present = present;
	if (c->jack_poll_active && jack_poll_ms)
		schedule_delayed_work(&c->jack_work,
				      msecs_to_jiffies(jack_poll_ms));
out_unlock:
	mutex_unlock(&c->lock);
}

static void cs42_jack_poll_start(struct cs42l81 *c)
{
	if (!jack_poll_ms)
		return;
	c->jack_poll_active = true;
	c->jack_last_present = cs42_headset_ready();
	cancel_delayed_work(&c->jack_work);
	schedule_delayed_work(&c->jack_work, msecs_to_jiffies(jack_poll_ms));
}

/*
 * OSOS sub_400330 → sub_3FA0E0(551=0x227, value&0x7f).
 * D3280(4) calls this with 64 then 65 before final 0x229=0x41.
 */
static int cs42l81_set_output_gain(struct cs42l81 *c, u8 val)
{
	if (val & 0x40)
		val |= 0x80;
	return cs42l81_write(c, 0x0227, val);
}

/*
 * OSOS sub_D2F64(271) — output-path “on” mixer/mode blast (via D2D2C(1)).
 * Values from Hex-Rays of osos.dec.bin.ida.c — do not invent.
 */
static int cs42l81_apply_mode_271(struct cs42l81 *c)
{
	int ret;

	ret = cs42l81_rmw(c, 0x0006, 0x04, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0220, 0x28, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x000d, 0x03, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0206, 0x3f, 0x3d);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0207, 0x3f, 0x3d);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0205, 0xff, 0x5a);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0204, 0x03, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0206, 0xc0, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0207, 0xc0, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0203, 0xc0, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x000e, 0x40, 0x40);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x011f, 0x3f, 0x1c);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0120, 0x3f, 0x1c);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x012e, 0xff, 0xaa);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x000e, 0x40, 0x00);
	if (ret)
		return ret;
	msleep(105);
	return 0;
}

/* OSOS sub_D2F64(6) — output-path “off” companion (via D2D2C(0)). */
static int cs42l81_apply_mode_6(struct cs42l81 *c)
{
	int ret;

	ret = cs42l81_rmw(c, 0x0006, 0x04, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0220, 0x28, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x000d, 0x03, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0206, 0x3f, 0x34);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0207, 0x3f, 0x34);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0204, 0x03, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0206, 0xc0, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0207, 0xc0, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0203, 0xc0, 0xc0);
	if (ret)
		return ret;
	msleep(60);
	return 0;
}

/* OSOS sub_D2D2C(1) — enable HP/output path before D3280(4). */
static int cs42l81_output_path_enable(struct cs42l81 *c)
{
	int ret;

	ret = cs42l81_rmw(c, 0x0206, 0x3f, 0x08);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0207, 0x3f, 0x08);
	if (ret)
		return ret;
	ret = cs42l81_apply_mode_271(c);
	if (ret)
		return ret;
	dev_info(&c->spi->dev, "D2D2C(1) / output_path_enable complete\n");
	return 0;
}

/* OSOS sub_D2D2C(0). Kept for teardown; HP bring-up uses enable only. */
static int __maybe_unused cs42l81_output_path_disable(struct cs42l81 *c)
{
	int ret;

	ret = cs42l81_apply_mode_6(c);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0206, 0x3f, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0207, 0x3f, 0x00);
	if (ret)
		return ret;
	dev_info(&c->spi->dev, "D2D2C(0) / output_path_disable complete\n");
	return 0;
}

/*
 * OSOS sub_D3280(a1==4) — active HP output configuration (RE-backed).
 * Do not treat D3280(3) alone as playback-active.
 * Sequence includes sub_400330(64)/(65) before final 0x229=0x41.
 */
static int cs42l81_state_4_output_on(struct cs42l81 *c)
{
	int ret;

	ret = cs42l81_rmw(c, 0x0007, 0x40, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0219, 0x78, 0x78);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0229, 0x40);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0006, 0x01, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0201, 0xe0, 0x40);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0xc81f, 0xff);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0xc85f, 0x0f);
	if (ret)
		return ret;
	/* RE state 4 base is 0x0E; glass A/B via c96f_final (try 0x1E). */
	ret = cs42l81_write(c, 0xc96f, (u8)(c96f_final & 0xff));
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0223, 0x08);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0224, 0x09);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0225, 0x00);
	if (ret)
		return ret;
	/*
	 * Exact D3280(4) RE: 400330(64)/400330(65) then 229=0x41.
	 * Optional glass rail nudge (c96f_final=0x1e) keeps prior 219 lo3 dance.
	 */
	if ((c96f_final & 0xff) == 0x1e) {
		ret = cs42l81_rmw(c, 0x0219, 0x07, 0x01);
		if (ret)
			return ret;
		msleep(100);
		ret = cs42l81_write(c, 0xc96f, 0x1e);
		if (ret)
			return ret;
	}
	/* D3280(4): 3FA0E0(553,64) before 400330 pair on 227. */
	ret = cs42l81_write(c, 0x0229, 0x40);
	if (ret)
		return ret;
	ret = cs42l81_set_output_gain(c, 64);
	if (ret)
		return ret;
	ret = cs42l81_set_output_gain(c, 65);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x0229, 0x41);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x000e, 0xc0, 0x40);
	if (ret)
		return ret;

	dev_info(&c->spi->dev, "D3280(4) / output_on complete\n");
	return 0;
}

/*
 * OSOS sub_D3280(a1==3) — headset detect / pre-output (not full play).
 */
static int cs42l81_state_3_headset_detect(struct cs42l81 *c)
{
	u8 r74 = 0, r7b = 0, r7c = 0, r0f = 0, r2f = 0;
	int ret;

	ret = cs42l81_rmw(c, 0x0007, 0x40, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0006, 0x40, 0x00);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0220, 0x28, 0x28);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x000f, 0x80, 0x80);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x0075, 0x40, 0x40);
	if (ret)
		return ret;
	cs42l81_read(c, 0x0074, &r74);
	cs42l81_write(c, 0x0074, (r74 & 0xe7) | 0x08);
	cs42l81_read(c, 0x007b, &r7b);
	cs42l81_read(c, 0x007c, &r7c);
	cs42l81_write(c, 0x0074, r74);
	cs42l81_rmw(c, 0x0075, 0x40, 0x00);
	cs42l81_rmw(c, 0x0075, 0x80, 0x80);
	cs42l81_read(c, 0x000f, &r0f);
	cs42l81_read(c, 0x002f, &r2f);
	dev_info(&c->spi->dev,
		 "D3280(3) 0x0F=0x%02x 0x2F=0x%02x 0x7B=0x%02x 0x7C=0x%02x\n",
		 r0f, r2f, r7b, r7c);
	return 0;
}

/* OSOS sub_D34C0 / 183138 rate programming. */
static int cs42l81_set_rate(struct cs42l81 *c, unsigned int rate)
{
	const struct n31_rate_cfg *r = n31_find_rate(rate);
	u8 code;
	int ret;

	if (!r)
		return -EINVAL;
	code = r->cs42_rate_code;

	ret = cs42l81_rmw(c, 0x000e, 0xc0, 0xc0);
	if (ret)
		return ret;
	ret = cs42l81_rmw(c, 0x000f, 0x0f, code);
	if (ret)
		return ret;
	ret = cs42l81_write(c, 0x012f, (u8)(code | (code << 4)));
	if (ret)
		return ret;

	/*
	 * sub_183138 branches on rate code:
	 *   code==12 (48 kHz): 0x10B=8, 0x10C=9, 0x131 bit0=1
	 *   else (e.g. 10=44.1): 0x121=8, 0x122=9, 0x130 lo=code,
	 *                        0x131 bit0=0, 0x10B=4, 0x10C=0x33
	 * Linux previously always took the 48 kHz arm while IIS ran
	 * 44.1 (CLKDIV 272) → ASP/SRC mismatch → pulsed noise.
	 */
	if (code == 12) {
		ret = cs42l81_write(c, 0x010b, 0x08);
		if (ret)
			return ret;
		ret = cs42l81_write(c, 0x010c, 0x09);
		if (ret)
			return ret;
		ret = cs42l81_rmw(c, 0x0131, 0x01, 0x01);
		if (ret)
			return ret;
	} else {
		ret = cs42l81_write(c, 0x0121, 0x08);
		if (ret)
			return ret;
		ret = cs42l81_write(c, 0x0122, 0x09);
		if (ret)
			return ret;
		ret = cs42l81_rmw(c, 0x0130, 0x0f, code);
		if (ret)
			return ret;
		ret = cs42l81_rmw(c, 0x0131, 0x01, 0x00);
		if (ret)
			return ret;
		ret = cs42l81_write(c, 0x010b, 0x04);
		if (ret)
			return ret;
		ret = cs42l81_write(c, 0x010c, 0x33);
		if (ret)
			return ret;
	}

	ret = cs42l81_rmw(c, 0x000e, 0xc0, 0x40);
	if (ret)
		return ret;

	c->rate = rate;
	dev_info(&c->spi->dev,
		 "CS42 set_rate %u code=%u 10B=%02x 10C=%02x 131bit0=%d\n",
		 rate, code, code == 12 ? 0x08 : 0x04,
		 code == 12 ? 0x09 : 0x33, code == 12 ? 1 : 0);
	return 0;
}

/*
 * Codec prepare — rails, rate, D2D2C, D3280(4). No 42D364 play graph.
 * RetailOS play latch is cs42_retailos_play_start() at transport START.
 */
static int cs42_codec_prepare(struct cs42l81 *c, unsigned int rate)
{
	u8 st = 0, r219 = 0;
	int ret;
	int jack = -ENODEV;
	int (*mikey_jack)(void);

	if (!cs42_headset_ready()) {
		dev_warn(&c->spi->dev,
			 "headset not ready (8925CF4) — RetailOS gates 570620\n");
		if (!force_headset)
			return -ENODEV;
	}

	mikey_jack = (int (*)(void))__symbol_get("apple_mikeybus_jack_present");
	if (mikey_jack) {
		jack = mikey_jack();
		__symbol_put("apple_mikeybus_jack_present");
	}
	if (jack < 0)
		dev_info(&c->spi->dev,
			 "MikeyBus unbound (uart2 disabled) — analog HP not gated\n");
	else if (jack == 0)
		dev_warn(&c->spi->dev,
			 "MikeyBus open circuit — force_headset=1 to override\n");
	else
		dev_info(&c->spi->dev, "MikeyBus jack present\n");

	ret = cs42l81_state_3_headset_detect(c);
	if (ret)
		return ret;

	if (!rate)
		rate = cs42_pick_rate(c, 0);
	ret = cs42l81_set_rate(c, rate);
	if (ret)
		return ret;

	ret = cs42l81_output_path_enable(c);
	if (ret)
		return ret;

	ret = cs42l81_state_4_output_on(c);
	if (ret)
		return ret;

	/* Play-object companions (40C028/54F/220) — safe before graph latch. */
	cs42l81_rmw(c, 0x0075, 0x3f, 0x3c);
	cs42l81_rmw(c, 0x054f, 0xf0, 0x00);
	cs42l81_rmw(c, 0x0220, 0x28, 0x28);

	cs42_hsdet_pulse(c);

	cs42l81_read(c, 0x0227, &st);
	cs42l81_read(c, 0x0219, &r219);
	cs42l81_push_pcm_q8(c->user_vol);
	cs42_log_graph_snapshot(c, "pre_play");
	cs42l81_log_start_state(c, "codec_prepare");
	dev_info(&c->spi->dev,
		 "codec_prepare 0x227=0x%02x 0x219=0x%02x rate=%u graph_mode=%d\n",
		 st, r219, rate, graph_mode);
	c->codec_prepared = true;
	cs42_jack_poll_start(c);

	/* audio_path_mode=0: legacy debug — graph folded into prepare. */
	if (audio_path_mode == 0) {
		ret = cs42_f141c_play_unmute(c, true);
		if (ret)
			return ret;
		ret = cs42_570620_play_graph(c, 1);
		if (ret)
			return ret;
		c->play_started = true;
		c->route_playing = true;
	}
	return 0;
}

/* Sysfs / legacy: full prepare + play_start (ALSA uses split lifecycle). */
static int cs42l81_audio_on(struct cs42l81 *c)
{
	int ret;

	ret = cs42_codec_prepare(c, cs42_pick_rate(c, 0));
	if (ret)
		return ret;
	if (audio_path_mode == 0) {
		/* Legacy debug: graph in prepare (non-RetailOS order). */
		ret = cs42_f141c_play_unmute(c, true);
		if (ret)
			return ret;
		ret = cs42_570620_play_graph(c, 1);
		if (ret)
			return ret;
		c->play_started = true;
		c->route_playing = true;
		return 0;
	}
	return cs42_retailos_play_start(c);
}

/* Legacy mute helper — prefer cs42_f141c_play_unmute / cs42_retailos_play_stop. */
static int __maybe_unused cs42l81_set_mute(struct cs42l81 *c, int mute)
{
	if (mute) {
		cs42l81_write(c, 0x0527, 0xff);
		cs42l81_rmw(c, 0x0401, 0x03, 0x01);
		/* F1444: pulse meter/soft-ramp bits. */
		cs42l81_rmw(c, 0x051e, 0x20, 0x20);
		cs42l81_rmw(c, 0x051e, 0x20, 0x00);
		cs42l81_rmw(c, 0x0523, 0x20, 0x20);
		cs42l81_rmw(c, 0x0523, 0x20, 0x00);
	} else {
		cs42l81_write(c, 0x0527, 0x60);
		cs42l81_rmw(c, 0x0401, 0x03, 0x02);
	}
	return 0;
}

static void cs42l81_push_pcm_q8(unsigned int vol)
{
	void (*set)(unsigned int);

	set = (void (*)(unsigned int))__symbol_get("s5l8740_set_user_vol_q8");
	if (set) {
		set(vol);
		__symbol_put("s5l8740_set_user_vol_q8");
	}
}

/* User vol mute during play: F141C(0) only — not full 42D364 teardown. */
static int cs42l81_apply_user_vol(struct cs42l81 *c)
{
	unsigned int q8 = c->dai_mute ? 0 : c->user_vol;

	cs42l81_push_pcm_q8(q8);
	if (q8 == 0)
		return cs42_f141c_play_unmute(c, false);
	if (c->play_started)
		return cs42_play_unmute(c);
	return cs42_f141c_play_unmute(c, true);
}

/* CONFIRMED_N31 readback set from CURSOR-N31-AUDIO-FIRST-SOUND-HANDOFF.md P0.1 */
static void cs42l81_log_start_state(struct cs42l81 *c, const char *tag)
{
	u8 r2f = 0, r401 = 0, r527 = 0, r219 = 0, rc96f = 0;

	cs42l81_read(c, 0x002f, &r2f);
	cs42l81_read(c, 0x0401, &r401);
	cs42l81_read(c, 0x0527, &r527);
	cs42l81_read(c, 0x0219, &r219);
	cs42l81_read(c, 0xc96f, &rc96f);
	dev_info(&c->spi->dev,
		 "CS42 %s: vol=%u/%u dai_mute=%d 2F=%02x 527=%02x 401=%02x 219=%02x C96F=%02x\n",
		 tag, c->user_vol, CS42L81_USER_VOL_MAX, c->dai_mute,
		 r2f, r527, r401, r219, rc96f);
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	unsigned int reg, val;
	int n;

	n = sscanf(buf, "%x %x", &reg, &val);
	if (n != 2 || reg > 0xffff || val > 0xff)
		return -EINVAL;
	mutex_lock(&c->lock);
	n = cs42l81_write(c, (u16)reg, (u8)val);
	mutex_unlock(&c->lock);
	return n ? n : count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sysfs_emit(buf, "write: echo \"RRRR VV\" > reg  (hex)\n");
}
static DEVICE_ATTR_RW(reg);

static ssize_t status_227_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	u8 st = 0;
	int ret;

	mutex_lock(&c->lock);
	ret = cs42l81_read(c, 0x0227, &st);
	mutex_unlock(&c->lock);
	if (ret)
		return ret;
	return sysfs_emit(buf, "0x%02x\n", st);
}
static DEVICE_ATTR_RO(status_227);

static ssize_t bringup_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	int ret;

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;

	mutex_lock(&c->lock);
	ret = cs42l81_bringup(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(bringup);

static ssize_t dump_graph_regs_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	int n = 0, ret;
	u16 r;
	u8 val;

	mutex_lock(&c->lock);
	n += scnprintf(buf + n, PAGE_SIZE - n,
		       "mode=%d tap_l=%u tap_r=%u status528=%02x\n",
		       c->graph.mode, c->graph.tap_l, c->graph.tap_r,
		       c->graph.status_528);
	for (r = 0x400; r <= 0x448; r++) {
		ret = cs42l81_read(c, r, &val);
		if (ret)
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "0x%04x: ERR %d\n", r, ret);
		else
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "0x%04x: 0x%02x\n", r, val);
		if (n >= PAGE_SIZE - 64)
			break;
	}
	for (r = 0x529; r <= 0x534; r++) {
		ret = cs42l81_read(c, r, &val);
		if (ret)
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "0x%04x: ERR %d\n", r, ret);
		else
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "0x%04x: 0x%02x\n", r, val);
	}
	for (r = 0x54f; r <= 0x54f; r++) {
		ret = cs42l81_read(c, r, &val);
		if (!ret)
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "0x%04x: 0x%02x\n", r, val);
	}
	ret = cs42l81_read(c, 0x528, &val);
	if (!ret)
		n += scnprintf(buf + n, PAGE_SIZE - n, "0x0528: 0x%02x\n", val);
	mutex_unlock(&c->lock);
	return n;
}
static DEVICE_ATTR_RO(dump_graph_regs);

static ssize_t dump_key_regs_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	static const u16 regs[] = {
		0x0227, 0x0229, 0x0219, 0xc96f, 0xc81f, 0xc85f,
		0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b,
		0x000c, 0x000d, 0x000e, 0x000f, 0x0012, 0x0019,
		0x0070, 0x0071, 0x0073, 0x0074, 0x0075, 0x0076,
		0x0079, 0x007a, 0x007b, 0x007c,
		0x002f, 0x0131, 0x0220, 0x0201, 0x0203, 0x0204,
		0x0205, 0x0206, 0x0207, 0x0222, 0x0223, 0x0224, 0x0225,
		0x011f, 0x0120, 0x012e, 0x0121, 0x0122, 0x012f,
		0x010b, 0x010c, 0x0529, 0x052a, 0x0533, 0x0534,
		0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0405, 0x0406,
		0x0407, 0x040a, 0x040d, 0x0410, 0x0425, 0x0428, 0x042b, 0x042e,
		0x0527, 0x051e, 0x0523, 0x054f,
		0x0500, 0x051f, 0x0520, 0x0521, 0x0524, 0x0525, 0x0528,
		0x001a, 0x001b, 0x001e, 0x001f,
		0x0034, 0x0035, 0x0039, 0x003a,
	};
	int i, n = 0, ret;
	u8 val;

	mutex_lock(&c->lock);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = cs42l81_read(c, regs[i], &val);
		if (ret) {
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "0x%04x: ERR %d\n", regs[i], ret);
		} else {
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "0x%04x: 0x%02x\n", regs[i], val);
		}
	}
	mutex_unlock(&c->lock);
	return n;
}
static DEVICE_ATTR_RO(dump_key_regs);

/* RetailOS user volume 0..256. 0x403/0x404 stay mixer taps. */
static ssize_t volume_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	unsigned int vol;
	int ret;

	if (kstrtouint(buf, 0, &vol))
		return -EINVAL;
	if (vol > CS42L81_USER_VOL_MAX)
		return -EINVAL;

	mutex_lock(&c->lock);
	c->user_vol = vol;
	ret = cs42l81_apply_user_vol(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}

static ssize_t volume_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	u8 tap_l = 0, tap_r = 0, analog = 0;
	int ra, rb, rc;

	mutex_lock(&c->lock);
	ra = cs42l81_read(c, 0x0403, &tap_l);
	rb = cs42l81_read(c, 0x0404, &tap_r);
	rc = cs42l81_read(c, 0x0527, &analog);
	mutex_unlock(&c->lock);
	if (ra || rb || rc)
		return sysfs_emit(buf, "read err %d/%d/%d\n", ra, rb, rc);
	return sysfs_emit(buf,
			  "user=%u/%u dai_mute=%d\n"
			  "0x403=0x%02x 0x404=0x%02x (taps, play 2/1)\n"
			  "0x527=0x%02x analog_mute=%d\n",
			  c->user_vol, CS42L81_USER_VOL_MAX, c->dai_mute,
			  tap_l, tap_r, analog, analog == 0xff);
}
static DEVICE_ATTR_RW(volume);

static ssize_t audio_on_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	int ret;

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	mutex_lock(&c->lock);
	ret = cs42l81_audio_on(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(audio_on);

static ssize_t mute_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	unsigned int v;
	int ret;

	if (kstrtouint(buf, 0, &v))
		return -EINVAL;
	mutex_lock(&c->lock);
	c->dai_mute = v ? 1 : 0;
	ret = cs42l81_apply_user_vol(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}

static ssize_t mute_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	u8 v = 0;
	int ret;

	mutex_lock(&c->lock);
	ret = cs42l81_read(c, 0x0527, &v);
	mutex_unlock(&c->lock);
	if (ret)
		return ret;
	return sysfs_emit(buf, "0x527=0x%02x mute=%d\n", v, v == 0xff);
}
static DEVICE_ATTR_RW(mute);

static ssize_t rreg_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	unsigned int reg;
	u8 val = 0;
	int ret;

	if (kstrtouint(buf, 0, &reg) || reg > 0xffff)
		return -EINVAL;
	mutex_lock(&c->lock);
	ret = cs42l81_read(c, (u16)reg, &val);
	mutex_unlock(&c->lock);
	if (ret)
		return ret;
	dev_info(&c->spi->dev, "rreg 0x%04x = 0x%02x\n", reg, val);
	return count;
}
static DEVICE_ATTR_WO(rreg);

/*
 * After IIS BCLK/LRCK run (RetailOS 26DDDE: 414FAE before sustained PCM).
 * Re-run 183138 clock regs and poll 0x2F bit6 for ASP sync (see asp_bit6_is_los).
 * LOS does not always self-recover — pulse 0x220 and retry clock prog.
 */
static void cs42l81_asp_clock_pulse(struct cs42l81 *c)
{
	cs42l81_rmw(c, 0x0220, 0x20, 0x00);
	udelay(50);
	cs42l81_rmw(c, 0x0220, 0x20, 0x20);
}

/* Re-apply full 183138 during ASP lock (not just 0x0E/0x0F/0x12F). */
static void cs42l81_asp_program_rate(struct cs42l81 *c)
{
	cs42l81_set_rate(c, cs42_pick_rate(c, c->rate));
}

int cs42l81_asp_hold_light(void);

static int cs42l81_asp_lock(struct cs42l81 *c)
{
	unsigned int attempt, i;
	u8 r2f = 0, r0e = 0, r0f = 0, r08 = 0, r09 = 0;

	for (attempt = 0; attempt < 5; attempt++) {
		if (attempt)
			cs42l81_asp_clock_pulse(c);
		cs42l81_asp_program_rate(c);
		for (i = 0; i < 80; i++) {
			cs42l81_read(c, 0x002f, &r2f);
			if (cs42l81_asp_synced(r2f))
				break;
			usleep_range(500, 1000);
		}
		if (cs42l81_asp_synced(r2f))
			break;
	}
	cs42l81_read(c, 0x0008, &r08);
	cs42l81_read(c, 0x0009, &r09);
	cs42l81_read(c, 0x000e, &r0e);
	cs42l81_read(c, 0x000f, &r0f);
	dev_info(&c->spi->dev,
		 "asp_lock 0x2F=0x%02x los=%d synced=%d 0x0E=0x%02x 0x0F=0x%02x 0x08=0x%02x 0x09=0x%02x\n",
		 r2f, asp_bit6_is_los, cs42l81_asp_synced(r2f), r0e, r0f, r08, r09);
	return cs42l81_asp_synced(r2f) ? 0 : -EAGAIN;
}

/*
 * Mid-stream LOS recovery: one 183138 pulse + short poll (~10 ms).
 * dma_tone calls this when 0x2F bit6 asserts LOS mid-tone.
 */
int cs42l81_asp_hold_light(void)
{
	struct cs42l81 *c = cs42l81_dev;
	unsigned int i;
	u8 r2f = 0, before = 0;
	int ret = -ENODEV;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	cs42l81_read(c, 0x002f, &before);
	if (cs42l81_asp_synced(before)) {
		ret = 0;
		goto out;
	}
	cs42l81_asp_program_rate(c);
	for (i = 0; i < 24; i++) {
		cs42l81_read(c, 0x002f, &r2f);
		if (cs42l81_asp_synced(r2f))
			break;
		usleep_range(400, 800);
	}
	if (cs42l81_asp_synced(r2f)) {
		ret = 0;
	} else {
		cs42l81_asp_clock_pulse(c);
		cs42l81_asp_program_rate(c);
		for (i = 0; i < 16; i++) {
			cs42l81_read(c, 0x002f, &r2f);
			if (cs42l81_asp_synced(r2f))
				break;
			usleep_range(400, 800);
		}
		ret = cs42l81_asp_synced(r2f) ? 0 : -EAGAIN;
	}
	dev_info(&c->spi->dev,
		 "asp_hold_light 0x2F 0x%02x->0x%02x ret=%d\n",
		 before, r2f, ret);
out:
	mutex_unlock(&c->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cs42l81_asp_hold_light);

int cs42l81_play_prepare(void)
{
	struct cs42l81 *c = cs42l81_dev;
	int ret;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	if (c->codec_prepared && !force_full_prepare) {
		mutex_unlock(&c->lock);
		return 0;
	}
	if (force_full_prepare && c->play_started)
		cs42_retailos_play_stop(c);
	ret = cs42_codec_prepare(c, cs42_pick_rate(c, c->rate));
	mutex_unlock(&c->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cs42l81_play_prepare);

int cs42l81_play_start(void)
{
	struct cs42l81 *c = cs42l81_dev;
	int ret;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	if (!c->codec_prepared) {
		ret = cs42_codec_prepare(c, cs42_pick_rate(c, c->rate));
		if (ret)
			goto out;
	}
	if (c->play_started && !force_full_prepare) {
		ret = 0;
		goto out;
	}
	if (force_full_prepare && c->play_started)
		cs42_retailos_play_stop(c);
	ret = cs42_retailos_play_start(c);
out:
	mutex_unlock(&c->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cs42l81_play_start);

int cs42l81_play_stop(void)
{
	struct cs42l81 *c = cs42l81_dev;
	int ret;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	ret = cs42_retailos_play_stop(c);
	if (!ret && force_full_prepare)
		c->codec_prepared = false;
	mutex_unlock(&c->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cs42l81_play_stop);

int cs42l81_get_audio_path_mode(void)
{
	return audio_path_mode;
}
EXPORT_SYMBOL_GPL(cs42l81_get_audio_path_mode);

/*
 * Called after IIS TXCOM kick. Telemetry on 0x2F; unmute unless asp_gate_unmute.
 */
int cs42l81_pre_iis_start(void)
{
	struct cs42l81 *c = cs42l81_dev;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	if (of_asp_slave)
		cs42l81_rmw(c, 0x000f, 0x80, 0x00);
	mutex_unlock(&c->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(cs42l81_pre_iis_start);

int cs42l81_post_iis_start(void)
{
	struct cs42l81 *c = cs42l81_dev;
	int probe;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	probe = cs42l81_asp_lock(c);
	/*
	 * Checkpoint-010 / handoff: do not gate HP unmute on dai_mute.
	 * ALSA mute_stream(1) on a prior close left dai_mute stuck, so
	 * dma_tone/post_iis kept 0x527=0xFF while ASP was locked and
	 * TXCOM=6 — silent jack with "perfect" digital telemetry.
	 * asp_gate_unmute=0 (default): always force 0x527=0x60 / 0x401&3=2.
	 */
	if (!asp_gate_unmute || !probe) {
		c->dai_mute = false;
		cs42l81_write(c, 0x0229, 0x41);
		cs42l81_write(c, 0xc96f, (u8)(c96f_final & 0xff));
		cs42_play_unmute(c);
		cs42l81_apply_user_vol(c);
	}
	cs42l81_log_start_state(c, "post_iis");
	cs42_log_final_state(c, "post_iis");
	mutex_unlock(&c->lock);
	return asp_gate_unmute ? probe : 0;
}
EXPORT_SYMBOL_GPL(cs42l81_post_iis_start);

static void cs42l81_asp_post_workfn(struct work_struct *work)
{
	cs42l81_post_iis_start();
}

void cs42l81_schedule_post_iis(void)
{
	struct cs42l81 *c = cs42l81_dev;

	if (!c)
		return;
	cancel_delayed_work(&c->asp_post_work);
	schedule_delayed_work(&c->asp_post_work, msecs_to_jiffies(25));
}
EXPORT_SYMBOL_GPL(cs42l81_schedule_post_iis);

void cs42l81_cancel_post_iis(void)
{
	struct cs42l81 *c = cs42l81_dev;

	if (!c)
		return;
	cancel_delayed_work_sync(&c->asp_post_work);
}
EXPORT_SYMBOL_GPL(cs42l81_cancel_post_iis);

static ssize_t probe_2f_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	unsigned int n, i;
	u8 r2f;

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	n = 10;
	if (kstrtouint(buf + 1, 0, &n) == 0 && n > 0 && n <= 32)
		; /* optional count after '1' */
	else
		n = 10;
	mutex_lock(&c->lock);
	for (i = 0; i < n; i++) {
		cs42l81_read(c, 0x002f, &r2f);
		dev_info(&c->spi->dev, "probe_2f[%u]=0x%02x synced=%d\n",
			 i, r2f, cs42l81_asp_synced(r2f));
	}
	mutex_unlock(&c->lock);
	return count;
}
static DEVICE_ATTR_WO(probe_2f);

static ssize_t force_play_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	mutex_lock(&c->lock);
	c->dai_mute = false;
	cs42l81_write(c, 0x0229, 0x41);
	cs42l81_write(c, 0xc96f, 0x1e);
	cs42l81_write(c, 0x0527, 0x60);
	cs42l81_rmw(c, 0x0401, 0x03, 0x02);
	cs42l81_apply_user_vol(c);
	cs42l81_log_start_state(c, "force_play");
	mutex_unlock(&c->lock);
	return count;
}
static DEVICE_ATTR_WO(force_play);

static ssize_t asp_lock_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	int ret;

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	mutex_lock(&c->lock);
	ret = cs42l81_asp_lock(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(asp_lock);

static struct attribute *cs42l81_attrs[] = {
	&dev_attr_reg.attr,
	&dev_attr_status_227.attr,
	&dev_attr_bringup.attr,
	&dev_attr_dump_key_regs.attr,
	&dev_attr_dump_graph_regs.attr,
	&dev_attr_volume.attr,
	&dev_attr_audio_on.attr,
	&dev_attr_probe_2f.attr,
	&dev_attr_force_play.attr,
	&dev_attr_asp_lock.attr,
	&dev_attr_mute.attr,
	&dev_attr_rreg.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cs42l81);

static int cs42l81_dai_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct cs42l81 *c = snd_soc_component_get_drvdata(dai->component);
	unsigned int rate = params_rate(params);
	int ret;

	mutex_lock(&c->lock);
	c->rate = rate;
	ret = cs42_codec_prepare(c, rate);
	mutex_unlock(&c->lock);
	dev_info(&c->spi->dev, "DAI hw_params rate=%u ret=%d (prepare only)\n",
		 rate, ret);
	return ret;
}

static int cs42l81_dai_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct cs42l81 *c = snd_soc_component_get_drvdata(dai->component);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* CPU IIS + DMA kick first; ASP lock runs sleepable in workqueue. */
		cs42l81_schedule_post_iis();
		dev_info(&c->spi->dev, "DAI trigger START (asp work scheduled)\n");
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* IIS CPU DAI owns 42D364 and post_iis cancel. Cancelling
		 * here aborts unmute if ALSA xruns 20-50ms after START.
		 */
		dev_info_ratelimited(&c->spi->dev,
				     "DAI trigger STOP (codec leave to IIS)\n");
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int cs42l81_dai_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct cs42l81 *c = snd_soc_component_get_drvdata(dai->component);

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;
	mutex_lock(&c->lock);
	c->dai_mute = mute ? 1 : 0;
	/* Do not F141C-mute on ALSA mute(1). Short START/STOP bursts were
	 * remuting HP 20-50ms after play_start. Real stop is 42D364 on IIS.
	 */
	if (!mute)
		cs42l81_apply_user_vol(c);
	mutex_unlock(&c->lock);
	dev_info_ratelimited(&c->spi->dev,
			     "DAI mute=%d user_vol=%u%s\n", mute, c->user_vol,
			     mute ? " (deferred)" : "");
	return 0;
}

static const struct snd_soc_dai_ops cs42l81_dai_ops = {
	.hw_params = cs42l81_dai_hw_params,
	.trigger = cs42l81_dai_trigger,
	.mute_stream = cs42l81_dai_mute_stream,
};

static int cs42l81_vol_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = CS42L81_USER_VOL_MAX;
	return 0;
}

static int cs42l81_vol_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = c->user_vol;
	return 0;
}

static void cs42l81_notify_master_vol(struct cs42l81 *c)
{
	struct snd_soc_component *comp = c->component;
	struct snd_kcontrol *kctl;

	if (!comp || !comp->card || !comp->card->snd_card)
		return;
	kctl = snd_soc_component_get_kcontrol(comp, "Master Playback Volume");
	if (!kctl)
		return;
	snd_ctl_notify(comp->card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &kctl->id);
}

/*
 * Vol± from gpio-s5l8740 (KEY_VOLUMEUP/DOWN) → Master Playback Volume.
 * Input softirq must not SPI; defer apply + ALSA notify to process context.
 */
static void cs42l81_vol_workfn(struct work_struct *work)
{
	struct cs42l81 *c = container_of(work, struct cs42l81, vol_work);
	int steps = atomic_xchg(&c->vol_steps, 0);
	int delta;
	unsigned int vol, prev;
	bool unmute = false;

	if (!steps)
		return;
	delta = steps * (int)CS42L81_VOL_STEP;

	mutex_lock(&c->lock);
	prev = c->user_vol;
	if (delta > 0) {
		vol = prev + (unsigned int)delta;
		if (vol > CS42L81_USER_VOL_MAX)
			vol = CS42L81_USER_VOL_MAX;
		if (c->dai_mute && vol > 0) {
			c->dai_mute = false;
			unmute = true;
		}
	} else {
		unsigned int down = (unsigned int)(-delta);

		vol = (prev > down) ? prev - down : 0;
	}
	if (vol != prev || unmute) {
		c->user_vol = vol;
		cs42l81_apply_user_vol(c);
	}
	mutex_unlock(&c->lock);

	if (vol != prev || unmute) {
		cs42l81_notify_master_vol(c);
		dev_info(&c->spi->dev,
			 "Vol%c → Master %u/%u%s\n",
			 delta > 0 ? '+' : '-', vol, CS42L81_USER_VOL_MAX,
			 unmute ? " (unmuted)" : "");
	}
}

static void cs42l81_input_event(struct input_handle *handle,
				unsigned int type, unsigned int code, int value)
{
	struct cs42l81 *c = handle->private;

	/* value 1 = press, 2 = autorepeat; ignore release */
	if (type != EV_KEY || value == 0 || !c)
		return;
	if (code == KEY_VOLUMEUP)
		atomic_add(1, &c->vol_steps);
	else if (code == KEY_VOLUMEDOWN)
		atomic_add(-1, &c->vol_steps);
	else
		return;
	schedule_work(&c->vol_work);
}

static int cs42l81_input_connect(struct input_handler *handler,
				 struct input_dev *dev,
				 const struct input_device_id *id)
{
	struct cs42l81 *c = container_of(handler, struct cs42l81,
					 input_handler);
	struct input_handle *handle;
	int err;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cs42l81-vol";
	handle->private = c;

	err = input_register_handle(handle);
	if (err)
		goto err_free;
	err = input_open_device(handle);
	if (err)
		goto err_unregister;

	dev_info(&c->spi->dev, "Vol± keys → Master Playback Volume (%s)\n",
		 dev->name ? dev->name : "input");
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return err;
}

static void cs42l81_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id cs42l81_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = {
			[BIT_WORD(KEY_VOLUMEUP)] =
				BIT_MASK(KEY_VOLUMEUP) | BIT_MASK(KEY_VOLUMEDOWN),
		},
	},
	{ },
};

static int cs42l81_vol_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);
	unsigned int vol = ucontrol->value.integer.value[0];
	int changed;

	if (vol > CS42L81_USER_VOL_MAX)
		return -EINVAL;
	mutex_lock(&c->lock);
	changed = vol != c->user_vol;
	c->user_vol = vol;
	cs42l81_apply_user_vol(c);
	mutex_unlock(&c->lock);
	return changed;
}

/* 1 = unmuted (ALSA convention), 0 = muted — mirrors sysfs mute / dai_mute. */
static int cs42l81_sw_info(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int cs42l81_sw_get(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = c->dai_mute ? 0 : 1;
	return 0;
}

static int cs42l81_sw_put(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);
	bool mute = !ucontrol->value.integer.value[0];
	int changed;

	mutex_lock(&c->lock);
	changed = mute != c->dai_mute;
	c->dai_mute = mute;
	cs42l81_apply_user_vol(c);
	mutex_unlock(&c->lock);
	return changed;
}

static const struct snd_kcontrol_new cs42l81_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Volume",
		.info = cs42l81_vol_info,
		.get = cs42l81_vol_get,
		.put = cs42l81_vol_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Switch",
		.info = cs42l81_sw_info,
		.get = cs42l81_sw_get,
		.put = cs42l81_sw_put,
	},
};

static struct snd_soc_dai_driver cs42l81_dai = {
	.name = "cs42l81-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cs42l81_dai_ops,
};

static int cs42l81_component_probe(struct snd_soc_component *component)
{
	struct cs42l81 *c = snd_soc_component_get_drvdata(component);

	c->component = component;
	return 0;
}

static const struct snd_soc_component_driver cs42l81_component = {
	.probe = cs42l81_component_probe,
	.idle_bias_on = 1,
	.endianness = 1,
	.controls = cs42l81_controls,
	.num_controls = ARRAY_SIZE(cs42l81_controls),
};

static int cs42l81_probe(struct spi_device *spi)
{
	struct cs42l81 *c;
	int ret;

	c = devm_kzalloc(&spi->dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->spi = spi;
	c->user_vol = CS42L81_USER_VOL_MAX;
	c->dai_mute = false;
	mutex_init(&c->lock);
	atomic_set(&c->vol_steps, 0);
	INIT_WORK(&c->vol_work, cs42l81_vol_workfn);
	INIT_DELAYED_WORK(&c->asp_post_work, cs42l81_asp_post_workfn);
	INIT_DELAYED_WORK(&c->jack_work, cs42_jack_workfn);
	spi_set_drvdata(spi, c);

	mutex_lock(&c->lock);
	ret = cs42l81_bringup(c);
	mutex_unlock(&c->lock);
	if (ret)
		dev_warn(&spi->dev,
			 "bring-up SPI err %d (codec unpowered/SPI0 — sysfs still up)\n",
			 ret);

	cs42l81_dev = c;

	ret = sysfs_create_groups(&spi->dev.kobj, cs42l81_groups);
	if (ret)
		dev_warn(&spi->dev, "sysfs groups failed: %d\n", ret);

	ret = devm_snd_soc_register_component(&spi->dev, &cs42l81_component,
					      &cs42l81_dai, 1);
	if (ret) {
		dev_err(&spi->dev, "snd_soc_register_component: %d\n", ret);
		sysfs_remove_groups(&spi->dev.kobj, cs42l81_groups);
		return ret;
	}

	c->input_handler.event = cs42l81_input_event;
	c->input_handler.connect = cs42l81_input_connect;
	c->input_handler.disconnect = cs42l81_input_disconnect;
	c->input_handler.name = "cs42l81-vol";
	c->input_handler.id_table = cs42l81_input_ids;
	ret = input_register_handler(&c->input_handler);
	if (ret)
		dev_warn(&spi->dev, "Vol± input handler: %d\n", ret);
	else
		c->input_handler_reg = true;

	dev_info(&spi->dev,
		 "CS42L81 SPI + ASoC DAI cs42l81-hifi (Vol±→Master step=%u)\n",
		 CS42L81_VOL_STEP);
	return 0;
}

static void cs42l81_remove(struct spi_device *spi)
{
	struct cs42l81 *c = spi_get_drvdata(spi);

	if (c) {
		if (c->input_handler_reg) {
			input_unregister_handler(&c->input_handler);
			c->input_handler_reg = false;
		}
		cancel_work_sync(&c->vol_work);
		cancel_delayed_work_sync(&c->asp_post_work);
		cs42_jack_poll_stop(c);
		c->component = NULL;
	}
	if (cs42l81_dev == c)
		cs42l81_dev = NULL;
	sysfs_remove_groups(&spi->dev.kobj, cs42l81_groups);
}

static const struct of_device_id cs42l81_of_match[] = {
	{ .compatible = "cirrus,cs42l81" },
	{ .compatible = "apple,338s1146" },
	{ }
};
MODULE_DEVICE_TABLE(of, cs42l81_of_match);

/* SPI core warns if OF compatibles have no matching id_table name. */
static const struct spi_device_id cs42l81_ids[] = {
	{ "cs42l81", 0 },
	{ "338s1146", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, cs42l81_ids);

static struct spi_driver cs42l81_driver = {
	.driver = {
		.name = "cs42l81-spi",
		.of_match_table = cs42l81_of_match,
	},
	.id_table = cs42l81_ids,
	.probe = cs42l81_probe,
	.remove = cs42l81_remove,
};
module_spi_driver(cs42l81_driver);

MODULE_DESCRIPTION("CS42L81 SPI codec + ASoC DAI (N31 RetailOS framing)");
MODULE_LICENSE("GPL");
