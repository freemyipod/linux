// SPDX-License-Identifier: GPL-2.0-only
/*
 * CS42L81 / Apple 338S1146 codec, SPI control port (iPod nano 7G / N31).
 *
 * The register sequences here are transcribed from the iPod nano 7G stock
 * firmware. docs-internal/n7g-audio/N31-AUDIO-STOCK-MAP.md maps every
 * function in this file to the stock routine it came from, documents the
 * register semantics, and records the evidence behind the choices that are
 * not self-evident from the code. Read it before changing any sequence
 * below: the ordering, the settling delays and the register values are all
 * load-bearing, and several of them are not what they look like.
 *
 * Where this driver deliberately departs from stock -- bounded polls, and
 * running the graph and path configuration from .prepare rather than the
 * transport trigger -- the reasons are in section 7 of that document.
 */
#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include <linux/apple-n31.h>

#include "n31-audio-rates.h"

/* Output gain, 0x0227. cs42_db_to_code() maps dB to the code it carries. */
#define CS42_DB_MIN		(-76)
#define CS42_DB_MAX		12
#define CS42_DB_KNEE		(-50)
#define CS42_VOL_MAX		(CS42_DB_MAX - CS42_DB_MIN)
#define CS42_VOL_DEFAULT	(CS42_VOL_MAX - 32)	/* -20 dB */

/* The 2.5 V backpower rail moves across this gain threshold. */
#define CS42_RAIL_DB		(-8)

/* Bounded replacement for stock's unbounded readiness poll. */
/*
 * Per-transfer tracing off by default.
 *
 * printk to a serial console is synchronous: the console is on ttySAC0 at
 * 115200, and one of these lines is on the order of ten milliseconds on the
 * wire. Emitting them from hw_params, trigger and the re-arm path put that
 * delay inside the window that re-arms audio DMA, which underran, which
 * logged, which delayed the next re-arm. The stream then start-stopped about
 * twice a second until printk's own rate limiter silenced it and playback
 * recovered on its own.
 *
 * So these are dev_dbg unless asked for: available through dyndbg, and
 * through cs42_vinfo=1 when a whole subsystem's trace is wanted at once. Probe,
 * removal and anything at warning or above are unaffected.
 */
static bool verbose;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose,
		 "Log per-transfer audio activity (default N)");

#define cs42_vinfo(dev, fmt, ...) \
	do { \
		if (verbose) \
			dev_info((dev), fmt, ##__VA_ARGS__); \
		else \
			dev_dbg((dev), fmt, ##__VA_ARGS__); \
	} while (0)

#define CS42_READY_POLLS	50

static bool trace_spi;
module_param(trace_spi, bool, 0644);
MODULE_PARM_DESC(trace_spi, "log every SPI frame sent to the codec");

static bool debug_regs;
module_param(debug_regs, bool, 0644);
MODULE_PARM_DESC(debug_regs, "dump codec state at each lifecycle transition");

/*
 * Mixer graph selection.
 *
 * Stock picks between a fixed graph image and a computed one on a runtime
 * configuration word that neither firmware image initialises, and which this
 * driver has no source for. Unset, that word selects the computed path, so
 * that is the faithful default. The fixed image stays reachable because it
 * is what stock emits when the word is set, and the two agree.
 *
 *	0    fixed image
 *	1    computed, route 1 (default)
 *	3, 4 computed, routes 3 and 4
 */
/*
 * Sample-rate-converter override for cs42_set_rate_long().
 *
 *	0  follow stock, which converts at 32 kHz and at no other rate
 *	1  force the converter on
 *	2  force it off
 *
 * A bench handle for A/B comparison, not a tuning knob. Leave it at 0.
 */
static int src_mode;
module_param(src_mode, int, 0644);
MODULE_PARM_DESC(src_mode,
	"0=stock condition (SRC at 32k only), 1=force codec SRC on, 2=force it off");

static int graph_mode = 1;
module_param(graph_mode, int, 0644);
MODULE_PARM_DESC(graph_mode,
		 "mixer graph: 1=computed route 1 (default), 0=fixed image, 3/4=computed");

/*
 * Codec master clock, in kHz.
 *
 * The codec clock divider and the whole rate-divider table are derived from
 * this. It has never been measured on the hardware: every divider stock uses
 * is 12000000/rate, which is consistent with 12 MHz but does not prove it.
 * It is a parameter rather than a constant so the assumption stays visible
 * and can be corrected in one place once it is measured.
 */
static unsigned int mclk_khz = 12000;
module_param(mclk_khz, uint, 0644);
MODULE_PARM_DESC(mclk_khz,
		 "codec master clock in kHz (12000 or 6000); UNMEASURED");

/*
 * Per-slot gain and tap values for the computed graph.
 *
 * Stock derives these from an initialised data block rather than hardcoding
 * them, and the derivation is reproduced here rather than the result being
 * assumed. It yields a slot gain of 480 and taps of 9 and 8, which are
 * exactly the values stock's fixed graph image writes -- the two paths agree
 * to the byte. The source block and the arithmetic are in the stock map.
 */
static const u32 cs42_graph_table[] = {
	0, 27, 37, 37, 40, 54, 74, 74, 80, 107, 147, 148, 160,
};

#define CS42_IDX_LO		2
#define CS42_IDX_HI		12

/* Stock uses a plain unsigned divide here. */
static u32 cs42_graph_accum(void)
{
	if (CS42_IDX_HI >= ARRAY_SIZE(cs42_graph_table))
		return 0;
	return (CS42_IDX_LO + 1) * cs42_graph_table[CS42_IDX_HI];
}

enum cs42_state {
	CS42_UNKNOWN = 0,
	CS42_PROBED,
	CS42_PREPARED,
	CS42_PLAYING,
	CS42_STANDBY,
};

static const char * const cs42_state_names[] = {
	"unknown", "probed", "prepared", "playing", "standby",
};

struct cs42l81 {
	struct spi_device *spi;
	struct mutex lock;
	struct snd_soc_component *component;
	struct delayed_work post_iis_work;

	enum cs42_state state;
	unsigned int rate;		/* rate the codec is configured for */
	unsigned int user_vol;		/* 0..CS42_VOL_MAX */
	bool dai_mute;
	bool key_done;			/* unlock key already sent this boot */
	bool graph_built;

	/*
	 * Shadow of the two per-path hold bits in 0x0220. State 3 sets it
	 * and cs42_path_mode() recomputes it; cs42_set_rate() dispatches on
	 * it, because it says which output path is live and therefore which
	 * one to program.
	 */
	u8 path_shadow;

	/* Cached left and right gain, in dB. */
	int gain_l_db;
	int gain_r_db;
};

struct cs42_reg {
	u16 reg;
	u8 val;
};

static struct cs42l81 *cs42l81_dev;

/* ------------------------------------------------------------------ */
/* Transport								*/
/* ------------------------------------------------------------------ */

/*
 * Two write frames. The narrow one sends five bytes and is used for
 * everything except a small set of registers; the wide one sends six and writes the
 * addressed register *and the one above it* -- the left/right pair of an
 * analog control. Dispatch is by register number so no call site can get it
 * wrong.
 */
static bool cs42_reg_is_pair(u16 reg)
{
	switch (reg) {
	case 0x0206:
	case 0x0225:
	case 0x0227:
	case 0x0229:
		return true;
	default:
		return false;
	}
}

static int cs42_xfer(struct cs42l81 *c, const u8 *tx, u8 *rx, size_t len)
{
	struct spi_transfer t = { .tx_buf = tx, .rx_buf = rx, .len = len };
	struct spi_message m;
	int ret;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(c->spi, &m);
	if (trace_spi)
		dev_info(&c->spi->dev, "spi %*ph%s\n", (int)len, tx,
			 ret ? " ERR" : "");
	return ret;
}

/* Narrow write: one register. */
static int cs42_wr8(struct cs42l81 *c, u16 reg, u8 val)
{
	u8 tx[5] = { 0x6c, reg >> 8, reg & 0xff, 0x00, val };

	return cs42_xfer(c, tx, NULL, sizeof(tx));
}

/* Wide write: reg and reg+1. */
static int cs42_wr16(struct cs42l81 *c, u16 reg, u8 val)
{
	u8 tx[6] = { 0x6c, reg >> 8, (reg & 0xff) | 0x80, 0x01, val, val };

	return cs42_xfer(c, tx, NULL, sizeof(tx));
}

/* Register read. */
static int cs42_rd(struct cs42l81 *c, u16 reg, u8 *val)
{
	u8 tx[5] = { 0x6d, reg >> 8, reg & 0xff, 0x00, 0xff };
	u8 rx[5] = { 0 };
	int ret = cs42_xfer(c, tx, rx, sizeof(tx));

	if (ret)
		return ret;
	*val = rx[4];
	return 0;
}

/*
 * Masked read-modify-write.
 *
 * The mask narrows the value, not the transaction: a zero mask still
 * performs a read and a write, and the path mode sequence relies on that.
 * Do not optimise it away.
 */
static int cs42_rmw(struct cs42l81 *c, u16 reg, u8 mask, u8 val)
{
	u8 cur = 0;
	int ret = cs42_rd(c, reg, &cur);

	if (ret)
		return ret;
	return cs42_wr8(c, reg, (cur & ~mask) | (val & mask));
}

static int cs42_write_table(struct cs42l81 *c, const struct cs42_reg *t,
			    unsigned int n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = cs42_wr8(c, t[i].reg, t[i].val);
		if (ret)
			return ret;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Message mailbox							*/
/* ------------------------------------------------------------------ */

/*
 * Accessory and headphone-remote message channel.
 *
 * 0x051E..0x0525 is a byte-oriented message FIFO that shares the 0x05xx page
 * with the audio graph registers but has nothing to do with audio:
 *
 *	0x051E	write-side control; bit 0 latches the level pair, bit 5
 *		strobes a reset
 *	0x051F	write-side status, bit 1 = full
 *	0x0520	free space, less one
 *	0x0521	write data port
 *	0x0523	read-side control, bit 5 strobes a reset
 *	0x0524	read-side status
 *	0x0525	read data port
 *
 * Frames carry a CRC-16 over the first len+5 bytes. Stock computes it from a
 * ROM table that is bit-exact CRC-16/ARC, so the kernel's crc16() is a drop-in
 * and nothing here is invented.
 *
 * No part of the audio path sends a message. This is implemented and exported
 * so the MikeyBus side has a transport, and so that the audio path's absence
 * of mailbox traffic reads as a deliberate match with stock rather than a gap.
 */
#define CS42_MBOX_LEN_OFF	1	/* payload length lives in byte 1 */
#define CS42_MBOX_OVERHEAD	7	/* header + the two CRC bytes */
#define CS42_MBOX_MAX		64	/* bound on the prepare-time FIFO drain */

/* Append the CRC-16 and return the full frame length. */
static size_t cs42_mbox_frame(u8 *buf, size_t buf_size)
{
	size_t len = buf[CS42_MBOX_LEN_OFF];
	u16 crc;

	if (len + CS42_MBOX_OVERHEAD > buf_size)
		return 0;
	crc = crc16(0, buf, len + 5);
	buf[len + 5] = crc >> 8;
	buf[len + 6] = crc & 0xff;
	return len + CS42_MBOX_OVERHEAD;
}

/*
 * Check the free space, then push the frame a byte at a time. Stock reports
 * a distinct error when the frame does not fit; -ENOSPC is the equivalent.
 */
static int cs42_mbox_send_locked(struct cs42l81 *c, const u8 *frame, size_t len)
{
	u8 status = 0, space = 0;
	size_t i;
	int ret;

	ret = cs42_rmw(c, 0x051e, 0x01, 0x01);
	if (ret)
		return ret;
	cs42_rd(c, 0x051f, &status);
	cs42_rd(c, 0x0520, &space);
	ret = cs42_rmw(c, 0x051e, 0x01, 0x00);
	if (ret)
		return ret;

	if (len >= ((status & 0x02) ? 0 : (unsigned int)space + 1))
		return -ENOSPC;

	for (i = 0; i < len; i++) {
		ret = cs42_wr8(c, 0x0521, frame[i]);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * Frame and send one message. @buf must hold the header, the payload and
 * CS42_MBOX_OVERHEAD bytes of room; byte 1 is the payload length, and the
 * CRC is appended here.
 */
int cs42l81_mbox_send(u8 *buf, size_t buf_size)
{
	struct cs42l81 *c = cs42l81_dev;
	size_t len;
	int ret;

	if (!c)
		return -ENODEV;
	len = cs42_mbox_frame(buf, buf_size);
	if (!len)
		return -EINVAL;

	mutex_lock(&c->lock);
	ret = cs42_mbox_send_locked(c, buf, len);
	mutex_unlock(&c->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cs42l81_mbox_send);

/* Drain whatever the read side has queued. Returns the byte count. */
int cs42l81_mbox_recv(u8 *buf, size_t buf_size)
{
	struct cs42l81 *c = cs42l81_dev;
	u8 level = 0;
	size_t i;

	if (!c)
		return -ENODEV;

	mutex_lock(&c->lock);
	cs42_rd(c, 0x0524, &level);
	if (level > buf_size)
		level = buf_size;
	for (i = 0; i < level; i++) {
		if (cs42_rd(c, 0x0525, &buf[i]))
			break;
	}
	mutex_unlock(&c->lock);
	return i;
}
EXPORT_SYMBOL_GPL(cs42l81_mbox_recv);

/* ------------------------------------------------------------------ */
/* SoC clock (owned by the IIS driver)					*/
/* ------------------------------------------------------------------ */

static void cs42_soc_clk_gate(struct cs42l81 *c, bool on)
{
	void (*gate)(bool);

	gate = (void (*)(bool))__symbol_get("s5l8740_codec_clk_gate");
	if (!gate) {
		dev_warn_once(&c->spi->dev, "codec clock gate unavailable\n");
		return;
	}
	gate(on);
	__symbol_put("s5l8740_codec_clk_gate");
}

/*
 * Codec clock divider, the first thing state 3 does -- before the clock gate
 * is opened. It follows the master clock: 2 at 12 MHz, otherwise 4. Every
 * divider in the rate table is 12000000/rate, so 12 MHz is the case this
 * board runs and 2 is the value stock selects.
 */
static void cs42_soc_clk_divider(struct cs42l81 *c)
{
	void (*divider)(unsigned int);

	divider = (void (*)(unsigned int))
		__symbol_get("s5l8740_codec_clk_divider");
	if (!divider) {
		dev_warn_once(&c->spi->dev,
			      "codec clock divider unavailable\n");
		return;
	}
	divider(mclk_khz == 12000 ? 2 : 4);
	__symbol_put("s5l8740_codec_clk_divider");
}

/* ------------------------------------------------------------------ */
/* Gain									*/
/* ------------------------------------------------------------------ */

/* dB to register code: 1 dB per step down to -50, 2 dB below that. */
static int cs42_db_to_code(int db)
{
	if (db > CS42_DB_MAX)
		db = CS42_DB_MAX;
	if (db < CS42_DB_MIN)
		db = CS42_DB_MIN;
	if (db > CS42_DB_KNEE)
		return db;
	if (db & 1)
		db--;
	return CS42_DB_KNEE + (db - CS42_DB_KNEE) / 2;
}

/* Register code back to dB. Code -64 is the mute entry. */
static int cs42_code_to_db(u8 raw)
{
	int code = (s8)((raw & 0x40) ? (raw | 0x80) : (raw & 0x7f));

	if (code == -64)
		return -90;
	if (code < CS42_DB_KNEE)
		return 2 * code + 50;
	return code;
}

/*
 * Write the DAC gain, and move the 2.5 V backpower rail across the -8 dB
 * threshold.
 *
 * The rail is the class-H supply for the headphone amplifier. Stock raises it
 * when the gain comes up through -8 dB and, going the other way, arms a
 * 601 ms timer and drops it when that expires. The delayed drop is not
 * implemented: leaving the rail up is the safe direction, and the timer only
 * saves idle current.
 */
static int cs42_dac_gain(struct cs42l81 *c, int code)
{
	int new_db = cs42_code_to_db(code & 0x7f);
	int old_db;
	u8 raw = 0;
	int ret;

	ret = cs42_rd(c, 0x0227, &raw);
	old_db = ret ? -90 : cs42_code_to_db(raw);

	if (old_db < CS42_RAIL_DB && new_db >= CS42_RAIL_DB) {
		u8 rail = 0, comp = 0;

		if (!cs42_rd(c, 0xc96f, &rail) &&
		    !cs42_rd(c, 0x0219, &comp) &&
		    (rail != 0x1e || (comp & 0x07) != 0x01)) {
			cs42_wr8(c, 0xc96f, 0x0e);
			cs42_rmw(c, 0x0219, 0x07, 0x01);
			/*
			 * 155 us, not milliseconds. Stock reaches this
			 * through a delay helper whose argument is scaled
			 * by 1000 relative to the one used elsewhere in
			 * this file; the underlying unit is microseconds.
			 */
			usleep_range(155, 200);
			cs42_wr8(c, 0xc96f, 0x1e);
		}
	} else if (old_db >= CS42_RAIL_DB && new_db < CS42_RAIL_DB) {
		u8 rail = 0, comp = 0;

		/*
		 * The other half of the pair: drop the 2.5 V backpower and
		 * clear the compensation bits.
		 *
		 * Stock gates this on a shadow it sets when the rail was
		 * raised. This driver keeps no such shadow, so it reads the
		 * two registers and acts only when they are in the raised
		 * state -- the same test the raise above uses, inverted.
		 *
		 * Without it the rail stays at 0x1E and 0x0219[2:0] at 1 for
		 * the life of the part once any gain has crossed the
		 * threshold, because nothing else writes them down.
		 */
		if (!cs42_rd(c, 0xc96f, &rail) &&
		    !cs42_rd(c, 0x0219, &comp) &&
		    (rail == 0x1e || (comp & 0x07) == 0x01)) {
			cs42_wr8(c, 0xc96f, 0x0e);
			cs42_rmw(c, 0x0219, 0x07, 0x00);
		}
	}

	return cs42_wr16(c, 0x0227, code & 0x7f);
}

/*
 * Per-channel gain, for the two selectors this driver uses. Selector 0 is the
 * DAC pair at 0x0227 and always routes through cs42_dac_gain(); selector 1 is
 * the pair at 0x0229 and is a plain wide write.
 */
static int cs42_channel_gain(struct cs42l81 *c, int sel, int db)
{
	if (sel == 0)
		return cs42_dac_gain(c, cs42_db_to_code(db));
	return cs42_wr16(c, 0x0229, cs42_db_to_code(db) & 0x7f);
}

/*
 * Gain re-apply, right channel first and then left. The playback engine runs
 * this before it submits its first buffer, which is why 0x0229 carries a real
 * gain rather than the constant power-up leaves there.
 */
static int cs42_apply_gains(struct cs42l81 *c)
{
	int ret = cs42_channel_gain(c, 1, c->gain_r_db);

	if (ret)
		return ret;
	return cs42_channel_gain(c, 0, c->gain_l_db);
}

static int cs42_apply_user_vol(struct cs42l81 *c)
{
	c->gain_l_db = (int)c->user_vol + CS42_DB_MIN;
	c->gain_r_db = c->gain_l_db;
	return cs42_apply_gains(c);
}

/* ------------------------------------------------------------------ */
/* Power states								*/
/* ------------------------------------------------------------------ */

/* The 0x9901 unlock key, once per boot. */
static void cs42_user_key(struct cs42l81 *c)
{
	if (c->key_done)
		return;
	cs42_wr8(c, 0x9901, 0xa5);
	cs42_wr8(c, 0x9901, 0x00);
	c->key_done = true;
}

/*
 * Park. Configures the analog block, waits for it to report ready, raises the
 * freeze latch and drops the codec clock. cs42_state_run() is the matching
 * half; stock never runs one without the other.
 */
static int cs42_state_park(struct cs42l81 *c)
{
	unsigned int i;
	u8 v2f = 0;
	int ret;

	/*
	 * The codec clock has to be running before any of this is written.
	 *
	 * Park does not enable the clock -- it ends by dropping it, and
	 * assumes it was on at entry. Stock always satisfies that, because
	 * every entry to park follows either the bootloader's analog
	 * power-up or state 3, both of which leave the clock on.
	 *
	 * Nothing in this driver's own flow satisfies it, since cs42_prepare()
	 * parks first. Writes into a gated bus complete normally at the SPI
	 * layer and land nowhere, which leaves the analog output block
	 * unpowered while the codec still answers reads and every register
	 * state 3 touches looks correct. Establish the precondition here
	 * rather than in the caller, so it holds for every entry to park.
	 */
	cs42_soc_clk_divider(c);
	cs42_soc_clk_gate(c, true);

	/*
	 * The 0x9901 key has to be in before the 0x02xx writes below, or the
	 * codec discards them.
	 *
	 * The key is applied once per boot. Stock issues it from state 3,
	 * which in its flow has always run long before any park. This driver
	 * enters park first, so without the call here every 0x02xx write in
	 * this function is rejected -- silently, because the SPI transfer
	 * itself completes normally, and page 0x00 writes in the same
	 * sequence still land.
	 *
	 * cs42_user_key() is idempotent, so state 3's call stays where stock
	 * has it and becomes a no-op after this.
	 */
	cs42_user_key(c);

	ret = cs42_wr16(c, 0x0227, 0x40);
	if (ret)
		return ret;
	ret = cs42_wr16(c, 0x0225, 0x33);
	if (ret)
		return ret;
	ret = cs42_wr16(c, 0x0229, 0x40);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0075, 0x80, 0x00);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0220, 0x78, 0x78);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0006, 0x01, 0x01);
	if (ret)
		return ret;

	/*
	 * Bounded, unlike stock.
	 *
	 * Stock spins on this bit with no counter, no deadline and no error
	 * arm. That is affordable for an RTOS that owns the machine and not
	 * for a kernel, where it would wedge whichever thread ALSA called us
	 * on. The 1 ms per iteration follows stock's delay; the iteration
	 * count is ours and has no stock basis.
	 *
	 * On timeout, fail rather than continue. The writes below are ones
	 * stock cannot reach without this bit set, because it is still in the
	 * loop; issuing them anyway puts the codec in a state stock never
	 * produces.
	 */
	for (i = 0; i < CS42_READY_POLLS; i++) {
		usleep_range(1000, 1500);
		ret = cs42_rd(c, 0x002f, &v2f);
		if (ret) {
			dev_err(&c->spi->dev,
				"0x2F read failed (%d) while waiting for analog ready\n",
				ret);
			return ret;
		}
		if (v2f & 0x80)
			break;
	}
	if (!(v2f & 0x80)) {
		dev_err(&c->spi->dev,
			"analog block not ready after %u polls (0x2F=0x%02x)\n",
			i, v2f);
		return -ETIMEDOUT;
	}

	ret = cs42_rmw(c, 0x0006, 0x40, 0x40);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0007, 0x40, 0x40);
	if (ret)
		return ret;

	cs42_soc_clk_gate(c, false);
	return 0;
}

/*
 * Run. Selects the codec clock divider, turns the clock back on, sends the
 * key once, releases the freeze latch and reads the two trim bytes stock
 * caches for later use.
 */
static int cs42_state_run(struct cs42l81 *c)
{
	u8 r74 = 0, trim0 = 0, trim1 = 0;
	int ret;

	cs42_soc_clk_divider(c);
	cs42_soc_clk_gate(c, true);
	cs42_user_key(c);

	ret = cs42_rmw(c, 0x0007, 0x40, 0x00);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0006, 0x40, 0x00);
	if (ret)
		return ret;
	c->path_shadow = 0x28;
	ret = cs42_rmw(c, 0x0220, 0x28, 0x28);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x000f, 0x80, 0x80);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0075, 0x40, 0x40);
	if (ret)
		return ret;

	ret = cs42_rd(c, 0x0074, &r74);
	if (ret)
		return ret;
	cs42_wr8(c, 0x0074, (r74 & 0xe7) | 0x08);
	cs42_rd(c, 0x007b, &trim0);
	cs42_rd(c, 0x007c, &trim1);
	cs42_wr8(c, 0x0074, r74);

	ret = cs42_rmw(c, 0x0075, 0x40, 0x00);
	if (ret)
		return ret;
	return cs42_rmw(c, 0x0075, 0x80, 0x80);
}

/* ------------------------------------------------------------------ */
/* Path mode and output path						*/
/* ------------------------------------------------------------------ */

/*
 * Path mode, for the three modes this driver uses. The sequence is fixed and
 * only the values and two conditional blocks differ, so the shape is written
 * once and the per-mode values come from a table.
 *
 * 0x0204 and 0x0203 are issued even when their mask is zero, because stock
 * issues them and a masked write is still a read and a write.
 */
struct cs42_path_mode {
	u8 r06;			/* 0x0006 mask 0x04 */
	u8 r220;		/* 0x0220 mask 0x28 */
	bool has_r0d;		/* 0x000D written at all */
	u8 r0d;
	u8 r206;		/* 0x0206 and 0x0207, mask 0x3F */
	bool has_r205;		/* 0x0205 written at all */
	u8 r205;
	u8 r204_mask;		/* 0x0204 -- issued even when this is zero */
	u8 r204;
	u8 r203;		/* 0x0203 mask 0xC0 */
	bool has_eq;		/* 0x000E / 0x011F / 0x0120 / 0x012E block */
	unsigned int settle_ms;
	u8 path_shadow;		/* 0x0220 hold bits this mode leaves behind */
};

/* Mode 271, installed by the output path enable. */
static const struct cs42_path_mode cs42_mode_271 = {
	.r06 = 0x00, .r220 = 0x00,
	.has_r0d = false,
	.r206 = 0x3d,
	.has_r205 = true, .r205 = 0x5a,
	.r204_mask = 0x00, .r204 = 0x00,
	.r203 = 0x00,
	.has_eq = true,
	.settle_ms = 105,
	.path_shadow = 0x00,
};

/* Mode 18, applied by the playback engine before its first buffer. */
static const struct cs42_path_mode cs42_mode_18 = {
	.r06 = 0x04, .r220 = 0x08,
	.has_r0d = true, .r0d = 0x00,
	.r206 = 0x34,
	.has_r205 = false,
	.r204_mask = 0x03, .r204 = 0x00,
	.r203 = 0xc0,
	.has_eq = false,
	.settle_ms = 60,
	.path_shadow = 0x08,
};

/* Mode 6, the companion of the output path disable. */
static const struct cs42_path_mode cs42_mode_6 = {
	.r06 = 0x04, .r220 = 0x00,
	.has_r0d = true, .r0d = 0x00,
	.r206 = 0x34,
	.has_r205 = false,
	.r204_mask = 0x03, .r204 = 0x00,
	.r203 = 0xc0,
	.has_eq = false,
	.settle_ms = 60,
	.path_shadow = 0x00,
};

static int cs42_set_rate(struct cs42l81 *c, unsigned int rate);
static int cs42_src_bypass(struct cs42l81 *c);
static int cs42_hp_mute(struct cs42l81 *c, bool mute);

static int cs42_path_mode(struct cs42l81 *c, const struct cs42_path_mode *m)
{
	int ret;

	ret = cs42_rmw(c, 0x0006, 0x04, m->r06);
	if (ret)
		return ret;
	/*
	 * Every mode but the bring-up one leaves the headphone hold clear.
	 * Keep it raised when the user has muted, so applying a mode -- which
	 * happens on every stream start -- does not undo the control.
	 */
	ret = cs42_rmw(c, 0x0220, 0x28,
		       c->dai_mute ? (u8)(m->r220 | 0x20) : m->r220);
	if (ret)
		return ret;
	c->path_shadow = m->path_shadow;
	if (m->has_r0d) {
		ret = cs42_rmw(c, 0x000d, 0x03, m->r0d);
		if (ret)
			return ret;
	}
	ret = cs42_rmw(c, 0x0206, 0x3f, m->r206);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0207, 0x3f, m->r206);
	if (ret)
		return ret;
	if (m->has_r205) {
		ret = cs42_rmw(c, 0x0205, 0xff, m->r205);
		if (ret)
			return ret;
	}
	ret = cs42_rmw(c, 0x0204, m->r204_mask, m->r204);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0206, 0xc0, 0x00);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0207, 0xc0, 0x00);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0203, 0xc0, m->r203);
	if (ret)
		return ret;

	if (m->has_eq) {
		ret = cs42_rmw(c, 0x000e, 0x40, 0x40);
		if (ret)
			return ret;
		cs42_rmw(c, 0x011f, 0x3f, 0x1c);
		cs42_rmw(c, 0x0120, 0x3f, 0x1c);
		cs42_rmw(c, 0x012e, 0xff, 0xaa);
		ret = cs42_rmw(c, 0x000e, 0x40, 0x00);
		if (ret)
			return ret;
	}

	msleep(m->settle_ms);

	/*
	 * Re-apply the rate against the shadow this call just installed.
	 *
	 * The rate program picks which output path to configure from the
	 * shadow, so a mode change that moves the shadow invalidates it.
	 * Stock re-applies from the output-enable op, and only when the
	 * shadow selects a path -- not from the path mode itself.
	 *
	 * Gating on the shadow reproduces that. Re-applying unconditionally
	 * would additionally run the 0x010x sink's program from mode 271's
	 * shadow of zero, which writes 0x010B and 0x010C; a live stock dump
	 * shows both still holding their reset values.
	 */
	if (c->rate && (m->path_shadow & 0x08))
		return cs42_set_rate(c, c->rate);
	return 0;
}

/* Output path enable. */
static int cs42_output_path_on(struct cs42l81 *c)
{
	int ret = cs42_rmw(c, 0x0206, 0x3f, 0x08);

	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0207, 0x3f, 0x08);
	if (ret)
		return ret;
	return cs42_path_mode(c, &cs42_mode_271);
}

/* Output path disable: the mode runs first, then the path is taken down. */
static int cs42_output_path_off(struct cs42l81 *c)
{
	int ret;

	/*
	 * Stock's output-path disable puts the converter back in bypass
	 * before it takes the rest of the path down, which is why a live
	 * stock dump reads 0x0131 bit 0 set between streams.
	 */
	ret = cs42_src_bypass(c);
	if (ret)
		return ret;
	ret = cs42_path_mode(c, &cs42_mode_6);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0206, 0x3f, 0x00);
	if (ret)
		return ret;
	return cs42_rmw(c, 0x0207, 0x3f, 0x00);
}

/*
 * Analog bring-up: power the output stage.
 *
 * This is a climb, not a power-down. Stock's state applier stages this state
 * through the run state when the current state is below it, which is not
 * something anything does to reach a power-down, and the sequence itself
 * powers the analog block, raises the 2.5 V backpower and applies gain.
 *
 * A live dump of this driver before the state was reached read 0x0219 as
 * zero, so the analog stage had never come up, which also explains the
 * absence of any discharge plop on reboot.
 */
static int cs42_state_analog_on(struct cs42l81 *c)
{
	int ret;

	ret = cs42_rmw(c, 0x0007, 0x40, 0x00);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0219, 0x78, 0x78);
	if (ret)
		return ret;
	ret = cs42_wr16(c, 0x0229, 0x40);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0006, 0x01, 0x00);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0201, 0xe0, 0x40);
	if (ret)
		return ret;
	cs42_wr8(c, 0xc81f, 0xff);
	cs42_wr8(c, 0xc85f, 0x0f);
	cs42_wr8(c, 0xc96f, 0x0e);
	cs42_wr8(c, 0x0223, 0x08);
	cs42_wr8(c, 0x0224, 0x09);
	cs42_wr16(c, 0x0225, 0x00);

	cs42_dac_gain(c, 0x40);
	cs42_dac_gain(c, 0x41);
	c->gain_l_db = cs42_code_to_db(0x41);
	cs42_wr16(c, 0x0229, 0x41);
	c->gain_r_db = cs42_code_to_db(0x41);

	ret = cs42_rmw(c, 0x000e, 0xc0, 0x40);
	if (!ret)
		c->state = CS42_STANDBY;
	return ret;
}

/* ------------------------------------------------------------------ */
/* Sample rate								*/
/* ------------------------------------------------------------------ */

/*
 * Put the converter back in bypass.
 *
 * This is the whole of stock's rate program when its second argument is
 * zero: the argument gates the function at its first instruction, and a zero
 * skips everything and issues this single write. The output-path disable
 * calls it that way.
 */
static int cs42_src_bypass(struct cs42l81 *c)
{
	return cs42_rmw(c, 0x0131, 0x01, 0x01);
}

/*
 * Rate program for the 0x010x sink. Ends muted, with the 0x0220 bit-5 hold
 * raised.
 *
 * Stock guards the body with an accessory-route test that has no counterpart
 * here -- see section 3 of the stock map. The gate can only suppress writes,
 * so this programs unconditionally rather than substituting a test of our
 * own.
 *
 * The converter test below is against rate code 12 and is written directly
 * into stock's instruction stream. It is not the test cs42_set_rate_long()
 * uses; the two functions genuinely differ. 0x010B and 0x010C belong to a
 * different sink than the 0x022x headphone path, and this function never
 * touches 0x0222, 0x0223 or 0x0224.
 */
static int cs42_set_rate_183138(struct cs42l81 *c, u8 code)
{
	int ret;

	ret = cs42_rmw(c, 0x000e, 0xc0, 0xc0);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x000f, 0x0f, code);
	if (ret)
		return ret;
	ret = cs42_wr8(c, 0x012f, code | (code << 4));
	if (ret)
		return ret;

	if (code == 12) {
		cs42_wr8(c, 0x010b, 0x08);
		cs42_wr8(c, 0x010c, 0x09);
		cs42_rmw(c, 0x0131, 0x01, 0x01);
	} else {
		cs42_wr8(c, 0x0121, 0x08);
		cs42_wr8(c, 0x0122, 0x09);
		cs42_rmw(c, 0x0130, 0x0f, code);
		cs42_rmw(c, 0x0131, 0x01, 0x00);
		cs42_wr8(c, 0x010b, 0x04);
		cs42_wr8(c, 0x010c, 0x33);
	}

	ret = cs42_rmw(c, 0x000e, 0xc0, 0x40);
	if (ret)
		return ret;
	ret = cs42_dac_gain(c, 0x40);
	if (ret)
		return ret;
	return cs42_rmw(c, 0x0220, 0x20, 0x20);
}

/*
 * Rate program when both paths are held: the rate alone, with no routing.
 */
static int cs42_set_rate_short(struct cs42l81 *c, u8 code)
{
	int ret = cs42_rmw(c, 0x000e, 0xc0, 0xc0);

	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x000f, 0x0f, code);
	if (ret)
		return ret;
	ret = cs42_wr8(c, 0x012f, code | (code << 4));
	if (ret)
		return ret;

	return cs42_rmw(c, 0x000e, 0xc0, 0x40);
}

/*
 * Rate program for the 0x022x headphone path, taken when the shadow carries
 * 0x08 but not 0x20. Mutes, raises the hold, programs the rate and the path
 * routing inside the 0x000E configuration guard, drops the hold and restores
 * the gain.
 *
 * This is the only function that writes 0x0222, 0x0223 and 0x0224, so it is
 * the only one that configures the headphone path at all.
 *
 * On a retail device stock's converter condition reduces to rate == 32000,
 * and a live stock dump at 44.1 kHz confirms it: 0x0222 = 0x0a (the DAC
 * running natively at 44.1), 0x0223/0x0224 = 0x08/0x09 (the direct feed) and
 * 0x0131 = 0x01 (converter bypassed).
 *
 * The reason stock bypasses at 44.1 is that 12 MHz has no integer divider
 * for that family. The IIS divider is 12e6/rate truncated, so the serialiser
 * really clocks 44117.65 Hz at "44100", 22058.82 at "22050" and 11029.41 at
 * "11025" -- each 400 ppm fast. Through the converter, which is told its
 * input arrives at the tabled rate, that mismatch slips a sample about
 * eighteen times a second and is plainly audible. Clocking the DAC straight
 * off LRCK leaves the same 400 ppm as a pitch offset of a fifteenth of a
 * semitone. 32 kHz, the one rate stock does convert, divides exactly.
 *
 * Full derivation and the dump are in section 3 of the stock map.
 */
static int cs42_set_rate_long(struct cs42l81 *c, u8 code)
{
	/* Stock's condition on a retail device: rate == 32000, code 9. */
	bool src = code == 9;
	int ret;

	if (src_mode == 1)
		src = true;
	else if (src_mode == 2)
		src = false;

	ret = cs42_dac_gain(c, 0x40);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0220, 0x20, 0x20);
	if (ret)
		return ret;
	msleep(50);

	ret = cs42_rmw(c, 0x000e, 0xc0, 0xc0);
	if (ret)
		return ret;
	cs42_rmw(c, 0x000f, 0x0f, code);
	cs42_wr8(c, 0x012f, code | (code << 4));

	if (src) {
		cs42_wr8(c, 0x0121, 0x08);
		cs42_wr8(c, 0x0122, 0x09);
		cs42_rmw(c, 0x0130, 0x0f, code);
		cs42_rmw(c, 0x0131, 0x01, 0x00);
		cs42_wr8(c, 0x0223, 0x04);
		cs42_wr8(c, 0x0224, 0x33);
	} else {
		cs42_rmw(c, 0x0131, 0x01, 0x01);
		cs42_wr8(c, 0x0223, 0x08);
		cs42_wr8(c, 0x0224, 0x09);
	}
	cs42_wr8(c, 0x0222, src ? 12 : code);

	ret = cs42_rmw(c, 0x000e, 0xc0, 0x40);
	if (ret)
		return ret;
	ret = cs42_hp_mute(c, c->dai_mute);
	if (ret)
		return ret;
	msleep(60);

	/*
	 * Restore the gain: the volume is summed into the gain accumulator,
	 * mapped to a code and written. One call, and deliberately not
	 * cs42_apply_gains() -- stock does not touch 0x0229 here, because
	 * the right-channel write belongs to the separate gain re-apply the
	 * playback engine runs.
	 */
	return cs42_dac_gain(c, cs42_db_to_code(c->gain_l_db));
}

/* Rate dispatch: which path is live decides which program to run. */
static int cs42_set_rate(struct cs42l81 *c, unsigned int rate)
{
	const struct n31_rate_cfg *r = n31_find_rate(rate);
	int ret;

	if (!r)
		return -EINVAL;

	if (!(c->path_shadow & 0x08))
		ret = cs42_set_rate_183138(c, r->cs42_rate_code);
	else if (c->path_shadow & 0x20)
		ret = cs42_set_rate_short(c, r->cs42_rate_code);
	else
		ret = cs42_set_rate_long(c, r->cs42_rate_code);

	if (!ret)
		c->rate = rate;
	return ret;
}

/* ------------------------------------------------------------------ */
/* Mixer graph								*/
/* ------------------------------------------------------------------ */

/* The fixed graph image, verbatim. */
static const struct cs42_reg cs42_graph_static[] = {
	{ 0x0006, 0x24 },
	{ 0x0529, 0x2c }, { 0x052a, 0x2c }, { 0x0533, 0x2c }, { 0x0534, 0x2c },

	{ 0x0400, 0x04 }, { 0x0401, 0x10 }, { 0x0402, 0x00 },
	{ 0x0403, 0x09 }, { 0x0404, 0x08 }, { 0x0405, 0x00 }, { 0x0406, 0x00 },

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

	{ 0x0400, 0x04 }, { 0x0401, 0x12 },
};

static int cs42_play_graph_static(struct cs42l81 *c)
{
	u8 status = 0;
	int ret;

	ret = cs42_write_table(c, cs42_graph_static,
			       ARRAY_SIZE(cs42_graph_static));
	if (ret)
		return ret;
	msleep(100);
	/*
	 * No 0x0500 write here.
	 *
	 * The 0x0400/0x0500 space on this part is the DSP and write-sequencer
	 * subsystem, not the serial audio port, and stock writes 0x0500 once
	 * in the whole image from an override branch that is not on the audio
	 * path. Nothing here should be setting a sample rate or bit depth.
	 */
	cs42_rd(c, 0x0528, &status);
	if (debug_regs)
		dev_info(&c->spi->dev, "graph: 0x528=0x%02x\n", status);
	return 0;
}

/*
 * Slot map: eleven slots of three registers each. The source index runs from
 * 0 on the left and 10 on the right; the terminator marks the slot that
 * carries the tail value.
 */
static int cs42_slot_map(struct cs42l81 *c, int side, u8 count, u16 base_val,
			 u16 gain)
{
	u16 base_reg = side ? 0x0428 : 0x0407;
	u8 source = side ? 10 : 0;
	u8 term = side ? 0xff : 0xfe;
	int i, ret;

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

		ret = cs42_wr8(c, reg, a);
		if (ret)
			return ret;
		ret = cs42_wr8(c, reg + 1, b);
		if (ret)
			return ret;
		ret = cs42_wr8(c, reg + 2, cc);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * Computed mixer graph.
 *
 * Per-side counts and bases by route:
 *
 *	route 1	count_l 0, count_r 0, base_l 0,   base_r 0
 *	route 3	count_l 0, count_r 0, base_l 160, base_r 160
 *	route 4	count_l 2, count_r 0, base_l 160, base_r 160
 *
 * The tap indices follow. The accumulators are only refreshed when the
 * matching count is non-zero, and the count multiplies the accumulator in the
 * tap term, so a zero count drops the gain out:
 *
 *	if (count_l) accum_l = (idx_l_lo + 1) * table[idx_l_hi]
 *	tap_l = (base_l + accum_l * count_l + 159) / 160 + 2
 *	if (count_r) accum_r = (idx_r_lo + 1) * table[idx_r_hi]
 *	tap_r = (base_r + accum_r * count_r + 159) / 160 + 1
 *
 * On route 4 this yields tap_r = 2 where the fixed graph image writes 8.
 * That is not a defect in either transcription: 8 requires count_r = 2,
 * which stock never installs here, so the fixed image was baked from a state
 * this path does not produce. The left side matches exactly.
 */
static int cs42_play_graph_dynamic(struct cs42l81 *c, int mode)
{
	u8 count_l = (mode == 4) ? 2 : 0;
	u8 count_r = 0;
	u16 base = (mode == 1) ? 0 : 160;
	u32 accum_l = count_l ? cs42_graph_accum() : 0;
	u32 accum_r = count_r ? cs42_graph_accum() : 0;
	u8 tap_l = (base + accum_l * count_l + 159) / 160 + 2;
	u8 tap_r = (base + accum_r * count_r + 159) / 160 + 1;
	u8 status = 0;
	int i, ret;

	/* The packed index nibbles, per active slot. */
	for (i = 0; i < count_l; i++) {
		ret = cs42_rmw(c, 0x0529 + i, 0x3f,
			       CS42_IDX_HI | (CS42_IDX_LO << 4));
		if (ret)
			return ret;
	}
	for (i = 0; i < count_r; i++) {
		ret = cs42_rmw(c, 0x0533 + i, 0x3f,
			       CS42_IDX_HI | (CS42_IDX_LO << 4));
		if (ret)
			return ret;
	}

	ret = cs42_rmw(c, 0x054f, 0xf0, 0x00);
	if (ret)
		return ret;
	cs42_rmw(c, 0x0401, 0x01, 0x00);
	cs42_rmw(c, 0x0401, 0x02, 0x00);
	cs42_wr8(c, 0x0402, 0x00);
	cs42_wr8(c, 0x0403, tap_l);
	cs42_wr8(c, 0x0404, tap_r);
	cs42_wr8(c, 0x0405, 0x00);
	cs42_wr8(c, 0x0406, 0x00);

	ret = cs42_slot_map(c, 0, count_l, base, accum_l);
	if (ret)
		return ret;
	ret = cs42_slot_map(c, 1, count_r, base, accum_r);
	if (ret)
		return ret;

	ret = cs42_rmw(c, 0x0401, 0x02, 0x02);
	if (ret)
		return ret;
	msleep(100);
	cs42_rd(c, 0x0528, &status);
	if (debug_regs)
		cs42_vinfo(&c->spi->dev,
			 "graph: mode=%d taps=%u/%u 0x528=0x%02x\n",
			 mode, tap_l, tap_r, status);
	return 0;
}

static int cs42_play_graph(struct cs42l81 *c)
{
	if (graph_mode == 0)
		return cs42_play_graph_static(c);
	return cs42_play_graph_dynamic(c, graph_mode);
}

/* ------------------------------------------------------------------ */
/* Transport								*/
/* ------------------------------------------------------------------ */

/* Analog mute. Stock drives this from the transport, not from a user control. */
static int cs42_analog_mute(struct cs42l81 *c, bool mute)
{
	return cs42_wr8(c, 0x0527, mute ? 0xff : 0x60);
}

/*
 * Headphone output mute: the 0x0220 bit-5 hold.
 *
 * This is what stock's own mute op writes for this path -- it takes a path
 * selector, resolves it to bit 5 for the headphone output and bit 3 for the
 * other sink, and sets or clears that one bit. It is the same hold the rate
 * program raises around a reprogram and the output-path disable leaves
 * raised, so it silences the output stage itself rather than the source
 * feeding it, and it does so without a power transient.
 *
 * The path shadow is deliberately not updated. Stock's mute op does not
 * touch it either, and the shadow is what selects which output path the rate
 * program configures -- a user mute must not change that.
 */
static int cs42_hp_mute(struct cs42l81 *c, bool mute)
{
	return cs42_rmw(c, 0x0220, 0x20, mute ? 0x20 : 0x00);
}

/* Bit-5 strobes on both mailbox FIFO control registers. */
static void cs42_fifo_strobe(struct cs42l81 *c)
{
	cs42_rmw(c, 0x051e, 0x20, 0x20);
	cs42_rmw(c, 0x051e, 0x20, 0x00);
	cs42_rmw(c, 0x0523, 0x20, 0x20);
	cs42_rmw(c, 0x0523, 0x20, 0x00);
}

/*
 * Transport start, stage 1. The graph is already built by .prepare, so this
 * is the unmute and the latch: bit 0 cleared and bit 1 set. Bit 0 has to be
 * cleared here because the stop path sets it and nothing else takes it down.
 */
static int cs42_transport_start(struct cs42l81 *c)
{
	int ret;

	ret = cs42_analog_mute(c, c->dai_mute);
	if (ret)
		return ret;

	if (!c->graph_built) {
		ret = cs42_play_graph(c);
		if (ret)
			return ret;
		c->graph_built = true;
	} else {
		ret = cs42_rmw(c, 0x0401, 0x01, 0x00);
		if (ret)
			return ret;
		ret = cs42_rmw(c, 0x0401, 0x02, 0x02);
		if (ret)
			return ret;
	}

	/*
	 * Transport start, stage 2.
	 *
	 * Starting playback is two steps, not one. Stage 1 unmutes and
	 * latches route 1; stage 2 installs route 3 or 4, chosen by the path
	 * shadow -- 3 when the headphone path is the live one, 4 otherwise.
	 *
	 * It matters because route 1 carries a count and a tail of zero,
	 * which makes the slot map come out at gain 0 all the way down.
	 * Stopping after stage 1 leaves the mixer programmed to silence.
	 * Routes 3 and 4 both carry a tail of 160, and 4 additionally
	 * carries two live slots.
	 *
	 * Only the computed path takes this step; the fixed image is
	 * complete on its own.
	 */
	if (graph_mode != 0) {
		int mode2 = (c->path_shadow & 0x08) ? 3 : 4;

		ret = cs42_play_graph_dynamic(c, mode2);
		if (ret)
			return ret;
		dev_dbg(&c->spi->dev,
			"transport: stage 2 graph mode=%d (path_shadow=0x%02x)\n",
			mode2, c->path_shadow);
	}

	c->state = CS42_PLAYING;
	return 0;
}

/* Transport stop. */
static int cs42_transport_stop(struct cs42l81 *c)
{
	int ret = cs42_analog_mute(c, true);

	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0401, 0x01, 0x01);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0401, 0x02, 0x00);
	if (ret)
		return ret;
	cs42_fifo_strobe(c);
	if (c->state == CS42_PLAYING)
		c->state = CS42_PREPARED;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Lifecycle								*/
/* ------------------------------------------------------------------ */

static void cs42_log_state(struct cs42l81 *c, const char *tag)
{
	u8 r2f = 0, r401 = 0, r527 = 0, r219 = 0, rc96f = 0, r0f = 0;

	if (!debug_regs)
		return;
	cs42_rd(c, 0x002f, &r2f);
	cs42_rd(c, 0x0401, &r401);
	cs42_rd(c, 0x0527, &r527);
	cs42_rd(c, 0x0219, &r219);
	cs42_rd(c, 0xc96f, &rc96f);
	cs42_rd(c, 0x000f, &r0f);
	cs42_vinfo(&c->spi->dev,
		 "%s: vol=%u (%d dB) mute=%d 2F=%02x 401=%02x 527=%02x 219=%02x C96F=%02x 0F=%02x path_shadow=%02x\n",
		 tag, c->user_vol, c->gain_l_db, c->dai_mute,
		 r2f, r401, r527, r219, rc96f, r0f, c->path_shadow);
}

/*
 * Bring the codec up and configure it for @rate.
 *
 * This is everything stock does between "a route exists" and "the first
 * buffer is submitted", in stock's order:
 *
 *	park		analog config, clock off
 *	run		divider, clock on, unfreeze
 *	analog on	power the output stage
 *	rate
 *	output path	which runs path mode 271
 *	analog unmute
 *	play graph
 *
 * The path mode is left to cs42l81_play_start(), which is where the playback
 * engine applies it in stock.
 */

static bool rate_only_change = true;
module_param(rate_only_change, bool, 0644);
MODULE_PARM_DESC(rate_only_change,
		 "Change rate without re-running the bring-up (default Y)");

static int cs42_prepare(struct cs42l81 *c, unsigned int rate)
{
	int ret;

	/*
	 * One line per bring-up. This sequence cycles the clock gate and
	 * powers the analog stage, both of which are audible, so it has to
	 * be visible when it runs more than once per boot.
	 */
	cs42_vinfo(&c->spi->dev, "prepare entry: state=%s rate=%u want=%u\n",
		 cs42_state_names[c->state], c->rate, rate);

	/*
	 * Already up at this rate: nothing to do.
	 *
	 * A playing stream must not fall through. Re-running the bring-up
	 * underneath a part that is already playing -- park, clock gate off,
	 * run, clock gate on, analog stage on -- puts a transient on the
	 * output at every step, which is audible as a series of plops before
	 * a track starts. Stock brings the codec up once and leaves it up.
	 *
	 * A genuine rate change still falls through, because the rate is part
	 * of the test.
	 */
	if ((c->state == CS42_PREPARED || c->state == CS42_PLAYING) &&
	    c->rate == rate)
		return 0;

	/*
	 * Already up, only the rate differs.
	 *
	 * cs42_set_rate() writes the rate code and nothing else, so a codec
	 * that is running does not need parking, re-running, the analog stage
	 * brought up again, the graph reapplied or the volume rewritten just
	 * to move between rates. Doing all of that is what makes a rate change
	 * audible, and a player that probes a rate before settling on the
	 * track's own -- 44100 to 8000 and back, in the case that prompted
	 * this -- pays for it twice before a note is played.
	 *
	 * The divider still steps, on both the codec and the IIS side, so the
	 * headphone hold goes up across the write and comes back down after.
	 * A stream muted by the user stays muted: dai_mute is the control and
	 * this must not clear it.
	 */
	if (rate_only_change && c->rate && c->rate != rate &&
	    (c->state == CS42_PREPARED || c->state == CS42_PLAYING)) {
		bool held = c->dai_mute;

		if (!held)
			cs42_hp_mute(c, true);
		ret = cs42_set_rate(c, rate);
		if (!held)
			cs42_hp_mute(c, false);

		if (!ret) {
			cs42_vinfo(&c->spi->dev,
				 "prepare: rate only, %u without a re-bring-up\n",
				 rate);
			return 0;
		}
		/*
		 * It did not take. Fall through and do the whole thing, which
		 * is the path that is known to work.
		 */
		dev_warn(&c->spi->dev,
			 "prepare: rate-only change failed (%d), full bring-up\n",
			 ret);
	}

	/*
	 * Log the accessory routing, if the tristar driver is loaded. It is a
	 * separate module and may be absent, so a missing symbol is not an
	 * error -- playback continues without the log.
	 *
	 * No PMIC work happens here. The rail trim the analog headphone path
	 * depends on is performed once by the bootloader, before the OS image
	 * is even loaded, and stock's audio path issues no PMIC transactions
	 * at all. Replaying it from here disturbs rails the display is using.
	 */
	{
		void (*ts_path)(struct device *) = (void (*)(struct device *))
			__symbol_get("apple_tristar_log_audio_path");

		if (ts_path) {
			ts_path(&c->spi->dev);
			__symbol_put("apple_tristar_log_audio_path");
		}
	}
	ret = cs42_state_park(c);
	if (ret)
		return ret;
	ret = cs42_state_run(c);
	if (ret)
		return ret;

	/*
	 * The analog output stage. Stock reaches this only after the run
	 * state, and stopping at run leaves the stage unpowered.
	 */
	ret = cs42_state_analog_on(c);
	if (ret)
		return ret;

	/*
	 * Rate before the output path, which is the order stock uses. The
	 * path configuration depends on the rate already being programmed,
	 * so enabling it first configures it against the previous rate.
	 */
	ret = cs42_set_rate(c, rate);
	if (ret)
		return ret;
	ret = cs42_output_path_on(c);
	if (ret)
		return ret;

	ret = cs42_analog_mute(c, c->dai_mute);
	if (ret)
		return ret;
	ret = cs42_play_graph(c);
	if (ret)
		return ret;
	c->graph_built = true;

	ret = cs42_apply_user_vol(c);
	if (ret)
		return ret;

	c->state = CS42_PREPARED;
	cs42_log_state(c, "prepare");
	return 0;
}

int cs42l81_play_prepare(void)
{
	struct cs42l81 *c = cs42l81_dev;
	int ret;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	ret = cs42_prepare(c, c->rate ? c->rate : N31_RATE_DEFAULT);
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
	if (c->state != CS42_PREPARED && c->state != CS42_PLAYING) {
		ret = cs42_prepare(c, c->rate ? c->rate : N31_RATE_DEFAULT);
		if (ret)
			goto out;
	}
	if (c->state == CS42_PLAYING) {
		ret = 0;
		goto out;
	}
	/*
	 * Path mode 18, where stock applies it: in the playback task's
	 * preamble, immediately before the producer fills the first buffer
	 * and the DMA channel is enabled. Gain re-apply follows, which
	 * cs42_transport_start() does.
	 *
	 * It sets the path shadow to 0x08, which selects the headphone path
	 * for the rate program and route 3 for the stage-2 graph.
	 *
	 * Once per stream start, like stock: the preamble runs once and the
	 * steady-state loop re-enters below it.
	 */
	ret = cs42_path_mode(c, &cs42_mode_18);
	if (ret) {
		dev_err(&c->spi->dev, "path mode 18 failed: %d\n", ret);
		goto out;
	}

	ret = cs42_transport_start(c);
	cs42_log_state(c, "play_start");
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
	ret = cs42_transport_stop(c);
	cs42_log_state(c, "play_stop");
	mutex_unlock(&c->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cs42l81_play_stop);

/*
 * The IIS driver drives the codec's clock role from its mastership profile.
 * Stock sets 0x000F bit 7 once, in the run state, and never clears it.
 */
int cs42l81_set_clock_role(bool drive)
{
	struct cs42l81 *c = cs42l81_dev;
	int ret;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
		/*
		 * Stock sets this bit once, during the run transition, and
		 * never clears it anywhere. Set it or leave it alone; do not
		 * clear it.
		 *
		 * 0x000F[3:0] in the same register is the sample-rate index,
		 * which is why this has to stay a masked read-modify-write.
		 */
	if (!drive) {
		mutex_unlock(&c->lock);
		return 0;
	}
	ret = cs42_rmw(c, 0x000f, 0x80, 0x80);
	mutex_unlock(&c->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cs42l81_set_clock_role);

int cs42l81_pre_iis_start(void)
{
	return 0;
}
EXPORT_SYMBOL_GPL(cs42l81_pre_iis_start);

/*
 * Runs after the IIS transmitter is kicked. The rate program leaves the part
 * muted at -90 dB by design, so the cached gain has to come back; stock does
 * that from its gain re-apply on the same edge.
 */
int cs42l81_post_iis_start(void)
{
	struct cs42l81 *c = cs42l81_dev;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	if (!c->dai_mute) {
		cs42_analog_mute(c, false);
		cs42_rmw(c, 0x0401, 0x02, 0x02);
	}
	cs42_apply_gains(c);
	cs42_log_state(c, "post_iis");
	mutex_unlock(&c->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(cs42l81_post_iis_start);

static void cs42_post_iis_workfn(struct work_struct *work)
{
	cs42l81_post_iis_start();
}

void cs42l81_schedule_post_iis(void)
{
	struct cs42l81 *c = cs42l81_dev;

	if (!c)
		return;
	mod_delayed_work(system_wq, &c->post_iis_work, msecs_to_jiffies(25));
}
EXPORT_SYMBOL_GPL(cs42l81_schedule_post_iis);

/*
 * Called from the IIS stop path, which must not block: the work item takes
 * c->lock and the caller may already hold it. The synchronous form belongs at
 * remove, where the work genuinely must be over.
 */
void cs42l81_cancel_post_iis(void)
{
	struct cs42l81 *c = cs42l81_dev;

	if (!c)
		return;
	cancel_delayed_work(&c->post_iis_work);
}
EXPORT_SYMBOL_GPL(cs42l81_cancel_post_iis);

/* Diagnostic, kept so the IIS driver's call site still links. */
int cs42l81_asp_hold_light(void)
{
	struct cs42l81 *c = cs42l81_dev;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	cs42_log_state(c, "asp_status");
	mutex_unlock(&c->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(cs42l81_asp_hold_light);

/*
 * The IIS driver used this to pick between two orderings of the codec and
 * transport calls. Only stock's ordering exists now.
 */
int cs42l81_get_audio_path_mode(void)
{
	return 1;
}
EXPORT_SYMBOL_GPL(cs42l81_get_audio_path_mode);

bool n31_audio_playback_active(void)
{
	struct cs42l81 *c = cs42l81_dev;

	return c && c->state == CS42_PLAYING;
}
EXPORT_SYMBOL_GPL(n31_audio_playback_active);

/* ------------------------------------------------------------------ */
/* ASoC									*/
/* ------------------------------------------------------------------ */

static int cs42_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  struct snd_soc_dai *dai)
{
	struct cs42l81 *c = snd_soc_component_get_drvdata(dai->component);
	unsigned int rate = n31_resolve_rate(params_rate(params));
	int ret;

	mutex_lock(&c->lock);
	ret = cs42_prepare(c, rate);
	mutex_unlock(&c->lock);
	return ret;
}

static int cs42_trigger(struct snd_pcm_substream *substream, int cmd,
			struct snd_soc_dai *dai)
{
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		cs42l81_schedule_post_iis();
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* The IIS CPU DAI owns the analog stop. */
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct snd_soc_dai_ops cs42_dai_ops = {
	.hw_params = cs42_hw_params,
	.trigger = cs42_trigger,
};

static int cs42_vol_info(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = CS42_VOL_MAX;
	return 0;
}

static int cs42_vol_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = c->user_vol;
	return 0;
}

static int cs42_vol_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);
	long vol = ucontrol->value.integer.value[0];
	int changed;

	if (vol < 0 || vol > CS42_VOL_MAX)
		return -EINVAL;

	mutex_lock(&c->lock);
	changed = vol != c->user_vol;
	c->user_vol = vol;
	if (changed)
		cs42_apply_user_vol(c);
	mutex_unlock(&c->lock);
	return changed;
}

static int cs42_sw_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int cs42_sw_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = c->dai_mute ? 0 : 1;
	return 0;
}

static int cs42_sw_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);
	bool mute = !ucontrol->value.integer.value[0];
	int changed;

	mutex_lock(&c->lock);
	changed = mute != c->dai_mute;
	c->dai_mute = mute;
	if (changed) {
		/*
		 * Hold outermost, so the output stage is already silenced
		 * before the source is cut and is released only once the
		 * source is back.
		 */
		if (mute) {
			cs42_hp_mute(c, true);
			cs42_analog_mute(c, true);
		} else {
			cs42_analog_mute(c, false);
			cs42_hp_mute(c, false);
		}
	}
	mutex_unlock(&c->lock);
	return changed;
}

static const DECLARE_TLV_DB_SCALE(cs42_hp_tlv, CS42_DB_MIN * 100, 100, 0);

static const struct snd_kcontrol_new cs42_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Headphones Playback Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
			  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
		.info = cs42_vol_info,
		.get = cs42_vol_get,
		.put = cs42_vol_put,
		.tlv.p = cs42_hp_tlv,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Headphones Playback Switch",
		.info = cs42_sw_info,
		.get = cs42_sw_get,
		.put = cs42_sw_put,
	},
};

static int cs42_component_probe(struct snd_soc_component *component)
{
	struct cs42l81 *c = snd_soc_component_get_drvdata(component);

	c->component = component;
	return 0;
}

static const struct snd_soc_component_driver cs42_component = {
	.probe = cs42_component_probe,
	.idle_bias_on = 1,
	.endianness = 1,
	.controls = cs42_controls,
	.num_controls = ARRAY_SIZE(cs42_controls),
};

static struct snd_soc_dai_driver cs42_dai = {
	.name = "cs42l81-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		/*
		 * Both DAIs on the link have to allow a rate before ALSA will
		 * offer it, and this is the codec half. The effective gate
		 * stays in s5l8740-i2s.c, whose component open only widens
		 * the runtime rates when hires is on, so a wider mask here
		 * changes nothing by itself.
		 */
		/*
		 * The codec side advertises the superset, including 44.1 kHz,
		 * which it has a rate code for. Whether 44.1 reaches
		 * applications is decided by the CPU DAI, because the
		 * limitation is the 12 MHz reference rather than the codec:
		 * no integer divider yields 44100. ASoC intersects the two,
		 * so keeping the gate in one place keeps allow_44100
		 * meaningful.
		 */
		.rates = N31_RATE_MASK,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cs42_dai_ops,
};

/* ------------------------------------------------------------------ */
/* sysfs								*/
/* ------------------------------------------------------------------ */

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	unsigned int reg, val;
	int ret;

	if (sscanf(buf, "%x %x", &reg, &val) != 2 || reg > 0xffff || val > 0xff)
		return -EINVAL;
	mutex_lock(&c->lock);
	ret = cs42_reg_is_pair(reg) ? cs42_wr16(c, reg, val)
				    : cs42_wr8(c, reg, val);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(reg);

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
	ret = cs42_rd(c, (u16)reg, &val);
	mutex_unlock(&c->lock);
	if (ret)
		return ret;
	dev_info(&c->spi->dev, "0x%04x = 0x%02x\n", reg, val);
	return count;
}
static DEVICE_ATTR_WO(rreg);

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	u8 r2f = 0, r401 = 0, r527 = 0, rc96f = 0, r0f = 0;

	mutex_lock(&c->lock);
	cs42_rd(c, 0x002f, &r2f);
	cs42_rd(c, 0x0401, &r401);
	cs42_rd(c, 0x0527, &r527);
	cs42_rd(c, 0xc96f, &rc96f);
	cs42_rd(c, 0x000f, &r0f);
	mutex_unlock(&c->lock);

	return sysfs_emit(buf,
			  "state=%s rate=%u vol=%u/%u (%d dB) mute=%d path_shadow=0x%02x\n"
			  "0x002F=0x%02x 0x0401=0x%02x 0x0527=0x%02x 0x000F=0x%02x 0xC96F=0x%02x\n",
			  cs42_state_names[c->state], c->rate, c->user_vol,
			  CS42_VOL_MAX, c->gain_l_db, c->dai_mute, c->path_shadow,
			  r2f, r401, r527, r0f, rc96f);
}
static DEVICE_ATTR_RO(state);

static ssize_t standby_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	int ret;

	mutex_lock(&c->lock);
	ret = cs42_state_analog_on(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(standby);

static struct attribute *cs42_attrs[] = {
	&dev_attr_reg.attr,
	&dev_attr_rreg.attr,
	&dev_attr_state.attr,
	&dev_attr_standby.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cs42);

/* ------------------------------------------------------------------ */
/* Driver								*/
/* ------------------------------------------------------------------ */

/*
 * Log the analog and backpower state exactly as the bootloader left it,
 * before this driver writes anything.
 *
 * 0x002F carries the analog-ready and ASP-sync status bits; 0x0219 is the
 * analog and backpower companion whose low three bits cs42_dac_gain() drives
 * to 1; 0xC96F is the 2.5 V backpower rail, 0x0E during the settle and 0x1E
 * when up. Reading them here is what says whether the codec arrives powered
 * or whether our own prepare is the only thing that brings it up.
 */
static void cs42_dump_analog(struct cs42l81 *c, const char *tag)
{
	static const u16 regs[] = { 0x002f, 0x0219, 0xc96f };
	static const char *const names[] = {
		"analog-ready/ASP-sync", "analog backpower companion",
		"2.5V backpower"
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		u8 v = 0;
		int ret = cs42_rd(c, regs[i], &v);

		if (ret)
			dev_info(&c->spi->dev, "codec %s: 0x%04x %-26s read failed (%d)\n",
					tag, regs[i], names[i], ret);
		else
			dev_info(&c->spi->dev, "codec %s: 0x%04x %-26s = 0x%02x\n",
					tag, regs[i], names[i], v);
	}
}

static int cs42l81_probe(struct spi_device *spi)
{
	struct cs42l81 *c;
	int ret;

	c = devm_kzalloc(&spi->dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->spi = spi;
	c->user_vol = CS42_VOL_DEFAULT;
	c->gain_l_db = (int)c->user_vol + CS42_DB_MIN;
	c->gain_r_db = c->gain_l_db;
	c->rate = N31_RATE_DEFAULT;
	c->state = CS42_PROBED;
	mutex_init(&c->lock);
	INIT_DELAYED_WORK(&c->post_iis_work, cs42_post_iis_workfn);
	spi_set_drvdata(spi, c);

	ret = devm_snd_soc_register_component(&spi->dev, &cs42_component,
					      &cs42_dai, 1);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "failed to register component\n");

	ret = sysfs_create_groups(&spi->dev.kobj, cs42_groups);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to create sysfs\n");

	cs42l81_dev = c;

	/* Baseline, before this driver writes a single register. */
	cs42_dump_analog(c, "as-found");
	dev_info(&spi->dev, "CS42L81 codec ready\n");
	return 0;
}

static void cs42l81_remove(struct spi_device *spi)
{
	struct cs42l81 *c = spi_get_drvdata(spi);

	cs42l81_dev = NULL;
	cancel_delayed_work_sync(&c->post_iis_work);

	/*
	 * Leave the part the way stock leaves it after a session rather than
	 * in whatever state the last stream produced. There is no reset line
	 * on this board, so the register file survives module unload and
	 * carries straight into the next load.
	 */
	mutex_lock(&c->lock);
	if (c->state == CS42_PLAYING)
		cs42_transport_stop(c);
	if (c->state == CS42_PREPARED)
		cs42_output_path_off(c);
	mutex_unlock(&c->lock);

	sysfs_remove_groups(&spi->dev.kobj, cs42_groups);
	c->component = NULL;
}

static void cs42l81_shutdown(struct spi_device *spi)
{
	struct cs42l81 *c = spi_get_drvdata(spi);

	if (!c)
		return;
	mutex_lock(&c->lock);
	if (c->state == CS42_PLAYING)
		cs42_transport_stop(c);
	mutex_unlock(&c->lock);
}

static const struct of_device_id cs42l81_of_match[] = {
	{ .compatible = "cirrus,cs42l81" },
	{ .compatible = "apple,338s1146" },
	{ }
};
MODULE_DEVICE_TABLE(of, cs42l81_of_match);

static const struct spi_device_id cs42l81_spi_ids[] = {
	{ "cs42l81", 0 },
	{ "338s1146", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, cs42l81_spi_ids);

/*
 * Suspend and resume.
 *
 * Across S3, stock's codec analog block is brought back by the bootloader,
 * not by the OS. This boot chain never runs that bootloader, so nothing would
 * power the analog block on resume and playback would come back silent.
 *
 * Rather than replay a sequence from here, invalidate the cached state so the
 * next .prepare runs the whole bring-up in stock's order. That sequence is
 * already known correct, and duplicating it in a resume path would be a
 * second copy to keep in step.
 *
 * key_done is cleared because the codec loses the 0x9901 unlock across a
 * power cycle. If it did not, re-issuing it is harmless -- it is idempotent
 * in stock too.
 */
static int cs42l81_pm_suspend(struct device *dev)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));

	mutex_lock(&c->lock);
	cs42_state_analog_on(c);
	mutex_unlock(&c->lock);
	return 0;
}

static int cs42l81_pm_resume(struct device *dev)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));

	mutex_lock(&c->lock);
	c->state = CS42_UNKNOWN;
	c->graph_built = false;
	c->key_done = false;
	mutex_unlock(&c->lock);

	dev_info(dev, "resume: codec state invalidated, full bring-up on next prepare\n");
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cs42l81_pm_ops,
				cs42l81_pm_suspend, cs42l81_pm_resume);

static struct spi_driver cs42l81_driver = {
	.driver = {
		.name = "cs42l81-spi",
		.of_match_table = cs42l81_of_match,
		.pm = pm_sleep_ptr(&cs42l81_pm_ops),
	},
	.id_table = cs42l81_spi_ids,
	.probe = cs42l81_probe,
	.remove = cs42l81_remove,
	.shutdown = cs42l81_shutdown,
};
module_spi_driver(cs42l81_driver);

MODULE_DESCRIPTION("CS42L81 / Apple 338S1146 codec (SPI)");
MODULE_LICENSE("GPL");
