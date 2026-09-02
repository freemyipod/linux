// SPDX-License-Identifier: GPL-2.0-only
/*
 * CS42L81 / Apple 338S1146 codec, SPI control port (iPod nano 7G / N31).
 *
 * Every register sequence here is transcribed from the RetailOS image
 * (osos.dec.bin) and is documented, access by access, in
 * docs-internal/n7g-audio/N31-REGISTER-TRACE-STOCK.md. The matching trace of
 * this driver is N31-REGISTER-TRACE-LINUX.md, and the differences between
 * them are enumerated in N31-REGISTER-TRACE-COMPARISON.md.
 *
 * Function names follow the stock routine each one transcribes, so the two
 * traces can be read side by side:
 *
 *	sub_D3280(1)  cs42_state_park		analog config, then clock off
 *	sub_D2EFC     cs42_user_key		0x9901 key, once per boot
 *	sub_D3280(3)  cs42_state_run		divider, clock on, unfreeze
 *	sub_D3280(4)  cs42_state_analog_on
 *	sub_D2D2C     cs42_output_path_on/off
 *	sub_D2F64     cs42_path_mode
 *	sub_D34C0     cs42_set_rate		three-way dispatch on mode38
 *	sub_183138    cs42_set_rate_183138
 *	sub_42D364    cs42_transport_start/stop
 *	sub_F141C     cs42_analog_mute
 *	sub_F1444     cs42_fifo_strobe
 *	sub_570620    cs42_play_graph
 *	sub_5707D8    cs42_play_graph_static
 *	sub_165BD4    cs42_slot_map
 *	sub_400330    cs42_dac_gain		gain + 2v5 backpower rail
 *	sub_3C36A4    cs42_channel_gain
 *	sub_D2F2C     cs42_apply_gains		right channel, then left
 *
 * Deliberate deviations from stock, and only these:
 *
 *  - Every poll is bounded. Stock spins on 0x002F bit 7 with no escape,
 *    which an RTOS owning the machine can afford and a kernel cannot.
 *  - The play graph and the mode-18 path config run in .prepare rather than
 *    in the transport trigger. Between them they carry a 100 ms and a 60 ms
 *    settle; run from the trigger that delay lands between the application
 *    asking for playback and the DMA being armed, and the stream underruns
 *    before it starts. ALSA calls .prepare before every start, xrun recovery
 *    included, so the ordering relative to stock is preserved.
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

/* Output gain, 0x0227. sub_D2C98 maps dB to the code the register carries. */
#define CS42_DB_MIN		(-76)
#define CS42_DB_MAX		12
#define CS42_DB_KNEE		(-50)
#define CS42_VOL_MAX		(CS42_DB_MAX - CS42_DB_MIN)
#define CS42_VOL_DEFAULT	(CS42_VOL_MAX - 32)	/* -20 dB */

/* sub_400330 moves the 2v5 backpower rail across this gain threshold. */
#define CS42_RAIL_DB		(-8)

/* Bounded replacement for stock's unbounded readiness poll. */
#define CS42_READY_POLLS	50

static bool trace_spi;
module_param(trace_spi, bool, 0644);
MODULE_PARM_DESC(trace_spi, "log every SPI frame sent to the codec");

static bool debug_regs;
module_param(debug_regs, bool, 0644);
MODULE_PARM_DESC(debug_regs, "dump codec state at each lifecycle transition");

/*
 * sub_570620's graph selector.
 *
 * The branch is not on a magic variable. Disassembled at 0x08570644:
 *
 *	ldr   r0, [pc, #0xa4]	; r0 = 0x08A8F9DC
 *	ldr.w r0, [r0, #0x17c]	; r0 = config[0x17C / 4 = 95]
 *	cbz   r0, dynamic	; zero selects the dynamic path
 *	bl    sub_5707D8	; non-zero passes the blob pointer
 *
 * 0x08A8F9DC is the same runtime config table sub_149E98 reads at indices
 * 126 and 127 from sub_D34C0, so the selector is config entry 95. Neither
 * the table nor the entry is initialised by either firmware image -- both
 * addresses sit past the end of the image -- so it is populated at runtime
 * from configuration this driver does not have.
 *
 * With no config word, the value stock would read is zero, and zero selects
 * the dynamic path. That is therefore the faithful default here. The static
 * image stays reachable, because it is what stock emits when the entry is
 * set, and the two agree: the blob's taps (9, 8) are exactly what
 * sub_174E7C computes for the state sub_5706F4 installs.
 *
 *	0 = static sub_5707D8	(config entry 95 non-zero)
 *	1 = dynamic, route 1	(config entry 95 zero -- the default)
 *	3, 4 = dynamic, routes 3 and 4
 */
/*
 * Sample-rate-converter selection in cs42_set_rate_long(). 0 follows the
 * stock condition; 1 forces it on; 2 forces it off. See the comment there.
 */
static int src_mode;
module_param(src_mode, int, 0644);
MODULE_PARM_DESC(src_mode,
	"0=stock condition, 1=force codec SRC on, 2=force it off");

static int graph_mode = 1;
module_param(graph_mode, int, 0644);
MODULE_PARM_DESC(graph_mode,
		 "sub_570620 graph: 1=dynamic route 1 (default), 0=static, 3/4=dynamic");

/*
 * MEMORY[0x892A02C], the MCLK in kHz. sub_D3280(3) selects clock divider 2
 * when this reads 12000 and 4 otherwise, and sub_D34C0 halves every entry of
 * the rate-divider table when it reads 6000.
 *
 * This has never been measured on the hardware -- every divider in the rate
 * table is 12000000/rate, which is consistent with 12 MHz but does not prove
 * it. It is a parameter rather than a constant so the assumption is visible
 * and can be moved in one place when it is measured.
 */
static unsigned int mclk_khz = 12000;
module_param(mclk_khz, uint, 0644);
MODULE_PARM_DESC(mclk_khz,
		 "MEMORY[0x892A02C], codec MCLK in kHz (12000 or 6000); UNMEASURED");

/*
 * sub_174E7C's inputs, read out of the image at 0x0892A05C..0x0892A06C.
 *
 * This block is initialised data, not BSS -- the bytes are present in
 * osos.dec.bin and 0x0892A000..0x0892A100 carries 88 non-zero bytes:
 *
 *	0x0892A05C count_l  = 2	   0x0892A05F count_r  = 0
 *	0x0892A05D idx_l_lo = 2	   0x0892A060 idx_r_lo = 2
 *	0x0892A05E idx_l_hi = 12   0x0892A061 idx_r_hi = 12
 *	0x0892A062 base_l   = 160  0x0892A064 base_r   = 160
 *
 * and the dword table at 0x0892A068 is
 *
 *	0, 27, 37, 37, 40, 54, 74, 74, 80, 107, 147, 148, 160
 *
 * so the per-slot gain sub_174E7C derives is
 *
 *	accum = (idx_lo + 1) * table[idx_hi] = 3 * 160 = 480 = 0x01E0
 *
 * which is exactly the gain sub_5707D8's static image writes into each
 * active slot, and feeding it back through the tap formula with
 * count_l = count_r = 2 and base 160 gives 9 and 8 -- the 0x0403 and 0x0404
 * that image writes. The static and dynamic paths agree exactly, and the
 * gain is derived here rather than assumed.
 */
static const u32 cs42_graph_table[] = {
	0, 27, 37, 37, 40, 54, 74, 74, 80, 107, 147, 148, 160,
};

#define CS42_IDX_LO		2
#define CS42_IDX_HI		12

/* sub_440AA4 is a plain unsigned divide. */
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
	bool key_done;			/* MEMORY[0x892A028] */
	bool graph_built;

	/*
	 * MEMORY[0x892A038]: shadow of 0x0220 bits 0x28, set by state 3 and
	 * recomputed by sub_D2F64. sub_D34C0 dispatches on it.
	 */
	u8 mode38;

	/* MEMORY[0x892A044] / [0x892A048]: cached left/right gain, in dB. */
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
 * Two write frames. sub_43CDB4 sends five bytes and is used for everything
 * except a small set of registers; sub_3FA0E0 sends six and writes the
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

/* sub_43CDB4 */
static int cs42_wr8(struct cs42l81 *c, u16 reg, u8 val)
{
	u8 tx[5] = { 0x6c, reg >> 8, reg & 0xff, 0x00, val };

	return cs42_xfer(c, tx, NULL, sizeof(tx));
}

/* sub_3FA0E0 -- writes reg and reg+1 */
static int cs42_wr16(struct cs42l81 *c, u16 reg, u8 val)
{
	u8 tx[6] = { 0x6c, reg >> 8, (reg & 0xff) | 0x80, 0x01, val, val };

	return cs42_xfer(c, tx, NULL, sizeof(tx));
}

/* sub_43CDFA */
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
 * sub_42A5D6. The mask narrows the value, not the transaction: a zero mask
 * still performs a read and a write, and stock relies on that in sub_D2F64.
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
/* Message mailbox -- sub_15A50C / sub_19A838				*/
/* ------------------------------------------------------------------ */

/*
 * 0x051E..0x0525 is a byte-oriented message FIFO, not the "coherent level
 * sampling" an earlier version of this driver described:
 *
 *	0x051E	write-side control; bit 0 latches the level pair, bit 5
 *		strobes a reset (sub_F1444 pulses it)
 *	0x051F	write-side status, bit 1 = full
 *	0x0520	free space, less one
 *	0x0521	write data port
 *	0x0523	read-side control, bit 5 strobes a reset
 *	0x0524	read-side status
 *	0x0525	read data port
 *
 * The payload is framed by sub_19A838, which appends a CRC-16 over the first
 * len+5 bytes using a table at MEMORY[0x892A0C8]. That table is in ROM and is
 * bit-exact CRC-16/ARC (reflected polynomial 0xA001), which is what the
 * kernel's crc16() computes, so nothing here is invented.
 *
 * Nothing in the audio path sends a message: sub_15A50C's only callers are
 * sub_131B84 and sub_1757B0, neither of which is reachable from sub_D3280,
 * sub_D2F64, sub_D34C0, sub_570620 or sub_42D364. This is the accessory /
 * headphone-remote channel. It is implemented and exported so the MikeyBus
 * side has the transport, and so that the audio path's *absence* of mailbox
 * traffic is a deliberate, documented match with stock rather than a gap.
 */
#define CS42_MBOX_LEN_OFF	1	/* payload length lives in byte 1 */
#define CS42_MBOX_OVERHEAD	7	/* header + the two CRC bytes */
#define CS42_MBOX_MAX		64	/* bound on the prepare-time FIFO drain */

/* sub_19A838: append the CRC-16 and return the full frame length. */
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
 * sub_15A50C: check the free space, then push the frame a byte at a time.
 * Stock returns 33 when the frame does not fit; -ENOSPC is the equivalent.
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
 * sub_345D28(9, 0, div), the first thing state 3 does -- before the clock
 * gate is opened. The divider follows MEMORY[0x892A02C], the MCLK in kHz:
 * 2 when it reads 12000, otherwise 4. Every divider in the rate table is
 * 12000000/rate, so 12 MHz is the case this board runs and 2 is the value
 * stock selects.
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

/* sub_D2C98: dB to register code. 1 dB per step to -50, 2 dB below it. */
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

/* sub_3C6244: register code back to dB. Code -64 is the mute entry. */
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
 * sub_400330: write the DAC gain, and move the 2.5 V backpower rail across
 * the -8 dB threshold.
 *
 * The rail is the class-H supply for the headphone amplifier. Stock raises
 * it when the gain comes up through -8 dB and, going the other way, arms a
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
			 * sub_345D58(155). Both delay helpers call the same
			 * primitive; sub_345D48 multiplies its argument by
			 * 1000 first, so the primitive's unit is microseconds
			 * and this settle is 155 us, not milliseconds.
			 */
			usleep_range(155, 200);
			cs42_wr8(c, 0xc96f, 0x1e);
		}
	} else if (old_db >= CS42_RAIL_DB && new_db < CS42_RAIL_DB) {
		u8 rail = 0, comp = 0;

		/*
		 * The other half of the pair, sub_1883A8: drop the 2.5 V
		 * backpower and clear the compensation bits.
		 *
		 * Stock gates it on its own shadow, MEMORY[0x892A024], which
		 * is set when the rail was raised and cleared here. This
		 * driver has no such shadow, so it reads the two registers
		 * instead and only acts when they are in the raised state --
		 * the same test the raise above uses, inverted.
		 *
		 * Without this the rail stays at 0x1E and 0x0219[2:0] stays
		 * at 1 for the life of the part once any gain has crossed
		 * the threshold, because nothing else ever writes them down.
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
 * sub_3C36A4(sel, 3, dB) for the two selectors this driver uses. Selector 0
 * is the DAC pair at 0x0227 and always routes through sub_400330; selector 1
 * is the pair at 0x0229 and is a plain wide write.
 */
static int cs42_channel_gain(struct cs42l81 *c, int sel, int db)
{
	if (sel == 0)
		return cs42_dac_gain(c, cs42_db_to_code(db));
	return cs42_wr16(c, 0x0229, cs42_db_to_code(db) & 0x7f);
}

/*
 * sub_D2F2C, the gain re-apply the playback engine runs before it submits
 * its first buffer: right channel first, then left. This is why 0x0229
 * carries a real gain in stock rather than the constant its power-up leaves.
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
/* Power states -- sub_D3280						*/
/* ------------------------------------------------------------------ */

/* sub_D2EFC: the 0x9901 key, once per boot, from inside state 3. */
static void cs42_user_key(struct cs42l81 *c)
{
	if (c->key_done)
		return;
	cs42_wr8(c, 0x9901, 0xa5);
	cs42_wr8(c, 0x9901, 0x00);
	c->key_done = true;
}

/*
 * sub_D3280(1). Configures the analog block, waits for it to report ready,
 * raises the freeze latch and drops the codec clock. State 3 is the matching
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
	 * sub_D3280(a1 == 1) does not enable the clock -- it *ends* by
	 * dropping it, via sub_41CBD8(v6, 0), and simply assumes it was on
	 * when it was entered. Stock always satisfies that: the bootloader's
	 * own analog power-up (sub_1310) runs with the clock on, and every
	 * later entry comes from state 3, which leaves it on.
	 *
	 * Nothing in this driver satisfied it. State 3 was the only place
	 * that ever turned the clock on, and it runs *after* this, so every
	 * park wrote its whole sequence into a gated bus. The writes were
	 * issued, the SPI transfers completed, and nothing landed: 0x0225
	 * stayed 0x00 instead of 0x33 and 0x0220 kept none of the 0x78 --
	 * which is the analog output block never being powered up at all,
	 * while the codec still answered reads and every register state 3
	 * touches looked perfect.
	 *
	 * Establish the precondition here rather than in the caller, so it
	 * holds for every entry to park, not just the one through prepare.
	 */
	cs42_soc_clk_divider(c);
	cs42_soc_clk_gate(c, true);

	/*
	 * The 0x9901 key has to be in before the 0x02xx writes below, or the
	 * codec discards them.
	 *
	 * sub_D2EFC applies it once per boot and records that in
	 * MEMORY[0x892A028]; stock issues it from state 3, which in its flow
	 * has already run long before any state 1. This driver enters park
	 * first, so the key was still outstanding and every 0x02xx write in
	 * this function was rejected -- silently, because the SPI transfer
	 * itself completes normally.
	 *
	 * The evidence is in a trace_spi capture: park issues
	 *   6c 02 a5 01 33 33      0x0225 = 0x33
	 *   6c 02 20 00 78         0x0220 = 0x78
	 * and state 3's very next read-modify-write of 0x0220 stores 0x28,
	 * which is (0x00 & ~0x28) | 0x28 -- so the read came back 0x00 and
	 * neither write had taken. Page 0x00 writes in the same sequence
	 * (0x0075, 0x0006, 0x0007) all landed, and everything after the key
	 * landed. 0x0225 is the analog output register, so what this cost was
	 * the analog block never powering up, with the codec still answering
	 * reads and every register state 3 touches looking correct.
	 *
	 * cs42_user_key() is idempotent via c->key_done, so state 3's call
	 * stays where stock has it and simply becomes a no-op after this.
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
	 * Stock spins here forever. sub_D3280 at EA 0x0D347C is
	 *
	 *	delay(1); read(0x002F); if (!(v & 0x80)) goto back;
	 *
	 * with no counter, no deadline and no error arm -- the bpl is the
	 * only way out. That is the one comment in this driver the decomp
	 * actually backs.
	 *
	 * We cannot copy it: an unbounded spin here wedges whichever thread
	 * ALSA called us on. The 1 ms per iteration matches stock's delay(1)
	 * argument; the count of 50 is OURS and has no stock basis.
	 *
	 * What we must not do is carry on. This used to warn and fall
	 * through into the 0x0006 and 0x0007 writes below -- writes stock
	 * can never reach without bit 7 set, because it is still in the
	 * loop. Continuing puts the codec in a state stock never produces,
	 * which is worse than failing. Fail instead.
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
 * sub_D3280(3). Selects the codec clock divider, turns the clock back on,
 * sends the key once, releases the freeze latch and reads the two trim bytes
 * stock caches at MEMORY[0x892A054].
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
	c->mode38 = 0x28;
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
/* Path mode -- sub_D2F64 / sub_D2D2C					*/
/* ------------------------------------------------------------------ */

/*
 * sub_D2F64 evaluated for the three modes this driver uses. The skeleton is
 * fixed and only the values and two conditional blocks differ, so the shape
 * is written once and the per-mode values come from a table.
 *
 * 0x0204 and 0x0203 are issued even when their mask is zero, because stock
 * issues them: sub_42A5D6 is a read and a write regardless of the mask.
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
	u8 mode38;		/* MEMORY[0x892A038] afterwards */
};

/* sub_D2F64(271), reached through sub_D2D2C(1). */
static const struct cs42_path_mode cs42_mode_271 = {
	.r06 = 0x00, .r220 = 0x00,
	.has_r0d = false,
	.r206 = 0x3d,
	.has_r205 = true, .r205 = 0x5a,
	.r204_mask = 0x00, .r204 = 0x00,
	.r203 = 0x00,
	.has_eq = true,
	.settle_ms = 105,
	.mode38 = 0x00,
};

/* sub_D2F64(18), applied by the playback engine before its first buffer. */
static const struct cs42_path_mode cs42_mode_18 = {
	.r06 = 0x04, .r220 = 0x08,
	.has_r0d = true, .r0d = 0x00,
	.r206 = 0x34,
	.has_r205 = false,
	.r204_mask = 0x03, .r204 = 0x00,
	.r203 = 0xc0,
	.has_eq = false,
	.settle_ms = 60,
	.mode38 = 0x08,
};

/* sub_D2F64(6), the companion of sub_D2D2C(0). */
static const struct cs42_path_mode cs42_mode_6 = {
	.r06 = 0x04, .r220 = 0x00,
	.has_r0d = true, .r0d = 0x00,
	.r206 = 0x34,
	.has_r205 = false,
	.r204_mask = 0x03, .r204 = 0x00,
	.r203 = 0xc0,
	.has_eq = false,
	.settle_ms = 60,
	.mode38 = 0x00,
};

static int cs42_set_rate(struct cs42l81 *c, unsigned int rate);

static int cs42_path_mode(struct cs42l81 *c, const struct cs42_path_mode *m)
{
	int ret;

	ret = cs42_rmw(c, 0x0006, 0x04, m->r06);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0220, 0x28, m->r220);
	if (ret)
		return ret;
	c->mode38 = m->mode38;
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
	 * sub_D2F64 ends with a BL to sub_18311C at EA 0x000D31F2 -- found
	 * by scanning the image for Thumb BL targets, because the .c export
	 * lists no call site. sub_18311C saves MEMORY[0x892A034], clears it,
	 * calls sub_D34C0 and restores it: re-run the rate programming.
	 *
	 * It matters because sub_D34C0 picks its arm on the shadow this
	 * function writes. 0x28 takes the short arm, which programs only the
	 * mute bracket, 0x000F and 0x012F; 0x08 takes the long arm, the only
	 * one that writes 0x0121, 0x0122, 0x0130, 0x0131, 0x0223, 0x0224 and
	 * 0x0222. cs42_state_run() sets the shadow to 0x28 at the top of
	 * every prepare, so without this second pass the converter registers
	 * keep their bring-up values -- the 48 kHz ones -- at every rate.
	 *
	 * Measured on a freshly power-cycled part playing 44.1 kHz without
	 * it: 0x0222 = 0x0c, 0x0130 = 0xcc, 0x0131 = 0x01, i.e. the codec
	 * configured for 48 kHz while the serialiser clocked 44117.65 Hz --
	 * 2900/4900 Hz spurs at -33 dBc and 0.102% THD. With it, and the
	 * converter engaged, -55 dBc and 0.0067%.
	 */
	if (c->rate)
		return cs42_set_rate(c, c->rate);
	return 0;
}

/* sub_D2D2C(1) */
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

/* sub_D2D2C(0): the mode runs first, then the path is taken down. */
static int cs42_output_path_off(struct cs42l81 *c)
{
	int ret = cs42_path_mode(c, &cs42_mode_6);

	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0206, 0x3f, 0x00);
	if (ret)
		return ret;
	return cs42_rmw(c, 0x0207, 0x3f, 0x00);
}

/*
 * sub_D3280(4) -- the ANALOG BRING-UP, not a power-down.
 *
 * This was called cs42_state_standby and described as "a power-down:
 * analog enable off, rail down, gain to the mute code". The register
 * sequence below is a byte-exact transliteration and always was; only the
 * name and the call site were wrong, and the name is what kept it off the
 * playback path.
 *
 * What settles it is the state applier sub_D3700, which IDA omits, read out
 * of the raw image at 0x000D3752:
 *
 *	d3756  cmp  r1, #4       ; target mode
 *	d3758  bne  0xd3768
 *	d375a  cmp  r0, #3       ; current mode
 *	d375c  bcs  0xd3768      ; skip when current >= 3
 *	d375e  movs r0, #3
 *	d3760  bl   0xd3280      ; sub_D3280(3) FIRST
 *	d3768  ldrb r0, [r5, #4]
 *	d376a  bl   0xd3280      ; then sub_D3280(target)
 *
 * Mode 4 is staged THROUGH mode 3. Nothing stages through the run state to
 * reach a power-down; 1 (park) -> 3 (run) -> 4 is a climb.
 *
 * The body agrees. From osos.dec.bin.ida.c, decimal register numbers
 * decoded: 0x219 |= 0x78, 0x201[7:5] = 0x40, 0xC81F = 0xFF, 0xC85F = 0x0F,
 * 0xC96F = 0x0E (the 2.5 V backpower), 0x223 = 0x08, 0x224 = 0x09, then
 * sub_400330(64)/(65) applying gain, 0x229/0x22A = 0x41, and finally
 * 0x00E[7:6] = 0x40 closing the config guard. That is an output stage being
 * powered and gained, not torn down.
 *
 * Corroborated by measurement: our live codec dump read 0x0219 = 0x00,
 * i.e. that 0x78 was never set, and the device produces no plop on reboot
 * -- nothing to discharge, because the analog stage never came up.
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
/* Sample rate -- sub_D34C0 / sub_183138				*/
/* ------------------------------------------------------------------ */

/* sub_183138(code, 1). Ends muted with the 0x0220 bit-5 hold raised. */
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

/* sub_D34C0, short branch: mode38 carries both 0x08 and 0x20. */
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
 * sub_D34C0, long branch: mode38 carries 0x08 but not 0x20. Mutes, raises
 * the hold, programs the rate inside the 0x000E bracket, drops the hold and
 * restores the cached gain.
 *
 * Stock chooses between the native and converted arms with two config-flag
 * lookups crossed with the rate. Without that config the native case is
 * taken to be rate code 12 exactly, which is the test sub_183138 uses.
 */
static int cs42_set_rate_long(struct cs42l81 *c, u8 code)
{
	/*
	 * Stock's condition is
	 *
	 *	(sub_149E98(126) && rate != 48000) ||
	 *	(rate == 32000 && !sub_149E98(127))
	 *
	 * where sub_149E98(n) is *(u32 *)(0x8A8F9DC + 4n) != 0.
	 *
	 * That table is resolvable. Its initialiser zeroes all 0xBB entries
	 * and then populates them only when sub_16297C("_enable_options", ..)
	 * succeeds and reports the file present -- an Apple diagnostic options
	 * file under iPod_Control\Device\. When it is absent the function
	 * returns with the table still entirely zero, which is the retail
	 * case.
	 *
	 * With entries 126 and 127 zero the first clause is false and the
	 * second reduces to rate == 32000, i.e. rate code 9. Every other rate
	 * takes the native arm here.
	 *
	 * Note this is the opposite sense to sub_183138, which tests its rate
	 * code against 12 directly in the instruction stream. The two
	 * functions genuinely differ; they are not two spellings of one test.
	 */
	/*
	 * src_mode: 0 = stock condition (default), 1 = force SRC on,
	 * 2 = force SRC off. An experiment handle, not a feature.
	 *
	 * The stock condition above reduces to "rate == 32000" only if
	 * sub_149E98(126) is zero, which is what we concluded from the
	 * _enable_options file being absent on a retail device. That
	 * conclusion is worth testing, because 44.1 kHz is measurably
	 * distorted and 48 kHz is not, and the difference between them is
	 * exactly the thing an SRC would absorb:
	 *
	 *	48000 -> clkdiv 250, 12 MHz / 250 = 48000.00 exactly
	 *	44100 -> clkdiv 272, 12 MHz / 272 = 44117.65
	 *
	 * No integer divider off a 12 MHz reference produces 44100, so at
	 * "44.1" the serialiser really clocks 44117.6 while the codec is
	 * told rate code 10. If sub_149E98(126) is in fact non-zero on real
	 * hardware, stock's condition is true for every rate but 48000 and
	 * stock engages the SRC exactly where we do not.
	 */
	/*
	 * Sample-rate-converter selection, stock's condition.
	 *
	 * src_mode overrides it for comparison: 1 forces on, 2 forces off.
	 */
	/*
	 * Engage the converter for every rate except 48 kHz.
	 *
	 * Stock's test is
	 *
	 *	sub_149E98(126) && rate != 48000 || rate == 32000 && !sub_149E98(127)
	 *
	 * and it was implemented here as "rate == 32000", which is what it
	 * reduces to only if sub_149E98(126) is zero. Measured at the jack,
	 * that assumption is wrong: with the converter bypassed a 1 kHz tone
	 * at 44.1 kHz shows 2900/4900 Hz spurs at -33 dBc and 0.102% THD,
	 * and with it engaged the same tone gives -55 dBc and 0.0067% --
	 * better THD than the 48 kHz control. So sub_149E98(126) is
	 * non-zero on this hardware and the condition is "not 48 kHz".
	 *
	 * 48 kHz is the one rate a 12 MHz reference divides exactly
	 * (12e6/250), so it is also the one rate with nothing for the
	 * converter to correct.
	 */
	bool src = code != 12;

	if (src_mode == 1)
		src = true;
	else if (src_mode == 2)
		src = false;
	int ret;

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
		/* src_mode=2 only: bypass, for comparison against the above. */
		cs42_rmw(c, 0x0131, 0x01, 0x01);
		cs42_wr8(c, 0x0223, 0x08);
		cs42_wr8(c, 0x0224, 0x09);
	}
	cs42_wr8(c, 0x0222, src ? 12 : code);

	ret = cs42_rmw(c, 0x000e, 0xc0, 0x40);
	if (ret)
		return ret;
	ret = cs42_rmw(c, 0x0220, 0x20, 0x00);
	if (ret)
		return ret;
	msleep(60);

	return cs42_apply_gains(c);
}

/* sub_D34C0's three-way dispatch, on the shadow stock maintains. */
static int cs42_set_rate(struct cs42l81 *c, unsigned int rate)
{
	const struct n31_rate_cfg *r = n31_find_rate(rate);
	int ret;

	if (!r)
		return -EINVAL;

	if (!(c->mode38 & 0x08))
		ret = cs42_set_rate_183138(c, r->cs42_rate_code);
	else if (c->mode38 & 0x20)
		ret = cs42_set_rate_short(c, r->cs42_rate_code);
	else
		ret = cs42_set_rate_long(c, r->cs42_rate_code);

	if (!ret)
		c->rate = rate;
	return ret;
}

/* ------------------------------------------------------------------ */
/* Play graph -- sub_570620 / sub_5707D8 / sub_165BD4			*/
/* ------------------------------------------------------------------ */

/* sub_5707D8, the static image, verbatim. */
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
	 * No 0x0500 write here. This used to do cs42_wr8(c, 0x0500, 0x05)
	 * and it was ours, not stock's.
	 *
	 * Stock writes 0x0500 exactly once in the whole image, at EA
	 * 0x00570AF4 inside sub_5707D8 -- an override branch reached only
	 * when MEMORY[0x8A8FB58] is non-zero, sitting between a 0x401 = 0x12
	 * write and a 0x528 status read. It is not on the audio path and it
	 * is not a sample-rate or bit-depth setting.
	 *
	 * The 0x400/0x500/0x51E-0x534 space on this part is the DSP and
	 * write-sequencer subsystem, not the serial audio port. This write
	 * was put here on the assumption that 0x0500 was AIF1_BCLK_Ctrl.
	 * That assumption is unsupported.
	 */
	cs42_rd(c, 0x0528, &status);
	if (debug_regs)
		dev_info(&c->spi->dev, "graph: 0x528=0x%02x\n", status);
	return 0;
}

/*
 * sub_165BD4(side): eleven slots of three registers each. The source index
 * runs from 0 on the left and 10 on the right; the terminator marks the slot
 * that carries the tail value.
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
 * sub_570620's dynamic branch.
 *
 * sub_5706F4 installs the per-side counts and bases, transcribed from its
 * switch:
 *
 *	route 1	count_l 0, count_r 0, base_l 0,   base_r 0
 *	route 3	count_l 0, count_r 0, base_l 160, base_r 160
 *	route 4	count_l 2, count_r 0, base_l 160, base_r 160
 *
 * sub_174E7C then derives the tap indices. Its accumulators are only
 * refreshed when the matching count is non-zero, and the count multiplies
 * the accumulator in the tap term, so a zero count drops the gain out:
 *
 *	if (count_l) accum_l = (idx_l_lo + 1) * table[idx_l_hi]
 *	tap_l = (base_l + accum_l * count_l + 159) / 160 + 2
 *	if (count_r) accum_r = (idx_r_lo + 1) * table[idx_r_hi]
 *	tap_r = (base_r + accum_r * count_r + 159) / 160 + 1
 *
 * With sub_5706F4's count_r of 0 this yields tap_r = 2 on route 4, where
 * sub_5707D8's static image writes 8. That is not a defect in either
 * transcription: 8 requires count_r = 2, which sub_5706F4 never installs, so
 * the static image was baked from a state this function does not produce.
 * The left side matches exactly (9).
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

	/* sub_174E38(side): the packed index nibbles, per active slot. */
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
		dev_info(&c->spi->dev,
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
/* Transport -- sub_42D364 / sub_F141C / sub_F1444			*/
/* ------------------------------------------------------------------ */

/* sub_F141C */
static int cs42_analog_mute(struct cs42l81 *c, bool mute)
{
	return cs42_wr8(c, 0x0527, mute ? 0xff : 0x60);
}

/* sub_F1444: bit-5 strobes on both mailbox FIFO control registers. */
static void cs42_fifo_strobe(struct cs42l81 *c)
{
	cs42_rmw(c, 0x051e, 0x20, 0x20);
	cs42_rmw(c, 0x051e, 0x20, 0x00);
	cs42_rmw(c, 0x0523, 0x20, 0x20);
	cs42_rmw(c, 0x0523, 0x20, 0x00);
}

/*
 * sub_42D364(1). The graph is already built by .prepare, so this is the
 * unmute and the latch: bit 0 cleared and bit 1 set, which is what
 * sub_570620 does around its own graph write. Bit 0 has to be cleared here
 * because sub_42D364(0) sets it on every stop and nothing else takes it down.
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
	 * sub_42D364(2) -- the second transport call, which this driver never
	 * made.
	 *
	 * Starting playback is two steps in stock, not one:
	 *
	 *	sub_42D364(1): 0x892A058 = 1, sub_F141C(1) (the 0x0527 analog
	 *	               unmute), then sub_570620(1)
	 *	sub_42D364(2): resolves to 3 or 4, then sub_570620(3 or 4)
	 *
	 * and the resolution is
	 *
	 *	a1 = (MEMORY[0x892A038] & 8) ? 3 : 4;
	 *
	 * where 0x892A038 is the shadow of 0x0220 & 0x28 -- so the mode-18
	 * path leaves it 0x08 and selects 3, while 271 and 6 select 4.
	 *
	 * This matters because sub_5706F4 gives mode 1 a count and a tail of
	 * zero, which makes sub_165BD4 emit a slot map that is gain 0 all the
	 * way down. Stopping after step 1, as this function did, therefore
	 * left the mixer programmed to silence. Modes 3 and 4 both carry a
	 * tail of 160, and 4 additionally carries two live slots.
	 *
	 * Only the dynamic path takes this step: stock's static-blob branch
	 * calls sub_5707D8 and returns without going near sub_5706F4.
	 */
	if (graph_mode != 0) {
		int mode2 = (c->mode38 & 0x08) ? 3 : 4;

		ret = cs42_play_graph_dynamic(c, mode2);
		if (ret)
			return ret;
		dev_dbg(&c->spi->dev,
			"transport: stage 2 graph mode=%d (mode38=0x%02x)\n",
			mode2, c->mode38);
	}

	c->state = CS42_PLAYING;
	return 0;
}

/* sub_42D364(0) */
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
	dev_info(&c->spi->dev,
		 "%s: vol=%u (%d dB) mute=%d 2F=%02x 401=%02x 527=%02x 219=%02x C96F=%02x 0F=%02x mode38=%02x\n",
		 tag, c->user_vol, c->gain_l_db, c->dai_mute,
		 r2f, r401, r527, r219, rc96f, r0f, c->mode38);
}

/*
 * The configuration stock performs between "a route exists" and "the first
 * buffer is submitted", in stock's order:
 *
 *	sub_D3280(1)	analog config, clock off
 *	sub_D3280(3)	divider, clock on, unfreeze
 *	sub_D2D2C(1)	output path, which runs sub_D2F64(271)
 *	sub_D34C0	rate -- dispatches on the shadow D2F64 just wrote
 *	sub_F141C(1)	analog unmute
 *	sub_570620	play graph
 *	sub_D2F64(18)	the playback engine's path mode
 *	sub_D2F2C	gain re-apply, right channel then left
 *
 * The rate call comes after the path mode because it dispatches on the
 * shadow the path mode maintains; running it first takes a branch stock
 * would never take.
 */

static int cs42_prepare(struct cs42l81 *c, unsigned int rate)
{
	int ret;

	/*
	 * One line per bring-up. This is the sequence that plops -- the
	 * clock gate goes ON/OFF/ON and the analog stage powers -- so it
	 * needs to be visible when it happens more than once per boot,
	 * which is exactly the bug this caught.
	 */
	dev_info(&c->spi->dev, "prepare entry: state=%s rate=%u want=%u\n",
		 cs42_state_names[c->state], c->rate, rate);

	/*
	 * Already up at this rate: nothing to do.
	 *
	 * CS42_PLAYING used to fall through here, so a second hw_params on a
	 * running stream re-ran the whole bring-up underneath it -- park,
	 * clock gate off, run, clock gate on, analog stage on -- on a part
	 * that was already playing. Every one of those steps is a transient
	 * on the output, which is the "plop plop plop" before a track starts:
	 * measured as "prepare entry: state=playing" immediately followed by
	 * a clock gate OFF, once per mpg123 invocation.
	 *
	 * Stock brings the codec up once and leaves it up; there is no path
	 * in the image that re-parks a playing part to prepare it again. A
	 * genuine rate change still falls through, because the rate is part
	 * of the test.
	 */
	if ((c->state == CS42_PREPARED || c->state == CS42_PLAYING) &&
	    c->rate == rate)
		return 0;

	/*
	 * Two stages that are not codec register writes, and which a
	 * previous refactor of this file dropped entirely.
	 *
	 * The PMIC half trims the sibling LDOs the analog headphone path
	 * runs from (gpio-d1830.c, sub_23EC in the stock *bootloader* --
	 * so nothing in our boot chain performs it). The tristar half
	 * reports the accessory routing. Both are weak references: these
	 * drivers are separate modules and either may be absent, so a
	 * missing symbol is logged and playback continues rather than
	 * failing.
	 */
	{
		int (*rails)(void);
		void (*ts_path)(struct device *);

		/*
		 * No PMIC call here.
		 *
		 * This used to __symbol_get("d1830_audio_rails") and run the whole
		 * board rail trim on every codec prepare, on the belief that it
		 * "trims the sibling LDOs the analog headphone path runs from".
		 * That belief has no support in the decomp, and the sequence it was
		 * replaying does not exist in the OS image at all.
		 *
		 * sub_23EC and sub_27F4 are BOOTLOADER functions. sub_23EC has one
		 * caller, sub_27F4 at EA 0x29DC; sub_27F4 has one caller, the C
		 * entry sub_E2C at EA 0x0E48 -- before DRAM init, before the OS
		 * image is loaded. The rewrite it performs, "bic r0,#0xd0 / orr
		 * r0,#0x10", occurs exactly once in the whole firmware, at
		 * bootloader offset 0x24F8. It occurs ZERO times in osos.dec.bin.
		 *
		 * The OS cannot write registers 0x14-0x17 even if it wanted to: its
		 * only path to them is sub_2D404 cases 1-4, whose sole caller
		 * sub_2A120 maps consumer ids 1..10 to rail indices 6..15, so those
		 * cases are dead code. Stock's audio path issues no PMIC
		 * transactions at all, and stock's own codec driver
		 * (sphwDACCS42L81.c, sub_400330) touches no rail -- it handles the
		 * 2.5 V back-power concern with a timestamp and a timer.
		 *
		 * And the trim has already run on this device: the as-found probe
		 * dump reads 0x14=0x0b 0x15=0x09 0x16=0x09 0x17=0x09, which are
		 * exactly the codes sub_23EC computes for fuse index 0. Replaying it
		 * from here re-cleared ACTIVE1 bits 6 and 7 with the panel live,
		 * which is the white screen, and it is what the guards in
		 * d1830_sec_trim_seq() were patching over.
		 */

		ts_path = (void (*)(struct device *))
			__symbol_get("apple_tristar_log_audio_path");
		if (ts_path) {
			ts_path(&c->spi->dev);
			__symbol_put("apple_tristar_log_audio_path");
		}
	}

	/*
	 * graph_built is NOT cleared here any more.
	 *
	 * sub_570620, the play graph, is not on the streaming path in stock at
	 * all. Its only caller is sub_42D364, and sub_42D364 has exactly four
	 * call sites: three in the route-event task at 0x08587E60, one in
	 * sub_500E54, and the codec output-enable op at 0x000D314C. None of
	 * them is the buffer/DMA path, and the MeCCAOutputTask preamble does
	 * not touch it.
	 *
	 * Clearing the flag here forced a full graph rebuild, and its 100 ms
	 * settle, on every .prepare -- which ALSA calls before every start and
	 * every xrun recovery. Stock pays that cost on a route change, not per
	 * stream.
	 */

	ret = cs42_state_park(c);
	if (ret)
		return ret;
	ret = cs42_state_run(c);
	if (ret)
		return ret;

	/*
	 * sub_D3280(4), which sub_D3700 reaches only after mode 3. We stopped
	 * at mode 3 and never ran this, so the analog output stage was never
	 * powered -- 0x0219 read 0x00 instead of 0x78 on a live device.
	 */
	ret = cs42_state_analog_on(c);
	if (ret)
		return ret;

	/*
	 * cs42_mailbox_drain() used to run here. Removed: stock's audio path
	 * does not drain the mailbox.
	 *
	 * 0x051E..0x0525 is the MikeyBus accessory message channel, not an
	 * audio block -- it just shares the 0x05xx page with the audio graph
	 * registers. Its users are sub_15A50C (latch/sample), sub_1326D2
	 * (read-and-discard ack), sub_155D74 (the consumer of the
	 * _mikeybus_bulk_debug option), sub_14DD16, sub_15409C and sub_19A07C,
	 * and **none of them is reachable from the audio call closure** (94
	 * functions, depth 4).
	 *
	 * What the audio path *does* touch there is sub_F1444, which strobes
	 * bit 5 of 0x051E and 0x0523 -- and that is implemented faithfully in
	 * cs42_fifo_strobe(). Draining the FIFO as well was this driver's own
	 * invention, carried over from a pre-refactor stage list.
	 */

	/*
	 * Rate before the output path, which is the order stock uses and the
	 * reverse of what this function did. The path configuration depends
	 * on the rate that is already programmed, so enabling it first
	 * configures it against the previous rate.
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

	/*
	 * cs42_mode_18 used to be applied here. It is removed, and the reason
	 * matters because it changes which graph the transport builds.
	 *
	 * sub_D2F64's argument decides the path shadow MEMORY[0x892A038]:
	 *
	 *	if (a1 & 0x200) { ... }
	 *	else {
	 *		if (a1 & 2) {...} else v3 |= 0x20;
	 *		if (a1 & 4) {...} else v3 |= 0x08;
	 *		MEMORY[0x892A038] = v3 & 0x28;
	 *	}
	 *
	 * with v3 = 0 on entry. So the shadow only becomes 0x08 for an
	 * argument with **bit 2 clear**, and 18 is such a value while 271 and
	 * 6 are not -- both of those leave it 0x00.
	 *
	 * That shadow is what sub_42D364(2) resolves on:
	 *
	 *	a1 = (MEMORY[0x892A038] & 8) ? 3 : 4;
	 *
	 * so mode 18 steered the transport to graph mode 3, whose slot map
	 * carries a tail of 160 and **no per-slot gain at all**. Mode 4
	 * carries two live slots at gain 480 -- and mode 4's output is
	 * byte-identical to the fixed-constant path in sub_5707D8, which is
	 * strong corroboration that mode 4 is the real playback graph.
	 *
	 * REFUTED, 2026-09-01. The paragraph that stood here claimed the only
	 * literal sub_D2F64 arguments in the image were 271 and 6, that the
	 * sole other call site only restored a saved mode from a struct field
	 * (ldrh r0, [r5, #0xe]), and therefore that "nothing applies 18".
	 *
	 * sub_D2F64 has THREE wrappers, not two: sub_D2D2C, the state applier
	 * at 0x000D3700 (which is the ldrh site, at EA 0x080D37B0), and
	 * sub_4142D0. sub_4142D0 dispatches slot +8 of the codec op table at
	 * 0x0891DE74, which is 0x080D2F65 -- so sub_4142D0(x) IS sub_D2F64(x).
	 *
	 * And it is called with the literal 18, at EA 0x080B5B1C, in the
	 * preamble of the MeCCAOutputTask -- the playback task itself, whose
	 * body is the unnamed function at 0x000B5AE0 reached through the
	 * thunk at 0x080A8E52. Two further literals exist on the record
	 * paths: 0x601 at 0x0826DE3E and 0xA01 at 0x0826DE50.
	 *
	 * So stock DOES apply 18, on the playback path, once per stream in
	 * the task preamble, immediately before the producer fills the first
	 * buffer and the PL080 channel is enabled.
	 *
	 * The consequence for the paragraph above is direct: with 18 applied,
	 * MEMORY[0x892A038] is 0x08, so sub_42D364(2) resolves to mode 3, not
	 * mode 4. The preference for mode 4 recorded above rests entirely on
	 * the refuted claim and must be re-derived before it is trusted.
	 * Left as it is for now rather than changed blind -- it is an audio
	 * path change and belongs in its own flash.
	 *
	 * cs42_output_path_on() already performs sub_D2D2C(1) -- 0x0206/0x0207
	 * masked to 8, then mode 271 -- which leaves the shadow at 0. Dropping
	 * this call lets that stand, so the transport resolves to mode 4.
	 *
	 * Which argument sub_D2D2C takes on the music path is NOT established:
	 * it is reached only through a vtable and has no direct callers. It
	 * does not matter for this, because 271 and 6 both leave the shadow 0.
	 */
	/*
	 * A second 0x054F RMW used to sit here. Stock issues that register
	 * exactly once in the whole image -- sub_42A5D6(1359, 240, 0), as
	 * step 1 of sub_570620 -- which cs42_play_graph() already does. This
	 * one was ours, and repeating a masked write after the path mode is
	 * not something stock ever does.
	 */
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
	 * sub_D2F64(18), where stock has it: the MeCCAOutputTask preamble,
	 * at EA 0x080B5B1C, immediately before the producer callback fills
	 * the first buffer and the PL080 channel is enabled. sub_D2F2C
	 * (gains) follows at 0x080B5B20, which cs42_transport_start does.
	 *
	 * This driver applied no 18 at all, on a claim that nothing in the
	 * image applies it. That was refuted: sub_4142D0 is a third wrapper
	 * for sub_D2F64 -- it dispatches slot +8 of the codec op table at
	 * 0x0891DE74, which is 0x080D2F65 -- and it is called with the
	 * literal 18 on the playback path.
	 *
	 * Applying it sets the path shadow MEMORY[0x892A038] to 0x08, which
	 * is what the stage-2 graph resolves on: sub_42D364(2) computes
	 * (shadow & 8) ? 3 : 4. So this also moves that graph from 4 to 3
	 * without a separate change -- cs42_transport_start already derives
	 * its mode from c->mode38.
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
 * Stock sets 0x000F bit 7 once, in state 3, and never clears it.
 */
int cs42l81_set_clock_role(bool drive)
{
	struct cs42l81 *c = cs42l81_dev;
	int ret;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	/*
	 * Stock SETS 0x000F bit 7 exactly once, at EA 0x080D3312 inside
	 * the sub_D3280(3) transition, and NEVER clears it anywhere in
	 * the image. Those are the only writes to bit 7 that exist.
	 *
	 * So clearing it, which this used to do whenever drive was
	 * false, has no counterpart in stock at all. Nothing in the
	 * decomp ties this bit to a clock role either -- that reading
	 * was ours. Set it or leave it; never clear it.
	 *
	 * Note 0x000F[3:0] in the same register is the sample-rate
	 * index, which is why this must stay a masked RMW.
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
 * Runs after the IIS transmitter is kicked. sub_183138 leaves the part muted
 * at -90 dB by design, so the cached gain has to come back; stock does that
 * from sub_D2F2C on the same edge.
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
 * c->lock and the caller may already hold it. The synchronous form belongs
 * at remove, where the work genuinely must be over.
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
	if (changed)
		cs42_analog_mute(c, mute);
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
		 * offer it, and this is the codec half. It carried the base
		 * mask while the CPU DAI carried the hi-res one, so 88.2 and
		 * 96 kHz were refused at open with hires=1 set and the i2s
		 * hw_params callback never even ran.
		 *
		 * The real gate stays in s5l8740-i2s.c: its component open
		 * only widens the runtime rates when hires is on, so with the
		 * default this mask being wider changes nothing.
		 */
		/*
		 * The codec side advertises the superset, including 44.1 kHz,
		 * which it has a rate code for (10). Whether 44.1 is offered
		 * to applications is decided by the CPU DAI, because the
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
			  "state=%s rate=%u vol=%u/%u (%d dB) mute=%d mode38=0x%02x\n"
			  "0x002F=0x%02x 0x0401=0x%02x 0x0527=0x%02x 0x000F=0x%02x 0xC96F=0x%02x\n",
			  cs42_state_names[c->state], c->rate, c->user_vol,
			  CS42_VOL_MAX, c->gain_l_db, c->dai_mute, c->mode38,
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
 * Log the analog/backpower state exactly as the bootloader left it, before
 * this driver writes anything.
 *
 * 0x002F carries the analog-ready and ASP-sync status bits; 0x0219 is the
 * analog/backpower companion whose low three bits cs42_dac_gain drives to 1;
 * 0xC96F is the 2.5 V backpower rail (0x0E during the settle, 0x1E when up).
 * Reading them here is what tells us whether the codec arrives powered or
 * whether our own prepare is the only thing that brings it up.
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
 * Suspend / resume.
 *
 * On S3, stock's codec analog block is brought back by the *bootloader*
 * (sub_1314, on the hibernate arm), not by RetailOS. Our boot chain never
 * runs that bootloader, so on resume nothing would power the analog block
 * and playback would come back silent in exactly the way it did before the
 * 0x9901 key ordering was fixed.
 *
 * Rather than replay a sequence from here, invalidate the cached state so
 * the next .prepare runs the whole bring-up in stock's order -- park (which
 * now issues the key first), run, mailbox, rate, path, graph, gains. That is
 * the sequence that is already known correct; duplicating it in a resume
 * path would be a second copy to keep in step.
 *
 * key_done is cleared because the codec loses the 0x9901 unlock across a
 * power cycle. If it did not, re-issuing it is harmless -- sub_D2EFC is
 * idempotent in stock too, guarded by MEMORY[0x892A028].
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
