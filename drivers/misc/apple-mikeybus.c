// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple N31 MikeyBus.
 *
 * MikeyBus is not a UART. It is a byte FIFO inside the CS42L81 codec, driven
 * over the codec's SPI control port, and this driver is a client of that
 * codec rather than a serdev client of anything.
 *
 * The previous version of this file bound to UART2 at 0x3DC00000 and carried a
 * baud rate. That binding is closed negatively: 0x3DC00000 appears in RetailOS
 * only in the generic four-UART descriptor table at file 0x929C00 (IRQs 24..27)
 * and has no owner, no transfer and no baud setup anywhere in the image. The
 * only UART MMIO in the decompile is UART1 at 0x3DB00000, which is Bluetooth.
 *
 * Where the transport lives, all confirmed in raw disassembly:
 *
 *	0x051E..0x0528	the FIFO registers, on the codec's SPI port
 *	sub_15A50C  0x15A50C	send a frame
 *	sub_14DD16  0x14DD16	read-side level
 *	sub_15409C  0x15409C	read a frame
 *	sub_F141C   0xF141C	enable (0x0527 = 0x60) / disable (0xFF)
 *	sub_F1444   0xF1444	reset both FIFOs, bit 5 of 0x051E and 0x0523
 *
 * cs42l81-spi.c owns those registers and exports the five entry points this
 * file uses. Nothing here touches MMIO.
 *
 * Wire format
 * -----------
 * One frame, from sub_1757B0 (0x1757B0, the command serializer) and
 * sub_19A838 (0x19A838, the CRC finalizer):
 *
 *	[0]		0
 *	[1]		payload length
 *	[2]		0x00 or 0xFF, the dispatch selector
 *	[3]		flags: 0x40 on a command, low nibble 0 = packet,
 *			low nibble 2 = one byte in [4]
 *	[4]		sequence number
 *	[5 ..]		the payload
 *	[5+len]		CRC-16/ARC high byte, over [0 .. len+4]
 *	[6+len]		CRC low byte
 *
 * And the payload is self-describing, the same shape in both directions:
 *
 *	[0]		payload length again, 3 + parameter count
 *	[1]		command
 *	[2]		channel
 *	[3 ..]		parameters
 *
 * That last point corrects the reading this driver was written against, which
 * called payload[0] a "class". It is the length. Three commands agree:
 * sub_43B2D8 (0x43B2D8) channel enable is {4, 0x60, ch, on}; sub_410DB0
 * (0x410DB0) resistor query is {3, 0x8D, 3}; sub_570BA8 (0x570BA8) read-channel
 * open is {9, 0x71, 4, 0x83|..., 0x2A, 0x30, 0x1C, 0x80, 0x80}. In each the
 * first byte equals the payload length. The receive side reads it the same
 * way: sub_500E98 takes pkt[2] as the channel and drains pkt[0] - 3 payload
 * bytes from pkt[3].
 *
 * Channel dispatch (sub_500E98, 0x500E98, tbb table at 0x500EA2)
 * --------------------------------------------------------------
 *	3	sub_16FCC0	command 0x8E only: the accessory-reported model
 *	4	sub_500ECC	commands 0x70, 0x74, 0x76, 0x8A
 *	5	sub_15C1E8
 *	6	sub_167F54
 *	0x10	sub_15C218
 * Every other channel below 17 returns without doing anything, and 17 and
 * above are rejected before the table.
 *
 * What this driver does not know
 * ------------------------------
 * The remote-button wire encoding. Stock appends channel-4 command 0x70
 * payload bytes to a 1024-byte ring at 0x8AE5298 (head 0x8AE5294, masked to 10
 * bits) and drains it in sub_2542F0 (0x2542F0), which feeds bytes one at a time
 * to a parser and inserts an extra 0x01 after a raw 0xAA. That proves a
 * serialized, byte-stuffed protocol and nothing about button codes. The
 * parser's completed-frame callback was not recovered, so there is no decoder
 * here and no input device: an inferred keymap would be a guess wearing a
 * driver's clothes.
 *
 * To close it, capture the channel-4 stream for each action separately --
 * press, release, click, double click, volume up, volume down, hold -- and log
 * both ch4_raw and ch4_stuffed with timestamps. Framing should be inferred only
 * after repeated captures agree. The two rings and their sysfs files exist for
 * exactly that.
 *
 * Link state, and why the active commands default off
 * --------------------------------------------------
 * Stock will not serialize a command unless the MikeyBus link state at
 * 0x892A058 is at least 3 (sub_1757B0 at 0x1757BC), and sub_410DB0 refuses
 * with error 35 unless it is non-zero. That state is written only by
 * sub_42D364, driven from the Mikey task at 0x587E60 on RTOS events 74 and 76
 * -- and the hardware source of those events is a trace question, not a
 * software one: the software edge ends at sub_566E8C -> sub_42D5F4(76, ...)
 * and sub_5671E8 -> sub_42D5F4(74, ...), consecutive entries in the callback
 * table at runtime 0x08795EEC, with no GPIO or EIC line carried in.
 *
 * So there is no way to reproduce stock's gate. Sending is implemented and
 * left switched off; jack detection needs none of it, because that is a codec
 * register read.
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

/*
 * Provisional prototypes for the codec entry points this driver calls. Their
 * permanent home is <linux/apple-n31.h>, which no single driver owns; delete
 * this block when that declaration lands. The identical block is in
 * cs42l81-spi.c.
 */
int cs42l81_mbox_send(u8 *buf, size_t buf_size);
int cs42l81_mbox_level(void);
int cs42l81_mbox_read_frame(u8 *buf, size_t buf_size);
int cs42l81_mbox_enable(bool on);
int cs42l81_mbox_reset(void);
int cs42l81_headset_model(void);

/* Frame offsets, per sub_1757B0 and sub_19A838. */
#define MIKEY_FRM_LEN		1
#define MIKEY_FRM_SELECTOR	2
#define MIKEY_FRM_FLAGS		3
#define MIKEY_FRM_SEQ		4
#define MIKEY_FRM_PAYLOAD	5
#define MIKEY_FRM_OVERHEAD	7	/* header plus the two CRC bytes */

#define MIKEY_SELECTOR_PACKET	0xff	/* 0x587FEC: 0xFF dispatches sub_500E98 */
#define MIKEY_SELECTOR_ALT	0x00	/* 0x587FF8: 0x00 dispatches sub_4FF5E4 */

#define MIKEY_FLAGS_COMMAND	0x40	/* 0x1757EA */

/* Payload offsets, the same in both directions. */
#define MIKEY_PL_LEN		0
#define MIKEY_PL_CMD		1
#define MIKEY_PL_CHAN		2
#define MIKEY_PL_PARAM		3
#define MIKEY_PL_MIN		3	/* len, cmd, channel */

/* Channels, from sub_500E98's table. */
#define MIKEY_CH_MODEL		3
#define MIKEY_CH_REMOTE		4
#define MIKEY_CH_BULK		5
#define MIKEY_CH_SIX		6
#define MIKEY_CH_TEN		0x10
#define MIKEY_CH_MAX		0x10

/* Channel-4 commands, from sub_500ECC (0x500ECC). */
#define MIKEY_CMD_BYTES		0x70	/* payload bytes into the ring */
#define MIKEY_CMD_IGNORED	0x74	/* stock returns without acting */
#define MIKEY_CMD_STATUS_76	0x76	/* sub_18911C, and posts */
#define MIKEY_CMD_STATUS_8A	0x8a	/* sub_182AFC */

/* Channel-3 command, from sub_16FCC0 (0x16FCC0). */
#define MIKEY_CMD_MODEL		0x8e

/* Commands this driver can send. */
#define MIKEY_CMD_CHAN_ENABLE	0x60	/* sub_43B2D8 */
#define MIKEY_CMD_RESISTOR	0x8d	/* sub_410DB0 */
#define MIKEY_CMD_READ_OPEN	0x71	/* sub_570BA8 */

/*
 * sub_15409C's bounds: it will not start below a read level of 8, and refuses
 * a frame whose length byte plus three exceeds 152.
 */
#define MIKEY_FRAME_TAIL_MAX	152
#define MIKEY_FRAME_MAX		(MIKEY_FRM_PAYLOAD + MIKEY_FRAME_TAIL_MAX)

/* Stock's ring at 0x8AE5298: 1024 bytes, head masked to ten bits. */
#define MIKEY_CH4_RING		1024
/* The byte-stuffed replica of sub_2542F0, which can double every byte. */
#define MIKEY_CH4_STUFFED	2048
/* Stock's eight-entry single-byte ring at 0x8AE4F1A. */
#define MIKEY_BYTE_RING		8

/* Models sub_140EC8 returns that mean "nothing in the jack". */
#define MIKEY_MODEL_REMOVED	0
#define MIKEY_MODEL_OPEN	11

/* -------------------- module parameters -------------------- */

/*
 * 200 ms sits inside stock's own band: the headset task refreshes the jack
 * cache at its loop head and sleeps 100 to 300 ms a turn, with 15 ms settles
 * inside the detector. Note this shares SPI0 with the codec's audio traffic,
 * so it is not free.
 */
static int poll_ms = 200;
module_param(poll_ms, int, 0644);
MODULE_PARM_DESC(poll_ms, "receive and detect poll interval in milliseconds");

static bool auto_report = true;
module_param(auto_report, bool, 0644);
MODULE_PARM_DESC(auto_report, "log plug, unplug and model changes");

/*
 * Sending. Off by default because stock gates every command on a link state
 * this port cannot reproduce -- see the header. The frames themselves are
 * correct; what is missing is the condition under which stock believes the
 * far end is listening.
 */
static bool allow_tx;
module_param(allow_tx, bool, 0644);
MODULE_PARM_DESC(allow_tx,
		 "permit MikeyBus command frames (default N, no link-state gate exists)");

/*
 * FIFO enable. Writing 0x0527 = 0x60 is what sub_F141C does, and stock reaches
 * it from sub_570620 (0x570620) only when two conditions hold together:
 * MEMORY[0x8925CF4], the accessory port type, reads 1, and sub_410090()
 * returns non-zero (both via sub_41F944 at 0x41F944; either failure means
 * sub_42D364(0) and error 17). Neither condition's physical meaning is
 * determined, so the write is available and not automatic.
 *
 * sub_570620 also writes codec 0x054F under mask 0xF0, clears bits 1 and 2 of
 * 0x0401, writes 0x0404..0x0406, sets bit 1 of 0x0401, waits 100 ms and reads
 * 0x0528. That sequence is not reproduced here: it is not the FIFO, and it was
 * not traced far enough to know what it configures.
 */
static bool enable_fifo;
module_param(enable_fifo, bool, 0644);
MODULE_PARM_DESC(enable_fifo,
		 "write 0x0527 = 0x60 at probe (default N; stock also gates on 0x8925CF4 == 1)");

/* -------------------- state -------------------- */

struct mikey_ring {
	u8 *data;
	size_t size;
	size_t head;
	size_t tail;
	u32 drops;
};

struct apple_mikeybus {
	struct device *dev;
	struct mutex lock;
	struct delayed_work work;

	bool auto_report;

	/* From cs42l81_headset_model(); -1 until it has answered. */
	int model;
	int last_reported_model;

	/*
	 * The accessory's own report, channel 3 command 0x8E: four bits, per
	 * sub_16FCC0. Stock keeps it at 0x8A92444 and it is a different thing
	 * from the codec's model number.
	 */
	int accessory_sample;
	bool accessory_valid;

	/* Stock's 0x892A2C8, written by sub_18911C and sub_182AFC. */
	u16 status_word;

	/* Stock's channel-enable shadow at 0x8A9239C. */
	u32 channel_shadow;

	/* Stock's sequence counter at 0x892A09C. */
	u8 seq;

	struct mikey_ring ch4_raw;
	struct mikey_ring ch4_stuffed;
	u8 byte_ring[MIKEY_BYTE_RING];
	u8 byte_ring_head;
	u8 byte_ring_tail;

	u32 frames;
	u32 frame_errors;
	u32 packets;
	u32 single_bytes;
	u32 ch_unhandled[MIKEY_CH_MAX + 1];
	u32 ch4_bytes;
	u32 aa_stuffed;
	u32 status_events;
	u32 model_reports;
	u32 tx_frames;
	u32 tx_errors;
	u32 detect_errors;
};

static struct platform_device *mikey_pdev;

/* -------------------- model names -------------------- */

/*
 * The names sub_DCEC formats for the UI, against the model numbers
 * sub_140EC8 returns. 0 and 11 are the two that mean nothing is connected.
 */
static const char *mikey_model_name(int model)
{
	switch (model) {
	case 0:  return "removed";
	case 1:  return "A18";
	case 2:  return "B18";
	case 3:  return "A62";
	case 4:  return "B15";
	case 5:  return "A36";
	case 6:  return "Apple noise occluding";
	case 7:  return "mfg noise occluding";
	case 8:  return "mfg noise occluding w/ mic";
	case 9:  return "mfg std";
	case 10: return "mfg std w/ mic";
	case 11: return "open circuit";
	case 13: return "B60f";
	case 14: return "B60g";
	case 15: return "B149";
	case 16: return "B187";
	default: return "unnamed";
	}
}

/*
 * sub_40BE5C (0x40BE5C): the models stock treats as carrying a Mikey remote.
 * sub_150B2C is this set plus 3.
 */
static bool mikey_model_has_remote(int model)
{
	switch (model) {
	case 2: case 4: case 5: case 6: case 7:
	case 8: case 9: case 10: case 13: case 14: case 16:
		return true;
	default:
		return false;
	}
}

/* -------------------- exports for the codec side -------------------- */

/*
 * Presence, derived from the model exactly as stock derives it: 0 is removed,
 * 11 is an open circuit, anything else is something in the jack. A negative
 * model means the codec could not answer, which is not the same as unplugged
 * and is reported as an error rather than as 0.
 */
int apple_mikeybus_jack_present(void)
{
	int model = cs42l81_headset_model();

	if (model < 0)
		return model;
	return (model != MIKEY_MODEL_REMOVED && model != MIKEY_MODEL_OPEN);
}
EXPORT_SYMBOL_GPL(apple_mikeybus_jack_present);

int apple_mikeybus_headset_ready(void)
{
	return apple_mikeybus_jack_present();
}
EXPORT_SYMBOL_GPL(apple_mikeybus_headset_ready);

/* -------------------- rings -------------------- */

static void mikey_ring_put(struct mikey_ring *r, u8 b)
{
	size_t next;

	if (!r->data || !r->size)
		return;

	next = (r->head + 1) % r->size;
	if (next == r->tail) {
		r->drops++;
		r->tail = (r->tail + 1) % r->size;
	}
	r->data[r->head] = b;
	r->head = next;
}

static size_t mikey_ring_dump(struct mikey_ring *r, char *buf, size_t max)
{
	size_t n = 0, p;

	if (!r->data)
		return 0;

	for (p = r->tail; p != r->head && n + 4 < max; p = (p + 1) % r->size)
		n += scnprintf(buf + n, max - n, "%02x ", r->data[p]);

	if (n && n < max)
		buf[n - 1] = '\n';
	return n;
}

static int mikey_ring_alloc(struct apple_mikeybus *m, struct mikey_ring *r,
			    size_t size)
{
	r->data = devm_kzalloc(m->dev, size, GFP_KERNEL);
	if (!r->data)
		return -ENOMEM;
	r->size = size;
	return 0;
}

/* -------------------- receive -------------------- */

/*
 * Channel 4, command 0x70 (sub_500ECC at 0x500EE8).
 *
 * Stock appends the payload bytes to the ring at 0x8AE5298 and posts event 33.
 * The stuffed copy is sub_2542F0's (0x2542F0) view of the same stream: every
 * byte, plus an extra 0x01 after each raw 0xAA. Both are kept because the
 * encoding is unknown and a capture wants to see the escape as well as the
 * bytes that produced it.
 */
static void mikey_ch4_bytes_locked(struct apple_mikeybus *m,
				   const u8 *payload, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		u8 b = payload[i];

		mikey_ring_put(&m->ch4_raw, b);
		mikey_ring_put(&m->ch4_stuffed, b);
		if (b == 0xaa) {
			mikey_ring_put(&m->ch4_stuffed, 0x01);
			m->aa_stuffed++;
		}
		m->ch4_bytes++;
	}
}

/*
 * Channel 4, commands 0x76 and 0x8A (sub_18911C at 0x18911C, sub_182AFC at
 * 0x182AFC). Both set the same 16-bit word at 0x892A2C8 from payload[0]:
 * bit 4 stores 0, bit 5 stores 0x80, and neither bit means the word is left
 * alone. 0x76 additionally posts on 0x8AE5288; there is nothing waiting on
 * that here.
 */
static void mikey_ch4_status_locked(struct apple_mikeybus *m, u8 b)
{
	if (b & 0x10)
		m->status_word = 0;
	else if (b & 0x20)
		m->status_word = 0x80;
	else
		return;

	m->status_events++;
}

/* Channel 4 (sub_500ECC, 0x500ECC). */
static void mikey_ch4_locked(struct apple_mikeybus *m, const u8 *pl,
			     unsigned int len)
{
	unsigned int count;

	switch (pl[MIKEY_PL_CMD]) {
	case MIKEY_CMD_BYTES:
		if (pl[MIKEY_PL_LEN] < MIKEY_PL_MIN ||
		    pl[MIKEY_PL_LEN] > len)
			return;
		count = pl[MIKEY_PL_LEN] - MIKEY_PL_MIN;
		mikey_ch4_bytes_locked(m, &pl[MIKEY_PL_PARAM], count);
		break;

	case MIKEY_CMD_IGNORED:
		/* 0x500F34: stock returns without acting. */
		break;

	case MIKEY_CMD_STATUS_76:
	case MIKEY_CMD_STATUS_8A:
		if (len > MIKEY_PL_PARAM)
			mikey_ch4_status_locked(m, pl[MIKEY_PL_PARAM]);
		break;

	default:
		dev_dbg(m->dev, "ch4 cmd 0x%02x len %u unhandled\n",
			pl[MIKEY_PL_CMD], len);
		break;
	}
}

/*
 * Channel 3 (sub_16FCC0, 0x16FCC0).
 *
 * One command and one byte, with no candidate scan: command 0x8E, and the
 * accessory-reported model is payload[0] & 0x0F. Stock stores it at 0x8A92444
 * and signals 0x8A92448; sub_410DB0 then reads it back and, when it reads 15
 * with the modifier at 0x8A9244C zero, substitutes 100.
 */
static void mikey_ch3_locked(struct apple_mikeybus *m, const u8 *pl,
			     unsigned int len)
{
	if (pl[MIKEY_PL_CMD] != MIKEY_CMD_MODEL)
		return;
	if (len <= MIKEY_PL_PARAM)
		return;

	m->accessory_sample = pl[MIKEY_PL_PARAM] & 0x0f;
	m->accessory_valid = true;
	m->model_reports++;

	if (m->auto_report)
		dev_info(m->dev, "accessory reports sample %u\n",
			 m->accessory_sample);
}

/*
 * sub_500E98 (0x500E98). Channel is payload[2]; 17 and above are rejected
 * before the table, and channels 0, 1, 2 and 7..15 are in the table but
 * branch straight to a return.
 */
static void mikey_dispatch_locked(struct apple_mikeybus *m, const u8 *pl,
				  unsigned int len)
{
	u8 ch;

	if (len < MIKEY_PL_MIN)
		return;

	ch = pl[MIKEY_PL_CHAN];
	if (ch > MIKEY_CH_MAX) {
		dev_dbg(m->dev, "channel %u rejected\n", ch);
		return;
	}

	m->packets++;

	switch (ch) {
	case MIKEY_CH_MODEL:
		mikey_ch3_locked(m, pl, len);
		break;
	case MIKEY_CH_REMOTE:
		mikey_ch4_locked(m, pl, len);
		break;
	case MIKEY_CH_BULK:		/* sub_15C1E8 */
	case MIKEY_CH_SIX:		/* sub_167F54 */
	case MIKEY_CH_TEN:		/* sub_15C218 */
		m->ch_unhandled[ch]++;
		dev_dbg(m->dev, "channel %u cmd 0x%02x len %u not decoded\n",
			ch, pl[MIKEY_PL_CMD], len);
		break;
	default:
		/* In stock's table and deliberately inert. */
		m->ch_unhandled[ch]++;
		break;
	}
}

/*
 * One frame, as the stock receive task at 0x587F78 handles it.
 *
 * flags bit 7 clears the sequence counter (0x587FA0); the low nibble selects
 * between a packet (0) and a single byte in frame[4] (2); anything else is
 * dropped. A packet goes to sub_500E98 when the selector is 0xFF and to
 * sub_4FF5E4 when it is 0x00. sub_4FF5E4 is not decoded here.
 */
static void mikey_frame_locked(struct apple_mikeybus *m, const u8 *frm,
			       unsigned int len)
{
	u8 flags, low, next;

	if (len < MIKEY_FRM_PAYLOAD)
		return;

	m->frames++;
	flags = frm[MIKEY_FRM_FLAGS];

	if (flags & 0x80)
		m->seq = 0;

	low = flags & 0x0f;
	if (low == 2) {
		/* 0x587FB4: one byte into the eight-entry ring. */
		next = (m->byte_ring_head + 1) & (MIKEY_BYTE_RING - 1);
		if (next != m->byte_ring_tail) {
			m->byte_ring[m->byte_ring_head] = frm[MIKEY_FRM_SEQ];
			m->byte_ring_head = next;
		}
		m->single_bytes++;
		return;
	}
	if (low != 0)
		return;

	if (frm[MIKEY_FRM_SELECTOR] == MIKEY_SELECTOR_PACKET) {
		mikey_dispatch_locked(m, &frm[MIKEY_FRM_PAYLOAD],
				      len - MIKEY_FRM_PAYLOAD);
	} else if (frm[MIKEY_FRM_SELECTOR] == MIKEY_SELECTOR_ALT) {
		dev_dbg(m->dev, "selector 0x00 packet, sub_4FF5E4, not decoded\n");
	}
}

/* -------------------- transmit -------------------- */

/*
 * Build and send one command frame.
 *
 * @cmd and @chan go into the payload; @params is appended after them. The
 * payload length byte is 3 + @nparams, which is what stock writes and what the
 * receive side reads back. The outer header and the CRC are the codec's
 * cs42l81_mbox_send(), which frames per sub_19A838.
 */
static int mikey_send_locked(struct apple_mikeybus *m, u8 cmd, u8 chan,
			     const u8 *params, unsigned int nparams)
{
	u8 frame[MIKEY_FRAME_MAX + MIKEY_FRM_OVERHEAD];
	unsigned int pl_len = MIKEY_PL_MIN + nparams;
	int ret;

	if (!allow_tx)
		return -EPERM;
	if (MIKEY_FRM_PAYLOAD + pl_len + 2 > sizeof(frame))
		return -EINVAL;

	memset(frame, 0, sizeof(frame));
	frame[0] = 0;
	frame[MIKEY_FRM_LEN] = pl_len;
	frame[MIKEY_FRM_SELECTOR] = MIKEY_SELECTOR_PACKET;
	frame[MIKEY_FRM_FLAGS] = MIKEY_FLAGS_COMMAND;
	frame[MIKEY_FRM_SEQ] = m->seq++;
	frame[MIKEY_FRM_PAYLOAD + MIKEY_PL_LEN] = pl_len;
	frame[MIKEY_FRM_PAYLOAD + MIKEY_PL_CMD] = cmd;
	frame[MIKEY_FRM_PAYLOAD + MIKEY_PL_CHAN] = chan;
	if (nparams)
		memcpy(&frame[MIKEY_FRM_PAYLOAD + MIKEY_PL_PARAM], params,
		       nparams);

	ret = cs42l81_mbox_send(frame, sizeof(frame));
	if (ret) {
		m->tx_errors++;
		return ret;
	}
	m->tx_frames++;
	return 0;
}

/* sub_43B2D8 (0x43B2D8): {4, 0x60, ch, on}, with a shadow of the mask. */
static int mikey_channel_enable_locked(struct apple_mikeybus *m, u8 ch, bool on)
{
	u8 param = on ? 1 : 0;
	bool cur = !!(m->channel_shadow & BIT(ch));
	int ret;

	if (ch > 31)
		return -EINVAL;
	if (cur == on)
		return 0;

	ret = mikey_send_locked(m, MIKEY_CMD_CHAN_ENABLE, ch, &param, 1);
	if (ret)
		return ret;

	if (on)
		m->channel_shadow |= BIT(ch);
	else
		m->channel_shadow &= ~BIT(ch);
	return 0;
}

/*
 * sub_410DB0 (0x410DB0): enable channel 3, then {3, 0x8D, 3}. Stock waits
 * 100 ms on 0x8A92448 for the reply; here the reply arrives through the poll
 * worker as a channel-3 command 0x8E and lands in accessory_sample.
 */
static int mikey_query_resistor_locked(struct apple_mikeybus *m)
{
	int ret = mikey_channel_enable_locked(m, MIKEY_CH_MODEL, true);

	if (ret)
		return ret;
	return mikey_send_locked(m, MIKEY_CMD_RESISTOR, MIKEY_CH_MODEL,
				 NULL, 0);
}

/*
 * sub_570BA8 (0x570BA8): enable channel 4, then {9, 0x71, 4, p0, 0x2A, 0x30,
 * 0x1C, 0x80, 0x80} where p0 = 0x83 | (a << 4) | (b ? 0 : 4). Stock then
 * clears the ring head and tail and the status word, which is done here too.
 */
static int mikey_open_read_channel_locked(struct apple_mikeybus *m, u8 a, bool b)
{
	u8 params[6] = { 0x83, 0x2a, 0x30, 0x1c, 0x80, 0x80 };
	int ret;

	params[0] = 0x83 | (u8)(a << 4) | (b ? 0x00 : 0x04);

	ret = mikey_channel_enable_locked(m, MIKEY_CH_REMOTE, true);
	if (ret)
		return ret;
	ret = mikey_send_locked(m, MIKEY_CMD_READ_OPEN, MIKEY_CH_REMOTE,
				params, sizeof(params));
	if (ret)
		return ret;

	m->ch4_raw.head = 0;
	m->ch4_raw.tail = 0;
	m->status_word = 0;
	return 0;
}

/* -------------------- poll worker -------------------- */

static void mikey_report_locked(struct apple_mikeybus *m)
{
	if (!m->auto_report || m->model == m->last_reported_model)
		return;

	/*
	 * A negative model is dev_dbg, not dev_info: it can alternate with a
	 * real answer -- the 0x0078 bit-0 read refuses with stock's error 71
	 * whenever 0x0074 bits [2:1] happen to read 1 -- and at this poll rate
	 * that would be a log flood rather than a jack event.
	 */
	if (m->model < 0)
		dev_dbg(m->dev, "headset state unknown (%d)\n", m->model);
	else if (m->model == MIKEY_MODEL_REMOVED)
		dev_info(m->dev, "headset removed\n");
	else if (m->model == MIKEY_MODEL_OPEN)
		dev_info(m->dev, "jack open circuit\n");
	else
		dev_info(m->dev, "headset present: model %d (%s), remote %s\n",
			 m->model, mikey_model_name(m->model),
			 mikey_model_has_remote(m->model) ? "yes" : "no");

	m->last_reported_model = m->model;
}

static void mikey_work_fn(struct work_struct *work)
{
	struct apple_mikeybus *m = container_of(to_delayed_work(work),
					        struct apple_mikeybus, work);
	u8 frame[MIKEY_FRAME_MAX];
	unsigned int guard;
	int model, ret;

	/*
	 * The detect call takes the codec's own mutex, so it is made outside
	 * this driver's lock.
	 */
	model = cs42l81_headset_model();

	mutex_lock(&m->lock);
	if (model < 0 && m->model >= 0)
		m->detect_errors++;
	m->model = model;
	mikey_report_locked(m);

	/*
	 * Drain what the FIFO has. Bounded, because stock's equivalent loop at
	 * 0x587F86 is driven by an event and re-tests the level rather than
	 * running on a timer.
	 */
	for (guard = 0; guard < 16; guard++) {
		ret = cs42l81_mbox_read_frame(frame, sizeof(frame));
		if (ret <= 0) {
			if (ret < 0 && ret != -ENODEV)
				m->frame_errors++;
			break;
		}
		mikey_frame_locked(m, frame, ret);
	}
	mutex_unlock(&m->lock);

	schedule_delayed_work(&m->work,
			      msecs_to_jiffies(clamp(poll_ms, 50, 10000)));
}

/* -------------------- sysfs -------------------- */

static ssize_t model_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	int model;

	mutex_lock(&m->lock);
	model = m->model;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "%d\n", model);
}
static DEVICE_ATTR_RO(model);

static ssize_t model_name_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	int model;

	mutex_lock(&m->lock);
	model = m->model;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "%s\n",
			  model < 0 ? "unknown" : mikey_model_name(model));
}
static DEVICE_ATTR_RO(model_name);

static ssize_t plugged_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int ret = apple_mikeybus_jack_present();

	if (ret < 0)
		return sysfs_emit(buf, "unknown\n");
	return sysfs_emit(buf, "%d\n", ret);
}
static DEVICE_ATTR_RO(plugged);

static ssize_t accessory_sample_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&m->lock);
	n = m->accessory_valid ? sysfs_emit(buf, "%d\n", m->accessory_sample)
			       : sysfs_emit(buf, "none\n");
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(accessory_sample);

static ssize_t mbox_level_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", cs42l81_mbox_level());
}
static DEVICE_ATTR_RO(mbox_level);

static ssize_t ch4_raw_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	size_t n;

	mutex_lock(&m->lock);
	n = mikey_ring_dump(&m->ch4_raw, buf, PAGE_SIZE);
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(ch4_raw);

static ssize_t ch4_stuffed_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	size_t n;

	mutex_lock(&m->lock);
	n = mikey_ring_dump(&m->ch4_stuffed, buf, PAGE_SIZE);
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(ch4_stuffed);

static ssize_t stats_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&m->lock);
	n = sysfs_emit(buf,
		       "frames=%u frame_errors=%u packets=%u single_bytes=%u\n"
		       "ch4_bytes=%u aa_stuffed=%u ch4_drops=%u status_events=%u\n"
		       "status_word=0x%04x model_reports=%u channel_shadow=0x%08x\n"
		       "tx_frames=%u tx_errors=%u detect_errors=%u seq=%u\n"
		       "unhandled: ch5=%u ch6=%u ch16=%u\n",
		       m->frames, m->frame_errors, m->packets, m->single_bytes,
		       m->ch4_bytes, m->aa_stuffed, m->ch4_raw.drops,
		       m->status_events, m->status_word, m->model_reports,
		       m->channel_shadow, m->tx_frames, m->tx_errors,
		       m->detect_errors, m->seq,
		       m->ch_unhandled[MIKEY_CH_BULK],
		       m->ch_unhandled[MIKEY_CH_SIX],
		       m->ch_unhandled[MIKEY_CH_TEN]);
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(stats);

/* Write "1" to send stock's resistor query, {3, 0x8D, 3}. */
static ssize_t query_resistor_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&m->lock);
	ret = mikey_query_resistor_locked(m);
	mutex_unlock(&m->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(query_resistor);

/*
 * Write "<a> <b>" to send stock's read-channel open, sub_570BA8's
 * {9, 0x71, 4, 0x83|(a<<4)|(b?0:4), 0x2A, 0x30, 0x1C, 0x80, 0x80}. This is
 * what starts the channel-4 byte stream a button capture needs.
 */
static ssize_t open_read_channel_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int a = 0, b = 0;
	int ret;

	if (sscanf(buf, "%u %u", &a, &b) < 1)
		return -EINVAL;
	if (a > 15)
		return -EINVAL;

	mutex_lock(&m->lock);
	ret = mikey_open_read_channel_locked(m, (u8)a, b != 0);
	mutex_unlock(&m->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(open_read_channel);

/* Write "<ch> <0|1>" for stock's channel enable, {4, 0x60, ch, on}. */
static ssize_t channel_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int ch, on;
	int ret;

	if (sscanf(buf, "%u %u", &ch, &on) != 2)
		return -EINVAL;
	if (ch > 31)
		return -EINVAL;

	mutex_lock(&m->lock);
	ret = mikey_channel_enable_locked(m, (u8)ch, on != 0);
	mutex_unlock(&m->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(channel_enable);

/* Write "1" for sub_F1444's reset of both FIFOs. */
static ssize_t fifo_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = cs42l81_mbox_reset();

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(fifo_reset);

/* Write "0" or "1" for sub_F141C's 0x0527 = 0xFF / 0x60. */
static ssize_t fifo_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	bool on;
	int ret;

	if (kstrtobool(buf, &on))
		return -EINVAL;
	ret = cs42l81_mbox_enable(on);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(fifo_enable);

/*
 * Feed a frame in as if the FIFO had produced it, for testing the dispatch
 * without hardware. Hex bytes, whitespace separated, starting at frame[0].
 */
static ssize_t frame_inject_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 frame[MIKEY_FRAME_MAX];
	unsigned int n = 0;
	const char *p = buf;
	const char *end = buf + count;

	while (p < end && n < sizeof(frame)) {
		unsigned int v = 0, digits = 0;
		int d;

		while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' ||
				   *p == '\r' || *p == ','))
			p++;
		while (p < end && digits < 2) {
			d = hex_to_bin(*p);
			if (d < 0)
				break;
			v = (v << 4) | (unsigned int)d;
			digits++;
			p++;
		}
		if (!digits)
			break;
		frame[n++] = (u8)v;
	}
	if (n < MIKEY_FRM_PAYLOAD)
		return -EINVAL;

	mutex_lock(&m->lock);
	mikey_frame_locked(m, frame, n);
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_WO(frame_inject);

static struct attribute *mikey_attrs[] = {
	&dev_attr_model.attr,
	&dev_attr_model_name.attr,
	&dev_attr_plugged.attr,
	&dev_attr_accessory_sample.attr,
	&dev_attr_mbox_level.attr,
	&dev_attr_ch4_raw.attr,
	&dev_attr_ch4_stuffed.attr,
	&dev_attr_stats.attr,
	&dev_attr_query_resistor.attr,
	&dev_attr_open_read_channel.attr,
	&dev_attr_channel_enable.attr,
	&dev_attr_fifo_reset.attr,
	&dev_attr_fifo_enable.attr,
	&dev_attr_frame_inject.attr,
	NULL,
};

static const struct attribute_group mikey_attr_group = {
	.attrs = mikey_attrs,
};

/* -------------------- probe / remove -------------------- */

static int mikey_probe(struct platform_device *pdev)
{
	struct apple_mikeybus *m;
	int ret;

	m = devm_kzalloc(&pdev->dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->dev = &pdev->dev;
	m->auto_report = auto_report;
	m->model = -1;
	m->last_reported_model = -2;	/* so the first answer always logs */
	m->accessory_sample = -1;
	mutex_init(&m->lock);
	INIT_DELAYED_WORK(&m->work, mikey_work_fn);

	ret = mikey_ring_alloc(m, &m->ch4_raw, MIKEY_CH4_RING);
	if (ret)
		return ret;
	ret = mikey_ring_alloc(m, &m->ch4_stuffed, MIKEY_CH4_STUFFED);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, m);

	ret = sysfs_create_group(&pdev->dev.kobj, &mikey_attr_group);
	if (ret)
		return ret;

	if (enable_fifo) {
		ret = cs42l81_mbox_enable(true);
		if (ret)
			dev_warn(m->dev, "0x0527 enable failed: %d\n", ret);
		else
			cs42l81_mbox_reset();
	}

	dev_info(m->dev,
		 "MikeyBus on the CS42L81 FIFO: poll=%dms tx=%d fifo_enable=%d\n",
		 poll_ms, allow_tx, enable_fifo);

	schedule_delayed_work(&m->work, msecs_to_jiffies(100));
	return 0;
}

static void mikey_remove(struct platform_device *pdev)
{
	struct apple_mikeybus *m = platform_get_drvdata(pdev);

	if (!m)
		return;

	cancel_delayed_work_sync(&m->work);
	sysfs_remove_group(&pdev->dev.kobj, &mikey_attr_group);
}

static struct platform_driver mikey_driver = {
	.probe = mikey_probe,
	.remove = mikey_remove,
	.driver = {
		.name = "apple-mikeybus",
	},
};

/*
 * No device tree node.
 *
 * The old one was a child of serial@3dc00000, which is not MikeyBus. There is
 * nothing left for a node to describe: the registers belong to the codec and
 * the codec is found through its exports, so the device is created here.
 */
static int __init mikey_init(void)
{
	int ret = platform_driver_register(&mikey_driver);

	if (ret)
		return ret;

	mikey_pdev = platform_device_register_simple("apple-mikeybus",
						     PLATFORM_DEVID_NONE,
						     NULL, 0);
	if (IS_ERR(mikey_pdev)) {
		ret = PTR_ERR(mikey_pdev);
		mikey_pdev = NULL;
		platform_driver_unregister(&mikey_driver);
		return ret;
	}
	return 0;
}

static void __exit mikey_exit(void)
{
	if (mikey_pdev) {
		platform_device_unregister(mikey_pdev);
		mikey_pdev = NULL;
	}
	platform_driver_unregister(&mikey_driver);
}

module_init(mikey_init);
module_exit(mikey_exit);

MODULE_DESCRIPTION("Apple N31 MikeyBus, on the CS42L81 message FIFO");
MODULE_AUTHOR("FreeMyiPod");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:apple-mikeybus");
