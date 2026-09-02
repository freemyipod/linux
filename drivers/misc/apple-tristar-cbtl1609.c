// SPDX-License-Identifier: GPL-2.0+
/*
 * Apple Lightning Tristar mux — NXP CBTL1609A1 (iPod nano 7G / N31)
 *
 * Transport (proven on glass 2026-08-27): I2C1 7-bit 0x1a
 * (controller 3c900000). I2C0/IIC0 does not complete emcore-TX here —
 * do not bind Tristar there. Public 0x34 write / 0x35 read is the
 * 8-bit form of that address (nyansatan).
 *
 * Routing is IDBUS inside the chip, not Linux Dx register writes.
 * RetailOS N31 RE observed zero Dx/mux I2C writes. The 0x75 accessory
 * ID byte programs ACCx/Dx per the THS7383 tables (nyansatan; first-gen
 * CBTL1608 is documented as backwards compatible with those tables).
 * CBTL1609 is the nano7 first-gen part — same IDBUS decode, no invented
 * I2C mux map.
 *
 * N31 analog 3.5 mm jack is CS42 + MikeyBus UART2 (accessoryMgr type 1,
 * sub_35A4). It is not a Tristar Dx path. Lightning analog EarPods use
 * ID 04 F1 00 00 00 00 (nyansatan) — a different connector. CS42 still
 * refreshes Tristar on prepare so we log whether Lightning is USB,
 * analog, idle, or I2C-echo; we do not mute the jack because USB is
 * routed.
 *
 * OSOS software lane (sub_11C8C): TriStarID/VBUS/CONDET tasks wait on
 * event source 13 and branch on raw bits 0x01/0x04/0x08/0x10/0x20.
 * Those bits are not mapped to I2C registers (TODO RE). Linux keeps
 * osos_event=0 and reports hardware observations separately. Do not
 * treat dump-not-flat as OSOS bit 0x04.
 *
 * Register 0x11 is CBTL1610 "configuration status" (Lina/nyansatan).
 * First-gen may NAK it or the bus may echo 0x35 — the read result is
 * reported, never invented.
 *
 * accessoryMgr type 2 is TODO RE — logged, not stub-handled.
 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#define TRISTAR_DUMP_LEN	0x40
#define TRISTAR_LOG_LEN		64

/* OSOS sub_11C8C masks — names only. Not produced from I2C until RE maps them. */
#define N31_TS_EVENT_01		BIT(0)
#define N31_TS_EVENT_04		BIT(2)
#define N31_TS_EVENT_08		BIT(3)
#define N31_TS_EVENT_10		BIT(4)
#define N31_TS_EVENT_20		BIT(5)

/* nyansatan 0x75 first byte: ACCx[7:6] Dx[5:4] DATA[3:0] */
#define TS_ID_ACCX(id0)		(((id0) >> 6) & 3)
#define TS_ID_DX(id0)		(((id0) >> 4) & 3)

/*
 * Control/status register map for this accessory switch.
 *
 * This part is the Lightning-side analogue mux: two "Dx" data lanes and two
 * accessory lanes, each routed by a small field, plus an ID bus with a byte
 * FIFO the accessory answers on. Until now this driver had no register
 * semantics whatsoever -- it dumped 0x00..0x3F and pattern-matched the
 * result -- which is why its audio-path line could only ever report
 * "unmapped".
 *
 * Names here are ours. The field layout is what the hardware does.
 */
#define TS_REG_DX_SWITCH	0x01	/* both Dx lanes */
#define TS_REG_ACC_SWITCH	0x02	/* both accessory lanes */
#define TS_REG_CHARGE_DET	0x03	/* charger-detect source/sink */
#define TS_REG_IDBUS_CTL	0x05
#define TS_REG_DIG_ID		0x06	/* raw ID-pin levels */
#define TS_REG_STATUS		0x0c
#define TS_REG_FIFO_TOP		0x3f	/* ID FIFO; payload reads downward */

/* TS_REG_DX_SWITCH: lane 1 in bits 2:0, lane 2 in bits 6:4, override above each */
#define TS_DX1_SEL		GENMASK(2, 0)
#define TS_DX1_OVERRIDE		BIT(3)
#define TS_DX2_SEL		GENMASK(6, 4)
#define TS_DX2_OVERRIDE		BIT(7)
#define TS_DX_SEL_OPEN		0
#define TS_DX_SEL_USB		1
#define TS_DX_SEL_UART0		2
#define TS_DX_SEL_DIGITAL_ID	3
#define TS_DX_SEL_BRICK_ID_P	4
#define TS_DX_SEL_BRICK_ID_N	5
/* 6 and 7 differ per lane: lane 1 gives a second USB and JTAG data, */
/* lane 2 gives two further UARTs. */
#define TS_DX1_SEL_USB_ALT	6
#define TS_DX1_SEL_JTAG_DIO	7
#define TS_DX2_SEL_UART2	6
#define TS_DX2_SEL_UART1	7

/* TS_REG_ACC_SWITCH: lane 1 in bits 1:0, lane 2 in bits 4:3 */
#define TS_ACC1_SEL		GENMASK(1, 0)
#define TS_ACC1_OVERRIDE	BIT(2)
#define TS_ACC2_SEL		GENMASK(4, 3)
#define TS_ACC2_OVERRIDE	BIT(5)
#define TS_ACC_SEL_OPEN		0
#define TS_ACC_SEL_UART1	1	/* rx on lane 1, tx on lane 2 */
#define TS_ACC_SEL_JTAG		2	/* data on lane 1, clock on lane 2 */
#define TS_ACC_SEL_ACC_POWER	3

/* TS_REG_CHARGE_DET */
#define TS_CHG_ID_SINK_EN	BIT(3)
#define TS_CHG_SRC_SEL		GENMASK(2, 0)
#define TS_CHG_SRC_OFF		0
#define TS_CHG_SRC_DP1		1
#define TS_CHG_SRC_DN1		2
#define TS_CHG_SRC_DP2		3
#define TS_CHG_SRC_DN2		4

/* TS_REG_IDBUS_CTL */
#define TS_IDBUS_RESET		BIT(3)
#define TS_IDBUS_BREAK		BIT(2)
#define TS_IDBUS_REORIENT	BIT(1)
#define TS_IDBUS_SINK_EN	BIT(0)

/* TS_REG_DIG_ID: raw levels on the two Dx and two accessory ID pins */
#define TS_DIG_ID_DX1		BIT(3)
#define TS_DIG_ID_DX0		BIT(2)
#define TS_DIG_ID_ACC1		BIT(1)
#define TS_DIG_ID_ACC0		BIT(0)

/* TS_REG_STATUS */
#define TS_ST_IDBUS_CONNECTED	BIT(7)
#define TS_ST_IDBUS_ORIENT	BIT(6)
#define TS_ST_SWITCH_EN		BIT(5)
#define TS_ST_HOST_RESET	BIT(4)
#define TS_ST_OVP		BIT(3)
#define TS_ST_CON_DET_N		BIT(2)	/* active low: set means NOT detected */

/* The accessory answers with this marker before the ID payload. */
#define TS_ID_FIFO_MARKER	0x75
#define TS_ID_PAYLOAD_LEN	6

struct tristar_id_sig {
	u8 bytes[6];
	const char *name;
};

/* nyansatan Lightning ID table (HOSTID=1). */
static const struct tristar_id_sig tristar_known_ids[] = {
	{ { 0x10, 0x0c, 0x00, 0x00, 0x00, 0x00 }, "usb-cable" },
	{ { 0x04, 0xf1, 0x00, 0x00, 0x00, 0x00 }, "lightning-analog" },
	{ { 0x0b, 0xf0, 0x00, 0x00, 0x00, 0x00 }, "haywire-hdmi" },
	{ { 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 }, "dcsd-or-uart-charge" },
	{ { 0x20, 0x02, 0x00, 0x00, 0x00, 0x00 }, "kong-swd-idle" },
	{ { 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00 }, "kong-swd-astris" },
	{ { 0x20, 0x0e, 0x00, 0x00, 0x00, 0x00 }, "kanzi-swd-idle" },
	{ { 0xa0, 0x0c, 0x00, 0x00, 0x00, 0x00 }, "kanzi-swd-astris" },
	{ { 0x20, 0x00, 0x10, 0x00, 0x00, 0x00 }, "uart-charge" },
};

struct apple_tristar {
	struct i2c_client *client;
	struct mutex lock;
	struct delayed_work poll;
	struct dentry *debug_root;
	u8 last_dump[TRISTAR_DUMP_LEN];
	u8 read_reg;
	u8 read_val;
	bool dump_ok;
	bool dump_flat;
	bool i2c_echo;
	int reg11_ret;
	u8 reg11;
	bool id_valid;
	u8 id75[6];
	unsigned int id_off;
	const char *id_name;
	u8 accx;
	u8 dx;
	/* OSOS v36 — stays 0. Not synthesized from I2C. */
	u8 osos_event;
	u8 prev_osos_event;
	bool cf9_latch;
	u8 cfa_state;
	u32 seen_mask;
	u32 polls;
	u32 deltas;
	u32 writes;
	u32 i2c_fail_streak;
	bool poll_disabled;
	char log[TRISTAR_LOG_LEN][112];
	unsigned int log_head;
	unsigned int log_count;
};

static struct apple_tristar *tristar_singleton;
static DEFINE_MUTEX(tristar_singleton_lock);

int apple_tristar_refresh(void);
int apple_tristar_connected(void);
int apple_tristar_usb_routed(void);
int apple_tristar_lightning_analog(void);
int apple_tristar_vbus(void);
int apple_tristar_config_reg11(u8 *val);
void apple_tristar_log_audio_path(struct device *audio_dev);

static bool read_only = true;
module_param(read_only, bool, 0644);
MODULE_PARM_DESC(read_only,
		 "Refuse I2C writes (default 1). IDBUS routing does not need them.");

static bool unsafe_acks;
module_param(unsafe_acks, bool, 0600);
MODULE_PARM_DESC(unsafe_acks,
		 "Unused: OSOS event ack path not recovered. Default 0.");

static bool unsafe_writes;
module_param(unsafe_writes, bool, 0600);
MODULE_PARM_DESC(unsafe_writes,
		 "Allow poke / DT init-sequence writes. Default 0.");

/*
 * Glass: I2C0 often -110 / echo 0x35. Polling reg0 every 250ms floods dmesg
 * and burns the bus. Default off; enable only when I2C0 ACKs for real.
 */
static int poll_ms;
module_param(poll_ms, int, 0644);
MODULE_PARM_DESC(poll_ms,
		 "Status poll interval ms (0=off). Default 0 — I2C0 often times out.");

static int tristar_read_reg(struct apple_tristar *ts, u8 reg, u8 *val)
{
	int ret = i2c_smbus_read_byte_data(ts->client, reg);

	if (ret < 0)
		return ret;
	*val = (u8)ret;
	return 0;
}

static int tristar_write_reg(struct apple_tristar *ts, u8 reg, u8 val)
{
	int ret;

	if (read_only && !unsafe_writes)
		return -EPERM;
	ret = i2c_smbus_write_byte_data(ts->client, reg, val);
	if (!ret)
		ts->writes++;
	return ret;
}

static bool tristar_dump_is_flat(const u8 *dump, size_t len)
{
	size_t i;

	for (i = 1; i < len; i++) {
		if (dump[i] != dump[0])
			return false;
	}
	return true;
}

static u8 tristar_read_addr_echo(struct apple_tristar *ts)
{
	return (u8)((ts->client->addr << 1) | 1);
}

static const char *tristar_dx_usb_id0(u8 dx)
{
	switch (dx) {
	case 0:
		return "hiz";
	case 1:
		return "usb0-on-dp1dn1";
	case 2:
		return "usb0-on-dp1dn1+uart-on-dp2dn2";
	default:
		return "hiz";
	}
}

static const char *tristar_dx_usb_id1(u8 dx)
{
	switch (dx) {
	case 0:
		return "hiz";
	case 1:
		return "usb0-on-dp2dn2";
	case 2:
		return "usb0-on-dp1dn1+uart-on-dp2dn2";
	default:
		return "hiz";
	}
}

static const char *tristar_accx_name(u8 accx)
{
	switch (accx) {
	case 0:
		return "hiz-idbus";
	case 1:
		return "uart1";
	case 2:
		return "jtag-swd";
	default:
		return "host-reset";
	}
}

static bool tristar_usb_dp_from_dx(u8 dx)
{
	return dx == 1 || dx == 2;
}

static bool tristar_is_lightning_analog(struct apple_tristar *ts)
{
	return ts->id_valid && ts->id_name &&
	       !strcmp(ts->id_name, "lightning-analog");
}

static const char *tristar_id_label(struct apple_tristar *ts)
{
	if (ts->i2c_echo)
		return "i2c-echo";
	if (ts->id_name)
		return ts->id_name;
	if (!ts->dump_ok)
		return "unread";
	if (ts->dump_flat)
		return "idle";
	return "unknown";
}

static void tristar_log_line(struct apple_tristar *ts, const char *why)
{
	unsigned int i = ts->log_head;

	snprintf(ts->log[i], sizeof(ts->log[i]),
		 "%s osos=0x%02x (unmapped) flat=%d echo=%d id=%s accx=%u dx=%u r11=%s%02x",
		 why, ts->osos_event, ts->dump_flat, ts->i2c_echo,
		 tristar_id_label(ts), ts->accx, ts->dx,
		 ts->reg11_ret ? "ERR" : "",
		 ts->reg11_ret ? 0 : ts->reg11);
	ts->log_head = (i + 1) % TRISTAR_LOG_LEN;
	if (ts->log_count < TRISTAR_LOG_LEN)
		ts->log_count++;
}

static void tristar_clear_id(struct apple_tristar *ts)
{
	ts->id_valid = false;
	ts->id_name = NULL;
	ts->id_off = 0;
	memset(ts->id75, 0, sizeof(ts->id75));
	ts->accx = 0;
	ts->dx = 0;
}

static void tristar_find_id(struct apple_tristar *ts)
{
	unsigned int s, off;
	const struct tristar_id_sig *sig;

	tristar_clear_id(ts);

	if (!ts->dump_ok || ts->dump_flat || ts->i2c_echo)
		return;

	for (s = 0; s < ARRAY_SIZE(tristar_known_ids); s++) {
		sig = &tristar_known_ids[s];
		for (off = 0; off + 6 <= TRISTAR_DUMP_LEN; off++) {
			if (memcmp(ts->last_dump + off, sig->bytes, 6))
				continue;
			memcpy(ts->id75, sig->bytes, 6);
			ts->id_name = sig->name;
			ts->id_valid = true;
			ts->id_off = off;
			ts->accx = TS_ID_ACCX(sig->bytes[0]);
			ts->dx = TS_ID_DX(sig->bytes[0]);
			dev_info(&ts->client->dev,
				 "tristar: IDBUS id %s @dump+0x%x accx=%u(%s) dx=%u id0=%s id1=%s\n",
				 sig->name, off, ts->accx,
				 tristar_accx_name(ts->accx), ts->dx,
				 tristar_dx_usb_id0(ts->dx),
				 tristar_dx_usb_id1(ts->dx));
			return;
		}
	}
}

static int tristar_refresh_locked(struct apple_tristar *ts, const char *why)
{
	int i, ret;
	u8 v;
	u8 prior[TRISTAR_DUMP_LEN];
	unsigned int n = 0;
	u8 echo = tristar_read_addr_echo(ts);
	bool echo_now;
	const char *prev_id;

	memcpy(prior, ts->last_dump, sizeof(prior));
	prev_id = tristar_id_label(ts);

	for (i = 0; i < TRISTAR_DUMP_LEN; i++) {
		ret = tristar_read_reg(ts, (u8)i, &v);
		if (ret) {
			ts->dump_ok = false;
			ts->i2c_echo = false;
			tristar_clear_id(ts);
			ts->reg11_ret = ret;
			ts->i2c_fail_streak++;
			/*
			 * One line per outage, not one per poll. I2C0 -110 is
			 * common until the mux/bus is actually live. Kill poll
			 * immediately so a leftover poll_ms=250 never storms.
			 */
			ts->poll_disabled = true;
			if (ts->i2c_fail_streak == 1)
				dev_warn(&ts->client->dev,
					 "tristar: I2C read 0x%02x failed %d (poll off until bus recovers)\n",
					 i, ret);
			return ret;
		}
		ts->i2c_fail_streak = 0;
		ts->last_dump[i] = v;
		/*
		 * I2C0 often echoes the 8-bit read address (0x35). Four
		 * identical echo bytes means the dump is not chip SRAM —
		 * stop before a 64-register storm.
		 */
		if (i == 3 && ts->last_dump[0] == echo &&
		    ts->last_dump[1] == echo &&
		    ts->last_dump[2] == echo &&
		    ts->last_dump[3] == echo) {
			memset(ts->last_dump, echo, TRISTAR_DUMP_LEN);
			break;
		}
	}
	ts->dump_ok = true;
	echo_now = tristar_dump_is_flat(ts->last_dump, TRISTAR_DUMP_LEN) &&
		   ts->last_dump[0] == echo;
	ts->i2c_echo = echo_now;
	ts->dump_flat = tristar_dump_is_flat(ts->last_dump, TRISTAR_DUMP_LEN);

	if (echo_now) {
		ts->reg11_ret = -ENOTSUPP;
		ts->reg11 = echo;
		tristar_clear_id(ts);
	} else {
		ts->reg11 = ts->last_dump[0x11];
		ts->reg11_ret = 0;
		tristar_find_id(ts);
	}

	ts->polls++;
	for (i = 0; i < TRISTAR_DUMP_LEN; i++) {
		if (prior[i] != ts->last_dump[i])
			n++;
	}
	ts->deltas += n;

	if (n || strcmp(prev_id, tristar_id_label(ts))) {
		dev_info(&ts->client->dev,
			 "tristar: %s osos=0x00 (unmapped) flat=%d echo=%d deltas=%u id=%s r11=%d/%02x CONDET=%s VBUS=ENOTSUPP accmgr_type2=TODO\n",
			 why, ts->dump_flat, ts->i2c_echo, n,
			 tristar_id_label(ts), ts->reg11_ret,
			 ts->reg11_ret ? 0 : ts->reg11,
			 (!ts->dump_ok || ts->i2c_echo) ? "unknown" :
			 (ts->dump_flat ? "idle" : "not-flat"));
		if (!ts->dump_flat && !ts->i2c_echo && !ts->id_valid)
			dev_info(&ts->client->dev,
				 "tristar: unknown non-flat dump[0..15] %*ph\n",
				 16, ts->last_dump);
		tristar_log_line(ts, why);
	}
	return 0;
}

static int tristar_poll_cheap_locked(struct apple_tristar *ts)
{
	u8 v, echo = tristar_read_addr_echo(ts);
	int ret;

	if (ts->poll_disabled)
		return -ENOTSUPP;

	ret = tristar_read_reg(ts, 0x00, &v);
	if (ret) {
		ts->i2c_fail_streak++;
		ts->dump_ok = false;
		ts->poll_disabled = true;
		if (ts->i2c_fail_streak == 1)
			dev_warn(&ts->client->dev,
				 "tristar: poll read 0x00 failed %d — disabling poll\n",
				 ret);
		/* Do not call full 64-reg refresh on NACK — that is the storm. */
		return ret;
	}
	ts->i2c_fail_streak = 0;
	if (ts->i2c_echo && v == echo) {
		ts->polls++;
		return 0;
	}
	if (ts->dump_ok && ts->dump_flat && !ts->i2c_echo && v == ts->last_dump[0]) {
		ts->polls++;
		return 0;
	}
	return tristar_refresh_locked(ts, "poll");
}

static int tristar_refresh(struct apple_tristar *ts, const char *why)
{
	int ret;

	mutex_lock(&ts->lock);
	ret = tristar_refresh_locked(ts, why);
	mutex_unlock(&ts->lock);
	return ret;
}

int apple_tristar_refresh(void)
{
	struct apple_tristar *ts;
	int ret;

	mutex_lock(&tristar_singleton_lock);
	ts = tristar_singleton;
	if (!ts) {
		mutex_unlock(&tristar_singleton_lock);
		return -ENODEV;
	}
	ret = tristar_refresh(ts, "export");
	mutex_unlock(&tristar_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_tristar_refresh);

int apple_tristar_connected(void)
{
	struct apple_tristar *ts;
	int ret;

	mutex_lock(&tristar_singleton_lock);
	ts = tristar_singleton;
	if (!ts) {
		mutex_unlock(&tristar_singleton_lock);
		return -ENODEV;
	}
	mutex_lock(&ts->lock);
	if (!ts->dump_ok)
		ret = -EIO;
	else if (ts->i2c_echo)
		ret = -ENOTSUPP;
	else
		ret = ts->dump_flat ? 0 : 1;
	mutex_unlock(&ts->lock);
	mutex_unlock(&tristar_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_tristar_connected);

int apple_tristar_usb_routed(void)
{
	struct apple_tristar *ts;
	int ret;

	mutex_lock(&tristar_singleton_lock);
	ts = tristar_singleton;
	if (!ts) {
		mutex_unlock(&tristar_singleton_lock);
		return -ENODEV;
	}
	mutex_lock(&ts->lock);
	if (!ts->dump_ok)
		ret = -EIO;
	else if (ts->i2c_echo)
		ret = -ENOTSUPP;
	else if (!ts->id_valid)
		ret = 0;
	else
		ret = tristar_usb_dp_from_dx(ts->dx) ? 1 : 0;
	mutex_unlock(&ts->lock);
	mutex_unlock(&tristar_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_tristar_usb_routed);

int apple_tristar_lightning_analog(void)
{
	struct apple_tristar *ts;
	int ret;

	mutex_lock(&tristar_singleton_lock);
	ts = tristar_singleton;
	if (!ts) {
		mutex_unlock(&tristar_singleton_lock);
		return -ENODEV;
	}
	mutex_lock(&ts->lock);
	if (!ts->dump_ok)
		ret = -EIO;
	else if (ts->i2c_echo)
		ret = -ENOTSUPP;
	else
		ret = tristar_is_lightning_analog(ts) ? 1 : 0;
	mutex_unlock(&ts->lock);
	mutex_unlock(&tristar_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_tristar_lightning_analog);

int apple_tristar_vbus(void)
{
	mutex_lock(&tristar_singleton_lock);
	if (!tristar_singleton) {
		mutex_unlock(&tristar_singleton_lock);
		return -ENODEV;
	}
	mutex_unlock(&tristar_singleton_lock);
	/* TriStarVBUSProcessTask exists; I2C register is TODO RE. */
	return -ENOTSUPP;
}
EXPORT_SYMBOL_GPL(apple_tristar_vbus);

int apple_tristar_config_reg11(u8 *val)
{
	struct apple_tristar *ts;
	int ret;

	if (!val)
		return -EINVAL;
	mutex_lock(&tristar_singleton_lock);
	ts = tristar_singleton;
	if (!ts) {
		mutex_unlock(&tristar_singleton_lock);
		return -ENODEV;
	}
	mutex_lock(&ts->lock);
	if (!ts->dump_ok)
		ret = -EIO;
	else if (ts->i2c_echo)
		ret = -ENOTSUPP;
	else if (ts->reg11_ret)
		ret = ts->reg11_ret;
	else {
		*val = ts->reg11;
		ret = 0;
	}
	mutex_unlock(&ts->lock);
	mutex_unlock(&tristar_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_tristar_config_reg11);

/* Human-readable routing for one Dx lane. Lanes 1 and 2 differ at 6 and 7. */
static const char *tristar_dx_route(u8 sel, bool lane2)
{
	switch (sel) {
	case TS_DX_SEL_OPEN:		return "open";
	case TS_DX_SEL_USB:		return "usb";
	case TS_DX_SEL_UART0:		return "uart0";
	case TS_DX_SEL_DIGITAL_ID:	return "digital-id";
	case TS_DX_SEL_BRICK_ID_P:	return "brick-id+";
	case TS_DX_SEL_BRICK_ID_N:	return "brick-id-";
	case 6:				return lane2 ? "uart2" : "usb-alt";
	case 7:				return lane2 ? "uart1" : "jtag-dio";
	}
	return "?";
}

static const char *tristar_acc_route(u8 sel, bool lane2)
{
	switch (sel) {
	case TS_ACC_SEL_OPEN:		return "open";
	case TS_ACC_SEL_UART1:		return lane2 ? "uart1-tx" : "uart1-rx";
	case TS_ACC_SEL_JTAG:		return lane2 ? "jtag-clk" : "jtag-dio";
	case TS_ACC_SEL_ACC_POWER:	return "acc-power";
	}
	return "?";
}

/*
 * Read the accessory ID off the ID bus.
 *
 * The accessory answers with a fixed marker byte at the top of the FIFO,
 * followed by the payload in the registers *below* it, read downward. If the
 * marker is not there the bus may simply be the wrong way round, and a
 * re-orient plus a settle makes it answer.
 *
 * This driver never implemented any of that -- it dumped 0x00..0x3F and
 * pattern-matched, so it could not distinguish "no accessory" from "cable in
 * the other orientation".
 *
 * The re-orient step is a write, so it is skipped when the driver is in its
 * default read-only mode; the caller is told which case it got.
 */
static int tristar_read_accessory_id(struct apple_tristar *ts,
				     u8 id[TS_ID_PAYLOAD_LEN])
{
	u8 status = 0, marker = 0;
	unsigned int i;
	int ret;

	ret = tristar_read_reg(ts, TS_REG_STATUS, &status);
	if (ret)
		return ret;

	if ((status & TS_ST_CON_DET_N) || !(status & TS_ST_IDBUS_CONNECTED))
		return -ENODEV;

	ret = tristar_read_reg(ts, TS_REG_FIFO_TOP, &marker);
	if (ret)
		return ret;

	if (marker != TS_ID_FIFO_MARKER) {
		if (read_only) {
			dev_dbg(&ts->client->dev,
				"id: marker 0x%02x, re-orient needs a write (read_only=1)\n",
				marker);
			return -EAGAIN;
		}
		ret = i2c_smbus_write_byte_data(ts->client, TS_REG_IDBUS_CTL,
						TS_IDBUS_REORIENT);
		if (ret < 0)
			return ret;
		msleep(100);
		ret = tristar_read_reg(ts, TS_REG_FIFO_TOP, &marker);
		if (ret)
			return ret;
	}

	if (marker != TS_ID_FIFO_MARKER)
		return -EAGAIN;

	for (i = 0; i < TS_ID_PAYLOAD_LEN; i++) {
		ret = tristar_read_reg(ts, TS_REG_FIFO_TOP - (i + 1), &id[i]);
		if (ret)
			return ret;
	}
	return 0;
}

void apple_tristar_log_audio_path(struct device *audio_dev)
{
	struct apple_tristar *ts;

	if (!audio_dev)
		return;

	mutex_lock(&tristar_singleton_lock);
	ts = tristar_singleton;
	if (!ts) {
		mutex_unlock(&tristar_singleton_lock);
		dev_info(audio_dev,
			 "tristar unbound — 3.5mm is Mikey/CS42, not Lightning Dx\n");
		return;
	}
	mutex_lock(&ts->lock);
	tristar_refresh_locked(ts, "audio-path");
	{
		u8 status = 0, dx = 0, acc = 0;
		u8 id[TS_ID_PAYLOAD_LEN] = {};
		int idret;

		/*
		 * Report what the switches are actually doing.
		 *
		 * Decoded switch state, so dev_dbg: this whole block runs on
		 * every stream start and is a snapshot for diagnosis, not an
		 * event. Dynamic debug brings it back per call site.
		 */
		if (!tristar_read_reg(ts, TS_REG_STATUS, &status) &&
		    !tristar_read_reg(ts, TS_REG_DX_SWITCH, &dx) &&
		    !tristar_read_reg(ts, TS_REG_ACC_SWITCH, &acc)) {
			dev_dbg(audio_dev,
				 "tristar: connected=%d orient=%d switch_en=%d con_det=%d ovp=%d\n",
				 !!(status & TS_ST_IDBUS_CONNECTED),
				 !!(status & TS_ST_IDBUS_ORIENT),
				 !!(status & TS_ST_SWITCH_EN),
				 !(status & TS_ST_CON_DET_N),
				 !!(status & TS_ST_OVP));
			dev_dbg(audio_dev,
				 "tristar: Dx1=%s%s Dx2=%s%s ACC1=%s%s ACC2=%s%s\n",
				 tristar_dx_route(FIELD_GET(TS_DX1_SEL, dx), false),
				 (dx & TS_DX1_OVERRIDE) ? "(ovrd)" : "",
				 tristar_dx_route(FIELD_GET(TS_DX2_SEL, dx), true),
				 (dx & TS_DX2_OVERRIDE) ? "(ovrd)" : "",
				 tristar_acc_route(FIELD_GET(TS_ACC1_SEL, acc), false),
				 (acc & TS_ACC1_OVERRIDE) ? "(ovrd)" : "",
				 tristar_acc_route(FIELD_GET(TS_ACC2_SEL, acc), true),
				 (acc & TS_ACC2_OVERRIDE) ? "(ovrd)" : "");
		}

		idret = tristar_read_accessory_id(ts, id);
		if (!idret)
			dev_dbg(audio_dev, "tristar: accessory id %*ph\n",
				 TS_ID_PAYLOAD_LEN, id);
		else if (idret == -ENODEV)
			dev_dbg(audio_dev, "tristar: nothing on the ID bus\n");
		else
			dev_dbg(audio_dev,
				 "tristar: ID bus did not answer (%d)\n", idret);
	}

	dev_dbg(audio_dev,
		 "tristar audio-path: lightning=%s flat=%d echo=%d usb_dx=%u accx=%u analog_lightning=%d — 3.5mm jack is CS42+Mikey not Dx; USB route does not mute jack\n",
		 tristar_id_label(ts), ts->dump_flat, ts->i2c_echo, ts->dx,
		 ts->accx, tristar_is_lightning_analog(ts));
	if (tristar_is_lightning_analog(ts))
		dev_warn(audio_dev,
			 "Lightning analog ID 04 F1 present — that is EarPods-on-Lightning, not the onboard 3.5mm CS42 jack\n");
	mutex_unlock(&ts->lock);
	mutex_unlock(&tristar_singleton_lock);
}
EXPORT_SYMBOL_GPL(apple_tristar_log_audio_path);

static void tristar_poll_work(struct work_struct *work)
{
	struct apple_tristar *ts = container_of(to_delayed_work(work),
						struct apple_tristar, poll);

	mutex_lock(&ts->lock);
	tristar_poll_cheap_locked(ts);
	if (ts->poll_disabled) {
		mutex_unlock(&ts->lock);
		return;
	}
	mutex_unlock(&ts->lock);
	if (poll_ms > 0)
		schedule_delayed_work(&ts->poll, msecs_to_jiffies(poll_ms));
}

static int tristar_apply_init_sequence(struct apple_tristar *ts)
{
	struct device *dev = &ts->client->dev;
	struct device_node *np = dev->of_node;
	int n, i, ret;
	u32 reg, val;

	if (!np)
		return 0;

	n = of_property_count_u32_elems(np, "apple,init-sequence");
	if (n <= 0)
		return 0;
	if (!unsafe_writes && read_only) {
		dev_warn(dev,
			 "tristar: apple,init-sequence present but read_only=1 unsafe_writes=0 — not applied\n");
		return 0;
	}
	if (n % 2) {
		dev_err(dev, "apple,init-sequence must be reg,val pairs\n");
		return -EINVAL;
	}

	for (i = 0; i < n; i += 2) {
		of_property_read_u32_index(np, "apple,init-sequence", i, &reg);
		of_property_read_u32_index(np, "apple,init-sequence", i + 1, &val);
		ret = tristar_write_reg(ts, (u8)reg, (u8)val);
		if (ret) {
			dev_err(dev, "init write 0x%02x=0x%02x failed: %d\n",
				reg, val, ret);
			return ret;
		}
		dev_info(dev, "init 0x%02x <= 0x%02x\n", reg, val);
		udelay(100);
	}
	return 0;
}

static ssize_t dump_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	int i, n = 0;

	if (tristar_refresh(ts, "sysfs-dump"))
		return -EIO;
	mutex_lock(&ts->lock);
	for (i = 0; i < TRISTAR_DUMP_LEN; i++)
		n += scnprintf(buf + n, PAGE_SIZE - n, "%02x%s",
			       ts->last_dump[i],
			       (i + 1) % 16 ? " " : "\n");
	mutex_unlock(&ts->lock);
	return n;
}
static DEVICE_ATTR_RO(dump);

static ssize_t poke_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int reg, val;
	int ret;

	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;
	if (reg > 0xff || val > 0xff)
		return -EINVAL;
	if (!unsafe_writes)
		return -EPERM;
	ret = tristar_write_reg(ts, (u8)reg, (u8)val);
	if (ret)
		return ret;
	dev_info(dev, "poke 0x%02x <= 0x%02x\n", reg, val);
	return count;
}
static DEVICE_ATTR_WO(poke);

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	const char *name;

	if (tristar_refresh(ts, "sysfs-mode"))
		return sysfs_emit(buf, "unread\n");
	mutex_lock(&ts->lock);
	name = tristar_id_label(ts);
	mutex_unlock(&ts->lock);
	return sysfs_emit(buf, "%s\n", name);
}
static DEVICE_ATTR_RO(mode);

static ssize_t route_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	int n;

	if (tristar_refresh(ts, "sysfs-route"))
		return -EIO;
	mutex_lock(&ts->lock);
	n = sysfs_emit(buf,
		       "id=%s valid=%d accx=%u (%s) dx=%u id0=%s id1=%s usb=%d lightning_analog=%d echo=%d jack_3v5=cs42+mikey vbus=ENOTSUPP osos_event=0x00\n",
		       tristar_id_label(ts), ts->id_valid, ts->accx,
		       tristar_accx_name(ts->accx), ts->dx,
		       tristar_dx_usb_id0(ts->dx), tristar_dx_usb_id1(ts->dx),
		       ts->id_valid && tristar_usb_dp_from_dx(ts->dx),
		       tristar_is_lightning_analog(ts), ts->i2c_echo);
	mutex_unlock(&ts->lock);
	return n;
}
static DEVICE_ATTR_RO(route);

static ssize_t audio_path_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	int n;

	if (tristar_refresh(ts, "sysfs-audio-path"))
		return -EIO;
	mutex_lock(&ts->lock);
	if (tristar_is_lightning_analog(ts))
		n = sysfs_emit(buf,
			       "selected=lightning-analog (04 F1) — not onboard 3.5mm\n");
	else
		n = sysfs_emit(buf,
			       "selected=onboard-3.5mm cs42+mikey lightning=%s\n",
			       tristar_id_label(ts));
	mutex_unlock(&ts->lock);
	return n;
}
static DEVICE_ATTR_RO(audio_path);

static ssize_t vbus_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return sysfs_emit(buf, "ENOTSUPP (TriStarVBUSProcessTask I2C map TODO RE)\n");
}
static DEVICE_ATTR_RO(vbus);

static ssize_t read_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int reg;
	u8 val;
	int ret;

	if (kstrtouint(buf, 0, &reg) || reg > 0xff)
		return -EINVAL;
	ret = tristar_read_reg(ts, (u8)reg, &val);
	if (ret)
		return ret;
	ts->read_reg = (u8)reg;
	ts->read_val = val;
	return count;
}

static ssize_t read_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "0x%02x\n", ts->read_val);
}
static DEVICE_ATTR_RW(read);

static ssize_t value_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "0x%02x (reg 0x%02x)\n",
			  ts->read_val, ts->read_reg);
}
static DEVICE_ATTR_RO(value);

static ssize_t verify_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	int n;

	if (tristar_refresh(ts, "sysfs-verify"))
		return sysfs_emit(buf, "FAIL read\n");
	mutex_lock(&ts->lock);
	if (ts->i2c_echo)
		n = sysfs_emit(buf,
			       "I2C_ECHO 0x%02x (8-bit read addr; chip SRAM not visible)\n",
			       ts->last_dump[0]);
	else if (ts->dump_flat)
		n = sysfs_emit(buf, "IDLE flat 0x%02x (no Lightning IDBUS accessory)\n",
			       ts->last_dump[0]);
	else
		n = sysfs_emit(buf, "STATUS_OK id=%s\n", tristar_id_label(ts));
	mutex_unlock(&ts->lock);
	return n;
}
static DEVICE_ATTR_RO(verify);

static ssize_t poll_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	int ret;

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	ret = tristar_refresh(ts, "sysfs-poll");
	return ret ? ret : count;
}

static ssize_t poll_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	int n;

	mutex_lock(&ts->lock);
	n = sysfs_emit(buf, "flat=%d echo=%d last_ok=%d polls=%u id=%s\n",
		       ts->dump_flat, ts->i2c_echo, ts->dump_ok, ts->polls,
		       tristar_id_label(ts));
	mutex_unlock(&ts->lock);
	return n;
}
static DEVICE_ATTR_RW(poll);

/*
 * Read-only telemetry.
 *
 * TriStar is not the charger -- it decides what is attached to Lightning
 * and where the signal paths go, while the D1830 owns the battery and the
 * power path. What it does hold is the attach/detach state, which is the
 * missing input to any charging policy, so making its state observable is
 * the useful contribution here rather than trying to drive anything.
 *
 *   tristar_stats  counters and decoded state in one place
 *   tristar_regs   the dump with labels on the registers we can justify
 *   tristar_watch  echo 1 to snapshot, read to see only what moved
 *
 * The watch is the one that matters for attach detection: snapshot, change
 * the cable, read back, and whatever moved is the short list.
 */
static ssize_t tristar_stats_show(struct device *dev,
				  struct device_attribute *a, char *buf)
{
	struct apple_tristar *ts = dev_get_drvdata(dev);

	if (!ts)
		return -ENODEV;

	return sysfs_emit(buf,
		"polls=%u deltas=%u writes=%u i2c_fail_streak=%u poll_disabled=%d\n"
		"dump_ok=%d dump_flat=%d i2c_echo=%d seen_mask=%08x\n"
		"id=%s id_valid=%d id_off=%u accx=%02x dx=%02x\n"
		"reg11=%02x reg11_ret=%d osos_event=%02x prev=%02x cf9_latch=%d cfa=%02x\n",
		ts->polls, ts->deltas, ts->writes, ts->i2c_fail_streak,
		ts->poll_disabled,
		ts->dump_ok, ts->dump_flat, ts->i2c_echo, ts->seen_mask,
		ts->id_name ? ts->id_name : "unknown", ts->id_valid,
		ts->id_off, ts->accx, ts->dx,
		ts->reg11, ts->reg11_ret, ts->osos_event, ts->prev_osos_event,
		ts->cf9_latch, ts->cfa_state);
}
static DEVICE_ATTR_RO(tristar_stats);

/*
 * Only 0x11 has a name we can defend -- CBTL1610 configuration status,
 * and it may NAK or echo on a 1609. The rest are listed as offsets with
 * their observed values so a diff has somewhere to point; naming them
 * before correlation would turn guesses into apparent fact, which is
 * exactly how a rail block at 0x40 got invented for the PMIC.
 */
static ssize_t tristar_regs_show(struct device *dev,
				 struct device_attribute *a, char *buf)
{
	struct apple_tristar *ts = dev_get_drvdata(dev);
	unsigned int i;
	int len = 0;

	if (!ts)
		return -ENODEV;
	if (!ts->dump_ok)
		return sysfs_emit(buf, "no valid dump (dump_ok=0)\n");

	mutex_lock(&ts->lock);
	for (i = 0; i < TRISTAR_DUMP_LEN && len < PAGE_SIZE - 48; i++) {
		if (!ts->last_dump[i])
			continue;	/* zeros are the overwhelming majority */
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "0x%02x = %02x%s\n", i, ts->last_dump[i],
				 i == 0x11 ? "  (CBTL1610 config status)" : "");
	}
	mutex_unlock(&ts->lock);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "# non-zero offsets only; flat=%d echo=%d\n",
			 ts->dump_flat, ts->i2c_echo);
	return len;
}
static DEVICE_ATTR_RO(tristar_regs);

static u8 tristar_snap[TRISTAR_DUMP_LEN];
static bool tristar_snap_valid;

static ssize_t tristar_watch_show(struct device *dev,
				  struct device_attribute *a, char *buf)
{
	struct apple_tristar *ts = dev_get_drvdata(dev);
	unsigned int i, changed = 0;
	int len = 0;

	if (!ts)
		return -ENODEV;
	if (!tristar_snap_valid)
		return sysfs_emit(buf,
				  "no snapshot; echo 1 > tristar_watch first\n");

	mutex_lock(&ts->lock);
	for (i = 0; i < TRISTAR_DUMP_LEN && len < PAGE_SIZE - 64; i++) {
		if (ts->last_dump[i] == tristar_snap[i])
			continue;
		changed++;
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "0x%02x %02x -> %02x  (xor %02x)\n",
				 i, tristar_snap[i], ts->last_dump[i],
				 tristar_snap[i] ^ ts->last_dump[i]);
	}
	mutex_unlock(&ts->lock);
	if (!changed)
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "no change\n");
	return len;
}

static ssize_t tristar_watch_store(struct device *dev,
				   struct device_attribute *a,
				   const char *buf, size_t count)
{
	struct apple_tristar *ts = dev_get_drvdata(dev);

	if (!ts)
		return -ENODEV;
	if (buf[0] != '1')
		return -EINVAL;
	mutex_lock(&ts->lock);
	memcpy(tristar_snap, ts->last_dump, TRISTAR_DUMP_LEN);
	tristar_snap_valid = true;
	mutex_unlock(&ts->lock);
	return count;
}
static DEVICE_ATTR_RW(tristar_watch);

static struct attribute *tristar_attrs[] = {
	&dev_attr_dump.attr,
	&dev_attr_poke.attr,
	&dev_attr_mode.attr,
	&dev_attr_route.attr,
	&dev_attr_audio_path.attr,
	&dev_attr_vbus.attr,
	&dev_attr_read.attr,
	&dev_attr_value.attr,
	&dev_attr_verify.attr,
	&dev_attr_poll.attr,
	&dev_attr_tristar_stats.attr,
	&dev_attr_tristar_regs.attr,
	&dev_attr_tristar_watch.attr,
	NULL,
};
ATTRIBUTE_GROUPS(tristar);

static int tristar_dbg_anchors_show(struct seq_file *m, void *p)
{
	seq_puts(m,
		 "OSOS central loop: sub_11C8C\n"
		 "OSOS tasks: TriStarIDProcessTask, TriStarVBUSProcessTask, TriStarCONDETProcessTask\n"
		 "OSOS accessory manager: accessoryMgr.cpp:716 type1=MikeyBus UART2, type2=TODO RE\n"
		 "OSOS MikeyBus: CMikeyBusUartReadTask, CMikeyBusUartResistorTask, mikeyTask.cpp:169\n"
		 "I2C: 7-bit 0x1a (8-bit WR 0x34 / RD 0x35)\n"
		 "Routing: IDBUS 0x74/0x75 inside CBTL1609 — Linux does not write Dx\n"
		 "0x75 ACCx/Dx tables: THS7383 datasheet via nyansatan\n"
		 "reg 0x11: CBTL1610 config status (Lina); may NAK or echo on CBTL1609\n"
		 "OSOS v36 bits 0x01/0x04/0x08/0x10/0x20: logged as unmapped, not synthesized\n"
		 "3.5mm analog: CS42 + apple-mikeybus, not Tristar Dx\n"
		 "Lightning analog EarPods: ID 04 F1 00 00 00 00\n"
		 "Bootloader PMIC sub_3F40/3F60 is not Tristar\n"
		 "Candidate index pmic-tristar-ida-out.txt is not a write recipe\n");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(tristar_dbg_anchors);

static int tristar_dbg_status_show(struct seq_file *m, void *p)
{
	struct apple_tristar *ts = m->private;

	mutex_lock(&ts->lock);
	seq_printf(m,
		   "osos_event=0x%02x\nprev_osos_event=0x%02x\nseen_mask=0x%02x\n"
		   "osos_masks=0x%02x,0x%02x,0x%02x,0x%02x,0x%02x (I2C producer TODO RE)\n"
		   "cf9_latch=%d\ncfa_state=%u\n"
		   "dump_ok=%d dump_flat=%d i2c_echo=%d\nreg11_ret=%d reg11=0x%02x\n"
		   "id=%s usb_routed=%d lightning_analog=%d\n"
		   "read_only=%d unsafe_writes=%d unsafe_acks=%d poll_ms=%d\n"
		   "polls=%u writes=%u\n",
		   ts->osos_event, ts->prev_osos_event, ts->seen_mask & 0xff,
		   (unsigned int)N31_TS_EVENT_01, (unsigned int)N31_TS_EVENT_04,
		   (unsigned int)N31_TS_EVENT_08, (unsigned int)N31_TS_EVENT_10,
		   (unsigned int)N31_TS_EVENT_20,
		   ts->cf9_latch, ts->cfa_state,
		   ts->dump_ok, ts->dump_flat, ts->i2c_echo, ts->reg11_ret,
		   ts->reg11, tristar_id_label(ts),
		   ts->id_valid && tristar_usb_dp_from_dx(ts->dx),
		   tristar_is_lightning_analog(ts),
		   read_only, unsafe_writes, unsafe_acks, poll_ms, ts->polls,
		   ts->writes);
	mutex_unlock(&ts->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(tristar_dbg_status);

static int tristar_dbg_log_show(struct seq_file *m, void *p)
{
	struct apple_tristar *ts = m->private;
	unsigned int i, n, idx;

	mutex_lock(&ts->lock);
	n = ts->log_count;
	idx = (ts->log_head + TRISTAR_LOG_LEN - n) % TRISTAR_LOG_LEN;
	for (i = 0; i < n; i++) {
		seq_printf(m, "%s\n", ts->log[idx]);
		idx = (idx + 1) % TRISTAR_LOG_LEN;
	}
	mutex_unlock(&ts->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(tristar_dbg_log);

static int tristar_dbg_counters_show(struct seq_file *m, void *p)
{
	struct apple_tristar *ts = m->private;

	mutex_lock(&ts->lock);
	seq_printf(m, "polls=%u\ndeltas=%u\nwrites=%u\nacks=0\n",
		   ts->polls, ts->deltas, ts->writes);
	mutex_unlock(&ts->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(tristar_dbg_counters);

static int tristar_dbg_mode_show(struct seq_file *m, void *p)
{
	struct apple_tristar *ts = m->private;

	mutex_lock(&ts->lock);
	seq_printf(m,
		   "read_only=%d\nunsafe_writes=%d\nunsafe_acks=%d\n"
		   "i2c_echo=%d\nid=%s\n",
		   read_only, unsafe_writes, unsafe_acks, ts->i2c_echo,
		   tristar_id_label(ts));
	mutex_unlock(&ts->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(tristar_dbg_mode);

static int tristar_dbg_unsafe_show(struct seq_file *m, void *p)
{
	seq_printf(m, "%d\n", unsafe_writes);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(tristar_dbg_unsafe);

static void tristar_debugfs_init(struct apple_tristar *ts)
{
	ts->debug_root = debugfs_create_dir("n31_tristar", NULL);
	if (IS_ERR_OR_NULL(ts->debug_root)) {
		ts->debug_root = NULL;
		return;
	}
	debugfs_create_file("source_anchors", 0444, ts->debug_root, ts,
			    &tristar_dbg_anchors_fops);
	debugfs_create_file("raw_status", 0444, ts->debug_root, ts,
			    &tristar_dbg_status_fops);
	debugfs_create_file("event_log", 0444, ts->debug_root, ts,
			    &tristar_dbg_log_fops);
	debugfs_create_file("counters", 0444, ts->debug_root, ts,
			    &tristar_dbg_counters_fops);
	debugfs_create_file("mode", 0444, ts->debug_root, ts,
			    &tristar_dbg_mode_fops);
	debugfs_create_file("unsafe_writes_enabled", 0444, ts->debug_root, ts,
			    &tristar_dbg_unsafe_fops);
}

static int apple_tristar_probe(struct i2c_client *client)
{
	struct apple_tristar *ts;
	struct device *dev = &client->dev;
	int ret;
	u8 id0 = 0xff;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->client = client;
	mutex_init(&ts->lock);
	INIT_DELAYED_WORK(&ts->poll, tristar_poll_work);
	i2c_set_clientdata(client, ts);

	dev_info(dev,
		 "N31 Tristar: OSOS sub_11C8C ID/VBUS/CONDET; I2C 7-bit 0x%02x on %s\n",
		 client->addr, client->adapter->name);
	dev_info(dev,
		 "N31 Tristar: IDBUS routes USB/UART/SWD; read_only=%d unsafe_writes=%d poll_ms=%d\n",
		 read_only, unsafe_writes, poll_ms);
	dev_info(dev,
		 "N31 Tristar: 3.5mm analog is CS42+MikeyBus — not a Dx write; type2 accessoryMgr TODO RE\n");
	dev_info(dev,
		 "N31 Tristar: OSOS v36 bits 0x01/0x04/0x08/0x10/0x20 unmapped; VBUS I2C map ENOTSUPP\n");

	if (of_property_read_bool(dev->of_node, "apple,require-ack")) {
		ret = tristar_read_reg(ts, 0x00, &id0);
		if (ret) {
			dev_err(dev,
				"Tristar no ACK at 7-bit 0x%02x on %s (err=%d)\n",
				client->addr, client->adapter->name, ret);
			return -ENODEV;
		}
		dev_info(dev, "Lightning Tristar ACK @7bit=0x%02x reg0=0x%02x\n",
			 client->addr, id0);
	}

	tristar_apply_init_sequence(ts);
	tristar_debugfs_init(ts);

	ret = sysfs_create_groups(&dev->kobj, tristar_groups);
	if (ret)
		dev_warn(dev, "sysfs groups failed: %d\n", ret);

	mutex_lock(&tristar_singleton_lock);
	tristar_singleton = ts;
	mutex_unlock(&tristar_singleton_lock);

	/*
	 * Probe with a single cheap ACK. Full 64-reg refresh only if the
	 * chip answers — otherwise one warn and stay quiet (I2C0 often
	 * NACKs until Lightning/mux is up).
	 */
	ret = tristar_read_reg(ts, 0x00, &id0);
	if (ret) {
		ts->dump_ok = false;
		ts->poll_disabled = true;
		ts->i2c_fail_streak = 1;
		dev_warn(dev,
			 "tristar: probe ACK failed %d — leaving unbound quiet (poll off)\n",
			 ret);
	} else {
		tristar_refresh(ts, "probe");
		if (poll_ms > 0 && !ts->poll_disabled)
			schedule_delayed_work(&ts->poll,
					      msecs_to_jiffies(poll_ms));
	}
	return 0;
}

static void apple_tristar_remove(struct i2c_client *client)
{
	struct apple_tristar *ts = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&ts->poll);
	mutex_lock(&tristar_singleton_lock);
	if (tristar_singleton == ts)
		tristar_singleton = NULL;
	mutex_unlock(&tristar_singleton_lock);
	debugfs_remove_recursive(ts->debug_root);
	sysfs_remove_groups(&client->dev.kobj, tristar_groups);
}

static const struct of_device_id apple_tristar_of_match[] = {
	{ .compatible = "apple,tristar-cbtl1609" },
	{ .compatible = "nxp,cbtl1609a1" },
	{ .compatible = "apple,n31-tristar" },
	{ },
};
MODULE_DEVICE_TABLE(of, apple_tristar_of_match);

static const struct i2c_device_id apple_tristar_id[] = {
	{ "tristar-cbtl1609" },
	{ },
};
MODULE_DEVICE_TABLE(i2c, apple_tristar_id);

static struct i2c_driver apple_tristar_driver = {
	.driver = {
		.name = "apple-tristar",
		.of_match_table = apple_tristar_of_match,
	},
	.probe = apple_tristar_probe,
	.remove = apple_tristar_remove,
	.id_table = apple_tristar_id,
};
module_i2c_driver(apple_tristar_driver);

MODULE_DESCRIPTION("Apple Lightning Tristar / NXP CBTL1609A1 mux");
MODULE_AUTHOR("Hydrogenuine / FreeMyiPod N31 bring-up");
MODULE_LICENSE("GPL");
