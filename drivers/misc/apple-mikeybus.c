// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple N31 MikeyBus, decomp-aligned Linux driver.
 *
 * RetailOS objects:
 *   CMikeyBusUartReadTask
 *   CMikeyBusUartResistorTask
 *
 * Read path:
 *   sub_570BA8 opens channel 4 with command 9/0x71/channel4.
 *   sub_500ECC handles lower packet type 0x70 and appends payload bytes
 *   to a 1024-byte RX ring.
 *   sub_2542F0 drains that RX ring and appends each byte to a parent stream.
 *   If byte == 0xAA, it appends an extra 0x01 to the stream.
 *
 * Resistor/model path:
 *   sub_410DB0 enables channel 3 and submits command 3/0x8D/channel3.
 *   It waits with timeout 100 on the backend result object.
 *   It reads sample byte 0x8A92444 and modifier byte 0x8A9244C.
 *   If sample == 15 and modifier == 0, sample is remapped to 100.
 *
 * Presence/model state:
 *   sub_587F38 consumes 0x7E / 0x8A-like presence events.
 *   sub_17DD6C toggles the model modifier/state byte and opens/closes
 *   the read path.
 *
 * Gate/audio route:
 *   sub_42D364 and sub_587E60 tie the Mikey state to the CS42/audio route.
 *   Do not blindly poke audio rails here.
 *
 * Not implemented yet:
 *   button input-event mapping.
 *
 * Never do:
 *   GPIO66/67 resistor detection. Those are UART pins, not model-detect GPIOs.
 *
 * Optional UART pad mux (GPIO 66/67 = TX/RX):
 *   GPIO_PHYS 0x3cf00000, GPIOCMD_PHYS 0x3cf001e0, TX=0x42 RX=0x43,
 *   GPIOCMD mode 2 for UART function ONLY. Never sample those pins as
 *   resistor DIN / model detect.
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#define GPIO_PHYS		0x3cf00000ul
#define GPIOCMD_PHYS		0x3cf001e0ul

/* UART2 pad mux only — NOT DIN / resistor detect. */
#define MIKEY_GPIO_TX		0x42u	/* 66 — UART TX mux */
#define MIKEY_GPIO_RX		0x43u	/* 67 — UART RX mux */

#define MIKEY_RX_RING_SIZE	1024
#define MIKEY_TASK_RING_SIZE	2048

#define MIKEY_SAMPLE_DEFAULT		0x64
#define MIKEY_SAMPLE_OPEN_CIRCUIT	0x0b
#define MIKEY_SAMPLE_REMAP_FROM		0x0f

#define MIKEY_CH_RESISTOR	3
#define MIKEY_CH_READ		4

#define MIKEY_PKT_RX_BYTES	0x70
#define MIKEY_PKT_IGNORED_74	0x74
#define MIKEY_PKT_STATUS_76	0x76
#define MIKEY_PKT_STATUS_8A	0x8a

#define MIKEY_INJECT_MAX	64

/* -------------------- module parameters -------------------- */

static bool force_plugged;
module_param(force_plugged, bool, 0644);
MODULE_PARM_DESC(force_plugged, "Force headset plugged state for bring-up");

static int force_model = -1;
module_param(force_model, int, 0644);
MODULE_PARM_DESC(force_model, "Force headset model sample, -1 disables");

static bool auto_report = true;
module_param(auto_report, bool, 0644);
MODULE_PARM_DESC(auto_report, "Print plug/unplug/model changes to kernel log");

static bool active_probe;
module_param(active_probe, bool, 0644);
MODULE_PARM_DESC(active_probe,
		 "Experimental: actively send model probe command if transport is implemented");

static int poll_ms = 500;
module_param(poll_ms, int, 0644);
MODULE_PARM_DESC(poll_ms, "Model poll interval in milliseconds");

/*
 * Remote buttons.
 *
 * What the decomp proves: the event vocabulary, from the handler names --
 * HandleMikeyCenter, HandleMikeyVolumeUp, HandleMikeyVolumeDown and
 * HandleMikeyAllUp. That last one is the useful inference. An "all up"
 * event only makes sense if the wire carries a bitmap of the buttons
 * currently held rather than discrete press and release codes: with
 * discrete codes each button would report its own release and a combined
 * all-up event would be redundant. So this decodes a held-button bitmap and
 * derives press/release by comparing against the previous value, with an
 * all-zero byte meaning everything is released.
 *
 * What the decomp does NOT give is which bit is which button. The names
 * prove the vocabulary, not the wire encoding, and nothing in the
 * disassembly pins the bit order down. Guessing it in the source would bury
 * an assumption somewhere it cannot be seen, so it lives here instead: one
 * keycode per bit, changeable at runtime. Read remote_raw while pressing a
 * known button, see which bit moves, and set the map accordingly -- that is
 * one session with the hardware rather than an argument about byte order.
 *
 * The default below is a placeholder ordering, NOT an attested one.
 */
#define MIKEY_BUTTON_BITS	8

static int button_map[MIKEY_BUTTON_BITS] = {
	KEY_PLAYPAUSE, KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_NEXTSONG,
	KEY_PREVIOUSSONG, 0, 0, 0,
};
static int button_map_count = MIKEY_BUTTON_BITS;
module_param_array(button_map, int, &button_map_count, 0644);
MODULE_PARM_DESC(button_map,
		 "keycode per remote bitmap bit 0..7, 0=unused; bit order is NOT attested, confirm with remote_raw");

static int baud = 115200;
module_param(baud, int, 0644);
MODULE_PARM_DESC(baud, "MikeyBus UART baud rate");

static bool accept_case3_model = true;
module_param(accept_case3_model, bool, 0644);
MODULE_PARM_DESC(accept_case3_model,
		 "Accept backend packet class 3 as model sample candidate");

/* -------------------- state -------------------- */

struct apple_mikey_ring {
	u8 data[MIKEY_TASK_RING_SIZE];
	u16 head;
	u16 tail;
	u32 drops;
};

struct apple_mikeybus {
	struct device *dev;
	struct serdev_device *serdev;
	struct mutex lock;
	struct delayed_work poll_work;

	void __iomem *gpio;
	void __iomem *gpiocmd;
	bool pinmux_on;
	bool uart_opened;

	bool auto_report;
	bool active_probe;
	bool resistor_backend_ready;

	bool plugged;
	bool last_reported_plugged;

	u8 model;
	u8 last_reported_model;
	u8 model_sample;
	u8 model_modifier;

	bool force_plugged;
	int force_model;

	u32 decomp_channel_mask_shadow;
	u8 rx_status_shadow;

	struct apple_mikey_ring rx_raw;
	struct apple_mikey_ring rx_task_stream;
	struct apple_mikey_ring remote_raw;	/* channel 4 only */

	struct input_dev *input;
	u8 last_buttons;
	u32 remote_bytes;
	u32 button_events;
	u32 all_up_events;

	u32 rx_bytes;
	u32 lower_packets;
	u32 lower_rx70_packets;
	u32 lower_status_packets;
	u32 presence_packets;
	u32 aa_stuff_count;
	u32 model_changes;
	u32 plug_events;
	u32 unplug_events;
	u32 active_probe_count;
	u32 active_probe_fail_count;

	int baud;
};

static struct apple_mikeybus *mikeybus_singleton;
static DEFINE_MUTEX(mikeybus_singleton_lock);
static struct platform_device *mikey_plat_pdev;
static void mikey_ensure_plat(struct work_struct *work);
static DECLARE_WORK(mikey_plat_work, mikey_ensure_plat);

/* -------------------- model tables -------------------- */

static const char *mikey_model_name(u8 model)
{
	switch (model) {
	case 0x01: return "A18";
	case 0x02: return "B18";
	case 0x03: return "A62";
	case 0x04: return "B15";
	case 0x05: return "A36";
	case 0x06: return "Apple noise occluding";
	case 0x07: return "mfg noise occluding";
	case 0x08: return "mfg noise occluding w/ mic";
	case 0x09: return "mfg std";
	case 0x0a: return "mfg std w/ mic";
	case 0x0b: return "open circuit";
	case 0x0d: return "B60f";
	case 0x0e: return "B60g";
	case 0x0f: return "B149";
	case 0x10: return "B187";
	case 0x64: return "default/open/unknown";
	default: return "inscrutable";
	}
}

static bool mikey_sample_is_plugged(u8 sample)
{
	switch (sample) {
	case 0x0b: /* open circuit */
	case 0x64: /* RetailOS default/open/unknown */
		return false;
	default:
		return true;
	}
}

/* -------------------- CS42 exports -------------------- */

int apple_mikeybus_jack_present(void)
{
	int ret;

	mutex_lock(&mikeybus_singleton_lock);
	if (!mikeybus_singleton) {
		mutex_unlock(&mikeybus_singleton_lock);
		return -ENODEV;
	}
	mutex_lock(&mikeybus_singleton->lock);
	ret = mikeybus_singleton->plugged ? 1 : 0;
	mutex_unlock(&mikeybus_singleton->lock);
	mutex_unlock(&mikeybus_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_mikeybus_jack_present);

int apple_mikeybus_headset_ready(void)
{
	/* Same as jack_present for now (analog HP gate). */
	return apple_mikeybus_jack_present();
}
EXPORT_SYMBOL_GPL(apple_mikeybus_headset_ready);

/* -------------------- rings -------------------- */

static void mikey_ring_put(struct apple_mikey_ring *r, u8 b)
{
	u16 next = (r->head + 1) % sizeof(r->data);

	if (next == r->tail) {
		r->drops++;
		r->tail = (r->tail + 1) % sizeof(r->data);
	}

	r->data[r->head] = b;
	r->head = next;
}

static size_t mikey_ring_dump_hex(struct apple_mikey_ring *r,
				  char *buf, size_t max)
{
	size_t n = 0;
	u16 p = r->tail;

	while (p != r->head && n + 4 < max) {
		n += scnprintf(buf + n, max - n, "%02x ", r->data[p]);
		p = (p + 1) % sizeof(r->data);
	}

	if (n && n < max)
		buf[n - 1] = '\n';

	return n;
}

/* -------------------- UART pad mux (UART function only) -------------------- */

static void mikey_gpiocmd(struct apple_mikeybus *m, u8 gpio, u8 mode)
{
	u32 bank = gpio >> 3;
	u32 pin = gpio & 7;

	if (!m->gpiocmd)
		return;
	writel((bank << 16) | (pin << 8) | mode, m->gpiocmd);
}

static void mikey_pinmux_uart(struct apple_mikeybus *m, bool on)
{
	u32 bank, pin, dir;
	void __iomem *b;

	if (!m->gpio || !m->gpiocmd)
		return;

	if (on) {
		bank = MIKEY_GPIO_TX >> 3;
		pin = MIKEY_GPIO_TX & 7;
		b = m->gpio + 32 * bank;
		dir = readl(b + 0x14);
		writel(dir | BIT(pin), b + 0x14);
		mikey_gpiocmd(m, MIKEY_GPIO_TX, 2);

		bank = MIKEY_GPIO_RX >> 3;
		pin = MIKEY_GPIO_RX & 7;
		b = m->gpio + 32 * bank;
		dir = readl(b + 0x14);
		writel(dir | BIT(pin), b + 0x14);
		mikey_gpiocmd(m, MIKEY_GPIO_RX, 2);
		m->pinmux_on = true;
	} else {
		bank = MIKEY_GPIO_TX >> 3;
		pin = MIKEY_GPIO_TX & 7;
		b = m->gpio + 32 * bank;
		dir = readl(b + 0x14);
		writel(dir & ~BIT(pin), b + 0x14);
		mikey_gpiocmd(m, MIKEY_GPIO_TX, 0);

		bank = MIKEY_GPIO_RX >> 3;
		pin = MIKEY_GPIO_RX & 7;
		b = m->gpio + 32 * bank;
		dir = readl(b + 0x14);
		writel(dir & ~BIT(pin), b + 0x14);
		mikey_gpiocmd(m, MIKEY_GPIO_RX, 0);
		m->pinmux_on = false;
	}
}

/* -------------------- RX / state -------------------- */

static void mikey_rx_byte_locked(struct apple_mikeybus *m, u8 b)
{
	mikey_ring_put(&m->rx_raw, b);
	mikey_ring_put(&m->rx_task_stream, b);

	if (b == 0xaa) {
		mikey_ring_put(&m->rx_task_stream, 0x01);
		m->aa_stuff_count++;
	}

	m->rx_bytes++;
}

/*
 * One byte of the channel 4 remote stream.
 *
 * Treated as a held-button bitmap: bits set now that were not set before are
 * presses, bits that cleared are releases, and 0x00 releases everything --
 * the AllUp case. Reporting is edge-driven, so a repeated identical byte
 * costs nothing and a dropped byte self-corrects on the next one.
 */
static void mikey_remote_byte_locked(struct apple_mikeybus *m, u8 b)
{
	u8 changed;
	unsigned int bit;

	mikey_ring_put(&m->remote_raw, b);
	m->remote_bytes++;

	if (!m->input)
		return;

	changed = b ^ m->last_buttons;
	if (!changed)
		return;

	for (bit = 0; bit < MIKEY_BUTTON_BITS; bit++) {
		int code = button_map[bit];

		if (!code || !(changed & BIT(bit)))
			continue;
		input_report_key(m->input, code, !!(b & BIT(bit)));
		m->button_events++;
	}
	input_sync(m->input);

	if (!b && m->last_buttons)
		m->all_up_events++;

	m->last_buttons = b;
}

static void mikey_report_state_locked(struct apple_mikeybus *m,
				      const char *reason)
{
	if (!m->auto_report)
		return;

	if (m->plugged == m->last_reported_plugged &&
	    m->model == m->last_reported_model)
		return;

	if (m->plugged && m->last_reported_plugged &&
	    m->model != m->last_reported_model) {
		dev_info(m->dev,
			 "headset changed: %s sample=0x%02x modifier=%u reason=%s\n",
			 mikey_model_name(m->model), m->model_sample,
			 m->model_modifier, reason);
	} else if (m->plugged) {
		dev_info(m->dev,
			 "headset plugged: %s sample=0x%02x modifier=%u reason=%s\n",
			 mikey_model_name(m->model), m->model_sample,
			 m->model_modifier, reason);
		m->plug_events++;
	} else {
		dev_info(m->dev,
			 "headset unplugged: %s sample=0x%02x modifier=%u reason=%s\n",
			 mikey_model_name(m->model), m->model_sample,
			 m->model_modifier, reason);
		m->unplug_events++;
	}

	m->last_reported_plugged = m->plugged;
	m->last_reported_model = m->model;
}

static void mikey_apply_model_sample_locked(struct apple_mikeybus *m,
					    u8 sample,
					    const char *reason)
{
	bool plugged;
	u8 model;

	if (sample == MIKEY_SAMPLE_REMAP_FROM && m->model_modifier == 0)
		sample = MIKEY_SAMPLE_DEFAULT;

	if (m->force_plugged) {
		plugged = true;
		model = (m->force_model >= 0) ? (u8)m->force_model : sample;
	} else if (m->force_model >= 0) {
		sample = (u8)m->force_model;
		model = sample;
		plugged = mikey_sample_is_plugged(sample);
	} else {
		model = sample;
		plugged = mikey_sample_is_plugged(sample);
	}

	if (m->model_sample != sample || m->model != model ||
	    m->plugged != plugged)
		m->model_changes++;

	m->model_sample = sample;
	m->model = model;
	m->plugged = plugged;

	mikey_report_state_locked(m, reason);
}

/* -------------------- lower / backend packets -------------------- */

static void mikey_handle_lower_packet_locked(struct apple_mikeybus *m,
					     const u8 *pkt, size_t len)
{
	u8 type;
	u8 v;
	size_t i;
	u8 count;

	if (len < 2)
		return;

	m->lower_packets++;
	type = pkt[1];

	switch (type) {
	case MIKEY_PKT_RX_BYTES:
		if (len < 3 || pkt[0] < 3 || pkt[0] > len)
			return;

		count = pkt[0] - 3;

		/*
		 * pkt[2] is the channel and was being thrown away, so the
		 * headset-model stream on channel 3 and the remote stream on
		 * channel 4 were interleaved into one buffer. They are
		 * different protocols; anything reading the mixed result is
		 * parsing two things at once. Keep feeding both to the raw
		 * ring for tracing, but route channel 4 to the remote decoder.
		 */
		for (i = 0; i < count; i++) {
			u8 payload = pkt[3 + i];

			mikey_rx_byte_locked(m, payload);
			if (pkt[2] == MIKEY_CH_READ)
				mikey_remote_byte_locked(m, payload);
		}

		m->lower_rx70_packets++;
		break;

	case MIKEY_PKT_IGNORED_74:
		break;

	case MIKEY_PKT_STATUS_76:
	case MIKEY_PKT_STATUS_8A:
		if (len < 4)
			return;

		v = pkt[3];

		if (v & 0x10)
			m->rx_status_shadow = 0;
		else if (v & 0x20)
			m->rx_status_shadow = 0x80;

		m->lower_status_packets++;
		break;

	default:
		dev_dbg(m->dev, "unknown lower packet type=0x%02x len=%zu\n",
			type, len);
		break;
	}
}

static bool model_completion_byte_plausible(u8 b)
{
	switch (b) {
	case 0x01 ... 0x0b:
	case 0x0d ... 0x10:
	case 0x64:
		return true;
	default:
		return false;
	}
}

static void mikey_handle_model_completion_candidate_locked(
	struct apple_mikeybus *m, const u8 *pkt, size_t len)
{
	u8 sample = 0xff;

	if (!accept_case3_model)
		return;

	/*
	 * Conservative candidate extraction:
	 *   Try packet[3], packet[4], packet[1] in that order.
	 *
	 * Why not hard-code one forever?
	 *   The decomp proves the sample global is written in the case-3 lane,
	 *   but the current dump does not yet show enough local context to
	 *   name the exact source byte with full confidence.
	 */
	if (len > 3 && model_completion_byte_plausible(pkt[3]))
		sample = pkt[3];
	else if (len > 4 && model_completion_byte_plausible(pkt[4]))
		sample = pkt[4];
	else if (len > 1 && model_completion_byte_plausible(pkt[1]))
		sample = pkt[1];

	if (sample == 0xff) {
		dev_dbg(m->dev, "case3 model candidate ignored len=%zu\n", len);
		return;
	}

	m->resistor_backend_ready = true;
	mikey_apply_model_sample_locked(m, sample, "case3");
}

static void mikey_handle_presence_candidate_locked(struct apple_mikeybus *m,
						   const u8 *pkt, size_t len)
{
	u8 code;
	u8 arg = 0;

	if (len < 2)
		return;

	code = pkt[1];
	if (len > 4)
		arg = pkt[4];

	m->presence_packets++;

	if (code == 0x7e) {
		m->model_modifier = arg & 1;
		dev_info(m->dev,
			 "presence event: code=0x7e arg=0x%02x modifier=%u\n",
			 arg, m->model_modifier);
		return;
	}

	if (code == 0x8a) {
		dev_info(m->dev, "presence event: code=0x8a\n");
		return;
	}

	dev_dbg(m->dev, "presence candidate code=0x%02x arg=0x%02x\n",
		code, arg);
}

static void mikey_handle_backend_packet_locked(struct apple_mikeybus *m,
					       const u8 *pkt, size_t len)
{
	u8 cls;

	if (len < 3)
		return;

	cls = pkt[2];

	switch (cls) {
	case 3:
		mikey_handle_model_completion_candidate_locked(m, pkt, len);
		break;

	case 4:
		mikey_handle_lower_packet_locked(m, pkt, len);
		break;

	case 5:
		dev_dbg(m->dev, "backend case5 len=%zu\n", len);
		break;

	case 6:
		mikey_handle_presence_candidate_locked(m, pkt, len);
		break;

	case 16:
		dev_dbg(m->dev, "backend case16 len=%zu\n", len);
		break;

	default:
		dev_dbg(m->dev, "backend packet class=%u len=%zu\n", cls, len);
		break;
	}
}

/* -------------------- active probe / poll -------------------- */

static int mikey_send_active_probe(struct apple_mikeybus *m)
{
	/*
	 * Do not write this blindly unless the lower transport framing is known.
	 *
	 * RetailOS command-object shape:
	 *   cmd[6] = 3
	 *   cmd[7] = 0x8D
	 *   cmd[8] = 3
	 *   cmd[3] = 0xFF
	 *
	 * The Linux serial transport may not accept the command object bytes
	 * directly. Keep this disabled until glass capture proves framing.
	 */
	u8 cmd[] = {
		0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x03, 0x8d, 0x03
	};
	int ret;

	if (!m->active_probe)
		return -EOPNOTSUPP;

	m->active_probe_count++;

	if (!m->serdev || !m->uart_opened)
		return -ENODEV;

	m->decomp_channel_mask_shadow |= BIT(MIKEY_CH_RESISTOR);

	ret = serdev_device_write_buf(m->serdev, cmd, sizeof(cmd));
	if (ret < 0)
		return ret;
	if (ret != sizeof(cmd))
		return -EIO;
	return 0;
}

static void mikey_poll_work(struct work_struct *work)
{
	struct apple_mikeybus *m =
		container_of(to_delayed_work(work), struct apple_mikeybus,
			     poll_work);
	int ret;
	int interval;

	mutex_lock(&m->lock);

	if (m->force_plugged || m->force_model >= 0) {
		u8 sample = (m->force_model >= 0) ?
			(u8)m->force_model : MIKEY_SAMPLE_DEFAULT;
		mikey_apply_model_sample_locked(m, sample, "force");
		goto out;
	}

	if (m->active_probe) {
		ret = mikey_send_active_probe(m);
		if (ret < 0) {
			m->active_probe_fail_count++;
			dev_dbg(m->dev, "active probe failed ret=%d\n", ret);
		}
	}

out:
	mutex_unlock(&m->lock);

	interval = poll_ms;
	if (interval < 100)
		interval = 100;
	schedule_delayed_work(&m->poll_work, msecs_to_jiffies(interval));
}

/* -------------------- hex inject parser -------------------- */

static int mikey_parse_hex_bytes(const char *buf, size_t count,
				 u8 *out, size_t max, size_t *out_len)
{
	const char *p = buf;
	const char *end = buf + count;
	size_t n = 0;

	while (p < end) {
		u8 byte;
		char tok[32];
		size_t i = 0;

		while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' ||
				   *p == '\r' || *p == ','))
			p++;
		if (p >= end)
			break;

		while (p < end && *p != ' ' && *p != '\t' && *p != '\n' &&
		       *p != '\r' && *p != ',' && i + 1 < sizeof(tok))
			tok[i++] = *p++;
		tok[i] = '\0';
		if (!i)
			break;

		/* Inject ABI is hex bytes (guide: "06 70 00 aa 55 33"). */
		if (kstrtou8(tok, 16, &byte))
			return -EINVAL;
		if (n >= max)
			return -EINVAL;
		out[n++] = byte;
	}

	*out_len = n;
	return n ? 0 : -EINVAL;
}

/* -------------------- serdev -------------------- */

static size_t mikey_serdev_receive(struct serdev_device *serdev,
				   const u8 *data, size_t count)
{
	struct apple_mikeybus *m = serdev_device_get_drvdata(serdev);
	size_t i;

	if (!m)
		return count;

	mutex_lock(&m->lock);
	for (i = 0; i < count; i++)
		mikey_rx_byte_locked(m, data[i]);
	mutex_unlock(&m->lock);

	dev_dbg(&serdev->dev, "Mikey RX %zu: %*ph\n", count,
		(int)min(count, (size_t)16), data);

	return count;
}

static const struct serdev_device_ops mikey_serdev_ops = {
	.receive_buf = mikey_serdev_receive,
	.write_wakeup = serdev_device_write_wakeup,
};

/* -------------------- sysfs -------------------- */

static ssize_t info_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&m->lock);
	n = sysfs_emit(buf,
		       "N31 MikeyBus\n"
		       "status=YELLOW\n"
		       "read_path=channel4 command 9/0x71\n"
		       "resistor_path=channel3 command 3/0x8D\n"
		       "auto_report=%d\n"
		       "active_probe=%d\n"
		       "plugged=%d\n"
		       "model=0x%02x %s\n"
		       "model_sample=0x%02x\n"
		       "model_modifier=%u\n"
		       "force_plugged=%d\n"
		       "force_model=%d\n"
		       "resistor_backend_ready=%d\n"
		       "accept_case3_model=%d\n"
		       "baud=%d\n"
		       "serdev=%s\n"
		       "uart_opened=%d\n"
		       "pinmux_on=%d\n"
		       "button_decode=raw-only\n",
		       m->auto_report, m->active_probe, m->plugged,
		       m->model, mikey_model_name(m->model),
		       m->model_sample, m->model_modifier,
		       m->force_plugged, m->force_model,
		       m->resistor_backend_ready, accept_case3_model,
		       m->baud, m->serdev ? "yes" : "no",
		       m->uart_opened, m->pinmux_on);
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(info);

static ssize_t plugged_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	bool plugged;

	mutex_lock(&m->lock);
	plugged = m->plugged;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "%d\n", plugged);
}
static DEVICE_ATTR_RO(plugged);

static ssize_t model_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 model;

	mutex_lock(&m->lock);
	model = m->model;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "0x%02x\n", model);
}
static DEVICE_ATTR_RO(model);

static ssize_t model_name_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 model;

	mutex_lock(&m->lock);
	model = m->model;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "%s\n", mikey_model_name(model));
}
static DEVICE_ATTR_RO(model_name);

static ssize_t model_sample_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 sample;

	mutex_lock(&m->lock);
	sample = m->model_sample;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "0x%02x\n", sample);
}
static DEVICE_ATTR_RO(model_sample);

static ssize_t model_sample_inject_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val > 0xff)
		return -EINVAL;

	mutex_lock(&m->lock);
	mikey_apply_model_sample_locked(m, (u8)val, "sample_inject");
	mutex_unlock(&m->lock);

	return count;
}
static DEVICE_ATTR_WO(model_sample_inject);

static ssize_t model_modifier_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 mod;

	mutex_lock(&m->lock);
	mod = m->model_modifier;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "%u\n", mod);
}

static ssize_t model_modifier_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val > 1)
		return -EINVAL;

	mutex_lock(&m->lock);
	m->model_modifier = (u8)val;
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(model_modifier);

static ssize_t force_plugged_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m->force_plugged);
}

static ssize_t force_plugged_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	bool v;
	int ret;

	ret = kstrtobool(buf, &v);
	if (ret)
		return ret;

	mutex_lock(&m->lock);
	m->force_plugged = v;
	force_plugged = v;
	if (v) {
		u8 sample = (m->force_model >= 0) ?
			(u8)m->force_model : MIKEY_SAMPLE_DEFAULT;
		mikey_apply_model_sample_locked(m, sample, "force");
	} else if (m->force_model < 0) {
		mikey_apply_model_sample_locked(m, m->model_sample, "force_clear");
	}
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(force_plugged);

static ssize_t force_model_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m->force_model);
}

static ssize_t force_model_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	int val;
	int ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;
	if (val < -1 || val > 0xff)
		return -EINVAL;

	mutex_lock(&m->lock);
	m->force_model = val;
	force_model = val;
	if (m->force_plugged || val >= 0) {
		u8 sample = (val >= 0) ? (u8)val : MIKEY_SAMPLE_DEFAULT;

		mikey_apply_model_sample_locked(m, sample, "force");
	}
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(force_model);

static ssize_t auto_report_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m->auto_report);
}

static ssize_t auto_report_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	bool v;
	int ret;

	ret = kstrtobool(buf, &v);
	if (ret)
		return ret;

	mutex_lock(&m->lock);
	m->auto_report = v;
	auto_report = v;
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(auto_report);

static ssize_t active_probe_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m->active_probe);
}

static ssize_t active_probe_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	bool v;
	int ret;

	ret = kstrtobool(buf, &v);
	if (ret)
		return ret;

	mutex_lock(&m->lock);
	m->active_probe = v;
	active_probe = v;
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(active_probe);

static ssize_t resistor_backend_ready_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	bool ready;

	mutex_lock(&m->lock);
	ready = m->resistor_backend_ready;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "%d\n", ready);
}
static DEVICE_ATTR_RO(resistor_backend_ready);

static ssize_t decomp_channel_mask_shadow_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u32 mask;

	mutex_lock(&m->lock);
	mask = m->decomp_channel_mask_shadow;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "0x%08x\n", mask);
}
static DEVICE_ATTR_RO(decomp_channel_mask_shadow);

static ssize_t rx_status_shadow_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 st;

	mutex_lock(&m->lock);
	st = m->rx_status_shadow;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "0x%02x\n", st);
}
static DEVICE_ATTR_RO(rx_status_shadow);

static ssize_t remote_raw_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	size_t n;

	mutex_lock(&m->lock);
	n = mikey_ring_dump_hex(&m->remote_raw, buf, PAGE_SIZE);
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(remote_raw);

/*
 * Everything needed to pin down the bit order: the live bitmap, the map in
 * force, and whether any of it is moving. Hold a button, read this.
 */
static ssize_t buttons_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int i;
	size_t n = 0;

	mutex_lock(&m->lock);
	n += scnprintf(buf + n, PAGE_SIZE - n,
		       "held=0x%02x remote_bytes=%u events=%u all_up=%u\n",
		       m->last_buttons, m->remote_bytes, m->button_events,
		       m->all_up_events);
	for (i = 0; i < MIKEY_BUTTON_BITS; i++)
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "bit%u keycode=%d held=%d\n",
			       i, button_map[i],
			       !!(m->last_buttons & BIT(i)));
	n += scnprintf(buf + n, PAGE_SIZE - n,
		       "note: bit order is not attested by the decomp; confirm here\n");
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(buttons);

static ssize_t rx_raw_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	size_t n;

	mutex_lock(&m->lock);
	n = mikey_ring_dump_hex(&m->rx_raw, buf, PAGE_SIZE);
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(rx_raw);

static ssize_t rx_task_stream_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	size_t n;

	mutex_lock(&m->lock);
	n = mikey_ring_dump_hex(&m->rx_task_stream, buf, PAGE_SIZE);
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(rx_task_stream);

static ssize_t rx_stats_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&m->lock);
	n = sysfs_emit(buf,
		       "rx_bytes=%u\n"
		       "aa_stuff_count=%u\n"
		       "lower_packets=%u\n"
		       "lower_rx70_packets=%u\n"
		       "lower_status_packets=%u\n"
		       "presence_packets=%u\n"
		       "model_changes=%u\n"
		       "plug_events=%u\n"
		       "unplug_events=%u\n"
		       "active_probe_count=%u\n"
		       "active_probe_fail_count=%u\n"
		       "rx_raw_drops=%u\n"
		       "rx_task_drops=%u\n",
		       m->rx_bytes, m->aa_stuff_count,
		       m->lower_packets, m->lower_rx70_packets,
		       m->lower_status_packets, m->presence_packets,
		       m->model_changes, m->plug_events, m->unplug_events,
		       m->active_probe_count, m->active_probe_fail_count,
		       m->rx_raw.drops, m->rx_task_stream.drops);
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(rx_stats);

static ssize_t lower_packet_inject_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 pkt[MIKEY_INJECT_MAX];
	size_t len;
	int ret;

	ret = mikey_parse_hex_bytes(buf, count, pkt, sizeof(pkt), &len);
	if (ret)
		return ret;

	mutex_lock(&m->lock);
	mikey_handle_lower_packet_locked(m, pkt, len);
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_WO(lower_packet_inject);

static ssize_t backend_packet_inject_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 pkt[MIKEY_INJECT_MAX];
	size_t len;
	int ret;

	ret = mikey_parse_hex_bytes(buf, count, pkt, sizeof(pkt), &len);
	if (ret)
		return ret;

	mutex_lock(&m->lock);
	mikey_handle_backend_packet_locked(m, pkt, len);
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_WO(backend_packet_inject);

static struct attribute *mikey_attrs[] = {
	&dev_attr_info.attr,
	&dev_attr_plugged.attr,
	&dev_attr_model.attr,
	&dev_attr_model_name.attr,
	&dev_attr_model_sample.attr,
	&dev_attr_model_sample_inject.attr,
	&dev_attr_model_modifier.attr,
	&dev_attr_force_plugged.attr,
	&dev_attr_force_model.attr,
	&dev_attr_auto_report.attr,
	&dev_attr_active_probe.attr,
	&dev_attr_resistor_backend_ready.attr,
	&dev_attr_decomp_channel_mask_shadow.attr,
	&dev_attr_remote_raw.attr,
	&dev_attr_buttons.attr,
	&dev_attr_rx_status_shadow.attr,
	&dev_attr_rx_raw.attr,
	&dev_attr_rx_task_stream.attr,
	&dev_attr_rx_stats.attr,
	&dev_attr_lower_packet_inject.attr,
	&dev_attr_backend_packet_inject.attr,
	NULL,
};

static const struct attribute_group mikey_attr_group = {
	.attrs = mikey_attrs,
};

static const struct attribute_group *mikey_groups[] = {
	&mikey_attr_group,
	NULL,
};

/*
 * The remote is an input device, so publish it as one.
 *
 * Without this the driver decoded a button stream into nothing: no evdev
 * node, so no key ever reached userspace no matter how well the packets
 * parsed. devm-managed, so teardown follows the device.
 */
static int mikey_register_input(struct apple_mikeybus *m)
{
	struct input_dev *in;
	unsigned int i;
	int ret;

	in = devm_input_allocate_device(m->dev);
	if (!in)
		return -ENOMEM;

	in->name = "Apple MikeyBus Remote";
	in->phys = "mikeybus/input0";
	in->id.bustype = BUS_HOST;

	for (i = 0; i < MIKEY_BUTTON_BITS; i++)
		if (button_map[i])
			input_set_capability(in, EV_KEY, button_map[i]);

	ret = input_register_device(in);
	if (ret)
		return ret;

	m->input = in;
	return 0;
}

static int mikey_create_sysfs(struct apple_mikeybus *m)
{
	return sysfs_create_groups(&m->dev->kobj, mikey_groups);
}

static void mikey_remove_sysfs(struct apple_mikeybus *m)
{
	sysfs_remove_groups(&m->dev->kobj, mikey_groups);
}

/* -------------------- bind / unbind -------------------- */

static int mikey_bind(struct device *dev, struct serdev_device *serdev)
{
	struct apple_mikeybus *m;
	int ret;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->dev = dev;
	m->serdev = serdev;
	m->baud = baud;
	m->auto_report = auto_report;
	m->active_probe = active_probe;
	/*
	 * The module parameter, OR the device tree. apple,force-plugged has
	 * been in the N31 dts since the port began and was read by nothing:
	 * this driver had no property reads at all, so the board said "assume
	 * a headset is plugged in until the resistor backend exists" and
	 * nothing was listening.
	 *
	 * Ported from n31/wip-local-backup-20260827 (8efa5c0e11ec). The rest
	 * of that commit is superseded: its codec half stopped jack detection
	 * returning -ENODEV, and this tree has since removed codec headset
	 * detection altogether.
	 */
	m->force_plugged = force_plugged ||
			   device_property_read_bool(dev, "apple,force-plugged");
	m->force_model = force_model;

	mutex_init(&m->lock);
	INIT_DELAYED_WORK(&m->poll_work, mikey_poll_work);

	m->model_sample = MIKEY_SAMPLE_DEFAULT;
	m->model = MIKEY_SAMPLE_DEFAULT;
	m->plugged = false;
	m->last_reported_plugged = false;
	m->last_reported_model = MIKEY_SAMPLE_DEFAULT;

	m->gpio = devm_ioremap(dev, GPIO_PHYS, 0x200);
	m->gpiocmd = devm_ioremap(dev, GPIOCMD_PHYS, 4);

	dev_set_drvdata(dev, m);

	if (serdev) {
		serdev_device_set_drvdata(serdev, m);
		serdev_device_set_client_ops(serdev, &mikey_serdev_ops);

		ret = serdev_device_open(serdev);
		if (ret)
			return ret;

		serdev_device_set_baudrate(serdev, m->baud);
		serdev_device_set_flow_control(serdev, false);
		m->uart_opened = true;
		m->decomp_channel_mask_shadow |= BIT(MIKEY_CH_READ);
	}

	mikey_pinmux_uart(m, true);

	/*
	 * Not fatal: the bus is still useful for headset detection and
	 * tracing even if the input node cannot be created.
	 */
	ret = mikey_register_input(m);
	if (ret)
		dev_warn(m->dev, "no input device: %d\n", ret);

	ret = mikey_create_sysfs(m);
	if (ret) {
		if (serdev && m->uart_opened) {
			serdev_device_close(serdev);
			m->uart_opened = false;
		}
		return ret;
	}

	mutex_lock(&mikeybus_singleton_lock);
	mikeybus_singleton = m;
	mutex_unlock(&mikeybus_singleton_lock);

	dev_info(dev,
		 "N31 MikeyBus loaded (%s): auto_report=%d active_probe=%d force_plugged=%d force_model=%d baud=%d\n",
		 serdev ? "serdev" : "platform",
		 m->auto_report, m->active_probe,
		 m->force_plugged, m->force_model, m->baud);

	schedule_delayed_work(&m->poll_work, msecs_to_jiffies(100));
	return 0;
}

static void mikey_unbind(struct device *dev)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	if (!m)
		return;

	cancel_delayed_work_sync(&m->poll_work);

	mutex_lock(&mikeybus_singleton_lock);
	if (mikeybus_singleton == m)
		mikeybus_singleton = NULL;
	mutex_unlock(&mikeybus_singleton_lock);

	mikey_remove_sysfs(m);

	mutex_lock(&m->lock);
	if (m->uart_opened && m->serdev) {
		serdev_device_close(m->serdev);
		m->uart_opened = false;
	}
	mikey_pinmux_uart(m, false);
	mutex_unlock(&m->lock);
}

/* -------------------- platform fallback -------------------- */

static void mikey_ensure_plat(struct work_struct *work)
{
	struct device_node *uart_np, *mikey_np = NULL;
	int ret;

	(void)work;
	if (mikeybus_singleton)
		return;

	/*
	 * Prefer serdev when uart2 is okay in DT. Only instantiate the
	 * platform fallback when uart2 is disabled / missing so exports
	 * and sysfs still exist for CS42 bring-up.
	 */
	uart_np = of_find_node_by_path("/soc/serial@3dc00000");
	if (uart_np && of_device_is_available(uart_np)) {
		pr_info("apple-mikeybus: uart2 okay in DT — waiting on serdev\n");
		of_node_put(uart_np);
		return;
	}
	if (uart_np)
		mikey_np = of_get_child_by_name(uart_np, "mikeybus");

	mikey_plat_pdev = platform_device_alloc("apple-mikeybus-plat",
						PLATFORM_DEVID_NONE);
	if (!mikey_plat_pdev)
		goto out;
	if (mikey_np)
		mikey_plat_pdev->dev.of_node = of_node_get(mikey_np);
	ret = platform_device_add(mikey_plat_pdev);
	if (ret) {
		pr_warn("apple-mikeybus: plat add %d\n", ret);
		platform_device_put(mikey_plat_pdev);
		mikey_plat_pdev = NULL;
	}
out:
	if (mikey_np)
		of_node_put(mikey_np);
	if (uart_np)
		of_node_put(uart_np);
}

static int mikey_serdev_probe(struct serdev_device *serdev)
{
	return mikey_bind(&serdev->dev, serdev);
}

static void mikey_serdev_remove(struct serdev_device *serdev)
{
	mikey_unbind(&serdev->dev);
}

static const struct of_device_id mikey_serdev_of_match[] = {
	{ .compatible = "apple,mikeybus" },
	{ .compatible = "apple,n31-mikeybus" },
	{ }
};
MODULE_DEVICE_TABLE(of, mikey_serdev_of_match);

static struct serdev_device_driver mikey_serdev_driver = {
	.probe = mikey_serdev_probe,
	.remove = mikey_serdev_remove,
	.driver = {
		.name = "apple-mikeybus",
		.of_match_table = mikey_serdev_of_match,
	},
};

static int mikey_plat_probe(struct platform_device *pdev)
{
	return mikey_bind(&pdev->dev, NULL);
}

static void mikey_plat_remove(struct platform_device *pdev)
{
	mikey_unbind(&pdev->dev);
}

static struct platform_driver mikey_plat_driver = {
	.probe = mikey_plat_probe,
	.remove = mikey_plat_remove,
	.driver = {
		.name = "apple-mikeybus-plat",
	},
};

static int __init mikey_init(void)
{
	int ret;

	ret = serdev_device_driver_register(&mikey_serdev_driver);
	if (ret)
		return ret;
	ret = platform_driver_register(&mikey_plat_driver);
	if (ret) {
		serdev_device_driver_unregister(&mikey_serdev_driver);
		return ret;
	}
	schedule_work(&mikey_plat_work);
	return 0;
}

static void __exit mikey_exit(void)
{
	cancel_work_sync(&mikey_plat_work);
	if (mikey_plat_pdev) {
		platform_device_unregister(mikey_plat_pdev);
		mikey_plat_pdev = NULL;
	}
	platform_driver_unregister(&mikey_plat_driver);
	serdev_device_driver_unregister(&mikey_serdev_driver);
}

module_init(mikey_init);
module_exit(mikey_exit);

MODULE_DESCRIPTION("Apple N31 MikeyBus (serdev RX, model/jack state, CS42 exports)");
MODULE_AUTHOR("FreeMyiPod");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:apple-mikeybus-plat");
