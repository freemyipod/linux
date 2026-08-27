// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple MikeyBus — N31 headset / remote (UART2 @ 0x3DC00000)
 *
 * OSOS decomp (finish-line — grounded):
 *   Read open:     sub_570BA8 → cmd 9/0x71/channel4 (bit4 @ 0x8A9239C)
 *   RX producer:   sub_500ECC — packet type 0x70 appends to 1024B ring
 *                  (@0x8AE5298, index @0x8AE5294, wrap &0x3FF)
 *   ReadTask:      sub_2542F0 drains ring via sub_570C1C/sub_150A38;
 *                  on byte 0xAA appends synthetic 0x01 (no button decode)
 *   Resistor:      sub_410DB0 — cmd 3/0x8D/channel3; wait timeout 100;
 *                  result @0x8A92444 (sample 15→100 if flag clear);
 *                  NOT GPIO66/67 DIN polling; NOT raw UART decode
 *   ResistorTask:  0x00254382 — measure → sub_41F0D8(0x80, sample, 1)
 *   Status pkts:   0x76 / 0x8A → v=pkt[3]; bit4→0, bit5→128
 *
 * Linux layers:
 *   1) RX byte trace — raw + osos-shaped (0xAA→+0x01); no button decode
 *   2) Resistor/model — force_plugged default; measure = -EOPNOTSUPP
 *      until command backend mapped; never flap jack on unknown
 *   3) Event/jack — ALSA/export may use force_plugged / force_model only
 *
 * GPIO 66/67 = UART2 pad mux only (sub_5714EE case 2). Not resistor detect.
 */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#define GPIO_PHYS		0x3cf00000ul
#define GPIOCMD_PHYS		0x3cf001e0ul

/* UART2 pad mux only — NOT DIN / resistor detect. */
#define MIKEY_GPIO_TX		0x42u	/* 66 — UART TX mux */
#define MIKEY_GPIO_RX		0x43u	/* 67 — UART RX mux */

#define MIKEY_MODEL_OPEN	0x0Bu
#define MIKEY_MODEL_A18		0x01u

/* OSOS RX ring is 1024 bytes (index wrap & 0x3FF). */
#define MIKEY_RX_RING_SIZE	1024

/*
 * force_plugged until command 3/0x8D resistor backend is mapped.
 * Do not invent DIN thresholds; do not treat missing RX as unplug.
 */
static bool force_plugged_param = true;
module_param_named(force_plugged, force_plugged_param, bool, 0644);
MODULE_PARM_DESC(force_plugged,
		 "Force jack present until resistor cmd backend (default 1)");

static u8 force_model_param = MIKEY_MODEL_A18;
module_param_named(force_model, force_model_param, byte, 0644);
MODULE_PARM_DESC(force_model,
		 "Model reported under force_plugged (default 0x01 A18)");

static bool uart_auto_open = true;
module_param(uart_auto_open, bool, 0644);
MODULE_PARM_DESC(uart_auto_open,
		 "Open UART2 for raw RX trace (default 1)");

/*
 * ResistorTask loop period placeholder. OSOS: wait timeout 100, default
 * sample 100, fail-path delay 10 — none proven as Linux ms period.
 */
static unsigned int resistor_period_ms = 100;
module_param(resistor_period_ms, uint, 0644);
MODULE_PARM_DESC(resistor_period_ms,
		 "Resistor worker period (NOT proven ms; 0=off)");

static bool instantiate_uart2;
module_param(instantiate_uart2, bool, 0444);
MODULE_PARM_DESC(instantiate_uart2,
		 "ignored; use DT uart2 okay");

struct mikey_rx_ring {
	u8 buf[MIKEY_RX_RING_SIZE];
	unsigned int head;	/* next write */
	unsigned int count;
};

struct apple_mikeybus {
	struct device *dev;
	struct serdev_device *serdev;
	void __iomem *gpio;
	void __iomem *gpiocmd;
	struct mutex lock;

	u8 model;
	u8 force_model;
	bool force_plugged;
	bool pinmux_on;
	u32 baud;

	/* Layer 1: dual RX rings (Linux shadows of OSOS ring). */
	struct mikey_rx_ring raw_rx;	/* exact serdev bytes */
	struct mikey_rx_ring osos_rx;	/* ReadTask-shaped (+0x01 after 0xAA) */
	u32 rx_bytes;
	u8 rx_last[64];
	unsigned int rx_last_len;

	/*
	 * Linux shadows of OSOS globals (NOT literal addresses):
	 *   rx_status        ↔ 0x892A2C8-ish status from 0x76/0x8A
	 *   channel_mask     ↔ 0x8A9239C feature bits (4=read, 3=resistor)
	 *   model_sample     ↔ last sub_410DB0 sample (or forced)
	 */
	u8 rx_status;			/* 0 / 128 / unchanged */
	u32 channel_mask_shadow;	/* bits we "would" enable */
	u8 model_sample;
	bool resistor_backend_ready;	/* false until cmd 3/0x8D mapped */

	bool uart_opened;
	struct delayed_work uart_open_work;
	struct delayed_work resistor_work;
	bool resistor_active;
	u32 resistor_ticks;
};

static struct apple_mikeybus *mikeybus_singleton;
static DEFINE_MUTEX(mikeybus_singleton_lock);
static struct platform_device *mikey_plat_pdev;
static void mikey_ensure_plat(struct work_struct *work);
static DECLARE_WORK(mikey_plat_work, mikey_ensure_plat);

static const char *mikey_model_name(u8 model)
{
	switch (model) {
	case 1:  return "A18";
	case 2:  return "B18";
	case 3:  return "A62";
	case 4:  return "B15";
	case 5:  return "A36";
	case 6:  return "Apple noise occluding";
	case 7:  return "mfg noise occluding";
	case 8:  return "mfg noise occluding w/ mic";
	case 9:  return "mfg std";
	case 0xA: return "mfg std w/ mic";
	case 0xB: return "open circuit";
	case 0xD: return "B60f";
	case 0xE: return "B60g";
	case 0xF: return "B149";
	case 0x10: return "B187";
	default: return "inscrutable";
	}
}

static bool mikey_headset_has_remote(u8 model)
{
	switch (model) {
	case 2: case 4: case 5: case 6: case 7: case 8: case 9:
	case 0xA: case 0xD: case 0xE: case 0x10:
		return true;
	default:
		return false;
	}
}

static bool mikey_headset_ready_locked(struct apple_mikeybus *m)
{
	if (m->force_plugged)
		return true;
	if (m->model == MIKEY_MODEL_OPEN)
		return false;
	return true;
}

/*
 * Jack present: force_plugged wins. Unknown / open must NOT flap to
 * unplugged (false PLUG lesson). Only clear when measure proves open.
 */
static bool mikey_jack_present_locked(struct apple_mikeybus *m)
{
	if (m->force_plugged)
		return true;
	if (m->model == 0 || m->model == MIKEY_MODEL_OPEN)
		return false;
	return true;
}

int apple_mikeybus_jack_present(void)
{
	int ret;

	mutex_lock(&mikeybus_singleton_lock);
	if (!mikeybus_singleton) {
		mutex_unlock(&mikeybus_singleton_lock);
		return -ENODEV;
	}
	mutex_lock(&mikeybus_singleton->lock);
	ret = mikey_jack_present_locked(mikeybus_singleton) ? 1 : 0;
	mutex_unlock(&mikeybus_singleton->lock);
	mutex_unlock(&mikeybus_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_mikeybus_jack_present);

int apple_mikeybus_headset_ready(void)
{
	int ret;

	mutex_lock(&mikeybus_singleton_lock);
	if (!mikeybus_singleton) {
		mutex_unlock(&mikeybus_singleton_lock);
		return -ENODEV;
	}
	mutex_lock(&mikeybus_singleton->lock);
	ret = mikey_headset_ready_locked(mikeybus_singleton) ? 1 : 0;
	mutex_unlock(&mikeybus_singleton->lock);
	mutex_unlock(&mikeybus_singleton_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_mikeybus_headset_ready);

static void mikey_ring_put(struct mikey_rx_ring *r, u8 b)
{
	r->buf[r->head] = b;
	r->head = (r->head + 1) & (MIKEY_RX_RING_SIZE - 1);
	if (r->count < MIKEY_RX_RING_SIZE)
		r->count++;
}

static void mikey_ring_reset(struct mikey_rx_ring *r)
{
	r->head = 0;
	r->count = 0;
}

/*
 * Snapshot newest bytes into @dst (up to @max), oldest→newest order among
 * the retained window.
 */
static unsigned int mikey_ring_snapshot(const struct mikey_rx_ring *r,
					u8 *dst, unsigned int max)
{
	unsigned int n, i, start;

	n = min(r->count, max);
	if (!n)
		return 0;
	start = (r->head - n) & (MIKEY_RX_RING_SIZE - 1);
	for (i = 0; i < n; i++)
		dst[i] = r->buf[(start + i) & (MIKEY_RX_RING_SIZE - 1)];
	return n;
}

/* UART pad mux only — never used as DIN sample / resistor path. */
static void mikey_gpiocmd(struct apple_mikeybus *m, u8 gpio, u8 mode)
{
	u32 bank = gpio >> 3;
	u32 pin = gpio & 7;

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

/*
 * ReadTask-shaped append (sub_2542F0): put byte; if 0xAA also put 0x01.
 * Raw ring keeps exact wire bytes separately.
 */
static void mikey_rx_append_byte(struct apple_mikeybus *m, u8 b)
{
	mikey_ring_put(&m->raw_rx, b);
	mikey_ring_put(&m->osos_rx, b);
	if (b == 0xaa)
		mikey_ring_put(&m->osos_rx, 0x01);
}

/*
 * Lower-packet dispatcher (sub_500ECC shape). Only call with proven
 * packet-framed envelopes — never feed raw serdev bytes here.
 */
static void mikey_lower_packet_rx(struct apple_mikeybus *m,
				  const u8 *pkt, size_t len)
{
	u8 type, v;
	size_t i, count;

	if (len < 3)
		return;

	type = pkt[1];
	switch (type) {
	case 0x70:
		/* payload = packet+3; count = packet[0]-3 (OSOS). */
		count = pkt[0];
		if (count < 3 || count > len)
			count = len;
		count -= 3;
		for (i = 0; i < count; i++)
			mikey_rx_append_byte(m, pkt[3 + i]);
		break;
	case 0x76:
	case 0x8a:
		/* sub_18911C / sub_182AFC status shadow. */
		v = pkt[3];
		if (v & 0x10)
			m->rx_status = 0;
		else if (v & 0x20)
			m->rx_status = 128;
		break;
	default:
		break;
	}
}

/*
 * Command-backend resistor measure (sub_410DB0). Not implemented until
 * channel3 / cmd 3/0x8D / wait@0x8A92448 are mapped to Linux.
 */
static int mikey_measure_model(struct apple_mikeybus *m, u8 *sample)
{
	(void)m;
	(void)sample;
	return -EOPNOTSUPP;
}

static int mikey_uart_open_locked(struct apple_mikeybus *m)
{
	int ret;

	if (m->uart_opened)
		return 0;
	if (!m->serdev)
		return -ENODEV;
	ret = serdev_device_open(m->serdev);
	if (ret)
		return ret;
	serdev_device_set_baudrate(m->serdev, m->baud);
	serdev_device_set_flow_control(m->serdev, false);
	m->uart_opened = true;
	/* Shadow: Read open enables channel bit 4. */
	m->channel_mask_shadow |= BIT(4);
	dev_info(m->dev,
		 "Mikey UART opened baud=%u (raw RX trace; no button decode; "
		 "channel4 shadow set)\n",
		 m->baud);
	return 0;
}

static void mikey_uart_close_locked(struct apple_mikeybus *m)
{
	if (!m->uart_opened || !m->serdev)
		return;
	serdev_device_close(m->serdev);
	m->uart_opened = false;
	m->channel_mask_shadow &= ~BIT(4);
	dev_info(m->dev, "Mikey UART closed\n");
}

static void mikey_uart_open_workfn(struct work_struct *work)
{
	struct apple_mikeybus *m =
		container_of(work, struct apple_mikeybus, uart_open_work.work);
	int ret;

	mutex_lock(&m->lock);
	ret = mikey_uart_open_locked(m);
	mutex_unlock(&m->lock);
	if (ret)
		dev_warn(m->dev, "Mikey UART auto-open failed: %d\n", ret);
}

/*
 * ResistorTask-shaped worker (0x00254382):
 *   force_plugged → keep force_model, stay plugged, return
 *   measure EOPNOTSUPP → unknown, do NOT flap jack
 *   on change → update model_sample / model (when backend ready)
 */
static void mikey_resistor_workfn(struct work_struct *work)
{
	struct apple_mikeybus *m =
		container_of(work, struct apple_mikeybus, resistor_work.work);
	u8 sample = 0;
	int ret;

	if (!m->resistor_active || !resistor_period_ms)
		return;

	mutex_lock(&m->lock);
	m->resistor_ticks++;

	if (m->force_plugged) {
		m->model = m->force_model ? m->force_model : MIKEY_MODEL_A18;
		m->model_sample = m->model;
		if (m->resistor_ticks == 1)
			dev_info(m->dev,
				 "Mikey resistor: force_plugged model=0x%02x "
				 "(%s); backend_ready=%d\n",
				 m->model, mikey_model_name(m->model),
				 m->resistor_backend_ready);
		goto resched;
	}

	ret = mikey_measure_model(m, &sample);
	if (ret == -EOPNOTSUPP) {
		/*
		 * Backend not mapped. Report unknown sample shadow only;
		 * do not clear plugged / do not set OPEN from lack of RX.
		 */
		if (m->resistor_ticks == 1)
			dev_info(m->dev,
				 "Mikey resistor: measure -EOPNOTSUPP "
				 "(cmd 3/0x8D/ch3 not mapped); jack unchanged\n");
		goto resched;
	}
	if (ret) {
		/* OSOS fail path uses sample=100 then delay 10 — shadow only. */
		sample = 100;
		m->model_sample = sample;
		goto resched;
	}

	m->channel_mask_shadow |= BIT(3);
	if (sample != m->model_sample) {
		m->model_sample = sample;
		m->model = sample;
		dev_info(m->dev,
			 "Mikey model_sample=%u (0x%02x %s) via measure\n",
			 sample, sample, mikey_model_name(sample));
	}

resched:
	mutex_unlock(&m->lock);
	if (m->resistor_active && resistor_period_ms)
		schedule_delayed_work(&m->resistor_work,
				      msecs_to_jiffies(resistor_period_ms));
}

static void mikey_ensure_plat(struct work_struct *work)
{
	struct device_node *uart_np, *mikey_np = NULL;
	int ret;

	(void)work;
	if (mikeybus_singleton)
		return;

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

/* -------------------- sysfs (Linux shadows) -------------------- */

static ssize_t model_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%02x %s\n", m->model, mikey_model_name(m->model));
}

static ssize_t model_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int v;
	int ret;

	ret = kstrtouint(buf, 0, &v);
	if (ret || v > 0xff)
		return -EINVAL;
	mutex_lock(&m->lock);
	m->model = (u8)v;
	m->model_sample = (u8)v;
	dev_info(dev, "Mikey model set 0x%02x (%s) via sysfs\n",
		 m->model, mikey_model_name(m->model));
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(model);

static ssize_t force_model_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%02x %s\n", m->force_model,
			  mikey_model_name(m->force_model));
}

static ssize_t force_model_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int v;
	int ret;

	ret = kstrtouint(buf, 0, &v);
	if (ret || v > 0xff)
		return -EINVAL;
	mutex_lock(&m->lock);
	m->force_model = (u8)v;
	if (m->force_plugged) {
		m->model = m->force_model;
		m->model_sample = m->force_model;
	}
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(force_model);

static ssize_t model_sample_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->model_sample);
}
static DEVICE_ATTR_RO(model_sample);

static ssize_t plugged_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	int p;

	mutex_lock(&m->lock);
	p = mikey_jack_present_locked(m);
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "%d\n", p);
}
static DEVICE_ATTR_RO(plugged);

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
	unsigned int v;
	int ret;

	ret = kstrtouint(buf, 0, &v);
	if (ret)
		return ret;
	mutex_lock(&m->lock);
	m->force_plugged = !!v;
	if (m->force_plugged) {
		m->model = m->force_model ? m->force_model : MIKEY_MODEL_A18;
		m->model_sample = m->model;
	}
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(force_plugged);

static ssize_t baud_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->baud);
}

static ssize_t baud_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int v;
	int ret;

	ret = kstrtouint(buf, 0, &v);
	if (ret || !v)
		return -EINVAL;
	mutex_lock(&m->lock);
	m->baud = v;
	if (m->serdev && m->uart_opened)
		serdev_device_set_baudrate(m->serdev, v);
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(baud);

static ssize_t rx_bytes_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->rx_bytes);
}
static DEVICE_ATTR_RO(rx_bytes);

static ssize_t rx_raw_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 tmp[64];
	unsigned int n;
	ssize_t out;

	mutex_lock(&m->lock);
	n = mikey_ring_snapshot(&m->raw_rx, tmp, sizeof(tmp));
	out = sysfs_emit(buf, "count=%u last=%*ph\n", m->raw_rx.count, n, tmp);
	mutex_unlock(&m->lock);
	return out;
}
static DEVICE_ATTR_RO(rx_raw);

static ssize_t rx_task_stream_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 tmp[64];
	unsigned int n;
	ssize_t out;

	mutex_lock(&m->lock);
	n = mikey_ring_snapshot(&m->osos_rx, tmp, sizeof(tmp));
	out = sysfs_emit(buf,
			 "count=%u (ReadTask-shaped; 0xAA→+0x01) last=%*ph\n",
			 m->osos_rx.count, n, tmp);
	mutex_unlock(&m->lock);
	return out;
}
static DEVICE_ATTR_RO(rx_task_stream);

static ssize_t rx_status_892A2C8_shadow_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->rx_status);
}
static DEVICE_ATTR_RO(rx_status_892A2C8_shadow);

static ssize_t decomp_channel_mask_shadow_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf,
			  "0x%08x (bit3=resistor cmd shadow, bit4=read open)\n",
			  m->channel_mask_shadow);
}
static DEVICE_ATTR_RO(decomp_channel_mask_shadow);

static ssize_t resistor_backend_ready_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m->resistor_backend_ready);
}
static DEVICE_ATTR_RO(resistor_backend_ready);

/*
 * Debug inject of a framed lower packet (hex bytes). Does NOT accept
 * unframed serdev streams — operator must supply OSOS envelopes.
 * Format: echo "len type ..." with decimal/hex tokens, e.g.
 *   echo "6 0x70 0 aa 01 02" > lower_packet_inject
 * First token is OSOS packet[0] length field.
 */
static ssize_t lower_packet_inject_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 pkt[64];
	unsigned int vals[64];
	int n = 0, i;
	const char *p = buf;

	while (n < 64 && *p) {
		unsigned int v;
		int matched;

		while (*p == ' ' || *p == '\t' || *p == '\n')
			p++;
		if (!*p)
			break;
		matched = sscanf(p, "%i%n", &v, &i);
		if (matched < 1)
			break;
		vals[n++] = v & 0xff;
		p += i;
	}
	if (n < 3)
		return -EINVAL;
	for (i = 0; i < n; i++)
		pkt[i] = (u8)vals[i];

	mutex_lock(&m->lock);
	mikey_lower_packet_rx(m, pkt, n);
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_WO(lower_packet_inject);

static ssize_t uart_open_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m->uart_opened);
}

static ssize_t uart_open_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int v;
	int ret;

	ret = kstrtouint(buf, 0, &v);
	if (ret)
		return ret;
	mutex_lock(&m->lock);
	if (v)
		ret = mikey_uart_open_locked(m);
	else {
		mikey_uart_close_locked(m);
		ret = 0;
	}
	mutex_unlock(&m->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(uart_open);

static ssize_t info_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf,
			  "MikeyBus UART2 @0x3DC (pads GPIO66/67 mux ONLY)\n"
			  "decomp: ReadTask drains 0x70 ring; resistor=cmd "
			  "3/0x8D/ch3 (NOT DIN poll)\n"
			  "model=0x%02x (%s) sample=%u remote=%d plugged=%d "
			  "force=%d force_model=0x%02x\n"
			  "pinmux=%d baud=%u uart_open=%d rx_bytes=%u "
			  "raw_ring=%u osos_ring=%u\n"
			  "rx_status_shadow=%u channel_mask_shadow=0x%x "
			  "resistor_backend_ready=%d ticks=%u\n"
			  "NO button decode; NO GPIO66/67 DIN detect claim\n",
			  m->model, mikey_model_name(m->model), m->model_sample,
			  mikey_headset_has_remote(m->model),
			  mikey_jack_present_locked(m), m->force_plugged,
			  m->force_model, m->pinmux_on, m->baud, m->uart_opened,
			  m->rx_bytes, m->raw_rx.count, m->osos_rx.count,
			  m->rx_status, m->channel_mask_shadow,
			  m->resistor_backend_ready, m->resistor_ticks);
}
static DEVICE_ATTR_RO(info);

static struct attribute *mikey_attrs[] = {
	&dev_attr_model.attr,
	&dev_attr_force_model.attr,
	&dev_attr_model_sample.attr,
	&dev_attr_plugged.attr,
	&dev_attr_force_plugged.attr,
	&dev_attr_baud.attr,
	&dev_attr_rx_bytes.attr,
	&dev_attr_rx_raw.attr,
	&dev_attr_rx_task_stream.attr,
	&dev_attr_rx_status_892A2C8_shadow.attr,
	&dev_attr_decomp_channel_mask_shadow.attr,
	&dev_attr_resistor_backend_ready.attr,
	&dev_attr_lower_packet_inject.attr,
	&dev_attr_uart_open.attr,
	&dev_attr_info.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mikey);

static size_t mikey_serdev_receive(struct serdev_device *serdev,
				   const u8 *data, size_t count)
{
	struct apple_mikeybus *m = serdev_device_get_drvdata(serdev);
	size_t i, n;

	if (!m || !count)
		return count;

	mutex_lock(&m->lock);
	m->rx_bytes += count;
	for (i = 0; i < count; i++)
		mikey_rx_append_byte(m, data[i]);
	n = min(count, sizeof(m->rx_last));
	memcpy(m->rx_last, data + count - n, n);
	m->rx_last_len = n;
	/*
	 * Raw serdev bytes → rings only. Do NOT run lower_packet_rx here
	 * until the wire stream is proven packet-framed.
	 */
	dev_info(m->dev, "Mikey RX %zu: %*ph\n", count, (int)min(count, 16),
		 data);
	mutex_unlock(&m->lock);
	return count;
}

static const struct serdev_device_ops mikey_serdev_ops = {
	.receive_buf = mikey_serdev_receive,
};

static int mikey_bind(struct device *dev, struct serdev_device *serdev)
{
	struct apple_mikeybus *m;
	u32 baud = 115200;
	int ret;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->dev = dev;
	m->serdev = serdev;
	m->baud = baud;
	m->model = 0;
	m->force_model = force_model_param ? force_model_param : MIKEY_MODEL_A18;
	m->force_plugged = force_plugged_param ||
			   of_property_read_bool(dev->of_node,
						 "apple,force-plugged");
	m->resistor_backend_ready = false;
	m->channel_mask_shadow = 0;
	m->rx_status = 0;
	if (dev->of_node &&
	    !of_property_read_u32(dev->of_node, "current-speed", &baud))
		m->baud = baud;
	if (m->force_plugged) {
		m->model = m->force_model;
		m->model_sample = m->force_model;
	}

	mutex_init(&m->lock);
	mikey_ring_reset(&m->raw_rx);
	mikey_ring_reset(&m->osos_rx);
	INIT_DELAYED_WORK(&m->uart_open_work, mikey_uart_open_workfn);
	INIT_DELAYED_WORK(&m->resistor_work, mikey_resistor_workfn);
	m->gpio = devm_ioremap(dev, GPIO_PHYS, 0x200);
	m->gpiocmd = devm_ioremap(dev, GPIOCMD_PHYS, 4);

	dev_set_drvdata(dev, m);
	if (serdev) {
		serdev_device_set_drvdata(serdev, m);
		serdev_device_set_client_ops(serdev, &mikey_serdev_ops);
	}

	mikey_pinmux_uart(m, true);

	ret = sysfs_create_groups(&dev->kobj, mikey_groups);
	if (ret)
		dev_warn(dev, "sysfs: %d\n", ret);

	mutex_lock(&mikeybus_singleton_lock);
	mikeybus_singleton = m;
	mutex_unlock(&mikeybus_singleton_lock);

	dev_info(dev,
		 "MikeyBus ready (%s) baud=%u model=0x%02x (%s) force_plugged=%d "
		 "(RX=raw+osos rings; resistor=EOPNOTSUPP; no DIN detect; "
		 "no button decode)\n",
		 serdev ? "serdev" : "platform",
		 m->baud, m->model, mikey_model_name(m->model),
		 m->force_plugged);

	if (serdev && uart_auto_open)
		schedule_delayed_work(&m->uart_open_work, msecs_to_jiffies(50));

	if (resistor_period_ms) {
		m->resistor_active = true;
		schedule_delayed_work(&m->resistor_work,
				      msecs_to_jiffies(resistor_period_ms));
	}
	return 0;
}

static void mikey_unbind(struct device *dev)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	if (!m)
		return;

	m->resistor_active = false;
	cancel_delayed_work_sync(&m->resistor_work);
	cancel_delayed_work_sync(&m->uart_open_work);

	mutex_lock(&mikeybus_singleton_lock);
	if (mikeybus_singleton == m)
		mikeybus_singleton = NULL;
	mutex_unlock(&mikeybus_singleton_lock);

	sysfs_remove_groups(&dev->kobj, mikey_groups);
	mutex_lock(&m->lock);
	mikey_uart_close_locked(m);
	mikey_pinmux_uart(m, false);
	mutex_unlock(&m->lock);
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
	if (instantiate_uart2)
		pr_warn("apple-mikeybus: instantiate_uart2 ignored\n");
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

MODULE_DESCRIPTION("Apple MikeyBus N31 (UART2 RX rings + force jack; resistor cmd TBD)");
MODULE_AUTHOR("FreeMyiPod");
MODULE_LICENSE("GPL");
