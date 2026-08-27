// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple MikeyBus — N31 headset jack model / remote (UART2 @ 0x3DC00000)
 *
 * RetailOS (osos 1.0.2):
 *   Tasks: CMikeyBusUartReadTask / CMikeyBusUartResistorTask (sub_35A4)
 *   UART open pinmux: sub_5714EE case 2 → GPIOCMD(0x42,2) + (0x43,2)
 *                     = GPIO 66/67 func mode 2, then sub_428F70(0x42,1)
 *   Model table: sub_DCEC / mikeyTask.cpp — MEMORY[0x8925CD3]
 *     1=A18 … 0xB=open circuit (unplugged) … 0x10=B187
 *   headsetHasMikey: sub_40BE5C
 *
 * This is jack *identity* + remote (resistor/UART), not Tristar Lightning mux
 * and not the CS42 HP amp itself. RetailOS HP mute is CS42 0x527; mixer
 * bring-up sub_570620 is gated on headset state 0x8925CF4==1 — Linux CS42
 * audio_on already applies the HP sequence, but jack model must still be
 * tracked so we do not treat open-circuit as headphones.
 *
 * Baud / byte protocol: OPEN until accessory MMIO snap. Default trial
 * 115200 8N1 (family heuristic). force_model sysfs for glass bring-up.
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

#define MIKEY_UART_PHYS		0x3dc00000ul
#define MIKEY_UART_LEN		0x3c
#define GPIO_PHYS		0x3cf00000ul
#define GPIOCMD_PHYS		0x3cf001e0ul

/* sub_5714EE case 2 */
#define MIKEY_GPIO_TX		0x42u	/* 66 */
#define MIKEY_GPIO_RX		0x43u	/* 67 */

#define MIKEY_MODEL_OPEN	0x0Bu
#define MIKEY_MODEL_A18		0x01u	/* passive HP family */

/* Glass: analog HP is in. Resistor protocol is still OPEN. */
static bool force_plugged_param = true;
module_param_named(force_plugged, force_plugged_param, bool, 0644);
MODULE_PARM_DESC(force_plugged,
		 "Treat jack as plugged until resistor task works (default 1)");

/*
 * Live s5l-uart instantiate (pinmux then platform_device_add) locked
 * glass 2026-08-27 — CPU died during samsung probe. UART2 is enabled
 * from DT at boot (uart3 remains first). This param is ignored.
 */
static bool instantiate_uart2;
module_param(instantiate_uart2, bool, 0444);
MODULE_PARM_DESC(instantiate_uart2,
		 "ignored; live s5l-uart add locked glass — use DT uart2 okay");

struct apple_mikeybus {
	struct device *dev;
	struct serdev_device *serdev;
	void __iomem *gpio;
	void __iomem *gpiocmd;
	struct mutex lock;
	u8 model;		/* 0x8925CD3 mirror */
	bool force_plugged;	/* glass: ignore open-circuit until resistor RE */
	bool pinmux_on;
	u32 baud;
	u32 rx_bytes;
	u8 rx_last[64];
	unsigned int rx_last_len;
	bool uart_opened;
};

static struct apple_mikeybus *mikeybus_singleton;
static DEFINE_MUTEX(mikeybus_singleton_lock);
static struct platform_device *mikey_plat_pdev;
static void mikey_ensure_plat(struct work_struct *work);
static DECLARE_WORK(mikey_plat_work, mikey_ensure_plat);

/* sub_DCEC name table (non-LVTM branch). */
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

/* sub_40BE5C — models expected to speak Mikey UART remote. */
static bool mikey_headset_has_remote(u8 model)
{
	switch (model) {
	case 2:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 0xA:
	case 0xD:
	case 0xE:
	case 0x10:
		return true;
	default:
		return false;
	}
}

static bool mikey_headset_ready_locked(struct apple_mikeybus *m)
{
	if (m->force_plugged)
		return true;
	/*
	 * 0xB is RetailOS "open circuit" only after the resistor task.
	 * Unmeasured model 0 is not unplugged — analog HP may already be in.
	 */
	if (m->model == MIKEY_MODEL_OPEN)
		return false;
	return true;
}

static bool mikey_jack_present_locked(struct apple_mikeybus *m)
{
	if (m->force_plugged)
		return true;
	if (m->model == 0 || m->model == MIKEY_MODEL_OPEN)
		return false;
	return true;
}

/**
 * apple_mikeybus_jack_present - headphones / headset tip present
 * Return: 1 present, 0 open circuit / unknown, -ENODEV if no driver
 */
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

/**
 * apple_mikeybus_headset_ready - RetailOS 0x8925CF4 gate for sub_570620
 * Return: 1 ready, 0 not ready, -ENODEV if no driver
 */
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

static void mikey_gpiocmd(struct apple_mikeybus *m, u8 gpio, u8 mode)
{
	u32 bank = gpio >> 3;
	u32 pin = gpio & 7;

	writel((bank << 16) | (pin << 8) | mode, m->gpiocmd);
}

/* sub_5714EE(UART2): mode 2 on 66/67. Close path uses mode 0xFFFE (65534). */
static void mikey_pinmux_uart(struct apple_mikeybus *m, bool on)
{
	u32 bank, pin, dir;
	void __iomem *b;

	if (!m->gpio || !m->gpiocmd)
		return;

	if (on) {
		/* mode 2 → DIR out + GPIOCMD mode byte (sub_43D38C) */
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
		/* mode 0xFFFE: clear DIR, cmd 0 (sub_571374 close) */
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
 * When the running DTB still has uart2 disabled, serdev never probes.
 * Bind a platform device so headset_ready() is 1 (force_plugged) instead
 * of -ENODEV. Do NOT platform_device_add("s5l-uart") — that locked glass
 * (samsung probe after GPIO 66/67 pinmux, 2026-08-27). UART2 itself is
 * enabled from DT at boot, uart3 first.
 */
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
	} else {
		pr_info("apple-mikeybus: platform bind (uart2 still DT-disabled)\n");
	}
out:
	if (mikey_np)
		of_node_put(mikey_np);
	if (uart_np)
		of_node_put(uart_np);
}

static ssize_t model_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	u8 model;

	mutex_lock(&m->lock);
	model = m->model;
	mutex_unlock(&m->lock);
	return sysfs_emit(buf, "0x%02x %s\n", model, mikey_model_name(model));
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
	mutex_unlock(&m->lock);
	dev_info(dev, "model set 0x%02x (%s) has_remote=%d plugged=%d\n",
		 m->model, mikey_model_name(m->model),
		 mikey_headset_has_remote(m->model),
		 mikey_jack_present_locked(m));
	return count;
}
static DEVICE_ATTR_RW(model);

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
	if (m->force_plugged &&
	    (m->model == 0 || m->model == MIKEY_MODEL_OPEN))
		m->model = MIKEY_MODEL_A18;
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(force_plugged);

static ssize_t pinmux_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m->pinmux_on);
}

static ssize_t pinmux_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	unsigned int v;
	int ret;

	ret = kstrtouint(buf, 0, &v);
	if (ret)
		return ret;
	mutex_lock(&m->lock);
	mikey_pinmux_uart(m, !!v);
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(pinmux);

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
	if (m->serdev)
		serdev_device_set_baudrate(m->serdev, v);
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(baud);

static ssize_t rx_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&m->lock);
	n = sysfs_emit(buf, "bytes=%u last_len=%u last=%*ph\n",
		       m->rx_bytes, m->rx_last_len,
		       m->rx_last_len, m->rx_last);
	mutex_unlock(&m->lock);
	return n;
}
static DEVICE_ATTR_RO(rx);

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
	if (v && !m->uart_opened) {
		if (!m->serdev) {
			mutex_unlock(&m->lock);
			return -ENODEV;
		}
		ret = serdev_device_open(m->serdev);
		if (ret) {
			mutex_unlock(&m->lock);
			return ret;
		}
		serdev_device_set_baudrate(m->serdev, m->baud);
		serdev_device_set_flow_control(m->serdev, false);
		m->uart_opened = true;
		dev_info(dev, "Mikey UART opened baud=%u\n", m->baud);
	} else if (!v && m->uart_opened) {
		serdev_device_close(m->serdev);
		m->uart_opened = false;
		dev_info(dev, "Mikey UART closed\n");
	}
	mutex_unlock(&m->lock);
	return count;
}
static DEVICE_ATTR_RW(uart_open);

static ssize_t info_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	return sysfs_emit(buf,
			  "MikeyBus UART2 @0x3DC GPIO 66/67\n"
			  "model=0x%02x (%s) remote=%d plugged=%d force=%d\n"
			  "pinmux=%d baud=%u uart_open=%d rx_bytes=%u\n"
			  "protocol baud OPEN — resistor task RE pending\n",
			  m->model, mikey_model_name(m->model),
			  mikey_headset_has_remote(m->model),
			  mikey_jack_present_locked(m), m->force_plugged,
			  m->pinmux_on, m->baud, m->uart_opened, m->rx_bytes);
}
static DEVICE_ATTR_RO(info);

static struct attribute *mikey_attrs[] = {
	&dev_attr_model.attr,
	&dev_attr_plugged.attr,
	&dev_attr_force_plugged.attr,
	&dev_attr_pinmux.attr,
	&dev_attr_baud.attr,
	&dev_attr_rx.attr,
	&dev_attr_uart_open.attr,
	&dev_attr_info.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mikey);

static size_t mikey_serdev_receive(struct serdev_device *serdev,
				   const u8 *data, size_t count)
{
	struct apple_mikeybus *m = serdev_device_get_drvdata(serdev);
	size_t n;

	if (!m || !count)
		return count;

	mutex_lock(&m->lock);
	m->rx_bytes += count;
	n = min(count, sizeof(m->rx_last));
	memcpy(m->rx_last, data + count - n, n);
	m->rx_last_len = n;
	/* Protocol OPEN — log only until resistor/remote decode lands. */
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
	m->force_plugged = force_plugged_param ||
			   of_property_read_bool(dev->of_node,
						 "apple,force-plugged");
	if (dev->of_node &&
	    !of_property_read_u32(dev->of_node, "current-speed", &baud))
		m->baud = baud;
	if (m->force_plugged)
		m->model = MIKEY_MODEL_A18;

	mutex_init(&m->lock);
	m->gpio = devm_ioremap(dev, GPIO_PHYS, 0x200);
	m->gpiocmd = devm_ioremap(dev, GPIOCMD_PHYS, 4);
	if (!m->gpio || !m->gpiocmd)
		dev_warn(dev, "GPIO/GPIOCMD map failed — pinmux sysfs limited\n");

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
		 "MikeyBus ready (%s) baud=%u model=0x%02x (%s) force_plugged=%d\n",
		 serdev ? "serdev, UART not opened" : "platform, uart2 bound",
		 m->baud, m->model, mikey_model_name(m->model),
		 m->force_plugged);
	return 0;
}

static void mikey_unbind(struct device *dev)
{
	struct apple_mikeybus *m = dev_get_drvdata(dev);

	if (!m)
		return;
	mutex_lock(&mikeybus_singleton_lock);
	if (mikeybus_singleton == m)
		mikeybus_singleton = NULL;
	mutex_unlock(&mikeybus_singleton_lock);

	sysfs_remove_groups(&dev->kobj, mikey_groups);
	mikey_pinmux_uart(m, false);
	if (m->uart_opened && m->serdev) {
		serdev_device_close(m->serdev);
		m->uart_opened = false;
	}
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
		pr_warn("apple-mikeybus: instantiate_uart2 ignored (live s5l-uart add locked glass)\n");
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

MODULE_DESCRIPTION("Apple MikeyBus headset jack model/remote (N31 UART2)");
MODULE_AUTHOR("FreeMyiPod");
MODULE_LICENSE("GPL");
