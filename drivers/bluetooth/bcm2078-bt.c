// SPDX-License-Identifier: GPL-2.0-only
/*
 * BCM2078 companion — N31
 *
 * Power: RetailOS sub_43D38C mode-2 / 0xFFFE on GPIOs 0x61/0x62/0x77.
 * UART1 HCI @ 0x3DB00000 / 115200 (BT Uart RxLoop).
 * Patchram: stream Vincent HCD (Write_RAM 0xFC4C ×146 + Launch_RAM 0xFC4E).
 * FM: HCI vendor 0xFC15 cookbook from RetailOS BroadcomFM FIFO (sub_4290C4).
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#define BCM_GPIO_PHYS		0x3cf00000UL
#define BCM_GPIOCMD_OFF		0x1e0
#define BCM_UART1_PHYS		0x3db00000UL
#define BCM_UART_STATUS		0x10
#define BCM_UART_TX		0x20
#define BCM_UART_RX		0x24
#define BCM_UART_TXFULL		0x20	/* STATUS bit — refine on HW if needed */
#define BCM_UART_RXRDY		0x01

#define BCM_GPIO_A		0x61
#define BCM_GPIO_B		0x62
#define BCM_GPIO_C		0x77
#define BCM_GPIO_NOP		0xC8
#define BCM_MODE_POWER		2
#define BCM_MODE_CLEAR		0xFFFE

#define BCM_FW_NAME		"brcm/BCM2076B1.hcd"
#define HCI_OP_FC15		0xFC15

struct bcm2078_bt {
	struct device *dev;
	void __iomem *gpio;
	void __iomem *gpiocmd;
	void __iomem *uart;
	struct clk *uart_clk;
	bool powered;
	bool patched;
	bool fw_present;
	struct mutex lock;
};

/* ---------- GPIO (RetailOS sub_43D38C) ---------- */

static void bcm_43D38C(struct bcm2078_bt *bt, unsigned int gpio, u16 mode, int val)
{
	void __iomem *bank;
	u32 pin, dir;
	u8 cmd;

	if (!bt->gpio || !bt->gpiocmd || gpio == BCM_GPIO_NOP)
		return;
	bank = bt->gpio + 32 * (gpio >> 3);
	pin = gpio & 7;
	if (mode == 1) {
		cmd = val ? 15 : 14;
	} else if (mode == BCM_MODE_CLEAR) {
		dir = readl(bank + 0x14);
		writel(dir & ~BIT(pin), bank + 0x14);
		cmd = 0;
	} else {
		cmd = (u8)mode;
		dir = readl(bank + 0x14);
		writel(dir | BIT(pin), bank + 0x14);
	}
	writel(((gpio >> 3) << 16) | (pin << 8) | cmd, bt->gpiocmd);
}

static void bcm_power_pins_on(struct bcm2078_bt *bt)
{
	bcm_43D38C(bt, BCM_GPIO_NOP, 0, 0);
	bcm_43D38C(bt, BCM_GPIO_A, BCM_MODE_POWER, 0);
	bcm_43D38C(bt, BCM_GPIO_B, BCM_MODE_POWER, 0);
	bcm_43D38C(bt, BCM_GPIO_C, BCM_MODE_POWER, 0);
}

static void bcm_power_pins_off(struct bcm2078_bt *bt)
{
	bcm_43D38C(bt, BCM_GPIO_NOP, BCM_MODE_CLEAR, 0);
	bcm_43D38C(bt, BCM_GPIO_A, BCM_MODE_CLEAR, 0);
	bcm_43D38C(bt, BCM_GPIO_B, BCM_MODE_CLEAR, 0);
	bcm_43D38C(bt, BCM_GPIO_C, BCM_MODE_CLEAR, 0);
}

/* ---------- UART1 H4 HCI ---------- */

static int bcm_uart_tx(struct bcm2078_bt *bt, const u8 *buf, size_t len)
{
	size_t i;
	unsigned guard;

	for (i = 0; i < len; i++) {
		guard = 200000;
		while (guard-- && (readl(bt->uart + BCM_UART_STATUS) & BCM_UART_TXFULL))
			cpu_relax();
		writel(buf[i], bt->uart + BCM_UART_TX);
	}
	return 0;
}

static size_t bcm_uart_rx(struct bcm2078_bt *bt, u8 *buf, size_t maxlen,
			  unsigned timeout_ms)
{
	size_t n = 0;
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);

	while (n < maxlen && time_before(jiffies, deadline)) {
		if (readl(bt->uart + BCM_UART_STATUS) & BCM_UART_RXRDY)
			buf[n++] = (u8)readl(bt->uart + BCM_UART_RX);
		else
			cpu_relax();
	}
	return n;
}

static void bcm_uart_drain(struct bcm2078_bt *bt)
{
	unsigned guard = 10000;

	while (guard-- && (readl(bt->uart + BCM_UART_STATUS) & BCM_UART_RXRDY))
		(void)readl(bt->uart + BCM_UART_RX);
}

static int bcm_hci_cmd(struct bcm2078_bt *bt, u16 opcode, const u8 *plen_payload,
		       u8 plen, u8 *evt, size_t evt_max, size_t *evt_n)
{
	u8 hdr[4];
	size_t n;

	hdr[0] = 0x01; /* H4 CMD */
	hdr[1] = opcode & 0xff;
	hdr[2] = opcode >> 8;
	hdr[3] = plen;
	bcm_uart_drain(bt);
	bcm_uart_tx(bt, hdr, 4);
	if (plen && plen_payload)
		bcm_uart_tx(bt, plen_payload, plen);
	n = bcm_uart_rx(bt, evt, evt_max, 500);
	if (evt_n)
		*evt_n = n;
	return n > 0 ? 0 : -ETIMEDOUT;
}

static int bcm_hci_reset(struct bcm2078_bt *bt)
{
	u8 evt[32];
	size_t n;
	int ret;

	ret = bcm_hci_cmd(bt, 0x0c03, NULL, 0, evt, sizeof(evt), &n);
	dev_info(bt->dev, "HCI Reset → %d RX %zu:%*ph\n", ret, n, (int)n, evt);
	return ret;
}

/* ---------- Patchram (Vincent HCD) ---------- */

static int bcm_load_hcd(struct bcm2078_bt *bt)
{
	const struct firmware *fw;
	const u8 *p, *end;
	u8 evt[64];
	size_t n;
	unsigned cmds = 0;
	int ret;

	ret = request_firmware(&fw, BCM_FW_NAME, bt->dev);
	if (ret) {
		dev_err(bt->dev, "firmware %s: %d\n", BCM_FW_NAME, ret);
		return ret;
	}

	p = fw->data;
	end = p + fw->size;
	while (p + 4 <= end) {
		u8 type = p[0];
		u16 opcode;
		u8 plen;

		if (type != 0x01) {
			dev_err(bt->dev, "HCD bad type %02x @+%zx\n",
				type, p - fw->data);
			ret = -EINVAL;
			break;
		}
		opcode = p[1] | (p[2] << 8);
		plen = p[3];
		if (p + 4 + plen > end) {
			ret = -EINVAL;
			break;
		}
		bcm_uart_drain(bt);
		bcm_uart_tx(bt, p, 4 + plen);
		n = bcm_uart_rx(bt, evt, sizeof(evt), 1000);
		cmds++;
		if (n < 2 || evt[0] != 0x04) {
			dev_warn(bt->dev,
				 "patch cmd#%u op=%04x plen=%u RX %zu:%*ph\n",
				 cmds, opcode, plen, n, (int)n, evt);
		}
		p += 4 + plen;
		/* Launch_RAM is last */
		if (opcode == 0xfc4e)
			break;
	}
	release_firmware(fw);
	bt->patched = (ret == 0);
	dev_info(bt->dev, "patchram %s — %u HCI cmds\n",
		 ret ? "FAIL" : "OK", cmds);
	return ret;
}

/* ---------- FM 0xFC15 cookbook (RetailOS BroadcomFM) ---------- */

static int bcm_fc15(struct bcm2078_bt *bt, const u8 *payload, u8 plen)
{
	u8 evt[64];
	size_t n;
	int ret;

	ret = bcm_hci_cmd(bt, HCI_OP_FC15, payload, plen, evt, sizeof(evt), &n);
	dev_dbg(bt->dev, "FC15 plen=%u → RX %zu:%*ph\n", plen, n, (int)n, evt);
	return ret;
}

/* Encode FIFO-style write8: plen=3, reg, 0x00, val */
static int bcm_fm_w8(struct bcm2078_bt *bt, u8 reg, u8 val)
{
	u8 p[3] = { reg, 0x00, val };

	return bcm_fc15(bt, p, 3);
}

static int bcm_fm_w16(struct bcm2078_bt *bt, u8 reg, u16 val)
{
	u8 p[4] = { reg, 0x00, val & 0xff, val >> 8 };

	return bcm_fc15(bt, p, 4);
}

static int bcm_fm_power_on(struct bcm2078_bt *bt)
{
	int ret;
	/* DD136 + DD334(0) + DD458 + DD2FC(33,20) */
	ret = bcm_fm_w8(bt, 0x00, 0x03);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0x14, 0x0c);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0x02, 0x02);
	if (ret)
		return ret;
	ret = bcm_fm_w16(bt, 0x05, 0x0001);
	if (ret)
		return ret;
	{
		u8 rd[3] = { 0x4d, 0x01, 0x01 }; /* read status */

		bcm_fc15(bt, rd, 3);
	}
	{
		/* RSSI=33 noise=20 — DD2FC */
		u8 p[11] = {
			0xf9, 0x00, 0x21, 0x00, 0x00, 0x00,
			0x14, 0x00, 0x00, 0x00, 0x00
		};

		ret = bcm_fc15(bt, p, 11);
	}
	dev_info(bt->dev, "FM power ON (0xFC15 cookbook)%s\n",
		 ret ? " FAIL" : "");
	return ret;
}

static int bcm_fm_power_off(struct bcm2078_bt *bt)
{
	int ret = bcm_fm_w8(bt, 0x00, 0x00); /* DD118: 00 00 00 */

	dev_info(bt->dev, "FM power OFF%s\n", ret ? " FAIL" : "");
	return ret;
}

static int bcm_fm_tune_khz(struct bcm2078_bt *bt, unsigned int khz)
{
	u16 enc;
	int ret;

	/* Band: >=87500 → 2 else 3 */
	ret = bcm_fm_w8(bt, 0x01, khz >= 87500 ? 2 : 3);
	if (ret)
		return ret;
	/* Pre-tune 56DB66: reg 0x10 = 0x1203 */
	ret = bcm_fm_w16(bt, 0x10, 0x1203);
	if (ret)
		return ret;
	enc = (u16)((khz + 1536) & 0xffff);
	ret = bcm_fm_w16(bt, 0x0a, enc);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0x09, 0x01); /* unmute */
	dev_info(bt->dev, "FM tune %u kHz enc=%04x%s\n",
		 khz, enc, ret ? " FAIL" : "");
	return ret;
}

static int bcm_fm_seek(struct bcm2078_bt *bt, int up, u8 rssi)
{
	u8 flags = 0x70 | (up ? 0x80 : 0);
	int ret;

	ret = bcm_fm_w8(bt, 0x07, flags);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0x08, rssi ? rssi : 33);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0xde, 0x01);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0xfc, 0x00);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0x09, 0x02);
	dev_info(bt->dev, "FM seek %s rssi=%u%s\n",
		 up ? "up" : "down", rssi ? rssi : 33, ret ? " FAIL" : "");
	return ret;
}

/* ---------- Power / bring-up ---------- */

static int bcm_power_on(struct bcm2078_bt *bt)
{
	if (bt->uart_clk)
		clk_prepare_enable(bt->uart_clk);
	bcm_power_pins_on(bt);
	msleep(150);
	bt->powered = true;
	bcm_hci_reset(bt);
	return 0;
}

static void bcm_power_off(struct bcm2078_bt *bt)
{
	if (bt->patched)
		bcm_fm_power_off(bt);
	bcm_power_pins_off(bt);
	if (bt->uart_clk)
		clk_disable_unprepare(bt->uart_clk);
	bt->powered = false;
	bt->patched = false;
}

/* ---------- sysfs ---------- */

static ssize_t power_on_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", bt->powered ? 1 : 0);
}

static ssize_t power_on_store(struct device *dev, struct device_attribute *a,
			      const char *buf, size_t count)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	unsigned int on;

	if (kstrtouint(buf, 0, &on))
		return -EINVAL;
	mutex_lock(&bt->lock);
	if (on)
		bcm_power_on(bt);
	else
		bcm_power_off(bt);
	mutex_unlock(&bt->lock);
	return count;
}
static DEVICE_ATTR_RW(power_on);

static ssize_t patchram_store(struct device *dev, struct device_attribute *a,
			      const char *buf, size_t count)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&bt->lock);
	if (!bt->powered)
		bcm_power_on(bt);
	ret = bcm_load_hcd(bt);
	mutex_unlock(&bt->lock);
	return ret ? ret : count;
}

static ssize_t patchram_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", bt->patched ? 1 : 0);
}
static DEVICE_ATTR_RW(patchram);

static ssize_t fm_power_store(struct device *dev, struct device_attribute *a,
			      const char *buf, size_t count)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	unsigned int on;
	int ret;

	if (kstrtouint(buf, 0, &on))
		return -EINVAL;
	mutex_lock(&bt->lock);
	if (!bt->powered)
		bcm_power_on(bt);
	if (!bt->patched)
		bcm_load_hcd(bt);
	ret = on ? bcm_fm_power_on(bt) : bcm_fm_power_off(bt);
	mutex_unlock(&bt->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(fm_power);

static ssize_t fm_tune_store(struct device *dev, struct device_attribute *a,
			     const char *buf, size_t count)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	unsigned int khz;
	int ret;

	/* accept kHz or MHz*10 (e.g. 991 for 99.1) */
	if (kstrtouint(buf, 0, &khz))
		return -EINVAL;
	if (khz < 1000)
		khz *= 100; /* 991 → 99100 */
	mutex_lock(&bt->lock);
	if (!bt->powered)
		bcm_power_on(bt);
	if (!bt->patched)
		bcm_load_hcd(bt);
	ret = bcm_fm_tune_khz(bt, khz);
	mutex_unlock(&bt->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(fm_tune);

static ssize_t fm_seek_store(struct device *dev, struct device_attribute *a,
			     const char *buf, size_t count)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	int up = 1;
	int ret;

	if (buf[0] == 'd' || buf[0] == '0' || buf[0] == '-')
		up = 0;
	mutex_lock(&bt->lock);
	if (!bt->powered)
		bcm_power_on(bt);
	if (!bt->patched)
		bcm_load_hcd(bt);
	ret = bcm_fm_seek(bt, up, 33);
	mutex_unlock(&bt->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(fm_seek);

static ssize_t patchram_info_show(struct device *dev, struct device_attribute *a,
				  char *buf)
{
	return sysfs_emit(buf,
		"hcd=/lib/firmware/%s\n"
		"hci=Write_RAM(0xFC4C)x146+Launch_RAM(0xFC4E)\n"
		"load=echo 1 > patchram\n"
		"fm=echo 1 > fm_power; echo 99100 > fm_tune; echo up > fm_seek\n"
		"fc15=RetailOS BroadcomFM FIFO cookbook\n",
		BCM_FW_NAME);
}
static DEVICE_ATTR_RO(patchram_info);

static struct attribute *bcm_attrs[] = {
	&dev_attr_power_on.attr,
	&dev_attr_patchram.attr,
	&dev_attr_patchram_info.attr,
	&dev_attr_fm_power.attr,
	&dev_attr_fm_tune.attr,
	&dev_attr_fm_seek.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bcm);

static int bcm2078_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm2078_bt *bt;
	const struct firmware *fw;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;
	bt->dev = dev;
	mutex_init(&bt->lock);
	platform_set_drvdata(pdev, bt);

	bt->gpio = devm_ioremap(dev, BCM_GPIO_PHYS, 0x400);
	if (!bt->gpio)
		return -ENOMEM;
	bt->gpiocmd = bt->gpio + BCM_GPIOCMD_OFF;

	bt->uart = devm_ioremap(dev, BCM_UART1_PHYS, 0x40);
	if (!bt->uart)
		return -ENOMEM;

	bt->uart_clk = devm_clk_get_optional(dev->parent ? dev->parent : dev,
					     "uart");
	if (IS_ERR(bt->uart_clk))
		bt->uart_clk = NULL;

	if (request_firmware(&fw, BCM_FW_NAME, dev) == 0) {
		bt->fw_present = fw->size > 0;
		release_firmware(fw);
	}

	if (sysfs_create_groups(&dev->kobj, bcm_groups))
		dev_warn(dev, "sysfs groups failed\n");

	/*
	 * Do NOT power UART or load patchram at probe — boot-time HCI + raw GPIO
	 * poke (0x61/0x62/0x77) correlated with reset back to RetailOS.
	 * Use sysfs power_on / patchram or /bin/n31-bt-up after init is up.
	 */
	dev_info(dev,
		 "BCM2078 deferred — echo 1 > power_on; echo 1 > patchram (fw=%s)\n",
		 bt->fw_present ? BCM_FW_NAME : "missing");
	return 0;
}

static void bcm2078_remove(struct platform_device *pdev)
{
	struct bcm2078_bt *bt = platform_get_drvdata(pdev);

	sysfs_remove_groups(&pdev->dev.kobj, bcm_groups);
	bcm_power_off(bt);
}

static const struct of_device_id bcm2078_of_match[] = {
	{ .compatible = "brcm,bcm2078" },
	{ .compatible = "brcm,bcm4329-bt" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2078_of_match);

static struct platform_driver bcm2078_driver = {
	.probe = bcm2078_probe,
	.remove = bcm2078_remove,
	.driver = {
		.name = "bcm2078-bt",
		.of_match_table = bcm2078_of_match,
	},
};

static int __init bcm2078_init(void)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "brcm,bcm2078")
		if (of_device_is_available(np))
			of_platform_device_create(np, NULL, NULL);
	for_each_compatible_node(np, NULL, "brcm,bcm4329-bt")
		if (of_device_is_available(np))
			of_platform_device_create(np, NULL, NULL);
	return platform_driver_register(&bcm2078_driver);
}
module_init(bcm2078_init);

static void __exit bcm2078_exit(void)
{
	platform_driver_unregister(&bcm2078_driver);
}
module_exit(bcm2078_exit);

MODULE_DESCRIPTION("BCM2078 HCI patchram + FM 0xFC15 (N31)");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(BCM_FW_NAME);
