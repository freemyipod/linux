// SPDX-License-Identifier: GPL-2.0-only
/*
 * BCM2078 N31 companion — RetailOS GPIO mode-2 + FM 0xFC15.
 *
 * UART1 HCI / patchram belong to apple,s5l-uart + serdev + hci_bcm/btbcm
 * (hci0). This driver must NOT ioremap UART1 or of_platform_device_create the
 * bluetooth child — that steals the port from hci_bcm.
 *
 * FM vendor opcode 0xFC15 is sent via __hci_cmd_sync on hci0 when HCI_UP.
 * Phase 3: thin V4L2 radio (/dev/radio0). Sysfs fm_* remains debug.
 * Audio is IIS2 ALSA capture → userspace → IIS0 play. No FM→A2DP path.
 */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#define BCM_GPIO_PHYS		0x3cf00000UL
#define BCM_GPIOCMD_OFF		0x1e0

#define BCM_GPIO_A		0x61	/* 97 — shutdown / REG_ON */
#define BCM_GPIO_B		0x62	/* 98 — device-wakeup */
#define BCM_GPIO_C		0x77	/* 119 — host-wakeup */
#define BCM_GPIO_NOP		0xC8
#define BCM_MODE_POWER		2
#define BCM_MODE_CLEAR		0xFFFE

#define HCI_OP_FC15		0xFC15

/* V4L2_TUNER_CAP_LOW: 62.5 Hz units → kHz * 16 */
#define BCM_FM_FREQ_TO_V4L(khz)	((khz) * 16u)
#define BCM_FM_V4L_TO_FREQ(f)	((f) / 16u)
#define BCM_FM_KHZ_MIN		87500u
#define BCM_FM_KHZ_MAX		108000u
#define BCM_FM_KHZ_DEFAULT	94700u	/* Canada test station */

/*
 * FC15 reg 0x05 audio ctrl (BCM4325/2048 family, via 0xFC15 on 2078):
 *   0x0001 = RetailOS audio route (DD334)
 *   0x0040 = 75 µs de-emphasis (Canada/US; 50 µs = clear bit)
 *   0x0020 = I2S PCM route (IIS2 on N31)
 */
#define BCM_FM_AUDIO_ROUTE_ORACLE	0x0001u
#define BCM_FM_AUDIO_DEMPH_75US		0x0040u
#define BCM_FM_AUDIO_ROUTE_I2S		0x0020u
#define BCM_FM_AUDIO_CTRL0_DEFAULT	(BCM_FM_AUDIO_ROUTE_ORACLE | \
					 BCM_FM_AUDIO_DEMPH_75US | \
					 BCM_FM_AUDIO_ROUTE_I2S)

static u16 fm_audio_ctrl0 = BCM_FM_AUDIO_CTRL0_DEFAULT;
module_param(fm_audio_ctrl0, ushort, 0644);
MODULE_PARM_DESC(fm_audio_ctrl0,
		 "FC15 reg0x05 audio ctrl (default 0x61 = route+I2S+75us deemph)");

struct bcm2078_bt {
	struct device *dev;
	void __iomem *gpio;
	void __iomem *gpiocmd;
	bool powered;
	bool fm_on;
	unsigned int fm_khz;
	struct mutex lock;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	bool radio_registered;
};

/* ---------- GPIO (RetailOS sub_43D38C mode-2) ---------- */

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

/* ---------- FM 0xFC15 via hci0 (not raw UART) ---------- */

static int bcm_fc15(struct bcm2078_bt *bt, const u8 *payload, u8 plen)
{
	struct hci_dev *hdev;
	struct sk_buff *skb;

	hdev = hci_dev_get(0);
	if (!hdev) {
		dev_warn_ratelimited(bt->dev,
				     "FC15: no hci0 — bring up hci_bcm first\n");
		return -ENODEV;
	}
	if (!test_bit(HCI_UP, &hdev->flags)) {
		hci_dev_put(hdev);
		dev_warn_ratelimited(bt->dev,
				     "FC15: hci0 down — run n31-bt-up / HCIDEVUP\n");
		return -ENETDOWN;
	}

	skb = __hci_cmd_sync(hdev, HCI_OP_FC15, plen, payload, HCI_CMD_TIMEOUT);
	hci_dev_put(hdev);
	if (IS_ERR(skb)) {
		dev_dbg(bt->dev, "FC15 plen=%u → %ld\n", plen, PTR_ERR(skb));
		return PTR_ERR(skb);
	}
	dev_dbg(bt->dev, "FC15 plen=%u → OK len=%u\n", plen, skb->len);
	kfree_skb(skb);
	return 0;
}

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

	ret = bcm_fm_w8(bt, 0x00, 0x03);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0x14, 0x0c);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0x02, 0x02);
	if (ret)
		return ret;
	ret = bcm_fm_w16(bt, 0x05, fm_audio_ctrl0);
	if (ret)
		return ret;
	{
		u8 rd[3] = { 0x4d, 0x01, 0x01 };

		bcm_fc15(bt, rd, 3);
	}
	{
		u8 p[11] = {
			0xf9, 0x00, 0x21, 0x00, 0x00, 0x00,
			0x14, 0x00, 0x00, 0x00, 0x00
		};

		ret = bcm_fc15(bt, p, 11);
	}
	bt->fm_on = !ret;
	dev_info(bt->dev, "FM power ON (0xFC15 via hci0) audio_ctrl=0x%04x%s\n",
		 fm_audio_ctrl0, ret ? " FAIL" : "");
	return ret;
}

static int bcm_fm_power_off(struct bcm2078_bt *bt)
{
	int ret = bcm_fm_w8(bt, 0x00, 0x00);

	bt->fm_on = false;
	dev_info(bt->dev, "FM power OFF%s\n", ret ? " FAIL" : "");
	return ret;
}

static int bcm_fm_tune_khz(struct bcm2078_bt *bt, unsigned int khz)
{
	u16 enc;
	int ret;

	if (!bt->fm_on) {
		ret = bcm_fm_power_on(bt);
		if (ret)
			return ret;
	}
	ret = bcm_fm_w8(bt, 0x01, khz >= 87500 ? 2 : 3);
	if (ret)
		return ret;
	ret = bcm_fm_w16(bt, 0x10, 0x1203);
	if (ret)
		return ret;
	enc = (u16)((khz + 1536) & 0xffff);
	ret = bcm_fm_w16(bt, 0x0a, enc);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, 0x09, 0x01);
	if (!ret)
		bt->fm_khz = khz;
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

static int bcm_power_on(struct bcm2078_bt *bt)
{
	bcm_power_pins_on(bt);
	msleep(150);
	bt->powered = true;
	dev_info(bt->dev,
		 "RetailOS mode-2 GPIOs on — hci_bcm owns UART1/hci0\n");
	return 0;
}

static void bcm_power_off(struct bcm2078_bt *bt)
{
	bcm_power_pins_off(bt);
	bt->powered = false;
}

/* ---------- V4L2 radio (tuner control only; PCM is ALSA IIS2) ---------- */

static int bcm_radio_querycap(struct file *file, void *fh,
			      struct v4l2_capability *cap)
{
	strscpy(cap->driver, "bcm2078-fm", sizeof(cap->driver));
	strscpy(cap->card, "N31 BCM2078 FM", sizeof(cap->card));
	strscpy(cap->bus_info, "hci0:0xFC15", sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_RADIO | V4L2_CAP_TUNER |
			   V4L2_CAP_HW_FREQ_SEEK;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int bcm_radio_g_tuner(struct file *file, void *fh, struct v4l2_tuner *t)
{
	struct bcm2078_bt *bt = video_drvdata(file);

	if (t->index > 0)
		return -EINVAL;
	strscpy(t->name, "FM", sizeof(t->name));
	t->type = V4L2_TUNER_RADIO;
	t->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
			V4L2_TUNER_CAP_FREQ_BANDS;
	t->rangelow = BCM_FM_FREQ_TO_V4L(BCM_FM_KHZ_MIN);
	t->rangehigh = BCM_FM_FREQ_TO_V4L(BCM_FM_KHZ_MAX);
	t->rxsubchans = V4L2_TUNER_SUB_STEREO;
	t->audmode = V4L2_TUNER_MODE_STEREO;
	t->signal = bt->fm_on ? 0xffff : 0;
	t->afc = 0;
	return 0;
}

static int bcm_radio_s_tuner(struct file *file, void *fh,
			     const struct v4l2_tuner *t)
{
	if (t->index > 0)
		return -EINVAL;
	return 0;
}

static int bcm_radio_g_frequency(struct file *file, void *fh,
				 struct v4l2_frequency *f)
{
	struct bcm2078_bt *bt = video_drvdata(file);

	if (f->tuner != 0)
		return -EINVAL;
	f->type = V4L2_TUNER_RADIO;
	f->frequency = BCM_FM_FREQ_TO_V4L(bt->fm_khz ? bt->fm_khz :
					  BCM_FM_KHZ_DEFAULT);
	return 0;
}

static int bcm_radio_s_frequency(struct file *file, void *fh,
				 const struct v4l2_frequency *f)
{
	struct bcm2078_bt *bt = video_drvdata(file);
	unsigned int khz;
	int ret;

	if (f->tuner != 0 || f->type != V4L2_TUNER_RADIO)
		return -EINVAL;
	khz = BCM_FM_V4L_TO_FREQ(f->frequency);
	if (khz < BCM_FM_KHZ_MIN)
		khz = BCM_FM_KHZ_MIN;
	if (khz > BCM_FM_KHZ_MAX)
		khz = BCM_FM_KHZ_MAX;

	mutex_lock(&bt->lock);
	if (!bt->powered)
		bcm_power_on(bt);
	if (!bt->fm_on) {
		ret = bcm_fm_power_on(bt);
		if (ret)
			goto out;
	}
	ret = bcm_fm_tune_khz(bt, khz);
out:
	mutex_unlock(&bt->lock);
	return ret;
}

static int bcm_radio_s_hw_freq_seek(struct file *file, void *fh,
				    const struct v4l2_hw_freq_seek *a)
{
	struct bcm2078_bt *bt = video_drvdata(file);
	int ret;

	if (a->tuner != 0 || a->type != V4L2_TUNER_RADIO)
		return -EINVAL;

	mutex_lock(&bt->lock);
	if (!bt->powered)
		bcm_power_on(bt);
	if (!bt->fm_on) {
		ret = bcm_fm_power_on(bt);
		if (ret)
			goto out;
	}
	ret = bcm_fm_seek(bt, a->seek_upward ? 1 : 0, 33);
out:
	mutex_unlock(&bt->lock);
	return ret;
}

static int bcm_radio_enum_freq_bands(struct file *file, void *fh,
				     struct v4l2_frequency_band *band)
{
	if (band->tuner != 0 || band->index > 0)
		return -EINVAL;
	band->type = V4L2_TUNER_RADIO;
	band->capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO;
	band->rangelow = BCM_FM_FREQ_TO_V4L(BCM_FM_KHZ_MIN);
	band->rangehigh = BCM_FM_FREQ_TO_V4L(BCM_FM_KHZ_MAX);
	band->modulation = V4L2_BAND_MODULATION_FM;
	return 0;
}

static const struct v4l2_ioctl_ops bcm_radio_ioctl_ops = {
	.vidioc_querycap = bcm_radio_querycap,
	.vidioc_g_tuner = bcm_radio_g_tuner,
	.vidioc_s_tuner = bcm_radio_s_tuner,
	.vidioc_g_frequency = bcm_radio_g_frequency,
	.vidioc_s_frequency = bcm_radio_s_frequency,
	.vidioc_s_hw_freq_seek = bcm_radio_s_hw_freq_seek,
	.vidioc_enum_freq_bands = bcm_radio_enum_freq_bands,
};

static const struct v4l2_file_operations bcm_radio_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = v4l2_fh_release,
	.unlocked_ioctl = video_ioctl2,
};

static int bcm_radio_register(struct bcm2078_bt *bt)
{
	int ret;

	ret = v4l2_device_register(bt->dev, &bt->v4l2_dev);
	if (ret)
		return ret;

	strscpy(bt->v4l2_dev.name, "bcm2078-fm", sizeof(bt->v4l2_dev.name));
	bt->vdev = (struct video_device){
		.name = "N31 FM Radio",
		.v4l2_dev = &bt->v4l2_dev,
		.fops = &bcm_radio_fops,
		.ioctl_ops = &bcm_radio_ioctl_ops,
		.release = video_device_release_empty,
		.device_caps = V4L2_CAP_RADIO | V4L2_CAP_TUNER |
			       V4L2_CAP_HW_FREQ_SEEK,
	};
	video_set_drvdata(&bt->vdev, bt);

	ret = video_register_device(&bt->vdev, VFL_TYPE_RADIO, -1);
	if (ret) {
		v4l2_device_unregister(&bt->v4l2_dev);
		return ret;
	}
	bt->radio_registered = true;
	bt->fm_khz = BCM_FM_KHZ_DEFAULT;
	dev_info(bt->dev, "V4L2 radio %s (0xFC15; headphones required; no FM→A2DP)\n",
		 video_device_node_name(&bt->vdev));
	return 0;
}

static void bcm_radio_unregister(struct bcm2078_bt *bt)
{
	if (!bt->radio_registered)
		return;
	video_unregister_device(&bt->vdev);
	v4l2_device_unregister(&bt->v4l2_dev);
	bt->radio_registered = false;
}

/* ---------- sysfs (debug FM + GPIO helper) ---------- */

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

static ssize_t patchram_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	struct hci_dev *hdev = hci_dev_get(0);
	int up = 0;

	if (hdev) {
		up = test_bit(HCI_UP, &hdev->flags);
		hci_dev_put(hdev);
	}
	return sysfs_emit(buf, "%d\n", up);
}

static ssize_t patchram_store(struct device *dev, struct device_attribute *a,
			      const char *buf, size_t count)
{
	/* Interim: patchram is owned by hci_bcm/btbcm — do not steal UART. */
	dev_info(dev,
		 "patchram retired — use hci0 (hci_bcm + brcm/BCM2076B1.hcd)\n");
	return count;
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

	if (kstrtouint(buf, 0, &khz))
		return -EINVAL;
	if (khz < 1000)
		khz *= 100;
	mutex_lock(&bt->lock);
	if (!bt->powered)
		bcm_power_on(bt);
	if (!bt->fm_on) {
		ret = bcm_fm_power_on(bt);
		if (ret)
			goto out;
	}
	ret = bcm_fm_tune_khz(bt, khz);
out:
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
	ret = bcm_fm_seek(bt, up, 33);
	mutex_unlock(&bt->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(fm_seek);

static ssize_t patchram_info_show(struct device *dev, struct device_attribute *a,
				  char *buf)
{
	return sysfs_emit(buf,
		"hci=hci_bcm/serdev on uart1 (not this companion)\n"
		"hcd=/lib/firmware/brcm/BCM2076B1.hcd\n"
		"bringup=/bin/n31-bt-up → hci0 + HCIDEVUP\n"
		"gpio=RetailOS mode-2 on 0x61/0x62/0x77 via power_on\n"
		"radio=/dev/radio0 V4L2 (prefer); sysfs fm_* = debug\n"
		"audio=IIS2 capture → arecord|aplay IIS0 (headphones required)\n"
		"fm_default=94700 kHz deemph=75us (Canada)\n"
		"no_fm_a2dp=1 (local speakers/HP only)\n");
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
	int ret;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;
	bt->dev = dev;
	bt->fm_khz = BCM_FM_KHZ_DEFAULT;
	mutex_init(&bt->lock);
	platform_set_drvdata(pdev, bt);

	bt->gpio = devm_ioremap(dev, BCM_GPIO_PHYS, 0x400);
	if (!bt->gpio)
		return -ENOMEM;
	bt->gpiocmd = bt->gpio + BCM_GPIOCMD_OFF;

	if (sysfs_create_groups(&dev->kobj, bcm_groups))
		dev_warn(dev, "sysfs groups failed\n");

	ret = bcm_radio_register(bt);
	if (ret)
		dev_warn(dev, "V4L2 radio register: %d (sysfs FM still OK)\n",
			 ret);

	/*
	 * Do NOT poke GPIOs or touch UART at probe — early mode-2 correlated
	 * with reset to RetailOS. Use power_on / n31-bt-up after init is up.
	 */
	dev_info(dev,
		 "BCM2078 companion (GPIO+FM+V4L2) — UART1 owned by hci_bcm\n");
	return 0;
}

static void bcm2078_remove(struct platform_device *pdev)
{
	struct bcm2078_bt *bt = platform_get_drvdata(pdev);

	bcm_radio_unregister(bt);
	sysfs_remove_groups(&pdev->dev.kobj, bcm_groups);
	bcm_power_off(bt);
}

static const struct of_device_id bcm2078_of_match[] = {
	{ .compatible = "apple,n31-bcm2078-companion" },
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

module_platform_driver(bcm2078_driver);

MODULE_DESCRIPTION("N31 BCM2078 GPIO companion + V4L2 FM (0xFC15 via hci0)");
MODULE_LICENSE("GPL");
