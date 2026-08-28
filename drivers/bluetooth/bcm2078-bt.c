// SPDX-License-Identifier: GPL-2.0-only
/*
 * BCM2078 N31 companion — RetailOS GPIO mode-2 + FM 0xFC15.
 *
 * UART1 HCI / patchram belong to apple,s5l-uart + serdev + hci_bcm/btbcm
 * (hci0). This driver must NOT ioremap UART1 or of_platform_device_create the
 * bluetooth child — that steals the port from hci_bcm.
 *
 * GPIO 97/98/119 are not this driver's business either, but for a different
 * reason than the UART. They were taken for BCM shutdown / device-wakeup /
 * host-wakeup, and this driver muxed them at function 2 under that name.
 * sub_15DD5C shows what they really are: FM power-on claims exactly these
 * three at function 2 next to programming audio device 2 (0x3D400000) and
 * kicking RXCOM, so they are the IIS2 PCM pads. The mux is right, the owner
 * was wrong -- s5l8740-i2s claims them with the rest of the capture setup,
 * which also means the capture PCM works without the tuner being on.
 * gpio_poke=1 restores the old local poking for bring-up comparisons.
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

/*
 * FM_RDS_Command register map, from the BlueTool hcidef (COMMAND
 * "FM_RDS_Command" 0x015). The opcode carries an I2C-style register
 * transaction: address, read/write, then either the write data or, for a
 * read, the byte count. Register widths below are the read lengths that
 * same definition encodes.
 */
#define FM_REG_RDS_SYSTEM	0x00	/* 1: FM_ON | RDS_ON */
#define FM_REG_FM_CTRL		0x01	/* 1: band, stereo, injection */
#define FM_REG_RDS_CTRL		0x02	/* 1 */
#define FM_REG_AUDIO_PAUSE	0x04	/* 1 */
#define FM_REG_AUDIO_CTRL	0x05	/* 2: mute/route/de-emphasis */
#define FM_REG_SEARCH_CTRL	0x07	/* 1: direction + RSSI threshold */
#define FM_REG_SEARCH_CTRL1	0x08	/* 1 */
#define FM_REG_SEARCH_TUNE	0x09	/* 1: 1 = tune, 2 = search */
#define FM_REG_FREQ		0x0a	/* 2 */
#define FM_REG_AF_FREQ		0x0c	/* 2 */
#define FM_REG_CARRIER		0x0e	/* 1 */
#define FM_REG_RSSI		0x0f	/* 1 */
#define FM_REG_RDS_MASK		0x10	/* 2 */
#define FM_REG_RDS_FLAG		0x12	/* 2 */
#define FM_REG_RDS_WLINE	0x14	/* 1: RDS FIFO watermark */
#define FM_REG_RDS_BLKB_MATCH	0x16	/* 2 */
#define FM_REG_RDS_BLKB_MASK	0x18	/* 2 */
#define FM_REG_RDS_PI_MATCH	0x1a	/* 2 */
#define FM_REG_RDS_PI_MASK	0x1c	/* 2 */
#define FM_REG_RDS_BOOT		0x1e	/* 1 */
#define FM_REG_RDS_TEST		0x1f	/* 1 */
#define FM_REG_SLAVE_CONFIG	0x29
#define FM_REG_ROUTE_PCM	0x4d	/* 1: tuner audio onto the PCM port */
#define FM_REG_RDS_DATA		0x80	/* n: RDS FIFO, caller-sized */
#define FM_REG_BEST_TUNE	0x90	/* 1 */
#define FM_REG_SMUTE_V3		0xda	/* 5 */
#define FM_REG_FEATURES		0xdb	/* 4 */
#define FM_REG_PRESCAN_QUALITY	0xde	/* 1 */
#define FM_REG_SNR		0xdf	/* 1 */
#define FM_REG_EXTRA_AUDIO	0xf5	/* 1: FMRX_2_AFIFO_ENABLE */
#define FM_REG_ANT_MATCHING	0xf6	/* 1 */
#define FM_REG_VOLUME_CTRL	0xf8	/* 2 */
#define FM_REG_BLEND_SMUTE	0xf9	/* 8: stereo blend + soft mute */
#define FM_REG_ANT_SELECT	0xfa	/* 1 */
#define FM_REG_SEARCH_BOUND	0xfb	/* 4: band edges */
#define FM_REG_SEARCH_METHOD	0xfc	/* 1 */
#define FM_REG_SEARCH_STEP	0xfd	/* 2 */
#define FM_REG_PRESET_MAX	0xfe	/* 1 */
#define FM_REG_PRESET_CHAN	0xff	/* n */

#define FM_MODE_WRITE		0
#define FM_MODE_READ		1

/* SEARCH_TUNE_MODE values. */
#define FM_TUNE_MODE_IDLE	0
#define FM_TUNE_MODE_PRESET	1
#define FM_TUNE_MODE_SEARCH	2

/* RDS_SYSTEM bits. */
#define FM_SYSTEM_FM_ON		0x01
#define FM_SYSTEM_RDS_ON	0x02

/* FM_CTRL bits. Band select 0 = 87.5-108 MHz, 1 = 76-90 MHz. */
#define FM_CTRL_BAND_JAPAN	0x01
#define FM_CTRL_STEREO_AUTO	0x02
#define FM_CTRL_STEREO_MANUAL	0x04
#define FM_CTRL_STEREO_BLEND	0x08
#define FM_CTRL_INJECTION	0x10

/* AUDIO_CTRL bits 6:0; 15:7 is the audio bandwidth select. */
#define FM_AUDIO_RF_MUTE	0x0001
#define FM_AUDIO_MANUAL_MUTE	0x0002
#define FM_AUDIO_Z_MUTE_LEFT	0x0004
#define FM_AUDIO_Z_MUTE_RIGHT	0x0008
#define FM_AUDIO_ROUTE_DAC	0x0010
#define FM_AUDIO_ROUTE_I2S	0x0020
#define FM_AUDIO_DEEMPH_75US	0x0040

/* Largest RDS read we will ask for in one go. */
#define FM_RDS_READ_MAX		60
#define FM_RDS_TEXT_MAX		64
#define FM_RDS_PS_MAX		8

/* Bounds for the raw HCI passthrough. */
#define BCM_HCI_PARAM_MAX	255
#define BCM_HCI_RSP_MAX		64

/* V4L2_TUNER_CAP_LOW: 62.5 Hz units → kHz * 16 */
#define BCM_FM_FREQ_TO_V4L(khz)	((khz) * 16u)
#define BCM_FM_V4L_TO_FREQ(f)	((f) / 16u)
#define BCM_FM_KHZ_MIN		87500u
#define BCM_FM_KHZ_MAX		108000u
#define BCM_FM_KHZ_DEFAULT	94700u	/* Canada test station */

/*
 * RF_MUTE squelches the output as C/N falls, de-emphasis is 75 us for
 * North America (clear the bit for the 50 us regions), and ROUTE_I2S is
 * what puts tuner audio on the PCM port that IIS2 captures.
 */
#define BCM_FM_AUDIO_CTRL0_DEFAULT	(FM_AUDIO_RF_MUTE | \
					 FM_AUDIO_DEEMPH_75US | \
					 FM_AUDIO_ROUTE_I2S)

/*
 * Off by default: these pins belong to hci_bcm. See the file header.
 */
static bool gpio_poke;
module_param(gpio_poke, bool, 0644);
MODULE_PARM_DESC(gpio_poke,
		 "Drive the BCM control pins directly (default N; hci_bcm owns them)");

static u8 rds_wline = 12;
module_param(rds_wline, byte, 0644);
MODULE_PARM_DESC(rds_wline, "RDS FIFO watermark in blocks (default 12)");

static bool fm_route_pcm = true;
module_param(fm_route_pcm, bool, 0644);
MODULE_PARM_DESC(fm_route_pcm,
		 "Write ROUTE_PCM on FM power-on so IIS2 receives audio");

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
	bool rds_on;
	unsigned int fm_khz;
	/* Last decoded RDS. Guarded by lock along with everything else. */
	char rds_ps[FM_RDS_PS_MAX + 1];
	char rds_rt[FM_RDS_TEXT_MAX + 1];
	char rds_ps_build[FM_RDS_PS_MAX];
	char rds_rt_build[FM_RDS_TEXT_MAX];
	u16 rds_pi;
	u8 rds_pty;
	u8 rds_rt_ab;
	unsigned int rds_groups;
	u8 reg_addr;
	u8 reg_len;
	u8 reg_data[FM_RDS_READ_MAX];
	u16 hci_opcode;
	bool hci_valid;
	u8 hci_rsp_len;
	u8 hci_rsp[BCM_HCI_RSP_MAX];
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

/*
 * A read transaction is address + mode + byte count; the count is fixed
 * per register except for the RDS FIFO, where the caller chooses it.
 *
 * The command-complete parameters begin with the status byte. Some
 * firmware revisions echo the address and mode ahead of the payload and
 * some do not, so accept either rather than assuming: the payload is
 * whatever trails a 1- or 3-byte header of the expected total length.
 */
static int bcm_fm_read(struct bcm2078_bt *bt, u8 reg, u8 *out, u8 len)
{
	u8 req[3] = { reg, FM_MODE_READ, len };
	struct hci_dev *hdev;
	struct sk_buff *skb;
	unsigned int hdr;
	int ret = 0;

	if (!len)
		return -EINVAL;

	hdev = hci_dev_get(0);
	if (!hdev)
		return -ENODEV;
	if (!test_bit(HCI_UP, &hdev->flags)) {
		hci_dev_put(hdev);
		return -ENETDOWN;
	}

	skb = __hci_cmd_sync(hdev, HCI_OP_FC15, sizeof(req), req,
			     HCI_CMD_TIMEOUT);
	hci_dev_put(hdev);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	if (skb->len == 1u + len)
		hdr = 1;
	else if (skb->len == 3u + len)
		hdr = 3;
	else {
		dev_warn_ratelimited(bt->dev,
				     "FC15 read reg 0x%02x: %u bytes for len %u: %*ph\n",
				     reg, skb->len, len,
				     min_t(int, skb->len, 16), skb->data);
		ret = -EPROTO;
		goto out;
	}
	if (skb->data[0]) {
		dev_dbg(bt->dev, "FC15 read reg 0x%02x status 0x%02x\n",
			reg, skb->data[0]);
		ret = -EIO;
		goto out;
	}
	memcpy(out, skb->data + hdr, len);
out:
	kfree_skb(skb);
	return ret;
}

static int bcm_fm_r8(struct bcm2078_bt *bt, u8 reg, u8 *val)
{
	return bcm_fm_read(bt, reg, val, 1);
}

static int bcm_fm_r16(struct bcm2078_bt *bt, u8 reg, u16 *val)
{
	u8 b[2];
	int ret = bcm_fm_read(bt, reg, b, 2);

	if (!ret)
		*val = (u16)b[0] | ((u16)b[1] << 8);
	return ret;
}

/* ---------- RDS ---------- */

/*
 * The FIFO hands back RDS blocks as 3-byte records: two data bytes and a
 * status byte whose low bits carry the block offset (A, B, C, C', D) and
 * whose upper bits flag correction/error. Four blocks make a group, and
 * only the well-formed ones are worth decoding.
 */
#define FM_RDS_REC_LEN		3
#define FM_RDS_BLK_MASK		0x07
#define FM_RDS_BLK_A		0
#define FM_RDS_BLK_B		1
#define FM_RDS_BLK_C		2
#define FM_RDS_BLK_CP		3
#define FM_RDS_BLK_D		4
#define FM_RDS_ERR_MASK		0x80

/* Printable-ASCII guard: RDS pads with 0x20 and terminates RT with 0x0D. */
static char bcm_rds_char(u8 c)
{
	return (c >= 0x20 && c < 0x7f) ? (char)c : ' ';
}

static void bcm_rds_group(struct bcm2078_bt *bt, const u16 blk[4])
{
	unsigned int type = blk[1] >> 12;
	unsigned int ver = (blk[1] >> 11) & 1;
	unsigned int i;

	bt->rds_pi = blk[0];
	bt->rds_pty = (blk[1] >> 5) & 0x1f;
	bt->rds_groups++;

	if (type == 0) {
		/* 0A/0B: two Program Service characters at offset 2*seg. */
		unsigned int seg = blk[1] & 0x03;

		bt->rds_ps_build[seg * 2] = bcm_rds_char(blk[3] >> 8);
		bt->rds_ps_build[seg * 2 + 1] = bcm_rds_char(blk[3] & 0xff);
		if (seg == 3) {
			memcpy(bt->rds_ps, bt->rds_ps_build, FM_RDS_PS_MAX);
			bt->rds_ps[FM_RDS_PS_MAX] = 0;
		}
	} else if (type == 2) {
		/*
		 * 2A carries four RadioText characters per group, 2B two. The
		 * A/B flag toggles when the station starts a new message, so
		 * clear the buffer rather than blending two texts together.
		 */
		unsigned int seg = blk[1] & 0x0f;
		unsigned int ab = (blk[1] >> 4) & 1;
		unsigned int n = ver ? 2 : 4;
		unsigned int base = seg * n;

		if (ab != bt->rds_rt_ab) {
			bt->rds_rt_ab = ab;
			memset(bt->rds_rt_build, ' ', FM_RDS_TEXT_MAX);
		}
		if (base + n <= FM_RDS_TEXT_MAX) {
			if (ver) {
				bt->rds_rt_build[base] = bcm_rds_char(blk[3] >> 8);
				bt->rds_rt_build[base + 1] =
					bcm_rds_char(blk[3] & 0xff);
			} else {
				bt->rds_rt_build[base] = bcm_rds_char(blk[2] >> 8);
				bt->rds_rt_build[base + 1] =
					bcm_rds_char(blk[2] & 0xff);
				bt->rds_rt_build[base + 2] =
					bcm_rds_char(blk[3] >> 8);
				bt->rds_rt_build[base + 3] =
					bcm_rds_char(blk[3] & 0xff);
			}
			memcpy(bt->rds_rt, bt->rds_rt_build, FM_RDS_TEXT_MAX);
			bt->rds_rt[FM_RDS_TEXT_MAX] = 0;
			for (i = FM_RDS_TEXT_MAX; i > 0; i--) {
				if (bt->rds_rt[i - 1] != ' ')
					break;
				bt->rds_rt[i - 1] = 0;
			}
		}
	}
}

/*
 * Drain the FIFO once and feed whole groups to the decoder. Blocks are
 * accumulated by their offset code so a partial group at either end of
 * the read is discarded rather than shifting everything that follows.
 */
static int bcm_rds_poll(struct bcm2078_bt *bt)
{
	u8 buf[FM_RDS_READ_MAX];
	u16 blk[4];
	bool have[4] = { false, false, false, false };
	unsigned int i;
	int ret;

	ret = bcm_fm_read(bt, FM_REG_RDS_DATA, buf, sizeof(buf));
	if (ret)
		return ret;

	for (i = 0; i + FM_RDS_REC_LEN <= sizeof(buf); i += FM_RDS_REC_LEN) {
		u8 st = buf[i + 2];
		unsigned int off = st & FM_RDS_BLK_MASK;
		u16 val = ((u16)buf[i] << 8) | buf[i + 1];

		if (st & FM_RDS_ERR_MASK)
			continue;
		if (off == FM_RDS_BLK_CP)
			off = FM_RDS_BLK_C;
		if (off > FM_RDS_BLK_D)
			continue;
		if (off == FM_RDS_BLK_A) {
			memset(have, 0, sizeof(have));
			blk[0] = val;
			have[0] = true;
			continue;
		}
		if (!have[0])
			continue;
		blk[off > FM_RDS_BLK_C ? 3 : off] = val;
		have[off > FM_RDS_BLK_C ? 3 : off] = true;
		if (have[0] && have[1] && have[2] && have[3]) {
			bcm_rds_group(bt, blk);
			memset(have, 0, sizeof(have));
		}
	}
	return 0;
}

static int bcm_rds_enable(struct bcm2078_bt *bt, bool on)
{
	int ret;

	ret = bcm_fm_w8(bt, FM_REG_RDS_SYSTEM,
			FM_SYSTEM_FM_ON | (on ? FM_SYSTEM_RDS_ON : 0));
	if (ret)
		return ret;
	if (on) {
		/* Interrupt once the FIFO holds this many blocks. */
		ret = bcm_fm_w8(bt, FM_REG_RDS_WLINE, rds_wline);
		if (ret)
			return ret;
	}
	bt->rds_on = on;
	dev_info(bt->dev, "RDS %s\n", on ? "on" : "off");
	return 0;
}

static int bcm_fm_power_on(struct bcm2078_bt *bt)
{
	int ret;

	/* Tuner and RDS decoder on together; RDS costs nothing when idle. */
	ret = bcm_fm_w8(bt, FM_REG_RDS_SYSTEM,
			FM_SYSTEM_FM_ON | FM_SYSTEM_RDS_ON);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, FM_REG_RDS_WLINE, rds_wline);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, FM_REG_RDS_CTRL, 0x02);
	if (ret)
		return ret;
	ret = bcm_fm_w16(bt, FM_REG_AUDIO_CTRL, fm_audio_ctrl0);
	if (ret)
		return ret;

	/*
	 * ROUTE_PCM is what actually puts tuner audio on the port IIS2
	 * captures. The old code only read this register back and never
	 * wrote it, leaving the route wherever the last owner left it.
	 */
	if (fm_route_pcm) {
		ret = bcm_fm_w8(bt, FM_REG_ROUTE_PCM, 0x01);
		if (ret)
			return ret;
	}

	/* Stereo blend and soft-mute curve; 8 data bytes per the register map. */
	{
		u8 p[11] = {
			FM_REG_BLEND_SMUTE, FM_MODE_WRITE,
			0x21, 0x00, 0x00, 0x00,
			0x14, 0x00, 0x00, 0x00, 0x00
		};

		ret = bcm_fc15(bt, p, sizeof(p));
	}
	bt->fm_on = !ret;
	bt->rds_on = !ret;
	memset(bt->rds_ps, 0, sizeof(bt->rds_ps));
	memset(bt->rds_rt, 0, sizeof(bt->rds_rt));
	memset(bt->rds_ps_build, ' ', sizeof(bt->rds_ps_build));
	memset(bt->rds_rt_build, ' ', sizeof(bt->rds_rt_build));
	bt->rds_groups = 0;
	dev_info(bt->dev,
		 "FM power ON audio_ctrl=0x%04x route_pcm=%d wline=%u%s\n",
		 fm_audio_ctrl0, fm_route_pcm, rds_wline,
		 ret ? " FAIL" : "");
	return ret;
}

static int bcm_fm_power_off(struct bcm2078_bt *bt)
{
	int ret = bcm_fm_w8(bt, FM_REG_RDS_SYSTEM, 0x00);

	bt->fm_on = false;
	bt->rds_on = false;
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
	/* Band select is inverted: the Japan band is the one with the bit. */
	ret = bcm_fm_w8(bt, FM_REG_FM_CTRL,
			FM_CTRL_STEREO_AUTO |
			(khz >= 87500 ? 0 : FM_CTRL_BAND_JAPAN));
	if (ret)
		return ret;
	/* Arm the tune/search-complete and RDS flags we care about. */
	ret = bcm_fm_w16(bt, FM_REG_RDS_MASK, 0x1203);
	if (ret)
		return ret;
	enc = (u16)((khz + 1536) & 0xffff);
	ret = bcm_fm_w16(bt, FM_REG_FREQ, enc);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, FM_REG_SEARCH_TUNE, FM_TUNE_MODE_PRESET);
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

	ret = bcm_fm_w8(bt, FM_REG_SEARCH_CTRL, flags);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, FM_REG_SEARCH_CTRL1, rssi ? rssi : 33);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, FM_REG_PRESCAN_QUALITY, 0x01);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, FM_REG_SEARCH_METHOD, 0x00);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, FM_REG_SEARCH_TUNE, FM_TUNE_MODE_SEARCH);
	dev_info(bt->dev, "FM seek %s rssi=%u%s\n",
		 up ? "up" : "down", rssi ? rssi : 33, ret ? " FAIL" : "");
	return ret;
}

static int bcm_power_on(struct bcm2078_bt *bt)
{
	bt->powered = true;
	if (!gpio_poke) {
		dev_dbg(bt->dev,
			"control pins left to hci_bcm (gpio_poke=0)\n");
		return 0;
	}
	bcm_power_pins_on(bt);
	msleep(150);
	dev_info(bt->dev,
		 "RetailOS mode-2 GPIOs forced on (gpio_poke=1)\n");
	return 0;
}

static void bcm_power_off(struct bcm2078_bt *bt)
{
	bt->powered = false;
	if (!gpio_poke)
		return;
	bcm_power_pins_off(bt);
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
	u8 rssi = 0;

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
	t->afc = 0;

	/*
	 * Report what the tuner actually sees. RSSI is a single byte on the
	 * chip's own scale, spread over the 16-bit field V4L2 expects; a
	 * failed read is reported as no signal rather than as an error, so
	 * that polling a powered-down tuner stays harmless.
	 */
	mutex_lock(&bt->lock);
	if (bt->fm_on && !bcm_fm_r8(bt, FM_REG_RSSI, &rssi))
		t->signal = (u16)rssi * 257;
	else
		t->signal = 0;
	mutex_unlock(&bt->lock);
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
		"gpio=97/98/119 owned by hci_bcm (gpio_poke=1 to override)\n"
		"radio=/dev/radio0 V4L2 (tune/seek/signal); sysfs fm_* = debug\n"
		"metrics=fm_rssi, fm_snr\n"
		"rds=fm_rds (read drains the FIFO; PS/RT/PI/PTY decoded)\n"
		"raw=fm_reg \"r|w <reg> <len|val>\" per BlueTool FM_RDS_Command map\n"
		"hci=hci_cmd \"<opcode> [bytes]\" for any command incl. vendor\n"
		"audio_bcm=FC15 reg0x05 bit0x20 routes tuner audio to the PCM port\n"
		"audio_soc=IIS2 is only clocked while the capture PCM is open:\n"
		"  arecord -D hw:0,1 -f S16_LE -r 44100 -c 2 | aplay -D hw:0,0 -\n"
		"fm_default=94700 kHz deemph=75us (Canada)\n"
		"no_fm_a2dp=1 (local speakers/HP only)\n");
}
static DEVICE_ATTR_RO(patchram_info);

/* Signal quality. Both are one byte; RSSI is the tuner's own scale. */
static ssize_t fm_rssi_show(struct device *dev, struct device_attribute *a,
			    char *buf)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	u8 v = 0;
	int ret;

	mutex_lock(&bt->lock);
	ret = bcm_fm_r8(bt, FM_REG_RSSI, &v);
	mutex_unlock(&bt->lock);
	return ret ? ret : sysfs_emit(buf, "%u\n", v);
}
static DEVICE_ATTR_RO(fm_rssi);

static ssize_t fm_snr_show(struct device *dev, struct device_attribute *a,
			   char *buf)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	u8 v = 0;
	int ret;

	mutex_lock(&bt->lock);
	ret = bcm_fm_r8(bt, FM_REG_SNR, &v);
	mutex_unlock(&bt->lock);
	return ret ? ret : sysfs_emit(buf, "%u\n", v);
}
static DEVICE_ATTR_RO(fm_snr);

/* Drain the FIFO, then report whatever has been decoded so far. */
static ssize_t fm_rds_show(struct device *dev, struct device_attribute *a,
			   char *buf)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	u16 flag = 0;
	int ret;

	mutex_lock(&bt->lock);
	ret = bt->fm_on ? bcm_rds_poll(bt) : -ENODEV;
	if (bt->fm_on && bcm_fm_r16(bt, FM_REG_RDS_FLAG, &flag))
		flag = 0;
	ret = sysfs_emit(buf,
			 "on=%d poll=%d groups=%u flag=0x%04x\n"
			 "pi=0x%04x pty=%u\n"
			 "ps=%s\n"
			 "rt=%s\n",
			 bt->rds_on, ret, bt->rds_groups, flag,
			 bt->rds_pi, bt->rds_pty,
			 bt->rds_ps, bt->rds_rt);
	mutex_unlock(&bt->lock);
	return ret;
}

/* "1" / "0" toggles the decoder without disturbing the tuner. */
static ssize_t fm_rds_store(struct device *dev, struct device_attribute *a,
			    const char *buf, size_t count)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	unsigned int on;
	int ret;

	if (kstrtouint(buf, 0, &on))
		return -EINVAL;
	mutex_lock(&bt->lock);
	ret = bcm_rds_enable(bt, on);
	mutex_unlock(&bt->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(fm_rds);

/*
 * Raw register access, for walking the map in the BlueTool hcidef without
 * a driver change:
 *   echo "w 05 0061" > fm_reg     write (1 or 2 bytes by value width)
 *   echo "r 0f 1"    > fm_reg     read, result appears on the next read
 */
static ssize_t fm_reg_show(struct device *dev, struct device_attribute *a,
			   char *buf)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);

	if (!bt->reg_len)
		return sysfs_emit(buf, "no read pending\n");
	return sysfs_emit(buf, "0x%02x: %*ph\n",
			  bt->reg_addr, bt->reg_len, bt->reg_data);
}

static ssize_t fm_reg_store(struct device *dev, struct device_attribute *a,
			    const char *buf, size_t count)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	unsigned int reg, val;
	char op;
	int ret, n;

	n = sscanf(buf, " %c %x %x", &op, &reg, &val);
	if (n < 3 || reg > 0xff)
		return -EINVAL;

	mutex_lock(&bt->lock);
	if (op == 'r' || op == 'R') {
		if (!val || val > sizeof(bt->reg_data))
			ret = -EINVAL;
		else
			ret = bcm_fm_read(bt, reg, bt->reg_data, val);
		bt->reg_addr = reg;
		bt->reg_len = ret ? 0 : val;
	} else if (op == 'w' || op == 'W') {
		ret = (val > 0xff) ? bcm_fm_w16(bt, reg, val) :
				     bcm_fm_w8(bt, reg, val);
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&bt->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(fm_reg);

/*
 * Arbitrary HCI command, so the whole vendor surface is reachable from a
 * shell without BlueZ tools present:
 *   echo "fc15 0f 01 01" > hci_cmd   opcode then parameter bytes
 *   cat hci_cmd                       status and command-complete payload
 * The kernel owns hci0; this only borrows it for one synchronous command.
 */
static ssize_t hci_cmd_show(struct device *dev, struct device_attribute *a,
			    char *buf)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);

	if (!bt->hci_valid)
		return sysfs_emit(buf, "no command run\n");
	if (!bt->hci_rsp_len)
		return sysfs_emit(buf, "opcode 0x%04x: no payload\n",
				  bt->hci_opcode);
	return sysfs_emit(buf, "opcode 0x%04x: %*ph\n",
			  bt->hci_opcode, bt->hci_rsp_len, bt->hci_rsp);
}

static ssize_t hci_cmd_store(struct device *dev, struct device_attribute *a,
			     const char *buf, size_t count)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	u8 params[BCM_HCI_PARAM_MAX];
	unsigned int opcode, v, plen = 0;
	struct hci_dev *hdev;
	struct sk_buff *skb;
	const char *p = buf;
	int used, ret = 0;

	if (sscanf(p, " %x%n", &opcode, &used) != 1 || opcode > 0xffff)
		return -EINVAL;
	p += used;
	while (plen < sizeof(params) && sscanf(p, " %x%n", &v, &used) == 1) {
		if (v > 0xff)
			return -EINVAL;
		params[plen++] = (u8)v;
		p += used;
	}

	hdev = hci_dev_get(0);
	if (!hdev)
		return -ENODEV;
	if (!test_bit(HCI_UP, &hdev->flags)) {
		hci_dev_put(hdev);
		return -ENETDOWN;
	}

	mutex_lock(&bt->lock);
	skb = __hci_cmd_sync(hdev, opcode, plen, plen ? params : NULL,
			     HCI_CMD_TIMEOUT);
	bt->hci_opcode = opcode;
	bt->hci_valid = true;
	if (IS_ERR(skb)) {
		bt->hci_rsp_len = 0;
		ret = PTR_ERR(skb);
	} else {
		bt->hci_rsp_len = min_t(unsigned int, skb->len,
					sizeof(bt->hci_rsp));
		memcpy(bt->hci_rsp, skb->data, bt->hci_rsp_len);
		kfree_skb(skb);
	}
	mutex_unlock(&bt->lock);
	hci_dev_put(hdev);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(hci_cmd);

static struct attribute *bcm_attrs[] = {
	&dev_attr_power_on.attr,
	&dev_attr_patchram.attr,
	&dev_attr_patchram_info.attr,
	&dev_attr_fm_power.attr,
	&dev_attr_fm_tune.attr,
	&dev_attr_fm_seek.attr,
	&dev_attr_fm_rssi.attr,
	&dev_attr_fm_snr.attr,
	&dev_attr_fm_rds.attr,
	&dev_attr_fm_reg.attr,
	&dev_attr_hci_cmd.attr,
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
	/*
	 * Turn the tuner off, but leave the control pins alone unless this
	 * driver was the one driving them -- otherwise unloading it drops
	 * REG_ON and takes hci0 down with it.
	 */
	if (bt->fm_on)
		bcm_fm_power_off(bt);
	if (gpio_poke)
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
