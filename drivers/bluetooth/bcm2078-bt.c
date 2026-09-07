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
#include <linux/apple-n31.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
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
/*
 * The PMIC driver is a module and this file is built in, so the link
 * can only go the other way: expose a hook and let gpio-d1830 register
 * its rail control when it probes. Everything here still works with
 * nothing registered -- the rails simply stay as the bootloader left
 * them, which is the behaviour we had before.
 */
static int (*bcm_bt_rails_fn)(bool on);

/*
 * What we asked for while nobody was listening.
 *
 * This driver is built in and probes at about t=2.4s. gpio-d1830, which
 * owns the PMIC rails, is a module that userspace loads later -- it
 * registered here at t=7.1s on the boot that exposed this. In between,
 * every bcm_bt_rails() call returned -ENODEV and the rails were simply
 * never switched on, so the controller had no supply. hci_bcm sent
 * 0xFC18 into a dead part at t=4.8s, timed out, and gave up two seconds
 * before the rails became available.
 *
 * The hook was always the right shape for a built-in talking to a module;
 * it just had no memory. Record the last request and apply it when the
 * provider finally shows up.
 */
static bool bcm_bt_rails_want;
static bool bcm_bt_rails_pending;

void bcm2078_register_bt_rails(int (*fn)(bool on))
{
	bcm_bt_rails_fn = fn;

	if (fn && bcm_bt_rails_pending) {
		int ret = fn(bcm_bt_rails_want);

		bcm_bt_rails_pending = false;
		pr_info("bcm2078-bt: rails provider arrived late; applied %s: %d\n",
			bcm_bt_rails_want ? "on" : "off", ret);
	}
}
EXPORT_SYMBOL_GPL(bcm2078_register_bt_rails);

/*
 * The enable line, which lives on the PMIC and not on a SoC pad.
 *
 * Same hook shape as the rails above and for the same reason, but with no
 * deferred replay: the steps are ordered against pad writes and delays in
 * bcm_power_seq(), so applying one late would put it in the wrong place.
 * If the provider is not up yet the sequence is not run, and the caller is
 * told, rather than half of it happening at the wrong time.
 */
static int (*bcm_bt_enable_fn)(unsigned int step);

void bcm2078_register_bt_enable(int (*fn)(unsigned int step))
{
	bcm_bt_enable_fn = fn;
}
EXPORT_SYMBOL_GPL(bcm2078_register_bt_enable);


/*
 * The rail, taken as a regulator.
 *
 * This is the ordering fix. The hook below can only report that no provider
 * exists yet; a regulator makes the kernel wait for one. devm_regulator_get()
 * returns -EPROBE_DEFER while gpio-d1830 is still unloaded, probe is retried
 * once it registers, and by the time this driver runs the rail is reachable.
 *
 * Power belongs here, in the chip driver, and not in the UART-to-HCI bridge:
 * the bridge should move bytes and nothing else. hci_bcm carries supplies and
 * shutdown-gpios upstream because on most boards it is also the chip driver,
 * which is not true on this one.
 */
static struct regulator *bcm_bt_vreg;

/*
 * The one instance, for the exported mixer hooks.
 *
 * There is exactly one BCM2078 on this board and its node is a fixed
 * soc:bcm2078-companion, so a singleton is honest rather than a shortcut --
 * the alternative would be handing the machine driver a pointer it has no
 * other use for. Cleared on remove so a mixer control cannot outlive it.
 */
static struct bcm2078_bt *bcm_bt_singleton;

/*
 * Whether WE hold an enable on bcm_bt_vreg.
 *
 * regulator_enable/disable are refcounted per consumer handle, and calling
 * disable without a matching enable is not merely ignored: _regulator_disable
 * WARNs with a full backtrace and returns -EIO. That is exactly what an
 * unbind produced --
 *
 *	_regulator_disable from regulator_disable
 *	regulator_disable from bcm_power_off
 *	bcm2078-bt: bt rail disable failed: -5
 *
 * -- because the enable had gone down the legacy-hook path (the regulator
 * was not acquired yet at the time) while the disable found the pointer set
 * and went to the regulator. Track it here rather than inferring it, so the
 * two paths cannot disagree.
 */
static bool bcm_bt_vreg_on;

static int bcm_bt_rails(bool on)
{
	if (bcm_bt_vreg) {
		int ret;

		if (on == bcm_bt_vreg_on)
			return 0;

		ret = on ? regulator_enable(bcm_bt_vreg)
			 : regulator_disable(bcm_bt_vreg);

		if (ret)
			pr_warn("bcm2078-bt: bt rail %s failed: %d\n",
				on ? "enable" : "disable", ret);
		else
			bcm_bt_vreg_on = on;
		return ret;
	}

	if (!bcm_bt_rails_fn) {
		/* Remember it; the provider may still be loading. */
		bcm_bt_rails_want = on;
		bcm_bt_rails_pending = true;
		pr_info("bcm2078-bt: rails %s deferred, no provider yet\n",
			on ? "on" : "off");
		return -ENODEV;
	}
	return bcm_bt_rails_fn(on);
}

#define BCM_GPIO_PHYS		0x3cf00000UL
#define BCM_GPIOCMD_OFF		0x1e0

#define BCM_GPIO_A		0x61	/* 97 — shutdown / REG_ON */
#define BCM_GPIO_B		0x62	/* 98 — device-wakeup */
#define BCM_GPIO_C		0x77	/* 119 — host-wakeup */
/*
 * 0xC8 = 200 is not a no-op. sub_17D4DC selects the Bluetooth power
 * control by board variant:
 *
 *   variant 1 or 2:  sub_428F70(0xC8, 1); sub_43D38C(0xC8, 1, 1);
 *   variant 5:       sub_43D38C(0x46, 1, 1); sub_428F70(0x46, 1);
 *
 * and sphwBluetooth_Init then drives 0x46 = 70 high unconditionally.
 * Both orders pair the pad write with sub_428F70, which sets the pad's
 * +0x0C bit -- something a gpiod output cannot express, so hci_bcm
 * driving shutdown-gpios does not cover it.
 */
#define BCM_GPIO_NOP		0xC8
#define BCM_GPIO_PWR		0x46	/* 70 — power control, all variants */
/*
 * The UART pads, both of them. sub_177AE0 configures 80 with (1, 1) and
 * 81 with (2, 0) plus a data write, so they are not interchangeable and
 * the old single BCM_GPIO_UART hid that.
 */
#define BCM_GPIO_UART_TX	0x50	/* 80 */
#define BCM_GPIO_UART_RX	0x51	/* 81 */
#define BCM_GPIO_UART		BCM_GPIO_UART_TX
#define BCM_GPIO_PWR_ALT	0xC8	/* 200 — variants 1 and 2 */
#define BCM_MODE_POWER		2
#define BCM_MODE_CLEAR		0xFFFE

#define HCI_OP_FC15		0xFC15
/* HCI_Write_BD_Addr. Occurs once in OSOS, so stock programs it too. */
#define HCI_OP_WRITE_BD_ADDR_VS	0xFC01

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
/*
 * 0x29 is never written by the stock firmware.
 *
 * Every FM_RDS_Command the stock image can issue passes through one 16-byte
 * queue entry builder, sub_4290C4 at 0x004290C4, whose first word packs
 * data << 24 | rw << 16 | reg << 8 | param_len. There are eleven call sites
 * image-wide and register 0x29 is not among them, so whatever this register
 * governs -- PCM slave configuration, on the reading that named it -- the
 * part is shipped with it already correct. It is kept named because a bare
 * 0x29 in a future trace should be recognisable, not because anything here
 * should write it.
 */
#define FM_REG_SLAVE_CONFIG	0x29
/*
 * 0x4D is read, never written.
 *
 * sub_DD458 at 0x000DD458 issues 0x01014D03: reg 0x4D, rw 1, one byte. Stock
 * calls it after every AUDIO_CTRL write and does nothing with the answer that
 * this image reveals. This driver used to write 0x01 here, on the reading
 * that it was the PCM route enable; it is not, and the route lives in
 * AUDIO_CTRL below.
 */
#define FM_REG_ROUTE_PCM	0x4d
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

/*
 * RDS_FLAG bits, read-to-clear. Only the two this driver acts on are named,
 * and both are inferred from live readings on 2026-09-04 rather than from a
 * register document: the flag read 0x0261 on a freshly tuned station with an
 * RDS group waiting, and 0x0000 with neither pending. Both bits are among the
 * four stock arms in RDS_MASK = 0x1203.
 */
#define FM_FLAG_TUNE_COMPLETE	0x0001
#define FM_FLAG_RDS_AVAIL	0x0200

/* Tune settle: SEARCH_TUNE starts the retune, it does not finish it. */
#define FM_TUNE_SETTLE_TRIES	40
#define FM_TUNE_SETTLE_MS	25

/*
 * Raw group ring. 64 groups is about 5.5 s of a full RDS stream (11.4
 * groups/s), which is long enough that a once-a-second reader never loses one
 * and short enough to stay a rounding error in this struct.
 */
#define FM_RDS_RING	64

struct bcm_rds_raw {
	u16 a, b, c, d;
};

/* RDS_SYSTEM bits. */
#define FM_SYSTEM_FM_ON		0x01
#define FM_SYSTEM_RDS_ON	0x02

/* FM_CTRL bits. Band select 0 = 87.5-108 MHz, 1 = 76-90 MHz. */
#define FM_CTRL_BAND_JAPAN	0x01
#define FM_CTRL_STEREO_AUTO	0x02
#define FM_CTRL_STEREO_MANUAL	0x04
#define FM_CTRL_STEREO_BLEND	0x08
#define FM_CTRL_INJECTION	0x10

/*
 * AUDIO_CTRL, which is where the FM audio route actually lives.
 *
 * Stock never composes this register from bit names. sub_DD334 at 0x000DD334
 * writes one of exactly two 16-bit values and nothing else:
 *
 *	route on   0x0060    (sub_42A8C, "BT_KEY_FM_RADIO_AUDIO_ROUTE" = "0N")
 *	route off  0x0001    (sub_157784, the same key = "0F", and the value
 *	                      the FM power-on in sub_DD136 leaves behind)
 *
 * They are alternatives, not a bitfield to be mixed: sub_DD334 is a single
 * ternary, 96 or 1. This driver's default was 0x0061 -- the "on" value with
 * the "off" value ORed into it -- assembled from bit names that came from a
 * register map rather than from the firmware, and that is a configuration
 * stock never writes.
 *
 * MANUAL_MUTE is kept as a bit because the ALSA mixer needs somewhere to put
 * mute and the FM path has no gain stage of its own; it is ORed onto whichever
 * of the two values is current, so the route is never disturbed by muting.
 */
/* Top of VOLUME_CTRL's own 0..256 range; the chip powers up above it. */
#define FM_VOLUME_UNITY		0x0100
#define FM_AUDIO_CTRL_ROUTE_OFF	0x0001
#define FM_AUDIO_CTRL_ROUTE_PCM	0x0060
#define FM_AUDIO_MANUAL_MUTE	0x0002

/* Largest RDS read we will ask for in one go. */
#define FM_RDS_READ_MAX		60
#define FM_RDS_TEXT_MAX		64
#define FM_RDS_PTYN_MAX		8
/* RadioText+ application id, and "no group announced yet". */
#define FM_RDS_AID_RTPLUS	0x4bd7
#define FM_RDS_AGT_NONE		0xffff
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
 * The PCM bit clock, per tuned station.
 *
 * Stock's FM audio enable, sub_42A8C at 0x00042A8C, does not use one clock.
 * It picks between three and programs the SoC's IIS2 dividers for whichever
 * it chose, then turns the route on:
 *
 *	4.8 MHz   default
 *	8.0 MHz   frequency exactly in the 18-entry table at 0x0891DAC8
 *	1.6 MHz   95.9, 96.0 or 96.1 MHz
 *
 * all divided from the same 24 MHz source, and all divided again by a literal
 * 32000 to reach CLKDIV. The selector is sub_3B82C at 0x0003B82C, called on
 * every tune from sub_348B8, so the clock moves with the station rather than
 * being set once.
 *
 * What the table is becomes obvious once it is written out: every entry is
 * 4.8 MHz times an integer, plus or minus 100 kHz.
 *
 *	16 x 4.8 = 76.8    76.7  76.8  76.9
 *	17 x 4.8 = 81.6    81.5  81.6  81.7
 *	18 x 4.8 = 86.4    86.3  86.4  86.5
 *	19 x 4.8 = 91.2    91.1  91.2  91.3
 *	21 x 4.8 = 100.8  100.7 100.8 100.9
 *	22 x 4.8 = 105.6  105.5 105.6 105.7
 *
 * These are the channels where the default PCM clock's own harmonic sits on
 * the carrier, and stock moves the clock rather than accept the interference.
 * 20 x 4.8 is 96.0, which is missing from the table because it gets the third
 * rate instead.
 *
 * The table is stock's, in stock's units of 10 kHz.
 */
#define BCM_FM_BITCLK_DEFAULT	4800000u
#define BCM_FM_BITCLK_TABLE	8000000u
#define BCM_FM_BITCLK_96MHZ	1600000u

static const u16 bcm_fm_bitclk_8mhz[] = {
	7670, 7680, 7690, 8150, 8160, 8170, 8630, 8640, 8650,
	9110, 9120, 9130, 10070, 10080, 10090, 10550, 10560, 10570,
};

/*
 * Off, because stock's Bluetooth power-on does not touch these pads.
 *
 * This was turned on under the reading that pad 97 is the controller's
 * REG_ON and that "stock sets all three to mode 2 as part of its power
 * sequence". Neither holds. The mode-2 trio 97/98/119 -- 0x61, 0x62, 0x77 --
 * appears in exactly one function image-wide:
 *
 *	sub_15DD5C, keyed on "BT_KEY_FM_RADIO_POWER"
 *	  a1 == 2 (FM on):   43D38C(0xC8, 0, 0), 428F70(0xC8, 1),
 *			     43D38C(0x61, 2, 0), (0x62, 2, 0), (0x77, 2, 0)
 *	  a1 == 1 (FM off):  43D38C(0xC8, 0xFFFE, 0), and the same three
 *			     released to 0xFFFE
 *
 * That is the FM tuner claiming the IIS2 PCM pads, which is why
 * s5l8740-i2s claims them too, and it is not reachable from
 * sphwBluetooth_Init. The Bluetooth power-on is pad 70 plus the PMIC
 * writes -- see bcm_bt_power_up() -- and the enable this driver was
 * missing turned out to be PMIC register 0x4B, not pad 97.
 *
 * Driving them here also has a cost beyond being wrong: they are the
 * capture bus, and one early attempt at it correlated with a reset back to
 * RetailOS. gpio_poke=1 restores the old behaviour for bring-up
 * comparisons.
 */
static bool gpio_poke;
module_param(gpio_poke, bool, 0644);
MODULE_PARM_DESC(gpio_poke,
		 "Drive the BCM control pins directly (default N; hci_bcm owns them)");

static u8 rds_wline = 12;
module_param(rds_wline, byte, 0644);
MODULE_PARM_DESC(rds_wline, "RDS FIFO watermark in blocks (default 12)");

static u16 fm_audio_ctrl0 = FM_AUDIO_CTRL_ROUTE_PCM;
module_param(fm_audio_ctrl0, ushort, 0644);
MODULE_PARM_DESC(fm_audio_ctrl0,
		 "FC15 reg0x05 value for route-on (stock writes 0x0060)");

struct bcm2078_bt {
	struct device *dev;
	void __iomem *gpio;
	void __iomem *gpiocmd;
	bool powered;
	bool fm_on;
	bool rds_on;
	/* AUDIO_CTRL state: which of stock's two values is currently written. */
	bool fm_route_on;
	bool fm_muted;
	bool tune_settled;
	bool bd_addr_done;
	/* Serialises every HCI command; see bcm_hci_cmd(). */
	struct mutex cmd_lock;
	bool cmd_settle;
	u16 rds_flag;
	struct bcm_rds_raw rds_ring[FM_RDS_RING];
	unsigned int rds_ring_head;
	unsigned int rds_ring_used;
	unsigned int rds_ring_drop;
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
	/* 10A: programme type name, eight characters in two segments. */
	char rds_ptyn[FM_RDS_PTYN_MAX + 1];
	char rds_ptyn_build[FM_RDS_PTYN_MAX];
	/*
	 * RadioText+, which has no fixed group of its own: a 3A announcement
	 * says which group carries it, so the group has to be learnt before
	 * anything can be decoded from it. 0xffff until one arrives.
	 */
	u16 rds_rtp_agt;
	u8 rds_rtp_type[2];
	u8 rds_rtp_start[2];
	u8 rds_rtp_len[2];
	bool rds_rtp_valid;
	bool rds_rtp_running;
	u8 rds_rtp_toggle;
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
	if (mode == 1)
		cmd = val ? 15 : 14;
	else if (mode == BCM_MODE_CLEAR)
		cmd = 0;
	else
		cmd = (u8)mode;

	/*
	 * sub_43D38C sets DIR for every mode but 0xFFFE, mode 1 included:
	 * the a2 == 1 arm picks command 14 or 15 and falls through to the
	 * shared "DIR |= bit". Skipping it here left pad 70 an input, so
	 * the level never reached the pin.
	 */
	dir = readl(bank + 0x14);
	if (mode == BCM_MODE_CLEAR)
		dir &= ~BIT(pin);
	else
		dir |= BIT(pin);
	writel(dir, bank + 0x14);
	writel(((gpio >> 3) << 16) | (pin << 8) | cmd, bt->gpiocmd);
}

/* sub_428F70(gpio, on): the pad's +0x0C bit, paired with every
 * sub_43D38C power write in sub_17D4DC. */
static void bcm_428F70(struct bcm2078_bt *bt, unsigned int gpio, int on)
{
	void __iomem *bank;
	u32 pin, v;

	if (!bt->gpio)
		return;
	bank = bt->gpio + 32 * (gpio >> 3);
	pin = gpio & 7;
	v = readl(bank + 0x0c);
	if (on)
		v |= BIT(pin);
	else
		v &= ~BIT(pin);
	writel(v, bank + 0x0c);
}

/*
 * The +0x0C half of the power sequence, kept separate from the mode-2
 * level pokes below. Those drive pads 97/98/119, which are the IIS2 PCM
 * bus, and doing that early once correlated with resets back to
 * RetailOS -- hence gpio_poke defaulting off. This is only an input
 * enable on the power pad, so it is safe to run whenever the caller
 * asks for power, and hci_bcm cannot do it through gpiod.
 */
static int bcm_bt_enable(struct bcm2078_bt *bt, unsigned int step)
{
	int ret;

	if (!bcm_bt_enable_fn) {
		dev_warn(bt->dev,
			 "PMIC enable step %u skipped: gpio-d1830 not loaded yet -- the part stays held in reset\n",
			 step);
		return -ENODEV;
	}
	ret = bcm_bt_enable_fn(step);
	if (ret)
		dev_warn(bt->dev, "PMIC enable step %u failed: %d\n",
			 step, ret);
	return ret;
}

static void bcm_power_pad_gate(struct bcm2078_bt *bt, int on)
{
	bcm_428F70(bt, BCM_GPIO_PWR, on);
	dev_dbg(bt->dev, "pad %#x +0x0C -> %d\n", BCM_GPIO_PWR, on);
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

/*
 * Send one command to hci0 and discard the reply.
 *
 * The hci0-up guard is why this is shared rather than inlined per caller: the
 * failure it catches is specific enough to be worth naming once. hci_bcm can
 * have a fully initialised controller -- patched, identified, answering -- while
 * the HCI device is still DOWN, because nothing has issued HCIDEVUP. There is no
 * bluez in this initramfs, so that is the normal state after boot, and it is why
 * n31-hciup exists.
 */
/*
 * One command at a time, and a settling gap after any that times out.
 *
 * A timed-out FC15 does not merely fail, it poisons the link. The HCI core
 * matches replies to waiters by opcode, so when __hci_cmd_sync gives up and the
 * part answers a moment later, that late reply is handed to the *next* FC15's
 * waiter. Every reply after it is then one behind, which is how a single
 * hiccup turned into minutes of
 *
 *	Bluetooth: hci0: command 0xfc15 tx timeout
 *	Bluetooth: hci0: Frame reassembly failed (-84)
 *
 * ending in a reboot. Two changes stop that.
 *
 * cmd_lock serialises every command this driver sends. Callers reach here from
 * the FM sysfs attributes, the V4L2 radio, the RDS poll and an ALSA mixer
 * handler, and those hold different locks or none -- the mute control is a
 * kcontrol put, which shares nothing with the poll worker. Without this two of
 * them can be in flight together, and the second is guaranteed to collect the
 * first's reply.
 *
 * And after a timeout the next command waits out the in-flight answer instead
 * of racing it. Nothing can be done about the reply that is already coming, but
 * arriving with no waiter it is dropped by the core, which is exactly what
 * should happen to it. The alternative -- pressing straight on -- is what
 * shifted the stream.
 */
#define BCM_CMD_SETTLE_MS	300

static int bcm_hci_cmd(struct bcm2078_bt *bt, u16 opcode,
		       const u8 *payload, u8 plen)
{
	struct hci_dev *hdev;
	struct sk_buff *skb;
	int ret;

	mutex_lock(&bt->cmd_lock);
	if (bt->cmd_settle) {
		msleep(BCM_CMD_SETTLE_MS);
		bt->cmd_settle = false;
	}

	hdev = hci_dev_get(0);
	if (!hdev) {
		dev_warn_ratelimited(bt->dev,
				     "0x%04x: no hci0 — bring up hci_bcm first\n",
				     opcode);
		mutex_unlock(&bt->cmd_lock);
		return -ENODEV;
	}
	if (!test_bit(HCI_UP, &hdev->flags)) {
		hci_dev_put(hdev);
		dev_warn_ratelimited(bt->dev,
				     "0x%04x: hci0 down — run n31-hciup\n",
				     opcode);
		mutex_unlock(&bt->cmd_lock);
		return -ENETDOWN;
	}

	skb = __hci_cmd_sync(hdev, opcode, plen, payload, HCI_CMD_TIMEOUT);
	hci_dev_put(hdev);
	if (IS_ERR(skb)) {
		ret = PTR_ERR(skb);
		/*
		 * A timeout leaves an answer in flight. Make the next command
		 * wait for it rather than collect it.
		 */
		if (ret == -ETIMEDOUT) {
			bt->cmd_settle = true;
			dev_warn_ratelimited(bt->dev,
					     "0x%04x timed out; settling %u ms before the next command\n",
					     opcode, BCM_CMD_SETTLE_MS);
		} else {
			dev_dbg(bt->dev, "0x%04x plen=%u → %d\n",
				opcode, plen, ret);
		}
		mutex_unlock(&bt->cmd_lock);
		return ret;
	}
	dev_dbg(bt->dev, "0x%04x plen=%u → OK len=%u\n",
		opcode, plen, skb->len);
	kfree_skb(skb);
	mutex_unlock(&bt->cmd_lock);
	return 0;
}

static int bcm_fc15(struct bcm2078_bt *bt, const u8 *payload, u8 plen)
{
	return bcm_hci_cmd(bt, HCI_OP_FC15, payload, plen);
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

	/*
	 * Reply shapes.
	 *
	 * 1 + len and 3 + len were the two this knew about. The part also
	 * answers 2 + len, which was rejected as -EPROTO -- and rejecting it is
	 * worse than it sounds, because the caller then retries and the stream
	 * drifts: measured on 2026-09-04, RSSI reads alternated between a value
	 * and `command 0xfc15 tx timeout` for minutes, with 144 KB received on
	 * UART1 for a few dozen register reads.
	 *
	 * The 2 + len form is one status byte, the payload, then one trailing
	 * byte. Reading reg 0x0F (RSSI) for len 1 returned `00 10 00`, `00 0a 00`,
	 * `00 01 00` and `00 09 00` across the band -- a zero status, a value
	 * that moves, and a constant zero after it. So the payload is at offset
	 * 1 and the tail is ignored, exactly as for 1 + len.
	 *
	 * Which also means the RSSI numbers this driver has been reporting were
	 * not RSSI. 16, 10, 1 and 9 are plausible on this part's scale; the
	 * 170-199 range that came out of a band sweep earlier was mis-framed
	 * replies being read as data. A sweep taken with this fix in place is the
	 * first one worth believing.
	 */
	if (skb->len == 1u + len || skb->len == 2u + len)
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
	dev_dbg(bt->dev, "FC15 read reg 0x%02x len %u: %*ph\n",
		reg, len, min_t(int, skb->len, 16), skb->data);
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
/*
 * The RDS FIFO record, derived from the bytes the chip actually returns.
 *
 * Three bytes per block, and the status comes FIRST, not last:
 *
 *	[status][value hi][value lo]
 *
 * with the block index in bits 5:4 and the low nibble zero on a clean
 * block. A dump off a station transmitting its name reads
 *
 *	00 ce c4  10 00 0c  20 e0 cd  30 56 4f
 *	00 ce c4  10 00 09  20 e0 cd  30 41 52
 *	00 ce c4  10 00 0a  20 e0 cd  30 2d 46
 *	00 ce c4  10 00 0f  20 e0 cd  30 4d 20
 *
 * which decodes as PI 0xCEC4, block B 0x000c/9/a/f -- group type 0,
 * version A, segments 0 to 3 -- and block D carrying "VO", "AR", "-F",
 * "M ": the Program Service name VOAR-FM, assembled in segment order.
 *
 * The previous reading of this had the status last and the block index in
 * bits 2:0 with an error bit at 0x80. That turned every group into an
 * identical bogus 1A with a PI of 0x304d -- which is two of the name's own
 * characters, "0M", read as a block -- and no PS or RadioText ever
 * appeared, because a station's 0A groups were never recognised as 0A.
 */
#define FM_RDS_REC_LEN		3
#define FM_RDS_BLK_SHIFT	4
/*
 * A read always returns the full 60 bytes and pads the slots the FIFO could
 * not fill with 7c ff ff -- status 0x7c, value 0xffff. Its block index is 7,
 * outside the four that exist, so the padding identifies itself and there is
 * no need to ask the chip how much it had. Masking the index to two bits
 * instead would fold that 7 onto block D and feed 0xffff into the group.
 */
#define FM_RDS_BLK_EMPTY	7
#define FM_RDS_BLK_A		0
#define FM_RDS_BLK_B		1
#define FM_RDS_BLK_C		2
#define FM_RDS_BLK_D		3
/* Low nibble: nonzero means the chip flagged errors in that block. */
#define FM_RDS_ERR_MASK		0x0f

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

	/*
	 * Keep the group itself, not just what this decoder made of it.
	 *
	 * PS and RT are the two things worth decoding in the kernel and they
	 * are also the two that stay empty longest -- PS needs all four
	 * segments, RT up to sixteen -- so "nothing yet" and "nothing ever"
	 * look identical from sysfs. A ring of the raw blocks distinguishes
	 * them, and it is what a userspace tool wants anyway: group types this
	 * driver ignores (1A, 3A, 4A clock-time, 8A TMC) are all in here.
	 *
	 * Oldest-dropped rather than newest-dropped: a reader that falls behind
	 * should lose history, not the group that just arrived.
	 */
	bt->rds_ring[bt->rds_ring_head] = (struct bcm_rds_raw){
		blk[0], blk[1], blk[2], blk[3]
	};
	bt->rds_ring_head = (bt->rds_ring_head + 1) % FM_RDS_RING;
	if (bt->rds_ring_used < FM_RDS_RING)
		bt->rds_ring_used++;
	else
		bt->rds_ring_drop++;

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
	} else if (type == 10 && !ver) {
		/*
		 * 10A: the Programme Type Name, eight characters that qualify
		 * the numeric PTY -- "Rock" says less than "OZ ROCK". Four
		 * characters per group in blocks C and D, two segments, and the
		 * segment is the low bit of block B.
		 */
		unsigned int seg = blk[1] & 1;
		unsigned int base = seg * 4;

		bt->rds_ptyn_build[base] = bcm_rds_char(blk[2] >> 8);
		bt->rds_ptyn_build[base + 1] = bcm_rds_char(blk[2] & 0xff);
		bt->rds_ptyn_build[base + 2] = bcm_rds_char(blk[3] >> 8);
		bt->rds_ptyn_build[base + 3] = bcm_rds_char(blk[3] & 0xff);
		if (seg == 1) {
			memcpy(bt->rds_ptyn, bt->rds_ptyn_build,
			       FM_RDS_PTYN_MAX);
			bt->rds_ptyn[FM_RDS_PTYN_MAX] = 0;
		}
	} else if (type == 3 && !ver) {
		/*
		 * 3A announces an Open Data Application: block D is the
		 * application id and the low five bits of block B are the group
		 * that will carry it, as a type in bits 4:1 and a version in
		 * bit 0. RadioText+ is 0x4BD7 and it is not assigned a fixed
		 * group, so this is the only way to know where to look for it;
		 * on the station this was written against it lands on 11A, but
		 * that is the station's choice rather than a constant.
		 */
		if (blk[3] == FM_RDS_AID_RTPLUS)
			bt->rds_rtp_agt = blk[1] & 0x1f;
	} else if (bt->rds_rtp_agt != FM_RDS_AGT_NONE &&
		   ((type << 1) | ver) == bt->rds_rtp_agt) {
		/*
		 * RadioText+. Two tags per group, each naming a content type
		 * and a run of characters inside the RadioText that is already
		 * being assembled, so this stores offsets rather than text and
		 * the reader slices the current RadioText with them.
		 *
		 *   B  bit 4    item toggle
		 *      bit 3    item running
		 *      bits 2:0 content type 1, high three bits of six
		 *   C  bits 15:13  content type 1, low three
		 *      bits 12:7   start of tag 1
		 *      bits 6:1    length of tag 1
		 *      bit 0       content type 2, high bit of six
		 *   D  bits 15:11  content type 2, low five
		 *      bits 10:5   start of tag 2
		 *      bits 4:0    length of tag 2
		 *
		 * The lengths are character counts as they stand, not counts less
		 * one: a group carrying "INTO THE GREAT WIDE OPEN" as its title
		 * gives 24, which is exactly that string.
		 */
		bt->rds_rtp_toggle = (blk[1] >> 4) & 1;
		bt->rds_rtp_running = (blk[1] >> 3) & 1;
		bt->rds_rtp_type[0] = ((blk[1] & 0x07) << 3) | (blk[2] >> 13);
		bt->rds_rtp_start[0] = (blk[2] >> 7) & 0x3f;
		bt->rds_rtp_len[0] = (blk[2] >> 1) & 0x3f;
		bt->rds_rtp_type[1] = ((blk[2] & 0x01) << 5) | (blk[3] >> 11);
		bt->rds_rtp_start[1] = (blk[3] >> 5) & 0x3f;
		bt->rds_rtp_len[1] = blk[3] & 0x1f;
		bt->rds_rtp_valid = true;
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
	u16 flag = 0;
	int ret;

	/*
	 * Ask the flag register before draining the FIFO.
	 *
	 * The FIFO read is a vendor command with a fixed-size reply, so an
	 * empty FIFO does not come back short -- it does not come back at all,
	 * and __hci_cmd_sync eats its full timeout. Measured 2026-09-04 on a
	 * station whose RDS was decoding: the first poll returned a group and
	 * flag=0x0261; the second returned -110 with flag=0x0000, having
	 * blocked for two seconds to learn that nothing had arrived yet. A
	 * poller on a timer would spend all its time in that timeout.
	 *
	 * Bit 9 is the RDS bit: it is set in 0x0261 when a group was waiting,
	 * clear in 0x0000 when none was, and it is one of the four bits stock's
	 * mask 0x1203 arms. Reading the flag also clears it, which is why this
	 * happens once per poll and the value is passed on to the caller.
	 */
	if (bcm_fm_r16(bt, FM_REG_RDS_FLAG, &flag))
		return -EIO;
	bt->rds_flag = flag;
	if (!(flag & FM_FLAG_RDS_AVAIL))
		return 0;

	ret = bcm_fm_read(bt, FM_REG_RDS_DATA, buf, sizeof(buf));
	if (ret)
		return ret;

	for (i = 0; i + FM_RDS_REC_LEN <= sizeof(buf); i += FM_RDS_REC_LEN) {
		u8 st = buf[i];
		unsigned int off = st >> FM_RDS_BLK_SHIFT;
		u16 val = ((u16)buf[i + 1] << 8) | buf[i + 2];

		/* Padding: everything from here on is empty slots. */
		if (off > FM_RDS_BLK_D)
			break;
		if (st & FM_RDS_ERR_MASK)
			continue;
		if (off == FM_RDS_BLK_A) {
			memset(have, 0, sizeof(have));
			blk[0] = val;
			have[0] = true;
			continue;
		}
		if (!have[0])
			continue;
		blk[off] = val;
		have[off] = true;
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

/* Defined below; the radio reset needs them and they need the FM block. */
static void bcm_power_off(struct bcm2078_bt *bt);
static int bcm_power_on(struct bcm2078_bt *bt);
static int bcm_fm_power_off(struct bcm2078_bt *bt);

/* Defined with the rest of the BD-address handling, below the FM block. */
static int bcm_write_bd_addr(struct bcm2078_bt *bt, const unsigned int m[6]);
static int bcm_bd_addr_from_chosen(struct bcm2078_bt *bt, unsigned int m[6]);

/*
 * Program the BD address on the first command that finds hci0 up.
 *
 * Not in probe: at probe hci0 is registered but DOWN, and 0xFC01 needs it up.
 * Not on every command either -- once is stock's behaviour and repeating it
 * would be a write to the radio for no reason. Failure is logged and not fatal;
 * a controller with a default address still works, it just is not this device.
 */
static void bcm_bd_addr_apply_once(struct bcm2078_bt *bt)
{
	unsigned int m[6];

	if (bt->bd_addr_done)
		return;
	if (bcm_bd_addr_from_chosen(bt, m))
		return;			/* no bootloader copy: sysfs override only */
	bt->bd_addr_done = true;
	if (bcm_write_bd_addr(bt, m))
		return;
	dev_info(bt->dev,
		 "BD address %02x:%02x:%02x:%02x:%02x:%02x from /chosen\n",
		 m[0], m[1], m[2], m[3], m[4], m[5]);
}

/*
 * The IIS2 side of the FM audio enable, filled in by s5l8740-i2s.
 *
 * This driver is built into the kernel and s5l8740-i2s is a module, so the
 * hook is published here and registered there -- the same shape as
 * n31_backlight_register_wled(). The tuner has to be the one that calls it:
 * the bit clock stock chooses depends on the station, and the station is here.
 * A NULL hook is normal (no capture driver loaded) and is not an error; the
 * route still goes on, at whatever bit clock the port was left with.
 */
static int (*bcm_fm_pcm_clk_hook)(unsigned int bitclk_hz);

void bcm2078_register_fm_pcm_clk(int (*fn)(unsigned int bitclk_hz))
{
	WRITE_ONCE(bcm_fm_pcm_clk_hook, fn);
}
EXPORT_SYMBOL_GPL(bcm2078_register_fm_pcm_clk);

/* khz -> stock's 10 kHz units, then stock's table. */
static unsigned int bcm_fm_bitclk_for(unsigned int khz)
{
	unsigned int f10 = khz / 10;
	unsigned int i;

	if (f10 == 9590 || f10 == 9600 || f10 == 9610)
		return BCM_FM_BITCLK_96MHZ;
	for (i = 0; i < ARRAY_SIZE(bcm_fm_bitclk_8mhz); i++) {
		if (bcm_fm_bitclk_8mhz[i] > f10)
			break;
		if (bcm_fm_bitclk_8mhz[i] == f10)
			return BCM_FM_BITCLK_TABLE;
	}
	return BCM_FM_BITCLK_DEFAULT;
}

/* Stock's two AUDIO_CTRL values, plus mute, which is this driver's addition. */
static u16 bcm_fm_audio_ctrl(struct bcm2078_bt *bt, bool route_on)
{
	u16 v = route_on ? fm_audio_ctrl0 : FM_AUDIO_CTRL_ROUTE_OFF;

	return bt->fm_muted ? (v | FM_AUDIO_MANUAL_MUTE) : v;
}

/*
 * sub_DD334 followed by sub_DD458: write AUDIO_CTRL, then read 0x4D.
 *
 * The read is stock's and is kept even though nothing is done with the byte,
 * because it is the other half of a pair the part is always given. Its result
 * is logged and never made fatal.
 */
static int bcm_fm_write_audio_ctrl(struct bcm2078_bt *bt, bool route_on)
{
	u8 dummy;
	int ret;

	ret = bcm_fm_w16(bt, FM_REG_AUDIO_CTRL, bcm_fm_audio_ctrl(bt, route_on));
	if (ret)
		return ret;
	bt->fm_route_on = route_on;
	if (bcm_fm_read(bt, FM_REG_ROUTE_PCM, &dummy, 1))
		dev_dbg(bt->dev, "FC15 reg 0x4d readback failed\n");
	return 0;
}

/*
 * Stock's FM audio enable, sub_42A8C, in stock's order.
 *
 * The SoC's dividers are programmed first and the tuner is told to put audio
 * on the port second, so the port is already clocking when data starts to
 * arrive. Reversing it leaves the BCM2078 driving a bus whose frame sync is
 * still at the previous station's rate.
 */
static int bcm_fm_audio_route(struct bcm2078_bt *bt, bool on)
{
	int (*clk)(unsigned int) = READ_ONCE(bcm_fm_pcm_clk_hook);
	unsigned int bitclk = bcm_fm_bitclk_for(bt->fm_khz);
	int ret;

	if (on && clk) {
		ret = clk(bitclk);
		if (ret && ret != -ENODEV)
			dev_warn(bt->dev, "IIS2 bit clock %u Hz: %d\n",
				 bitclk, ret);
	}
	ret = bcm_fm_write_audio_ctrl(bt, on);
	dev_info(bt->dev, "FM audio route %s bitclk=%u ctrl=0x%04x%s\n",
		 on ? "on" : "off", on ? bitclk : 0,
		 bcm_fm_audio_ctrl(bt, on), ret ? " FAIL" : "");
	return ret;
}

static int bcm_fm_power_on(struct bcm2078_bt *bt)
{
	int ret;

	bcm_bd_addr_apply_once(bt);

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
	/*
	 * Power-on leaves the route OFF, which is stock's behaviour: sub_DD136
	 * ends in sub_DD334(0), the 0x0001 arm. Audio is switched on later, by
	 * bcm_fm_audio_route(), once a station is tuned and the SoC's dividers
	 * have been set for it -- there is nothing to route before then.
	 */
	ret = bcm_fm_write_audio_ctrl(bt, false);
	if (ret)
		return ret;

	/*
	 * Stereo blend and soft mute: nine data bytes, of which stock fills
	 * two. sub_DD2FC packs 0x2100F90B and 0x14000000, which is the RSSI
	 * threshold 33 at data[0] and the SNR threshold 20 at data[4]; the
	 * defaults come from preference slots 166 and 167 and are what this
	 * board ships with.
	 */
	{
		u8 p[11] = {
			FM_REG_BLEND_SMUTE, FM_MODE_WRITE,
			0x21, 0x00, 0x00, 0x00,
			0x14, 0x00, 0x00, 0x00, 0x00
		};

		ret = bcm_fc15(bt, p, sizeof(p));
	}
	if (ret)
		return ret;

	/*
	 * Bring the receive volume down to unity, because the chip does not
	 * power up there.
	 *
	 * VOLUME_CTRL is a 0..256 scale and it reads back 0x0169 -- 361, about
	 * three decibels above the top of its own range -- until something sets
	 * it, and nothing here ever did. The tuner therefore drove the PCM port
	 * at digital full scale: captures peaked at 32764 of 32768 with dozens
	 * of samples above 32000, and loud passages came out with broken samples
	 * in them, audible as clicks that followed the music rather than the
	 * clock. Measured on the headphone output: four such transients in
	 * thirty seconds at the default, none at 256, and writing 448 instead
	 * pins the capture at both rails, which is what fixes the scale.
	 *
	 * Unity rather than lower: it leaves three decibels of headroom, which
	 * is enough, and every stage after this one has a volume of its own.
	 */
	ret = bcm_fm_w16(bt, FM_REG_VOLUME_CTRL, FM_VOLUME_UNITY);
	bt->fm_on = !ret;
	bt->rds_on = !ret;
	memset(bt->rds_ps, 0, sizeof(bt->rds_ps));
	memset(bt->rds_rt, 0, sizeof(bt->rds_rt));
	memset(bt->rds_ps_build, ' ', sizeof(bt->rds_ps_build));
	memset(bt->rds_rt_build, ' ', sizeof(bt->rds_rt_build));
	memset(bt->rds_ptyn, 0, sizeof(bt->rds_ptyn));
	memset(bt->rds_ptyn_build, ' ', sizeof(bt->rds_ptyn_build));
	/* The RT+ group is announced per station, so forget the last one. */
	bt->rds_rtp_agt = FM_RDS_AGT_NONE;
	bt->rds_rtp_valid = false;
	bt->rds_groups = 0;
	dev_info(bt->dev,
		 "FM power ON audio_ctrl=0x%04x (route off until tune) wline=%u%s\n",
		 bcm_fm_audio_ctrl(bt, false), rds_wline,
		 ret ? " FAIL" : "");
	return ret;
}

/*
 * FM mute, for the ALSA mixer.
 *
 * MANUAL_MUTE is the tuner's own mute bit in AUDIO_CTRL, so this squelches the
 * audio at the source rather than after it has crossed to the SoC. That is the
 * right place for it: the IIS2 side has no gain stage of its own, and muting by
 * closing the capture PCM would also stop the clock and drop RDS with it.
 *
 * Exported because the machine driver owns the mixer and this driver owns the
 * radio. bcm2078-bt is built in and nano7-audio is a module, so a plain export
 * is enough -- no hook indirection like the PMIC rails need, because the
 * dependency runs the other way.
 *
 * There is no FM volume register anywhere in the recovered FM_RDS_Command map,
 * so level on the FM path is the codec's playback volume once the audio is
 * looped to the headphones. Do not invent one here.
 */
int bcm2078_fm_mute_get(void)
{
	struct bcm2078_bt *bt = bcm_bt_singleton;

	if (!bt)
		return -ENODEV;
	return bt->fm_muted ? 1 : 0;
}
EXPORT_SYMBOL_GPL(bcm2078_fm_mute_get);

int bcm2078_fm_mute_set(bool mute)
{
	struct bcm2078_bt *bt = bcm_bt_singleton;
	int ret;

	if (!bt)
		return -ENODEV;
	mutex_lock(&bt->lock);
	/*
	 * Recorded even when the tuner is off, so unmuting before powering on
	 * is not silently forgotten: every AUDIO_CTRL write folds this bit in.
	 * The route half of the value is untouched, which is why mute cannot
	 * knock the audio off the PCM port the way an ORed-together
	 * fm_audio_ctrl0 could.
	 */
	bt->fm_muted = mute;
	ret = bt->fm_on ?
	      bcm_fm_write_audio_ctrl(bt, bt->fm_route_on) : 0;
	mutex_unlock(&bt->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(bcm2078_fm_mute_set);

/*
 * Power off in stock's order, from sub_157784: drop the audio route first,
 * then stop the tuner. The SoC's clock gate follows when the capture PCM
 * closes, which is s5l8740-i2s's iis2_hw_stop().
 */
static int bcm_fm_power_off(struct bcm2078_bt *bt)
{
	int ret;

	if (bt->fm_route_on)
		bcm_fm_audio_route(bt, false);
	ret = bcm_fm_w8(bt, FM_REG_RDS_SYSTEM, 0x00);

	bt->fm_on = false;
	bt->rds_on = false;
	bt->fm_route_on = false;
	dev_info(bt->dev, "FM power OFF%s\n", ret ? " FAIL" : "");
	return ret;
}

/*
 * Poll RDS_FLAG bit 0 until the tuner says the tune or search has landed.
 *
 * Shared by tune and seek because both end in a SEARCH_TUNE write and both
 * have to be settled before the audio route is switched: routing to a station
 * the front end has not reached yet puts noise on the PCM port and, for a
 * seek, does it at the wrong bit clock as well.
 */
static void bcm_fm_wait_settle(struct bcm2078_bt *bt)
{
	unsigned int tries;

	bt->tune_settled = false;
	for (tries = 0; tries < FM_TUNE_SETTLE_TRIES; tries++) {
		u16 flag = 0;

		if (bcm_fm_r16(bt, FM_REG_RDS_FLAG, &flag))
			return;
		if (flag & FM_FLAG_TUNE_COMPLETE) {
			bt->tune_settled = true;
			return;
		}
		msleep(FM_TUNE_SETTLE_MS);
	}
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
	/*
	 * Stock's order, from sub_DD26C at 0x000DD26C: RDS_MASK, then FM_CTRL
	 * and FREQ together in sub_56DA94/sub_56DA98, then SEARCH_TUNE. The
	 * mask has to be armed before the tune starts or the completion flag
	 * this function polls for is not enabled when the tune completes.
	 */
	ret = bcm_fm_w16(bt, FM_REG_RDS_MASK, 0x1203);
	if (ret)
		return ret;
	/* Band select is inverted: the Japan band is the one with the bit. */
	ret = bcm_fm_w8(bt, FM_REG_FM_CTRL,
			FM_CTRL_STEREO_AUTO |
			(khz >= 87500 ? 0 : FM_CTRL_BAND_JAPAN));
	if (ret)
		return ret;
	/*
	 * sub_56DA94 computes freq - 64000 in kHz, which is the same u16 as
	 * khz + 1536: 65536 - 64000 is 1536. Written little-endian, low byte
	 * in the first data slot.
	 */
	enc = (u16)((khz + 1536) & 0xffff);
	ret = bcm_fm_w16(bt, FM_REG_FREQ, enc);
	if (ret)
		return ret;
	ret = bcm_fm_w8(bt, FM_REG_SEARCH_TUNE, FM_TUNE_MODE_PRESET);
	if (!ret)
		bt->fm_khz = khz;
	/*
	 * Wait for the tuner to actually land before returning.
	 *
	 * Writing SEARCH_TUNE only starts the retune; RSSI and SNR keep
	 * reporting the previous channel until it completes. Measured
	 * 2026-09-04: a band sweep that read RSSI immediately after each tune
	 * produced a usable RSSI curve but an SNR column that was noise --
	 * 0 to 4 everywhere, including on strong stations. Tuning 96.7 MHz and
	 * waiting three seconds gave rssi=223 snr=27 on the same channel. So
	 * the RSSI path settles fast enough to look right and SNR does not,
	 * which is the worst combination: it invites you to trust the reading.
	 *
	 * FM_REG_RDS_MASK was armed with 0x1203 just above, and bit 0 of
	 * RDS_FLAG is search/tune-complete -- the flag read 0x0261 on a
	 * freshly tuned station and 0x0000 with nothing pending, both with
	 * bit 0 among the armed bits. Poll it rather than sleeping a fixed
	 * time, and treat a timeout as advisory: the tune itself was accepted,
	 * so failing the call would be wrong. Callers that need a trustworthy
	 * SNR should check tune_settled.
	 */
	if (!ret)
		bcm_fm_wait_settle(bt);
	dev_info(bt->dev, "FM tune %u kHz enc=%04x settled=%d%s\n",
		 khz, enc, bt->tune_settled, ret ? " FAIL" : "");
	/*
	 * And now the audio, which stock does on every completed tune rather
	 * than once at power-on: sub_348B8 reads the tuned frequency back and
	 * calls sub_3B82C, which picks this station's PCM bit clock and turns
	 * the route on. Re-running it for a station that needs the same clock
	 * as the last one costs one register write and keeps the one code path.
	 */
	if (!ret)
		bcm_fm_audio_route(bt, true);
	return ret;
}

/*
 * Search, in the order of sub_DD374 at 0x000DD374.
 *
 * RDS_MASK first, then SEARCH_CTRL, SEARCH_CTRL1, PRESCAN_QUALITY,
 * SEARCH_METHOD and finally SEARCH_TUNE = 2. The mask write was missing here,
 * which left the completion flag unarmed for the whole search.
 */
static int bcm_fm_seek(struct bcm2078_bt *bt, int up, u8 rssi)
{
	u8 flags = 0x70 | (up ? 0x80 : 0);
	u8 enc[2];
	int ret;

	ret = bcm_fm_w16(bt, FM_REG_RDS_MASK, 0x1203);
	if (ret)
		return ret;
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
	if (ret) {
		dev_info(bt->dev, "FM seek %s rssi=%u FAIL\n",
			 up ? "up" : "down", rssi ? rssi : 33);
		return ret;
	}
	bcm_fm_wait_settle(bt);
	/*
	 * Where the search stopped is only known by asking. Stock does exactly
	 * this on search-complete: sub_DD08C issues 0x02010A03, a two-byte read
	 * of FREQ, and the value is the same freq - 64000 the write uses.
	 */
	if (!bcm_fm_read(bt, FM_REG_FREQ, enc, sizeof(enc)))
		bt->fm_khz = ((unsigned int)enc[0] |
			      ((unsigned int)enc[1] << 8)) + 64000u;
	dev_info(bt->dev, "FM seek %s rssi=%u landed on %u kHz settled=%d\n",
		 up ? "up" : "down", rssi ? rssi : 33, bt->fm_khz,
		 bt->tune_settled);
	bcm_fm_audio_route(bt, true);
	return 0;
}

/*
 * The power-on stock actually performs, in stock's order.
 *
 * sub_57045C(1) is Bluetooth power on and is exactly three pad writes:
 *
 *     sub_17D0(1)             -> sub_43D38C(0x50, 2, 0)   pad 80 function 2
 *     sub_43D38C(0x46, 1, 1)                              pad 70 output high
 *     sub_428F70(0x46, 1)                                 pad 70 +0x0C
 *
 * and sub_570054 only then opens the port and speaks at 115200. Power off
 * is the same three with the argument inverted, which is what says pad 70
 * high means on.
 *
 * Deliberately NOT the mode-2 trio 97/98/119 that bcm_power_pins_on()
 * drives. Those are claimed by sub_15DD5C at function 2 when FM powers up
 * -- they are the IIS2 PCM pads -- and driving them here once put the
 * device back into RetailOS. They are not part of sub_57045C.
 *
 * This has to run before hci_bcm opens the port, so it is called from
 * probe rather than from the FM tuning path, which was the only caller
 * bcm_power_on() ever had.
 */
/*
 * WARNING: this is almost certainly NOT the power-on sequence.
 *
 * It transliterates sub_57045C, and sub_57045C is not what this file has
 * been assuming. Its two callers name it, in stock's own log strings:
 *
 *	sub_58B8F4: "Allow BT Chip to enter low power mode."   -> sub_57045C(0)
 *	sub_58B930: "Prevent BT Chip from entering low power mode." -> sub_57045C(1)
 *
 * So sub_57045C is the low-power-mode inhibit -- the BT_WAKE line -- and
 * not a supply or reset at all. The body agrees: sub_17D0(1) is
 * sub_43D38C(0x50, 2, 0) and the rest is sub_43D38C(0x46, 1, 1) plus its
 * gate. Two pins, toggled, nothing else. Our transliteration of it is
 * faithful; what it was labelled is wrong.
 *
 * Which fits what the hardware says. Measured with this running: the UART
 * transmits cleanly (UTRSTAT 0x06, tx and shift register both empty, zero
 * errors), the rx FIFO stays empty, /proc/interrupts shows no interrupts on
 * 3db00000.serial, and hci_bcm times out. Not one byte has ever come back,
 * and asserting a wake pin would not change that.
 *
 * Which command times out says how far it got, so quote it precisely. The
 * "command 0xfc18 tx timeout" readings date from before the command order
 * was corrected on 2026-09-03; bcm_setup() returns early if
 * btbcm_initialize() fails, so 0xFC18 is unreachable while HCI_Reset is
 * failing. The current symptom is "command 0x0c03 tx timeout" followed by
 * "BCM: Reset failed (-110)" -- the first command on the link, unanswered.
 *
 * UMSTAT read 0x00 at the same time. That part is not evidence of anything.
 * Bit 4 of UMSTAT is DCTS -- delta-CTS, latched and read-clear -- not the
 * CTS level. Read on a RetailOS 1.1.2 unit with Bluetooth on and connected,
 * the same register returned 0x10 once in eight samples and 0x00 the other
 * seven, so a sample finding it clear does not distinguish a working link
 * from a dead one. Any argument anywhere in this driver, or in
 * RCA-BLUETOOTH-PMIC-ENABLE.md, that starts from "CTS has never asserted"
 * is void; the silence of the RX path is the observation that stands.
 *
 * The real power-on has not been found yet. The trail runs from
 * "BTLocalDeviceSetModulePower" through sub_429538(handle, 3, -1, ...),
 * which is a BT stack call several layers above any GPIO.
 *
 * Left in place because it is a correct transliteration of a real stock
 * function and removing it would lose that, and because it is harmless.
 * It is just not the thing that turns the chip on.
 */
static void bcm_bt_power_up(struct bcm2078_bt *bt)
{
	/*
	 * sphwBluetooth_Init, read out of the image rather than the
	 * decompiled export, which drops the delay arguments:
	 *
	 *   570064  bl   0x17d4dc      variant power pad
	 *   570068  bl   0x51681c      delays
	 *   570072  bl   0x43d38c      pad 70 high, again
	 *   570076  movs r0, #50
	 *   570078  blx  0x345d48      delay(50)
	 *   57007e  bl   0x5703ea      delay, 50, delay
	 *   570082  bl   0x570360      open the port
	 *
	 * The variant byte at 0x8925CAC is 0x05 in this image, so sub_17D4DC
	 * takes its variant-5 arm -- pad 70 high, then the pad's +0x0C bit --
	 * and sub_5703EA takes sub_5169A8. The variant 1 and 2 arm drives pad
	 * 200, which sub_43D38C and sub_57056C both discard on a pad == 200
	 * test, so no second pin is reachable on this part.
	 */
	/*
	 * Both UART pads, with stock's arguments on stock's pads.
	 *
	 * sub_177AE0 -- the UART open path, reached from sub_570360 as
	 * sub_177AE0(port=1, 115200) -- does exactly this before it touches
	 * the port:
	 *
	 *	177af4  movs r2,#1 ; movs r0,#80 ; mov r1,r2
	 *	177afa  bl 0x43d38c          ; sub_43D38C(80, 1, 1)
	 *	177afe  movs r2,#0 ; movs r1,#2 ; movs r0,#81
	 *	177b04  bl 0x43d38c          ; sub_43D38C(81, 2, 0)
	 *	177b08  movs r1,#1 ; movs r0,#81
	 *	177b0c  bl 0x428f70          ; sub_428F70(81, 1)
	 *
	 * We had a single call, bcm_43D38C(80, 2, 0) -- which is pad 81's
	 * arguments applied to pad 80, with pad 81 never configured at all.
	 * A UART with one of its two pads unmuxed is a good way to transmit
	 * into nothing and hear nothing back, which is the symptom.
	 *
	 * The Bluetooth RCA said pads 80/81 "appear only in sub_570000, the
	 * close path". They appear in the open path too; it only looked at
	 * the BT power code and this is in the UART code.
	 *
	 * (1, 1) on pad 80 is a GPIO output driven high, not the UART mux.
	 * It idles the TX line while the port registers are programmed, and
	 * sub_177AE0 hands the pad to the UART afterwards -- see the mode-2
	 * write at the end of this function.
	 */
	bcm_43D38C(bt, BCM_GPIO_UART_TX, 1, 1);
	bcm_43D38C(bt, BCM_GPIO_UART_RX, BCM_MODE_POWER, 0);
	bcm_428F70(bt, BCM_GPIO_UART_RX, 1);

	/* sub_17D4DC(5) */
	bcm_43D38C(bt, BCM_GPIO_PWR, 1, 1);
	bcm_428F70(bt, BCM_GPIO_PWR, 1);

	/*
	 * sub_51681C, and this is where the sequence used to give up.
	 *
	 * Its three sub_345D40 calls were read as unrecoverable thunks into
	 * SRAM the image does not carry, and replaced with a bare msleep(50).
	 * The image does carry it: the SRAM block is copied from 0x08982B00
	 * by a memcpy in the boot path, so the veneers disassemble, and what
	 * they do is write the PMIC's GPIO block over I2C. The three writes
	 * are 0x52 = 0xEA, 0x56 = 0x49 and 0x4B = 0x09, with sub_345D48(10)
	 * -- ten milliseconds -- between the second and the third.
	 *
	 * 0x4B is the BCM2078's enable, and we have never written it. That
	 * is why the controller has never answered: it was held in reset for
	 * every HCI_Reset we have ever sent it.
	 */
	bcm_bt_enable(bt, D1830_BT_STEP_PREP);

	/* the second pad 70 write, which stock does unconditionally */
	bcm_43D38C(bt, BCM_GPIO_PWR, 1, 1);

	/* delay(50) -- the argument the decompiled export drops */
	msleep(50);

	/*
	 * sub_5703EA(5) -> sub_5169A8: 0x4B = 0x09, sub_43E006(50, 0), then
	 * 0x4B = 0x0B. The part starts on that last write.
	 */
	bcm_bt_enable(bt, D1830_BT_STEP_RELEASE);

	/*
	 * Pad 80 to function 2, which is the write that actually connects
	 * UART1's transmitter to the pin.
	 *
	 * sub_177AE0 does it last, on the success path of the port open:
	 *
	 *	8177b18  cbz  r0, 0x8177b22    ; sub_188FE0 returned 0
	 *	8177b22  movs r2, #0
	 *	8177b24  movs r1, #2
	 *	8177b26  movs r0, #80
	 *	8177b28  bl   0x843d38c        ; sub_43D38C(80, 2, 0)
	 *
	 * Only the failure arm skips it, and that arm goes to sub_570000 --
	 * the close path -- instead. Pad 81 is muxed above and stays muxed;
	 * pad 80 is the one that changes mode, from GPIO-high to the UART.
	 *
	 * Without this the pad remains a GPIO parked high, so every byte the
	 * driver writes to UTXH is shifted out inside the peripheral and
	 * never reaches the controller. Stock sequences it after the port
	 * registers are programmed; here it has to be at the end of probe,
	 * because serdev opens the port later.
	 */
	bcm_43D38C(bt, BCM_GPIO_UART_TX, BCM_MODE_POWER, 0);

	dev_info(bt->dev,
		 "BT power-on: pad 80 fn2, pad 70 high + gate, PMIC 0x4B released\n");
}

/*
 * Reset the radio, for hci_bcm's link-recovery ladder.
 *
 * hci_bcm calls this when a desynchronised receiver has not come back from a
 * resync, and again at the top of every setup so that setup always meets a chip
 * in its power-on state. It must leave the part where setup expects it:
 * unpatched, at the initial baud.
 *
 * Called from the receive path, so it cannot sleep for long or take bt->lock --
 * bcm_power_off/on already sleep tens of milliseconds between PMIC writes, which
 * is why this hands off to a work item rather than doing it here.
 */
static int bcm_radio_reset(void)
{
	struct bcm2078_bt *bt = bcm_bt_singleton;

	if (!bt)
		return -ENODEV;

	/*
	 * Synchronous, and that is the whole point.
	 *
	 * This was a work item first, and it did not work: bcm_setup() called
	 * the hook, schedule_work() returned immediately, and setup then talked
	 * to a chip that was still mid-reset or had not started resetting. The
	 * first HCIDEVUP after a cold boot failed with `command tx timeout` and
	 * `BCM: Reset failed (-110)` while dmesg showed the reset had been
	 * requested three times -- asked for and not waited for.
	 *
	 * Both callers can sleep. bcm_setup() runs from hci_dev_open() in
	 * process context, and the recovery ladder runs from the serdev receive
	 * path, which is the tty flip-buffer work -- also process context.
	 * Blocking the receiver while resetting the part it feeds from is
	 * exactly right; there is nothing worth reading until it is back.
	 */
	mutex_lock(&bt->lock);
	dev_warn(bt->dev, "radio reset\n");
	if (bt->fm_on)
		bcm_fm_power_off(bt);
	bcm_power_off(bt);
	msleep(20);
	bcm_power_on(bt);
	mutex_unlock(&bt->lock);
	return 0;
}

static int bcm_power_on(struct bcm2078_bt *bt)
{
	bool was_off = !bt->powered;
	int ret;

	bt->powered = true;
	/*
	 * Rails first, then the pad gate, then the level: powering a pin
	 * before its supply is the wrong order and is what the de-init
	 * sequence unwinds.
	 */
	ret = bcm_bt_rails(true);
	if (ret && ret != -ENODEV)
		dev_warn(bt->dev, "bt rails on: %d\n", ret);
	bcm_power_pad_gate(bt, 1);

	/*
	 * Release the chip's reset, which is what actually starts it.
	 *
	 * This used to stop at the rails and the pad gate, and with gpio_poke=0
	 * -- the correct default -- it returned right here, so `power_on 1`
	 * turned the supply back on and left the part held in reset. Measured
	 * 2026-09-04: after `power_on 0; power_on 1`, hci0 still would not come
	 * up, because bcm_power_off() had asserted the reset and nothing
	 * un-asserted it.
	 *
	 * bcm_bt_power_up() is stock's sequence -- PREP, pad 70, 50 ms, RELEASE,
	 * then pad 80 to function 2 -- and every write in it is an absolute
	 * value rather than a toggle, so running it again is idempotent. Sharing
	 * it is also the point: a second copy of a sequence this fiddly would
	 * drift from the first.
	 *
	 * With this, `power_on 0; power_on 1; n31-hciup up` is a recovery path:
	 * the chip comes back unpatched at 115200, where a re-run setup expects
	 * it.
	 *
	 * Only when it was actually off, though, and that qualifier is the whole
	 * point. Most callers of this function are not asking for a power cycle
	 * -- opening /dev/radio0 and the fm_* sysfs writes call it to make sure
	 * the part is powered before they talk to it. Running the reset
	 * unconditionally turned every one of those into a reset of a working
	 * controller. Measured on 2026-09-04: the chip patched cleanly at boot
	 * (build 0000 -> 0122 at t=14.7s), then something opened the radio at
	 * t=19.0s, this ran, and 0xFC15 timed out from t=21s onward with
	 * `fe:7 brk:7` on UART1 -- the framing damage of a host at 2.4 Mbaud
	 * talking to a chip that had just been reset back to 115200 and
	 * unpatched. FM had been working before that change.
	 *
	 * hci_bcm cannot re-patch on its own here, because
	 * HCI_QUIRK_NON_PERSISTENT_SETUP is held back (see the comment on the
	 * patch script), so a reset outside a deliberate power cycle is
	 * unrecoverable without a reboot.
	 */
	if (was_off)
		bcm_bt_power_up(bt);

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

/*
 * sub_51688C: two delays, then sub_158C82 zeroing entries 3 and 5.
 * Unwound in the reverse order of power-on -- levels, then the pad
 * gate, then the rails -- so the part is not left driving a pin whose
 * supply has already gone. Doing the rails is also what makes this a
 * real off rather than an idle: without it the companion keeps drawing
 * even with the control pin low.
 */
static void bcm_power_off(struct bcm2078_bt *bt)
{
	int ret;

	bt->powered = false;
	if (gpio_poke)
		bcm_power_pins_off(bt);
	msleep(2);
	bcm_power_pad_gate(bt, 0);

	/*
	 * Assert the chip's reset before the rail goes, which is stock's own
	 * step -- sub_516700(0), PMIC 0x4B back to the held value.
	 *
	 * This path used to drop the rail and the pad gate and leave reset
	 * released, which has two costs.
	 *
	 * Power: a part held out of reset with its supply removed is not off in
	 * any useful sense, and when the rail is shared it is not off at all.
	 * If the user has Bluetooth disabled we should be spending nothing on
	 * it.
	 *
	 * And kexec, which is the sharper one. hci_bcm runs its setup exactly
	 * once, at probe: patchram, then the 0xFC18 baud change to 2.4 Mbaud.
	 * A kexec into a second kernel re-probes a chip that is still patched
	 * and still at 2.4 Mbaud, while the new host opens the port at 115200 --
	 * every frame after that is garbage. It is the same failure a plain
	 * hci0 down/up produces:
	 *
	 *	Bluetooth: hci0: Frame reassembly failed (-84)
	 *	Bluetooth: hci0: Opcode 0x0c03 failed: -110
	 *
	 * Resetting the part here means the next kernel finds it where
	 * hci_bcm expects to find it: unpatched, at 115200. Reset first, then
	 * remove power, so the part is quiescent before its supply goes rather
	 * than being asked to hold state through a rail transition.
	 */
	ret = bcm_bt_enable(bt, D1830_BT_STEP_HOLD);
	if (ret && ret != -ENODEV)
		dev_warn(bt->dev, "bt reset assert: %d\n", ret);

	ret = bcm_bt_rails(false);
	if (ret && ret != -ENODEV)
		dev_warn(bt->dev, "bt rails off: %d\n", ret);
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
/*
 * Drain the raw group ring. One group per line:
 *
 *	<type><ver> pi=XXXX a=XXXX b=XXXX c=XXXX d=XXXX
 *
 * A read consumes what it reports, so two readers race and that is fine -- this
 * is a diagnostic, and the alternative (a per-open cursor) would need a
 * character device. The trailing line reports drops so a reader can tell "I am
 * keeping up" from "I am not".
 */
static ssize_t fm_rds_groups_show(struct device *dev,
				  struct device_attribute *a, char *buf)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	unsigned int i, n, tail, drop;
	int len = 0;

	mutex_lock(&bt->lock);
	if (bt->fm_on)
		bcm_rds_poll(bt);
	n = bt->rds_ring_used;
	drop = bt->rds_ring_drop;
	tail = (bt->rds_ring_head + FM_RDS_RING - n) % FM_RDS_RING;
	for (i = 0; i < n; i++) {
		const struct bcm_rds_raw *g =
			&bt->rds_ring[(tail + i) % FM_RDS_RING];
		unsigned int type = g->b >> 12, ver = (g->b >> 11) & 1;

		/* Leave room for the summary line rather than truncating it. */
		if (len > PAGE_SIZE - 96)
			break;
		len += sysfs_emit_at(buf, len,
				     "%u%c pi=%04x a=%04x b=%04x c=%04x d=%04x\n",
				     type, ver ? 'B' : 'A',
				     g->a, g->a, g->b, g->c, g->d);
	}
	bt->rds_ring_used = 0;
	bt->rds_ring_drop = 0;
	len += sysfs_emit_at(buf, len, "groups=%u dropped=%u total=%u\n",
			     i, drop, bt->rds_groups);
	mutex_unlock(&bt->lock);
	return len;
}
static DEVICE_ATTR_RO(fm_rds_groups);

/*
 * Program the controller's Bluetooth address, HCI_Write_BD_Addr (0xFC01).
 *
 * Without this the part answers as whatever the patchram left behind -- this
 * unit reported 20:78:a0:0a:aa:aa, whose 0a:aa:aa tail is a Broadcom default,
 * not an assigned address. The real one is in SysCfg, which the FTL already
 * parses and prints as bt_mac; on this device 40:b3:95:b7:ae:48, and 40:B3:95
 * is an Apple OUI. 0xFC01 occurs exactly once in OSOS, so stock programs it
 * too.
 *
 * This does NOT need storage, and an earlier note here claiming it did was
 * wrong. The bootloader hands the identity over in RAM: Apple's loader leaves an
 * "IsyS" struct whose +0x158 is the address and whose +0x350 is the Grape
 * calibration, and U-Boot copies the whole 0x560-byte object into the reserved
 * region at n31-touch_cal@9dff000, advertised on /chosen as
 * apple,n31-touch_cal-addr / -size. Those property names are historical -- the
 * calibration was the first field with a consumer -- and the region is the whole
 * object, so no U-Boot change was needed to reach the address.
 *
 * That handoff is read in the patched hci_bcm, not here, and the reason is
 * ordering. HCI_QUIRK_NON_PERSISTENT_SETUP re-runs setup on every open and the
 * patchram reinstalls its placeholder each time, so a one-shot write from this
 * driver gets undone; and the write has to land before the core issues
 * HCI_Read_BD_Addr, or hdev->bdaddr keeps the placeholder and every bond -- which
 * is keyed on the address -- names an address the controller no longer has. See
 * patch-hci-bcm-bd-addr.py.
 *
 * What stays here is the manual override: a write to the bd_addr attribute, for
 * trying an address without a rebuild. It goes to a live hci0, so it happens on
 * an FM/HCI use rather than in probe -- see bcm_hci_cmd(), and see
 * bcm_power_off() for what cycling the device costs.
 */
static int bcm_write_bd_addr(struct bcm2078_bt *bt, const unsigned int m[6])
{
	u8 p[6];
	int i;

	for (i = 0; i < 6; i++) {
		if (m[i] > 0xff)
			return -EINVAL;
		/* The command takes the address little-endian. */
		p[i] = (u8)m[5 - i];
	}
	return bcm_hci_cmd(bt, HCI_OP_WRITE_BD_ADDR_VS, p, sizeof(p));
}

/*
 * The bootloader's copy, if it left one. Absent today -- the U-Boot side of the
 * SysCfg handoff described above is not written yet -- so this is the hook that
 * makes it work the moment it is, and its absence is not an error.
 */
static int bcm_bd_addr_from_chosen(struct bcm2078_bt *bt, unsigned int m[6])
{
	struct device_node *chosen;
	const char *s;
	int ret = -ENOENT;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen)
		return -ENOENT;
	if (!of_property_read_string(chosen, "apple,n31-bt_mac", &s) &&
	    sscanf(s, "%x:%x:%x:%x:%x:%x",
		   &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6)
		ret = 0;
	of_node_put(chosen);
	return ret;
}

static ssize_t bd_addr_store(struct device *dev, struct device_attribute *a,
			     const char *buf, size_t count)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	unsigned int m[6];
	int ret;

	if (sscanf(buf, "%x:%x:%x:%x:%x:%x",
		   &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6)
		return -EINVAL;
	mutex_lock(&bt->lock);
	ret = bcm_write_bd_addr(bt, m);
	mutex_unlock(&bt->lock);
	dev_info(bt->dev, "BD address %02x:%02x:%02x:%02x:%02x:%02x%s\n",
		 m[0], m[1], m[2], m[3], m[4], m[5], ret ? " FAIL" : "");
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(bd_addr);

/*
 * Render the RadioText+ tags as text.
 *
 * A tag is not text: it is a content type and a run of characters inside the
 * RadioText that is already being assembled, so what it means depends on the
 * RadioText current at the time it is read. Slicing here rather than copying
 * when the group arrives keeps the two in step -- a tag whose RadioText has
 * since been replaced slices the new one, which is what a station intends when
 * it changes both together.
 *
 * Only the classes worth naming are named; the rest print as their number,
 * because the table runs to sixty-four entries and a reader that cares about
 * the others can look them up.
 */
static const char *bcm_rtplus_class(u8 t)
{
	switch (t) {
	case 1:  return "title";
	case 2:  return "album";
	case 3:  return "track";
	case 4:  return "artist";
	case 5:  return "composition";
	case 7:  return "conductor";
	case 8:  return "composer";
	case 9:  return "band";
	case 11: return "genre";
	case 12: return "news";
	case 15: return "sport";
	case 28: return "stationname";
	case 30: return "programme";
	case 31: return "programme.now";
	default: return NULL;
	}
}

static void bcm_rds_rtplus_text(struct bcm2078_bt *bt, char *out, size_t max)
{
	size_t rtlen = strlen(bt->rds_rt);
	int n = 0;
	unsigned int i;

	out[0] = 0;
	if (!bt->rds_rtp_valid) {
		scnprintf(out, max,
			  bt->rds_rtp_agt == FM_RDS_AGT_NONE ?
			  "(no 3A announcement yet)" : "(announced, no tag yet)");
		return;
	}
	if (!bt->rds_rtp_running)
		n += scnprintf(out + n, max - n, "[stopped] ");
	for (i = 0; i < 2; i++) {
		const char *name = bcm_rtplus_class(bt->rds_rtp_type[i]);
		unsigned int start = bt->rds_rtp_start[i];
		unsigned int len = bt->rds_rtp_len[i];

		/* Class 0 is the dummy, used when only one tag is carried. */
		if (!bt->rds_rtp_type[i])
			continue;
		if (start >= rtlen)
			continue;
		if (start + len > rtlen)
			len = rtlen - start;
		if (name)
			n += scnprintf(out + n, max - n, "%s=", name);
		else
			n += scnprintf(out + n, max - n, "class%u=",
				       bt->rds_rtp_type[i]);
		n += scnprintf(out + n, max - n, "%.*s ", (int)len,
			       bt->rds_rt + start);
	}
	if (n > 0 && out[n - 1] == ' ')
		out[n - 1] = 0;
}

static ssize_t fm_rds_show(struct device *dev, struct device_attribute *a,
			   char *buf)
{
	struct bcm2078_bt *bt = dev_get_drvdata(dev);
	char rtp[160];
	u16 flag;
	int ret;

	mutex_lock(&bt->lock);
	ret = bt->fm_on ? bcm_rds_poll(bt) : -ENODEV;
	/*
	 * RDS_FLAG is read-to-clear, and bcm_rds_poll() has already read it to
	 * decide whether to drain the FIFO. Reading it again here reported
	 * 0x0000 for the poll that had just consumed a group, which read as
	 * "nothing arrived" on the one call where something had. Report what
	 * the poll saw.
	 */
	flag = bt->fm_on ? bt->rds_flag : 0;
	bcm_rds_rtplus_text(bt, rtp, sizeof(rtp));
	ret = sysfs_emit(buf,
			 "on=%d poll=%d groups=%u flag=0x%04x\n"
			 "pi=0x%04x pty=%u ptyn=%s\n"
			 "ps=%s\n"
			 "rt=%s\n"
			 "rt+=%s\n",
			 bt->rds_on, ret, bt->rds_groups, flag,
			 bt->rds_pi, bt->rds_pty, bt->rds_ptyn,
			 bt->rds_ps, bt->rds_rt, rtp);
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
	&dev_attr_fm_rds_groups.attr,
	&dev_attr_bd_addr.attr,
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
	struct regulator *bt_vreg;
	int ret;

	/*
	 * Take the rail first, and let the kernel handle the ordering.
	 *
	 * This driver is built in and probes at about t=2.4s; gpio-d1830,
	 * which owns the PMIC, is a module userspace loads at about t=7.1s.
	 * Everything in between used to fail with -ENODEV and there was no
	 * way to wait, so the controller was never powered and hci_bcm timed
	 * out talking to a dead part. Deferring is the whole point: the
	 * kernel re-probes this driver once the provider registers.
	 *
	 * Optional on purpose. A device tree without bt-supply keeps the old
	 * hook path rather than refusing to probe at all.
	 */
	bt_vreg = devm_regulator_get_optional(dev, "bt");
	if (IS_ERR(bt_vreg)) {
		ret = PTR_ERR(bt_vreg);
		bt_vreg = NULL;
		if (ret == -EPROBE_DEFER) {
			dev_info(dev, "waiting for the bt rail provider\n");
			return ret;
		}
		dev_info(dev, "no bt-supply (%d); using the legacy rails hook\n",
			 ret);
	} else {
		bcm_bt_vreg = bt_vreg;
		dev_info(dev, "bt rail acquired as a regulator\n");
	}

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;
	bt->dev = dev;
	bt->fm_khz = BCM_FM_KHZ_DEFAULT;
	mutex_init(&bt->lock);
	mutex_init(&bt->cmd_lock);
	platform_set_drvdata(pdev, bt);
	bcm_bt_singleton = bt;

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
	 * Power the controller up here, before hci_bcm opens the port.
	 *
	 * Nothing used to do this at probe: bcm_power_on() hangs off the V4L2
	 * FM tuning path and is never reached on a plain boot, so hci_bcm bound
	 * to a part that had never been powered and every command timed out.
	 *
	 * The older note here said not to touch GPIOs at probe because early
	 * mode-2 correlated with a reset back to RetailOS. That observation
	 * stands and is why bcm_bt_power_up() deliberately leaves 97/98/119
	 * alone -- those are the IIS2 PCM pads, claimed by sub_15DD5C when FM
	 * powers on, and driving them is what reproduces the reset. They are
	 * not part of sub_57045C. What is left is pad 80 to function 2 and
	 * pad 70 high, which is the whole of stock Bluetooth power-on.
	 */
	/*
	 * Hand hci_bcm the reset it needs for link recovery, and for the top of
	 * every setup run. Registered before the power-up so a setup triggered
	 * by the serdev probe can already use it.
	 */
	n31_bcm_register_radio_reset(bcm_radio_reset);

	bcm_bt_power_up(bt);
	/*
	 * Record that the part is up, so the first bcm_power_on() -- which comes
	 * from opening /dev/radio0 or writing an fm_* attribute, not from anyone
	 * asking for a power cycle -- does not read a zeroed `powered` as "it was
	 * off" and reset a controller hci_bcm has just finished patching.
	 */
	bt->powered = true;
	dev_info(dev,
		 "BCM2078 companion (GPIO+FM+V4L2) — UART1 owned by hci_bcm\n");
	return 0;
}

/*
 * sub_51688C is the de-init OSOS runs when Bluetooth goes away, and
 * bcm_power_off now implements it. Reaching it only from remove() meant
 * the companion kept its rails across a reboot, so the part came up in
 * whatever state the previous kernel left it rather than from reset.
 */
/*
 * Reboot, poweroff and kexec all land here, and kexec is the one that makes it
 * load-bearing rather than tidy: see bcm_power_off() for why a chip left
 * patched and at 2.4 Mbaud breaks the next kernel's probe.
 *
 * The tuner goes first. FM keeps the radio's audio path routed to the PCM port,
 * and there is no reason to carry that into a reset -- or into whatever boots
 * next, which may not know the route exists.
 */
static void bcm2078_shutdown(struct platform_device *pdev)
{
	struct bcm2078_bt *bt = platform_get_drvdata(pdev);

	if (!bt)
		return;
	mutex_lock(&bt->lock);
	if (bt->fm_on)
		bcm_fm_power_off(bt);
	bcm_power_off(bt);
	mutex_unlock(&bt->lock);
}

static void bcm2078_remove(struct platform_device *pdev)
{
	struct bcm2078_bt *bt = platform_get_drvdata(pdev);

	bcm_bt_singleton = NULL;

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

	/*
	 * bcm_bt_vreg is a file-scope pointer to a devm_regulator_get_optional()
	 * handle, so devm frees it the moment this returns. Leaving the pointer
	 * set means the next bcm_bt_rails() -- from a rebind, or from the FM
	 * side, which shares these statics -- dereferences freed memory. Drop
	 * it here, along with the enable state it tracks.
	 */
	bcm_bt_vreg = NULL;
	bcm_bt_vreg_on = false;
}

static const struct of_device_id bcm2078_of_match[] = {
	{ .compatible = "apple,n31-bcm2078-companion" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm2078_of_match);

static struct platform_driver bcm2078_driver = {
	.probe = bcm2078_probe,
	.remove = bcm2078_remove,
	.shutdown = bcm2078_shutdown,
	.driver = {
		.name = "bcm2078-bt",
		.of_match_table = bcm2078_of_match,
	},
};

module_platform_driver(bcm2078_driver);

MODULE_DESCRIPTION("N31 BCM2078 GPIO companion + V4L2 FM (0xFC15 via hci0)");
MODULE_LICENSE("GPL");
