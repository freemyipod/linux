// SPDX-License-Identifier: GPL-2.0-only
/*
 * CS42L81 / Apple 338S1146 — SPI0 control path (N31 RetailOS-matched)
 *
 * Framing (sub_43CDB4 / sub_43CDFA):
 *   write: 0x6C, reg_hi, reg_lo, 0, data
 *   read:  0x6D, reg_hi, reg_lo, 0, 0xFF  (rx data on last byte)
 *
 * Bring-up (confirmed corpus):
 *   read 0x227; write 0xC96F=0x0E|0x1E; 0x9901 unlock A5 then 0;
 *   0xC81F=0xFF; 0xC85F=0x0F
 *
 * RetailOS user volume is an integer 0..256 (debug HUD, NVRAM key 11/179
 * clamped in 37138, Vol+/- in 1BB754/1BB874). That value is a CoreAudio
 * VolumeScalar (256 = unity), not a CS42 mixer byte.
 *
 * 0x403/0x404 are HP mixer tap indices from 174E7C / 440AA4(udiv, 160):
 *   play 5706F4(1): L=(0+159)/160+2 = 2, R=+1 = 1
 * Analog 0x527 is mute 0xFF / unmute 0x60 (F141C). Do not unlock 9901
 * after mixer.
 *
 * ASoC DAI cs42l81-hifi: analog on hw_params. IIS serializer is the CPU DAI.
 *
 * CS42L42/L83 (I2C, paged 8-bit regmap, snd_soc_cs42l42) are a newer Cirrus
 * line with nearly identical maps — chip ID + MCLK_CTL defaults differ. N31
 * CS42L81 / 338S1146 is SPI-framed 16-bit addresses (see N31-102-CS42-REG-CORPUS);
 * do not drop in cs42l42.c wholesale.
 *
 * Cirrus bring-up notes:
 *   Reset mutes outputs (0x527=0xFF); unmute 0x60 on play.
 *   LOS: BCLK/LRCK stop clears 0x2F bit6 — asp_lock after IIS kick.
 *   ASP lock after IIS clocks (414FAE), not before.
 *   I2S slave NB_NF 16-bit; no DAPM graph — path is register audio_on().
 */
#include <linux/delay.h>
#include <linux/kallsyms.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define CS42L81_USER_VOL_MAX	256
#define CS42L81_MIX_TAP_L	2	/* 174E7C play */
#define CS42L81_MIX_TAP_R	1

struct cs42l81 {
	struct spi_device *spi;
	struct mutex lock;
	unsigned int user_vol;
	bool dai_mute;
};

static struct cs42l81 *cs42l81_dev;

int cs42l81_post_iis_start(void);
int cs42l81_play_prepare(void);

static int cs42l81_write(struct cs42l81 *c, u16 reg, u8 val);
static int cs42l81_set_mute(struct cs42l81 *c, int mute);
static int cs42l81_apply_user_vol(struct cs42l81 *c);

static int cs42l81_write(struct cs42l81 *c, u16 reg, u8 val)
{
	u8 tx[5] = {
		0x6c,
		(reg >> 8) & 0xff,
		reg & 0xff,
		0x00,
		val,
	};
	struct spi_transfer t = { .tx_buf = tx, .len = 5 };
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(c->spi, &m);
}

static int cs42l81_read(struct cs42l81 *c, u16 reg, u8 *val)
{
	u8 tx[5] = {
		0x6d,
		(reg >> 8) & 0xff,
		reg & 0xff,
		0x00,
		0xff,
	};
	u8 rx[5] = { 0 };
	struct spi_transfer t = { .tx_buf = tx, .rx_buf = rx, .len = 5 };
	struct spi_message m;
	int ret;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(c->spi, &m);
	if (ret)
		return ret;
	*val = rx[4];
	return 0;
}

static int cs42l81_rmw(struct cs42l81 *c, u16 reg, u8 mask, u8 val)
{
	u8 cur = 0;
	int ret = cs42l81_read(c, reg, &cur);

	if (ret)
		return ret;
	return cs42l81_write(c, reg, (cur & ~mask) | (val & mask));
}

static int cs42l81_bringup(struct cs42l81 *c)
{
	u8 st = 0;
	int ret;

	ret = cs42l81_read(c, 0x0227, &st);
	dev_info(&c->spi->dev, "CS42 status 0x227 = 0x%02x (ret=%d)\n", st, ret);

	/* rail / backpower-ish — prefer safe idle 0x0E over 0x1E */
	ret = cs42l81_write(c, 0xc96f, 0x0e);
	if (ret)
		return ret;

	/* unlock-like */
	cs42l81_write(c, 0x9901, 0xa5);
	cs42l81_write(c, 0x9901, 0x00);

	cs42l81_write(c, 0xc81f, 0xff);
	cs42l81_write(c, 0xc85f, 0x0f);

	cs42l81_read(c, 0x0219, &st);
	dev_info(&c->spi->dev, "CS42 companion 0x219 = 0x%02x\n", st);
	return 0;
}

/* sub_5707D8 ASP/mixer blast. Bytes from Hex-Rays, not invented. */
static const u8 cs42l81_mix400[] = {
	0x04, 0x10, 0x00, 0x09, 0x08, 0x00, 0x00, 0x00,
	0x01, 0xe0, 0x01, 0x01, 0xe0, 0xfe, 0x00, 0xa0,
	0x02, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00,
	0x00, 0x05, 0x00, 0x00, 0x06, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x08, 0x00, 0x00, 0x09, 0x00, 0x00,
	0x0a, 0x01, 0xe0, 0x0b, 0x01, 0xe0, 0xff, 0x00,
	0xa0, 0x0c, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x0e,
	0x00, 0x00, 0x0f, 0x00, 0x00, 0x10, 0x00, 0x00,
	0x11, 0x00, 0x00, 0x12, 0x00, 0x00, 0x13, 0x00,
	0x00,
};

/* D3280(4) + 400330 2v5 + 183138(48k) + D3280(3) HP + 5707D8. */
static int cs42l81_audio_on(struct cs42l81 *c)
{
	u8 st = 0, r219 = 0;
	unsigned int i;
	int ret;

	/* sub_D3280(a1==4) */
	cs42l81_rmw(c, 0x0007, 0x40, 0x00);
	cs42l81_rmw(c, 0x0219, 0x78, 0x78);
	cs42l81_write(c, 0x0229, 0x40);
	cs42l81_rmw(c, 0x0006, 0x01, 0x00);
	cs42l81_rmw(c, 0x0201, 0xe0, 0x40);
	cs42l81_write(c, 0xc81f, 0xff);
	cs42l81_write(c, 0xc85f, 0x0f);
	ret = cs42l81_write(c, 0xc96f, 0x0e);
	if (ret)
		return ret;
	cs42l81_write(c, 0x0223, 0x08);
	cs42l81_write(c, 0x0224, 0x09);
	cs42l81_write(c, 0x0225, 0x00);

	/* sub_400330: 2.5V backpower — RetailOS WRITES 0x219 */
	cs42l81_rmw(c, 0x0219, 0x07, 0x01);
	msleep(100);
	cs42l81_write(c, 0xc96f, 0x1e);
	/* Glass: 5-byte write left 0x2F=0x00. write6 left 0x2F=0x80 (off). */
	cs42l81_write(c, 0x0227, 0x40);

	/* sub_183138 48 kHz (v10=12) */
	cs42l81_rmw(c, 0x000e, 0xc0, 0xc0);
	cs42l81_rmw(c, 0x000f, 0x0f, 0x0c);
	cs42l81_write(c, 0x012f, 0xcc);
	cs42l81_write(c, 0x010b, 0x08);
	cs42l81_write(c, 0x010c, 0x09);
	cs42l81_rmw(c, 0x0131, 0x01, 0x01);
	cs42l81_rmw(c, 0x000e, 0xc0, 0x40);
	cs42l81_rmw(c, 0x0220, 0x20, 0x20);

	/* sub_5707D8 */
	cs42l81_write(c, 0x0006, 0x24);
	cs42l81_write(c, 0x0529, 0x2c);
	cs42l81_write(c, 0x052a, 0x2c);
	cs42l81_write(c, 0x0533, 0x2c);
	cs42l81_write(c, 0x0534, 0x2c);
	for (i = 0; i < ARRAY_SIZE(cs42l81_mix400); i++)
		cs42l81_write(c, 0x0400 + i, cs42l81_mix400[i]);
	cs42l81_write(c, 0x0400, 0x04);
	cs42l81_write(c, 0x0401, 0x12);
	/*
	 * 570620(1) → 174E7C: 0x403/0x404 are mixer tap indices, not
	 * the 0–256 user volume and not a saturate-at-0xA0 gain.
	 * 440AA4 is unsigned divide by 160.
	 */
	cs42l81_write(c, 0x0402, 0x00);
	cs42l81_write(c, 0x0403, CS42L81_MIX_TAP_L);
	cs42l81_write(c, 0x0404, CS42L81_MIX_TAP_R);
	cs42l81_write(c, 0x0405, 0x00);
	cs42l81_write(c, 0x0406, 0x00);
	msleep(100);
	cs42l81_write(c, 0x0500, 0x05);
	/* sub_F141C(1): unmute/level. Mute path writes 0xFF. */
	cs42l81_write(c, 0x0527, 0x60);
	/* 26DDDE → 416440 → 40C028(2): 42A5D6(117, 63, 60). Not 0x75 bit7. */
	cs42l81_rmw(c, 0x0075, 0x3f, 0x3c);
	/* 570620: 42A5D6(1359, 240, 0) */
	cs42l81_rmw(c, 0x054f, 0xf0, 0x00);
	cs42l81_rmw(c, 0x0220, 0x28, 0x28);

	/*
	 * D3280(3) after mixer: 0x0F bit7 (pad drive) and 0x220.
	 * D2EFC/9901 already ran in bringup — do not unlock again
	 * (that wiped 0x527/mixer on glass). 41CBD8(9,1) is IIS ungate.
	 */
	cs42l81_rmw(c, 0x0007, 0x40, 0x00);
	cs42l81_rmw(c, 0x0006, 0x40, 0x00);
	cs42l81_rmw(c, 0x0220, 0x28, 0x28);
	cs42l81_rmw(c, 0x000f, 0x80, 0x80);
	cs42l81_rmw(c, 0x0075, 0x40, 0x40);
	{
		u8 r74 = 0, r7b = 0, r7c = 0, r0f = 0, r2f = 0;

		cs42l81_read(c, 0x0074, &r74);
		cs42l81_write(c, 0x0074, (r74 & 0xe7) | 0x08);
		cs42l81_read(c, 0x007b, &r7b);
		cs42l81_read(c, 0x007c, &r7c);
		cs42l81_write(c, 0x0074, r74);
		cs42l81_rmw(c, 0x0075, 0x40, 0x00);
		cs42l81_rmw(c, 0x0075, 0x80, 0x80);
		cs42l81_read(c, 0x000f, &r0f);
		cs42l81_read(c, 0x002f, &r2f);
		dev_info(&c->spi->dev,
			 "D3280(3) 0x0F=0x%02x 0x2F=0x%02x 0x7B=0x%02x 0x7C=0x%02x\n",
			 r0f, r2f, r7b, r7c);
	}

	/*
	 * D34C0 with 892A038=0x28 (D3280(3)): short serial, not 183138.
	 * 0x0E bits7-6 = 11 then 01, 0x0F low=12, 0x12F=0xCC.
	 * Short path does not touch 0x131 — 183138 already set bit0.
	 */
	cs42l81_rmw(c, 0x000e, 0xc0, 0xc0);
	cs42l81_rmw(c, 0x000f, 0x0f, 0x0c);
	cs42l81_write(c, 0x012f, 0xcc);
	cs42l81_rmw(c, 0x000e, 0xc0, 0x40);

	/*
	 * 4F08: enable tip/ring sense. 7984: Class-H charge-pump kick
	 * plus 0x0B headset-type (CS42L73-class HP stays Hi-Z until this).
	 * 40C028(2) already programmed 0x75=0x3C; D3280(3) set bit7 (HP).
	 */
	cs42l81_rmw(c, 0x0073, 0xc3, 0x00);
	cs42l81_rmw(c, 0x0073, 0xc0, 0xc0);
	cs42l81_rmw(c, 0x0079, 0x60, 0x00);
	{
		u8 r220 = 0, r2f = 0, r0b = 0, r08 = 0, r09 = 0;
		unsigned int i;

		cs42l81_read(c, 0x0220, &r220);
		cs42l81_rmw(c, 0x0220, 0x40, 0x40);
		msleep(1);
		cs42l81_rmw(c, 0x0009, 0xc0, 0xc0);
		for (i = 0; i < 3; i++) {
			msleep(1);
			cs42l81_read(c, 0x002f, &r2f);
			if (r2f & 0x40)
				break;
		}
		cs42l81_read(c, 0x000b, &r0b);
		cs42l81_rmw(c, 0x0009, 0xc0, 0x80);
		cs42l81_rmw(c, 0x0220, 0x40, r220 & 0x40);
		cs42l81_read(c, 0x0008, &r08);
		cs42l81_read(c, 0x0009, &r09);
		dev_info(&c->spi->dev,
			 "HSDET 0x0B=0x%02x type=%u 0x2F=0x%02x 0x08=0x%02x 0x09=0x%02x\n",
			 r0b, r0b & 3, r2f, r08, r09);
	}
	/* 42D364(1) play: unmute HP amp + mixer bit1. */
	cs42l81_write(c, 0x0527, 0x60);
	cs42l81_rmw(c, 0x0401, 0x03, 0x02);

	cs42l81_read(c, 0x0227, &st);
	cs42l81_read(c, 0x0219, &r219);
	cs42l81_apply_user_vol(c);
	dev_info(&c->spi->dev,
		 "CS42 audio_on C96F=0x1E status 0x227=0x%02x 0x219=0x%02x vol=%u/%u\n",
		 st, r219, c->user_vol, CS42L81_USER_VOL_MAX);
	return 0;
}

/* 42D364(0/1) + F141C: play unmute 0x527=0x60, mute 0xFF. */
static int cs42l81_set_mute(struct cs42l81 *c, int mute)
{
	if (mute) {
		cs42l81_write(c, 0x0527, 0xff);
		cs42l81_rmw(c, 0x0401, 0x03, 0x01);
		/* F1444: pulse meter/soft-ramp bits. */
		cs42l81_rmw(c, 0x051e, 0x20, 0x20);
		cs42l81_rmw(c, 0x051e, 0x20, 0x00);
		cs42l81_rmw(c, 0x0523, 0x20, 0x20);
		cs42l81_rmw(c, 0x0523, 0x20, 0x00);
	} else {
		cs42l81_write(c, 0x0527, 0x60);
		cs42l81_rmw(c, 0x0401, 0x03, 0x02);
	}
	return 0;
}

static void cs42l81_push_pcm_q8(unsigned int vol)
{
	void (*set)(unsigned int);

	set = (void (*)(unsigned int))__symbol_get("s5l8740_set_user_vol_q8");
	if (set) {
		set(vol);
		__symbol_put("s5l8740_set_user_vol_q8");
	}
}

/* RetailOS 0 = analog mute; 1..256 = unmute + Q8 PCM scalar (256 = unity). */
static int cs42l81_apply_user_vol(struct cs42l81 *c)
{
	unsigned int q8 = c->dai_mute ? 0 : c->user_vol;

	cs42l81_push_pcm_q8(q8);
	return cs42l81_set_mute(c, q8 == 0);
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	unsigned int reg, val;
	int n;

	n = sscanf(buf, "%x %x", &reg, &val);
	if (n != 2 || reg > 0xffff || val > 0xff)
		return -EINVAL;
	mutex_lock(&c->lock);
	n = cs42l81_write(c, (u16)reg, (u8)val);
	mutex_unlock(&c->lock);
	return n ? n : count;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sysfs_emit(buf, "write: echo \"RRRR VV\" > reg  (hex)\n");
}
static DEVICE_ATTR_RW(reg);

static ssize_t status_227_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	u8 st = 0;
	int ret;

	mutex_lock(&c->lock);
	ret = cs42l81_read(c, 0x0227, &st);
	mutex_unlock(&c->lock);
	if (ret)
		return ret;
	return sysfs_emit(buf, "0x%02x\n", st);
}
static DEVICE_ATTR_RO(status_227);

static ssize_t bringup_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	int ret;

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;

	mutex_lock(&c->lock);
	ret = cs42l81_bringup(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(bringup);

static ssize_t dump_key_regs_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	static const u16 regs[] = {
		0x0227, 0x0219, 0xc96f, 0xc81f, 0xc85f,
		0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b,
		0x000c, 0x000d, 0x000e, 0x000f, 0x0012, 0x0019,
		0x0070, 0x0071, 0x0073, 0x0074, 0x0075, 0x0076,
		0x0079, 0x007a, 0x007b, 0x007c,
		0x002f, 0x0131, 0x0220, 0x0201, 0x0222,
		0x0223, 0x0224, 0x0121, 0x0122, 0x012f,
		0x010b, 0x010c, 0x0529, 0x052a, 0x0533, 0x0534,
		0x0400, 0x0401, 0x0402, 0x0403, 0x0404,
		0x0527, 0x051e, 0x0523, 0x054f,
		0x0500, 0x051f, 0x0520, 0x0521, 0x0524, 0x0525, 0x0528,
		0x001a, 0x001b, 0x001e, 0x001f,
		0x0034, 0x0035, 0x0039, 0x003a,
	};
	int i, n = 0, ret;
	u8 val;

	mutex_lock(&c->lock);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = cs42l81_read(c, regs[i], &val);
		if (ret) {
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "0x%04x: ERR %d\n", regs[i], ret);
		} else {
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "0x%04x: 0x%02x\n", regs[i], val);
		}
	}
	mutex_unlock(&c->lock);
	return n;
}
static DEVICE_ATTR_RO(dump_key_regs);

/* RetailOS user volume 0..256. 0x403/0x404 stay mixer taps. */
static ssize_t volume_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	unsigned int vol;
	int ret;

	if (kstrtouint(buf, 0, &vol))
		return -EINVAL;
	if (vol > CS42L81_USER_VOL_MAX)
		return -EINVAL;

	mutex_lock(&c->lock);
	c->user_vol = vol;
	ret = cs42l81_apply_user_vol(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}

static ssize_t volume_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	u8 tap_l = 0, tap_r = 0, analog = 0;
	int ra, rb, rc;

	mutex_lock(&c->lock);
	ra = cs42l81_read(c, 0x0403, &tap_l);
	rb = cs42l81_read(c, 0x0404, &tap_r);
	rc = cs42l81_read(c, 0x0527, &analog);
	mutex_unlock(&c->lock);
	if (ra || rb || rc)
		return sysfs_emit(buf, "read err %d/%d/%d\n", ra, rb, rc);
	return sysfs_emit(buf,
			  "user=%u/%u dai_mute=%d\n"
			  "0x403=0x%02x 0x404=0x%02x (taps, play 2/1)\n"
			  "0x527=0x%02x analog_mute=%d\n",
			  c->user_vol, CS42L81_USER_VOL_MAX, c->dai_mute,
			  tap_l, tap_r, analog, analog == 0xff);
}
static DEVICE_ATTR_RW(volume);

static ssize_t audio_on_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	int ret;

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	mutex_lock(&c->lock);
	ret = cs42l81_audio_on(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(audio_on);

static ssize_t mute_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	unsigned int v;
	int ret;

	if (kstrtouint(buf, 0, &v))
		return -EINVAL;
	mutex_lock(&c->lock);
	c->dai_mute = v ? 1 : 0;
	ret = cs42l81_apply_user_vol(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}

static ssize_t mute_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	u8 v = 0;
	int ret;

	mutex_lock(&c->lock);
	ret = cs42l81_read(c, 0x0527, &v);
	mutex_unlock(&c->lock);
	if (ret)
		return ret;
	return sysfs_emit(buf, "0x527=0x%02x mute=%d\n", v, v == 0xff);
}
static DEVICE_ATTR_RW(mute);

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
	ret = cs42l81_read(c, (u16)reg, &val);
	mutex_unlock(&c->lock);
	if (ret)
		return ret;
	dev_info(&c->spi->dev, "rreg 0x%04x = 0x%02x\n", reg, val);
	return count;
}
static DEVICE_ATTR_WO(rreg);

/*
 * After IIS BCLK/LRCK run (RetailOS 26DDDE: 414FAE before sustained PCM).
 * Re-run 183138 clock regs and poll 0x2F bit6 (ASP sync / LOS clear).
 * LOS does not always self-recover — pulse 0x220 and retry clock prog.
 */
static void cs42l81_asp_clock_pulse(struct cs42l81 *c)
{
	cs42l81_rmw(c, 0x0220, 0x20, 0x00);
	udelay(50);
	cs42l81_rmw(c, 0x0220, 0x20, 0x20);
}

static void cs42l81_asp_program_48k(struct cs42l81 *c)
{
	cs42l81_rmw(c, 0x000e, 0xc0, 0xc0);
	cs42l81_rmw(c, 0x000f, 0x0f, 0x0c);
	cs42l81_write(c, 0x012f, 0xcc);
	cs42l81_rmw(c, 0x000e, 0xc0, 0x40);
}

static int cs42l81_asp_lock(struct cs42l81 *c)
{
	unsigned int attempt, i;
	u8 r2f = 0, r0e = 0, r0f = 0;

	for (attempt = 0; attempt < 3; attempt++) {
		if (attempt)
			cs42l81_asp_clock_pulse(c);
		cs42l81_asp_program_48k(c);
		for (i = 0; i < 50; i++) {
			cs42l81_read(c, 0x002f, &r2f);
			if (r2f & 0x40)
				break;
			usleep_range(1000, 2000);
		}
		if (r2f & 0x40)
			break;
	}
	cs42l81_read(c, 0x000e, &r0e);
	cs42l81_read(c, 0x000f, &r0f);
	dev_info(&c->spi->dev,
		 "asp_lock 0x2F=0x%02x 0x0E=0x%02x 0x0F=0x%02x (need bit6, IIS running)\n",
		 r2f, r0e, r0f);
	return (r2f & 0x40) ? 0 : -EAGAIN;
}

int cs42l81_play_prepare(void)
{
	struct cs42l81 *c = cs42l81_dev;
	int ret;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	ret = cs42l81_audio_on(c);
	if (!ret && !c->dai_mute)
		cs42l81_set_mute(c, 0);
	mutex_unlock(&c->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cs42l81_play_prepare);

/*
 * Called after IIS TXCOM kick. Clears LOS mute and ensures HP path unmuted.
 */
int cs42l81_post_iis_start(void)
{
	struct cs42l81 *c = cs42l81_dev;
	int ret;

	if (!c)
		return -ENODEV;
	mutex_lock(&c->lock);
	ret = cs42l81_asp_lock(c);
	if (!ret && !c->dai_mute)
		cs42l81_set_mute(c, 0);
	mutex_unlock(&c->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cs42l81_post_iis_start);

static ssize_t asp_lock_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct cs42l81 *c = spi_get_drvdata(to_spi_device(dev));
	int ret;

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	mutex_lock(&c->lock);
	ret = cs42l81_asp_lock(c);
	mutex_unlock(&c->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(asp_lock);

static struct attribute *cs42l81_attrs[] = {
	&dev_attr_reg.attr,
	&dev_attr_status_227.attr,
	&dev_attr_bringup.attr,
	&dev_attr_dump_key_regs.attr,
	&dev_attr_volume.attr,
	&dev_attr_audio_on.attr,
	&dev_attr_asp_lock.attr,
	&dev_attr_mute.attr,
	&dev_attr_rreg.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cs42l81);

static int cs42l81_dai_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct cs42l81 *c = snd_soc_component_get_drvdata(dai->component);
	int ret;

	mutex_lock(&c->lock);
	ret = cs42l81_audio_on(c);
	mutex_unlock(&c->lock);
	dev_info(&c->spi->dev, "DAI hw_params rate=%u ret=%d\n",
		 params_rate(params), ret);
	return ret;
}

static int cs42l81_dai_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct cs42l81 *c = snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* CPU IIS trigger runs first — BCLK/LRCK should be toggling. */
		mutex_lock(&c->lock);
		ret = cs42l81_asp_lock(c);
		if (!ret && !c->dai_mute)
			cs42l81_set_mute(c, 0);
		mutex_unlock(&c->lock);
		dev_info(&c->spi->dev, "DAI trigger START asp=%d\n", ret);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int cs42l81_dai_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct cs42l81 *c = snd_soc_component_get_drvdata(dai->component);

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;
	mutex_lock(&c->lock);
	c->dai_mute = mute ? 1 : 0;
	cs42l81_apply_user_vol(c);
	mutex_unlock(&c->lock);
	dev_info(&c->spi->dev, "DAI mute=%d user_vol=%u\n", mute, c->user_vol);
	return 0;
}

static const struct snd_soc_dai_ops cs42l81_dai_ops = {
	.hw_params = cs42l81_dai_hw_params,
	.trigger = cs42l81_dai_trigger,
	.mute_stream = cs42l81_dai_mute_stream,
};

static int cs42l81_vol_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = CS42L81_USER_VOL_MAX;
	return 0;
}

static int cs42l81_vol_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);

	ucontrol->value.integer.value[0] = c->user_vol;
	return 0;
}

static int cs42l81_vol_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_soc_kcontrol_component(kcontrol);
	struct cs42l81 *c = snd_soc_component_get_drvdata(comp);
	unsigned int vol = ucontrol->value.integer.value[0];
	int changed;

	if (vol > CS42L81_USER_VOL_MAX)
		return -EINVAL;
	mutex_lock(&c->lock);
	changed = vol != c->user_vol;
	c->user_vol = vol;
	cs42l81_apply_user_vol(c);
	mutex_unlock(&c->lock);
	return changed;
}

static const struct snd_kcontrol_new cs42l81_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Master Playback Volume",
		.info = cs42l81_vol_info,
		.get = cs42l81_vol_get,
		.put = cs42l81_vol_put,
	},
};

static struct snd_soc_dai_driver cs42l81_dai = {
	.name = "cs42l81-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cs42l81_dai_ops,
};

static const struct snd_soc_component_driver cs42l81_component = {
	.idle_bias_on = 1,
	.endianness = 1,
	.controls = cs42l81_controls,
	.num_controls = ARRAY_SIZE(cs42l81_controls),
};

static int cs42l81_probe(struct spi_device *spi)
{
	struct cs42l81 *c;
	int ret;

	c = devm_kzalloc(&spi->dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->spi = spi;
	c->user_vol = CS42L81_USER_VOL_MAX;
	mutex_init(&c->lock);
	spi_set_drvdata(spi, c);

	mutex_lock(&c->lock);
	ret = cs42l81_bringup(c);
	mutex_unlock(&c->lock);
	if (ret)
		dev_warn(&spi->dev,
			 "bring-up SPI err %d (codec unpowered/SPI0 — sysfs still up)\n",
			 ret);

	cs42l81_dev = c;

	ret = sysfs_create_groups(&spi->dev.kobj, cs42l81_groups);
	if (ret)
		dev_warn(&spi->dev, "sysfs groups failed: %d\n", ret);

	ret = devm_snd_soc_register_component(&spi->dev, &cs42l81_component,
					      &cs42l81_dai, 1);
	if (ret) {
		dev_err(&spi->dev, "snd_soc_register_component: %d\n", ret);
		sysfs_remove_groups(&spi->dev.kobj, cs42l81_groups);
		return ret;
	}

	dev_info(&spi->dev, "CS42L81 SPI + ASoC DAI cs42l81-hifi\n");
	return 0;
}

static void cs42l81_remove(struct spi_device *spi)
{
	if (cs42l81_dev == spi_get_drvdata(spi))
		cs42l81_dev = NULL;
	sysfs_remove_groups(&spi->dev.kobj, cs42l81_groups);
}

static const struct of_device_id cs42l81_of_match[] = {
	{ .compatible = "cirrus,cs42l81" },
	{ .compatible = "apple,338s1146" },
	{ }
};
MODULE_DEVICE_TABLE(of, cs42l81_of_match);

/* SPI core warns if OF compatibles have no matching id_table name. */
static const struct spi_device_id cs42l81_ids[] = {
	{ "cs42l81", 0 },
	{ "338s1146", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, cs42l81_ids);

static struct spi_driver cs42l81_driver = {
	.driver = {
		.name = "cs42l81-spi",
		.of_match_table = cs42l81_of_match,
	},
	.id_table = cs42l81_ids,
	.probe = cs42l81_probe,
	.remove = cs42l81_remove,
};
module_spi_driver(cs42l81_driver);

MODULE_DESCRIPTION("CS42L81 SPI codec + ASoC DAI (N31 RetailOS framing)");
MODULE_LICENSE("GPL");
