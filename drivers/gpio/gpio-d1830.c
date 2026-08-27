// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO driver for Dialog Semiconductor D1830 PMIC
 *
 * Exposes selected PMIC register bits as GPIO lines for gpio-keys-polled,
 * implements machine power-off via SEC-observed reg 13 bit0, and registers
 * a power_supply battery using the OSOS ADC path (not ACPI — this SoC
 * has none; power_supply is the Linux equivalent).
 *
 * OSOS RetailOS 1.0.2:
 *   439A98(1) → 2C778(channel 3, 5 samples) → 3477C / 347E4 / 3484C
 *   start: reg48 = (reg48 & 0xF0) | (ch & 0xF) | 0x10
 *   data:  10-bit = (reg50 << 2) | reg49
 * 158C82(3/5) writes bitfields in 87/88 — not the ADC. Keep poweroff
 * unchanged. Do not enable dlg,apply-sec-rails from here.
 *
 * Analog HP needs SEC sub_23EC sibling LDOs (regs 21–23 bit4) plus
 * reg16 bit4. Default probe stays gpio-only. CS42 calls
 * d1830_audio_rails() on prepare. Never replay hibernate cookie
 * writes (regs 1/2/73/96) unless boot_mode bit7 is actually set.
 *
 * Copyright (C) 2026 Vencislav Atanasov <user890104@freemyipod.org>
 */
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include <linux/apple-n31.h>

#define D1830_REG_POWEROFF	13
#define D1830_POWEROFF_BIT	BIT(0)
#define D1830_REG_ADC_CFG	48
#define D1830_REG_ADC_LOW	49
#define D1830_REG_ADC_HIGH	50
#define D1830_ADC_CH_VBAT	3	/* OSOS 439A98 case 1 */
#define D1830_ADC_START		0x10
#define D1830_ADC_SAMPLES	5
#define D1830_ADC_FS_MV		6000	/* 10-bit, 6 V FS (emcore/Apple) */
#define D1830_DESIGN_UAH	200000	/* nano 7 pack, 200 mAh */
#define D1830_DESIGN_MIN_UV	3300000
#define D1830_DESIGN_MAX_UV	4200000

/* Provisional Li-ion empty/full for capacity % (OPEN scale) */
#define D1830_MV_EMPTY		3300
#define D1830_MV_FULL		4200

static bool dump_only;
module_param(dump_only, bool, 0644);
MODULE_PARM_DESC(dump_only,
		 "Log PMIC rail ops, do not write (docs-internal n31-pmic dummies)");

/*
 * Chatty WR/RMW/reg dumps are off by default. Enable with verbose=1, or
 * via dynamic debug (dev_dbg) if the kernel was built with DYNAMIC_DEBUG.
 */
static bool verbose;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose, "Verbose n31-pmic I2C/reg logging (default N; also gpio_d1830.verbose=1 on cmdline)");

#define d1830_vinfo(dev, fmt, ...) \
	do { \
		if (verbose) \
			dev_info((dev), fmt, ##__VA_ARGS__); \
		else \
			dev_dbg((dev), fmt, ##__VA_ARGS__); \
	} while (0)

static bool allow_audio_rails = true;
module_param(allow_audio_rails, bool, 0644);
MODULE_PARM_DESC(allow_audio_rails,
		 "Apply sub_23EC LDO trim from d1830_audio_rails() (default on)");

/* Off by default: false Sleep during NAND CS storms was cutting power. */
static bool sleep_poweroff;
module_param(sleep_poweroff, bool, 0644);
MODULE_PARM_DESC(sleep_poweroff,
		 "Hold Sleep ~500ms → pm_power_off (default N)");

/*
 * The 100 ms r5-r8 sweep was a bring-up aid, not a product poll: Home /
 * Sleep / Play arrive on the PMIC nIRQ (GPIO 86 DIN) via
 * d1830_n31_din_nirq(). Polling it during a NAND CS storm is what
 * produced the phantom "SLEEP PRESS r7=0x0e" that power-watch turned
 * into a poweroff mid-recover. Interrupt-driven by default.
 */
static unsigned int btn_poll_ms;
module_param(btn_poll_ms, uint, 0644);
MODULE_PARM_DESC(btn_poll_ms,
		 "Fallback button poll period in ms (0=off, interrupt-only)");

/*
 * A single glitched I2C byte must not reach userspace either: power-watch
 * turns one KEY_POWER press into reboot(RB_POWER_OFF). Re-read after
 * btn_confirm_ms and only report the press if Sleep is still low.
 */
static unsigned int btn_confirm_ms = 60;
module_param(btn_confirm_ms, uint, 0644);
MODULE_PARM_DESC(btn_confirm_ms,
		 "Re-read delay confirming a Sleep press before KEY_POWER (0=off)");

struct d1830_gpio_map {
	u8 reg;
	u8 bit;
};

struct d1830_gpio {
	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	struct d1830_gpio_map *map;
	int num_gpios;
	struct power_supply *psy;
	struct power_supply *usb_psy;
	struct input_dev *input;
	u8 last_home, last_sleep, last_play;
	u8 sleep_hold;
	int last_r5, last_r6, last_r7, last_r8;
	bool keys_inited;
	bool lsb_logged;
	int last_mv;
	u16 last_adc;
	u8 last_r48, last_r49, last_r50;
	unsigned long last_adc_jiffies;
	int psy_ticks;
	struct delayed_work trace;
	struct delayed_work confirm;
	u8 sleep_pending;
	int trace_r[12];
	int trace_din;
	bool trace_inited;
};

static struct i2c_client *d1830_poweroff_client;

static int d1830_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_IN;
}

static int d1830_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct d1830_gpio *gpio_dev = gpiochip_get_data(chip);
	struct d1830_gpio_map *entry;
	int ret;

	if (offset >= gpio_dev->num_gpios)
		return -EINVAL;

	entry = &gpio_dev->map[offset];
	ret = i2c_smbus_read_byte_data(gpio_dev->client, entry->reg);
	if (ret < 0) {
		dev_err_ratelimited(&gpio_dev->client->dev,
				    "Failed to read reg 0x%02x: %d\n",
				    entry->reg, ret);
		return 0;
	}
	return !!(ret & BIT(entry->bit));
}

static int d1830_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return 0;
}

static int d1830_gpio_direction_output(struct gpio_chip *chip, unsigned int offset,
				       int value)
{
	return -ENOTSUPP;
}

static void d1830_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
}

static int d1830_gpio_parse_dt(struct d1830_gpio *gpio_dev)
{
	struct device *dev = &gpio_dev->client->dev;
	struct device_node *np = dev->of_node;
	int size, i;

	if (!np)
		return -ENODEV;

	size = of_property_count_u32_elems(np, "dlg,gpio-map");
	if (size <= 0 || size % 2) {
		dev_err(dev, "Invalid or missing 'dlg,gpio-map' (size=%d)\n", size);
		return -EINVAL;
	}

	gpio_dev->num_gpios = size / 2;
	gpio_dev->map = devm_kcalloc(dev, gpio_dev->num_gpios,
				     sizeof(*gpio_dev->map), GFP_KERNEL);
	if (!gpio_dev->map)
		return -ENOMEM;

	for (i = 0; i < gpio_dev->num_gpios; i++) {
		u32 reg, bit;

		of_property_read_u32_index(np, "dlg,gpio-map", i * 2, &reg);
		of_property_read_u32_index(np, "dlg,gpio-map", i * 2 + 1, &bit);
		if (reg > 0xff || bit > 7) {
			dev_err(dev, "GPIO %d: reg=0x%x bit=%u out of range\n",
				i, reg, bit);
			return -EINVAL;
		}
		gpio_dev->map[i].reg = (u8)reg;
		gpio_dev->map[i].bit = (u8)bit;
	}
	return 0;
}

static void d1830_cut_power(struct i2c_client *client)
{
	int v, ret;
	u8 out;

	if (!client)
		return;

	dev_emerg(&client->dev, "PMIC poweroff: reg %u |= 0x%02lx\n",
		  D1830_REG_POWEROFF, D1830_POWEROFF_BIT);

	v = i2c_smbus_read_byte_data(client, D1830_REG_POWEROFF);
	out = (v < 0) ? (u8)D1830_POWEROFF_BIT : (u8)(v | D1830_POWEROFF_BIT);

	ret = i2c_smbus_write_byte_data(client, D1830_REG_POWEROFF, out);
	if (ret) {
		dev_emerg(&client->dev, "PMIC poweroff write failed (%d); retry raw 1\n",
			  ret);
		i2c_smbus_write_byte_data(client, D1830_REG_POWEROFF,
					  (u8)D1830_POWEROFF_BIT);
	}

	mdelay(100);
	while (1)
		cpu_relax();
}

static void d1830_pm_power_off(void)
{
	d1830_cut_power(d1830_poweroff_client);
}

static ssize_t do_poweroff_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	d1830_cut_power(client);
	return count;
}
static DEVICE_ATTR_WO(do_poweroff);

static ssize_t audio_rails_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int ret;

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	ret = d1830_audio_rails();
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(audio_rails);

/*
 * Chain from RE (do not invert without new ARM):
 *   user key → PMIC status r5-r8 (sub_26520, read-only in OSOS)
 *   → SoC GPIO 86 DIN 0 = asserted (sub_42BBEC; EFBB4 loops while DIN==0)
 *   → EIC g2 b22 INTLEVEL=0 INTTYPE=1 (40641C(86,1); 7D490 set=high/clear=low)
 *   → VIC EXT3
 * SEC sub_27F4 is IIC1 + rail/hibernate RMW. No PMIC MCU image, no write of
 * r5-r8. OSOS 4118BC never writes 5-8. Linux must not replay 27F4 (reg13).
 */
static void d1830_dump_irq_chain(struct i2c_client *client, const char *tag)
{
	void __iomem *eic, *gpio;
	u8 i;
	int v[12];
	u32 din, pcon, dir, level, stat, en, itype;

	for (i = 0; i < 12; i++)
		v[i] = i2c_smbus_read_byte_data(client, i + 1);
	dev_dbg(&client->dev,
		"n31-pmic %s r1-8=%02x %02x %02x %02x %02x %02x %02x %02x r9-12=%02x %02x %02x %02x\n",
		tag,
		v[0] < 0 ? 0 : v[0], v[1] < 0 ? 0 : v[1],
		v[2] < 0 ? 0 : v[2], v[3] < 0 ? 0 : v[3],
		v[4] < 0 ? 0 : v[4], v[5] < 0 ? 0 : v[5],
		v[6] < 0 ? 0 : v[6], v[7] < 0 ? 0 : v[7],
		v[8] < 0 ? 0 : v[8], v[9] < 0 ? 0 : v[9],
		v[10] < 0 ? 0 : v[10], v[11] < 0 ? 0 : v[11]);

	/* GPIO 86: bank 10, pin 6. EIC group 2, bit 22. */
	gpio = ioremap(0x3cf00000ul + 32u * 10u, 32);
	eic = ioremap(0x39700000ul, 0x100);
	if (gpio && eic) {
		din = readl(gpio + 0x04);
		pcon = readl(gpio);
		dir = readl(gpio + 0x14);
		level = readl(eic + 0x80 + 8);
		stat = readl(eic + 0xa0 + 8);
		en = readl(eic + 0xc0 + 8);
		itype = readl(eic + 0xe0 + 8);
		dev_dbg(&client->dev,
			"n31-pmic %s gpio86 din=%u dir=%u pcon=%08x eic g2 L=%08x S=%08x E=%08x T=%08x bit22 L=%u S=%u E=%u T=%u irq=%d\n",
			tag, !!(din & BIT(6)), !!(dir & BIT(6)), pcon,
			level, stat, en, itype,
			!!(level & BIT(22)), !!(stat & BIT(22)),
			!!(en & BIT(22)), !!(itype & BIT(22)),
			client->irq);
	}
	if (gpio)
		iounmap(gpio);
	if (eic)
		iounmap(eic);

	{
		int r14 = i2c_smbus_read_byte_data(client, 14);
		int r41 = i2c_smbus_read_byte_data(client, 41);
		int r42 = i2c_smbus_read_byte_data(client, 42);
		int r43 = i2c_smbus_read_byte_data(client, 43);
		int r60 = i2c_smbus_read_byte_data(client, 60);

		dev_dbg(&client->dev,
			"n31-pmic %s r14=%02x r41=%02x r42=%02x r43=%02x r60=%02x (SEC want 14=20 41=(x&ec)|10 42=(x&c0)|14 43=(x&f0)|01 60=01)\n",
			tag,
			r14 < 0 ? 0 : r14, r41 < 0 ? 0 : r41,
			r42 < 0 ? 0 : r42, r43 < 0 ? 0 : r43,
			r60 < 0 ? 0 : r60);
	}
}

/*
 * OSOS BatteryLevel 0x588150 (not PCFPowerMgr, not SEC):
 *   51686C → 174288(0x891DB48, 0x1c1f40ee)
 *   174288 sbfx+1 per bit → write ~bytes to regs 9-12
 *   then 5D308(gpio 0x56, level=0, type=1) and EFBB4
 * SEC 27F4 never writes 9-12. Linux DFU skips OSOS, so replay this mask.
 * 158C82(3/5) in the same task is ADC regs 87/88 — not nIRQ.
 */
static void d1830_osos_nirq_mask(struct i2c_client *client)
{
	static const u8 regs[] = { 9, 10, 11, 12 };
	static const u8 vals[] = { 0x11, 0xbf, 0xe0, 0xe3 };
	int i, ret, before[4];

	for (i = 0; i < 4; i++)
		before[i] = i2c_smbus_read_byte_data(client, regs[i]);
	for (i = 0; i < 4; i++) {
		ret = i2c_smbus_write_byte_data(client, regs[i], vals[i]);
		if (ret)
			dev_err(&client->dev,
				"n31-pmic OSOS r%u=0x%02x write %d\n",
				regs[i], vals[i], ret);
	}
	dev_dbg(&client->dev,
		"n31-pmic OSOS 174288 mask r9-12 %02x %02x %02x %02x -> 11 bf e0 e3\n",
		before[0] < 0 ? 0 : before[0], before[1] < 0 ? 0 : before[1],
		before[2] < 0 ? 0 : before[2], before[3] < 0 ? 0 : before[3]);
}

static void d1830_btn_poll_once(struct d1830_gpio *gpio_dev);

/* GPIO 86 DIN edge from n31-btn poll — I2C read even if EIC missed the line. */
void d1830_n31_din_nirq(void)
{
	struct i2c_client *client = d1830_poweroff_client;
	struct d1830_gpio *gpio_dev;

	if (!client)
		return;
	gpio_dev = i2c_get_clientdata(client);
	if (gpio_dev)
		d1830_btn_poll_once(gpio_dev);
}

static irqreturn_t d1830_irq_thread(int irq, void *data)
{
	struct d1830_gpio *gpio_dev = data;
	static unsigned int hits;

	hits++;
	if (hits <= 8 || (hits & 0x3f) == 0) {
		dev_dbg(&gpio_dev->client->dev,
			"n31-pmic nIRQ irq=%d hit=%u (GPIO86 EIC g2 b22)\n",
			irq, hits);
		if (hits == 1)
			d1830_dump_irq_chain(gpio_dev->client, "nirq1");
	}
	d1830_btn_poll_once(gpio_dev);
	return IRQ_HANDLED;
}

static void d1830_key_active_low(struct d1830_gpio *gpio_dev, unsigned int code,
				 u8 now_bit, u8 *last, const char *name)
{
	bool pressed, was;

	if (now_bit == *last)
		return;
	/* OSOS sub_4195D8(id, bit==0): pressed when the status bit is clear. */
	pressed = !now_bit;
	was = !(*last);
	*last = now_bit;
	if (pressed == was)
		return;
	dev_dbg(&gpio_dev->client->dev,
		"n31-btn %s %s (bit=%u, 0=pressed OSOS)\n",
		name, pressed ? "PRESS" : "release", now_bit);
	s5l8740_n31_report_key(code, pressed);
	if (gpio_dev->input) {
		input_report_key(gpio_dev->input, code, pressed);
		input_sync(gpio_dev->input);
	}
}

static void d1830_btn_poll_once(struct d1830_gpio *gpio_dev)
{
	struct i2c_client *client = gpio_dev->client;
	int r5, r6, r7, r8;
	u8 home, sleep, play;

	r5 = i2c_smbus_read_byte_data(client, 5);
	r6 = i2c_smbus_read_byte_data(client, 6);
	r7 = i2c_smbus_read_byte_data(client, 7);
	r8 = i2c_smbus_read_byte_data(client, 8);
	if (r7 < 0)
		return;

	/* OSOS sub_26520: Home=r7b4, Sleep=r7b5, Play=r8b1. */
	home = !!(r7 & BIT(4));
	sleep = !!(r7 & BIT(5));
	play = (r8 >= 0) ? !!(r8 & BIT(1)) : 1;

	if (!gpio_dev->keys_inited) {
		gpio_dev->last_home = home;
		gpio_dev->last_sleep = sleep;
		gpio_dev->last_play = play;
		gpio_dev->last_r5 = r5 < 0 ? 0 : r5;
		gpio_dev->last_r6 = r6 < 0 ? 0 : r6;
		gpio_dev->last_r7 = r7;
		gpio_dev->last_r8 = r8 < 0 ? 0 : r8;
		gpio_dev->keys_inited = true;
		dev_dbg(&client->dev,
			"n31-pmic idle r5=0x%02x r6=0x%02x r7=0x%02x r8=0x%02x home=%u sleep=%u play=%u (OSOS active-low)\n",
			r5 < 0 ? 0 : r5, r6 < 0 ? 0 : r6, r7,
			r8 < 0 ? 0 : r8, home, sleep, play);
		return;
	}

	if (r5 >= 0 && r5 != gpio_dev->last_r5)
		dev_dbg(&client->dev, "n31-pmic r5 0x%02x->0x%02x xor=0x%02x\n",
			gpio_dev->last_r5, r5, gpio_dev->last_r5 ^ r5);
	if (r6 >= 0 && r6 != gpio_dev->last_r6)
		dev_dbg(&client->dev, "n31-pmic r6 0x%02x->0x%02x xor=0x%02x\n",
			gpio_dev->last_r6, r6, gpio_dev->last_r6 ^ r6);
	if (r7 != gpio_dev->last_r7)
		dev_dbg(&client->dev, "n31-pmic r7 0x%02x->0x%02x xor=0x%02x\n",
			gpio_dev->last_r7, r7, gpio_dev->last_r7 ^ r7);
	if (r8 >= 0 && r8 != gpio_dev->last_r8)
		dev_dbg(&client->dev, "n31-pmic r8 0x%02x->0x%02x xor=0x%02x\n",
			gpio_dev->last_r8, r8, gpio_dev->last_r8 ^ r8);

	d1830_key_active_low(gpio_dev, KEY_HOMEPAGE, home,
			     &gpio_dev->last_home, "HOME");
	d1830_key_active_low(gpio_dev, KEY_PLAYPAUSE, play,
			     &gpio_dev->last_play, "PLAY");

	if (!sleep) {
		if (gpio_dev->last_sleep) {
			/*
			 * First low sample only arms the press. A real press easily
			 * outlives btn_confirm_ms; a glitched I2C byte during a NAND
			 * CS storm does not, and power-watch turns one KEY_POWER into
			 * reboot(RB_POWER_OFF).
			 */
			if (btn_confirm_ms && !gpio_dev->sleep_pending) {
				gpio_dev->sleep_pending = 1;
				dev_dbg(&client->dev,
					"n31-btn SLEEP arm r7=0x%02x (confirm in %ums)\n",
					r7, btn_confirm_ms);
				schedule_delayed_work(&gpio_dev->confirm,
					      msecs_to_jiffies(btn_confirm_ms));
				return;
			}
			gpio_dev->sleep_pending = 0;
			d1830_vinfo(&client->dev,
				 "n31-btn SLEEP PRESS r7=0x%02x (bit5 1->0)\n",
				 r7);
			s5l8740_n31_report_key(KEY_POWER, 1);
			if (gpio_dev->input) {
				input_report_key(gpio_dev->input, KEY_POWER, 1);
				input_sync(gpio_dev->input);
			}
		}
		/* Hold Sleep across 5 polls (~500ms) before cutting power.
		 * Short press still emits KEY_POWER for power-watch /
		 * pm_power_off; hold is the kernel-direct fallback when
		 * userspace is not watching. One noisy I2C byte must not
		 * hibernate.
		 */
		if (gpio_dev->sleep_hold < 5)
			gpio_dev->sleep_hold++;
		if (gpio_dev->sleep_hold == 5) {
			if (!sleep_poweroff) {
				d1830_vinfo(&client->dev,
					      "n31-btn SLEEP held (poweroff disabled; sleep_poweroff=1 to enable)\n");
			} else {
				dev_warn(&client->dev,
					 "n31-btn SLEEP held — poweroff\n");
				/* Prefer machine pm_power_off (same cut_power). */
				if (pm_power_off)
					pm_power_off();
				else
					d1830_cut_power(client);
			}
		}
	} else {
		if (!gpio_dev->last_sleep) {
			dev_dbg(&client->dev,
				"n31-btn SLEEP release r7=0x%02x\n", r7);
			s5l8740_n31_report_key(KEY_POWER, 0);
			if (gpio_dev->input) {
				input_report_key(gpio_dev->input, KEY_POWER, 0);
				input_sync(gpio_dev->input);
			}
		}
		gpio_dev->sleep_pending = 0;
		gpio_dev->sleep_hold = 0;
	}
	gpio_dev->last_sleep = sleep;
	if (r5 >= 0)
		gpio_dev->last_r5 = r5;
	if (r6 >= 0)
		gpio_dev->last_r6 = r6;
	gpio_dev->last_r7 = r7;
	if (r8 >= 0)
		gpio_dev->last_r8 = r8;
}

/* Split the silent-PMIC case: do r5-r8 bits move when Home/Play/Sleep
 * are pressed, and does GPIO 86 DIN follow? Not a product poll.
 */
static void d1830_trace_work(struct work_struct *work)
{
	struct d1830_gpio *gpio_dev = container_of(to_delayed_work(work),
					   struct d1830_gpio, trace);
	unsigned int period = btn_poll_ms ? btn_poll_ms : 1000;
	unsigned int psy_every = 10000 / period;

	if (btn_poll_ms)
		d1830_btn_poll_once(gpio_dev);
	if (gpio_dev->psy && ++gpio_dev->psy_ticks >= (psy_every ? psy_every : 1)) {
		gpio_dev->psy_ticks = 0;
		power_supply_changed(gpio_dev->psy);
		if (gpio_dev->usb_psy)
			power_supply_changed(gpio_dev->usb_psy);
	}
	schedule_delayed_work(&gpio_dev->trace, msecs_to_jiffies(period));
}

/* Second look at Sleep after btn_confirm_ms; see d1830_btn_poll_once(). */
static void d1830_confirm_work(struct work_struct *work)
{
	struct d1830_gpio *gpio_dev = container_of(to_delayed_work(work),
					   struct d1830_gpio, confirm);

	d1830_btn_poll_once(gpio_dev);
}

/*
 * OSOS 3477C + 347E4 + 2C778(3, 5). Channel 3 is VBAT. 10-bit sample
 * averaged 5×. Scale is 10-bit * 6 V FS / 1023 (emcore / OSOS).
 * 439A98 then >>2 — logged only. RetailOS UI cache at 0x891DB18
 * is still unmapped. No writes to 87/88. Never write reg 13 here.
 */
static int d1830_adc_once(struct d1830_gpio *gpio_dev, int *adc,
			  u8 *r48, u8 *r49, u8 *r50)
{
	struct i2c_client *client = gpio_dev->client;
	int cfg, hi, lo, i;

	cfg = i2c_smbus_read_byte_data(client, D1830_REG_ADC_CFG);
	if (cfg < 0)
		return cfg;
	if (cfg & D1830_ADC_START) {
		usleep_range(1000, 1500);
		cfg = i2c_smbus_read_byte_data(client, D1830_REG_ADC_CFG);
		if (cfg < 0)
			return cfg;
		if (cfg & D1830_ADC_START)
			return -EBUSY;
	}
	cfg = i2c_smbus_write_byte_data(client, D1830_REG_ADC_CFG,
					(cfg & 0xF0) | D1830_ADC_CH_VBAT |
					D1830_ADC_START);
	if (cfg)
		return cfg;

	for (i = 0; i < 5; i++) {
		usleep_range(1000, 1500);
		cfg = i2c_smbus_read_byte_data(client, D1830_REG_ADC_CFG);
		if (cfg < 0)
			return cfg;
		if (cfg & D1830_ADC_START)
			break;
	}

	hi = i2c_smbus_read_byte_data(client, D1830_REG_ADC_HIGH);
	lo = i2c_smbus_read_byte_data(client, D1830_REG_ADC_LOW);
	if (hi < 0)
		return hi;
	if (lo < 0)
		return lo;
	*r48 = (u8)cfg;
	*r49 = (u8)lo;
	*r50 = (u8)hi;
	/* 347E4: (reg50 << 2) | reg49. Mask 10-bit; low nibble is 2 LSBs. */
	*adc = ((hi << 2) | lo) & 0x3ff;
	return 0;
}

static int d1830_adc_to_mv(int adc)
{
	return (adc * D1830_ADC_FS_MV) / 1023;
}

static int d1830_read_vbat(struct d1830_gpio *gpio_dev, int *mv)
{
	int i, ret, adc, sum = 0, n = 0;
	u8 r48 = 0, r49 = 0, r50 = 0;

	if (gpio_dev->last_adc_jiffies &&
	    time_before(jiffies, gpio_dev->last_adc_jiffies + HZ / 2) &&
	    gpio_dev->last_mv > 0) {
		*mv = gpio_dev->last_mv;
		return 0;
	}

	for (i = 0; i < D1830_ADC_SAMPLES; i++) {
		ret = d1830_adc_once(gpio_dev, &adc, &r48, &r49, &r50);
		if (ret) {
			if (n)
				break;
			return ret;
		}
		sum += adc;
		n++;
	}
	if (!n)
		return -EIO;
	adc = sum / n;
	*mv = d1830_adc_to_mv(adc);
	gpio_dev->last_adc = (u16)adc;
	gpio_dev->last_mv = *mv;
	gpio_dev->last_r48 = r48;
	gpio_dev->last_r49 = r49;
	gpio_dev->last_r50 = r50;
	gpio_dev->last_adc_jiffies = jiffies;

	if (!gpio_dev->lsb_logged) {
		dev_dbg(&gpio_dev->client->dev,
			 "n31-bat OSOS 2C778 ch3 adc=%u r48=%02x r49=%02x r50=%02x mv=%d osos>>2=%u\n",
			 adc, r48, r49, r50, *mv, adc >> 2);
		gpio_dev->lsb_logged = true;
	}
	return 0;
}

static int d1830_get_vbat_uV(struct d1830_gpio *gpio_dev, int *val)
{
	int mv, ret;

	ret = d1830_read_vbat(gpio_dev, &mv);
	if (ret)
		return ret;
	*val = mv * 1000;
	return 0;
}

static int d1830_get_capacity(struct d1830_gpio *gpio_dev, int *val)
{
	int mv, pct, ret;

	ret = d1830_read_vbat(gpio_dev, &mv);
	if (ret)
		return ret;
	if (mv <= D1830_MV_EMPTY)
		pct = 0;
	else if (mv >= D1830_MV_FULL)
		pct = 100;
	else
		pct = ((mv - D1830_MV_EMPTY) * 100) /
		      (D1830_MV_FULL - D1830_MV_EMPTY);
	*val = clamp_val(pct, 0, 100);
	return 0;
}

static ssize_t vbat_raw_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct d1830_gpio *gpio_dev = i2c_get_clientdata(to_i2c_client(dev));
	int mv, ret;

	ret = d1830_read_vbat(gpio_dev, &mv);
	if (ret)
		return ret;
	return sysfs_emit(buf,
			  "r48=%02x r49=%02x r50=%02x adc=%u mv=%d\n",
			  gpio_dev->last_r48, gpio_dev->last_r49,
			  gpio_dev->last_r50, gpio_dev->last_adc, mv);
}
static DEVICE_ATTR_RO(vbat_raw);

static int d1830_psy_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct d1830_gpio *gpio_dev = power_supply_get_drvdata(psy);
	int mv, pct, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return d1830_get_vbat_uV(gpio_dev, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = D1830_DESIGN_MIN_UV;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = D1830_DESIGN_MAX_UV;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = D1830_DESIGN_UAH;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		return d1830_get_capacity(gpio_dev, &val->intval);
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = d1830_get_capacity(gpio_dev, &pct);
		if (ret)
			return ret;
		if (pct <= 5)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else if (pct <= 15)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (pct >= 95)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (pct >= 80)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		ret = d1830_read_vbat(gpio_dev, &mv);
		if (ret) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			return 0;
		}
		/* USB gadget is the only supply we have; no charge-bit RE. */
		if (mv >= 4150)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = d1830_read_vbat(gpio_dev, &mv);
		if (ret) {
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
			return 0;
		}
		if (mv < 3000)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else if (mv > 4300)
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property d1830_psy_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_SCOPE,
};

static int d1830_usb_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		/* Gadget host is the only path this image runs. */
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = POWER_SUPPLY_USB_TYPE_SDP;
		return 0;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property d1830_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_SCOPE,
};

static int d1830_write8(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	d1830_vinfo(&client->dev, "n31-pmic: WR %02x <- %02x%s\n",
		    reg, val, dump_only ? " (suppressed)" : "");
	if (dump_only)
		return 0;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret)
		dev_err(&client->dev, "n31-pmic: WR %02x failed ret=%d\n",
			reg, ret);
	return ret;
}

static int d1830_rmw(struct i2c_client *client, u8 reg, u8 clear, u8 set)
{
	int v = i2c_smbus_read_byte_data(client, reg);
	u8 newv;

	if (v < 0)
		return v;
	newv = (u8)((v & ~clear) | set);
	d1830_vinfo(&client->dev,
		    "n31-pmic: RMW reg=%02x old=%02x clear=%02x set=%02x new=%02x%s\n",
		    reg, v, clear, set, newv, dump_only ? " (suppressed)" : "");
	if (dump_only)
		return 0;
	return i2c_smbus_write_byte_data(client, reg, newv);
}

static void d1830_log_audio_regs(struct i2c_client *client, const char *tag)
{
	static const u8 regs[] = {
		1, 2, 3, 5, 13, 14, 16, 17, 19, 20, 21, 22, 23, 26, 35,
		36, 37, 38, 41, 42, 43, 48, 89, 96, 110, 111
	};
	char hex[8];
	int i, v;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		v = i2c_smbus_read_byte_data(client, regs[i]);
		if (v < 0)
			snprintf(hex, sizeof(hex), "ERR");
		else
			snprintf(hex, sizeof(hex), "%02x", v);
		d1830_vinfo(&client->dev, "n31-pmic: %s RD %02x -> %s\n",
			    tag, regs[i], hex);
	}
}

/*
 * OSOS sub_20766(1) → 439B00(1) → 6644(4) → 7484(pmic, 9, on):
 * RMW D1830 register 16 bit 5. Targeted Nimbus rail — not the SEC seq.
 */
int d1830_nimbus_rail(bool on)
{
	struct i2c_client *client = d1830_poweroff_client;
	int before, ret;

	if (!client)
		return -ENODEV;
	before = i2c_smbus_read_byte_data(client, 16);
	if (before < 0)
		return before;
	ret = d1830_rmw(client, 16, BIT(5), on ? BIT(5) : 0);
	d1830_vinfo(&client->dev, "nimbus rail reg16 0x%02x -> bit5=%d ret=%d\n",
		    before, on, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(d1830_nimbus_rail);

/*
 * SEC sub_23EC trim — sibling LDOs used by analog HP.
 *
 * Bootloader writes computed 5-bit values from tables at 0x22004B70 /
 * 0x22004B80 (not recovered). Live Linux (gpio-only): r20=0x1a already
 * has bit4; r21=0x0a (no bit4); r22=r23=0x00. Match the group by
 * setting bit4 on 20–23 and forcing 21/22/23 to the same value, then
 * the documented r16/r17/r19 RMWs. Skip r26=0xB2 (charge). Never
 * touch POWEROFF (reg 13).
 *
 * Cold-boot sub_23EC also sets r16 bit5 (same bit Nimbus uses). Keep
 * that — OSOS 7484 will still toggle it for BT.
 */
static int d1830_sec_trim_seq(struct i2c_client *client, u8 boot_mode)
{
	int v20, v21, v35;
	u8 fill, r16;

	d1830_vinfo(&client->dev,
		 "n31-pmic: sub_23EC-equivalent begin boot_mode=0x%02x\n",
		 boot_mode);

	v35 = i2c_smbus_read_byte_data(client, 35);
	if (v35 >= 0)
		d1830_write8(client, 35, (u8)(v35 & 0xFC));

	v20 = i2c_smbus_read_byte_data(client, 20);
	v21 = i2c_smbus_read_byte_data(client, 21);
	if (v20 < 0 || v21 < 0)
		return v20 < 0 ? v20 : v21;

	/* Keep r20 extras (live 0x1a). 21–23 share one 5-bit field + bit4. */
	d1830_write8(client, 20, (u8)(v20 | 0x10));
	fill = (u8)((v21 & 0x1f) | 0x10);
	d1830_write8(client, 21, fill);
	d1830_write8(client, 22, fill);
	d1830_write8(client, 23, fill);

	v21 = i2c_smbus_read_byte_data(client, 16);
	if (v21 < 0)
		return v21;
	r16 = (u8)((v21 & 0x2f) | 0x10);
	if (!boot_mode)
		r16 |= 0x20;
	d1830_write8(client, 16, r16);

	d1830_rmw(client, 17, 0, 0x07);
	d1830_rmw(client, 19, 0, 0x02);

	d1830_vinfo(&client->dev, "n31-pmic: sub_23EC-equivalent complete\n");
	return 0;
}

/*
 * sub_23EC analog-adjacent trim only. Called from CS42 prepare.
 * Does not run hibernate cookie / POWEROFF / charger 0xB2.
 */
int d1830_audio_rails(void)
{
	struct i2c_client *client = d1830_poweroff_client;
	int r02, ret;

	if (!client)
		return -ENODEV;
	if (!allow_audio_rails) {
		d1830_vinfo(&client->dev, "n31-pmic: audio rails skipped (allow_audio_rails=0)\n");
		d1830_log_audio_regs(client, "audio-skip");
		return 0;
	}

	d1830_log_audio_regs(client, "audio-before");
	r02 = i2c_smbus_read_byte_data(client, 2);
	if (r02 < 0)
		return r02;
	ret = d1830_sec_trim_seq(client, (r02 & 0x80) ? 0x11 : 0x00);
	/* sub_27F4 analog-adjacent: r14 fixed 0x20. Not POWEROFF. */
	if (!ret)
		d1830_write8(client, 14, 0x20);
	d1830_log_audio_regs(client, "audio-after");
	return ret;
}
EXPORT_SYMBOL_GPL(d1830_audio_rails);

/*
 * IpodSec PMIC rail / charge bring-up. Opt-in via dlg,apply-sec-rails.
 *
 * The old Linux seq always wrote the hibernate-to-standby cluster
 * (reg2=0x80, reg73=0, reg1=0, cookie@96) and historically POWEROFF.
 * Bootloader only does that when reg2 bit7 is set. Match the dummies
 * doc: detect boot_mode, skip reset/hibernate unless that branch,
 * never write reg 13 here.
 */
static int d1830_sec_rail_seq(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int r02, r01;
	u8 boot_mode = 0;

	r02 = i2c_smbus_read_byte_data(client, 2);
	if (r02 < 0)
		return r02;
	d1830_vinfo(dev, "n31-pmic: RD 02 -> %02x\n", r02);
	if (r02 & 0x80) {
		boot_mode = 0x11;
		d1830_write8(client, 2, 0x80);
	}
	d1830_vinfo(dev, "n31-pmic: boot_mode=0x%02x\n", boot_mode);

	r01 = i2c_smbus_read_byte_data(client, 1);
	if (r01 < 0)
		return r01;
	d1830_vinfo(dev, "n31-pmic: RD 01 -> %02x\n", r01);

	if (boot_mode == 0x11) {
		dev_warn(dev,
			 "n31-pmic: hibernate-to-standby path detected, reset suppressed (reg01=%02x)\n",
			 r01);
		/* Do not write reg 0x49/0x60/0x01/0x0d or call sub_1130. */
	}

	d1830_sec_trim_seq(client, boot_mode);

	/* sub_27F4 post-trim, non-fatal. NEVER POWEROFF (reg 13). */
	d1830_rmw(client, 48, 0, 0x40);
	d1830_rmw(client, 89, 0x1C, 0);
	d1830_write8(client, 60, 0x01);
	if (boot_mode != 0x11)
		d1830_rmw(client, 41, 0x13, 0x10);
	d1830_rmw(client, 42, 0x3F, 0x14);
	d1830_rmw(client, 43, 0x0F, 0x01);
	d1830_write8(client, 14, 0x20);
	d1830_rmw(client, 38, 0x01, 0);

	d1830_vinfo(dev, "n31-pmic: sub_27F4-equivalent complete (POWEROFF skipped)\n");
	return 0;
}

static int d1830_gpio_probe(struct i2c_client *client)
{
	struct d1830_gpio *gpio_dev;
	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "Adapter does not support SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	gpio_dev = devm_kzalloc(dev, sizeof(*gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	gpio_dev->client = client;
	i2c_set_clientdata(client, gpio_dev);

	ret = d1830_gpio_parse_dt(gpio_dev);
	if (ret)
		return ret;

	d1830_poweroff_client = client;

	/* Opt-in only. Default probe is GPIO + VBAT reads — no rail writes.
	 * The old default seq wrote reg 13 = 0x01 (POWEROFF bit) at boot.
	 */
	if (of_property_read_bool(dev->of_node, "dlg,apply-sec-rails"))
		d1830_sec_rail_seq(client);
	else
		d1830_vinfo(dev, "d1830 gpio-only (rail seq off; CS42 calls d1830_audio_rails)\n");

	d1830_log_audio_regs(client, "probe");

	gpio_dev->gpio_chip.label = dev_name(dev);
	gpio_dev->gpio_chip.parent = dev;
	gpio_dev->gpio_chip.owner = THIS_MODULE;
	gpio_dev->gpio_chip.base = -1;
	gpio_dev->gpio_chip.ngpio = gpio_dev->num_gpios;
	gpio_dev->gpio_chip.can_sleep = true;
	gpio_dev->gpio_chip.get_direction = d1830_gpio_get_direction;
	gpio_dev->gpio_chip.direction_input = d1830_gpio_direction_input;
	gpio_dev->gpio_chip.direction_output = d1830_gpio_direction_output;
	gpio_dev->gpio_chip.get = d1830_gpio_get;
	gpio_dev->gpio_chip.set = d1830_gpio_set;

	ret = devm_gpiochip_add_data(dev, &gpio_dev->gpio_chip, gpio_dev);
	if (ret) {
		dev_err(dev, "Failed to add GPIO chip: %d\n", ret);
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_do_poweroff);
	if (ret)
		dev_warn(dev, "sysfs do_poweroff unavailable: %d\n", ret);

	ret = device_create_file(dev, &dev_attr_audio_rails);
	if (ret)
		dev_warn(dev, "sysfs audio_rails unavailable: %d\n", ret);

	ret = device_create_file(dev, &dev_attr_vbat_raw);
	if (ret)
		dev_warn(dev, "sysfs vbat_raw unavailable: %d\n", ret);

	if (!pm_power_off) {
		pm_power_off = d1830_pm_power_off;
		d1830_vinfo(dev, "registered pm_power_off (SEC reg %u bit0)\n",
			 D1830_REG_POWEROFF);
	} else {
		dev_warn(dev, "pm_power_off already set — sysfs do_poweroff only\n");
	}

	/* OSOS 9-12 mask only. No 27F4 tail, no 1-4 writeback, no IIC1 peek. */
	d1830_dump_irq_chain(client, "sec-left");
	d1830_osos_nirq_mask(client);
	d1830_dump_irq_chain(client, "osos-mask");

	if (client->irq > 0) {
		struct irq_data *d = irq_get_irq_data(client->irq);

		if (d)
			s5l8740_eic_enable_gpio(d->hwirq, IRQ_TYPE_LEVEL_LOW);
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						d1830_irq_thread,
						IRQF_ONESHOT | IRQF_TRIGGER_LOW,
						"d1830-nirq", gpio_dev);
		if (ret)
			dev_err(dev, "PMIC nIRQ %d failed: %d\n",
				client->irq, ret);
		else
			d1830_vinfo(dev,
				 "PMIC nIRQ virq=%d hwirq=%lu LEVEL_LOW (OSOS 40641C type=1, EFBB4 DIN=0)\n",
				 client->irq, d ? d->hwirq : 0);
	} else {
		dev_err(dev, "no PMIC nIRQ in DT (of_irq did not map GPIO 86)\n");
	}
	d1830_dump_irq_chain(client, "irq-on");

	{
		int v = i2c_smbus_read_byte_data(client, 7);

		d1830_vinfo(dev,
			 "PMIC 7bit=0x%02x wire WR=0x%02x RD=0x%02x reg7 %s (%d)\n",
			 client->addr, client->addr << 1,
			 (client->addr << 1) | 1,
			 v < 0 ? "read fail" : "ok", v);
	}

	{
		struct power_supply_config psy_cfg = {
			.drv_data = gpio_dev,
			.of_node = dev->of_node,
		};
		struct power_supply_desc *desc;
		int mv;

		desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
		if (desc) {
			desc->name = "d1830-battery";
			desc->type = POWER_SUPPLY_TYPE_BATTERY;
			desc->properties = d1830_psy_props;
			desc->num_properties = ARRAY_SIZE(d1830_psy_props);
			desc->get_property = d1830_psy_get_property;
			gpio_dev->psy = devm_power_supply_register(dev, desc, &psy_cfg);
			if (IS_ERR(gpio_dev->psy)) {
				dev_warn(dev, "battery psy unavailable: %ld\n",
					 PTR_ERR(gpio_dev->psy));
				gpio_dev->psy = NULL;
			} else if (!d1830_read_vbat(gpio_dev, &mv)) {
				d1830_vinfo(dev,
					 "battery psy OSOS ch3 10-bit*6 mV=%d (design %u mAh)\n",
					 mv, D1830_DESIGN_UAH / 1000);
			}
		}

		desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
		if (desc) {
			desc->name = "d1830-usb";
			desc->type = POWER_SUPPLY_TYPE_USB;
			desc->properties = d1830_usb_props;
			desc->num_properties = ARRAY_SIZE(d1830_usb_props);
			desc->get_property = d1830_usb_get_property;
			desc->usb_types = BIT(POWER_SUPPLY_USB_TYPE_SDP);
			gpio_dev->usb_psy = devm_power_supply_register(dev, desc,
								       &psy_cfg);
			if (IS_ERR(gpio_dev->usb_psy)) {
				dev_warn(dev, "usb psy unavailable: %ld\n",
					 PTR_ERR(gpio_dev->usb_psy));
				gpio_dev->usb_psy = NULL;
			}
		}
	}

	d1830_vinfo(dev, "Registered %u read-only GPIOs using Dialog D1830 driver\n",
		 gpio_dev->num_gpios);

	gpio_dev->input = devm_input_allocate_device(dev);
	if (gpio_dev->input) {
		gpio_dev->input->name = "n31-pmic-buttons";
		gpio_dev->input->phys = "d1830/gpio";
		gpio_dev->input->dev.parent = dev;
		gpio_dev->input->id.bustype = BUS_I2C;
		input_set_capability(gpio_dev->input, EV_KEY, KEY_HOMEPAGE);
		input_set_capability(gpio_dev->input, EV_KEY, KEY_POWER);
		input_set_capability(gpio_dev->input, EV_KEY, KEY_PLAYPAUSE);
		if (input_register_device(gpio_dev->input))
			gpio_dev->input = NULL;
	}

	/* Idle snapshot only. Home / Sleep / Play arrive on the PMIC nIRQ
	 * (GPIO 86) via d1830_n31_din_nirq; btn_poll_ms re-enables the old
	 * 100ms sweep if an EIC edge is ever missed. The slow tick that
	 * remains is the battery power_supply refresh.
	 */
	d1830_btn_poll_once(gpio_dev);
	INIT_DELAYED_WORK(&gpio_dev->trace, d1830_trace_work);
	INIT_DELAYED_WORK(&gpio_dev->confirm, d1830_confirm_work);
	schedule_delayed_work(&gpio_dev->trace,
			      msecs_to_jiffies(btn_poll_ms ? btn_poll_ms : 1000));
	d1830_n31_din_nirq_hook = d1830_n31_din_nirq;
	return 0;
}

static void d1830_gpio_remove(struct i2c_client *client)
{
	struct d1830_gpio *gpio_dev = i2c_get_clientdata(client);

	if (gpio_dev) {
		cancel_delayed_work_sync(&gpio_dev->trace);
		cancel_delayed_work_sync(&gpio_dev->confirm);
	}
	device_remove_file(&client->dev, &dev_attr_vbat_raw);
	device_remove_file(&client->dev, &dev_attr_audio_rails);
	device_remove_file(&client->dev, &dev_attr_do_poweroff);
	if (pm_power_off == d1830_pm_power_off)
		pm_power_off = NULL;
	d1830_n31_din_nirq_hook = NULL;
	d1830_poweroff_client = NULL;
}

static const struct of_device_id d1830_gpio_of_match[] = {
	{ .compatible = "dlg,d1830-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, d1830_gpio_of_match);

static const struct i2c_device_id d1830_gpio_id[] = {
	{ "d1830-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, d1830_gpio_id);

static struct i2c_driver d1830_gpio_driver = {
	.driver = {
		.name = "gpio-d1830",
		.of_match_table = d1830_gpio_of_match,
	},
	.probe = d1830_gpio_probe,
	.remove = d1830_gpio_remove,
	.id_table = d1830_gpio_id,
};

module_i2c_driver(d1830_gpio_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("Dialog Semiconductor D1830 PMIC GPIO + poweroff + battery");
MODULE_LICENSE("GPL v2");
