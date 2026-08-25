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

int s5l8740_eic_enable_gpio(unsigned int gpio, unsigned int irq_type);
void s5l8740_n31_report_key(unsigned int code, int pressed);
int s5l8740_n31_din86(void);
extern void (*d1830_n31_din_nirq_hook)(void);

/* Provisional Li-ion empty/full for capacity % (OPEN scale) */
#define D1830_MV_EMPTY		3300
#define D1830_MV_FULL		4200

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
	static unsigned hits;

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
			dev_dbg(&client->dev,
				"n31-btn SLEEP PRESS r7=0x%02x (bit5 1->0)\n",
				r7);
			s5l8740_n31_report_key(KEY_POWER, 1);
			if (gpio_dev->input) {
				input_report_key(gpio_dev->input, KEY_POWER, 1);
				input_sync(gpio_dev->input);
			}
		}
		/* Hold Sleep across 5 polls (~500ms) before cutting power.
		 * One noisy I2C byte must not hibernate. */
		if (gpio_dev->sleep_hold < 5)
			gpio_dev->sleep_hold++;
		if (gpio_dev->sleep_hold == 5) {
			dev_warn(&client->dev,
				 "n31-btn SLEEP held — poweroff\n");
			d1830_cut_power(client);
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

	d1830_btn_poll_once(gpio_dev);
	if (gpio_dev->psy && ++gpio_dev->psy_ticks >= 100) {
		gpio_dev->psy_ticks = 0;
		power_supply_changed(gpio_dev->psy);
		if (gpio_dev->usb_psy)
			power_supply_changed(gpio_dev->usb_psy);
	}
	schedule_delayed_work(&gpio_dev->trace, msecs_to_jiffies(100));
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

static int d1830_rmw(struct i2c_client *client, u8 reg, u8 clear, u8 set)
{
	int v = i2c_smbus_read_byte_data(client, reg);

	if (v < 0)
		return v;
	return i2c_smbus_write_byte_data(client, reg, (u8)((v & ~clear) | set));
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
	dev_info(&client->dev, "nimbus rail reg16 0x%02x -> bit5=%d ret=%d\n",
		 before, on, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(d1830_nimbus_rail);

/*
 * IpodSec PMIC rail / charge bring-up:
 *   sub_23EC — regs 20–23,26,16,17,19,35 (charge/rail-ish)
 *   sub_27F4 — IIC1 init already done by i2c driver; apply safe RMW sequence
 *              (skip hibernate Stpr cookie / fatal halt paths).
 */
static int d1830_sec_rail_seq(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int ret, v;
	u8 b;

	/* --- sub_23EC (rail/charge) --- */
	/* Reg20 ← (delay-derived) & 0x1F: use 0x10 as safe mid rail enable-ish */
	ret = i2c_smbus_write_byte_data(client, 20, 0x10);
	if (ret)
		dev_warn(dev, "rail reg20: %d\n", ret);

	v = i2c_smbus_read_byte_data(client, 35);
	if (v >= 0) {
		b = (u8)(v & 0xFC);
		ret = i2c_smbus_write_byte_data(client, 35, b);
		if (ret)
			dev_warn(dev, "rail reg35: %d\n", ret);
	}

	/* Regs 21–23 same pattern as 20 in SEC loop — use 0x10 */
	i2c_smbus_write_byte_data(client, 21, 0x10);
	i2c_smbus_write_byte_data(client, 22, 0x10);
	i2c_smbus_write_byte_data(client, 23, 0x10);

	/* Reg26 ← 0xB2 (-78) twice in SEC */
	i2c_smbus_write_byte_data(client, 26, 0xB2);
	i2c_smbus_write_byte_data(client, 26, 0xB2);

	v = i2c_smbus_read_byte_data(client, 16);
	if (v >= 0) {
		b = (u8)((v & 0x2F) | 0x10);
		/* optional |0x20 path when a1 set — keep base */
		i2c_smbus_write_byte_data(client, 16, b);
	}

	v = i2c_smbus_read_byte_data(client, 17);
	if (v >= 0)
		i2c_smbus_write_byte_data(client, 17, (u8)(v | 0x07));

	v = i2c_smbus_read_byte_data(client, 19);
	if (v >= 0)
		i2c_smbus_write_byte_data(client, 19, (u8)(v | 0x02));

	/* --- sub_27F4 safe subset (non-fatal) --- */
	i2c_smbus_write_byte_data(client, 2, 0x80);
	i2c_smbus_write_byte_data(client, 73, 0x00);
	i2c_smbus_write_byte_data(client, 1, 0x00);
	/* clear 4-byte cookie @96 without hibernate SPI (SEC writes 4 bytes) */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
		u8 z[4] = { 0, 0, 0, 0 };

		i2c_smbus_write_i2c_block_data(client, 96, 4, z);
	} else {
		i2c_smbus_write_byte_data(client, 96, 0);
	}
	/* NEVER write reg 13 here — bit0 is D1830_POWEROFF_BIT (cuts Vbat). */
	d1830_rmw(client, 48, 0, 0x40);		/* |= 0x40 */
	d1830_rmw(client, 89, 0x1C, 0);		/* &= 0xE3 */
	i2c_smbus_write_byte_data(client, 60, 0x01);
	d1830_rmw(client, 41, 0x13, 0x10);	/* (x & 0xEC) | 0x10 */
	d1830_rmw(client, 42, 0x3F, 0x14);	/* (x & 0xC0) | 0x14 */
	d1830_rmw(client, 43, 0x0F, 0x01);	/* (x & 0xF0) | 0x01 */
	i2c_smbus_write_byte_data(client, 14, 0x20);
	/* 36/37 depend on ADC helper — leave unread defaults */
	d1830_rmw(client, 38, 0x01, 0);		/* clear bit0 */
	/* do not RMW reg 13 — poweroff register */

	dev_info(dev, "SEC PMIC rail seq applied (sub_23EC + sub_27F4 safe)\n");
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

	/* Opt-in only. Default probe is GPIO + VBAT reads — no rail writes.
	 * The old default seq wrote reg 13 = 0x01 (POWEROFF bit) at boot.
	 */
	if (of_property_read_bool(dev->of_node, "dlg,apply-sec-rails"))
		d1830_sec_rail_seq(client);
	else
		dev_info(dev, "d1830 gpio-only (rail seq off; set dlg,apply-sec-rails to enable)\n");

	{
		static const u8 dump_regs[] = {
			1, 2, 3, 5, 13, 14, 16, 17, 19, 20, 21, 22, 23,
			26, 35, 36, 37, 41, 48, 49, 50, 96, 110, 111
		};
		int i, v;

		dev_dbg(dev, "PMIC identity dump @0x%02x:\n", client->addr);
		for (i = 0; i < ARRAY_SIZE(dump_regs); i++) {
			v = i2c_smbus_read_byte_data(client, dump_regs[i]);
			if (v < 0)
				dev_dbg(dev, "  reg 0x%02u: ERR %d\n", dump_regs[i], v);
			else
				dev_dbg(dev, "  reg 0x%02u = 0x%02x\n", dump_regs[i], v);
		}
	}

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

	ret = device_create_file(dev, &dev_attr_vbat_raw);
	if (ret)
		dev_warn(dev, "sysfs vbat_raw unavailable: %d\n", ret);

	d1830_poweroff_client = client;
	if (!pm_power_off) {
		pm_power_off = d1830_pm_power_off;
		dev_info(dev, "registered pm_power_off (SEC reg %u bit0)\n",
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
			dev_info(dev,
				 "PMIC nIRQ virq=%d hwirq=%lu LEVEL_LOW (OSOS 40641C type=1, EFBB4 DIN=0)\n",
				 client->irq, d ? d->hwirq : 0);
	} else {
		dev_err(dev, "no PMIC nIRQ in DT (of_irq did not map GPIO 86)\n");
	}
	d1830_dump_irq_chain(client, "irq-on");

	{
		int v = i2c_smbus_read_byte_data(client, 7);

		dev_info(dev,
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
				dev_info(dev,
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

	dev_info(dev, "Registered %u read-only GPIOs using Dialog D1830 driver\n",
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

	/* Idle snapshot plus 100ms poll. nIRQ (GPIO 86) still calls
	 * d1830_n31_din_nirq; poll covers a missed EIC edge so Home /
	 * Sleep / Play show on n31-btn. Trace prints only on change.
	 */
	d1830_btn_poll_once(gpio_dev);
	INIT_DELAYED_WORK(&gpio_dev->trace, d1830_trace_work);
	schedule_delayed_work(&gpio_dev->trace, msecs_to_jiffies(100));
	d1830_n31_din_nirq_hook = d1830_n31_din_nirq;
	return 0;
}

static void d1830_gpio_remove(struct i2c_client *client)
{
	struct d1830_gpio *gpio_dev = i2c_get_clientdata(client);

	if (gpio_dev)
		cancel_delayed_work_sync(&gpio_dev->trace);
	device_remove_file(&client->dev, &dev_attr_vbat_raw);
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
