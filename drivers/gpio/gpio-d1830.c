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
#include <linux/debugfs.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
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
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include <linux/apple-n31.h>
#include <linux/rtc.h>

/*
 * Power-off and restart are the same PMIC write. The bootloader's
 * __noreturn sub_128C does
 *
 *   sub_3F60(110, 1, 0)        reg 110 = 0
 *   sub_3F40(64, 4, v5)        read reg 64, add 2, write reg 69
 *   sub_3F60(73, 1, 1)         reg 73 = 1     restart-after-cut
 *   sub_3F40(13, ...)          reg 13 |= 1    cut power
 *   sub_1130()                 halt
 *
 * while its low-battery path does the reg 13 write with no reg 73 at
 * all and stays off. So bit 0 of 13 always cuts power, and 73 decides
 * whether the PMIC comes back. Writing 13 without clearing 73 is a
 * reboot dressed as a shutdown, which is what this driver was doing.
 */
/*
 * Registers are written in hex here, deliberately.
 *
 * Hex-Rays prints them in decimal, so sub_7484's "case 16: v7 = 40"
 * means register 40 decimal, which is 0x28 -- not 0x40. Reading those
 * listings as hex is what produced a whole fictional rail map with a
 * codec enable at 0x40, and 0x40 turned out to be a ticking counter.
 * One radix mistake is a bad afternoon; leaving the door open for a
 * second is a design choice.
 *
 * The decimal literals this driver used were correct -- reg 16 really
 * is 0x10, the rail bitfield -- but they made checking against a
 * decompiler listing an exercise in mental arithmetic, which is
 * exactly when radix mistakes happen.
 */
/*
 * sub_1E7C, the boot low-battery decision, tests this after finding
 * VBAT below 3550 mV. If the bit is set RetailOS permits Low Power Boot
 * even below 3400 mV; if it is clear below 3400 mV it powers the device
 * off instead. That is precisely the shape of an external-source
 * indication.
 *
 * Confirmed on glass. Unplugging cleared the bit and replugging set
 * it, observed live:
 *
 *   3094 external power OFF -> gadget disconnect
 *   3169 external power ON  -> gadget connect
 *
 * so this is the VBUS presence indication rather than merely some
 * external source. VBUS_PRESENT is what it means; the EXT_POWER
 * spelling is kept as an alias so the firmware notes that introduced
 * it still read straight.
 *
 * Bit 2 is separate and also moved. 0x05 went 0x68 to 0x6c and 0x06
 * went 0x40 to 0x44 across the same cable cycle, so bit 2 latched in
 * both. 0x05 to 0x08 are the event latches this driver already reads
 * for buttons, masked at 0x09 to 0x0c, which makes bit 2 an
 * attach/detach event rather than a level.
 *
 * Deliberately not wired to anything. One observation cannot separate
 * attach from detach, and acknowledging a latch we have not
 * characterised risks consuming an event something else depends on.
 */
#define D1830_REG_STATUS0	0x05	/* 5 */
#define D1830_STATUS_VBUS_PRESENT	BIT(6)
#define D1830_STATUS_EXT_POWER		D1830_STATUS_VBUS_PRESENT
#define D1830_REG_STATUS1		0x06	/* 6 */
/* Latched in both 0x05 and 0x06 by a cable event; polarity unproven. */
#define D1830_STATUS_CABLE_EVENT	BIT(2)

#define D1830_REG_POWEROFF	0x0d	/* 13 */
#define D1830_POWEROFF_BIT	BIT(0)

/*
 * Read at entry to sub_1E7C, before the battery is even measured: if
 * set, RetailOS writes 1 back and goes straight to Low Power Boot. A
 * persistent latch, and specifically NOT the external-power test --
 * that decision happens later from 0x05 bit 6.
 */
#define D1830_REG_LOWPOWER_LATCH	0x0e	/* 14 */
#define D1830_LOWPOWER_LATCH		BIT(0)

/* Programmed by the stock Low Power Boot path. Meanings unresolved. */
#define D1830_REG_LOWPOWER_CFG0		0x24	/* 36 */
#define D1830_REG_LOWPOWER_CFG1		0x29	/* 41 */
#define D1830_LOWPOWER_CFG1_MASK	0x07
#define D1830_LOWPOWER_CFG1_RETAIL	0x06

/*
 * sub_1E7C thresholds, in millivolts against d1830_adc_to_mv():
 *   >= 3550        boot normally
 *   3400..3549     Low Power Boot
 *   <  3400        Low Power Boot if external power, else power off
 */
#define N31_VBAT_BOOT_NORMAL_MV		3550
#define N31_VBAT_BOOT_MIN_MV		3400
#define D1830_REG_RESTART	0x49	/* 73; 1 = come back after the cut */
#define D1830_REG_CLR_ON_CUT	0x6e	/* 110 */

/*
 * SoC fallback from the tail of sub_128C, reached only if the PMIC
 * write fails. 0xA5 is a magic value rather than a bit pattern.
 */
#define S5L8740_RESET_A_PHYS	0x3c800000UL
#define S5L8740_RESET_B_PHYS	0x3c500050UL
#define S5L8740_RESET_MAGIC	0xa5
#define D1830_REG_ADC_CFG	0x30	/* 48 */
#define D1830_REG_ADC_LOW	0x31	/* 49 */
#define D1830_REG_ADC_HIGH	0x32	/* 50 */
#define D1830_ADC_CH_VBAT	3	/* OSOS 439A98 case 1 */
#define D1830_ADC_START		0x10
#define D1830_ADC_SAMPLES	5
/*
 * VBAT conversion, from the RetailOS formula rather than a full-scale
 * guess:
 *
 *   mv = (62 * raw + 207000) / 100
 *
 * which reproduces its own reference points exactly -- raw 1823 gives
 * 3200 mV, 2877 gives 3853, 3435 gives 4199.
 *
 * Two things follow. The transfer function has a 2070 mV offset, so
 * treating the ADC as linear from zero was wrong; and its slope is
 * 0.62 mV per count against the 1.47 mV per count that 6000/1023
 * implied, so the old conversion multiplied every LSB of ADC noise by
 * more than twice as much as it should have. That is a large part of
 * why vbat looked so unstable.
 *
 * The formula is written for a 12-bit raw. This ADC path assembles ten
 * bits -- (r50 << 2) | (r49 & 3) -- so it is shifted up by two rather
 * than the constants being rescaled, which keeps the published numbers
 * checkable against the source they came from.
 */
/*
 * sub_4234, the boot battery helper, and this is the conversion to use.
 * Its raw assembly is (reg32 << 2) | reg31, which is exactly what this
 * driver already reads, and its constants are explicit in the ARM:
 *
 *   0x3FF = 1023, 0x7D0 = 2000, 0x9C4 = 2500
 *   mv = 2500 + raw * 2000 / 1023
 *
 * so 0 is 2500 mV and 1023 is 4500 mV.
 *
 * This replaces (62 * raw12 + 207000) / 100, which came from the later
 * twelve-bit averaging API. Both are real, but they are different
 * paths, and ours assembles ten bits the way the boot helper does. The
 * deciding argument is that the 3550 and 3400 thresholds below are
 * compared against this function's output, so any other scale makes
 * them mean something the firmware never intended.
 */
#define D1830_VBAT_BASE_MV	2500
#define D1830_VBAT_SPAN_MV	2000
#define D1830_VBAT_FULL_SCALE	1023

/* RetailOS sub_8005C314 warns and acts at 3.2 V. Reported, not acted on
 * here: a driver that powers the machine off the instant a noisy read
 * dips below a threshold is worse than one that reports it. */
#define D1830_VBAT_LOW_MV	3200
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

/*
 * Off by default, and it should stay that way: this replays the
 * board-wide rail trim, whose ACTIVE_1 form is (old & 0x2F) | 0x10.
 * That is correct at boot, but it clears bits 6 and 7, so running it
 * again once the panel is up switches the display rail off. Measured on
 * glass: ACTIVE_1 goes 0x7f -> 0x3f on the first codec prepare after
 * boot, and the screen turns white.
 *
 * This comment used to end "the codec's analog supply is always on, so
 * audio does not need this at all". That claim is unverified and should
 * not be relied on -- but the correction attempted here, powering the
 * codec from registers 20-23 bit 4, was also wrong and locked the
 * kernel. Those registers hold a bare 5-bit voltage code with no enable
 * bit; see the note above d1830_audio_rails(). Where the CS42L81 analog
 * supply actually comes from is still an open question, and neither
 * claim in this paragraph's history should be treated as settled.
 */
static bool allow_audio_rails;
module_param(allow_audio_rails, bool, 0644);
MODULE_PARM_DESC(allow_audio_rails,
		 "Replay the whole board rail trim on codec prepare (default N; touches the display rail)");


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
/*
 * On by default: without it the nIRQ latch is never released. Left as a
 * knob only so the old behaviour can be reproduced when comparing.
 */
static bool pmic_irq;
module_param(pmic_irq, bool, 0444);
MODULE_PARM_DESC(pmic_irq,
		 "Request the PMIC nIRQ line (default N; polling is used)");

static bool ack_events = true;
module_param(ack_events, bool, 0644);
MODULE_PARM_DESC(ack_events,
		 "Write back the PMIC event latches to release nIRQ");

/*
 * Polled by default. The PMIC nIRQ reaches the EIC, but the EIC's level
 * behaviour is not yet pinned down, so this is what makes Home, Play and
 * Sleep work today. 100 ms is well inside a keypress.
 */
/*
 * On, and load-bearing. Do not turn this off.
 *
 * It was switched off once on the strength of the comment above
 * d1830_trace_work() calling it "not a product poll", and Home, Play and
 * Sleep stopped working immediately. The reason is in /proc/interrupts:
 * there is no PMIC nIRQ line there at all. The EIC path this is nominally
 * a fallback for does not deliver, so the poll is not backup -- it is the
 * only thing turning a key press into an input event.
 *
 * The cost is real: ten I2C transactions a second for the life of the
 * system. The fix is to make the PMIC interrupt work and then retire this,
 * not to delete the mechanism that currently carries the buttons.
 */
static unsigned int btn_poll_ms = 100;
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
	/* Event-register reads serviced, for the input diagnostics. */
	unsigned int irq_events;
	bool lsb_logged;
	int mv_filtered;
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

/* ------------------------------------------------------------------ */
/* RTC                                                                  */
/*                                                                      */
/* The counter lives in the PMIC rather than in SoC MMIO. OSOS reads it */
/* a byte at a time, least significant first, in sub_16517E:            */
/*                                                                      */
/*   sub_41286E(a1, 124, v8);              byte 0                       */
/*   sub_41286E(a1, 125, (char *)v8 + 1);  byte 1                       */
/*   sub_41286E(a1, 126, (char *)v8 + 2);  byte 2                       */
/*   sub_41286E(a1, 127, (char *)v8 + 3);  byte 3                       */
/*                                                                      */
/* and the bootloader's sub_FDE zeroes the same four, which is what you */
/* would expect of a counter being reset rather than of scratch space.  */
/*                                                                      */
/* Honest limitation: OSOS passes the four bytes through sub_16CA5E     */
/* before using them, and that routine is a long bit-manipulation which */
/* has not been decoded. This driver therefore treats the registers as  */
/* a plain little-endian seconds counter. Reads and writes are          */
/* self-consistent, so timekeeping across a reboot works; what is not   */
/* guaranteed is that the epoch agrees with RetailOS. Set the clock     */
/* once with hwclock and it will keep.                                  */
/* ------------------------------------------------------------------ */

/*
 * Calendar registers, not the seconds counter this driver first used.
 *
 * 124 to 127 looked like a 32-bit counter because OSOS sub_16517E reads
 * exactly those four LSB-first, and writing them round-tripped perfectly.
 * They are MEMBYTE, 0x60 to 0x87, which is general purpose non-volatile
 * scratch -- so that was our own value being read back, and OSOS keeps a
 * timestamp there by convention rather than because it is the clock.
 *
 * The real block, confirmed against the part: R64 counts seconds and was
 * observed ticking 0x23 to 0x26 across three seconds.
 *
 *   0x40 64  COUNT_SEC    bits 5:0, bit 6 MONITOR
 *   0x41 65  COUNT_MIN    bits 5:0
 *   0x42 66  COUNT_HOUR   bits 4:0
 *   0x43 67  COUNT_DAY    bits 4:0, 1 based
 *   0x44 68  COUNT_MONTH  bits 3:0, 1 based
 *   0x45 69  COUNT_YEAR   bits 5:0, 0 is 2000
 *
 * Reading COUNT_SEC latches the rest, and writing COUNT_YEAR commits
 * them, so the order of access is part of the interface rather than a
 * convenience.
 *
 * MONITOR is the part telling us whether it kept time: 0 means power was
 * lost. On this unit it reads 0 with day and month at zero, which are not
 * legal values -- the clock has been free-running since the factory and
 * was never set. Reporting that as a date would be worse than refusing,
 * so an unset clock returns -EINVAL and userspace can decide.
 */
#define D1830_RTC_SEC		0x40
#define D1830_RTC_MIN		0x41
#define D1830_RTC_HOUR		0x42
#define D1830_RTC_DAY		0x43
#define D1830_RTC_MONTH		0x44
#define D1830_RTC_YEAR		0x45
#define D1830_RTC_MONITOR	BIT(6)
#define D1830_RTC_YEAR_BASE	100	/* tm_year for 2000 */

/*
 * This uses MEMBYTE 124 to 127 rather than the hardware calendar, and the
 * reason is worth writing down because the obvious reading of the
 * register map does not survive contact with the part.
 *
 * The map has calendar at 0x40 to 0x45 and a 32-bit upcount at 0x4c to
 * 0x4f, with MEMBYTE, general purpose non-volatile scratch, at 0x60 to
 * 0x87. So 124 to 127 is scratch, and OSOS sub_16517E reading exactly
 * those four LSB-first is OSOS keeping its own timestamp there by
 * convention rather than reading a clock.
 *
 * Measured on this unit:
 *
 *   R64 ticks -- observed 0x23 to 0x26 across three seconds -- and
 *   accepts writes to its seconds field and to bit 6.
 *   R69 accepts a year write: 0xf7 became 0x1a for 2026.
 *   R65 to R68 accept nothing. They kept a2 26 00 00 through single
 *   writes and through a six-register block write, and they do not
 *   advance on their own -- R65 held a2 across several minutes, so it is
 *   not running minutes either.
 *   The upcount at 0x4c did not advance across three seconds.
 *
 * Day and month therefore read zero, which is not a legal value, and
 * rtc_valid_tm rightly refuses it. A calendar whose middle four
 * registers cannot be set is not a clock this driver can offer, and
 * guessing at another address for them would be inventing hardware.
 *
 * MEMBYTE does work, end to end and verified: writing 1787941200 read
 * back as 2026-08-28 18:20:00, and the raw registers held 50 d1 91 6a,
 * which is that value little-endian. It is battery-backed so it survives
 * a reboot, and it is what OSOS itself uses.
 *
 * The limitation is real and not hidden: scratch does not tick, so time
 * does not advance while the system is off. Userspace should write the
 * clock on shutdown, which is what CONFIG_RTC_SYSTOHC does. Fixing this
 * properly needs the addresses for minutes through month, which the
 * evidence here does not supply.
 */
#define D1830_RTC_MEM_BASE	0x7c	/* 124, inside MEMBYTE 0x60-0x87 */
#define D1830_RTC_MEM_COUNT	4

static int d1830_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	u32 secs = 0;
	int i, v;

	for (i = 0; i < D1830_RTC_MEM_COUNT; i++) {
		v = i2c_smbus_read_byte_data(client, D1830_RTC_MEM_BASE + i);
		if (v < 0)
			return v;
		secs |= (u32)(v & 0xff) << (8 * i);
	}
	rtc_time64_to_tm((time64_t)secs, tm);
	return 0;
}

static int d1830_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	time64_t secs = rtc_tm_to_time64(tm);
	int i, ret;

	if (secs < 0 || secs > U32_MAX)
		return -EINVAL;

	for (i = 0; i < D1830_RTC_MEM_COUNT; i++) {
		ret = i2c_smbus_write_byte_data(client, D1830_RTC_MEM_BASE + i,
						(u8)(((u32)secs >> (8 * i)) & 0xff));
		if (ret)
			return ret;
	}
	return 0;
}

static const struct rtc_class_ops d1830_rtc_ops = {
	.read_time = d1830_rtc_read_time,
	.set_time = d1830_rtc_set_time,
};

static int d1830_rtc_register(struct i2c_client *client)
{
	struct rtc_device *rtc;

	rtc = devm_rtc_allocate_device(&client->dev);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	rtc->ops = &d1830_rtc_ops;
	rtc->range_min = 0;
	rtc->range_max = U32_MAX;
	/*
	 * Four PMIC registers and nothing else: there is no alarm here. Say
	 * so, or registration goes looking for one -- __rtc_read_alarm walks
	 * forward from the current time hunting a valid match, which with a
	 * counter that reads zero means a long search over i2c inside probe,
	 * and the driver sits in Loading while it happens.
	 */
	clear_bit(RTC_FEATURE_ALARM, rtc->features);

	return devm_rtc_register_device(rtc);
}

static void d1830_soc_reset_fallback(void)
{
	void __iomem *a, *b;

	a = ioremap(S5L8740_RESET_A_PHYS, 4);
	b = ioremap(S5L8740_RESET_B_PHYS, 4);
	if (a)
		writel(S5L8740_RESET_MAGIC, a);
	if (b)
		writel(S5L8740_RESET_MAGIC, b);
	/* No iounmap: this does not return if it works. */
}

static void d1830_cut_power(struct i2c_client *client, bool restart)
{
	int v, ret;
	u8 out;

	if (!client)
		return;

	dev_emerg(&client->dev, "PMIC %s: reg %u |= 0x%02lx\n",
		  restart ? "restart" : "poweroff",
		  D1830_REG_POWEROFF, D1830_POWEROFF_BIT);

	/*
	 * Order matters and this half is the whole difference between the
	 * two: 73 must be settled before 13 is written, because 13 is what
	 * actually removes power and nothing runs afterwards.
	 */
	i2c_smbus_write_byte_data(client, D1830_REG_CLR_ON_CUT, 0);
	i2c_smbus_write_byte_data(client, D1830_REG_RESTART, restart ? 1 : 0);

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

	/*
	 * Still here, so the PMIC did not take it. For a restart the SoC can
	 * still do the job; for a power-off there is no equivalent, and
	 * resetting instead would be worse than stopping -- with panic=-1 on
	 * the command line, returning from here reboots, which is the
	 * "shutdown rebooted the device" symptom rather than a fix for it.
	 */
	if (restart) {
		dev_emerg(&client->dev,
			  "PMIC restart did not take; SoC reset\n");
		d1830_soc_reset_fallback();
		mdelay(100);
	} else {
		dev_emerg(&client->dev,
			  "PMIC poweroff did not take; halting\n");
	}
	while (1)
		cpu_relax();
}

static void d1830_pm_power_off(void)
{
	d1830_cut_power(d1830_poweroff_client, false);
}

/*
 * pm_power_off is the legacy global and mainline is migrating off it.
 * The sys-off handler is the current interface, is properly scoped to
 * this device, and unregisters itself, so the global is only kept as a
 * fallback for the in-driver callers that still reference it.
 */
static int d1830_sys_off_handler(struct sys_off_data *data)
{
	d1830_cut_power(d1830_poweroff_client, false);
	return NOTIFY_DONE;
}

/*
 * Without this, reboot fell through to whatever the architecture could
 * manage on its own, which on this SoC is nothing reliable. Priority is
 * high because the PMIC is the only thing here that genuinely restarts
 * the machine.
 */
static int d1830_restart_handler(struct notifier_block *nb,
				 unsigned long mode, void *cmd)
{
	d1830_cut_power(d1830_poweroff_client, true);
	return NOTIFY_DONE;
}

static struct notifier_block d1830_restart_nb = {
	.notifier_call = d1830_restart_handler,
	.priority = 192,
};

/*
 * Register window, for identifying this part against the DA9053
 * datasheet -- the closest public Dialog device. The layouts are
 * similar but not known to be identical, and several things worth
 * having (the RTC monitor bit, the alarm block, the charger) are only
 * usable once the offset between the two is established rather than
 * assumed.
 *
 *   echo 108 24 > regs   dump 24 registers from 108
 *   cat regs
 */
static unsigned int d1830_regs_first = 108, d1830_regs_count = 24;

static ssize_t regs_show(struct device *dev, struct device_attribute *a,
			 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int i;
	int len = 0, v;

	for (i = 0; i < d1830_regs_count && len < PAGE_SIZE - 24; i++) {
		v = i2c_smbus_read_byte_data(client, d1830_regs_first + i);
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "R%-3u = %02x\n",
				 d1830_regs_first + i, v < 0 ? 0 : v);
	}
	return len;
}

static ssize_t regs_store(struct device *dev, struct device_attribute *a,
			  const char *buf, size_t count)
{
	unsigned int first, n;

	if (sscanf(buf, "%u %u", &first, &n) != 2)
		return -EINVAL;
	if (first > 255 || n == 0 || n > 64 || first + n > 256)
		return -EINVAL;
	d1830_regs_first = first;
	d1830_regs_count = n;
	return count;
}
static DEVICE_ATTR_RW(regs);

static ssize_t do_poweroff_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;
	d1830_cut_power(client, false);
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

/* Screen-sleep and power-button policy; defined with the PMU rail code. */
static void n31_power_button(bool pressed);
static void n31_power_button_poll(void);
static void n31_home_button(bool pressed);

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
	if (code == KEY_HOMEPAGE)
		n31_home_button(pressed);
	if (gpio_dev->input) {
		input_report_key(gpio_dev->input, code, pressed);
		input_sync(gpio_dev->input);
	}
}

/*
 * Registers 5-8 are the event latches and 9-12 their masks, the usual
 * layout for this PMIC family -- d1830_osos_nirq_mask() programs the
 * masks. Event latches are write-1-to-clear, and nothing here was
 * clearing them.
 *
 * That is fatal rather than untidy. nIRQ is requested level-low and
 * one-shot, so an uncleared latch holds the line asserted: the handler
 * returns, the line is still low, the interrupt fires again, and genirq
 * eventually disables it as spurious. Buttons work once and then stop
 * for the rest of the boot, which is exactly the reported symptom.
 *
 * Only bits that actually read as set are written back, so this cannot
 * disturb a latch that armed between the read and the acknowledgement.
 */
/*
 * Everything needed to tell a dead nIRQ from a dead button, without
 * having to reflash to add a printk: the live event and mask registers,
 * how many interrupts genirq has actually delivered, and whether it has
 * given up on the line.
 */
static ssize_t buttons_show(struct device *dev, struct device_attribute *a,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct d1830_gpio *g = i2c_get_clientdata(client);
	int r5, r6, r7, r8, r9, r10, r11, r12;

	if (!g)
		return -ENODEV;
	r5 = i2c_smbus_read_byte_data(client, 5);
	r6 = i2c_smbus_read_byte_data(client, 6);
	r7 = i2c_smbus_read_byte_data(client, 7);
	r8 = i2c_smbus_read_byte_data(client, 8);
	r9 = i2c_smbus_read_byte_data(client, 9);
	r10 = i2c_smbus_read_byte_data(client, 10);
	r11 = i2c_smbus_read_byte_data(client, 11);
	r12 = i2c_smbus_read_byte_data(client, 12);

	return sysfs_emit(buf,
		"events  r5=%02x r6=%02x r7=%02x r8=%02x\n"
		"masks   r9=%02x r10=%02x r11=%02x r12=%02x\n"
		"home=%d sleep=%d play=%d  (0 = pressed)\n"
		"irq=%d serviced=%u ack_events=%d btn_poll_ms=%u\n"
		"input=%s\n",
		r5 & 0xff, r6 & 0xff, r7 & 0xff, r8 & 0xff,
		r9 & 0xff, r10 & 0xff, r11 & 0xff, r12 & 0xff,
		r7 < 0 ? -1 : !!(r7 & BIT(4)),
		r7 < 0 ? -1 : !!(r7 & BIT(5)),
		r8 < 0 ? -1 : !!(r8 & BIT(1)),
		client->irq, g->irq_events, ack_events, btn_poll_ms,
		g->input ? "registered" : "absent");
}
static DEVICE_ATTR_RO(buttons);

static void d1830_ack_events(struct i2c_client *client,
			     int r5, int r6, int r7, int r8)
{
	static const u8 regs[] = { 5, 6, 7, 8 };
	int vals[4] = { r5, r6, r7, r8 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		if (vals[i] <= 0)
			continue;
		if (i2c_smbus_write_byte_data(client, regs[i], (u8)vals[i]))
			dev_warn_ratelimited(&client->dev,
					     "event ack r%u=0x%02x failed\n",
					     regs[i], vals[i]);
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

	if (ack_events)
		d1830_ack_events(client, r5, r6, r7, r8);
	gpio_dev->irq_events++;

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
			n31_power_button(true);
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
		n31_power_button_poll();
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
					d1830_cut_power(client, false);
			}
		}
	} else {
		if (!gpio_dev->last_sleep) {
			dev_dbg(&client->dev,
				"n31-btn SLEEP release r7=0x%02x\n", r7);
			n31_power_button(false);
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
static int d1830_adc_once_ch(struct d1830_gpio *gpio_dev, u8 channel,
			     int *adc, u8 *r48, u8 *r49, u8 *r50)
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
					(cfg & 0xF0) | (channel & 0x0f) |
					D1830_ADC_START);
	if (cfg)
		return cfg;

	/*
	 * Wait for START to clear. It is the conversion-in-progress bit --
	 * the busy check at the top of this function already treats it that
	 * way -- but this loop used to break while it was still set, so the
	 * data registers were read mid-conversion every time. Averaging five
	 * torn samples still gives a torn answer, which is why vbat swung
	 * 3325 to 4058 mV inside twenty seconds and capacity wandered
	 * between 2 and 24 percent. Nothing downstream of this could be
	 * trusted, including the charging state.
	 */
	for (i = 0; i < 10; i++) {
		usleep_range(1000, 1500);
		cfg = i2c_smbus_read_byte_data(client, D1830_REG_ADC_CFG);
		if (cfg < 0)
			return cfg;
		if (!(cfg & D1830_ADC_START))
			break;
	}
	if (cfg & D1830_ADC_START)
		return -ETIMEDOUT;

	hi = i2c_smbus_read_byte_data(client, D1830_REG_ADC_HIGH);
	lo = i2c_smbus_read_byte_data(client, D1830_REG_ADC_LOW);
	if (hi < 0)
		return hi;
	if (lo < 0)
		return lo;
	*r48 = (u8)cfg;
	*r49 = (u8)lo;
	*r50 = (u8)hi;
	/*
	 * 347E4: (reg50 << 2) | reg49, where only the bottom two bits of
	 * reg49 are sample data. They were being OR'd in unmasked, so
	 * anything above bit 1 landed on top of bits belonging to reg50 --
	 * reg49 was observed at 0x0e, which is about 60 LSBs of corruption,
	 * or roughly 300 mV across a 10-bit range. That is most of the swing
	 * that made vbat and capacity unusable.
	 */
	*adc = (((hi & 0xff) << 2) | (lo & 0x3)) & 0x3ff;
	return 0;
}

static int d1830_adc_to_mv(int adc)
{
	return D1830_VBAT_BASE_MV +
	       (adc * D1830_VBAT_SPAN_MV) / D1830_VBAT_FULL_SCALE;
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
		ret = d1830_adc_once_ch(gpio_dev, D1830_ADC_CH_VBAT, &adc,
					&r48, &r49, &r50);
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

	/*
	 * Smooth across calls, not just within one.
	 *
	 * The conversions are honest -- r48 shows START clear before each
	 * read and r49 never exceeds 2 -- but the raw high byte still ranges
	 * over a7 to d3 between samples, which is real movement on VBAT
	 * under a switching load rather than a decoding fault. The five
	 * samples above are taken back to back in a few milliseconds, so
	 * they all land inside the same transient and averaging them
	 * smooths nothing. That is what made vbat swing 700 mV and capacity
	 * wander between 2 and 100 percent, which in turn made the charging
	 * state meaningless.
	 *
	 * An exponential average over successive reads spans many
	 * transients instead of one. Weight is 1/4 new, giving a time
	 * constant of a few seconds at the half-second cache interval --
	 * slow enough to be steady, quick enough to follow a real charge.
	 * The first reading seeds it rather than ramping up from zero.
	 */
	if (gpio_dev->mv_filtered)
		gpio_dev->mv_filtered = (gpio_dev->mv_filtered * 3 + *mv) / 4;
	else
		gpio_dev->mv_filtered = *mv;
	*mv = gpio_dev->mv_filtered;
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

/*
 * A voltage-curve estimate, not a fuel gauge. The hardware measures
 * VBAT and nothing else -- no coulomb counter, no charge current, no
 * temperature -- so this is an interpolation between two constants and
 * should be read as one. It is exposed because a UI that shows nothing
 * is worse than one showing an approximation, not because the number
 * is measured.
 */
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
		/*
		 * UNKNOWN is the honest answer. Charge state needs a VBUS or
		 * charger status bit and neither is identified: TriStar's own
		 * VBUS task exists in OSOS but its register is not mapped, and
		 * apple_tristar_vbus() is still a stub.
		 *
		 * Guessing from voltage alone was actively misleading. It
		 * reported Full above 4150 mV, so a cell sitting high after a
		 * charge read as Full while unplugged, and ADC noise flipped it
		 * between Full and Discharging inside a few seconds. A field
		 * that changes meaning with noise is worse than one that admits
		 * it does not know.
		 */
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
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

/* ------------------------------------------------------------------ */
/* Charger scaffold                                                     */
/*                                                                      */
/* Deliberately inert. Every property here answers -ENODATA, and no      */
/* register is written, because not one charger register on this part is */
/* proven: not VBUS presence, not enable, not input current limit, not   */
/* charge current or voltage, not termination, not fault, not battery    */
/* presence.                                                            */
/*                                                                      */
/* What exists is the shape. When the registers are found, filling in    */
/* d1830_charger_regs below and the switch bodies is the whole job --    */
/* the class device, the property list and the plumbing are already      */
/* here and already exercised.                                          */
/*                                                                      */
/* The reason this stops at the shape is asymmetry of consequence. A     */
/* wrong GPIO bit gives silence and you try the next one. A wrong        */
/* charger register pushes current into a soldered lithium cell with no  */
/* replaceable path, and the register map for this part has already been */
/* revised three times -- a rail block at 0x40 that turned out to be a   */
/* counter, a calendar whose middle registers refuse writes, and a       */
/* seconds store that turned out to be scratch. A default-off flag       */
/* guards against accidents, not against the values being wrong.        */
/*                                                                      */
/* Enabling charger_enable does not make it write anything. It only      */
/* registers the class device so a consumer can be developed against it. */
/* ------------------------------------------------------------------ */

/*
 * Read-only telemetry for charger reverse engineering.
 *
 * Nothing here writes a charger register, because none is proven. What
 * it does is make the part observable, which is the step that has to come
 * first: the way to find the VBUS bit is to watch which bit moves when
 * the cable state changes, not to guess an address.
 *
 *   adc_channels   sweep every ADC channel, not just VBAT
 *   charger_regs   labelled dump of the candidate window
 *   charger_watch  write 1 to snapshot, read to see what has changed
 *
 * Only channel 3 has a trustworthy interpretation today -- it is VBAT,
 * via sub_8005C2C4 to sub_8005C618(3, ...). The others are read and
 * reported as raw counts with no label, because plausible occupants
 * include VBUS voltage, charge current and a thermistor, and writing any
 * of those names into the output before correlating it would make a
 * guess look like a measurement.
 *
 * The register window is bounded rather than a blind 0x00..0xff sweep.
 * Some PMIC registers are clear-on-read or otherwise side-effecting --
 * the event latches in this same address space are write-one-to-clear --
 * so reading everything to see what happens is not free.
 */
#define D1830_ADC_CH_MAX	8

static ssize_t adc_channels_show(struct device *dev,
				 struct device_attribute *a, char *buf)
{
	struct d1830_gpio *gpio_dev = dev_get_drvdata(dev);
	unsigned int ch;
	int len = 0;

	if (!gpio_dev)
		return -ENODEV;

	for (ch = 0; ch < D1830_ADC_CH_MAX; ch++) {
		u8 r48 = 0, r49 = 0, r50 = 0;
		int adc = 0, ret;

		ret = d1830_adc_once_ch(gpio_dev, (u8)ch, &adc,
					&r48, &r49, &r50);
		if (ret) {
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "ch%u err=%d\n", ch, ret);
			continue;
		}
		/* mV is only meaningful for channel 3; the rest are counts. */
		if (ch == D1830_ADC_CH_VBAT)
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "ch%u raw=%4d r48=%02x r49=%02x r50=%02x mv=%d (VBAT)\n",
					 ch, adc, r48, r49, r50,
					 d1830_adc_to_mv(adc));
		else
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "ch%u raw=%4d r48=%02x r49=%02x r50=%02x (unidentified)\n",
					 ch, adc, r48, r49, r50);
	}
	return len;
}
static DEVICE_ATTR_RO(adc_channels);

/*
 * The window worth watching. Deliberately excludes the event latches at
 * 0x05..0x08, which are write-one-to-clear and whose read semantics are
 * not established, and the hibernate block, which RetailOS only touches
 * during power-state transitions.
 */
static const struct {
	u8 first, count;
	const char *what;
} d1830_charger_window[] = {
	/* Non-overlapping: a register appearing twice would be counted
	 * twice in the snapshot and reported twice in a diff. */
	/*
	 * The event latches. Previously left out because their read
	 * semantics were unestablished, but the buttons path reads them
	 * continuously without harm, and the cable event turned up here --
	 * bit 2 latched in both 0x05 and 0x06 -- so excluding them meant
	 * excluding the interesting part. Read only; nothing acknowledges.
	 */
	{ 0x05, 4, "event latches (0x05 bit6 = VBUS present, bit2 = cable event)" },
	{ 0x09, 4, "event masks" },
	{ 0x0d, 5, "0x0d poweroff, 0x10-0x11 rail enables (0x10 bit5 = Nimbus, proven)" },
	{ 0x14, 8, "bootloader LDO trims; 0x1a takes 0xb2, charge-adjacent" },
	{ 0x23, 2, "touched by the bootloader trim sequence" },
	{ 0x30, 4, "ADC config and result" },
	{ 0x57, 2, "BT companion rails (proven via sub_51688C)" },
	/*
	 * The regions RetailOS only touches during power-state changes.
	 * Live and stable -- two consecutive reads returned identical
	 * values, so nothing here is clear-on-read -- and unmapped.
	 *
	 * Worth watching rather than naming. 0xc9..0xcc reads ff 0d ff 0d,
	 * which is 0x0dff twice as little-endian 16-bit, or 3583. That
	 * sits beside RetailOS's own 3550 and 3400 millivolt thresholds,
	 * and a duplicated voltage constant in a power block is the shape
	 * of a charge or recharge threshold. One reading is not a decode,
	 * so it goes in the capture window and gets a name only if it
	 * moves when the charging state does.
	 */
	{ 0xa4, 1, "hibernate seq, RetailOS masks 0x3f" },
	{ 0xaf, 9, "hibernate config 0xaf-0xb7, unmapped" },
	{ 0xc0, 19, "hibernate config 0xc0-0xd2; 0xc9-0xcc looks like 3583 twice" },
};

static ssize_t charger_regs_show(struct device *dev,
				 struct device_attribute *a, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int i, j;
	int len = 0, v;

	for (i = 0; i < ARRAY_SIZE(d1830_charger_window) &&
	     len < PAGE_SIZE - 96; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "# %s\n",
				 d1830_charger_window[i].what);
		for (j = 0; j < d1830_charger_window[i].count &&
		     len < PAGE_SIZE - 32; j++) {
			u8 reg = d1830_charger_window[i].first + j;

			v = i2c_smbus_read_byte_data(client, reg);
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "0x%02x = %02x\n",
					 reg, v < 0 ? 0 : v);
		}
	}
	return len;
}
static DEVICE_ATTR_RO(charger_regs);

/*
 * Snapshot and diff. Write 1 to record the window, read to see only what
 * moved since. This is the tool that actually finds a status bit: take a
 * snapshot, change something in the world, read back, and the bits that
 * changed are the short list.
 */
static u8 d1830_snap[64];
static bool d1830_snap_valid;

static unsigned int d1830_snap_fill(struct i2c_client *client, u8 *out)
{
	unsigned int i, j, n = 0;

	for (i = 0; i < ARRAY_SIZE(d1830_charger_window); i++)
		for (j = 0; j < d1830_charger_window[i].count &&
		     n < sizeof(d1830_snap); j++) {
			int v = i2c_smbus_read_byte_data(client,
				d1830_charger_window[i].first + j);

			out[n++] = (v < 0) ? 0 : (u8)v;
		}
	return n;
}

static ssize_t charger_watch_show(struct device *dev,
				  struct device_attribute *a, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 now[sizeof(d1830_snap)];
	unsigned int i, j, n = 0, changed = 0;
	int len = 0;

	if (!d1830_snap_valid)
		return sysfs_emit(buf,
				  "no snapshot; echo 1 > charger_watch first\n");

	d1830_snap_fill(client, now);
	for (i = 0; i < ARRAY_SIZE(d1830_charger_window); i++)
		for (j = 0; j < d1830_charger_window[i].count &&
		     n < sizeof(d1830_snap); j++, n++) {
			u8 reg = d1830_charger_window[i].first + j;

			if (now[n] == d1830_snap[n])
				continue;
			changed++;
			if (len < PAGE_SIZE - 64)
				len += scnprintf(buf + len, PAGE_SIZE - len,
					"0x%02x %02x -> %02x  (xor %02x)  %s\n",
					reg, d1830_snap[n], now[n],
					d1830_snap[n] ^ now[n],
					d1830_charger_window[i].what);
		}
	if (!changed)
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "no change\n");
	return len;
}

static ssize_t charger_watch_store(struct device *dev,
				   struct device_attribute *a,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (buf[0] != '1')
		return -EINVAL;
	d1830_snap_fill(client, d1830_snap);
	d1830_snap_valid = true;
	return count;
}
static DEVICE_ATTR_RW(charger_watch);

/* ------------------------------------------------------------------ */
/* Charger state and RetailOS power init                                */
/* ------------------------------------------------------------------ */

enum d1830_input_type {
	D1830_INPUT_NONE,
	D1830_INPUT_USB,
	D1830_INPUT_ACCESSORY,
	D1830_INPUT_UNKNOWN_VBUS,
};

struct d1830_charger_state {
	bool vbus_known;	/* false until a VBUS source is identified */
	bool vbus_present;
	bool charging_enabled;
	bool charging;
	bool full;
	int vbat_mv;
	enum d1830_input_type input;
};

static struct d1830_charger_state d1830_chg;

/*
 * VBUS presence.
 *
 * Returns -ENODATA rather than a value, and that is the honest answer
 * today: no D1830 status bit and no TriStar register has been identified
 * as the VBUS indication. The existing USB supply reported ONLINE=1
 * unconditionally, which was a bring-up shortcut, and the note is right
 * that it must not become the charging policy -- a charger that believes
 * the cable is always attached will happily conclude it is charging while
 * running the battery flat.
 *
 * When the bit is found, this is the only function that needs to change.
 * Everything downstream already handles the unknown case explicitly
 * rather than defaulting to a convenient answer.
 */
/*
 * External power present, from sub_1E7C. Reads 0x05 bit 6.
 *
 * This is the one function the whole charger design was waiting on, and
 * everything downstream was already written to consume it. What it
 * reports is "a usable external source is available" -- the firmware
 * proves that much and no more, so consumers must not upgrade it to
 * "charging" on their own.
 */
static int d1830_vbus_present(struct i2c_client *client, bool *present)
{
	int v;

	if (!client)
		return -ENODEV;
	v = i2c_smbus_read_byte_data(client, D1830_REG_STATUS0);
	if (v < 0)
		return v;
	*present = !!(v & D1830_STATUS_EXT_POWER);
	return 0;
}

/*
 * sub_23EC, the RetailOS PMIC power initialisation, in its original order:
 *
 *   program the board trims at 0x14..0x17
 *   write 0x1a = 0xb2, twice
 *   0x10: clear 0xd0, set 0x10, and set 0x20 on a cold boot
 *   0x11: set 0x07
 *   0x13: set 0x02
 *
 * Off by default, and the reason is specific rather than caution in
 * general. On this device 0x10 currently reads 0x7f, and the RetailOS
 * masking takes that to 0x3f -- which clears bit 6. Bit 6 is a live rail
 * in our own table. RetailOS runs this at cold boot, before anything
 * depends on those rails; running it from a module probe, with the
 * display already up, would drop a rail out from under a running system.
 *
 * The 0x14..0x17 values are computed by RetailOS from board tables that
 * have not been recovered, so this reproduces the write ordering and the
 * masking, and deliberately leaves those four registers alone rather than
 * inventing trim values. That is why it is called an init reproduction
 * and not an equivalent.
 *
 * 0x1a already reads 0xb2 on this unit -- the bootloader wrote it -- so
 * that part is a confirmation rather than a change.
 */
/*
 * Off by default, and this one is a correction rather than caution.
 *
 * This write was enabled at probe while 0x1a was believed to be
 * charger configuration. It is not: the OSOS rail setter writes it as
 * (code & 0x1f) | 0xa0 with code = (mv - 1200) / 100, so 0xb2 is a
 * 3.0 V rail. Writing a live rail register during probe on the
 * strength of a label that turned out to be wrong is not something
 * to leave on by default, and two boots failed to come up with it
 * enabled.
 *
 * The value is what the bootloader already leaves there, so this is
 * expected to be a no-op -- but expected is not the same as
 * observed, and the cost of being wrong is a device that needs
 * recovering by hand.
 */
static bool ldo_1a_write;
module_param(ldo_1a_write, bool, 0444);
MODULE_PARM_DESC(ldo_1a_write,
		 "Re-write the 0x1a rail voltage at probe (default N)");

static bool charger_hw_init;
module_param(charger_hw_init, bool, 0444);
MODULE_PARM_DESC(charger_hw_init,
		 "Replay the sub_23EC PMIC init (clears 0x10 bit 6; boot only)");

/*
 * 0x1a is a rail voltage register, not charger configuration.
 *
 * The OSOS rail setter writes it as
 *
 *   code = (millivolts - 1200) / 100
 *   reg  = (code & 0x1f) | 0xa0
 *
 * and 0xb2 decodes exactly: 0xb2 & 0xe0 is 0xa0, matching the mask,
 * and 0xb2 & 0x1f is 18, giving 1200 + 1800 = 3000 mV. The same
 * function covers 0x14 through 0x22 with per-register bases and steps,
 * which is also what the bootloader is doing when it programs
 * 0x14..0x17 from board tables -- those are voltages.
 *
 * So writing 0xb2 here sets a 3.0 V rail. It is harmless because that
 * is the value already present, but calling it CHG_CFG was wrong and
 * would have sent the next person looking for a charger in the rail
 * block.
 */
#define D1830_REG_LDO_1A	0x1a	/* 26 */
#define D1830_LDO_1A_STOCK	0xb2	/* 3000 mV: 0xa0 | ((3000-1200)/100) */
#define D1830_REG_ACTIVE_1	0x10	/* 16 */
#define D1830_REG_ACTIVE_2	0x11	/* 17 */
#define D1830_REG_CTRL_13	0x13	/* 19 */

/*
 * The charger-configuration half of sub_23EC, on its own: 0x1a takes
 * 0xb2, written twice, exactly as both the bootloader and RetailOS do.
 * The second write is not a different value -- the firmware reuses the
 * same stack byte -- so this reproduces it rather than tidying it away.
 *
 * Safe to run, and enabled by default: this device already reads 0xb2
 * at 0x1a because the bootloader put it there, so this confirms the
 * state rather than changing it, and it makes the driver correct on a
 * path where the bootloader did not.
 *
 * What the individual bits of 0xb2 mean is not decoded. There is no
 * bitwise read-modify-write of this register anywhere in the extracted
 * firmware to give any bit an independent meaning, so the whole byte is
 * written as a unit and no D1830_CHARGE_ENABLE is invented from it.
 */
static int d1830_charger_config(struct i2c_client *client)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, D1830_REG_LDO_1A,
					D1830_LDO_1A_STOCK);
	if (ret)
		return ret;
	return i2c_smbus_write_byte_data(client, D1830_REG_LDO_1A,
					 D1830_LDO_1A_STOCK);
}

static int d1830_charger_hw_init(struct i2c_client *client, bool cold_boot)
{
	int v, ret;

	/* 0x14..0x17 are deliberately not written: the trim values come from
	 * board tables we have not recovered, and a wrong trim is worse than
	 * whatever the bootloader already left there. */

	ret = d1830_charger_config(client);
	if (ret)
		return ret;

	v = i2c_smbus_read_byte_data(client, D1830_REG_ACTIVE_1);
	if (v < 0)
		return v;
	v &= ~0xd0;
	v |= 0x10;
	if (cold_boot)
		v |= 0x20;
	ret = i2c_smbus_write_byte_data(client, D1830_REG_ACTIVE_1, (u8)v);
	if (ret)
		return ret;

	v = i2c_smbus_read_byte_data(client, D1830_REG_ACTIVE_2);
	if (v < 0)
		return v;
	ret = i2c_smbus_write_byte_data(client, D1830_REG_ACTIVE_2,
					(u8)(v | 0x07));
	if (ret)
		return ret;

	v = i2c_smbus_read_byte_data(client, D1830_REG_CTRL_13);
	if (v < 0)
		return v;
	return i2c_smbus_write_byte_data(client, D1830_REG_CTRL_13,
					 (u8)(v | 0x02));
}

/*
 * Deliberately absent: d1830_charger_set_enabled().
 *
 * 0x1a = 0xb2 is proven as configuration written during init, twice, by
 * both the bootloader and RetailOS. It is not proven to be the bit that
 * starts and stops charging at runtime, and nothing observed so far shows
 * RetailOS toggling it dynamically. Writing a guessed enable would be the
 * one mistake in this driver that damages hardware rather than annoying
 * the user, so the dynamic enable stays unimplemented until the bit is
 * identified.
 */

/*
 * The policy, written now so that identifying VBUS is the only remaining
 * step rather than the start of a design. Note what it does not do: it
 * never keys off USB enumeration. VBUS present, host detected and gadget
 * configured are three different things -- a wall charger gives VBUS with
 * no enumeration at all -- so gating charge on enumeration would refuse to
 * charge from exactly the source most likely to be attached.
 */
static void d1830_charger_update(struct i2c_client *client)
{
	struct d1830_gpio *gpio_dev = i2c_get_clientdata(client);
	bool present = false;
	int mv = 0;

	if (d1830_vbus_present(client, &present)) {
		d1830_chg.vbus_known = false;
		d1830_chg.input = D1830_INPUT_NONE;
		d1830_chg.charging = false;
		d1830_chg.full = false;
	} else {
		d1830_chg.vbus_known = true;
		d1830_chg.vbus_present = present;
		/*
		 * UNKNOWN_VBUS, not USB. 0x05 bit 6 says an external source
		 * is available; it does not say what kind. Classifying the
		 * attachment is TriStar's job and is not wired up yet.
		 */
		d1830_chg.input = present ? D1830_INPUT_UNKNOWN_VBUS :
					    D1830_INPUT_NONE;
		/*
		 * Charging stays false even with external power. Present and
		 * delivering current are different claims, and no bit
		 * separating input-present from charger-active, CV phase or
		 * complete has been isolated. Reporting CHARGING here would be
		 * the same category of error as the old voltage heuristic.
		 */
		d1830_chg.charging = false;
		d1830_chg.full = false;
	}

	if (gpio_dev && !d1830_read_vbat(gpio_dev, &mv))
		d1830_chg.vbat_mv = mv;
}

static bool charger_enable;
module_param(charger_enable, bool, 0444);
MODULE_PARM_DESC(charger_enable,
		 "Register the charger class device (properties still unproven)");

/*
* Fill these in as they are established, one line per proven register,
* with the evidence in the comment. An entry that is still zero means
* nobody has proven it, and the property stays -ENODATA.
*/
struct d1830_charger_reg {
	const char *what;
	u8 reg;		/* 0 = not identified */
	u8 mask;
};

static const struct d1830_charger_reg d1830_charger_regs[] = {
	{ "vbus_present",	0, 0 },
	{ "charge_enable",	0, 0 },
	{ "input_current_limit",	0, 0 },
	{ "charge_current",	0, 0 },
	{ "charge_voltage",	0, 0 },
	{ "term_current",	0, 0 },
	{ "charge_status",	0, 0 },
	{ "fault",		0, 0 },
};

static int d1830_charger_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		if (!d1830_chg.vbus_known)
			return -ENODATA;
		val->intval = d1830_chg.vbus_present ? 1 : 0;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		if (!d1830_chg.vbus_known)
			return -ENODATA;
		/*
		 * NOT_CHARGING rather than CHARGING when external power is
		 * present. charging and full are only ever set from a proven
		 * status bit, and none is identified, so this reports the one
		 * thing that is known: a source is or is not attached.
		 */
		if (!d1830_chg.vbus_present)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (d1830_chg.full)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (d1830_chg.charging)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		/* Register not identified. Saying so beats a number. */
		return -ENODATA;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property d1830_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_SCOPE,
};

static const struct power_supply_desc d1830_charger_desc = {
	.name = "d1830-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = d1830_charger_props,
	.num_properties = ARRAY_SIZE(d1830_charger_props),
	.get_property = d1830_charger_get_property,
};

static void d1830_charger_register(struct device *dev)
{
	struct power_supply_config cfg = { };
	struct power_supply *psy;
	unsigned int i, known = 0;

	if (!charger_enable)
		return;

	for (i = 0; i < ARRAY_SIZE(d1830_charger_regs); i++)
		if (d1830_charger_regs[i].reg)
			known++;

	psy = devm_power_supply_register(dev, &d1830_charger_desc, &cfg);
	if (IS_ERR(psy)) {
		dev_warn(dev, "charger scaffold not registered: %ld\n",
			 PTR_ERR(psy));
		return;
	}
	dev_info(dev,
		 "charger scaffold registered: %u/%zu registers proven, all properties -ENODATA\n",
		 known, ARRAY_SIZE(d1830_charger_regs));
}

static int d1830_usb_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		/*
		 * Was hardcoded to 1. That was a bring-up shortcut and it
		 * cannot be the charging policy: a supply that claims to be
		 * online whether or not a cable is attached will report
		 * charging while the battery drains. No VBUS source is
		 * identified, so -ENODATA is the true answer until
		 * d1830_vbus_present() can give one.
		 */
		if (!d1830_chg.vbus_known)
			return -ENODATA;
		val->intval = d1830_chg.vbus_present ? 1 : 0;
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

/* ------------------------------------------------------------------ */
/* N31 PMU register / rail decode                                       */
/*                                                                      */
/* Read-only by default. The point is to make the boot rail model       */
/* visible so audio and display power can be debugged from evidence     */
/* rather than by poking registers and watching what breaks. Names are  */
/* driver-local: they describe what the boot sequence does with each    */
/* register, and carry no claim about which peripheral owns a rail      */
/* until a snapshot diff proves it.                                     */
/* ------------------------------------------------------------------ */

#define N31_PMU_VARIANT		"n31_d1830_pmu_v1"
#define N31_PMU_REG_MAX		0x88	/* dump 0x00..0x87 */
#define N31_PMU_VSEL_MASK	0x1f
#define N31_PMU_NO_VSEL		0xff

static char *pmu_variant = N31_PMU_VARIANT;
module_param(pmu_variant, charp, 0444);
MODULE_PARM_DESC(pmu_variant, "PMU register-map variant this driver decodes");

static bool allow_pmu_writes;
module_param(allow_pmu_writes, bool, 0644);
MODULE_PARM_DESC(allow_pmu_writes,
		 "Permit PMU rail writes at all (default N — decode only)");

static bool apply_boot_rails;
module_param(apply_boot_rails, bool, 0644);
MODULE_PARM_DESC(apply_boot_rails,
		 "Replay the boot rail sequence exactly (needs allow_pmu_writes)");

static bool audio_rail_test;
module_param(audio_rail_test, bool, 0644);
MODULE_PARM_DESC(audio_rail_test,
		 "Arm the analog rail experiment (needs allow_pmu_writes)");

static bool restore_after_test = true;
module_param(restore_after_test, bool, 0644);
MODULE_PARM_DESC(restore_after_test,
		 "Restore the pre-test register values afterwards (default Y)");

struct n31_pmu_regname {
	u8 reg;
	const char *name;
};

static const struct n31_pmu_regname n31_pmu_names[] = {
	{ 0x00, "PMU_CHIP_ID" },
	{ 0x01, "PMU_EVENT_A" },
	{ 0x02, "PMU_EVENT_B" },
	{ 0x03, "PMU_EVENT_C" },
	{ 0x04, "PMU_EVENT_D" },
	{ 0x05, "PMU_STATUS_A" },
	{ 0x06, "PMU_STATUS_B" },
	{ 0x07, "PMU_STATUS_C" },
	{ 0x08, "PMU_STATUS_D" },
	{ 0x09, "PMU_IRQ_MASK_A" },
	{ 0x0a, "PMU_IRQ_MASK_B" },
	{ 0x0b, "PMU_IRQ_MASK_C" },
	{ 0x0c, "PMU_IRQ_MASK_D" },
	{ 0x0d, "PMU_SYS_CONTROL" },
	{ 0x0e, "PMU_FAULT_LOG" },
	{ 0x10, "PMU_ACTIVE_1" },
	{ 0x11, "PMU_ACTIVE_2" },
	{ 0x12, "PMU_STANDBY_1" },
	{ 0x13, "PMU_HIBERNATE_1" },
	{ 0x14, "PMU_BUCK_1_CFG" },
	{ 0x15, "PMU_BUCK_2_CFG" },
	{ 0x16, "PMU_SPECIAL_CFG" },
	{ 0x17, "PMU_LDO_1_CFG" },
	{ 0x18, "PMU_LDO_2_CFG" },
	{ 0x19, "PMU_LDO_3_CFG" },
	{ 0x1a, "PMU_LDO_4_CFG" },
	{ 0x1b, "PMU_LDO_5_CFG" },
	{ 0x1c, "PMU_LDO_6_CFG" },
	{ 0x1d, "PMU_LDO_7_CFG" },
	{ 0x1e, "PMU_LDO_8_CFG" },
	{ 0x1f, "PMU_LDO_9_CFG" },
	{ 0x20, "PMU_LDO_10_CFG" },
	{ 0x21, "PMU_LDO_11_CFG" },
	{ 0x22, "PMU_LDO_CONTROL" },
	{ 0x23, "PMU_BUCK_CONTROL" },
	{ 0x24, "PMU_BUCK_CONTROL_2" },
	{ 0x25, "PMU_WLED_ISET" },
	{ 0x26, "PMU_WLED_CONTROL" },
	{ 0x27, "PMU_CHARGE_BUCK_CONTROL" },
	{ 0x28, "PMU_CHARGE_CONTROL_A" },
	{ 0x29, "PMU_CHARGE_CONTROL_B" },
	{ 0x2a, "PMU_CHARGE_TIME" },
	{ 0x2b, "PMU_CHARGE_MISC" },
	{ 0x30, "PMU_ADC_CONTROL" },
	{ 0x31, "PMU_ADC_LSB" },
	{ 0x32, "PMU_ADC_MSB" },
	{ 0x35, "PMU_ICHG_AVG" },
	{ 0x3c, "PMU_MISC_ENABLE" },
	{ 0x5f, "PMU_SYS_CONFIG" },
	{ 0x6e, "PMU_N31_STATE" },
	{ 0x6f, "PMU_N31_CAL_INPUT" },
};

/* Named ranges; 0x6E/0x6F are looked up before the MEMBYTE range wins. */
static const char *n31_pmu_reg_name(u8 reg, char *buf, size_t len)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(n31_pmu_names); i++)
		if (n31_pmu_names[i].reg == reg)
			return n31_pmu_names[i].name;

	if (reg >= 0x40 && reg <= 0x4f)
		scnprintf(buf, len, "PMU_RTC_OR_UPCOUNT_%u", reg - 0x40);
	else if (reg >= 0x50 && reg <= 0x57)
		scnprintf(buf, len, "PMU_GPIO_%u", reg - 0x50 + 1);
	else if (reg >= 0x59 && reg <= 0x5b)
		scnprintf(buf, len, "PMU_GPIO_DEBOUNCE_%u", reg - 0x59 + 1);
	else if (reg >= 0x5c && reg <= 0x5e)
		scnprintf(buf, len, "PMU_BUTTON_%u", reg - 0x5c + 1);
	else if (reg >= 0x60 && reg <= 0x87)
		scnprintf(buf, len, "PMU_MEMBYTE_%u", reg - 0x60);
	else
		scnprintf(buf, len, "PMU_RESERVED_%02X", reg);
	return buf;
}

/*
 * Rail decode. `active` names the register whose `mask` bit gates the rail;
 * `vsel` holds the 5-bit voltage code. Ownership is deliberately unassigned:
 * which peripheral each rail feeds should come out of a snapshot diff.
 */
struct n31_pmu_rail {
	const char *name;
	u8 vsel;
	u8 active_reg;
	u8 active_mask;
	u16 base_mv;
	u16 step_mv;
};

static const struct n31_pmu_rail n31_pmu_rails[] = {
	{ "PMU_LDO_1",  0x17, 0x10, 0x08, 2500,  50 },
	{ "PMU_LDO_2",  0x18, 0x10, 0x10, 1500,  50 },
	{ "PMU_LDO_3",  0x19, 0x10, 0x20, 2500,  50 },
	{ "PMU_LDO_4",  0x1a, 0x10, 0x40, 1800,  50 },
	{ "PMU_LDO_5",  0x1b, 0x10, 0x80, 2500,  50 },
	{ "PMU_LDO_6",  0x1c, 0x11, 0x01, 2500,  50 },
	{ "PMU_LDO_7",  0x1d, 0x11, 0x02, 1500, 100 },
	{ "PMU_LDO_8",  0x1e, 0x11, 0x04, 2000,  50 },
	{ "PMU_LDO_9",  0x1f, 0x11, 0x80, 1200,  25 },
	{ "PMU_LDO_10", 0x20, 0x11, 0x10, 1700,  50 },
	{ "PMU_LDO_11", 0x21, 0x11, 0x20, 1700,  50 },
	{ "PMU_WDIG",   N31_PMU_NO_VSEL, 0x11, 0x40, 0, 0 },
};

/* Registers worth watching across an audio or display state change. */
static const u8 n31_pmu_watch[] = {
	0x0d, 0x0e, 0x10, 0x11, 0x13, 0x14, 0x15, 0x16, 0x17, 0x1a,
	0x23, 0x24, 0x25, 0x26, 0x29, 0x2a, 0x2b, 0x30, 0x3c,
};

/*
 * Registers the PMU updates on its own: measurement results, counters and
 * latched status. They differ between any two reads, so a diff marks them
 * instead of letting them bury a real rail change.
 */
static bool n31_pmu_reg_is_live(u8 reg)
{
	if (reg >= 0x01 && reg <= 0x08)		/* events and status */
		return true;
	if (reg >= 0x31 && reg <= 0x35)		/* ADC result, charge current */
		return true;
	if (reg >= 0x3d && reg <= 0x3f)		/* observed ticking with the ADC */
		return true;
	if (reg >= 0x40 && reg <= 0x4f)		/* RTC / up-counter */
		return true;
	return false;
}

static u8 n31_pmu_snap[N31_PMU_REG_MAX];
static bool n31_pmu_snap_valid;
static struct dentry *n31_pmu_debugfs;

static int n31_pmu_read(u8 reg)
{
	struct i2c_client *client = d1830_poweroff_client;

	if (!client)
		return -ENODEV;
	return i2c_smbus_read_byte_data(client, reg);
}

static void n31_pmu_read_all(u8 *out, bool *ok)
{
	unsigned int r;

	for (r = 0; r < N31_PMU_REG_MAX; r++) {
		int v = n31_pmu_read((u8)r);

		ok[r] = v >= 0;
		out[r] = ok[r] ? (u8)v : 0;
	}
}

static int n31_pmu_variant_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%s\n", pmu_variant ? pmu_variant : N31_PMU_VARIANT);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_variant);

static int n31_pmu_regs_raw_show(struct seq_file *s, void *unused)
{
	unsigned int r;

	for (r = 0; r < N31_PMU_REG_MAX; r += 16) {
		unsigned int i;

		seq_printf(s, "%02x:", r);
		for (i = 0; i < 16 && r + i < N31_PMU_REG_MAX; i++) {
			int v = n31_pmu_read((u8)(r + i));

			if (v < 0)
				seq_puts(s, " --");
			else
				seq_printf(s, " %02x", v);
		}
		seq_puts(s, "\n");
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_regs_raw);

static int n31_pmu_regs_named_show(struct seq_file *s, void *unused)
{
	char buf[32];
	unsigned int r;

	for (r = 0; r < N31_PMU_REG_MAX; r++) {
		int v = n31_pmu_read((u8)r);

		if (v < 0)
			continue;
		seq_printf(s, "0x%02x  %-24s 0x%02x\n", r,
			   n31_pmu_reg_name((u8)r, buf, sizeof(buf)), v);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_regs_named);

static int n31_pmu_rails_show(struct seq_file *s, void *unused)
{
	unsigned int i;

	seq_puts(s, "rail        vsel active      raw  on  mV\n");
	for (i = 0; i < ARRAY_SIZE(n31_pmu_rails); i++) {
		const struct n31_pmu_rail *ra = &n31_pmu_rails[i];
		int act = n31_pmu_read(ra->active_reg);
		int cfg = ra->vsel == N31_PMU_NO_VSEL ? -1 :
			  n31_pmu_read(ra->vsel);
		bool on = act >= 0 && (act & ra->active_mask);

		seq_printf(s, "%-11s ", ra->name);
		if (ra->vsel == N31_PMU_NO_VSEL)
			seq_puts(s, "---- ");
		else
			seq_printf(s, "0x%02x ", ra->vsel);
		seq_printf(s, "0x%02x/0x%02x ", ra->active_reg, ra->active_mask);
		if (cfg < 0)
			seq_puts(s, "  -- ");
		else
			seq_printf(s, "0x%02x ", cfg);
		seq_printf(s, "%-3s ", act < 0 ? "?" : (on ? "yes" : "no"));
		if (cfg >= 0 && ra->step_mv)
			seq_printf(s, "%u",
				   ra->base_mv +
				   (cfg & N31_PMU_VSEL_MASK) * ra->step_mv);
		else
			seq_puts(s, "-");
		seq_puts(s, "\n");
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_rails);

/*
 * The boot rail sequence in this driver's register names. Blind writes and
 * read-modify-writes are shown as they appear, because reproducing one as the
 * other would change what the hardware ends up with.
 */
static int n31_pmu_bootseq_decode_show(struct seq_file *s, void *unused)
{
	seq_puts(s, "sub_23EC — rail configuration and active state\n");
	seq_puts(s, "  PMU_BUCK_CONTROL   [0x23] &= 0xFC\n");
	seq_puts(s, "  PMU_BUCK_1_CFG     [0x14]  = computed 5-bit A\n");
	seq_puts(s, "  PMU_BUCK_2_CFG     [0x15]  = computed 5-bit B\n");
	seq_puts(s, "  PMU_SPECIAL_CFG    [0x16]  = computed 5-bit B\n");
	seq_puts(s, "  PMU_LDO_1_CFG      [0x17]  = computed 5-bit B\n");
	seq_puts(s, "  PMU_LDO_4_CFG      [0x1A]  = 0xB2   (written twice)\n");
	seq_puts(s, "  PMU_ACTIVE_1       [0x10]  = (old & 0x2F) | 0x10\n");
	seq_puts(s, "                             cold boot also | 0x20\n");
	seq_puts(s, "  PMU_ACTIVE_2       [0x11] |= 0x07\n");
	seq_puts(s, "  PMU_HIBERNATE_1    [0x13] |= 0x02\n");
	seq_puts(s, "\n");
	seq_puts(s, "sub_27F4 — board power initialisation tail\n");
	seq_puts(s, "  PMU_SYS_CONTROL    [0x0D] &= 0x8F\n");
	seq_puts(s, "  (apply sub_23EC)\n");
	seq_puts(s, "  PMU_ADC_CONTROL    [0x30] |= 0x40\n");
	seq_puts(s, "  PMU_GPIO_DEBOUNCE_1[0x59] &= 0xE3\n");
	seq_puts(s, "  PMU_MISC_ENABLE    [0x3C]  = 0x01\n");
	seq_puts(s, "  PMU_CHARGE_CONTROL_B[0x29] = (old & 0xEC) | 0x10\n");
	seq_puts(s, "  PMU_CHARGE_TIME    [0x2A]  = (old & 0xC0) | 0x14\n");
	seq_puts(s, "  PMU_CHARGE_MISC    [0x2B]  = (old & 0xF0) | 0x01\n");
	seq_puts(s, "  PMU_FAULT_LOG      [0x0E]  = 0x20\n");
	seq_puts(s, "  PMU_BUCK_CONTROL_2 [0x24]  = f(cal) >> 1\n");
	seq_puts(s, "  PMU_WLED_ISET      [0x25]  = 4 * (f(cal) & 1)\n");
	seq_puts(s, "  PMU_WLED_CONTROL   [0x26] &= ~0x01\n");
	seq_puts(s, "  PMU_SYS_CONTROL    [0x0D] &= 0xF3\n");
	seq_puts(s, "\n");
	seq_puts(s, "sub_1E7C — low-power boot finish\n");
	seq_puts(s, "  PMU_CHARGE_CONTROL_B[0x29] = (old & 0xF8) | 0x06\n");
	seq_puts(s, "  PMU_BUCK_CONTROL_2 [0x24]  = f(0x28) >> 1\n");
	seq_puts(s, "  PMU_WLED_ISET      [0x25]  = 4 * (f(0x28) & 1)\n");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_bootseq_decode);

/* Reading this captures the current register file; diff_last compares to it. */
static int n31_pmu_snapshot_show(struct seq_file *s, void *unused)
{
	bool ok[N31_PMU_REG_MAX];
	char buf[32];
	unsigned int r;

	n31_pmu_read_all(n31_pmu_snap, ok);
	n31_pmu_snap_valid = true;

	seq_puts(s, "captured; compare with diff_last\n");
	for (r = 0; r < N31_PMU_REG_MAX; r++) {
		if (!ok[r])
			continue;
		seq_printf(s, "0x%02x  %-24s 0x%02x\n", r,
			   n31_pmu_reg_name((u8)r, buf, sizeof(buf)),
			   n31_pmu_snap[r]);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_snapshot);

static int n31_pmu_diff_last_show(struct seq_file *s, void *unused)
{
	char buf[32];
	unsigned int r, n = 0;

	if (!n31_pmu_snap_valid) {
		seq_puts(s, "no snapshot yet; read snapshot first\n");
		return 0;
	}
	for (r = 0; r < N31_PMU_REG_MAX; r++) {
		int v = n31_pmu_read((u8)r);
		u8 old, xor;

		if (v < 0)
			continue;
		old = n31_pmu_snap[r];
		if (old == (u8)v)
			continue;
		xor = old ^ (u8)v;
		seq_printf(s, "0x%02x  %-24s 0x%02x -> 0x%02x  xor=0x%02x  set=0x%02x cleared=0x%02x%s\n",
			   r, n31_pmu_reg_name((u8)r, buf, sizeof(buf)),
			   old, v, xor, (u8)(xor & v), (u8)(xor & old),
			   n31_pmu_reg_is_live((u8)r) ? "  [live]" : "");
		if (!n31_pmu_reg_is_live((u8)r))
			n++;
	}
	if (!n)
		seq_puts(s,
			 "no change since snapshot (ignoring [live] registers)\n");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_diff_last);

static void n31_pmu_show_watch(struct seq_file *s, const char *tag)
{
	char buf[32];
	unsigned int i;

	seq_printf(s, "=== %s ===\n", tag);
	for (i = 0; i < ARRAY_SIZE(n31_pmu_watch); i++) {
		u8 reg = n31_pmu_watch[i];
		int v = n31_pmu_read(reg);

		seq_printf(s, "0x%02x  %-24s ", reg,
			   n31_pmu_reg_name(reg, buf, sizeof(buf)));
		if (v < 0)
			seq_puts(s, "--\n");
		else
			seq_printf(s, "0x%02x\n", v);
	}
}

static int n31_pmu_audio_snapshot_show(struct seq_file *s, void *unused)
{
	n31_pmu_show_watch(s, "audio rail watch");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_audio_snapshot);

static int n31_pmu_display_snapshot_show(struct seq_file *s, void *unused)
{
	n31_pmu_show_watch(s, "display rail watch");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_display_snapshot);

/*
 * Show what the boot sequence would write given the registers as they stand,
 * without writing any of it. The read-modify-write steps are evaluated
 * against live values so the result is what would actually land.
 */
static int n31_pmu_apply_bootseq_dryrun_show(struct seq_file *s, void *unused)
{
	int r10, r11, r13, r23, r0d, r29, r2a, r2b, r26, r30, r59;

	seq_printf(s, "allow_pmu_writes=%d apply_boot_rails=%d (dry run only)\n\n",
		   allow_pmu_writes, apply_boot_rails);

	r23 = n31_pmu_read(0x23);
	r10 = n31_pmu_read(0x10);
	r11 = n31_pmu_read(0x11);
	r13 = n31_pmu_read(0x13);
	r0d = n31_pmu_read(0x0d);
	r29 = n31_pmu_read(0x29);
	r2a = n31_pmu_read(0x2a);
	r2b = n31_pmu_read(0x2b);
	r26 = n31_pmu_read(0x26);
	r30 = n31_pmu_read(0x30);
	r59 = n31_pmu_read(0x59);
	if (r23 < 0 || r10 < 0 || r11 < 0 || r13 < 0) {
		seq_puts(s, "PMU read failed\n");
		return 0;
	}

	seq_puts(s, "sub_23EC:\n");
	seq_printf(s, "  PMU_BUCK_CONTROL   0x%02x -> 0x%02x\n",
		   r23, r23 & 0xfc);
	seq_printf(s, "  PMU_LDO_4_CFG      ---- -> 0xb2 (blind, twice)\n");
	seq_printf(s, "  PMU_ACTIVE_1       0x%02x -> 0x%02x (cold: 0x%02x)\n",
		   r10, (r10 & 0x2f) | 0x10, (r10 & 0x2f) | 0x30);
	seq_printf(s, "  PMU_ACTIVE_2       0x%02x -> 0x%02x\n",
		   r11, r11 | 0x07);
	seq_printf(s, "  PMU_HIBERNATE_1    0x%02x -> 0x%02x\n",
		   r13, r13 | 0x02);

	seq_puts(s, "sub_27F4 tail:\n");
	if (r0d >= 0)
		seq_printf(s, "  PMU_SYS_CONTROL    0x%02x -> 0x%02x then 0x%02x\n",
			   r0d, r0d & 0x8f, (r0d & 0x8f) & 0xf3);
	if (r30 >= 0)
		seq_printf(s, "  PMU_ADC_CONTROL    0x%02x -> 0x%02x\n",
			   r30, r30 | 0x40);
	if (r59 >= 0)
		seq_printf(s, "  PMU_GPIO_DEBOUNCE_1 0x%02x -> 0x%02x\n",
			   r59, r59 & 0xe3);
	seq_puts(s, "  PMU_MISC_ENABLE    ---- -> 0x01 (blind)\n");
	if (r29 >= 0)
		seq_printf(s, "  PMU_CHARGE_CONTROL_B 0x%02x -> 0x%02x\n",
			   r29, (r29 & 0xec) | 0x10);
	if (r2a >= 0)
		seq_printf(s, "  PMU_CHARGE_TIME    0x%02x -> 0x%02x\n",
			   r2a, (r2a & 0xc0) | 0x14);
	if (r2b >= 0)
		seq_printf(s, "  PMU_CHARGE_MISC    0x%02x -> 0x%02x\n",
			   r2b, (r2b & 0xf0) | 0x01);
	seq_puts(s, "  PMU_FAULT_LOG      ---- -> 0x20 (blind)\n");
	if (r26 >= 0)
		seq_printf(s, "  PMU_WLED_CONTROL   0x%02x -> 0x%02x\n",
			   r26, r26 & ~0x01);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(n31_pmu_apply_bootseq_dryrun);


/* ------------------------------------------------------------------ */
/* Consumer rail control                                                */
/*                                                                      */
/* A rail is enabled while at least one consumer holds it and is turned */
/* off again once the last reference has been gone for rail_off_delay_ms.*/
/* The delay matters for the analog rail: track changes drop and retake  */
/* it within a second, and cycling it each time both wastes power and    */
/* risks an audible pop.                                                 */
/*                                                                      */
/* Only the rail's own enable bit in PMU_ACTIVE_* is touched, never the  */
/* whole register.                                                       */
/* ------------------------------------------------------------------ */

static bool rail_control = true;
module_param(rail_control, bool, 0644);
MODULE_PARM_DESC(rail_control,
		 "Let drivers enable/disable their rail via n31_pmu_rail_get/put");

static unsigned int rail_off_delay_ms = 5000;
module_param(rail_off_delay_ms, uint, 0644);
MODULE_PARM_DESC(rail_off_delay_ms,
		 "Idle time before a released rail is powered down (ms)");

struct n31_pmu_rail_state {
	int users;
	bool on;
	struct delayed_work off_work;
};

static struct n31_pmu_rail_state n31_pmu_rail_state[ARRAY_SIZE(n31_pmu_rails)];
static DEFINE_MUTEX(n31_pmu_rail_lock);

/* Enable bits in `active_reg` belonging to rails a consumer still holds. */
static u8 n31_pmu_rail_held_mask(u8 active_reg)
{
	unsigned int i;
	u8 mask = 0;

	mutex_lock(&n31_pmu_rail_lock);
	for (i = 0; i < ARRAY_SIZE(n31_pmu_rails); i++)
		if (n31_pmu_rails[i].active_reg == active_reg &&
		    n31_pmu_rail_state[i].users)
			mask |= n31_pmu_rails[i].active_mask;
	mutex_unlock(&n31_pmu_rail_lock);
	return mask;
}

static int n31_pmu_rail_apply(unsigned int id, bool on)
{
	const struct n31_pmu_rail *ra = &n31_pmu_rails[id];
	struct i2c_client *client = d1830_poweroff_client;
	int ret;

	if (!client)
		return -ENODEV;
	if (!rail_control)
		return -EPERM;

	ret = d1830_rmw(client, ra->active_reg,
			on ? 0 : ra->active_mask,
			on ? ra->active_mask : 0);
	if (ret) {
		dev_warn(&client->dev, "%s %s failed: %d\n",
			 ra->name, on ? "enable" : "disable", ret);
		return ret;
	}
	d1830_vinfo(&client->dev, "%s %s\n", ra->name, on ? "on" : "off");
	return 0;
}

static void n31_pmu_rail_off_work(struct work_struct *work)
{
	struct n31_pmu_rail_state *st = container_of(to_delayed_work(work),
						     struct n31_pmu_rail_state,
						     off_work);
	unsigned int id = st - n31_pmu_rail_state;

	mutex_lock(&n31_pmu_rail_lock);
	if (!st->users && st->on && !n31_pmu_rail_apply(id, false))
		st->on = false;
	mutex_unlock(&n31_pmu_rail_lock);
}

/* Enable a rail and hold it. Balanced by n31_pmu_rail_put(). */
int n31_pmu_rail_get(unsigned int id)
{
	struct n31_pmu_rail_state *st;
	int ret = 0;

	if (id >= ARRAY_SIZE(n31_pmu_rails))
		return -EINVAL;
	st = &n31_pmu_rail_state[id];

	mutex_lock(&n31_pmu_rail_lock);
	cancel_delayed_work(&st->off_work);
	if (!st->on) {
		ret = n31_pmu_rail_apply(id, true);
		if (!ret)
			st->on = true;
	}
	if (!ret)
		st->users++;
	mutex_unlock(&n31_pmu_rail_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(n31_pmu_rail_get);

/* Drop a reference; the rail powers down once idle for rail_off_delay_ms. */
void n31_pmu_rail_put(unsigned int id)
{
	struct n31_pmu_rail_state *st;

	if (id >= ARRAY_SIZE(n31_pmu_rails))
		return;
	st = &n31_pmu_rail_state[id];

	mutex_lock(&n31_pmu_rail_lock);
	if (st->users)
		st->users--;
	if (!st->users && st->on)
		schedule_delayed_work(&st->off_work,
				      msecs_to_jiffies(rail_off_delay_ms));
	mutex_unlock(&n31_pmu_rail_lock);
}
EXPORT_SYMBOL_GPL(n31_pmu_rail_put);

static void n31_pmu_rail_init(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(n31_pmu_rail_state); i++)
		INIT_DELAYED_WORK(&n31_pmu_rail_state[i].off_work,
				  n31_pmu_rail_off_work);
}

static void n31_pmu_rail_exit(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(n31_pmu_rail_state); i++) {
		cancel_delayed_work_sync(&n31_pmu_rail_state[i].off_work);
		/*
		 * Leave rail state as-is on unbind: the boot configuration is
		 * what the rest of the system expects to find.
		 */
	}
}


/* ------------------------------------------------------------------ */
/* Screen sleep and power-button policy                                 */
/*                                                                      */
/* Short press toggles screen sleep: fade the backlight out and quiesce */
/* touch. Holding the button past power_hold_ms asks the kernel to shut */
/* down, which reaches the PMU through the machine pm_power_off this    */
/* driver already installs.                                             */
/*                                                                      */
/* The panel itself is deliberately left running. Nothing in this stack  */
/* can re-initialise it, so powering it down would be a one-way trip     */
/* until the panel init sequence is recovered. Backlight and touch are   */
/* where the current draw is anyway.                                     */
/*                                                                      */
/* Two tiers are selected from whether the analog audio rail is held:    */
/* with playback running a press only sleeps the screen, and the deeper  */
/* path is left for when nothing is playing.                             */
/* ------------------------------------------------------------------ */

/*
 * Off by default. Twice now a power press has left the panel stuck --
 * image still on screen, no fade, and no wake from a second press --
 * needing DFU to recover. The transition has never been driven
 * deliberately, only caught by accident, so arming it on every boot
 * risks the display on a device whose only recovery is a reflash.
 *
 * The mechanism is still there and still wired: set screen_sleep_enable=1
 * to test it on purpose, with the console reachable, rather than
 * discovering it by pressing a button.
 */
static bool screen_sleep_enable;
module_param(screen_sleep_enable, bool, 0644);
MODULE_PARM_DESC(screen_sleep_enable,
		 "Short press toggles screen sleep (default Y)");

static unsigned int power_hold_ms = 4000;
module_param(power_hold_ms, uint, 0644);
MODULE_PARM_DESC(power_hold_ms,
		 "Hold the power button this long to power off (0 = never)");

static unsigned int backlight_fade_ms = 400;
module_param(backlight_fade_ms, uint, 0644);
MODULE_PARM_DESC(backlight_fade_ms, "Backlight ramp duration either way (ms)");

static bool n31_screen_asleep;
static int n31_screen_saved_level = -1;
static unsigned long n31_power_press_jiffies;
static bool n31_power_off_pending;
static DEFINE_MUTEX(n31_screen_lock);

/*
 * True while audio is playing. The codec has no PMU rail of its own, so
 * this asks the codec directly rather than inferring it from rail state.
 */
static bool n31_audio_active(void)
{
	bool (*active)(void);
	bool r = false;

	active = (bool (*)(void))__symbol_get("n31_audio_playback_active");
	if (active) {
		r = active();
		__symbol_put("n31_audio_playback_active");
	}
	return r;
}

/*
 * Sleeping the panel is only safe if we can also bring it back. These
 * are resolved through __symbol_get, so a provider that is not loaded
 * silently does half the job: the panel goes off with no fade, which
 * looks like a lockup, or the backlight comes up over a panel that is
 * still off, which looks like a device that will not wake. Neither is
 * distinguishable from a crash at the time, so refuse to start a
 * transition we cannot finish and say why.
 */
static void n31_screen_set(bool asleep)
{
	int (*fade)(int, unsigned int);
	int (*level)(void);
	int (*touch)(void);
	int (*lcd)(bool);

	mutex_lock(&n31_screen_lock);
	if (asleep == n31_screen_asleep)
		goto out;

	fade = (int (*)(int, unsigned int))__symbol_get("n31_backlight_fade");
	level = (int (*)(void))__symbol_get("n31_backlight_level");
	lcd = (int (*)(bool))__symbol_get("n31_lcd_power");

	if (asleep && !lcd && !fade) {
		/*
		 * Nothing to dim and nothing to blank. Going "asleep" here
		 * would only set a flag that stops the next press waking
		 * anything, which is worse than staying awake.
		 */
		if (level)
			__symbol_put("n31_backlight_level");
		pr_warn("n31: screen sleep skipped -- no backlight or LCD provider\n");
		goto out;
	}

	if (asleep) {
		if (level) {
			n31_screen_saved_level = level();
			__symbol_put("n31_backlight_level");
		}
		if (fade) {
			fade(0, backlight_fade_ms);
			__symbol_put("n31_backlight_fade");
		}
		touch = (int (*)(void))__symbol_get("n31_touch_suspend");
		if (touch) {
			touch();
			__symbol_put("n31_touch_suspend");
		}
		/* Panel last, once nothing is drawing to it. */
		if (lcd)
			lcd(false);
	} else {
		/* Panel first: it has to be scanning before the light comes up. */
		if (lcd)
			lcd(true);
		touch = (int (*)(void))__symbol_get("n31_touch_resume");
		if (touch) {
			touch();
			__symbol_put("n31_touch_resume");
		}
		if (level)
			__symbol_put("n31_backlight_level");
		if (fade) {
			fade(n31_screen_saved_level > 0 ?
			     n31_screen_saved_level : 1,
			     backlight_fade_ms);
			__symbol_put("n31_backlight_fade");
		}
	}

	if (lcd)
		__symbol_put("n31_lcd_power");

	n31_screen_asleep = asleep;
	pr_info("n31: screen %s (audio %s, bl=%s lcd=%s)\n",
		asleep ? "asleep" : "awake",
		n31_audio_active() ? "active" : "idle",
		fade ? "yes" : "no", lcd ? "yes" : "no");
out:
	mutex_unlock(&n31_screen_lock);
}

/*
 * The transition takes backlight_fade_ms either way and holds
 * n31_screen_lock while it does. Called straight from the button path
 * that runs it, that stalls every other button for the length of the
 * fade -- so a press during it appears to do nothing, which reads as a
 * hang rather than a fade. Push it to a workqueue and let the poller
 * carry on.
 */
static bool n31_screen_want_asleep;

static void n31_screen_work_fn(struct work_struct *work)
{
	n31_screen_set(READ_ONCE(n31_screen_want_asleep));
}
static DECLARE_WORK(n31_screen_worker, n31_screen_work_fn);

static void n31_screen_request(bool asleep)
{
	WRITE_ONCE(n31_screen_want_asleep, asleep);
	schedule_work(&n31_screen_worker);
}

bool n31_screen_is_asleep(void)
{
	return n31_screen_asleep;
}
EXPORT_SYMBOL_GPL(n31_screen_is_asleep);

static void n31_power_off_work(struct work_struct *work)
{
	pr_warn("n31: power button held — shutting down\n");
	orderly_poweroff(true);
}
static DECLARE_WORK(n31_power_off_worker, n31_power_off_work);

/*
 * Called on every observed Sleep-button edge. Press only records when it
 * started; the decision happens on release, or as soon as the hold passes
 * power_hold_ms while still down.
 */
static void n31_power_button(bool pressed)
{
	unsigned long held_ms;

	if (pressed) {
		n31_power_press_jiffies = jiffies;
		n31_power_off_pending = false;
		return;
	}

	if (!n31_power_press_jiffies)
		return;
	held_ms = jiffies_to_msecs(jiffies - n31_power_press_jiffies);
	n31_power_press_jiffies = 0;

	if (power_hold_ms && held_ms >= power_hold_ms) {
		if (!n31_power_off_pending) {
			n31_power_off_pending = true;
			schedule_work(&n31_power_off_worker);
		}
		return;
	}

	if (!screen_sleep_enable)
		return;

	/*
	 * A press while asleep only wakes; it should not immediately put the
	 * screen back down.
	 */
	n31_screen_request(!n31_screen_asleep);
}

/* Long holds must act while the button is still down, not on release. */
static void n31_power_button_poll(void)
{
	unsigned long held_ms;

	if (!n31_power_press_jiffies || !power_hold_ms)
		return;
	if (n31_power_off_pending)
		return;
	held_ms = jiffies_to_msecs(jiffies - n31_power_press_jiffies);
	if (held_ms < power_hold_ms)
		return;
	n31_power_off_pending = true;
	schedule_work(&n31_power_off_worker);
}

/* Home wakes the screen but is otherwise left to userspace. */
static void n31_home_button(bool pressed)
{
	if (pressed && n31_screen_asleep)
		n31_screen_request(false);
}

/* ------------------------------------------------------------------ */
/* Regulator provider                                                   */
/*                                                                      */
/* The rails already have a refcounted in-kernel API, but nothing gave  */
/* userspace or other drivers the conventional view of them. Exposing   */
/* them as regulators means /sys/class/regulator carries the real names */
/* and voltages, and a consumer can be wired up in DT like any other    */
/* board.                                                               */
/*                                                                      */
/* Enable and disable route through n31_pmu_rail_get/put rather than    */
/* touching the ACTIVE register directly, so the regulator count and    */
/* the driver-internal holders cannot disagree about who wants a rail   */
/* on -- which is exactly the confusion that let the display rail be    */
/* dropped underneath a live panel.                                     */
/* ------------------------------------------------------------------ */

struct n31_pmu_reg {
	struct regulator_desc desc;
	char name[16];
	unsigned int id;
};

static struct n31_pmu_reg n31_pmu_regs[ARRAY_SIZE(n31_pmu_rails)];

static int n31_pmu_reg_enable(struct regulator_dev *rdev)
{
	unsigned int id = rdev_get_id(rdev);

	if (!allow_pmu_writes)
		return -EPERM;
	return n31_pmu_rail_get(id);
}

static int n31_pmu_reg_disable(struct regulator_dev *rdev)
{
	unsigned int id = rdev_get_id(rdev);

	if (!allow_pmu_writes)
		return -EPERM;
	n31_pmu_rail_put(id);
	return 0;
}

/*
 * Report what the hardware says rather than what this driver last asked
 * for: the rail may have been brought up by the bootloader, and a rail
 * that is already on is the normal case at probe.
 */
static int n31_pmu_reg_is_enabled(struct regulator_dev *rdev)
{
	unsigned int id = rdev_get_id(rdev);
	const struct n31_pmu_rail *r;
	struct i2c_client *client = d1830_poweroff_client;
	int v;

	if (id >= ARRAY_SIZE(n31_pmu_rails) || !client)
		return -ENODEV;
	r = &n31_pmu_rails[id];
	v = i2c_smbus_read_byte_data(client, r->active_reg);
	if (v < 0)
		return v;
	return !!(v & r->active_mask);
}

static int n31_pmu_reg_get_voltage(struct regulator_dev *rdev)
{
	unsigned int id = rdev_get_id(rdev);
	const struct n31_pmu_rail *r;
	struct i2c_client *client = d1830_poweroff_client;
	int v;

	if (id >= ARRAY_SIZE(n31_pmu_rails) || !client)
		return -ENODEV;
	r = &n31_pmu_rails[id];
	if (r->vsel == N31_PMU_NO_VSEL || !r->step_mv)
		return -EINVAL;
	v = i2c_smbus_read_byte_data(client, r->vsel);
	if (v < 0)
		return v;
	/* Low bits select the step; the upper bits are trim/control. */
	return (r->base_mv + (v & 0x1f) * r->step_mv) * 1000;
}

static const struct regulator_ops n31_pmu_reg_ops = {
	.enable = n31_pmu_reg_enable,
	.disable = n31_pmu_reg_disable,
	.is_enabled = n31_pmu_reg_is_enabled,
	.get_voltage = n31_pmu_reg_get_voltage,
};

#define D1830_BT_REG_A		0x57	/* 87 */
#define D1830_BT_REG_B		0x58	/* 88 */
#define D1830_BT_A_MASK		0xc0	/* bits 7:6 */
#define D1830_BT_B_MASK		0x71	/* bit 0 and bits 6:4 */

/* Defined below; the regulator ops need it before its definition. */
int d1830_bt_rails(bool on);

/*
 * The Bluetooth rail, as a regulator.
 *
 * It does not fit n31_pmu_rails[]: that table describes single-register
 * LDOs at 0x17..0x21, and this one is two registers -- 0x57 bits 7:6 and
 * 0x58 bit 0 plus bits 6:4. So it gets its own descriptor whose enable and
 * disable defer to d1830_bt_rails(), which already knows the sequence.
 *
 * The point of exposing it at all is ordering. bcm2078-bt is built into
 * the kernel and probes around t=2.4s; this driver is a module userspace
 * loads at about t=7.1s. A driver asking for power through a bespoke hook
 * in that window gets -ENODEV and has no way to wait, so the controller
 * simply never came up. Asking for it as a regulator makes the kernel do
 * the waiting: devm_regulator_get() returns -EPROBE_DEFER until this
 * driver registers, and the consumer is re-probed afterwards.
 *
 * of_match plus regulators_node is what lets a device tree node name it,
 * which is the whole mechanism -- without those it is registered but
 * unreachable from DT.
 */
static int n31_bt_reg_enable(struct regulator_dev *rdev)
{
	return d1830_bt_rails(true);
}

static int n31_bt_reg_disable(struct regulator_dev *rdev)
{
	return d1830_bt_rails(false);
}

static int n31_bt_reg_is_enabled(struct regulator_dev *rdev)
{
	struct i2c_client *client = d1830_poweroff_client;
	int a;

	if (!client)
		return 0;
	a = i2c_smbus_read_byte_data(client, D1830_BT_REG_A);
	if (a < 0)
		return a;
	return (a & D1830_BT_A_MASK) ? 1 : 0;
}

static const struct regulator_ops n31_bt_reg_ops = {
	.enable = n31_bt_reg_enable,
	.disable = n31_bt_reg_disable,
	.is_enabled = n31_bt_reg_is_enabled,
};

static const struct regulator_desc n31_bt_reg_desc = {
	.name = "bt",
	.of_match = "bt",
	.regulators_node = "regulators",
	.id = 0x100,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &n31_bt_reg_ops,
	.n_voltages = 1,
};

static void n31_pmu_regulators_register(struct device *dev)
{
	struct regulator_config cfg = { };
	struct regulator_dev *rdev;
	unsigned int i;

	cfg.dev = dev;
	for (i = 0; i < ARRAY_SIZE(n31_pmu_rails); i++) {
		struct n31_pmu_reg *pr = &n31_pmu_regs[i];

		strscpy(pr->name, n31_pmu_rails[i].name, sizeof(pr->name));
		pr->id = i;
		pr->desc.name = pr->name;
		pr->desc.id = i;
		pr->desc.type = REGULATOR_VOLTAGE;
		pr->desc.owner = THIS_MODULE;
		pr->desc.ops = &n31_pmu_reg_ops;
		pr->desc.n_voltages = 1;

		cfg.driver_data = pr;
		rdev = devm_regulator_register(dev, &pr->desc, &cfg);
		if (IS_ERR(rdev))
			dev_warn(dev, "regulator %s: %ld\n",
				 pr->name, PTR_ERR(rdev));
	}
	{
		struct regulator_config btcfg = { };
		struct regulator_dev *btrdev;

		btcfg.dev = dev;
		btrdev = devm_regulator_register(dev, &n31_bt_reg_desc, &btcfg);
		if (IS_ERR(btrdev))
			dev_warn(dev, "bt regulator: %ld\n", PTR_ERR(btrdev));
		else
			dev_info(dev, "bt rail exposed as a regulator\n");
	}

	dev_info(dev, "%u PMU rails exposed as regulators (writes %s)\n",
		 (unsigned int)ARRAY_SIZE(n31_pmu_rails),
		 allow_pmu_writes ? "allowed" : "blocked");
}

static void n31_pmu_debugfs_init(void)
{
	struct dentry *d;

	d = debugfs_create_dir("n31_pmu", NULL);
	if (IS_ERR(d))
		return;
	n31_pmu_debugfs = d;

	debugfs_create_file("variant", 0444, d, NULL, &n31_pmu_variant_fops);
	debugfs_create_file("regs_raw", 0444, d, NULL, &n31_pmu_regs_raw_fops);
	debugfs_create_file("regs_named", 0444, d, NULL,
			    &n31_pmu_regs_named_fops);
	debugfs_create_file("rails", 0444, d, NULL, &n31_pmu_rails_fops);
	debugfs_create_file("bootseq_decode", 0444, d, NULL,
			    &n31_pmu_bootseq_decode_fops);
	debugfs_create_file("snapshot", 0444, d, NULL, &n31_pmu_snapshot_fops);
	debugfs_create_file("diff_last", 0444, d, NULL,
			    &n31_pmu_diff_last_fops);
	debugfs_create_file("audio_snapshot", 0444, d, NULL,
			    &n31_pmu_audio_snapshot_fops);
	debugfs_create_file("display_snapshot", 0444, d, NULL,
			    &n31_pmu_display_snapshot_fops);
	debugfs_create_file("apply_bootseq_dryrun", 0444, d, NULL,
			    &n31_pmu_apply_bootseq_dryrun_fops);
}

static void n31_pmu_debugfs_exit(void)
{
	debugfs_remove_recursive(n31_pmu_debugfs);
	n31_pmu_debugfs = NULL;
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
/*
 * Bluetooth companion rails.
 *
 * sub_51688C, the de-init half of the Bluetooth bring-up, zeroes two
 * entries through sub_158C82 -- a different accessor from the sub_7484
 * rail toggler, writing multi-bit fields rather than a single enable:
 *
 *   index 3: reg 87 bits 7:6, and reg 88 bit 0
 *   index 5: reg 88 bits 6:4
 *
 * Nothing in the init path sets them, so their running values come from
 * the bootloader. Clearing them without keeping the originals would mean
 * Bluetooth could be turned off exactly once per boot and never restored,
 * so the first power-off saves what it found and power-on puts it back.
 */

static u8 d1830_bt_saved_a, d1830_bt_saved_b;
static bool d1830_bt_saved;

int d1830_bt_rails(bool on)
{
	struct i2c_client *client = d1830_poweroff_client;
	int a, b, ret;

	if (!client)
		return -ENODEV;

	if (on) {
		if (!d1830_bt_saved)
			return 0;	/* never turned off; leave boot state alone */
		ret = d1830_rmw(client, D1830_BT_REG_A, D1830_BT_A_MASK,
				d1830_bt_saved_a & D1830_BT_A_MASK);
		if (!ret)
			ret = d1830_rmw(client, D1830_BT_REG_B, D1830_BT_B_MASK,
					d1830_bt_saved_b & D1830_BT_B_MASK);
		d1830_vinfo(&client->dev,
			    "bt rails restored r87=%02x r88=%02x ret=%d\n",
			    d1830_bt_saved_a, d1830_bt_saved_b, ret);
		return ret;
	}

	a = i2c_smbus_read_byte_data(client, D1830_BT_REG_A);
	if (a < 0)
		return a;
	b = i2c_smbus_read_byte_data(client, D1830_BT_REG_B);
	if (b < 0)
		return b;
	if (!d1830_bt_saved) {
		d1830_bt_saved_a = (u8)a;
		d1830_bt_saved_b = (u8)b;
		d1830_bt_saved = true;
	}
	ret = d1830_rmw(client, D1830_BT_REG_A, D1830_BT_A_MASK, 0);
	if (!ret)
		ret = d1830_rmw(client, D1830_BT_REG_B, D1830_BT_B_MASK, 0);
	d1830_vinfo(&client->dev,
		    "bt rails off (was r87=%02x r88=%02x) ret=%d\n",
		    a, b, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(d1830_bt_rails);

/* bcm2078-bt is built in and cannot link against this module, so it
 * publishes a hook and we fill it in. */
void bcm2078_register_bt_rails(int (*fn)(bool on));

/*
 * Read PMIC register 0x51 during touch bring-up.
 *
 * OSOS does this in sub_20E94, via sub_26144(...,8) -> sub_41286E(81),
 * between the 26494 probe and the download loop. The value it reads is
 * not used afterwards, which is exactly why it was skipped here for so
 * long -- and that reasoning was wrong. A register read is a bus
 * transaction, and plenty of PMIC registers do something when they are
 * addressed: clear a latched status, sample an input, arm a state
 * machine. The decompiler can show the returned value going nowhere and
 * still tell us nothing about what the read did to the part.
 *
 * 0x51 is inside the PMIC GPIO block (0x50..0x57). Whether this one
 * latches anything is unknown; what is known is that stock performs it
 * on every touch bring-up and we did not. Reproduce the sequence,
 * including the parts whose purpose is not obvious yet.
 */
int d1830_touch_bringup_read(void)
{
	struct i2c_client *client = d1830_poweroff_client;
	int v;

	if (!client)
		return -ENODEV;

	v = i2c_smbus_read_byte_data(client, 0x51);
	if (v < 0) {
		dev_warn(&client->dev,
			 "n31-pmic: touch bring-up read of 0x51 failed: %d\n",
			 v);
		return v;
	}

	d1830_vinfo(&client->dev,
		    "n31-pmic: touch bring-up read 0x51 = 0x%02x\n", v);
	return v;
}
EXPORT_SYMBOL_GPL(d1830_touch_bringup_read);

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
	/*
	 * The boot form of this write clears bits 6 and 7 -- display and
	 * accessory. That is correct exactly once, on a cold boot, when
	 * neither is up yet. Replaying it later switches off a panel that
	 * is already lit, which is what the white screen was.
	 *
	 * Two guards, because one is not enough. Restoring rails a driver
	 * holds only works if a driver actually claimed one, and the
	 * display is the case where none did: the panel arrives already
	 * running from the bootloader handoff, and the DRM driver is
	 * built in, so it probes long before this module exists and its
	 * claim cannot reach us. So also refuse to clear any of those two
	 * bits that the hardware currently has set. Turning a live rail
	 * off is never what this sequence is for.
	 */
	r16 |= n31_pmu_rail_held_mask(0x10);
	if (!apply_boot_rails)
		r16 |= (u8)(v21 & 0xc0);
	d1830_write8(client, 16, r16);

	d1830_rmw(client, 17, 0, 0x07);
	d1830_rmw(client, 19, 0, 0x02);

	d1830_vinfo(&client->dev, "n31-pmic: sub_23EC-equivalent complete\n");
	return 0;
}

/*
 * There is no "analog LDO enable" at registers 20-23 (0x14-0x17).
 *
 * A note in our Rockbox placeholder claimed the codec analog side was
 * powered by "regs 21-23 bit 4", and acting on it locked the kernel on
 * boot. The OSOS rail setter settles it: registers 20-23 take LABEL_25,
 * which computes the code at a 25 mV step and writes (code & 0x1F) --
 * the low five bits and nothing else. Contrast case 17/18, registers 46
 * and 47, which deliberately preserve (old & 0xE0); 20-23 preserve
 * nothing, because there is nothing up there to preserve. Register 0x1a
 * is different again: it alone uses (code & 0x1F) | 0xA0 at a 100 mV
 * step, which is where the idea of an enable pattern came from.
 *
 * So bit 4 in these registers is the top bit of a voltage field, not a
 * switch. Setting it on 0x15-0x17, which read 0x09 on this unit, moved
 * three rails by +16 steps -- 400 mV each, simultaneously -- and the
 * device did not survive it.
 *
 * If the codec analog supply is gated somewhere, it is not here. Whoever
 * picks this up next: establish which rail actually feeds the CS42L81
 * analog stage before writing anything in this range.
 */

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
		d1830_vinfo(&client->dev,
			    "n31-pmic: board rail trim skipped (allow_audio_rails=0)\n");
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
	/* bcm2078-bt is built in; hand it our rail control now that we have
	 * a client to talk to. */
	bcm2078_register_bt_rails(d1830_bt_rails);

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

	/* Every create here has a matching remove below. A leaked attribute
	 * outlives the module and reading it jumps into freed text, which is
	 * how buttons oopsed. */
	if (device_create_file(dev, &dev_attr_adc_channels))
		dev_warn(dev, "adc_channels sysfs failed\n");
	if (device_create_file(dev, &dev_attr_charger_regs))
		dev_warn(dev, "charger_regs sysfs failed\n");
	if (device_create_file(dev, &dev_attr_charger_watch))
		dev_warn(dev, "charger_watch sysfs failed\n");
	if (device_create_file(dev, &dev_attr_regs))
		dev_warn(dev, "regs sysfs failed\n");
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
		if (devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF,
						  SYS_OFF_PRIO_DEFAULT,
						  d1830_sys_off_handler, NULL))
			dev_warn(dev, "sys-off handler not registered\n");
		/*
	 * The charger configuration write runs by default: 0x1a already
	 * reads 0xb2 here because the bootloader wrote it, so this confirms
	 * the stock state and makes us correct on a path where it did not.
	 *
	 * The full sub_23EC replay stays behind charger_hw_init, because its
	 * 0x10 masking clears bit 6 -- a live rail -- and RetailOS only runs
	 * that at cold boot, before anything depends on those rails.
	 */
	if (charger_hw_init) {
		int cret = d1830_charger_hw_init(client, true);

		dev_warn(dev, "sub_23EC PMIC init replayed: %d\n", cret);
	} else if (ldo_1a_write) {
		int cret = d1830_charger_config(client);

		if (cret)
			dev_warn(dev, "0x1a rail write: %d\n", cret);
	}
	d1830_charger_update(client);
	d1830_charger_register(dev);
	if (d1830_rtc_register(client))
			dev_warn(&client->dev, "RTC not registered\n");
		if (register_restart_handler(&d1830_restart_nb))
			dev_warn(&client->dev,
				 "restart handler not registered\n");
		d1830_vinfo(dev, "registered pm_power_off (SEC reg %u bit0)\n",
			 D1830_REG_POWEROFF);
	} else {
		dev_warn(dev, "pm_power_off already set — sysfs do_poweroff only\n");
	}

	/* OSOS 9-12 mask only. No 27F4 tail, no 1-4 writeback, no IIC1 peek. */
	d1830_dump_irq_chain(client, "sec-left");
	d1830_osos_nirq_mask(client);
	d1830_dump_irq_chain(client, "osos-mask");

	/*
	 * Requesting this arms a real EIC line. The level/type encoding is
	 * now taken from the stock configuration path, but it has not been
	 * confirmed on hardware, and getting it wrong wedges the system
	 * rather than merely failing. Polling covers the buttons meanwhile,
	 * so this stays opt-in until INTSTAT is observed tracking a press.
	 */
	if (client->irq > 0 && pmic_irq) {
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
		/*
		 * Home, Sleep and Play live on the PMIC status registers, so
		 * they are reported here and nowhere else. n31-buttons keeps
		 * Vol+/Vol-, which are SoC GPIOs.
		 */
		gpio_dev->input->name = "n31-pmic-buttons";
		gpio_dev->input->phys = "d1830/gpio";
		gpio_dev->input->dev.parent = dev;
		gpio_dev->input->id.bustype = BUS_I2C;
		gpio_dev->input->phys = "d1830/input0";
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
	n31_pmu_rail_init();
	n31_pmu_debugfs_init();
	n31_pmu_regulators_register(&client->dev);
	if (device_create_file(dev, &dev_attr_buttons))
		dev_warn(dev, "buttons sysfs\n");
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
	device_remove_file(&client->dev, &dev_attr_adc_channels);
	device_remove_file(&client->dev, &dev_attr_charger_regs);
	device_remove_file(&client->dev, &dev_attr_charger_watch);
	device_remove_file(&client->dev, &dev_attr_buttons);
	device_remove_file(&client->dev, &dev_attr_regs);
	device_remove_file(&client->dev, &dev_attr_do_poweroff);
	if (pm_power_off == d1830_pm_power_off)
		pm_power_off = NULL;
	unregister_restart_handler(&d1830_restart_nb);
	d1830_n31_din_nirq_hook = NULL;
	n31_pmu_debugfs_exit();
	n31_pmu_rail_exit();
	bcm2078_register_bt_rails(NULL);
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
