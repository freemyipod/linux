// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform backlight for Samsung/Apple S5L8740 (iPod nano 7G / N31)
 *
 * MMIO block @ 0x3E000000 (LCD AUX / backlight):
 *   +0x04  enable — bit0
 *   +0x08  level  — low 8 bits hold 1..62; bit0 also acts as enable
 *
 * Init (matches U-Boot / panel bring-up): write 62 to +0x08, then set bit0
 * on +0x04 and +0x08. Brightness 0 = off; userspace 1..max → HW 1..62.
 *
 * This is MMIO, not the Dialog PMIC, and that is from the decomp rather
 * than from anyone's recollection:
 *
 *   - The stock bootloader ends its display bring-up with
 *     sub_3522(0x3E000008, 62) -- sub_3522 is its plain MMIO write
 *     helper, the same one used for CLKCON and IIC -- immediately after
 *     the 0x3D7000xx LCDIF/DSI block and just before 0x3D700008 = 1.
 *     62 is exactly this register's full-scale level.
 *   - OSOS sub_4399FC sets 0x3E000008 bit 0 when SoC power domain 2
 *     comes up, and domain 2 is the LCD domain. So the domain gates the
 *     block; it does not supply the level.
 *
 * No PMIC register anywhere in the D1830 map carries brightness. A rail
 * feeding the LED string would be a separate question, but nothing in the
 * PMIC state analysis names one, so do not assume it either way.
 *
 * Kconfig fragment (wire Makefile / Kconfig separately):
 *   config BACKLIGHT_S5L8740
 *   	tristate "Samsung/Apple S5L8740 backlight"
 *   	depends on BACKLIGHT_CLASS_DEVICE && (ARCH_S5L8740 || COMPILE_TEST)
 *   	default y if ARCH_S5L8740
 *   	help
 *   	  LCD backlight at 0x3E000000 for iPod nano 7G (N31).
 */
#include <linux/backlight.h>
#include <linux/apple-n31.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#define S5L8740_BL_ENABLE_OFF	0x04
#define S5L8740_BL_LEVEL_OFF	0x08
#define S5L8740_BL_MAX		62

/*
 * The LED boost is the expensive part of the display, so screen sleep
 * ramps it rather than cutting it. Stepping happens in a work item so a
 * caller in a button handler does not block for the length of the fade.
 */
struct s5l8740_bl {
	void __iomem *base;
	struct backlight_device *bd;
	struct delayed_work fade;
	struct mutex lock;
	int level;		/* what the hardware currently has */
	int target;
	unsigned int step_ms;
};

static struct s5l8740_bl *s5l8740_bl_dev;

static void s5l8740_bl_hw_set(struct s5l8740_bl *bl, int level)
{
	u32 en, lvl;

	if (level > S5L8740_BL_MAX)
		level = S5L8740_BL_MAX;
	bl->level = level > 0 ? level : 0;

	if (level <= 0) {
		en = readl(bl->base + S5L8740_BL_ENABLE_OFF);
		writel(en & ~BIT(0), bl->base + S5L8740_BL_ENABLE_OFF);
		lvl = readl(bl->base + S5L8740_BL_LEVEL_OFF);
		writel(lvl & ~BIT(0), bl->base + S5L8740_BL_LEVEL_OFF);
		return;
	}

	writel((u32)level, bl->base + S5L8740_BL_LEVEL_OFF);
	en = readl(bl->base + S5L8740_BL_ENABLE_OFF);
	writel(en | BIT(0), bl->base + S5L8740_BL_ENABLE_OFF);
	lvl = readl(bl->base + S5L8740_BL_LEVEL_OFF);
	writel(lvl | BIT(0), bl->base + S5L8740_BL_LEVEL_OFF);
}

static void s5l8740_bl_fade_work(struct work_struct *work)
{
	struct s5l8740_bl *bl = container_of(to_delayed_work(work),
					     struct s5l8740_bl, fade);
	bool more;

	mutex_lock(&bl->lock);
	if (bl->level < bl->target)
		s5l8740_bl_hw_set(bl, bl->level + 1);
	else if (bl->level > bl->target)
		s5l8740_bl_hw_set(bl, bl->level - 1);
	more = bl->level != bl->target;
	mutex_unlock(&bl->lock);

	if (more)
		schedule_delayed_work(&bl->fade,
				      msecs_to_jiffies(bl->step_ms));
}

/*
 * Ramp to `level` over roughly `ms`. ms = 0 jumps straight there, which is
 * what the backlight class writes should do. Returns the level that was
 * programmed before the fade started, so a caller can restore it on wake.
 */
int n31_backlight_fade(int level, unsigned int ms)
{
	struct s5l8740_bl *bl = s5l8740_bl_dev;
	int previous, delta;

	if (!bl)
		return -ENODEV;
	if (level < 0)
		level = 0;
	if (level > S5L8740_BL_MAX)
		level = S5L8740_BL_MAX;

	cancel_delayed_work_sync(&bl->fade);
	mutex_lock(&bl->lock);
	previous = bl->level;
	bl->target = level;
	delta = abs(level - bl->level);
	if (!ms || !delta) {
		s5l8740_bl_hw_set(bl, level);
		mutex_unlock(&bl->lock);
		return previous;
	}
	bl->step_ms = max_t(unsigned int, ms / delta, 1);
	mutex_unlock(&bl->lock);

	schedule_delayed_work(&bl->fade, msecs_to_jiffies(bl->step_ms));
	return previous;
}
EXPORT_SYMBOL_GPL(n31_backlight_fade);

/* Level the hardware currently has, ignoring any fade in progress. */
int n31_backlight_level(void)
{
	struct s5l8740_bl *bl = s5l8740_bl_dev;

	return bl ? bl->level : -ENODEV;
}
EXPORT_SYMBOL_GPL(n31_backlight_level);

static int s5l8740_bl_update_status(struct backlight_device *bd)
{
	struct s5l8740_bl *bl = bl_get_data(bd);
	int brightness = backlight_get_brightness(bd);

	cancel_delayed_work_sync(&bl->fade);
	mutex_lock(&bl->lock);
	bl->target = brightness;
	s5l8740_bl_hw_set(bl, brightness);
	mutex_unlock(&bl->lock);
	return 0;
}

static const struct backlight_ops s5l8740_bl_ops = {
	.update_status = s5l8740_bl_update_status,
};

static int s5l8740_bl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l8740_bl *bl;
	struct backlight_properties props = { };
	struct backlight_device *bd;

	bl = devm_kzalloc(dev, sizeof(*bl), GFP_KERNEL);
	if (!bl)
		return -ENOMEM;

	bl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bl->base))
		return PTR_ERR(bl->base);

	mutex_init(&bl->lock);
	INIT_DELAYED_WORK(&bl->fade, s5l8740_bl_fade_work);
	bl->target = S5L8740_BL_MAX;

	/* Full brightness + enables (U-Boot path) */
	s5l8740_bl_hw_set(bl, S5L8740_BL_MAX);

	props.type = BACKLIGHT_RAW;
	props.max_brightness = S5L8740_BL_MAX;
	props.brightness = S5L8740_BL_MAX;

	bd = devm_backlight_device_register(dev, "s5l8740-backlight", dev, bl,
					   &s5l8740_bl_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	bl->bd = bd;
	s5l8740_bl_dev = bl;
	platform_set_drvdata(pdev, bl);
	dev_info(dev, "S5L8740 backlight @%pR max=%u\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0), S5L8740_BL_MAX);
	return 0;
}

static const struct of_device_id s5l8740_bl_of_match[] = {
	{ .compatible = "apple,s5l8740-backlight" },
	{ .compatible = "samsung,s5l8740-backlight" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8740_bl_of_match);

static void s5l8740_bl_remove(struct platform_device *pdev)
{
	struct s5l8740_bl *bl = platform_get_drvdata(pdev);

	s5l8740_bl_dev = NULL;
	cancel_delayed_work_sync(&bl->fade);
}

static struct platform_driver s5l8740_bl_driver = {
	.probe = s5l8740_bl_probe,
	.remove = s5l8740_bl_remove,
	.driver = {
		.name = "backlight-s5l8740",
		.of_match_table = s5l8740_bl_of_match,
	},
};
module_platform_driver(s5l8740_bl_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8740 LCD backlight");
MODULE_LICENSE("GPL");
