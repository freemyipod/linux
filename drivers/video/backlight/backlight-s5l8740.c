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
 * Kconfig fragment (wire Makefile / Kconfig separately):
 *   config BACKLIGHT_S5L8740
 *   	tristate "Samsung/Apple S5L8740 backlight"
 *   	depends on BACKLIGHT_CLASS_DEVICE && (ARCH_S5L8740 || COMPILE_TEST)
 *   	default y if ARCH_S5L8740
 *   	help
 *   	  LCD backlight at 0x3E000000 for iPod nano 7G (N31).
 */
#include <linux/backlight.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define S5L8740_BL_ENABLE_OFF	0x04
#define S5L8740_BL_LEVEL_OFF	0x08
#define S5L8740_BL_MAX		62

struct s5l8740_bl {
	void __iomem *base;
	struct backlight_device *bd;
};

static void s5l8740_bl_hw_set(struct s5l8740_bl *bl, int level)
{
	u32 en, lvl;

	if (level <= 0) {
		en = readl(bl->base + S5L8740_BL_ENABLE_OFF);
		writel(en & ~BIT(0), bl->base + S5L8740_BL_ENABLE_OFF);
		lvl = readl(bl->base + S5L8740_BL_LEVEL_OFF);
		writel(lvl & ~BIT(0), bl->base + S5L8740_BL_LEVEL_OFF);
		return;
	}

	if (level > S5L8740_BL_MAX)
		level = S5L8740_BL_MAX;

	writel((u32)level, bl->base + S5L8740_BL_LEVEL_OFF);
	en = readl(bl->base + S5L8740_BL_ENABLE_OFF);
	writel(en | BIT(0), bl->base + S5L8740_BL_ENABLE_OFF);
	lvl = readl(bl->base + S5L8740_BL_LEVEL_OFF);
	writel(lvl | BIT(0), bl->base + S5L8740_BL_LEVEL_OFF);
}

static int s5l8740_bl_update_status(struct backlight_device *bd)
{
	struct s5l8740_bl *bl = bl_get_data(bd);
	int brightness = backlight_get_brightness(bd);

	s5l8740_bl_hw_set(bl, brightness);
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

static struct platform_driver s5l8740_bl_driver = {
	.probe = s5l8740_bl_probe,
	.driver = {
		.name = "backlight-s5l8740",
		.of_match_table = s5l8740_bl_of_match,
	},
};
module_platform_driver(s5l8740_bl_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8740 LCD backlight");
MODULE_LICENSE("GPL");
