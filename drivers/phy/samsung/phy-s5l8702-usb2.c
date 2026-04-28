// SPDX-License-Identifier: GPL-2.0+
/*
 * Apple/Samsung S5L8702 USB OTG PHY.
 *
 * Register layout and power-on sequence match u-boot's
 * arch/arm/mach-s5l87xx/s5l87xx.c (reverse-engineered from a disk-mode QEMU
 * trace and confirmed working for DFU). The PHY enable bit lives at offset
 * 0x100, so the mapped region must cover at least 0x104 bytes.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/io.h>

struct s5l8702_usbphy {
	struct device *dev;
	struct phy *phy;
	void __iomem *base;
};

#define S5L8702_OTGPHY_PWR	0x000
#define S5L8702_OTGPHY_CLK	0x004
#define S5L8702_OTGPHY_RSTCON	0x008
#define S5L8702_OTGPHY_BIAS	0x018
#define S5L8702_OTGPHY_INTFCON	0x030
#define S5L8702_OTGPHY_CTRL1	0x040
#define S5L8702_OTGPHY_CTRL2	0x044
#define S5L8702_OTGPHY_ENABLE	0x100

static int s5l8702_usbphy_phy_init(struct phy *phy)
{
	return 0;
}

static int s5l8702_usbphy_phy_exit(struct phy *phy)
{
	return 0;
}

static int s5l8702_usbphy_phy_power_on(struct phy *phy)
{
	struct s5l8702_usbphy *usbphy = phy_get_drvdata(phy);
	void __iomem *b = usbphy->base;

	writel(0x000, b + S5L8702_OTGPHY_PWR);
	writel(0x000, b + S5L8702_OTGPHY_CLK);
	writel(0x400, b + S5L8702_OTGPHY_BIAS);
	writel(0x007, b + S5L8702_OTGPHY_RSTCON);

	/* Analog stage 1 ramp */
	writel(0x300, b + S5L8702_OTGPHY_CTRL1);
	writel(0x340, b + S5L8702_OTGPHY_CTRL1);
	writel(0x346, b + S5L8702_OTGPHY_CTRL1);
	writel(0x347, b + S5L8702_OTGPHY_CTRL1);

	/* Analog stage 2 ramp */
	writel(0x0c00, b + S5L8702_OTGPHY_CTRL2);
	writel(0x0fc0, b + S5L8702_OTGPHY_CTRL2);
	writel(0x0fe0, b + S5L8702_OTGPHY_CTRL2);
	writel(0x0ff0, b + S5L8702_OTGPHY_CTRL2);
	writel(0x0fff, b + S5L8702_OTGPHY_CTRL2);

	writel(1, b + S5L8702_OTGPHY_ENABLE);

	writel(0x000, b + S5L8702_OTGPHY_RSTCON);
	writel(0x400, b + S5L8702_OTGPHY_BIAS);
	writel(0x000, b + S5L8702_OTGPHY_INTFCON);
	writel(0x000, b + S5L8702_OTGPHY_BIAS);

	/* Let the PLL lock before the controller starts poking the core. */
	mdelay(40);
	return 0;
}

static int s5l8702_usbphy_phy_power_off(struct phy *phy)
{
	struct s5l8702_usbphy *usbphy = phy_get_drvdata(phy);
	void __iomem *b = usbphy->base;

	writel(0x0, b + S5L8702_OTGPHY_CTRL2);
	writel(0x0, b + S5L8702_OTGPHY_CTRL1);
	writel(0x7, b + S5L8702_OTGPHY_RSTCON);
	writel(0xff, b + S5L8702_OTGPHY_PWR);
	return 0;
}

static const struct phy_ops s5l8702_usbphy_phy_ops = {
	.init = s5l8702_usbphy_phy_init,
	.exit = s5l8702_usbphy_phy_exit,
	.power_on = s5l8702_usbphy_phy_power_on,
	.power_off = s5l8702_usbphy_phy_power_off,
	.owner = THIS_MODULE,
};

static int s5l8702_usbphy_probe(struct platform_device *pdev)
{
	struct s5l8702_usbphy *usbphy;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	int ret;

	usbphy = devm_kzalloc(dev, sizeof(*usbphy), GFP_KERNEL);
	if (!usbphy)
		return -ENOMEM;
	usbphy->dev = dev;
	dev_set_drvdata(dev, usbphy);

	usbphy->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(usbphy->base))
		return PTR_ERR(usbphy->base);

	usbphy->phy = devm_phy_create(dev, NULL, &s5l8702_usbphy_phy_ops);
	if (IS_ERR(usbphy->phy)) {
		ret = PTR_ERR(usbphy->phy);
		dev_err(dev, "failed to create phy: %d\n", ret);
		return ret;
	}

	phy_set_drvdata(usbphy->phy, usbphy);
	phy_provider = devm_of_phy_provider_register(&pdev->dev,
						     of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static void s5l8702_usbphy_remove(struct platform_device *pdev)
{
}

static const struct of_device_id s5l8702_usbphy_of_match[] = {
	{ .compatible = "apple,s5l8702-otgphy", },
	{ },
};
MODULE_DEVICE_TABLE(of, s5l8702_usbphy_of_match);

static struct platform_driver s5l8702_usbphy_driver = {
	.probe = s5l8702_usbphy_probe,
	.remove = s5l8702_usbphy_remove,
	.driver = {
		.of_match_table = s5l8702_usbphy_of_match,
		.name = "s5l8702-usbphy",
	}
};
module_platform_driver(s5l8702_usbphy_driver);
