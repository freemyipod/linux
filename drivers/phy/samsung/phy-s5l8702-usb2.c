// SPDX-License-Identifier: GPL-2.0+
/*
 * Apple/Samsung S5L8702 / S5L87xx USB OTG PHY.
 *
 * S5L8702 (Nano 3G) uses the analog stage ramp sequence.
 * S5L8723/8740 (Nano 6G/7G) use the shorter Freemyipod/U-Boot
 * s5l87xx sequence — N31 must NOT run the 8702 dance or the
 * Lightning-facing link dies while DWC2 still loads.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>

enum s5l_usbphy_kind {
	S5L_USBPHY_8702 = 0,
	S5L_USBPHY_87XX = 1,
};

struct s5l8702_usbphy {
	struct device *dev;
	struct phy *phy;
	void __iomem *base;
	enum s5l_usbphy_kind kind;
};

#define S5L8702_OTGPHY_PWR	0x000
#define S5L8702_OTGPHY_CLK	0x004
#define S5L8702_OTGPHY_RSTCON	0x008
#define S5L8702_OTGPHY_BIAS	0x018
#define S5L8702_OTGPHY_MODE	0x01c	/* s5l87xx unkcon */
#define S5L8702_OTGPHY_INTFCON	0x030
#define S5L8702_OTGPHY_CTRL1	0x040
#define S5L8702_OTGPHY_CTRL2	0x044
#define S5L8702_OTGPHY_ENABLE	0x100

/* DWC2 PCGCCTL — clear USB suspend before PHY on (U-Boot does this) */
#define S5L87XX_OTG_PCGCCTL	0x38400e00
/* DWC2 DCTL — drop U-Boot D+ before PHY reset or Windows sees 0000:0002 */
#define S5L87XX_OTG_DCTL	0x38400804
#define S5L87XX_DCTL_SFTDISCON	BIT(1)

static int s5l8702_usbphy_phy_init(struct phy *phy)
{
	return 0;
}

static int s5l8702_usbphy_phy_exit(struct phy *phy)
{
	return 0;
}

static void s5l87xx_clear_suspend(struct s5l8702_usbphy *usbphy)
{
	void __iomem *pcgc;

	pcgc = ioremap(S5L87XX_OTG_PCGCCTL, 4);
	if (!pcgc) {
		dev_warn(usbphy->dev, "PCGCCTL ioremap failed\n");
		return;
	}
	writel(0, pcgc);
	iounmap(pcgc);
}

static void s5l87xx_soft_disconnect(struct s5l8702_usbphy *usbphy)
{
	void __iomem *dctl;
	u32 val;

	dctl = ioremap(S5L87XX_OTG_DCTL, 4);
	if (!dctl) {
		dev_warn(usbphy->dev, "DCTL ioremap failed\n");
		return;
	}
	val = readl(dctl) | S5L87XX_DCTL_SFTDISCON;
	writel(val, dctl);
	iounmap(dctl);
}

/*
 * Matches upstream/u-boot arch/arm/mach-s5l87xx/s5l87xx-otg-phy.c
 * and is the sequence that already proves DFU gadget on N31.
 */
static int s5l87xx_usbphy_power_on(struct s5l8702_usbphy *usbphy)
{
	void __iomem *b = usbphy->base;

	/* U-Boot DFU leaves D+ pulled up. Drop it before PHY reset. */
	s5l87xx_soft_disconnect(usbphy);
	s5l87xx_clear_suspend(usbphy);
	mdelay(10);

	writel(0, b + S5L8702_OTGPHY_PWR);
	mdelay(10);
	writel(1, b + S5L8702_OTGPHY_RSTCON);
	mdelay(10);
	writel(0, b + S5L8702_OTGPHY_RSTCON);
	mdelay(10);
	writel(6, b + S5L8702_OTGPHY_MODE);
	writel(1, b + S5L8702_OTGPHY_CLK); /* con @ +0x04 */
	/* U-Boot waits ~400ms for PLL lock */
	mdelay(400);

	dev_info(usbphy->dev, "s5l87xx OTG PHY on (N31/N20 path)\n");
	return 0;
}

static int s5l87xx_usbphy_power_off(struct s5l8702_usbphy *usbphy)
{
	void __iomem *b = usbphy->base;

	writel(0xff, b + S5L8702_OTGPHY_PWR);
	mdelay(10);
	writel(0xff, b + S5L8702_OTGPHY_RSTCON);
	mdelay(10);
	writel(4, b + S5L8702_OTGPHY_MODE);
	return 0;
}

static int s5l8702_usbphy_power_on_legacy(struct s5l8702_usbphy *usbphy)
{
	void __iomem *b = usbphy->base;

	writel(0x000, b + S5L8702_OTGPHY_PWR);
	writel(0x000, b + S5L8702_OTGPHY_CLK);
	writel(0x400, b + S5L8702_OTGPHY_BIAS);
	writel(0x007, b + S5L8702_OTGPHY_RSTCON);

	writel(0x300, b + S5L8702_OTGPHY_CTRL1);
	writel(0x340, b + S5L8702_OTGPHY_CTRL1);
	writel(0x346, b + S5L8702_OTGPHY_CTRL1);
	writel(0x347, b + S5L8702_OTGPHY_CTRL1);

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

	mdelay(40);
	return 0;
}

static int s5l8702_usbphy_power_off_legacy(struct s5l8702_usbphy *usbphy)
{
	void __iomem *b = usbphy->base;

	writel(0x0, b + S5L8702_OTGPHY_CTRL2);
	writel(0x0, b + S5L8702_OTGPHY_CTRL1);
	writel(0x7, b + S5L8702_OTGPHY_RSTCON);
	writel(0xff, b + S5L8702_OTGPHY_PWR);
	return 0;
}

static int s5l8702_usbphy_phy_power_on(struct phy *phy)
{
	struct s5l8702_usbphy *usbphy = phy_get_drvdata(phy);

	if (usbphy->kind == S5L_USBPHY_87XX)
		return s5l87xx_usbphy_power_on(usbphy);
	return s5l8702_usbphy_power_on_legacy(usbphy);
}

static int s5l8702_usbphy_phy_power_off(struct phy *phy)
{
	struct s5l8702_usbphy *usbphy = phy_get_drvdata(phy);

	if (usbphy->kind == S5L_USBPHY_87XX)
		return s5l87xx_usbphy_power_off(usbphy);
	return s5l8702_usbphy_power_off_legacy(usbphy);
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
	const struct of_device_id *match;
	int ret;

	usbphy = devm_kzalloc(dev, sizeof(*usbphy), GFP_KERNEL);
	if (!usbphy)
		return -ENOMEM;
	usbphy->dev = dev;
	dev_set_drvdata(dev, usbphy);

	match = of_match_device(dev->driver->of_match_table, dev);
	if (match && match->data)
		usbphy->kind = (uintptr_t)match->data;
	else
		usbphy->kind = S5L_USBPHY_8702;

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

	dev_info(dev, "USB OTG PHY kind=%s\n",
		 usbphy->kind == S5L_USBPHY_87XX ? "s5l87xx" : "s5l8702");

	return PTR_ERR_OR_ZERO(phy_provider);
}

static void s5l8702_usbphy_remove(struct platform_device *pdev)
{
}

static const struct of_device_id s5l8702_usbphy_of_match[] = {
	{ .compatible = "apple,s5l8702-otgphy",
	  .data = (void *)(uintptr_t)S5L_USBPHY_8702 },
	{ .compatible = "apple,s5l87xx-otgphy",
	  .data = (void *)(uintptr_t)S5L_USBPHY_87XX },
	{ .compatible = "apple,s5l8740-otgphy",
	  .data = (void *)(uintptr_t)S5L_USBPHY_87XX },
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

MODULE_DESCRIPTION("Apple/Samsung S5L8702/S5L87xx USB OTG PHY");
MODULE_LICENSE("GPL");
