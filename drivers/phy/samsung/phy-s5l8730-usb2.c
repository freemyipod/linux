#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>

struct s5l8730_usbphy {
    struct device *dev;
    struct phy *phy;
    void __iomem *base;
};

#define S5L8730_OTGPHY_PWR 0x00
#define S5L8730_OTGPHY_CON 0x04
#define S5L8730_OTGPHY_RSTCON 0x08
#define S5L8730_OTGPHY_UNKCON 0x1C

static int s5l8730_usbphy_phy_init(struct phy *phy)
{
    printk("%s...\n", __func__);
    return 0;
}

static int s5l8730_usbphy_phy_exit(struct phy *phy)
{
    printk("%s...\n", __func__);
    return 0;
}

static int s5l8730_usbphy_phy_power_on(struct phy *phy)
{
	struct s5l8730_usbphy *usbphy = phy_get_drvdata(phy);
    printk("%s...\n", __func__);

	writel_relaxed(0, usbphy->base + S5L8730_OTGPHY_PWR);
    mdelay(10);
	writel_relaxed(1, usbphy->base + S5L8730_OTGPHY_RSTCON);
    mdelay(10);
	writel_relaxed(0, usbphy->base + S5L8730_OTGPHY_RSTCON);
    mdelay(10);
	writel_relaxed(6, usbphy->base + S5L8730_OTGPHY_UNKCON);
	writel_relaxed(1, usbphy->base + S5L8730_OTGPHY_CON);
    mdelay(400);
    return 0;
}

static int s5l8730_usbphy_phy_power_off(struct phy *phy)
{
	struct s5l8730_usbphy *usbphy = phy_get_drvdata(phy);
    printk("%s...\n", __func__);

    writel_relaxed(0xff, usbphy->base + S5L8730_OTGPHY_PWR);
    mdelay(10);
    writel_relaxed(0xff, usbphy->base + S5L8730_OTGPHY_RSTCON);
    mdelay(10);
    writel_relaxed(4, usbphy->base + S5L8730_OTGPHY_UNKCON);
    return 0;
}


static const struct phy_ops s5l8730_usbphy_phy_ops = {
    .init = s5l8730_usbphy_phy_init,
    .exit = s5l8730_usbphy_phy_exit,
    .power_on = s5l8730_usbphy_phy_power_on,
    .power_off = s5l8730_usbphy_phy_power_off,
    .owner = THIS_MODULE,
};

static int s5l8730_usbphy_probe(struct platform_device *pdev)
{
	struct s5l8730_usbphy *usbphy;
    struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
    struct device_node *np = dev->of_node;
    int ret;

    printk("%s...\n", __func__);

    usbphy = devm_kzalloc(dev, sizeof(*usbphy), GFP_KERNEL);
    if (!usbphy)
        return -ENOMEM;
    usbphy->dev = dev;
    dev_set_drvdata(dev, usbphy);

    usbphy->base = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(usbphy->base))
        return PTR_ERR(usbphy->base);

    usbphy->phy = devm_phy_create(dev, NULL, &s5l8730_usbphy_phy_ops);
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

static int s5l8730_usbphy_remove(struct platform_device *pdev)
{
    printk("%s...\n", __func__);
    return 0;
}

static const struct of_device_id s5l8730_usbphy_of_match[] = {
    { .compatible = "apple,s5l8730-otgphy", },
    { },
};
MODULE_DEVICE_TABLE(of, s5l8730_usbphy_of_match);

static struct platform_driver s5l8730_usbphy_driver = {
	.probe = s5l8730_usbphy_probe,
	.remove = s5l8730_usbphy_remove,
	.driver = {
		.of_match_table = s5l8730_usbphy_of_match,
		.name = "s5l8730-usbphy",
		//.pm = &stm32_usbphyc_pm_ops,
	}
};
module_platform_driver(s5l8730_usbphy_driver);

