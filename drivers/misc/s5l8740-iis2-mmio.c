// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 IIS2 MMIO hook — FM digital RX @ 0x3D400000 (N31 RE).
 * Register model OPEN: probe + regs sysfs only, no invented capture PCM.
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#define IIS2_MMIO_LEN	0x40

struct s5l8740_iis2 {
	void __iomem *base;
	struct clk_bulk_data *clks;
	int num_clks;
};

static ssize_t regs_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct s5l8740_iis2 *iis2 = dev_get_drvdata(dev);
	unsigned int i;
	ssize_t n = 0;

	if (!iis2 || !iis2->base)
		return sysfs_emit(buf, "not mapped\n");

	for (i = 0; i < IIS2_MMIO_LEN; i += 4) {
		n += sysfs_emit_at(buf, n, "%02x: %08x\n", i,
				   readl(iis2->base + i));
		if (n >= PAGE_SIZE - 32)
			break;
	}
	return n;
}
static DEVICE_ATTR_RO(regs);

static struct attribute *iis2_attrs[] = {
	&dev_attr_regs.attr,
	NULL,
};
static const struct attribute_group iis2_attr_group = {
	.attrs = iis2_attrs,
};

static int s5l8740_iis2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l8740_iis2 *iis2;
	struct resource *res;
	int ret;

	iis2 = devm_kzalloc(dev, sizeof(*iis2), GFP_KERNEL);
	if (!iis2)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iis2->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(iis2->base))
		return PTR_ERR(iis2->base);

	ret = devm_clk_bulk_get_all(dev, &iis2->clks);
	if (ret > 0) {
		iis2->num_clks = ret;
		clk_bulk_prepare_enable(iis2->num_clks, iis2->clks);
	}

	ret = sysfs_create_group(&dev->kobj, &iis2_attr_group);
	if (ret)
		dev_warn(dev, "sysfs: %d\n", ret);

	dev_set_drvdata(dev, iis2);
	dev_info(dev, "IIS2 FM hook @%pR — regs sysfs; capture PCM OPEN\n", res);
	return 0;
}

static void s5l8740_iis2_remove(struct platform_device *pdev)
{
	struct s5l8740_iis2 *iis2 = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &iis2_attr_group);
	if (iis2 && iis2->num_clks)
		clk_bulk_disable_unprepare(iis2->num_clks, iis2->clks);
}

static const struct of_device_id s5l8740_iis2_of_match[] = {
	{ .compatible = "apple,s5l8740-iis2" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8740_iis2_of_match);

static struct platform_driver s5l8740_iis2_driver = {
	.probe = s5l8740_iis2_probe,
	.remove = s5l8740_iis2_remove,
	.driver = {
		.name = "s5l8740-iis2",
		.of_match_table = s5l8740_iis2_of_match,
	},
};
module_platform_driver(s5l8740_iis2_driver);

MODULE_DESCRIPTION("S5L8740 IIS2 FM MMIO hook (N31)");
MODULE_LICENSE("GPL");
