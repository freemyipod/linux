// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 PRNG accelerator
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define S5L8702_PRNG_CONF	0x00
#define S5L8702_PRNG_DATA	0x04
#define S5L8702_PRNG_SEED	0x08

#define S5L8702_PRNG_CONF_READY GENMASK(2, 0)

struct s5l8702_prng_dev {
	struct device *dev;
	void __iomem *regs;
	void __iomem *clk_reg;
	struct hwrng rng;
	bool seeded;
};

static inline void s5l8702_prng_writel(struct s5l8702_prng_dev *prng_dev,
					  u32 reg, u32 val)
{
	writel(val, prng_dev->regs + reg);
}

static inline u32 s5l8702_prng_readl(struct s5l8702_prng_dev *prng_dev, u32 reg)
{
	return readl(prng_dev->regs + reg);
}

static void s5l8702_prng_enable_clockgate(struct s5l8702_prng_dev *prng_dev)
{
	// TODO clk_prepare_enable()
	writel(readl(prng_dev->clk_reg) & ~BIT(0), prng_dev->clk_reg);
}

static void s5l8702_prng_disable_clockgate(struct s5l8702_prng_dev *prng_dev)
{
	// TODO clk_disable_unprepare()
	writel(readl(prng_dev->clk_reg) | BIT(0), prng_dev->clk_reg);
}

static int s5l8702_prng_get_data(struct s5l8702_prng_dev *prng_dev, u32 *data, bool wait)
{
	int ret;
	u32 conf;

	ret = readl_poll_timeout(prng_dev->regs + S5L8702_PRNG_CONF, conf,
			 conf & S5L8702_PRNG_CONF_READY, 10, wait ? 1000 : 0);

	if (ret) {
		return ret;
	}

	*data = readl(prng_dev->regs + S5L8702_PRNG_DATA);

	return 0;
}

static int s5l8702_prng_init(struct hwrng *rng)
{
	struct s5l8702_prng_dev *prng_dev = container_of(rng, struct s5l8702_prng_dev, rng);
	u32 seed;
	u32 data;
	int i, ret;

	s5l8702_prng_enable_clockgate(prng_dev);

	// software reset
	s5l8702_prng_writel(prng_dev, S5L8702_PRNG_CONF, 0);

	if (!prng_dev->seeded) {
		get_random_bytes(&seed, sizeof(seed));
		s5l8702_prng_writel(prng_dev, S5L8702_PRNG_SEED, seed);

		for (i = 1; i < 20; i++) {
			ret = s5l8702_prng_get_data(prng_dev, &data, true);

			if (ret) {
				return ret;
			}
		}

		dev_info(prng_dev->dev, "S5L8702 PRNG seeded\n");
		prng_dev->seeded = true;
	}

	return 0;
}

static void s5l8702_prng_cleanup(struct hwrng *rng)
{
	struct s5l8702_prng_dev *prng_dev = container_of(rng, struct s5l8702_prng_dev, rng);

	s5l8702_prng_disable_clockgate(prng_dev);
}

static int s5l8702_prng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct s5l8702_prng_dev *prng_dev = container_of(rng, struct s5l8702_prng_dev, rng);

	u32 *data = buf;
	size_t words;
	size_t i;
	int ret;

	words = max / sizeof(u32);

	for (i = 0; i < words; i++) {
		ret = s5l8702_prng_get_data(prng_dev, &data[i], wait);

		if (ret) {
			if (i) {
				dev_err(prng_dev->dev, "timeout waiting for random data, returning only %zu bytes\n", i);
				return i * sizeof(u32);
			}

			dev_err(prng_dev->dev, "timeout waiting for random data\n");
			return ret;
		}

		dev_info(prng_dev->dev, "generated 0x%08X\n", data[i]);
	}

	dev_info(prng_dev->dev, "generated %zu random words\n", words);
	return words * sizeof(u32);
}

static int s5l8702_prng_probe(struct platform_device *pdev)
{
	struct s5l8702_prng_dev *prng_dev;
	struct device *dev = &pdev->dev;
	int ret;

	prng_dev = devm_kzalloc(dev, sizeof(*prng_dev), GFP_KERNEL);
	if (!prng_dev) {
		return -ENOMEM;
	}

	prng_dev->dev = dev;

	prng_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(prng_dev->regs)) {
		return PTR_ERR(prng_dev->regs);
	}

	// TODO: samsung_clk_register_gate() or similar
	prng_dev->clk_reg = devm_ioremap(dev, 0x3C50004C, 4);
	if (!prng_dev->clk_reg) {
		return -ENOMEM;
	}

	prng_dev->rng.name = pdev->name;
	prng_dev->rng.init = s5l8702_prng_init;
	prng_dev->rng.cleanup = s5l8702_prng_cleanup;
	prng_dev->rng.read = s5l8702_prng_read;

	ret = devm_hwrng_register(dev, &prng_dev->rng);
	if (ret) {
		return dev_err_probe(dev, ret, "Failed to register hwrng\n");
	}

	platform_set_drvdata(pdev, prng_dev);

	dev_info(dev, "S5L8702 PRNG accelerator registered\n");

	return 0;
}

static const struct of_device_id s5l8702_prng_of_match[] = {
	{ .compatible = "samsung,s5l8702-prng" },
	{}
};
MODULE_DEVICE_TABLE(of, s5l8702_prng_of_match);

static struct platform_driver s5l8702_prng_driver = {
	.probe		= s5l8702_prng_probe,
	.driver		= {
		.name	= "s5l8702-prng",
		.of_match_table = s5l8702_prng_of_match,
	},
};

module_platform_driver(s5l8702_prng_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_AUTHOR("Sylvia Petzanova <s.petzanova@gmail.com>");
MODULE_DESCRIPTION("S5L8702 PRNG accelerator");
MODULE_LICENSE("GPL v2");
