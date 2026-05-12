// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 PRNG accelerator
 *
 * Note: This driver returns raw hardware output. If using for
 * cryptographic purposes, callers should consider discarding the
 * first 20 outputs after seeding to avoid potential weak/correlated
 * values.
 */

#include <crypto/internal/rng.h>
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define S5L8702_PRNG_CONF	0x00
#define S5L8702_PRNG_DATA	0x04
#define S5L8702_PRNG_SEED	0x08

#define S5L8702_PRNG_CONF_READY GENMASK(2, 0)

struct s5l8702_prng_dev {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct mutex lock;
};

struct s5l8702_prng_ctx {
	struct s5l8702_prng_dev *prng_dev;
};

struct s5l8702_prng_dev *prng_dev_global;

static inline void s5l8702_prng_writel(struct s5l8702_prng_dev *prng_dev,
				       u32 reg, u32 val)
{
	writel(val, prng_dev->regs + reg);
}

static inline u32 s5l8702_prng_readl(struct s5l8702_prng_dev *prng_dev, u32 reg)
{
	return readl(prng_dev->regs + reg);
}

static int s5l8702_prng_get_data(struct s5l8702_prng_dev *prng_dev, u32 *data)
{
	int ret;
	u32 conf;

	ret = readl_poll_timeout(prng_dev->regs + S5L8702_PRNG_CONF, conf,
				 conf & S5L8702_PRNG_CONF_READY, 10, 1000);

	if (ret) {
		return ret;
	}

	*data = readl(prng_dev->regs + S5L8702_PRNG_DATA);

	return 0;
}

static int s5l8702_prng_cra_init(struct crypto_tfm *tfm)
{
	struct s5l8702_prng_ctx *ctx = crypto_tfm_ctx(tfm);
	int ret;

	if (WARN_ON(!prng_dev_global)) {
		return -ENODEV;
	}
	ctx->prng_dev = prng_dev_global;

	ret = clk_prepare_enable(ctx->prng_dev->clk);
	if (ret) {
		dev_err(ctx->prng_dev->dev, "clk_prepare_enable failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static void s5l8702_prng_cra_exit(struct crypto_tfm *tfm)
{
	struct s5l8702_prng_ctx *ctx = crypto_tfm_ctx(tfm);
	struct s5l8702_prng_dev *prng_dev = ctx->prng_dev;

	if (WARN_ON(!prng_dev)) {
        return;
    }

	clk_disable_unprepare(prng_dev->clk);
}

static int s5l8702_prng_seed(struct crypto_rng *tfm, const u8 *seed, unsigned int slen)
{
	struct s5l8702_prng_ctx *ctx = crypto_rng_ctx(tfm);
	struct s5l8702_prng_dev *prng_dev = ctx->prng_dev;
	u32 seed_val;

	if (slen < sizeof(u32) || WARN_ON(!prng_dev)) {
		return -EINVAL;
	}

	mutex_lock(&prng_dev->lock);

	s5l8702_prng_writel(prng_dev, S5L8702_PRNG_CONF, 0);
	memcpy(&seed_val, seed, sizeof(seed_val));
	s5l8702_prng_writel(prng_dev, S5L8702_PRNG_SEED, seed_val);

	mutex_unlock(&prng_dev->lock);

	dev_dbg(prng_dev->dev, "S5L8702 PRNG seeded (%u bytes)\n", slen);

	return 0;
}

static int s5l8702_prng_generate(struct crypto_rng *tfm, const u8 *src, unsigned int slen,
				  u8 *rdata, unsigned int dlen)
{
	struct s5l8702_prng_ctx *ctx = crypto_rng_ctx(tfm);
	struct s5l8702_prng_dev *prng_dev = ctx->prng_dev;
	u32 *data = (u32 *)rdata;
	size_t words = dlen / sizeof(u32);
	size_t i;
	int ret;

	if (!words || WARN_ON(!prng_dev)) {
        return 0;
    }

	mutex_lock(&prng_dev->lock);

	// check if seed is provided
	if (src && slen >= sizeof(u32)) {
		u32 seed_val;
		memcpy(&seed_val, src, sizeof(seed_val));
		s5l8702_prng_writel(prng_dev, S5L8702_PRNG_SEED, seed_val);
	}

	for (i = 0; i < words; i++) {
		ret = s5l8702_prng_get_data(prng_dev, &data[i]);
		if (ret) {
			size_t generated = i * sizeof(u32);
			mutex_unlock(&prng_dev->lock);
			return generated ? (int)generated : ret;
		}
	}

	mutex_unlock(&prng_dev->lock);

	dev_dbg(prng_dev->dev, "Generated %u bytes\n", dlen);

	return dlen;
}

static struct rng_alg s5l8702_rng_alg = {
	.generate	= s5l8702_prng_generate,
	.seed		= s5l8702_prng_seed,
	.seedsize	= sizeof(u32),
	.base		= {
		.cra_name			= "prng-s5l8702",
		.cra_driver_name	= "s5l8702-prng",
		.cra_priority		= 300,
		.cra_flags			= CRYPTO_ALG_TYPE_RNG,
		.cra_ctxsize		= sizeof(struct s5l8702_prng_ctx),
		.cra_module			= THIS_MODULE,
		.cra_init			= s5l8702_prng_cra_init,
		.cra_exit			= s5l8702_prng_cra_exit,
	},
};

static int s5l8702_prng_probe(struct platform_device *pdev)
{
	struct s5l8702_prng_dev *prng_dev;
	struct device *dev = &pdev->dev;
	int ret;

	if (WARN_ON(prng_dev_global)) {
        dev_err(dev, "S5L8702 PRNG accelerator already registered\n");
		return -EBUSY;
	}

	prng_dev = devm_kzalloc(dev, sizeof(*prng_dev), GFP_KERNEL);
	if (!prng_dev) {
		return -ENOMEM;
	}

	prng_dev->dev = dev;
	mutex_init(&prng_dev->lock);

	prng_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(prng_dev->regs)) {
		return PTR_ERR(prng_dev->regs);
	}

	prng_dev->clk = devm_clk_get(dev, "prng");
	if (IS_ERR(prng_dev->clk)) {
		dev_err(dev, "Can't retrieve prng clock: %ld\n", PTR_ERR(prng_dev->clk));
		return PTR_ERR(prng_dev->clk);
	}

	ret = crypto_register_rng(&s5l8702_rng_alg);
	if (ret) {
		return dev_err_probe(dev, ret, "Failed to register crypto RNG\n");
	}

	prng_dev_global = prng_dev;
	platform_set_drvdata(pdev, prng_dev);

	dev_info(dev, "S5L8702 PRNG registered\n");

	return 0;
}

static void s5l8702_prng_remove(struct platform_device *pdev)
{
	struct s5l8702_prng_dev *prng_dev = platform_get_drvdata(pdev);

	if (prng_dev_global == prng_dev) {
		prng_dev_global = NULL;
	}

	crypto_unregister_rng(&s5l8702_rng_alg);
}

#ifdef CONFIG_OF
static const struct of_device_id s5l8702_prng_of_match[] = {
	{ .compatible = "samsung,s5l8702-prng" },
	{}
};
MODULE_DEVICE_TABLE(of, s5l8702_prng_of_match);
#endif

static struct platform_driver s5l8702_prng_driver = {
	.probe		= s5l8702_prng_probe,
	.remove		= s5l8702_prng_remove,
	.driver		= {
		.name	= "s5l8702-prng",
		.of_match_table = of_match_ptr(s5l8702_prng_of_match),
	},
};
module_platform_driver(s5l8702_prng_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_AUTHOR("Sylvia Petzanova <s.petzanova@gmail.com>");
MODULE_DESCRIPTION("S5L8702 PRNG accelerator");
MODULE_LICENSE("GPL v2");
