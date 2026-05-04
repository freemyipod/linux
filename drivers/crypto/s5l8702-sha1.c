// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 SHA-1 accelerator driver
 */

#include <crypto/internal/hash.h>
#include <crypto/sha1.h>
#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define WORD_SIZE (sizeof(u32))
#define SHA1_BLOCK_WORDS (SHA1_BLOCK_SIZE / WORD_SIZE)
#define SHA1_PAD_LEN (SHA1_BLOCK_SIZE - sizeof(u64))

#define S5L8702_SHA1_CONF     0x00
#define S5L8702_SHA1_SWRESET  0x04
//#define S5L8702_SHA1_INT_SRC  0x08
//#define S5L8702_SHA1_INT_MASK 0x0C
#define S5L8702_SHA1_ENDIAN   0x10

// Result is 20 bytes (160 bits) 0x20-0x33
#define S5L8702_SHA1_RESULT 0x20

// Input is 64 bytes (512 bits) 0x40-0x7F
#define S5L8702_SHA1_DATA   0x40

#define S5L8702_SHA1_MASTER_MODE   0x80
//#define S5L8702_SHA1_MS_START_ADDR 0x84
//#define S5L8702_SHA1_VERSION       0x88
//#define S5L8702_SHA1_MS_SIZE       0x8C
//#define S5L8702_SHA1_FIFO_PARAM    0x90
//#define S5L8702_SHA1_FIFO_CMD      0x94
//#define S5L8702_SHA1_TX_FIFO_STAT  0x98
//#define S5L8702_SHA1_TX_FIFO       0xA0

#define S5L8702_SHA1_CONF_BUSY BIT(0)
#define S5L8702_SHA1_CONF_GO   BIT(1)
#define S5L8702_SHA1_CONF_CONT BIT(3)

#define S5L8702_SHA1_TIMEOUT_MS	100

struct s5l8702_sha1_dev {
	struct device *dev;
	void __iomem *regs;
	void __iomem *clk_reg;
	struct mutex req_lock;
};

struct s5l8702_sha1_desc_ctx {
	struct s5l8702_sha1_dev *sha1_dev;
	u8 buffer[SHA1_BLOCK_SIZE];
	u32 buf_len;
	u64 total_len;
	bool is_first_block;
};

struct s5l8702_sha1_dev *sha1_dev_global;

static inline void s5l8702_sha1_writel(struct s5l8702_sha1_dev *sha1_dev,
					  u32 reg, u32 val)
{
	writel(val, sha1_dev->regs + reg);
}

static inline u32 s5l8702_sha1_readl(struct s5l8702_sha1_dev *sha1_dev, u32 reg)
{
	return readl(sha1_dev->regs + reg);
}

static int s5l8702_sha1_hw_wait_idle(struct s5l8702_sha1_dev *sha1_dev)
{
	u32 conf;
	return readl_poll_timeout(sha1_dev->regs + S5L8702_SHA1_CONF, conf,
		!(conf & S5L8702_SHA1_CONF_BUSY), 10, S5L8702_SHA1_TIMEOUT_MS * 1000);
}

static int s5l8702_sha1_hw_init(struct s5l8702_sha1_dev *sha1_dev)
{
	// wait for idle
	int ret = s5l8702_sha1_hw_wait_idle(sha1_dev);

	if (ret) {
		return ret;
	}

	// software reset
	s5l8702_sha1_writel(sha1_dev, S5L8702_SHA1_SWRESET, 1);
	s5l8702_sha1_writel(sha1_dev, S5L8702_SHA1_SWRESET, 0);

	// reset config register
	s5l8702_sha1_writel(sha1_dev, S5L8702_SHA1_CONF, 0);

	// slave mode
	s5l8702_sha1_writel(sha1_dev, S5L8702_SHA1_MASTER_MODE, 0);

	// little endian
	s5l8702_sha1_writel(sha1_dev, S5L8702_SHA1_ENDIAN, 0);

	return 0;
}

static int s5l8702_sha1_hw_run(struct s5l8702_sha1_dev *sha1_dev, const u32 *input, bool *is_first_block)
{
	int i;
	u32 conf;

	// copy input in 32-bit words
	for (i = 0; i < SHA1_BLOCK_WORDS; i++) {
		s5l8702_sha1_writel(sha1_dev, S5L8702_SHA1_DATA + (i * WORD_SIZE), input[i]);
	}

	// run the engine
	conf = s5l8702_sha1_readl(sha1_dev, S5L8702_SHA1_CONF);

	if (*is_first_block) {
		// start new hash
		conf &= ~S5L8702_SHA1_CONF_CONT;
		*is_first_block = false;
	}
	else {
		// continue hash
		conf |= S5L8702_SHA1_CONF_CONT;
	}

	// start calculating
	conf |= S5L8702_SHA1_CONF_GO;
	s5l8702_sha1_writel(sha1_dev, S5L8702_SHA1_CONF, conf);

	// wait for idle
	return s5l8702_sha1_hw_wait_idle(sha1_dev);
}

static void s5l8702_sha1_hw_get_hash(struct s5l8702_sha1_dev *sha1_dev, u32 *output)
{
	int i;

	// copy output in 32-bit words
	for (i = 0; i < SHA1_DIGEST_WORDS; i++) {
		output[i] = s5l8702_sha1_readl(sha1_dev, S5L8702_SHA1_RESULT + (i * WORD_SIZE));
	}
}

static void s5l8702_sha1_enable_clockgate(struct s5l8702_sha1_dev *sha1_dev)
{
	// TODO clk_prepare_enable()
	writel(readl(sha1_dev->clk_reg) & ~BIT(0), sha1_dev->clk_reg);
}

static void s5l8702_sha1_disable_clockgate(struct s5l8702_sha1_dev *sha1_dev)
{
	// TODO clk_disable_unprepare()
	writel(readl(sha1_dev->clk_reg) | BIT(0), sha1_dev->clk_reg);
}

static int s5l8702_sha1_init(struct shash_desc *desc)
{
	struct s5l8702_sha1_desc_ctx *dctx = shash_desc_ctx(desc);
	struct s5l8702_sha1_dev *sha1_dev;
	int ret;

	if (!sha1_dev_global) {
		return -ENODEV;
	}
	sha1_dev = sha1_dev_global;

	mutex_lock(&sha1_dev->req_lock);

	dctx->sha1_dev = sha1_dev;
	dctx->buf_len = 0;
	dctx->total_len = 0;

	s5l8702_sha1_enable_clockgate(sha1_dev);

	ret = s5l8702_sha1_hw_init(sha1_dev);

	if (ret) {
		s5l8702_sha1_disable_clockgate(sha1_dev);
		mutex_unlock(&sha1_dev->req_lock);
		return ret;
	}

	// new hash
	dctx->is_first_block = true;

	return 0;
}

static int s5l8702_sha1_update(struct shash_desc *desc,
			       const u8 *data, unsigned int len)
{
	struct s5l8702_sha1_desc_ctx *dctx = shash_desc_ctx(desc);
	struct s5l8702_sha1_dev *sha1_dev = dctx->sha1_dev;
	int ret;

	dctx->total_len += len;

	// fill leftover, if any
	if (dctx->buf_len) {
		u32 fill = SHA1_BLOCK_SIZE - dctx->buf_len;
		if (len < fill) {
			// not enough data for a full block
			memcpy(dctx->buffer + dctx->buf_len, data, len);
			dctx->buf_len += len;
			return 0;
		}

		// enough data for a full block
		memcpy(dctx->buffer + dctx->buf_len, data, fill);
		u32 block[SHA1_BLOCK_WORDS];
		memcpy(block, dctx->buffer, SHA1_BLOCK_SIZE);
		ret = s5l8702_sha1_hw_run(sha1_dev, block, &dctx->is_first_block);

		if (ret) {
			goto cleanup;
		}

		data += fill;
		len  -= fill;
		dctx->buf_len = 0;
	}

	// process full blocks
	while (len >= SHA1_BLOCK_SIZE) {
		u32 block[SHA1_BLOCK_WORDS];
		memcpy(block, data, SHA1_BLOCK_SIZE);
		ret = s5l8702_sha1_hw_run(sha1_dev, block, &dctx->is_first_block);

		if (ret) {
			goto cleanup;
		}

		data += SHA1_BLOCK_SIZE;
		len  -= SHA1_BLOCK_SIZE;
	}

	// save leftover
	if (len) {
		memcpy(dctx->buffer, data, len);
		dctx->buf_len = len;
	}

	return 0;

cleanup:
	s5l8702_sha1_disable_clockgate(sha1_dev);
	mutex_unlock(&sha1_dev->req_lock);
	return ret;
}

static int s5l8702_sha1_final(struct shash_desc *desc, u8 *out)
{
	struct s5l8702_sha1_desc_ctx *dctx = shash_desc_ctx(desc);
	struct s5l8702_sha1_dev *sha1_dev = dctx->sha1_dev;
	int ret;
	u8 *buf = dctx->buffer;
	u64 bits = cpu_to_be64(dctx->total_len * 8);
	u32 pad_len;

	// append 0x80 after the last block data
	if (dctx->buf_len >= SHA1_BLOCK_SIZE) {
		ret = -EINVAL;
		goto out;
	}

	buf[dctx->buf_len++] = 0x80;

	// if not enough room for the length (8 bytes), pad and run another pass
	if (dctx->buf_len > SHA1_PAD_LEN) {
		memset(buf + dctx->buf_len, 0, SHA1_BLOCK_SIZE - dctx->buf_len);
		u32 block[SHA1_BLOCK_WORDS];
		memcpy(block, buf, SHA1_BLOCK_SIZE);
		ret = s5l8702_sha1_hw_run(sha1_dev, block, &dctx->is_first_block);

		if (ret) {
			goto out;
		}

		dctx->buf_len = 0;
	}

	// pad with zeroes, leave 8 bytes for length
	pad_len = SHA1_PAD_LEN - dctx->buf_len;
	memset(buf + dctx->buf_len, 0, pad_len);
	dctx->buf_len += pad_len;

	// append length in big endian
	memcpy(buf + dctx->buf_len, &bits, 8);
	dctx->buf_len += 8;

	// final block
	u32 block[SHA1_BLOCK_WORDS];
	memcpy(block, buf, SHA1_BLOCK_SIZE);
	ret = s5l8702_sha1_hw_run(sha1_dev, block, &dctx->is_first_block);

	if (ret) {
		goto out;
	}

	// read back result
	u32 result[SHA1_DIGEST_WORDS];
	s5l8702_sha1_hw_get_hash(sha1_dev, result);
	memcpy(out, result, SHA1_DIGEST_SIZE);

out:
	s5l8702_sha1_disable_clockgate(sha1_dev);
	mutex_unlock(&sha1_dev->req_lock);

	return ret;
}

static int s5l8702_sha1_finup(struct shash_desc *desc,
			     const u8 *data, unsigned int len, u8 *out)
{
	int ret = s5l8702_sha1_update(desc, data, len);

	if (ret) {
		return ret;
	}

	return s5l8702_sha1_final(desc, out);
}

static int s5l8702_sha1_digest(struct shash_desc *desc,
			     const u8 *data, unsigned int len, u8 *out)
{
	int ret = s5l8702_sha1_init(desc);

	if (ret) {
		return ret;
	}

	ret = s5l8702_sha1_update(desc, data, len);

	if (ret) {
		return ret;
	}

	return s5l8702_sha1_final(desc, out);
}

static struct shash_alg s5l8702_sha1_alg = {
	.digestsize	= SHA1_DIGEST_SIZE,
	.init		= s5l8702_sha1_init,
	.update		= s5l8702_sha1_update,
	.final		= s5l8702_sha1_final,
	.finup		= s5l8702_sha1_finup,
	.digest		= s5l8702_sha1_digest,
	.descsize	= sizeof(struct s5l8702_sha1_desc_ctx),
	.base		= {
		.cra_name			= "sha1",
		.cra_driver_name	= "s5l8702-sha1",
		.cra_priority		= 300,
		.cra_blocksize		= SHA1_BLOCK_SIZE,
		.cra_flags			= CRYPTO_ALG_TYPE_SHASH,
		.cra_module			= THIS_MODULE,
	}
};

static int s5l8702_sha1_probe(struct platform_device *pdev)
{
	struct s5l8702_sha1_dev *sha1_dev;
	struct device *dev = &pdev->dev;
	int ret;

	sha1_dev = devm_kzalloc(dev, sizeof(*sha1_dev), GFP_KERNEL);
	if (!sha1_dev) {
		return -ENOMEM;
	}

	sha1_dev->dev = dev;

	sha1_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(sha1_dev->regs)) {
		return PTR_ERR(sha1_dev->regs);
	}

	// TODO: samsung_clk_register_gate() or similar
	sha1_dev->clk_reg = devm_ioremap(dev, 0x3C500048, 4);

	if (!sha1_dev->clk_reg) {
		return -ENOMEM;
	}

	mutex_init(&sha1_dev->req_lock);

	platform_set_drvdata(pdev, sha1_dev);

	sha1_dev_global = sha1_dev;

	ret = crypto_register_shash(&s5l8702_sha1_alg);

	if (ret) {
		return ret;
	}

	dev_info(dev, "Initialized");

	return 0;
}

static void s5l8702_sha1_remove(struct platform_device *pdev)
{
	sha1_dev_global = NULL;

	crypto_unregister_shash(&s5l8702_sha1_alg);
}


#ifdef CONFIG_OF
static const struct of_device_id s5l8702_sha1_of_match[] = {
	{ .compatible = "samsung,s5l8702-sha1" },
	{},
};
MODULE_DEVICE_TABLE(of, s5l8702_sha1_of_match);
#endif

static struct platform_driver s5l8702_sha1_driver = {
	.probe		= s5l8702_sha1_probe,
	.remove		= s5l8702_sha1_remove,
	.driver		= {
		.name	= "s5l8702-sha1",
		.of_match_table = of_match_ptr(s5l8702_sha1_of_match),
	},
};
module_platform_driver(s5l8702_sha1_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("S5L8702 SHA-1 accelerator");
MODULE_LICENSE("GPL v2");
