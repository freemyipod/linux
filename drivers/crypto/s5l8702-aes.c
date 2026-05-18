// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 AES Accelerator Driver
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include <crypto/aes.h>
#include <crypto/internal/skcipher.h>
#include <crypto/scatterwalk.h>
#include <crypto/skcipher.h>

#define DRV_NAME "s5l8702-aes"

#define S5L8702_AES_POWER			0x00
#define S5L8702_AES_COMMAND			0x04
#define S5L8702_AES_SWRST			0x08
#define S5L8702_AES_IRQ				0x0C
#define S5L8702_AES_IRQ_MASK		0x10
#define S5L8702_AES_CFG				0x14
#define S5L8702_AES_XFR_NUM			0x18
#define S5L8702_AES_XFR_CNT			0x1C
#define S5L8702_AES_TBUF_START		0x20
#define S5L8702_AES_TBUF_SIZE		0x24
#define S5L8702_AES_SBUF_START		0x28
#define S5L8702_AES_SBUF_SIZE		0x2C
#define S5L8702_AES_CRYPT_START		0x30
#define S5L8702_AES_CRYPT_SIZE		0x34
#define S5L8702_AES_CADDR_TBUF		0x38
#define S5L8702_AES_CADDR_SBUF		0x3C
#define S5L8702_AES_XFR_STATUS		0x40
#define S5L8702_AES_BUS_FIFO_STATUS	0x44
#define S5L8702_AES_FIFO_STATUS		0x48
#define S5L8702_AES_KEY_MX			0x4C
#define S5L8702_AES_KEY_MH			0x50
#define S5L8702_AES_KEY_MM			0x54
#define S5L8702_AES_KEY_ML			0x58
#define S5L8702_AES_KEY_X			0x5C
#define S5L8702_AES_KEY_H			0x60
#define S5L8702_AES_KEY_M			0x64
#define S5L8702_AES_KEY_L			0x68
#define S5L8702_AES_CIPHERKEY_SEL	0x6C
#define S5L8702_AES_ENDIAN			0x70
#define S5L8702_AES_IV_1			0x74
#define S5L8702_AES_IV_2			0x78
#define S5L8702_AES_IV_3			0x7C
#define S5L8702_AES_IV_4			0x80
#define S5L8702_AES_COMPLIMENT		0x88
#define S5L8702_AES_UNK8C			0x8C

#define S5L8702_AES_KEY_TYPE_USER_DEFINE	0
#define S5L8702_AES_KEY_TYPE_GLOBAL_ID		1
#define S5L8702_AES_KEY_TYPE_USER_ID		2
#define S5L8702_AES_KEY_TYPE_ZERO			3

#define S5L8702_AES_KEY_SIZE_128	0
#define S5L8702_AES_KEY_SIZE_192	1
#define S5L8702_AES_KEY_SIZE_256	2

#define S5L8702_AES_CMD_STOP		0
#define S5L8702_AES_CMD_START		1
#define S5L8702_AES_CMD_ABORT		2
#define S5L8702_AES_CMD_CONTINUE	3

#define S5L8702_AES_CFG_KEYSIZE	GENMASK(5, 4)
#define S5L8702_AES_CFG_PAUSE	GENMASK(2, 1)

#define S5L8702_AES_IRQ_ALL	GENMASK(3, 0)

#define S5L8702_AES_POLL_TIMEOUT_US 500000

struct s5l8702_aes_dev {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct mutex lock;
};

struct s5l8702_aes_ctx {
	struct s5l8702_aes_dev *aes_dev;
	u8 key[AES_MAX_KEY_SIZE];
	u32 keylen;
	bool cbc;
};

static struct s5l8702_aes_dev *aes_dev_global;

static inline void s5l8702_aes_writel(struct s5l8702_aes_dev *aes_dev,
					  u32 reg, u32 val)
{
	writel(val, aes_dev->regs + reg);
}

static inline u32 s5l8702_aes_readl(struct s5l8702_aes_dev *aes_dev, u32 reg)
{
	return readl(aes_dev->regs + reg);
}

static inline void s5l8702_aes_reset(struct s5l8702_aes_dev *aes_dev)
{
	s5l8702_aes_writel(aes_dev, S5L8702_AES_SWRST, 1);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_SWRST, 0);
}

static inline void s5l8702_aes_clear_state(struct s5l8702_aes_dev *aes_dev)
{
	s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_MX, 0);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_MH, 0);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_MM, 0);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_ML, 0);

	s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_X, 0);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_H, 0);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_M, 0);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_L, 0);

	s5l8702_aes_writel(aes_dev, S5L8702_AES_IV_1, 0);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IV_2, 0);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IV_3, 0);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IV_4, 0);
}

static inline int s5l8702_aes_write_key(struct s5l8702_aes_dev *aes_dev, const u8 *key, unsigned int keylen)
{
	size_t offset = 0;

	switch (keylen) {
		case AES_KEYSIZE_256:
			s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_MX, get_unaligned_le32(key + offset));
			offset += sizeof(u32);
			s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_MH, get_unaligned_le32(key + offset));
			offset += sizeof(u32);
			fallthrough;
		case AES_KEYSIZE_192:
			s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_MM, get_unaligned_le32(key + offset));
			offset += sizeof(u32);
			s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_ML, get_unaligned_le32(key + offset));
			offset += sizeof(u32);
			fallthrough;
		case AES_KEYSIZE_128:
			s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_X, get_unaligned_le32(key + offset));
			offset += sizeof(u32);
			s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_H, get_unaligned_le32(key + offset));
			offset += sizeof(u32);
			s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_M, get_unaligned_le32(key + offset));
			offset += sizeof(u32);
			s5l8702_aes_writel(aes_dev, S5L8702_AES_KEY_L, get_unaligned_le32(key + offset));
		break;
		default:
			dev_err(aes_dev->dev, "Invalid key size: %u\n", keylen);
			return -EINVAL;
	}

	return 0;
}

static inline void s5l8702_aes_read_iv(struct s5l8702_aes_dev *aes_dev, void *iv)
{
	put_unaligned_le32(s5l8702_aes_readl(aes_dev, S5L8702_AES_IV_1), iv);
	put_unaligned_le32(s5l8702_aes_readl(aes_dev, S5L8702_AES_IV_2), iv + sizeof(u32));
	put_unaligned_le32(s5l8702_aes_readl(aes_dev, S5L8702_AES_IV_3), iv + sizeof(u32) * 2);
	put_unaligned_le32(s5l8702_aes_readl(aes_dev, S5L8702_AES_IV_4), iv + sizeof(u32) * 3);
}

static inline void s5l8702_aes_write_iv(struct s5l8702_aes_dev *aes_dev, const void *iv)
{
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IV_1, get_unaligned_le32(iv));
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IV_2, get_unaligned_le32(iv + sizeof(u32)));
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IV_3, get_unaligned_le32(iv + sizeof(u32) * 2));
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IV_4, get_unaligned_le32(iv + sizeof(u32) * 3));
}

static inline void s5l8702_aes_write_buf(struct s5l8702_aes_dev *aes_dev, u32 src_start, u32 dst_start, u32 size)
{
	s5l8702_aes_writel(aes_dev, S5L8702_AES_XFR_NUM, size);

	s5l8702_aes_writel(aes_dev, S5L8702_AES_TBUF_START, dst_start);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_TBUF_SIZE, size);

	s5l8702_aes_writel(aes_dev, S5L8702_AES_SBUF_START, src_start);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_SBUF_SIZE, size);

	s5l8702_aes_writel(aes_dev, S5L8702_AES_CRYPT_START, src_start);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_CRYPT_SIZE, size);
}

static int s5l8702_aes_init(struct crypto_skcipher *tfm, bool cbc)
{
	struct s5l8702_aes_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (!aes_dev_global) {
		return -ENODEV;
	}

	ctx->aes_dev = aes_dev_global;
	ctx->cbc = cbc;

	return 0;
}

static int s5l8702_aes_init_ecb(struct crypto_skcipher *tfm)
{
	return s5l8702_aes_init(tfm, false);
}

static int s5l8702_aes_init_cbc(struct crypto_skcipher *tfm)
{
	return s5l8702_aes_init(tfm, true);
}

static void s5l8702_aes_exit(struct crypto_skcipher *tfm)
{
	struct s5l8702_aes_ctx *ctx = crypto_skcipher_ctx(tfm);

	memzero_explicit(ctx->key, sizeof(ctx->key));
}

static int s5l8702_aes_setkey(struct crypto_skcipher *tfm, const u8 *key, unsigned int keylen)
{
	struct s5l8702_aes_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 && keylen != AES_KEYSIZE_256)
		return -EINVAL;

	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;
	return 0;
}

static int s5l8702_aes_crypt(struct skcipher_request *req, bool encrypt)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct s5l8702_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct s5l8702_aes_dev *aes_dev = ctx->aes_dev;
	struct device *dev = aes_dev->dev;
	struct skcipher_walk walk;
	void *key = ctx->key;
	u32 key_len = ctx->keylen;
	u32 key_type, compliment;
	u32 cfg, irq;
	int ret;

	if (ctx->cbc && !req->iv)
		return -EINVAL;

	ret = skcipher_walk_virt(&walk, req, false);
	if (ret)
		return ret;

	mutex_lock(&aes_dev->lock);

	ret = clk_prepare_enable(aes_dev->clk);
	if (ret) {
		dev_err(dev, "clk_prepare_enable failed: %d\n", ret);
		goto out;
	}

	// init
	s5l8702_aes_reset(aes_dev); // skipped on zero key, we'll do it anyway
	s5l8702_aes_writel(aes_dev, S5L8702_AES_POWER, 1);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IRQ_MASK, 0); // enable all interrupts
	s5l8702_aes_clear_state(aes_dev);

	// key type
	if (memchr_inv(key, 0, key_len)) {
		key_type = S5L8702_AES_KEY_TYPE_USER_DEFINE;
	}
	else {
		key_type = S5L8702_AES_KEY_TYPE_ZERO;
	}

	s5l8702_aes_writel(aes_dev, S5L8702_AES_CIPHERKEY_SEL, key_type);

	// compliment
	compliment = ~s5l8702_aes_readl(aes_dev, S5L8702_AES_CIPHERKEY_SEL);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_COMPLIMENT, compliment);

	// user-defined key
	if (key_type == S5L8702_AES_KEY_TYPE_USER_DEFINE) {
		ret = s5l8702_aes_write_key(aes_dev, key, key_len);
		if (ret)
			goto out;
	}

	// unknown register
	s5l8702_aes_writel(aes_dev, S5L8702_AES_UNK8C, 0);

	// config
	cfg = s5l8702_aes_readl(aes_dev, S5L8702_AES_CFG);

	// encrypt/decrypt
	if (encrypt) {
		cfg |= BIT(0);
	}
	else {
		cfg &= ~BIT(0);
	}

	// pause engine
	cfg |= S5L8702_AES_CFG_PAUSE;

	// chaining mode
	if (ctx->cbc) {
		// CBC
		cfg |= BIT(3);
	}
	else {
		// ECB
		cfg &= ~BIT(3);
	}

	// key size
	cfg &= ~S5L8702_AES_CFG_KEYSIZE;

	switch (key_len) {
		case AES_KEYSIZE_128:
			cfg |= FIELD_PREP(S5L8702_AES_CFG_KEYSIZE, S5L8702_AES_KEY_SIZE_128);
			break;
		case AES_KEYSIZE_192:
			cfg |= FIELD_PREP(S5L8702_AES_CFG_KEYSIZE, S5L8702_AES_KEY_SIZE_192);
			break;
		case AES_KEYSIZE_256:
			cfg |= FIELD_PREP(S5L8702_AES_CFG_KEYSIZE, S5L8702_AES_KEY_SIZE_256);
			break;
		default:
			dev_err(dev, "Invalid key length: %u\n", key_len);
			ret = -EINVAL;
			goto out;
	}

	s5l8702_aes_writel(aes_dev, S5L8702_AES_CFG, cfg);

	while (walk.nbytes > 0) {
		dma_addr_t src, dst;
		unsigned int chunk_len = round_down(walk.nbytes, AES_BLOCK_SIZE);

		if (chunk_len == 0)
			break;

		// set IV for CBC
		if (ctx->cbc)
			s5l8702_aes_write_iv(aes_dev, walk.iv);

		// map addresses
		src = dma_map_single(dev, walk.src.virt.addr, chunk_len, DMA_TO_DEVICE);
		if (dma_mapping_error(dev, src)) {
			ret = -ENOMEM;
			goto out;
		}

		dst = dma_map_single(dev, walk.dst.virt.addr, chunk_len, DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, dst)) {
			dma_unmap_single(dev, src, chunk_len, DMA_TO_DEVICE);
			ret = -ENOMEM;
			goto out;
		}

		// set src/dst buffer addresses and size
		s5l8702_aes_write_buf(aes_dev, src, dst, chunk_len);

		// go!
		s5l8702_aes_writel(aes_dev, S5L8702_AES_COMMAND, S5L8702_AES_CMD_START);

		// wait for completion
		ret = readl_poll_timeout(aes_dev->regs + S5L8702_AES_IRQ, irq,
					 irq & S5L8702_AES_IRQ_ALL, 2, S5L8702_AES_POLL_TIMEOUT_US);
		if (ret) {
			dev_err(dev, "AES timed out (IRQ=0x%08x)\n", irq);

			dma_unmap_single(dev, src, chunk_len, DMA_TO_DEVICE);
			dma_unmap_single(dev, dst, chunk_len, DMA_FROM_DEVICE);

			goto out;
		}

		// clear all pending IRQs
		s5l8702_aes_writel(aes_dev, S5L8702_AES_IRQ, S5L8702_AES_IRQ_ALL);

		// unmap addresses
		dma_unmap_single(dev, src, chunk_len, DMA_TO_DEVICE);
		dma_unmap_single(dev, dst, chunk_len, DMA_FROM_DEVICE);

		// update IV
		if (ctx->cbc)
			s5l8702_aes_read_iv(aes_dev, walk.iv);

		// update remaining bytes and process next chunk
		ret = skcipher_walk_done(&walk, walk.nbytes - chunk_len);
		if (ret)
			goto out;
	}

	ret = 0;

out:
	s5l8702_aes_clear_state(aes_dev);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IRQ_MASK, S5L8702_AES_IRQ_ALL); // disable all interrupts
	s5l8702_aes_writel(aes_dev, S5L8702_AES_POWER, 0);
	clk_disable_unprepare(aes_dev->clk);
	mutex_unlock(&aes_dev->lock);
	return ret;
}

static int s5l8702_aes_encrypt(struct skcipher_request *req)
{
	return s5l8702_aes_crypt(req, true);
}

static int s5l8702_aes_decrypt(struct skcipher_request *req)
{
	return s5l8702_aes_crypt(req, false);
}

static struct skcipher_alg s5l8702_aes_alg_ecb = {
	.base = {
		.cra_name			= "ecb(aes)",
		.cra_driver_name	= DRV_NAME "-ecb",
		.cra_priority		= 300,
		.cra_flags			= 0,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct s5l8702_aes_ctx),
		.cra_alignmask		= GENMASK(1, 0),
		.cra_module			= THIS_MODULE,
	},
	.init			= s5l8702_aes_init_ecb,
	.exit			= s5l8702_aes_exit,
	.setkey			= s5l8702_aes_setkey,
	.encrypt		= s5l8702_aes_encrypt,
	.decrypt		= s5l8702_aes_decrypt,
	.min_keysize	= AES_MIN_KEY_SIZE,
	.max_keysize	= AES_MAX_KEY_SIZE,
};

static struct skcipher_alg s5l8702_aes_alg_cbc = {
	.base = {
		.cra_name			= "cbc(aes)",
		.cra_driver_name	= DRV_NAME "-cbc",
		.cra_priority		= 300,
		.cra_flags			= 0,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct s5l8702_aes_ctx),
		.cra_alignmask		= GENMASK(1, 0),
		.cra_module			= THIS_MODULE,
	},
	.init			= s5l8702_aes_init_cbc,
	.exit			= s5l8702_aes_exit,
	.setkey			= s5l8702_aes_setkey,
	.encrypt		= s5l8702_aes_encrypt,
	.decrypt		= s5l8702_aes_decrypt,
	.min_keysize	= AES_MIN_KEY_SIZE,
	.max_keysize	= AES_MAX_KEY_SIZE,
	.ivsize			= AES_BLOCK_SIZE,
};

static int s5l8702_aes_probe(struct platform_device *pdev)
{
	struct s5l8702_aes_dev *aes_dev;
	struct device *dev = &pdev->dev;
	int ret;

	if (WARN_ON(aes_dev_global)) {
		dev_err(dev, "S5L8702 AES accelerator already registered\n");
		return -EBUSY;
	}

	aes_dev = devm_kzalloc(dev, sizeof(*aes_dev), GFP_KERNEL);
	if (!aes_dev)
		return -ENOMEM;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "dma_set_mask_and_coherent failed (%d)\n", ret);
		return ret;
	}

	aes_dev->dev = dev;
	mutex_init(&aes_dev->lock);

	aes_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(aes_dev->regs))
		return PTR_ERR(aes_dev->regs);

	aes_dev->clk = devm_clk_get(dev, "aes");
	if (IS_ERR(aes_dev->clk)) {
		dev_err(dev, "Can't retrieve aes clock: %ld\n", PTR_ERR(aes_dev->clk));
		return PTR_ERR(aes_dev->clk);
	}

	ret = crypto_register_skcipher(&s5l8702_aes_alg_ecb);
	if (ret)
		return ret;

	ret = crypto_register_skcipher(&s5l8702_aes_alg_cbc);
	if (ret) {
		crypto_unregister_skcipher(&s5l8702_aes_alg_ecb);
		return ret;
	}

	aes_dev_global = aes_dev;
	platform_set_drvdata(pdev, aes_dev);

	dev_info(dev, "S5L8702 AES accelerator initialized\n");

	return 0;
}

static void s5l8702_aes_remove(struct platform_device *pdev)
{
	struct s5l8702_aes_dev *aes_dev = platform_get_drvdata(pdev);

	if (aes_dev_global == aes_dev) {
		aes_dev_global = NULL;
	}

	crypto_unregister_skcipher(&s5l8702_aes_alg_ecb);
	crypto_unregister_skcipher(&s5l8702_aes_alg_cbc);
}

#ifdef CONFIG_OF
static const struct of_device_id s5l8702_aes_of_match[] = {
	{ .compatible = "samsung,s5l8702-aes" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8702_aes_of_match);
#endif

static struct platform_driver s5l8702_aes_driver = {
	.probe	= s5l8702_aes_probe,
	.remove = s5l8702_aes_remove,
	.driver	= {
		.name	= DRV_NAME,
		.of_match_table = of_match_ptr(s5l8702_aes_of_match),
	},
};
module_platform_driver(s5l8702_aes_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("S5L8702 AES Accelerator");
MODULE_LICENSE("GPL v2");
