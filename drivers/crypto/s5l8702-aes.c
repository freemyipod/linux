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

#define S5L8702_AES_IRQ_ALL				GENMASK(3, 0)
#define S5L8702_AES_IRQ_XFR_DONE	BIT(0)
#define S5L8702_AES_IRQ_TBUF_FULL	BIT(1)
#define S5L8702_AES_IRQ_SBUF_EMPTY	BIT(2)
#define S5L8702_AES_IRQ_ILLEGAL_OP	BIT(3)

#define S5L8702_AES_POLL_TIMEOUT_US 500000

struct s5l8702_aes_dev {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct mutex lock;
	struct s5l8702_aes_alg *algs;
};

struct s5l8702_aes_alg {
	struct skcipher_alg alg;
	struct s5l8702_aes_dev *aes_dev;
};

struct s5l8702_aes_ctx {
	struct s5l8702_aes_dev *aes_dev;
	u8 key[AES_MAX_KEY_SIZE];
	u32 keylen;
	bool cbc;
	int type;
};

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
			return -EINVAL;
	}

	return 0;
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

static int s5l8702_aes_init(struct crypto_skcipher *tfm, bool cbc, int type)
{
	struct s5l8702_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct skcipher_alg *base = crypto_skcipher_alg(tfm);
	struct s5l8702_aes_alg *alg = container_of(base, struct s5l8702_aes_alg, alg);

	if (!alg->aes_dev) {
		return -ENODEV;
	}

	ctx->aes_dev = alg->aes_dev;
	ctx->cbc = cbc;
	ctx->type = type;

	return 0;
}

static int s5l8702_aes_init_ecb(struct crypto_skcipher *tfm)
{
	return s5l8702_aes_init(tfm, false, S5L8702_AES_KEY_TYPE_USER_DEFINE);
}

static int s5l8702_aes_init_cbc(struct crypto_skcipher *tfm)
{
	return s5l8702_aes_init(tfm, true, S5L8702_AES_KEY_TYPE_USER_DEFINE);
}

static int s5l8702_aes_init_cbc_gid(struct crypto_skcipher *tfm)
{
	return s5l8702_aes_init(tfm, true, S5L8702_AES_KEY_TYPE_GLOBAL_ID);
}

static int s5l8702_aes_init_cbc_uid(struct crypto_skcipher *tfm)
{
	return s5l8702_aes_init(tfm, true, S5L8702_AES_KEY_TYPE_USER_ID);
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

static inline int s5l8702_aes_check_fused_key_length(struct crypto_skcipher *tfm, const u8 *key, unsigned int keylen)
{
	struct s5l8702_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct s5l8702_aes_dev *aes_dev = ctx->aes_dev;
	struct device *dev = aes_dev->dev;

	if (keylen != AES_KEYSIZE_128) {
		dev_err(dev, "cbc-uid and cbc-gid algorithms can only be used with AES-128, received key size %u\n", keylen);
		return -EINVAL;
	}

	return 0;
}

static void s5l8702_aes_hw_exit(struct s5l8702_aes_dev *aes_dev)
{
	s5l8702_aes_clear_state(aes_dev);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IRQ_MASK, S5L8702_AES_IRQ_ALL); // disable all interrupts
	s5l8702_aes_writel(aes_dev, S5L8702_AES_POWER, 0);
	clk_disable_unprepare(aes_dev->clk);
}

static int s5l8702_aes_hw_init(struct s5l8702_aes_ctx *ctx, bool encrypt)
{
	struct s5l8702_aes_dev *aes_dev = ctx->aes_dev;
	struct device *dev = aes_dev->dev;
	u32 compliment, cfg;
	int ret, hw_key_type;

	ret = clk_prepare_enable(aes_dev->clk);
	if (ret) {
		dev_err(dev, "clk_prepare_enable failed: %d\n", ret);
		return ret;
	}

	// init
	s5l8702_aes_reset(aes_dev); // skipped on zero key, we'll do it anyway
	s5l8702_aes_writel(aes_dev, S5L8702_AES_POWER, 1);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IRQ_MASK, 0); // enable all interrupts
	s5l8702_aes_clear_state(aes_dev);

	// key type
	if (ctx->type == S5L8702_AES_KEY_TYPE_USER_DEFINE && !memchr_inv(ctx->key, 0, ctx->keylen))
		hw_key_type = S5L8702_AES_KEY_TYPE_ZERO;
	else
		hw_key_type = ctx->type;

	s5l8702_aes_writel(aes_dev, S5L8702_AES_CIPHERKEY_SEL, hw_key_type);

	// compliment
	compliment = ~s5l8702_aes_readl(aes_dev, S5L8702_AES_CIPHERKEY_SEL);
	s5l8702_aes_writel(aes_dev, S5L8702_AES_COMPLIMENT, compliment);

	// user-defined key
	if (hw_key_type == S5L8702_AES_KEY_TYPE_USER_DEFINE) {
		ret = s5l8702_aes_write_key(aes_dev, ctx->key, ctx->keylen);
		if (ret) {
			dev_err(dev, "Invalid key size: %u\n", ctx->keylen);
			goto err_hw;
		}
	}

	// unknown register
	s5l8702_aes_writel(aes_dev, S5L8702_AES_UNK8C, 0);

	// config
	cfg = s5l8702_aes_readl(aes_dev, S5L8702_AES_CFG);

	// encrypt/decrypt
	if (encrypt)
		cfg |= BIT(0);
	else
		cfg &= ~BIT(0);

	// pause engine
	cfg |= S5L8702_AES_CFG_PAUSE;

	// chaining mode
	if (ctx->cbc)
		cfg |= BIT(3);	// CBC
	else
		cfg &= ~BIT(3);	// ECB

	// key size
	cfg &= ~S5L8702_AES_CFG_KEYSIZE;

	if (hw_key_type == S5L8702_AES_KEY_TYPE_USER_DEFINE) {
		switch (ctx->keylen) {
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
				dev_err(dev, "Invalid key length: %u\n", ctx->keylen);
				ret = -EINVAL;
				goto err_hw;
		}
	}
	// else i.e. for key types UID and GID, key size is set to 0 - nothing to do

	s5l8702_aes_writel(aes_dev, S5L8702_AES_CFG, cfg);

	return 0;

err_hw:
	s5l8702_aes_hw_exit(aes_dev);
	return ret;
}

static int s5l8702_aes_hw_crypt(struct s5l8702_aes_dev *aes_dev, dma_addr_t src, dma_addr_t dst, unsigned int len)
{
	struct device *dev = aes_dev->dev;
	u32 irq;
	int ret;

	// set src/dst buffer addresses and size
	s5l8702_aes_write_buf(aes_dev, src, dst, len);

	// go!
	s5l8702_aes_writel(aes_dev, S5L8702_AES_COMMAND, S5L8702_AES_CMD_START);

	// wait for completion
	ret = readl_poll_timeout(aes_dev->regs + S5L8702_AES_IRQ, irq,
				 irq & S5L8702_AES_IRQ_ALL, 2, S5L8702_AES_POLL_TIMEOUT_US);
	if (ret) {
		dev_err(dev, "AES timed out (IRQ=0x%08x)\n", irq);
		return ret;
	}

	if (irq & S5L8702_AES_IRQ_ILLEGAL_OP) {
		dev_err(dev, "AES illegal operation (IRQ=0x%08x)\n", irq);
		ret = -EIO;
		goto out_clear_irq;
	}

out_clear_irq:
	// clear all pending IRQs
	s5l8702_aes_writel(aes_dev, S5L8702_AES_IRQ, S5L8702_AES_IRQ_ALL);

	return ret;
}

static void s5l8702_aes_update_walk_iv(struct skcipher_walk *walk, unsigned int nbytes, bool encrypt)
{
    const u8 *src = walk->src.virt.addr;
    const u8 *dst = walk->dst.virt.addr;

    if (encrypt)
        memcpy(walk->iv, dst + nbytes - AES_BLOCK_SIZE, AES_BLOCK_SIZE);
    else
        memcpy(walk->iv, src + nbytes - AES_BLOCK_SIZE, AES_BLOCK_SIZE);
}

static int s5l8702_aes_crypt(struct skcipher_request *req, bool encrypt)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct s5l8702_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct s5l8702_aes_dev *aes_dev = ctx->aes_dev;
	struct device *dev = aes_dev->dev;
	struct skcipher_walk walk;
	int ret;

	if (ctx->cbc && !req->iv)
		return -EINVAL;

	ret = skcipher_walk_virt(&walk, req, false);
	if (ret)
		return ret;

	mutex_lock(&aes_dev->lock);

	ret = s5l8702_aes_hw_init(ctx, encrypt);
	if (ret)
		goto out_unlock;

	while (walk.nbytes) {
		dma_addr_t src, dst;

		// set IV for the current operation if needed
		if (ctx->cbc)
			s5l8702_aes_write_iv(aes_dev, walk.iv);

		// map addresses
		src = dma_map_single(dev, walk.src.virt.addr, walk.nbytes, DMA_TO_DEVICE);
		if (dma_mapping_error(dev, src)) {
			ret = -ENOMEM;
			break;
		}

		dst = dma_map_single(dev, walk.dst.virt.addr, walk.nbytes, DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, dst)) {
			dma_unmap_single(dev, src, walk.nbytes, DMA_TO_DEVICE);
			ret = -ENOMEM;
			break;
		}

		ret = s5l8702_aes_hw_crypt(aes_dev, src, dst, walk.nbytes);

		// unmap addresses
		dma_unmap_single(dev, dst, walk.nbytes, DMA_FROM_DEVICE);
		dma_unmap_single(dev, src, walk.nbytes, DMA_TO_DEVICE);

		if (ret)
			break;

		// prepare IV for the next operation if needed
		if (ctx->cbc)
			s5l8702_aes_update_walk_iv(&walk, walk.nbytes, encrypt);

		// update remaining bytes and process next chunk
		ret = skcipher_walk_done(&walk, 0);
		if (ret)
			break;
	}

	s5l8702_aes_hw_exit(aes_dev);

out_unlock:
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

static const struct s5l8702_aes_alg s5l8702_aes_alg_template[] = {
	{
		.alg = {
			.base = {
				.cra_name			= "ecb(aes)",
				.cra_driver_name	= DRV_NAME "-ecb",
				.cra_priority		= 300,
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
		},
	},
	{
		.alg = {
			.base = {
				.cra_name			= "cbc(aes)",
				.cra_driver_name	= DRV_NAME "-cbc",
				.cra_priority		= 300,
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
		},
	},
	{
		.alg = {
			.base = {
				.cra_name			= "cbc(aes-gid)",
				.cra_driver_name	= DRV_NAME "-cbc-gid",
				.cra_priority		= 300,
				.cra_blocksize		= AES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct s5l8702_aes_ctx),
				.cra_alignmask		= GENMASK(1, 0),
				.cra_module			= THIS_MODULE,
			},
			.init			= s5l8702_aes_init_cbc_gid,
			.setkey			= s5l8702_aes_check_fused_key_length,
			.encrypt		= s5l8702_aes_encrypt,
			.decrypt		= s5l8702_aes_decrypt,
			.min_keysize	= AES_KEYSIZE_128,
			.max_keysize	= AES_KEYSIZE_128,
			.ivsize			= AES_BLOCK_SIZE,
		},
	},
	{
		.alg = {
			.base = {
				.cra_name			= "cbc(aes-uid)",
				.cra_driver_name	= DRV_NAME "-cbc-uid",
				.cra_priority		= 300,
				.cra_blocksize		= AES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct s5l8702_aes_ctx),
				.cra_alignmask		= GENMASK(1, 0),
				.cra_module			= THIS_MODULE,
			},
			.init			= s5l8702_aes_init_cbc_uid,
			.setkey			= s5l8702_aes_check_fused_key_length,
			.encrypt		= s5l8702_aes_encrypt,
			.decrypt		= s5l8702_aes_decrypt,
			.min_keysize	= AES_KEYSIZE_128,
			.max_keysize	= AES_KEYSIZE_128,
			.ivsize			= AES_BLOCK_SIZE,
		},
	},
};

#define S5L8702_AES_NUM_ALGS ARRAY_SIZE(s5l8702_aes_alg_template)

static int s5l8702_aes_register_algs(struct s5l8702_aes_alg *algs)
{
	int i, ret;

	for (i = 0; i < S5L8702_AES_NUM_ALGS; i++) {
		ret = crypto_register_skcipher(&algs[i].alg);
		if (ret)
			goto err_unregister;
	}

	return 0;

err_unregister:
		while (i--)
			crypto_unregister_skcipher(&algs[i].alg);

	return ret;
}

static void s5l8702_aes_unregister_algs(struct s5l8702_aes_alg *algs)
{
	int i;

	for (i = 0; i < S5L8702_AES_NUM_ALGS; i++)
		crypto_unregister_skcipher(&algs[i].alg);
}

static int s5l8702_aes_probe(struct platform_device *pdev)
{
	struct s5l8702_aes_dev *aes_dev;
	struct device *dev = &pdev->dev;
	int i, ret;

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

	aes_dev->algs = devm_kmemdup(dev, s5l8702_aes_alg_template,
		sizeof(s5l8702_aes_alg_template), GFP_KERNEL);
	if (!aes_dev->algs)
		return -ENOMEM;

	for (i = 0; i < S5L8702_AES_NUM_ALGS; i++)
		aes_dev->algs[i].aes_dev = aes_dev;

	ret = s5l8702_aes_register_algs(aes_dev->algs);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, aes_dev);

	dev_info(dev, "S5L8702 AES accelerator initialized\n");

	return 0;
}

static void s5l8702_aes_remove(struct platform_device *pdev)
{
	struct s5l8702_aes_dev *aes_dev = platform_get_drvdata(pdev);

	s5l8702_aes_unregister_algs(aes_dev->algs);
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
