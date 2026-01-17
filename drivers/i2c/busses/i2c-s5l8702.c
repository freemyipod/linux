// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 I2C controller driver
 */

#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#define S5L8702_I2C_CON   0x0  /* Control register */
#define S5L8702_I2C_STAT  0x4  /* Control/status register */
#define S5L8702_I2C_ADD   0x8  /* Bus address register */
#define S5L8702_I2C_DS    0xc  /* Transmit/receive data shift register */
#define S5L8702_I2C_BUSY  0x10
#define S5L8702_I2C_UNK14 0x14
#define S5L8702_I2C_UNK18 0x18
#define S5L8702_I2C_INT   0x20 /* Interrupt status register */
#define S5L8702_I2C_UNK28 0x28

#define S5L8702_I2C_CON_INIT     (0x3f00)
#define S5L8702_I2C_CON_BUSHOLD  BIT(4)
#define S5L8702_I2C_CON_CKSEL16  (0 << 6)
#define S5L8702_I2C_CON_CKSEL512 BIT(6)
#define S5L8702_I2C_CON_ACKGEN   BIT(7)

#define S5L8702_I2C_STAT_LRB    BIT(0)
#define S5L8702_I2C_STAT_SOE    BIT(4)
#define S5L8702_I2C_STAT_BB     BIT(5)
#define S5L8702_I2C_STAT_TX     BIT(6)
#define S5L8702_I2C_STAT_MASTER BIT(7)

#define S5L8702_I2C_INT_BUSHOLD BIT(8)
#define S5L8702_I2C_INT_TIMEOUT BIT(9)
#define S5L8702_I2C_INT_RX      BIT(10)
#define S5L8702_I2C_INT_TX      BIT(11)
#define S5L8702_I2C_INT_START   BIT(12)
#define S5L8702_I2C_INT_STOP    BIT(13)
#define S5L8702_I2C_INT_ALL     (0x3f00)

struct s5l8702_i2c_dev {
	struct device *dev;
	void __iomem *regs;
	int irq;
	struct i2c_adapter adapter;
};

static inline void s5l8702_i2c_writel(struct s5l8702_i2c_dev *i2c_dev,
				      u32 reg, u32 val)
{
	writel(val, i2c_dev->regs + reg);
}

static inline u32 s5l8702_i2c_readl(struct s5l8702_i2c_dev *i2c_dev, u32 reg)
{
	return readl(i2c_dev->regs + reg);
}

static irqreturn_t s5l8702_i2c_isr(int this_irq, void *data)
{
	struct s5l8702_i2c_dev *i2c_dev = data;
	u32 val;

	dev_info(i2c_dev->dev, "%s", __func__);

	val = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_INT);

	// TODO
	if (val) {
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int s5l8702_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			    int num)
{
	struct s5l8702_i2c_dev *i2c_dev = i2c_get_adapdata(adap);

	dev_info(i2c_dev->dev, "%s start", __func__);

	int i;

	for (i = 0; i < num; i++) {
		dev_info(i2c_dev->dev, "%s addr=0x%04x flags=0x%04x len=%u buf=%02x",
			__func__, msgs[i].addr, msgs[i].flags, msgs[i].len, msgs[i].buf[0]);
	}

	return -EIO;
}

static u32 s5l8702_i2c_func(struct i2c_adapter *adap)
{
	struct s5l8702_i2c_dev *i2c_dev = i2c_get_adapdata(adap);

	dev_info(i2c_dev->dev, "%s", __func__);

	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm s5l8702_i2c_algo = {
	.xfer = s5l8702_i2c_xfer,
	.functionality = s5l8702_i2c_func,
};

static int s5l8702_i2c_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "%s", __func__);
	struct s5l8702_i2c_dev *i2c_dev;
	int ret;
	struct i2c_adapter *adap;

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2c_dev);
	i2c_dev->dev = &pdev->dev;

	i2c_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(i2c_dev->regs))
		return PTR_ERR(i2c_dev->regs);

	i2c_dev->irq = platform_get_irq(pdev, 0);
	if (i2c_dev->irq < 0) {
		ret = i2c_dev->irq;
		goto err;
	}

	ret = request_irq(i2c_dev->irq, s5l8702_i2c_isr, IRQF_SHARED,
			  dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		goto err;
	}

	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	snprintf(adap->name, sizeof(adap->name), "s5l8702 (%s)",
		 of_node_full_name(pdev->dev.of_node));
	adap->algo = &s5l8702_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	ret = i2c_add_adapter(adap);
	if (ret)
		goto err_free_irq;

	return 0;

err_free_irq:
	free_irq(i2c_dev->irq, i2c_dev);
err:
	return ret;
}

static void s5l8702_i2c_remove(struct platform_device *pdev)
{
	struct s5l8702_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	dev_info(i2c_dev->dev, "%s", __func__);

	free_irq(i2c_dev->irq, i2c_dev);
	i2c_del_adapter(&i2c_dev->adapter);
}

#ifdef CONFIG_OF
static const struct of_device_id s5l8702_i2c_of_match[] = {
	{ .compatible = "samsung,s5l8702-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, s5l8702_i2c_of_match);
#endif

static struct platform_driver s5l8702_i2c_driver = {
	.probe		= s5l8702_i2c_probe,
	.remove		= s5l8702_i2c_remove,
	.driver		= {
		.name	= "i2c-s5l8702",
		.of_match_table = of_match_ptr(s5l8702_i2c_of_match),
	},
};
module_platform_driver(s5l8702_i2c_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("S5L8702 I2C bus adapter");
MODULE_LICENSE("GPL v2");
