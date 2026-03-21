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

#define S5L8702_I2C_CON_CK_REG(x)		((x) & 0xf)
#define S5L8702_I2C_CON_BUSHOLD  		BIT(4)
#define S5L8702_I2C_CON_CKSEL16  		(0 << 6)
#define S5L8702_I2C_CON_CKSEL512 		BIT(6)
#define S5L8702_I2C_CON_ACKGEN   		BIT(7)
#define S5L8702_I2C_CON_INTEN_BUSHOLD 	BIT(8)
#define S5L8702_I2C_CON_INTEN_TIMEOUT 	BIT(9)
#define S5L8702_I2C_CON_INTEN_RX      	BIT(10)
#define S5L8702_I2C_CON_INTEN_TX      	BIT(11)
#define S5L8702_I2C_CON_INTEN_START   	BIT(12)
#define S5L8702_I2C_CON_INTEN_STOP    	BIT(13)
#define S5L8702_I2C_CON_INTEN_ALL 		(0x3f00)

#define S5L8702_I2C_STAT_LRB    BIT(0)
// The missing bits probably match S5L8700X datasheet ADDR_ZERO, AAS, LBA
#define S5L8702_I2C_STAT_SOE    BIT(4) // Serial Output Enable
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

#define S5L8702_I2C_XFER_TIMEOUT	(msecs_to_jiffies(100))

/* i2c controller state */
enum s5l8702_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_PREPARE_READ,
	STATE_WRITE,
	STATE_STOP
};

struct s5l8702_i2c_dev {
	struct device *dev;
	void __iomem *regs;
	int irq;
	enum s5l8702_i2c_state state;
	struct i2c_msg	*msg;
	unsigned int msg_pos;
	unsigned int nmsgs;
	int msg_ret;
	unsigned int iicstat;
	unsigned int iiccon;
	unsigned int pending_irq;
	struct completion msg_complete;
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

static void s5l8702_i2c_state_machine(struct s5l8702_i2c_dev *i2c_dev) {
	switch ( i2c_dev->state )
	{

	  case STATE_START:
		i2c_dev->pending_irq = S5L8702_I2C_INT_BUSHOLD;
		if (i2c_dev->msg->flags & I2C_M_RD) {
			i2c_dev->iicstat = S5L8702_I2C_STAT_SOE | S5L8702_I2C_STAT_MASTER;  
		} else {
		  	i2c_dev->iicstat = S5L8702_I2C_STAT_SOE | S5L8702_I2C_STAT_TX | S5L8702_I2C_STAT_MASTER;
		}
		i2c_dev->iiccon &= ~S5L8702_I2C_CON_BUSHOLD;
		i2c_dev->iiccon |= S5L8702_I2C_CON_ACKGEN;
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, i2c_dev->iicstat);
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_DS, i2c_8bit_addr_from_msg(i2c_dev->msg));
		i2c_dev->iicstat |= S5L8702_I2C_STAT_BB;
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, i2c_dev->iicstat);
		if (i2c_dev->msg->flags & I2C_M_RD) {
		  	i2c_dev->state = STATE_PREPARE_READ;
		}
		else {
			i2c_dev->state = STATE_WRITE;
		}
		break;

	  case STATE_WRITE: // Write
		if ( s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT) & S5L8702_I2C_STAT_LRB ) { // Did we receive ACK?
		  	i2c_dev->msg_ret = -EIO;
		  	goto generate_stop;
		}
		if ( i2c_dev->msg_pos == i2c_dev->msg->len ) { // is the end of the msg
		  	goto generate_stop;
		}
		i2c_dev->pending_irq = S5L8702_I2C_INT_BUSHOLD;
		i2c_dev->iiccon |= S5L8702_I2C_CON_BUSHOLD;
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_DS, i2c_dev->msg->buf[i2c_dev->msg_pos++]);
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
		break;

	  case STATE_READ: // Read
		i2c_dev->msg->buf[i2c_dev->msg_pos++] = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_DS);
		fallthrough;
	  case STATE_PREPARE_READ: // Prepare read
		if ( !i2c_dev->msg_pos && (s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT) & S5L8702_I2C_STAT_LRB) ) {
			i2c_dev->msg_ret = -EIO;
			goto generate_stop;
		}

		if ( i2c_dev->msg_pos == i2c_dev->msg->len ) { // is the end of the msg
			goto generate_stop;
		}

		if ( (i2c_dev->msg->len - i2c_dev->msg_pos) == 1 ) { // last byte of msg NACK
			i2c_dev->iiccon &= ~S5L8702_I2C_CON_ACKGEN;
		}
		i2c_dev->iiccon |= S5L8702_I2C_CON_BUSHOLD;
		i2c_dev->pending_irq = S5L8702_I2C_INT_BUSHOLD;
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
		i2c_dev->state = STATE_READ;
		break;

	  case STATE_STOP: // Generate Stop
generate_stop:
		i2c_dev->pending_irq = S5L8702_I2C_INT_STOP;

		if (i2c_dev->msg->flags & I2C_M_RD) {
			i2c_dev->iicstat &= ~S5L8702_I2C_STAT_BB;
		}
		else {
			i2c_dev->iicstat &= ~( S5L8702_I2C_STAT_BB | S5L8702_I2C_STAT_TX );
		}
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, i2c_dev->iicstat);
		i2c_dev->iiccon &= ~S5L8702_I2C_CON_ACKGEN;
		i2c_dev->iiccon |= S5L8702_I2C_CON_BUSHOLD;
		s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
		i2c_dev->nmsgs--;
		i2c_dev->msg++;
		i2c_dev->msg_pos = 0;

		// If we have an error or we processed all messages then we are done
		if (i2c_dev->msg_ret || (i2c_dev->nmsgs == 0)) {
			i2c_dev->state = STATE_IDLE;
		} else {
			i2c_dev->state = STATE_START;
		}
		break;
		
	  case STATE_IDLE: // We are done
		i2c_dev->pending_irq = 0;
		complete(&i2c_dev->msg_complete);
		break;
	}
}

static irqreturn_t s5l8702_i2c_isr(int this_irq, void *data)
{
	struct s5l8702_i2c_dev *i2c_dev = data;
	u32 val;

	val = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_INT);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_INT, val);

	dev_dbg(i2c_dev->dev, "%s state=0x%04x msg_ret=0x%04x pending_irq=0x%04x val=0x%04x",
		__func__, i2c_dev->state, i2c_dev->msg_ret, i2c_dev->pending_irq, val);

	i2c_dev->pending_irq &= ~val;

	// [TODO] Is this the best way due to us getting other interrupts?
	if (!i2c_dev->pending_irq) {
		s5l8702_i2c_state_machine(i2c_dev);
	}

	return IRQ_HANDLED;
}

static int s5l8702_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
				int num)
{
	unsigned long time_left;
	struct s5l8702_i2c_dev *i2c_dev = i2c_get_adapdata(adap);

	dev_dbg(i2c_dev->dev, "%s start", __func__);

	// [TODO] implement clocks this is equivalent to set controller active and clear interrupts
	// but we are missing clock enable and disable
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_UNK14, 1);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_INT, S5L8702_I2C_INT_ALL);

	int i;

	for (i = 0; i < num; i++) {
		dev_dbg(i2c_dev->dev, "%s addr=0x%04x flags=0x%04x len=%u buf=%02x",
			__func__, msgs[i].addr, msgs[i].flags, msgs[i].len, msgs[i].buf[0]);
	}

	i2c_dev->msg     = msgs;
	i2c_dev->nmsgs   = num;
	i2c_dev->msg_ret = 0;
	i2c_dev->msg_pos = 0;
	i2c_dev->state   = STATE_START;

	s5l8702_i2c_state_machine(i2c_dev);

	time_left = wait_for_completion_timeout(&i2c_dev->msg_complete,
						S5L8702_I2C_XFER_TIMEOUT);

	dev_dbg(i2c_dev->dev, "%s done time_left=0x%04lx msg_ret=0x%04x", __func__, time_left, i2c_dev->msg_ret);
	if (time_left == 0)
		return -ETIMEDOUT;

	return i2c_dev->msg_ret ? : num;
}

static u32 s5l8702_i2c_func(struct i2c_adapter *adap)
{
	struct s5l8702_i2c_dev *i2c_dev = i2c_get_adapdata(adap);

	dev_dbg(i2c_dev->dev, "%s", __func__);

	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm s5l8702_i2c_algo = {
	.xfer = s5l8702_i2c_xfer,
	.functionality = s5l8702_i2c_func,
};

static int s5l8702_i2c_init(struct s5l8702_i2c_dev *i2c_dev) {
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_ADD, 0x40); // [TODO] Get slave address from DT

	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_UNK14, 1);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_UNK18, 0);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, S5L8702_I2C_STAT_MASTER);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, 0);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, 0); 
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_DS, 0x40); // [TODO] Get slave address from DT

	// [TODO] calculate divisors from freq in DT
	// S5L8702_I2C_CON_CK_REG = 1 and S5L8702_I2C_CON_CKSEL16 so PCLK / 16 / 2
	// and S5L8702_I2C_CON_INTEN_BUSHOLD probably
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_CON, S5L8702_I2C_CON_INTEN_BUSHOLD | S5L8702_I2C_CON_ACKGEN | S5L8702_I2C_CON_CK_REG(1));
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_STAT, S5L8702_I2C_STAT_SOE); 
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_UNK28, 0);

	i2c_dev->iicstat = S5L8702_I2C_STAT_SOE;
	i2c_dev->iiccon = S5L8702_I2C_CON_INTEN_STOP | S5L8702_I2C_CON_INTEN_BUSHOLD | S5L8702_I2C_CON_CK_REG(1);

	return 0;
}

static int s5l8702_i2c_probe(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s", __func__);
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

	ret = s5l8702_i2c_init(i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could initialize I2C controller\n");
		goto err;
	}

	i2c_dev->irq = platform_get_irq(pdev, 0);
	if (i2c_dev->irq < 0) {
		ret = i2c_dev->irq;
		goto err;
	}

	ret = devm_request_irq(&pdev->dev, i2c_dev->irq, s5l8702_i2c_isr, IRQF_SHARED,
			  dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		goto err;
	}

	init_completion(&i2c_dev->msg_complete);

	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	snprintf(adap->name, sizeof(adap->name), "s5l8702 (%s)",
		 dev_name(&pdev->dev));
	adap->algo = &s5l8702_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	ret = devm_i2c_add_adapter(&pdev->dev, adap);
	if (ret)
		goto err;

	return 0;

err:
	return ret;
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
	.driver		= {
		.name	= "i2c-s5l8702",
		.of_match_table = of_match_ptr(s5l8702_i2c_of_match),
	},
};
module_platform_driver(s5l8702_i2c_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("S5L8702 I2C bus adapter");
MODULE_LICENSE("GPL v2");
