// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 I2C controller driver — polled mode
 *
 * Based on the Rockbox i2c-s5l8702.c driver by theseven.
 * Polling mirrors Rockbox's wait_rdy / i2c_wait_io pattern exactly.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

// Register offsets
#define IICCON   0x00
#define IICSTAT  0x04
#define IICADD   0x08
#define IICDS    0x0c
#define IICBUSY  0x10  // hardware-busy flag
#define IICUNK14 0x14  // clock source select: 1 = ECLK
#define IICUNK18 0x18
#define IICSTA2  0x20  // Apple extended status; write 1-bits to clear

// IICCON
#define CON_CK_REG(x)  ((x) & 0xf)
#define CON_IRQ        BIT(4)   // IRQ-pending latch
#define CON_CKSEL512   BIT(6)   // divide by 512 (vs 16)
#define CON_ACKGEN     BIT(7)   // master generates ACK during receive

// IICSTAT
#define STAT_LRB    BIT(0)  // last received bit: 0=ACK, 1=NACK
#define STAT_SOE    BIT(4)  // serial output enable
#define STAT_BB     BIT(5)  // bus busy
#define STAT_TX     BIT(6)  // transmit mode
#define STAT_MASTER BIT(7)

// IICSTA2
#define STA2_BUSHOLD  BIT(8)   // byte transferred (TX or RX)
#define STA2_STOP     BIT(13)  // STOP condition completed
#define STA2_CLEAR    (STA2_BUSHOLD | STA2_STOP)

#define BUSY_LOOPS  5000

struct s5l8702_i2c {
	void __iomem      *regs;
	struct i2c_adapter adap;
};

// Poll the hardware-busy flag before every register write.
static void rdy(struct s5l8702_i2c *i2c) {
	unsigned int n = BUSY_LOOPS;

	while (readl(i2c->regs + IICBUSY) && --n) cpu_relax();
}

// Wait for a byte transfer or STOP to complete.
static int io_wait(struct s5l8702_i2c *i2c) {
	unsigned long deadline = jiffies + msecs_to_jiffies(100);

	while (1) {
		u32 stat = readl(i2c->regs + IICSTAT);
		u32 sta2 = readl(i2c->regs + IICSTA2);

		if ((sta2 & STA2_CLEAR) || !(stat & STAT_BB)) break;
		if (time_after(jiffies, deadline)) return -ETIMEDOUT;
		cpu_relax();
	}

	rdy(i2c);
	writel(STA2_CLEAR, i2c->regs + IICSTA2);
	return 0;
}

static int s5l8702_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num) {
	struct s5l8702_i2c *i2c = i2c_get_adapdata(adap);
	const u32 con = CON_ACKGEN | CON_CKSEL512 | CON_CK_REG(0);
	int i, ret = 0;

	// Select ECLK as I2C source clock and clear any stale status.
	writel(1, i2c->regs + IICUNK14);
	writel(STA2_CLEAR, i2c->regs + IICSTA2);

	for (i = 0; i < num; i++) {
		struct i2c_msg *m = &msgs[i];
		bool rd   = !!(m->flags & I2C_M_RD);
		u8   addr = i2c_8bit_addr_from_msg(m);
		// mode: MASTER | SOE | (TX for writes)
		u32  mode = STAT_MASTER | STAT_SOE | (rd ? 0 : STAT_TX);
		int  j;

		// START
		rdy(i2c);
		writel(con, i2c->regs + IICCON);     // set clock / ACK
		rdy(i2c);
		writel(mode, i2c->regs + IICSTAT);   // mode, bus idle
		rdy(i2c);
		writel(addr, i2c->regs + IICDS);     // slave address
		rdy(i2c);
		writel(mode | STAT_BB, i2c->regs + IICSTAT); // assert START
		rdy(i2c);

		ret = io_wait(i2c);
		if (ret) goto stop;

		// Check address ACK
		if (readl(i2c->regs + IICSTAT) & STAT_LRB) {
			ret = -EREMOTEIO;
			goto stop;
		}

		// DATA
		if (!rd) {
			for (j = 0; j < m->len; j++) {
				u32 cur;

				rdy(i2c);
				writel(m->buf[j], i2c->regs + IICDS);
				udelay(5);
				rdy(i2c);
				cur = readl(i2c->regs + IICCON);
				rdy(i2c);
				writel(cur, i2c->regs + IICCON); // RMW no-op

				ret = io_wait(i2c);
				if (ret) goto stop;
				if (readl(i2c->regs + IICSTAT) & STAT_LRB) {
					ret = -EREMOTEIO;
					goto stop;
				}
			}
		} else {
			for (j = 0; j < m->len; j++) {
				u32 cur;

				rdy(i2c);
				cur = readl(i2c->regs + IICCON);
				// NAK the last byte
				if (j == m->len - 1) cur &= ~CON_ACKGEN;
				else cur |= CON_ACKGEN;
				rdy(i2c);
				writel(cur, i2c->regs + IICCON);

				ret = io_wait(i2c);
				if (ret) goto stop;
				m->buf[j] = readl(i2c->regs + IICDS);
			}
		}

stop:
		// Generate STOP
		rdy(i2c);
		writel(mode, i2c->regs + IICSTAT); // mode without BB = STOP
		rdy(i2c);
		writel(CON_IRQ, i2c->regs + IICCON);
		io_wait(i2c); // wait for STOP

		if (ret) break;
	}

	return ret ? ret : num;
}

static u32 s5l8702_i2c_func(struct i2c_adapter *adap) { return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL; }

static const struct i2c_algorithm s5l8702_i2c_algo = {
	.xfer          = s5l8702_i2c_xfer,
	.functionality = s5l8702_i2c_func,
};

static void s5l8702_i2c_hw_init(struct s5l8702_i2c *i2c) {
	// Initialize I2C hardware.
	writel(0x40, i2c->regs + IICADD);   // own slave address
	writel(1,    i2c->regs + IICUNK14); // SRCCLK = ECLK
	writel(0,    i2c->regs + IICUNK18);
	rdy(i2c);
	writel(STAT_MASTER, i2c->regs + IICSTAT);
	rdy(i2c);
	writel(0, i2c->regs + IICCON);
	rdy(i2c);
	writel(0, i2c->regs + IICSTAT);
	rdy(i2c);
	writel(STA2_CLEAR, i2c->regs + IICSTA2); // clear status
}

static int s5l8702_i2c_probe(struct platform_device *pdev) {
	struct s5l8702_i2c *i2c;

	i2c = devm_kzalloc(&pdev->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c) return -ENOMEM;
	platform_set_drvdata(pdev, i2c);

	i2c->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(i2c->regs)) return PTR_ERR(i2c->regs);

	s5l8702_i2c_hw_init(i2c);

	i2c->adap.owner       = THIS_MODULE;
	i2c->adap.class       = I2C_CLASS_DEPRECATED;
	i2c->adap.algo        = &s5l8702_i2c_algo;
	i2c->adap.dev.parent  = &pdev->dev;
	i2c->adap.dev.of_node = pdev->dev.of_node;
	i2c_set_adapdata(&i2c->adap, i2c);
	snprintf(i2c->adap.name, sizeof(i2c->adap.name), "s5l8702 (%s)", dev_name(&pdev->dev));

	return devm_i2c_add_adapter(&pdev->dev, &i2c->adap);
}

#ifdef CONFIG_OF
static const struct of_device_id s5l8702_i2c_of_match[] = {
	{ .compatible = "samsung,s5l8702-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, s5l8702_i2c_of_match);
#endif

static struct platform_driver s5l8702_i2c_driver = {
	.probe  = s5l8702_i2c_probe,
	.driver = {
		.name           = "i2c-s5l8702",
		.of_match_table = of_match_ptr(s5l8702_i2c_of_match),
	},
};
module_platform_driver(s5l8702_i2c_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("S5L8702 I2C bus adapter");
MODULE_LICENSE("GPL v2");
