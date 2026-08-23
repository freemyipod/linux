// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 / S5L8740 I2C — IRQ master (S3C2410-compatible CON/STAT)
 *
 * Glass 2026-08-22: "IRQ timeout INT=0 STAT=30 CON=181 or 1B1"
 *
 * +0x20 INT is always 0. Completion is IICCON bit4 IRQPEND (S3C INTPEND).
 * CON=0x1B1 = 0x1A1|IRQPEND — byte done, ISR used to read +0x20, return
 * IRQ_NONE, leave SCL stretched. Old "BUSHOLD" *set* bit4, which is the
 * hold bit — inverted. Clear IRQPEND to resume; set IRQEN (bit5) for VIC.
 *
 * Glass 2026-08-22: VIC IRQ works (virq 38 = hwirq 22). STAT bit0 as
 * S3C LASTBIT/NAK aborted every xfer with -EIO (-5) including PMIC@0x73.
 * Ignore bit0; writes still complete on IRQPEND.
 *
 * Reads: address IRQPEND is not a data byte (DS still holds addr8).
 * clock_rx_byte() held IRQPEND *clear* and wiped the next byte-done,
 * so DS stayed 0x31/0xe7. One IRQPEND per RX byte, then read DS.
 * SEC 4AC4 / Rockbox also treat INT 0x100 as byte-ready. Not PIO.
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define S5L8702_I2C_CON		0x0
#define S5L8702_I2C_STAT	0x4
#define S5L8702_I2C_ADD		0x8
#define S5L8702_I2C_DS		0xc
#define S5L8702_I2C_BUSY	0x10
#define S5L8702_I2C_UNK14	0x14
#define S5L8702_I2C_UNK18	0x18
#define S5L8702_I2C_INT		0x20
#define S5L8702_I2C_UNK28	0x28

/* S3C2410 IICCON low byte + SEC bit8 */
#define S5L8702_I2C_CON_SCALE(x)	((x) & 0xf)
#define S5L8702_I2C_CON_IRQPEND		BIT(4)	/* W0C to resume SCL */
#define S5L8702_I2C_CON_IRQEN		BIT(5)
#define S5L8702_I2C_CON_TXDIV_512	BIT(6)
#define S5L8702_I2C_CON_ACKEN		BIT(7)
#define S5L8702_I2C_CON_SEC_BIT8	BIT(8)

#define S5L8702_I2C_CON_IDLE		(S5L8702_I2C_CON_SEC_BIT8 | \
					 S5L8702_I2C_CON_ACKEN | \
					 S5L8702_I2C_CON_IRQEN | \
					 S5L8702_I2C_CON_SCALE(1))	/* 0x1A1 */

#define S5L8702_I2C_STAT_LASTBIT	BIT(0)
#define S5L8702_I2C_STAT_TXRXEN		BIT(4)
#define S5L8702_I2C_STAT_START		BIT(5)
#define S5L8702_I2C_STAT_TX		BIT(6)
#define S5L8702_I2C_STAT_MASTER		BIT(7)
#define S5L8702_I2C_STAT_MASTER_TX	(S5L8702_I2C_STAT_MASTER | \
					 S5L8702_I2C_STAT_TX | \
					 S5L8702_I2C_STAT_TXRXEN)	/* 0xD0 */
#define S5L8702_I2C_STAT_MASTER_RX	(S5L8702_I2C_STAT_MASTER | \
					 S5L8702_I2C_STAT_TXRXEN)	/* 0x90 */

#define S5L8702_I2C_INT_ALL		0x3f00
#define S5L8702_I2C_INT_BYTE		BIT(8)	/* SEC 4AC4 / Rockbox STA2 */
#define S5L8702_I2C_INT_STOP		BIT(13)

/* SEC 1C8C canned STAT: v4=0x80 read / 0xC0 write, then |0x10 / |0x30 */
#define S5L8702_I2C_STAT_SEC_RX		0x80
#define S5L8702_I2C_STAT_SEC_TX		0xC0
#define S5L8702_I2C_STAT_SEC_SOE		0x10
#define S5L8702_I2C_STAT_SEC_GO		0x30	/* SOE|BB */

#define S5L8702_I2C_XFER_TIMEOUT	(msecs_to_jiffies(200))
#define S5L8702_I2C_BUSY_LOOPS		10000

enum s5l8702_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
};

struct s5l8702_i2c_dev {
	struct device *dev;
	void __iomem *regs;
	int irq;
	spinlock_t lock;
	enum s5l8702_i2c_state state;
	struct i2c_msg *msg;
	unsigned int msg_pos;
	unsigned int nmsgs;
	int msg_ret;
	unsigned int iiccon;
	bool timeout_logged;
	bool start_logged;
	bool isr_logged;
	bool polled_logged;
	bool stat_logged;
	bool rd_logged;
	bool drop_pend;	/* READ: skip address-phase IRQPEND (DS still addr8) */
	u8 rx_retry;
	struct completion msg_complete;
	struct i2c_adapter adapter;
	struct clk_bulk_data *clks;
	int num_clks;
};

static inline u32 s5l8702_i2c_readl(struct s5l8702_i2c_dev *i2c_dev, u32 reg)
{
	return readl(i2c_dev->regs + reg);
}

static void s5l8702_i2c_write_raw(struct s5l8702_i2c_dev *i2c_dev, u32 reg,
				  u32 val)
{
	writel(val, i2c_dev->regs + reg);
}

static void s5l8702_i2c_wait_rdy(struct s5l8702_i2c_dev *i2c_dev)
{
	unsigned int n = S5L8702_I2C_BUSY_LOOPS;

	while (s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_BUSY) && --n)
		cpu_relax();
}

static void s5l8702_i2c_writel(struct s5l8702_i2c_dev *i2c_dev, u32 reg, u32 val)
{
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, reg, val);
}

/* Clear IRQPEND so SCL runs. Never set bit4 — that stretches the bus. */
static void s5l8702_i2c_resume(struct s5l8702_i2c_dev *i2c_dev)
{
	u32 con = i2c_dev->iiccon & ~S5L8702_I2C_CON_IRQPEND;
	unsigned int n = 10000;

	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, con);
	/* Wait until this IRQPEND drops so the next service is a new byte. */
	while ((s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_CON) &
		S5L8702_I2C_CON_IRQPEND) && --n)
		cpu_relax();
}

static void s5l8702_i2c_state_machine(struct s5l8702_i2c_dev *i2c_dev);

static void s5l8702_i2c_finish(struct s5l8702_i2c_dev *i2c_dev)
{
	i2c_dev->state = STATE_IDLE;
	complete(&i2c_dev->msg_complete);
}

static void s5l8702_i2c_stop(struct s5l8702_i2c_dev *i2c_dev)
{
	u32 mode = (i2c_dev->msg->flags & I2C_M_RD) ?
		S5L8702_I2C_STAT_SEC_RX : S5L8702_I2C_STAT_SEC_TX;

	/* SEC 1C8C stop: INT=0x2000, STAT = v4|0x10 (0x90 read / 0xD0
	 * write). Do not read-modify current STAT — that left 0xe1
	 * (MASTER|TX|START|bit0) in DS on the next read.
	 */
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_INT, 0x2000);
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_STAT,
			      mode | S5L8702_I2C_STAT_SEC_SOE);
	i2c_dev->iiccon = S5L8702_I2C_CON_IDLE;
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
	i2c_dev->nmsgs = 0;
	s5l8702_i2c_finish(i2c_dev);
}

static void s5l8702_i2c_state_machine(struct s5l8702_i2c_dev *i2c_dev)
{
	u32 stat;

	switch (i2c_dev->state) {
	case STATE_START:
		dev_dbg(i2c_dev->dev, "IIC START 7bit=0x%02x DS=0x%02x %s\n",
			i2c_dev->msg->addr,
			i2c_8bit_addr_from_msg(i2c_dev->msg),
			(i2c_dev->msg->flags & I2C_M_RD) ? "RD" : "WR");
		/* Write: MASTER_TX|START = 0xF0. Read: MASTER_RX|START = 0xB0
		 * (same 0xB0 glass saw). INT +0x20 stays 0 — IRQPEND only.
		 */
		if (i2c_dev->msg->flags & I2C_M_RD)
			stat = S5L8702_I2C_STAT_MASTER_RX;
		else
			stat = S5L8702_I2C_STAT_MASTER_TX;
		i2c_dev->iiccon = S5L8702_I2C_CON_IDLE;
		s5l8702_i2c_wait_rdy(i2c_dev);
		s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_STAT, stat);
		s5l8702_i2c_wait_rdy(i2c_dev);
		s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_DS,
				      i2c_8bit_addr_from_msg(i2c_dev->msg));
		ndelay(50);
		s5l8702_i2c_wait_rdy(i2c_dev);
		s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
		s5l8702_i2c_wait_rdy(i2c_dev);
		s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_STAT,
				      stat | S5L8702_I2C_STAT_START);
		i2c_dev->state = (i2c_dev->msg->flags & I2C_M_RD) ?
			STATE_READ : STATE_WRITE;
		/* NACK the sole RX byte before the address IRQ is dropped. */
		if ((i2c_dev->msg->flags & I2C_M_RD) &&
		    i2c_dev->msg->len == 1)
			i2c_dev->iiccon &= ~S5L8702_I2C_CON_ACKEN;
		/*
		 * Glass #73: first RD IRQPEND is address complete, DS=addr8.
		 * Only skipping when IRQPEND was already set ate that IRQ
		 * as data (DS=31) and wedged the next xfer (-110).
		 */
		if (i2c_dev->msg->flags & I2C_M_RD)
			i2c_dev->drop_pend = true;
		break;

	case STATE_WRITE: {
		u32 st = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT);

		/*
		 * Do not abort on STAT bit0. S3C LASTBIT=NAK, but on this
		 * block it is 1 whenever IRQPEND fires — PMIC@0x73 (CONFIRMED)
		 * and LIS3 both returned -EIO (-5) after a working VIC IRQ.
		 */
		dev_dbg(i2c_dev->dev,
			"IIC WR STAT=%08x CON=%08x bit0=%u\n",
			st, s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_CON),
			!!(st & S5L8702_I2C_STAT_LASTBIT));
		if (i2c_dev->msg_pos == i2c_dev->msg->len) {
			s5l8702_i2c_stop(i2c_dev);
			break;
		}
		s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_DS,
				      i2c_dev->msg->buf[i2c_dev->msg_pos++]);
		ndelay(50);
		break;
	}

	case STATE_READ: {
		u32 ds, st;
		u8 addr8 = i2c_8bit_addr_from_msg(i2c_dev->msg);

		/*
		 * This IRQPEND is a completed RX byte. The address-phase
		 * IRQPEND is dropped in service_pend (DS still addr8).
		 * Do not hold IRQPEND clear — that wiped the data-ready
		 * flag and left DS=0x31/0xe7 on glass.
		 */
		if (i2c_dev->msg_pos >= i2c_dev->msg->len) {
			s5l8702_i2c_stop(i2c_dev);
			break;
		}
		ds = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_DS) & 0xff;
		st = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT) & 0xff;
		if (!i2c_dev->rd_logged) {
			i2c_dev->rd_logged = true;
			dev_info(i2c_dev->dev,
				 "IIC RD DS=%02x STAT=%02x addr8=%02x%s\n",
				 ds, st, addr8,
				 (ds == addr8) ? " (still addr)" : "");
		}
		i2c_dev->msg->buf[i2c_dev->msg_pos++] = (u8)ds;
		if (i2c_dev->msg_pos >= i2c_dev->msg->len)
			s5l8702_i2c_stop(i2c_dev);
		else if (i2c_dev->msg_pos + 1 == i2c_dev->msg->len)
			i2c_dev->iiccon &= ~S5L8702_I2C_CON_ACKEN;
		break;
	}

	case STATE_IDLE:
		break;
	}
}

#define S5L8740_VIC0_PHYS	0x38E00000ul
#define S5L8740_VIC_SIZE	0x2000

static void __iomem *s5l_vic;

static void s5l8702_dump_vic(struct s5l8702_i2c_dev *i2c_dev)
{
	void __iomem *v0, *v1;
	u32 r0, s0, e0, r1, s1, e1;

	if (!s5l_vic)
		s5l_vic = ioremap(S5L8740_VIC0_PHYS, S5L8740_VIC_SIZE);
	if (!s5l_vic)
		return;
	v0 = s5l_vic;
	v1 = s5l_vic + 0x1000;
	r0 = readl(v0 + 0x08);
	s0 = readl(v0 + 0x00);
	e0 = readl(v0 + 0x10);
	r1 = readl(v1 + 0x08);
	s1 = readl(v1 + 0x00);
	e1 = readl(v1 + 0x10);
	dev_err(i2c_dev->dev,
		"VIC0 raw=%08x stat=%08x en=%08x  VIC1 raw=%08x stat=%08x en=%08x\n",
		r0, s0, e0, r1, s1, e1);
	if (r0)
		dev_err(i2c_dev->dev, "VIC0 pending bit %u (DT IIC is 21/22)\n",
			ffs(r0) - 1);
	if (r1)
		dev_err(i2c_dev->dev, "VIC1 pending bit %u\n", ffs(r1) - 1);
}

/* Caller holds i2c_dev->lock. */
static bool s5l8702_i2c_service_pend(struct s5l8702_i2c_dev *i2c_dev)
{
	u32 con = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_CON);
	u32 extra = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_INT);

	if (!(con & S5L8702_I2C_CON_IRQPEND) &&
	    !(extra & (S5L8702_I2C_INT_ALL | S5L8702_I2C_INT_BYTE |
		       S5L8702_I2C_INT_STOP)))
		return false;
	if (extra & (S5L8702_I2C_INT_ALL | S5L8702_I2C_INT_BYTE |
		     S5L8702_I2C_INT_STOP))
		s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_INT, extra);
	if (i2c_dev->drop_pend) {
		i2c_dev->drop_pend = false;
		s5l8702_i2c_resume(i2c_dev);
		return true;
	}
	s5l8702_i2c_state_machine(i2c_dev);
	/* One S3C-style out_ack per IRQ. Do not ack inside each state. */
	s5l8702_i2c_resume(i2c_dev);
	return true;
}

static irqreturn_t s5l8702_i2c_isr(int this_irq, void *data)
{
	struct s5l8702_i2c_dev *i2c_dev = data;
	bool handled;

	spin_lock(&i2c_dev->lock);
	handled = s5l8702_i2c_service_pend(i2c_dev);
	spin_unlock(&i2c_dev->lock);
	if (!handled)
		return IRQ_NONE;
	if (!i2c_dev->isr_logged) {
		struct irq_data *d = irq_get_irq_data(this_irq);

		i2c_dev->isr_logged = true;
		dev_dbg(i2c_dev->dev, "IIC ISR fired irq=%d hwirq=%lu\n",
			this_irq, d ? d->hwirq : 0);
	}
	return IRQ_HANDLED;
}

static int s5l8702_i2c_init(struct s5l8702_i2c_dev *i2c_dev);

/* Drop leftover IRQPEND/INT so the next START's first pend is a new byte. */
static void s5l8702_i2c_drain_pend(struct s5l8702_i2c_dev *i2c_dev)
{
	unsigned int n = 8;

	while (n--) {
		u32 con = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_CON);
		u32 extra = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_INT);

		if (!(con & S5L8702_I2C_CON_IRQPEND) &&
		    !(extra & S5L8702_I2C_INT_ALL))
			break;
		if (extra & S5L8702_I2C_INT_ALL)
			s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_INT, extra);
		s5l8702_i2c_resume(i2c_dev);
	}
}

/* One START/STOP, like SEC 1C8C. IRQPEND completion for write and read. */
static int s5l8702_i2c_xfer_one(struct s5l8702_i2c_dev *i2c_dev,
				struct i2c_msg *msg)
{
	unsigned long flags, deadline;

	reinit_completion(&i2c_dev->msg_complete);
	s5l8702_i2c_writel(i2c_dev, S5L8702_I2C_UNK14, 1);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_INT, S5L8702_I2C_INT_ALL);

	spin_lock_irqsave(&i2c_dev->lock, flags);
	s5l8702_i2c_drain_pend(i2c_dev);
	i2c_dev->drop_pend = false;
	i2c_dev->rx_retry = 0;
	i2c_dev->msg = msg;
	i2c_dev->nmsgs = 1;
	i2c_dev->msg_ret = 0;
	i2c_dev->msg_pos = 0;
	i2c_dev->state = STATE_START;
	s5l8702_i2c_state_machine(i2c_dev);
	spin_unlock_irqrestore(&i2c_dev->lock, flags);

	/*
	 * Completion is IICCON IRQPEND (S3C bit4). Glass: VIC 22 can fire
	 * once ("IIC ISR fired") then go silent for later bytes — ISR-only
	 * wait became -110 on every PMIC/LIS3 xfer. Service the same
	 * IRQPEND bit from this thread when the ISR does not. Same lock as
	 * the ISR, so one IRQPEND is one step. Not the old +0x20 PIO path.
	 */
	deadline = jiffies + S5L8702_I2C_XFER_TIMEOUT;
	while (!try_wait_for_completion(&i2c_dev->msg_complete)) {
		bool serviced;

		if (time_after(jiffies, deadline)) {
			if (!i2c_dev->timeout_logged) {
				i2c_dev->timeout_logged = true;
				dev_err(i2c_dev->dev,
					"IIC IRQ timeout 7bit=0x%02x DS=0x%02x INT=%08x STAT=%08x CON=%08x state=%d isr=%d\n",
					i2c_dev->msg ? i2c_dev->msg->addr : 0,
					i2c_dev->msg ? i2c_8bit_addr_from_msg(i2c_dev->msg) : 0,
					s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_INT),
					s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT),
					s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_CON),
					i2c_dev->state,
					i2c_dev->isr_logged);
				s5l8702_dump_vic(i2c_dev);
			}
			spin_lock_irqsave(&i2c_dev->lock, flags);
			i2c_dev->state = STATE_IDLE;
			s5l8702_i2c_init(i2c_dev);
			spin_unlock_irqrestore(&i2c_dev->lock, flags);
			return -ETIMEDOUT;
		}

		spin_lock_irqsave(&i2c_dev->lock, flags);
		serviced = s5l8702_i2c_service_pend(i2c_dev);
		spin_unlock_irqrestore(&i2c_dev->lock, flags);
		if (serviced) {
			if (!i2c_dev->polled_logged && !i2c_dev->isr_logged) {
				i2c_dev->polled_logged = true;
				dev_dbg(i2c_dev->dev,
					"IIC IRQPEND polled (VIC irq=%d missed)\n",
					i2c_dev->irq);
				s5l8702_dump_vic(i2c_dev);
			}
			continue;
		}
		cpu_relax();
	}
	return i2c_dev->msg_ret;
}

/*
 * emcore/umsboot s5l87xx_i2c_recv_byte — the RX kick this IP needs.
 * After the address IRQPEND, rewrite CON to 0xB7 (ACK) or 0x37 (NACK),
 * wait for bit4, then read DS. Clearing bit4 (our write resume) does
 * not clock a payload byte — glass #74 still had DS=addr8.
 * Writes stay on the IRQPEND path. Not samsung,pio-mode.
 */
static int s5l8702_i2c_wait_con_pend(struct s5l8702_i2c_dev *i2c_dev)
{
	unsigned int i;

	for (i = 0; i < 200000; i++) {
		if (s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_CON) &
		    S5L8702_I2C_CON_IRQPEND)
			return 0;
		cpu_relax();
	}
	return -ETIMEDOUT;
}

static int s5l8702_i2c_xfer_read(struct s5l8702_i2c_dev *i2c_dev,
				 struct i2c_msg *msg)
{
	unsigned int i;
	int ret;
	u8 addr8 = i2c_8bit_addr_from_msg(msg);
	static bool rd_ok_logged;

	disable_irq(i2c_dev->irq);
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_DS, addr8);
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_STAT, 0xb0);
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, 0xb7);
	ret = s5l8702_i2c_wait_con_pend(i2c_dev);
	if (ret)
		goto out;

	for (i = 0; i < msg->len; i++) {
		u8 ds;
		u32 st;
		bool ack = (i + 1 < msg->len);

		s5l8702_i2c_wait_rdy(i2c_dev);
		s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON,
				      ack ? 0xb7 : 0x37);
		ret = s5l8702_i2c_wait_con_pend(i2c_dev);
		if (ret)
			goto out;
		ds = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_DS) & 0xff;
		st = s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT) & 0xff;
		msg->buf[i] = ds;
		if (!rd_ok_logged) {
			rd_ok_logged = true;
			dev_info(i2c_dev->dev,
				 "IIC RD DS=%02x STAT=%02x addr8=%02x%s\n",
				 ds, st, addr8,
				 (ds == addr8) ? " (still addr)" : "");
		}
	}

out:
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_STAT, 0x90);
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, 0xb7);
	for (i = 0; i < 200000; i++) {
		if (!(s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT) &
		      S5L8702_I2C_STAT_START))
			break;
		cpu_relax();
	}
	i2c_dev->iiccon = S5L8702_I2C_CON_IDLE;
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
	enable_irq(i2c_dev->irq);
	if (ret)
		dev_err_once(i2c_dev->dev, "IIC emcore-RX timeout\n");
	return ret;
}

/* emcore i2c_send: DS then CON=0xB7, wait bit4. Same kick as RX. */
static int s5l8702_i2c_xfer_write(struct s5l8702_i2c_dev *i2c_dev,
				  struct i2c_msg *msg)
{
	unsigned int i;
	int ret;
	u8 addr8 = i2c_8bit_addr_from_msg(msg);

	disable_irq(i2c_dev->irq);
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_DS, addr8);
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_STAT, 0xf0);
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, 0xb7);
	ret = s5l8702_i2c_wait_con_pend(i2c_dev);
	if (ret)
		goto out;
	for (i = 0; i < msg->len; i++) {
		s5l8702_i2c_wait_rdy(i2c_dev);
		s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_DS, msg->buf[i]);
		s5l8702_i2c_wait_rdy(i2c_dev);
		s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, 0xb7);
		ret = s5l8702_i2c_wait_con_pend(i2c_dev);
		if (ret)
			goto out;
	}
out:
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_STAT, 0xd0);
	s5l8702_i2c_wait_rdy(i2c_dev);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, 0xb7);
	for (i = 0; i < 200000; i++) {
		if (!(s5l8702_i2c_readl(i2c_dev, S5L8702_I2C_STAT) &
		      S5L8702_I2C_STAT_START))
			break;
		cpu_relax();
	}
	i2c_dev->iiccon = S5L8702_I2C_CON_IDLE;
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
	enable_irq(i2c_dev->irq);
	if (ret)
		dev_err_once(i2c_dev->dev, "IIC emcore-TX timeout\n");
	return ret;
}

static int s5l8702_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			    int num)
{
	struct s5l8702_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int i, ret;
	static bool split_logged;

	/*
	 * SEC 3F40 = 3F60 write of the register, STOP, then 1C8C READ.
	 * Linux smbus_read_byte_data is two msgs in one xfer (Sr).
	 * Chaining the READ START inside the WRITE's IRQ left STAT=0xe1
	 * and every r1–r12 read returned that byte.
	 */
	if (!split_logged) {
		split_logged = true;
		dev_dbg(i2c_dev->dev,
			"IIC one-msg xfer (SEC 1C8C STOP, IRQPEND write+read)\n");
	}
	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = s5l8702_i2c_xfer_read(i2c_dev, &msgs[i]);
		else
			ret = s5l8702_i2c_xfer_write(i2c_dev, &msgs[i]);
		if (ret)
			return ret;
		if (i + 1 < num)
			udelay(100);
	}
	return num;
}

static u32 s5l8702_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm s5l8702_i2c_algo = {
	.xfer = s5l8702_i2c_xfer,
	.functionality = s5l8702_i2c_func,
};

#define S5L87XX_CLK_BASE	0x3C500000u
#define S5L87XX_PWRCON1		(S5L87XX_CLK_BASE + 0x4C)

static void s5l8702_i2c_ungate(struct s5l8702_i2c_dev *i2c_dev,
			       resource_size_t base)
{
	void __iomem *pwr;
	u32 val, mask;

	if (base == 0x3C600000)
		mask = BIT(4);
	else if (base == 0x3C900000)
		mask = BIT(6);
	else
		return;

	pwr = ioremap(S5L87XX_PWRCON1, 4);
	if (!pwr)
		return;
	val = readl(pwr) & ~mask;
	writel(val, pwr);
	iounmap(pwr);
	udelay(50);
}

static int s5l8702_i2c_init(struct s5l8702_i2c_dev *i2c_dev)
{
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_ADD, 0x40);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_UNK14, 1);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_UNK18, 0);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_STAT, 0);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, 0);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_DS, 0x40);
	i2c_dev->iiccon = S5L8702_I2C_CON_IDLE;
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_CON, i2c_dev->iiccon);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_STAT, S5L8702_I2C_STAT_TXRXEN);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_UNK28, 0);
	s5l8702_i2c_write_raw(i2c_dev, S5L8702_I2C_INT, S5L8702_I2C_INT_ALL);
	return 0;
}

static int s5l8702_i2c_probe(struct platform_device *pdev)
{
	struct s5l8702_i2c_dev *i2c_dev;
	struct i2c_adapter *adap;
	struct resource *res;
	int ret;

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2c_dev);
	i2c_dev->dev = &pdev->dev;
	spin_lock_init(&i2c_dev->lock);

	i2c_dev->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(i2c_dev->regs))
		return PTR_ERR(i2c_dev->regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		s5l8702_i2c_ungate(i2c_dev, res->start);

	ret = devm_clk_bulk_get_all(&pdev->dev, &i2c_dev->clks);
	if (ret > 0) {
		i2c_dev->num_clks = ret;
		ret = clk_bulk_prepare_enable(i2c_dev->num_clks, i2c_dev->clks);
		if (ret)
			return ret;
	}

	s5l8702_i2c_init(i2c_dev);

	i2c_dev->irq = platform_get_irq(pdev, 0);
	if (i2c_dev->irq < 0)
		return i2c_dev->irq;

	ret = devm_request_irq(&pdev->dev, i2c_dev->irq, s5l8702_i2c_isr, 0,
			       dev_name(&pdev->dev), i2c_dev);
	if (ret)
		return ret;

	init_completion(&i2c_dev->msg_complete);
	dev_info(&pdev->dev, "IIC IRQ INTPEND irq=%d CON=0x%03lx\n",
		 i2c_dev->irq, (unsigned long)S5L8702_I2C_CON_IDLE);

	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, i2c_dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	snprintf(adap->name, sizeof(adap->name), "s5l8702 (%s)",
		 dev_name(&pdev->dev));
	adap->algo = &s5l8702_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	return devm_i2c_add_adapter(&pdev->dev, adap);
}

static const struct of_device_id s5l8702_i2c_of_match[] = {
	{ .compatible = "samsung,s5l8702-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8702_i2c_of_match);

static struct platform_driver s5l8702_i2c_driver = {
	.probe = s5l8702_i2c_probe,
	.driver = {
		.name = "i2c-s5l8702",
		.of_match_table = s5l8702_i2c_of_match,
	},
};
module_platform_driver(s5l8702_i2c_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("S5L8702 I2C bus adapter");
MODULE_LICENSE("GPL v2");
