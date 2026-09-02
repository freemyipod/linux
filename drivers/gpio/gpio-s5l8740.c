// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO driver for Samsung/Apple S5L8740 (iPod nano 7G / N31)
 *
 * Banked MMIO @ 0x3CF00000 (RetailOS / Rockbox):
 *   bank_base = base + 32 * (gpio >> 3)
 *   DIN  = bank + 0x04
 *   DOUT = bank + 0x08
 *   DIR  = bank + 0x14   (bit set on pinmux/mode!=1; cleared on mode 0xFFFE)
 *
 * GPIOCMD latch @ 0x3CF001E0 (sub_43D38C):
 *   word = (bank << 16) | (pin << 8) | cmd
 *   mode==1:      cmd = val ? 15 : 14   (drive high / low)
 *   mode==0xFFFE: clear DIR bit, cmd=0
 *   else:         set DIR bit, cmd=(u8)mode   (pinmux / EN-enable mode 0)
 *
 * IRQ: gc.to_irq maps through the sibling EIC (apple,eic / apple,s5l8740-eic)
 * after s5l8740_eic_enable_gpio(offset, IRQ_TYPE_LEVEL_LOW). Enough for
 * gpio-keys once EIC parent chaining is proven; hierarchical irqchip optional.
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include <linux/apple-n31.h>

#define S5L8740_GPIO_BANK_STRIDE	32
#define S5L8740_GPIO_PCON_OFF		0x00
#define S5L8740_GPIO_DIN_OFF		0x04
/* sub_428F70 target: input/pull enable, one bit per pad. */
#define S5L8740_GPIO_INEN_OFF		0x0c
#define S5L8740_GPIO_DOUT_OFF		0x08
#define S5L8740_GPIO_DIR_OFF		0x14

/*
 * Guards every read-modify-write of a per-bank GPIO register. PCON, DIR,
 * PUNB and PUNC each pack eight pads into one word, so an unlocked RMW can
 * silently drop a bit another caller just set.
 */
static DEFINE_RAW_SPINLOCK(s5l8740_gpio_reg_lock);
#define S5L8740_GPIOCMD_OFF		0x1e0
/*
 * 32 banks of 8 across the 0x400 the block occupies. The old 128 cleared
 * BT host-wake at 119 but not pad 200, which sub_17D4DC uses as the
 * Bluetooth power control on some board variants.
 */
#define S5L8740_GPIO_DEFAULT_NGPIO	256

#define S5L8740_CMD_OUT_LOW		14
#define S5L8740_CMD_OUT_HIGH		15

/*
 * IpodSec sub_223C / sub_47CC — packed pinmux word:
 *   [31:24] bank, [23:16] pin, [15] pull?, [12] bit→+0x14?, [8] bit→+0x10,
 *   [4] bit→+0x0C, [3:0] nibble into bank+0 function field.
 * Table extracted from bootloader VA 0x22004C6C (121 words).
 */
#include "pinmux_table.inc"

static void s5l8740_pinmux_apply_word(void __iomem *gpio_base, u32 a1)
{
	unsigned int bank = (a1 >> 24) & 0xff;
	unsigned int pin = (a1 >> 16) & 0xff;
	void __iomem *base = gpio_base + 32u * bank;
	u32 v;

	unsigned long flags;

	/*
	 * Four per-bank registers, each shared by eight pads. Held across
	 * the whole group so a concurrent pad change cannot land between the
	 * function nibble and the direction bit and leave the pad half
	 * configured.
	 */
	raw_spin_lock_irqsave(&s5l8740_gpio_reg_lock, flags);
	v = readl(base + 0x00);
	writel(((a1 & 0xfu) << (4u * pin)) | (v & ~(15u << (4u * pin))),
	       base + 0x00);

	v = readl(base + 0x14);
	writel((((a1 >> 12) & 1u) << pin) | (v & ~BIT(pin)), base + 0x14);

	v = readl(base + 0x0c);
	writel((((a1 >> 4) & 1u) << pin) | (v & ~BIT(pin)), base + 0x0c);

	v = readl(base + 0x10);
	writel((((a1 >> 8) & 1u) << pin) | (v & ~BIT(pin)), base + 0x10);
	raw_spin_unlock_irqrestore(&s5l8740_gpio_reg_lock, flags);
}

static void s5l8740_pinmux_223C(struct device *dev, void __iomem *gpio_base)
{
	unsigned int i;
	void __iomem *eic;

	s5l8740_pinmux_apply_word(gpio_base, 0x0C03000Fu);
	for (i = 0; i < ARRAY_SIZE(k_pinmux_table); i++)
		s5l8740_pinmux_apply_word(gpio_base, k_pinmux_table[i]);

	/* EIC mask-all (SEC sub_223C @0x39700080…E0) */
	eic = ioremap(0x39700000ul, 0x100);
	if (eic) {
		for (i = 0; i <= 6; i++) {
			writel(0, eic + 0x80 + 4 * i);
			writel(0xffffffffu, eic + 0xa0 + 4 * i);
			writel(0, eic + 0xc0 + 4 * i);
			writel(0, eic + 0xe0 + 4 * i);
		}
		iounmap(eic);
	}

	s5l8740_pinmux_apply_word(gpio_base, 0x0C041100u);
	/* SEC busy(~0x1F4); approximate with short delay */
	udelay(500);
	s5l8740_pinmux_apply_word(gpio_base, 0x0C040000u);

	writel(1377685u, gpio_base + 0x380);
	writel(1, gpio_base + 0x388);
	writel(1, gpio_base + 0x3f4);
	writel(1, gpio_base + 0x3e0);

	dev_info(dev, "SEC pinmux sub_223C applied (%u table words)\n",
		 (unsigned int)ARRAY_SIZE(k_pinmux_table));
}

#define S5L8740_NKEYS	2
/* Re-entries before a key is judged to be misconfigured, not pressed. */
#define S5L8740_KEY_STORM_MAX	64

struct s5l8740_key {
	struct s5l8740_gpio *sg;
	struct delayed_work release;
	const char *name;
	unsigned int gpio;
	unsigned int code;
	int irq;
	bool down;
	bool masked;
	unsigned int storm;
};

struct s5l8740_gpio {
	void __iomem *base;
	void __iomem *gpiocmd;
	struct gpio_chip gc;
	struct irq_domain *eic_domain;
	struct timer_list din_timer;
	struct work_struct poweroff_work;
	struct input_dev *input;
	u8 last40, last41, last86;
	bool din_inited;
	struct s5l8740_key keys[S5L8740_NKEYS];
	bool keys_on_irq;
};

static struct s5l8740_gpio *s5l8740_n31;

/* ------------------------------------------------------------------ */
/* Pad-function debugfs                                                 */
/*                                                                      */
/* gpiolib covers direction and value, but not the pad function nibble, */
/* which is what most N31 bring-up questions are actually about: which  */
/* peripheral owns a pin right now. Reading it previously meant mapping */
/* /dev/mem from userspace, which is an easy way to mistake a tool bug  */
/* for a hardware finding.                                              */
/*                                                                      */
/*   pads      one line per bank: PCON word then the eight nibbles      */
/*   pad_set   "<gpio> <func>" -- 14/15 drive an output low/high,       */
/*             0-7 select a peripheral function, 0xFFFE releases to in  */
/* ------------------------------------------------------------------ */

#define S5L8740_GPIO_BANKS	16
#define S5L8740_GPIO_BANK_STRIDE	32
#define S5L8740_GPIO_DIR	0x14
#define S5L8740_GPIO_RELEASE	0xfffe

static struct dentry *s5l8740_gpio_debugfs;

static int s5l8740_pads_show(struct seq_file *s, void *unused)
{
	struct s5l8740_gpio *sg = s->private;
	unsigned int bank, pin;

	if (!sg || !sg->base)
		return -ENODEV;
	seq_puts(s, "bank PCON     dir      pads 0..7 (function nibble)\n");
	for (bank = 0; bank < S5L8740_GPIO_BANKS; bank++) {
		void __iomem *b = sg->base + S5L8740_GPIO_BANK_STRIDE * bank;
		u32 pcon = readl(b);
		u32 dir = readl(b + S5L8740_GPIO_DIR);

		seq_printf(s, "%-4u %08x %08x ", bank, pcon, dir);
		for (pin = 0; pin < 8; pin++)
			seq_printf(s, "%u%s", (pcon >> (4 * pin)) & 0xf,
				   pin == 7 ? "" : " ");
		seq_printf(s, "   (gpio %u-%u)\n", bank * 8, bank * 8 + 7);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(s5l8740_pads);

static ssize_t s5l8740_pad_set_write(struct file *file,
				     const char __user *ubuf,
				     size_t len, loff_t *ppos)
{
	struct s5l8740_gpio *sg = file_inode(file)->i_private;
	char buf[32];
	unsigned int gpio, func, bank, pin;
	u32 dir;

	if (!sg || !sg->base || !sg->gpiocmd)
		return -ENODEV;
	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;
	if (sscanf(buf, "%u %i", &gpio, &func) != 2)
		return -EINVAL;
	if (gpio >= S5L8740_GPIO_BANKS * 8)
		return -EINVAL;

	bank = gpio >> 3;
	pin = gpio & 7;
	dir = readl(sg->base + S5L8740_GPIO_BANK_STRIDE * bank +
		    S5L8740_GPIO_DIR);
	if (func == S5L8740_GPIO_RELEASE)
		dir &= ~BIT(pin);
	else
		dir |= BIT(pin);
	writel(dir, sg->base + S5L8740_GPIO_BANK_STRIDE * bank +
		    S5L8740_GPIO_DIR);
	writel((bank << 16) | (pin << 8) |
	       (func == S5L8740_GPIO_RELEASE ? 0 : (func & 0xff)),
	       sg->gpiocmd);

	/* Echo of a debugfs pad write: tracing. */
	dev_dbg(sg->gc.parent, "pad gpio %u (bank %u pin %u) -> func %u\n",
		gpio, bank, pin, func);
	return len;
}

static const struct file_operations s5l8740_pad_set_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = s5l8740_pad_set_write,
	.llseek = noop_llseek,
};

static void s5l8740_gpio_debugfs_init(struct s5l8740_gpio *sg)
{
	struct dentry *d = debugfs_create_dir("s5l8740_gpio", NULL);

	if (IS_ERR(d))
		return;
	s5l8740_gpio_debugfs = d;
	debugfs_create_file("pads", 0444, d, sg, &s5l8740_pads_fops);
	debugfs_create_file("pad_set", 0200, d, sg, &s5l8740_pad_set_fops);
}


void (*d1830_n31_din_nirq_hook)(void);
EXPORT_SYMBOL_GPL(d1830_n31_din_nirq_hook);

/*
 * Report a key on n31-buttons. Only for keys this driver actually sources;
 * PMIC-sourced keys go out on n31-pmic-buttons instead, so that a single
 * physical press produces a single event.
 */
void s5l8740_n31_report_key(unsigned int code, int pressed)
{
	if (!s5l8740_n31 || !s5l8740_n31->input)
		return;
	input_report_key(s5l8740_n31->input, code, pressed);
	input_sync(s5l8740_n31->input);
}
EXPORT_SYMBOL_GPL(s5l8740_n31_report_key);

static void __iomem *s5l8740_bank(struct s5l8740_gpio *sg, unsigned int offset)
{
	return sg->base + S5L8740_GPIO_BANK_STRIDE * (offset >> 3);
}

/* RetailOS sub_43D38C(gpio, mode, val) */
static void s5l8740_gpiocmd_mode(struct s5l8740_gpio *sg, unsigned int gpio,
				 u16 mode, int val)
{
	void __iomem *bank = s5l8740_bank(sg, gpio);
	u32 pin = gpio & 7;
	unsigned long flags;
	u32 dir;
	u8 cmd;

	if (gpio == 200)
		return;

	if (mode == 1)
		cmd = val ? S5L8740_CMD_OUT_HIGH : S5L8740_CMD_OUT_LOW;
	else if (mode == 0xFFFE)
		cmd = 0;
	else
		cmd = (u8)mode;

	/*
	 * DIR follows the mode for everything except 0xFFFE, mode 1
	 * included. sub_43D38C picks command 14 or 15 for mode 1 and then
	 * falls through to the shared "DIR |= bit"; only the 0xFFFE arm
	 * branches away to "DIR &= ~bit":
	 *
	 *     if (a2 == 1) { v8 = a3 ? 15 : 14; }
	 *     else { ... if (a2 == 65534) { v9 = DIR & ~v7; goto L10; } }
	 *     v9 = DIR | v7;
	 *   L10: DIR = v9; GPIOCMD = ...;
	 *
	 * This used to skip DIR for mode 1, with a comment claiming stock
	 * did the same. It does not, and the cost was that every output
	 * this chip drove stayed an input: the Bluetooth controller's
	 * REG_ON read back "in lo" in debugfs and the part never powered
	 * up, so hci0 bound to a dead chip and every command timed out.
	 *
	 * Eight pads share one DIR word, so this is a read-modify-write
	 * under the same lock as every other writer to the bank.
	 */
	raw_spin_lock_irqsave(&s5l8740_gpio_reg_lock, flags);
	dir = readl(bank + S5L8740_GPIO_DIR_OFF);
	if (mode == 0xFFFE)
		dir &= ~BIT(pin);
	else
		dir |= BIT(pin);
	writel(dir, bank + S5L8740_GPIO_DIR_OFF);
	writel(((gpio >> 3) << 16) | (pin << 8) | cmd, sg->gpiocmd);
	raw_spin_unlock_irqrestore(&s5l8740_gpio_reg_lock, flags);
}

static int s5l8740_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct s5l8740_gpio *sg = gpiochip_get_data(gc);
	u32 din = readl(s5l8740_bank(sg, offset) + S5L8740_GPIO_DIN_OFF);

	return !!(din & BIT(offset & 7));
}

static void s5l8740_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct s5l8740_gpio *sg = gpiochip_get_data(gc);

	/* mode==1 — cmd 14/15 only (DIR untouched per sub_43D38C) */
	s5l8740_gpiocmd_mode(sg, offset, 1, value);
}

static void __maybe_unused s5l8740_button_as_input(struct s5l8740_gpio *sg,
						   unsigned int gpio,
						   u32 pinmux_word)
{
	void __iomem *bank = s5l8740_bank(sg, gpio);
	u32 pin = gpio & 7;
	u32 v;

	/* SEC PCON/PUNB. Bit8 (PUNC/+0x10) is 0 in the table = pull-down;
	 * Vol± are pull-up active-low, so force +0x0c and +0x10. */
	s5l8740_pinmux_apply_word(sg->base, pinmux_word | 0x00000100u);
	v = readl(bank + 0x0c);
	writel(v | BIT(pin), bank + 0x0c);
	v = readl(bank + 0x10);
	writel(v | BIT(pin), bank + 0x10);
	s5l8740_gpiocmd_mode(sg, gpio, 0xFFFE, 0);
}

static int s5l8740_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct s5l8740_gpio *sg = gpiochip_get_data(gc);

	/* Vol± / nIRQ: do not re-pinmux or GPIOCMD 0xFFFE — last image did
	 * that and DIN never moved. Leave SEC / U-Boot pad state.
	 */
	if (offset == 40 || offset == 41 || offset == 86)
		return 0;
	s5l8740_gpiocmd_mode(sg, offset, 0xFFFE, 0);
	return 0;
}

static int s5l8740_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
					 int value)
{
	s5l8740_gpio_set(gc, offset, value);
	return 0;
}

static int s5l8740_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct s5l8740_gpio *sg = gpiochip_get_data(gc);
	u32 dir = readl(s5l8740_bank(sg, offset) + S5L8740_GPIO_DIR_OFF);

	if (dir & BIT(offset & 7))
		return GPIO_LINE_DIRECTION_OUT;
	return GPIO_LINE_DIRECTION_IN;
}

static int s5l8740_gpio_to_irq(struct gpio_chip *gc, unsigned int offset)
{
	struct s5l8740_gpio *sg = gpiochip_get_data(gc);
	int ret, virq;

	if (!sg->eic_domain)
		return -ENXIO;

	/*
	 * Stock arms an interrupt-capable pad with sub_43D38C(gpio, 0, 1)
	 * followed by sub_428F70(gpio, 1), which sets the bank's +0x0C bit.
	 * Without that second step the pad is muxed but its input stage is
	 * not enabled, so the EIC has nothing to level-detect.
	 */
	{
		void __iomem *b = s5l8740_bank(sg, offset);
		unsigned int pin = offset & 7;

		writel(readl(b + S5L8740_GPIO_INEN_OFF) | BIT(pin),
		       b + S5L8740_GPIO_INEN_OFF);
	}

	ret = s5l8740_eic_enable_gpio(offset, IRQ_TYPE_LEVEL_LOW);
	if (ret)
		return ret;

	virq = irq_create_mapping(sg->eic_domain, offset);
	if (!virq)
		return -EINVAL;
	return virq;
}

static u8 s5l8740_din_bit(struct s5l8740_gpio *sg, unsigned int gpio)
{
	u32 din = readl(s5l8740_bank(sg, gpio) + S5L8740_GPIO_DIN_OFF);

	return !!(din & BIT(gpio & 7));
}

/*
 * Pinmux ownership (SEC 223C table + every OSOS 43D38C immediate):
 *   SEC nibbles are only 0 / 2 / 4 / 14. No IIC/IIS-specific nibble.
 *   OSOS never GPIOCMDs IIC0/IIC1 — IIC1 works from SEC/WTF leftover.
 *   OSOS 5714EE is UART pairs func2: (4,5)(78,79)(66,67)(83,84).
 *   OSOS 20690 is SPI2: 87/5, 88/3, 89/3, 90/3.
 *   Grape: 14 EN, 39 RST, 38 IRQ. Vol 40/41. nIRQ 86.
 *   IIS0: OSOS BCB60 GPIOCMD 7 and 20 only (mode 3=on, 2=off).
 *   IIS1/IIS2: no 43D38C. Do not treat 21-22/49-54/57-63 as IIS.
 *   IIC0/IIC1: no named SCL/SDA GPIO; no PUNB/PUNC. Clock IIC1 =
 *   PWRCON1 bit 6 (SEC 2308). Do not invent IIC pulls.
 * Do not replay 20690 or 5714EE in GATE0. Do not 0xFFFE IIC/Vol/86.
 */
static void s5l8740_log_pad(struct s5l8740_gpio *sg, unsigned int gpio,
			    char *out, size_t n)
{
	void __iomem *b = s5l8740_bank(sg, gpio);
	u32 pin = gpio & 7;
	u32 pcon = readl(b), din = readl(b + 0x04);
	u32 dir = readl(b + S5L8740_GPIO_DIR_OFF);
	u32 punb = readl(b + 0x0c), punc = readl(b + 0x10);

	snprintf(out, n, "%u:n%x/d%u/i%u/b%u/c%u", gpio,
		 (pcon >> (4 * pin)) & 0xf, !!(dir & BIT(pin)),
		 !!(din & BIT(pin)), !!(punb & BIT(pin)), !!(punc & BIT(pin)));
}

static void s5l8740_log_pads(struct s5l8740_gpio *sg, const char *tag,
			     const char *what, const unsigned int *gpios,
			     unsigned int n)
{
	char buf[160];
	unsigned int i, off = 0;

	off = snprintf(buf, sizeof(buf), "n31-btn pinmux %s %s", tag, what);
	for (i = 0; i < n && off < sizeof(buf) - 36; i++) {
		char p[36];

		s5l8740_log_pad(sg, gpios[i], p, sizeof(p));
		off += snprintf(buf + off, sizeof(buf) - off, " %s", p);
	}
	/*
	 * Pad-state tracing, so dev_dbg rather than dev_err.
	 *
	 * This runs nine times for every IIS hw_params. At err level no
	 * console loglevel can hide it, so a stream start buries whatever
	 * else was being read at the time. Dynamic debug turns it back on
	 * per call site when the pinmux is what is under investigation.
	 */
	dev_dbg(sg->gc.parent, "%s\n", buf);
}

static void s5l8740_log_pinmux_map(struct s5l8740_gpio *sg, const char *tag)
{
	static const unsigned int keys[] = { 40, 41, 86 };
	static const unsigned int spi2[] = { 87, 88, 89, 90 };
	static const unsigned int nim[] = { 14, 38, 39 };
	static const unsigned int spi0[] = { 0, 1, 2, 3 };
	static const unsigned int uart0[] = { 4, 5 };
	static const unsigned int uart1[] = { 78, 79 };
	static const unsigned int uart2[] = { 66, 67 };
	static const unsigned int uart3p[] = { 83, 84 };
	static const unsigned int iis0[] = { 7, 20 };

	s5l8740_log_pads(sg, tag, "key", keys, ARRAY_SIZE(keys));
	s5l8740_log_pads(sg, tag, "spi2", spi2, ARRAY_SIZE(spi2));
	s5l8740_log_pads(sg, tag, "nim", nim, ARRAY_SIZE(nim));
	s5l8740_log_pads(sg, tag, "spi0", spi0, ARRAY_SIZE(spi0));
	s5l8740_log_pads(sg, tag, "uartA", uart0, ARRAY_SIZE(uart0));
	s5l8740_log_pads(sg, tag, "uartB", uart1, ARRAY_SIZE(uart1));
	s5l8740_log_pads(sg, tag, "uartC", uart2, ARRAY_SIZE(uart2));
	s5l8740_log_pads(sg, tag, "uartD", uart3p, ARRAY_SIZE(uart3p));
	s5l8740_log_pads(sg, tag, "iis0", iis0, ARRAY_SIZE(iis0));
}

static void s5l8740_sec_gpio86(struct s5l8740_gpio *sg)
{
	s5l8740_log_pinmux_map(sg, "before-SEC");
	s5l8740_pinmux_apply_word(sg->base, 0x0A061010u);
	/* OSOS BCB60 IIS0 on: 43D38C(20,3) 43D38C(7,3). No IIC GPIOCMD. */
	s5l8740_gpiocmd_mode(sg, 20, 3, 0);
	s5l8740_gpiocmd_mode(sg, 7, 3, 0);
	s5l8740_log_pinmux_map(sg, "after-SEC-86-iis0");
}

/* SEC sub_223C IIS0 ASP pins: func2 + DIR (table @ 0x22004C6C). */
static void s5l8740_iis0_pinmux_sec(struct s5l8740_gpio *sg)
{
	s5l8740_pinmux_apply_word(sg->base, 0x00061002u); /* GPIO6 BCLK? */
	s5l8740_pinmux_apply_word(sg->base, 0x00071002u); /* GPIO7 */
	s5l8740_pinmux_apply_word(sg->base, 0x02041002u); /* GPIO20 */
}

/*
 * OSOS BCB60 IIS0 pad enable. mode 3=on, 2=off (BCB60 teardown).
 * Re-applies SEC func2 on 6/7/20 then GPIOCMD.
 */
void s5l8740_iis0_pads_enable(unsigned int mode)
{
	struct s5l8740_gpio *sg = s5l8740_n31;
	u16 m = mode ? mode : 3;

	if (!sg)
		return;
	s5l8740_iis0_pinmux_sec(sg);
	s5l8740_gpiocmd_mode(sg, 20, m, 0);
	s5l8740_gpiocmd_mode(sg, 7, m, 0);
}
EXPORT_SYMBOL_GPL(s5l8740_iis0_pads_enable);

void s5l8740_iis0_pads_disable(void)
{
	s5l8740_iis0_pads_enable(2);
}
EXPORT_SYMBOL_GPL(s5l8740_iis0_pads_disable);

void s5l8740_iis0_pad6_enable(unsigned int mode)
{
	struct s5l8740_gpio *sg = s5l8740_n31;
	u16 m = mode ? mode : 3;

	if (!sg)
		return;
	s5l8740_pinmux_apply_word(sg->base, 0x00061002u);
	s5l8740_gpiocmd_mode(sg, 6, m, 0);
}
EXPORT_SYMBOL_GPL(s5l8740_iis0_pad6_enable);

/*
 * Set one pad's function nibble and direction, and nothing else.
 *
 * Audio checkpoint-010 records the stock pad state while RetailOS plays:
 * bank0 PCON 0x32112224 / DIR 0xFF and bank2 PCON 0x02230000 / DIR 0x70,
 * which is GPIO6 func2, GPIO7 func3, GPIO20 func3, GPIO21 func2 and GPIO22
 * func2, all outputs. Linux sets 7 and 20 and leaves 6, 21 and 22 alone.
 *
 * That matters because everything upstream now matches the oracle exactly
 * -- TXCON, TXCOM, CLKDIV 272, CLKCON +0x18 and +0x1C, IIS STATUS, the
 * PL080 channel -- and the jack is still silent. Clocks and status can all
 * be right while the serialiser's data pin is not muxed out of the SoC.
 *
 * A whole-word PCON write would take pins 0..5 with it, which is why this
 * is per-pad read-modify-write. The doc is explicit that GPIO6 must not go
 * through GPIOCMD, so the nibble is written directly.
 */
int s5l8740_gpio_set_pad(unsigned int gpio, unsigned int func, bool out)
{
	struct s5l8740_gpio *sg = s5l8740_n31;
	void __iomem *bank;
	unsigned int pin = gpio & 7;
	unsigned long flags;
	u32 v;

	if (!sg || !sg->base || func > 15)
		return -ENODEV;
	bank = sg->base + (gpio >> 3) * S5L8740_GPIO_BANK_STRIDE;

	raw_spin_lock_irqsave(&s5l8740_gpio_reg_lock, flags);
	v = readl(bank + S5L8740_GPIO_PCON_OFF);
	v &= ~(0xfu << (pin * 4));
	v |= (func & 0xf) << (pin * 4);
	writel(v, bank + S5L8740_GPIO_PCON_OFF);

	v = readl(bank + S5L8740_GPIO_DIR_OFF);
	if (out)
		v |= BIT(pin);
	else
		v &= ~BIT(pin);
	writel(v, bank + S5L8740_GPIO_DIR_OFF);
	raw_spin_unlock_irqrestore(&s5l8740_gpio_reg_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(s5l8740_gpio_set_pad);

/*
 * Set one pad's direction bit, and nothing else, under the same lock.
 *
 * PCON, DIR, PUNB and PUNC are per-bank: eight pads share one word, so a
 * read-modify-write on any of them can drop another pad's bit if two
 * owners race. There were two owners -- this driver and s5l8740-i2s.c,
 * which was doing its own unlocked RMW on DIR for the IIS pads. Exporting
 * this gives the audio driver a way to set what it needs without becoming
 * a second writer to a register it does not own.
 */
int s5l8740_gpio_dir_set(unsigned int gpio, bool out);
int s5l8740_gpio_dir_set(unsigned int gpio, bool out)
{
	struct s5l8740_gpio *sg = s5l8740_n31;
	unsigned int pin = gpio & 7;
	unsigned long flags;
	void __iomem *bank;
	u32 v;

	if (!sg || !sg->base)
		return -ENODEV;
	bank = sg->base + (gpio >> 3) * S5L8740_GPIO_BANK_STRIDE;

	raw_spin_lock_irqsave(&s5l8740_gpio_reg_lock, flags);
	v = readl(bank + S5L8740_GPIO_DIR_OFF);
	if (out)
		v |= BIT(pin);
	else
		v &= ~BIT(pin);
	writel(v, bank + S5L8740_GPIO_DIR_OFF);
	raw_spin_unlock_irqrestore(&s5l8740_gpio_reg_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(s5l8740_gpio_dir_set);

void s5l8740_gpio_log_iis0_pads(const char *tag)
{
	struct s5l8740_gpio *sg = s5l8740_n31;

	if (!sg)
		return;
	s5l8740_log_pinmux_map(sg, tag ? tag : "iis0");
}
EXPORT_SYMBOL_GPL(s5l8740_gpio_log_iis0_pads);

int s5l8740_n31_din86(void)
{
	if (!s5l8740_n31)
		return -1;
	return s5l8740_din_bit(s5l8740_n31, 86);
}
EXPORT_SYMBOL_GPL(s5l8740_n31_din86);

static void s5l8740_poweroff_work(struct work_struct *work)
{
	if (pm_power_off)
		pm_power_off();
}

static void s5l8740_key_edge(struct s5l8740_gpio *sg, unsigned int code,
			     u8 now, u8 *last, const char *name)
{
	if (now == *last)
		return;
	if (sg->input) {
		/* Active-low pad: 0 = pressed */
		input_report_key(sg->input, code, now ? 0 : 1);
		input_sync(sg->input);
	}
	/* One line per key edge: tracing, not an error. */
	dev_dbg(sg->gc.parent, "n31-btn %s %s din=%u\n",
		name, now ? "release" : "PRESS", now);
	*last = now;
}

static void s5l8740_din_timer(struct timer_list *t)
{
	struct s5l8740_gpio *sg = container_of(t, struct s5l8740_gpio, din_timer);
	u8 v40 = s5l8740_din_bit(sg, 40);
	u8 v41 = s5l8740_din_bit(sg, 41);
	u8 v86 = s5l8740_din_bit(sg, 86);

	if (!sg->din_inited) {
		sg->last40 = v40;
		sg->last41 = v41;
		sg->last86 = v86;
		sg->din_inited = true;
	} else {
		/* OSOS GPIOButtonManager: only GPIO 40/41. Home/Play/Sleep
		 * are PMIC bits; GPIO 86 is the nIRQ doorbell into d1830.
		 */
		if (!sg->keys_on_irq) {
			s5l8740_key_edge(sg, KEY_VOLUMEUP, v40,
					 &sg->last40, "VOL+");
			s5l8740_key_edge(sg, KEY_VOLUMEDOWN, v41,
					 &sg->last41, "VOL-");
		}
		if (v86 != sg->last86) {
			dev_dbg(sg->gc.parent, "n31-btn NIRQ86 %u->%u\n",
				sg->last86, v86);
			sg->last86 = v86;
			if (d1830_n31_din_nirq_hook)
				d1830_n31_din_nirq_hook();
		}
	}

	mod_timer(&sg->din_timer, jiffies + msecs_to_jiffies(50));
}

/* ------------------------------------------------------------------ */
/* Volume keys on real interrupts                                       */
/*                                                                      */
/* The EIC only offers level-low, so a held key would re-assert forever  */
/* and genirq would retire the line as spurious -- the same failure the  */
/* PMIC event latches caused. The shape that works on a level-only       */
/* irqchip is: take the interrupt for the press, mask the line, then     */
/* poll only while the key is down, and unmask on release.              */
/*                                                                      */
/* So the press is interrupt-driven, which is the part latency is        */
/* visible in, and polling exists only for the tens of milliseconds a    */
/* finger is actually on the button instead of forever at 50 ms.         */
/* ------------------------------------------------------------------ */

/*
 * Off until the EIC level semantics are understood. The VIC routing is
 * now correct, so enabling this actually delivers interrupts -- and an
 * idle active-low pad reads as permanently asserted, which wedges the
 * system. The sweep is slower but it works.
 */
static bool btn_irq;
module_param(btn_irq, bool, 0444);
MODULE_PARM_DESC(btn_irq,
		 "Drive the volume keys from EIC interrupts (default Y)");

static unsigned int btn_release_ms = 30;
module_param(btn_release_ms, uint, 0644);
MODULE_PARM_DESC(btn_release_ms,
		 "Poll interval while a key is held, waiting for release");

static irqreturn_t s5l8740_key_isr(int irq, void *data)
{
	struct s5l8740_key *k = data;
	struct s5l8740_gpio *sg = k->sg;

	/*
	 * Mask before reporting. The pad stays low for as long as the key is
	 * held, so leaving it unmasked here is an instant interrupt storm.
	 */
	if (!k->masked) {
		disable_irq_nosync(irq);
		k->masked = true;
	}
	/*
	 * If the line keeps re-asserting with nobody touching the key, the
	 * polarity or routing is wrong and re-enabling would spin here
	 * forever, taking userspace down with it. Give up instead and say
	 * so; the sweep still reports the key.
	 */
	if (++k->storm > S5L8740_KEY_STORM_MAX) {
		pr_warn_once("n31-btn %s: runaway interrupt, left masked\n",
			     k->name);
		k->sg->keys_on_irq = false;
		return IRQ_HANDLED;
	}
	if (!k->down) {
		k->down = true;
		if (sg->input) {
			input_report_key(sg->input, k->code, 1);
			input_sync(sg->input);
		}
		dev_dbg(sg->gc.parent, "n31-btn %s PRESS (irq)\n", k->name);
	}
	schedule_delayed_work(&k->release,
			      msecs_to_jiffies(btn_release_ms));
	return IRQ_HANDLED;
}

static void s5l8740_key_release_work(struct work_struct *work)
{
	struct s5l8740_key *k = container_of(to_delayed_work(work),
					     struct s5l8740_key, release);
	struct s5l8740_gpio *sg = k->sg;

	/* Pad is active low, so a 1 here means the key came back up. */
	if (!s5l8740_din_bit(sg, k->gpio)) {
		schedule_delayed_work(&k->release,
				      msecs_to_jiffies(btn_release_ms));
		return;
	}
	k->storm = 0;
	if (k->down) {
		k->down = false;
		if (sg->input) {
			input_report_key(sg->input, k->code, 0);
			input_sync(sg->input);
		}
		dev_dbg(sg->gc.parent, "n31-btn %s release\n", k->name);
	}
	if (k->masked) {
		k->masked = false;
		enable_irq(k->irq);
	}
}

/*
 * Returns the number of keys successfully wired. The caller keeps the
 * legacy sweep running if this is not the full set, so a partial or
 * failed setup degrades to the old behaviour instead of losing input.
 */
static unsigned int s5l8740_keys_irq_init(struct s5l8740_gpio *sg)
{
	static const struct {
		unsigned int gpio, code;
		const char *name;
	} want[] = {
		{ 40, KEY_VOLUMEUP,   "VOL+" },
		{ 41, KEY_VOLUMEDOWN, "VOL-" },
	};
	unsigned int i, ok = 0;

	if (!btn_irq || !sg->eic_domain)
		return 0;

	for (i = 0; i < ARRAY_SIZE(want) && i < S5L8740_NKEYS; i++) {
		struct s5l8740_key *k = &sg->keys[i];
		int virq;

		k->sg = sg;
		k->gpio = want[i].gpio;
		k->code = want[i].code;
		k->name = want[i].name;
		INIT_DELAYED_WORK(&k->release, s5l8740_key_release_work);

		virq = s5l8740_gpio_to_irq(&sg->gc, k->gpio);
		if (virq <= 0) {
			dev_info(sg->gc.parent,
				 "key %s: no EIC irq (%d), staying on the sweep\n",
				 k->name, virq);
			continue;
		}
		k->irq = virq;
		if (devm_request_irq(sg->gc.parent, virq, s5l8740_key_isr,
				     IRQF_TRIGGER_LOW | IRQF_SHARED,
				     k->name, k)) {
			dev_info(sg->gc.parent,
				 "key %s: irq %d busy, staying on the sweep\n",
				 k->name, virq);
			k->irq = 0;
			continue;
		}
		ok++;
		dev_info(sg->gc.parent, "key %s on irq %d (EIC level-low)\n",
			 k->name, virq);
	}
	return ok;
}

static struct irq_domain *s5l8740_gpio_find_eic_domain(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *eic_np = NULL;
	struct irq_domain *domain = NULL;

	/* Preferred: DT phandle apple,eic = <&eic> on the gpio node */
	if (np)
		eic_np = of_parse_phandle(np, "apple,eic", 0);

	if (!eic_np && np)
		eic_np = of_parse_phandle(np, "interrupt-parent", 0);

	if (!eic_np && np)
		eic_np = of_irq_find_parent(np);

	if (!eic_np)
		eic_np = of_find_compatible_node(NULL, NULL, "apple,s5l8740-eic");

	if (!eic_np)
		eic_np = of_find_compatible_node(NULL, NULL, "samsung,s5l8740-eic");

	if (eic_np) {
		domain = irq_find_host(eic_np);
		of_node_put(eic_np);
	}

	return domain;
}

static int s5l8740_gpio_probe(struct platform_device *pdev)
{
	struct s5l8740_gpio *sg;
	struct device *dev = &pdev->dev;
	struct resource *res;
	u32 ngpios = S5L8740_GPIO_DEFAULT_NGPIO;
	int ret;

	sg = devm_kzalloc(dev, sizeof(*sg), GFP_KERNEL);
	if (!sg)
		return -ENOMEM;

	sg->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sg->base))
		return PTR_ERR(sg->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res && resource_size(res) > S5L8740_GPIOCMD_OFF)
		sg->gpiocmd = sg->base + S5L8740_GPIOCMD_OFF;
	else
		sg->gpiocmd = devm_ioremap(dev, 0x3cf001e0, 4);
	if (!sg->gpiocmd)
		return -ENOMEM;

	of_property_read_u32(dev->of_node, "ngpios", &ngpios);

	sg->eic_domain = s5l8740_gpio_find_eic_domain(dev);
	if (!sg->eic_domain)
		dev_warn(dev, "EIC irq domain not found — to_irq unavailable\n");

	sg->gc.label = dev_name(dev);
	sg->gc.parent = dev;
	sg->gc.owner = THIS_MODULE;
	sg->gc.base = -1;
	sg->gc.ngpio = ngpios;
	sg->gc.get = s5l8740_gpio_get;
	sg->gc.set = s5l8740_gpio_set;
	sg->gc.direction_input = s5l8740_gpio_direction_input;
	sg->gc.direction_output = s5l8740_gpio_direction_output;
	sg->gc.get_direction = s5l8740_gpio_get_direction;
	sg->gc.to_irq = s5l8740_gpio_to_irq;
	/* Do not set gc.irq.* without a full gpio irqchip — to_irq alone. */

	ret = devm_gpiochip_add_data(dev, &sg->gc, sg);
	if (ret) {
		dev_err(dev, "gpiochip_add failed: %d\n", ret);
		return ret;
	}

	/* Re-apply SEC pinmux so WTF/U-Boot leftovers match cold-boot */
	if (!of_property_read_bool(dev->of_node, "apple,skip-sec-pinmux"))
		s5l8740_pinmux_223C(dev, sg->base);
	else
		s5l8740_sec_gpio86(sg);

	/* Vol± stay as SEC/U-Boot left them. GPIO 86 got its SEC word only. */
	sg->input = devm_input_allocate_device(dev);
	if (sg->input) {
		sg->input->name = "n31-buttons";
		sg->input->phys = "s5l8740/gpio";
		sg->input->dev.parent = dev;
		sg->input->id.bustype = BUS_HOST;
		/*
		 * Vol+/Vol- only. Home, Sleep and Play are PMIC-sourced and
		 * are reported on n31-pmic-buttons; declaring them here too
		 * made one press look like two events to userspace.
		 */
		input_set_capability(sg->input, EV_KEY, KEY_VOLUMEUP);
		input_set_capability(sg->input, EV_KEY, KEY_VOLUMEDOWN);
		if (input_register_device(sg->input))
			sg->input = NULL;
	}

	INIT_WORK(&sg->poweroff_work, s5l8740_poweroff_work);
	/*
	 * GPIO 86 is the PMIC doorbell and still needs watching, so the
	 * sweep runs either way; it just stops carrying the volume keys
	 * once they are on interrupts.
	 */
	sg->keys_on_irq = s5l8740_keys_irq_init(sg) == S5L8740_NKEYS;
	dev_info(dev, "volume keys: %s\n",
		 sg->keys_on_irq ? "EIC interrupts" : "50 ms sweep");
	timer_setup(&sg->din_timer, s5l8740_din_timer, 0);
	mod_timer(&sg->din_timer, jiffies + msecs_to_jiffies(50));
	s5l8740_n31 = sg;
	platform_set_drvdata(pdev, sg);
	s5l8740_gpio_debugfs_init(sg);

	dev_info(dev, "S5L8740 GPIO @%pR ngpios=%u (GPIOCMD @+0x1E0) eic=%s\n",
		 res, ngpios, sg->eic_domain ? "yes" : "no");
	return 0;
}

static void s5l8740_gpio_remove(struct platform_device *pdev)
{
	struct s5l8740_gpio *sg = platform_get_drvdata(pdev);

	if (!sg)
		return;
	s5l8740_n31 = NULL;
	timer_delete_sync(&sg->din_timer);
	cancel_work_sync(&sg->poweroff_work);
}

static const struct of_device_id s5l8740_gpio_of_match[] = {
	{ .compatible = "apple,s5l8740-gpio" },
	{ .compatible = "samsung,s5l8740-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8740_gpio_of_match);

static struct platform_driver s5l8740_gpio_driver = {
	.probe = s5l8740_gpio_probe,
	.remove = s5l8740_gpio_remove,
	.driver = {
		.name = "gpio-s5l8740",
		.of_match_table = s5l8740_gpio_of_match,
	},
};
module_platform_driver(s5l8740_gpio_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8740 banked GPIO + GPIOCMD + EIC to_irq");
MODULE_LICENSE("GPL");
