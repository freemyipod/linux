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
#define S5L8740_GPIO_PUNC_OFF		0x10
#define S5L8740_GPIO_DIR_OFF		0x14

/*
 * Guards every read-modify-write of a per-bank GPIO register. PCON, DIR,
 * PUNB and PUNC each pack eight pads into one word, so an unlocked RMW can
 * silently drop a bit another caller just set.
 */
static DEFINE_RAW_SPINLOCK(s5l8740_gpio_reg_lock);
#define S5L8740_GPIOCMD_OFF		0x1e0

/*
 * 15 banks of 8, so 120 pads, and the GPIOCMD offset is the proof.
 *
 * sub_43D38C forms a bank pointer as base + 32 * (pad >> 3) and latches
 * GPIOCMD at base + 0x1E0. 0x1E0 is 32 * 15, i.e. exactly where a bank 15
 * would begin, so the banked pad registers can only occupy banks 0..14 and
 * the block's remaining 0x400 - 0x1E0 is command and status space, not more
 * pads. The bootloader's pinmux table agrees independently: 120 packed words
 * covering banks 0x00 through 0x0E, then a zero terminator.
 *
 * Pad 200 is a "no pad" sentinel, not a 26th bank. sub_43D38C and sub_57056C
 * each open with an equality test against 200 and return having touched
 * nothing (43D38E: cmp r0, #0xc8 / beq), so every stock sub_43D38C(200, ...)
 * -- including the pair in sub_17D4DC that a comment here used to call a
 * Bluetooth power control -- is a no-op. Tables that hold a pad number per
 * board variant use 200 for "this variant does not have one"; sub_17D4DC's
 * other branch names pad 70, which is real.
 */
#define S5L8740_GPIO_PADS		120
#define S5L8740_GPIO_DEFAULT_NGPIO	S5L8740_GPIO_PADS

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
	/*
	 * sub_347C(0x1F4) waits 1 ms, and this used to wait half of it.
	 *
	 * It is not a magic "short delay": sub_347C spins on the counter at
	 * 0x3C700084 -- the same one sub_41D558 reads inside the firmware
	 * download -- until the delta exceeds 2 * a1, so 0x1F4 is 1000 ticks.
	 * A tick is one microsecond: the millisecond form of the same helper,
	 * at 0x8983AFA, converts its argument for this counter as
	 * (125 * ms) << 3 = 1000 ticks per millisecond, and caps it at
	 * 0x00418937 = 4295479 ms, which is 2^32 microseconds.
	 *
	 * So this is 1000 us. udelay(500) was the "approximate with a short
	 * delay" that stood here, and it was half the wait stock takes across
	 * a strap read.
	 */
	udelay(1000);
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

/*
 * Sweep cadence, from the stock GPIO button task sub_E10BC (0xE10BC): its
 * wait on RTOS event 72 times out after 200 ms when every watched pad is
 * settled, and after 5 ms while any of them is still moving.
 */
#define S5L8740_DIN_IDLE_MS	200
#define S5L8740_DIN_SETTLE_MS	5

/* The pads this driver sweeps, in the order of struct s5l8740_gpio::din. */
enum {
	S5L8740_DIN_VOLUP,
	S5L8740_DIN_VOLDN,
	S5L8740_DIN_NIRQ,
	S5L8740_DIN_COUNT,
};

/*
 * One pad's debounce state, laid out like stock's three per-pad words.
 *
 * sub_E10BC keeps a raw shadow at 0x8929F6C that follows every sample and
 * restamps 0x8929F70 whenever it moves, and a reported shadow at 0x8929F68
 * that only advances once the elapsed-time test passes. Only the second
 * shadow moving calls sub_4195D8.
 */
struct s5l8740_din {
	u8 sample;		/* 0x8929F6C: the last raw DIN bit read */
	u8 reported;		/* 0x8929F68: the last level handed on */
	unsigned long stamp;	/* 0x8929F70: when @sample last changed */
};

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
	struct work_struct nirq_work;
	struct input_dev *input;
	struct s5l8740_din din[S5L8740_DIN_COUNT];
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

int s5l8740_gpio_input_enable(unsigned int gpio, bool enable);

static int s5l8740_gpio_to_irq_sense(struct s5l8740_gpio *sg,
				     unsigned int offset, unsigned int sense)
{
	int ret, virq;

	if (!sg->eic_domain)
		return -ENXIO;

	/*
	 * Stock arms an interrupt-capable pad with sub_43D38C(gpio, 0, 1)
	 * followed by sub_428F70(gpio, 1), which sets the bank's +0x0C bit.
	 * Without that second step the pad is muxed but its input stage is
	 * not enabled, so the EIC has nothing to detect.
	 *
	 * sub_428F70 is more than that one bit, so this goes through the
	 * helper that reproduces all of it rather than setting +0x0C alone.
	 */
	ret = s5l8740_gpio_input_enable(offset, true);
	if (ret)
		return ret;

	ret = s5l8740_eic_enable_gpio(offset, sense);
	if (ret)
		return ret;

	virq = irq_create_mapping(sg->eic_domain, offset);
	if (!virq)
		return -EINVAL;
	return virq;
}

static int s5l8740_gpio_to_irq(struct gpio_chip *gc, unsigned int offset)
{
	return s5l8740_gpio_to_irq_sense(gpiochip_get_data(gc), offset,
					 IRQ_TYPE_LEVEL_LOW);
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

/*
 * The rest of sub_223C, for the pads nothing in this kernel owns.
 *
 * Replaying the whole table is not an option -- it would put pads 87-90,
 * 70, 78-82 and 97-99 back to their cold-boot functions underneath SPI2,
 * Bluetooth and two serial peripherals that have since claimed them. That
 * is not a reason to skip the table, though, only a reason to apply it a
 * pad at a time. Diffing all 121 words against a live dump of the fifteen
 * banks leaves 31 pads that differ; naming an owner for each of them
 * leaves these, and only these, in a state stock reaches and we never do:
 *
 *   99, 100  the strap sub_223C pulses either side of its 500 us wait,
 *            reading GPIO +0x404 in between. Pad 99 goes high before the
 *            table and low after it; pad 100 takes DIR and PUNC, is read,
 *            and is released.
 *   64       PUNC only. Nothing claims the pad and no capture of bank 8
 *            exists to say otherwise, so the bootloader's value stands.
 *
 * The four block registers at the end are the same case: SEC writes them
 * once at cold boot and our chain never has.
 *
 * Two things sub_223C does are deliberately not here. The 121-word loop is
 * covered above. The EIC mask-all belongs to irq-s5l8740-eic, which has
 * configured that block by the time this runs; SEC does it before any OS
 * exists, and repeating it from a driver probe would take out the Vol±
 * interrupts, which is what made the whole table get skipped in the first
 * place.
 */
#define S5L8740_GPIO_STRAP_STATUS	0x404

static void s5l8740_sec_strap_and_block(struct s5l8740_gpio *sg)
{
	u32 strap;

	/* 0x0C03000F then 0x0C03000E: pad 99 high across the table, then low. */
	s5l8740_pinmux_apply_word(sg->base, 0x0C03000Fu);
	s5l8740_pinmux_apply_word(sg->base, 0x0C03000Eu);

	/* 0x08001002: pad 64, PUNC cleared as the bootloader leaves it. */
	s5l8740_pinmux_apply_word(sg->base, 0x08001002u);

	/*
	 * 0x0C041100, sub_347C(0x1F4), read +0x404, 0x0C040000. 1000 ticks of
	 * the counter at 0x3C700084 at one microsecond each; see the
	 * derivation in s5l8740_pinmux_223C().
	 */
	s5l8740_pinmux_apply_word(sg->base, 0x0C041100u);
	udelay(1000);
	strap = readl(sg->base + S5L8740_GPIO_STRAP_STATUS);
	s5l8740_pinmux_apply_word(sg->base, 0x0C040000u);

	writel(1377685u, sg->base + 0x380);
	writel(1, sg->base + 0x388);
	writel(1, sg->base + 0x3f4);
	writel(1, sg->base + 0x3e0);

	dev_info(sg->gc.parent, "SEC sub_223C strap read +0x404 = 0x%08x\n",
		 strap);
}

static void s5l8740_sec_gpio86(struct s5l8740_gpio *sg)
{
	s5l8740_log_pinmux_map(sg, "before-SEC");
	s5l8740_pinmux_apply_word(sg->base, 0x0A061010u);
	/* OSOS BCB60 IIS0 on: 43D38C(20,3) 43D38C(7,3). No IIC GPIOCMD. */
	s5l8740_gpiocmd_mode(sg, 20, 3, 0);
	s5l8740_gpiocmd_mode(sg, 7, 3, 0);
	s5l8740_sec_strap_and_block(sg);
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

/*
 * sub_428F70 -- enable one pad's input stage, the way stock does it.
 *
 * sub_428F70 splits the pad and tail-calls sub_428F76, which builds a packed
 * pad descriptor, fills it from the live bank through sub_72F4, flips one bit
 * of it, and writes every field back through sub_7358:
 *
 *	sub_72F4     w = (bank << 24) | (pin << 16)
 *	                 | ((DIR  >> pin) & 1) << 12
 *	                 | ((PUNC >> pin) & 1) <<  8
 *	                 | ((INEN >> pin) & 1) <<  4
 *	                 | nibble
 *	             nibble = (PCON >> 4 * pin) & 0xf, remapped from 1 to
 *	             DIN ? 15 : 14
 *	sub_428F76   w = (w & ~0x10) | (enable ? 0x10 : 0)
 *	sub_7358     PCON nibble = w & 0xf;  DIR bit = w >> 12;
 *	             INEN bit    = w >> 4;   PUNC bit = w >> 8
 *
 * Only bit 4 -- the bank's +0x0C input-enable bit -- carries new information.
 * DIR and PUNC go back with the value they were just read with, and they are
 * written here anyway because sub_7358 writes them unconditionally and this
 * is not the place to decide a store the hardware was shipped with is
 * redundant.
 *
 * PCON is the field that genuinely changes: a nibble of 1 -- plain output --
 * comes back as 14 or 15 according to the pad's current DIN, which is the
 * same drive-low / drive-high encoding sub_43D38C latches through GPIOCMD.
 * Enabling the input stage on a pad that is currently an output therefore
 * also rewrites its mode into the explicit drive form, and omitting that
 * would leave the pad in a state stock never leaves it in.
 *
 * Pads at or above S5L8740_GPIO_PADS are refused, which stock does not do:
 * sub_428F70 has no sentinel guard where sub_43D38C and sub_57056C both have
 * one, so stock reaches this arithmetic with pad 200 from sub_15DD5C's FM
 * power-on and from sub_17D4DC, computes base + 0x320, and stores into space
 * the block does not decode. That write is discarded on hardware. Refusing
 * it here loses nothing and keeps pads 120..127, whose bank base would be
 * the GPIOCMD latch itself, from being written through this path.
 */
int s5l8740_gpio_input_enable(unsigned int gpio, bool enable)
{
	struct s5l8740_gpio *sg = s5l8740_n31;
	unsigned int pin = gpio & 7;
	unsigned int shift = 4 * pin;
	void __iomem *bank;
	unsigned long flags;
	u32 pcon, nibble, v;

	if (!sg || !sg->base)
		return -ENODEV;
	if (gpio >= S5L8740_GPIO_PADS)
		return -EINVAL;
	bank = s5l8740_bank(sg, gpio);

	raw_spin_lock_irqsave(&s5l8740_gpio_reg_lock, flags);
	pcon = readl(bank + S5L8740_GPIO_PCON_OFF);
	nibble = (pcon >> shift) & 0xf;
	if (nibble == 1)
		nibble = (readl(bank + S5L8740_GPIO_DIN_OFF) & BIT(pin)) ?
			 S5L8740_CMD_OUT_HIGH : S5L8740_CMD_OUT_LOW;
	writel((pcon & ~(0xfu << shift)) | (nibble << shift),
	       bank + S5L8740_GPIO_PCON_OFF);

	v = readl(bank + S5L8740_GPIO_DIR_OFF);
	writel(v, bank + S5L8740_GPIO_DIR_OFF);

	v = readl(bank + S5L8740_GPIO_INEN_OFF);
	if (enable)
		v |= BIT(pin);
	else
		v &= ~BIT(pin);
	writel(v, bank + S5L8740_GPIO_INEN_OFF);

	v = readl(bank + S5L8740_GPIO_PUNC_OFF);
	writel(v, bank + S5L8740_GPIO_PUNC_OFF);
	raw_spin_unlock_irqrestore(&s5l8740_gpio_reg_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(s5l8740_gpio_input_enable);

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

/*
 * How long a level must hold before it is believed.
 *
 * sub_E10BC never reports the first sample of a new level: the raw shadow
 * takes it, the timestamp is refreshed, and sub_4195D8 is reached only on a
 * later pass whose elapsed-time test passes. The threshold itself is not
 * recoverable from the image -- sub_345D68 is a thunk to 0x22000FE6, outside
 * OSOS -- so the number below is this port's, not stock's. What is stock is
 * the shape: a level seen once is never an event, and the re-check comes
 * 5 ms later.
 *
 * This mattered because the sweep used to report the first differing sample
 * straight to the input device, which turns one disturbed read of a DIN word
 * into a press and a release that nobody performed.
 */
static unsigned int btn_debounce_ms = 20;
module_param(btn_debounce_ms, uint, 0644);
MODULE_PARM_DESC(btn_debounce_ms,
		 "How long a pad level must hold before it is reported (0=off)");

/*
 * Fold one sample into a pad's two shadows, the inner half of sub_E10BC.
 *
 * Returns true when the pad is settled -- either unchanged, or changed and
 * now old enough to believe. @changed says whether ->reported just moved,
 * which is the only condition under which stock emits an event.
 */
static bool s5l8740_din_settle(struct s5l8740_din *d, u8 now, bool *changed)
{
	*changed = false;

	if (now != d->sample) {
		/* 0x8929F6C moves and 0x8929F70 is restamped; no event. */
		d->sample = now;
		d->stamp = jiffies;
		return false;
	}
	if (d->sample == d->reported)
		return true;
	if (btn_debounce_ms &&
	    time_before(jiffies, d->stamp + msecs_to_jiffies(btn_debounce_ms)))
		return false;

	/* 0x8929F68 moves: this is the sub_4195D8 call site. */
	d->reported = d->sample;
	*changed = true;
	return true;
}

static void s5l8740_key_edge(struct s5l8740_gpio *sg, unsigned int code,
			     u8 level, const char *name)
{
	if (sg->input) {
		/*
		 * sub_E10BC calls sub_4195D8(id, table[2] != din) and both
		 * table entries at 0x87891B8 carry 1, so a pad reading 0 is
		 * a press.
		 */
		input_report_key(sg->input, code, level ? 0 : 1);
		input_sync(sg->input);
	}
	/* One line per key edge: tracing, not an error. */
	dev_dbg(sg->gc.parent, "n31-btn %s %s din=%u\n",
		name, level ? "release" : "PRESS", level);
}

/*
 * The PMIC doorbell, off the timer.
 *
 * d1830_n31_din_nirq() reads four I2C registers, which sleeps, and the sweep
 * runs in softirq context -- so this used to be a sleeping call from a timer
 * callback. It is also a second, unsynchronised sampler of the same PMIC
 * registers the PMIC driver's own poll reads, and two samplers interleaving
 * their read-compare-commit can apply a stale sample after a fresh one and
 * manufacture a key transition out of a real one.
 */
static void s5l8740_nirq_work(struct work_struct *work)
{
	void (*hook)(void) = READ_ONCE(d1830_n31_din_nirq_hook);

	if (hook)
		hook();
}

static void s5l8740_din_timer(struct timer_list *t)
{
	struct s5l8740_gpio *sg = container_of(t, struct s5l8740_gpio, din_timer);
	/* OSOS GPIOButtonManager (0x87891B8): only GPIO 40 and 41 are keys.
	 * Home, Play and Sleep are PMIC status bits; GPIO 86 is the PMIC
	 * nIRQ doorbell, which sub_EFBB4 services while it reads 0.
	 */
	static const unsigned int gpios[S5L8740_DIN_COUNT] = { 40, 41, 86 };
	u8 now[S5L8740_DIN_COUNT];
	bool settled = true;
	unsigned int i;

	for (i = 0; i < S5L8740_DIN_COUNT; i++)
		now[i] = s5l8740_din_bit(sg, gpios[i]);

	if (!sg->din_inited) {
		for (i = 0; i < S5L8740_DIN_COUNT; i++) {
			sg->din[i].sample = now[i];
			sg->din[i].reported = now[i];
			sg->din[i].stamp = jiffies;
		}
		sg->din_inited = true;
		goto rearm;
	}

	for (i = 0; i < S5L8740_DIN_COUNT; i++) {
		bool changed;

		if (!s5l8740_din_settle(&sg->din[i], now[i], &changed))
			settled = false;
		if (!changed)
			continue;

		switch (i) {
		case S5L8740_DIN_VOLUP:
			s5l8740_key_edge(sg, KEY_VOLUMEUP,
					 sg->din[i].reported, "VOL+");
			break;
		case S5L8740_DIN_VOLDN:
			s5l8740_key_edge(sg, KEY_VOLUMEDOWN,
					 sg->din[i].reported, "VOL-");
			break;
		case S5L8740_DIN_NIRQ:
			dev_dbg(sg->gc.parent, "n31-btn NIRQ86 -> %u\n",
				sg->din[i].reported);
			/*
			 * sub_EFBB4 drains the PMIC while sub_42BBEC(0x56)
			 * reads 0 and only re-arms once it reads 1, so the
			 * de-asserting edge is not a reason to go and look.
			 */
			if (!sg->din[i].reported)
				schedule_work(&sg->nirq_work);
			break;
		}
	}

rearm:
	mod_timer(&sg->din_timer,
		  jiffies + msecs_to_jiffies(settled ? S5L8740_DIN_IDLE_MS
						     : S5L8740_DIN_SETTLE_MS));
}

/* ------------------------------------------------------------------ */
/* Volume keys on real interrupts                                       */
/*                                                                      */
/* Stock does not report a key from the interrupt. sub_E10BC waits on    */
/* RTOS event 72, and whatever wakes it -- the EIC line or the 200 ms    */
/* timeout -- it then reads DIN through sub_42BBEC and runs the two      */
/* shadows before it will call sub_4195D8. The interrupt is a wakeup.    */
/*                                                                      */
/* So the handler here masks its line and pulls the sweep forward to     */
/* stock's 5 ms settle interval. It does not report, because it has not  */
/* read the pad: an interrupt that fires for any other reason -- a       */
/* mis-set polarity, a shared line, a level source that never            */
/* deasserts -- would otherwise become a keypress out of nothing.        */
/* ------------------------------------------------------------------ */

/*
 * Off by default, and the description used to claim otherwise.
 *
 * The reason it had to be off is now understood: this path armed the pads
 * with IRQ_TYPE_LEVEL_LOW, which eic_encode_sense() programs as INTLEVEL
 * clear plus INTTYPE set -- and a clear INTLEVEL asserts while the pad is
 * HIGH, which is how a released volume key looks. Stock arms these two pads
 * as an edge source instead: sub_E10BC calls sub_5D308(gpio, 2, 0), which is
 * INTLEVEL set and INTTYPE clear. s5l8740_keys_irq_init() now asks for that,
 * but the default stays off until a flash confirms it, because getting EIC
 * polarity wrong wedges the system rather than merely failing.
 */
static bool btn_irq;
module_param(btn_irq, bool, 0444);
MODULE_PARM_DESC(btn_irq,
		 "Drive the volume keys from EIC interrupts (default N)");

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
	k->down = true;
	dev_dbg(sg->gc.parent, "n31-btn %s wake (irq)\n", k->name);
	/*
	 * Hand the pad to the sweep rather than reporting from here. That is
	 * stock's split, and it means a spurious interrupt costs one early
	 * DIN read instead of a keypress userspace cannot distinguish from a
	 * real one.
	 */
	mod_timer(&sg->din_timer,
		  jiffies + msecs_to_jiffies(S5L8740_DIN_SETTLE_MS));
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
	k->down = false;
	/*
	 * The release event belongs to the sweep, which owns the shadows for
	 * this pad. All this has to do is put the line back.
	 */
	if (k->masked) {
		k->masked = false;
		enable_irq(k->irq);
	}
}

/*
 * Stock's pad arming for the two volume keys, in stock's order.
 *
 * sub_E10BC walks the table at 0x87891B8 -- {id 8, gpio 0x28, 1} and
 * {id 7, gpio 0x29, 1} -- and for each entry runs:
 *
 *	sub_43D38C(gpio, 0, 1)	function nibble 0, DIR bit set
 *	sub_428F70(gpio, 1)	bank +0x0C bit set, the input stage
 *	sub_5D308(gpio, 2, 0)	INTLEVEL set, INTTYPE clear, INTEN, INTSTAT
 *	sub_7D490(gpio, 2)	INTLEVEL set again
 *
 * The last two steps are s5l8740_keys_irq_init()'s EDGE_FALLING arm. The
 * first two are here, and off by default: an earlier image re-muxed these
 * pads and recorded that DIN then stopped moving. That note is not evidence
 * and the device cannot be retested from here, but the sweep works on
 * whatever SEC and U-Boot leave behind, so this stays available rather than
 * automatic. Set btn_arm_pads=1 on a flash to settle it in one boot.
 */
static bool btn_arm_pads;
module_param(btn_arm_pads, bool, 0444);
MODULE_PARM_DESC(btn_arm_pads,
		 "Replay sub_E10BC's volume-key pad arming (default N)");

static void s5l8740_keys_arm_pads(struct s5l8740_gpio *sg)
{
	static const unsigned int gpios[] = { 40, 41 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		void __iomem *b = s5l8740_bank(sg, gpios[i]);
		unsigned int pin = gpios[i] & 7;
		unsigned long flags;

		s5l8740_gpiocmd_mode(sg, gpios[i], 0, 1);
		raw_spin_lock_irqsave(&s5l8740_gpio_reg_lock, flags);
		writel(readl(b + S5L8740_GPIO_INEN_OFF) | BIT(pin),
		       b + S5L8740_GPIO_INEN_OFF);
		raw_spin_unlock_irqrestore(&s5l8740_gpio_reg_lock, flags);
	}
	dev_info(sg->gc.parent, "volume pads armed per sub_E10BC\n");
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

	/*
	 * Populate the array unconditionally: remove() cancels these works,
	 * and cancelling one that was never initialised is not something the
	 * workqueue API forgives.
	 */
	for (i = 0; i < ARRAY_SIZE(want) && i < S5L8740_NKEYS; i++) {
		struct s5l8740_key *k = &sg->keys[i];

		k->sg = sg;
		k->gpio = want[i].gpio;
		k->code = want[i].code;
		k->name = want[i].name;
		INIT_DELAYED_WORK(&k->release, s5l8740_key_release_work);
	}

	if (!btn_irq || !sg->eic_domain)
		return 0;

	for (i = 0; i < ARRAY_SIZE(want) && i < S5L8740_NKEYS; i++) {
		struct s5l8740_key *k = &sg->keys[i];
		int virq;

		/*
		 * sub_5D308(gpio, 2, 0) -- INTLEVEL set, INTTYPE clear --
		 * is what sub_E10BC arms these two pads with, so ask for the
		 * sense that eic_encode_sense() programs that way. The old
		 * IRQ_TYPE_LEVEL_LOW is the opposite pair and leaves a
		 * released, idle-high key looking permanently asserted.
		 */
		virq = s5l8740_gpio_to_irq_sense(sg, k->gpio,
						 IRQ_TYPE_EDGE_FALLING);
		if (virq <= 0) {
			dev_info(sg->gc.parent,
				 "key %s: no EIC irq (%d), staying on the sweep\n",
				 k->name, virq);
			continue;
		}
		k->irq = virq;
		/*
		 * Not IRQF_SHARED. Each pad gets its own EIC hwirq, and
		 * declaring the line shared would let a handler run for an
		 * interrupt raised by some other pad -- which, before this
		 * handler stopped reporting, was a press with no press.
		 */
		if (devm_request_irq(sg->gc.parent, virq, s5l8740_key_isr,
				     IRQF_TRIGGER_FALLING, k->name, k)) {
			dev_info(sg->gc.parent,
				 "key %s: irq %d busy, staying on the sweep\n",
				 k->name, virq);
			k->irq = 0;
			continue;
		}
		ok++;
		dev_info(sg->gc.parent, "key %s on irq %d (EIC falling edge)\n",
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
	INIT_WORK(&sg->nirq_work, s5l8740_nirq_work);
	timer_setup(&sg->din_timer, s5l8740_din_timer, 0);

	if (btn_arm_pads)
		s5l8740_keys_arm_pads(sg);
	/*
	 * The sweep owns the key events either way. GPIO 86 needs watching,
	 * and stock reads DIN and debounces even when the EIC line is armed,
	 * so an interrupt only buys an earlier read.
	 */
	sg->keys_on_irq = s5l8740_keys_irq_init(sg) == S5L8740_NKEYS;
	dev_info(dev, "volume keys: %u ms sweep%s\n", S5L8740_DIN_IDLE_MS,
		 sg->keys_on_irq ? ", woken by EIC" : "");
	mod_timer(&sg->din_timer,
		  jiffies + msecs_to_jiffies(S5L8740_DIN_SETTLE_MS));
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
	unsigned int i;

	if (!sg)
		return;
	s5l8740_n31 = NULL;
	timer_delete_sync(&sg->din_timer);
	/*
	 * The release works hold a pointer to devm memory and re-arm
	 * themselves while a key is down, so they have to be stopped here
	 * rather than left for devm to outlive.
	 */
	for (i = 0; i < S5L8740_NKEYS; i++)
		cancel_delayed_work_sync(&sg->keys[i].release);
	cancel_work_sync(&sg->nirq_work);
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
