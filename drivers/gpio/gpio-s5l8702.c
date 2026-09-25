// SPDX-License-Identifier: GPL-2.0
/*
 * GPIO and external interrupt controller driver for Samsung/Apple S5L8702
 * (iPod nano 3rd generation / iPod classic)
 *
 * 16 ports of 8 pins. Each pin has a 4-bit function nibble in PCON:
 *   0x0       input
 *   0x1       output
 *   0x2..0xd  peripheral functions
 *   0xe/0xf   output driven low/high
 * GPIOCMD configures a single pin atomically, which also lets us drive
 * outputs without a read-modify-write of PDAT.
 *
 * GPIO interrupts go through the external interrupt controller (EIC) in
 * the "system alive" block. Its 7 groups of 32 bits are wired to the
 * VIC's EINT6..EINT0 inputs in reverse order; GPIO pins 0..127 land in
 * groups 6..3, which reach VIC0 lines 0..3 (EINT0..3). Groups 0..2 carry
 * SoC-internal sources (alive timer, USB detect) and are not handled here.
 *
 * Register layout and EIC bit mapping from Rockbox
 * (firmware/target/arm/s5l8702/gpio-s5l8702.[ch]).
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define S5L8702_GPIO_NPORTS	16
#define S5L8702_GPIO_NPINS	(S5L8702_GPIO_NPORTS * 8)

#define S5L8702_PCON(port)	((port) * 0x20)
#define S5L8702_PDAT(port)	((port) * 0x20 + 0x04)
#define S5L8702_GPIOCMD		0x200

#define GPIOCMD(port, pin, func) (((port) << 16) | ((pin) << 8) | (func))

#define PCON_FUNC_INPUT		0x0
#define PCON_FUNC_OUTPUT	0x1
#define PCON_FUNC_OUT_LOW	0xe
#define PCON_FUNC_OUT_HIGH	0xf

// EIC registers, relative to the EIC base (group g = 0..6)
#define EIC_INTLEVEL(g)		(0x00 + (g) * 4)	// 0 = low/falling, 1 = high/rising
#define EIC_INTSTAT(g)		(0x20 + (g) * 4)	// write 1 to clear edge interrupts
#define EIC_INTEN(g)		(0x40 + (g) * 4)
#define EIC_INTTYPE(g)		(0x60 + (g) * 4)	// 0 = edge, 1 = level

// The GPIO pins are spread over EIC groups 6..3, one VIC parent each
#define S5L8702_EIC_NPARENTS	4
// The last four bits of group 3 (GPIO 124..127) are not wired
#define S5L8702_EIC_NPINS	124

struct s5l8702_gpio;

struct s5l8702_gpio_parent {
	struct s5l8702_gpio *sg;
	unsigned int index;
	int irq;
};

struct s5l8702_gpio {
	struct gpio_chip gc;
	void __iomem *base;
	void __iomem *eic;
	raw_spinlock_t lock;
	DECLARE_BITMAP(both_edges, S5L8702_GPIO_NPINS);
	struct s5l8702_gpio_parent parents[S5L8702_EIC_NPARENTS];
};

static inline unsigned int s5l8702_eic_group(unsigned int offset)
{
	return 6 - (offset >> 5);
}

// Ports are stored in reverse order within their group's 32 bits. The
// mapping is its own inverse, so it also turns a group bit back into
// the pin offset within the group.
static inline unsigned int s5l8702_eic_bit(unsigned int offset)
{
	return (0x18 - (offset & 0x18)) | (offset & 0x7);
}

static unsigned int s5l8702_gpio_func(struct s5l8702_gpio *sg, unsigned int offset)
{
	u32 pcon = readl(sg->base + S5L8702_PCON(offset / 8));

	return (pcon >> ((offset % 8) * 4)) & 0xf;
}

static int s5l8702_gpio_request(struct gpio_chip *gc, unsigned int offset)
{
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);
	unsigned int func = s5l8702_gpio_func(sg, offset);

	// Don't let a GPIO user take a pin away from a peripheral
	switch (func) {
	case PCON_FUNC_INPUT:
	case PCON_FUNC_OUTPUT:
	case PCON_FUNC_OUT_LOW:
	case PCON_FUNC_OUT_HIGH:
		return 0;
	default:
		dev_dbg(gc->parent, "pin %u is muxed to function %#x\n", offset, func);
		return -EBUSY;
	}
}

static int s5l8702_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);

	switch (s5l8702_gpio_func(sg, offset)) {
	case PCON_FUNC_OUTPUT:
	case PCON_FUNC_OUT_LOW:
	case PCON_FUNC_OUT_HIGH:
		return GPIO_LINE_DIRECTION_OUT;
	default:
		return GPIO_LINE_DIRECTION_IN;
	}
}

static int s5l8702_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);

	return !!(readl(sg->base + S5L8702_PDAT(offset / 8)) & BIT(offset % 8));
}

// Setting a value also (re)configures the pin as an output
static void s5l8702_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);

	writel(GPIOCMD(offset / 8, offset % 8, value ? PCON_FUNC_OUT_HIGH : PCON_FUNC_OUT_LOW),
	       sg->base + S5L8702_GPIOCMD);
}

static int s5l8702_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);

	writel(GPIOCMD(offset / 8, offset % 8, PCON_FUNC_INPUT), sg->base + S5L8702_GPIOCMD);
	return 0;
}

static int s5l8702_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	s5l8702_gpio_set(gc, offset, value);
	return 0;
}

// -----------------------------------------------------------------------
// EIC interrupts

static void s5l8702_eic_rmw(struct s5l8702_gpio *sg, u32 reg, u32 mask, u32 set)
{
	u32 val = readl(sg->eic + reg);

	writel((val & ~mask) | set, sg->eic + reg);
}

// For both-edge interrupts, arm the edge opposite to the current pin level
static void s5l8702_eic_flip_edge(struct s5l8702_gpio *sg, unsigned int offset)
{
	unsigned int group = s5l8702_eic_group(offset);
	u32 bit = BIT(s5l8702_eic_bit(offset));

	s5l8702_eic_rmw(sg, EIC_INTLEVEL(group), bit,
			s5l8702_gpio_get(&sg->gc, offset) ? 0 : bit);
}

static void s5l8702_eic_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	unsigned int group = s5l8702_eic_group(hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&sg->lock, flags);
	writel(BIT(s5l8702_eic_bit(hwirq)), sg->eic + EIC_INTSTAT(group));
	if (test_bit(hwirq, sg->both_edges))
		s5l8702_eic_flip_edge(sg, hwirq);
	raw_spin_unlock_irqrestore(&sg->lock, flags);
}

static void s5l8702_eic_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&sg->lock, flags);
	s5l8702_eic_rmw(sg, EIC_INTEN(s5l8702_eic_group(hwirq)),
			BIT(s5l8702_eic_bit(hwirq)), 0);
	raw_spin_unlock_irqrestore(&sg->lock, flags);

	gpiochip_disable_irq(gc, hwirq);
}

static void s5l8702_eic_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	unsigned long flags;
	u32 bit = BIT(s5l8702_eic_bit(hwirq));

	gpiochip_enable_irq(gc, hwirq);

	raw_spin_lock_irqsave(&sg->lock, flags);
	s5l8702_eic_rmw(sg, EIC_INTEN(s5l8702_eic_group(hwirq)), bit, bit);
	raw_spin_unlock_irqrestore(&sg->lock, flags);
}

static int s5l8702_eic_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	unsigned int group = s5l8702_eic_group(hwirq);
	u32 bit = BIT(s5l8702_eic_bit(hwirq));
	unsigned long flags;
	bool level, high;

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		level = true;
		high = true;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		level = true;
		high = false;
		break;
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_BOTH:
		level = false;
		high = true;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		level = false;
		high = false;
		break;
	default:
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&sg->lock, flags);

	s5l8702_eic_rmw(sg, EIC_INTTYPE(group), bit, level ? bit : 0);
	s5l8702_eic_rmw(sg, EIC_INTLEVEL(group), bit, high ? bit : 0);

	// The EIC has no native both-edge mode: emulate it by flipping polarity
	if (type == IRQ_TYPE_EDGE_BOTH) {
		set_bit(hwirq, sg->both_edges);
		s5l8702_eic_flip_edge(sg, hwirq);
	} else {
		clear_bit(hwirq, sg->both_edges);
	}

	writel(bit, sg->eic + EIC_INTSTAT(group));

	raw_spin_unlock_irqrestore(&sg->lock, flags);

	irq_set_handler_locked(d, level ? handle_level_irq : handle_edge_irq);

	return 0;
}

static const struct irq_chip s5l8702_eic_chip = {
	.name		= "s5l8702-eic",
	.irq_ack	= s5l8702_eic_ack,
	.irq_mask	= s5l8702_eic_mask,
	.irq_unmask	= s5l8702_eic_unmask,
	.irq_set_type	= s5l8702_eic_set_type,
	.flags		= IRQCHIP_IMMUTABLE | IRQCHIP_SKIP_SET_WAKE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void s5l8702_eic_irq_handler(struct irq_desc *desc)
{
	struct s5l8702_gpio_parent *parent = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct s5l8702_gpio *sg = parent->sg;
	unsigned int group = 6 - parent->index;
	unsigned long pending;
	unsigned int bit;

	chained_irq_enter(chip, desc);

	pending = readl(sg->eic + EIC_INTSTAT(group)) & readl(sg->eic + EIC_INTEN(group));
	for_each_set_bit(bit, &pending, 32)
		generic_handle_domain_irq(sg->gc.irq.domain,
					  parent->index * 32 + s5l8702_eic_bit(bit));

	chained_irq_exit(chip, desc);
}

static void s5l8702_eic_init_valid_mask(struct gpio_chip *gc, unsigned long *valid_mask,
					unsigned int ngpios)
{
	bitmap_clear(valid_mask, S5L8702_EIC_NPINS, ngpios - S5L8702_EIC_NPINS);
}

static int s5l8702_eic_hw_init(struct gpio_chip *gc)
{
	struct s5l8702_gpio *sg = gpiochip_get_data(gc);
	unsigned int group;

	// Start with every GPIO interrupt masked and cleared
	for (group = 7 - S5L8702_EIC_NPARENTS; group <= 6; group++) {
		writel(0, sg->eic + EIC_INTEN(group));
		writel(~0U, sg->eic + EIC_INTSTAT(group));
	}

	return 0;
}

static void s5l8702_eic_remove_handlers(void *data)
{
	struct s5l8702_gpio *sg = data;
	unsigned int i;

	for (i = 0; i < S5L8702_EIC_NPARENTS; i++)
		irq_set_chained_handler_and_data(sg->parents[i].irq, NULL, NULL);
}

// -----------------------------------------------------------------------

static int s5l8702_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_irq_chip *girq;
	struct s5l8702_gpio *sg;
	struct clk *clk;
	unsigned int i;
	int ret;

	sg = devm_kzalloc(dev, sizeof(*sg), GFP_KERNEL);
	if (!sg)
		return -ENOMEM;

	raw_spin_lock_init(&sg->lock);

	sg->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sg->base))
		return PTR_ERR(sg->base);

	sg->eic = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(sg->eic))
		return PTR_ERR(sg->eic);

	clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to enable clock\n");

	for (i = 0; i < S5L8702_EIC_NPARENTS; i++) {
		ret = platform_get_irq(pdev, i);
		if (ret < 0)
			return ret;

		sg->parents[i].sg = sg;
		sg->parents[i].index = i;
		sg->parents[i].irq = ret;
	}

	sg->gc.label = dev_name(dev);
	sg->gc.parent = dev;
	sg->gc.owner = THIS_MODULE;
	sg->gc.base = -1;
	sg->gc.ngpio = S5L8702_GPIO_NPINS;
	sg->gc.request = s5l8702_gpio_request;
	sg->gc.get_direction = s5l8702_gpio_get_direction;
	sg->gc.direction_input = s5l8702_gpio_direction_input;
	sg->gc.direction_output = s5l8702_gpio_direction_output;
	sg->gc.get = s5l8702_gpio_get;
	sg->gc.set = s5l8702_gpio_set;

	girq = &sg->gc.irq;
	gpio_irq_chip_set_chip(girq, &s5l8702_eic_chip);
	girq->handler = handle_bad_irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->init_hw = s5l8702_eic_hw_init;
	girq->init_valid_mask = s5l8702_eic_init_valid_mask;
	// The parents are chained below, each with its own handler data
	girq->parent_handler = NULL;
	girq->num_parents = 0;

	ret = devm_gpiochip_add_data(dev, &sg->gc, sg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register GPIO chip\n");

	for (i = 0; i < S5L8702_EIC_NPARENTS; i++)
		irq_set_chained_handler_and_data(sg->parents[i].irq, s5l8702_eic_irq_handler,
						 &sg->parents[i]);

	return devm_add_action_or_reset(dev, s5l8702_eic_remove_handlers, sg);
}

static const struct of_device_id s5l8702_gpio_of_match[] = {
	{ .compatible = "apple,s5l8702-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8702_gpio_of_match);

static struct platform_driver s5l8702_gpio_driver = {
	.probe = s5l8702_gpio_probe,
	.driver = {
		.name = "gpio-s5l8702",
		.of_match_table = s5l8702_gpio_of_match,
	},
};
module_platform_driver(s5l8702_gpio_driver);

MODULE_AUTHOR("Tucker Osman <osmiumusa@gmail.com>");
MODULE_DESCRIPTION("Samsung/Apple S5L8702 GPIO and external interrupt controller driver");
MODULE_LICENSE("GPL");
