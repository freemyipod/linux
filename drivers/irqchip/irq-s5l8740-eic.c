// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 GPIO External Interrupt Controller (EIC) @ 0x39700000
 *
 * Topology (CONFIRMED_N31):
 *   GPIO → EIC → EXT line → PL192 VIC @ 0x38E00000 → CPU
 *
 * Registers (group g = 0..6):
 *   +0x80+4*g  INTLEVEL  0=low, 1=high
 *   +0xA0+4*g  INTSTAT   W1C (edge) / status
 *   +0xC0+4*g  INTEN
 *   +0xE0+4*g  INTTYPE   0=edge, 1=level
 *
 * GPIO → (group, bit): group = gpio >> 5, bit = gpio & 31
 * Group g is parented to VIC EXT irq g (Nimbus GPIO38 → g=1 bit6 → VIC1).
 *
 * RetailOS: sub_7D490 (level), sub_40641C (type+enable+ack).
 *
 * #interrupt-cells = <2> via irq_domain_xlate_twocell (hwirq, flags).
 *
 * Chaining: only chain parents that appear in DT interrupts. The safe N31
 * config is a single parent — EXT1 (interrupts = <1>) for Nimbus GPIO38 /
 * group1 — until multi-EXT chaining is HW-proven. Omitting interrupts =
 * MMIO helper only (s5l8740_eic_enable_gpio / gpio_to_irq).
 */
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define EIC_NGROUPS		7
#define EIC_GPIOS		(EIC_NGROUPS * 32)

#define EIC_INTLEVEL(g)		(0x80 + 4 * (g))
#define EIC_INTSTAT(g)		(0xa0 + 4 * (g))
#define EIC_INTEN(g)		(0xc0 + 4 * (g))
#define EIC_INTTYPE(g)		(0xe0 + 4 * (g))

struct s5l8740_eic {
	void __iomem *base;
	struct irq_domain *domain;
	int parent_irq[EIC_NGROUPS];
};

static struct s5l8740_eic *s5l8740_eic_global;

static void eic_mask(struct irq_data *d)
{
	struct s5l8740_eic *eic = irq_data_get_irq_chip_data(d);
	unsigned int gpio = irqd_to_hwirq(d);
	u32 g = gpio >> 5, bit = gpio & 31;
	u32 en;

	en = readl(eic->base + EIC_INTEN(g));
	writel(en & ~BIT(bit), eic->base + EIC_INTEN(g));
}

static void eic_unmask(struct irq_data *d)
{
	struct s5l8740_eic *eic = irq_data_get_irq_chip_data(d);
	unsigned int gpio = irqd_to_hwirq(d);
	u32 g = gpio >> 5, bit = gpio & 31;
	u32 en;

	/* ack sticky */
	writel(BIT(bit), eic->base + EIC_INTSTAT(g));
	en = readl(eic->base + EIC_INTEN(g));
	writel(en | BIT(bit), eic->base + EIC_INTEN(g));
}

/*
 * Hardware INTLEVEL is one polarity. gpio-keys always requests EDGE_BOTH
 * (IRQF_TRIGGER_RISING|FALLING) → -EINVAL without this. Flip polarity on
 * each ack so press (falling, idle-high) and release (rising) both fire.
 */
static void eic_ack(struct irq_data *d)
{
	struct s5l8740_eic *eic = irq_data_get_irq_chip_data(d);
	unsigned int gpio = irqd_to_hwirq(d);
	u32 g = gpio >> 5, bit = gpio & 31;

	writel(BIT(bit), eic->base + EIC_INTSTAT(g));

	if ((irqd_get_trigger_type(d) & IRQ_TYPE_SENSE_MASK) ==
	    IRQ_TYPE_EDGE_BOTH) {
		u32 level = readl(eic->base + EIC_INTLEVEL(g));

		writel(level ^ BIT(bit), eic->base + EIC_INTLEVEL(g));
	}
}

static int eic_set_type(struct irq_data *d, unsigned int type)
{
	struct s5l8740_eic *eic = irq_data_get_irq_chip_data(d);
	unsigned int gpio = irqd_to_hwirq(d);
	u32 g = gpio >> 5, bit = gpio & 31;
	u32 level, itype;
	unsigned int sense = type & IRQ_TYPE_SENSE_MASK;

	level = readl(eic->base + EIC_INTLEVEL(g));
	itype = readl(eic->base + EIC_INTTYPE(g));

	switch (sense) {
	case IRQ_TYPE_LEVEL_LOW:
		level &= ~BIT(bit);	/* 0 = low */
		itype |= BIT(bit);	/* 1 = level */
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		level |= BIT(bit);
		itype |= BIT(bit);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		level &= ~BIT(bit);
		itype &= ~BIT(bit);	/* 0 = edge */
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_RISING:
		level |= BIT(bit);
		itype &= ~BIT(bit);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		/* Idle-high active-low keys: first event is falling. */
		level &= ~BIT(bit);
		itype &= ~BIT(bit);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	default:
		return -EINVAL;
	}

	writel(level, eic->base + EIC_INTLEVEL(g));
	writel(itype, eic->base + EIC_INTTYPE(g));
	return 0;
}

static struct irq_chip s5l8740_eic_chip = {
	.name		= "s5l8740-eic",
	.irq_ack	= eic_ack,
	.irq_mask	= eic_mask,
	.irq_unmask	= eic_unmask,
	.irq_set_type	= eic_set_type,
	.flags		= IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_SKIP_SET_WAKE |
			  IRQCHIP_SET_TYPE_MASKED,
};

static void eic_chained_handler(struct irq_desc *desc)
{
	struct s5l8740_eic *eic = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int g, parent = irq_desc_get_irq(desc);
	u32 stat, en, pending;
	unsigned int bit;

	chained_irq_enter(chip, desc);

	for (g = 0; g < EIC_NGROUPS; g++) {
		if (eic->parent_irq[g] != parent)
			continue;
		stat = readl(eic->base + EIC_INTSTAT(g));
		en = readl(eic->base + EIC_INTEN(g));
		pending = stat & en;
		for_each_set_bit(bit, (unsigned long *)&pending, 32) {
			unsigned int virq = irq_find_mapping(eic->domain,
							    (g << 5) | bit);
			static unsigned hits;

			if (hits < 8) {
				hits++;
				pr_debug("EIC hit g%u b%u virq=%u\n", g, bit,
					 virq);
			}
			if (virq)
				generic_handle_irq(virq);
			/* W1C ack */
			writel(BIT(bit), eic->base + EIC_INTSTAT(g));
		}
	}

	chained_irq_exit(chip, desc);
}

static int eic_domain_map(struct irq_domain *d, unsigned int irq,
			  irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &s5l8740_eic_chip, handle_level_irq);
	irq_set_chip_data(irq, d->host_data);
	irq_set_probe(irq);
	return 0;
}

static const struct irq_domain_ops eic_domain_ops = {
	.map = eic_domain_map,
	.xlate = irq_domain_xlate_twocell,
};

/* Export for early consumers (nimbus) before domain lookup */
int s5l8740_eic_enable_gpio(unsigned int gpio, unsigned int irq_type)
{
	struct s5l8740_eic *eic = s5l8740_eic_global;
	u32 g, bit;
	u32 level, itype, en;

	if (!eic || gpio >= EIC_GPIOS)
		return -ENODEV;

	g = gpio >> 5;
	bit = gpio & 31;

	level = readl(eic->base + EIC_INTLEVEL(g));
	itype = readl(eic->base + EIC_INTTYPE(g));

	if (irq_type & IRQ_TYPE_LEVEL_HIGH)
		level |= BIT(bit);
	else
		level &= ~BIT(bit); /* default active-low */

	if (irq_type & (IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING))
		itype &= ~BIT(bit);
	else
		itype |= BIT(bit); /* level */

	writel(level, eic->base + EIC_INTLEVEL(g));
	writel(itype, eic->base + EIC_INTTYPE(g));
	writel(BIT(bit), eic->base + EIC_INTSTAT(g));
	en = readl(eic->base + EIC_INTEN(g));
	writel(en | BIT(bit), eic->base + EIC_INTEN(g));
	return 0;
}
EXPORT_SYMBOL_GPL(s5l8740_eic_enable_gpio);

/**
 * s5l8740_eic_gpio_to_irq - create/return Linux IRQ for an SoC GPIO line
 * @gpio: SoC GPIO number (group = gpio>>5, bit = gpio&31)
 */
int s5l8740_eic_gpio_to_irq(unsigned int gpio)
{
	struct s5l8740_eic *eic = s5l8740_eic_global;
	int virq;

	if (!eic || !eic->domain || gpio >= EIC_GPIOS)
		return -ENODEV;

	virq = irq_create_mapping(eic->domain, gpio);
	if (!virq)
		return -EINVAL;
	return virq;
}
EXPORT_SYMBOL_GPL(s5l8740_eic_gpio_to_irq);

static int s5l8740_eic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l8740_eic *eic;
	int g, nirq, ngrp, i, ret, chained = 0;

	eic = devm_kzalloc(dev, sizeof(*eic), GFP_KERNEL);
	if (!eic)
		return -ENOMEM;

	eic->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(eic->base))
		return PTR_ERR(eic->base);

	/* Mask all, clear status (same as SEC pinmux_223C) */
	for (g = 0; g < EIC_NGROUPS; g++) {
		writel(0, eic->base + EIC_INTEN(g));
		writel(0xffffffff, eic->base + EIC_INTSTAT(g));
		writel(0, eic->base + EIC_INTLEVEL(g));
		writel(0, eic->base + EIC_INTTYPE(g));
		eic->parent_irq[g] = -1;
	}

	eic->domain = irq_domain_add_linear(dev->of_node, EIC_GPIOS,
					    &eic_domain_ops, eic);
	if (!eic->domain)
		return -ENOMEM;

	nirq = of_irq_count(dev->of_node);
	if (nirq < 0)
		nirq = 0;
	if (nirq > EIC_NGROUPS)
		nirq = EIC_NGROUPS;

	ngrp = of_property_count_u32_elems(dev->of_node, "apple,eic-groups");
	if (ngrp < 0)
		ngrp = 0;

	/*
	 * Chain only the VIC EXTn parents listed in DT. Map each to an EIC
	 * group via apple,eic-groups (parallel to interrupts). GPIO 86 is
	 * group 2 (86>>5) on VIC EXT3 = 3 — do not chain every EXT0..6
	 * (that hung boot).
	 */
	for (i = 0; i < nirq; i++) {
		u32 group = i;

		ret = platform_get_irq(pdev, i);
		if (ret < 0)
			continue;
		if (ngrp == nirq)
			of_property_read_u32_index(dev->of_node,
						   "apple,eic-groups", i,
						   &group);
		if (group >= EIC_NGROUPS)
			continue;
		eic->parent_irq[group] = ret;
		irq_set_chained_handler_and_data(ret, eic_chained_handler, eic);
		chained++;
		dev_info(dev, "EIC group%u <- VIC irq %d\n", group, ret);
	}

	s5l8740_eic_global = eic;
	platform_set_drvdata(pdev, eic);
	dev_info(dev,
		 "EIC @%pR groups=%d dt_irqs=%d chained_parents=%d (prefer EXT1-only)\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0), EIC_NGROUPS,
		 nirq, chained);
	return 0;
}

static const struct of_device_id s5l8740_eic_of_match[] = {
	{ .compatible = "apple,s5l8740-eic" },
	{ .compatible = "samsung,s5l8740-eic" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8740_eic_of_match);

static struct platform_driver s5l8740_eic_driver = {
	.probe = s5l8740_eic_probe,
	.driver = {
		.name = "s5l8740-eic",
		.of_match_table = s5l8740_eic_of_match,
	},
};
builtin_platform_driver(s5l8740_eic_driver);

MODULE_DESCRIPTION("S5L8740 GPIO EIC (External Interrupt Controller)");
MODULE_LICENSE("GPL");
