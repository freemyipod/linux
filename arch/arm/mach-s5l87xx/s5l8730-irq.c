// SPDX-License-Identifier: GPL-2.0-only

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/irq.h>

#include <linux/irqchip.h>

// TODO: put these to dts
#define SRCPND       (*((uint32_t volatile*)(0x39C00000)))
#define INTMOD       (*((uint32_t volatile*)(0x39C00004)))
#define INTMSK       (*((uint32_t volatile*)(0x39C00008)))
#define INTPRIO      (*((uint32_t volatile*)(0x39C0000C)))
#define INTPND       (*((uint32_t volatile*)(0x39C00010)))
#define INTOFFSET    (*((uint32_t volatile*)(0x39C00014)))
#define EINTPOL      (*((uint32_t volatile*)(0x39C00018)))
#define EINTPEND     (*((uint32_t volatile*)(0x39C0001C)))
#define EINTMSK      (*((uint32_t volatile*)(0x39C00020)))

void s5l87xx_irq_unmask(struct irq_data *data)
{
    INTMSK |= (1 << data->hwirq);
}

void s5l87xx_irq_mask(struct irq_data *data)
{
    INTMSK &= ~(1 << data->hwirq);
}

void s5l87xx_irq_maskack(struct irq_data *data)
{
    s5l87xx_irq_mask(data);

    SRCPND = (1 << data->hwirq);
    INTPND = INTPND;
}

static struct irq_chip  s5l8700_irq_chip = {
    .name = "s5l8700-irq",
    .irq_mask   = s5l87xx_irq_mask,
    .irq_unmask = s5l87xx_irq_unmask,
};

struct irq_domain	*domain;

static void s5l8700_handle_irq(struct pt_regs *regs)
{
    int irq_num = INTOFFSET;
    generic_handle_domain_irq(domain, irq_num);
    SRCPND = (1 << irq_num);
    INTPND = INTPND;
}

static int s5l8700_irq_map_of(struct irq_domain *h, unsigned int irq,
							irq_hw_number_t hw)
{
    irq_set_chip_and_handler(irq, &s5l8700_irq_chip, handle_level_irq);
    irq_set_chip_data(irq, NULL);
    irq_set_probe(irq);

	return 0;
}

static const struct irq_domain_ops s5l8700_irq_ops_of = {
	.map = s5l8700_irq_map_of,
	.xlate = irq_domain_xlate_onecell,
};

static int __init s5l8700_init_intc_of(struct device_node *np,
			struct device_node *interrupt_parent)
{
    // TODO: there are "sub" interrupts for the timers & dma.
	domain = irq_domain_add_linear(np, 32,
						     &s5l8700_irq_ops_of, NULL);

    set_handle_irq(s5l8700_handle_irq);

    return 0;
}

IRQCHIP_DECLARE(s5l8700_irq, "samsung,s5l8700-irq", s5l8700_init_intc_of);
