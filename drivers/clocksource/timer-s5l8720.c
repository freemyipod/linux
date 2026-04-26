// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 __gsch
 *
 * Timer driver for the Samsung/Apple S5L87xx SoC family.
 *
 * Covers S5L8702 (iPod nano 3G, iPod Classic 6G) and
 *        S5L8720 (iPod nano 4G) and
 *        S5L8740 (iPod nano 7G)
 *
 * The timer block register layout is identical across the family.
 * SoC-specific differences (e.g. clock gating) are handled per-compatible.
 */

#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

/*
 * Timer block layout (each sub-block is 0x20 bytes, one per channel):
 *
 *   Offset  Channel  Width   Use
 *   0x000   A        16-bit  (unused by this driver)
 *   0x020   B        16-bit  (unused)
 *   0x040   C        16-bit  (absent in S5L8702 silicon)
 *   0x060   D        16-bit  (absent in S5L8702 silicon)
 *   0x080   E        64-bit  (undocumented; not used)
 *   0x0A0   F        32-bit  CLOCK EVENT
 *   0x0C0   G        32-bit  CLOCK SOURCE (free-running)
 *   0x0E0   H        32-bit  (unused)
 *   0x100   I        32-bit  (unused)
 *
 * IRQ routing: 16-bit timers share IRQ_TIMER (VIC0 #7);
 *              32-bit timers share IRQ_TIMER32 (VIC0 #8).
 */
#define TIMER_CLKEVT  0x0A0u   /* Timer F — clock event  */
#define TIMER_CLKSRC  0x0C0u   /* Timer G — clock source */

/* Per-channel register offsets */
#define REG_CON    0x00
#define REG_CMD    0x04
#define REG_DATA0  0x08
#define REG_DATA1  0x0C
#define REG_PRE    0x10
#define REG_CNT    0x14

/* TSTAT: 32-bit timer interrupt status/clear (shared, from block base) */
#define REG_TSTAT  0x118

#define CMD_STOP   0u
#define CMD_START  BIT(0)
#define CMD_CLR    BIT(1)

/* TCON bit 12: compare-0 interrupt enable */
#define INT0_EN    BIT(12)

/*
 * Both timers use ECLK as source with no CS division (CS=0b100, ECLK=1),
 * then a prescaler of 11 to get 12 MHz / 12 = 1 MHz.
 */
#define TIMER_CON_ECLK   0x440u  /* ECLK=1 (bit 6), CS=0b100 (bits 10:8) */
#define TIMER_PRE_1MHZ   11u     /* ECLK / (11+1) = 1 MHz */
#define TIMER_RATE       1000000u

#ifdef CONFIG_TIMER_OF

struct s5l8720_tcu {
	void __iomem *base;
};

static struct s5l8720_tcu *s5l8720_tcu;

/* --- Clocksource --- */

static u64 notrace s5l8720_read_sched_clock(void)
{
	return readl(s5l8720_tcu->base + TIMER_CLKSRC + REG_CNT);
}

/* --- Clock event --- */

static int s5l8720_clkevt_set_next_event(unsigned long delta,
					  struct clock_event_device *ce)
{
	struct s5l8720_tcu *t = s5l8720_tcu;
	u32 tcon = readl(t->base + TIMER_CLKEVT + REG_CON);

	writel(tcon & ~INT0_EN, t->base + TIMER_CLKEVT + REG_CON);
	writel(CMD_STOP,       t->base + TIMER_CLKEVT + REG_CMD);
	writel(delta,          t->base + TIMER_CLKEVT + REG_DATA0);
	writel(CMD_START | CMD_CLR, t->base + TIMER_CLKEVT + REG_CMD);
	writel(tcon | INT0_EN, t->base + TIMER_CLKEVT + REG_CON);
	return 0;
}

static int s5l8720_clkevt_shutdown(struct clock_event_device *ce)
{
	writel(CMD_STOP, s5l8720_tcu->base + TIMER_CLKEVT + REG_CMD);
	return 0;
}

static int s5l8720_clkevt_set_oneshot(struct clock_event_device *ce)
{
	struct s5l8720_tcu *t = s5l8720_tcu;

	writel(CMD_STOP, t->base + TIMER_CLKEVT + REG_CMD);
	/* MODE_SEL=2 (one-shot) | ECLK | CS=0b100 */
	writel(TIMER_CON_ECLK | 0x40, t->base + TIMER_CLKEVT + REG_CON);
	writel(TIMER_PRE_1MHZ,        t->base + TIMER_CLKEVT + REG_PRE);
	return 0;
}

static irqreturn_t s5l8720_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *ce = dev_id;
	u32 stat = readl(s5l8720_tcu->base + REG_TSTAT);

	writel(stat, s5l8720_tcu->base + REG_TSTAT);
	ce->event_handler(ce);
	return IRQ_HANDLED;
}

static struct clock_event_device s5l8720_clockevent = {
	.name			= "s5l87xx-timerF",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.set_state_shutdown	= s5l8720_clkevt_shutdown,
	.set_state_oneshot	= s5l8720_clkevt_set_oneshot,
	.set_next_event		= s5l8720_clkevt_set_next_event,
	.rating			= 300,
};

/*
 * s5l8720_timer_common_init - shared init for all SoC variants.
 *
 * Assumes Timer G's clock gate is already open (either by the bootloader
 * or by the SoC-specific wrapper below).
 */
static int __init s5l8720_timer_common_init(struct device_node *np)
{
	struct s5l8720_tcu *tcu;
	int irq, ret;

	tcu = kzalloc(sizeof(*tcu), GFP_KERNEL);
	if (!tcu)
		return -ENOMEM;

	tcu->base = of_iomap(np, 0);
	if (!tcu->base) {
		pr_err("s5l87xx-timer: can't remap registers\n");
		ret = -ENXIO;
		goto out_free;
	}

	s5l8720_tcu = tcu;

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("s5l87xx-timer: can't parse IRQ\n");
		ret = -EINVAL;
		goto out_free;
	}

	/* Initialize Timer G as a free-running 32-bit clocksource at 1 MHz. */
	writel(CMD_STOP,           tcu->base + TIMER_CLKSRC + REG_CMD);
	writel(TIMER_CON_ECLK,     tcu->base + TIMER_CLKSRC + REG_CON);
	writel(TIMER_PRE_1MHZ,     tcu->base + TIMER_CLKSRC + REG_PRE);
	writel(0xFFFFFFFF,         tcu->base + TIMER_CLKSRC + REG_DATA0);
	writel(CMD_START | CMD_CLR, tcu->base + TIMER_CLKSRC + REG_CMD);

	clocksource_mmio_init(tcu->base + TIMER_CLKSRC + REG_CNT,
			      "s5l87xx-timerG", TIMER_RATE,
			      200, 32, clocksource_mmio_readl_up);

	sched_clock_register(s5l8720_read_sched_clock, 32, TIMER_RATE);

	ret = request_irq(irq, s5l8720_timer_interrupt,
			  IRQF_TIMER | IRQF_IRQPOLL,
			  "s5l87xx timer", &s5l8720_clockevent);
	if (ret) {
		pr_err("s5l87xx-timer: failed to request irq %d\n", irq);
		goto out_free;
	}

	clockevents_config_and_register(&s5l8720_clockevent,
					TIMER_RATE, 1, UINT_MAX);
	return 0;

out_free:
	kfree(tcu);
	return ret;
}

/*
 * S5L8702 wrapper: open the secondary clock gate for Timer G before calling
 * common init.  On S5L8702, PWRCON_APB bit 28 is the secondary gate for the
 * timer at offset 0xC0.  This bit is not present (or at a different location)
 * on other family members, so the gate manipulation lives here, not in the
 * common path.
 */
#define S5L8702_PWRCON_APB      0x3C50004CUL
#define S5L8702_TIMER_G_GATE    BIT(28)

static int __init s5l8702_timer_init(struct device_node *np)
{
	void __iomem *pwrcon = ioremap(S5L8702_PWRCON_APB, 4);

	if (pwrcon) {
		writel(readl(pwrcon) & ~S5L8702_TIMER_G_GATE, pwrcon);
		iounmap(pwrcon);
	} else {
		pr_warn("s5l8702-timer: can't ioremap PWRCON_APB, "
			"Timer G gate may be closed\n");
	}

	return s5l8720_timer_common_init(np);
}

TIMER_OF_DECLARE(s5l8702_timer, "samsung,s5l8702-timer", s5l8702_timer_init);
TIMER_OF_DECLARE(s5l8720_timer, "samsung,s5l8720-timer", s5l8720_timer_common_init);

#endif
