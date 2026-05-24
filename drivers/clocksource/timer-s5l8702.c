// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Tucker Osman
 * Timer driver for the Samsung/Apple S5L8702 SoC
 * (iPod nano 3rd Generation, iPod classic 6th Generation)
 *
 * The timer block register layout is shared with the S5L8720/S5L8740 family.
 * The S5L8702-specific difference is an extra clock gate in PWRCON_APB that
 * must be opened before Timer G (used as clocksource) will tick.
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

struct s5l8702_tcu {
	void __iomem *base;
};

static struct s5l8702_tcu *s5l8702_tcu;

/* --- Clocksource --- */

static u64 notrace s5l8702_read_sched_clock(void)
{
	return readl(s5l8702_tcu->base + TIMER_CLKSRC + REG_CNT);
}

/* --- Clock event --- */

static int s5l8702_clkevt_set_next_event(unsigned long delta,
					  struct clock_event_device *ce)
{
	struct s5l8702_tcu *t = s5l8702_tcu;
	u32 tcon = readl(t->base + TIMER_CLKEVT + REG_CON);

	writel(tcon & ~INT0_EN, t->base + TIMER_CLKEVT + REG_CON);
	writel(CMD_STOP,        t->base + TIMER_CLKEVT + REG_CMD);
	writel(delta,           t->base + TIMER_CLKEVT + REG_DATA0);
	writel(CMD_START | CMD_CLR, t->base + TIMER_CLKEVT + REG_CMD);
	writel(tcon | INT0_EN,  t->base + TIMER_CLKEVT + REG_CON);
	return 0;
}

static int s5l8702_clkevt_shutdown(struct clock_event_device *ce)
{
	writel(CMD_STOP, s5l8702_tcu->base + TIMER_CLKEVT + REG_CMD);
	return 0;
}

static int s5l8702_clkevt_set_oneshot(struct clock_event_device *ce)
{
	struct s5l8702_tcu *t = s5l8702_tcu;

	writel(CMD_STOP, t->base + TIMER_CLKEVT + REG_CMD);
	/* MODE_SEL=2 (one-shot) | ECLK | CS=0b100 */
	writel(TIMER_CON_ECLK | 0x40, t->base + TIMER_CLKEVT + REG_CON);
	writel(TIMER_PRE_1MHZ,        t->base + TIMER_CLKEVT + REG_PRE);
	return 0;
}

static irqreturn_t s5l8702_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *ce = dev_id;
	u32 stat = readl(s5l8702_tcu->base + REG_TSTAT);

	writel(stat, s5l8702_tcu->base + REG_TSTAT);
	ce->event_handler(ce);
	return IRQ_HANDLED;
}

static struct clock_event_device s5l8702_clockevent = {
	.name			= "s5l8702-timerF",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.set_state_shutdown	= s5l8702_clkevt_shutdown,
	.set_state_oneshot	= s5l8702_clkevt_set_oneshot,
	.set_next_event		= s5l8702_clkevt_set_next_event,
	.rating			= 300,
};

/*
 * On S5L8702, PWRCON_APB bit 28 is a secondary clock gate for Timer G
 * (offset 0xC0). Clear it before bringing up the clocksource.
 */
#define S5L8702_PWRCON_APB    0x3C50004CUL
#define S5L8702_TIMER_G_GATE  BIT(28)

static int __init s5l8702_timer_init(struct device_node *np)
{
	struct s5l8702_tcu *tcu;
	void __iomem *pwrcon;
	int irq, ret;

	pwrcon = ioremap(S5L8702_PWRCON_APB, 4);
	if (pwrcon) {
		writel(readl(pwrcon) & ~S5L8702_TIMER_G_GATE, pwrcon);
		iounmap(pwrcon);
	} else {
		pr_warn("s5l8702-timer: can't ioremap PWRCON_APB, "
			"Timer G gate may be closed\n");
	}

	tcu = kzalloc(sizeof(*tcu), GFP_KERNEL);
	if (!tcu)
		return -ENOMEM;

	tcu->base = of_iomap(np, 0);
	if (!tcu->base) {
		pr_err("s5l8702-timer: can't remap registers\n");
		ret = -ENXIO;
		goto out_free;
	}

	s5l8702_tcu = tcu;

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("s5l8702-timer: can't parse IRQ\n");
		ret = -EINVAL;
		goto out_free;
	}

	/* Initialize Timer G as a free-running 32-bit clocksource at 1 MHz. */
	writel(CMD_STOP,            tcu->base + TIMER_CLKSRC + REG_CMD);
	writel(TIMER_CON_ECLK,      tcu->base + TIMER_CLKSRC + REG_CON);
	writel(TIMER_PRE_1MHZ,      tcu->base + TIMER_CLKSRC + REG_PRE);
	writel(0xFFFFFFFF,          tcu->base + TIMER_CLKSRC + REG_DATA0);
	writel(CMD_START | CMD_CLR, tcu->base + TIMER_CLKSRC + REG_CMD);

	clocksource_mmio_init(tcu->base + TIMER_CLKSRC + REG_CNT,
			      "s5l8702-timerG", TIMER_RATE,
			      200, 32, clocksource_mmio_readl_up);

	sched_clock_register(s5l8702_read_sched_clock, 32, TIMER_RATE);

	ret = request_irq(irq, s5l8702_timer_interrupt,
			  IRQF_TIMER | IRQF_IRQPOLL,
			  "s5l8702 timer", &s5l8702_clockevent);
	if (ret) {
		pr_err("s5l8702-timer: failed to request irq %d\n", irq);
		goto out_free;
	}

	clockevents_config_and_register(&s5l8702_clockevent,
					TIMER_RATE, 1, UINT_MAX);
	return 0;

out_free:
	kfree(tcu);
	return ret;
}

TIMER_OF_DECLARE(s5l8702_timer, "samsung,s5l8702-timer", s5l8702_timer_init);

#endif
