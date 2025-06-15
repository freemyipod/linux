// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 __gsch
 *
 * S5L8720 timer driver, based on code by Sergiusz 'q3k' Bazanski
 */

 #include <linux/clk.h>
 #include <linux/clocksource.h>
 #include <linux/clockchips.h>
 #include <linux/of.h>
 #include <linux/init.h>
 #include <linux/interrupt.h>
 #include <linux/of_address.h>
 #include <linux/of_irq.h>
 #include <linux/sched_clock.h>
 
 #define TIMER_E     0xA0
 
 #define REG_CON     0x00
 #define REG_CMD     0x04
 #define REG_DATA0   0x08
 #define REG_DATA1   0x0C
 #define REG_PRE     0x10
 #define REG_CNT     0x14
 
 #define REG_IRQSTAT 0x10000
 #define REG_IRQLATCH 0x118
 
 #define CMD_STOP  (0<<0)
 #define CMD_START (1<<0)
 #define CMD_CLR   (1<<1)

 #define INT0_EN   (1<<12)

 #define TIMER_64    0x80

 #define TM64_CNTH	 0x00
 #define TM64_CNTL	 0x04


 #define S5L8720_TIMERE_RATE  1000000
 #define S5L8720_TIMER64_RATE 2000000
 
 #ifdef CONFIG_TIMER_OF
 
 struct s5l8720_tcu {
	void __iomem *base;
};

static struct s5l8720_tcu *s5l8720_tcu;
 
 static inline void s5l8720_timer_disable(struct s5l8720_tcu *timer)
 {
     writel(CMD_STOP, timer->base + TIMER_E + REG_CMD);
 }
 
 static inline void s5l8720_timer_enable(struct s5l8720_tcu *timer)
 {
     writel(CMD_START | CMD_CLR, timer->base + TIMER_E + REG_CMD);
 }
 
 static inline void s5l8720_timer_ack(struct s5l8720_tcu *timer)
 {
     u32 stat;
 
     stat = readl(timer->base + REG_IRQSTAT);
     writel(stat, timer->base + REG_IRQLATCH);
 }
 
 static u64 s5l8720_clocksource_read(struct clocksource *c)
 {
    struct s5l8720_tcu *timer = s5l8720_tcu;
    u32 high, low, h_tmp;
    high = readl(timer->base + TIMER_64 + TM64_CNTH);
    low = readl(timer->base + TIMER_64 + TM64_CNTL);
    h_tmp = readl(timer->base + TIMER_64 + TM64_CNTH);
    if ( high != h_tmp )
        low = readl(timer->base + TIMER_64 + TM64_CNTL);
 
     return low + ((u64)high << 32);
 }
 
 static u64 notrace s5l8720_read_sched_clock(void)
 {
     return s5l8720_clocksource_read(NULL);
 }
 
 static int s5l8720_clkevt_set_next_event(unsigned long delta, struct clock_event_device *ce) {
     struct s5l8720_tcu *timer = s5l8720_tcu;
     u32 tcon;
     tcon = readl(timer->base + TIMER_E + REG_CON);
     writel(tcon & ~INT0_EN, timer->base + TIMER_E + REG_CON);
     s5l8720_timer_disable(timer);
     writel(delta, timer->base + TIMER_E + REG_DATA0);
     s5l8720_timer_enable(timer);
     writel(tcon | INT0_EN, timer->base + TIMER_E + REG_CON);
     return 0;
 }
 
 static int s5l8720_clkevt_shutdown(struct clock_event_device *ce) {
     struct s5l8720_tcu *timer = s5l8720_tcu;
     s5l8720_timer_disable(timer);
     return 0;
 };
 
 static int s5l8720_clkevt_set_oneshot(struct clock_event_device *ce) {
    struct s5l8720_tcu *timer = s5l8720_tcu;
     s5l8720_timer_disable(timer);
     writel(0x440, timer->base + TIMER_E + REG_CON);
     writel(23, timer->base + TIMER_E + REG_PRE);
     return 0;
 };
 
 static irqreturn_t s5l8720_timer_interrupt(int irq, void *dev_id) {
     struct clock_event_device *ce = dev_id;
     struct s5l8720_tcu *timer = s5l8720_tcu;
 
     s5l8720_timer_ack(timer);
     ce->event_handler(ce);
 
     return IRQ_HANDLED;
 }
 
 static struct clock_event_device s5l8720_clockevent = {
	.name			= "timerE",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.set_state_shutdown	= s5l8720_clkevt_shutdown,
	.set_state_oneshot	= s5l8720_clkevt_set_oneshot,
	.set_next_event		= s5l8720_clkevt_set_next_event,
	.rating			= 300,
 };

 static int __init s5l8720_timer_init(struct device_node *np)
 {
	int irq;
	unsigned long flags = IRQF_TIMER | IRQF_IRQPOLL;
	struct s5l8720_tcu *tcu;
	int ret;

	tcu = kzalloc(sizeof(*tcu), GFP_KERNEL);
	if (!tcu)
		return -ENOMEM;

	tcu->base = of_iomap(np, 0);
	if (!tcu->base) {
		pr_err("Can't remap registers\n");
		ret = -ENXIO;
		goto out_free;
	}

	s5l8720_tcu = tcu;

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		ret = -EINVAL;
		pr_err("S5L8720 Timer Can't parse IRQ %d", irq);
		goto out_free;
	}

    // [TODO] Enable clock gates and configure TIMER64 here
    // instead of trusting the bootrom/bootloader
    // TIMER64 is configured by bootloader in IpodSec

	// /* Enable and register clocksource and sched_clock on timer 64 */
	clocksource_mmio_init(NULL, "timer64",
                S5L8720_TIMER64_RATE, 200, 64,
				s5l8720_clocksource_read);
	sched_clock_register(s5l8720_read_sched_clock, 64,
                S5L8720_TIMER64_RATE);

	/* Set up clockevent on timer E */
	if (request_irq(irq, s5l8720_timer_interrupt, flags, "s5l8720 timer",
		&s5l8720_clockevent))
		pr_err("Failed to request irq %d (s5l8720 timer)\n", irq);

	clockevents_config_and_register(&s5l8720_clockevent,
                S5L8720_TIMERE_RATE,
				1,
				UINT_MAX);

	return 0;

out_free:
	kfree(tcu);
	return ret;
}
 
 TIMER_OF_DECLARE(s5l8720_timer, "samsung,s5l8720-timer", s5l8720_timer_init);
 
 #endif