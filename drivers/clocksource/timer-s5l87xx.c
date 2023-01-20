// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 20223 Sergiusz 'q3k' Bazanski
 *
 * S5L87XX timer driver, 16-bit and 32-bit.
 */

#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define TIMER_C0 0xc0

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

#ifdef CONFIG_TIMER_OF

struct s5l87xx_timer {
    void __iomem *base;
    unsigned int irq;

    struct clock_event_device ce;
};

static inline void s5l87xx_timer_disable(struct s5l87xx_timer *timer)
{
    pr_debug("%s...\n", __func__);
    writel_relaxed(CMD_STOP, timer->base + TIMER_C0 + REG_CMD);
}

static inline void s5l87xx_timer_enable(struct s5l87xx_timer *timer)
{
    pr_debug("%s...\n", __func__);
    writel_relaxed(CMD_START | CMD_CLR, timer->base + TIMER_C0 + REG_CMD);
}

static inline void s5l87xx_timer_ack(struct s5l87xx_timer *timer)
{
    u32 stat;

    stat = readl_relaxed(timer->base + REG_IRQSTAT);
    writel_relaxed(stat, timer->base + REG_IRQLATCH);
}

static struct s5l87xx_timer *clksrc;

static inline struct s5l87xx_timer *s5l87xx_timer(struct clock_event_device *ce)
{
    return container_of(ce, struct s5l87xx_timer, ce);
}

static inline int s5l87xx_timer_set_next_event(unsigned long cycles, struct clock_event_device *ce) {
    pr_debug("%s(%ld)...\n", __func__, cycles);
    return 0;
}

static int s5l87xx_timer_shutdown(struct clock_event_device *ce) {
    struct s5l87xx_timer *timer = s5l87xx_timer(ce);
    s5l87xx_timer_disable(timer);
    return 0;
};

static int s5l87xx_timer_set_periodic(struct clock_event_device *ce) {
    struct s5l87xx_timer *timer = s5l87xx_timer(ce);
    pr_debug("%s\n", __func__);
    s5l87xx_timer_disable(timer);
    s5l87xx_timer_enable(timer);
    return 0;
};

static void s5l87xx_timer_dump(struct s5l87xx_timer *timer) {
    pr_debug(" offs: %08x\n", timer->base);
    pr_debug("  CON: %08x\n", readl_relaxed(timer->base + TIMER_C0 + REG_CON));
    pr_debug("  CMD: %08x\n", readl_relaxed(timer->base + TIMER_C0 + REG_CMD));
    pr_debug("DATA0: %08x\n", readl_relaxed(timer->base + TIMER_C0 + REG_DATA0));
    pr_debug("DATA1: %08x\n", readl_relaxed(timer->base + TIMER_C0 + REG_DATA1));
    pr_debug("  PRE: %08x\n", readl_relaxed(timer->base + TIMER_C0 + REG_PRE));
    pr_debug("  CND: %08x\n", readl_relaxed(timer->base + TIMER_C0 + REG_CNT));
    pr_debug("\n");
    pr_debug(" IRQSTAT: %08x\n", readl_relaxed(timer->base + REG_IRQSTAT));
    pr_debug("IRQLATCH: %08x\n", readl_relaxed(timer->base + REG_IRQLATCH));
    pr_debug("\n");
}

static irqreturn_t s5l87xx_timer_interrupt(int irq, void *dev_id) {
    struct clock_event_device *ce = dev_id;
    struct s5l87xx_timer *timer = s5l87xx_timer(ce);

    s5l87xx_timer_ack(timer);
    ce->event_handler(ce);

    return IRQ_HANDLED;
}

static int __init s5l87xx_timer_init(struct device_node *np)
{
    struct clock_event_device *ce;
    int ret = -EINVAL;

    pr_debug("%s...\n", __func__);

    clksrc = kzalloc(sizeof(struct s5l87xx_timer), GFP_KERNEL);
    if (!clksrc) {
        ret = -ENOMEM;
        goto out;
    }

    clksrc->base = of_iomap(np, 0);
    if (!clksrc->base) {
        pr_err("Failed to get base address for timer\n");
        ret = -ENXIO;
        goto out;
    };
    s5l87xx_timer_disable(clksrc);

    //timer_clk = of_clk_get_by_name(np, "timer");
    //if (IS_ERR(timer_clk)) {
    //    ret = PTR_ERR(timer_clk);
    //    pr_err("Failed to get timer clock for timer\n");
    //    goto out_timer_clk;
    //}

    //ret = clk_prepare_enable(timer_clk);
    //if (ret) {
    //    pr_err("Failed to enable timer lock\n");
    //    goto out_timer_clk;
    //}

    //clksrc->clk = timer_clk;

    clksrc->irq = irq_of_parse_and_map(np, 0);
    if (!clksrc->irq) {
        ret = -EINVAL;
        pr_err("Failed to map interrupts for timer\n");
        goto out_irq;
    }

    ce = &clksrc->ce;

    ce->name = "s5l87xx-timer";
    ce->features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
    ce->set_next_event =  s5l87xx_timer_set_next_event;
    ce->set_state_shutdown = s5l87xx_timer_shutdown;
    ce->set_state_periodic = s5l87xx_timer_set_periodic;
    ce->irq = clksrc->irq;
    ce->cpumask = cpu_possible_mask;
    ce->rating = 2137;

    ret = request_irq(ce->irq, s5l87xx_timer_interrupt, IRQF_TIMER, "s5l87xx-timer", ce);
    if (ret) {
        pr_err("Failed to initialize timer: %d\n", ret);
        goto out_irq;
    }

    clockevents_config_and_register(ce, 1000, 1, UINT_MAX);

    pr_debug("%s: success\n", __func__);
    return 0;

out_irq:
    //clk_disable_unprepare(timer_clk);
out_timer_clk:
    iounmap(clksrc->base);
out:
    clksrc = ERR_PTR(ret);
    return ret;
}

TIMER_OF_DECLARE(s5l87xx_timer, "samsung,s5l87xx-pwm", s5l87xx_timer_init);

#endif
