// SPDX-License-Identifier: GPL-2.0-only
/*
 * N31 earliest bring-up hooks (before any platform drivers):
 *   1) SEC WDT @0x3C800000 disarm (stage0 sequence + clear enable bits)
 *   2) CLKCON+0x50 fatal latch clear
 *   3) Optional glass witness: backlight OFF @0x3E000000 (U-Boot leaves BL on)
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/printk.h>

#define N31_WDT_PHYS		0x3c800000ul
#define N31_BL_PHYS		0x3e000000ul
#define N31_CLKCON_PHYS		0x3c500000ul

static void n31_wdt_disarm_full(void)
{
	void __iomem *wdt, *fatal;
	u32 wcon, wcnt;

	wdt = ioremap(N31_WDT_PHYS, 8);
	if (!wdt)
		return;

	wcon = readl(wdt);
	wcnt = readl(wdt + 4);
	writel(0, wdt + 4);
	writel(0, wdt);
	writel(0, wdt + 4);
	writel(0, wdt);

	pr_alert("N31>> WDT disarm con=%08x cnt=%08x now=%08x/%08x\n",
		 wcon, wcnt, readl(wdt), readl(wdt + 4));
	iounmap(wdt);

	fatal = ioremap(N31_CLKCON_PHYS + 0x50, 4);
	if (fatal) {
		u32 v = readl(fatal);

		if (v)
			pr_alert("N31>> CLKCON+0x50 was %08x — clearing\n", v);
		writel(0, fatal);
		iounmap(fatal);
	}
}

static void n31_bl_off_witness(void)
{
	void __iomem *aux;

	aux = ioremap(N31_BL_PHYS, 0x10);
	if (!aux)
		return;
	writel(readl(aux + 0x04) & ~1u, aux + 0x04);
	writel(readl(aux + 0x08) & ~1u, aux + 0x08);
	pr_alert("N31>> BL OFF witness (expect screen black)\n");
	iounmap(aux);
}

static int __init n31_early_bringup(void)
{
	n31_wdt_disarm_full();
	n31_bl_off_witness();
	return 0;
}
early_initcall(n31_early_bringup);
