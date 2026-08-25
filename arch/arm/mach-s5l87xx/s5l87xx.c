// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung/Apple S5L87XX SoC family machine descriptor
 *
 * Covers S5L8702 (ARM926EJ-S, iPod nano 3g / N46) and
 *        S5L8740 (Cortex-A5, iPod nano 7g)
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/namei.h>
#include <linux/path.h>
#include <linux/printk.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#define S5L8740_WDT_PHYS	0x3c800000ul

/*
 * WTF/U-Boot leave the SEC watchdog armed. A bigger zImage loses the race.
 * Stage0 sequence only — never CLKCON+0x50 (fatal latch).
 */
static int __init s5l87xx_wdt_disarm(void)
{
	void __iomem *wdt = ioremap(S5L8740_WDT_PHYS, 8);

	if (!wdt)
		return 0;
	writel(0, wdt);
	writel(0, wdt + 4);
	writel(0, wdt);
	writel(0, wdt + 4);
	pr_info("s5l87xx: WDT disarmed con=%08x cnt=%08x\n",
		readl(wdt), readl(wdt + 4));
	iounmap(wdt);
	return 0;
}
early_initcall(s5l87xx_wdt_disarm);

/* Print before Run /init so glass shows whether rootfs actually has PID 1. */
static int __init n31_init_witness(void)
{
	struct path path;
	int err = kern_path("/init", LOOKUP_FOLLOW, &path);

	if (err) {
		pr_err("n31: /init missing (%d) — initramfs not in rootfs\n", err);
		return 0;
	}
	pr_info("n31: /init present mode=%o size=%lld\n",
		path.dentry->d_inode->i_mode,
		(long long)i_size_read(path.dentry->d_inode));
	path_put(&path);
	return 0;
}
late_initcall(n31_init_witness);

/*
 * Map the debug UART into a fixed virtual address so earlyprintk works
 * across the MMU transition.  The physical base is 0x3CC00000; UART3
 * (used by the n31 / S5L8740) adds 0xE00000 + 3×0x100000 on top.
 * The matching virtual base lives at 0xF7000000 (S3C_VA_UART).
 *
 * CONFIG_DEBUG_S3C_UART carries the uart index (0 for n3g / N46, 3 for n31).
 */
#ifdef CONFIG_DEBUG_LL

#define S5L87XX_PA_UART_BASE	0x3CC00000UL
#define S5L87XX_VA_UART_BASE	0xF7000000UL
#define S5L87XX_UART_STRIDE	0x100000UL

#if CONFIG_DEBUG_S3C_UART != 0
#define S5L87XX_DEBUG_UART_PHYS \
	(S5L87XX_PA_UART_BASE + 0xE00000UL + \
	 S5L87XX_UART_STRIDE * CONFIG_DEBUG_S3C_UART)
#define S5L87XX_DEBUG_UART_VIRT \
	(S5L87XX_VA_UART_BASE + 0xE00000UL + \
	 S5L87XX_UART_STRIDE * CONFIG_DEBUG_S3C_UART)
#else
#define S5L87XX_DEBUG_UART_PHYS	S5L87XX_PA_UART_BASE
#define S5L87XX_DEBUG_UART_VIRT	S5L87XX_VA_UART_BASE
#endif

static struct map_desc s5l87xx_io_desc[] __initdata = {
	{
		.virtual = S5L87XX_DEBUG_UART_VIRT,
		.pfn	 = __phys_to_pfn(S5L87XX_DEBUG_UART_PHYS),
		.length	 = SZ_1M,
		.type	 = MT_DEVICE,
	},
};

static void __init s5l87xx_map_io(void)
{
	iotable_init(s5l87xx_io_desc, ARRAY_SIZE(s5l87xx_io_desc));
}

#define S5L87XX_MAP_IO	s5l87xx_map_io
#else
#define S5L87XX_MAP_IO	NULL
#endif /* CONFIG_DEBUG_LL */

static const char * const s5l87xx_compat[] = {
	"samsung,s5l8702",
	"samsung,s5l8723",
	"samsung,s5l8740",
	"samsung,s5l87xx",
	NULL,
};

DT_MACHINE_START(S5L87XX_DT, "Samsung/Apple S5L87XX (Device Tree)")
	.map_io		= S5L87XX_MAP_IO,
	.dt_compat	= s5l87xx_compat,
MACHINE_END
