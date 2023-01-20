#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/mach/arch.h>

static const char * const s5l87xx_dt_compat[] = {
    "samsung,s5l8730",
	NULL,
};

DT_MACHINE_START(SUNXI_DT, "Samsung/Apple S5L8730")
	.dt_compat	= s5l87xx_dt_compat,
MACHINE_END