#include <linux/init.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sys_soc.h>
#include <asm/mach/arch.h>
#include <asm/system_info.h>

static const char * const s5l8730_dt_compat[] = {
    "samsung,s5l8730",
	NULL,
};

#define S5L87XX_CHIPID_DIEIDL 0x0c
#define S5L87XX_CHIPID_DIEIDH 0x10

static void __init s5l87xx_open_all_clkgates(void)
{
    struct device_node *np;
    void __iomem *syscon_base;

    printk("%s: HACK: enabling all s5l8730 clock gates!\n", __func__);
    np = of_find_compatible_node(NULL, NULL, "samsung,s5l87xx-syscon");
    if (!np) {
        pr_err("%s: no devcfg node found\n", __func__);
        return;
    }

    syscon_base = of_iomap(np, 0);
    of_node_put(np);
    if (!syscon_base) {
        pr_err("%s: unable to map i/o memory\n", __func__);
        return;
    }

    for (int i = 0; i < 9; i++) {
        writel(0, syscon_base + 0x48 + i * 4);
    }
}

static u64 __init s5l87xx_get_dieid(void)
{
    struct device_node *np;
    void __iomem *chipid_base;
    u32 dieidl, dieidh;

    np = of_find_compatible_node(NULL, NULL, "samsung,s5l87xx-chipid");
    if (!np) {
        pr_err("%s: no devcfg node found\n", __func__);
        return 0;
    }

    chipid_base = of_iomap(np, 0);
    of_node_put(np);
    if (!chipid_base) {
        pr_err("%s: unable to map i/o memory\n", __func__);
        return 0;
    }

    dieidl = readl(chipid_base + S5L87XX_CHIPID_DIEIDL);
    dieidh = readl(chipid_base + S5L87XX_CHIPID_DIEIDH);

    return (((u64)dieidh) << 32) | dieidl;
}

static void __init s5l8730_init_machine(void)
{
    struct device_node *root;
    struct soc_device *soc_dev;
    struct soc_device_attribute *soc_dev_attr;
    struct device *parent;
    int ret;

    printk("%s...\n", __func__);

    s5l87xx_open_all_clkgates();

    soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
    if (!soc_dev_attr) {
        return;
    }

    root = of_find_node_by_path("/");
    ret = of_property_read_string(root, "model", &soc_dev_attr->machine);
    if (ret) {
        printk("%s: model read failed\n", __func__);
        kfree(soc_dev_attr);
        return;
    }

    soc_dev_attr->family = "Samsung/Apple S5L87XX";
    soc_dev_attr->soc_id = "8730";
    soc_dev_attr->revision = "";
    soc_dev_attr->serial_number = kasprintf(GFP_KERNEL, "%llx", s5l87xx_get_dieid());

    system_rev = 0x8730;
    system_serial = soc_dev_attr->serial_number;

    soc_dev = soc_device_register(soc_dev_attr);
    if (IS_ERR(soc_dev)) {
        kfree(soc_dev_attr->serial_number);
        kfree(soc_dev_attr);
        return;
    }

    parent = soc_device_to_device(soc_dev);
    of_platform_default_populate(NULL, NULL, parent);
}

DT_MACHINE_START(SUNXI_DT, "Samsung/Apple S5L8730")
	.dt_compat	= s5l8730_dt_compat,
    .init_machine = s5l8730_init_machine,
MACHINE_END