// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 / S5L8740 Clockgates
 *
 * Bring-up policy (Phases 0–4): ungate-all documented PWRCON banks so
 * peripherals stay alive without a Linux consumer. CCF also marks the
 * published gates CLK_IS_CRITICAL | CLK_IGNORE_UNUSED so
 * clk_disable_unused cannot write those bits later.
 *
 * Selective IIS/UART/CG16 CCF consumers are DEFERRED to Phase 5
 * (p5-ccf-pm). Rationale: HCI/ALSA/FM glass proof still needs a stable
 * clock baseline; early selective gating caused false leads (timer poke,
 * SYS remux). Keep absolute CLKCON+0x30 play (0x32190-class) /
 * idle (0x1c20) in the IIS drivers until audio/BT prove out.
 *
 * Never remux SYS PLL (+0x00/+0x04) — that kills live DRAM.
 * Never write CLKCON+0x50 — that is the fatal/WDT latch (0xA5).
 * Never poke the TIMER MMIO block @0x3C700000 from here.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <dt-bindings/clock/samsung,s5l8702-clock.h>

#define CLKCON_PWRCON0		0x48
#define CLKCON_PWRCON1		0x4c
#define CLKCON_PWRCON2		0x58
#define CLKCON_PWRCON4		0x6c

struct s5l8702_clk_data {
	void __iomem *regs;
	struct clk_hw_onecell_data *hw_data;
	spinlock_t lock;
};

struct s5l8702_clk_gate {
	const char *name;
	const char *parent_name;
	u32 reg;
	u8 bit;
};

#define GATE(_name, _parent, _reg, _bit) \
	{ \
		.name = (_name), \
		.parent_name = (_parent), \
		.reg = (_reg), \
		.bit = (_bit), \
	}

static const struct s5l8702_clk_gate s5l8702_gates[] = {
	[CLK_SHA1]	= GATE("sha1",	NULL, 0x48, 0),
	[CLK_AES]	= GATE("aes",	NULL, 0x48, 7),

	[CLK_PRNG]	= GATE("prng",	NULL, 0x4c, 0),
};

/*
 * SET_TO_DISABLE: bit clear = clock running. Write 0 to the known PWRCON
 * banks so every AHB/APB gate is on even without a driver. CG16 enable
 * bits (RetailOS 41CBD8) are the high halves of the divider regs — clear
 * those bits only; leave the divider fields WTF/U-Boot programmed.
 */
static void s5l8740_ungate_all(struct device *dev, void __iomem *regs)
{
	dev_info(dev,
		 "PWRCON before: +48=%08x +4c=%08x +58=%08x +6c=%08x SYS+00=%08x\n",
		 readl(regs + CLKCON_PWRCON0), readl(regs + CLKCON_PWRCON1),
		 readl(regs + CLKCON_PWRCON2), readl(regs + CLKCON_PWRCON4),
		 readl(regs + 0x00));

	writel(0, regs + CLKCON_PWRCON0);
	writel(0, regs + CLKCON_PWRCON1);
	writel(0, regs + CLKCON_PWRCON2);
	writel(0, regs + CLKCON_PWRCON4);

	writel(readl(regs + 0x08) & ~0x80008000u, regs + 0x08);
	writel(readl(regs + 0x0c) & ~0x80008000u, regs + 0x0c);
	writel(readl(regs + 0x10) & ~0x8000u,     regs + 0x10);
	writel(readl(regs + 0x14) & ~0x80008000u, regs + 0x14);

	dev_info(dev,
		 "PWRCON after ungate-all: +48=%08x +4c=%08x +58=%08x +6c=%08x\n",
		 readl(regs + CLKCON_PWRCON0), readl(regs + CLKCON_PWRCON1),
		 readl(regs + CLKCON_PWRCON2), readl(regs + CLKCON_PWRCON4));
}

static int s5l8702_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l8702_clk_data *clk_data;
	struct clk_hw_onecell_data *hw_data;
	int i, ret;
	size_t num_clks;
	const unsigned long gate_flags =
		CLK_IGNORE_UNUSED | CLK_IS_CRITICAL;

	num_clks = ARRAY_SIZE(s5l8702_gates);

	clk_data = devm_kzalloc(dev, sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(clk_data->regs))
		return PTR_ERR(clk_data->regs);

	s5l8740_ungate_all(dev, clk_data->regs);

	hw_data = devm_kzalloc(dev, struct_size(hw_data, hws, num_clks), GFP_KERNEL);
	if (!hw_data)
		return -ENOMEM;

	hw_data->num = num_clks;

	spin_lock_init(&clk_data->lock);

	for (i = 0; i < num_clks; i++) {
		const struct s5l8702_clk_gate *clk_gate = &s5l8702_gates[i];

		hw_data->hws[i] = devm_clk_hw_register_gate(dev, clk_gate->name,
			clk_gate->parent_name, gate_flags,
			clk_data->regs + clk_gate->reg, clk_gate->bit,
			CLK_GATE_SET_TO_DISABLE, &clk_data->lock);

		if (IS_ERR(hw_data->hws[i]))
			return PTR_ERR(hw_data->hws[i]);
	}

	clk_data->hw_data = hw_data;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, hw_data);
	if (ret)
		return ret;

	dev_info(dev, "Registered %zu clockgate(s), unused left enabled\n",
		 num_clks);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id s5l8702_clk_of_match[] = {
	{ .compatible = "samsung,s5l8702-clock" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8702_clk_of_match);
#endif

static struct platform_driver s5l8702_clk_driver = {
	.probe = s5l8702_clk_probe,
	.driver = {
		.name = "s5l8702-clk",
		.of_match_table = of_match_ptr(s5l8702_clk_of_match),
	},
};
module_platform_driver(s5l8702_clk_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("S5L8702 Clockgates");
MODULE_LICENSE("GPL v2");
