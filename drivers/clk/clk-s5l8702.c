// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 Clockgates driver
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <dt-bindings/clock/samsung,s5l8702-clock.h>

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
	[CLK_SHA1] = GATE("sha1", NULL, 0x48, 0),
	[CLK_PRNG] = GATE("prng", NULL, 0x4c, 0),
};

static int s5l8702_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l8702_clk_data *clk_data;
	struct clk_hw_onecell_data *hw_data;
	int i, ret;
	size_t num_clks;

	num_clks = ARRAY_SIZE(s5l8702_gates);

	clk_data = devm_kzalloc(dev, sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data) {
		return -ENOMEM;
	}

	clk_data->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(clk_data->regs)) {
		return PTR_ERR(clk_data->regs);
	}

	hw_data = devm_kzalloc(dev, struct_size(hw_data, hws, num_clks), GFP_KERNEL);
	if (!hw_data) {
		return -ENOMEM;
	}

	hw_data->num = num_clks;

	spin_lock_init(&clk_data->lock);

	for (i = 0; i < num_clks; i++) {
		const struct s5l8702_clk_gate *clk_gate = &s5l8702_gates[i];

		hw_data->hws[i] = devm_clk_hw_register_gate(dev, clk_gate->name, clk_gate->parent_name, 0,
			clk_data->regs + clk_gate->reg, clk_gate->bit, CLK_GATE_SET_TO_DISABLE, &clk_data->lock);

		if (IS_ERR(hw_data->hws[i])) {
			return PTR_ERR(hw_data->hws[i]);
		}
	}

	clk_data->hw_data = hw_data;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, hw_data);
	if (ret) {
		return ret;
	}

	dev_info(dev, "Registered %d clockgate(s)", num_clks);

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
