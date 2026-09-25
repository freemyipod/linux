// SPDX-License-Identifier: GPL-2.0
/*
 * S5L8702 clock controller driver
 *
 * Two register layouts share this driver:
 *
 *  - "samsung,s5l8720-clock" (S5L8720 family: N20, N31): only the crypto
 *    engine clock gates are modelled.
 *
 *  - "samsung,s5l8702-clock" (S5L8702: N46): the full clock tree as
 *    reverse engineered by Rockbox (firmware/target/arm/s5l8702/
 *    clocking-s5l8702.h):
 *
 *      OSC0 (12 MHz) --+--> PLL0/1/2 --+
 *      OSC1 (32 kHz) --+               +--> CG16_SYS ---> FClk --> CClk
 *                      +--> OSCSEL ----+                       +-> HClk --> AHB gates
 *                                      +--> CG16_RTIME -> EClk +-> PClk --> APB gates
 *                                      +--> CG16_AUDx --> I2S MCLK
 *
 *    The PLLs, FClk and the CPU/AHB/APB dividers are set up by the
 *    bootloader and are read-only here. The audio CG16 generators can
 *    be gated, reparented and divided.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <dt-bindings/clock/samsung,s5l8702-clock.h>

// S5L8702 clock controller registers
#define S5L8702_CLKCON1		0x04
#define S5L8702_PLLPMS(n)	(0x20 + (n) * 4)
#define S5L8702_PLLMODE		0x44
#define S5L8702_PWRCON_AHB	0x48
#define S5L8702_PWRCON_APB	0x4c
#define S5L8702_PLLMOD2		0x60

// CG16 generators: 16-bit halves of the CLKCONx registers
#define S5L8702_CG16_SYS	0x00
#define S5L8702_CG16_AUD0	0x0c
#define S5L8702_CG16_AUD1	0x0e
#define S5L8702_CG16_AUD2	0x10
#define S5L8702_CG16_RTIME	0x12

#define CG16_DISABLE		BIT(15)
#define CG16_SEL_SHIFT		12
#define CG16_SEL_MASK		0x3
#define CG16_UNKOSC		BIT(11)
#define CG16_DIV2_SHIFT		4
#define CG16_DIV_MASK		0xf
#define CG16_DIV_MAX		(CG16_DIV_MASK + 1)
#define CG16_PARENT_UNKOSC	4

// CLKCON1: divisor = EN ? 2 * (field + 1) : 1
#define CLKCON1_CDIV_SHIFT	24
#define CLKCON1_CDIV_EN		BIT(30)
#define CLKCON1_HDIV_SHIFT	16
#define CLKCON1_HDIV_EN		BIT(22)
#define CLKCON1_PDIV_SHIFT	8
#define CLKCON1_PDIV_EN		BIT(14)
#define CLKCON1_DIV_MASK	0x1f

#define PLLPMS_PDIV_SHIFT	24
#define PLLPMS_PDIV_MASK	0x3f
#define PLLPMS_MDIV_SHIFT	8
#define PLLPMS_MDIV_MASK	0xff
#define PLLPMS_SDIV_MASK	0x7

#define PLLMODE_EN(n)		BIT(n)
#define PLLMODE_PMSMOD(n)	BIT(4 + (n))	// PLL0/1: 0 = multiply, 1 = divide
#define PLLMODE_PLL2DMOSC	BIT(6)
#define PLLMODE_OSCSEL_SHIFT	8
#define PLLMODE_PLLOUT(n)	BIT(16 + (n))	// 0 = slow mode (OSC1)
#define PLLMOD2_DMOSC(n)	BIT(4 + (n))

// Selected by CG16_UNKOSC, TBC (24 MHz per Rockbox's measurements)
#define S5L8702_UNKOSC_HZ	24000000

struct s5l8702_clk_data {
	void __iomem *regs;
	struct clk_hw_onecell_data *hw_data;
	spinlock_t lock;
};

struct s5l87xx_clk_soc {
	unsigned int num_clks;
	int (*register_clks)(struct device *dev, struct s5l8702_clk_data *data);
};

// -----------------------------------------------------------------------
// S5L8720 family: crypto clock gates only

struct s5l8720_clk_gate {
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

static const struct s5l8720_clk_gate s5l8720_gates[] = {
	[CLK_SHA1]	= GATE("sha1",	NULL, 0x48, 0),
	[CLK_AES]	= GATE("aes",	NULL, 0x48, 7),

	[CLK_PRNG]	= GATE("prng",	NULL, 0x4c, 0),
};

static int s5l8720_register_clks(struct device *dev, struct s5l8702_clk_data *data)
{
	struct clk_hw **hws = data->hw_data->hws;
	int i;

	for (i = 0; i < ARRAY_SIZE(s5l8720_gates); i++) {
		const struct s5l8720_clk_gate *clk_gate = &s5l8720_gates[i];

		hws[i] = devm_clk_hw_register_gate(dev, clk_gate->name, clk_gate->parent_name, 0,
			data->regs + clk_gate->reg, clk_gate->bit, CLK_GATE_SET_TO_DISABLE, &data->lock);

		if (IS_ERR(hws[i])) {
			return PTR_ERR(hws[i]);
		}
	}

	return 0;
}

static const struct s5l87xx_clk_soc s5l8720_clk_soc = {
	.num_clks = ARRAY_SIZE(s5l8720_gates),
	.register_clks = s5l8720_register_clks,
};

// -----------------------------------------------------------------------
// S5L8702 PLLs (read-only)

struct s5l8702_pll {
	struct clk_hw hw;
	void __iomem *regs;
	int id;
};

#define to_s5l8702_pll(_hw) container_of(_hw, struct s5l8702_pll, hw)

// PLL parents: 0 = OSC0 (divide mode), 1 = OSC1 (multiply or slow mode)
static bool s5l8702_pll_divide_mode(struct s5l8702_pll *pll, u32 mode)
{
	// PLL2 only supports divide mode
	return pll->id == 2 || (mode & PLLMODE_PMSMOD(pll->id));
}

static bool s5l8702_pll_alt_osc(struct s5l8702_pll *pll, u32 mode)
{
	if (pll->id == 2)
		return mode & PLLMODE_PLL2DMOSC;

	return readl(pll->regs + S5L8702_PLLMOD2) & PLLMOD2_DMOSC(pll->id);
}

static u8 s5l8702_pll_get_parent(struct clk_hw *hw)
{
	struct s5l8702_pll *pll = to_s5l8702_pll(hw);
	u32 mode = readl(pll->regs + S5L8702_PLLMODE);

	if (!(mode & PLLMODE_PLLOUT(pll->id)))
		return 1;

	return s5l8702_pll_divide_mode(pll, mode) ? 0 : 1;
}

static unsigned long s5l8702_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct s5l8702_pll *pll = to_s5l8702_pll(hw);
	u32 mode = readl(pll->regs + S5L8702_PLLMODE);
	u32 pms, p, m, s;

	// Slow mode: the PLL output is bypassed to OSC1
	if (!(mode & PLLMODE_PLLOUT(pll->id)))
		return parent_rate;

	if (!(mode & PLLMODE_EN(pll->id)))
		return 0;

	pms = readl(pll->regs + S5L8702_PLLPMS(pll->id));
	p = (pms >> PLLPMS_PDIV_SHIFT) & PLLPMS_PDIV_MASK;
	m = (pms >> PLLPMS_MDIV_SHIFT) & PLLPMS_MDIV_MASK;
	s = pms & PLLPMS_SDIV_MASK;

	if (!p)
		return 0;

	// Per Rockbox's measurements, M values 0 and 1 behave as 256 and 257
	if (m < 2)
		m += 256;

	if (s5l8702_pll_divide_mode(pll, mode)) {
		// The alternate oscillator frequencies are unknown on this board
		if (s5l8702_pll_alt_osc(pll, mode))
			return 0;

		return div_u64((u64)parent_rate * m, p) >> s;
	}

	return ((u64)parent_rate * m * p) >> s;
}

static const struct clk_ops s5l8702_pll_ops = {
	.recalc_rate = s5l8702_pll_recalc_rate,
	.get_parent = s5l8702_pll_get_parent,
};

// -----------------------------------------------------------------------
// S5L8702 CG16 clock generators

// Each CG16 is one half of a 32-bit CLKCONx register. Only access them
// with 32-bit read-modify-writes, like Rockbox's cg16_config(): a 16-bit
// store may clobber the other half, and next to AUD2 sits RTIME (EClk),
// which clocks the timers and the SDRAM refresh.
struct s5l8702_cg16 {
	struct clk_hw hw;
	void __iomem *reg;	// containing 32-bit register
	u8 shift;		// 0 = low half, 16 = high half
	spinlock_t *lock;
};

#define to_s5l8702_cg16(_hw) container_of(_hw, struct s5l8702_cg16, hw)

static u16 s5l8702_cg16_read(struct s5l8702_cg16 *cg)
{
	return readl(cg->reg) >> cg->shift;
}

static u8 s5l8702_cg16_get_parent(struct clk_hw *hw)
{
	u16 val = s5l8702_cg16_read(to_s5l8702_cg16(hw));

	if (val & CG16_UNKOSC)
		return CG16_PARENT_UNKOSC;

	return (val >> CG16_SEL_SHIFT) & CG16_SEL_MASK;
}

static unsigned long s5l8702_cg16_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	u16 val = s5l8702_cg16_read(to_s5l8702_cg16(hw));
	unsigned int div1 = (val & CG16_DIV_MASK) + 1;
	unsigned int div2 = ((val >> CG16_DIV2_SHIFT) & CG16_DIV_MASK) + 1;

	return parent_rate / (div1 * div2);
}

static int s5l8702_cg16_is_enabled(struct clk_hw *hw)
{
	return !(s5l8702_cg16_read(to_s5l8702_cg16(hw)) & CG16_DISABLE);
}

static void s5l8702_cg16_update(struct s5l8702_cg16 *cg, u16 mask, u16 set)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(cg->lock, flags);
	val = readl(cg->reg);
	val &= ~((u32)mask << cg->shift);
	val |= (u32)set << cg->shift;
	writel(val, cg->reg);
	spin_unlock_irqrestore(cg->lock, flags);
}

static int s5l8702_cg16_enable(struct clk_hw *hw)
{
	s5l8702_cg16_update(to_s5l8702_cg16(hw), CG16_DISABLE, 0);
	return 0;
}

static void s5l8702_cg16_disable(struct clk_hw *hw)
{
	s5l8702_cg16_update(to_s5l8702_cg16(hw), CG16_DISABLE, CG16_DISABLE);
}

static int s5l8702_cg16_set_parent(struct clk_hw *hw, u8 index)
{
	struct s5l8702_cg16 *cg = to_s5l8702_cg16(hw);

	if (index == CG16_PARENT_UNKOSC)
		s5l8702_cg16_update(cg, CG16_UNKOSC, CG16_UNKOSC);
	else
		s5l8702_cg16_update(cg, CG16_UNKOSC | (CG16_SEL_MASK << CG16_SEL_SHIFT),
				    index << CG16_SEL_SHIFT);

	return 0;
}

// Find DIV1/DIV2 (1..16 each) giving the highest rate not above @rate
static unsigned long s5l8702_cg16_best_divs(unsigned long rate, unsigned long parent_rate,
					    unsigned int *best_div1, unsigned int *best_div2)
{
	unsigned long best = 0;
	unsigned int div1, div2;

	*best_div1 = CG16_DIV_MAX;
	*best_div2 = CG16_DIV_MAX;

	for (div1 = 1; div1 <= CG16_DIV_MAX; div1++) {
		for (div2 = div1; div2 <= CG16_DIV_MAX; div2++) {
			unsigned long r = parent_rate / (div1 * div2);

			if (r <= rate && r > best) {
				best = r;
				*best_div1 = div1;
				*best_div2 = div2;
			}
		}
	}

	return best ? best : parent_rate / (CG16_DIV_MAX * CG16_DIV_MAX);
}

static int s5l8702_cg16_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	unsigned int div1, div2;

	req->rate = s5l8702_cg16_best_divs(req->rate, req->best_parent_rate, &div1, &div2);
	return 0;
}

static int s5l8702_cg16_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	unsigned int div1, div2;

	s5l8702_cg16_best_divs(rate, parent_rate, &div1, &div2);
	s5l8702_cg16_update(to_s5l8702_cg16(hw),
			    (CG16_DIV_MASK << CG16_DIV2_SHIFT) | CG16_DIV_MASK,
			    ((div2 - 1) << CG16_DIV2_SHIFT) | (div1 - 1));
	return 0;
}

// No .is_enabled: CG16_SYS reads back with DISABLE set while FClk is
// clearly running, so the bit doesn't mean "off" on these generators.
static const struct clk_ops s5l8702_cg16_ro_ops = {
	.recalc_rate = s5l8702_cg16_recalc_rate,
	.get_parent = s5l8702_cg16_get_parent,
};

static const struct clk_ops s5l8702_cg16_ops = {
	.enable = s5l8702_cg16_enable,
	.disable = s5l8702_cg16_disable,
	.is_enabled = s5l8702_cg16_is_enabled,
	.recalc_rate = s5l8702_cg16_recalc_rate,
	.determine_rate = s5l8702_cg16_determine_rate,
	.set_rate = s5l8702_cg16_set_rate,
	.get_parent = s5l8702_cg16_get_parent,
	.set_parent = s5l8702_cg16_set_parent,
};

// -----------------------------------------------------------------------
// S5L8702 CPU/AHB/APB dividers (CLKCON1, read-only)

struct s5l8702_sysdiv {
	struct clk_hw hw;
	void __iomem *reg;
	u32 en_bit;
	u8 shift;
};

#define to_s5l8702_sysdiv(_hw) container_of(_hw, struct s5l8702_sysdiv, hw)

static unsigned long s5l8702_sysdiv_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct s5l8702_sysdiv *sd = to_s5l8702_sysdiv(hw);
	u32 val = readl(sd->reg);

	if (!(val & sd->en_bit))
		return parent_rate;

	return parent_rate / ((((val >> sd->shift) & CLKCON1_DIV_MASK) + 1) * 2);
}

static const struct clk_ops s5l8702_sysdiv_ops = {
	.recalc_rate = s5l8702_sysdiv_recalc_rate,
};

// -----------------------------------------------------------------------
// S5L8702 clock tree

struct s5l8702_gate {
	unsigned int id;
	const char *name;
	bool apb;
	u8 bit;
	unsigned long flags;
};

// Gate numbering follows Rockbox's CLOCKGATE_* (firmware/export/s5l8702.h).
// Unclaimed gates are left as the bootloader configured them. Gates without
// a clock consumer that Linux depends on are critical.
static const struct s5l8702_gate s5l8702_gates[] = {
	{ CLK_SHA1,	"sha1",		false,	0 },
	{ CLK_LCD,	"lcd",		false,	1 },
	{ CLK_USBOTG,	"usbotg",	false,	2 },
	{ CLK_SMX,	"smx",		false,	3,	CLK_IS_CRITICAL },	// IRAM0/1
	{ CLK_SM1,	"sm1",		false,	4 },			// IRAM1
	{ CLK_ATA,	"ata",		false,	5 },
	{ CLK_NAND,	"nand",		false,	8 },
	{ CLK_SDCI,	"sdci",		false,	9 },
	{ CLK_AES,	"aes",		false,	10 },
	{ CLK_NANDECC,	"nandecc",	false,	12 },
	{ CLK_DMAC0,	"dmac0",	false,	25 },
	{ CLK_DMAC1,	"dmac1",	false,	26 },
	{ CLK_ROM,	"rom",		false,	30 },

	{ CLK_RTC,	"rtc",		true,	0 },
	{ CLK_CWHEEL,	"cwheel",	true,	1 },
	{ CLK_SPI0,	"spi0",		true,	2 },
	{ CLK_USBPHY,	"usbphy",	true,	3 },
	{ CLK_I2C0,	"i2c0",		true,	4 },
	// timer-s5l8702 is a TIMER_OF_DECLARE driver and runs before this one
	{ CLK_TIMER,	"timer",	true,	5,	CLK_IS_CRITICAL },
	{ CLK_I2C1,	"i2c1",		true,	6 },
	{ CLK_I2S0,	"i2s0",		true,	7 },
	{ CLK_UART,	"uart",		true,	9 },
	{ CLK_I2S1,	"i2s1",		true,	10 },
	{ CLK_SPI1,	"spi1",		true,	11 },
	// Other drivers still write their own pin mux into PCON directly
	{ CLK_GPIO,	"gpio",		true,	12,	CLK_IS_CRITICAL },
	{ CLK_CHIPID,	"chipid",	true,	14 },
	// Rockbox's clocking diagram swaps these two; this matches its code
	{ CLK_I2S2,	"i2s2",		true,	15 },
	{ CLK_SPI2,	"spi2",		true,	16 },
};

struct s5l8702_cg16_desc {
	unsigned int id;
	const char *name;
	u32 offset;
	bool writable;
};

// FClk and EClk drive the CPU, buses, timers and SDRAM refresh: read-only
static const struct s5l8702_cg16_desc s5l8702_cg16s[] = {
	{ CLK_FCLK,	"fclk",	S5L8702_CG16_SYS,	false },
	{ CLK_ECLK,	"eclk",	S5L8702_CG16_RTIME,	false },
	{ CLK_AUD0,	"aud0",	S5L8702_CG16_AUD0,	true },
	{ CLK_AUD1,	"aud1",	S5L8702_CG16_AUD1,	true },
	{ CLK_AUD2,	"aud2",	S5L8702_CG16_AUD2,	true },
};

static struct clk_hw *s5l8702_register_cg16(struct device *dev, struct s5l8702_clk_data *data,
					    const char *name, u32 offset,
					    const struct clk_parent_data *parents,
					    const struct clk_ops *ops)
{
	struct clk_init_data init = {};
	struct s5l8702_cg16 *cg;
	int ret;

	cg = devm_kzalloc(dev, sizeof(*cg), GFP_KERNEL);
	if (!cg)
		return ERR_PTR(-ENOMEM);

	cg->reg = data->regs + (offset & ~3);
	cg->shift = (offset & 2) * 8;
	cg->lock = &data->lock;

	init.name = name;
	init.ops = ops;
	init.parent_data = parents;
	init.num_parents = 5;
	// Like the gates, leave whatever the bootloader set up until a consumer
	// claims the clock
	init.flags = CLK_SET_RATE_NO_REPARENT | CLK_IGNORE_UNUSED;
	cg->hw.init = &init;

	ret = devm_clk_hw_register(dev, &cg->hw);
	return ret ? ERR_PTR(ret) : &cg->hw;
}

static struct clk_hw *s5l8702_register_sysdiv(struct device *dev, struct s5l8702_clk_data *data,
					      const char *name, struct clk_hw *fclk,
					      u8 shift, u32 en_bit)
{
	struct clk_init_data init = {};
	struct s5l8702_sysdiv *sd;
	int ret;

	sd = devm_kzalloc(dev, sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return ERR_PTR(-ENOMEM);

	sd->reg = data->regs + S5L8702_CLKCON1;
	sd->shift = shift;
	sd->en_bit = en_bit;

	init.name = name;
	init.ops = &s5l8702_sysdiv_ops;
	init.parent_hws = (const struct clk_hw **)&fclk;
	init.num_parents = 1;
	sd->hw.init = &init;

	ret = devm_clk_hw_register(dev, &sd->hw);
	return ret ? ERR_PTR(ret) : &sd->hw;
}

static int s5l8702_register_clks(struct device *dev, struct s5l8702_clk_data *data)
{
	static const struct clk_parent_data osc_parents[] = {
		{ .fw_name = "osc0" },
		{ .fw_name = "osc1" },
	};
	static const char * const pll_names[] = { "pll0", "pll1", "pll2" };
	struct clk_hw **hws = data->hw_data->hws;
	struct clk_parent_data cg16_parents[5] = {};
	struct clk_hw *oscsel, *unkosc, *hclk, *pclk;
	int i;

	for (i = 0; i < data->hw_data->num; i++)
		hws[i] = ERR_PTR(-ENOENT);

	unkosc = devm_clk_hw_register_fixed_rate(dev, "unkosc", NULL, 0, S5L8702_UNKOSC_HZ);
	if (IS_ERR(unkosc))
		return PTR_ERR(unkosc);

	oscsel = devm_clk_hw_register_mux_parent_data_table(dev, "oscsel", osc_parents,
			ARRAY_SIZE(osc_parents), 0, data->regs + S5L8702_PLLMODE,
			PLLMODE_OSCSEL_SHIFT, 1, CLK_MUX_READ_ONLY, NULL, &data->lock);
	if (IS_ERR(oscsel))
		return PTR_ERR(oscsel);

	for (i = 0; i < ARRAY_SIZE(pll_names); i++) {
		struct clk_init_data init = {};
		struct s5l8702_pll *pll;
		int ret;

		pll = devm_kzalloc(dev, sizeof(*pll), GFP_KERNEL);
		if (!pll)
			return -ENOMEM;

		pll->regs = data->regs;
		pll->id = i;

		init.name = pll_names[i];
		init.ops = &s5l8702_pll_ops;
		init.parent_data = osc_parents;
		init.num_parents = ARRAY_SIZE(osc_parents);
		pll->hw.init = &init;

		ret = devm_clk_hw_register(dev, &pll->hw);
		if (ret)
			return ret;

		hws[CLK_PLL0 + i] = &pll->hw;
	}

	// CG16 source select: 0 = OSCSEL, 1..3 = PLL0..2, UNKOSC bit overrides
	cg16_parents[0].hw = oscsel;
	cg16_parents[1].hw = hws[CLK_PLL0];
	cg16_parents[2].hw = hws[CLK_PLL1];
	cg16_parents[3].hw = hws[CLK_PLL2];
	cg16_parents[CG16_PARENT_UNKOSC].hw = unkosc;

	for (i = 0; i < ARRAY_SIZE(s5l8702_cg16s); i++) {
		const struct s5l8702_cg16_desc *d = &s5l8702_cg16s[i];

		hws[d->id] = s5l8702_register_cg16(dev, data, d->name, d->offset, cg16_parents,
						   d->writable ? &s5l8702_cg16_ops
							       : &s5l8702_cg16_ro_ops);
		if (IS_ERR(hws[d->id]))
			return PTR_ERR(hws[d->id]);
	}

	hws[CLK_CCLK] = s5l8702_register_sysdiv(dev, data, "cclk", hws[CLK_FCLK],
						CLKCON1_CDIV_SHIFT, CLKCON1_CDIV_EN);
	if (IS_ERR(hws[CLK_CCLK]))
		return PTR_ERR(hws[CLK_CCLK]);

	hclk = s5l8702_register_sysdiv(dev, data, "hclk", hws[CLK_FCLK],
				       CLKCON1_HDIV_SHIFT, CLKCON1_HDIV_EN);
	if (IS_ERR(hclk))
		return PTR_ERR(hclk);
	hws[CLK_HCLK] = hclk;

	pclk = s5l8702_register_sysdiv(dev, data, "pclk", hws[CLK_FCLK],
				       CLKCON1_PDIV_SHIFT, CLKCON1_PDIV_EN);
	if (IS_ERR(pclk))
		return PTR_ERR(pclk);
	hws[CLK_PCLK] = pclk;

	for (i = 0; i < ARRAY_SIZE(s5l8702_gates); i++) {
		const struct s5l8702_gate *g = &s5l8702_gates[i];
		u32 reg = g->apb ? S5L8702_PWRCON_APB : S5L8702_PWRCON_AHB;

		hws[g->id] = devm_clk_hw_register_gate_parent_hw(dev, g->name,
				g->apb ? pclk : hclk, g->flags | CLK_IGNORE_UNUSED,
				data->regs + reg, g->bit, CLK_GATE_SET_TO_DISABLE, &data->lock);
		if (IS_ERR(hws[g->id]))
			return PTR_ERR(hws[g->id]);
	}

	return 0;
}

static const struct s5l87xx_clk_soc s5l8702_clk_soc = {
	.num_clks = CLK_SPI2 + 1,
	.register_clks = s5l8702_register_clks,
};

// -----------------------------------------------------------------------

static int s5l8702_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct s5l87xx_clk_soc *soc;
	struct s5l8702_clk_data *clk_data;
	struct clk_hw_onecell_data *hw_data;
	int ret;

	soc = of_device_get_match_data(dev);
	if (!soc) {
		return -ENODEV;
	}

	clk_data = devm_kzalloc(dev, sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data) {
		return -ENOMEM;
	}

	clk_data->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(clk_data->regs)) {
		return PTR_ERR(clk_data->regs);
	}

	hw_data = devm_kzalloc(dev, struct_size(hw_data, hws, soc->num_clks), GFP_KERNEL);
	if (!hw_data) {
		return -ENOMEM;
	}

	hw_data->num = soc->num_clks;
	clk_data->hw_data = hw_data;

	spin_lock_init(&clk_data->lock);

	ret = soc->register_clks(dev, clk_data);
	if (ret) {
		return dev_err_probe(dev, ret, "failed to register clocks\n");
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, hw_data);
	if (ret) {
		return ret;
	}

	dev_info(dev, "Registered %u clock(s)\n", soc->num_clks);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id s5l8702_clk_of_match[] = {
	{ .compatible = "samsung,s5l8702-clock", .data = &s5l8702_clk_soc },
	{ .compatible = "samsung,s5l8720-clock", .data = &s5l8720_clk_soc },
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
MODULE_DESCRIPTION("S5L8702 clock controller");
MODULE_LICENSE("GPL v2");
