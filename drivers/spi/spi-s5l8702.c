// SPDX-License-Identifier: GPL-2.0
/*
 * SPI controller for Samsung/Apple S5L8702 / S5L8740
 *
 * RetailOS PIO (sub_4043D0) is shared by SPI0/CS42 and SPI2/Nimbus:
 *   CS = SPIPIN bit1 (4045D4(n): assert=clear, idle=set)
 *   flush STATUS & 0x7C0 / 0xF800 == 0
 *   per-byte: wait !0x7C0, TXDATA, wait 0xF800, RXDATA
 *   SETUP 0x402C|0x10, CLKDIV 4
 *
 * Do not remux SPI0 pads 0–3 (SEC leftover 4/2/2/2). OSOS never GPIOCMDs
 * those pads. Do not write 0x3CF00200 — CS42 CS is 4045D4(0), not GPIOCMD.
 *
 * "DMA" in 4043D0 is the SPI-controller block path (SETUP bit5 after
 * 11B70), not PL080. No SPI0/SPI2 PL080 peri IDs in OSOS. sub_3914
 * only does STATUS |= 0x40003F. Nimbus/CS42 stay on this PIO loop;
 * do not invent peri IDs.
 */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define SPICTRL			0x00
#define SPISETUP		0x04
#define SPISTATUS		0x08
#define SPIPIN			0x0c
#define SPITXDATA		0x10
#define SPIRXDATA		0x20
#define SPICLKDIV		0x30
#define SPIRXLIMIT		0x34
#define SPIUNK4C		0x4c	/* 4043D0: write 1 after TXDATA */

#define SPISTATUS_TXFULL	0x100
#define SPISTATUS_TXLVL_MASK	0x1f0
#define SPISTATUS_RXLVL_MASK	0x3e00
#define SPISTATUS_TXBUSY_ROS	0x7c0
#define SPISTATUS_RXRDY_ROS	0xf800

/*
 * Two different status encodings have been seen on this engine.
 *
 *   ROS      TX busy 0x7C0 -> 0, RX ready 0xF800 nonzero
 *   CLASSIC  TX level 0x1F0 -> 0, RX level 0x3E00 nonzero
 *
 * This driver used to wait on the ROS masks and then, on timeout, accept
 * the CLASSIC masks as a fallback -- and vice versa for RX. That is a
 * correctness hazard, not just a slow path: the two families' masks
 * overlap (0x7C0 vs 0x1F0 share bits 6-8, 0xF800 vs 0x3E00 share bits
 * 11-13), so the wrong family's test can read as satisfied while the
 * transfer is still in flight. RXDATA is then sampled early and returns
 * the previous FIFO contents. That does not look like noise: it repeats,
 * which is exactly the shape of the 4f814f81 pattern seen on SPI2.
 *
 * So the encoding is now a per-instance property. In AUTO the first
 * transfer polls both families in one loop and latches whichever
 * genuinely satisfies first -- a measurement, made once, and logged.
 * After that only the latched family is consulted and a timeout is a
 * real timeout rather than a cue to try the other interpretation.
 */
enum s5l8702_spi_fam {
	SPI_FAM_AUTO = 0,
	SPI_FAM_ROS,
	SPI_FAM_CLASSIC,
};

static int spi_family;
module_param(spi_family, int, 0644);
MODULE_PARM_DESC(spi_family,
		 "status encoding: 0=auto-latch per instance, 1=ROS (0x7C0/0xF800), 2=Classic (0x1F0/0x3E00)");

#define SPISETUP_RXMODE		BIT(0)
#define SPISETUP_RETAILOS	0x403c	/* 0x402C | 0x10 — SPI2 / 11B70(2,0x1A,…) */
#define SPISETUP_SPI0_11B70	0x403e	/* 11B70(0,0x1A,0x2EE0,8) → 0x402E|0x10 */

#define SPICTRL_RESET_FIFO	0xc
#define SPICTRL_ENABLE		0x1

#define SPISTATUS_KICK		0x400000	/* set after SETUP clear RXMODE */

#define SPIPIN_CS_BIT		BIT(1)

#define S5L8702_PCON0_PHYS	0x3cf00000UL
#define S5L8702_GPIOCMD_PHYS	0x3cf001e0UL
#define S5L8702_PWRCON1_PHYS	0x3c50004cUL
#define S5L8702_PWRCON4_PHYS	0x3c50006cUL
#define PWRCON1_SPI0_BIT	BIT(2)
#define PWRCON1_SPI2_BIT	BIT(16)
#define PWRCON4_SPI0_2_BIT	BIT(13)	/* CLK_SPI0_2 / bring-up table */
#define SPI_CLKDIV_DEFAULT	4
#define SPI0_BASE_PHYS		0x3c300000UL
#define SPI2_BASE_PHYS		0x3d200000UL

/*
 * Spin budget per status wait.
 *
 * This was 500000, which is not a timeout so much as a way to make a stuck
 * bus look like a dead device: every wait burns the full budget with
 * preemption held off, and with dozens of register accesses in one codec
 * bring-up the machine stops scheduling userspace entirely. The symptom is
 * distinctive and misleading -- ping still answers because that is
 * interrupt context, while ssh times out and the device reads as locked.
 * More than one "lockup" chased tonight looked exactly like that.
 *
 * A transfer that has not progressed in a few milliseconds is not going to.
 * Fail fast, report it, and let the caller decide.
 *
 * Do NOT cond_resched() in these loops. transfer_one can run in atomic
 * context, and sleeping there is "BUG: scheduling while atomic" -- a panic,
 * which with panic=-1 on the cmdline reboots before the message can be
 * read. That was added here as a fix for CPU starvation and was itself the
 * crash. Lowering the budget is the safe half of that idea; yielding is
 * not, and the correct place to solve starvation is a smaller budget, not
 * a sleep in a path that may not sleep.
 */
#define SPI_WAIT_GUARD		20000

/* Verbose 11B70/CS setup spam off by default. */
static bool verbose;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose, "Verbose S5L SPI controller logs (default N)");

#define spi_vinfo(dev, fmt, ...)					\
	do {							\
		if (verbose)					\
			dev_info((dev), fmt, ##__VA_ARGS__);	\
		else						\
			dev_dbg((dev), fmt, ##__VA_ARGS__);	\
	} while (0)


struct s5l8702_spi {
	void __iomem *base;
	struct device *dev;
	void __iomem *gpiocmd;
	void __iomem *gpio_base;
	void __iomem *pwrcon1;
	struct clk_bulk_data *clks;
	int num_clks;
	bool spi0;
	bool spi2_nimbus;
	bool prepared;
	int last_err;
	u32 last_status;
	enum s5l8702_spi_fam fam;
	bool cs_held;		/* CS left asserted by a cs_change transfer */
	unsigned int fam_latch_status;
	unsigned int tx_timeouts;
	unsigned int rx_timeouts;
};

static void s5l8702_gpiocmd_func(struct s5l8702_spi *sspi, unsigned int gpio, u8 func)
{
	u32 bank = gpio >> 3;
	u32 pin = gpio & 7;
	void __iomem *b;
	u32 dir;

	if (!sspi->gpiocmd)
		return;
	if (sspi->gpio_base) {
		b = sspi->gpio_base + 32 * bank;
		dir = readl(b + 0x14);
		writel(dir | BIT(pin), b + 0x14);
	}
	writel((bank << 16) | (pin << 8) | func, sspi->gpiocmd);
}

static void s5l8702_spi2_pinmux(struct s5l8702_spi *sspi)
{
	void __iomem *b;
	u32 pin, punc;

	/* sub_20690(1): sub_23CD0(0x57, 0) clears PUNC (+0x10) on GPIO 87 */
	if (sspi->gpio_base) {
		b = sspi->gpio_base + 32 * (0x57 >> 3);
		pin = 0x57 & 7;
		punc = readl(b + 0x10);
		writel(punc & ~BIT(pin), b + 0x10);
	}
	s5l8702_gpiocmd_func(sspi, 0x57, 5);
	s5l8702_gpiocmd_func(sspi, 0x58, 3);
	s5l8702_gpiocmd_func(sspi, 0x59, 3);
	s5l8702_gpiocmd_func(sspi, 0x5A, 3);
}

/* OSOS sub_743A4: 43D38C(0,4) (1,2) (2,2) (3,2) then 11B70(0,0x1A,0x2EE0,8). */
static void s5l8702_spi0_pinmux(struct s5l8702_spi *sspi)
{
	s5l8702_gpiocmd_func(sspi, 0, 4);
	s5l8702_gpiocmd_func(sspi, 1, 2);
	s5l8702_gpiocmd_func(sspi, 2, 2);
	s5l8702_gpiocmd_func(sspi, 3, 2);
}

/*
 * sub_11B70(0, 0x1A, 0x2EE0, 8). Clock ids 4/5 via 43CFCC.
 * Id 5 mux 0/5 is fixed 24000 (3D7A2C). 440A58(24000,1000)=24,
 * 440A58(24000,0x2EE0)=2. Id 4 uses the same 24 k-unit if PLL field
 * is the usual 24 MHz-class source.
 */
static void s5l8702_spi0_11b70(struct s5l8702_spi *sspi)
{
	const unsigned int a4 = 8;
	const unsigned int clk_kunit = 24;
	u32 dd = clk_kunit * a4;
	u32 u3c = 3 * clk_kunit * (a4 + 1);
	u32 clkdiv = 2;

	writel(0xf, sspi->base + SPISTATUS);
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO,
	       sspi->base + SPICTRL);
	writel(10, sspi->base + 0x44);
	writel(dd, sspi->base + 0x38);
	writel(255, sspi->base + 0x40);
	writel(u3c, sspi->base + 0x3c);
	writel(clkdiv, sspi->base + SPICLKDIV);
	writel(0x6, sspi->base + SPIPIN);
	writel(SPISETUP_SPI0_11B70, sspi->base + SPISETUP);
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO,
	       sspi->base + SPICTRL);
	writel(SPICTRL_ENABLE, sspi->base + SPICTRL);
	sspi->prepared = true;
	spi_vinfo(sspi->dev,
		 "SPI0 11B70 SETUP=0x%x CLKDIV=%u dd=%u u3c=%u u40=255 u44=10\n",
		 SPISETUP_SPI0_11B70, clkdiv, dd, u3c);
}

/*
 * sub_11B70(2, 0x1A, 0x2EE0, 1) — same engine as SPI0, a4=1.
 * CLKDIV stays 2 (440A58(24000, 0x2EE0)). Do not use the generic
 * CLKDIV=4 path; that left +0x38/+0x3c at reset and ping RX was junk.
 */
/*
 * The RetailOS SPI2 oracle, captured with touch working, has +0x3c at
 * 0x18c. The formula below yields 0x90 for a4=1, and it is not a matter
 * of the wrong a4: solving 3*24*(a4+1) = 396 gives 4.5, so the
 * derivation itself does not reproduce the hardware. +0x38 does match
 * at 0x18, and this driver already records that leaving the pair at
 * reset made ping RX junk, so the register affects receive behaviour.
 * Overridable while that is being tested.
 */
static int spi2_u3c = 0x18c;
module_param(spi2_u3c, int, 0644);
MODULE_PARM_DESC(spi2_u3c,
		 "SPI2 +0x3c value (0 = computed 0x90, default 0x18c = stock)");

static struct s5l8702_spi *s5l8702_spi2_dev;

static void s5l8702_spi2_11b70(struct s5l8702_spi *sspi)
{
	const unsigned int a4 = 1;
	const unsigned int clk_kunit = 24;
	u32 dd = clk_kunit * a4;
	u32 u3c = spi2_u3c ? (u32)spi2_u3c : 3 * clk_kunit * (a4 + 1);
	u32 clkdiv = 2;

	/*
	 * sub_11B70(2, 0x1A, 0x2EE0, 1) — mode 0x1A → SETUP 0x402E|0x10
	 * = 0x403E. Do not use 0x403C (that is a different mode bit).
	 * OSOS does not write SPIPIN or STATUS here.
	 */
	writel(10, sspi->base + 0x44);
	writel(dd, sspi->base + 0x38);
	writel(255, sspi->base + 0x40);
	writel(u3c, sspi->base + 0x3c);
	writel(clkdiv, sspi->base + SPICLKDIV);
	writel(SPISETUP_SPI0_11B70, sspi->base + SPISETUP);
	writel(SPICTRL_ENABLE, sspi->base + SPICTRL);
	sspi->prepared = true;
	spi_vinfo(sspi->dev,
		 "SPI2 11B70 SETUP=0x%x CLKDIV=%u dd=%u u3c=%u (mode 0x1A)\n",
		 SPISETUP_SPI0_11B70, clkdiv, dd, u3c);
}

/*
 * Re-run the SPI2 engine setup. The stock firmware reapplies sub_11B70
 * after every pinmux enable, and the touch driver does the same on
 * bring-up; routing it here keeps one owner for the divider and timing.
 */
void s5l8702_spi2_reinit(void)
{
	if (s5l8702_spi2_dev)
		s5l8702_spi2_11b70(s5l8702_spi2_dev);
}
EXPORT_SYMBOL_GPL(s5l8702_spi2_reinit);

static void s5l8702_spi_cs(struct s5l8702_spi *sspi, bool assert)
{
	u32 pin = readl(sspi->base + SPIPIN);

	if (assert)
		pin &= ~SPIPIN_CS_BIT;
	else
		pin |= SPIPIN_CS_BIT;
	writel(pin, sspi->base + SPIPIN);
}

static const char *s5l8702_fam_name(enum s5l8702_spi_fam f)
{
	switch (f) {
	case SPI_FAM_ROS:
		return "ROS(0x7C0/0xF800)";
	case SPI_FAM_CLASSIC:
		return "Classic(0x1F0/0x3E00)";
	default:
		return "auto";
	}
}

/* Latch the observed encoding once, and say so. */
static void s5l8702_latch_fam(struct s5l8702_spi *sspi,
			      enum s5l8702_spi_fam f, u32 status)
{
	if (sspi->fam == f)
		return;
	sspi->fam = f;
	sspi->fam_latch_status = status;
	dev_info(sspi->dev, "status encoding latched: %s (SPISTATUS=0x%08x)\n",
		 s5l8702_fam_name(f), status);
}

/*
 * Wait for the transmit side to go idle.
 *
 * Only the latched family is consulted. While still AUTO both are polled in
 * the same loop so the first genuine completion decides, rather than one
 * family being given a full guard interval of head start.
 */
static int s5l8702_wait_tx_idle(struct s5l8702_spi *sspi)
{
	unsigned int guard = SPI_WAIT_GUARD;
	u32 val = 0;

	while (guard--) {
		val = readl(sspi->base + SPISTATUS);

		if (sspi->fam != SPI_FAM_CLASSIC &&
		    (val & SPISTATUS_TXBUSY_ROS) == 0) {
			s5l8702_latch_fam(sspi, SPI_FAM_ROS, val);
			return 0;
		}
		if (sspi->fam != SPI_FAM_ROS &&
		    (val & SPISTATUS_TXLVL_MASK) == 0) {
			s5l8702_latch_fam(sspi, SPI_FAM_CLASSIC, val);
			return 0;
		}
		cpu_relax();
	}

	sspi->tx_timeouts++;
	sspi->last_status = val;
	return -ETIMEDOUT;
}

/* Wait for received data to be available. Same latching rule as TX. */
static int s5l8702_wait_rx_ready(struct s5l8702_spi *sspi)
{
	unsigned int guard = SPI_WAIT_GUARD;
	u32 val = 0;

	while (guard--) {
		val = readl(sspi->base + SPISTATUS);

		if (sspi->fam != SPI_FAM_CLASSIC &&
		    (val & SPISTATUS_RXRDY_ROS)) {
			s5l8702_latch_fam(sspi, SPI_FAM_ROS, val);
			return 0;
		}
		if (sspi->fam != SPI_FAM_ROS &&
		    (val & SPISTATUS_RXLVL_MASK)) {
			s5l8702_latch_fam(sspi, SPI_FAM_CLASSIC, val);
			return 0;
		}
		cpu_relax();
	}

	sspi->rx_timeouts++;
	sspi->last_status = val;
	return -ETIMEDOUT;
}

/*
 * Thin shims so existing call sites keep reading naturally. The mask
 * argument now only selects which side is meant, not which encoding.
 */
static int s5l8702_wait_clear(struct s5l8702_spi *sspi, u32 mask)
{
	if (mask == SPISTATUS_TXBUSY_ROS || mask == SPISTATUS_TXLVL_MASK)
		return s5l8702_wait_tx_idle(sspi);

	/* Anything else is a literal wait on the caller's own mask. */
	{
		unsigned int guard = SPI_WAIT_GUARD;

		while (guard--) {
			if ((readl(sspi->base + SPISTATUS) & mask) == 0)
				return 0;
			cpu_relax();
		}
	}
	return -ETIMEDOUT;
}

static int s5l8702_wait_set(struct s5l8702_spi *sspi, u32 mask)
{
	if (mask == SPISTATUS_RXRDY_ROS || mask == SPISTATUS_RXLVL_MASK)
		return s5l8702_wait_rx_ready(sspi);

	{
		unsigned int guard = SPI_WAIT_GUARD;

		while (guard--) {
			if (readl(sspi->base + SPISTATUS) & mask)
				return 0;
			cpu_relax();
		}
	}
	return -ETIMEDOUT;
}

static void s5l8702_spi_hw_init(struct s5l8702_spi *sspi)
{
	writel(0xf, sspi->base + SPISTATUS);
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO,
	       sspi->base + SPICTRL);
	writel(SPI_CLKDIV_DEFAULT, sspi->base + SPICLKDIV);
	/* idle: CS deasserted (bit1 set), match prior SPIPIN=6 */
	writel(0x6, sspi->base + SPIPIN);
	writel(SPISETUP_RETAILOS, sspi->base + SPISETUP);
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO,
	       sspi->base + SPICTRL);
	writel(SPICTRL_ENABLE, sspi->base + SPICTRL);
	sspi->prepared = true;
}

/* Rockbox touch-nano7g: CLKDIV=4, SETUP 0x402C|0x10, CTRL=1. No SPIPIN. */
static void s5l8702_spi2_hw_init(struct s5l8702_spi *sspi)
{
	writel(4, sspi->base + SPICLKDIV);
	writel(0x402c, sspi->base + SPISETUP);
	writel(SPISETUP_RETAILOS, sspi->base + SPISETUP);
	writel(SPICTRL_ENABLE, sspi->base + SPICTRL);
	sspi->prepared = true;
}

static int s5l8702_spi2_pio_one(struct s5l8702_spi *sspi,
				const u8 *tx, u8 *rx, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i++) {
		unsigned int guard = SPI_WAIT_GUARD;
		u32 st;
		int ret;

		writel(1, sspi->base + SPIRXLIMIT);

		/*
		 * TX-full is the one test that is genuinely encoding-specific
		 * here (0x100 is the full flag within the Classic level field),
		 * so keep it, but only while this instance is actually running
		 * the Classic encoding. Otherwise defer to the latched waits --
		 * this loop previously asserted Classic unconditionally, which
		 * is how a ROS-encoded SPI2 ended up sampling RXDATA early.
		 */
		if (sspi->fam == SPI_FAM_CLASSIC) {
			while (guard--) {
				st = readl(sspi->base + SPISTATUS);
				if ((st & SPISTATUS_TXFULL) == 0)
					break;
				cpu_relax();
			}
			if (readl(sspi->base + SPISTATUS) & SPISTATUS_TXFULL) {
				sspi->tx_timeouts++;
				return -ETIMEDOUT;
			}
		} else {
			ret = s5l8702_wait_tx_idle(sspi);
			if (ret)
				return ret;
		}

		writel(tx ? tx[i] : 0xff, sspi->base + SPITXDATA);

		ret = s5l8702_wait_rx_ready(sspi);
		if (ret)
			return ret;
		{
			u8 b = (u8)readl(sspi->base + SPIRXDATA);

			if (rx)
				rx[i] = b;
		}
	}
	return 0;
}

static void s5l8702_spi_set_cs(struct spi_device *spi, bool enable)
{
	/* CS is owned by transfer_one (RetailOS 4045D4 order) */
	(void)spi;
	(void)enable;
}

static int s5l8702_spi_prepare_message(struct spi_controller *ctlr,
				       struct spi_message *msg)
{
	struct s5l8702_spi *sspi = spi_controller_get_devdata(ctlr);

	if (!sspi->prepared)
		s5l8702_spi_hw_init(sspi);
	return 0;
}

/*
 * One transfer's worth of PIO.
 *
 * @assert_cs:   pull CS down before the preamble. False when a previous
 *               transfer already left it down.
 * @deassert_cs: release CS when finished. False when the next transfer in
 *               this message must see the same selection.
 *
 * This used to assert and release unconditionally, which meant CS dropped
 * between the transfers of a multi-transfer message. For a protocol that
 * frames on chip select -- HBPP being the one that matters here -- that
 * silently splits one frame into several, so a caller could not express a
 * held-CS sequence through the SPI core at all and had to drive the
 * registers itself. Honouring cs_change is what makes spi_sync() usable.
 */
static int s5l8702_spi_pio_one(struct s5l8702_spi *sspi,
			       const u8 *tx, u8 *rx, unsigned int len,
			       bool assert_cs, bool deassert_cs)
{
	unsigned int i;
	int ret;

	if (assert_cs)
		s5l8702_spi_cs(sspi, true);
	sspi->cs_held = true;

	/* sub_4043D0 preamble */
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO,
	       sspi->base + SPICTRL);
	ret = s5l8702_wait_clear(sspi, SPISTATUS_TXBUSY_ROS);
	/* After 11B70, STATUS bit6 (0x40) stays set; 0x7C0 never hits 0. */
	if (ret && (readl(sspi->base + SPISTATUS) & SPISTATUS_TXBUSY_ROS) == 0x40)
		ret = 0;
	if (ret)
		goto out_cs;
	ret = s5l8702_wait_clear(sspi, SPISTATUS_RXRDY_ROS);
	if (ret)
		goto out_cs;

	/*
	 * 4043D0: TX present → SETUP &= ~1, STATUS |= 0x400000.
	 * RX-only (tx==NULL) → SETUP |= 1, STATUS |= 1 (auto-clock).
	 */
	if (tx) {
		writel(readl(sspi->base + SPISETUP) & ~SPISETUP_RXMODE,
		       sspi->base + SPISETUP);
		writel(readl(sspi->base + SPISTATUS) | SPISTATUS_KICK,
		       sspi->base + SPISTATUS);
	} else {
		writel(readl(sspi->base + SPISETUP) | SPISETUP_RXMODE,
		       sspi->base + SPISETUP);
		writel(readl(sspi->base + SPISTATUS) | 1,
		       sspi->base + SPISTATUS);
	}

	for (i = 0; i < len; i++) {
		/* 4043D0: RXLIMIT=1 only when the call has an RX buffer */
		writel(rx ? 1 : 0, sspi->base + SPIRXLIMIT);

		ret = s5l8702_wait_clear(sspi, SPISTATUS_TXBUSY_ROS);
		if (ret && (readl(sspi->base + SPISTATUS) & SPISTATUS_TXBUSY_ROS) == 0x40)
			ret = 0;
		if (ret)
			break;

		if (tx) {
			writel(tx[i], sspi->base + SPITXDATA);
			/* 4043D0 PIO: * (base+0x4C) = 1 after each TX word */
			writel(1, sspi->base + SPIUNK4C);
		}

		if (rx) {
			ret = s5l8702_wait_set(sspi, SPISTATUS_RXRDY_ROS);
			if (ret)
				break;
			rx[i] = (u8)readl(sspi->base + SPIRXDATA);
		}
	}
	ret = s5l8702_wait_clear(sspi, SPISTATUS_TXBUSY_ROS);
	if (ret && (readl(sspi->base + SPISTATUS) & SPISTATUS_TXBUSY_ROS) == 0x40)
		ret = 0;

	/* sub_4043D0 epilogue: SETUP &= ~0x400001 */
	writel(readl(sspi->base + SPISETUP) & ~0x400001u,
	       sspi->base + SPISETUP);

out_cs:
	/*
	 * Always drop CS on error: leaving it asserted after a timeout would
	 * strand the bus for every later message.
	 */
	if (deassert_cs || ret) {
		s5l8702_spi_cs(sspi, false);
		sspi->cs_held = false;
	}
	if (ret) {
		sspi->last_err = ret;
		sspi->last_status = readl(sspi->base + SPISTATUS);
		dev_err_ratelimited(sspi->dev,
				    "4043D0 timeout st=%08x setup=%08x ctrl=%08x pin=%08x dd=%08x u3c=%08x u40=%08x u44=%08x\n",
				    sspi->last_status,
				    readl(sspi->base + SPISETUP),
				    readl(sspi->base + SPICTRL),
				    readl(sspi->base + SPIPIN),
				    readl(sspi->base + 0x38),
				    readl(sspi->base + 0x3c),
				    readl(sspi->base + 0x40),
				    readl(sspi->base + 0x44));
		dev_err_ratelimited(sspi->dev, "4043D0 u4c=%08x\n",
				    readl(sspi->base + SPIUNK4C));
	}
	return ret;
}

/*
 * Map the SPI core's cs_change rules onto the CS line.
 *
 * Within a message CS stays down between transfers; a transfer with
 * cs_change set toggles it afterwards. On the final transfer the meaning
 * inverts -- cs_change there means keep the device selected past the end
 * of this message, which is how a caller holds one frame across several
 * spi_sync() calls.
 */
static int s5l8702_spi_transfer_one(struct spi_controller *ctlr,
				    struct spi_device *spi,
				    struct spi_transfer *xfer)
{
	struct s5l8702_spi *sspi = spi_controller_get_devdata(ctlr);
	struct spi_message *msg = ctlr->cur_msg;
	bool last = true;
	bool deassert;

	(void)spi;

	if (msg)
		last = list_is_last(&xfer->transfer_list, &msg->transfers);

	if (last)
		deassert = !xfer->cs_change;
	else
		deassert = xfer->cs_change;

	return s5l8702_spi_pio_one(sspi, xfer->tx_buf, xfer->rx_buf,
				   xfer->len, !sspi->cs_held, deassert);
}

static int s5l8702_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct s5l8702_spi *sspi;
	struct resource *res;
	int ret;

	ctlr = devm_spi_alloc_host(&pdev->dev, sizeof(*sspi));
	if (!ctlr)
		return -ENOMEM;

	sspi = spi_controller_get_devdata(ctlr);
	sspi->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sspi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sspi->base))
		return PTR_ERR(sspi->base);

	sspi->spi0 = res && res->start == SPI0_BASE_PHYS;
	sspi->spi2_nimbus = res && res->start == SPI2_BASE_PHYS;

	/*
	 * Start in AUTO unless told otherwise, so each instance reports the
	 * encoding it actually uses instead of inheriting an assumption. The
	 * two instances have been seen to differ depending on which init path
	 * ran, which is precisely why this is per-instance and not global.
	 */
	switch (spi_family) {
	case 1:
		sspi->fam = SPI_FAM_ROS;
		break;
	case 2:
		sspi->fam = SPI_FAM_CLASSIC;
		break;
	default:
		sspi->fam = SPI_FAM_AUTO;
		break;
	}

	/* Optional DT clocks (CLK_SPI* / secondary); ignore -ENOENT */
	ret = devm_clk_bulk_get_all(&pdev->dev, &sspi->clks);
	if (ret > 0) {
		sspi->num_clks = ret;
		ret = clk_bulk_prepare_enable(sspi->num_clks, sspi->clks);
		if (ret)
			dev_warn(&pdev->dev, "clk_bulk_prepare_enable failed: %d\n", ret);
		else
			spi_vinfo(&pdev->dev, "enabled %d SPI clockgate(s)\n", sspi->num_clks);
	} else {
		sspi->num_clks = 0;
	}

	if (sspi->spi0) {
		sspi->pwrcon1 = devm_ioremap(&pdev->dev, S5L8702_PWRCON1_PHYS, 4);
		sspi->gpiocmd = devm_ioremap(&pdev->dev, S5L8702_GPIOCMD_PHYS, 4);
		sspi->gpio_base = devm_ioremap(&pdev->dev, S5L8702_PCON0_PHYS, 0x400);
		if (!sspi->pwrcon1 || !sspi->gpiocmd || !sspi->gpio_base)
			return -ENOMEM;
		writel(readl(sspi->pwrcon1) & ~PWRCON1_SPI0_BIT, sspi->pwrcon1);
		{
			void __iomem *pwrcon4 = ioremap(S5L8702_PWRCON4_PHYS, 4);

			if (pwrcon4) {
				writel(readl(pwrcon4) & ~PWRCON4_SPI0_2_BIT, pwrcon4);
				iounmap(pwrcon4);
			}
		}
		s5l8702_spi0_pinmux(sspi);
		s5l8702_spi0_11b70(sspi);
		spi_vinfo(&pdev->dev, "SPI0 CS42 4043D0 CS=SPIPIN.1 PWRCON1=%08x\n",
			 readl(sspi->pwrcon1));
	} else if (sspi->spi2_nimbus) {
		void __iomem *pwrcon4;

		sspi->pwrcon1 = devm_ioremap(&pdev->dev, S5L8702_PWRCON1_PHYS, 4);
		sspi->gpiocmd = devm_ioremap(&pdev->dev, S5L8702_GPIOCMD_PHYS, 4);
		sspi->gpio_base = devm_ioremap(&pdev->dev, S5L8702_PCON0_PHYS, 0x400);
		if (!sspi->pwrcon1 || !sspi->gpiocmd || !sspi->gpio_base)
			return -ENOMEM;
		/* bit16 = 8702 SPI2; bit15 = 8720 SPI2 / I2S2 overlap — clear both */
		writel(readl(sspi->pwrcon1) & ~(PWRCON1_SPI2_BIT | BIT(15)),
		       sspi->pwrcon1);
		pwrcon4 = ioremap(0x3c50006cUL, 4);
		if (pwrcon4) {
			writel(readl(pwrcon4) & ~BIT(15), pwrcon4); /* SPI2_2 */
			iounmap(pwrcon4);
		}
		spi_vinfo(&pdev->dev, "SPI2 PWRCON1=%08x (after ungate)\n",
			 readl(sspi->pwrcon1));
		s5l8702_spi2_pinmux(sspi);
		s5l8702_spi2_11b70(sspi);
		s5l8702_spi2_dev = sspi;
		spi_vinfo(&pdev->dev,
			 "SPI2 Nimbus 4043D0 PIO (SETUP=0x%x CLKDIV=2 CS=SPIPIN.1 11B70)\n",
			 SPISETUP_SPI0_11B70);
	}

	ctlr->dev.of_node = pdev->dev.of_node;
	ctlr->bus_num = pdev->id;
	ctlr->num_chipselect = 1;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->set_cs = s5l8702_spi_set_cs;
	ctlr->prepare_message = s5l8702_spi_prepare_message;
	ctlr->transfer_one = s5l8702_spi_transfer_one;

	platform_set_drvdata(pdev, ctlr);
	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret)
		dev_err(&pdev->dev, "failed to register SPI controller: %d\n", ret);
	return ret;
}

static const struct of_device_id s5l8702_spi_of_match[] = {
	{ .compatible = "apple,s5l8702-spi" },
	{ .compatible = "samsung,s5l8702-spi" },
	{ .compatible = "samsung,s5l8740-spi" },
	{}
};
MODULE_DEVICE_TABLE(of, s5l8702_spi_of_match);

static struct platform_driver s5l8702_spi_driver = {
	.probe  = s5l8702_spi_probe,
	.driver = {
		.name           = "spi-s5l8702",
		.of_match_table = s5l8702_spi_of_match,
	},
};
module_platform_driver(s5l8702_spi_driver);

MODULE_DESCRIPTION("SPI controller driver for Samsung/Apple S5L87xx");
MODULE_LICENSE("GPL v2");
