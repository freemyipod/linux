// SPDX-License-Identifier: GPL-2.0
/*
 * SPI controller driver for Samsung/Apple S5L8702
 * (iPod nano 3rd generation)
 *
 * Ported from Rockbox's spi-s5l8702.c by Michael Sparmann.
 */
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define SPICTRL		0x00
#define SPISETUP	0x04
#define SPISTATUS	0x08
#define SPIPIN		0x0c
#define SPITXDATA	0x10
#define SPIRXDATA	0x20
#define SPICLKDIV	0x30
#define SPIRXLIMIT	0x34

// SPISTATUS: TX FIFO level in bits [8:4] (max 16 = full),
//            RX FIFO level in bits [13:9].
#define SPISTATUS_TXFULL		0x100	// TX FIFO level == 16 (full)
#define SPISTATUS_TXLVL_MASK	0x1f0
#define SPISTATUS_RXLVL_MASK	0x3e00	// any RX data present

// SPISETUP: bit 0 enables bulk-receive mode.
#define SPISETUP_RXMODE		BIT(0)
#define SPISETUP_INIT		0x10618

// SPICTRL: bits 3:2 reset FIFOs, bit 0 enables the controller.
#define SPICTRL_RESET_FIFO	0xc
#define SPICTRL_ENABLE		0x1

// Fixed SoC peripheral addresses.
#define S5L8702_PCON0_PHYS		0x3cf00000UL
#define S5L8702_GPIOCMD_PHYS	0x3cf00200UL

// PWRCON(1) = 0x3C500048 + 4*1; clockgate 34 sits in PWRCON1, bit 2
#define S5L8702_PWRCON1_PHYS	0x3c50004cUL
#define PWRCON1_SPI0_BIT		BIT(2)

// PCON0[15:0]: set pins 0-3 to SPI function (each 4 bits = 0x2)
#define PCON0_SPI_MASK		0xffffU
#define PCON0_SPI_FUNC		0x2222U

// GPIOCMD encoding for SPI0 CS: port 0, pin 0; 0xe = output-low, 0xf = output-high
#define GPIOCMD_SPI0_CS_ASSERT		0x0000eU
#define GPIOCMD_SPI0_CS_DEASSERT	0x0000fU

// Default clock divider: SPI clock = PClk / (div + 1)
#define SPI0_CLKDIV_DEFAULT	4

// Worst-case timeout for a single FIFO slot
#define SPI_POLL_TIMEOUT_US	100000

struct s5l8702_spi {
	void __iomem *base;
	void __iomem *pcon0;
	void __iomem *gpiocmd;
	void __iomem *pwrcon1;
};

static int s5l8702_spi_wait_rx(struct s5l8702_spi *sspi) {
	u32 val;
	return readl_poll_timeout(sspi->base + SPISTATUS, val, val & SPISTATUS_RXLVL_MASK, 0, SPI_POLL_TIMEOUT_US);
}

static int s5l8702_spi_wait_tx(struct s5l8702_spi *sspi) {
	u32 val;
	return readl_poll_timeout(sspi->base + SPISTATUS, val, (val & SPISTATUS_TXLVL_MASK) != SPISTATUS_TXFULL, 0, SPI_POLL_TIMEOUT_US);
}

static void s5l8702_spi_prepare(struct s5l8702_spi *sspi) {
	writel(0xf, sspi->base + SPISTATUS);
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO, sspi->base + SPICTRL);
	writel(SPI0_CLKDIV_DEFAULT, sspi->base + SPICLKDIV);
	writel(6, sspi->base + SPIPIN);
	writel(SPISETUP_INIT, sspi->base + SPISETUP);
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO, sspi->base + SPICTRL);
	writel(SPICTRL_ENABLE, sspi->base + SPICTRL);
}

static void s5l8702_spi_set_cs(struct spi_device *spi, bool enable) {
	struct s5l8702_spi *sspi = spi_controller_get_devdata(spi->controller);
	writel(enable ? GPIOCMD_SPI0_CS_DEASSERT : GPIOCMD_SPI0_CS_ASSERT, sspi->gpiocmd);
}

static int s5l8702_spi_prepare_message(struct spi_controller *ctlr, struct spi_message *msg) {
	s5l8702_spi_prepare(spi_controller_get_devdata(ctlr));
	return 0;
}

static int s5l8702_spi_transfer_one(struct spi_controller *ctlr, struct spi_device *spi, struct spi_transfer *xfer) {
	struct s5l8702_spi *sspi = spi_controller_get_devdata(ctlr);
	const u8 *tx = xfer->tx_buf;
	u8 *rx = xfer->rx_buf;
	unsigned int len = xfer->len;
	unsigned int i;
	int ret = 0;

	if (rx && !tx) {
		// Pure receive: program SPIRXLIMIT and set RXMODE.
		writel(len, sspi->base + SPIRXLIMIT);
		writel(readl(sspi->base + SPISETUP) | SPISETUP_RXMODE, sspi->base + SPISETUP);
		for (i = 0; i < len; i++) {
			ret = s5l8702_spi_wait_rx(sspi);
			if (ret) goto out_rxmode;
			rx[i] = readl(sspi->base + SPIRXDATA);
		}
out_rxmode:
		writel(readl(sspi->base + SPISETUP) & ~SPISETUP_RXMODE, sspi->base + SPISETUP);
	} else {
		// TX or full-duplex.
		for (i = 0; i < len; i++) {
			writel(1, sspi->base + SPIRXLIMIT);
			ret = s5l8702_spi_wait_tx(sspi);
			if (ret) break;
			writel(tx ? tx[i] : 0xff, sspi->base + SPITXDATA);
			ret = s5l8702_spi_wait_rx(sspi);
			if (ret) break;
			if (rx) rx[i] = readl(sspi->base + SPIRXDATA);
			else readl(sspi->base + SPIRXDATA);
		}
	}
	return ret;
}

static int s5l8702_spi_probe(struct platform_device *pdev) {
	struct spi_controller *ctlr;
	struct s5l8702_spi *sspi;
	int ret;

	ctlr = devm_spi_alloc_host(&pdev->dev, sizeof(*sspi));
	if (!ctlr) return -ENOMEM;

	sspi = spi_controller_get_devdata(ctlr);
	sspi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sspi->base)) return PTR_ERR(sspi->base);

	sspi->pcon0 = devm_ioremap(&pdev->dev, S5L8702_PCON0_PHYS, 4);
	sspi->gpiocmd = devm_ioremap(&pdev->dev, S5L8702_GPIOCMD_PHYS, 4);
	sspi->pwrcon1 = devm_ioremap(&pdev->dev, S5L8702_PWRCON1_PHYS, 4);
	if (!sspi->pcon0 || !sspi->gpiocmd || !sspi->pwrcon1) return -ENOMEM;

	// Route GPIO pins to SPI function
	writel((readl(sspi->pcon0) & ~PCON0_SPI_MASK) | PCON0_SPI_FUNC, sspi->pcon0);

	// Enable SPI0 clock gate
	writel(readl(sspi->pwrcon1) & ~PWRCON1_SPI0_BIT, sspi->pwrcon1);

	// Deassert CS
	writel(GPIOCMD_SPI0_CS_DEASSERT, sspi->gpiocmd);

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
	if (ret) dev_err(&pdev->dev, "failed to register SPI controller: %d\n", ret);
	return ret;
}

static const struct of_device_id s5l8702_spi_of_match[] = {
	{ .compatible = "apple,s5l8702-spi" },
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

MODULE_DESCRIPTION("SPI controller driver for Samsung/Apple S5L8702");
MODULE_AUTHOR("Tucker Osman <osmiumusa@gmail.com>");
MODULE_LICENSE("GPL v2");
