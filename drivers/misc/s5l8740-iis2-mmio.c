// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 IIS2 @ 0x3D400000 — BCM2078 digital PCM port (N31).
 *
 * RetailOS oracles (fm/, bt-*-scsi-live/):
 *   IIS1 @ 0x3CD00000 = XSP, always zero — NOT BCM TX.
 *   IIS2 = shared BCM2078 I²S: RX FIFO @ +0x38 (PL080 peri 13, FM + module PCM in),
 *   TXCON @ +0x04 = 0x0b000099 programmed for music/FM/BT (TXCOM often 0 on BT).
 *   BT A2DP over-the-air = UART1 @ 0x3DB HCI → BCM2078 (no IIS0/CS42).
 *   CLKCON  +0x00 = 0x1
 *   TXCON   +0x04 = 0x0b000099   (RetailOS programs this on IIS2 too)
 *   RXCON   +0x30 = 0x1000
 *   RXCOM   +0x34 = 0x6          (DMA kick; idle/stopped often 0x2)
 *   RXFIFO  +0x38 ← PL080 peri 13 P2M
 *   STATUS  +0x3c = 0x10804 live
 *   CLKDIV  +0x40 = 0x96         (FM oracle; IIS0 play uses 0x177/375)
 *   REG44   +0x44 = 0x00010007   (same as IIS0 music oracle)
 *
 * SoC clocks: CLKCON+0x30 = 0x32190 play; FM also +0x10 = 0x4
 * (vs music/idle 0x8004). No FM→BT / A2DP path here — local PCM only.
 */
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "n31-audio-rates.h"

#define S5L8740_IIS2_RATES	(SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)
#define S5L8740_IIS2_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE)

#define I2SCLKCON		0x00
#define I2STXCON		0x04
#define I2SRXCON		0x30
#define I2SRXCOM		0x34
#define I2SRXFIFO		0x38
#define I2SSTATUS		0x3c
#define I2SCLKDIV		0x40
#define I2SREG44		0x44

#define MCLK_ASSUME_HZ		12000000u

/* fm/20260826T203834Z oracle */
#define IIS2_CLKCON_ON		0x1u
#define IIS2_TXCON_FM		0x0b000099u
#define IIS2_RXCON_FM		0x1000u
#define IIS2_RXCOM_DMA		0x6u
#define IIS2_RXCOM_IDLE		0x2u
#define IIS2_CLKDIV_FM_ORACLE	0x96u
#define IIS2_REG44_ORACLE	0x00010007u

static uint iis2_clkdiv;
module_param(iis2_clkdiv, uint, 0644);
MODULE_PARM_DESC(iis2_clkdiv, "IIS2 CLKDIV override; 0 = FM oracle 0x96");

#define CLKCON_PHYS		0x3c500000ul
#define CLKCON_AUDIO_OFF	0x30
#define CLKCON_FM_GATE_OFF	0x10
#define CLKCON_AUDIO_PLAY	0x32190u
#define CLKCON_AUDIO_IDLE	0x1c20u
#define CLKCON_FM_GATE_ON	0x4u
#define CLKCON_FM_GATE_OFF_VAL	0x8004u

#define IIS2_REGS_LEN		0x48

struct s5l8740_iis2 {
	void __iomem *base;
	void __iomem *clkcon;
	struct device *dev;
	struct clk_bulk_data *clks;
	int num_clks;
	bool has_dma;
	struct snd_dmaengine_dai_dma_data cap_dma;
	u32 clkcon10_saved;
	bool clkcon10_held;
	unsigned int rate;
};

static u32 iis2_pick_clkdiv(unsigned int rate)
{
	const struct n31_rate_cfg *r;

	if (iis2_clkdiv)
		return iis2_clkdiv;
	/*
	 * FM IIS2 oracle differs from IIS0: 0x96 while IIS0 HP path uses
	 * 0x177 (32 kHz table entry) during the same FM session.
	 */
	if (rate == 44100 || rate == 48000)
		return IIS2_CLKDIV_FM_ORACLE;
	r = n31_find_rate(rate);
	if (r)
		return r->clkdiv;
	if (!rate)
		rate = 44100;
	return MCLK_ASSUME_HZ / rate;
}

static void iis2_clkcon_audio(struct s5l8740_iis2 *iis2, u32 val)
{
	if (!iis2 || !iis2->clkcon)
		return;
	writel(val, iis2->clkcon + CLKCON_AUDIO_OFF);
}

static void iis2_clkcon_fm_gate(struct s5l8740_iis2 *iis2, bool on)
{
	u32 cur;

	if (!iis2 || !iis2->clkcon)
		return;
	cur = readl(iis2->clkcon + CLKCON_FM_GATE_OFF);
	if (on) {
		if (!iis2->clkcon10_held) {
			iis2->clkcon10_saved = cur;
			iis2->clkcon10_held = true;
		}
		writel(CLKCON_FM_GATE_ON, iis2->clkcon + CLKCON_FM_GATE_OFF);
	} else if (iis2->clkcon10_held) {
		writel(iis2->clkcon10_saved ?
		       iis2->clkcon10_saved : CLKCON_FM_GATE_OFF_VAL,
		       iis2->clkcon + CLKCON_FM_GATE_OFF);
		iis2->clkcon10_held = false;
	}
}

/*
 * Program IIS2 RX from fm-playing dump. Peri 13 DMA must be armed by
 * dmaengine before RXCOM |= 0x6 (same kick model as IIS0 TXCOM).
 */
static void iis2_program_rx(struct s5l8740_iis2 *iis2)
{
	u32 div;

	iis2_clkcon_fm_gate(iis2, true);
	iis2_clkcon_audio(iis2, CLKCON_AUDIO_PLAY);
	writel(IIS2_CLKCON_ON, iis2->base + I2SCLKCON);
	writel(IIS2_TXCON_FM, iis2->base + I2STXCON);
	writel(IIS2_RXCON_FM, iis2->base + I2SRXCON);
	div = iis2_pick_clkdiv(iis2->rate);
	writel(div, iis2->base + I2SCLKDIV);
	writel(IIS2_REG44_ORACLE, iis2->base + I2SREG44);
}

static void iis2_rx_kick(struct s5l8740_iis2 *iis2)
{
	writel(IIS2_RXCOM_DMA, iis2->base + I2SRXCOM);
}

static void iis2_hw_stop(struct s5l8740_iis2 *iis2)
{
	if (!iis2 || !iis2->base)
		return;
	writel(IIS2_RXCOM_IDLE, iis2->base + I2SRXCOM);
	iis2_clkcon_audio(iis2, CLKCON_AUDIO_IDLE);
	iis2_clkcon_fm_gate(iis2, false);
}

static int s5l8740_iis2_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct s5l8740_iis2 *iis2 = snd_soc_dai_get_drvdata(dai);

	if (!iis2 || !iis2->base)
		return -ENODEV;
	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;
	iis2->rate = params_rate(params);
	iis2_program_rx(iis2);
	dev_info(dai->dev,
		 "IIS2 hw_params rate=%u ch=%u clkdiv=0x%x reg44=0x%x status=0x%x\n",
		 iis2->rate, params_channels(params),
		 readl(iis2->base + I2SCLKDIV), readl(iis2->base + I2SREG44),
		 readl(iis2->base + I2SSTATUS));
	return 0;
}

static int s5l8740_iis2_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	struct s5l8740_iis2 *iis2 = snd_soc_dai_get_drvdata(dai);

	if (!iis2 || !iis2->base)
		return -ENODEV;
	if (substream->stream != SNDRV_PCM_STREAM_CAPTURE)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		iis2_program_rx(iis2);
		iis2_rx_kick(iis2);
		dev_info(dai->dev, "IIS2 capture start rxcom=0x%x status=0x%x\n",
			 readl(iis2->base + I2SRXCOM),
			 readl(iis2->base + I2SSTATUS));
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		iis2_hw_stop(iis2);
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5l8740_iis2_dai_probe(struct snd_soc_dai *dai)
{
	struct s5l8740_iis2 *iis2 = snd_soc_dai_get_drvdata(dai);

	if (iis2->has_dma)
		snd_soc_dai_init_dma_data(dai, NULL, &iis2->cap_dma);
	return 0;
}

static const struct snd_soc_dai_ops s5l8740_iis2_dai_ops = {
	.probe = s5l8740_iis2_dai_probe,
	.hw_params = s5l8740_iis2_hw_params,
	.trigger = s5l8740_iis2_trigger,
};

static struct snd_soc_dai_driver s5l8740_iis2_dai = {
	.name = "bcm2078-pcm",
	.capture = {
		.stream_name = "BCM2078 PCM Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = S5L8740_IIS2_RATES,
		.formats = S5L8740_IIS2_FORMATS,
	},
	.ops = &s5l8740_iis2_dai_ops,
};

static const struct snd_soc_component_driver s5l8740_iis2_component = {
	.name = "bcm2078-pcm",
	.legacy_dai_naming = 1,
};

static ssize_t regs_show(struct device *dev, struct device_attribute *a,
			 char *buf)
{
	struct s5l8740_iis2 *iis2 = dev_get_drvdata(dev);
	unsigned int i;
	ssize_t n = 0;

	if (!iis2 || !iis2->base)
		return sysfs_emit(buf, "not mapped\n");

	for (i = 0; i < IIS2_REGS_LEN; i += 4) {
		n += sysfs_emit_at(buf, n, "%02x: %08x\n", i,
				   readl(iis2->base + i));
		if (n >= PAGE_SIZE - 32)
			break;
	}
	if (iis2->clkcon) {
		n += sysfs_emit_at(buf, n, "clk+10: %08x\n",
				   readl(iis2->clkcon + CLKCON_FM_GATE_OFF));
		n += sysfs_emit_at(buf, n, "clk+30: %08x\n",
				   readl(iis2->clkcon + CLKCON_AUDIO_OFF));
	}
	return n;
}
static DEVICE_ATTR_RO(regs);

static int s5l8740_iis2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l8740_iis2 *iis2;
	struct resource *res;
	int ret;

	iis2 = devm_kzalloc(dev, sizeof(*iis2), GFP_KERNEL);
	if (!iis2)
		return -ENOMEM;
	iis2->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iis2->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(iis2->base))
		return PTR_ERR(iis2->base);

	iis2->clkcon = devm_ioremap(dev, CLKCON_PHYS, 0x80);

	ret = devm_clk_bulk_get_all(dev, &iis2->clks);
	if (ret > 0) {
		iis2->num_clks = ret;
		ret = clk_bulk_prepare_enable(iis2->num_clks, iis2->clks);
		if (ret)
			dev_warn(dev, "clk_bulk: %d\n", ret);
	}

	if (res) {
		iis2->cap_dma.addr = res->start + I2SRXFIFO;
		iis2->cap_dma.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		iis2->cap_dma.maxburst = 1;
	}

	platform_set_drvdata(pdev, iis2);
	dev_set_drvdata(dev, iis2);

	if (of_property_present(dev->of_node, "dmas")) {
		ret = devm_snd_dmaengine_pcm_register(dev, NULL, 0);
		if (ret) {
			dev_err(dev, "dmaengine_pcm: %d\n", ret);
			return ret;
		}
		iis2->has_dma = true;
	} else {
		dev_err(dev, "missing dmas (need peri 13 rx)\n");
		return -EINVAL;
	}

	ret = devm_snd_soc_register_component(dev, &s5l8740_iis2_component,
					      &s5l8740_iis2_dai, 1);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_regs);
	if (ret)
		dev_warn(dev, "regs sysfs: %d\n", ret);

	dev_info(dev,
		 "BCM2078 PCM RX @%pR peri13 FIFO@+0x38 (IIS2; FM/A2DP PCM in)\n",
		 res);
	return 0;
}

static void s5l8740_iis2_remove(struct platform_device *pdev)
{
	struct s5l8740_iis2 *iis2 = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_regs);
	iis2_hw_stop(iis2);
	if (iis2 && iis2->num_clks)
		clk_bulk_disable_unprepare(iis2->num_clks, iis2->clks);
}

static const struct of_device_id s5l8740_iis2_of_match[] = {
	{ .compatible = "apple,s5l8740-bcm2078-pcm" },
	{ .compatible = "apple,s5l8740-iis2" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8740_iis2_of_match);

static struct platform_driver s5l8740_iis2_driver = {
	.probe = s5l8740_iis2_probe,
	.remove = s5l8740_iis2_remove,
	.driver = {
		.name = "s5l8740-iis2",
		.of_match_table = s5l8740_iis2_of_match,
	},
};
module_platform_driver(s5l8740_iis2_driver);

MODULE_DESCRIPTION("S5L8740 BCM2078 PCM capture DAI (IIS2 @0x3D400000, peri 13 RX)");
MODULE_LICENSE("GPL");
MODULE_SOFTDEP("pre: dma_s5l8740_pl080");
