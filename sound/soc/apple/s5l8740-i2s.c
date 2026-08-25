// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 IIS0 (I2S) platform DAI — N31
 * IIS0 @ 0x3CA00000, TX FIFO @ +0x10. Optional PL080 dmaengine PCM.
 */
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define S5L8740_I2S_RATES	(SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)
#define S5L8740_I2S_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE)
#define I2SCLKCON	0x00
#define I2STXCON	0x04
#define I2STXCOM	0x08
#define I2STXFIFO	0x10
#define I2SRXCON	0x30
#define I2SRXCOM	0x34
#define I2SSTATUS	0x3c
#define I2SCLKDIV	0x40	/* OSOS 4F716: *(base+64). Not Rockbox +0x24. */
#define MCLK_ASSUME_HZ	12000000u
/* BCB60 a3!=0 a5!=24: 1048728|50331649 = 0x100098|0x03000001. Not 0x03100219. */
#define I2STXCON_N31_16	0x03100099u
#define I2SRXCON_N31	0x1000u
/* OSOS enable ORs 0x100218 (bit20). Live: that bit holds STATUS at
 * 0x24 (no external clock). Clearing it moves STATUS to 0x8020.
 * Override via txcon= for bring-up; default stays OSOS. */
static uint txcon = I2STXCON_N31_16;
module_param(txcon, uint, 0644);
MODULE_PARM_DESC(txcon, "I2STXCON (default 0x03100099 BCB60 16-bit)");
/*
 * D34C0 → 4F716(port, div). 48 kHz is 250, or 125 if 892A02C==6000.
 * 0 = 12 MHz / rate (250 @ 48 kHz).
 */
static uint clkdiv;
module_param(clkdiv, uint, 0644);
MODULE_PARM_DESC(clkdiv, "I2SCLKDIV override; 0 = 12000000/rate");
/*
 * dma_tone FIFO beat width. Rockbox s5l8702 PCM is 16-bit (WIDTH_16).
 * Default 2 = 16-bit stereo interleaved (Rockbox/OSOS BCB60 16-bit).
 * 4 = packed LR 32-bit beats for pio_tone CPU path.
 */
static int tone_width = 2;
module_param(tone_width, int, 0644);
MODULE_PARM_DESC(tone_width, "dma_tone dst width bytes 2 or 4 (default 2)");
/*
 * OSOS B6620(port,0) does TXCOM |= 6 after PL080 is armed (peri 12).
 * That is a DMA kick. CPU PIO has no DMA: Rockbox-family bit 3 must be
 * set or the serializer never leaves STATUS 0x24. Default 0xC = PIO.
 */
#define I2STXCOM_DMA	0x6
#define I2STXCOM_PIO	0xc
#define I2STXCOM_STOP	0x0
#define CLKCON_PHYS	0x3c500000ul
#define GPIO_PHYS	0x3cf00000ul
#define GPIOCMD_PHYS	0x3cf001e0ul

/*
 * RetailOS user volume is 0..256 (VolumeScalar, 256 = unity). CS42
 * analog 0x527 is only mute/unmute; the integer is a PCM Q8 gain.
 */
#define S5L8740_USER_VOL_MAX	256
static atomic_t s5l8740_user_vol_q8 = ATOMIC_INIT(S5L8740_USER_VOL_MAX);

void s5l8740_set_user_vol_q8(unsigned int vol);
unsigned int s5l8740_get_user_vol_q8(void);

void s5l8740_set_user_vol_q8(unsigned int vol)
{
	if (vol > S5L8740_USER_VOL_MAX)
		vol = S5L8740_USER_VOL_MAX;
	atomic_set(&s5l8740_user_vol_q8, vol);
}
EXPORT_SYMBOL_GPL(s5l8740_set_user_vol_q8);

unsigned int s5l8740_get_user_vol_q8(void)
{
	return atomic_read(&s5l8740_user_vol_q8);
}
EXPORT_SYMBOL_GPL(s5l8740_get_user_vol_q8);

static s16 s5l8740_scale_s16(s16 s)
{
	unsigned int q8 = atomic_read(&s5l8740_user_vol_q8);

	if (q8 >= S5L8740_USER_VOL_MAX)
		return s;
	return (s16)(((int)s * (int)q8) / S5L8740_USER_VOL_MAX);
}

static u32 s5l8740_scale_lr(u32 sample)
{
	unsigned int q8 = atomic_read(&s5l8740_user_vol_q8);
	s16 l, r;

	if (q8 >= S5L8740_USER_VOL_MAX)
		return sample;
	l = s5l8740_scale_s16((s16)sample);
	r = s5l8740_scale_s16((s16)(sample >> 16));
	return ((u32)(u16)r << 16) | (u16)l;
}

static int use_pio;
module_param(use_pio, int, 0644);
MODULE_PARM_DESC(use_pio, "1 = CPU FIFO PCM; 0 = OSOS PL080 M2P peri 12 (default)");

static int txcom_pio = I2STXCOM_PIO;
module_param(txcom_pio, int, 0644);
MODULE_PARM_DESC(txcom_pio, "TXCOM when use_pio=1 (default 0xC; OSOS DMA is 0x6)");

/* BCB60 sets DIR. Live pad_oe=0: GPIO 7/20 stop, GPIO 6 still
 * toggles — BCLK/LRCK are SoC-driven, not codec-master. */
static int pad_oe = 1;
module_param(pad_oe, int, 0644);
MODULE_PARM_DESC(pad_oe, "1 = OSOS DIR out (default); 0 = mode 3, DIR in");

struct s5l8740_i2s {
	void __iomem *base;
	void __iomem *clkcon;
	void __iomem *gpio;
	void __iomem *gpiocmd;
	struct device *dev;
	struct clk_bulk_data *clks;
	int num_clks;
	bool has_dma;
	struct dma_chan *tx_chan;	/* cached — avoid dma:tx symlink churn */
	struct mutex dma_lock;
	struct snd_dmaengine_dai_dma_data play_dma;
	struct snd_pcm_substream *ss;
	struct task_struct *kthread;
	bool pio_run;
	unsigned int pio_hw_ptr;
	unsigned int rate;
};

/*
 * SEC sub_2034 leftovers. OSOS 983430 never programs clock 9;
 * it does program clocks 6/20 into +0x1C after SEC. If U-Boot
 * zeroed the pair, IIS has no parent. Do not write +00/+04/+44.
 */
#define SEC_CLKCON_18	0x20012001u
#define SEC_CLKCON_1C	0x10122003u

/* sub_41CBD8(9,1): CLKCON+0x0C bit 15 clear = IIS0 CG16 on. */
static void s5l8740_i2s_ungate(struct s5l8740_i2s *i2s)
{
	u32 v, r18, r1c;

	if (!i2s->clkcon)
		return;
	r18 = readl(i2s->clkcon + 0x18);
	r1c = readl(i2s->clkcon + 0x1c);
	if (!r18)
		writel(SEC_CLKCON_18, i2s->clkcon + 0x18);
	if (!r1c)
		writel(SEC_CLKCON_1C, i2s->clkcon + 0x1c);
	v = readl(i2s->clkcon + 0x0c);
	if (v & 0x8000u)
		writel(v & ~0x8000u, i2s->clkcon + 0x0c);
}

/* sub_43D38C(7,3) and (20,3) — IIS0 pads only (do not touch 0x0A061010 / GPIO86). */
static void s5l8740_i2s_pads(struct s5l8740_i2s *i2s)
{
	static const u8 gpios[] = { 7, 20 };
	unsigned int i;

	if (!i2s->gpio || !i2s->gpiocmd)
		return;
	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		unsigned int gpio = gpios[i];
		unsigned int bank = gpio >> 3;
		unsigned int pin = gpio & 7;
		void __iomem *b = i2s->gpio + 32 * bank;
		u32 dir = readl(b + 0x14);

		if (pad_oe)
			writel(dir | BIT(pin), b + 0x14);
		else
			writel(dir & ~BIT(pin), b + 0x14);
		writel((bank << 16) | (pin << 8) | 3, i2s->gpiocmd);
	}
}

static void s5l8740_i2s_c09ac_start(struct s5l8740_i2s *i2s);
static void s5l8740_i2s_program(struct s5l8740_i2s *i2s, unsigned int rate);
static void s5l8740_i2s_tx_kick(struct s5l8740_i2s *i2s, bool dma);

static int fifo_wait_loops = 50;
module_param(fifo_wait_loops, int, 0644);
MODULE_PARM_DESC(fifo_wait_loops, "max polls for IIS TX FIFO ready before write");

static int s5l8740_i2s_codec_prepare(void)
{
	int (*prep)(void) = __symbol_get("cs42l81_play_prepare");
	int ret = 0;

	if (prep) {
		ret = prep();
		__symbol_put("cs42l81_play_prepare");
	}
	return ret;
}

static int s5l8740_i2s_asp_lock(void)
{
	int (*asp)(void) = __symbol_get("cs42l81_post_iis_start");
	int ret = -ENOENT;

	if (asp) {
		msleep(20);
		ret = asp();
		__symbol_put("cs42l81_post_iis_start");
	}
	return ret;
}

static void s5l8740_i2s_fifo_write(struct s5l8740_i2s *i2s, s16 s)
{
	unsigned int n;
	u32 status;

	for (n = 0; n < fifo_wait_loops; n++) {
		status = readl(i2s->base + I2SSTATUS);
		if (!(status & 0x20))
			break;
		cpu_relax();
	}
	writel((u32)(u16)s | ((u32)(u16)s << 16), i2s->base + I2STXFIFO);
}

static int s5l8740_i2s_play_start(struct s5l8740_i2s *i2s, bool dma)
{
	int ret;

	ret = s5l8740_i2s_codec_prepare();
	if (ret && i2s->dev)
		dev_warn(i2s->dev, "codec prepare: %d\n", ret);
	s5l8740_i2s_program(i2s, 48000);
	s5l8740_i2s_tx_kick(i2s, dma);
	ret = s5l8740_i2s_asp_lock();
	if (i2s->dev)
		dev_info(i2s->dev, "play_start dma=%d asp=%d status=0x%x txcom=0x%x\n",
			 dma, ret, readl(i2s->base + I2SSTATUS),
			 readl(i2s->base + I2STXCOM));
	return ret;
}

/*
 * 345D70 is JUMPOUT 0x22000350 = bootloader sub_350 (SCTLR C-bit).
 * Play 414FAE only starts — it does not C09AC-stop first.
 */
static void s5l8740_i2s_c09ac_start(struct s5l8740_i2s *i2s)
{
	writel(1, i2s->base + I2SCLKCON);
}

/* 26DDDE: 41CBD8(9,1), 5705DC RX, 414FAE (C09AC + BCB60), D34C0 CLKDIV. */
static void s5l8740_i2s_program(struct s5l8740_i2s *i2s, unsigned int rate)
{
	u32 div = clkdiv ? clkdiv : MCLK_ASSUME_HZ / (rate ? rate : 48000);
	u32 rxcom;

	if (div < 1)
		div = 1;
	s5l8740_i2s_ungate(i2s);
	s5l8740_i2s_c09ac_start(i2s);
	s5l8740_i2s_pads(i2s);
	writel(txcon, i2s->base + I2STXCON);
	writel(I2SRXCON_N31, i2s->base + I2SRXCON);
	rxcom = readl(i2s->base + I2SRXCOM);
	writel(rxcom & ~4u, i2s->base + I2SRXCOM);
	writel(div, i2s->base + I2SCLKDIV);
	/* C095E/BB9F8 is not on the 26DDDE play path. Bit15 looks W1C. */
	i2s->rate = rate ? rate : 48000;
}

/*
 * OSOS B6620(port,0): TXCOM |= 6 after PL080 armed. Glass also needs bit 3
 * (PIO path 0xC) or STATUS stays 0x24 / jack silent. Set bit 3 before DMA.
 */
static void s5l8740_i2s_tx_kick(struct s5l8740_i2s *i2s, bool dma)
{
	u32 txcom;

	if (!i2s || !i2s->base)
		return;
	if (dma) {
		txcom = readl(i2s->base + I2STXCOM);
		writel(txcom | I2STXCOM_PIO, i2s->base + I2STXCOM);
		writel(txcom | I2STXCOM_PIO | I2STXCOM_DMA,
		       i2s->base + I2STXCOM);
	} else {
		writel(txcom_pio, i2s->base + I2STXCOM);
	}
}

static int s5l8740_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dai->dev);
	unsigned int rate = params_rate(params);
	u32 div;

	if (!i2s || !i2s->base)
		return -ENODEV;
	s5l8740_i2s_program(i2s, rate);
	div = MCLK_ASSUME_HZ / (i2s->rate ? i2s->rate : 48000);
	dev_info(dai->dev, "IIS hw_params rate=%u clkdiv=%u dma=%d pio=%d txcom=0x%x\n",
		 rate, div, i2s->has_dma, use_pio,
		 use_pio ? txcom_pio : I2STXCOM_DMA);
	return 0;
}

static int s5l8740_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dai->dev);

	if (!i2s || !i2s->base)
		return -ENODEV;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		s5l8740_i2s_tx_kick(i2s, !use_pio);
		i2s->pio_run = use_pio;
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		i2s->pio_run = false;
		writel(I2STXCOM_STOP, i2s->base + I2STXCOM);
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5l8740_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dai->dev);

	if (i2s->has_dma)
		snd_soc_dai_init_dma_data(dai, &i2s->play_dma, NULL);
	return 0;
}

static const struct snd_soc_dai_ops s5l8740_i2s_dai_ops = {
	.probe = s5l8740_i2s_dai_probe,
	.hw_params = s5l8740_i2s_hw_params,
	.trigger = s5l8740_i2s_trigger,
};

static struct snd_soc_dai_driver s5l8740_i2s_dai = {
	.name = "s5l8740-i2s",
	.playback = {
		.stream_name = "I2S Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = S5L8740_I2S_RATES,
		.formats = S5L8740_I2S_FORMATS,
	},
	.ops = &s5l8740_i2s_dai_ops,
};

static const struct snd_pcm_hardware s5l8740_pio_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = S5L8740_I2S_FORMATS,
	.rates = S5L8740_I2S_RATES,
	.rate_min = 44100,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 65536,
	.period_bytes_min = 256,
	.period_bytes_max = 8192,
	.periods_min = 2,
	.periods_max = 16,
};

static int s5l8740_pio_thread(void *data)
{
	struct s5l8740_i2s *i2s = data;

	while (!kthread_should_stop()) {
		struct snd_pcm_substream *ss = i2s->ss;
		struct snd_pcm_runtime *rt;
		unsigned int pos, rate, burst, i;
		u32 sample;

		if (!READ_ONCE(i2s->pio_run) || !ss) {
			usleep_range(2000, 4000);
			continue;
		}
		rt = ss->runtime;
		if (!rt || !rt->dma_area) {
			usleep_range(2000, 4000);
			continue;
		}
		/*
		 * Pace with udelay — usleep_range(167us) rounds to a jiffy
		 * (~10 ms) and a 3s tone hung for a minute. Burst ~4 ms of
		 * realtime writes, then cond_resched so RNDIS still runs.
		 */
		rate = i2s->rate ? i2s->rate : 48000;
		burst = rate / 250; /* ~4 ms */
		if (burst < 16)
			burst = 16;
		pos = i2s->pio_hw_ptr;
		for (i = 0; i < burst && READ_ONCE(i2s->pio_run); i++) {
			if (pos >= rt->buffer_size)
				pos = 0;
			sample = s5l8740_scale_lr(*(u32 *)(rt->dma_area +
					  frames_to_bytes(rt, pos)));
			writel(sample, i2s->base + I2STXFIFO);
			pos++;
			if (pos >= rt->buffer_size)
				pos = 0;
			if (rt->period_size && (pos % rt->period_size) == 0)
				snd_pcm_period_elapsed(ss);
			udelay(1000000 / rate);
		}
		i2s->pio_hw_ptr = pos;
		cond_resched();
	}
	return 0;
}

static int s5l8740_pio_open(struct snd_soc_component *comp,
			    struct snd_pcm_substream *ss)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(comp->dev);

	if (!use_pio)
		return 0;
	snd_soc_set_runtime_hwparams(ss, &s5l8740_pio_hw);
	i2s->ss = ss;
	i2s->pio_hw_ptr = 0;
	if (!i2s->kthread) {
		i2s->kthread = kthread_run(s5l8740_pio_thread, i2s,
					   "n31-i2s-pio");
		if (IS_ERR(i2s->kthread)) {
			int ret = PTR_ERR(i2s->kthread);

			i2s->kthread = NULL;
			return ret;
		}
	}
	return 0;
}

static int s5l8740_pio_close(struct snd_soc_component *comp,
			     struct snd_pcm_substream *ss)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(comp->dev);

	i2s->pio_run = false;
	i2s->ss = NULL;
	return 0;
}

static snd_pcm_uframes_t s5l8740_pio_pointer(struct snd_soc_component *comp,
					     struct snd_pcm_substream *ss)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(comp->dev);

	return i2s->pio_hw_ptr;
}

static int s5l8740_pio_pcm_new(struct snd_soc_component *comp,
			       struct snd_soc_pcm_runtime *rtd)
{
	if (!use_pio)
		return 0;
	return snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC,
					      NULL, 64 * 1024, 64 * 1024);
}

static const struct snd_soc_component_driver s5l8740_i2s_component = {
	.name = "s5l8740-i2s",
	.legacy_dai_naming = 1,
	.open = s5l8740_pio_open,
	.close = s5l8740_pio_close,
	.pointer = s5l8740_pio_pointer,
	.pcm_construct = s5l8740_pio_pcm_new,
};

/* DMA path: dmaengine_pcm owns PCM ops. Do not install pointer(). */
static const struct snd_soc_component_driver s5l8740_i2s_dai_component = {
	.name = "s5l8740-i2s",
	.legacy_dai_naming = 1,
};

static ssize_t regs_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dev);
	static const u32 offs[] = {
		I2SCLKCON, I2STXCON, I2STXCOM, I2STXFIFO,
		I2SRXCON, I2SRXCOM, I2SSTATUS, I2SCLKDIV,
	};
	int i, n = 0;

	if (!i2s || !i2s->base)
		return -ENODEV;
	for (i = 0; i < ARRAY_SIZE(offs); i++)
		n += scnprintf(buf + n, PAGE_SIZE - n, "+0x%02x: 0x%08x\n",
			       offs[i], readl(i2s->base + offs[i]));
	if (i2s->clkcon) {
		static const u32 clk_offs[] = {
			0x00, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
			0x44, 0x48, 0x4c, 0x58, 0x68, 0x6c,
		};
		int c;

		for (c = 0; c < ARRAY_SIZE(clk_offs); c++)
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       "clk+0x%02x: 0x%08x\n", clk_offs[c],
				       readl(i2s->clkcon + clk_offs[c]));
	}
	if (i2s->gpio) {
		u32 p0 = readl(i2s->gpio);
		u32 d0 = readl(i2s->gpio + 0x04);
		u32 p2 = readl(i2s->gpio + 64);
		u32 d2 = readl(i2s->gpio + 68);

		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "pcon0=%08x din0=%08x pcon2=%08x din2=%08x\n",
			       p0, d0, p2, d2);
	}
	return n;
}
static DEVICE_ATTR_RO(regs);

static ssize_t volume_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned int vol;

	if (kstrtouint(buf, 0, &vol) || vol > S5L8740_USER_VOL_MAX)
		return -EINVAL;
	s5l8740_set_user_vol_q8(vol);
	return count;
}

static ssize_t volume_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sysfs_emit(buf, "%u/%u (RetailOS Q8, 256=unity)\n",
			  s5l8740_get_user_vol_q8(), S5L8740_USER_VOL_MAX);
}
static DEVICE_ATTR_RW(volume);

/* 1 kHz sine @ 48 kHz, 48 samples/period, peak 32767. No FP in the loop. */
static const s16 sine_1khz_48k[48] = {
	0, 4277, 8481, 12540, 16384, 19948, 23170, 25997,
	28378, 30274, 31651, 32487, 32767, 32487, 31651, 30274,
	28378, 25997, 23170, 19948, 16384, 12540, 8481, 4277,
	0, -4277, -8481, -12540, -16384, -19948, -23170, -25997,
	-28378, -30274, -31651, -32487, -32767, -32487, -31651, -30274,
	-28378, -25997, -23170, -19948, -16384, -12540, -8481, -4277,
};

/* CPU-paced FIFO write. TXCOM 6 = OSOS B6620 TX start. */
static ssize_t pio_tone_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dev);
	unsigned int frames, i;
	s16 s;

	if (!i2s || !i2s->base)
		return -ENODEV;
	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;

	s5l8740_i2s_play_start(i2s, false);

	frames = 48000 * 2; /* ~2 s */
	for (i = 0; i < frames; i++) {
		s = s5l8740_scale_s16(sine_1khz_48k[i % 48]);
		s5l8740_i2s_fifo_write(i2s, s);
		udelay(20);
	}
	writel(I2STXCOM_STOP, i2s->base + I2STXCOM);
	dev_info(dev, "pio_tone 2s done status=0x%08x\n",
		 readl(i2s->base + I2SSTATUS));
	return count;
}
static DEVICE_ATTR_WO(pio_tone);

struct dma_chan *s5l_pl080_request_slave(struct device *consumer,
					 unsigned int idx);

static struct dma_chan *s5l8740_i2s_tx_get(struct s5l8740_i2s *i2s,
					   struct device *dev)
{
	if (i2s->tx_chan)
		return i2s->tx_chan;
	i2s->tx_chan = s5l_pl080_request_slave(dev, 0);
	return i2s->tx_chan;
}

static void s5l8740_i2s_tx_put(struct s5l8740_i2s *i2s)
{
	if (!i2s || !i2s->tx_chan)
		return;
	dma_release_channel(i2s->tx_chan);
	i2s->tx_chan = NULL;
}

/* One-shot OSOS path: PL080 M2P -> +0x10, then TXCOM=0xE. */
static ssize_t dma_tone_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dev);
	struct dma_chan *chan;
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config cfg = { };
	dma_cookie_t cookie;
	dma_addr_t dma;
	s16 *tone;
	size_t bytes = 48000 * 2 * 2 * 2; /* 2 s stereo S16 */
	unsigned int i;
	s16 s;
	int ret;

	if (!i2s || !i2s->base)
		return -ENODEV;
	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;

	mutex_lock(&i2s->dma_lock);
	chan = s5l8740_i2s_tx_get(i2s, dev);
	if (IS_ERR(chan)) {
		ret = PTR_ERR(chan);
		dev_err(dev, "dma_tone request tx: %d\n", ret);
		mutex_unlock(&i2s->dma_lock);
		return ret;
	}

	tone = dma_alloc_coherent(dev, bytes, &dma, GFP_KERNEL);
	if (!tone) {
		ret = -ENOMEM;
		goto out_unlock;
	}
	for (i = 0; i < bytes / 4; i++) {
		s = s5l8740_scale_s16(sine_1khz_48k[i % 48]);
		tone[i * 2] = s;
		tone[i * 2 + 1] = s;
	}
	dma_sync_single_for_device(dev, dma, bytes, DMA_TO_DEVICE);

	cfg.direction = DMA_MEM_TO_DEV;
	cfg.dst_addr = i2s->play_dma.addr;
	if (tone_width == 2)
		cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	else
		cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_maxburst = 1;
	ret = dmaengine_slave_config(chan, &cfg);
	if (ret) {
		dev_err(dev, "dma_tone slave_config: %d\n", ret);
		goto out_chan;
	}

	s5l8740_i2s_codec_prepare();
	s5l8740_i2s_program(i2s, 48000);
	desc = dmaengine_prep_slave_single(chan, dma, bytes, DMA_MEM_TO_DEV,
					   DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dev, "dma_tone prep_slave_single failed\n");
		ret = -EIO;
		goto out_chan;
	}
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		ret = cookie;
		goto out_chan;
	}
	dma_async_issue_pending(chan);
	s5l8740_i2s_tx_kick(i2s, true);
	{
		int asp = s5l8740_i2s_asp_lock();

		dev_info(dev, "dma_tone asp_lock=%d\n", asp);
	}
	{
		void __iomem *pl = ioremap(0x38200000ul, 0x200);
		unsigned int t, i;

		if (i2s->gpio) {
			u32 xor[8] = { }, last[8] = { }, pcon[8] = { };
			unsigned int b;

			for (b = 0; b < 8; b++) {
				pcon[b] = readl(i2s->gpio + 32 * b);
				last[b] = readl(i2s->gpio + 32 * b + 4);
			}
			for (i = 0; i < 20000; i++) {
				for (b = 0; b < 8; b++) {
					u32 d = readl(i2s->gpio + 32 * b + 4);

					xor[b] |= d ^ last[b];
					last[b] = d;
				}
			}
			dev_info(dev,
				 "dma_tone pads xor %02x %02x %02x %02x %02x %02x %02x %02x\n",
				 xor[0], xor[1], xor[2], xor[3],
				 xor[4], xor[5], xor[6], xor[7]);
			dev_info(dev,
				 "dma_tone pcon %08x %08x %08x %08x\n",
				 pcon[0], pcon[1], pcon[2], pcon[3]);
		}

		if (pl) {
			for (t = 0; t < 3; t++) {
				u32 en = readl(pl + 0x1c);
				u32 st = readl(i2s->base + I2SSTATUS);
				int ch;

				dev_info(dev,
					 "dma_tone t=%ums status=0x%x txcom=0x%x en=0x%x rawtc=0x%x\n",
					 t * 100, st,
					 readl(i2s->base + I2STXCOM), en,
					 readl(pl + 0x14));
				for (ch = 0; ch < 8; ch++) {
					u32 dst = readl(pl + 0x104 + ch * 0x20);
					u32 src = readl(pl + 0x100 + ch * 0x20);
					u32 cfg = readl(pl + 0x110 + ch * 0x20);
					u32 c2 = readl(pl + 0x114 + ch * 0x20);

					if (!(en & BIT(ch)) && dst != 0x3ca00010)
						continue;
					dev_info(dev,
						 "  ch%u src=0x%x dst=0x%x cfg=0x%x c2=0x%x\n",
						 ch, src, dst, cfg, c2);
				}
				if (t == 0)
					msleep(100);
				else if (t == 1)
					msleep(1900);
			}
			iounmap(pl);
		}
	}
	dev_info(dev, "dma_tone 1kHz 2s status=0x%x txcom=0x%x\n",
		 readl(i2s->base + I2SSTATUS),
		 readl(i2s->base + I2STXCOM));
	dmaengine_terminate_sync(chan);
	writel(I2STXCOM_STOP, i2s->base + I2STXCOM);
	ret = 0;
out_chan:
	dma_free_coherent(dev, bytes, tone, dma);
out_unlock:
	mutex_unlock(&i2s->dma_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(dma_tone);

/* Sample GPIO DIN xor across banks 0-7. Use after clk_run or at idle. */
static ssize_t pad_scan_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dev);
	u32 xor[8] = { }, pcon[8] = { }, last[8] = { };
	unsigned int b, i, n = 0;

	if (!i2s || !i2s->gpio)
		return -ENODEV;
	for (b = 0; b < 8; b++) {
		pcon[b] = readl(i2s->gpio + 32 * b);
		last[b] = readl(i2s->gpio + 32 * b + 4);
	}
	for (i = 0; i < 40000; i++) {
		for (b = 0; b < 8; b++) {
			u32 d = readl(i2s->gpio + 32 * b + 4);

			xor[b] |= d ^ last[b];
			last[b] = d;
		}
	}
	n += scnprintf(buf + n, PAGE_SIZE - n,
		       "clkcon=0x%x txcon=0x%x txcom=0x%x status=0x%x\n",
		       readl(i2s->base + I2SCLKCON),
		       readl(i2s->base + I2STXCON),
		       readl(i2s->base + I2STXCOM),
		       readl(i2s->base + I2SSTATUS));
	for (b = 0; b < 8; b++)
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "b%u pcon=%08x xor=%02x\n", b, pcon[b], xor[b]);
	return n;
}
static DEVICE_ATTR_RO(pad_scan);

/* Program IIS and leave TXCOM running so BCLK/LRCK (and MCLK if any) stay up. */
static ssize_t clk_run_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dev);
	unsigned int v;

	if (!i2s || !i2s->base)
		return -ENODEV;
	if (kstrtouint(buf, 0, &v))
		return -EINVAL;
	if (v) {
		s5l8740_i2s_program(i2s, 48000);
		s5l8740_i2s_tx_kick(i2s, false);
	} else {
		writel(I2STXCOM_STOP, i2s->base + I2STXCOM);
	}
	dev_info(dev, "clk_run=%u status=0x%x txcom=0x%x\n",
		 v, readl(i2s->base + I2SSTATUS),
		 readl(i2s->base + I2STXCOM));
	return count;
}
static DEVICE_ATTR_WO(clk_run);

static int s5l8740_i2s_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l8740_i2s *i2s;
	struct resource *res;
	int ret;

	i2s = devm_kzalloc(dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;
	i2s->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2s->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(i2s->base))
		return PTR_ERR(i2s->base);
	i2s->clkcon = devm_ioremap(dev, CLKCON_PHYS, 0x80);
	i2s->gpio = devm_ioremap(dev, GPIO_PHYS, 0x200);
	i2s->gpiocmd = devm_ioremap(dev, GPIOCMD_PHYS, 4);

	ret = devm_clk_bulk_get_all(dev, &i2s->clks);
	if (ret > 0) {
		i2s->num_clks = ret;
		ret = clk_bulk_prepare_enable(i2s->num_clks, i2s->clks);
		if (ret)
			dev_warn(dev, "clk_bulk_prepare_enable: %d (CLKCON ungate-all still on)\n",
				 ret);
	}

	if (res) {
		i2s->play_dma.addr = res->start + I2STXFIFO;
		i2s->play_dma.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		i2s->play_dma.maxburst = 1;
	}

	platform_set_drvdata(pdev, i2s);
	dev_set_drvdata(dev, i2s);
	mutex_init(&i2s->dma_lock);

	if (!use_pio && of_property_present(dev->of_node, "dmas")) {
		ret = devm_snd_dmaengine_pcm_register(dev, NULL, 0);
		if (ret) {
			dev_warn(dev, "dmaengine_pcm: %d — falling back to PIO\n",
				 ret);
			use_pio = 1;
		} else {
			i2s->has_dma = true;
		}
	} else if (!use_pio) {
		dev_warn(dev, "no dmas in DT — PIO\n");
		use_pio = 1;
	}

	ret = devm_snd_soc_register_component(dev,
					      use_pio ? &s5l8740_i2s_component :
							&s5l8740_i2s_dai_component,
					      &s5l8740_i2s_dai, 1);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_regs);
	if (ret)
		dev_warn(dev, "regs sysfs: %d\n", ret);
	ret = device_create_file(dev, &dev_attr_volume);
	if (ret)
		dev_warn(dev, "volume sysfs: %d\n", ret);
	ret = device_create_file(dev, &dev_attr_pio_tone);
	if (ret)
		dev_warn(dev, "pio_tone sysfs: %d\n", ret);
	ret = device_create_file(dev, &dev_attr_dma_tone);
	if (ret)
		dev_warn(dev, "dma_tone sysfs: %d\n", ret);
	ret = device_create_file(dev, &dev_attr_pad_scan);
	if (ret)
		dev_warn(dev, "pad_scan sysfs: %d\n", ret);
	ret = device_create_file(dev, &dev_attr_clk_run);
	if (ret)
		dev_warn(dev, "clk_run sysfs: %d\n", ret);

	dev_info(dev, "S5L8740 IIS0 @%pR dma=%s pio=%d\n",
		 res, i2s->has_dma ? "yes" : "no", use_pio);
	return 0;
}

static void s5l8740_i2s_remove(struct platform_device *pdev)
{
	struct s5l8740_i2s *i2s = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_regs);
	device_remove_file(&pdev->dev, &dev_attr_volume);
	device_remove_file(&pdev->dev, &dev_attr_pio_tone);
	device_remove_file(&pdev->dev, &dev_attr_dma_tone);
	device_remove_file(&pdev->dev, &dev_attr_pad_scan);
	device_remove_file(&pdev->dev, &dev_attr_clk_run);
	if (i2s && i2s->kthread) {
		i2s->pio_run = false;
		kthread_stop(i2s->kthread);
		i2s->kthread = NULL;
	}
	s5l8740_i2s_tx_put(i2s);
	if (i2s && i2s->num_clks)
		clk_bulk_disable_unprepare(i2s->num_clks, i2s->clks);
}

static const struct of_device_id s5l8740_i2s_of_match[] = {
	{ .compatible = "apple,s5l8740-i2s" },
	{ .compatible = "samsung,s5l8740-i2s" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8740_i2s_of_match);

static struct platform_driver s5l8740_i2s_driver = {
	.probe = s5l8740_i2s_probe,
	.remove = s5l8740_i2s_remove,
	.driver = {
		.name = "s5l8740-i2s",
		.of_match_table = s5l8740_i2s_of_match,
	},
};
module_platform_driver(s5l8740_i2s_driver);

MODULE_DESCRIPTION("S5L8740 IIS0 DAI + optional PL080 PCM (N31)");
MODULE_LICENSE("GPL");
