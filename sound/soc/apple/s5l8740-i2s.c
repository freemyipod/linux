// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 I2S platform DAIs — N31
 *
 * Both of the board's audio ports live here because they share the
 * SoC audio clock gate at CLKCON+0x30, and arbitrating that across two
 * modules is not worth the symbol traffic:
 *
 *   IIS0 @ 0x3CA00000  TX FIFO +0x10, PL080 peri 10 -> CS42L81 headphones
 *   IIS2 @ 0x3D400000  RX FIFO +0x38, PL080 peri 13 <- BCM2078 digital PCM
 *
 * They face different chips, but to userspace they are simply the
 * playback and capture PCMs of one card.
 *
 * IIS2 register program is from the RetailOS fm-playing MMIO capture:
 *   CLKCON +0x00 = 0x1        TXCON  +0x04 = 0x0b000099
 *   RXCON  +0x30 = 0x1000     RXCOM  +0x34 = 0x6 running, 0x2 idle
 *   CLKDIV +0x40 = 0x96       REG44  +0x44 = 0x00010007
 * IIS1 @ 0x3CD00000 is XSP and always reads zero -- it is not a BCM port.
 * A2DP does not appear here at all: it is host-encoded over UART1 HCI.
 */
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/apple-n31.h>

#include "n31-audio-rates.h"

#define S5L8740_I2S_RATES	(SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)
#define S5L8740_I2S_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE)
#define I2SCLKCON	0x00
#define I2STXCON	0x04
#define I2STXCOM	0x08
#define I2STXFIFO	0x10
#define I2SRXCON	0x30
#define I2SRXCOM	0x34
#define I2SRXFIFO	0x38
#define I2SSTATUS	0x3c
#define I2SCLKDIV	0x40	/* OSOS 4F716: *(base+64). Not Rockbox +0x24. */
/* RetailOS music IIS0+0x44 readback 0x00010007 (oracle 2026-08-25). */
#define I2SREG44	0x44
/*
 * OSOS sub_C095E(port, ch): STATUS W1C — TX sticky is bit15 (1<<15).
 * Linux never cleared this; silent dumps always show 0x8xxx.
 */
#define I2SSTATUS_TX_W1C	0x8000u
#define MCLK_ASSUME_HZ	12000000u
/* BCB60 a3!=0 a5!=24: 1048728|50331649 = 0x100098|0x03000001. Not 0x03100219.
 * NEVER leave Rockbox 0x0B100019 in txcon — glass SRC sticks at +0x1a and
 * FIFO never drains (status 0x82a4). */
#define I2STXCON_N31_16	0x03100099u
#define I2SRXCON_N31	0x1000u
/* OSOS enable ORs 0x100218 (bit20). Live: that bit holds STATUS at
 * 0x24 (no external clock). Clearing it moves STATUS to 0x8020.
 * Override via txcon= for bring-up; default stays OSOS. */
static uint txcon = I2STXCON_N31_16;
module_param(txcon, uint, 0644);
MODULE_PARM_DESC(txcon, "I2STXCON (default 0x03100099; NOT Rockbox 0x0B100019)");
/*
 * D34C0 → 4F716(port, div). Table in n31-audio-rates.h.
 * 0 = 12 MHz / rate (272 @ 44.1 kHz RetailOS music).
 */
static uint clkdiv;
module_param(clkdiv, uint, 0644);
MODULE_PARM_DESC(clkdiv, "I2SCLKDIV override; 0 = OSOS table / 12000000/rate");
/*
 * Default IIS program rate for clk_run / dma_tone when no ALSA hw_params.
 * 0 = RetailOS 44100. ALSA playback uses the PCM rate, not this.
 */
static uint default_rate;
module_param(default_rate, uint, 0644);
MODULE_PARM_DESC(default_rate, "clk_run/dma_tone rate; 0 = 44100 OSOS default");
/*
 * dma_tone FIFO beat width. Rockbox s5l8702 PCM is 16-bit (WIDTH_16).
 * Default 2 = 16-bit stereo interleaved (Rockbox/OSOS BCB60 16-bit).
 */
static int tone_width = 2;
module_param(tone_width, int, 0644);
MODULE_PARM_DESC(tone_width, "dma_tone dst width bytes 2 or 4 (default 2)");
/*
 * dma_tone / pio_tone sample rate. 0 = default_rate / OSOS 44100.
 */
static uint tone_rate;
module_param(tone_rate, uint, 0644);
MODULE_PARM_DESC(tone_rate, "dma_tone/pio_tone rate; 0 = OSOS 44100");

/* Keep TX/codec up this long after START even if ALSA xruns. */
static uint sustain_ms = 5000;
module_param(sustain_ms, uint, 0644);
MODULE_PARM_DESC(sustain_ms, "ignore ALSA STOP for this many ms after START (default 5000)");

static uint fifo_prefill = 16;
module_param(fifo_prefill, uint, 0644);
MODULE_PARM_DESC(fifo_prefill,
		 "silent stereo words to push into TX FIFO before TXCOM kick");
/*
 * OSOS B6620(port,0) does TXCOM |= 6 after PL080 is armed (peri 10).
 * RE body: sub_B6620 only ORs 0x6 — not 0xC. Hybrid 0xE was Linux invention.
 */
#define I2STXCOM_DMA	0x6
#define I2STXCOM_PIO	0xc
#define I2STXCOM_STOP	0x0
#define CLKCON_PHYS	0x3c500000ul
/* RetailOS oracle dwords at CLKCON+0x30 (music vs idle/A2DP). */
#define CLKCON_AUDIO_OFF	0x30
#define CLKCON_AUDIO_PLAY	0x32190u
#define CLKCON_AUDIO_IDLE	0x1c20u
/* FM additionally regates CLKCON+0x10; music/idle leaves it at 0x8004. */
#define CLKCON_FM_GATE		0x10
#define CLKCON_FM_GATE_ON	0x4u
#define CLKCON_FM_GATE_IDLE	0x8004u

/* fm-playing oracle values for the IIS2 side. */
#define IIS2_CLKCON_ON		0x1u
#define IIS2_TXCON_FM		0x0b000099u
#define IIS2_RXCON_FM		0x1000u
#define IIS2_RXCOM_DMA		0x6u
#define IIS2_RXCOM_IDLE		0x2u
#define IIS2_CLKDIV_FM_ORACLE	0x96u
#define IIS2_REG44_ORACLE	0x00010007u
#define IIS2_REGS_LEN		0x48

/*
 * IIS2 PCM pads. sub_15DD5C claims these three at function 2 when FM
 * powers on and releases them to input when it powers off, in the same
 * breath as programming device 2 (0x3D400000) and kicking RXCOM -- so
 * they belong to this port, not to the Bluetooth controller. They were
 * previously described as BCM shutdown / device-wakeup / host-wakeup and
 * handed to hci_bcm, which drove the capture bus as GPIOs.
 *
 * Claimed alongside the register program rather than from FM power-on,
 * so opening the capture PCM works regardless of who owns the tuner.
 */
#define IIS2_PAD_BCLK		97	/* 0x61 */
#define IIS2_PAD_SYNC		98	/* 0x62 */
#define IIS2_PAD_DATA		119	/* 0x77 */
#define IIS2_PAD_FUNC		2
#define IIS2_PAD_RELEASE	0

/* Ports sharing the CLKCON+0x30 gate. */
#define S5L8740_AUDIO_PORT_IIS0	0
#define S5L8740_AUDIO_PORT_IIS2	1
#define S5L8740_AUDIO_PORTS	2
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
MODULE_PARM_DESC(use_pio, "1 = CPU FIFO PCM; 0 = PL080 M2P peri 10 from DT (default)");

static int txcom_pio = I2STXCOM_PIO;
module_param(txcom_pio, int, 0644);
MODULE_PARM_DESC(txcom_pio, "TXCOM when use_pio=1 (default 0xC; OSOS DMA is 0x6)");

/*
 * TXCOM kick mode (checkpoint-003 / handoff P0.3):
 *   0 = retail: TXCOM |= 0x6 after DMA armed
 *   1 = pio:    TXCOM = txcom_pio (0xC)
 *   2 = hybrid: bit3 then |0x6 (glass experimental default)
 */
/*
 * RetailOS music-playing SCSI oracle 2026-08-25: TXCOM readback = 0x6.
 * sub_B6620 does |= 6 only. Hybrid 0xE was a Linux false lead.
 */
static int txcom_mode;
module_param(txcom_mode, int, 0644);
MODULE_PARM_DESC(txcom_mode, "0=retail |=6 (default), 1=pio 0xC, 2=hybrid |C|6");

/*
 * txcom_exact: when >=0, tx_kick writes this value exactly (no OR). -1=use mode.
 * Isolation tests: 0x6 / 0xC / 0xE from TXCOM=0 baseline.
 */
static int txcom_exact = -1;
module_param(txcom_exact, int, 0644);
MODULE_PARM_DESC(txcom_exact, "Exact TXCOM write when >=0; -1=txcom_mode (default)");

/* RetailOS local music CLKDIV=0x110 (272) ≈ 12 MHz / 44100. */

/* BCB60 sets DIR. Live pad_oe=0: GPIO 7/20 stop, GPIO 6 still
 * toggles — BCLK/LRCK are SoC-driven, not codec-master. */
static int pad_oe = 1;
module_param(pad_oe, int, 0644);
MODULE_PARM_DESC(pad_oe, "1 = OSOS DIR out (default); 0 = mode 3, DIR in");

/*
 * Pad bring-up variants (exhaust RE before RetailOS GPIO oracle):
 *   0 = local 43D38C(7,3)(20,3) only [BCB60 — CONFIRMED OSOS body]
 *   1 = +43D38C(6,3) — NO OSOS 43D38C call site for GPIO6; debug only
 *   2 = gpio-s5l8740 s5l8740_iis0_pads_enable(3) — SEC pinmux + GPIOCMD
 *   3 = gpio driver mode 2 (BCB60 teardown)
 *   4 = SEC pinmux 6/7/20 + local mode3 on all three
 *   5 = mode2 + SEC pinmux refresh (func2 only, no mode3)
 */
static int pad_mode;
module_param(pad_mode, int, 0644);
MODULE_PARM_DESC(pad_mode, "0=7/20 local; 1=+gpio6; 2=gpio drv; 3=mode2; 4=SEC+6/7/20; 5=SEC func2 only");

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
	u8 tx_chan_borrowed;		/* 1 = lookup_peri, do not dma_release */
	struct mutex dma_lock;
	struct snd_dmaengine_dai_dma_data play_dma;
	struct snd_pcm_substream *ss;
	struct task_struct *kthread;
	bool pio_run;
	unsigned int pio_hw_ptr;
	unsigned int rate;
	struct delayed_work dma_watch;
	unsigned long play_jiffies;
	u32 last_dma_src;
	u8 watch_ticks;
};

/*
 * SEC sub_2034 leftovers. OSOS 983430 never programs clock 9;
 * it does program clocks 6/20 into +0x1C after SEC. If U-Boot
 * zeroed the pair, IIS has no parent. Do not write +00/+04/+44.
 *
 * RetailOS music-playing oracle (checkpoint-010): +0x1C = 0xD0052003.
 * Leaving the SEC bring-up value 0x10122003 yields IIS STATUS 0x82A0
 * and a silent jack even with TXCOM=6 / CS42 unmuted. Always force
 * the stock music parent when force_stock_audio_parent=1 (default).
 */
#define SEC_CLKCON_18		0x20012001u
#define SEC_CLKCON_1C		0x10122003u
/* artifacts/retailos-mmio/music-playing/CLKCON.bin — checkpoint-010 */
#define STOCK_CLKCON_08		0xa009200au
#define STOCK_CLKCON_0C		0x80000001u
#define STOCK_CLKCON_10		0x00008000u
#define STOCK_CLKCON_14		0x80002200u
#define STOCK_CLKCON_18		0x20012001u
#define STOCK_CLKCON_1C		0xD0052003u

static bool force_stock_audio_parent = true;
module_param(force_stock_audio_parent, bool, 0644);
MODULE_PARM_DESC(force_stock_audio_parent,
		 "1=force CLKCON+0x08..0x1C to RetailOS music-playing snapshot");

/* sub_41CBD8(9,1): CLKCON+0x0C bit 15 clear = IIS0 CG16 on. */
static void s5l8740_i2s_ungate(struct s5l8740_i2s *i2s)
{
	u32 v, r18, r1c;

	if (!i2s->clkcon)
		return;
	r18 = readl(i2s->clkcon + 0x18);
	r1c = readl(i2s->clkcon + 0x1c);
	if (force_stock_audio_parent) {
		/*
		 * +0x1C alone left STATUS at 0x82A0. Push the rest of the
		 * music-playing parent snapshot (checkpoint-010 §5.B).
		 */
		writel(STOCK_CLKCON_08, i2s->clkcon + 0x08);
		writel(STOCK_CLKCON_0C, i2s->clkcon + 0x0c);
		writel(STOCK_CLKCON_10, i2s->clkcon + 0x10);
		writel(STOCK_CLKCON_14, i2s->clkcon + 0x14);
		writel(STOCK_CLKCON_18, i2s->clkcon + 0x18);
		writel(STOCK_CLKCON_1C, i2s->clkcon + 0x1c);
	} else {
		if (!r18)
			writel(SEC_CLKCON_18, i2s->clkcon + 0x18);
		if (!r1c)
			writel(SEC_CLKCON_1C, i2s->clkcon + 0x1c);
		v = readl(i2s->clkcon + 0x0c);
		if (v & 0x8000u)
			writel(v & ~0x8000u, i2s->clkcon + 0x0c);
	}
}

/* RetailOS absolute dword — better than sticky play when idle. */
/*
 * CLKCON+0x30 gates the audio clock for IIS0 and IIS2 together. Each port
 * used to write it directly, so stopping FM capture also idled the clock
 * out from under music that was still playing. Track which ports want it
 * running and only idle the gate once nobody does.
 */
static DEFINE_SPINLOCK(s5l8740_audio_clk_lock);
static bool s5l8740_audio_clk_wanted[S5L8740_AUDIO_PORTS];

static void s5l8740_audio_clk_set(void __iomem *clkcon, unsigned int port,
				  bool on)
{
	unsigned long flags;
	unsigned int i;
	bool any = false;

	if (!clkcon)
		return;
	spin_lock_irqsave(&s5l8740_audio_clk_lock, flags);
	s5l8740_audio_clk_wanted[port] = on;
	for (i = 0; i < S5L8740_AUDIO_PORTS; i++)
		any |= s5l8740_audio_clk_wanted[i];
	writel(any ? CLKCON_AUDIO_PLAY : CLKCON_AUDIO_IDLE,
	       clkcon + CLKCON_AUDIO_OFF);
	spin_unlock_irqrestore(&s5l8740_audio_clk_lock, flags);
}

static void s5l8740_i2s_clkcon_audio(struct s5l8740_i2s *i2s, u32 val)
{
	if (!i2s)
		return;
	s5l8740_audio_clk_set(i2s->clkcon, S5L8740_AUDIO_PORT_IIS0,
			      val != CLKCON_AUDIO_IDLE);
}

/*
 * STOP teardown (beats RetailOS sticky TX): TXCOM stop → terminate DMA →
 * clear I2SCLKCON → CLKCON+0x30 idle dword.
 */
static void s5l8740_i2s_hw_stop(struct s5l8740_i2s *i2s,
				struct snd_pcm_substream *substream)
{
	struct dma_chan *chan = NULL;

	if (!i2s || !i2s->base)
		return;

	writel(I2STXCOM_STOP, i2s->base + I2STXCOM);

	if (substream && i2s->has_dma && !use_pio)
		chan = snd_dmaengine_pcm_get_chan(substream);
	if (!chan)
		chan = i2s->tx_chan;
	if (chan)
		dmaengine_terminate_sync(chan);

	writel(0, i2s->base + I2SCLKCON);
	s5l8740_i2s_clkcon_audio(i2s, CLKCON_AUDIO_IDLE);
}

/* Packed pinmux word — same as gpio-s5l8740 sub_223C / sub_47CC. */
static void s5l8740_i2s_pinmux_word(struct s5l8740_i2s *i2s, u32 word)
{
	unsigned int bank = (word >> 24) & 0xff;
	unsigned int pin = (word >> 16) & 0xff;
	void __iomem *base;
	u32 v;

	if (!i2s->gpio)
		return;
	base = i2s->gpio + 32u * bank;
	v = readl(base + 0x00);
	writel(((word & 0xfu) << (4u * pin)) | (v & ~(15u << (4u * pin))),
	       base + 0x00);
	v = readl(base + 0x14);
	writel((((word >> 12) & 1u) << pin) | (v & ~BIT(pin)), base + 0x14);
	v = readl(base + 0x0c);
	writel((((word >> 4) & 1u) << pin) | (v & ~BIT(pin)), base + 0x0c);
	v = readl(base + 0x10);
	writel((((word >> 8) & 1u) << pin) | (v & ~BIT(pin)), base + 0x10);
}

static void s5l8740_i2s_gpiocmd(struct s5l8740_i2s *i2s, unsigned int gpio, u8 cmd)
{
	unsigned int bank = gpio >> 3;
	unsigned int pin = gpio & 7;
	void __iomem *b;
	u32 dir;

	if (!i2s->gpio || !i2s->gpiocmd)
		return;
	b = i2s->gpio + 32 * bank;
	dir = readl(b + 0x14);
	writel(dir | BIT(pin), b + 0x14);
	writel((bank << 16) | (pin << 8) | cmd, i2s->gpiocmd);
}

static void s5l8740_i2s_log_iis_gpio(struct s5l8740_i2s *i2s, const char *tag)
{
	void (*logpads)(const char *);

	logpads = (void (*)(const char *))__symbol_get("s5l8740_gpio_log_iis0_pads");
	if (logpads) {
		logpads(tag);
		__symbol_put("s5l8740_gpio_log_iis0_pads");
	} else if (i2s->dev) {
		dev_info(i2s->dev, "%s pads (no gpio export)\n", tag);
	}
}

/*
 * GPIO 4/5 are deliberately left alone. Two stock paths claim them and it
 * is not settled which applies here: sub_71B8 drives them as a two-bit
 * output mux, while the per-bus I2C pinmux helper puts them at function 2
 * as a SCL/SDA pair. i2c0 is the Tristar bus and it times out on glass,
 * so the I2C reading is the more likely one and forcing them to outputs
 * would make that permanent. Nothing in the audio path needs them.
 */

/*
 * sub_BCB60 muxes both IIS0 pads together on every TX enable:
 * sub_43D38C(0x14, 3) and sub_43D38C(7, 3), and puts them back to
 * function 2 on disable. GPIO 7 is an I2S pad, not a display pad -- the
 * panel is driven entirely from the LCDIF and no display code here
 * touches GPIO at all. Claiming only GPIO 20 leaves the bus incomplete
 * and the jack silent. Optional (6,3) in pad_mode 1/4.
 */
static void s5l8740_i2s_pads(struct s5l8740_i2s *i2s)
{
	static const u8 sec_words[] = { 6, 7, 20 };
	unsigned int i;

	if (pad_mode == 2) {
		void (*en)(unsigned int);

		en = (void (*)(unsigned int))__symbol_get("s5l8740_iis0_pads_enable");
		if (en) {
			en(3);
			__symbol_put("s5l8740_iis0_pads_enable");
		}
		s5l8740_i2s_log_iis_gpio(i2s, "pads-mode2-gpio");
		return;
	}

	if (pad_mode == 3) {
		void (*en)(unsigned int);

		en = (void (*)(unsigned int))__symbol_get("s5l8740_iis0_pads_enable");
		if (en) {
			en(2);
			__symbol_put("s5l8740_iis0_pads_enable");
		}
		s5l8740_i2s_log_iis_gpio(i2s, "pads-mode3-off");
		return;
	}

	if (pad_mode >= 4) {
		s5l8740_i2s_pinmux_word(i2s, 0x00061002u);
		s5l8740_i2s_pinmux_word(i2s, 0x00071002u);
		s5l8740_i2s_pinmux_word(i2s, 0x02041002u);
	}

	if (pad_mode == 5) {
		s5l8740_i2s_log_iis_gpio(i2s, "pads-mode5-sec-only");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(sec_words); i++) {
		unsigned int g = sec_words[i];

		if (g == 6 && pad_mode != 1 && pad_mode != 4)
			continue;
		if (pad_oe || g == 7 || g == 20)
			s5l8740_i2s_gpiocmd(i2s, g, 3);
	}

	s5l8740_i2s_log_iis_gpio(i2s, "pads-applied");
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

static int s5l8740_i2s_codec_play_start(void)
{
	int (*start)(void) = __symbol_get("cs42l81_play_start");
	int ret = 0;

	if (start) {
		ret = start();
		__symbol_put("cs42l81_play_start");
	}
	return ret;
}

static void s5l8740_i2s_codec_play_stop(void)
{
	void (*stop)(void) = __symbol_get("cs42l81_play_stop");

	if (stop) {
		stop();
		__symbol_put("cs42l81_play_stop");
	}
}

static int s5l8740_i2s_audio_path_mode(void)
{
	int (*mode)(void) = __symbol_get("cs42l81_get_audio_path_mode");
	int m = 1;

	if (mode) {
		m = mode();
		__symbol_put("cs42l81_get_audio_path_mode");
	}
	return m;
}

static int __maybe_unused s5l8740_i2s_asp_lock(void)
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

static int __maybe_unused s5l8740_i2s_asp_hold_light(void)
{
	int (*hold)(void) = __symbol_get("cs42l81_asp_hold_light");
	int ret = -ENOENT;

	if (hold) {
		ret = hold();
		__symbol_put("cs42l81_asp_hold_light");
	}
	return ret;
}

static void s5l8740_i2s_log_clocks(struct s5l8740_i2s *i2s, const char *tag)
{
	u32 c0c = 0, c18 = 0, c1c = 0;
	u32 clkcon = 0, txcon = 0, txcom = 0, rxcon = 0, rxcom = 0;
	u32 status = 0, clkdiv = 0;

	if (!i2s || !i2s->dev)
		return;
	if (i2s->clkcon) {
		c0c = readl(i2s->clkcon + 0x0c);
		c18 = readl(i2s->clkcon + 0x18);
		c1c = readl(i2s->clkcon + 0x1c);
	}
	if (i2s->base) {
		clkcon = readl(i2s->base + I2SCLKCON);
		txcon = readl(i2s->base + I2STXCON);
		txcom = readl(i2s->base + I2STXCOM);
		rxcon = readl(i2s->base + I2SRXCON);
		rxcom = readl(i2s->base + I2SRXCOM);
		status = readl(i2s->base + I2SSTATUS);
		clkdiv = readl(i2s->base + I2SCLKDIV);
	}
	dev_info(i2s->dev,
		 "%s CLKCON+0C=%08x +18=%08x +1C=%08x IIS0 +00=%08x +04=%08x +08=%08x +30=%08x +34=%08x +3C=%08x +40=%08x\n",
		 tag, c0c, c18, c1c, clkcon, txcon, txcom, rxcon, rxcom, status,
		 clkdiv);
}

static void s5l8740_i2s_pre_codec(void)
{
	void (*pre)(void);

	pre = (void (*)(void))__symbol_get("cs42l81_pre_iis_start");
	if (pre) {
		pre();
		__symbol_put("cs42l81_pre_iis_start");
	}
}

static void s5l8740_i2s_schedule_asp(void)
{
	void (*sched)(void);

	sched = (void (*)(void))__symbol_get("cs42l81_schedule_post_iis");
	if (sched) {
		sched();
		__symbol_put("cs42l81_schedule_post_iis");
	}
}

static void s5l8740_i2s_cancel_asp(void)
{
	void (*cancel)(void);

	cancel = (void (*)(void))__symbol_get("cs42l81_cancel_post_iis");
	if (cancel) {
		cancel();
		__symbol_put("cs42l81_cancel_post_iis");
	}
}
static void s5l8740_i2s_log_txcon(struct device *dev, u32 v, const char *tag)
{
	dev_info(dev,
		 "%s txcon=0x%08x %s%s%s%s%s\n",
		 tag, v,
		 v == 0x0B100019u ? "ROCKBOX-POISON " : "",
		 (v & 0x03000001u) ? "BCB60-low " : "",
		 (v & 0x00100098u) ? "16bit " : "",
		 v == 0x03100219u ? "extclk " : "",
		 v == 0x03100099u ? "intclk " : "");
}

/*
 * 345D70 is JUMPOUT 0x22000350 = bootloader sub_350 (SCTLR C-bit).
 * Play 414FAE only starts — it does not C09AC-stop first.
 * RetailOS music: I2SCLKCON = 0x1 (not 0x2 stop-ack).
 */
static void s5l8740_i2s_c09ac_start(struct s5l8740_i2s *i2s)
{
	writel(1, i2s->base + I2SCLKCON);
}

/* OSOS sub_C095E(port, 0): clear TX sticky STATUS bit15 (W1C). */
static void s5l8740_i2s_status_w1c_tx(struct s5l8740_i2s *i2s)
{
	u32 before, after;

	if (!i2s || !i2s->base)
		return;
	before = readl(i2s->base + I2SSTATUS);
	writel(I2SSTATUS_TX_W1C, i2s->base + I2SSTATUS);
	after = readl(i2s->base + I2SSTATUS);
	if (i2s->dev && (before & I2SSTATUS_TX_W1C))
		dev_info(i2s->dev, "STATUS W1C tx sticky %08x->%08x\n",
			 before, after);
}

/* 26DDDE: 41CBD8(9,1), 5705DC RX, 414FAE (C09AC + BCB60), D34C0 CLKDIV. */
static void s5l8740_i2s_program(struct s5l8740_i2s *i2s, unsigned int rate)
{
	const struct n31_rate_cfg *r = n31_find_rate(rate);
	u32 div;
	u32 rxcom;

	if (clkdiv)
		div = clkdiv;
	else if (r)
		div = r->clkdiv;
	else
		div = MCLK_ASSUME_HZ / n31_pick_rate(rate);
	if (div < 1)
		div = 1;
	s5l8740_i2s_ungate(i2s);
	s5l8740_i2s_clkcon_audio(i2s, CLKCON_AUDIO_PLAY);
	s5l8740_i2s_c09ac_start(i2s);
	s5l8740_i2s_pads(i2s);
	/* sub_BCB60 a3!=0 a5=16 → TXCON; +0x30 = 0x1000 */
	writel(txcon, i2s->base + I2STXCON);
	if (i2s->dev)
		s5l8740_i2s_log_txcon(i2s->dev, txcon, "program");
	writel(I2SRXCON_N31, i2s->base + I2SRXCON);
	rxcom = readl(i2s->base + I2SRXCOM);
	writel(rxcom & ~4u, i2s->base + I2SRXCOM);
	writel(div, i2s->base + I2SCLKDIV);
	/* RetailOS music IIS0+0x44 = 0x00010007 (IIS2 already programs this). */
	writel(0x00010007u, i2s->base + I2SREG44);
	/* Setup only — TXCOM stays 0 until .trigger START (OSOS B6620). */
	writel(I2STXCOM_STOP, i2s->base + I2STXCOM);
	i2s->rate = n31_pick_rate(rate);
}

/*
 * OSOS B6620(port,0): TXCOM |= 6 after PL080 armed. Glass also needs bit 3
 * (PIO path 0xC) or STATUS stays 0x24 / jack silent. Set bit 3 before DMA.
 */
static void s5l8740_i2s_tx_kick(struct s5l8740_i2s *i2s, bool dma)
{
	u32 txcom, before, after;

	if (!i2s || !i2s->base)
		return;
	s5l8740_i2s_pre_codec();
	/*
	 * Prime the FIFO with silence so the serialiser has something to
	 * clock out between the kick and the first DMA burst. This used to
	 * push a generated tone, which mixed a chirp into the front of every
	 * stream the card played.
	 */
	{
		unsigned int n = fifo_prefill, i;

		if (n > 64)
			n = 64;
		for (i = 0; i < n; i++)
			writel(0, i2s->base + I2STXFIFO);
	}
	before = readl(i2s->base + I2STXCOM);
	if (txcom_exact >= 0) {
		writel((u32)txcom_exact, i2s->base + I2STXCOM);
	} else if (dma) {
		switch (txcom_mode) {
		case 0: /* retail: OSOS B6620 TXCOM = 0x6 after DMA armed */
			writel(I2STXCOM_DMA, i2s->base + I2STXCOM);
			break;
		case 1: /* pio-only kick (debug) */
			writel(txcom_pio, i2s->base + I2STXCOM);
			break;
		default: /* hybrid: glass bit3 + DMA */
			txcom = before;
			writel(txcom | I2STXCOM_PIO, i2s->base + I2STXCOM);
			writel(txcom | I2STXCOM_PIO | I2STXCOM_DMA,
			       i2s->base + I2STXCOM);
			break;
		}
	} else {
		writel(txcom_pio, i2s->base + I2STXCOM);
	}
	after = readl(i2s->base + I2STXCOM);
	/* RetailOS music never leaves TX sticky 0x8000 set — clear after kick. */
	s5l8740_i2s_status_w1c_tx(i2s);
	/* Keep C09AC start bit; 0x2 is stop-ack class (sub_C09AC wait). */
	if ((readl(i2s->base + I2SCLKCON) & 1u) == 0)
		writel(1, i2s->base + I2SCLKCON);
	if (i2s->dev)
		dev_info_ratelimited(i2s->dev,
			 "tx_kick dma=%d mode=%d txcom %08x->%08x status=%08x clkcon=%08x\n",
			 dma, txcom_mode, before, after,
			 readl(i2s->base + I2SSTATUS),
			 readl(i2s->base + I2SCLKCON));
}

static void s5l8740_i2s_dma_watch(struct work_struct *work)
{
	struct s5l8740_i2s *i2s = container_of(work, struct s5l8740_i2s,
					       dma_watch.work);
	u32 src = 0, dst = 0, en = 0, st, txcom;
	int ret;

	if (!i2s || !i2s->base)
		return;
	ret = s5l_pl080_peri_snapshot(10, &src, &dst, &en);
	st = readl(i2s->base + I2SSTATUS);
	txcom = readl(i2s->base + I2STXCOM);
	{
		const char *tag;

		if (ret)
			tag = "NOPERI";
		else if (i2s->watch_ticks == 0)
			tag = "BASE";
		else if (src != i2s->last_dma_src)
			tag = "WALK";
		else
			tag = "STUCK";
		dev_info(i2s->dev,
			 "dma_watch t=%ums src=%08x %s dst=%08x en=%x status=%08x txcom=%08x\n",
			 i2s->watch_ticks * 100, src, tag, dst, en, st, txcom);
	}
	i2s->last_dma_src = src;
	i2s->watch_ticks++;
	if (i2s->watch_ticks < 50)
		schedule_delayed_work(&i2s->dma_watch, msecs_to_jiffies(100));
}

static int s5l8740_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dai->dev);
	unsigned int rate = params_rate(params);
	const struct n31_rate_cfg *r = n31_find_rate(rate);
	u32 div;
	int ret;

	if (!i2s || !i2s->base)
		return -ENODEV;
	if (!r && !clkdiv)
		return -EINVAL;
	ret = s5l8740_i2s_codec_prepare();
	if (ret && i2s->dev)
		dev_warn(i2s->dev, "codec prepare in hw_params: %d\n", ret);
	s5l8740_i2s_program(i2s, rate);
	div = clkdiv ? clkdiv : (r ? r->clkdiv : 0);
	s5l8740_i2s_log_clocks(i2s, "hw_params");
	dev_info(dai->dev,
		 "IIS hw_params rate=%u code=%u clkdiv=%u dma=%d pio=%d txcom=%08x\n",
		 rate, r ? r->cs42_rate_code : 0, div, i2s->has_dma, use_pio,
		 readl(i2s->base + I2STXCOM));
	return 0;
}

static int s5l8740_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dai->dev);
	int path_mode;

	if (!i2s || !i2s->base)
		return -ENODEV;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		path_mode = s5l8740_i2s_audio_path_mode();
		if (path_mode == 1)
			s5l8740_i2s_codec_play_start();
		s5l8740_i2s_tx_kick(i2s, !use_pio);
		if (path_mode == 2)
			s5l8740_i2s_codec_play_start();
		s5l8740_i2s_log_clocks(i2s, "trigger_start");
		s5l8740_i2s_schedule_asp();
		i2s->pio_run = use_pio;
		i2s->play_jiffies = jiffies;
		i2s->watch_ticks = 0;
		i2s->last_dma_src = 0;
		mod_delayed_work(system_wq, &i2s->dma_watch, msecs_to_jiffies(100));
		dev_info(dai->dev,
			 "DAI trigger START path_mode=%d txcom=%08x sustain=%ums\n",
			 path_mode, readl(i2s->base + I2STXCOM), sustain_ms);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (sustain_ms &&
		    time_before(jiffies,
				i2s->play_jiffies +
				msecs_to_jiffies(sustain_ms))) {
			dev_info_ratelimited(dai->dev,
					     "DAI trigger STOP ignored (%ums sustain)\n",
					     sustain_ms);
			return 0;
		}
		i2s->pio_run = false;
		cancel_delayed_work(&i2s->dma_watch);
		s5l8740_i2s_cancel_asp();
		s5l8740_i2s_codec_play_stop();
		s5l8740_i2s_hw_stop(i2s, substream);
		dev_info_ratelimited(dai->dev, "DAI trigger STOP txcom=0\n");
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
		rate = i2s->rate ? i2s->rate : N31_RATE_DEFAULT;
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
		I2SRXCON, I2SRXCOM, I2SSTATUS, I2SCLKDIV, I2SREG44,
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
			0x30, 0x44, 0x48, 0x4c, 0x58, 0x68, 0x6c,
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

static struct dma_chan *s5l8740_i2s_tx_get(struct s5l8740_i2s *i2s)
{
	struct dma_chan *chan;

	if (i2s->tx_chan)
		return i2s->tx_chan;
	chan = s5l_pl080_lookup_peri(10);
	if (chan) {
		i2s->tx_chan = chan;
		i2s->tx_chan_borrowed = 1;
		return chan;
	}
	chan = s5l_pl080_request_slave(i2s->dev, 0);
	if (IS_ERR_OR_NULL(chan))
		return chan;
	i2s->tx_chan = chan;
	i2s->tx_chan_borrowed = 0;
	return chan;
}

static void s5l8740_i2s_tx_put(struct s5l8740_i2s *i2s)
{
	if (!i2s || !i2s->tx_chan)
		return;
	if (!i2s->tx_chan_borrowed)
		dma_release_channel(i2s->tx_chan);
	i2s->tx_chan = NULL;
	i2s->tx_chan_borrowed = 0;
}

static unsigned int s5l8740_i2s_tone_rate(void)
{
	if (tone_rate)
		return n31_pick_rate(tone_rate);
	return n31_pick_rate(default_rate);
}

static ssize_t pio_tone_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dev);
	unsigned int rate, frames, i;
	s16 s;

	if (!i2s || !i2s->base)
		return -ENODEV;
	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;

	rate = s5l8740_i2s_tone_rate();
	s5l8740_i2s_codec_prepare();
	s5l8740_i2s_program(i2s, rate);
	s5l8740_i2s_codec_play_start();
	s5l8740_i2s_tx_kick(i2s, false);

	frames = rate * 2;
	for (i = 0; i < frames; i++) {
		s = s5l8740_scale_s16(n31_tone_s16(i, rate));
		writel(((u32)(u16)s << 16) | (u16)s, i2s->base + I2STXFIFO);
		udelay(1000000 / rate);
	}
	s5l8740_i2s_codec_play_stop();
	s5l8740_i2s_hw_stop(i2s, NULL);
	dev_info(dev, "pio_tone 2s rate=%u status=0x%08x\n",
		 rate, readl(i2s->base + I2SSTATUS));
	return count;
}
static DEVICE_ATTR_WO(pio_tone);

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
	unsigned int rate, frames, i;
	size_t bytes;
	s16 s;
	int ret;

	if (!i2s || !i2s->base)
		return -ENODEV;
	if (buf[0] != '1' && buf[0] != 'y' && buf[0] != 'Y')
		return -EINVAL;

	rate = s5l8740_i2s_tone_rate();
	frames = n31_tone_period_frames(rate);
	bytes = frames * 2 * sizeof(s16);

	mutex_lock(&i2s->dma_lock);
	chan = s5l8740_i2s_tx_get(i2s);
	if (IS_ERR_OR_NULL(chan)) {
		ret = chan ? PTR_ERR(chan) : -ENODEV;
		dev_err(dev, "dma_tone request tx: %d\n", ret);
		mutex_unlock(&i2s->dma_lock);
		return ret;
	}

	tone = dma_alloc_coherent(dev, bytes, &dma, GFP_KERNEL);
	if (!tone) {
		ret = -ENOMEM;
		goto out_unlock;
	}
	for (i = 0; i < frames; i++) {
		s = s5l8740_scale_s16(n31_tone_s16(i, rate));
		tone[i * 2] = s;
		tone[i * 2 + 1] = s;
	}
	dma_sync_single_for_device(dev, dma, bytes, DMA_TO_DEVICE);

	cfg.direction = DMA_MEM_TO_DEV;
	cfg.dst_addr = i2s->play_dma.addr;
	cfg.dst_addr_width = (tone_width == 2) ?
		DMA_SLAVE_BUSWIDTH_2_BYTES : DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_maxburst = 1;
	ret = dmaengine_slave_config(chan, &cfg);
	if (ret) {
		dev_err(dev, "dma_tone slave_config: %d\n", ret);
		goto out_buf;
	}

	s5l8740_i2s_codec_prepare();
	s5l8740_i2s_program(i2s, rate);
	desc = dmaengine_prep_dma_cyclic(chan, dma, bytes, bytes,
					 DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dev, "dma_tone prep_dma_cyclic failed\n");
		ret = -ENOMEM;
		goto out_buf;
	}
	cookie = dmaengine_submit(desc);
	if (dma_submit_error(cookie)) {
		ret = cookie;
		goto out_buf;
	}
	dma_async_issue_pending(chan);
	s5l8740_i2s_codec_play_start();
	s5l8740_i2s_tx_kick(i2s, true);
	s5l8740_i2s_schedule_asp();
	dev_info(dev, "dma_tone 1kHz rate=%u frames=%u bytes=%zu cyclic 2s\n",
		 rate, frames, bytes);
	msleep(2000);
	s5l8740_i2s_cancel_asp();
	s5l8740_i2s_codec_play_stop();
	dmaengine_terminate_sync(chan);
	s5l8740_i2s_hw_stop(i2s, NULL);
	dev_info(dev, "dma_tone done status=0x%x txcom=0x%x\n",
		 readl(i2s->base + I2SSTATUS),
		 readl(i2s->base + I2STXCOM));
	ret = 0;
out_buf:
	dma_free_coherent(dev, bytes, tone, dma);
out_unlock:
	mutex_unlock(&i2s->dma_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(dma_tone);

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
		s5l8740_i2s_program(i2s, n31_pick_rate(default_rate));
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
		i2s->play_dma.maxburst = 4;
	}

	platform_set_drvdata(pdev, i2s);
	dev_set_drvdata(dev, i2s);
	mutex_init(&i2s->dma_lock);
	INIT_DELAYED_WORK(&i2s->dma_watch, s5l8740_i2s_dma_watch);

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
	cancel_delayed_work_sync(&i2s->dma_watch);
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

/* ------------------------------------------------------------------ */
/* IIS2 — BCM2078 digital PCM capture                                   */
/* ------------------------------------------------------------------ */

static uint iis2_clkdiv;
module_param(iis2_clkdiv, uint, 0644);
MODULE_PARM_DESC(iis2_clkdiv, "IIS2 CLKDIV override; 0 = FM oracle 0x96");

struct s5l8740_iis2 {
	void __iomem *base;
	void __iomem *clkcon;
	void __iomem *gpio;
	void __iomem *gpiocmd;
	struct device *dev;
	struct clk_bulk_data *clks;
	int num_clks;
	bool has_dma;
	struct snd_dmaengine_dai_dma_data cap_dma;
	u32 fm_gate_saved;
	bool fm_gate_held;
	unsigned int rate;
};

static u32 iis2_pick_clkdiv(unsigned int rate)
{
	const struct n31_rate_cfg *r;

	if (iis2_clkdiv)
		return iis2_clkdiv;
	/*
	 * The FM capture uses 0x96 where IIS0 runs 0x177 in the same session,
	 * so this divider is not derived from the IIS0 rate table.
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

static void iis2_pads(struct s5l8740_iis2 *iis2, bool claim)
{
	static const u8 pads[] = {
		IIS2_PAD_BCLK, IIS2_PAD_SYNC, IIS2_PAD_DATA,
	};
	void __iomem *bank;
	unsigned int i, pin;
	u32 dir;

	if (!iis2->gpio || !iis2->gpiocmd)
		return;
	for (i = 0; i < ARRAY_SIZE(pads); i++) {
		bank = iis2->gpio + 32u * (pads[i] >> 3);
		pin = pads[i] & 7;
		dir = readl(bank + 0x14);
		if (claim)
			dir |= BIT(pin);
		else
			dir &= ~BIT(pin);
		writel(dir, bank + 0x14);
		writel(((u32)(pads[i] >> 3) << 16) | (pin << 8) |
		       (claim ? IIS2_PAD_FUNC : IIS2_PAD_RELEASE),
		       iis2->gpiocmd);
	}
	if (iis2->dev)
		dev_dbg(iis2->dev, "IIS2 pads 97/98/119 %s\n",
			claim ? "claimed" : "released");
}

static void iis2_fm_gate(struct s5l8740_iis2 *iis2, bool on)
{
	u32 cur;

	if (!iis2 || !iis2->clkcon)
		return;
	cur = readl(iis2->clkcon + CLKCON_FM_GATE);
	if (on) {
		if (!iis2->fm_gate_held) {
			iis2->fm_gate_saved = cur;
			iis2->fm_gate_held = true;
		}
		writel(CLKCON_FM_GATE_ON, iis2->clkcon + CLKCON_FM_GATE);
	} else if (iis2->fm_gate_held) {
		writel(iis2->fm_gate_saved ? iis2->fm_gate_saved :
					     CLKCON_FM_GATE_IDLE,
		       iis2->clkcon + CLKCON_FM_GATE);
		iis2->fm_gate_held = false;
	}
}

/* Peri 13 must be armed by dmaengine before RXCOM is kicked. */
static void iis2_program_rx(struct s5l8740_iis2 *iis2)
{
	iis2_pads(iis2, true);
	iis2_fm_gate(iis2, true);
	s5l8740_audio_clk_set(iis2->clkcon, S5L8740_AUDIO_PORT_IIS2, true);
	writel(IIS2_CLKCON_ON, iis2->base + I2SCLKCON);
	writel(IIS2_TXCON_FM, iis2->base + I2STXCON);
	writel(IIS2_RXCON_FM, iis2->base + I2SRXCON);
	writel(iis2_pick_clkdiv(iis2->rate), iis2->base + I2SCLKDIV);
	writel(IIS2_REG44_ORACLE, iis2->base + I2SREG44);
}

static void iis2_hw_stop(struct s5l8740_iis2 *iis2)
{
	if (!iis2 || !iis2->base)
		return;
	writel(IIS2_RXCOM_IDLE, iis2->base + I2SRXCOM);
	s5l8740_audio_clk_set(iis2->clkcon, S5L8740_AUDIO_PORT_IIS2, false);
	iis2_fm_gate(iis2, false);
	iis2_pads(iis2, false);
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
		writel(IIS2_RXCOM_DMA, iis2->base + I2SRXCOM);
		dev_info(dai->dev,
			 "IIS2 capture start rxcom=0x%x status=0x%x\n",
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
		.rates = S5L8740_I2S_RATES,
		.formats = S5L8740_I2S_FORMATS,
	},
	.ops = &s5l8740_iis2_dai_ops,
};

static const struct snd_soc_component_driver s5l8740_iis2_component = {
	.name = "bcm2078-pcm",
	.legacy_dai_naming = 1,
};

static ssize_t iis2_regs_show(struct device *dev,
			      struct device_attribute *a, char *buf)
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
				   readl(iis2->clkcon + CLKCON_FM_GATE));
		n += sysfs_emit_at(buf, n, "clk+30: %08x\n",
				   readl(iis2->clkcon + CLKCON_AUDIO_OFF));
	}
	return n;
}
/* Same sysfs name as the IIS0 dump; different device, different symbol. */
static struct device_attribute dev_attr_iis2_regs =
	__ATTR(regs, 0444, iis2_regs_show, NULL);

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
	iis2->gpio = devm_ioremap(dev, GPIO_PHYS, 0x200);
	iis2->gpiocmd = devm_ioremap(dev, GPIOCMD_PHYS, 4);

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

	if (!of_property_present(dev->of_node, "dmas")) {
		dev_err(dev, "missing dmas (need peri 13 rx)\n");
		return -EINVAL;
	}
	ret = devm_snd_dmaengine_pcm_register(dev, NULL, 0);
	if (ret)
		return dev_err_probe(dev, ret, "dmaengine_pcm\n");
	iis2->has_dma = true;

	ret = devm_snd_soc_register_component(dev, &s5l8740_iis2_component,
					      &s5l8740_iis2_dai, 1);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_iis2_regs);
	if (ret)
		dev_warn(dev, "regs sysfs: %d\n", ret);

	dev_info(dev, "BCM2078 PCM RX @%pR peri13 FIFO@+0x38\n", res);
	return 0;
}

static void s5l8740_iis2_remove(struct platform_device *pdev)
{
	struct s5l8740_iis2 *iis2 = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_iis2_regs);
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

static struct platform_driver * const s5l8740_audio_drivers[] = {
	&s5l8740_i2s_driver,
	&s5l8740_iis2_driver,
};

static int __init s5l8740_audio_init(void)
{
	return platform_register_drivers(s5l8740_audio_drivers,
					 ARRAY_SIZE(s5l8740_audio_drivers));
}
module_init(s5l8740_audio_init);

static void __exit s5l8740_audio_exit(void)
{
	platform_unregister_drivers(s5l8740_audio_drivers,
				    ARRAY_SIZE(s5l8740_audio_drivers));
}
module_exit(s5l8740_audio_exit);

MODULE_DESCRIPTION("S5L8740 I2S DAIs: IIS0 playback + IIS2 capture (N31)");
MODULE_LICENSE("GPL");
