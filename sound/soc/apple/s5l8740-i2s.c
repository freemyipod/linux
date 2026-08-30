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
#include <linux/math64.h>
#include <linux/swab.h>
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

/* Advertise every rate the divider table can actually clock. */
#define S5L8740_I2S_RATES	N31_RATE_MASK
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
/* module_param moved below s5l8740_i2s_iis0; see txcon_set(). */
/*
 * D34C0 → 4F716(port, div). Table in n31-audio-rates.h.
 * 0 = 12 MHz / rate (272 @ 44.1 kHz RetailOS music).
 */
/*
 * DMA progress tracing.
 *
 * This ran unconditionally for 50 ticks at 100 ms on every stream start,
 * so each playback put fifty KERN_INFO lines on the console -- including a
 * long tail of STUCK reports after the transfer had already finished and
 * en had gone to 0, which reads like a fault but is just the timer
 * outliving the work. On a framebuffer console that much printk during
 * boot is slow enough to matter on its own.
 *
 * It stays available because it is genuinely useful for watching the
 * descriptor walk, but it is now something you ask for: set the number of
 * 100 ms samples to take, 0 (default) for silence.
 */
static uint dma_watch_ticks;
module_param(dma_watch_ticks, uint, 0644);
MODULE_PARM_DESC(dma_watch_ticks,
		 "log DMA progress for N samples of 100ms after each start; 0=off (default)");

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
/*
 * Off, and the stop path it used to suppress is fixed.
 *
 * Ignoring ALSA's STOP for five seconds is a bring-up crutch and it makes
 * the PCM layer and the hardware disagree about whether a stream is
 * running. It should be 0. But setting it to 0 was the one behavioural
 * change between a clean boot and a boot that hangs, and the reason is
 * that the STOP path has never actually executed: with the crutch in
 * place it was skipped every time.
 *
 * What it hits is s5l8740_i2s_cancel_asp() -> cs42l81_cancel_post_iis(),
 * which is a cancel_delayed_work_sync() on a work item that takes
 * c->lock. That blocks until the work completes, so if the work is
 * waiting on that lock the stop never returns.
 *
 * That cancel is now the non-blocking form, so the stop path no longer
 * waits on a work item that wants the codec lock. Dead code that has never
 * run is not the same as code that works, which is the whole reason this
 * surfaced the moment the crutch came off.
 */
/*
 * Backtrace the first few hw_params calls, on by default until the thing
 * that drives the boot-time open/start loop is identified.
 */
static bool hw_params_trace = true;
module_param(hw_params_trace, bool, 0644);
MODULE_PARM_DESC(hw_params_trace, "1=dump_stack() on the first few hw_params calls");
static unsigned int hw_params_seen;

/*
 * Halt after this many hw_params calls, naming the caller. 0 disables.
 * A normal boot opens the PCM a handful of times at most.
 */
static unsigned int hw_params_loop_panic;	/* off: the loop was tone-loop from net-up */
module_param(hw_params_loop_panic, uint, 0644);
MODULE_PARM_DESC(hw_params_loop_panic,
		 "panic after N PCM opens to freeze the caller name on screen (0=off)");

static uint sustain_ms;
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

/* ------------------------------------------------------------------ */
/* PCM inspection and sample-format probes                              */
/*                                                                      */
/* The jack carries a 1 kHz fundamental with 2k/3k/4k/5k harmonics at    */
/* comparable amplitude, and a peak around 255 against a source that     */
/* peaks near 23000. A gain error would keep the sine clean and only     */
/* lower it; a mangled periodic waveform means the sample word is being  */
/* interpreted wrongly somewhere between the ring buffer and the codec.  */
/*                                                                      */
/* pcm_dump prints what the driver is actually about to hand the DMA, so */
/* the question "is the buffer already byte-scale?" is answered from the */
/* kernel rather than inferred from the analog end.                      */
/* ------------------------------------------------------------------ */

/*
 * Off. pio_tone, dma_tone and walk_bit exist to poke tones and sweep TXCON
 * bits by hand during bring-up. They drive the codec and the DMA engine
 * directly, and walk_one re-enters codec prepare, so a stray write to any
 * of them from a script or a stale test harness moves real hardware. They
 * are not needed for normal operation; set debug_tone=1 when deliberately
 * using them.
 */
static bool debug_tone;
module_param(debug_tone, bool, 0644);
MODULE_PARM_DESC(debug_tone,
		 "1=enable the pio_tone/dma_tone/walk_bit debug pokes (default off)");

static unsigned int walk_ms = 1000;
module_param(walk_ms, uint, 0644);
MODULE_PARM_DESC(walk_ms, "Milliseconds per walking-bit step");

static unsigned int walk_gap_ms = 400;
module_param(walk_gap_ms, uint, 0644);
MODULE_PARM_DESC(walk_gap_ms, "Silence between walking-bit steps");

static bool pcm_dump;
module_param(pcm_dump, bool, 0644);
MODULE_PARM_DESC(pcm_dump,
		 "Log the first frames and their range at each stream start");

/*
 * Sample rewrites applied on the way to the FIFO. All default to off, so
 * the transport is unchanged unless something is being tested.
 */
static int sample_shift;
module_param(sample_shift, int, 0644);
MODULE_PARM_DESC(sample_shift,
		 "Arithmetic shift applied per sample, -16..16 (0 = none)");

static bool sample_byteswap;
module_param(sample_byteswap, bool, 0644);
MODULE_PARM_DESC(sample_byteswap, "Swap the two bytes of each sample");

static bool sample_swap_lr;
module_param(sample_swap_lr, bool, 0644);
MODULE_PARM_DESC(sample_swap_lr, "Swap left and right within each frame");

static s16 s5l8740_sample_fix(s16 v)
{
	int x = v;

	if (sample_shift > 0)
		x <<= min(sample_shift, 16);
	else if (sample_shift < 0)
		x >>= min(-sample_shift, 16);
	if (sample_byteswap)
		x = (s16)__swab16((u16)x);
	return (s16)x;
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
	bool programmed;
	unsigned long play_jiffies;
	u32 last_dma_src;
	u8 watch_ticks;
};

/*
 * Report the buffer as the hardware will see it: signed values, the range
 * over a period, and the raw halfwords. A source that peaks in the low
 * hundreds here is an application or format-negotiation fault and nothing
 * downstream needs changing.
 */
static void s5l8740_pcm_dump(struct s5l8740_i2s *i2s, const s16 *buf,
			     unsigned int frames, const char *tag)
{
	int lo = 32767, hi = -32768;
	unsigned int i, n = min(frames, 512u);
	long long acc = 0;

	if (!pcm_dump || !buf || !n || !i2s->dev)
		return;
	for (i = 0; i < n * 2; i++) {
		int v = buf[i];

		lo = min(lo, v);
		hi = max(hi, v);
		acc += (long long)v * v;
	}
	dev_info(i2s->dev,
		 "pcm %s: %u frames min=%d max=%d rms=%u\n",
		 tag, n, lo, hi,
		 (unsigned int)int_sqrt((unsigned long)div64_u64(acc, n * 2)));
	dev_info(i2s->dev, "pcm %s: first 8 (L,R) %*ph\n",
		 tag, 32, buf);
}


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

/*
 * CLKCON+0x10 is the FM clock, and both paths write it.
 *
 * The decomp names it: sub_15DD5C powers FM through sub_41CBD8(v2, on)
 * with v2 = sub_4E7B0() = 11, and case 11 of sub_41CBD8 clears bit 15
 * of 0x3C500010 to enable and sets it to disable. That is exactly the
 * CLKCON_FM_GATE_ON / _IDLE pair below, which had been derived from the
 * oracle.
 *
 * s5l8740_i2s_ungate writes the whole music-playing CLKCON snapshot,
 * and in that snapshot FM is off -- STOCK_CLKCON_10 is 0x8000, bit 15
 * set, divider nibble zeroed. Playback starting while FM capture ran
 * therefore gated off the capture's own clock and destroyed its
 * divider. That is the same failure the +0x30 arbitration above already
 * fixes in the other direction, so it gets the same treatment: while
 * IIS2 holds the FM gate, playback leaves +0x10 alone.
 */
static bool s5l8740_fm_gate_held;

/*
 * OFF by default. This pushes a RetailOS music-playing snapshot into
 * CLKCON +0x08..0x1C as whole-register writes, and those are SoC-wide
 * clock gates, not audio-private ones. Blind full-register writes
 * therefore discard whatever every other block had set. Observed: the
 * device boots, FIL_Init reports the NAND fine, then the first audio
 * start overwrites the clock tree and the FMSS controller loses its
 * clock -- FMCTRL1 bit 30 never sets again, FMCTRL0 and NANDSTAT both
 * read back the same stale word, and storage is gone until reboot.
 *
 * The conservative branch below is also the attested one: it only
 * ungates what sub_41CBD8(9,1) actually specifies, read-modify-write,
 * and leaves every other block's bits alone. Set this to 1 only to
 * reproduce the snapshot experiment, and expect to lose storage.
 */
/*
 * Write only CLKCON+0x1C, and nothing else.
 *
 * force_stock_audio_parent below is off because it pushes the whole
 * music-playing snapshot into +0x08..0x1C, and those are SoC-wide gates:
 * the first audio start took the FMSS clock with it and storage was gone
 * until reboot. That is a good reason to distrust the snapshot, and a bad
 * reason to leave the one register the oracle actually calls out.
 *
 * Checkpoint-010's truth table gives +0x1C = 0xD0052003 for RetailOS
 * playing music. We run 0x10122003, the SEC bring-up value. The low half
 * is identical -- 0x2003 either way -- so the difference is entirely in
 * the upper 16 bits, which is where the audio parent selection lives.
 *
 * +0x08, +0x0C, +0x10 and +0x14 are the registers that killed the NAND and
 * they are not touched. This writes one register that the oracle attests
 * to, and the storage path is checked after every test.
 */
static bool stock_clkcon_1c = true;
module_param(stock_clkcon_1c, bool, 0644);
MODULE_PARM_DESC(stock_clkcon_1c,
		 "write the RetailOS audio parent to CLKCON+0x1C only (default Y)");

static bool force_stock_audio_parent;
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
		if (!READ_ONCE(s5l8740_fm_gate_held))
			writel(STOCK_CLKCON_10, i2s->clkcon + 0x10);
		writel(STOCK_CLKCON_14, i2s->clkcon + 0x14);
		writel(STOCK_CLKCON_18, i2s->clkcon + 0x18);
		writel(STOCK_CLKCON_1C, i2s->clkcon + 0x1c);
	} else {
		if (!r18)
			writel(SEC_CLKCON_18, i2s->clkcon + 0x18);
		if (stock_clkcon_1c) {
			if (r1c != STOCK_CLKCON_1C) {
				writel(STOCK_CLKCON_1C, i2s->clkcon + 0x1c);
				dev_info(i2s->dev,
					 "CLKCON+1C %08x -> %08x (stock audio parent)\n",
					 r1c, STOCK_CLKCON_1C);
			}
		} else if (!r1c) {
			writel(SEC_CLKCON_1C, i2s->clkcon + 0x1c);
		}
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

/*
 * Only touch the bits that are actually about audio.
 *
 * CLKCON_AUDIO_PLAY (0x32190) and CLKCON_AUDIO_IDLE (0x1c20) are whole-
 * register snapshots taken from RetailOS at two moments, and this used to
 * writel() one of them over CLKCON+0x30 in its entirety on every play and
 * every stop. That imposes the entire captured clock state of the SoC,
 * including whatever every other peripheral happened to be doing when the
 * snapshot was taken -- and CLKCON+0x30 carries gates that are nothing to
 * do with audio. The FMSS is one of them: after a few play/stop cycles the
 * NAND controller reads back FMCTRL0=0 NANDSTAT=0, sub_10453C times out on
 * FMCTRL1 bit 30, and Whimory open fails with -110. Storage is gone until
 * the next boot.
 *
 * Stock never does this. sub_41CBD8 sets or clears exactly one bit:
 *
 *     v = MEMORY[0x3C500008] & 0x7FFFFFFF;   // or | 0x80000000
 *
 * and leaves the register otherwise untouched.
 *
 * The two snapshots differ in a fixed set of bits, and those are the ones
 * that plausibly belong to audio. Everything outside that set is somebody
 * else's and is now preserved from the live register.
 */
#define CLKCON_AUDIO_MASK	(CLKCON_AUDIO_PLAY ^ CLKCON_AUDIO_IDLE)


/*
 * Apply TXCON to the live register as soon as it is written.
 *
 * Sweeping TXCON used to mean setting clkdiv to defeat the "already
 * programmed" check in s5l8740_i2s_program(), which re-ran the pad mux and
 * the CLKCON writes on every play. That is how the NAND got clock-gated and
 * how the device wedged mid-stream. This writes IIS0+0x04 and nothing else,
 * so a sweep costs one register write and cannot disturb any clock.
 */
static struct s5l8740_i2s *s5l8740_i2s_iis0;

static int txcon_set(const char *val, const struct kernel_param *kp)
{
	struct s5l8740_i2s *i2s = READ_ONCE(s5l8740_i2s_iis0);
	int ret = param_set_uint(val, kp);

	if (ret)
		return ret;
	if (i2s && i2s->base) {
		writel(txcon, i2s->base + I2STXCON);
		dev_info(i2s->dev, "txcon live -> 0x%08x (status=0x%08x)\n",
			 txcon, readl(i2s->base + I2SSTATUS));
	}
	return 0;
}

static const struct kernel_param_ops txcon_ops = {
	.set = txcon_set,
	.get = param_get_uint,
};
module_param_cb(txcon, &txcon_ops, &txcon, 0644);
MODULE_PARM_DESC(txcon, "I2STXCON; written live on set (default 0x03100099)");

static void s5l8740_audio_clk_set(void __iomem *clkcon, unsigned int port,
				  bool on)
{
	unsigned long flags;
	unsigned int i;
	bool any = false;
	u32 want, cur, new;

	if (!clkcon)
		return;
	spin_lock_irqsave(&s5l8740_audio_clk_lock, flags);
	s5l8740_audio_clk_wanted[port] = on;
	for (i = 0; i < S5L8740_AUDIO_PORTS; i++)
		any |= s5l8740_audio_clk_wanted[i];
	want = any ? CLKCON_AUDIO_PLAY : CLKCON_AUDIO_IDLE;
	cur = readl(clkcon + CLKCON_AUDIO_OFF);
	new = (cur & ~CLKCON_AUDIO_MASK) | (want & CLKCON_AUDIO_MASK);
	if (new != cur)
		writel(new, clkcon + CLKCON_AUDIO_OFF);
	spin_unlock_irqrestore(&s5l8740_audio_clk_lock, flags);
}


/*
 * The codec clock gate, RetailOS sub_41CBD8 with the id sub_4F82F8 returns.
 *
 * sub_4F82F8() returns 9, and case 9 of sub_41CBD8 is CLKCON+0x0C bit 15,
 * active low:
 *
 *     if (on)  MEMORY[0x3C50000C] &= 0xFFFF7FFF;
 *     else     MEMORY[0x3C50000C] |= 0x8000;
 *
 * D3280(1) ends by turning it OFF and D3280(3) begins by turning it back
 * ON, which is what the 0x0006 / 0x0007 bit-6 freeze latch is bracketing:
 * the codec is parked, its clock is stopped, and then the clock returns and
 * the latch is released. This driver left the clock running the whole time,
 * so that transition never happened.
 *
 * Exported because the CLKCON mapping lives here and the codec driver needs
 * it. One bit, read-modify-write, nothing else touched -- the opposite of
 * what s5l8740_audio_clk_set() used to do to CLKCON+0x30.
 */
void s5l8740_codec_clk_gate(bool on)
{
	struct s5l8740_i2s *i2s = READ_ONCE(s5l8740_i2s_iis0);
	unsigned long flags;
	u32 v;

	if (!i2s || !i2s->clkcon)
		return;
	spin_lock_irqsave(&s5l8740_audio_clk_lock, flags);
	v = readl(i2s->clkcon + 0x0c);
	if (on)
		v &= ~0x8000u;
	else
		v |= 0x8000u;
	writel(v, i2s->clkcon + 0x0c);
	spin_unlock_irqrestore(&s5l8740_audio_clk_lock, flags);
	dev_info(i2s->dev, "codec clk gate %s (CLKCON+0x0C=0x%08x)\n",
		 on ? "ON" : "OFF", v);
}
EXPORT_SYMBOL_GPL(s5l8740_codec_clk_gate);

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
/*
 * The stock pad set while RetailOS plays, from audio checkpoint-010:
 * bank0 PCON 0x32112224 / DIR 0xFF, bank2 PCON 0x02230000 / DIR 0x70.
 *
 * We set 7 and 20 and have never touched 6, 21 or 22. That gap is worth
 * closing now because everything else matches the oracle -- TXCON, TXCOM,
 * CLKDIV 272, CLKCON +0x18 and +0x1C, IIS STATUS 0x424, PL080 channel 2 on
 * peri 10 -- and the jack is still silent. Clocks and status can all read
 * correct while the serialiser's data pin is not muxed out of the SoC,
 * which is exactly the case the checkpoint's "wire/data" branch describes.
 */
static const struct { u8 gpio, func; } stock_audio_pads[] = {
	{ 6, 2 }, { 7, 3 }, { 20, 3 }, { 21, 2 }, { 22, 2 },
};

/*
 * Off: measured, and there is nothing to restore.
 *
 * devmem on the running device reads bank0 PCON 0x32222224 / DIR 0xFF and
 * bank2 PCON 0x02230000 / DIR 0x70 against the oracle's 0x32112224 / 0xFF
 * and 0x02230000 / 0x70. Every audio pad already matches -- GPIO6 f2,
 * GPIO7 f3, GPIO20 f3, GPIO21 f2, GPIO22 f2, all outputs. The only bank0
 * difference is pins 4 and 5, which belong to I2C0 and which the
 * checkpoint explicitly says to leave alone.
 *
 * Kept because it costs nothing and pins the invariant, but it is not the
 * silence and turning it on requires a kernel rebuild -- gpio-s5l8740 is
 * built in, so the export it needs is not reachable from a module reload.
 */
static bool force_stock_audio_pads;
module_param(force_stock_audio_pads, bool, 0644);
MODULE_PARM_DESC(force_stock_audio_pads,
		 "restore the stock GPIO6/7/20/21/22 audio pad nibbles (default Y)");

static void s5l8740_i2s_stock_pads(struct s5l8740_i2s *i2s)
{
	int (*setpad)(unsigned int, unsigned int, bool);
	unsigned int i;

	setpad = (int (*)(unsigned int, unsigned int, bool))
		 __symbol_get("s5l8740_gpio_set_pad");
	if (!setpad) {
		dev_warn(i2s->dev, "stock pads: gpio export missing\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(stock_audio_pads); i++)
		setpad(stock_audio_pads[i].gpio, stock_audio_pads[i].func,
		       true);
	__symbol_put("s5l8740_gpio_set_pad");
	dev_info(i2s->dev,
		 "stock audio pads applied: 6=f2 7=f3 20=f3 21=f2 22=f2, all out\n");
}

static void s5l8740_i2s_pads(struct s5l8740_i2s *i2s)
{
	static const u8 sec_words[] = { 6, 7, 20 };
	unsigned int i;

	if (force_stock_audio_pads) {
		s5l8740_i2s_stock_pads(i2s);
		s5l8740_i2s_log_iis_gpio(i2s, "pads-stock");
		return;
	}

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
	const struct n31_rate_cfg *r;
	u32 div;
	u32 rxcom;

	/* Same resolver the codec uses, so the divider always matches. */
	rate = n31_resolve_rate(rate);
	r = n31_find_rate(rate);

	/*
	 * Do not re-do all of this for a rate we are already programmed for.
	 *
	 * Something opens the PCM during boot and prepare can fail, and when
	 * it does the PCM layer retries at the next advertised rate. Every one
	 * of those retries landed here and re-ran the pad mux, the TXCON and
	 * RXCON writes and the clock programming from scratch. With two rates
	 * advertised that was a couple of passes; advertising all nine the
	 * hardware supports turned it into a visible storm of pinmux and reset
	 * lines at boot, which is alarming to read and does real work for no
	 * reason.
	 *
	 * Programming is idempotent, so the cheapest correct answer is not to
	 * repeat it. This does not fix whatever opens the PCM or whatever makes
	 * prepare fail -- both are still open -- it stops those from thrashing
	 * the pads and the clock while they are unresolved.
	 */
	if (i2s->programmed && i2s->rate == rate && !clkdiv) {
		dev_dbg(i2s->dev, "program: already at %u, skipping\n", rate);
		return;
	}

	if (clkdiv)
		div = clkdiv;
	else if (r)
		div = r->clkdiv;
	else
		div = MCLK_ASSUME_HZ / N31_RATE_DEFAULT;
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
	i2s->rate = rate;
	i2s->programmed = true;
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
		case 0:
			/*
			 * OSOS sub_B6620(port, 0) is
			 *     *(base + 8) |= 6;
			 * an OR, not an assignment. This wrote a bare 0x6 and
			 * so cleared every other bit in TXCOM. It happens to
			 * be equivalent while TXCOM reads 0 beforehand, which
			 * is the case today, but the moment anything else
			 * sets a bit here the plain write silently drops it.
			 */
			writel(readl(i2s->base + I2STXCOM) | I2STXCOM_DMA,
			       i2s->base + I2STXCOM);
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

	if (!i2s || !i2s->base || !dma_watch_ticks)
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
	if (i2s->watch_ticks < dma_watch_ticks)
		schedule_delayed_work(&i2s->dma_watch, msecs_to_jiffies(100));
}

static int s5l8740_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dai->dev);
	unsigned int rate = params_rate(params);
	unsigned int resolved = n31_resolve_rate(rate);
	const struct n31_rate_cfg *r = n31_find_rate(resolved);
	u32 div;
	int ret;

	if (!i2s || !i2s->base)
		return -ENODEV;
	/*
	 * Do not refuse an out-of-table rate: resolve it to the nearest
	 * supported one and let the codec SRC carry it. Refusing here while
	 * the codec silently fell back to 44.1 was how the two ends ended up
	 * disagreeing.
	 */
	if (resolved != rate)
		dev_warn(dai->dev, "rate %u unsupported, using %u (SRC)\n",
			 rate, resolved);
	/*
	 * Name whatever is driving the open/start cycle.
	 *
	 * The whole sequence -- hw_params, program, pinmux, DMA start, mute --
	 * repeats endlessly at boot, and every fix so far has been to a step
	 * inside it, which cannot stop something that keeps calling the cycle
	 * again. Individual steps are not the problem; the caller is.
	 *
	 * hw_params is the top of that cycle, so print a backtrace for the
	 * first few passes. Three is enough to see whether it arrives from a
	 * syscall (a process opening the PCM), from a kernel worker, or from
	 * the same place every time.
	 */
	/*
	 * One line, every time, naming who asked.
	 *
	 * This was three dump_stack() calls, which is the wrong shape for a
	 * device with no console: by the time anyone reads the screen the
	 * loop has run hundreds of times and those three multi-line traces
	 * are long gone off the top. current->comm and pid fit on one line
	 * and answer the only question that matters -- whether this arrives
	 * from a userspace process, and which, or from a kernel thread.
	 */
	if (hw_params_trace) {
		hw_params_seen++;
		dev_info(dai->dev, "hw_params #%u by %s[%d] rate=%u\n",
			 hw_params_seen, current->comm, current->pid,
			 params_rate(params));

		/*
		 * Stop the scroll and leave the answer on screen.
		 *
		 * This device has no console; the log is read off the display
		 * by eye. A loop that reopens the PCM hundreds of times makes
		 * every added line unreadable -- it scrolls past faster than
		 * anyone can parse, so more logging cannot help. Halting can:
		 * with panic=0 the machine stops with this as the last thing
		 * printed, and it names exactly who kept asking.
		 *
		 * Only fires when the loop is real. A handful of opens during
		 * a normal boot stays well under the limit.
		 */
		if (hw_params_loop_panic &&
		    hw_params_seen >= hw_params_loop_panic)
			panic("n31: PCM reopened %u times, last by %s[%d] rate=%u",
			      hw_params_seen, current->comm, current->pid,
			      params_rate(params));
	}

	ret = s5l8740_i2s_codec_prepare();
	if (ret && i2s->dev)
		dev_warn(i2s->dev, "codec prepare in hw_params: %d\n", ret);
	s5l8740_i2s_program(i2s, resolved);
	div = clkdiv ? clkdiv : (r ? r->clkdiv : 0);
	s5l8740_i2s_log_clocks(i2s, "hw_params");
	dev_info(dai->dev,
		 "IIS hw_params rate=%u resolved=%u code=%u clkdiv=%u dma=%d pio=%d txcom=%08x\n",
		 rate, resolved, r ? r->cs42_rate_code : 0, div, i2s->has_dma,
		 use_pio, readl(i2s->base + I2STXCOM));
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
		/*
		 * Stage markers, printed before each step. The trigger has
		 * hung here, and the DMA start log was the last thing on
		 * screen -- which tells us only that it got that far, not
		 * which of the following steps failed to return. Naming the
		 * step before running it answers that directly, on a device
		 * whose only diagnostic is what is left on the display.
		 */
		path_mode = s5l8740_i2s_audio_path_mode();
		dev_info(dai->dev, "trig: path_mode=%d\n", path_mode);
		if (path_mode == 1) {
			dev_info(dai->dev, "trig: codec_play_start(1)\n");
			s5l8740_i2s_codec_play_start();
		}
		dev_info(dai->dev, "trig: tx_kick\n");
		s5l8740_i2s_tx_kick(i2s, !use_pio);
		if (path_mode == 2) {
			dev_info(dai->dev, "trig: codec_play_start(2)\n");
			s5l8740_i2s_codec_play_start();
		}
		dev_info(dai->dev, "trig: log_clocks\n");
		s5l8740_i2s_log_clocks(i2s, "trigger_start");
		dev_info(dai->dev, "trig: schedule_asp\n");
		s5l8740_i2s_schedule_asp();
		dev_info(dai->dev, "trig: done\n");
		i2s->pio_run = use_pio;
		i2s->play_jiffies = jiffies;
		i2s->watch_ticks = 0;
		i2s->last_dma_src = 0;
		if (dma_watch_ticks)
			mod_delayed_work(system_wq, &i2s->dma_watch,
					 msecs_to_jiffies(100));
		dev_info_ratelimited(dai->dev,
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

	if (!debug_tone) {
		dev_info(dev, "debug tone disabled (set debug_tone=1)\n");
		return -EPERM;
	}

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

/*
 * Walking-bit oracle.
 *
 * A sine cannot tell you which bit lanes survive the trip to the codec,
 * because every wrong answer still looks like "quiet and dirty". This
 * plays a square wave built from one bit at a time -- +BIT(n), -BIT(n),
 * alternating -- so the analog result is a direct readout of that bit's
 * significance.
 *
 * A correct 16-bit path doubles the output for each step of n. Reading
 * the result:
 *
 *   only low bits audible    the codec is latching the wrong byte lane
 *   only high bits audible   samples are landing too far down the slot
 *   bit 15 not the loudest   sign or justification is wrong
 *   flat across all n        the link is not carrying sample data at all
 *
 * Square edges are deliberate: they survive whatever the analog stage
 * does far better than a low-amplitude sine, which matters when the
 * quantity being measured may be 40 dB down.
 *
 *   echo 9 > walk_bit    play ~1 s of +/-BIT(9)
 *   echo -1 > walk_bit   sweep 0..15, one second each, logging as it goes
 */
#define S5L8740_WALK_HZ		1000u

static int s5l8740_walk_one(struct s5l8740_i2s *i2s, int bit,
			    unsigned int ms)
{
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config cfg = { };
	struct dma_chan *chan;
	dma_addr_t dma;
	unsigned int rate, frames, half, i;
	size_t bytes;
	s16 *buf, hi, lo;
	int ret;

	if (bit < 0 || bit > 15)
		return -EINVAL;

	rate = i2s->rate ? i2s->rate : N31_RATE_DEFAULT;
	half = max(1u, rate / (2 * S5L8740_WALK_HZ));
	frames = half * 2;
	bytes = frames * 2 * sizeof(s16);

	/*
	 * Bit 15 is the sign bit, so the pair is 0 and -32768 rather than
	 * a symmetric swing; every other bit alternates about zero.
	 */
	hi = (bit == 15) ? 0 : (s16)(1 << bit);
	lo = (bit == 15) ? (s16)-32768 : (s16)-(1 << bit);

	mutex_lock(&i2s->dma_lock);
	chan = s5l8740_i2s_tx_get(i2s);
	if (IS_ERR_OR_NULL(chan)) {
		mutex_unlock(&i2s->dma_lock);
		return chan ? PTR_ERR(chan) : -ENODEV;
	}
	buf = dma_alloc_coherent(i2s->dev, bytes, &dma, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out_unlock;
	}
	for (i = 0; i < frames; i++) {
		s16 v = (i < half) ? hi : lo;

		buf[i * 2] = v;
		buf[i * 2 + 1] = v;
	}
	s5l8740_pcm_dump(i2s, buf, frames, "walk");

	cfg.direction = DMA_MEM_TO_DEV;
	cfg.dst_addr = i2s->play_dma.addr;
	cfg.dst_addr_width = (tone_width == 2) ?
		DMA_SLAVE_BUSWIDTH_2_BYTES : DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_maxburst = 1;
	ret = dmaengine_slave_config(chan, &cfg);
	if (ret)
		goto out_buf;

	s5l8740_i2s_codec_prepare();
	s5l8740_i2s_program(i2s, rate);
	desc = dmaengine_prep_dma_cyclic(chan, dma, bytes, bytes,
					 DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc) {
		ret = -ENOMEM;
		goto out_buf;
	}
	if (dma_submit_error(dmaengine_submit(desc))) {
		ret = -EIO;
		goto out_buf;
	}
	dma_async_issue_pending(chan);
	s5l8740_i2s_codec_play_start();
	s5l8740_i2s_tx_kick(i2s, !use_pio);
	s5l8740_i2s_schedule_asp();
	dev_info(i2s->dev, "walk bit %d: +%d/%d for %u ms\n",
		 bit, hi, lo, ms);
	msleep(ms);
	s5l8740_i2s_cancel_asp();
	s5l8740_i2s_codec_play_stop();
	dmaengine_terminate_sync(chan);
	s5l8740_i2s_hw_stop(i2s, NULL);
	ret = 0;
out_buf:
	dma_free_coherent(i2s->dev, bytes, buf, dma);
out_unlock:
	mutex_unlock(&i2s->dma_lock);
	return ret;
}

static ssize_t walk_bit_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dev);
	int bit, ret;

	if (!debug_tone) {
		dev_info(dev, "debug tone disabled (set debug_tone=1)\n");
		return -EPERM;
	}

	if (!i2s || !i2s->base)
		return -ENODEV;
	if (kstrtoint(buf, 0, &bit))
		return -EINVAL;

	if (bit >= 0) {
		ret = s5l8740_walk_one(i2s, bit, walk_ms);
		return ret ? ret : count;
	}
	for (bit = 0; bit < 16; bit++) {
		ret = s5l8740_walk_one(i2s, bit, walk_ms);
		if (ret) {
			dev_err(dev, "walk bit %d: %d\n", bit, ret);
			return ret;
		}
		msleep(walk_gap_ms);
	}
	return count;
}
static DEVICE_ATTR_WO(walk_bit);

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

	if (!debug_tone) {
		dev_info(dev, "debug tone disabled (set debug_tone=1)\n");
		return -EPERM;
	}

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
		s = s5l8740_sample_fix(s5l8740_scale_s16(n31_tone_s16(i, rate)));
		tone[i * 2] = s;
		tone[i * 2 + 1] = s;
	}
	s5l8740_pcm_dump(i2s, tone, frames, "dma_tone");
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
	WRITE_ONCE(s5l8740_i2s_iis0, i2s);
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
	ret = device_create_file(dev, &dev_attr_walk_bit);
	if (ret)
		dev_warn(dev, "walk_bit sysfs: %d\n", ret);
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

/*
 * IIS0 drives the codec through PL080. Left running across a kexec it
 * keeps fetching from a buffer the next kernel has reused, so the
 * handover is audible as well as unsafe. hw_stop is the same teardown
 * the STOP path uses -- TXCOM stop, DMA terminated, pads released.
 */
static void s5l8740_i2s_shutdown_pdev(struct platform_device *pdev)
{
	struct s5l8740_i2s *i2s = platform_get_drvdata(pdev);

	if (!i2s)
		return;
	i2s->pio_run = false;
	s5l8740_i2s_hw_stop(i2s, NULL);
	s5l8740_i2s_tx_put(i2s);
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
	.shutdown = s5l8740_i2s_shutdown_pdev,
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
		WRITE_ONCE(s5l8740_fm_gate_held, true);
		writel(CLKCON_FM_GATE_ON, iis2->clkcon + CLKCON_FM_GATE);
	} else if (iis2->fm_gate_held) {
		writel(iis2->fm_gate_saved ? iis2->fm_gate_saved :
					     CLKCON_FM_GATE_IDLE,
		       iis2->clkcon + CLKCON_FM_GATE);
		iis2->fm_gate_held = false;
		WRITE_ONCE(s5l8740_fm_gate_held, false);
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

/*
 * IIS2 captures FM over PL080 and holds the CLKCON+0x10 gate while it
 * does. Stopping it here also puts that gate back, so the next kernel
 * does not inherit a clock enabled by a driver that no longer exists.
 */
static void s5l8740_iis2_shutdown(struct platform_device *pdev)
{
	iis2_hw_stop(platform_get_drvdata(pdev));
}

static const struct of_device_id s5l8740_iis2_of_match[] = {
	{ .compatible = "apple,s5l8740-bcm2078-pcm" },
	{ .compatible = "apple,s5l8740-iis2" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8740_iis2_of_match);

static struct platform_driver s5l8740_iis2_driver = {
	.probe = s5l8740_iis2_probe,
	.shutdown = s5l8740_iis2_shutdown,
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
