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
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/apple-n31.h>

#include "n31-audio-rates.h"
#include "s5l8740-clk.h"

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
/*
 * TXCON, read out of sub_BCB60 (0x080BCB60).
 *
 * sub_BCB60 builds this register two ways, selected by its a3 argument,
 * and a3 also picks the pad function in s5l8740_i2s_pads():
 *
 *     a3 == 0:  r0 = (*txcon & 0x780) | 0x08000018;   pads function 2
 *     a3 != 0:  r0 = 0x00100098;                      pads function 3
 *     both:     r0 |= (a5 == 24) ? 0x03000041 : 0x03000001;
 *               *txcon = r0;  *(base + 0x30) = 0x1000;
 *
 * Which arm IIS0 takes is NOT statically known. sub_BCB60 has two callers:
 *
 *   0x0815DE20  passes a3 = 0 and a5 = 16 as immediates, but its own two
 *               callers pass unit 2 (0x08471CB0, followed by r7 = 32000)
 *               and unit 1 (0x08471DD0). It never runs for IIS0. The GPIO
 *               7 and 20 mux inside sub_BCB60 is guarded by
 *               sub_414FAA() == unit, and sub_414FAA returns 0, so those
 *               two pads are muxed only for unit 0 -- they are IIS0's, and
 *               the 97/98/119 that caller muxes are IIS1/IIS2's.
 *
 *   0x08414FC4  sub_414FAE, which loads all five arguments from bytes at
 *               [r4+21 .. r4+25]. This is the path IIS0 reaches, and its
 *               a3 is runtime data this image does not pin down.
 *
 * So the arm is chosen here on two pieces of evidence rather than on a
 * static read:
 *
 *   - Hardware. With bit 20 set (the a3 != 0 constant) the transmitter
 *     never starts: STATUS sits at 0x00000024 indefinitely, the TX FIFO
 *     fills to its eight entries, and the DMA stops with no error and no
 *     terminal count. Clearing it moves STATUS to 0x00008020 at once.
 *     Only the a3 == 0 arm produces a word without bit 20.
 *
 *   - The IIS2 oracle. The RetailOS MMIO capture of IIS2 during FM reads
 *     TXCON 0x0b000099 (IIS2_TXCON_FM below), which is exactly what the
 *     a3 == 0 arm yields with bit 7 already set. That confirms the arm's
 *     arithmetic, on a port we have a capture for.
 *
 * There is no IIS0 TXCON capture. If one is ever taken and it shows
 * 0x03100099, this and the pad function both belong on the a3 != 0 arm.
 */
#define I2STXCON_KEEP		0x780u		/* preserved across the RMW */
#define I2STXCON_A3_0		0x08000018u
#define I2STXCON_A3_NONZERO	0x00100098u
#define I2STXCON_WIDTH_16	0x03000001u
#define I2STXCON_WIDTH_24	0x03000041u
#define I2SRXCON_N31		0x1000u
/*
 * 0 follows sub_BCB60; anything else is written verbatim, which is what the
 * bring-up sweeps use.
 */
static uint txcon;
/* 0=soc_master, 1=ext_clock, 2=ext_clock with codec ASP bit7 cleared. */
/*
 * TEST 2026-09-01: defaulted to profile 1 to run the other coherent pairing.
 *
 * sub_BCB60 selects the pad function and the TXCON word from the same a3:
 *
 *	a3 == 0:  pads 7,20 -> fn 2,  TXCON = (old & 0x780) | 0x8000018 | 0x03000001
 *	a3 != 0:  pads 7,20 -> fn 3,  TXCON = 0x100098 | ...
 *
 * Both are stock configurations; which one the music path uses depends on
 * sub_BCB60's caller, and sub_414FAE has six callers that have not been
 * resolved. Profile 0 (fn 2) has been tested repeatedly and is silent.
 * Profile 1 (fn 3) has never been run -- and the one historical report of
 * audible, wrong-pitch output came from that combination, which would mean
 * signal actually reaching the amplifier.
 *
 * Revert to 0 if this is also silent; it is an A/B between two legitimate
 * stock pairings, not a claim that 1 is correct.
 */
/*
 * TXCON mastership arm. Default 1, which is what stock asks for.
 *
 * sub_414FAE calls sub_BCB60(cfg[21], cfg[22], cfg[23], cfg[24], cfg[25])
 * and sub_AA0AE sets cfg[23] = 1, so a3 = 1 is stock, giving TXCON
 * 0x03100099 and pads 7/20 at function 3.
 *
 * This defaulted to 0 on the strength of a measurement and a hypothesis,
 * and the hypothesis was wrong. The comment here used to say profile 1
 * could not move a sample "until that register is identified from the
 * decomp" -- meaning a codec master-clock enable. No such register exists.
 * A raw Thumb-BL scan of the whole image finds only three codec accessors
 * (sub_43CDB4, sub_43CDFA, sub_42A5D6) and their complete call set contains
 * no master/slave, clock-direction, port-format or frame-length write.
 *
 * The freeze that justified profile 0 -- one 32-byte burst into the TX
 * FIFO, src stuck at 0x09600020, hw_ptr never leaving 3 -- was measured
 * against a codec carrying two writes of our own invention: 0x0500 = 0x05,
 * which stock only writes in an override branch off the audio path, and a
 * clear of 0x000F bit 7, which stock sets once and never clears. Both are
 * gone, so that measurement no longer describes this driver.
 *
 * If profile 1 still freezes, the cause is NOT a missing codec master
 * enable, and looking for one again is a dead end.
 */
static uint mastership_profile = 1;
/* module_param moved below s5l8740_i2s_iis0; see txcon_set(). */
/*
 * D34C0 → 4F716(port, div). Table in n31-audio-rates.h.
 * 0 = 12 MHz / rate (272 @ 44.1 kHz RetailOS music).
 */

static uint clkdiv;
module_param(clkdiv, uint, 0644);
MODULE_PARM_DESC(clkdiv, "I2SCLKDIV override; 0 = OSOS table / 12000000/rate");
/*
 * Default IIS program rate for clk_run when no ALSA hw_params.
 * 0 = RetailOS 44100. ALSA playback uses the PCM rate, not this.
 */
static uint default_rate;
module_param(default_rate, uint, 0644);
MODULE_PARM_DESC(default_rate, "clk_run rate; 0 = 44100 OSOS default");
/*
 * dma_tone / pio_tone sample rate. 0 = default_rate / OSOS 44100.
 */

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
MODULE_PARM_DESC(hw_params_trace, "1=log caller comm/pid on each hw_params call (default Y)");
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
MODULE_PARM_DESC(sustain_ms, "ignore ALSA STOP for this many ms after START (default 0 = off)");

/*
 * Backtrace the first few PCM stops, to identify a caller that is stopping
 * a stream this driver did not stop itself. Off by default: dump_stack()
 * prints unconditionally, so this cannot live behind dynamic debug, and an
 * ordinary boot would spend a few hundred log lines on it.
 */
static bool trace_trigger;
module_param(trace_trigger, bool, 0644);
MODULE_PARM_DESC(trace_trigger,
		 "backtrace the first few PCM STOPs to name the caller (default N)");

/*
 * Off. Stock never writes the TX FIFO directly on the DMA path -- across the
 * whole image the only IIS0 offsets written are +0x00, +0x04, +0x08, +0x30,
 * +0x34, +0x3C and +0x40, and +0x10 is not among them. Priming the FIFO with
 * silence was our invention; it also made the DMA's first burst land on a
 * partly full FIFO, which confused the reading of how much it had moved.
 */
static uint fifo_prefill;
module_param(fifo_prefill, uint, 0644);
MODULE_PARM_DESC(fifo_prefill,
		 "silent stereo words to push into TX FIFO before TXCOM kick");
/*
 * OSOS B6620(port,0) does TXCOM |= 6 after PL080 is armed (peri 10).
 * RE body: sub_B6620 only ORs 0x6 — not 0xC. Hybrid 0xE was Linux invention.
 */
#define I2STXCOM_DMA	0x6
#define I2STXCOM_PIO	0xc
/*
 * Stopping the TX transport clears bit 2 ONLY -- it does not zero the
 * register.
 *
 * Stock arms and disarms with read-modify-writes, never plain stores:
 *
 *	arm    (sub_B6620, dir 0):  *(iis + 0x08) |= 6
 *	disarm (sub_5705DC, dir 0): *(iis + 0x08) &= ~4
 *
 * (The RX side is the same pair against +0x34.) The disarm deliberately
 * leaves bit 1 set; only bit 2 is dropped.
 *
 * This driver used to store 0 here, clearing bit 1 as well, on every stop --
 * including the stop that immediately precedes each arm. Whatever bit 1
 * latches, stock keeps it across the whole stop/start cycle and we were
 * destroying it every trigger.
 */
#define I2STXCOM_RUN	BIT(2)		/* the bit stock drops on disarm */
#define I2STXCOM_STOP	0x0		/* legacy; do not store this */
#define CLKCON_PHYS	0x3c500000ul
/* RetailOS oracle dwords at CLKCON+0x30 (music vs idle/A2DP). */
/* SRAM window: 0x22000000..0x2202FFFF, 192 KiB (bootloader descriptor). */
#define S5L8740_SRAM_BASE	0x22000000u
#define S5L8740_SRAM_END	0x22030000u
#define CLKCON_PWRCON1	0x4c	/* IIS0 gate bank; see s5l8740_i2s_iis0_gate() */
#define PWRCON1_IIS0_GATE	BIT(7)
#define CLKCON_AUDIO_OFF	0x30
/* Off. +0x30 is a PLL parameter; see s5l8740_audio_clk_set(). */
#define CLKCON_AUDIO_PLAY	0x32190u
#define CLKCON_AUDIO_IDLE	0x1c20u
/* FM additionally regates CLKCON+0x10; music/idle leaves it at 0x8004. */
#define CLKCON_FM_GATE		0x10
#define CLKCON_FM_GATE_ON	0x0u	/* bit 15 clear = on */
#define CLKCON_FM_GATE_IDLE	0x8000u	/* bit 15 set = gated */
/*
 * Bit 15 of CLKCON+0x10, and nothing else.
 *
 * This is stock's own gate, sub_41CBD8 case 11:
 *
 *   if (a2) MEMORY[0x3C500010] &= 0xFFFF7FFF;
 *   else    MEMORY[0x3C500010] |= 0x8000;
 *
 * so on clears bit 15 and off sets it. Every write to this register in the
 * firmware was checked: case 11 above, the divider setter, and the
 * save/restore critical section. None of them touches bit 2, which this
 * driver used to set from a value of its own -- that bit is not ours and
 * is not attested anywhere. CLKCON is shared with every other peripheral
 * on this SoC, so the rest of the word is not ours either.
 */
#define CLKCON_FM_GATE_MASK	0x8000u

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


/*
 * 1 = drive the DMA the way RetailOS does: one period-sized transfer at a
 * time, re-armed from the completion callback, instead of snd_dmaengine_pcm's
 * free-running cyclic chain. See s5l8740_rearm_submit().
 */
struct s5l8740_i2s;
static struct dma_chan *s5l8740_i2s_tx_get(struct s5l8740_i2s *i2s);
static int s5l8740_rearm_start(struct s5l8740_i2s *i2s);
static void s5l8740_rearm_stop(struct s5l8740_i2s *i2s);
static void s5l8740_rearm_sync(struct s5l8740_i2s *i2s);

/*
 * IIS0+0x44. Not written by stock; see s5l8740_i2s_program(). Non-zero
 * writes that value, 0 leaves the register alone.
 */
static uint reg44;
module_param(reg44, uint, 0644);
/*
 * Stock NEVER writes IIS0+0x44. All eight functions that resolve an IIS base
 * through sub_43A858 -- sub_4F6DC, sub_4F716, sub_B6620, sub_BCB60,
 * sub_C093C, sub_C095E, sub_C09AC, sub_5705DC -- contain no access to it at
 * all, read or write.
 *
 * A live read of RetailOS with music playing gives 0x00010007, so that is a
 * reset default or a bootloader leftover, not something the audio path
 * programs. Leaving it alone is therefore correct and is NOT a gap; the
 * register is logged next to the others so ours can be compared against
 * 0x00010007 rather than assumed.
 */
MODULE_PARM_DESC(reg44,
		 "IIS0+0x44 override; 0 = leave alone (default, and what stock does)");

/*
 * Milliseconds without a DMA completion before the re-arm path gives up and
 * xruns the stream. A transfer that never completes must not become an
 * unkillable writer blocked in snd_pcm_write: that wedges the PCM, makes
 * every later rmmod fail, and costs a reboot. 0 disables the guard.
 */
/*
 * Milliseconds the DMA source pointer may stand still during an active
 * stream before the driver terminates the transfer and xruns it. Applies to
 * both the cyclic and the re-arm paths, because it watches the controller
 * rather than either model's bookkeeping. 0 disables it.
 */

static uint rearm_stall_ms = 500;
module_param(rearm_stall_ms, uint, 0644);
MODULE_PARM_DESC(rearm_stall_ms,
		 "ms without a DMA completion before xrunning the stream (0=off)");

/*
 * 0 = snd_dmaengine_pcm's cyclic chain (default), 1 = the per-buffer re-arm
 * model stock uses.
 *
 * The re-arm path was written because stock re-arms per buffer and the
 * cyclic path was stalling. That stall turned out to be TXCON bit 20 -- the
 * serialiser was never clocking, so nothing could complete under either
 * model. Every cyclic measurement was taken against a dead transmitter and
 * none of them meant anything.
 *
 * With the transmitter fixed the standard path deserves the first try: it is
 * the one the rest of ALSA is written around. dma_rearm=1 still selects the
 * stock-shaped path, which remains the more faithful model if cyclic turns
 * out not to suit this controller.
 */
/*
 * Advertise the two extrapolated hi-res rates. Read-only at load time; see
 * s5l8740_rearm_open() and the table in n31-audio-rates.h for why this is
 * not on by default.
 */

static int dma_rearm = 1;
module_param(dma_rearm, int, 0644);
MODULE_PARM_DESC(dma_rearm,
		 "1 = per-buffer re-arm like stock (default), 0 = cyclic dmaengine");

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

/*
 * BCB60 sets DIR. Measured: with DIR left as an input, GPIO 7/20 stop
 * and GPIO 6 still toggles -- BCLK/LRCK are SoC-driven, not
 * codec-master.
 */

/*
 * Pad bring-up variants (exhaust RE before RetailOS GPIO oracle):
 *   0 = local 43D38C(7,3)(20,3) only [BCB60 — CONFIRMED OSOS body]
 *   1 = +43D38C(6,3) — NO OSOS 43D38C call site for GPIO6; debug only
 *   2 = gpio-s5l8740 s5l8740_iis0_pads_enable(3) — SEC pinmux + GPIOCMD
 *   3 = gpio driver mode 2 (BCB60 teardown)
 *   4 = SEC pinmux 6/7/20 + local mode3 on all three
 *   5 = mode2 + SEC pinmux refresh (func2 only, no mode3)
 */

struct s5l8740_i2s {
	void __iomem *base;
	void __iomem *clkcon;
	void __iomem *gpio;
	void __iomem *gpiocmd;
	struct device *dev;
	struct clk_bulk_data *clks;
	int num_clks;
	bool has_dma;
	/* Per-buffer re-arm playback; see s5l8740_rearm_submit(). */
	bool rearm_run;
	/*
	 * Stream generation. Incremented on every stop, stamped in when the
	 * DMA callback queues work, and compared in the handler.
	 *
	 * rearm_stop() cancels without waiting, because waiting there
	 * deadlocks against a closer holding the PCM stream lock. That
	 * leaves a queued item able to outlive its stream, and rearm_run
	 * alone cannot detect it: by the time the stale item runs, the next
	 * stream has set rearm_run back to true. It then reports a period
	 * the application never wrote, hw_ptr overtakes appl_ptr, and ALSA
	 * declares an xrun -- which stops the stream and queues another
	 * stale item, so the stream can never sustain.
	 */
	unsigned int rearm_gen;
	unsigned int rearm_gen_queued;
	/* Mirrors stock obj[0x0C]: cleared on channel (re)allocation, set
	 * once the self-linked node has been armed. */
	bool rearm_armed;
	snd_pcm_uframes_t rearm_pos;
	unsigned int rearm_periods;
	unsigned int rearm_underrun;
	struct work_struct rearm_work;
	struct delayed_work rearm_watch;
	/*
	 * Not system_wq. rearm_work runs snd_pcm_period_elapsed(), which is
	 * what tells ALSA a period is free again, and it has to happen well
	 * inside one period or the application never gets the space back in
	 * time. system_wq is shared with everything else on this single core
	 * -- the FTL in particular, which does long NAND scans -- so a period
	 * callback could sit behind unrelated work for tens of milliseconds
	 * and the stream would xrun with the FIFO still full. The DMA
	 * callback path already runs on its own WQ_HIGHPRI queue for the same
	 * reason; this is the other half of that path.
	 */
	struct workqueue_struct *rearm_wq;
	struct dma_chan *tx_chan;	/* cached — avoid dma:tx symlink churn */
	u8 tx_chan_borrowed;		/* 1 = lookup_peri, do not dma_release */
	struct mutex dma_lock;
	struct snd_dmaengine_dai_dma_data play_dma;
	struct snd_pcm_substream *ss;
	struct task_struct *kthread;
	bool pio_run;
	unsigned int pio_hw_ptr;
	unsigned int rate;
	bool programmed;
	unsigned long play_jiffies;
};

/*
 * SEC sub_2034 leftovers. OSOS 983430 never programs clock 9;
 * it does program clocks 6/20 into +0x1C after SEC. If U-Boot
 * zeroed the pair, IIS has no parent. Do not write +00/+04/+44.
 *
 * RetailOS music-playing oracle (checkpoint-010): +0x1C = 0xD0052003.
 * That is a sample of a live register, not a value stock ever stores:
 * only its bits 29:16 are ours, and only those are written.
 * Leaving the SEC bring-up value 0x10122003 yields IIS STATUS 0x82A0
 * and a silent jack even with TXCOM=6 / CS42 unmuted. The divider field
 * is what that turned out to need; the gate bits in the same word were
 * never ours, and writing them is what cost us storage.
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
 * The divider field of CLKCON+0x1C, and the only part of it that belongs to
 * audio. Bits 31:30 and 15:14 are clock-domain gates (sub_41CBD8 cases
 * 'E','F','G','H'), and bits 13:0 are a second domain's divider; none of
 * them are ours to write. Mirrors stock's own "& 0xC000FFFF" writer.
 */
#define CLKCON_1C_DIV_MASK	0x3FFF0000u

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
 * That paragraph was right, and the code under it was not, which is why
 * this kept coming back. The dangerous knob (force_stock_audio_parent)
 * defaulted off and carried this warning; meanwhile stock_clkcon_1c
 * defaulted ON and did the same class of blind whole-register write to
 * the same +0x1C. The comment claimed the branch below was
 * read-modify-write and left other blocks alone. It was not, and it did
 * not. Anyone who read the warning, checked that the warned-about
 * parameter was off, and moved on would conclude the driver was safe.
 *
 * Both are fixed now. The snapshot path is deleted outright -- there is
 * no safe version of it -- and +0x1C is a masked write of the audio
 * divider field only. See the note at the write site.
 */
/*
 * Write only CLKCON+0x1C, and nothing else.
 *
 * The whole-snapshot path that used to sit below is deleted. It pushed
 * a sampled copy into +0x08..0x1C, and those are SoC-wide gates:
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

/* sub_41CBD8(9,1): CLKCON+0x0C bit 15 clear = IIS0 CG16 on. */
static void s5l8740_i2s_ungate(struct s5l8740_i2s *i2s)
{
	u32 v, r18, r1c;

	if (!i2s->clkcon)
		return;
	r18 = readl(i2s->clkcon + 0x18);
	r1c = readl(i2s->clkcon + 0x1c);
	/*
	 * The whole-register CLKCON snapshot that used to live here is gone.
	 *
	 * force_stock_audio_parent stamped a sampled copy of CLKCON+0x08
	 * through +0x1C -- six SoC-wide clock registers -- straight over
	 * whatever every other peripheral had configured. It was sampled
	 * while RetailOS played music, so it carried the NAND, display and
	 * USB clock bits along with the audio ones and republished all of
	 * them. It is documented as destroying storage, and it does.
	 *
	 * There is no safe version of that write, so there is no parameter
	 * for it any more. What audio actually needs from this block is
	 * done below, one field at a time.
	 */
	{
		if (!r18)
			writel(SEC_CLKCON_18, i2s->clkcon + 0x18);
		if (stock_clkcon_1c) {
			/*
			 * Write the divider field ONLY. Never the whole word.
			 *
			 * This register was killing the NAND, and it did it by
			 * construction: STOCK_CLKCON_1C is a snapshot of the live
			 * register sampled while RetailOS played music, and a
			 * snapshot carries every other peripheral's bits in
			 * whatever state they happened to be in at that instant.
			 * Storing it whole republished all of them.
			 *
			 * Measured here: live 0x10122003 -> our 0xD0052003, so the
			 * top two bits go 00 -> 11. Per sub_41CBD8 cases 'E' and
			 * 'F' those two bits are clock-domain gates whose polarity
			 * is set-means-off ("if (a2) clear else set"), so the blind
			 * store gated off two domains, one of which the FMSS needs.
			 * That is the whole story of storage working at boot and
			 * dying with -EBUSY the moment audio ran: the CS sequencer
			 * lost its clock mid-flight and never went idle again,
			 * which is what fmss_cs_preflight() then refused on.
			 *
			 * Stock never leaves a whole-word value here. Every
			 * audio-path access is a masked read-modify-write, and the
			 * writer for this very field is
			 *
			 *   MEMORY[0x3C50001C] =
			 *       MEMORY[0x3C50001C] & 0xC000FFFF | ...
			 *
			 * preserving bits 31:30 and 15:0. The only whole-word store
			 * in the firmware saves the register first and restores it
			 * before returning.
			 *
			 * So this uses stock's own mask: bits 29:16, nothing else.
			 */
			u32 want = (r1c & ~CLKCON_1C_DIV_MASK) |
				   (STOCK_CLKCON_1C & CLKCON_1C_DIV_MASK);

			if (r1c != want) {
				writel(want, i2s->clkcon + 0x1c);
				dev_dbg(i2s->dev,
					 "CLKCON+1C %08x -> %08x (audio divider only, gates preserved)\n",
					 r1c, want);
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
 * CLKCON+0x30 is a PLL parameter, not an audio clock gate. Do not write it.
 *
 * This used to writel() CLKCON_AUDIO_PLAY (0x32190) here on every play and
 * CLKCON_AUDIO_IDLE (0x1c20) on every stop, on the reading that +0x30 was
 * the audio enable. It is not. The register is accessed exactly once in the
 * whole OSOS image, inside the PLL bring-up at sub_63B4:
 *
 *     if ((MEMORY[0x3C500044] & 1) == 0) {
 *         MEMORY[0x3C500044] &= ~0x10000u;
 *         MEMORY[0x3C500044] &= ~1u;
 *         MEMORY[0x3C500020] = <params from 0x8789194/98/9C>;
 *         MEMORY[0x3C500030] = 7200;
 *         MEMORY[0x3C500044] |= 0x10001u;
 *         while ((MEMORY[0x3C500040] & 1) == 0)
 *             ;
 *     }
 *
 * 7200 is 0x1C20 -- the value this driver called CLKCON_AUDIO_IDLE. So the
 * "idle" write happened to restore the correct PLL parameter, and the
 * "play" write put 0x32190 into it. Every playback corrupted the PLL and
 * every stop quietly repaired it, which is why the register always read
 * 0x1c20 whenever it was inspected at rest and nothing looked wrong.
 *
 * That is consistent with the whole symptom set: the IIS0 serialiser gets
 * no usable clock while playing, so the TX FIFO fills once and never
 * drains and the DMA stalls after topping it up; and the FMSS loses its
 * clock too, which is how the NAND died mid-session and stayed dead until
 * reboot.
 *
 * The real audio gate is sub_41CBD8, one bit per clock id -- id 9 is
 * CLKCON+0x0C bit 15 and is handled by s5l8740_codec_clk_gate().
 *
 * The wanted[] bookkeeping stays so the ports still declare intent and the
 * log still says who wanted the clock; it simply no longer writes.
 */
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
	spin_unlock_irqrestore(&s5l8740_audio_clk_lock, flags);

	/*
	 * Nothing is written to CLKCON+0x30 here, and there is no parameter
	 * to make it happen any more. That register is the audio PLL, it is
	 * shared, and writing it corrupted the PLL and took the NAND with
	 * it. The knob existed only to reproduce that on demand, which is
	 * not something this driver needs to be able to do. The port
	 * bookkeeping above is the entire job.
	 */
	(void)any;
}

static struct s5l8740_i2s *s5l8740_i2s_iis0;

/*
 * Apply TXCON to the live register as soon as it is written.
 *
 * Sweeping TXCON used to mean setting clkdiv to defeat the "already
 * programmed" check in s5l8740_i2s_program(), which re-ran the pad mux and
 * the CLKCON writes on every play. This writes IIS0+0x04 and nothing else,
 * so a sweep cannot disturb any clock.
 */
/* Resolve a coherent sub_BCB60 arm (or the explicit lab override). */
static u32 s5l8740_i2s_txcon_value(struct s5l8740_i2s *i2s)
{
	u32 forced = READ_ONCE(txcon);

	if (forced)
		return forced;
	if (READ_ONCE(mastership_profile))
		return I2STXCON_A3_NONZERO | I2STXCON_WIDTH_16;
	return (readl(i2s->base + I2STXCON) & I2STXCON_KEEP) |
	       I2STXCON_A3_0 | I2STXCON_WIDTH_16;
}

static int mastership_profile_set(const char *val,
				  const struct kernel_param *kp)
{
	struct s5l8740_i2s *i2s = READ_ONCE(s5l8740_i2s_iis0);
	unsigned int next;
	int ret;

	(void)kp;
	ret = kstrtouint(val, 0, &next);
	if (ret)
		return ret;
	if (next > 2)
		return -EINVAL;
	WRITE_ONCE(mastership_profile, next);
	if (i2s) {
		i2s->programmed = false;
		dev_dbg(i2s->dev,
			 "mastership_profile=%u (%s); applies on next program/start\n",
			 next, next == 0 ? "soc_master" :
			 next == 1 ? "ext_clock" : "ext_clock_slavecodec");
	}
	return 0;
}

static const struct kernel_param_ops mastership_profile_ops = {
	.set = mastership_profile_set,
	.get = param_get_uint,
};
module_param_cb(mastership_profile, &mastership_profile_ops,
		&mastership_profile, 0644);
MODULE_PARM_DESC(mastership_profile,
		 "0=soc_master (default), 1=ext_clock, 2=ext_clock with codec ASP bit7 clear");

static int txcon_set(const char *val, const struct kernel_param *kp)
{
	struct s5l8740_i2s *i2s = READ_ONCE(s5l8740_i2s_iis0);
	int ret = param_set_uint(val, kp);

	if (ret)
		return ret;
	if (i2s && i2s->base) {
		u32 v = s5l8740_i2s_txcon_value(i2s);

		writel(v, i2s->base + I2STXCON);
		dev_dbg(i2s->dev, "txcon live -> 0x%08x (status=0x%08x)\n",
			 v, readl(i2s->base + I2SSTATUS));
	}
	return 0;
}

static const struct kernel_param_ops txcon_ops = {
	.set = txcon_set,
	.get = param_get_uint,
};
module_param_cb(txcon, &txcon_ops, &txcon, 0644);
MODULE_PARM_DESC(txcon, "I2STXCON override; 0 (default) follows sub_BCB60");

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
		v &= ~CG16_DISABLE_BIT;
	else
		v |= CG16_DISABLE_BIT;
	writel(v, i2s->clkcon + 0x0c);
	spin_unlock_irqrestore(&s5l8740_audio_clk_lock, flags);
	dev_dbg(i2s->dev, "codec clk gate %s (CLKCON+0x0C=0x%08x)\n",
		 on ? "ON" : "OFF", v);
}
EXPORT_SYMBOL_GPL(s5l8740_codec_clk_gate);

/*
 * Codec clock divider — RetailOS sub_345D28(9, 0, div).
 *
 * The C export shows this as a bare sub_345D28() with no arguments, which is
 * why it was read as a delay for so long. The arguments are visible in the
 * disassembly at 0x080D32BE: clock id 9 (the same id s5l8740_codec_clk_gate
 * gates), selector 0, and a divider taken from the MCLK word --
 * 2 when it reads 12000, otherwise 4.
 *
 * The body is a ROM thunk into IRAM at 0x22000930, which sub_434() copies
 * from ROM. Its clock-9 case is:
 *
 *	r6 = 0x3C500000				(CLKCON)
 *	r2 = CLKCON[0x0C]
 *	bfc r2, #0, #15				clear bits 14:0
 *	(selector 0 adds no source bits)
 *	if (div != 1 && div <= 17) r2 |= (div - 1) & 0xF
 *	CLKCON[0x0C] = r2
 *
 * So bits 3:0 of CLKCON+0x0C are the codec clock divider, held as div-1,
 * bits 13:12 are a source select, and bit 15 is the gate. Stock therefore
 * runs the codec clock at source/2, and a register left at zero runs it at
 * source/1 -- twice stock's rate.
 *
 * Bit 15 is preserved so this composes with the gate in either order.
 */
void s5l8740_codec_clk_divider(unsigned int div)
{
	struct s5l8740_i2s *i2s = READ_ONCE(s5l8740_i2s_iis0);
	unsigned long flags;
	unsigned int i;
	u32 v;

	if (!i2s || !i2s->clkcon)
		return;
	if (!div || div > 17)
		return;

	/*
	 * CLKCON+0x0C is CG16_AUD0, a 16-bit clock-gate register; CG16_AUD1
	 * sits at +0x0E and therefore occupies bits 31:16 of the same 32-bit
	 * word (Rockbox s5l87xx.h). The output is
	 *
	 *	!DISABLE * SEL_freq / (DIV1 + 1) / (DIV2 + 1)
	 *
	 *	bit 15		DISABLE, set masks the clock -- the "gate"
	 *			owned by s5l8740_codec_clk_gate()
	 *	bits 13:12	SEL, source select (0 = OSC, 1..3 = PLL0..2)
	 *	bits 7:4	DIV2
	 *	bits 3:0	DIV1
	 *
	 * This is exactly the layout the ROM routine drives: its clock-9 case
	 * clears bits 14:0 and ORs in (div - 1), and its selector argument
	 * contributes 0x1000/0x2000/0x3000 -- the SEL field.
	 *
	 * Bits 14:0 are all AUD0's own fields, so clearing them is safe and
	 * reproduces the ROM exactly. What must be preserved is bit 15, which
	 * belongs to the gate, and bits 31:16, which are CG16_AUD1 -- another
	 * I2S clock. Never write this register as a whole word.
	 */
	spin_lock_irqsave(&s5l8740_audio_clk_lock, flags);
	v = readl(i2s->clkcon + 0x0c);
	v &= ~CG16_FIELDS;
	if (div != 1)
		v |= ((div - 1) & CG16_DIV1_MSK) << CG16_DIV1_POS;
	writel(v, i2s->clkcon + 0x0c);

	/*
	 * Wait for the divider to take before returning.
	 *
	 * The bootloader's sub_14FC does exactly this, and the wait is the
	 * part that was missing here:
	 *
	 *	v1 = (read(0x3C50000C) >> 15 << 15) | 1;
	 *	write(0x3C50000C, v1);
	 *	while (read(0x3C50000C) != v1)   ;
	 *	clear bit 15                        (release the gate)
	 *
	 * It writes, spins until the register reads back what it wrote, and
	 * only then ungates. This driver wrote and returned, leaving the
	 * caller free to release the gate immediately -- so the codec could
	 * be handed a clock whose divider had not settled.
	 *
	 * Bounded, because a spin with no escape in a driver is not the same
	 * proposition as one in a bootloader that owns the machine.
	 */
	for (i = 0; i < 10000; i++) {
		if (readl(i2s->clkcon + 0x0c) == v)
			break;
		cpu_relax();
	}
	spin_unlock_irqrestore(&s5l8740_audio_clk_lock, flags);

	if (i == 10000)
		dev_warn(i2s->dev,
			 "codec clk divider did not read back (want 0x%08x, got 0x%08x)\n",
			 v, readl(i2s->clkcon + 0x0c));
	dev_dbg(i2s->dev, "codec clk CG16_AUD0 DIV1=/%u (CLKCON+0x0C=0x%08x)\n",
		 div, v);
}
EXPORT_SYMBOL_GPL(s5l8740_codec_clk_divider);

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
static void s5l8740_i2s_iis0_gate(struct s5l8740_i2s *i2s, bool on);

static void s5l8740_i2s_hw_stop(struct s5l8740_i2s *i2s,
				struct snd_pcm_substream *substream)
{
	struct dma_chan *chan = NULL;

	if (!i2s || !i2s->base)
		return;

	/* Stock's stop writes are below, after the DMA is quiesced. */

	/*
	 * Only ask dmaengine_pcm for the channel when dmaengine_pcm is
	 * actually registered. The re-arm path deliberately does not
	 * register it -- it owns the PCM itself -- so
	 * substream->runtime->private_data is not a
	 * dmaengine_pcm_runtime_data and snd_dmaengine_pcm_get_chan()
	 * dereferences NULL:
	 *
	 *   Unable to handle kernel NULL pointer dereference at 00000000
	 *   PC is at snd_dmaengine_pcm_get_chan+0x8/0x10
	 *   LR is at s5l8740_i2s_trigger+0x1f8
	 *
	 * i2s->tx_chan is the channel on that path anyway.
	 */
	if (substream && i2s->has_dma && !use_pio && !dma_rearm)
		chan = snd_dmaengine_pcm_get_chan(substream);
	if (!chan)
		chan = i2s->tx_chan;
	if (chan) {
		/*
		 * _async, not _sync. This runs from the DAI trigger, and the
		 * trigger is reachable from inside the DMA period callback --
		 * an xrun in snd_pcm_period_elapsed() calls snd_pcm_do_stop()
		 * which lands right here. The sync variant waits for that
		 * callback to finish, from inside the callback.
		 *
		 * Nothing is lost by not waiting. terminate_all stops the
		 * channel synchronously in the controller, with a bounded
		 * poll, so the hardware is already quiet before the clock
		 * gating below; the sync half only waits on callback
		 * delivery, and ALSA does that properly through sync_stop.
		 */
		dmaengine_terminate_async(chan);
	}

	/*
	 * No teardown. Stock does not tear the transmitter down on stop.
	 *
	 * MEASURED on RetailOS. After a music session was stopped and the
	 * FM radio entered and stopped, with the screen off, IIS0 still read:
	 *
	 *	CLKCON 00000001   TXCON  03100099   TXCOM 00000006
	 *	RXCON  00001000   RXCOM  00000000   REG44 00010007
	 *
	 * every one identical to the values captured while it was playing.
	 * PL080 ch2 was still enabled at the same time -- EnbldChns 4,
	 * Config 1, cfg 00028a81, ctl 84249000 -- with only src and the
	 * count at different positions. PWRCON1 bit 7 was still CLEAR, so
	 * the clock had not been re-gated either.
	 *
	 * So sub_C09AC(iis, 0) exists in the image but is not what a normal
	 * stop runs. This function used to transliterate it faithfully --
	 * TXCOM=0, RXCOM=0, CLKCON=0, spin for the stop-ack, re-gate -- and
	 * that was a faithful copy of a path stock does not take.
	 *
	 * It also broke the start side by implication: our start ran against
	 * a fully torn-down block every time, while stock starts against one
	 * that is already configured, enabled and clocked. Same start code,
	 * different initial state.
	 *
	 * The DMA is still terminated above, because ALSA owns that buffer
	 * and must not have it read after the stream goes away. Everything
	 * the I2S block holds is left exactly as stock leaves it.
	 */
}


static void s5l8740_i2s_gpiocmd(struct s5l8740_i2s *i2s, unsigned int gpio, u8 cmd)
{
	unsigned int bank = gpio >> 3;
	unsigned int pin = gpio & 7;
	int (*dir_set)(unsigned int, bool);
	void __iomem *b;

	if (!i2s->gpio || !i2s->gpiocmd)
		return;
	/*
	 * The direction bit goes through the gpio driver, which owns this
	 * register and takes a lock across it.
	 *
	 * DIR is per-bank: eight pads in one word. This used to be an
	 * unlocked read-modify-write here while gpio-s5l8740.c did its own
	 * unlocked read-modify-write on the same word, so either could drop
	 * a bit the other had just set. Two owners of one register is the
	 * bug; doing our half under a lock the other owner does not take
	 * would not have fixed it.
	 *
	 * Only the direction is delegated. GPIOCMD below is a separate
	 * write-only register and this driver is its only user.
	 */
	dir_set = (int (*)(unsigned int, bool))
		  __symbol_get("s5l8740_gpio_dir_set");
	if (dir_set) {
		dir_set(gpio, true);
		__symbol_put("s5l8740_gpio_dir_set");
	} else {
		dev_warn_once(i2s->dev,
			      "gpio driver absent; DIR left as found for %u\n",
			      gpio);
	}

	b = i2s->gpio + 32 * bank;
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
		dev_dbg(i2s->dev, "%s pads (no gpio export)\n", tag);
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
 * and the jack silent.
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

static void s5l8740_i2s_pads(struct s5l8740_i2s *i2s)
{
	unsigned int func = READ_ONCE(mastership_profile) ? 3 : 2;

	/*
		 * GPIO 7 and 20, with their function selected by the same profile
		 * that selects TXCON.
	 *
	 * sub_BCB60 muxes these two pads itself, guarded by
	 * sub_414FAA() == unit -- and sub_414FAA is "movs r0, #0; bx lr", so
	 * the guard passes only for unit 0. These are IIS0's pads; the 97, 98
	 * and 119 muxed by sub_BCB60's other caller belong to IIS1 and IIS2.
	 *
	 * The function comes from the same a3 that selects the TXCON word:
	 *
	 *   80bcb8c  cbz  r7, 80bcbba      a3 == 0 ?
	 *   80bcb96  movs r1, #3           a3 != 0: sub_43D38C(20, 3, 0)
	 *   80bcba2  movs r1, #3                    sub_43D38C(7,  3, 0)
	 *   80bcbc4  movs r1, #2           a3 == 0: sub_43D38C(20, 2, 0)
	 *   80bcbce  movs r1, #2                    sub_43D38C(7,  2, 0)
	 *
	 * sub_43D38C is the pad-function call: its bank base is
	 * 32*(gpio>>3) + 1022361600, and 1022361600 is 0x3CF00000.
	 *
		 * The old default split the two halves: function 3 (a3 != 0) with the
		 * a3 == 0 TXCON. They are kept together here. Profile 1 restores the
		 * coherent a3 != 0 combination from the only historical warm-boot run
		 * in which an audible, wrong-pitch tone was reported. That result does
		 * not establish a cold-boot default, so profile 0 remains the default
		 * until the on-device matrix is captured.
		 *
		 * This is a controlled A/B mechanism, not a claim that either profile
		 * is correct before the cold-boot evidence exists.
	 */
	s5l8740_i2s_gpiocmd(i2s, 7, func);
	s5l8740_i2s_gpiocmd(i2s, 20, func);
	s5l8740_i2s_log_iis_gpio(i2s, "pads-applied");
	dev_dbg(i2s->dev, "mastership_profile=%u pads 7/20 func=%u\n",
		 mastership_profile, func);
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
	u32 status = 0, clkdiv = 0, reg44v = 0, pw1 = 0;

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
		reg44v = readl(i2s->base + I2SREG44);
	}
	if (i2s->clkcon)
		pw1 = readl(i2s->clkcon + CLKCON_PWRCON1);
	dev_dbg(i2s->dev,
		 "%s CLKCON+0C=%08x +18=%08x +1C=%08x PWRCON1=%08x IIS0 +00=%08x +04=%08x +08=%08x +30=%08x +34=%08x +3C=%08x +40=%08x +44=%08x\n",
		 tag, c0c, c18, c1c, pw1, clkcon, txcon, txcom, rxcon, rxcom,
		 status, clkdiv, reg44v);
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

static void s5l8740_i2s_set_codec_clock_role(struct s5l8740_i2s *i2s)
{
	int (*set_role)(bool);
	unsigned int profile = READ_ONCE(mastership_profile);
	bool asp_bit7 = profile != 2;
	int ret = -ENOENT;

	set_role = (int (*)(bool))__symbol_get("cs42l81_set_clock_role");
	if (set_role) {
		ret = set_role(asp_bit7);
		__symbol_put("cs42l81_set_clock_role");
	}
	dev_dbg(i2s->dev,
		 "mastership_profile=%u codec ASP_BIT7=%d ret=%d\n",
		 profile, asp_bit7, ret);
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
	dev_dbg(dev,
		 "%s txcon=0x%08x %s%s%s%s\n",
		 tag, v,
		 (v & 0x40u) ? "24bit " : "16bit ",
		 (v & I2STXCON_A3_0) == I2STXCON_A3_0 ? "BCB60-a3zero " : "",
		 (v & 0x00100000u) ? "bit20-a3nonzero " : "",
		 v == 0x0B100019u ? "ROCKBOX-POISON " : "");
}

/*
 * sub_C09AC(port, 1) is CLKCON = 1, but it is preceded by
 * sub_345D70(0x26, 0, 1), and what that call does is NOT established.
 *
 * This comment used to assert "345D70 is JUMPOUT 0x22000350 = bootloader
 * sub_350 (SCTLR C-bit)". The thunk half is right and the conclusion is
 * wrong. sub_345D70 is "ldr pc, [pc, #-4]" loading 0x22000351 -- bit 0 set,
 * so an interworking branch to THUMB at 0x22000350. In the bootloader image
 * we have on disk, file offset 0x350 is ARM:
 *
 *	0x350: mov r1,#4 / mrc p15,0,r0,c1,c0,0 / orr / mcr / bx lr
 *	0x364: same with bic          0x378: mov r1,#0x1000
 *
 * i.e. SCTLR C set, C cleared, I set. Two things rule that out as the
 * callee:
 *
 *   - A Thumb target cannot be that ARM code. Every sibling thunk in the
 *     same block has the Thumb bit set and lands on an ARM word too:
 *     345D28 -> 0x22000931 (e3a000d3), 345D40 -> 0x220002B3 (mid-word),
 *     345D48 -> 0x22001023, 345D58 -> 0x2200104B, 345CE8 -> 0x220011E5.
 *     The SRAM resident at 0x22000000 while the OS runs is not the
 *     bootloader image on disk; 0x350 merely happens to look decodable.
 *
 *   - An SCTLR cache-bit setter takes no arguments. sub_C09AC passes
 *     (0x26, 0, 1) and sub_11B70 passes (0x1E, 0, 1) before it touches any
 *     SPI2 register. Two unrelated callers supplying what reads as
 *     (peripheral id, ?, on) is not a cache toggle.
 *
 * So from usage it is a per-peripheral gate keyed by id, and the id is
 * selected by port. In sub_C09AC the selection is, from the raw image:
 *
 *	0x0C09CC  cbz  r2, 0x0C09DA      ; port 0
 *	0x0C09CE  cmp  r2, #1
 *	0x0C09D0  beq  0x0C09DE
 *	0x0C09D2  cmp  r2, #2
 *	0x0C09D4  beq  0x0C09F0
 *	0x0C09DA  movs r0, #0x26         ; IIS0  <-- our port
 *	0x0C09DE  movs r0, #0x23         ; IIS1
 *	0x0C09E0  movs r1, #0
 *	0x0C09E4  movs r2, #1            ; 0 on the disable path at 0x0C0A06
 *	0x0C09E6  blx  0x00345D70
 *
 * so IIS0 enable is sub_345D70(0x26, 0, 1). sub_11B70 does the same shape
 * for SPI with ids 0x22 and 0x2b (0x011BD8 / 0x011BDE), not 0x1E -- an
 * earlier note here had both SPI ids wrong.
 *
 * Note these are BLX, not BL: sub_345D70 is ARM. A BL scan finds nothing,
 * which is how the call sites got mis-reported once already.
 *
 * It is NOT sub_41CBD8/id 9. That one is CLKCON+0x0C bit 15, where CLEAR
 * means the IIS0 clock is on, and we already drive it via
 * s5l8740_codec_clk_gate(); measured 0x00000001 at trigger, i.e. correct.
 *
 * WE DO NOT ASSERT THE 0x26 GATE AT ALL. s5l8740_i2s_c09ac_start() below
 * writes CLKCON = 1 and nothing else, so this step of sub_C09AC has no
 * counterpart here.
 *
 * This matters: if the 0x26 gate is something the IIS needs and we never
 * assert an equivalent, that is a live candidate for the transmitter not
 * clocking. Do not resolve it by sweeping CLKCON.
 *
 * Play 414FAE only starts -- it does not C09AC-stop first.
 * RetailOS music: I2SCLKCON = 0x1 (not 0x2 stop-ack).
 */
/*
 * IIS0 peripheral clock gate -- CLKCON+0x4C (PWRCON1) bit 7.
 *
 * MEASURED ON HARDWARE, RetailOS 1.0.2, with music playing and the output
 * captured on the USB mixer at -21 dBFS peak / -37.6 dBFS RMS, 92 of 99
 * 100 ms windows active, so the transmitter was demonstrably producing sound
 * at the moment these registers were read:
 *
 *	PWRCON1 idle    0xef226fe3      bit 7 SET   -- IIS0 gated off
 *	PWRCON1 playing 0xef226f23      bit 7 CLEAR -- IIS0 ungated
 *
 * The route to that bit: sub_C09AC(port, 1) calls sub_345D70(id, 0, 1) BEFORE
 * writing I2SCLKCON, and picks the id by port -- cbz r2 at 0x0C09CC sends
 * port 0 to "movs r0, #0x26". sub_345D70 lives in resident SRAM (not the
 * bootloader image on disk, which decodes to something else entirely at the
 * same address); dumped over untethered SCSI it is a gate keyed by id:
 *
 *	entry = 0x08917860 + id * 32
 *	[entry+0x04] -> mask for CLKCON+0x48    [entry+0x10] -> +0x68
 *	[entry+0x08] -> mask for CLKCON+0x4C    [entry+0x14] -> +0x6C
 *	[entry+0x0C] -> mask for CLKCON+0x58
 *	cbz r4 -> orrs, else bics, so on == 1 CLEARS the bit
 *
 * Live table entry 0x26 reads "26 00 00 00 | 80 00 00 00" -- id in field 0,
 * then mask 0x80 in the +0x08 slot, i.e. PWRCON1 bit 7. Every bit music
 * cleared is accounted for by an entry: 0x26 and 0x27 in PWRCON1, and 0x3C,
 * 0x3D, 0x3E in PWRCON3.
 *
 * Why this is the whole failure: with the gate set, the register interface is
 * still clocked, so every IIS0 register we program reads back correct and
 * identical to stock's working set -- CLKCON=1, TXCON=0x03100099, TXCOM=6,
 * RXCON=0x1000, CLKDIV=0x110. The serialiser is not clocked, so it neither
 * drains the TX FIFO nor raises an underrun; the DMA halts after its first
 * 32-byte burst, terminal count never arrives, and the PL080 interrupt never
 * fires. That is exactly what we measured.
 *
 * Strict single-bit read-modify-write. PWRCON1 packs 32 unrelated
 * peripherals and a set bit gates one OFF, so writing anything wider here
 * would switch off blocks nothing has claimed. Do not "simplify" this into a
 * whole-register write.
 *
 * Only IIS0 is handled. sub_C09AC sends port 1 to id 0x23 and port 2 to the
 * arm at 0x0C09F0, which is not decoded, so those are left alone.
 */

static void s5l8740_i2s_iis0_gate(struct s5l8740_i2s *i2s, bool on)
{
	/*
	 * Deliberately does nothing. PWRCON1 bit 7 has an owner.
	 *
	 * This briefly did a direct read-modify-write here, on the strength
	 * of the RetailOS measurement (PWRCON1 0xef226fe3 idle ->
	 * 0xef226f23 playing). The bit and the polarity were right and it
	 * changed nothing: still 0 periods, still no PL080 interrupt.
	 *
	 * It was also the wrong way to do it. clk-s5l8702.c already
	 * registers that exact bit as a clock:
	 *
	 *	[CLK_I2S0] = GATE("i2s0", NULL, CLKCON_PWRCON1, 7)
	 *
	 * so a raw write here gives one bit two owners and fights the clock
	 * framework. The real defect was in the devicetree: i2s@3ca00000 had
	 * no clocks property at all, so devm_clk_bulk_get_all() returned
	 * zero clocks and clk_bulk_prepare_enable() had nothing to enable.
	 * The nodrm variant of the DTS already carried the property; the
	 * main one did not. It does now, naming CLK_I2S0 and CLK_CG16_9.
	 *
	 * Kept as a stub rather than deleted so the call sites still mark
	 * where sub_C09AC gates and ungates, and so nobody re-adds the raw
	 * write when the next gate turns up.
	 */
	(void)i2s;
	(void)on;
}

static void s5l8740_i2s_c09ac_start(struct s5l8740_i2s *i2s)
{
	/* sub_C09AC order: gate first, then I2SCLKCON. */
	s5l8740_i2s_iis0_gate(i2s, true);
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
		dev_dbg(i2s->dev, "STATUS W1C tx sticky %08x->%08x\n",
			 before, after);
}

/* 26DDDE: 41CBD8(9,1), 5705DC RX, 414FAE (C09AC + BCB60), D34C0 CLKDIV. */
static void s5l8740_i2s_program(struct s5l8740_i2s *i2s, unsigned int rate)
{
	const struct n31_rate_cfg *r;
	u32 div;
	u32 rxcom;
	u32 txcon_v;

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
	s5l8740_i2s_set_codec_clock_role(i2s);
	s5l8740_i2s_pads(i2s);
	/* Coherent sub_BCB60 profile, a5 = 16; +0x30 = 0x1000. */
	txcon_v = s5l8740_i2s_txcon_value(i2s);
	writel(txcon_v, i2s->base + I2STXCON);
	if (i2s->dev)
		s5l8740_i2s_log_txcon(i2s->dev, txcon_v, "program");
	writel(I2SRXCON_N31, i2s->base + I2SRXCON);
	rxcom = readl(i2s->base + I2SRXCOM);
	writel(rxcom & ~4u, i2s->base + I2SRXCOM);
	writel(div, i2s->base + I2SCLKDIV);
	/*
	 * IIS0+0x44 is not written by stock. The 0x00010007 here came from a
	 * register *readback* taken while RetailOS was playing, not from any
	 * write in the image: enumerating every function that reaches the IIS
	 * base through sub_43A858 gives offsets +0x00, +0x04, +0x08, +0x30,
	 * +0x34, +0x3C and +0x40, and nothing else. Whatever put 0x00010007
	 * there did it from somewhere we have not identified, so writing it
	 * ourselves is a guess about a register we cannot name.
	 *
	 * reg44 restores the write for comparison.
	 */
	if (reg44)
		writel((u32)reg44, i2s->base + I2SREG44);
	/*
	 * Setup only. TXCOM stays 0 until .trigger START (OSOS sub_B6620).
	 *
	 * A previous attempt left 0xC here so that bit 3 would be set before
	 * dmaengine arms the channel, on the strength of a "bit 3 has to be
	 * set before the DMA kick, or STATUS sticks at 0x24" note. That note
	 * is this driver's own, and the Rockbox N7G port inherited it from
	 * here -- so reading it back out of Rockbox was not corroboration.
	 *
	 * The image disagrees. TXCOM is written by exactly two functions in
	 * the whole of OSOS: sub_B6620 ORs 6 into it (and 6 into RXCOM at
	 * +52 on the capture side), and sub_C09AC writes 0 on stop. Bit 3 is
	 * never set anywhere. Whatever makes the serialiser start, it is not
	 * that.
	 *
	 * txcom_mode=2 still sets bit 3 for anyone who wants to retry it as a
	 * deliberate experiment.
	 */
	/* sub_5705DC: clear bit 2, keep the rest. */
	writel(readl(i2s->base + I2STXCOM) & ~I2STXCOM_RUN,
	       i2s->base + I2STXCOM);
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
			 * sub_B6620(port, 0) is *(base + 8) |= 6 -- an OR, and
			 * it stays an OR. s5l8740_i2s_program() now leaves
			 * 0xC here, so this lands on 0xE and bit 3 stays set
			 * through playback rather than being dropped.
			 *
			 * The read is kept because the stock start path does
			 * one, and a read-then-write is not the same bus
			 * activity as a bare write on this block.
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
		dev_dbg(i2s->dev,
			 "tx_kick dma=%d mode=%d txcom %08x->%08x status=%08x clkcon=%08x\n",
			 dma, txcom_mode, before, after,
			 readl(i2s->base + I2SSTATUS),
			 readl(i2s->base + I2SCLKCON));
}

/*
 * s5l8740_i2s_dma_watch() is gone.
 *
 * It sampled the PL080 source pointer every 100 ms and xrun-ed the stream
 * after dma_stall_ms of no movement. Stock has no counterpart: sub_43A858,
 * the only IIS base resolver, is called from exactly six functions and every
 * one is a configuration, enable, disable or status call on a control path.
 * Nothing in the image reads Cx_SrcAddr or Cx_DstAddr for progress, and there
 * are exactly two audio threads -- MeCCAInputTask and MeCCAOutputTask --
 * with no timer or monitor between them.
 *
 * Stock's only bound on the DMA is the per-submit semaphore timeout inside
 * sub_B65F4 (10000 ticks, returning 31), and sub_BFA50 at EA 0x0BFA6E
 * discards that return entirely -- a DMA timeout in the audio path is
 * invisible and the task simply carries on to the next period.
 *
 * A kernel cannot copy that: stock's producer IS the RTOS task, while ours
 * has an ALSA writer blocked in snd_pcm_write() that would never be released.
 * s5l8740_rearm_watchfn() is now the single bound, and it sits at the same
 * point in the sequence as sub_B65F4 -- one deadline per submitted period.
 * Its value is OURS; the tick period behind stock's 10000 is not established.
 */

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

	/*
	 * Name who is driving the trigger, and from where.
	 *
	 * The stream start/stops about nine times a second and never moves a
	 * sample: DAI trigger START, then STOP under a millisecond later,
	 * then START again. The 110 ms between cycles is just the codec
	 * rate-change settle inside codec_play_start, so the loop is as
	 * tight as the hardware allows. Nothing in this driver asks for that
	 * STOP, and the re-arm engine is the only bound left, so the caller
	 * is upstream of us and this is the only way to see it.
	 *
	 * Backtrace on the first few STOPs only. A trace every 110 ms would
	 * push itself out of the ring buffer before anyone could read it,
	 * which is exactly what happened to the last set of dump_stack()
	 * calls put in the hw_params path.
	 */
	{
		static unsigned int trig_seen;
		struct snd_pcm_runtime *rt = substream->runtime;

		if (trig_seen < 24) {
			trig_seen++;
			dev_dbg(dai->dev,
				 "trigger cmd=%d by %s[%d] state=%d hw_ptr=%lu appl_ptr=%lu%s",
				 cmd, current->comm, current->pid,
				 rt ? (int)rt->status->state : -1,
				 rt ? (unsigned long)rt->status->hw_ptr : 0UL,
				 rt ? (unsigned long)rt->control->appl_ptr : 0UL,
				 "\n");
			/*
			 * dump_stack() prints unconditionally, so dynamic
			 * debug cannot gate it and an ordinary stream stop
			 * would emit around two hundred lines per boot.
			 * Behind a parameter instead: set trace_trigger=1 to
			 * identify a caller that is stopping the stream.
			 */
			if (trace_trigger &&
			    cmd == SNDRV_PCM_TRIGGER_STOP && trig_seen < 8)
				dump_stack();
		}
	}

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
		/*
		 * dmaengine_pcm owns the substream on the cyclic path, so the
		 * stall guard would otherwise have nothing to xrun. The DAI
		 * trigger sees it under both models.
		 */
		WRITE_ONCE(i2s->ss, substream);
		dev_dbg(dai->dev, "trig: path_mode=%d\n", path_mode);
		if (path_mode == 1) {
			dev_dbg(dai->dev, "trig: codec_play_start(1)\n");
			s5l8740_i2s_codec_play_start();
		}
		if (dma_rearm && !use_pio) {
			/*
			 * Stock submits the first buffer and only then does
			 * TXCOM |= 6. The dmaengine path would have armed the
			 * channel before this callback ever ran.
			 */
			int rret = s5l8740_rearm_start(i2s);

			dev_dbg(dai->dev, "trig: rearm_start ret=%d\n", rret);
		}
		dev_dbg(dai->dev, "trig: tx_kick\n");
		s5l8740_i2s_tx_kick(i2s, !use_pio);
		if (path_mode == 2) {
			dev_dbg(dai->dev, "trig: codec_play_start(2)\n");
			s5l8740_i2s_codec_play_start();
		}
		dev_dbg(dai->dev, "trig: log_clocks\n");
		s5l8740_i2s_log_clocks(i2s, "trigger_start");
		dev_dbg(dai->dev, "trig: schedule_asp\n");
		s5l8740_i2s_schedule_asp();
		dev_dbg(dai->dev, "trig: done\n");
		i2s->pio_run = use_pio;
		i2s->play_jiffies = jiffies;
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
		if (!use_pio && !dma_rearm)
			WRITE_ONCE(i2s->ss, NULL);
		s5l8740_i2s_cancel_asp();
		if (dma_rearm && !use_pio)
			s5l8740_rearm_stop(i2s);
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

/*
 * Bring the codec up here, not in the trigger.
 *
 * cs42l81_play_start() sleeps for 160 ms of stock's own settle time --
 * msleep(60) for the mode-18 path settle and msleep(100) for the stage-2
 * graph. Those durations are stock's and are not ours to shorten.
 *
 * Where they ran was ours, and it was wrong. The trigger runs inside
 * snd_pcm_start() with the PCM stream lock held, and on a nonatomic DAI
 * link that lock is a mutex the writer needs to add a period. tinyalsa
 * starts the stream at a start_threshold of one period, so the sequence was:
 * write one period, block 160 ms in the trigger, DMA starts, the single
 * queued period drains in 23 ms, and snd_pcm_period_elapsed() finds
 * hw_ptr == appl_ptr and declares an underrun. The application had never
 * been scheduled with the lock free. Measured on the device: 1 period, then
 * XRUN, then the recovery paid the 160 ms again -- forever. Playback ran
 * about 200 ms per second and mmap playback, which prefills the whole
 * buffer before starting, ran a full 423-period file with 0 underruns.
 *
 * prepare() is the callback for exactly this: it runs before the
 * application fills anything and before the stream starts, so the settle
 * cost is paid once, off the playback clock. That is also stock's order --
 * the MeCCAOutputTask preamble at EA 0x080B5B1C precedes the producer
 * callback that fills the first buffer, which precedes the PL080 enable.
 * Running the codec start from the trigger put it AFTER the fill, which is
 * the divergence, not the fix.
 *
 * The trigger still calls it. cs42l81_play_start() returns immediately once
 * the state is CS42_PLAYING, so that call is a no-op on the normal path and
 * remains the backstop if prepare() was skipped.
 */
static int s5l8740_i2s_dai_prepare(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(dai->dev);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;
	if (s5l8740_i2s_audio_path_mode() != 1)
		return 0;

	WRITE_ONCE(i2s->ss, substream);
	return s5l8740_i2s_codec_play_start();
}

static const struct snd_soc_dai_ops s5l8740_i2s_dai_ops = {
	.probe = s5l8740_i2s_dai_probe,
	.hw_params = s5l8740_i2s_hw_params,
	.prepare = s5l8740_i2s_dai_prepare,
	.trigger = s5l8740_i2s_trigger,
};

static struct snd_soc_dai_driver s5l8740_i2s_dai = {
	.name = "s5l8740-i2s",
	.playback = {
		.stream_name = "I2S Playback",
		.channels_min = 1,
		.channels_max = 2,
		/*
		 * THIS is the hi-res gate, not the component open.
		 *
		 * ASoC builds runtime->hw from the DAIs on the link and that
		 * overrides whatever snd_soc_set_runtime_hwparams() put there,
		 * so widening the mask in the component achieved nothing: with
		 * a rate outside the advertised set still negotiated and reached
		 * hw_params as "rate=96000 code=15", the code measured not to
		 * lock. Playback then died with an unrecoverable ALSA I/O
		 * error instead of being resampled.
		 *
		 * So this stays at the base mask and the probe widens it only
		 * otherwise. The codec DAI on the other end of the
		 * link carries the wide mask unconditionally; ALSA offers the
		 * intersection, so gating this one end is sufficient.
		 */
		.rates = N31_RATE_MASK,
		.formats = S5L8740_I2S_FORMATS,
	},
	.ops = &s5l8740_i2s_dai_ops,
};

static const struct snd_pcm_hardware s5l8740_pio_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = S5L8740_I2S_FORMATS,
	.rates = S5L8740_I2S_RATES,
	/*
	 * 8000, not 44100: the advertised set is now every rate a 12 MHz
	 * reference divides to exactly, and 8000 is the lowest of them.
	 * See N31_RATE_MASK.
	 */
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 65536,
	.period_bytes_min = 256,
	/*
	 * One period must fit in ONE LLI node, because the re-arm model is
	 * a single node pointing at itself and s5l_pl080_rearm_set_src()
	 * refuses anything else -- rewriting node 0 of a multi-node chain
	 * would corrupt it.
	 *
	 * A node carries at most PL080_MAX_XFER_WORDS (0xfff) transfer units
	 * of s5l_pl080_unit() bytes. The only format here is S16_LE, so that
	 * is 4095 * 2 = 8190 bytes, and a frame is 4 bytes, giving 8188.
	 *
	 * This said 8192, which is two bytes over. mpg123 asked for exactly
	 * the advertised maximum, prep_dma_cyclic split it across two nodes,
	 * and the stream died at the first arm with "cannot stage period 2
	 * (-22)" -- no terminal-count interrupt was even enabled on the
	 * first node (ctl=0x04249000 against the working 0x84249000).
	 */
	.period_bytes_max = 8188,
	/*
	 * Four, not two.
	 *
	 * SDL2 asks for a buffer of exactly two periods, and with a minimum
	 * of two it got one: measured "period 8188 buffer 16376" out of
	 * ffplay, which at 48 kHz is 42.6 ms per period and 85 ms of buffer
	 * in total. The hardware pointer here only moves at period
	 * boundaries and the period callback comes from a workqueue, so
	 * every one of those 42 ms windows had to be met or the stream xran
	 * -- with the FIFO still full, which is why the underrun counter
	 * stayed at zero through every dropout. Random dropouts every few
	 * seconds under any other load is exactly that.
	 *
	 * Four periods gives the same application 170 ms and the callback
	 * three periods of slack instead of one. Costs latency no one here
	 * is asking for; buys a stream that survives an FTL read.
	 */
	.periods_min = 4,
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

/*
 * Per-buffer re-arm playback, the way RetailOS actually drives this DMA.
 *
 * snd_dmaengine_pcm drives the controller cyclically: one multi-period LLI
 * chain, submitted once, left to free-run while the hardware walks the
 * descriptors forever. Stock does not do that, and the difference sits on
 * exactly the path that stalls.
 *
 * From the playback engine at 0xB5B14 and sub_BFA50's dispatch:
 *
 *   first buffer    sub_BBA2C -> sub_BFA50 -> sub_C34EE -> sub_B424C(a4=1)
 *                   builds a five-dword LLI whose next-pointer is ITSELF,
 *                   then falls through to sub_B6620 for TXCOM |= 6
 *
 *   every one after sub_BBA12 -> sub_BFA50 -> sub_C34DC -> sub_C4960(a4=0)
 *                   programs the channel registers directly, writes the LLI
 *                   register as 0, and never touches TXCOM again
 *
 * So the hardware is handed one transfer at a time and software re-arms it
 * from the completion path. The self-link on the first descriptor is what
 * keeps the channel busy if software is late -- it repeats the last period
 * rather than stopping.
 *
 * This implements that model: one period-sized slave transfer at a time,
 * re-armed from the DMA completion callback, with the transport kick issued
 * once after the first submit. The ALSA pointer advances from the callback
 * rather than from residue, so it does not depend on the residue reporting
 * that the cyclic path needs.
 *
 * The callback also reads IIS STATUS bit 15 and write-1-clears it, counting
 * asserts. That is what the stock loop does after every submit (sub_BB9F8
 * via sub_C093C/sub_C095E, incrementing a counter at +0x164), and it is an
 * underrun flag, not a completion handshake.
 */
static void s5l8740_i2s_destroy_wq(void *wq)
{
	destroy_workqueue(wq);
}

static void s5l8740_rearm_done(void *data);

static int s5l8740_rearm_submit(struct s5l8740_i2s *i2s)
{
	struct snd_pcm_substream *ss = READ_ONCE(i2s->ss);
	struct dma_async_tx_descriptor *desc;
	struct snd_pcm_runtime *rt;
	snd_pcm_uframes_t stage_pos;
	dma_addr_t addr, stage;
	size_t period;
	int ret;

	if (!ss || !READ_ONCE(i2s->rearm_run) || !i2s->tx_chan)
		return -ENODEV;
	rt = ss->runtime;
	if (!rt || !rt->period_size)
		return -EINVAL;

	period = frames_to_bytes(rt, rt->period_size);
	addr = rt->dma_addr + frames_to_bytes(rt, i2s->rearm_pos);

	/* The period the hardware picks up at its next reload. */
	stage_pos = i2s->rearm_pos + rt->period_size;
	if (rt->buffer_size && stage_pos >= rt->buffer_size)
		stage_pos -= rt->buffer_size;
	stage = rt->dma_addr + frames_to_bytes(rt, stage_pos);

	/*
	 * Stock arms the channel ONCE and then only rewrites the descriptor.
	 *
	 * sub_BFA50 dispatches on a one-shot byte, obj[0x0C]: zero takes
	 * sub_C34EE -> sub_B424C(a4=1), which programs SrcAddr, DstAddr, LLI,
	 * Control, Control2 and finally Config with the enable bit, and builds
	 * a descriptor whose next-pointer is ITSELF. Non-zero takes
	 * sub_C34DC -> sub_C4960(a4=0), which writes only the 20-byte
	 * descriptor in memory -- no channel register at all -- and then waits
	 * on the per-channel semaphore.
	 *
	 * Because the node points at itself the transfer never terminates:
	 * after each terminal count the PL080S reloads SrcAddr, DstAddr, LLI,
	 * Control and Control2 from the descriptor, and Config.Enable stays
	 * set for the whole stream. There is no channel-disabled edge; the TC
	 * interrupt is the only completion signal.
	 *
	 * We used to call dmaengine_prep_slave_single() every period, which
	 * terminates the channel at each boundary and reprograms it from a
	 * workqueue. The IIS TX FIFO drains in about a millisecond, so every
	 * period boundary was an underrun window.
	 *
	 * prep_dma_cyclic() with buf_len == period_len builds exactly stock's
	 * topology -- nlli == 1, lli[0].lli == lli_phys -- so the first call
	 * arms it and every later call only moves the source.
	 */
	/*
	 * Nothing to post per period any more -- see the ring note below.
	 * The controller walks the buffer on its own, so once it is armed
	 * this is a no-op and only the ALSA bookkeeping in rearm_done runs.
	 */
	if (i2s->rearm_armed)
		return 0;

	desc = dmaengine_prep_dma_cyclic(i2s->tx_chan, addr, period, period,
					 DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err_ratelimited(i2s->dev,
				    "rearm: cyclic prep failed (period=%zu)\n",
				    period);
		return -ENOMEM;
	}
	desc->callback = s5l8740_rearm_done;
	desc->callback_param = i2s;
	if (dma_submit_error(dmaengine_submit(desc)))
		return -EIO;
	dma_async_issue_pending(i2s->tx_chan);
	i2s->rearm_armed = true;

	/*
	 * Hand the ring over and let the terminal-count interrupt walk it.
	 *
	 * The source walk for a cyclic ALSA buffer is fully determined by
	 * base, length and period, so software scheduling has no business
	 * being in the loop. It used to be: this function posted the next
	 * period from the same workqueue item as snd_pcm_period_elapsed(),
	 * inside a window exactly one period wide. On a single core also
	 * running USB that window gets missed, the PL080S reloads the stale
	 * source, and it replays the period it just finished -- audible as
	 * break-up, with nothing to show for it because the FIFO never ran
	 * dry and the underrun counter stayed at zero.
	 *
	 * s5l_pl080_start() has already loaded period zero into Cx_SrcAddr,
	 * which is why set_ring stages period one for the first reload.
	 */
	ret = s5l_pl080_rearm_set_ring(i2s->tx_chan, rt->dma_addr,
				       frames_to_bytes(rt, rt->buffer_size),
				       period);
	if (ret) {
		dev_err(i2s->dev, "rearm: cannot install ring (%d)\n", ret);
		return ret;
	}

	dev_dbg(i2s->dev,
		 "rearm: armed self-linked node, period %zu buffer %zu\n",
		 period, (size_t)frames_to_bytes(rt, rt->buffer_size));
	return 0;
}

/*
 * Process context, not the DMA tasklet.
 *
 * The DAI link is nonatomic, which makes the PCM stream lock a mutex, so
 * snd_pcm_period_elapsed() sleeps. Calling it from the vchan tasklet gives
 * "scheduling while atomic" and kills the re-arm after exactly one period:
 *
 *   __schedule_bug from __schedule
 *   __mutex_lock from _snd_pcm_stream_lock_irqsave
 *   snd_pcm_period_elapsed from s5l8740_rearm_done
 *   s5l8740_rearm_done from vchan_complete
 *
 * The descriptor prep is deferred with it rather than left in the callback,
 * because the PL080 prep path allocates and is not written for atomic
 * context either. One period is ~23 ms at 44.1 kHz, so workqueue latency has
 * plenty of room.
 */
/*
 * Nothing has completed for rearm_stall_ms. Stop the stream so the writer
 * gets -EPIPE instead of blocking forever.
 */
static void s5l8740_rearm_watchfn(struct work_struct *work)
{
	struct s5l8740_i2s *i2s = container_of(to_delayed_work(work),
					       struct s5l8740_i2s, rearm_watch);
	struct snd_pcm_substream *ss = READ_ONCE(i2s->ss);

	if (!ss || !READ_ONCE(i2s->rearm_run))
		return;
	dev_err(i2s->dev,
		"rearm stalled: %u periods done, no completion in %u ms -- xrun\n",
		i2s->rearm_periods, rearm_stall_ms);
	WRITE_ONCE(i2s->rearm_run, false);
	if (i2s->tx_chan)
		dmaengine_terminate_async(i2s->tx_chan);
	/*
	 * snd_pcm_stop_xrun() runs the stop synchronously, so this
	 * reaches s5l8740_i2s_trigger(STOP) and s5l8740_rearm_stop on
	 * this thread. rearm_stop tests current_work() and skips the
	 * sync cancel of this very item; without that test it waits for
	 * a completion only this thread can signal.
	 */
	snd_pcm_stop_xrun(ss);
}

static void s5l8740_rearm_workfn(struct work_struct *work)
{
	struct s5l8740_i2s *i2s = container_of(work, struct s5l8740_i2s,
					       rearm_work);
	struct snd_pcm_substream *ss = READ_ONCE(i2s->ss);
	int ret;

	if (!ss || !READ_ONCE(i2s->rearm_run)) {
		dev_dbg(i2s->dev,
				     "rearm work: ss=%p run=%d -- stopping\n",
				     ss, READ_ONCE(i2s->rearm_run));
		return;
	}
	/*
	 * Refuse to report a period for a stream that has already been
	 * stopped, even if a new one is running now. See rearm_gen.
	 */
	if (READ_ONCE(i2s->rearm_gen_queued) != READ_ONCE(i2s->rearm_gen))
		return;
	/*
	 * Per-period tracing, so dev_dbg rather than dev_info.
	 *
	 * This runs once per period for the whole life of every stream. At
	 * info level that is a steady few lines a second with nothing wrong,
	 * which fills the log buffer and evicts everything around it over the
	 * length of a track. Ratelimiting caps the rate; it does not stop a
	 * healthy stream from writing to the log forever.
	 *
	 * Dynamic debug keeps the call site compiled in and silent until it
	 * is asked for, so per-period tracing stays available:
	 *
	 *   echo 'file s5l8740-i2s.c +p' >/sys/kernel/debug/dynamic_debug/control
	 *
	 * The per-stream summary in rearm_stop() stays at info: one line per
	 * stream, and it carries the underrun count.
	 */
	dev_dbg(i2s->dev, "rearm work: elapsed+resubmit\n");
	snd_pcm_period_elapsed(ss);
	/*
	 * period_elapsed() can stop the stream underneath us on an
	 * underrun, which lands in rearm_stop and clears rearm_run.
	 * Re-arming after that would restart a stream ALSA has already
	 * torn down.
	 */
	if (!READ_ONCE(i2s->rearm_run))
		return;
	ret = s5l8740_rearm_submit(i2s);
	if (ret)
		dev_err_ratelimited(i2s->dev,
				    "rearm work: resubmit failed %d (period %u)\n",
				    ret, i2s->rearm_periods);
}

static void s5l8740_rearm_done(void *data)
{
	struct s5l8740_i2s *i2s = data;
	struct snd_pcm_substream *ss = READ_ONCE(i2s->ss);
	struct snd_pcm_runtime *rt;
	u32 st;

	if (!ss || !READ_ONCE(i2s->rearm_run))
		return;
	rt = ss->runtime;
	if (!rt || !rt->period_size)
		return;

	/* sub_BB9F8: check and write-1-clear the underrun flag. */
	if (i2s->base) {
		st = readl(i2s->base + I2SSTATUS);
		/*
		 * sub_BB9F8: sub_C093C reads STATUS bit 15 and sub_C095E writes
		 * that bit back. Stock issues the clear UNCONDITIONALLY on every
		 * iteration, with only the counter gated on the test. We used to
		 * skip the write whenever the bit read clear.
		 */
		writel(I2SSTATUS_TX_W1C, i2s->base + I2SSTATUS);
		if (st & I2SSTATUS_TX_W1C)
			i2s->rearm_underrun++;
	}

	i2s->rearm_pos += rt->period_size;
	if (i2s->rearm_pos >= rt->buffer_size)
		i2s->rearm_pos = 0;
	i2s->rearm_periods++;
	/* Per-period; see the note in rearm_workfn(). */
	dev_dbg(i2s->dev, "rearm done: period %u pos %lu\n",
		i2s->rearm_periods, (unsigned long)i2s->rearm_pos);

	if (rearm_stall_ms)
		mod_delayed_work(i2s->rearm_wq, &i2s->rearm_watch,
				 msecs_to_jiffies(rearm_stall_ms));
	WRITE_ONCE(i2s->rearm_gen_queued, READ_ONCE(i2s->rearm_gen));
	queue_work(i2s->rearm_wq, &i2s->rearm_work);
}

static int s5l8740_rearm_start(struct s5l8740_i2s *i2s)
{
	struct dma_chan *chan;
	struct dma_slave_config cfg = { };
	int ret;

	chan = s5l8740_i2s_tx_get(i2s);
	if (IS_ERR_OR_NULL(chan))
		return chan ? PTR_ERR(chan) : -ENODEV;

	cfg.direction = DMA_MEM_TO_DEV;
	cfg.dst_addr = i2s->play_dma.addr;
	cfg.dst_addr_width = i2s->play_dma.addr_width;
	cfg.dst_maxburst = i2s->play_dma.maxburst;
	ret = dmaengine_slave_config(chan, &cfg);
	if (ret)
		return ret;

	i2s->rearm_pos = 0;
	i2s->rearm_periods = 0;
	i2s->rearm_armed = false;
	i2s->rearm_underrun = 0;
	WRITE_ONCE(i2s->rearm_run, true);

	/* First transfer, then the transport kick -- sub_BFA50 then sub_B6620. */
	ret = s5l8740_rearm_submit(i2s);
	if (ret) {
		WRITE_ONCE(i2s->rearm_run, false);
		return ret;
	}
	if (rearm_stall_ms)
		mod_delayed_work(i2s->rearm_wq, &i2s->rearm_watch,
				 msecs_to_jiffies(rearm_stall_ms));
	return 0;
}

/*
 * Stop the re-arm engine. Must never block.
 *
 * Two distinct deadlocks reach this path, and neither can be waited out.
 *
 * Re-entrant: an xrun raised inside rearm_work returns to this function on
 * the workqueue thread that is servicing the period, so a synchronous
 * cancel would wait for a completion only that thread can signal.
 *
 *   s5l8740_rearm_workfn
 *     snd_pcm_period_elapsed
 *       snd_pcm_update_hw_ptr0 -> snd_pcm_update_state
 *         snd_pcm_do_stop -> soc_pcm_trigger
 *           s5l8740_i2s_trigger(STOP) -> s5l8740_rearm_stop
 *             cancel_work_sync(&i2s->rearm_work)   <- this work
 *
 * Lock cycle: any other task closing the PCM holds the substream lock
 * across the trigger, while the worker needs that same lock inside
 * snd_pcm_period_elapsed().
 *
 *   closer   snd_pcm_drop -> snd_pcm_action_single    [holds stream lock]
 *              soc_pcm_trigger -> s5l8740_rearm_stop
 *                cancel_work_sync()                   waits for worker
 *   worker   s5l8740_rearm_workfn
 *              snd_pcm_period_elapsed
 *                _snd_pcm_stream_lock_irqsave         waits for the lock
 *
 * A current_work() test does not catch the second one: the canceller is a
 * different task, so the re-entrancy check says waiting is safe. Both
 * tasks then sit in D state, taking the writer, /proc/asound and the UI
 * down with them, and the device has to be power cycled.
 *
 * So this function only clears rearm_run and cancels without waiting. The
 * blocking half is s5l8740_rearm_sync(), which ALSA invokes through
 * .sync_stop once the stream lock has been dropped.
 *
 * The cyclic path in dma-s5l8740-pl080.c guards the same way for the same
 * reason.
 */
static void s5l8740_rearm_stop(struct s5l8740_i2s *i2s)
{
	/*
	 * Clearing this is what actually stops the engine: both handlers
	 * re-check it and return without resubmitting.
	 */
	WRITE_ONCE(i2s->rearm_run, false);
	/* Anything queued before this point belongs to the ending stream. */
	WRITE_ONCE(i2s->rearm_gen, READ_ONCE(i2s->rearm_gen) + 1);

	cancel_delayed_work(&i2s->rearm_watch);
	cancel_work(&i2s->rearm_work);
	if (i2s->tx_chan)
		dmaengine_terminate_async(i2s->tx_chan);
	dev_info_ratelimited(i2s->dev,
			     "rearm stop: %u periods, %u underruns\n",
			     i2s->rearm_periods, i2s->rearm_underrun);
}

/*
 * The blocking half, run from .sync_stop.
 *
 * ALSA calls sync_stop with the stream lock dropped and in a context that
 * may sleep, which is the only safe place to wait for a handler that takes
 * that lock itself. A worker still blocked in snd_pcm_period_elapsed()
 * makes progress as soon as the closing thread releases the lock, so by
 * the time this runs there is nothing left to deadlock against.
 *
 * Still guarded for re-entrancy: an xrun raised from inside rearm_work
 * reaches the stop path on the workqueue thread itself.
 */
static void s5l8740_rearm_sync(struct s5l8740_i2s *i2s)
{
	struct work_struct *cur = current_work();

	if (cur == &i2s->rearm_work || cur == &i2s->rearm_watch.work) {
		dev_dbg(i2s->dev,
				     "rearm sync from the work itself; not waiting\n");
		return;
	}
	cancel_delayed_work_sync(&i2s->rearm_watch);
	cancel_work_sync(&i2s->rearm_work);
}

static int s5l8740_rearm_sync_stop(struct snd_soc_component *comp,
				   struct snd_pcm_substream *ss)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(comp->dev);

	s5l8740_rearm_sync(i2s);
	return 0;
}

static snd_pcm_uframes_t s5l8740_rearm_pointer(struct snd_soc_component *comp,
					       struct snd_pcm_substream *ss)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(comp->dev);

	return i2s->rearm_pos;
}

static const struct snd_pcm_hardware s5l8740_i2s_dma_hw;

static int s5l8740_rearm_open(struct snd_soc_component *comp,
			      struct snd_pcm_substream *ss)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(comp->dev);

	/*
	 * Describe the hardware to ALSA. Without this the runtime carries no
	 * rate, format or buffer limits and the open is refused outright.
	 */
	snd_soc_set_runtime_hwparams(ss, &s5l8740_i2s_dma_hw);

	/*
	 * The buffer must be a whole number of periods.
	 *
	 * Without this ALSA is free to hand back a buffer_size that is not a
	 * multiple of period_size -- it just truncates when it computes the
	 * period count. mpg123 landed on exactly that: period 8188 bytes,
	 * buffer 65536, which is eight periods plus 32 bytes.
	 *
	 * Both the position walk here and the hardware ring step by whole
	 * periods and wrap at the buffer, so a remainder puts the last period
	 * of every lap partly past the end of the buffer and desynchronises
	 * our position from ALSA's. The ring install refuses that shape; this
	 * makes sure it never gets asked to accept it.
	 */
	snd_pcm_hw_constraint_integer(ss->runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	/*
	 * A period must fit in one LLI node.
	 *
	 * The re-arm ring is a single self-linked descriptor, so the whole
	 * period has to be expressible as one transfer. The controller counts
	 * in transfer units and tops out at PL080_MAX_XFER_WORDS of them, so
	 * at the 16-bit width this link runs at the ceiling is 8190 bytes.
	 * prep_dma_cyclic() splits anything larger across several nodes,
	 * s5l_pl080_rearm_set_ring() refuses that shape, and the stream then
	 * produces no periods at all rather than degrading.
	 *
	 * period_bytes_max in the hw description is the buffer-side limit and
	 * is larger than this, so the constraint has to come from the DMA
	 * driver rather than from that table. With the 65536-byte buffer and
	 * integer periods above, this lands on 4096 bytes and 16 periods.
	 */
	{
		size_t seg = s5l_pl080_max_seg_bytes();

		if (seg && seg < s5l8740_i2s_dma_hw.period_bytes_max)
			snd_pcm_hw_constraint_minmax(ss->runtime,
					SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
					s5l8740_i2s_dma_hw.period_bytes_min,
					seg);
	}
	/*
	 * Use the whole SRAM buffer, always.
	 *
	 * The 64 KiB pool is preallocated whether or not a stream uses it, so
	 * a smaller buffer buys nothing and costs slack. ffplay was choosing
	 * 32752 bytes -- four periods, 170 ms at 48 kHz -- and had to decode,
	 * soxr-resample and read the FTL inside every one of those windows.
	 * It could not, and the result was a restart every few periods:
	 * 1 to 6 periods per stream, always with "0 underruns", because the
	 * hardware FIFO was never the thing that ran dry.
	 *
	 * 65536 is eight periods and 341 ms, double the slack, for memory
	 * that was already spent. Latency is irrelevant here -- nothing on
	 * this device is monitoring or playing live.
	 */
	snd_pcm_hw_constraint_minmax(ss->runtime, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				     65536, 65536);
	WRITE_ONCE(i2s->ss, ss);
	i2s->rearm_pos = 0;
	return 0;
}

static int s5l8740_rearm_close(struct snd_soc_component *comp,
			       struct snd_pcm_substream *ss)
{
	struct s5l8740_i2s *i2s = dev_get_drvdata(comp->dev);

	s5l8740_rearm_stop(i2s);
	/* close() may sleep and holds no stream lock. */
	s5l8740_rearm_sync(i2s);
	WRITE_ONCE(i2s->ss, NULL);
	return 0;
}

static int s5l8740_rearm_pcm_new(struct snd_soc_component *comp,
				 struct snd_soc_pcm_runtime *rtd)
{
	/*
	 * SNDRV_DMA_TYPE_DEV, not VMALLOC: the re-arm path hands
	 * runtime->dma_addr straight to the controller, so the buffer has to
	 * be DMA-coherent and physically contiguous.
	 */
	int ret = snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
						 comp->dev, 64 * 1024, 64 * 1024);

	/*
	 * Say where it landed. SRAM is 0x22000000..0x2202FFFF; anything
	 * else means the memory-region binding did not take and we are
	 * back on DRAM, which is exactly the difference against stock this
	 * change exists to remove.
	 */
	/*
	 * The buffer MUST be in SRAM. Stock runs PL080 ch2 with its source
	 * in SRAM -- measured src=0x220025d0 against dst=0x3ca00010 while
	 * playing -- and whether this controller can sustain the transfer
	 * from DRAM at all is NOT established. Our DRAM-sourced attempts
	 * moved exactly one 32-byte burst and then stalled.
	 *
	 * So a DRAM buffer is not an acceptable fallback: it would run the
	 * card in a configuration stock never uses, silently, and every
	 * measurement taken against it would be worthless. Fail instead.
	 */
	if (!ret) {
		struct snd_pcm_substream *ss =
			rtd->pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;

		if (!ss)
			return -ENODEV;
		if (ss->dma_buffer.addr < S5L8740_SRAM_BASE ||
			    ss->dma_buffer.addr >= S5L8740_SRAM_END) {
			dev_err(comp->dev,
				"PCM buffer at %pad is not SRAM -- refusing\n",
				&ss->dma_buffer.addr);
			return -ENXIO;
		}
		dev_info(comp->dev, "PCM buffer at %pad (SRAM, like stock)\n",
			 &ss->dma_buffer.addr);
	}
	return ret;
}

static const struct snd_soc_component_driver s5l8740_i2s_rearm_component = {
	.name = "s5l8740-i2s",
	.legacy_dai_naming = 1,
	.open = s5l8740_rearm_open,
	.close = s5l8740_rearm_close,
	.pointer = s5l8740_rearm_pointer,
	.sync_stop = s5l8740_rearm_sync_stop,
	.pcm_construct = s5l8740_rearm_pcm_new,
};

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
		/* sub_5705DC: clear bit 2, keep the rest. */
	writel(readl(i2s->base + I2STXCOM) & ~I2STXCOM_RUN,
	       i2s->base + I2STXCOM);
	}
	dev_info(dev, "clk_run=%u status=0x%x txcom=0x%x\n",
		 v, readl(i2s->base + I2SSTATUS),
		 readl(i2s->base + I2STXCOM));
	return count;
}
static DEVICE_ATTR_WO(clk_run);

/*
 * Hardware limits for the dmaengine PCM.
 *
 * The card is registered nonatomic, because the codec is on SPI and its
 * trigger sleeps. That makes snd_pcm_period_elapsed() a sleeping call, so the
 * DMA driver has to deliver period callbacks from a workqueue rather than
 * from its interrupt. A high-priority workqueue still costs milliseconds.
 *
 * With the defaults, tinyalsa negotiated two 1024-frame periods -- a 46 ms
 * buffer with a 23 ms period. Any scheduling delay past 23 ms empties it, and
 * measured on the device every stream went to XRUN within ~30 ms of START.
 *
 * The limits below stay permissive so applications can still choose small
 * periods; what they add over the defaults is headroom -- a 256 KB
 * preallocation and a 256 KB ceiling -- so an application that asks for a
 * large buffer actually gets one.
 */
static const struct snd_pcm_hardware s5l8740_i2s_dma_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.buffer_bytes_max = 256 * 1024,
	.period_bytes_min = 256,
	.period_bytes_max = 32768,
	/*
	 * Four, for the same reason as the playback component: a two-period
	 * buffer leaves exactly one period of slack, and the period callback
	 * is a workqueue item on a single core that also runs the FTL. On
	 * capture the consequence is an overrun rather than an underrun, but
	 * the timing margin is identical. Measured on playback as random
	 * dropouts every few seconds with the hardware never once starving.
	 */
	.periods_min = 4,
	.periods_max = 32,
	.fifo_size = 8,
};

static const struct snd_dmaengine_pcm_config s5l8740_i2s_dma_cfg = {
	.pcm_hardware = &s5l8740_i2s_dma_hw,
	.prealloc_buffer_size = 256 * 1024,
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
};

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

	/*
	 * Bind the SRAM DMA pool named by memory-region, so the PCM buffer
	 * comes from SRAM the way stock's does.
	 *
	 * RetailOS runs PL080 ch2 with src in SRAM (0x220025d0 measured
	 * while playing, dst 0x3ca00010); ours was DRAM at 0x095c0020.
	 * Without this call the memory-region property is inert and
	 * dma_alloc_coherent() still lands in the default CMA/DRAM pool.
	 *
	 * Non-fatal: if the pool is missing or too small we keep the DRAM
	 * buffer rather than refusing to probe, because a working DRAM
	 * buffer is strictly better than no sound card at all.
	 */
	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_err(dev,
			"SRAM dma pool did not bind (%d) -- refusing to run from DRAM\n",
			ret);
		return ret;
	}
	dev_info(dev, "PCM buffer pool: SRAM via memory-region\n");

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
	INIT_WORK(&i2s->rearm_work, s5l8740_rearm_workfn);
	INIT_DELAYED_WORK(&i2s->rearm_watch, s5l8740_rearm_watchfn);
	/* See the rearm_wq comment in struct s5l8740_i2s. */
	i2s->rearm_wq = alloc_workqueue("n31-i2s-rearm",
					WQ_HIGHPRI | WQ_MEM_RECLAIM, 0);
	if (!i2s->rearm_wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, s5l8740_i2s_destroy_wq,
				       i2s->rearm_wq);
	if (ret)
		return ret;

	if (dma_rearm && !use_pio && of_property_present(dev->of_node, "dmas")) {
		/*
		 * The re-arm path owns the PCM itself, so dmaengine_pcm is not
		 * registered at all: it would install its own cyclic ops and a
		 * residue-based pointer, which is the model stock does not use.
		 */
		i2s->has_dma = true;
		dev_info(dev, "PCM: per-buffer re-arm (stock model)\n");
	} else if (!use_pio && of_property_present(dev->of_node, "dmas")) {
		ret = devm_snd_dmaengine_pcm_register(dev,
						     &s5l8740_i2s_dma_cfg, 0);
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

	/*
	 * Widen the advertised rates before the DAI is registered. See the
	 * .rates comment on s5l8740_i2s_dai: this is the only thing that
	 * decides whether ALSA will ever offer 88.2/96 kHz, because ASoC
	 * derives runtime->hw from the DAIs and ignores what the component
	 * put there.
	 */

	ret = devm_snd_soc_register_component(dev,
					      use_pio ? &s5l8740_i2s_component :
					      (dma_rearm ?
					       &s5l8740_i2s_rearm_component :
					       &s5l8740_i2s_dai_component),
					      &s5l8740_i2s_dai, 1);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_regs);
	if (ret)
		dev_warn(dev, "regs sysfs: %d\n", ret);
	ret = device_create_file(dev, &dev_attr_volume);
	if (ret)
		dev_warn(dev, "volume sysfs: %d\n", ret);
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

/*
 * IIS2 gets its own PCM config rather than sharing the playback one,
 * because its buffer lives in the 32 KiB i2s2_sram pool and the shared
 * config preallocates 256 KiB -- which would simply exhaust the pool.
 *
 * periods_min is 4 here for the same reason it is on playback: a
 * two-period buffer leaves one period of slack for a callback that runs
 * on a workqueue, on a single core that also runs the FTL.
 */
static const struct snd_pcm_hardware s5l8740_iis2_dma_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.buffer_bytes_max = 32 * 1024,
	.period_bytes_min = 256,
	.period_bytes_max = 8192,
	.periods_min = 4,
	.periods_max = 16,
	.fifo_size = 8,
};

static const struct snd_dmaengine_pcm_config s5l8740_iis2_dma_cfg = {
	.pcm_hardware = &s5l8740_iis2_dma_hw,
	.prealloc_buffer_size = 32 * 1024,
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
};

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
	/* i2s2_sram bound; see the probe. Capture is refused without it. */
	bool sram_ok;
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
		writel((cur & ~CLKCON_FM_GATE_MASK) |
		       (CLKCON_FM_GATE_ON & CLKCON_FM_GATE_MASK),
		       iis2->clkcon + CLKCON_FM_GATE);
	} else if (iis2->fm_gate_held) {
		u32 prev = iis2->fm_gate_saved ? iis2->fm_gate_saved :
						 CLKCON_FM_GATE_IDLE;

		/* Restore our own bits only, not the whole word. */
		writel((cur & ~CLKCON_FM_GATE_MASK) |
		       (prev & CLKCON_FM_GATE_MASK),
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
	/* No SRAM pool, no capture -- see the probe. */
	if (!iis2->sram_ok) {
		dev_err_ratelimited(dai->dev,
			"capture refused: i2s2_sram did not bind, and DRAM is not a configuration stock uses\n");
		return -ENODEV;
	}
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
		/*
		 * Base mask, never the hi-res one: this is IIS2 to the
		 * BCM2078, the FM and Bluetooth port. It has nothing to do
		 * with the headphone path and no reason to claim 88.2/96.
		 */
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
	/*
	 * Capture buffer in SRAM, like playback.
	 *
	 * The playback port refuses to run at all unless its buffer is in low
	 * SRAM: stock's PL080 channel was measured with src=0x220025d0, and
	 * the DRAM-sourced attempt moved exactly one 32-byte burst and then
	 * stalled. Whether receive behaves the same way is untested -- the
	 * point of putting it here is that nobody has to find out. Both ports
	 * now run from the memory stock uses.
	 *
	 * A failure here is recorded and refused at hw_params, NOT at probe.
	 * Failing the probe was tried and it takes the whole sound card down
	 * with it: nano7-audio has two dai_links, and with this DAI missing
	 * the card does not register at all -- "no soundcards", no headphone
	 * playback, because of a port nothing is using yet. Refusing the
	 * stream keeps the guarantee (nothing ever captures from DRAM)
	 * without holding playback hostage to an old DT.
	 */
	iis2->sram_ok = of_reserved_mem_device_init(dev) == 0;
	if (!iis2->sram_ok)
		dev_err(dev,
			"i2s2_sram pool did not bind -- capture disabled (DT needs the audio-dma@22012000 node)\n");

	ret = devm_snd_dmaengine_pcm_register(dev, &s5l8740_iis2_dma_cfg, 0);
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
/*
 * Hand IIS2 over quiesced, DMA channel included.
 *
 * iis2_hw_stop() puts RXCOM back to idle and gates the clocks, which stops
 * the port producing data, but it never touches the DMA. The capture
 * channel belongs to snd_dmaengine_pcm, which is not in this path at all
 * on a shutdown, so a capture that was running at kexec time carries into
 * the next firmware as an enabled PL080 channel holding a descriptor that
 * points into DRAM the new image is about to be written over. It stalls
 * rather than scribbles, because the port has stopped feeding it -- but
 * "stalled and still enabled" is exactly the state this hook exists to
 * avoid, and it costs one call to avoid it.
 *
 * peri 13 is IIS2 RX, matching "dmas = <&dmac 13 0>" in the DT. The lookup
 * returns NULL when nothing has claimed it, which is the common case: no
 * capture was running, and there is nothing to stop.
 */
static void s5l8740_iis2_shutdown(struct platform_device *pdev)
{
	struct dma_chan *rx;

	iis2_hw_stop(platform_get_drvdata(pdev));

	rx = s5l_pl080_lookup_peri(13);
	if (rx)
		dmaengine_terminate_async(rx);
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
