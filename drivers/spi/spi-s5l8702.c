// SPDX-License-Identifier: GPL-2.0
/*
 * SPI controller for Samsung/Apple S5L8702 / S5L8740
 *
 * sub_4043D0(tx, rx, len, port) is the single transfer routine, shared by
 * SPI0/CS42 and SPI2/Grape. It has two arms, selected at 0x40442A on
 * SPISETUP bits 6:5:
 *
 *   PIO arm   (0x404474..0x4045B8) -- bits 6:5 clear. Per byte: RXLIMIT =
 *             rx ? 1 : 0, wait STATUS & 0x7C0 == 0, TXDATA, +0x4C = 1,
 *             wait STATUS & 0xF800 != 0, RXDATA.
 *   Block arm (0x4044C0..0x40455E) -- bits 6:5 set. +0x4C = whole TX
 *             length, +0x34 = whole RX length, SETUP gets the enable bits,
 *             SETUP |= 0x100 starts it, and the transfer completes on
 *             event 68 (sub_43CCA0(68, 100)).
 *
 * The block arm is NOT autonomous hardware DMA and it is not the PL080.
 * sub_4043D0 stores the two buffer pointers and the two counts into the
 * global at 0x08929BE0 (0x4044C4..0x4044F0) and the SPI interrupt handler
 * sub_103C moves the bytes: it drains RXDATA while STATUS & 0xF800, and
 * refills TXDATA while the TX level field (STATUS bits 10:6, ubfx #6,#5 at
 * 0x10AE) is below 16. So the FIFO is 16 deep and the block arm's benefit
 * is that SCLK runs continuously across a FIFO-full window instead of
 * stopping between every byte.
 *
 * We have no event 68, so s5l8702_spi_block_one() runs sub_103C's body from
 * the CPU in a poll loop and finishes on the same condition the handler
 * finishes on (STATUS bit 0 or bit 22 at 0x10CC/0x10D0).
 *
 * SPISTATUS bit fields, established from sub_103C:
 *   bits 10:6  TX FIFO level   (mask 0x7C0)
 *   bits 15:11 RX FIFO level   (mask 0xF800)
 *   bit 0, bit 22              transfer-complete causes
 *
 * CS = SPIPIN bit1 (sub_4045D4(port, level): assert = clear, idle = set;
 * the export drops the second argument, the disassembly at 0x4045D6 has it).
 *
 * Do not remux SPI0 pads 0–3 (SEC leftover 4/2/2/2). OSOS never GPIOCMDs
 * those pads. Do not write 0x3CF00200 — CS42 CS is 4045D4(0), not GPIOCMD.
 */
#include <linux/apple-n31.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

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
 * Block-arm register bits, all read out of sub_4043D0 at 0x4044C0ff and its
 * interrupt handler sub_103C.
 */
#define SPISTATUS_TXLVL_SHIFT	6		/* sub_103C 0x10AE ubfx #6,#5 */
#define SPISTATUS_LVL_BITS	0x1fu
#define SPI_FIFO_DEPTH		16		/* sub_103C 0x10B2 cmp #16 */
#define SPISTATUS_ACK_ALL	0x0040003fu	/* sub_3914 literal @ 0x3924 */
#define SPISTATUS_DONE		(BIT(0) | BIT(22))	/* sub_103C 0x10CC/0x10D0 */

#define SPISETUP_ENGINE_BLOCK	0x20u		/* 0x40444C orr #0x20 */
#define SPISETUP_TXEN		0x200000u	/* 0x4044FE / 0x404516 */
#define SPISETUP_RXEN		0x1080u		/* 0x40452A / 0x40453A */
#define SPISETUP_RUN		0x100u		/* 0x404542 orr #0x100 */
#define SPISETUP_ERR_MASK	0xffdfee7fu	/* literal @ 0x4045CC */
#define SPI_BLOCK_LEN_MASK	0x1ffffu	/* 0x4044E8 ubfx #0,#17 */

/*
 * Which engine each port uses, as a runtime bitmask so the change can be
 * bisected on hardware without a rebuild.
 *
 *   bit0  SPI0 (CS42L81 audio codec) uses the block arm
 *   bit1  SPI2 (Grape touch)          uses the block arm
 *
 * Default 3, which is what stock does: sub_11B70 leaves SPISETUP bit 5 set
 * on both ports (0x403E), sub_4043D0's branch at 0x40442A therefore takes
 * the block arm, and its SVC 0x46 selector 2 ("may I block?") answers
 * non-zero in task context -- both ports are brought up from task context.
 *
 * spi_engine=0 restores this driver's previous behaviour exactly: the polled
 * byte loop with SPISETUP bits 6:5 forced clear. spi_engine=2 keeps audio on
 * the old path while Grape uses the new one.
 */
static int spi_engine = 3;
module_param(spi_engine, int, 0644);
MODULE_PARM_DESC(spi_engine,
		 "engine bitmask: bit0=SPI0 block arm, bit1=SPI2 block arm (default 3; 0 = legacy per-byte PIO on both)");

/*
 * sub_40F770 does delay(10) at 0x40F782 immediately before its CS assert
 * (sub_4045D4(2, 0) at 0x40F78C) -- the minimum CS-high gap between frames.
 *
 * The units are established, and 10 us is right. sub_345D58 is an ARM
 * veneer to 0x2200104B, which IS in this image: the IRAM module is
 * memcpy(0x22000000, 0x08982B00, 0x3A60), so it reads at file offset
 * 0x983B4A. It spins on the counter at 0x3C700084 until
 * sub_983AE6(start, 10) reports the delta has reached 10.
 *
 * A tick is one microsecond. The sibling helper at 0x8983AFA takes
 * milliseconds and converts them for the same counter as
 * (125 * ms) << 3, which is 1000 ticks per millisecond, and caps its
 * argument at 0x00418937 = 4295479 ms -- 2^32 microseconds, the point at
 * which a 32-bit microsecond counter wraps.
 */
static unsigned int cs_delay_us = 10;
module_param(cs_delay_us, uint, 0644);
MODULE_PARM_DESC(cs_delay_us,
		 "delay before asserting CS, in microseconds (0x40F782 delay(10); default 10)");

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
/*
 * Bits 6:5 select which transfer engine the port is configured for.
 *
 * sub_4043D0's select, 0x40442A..0x40446C, worked through exhaustively:
 *
 *   sl = (SPISETUP & 0x60) ? 0 : 1;      // 0 = block, 1 = PIO
 *   r0 = SVC 0x46 selector 2;            // "may I block?"
 *   if (r0 == sl) {                      // register disagrees with r0
 *           if (sl) { SPISETUP = (s & ~0x60) | 0x20; sl = 0; }  // 0x40444C
 *           else    { SPISETUP =  s & ~0x60;         sl = 1; }  // 0x404446
 *   }
 *   // r0 boolean => sl == !r0 in every case
 *
 * So the register is rewritten only when it does not already agree with the
 * chosen arm; when SPISETUP already has bit 5 set and the block arm is taken,
 * stock writes SPISETUP not at all. s5l8702_select_engine() reproduces both
 * directions.
 *
 * This driver used to clear 0x60 unconditionally, forcing the polled loop on
 * a port sub_11B70 had armed for the other engine. That is a state stock
 * never occupies.
 */
#define SPISETUP_ENGINE_MASK	0x60u
/*
 * SPISETUP_RETAILOS was 0x403c. That value is refuted: no path in either
 * image produces it. Both sub_11B70 call sites pass mode 0x1A, which
 * yields 0x403E, and the bootloader uses 0x401E on SPI0.
 */
#define SPISETUP_SPI0_11B70	0x403e	/* 11B70(0,0x1A,0x2EE0,8) → 0x402E|0x10 */

/*
 * Transfer word size. sub_11B70 always leaves this at 8-bit for both ports,
 * so this field is ours to drive per transfer rather than something stock
 * varies at setup time.
 *
 * Apple's own loaders do vary it mid-download: upstream apple_z2 sends the
 * HBPP init payload with 8-bit words and every later firmware blob with
 * 16-bit words. On a little-endian host a 16-bit word puts the high byte on
 * the wire first, so the same byte array clocks out pair-swapped -- which is
 * exactly the "pair-swapped payload bytes" signature our own HBPP14 notes
 * record. That makes word width, not just baud rate, a candidate for a
 * bootstrap/runtime mismatch.
 */
/*
 * There is no runtime word-size control here.
 *
 * sub_11B70 builds SPISETUP from its mode argument and then ORs in the
 * literal 0x4000 unconditionally at EA 0x08011C60, writes it at 0x08011C64,
 * and ORs in 0x10 at 0x08011C78. Both of its call sites pass the SAME mode,
 * 0x1A -- sub_11B70(2, 0x1A, 0x2EE0, 1) for SPI2 at EA 0x0801A600 and
 * sub_11B70(0, 0x1A, 0x2EE0, 8) for SPI0 at EA 0x0807441A -- so both ports
 * end up with the same SPISETUP. There are no other callers.
 *
 * Every other write to SPISETUP in either image preserves bits 13 and 14:
 * the masks are ~0x60, ~0x1080, ~0x100, ~0x200000, ~1, 0xFFDFEE7F and
 * 0xFFBFFFFE. Bit 13 is never set anywhere in either image.
 *
 * So this driver used to drive a field stock treats as a constant. The
 * per-transfer word-size write is gone. Note also that 0x4000 under the
 * bits-13:14 encoding this file used would decode as "32-bit", which
 * contradicts the byte-at-a-time movement stock actually performs -- so
 * either the field position was wrong or 0x4000 is not a word-size field at
 * all. Which of those is true is NOT established from the image; what is
 * established is that the bits never change.
 */

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
	bool spi2_grape;
	bool prepared;
	int last_err;
	u32 last_status;
	enum s5l8702_spi_fam fam;
	bool cs_held;		/* CS left asserted by a cs_change transfer */
	unsigned int fam_latch_status;
	unsigned int tx_timeouts;
	unsigned int rx_timeouts;
	unsigned int block_xfers;
	unsigned int block_no_done;	/* completed without STATUS bit0/bit22 */
	/*
	 * Receive evidence from the last transfer, for callers that have to
	 * tell a genuine reply apart from an empty FIFO being read out.
	 *
	 * A working RetailOS was captured with SPISTATUS = 0x00000002 -- both
	 * receive-level fields zero, so the FIFO was empty -- while SPIRXDATA
	 * still returned 0x48484848 / 0x79797979. Reading RXDATA therefore
	 * yields a byte pattern whether or not anything arrived, and a caller
	 * that only looks at the bytes cannot tell the two cases apart.
	 *
	 * rx_last_status is SPISTATUS sampled immediately before the final
	 * RXDATA read, so it still shows the level that satisfied the wait.
	 * rx_last_short is how many requested bytes never arrived: the block
	 * arm can leave the loop on SPISTATUS_DONE with rx_left non-zero and
	 * still return success, which leaves the tail of the caller's buffer
	 * untouched.
	 */
	u32 rx_last_status;
	unsigned int rx_last_short;
	/*
	 * SPISTATUS as it read inside s5l8702_wait_rx_ready at the moment
	 * the wait was satisfied, kept separately from rx_last_status.
	 *
	 * The first run of this instrumentation reported a byte taken with
	 * rx_last_status = 0x00000002 -- both level fields clear. No wait in
	 * this driver can pass on that value, so the level cannot still have
	 * been readable one register read later. Recording the satisfying
	 * value as well as the pre-pop value separates a level that clears
	 * when read from a wait that never gated on a level at all.
	 */
	u32 rx_wait_status;
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
 * sub_11B70's timing registers, read out of the disassembly at
 * 0x11BE6..0x11C74:
 *
 *   *(base+0x44) = 10                                       0x11BE8
 *   clk5 = sub_43CFCC(5);  clk4 = sub_43CFCC(4)             0x11BF2 / 0x11BFC
 *   *(base+0x38) = (clk5 / 1000) * a4                       0x11C0C
 *   *(base+0x40) = 255                                      0x11C14
 *   *(base+0x3c) = 3 * (clk4 / 1000) * (a4 + 1)             0x11C24
 *   CLKDIV       = (clk5 / arg2) & 0x7FF                    0x11C6E
 *
 * The two dividers use DIFFERENT clock ids: +0x38 and CLKDIV come from id 5
 * (ldr r0,[sp,#0]), +0x3c comes from id 4 (ldr r0,[sp,#4]). This driver
 * assumed both were 24 MHz, which is where the hardcoded 0x18c on SPI2 came
 * from and why "solving 3*24*(a4+1) = 396 gives 4.5" never closed: the wrong
 * term was being solved for.
 *
 * clk5 = 24000 kHz is fixed by CLKDIV: arg2 is 0x2EE0 = 12000 at both call
 * sites and the captured CLKDIV is 2.
 *
 * clk4 = 66000 kHz is NOT read from the image -- sub_43CFCC is a tbb jump
 * table over live PLL/divider registers. It is solved from the captured
 * RetailOS SPI2 value +0x3c = 0x18c with a4 = 1: 396 = 3 * k * 2 => k = 66.
 * Overridable so that inference can be tested rather than assumed.
 */
static unsigned int spi_clk4_khz = 66000;
module_param(spi_clk4_khz, uint, 0644);
MODULE_PARM_DESC(spi_clk4_khz,
		 "clock id 4 in kHz, feeds +0x3c = 3*(clk4/1000)*(a4+1) (default 66000, solved from the captured SPI2 0x18c)");

static unsigned int spi_clk5_khz = 24000;
module_param(spi_clk5_khz, uint, 0644);
MODULE_PARM_DESC(spi_clk5_khz,
		 "clock id 5 in kHz, feeds +0x38 and CLKDIV (default 24000, fixed by the captured CLKDIV=2)");

static u32 s5l8702_u38(unsigned int a4)
{
	return (spi_clk5_khz / 1000) * a4;		/* 0x11C0C */
}

static u32 s5l8702_u3c(unsigned int a4)
{
	return 3 * (spi_clk4_khz / 1000) * (a4 + 1);	/* 0x11C24 */
}

/*
 * SPI0's +0x3c override.
 *
 * The formula above makes SPI0 0x6F6 where this driver has been writing
 * 0x288 (3 * 24 * 9). SPI0 carries the CS42L81 and audio currently works, so
 * this exists to put the old value back without a rebuild.
 */
static unsigned int spi0_u3c;
module_param(spi0_u3c, uint, 0644);
MODULE_PARM_DESC(spi0_u3c,
		 "SPI0 +0x3c override (0 = computed 3*(clk4/1000)*9 = 0x6f6; 648 = this driver's previous value)");

/* sub_11B70(0, 0x1A, 0x2EE0, 8) */
static void s5l8702_spi0_11b70(struct s5l8702_spi *sspi)
{
	const unsigned int a4 = 8;
	u32 dd = s5l8702_u38(a4);
	u32 u3c = spi0_u3c ? spi0_u3c : s5l8702_u3c(a4);
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
 * SPI2 +0x3c is now derived rather than hardcoded.
 *
 * s5l8702_u3c(1) = 3 * 66 * 2 = 396 = 0x18c, which is the value the
 * RetailOS SPI2 oracle was captured with. This used to be a literal 0x18c
 * with a note that the formula "does not reproduce the hardware"; it does,
 * once +0x3c is read as clock id 4 rather than id 5.
 */
static unsigned int spi2_u3c;
module_param(spi2_u3c, uint, 0644);
MODULE_PARM_DESC(spi2_u3c,
		 "SPI2 +0x3c override (0 = computed 3*(clk4/1000)*2 = 0x18c)");

static struct s5l8702_spi *s5l8702_spi2_dev;

static void s5l8702_spi2_11b70(struct s5l8702_spi *sspi)
{
	const unsigned int a4 = 1;
	u32 dd = s5l8702_u38(a4);
	u32 u3c = spi2_u3c ? spi2_u3c : s5l8702_u3c(a4);
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
			sspi->rx_wait_status = val;
			return 0;
		}
		if (sspi->fam != SPI_FAM_ROS &&
		    (val & SPISTATUS_RXLVL_MASK)) {
			s5l8702_latch_fam(sspi, SPI_FAM_CLASSIC, val);
			sspi->rx_wait_status = val;
			return 0;
		}
		cpu_relax();
	}

	sspi->rx_timeouts++;
	sspi->last_status = val;
	sspi->rx_wait_status = val;
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

/*
 * A literal wait on a SPISTATUS mask, with no encoding-family logic.
 *
 * The block arm's drains are literal in stock (`tst.w r1, #0x7c0` at
 * 0x404498 and `#0xf800` at 0x4044A0) and sub_103C settles the field layout:
 * the TX level is STATUS bits 10:6 (0x7C0) and the RX level is bits 15:11
 * (0xF800). Nothing in the image supports the alternative 0x1F0/0x3E00
 * reading, so the block path does not consult sspi->fam at all.
 *
 * Stock spins here without a bound; we keep a guard so a dead bus reports as
 * dead instead of wedging the CPU.
 */
static int s5l8702_wait_status_clear(struct s5l8702_spi *sspi, u32 mask)
{
	unsigned int guard = SPI_WAIT_GUARD;
	u32 val = 0;

	while (guard--) {
		val = readl(sspi->base + SPISTATUS);
		if ((val & mask) == 0)
			return 0;
		cpu_relax();
	}
	sspi->last_status = val;
	return -ETIMEDOUT;
}

static bool s5l8702_use_block(struct s5l8702_spi *sspi)
{
	if (sspi->spi0)
		return !!(spi_engine & 1);
	if (sspi->spi2_grape)
		return !!(spi_engine & 2);
	return false;
}

/* sub_4043D0 0x40442A..0x40446C: make SPISETUP agree with the chosen arm. */
static void s5l8702_select_engine(struct s5l8702_spi *sspi, bool block)
{
	u32 setup = readl(sspi->base + SPISETUP);

	if (block) {
		if (!(setup & SPISETUP_ENGINE_MASK))
			writel((setup & ~SPISETUP_ENGINE_MASK) |
			       SPISETUP_ENGINE_BLOCK, sspi->base + SPISETUP);
	} else {
		if (setup & SPISETUP_ENGINE_MASK)
			writel(setup & ~SPISETUP_ENGINE_MASK,
			       sspi->base + SPISETUP);
	}
}

/*
 * The block arm of sub_4043D0, 0x4044C0..0x40455E, with sub_103C's body
 * polled in place of the event wait.
 *
 * Register programming, hunk by hunk:
 *
 *   0x4044C0  sub_3914(port)               STATUS |= 0x0040003F
 *   0x4044CA  CTRL   |= 0xC                FIFO reset
 *   0x4044D6  while (STATUS & 0x7C0)
 *   0x4044DE  while (STATUS & 0xF800)
 *   0x4044E8  len &= 0x1FFFF               (ubfx #0, #17)
 *   0x4044F0  +0x4C = len  (tx)            0x404508: +0x4C = 0 (no tx)
 *   0x4044F6  SETUP &= ~1  (tx)            0x40450E: SETUP |= 1  (no tx)
 *   0x4044FE  SETUP |= 0x200000 (tx)       0x404516: SETUP &= ~0x200000
 *   0x404520  +0x34 = len  (rx)            0x404534: +0x34 = 0  (no rx)
 *   0x40452A  SETUP |= 0x1080   (rx)       0x40453A: SETUP &= ~0x1080
 *   0x404546  SETUP |= 0x100               start
 *   0x40454C  sub_43CCA0(68, 100)          block on event 68
 *   0x404554  on failure SETUP &= 0xFFDFEE7F, return 81
 *
 * The event does not exist for us. sub_103C is what would have set it, and
 * it is also what actually moves the bytes -- the controller does not fetch
 * them itself -- so the loop below is that handler, run from the CPU:
 *
 *   0x1074  drain RXDATA while the RX level is non-zero and rx_left
 *   0x109A  push TXDATA while the TX level is < 16 and tx_left
 *   0x10BC  both counts drained -> SETUP &= ~0x100
 *   0x10CC  STATUS & (bit0|bit22) -> SETUP &= 0xFFDFEE7F, signal event 68
 *   0x10E4  STATUS = status (acknowledge)
 *
 * Two deliberate deviations, both noted because they are ours and not
 * stock's:
 *
 *  - sub_103C gates its RX drain on STATUS & 0x23 and its TX refill on
 *    STATUS & 2, which are the interrupt causes that woke it. Polling has no
 *    cause to read, so both inner loops run unconditionally. They are
 *    self-limiting on the FIFO level and the remaining count, so this can
 *    never move more data than the handler would.
 *  - If both counts drain and the TX FIFO empties without STATUS ever
 *    showing bit 0 or bit 22, the transfer is treated as complete and
 *    counted in block_no_done. Without that, a completion cause that only
 *    latches when the interrupt is unmasked would turn every transfer into
 *    a timeout.
 */
static int s5l8702_spi_block_one(struct s5l8702_spi *sspi,
				 const u8 *tx, u8 *rx, unsigned int len)
{
	const u8 *txp = tx;
	u8 *rxp = rx;
	unsigned int tx_left, rx_left;
	unsigned int guard = SPI_WAIT_GUARD;
	bool done = false;
	u32 setup, st;
	int ret;

	/* 0x40440A / 0x404414: nothing to do, return success. */
	if ((!tx && !rx) || !len)
		return 0;

	len &= SPI_BLOCK_LEN_MASK;
	tx_left = tx ? len : 0;
	rx_left = rx ? len : 0;

	writel(readl(sspi->base + SPISTATUS) | SPISTATUS_ACK_ALL,
	       sspi->base + SPISTATUS);
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO,
	       sspi->base + SPICTRL);

	ret = s5l8702_wait_status_clear(sspi, SPISTATUS_TXBUSY_ROS);
	if (ret)
		goto out_err;
	ret = s5l8702_wait_status_clear(sspi, SPISTATUS_RXRDY_ROS);
	if (ret)
		goto out_err;

	if (tx) {
		writel(len, sspi->base + SPIUNK4C);
		writel(readl(sspi->base + SPISETUP) & ~SPISETUP_RXMODE,
		       sspi->base + SPISETUP);
		setup = readl(sspi->base + SPISETUP) | SPISETUP_TXEN;
	} else {
		writel(0, sspi->base + SPIUNK4C);
		writel(readl(sspi->base + SPISETUP) | SPISETUP_RXMODE,
		       sspi->base + SPISETUP);
		setup = readl(sspi->base + SPISETUP) & ~SPISETUP_TXEN;
	}
	writel(setup, sspi->base + SPISETUP);

	if (rx) {
		writel(len, sspi->base + SPIRXLIMIT);
		setup = readl(sspi->base + SPISETUP) | SPISETUP_RXEN;
	} else {
		writel(0, sspi->base + SPIRXLIMIT);
		setup = readl(sspi->base + SPISETUP) & ~SPISETUP_RXEN;
	}
	writel(setup, sspi->base + SPISETUP);

	writel(readl(sspi->base + SPISETUP) | SPISETUP_RUN,
	       sspi->base + SPISETUP);

	while (guard--) {
		bool moved = false;

		st = readl(sspi->base + SPISTATUS);

		while (rx_left &&
		       (readl(sspi->base + SPISTATUS) & SPISTATUS_RXRDY_ROS)) {
			*rxp++ = (u8)readl(sspi->base + SPIRXDATA);
			rx_left--;
			moved = true;
		}

		while (tx_left &&
		       ((readl(sspi->base + SPISTATUS) >> SPISTATUS_TXLVL_SHIFT)
			& SPISTATUS_LVL_BITS) < SPI_FIFO_DEPTH) {
			writel(*txp++, sspi->base + SPITXDATA);
			tx_left--;
			moved = true;
		}

		if (!tx_left && !rx_left)
			writel(readl(sspi->base + SPISETUP) & ~SPISETUP_RUN,
			       sspi->base + SPISETUP);

		writel(st, sspi->base + SPISTATUS);

		if (st & SPISTATUS_DONE) {
			done = true;
			break;
		}
		if (!tx_left && !rx_left &&
		    !(readl(sspi->base + SPISTATUS) & SPISTATUS_TXBUSY_ROS)) {
			sspi->block_no_done++;
			done = true;
			break;
		}
		if (moved)
			guard = SPI_WAIT_GUARD;
		cpu_relax();
	}

	/*
	 * rx_left survives the SPISTATUS_DONE break above, which does not
	 * test it. Record it before that success is returned.
	 */
	sspi->rx_last_short = rx ? rx_left : 0;
	sspi->rx_last_status = readl(sspi->base + SPISTATUS);

	if (!done) {
		ret = -ETIMEDOUT;
		goto out_err;
	}

	/* sub_103C 0x10D8 clears the same four bits the error path does. */
	writel(readl(sspi->base + SPISETUP) & SPISETUP_ERR_MASK,
	       sspi->base + SPISETUP);
	sspi->block_xfers++;
	return 0;

out_err:
	writel(readl(sspi->base + SPISETUP) & SPISETUP_ERR_MASK,
	       sspi->base + SPISETUP);
	sspi->last_err = ret;
	sspi->last_status = readl(sspi->base + SPISTATUS);
	return ret;
}

static void s5l8702_spi_hw_init(struct s5l8702_spi *sspi)
{
	writel(0xf, sspi->base + SPISTATUS);
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO,
	       sspi->base + SPICTRL);
	writel(SPI_CLKDIV_DEFAULT, sspi->base + SPICLKDIV);
	/* idle: CS deasserted (bit1 set), match prior SPIPIN=6 */
	writel(0x6, sspi->base + SPIPIN);
	writel(SPISETUP_SPI0_11B70, sspi->base + SPISETUP);
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
	writel(SPISETUP_SPI0_11B70, sspi->base + SPISETUP);
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
		if (ret) {
			sspi->rx_last_short = rx ? len - i : 0;
			return ret;
		}
		{
			/*
			 * Sampled before the pop, so it still carries the
			 * level that satisfied the wait above. If this reads
			 * back with both level fields clear, the wait latched
			 * on something other than a byte having arrived and
			 * the value below is an empty-FIFO read.
			 */
			u32 st_rx = readl(sspi->base + SPISTATUS);
			u8 b = (u8)readl(sspi->base + SPIRXDATA);

			if (rx) {
				rx[i] = b;
				sspi->rx_last_status = st_rx;
				if (sspi->rx_last_short)
					sspi->rx_last_short--;
			}
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
 * The PIO arm of sub_4043D0, 0x404474..0x4045B8. CS is the caller's, exactly
 * as it is stock's: sub_4043D0 never touches SPIPIN, sub_40F770 does.
 *
 * One byte per iteration, because that is what stock does. It reads the TX
 * buffer with ldrb, writes SPITXDATA a byte at a time, sets SPIRXLIMIT
 * (+0x34) to rx ? 1 : 0 and SPIUNK4C (+0x4c) to 1 on every byte, and takes
 * RX back one byte at a time. Nothing in either image moves more than a byte
 * per FIFO slot.
 *
 * The three "STATUS & 0x7C0 == 0x40 is close enough" escapes are gone.
 * 0x7C0 is the TX FIFO level field (sub_103C 0x10AE), so 0x40 means one byte
 * still queued, and stock's `tst.w r1, #0x7c0 / bne` loops at 0x404496,
 * 0x40457A and 0x4045AA require the field to be empty.
 */
static int s5l8702_spi_pio_core(struct s5l8702_spi *sspi,
				const u8 *tx, u8 *rx, unsigned int len)
{
	unsigned int i;
	int ret;

	/* sub_4043D0 preamble, 0x40448E..0x4044A4 */
	writel(readl(sspi->base + SPICTRL) | SPICTRL_RESET_FIFO,
	       sspi->base + SPICTRL);
	ret = s5l8702_wait_clear(sspi, SPISTATUS_TXBUSY_ROS);
	if (ret)
		return ret;
	ret = s5l8702_wait_clear(sspi, SPISTATUS_RXRDY_ROS);
	if (ret)
		return ret;

	/*
	 * 0x4044AC: TX present -> SETUP &= ~1, STATUS |= 0x400000.
	 * 0x404560: RX-only (tx == NULL) -> SETUP |= 1, STATUS |= 1.
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
		/* 0x40456E: RXLIMIT = 1 only when the call has an RX buffer */
		writel(rx ? 1 : 0, sspi->base + SPIRXLIMIT);

		ret = s5l8702_wait_clear(sspi, SPISTATUS_TXBUSY_ROS);
		if (ret)
			break;

		if (tx) {
			writel(tx[i], sspi->base + SPITXDATA);
			/* 0x40458C: *(base+0x4C) = 1 after each TX byte */
			writel(1, sspi->base + SPIUNK4C);
		}

		if (rx) {
			ret = s5l8702_wait_set(sspi, SPISTATUS_RXRDY_ROS);
			if (ret)
				break;
			rx[i] = (u8)readl(sspi->base + SPIRXDATA);
		}
	}
	if (!ret)
		ret = s5l8702_wait_clear(sspi, SPISTATUS_TXBUSY_ROS);

	/* 0x4045B2 epilogue: SETUP &= 0xFFBFFFFE */
	writel(readl(sspi->base + SPISETUP) & ~0x400001u,
	       sspi->base + SPISETUP);
	return ret;
}

/*
 * One transfer, chip select and engine selection included.
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
static int s5l8702_spi_xfer_one(struct s5l8702_spi *sspi,
			       const u8 *tx, u8 *rx, unsigned int len,
			       bool assert_cs, bool deassert_cs)
{
	bool block = s5l8702_use_block(sspi);
	int ret;

	/* Start pessimistic: every requested byte is missing until read. */
	sspi->rx_last_short = rx ? len : 0;
	sspi->rx_last_status = 0;

	if (assert_cs) {
		/* 0x40F782: delay(10) immediately before sub_4045D4(2, 0) */
		if (cs_delay_us)
			udelay(cs_delay_us);
		s5l8702_spi_cs(sspi, true);
	}
	sspi->cs_held = true;

	s5l8702_select_engine(sspi, block);

	if (block)
		ret = s5l8702_spi_block_one(sspi, tx, rx, len);
	else
		ret = s5l8702_spi_pio_core(sspi, tx, rx, len);

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
		dev_err_ratelimited(sspi->dev,
				    "4043D0 u4c=%08x rxlimit=%08x engine=%s block_ok=%u block_no_done=%u\n",
				    readl(sspi->base + SPIUNK4C),
				    readl(sspi->base + SPIRXLIMIT),
				    block ? "block(4044C0)" : "pio(404474)",
				    sspi->block_xfers, sspi->block_no_done);
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

	return s5l8702_spi_xfer_one(sspi, xfer->tx_buf, xfer->rx_buf,
				   xfer->len,
				   !sspi->cs_held, deassert);
}

/*
 * What the last transfer on @spi's controller actually received.
 *
 * @status:    SPISTATUS as it stood immediately before the final RXDATA
 *             read, so the receive-level fields still describe what the
 *             pop was about to take.
 * @wait_status: SPISTATUS as read inside the receive wait at the moment
 *             it was satisfied, before any pop. Compare it against
 *             @status: if a level shows here and not there, the field
 *             clears when read.
 * @shortfall: requested receive bytes that never arrived. Zero means the
 *             buffer was filled from the FIFO; equal to the transfer
 *             length means nothing came back at all and every byte in
 *             the caller's buffer is either its own initial value or an
 *             empty-FIFO read.
 *
 * Returns -ENODEV if @spi is not on this controller, so a caller can ask
 * unconditionally and simply ignore the answer elsewhere. The values are
 * per-controller and valid only until the next transfer.
 */
int s5l8702_spi_last_rx(struct spi_device *spi, u32 *status,
			unsigned int *shortfall, u32 *wait_status)
{
	struct spi_controller *ctlr = spi ? spi->controller : NULL;
	struct s5l8702_spi *sspi;

	if (!ctlr || ctlr->transfer_one != s5l8702_spi_transfer_one)
		return -ENODEV;

	sspi = spi_controller_get_devdata(ctlr);
	if (!sspi)
		return -ENODEV;

	if (status)
		*status = sspi->rx_last_status;
	if (shortfall)
		*shortfall = sspi->rx_last_short;
	if (wait_status)
		*wait_status = sspi->rx_wait_status;
	return 0;
}
EXPORT_SYMBOL_GPL(s5l8702_spi_last_rx);

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
	sspi->spi2_grape = res && res->start == SPI2_BASE_PHYS;

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
		dev_info(&pdev->dev,
			 "SPI0 CS42 4043D0 %s CS=SPIPIN.1 PWRCON1=%08x\n",
			 s5l8702_use_block(sspi) ? "block arm 4044C0" :
						   "PIO arm 404474",
			 readl(sspi->pwrcon1));
	} else if (sspi->spi2_grape) {
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
		dev_info(&pdev->dev,
			 "SPI2 Grape 4043D0 %s (SETUP=0x%x CLKDIV=2 CS=SPIPIN.1 11B70 u3c=%u)\n",
			 s5l8702_use_block(sspi) ? "block arm 4044C0" :
						   "PIO arm 404474",
			 SPISETUP_SPI0_11B70,
			 spi2_u3c ? spi2_u3c : s5l8702_u3c(1));
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
