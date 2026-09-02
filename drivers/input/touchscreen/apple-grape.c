// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple Grape / Grape multitouch — N31 SPI2 @ 0x3D200000
 *
 * RetailOS 1.0.2 sequences (osos extracts):
 *   bring-up     sub_1A5AC / 2075A / 20766 / 20690 / 11B70
 *   teardown     sub_1A878: IRQ off, RST, 20690(0), rail off, EN mode 1
 *                (13A20 retries: 1A878 + sleep 50 + 1A5AC, max 3)
 *   grape.bin    IS the app. SEC bootloader has no grape/Grape path.
 *   1A5AC delays 2075A(1)+5 → 20766(1)+15 → 20690(1)+5 → 11B70 →
 *                20848 +15 → 2075A(0)+30 → 20E94 (no extra POR pulse)
 *   bootload cmd sub_20848(6593): 6593 = 0x19C1 = HBPP ENTER (not a
 *                firmware byte count). TX 19 C1 + (18 E1)* pad.
 *
 * Firmware (SPI → controller @ dest=offset, start 0) — 1A640 / 204E0 / 2D640:
 *   Full "8740" GrapeFirmware-style container on disk:
 *     0x000..0x3ff  Apple/N31 header (NOT sent over SPI)
 *     0x400..       ARM app body; length = le32(file+0x0c)
 *     rev 3: GID-CBC IV=0 decrypt of ARM body before send
 *   ARM-only cut (no 8740 magic, e.g. 18 F0 9F E5…): whole file = body
 *   Chunks: max 0x1FF0. Upload = sub_3B9D0 envelope (see below). ACK 0x4BC1.
 *   Callsite 2D640: r1 = firmware offset (NOT 0x00100000). EXEC 0x00100018
 *   is the bootloader-mapped app PC, not the upload destination.
 *
 * Calibration (SPI → controller @ 0x00400200) — 2D7A4 / 273A0:
 *   Per-device touch calibration comes from the A34 handoff, not a host file:
 *     desc @ 0x2202FE18 (sub_A34(24)): magic 0x53797349 "touch calibration", ptr @ +4
 *     memcpy 0x560 from ptr (sub_564)
 *     cal = bytes at decimal +350, length 0x200, reverse each u32 (sub_273A0)
 *     then 2D7A4 that window; DATA packet still does b1b0b3b2 wire swizzle
 *   Callsite 2D7A4: r1 = 0x00400200 + offset.
 *   No grape-cal.bin, no FTL touch calibration scan, no GrapeFirmware.bin+350.
 *   Preferred source: U-Boot copy in reserved DRAM, advertised in /chosen
 *     apple,n31-touch_cal-addr / apple,n31-touch_cal-size. Never consume the original
 *     A34 pointer. Live A34 ioremap is fallback only.
 *
 * sub_3B9D0 upload frame (full SPI length = payload_len + 16):
 *   [0..1]     18 E1
 *   [2..3]     30 01
 *   [4..5]     word_count = len>>2 as hi,lo  (len>>10, len>>2)
 *   [6..9]     dest swizzled BYTE1,BYTE0,BYTE3,BYTE2
 *   [10..11]   u16 byte-sum of [4..9], big-endian
 *   [12..12+len)  payload u32s swizzled B1 B0 B3 B2
 *   [12+len..] u32 byte-sum of swizzled payload, stored B1 B0 B3 B2
 *   First FW prefix (dest=0,len=0x1FF0): 18 E1 30 01 07 FC 00 00 00 00 01 03
 *   Cal prefix (dest=0x400200,len=0x200): 18 E1 30 01 00 80 02 00 00 40 00 C2
 *
 *   After cal: 2D5B0 RequestCal → 2D54C EXEC → 40 ms → runtime ping.
 *   Register input only after runtime ping. runtime_ready = ping csum.
 *
 * Firmware host file: request_firmware("apple/grape.bin") and/or
 * FTL gpfw/8740 when fw_prefer_ftl=1. That is the ARM app, not cal.
 */
#include <crypto/aes.h>
#include <crypto/skcipher.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/kallsyms.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include <linux/apple-n31.h>

#define GRAPE_MAGIC		0xEA
#define GRAPE_PING_TYPE	490
#define GRAPE_BOOTLOAD_WORD	6593		/* 0x19C1 */
#define GRAPE_FRAME_LEN	16
#define GRAPE_READ_MAX		512
#define GRAPE_SLOTS		8
#define GRAPE_ABS_X_MAX	239
#define GRAPE_ABS_Y_MAX	431
#define GRAPE_SCALE_X_DIV	0x0B1D
#define GRAPE_SCALE_Y_DIV	0x1482

#define GRAPE_CHUNK_MAX	0x1FF0		/* 8176 — sub_2D640 */
#define GRAPE_HDR_LEN		16		/* 2 outer + 10 body hdr + 4 payload sum */
#define GRAPE_CAL_DEST		0x00400200u	/* 2D7A4 literal 0x400200 */
#define GRAPE_FW_HDR_OFF	350
#define GRAPE_FW_HDR_LEN	0x200
#define GRAPE_IMG1_BODY_OFF	1024	/* IMG1 body offset, 0x400 */
#define GRAPE_IMG1_SIG_LEN	128	/* IMG1 body signature, 0x80 */
#define GRAPE_IMG1_CERT_LEN	768	/* sub_204E0 guard constant */
#define GRAPE_ARM_OFFICIAL	0xe970		/* 8740 le32(+0x0c); 204E0 2D640 size */
#define GRAPE_TOUCH_CAL_MAGIC	0x53797349u	/* 'touch calibration' — sub_564 */
#define GRAPE_TOUCH_CAL_LEN		0x560
#define GRAPE_A34_BASE		0x2202fe00UL	/* sub_A34(idx) = 0x2202FE00+idx */
#define GRAPE_A34_TOUCH_CAL_DESC	(GRAPE_A34_BASE + 0x18)	/* sub_A34(24) */

/* Whimory FTL (nand-s5l8740.ko) — optional cal/FW from device NAND */
#define GRAPE_FTL_SECTOR_SIZE	4096U
#define GRAPE_GPFW_TAG		0x67706677u	/* 'gpfw' LE */

#define GRAPE_ACK_CHUNK	0x4BC1		/* 19393 */
#define GRAPE_ACK_34AD0	0x4AD1		/* 19153 */
#define GRAPE_POST_POKE	0x011F		/* 287 */
#define GRAPE_WATCHDOG_MS	1000	/* IRQ-primary: this thread is a fallback */
#define GRAPE_EXEC_SETTLE_MS	40		/* 273A0 tail: 410522(40) */

#define GRAPE_GPIO_EN		0x0E
#define GRAPE_GPIO_RST		0x27
#define GRAPE_GPIO_IRQ		0x26

#define S5L8740_GPIO_PHYS	0x3cf00000UL
#define S5L8740_GPIOCMD_PHYS	0x3cf001e0UL
#define S5L8740_SPI2_PHYS	0x3d200000UL
#define SPI2_CTRL		0x00
#define SPI2_SETUP		0x04
#define SPI2_STATUS		0x08
#define SPI2_PIN		0x0c
#define SPI2_TXDATA		0x10
#define SPI2_RXDATA		0x20
#define SPI2_CLKDIV		0x30
#define SPI2_RXLIMIT		0x34
#define SPI2_UNK4C		0x4c
#define SPI2_CTRL_FIFO_RST	0x0c
#define SPI2_CTRL_ENABLE	0x01
#define SPI2_SETUP_11B70	0x403e	/* 11B70(2, 0x1A, 0x2EE0, 1) */
#define SPI2_CS_BIT		BIT(1)

#define GRAPE_Z2_HDR_LEN	16
#define GRAPE_Z2_MAGIC_5A5A	0x5a5a0000u
#define GRAPE_Z2_MAGIC_C3F5	0xc3f50000u
#define GRAPE_Z2FW_MAGIC	0x5746325au	/* apple_z2 "Z2FW" container */

#define GRAPE_CS_BEGIN		BIT(0)
#define GRAPE_CS_END		BIT(1)

/*
 * 0 lets the SPI controller own the engine setup, which is what the stock
 * firmware does: one sub_11B70 configuration, divider 2 for a 12 MHz bus.
 * This driver used to reprogram the same registers afterwards with a much
 * larger divider, quietly running the bus eight times too slow.
 */
static int spi_clkdiv;
module_param(spi_clkdiv, int, 0644);
MODULE_PARM_DESC(spi_clkdiv,
		 "Override SPI2 CLKDIV (0 = leave it to the SPI controller)");
/*
 * The stock code retries the transfer itself three times before giving
 * up; the caller's retry re-runs the whole bring-up.
 */
static unsigned int download_tries = 3;
module_param(download_tries, uint, 0644);
MODULE_PARM_DESC(download_tries,
		 "Firmware download attempts before re-running bring-up");

/*
 * The stock path does nothing after a good download but wait 2 ms and
 * enable the interrupt. These register pokes keep speaking the boot
 * protocol at a part that should already be running its application.
 */
static int reset_hold_ms = 5;
module_param(reset_hold_ms, int, 0644);
MODULE_PARM_DESC(reset_hold_ms, "RST low ms in optional extra_por_pulse (1A5AC uses 5)");
/* 1A5AC: 2075A(0) then sleep 30 before 20E94 — not FAMILY 15 / old 100. */
static int reset_release_ms = 30;
module_param(reset_release_ms, int, 0644);
MODULE_PARM_DESC(reset_release_ms, "ms after RST release before probe/FW (1A5AC=30)");
static int extra_por_pulse;
module_param(extra_por_pulse, int, 0644);
MODULE_PARM_DESC(extra_por_pulse, "1=extra RST low/high/low before 1A5AC (not in RetailOS)");
static int go_spi_setup;
module_param(go_spi_setup, int, 0644);
MODULE_PARM_DESC(go_spi_setup, "SPI2 SETUP override for 2D54C GO (0=11B70)");
/* 0=8-bit PIO (RetailOS HBPP default), 1=u16 TXDATA pairs, 2=spi_sync */
/*
 * Read a report whenever ATTN is asserted, without requiring the ping
 * to checksum first. Off restores the ping-gated behaviour.
 */
static bool attn_read = true;
module_param(attn_read, bool, 0644);
MODULE_PARM_DESC(attn_read,
		 "Read a frame when ATTN asserts even if the ping fails");

static bool post_poke_strict;
module_param(post_poke_strict, bool, 0644);
MODULE_PARM_DESC(post_poke_strict,
		 "Require a known 2D5B0 status before EXEC");

/*
 * Never park.
 *
 * Parking sets stopped and ends servicing, so the ATTN line stops being
 * read and the interrupt stops being acted on. That makes it impossible
 * to answer the one question stock actually asks after EXEC -- does the
 * part raise ATTN when the panel is touched -- because the driver has
 * already given up by the time anyone puts a finger on the glass.
 *
 * For measuring only. A part that never starts will sit here forever.
 */
/*
 * Consecutive failed services before ATTN is masked.
 *
 * The interrupt is level-triggered on a line the application holds low
 * until a report is read out of it, and IRQF_ONESHOT re-enables as soon
 * as the threaded handler returns. So a read that never succeeds means
 * the line is never released and the interrupt re-fires immediately,
 * forever: the count climbed past twelve thousand while nothing was
 * being delivered. That is real CPU burned on a part that is not
 * answering.
 *
 * Masking after a ceiling keeps the failure visible without letting it
 * spin. Write attn_rearm to try again once something has changed.
 */
static unsigned int irq_fail_max = 20;
module_param(irq_fail_max, uint, 0644);
MODULE_PARM_DESC(irq_fail_max,
		 "Consecutive failed services before ATTN is masked (0 = never)");

static bool no_park;
module_param(no_park, bool, 0644);
MODULE_PARM_DESC(no_park,
		 "Keep servicing after a failed bring-up instead of parking");

static bool park_power_down = true;
module_param(park_power_down, bool, 0644);
MODULE_PARM_DESC(park_power_down,
		 "Cut the rail when parking (0 keeps it up for raw_xfer)");

static int go_xfer;
module_param(go_xfer, int, 0644);
MODULE_PARM_DESC(go_xfer,
		 "2D54C EXEC xfer: 0=8-bit burst 1=u16 burst 2=spi_sync");
/* Default 0: N31 RetailOS path has no Z2/5A5A host container. */
static int prepend_z2_hdr;
module_param(prepend_z2_hdr, int, 0644);
MODULE_PARM_DESC(prepend_z2_hdr, "0=none (N31 default) 1=5A5A+BE len+CRC32 2=c3f5 hdr");
static int chunk_spi;
module_param(chunk_spi, int, 0644);
MODULE_PARM_DESC(chunk_spi, "1=spi_sync chunk xfers (apple_z2-style atomic CS)");
/* Default quiet: bring-up spam off; set quiet=0 or verbose=1 for detail. */
static int quiet = 1;
module_param(quiet, int, 0644);
MODULE_PARM_DESC(quiet, "1=minimal logs (default); 0=verbose bring-up (or apple_grape.verbose=1)");

static bool verbose;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose, "Verbose Grape logs (overrides quiet; default N)");
static int skip_download;
module_param(skip_download, int, 0644);
MODULE_PARM_DESC(skip_download, "1=bootload+ping only, no FW chunks");

static int force_gid;
module_param(force_gid, int, 0644);
MODULE_PARM_DESC(force_gid, "1=422FFA decrypt attempt even without 8740 rev3 hdr");

static int cal_try_dt = 1;
module_param(cal_try_dt, int, 0644);
MODULE_PARM_DESC(cal_try_dt,
		 "Read U-Boot touch calibration copy from /chosen apple,n31-touch_cal-* (default on)");

static int cal_try_a34 = 1;
module_param(cal_try_a34, int, 0644);
MODULE_PARM_DESC(cal_try_a34,
		 "Fallback: read live A34 descriptor at 0x2202FE18 (default on)");

/*
 * Stock does not ping for a runtime status after EXEC.
 *
 * sub_2D54C returns sub_40F770(...) == 0 -- success is that the 12-byte
 * EXEC transfer completed, nothing more. sub_20E94 sets the success flag
 * from that, and sub_20490(1) then does exactly two things: puts pad 0x26
 * (38) in mode 0 as an input and arms its interrupt. The host waits for
 * ATTN; it never asks the part to identify itself again.
 *
 * That matters because 0x4F81 is a bootloader-format status. A part that
 * has correctly left the bootloader has no reason to answer a
 * bootloader-style ping, so requiring one turns a good boot into a
 * failure. Worse, the retry loop below only exited on that ping, so a
 * running application was torn down and re-uploaded three times before we
 * gave up and parked.
 *
 * Default off, which is stock. Set it to require the ping back if a build
 * needs the old, stricter behaviour for comparison.
 */
static bool require_runtime;
module_param(require_runtime, bool, 0644);
MODULE_PARM_DESC(require_runtime,
		 "Require a runtime ping after EXEC before registering input "
		 "(stock does not; default N)");

static unsigned int exec_wait_ms = 40;
module_param(exec_wait_ms, uint, 0644);
MODULE_PARM_DESC(exec_wait_ms,
		 "ms after EXEC before runtime ping (OSOS 2D54C success wait = 40)");

static unsigned int cal_ftl_start;
module_param(cal_ftl_start, uint, 0644);
MODULE_PARM_DESC(cal_ftl_start, "FTL LBA to start gpfw/8740 firmware scan (not cal)");

static unsigned int cal_ftl_count = 4096;
module_param(cal_ftl_count, uint, 0644);
MODULE_PARM_DESC(cal_ftl_count,
		 "FTL LBAs to scan for gpfw/8740 firmware (not touch calibration cal)");

/*
 * fw_prefer_ftl is retired -- the gpfw store does not exist on this device.
 *
 * Stock's tag lookup is not an FPART special block. sub_201FC('gpfw') goes
 * through sub_683E0 -> sub_2F51C -> sub_38DA8 (ATABlock) -> sub_26794 /
 * sub_261A4 and bottoms out in ATA READ SECTORS / READ DMA against a drive
 * whose IDENTIFY model is "NAND FLASH DRIVE" -- an ATA-shaped facade over
 * Whimory, reading plain FTL logical blocks of 4096 bytes, in the same LBA
 * space as the FAT32 volume.
 *
 * It expects LBA 0 to be an MBR, takes the entry whose type byte is 0x00 or
 * 0x3F (RetailOS writes the firmware partition as entry 1, type 0x3F, at LBA
 * 64), then requires le32 @ 0x100 == 0x5B68695D ("[hi]" byte-reversed) and
 * walks sixteen 40-byte directory entries for tag 0x67706677.
 *
 * Measured on this unit, LBA 0 is the FAT32 VBR, not an MBR:
 *
 *	00000000: eb3c 902a 554f 4b4a 4948 4300 1004 2000  .<.*UOKJIHC... .
 *	000001be: 0000 0000 0000 0000 0000 0000 0000 0000
 *	000001fe: 55aa
 *	00000100: 0000 0000 0000 0000 0000 0000 0000 0000
 *
 * eb 3c 90 jump, OEM "*UOKJIHC", all four partition entries zero, no "[hi]"
 * header, and the 5d 69 68 5b magic appears nowhere in block 0 or block 64.
 * The FTL LBA space begins directly with the FAT volume, so the lookup would
 * pass its 0xAA55 test on the VBR, match type 0x00 on a zeroed entry, take
 * start LBA 0, fail the "[hi]" check and give up.
 *
 * MEMORY[0x8A8FAA4] must therefore be non-zero on this hardware and stock
 * takes the filesystem branch. The option could never have found anything,
 * and leaving it selectable invites someone to enable it while chasing a
 * touch failure and change two variables at once.
 */

static int fw_allow_file = 1;
module_param(fw_allow_file, int, 0644);
MODULE_PARM_DESC(fw_allow_file, "1=allow apple/grape.bin fallback");

/*
 * 2D640 r1 = firmware offset (start 0). Do NOT default to 0x00100000 —
 * that is the EXEC-mapped app window, not the upload dest. Override only
 * for deliberate A/B experiments.
 */
static unsigned int fw_dest;
module_param(fw_dest, uint, 0644);
MODULE_PARM_DESC(fw_dest,
		 "2D640 ARM upload base dest (OSOS offset 0; cal stays 0x400200)");

static unsigned int exec_addr = 0x00100018;
module_param(exec_addr, uint, 0644);
MODULE_PARM_DESC(exec_addr,
		 "2D54C EXEC word0 (OSOS 0x00100018; bootloader-mapped PC)");
static unsigned int exec_word1 = 0x00000100;
module_param(exec_word1, uint, 0644);
MODULE_PARM_DESC(exec_word1, "2D54C EXEC word1 (OSOS 0x00000100)");


static bool grape_verbose;

#define grape_dev_vinfo(dev, fmt, ...) \
	do { \
		if (grape_verbose) \
			dev_info((dev), fmt, ##__VA_ARGS__); \
		else \
			dev_dbg((dev), fmt, ##__VA_ARGS__); \
	} while (0)

#define grape_vinfo(n, fmt, ...) \
	do { \
		if (!(n)->parked) \
			grape_dev_vinfo(&(n)->spi->dev, fmt, ##__VA_ARGS__); \
	} while (0)

struct grape {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *enable;
	struct gpio_desc *reset;
	struct gpio_desc *attn;
	void __iomem *gpio_base;
	void __iomem *gpiocmd;
	void __iomem *spi2;
	struct task_struct *thread;
	struct mutex lock;
	bool stopped;
	bool fw_uploaded;	/* 2D640/2D7A4 transport ACKs */
	bool cal_uploaded;
	bool requestcal_done;	/* 2D5B0 / 1F01 path done */
	bool exec_sent;		/* 2D54C SPI xfer completed — not runtime */
	u8 *raw_rx;		/* last raw_xfer response */
	unsigned int raw_n;
	unsigned int attn_fails;
	bool irq_masked;
	unsigned int irq_fails;	/* recycle must not reset this */
	/* true when the blob came from the FTL gpfw tag, false for a
	 * host file. Only the gpfw path inspects the "8740" magic. */
	bool fw_from_ftl;
	unsigned int irq_count;
	unsigned int irq_seen;
	bool runtime_ready;	/* valid 182590 ping checksum */
	bool fw_loaded;		/* alias of runtime_ready for older call sites */
	bool fw_tried;
	bool spi_ok;
	bool use_irq;
	bool blob16;	/* S5L TXDATA is 8-bit; 16-bit writes fail 4BC1 */
	bool parked;	/* give up after recycle budget — stop SPI spam */
	bool have_touch_cal;
	bool have_cal;
	bool touch_cal_sysfs;
	u8 touch_cal[GRAPE_TOUCH_CAL_LEN];
	u8 cal_upload[GRAPE_FW_HDR_LEN];
	int irq;
	unsigned int ping_fails;
	unsigned int recycle_count;
	int spi_fam;		/* 0 auto, 1 ROS, 2 Classic */
	unsigned int tx_timeouts;
	unsigned int rx_timeouts;
};

/* From gpio-d1830.c — OSOS 20766 / 6644(4) / reg16 bit5 */

static u16 grape_sum16(const u8 *buf, int len)
{
	u16 sum = 0;

	while (len-- > 0)
		sum += *buf++;
	return sum;
}

static void grape_gpiocmd_mode(struct grape *n, unsigned int gpio, u16 mode, int val)
{
	void __iomem *bank;
	u32 pin = gpio & 7;
	u32 dir;
	u8 cmd;

	if (!n->gpiocmd || !n->gpio_base)
		return;

	bank = n->gpio_base + 32 * (gpio >> 3);
	if (mode == 1) {
		/* 43D38C: mode 1 also sets DIR, then GPIOCMD 14/15 */
		cmd = val ? 15 : 14;
		dir = readl(bank + 0x14);
		writel(dir | BIT(pin), bank + 0x14);
	} else if (mode == 0xFFFE) {
		dir = readl(bank + 0x14);
		writel(dir & ~BIT(pin), bank + 0x14);
		cmd = 0;
	} else {
		cmd = (u8)mode;
		dir = readl(bank + 0x14);
		writel(dir | BIT(pin), bank + 0x14);
	}
	writel(((gpio >> 3) << 16) | (pin << 8) | cmd, n->gpiocmd);
}

/* sub_23CD0(gpio, on) — PUNC bit at bank+0x10 */
static void grape_punc(struct grape *n, unsigned int gpio, bool set)
{
	void __iomem *bank;
	u32 pin, punc;

	if (!n->gpio_base)
		return;
	bank = n->gpio_base + 32 * (gpio >> 3);
	pin = gpio & 7;
	punc = readl(bank + 0x10);
	if (set)
		punc |= BIT(pin);
	else
		punc &= ~BIT(pin);
	writel(punc, bank + 0x10);
}

/* sub_20690(a1) — SPI2 pads 0x57–0x5A */
static void grape_spi2_pinmux(struct grape *n, bool on)
{
	if (on) {
		grape_punc(n, 0x57, false);
		grape_gpiocmd_mode(n, 0x57, 5, 0);
		grape_gpiocmd_mode(n, 0x58, 3, 0);
		grape_gpiocmd_mode(n, 0x59, 3, 0);
		grape_gpiocmd_mode(n, 0x5A, 3, 0);
	} else {
		grape_gpiocmd_mode(n, 0x57, 0xFFFE, 0);
		grape_punc(n, 0x57, true);
		grape_gpiocmd_mode(n, 0x58, 1, 0);
		grape_gpiocmd_mode(n, 0x59, 1, 0);
		grape_gpiocmd_mode(n, 0x5A, 0xFFFE, 0);
	}
}

/*
 * sub_1A878 — disable / retry power-cut:
 *   20490(0), RST assert, 20690(0), 20766(0) rail off + EN mode 1.
 */
static void grape_power_down(struct grape *n)
{
	grape_gpiocmd_mode(n, GRAPE_GPIO_IRQ, 0xFFFE, 0);
	grape_gpiocmd_mode(n, GRAPE_GPIO_RST, 1, 0);
	grape_spi2_pinmux(n, false);
	d1830_grape_rail(false);
	grape_gpiocmd_mode(n, GRAPE_GPIO_EN, 1, 0);
	grape_vinfo(n, "1A878 power-cut (RST hold, rail off, EN mode 1)\n");
}

/*
 * CLKCON oracle replay.
 *
 * Deliberately not named after any peripheral. Nothing in the extracted
 * Grape boot path -- sub_13A20, sub_1A5AC and the 2075A/20766/20690/
 * 11B70/20848/20E94/20490 chain -- writes CLKCON or calls sub_41CBD8 at
 * all, so there is no evidence naming any of these bits as a touch
 * clock. What is established is narrower and still worth replaying:
 * Linux is missing global CLKCON state that both the Apple bootloader
 * and the running stock system have.
 *
 * The N31 bootloader sets, at 0x3C500000:
 *
 *   +0x08 = 0x2009200A   we inherit this unchanged
 *   +0x0C = 0x80008000   we have 0x00000000
 *   +0x10 = 0x00008000   we have 0x00000000
 *   +0x14 = 0x80008000   we have 0x00002200
 *   +0x18 = 0x20012001   we inherit this unchanged
 *
 * so something between DFU, u-boot and Linux clears two of them and
 * rewrites a third, while leaving their neighbours alone.
 *
 * Polarity is inverted -- sub_41CBD8 clears a bit to enable and sets it
 * to disable -- so our zeroes mean more clocks running than stock, not
 * fewer. And the low nibble of +0x10 is a divider, not a flag: the rate
 * decoder reads (MEMORY[0x3C500010] & 0xF) + 1. The stock live-touch
 * 0x8000 -> 0x8004 delta is therefore a divider change, not an enable.
 *
 *   clkcon_oracle=1  replay stock's live-touch values
 *   clkcon_oracle=2  replay the bootloader's post-init values
 *
 * Values are restored on unload so a failed experiment does not leave
 * the clock tree in a state the rest of the kernel did not ask for.
 */
#define N31_CLKCON_PHYS		0x3c500000UL

static int clkcon_oracle;
module_param(clkcon_oracle, int, 0644);
MODULE_PARM_DESC(clkcon_oracle,
		 "Replay observed CLKCON state: 1=stock touch, 2=bootloader");

static const unsigned int n31_clkcon_off[] = { 0x08, 0x0c, 0x10, 0x14 };
static const u32 n31_clkcon_touch[] = {
	0xa009200a, 0x80000001, 0x00008004, 0x80002200,
};
static const u32 n31_clkcon_boot[] = {
	0x2009200a, 0x80008000, 0x00008000, 0x80008000,
};
static u32 n31_clkcon_saved[ARRAY_SIZE(n31_clkcon_off)];
static bool n31_clkcon_applied;

static void grape_clkcon_replay(struct grape *n)
{
	const u32 *want;
	void __iomem *ck;
	unsigned int i;

	if (!clkcon_oracle || n31_clkcon_applied)
		return;
	want = (clkcon_oracle == 2) ? n31_clkcon_boot : n31_clkcon_touch;

	ck = ioremap(N31_CLKCON_PHYS, 0x80);
	if (!ck)
		return;
	for (i = 0; i < ARRAY_SIZE(n31_clkcon_off); i++) {
		n31_clkcon_saved[i] = readl(ck + n31_clkcon_off[i]);
		writel(want[i], ck + n31_clkcon_off[i]);
		dev_info(&n->spi->dev,
			 "ORACLE_REPLAY clkcon+0x%02x %08x -> %08x\n",
			 n31_clkcon_off[i], n31_clkcon_saved[i],
			 readl(ck + n31_clkcon_off[i]));
	}
	n31_clkcon_applied = true;
	iounmap(ck);
}

static void grape_clkcon_restore(void)
{
	void __iomem *ck;
	unsigned int i;

	if (!n31_clkcon_applied)
		return;
	ck = ioremap(N31_CLKCON_PHYS, 0x80);
	if (!ck)
		return;
	for (i = 0; i < ARRAY_SIZE(n31_clkcon_off); i++)
		writel(n31_clkcon_saved[i], ck + n31_clkcon_off[i]);
	n31_clkcon_applied = false;
	iounmap(ck);
}

/*
 * sub_11B70(2, 0x1A, 0x2EE0, 1) after every 20690(1).
 * 1A5AC always re-inits SPI2 here. Skipping it after 1A878 remux
 * left an extra SCLK edge: 0x1f01/0x4879 came back as 0x0f80/0xa43c
 * and shifted one more bit on each retry.
 */
static void grape_spi2_11b70(struct grape *n)
{
	void (*reinit)(void);
	u32 setup;

	if (!n->spi2)
		return;
	reinit = (void (*)(void))__symbol_get("s5l8702_spi2_reinit");
	if (reinit) {
		reinit();
		__symbol_put("s5l8702_spi2_reinit");
	} else {
		/* Controller absent: same sequence, same numbers. */
		writel(0xf, n->spi2 + SPI2_STATUS);
		writel(readl(n->spi2 + SPI2_CTRL) | SPI2_CTRL_FIFO_RST,
		       n->spi2 + SPI2_CTRL);
		writel(10, n->spi2 + 0x44);
		writel(24, n->spi2 + 0x38);	/* 24 * a4=1 */
		writel(255, n->spi2 + 0x40);
		writel(144, n->spi2 + 0x3c);	/* 3 * 24 * (1+1) */
		writel(2, n->spi2 + SPI2_CLKDIV);	/* 24000/12000 */
		writel(SPI2_SETUP_11B70, n->spi2 + SPI2_SETUP);
		writel(readl(n->spi2 + SPI2_CTRL) | SPI2_CTRL_FIFO_RST,
		       n->spi2 + SPI2_CTRL);
		writel(SPI2_CTRL_ENABLE, n->spi2 + SPI2_CTRL);
	}
	if (spi_clkdiv)
		writel(clamp(spi_clkdiv, 1, 255), n->spi2 + SPI2_CLKDIV);
	setup = readl(n->spi2 + SPI2_SETUP);
	grape_vinfo(n, "11B70 SPI2 SETUP=0x%x CLKDIV=%u\n",
		 setup, readl(n->spi2 + SPI2_CLKDIV));
}

static void grape_spi2_cs(struct grape *n, bool assert)
{
	u32 pin;

	if (!n->spi2)
		return;
	pin = readl(n->spi2 + SPI2_PIN);
	if (assert)
		pin &= ~SPI2_CS_BIT;
	else
		pin |= SPI2_CS_BIT;
	writel(pin, n->spi2 + SPI2_PIN);
}

/*
 * Tight 4043D0 PIO for 16-byte app frames. OSOS 11B70 leaves SETUP
 * bit5 set so 40F770 takes the DMA path — continuous clocks. Linux
 * per-byte spi_sync gaps are fine for the bootloader, not the app.
 */
static void grape_spi2_fifo_flush(struct grape *n)
{
	if (!n->spi2)
		return;
	writel(readl(n->spi2 + SPI2_CTRL) | SPI2_CTRL_FIFO_RST,
	       n->spi2 + SPI2_CTRL);
	writel(0xf, n->spi2 + SPI2_STATUS);
}

/*
 * SPI2 status polling.
 *
 * Both transfer loops below used to do this:
 *
 *   guard = 100000;
 *   do { st = readl(STATUS); } while (!(st & 0xf800) && --guard);
 *   rx[i] = readl(RXDATA);
 *
 * The guard running out was not treated as a failure. RXDATA was read
 * either way and the function returned 0, so when the ready bit never set,
 * the FIFO's previous contents were handed back as if they were a reply.
 * That does not look like noise because it is not noise: it is the same
 * stale word every time, which is where the repeating 4f81 came from, and
 * it is why the part appeared to answer while telling us nothing.
 *
 * Two changes. A timeout is now an error that propagates, so a dead bus
 * reports as dead instead of inventing a reply. And because this engine has
 * been seen reporting readiness in either of two encodings depending on the
 * init path that ran, both are polled in one loop until one genuinely
 * satisfies; that result is latched, logged once, and used exclusively
 * afterwards. Latching matters: alternating between interpretations is how
 * a transfer gets declared complete early.
 */
#define GRAPE_SPI_GUARD	100000u

#define GRAPE_TXBUSY_ROS	0x7c0u
#define GRAPE_TXBUSY_RESIDUE	0x40u	/* stays set after 11B70 setup */
#define GRAPE_TXLVL_CLASSIC	0x1f0u
#define GRAPE_RXRDY_ROS	0xf800u
#define GRAPE_RXLVL_CLASSIC	0x3e00u

static int grape_spi_family;
module_param(grape_spi_family, int, 0644);
MODULE_PARM_DESC(grape_spi_family,
		 "SPI2 status encoding: 0=auto-latch, 1=ROS (0x7C0/0xF800), 2=Classic (0x1F0/0x3E00)");

/*
 * Fail closed on a receive timeout.
 *
 * This was zero-initialised while its description said strict was the
 * default. In the direct-PIO path a receive timeout was then followed by a
 * read of whatever RXDATA still held, so stale FIFO contents could satisfy
 * an 0x4bc1 ACK, a status read, or a runtime-ready check. A silent
 * controller could look like a working one.
 */
static bool grape_strict_rx = true;
module_param(grape_strict_rx, bool, 0644);
MODULE_PARM_DESC(grape_strict_rx,
		 "1=receive timeout fails the transfer (default); 0=legacy, read the FIFO anyway");

static void grape_latch_fam(struct grape *n, int fam, u32 st)
{
	if (n->spi_fam == fam)
		return;
	n->spi_fam = fam;
	dev_info(&n->spi->dev,
		 "SPI2 status encoding latched: %s (STATUS=0x%08x)\n",
		 fam == 1 ? "ROS(0x7C0/0xF800)" : "Classic(0x1F0/0x3E00)", st);
}

/* Wait for the transmit side to accept another word. */
static int grape_wait_tx(struct grape *n)
{
	unsigned int guard = GRAPE_SPI_GUARD;
	u32 st = 0;

	while (guard--) {
		st = readl(n->spi2 + SPI2_STATUS);

		if (n->spi_fam != 2) {
			u32 b = st & GRAPE_TXBUSY_ROS;

			if (b == 0 || b == GRAPE_TXBUSY_RESIDUE) {
				grape_latch_fam(n, 1, st);
				return 0;
			}
		}
		if (n->spi_fam != 1 && (st & GRAPE_TXLVL_CLASSIC) == 0) {
			grape_latch_fam(n, 2, st);
			return 0;
		}
		cpu_relax();
	}

	if (!n->tx_timeouts++)
		dev_warn(&n->spi->dev,
			 "SPI2 transmit never went idle (STATUS=0x%08x)\n", st);
	return -ETIMEDOUT;
}

/* Wait for a received word to actually be available. */
static int grape_wait_rx(struct grape *n)
{
	unsigned int guard = GRAPE_SPI_GUARD;
	u32 st = 0;

	while (guard--) {
		st = readl(n->spi2 + SPI2_STATUS);

		if (n->spi_fam != 2 && (st & GRAPE_RXRDY_ROS)) {
			grape_latch_fam(n, 1, st);
			return 0;
		}
		if (n->spi_fam != 1 && (st & GRAPE_RXLVL_CLASSIC)) {
			grape_latch_fam(n, 2, st);
			return 0;
		}
		cpu_relax();
	}

	if (!n->rx_timeouts++)
		dev_warn(&n->spi->dev,
			 "SPI2 receive never became ready (STATUS=0x%08x), refusing to report stale FIFO\n",
			 st);
	return -ETIMEDOUT;
}

/*
 * Transfers go through the SPI core.
 *
 * This driver is bound as an spi_device yet drove the SPI2 registers itself:
 * its own CS, its own FIFO reset, its own TXDATA/RXDATA polling. That meant
 * none of the usual guarantees applied -- no bus locking against another
 * client, no controller-side clock or mode setup, no error propagation --
 * and it duplicated the controller driver's completion logic well enough to
 * drift from it. Two copies of a tricky wait loop is one copy too many.
 *
 * The only thing the register path really offered was holding CS down across
 * several calls, which the core expresses with cs_change on the final
 * transfer of a message. The controller now honours that, so the whole thing
 * is reachable through spi_sync().
 *
 * The legacy path is kept behind grape_use_spi=0 purely so the two can be
 * compared on hardware; it is not the supported route.
 */
static bool grape_use_spi;	/* opt-in until proven on glass */
module_param(grape_use_spi, bool, 0644);
MODULE_PARM_DESC(grape_use_spi,
		 "1=transfer via the SPI core (default); 0=legacy direct SPI2 register PIO");

/*
 * One HBPP burst as a single spi_message.
 *
 * @hold_cs: leave the part selected when the message ends, for a frame that
 *           spans more than one call. Expressed as cs_change on the last
 *           transfer, which is exactly what that flag means there.
 *
 * A TX-only caller passes rx == NULL straight through, so spi-s5l8702 takes
 * its SPIRXLIMIT = 0 branch and never waits for a receive byte.
 *
 * This used to allocate a throwaway drain buffer, on the reasoning that the
 * part clocks a reply out regardless and an undrained 8 KiB chunk would
 * overrun the RX FIFO. Stock does not do that: sub_2D640 sends every chunk
 * as sub_40F770(buf, len + 16, 0, 0) with rxlen zero, and sub_4043D0 then
 * writes SPIRXLIMIT = 0 and never reads SPIRXDATA. Supplying a drain buffer
 * forced full duplex on exactly the frames stock sends one-way.
 */
static int grape_spi_burst(struct grape *n, const u8 *tx, u8 *rx,
			    unsigned int len, bool hold_cs, unsigned int bpw)
{
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = len,
		.cs_change = hold_cs,
		.bits_per_word = bpw,
	};
	struct spi_message m;
	int ret;

	if (!n->spi)
		return -ENODEV;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(n->spi, &m);

	if (ret && !n->tx_timeouts++)
		dev_warn(&n->spi->dev, "spi_sync failed: %d\n", ret);
	return ret;
}

static int grape_burst_ex(struct grape *n, const u8 *tx, u8 *rx,
			   unsigned int len, unsigned int cs_flags)
{
	unsigned int i;
	int ret = 0;

	if (grape_use_spi)
		return grape_spi_burst(n, tx, rx, len,
					!(cs_flags & GRAPE_CS_END), 8);

	if (!n->spi2)
		return -ENODEV;
	if (cs_flags & GRAPE_CS_BEGIN) {
		grape_spi2_cs(n, true);
		/* No delay: sub_40F770 clocks the first byte straight
		 * after sub_4045D4(2, 0). */
		/*
		 * Read CS back rather than trusting the write. A MISO that
		 * floats for a whole frame looks identical to a part that
		 * never saw the transaction; the difference is whether
		 * SPIPIN bit 1 is actually low while we clock.
		 */
		dev_info_once(&n->spi->dev,
			      "CS during xfer: PIN=0x%08x (%s) SETUP=0x%08x STATUS=0x%08x\n",
			      readl(n->spi2 + SPI2_PIN),
			      (readl(n->spi2 + SPI2_PIN) & SPI2_CS_BIT) ?
					"HIGH, NOT asserted" : "low, asserted",
			      readl(n->spi2 + SPI2_SETUP),
			      readl(n->spi2 + SPI2_STATUS));
		grape_spi2_fifo_flush(n);
		writel(readl(n->spi2 + SPI2_SETUP) & ~BIT(0), n->spi2 + SPI2_SETUP);
		writel(readl(n->spi2 + SPI2_STATUS) | 0x400000u, n->spi2 + SPI2_STATUS);
	}
	/*
	 * A TX-only frame must not touch the receive side at all.
	 *
	 * sub_4043D0 writes SPIRXLIMIT = rx ? 1 : 0 and only waits for and
	 * reads SPIRXDATA when there is an rx buffer. Every firmware and
	 * calibration DATA frame goes out as sub_40F770(buf, len + 16, 0, 0)
	 * -- rxlen zero -- so on stock those chunks never wait for a receive
	 * byte.
	 *
	 * We wrote SPIRXLIMIT = 1 unconditionally and then waited for and
	 * drained a receive byte for every byte of every chunk, which is a
	 * per-byte wait on a FIFO stock never asks to fill.
	 */
	for (i = 0; i < len; i++) {
		writel(rx ? 1 : 0, n->spi2 + SPI2_RXLIMIT);

		ret = grape_wait_tx(n);
		if (ret)
			goto out;

		writel(tx[i], n->spi2 + SPI2_TXDATA);
		writel(1, n->spi2 + SPI2_UNK4C);

		if (rx) {
			ret = grape_wait_rx(n);
			if (ret && grape_strict_rx)
				goto out;
			ret = 0;
			rx[i] = (u8)readl(n->spi2 + SPI2_RXDATA);
		}
	}

out:
	if (cs_flags & GRAPE_CS_END) {
		writel(readl(n->spi2 + SPI2_SETUP) & ~0x400001u, n->spi2 + SPI2_SETUP);
		grape_spi2_cs(n, false);
	}
	return ret;
}

static int grape_burst(struct grape *n, const u8 *tx, u8 *rx, unsigned int len)
{
	return grape_burst_ex(n, tx, rx, len, GRAPE_CS_BEGIN | GRAPE_CS_END);
}

/*
 * The 16-bit burst variants are gone.
 *
 * They existed to drive the SPISETUP word-size field so each pair would go
 * out high byte first, matching upstream apple_z2. sub_11B70 ORs the literal
 * 0x4000 into SPISETUP once per port and never touches those bits again, and
 * both of its call sites pass the same mode 0x1A, so the two ports cannot
 * even differ. Bit 13 is never set anywhere in either image.
 *
 * The pair swap the Grape needs is real, but stock does it in the CPU while
 * packing the buffer -- grape_build_upload_frame() already matches sub_3B9D0
 * byte for byte -- not by changing the controller word width.
 */
static int grape_burst_u16_ex(struct grape *n, const u8 *tx, u8 *rx,
			      unsigned int len, unsigned int cs_flags)
{
	return grape_burst_ex(n, tx, rx, len, cs_flags);
}

static int grape_burst_u16(struct grape *n, const u8 *tx, u8 *rx,
			    unsigned int len)
{
	return grape_burst_u16_ex(n, tx, rx, len,
				   GRAPE_CS_BEGIN | GRAPE_CS_END);
}

static int grape_burst16(struct grape *n, const u8 *tx, u8 *rx)
{
	return grape_burst(n, tx, rx, GRAPE_FRAME_LEN);
}

/*
 * One transport for every Grape frame.
 *
 * Stock has exactly one: sub_40F770 takes the mutex, asserts CS via
 * sub_4045D4(2, 0), calls sub_4043D0(..., port 2), releases CS, unlocks.
 * Probe handshake, firmware chunks, calibration, register writes, EXEC,
 * ping and report reads all go through it.
 *
 * This driver had two. grape_probe_26494, grape_status_poll,
 * grape_cmd_34ad0, the 0x011F poke, grape_bootload_cmd and
 * grape_read_reports went through spi_sync, while grape_send_chunk_ex,
 * grape_cmd_2d54c and grape_ping went through direct SPI2 register PIO --
 * so a single bring-up sequence alternated between two transports with
 * different CS handling, different FIFO handling and different error
 * paths. grape_xfer also allocated a drain buffer, forcing full duplex on
 * frames stock sends one-way.
 *
 * Everything now funnels through grape_burst_ex(), which dispatches on
 * grape_use_spi so the PIO-versus-SPI-core comparison stays a single
 * one-variable experiment instead of a property of which function you
 * happened to call.
 */
static int grape_xfer(struct grape *n, const u8 *tx, u8 *rx, unsigned int len)
{
	return grape_burst_ex(n, tx, rx, len, GRAPE_CS_BEGIN | GRAPE_CS_END);
}

/* sub_2C87E — bootloader opcode whitelist */
static bool grape_opcode_known(u16 w);
static bool grape_looks_like_arm(const u8 *p, size_t n);
static void grape_bswap32_words(u8 *p, unsigned int len);
static u32 grape_sum32(const u8 *p, unsigned int len);

/*
 * The eight words sub_2C87E accepts, and only those.
 *
 * 0x4F81 used to be in this list. It is not in stock's:
 *
 *     return a1 == 6369 || a1 == 6817 || a1 == 7937 || a1 == 6593
 *         || a1 == 18553 || a1 == 19393 || a1 == 18793 || a1 == 19153;
 *
 * which is 0x18E1 0x1AA1 0x1F01 0x19C1 0x4879 0x4BC1 0x4969 0x4AD1.
 * So 0x4F81 is not a bootloader status the firmware recognises at
 * all -- it is a word stock never expects to see.
 *
 * That matters at sub_26494, which probes with 1A A1 + 18 E1 padding
 * and requires BOTH returned halfwords to pass this test before
 * sub_20E94 will download anything. Accepting 0x4F81 here let our
 * probe succeed where stock's would have refused, so we uploaded
 * firmware into a part stock would never have downloaded to.
 *
 * grape_bootloader_word() below still lists 0x4F81, because naming
 * what came back is useful even when it is not a legal status.
 */
static bool grape_opcode_known(u16 w)
{
	return w == 0x18e1 || w == 0x1aa1 || w == 0x1f01 || w == 0x19c1 ||
	       w == 0x4879 || w == 0x4bc1 || w == 0x4969 || w == 0x4ad1;
}

/*
 * Words the bootloader answers with.
 *
 * 0x4F81 says less than it looks like it says.
 *
 * It is not in sub_2C87E, the firmware own list of legal protocol
 * words, and it appears nowhere in the OSOS image. What it is not is
 * evidence about the application.
 *
 * The ping that provokes it is genuine. sub_182590 sends 490 as a
 * little-endian u32, then zeros, with a 16-bit checksum over the
 * first 14 bytes placed at offset 14, and reads 16 bytes back; a
 * reply is good when the same checksum over rx[0..13] equals
 * rx[14..15], and the pending length is then rx[1..2]. That is
 * exactly grape_ping, and sub_188FFC drives it with five retries.
 *
 * So a failed ping here means the checksum did not verify, not that
 * the part is in the bootloader. Where stock differs is what it does
 * with the result: sub_188FFC only reads a report when the pending
 * length is non-zero, and nothing gates registering the input device
 * on a ping at all -- sub_20490(1) simply arms ATTN on pad 38.
 */
static bool grape_status_is_bootloader(u16 w)
{
	return w == 0x4f81 || w == 0x4879 || w == 0x4bc1 ||
	       w == 0x4ad1 || w == 0x4969;
}

/* sub_26494 — 16↔16 1A A1 + 18 E1 pad; two rev16 words must be known */
static int grape_probe_26494(struct grape *n, const char *tag)
{
	u8 tx[GRAPE_FRAME_LEN];
	u8 rx[GRAPE_FRAME_LEN] = { 0 };
	unsigned int i;
	u16 w0, w1;
	int ret;

	tx[0] = 0x1a;
	tx[1] = 0xa1;
	for (i = 2; i < GRAPE_FRAME_LEN; i += 2) {
		tx[i] = 0x18;
		tx[i + 1] = 0xe1;
	}
	ret = grape_xfer(n, tx, rx, GRAPE_FRAME_LEN);
	w0 = (u16)((rx[0] << 8) | rx[1]);
	w1 = (u16)((rx[2] << 8) | rx[3]);
	grape_vinfo(n,
		 "26494 %s ret=%d words 0x%04x 0x%04x known=%d rx %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 tag, ret, w0, w1, grape_opcode_known(w0) && grape_opcode_known(w1),
		 rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
	/*
	 * The stock probe only rejects the part when the transfer worked and
	 * came back with words it does not recognise. A failed transfer is not
	 * a verdict, and treating it as one meant a single soft SPI error
	 * skipped the firmware download entirely.
	 */
	if (ret) {
		dev_warn(&n->spi->dev,
			 "26494 %s transfer %d; continuing to download\n",
			 tag, ret);
		return 0;
	}
	if (!grape_opcode_known(w0) || !grape_opcode_known(w1))
		return -EIO;
	return 0;
}

/* sub_3D5706 — TX 1A A1, RX 2, byteswap */
static int grape_status_poll(struct grape *n, u16 *status)
{
	u8 tx[2] = { 0x1a, 0xa1 };
	u8 rx[2] = { 0 };
	int ret;

	ret = grape_xfer(n, tx, rx, 2);
	if (ret)
		return ret;
	if (status)
		*status = (u16)((rx[0] << 8) | rx[1]); /* __rev16 of LE word */
	return 0;
}

static bool grape_fw_has_8740_hdr(const u8 *data, size_t size)
{
	return size >= 8 && data[0] == '8' && data[1] == '7' &&
	       data[2] == '4' && data[3] == '0';
}

static bool grape_fw_has_z2fw_hdr(const u8 *data, size_t size)
{
	u32 magic;

	if (size < 8)
		return false;
	magic = get_unaligned_le32(data);
	return magic == GRAPE_Z2FW_MAGIC;
}

/* Classify host grape file: full 8740 container vs ARM-only cut vs Z2FW. */
static void grape_fwfile_classify(struct grape *n, const u8 *data, size_t size)
{
	bool h8740 = grape_fw_has_8740_hdr(data, size);
	bool hz2 = grape_fw_has_z2fw_hdr(data, size);
	bool arm0 = size >= 4 && grape_looks_like_arm(data, size);
	bool arm400 = size >= 0x410 && grape_looks_like_arm(data + 0x400, 16);
	u32 le0c = (h8740 && size >= 0x10) ? get_unaligned_le32(data + 0xc) : 0;
	u8 rev = (h8740 && size >= 5) ? data[4] : 0;

	grape_vinfo(n,
		 "FWFILE size=%zu first16=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x has_8740=%d rev=%u le32(+0xc)=0x%x arm@0=%d arm@0x400=%d Z2FW=%d\n",
		 size,
		 size > 0 ? data[0] : 0, size > 1 ? data[1] : 0,
		 size > 2 ? data[2] : 0, size > 3 ? data[3] : 0,
		 size > 4 ? data[4] : 0, size > 5 ? data[5] : 0,
		 size > 6 ? data[6] : 0, size > 7 ? data[7] : 0,
		 size > 8 ? data[8] : 0, size > 9 ? data[9] : 0,
		 size > 10 ? data[10] : 0, size > 11 ? data[11] : 0,
		 size > 12 ? data[12] : 0, size > 13 ? data[13] : 0,
		 size > 14 ? data[14] : 0, size > 15 ? data[15] : 0,
		 h8740, rev, le0c, arm0, arm400, hz2);
	if (!h8740 && arm0)
		dev_warn(&n->spi->dev,
			 "FWFILE is ARM-only cut — grape file +350 is not touch calibration cal\n");
}

static void __maybe_unused grape_log_calcand(struct grape *n, const char *name,
			       const u8 *data, size_t size, unsigned int off)
{
	u8 tmp[GRAPE_FW_HDR_LEN];
	u32 s;

	if (size < off + GRAPE_FW_HDR_LEN) {
		grape_vinfo(n, "CALCAND %s off=%u OOB (file=%zu)\n",
			 name, off, size);
		return;
	}
	memcpy(tmp, data + off, GRAPE_FW_HDR_LEN);
	grape_bswap32_words(tmp, GRAPE_FW_HDR_LEN);
	s = grape_sum32(tmp, GRAPE_FW_HDR_LEN);
	grape_vinfo(n,
		 "CALCAND %s off=%u sum32=0x%08x first16=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x (post-bswap)\n",
		 name, off, s,
		 tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6],
		 tmp[7], tmp[8], tmp[9], tmp[10], tmp[11], tmp[12], tmp[13],
		 tmp[14], tmp[15]);
}

static void __maybe_unused grape_dump_calcands(struct grape *n, const u8 *data,
						size_t size)
{
	/* Diagnostic only — does not select a candidate for upload. */
	grape_log_calcand(n, "dec350", data, size, 350);
	grape_log_calcand(n, "hex350", data, size, 0x350);
	grape_log_calcand(n, "arm_plus_dec350", data, size, 0x400 + 350);
	grape_log_calcand(n, "arm_plus_hex350", data, size, 0x400 + 0x350);
}

static u32 grape_crc32_payload(const u8 *p, size_t len)
{
	return crc32_le(~0U, p, len) ^ ~0U;
}

static void grape_fw_audit(struct grape *n, const u8 *body, size_t body_len,
			    const char *tag)
{
	u32 crc;
	size_t pad;

	if (!body_len)
		return;
	crc = grape_crc32_payload(body, body_len);
	pad = body_len & 3u;
	grape_vinfo(n,
		 "FW audit %s %zuB arm=%d crc32=0x%08x pad=%zu\n",
		 tag, body_len, grape_looks_like_arm(body, body_len), crc, pad);
	if (body_len >= 16) {
		u32 m = get_unaligned_le32(body);
		u32 ln_le = get_unaligned_le32(body + 4);
		u32 ln_be = get_unaligned_be32(body + 4);
		u32 c_le = get_unaligned_le32(body + 8);

		if (m == GRAPE_Z2_MAGIC_5A5A || m == GRAPE_Z2_MAGIC_C3F5)
			grape_vinfo(n,
				 "  z2-dl hdr magic=0x%08x len_le=%u len_be=%u crc=0x%08x\n",
				 m, ln_le, ln_be, c_le);
	}
}

static int grape_build_z2_dl_hdr(u8 *hdr, const u8 *payload, size_t len,
				 u32 magic)
{
	u32 crc = grape_crc32_payload(payload, len);

	put_unaligned_le32(magic, hdr);
	put_unaligned_be32(len, hdr + 4);
	put_unaligned_le32(crc, hdr + 8);
	put_unaligned_le32(0, hdr + 12);
	return 0;
}

static u8 *grape_maybe_prepend_z2_hdr(struct grape *n, const u8 *body,
				       size_t body_len, size_t *out_len)
{
	u8 *buf;
	u32 magic;

	if (!prepend_z2_hdr || body_len < 4)
		return NULL;
	magic = prepend_z2_hdr == 2 ? GRAPE_Z2_MAGIC_C3F5 : GRAPE_Z2_MAGIC_5A5A;
	buf = kmalloc(GRAPE_Z2_HDR_LEN + body_len + 3, GFP_KERNEL);
	if (!buf)
		return NULL;
	grape_build_z2_dl_hdr(buf, body, body_len, magic);
	memcpy(buf + GRAPE_Z2_HDR_LEN, body, body_len);
	*out_len = GRAPE_Z2_HDR_LEN + body_len;
	if (*out_len & 3) {
		memset(buf + *out_len, 0, 4 - (*out_len & 3));
		*out_len = round_up(*out_len, 4);
	}
	grape_vinfo(n,
		 "prepended Z2 dl hdr magic=0x%08x total=%zu\n", magic, *out_len);
	return buf;
}

static int grape_wait_ack(struct grape *n, u16 expect, int retries)
{
	int i;
	u16 st = 0;

	for (i = 0; i < retries; i++) {
		if (grape_status_poll(n, &st) == 0 && st == expect)
			return 0;
		msleep(2);
	}
	dev_warn(&n->spi->dev, "ACK wait fail (want 0x%04x got 0x%04x)\n",
		 expect, st);
	return -ETIMEDOUT;
}

/*
 * S5LBox §6.5 / iOS AppleMultitouchZ2SPI MemRead:
 *   1C 73 + addr middle-endian + sum16(addr bytes)
 *   then 8-byte ATN 1A A1 18 E1×3; value at rx[2..5] middle-endian.
 */
static int grape_rdreg(struct grape *n, u32 addr, u32 *val)
{
	u8 tx[8] = { 0x1c, 0x73 };
	u8 rx[8] = { 0 };
	u8 atn_tx[8] = { 0x1a, 0xa1, 0x18, 0xe1, 0x18, 0xe1, 0x18, 0xe1 };
	u8 atn_rx[8] = { 0 };
	u16 csum;
	int ret;

	tx[2] = (addr >> 8) & 0xff;
	tx[3] = addr & 0xff;
	tx[4] = (addr >> 24) & 0xff;
	tx[5] = (addr >> 16) & 0xff;
	csum = grape_sum16(tx + 2, 4);
	tx[6] = (csum >> 8) & 0xff;
	tx[7] = csum & 0xff;

	ret = grape_xfer(n, tx, rx, 8);
	if (ret)
		return ret;
	ret = grape_xfer(n, atn_tx, atn_rx, 8);
	if (ret)
		return ret;
	if (val)
		*val = ((u32)atn_rx[2] << 8) | atn_rx[3] |
		       ((((u32)atn_rx[4] << 8) | atn_rx[5]) << 16);
	dev_dbg(&n->spi->dev,
		"RDREG 0x%08x = 0x%08x atn %02x %02x %02x %02x %02x %02x %02x %02x\n",
		addr, val ? *val : 0, atn_rx[0], atn_rx[1], atn_rx[2],
		atn_rx[3], atn_rx[4], atn_rx[5], atn_rx[6], atn_rx[7]);
	return 0;
}


static void grape_peek(struct grape *n, const char *tag)
{
	static const u32 addrs[] = {
		0x00000000, 0x0000d208, 0x0000e970, 0x00400200,
		0x0040f7f4, 0x0040fffc, 0x1000300c, 0x10008ffc,
	};
	unsigned int i;

	if (!grape_verbose)
		return;
	for (i = 0; i < ARRAY_SIZE(addrs); i++) {
		u32 v = 0;

		if (grape_rdreg(n, addrs[i], &v) == 0)
			grape_vinfo(n, "peek %s %08x=%08x\n", tag, addrs[i], v);
	}
}

/* HBPP MemRead, dest packing is B1,B0,B3,B2 — same as DATA offset. */
static int grape_rdmem(struct grape *n, u32 addr, u8 *buf, unsigned int len)
{
	unsigned int i;

	if (len & 3)
		return -EINVAL;
	for (i = 0; i < len; i += 4) {
		u32 v = 0;

		if (grape_rdreg(n, addr + i, &v))
			return -EIO;
		put_unaligned_le32(v, buf + i);
	}
	return 0;
}

/*
 * Prove whether 2D640 landed the ARM image at dest 0 or at the EXEC
 * word 0x00100018. Cal dest 0x400200 is a separate window.
 */
static void grape_fw_readback(struct grape *n, const char *tag)
{
	static const u32 addrs[] = {
		0x00000000, 0x00000018, 0x00100000, 0x00100018,
		0x00400000, 0x00400200,
	};
	u8 *buf;
	unsigned int i;

	/*
	 * Six 4 KB reads in the middle of the download sequence. Harmless to
	 * look at, but not something to put on the bus by default.
	 */
	if (!verbose)
		return;

	buf = kmalloc(0x1000, GFP_KERNEL);
	if (!buf)
		return;
	grape_vinfo(n, "GRAPE FW_READBACK %s:\n", tag);
	for (i = 0; i < ARRAY_SIZE(addrs); i++) {
		u32 crc100, crc1000;

		memset(buf, 0xa5, 0x1000);
		if (grape_rdmem(n, addrs[i], buf, 0x1000)) {
			dev_warn(&n->spi->dev,
				 "  addr=%08x RDREG fail\n", addrs[i]);
			continue;
		}
		crc100 = grape_crc32_payload(buf, 0x100);
		crc1000 = grape_crc32_payload(buf, 0x1000);
		grape_vinfo(n,
			 "  addr=%08x first32=%32ph crc100=0x%08x crc1000=0x%08x\n",
			 addrs[i], buf, crc100, crc1000);
	}
	kfree(buf);
}

static void grape_cal_readback(struct grape *n, const u8 *upload)
{
	u8 *buf;
	u32 crc_chip, crc_host;

	if (!verbose)
		return;

	buf = kmalloc(GRAPE_FW_HDR_LEN, GFP_KERNEL);
	if (!buf)
		return;
	if (grape_rdmem(n, GRAPE_CAL_DEST, buf, GRAPE_FW_HDR_LEN)) {
		dev_warn(&n->spi->dev, "cal readback RDREG fail @0x%08x\n",
			 GRAPE_CAL_DEST);
		kfree(buf);
		return;
	}
	crc_chip = grape_crc32_payload(buf, GRAPE_FW_HDR_LEN);
	crc_host = grape_crc32_payload(upload, GRAPE_FW_HDR_LEN);
	grape_vinfo(n,
		 "GRAPE CAL_READBACK @%08x first64=%32ph %32ph crc200=0x%08x host_crc=0x%08x match=%d\n",
		 GRAPE_CAL_DEST, buf, buf + 32, crc_chip, crc_host,
		 crc_chip == crc_host && !memcmp(buf, upload, GRAPE_FW_HDR_LEN));
	kfree(buf);
}

/* sub_20848(6593) */
static int grape_bootload_cmd(struct grape *n)
{
	u8 tx[GRAPE_FRAME_LEN];
	u8 rx[GRAPE_FRAME_LEN];
	unsigned int i;
	int ret;

	tx[0] = (GRAPE_BOOTLOAD_WORD >> 8) & 0xff;
	tx[1] = GRAPE_BOOTLOAD_WORD & 0xff;
	for (i = 2; i < GRAPE_FRAME_LEN; i += 2) {
		tx[i] = 0x18;
		tx[i + 1] = 0xe1;
	}
	memset(rx, 0, sizeof(rx));
	ret = grape_xfer(n, tx, rx, GRAPE_FRAME_LEN);
	grape_vinfo(n,
		 "bootload 6593 ret=%d rx %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 ret, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
	return ret;
}

/*
 * sub_3B9D0 dword swizzle into chunk payload: b0 b1 b2 b3 → b1 b0 b3 b2
 * (distinct from the full u32 byte-reverse used at FW+350).
 */
static void grape_grape_swizzle32(u8 *dst, const u8 *src, unsigned int len)
{
	unsigned int i;

	for (i = 0; i + 3 < len; i += 4) {
		dst[i]     = src[i + 1];
		dst[i + 1] = src[i];
		dst[i + 2] = src[i + 3];
		dst[i + 3] = src[i + 2];
	}
}

/* 273A0: full u32 reverse of the 512-byte cal window before 2D7A4. */
static void grape_bswap32_words(u8 *p, unsigned int len)
{
	unsigned int i;

	for (i = 0; i + 3 < len; i += 4) {
		u8 t0 = p[i], t1 = p[i + 1];

		p[i]     = p[i + 3];
		p[i + 1] = p[i + 2];
		p[i + 2] = t1;
		p[i + 3] = t0;
	}
}

static u32 grape_sum32(const u8 *p, unsigned int len)
{
	u32 s = 0;

	while (len--)
		s += *p++;
	return s;
}

/*
 * Family clue (iPhone 4S AppleMultitouchN1SPI): cal is a separate
 * multi-touch-calibration property starting "NI" (4e 49 …), not FW+350.
 * N31 RetailOS window has been observed as 4e 49 02 01 (vs 4S 4e 49 01 01).
 * Host/touch calibration order is checked BEFORE the RetailOS u32-reverse into win[].
 */
static bool grape_cal_looks_ni(const u8 *p)
{
	return p && p[0] == 0x4e && p[1] == 0x49;
}

/*
 * OSOS sub_273A0: copy touch calibration[350 : 350+0x200], reverse each u32 in that
 * copy only (do not mutate the 0x560 object).
 */
static int grape_prepare_cal_from_touch_cal(struct grape *n, const u8 *touch_cal,
					size_t touch_cal_len)
{
	const u8 *raw;
	u32 sum;

	if (touch_cal_len != GRAPE_TOUCH_CAL_LEN) {
		dev_err(&n->spi->dev, "touch calibration bad size: got=%zu want=0x%x\n",
			touch_cal_len, GRAPE_TOUCH_CAL_LEN);
		return -EINVAL;
	}

	memcpy(n->touch_cal, touch_cal, GRAPE_TOUCH_CAL_LEN);
	n->have_touch_cal = true;

	raw = n->touch_cal + GRAPE_FW_HDR_OFF;
	memcpy(n->cal_upload, raw, GRAPE_FW_HDR_LEN);
	grape_vinfo(n,
		 "cal +350 raw head %02x %02x %02x %02x%s\n",
		 raw[0], raw[1], raw[2], raw[3],
		 grape_cal_looks_ni(raw) ? " (NI family — good)" :
		 " (not NI — suspect vs 4S/IOReg cal)");
	grape_bswap32_words(n->cal_upload, GRAPE_FW_HDR_LEN);
	sum = grape_sum32(n->cal_upload, GRAPE_FW_HDR_LEN);
	grape_vinfo(n,
		 "Grape touch calibration cal prepared: off=%u len=0x%x sum32=0x%08x upload_first32=%32ph\n",
		 GRAPE_FW_HDR_OFF, GRAPE_FW_HDR_LEN, sum, n->cal_upload);
	if (!sum) {
		dev_err(&n->spi->dev,
			"touch calibration +350 window is all zeros — not a usable cal\n");
		n->have_cal = false;
		return -EINVAL;
	}
	n->have_cal = true;
	return 0;
}

/*
 * OSOS sub_564: desc = sub_A34(24) = 0x2202FE18
 *   desc[0] == 0x53797349
 *   memcpy(0x08A8B510, desc[1], 0x560)
 * Linux reads the live descriptor if boot preserved that SRAM.
 */
static int grape_load_touch_cal_from_a34(struct grape *n)
{
	void __iomem *desc_io;
	void __iomem *src_io;
	u32 magic;
	u32 ptr;
	u8 *tmp;
	int ret;

	if (!cal_try_a34)
		return -ENOENT;

	desc_io = ioremap(GRAPE_A34_TOUCH_CAL_DESC, 8);
	if (!desc_io)
		return -ENOMEM;

	magic = readl(desc_io);
	ptr = readl(desc_io + 4);
	iounmap(desc_io);

	grape_vinfo(n,
		 "A34 touch calibration descriptor: magic=0x%08x ptr=0x%08x\n",
		 magic, ptr);

	if (magic != GRAPE_TOUCH_CAL_MAGIC) {
		dev_warn(&n->spi->dev,
			 "A34 touch calibration missing: magic=0x%08x want=0x%08x\n",
			 magic, GRAPE_TOUCH_CAL_MAGIC);
		return -ENOENT;
	}
	if (!ptr) {
		dev_warn(&n->spi->dev, "A34 touch calibration pointer is NULL\n");
		return -ENOENT;
	}

	src_io = ioremap(ptr, GRAPE_TOUCH_CAL_LEN);
	if (!src_io)
		return -ENOMEM;

	tmp = kmalloc(GRAPE_TOUCH_CAL_LEN, GFP_KERNEL);
	if (!tmp) {
		iounmap(src_io);
		return -ENOMEM;
	}

	memcpy_fromio(tmp, src_io, GRAPE_TOUCH_CAL_LEN);
	iounmap(src_io);

	grape_vinfo(n,
		 "A34 touch calibration read: ptr=0x%08x len=0x%x first32=%32ph calraw_first16=%16ph\n",
		 ptr, GRAPE_TOUCH_CAL_LEN, tmp, tmp + GRAPE_FW_HDR_OFF);

	ret = grape_prepare_cal_from_touch_cal(n, tmp, GRAPE_TOUCH_CAL_LEN);
	kfree(tmp);
	return ret;
}

/*
 * U-Boot copies the 0x560 touch calibration object to reserved DRAM and publishes
 * apple,n31-touch_cal-addr / apple,n31-touch_cal-size on /chosen. That copy is the
 * safe address — never the original A34 pointer.
 */
static int grape_load_touch_cal_from_dt(struct grape *n)
{
	struct device_node *chosen;
	u32 addr;
	u32 size;
	void *p;
	u8 *tmp;
	int ret;

	if (!cal_try_dt)
		return -ENOENT;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen)
		return -ENOENT;

	ret = of_property_read_u32(chosen, "apple,n31-touch_cal-addr", &addr);
	if (ret)
		goto out;

	ret = of_property_read_u32(chosen, "apple,n31-touch_cal-size", &size);
	if (ret)
		goto out;

	grape_vinfo(n, "DT touch_cal: addr=0x%08x size=0x%x\n", addr, size);

	if (!addr || size != GRAPE_TOUCH_CAL_LEN) {
		ret = -EINVAL;
		goto out;
	}

	p = memremap(addr, size, MEMREMAP_WB);
	if (!p) {
		ret = -ENOMEM;
		goto out;
	}

	tmp = kmemdup(p, size, GFP_KERNEL);
	memunmap(p);
	if (!tmp) {
		ret = -ENOMEM;
		goto out;
	}

	grape_vinfo(n,
		 "DT touch_cal read: addr=0x%08x len=0x%x first32=%32ph calraw_first16=%16ph\n",
		 addr, size, tmp, tmp + GRAPE_FW_HDR_OFF);

	ret = grape_prepare_cal_from_touch_cal(n, tmp, size);
	kfree(tmp);

out:
	of_node_put(chosen);
	return ret;
}

static int grape_acquire_touch_cal_cal(struct grape *n)
{
	int ret;

	if (n->have_cal)
		return 0;

	ret = grape_load_touch_cal_from_dt(n);
	if (!ret)
		return 0;

	grape_vinfo(n, "DT touch_cal unavailable: %d; trying A34 live\n",
		 ret);

	ret = grape_load_touch_cal_from_a34(n);
	if (!ret)
		return 0;

	dev_err(&n->spi->dev,
		"No touch calibration calibration from DT or A34; not registering input\n");
	return ret;
}

/* Optional fmss FTL export — grape firmware only, not touch calibration cal. */
static bool (*grape_ftl_present_fn)(void);
static int (*grape_ftl_read_fn)(u64 logical_sector, void *buf);
static bool grape_ftl_inited;

static void grape_ftl_init_once(void)
{
	if (grape_ftl_inited)
		return;
	grape_ftl_inited = true;
	grape_ftl_present_fn = symbol_get(nand_ftl_present);
	grape_ftl_read_fn = symbol_get(nand_ftl_read_sector);
}

static bool grape_ftl_ready(void)
{
	grape_ftl_init_once();
	return grape_ftl_present_fn && grape_ftl_read_fn &&
	       grape_ftl_present_fn();
}

/*
 * Walk FTL for Apple 8740 / gpfw IMG1. Returns kmalloc'd buffer + size.
 * Caller kfree() on success. This is the ARM app, not touch calibration cal.
 */
/*
 * Kept with no caller: the transliteration of the gpfw read, retained so the
 * shape is not lost if a unit ever turns up with an MBR at LBA 0 and an
 * APPLE_MDFW partition. On this device LBA 0 is the FAT32 VBR, so it can
 * only fail. See the note on fw_prefer_ftl.
 */
static u8 __maybe_unused *grape_try_gpfw_from_ftl(struct device *dev,
						   size_t *out_len)
{
	u8 *sec, *buf = NULL;
	u64 lba, end;
	unsigned int off;
	size_t need, got;
	u32 body_sz;

	if (!grape_ftl_ready())
		return NULL;

	sec = kmalloc(GRAPE_FTL_SECTOR_SIZE, GFP_KERNEL);
	if (!sec)
		return NULL;

	end = min_t(u64, cal_ftl_start + cal_ftl_count, 256ULL);
	for (lba = 0; lba < end; lba++) {
		if (grape_ftl_read_fn(lba, sec))
			continue;
		for (off = 0; off + 0x410 <= GRAPE_FTL_SECTOR_SIZE; off += 4) {
			if (memcmp(sec + off, "8740", 4))
				continue;
			body_sz = get_unaligned_le32(sec + off + 0x0c);
			if (!body_sz || body_sz > 1024 * 1024)
				continue;
			need = 0x400 + round_up(body_sz, 16);
			buf = kmalloc(need, GFP_KERNEL);
			if (!buf)
				goto out;
			memcpy(buf, sec + off, min_t(size_t, need,
						    GRAPE_FTL_SECTOR_SIZE - off));
			got = min_t(size_t, need, GRAPE_FTL_SECTOR_SIZE - off);
			while (got < need && lba + 1 < end) {
				lba++;
				if (grape_ftl_read_fn(lba, sec)) {
					kfree(buf);
					buf = NULL;
					goto out;
				}
				memcpy(buf + got, sec,
				       min_t(size_t, need - got,
					       GRAPE_FTL_SECTOR_SIZE));
				got += min_t(size_t, need - got,
					     GRAPE_FTL_SECTOR_SIZE);
			}
			if (got >= 0x410) {
				grape_dev_vinfo(dev,
					 "gpfw/8740 from FTL lba=%llu off=%u need=%zu got=%zu rev=%u\n",
					 lba, off, need, got, buf[7]);
				*out_len = got;
				goto out;
			}
			kfree(buf);
			buf = NULL;
		}
	}

out:
	kfree(sec);
	return buf;
}

static int grape_acquire_fw(struct device *dev, const u8 **data,
			     size_t *size, const struct firmware **fw_out,
			     u8 **kbuf_out, bool *from_ftl)
{
	*fw_out = NULL;
	*kbuf_out = NULL;
	*from_ftl = false;

	/*
	 * No gpfw attempt: see the note on fw_prefer_ftl above. The store is
	 * not present on this device, so the host file is the only source.
	 */
	if (!fw_allow_file)
		return -ENOENT;
	if (request_firmware(fw_out, "apple/grape.bin", dev) ||
	    !*fw_out)
		return -ENOENT;
	*from_ftl = false;
	*data = (*fw_out)->data;
	*size = (*fw_out)->size;

	/*
	 * The firmware blob is an IMG1 file, and "8740" is its SoC code.
	 *
	 * sub_1A640 tests the magic with sub_440CD0, which is memcmp and
	 * returns 0 on a match:
	 *
	 *     if (sub_440CD0(buf, "8740", 4)) { ptr = buf; len = size; }
	 *     else if (sub_204E0(buf, size, &len, &ptr)) { ... }
	 *
	 * so the whole-blob arm is taken when the magic is ABSENT, and a blob
	 * that does start with "8740" goes to sub_204E0. That function is an
	 * IMG1 parser, which the freemyipod IMG1 notes and wInd3x pkg/image
	 * describe independently:
	 *
	 *     +0  magic[4]      SoC code, "8740" here
	 *     +4  version[3]    "2.0"
	 *     +7  format        4 = X509_SIGNED
	 *     +8  entrypoint
	 *     +12 body_length
	 *     +16 data_length
	 *     +20 cert_offset
	 *     +24 cert_length
	 *
	 * with the body at 0x400 for this device. sub_204E0 returns exactly
	 * that: ptr = buf + 1024, len = *(u32 *)(buf + 12). Its guards are
	 * IMG1 sanity checks -- its 768 is the certificate length and its 128
	 * is the 0x80 body signature.
	 *
	 * This blob checks out field for field: body 59760 at 0x400, then a
	 * 0x80 signature, then 768 of certificate, which is 61680 exactly.
	 * data_length equals body + signature + certificate, and cert_offset
	 * equals body + signature.
	 *
	 * body_length is rounded up to the AES block size before use. It is
	 * already 16-aligned here so nothing moves, but an image whose body
	 * is not would otherwise be uploaded short by up to 15 bytes.
	 *
	 * Earlier versions uploaded from offset 0 -- once truncated to
	 * body_length, once whole -- so the part was handed the IMG1 header
	 * where it expected the vector table. Chunks still acked 0x4BC1 and
	 * EXEC still completed, which is why it looked like a good download
	 * that would not start.
	 */
	if (*size > GRAPE_IMG1_BODY_OFF && !memcmp(*data, "8740", 4)) {
		u32 entry = get_unaligned_le32(*data + 8);
		u32 body = get_unaligned_le32(*data + 12);
		u32 data_len = get_unaligned_le32(*data + 16);
		u32 cert_off = get_unaligned_le32(*data + 20);
		u32 aligned;

		/* IMG1 body_length rounds up to the AES block size. */
		if (body & 0xf) {
			dev_info(dev, "IMG1 body %u rounded up to %u\n",
				 body, (body + 0xf) & ~0xfu);
			body = (body + 0xf) & ~0xfu;
		}
		aligned = (body + 0xf) & ~0xfu;

		/* sub_204E0 guards, in its order. */
		if (body && data_len >= aligned + GRAPE_IMG1_CERT_LEN &&
		    cert_off >= aligned + GRAPE_IMG1_SIG_LEN &&
		    !(cert_off & 0xf) && entry <= aligned &&
		    *size >= GRAPE_IMG1_BODY_OFF + body) {
			dev_info(dev,
				 "IMG1 8740 v%c.%c fmt %u: body %u at +%u, entry 0x%08x, of %zu\n",
				 (*data)[4], (*data)[6], (*data)[7], body,
				 GRAPE_IMG1_BODY_OFF, entry, *size);
			*data += GRAPE_IMG1_BODY_OFF;
			*size = body;
		} else {
			dev_warn(dev,
				 "IMG1 header fails sub_204E0 guards (body=%u data=%u cert_off=%u entry=0x%08x); using the whole blob\n",
				 body, data_len, cert_off, entry);
		}
	} else {
		dev_info(dev, "fw has no 8740 IMG1 magic; uploading all %zu bytes\n",
			 *size);
	}
	return 0;
}

static void grape_release_fw(const struct firmware *fw, u8 *kbuf)
{
	/* Exactly one of the decoded copy and the firmware blob is live. */
	if (!kbuf && fw)
		release_firmware(fw);
	kfree(kbuf);
}

/*
 * 2D7A4 payload → controller @ 0x00400200.
 * Cal is the transformed A34 touch calibration window only.
 */
static int grape_load_cal_window(struct grape *n, u8 *win)
{
	int ret;

	ret = grape_acquire_touch_cal_cal(n);
	if (ret)
		return ret;
	memcpy(win, n->cal_upload, GRAPE_FW_HDR_LEN);
	return 0;
}

/*
 * sub_2D640 / 2D7A4 + trampoline sub_35C1C → sub_3B9D0:
 *   frame[0..1]  18 E1
 *   body @ +2:
 *     30 01
 *     word_count hi/lo = (len>>10),(len>>2)
 *     dest B1 B0 B3 B2
 *     u16 byte-sum of previous 6 body bytes (words+dest), BE
 *     payload u32s swizzled B1 B0 B3 B2
 *     u32 byte-sum of swizzled payload, stored B1 B0 B3 B2
 * SPI len = payload_len + 16; max payload 0x1FF0; ACK 0x4BC1 (retry ≤5).
 *
 * Expected prefixes (exact glass check):
 *   FW chunk0 dest=0 len=0x1FF0:
 *     18 E1 30 01 07 FC 00 00 00 00 01 03
 *   CAL chunk0 dest=0x00400200 len=0x200:
 *     18 E1 30 01 00 80 02 00 00 40 00 C2
 */
static unsigned int grape_build_upload_frame(u8 *buf, u32 dest,
					      const u8 *src, unsigned int len)
{
	u16 hdr_sum;
	u32 payload_sum;

	buf[0] = 0x18;
	buf[1] = 0xe1;
	buf[2] = 0x30;
	buf[3] = 0x01;
	/* word_count = len/4 as big-endian u16 via (len>>10),(len>>2) */
	buf[4] = (len >> 10) & 0xff;
	buf[5] = (len >> 2) & 0xff;
	/* dest swizzle B1 B0 B3 B2 */
	buf[6] = (dest >> 8) & 0xff;
	buf[7] = dest & 0xff;
	buf[8] = (dest >> 24) & 0xff;
	buf[9] = (dest >> 16) & 0xff;
	hdr_sum = grape_sum16(buf + 4, 6);
	buf[10] = (hdr_sum >> 8) & 0xff;
	buf[11] = hdr_sum & 0xff;

	grape_grape_swizzle32(buf + 12, src, len);

	payload_sum = grape_sum32(buf + 12, len);
	buf[12 + len]     = (payload_sum >> 8) & 0xff;
	buf[12 + len + 1] = payload_sum & 0xff;
	buf[12 + len + 2] = (payload_sum >> 24) & 0xff;
	buf[12 + len + 3] = (payload_sum >> 16) & 0xff;

	return len + GRAPE_HDR_LEN;
}

/* Glass/oracle prefixes from RetailOS 2D640 / 2D7A4 — fail loud if wrong. */
static void grape_check_upload_prefix(struct grape *n, u32 dest,
				       unsigned int len, const u8 *tx)
{
	static const u8 fw0[12] = {
		0x18, 0xe1, 0x30, 0x01, 0x07, 0xfc, 0x00, 0x00,
		0x00, 0x00, 0x01, 0x03
	};
	static const u8 cal0[12] = {
		0x18, 0xe1, 0x30, 0x01, 0x00, 0x80, 0x02, 0x00,
		0x00, 0x40, 0x00, 0xc2
	};

	if (dest == 0 && len == GRAPE_CHUNK_MAX && memcmp(tx, fw0, 12)) {
		dev_err(&n->spi->dev,
			"FW_UPLOAD prefix MISMATCH want 18 e1 30 01 07 fc 00 00 00 00 01 03 got %12ph\n",
			tx);
	}
	if (dest == GRAPE_CAL_DEST && len == GRAPE_FW_HDR_LEN &&
	    memcmp(tx, cal0, 12)) {
		dev_err(&n->spi->dev,
			"CAL_UPLOAD prefix MISMATCH want 18 e1 30 01 00 80 02 00 00 40 00 c2 got %12ph\n",
			tx);
	}
}

static void grape_log_upload_prefix(struct grape *n, const char *tag,
				     unsigned int chunk_idx, u32 dest,
				     unsigned int len, const u8 *tx,
				     unsigned int xfer_len, u16 ack, int ack_ret)
{
	grape_vinfo(n,
		 "GRAPE %s chunk=%u dest=%08x len=%04x xfer=%u ACK=0x%04x ret=%d\n",
		 tag, chunk_idx, dest, len, xfer_len, ack, ack_ret);
	grape_vinfo(n,
		 "  tx[0:16] = %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7],
		 tx[8], tx[9], tx[10], tx[11],
		 xfer_len > 12 ? tx[12] : 0, xfer_len > 13 ? tx[13] : 0,
		 xfer_len > 14 ? tx[14] : 0, xfer_len > 15 ? tx[15] : 0);
}

static void grape_dump_hbpp_tx(struct grape *n, const char *tag,
				const u8 *raw, unsigned int chunk_idx,
				unsigned int dest, unsigned int chunk_len,
				const u8 *tx, unsigned int xfer_len, u16 ack,
				int ack_ret)
{
	unsigned int last_off;

	grape_log_upload_prefix(n, tag, chunk_idx, dest, chunk_len,
				 tx, xfer_len, ack, ack_ret);
	if (raw && chunk_len >= 64)
		grape_vinfo(n, "  raw first64=%32ph %32ph\n",
			 raw, raw + 32);
	else if (raw)
		grape_vinfo(n, "  raw first%u=%*ph\n",
			 chunk_len, chunk_len, raw);
	if (xfer_len >= 96)
		grape_vinfo(n,
			 "  tx first96=%32ph %32ph %32ph\n",
			 tx, tx + 32, tx + 64);
	else if (xfer_len > 16)
		grape_vinfo(n, "  tx first%u=%*ph\n",
			 xfer_len, xfer_len, tx);
	if (xfer_len >= 32) {
		last_off = xfer_len - 32;
		grape_vinfo(n, "  tx last32=%32ph\n", tx + last_off);
	}
}

static int grape_send_chunk_ex(struct grape *n, const u8 *data,
				unsigned int dest, unsigned int len,
				unsigned int cs_flags, bool dump,
				const char *tag, unsigned int chunk_idx,
				unsigned int file_off)
{
	u8 *buf;
	unsigned int xfer_len;
	int ret = -EIO, try, ack_ret = -ETIMEDOUT;
	u16 ack = 0;

	if (!len || len > GRAPE_CHUNK_MAX || (len & 3))
		return -EINVAL;

	xfer_len = len + GRAPE_HDR_LEN;
	buf = kzalloc(xfer_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (grape_build_upload_frame(buf, dest, data, len) != xfer_len) {
		kfree(buf);
		return -EINVAL;
	}
	grape_check_upload_prefix(n, dest, len, buf);

	/* Z2 SEND_BLOB: spi_sync keeps CS down for whole HBPP frame. */
	for (try = 0; try < 5; try++) {
		if (chunk_spi)
			ret = grape_xfer(n, buf, NULL, xfer_len);
		else if (n->blob16)
			ret = grape_burst_u16_ex(n, buf, NULL,
						  xfer_len, cs_flags);
		else
			ret = grape_burst_ex(n, buf, NULL, xfer_len, cs_flags);
		if (ret)
			continue;
		/* 1A A1 → 2 bytes → rev16; expect 0x4BC1 */
		ack_ret = grape_wait_ack(n, GRAPE_ACK_CHUNK, 8);
		if (ack_ret == 0) {
			ack = GRAPE_ACK_CHUNK;
			if (dump)
				grape_dump_hbpp_tx(n, tag, data, chunk_idx,
						    dest, len, buf, xfer_len,
						    ack, ack_ret);
			else if (!chunk_idx)
				grape_log_upload_prefix(n, tag, chunk_idx,
							 dest, len, buf,
							 xfer_len, ack,
							 ack_ret);
			kfree(buf);
			return 0;
		}
		if (n->blob16 && chunk_idx && try == 0) {
			n->blob16 = false;
			grape_vinfo(n,
				 "16-bit DATA no 4BC1 — falling back to 8-bit PIO\n");
		}
	}
	if (dump || !chunk_idx)
		grape_dump_hbpp_tx(n, tag, data, chunk_idx, dest, len, buf,
				    xfer_len, ack, ack_ret);
	kfree(buf);
	return -EIO;
}

static int grape_send_chunk(struct grape *n, const u8 *data,
			     unsigned int dest, unsigned int len, bool dump,
			     unsigned int file_off, const char *tag,
			     unsigned int chunk_idx)
{
	return grape_send_chunk_ex(n, data, dest, len,
				    GRAPE_CS_BEGIN | GRAPE_CS_END, dump,
				    tag, chunk_idx, file_off);
}

static int grape_send_blob(struct grape *n, const u8 *data, unsigned int len,
			    unsigned int dest_off)
{
	unsigned int off = 0;
	unsigned int chunk_idx = 0;
	u8 pad[4];
	const char *tag;
	bool is_cal = (dest_off == GRAPE_CAL_DEST);

	tag = is_cal ? "CAL_UPLOAD" : "FW_UPLOAD";

	while (off < len) {
		unsigned int chunk = min_t(unsigned int, len - off, GRAPE_CHUNK_MAX);
		int ret;
		bool last, dump;

		/* RetailOS always transfers whole words */
		if (chunk & 3)
			chunk &= ~3u;
		if (!chunk) {
			memset(pad, 0, sizeof(pad));
			memcpy(pad, data + off, len - off);
			return grape_send_chunk(n, pad, dest_off + off, 4,
						 true, off, tag, chunk_idx);
		}
		last = (off + chunk >= len);
		/* Always dump first + last FW chunk and the sole cal chunk. */
		dump = (off == 0) || last || is_cal;
		ret = grape_send_chunk(n, data + off, dest_off + off, chunk,
					dump, off, tag, chunk_idx);
		if (ret)
			return ret;
		off += chunk;
		chunk_idx++;
	}
	return 0;
}

/* sub_34AD0(a1,a2,a3) — TX 1E 33 + 12-byte pack + sum16, expect ACK 0x4AD1 */
static int grape_cmd_34ad0(struct grape *n, u32 a1, u32 a2, u32 a3)
{
	u8 tx[16];
	u8 rx[16];
	u8 body[12];
	u16 csum;
	int ret;

	tx[0] = 0x1e;
	tx[1] = 0x33;

	/* Packing from Hex-Rays sub_34AD0 */
	body[0] = (a1 >> 8) & 0xff;
	body[1] = a1 & 0xff;
	body[2] = (a1 >> 24) & 0xff;
	body[3] = (a1 >> 16) & 0xff;
	body[4] = (a3 >> 8) & 0xff;
	body[5] = a3 & 0xff;
	body[6] = (a3 >> 24) & 0xff;
	body[7] = (a3 >> 16) & 0xff;
	body[8] = (a2 >> 8) & 0xff;
	body[9] = a2 & 0xff;
	body[10] = (a2 >> 24) & 0xff;
	body[11] = (a2 >> 16) & 0xff;
	csum = grape_sum16(body, 12);
	memcpy(tx + 2, body, 12);
	tx[14] = (csum >> 8) & 0xff;
	tx[15] = csum & 0xff;

	/* 34AD0: 40F770 16↔16 then 3D5706 == 0x4AD1 */
	ret = grape_xfer(n, tx, rx, 16);
	if (ret)
		return ret;
	return grape_wait_ack(n, GRAPE_ACK_34AD0, 8);
}

/* sub_2D5B0 post-download */
static int grape_post_download(struct grape *n)
{
	u8 tx[2], rx[2];
	u16 st = 0;
	int ret, i;

	static const struct {
		u32 a1, a2, a3;
	} pokes[] = {
		/* 2D5B0: ldr 0x1000300C, then +0x50 / +0x4C / -0x0C */
		{ 0x1000300c, 5859, (u32)-1 },
		{ 0x1000305c, 32, (u32)-1 },
		{ 0x10003058, 6, (u32)-1 },
		{ 0x10003000, 3, (u32)-1 },
	};

	for (i = 0; i < ARRAY_SIZE(pokes); i++) {
		u32 rb = 0;

		ret = grape_cmd_34ad0(n, pokes[i].a1, pokes[i].a2, pokes[i].a3);
		grape_vinfo(n, "34AD0[%d] %d\n", i, ret);
		if (ret)
			return ret;
		/*
		 * 4AD1 acknowledges the write without echoing it, so the only
		 * way to see the value is a separate RDREG. Stock sub_2D5B0
		 * does not do this, and it inserts a transfer between pokes
		 * that the part was not told to expect. Verbose runs only.
		 */
		if (verbose && grape_rdreg(n, pokes[i].a1, &rb) == 0)
			grape_vinfo(n,
				 "34AD0[%d] RDREG 0x%08x -> 0x%08x (wrote %u)\n",
				 i, pokes[i].a1, rb, pokes[i].a2);
	}

	/*
	 * Stock gates everything after this point on the reply:
	 *
	 *   if (!sub_40F770(&v8, 2, &v10, 2)) { sub_410522(65);
	 *       if (!sub_3D5706(&v9, ...)) return 1; }
	 *   return 0;
	 *
	 * We were throwing both answers away. grape_xfer returning 0
	 * only says the SPI transfer completed, and grape_status_poll
	 * below returns 0 for any status at all, so post_download always
	 * reported success and we went on to EXEC no matter what the part
	 * said. This is the last handshake before the application is
	 * supposed to start, and its reply has never been looked at.
	 */
	put_unaligned_le16(GRAPE_POST_POKE, tx);
	ret = grape_xfer(n, tx, rx, 2);
	if (ret)
		return ret;
	st = (u16)((rx[0] << 8) | rx[1]);
	dev_info(&n->spi->dev,
		 "011F reply %02x %02x (0x%04x)%s\n",
		 rx[0], rx[1], st,
		 grape_opcode_known(st) ? " known" : " UNKNOWN");
	st = 0;
	msleep(65);
	/* 2D5B0: 3D5706 success only — does not require 0x4BC1 */
	if (grape_status_poll(n, &st) != 0)
		return -EIO;

	dev_info(&n->spi->dev, "post-poke 1AA1 status 0x%04x%s\n",
		 st, grape_opcode_known(st) ? " known" : " UNKNOWN");

	/*
	 * Refuse to EXEC on an unrecognised status when post_poke_strict
	 * is set. Off by default so a run still reaches EXEC and we can
	 * see both halves before deciding which status stock treats as a
	 * pass.
	 */
	if (post_poke_strict && !grape_opcode_known(st)) {
		dev_warn(&n->spi->dev,
			 "post-poke status 0x%04x rejected; not running EXEC\n",
			 st);
		return -EIO;
	}
	n->requestcal_done = true;
	return 0;
}

/* sub_2D54C — 12↔12: 1D 53 + two LE u32 + sum16 */
/*
 * Neither implementation reconfigures SPI2 for EXEC -- go_spi_setup
 * defaults to 0 here, and stock's sub_2D54C goes through sub_40F770
 * like every other command, same channel-2 bracket and all. But the
 * part stops driving MISO from EXEC onwards, so the question is
 * whether the controller's own state moves underneath us. Measure it
 * either side of the transfer rather than reasoning about it.
 */
static void grape_spi2_dump(struct grape *n, const char *when)
{
	if (!n->spi2)
		return;
	dev_info(&n->spi->dev,
		 "SPI2 %-6s ctrl=%08x setup=%08x status=%08x pin=%08x clkdiv=%08x\n",
		 when,
		 readl(n->spi2 + SPI2_CTRL),
		 readl(n->spi2 + SPI2_SETUP),
		 readl(n->spi2 + SPI2_STATUS),
		 readl(n->spi2 + SPI2_PIN),
		 readl(n->spi2 + SPI2_CLKDIV));
}

static int grape_cmd_2d54c_raw(struct grape *n, u32 word0, u32 word1)
{
	u8 tx[12] = { 0x1d, 0x53 };
	u8 rx[12] = { 0 };
	u16 csum;
	u32 saved_setup = 0;
	int ret;

	put_unaligned_le32(word0, tx + 2);
	put_unaligned_le32(word1, tx + 6);
	csum = grape_sum16(tx + 2, 8);
	tx[10] = (csum >> 8) & 0xff;
	tx[11] = csum & 0xff;
	if (n->spi2) {
		grape_spi2_fifo_flush(n);
		if (go_spi_setup > 0) {
			saved_setup = readl(n->spi2 + SPI2_SETUP);
			writel((u32)go_spi_setup, n->spi2 + SPI2_SETUP);
			grape_vinfo(n, "2D54C GO SETUP 0x%x (was 0x%x)\n",
				 go_spi_setup, saved_setup);
		}
	}
	grape_spi2_dump(n, "pre");
	if (go_xfer == 2)
		ret = grape_xfer(n, tx, rx, 12);
	else if (go_xfer == 1)
		ret = grape_burst_u16(n, tx, rx, 12);
	else
		ret = grape_burst(n, tx, rx, 12);
	grape_spi2_dump(n, "post");
	if (saved_setup)
		writel(saved_setup, n->spi2 + SPI2_SETUP);
	/*
	 * No settle here. sub_273A0 waits ONCE, 40 ms, after a successful
	 * EXEC. This helper used to sleep, its caller slept again, and probe
	 * slept a third time before the runtime ping -- 120 ms in normal
	 * operation for a 40 ms wait. The transfer helper transfers; the
	 * state-transition owner owns the delay.
	 */
	grape_spi2_dump(n, "settle");
	grape_vinfo(n,
		 "2D54C %08x %08x ret=%d xfer=%d rx %02x %02x %02x %02x %02x %02x\n",
		 word0, word1, ret, go_xfer, rx[0], rx[1], rx[2], rx[3],
		 rx[4], rx[5]);
	return ret;
}

static void __maybe_unused grape_drain(struct grape *n, unsigned int bytes)
{
	u8 tx[GRAPE_FRAME_LEN] = { 0 };
	u8 rx[GRAPE_FRAME_LEN] = { 0 };
	unsigned int nxf = bytes < GRAPE_FRAME_LEN ? bytes : GRAPE_FRAME_LEN;

	/* Optional post-fail diagnostics only — never call on EXEC path. */
	grape_burst(n, tx, rx, nxf);
}

static void grape_pre_exec_verify(struct grape *n)
{
	static const u32 addrs[] = {
		0x00000000, 0x00000004, 0x00000008, 0x00000020,
		0x00400200, 0x00400204, 0x004003fc,
	};
	unsigned int i;

	if (!verbose)
		return;

	for (i = 0; i < ARRAY_SIZE(addrs); i++) {
		u32 v = 0;

		if (grape_rdreg(n, addrs[i], &v) == 0)
			grape_vinfo(n, "pre-EXEC RDREG 0x%08x=0x%08x\n",
				 addrs[i], v);
	}
}

/*
 * sub_2D54C — one-shot EXEC packet only.
 * Do NOT poll 1A A1 / drain after EXEC: that keeps speaking HBPP across the
 * bootloader→runtime boundary. Success is proven only by 182590 ping csum.
 */
static int grape_cmd_2d54c(struct grape *n)
{
	int ret;

	grape_pre_exec_verify(n);
	ret = grape_cmd_2d54c_raw(n, exec_addr, exec_word1);
	if (!ret) {
		n->exec_sent = true;
		/* Stock waits 40 ms here before touching the part again. */
		msleep(GRAPE_EXEC_SETTLE_MS);
	}
	return ret;
}




#define S5L8740_AES_PHYS	0x38c00000UL

/*
 * Touch FW GID decrypt (OSOS sub_422FFA / sub_204E0):
 *   MMIO @ 0x38C00000 AES, keysel=1 (GID), CBC IV=0, CFG=0xE|enc.
 *   26CCC verifies with 16-byte encrypt; 204E0 decrypts full ARM @ +0x400
 *   for 8740 rev 3 only. grape.bin on DFU is usually pre-decrypted-cut.
 * force_gid=1 tries 422FFA even when loading plaintext blob (bring-up).
 */
static int grape_422ffa_mmio(struct device *dev, u8 *buf, unsigned int len,
			      bool encrypt)
{
	struct device *aes_dev;
	void __iomem *aes;
	struct clk *clk = NULL;
	dma_addr_t phys;
	u32 irq = 0;
	int ret;

	if (!len || (len & 15))
		return -EINVAL;

	aes_dev = bus_find_device_by_name(&platform_bus_type, NULL,
					  "38c00000.aes");
	if (!aes_dev)
		aes_dev = dev;

	clk = clk_get(aes_dev, "aes");
	if (IS_ERR(clk))
		clk = NULL;
	if (clk) {
		ret = clk_prepare_enable(clk);
		if (ret) {
			clk_put(clk);
			if (aes_dev != dev)
				put_device(aes_dev);
			return ret;
		}
	}

	aes = ioremap(S5L8740_AES_PHYS, 0x100);
	if (!aes) {
		ret = -ENOMEM;
		goto out_clk;
	}

	phys = dma_map_single(aes_dev, buf, len, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(aes_dev, phys)) {
		ret = -ENOMEM;
		goto out_io;
	}

	writel(1, aes + 0x08);
	{
		unsigned int guard = 100000;

		while (readl(aes + 0x08) && --guard)
			;
	}
	writel(1, aes + 0x70);
	writel(1, aes + 0x6c);
	writel(~1u, aes + 0x88);
	writel(1, aes + 0x00);
	writel((encrypt ? 1u : 0u) | 0xeu, aes + 0x14);
	writel(len, aes + 0x18);
	writel(phys, aes + 0x28);
	writel(len, aes + 0x2c);
	writel(phys, aes + 0x20);
	writel(len, aes + 0x24);
	writel(phys, aes + 0x30);
	writel(len, aes + 0x34);
	writel(0, aes + 0x74);
	writel(0, aes + 0x78);
	writel(0, aes + 0x7c);
	writel(0, aes + 0x80);
	writel(7, aes + 0x0c);
	writel(1, aes + 0x04);
	ret = readl_poll_timeout(aes + 0x0c, irq, irq & 1, 2, 500000);
	writel(0, aes + 0x00);
	dma_unmap_single(aes_dev, phys, len, DMA_BIDIRECTIONAL);
	if (ret)
		dev_err(dev, "422FFA MMIO timeout IRQ=0x%x\n", irq);

out_io:
	iounmap(aes);
out_clk:
	if (clk) {
		clk_disable_unprepare(clk);
		clk_put(clk);
	}
	if (aes_dev != dev)
		put_device(aes_dev);
	return ret;
}

static int grape_gid_crypt(struct device *dev, u8 *buf, unsigned int len,
			    bool encrypt)
{
	struct crypto_skcipher *tfm;
	struct skcipher_request *req;
	struct scatterlist sg;
	u8 key[AES_KEYSIZE_128] = { 0 };
	u8 iv[AES_BLOCK_SIZE] = { 0 };
	int ret;

	if (!len || (len & 15))
		return -EINVAL;

	tfm = crypto_alloc_skcipher("cbc(aes-gid)", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);
	ret = crypto_skcipher_setkey(tfm, key, sizeof(key));
	if (ret)
		goto out_tfm;
	req = skcipher_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		ret = -ENOMEM;
		goto out_tfm;
	}
	sg_init_one(&sg, buf, len);
	skcipher_request_set_callback(req, 0, NULL, NULL);
	skcipher_request_set_crypt(req, &sg, &sg, len, iv);
	ret = encrypt ? crypto_skcipher_encrypt(req) : crypto_skcipher_decrypt(req);
	skcipher_request_free(req);
out_tfm:
	crypto_free_skcipher(tfm);
	if (ret)
		dev_warn(dev, "cbc(aes-gid) %s %d\n",
			 encrypt ? "enc" : "dec", ret);
	return ret;
}

static bool grape_looks_like_arm(const u8 *p, size_t n)
{
	return n >= 4 && p[0] == 0x18 && p[1] == 0xf0 &&
	       p[2] == 0x9f && p[3] == 0xe5;
}

/*
 * v6 RE (2026-08-25): post-GID plaintext may already be a preconstructed
 * HBPP DATA object (18 E1 30 01 …) — Corellium GEN_1 sends Constructed
 * Firmware unchanged. Detect that and SPI-send as-is (ACK 0x4BC1).
 */
static bool grape_looks_like_hbpp_data(const u8 *p, size_t n)
{
	if (n >= 4 && p[0] == 0x18 && p[1] == 0xe1 &&
	    p[2] == 0x30 && p[3] == 0x01)
		return true;
	if (n >= 2 && p[0] == 0x30 && p[1] == 0x01)
		return true;
	return false;
}

/**
 * grape_send_preconstructed_hbpp - SPI the whole HBPP frame, expect 0x4BC1
 * (DATA ACK). Do not re-wrap or swizzle — bytes are already HBPP.
 */
static int grape_send_preconstructed_hbpp(struct grape *n, const u8 *data,
					   size_t len)
{
	int try, ret;

	if (len < 16)
		return -EINVAL;

	for (try = 0; try < 5; try++) {
		if (chunk_spi)
			ret = grape_xfer(n, data, NULL, len);
		else if (n->blob16)
			ret = grape_burst_u16_ex(n, data, NULL, len, 0);
		else
			ret = grape_burst_ex(n, data, NULL, len, 0);
		if (ret)
			continue;
		if (grape_wait_ack(n, GRAPE_ACK_CHUNK, 8) == 0) {
			grape_vinfo(n,
				 "preconstructed HBPP %zuB ACK 0x4BC1 try=%d\n",
				 len, try);
			return 0;
		}
		if (n->blob16 && try == 0) {
			n->blob16 = false;
			grape_vinfo(n,
				 "preconstructed HBPP: fall back to 8-bit\n");
		}
	}
	return -EIO;
}

/*
 * 204E0 sends le32(8740+0x0c)=0xe970. The decrypted cut is 0xecf0 and the
 * extra 896 bytes are 0x53/0x43 fill. Downloading that fill to dest 0xe970
 * stomps SRAM just past the official image (likely BSS / bootloader workspace).
 */
static size_t grape_official_arm_len(const u8 *body, size_t len)
{
	size_t i;

	if (len <= GRAPE_ARM_OFFICIAL || !grape_looks_like_arm(body, len))
		return len;
	for (i = GRAPE_ARM_OFFICIAL; i < len; i++) {
		if (body[i] != 0x53 && body[i] != 0x43)
			return len;
	}
	return GRAPE_ARM_OFFICIAL;
}

static int grape_download_fw(struct grape *n, const u8 *data, size_t size,
			      bool arm_at_zero)
{
	u8 *dec = NULL;
	int ret;
	const u8 *body = data;
	size_t body_len = size;
	/*
	 * Only a gpfw-sourced blob gets the container treatment.
	 *
	 * sub_1A640 has two mutually exclusive sources, chosen on
	 * MEMORY[0x8A8FAA4]. The FILESYSTEM path -- iPod_Control/Device/
	 * GrapeFirmware.bin -- reads the file, sets MEMORY[0x8925F88] to the
	 * buffer itself and MEMORY[0x8925F8C] to the whole file size, and
	 * never looks at the magic at all. There is no +0x400, no length
	 * field and no decrypt on that path.
	 *
	 * Only the gpfw path checks:
	 *
	 *	if (sub_440CD0(buf, "8740", 4))   // memcmp, non-zero = differs
	 *		send the whole blob;
	 *	else
	 *		sub_204E0();                  // +0x400, le32(+0x0c), GID
	 *
	 * request_firmware("apple/grape.bin") is the filesystem analogue, so
	 * applying the container treatment to it skipped 0x400 bytes and
	 * decrypted a payload stock would have sent verbatim.
	 */
	bool apple_hdr = n->fw_from_ftl &&
			 grape_fw_has_8740_hdr(data, size);

	(void)arm_at_zero;
	grape_fwfile_classify(n, data, size);

	/*
	 * 1A640 NOR 8740 → 204E0. ARM at +0x400, size le32(+0x0c).
	 * Rev 3: 422FFA GID-CBC IV=0 decrypt in place (NOR gpfw).
	 * Rev 4: no decrypt; 204E0 then returns 33 so NOR drops it.
	 */
	if (apple_hdr) {
		u32 hdr_sz;
		u8 rev;

		if (size < 0x400 + 4) {
			dev_warn(&n->spi->dev,
				 "FW too small (%zu) — skip download\n", size);
			return -EINVAL;
		}
		hdr_sz = get_unaligned_le32(data + 0x0c);
		/* Short NOR slice: take the ARM bytes we have. Never expand. */
		if (!hdr_sz || hdr_sz > size - 0x400)
			hdr_sz = size - 0x400;
		/* 204E0: size rounded up to 16 for 422FFA; keep ≤ available. */
		hdr_sz = round_up(hdr_sz, 16);
		if (hdr_sz > size - 0x400)
			hdr_sz = (size - 0x400) & ~15u;
		body = data + 0x400;
		body_len = hdr_sz;
		rev = data[7];
		/*
		 * Disk path (1A640) sends the raw file, no 204E0.
		 * NOR 204E0 rev≠3 returns 33 after writing outputs and
		 * the NOR path frees the buffer. Rev 4 plaintext is the
		 * disk-shaped image — send the whole 8740+ARM at dest 0.
		 */
		if (rev != 3) {
			grape_vinfo(n,
				 "204E0 ARM-at-0 %zuB dest 0 (rev=%u hdr+0x0c=0x%x file=%zu)\n",
				 body_len, rev, get_unaligned_le32(data + 0x0c),
				 size);
			if (force_gid && size >= 0x400 + 16) {
				u8 *try = kmemdup(data + 0x400, min_t(size_t, body_len, size - 0x400),
						  GFP_KERNEL);
				if (try && grape_422ffa_mmio(&n->spi->dev, try,
							      round_up(min_t(size_t, body_len, size - 0x400) & ~15u, 16),
							      false) == 0 &&
				    grape_looks_like_arm(try, min_t(size_t, 16, body_len))) {
					grape_vinfo(n, "force_gid 422FFA ARM ok\n");
					body = try;
					body_len = min_t(size_t, body_len, size - 0x400);
					dec = try;
				} else {
					kfree(try);
				}
			}
			goto send;
		}
		grape_vinfo(n,
			 "204E0 ARM-at-0 %zu bytes rev=%u (hdr+0x0c=0x%x file=%zu)\n",
			 body_len, rev, get_unaligned_le32(data + 0x0c), size);

		if (rev == 3) {
			u8 *probe;
			bool verified = false;

			probe = kmemdup(data, 16, GFP_KERNEL);
			if (!probe)
				return -ENOMEM;
			if (grape_gid_crypt(&n->spi->dev, probe, 16, true) == 0 &&
			    !memcmp(probe, data + 0x40, 16)) {
				verified = true;
				grape_vinfo(n, "26CCC GID verify OK\n");
			} else {
				memcpy(probe, data, 16);
				if (grape_422ffa_mmio(&n->spi->dev, probe, 16,
						       true) == 0 &&
				    !memcmp(probe, data + 0x40, 16)) {
					verified = true;
					grape_vinfo(n,
						 "26CCC 422FFA verify OK\n");
				} else {
					dev_warn(&n->spi->dev,
						 "26CCC GID verify fail (sig %02x%02x%02x%02x got %02x%02x%02x%02x)\n",
						 data[0x40], data[0x41],
						 data[0x42], data[0x43],
						 probe[0], probe[1],
						 probe[2], probe[3]);
				}
			}
			kfree(probe);

			dec = kmemdup(body, body_len, GFP_KERNEL);
			if (!dec)
				return -ENOMEM;
			/* 204E0: one-shot 422FFA, not the Linux AES walk. */
			ret = grape_422ffa_mmio(&n->spi->dev, dec, body_len,
						 false);
			if (ret || !grape_looks_like_arm(dec, body_len)) {
				memcpy(dec, body, body_len);
				ret = grape_gid_crypt(&n->spi->dev, dec,
						       body_len, false);
			}
			grape_vinfo(n,
				 "204E0 GID decrypt ret=%d arm=%d head %02x %02x %02x %02x ver=%d\n",
				 ret, grape_looks_like_arm(dec, body_len),
				 dec[0], dec[1], dec[2], dec[3], verified);
			if (!ret && body_len > 0xd210)
				grape_vinfo(n,
					 "ARM +0x54 %02x%02x%02x%02x +0x100 %02x%02x%02x%02x +0x1000 %02x%02x%02x%02x +0xD208 %02x%02x%02x%02x +0x20=%08x\n",
					 dec[0x54], dec[0x55], dec[0x56], dec[0x57],
					 dec[0x100], dec[0x101], dec[0x102], dec[0x103],
					 dec[0x1000], dec[0x1001], dec[0x1002],
					 dec[0x1003],
					 dec[0xd208], dec[0xd209], dec[0xd20a],
					 dec[0xd20b],
					 get_unaligned_le32(dec + 0x20));
			if (!ret) {
				body = dec;
			} else {
				kfree(dec);
				dec = NULL;
			}
		}
	} else if (size < 4) {
		dev_warn(&n->spi->dev, "FW empty (%zu) — skip download\n", size);
		return -EINVAL;
	} else {
		grape_vinfo(n, "Grape FW download %zu bytes (no 8740)\n",
			 size);
	}

send:
	/*
	 * v6: if body is already 18 E1 30 01… (post-GID constructed FW),
	 * send once and run post-download. Do not re-packetize ARM.
	 */
	if (grape_looks_like_hbpp_data(body, body_len)) {
		grape_vinfo(n,
			 "preconstructed HBPP DATA %zuB — direct SPI (no ARM wrap)\n",
			 body_len);
		ret = grape_send_preconstructed_hbpp(n, body, body_len);
		if (!ret) {
			ret = grape_post_download(n);
			if (!ret)
				ret = grape_cmd_2d54c(n);
		}
		kfree(dec);
		return ret;
	}

	{
		size_t official = grape_official_arm_len(body, body_len);
		size_t dl_len = body_len;
		const u8 *dl_body = body;
		u8 *z2_prep = NULL;
		u8 *pad_buf = NULL;
		u8 *win;
		int try, cal;

		if (official < body_len) {
			grape_vinfo(n,
				 "cap ARM %zu -> %zu (204E0 +0x0c; strip S/C fill)\n",
				 body_len, official);
			body_len = official;
			dl_len = body_len;
		}

		z2_prep = grape_maybe_prepend_z2_hdr(n, body, body_len, &dl_len);
		if (z2_prep)
			dl_body = z2_prep;
		else if (dl_len & 3) {
			pad_buf = kmalloc(round_up(dl_len, 4), GFP_KERNEL);
			if (!pad_buf) {
				kfree(dec);
				return -ENOMEM;
			}
			memcpy(pad_buf, dl_body, dl_len);
			memset(pad_buf + dl_len, 0, round_up(dl_len, 4) - dl_len);
			dl_len = round_up(dl_len, 4);
			dl_body = pad_buf;
			grape_vinfo(n, "FW padded to %zu (4-byte align)\n",
				 dl_len);
		}
		grape_fw_audit(n, dl_body, dl_len, "2D640");

		win = kzalloc(GRAPE_FW_HDR_LEN, GFP_KERNEL);
		if (!win) {
			kfree(z2_prep);
			kfree(pad_buf);
			kfree(dec);
			return -ENOMEM;
		}
		/*
		 * Host cal source ≠ controller address.
		 * 2D640 ARM → dest = fw_dest + offset (OSOS fw_dest=0).
		 * 2D7A4 cal → dest = 0x00400200 + offset.
		 * EXEC 0x00100018 is mapped app PC — not upload dest.
		 */
		cal = grape_load_cal_window(n, win);
		if (cal < 0) {
			dev_err(&n->spi->dev, "2D7A4 aborted: no device cal\n");
			kfree(win);
			kfree(z2_prep);
			kfree(pad_buf);
			kfree(dec);
			return cal;
		}

		grape_vinfo(n,
			 "2D640 ARM dest_base=0x%08x EXEC=0x%08x cal=0x%08x\n",
			 fw_dest, exec_addr, GRAPE_CAL_DEST);
		if (fw_dest == 0)
			grape_vinfo(n,
				 "expect FW prefix: 18 e1 30 01 07 fc 00 00 00 00 01 03 (len=0x1ff0)\n");
		else
			dev_warn(&n->spi->dev,
				 "fw_dest override 0x%08x — OSOS uses 0 (A/B only)\n",
				 fw_dest);
		grape_vinfo(n,
			 "expect CAL prefix: 18 e1 30 01 00 80 02 00 00 40 00 c2\n");

		/* 20E94: 273A0 up to 3 times, no 1A878 between. */
		for (try = 0; try < 3; try++) {
			ret = grape_send_blob(n, dl_body, dl_len, fw_dest);
			if (ret) {
				dev_err(&n->spi->dev,
					"2D640 ARM@0x%08x try %d: %d\n",
					fw_dest, try, ret);
				continue;
			}
			n->fw_uploaded = true;
			grape_fw_readback(n, "post-2D640");
			ret = grape_send_blob(n, win, GRAPE_FW_HDR_LEN,
					       GRAPE_CAL_DEST);
			if (ret) {
				dev_err(&n->spi->dev,
					"2D7A4 cal@0x%08x try %d: %d\n",
					GRAPE_CAL_DEST, try, ret);
				continue;
			}
			n->cal_uploaded = true;
			grape_vinfo(n,
				 "2D7A4 512B cal @0x%08x ACK (transport only)\n",
				 GRAPE_CAL_DEST);
			grape_cal_readback(n, win);
			ret = grape_post_download(n);
			if (ret) {
				dev_warn(&n->spi->dev,
					 "2D5B0 try %d: %d\n", try, ret);
				continue;
			}
			ret = grape_cmd_2d54c(n);
			if (!ret) {
				grape_vinfo(n,
					 "2D54C EXEC sent (try %d) — await runtime ping\n",
					 try);
				break;
			}
			dev_warn(&n->spi->dev, "2D54C try %d: %d\n", try, ret);
		}
		kfree(win);
		kfree(z2_prep);
		kfree(pad_buf);
	}
	kfree(dec);
	/* EXEC transport success is NOT runtime_ready / fw_loaded. */
	return ret;
}

static int grape_ping(struct grape *n, u16 *status_out)
{
	u8 tx[GRAPE_FRAME_LEN] = { 0 };
	u8 rx[GRAPE_FRAME_LEN] = { 0 };
	u16 csum, rx_csum;
	int ret, tries;

	put_unaligned_le32(GRAPE_PING_TYPE, tx);
	csum = grape_sum16(tx, 14);
	put_unaligned_le16(csum, tx + 14);

	/* 182590: up to 5 retries, sleep 1 between. After EXEC, 16-bit
	 * pairs match the app SPI width; 8-bit PIO is bootloader-only.
	 */
	for (tries = 0; tries < 6; tries++) {
		if (n->exec_sent && go_xfer)
			ret = grape_burst_u16(n, tx, rx, GRAPE_FRAME_LEN);
		else
			ret = grape_burst16(n, tx, rx);
		if (ret)
			return ret;

		rx_csum = get_unaligned_le16(rx + 14);
		if (!rx_csum && !grape_sum16(rx, 14)) {
			dev_warn(&n->spi->dev, "ping rx all-zero (MISO dead)\n");
			return -EIO;
		}
		if (grape_sum16(rx, 14) == rx_csum)
			break;
		/* One dump per call; MultitouchTask rate-limits via ping_fails. */
		if (tries == 0 && n->ping_fails == 0)
			dev_warn(&n->spi->dev,
				 "ping csum fail rx %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				 rx[0], rx[1], rx[2], rx[3], rx[4],
				 rx[5], rx[6], rx[7], rx[8], rx[9],
				 rx[10], rx[11], rx[12], rx[13],
				 rx[14], rx[15]);
		{
			u16 w0 = get_unaligned_be16(rx);

			if (grape_status_is_bootloader(w0))
				dev_warn(&n->spi->dev,
					 "bootloader status 0x%04x: answering 0x4F81, which is not a word sub_2C87E accepts\n",
					 w0);
		}
		if (tries == 5)
			return -EIO;
		msleep(1);
	}

	if (status_out)
		*status_out = get_unaligned_le16(rx + 1);
	return 0;
}

static void grape_map_coords(s16 rawx, s16 rawy, int *x, int *y)
{
	int xx = (GRAPE_ABS_X_MAX * ((int)rawx + 75)) / GRAPE_SCALE_X_DIV;
	int yy = (GRAPE_ABS_Y_MAX * ((int)rawy + 75)) / GRAPE_SCALE_Y_DIV;

	if (xx < 0)
		xx = 0;
	if (xx > GRAPE_ABS_X_MAX)
		xx = GRAPE_ABS_X_MAX;
	yy = GRAPE_ABS_Y_MAX - yy;
	if (yy < 0)
		yy = 0;
	if (yy > GRAPE_ABS_Y_MAX)
		yy = GRAPE_ABS_Y_MAX;
	*x = xx;
	*y = yy;
}

/*
 * sub_187AB4 -- one report frame.
 *
 * Slot identity comes from the CONTACT ID in record byte 0, not from the
 * record's position in the frame. Stock keeps eight persistent ID-keyed
 * slots, reuses a slot when the same ID appears again, allocates a free one
 * for a new ID, and explicitly releases any slot whose ID is absent from the
 * frame.
 *
 * This used to bind slot i to record i and treat any non-zero byte 1 as
 * down. That makes contacts jump whenever the controller reorders records,
 * leaves stale contacts when a record disappears, and cannot express a
 * release at all. Linux Type-B has the same requirement as stock here:
 * identifiable contacts must keep slot identity, and a tracking ID that
 * disappears must be released.
 *
 * input_mt_get_slot_by_key() does the keying, and input_mt_sync_frame()
 * performs the release of every slot not touched this frame -- provided
 * every present contact is reported first, which is why the loop no longer
 * skips records.
 *
 * Byte 1 and byte 2 are both part of the state. Byte 1 == 0 is not-down.
 * The finer bytes1/2 lifecycle stock runs is NOT transliterated here and is
 * NOT established from this driver's evidence; what is implemented is the
 * down/absent distinction plus ID keying. Do not invent pressure or width
 * from bytes whose offsets are unproven.
 */
static void grape_parse_D(struct grape *n, const u8 *payload, unsigned int len)
{
	const u8 *rec;
	u8 count, stride;
	unsigned int off;
	int i;

	if (!n->input)
		return;
	if (len < 18 || payload[0] != 0x44)
		return;

	off = payload[2];
	count = payload[16];
	stride = payload[17];
	if (!stride || off >= len)
		return;
	if (count > GRAPE_SLOTS)
		count = GRAPE_SLOTS;

	rec = payload + off;
	for (i = 0; i < count; i++) {
		s16 rawx, rawy;
		int x, y, slot;
		u8 id, st1, st2;

		if (rec + stride > payload + len)
			break;

		id = rec[0];
		st1 = rec[1];
		st2 = rec[2];
		rawx = (s16)get_unaligned_le16(rec + 4);
		rawy = (s16)get_unaligned_le16(rec + 6);
		grape_map_coords(rawx, rawy, &x, &y);

		slot = input_mt_get_slot_by_key(n->input, id);
		if (slot < 0) {
			rec += stride;
			continue;
		}

		input_mt_slot(n->input, slot);
		input_mt_report_slot_state(n->input, MT_TOOL_FINGER, st1 != 0);
		if (st1) {
			input_report_abs(n->input, ABS_MT_POSITION_X, x);
			input_report_abs(n->input, ABS_MT_POSITION_Y, y);
			pr_warn_ratelimited(
				"grape touch slot%d id=%u s1=%u s2=%u raw=%d,%d -> %d,%d\n",
				slot, id, st1, st2, rawx, rawy, x, y);
		}
		rec += stride;
	}
	/* Releases every slot no contact claimed this frame. */
	input_mt_sync_frame(n->input);
	input_sync(n->input);
}

/*
 * sub_17E404 -- read one report, exactly as stock frames it.
 *
 *     zero a 512-byte TX buffer
 *     tx[0] = 0xEA; tx[1] = 0x01; tx[2] = 0x01
 *     *(u16 *)(tx + len - 2) = sum16(tx, 14)
 *     if (len > 0x200) len = 512
 *     xfer(tx, len, rx, len)
 *     if (xfer failed || rx[0] != 0xEA)            return 81
 *     if ((u8)sum16(rx, 5))                        return 87
 *     plen = rx[2]
 *     if (plen && plen != 2) {
 *         if (*(u16 *)(rx + len - 2) == sum16(rx + 5, plen - 2))
 *             deliver(rx + 5, plen - 2);
 *         else                                     return 87
 *     }
 *
 * The two checksum tests were both missing here. The header one is an
 * 8-bit test -- stock takes the low byte of the 16-bit sum over rx[0..4]
 * and requires zero -- and the payload one compares the trailer at
 * rx[len - 2] against the sum over the report body. Without them any
 * garbage that happened to start with 0xEA was handed to the parser.
 *
 * The old code also had a fallback that parsed from rx[5] whenever it saw
 * 0x44 there, and floored the length at 16. Stock does neither: the
 * length is whatever the pending count asked for, and a frame that fails
 * either checksum is an error, not something to salvage.
 */
static int grape_read_reports(struct grape *n, u16 pending)
{
	unsigned int len = (unsigned int)pending + 5;
	u8 *tx, *rx;
	unsigned int plen;
	int ret;

	if (len > GRAPE_READ_MAX)
		len = GRAPE_READ_MAX;
	if (len < 8)
		return -EINVAL;

	tx = kzalloc(GRAPE_READ_MAX, GFP_KERNEL);
	rx = kzalloc(GRAPE_READ_MAX, GFP_KERNEL);
	if (!tx || !rx) {
		ret = -ENOMEM;
		goto out;
	}

	tx[0] = GRAPE_MAGIC;
	tx[1] = 0x01;
	tx[2] = 0x01;
	put_unaligned_le16(grape_sum16(tx, 14), tx + len - 2);

	ret = grape_xfer(n, tx, rx, len);
	if (ret)
		goto out;

	if (rx[0] != GRAPE_MAGIC) {
		dev_info_once(&n->spi->dev,
			      "17E404 magic: rx %02x %02x %02x %02x %02x %02x %02x %02x\n",
			      rx[0], rx[1], rx[2], rx[3], rx[4], rx[5],
			      rx[6], rx[7]);
		ret = -EIO;
		goto out;
	}

	if ((u8)grape_sum16(rx, 5)) {
		dev_info_once(&n->spi->dev,
			      "17E404 header sum: rx %02x %02x %02x %02x %02x\n",
			      rx[0], rx[1], rx[2], rx[3], rx[4]);
		ret = -EIO;
		goto out;
	}

	plen = rx[2];
	if (plen && plen != 2) {
		u16 want, got;

		if (5 + (plen - 2) > len) {
			ret = -EIO;
			goto out;
		}
		got = get_unaligned_le16(rx + len - 2);
		want = grape_sum16(rx + 5, plen - 2);
		if (got != want) {
			dev_info_once(&n->spi->dev,
				      "17E404 payload sum %04x want %04x plen %u len %u\n",
				      got, want, plen, len);
			ret = -EIO;
			goto out;
		}
		grape_parse_D(n, rx + 5, plen - 2);
	}
	ret = 0;
out:
	kfree(tx);
	kfree(rx);
	return ret;
}

/*
 * sub_188FFC -- ask how much is pending, then read exactly that.
 *
 *     if (!powered)                       return 18
 *     result = sub_182590(&pending, 5)
 *     if (result || (pending && (result = sub_17E404(pending + 5))))
 *         if (retries) { sleep 1; recurse with retries - 1 }
 *     return result
 *
 * A pending count of zero is not a failure and not a reason to read: the
 * part simply has nothing to hand over. Reading anyway, which this driver
 * did from the ATTN path with a hardcoded zero, puts a frame on the bus
 * that stock would never send.
 */
static int grape_service_once(struct grape *n, unsigned int retries)
{
	u16 pending = 0;
	int ret;

	ret = grape_ping(n, &pending);
	if (!ret && pending)
		ret = grape_read_reports(n, pending);
	if (ret && retries) {
		msleep(1);
		return grape_service_once(n, retries - 1);
	}
	return ret;
}

static void grape_dump_pad(struct grape *n, unsigned int gpio, const char *name)
{
	void __iomem *b;
	u32 pin, pcon, din, dir;

	if (!grape_verbose || !n->gpio_base)
		return;
	b = n->gpio_base + 32 * (gpio >> 3);
	pin = gpio & 7;
	pcon = readl(b);
	din = readl(b + 0x04);
	dir = readl(b + 0x14);
	grape_vinfo(n,
		 "pad %s gpio%u pcon=%x din=%u dir=%u dout=%u punb=%u punc=%u\n",
		 name, gpio, (pcon >> (4 * pin)) & 0xf,
		 !!(din & BIT(pin)), !!(dir & BIT(pin)),
		 !!(readl(b + 0x08) & BIT(pin)),
		 !!(readl(b + 0x0c) & BIT(pin)),
		 !!(readl(b + 0x10) & BIT(pin)));
}

/*
 * Optional glass experiment only — not in OSOS sub_1A5AC.
 * Default off (extra_por_pulse=0).
 */
static void grape_gpio_por_reset(struct grape *n)
{
	grape_gpiocmd_mode(n, GRAPE_GPIO_RST, 1, 0);
	msleep(reset_hold_ms);
	grape_gpiocmd_mode(n, GRAPE_GPIO_RST, 1, 1);
	msleep(reset_release_ms);
	grape_gpiocmd_mode(n, GRAPE_GPIO_RST, 1, 0);
	msleep(5);
	grape_vinfo(n, "extra POR RST %dms low / %dms high\n",
		 reset_hold_ms, reset_release_ms);
}

/*
 * 1A5AC GPIO half (before 20848):
 *   2075A(1) sleep5 → 20766(1) sleep15 → 20690(1) sleep5 → 11B70
 */
static void grape_gpio_bringup(struct grape *n)
{
	int rail;

	grape_clkcon_replay(n);

	/* GPIOCMD only — gpiod set_value fights polarity on RST. */
	grape_gpiocmd_mode(n, GRAPE_GPIO_RST, 1, 0);
	msleep(5);
	/*
	 * sub_20766(1): 439B00(1) rail first, 66A8(8)+sleep 3, then
	 * GPIOCMD EN mode 0 (not output-high).
	 */
	rail = d1830_grape_rail(true);
	if (rail)
		dev_warn(&n->spi->dev, "20766 PMIC rail: %d\n", rail);
	else {
		/* 66A8(8) is 345D40 thunk (0x220002B2), not an 8ms sleep */
		msleep(3);
	}
	/* 20766(1): EN mode 0, val 0. Do not cmd-15 the latch. */
	grape_gpiocmd_mode(n, GRAPE_GPIO_EN, 0, 0);
	grape_dump_pad(n, GRAPE_GPIO_EN, "en-mode0");
	msleep(15);
	/* 20690(1) after EN — OSOS order; required after 1A878 unmux */
	grape_spi2_pinmux(n, true);
	msleep(5);
	/* 1A5AC: 11B70 after remux, still in reset, before 20848 */
	grape_spi2_11b70(n);
	grape_dump_pad(n, GRAPE_GPIO_EN, "en");
	grape_dump_pad(n, GRAPE_GPIO_RST, "rst");
	grape_dump_pad(n, GRAPE_GPIO_IRQ, "irq");
	grape_dump_pad(n, 87, "spi2-87");
	grape_dump_pad(n, 88, "spi2-88");
	grape_dump_pad(n, 89, "spi2-89");
	grape_dump_pad(n, 90, "spi2-90");
}

/* 1A5AC: 2075A(0) then sleep reset_release_ms (default 30). */
static void grape_gpio_release_reset(struct grape *n)
{
	grape_gpiocmd_mode(n, GRAPE_GPIO_RST, 1, 1);
	msleep(reset_release_ms);
	grape_dump_pad(n, GRAPE_GPIO_RST, "rst-rel");
}

/* sub_20490(1) — GPIOCMD input + EIC enable for GPIO 38 */
static void grape_irq_enable(struct grape *n)
{
	grape_gpiocmd_mode(n, GRAPE_GPIO_IRQ, 0, 0);
	/* RetailOS: level, active-low → VIC EXT1 */
	if (s5l8740_eic_enable_gpio(GRAPE_GPIO_IRQ, IRQ_TYPE_LEVEL_LOW) == 0)
		grape_vinfo(n, "EIC enabled GPIO%d level-low\n",
			 GRAPE_GPIO_IRQ);
}

static int grape_1a5ac_and_download(struct grape *n, const u8 *data,
				     size_t size, const char *tag)
{
	unsigned int attempt;
	int err;

	/* Dump path has no pre-1A5AC POR; gate for glass A/B only. */
	if (extra_por_pulse)
		grape_gpio_por_reset(n);
	grape_gpio_bringup(n);
	err = grape_bootload_cmd(n);
	if (err)
		dev_warn(&n->spi->dev, "bootload %s: %d\n", tag, err);
	msleep(15); /* 1A5AC: after 20848, before 2075A(0) */
	grape_gpio_release_reset(n);
	if (skip_download) {
		grape_vinfo(n, "skip_download — no 2D640\n");
		return 0;
	}
	/* 20E94: 26494 then 273A0; settle already done in release (30ms). */
	if (grape_probe_26494(n, tag)) {
		dev_warn(&n->spi->dev, "26494 %s failed\n", tag);
		return -EIO;
	}
	/*
	 * Retry the transfer before escalating. The caller's retry re-runs the
	 * whole rail/reset/HBPP bring-up, which is a lot of disruption for what
	 * is usually a transient bus error; the stock code retries just this.
	 */
	for (attempt = 0; attempt < download_tries; attempt++) {
		err = grape_download_fw(n, data, size, false);
		if (!err)
			break;
		dev_warn(&n->spi->dev, "download %s attempt %u: %d\n",
			 tag, attempt + 1, err);
	}
	n->fw_tried = true;
	return err;
}

/*
 * 1703E8: 10 failed pings → 13A20(0) then 13A20(1). Cap recycles so a
 * stuck bootloader does not spam forever (GO ACKs but app never runs).
 */
#define GRAPE_RECYCLE_MAX	3

static void grape_park(struct grape *n, const char *why)
{
	if (n->parked)
		return;
	if (no_park) {
		dev_warn(&n->spi->dev,
			 "would park (%s); no_park=1, still servicing\n", why);
		return;
	}
	n->parked = true;
	n->stopped = true;
	grape_verbose = false;
	dev_warn(&n->spi->dev,
		 "grape parked (%s) — rmmod/insmod to retry\n", why);
	/*
	 * Parking cuts the rail, which makes every post-mortem probe read
	 * an undriven bus and look exactly like the failure being
	 * investigated. touch_power_down only covers suspend, so there was
	 * no way to examine a part that had reached EXEC and stayed up.
	 */
	if (park_power_down)
		grape_power_down(n);
	else
		dev_warn(&n->spi->dev,
			 "park_power_down=0: rail left on for probing\n");
}

/* ------------------------------------------------------------------ */
/* Screen-sleep suspend / resume                                        */
/*                                                                      */
/* Two levels, because they trade power against wake latency:           */
/*                                                                      */
/*   touch_power_down=0  IRQ masked and SPI2 released, controller still */
/*                       powered. Resume is immediate.                  */
/*   touch_power_down=1  full 1A878 power cut including the PMU rail.   */
/*                       Resume has to re-run bring-up and download the */
/*                       firmware again, so it costs a few hundred ms.  */
/* ------------------------------------------------------------------ */

static struct grape *grape_pm_dev;

static bool touch_power_down = true;
module_param(touch_power_down, bool, 0644);
MODULE_PARM_DESC(touch_power_down,
		 "Cut the touch rail on screen sleep (default Y; N keeps it powered)");

static bool grape_pm_suspended;

int n31_touch_suspend(void)
{
	struct grape *n = grape_pm_dev;

	if (!n)
		return -ENODEV;
	if (grape_pm_suspended)
		return 0;

	if (n->irq > 0)
		disable_irq(n->irq);

	if (touch_power_down) {
		mutex_lock(&n->lock);
		grape_power_down(n);
		n->runtime_ready = false;
		n->spi_ok = false;
		mutex_unlock(&n->lock);
	} else {
		/* Stop driving the bus, leave the controller alive. */
		mutex_lock(&n->lock);
		grape_spi2_pinmux(n, false);
		mutex_unlock(&n->lock);
	}

	grape_pm_suspended = true;
	grape_vinfo(n, "touch suspended (power_down=%d)\n", touch_power_down);
	return 0;
}
EXPORT_SYMBOL_GPL(n31_touch_suspend);

int n31_touch_resume(void)
{
	struct grape *n = grape_pm_dev;
	const struct firmware *fw = NULL;
	const u8 *data = NULL;
	u8 *kbuf = NULL;
	size_t size = 0;
	int ret = 0;

	if (!n)
		return -ENODEV;
	if (!grape_pm_suspended)
		return 0;

	if (!touch_power_down) {
		mutex_lock(&n->lock);
		grape_spi2_pinmux(n, true);
		mutex_unlock(&n->lock);
		goto done;
	}

	/*
	 * The controller lost its firmware with the rail, so this is the same
	 * bring-up the probe runs. Storage is back by the time a wake reaches
	 * here, so fetching the blob again is safe.
	 */
	ret = grape_acquire_fw(&n->spi->dev, &data, &size, &fw, &kbuf,
				 &n->fw_from_ftl);
	if (ret) {
		dev_warn(&n->spi->dev, "touch resume: no firmware (%d)\n", ret);
		goto done;
	}

	mutex_lock(&n->lock);
	n->fw_uploaded = false;
	n->cal_uploaded = false;
	n->requestcal_done = false;
	n->exec_sent = false;
	n->runtime_ready = false;
	n->fw_loaded = false;
	n->spi_ok = false;
	ret = grape_1a5ac_and_download(n, data, size, "resume");
	mutex_unlock(&n->lock);

	grape_release_fw(fw, kbuf);
	if (ret)
		dev_warn(&n->spi->dev, "touch resume download: %d\n", ret);

done:
	if (n->irq > 0)
		enable_irq(n->irq);
	grape_pm_suspended = false;
	grape_vinfo(n, "touch resumed (%d)\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(n31_touch_resume);


static void grape_recycle(struct grape *n)
{
	const struct firmware *fw = NULL;
	const u8 *data;
	u8 *kbuf = NULL;
	size_t size;
	u16 st = 0;

	if (n->parked)
		return;
	if (n->recycle_count >= GRAPE_RECYCLE_MAX) {
		grape_park(n, "recycle budget");
		return;
	}
	n->recycle_count++;
	grape_vinfo(n,
		 "1703E8 10 ping fails — 13A20 recycle %u/%u\n",
		 n->recycle_count, GRAPE_RECYCLE_MAX);
	grape_power_down(n);
	n->fw_loaded = false;
	n->fw_tried = false;
	n->spi_ok = false;
	n->ping_fails = 0;
	msleep(50);
	if (grape_acquire_fw(&n->spi->dev, &data, &size, &fw, &kbuf,
				 &n->fw_from_ftl)) {
		dev_warn(&n->spi->dev, "recycle: no FW (FTL or file)\n");
		grape_park(n, "no firmware");
		return;
	}
	grape_1a5ac_and_download(n, data, size, "recycle");
	grape_release_fw(fw, kbuf);
	msleep(2);
	if (!grape_ping(n, &st)) {
		n->spi_ok = true;
		grape_vinfo(n, "recycle ping ok, status=0x%04x\n", st);
	} else if (n->recycle_count >= GRAPE_RECYCLE_MAX) {
		grape_park(n, "still bootloader after GO");
	}
}

static int grape_service(struct grape *n)
{
	int ret;

	if (n->parked)
		return -ENODEV;

	/*
	 * sub_188FFC, with its own five retries inside grape_service_once:
	 * ask sub_182590 how much is pending, and read exactly that much
	 * with sub_17E404 only when the count is non-zero.
	 *
	 * What used to be here read a report whenever ATTN was asserted and
	 * the ping had failed, passing a pending count of zero. That was
	 * added because a failed ping was being taken as proof the part had
	 * moved on to the application, which it never was -- the ping is
	 * sub_182590 and a failure means its checksum did not verify. A read
	 * framed from a zero count is a frame stock never sends, so it is
	 * gone; if the count is zero the part has nothing to hand over and
	 * the right thing is to wait for the next ATTN.
	 */
	ret = grape_service_once(n, 5);
	if (!ret) {
		n->spi_ok = true;
		n->ping_fails = 0;
		n->attn_fails = 0;
		return 0;
	}

	n->ping_fails++;
	if (n->ping_fails <= 3 || n->ping_fails == 10)
		dev_info(&n->spi->dev,
			 "188FFC failed (%d), attempt %u, attn=%d\n",
			 ret, n->ping_fails,
			 n->attn ? gpiod_get_value_cansleep(n->attn) : -1);
	return ret;
}

static irqreturn_t grape_irq_thread(int irq, void *data)
{
	struct grape *n = data;
	bool mask = false;

	WRITE_ONCE(n->irq_count, n->irq_count + 1);
	mutex_lock(&n->lock);
	if (!n->stopped) {
		if (grape_service(n))
			n->irq_fails++;
		else
			n->irq_fails = 0;

		/*
		 * Re-arm ATN after every service. The stock ATN thread at
		 * EA 0x000F1B74 calls sub_20490(1) at 0x000F1BAE on each
		 * pass round its loop; this driver armed once in probe and
		 * never again.
		 */
		grape_irq_enable(n);
	}
	/*
	 * ATTN is only released when a report is actually read, so a
	 * service that keeps failing leaves the line asserted and this
	 * handler re-entered without pause. Mask rather than spin.
	 */
	if (irq_fail_max && !n->irq_masked &&
	    n->irq_fails >= irq_fail_max) {
		n->irq_masked = true;
		mask = true;
	}
	mutex_unlock(&n->lock);

	if (mask) {
		disable_irq_nosync(irq);
		dev_warn(&n->spi->dev,
			 "ATTN masked after %u failed services; rearm with attn_rearm\n",
			 n->irq_fails);
	}
	return IRQ_HANDLED;
}

static void grape_try_firmware(struct grape *n)
{
	const struct firmware *fw = NULL;
	const u8 *data;
	u8 *kbuf = NULL;
	size_t size;

	if (n->fw_tried || n->fw_loaded)
		return;
	/* Don't hammer SPI with 60KB download if ping already fails */
	if (!n->spi_ok) {
		u16 st;

		if (grape_ping(n, &st))
			return;
		n->spi_ok = true;
	}

	if (grape_acquire_fw(&n->spi->dev, &data, &size, &fw, &kbuf,
				 &n->fw_from_ftl))
		return;

	n->fw_tried = true;
	grape_download_fw(n, data, size, false);
	grape_release_fw(fw, kbuf);
}

static int grape_poll_thread(void *data)
{
	struct grape *n = data;
	int wait;
	int fail_backoff_ms = 50;

	for (wait = 0; wait < 30 && !n->fw_loaded && !n->fw_tried &&
	     !kthread_should_stop(); wait++) {
		mutex_lock(&n->lock);
		grape_try_firmware(n);
		mutex_unlock(&n->lock);
		if (!n->fw_loaded && !n->fw_tried)
			msleep(1000);
	}
	if (!n->fw_loaded && !n->fw_tried) {
		n->fw_tried = true;
		grape_vinfo(n,
			 "no apple/grape.bin — bootload+ping only\n");
	}

	while (!kthread_should_stop()) {
		bool do_poll = true;
		int attn = -1;

		/*
		 * 1703E8/188FFC runs every MultitouchTask loop, not only
		 * on attn. Gating on attn skipped ping if the app released
		 * GPIO 38 after 2D54C (bootloader holds it low).
		 */
		if (n->attn)
			attn = gpiod_get_value_cansleep(n->attn);
		do_poll = true;

		mutex_lock(&n->lock);
		if (n->parked || n->stopped) {
			mutex_unlock(&n->lock);
			/* Parked: sleep until kthread_stop; no SPI traffic. */
			msleep(500);
			continue;
		}
		/*
		 * Watchdog only: with the IRQ live, service from here just
		 * once every GRAPE_WATCHDOG_MS, and only if the handler has
		 * not run since the last pass. n->irq_count is bumped by
		 * grape_irq_thread().
		 */
		if (do_poll && n->use_irq) {
			unsigned int c = READ_ONCE(n->irq_count);

			if (c != n->irq_seen) {
				n->irq_seen = c;
				do_poll = false;
			}
		}
		if (do_poll) {
			grape_service(n);
			if (n->ping_fails >= 10)
				grape_recycle(n);
			fail_backoff_ms = n->spi_ok ? 50 :
				min(fail_backoff_ms * 2, 2000);
		}
		mutex_unlock(&n->lock);

		/*
		 * When the IRQ is live it is the primary trigger and this
		 * thread is only a watchdog.
		 *
		 * Stock services on ATN: the thread at EA 0x000F1B74 blocks on
		 * sub_43DAD4(0x43) and runs sub_1703E8 only when the ISR
		 * signals event 67. It has no periodic poll at all.
		 *
		 * We polled every 50 ms regardless of ATN, so both paths
		 * consumed the same report queue -- the mutex kept them from
		 * racing on the bus but not from one path taking the frame the
		 * other was there to service, which hides a dead IRQ.
		 */
		if (!do_poll)
			msleep(20);
		else if (!n->spi_ok)
			msleep(fail_backoff_ms);
		else if (n->use_irq)
			msleep(GRAPE_WATCHDOG_MS);
		else
			msleep(50);
	}
	return 0;
}

static ssize_t touch_cal_blob_read(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *attr, char *buf,
			      loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct grape *n = dev_get_drvdata(dev);

	if (!n || !n->have_touch_cal)
		return -ENODATA;
	if (off >= GRAPE_TOUCH_CAL_LEN)
		return 0;
	if (off + count > GRAPE_TOUCH_CAL_LEN)
		count = GRAPE_TOUCH_CAL_LEN - off;
	memcpy(buf, n->touch_cal + off, count);
	return count;
}

static struct bin_attribute touch_cal_blob_attr = {
	.attr = {
		.name = "touch_cal_blob",
		.mode = 0444,
	},
	.size = GRAPE_TOUCH_CAL_LEN,
	.read = touch_cal_blob_read,
};

static void grape_touch_cal_sysfs_remove(struct grape *n)
{
	if (!n || !n->touch_cal_sysfs)
		return;
	sysfs_remove_bin_file(&n->spi->dev.kobj, &touch_cal_blob_attr);
	n->touch_cal_sysfs = false;
}

/*
 * Bring-up state. This driver carries two dozen module parameters and had
 * no way to read back what any of them achieved, so a failed download and
 * a controller that never booted looked identical from userspace.
 *
 * The flags are the download milestones in order, so the first one that
 * reads 0 is where the sequence stopped.
 */
static ssize_t state_show(struct device *dev, struct device_attribute *a,
			  char *buf)
{
	struct grape *n = spi_get_drvdata(to_spi_device(dev));

	if (!n)
		return -ENODEV;
	return sysfs_emit(buf,
			  "spi_ok=%d fw_tried=%d fw_uploaded=%d cal_uploaded=%d\n"
			  "requestcal_done=%d exec_sent=%d runtime_ready=%d\n"
			  "irq=%d suspended=%d\n",
			  n->spi_ok, n->fw_tried, n->fw_uploaded,
			  n->cal_uploaded, n->requestcal_done, n->exec_sent,
			  n->runtime_ready, n->irq, grape_pm_suspended);
}
/*
 * Raw transfer window.
 *
 * The application firmware answers the bootloader's ping with a
 * repeating 16-bit word rather than a checksummed frame. A repeating
 * word is not an idle bus -- the part is clocking something out -- so
 * the remaining unknown is framing, not liveness. Guessing at that one
 * hypothesis per rebuild is far too slow, so expose the transfer itself:
 *
 *   echo 'ea 01 01 00 ...' > raw_xfer   send these bytes
 *   cat raw_xfer                        what came back
 *   echo N > raw_len                    pad/clock out to N bytes
 *   raw_mode=0  8-bit PIO   (bootloader width)
 *   raw_mode=1  16-bit PIO  (NOT stock; family comparison only)
 *   raw_mode=2  SPI core
 *
 * Read-only against a part that is already running: it clocks bytes and
 * reports what returns. It cannot wedge the SoC the way arming an
 * unclaimed level interrupt can, which is the whole point of preferring
 * it to another EIC experiment.
 */
/*
 * 0 = 8-bit PIO, which is the only width stock ever uses.
 *
 * This defaulted to 1, the 16-bit diagnostic. sub_11B70 ORs the literal
 * 0x4000 into SPISETUP once per port and never varies the word-size bits,
 * and both of its call sites pass the same mode 0x1A, so no N31 transfer is
 * ever wider than a byte.
 */
static int raw_mode;
module_param(raw_mode, int, 0644);
MODULE_PARM_DESC(raw_mode,
		 "raw_xfer transport: 0=8-bit PIO 1=16-bit PIO 2=SPI core");

static unsigned int raw_len;
module_param(raw_len, uint, 0644);
MODULE_PARM_DESC(raw_len,
		 "Clock raw_xfer out to this many bytes (0 = as written)");

static int grape_raw_do(struct grape *n, const u8 *tx, u8 *rx,
			 unsigned int len)
{
	switch (raw_mode) {
	case 0:
		return grape_burst(n, tx, rx, len);
	case 2:
		return grape_xfer(n, tx, rx, len);
	default:
		return grape_burst_u16(n, tx, rx, len);
	}
}

static ssize_t raw_xfer_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct grape *n = dev_get_drvdata(dev);
	unsigned int i;
	int len = 0;

	if (!n || !n->raw_rx || !n->raw_n)
		return sysfs_emit(buf, "(no transfer yet)\n");

	mutex_lock(&n->lock);
	for (i = 0; i < n->raw_n && len < PAGE_SIZE - 4; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%02x%c",
				 n->raw_rx[i],
				 ((i & 15) == 15 || i + 1 == n->raw_n) ?
				 '\n' : ' ');
	mutex_unlock(&n->lock);
	return len;
}

static ssize_t raw_xfer_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct grape *n = dev_get_drvdata(dev);
	unsigned int nb = 0, want;
	const char *p = buf;
	u8 *tx, *rx;
	int ret;

	if (!n)
		return -ENODEV;

	tx = kzalloc(GRAPE_READ_MAX, GFP_KERNEL);
	rx = kzalloc(GRAPE_READ_MAX, GFP_KERNEL);
	if (!tx || !rx) {
		ret = -ENOMEM;
		goto out;
	}

	while (*p && nb < GRAPE_READ_MAX) {
		unsigned int v;

		while (*p == ' ' || *p == ',' || *p == '\n' ||
		       *p == '\t')
			p++;
		if (!*p)
			break;
		if (sscanf(p, "%2x", &v) != 1)
			break;
		tx[nb++] = (u8)v;
		while (*p && *p != ' ' && *p != ',' && *p != '\n')
			p++;
	}

	/* Clocking past the written bytes is how you see a reply that
	 * arrives after the command, so honour raw_len when it is longer. */
	want = raw_len ? raw_len : nb;
	if (!want || want > GRAPE_READ_MAX) {
		ret = -EINVAL;
		goto out;
	}
	if ((raw_mode == 1) && (want & 1))
		want++;

	mutex_lock(&n->lock);
	ret = grape_raw_do(n, tx, rx, want);
	if (!ret) {
		if (!n->raw_rx)
			n->raw_rx = devm_kzalloc(&n->spi->dev,
						 GRAPE_READ_MAX, GFP_KERNEL);
		if (n->raw_rx) {
			memcpy(n->raw_rx, rx, want);
			n->raw_n = want;
		}
	}
	mutex_unlock(&n->lock);

	dev_info(&n->spi->dev,
		 "raw_xfer mode=%d len=%u ret=%d rx %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 raw_mode, want, ret, rx[0], rx[1], rx[2], rx[3],
		 rx[4], rx[5], rx[6], rx[7]);
out:
	kfree(tx);
	kfree(rx);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(raw_xfer);

/*
 * attn sysfs: the line the application drives when it has a report.
 * Readable straight from DIN, which is why touch does not have to wait
 * for the EIC to be understood.
 */
static ssize_t attn_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct grape *n = dev_get_drvdata(dev);

	if (!n || !n->attn)
		return sysfs_emit(buf, "-1\n");
	return sysfs_emit(buf, "%d\n",
			  gpiod_get_value_cansleep(n->attn));
}
static DEVICE_ATTR_RO(attn);

static DEVICE_ATTR_RO(state);

/* Force a re-run of the bring-up without unbinding the driver. */
static ssize_t redownload_store(struct device *dev,
				struct device_attribute *a,
				const char *buf, size_t count)
{
	struct grape *n = spi_get_drvdata(to_spi_device(dev));
	int ret;

	if (!n)
		return -ENODEV;
	if (buf[0] != '1')
		return -EINVAL;
	ret = n31_touch_suspend();
	if (!ret)
		ret = n31_touch_resume();
	return ret ? ret : count;
}
/*
 * Re-arm ATTN after it was masked.
 *
 * Masking is not a fix, it is a way to stop a level-triggered line the
 * part never releases from spinning the CPU. Once something has actually
 * changed, write here to let it try again without a module reload.
 */
static ssize_t attn_rearm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct grape *n = dev_get_drvdata(dev);
	bool rearm = false;

	if (!n)
		return -ENODEV;
	mutex_lock(&n->lock);
	if (n->irq_masked) {
		n->irq_masked = false;
		n->ping_fails = 0;
		n->attn_fails = 0;
		n->irq_fails = 0;
		rearm = true;
	}
	mutex_unlock(&n->lock);

	if (rearm && n->irq > 0) {
		enable_irq(n->irq);
		dev_info(dev, "ATTN re-armed\n");
	}
	return count;
}
static DEVICE_ATTR_WO(attn_rearm);

static DEVICE_ATTR_WO(redownload);

static struct attribute *grape_attrs[] = {
	&dev_attr_attn_rearm.attr,
	&dev_attr_raw_xfer.attr,
	&dev_attr_attn.attr,
	&dev_attr_state.attr,
	&dev_attr_redownload.attr,
	NULL,
};
ATTRIBUTE_GROUPS(grape);

static int grape_probe(struct spi_device *spi)
{
	struct grape *n;
	struct input_dev *input;
	const struct firmware *fw = NULL;
	const u8 *data;
	u8 *kbuf = NULL;
	size_t size;
	u16 ping_st = 0;
	int err;

	n = devm_kzalloc(&spi->dev, sizeof(*n), GFP_KERNEL);
	if (!n)
		return -ENOMEM;
	n->spi = spi;
	/*
	 * 8-bit only. This used to arm a 16-bit blob transport.
	 *
	 * Stock has exactly one SPI transport and it is byte-at-a-time:
	 * sub_4043D0's PIO loop is "ldrb.w r3,[ip],#1 / str r3,[SPITXDATA]"
	 * with +0x4C set to 1 per byte, and its IRQ engine does the same.
	 * The word-size bits in SPISETUP are a constant stock never varies.
	 *
	 * The Grape really is a 16-bit slave, but stock feeds it big-endian
	 * 16-bit words as BYTE PAIRS, doing the halfword swap in the CPU
	 * while packing the buffer (frame builder at EA 0x0003B9D0: each
	 * source u32 W is emitted as (W>>8), W, (W>>24), (W>>16)). It never
	 * sets a 16-bit SPI word width to achieve it.
	 *
	 * That also explains the note at the top of this file that 16-bit
	 * writes fail 4BC1 -- the byte order was being produced by the
	 * controller instead of by the packing, which is not the same thing.
	 */
	n->blob16 = false;
	grape_verbose = verbose || !quiet;
	mutex_init(&n->lock);
	spi_set_drvdata(spi, n);
	grape_pm_dev = n;
	if (sysfs_create_groups(&spi->dev.kobj, grape_groups))
		dev_warn(&spi->dev, "sysfs groups failed\n");

	n->gpio_base = devm_ioremap(&spi->dev, S5L8740_GPIO_PHYS, 0x400);
	n->gpiocmd = devm_ioremap(&spi->dev, S5L8740_GPIOCMD_PHYS, 4);
	n->spi2 = devm_ioremap(&spi->dev, S5L8740_SPI2_PHYS, 0x80);
	if (!n->gpio_base || !n->gpiocmd || !n->spi2)
		return -ENOMEM;

	n->enable = devm_gpiod_get_optional(&spi->dev, "enable", GPIOD_ASIS);
	if (IS_ERR(n->enable))
		return PTR_ERR(n->enable);
	n->reset = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_ASIS);
	if (IS_ERR(n->reset))
		return PTR_ERR(n->reset);
	n->attn = devm_gpiod_get_optional(&spi->dev, "attn", GPIOD_IN);
	if (IS_ERR(n->attn))
		return PTR_ERR(n->attn);

	err = grape_acquire_touch_cal_cal(n);
	if (err)
		return err;
	if (!sysfs_create_bin_file(&spi->dev.kobj, &touch_cal_blob_attr))
		n->touch_cal_sysfs = true;
	else
		dev_warn(&spi->dev, "touch_cal_blob sysfs failed\n");

	data = NULL;
	size = 0;
	err = grape_acquire_fw(&spi->dev, &data, &size, &fw, &kbuf,
				 &n->fw_from_ftl);
	if (err)
		dev_warn(&spi->dev,
			 "no grape firmware (%d) — A34 touch calibration is present but 2D640 cannot run\n",
			 err);

	if (!data || !size) {
		grape_park(n, "no grape firmware");
	} else {
		int attempt;

		/*
		 * 13A20(1): first try is 1A5AC only. 1A878 + sleep 50
		 * only after a failed 1A5AC, max 3. remove() already
		 * 1A878s on reload.
		 */
		/*
		 * sub_20E94 reads PMIC 0x51 here, between the 26494 probe and
		 * the download loop, and discards the value. Do the same: the
		 * read itself may be the point.
		 */
		{
			int (*pmic_read)(void);

			pmic_read = (int (*)(void))
				__symbol_get("d1830_touch_bringup_read");
			if (pmic_read) {
				pmic_read();
				__symbol_put("d1830_touch_bringup_read");
			}
		}

		for (attempt = 0; attempt < 3; attempt++) {
			if (attempt) {
				grape_power_down(n);
				msleep(50);
			}
			/* 0xEE is iOS3 Z2-only; N31 wake is 19 C1 in reset. */
			mutex_lock(&n->lock);
			n->fw_uploaded = false;
			n->cal_uploaded = false;
			n->requestcal_done = false;
			n->exec_sent = false;
			n->runtime_ready = false;
			n->fw_loaded = false;
			n->spi_ok = false;
			grape_1a5ac_and_download(n, data, size,
						  attempt ? "retry" : "1A5AC");
			{
				u16 st = 0;

				/*
				 * Cross EXEC boundary: short wait then runtime
				 * ping only. No HBPP 1A A1 until ping fails.
				 */
				/*
				 * No wait when EXEC was sent: the single 40 ms settle already
				 * happened in grape_exec(), which owns the state transition.
				 * sub_273A0 waits once, not three times.
				 */
				if (!n->exec_sent)
					msleep(2);
				err = grape_ping(n, &ping_st);
				if (!err) {
					n->spi_ok = true;
					n->runtime_ready = true;
					n->fw_loaded = true;
					grape_dev_vinfo(&spi->dev,
						 "runtime ping ok status=0x%04x (ready)\n",
						 ping_st);
				} else {
					dev_warn(&spi->dev,
						 "runtime ping fail attempt %d (exec_sent=%d)\n",
						 attempt, n->exec_sent);
					grape_peek(n, "post-go-fail");
					/* Diagnostics only after runtime fail. */
					if (grape_status_poll(n, &st) == 0)
						grape_dev_vinfo(&spi->dev,
							 "post-fail HBPP status 0x%04x\n",
							 st);
					err = 0; /* keep 1A878 retries going */
				}
			}
			mutex_unlock(&n->lock);
			/*
			 * Stock retries sub_273A0 only when the EXEC transfer
			 * itself failed, never because a ping came back
			 * wrong. Re-entering the loop re-asserts reset and
			 * re-uploads, so treating a failed ping as a reason
			 * to retry destroys an application that had already
			 * started.
			 */
			if (n->runtime_ready ||
			    (!require_runtime && n->exec_sent))
				break;
		}
	}
	grape_release_fw(fw, kbuf);
	grape_dev_vinfo(&spi->dev,
		 "grape state uploaded=%d cal=%d reqcal=%d exec=%d runtime=%d spi_ok=%d\n",
		 n->fw_uploaded, n->cal_uploaded, n->requestcal_done,
		 n->exec_sent, n->runtime_ready, n->spi_ok);

	/*
	 * Input device first, ATN second.
	 *
	 * The threaded level-low IRQ used to be requested before the input
	 * device existed. ATN is level-sensitive and may already be asserted
	 * at this point, so the handler could reach grape_parse_D() with
	 * n->input still NULL. grape_parse_D() now returns early on a NULL
	 * input as well, but the ordering is the actual fix.
	 */
	input = devm_input_allocate_device(&spi->dev);
	if (!input) {
		grape_touch_cal_sysfs_remove(n);
		return -ENOMEM;
	}
	n->input = input;
	input->name = "Apple Grape";
	input->phys = "grape/input0";
	input->id.bustype = BUS_SPI;
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, GRAPE_ABS_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, GRAPE_ABS_Y_MAX, 0, 0);
	err = input_mt_init_slots(input, GRAPE_SLOTS, INPUT_MT_DIRECT);
	if (err) {
		grape_touch_cal_sysfs_remove(n);
		return err;
	}
	err = input_register_device(input);
	if (err) {
		grape_touch_cal_sysfs_remove(n);
		return err;
	}

	if (n->cal_uploaded && n->exec_sent &&
	    (n->runtime_ready || !require_runtime)) {
			/*
		 * sub_1A5AC arms ATN (sub_20490) only after sub_20E94
		 * reports success. Done here, with the input device already
		 * registered, so no interrupt can outrun it.
		 *
		 * No IRQF_TRIGGER_LOW: that makes genirq call eic_set_type()
		 * and rewrite the INTLEVEL/INTTYPE pair grape_irq_enable()
		 * just programmed from sub_40641C.
		 */
		grape_irq_enable(n);
		n->irq = spi->irq;
		if (n->irq > 0) {
			err = devm_request_threaded_irq(&spi->dev, n->irq, NULL,
							grape_irq_thread,
							IRQF_ONESHOT,
							"grape", n);
			if (err) {
				dev_warn(&spi->dev,
					 "VIC IRQ %d request failed %d — attn poll\n",
					 n->irq, err);
				n->irq = -1;
			} else {
				n->use_irq = true;
				grape_dev_vinfo(&spi->dev,
					 "IRQ-driven (VIC irq %d + EIC GPIO%d)\n",
					 n->irq, GRAPE_GPIO_IRQ);
			}
		}
	} else {
		dev_err(&spi->dev,
			"Grape boot failed: cal=%d exec=%d runtime=%d; not registering input\n",
			n->cal_uploaded, n->exec_sent, n->runtime_ready);
		if (!n->parked)
			grape_park(n, "boot incomplete");
		return 0;
	}


	if (ping_st) {
		mutex_lock(&n->lock);
		if (ping_st)
			grape_read_reports(n, ping_st);
		mutex_unlock(&n->lock);
	}
	n->thread = kthread_run(grape_poll_thread, n, "grape-poll");
	if (IS_ERR(n->thread)) {
		dev_warn(&spi->dev,
			 "grape-poll kthread %ld — poll via IRQ only\n",
			 PTR_ERR(n->thread));
		n->thread = NULL;
	}

	dev_info(&spi->dev, "Grape up (attn=%d runtime=%d)\n",
		 !!n->attn, n->runtime_ready);
	return 0;
}

/*
 * OSOS sub_1A878 is the disable path -- 20490(0), RST asserted, 20690(0)
 * to release the SPI2 pads, rail off, EN mode 1 -- and grape_power_down
 * already implements it. It was only ever reached through remove(),
 * which a shutdown or kexec does not call, so the part was left powered
 * and holding the bus across the handover.
 */
static void grape_shutdown(struct spi_device *spi)
{
	struct grape *n = spi_get_drvdata(spi);

	if (!n)
		return;
	n->stopped = true;
	if (n->thread)
		kthread_stop(n->thread);
	n->thread = NULL;
	grape_power_down(n);
}

static void grape_remove(struct spi_device *spi)
{
	grape_pm_dev = NULL;
	struct grape *n = spi_get_drvdata(spi);

	n->stopped = true;
	if (n->thread)
		kthread_stop(n->thread);
	grape_touch_cal_sysfs_remove(n);
	/*
	 * probe creates these but nothing removed them, so the group
	 * outlived the module. The next insmod then hit a duplicate
	 * filename, and internal_create_group rolls the whole group back
	 * on failure -- so a second load silently lost every attribute,
	 * including state and redownload. It looked like the attributes
	 * were never registered rather than registered twice.
	 */
	sysfs_remove_groups(&spi->dev.kobj, grape_groups);
	grape_clkcon_restore();
	grape_power_down(n);
}

static const struct of_device_id grape_of_match[] = {
	{ .compatible = "apple,grape" },
	{ }
};
MODULE_DEVICE_TABLE(of, grape_of_match);

static struct spi_driver grape_driver = {
	.driver = {
		.name = "apple-grape",
		.of_match_table = grape_of_match,
	},
	.probe = grape_probe,
	.remove = grape_remove,
	.shutdown = grape_shutdown,
};
module_spi_driver(grape_driver);

MODULE_DESCRIPTION("Apple Grape multitouch (N31 SPI2, RetailOS-matched)");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("apple/grape.bin");
