// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple Nimbus / Grape multitouch — N31 SPI2 @ 0x3D200000
 *
 * RetailOS 1.0.2 sequences (osos extracts):
 *   bring-up     sub_1A5AC / 2075A / 20766 / 20690 / 11B70
 *   teardown     sub_1A878: IRQ off, RST, 20690(0), rail off, EN mode 1
 *                (13A20 retries: 1A878 + sleep 50 + 1A5AC, max 3)
 *   grape.bin    IS the app. SEC bootloader has no grape/Nimbus path.
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
 *   Per-device IsyS comes from the A34 handoff, not a host file:
 *     desc @ 0x2202FE18 (sub_A34(24)): magic 0x53797349 "IsyS", ptr @ +4
 *     memcpy 0x560 from ptr (sub_564)
 *     cal = bytes at decimal +350, length 0x200, reverse each u32 (sub_273A0)
 *     then 2D7A4 that window; DATA packet still does b1b0b3b2 wire swizzle
 *   Callsite 2D7A4: r1 = 0x00400200 + offset.
 *   No grape-nimbus-cal.bin, no FTL IsyS scan, no GrapeFirmware.bin+350.
 *   Preferred source: U-Boot copy in reserved DRAM, advertised in /chosen
 *     apple,n31-isys-addr / apple,n31-isys-size. Never consume the original
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
 * Firmware host file: request_firmware("apple/grape-nimbus.bin") and/or
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

#define NIMBUS_MAGIC		0xEA
#define NIMBUS_PING_TYPE	490
#define NIMBUS_BOOTLOAD_WORD	6593		/* 0x19C1 */
#define NIMBUS_FRAME_LEN	16
#define NIMBUS_READ_MAX		512
#define NIMBUS_SLOTS		8
#define NIMBUS_ABS_X_MAX	239
#define NIMBUS_ABS_Y_MAX	431
#define NIMBUS_SCALE_X_DIV	0x0B1D
#define NIMBUS_SCALE_Y_DIV	0x1482

#define NIMBUS_CHUNK_MAX	0x1FF0		/* 8176 — sub_2D640 */
#define NIMBUS_HDR_LEN		16		/* 2 outer + 10 body hdr + 4 payload sum */
#define NIMBUS_CAL_DEST		0x00400200u	/* 2D7A4 literal 0x400200 */
#define NIMBUS_FW_HDR_OFF	350
#define NIMBUS_FW_HDR_LEN	0x200
#define NIMBUS_ARM_OFFICIAL	0xe970		/* 8740 le32(+0x0c); 204E0 2D640 size */
#define NIMBUS_ISYS_MAGIC	0x53797349u	/* 'IsyS' — sub_564 */
#define NIMBUS_ISYS_LEN		0x560
#define NIMBUS_A34_BASE		0x2202fe00UL	/* sub_A34(idx) = 0x2202FE00+idx */
#define NIMBUS_A34_ISYS_DESC	(NIMBUS_A34_BASE + 0x18)	/* sub_A34(24) */

/* Whimory FTL (fmss-s5l8740.ko) — optional cal/FW from device NAND */
#define NIMBUS_FTL_SECTOR_SIZE	4096U
#define NIMBUS_GPFW_TAG		0x67706677u	/* 'gpfw' LE */

#define NIMBUS_ACK_CHUNK	0x4BC1		/* 19393 */
#define NIMBUS_ACK_34AD0	0x4AD1		/* 19153 */
#define NIMBUS_POST_POKE	0x011F		/* 287 */

#define NIMBUS_GPIO_EN		0x0E
#define NIMBUS_GPIO_RST		0x27
#define NIMBUS_GPIO_IRQ		0x26

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

#define NIMBUS_Z2_HDR_LEN	16
#define NIMBUS_Z2_MAGIC_5A5A	0x5a5a0000u
#define NIMBUS_Z2_MAGIC_C3F5	0xc3f50000u
#define NIMBUS_Z2FW_MAGIC	0x5746325au	/* apple_z2 "Z2FW" container */

#define NIMBUS_CS_BEGIN		BIT(0)
#define NIMBUS_CS_END		BIT(1)

static int spi_clkdiv = 16;
module_param(spi_clkdiv, int, 0644);
MODULE_PARM_DESC(spi_clkdiv, "SPI2 CLKDIV (higher=slower; try 8-32 for FW download)");
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
static int quiet;
module_param(quiet, int, 0644);
MODULE_PARM_DESC(quiet, "1=minimal logs (auto after GO fail)");
static int skip_download;
module_param(skip_download, int, 0644);
MODULE_PARM_DESC(skip_download, "1=bootload+ping only, no FW chunks");

static int force_gid;
module_param(force_gid, int, 0644);
MODULE_PARM_DESC(force_gid, "1=422FFA decrypt attempt even without 8740 rev3 hdr");

static int cal_try_dt = 1;
module_param(cal_try_dt, int, 0644);
MODULE_PARM_DESC(cal_try_dt,
		 "Read U-Boot IsyS copy from /chosen apple,n31-isys-* (default on)");

static int cal_try_a34 = 1;
module_param(cal_try_a34, int, 0644);
MODULE_PARM_DESC(cal_try_a34,
		 "Fallback: read live A34 descriptor at 0x2202FE18 (default on)");

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
		 "FTL LBAs to scan for gpfw/8740 firmware (not IsyS cal)");

static int fw_prefer_ftl;
module_param(fw_prefer_ftl, int, 0644);
MODULE_PARM_DESC(fw_prefer_ftl,
		 "1=try gpfw/8740 from FTL before grape-nimbus.bin (DFU default 0)");

static int fw_allow_file = 1;
module_param(fw_allow_file, int, 0644);
MODULE_PARM_DESC(fw_allow_file, "1=allow apple/grape-nimbus.bin fallback");

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

/* fmss-s5l8740.ko exports (optional link). */
bool fmss_ftl_present(void);
int fmss_ftl_read_sector(u64 logical_sector, void *buf);

static bool nimbus_verbose = true;

#define nimbus_vinfo(n, fmt, ...) \
	do { \
		if (nimbus_verbose && !(n)->parked) \
			dev_info(&(n)->spi->dev, fmt, ##__VA_ARGS__); \
	} while (0)

struct nimbus {
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
	bool runtime_ready;	/* valid 182590 ping checksum */
	bool fw_loaded;		/* alias of runtime_ready for older call sites */
	bool fw_tried;
	bool spi_ok;
	bool use_irq;
	bool blob16;	/* S5L TXDATA is 8-bit; 16-bit writes fail 4BC1 */
	bool parked;	/* give up after recycle budget — stop SPI spam */
	bool have_isys;
	bool have_cal;
	bool isys_sysfs;
	u8 isys[NIMBUS_ISYS_LEN];
	u8 cal_upload[NIMBUS_FW_HDR_LEN];
	int irq;
	unsigned int ping_fails;
	unsigned int recycle_count;
};

/* From irq-s5l8740-eic.c */
int s5l8740_eic_enable_gpio(unsigned int gpio, unsigned int irq_type);
/* From gpio-d1830.c — OSOS 20766 / 6644(4) / reg16 bit5 */
int d1830_nimbus_rail(bool on);

static u16 nimbus_sum16(const u8 *buf, int len)
{
	u16 sum = 0;

	while (len-- > 0)
		sum += *buf++;
	return sum;
}

static void nimbus_gpiocmd_mode(struct nimbus *n, unsigned int gpio, u16 mode, int val)
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
static void nimbus_punc(struct nimbus *n, unsigned int gpio, bool set)
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
static void nimbus_spi2_pinmux(struct nimbus *n, bool on)
{
	if (on) {
		nimbus_punc(n, 0x57, false);
		nimbus_gpiocmd_mode(n, 0x57, 5, 0);
		nimbus_gpiocmd_mode(n, 0x58, 3, 0);
		nimbus_gpiocmd_mode(n, 0x59, 3, 0);
		nimbus_gpiocmd_mode(n, 0x5A, 3, 0);
	} else {
		nimbus_gpiocmd_mode(n, 0x57, 0xFFFE, 0);
		nimbus_punc(n, 0x57, true);
		nimbus_gpiocmd_mode(n, 0x58, 1, 0);
		nimbus_gpiocmd_mode(n, 0x59, 1, 0);
		nimbus_gpiocmd_mode(n, 0x5A, 0xFFFE, 0);
	}
}

/*
 * sub_1A878 — disable / retry power-cut:
 *   20490(0), RST assert, 20690(0), 20766(0) rail off + EN mode 1.
 */
static void nimbus_power_down(struct nimbus *n)
{
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_IRQ, 0xFFFE, 0);
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_RST, 1, 0);
	nimbus_spi2_pinmux(n, false);
	d1830_nimbus_rail(false);
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_EN, 1, 0);
	dev_info(&n->spi->dev, "1A878 power-cut (RST hold, rail off, EN mode 1)\n");
}

/*
 * sub_11B70(2, 0x1A, 0x2EE0, 1) after every 20690(1).
 * 1A5AC always re-inits SPI2 here. Skipping it after 1A878 remux
 * left an extra SCLK edge: 0x1f01/0x4879 came back as 0x0f80/0xa43c
 * and shifted one more bit on each retry.
 */
static void nimbus_spi2_11b70(struct nimbus *n)
{
	u32 setup;

	if (!n->spi2)
		return;
	writel(0xf, n->spi2 + SPI2_STATUS);
	writel(readl(n->spi2 + SPI2_CTRL) | SPI2_CTRL_FIFO_RST,
	       n->spi2 + SPI2_CTRL);
	writel(10, n->spi2 + 0x44);
	writel(24, n->spi2 + 0x38);	/* 24 * a4=1 */
	writel(255, n->spi2 + 0x40);
	writel(144, n->spi2 + 0x3c);	/* 3 * 24 * (1+1) */
	writel(clamp(spi_clkdiv, 1, 255), n->spi2 + SPI2_CLKDIV);
	writel(SPI2_SETUP_11B70, n->spi2 + SPI2_SETUP);
	writel(readl(n->spi2 + SPI2_CTRL) | SPI2_CTRL_FIFO_RST,
	       n->spi2 + SPI2_CTRL);
	writel(SPI2_CTRL_ENABLE, n->spi2 + SPI2_CTRL);
	setup = readl(n->spi2 + SPI2_SETUP);
	dev_info(&n->spi->dev, "11B70 SPI2 SETUP=0x%x CLKDIV=%u\n",
		 setup, readl(n->spi2 + SPI2_CLKDIV));
}

static void nimbus_spi2_cs(struct nimbus *n, bool assert)
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
static void nimbus_spi2_fifo_flush(struct nimbus *n)
{
	if (!n->spi2)
		return;
	writel(readl(n->spi2 + SPI2_CTRL) | SPI2_CTRL_FIFO_RST,
	       n->spi2 + SPI2_CTRL);
	writel(0xf, n->spi2 + SPI2_STATUS);
}

static int nimbus_burst_ex(struct nimbus *n, const u8 *tx, u8 *rx,
			   unsigned int len, unsigned int cs_flags)
{
	unsigned int i, guard;
	u32 st;

	if (!n->spi2)
		return -ENODEV;
	if (cs_flags & NIMBUS_CS_BEGIN) {
		nimbus_spi2_cs(n, true);
		ndelay(2000);
		nimbus_spi2_fifo_flush(n);
		writel(readl(n->spi2 + SPI2_SETUP) & ~BIT(0), n->spi2 + SPI2_SETUP);
		writel(readl(n->spi2 + SPI2_STATUS) | 0x400000u, n->spi2 + SPI2_STATUS);
	}
	for (i = 0; i < len; i++) {
		writel(1, n->spi2 + SPI2_RXLIMIT);
		guard = 100000;
		do {
			st = readl(n->spi2 + SPI2_STATUS);
		} while ((st & 0x7c0) != 0 && (st & 0x7c0) != 0x40 && --guard);
		writel(tx[i], n->spi2 + SPI2_TXDATA);
		writel(1, n->spi2 + SPI2_UNK4C);
		guard = 100000;
		do {
			st = readl(n->spi2 + SPI2_STATUS);
		} while (!(st & 0xf800) && --guard);
		if (rx)
			rx[i] = (u8)readl(n->spi2 + SPI2_RXDATA);
		else
			readl(n->spi2 + SPI2_RXDATA);
	}
	if (cs_flags & NIMBUS_CS_END) {
		writel(readl(n->spi2 + SPI2_SETUP) & ~0x400001u, n->spi2 + SPI2_SETUP);
		nimbus_spi2_cs(n, false);
	}
	return 0;
}

static int nimbus_burst(struct nimbus *n, const u8 *tx, u8 *rx, unsigned int len)
{
	return nimbus_burst_ex(n, tx, rx, len, NIMBUS_CS_BEGIN | NIMBUS_CS_END);
}

/*
 * S5LBox §6.1: every 32-bit HBPP field is middle-endian because the
 * part is a 16-bit SPI slave. Our packed buffer is already wire-byte
 * order (18 E1 30 01 …); pair as BE u16 so TXDATA 0x18E1 clocks 18 then E1.
 * If TXDATA is 8-bit-only, only the low byte leaves and 4BC1 fails —
 * send_chunk then falls back to 8-bit PIO.
 */
static int nimbus_burst_u16_ex(struct nimbus *n, const u8 *tx, u8 *rx,
			       unsigned int len, unsigned int cs_flags)
{
	unsigned int i, guard;
	u32 st;

	if (!n->spi2)
		return -ENODEV;
	if (len & 1)
		return nimbus_burst_ex(n, tx, rx, len, cs_flags);
	if (cs_flags & NIMBUS_CS_BEGIN) {
		nimbus_spi2_cs(n, true);
		ndelay(2000);
		nimbus_spi2_fifo_flush(n);
		writel(readl(n->spi2 + SPI2_SETUP) & ~BIT(0), n->spi2 + SPI2_SETUP);
		writel(readl(n->spi2 + SPI2_STATUS) | 0x400000u, n->spi2 + SPI2_STATUS);
	}
	for (i = 0; i < len; i += 2) {
		u16 w = ((u16)tx[i] << 8) | tx[i + 1];
		u16 r;

		writel(1, n->spi2 + SPI2_RXLIMIT);
		guard = 100000;
		do {
			st = readl(n->spi2 + SPI2_STATUS);
		} while ((st & 0x7c0) != 0 && (st & 0x7c0) != 0x40 && --guard);
		writel(w, n->spi2 + SPI2_TXDATA);
		writel(1, n->spi2 + SPI2_UNK4C);
		guard = 100000;
		do {
			st = readl(n->spi2 + SPI2_STATUS);
		} while (!(st & 0xf800) && --guard);
		r = (u16)readl(n->spi2 + SPI2_RXDATA);
		if (rx) {
			rx[i] = (u8)(r >> 8);
			rx[i + 1] = (u8)r;
		}
	}
	if (cs_flags & NIMBUS_CS_END) {
		writel(readl(n->spi2 + SPI2_SETUP) & ~0x400001u, n->spi2 + SPI2_SETUP);
		nimbus_spi2_cs(n, false);
	}
	return 0;
}

static int nimbus_burst_u16(struct nimbus *n, const u8 *tx, u8 *rx,
			    unsigned int len)
{
	return nimbus_burst_u16_ex(n, tx, rx, len,
				   NIMBUS_CS_BEGIN | NIMBUS_CS_END);
}

static int nimbus_burst16(struct nimbus *n, const u8 *tx, u8 *rx)
{
	return nimbus_burst(n, tx, rx, NIMBUS_FRAME_LEN);
}

static int nimbus_xfer(struct nimbus *n, const u8 *tx, u8 *rx, unsigned int len)
{
	u8 *drain = NULL;
	int ret;
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = len,
	};
	struct spi_message m;

	/*
	 * s5l8702 pio_one skips RXDATA when rx==NULL. 2D640 is TX-only
	 * in OSOS (40F770 dest 0) but that path still drains the FIFO.
	 * Without a drain, 8 KiB chunks overflow RX and the payload
	 * after the 12-byte header is dropped — 4BC1 can still ACK.
	 */
	if (!rx) {
		drain = kzalloc(len, GFP_KERNEL);
		if (!drain)
			return -ENOMEM;
		t.rx_buf = drain;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(n->spi, &m);
	kfree(drain);
	return ret;
}

/* sub_2C87E — bootloader opcode whitelist */
static bool nimbus_opcode_known(u16 w);
static bool nimbus_looks_like_arm(const u8 *p, size_t n);
static void nimbus_bswap32_words(u8 *p, unsigned int len);
static u32 nimbus_sum32(const u8 *p, unsigned int len);

static bool nimbus_opcode_known(u16 w)
{
	return w == 0x18e1 || w == 0x1aa1 || w == 0x1f01 || w == 0x19c1 ||
	       w == 0x4879 || w == 0x4bc1 || w == 0x4969 || w == 0x4ad1;
}

/* sub_26494 — 16↔16 1A A1 + 18 E1 pad; two rev16 words must be known */
static int nimbus_probe_26494(struct nimbus *n, const char *tag)
{
	u8 tx[NIMBUS_FRAME_LEN];
	u8 rx[NIMBUS_FRAME_LEN] = { 0 };
	unsigned int i;
	u16 w0, w1;
	int ret;

	tx[0] = 0x1a;
	tx[1] = 0xa1;
	for (i = 2; i < NIMBUS_FRAME_LEN; i += 2) {
		tx[i] = 0x18;
		tx[i + 1] = 0xe1;
	}
	ret = nimbus_xfer(n, tx, rx, NIMBUS_FRAME_LEN);
	w0 = (u16)((rx[0] << 8) | rx[1]);
	w1 = (u16)((rx[2] << 8) | rx[3]);
	dev_info(&n->spi->dev,
		 "26494 %s ret=%d words 0x%04x 0x%04x known=%d rx %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 tag, ret, w0, w1, nimbus_opcode_known(w0) && nimbus_opcode_known(w1),
		 rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
	if (ret)
		return ret;
	if (!nimbus_opcode_known(w0) || !nimbus_opcode_known(w1))
		return -EIO;
	return 0;
}

/* sub_3D5706 — TX 1A A1, RX 2, byteswap */
static int nimbus_status_poll(struct nimbus *n, u16 *status)
{
	u8 tx[2] = { 0x1a, 0xa1 };
	u8 rx[2] = { 0 };
	int ret;

	ret = nimbus_xfer(n, tx, rx, 2);
	if (ret)
		return ret;
	if (status)
		*status = (u16)((rx[0] << 8) | rx[1]); /* __rev16 of LE word */
	return 0;
}

static bool nimbus_fw_has_8740_hdr(const u8 *data, size_t size)
{
	return size >= 8 && data[0] == '8' && data[1] == '7' &&
	       data[2] == '4' && data[3] == '0';
}

static bool nimbus_fw_has_z2fw_hdr(const u8 *data, size_t size)
{
	u32 magic;

	if (size < 8)
		return false;
	magic = get_unaligned_le32(data);
	return magic == NIMBUS_Z2FW_MAGIC;
}

/* Classify host grape file: full 8740 container vs ARM-only cut vs Z2FW. */
static void nimbus_fwfile_classify(struct nimbus *n, const u8 *data, size_t size)
{
	bool h8740 = nimbus_fw_has_8740_hdr(data, size);
	bool hz2 = nimbus_fw_has_z2fw_hdr(data, size);
	bool arm0 = size >= 4 && nimbus_looks_like_arm(data, size);
	bool arm400 = size >= 0x410 && nimbus_looks_like_arm(data + 0x400, 16);
	u32 le0c = (h8740 && size >= 0x10) ? get_unaligned_le32(data + 0xc) : 0;
	u8 rev = (h8740 && size >= 5) ? data[4] : 0;

	dev_info(&n->spi->dev,
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
			 "FWFILE is ARM-only cut — grape file +350 is not IsyS cal\n");
}

static void __maybe_unused nimbus_log_calcand(struct nimbus *n, const char *name,
			       const u8 *data, size_t size, unsigned int off)
{
	u8 tmp[NIMBUS_FW_HDR_LEN];
	u32 s;

	if (size < off + NIMBUS_FW_HDR_LEN) {
		dev_info(&n->spi->dev, "CALCAND %s off=%u OOB (file=%zu)\n",
			 name, off, size);
		return;
	}
	memcpy(tmp, data + off, NIMBUS_FW_HDR_LEN);
	nimbus_bswap32_words(tmp, NIMBUS_FW_HDR_LEN);
	s = nimbus_sum32(tmp, NIMBUS_FW_HDR_LEN);
	dev_info(&n->spi->dev,
		 "CALCAND %s off=%u sum32=0x%08x first16=%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x (post-bswap)\n",
		 name, off, s,
		 tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6],
		 tmp[7], tmp[8], tmp[9], tmp[10], tmp[11], tmp[12], tmp[13],
		 tmp[14], tmp[15]);
}

static void __maybe_unused nimbus_dump_calcands(struct nimbus *n, const u8 *data,
						size_t size)
{
	/* Diagnostic only — does not select a candidate for upload. */
	nimbus_log_calcand(n, "dec350", data, size, 350);
	nimbus_log_calcand(n, "hex350", data, size, 0x350);
	nimbus_log_calcand(n, "arm_plus_dec350", data, size, 0x400 + 350);
	nimbus_log_calcand(n, "arm_plus_hex350", data, size, 0x400 + 0x350);
}

static u32 nimbus_crc32_payload(const u8 *p, size_t len)
{
	return crc32_le(~0U, p, len) ^ ~0U;
}

static void nimbus_fw_audit(struct nimbus *n, const u8 *body, size_t body_len,
			    const char *tag)
{
	u32 crc;
	size_t pad;

	if (!body_len)
		return;
	crc = nimbus_crc32_payload(body, body_len);
	pad = body_len & 3u;
	dev_info(&n->spi->dev,
		 "FW audit %s %zuB arm=%d crc32=0x%08x pad=%zu\n",
		 tag, body_len, nimbus_looks_like_arm(body, body_len), crc, pad);
	if (body_len >= 16) {
		u32 m = get_unaligned_le32(body);
		u32 ln_le = get_unaligned_le32(body + 4);
		u32 ln_be = get_unaligned_be32(body + 4);
		u32 c_le = get_unaligned_le32(body + 8);

		if (m == NIMBUS_Z2_MAGIC_5A5A || m == NIMBUS_Z2_MAGIC_C3F5)
			dev_info(&n->spi->dev,
				 "  z2-dl hdr magic=0x%08x len_le=%u len_be=%u crc=0x%08x\n",
				 m, ln_le, ln_be, c_le);
	}
}

static int nimbus_build_z2_dl_hdr(u8 *hdr, const u8 *payload, size_t len,
				 u32 magic)
{
	u32 crc = nimbus_crc32_payload(payload, len);

	put_unaligned_le32(magic, hdr);
	put_unaligned_be32(len, hdr + 4);
	put_unaligned_le32(crc, hdr + 8);
	put_unaligned_le32(0, hdr + 12);
	return 0;
}

static u8 *nimbus_maybe_prepend_z2_hdr(struct nimbus *n, const u8 *body,
				       size_t body_len, size_t *out_len)
{
	u8 *buf;
	u32 magic;

	if (!prepend_z2_hdr || body_len < 4)
		return NULL;
	magic = prepend_z2_hdr == 2 ? NIMBUS_Z2_MAGIC_C3F5 : NIMBUS_Z2_MAGIC_5A5A;
	buf = kmalloc(NIMBUS_Z2_HDR_LEN + body_len + 3, GFP_KERNEL);
	if (!buf)
		return NULL;
	nimbus_build_z2_dl_hdr(buf, body, body_len, magic);
	memcpy(buf + NIMBUS_Z2_HDR_LEN, body, body_len);
	*out_len = NIMBUS_Z2_HDR_LEN + body_len;
	if (*out_len & 3) {
		memset(buf + *out_len, 0, 4 - (*out_len & 3));
		*out_len = round_up(*out_len, 4);
	}
	dev_info(&n->spi->dev,
		 "prepended Z2 dl hdr magic=0x%08x total=%zu\n", magic, *out_len);
	return buf;
}

static int nimbus_wait_ack(struct nimbus *n, u16 expect, int retries)
{
	int i;
	u16 st = 0;

	for (i = 0; i < retries; i++) {
		if (nimbus_status_poll(n, &st) == 0 && st == expect)
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
static int nimbus_rdreg(struct nimbus *n, u32 addr, u32 *val)
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
	csum = nimbus_sum16(tx + 2, 4);
	tx[6] = (csum >> 8) & 0xff;
	tx[7] = csum & 0xff;

	ret = nimbus_xfer(n, tx, rx, 8);
	if (ret)
		return ret;
	ret = nimbus_xfer(n, atn_tx, atn_rx, 8);
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

/* iOS3 MT_SPI_Z2_WAKE_CMD — 16-byte frame, opcode 0xEE, LE16 csum 0x00EE */
static int nimbus_hbpp_wake_ee(struct nimbus *n, const char *tag)
{
	u8 tx[NIMBUS_FRAME_LEN] = { 0xee };
	u8 rx[NIMBUS_FRAME_LEN] = { 0 };
	int ret;

	tx[14] = 0xee;
	ret = nimbus_burst16(n, tx, rx);
	dev_info(&n->spi->dev,
		 "HBPP 0xEE wake %s ret=%d rx %02x %02x %02x %02x %02x %02x\n",
		 tag, ret, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5]);
	return ret;
}

static void nimbus_peek(struct nimbus *n, const char *tag)
{
	static const u32 addrs[] = {
		0x00000000, 0x0000d208, 0x0000e970, 0x00400200,
		0x0040f7f4, 0x0040fffc, 0x1000300c, 0x10008ffc,
	};
	unsigned int i;

	if (!nimbus_verbose)
		return;
	for (i = 0; i < ARRAY_SIZE(addrs); i++) {
		u32 v = 0;

		if (nimbus_rdreg(n, addrs[i], &v) == 0)
			nimbus_vinfo(n, "peek %s %08x=%08x\n", tag, addrs[i], v);
	}
}

/* HBPP MemRead, dest packing is B1,B0,B3,B2 — same as DATA offset. */
static int nimbus_rdmem(struct nimbus *n, u32 addr, u8 *buf, unsigned int len)
{
	unsigned int i;

	if (len & 3)
		return -EINVAL;
	for (i = 0; i < len; i += 4) {
		u32 v = 0;

		if (nimbus_rdreg(n, addr + i, &v))
			return -EIO;
		put_unaligned_le32(v, buf + i);
	}
	return 0;
}

/*
 * Prove whether 2D640 landed the ARM image at dest 0 or at the EXEC
 * word 0x00100018. Cal dest 0x400200 is a separate window.
 */
static void nimbus_fw_readback(struct nimbus *n, const char *tag)
{
	static const u32 addrs[] = {
		0x00000000, 0x00000018, 0x00100000, 0x00100018,
		0x00400000, 0x00400200,
	};
	u8 *buf;
	unsigned int i;

	buf = kmalloc(0x1000, GFP_KERNEL);
	if (!buf)
		return;
	dev_info(&n->spi->dev, "NIMBUS FW_READBACK %s:\n", tag);
	for (i = 0; i < ARRAY_SIZE(addrs); i++) {
		u32 crc100, crc1000;

		memset(buf, 0xa5, 0x1000);
		if (nimbus_rdmem(n, addrs[i], buf, 0x1000)) {
			dev_warn(&n->spi->dev,
				 "  addr=%08x RDREG fail\n", addrs[i]);
			continue;
		}
		crc100 = nimbus_crc32_payload(buf, 0x100);
		crc1000 = nimbus_crc32_payload(buf, 0x1000);
		dev_info(&n->spi->dev,
			 "  addr=%08x first32=%32ph crc100=0x%08x crc1000=0x%08x\n",
			 addrs[i], buf, crc100, crc1000);
	}
	kfree(buf);
}

static void nimbus_cal_readback(struct nimbus *n, const u8 *upload)
{
	u8 *buf;
	u32 crc_chip, crc_host;

	buf = kmalloc(NIMBUS_FW_HDR_LEN, GFP_KERNEL);
	if (!buf)
		return;
	if (nimbus_rdmem(n, NIMBUS_CAL_DEST, buf, NIMBUS_FW_HDR_LEN)) {
		dev_warn(&n->spi->dev, "cal readback RDREG fail @0x%08x\n",
			 NIMBUS_CAL_DEST);
		kfree(buf);
		return;
	}
	crc_chip = nimbus_crc32_payload(buf, NIMBUS_FW_HDR_LEN);
	crc_host = nimbus_crc32_payload(upload, NIMBUS_FW_HDR_LEN);
	dev_info(&n->spi->dev,
		 "NIMBUS CAL_READBACK @%08x first64=%32ph %32ph crc200=0x%08x host_crc=0x%08x match=%d\n",
		 NIMBUS_CAL_DEST, buf, buf + 32, crc_chip, crc_host,
		 crc_chip == crc_host && !memcmp(buf, upload, NIMBUS_FW_HDR_LEN));
	kfree(buf);
}

/* sub_20848(6593) */
static int nimbus_bootload_cmd(struct nimbus *n)
{
	u8 tx[NIMBUS_FRAME_LEN];
	u8 rx[NIMBUS_FRAME_LEN];
	unsigned int i;
	int ret;

	tx[0] = (NIMBUS_BOOTLOAD_WORD >> 8) & 0xff;
	tx[1] = NIMBUS_BOOTLOAD_WORD & 0xff;
	for (i = 2; i < NIMBUS_FRAME_LEN; i += 2) {
		tx[i] = 0x18;
		tx[i + 1] = 0xe1;
	}
	memset(rx, 0, sizeof(rx));
	ret = nimbus_xfer(n, tx, rx, NIMBUS_FRAME_LEN);
	dev_info(&n->spi->dev,
		 "bootload 6593 ret=%d rx %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 ret, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
	return ret;
}

/*
 * sub_3B9D0 dword swizzle into chunk payload: b0 b1 b2 b3 → b1 b0 b3 b2
 * (distinct from the full u32 byte-reverse used at FW+350).
 */
static void nimbus_grape_swizzle32(u8 *dst, const u8 *src, unsigned int len)
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
static void nimbus_bswap32_words(u8 *p, unsigned int len)
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

static u32 nimbus_sum32(const u8 *p, unsigned int len)
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
 * Host/IsyS order is checked BEFORE the RetailOS u32-reverse into win[].
 */
static bool nimbus_cal_looks_ni(const u8 *p)
{
	return p && p[0] == 0x4e && p[1] == 0x49;
}

/*
 * OSOS sub_273A0: copy IsyS[350 : 350+0x200], reverse each u32 in that
 * copy only (do not mutate the 0x560 object).
 */
static int nimbus_prepare_cal_from_isys(struct nimbus *n, const u8 *isys,
					size_t isys_len)
{
	const u8 *raw;
	u32 sum;

	if (isys_len != NIMBUS_ISYS_LEN) {
		dev_err(&n->spi->dev, "IsyS bad size: got=%zu want=0x%x\n",
			isys_len, NIMBUS_ISYS_LEN);
		return -EINVAL;
	}

	memcpy(n->isys, isys, NIMBUS_ISYS_LEN);
	n->have_isys = true;

	raw = n->isys + NIMBUS_FW_HDR_OFF;
	memcpy(n->cal_upload, raw, NIMBUS_FW_HDR_LEN);
	dev_info(&n->spi->dev,
		 "cal +350 raw head %02x %02x %02x %02x%s\n",
		 raw[0], raw[1], raw[2], raw[3],
		 nimbus_cal_looks_ni(raw) ? " (NI family — good)" :
		 " (not NI — suspect vs 4S/IOReg cal)");
	nimbus_bswap32_words(n->cal_upload, NIMBUS_FW_HDR_LEN);
	sum = nimbus_sum32(n->cal_upload, NIMBUS_FW_HDR_LEN);
	dev_info(&n->spi->dev,
		 "Nimbus IsyS cal prepared: off=%u len=0x%x sum32=0x%08x upload_first32=%32ph\n",
		 NIMBUS_FW_HDR_OFF, NIMBUS_FW_HDR_LEN, sum, n->cal_upload);
	if (!sum) {
		dev_err(&n->spi->dev,
			"IsyS +350 window is all zeros — not a usable cal\n");
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
static int nimbus_load_isys_from_a34(struct nimbus *n)
{
	void __iomem *desc_io;
	void __iomem *src_io;
	u32 magic;
	u32 ptr;
	u8 *tmp;
	int ret;

	if (!cal_try_a34)
		return -ENOENT;

	desc_io = ioremap(NIMBUS_A34_ISYS_DESC, 8);
	if (!desc_io)
		return -ENOMEM;

	magic = readl(desc_io);
	ptr = readl(desc_io + 4);
	iounmap(desc_io);

	dev_info(&n->spi->dev,
		 "A34 IsyS descriptor: magic=0x%08x ptr=0x%08x\n",
		 magic, ptr);

	if (magic != NIMBUS_ISYS_MAGIC) {
		dev_warn(&n->spi->dev,
			 "A34 IsyS missing: magic=0x%08x want=0x%08x\n",
			 magic, NIMBUS_ISYS_MAGIC);
		return -ENOENT;
	}
	if (!ptr) {
		dev_warn(&n->spi->dev, "A34 IsyS pointer is NULL\n");
		return -ENOENT;
	}

	src_io = ioremap(ptr, NIMBUS_ISYS_LEN);
	if (!src_io)
		return -ENOMEM;

	tmp = kmalloc(NIMBUS_ISYS_LEN, GFP_KERNEL);
	if (!tmp) {
		iounmap(src_io);
		return -ENOMEM;
	}

	memcpy_fromio(tmp, src_io, NIMBUS_ISYS_LEN);
	iounmap(src_io);

	dev_info(&n->spi->dev,
		 "A34 IsyS read: ptr=0x%08x len=0x%x first32=%32ph calraw_first16=%16ph\n",
		 ptr, NIMBUS_ISYS_LEN, tmp, tmp + NIMBUS_FW_HDR_OFF);

	ret = nimbus_prepare_cal_from_isys(n, tmp, NIMBUS_ISYS_LEN);
	kfree(tmp);
	return ret;
}

/*
 * U-Boot copies the 0x560 IsyS object to reserved DRAM and publishes
 * apple,n31-isys-addr / apple,n31-isys-size on /chosen. That copy is the
 * safe address — never the original A34 pointer.
 */
static int nimbus_load_isys_from_dt(struct nimbus *n)
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

	ret = of_property_read_u32(chosen, "apple,n31-isys-addr", &addr);
	if (ret)
		goto out;

	ret = of_property_read_u32(chosen, "apple,n31-isys-size", &size);
	if (ret)
		goto out;

	dev_info(&n->spi->dev, "DT IsyS: addr=0x%08x size=0x%x\n", addr, size);

	if (!addr || size != NIMBUS_ISYS_LEN) {
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

	dev_info(&n->spi->dev,
		 "DT IsyS read: addr=0x%08x len=0x%x first32=%32ph calraw_first16=%16ph\n",
		 addr, size, tmp, tmp + NIMBUS_FW_HDR_OFF);

	ret = nimbus_prepare_cal_from_isys(n, tmp, size);
	kfree(tmp);

out:
	of_node_put(chosen);
	return ret;
}

static int nimbus_acquire_isys_cal(struct nimbus *n)
{
	int ret;

	if (n->have_cal)
		return 0;

	ret = nimbus_load_isys_from_dt(n);
	if (!ret)
		return 0;

	dev_info(&n->spi->dev, "DT IsyS unavailable: %d; trying A34 live\n",
		 ret);

	ret = nimbus_load_isys_from_a34(n);
	if (!ret)
		return 0;

	dev_err(&n->spi->dev,
		"No IsyS calibration from DT or A34; not registering input\n");
	return ret;
}

/* Optional fmss FTL export — grape firmware only, not IsyS cal. */
static bool (*nimbus_ftl_present_fn)(void);
static int (*nimbus_ftl_read_fn)(u64 logical_sector, void *buf);
static bool nimbus_ftl_inited;

static void nimbus_ftl_init_once(void)
{
	if (nimbus_ftl_inited)
		return;
	nimbus_ftl_inited = true;
	nimbus_ftl_present_fn = symbol_get(fmss_ftl_present);
	nimbus_ftl_read_fn = symbol_get(fmss_ftl_read_sector);
}

static bool nimbus_ftl_ready(void)
{
	nimbus_ftl_init_once();
	return nimbus_ftl_present_fn && nimbus_ftl_read_fn &&
	       nimbus_ftl_present_fn();
}

/*
 * Walk FTL for Apple 8740 / gpfw IMG1. Returns kmalloc'd buffer + size.
 * Caller kfree() on success. This is the ARM app, not IsyS cal.
 */
static u8 *nimbus_try_gpfw_from_ftl(struct device *dev, size_t *out_len)
{
	u8 *sec, *buf = NULL;
	u64 lba, end;
	unsigned int off;
	size_t need, got;
	u32 body_sz;

	if (!fw_prefer_ftl || !nimbus_ftl_ready())
		return NULL;

	sec = kmalloc(NIMBUS_FTL_SECTOR_SIZE, GFP_KERNEL);
	if (!sec)
		return NULL;

	end = min_t(u64, cal_ftl_start + cal_ftl_count, 256ULL);
	for (lba = 0; lba < end; lba++) {
		if (nimbus_ftl_read_fn(lba, sec))
			continue;
		for (off = 0; off + 0x410 <= NIMBUS_FTL_SECTOR_SIZE; off += 4) {
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
						    NIMBUS_FTL_SECTOR_SIZE - off));
			got = min_t(size_t, need, NIMBUS_FTL_SECTOR_SIZE - off);
			while (got < need && lba + 1 < end) {
				lba++;
				if (nimbus_ftl_read_fn(lba, sec)) {
					kfree(buf);
					buf = NULL;
					goto out;
				}
				memcpy(buf + got, sec,
				       min_t(size_t, need - got,
					       NIMBUS_FTL_SECTOR_SIZE));
				got += min_t(size_t, need - got,
					     NIMBUS_FTL_SECTOR_SIZE);
			}
			if (got >= 0x410) {
				dev_info(dev,
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

static int nimbus_acquire_fw(struct device *dev, const u8 **data,
			     size_t *size, const struct firmware **fw_out,
			     u8 **kbuf_out)
{
	size_t flen = 0;
	u8 *ftl;

	*fw_out = NULL;
	*kbuf_out = NULL;
	if (fw_prefer_ftl) {
		ftl = nimbus_try_gpfw_from_ftl(dev, &flen);
		if (ftl) {
			*data = ftl;
			*size = flen;
			*kbuf_out = ftl;
			return 0;
		}
	}
	if (!fw_allow_file)
		return -ENOENT;
	if (request_firmware(fw_out, "apple/grape-nimbus.bin", dev) ||
	    !*fw_out)
		return -ENOENT;
	*data = (*fw_out)->data;
	*size = (*fw_out)->size;
	return 0;
}

static void nimbus_release_fw(const struct firmware *fw, u8 *kbuf)
{
	if (kbuf)
		kfree(kbuf);
	else if (fw)
		release_firmware(fw);
}

/*
 * 2D7A4 payload → controller @ 0x00400200.
 * Cal is the transformed A34 IsyS window only.
 */
static int nimbus_load_cal_window(struct nimbus *n, u8 *win)
{
	int ret;

	ret = nimbus_acquire_isys_cal(n);
	if (ret)
		return ret;
	memcpy(win, n->cal_upload, NIMBUS_FW_HDR_LEN);
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
static unsigned int nimbus_build_upload_frame(u8 *buf, u32 dest,
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
	hdr_sum = nimbus_sum16(buf + 4, 6);
	buf[10] = (hdr_sum >> 8) & 0xff;
	buf[11] = hdr_sum & 0xff;

	nimbus_grape_swizzle32(buf + 12, src, len);

	payload_sum = nimbus_sum32(buf + 12, len);
	buf[12 + len]     = (payload_sum >> 8) & 0xff;
	buf[12 + len + 1] = payload_sum & 0xff;
	buf[12 + len + 2] = (payload_sum >> 24) & 0xff;
	buf[12 + len + 3] = (payload_sum >> 16) & 0xff;

	return len + NIMBUS_HDR_LEN;
}

/* Glass/oracle prefixes from RetailOS 2D640 / 2D7A4 — fail loud if wrong. */
static void nimbus_check_upload_prefix(struct nimbus *n, u32 dest,
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

	if (dest == 0 && len == NIMBUS_CHUNK_MAX && memcmp(tx, fw0, 12)) {
		dev_err(&n->spi->dev,
			"FW_UPLOAD prefix MISMATCH want 18 e1 30 01 07 fc 00 00 00 00 01 03 got %12ph\n",
			tx);
	}
	if (dest == NIMBUS_CAL_DEST && len == NIMBUS_FW_HDR_LEN &&
	    memcmp(tx, cal0, 12)) {
		dev_err(&n->spi->dev,
			"CAL_UPLOAD prefix MISMATCH want 18 e1 30 01 00 80 02 00 00 40 00 c2 got %12ph\n",
			tx);
	}
}

static void nimbus_log_upload_prefix(struct nimbus *n, const char *tag,
				     unsigned int chunk_idx, u32 dest,
				     unsigned int len, const u8 *tx,
				     unsigned int xfer_len, u16 ack, int ack_ret)
{
	dev_info(&n->spi->dev,
		 "NIMBUS %s chunk=%u dest=%08x len=%04x xfer=%u ACK=0x%04x ret=%d\n",
		 tag, chunk_idx, dest, len, xfer_len, ack, ack_ret);
	dev_info(&n->spi->dev,
		 "  tx[0:16] = %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7],
		 tx[8], tx[9], tx[10], tx[11],
		 xfer_len > 12 ? tx[12] : 0, xfer_len > 13 ? tx[13] : 0,
		 xfer_len > 14 ? tx[14] : 0, xfer_len > 15 ? tx[15] : 0);
}

static void nimbus_dump_hbpp_tx(struct nimbus *n, const char *tag,
				const u8 *raw, unsigned int chunk_idx,
				unsigned int dest, unsigned int chunk_len,
				const u8 *tx, unsigned int xfer_len, u16 ack,
				int ack_ret)
{
	unsigned int last_off;

	nimbus_log_upload_prefix(n, tag, chunk_idx, dest, chunk_len,
				 tx, xfer_len, ack, ack_ret);
	if (raw && chunk_len >= 64)
		dev_info(&n->spi->dev, "  raw first64=%32ph %32ph\n",
			 raw, raw + 32);
	else if (raw)
		dev_info(&n->spi->dev, "  raw first%u=%*ph\n",
			 chunk_len, chunk_len, raw);
	if (xfer_len >= 96)
		dev_info(&n->spi->dev,
			 "  tx first96=%32ph %32ph %32ph\n",
			 tx, tx + 32, tx + 64);
	else if (xfer_len > 16)
		dev_info(&n->spi->dev, "  tx first%u=%*ph\n",
			 xfer_len, xfer_len, tx);
	if (xfer_len >= 32) {
		last_off = xfer_len - 32;
		dev_info(&n->spi->dev, "  tx last32=%32ph\n", tx + last_off);
	}
}

static int nimbus_send_chunk_ex(struct nimbus *n, const u8 *data,
				unsigned int dest, unsigned int len,
				unsigned int cs_flags, bool dump,
				const char *tag, unsigned int chunk_idx,
				unsigned int file_off)
{
	u8 *buf;
	unsigned int xfer_len;
	int ret = -EIO, try, ack_ret = -ETIMEDOUT;
	u16 ack = 0;

	if (!len || len > NIMBUS_CHUNK_MAX || (len & 3))
		return -EINVAL;

	xfer_len = len + NIMBUS_HDR_LEN;
	buf = kzalloc(xfer_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (nimbus_build_upload_frame(buf, dest, data, len) != xfer_len) {
		kfree(buf);
		return -EINVAL;
	}
	nimbus_check_upload_prefix(n, dest, len, buf);

	/* Z2 SEND_BLOB: spi_sync keeps CS down for whole HBPP frame. */
	for (try = 0; try < 5; try++) {
		if (chunk_spi)
			ret = nimbus_xfer(n, buf, NULL, xfer_len);
		else if (n->blob16)
			ret = nimbus_burst_u16_ex(n, buf, NULL,
						  xfer_len, cs_flags);
		else
			ret = nimbus_burst_ex(n, buf, NULL, xfer_len, cs_flags);
		if (ret)
			continue;
		/* 1A A1 → 2 bytes → rev16; expect 0x4BC1 */
		ack_ret = nimbus_wait_ack(n, NIMBUS_ACK_CHUNK, 8);
		if (ack_ret == 0) {
			ack = NIMBUS_ACK_CHUNK;
			if (dump)
				nimbus_dump_hbpp_tx(n, tag, data, chunk_idx,
						    dest, len, buf, xfer_len,
						    ack, ack_ret);
			else if (!chunk_idx)
				nimbus_log_upload_prefix(n, tag, chunk_idx,
							 dest, len, buf,
							 xfer_len, ack,
							 ack_ret);
			kfree(buf);
			return 0;
		}
		if (n->blob16 && try == 0) {
			n->blob16 = false;
			dev_info(&n->spi->dev,
				 "16-bit DATA no 4BC1 — falling back to 8-bit PIO\n");
		}
	}
	if (dump || !chunk_idx)
		nimbus_dump_hbpp_tx(n, tag, data, chunk_idx, dest, len, buf,
				    xfer_len, ack, ack_ret);
	kfree(buf);
	return -EIO;
}

static int nimbus_send_chunk(struct nimbus *n, const u8 *data,
			     unsigned int dest, unsigned int len, bool dump,
			     unsigned int file_off, const char *tag,
			     unsigned int chunk_idx)
{
	return nimbus_send_chunk_ex(n, data, dest, len,
				    NIMBUS_CS_BEGIN | NIMBUS_CS_END, dump,
				    tag, chunk_idx, file_off);
}

static int nimbus_send_blob(struct nimbus *n, const u8 *data, unsigned int len,
			    unsigned int dest_off)
{
	unsigned int off = 0;
	unsigned int chunk_idx = 0;
	u8 pad[4];
	const char *tag;
	bool is_cal = (dest_off == NIMBUS_CAL_DEST);

	tag = is_cal ? "CAL_UPLOAD" : "FW_UPLOAD";

	while (off < len) {
		unsigned int chunk = min_t(unsigned int, len - off, NIMBUS_CHUNK_MAX);
		int ret;
		bool last, dump;

		/* RetailOS always transfers whole words */
		if (chunk & 3)
			chunk &= ~3u;
		if (!chunk) {
			memset(pad, 0, sizeof(pad));
			memcpy(pad, data + off, len - off);
			return nimbus_send_chunk(n, pad, dest_off + off, 4,
						 true, off, tag, chunk_idx);
		}
		last = (off + chunk >= len);
		/* Always dump first + last FW chunk and the sole cal chunk. */
		dump = (off == 0) || last || is_cal;
		ret = nimbus_send_chunk(n, data + off, dest_off + off, chunk,
					dump, off, tag, chunk_idx);
		if (ret)
			return ret;
		off += chunk;
		chunk_idx++;
	}
	return 0;
}

/* sub_34AD0(a1,a2,a3) — TX 1E 33 + 12-byte pack + sum16, expect ACK 0x4AD1 */
static int nimbus_cmd_34ad0(struct nimbus *n, u32 a1, u32 a2, u32 a3)
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
	csum = nimbus_sum16(body, 12);
	memcpy(tx + 2, body, 12);
	tx[14] = (csum >> 8) & 0xff;
	tx[15] = csum & 0xff;

	/* 34AD0: 40F770 16↔16 then 3D5706 == 0x4AD1 */
	ret = nimbus_xfer(n, tx, rx, 16);
	if (ret)
		return ret;
	return nimbus_wait_ack(n, NIMBUS_ACK_34AD0, 8);
}

/* sub_2D5B0 post-download */
static int nimbus_post_download(struct nimbus *n)
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

		ret = nimbus_cmd_34ad0(n, pokes[i].a1, pokes[i].a2, pokes[i].a3);
		dev_info(&n->spi->dev, "34AD0[%d] %d\n", i, ret);
		if (ret)
			return ret;
		/* 4AD1 = write ACK only; verify with RDREG while still in HBPP. */
		if (nimbus_rdreg(n, pokes[i].a1, &rb) == 0)
			dev_info(&n->spi->dev,
				 "34AD0[%d] RDREG 0x%08x -> 0x%08x (wrote %u)\n",
				 i, pokes[i].a1, rb, pokes[i].a2);
	}

	put_unaligned_le16(NIMBUS_POST_POKE, tx);
	ret = nimbus_xfer(n, tx, rx, 2);
	if (ret)
		return ret;
	msleep(65);
	/* 2D5B0: 3D5706 success only — does not require 0x4BC1 */
	if (nimbus_status_poll(n, &st) == 0) {
		dev_info(&n->spi->dev, "post-poke status 0x%04x\n", st);
		n->requestcal_done = true;
		return 0;
	}
	return -EIO;
}

/* sub_2D54C — 12↔12: 1D 53 + two LE u32 + sum16 */
static int nimbus_cmd_2d54c_raw(struct nimbus *n, u32 word0, u32 word1)
{
	u8 tx[12] = { 0x1d, 0x53 };
	u8 rx[12] = { 0 };
	u16 csum;
	u32 saved_setup = 0;
	int ret;

	put_unaligned_le32(word0, tx + 2);
	put_unaligned_le32(word1, tx + 6);
	csum = nimbus_sum16(tx + 2, 8);
	tx[10] = (csum >> 8) & 0xff;
	tx[11] = csum & 0xff;
	if (n->spi2) {
		nimbus_spi2_fifo_flush(n);
		if (go_spi_setup > 0) {
			saved_setup = readl(n->spi2 + SPI2_SETUP);
			writel((u32)go_spi_setup, n->spi2 + SPI2_SETUP);
			dev_info(&n->spi->dev, "2D54C GO SETUP 0x%x (was 0x%x)\n",
				 go_spi_setup, saved_setup);
		}
	}
	if (go_xfer == 2)
		ret = nimbus_xfer(n, tx, rx, 12);
	else if (go_xfer == 1)
		ret = nimbus_burst_u16(n, tx, rx, 12);
	else
		ret = nimbus_burst(n, tx, rx, 12);
	if (saved_setup)
		writel(saved_setup, n->spi2 + SPI2_SETUP);
	dev_info(&n->spi->dev,
		 "2D54C %08x %08x ret=%d xfer=%d rx %02x %02x %02x %02x %02x %02x\n",
		 word0, word1, ret, go_xfer, rx[0], rx[1], rx[2], rx[3],
		 rx[4], rx[5]);
	return ret;
}

static void __maybe_unused nimbus_drain(struct nimbus *n, unsigned int bytes)
{
	u8 tx[NIMBUS_FRAME_LEN] = { 0 };
	u8 rx[NIMBUS_FRAME_LEN] = { 0 };
	unsigned int nxf = bytes < NIMBUS_FRAME_LEN ? bytes : NIMBUS_FRAME_LEN;

	/* Optional post-fail diagnostics only — never call on EXEC path. */
	nimbus_burst(n, tx, rx, nxf);
}

static void nimbus_pre_exec_verify(struct nimbus *n)
{
	static const u32 addrs[] = {
		0x00000000, 0x00000004, 0x00000008, 0x00000020,
		0x00400200, 0x00400204, 0x004003fc,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(addrs); i++) {
		u32 v = 0;

		if (nimbus_rdreg(n, addrs[i], &v) == 0)
			dev_info(&n->spi->dev, "pre-EXEC RDREG 0x%08x=0x%08x\n",
				 addrs[i], v);
	}
}

/*
 * sub_2D54C — one-shot EXEC packet only.
 * Do NOT poll 1A A1 / drain after EXEC: that keeps speaking HBPP across the
 * bootloader→runtime boundary. Success is proven only by 182590 ping csum.
 */
static int nimbus_cmd_2d54c(struct nimbus *n)
{
	int ret;

	nimbus_pre_exec_verify(n);
	ret = nimbus_cmd_2d54c_raw(n, exec_addr, exec_word1);
	if (!ret)
		n->exec_sent = true;
	return ret;
}

/*
 * 273A0: 2D640(204E0 ARM, hdr+0x0c) → 2D7A4(BSS+350 @ 0x400200) → 2D5B0.
 */
static int nimbus_probe_z2_eb(struct nimbus *n)
{
	u8 tx[NIMBUS_FRAME_LEN] = { 0 };
	u8 rx[NIMBUS_FRAME_LEN] = { 0 };
	int ret;

	tx[0] = 0xeb;
	tx[1] = 0x01;
	put_unaligned_le16(0xeb + 1, tx + 14);
	ret = nimbus_burst16(n, tx, rx);
	dev_info(&n->spi->dev,
		 "z2-EB ret=%d rx %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 ret, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
	return (ret == 0 && rx[0] == 0xe1) ? 0 : -EIO;
}

static int nimbus_probe_ea16(struct nimbus *n)
{
	u8 tx[NIMBUS_FRAME_LEN] = { 0 };
	u8 rx[NIMBUS_FRAME_LEN] = { 0 };
	u16 csum;
	int ret;

	tx[0] = NIMBUS_MAGIC;
	tx[1] = 0x01;
	tx[2] = 0x01;
	csum = nimbus_sum16(tx, 14);
	put_unaligned_le16(csum, tx + 14);
	ret = nimbus_burst16(n, tx, rx);
	dev_info(&n->spi->dev,
		 "EA16 ret=%d rx %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 ret, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
	return (ret == 0 && rx[0] == NIMBUS_MAGIC) ? 0 : -EIO;
}

static int nimbus_probe_ping16(struct nimbus *n)
{
	u8 tx[NIMBUS_FRAME_LEN] = { 0 };
	u8 rx[NIMBUS_FRAME_LEN] = { 0 };
	u16 csum;
	int ret;
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = NIMBUS_FRAME_LEN,
		.bits_per_word = 16,
	};
	struct spi_message m;

	put_unaligned_le32(NIMBUS_PING_TYPE, tx);
	csum = nimbus_sum16(tx, 14);
	put_unaligned_le16(csum, tx + 14);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(n->spi, &m);
	dev_info(&n->spi->dev,
		 "ping16 ret=%d rx %02x %02x %02x %02x %02x %02x %02x %02x csum=%d\n",
		 ret, rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7],
		 nimbus_sum16(rx, 14) == get_unaligned_le16(rx + 14));
	return ret;
}

#define S5L8740_AES_PHYS	0x38c00000UL

/*
 * Touch FW GID decrypt (OSOS sub_422FFA / sub_204E0):
 *   MMIO @ 0x38C00000 AES, keysel=1 (GID), CBC IV=0, CFG=0xE|enc.
 *   26CCC verifies with 16-byte encrypt; 204E0 decrypts full ARM @ +0x400
 *   for 8740 rev 3 only. grape-nimbus.bin on DFU is usually pre-decrypted-cut.
 * force_gid=1 tries 422FFA even when loading plaintext blob (bring-up).
 */
static int nimbus_422ffa_mmio(struct device *dev, u8 *buf, unsigned int len,
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

static int nimbus_gid_crypt(struct device *dev, u8 *buf, unsigned int len,
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

static bool nimbus_looks_like_arm(const u8 *p, size_t n)
{
	return n >= 4 && p[0] == 0x18 && p[1] == 0xf0 &&
	       p[2] == 0x9f && p[3] == 0xe5;
}

/*
 * v6 RE (2026-08-25): post-GID plaintext may already be a preconstructed
 * HBPP DATA object (18 E1 30 01 …) — Corellium GEN_1 sends Constructed
 * Firmware unchanged. Detect that and SPI-send as-is (ACK 0x4BC1).
 */
static bool nimbus_looks_like_hbpp_data(const u8 *p, size_t n)
{
	if (n >= 4 && p[0] == 0x18 && p[1] == 0xe1 &&
	    p[2] == 0x30 && p[3] == 0x01)
		return true;
	if (n >= 2 && p[0] == 0x30 && p[1] == 0x01)
		return true;
	return false;
}

/**
 * nimbus_send_preconstructed_hbpp - SPI the whole HBPP frame, expect 0x4BC1
 * (DATA ACK). Do not re-wrap or swizzle — bytes are already HBPP.
 */
static int nimbus_send_preconstructed_hbpp(struct nimbus *n, const u8 *data,
					   size_t len)
{
	int try, ret;

	if (len < 16)
		return -EINVAL;

	for (try = 0; try < 5; try++) {
		if (chunk_spi)
			ret = nimbus_xfer(n, data, NULL, len);
		else if (n->blob16)
			ret = nimbus_burst_u16_ex(n, data, NULL, len, 0);
		else
			ret = nimbus_burst_ex(n, data, NULL, len, 0);
		if (ret)
			continue;
		if (nimbus_wait_ack(n, NIMBUS_ACK_CHUNK, 8) == 0) {
			dev_info(&n->spi->dev,
				 "preconstructed HBPP %zuB ACK 0x4BC1 try=%d\n",
				 len, try);
			return 0;
		}
		if (n->blob16 && try == 0) {
			n->blob16 = false;
			dev_info(&n->spi->dev,
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
static size_t nimbus_official_arm_len(const u8 *body, size_t len)
{
	size_t i;

	if (len <= NIMBUS_ARM_OFFICIAL || !nimbus_looks_like_arm(body, len))
		return len;
	for (i = NIMBUS_ARM_OFFICIAL; i < len; i++) {
		if (body[i] != 0x53 && body[i] != 0x43)
			return len;
	}
	return NIMBUS_ARM_OFFICIAL;
}

static int nimbus_download_fw(struct nimbus *n, const u8 *data, size_t size,
			      bool arm_at_zero)
{
	u8 *dec = NULL;
	int ret;
	const u8 *body = data;
	size_t body_len = size;
	bool apple_hdr = nimbus_fw_has_8740_hdr(data, size);

	(void)arm_at_zero;
	nimbus_fwfile_classify(n, data, size);

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
			dev_info(&n->spi->dev,
				 "204E0 ARM-at-0 %zuB dest 0 (rev=%u hdr+0x0c=0x%x file=%zu)\n",
				 body_len, rev, get_unaligned_le32(data + 0x0c),
				 size);
			if (force_gid && size >= 0x400 + 16) {
				u8 *try = kmemdup(data + 0x400, min_t(size_t, body_len, size - 0x400),
						  GFP_KERNEL);
				if (try && nimbus_422ffa_mmio(&n->spi->dev, try,
							      round_up(min_t(size_t, body_len, size - 0x400) & ~15u, 16),
							      false) == 0 &&
				    nimbus_looks_like_arm(try, min_t(size_t, 16, body_len))) {
					dev_info(&n->spi->dev, "force_gid 422FFA ARM ok\n");
					body = try;
					body_len = min_t(size_t, body_len, size - 0x400);
					dec = try;
				} else {
					kfree(try);
				}
			}
			goto send;
		}
		dev_info(&n->spi->dev,
			 "204E0 ARM-at-0 %zu bytes rev=%u (hdr+0x0c=0x%x file=%zu)\n",
			 body_len, rev, get_unaligned_le32(data + 0x0c), size);

		if (rev == 3) {
			u8 *probe;
			bool verified = false;

			probe = kmemdup(data, 16, GFP_KERNEL);
			if (!probe)
				return -ENOMEM;
			if (nimbus_gid_crypt(&n->spi->dev, probe, 16, true) == 0 &&
			    !memcmp(probe, data + 0x40, 16)) {
				verified = true;
				dev_info(&n->spi->dev, "26CCC GID verify OK\n");
			} else {
				memcpy(probe, data, 16);
				if (nimbus_422ffa_mmio(&n->spi->dev, probe, 16,
						       true) == 0 &&
				    !memcmp(probe, data + 0x40, 16)) {
					verified = true;
					dev_info(&n->spi->dev,
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
			ret = nimbus_422ffa_mmio(&n->spi->dev, dec, body_len,
						 false);
			if (ret || !nimbus_looks_like_arm(dec, body_len)) {
				memcpy(dec, body, body_len);
				ret = nimbus_gid_crypt(&n->spi->dev, dec,
						       body_len, false);
			}
			dev_info(&n->spi->dev,
				 "204E0 GID decrypt ret=%d arm=%d head %02x %02x %02x %02x ver=%d\n",
				 ret, nimbus_looks_like_arm(dec, body_len),
				 dec[0], dec[1], dec[2], dec[3], verified);
			if (!ret && body_len > 0xd210)
				dev_info(&n->spi->dev,
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
		dev_info(&n->spi->dev, "Grape FW download %zu bytes (no 8740)\n",
			 size);
	}

send:
	/*
	 * v6: if body is already 18 E1 30 01… (post-GID constructed FW),
	 * send once and run post-download. Do not re-packetize ARM.
	 */
	if (nimbus_looks_like_hbpp_data(body, body_len)) {
		dev_info(&n->spi->dev,
			 "preconstructed HBPP DATA %zuB — direct SPI (no ARM wrap)\n",
			 body_len);
		ret = nimbus_send_preconstructed_hbpp(n, body, body_len);
		if (!ret) {
				ret = nimbus_post_download(n);
				if (!ret)
					ret = nimbus_cmd_2d54c(n);
			}
		kfree(dec);
		return ret;
	}

	{
		size_t official = nimbus_official_arm_len(body, body_len);
		size_t dl_len = body_len;
		const u8 *dl_body = body;
		u8 *z2_prep = NULL;
		u8 *pad_buf = NULL;
		u8 *win;
		int try, cal;

		if (official < body_len) {
			dev_info(&n->spi->dev,
				 "cap ARM %zu -> %zu (204E0 +0x0c; strip S/C fill)\n",
				 body_len, official);
			body_len = official;
			dl_len = body_len;
		}

		z2_prep = nimbus_maybe_prepend_z2_hdr(n, body, body_len, &dl_len);
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
			dev_info(&n->spi->dev, "FW padded to %zu (4-byte align)\n",
				 dl_len);
		}
		nimbus_fw_audit(n, dl_body, dl_len, "2D640");

		win = kzalloc(NIMBUS_FW_HDR_LEN, GFP_KERNEL);
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
		cal = nimbus_load_cal_window(n, win);
		if (cal < 0) {
			dev_err(&n->spi->dev, "2D7A4 aborted: no device cal\n");
			kfree(win);
			kfree(z2_prep);
			kfree(pad_buf);
			kfree(dec);
			return cal;
		}

		dev_info(&n->spi->dev,
			 "2D640 ARM dest_base=0x%08x EXEC=0x%08x cal=0x%08x\n",
			 fw_dest, exec_addr, NIMBUS_CAL_DEST);
		if (fw_dest == 0)
			dev_info(&n->spi->dev,
				 "expect FW prefix: 18 e1 30 01 07 fc 00 00 00 00 01 03 (len=0x1ff0)\n");
		else
			dev_warn(&n->spi->dev,
				 "fw_dest override 0x%08x — OSOS uses 0 (A/B only)\n",
				 fw_dest);
		dev_info(&n->spi->dev,
			 "expect CAL prefix: 18 e1 30 01 00 80 02 00 00 40 00 c2\n");

		/* 20E94: 273A0 up to 3 times, no 1A878 between. */
		for (try = 0; try < 3; try++) {
			ret = nimbus_send_blob(n, dl_body, dl_len, fw_dest);
			if (ret) {
				dev_err(&n->spi->dev,
					"2D640 ARM@0x%08x try %d: %d\n",
					fw_dest, try, ret);
				continue;
			}
			n->fw_uploaded = true;
			nimbus_fw_readback(n, "post-2D640");
			ret = nimbus_send_blob(n, win, NIMBUS_FW_HDR_LEN,
					       NIMBUS_CAL_DEST);
			if (ret) {
				dev_err(&n->spi->dev,
					"2D7A4 cal@0x%08x try %d: %d\n",
					NIMBUS_CAL_DEST, try, ret);
				continue;
			}
			n->cal_uploaded = true;
			dev_info(&n->spi->dev,
				 "2D7A4 512B cal @0x%08x ACK (transport only)\n",
				 NIMBUS_CAL_DEST);
			nimbus_cal_readback(n, win);
			ret = nimbus_post_download(n);
			if (ret) {
				dev_warn(&n->spi->dev,
					 "2D5B0 try %d: %d\n", try, ret);
				continue;
			}
			ret = nimbus_cmd_2d54c(n);
			if (!ret) {
				dev_info(&n->spi->dev,
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

static int nimbus_ping(struct nimbus *n, u16 *status_out)
{
	u8 tx[NIMBUS_FRAME_LEN] = { 0 };
	u8 rx[NIMBUS_FRAME_LEN] = { 0 };
	u16 csum, rx_csum;
	int ret, tries;

	put_unaligned_le32(NIMBUS_PING_TYPE, tx);
	csum = nimbus_sum16(tx, 14);
	put_unaligned_le16(csum, tx + 14);

	/* 182590: up to 5 retries, sleep 1 between. After EXEC, 16-bit
	 * pairs match the app SPI width; 8-bit PIO is bootloader-only.
	 */
	for (tries = 0; tries < 6; tries++) {
		if (n->exec_sent && go_xfer)
			ret = nimbus_burst_u16(n, tx, rx, NIMBUS_FRAME_LEN);
		else
			ret = nimbus_burst16(n, tx, rx);
		if (ret)
			return ret;

		rx_csum = get_unaligned_le16(rx + 14);
		if (!rx_csum && !nimbus_sum16(rx, 14)) {
			dev_warn(&n->spi->dev, "ping rx all-zero (MISO dead)\n");
			return -EIO;
		}
		if (nimbus_sum16(rx, 14) == rx_csum)
			break;
		/* One dump per call; MultitouchTask rate-limits via ping_fails. */
		if (tries == 0 && n->ping_fails == 0)
			dev_warn(&n->spi->dev,
				 "ping csum fail rx %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				 rx[0], rx[1], rx[2], rx[3], rx[4],
				 rx[5], rx[6], rx[7], rx[8], rx[9],
				 rx[10], rx[11], rx[12], rx[13],
				 rx[14], rx[15]);
		if (tries == 5)
			return -EIO;
		msleep(1);
	}

	if (status_out)
		*status_out = get_unaligned_le16(rx + 1);
	return 0;
}

static void nimbus_map_coords(s16 rawx, s16 rawy, int *x, int *y)
{
	int xx = (NIMBUS_ABS_X_MAX * ((int)rawx + 75)) / NIMBUS_SCALE_X_DIV;
	int yy = (NIMBUS_ABS_Y_MAX * ((int)rawy + 75)) / NIMBUS_SCALE_Y_DIV;

	if (xx < 0)
		xx = 0;
	if (xx > NIMBUS_ABS_X_MAX)
		xx = NIMBUS_ABS_X_MAX;
	yy = NIMBUS_ABS_Y_MAX - yy;
	if (yy < 0)
		yy = 0;
	if (yy > NIMBUS_ABS_Y_MAX)
		yy = NIMBUS_ABS_Y_MAX;
	*x = xx;
	*y = yy;
}

static void nimbus_parse_D(struct nimbus *n, const u8 *payload, unsigned int len)
{
	const u8 *rec;
	u8 count, stride;
	unsigned int off;
	int i;

	if (len < 18 || payload[0] != 0x44)
		return;

	off = payload[2];
	count = payload[16];
	stride = payload[17];
	if (!stride || off >= len)
		return;
	if (count > NIMBUS_SLOTS)
		count = NIMBUS_SLOTS;

	rec = payload + off;
	for (i = 0; i < count; i++) {
		s16 rawx, rawy;
		int x, y;
		u8 tip;

		if (rec + stride > payload + len)
			break;
		rawx = (s16)get_unaligned_le16(rec + 4);
		rawy = (s16)get_unaligned_le16(rec + 6);
		tip = rec[1];
		nimbus_map_coords(rawx, rawy, &x, &y);

		input_mt_slot(n->input, i);
		input_mt_report_slot_state(n->input, MT_TOOL_FINGER, tip != 0);
		if (tip) {
			input_report_abs(n->input, ABS_MT_POSITION_X, x);
			input_report_abs(n->input, ABS_MT_POSITION_Y, y);
			/* Visible proof on quiet console (fbcon won't scroll from MT) */
			pr_warn_ratelimited("nimbus touch slot%d tip=%u raw=%d,%d -> %d,%d\n",
					    i, tip, rawx, rawy, x, y);
		}
		rec += stride;
	}
	input_mt_sync_frame(n->input);
	input_sync(n->input);
}

static int nimbus_read_reports(struct nimbus *n, u16 ping_st)
{
	u8 *tx, *rx;
	u16 csum;
	unsigned int len = ping_st + 5;
	int ret;

	/* 17E404(ping_status+5), cap 512 */
	if (len < NIMBUS_FRAME_LEN)
		len = NIMBUS_FRAME_LEN;
	if (len > NIMBUS_READ_MAX)
		len = NIMBUS_READ_MAX;

	tx = kzalloc(NIMBUS_READ_MAX, GFP_KERNEL);
	rx = kzalloc(NIMBUS_READ_MAX, GFP_KERNEL);
	if (!tx || !rx) {
		ret = -ENOMEM;
		goto out;
	}

	tx[0] = NIMBUS_MAGIC;
	tx[1] = 0x01;
	tx[2] = 0x01;
	csum = nimbus_sum16(tx, 14);
	/* 17E404 writes sum16(tx,14) at TX+(len-2) */
	put_unaligned_le16(csum, tx + len - 2);

	ret = nimbus_xfer(n, tx, rx, len);
	if (ret)
		goto out;

	if (rx[0] != NIMBUS_MAGIC) {
		dev_info_once(&n->spi->dev,
			      "read magic fail rx %02x %02x %02x %02x %02x %02x %02x %02x\n",
			      rx[0], rx[1], rx[2], rx[3], rx[4], rx[5],
			      rx[6], rx[7]);
		ret = -EIO;
		goto out;
	}

	if (rx[2] && rx[2] != 2) {
		unsigned int plen = rx[2];

		if (plen >= 2 && (5 + (plen - 2)) <= len)
			nimbus_parse_D(n, rx + 5, plen - 2);
	} else if (len > 5 && rx[5] == 0x44) {
		nimbus_parse_D(n, rx + 5, len - 5);
	}
	ret = 0;
out:
	kfree(tx);
	kfree(rx);
	return ret;
}

static void nimbus_dump_pad(struct nimbus *n, unsigned int gpio, const char *name)
{
	void __iomem *b;
	u32 pin, pcon, din, dir;

	if (!nimbus_verbose || !n->gpio_base)
		return;
	b = n->gpio_base + 32 * (gpio >> 3);
	pin = gpio & 7;
	pcon = readl(b);
	din = readl(b + 0x04);
	dir = readl(b + 0x14);
	dev_info(&n->spi->dev,
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
static void nimbus_gpio_por_reset(struct nimbus *n)
{
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_RST, 1, 0);
	msleep(reset_hold_ms);
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_RST, 1, 1);
	msleep(reset_release_ms);
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_RST, 1, 0);
	msleep(5);
	dev_info(&n->spi->dev, "extra POR RST %dms low / %dms high\n",
		 reset_hold_ms, reset_release_ms);
}

/*
 * 1A5AC GPIO half (before 20848):
 *   2075A(1) sleep5 → 20766(1) sleep15 → 20690(1) sleep5 → 11B70
 */
static void nimbus_gpio_bringup(struct nimbus *n)
{
	int rail;

	/* GPIOCMD only — gpiod set_value fights polarity on RST. */
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_RST, 1, 0);
	msleep(5);
	/*
	 * sub_20766(1): 439B00(1) rail first, 66A8(8)+sleep 3, then
	 * GPIOCMD EN mode 0 (not output-high).
	 */
	rail = d1830_nimbus_rail(true);
	if (rail)
		dev_warn(&n->spi->dev, "20766 PMIC rail: %d\n", rail);
	else {
		/* 66A8(8) is 345D40 thunk (0x220002B2), not an 8ms sleep */
		msleep(3);
	}
	/* 20766(1): EN mode 0, val 0. Do not cmd-15 the latch. */
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_EN, 0, 0);
	nimbus_dump_pad(n, NIMBUS_GPIO_EN, "en-mode0");
	msleep(15);
	/* 20690(1) after EN — OSOS order; required after 1A878 unmux */
	nimbus_spi2_pinmux(n, true);
	msleep(5);
	/* 1A5AC: 11B70 after remux, still in reset, before 20848 */
	nimbus_spi2_11b70(n);
	nimbus_dump_pad(n, NIMBUS_GPIO_EN, "en");
	nimbus_dump_pad(n, NIMBUS_GPIO_RST, "rst");
	nimbus_dump_pad(n, NIMBUS_GPIO_IRQ, "irq");
	nimbus_dump_pad(n, 87, "spi2-87");
	nimbus_dump_pad(n, 88, "spi2-88");
	nimbus_dump_pad(n, 89, "spi2-89");
	nimbus_dump_pad(n, 90, "spi2-90");
}

/* 1A5AC: 2075A(0) then sleep reset_release_ms (default 30). */
static void nimbus_gpio_release_reset(struct nimbus *n)
{
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_RST, 1, 1);
	msleep(reset_release_ms);
	nimbus_dump_pad(n, NIMBUS_GPIO_RST, "rst-rel");
}

/* sub_20490(1) — GPIOCMD input + EIC enable for GPIO 38 */
static void nimbus_irq_enable(struct nimbus *n)
{
	nimbus_gpiocmd_mode(n, NIMBUS_GPIO_IRQ, 0, 0);
	/* RetailOS: level, active-low → VIC EXT1 */
	if (s5l8740_eic_enable_gpio(NIMBUS_GPIO_IRQ, IRQ_TYPE_LEVEL_LOW) == 0)
		dev_info(&n->spi->dev, "EIC enabled GPIO%d level-low\n",
			 NIMBUS_GPIO_IRQ);
}

static int nimbus_1a5ac_and_download(struct nimbus *n, const u8 *data,
				     size_t size, const char *tag)
{
	int err;

	/* Dump path has no pre-1A5AC POR; gate for glass A/B only. */
	if (extra_por_pulse)
		nimbus_gpio_por_reset(n);
	nimbus_gpio_bringup(n);
	err = nimbus_bootload_cmd(n);
	if (err)
		dev_warn(&n->spi->dev, "bootload %s: %d\n", tag, err);
	msleep(15); /* 1A5AC: after 20848, before 2075A(0) */
	nimbus_gpio_release_reset(n);
	if (skip_download) {
		dev_info(&n->spi->dev, "skip_download — no 2D640\n");
		return 0;
	}
	/* 20E94: 26494 then 273A0; settle already done in release (30ms). */
	if (nimbus_probe_26494(n, tag)) {
		dev_warn(&n->spi->dev, "26494 %s failed\n", tag);
		return -EIO;
	}
	err = nimbus_download_fw(n, data, size, false);
	n->fw_tried = true;
	return err;
}

/*
 * 1703E8: 10 failed pings → 13A20(0) then 13A20(1). Cap recycles so a
 * stuck bootloader does not spam forever (GO ACKs but app never runs).
 */
#define NIMBUS_RECYCLE_MAX	3

static void nimbus_park(struct nimbus *n, const char *why)
{
	if (n->parked)
		return;
	n->parked = true;
	n->stopped = true;
	nimbus_verbose = false;
	dev_warn(&n->spi->dev,
		 "nimbus parked (%s) — rmmod/insmod to retry\n", why);
	nimbus_power_down(n);
}

static void nimbus_recycle(struct nimbus *n)
{
	const struct firmware *fw = NULL;
	const u8 *data;
	u8 *kbuf = NULL;
	size_t size;
	u16 st = 0;

	if (n->parked)
		return;
	if (n->recycle_count >= NIMBUS_RECYCLE_MAX) {
		nimbus_park(n, "recycle budget");
		return;
	}
	n->recycle_count++;
	dev_info(&n->spi->dev,
		 "1703E8 10 ping fails — 13A20 recycle %u/%u\n",
		 n->recycle_count, NIMBUS_RECYCLE_MAX);
	nimbus_power_down(n);
	n->fw_loaded = false;
	n->fw_tried = false;
	n->spi_ok = false;
	n->ping_fails = 0;
	msleep(50);
	if (nimbus_acquire_fw(&n->spi->dev, &data, &size, &fw, &kbuf)) {
		dev_warn(&n->spi->dev, "recycle: no FW (FTL or file)\n");
		nimbus_park(n, "no firmware");
		return;
	}
	nimbus_1a5ac_and_download(n, data, size, "recycle");
	nimbus_release_fw(fw, kbuf);
	msleep(2);
	if (!nimbus_ping(n, &st)) {
		n->spi_ok = true;
		dev_info(&n->spi->dev, "recycle ping ok, status=0x%04x\n", st);
	} else if (n->recycle_count >= NIMBUS_RECYCLE_MAX) {
		nimbus_park(n, "still bootloader after GO");
	}
}

static void nimbus_service(struct nimbus *n)
{
	u16 st = 0;

	if (n->parked)
		return;
	/* 188FFC: ping 490 (5 tries) then 17E404. Never 1A A1 after FW. */
	if (nimbus_ping(n, &st) == 0) {
		n->spi_ok = true;
		n->ping_fails = 0;
		if (st)
			nimbus_read_reports(n, st);
		return;
	}
	n->ping_fails++;
	if (nimbus_verbose && (n->ping_fails <= 3 || n->ping_fails == 10))
		dev_info(&n->spi->dev,
			 "188FFC ping still fail (%u) attn=%d\n",
			 n->ping_fails,
			 n->attn ? gpiod_get_value_cansleep(n->attn) : -1);
}

static irqreturn_t nimbus_irq_thread(int irq, void *data)
{
	struct nimbus *n = data;

	mutex_lock(&n->lock);
	if (!n->stopped)
		nimbus_service(n);
	mutex_unlock(&n->lock);
	return IRQ_HANDLED;
}

static void nimbus_try_firmware(struct nimbus *n)
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

		if (nimbus_ping(n, &st))
			return;
		n->spi_ok = true;
	}

	if (nimbus_acquire_fw(&n->spi->dev, &data, &size, &fw, &kbuf))
		return;

	n->fw_tried = true;
	nimbus_download_fw(n, data, size, false);
	nimbus_release_fw(fw, kbuf);
}

static int nimbus_poll_thread(void *data)
{
	struct nimbus *n = data;
	int wait;
	int fail_backoff_ms = 50;

	for (wait = 0; wait < 30 && !n->fw_loaded && !n->fw_tried &&
	     !kthread_should_stop(); wait++) {
		mutex_lock(&n->lock);
		nimbus_try_firmware(n);
		mutex_unlock(&n->lock);
		if (!n->fw_loaded && !n->fw_tried)
			msleep(1000);
	}
	if (!n->fw_loaded && !n->fw_tried) {
		n->fw_tried = true;
		dev_info(&n->spi->dev,
			 "no apple/grape-nimbus.bin — bootload+ping only\n");
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
		if (do_poll) {
			nimbus_service(n);
			if (n->ping_fails >= 10)
				nimbus_recycle(n);
			fail_backoff_ms = n->spi_ok ? 50 :
				min(fail_backoff_ms * 2, 2000);
		}
		mutex_unlock(&n->lock);

		if (!do_poll)
			msleep(20);
		else if (!n->spi_ok)
			msleep(fail_backoff_ms);
		else
			msleep(50);
	}
	return 0;
}

static ssize_t isys_blob_read(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *attr, char *buf,
			      loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct nimbus *n = dev_get_drvdata(dev);

	if (!n || !n->have_isys)
		return -ENODATA;
	if (off >= NIMBUS_ISYS_LEN)
		return 0;
	if (off + count > NIMBUS_ISYS_LEN)
		count = NIMBUS_ISYS_LEN - off;
	memcpy(buf, n->isys + off, count);
	return count;
}

static struct bin_attribute isys_blob_attr = {
	.attr = {
		.name = "isys_blob",
		.mode = 0444,
	},
	.size = NIMBUS_ISYS_LEN,
	.read = isys_blob_read,
};

static void nimbus_isys_sysfs_remove(struct nimbus *n)
{
	if (!n || !n->isys_sysfs)
		return;
	sysfs_remove_bin_file(&n->spi->dev.kobj, &isys_blob_attr);
	n->isys_sysfs = false;
}

static int nimbus_probe(struct spi_device *spi)
{
	struct nimbus *n;
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
	n->blob16 = false;
	nimbus_verbose = !quiet;
	mutex_init(&n->lock);
	spi_set_drvdata(spi, n);

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

	err = nimbus_acquire_isys_cal(n);
	if (err)
		return err;
	if (!sysfs_create_bin_file(&spi->dev.kobj, &isys_blob_attr))
		n->isys_sysfs = true;
	else
		dev_warn(&spi->dev, "isys_blob sysfs failed\n");

	data = NULL;
	size = 0;
	err = nimbus_acquire_fw(&spi->dev, &data, &size, &fw, &kbuf);
	if (err)
		dev_warn(&spi->dev,
			 "no grape firmware (%d) — A34 IsyS is present but 2D640 cannot run\n",
			 err);

	if (!data || !size) {
		nimbus_park(n, "no grape firmware");
	} else {
		int attempt;

		/*
		 * 13A20(1): first try is 1A5AC only. 1A878 + sleep 50
		 * only after a failed 1A5AC, max 3. remove() already
		 * 1A878s on reload.
		 */
		for (attempt = 0; attempt < 3; attempt++) {
			if (attempt) {
				nimbus_power_down(n);
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
			nimbus_1a5ac_and_download(n, data, size,
						  attempt ? "retry" : "1A5AC");
			{
				u16 st = 0;

				/*
				 * Cross EXEC boundary: short wait then runtime
				 * ping only. No HBPP 1A A1 until ping fails.
				 */
				if (n->exec_sent)
					msleep(exec_wait_ms ? exec_wait_ms : 1);
				else
					msleep(2);
				err = nimbus_ping(n, &ping_st);
				if (!err) {
					n->spi_ok = true;
					n->runtime_ready = true;
					n->fw_loaded = true;
					dev_info(&spi->dev,
						 "runtime ping ok status=0x%04x (ready)\n",
						 ping_st);
				} else {
					dev_warn(&spi->dev,
						 "runtime ping fail attempt %d (exec_sent=%d)\n",
						 attempt, n->exec_sent);
					nimbus_peek(n, "post-go-fail");
					/* Diagnostics only after runtime fail. */
					if (nimbus_status_poll(n, &st) == 0)
						dev_info(&spi->dev,
							 "post-fail HBPP status 0x%04x\n",
							 st);
					err = 0; /* keep 1A878 retries going */
				}
			}
			mutex_unlock(&n->lock);
			/* Only runtime_ready ends the 1A878 retry loop. */
			if (n->runtime_ready)
				break;
		}
	}
	nimbus_release_fw(fw, kbuf);
	dev_info(&spi->dev,
		 "nimbus state uploaded=%d cal=%d reqcal=%d exec=%d runtime=%d spi_ok=%d\n",
		 n->fw_uploaded, n->cal_uploaded, n->requestcal_done,
		 n->exec_sent, n->runtime_ready, n->spi_ok);

	if (n->cal_uploaded && n->exec_sent && n->runtime_ready) {
		/* 1A5AC: 20490 after 20E94, then MultitouchTask 1703E8/188FFC. */
		nimbus_irq_enable(n);
		n->irq = spi->irq;
		if (n->irq > 0) {
			err = devm_request_threaded_irq(&spi->dev, n->irq, NULL,
							nimbus_irq_thread,
							IRQF_ONESHOT | IRQF_TRIGGER_LOW,
							"nimbus", n);
			if (err) {
				dev_warn(&spi->dev,
					 "VIC IRQ %d request failed %d — attn poll\n",
					 n->irq, err);
				n->irq = -1;
			} else {
				n->use_irq = true;
				dev_info(&spi->dev,
					 "IRQ-driven (VIC irq %d + EIC GPIO%d)\n",
					 n->irq, NIMBUS_GPIO_IRQ);
			}
		}
	} else {
		dev_err(&spi->dev,
			"Nimbus boot failed: cal=%d exec=%d runtime=%d; not registering input\n",
			n->cal_uploaded, n->exec_sent, n->runtime_ready);
		if (!n->parked)
			nimbus_park(n, "boot incomplete");
		return 0;
	}

	input = devm_input_allocate_device(&spi->dev);
	if (!input) {
		nimbus_isys_sysfs_remove(n);
		return -ENOMEM;
	}
	n->input = input;
	input->name = "Apple Nimbus";
	input->phys = "nimbus/input0";
	input->id.bustype = BUS_SPI;
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, NIMBUS_ABS_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, NIMBUS_ABS_Y_MAX, 0, 0);
	err = input_mt_init_slots(input, NIMBUS_SLOTS, INPUT_MT_DIRECT);
	if (err) {
		nimbus_isys_sysfs_remove(n);
		return err;
	}
	err = input_register_device(input);
	if (err) {
		nimbus_isys_sysfs_remove(n);
		return err;
	}

	if (ping_st) {
		mutex_lock(&n->lock);
		nimbus_read_reports(n, ping_st);
		mutex_unlock(&n->lock);
	}
	n->thread = kthread_run(nimbus_poll_thread, n, "nimbus-poll");
	if (IS_ERR(n->thread)) {
		dev_warn(&spi->dev,
			 "nimbus-poll kthread %ld — poll via IRQ only\n",
			 PTR_ERR(n->thread));
		n->thread = NULL;
	}

	dev_info(&spi->dev, "Nimbus up (attn=%d runtime=%d)\n",
		 !!n->attn, n->runtime_ready);
	return 0;
}

static void nimbus_remove(struct spi_device *spi)
{
	struct nimbus *n = spi_get_drvdata(spi);

	n->stopped = true;
	if (n->thread)
		kthread_stop(n->thread);
	nimbus_isys_sysfs_remove(n);
	nimbus_power_down(n);
}

static const struct of_device_id nimbus_of_match[] = {
	{ .compatible = "apple,nimbus" },
	{ }
};
MODULE_DEVICE_TABLE(of, nimbus_of_match);

static struct spi_driver nimbus_driver = {
	.driver = {
		.name = "apple-nimbus",
		.of_match_table = nimbus_of_match,
	},
	.probe = nimbus_probe,
	.remove = nimbus_remove,
};
module_spi_driver(nimbus_driver);

MODULE_DESCRIPTION("Apple Nimbus/Grape multitouch (N31 SPI2, RetailOS-matched)");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("apple/grape-nimbus.bin");
