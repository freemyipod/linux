// SPDX-License-Identifier: GPL-2.0
/*
 * NAND Flash driver for Samsung/Apple S5L8702 (iPod nano 3rd generation)
 *
 * The S5L8702 contains a custom Flash Memory Interface (FMI) controller.
 *
 * The FMI does NOT expose a simple CLE/ALE/data bus to the CPU.  Instead
 * commands/addresses are written to dedicated registers and data is read
 * by programming a DMA destination address and triggering a transfer.
 * Four 512-byte DMA chunks make up a 2048-byte page.
 *
 * ECC is not implemented; the controller's hardware RS/BCH engine is
 * skipped entirely for now.  Reads will succeed on ECC-clean pages.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

// FMI register offsets (base 0x38A00000, periph stride 0x400)
#define FMI_CTRL0	0x000	// control / chip-select
#define FMI_CTRL1	0x004	// trigger / phase control
#define FMI_CMD		0x008	// NAND command byte
#define FMI_ADDR0	0x00c	// address bytes [3:0], packed little-endian
#define FMI_ADDR1	0x010	// address bytes [7:4] (typically row[2] only)
#define FMI_REG14	0x014	// bank chip-select for DMA phase
#define FMI_REG24	0x024	// unknown; chunk-indexed
#define FMI_ANUM	0x02c	// address cycle count minus 1
#define FMI_DNUM	0x030	// DMA byte count minus 1
#define FMI_DESTADDR	0x034	// DMA destination (physical address)
#define FMI_REG38	0x038	// DMA transfer flags
#define FMI_STATUS	0x048	// status / interrupt flags (W1C)
// Response FIFO. The ID/status FIFO is at 0x80.
#define FMI_FIFO0	0x080	// response FIFO word 0 (ID/status reads)
#define FMI_FIFO1	0x084	// response FIFO word 1
#define FMI_FIFO2	0x088	// response FIFO word 2

// FMI_CTRL1 RFIFO/WFIFO clear bits
#define FMI_CTRL1_CLEAR_RFIFO	BIT(7)
#define FMI_CTRL1_CLEAR_WFIFO	BIT(6)

// FMI_STATUS bits (write 1 to clear)
#define FMI_ST_CMD_DONE		BIT(1)	// command cycle complete
#define FMI_ST_ADDR_DONE	BIT(2)	// address cycle complete
#define FMI_ST_FIFO_DONE	BIT(3)	// short FIFO read complete
#define FMI_ST_DMA_DONE		BIT(20)	// 512-byte DMA chunk complete

// FMI_CTRL0
#define FMI_CTRL0_BASE		0x43801u
#define FMI_CTRL0_CS(bank)	(1u << ((bank) + 1))
// DMA mode: set bits 0,24; clear bit 10; OR with the running CTRL0 value
#define FMI_CTRL0_DMA_SET	(BIT(24) | BIT(0))
#define FMI_CTRL0_DMA_CLR	BIT(10)
#define FMI_CTRL0_IDLE		0x43802u // deselect all banks

// FMI_CTRL1 trigger values
#define FMI_CTRL1_RESET		0x1f0e0u // written at start of every page op
#define FMI_CTRL1_ADDR		0x1u     // issue address cycles
#define FMI_CTRL1_FIFO_READ	0x22u    // clock N bytes from NAND into FIFO
#define FMI_CTRL1_DMA		0x1a0u   // read 512-byte chunk via DMA

// Page/chunk geometry
#define FMI_PAGE_BYTES		2048
#define FMI_CHUNK_BYTES		512
#define FMI_CHUNKS_PER_PAGE	(FMI_PAGE_BYTES / FMI_CHUNK_BYTES)

// Poll timeout (1 second)
#define FMI_TIMEOUT_US		1000000

// PWRCON0: clock-gate register (bit = 0 means clocked)
#define S5L8702_PWRCON0_PHYS	0x3c500048UL
#define PWRCON0_NAND_BIT	BIT(8)
#define PWRCON0_NAND_ECC_BIT	BIT(12)

/*
 * GPIO pinmux for the NAND data/control bus.
 * GPIO base on s5l8702 is 0x3cf00000; PCON registers are 0x20 apart.
 * We only need to map the window covering PCON8..PCON10 (0x100..0x144).
 */
#define S5L8702_GPIO_PHYS	0x3cf00000UL
#define S5L8702_GPIO_MAP_OFF	0x100
#define S5L8702_GPIO_MAP_LEN	0x60	// covers PCON8, PCON9, PCON10
#define PCON8_OFF		0x000	// relative to mapped base
#define PCON9_OFF		0x020
#define PCON10_OFF		0x040

/* PMU (Dialog D1671-class) registers and writes needed to power the
 * NAND Vdd rail before the FMI can detect any chips.
 */
#define PMU_I2C_ADDR		0x73	// 7-bit address (0xe6 >> 1)
#define PMU_REG_LDO_CTRL	0x10	// LDO master enable bitmap
#define PMU_REG_VNAND		0x15	// Vnand: 2000 + val*50 mV
#define PMU_VNAND_3000MV	0x14
#define PMU_REG_VACCY		0x18	// secondary rail
#define PMU_VACCY_VAL		0x18
#define PMU_LDO_CTRL_NAND_BIT	BIT(3)	// NAND-related bit
#define PMU_LDO_CTRL_PRESERVE	0xdb	// mask of bits to keep in 0x10
#define PMU_LDO_CTRL_ALL_ON	0xff

// -----------------------------------------------------------------------
// Driver state
// -----------------------------------------------------------------------

struct s5l8702_nand {
	struct nand_controller	controller;
	struct nand_chip	chip;
	struct device		*dev;
	void __iomem		*regs;
	void __iomem		*pwrcon0;
	void __iomem		*gpio;	  // PCON8..PCON10 window
	u32			base_bank; // Starting NAND bank from DT
	u32			bank;	  // Current NAND bank for I/O

	// Pre-allocated DMA-coherent buffer: one full page (4 × 512 B)
	void			*dma_virt;
	dma_addr_t		dma_phys;

	// Captured OOB data (16 bytes per 512B chunk = 64 bytes total).
	// Filled as a side effect of fmi_read_chunk; consumed by the
	// ecc.read_page / ecc.read_oob hooks.
	u8			oob_buf[64] __aligned(4);
};

static inline struct s5l8702_nand *chip_to_priv(struct nand_chip *chip) { return container_of(chip, struct s5l8702_nand, chip); }

/* -----------------------------------------------------------------------
 * FMI helpers
 * ----------------------------------------------------------------------- */

static int fmi_wait(struct s5l8702_nand *priv, u32 bit) {
	u32 val;
	int ret;

	ret = readl_poll_timeout(priv->regs + FMI_STATUS, val, val & bit, 0, FMI_TIMEOUT_US);
	if (ret) {
		dev_err(priv->dev, "fmi_wait: TIMEOUT waiting for STATUS bit 0x%08x; STATUS=0x%08x CTRL0=0x%08x CTRL1=0x%08x\n", bit, readl(priv->regs + FMI_STATUS), readl(priv->regs + FMI_CTRL0), readl(priv->regs + FMI_CTRL1));
		return ret;
	}
	dev_dbg(priv->dev, "fmi_wait: STATUS bit 0x%08x fired (STATUS=0x%08x)\n", bit, val);
	writel(bit, priv->regs + FMI_STATUS);
	return 0;
}

static int fmi_cmd(struct s5l8702_nand *priv, u8 opcode) {
	dev_dbg(priv->dev, "fmi_cmd: CMD=0x%02x\n", opcode);
	writel(1u << priv->bank, priv->regs + FMI_REG14);
	writel(opcode, priv->regs + FMI_CMD);
	return fmi_wait(priv, FMI_ST_CMD_DONE);
}

static int fmi_addr(struct s5l8702_nand *priv, const u8 *addrs, unsigned int naddrs) {
	u32 addr0 = 0, addr1 = 0;

	/*
	 * The FMI clocks address bytes from a fixed set of register-byte
	 * positions: addr0[7:0], addr0[15:8], addr0[31:24], addr1[7:0],
	 * addr1[15:8]. Bits 23:16 of addr0 are unused. Pack the framework's
	 * little-endian addrs[] into those positions.
	 */
	if (naddrs > 0) addr0 |= (u32)addrs[0];
	if (naddrs > 1) addr0 |= (u32)addrs[1] << 8;
	if (naddrs > 2) addr0 |= (u32)addrs[2] << 24;
	if (naddrs > 3) addr1 |= (u32)addrs[3];
	if (naddrs > 4) addr1 |= (u32)addrs[4] << 8;
	if (naddrs > 5) return -EINVAL;

	dev_dbg(priv->dev, "fmi_addr: naddrs=%u addr0=0x%08x addr1=0x%08x\n", naddrs, addr0, addr1);

	writel(1u << priv->bank, priv->regs + FMI_REG14);
	writel(naddrs - 1, priv->regs + FMI_ANUM);
	writel(addr0, priv->regs + FMI_ADDR0);
	writel(addr1, priv->regs + FMI_ADDR1);
	writel(FMI_CTRL1_ADDR, priv->regs + FMI_CTRL1);
	return fmi_wait(priv, FMI_ST_ADDR_DONE);
}

/*
 * Read one 512-byte chunk from the FMI via DMA into the coherent
 * buffer at offset (chunk × 512), then copy to the caller's buffer.
 */
static int fmi_read_chunk(struct s5l8702_nand *priv, unsigned int chunk, u8 *dst) {
	dma_addr_t dest = priv->dma_phys + (dma_addr_t)chunk * FMI_CHUNK_BYTES;
	u32 ctrl0;
	unsigned int i;
	int ret;

	dev_dbg(priv->dev, "fmi_read_chunk: chunk=%u dest=0x%08x STATUS=0x%08x\n", chunk, (u32)dest, readl(priv->regs + FMI_STATUS));

	// Clear status bits
	writel(BIT(27) | FMI_ST_FIFO_DONE | FMI_ST_DMA_DONE, priv->regs + FMI_STATUS);

	// Set REG24 which is toggled halfway through the page
	writel(chunk < 2 ? 0 : 0x10, priv->regs + FMI_REG24);

	// 1. Fetch 16 bytes of OOB.
	writel(16 - 1, priv->regs + FMI_DNUM);
	writel(1u << (priv->bank + 4), priv->regs + FMI_REG14);
	writel(0x32, priv->regs + FMI_CTRL1);
	ret = fmi_wait(priv, FMI_ST_FIFO_DONE);
	if (ret) return ret;

	// Capture the 16 bytes of OOB for this chunk. The response FIFO is
	// exposed as four 32-bit windows at FMI_FIFO0..FMI_FIFO0+0xC, not as
	// a single popping register, so step through each window.
	for (i = 0; i < 4; i++) ((u32 *)priv->oob_buf)[chunk * 4 + i] = readl(priv->regs + FMI_FIFO0 + i * 4);

	// 2. Fetch 512 bytes of data from NAND to FMI internal buffer
	writel(FMI_CHUNK_BYTES - 1, priv->regs + FMI_DNUM);
	writel(1u << priv->bank, priv->regs + FMI_REG14);
	writel(FMI_CTRL1_FIFO_READ, priv->regs + FMI_CTRL1);
	ret = fmi_wait(priv, FMI_ST_FIFO_DONE);
	if (ret) return ret;

	// 3. DMA from FMI internal buffer to RAM
	writel((u32)dest, priv->regs + FMI_DESTADDR);
	writel(7, priv->regs + FMI_REG38);

	ctrl0 = readl(priv->regs + FMI_CTRL0);
	writel((ctrl0 & ~FMI_CTRL0_DMA_CLR) | FMI_CTRL0_DMA_SET, priv->regs + FMI_CTRL0);

	writel(1u << (priv->bank + 8), priv->regs + FMI_REG14);
	writel(FMI_CTRL1_DMA, priv->regs + FMI_CTRL1);

	ret = fmi_wait(priv, FMI_ST_DMA_DONE);
	if (ret) {
		dev_err(priv->dev, "fmi_read_chunk: chunk=%u DMA_DONE timeout\n", chunk);
		return ret;
	}

	memcpy(dst, (u8 *)priv->dma_virt + chunk * FMI_CHUNK_BYTES, FMI_CHUNK_BYTES);

	dev_dbg(priv->dev, "fmi_read_chunk: chunk=%u done, first 8 bytes: %*ph\n", chunk, 8, (u8 *)priv->dma_virt + chunk * FMI_CHUNK_BYTES);
	return 0;
}

/*
 * Trigger one FIFO read cycle of up to 4 bytes and copy the result.
 */
static int fmi_fifo_read_word(struct s5l8702_nand *priv, u8 *buf, unsigned int len) {
	u32 word;
	unsigned int i;
	int ret;

	if (WARN_ON(len == 0 || len > 2)) return -EINVAL;

	writel(FMI_ST_FIFO_DONE, priv->regs + FMI_STATUS);

	dev_dbg(priv->dev, "fmi_fifo_read_word: pre-trigger STATUS=0x%08x CTRL1=0x%08x\n", readl(priv->regs + FMI_STATUS), readl(priv->regs + FMI_CTRL1));

	writel(len - 1, priv->regs + FMI_DNUM);
	writel(1u << priv->bank, priv->regs + FMI_REG14);
	writel(FMI_CTRL1_FIFO_READ, priv->regs + FMI_CTRL1);

	ret = fmi_wait(priv, FMI_ST_FIFO_DONE);
	if (ret) return ret;

	udelay(200);

	word = readl(priv->regs + FMI_FIFO0);

	dev_dbg(priv->dev, "fmi_fifo_read_word: len=%u FIFO0=0x%08x post STATUS=0x%08x\n", len, word, readl(priv->regs + FMI_STATUS));

	for (i = 0; i < len; i++) buf[i] = (word >> (i * 8)) & 0xff;

	return 0;
}

/*
 * Read `len` bytes from the chip via the response FIFO.
 *
 * The FMI's FIFO_READ trigger can only deliver 2 bytes per cycle on
 * this controller, so larger reads are split into multiple triggers.
 * Within a single READID command the chip keeps clocking out the
 * next ID bytes on each continued FIFO_READ trigger, so we must NOT
 * re-issue CMD 0x90 + ADDR 0x00 between chunks — that would just
 * re-read bytes 0-1 every time.
 */
static int fmi_fifo_read(struct s5l8702_nand *priv, u8 *buf, unsigned int len)
{
	unsigned int done = 0;
	int ret;

	while (done < len) {
		unsigned int chunk = min(len - done, 2u);

		ret = fmi_fifo_read_word(priv, buf + done, chunk);
		if (ret)
			return ret;
		done += chunk;
	}

	return 0;
}

/* -----------------------------------------------------------------------
 * exec_op implementation
 * ----------------------------------------------------------------------- */

static int s5l8702_exec_instr(struct s5l8702_nand *priv,
			      const struct nand_op_instr *instr)
{
	unsigned int i;
	int ret = 0;

	switch (instr->type) {
	case NAND_OP_CMD_INSTR:
		ret = fmi_cmd(priv, instr->ctx.cmd.opcode);
		break;

	case NAND_OP_ADDR_INSTR:
		ret = fmi_addr(priv, instr->ctx.addr.addrs,
			       instr->ctx.addr.naddrs);
		break;

	case NAND_OP_DATA_IN_INSTR: {
		u8 *buf = instr->ctx.data.buf.in;
		unsigned int len = instr->ctx.data.len;

		if (len >= FMI_CHUNK_BYTES) {
			// Full-page (multi-chunk) read via DMA. As a side
			// effect, priv->oob_buf is filled with the page's
			// OOB; the ecc.read_page / read_oob hooks copy it
			// into chip->oob_poi for callers that ask for OOB.
			for (i = 0; i < len / FMI_CHUNK_BYTES; i++) {
				ret = fmi_read_chunk(priv, i,
						     buf + i * FMI_CHUNK_BYTES);
				if (ret)
					break;
			}
		} else {
			// Short reads: READID, READ STATUS, etc.
			ret = fmi_fifo_read(priv, buf, len);
		}
		break;
	}

	case NAND_OP_DATA_OUT_INSTR:
		// Write support not yet implemented
		return -EOPNOTSUPP;

	case NAND_OP_WAITRDY_INSTR:
		// The FMI handles R/B# internally. Use a fixed delay for now.
		udelay(500);
		break;
	}

	if (!ret && instr->delay_ns)
		ndelay(instr->delay_ns);

	return ret;
}

static int s5l8702_nand_exec_op(struct nand_chip *chip, const struct nand_operation *op, bool check_only) {
	struct s5l8702_nand *priv = chip_to_priv(chip);
	unsigned int i;
	int ret;

	if (check_only) return 0;

	priv->bank = priv->base_bank + op->cs;
	dev_dbg(priv->dev, "exec_op: %u instrs, cs=%u (bank=%u)\n", op->ninstrs, op->cs, priv->bank);

	writel(FMI_CTRL1_RESET, priv->regs + FMI_CTRL1);
	writel(FMI_CTRL0_BASE | FMI_CTRL0_CS(priv->bank), priv->regs + FMI_CTRL0);

	for (i = 0; i < op->ninstrs; i++) {
		ret = s5l8702_exec_instr(priv, &op->instrs[i]);
		if (ret) goto out_deselect;
	}
	ret = 0;

out_deselect:
	writel(FMI_CTRL0_IDLE, priv->regs + FMI_CTRL0);
	return ret;
}

/*
 * The default nand_read_oob_std issues READ0 with col=writesize, which
 * lands the chip in OOB territory and then expects a generic data read
 * to clock OOB bytes out. The FMI doesn't expose that path — OOB is
 * only reachable as a side effect of fmi_read_chunk. Override the page
 * and OOB read hooks so both go through the chunk loop, and copy the
 * captured OOB into chip->oob_poi.
 */
static int s5l8702_read_page_raw(struct nand_chip *chip, u8 *buf,
				 int oob_required, int page)
{
	struct s5l8702_nand *priv = chip_to_priv(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ret;

	ret = nand_read_page_op(chip, page, 0, buf, mtd->writesize);
	if (ret)
		return ret;

	if (oob_required)
		memcpy(chip->oob_poi, priv->oob_buf, mtd->oobsize);

	return 0;
}

static int s5l8702_read_oob_raw(struct nand_chip *chip, int page)
{
	struct s5l8702_nand *priv = chip_to_priv(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ret;

	// Throw the data away; we only want the OOB side effect.
	ret = nand_read_page_op(chip, page, 0, priv->dma_virt, mtd->writesize);
	if (ret)
		return ret;

	memcpy(chip->oob_poi, priv->oob_buf, mtd->oobsize);
	return 0;
}

static int s5l8702_nand_attach_chip(struct nand_chip *chip) {
	struct s5l8702_nand *priv = chip_to_priv(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);

	// Skip error correction for now.
	chip->ecc.engine_type = NAND_ECC_ENGINE_TYPE_NONE;
	chip->ecc.read_page = s5l8702_read_page_raw;
	chip->ecc.read_page_raw = s5l8702_read_page_raw;
	chip->ecc.read_oob = s5l8702_read_oob_raw;
	chip->ecc.read_oob_raw = s5l8702_read_oob_raw;

	/*
	 * Auto-detect from the 4-byte ID misreads the eraseblock-size field
	 * for this Apple-spec'd Micron part (full ID 0xA5D5D52C). The chip
	 * wants 128 pages per eraseblock (256 KiB); the framework picks 64
	 * (128 KiB). Halve eraseblocks_per_lun to keep total size at 2 GiB
	 * per chip and recompute mtd->erasesize before nand_scan_tail uses
	 * it.
	 */
	if (chip->base.memorg.pages_per_eraseblock == 64 &&
	    chip->base.memorg.eraseblocks_per_lun  == 16384) {
		chip->base.memorg.pages_per_eraseblock = 128;
		chip->base.memorg.eraseblocks_per_lun  = 8192;
		mtd->erasesize = chip->base.memorg.pages_per_eraseblock * mtd->writesize;
		// nand_scan_ident already computed these from the wrong
		// erasesize before attach_chip ran; recompute them here.
		chip->phys_erase_shift = ffs(mtd->erasesize) - 1;
		chip->bbt_erase_shift  = chip->phys_erase_shift;
		dev_info(priv->dev,
			 "geometry override: erasesize=%u blocks/chip=%u\n",
			 mtd->erasesize, chip->base.memorg.eraseblocks_per_lun);
	}

	return 0;
}

static const struct nand_controller_ops s5l8702_nand_ops = {
	.exec_op     = s5l8702_nand_exec_op,
	.attach_chip = s5l8702_nand_attach_chip,
};

/* -----------------------------------------------------------------------
 * PMU power-on sequence
 * ----------------------------------------------------------------------- */

/*
 * Write a single byte to a PMU register over I2C.
 * The PMU protocol is: [i2c_addr_W] [reg] [val]
 * TODO: factor this out into a separate PMU driver
 */
static int pmu_wr(struct i2c_adapter *adap, u8 reg, u8 val) {
	u8 buf[2] = { reg, val };
	struct i2c_msg msg = { .addr = PMU_I2C_ADDR, .flags = 0, .len = sizeof(buf), .buf = buf };

	return i2c_transfer(adap, &msg, 1) == 1 ? 0 : -EIO;
}

/*
 * Read a single byte from a PMU register over I2C.
 * Write [reg], then read [val].
 */
static int pmu_rd(struct i2c_adapter *adap, u8 reg, u8 *val) {
	struct i2c_msg msgs[2] = {
		{ .addr = PMU_I2C_ADDR, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = PMU_I2C_ADDR, .flags = I2C_M_RD, .len = 1, .buf = val },
	};
	return i2c_transfer(adap, msgs, 2) == 2 ? 0 : -EIO;
}

/*
 * Power on the NAND Vdd rail via the PMU.
 * Mirrors Rockbox's nand_power_on() for the iPod nano 3g.
 */
static int s5l8702_nand_power_on(struct device *dev, struct i2c_adapter *adap) {
	u8 ctrl = 0;
	int ret;

	// Vnand voltage select (~3000 mV).
	ret = pmu_wr(adap, PMU_REG_VNAND, PMU_VNAND_3000MV);
	dev_info(dev, "PMU: write Vnand[0x15]=0x14 -> %d\n", ret);
	if (ret) return dev_err_probe(dev, ret, "PMU: Vnand write failed\n");

	// Vaccy / secondary rail.
	ret = pmu_wr(adap, PMU_REG_VACCY, PMU_VACCY_VAL);
	dev_info(dev, "PMU: write Vaccy[0x18]=0x18 -> %d\n", ret);
	if (ret) return dev_err_probe(dev, ret, "PMU: Vaccy write failed\n");

	// RMW reg 0x10 sequence: keep most bits, set bit 3.
	ret = pmu_rd(adap, PMU_REG_LDO_CTRL, &ctrl);
	dev_info(dev, "PMU: read LDO ctrl[0x10]=0x%02x -> %d\n", ctrl, ret);
	if (ret) return dev_err_probe(dev, ret, "PMU: LDO ctrl read failed\n");

	ret = pmu_wr(adap, PMU_REG_LDO_CTRL, (ctrl & PMU_LDO_CTRL_PRESERVE) | PMU_LDO_CTRL_NAND_BIT);
	dev_info(dev, "PMU: pmu_preinit LDO ctrl[0x10]<-0x%02x -> %d\n", (ctrl & PMU_LDO_CTRL_PRESERVE) | PMU_LDO_CTRL_NAND_BIT, ret);
	if (ret) return dev_err_probe(dev, ret, "PMU: LDO ctrl write failed\n");

	// Force all LDOs on right before NAND I/O.
	ret = pmu_wr(adap, PMU_REG_LDO_CTRL, PMU_LDO_CTRL_ALL_ON);
	dev_info(dev, "PMU: dumper LDO ctrl[0x10]<-0xff -> %d\n", ret);
	if (ret) return dev_err_probe(dev, ret, "PMU: LDO all-on write failed\n");

	pmu_rd(adap, PMU_REG_LDO_CTRL, &ctrl);
	dev_info(dev, "PMU: readback LDO ctrl[0x10]=0x%02x\n", ctrl);

	// Let the rail ramp before any FMI traffic.
	msleep(50);
	return 0;
}

/* -----------------------------------------------------------------------
 * GPIO pinmux.
 * ----------------------------------------------------------------------- */

static void s5l8702_nand_configure_gpio(struct s5l8702_nand *priv) {
	u32 v;

	writel(0x22222222, priv->gpio + PCON8_OFF);

	v = readl(priv->gpio + PCON9_OFF);
	writel((v & 0xfff0fff0u) | 0x00020002u, priv->gpio + PCON9_OFF);

	v = readl(priv->gpio + PCON10_OFF);
	writel((v & 0xffff0000u) | 0x00002222u, priv->gpio + PCON10_OFF);

	dev_info(priv->dev, "GPIO pinmux: PCON8=0x%08x PCON9=0x%08x PCON10=0x%08x\n", readl(priv->gpio + PCON8_OFF), readl(priv->gpio + PCON9_OFF), readl(priv->gpio + PCON10_OFF));
}

/* -----------------------------------------------------------------------
 * Platform driver
 * ----------------------------------------------------------------------- */

static int s5l8702_nand_probe(struct platform_device *pdev) {
	struct s5l8702_nand *priv;
	struct i2c_adapter *adap;
	struct device_node *i2c_np;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	u32 bank;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) return -ENOMEM;

	priv->dev = &pdev->dev;
	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs)) return PTR_ERR(priv->regs);

	priv->pwrcon0 = devm_ioremap(&pdev->dev, S5L8702_PWRCON0_PHYS, 4);
	if (!priv->pwrcon0) return -ENOMEM;

	priv->gpio = devm_ioremap(&pdev->dev, S5L8702_GPIO_PHYS + S5L8702_GPIO_MAP_OFF, S5L8702_GPIO_MAP_LEN);
	if (!priv->gpio) return -ENOMEM;

	// Configure pin-mux PCON8..PCON10 to NAND alternate function.
	s5l8702_nand_configure_gpio(priv);

	dev_info(&pdev->dev, "PWRCON0 before ungate: 0x%08x\n", readl(priv->pwrcon0));
	writel(readl(priv->pwrcon0) & ~(PWRCON0_NAND_BIT | PWRCON0_NAND_ECC_BIT), priv->pwrcon0);
	dev_info(&pdev->dev, "PWRCON0 after  ungate: 0x%08x\n", readl(priv->pwrcon0));

	i2c_np = of_parse_phandle(pdev->dev.of_node, "pmu-i2c", 0);
	if (!i2c_np) return dev_err_probe(&pdev->dev, -ENODEV, "pmu-i2c phandle missing\n");
	adap = of_find_i2c_adapter_by_node(i2c_np);
	of_node_put(i2c_np);
	if (!adap) return dev_err_probe(&pdev->dev, -EPROBE_DEFER, "PMU I2C adapter not ready\n");
	ret = s5l8702_nand_power_on(&pdev->dev, adap);
	i2c_put_adapter(adap);
	if (ret) return ret;

	if (of_property_read_u32(pdev->dev.of_node, "apple,nand-bank", &bank))
		bank = 0;
	priv->base_bank = bank;
	priv->bank = bank;

	// DMA-coherent buffer for page reads (4 × 512 B)
	priv->dma_virt = dma_alloc_coherent(&pdev->dev, FMI_PAGE_BYTES,
					    &priv->dma_phys, GFP_KERNEL);
	if (!priv->dma_virt)
		return -ENOMEM;

	nand_controller_init(&priv->controller);
	priv->controller.ops = &s5l8702_nand_ops;

	chip = &priv->chip;
	chip->controller = &priv->controller;

	// The NAND reports a 16-bit bus, but the physical bus is 8-bit.
	// Set this flag to satisfy the framework's bus-width sanity check.
	chip->options |= NAND_BUSWIDTH_16;

	// Skip the bad-block scan. This NAND has probably been written by
	// Apple's NAND Driver, which stores its own metadata in OOB byte 0.
	// The standard "OOB[0] != 0xFF means bad" convention does not apply,
	// so suppress the scan and tell MTD not to consult the marker either.
	chip->options |= NAND_SKIP_BBTSCAN | NAND_NO_BBM_QUIRK;

	nand_set_flash_node(chip, pdev->dev.of_node);

	mtd = nand_to_mtd(chip);
	mtd->dev.parent = &pdev->dev;
	mtd->name = "s5l8702-nand";

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev,
		 "FMI regs before nand_scan: CTRL0=0x%08x CTRL1=0x%08x STATUS=0x%08x\n",
		 readl(priv->regs + FMI_CTRL0),
		 readl(priv->regs + FMI_CTRL1),
		 readl(priv->regs + FMI_STATUS));

	// 4 GB iPod nano 3g has 2 NAND banks; the 8 GB has 4. Both fit in
	// the maximum we pass here, and nand_scan stops as soon as a CS
	// fails to respond to READID.
	ret = nand_scan(chip, 4);
	if (ret) {
		dev_err(&pdev->dev, "nand_scan failed: %d\n", ret);
		goto err_free_dma;
	}

	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "mtd_device_register failed: %d\n", ret);
		nand_cleanup(chip);
		goto err_free_dma;
	}

	return 0;

err_free_dma:
	dma_free_coherent(&pdev->dev, FMI_PAGE_BYTES,
			  priv->dma_virt, priv->dma_phys);
	return ret;
}

static void s5l8702_nand_remove(struct platform_device *pdev) {
	struct s5l8702_nand *priv = platform_get_drvdata(pdev);
	struct nand_chip *chip = &priv->chip;
	int ret;

	ret = mtd_device_unregister(nand_to_mtd(chip));
	WARN_ON(ret);
	nand_cleanup(chip);
	dma_free_coherent(&pdev->dev, FMI_PAGE_BYTES, priv->dma_virt, priv->dma_phys);
}

static const struct of_device_id s5l8702_nand_of_match[] = {
	{ .compatible = "apple,s5l8702-nand" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8702_nand_of_match);

static struct platform_driver s5l8702_nand_driver = {
	.probe  = s5l8702_nand_probe,
	.remove = s5l8702_nand_remove,
	.driver = {
		.name           = "s5l8702-nand",
		.of_match_table = s5l8702_nand_of_match,
	},
};
module_platform_driver(s5l8702_nand_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NAND driver for Samsung/Apple S5L8702 (iPod nano 3g)");
MODULE_AUTHOR("Tucker Osman <osmiumusa@gmail.com>");
