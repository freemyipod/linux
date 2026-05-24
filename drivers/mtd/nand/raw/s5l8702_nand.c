// SPDX-License-Identifier: GPL-2.0
/*
 * NAND Flash driver for Samsung/Apple S5L8702 (iPod nano 3rd generation)
 *
 * The S5L8702 contains a custom Flash Memory Interface (FMI) controller.
 *
 * The FMI does NOT expose a simple CLE/ALE/data bus to the CPU.  Instead
 * commands/addresses are written to dedicated registers and data is read
 * by programming a DMA destination address and triggering a transfer.
 * Four 512-byte DMA chunks make up a 2048-byte page. The controller's
 * hardware BCH engine corrects bit errors in-place between the chunk
 * fetch and the DMA copy.
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

// FMI register offsets (channel base 0x38A00000, channel stride 0x400).
#define FMI_CTRL0	0x000	// chip-select + global enable
#define FMI_CTRL1	0x004	// operation/phase trigger
#define FMI_CMD		0x008	// NAND command byte
#define FMI_ADDRL	0x00c	// address bytes [3:0] (col0, col1, --, row0)
#define FMI_ADDRH	0x010	// address bytes [5:4] (row1, row2)
#define FMI_CHUNK_TRIG	0x014	// per-stage trigger bitmask
#define FMI_CHUNK_OFFSET 0x024	// which half of the page (0x00 or 0x10)
#define FMI_ANUM	0x02c	// address cycle count minus 1
#define FMI_CHUNK_SIZE	0x030	// bytes minus 1 for the next stage
#define FMI_DMA_DEST	0x034	// DMA destination physical address
#define FMI_DMA_LEN	0x038	// DMA beat count (always 7 = 512 bytes)
#define FMI_DMA_STATUS	0x044	// per-transfer DMA status (W1C)
#define FMI_STATUS	0x048	// composite status / interrupt flags (W1C)
// End-of-page spare-meta drain FIFO (12 bytes total).
#define FMI_SPARE_FIFO0	0x060
#define FMI_SPARE_FIFO1	0x064
#define FMI_SPARE_FIFO2	0x068
#define FMI_FIFO_CTL	0x078	// arm spare-FIFO drain
#define FMI_FIFO_STATUS	0x07c	// drain handshake (write 2; poll bit 1 clear)
// Short-response FIFO used for READID and similar short reads. This is a
// distinct window from the spare-meta FIFO above.
#define FMI_FIFO0	0x080
#define FMI_FIFO1	0x084
#define FMI_FIFO2	0x088

// FMI_STATUS bits (write 1 to clear)
#define FMI_ST_CMD_DONE		BIT(1)	// command cycle complete
#define FMI_ST_ADDR_DONE	BIT(2)	// address cycle complete
#define FMI_ST_FIFO_DONE	BIT(3)	// chunk / short FIFO read complete
#define FMI_ST_DMA_DONE		BIT(20)	// 512-byte DMA chunk complete
#define FMI_ST_ECC_READY	BIT(27)	// ECC engine has data ready to process

// Spare-meta FIFO drain magic.
#define FMI_FIFO_CTL_ARM	0x00005140u
#define FMI_FIFO_STATUS_BUSY	BIT(1)

// FMI BCH ECC engine registers (at FMI base + 0x800).
#define FMI_ECC_TRIG		0x80c    // chunk-select trigger
#define FMI_ECC_STATUS		0x810    // bit 0 = uncorrectable; bits 16:19 = err count
#define FMI_ECC_CONFIG		0x814    // engine config
#define FMI_ECC_STATUS_CLEAR	0x840    // write 0x7f to clear all, bit 2 = done
#define FMI_ECC_CONFIG_VAL	0x01000180u
#define FMI_ECC_DONE_BIT	BIT(2)   // in FMI_ECC_STATUS_CLEAR
#define FMI_ECC_UNCORR		BIT(0)   // in FMI_ECC_STATUS

// FMI_CTRL0
#define FMI_CTRL0_BASE		0x43801u
#define FMI_CTRL0_CS(bank)	(1u << ((bank) + 1))
// DMA mode: set bits 0,24; clear bit 10; OR with the running CTRL0 value
#define FMI_CTRL0_DMA_SET	(BIT(24) | BIT(0))
#define FMI_CTRL0_DMA_CLR	BIT(10)
#define FMI_CTRL0_IDLE		0x43802u // deselect all banks

// FMI_CTRL1 trigger values
#define FMI_CTRL1_RESET		0x1f0e0u // reset/idle at start of every page op
#define FMI_CTRL1_ADDR		0x1u     // issue address cycles
#define FMI_CTRL1_FIFO_READ_LO	0x22u    // FIFO read, chunks in low half of page
#define FMI_CTRL1_FIFO_READ_HI	0x32u    // FIFO read, chunks in high half of page
#define FMI_CTRL1_DMA		0x1a0u   // read 512-byte chunk via DMA
#define FMI_CTRL1_FIFO_READ	FMI_CTRL1_FIFO_READ_LO // alias for short reads

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
		dev_err(priv->dev, "fmi_wait: timeout waiting for STATUS bit 0x%08x\n", bit);
		return ret;
	}
	writel(bit, priv->regs + FMI_STATUS);
	return 0;
}

static int fmi_cmd(struct s5l8702_nand *priv, u8 opcode) {
	writel(1u << priv->bank, priv->regs + FMI_CHUNK_TRIG);
	writel(opcode, priv->regs + FMI_CMD);
	return fmi_wait(priv, FMI_ST_CMD_DONE);
}

static int fmi_addr(struct s5l8702_nand *priv, const u8 *addrs, unsigned int naddrs) {
	u32 addr0 = 0, addr1 = 0;

	/*
	 * The FMI packs the address-cycle stream contiguously into ADDRL
	 * then ADDRH, little-endian: bytes 0..3 go into ADDRL[0..31],
	 * bytes 4..5 into ADDRH[0..15].
	 */
	if (naddrs > 0) addr0 |= (u32)addrs[0];
	if (naddrs > 1) addr0 |= (u32)addrs[1] << 8;
	if (naddrs > 2) addr0 |= (u32)addrs[2] << 16;
	if (naddrs > 3) addr0 |= (u32)addrs[3] << 24;
	if (naddrs > 4) addr1 |= (u32)addrs[4];
	if (naddrs > 5) addr1 |= (u32)addrs[5] << 8;
	if (naddrs > 6) return -EINVAL;

	writel(1u << priv->bank, priv->regs + FMI_CHUNK_TRIG);
	writel(naddrs - 1, priv->regs + FMI_ANUM);
	writel(addr0, priv->regs + FMI_ADDRL);
	writel(addr1, priv->regs + FMI_ADDRH);
	writel(FMI_CTRL1_ADDR, priv->regs + FMI_CTRL1);
	return fmi_wait(priv, FMI_ST_ADDR_DONE);
}

/*
 * Run the FMI BCH ECC engine on a chunk that's currently sitting in the
 * controller's internal buffer. The hardware applies corrections in-place
 * before we DMA the chunk out to RAM.
 *
 * Returns:
 *   0          — chunk clean
 *   positive   — number of corrected bit errors
 *   -EBADMSG   — uncorrectable
 */
static int fmi_run_ecc(struct s5l8702_nand *priv, unsigned int chunk) {
	unsigned int meta = chunk & 1;
	unsigned int data = meta + 4;
	unsigned int hi   = chunk >> 1;
	u32 trig, status;
	int i;

	// ECC trigger encodes which sub-chunk bits to process + which half-page.
	trig = (1u << (data + 8)) | ((u32)hi << 16) | (1u << (meta + 8)) | 1;

	writel(0x7f,               priv->regs + FMI_ECC_STATUS_CLEAR);
	writel(FMI_ECC_CONFIG_VAL, priv->regs + FMI_ECC_CONFIG);
	writel(trig,               priv->regs + FMI_ECC_TRIG);

	// Poll FMI_ECC_STATUS_CLEAR bit 2 for ECC done.
	for (i = 0; i < FMI_TIMEOUT_US; i++) {
		if (readl(priv->regs + FMI_ECC_STATUS_CLEAR) & FMI_ECC_DONE_BIT)
			break;
		udelay(1);
	}
	if (i >= FMI_TIMEOUT_US) {
		dev_err(priv->dev, "fmi_run_ecc: chunk=%u ECC done timeout\n", chunk);
		return -ETIMEDOUT;
	}

	status = readl(priv->regs + FMI_ECC_STATUS);
	writel(FMI_ECC_DONE_BIT, priv->regs + FMI_ECC_STATUS_CLEAR);

	if (status & FMI_ECC_UNCORR) {
		dev_warn_ratelimited(priv->dev,
			"fmi_run_ecc: chunk=%u UNCORRECTABLE (status=0x%08x)\n",
			chunk, status);
		return -EBADMSG;
	}

	return (status >> 16) & 0xf;
}

/*
 * Read one 512-byte chunk into the coherent buffer at offset (chunk × 512),
 * then copy to the caller's buffer. Runs hardware BCH between the FIFO
 * read and the DMA so corrections are applied in place.
 */
static int fmi_read_chunk(struct s5l8702_nand *priv, unsigned int chunk, u8 *dst) {
	dma_addr_t dest = priv->dma_phys + (dma_addr_t)chunk * FMI_CHUNK_BYTES;
	unsigned int meta = chunk & 1;
	unsigned int data = meta + 4;
	unsigned int hi   = chunk >> 1;
	u32 ctrl0, status;
	int ret;

	writel(FMI_ST_ECC_READY | FMI_ST_FIFO_DONE | FMI_ST_DMA_DONE, priv->regs + FMI_STATUS);

	// 1. Spare/meta sub-chunk (16 bytes) — ECC framing.
	writel(16 - 1,                 priv->regs + FMI_CHUNK_SIZE);
	writel(1u << data,             priv->regs + FMI_CHUNK_TRIG);
	writel(hi ? 0x10 : 0,          priv->regs + FMI_CHUNK_OFFSET);
	writel(FMI_CTRL1_FIFO_READ_HI, priv->regs + FMI_CTRL1);
	ret = fmi_wait(priv, FMI_ST_FIFO_DONE);
	if (ret) return ret;

	// 2. Data sub-chunk (512 bytes) — clock NAND -> FMI internal buffer.
	writel(FMI_CHUNK_BYTES - 1,    priv->regs + FMI_CHUNK_SIZE);
	writel(1u << meta,             priv->regs + FMI_CHUNK_TRIG);
	writel(0,                      priv->regs + FMI_CHUNK_OFFSET);
	writel(FMI_CTRL1_FIFO_READ_LO, priv->regs + FMI_CTRL1);
	ret = fmi_wait(priv, FMI_ST_FIFO_DONE);
	if (ret) return ret;

	// 2b. ECC: if the engine has data ready, run BCH correction in place
	// before the DMA copies the corrected chunk to RAM.
	status = readl(priv->regs + FMI_STATUS);
	if (status & FMI_ST_ECC_READY) {
		ret = fmi_run_ecc(priv, chunk);
		// ret < 0 (e.g. -EBADMSG) means uncorrectable. Apple's FTL has
		// its own retry/scrub logic so we just propagate the error.
		// Corrected (ret > 0) is fine; data is already fixed in the buffer.
		if (ret < 0)
			return ret;
	}

	// 3. DMA from FMI internal buffer to RAM.
	writel((u32)dest, priv->regs + FMI_DMA_DEST);
	writel(7,         priv->regs + FMI_DMA_LEN);
	ctrl0 = readl(priv->regs + FMI_CTRL0);
	writel((ctrl0 & ~FMI_CTRL0_DMA_CLR) | FMI_CTRL0_DMA_SET, priv->regs + FMI_CTRL0);
	writel(1u << (meta + 8), priv->regs + FMI_CHUNK_TRIG);
	writel(FMI_CTRL1_DMA,    priv->regs + FMI_CTRL1);

	ret = fmi_wait(priv, FMI_ST_DMA_DONE);
	if (ret) {
		dev_err(priv->dev, "fmi_read_chunk: chunk=%u DMA_DONE timeout\n", chunk);
		return ret;
	}

	memcpy(dst, (u8 *)priv->dma_virt + chunk * FMI_CHUNK_BYTES, FMI_CHUNK_BYTES);
	return 0;
}

/*
 * After the 4 data chunks finish, drain the 12-byte spare-meta FIFO.
 * The three SPARE_FIFO registers (0x60/0x64/0x68) hold the FTL metadata
 * that Apple's NAND stack consumes alongside the page data.
 */
static int fmi_read_oob(struct s5l8702_nand *priv) {
	u32 status;
	int ret;

	writel(FMI_ST_FIFO_DONE | FMI_ST_DMA_DONE, priv->regs + FMI_STATUS);

	writel(FMI_FIFO_CTL_ARM,      priv->regs + FMI_FIFO_CTL);
	writel(FMI_FIFO_STATUS_BUSY,  priv->regs + FMI_FIFO_STATUS);

	ret = readl_poll_timeout(priv->regs + FMI_FIFO_STATUS, status,
				 !(status & FMI_FIFO_STATUS_BUSY),
				 0, FMI_TIMEOUT_US);
	if (ret) {
		dev_err(priv->dev, "fmi_read_oob: FIFO_STATUS timeout (=0x%08x)\n",
			readl(priv->regs + FMI_FIFO_STATUS));
		return ret;
	}

	memset(priv->oob_buf, 0, sizeof(priv->oob_buf));
	((u32 *)priv->oob_buf)[0] = readl(priv->regs + FMI_SPARE_FIFO0);
	((u32 *)priv->oob_buf)[1] = readl(priv->regs + FMI_SPARE_FIFO1);
	((u32 *)priv->oob_buf)[2] = readl(priv->regs + FMI_SPARE_FIFO2);
	return 0;
}

/*
 * Trigger one short FIFO read cycle (up to 2 bytes) and copy the result.
 */
static int fmi_fifo_read_word(struct s5l8702_nand *priv, u8 *buf, unsigned int len) {
	u32 word;
	unsigned int i;
	int ret;

	if (WARN_ON(len == 0 || len > 2)) return -EINVAL;

	writel(FMI_ST_FIFO_DONE, priv->regs + FMI_STATUS);

	writel(len - 1, priv->regs + FMI_CHUNK_SIZE);
	writel(1u << priv->bank, priv->regs + FMI_CHUNK_TRIG);
	writel(FMI_CTRL1_FIFO_READ, priv->regs + FMI_CTRL1);

	ret = fmi_wait(priv, FMI_ST_FIFO_DONE);
	if (ret) return ret;

	// Short reads need a small settle delay before the FIFO is valid.
	udelay(200);

	word = readl(priv->regs + FMI_FIFO0);

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
			unsigned int i;
			// Full-page read: 4 × 512 B chunked DMA for data
			// (proven path), then a small DMA for 64 B OOB.
			for (i = 0; i < len / FMI_CHUNK_BYTES; i++) {
				ret = fmi_read_chunk(priv, i,
						     buf + i * FMI_CHUNK_BYTES);
				if (ret)
					break;
			}
			if (!ret && len == FMI_PAGE_BYTES)
				ret = fmi_read_oob(priv);
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

/*
 * Scan the operation looking for partial-page reads: ADDR with non-zero col,
 * followed by a DATA_IN smaller than a full page. The FMI can't do partial
 * reads, so we redirect these to a full-page read and return the slice.
 */
static bool is_oob_only_read(struct nand_chip *chip,
			     const struct nand_operation *op,
			     u8 **out_buf, unsigned int *out_len,
			     unsigned int *out_col)
{
	bool has_nonzero_col = false;
	unsigned int col = 0;
	unsigned int i;

	for (i = 0; i < op->ninstrs; i++) {
		const struct nand_op_instr *instr = &op->instrs[i];

		if (instr->type == NAND_OP_ADDR_INSTR && instr->ctx.addr.naddrs >= 2) {
			col = instr->ctx.addr.addrs[0] |
			      (instr->ctx.addr.addrs[1] << 8);
			if (col != 0)
				has_nonzero_col = true;
		}

		if (instr->type == NAND_OP_DATA_IN_INSTR && has_nonzero_col &&
		    instr->ctx.data.len < FMI_PAGE_BYTES) {
			*out_buf = instr->ctx.data.buf.in;
			*out_len = instr->ctx.data.len;
			*out_col = col;
			return true;
		}
	}
	return false;
}

static int s5l8702_nand_exec_op(struct nand_chip *chip, const struct nand_operation *op, bool check_only) {
	struct s5l8702_nand *priv = chip_to_priv(chip);
	unsigned int i;
	u8 *oob_buf;
	unsigned int oob_len, oob_col;
	int ret;

	if (check_only) return 0;

	priv->bank = priv->base_bank + op->cs;

	writel(FMI_CTRL1_RESET, priv->regs + FMI_CTRL1);
	writel(FMI_CTRL0_BASE | FMI_CTRL0_CS(priv->bank), priv->regs + FMI_CTRL0);

	// If this is an OOB-only read, redirect to a full-page read internally.
	if (is_oob_only_read(chip, op, &oob_buf, &oob_len, &oob_col)) {

		// Replay the operation but with a full-page DATA_IN to dma_virt.
		// We rebuild the CMD/ADDR/CMD/WAITRDY sequence, then do the full
		// chunk read which populates priv->oob_buf as a side effect.
		for (i = 0; i < op->ninstrs; i++) {
			const struct nand_op_instr *instr = &op->instrs[i];
			struct nand_op_instr fake;

			if (instr->type == NAND_OP_DATA_IN_INSTR) {
				// Substitute a full-page read
				fake = *instr;
				fake.ctx.data.buf.in = priv->dma_virt;
				fake.ctx.data.len = FMI_PAGE_BYTES;
				ret = s5l8702_exec_instr(priv, &fake);
			} else if (instr->type == NAND_OP_ADDR_INSTR) {
				// Override column to 0 so we read from page start
				struct nand_op_instr addr_fake = *instr;
				u8 addrs[8];
				memcpy(addrs, instr->ctx.addr.addrs,
				       instr->ctx.addr.naddrs);
				addrs[0] = 0;
				if (instr->ctx.addr.naddrs > 1) addrs[1] = 0;
				addr_fake.ctx.addr.addrs = addrs;
				ret = s5l8702_exec_instr(priv, &addr_fake);
			} else {
				ret = s5l8702_exec_instr(priv, instr);
			}
			if (ret) goto out_deselect;
		}

		// Copy captured OOB into caller's buffer
		memcpy(oob_buf, priv->oob_buf, oob_len);
		ret = 0;
		goto out_deselect;
	}

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
	struct mtd_info *mtd = nand_to_mtd(chip);

	// ECC is handled inside fmi_read_chunk by the controller's BCH engine.
	// From MTD's perspective we present as "no software ECC needed".
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
 */
static int s5l8702_nand_power_on(struct device *dev, struct i2c_adapter *adap) {
	u8 ctrl = 0;
	int ret;

	// Vnand voltage select (~3000 mV).
	ret = pmu_wr(adap, PMU_REG_VNAND, PMU_VNAND_3000MV);
	if (ret) return dev_err_probe(dev, ret, "PMU: Vnand write failed\n");

	// Vaccy / secondary rail.
	ret = pmu_wr(adap, PMU_REG_VACCY, PMU_VACCY_VAL);
	if (ret) return dev_err_probe(dev, ret, "PMU: Vaccy write failed\n");

	// RMW reg 0x10 sequence: keep most bits, set bit 3.
	ret = pmu_rd(adap, PMU_REG_LDO_CTRL, &ctrl);
	if (ret) return dev_err_probe(dev, ret, "PMU: LDO ctrl read failed\n");

	ret = pmu_wr(adap, PMU_REG_LDO_CTRL, (ctrl & PMU_LDO_CTRL_PRESERVE) | PMU_LDO_CTRL_NAND_BIT);
	if (ret) return dev_err_probe(dev, ret, "PMU: LDO ctrl write failed\n");

	// Force all LDOs on right before NAND I/O.
	ret = pmu_wr(adap, PMU_REG_LDO_CTRL, PMU_LDO_CTRL_ALL_ON);
	if (ret) return dev_err_probe(dev, ret, "PMU: LDO all-on write failed\n");

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

	// Ungate the NAND and NAND-ECC clocks (PWRCON0: clear = clocked).
	writel(readl(priv->pwrcon0) & ~(PWRCON0_NAND_BIT | PWRCON0_NAND_ECC_BIT), priv->pwrcon0);

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

	// DMA-coherent buffer: one full page (4 × 512 B).
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
