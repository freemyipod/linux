// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 dual PL080 DMA (N31)
 *
 * Bases: 0x38200000 / 0x38700000 (OSOS pair). Not 0x384 (DWC OTG).
 *
 * DT #dma-cells = <2>: <peri_id ccr_flags>
 * Quirks (PL080 + I²S on S5L8740/N31):
 *   Burst: M2P dest=1 beat (fixed IIS FIFO @+0x10); src≈4 beats (half FIFO).
 *   Peri: RetailOS oracle 2026-08-25 — IIS0 TX **10** (→0x3CA00010), IIS2 RX **13**
 *         (←0x3D400038); BT A2DP uses UART1 only (no PL080). Glass: peri 12 stuck;
 *         Rockbox IIS0 TX=0xA. See artifacts/retailos-mmio/README.md.
 *   Cache: PL080 not coherent — dma_sync in start(); no CTL_PROT_CACHE on slave.
 *   LLI: OSOS B424C uses **5×u32 / 20-byte** nodes (src,dst,lli,ctl,count).
 *        Count lives in LLI[4] and is programmed to CONTROL2 @+0x114.
 *   terminate_all: CFG disable + bounded ENBLD poll — never spin on BUSY (amba-pl08x).
 *   SG: multi-element builds LLI chain; contiguous buffers preferred (CMA).
 *   AHB: M2P src=mem on AHB2 (ahb_s=1), dst=FIFO on AHB1 (ahb_d=0).
 *   FIFO: S3C64xx-style ~64 deep — src burst 8 (m2p_src_burst=2), dst=1.
 *   PL080S/S5L: CTL @+0x10c, CFG @+0x110, CONTROL2 count @+0x114 (OSOS B424C).
 *        Not the mainline Samsung map (CONTROL2@+0x10 / CFG@+0x14).
 *   Cache: ARM1176 32-byte lines — buffer 32-byte aligned; sync in start().
 */
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/llist.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include <linux/apple-n31.h>
#include "virt-dma.h"

#define PL080_INT_STATUS	0x00
#define PL080_INT_TC_STATUS	0x04
#define PL080_INT_TC_CLEAR	0x08
#define PL080_INT_ERR_STATUS	0x0c
#define PL080_INT_ERR_CLEAR	0x10
#define PL080_RAW_TC		0x14
#define PL080_RAW_ERR		0x18
#define PL080_ENBLD_CHNS	0x1c
#define PL080_SOFT_BREQ		0x20
#define PL080_SOFT_SREQ		0x24
#define PL080_CONFIG		0x30
#define PL080_CONFIG_EN		BIT(0)
#define PL080_SYNC		0x34

#define PL080_Cx_SRC(i)		(0x100 + (i) * 0x20)
#define PL080_Cx_DST(i)		(0x104 + (i) * 0x20)
#define PL080_Cx_LLI(i)		(0x108 + (i) * 0x20)
#define PL080_Cx_CTL(i)		(0x10c + (i) * 0x20)
#define PL080_Cx_CFG(i)		(0x110 + (i) * 0x20)
/* OSOS B424C writes transfer count here (PL080S CONTROL2). Not in DDI0196. */
#define PL080S_Cx_CONTROL2(i)	(0x114 + (i) * 0x20)

#define PL080_CH_COUNT		8
#define PL080_MAX_XFER_WORDS	0xfff

#define CTL_SRC_AI		BIT(26)
#define CTL_DST_AI		BIT(27)
#define CTL_TC_IRQ		BIT(31)
#define CTL_WIDTH_SHIFT		18
#define CTL_DBSIZE_SHIFT	15
#define CTL_SBSIZE_SHIFT	12

#define CFG_ENABLE		BIT(0)
#define CFG_SRC_PERI_SHIFT	1
#define CFG_DST_PERI_SHIFT	6
#define CFG_FLOW_SHIFT		11
#define CFG_IE			BIT(14)	/* unmask error IRQ */
#define CFG_ITC			BIT(15)	/* unmask TC IRQ */
#define FLOW_M2P		0x1
#define FLOW_P2M		0x2
#define CTL_PROT_PRIV		BIT(28)
#define CTL_PROT_BUFF		BIT(29)
#define CTL_PROT_CACHE		BIT(30)

/* Glass: peri 12 Active+c2 stuck. peri 10 SRC walks. Rockbox IIS0 TX=0xA. */
static int force_peri = -1;
module_param(force_peri, int, 0644);
MODULE_PARM_DESC(force_peri, "override DT DMA peri id (-1 = use DT)");
/*
 * Put the transfer count in CxControl[11:0] as well as CONTROL2.
 *
 * Stock writes the count to the channel's +0x14 (CONTROL2) and to the 5th
 * dword of the LLI -- sub_B424C does "*v25 = v27 & 0x1FFFFFFF" and
 * "v21[4] = v27" -- which is the Samsung PL080S layout and is what this
 * driver does. But stock also *preserves* CxControl's low bits: it only
 * rewrites SI/DI, via "*v10 & 0xF3FFFFFF | v13 | v17". Whatever sits in
 * CxControl[11:0] was put there by something else and stock never clears
 * it. This driver rebuilds the word from scratch each time and leaves the
 * field zero.
 *
 * Measured: with force_mem=1 the channel waits on no peripheral request at
 * all and still stops after exactly 32 bytes, which rules out the I2S and
 * points at the descriptor.
 */
/*
 * DMACSync: leave the synchronisation logic ENABLED.
 *
 * On the PL080 a SET bit in DMACSync *disables* the synchronisation logic
 * for that DMA request line. That logic exists for peripherals whose
 * request signal is in a different clock domain from the DMA controller,
 * which IIS0 is: it runs off the audio clock, not the AHB clock. With
 * synchronisation off the request is sampled unreliably, and the channel
 * takes one burst and then never sees another request -- no error, no
 * terminal count.
 *
 * OSOS writes 0x38200034 zero times in the whole image, leaving it at its
 * reset value of 0 with synchronisation enabled for every line. This driver
 * does the same. sync_mask restores the old value for comparison.
 */
static unsigned int sync_mask;
module_param(sync_mask, uint, 0644);
MODULE_PARM_DESC(sync_mask,
		 "DMACSync value; 0 = sync enabled (stock). Old behaviour was ~0.");

static int ctl_count;
module_param(ctl_count, int, 0644);
MODULE_PARM_DESC(ctl_count,
		 "1 = also place the transfer count in CxControl[11:0]");

static int force_mem;
module_param(force_mem, int, 0644);
MODULE_PARM_DESC(force_mem, "1 = M2M flow + soft req, dest still FIFO");
/*
 * RetailOS music CFG = 0x28a81 (Active RO bit17 set mid-play → base 0x8a81):
 *   DstPeri=10, FlowCntrl=1 (M2P DMA), ITC=1, IE=0.
 * Earlier misread of Active as Flow=5; keep Flow=1 per PL080-DECODE.md.
 */
static int force_flow = 1;
module_param(force_flow, int, 0644);
MODULE_PARM_DESC(force_flow, "PL080 FlowCntrl -1=auto M2P, 0=M2M+soft, 1=M2P (RetailOS), 5=M2P-peri");
/* DDI0196 CxControl bits 24/25: 0=AHB1, 1=AHB2. Kitra memcpy uses AHB1. */
/*
 * RetailOS music-playing CTL = 0x84249000:
 *   Prot=0, SI=1, width=16, SB=1, DB=1, AHB_S=0, AHB_D=0, TC_IRQ=1.
 * Prior Linux defaults (ahb_s=1, SB=8, DB=1, Prot=PRIV|BUFF) yielded
 * CTL 0xb5242000 and STATUS stuck in 0x2A0 class vs retail 0x320.
 */
static int ahb_s;
module_param(ahb_s, int, 0644);
MODULE_PARM_DESC(ahb_s, "source AHB master (0=AHB1 RetailOS music, 1=AHB2)");
static int ahb_d;
module_param(ahb_d, int, 0644);
MODULE_PARM_DESC(ahb_d, "dest AHB master (0=AHB1/periph, 1=AHB2/mem)");
/* 1=16-bit S16 LE (Rockbox pcm / OSOS BCB60 16-bit). 2=32-bit packed LR. */
static int xfer_width = 1;
module_param(xfer_width, int, 0644);
MODULE_PARM_DESC(xfer_width, "PL080 src/dst width 0=8 1=16 2=32");
/* RetailOS music SBSIZE/DBSIZE enc = 1 (4-beat? enc1) — CTL 0x84249000. */
/*
 * Off by default: this is a ~160 character line on every DMA start, and
 * with the console on a 115200 UART that is milliseconds inside the audio
 * re-arm path. It is the line that made playback start-stop until printk
 * rate limiting silenced it.
 */
static bool start_verbose;
module_param(start_verbose, bool, 0644);
MODULE_PARM_DESC(start_verbose,
		 "Log channel CFG write and read-back at every start");

static bool xfer_width_override;
module_param(xfer_width_override, bool, 0644);
MODULE_PARM_DESC(xfer_width_override,
		 "Force xfer_width on every channel, ignoring dma_slave_config");

static int m2p_src_burst = 1;
module_param(m2p_src_burst, int, 0644);
MODULE_PARM_DESC(m2p_src_burst, "M2P SBSIZE enc (default 1 = RetailOS music)");
static int m2p_dst_burst = 1;
module_param(m2p_dst_burst, int, 0644);
MODULE_PARM_DESC(m2p_dst_burst, "M2P DBSIZE enc (default 1 = RetailOS music)");
/* 1=match RetailOS Prot=0 on slave; 0=PRIV|BUFF (old Linux). */
static int retail_prot = 1;
module_param(retail_prot, int, 0644);
MODULE_PARM_DESC(retail_prot, "1=Prot=0 on M2P/P2M (RetailOS music CTL)");
static int force_eng;
module_param(force_eng, int, 0644);
MODULE_PARM_DESC(force_eng, "PL080 engine 0/1 for xlate (-1 = either; default 0 = PL080_0)");
/* Prefer physical channel (RetailOS music uses ch2). -1 = first free. */
static int force_ch = 2;
module_param(force_ch, int, 0644);
MODULE_PARM_DESC(force_ch, "prefer PL080 channel id 0..7 (-1=any; default 2=RetailOS)");

/* Verbose transfer/xlate spam off by default; use verbose=1 or dyndbg. */
static bool verbose;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose, "Verbose PL080 DMA bring-up logs (default N)");

#define pl080_vinfo(dev, fmt, ...) \
	do { \
		if (verbose) \
			dev_info((dev), fmt, ##__VA_ARGS__); \
		else \
			dev_dbg((dev), fmt, ##__VA_ARGS__); \
	} while (0)

/* OSOS B424C descriptor stride is 20 bytes; keep pool 32-byte aligned. */
#define PL080_LLI_ALIGN		32
#define PL080_TERM_POLL_US	10
#define PL080_TERM_POLL_MAX	10
#define PL080S_XFER_COUNT_MASK	0x1fffffffu

struct pl080_lli {
	__le32 src;
	__le32 dst;
	__le32 lli;
	__le32 ctrl;
	__le32 ctrl2; /* transfer count → CONTROL2 (OSOS v27) */
};

static size_t s5l_pl080_lli_size(unsigned int nlli)
{
	return ALIGN(nlli * sizeof(struct pl080_lli), PL080_LLI_ALIGN);
}

static dma_addr_t s5l_pl080_lli_pa(dma_addr_t base, unsigned int idx)
{
	return base + idx * sizeof(struct pl080_lli);
}

#define PL080_LLI_POOL_NODES	64

struct s5l_pl080_chan {
	struct virt_dma_chan	vc;
	struct s5l_pl080	*host;
	void __iomem		*base;
	u8			id;
	u8			peri;
	u8			src_burst;
	u8			dst_burst;
	/*
	 * Encoded PL080 width (0 = 8-bit, 1 = 16, 2 = 32) taken from the
	 * slave config, or -1 when the client never set one.
	 */
	s8			src_wid;
	s8			dst_wid;
	enum dma_transfer_direction dir;
	dma_addr_t		fifo_addr;
	struct s5l_pl080_desc	*running;
	/*
	 * DMA errors seen on this channel, and whether the driver has stopped
	 * trying. Bounded because an error that re-arms itself is an
	 * interrupt storm, and a storm on this SoC is a dead device rather
	 * than a slow one.
	 */
	unsigned int		err_count;
	bool			err_stuck;
	/*
	 * Cyclic period callbacks are delivered from a workqueue, not from
	 * virt-dma's tasklet. See s5l_pl080_cyc_workfn().
	 */
	struct work_struct	cyc_work;
	atomic_t		cyc_pending;
	dma_async_tx_callback	cyc_cb;
	void			*cyc_cb_param;
	bool			cyc_active;
};

struct s5l_pl080_desc {
	struct virt_dma_desc	vd;
	struct pl080_lli	*lli;
	dma_addr_t		lli_phys;
	unsigned int		nlli;
	unsigned int		lli_off;
	bool			lli_from_pool;
	u32			cfg;
	bool			cyclic;
	dma_addr_t		buf_addr;
	size_t			buf_len;
	size_t			period_len;
	unsigned int		periods;
	unsigned int		periods_done;
	/*
	 * Ring mode for the single self-linked node: the driver advances
	 * lli[0].src itself, from the terminal-count interrupt, instead of
	 * having a consumer post each period from process context.
	 * See s5l_pl080_rearm_set_ring().
	 */
	bool			ring;
	u32			ring_base;
	u32			ring_bytes;
	u32			ring_period;
	u32			ring_off;
	struct llist_node	free_node;
};

struct s5l_pl080 {
	struct device		*dev;
	void __iomem		*base[2];
	struct clk		*clk[2];
	struct dma_device	ddev;
	struct s5l_pl080_chan	chans[PL080_CH_COUNT * 2];
	spinlock_t		lock;
	void			*dummy_cpu;
	dma_addr_t		dummy_dma;
	struct pl080_lli	*lli_pool;
	dma_addr_t		lli_pool_phys;
	DECLARE_BITMAP(lli_busy, PL080_LLI_POOL_NODES);
	struct task_struct	*pump;
	/* Descriptors whose coherent LLI block must be freed outside
	 * atomic context; see s5l_pl080_desc_free().
	 */
	struct llist_head	free_list;
	struct work_struct	free_work;
	struct workqueue_struct	*cyc_wq;
};

static struct pl080_lli *s5l_pl080_lli_alloc(struct s5l_pl080 *pl,
					     unsigned int nlli,
					     dma_addr_t *phys,
					     unsigned int *off,
					     bool *from_pool)
{
	unsigned long flags;
	unsigned int i, j;
	struct pl080_lli *lli;

	if (!pl || !nlli || !phys || !off || !from_pool)
		return NULL;

	if (pl->lli_pool && nlli <= PL080_LLI_POOL_NODES) {
		spin_lock_irqsave(&pl->lock, flags);
		for (i = 0; i + nlli <= PL080_LLI_POOL_NODES; i++) {
			for (j = 0; j < nlli; j++) {
				if (test_bit(i + j, pl->lli_busy))
					break;
			}
			if (j != nlli)
				continue;
			for (j = 0; j < nlli; j++)
				set_bit(i + j, pl->lli_busy);
			*phys = pl->lli_pool_phys +
				i * sizeof(struct pl080_lli);
			*off = i;
			*from_pool = true;
			spin_unlock_irqrestore(&pl->lock, flags);
			return pl->lli_pool + i;
		}
		spin_unlock_irqrestore(&pl->lock, flags);
	}

	lli = dma_alloc_coherent(pl->dev, s5l_pl080_lli_size(nlli), phys,
				 GFP_NOWAIT);
	if (!lli) {
		dev_warn_ratelimited(pl->dev,
				     "LLI alloc nlli=%u ENOMEM\n", nlli);
		return NULL;
	}
	*off = 0;
	*from_pool = false;
	return lli;
}

/* Returning pool slots is just clearing bits, so any context will do. */
static void s5l_pl080_lli_pool_release(struct s5l_pl080 *pl,
				       struct s5l_pl080_desc *d)
{
	unsigned long flags;
	unsigned int j;

	spin_lock_irqsave(&pl->lock, flags);
	for (j = 0; j < d->nlli && d->lli_off + j < PL080_LLI_POOL_NODES; j++)
		clear_bit(d->lli_off + j, pl->lli_busy);
	spin_unlock_irqrestore(&pl->lock, flags);
	d->lli = NULL;
}

/* Drains descriptors parked by s5l_pl080_desc_free(). */
static void s5l_pl080_free_work(struct work_struct *work)
{
	struct s5l_pl080 *pl = container_of(work, struct s5l_pl080,
					    free_work);
	struct s5l_pl080_desc *d, *tmp;
	struct llist_node *pending;

	pending = llist_del_all(&pl->free_list);
	llist_for_each_entry_safe(d, tmp, pending, free_node) {
		dma_free_coherent(pl->dev, s5l_pl080_lli_size(d->nlli),
				  d->lli, d->lli_phys);
		kfree(d);
	}
}

static int s5l_pl080_need_soft(void)
{
	/* Flow 0/4: M2M or M2P under DMA control — drive with SOFT_BREQ. */
	return force_mem || force_flow == 0 || force_flow == 4;
}

static unsigned int s5l_pl080_burst_enc(unsigned int maxburst)
{
	if (maxburst <= 1)
		return 0;
	if (maxburst <= 4)
		return 1;
	if (maxburst <= 8)
		return 2;
	if (maxburst <= 16)
		return 3;
	if (maxburst <= 32)
		return 4;
	if (maxburst <= 64)
		return 5;
	if (maxburst <= 128)
		return 6;
	return 7;
}

static int s5l_pl080_pump(void *data)
{
	struct s5l_pl080 *pl = data;
	unsigned int i, n;

	while (!kthread_should_stop()) {
		if (!s5l_pl080_need_soft()) {
			usleep_range(20000, 40000);
			continue;
		}
		n = 0;
		for (i = 0; i < PL080_CH_COUNT * 2; i++) {
			struct s5l_pl080_chan *ch = &pl->chans[i];

			if (!ch->base || !ch->running)
				continue;
			writel(BIT(ch->id % PL080_CH_COUNT),
			       ch->base + PL080_SOFT_BREQ);
			n++;
		}
		if (!n)
			usleep_range(2000, 4000);
		else
			cond_resched();
	}
	return 0;
}

static struct s5l_pl080_chan *to_s5l_chan(struct dma_chan *c)
{
	return container_of(c, struct s5l_pl080_chan, vc.chan);
}

static struct s5l_pl080_desc *to_s5l_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct s5l_pl080_desc, vd);
}

/*
 * PL080 encodes transfer width as log2(bytes): 0 = 8-bit, 1 = 16-bit,
 * 2 = 32-bit. Anything wider than 32-bit has no encoding here.
 */
static int s5l_pl080_width_enc(enum dma_slave_buswidth w)
{
	switch (w) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		return 0;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		return 1;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		return 2;
	default:
		return -1;
	}
}

/*
 * Width actually used for a channel. A client's dma_slave_config wins;
 * xfer_width is the fallback and the bring-up override. Ignoring the slave
 * config here silently narrows a client that asked for 32-bit FIFO writes.
 */
static unsigned int s5l_pl080_chan_width(struct s5l_pl080_chan *ch,
					 bool dst)
{
	unsigned int w = xfer_width & 7;
	s8 cfgw = -1;

	if (ch)
		cfgw = dst ? ch->dst_wid : ch->src_wid;
	if (!xfer_width_override && cfgw >= 0)
		w = (unsigned int)cfgw;
	if (w > 2)
		w = 1;
	return w;
}

static unsigned int s5l_pl080_unit(void)
{
	unsigned int w = xfer_width & 7;

	if (w > 2)
		w = 1;
	return 1u << w;
}

/*
 * CTL template only — transfer count is NOT in CTL[11:0] on this SoC.
 * OSOS B424C / RetailOS music CTL (e.g. 0x84249000) keep size bits clear;
 * count is written to CONTROL2 and LLI ctrl2.
 */
static u32 s5l_pl080_build_ctl(struct s5l_pl080_chan *ch, u32 words,
			      bool src_inc, bool dst_inc, bool irq)
{
	unsigned int sw = s5l_pl080_chan_width(ch, false);
	unsigned int dw = s5l_pl080_chan_width(ch, true);
	unsigned int sb, db;
	u32 ctl;

	if (ch && (ch->dir == DMA_MEM_TO_DEV || ch->dir == DMA_DEV_TO_MEM)) {
		sb = ch->src_burst;
		db = ch->dst_burst;
		ctl = retail_prot ? 0 : (CTL_PROT_PRIV | CTL_PROT_BUFF);
	} else {
		/* M2M selftest / memcpy: Rockbox pcm-s5l8702 8/4 */
		sb = 2;
		db = 1;
		ctl = CTL_PROT_PRIV | CTL_PROT_BUFF | CTL_PROT_CACHE;
	}
	/* SWIDTH is the low field, DWIDTH sits three bits above it. */
	ctl |= (sw << CTL_WIDTH_SHIFT) | (dw << (CTL_WIDTH_SHIFT + 3)) |
	      (sb << CTL_SBSIZE_SHIFT) | (db << CTL_DBSIZE_SHIFT);
	if (ahb_s)
		ctl |= BIT(24);
	if (ahb_d)
		ctl |= BIT(25);

	if (src_inc)
		ctl |= CTL_SRC_AI;
	if (dst_inc)
		ctl |= CTL_DST_AI;
	if (irq)
		ctl |= CTL_TC_IRQ;
	if (ctl_count)
		ctl |= words & 0xfff;
	return ctl;
}

static void s5l_pl080_chan_disable(struct s5l_pl080_chan *ch)
{
	u8 id = ch->id % PL080_CH_COUNT;
	void __iomem *b = ch->base;
	unsigned int i;
	u32 en;

	if (!b)
		return;
	writel(0, b + PL080_Cx_CFG(id));
	/* Never spin forever on BUSY — bounded poll then force-clear IRQs. */
	for (i = 0; i < PL080_TERM_POLL_MAX; i++) {
		en = readl(b + PL080_ENBLD_CHNS);
		if (!(en & BIT(id)))
			break;
		udelay(PL080_TERM_POLL_US);
	}
	if (en & BIT(id))
		dev_warn(ch->host->dev,
			 "ch%u still enabled en=0x%x after disable (no BUSY wait)\n",
			 ch->id, en);
	writel(BIT(id), b + PL080_INT_TC_CLEAR);
	writel(BIT(id), b + PL080_INT_ERR_CLEAR);
}

/* SRAM window: 0x22000000..0x2202FFFF. */
/*
 * Stock programs Cx_LLI with bit 31 set.
 *
 * Measured on RetailOS with music playing: ch2 lli = 0x8b353be0. DRAM on
 * this board is 0x08000000..0x0BFFFFFF (memory@8000000, 64 MiB), so that is
 * 0x0b353be0 with bit 31 set -- an alias of a perfectly ordinary DRAM
 * address, not a different region. A bare 0x09588000 is not equivalent.
 *
 * Note stock keeps the audio BUFFER in SRAM but the LLI descriptor in DRAM,
 * through this alias.
 *
 * What bit 31 selects is NOT established -- an uncached view, or a
 * different AHB master, are both plausible and neither is proven. It is
 * applied because stock applies it. One define to revert.
 */
#define PL080_LLI_ALIAS		0x80000000u


#define S5L8740_SRAM_BASE	0x22000000u
#define S5L8740_SRAM_END	0x22030000u

static void s5l_pl080_sync_buffer(struct s5l_pl080_chan *ch,
				  struct s5l_pl080_desc *d)
{
	struct device *dev = ch->host->dev;

	if (!d->buf_len || !dev)
		return;

	/*
	 * Never do cache maintenance on an SRAM buffer.
	 *
	 * The audio PCM buffer now lives in SRAM, where stock keeps it
	 * (measured PL080 ch2 src=0x220025d0 on RetailOS while playing). That
	 * region is declared no-map, so it is not in the kernel linear map and
	 * dma_sync_single_for_device() reaches v7_dma_clean_range() on an
	 * address that has no cacheable mapping:
	 *
	 *	Unable to handle kernel paging request at virtual address da020000
	 *	PC is at v7_dma_clean_range+0x1c/0x34
	 *	LR is at arch_sync_dma_for_device+0x54/0xa8
	 *	arch_sync_dma_for_device from s5l_pl080_start
	 *
	 * There is nothing to maintain in any case: SRAM is not cached, so
	 * writes from the CPU are already visible to the controller. The sync
	 * exists for DRAM buffers, where this engine is not coherent.
	 */
	if (d->buf_addr >= S5L8740_SRAM_BASE && d->buf_addr < S5L8740_SRAM_END)
		return;
	if (ch->dir == DMA_MEM_TO_DEV)
		dma_sync_single_for_device(dev, d->buf_addr, d->buf_len,
					   DMA_TO_DEVICE);
	else if (ch->dir == DMA_DEV_TO_MEM)
		dma_sync_single_for_device(dev, d->buf_addr, d->buf_len,
					   DMA_FROM_DEVICE);
}

/*
 * Cyclic period callbacks, delivered from process context.
 *
 * virt-dma hands descriptor callbacks to a tasklet. For a cyclic audio
 * stream that callback is dmaengine_pcm_dma_complete(), which calls
 * snd_pcm_period_elapsed(), which takes the PCM stream lock -- and both DAI
 * links on this board are nonatomic, so that lock is a mutex, not a
 * spinlock.
 *
 * A mutex_lock() in a tasklet is not reliably fatal, which is what made
 * this so slow to find. Uncontended it takes the cmpxchg fast path and
 * returns without sleeping, so short playback looks fine. It is only when
 * the writer thread is actually holding the stream lock -- that is, during
 * sustained playback -- that the tasklet tries to sleep, calls schedule()
 * from atomic context, and takes the machine down. Playback that runs for
 * a moment and then hangs the whole device is exactly that shape.
 *
 * So the callback is queued to a workqueue instead. The pending count is
 * kept exactly, never coalesced: dmaengine_pcm_dma_complete() advances its
 * own position by one period per call, so swallowing a call would desync
 * the ALSA pointer from the hardware just as surely as never calling it.
 */
static void s5l_pl080_destroy_wq(void *wq)
{
	destroy_workqueue(wq);
}

static void s5l_pl080_cyc_workfn(struct work_struct *w)
{
	struct s5l_pl080_chan *ch = container_of(w, struct s5l_pl080_chan,
						 cyc_work);
	unsigned long flags;

	while (atomic_dec_if_positive(&ch->cyc_pending) >= 0) {
		dma_async_tx_callback cb;
		void *param;

		spin_lock_irqsave(&ch->vc.lock, flags);
		cb = ch->cyc_active ? ch->cyc_cb : NULL;
		param = ch->cyc_cb_param;
		spin_unlock_irqrestore(&ch->vc.lock, flags);

		if (!cb)
			break;
		cb(param);
	}
}

/*
 * Drop any callback still owed for this channel.
 *
 * Callers hold vc.lock, so this must not wait for a callback already in
 * flight. Clearing cyc_active is what stops the work function dead: it
 * re-reads the flag under the lock before every single invocation, so once
 * this returns no further callback can start. One already past that check
 * may still run, which is what device_synchronize() is for.
 */
static void s5l_pl080_cyc_drop(struct s5l_pl080_chan *ch)
{
	ch->cyc_active = false;
	ch->cyc_cb = NULL;
	ch->cyc_cb_param = NULL;
	atomic_set(&ch->cyc_pending, 0);
}

static void s5l_pl080_start(struct s5l_pl080_chan *ch, struct s5l_pl080_desc *d)
{
	void __iomem *b = ch->base;

	/* A channel the error path gave up on stays down until it is
	 * reconfigured; restarting it just resumes the storm.
	 */
	if (ch->err_stuck) {
		dev_warn_ratelimited(ch->host->dev,
				     "ch%u start refused: %u DMA errors\n",
				     ch->id, ch->err_count);
		return;
	}
	u8 id = ch->id % PL080_CH_COUNT;
	struct pl080_lli *first = d->lli;

	s5l_pl080_sync_buffer(ch, d);
	s5l_pl080_chan_disable(ch);
	s5l_pl080_cyc_drop(ch);
	if (d->cyclic) {
		ch->cyc_cb = d->vd.tx.callback;
		ch->cyc_cb_param = d->vd.tx.callback_param;
		ch->cyc_active = true;
	}
	writel(le32_to_cpu(first->src), b + PL080_Cx_SRC(id));
	writel(le32_to_cpu(first->dst), b + PL080_Cx_DST(id));
	/* Next LLI, not the first (already loaded into SRC/DST/CTL/C2). */
	writel(le32_to_cpu(first->lli) | PL080_LLI_ALIAS,
	       b + PL080_Cx_LLI(id));
	writel(le32_to_cpu(first->ctrl), b + PL080_Cx_CTL(id));
	/* B424C: *v25 = v27 & 0x1FFFFFFF — count only, never the CTL word. */
	writel(le32_to_cpu(first->ctrl2) & PL080S_XFER_COUNT_MASK,
	       b + PL080S_Cx_CONTROL2(id));
	writel(d->cfg | CFG_ENABLE, b + PL080_Cx_CFG(id));
	/*
	 * Read CFG straight back. If the enable does not stick, the channel
	 * never starts and every later symptom is downstream noise, so this
	 * distinguishes "never programmed" from "programmed then halted".
	 */
	if (start_verbose) {
		u32 rb = readl(b + PL080_Cx_CFG(id));

		dev_info(ch->host->dev,
			 "ch%u START peri=%u dir=%d cfg_want=0x%08x cfg_read=0x%08x en=0x%x ctl=0x%08x c2=0x%08x src=0x%08x dst=0x%08x\n",
			 ch->id, ch->peri, (int)ch->dir,
			 (u32)(d->cfg | CFG_ENABLE), rb,
			 readl(b + PL080_ENBLD_CHNS),
			 le32_to_cpu(first->ctrl),
			 le32_to_cpu(first->ctrl2),
			 le32_to_cpu(first->src),
			 le32_to_cpu(first->dst));
	}
	/* M2M / force_flow 0|4: software request. M2P peri waits for IIS DRQ. */
	if (s5l_pl080_need_soft()) {
		writel(BIT(id), b + PL080_SOFT_BREQ);
		if (force_mem || force_flow == 0)
			writel(BIT(id), b + PL080_SOFT_SREQ);
	}
	ch->running = d;
	dev_dbg(ch->host->dev,
		"ch%u start peri=%u cfg=0x%x nlli=%u src=0x%x dst=0x%x ctl=0x%x c2=0x%x\n",
		 ch->id, ch->peri, (u32)(d->cfg | CFG_ENABLE), d->nlli,
		 le32_to_cpu(first->src), le32_to_cpu(first->dst),
		 le32_to_cpu(first->ctrl), le32_to_cpu(first->ctrl2));
}

static void s5l_pl080_issue(struct dma_chan *c)
{
	struct s5l_pl080_chan *ch = to_s5l_chan(c);
	struct virt_dma_desc *vd;
	unsigned long flags;

	spin_lock_irqsave(&ch->vc.lock, flags);
	if (vchan_issue_pending(&ch->vc) && !ch->running) {
		vd = vchan_next_desc(&ch->vc);
		if (vd) {
			list_del(&vd->node);
			s5l_pl080_start(ch, to_s5l_desc(vd));
		}
	}
	spin_unlock_irqrestore(&ch->vc.lock, flags);
}

static enum dma_status s5l_pl080_tx_status(struct dma_chan *c,
					   dma_cookie_t cookie,
					   struct dma_tx_state *state)
{
	struct s5l_pl080_chan *ch = to_s5l_chan(c);
	enum dma_status st = dma_cookie_status(c, cookie, state);
	struct s5l_pl080_desc *d;
	unsigned long flags;
	u32 cur, start, end;

	if (!state || st == DMA_COMPLETE)
		return st;

	spin_lock_irqsave(&ch->vc.lock, flags);
	d = ch->running;
	if (d && d->buf_len) {
		u8 id = ch->id % PL080_CH_COUNT;
		u32 link, cnt, link2, cnt2;

		/*
		 * Derive the position from LINK and COUNT, never from SRCADDR.
		 *
		 * Rockbox's PL080 driver for this controller family documents
		 * the behaviour of reading the channel registers while the
		 * hardware is updating them: SRCADDR may return corrupted
		 * data, while LINK and COUNT always read back valid, and the
		 * pair is updated atomically. So read LINK, COUNT, LINK, COUNT
		 * and take the second pair when the link moved underneath the read.
		 *
		 * An earlier version of this function read SRCADDR, which is
		 * exactly the register that is not safe to trust here.
		 */
		link = readl(ch->base + PL080_Cx_LLI(id)) & ~PL080_LLI_ALIAS;
		cnt = readl(ch->base + PL080S_Cx_CONTROL2(id)) &
			PL080S_XFER_COUNT_MASK;
		link2 = readl(ch->base + PL080_Cx_LLI(id)) & ~PL080_LLI_ALIAS;
		cnt2 = readl(ch->base + PL080S_Cx_CONTROL2(id)) &
			PL080S_XFER_COUNT_MASK;
		if (link != link2) {
			link = link2;
			cnt = cnt2;
		}

		if (d->cyclic && d->nlli && d->lli && d->lli_phys &&
		    link >= lower_32_bits(d->lli_phys)) {
			u32 nodes = d->nlli;
			u32 next = (link - lower_32_bits(d->lli_phys)) /
				   sizeof(struct pl080_lli);
			u32 cur = (next + nodes - 1) % nodes;
			u32 per_node = d->buf_len / nodes;
			u32 words = le32_to_cpu(d->lli[cur].ctrl2) &
				    PL080S_XFER_COUNT_MASK;
			u32 pos = cur * per_node;

			/*
			 * LINK points at the descriptor the hardware will load
			 * next, so the one in flight is the previous node.
			 */
			if (words && cnt <= words)
				pos += ((words - cnt) * per_node) / words;
			if (pos < d->buf_len)
				state->residue = d->buf_len - pos;
			else
				state->residue = d->buf_len;
		} else {
			cur = readl(ch->base +
				    ((ch->dir == DMA_DEV_TO_MEM) ?
				     PL080_Cx_DST(id) : PL080_Cx_SRC(id)));
			start = lower_32_bits(d->buf_addr);
			end = start + d->buf_len;
			if (cur >= start && cur < end)
				state->residue = end - cur;
			else
				state->residue = d->buf_len;
		}
	}
	spin_unlock_irqrestore(&ch->vc.lock, flags);
	return st;
}

static int s5l_pl080_alloc(struct dma_chan *c)
{
	return 0;
}

/*
 * Wait for a callback that was already past the cyc_active check when
 * terminate ran. ALSA calls this via snd_pcm_sync_stop(), which is the
 * point at which it is safe to sleep.
 *
 * The current_work() test is not defensive programming, it is the whole
 * reason this function has a guard. The period callback is
 * snd_pcm_period_elapsed(), and an xrun detected inside it walks straight
 * back down into the driver:
 *
 *   s5l_pl080_cyc_workfn
 *     snd_pcm_period_elapsed -> snd_pcm_update_state -> snd_pcm_do_stop
 *       s5l8740_i2s_trigger(STOP) -> s5l8740_i2s_hw_stop
 *         dmaengine_synchronize -> cancel_work_sync(&ch->cyc_work)
 *
 * which is this work item waiting for itself to finish. It hangs the
 * worker, and then everything that touches the PCM -- the player, the
 * launcher reading /proc/asound, anything reading a status file -- piles
 * up behind the stream mutex the dead worker still holds. The device looks
 * exactly as bricked as a real lockup.
 *
 * Inside the callback there is nothing to wait for: the caller is the
 * thing synchronize would have waited on.
 */
static void s5l_pl080_synchronize(struct dma_chan *c)
{
	struct s5l_pl080_chan *ch = to_s5l_chan(c);

	if (current_work() != &ch->cyc_work)
		cancel_work_sync(&ch->cyc_work);
	vchan_synchronize(&ch->vc);
}

static void s5l_pl080_free(struct dma_chan *c)
{
	vchan_free_chan_resources(&to_s5l_chan(c)->vc);
}

/*
 * virt-dma calls this from its tasklet, and callers may hold the channel
 * lock, so it must not sleep. dma_free_coherent() can, so descriptors
 * holding a coherent LLI block are queued for s5l_pl080_free_work()
 * instead. Pool-backed descriptors are released inline.
 */
static void s5l_pl080_desc_free(struct virt_dma_desc *vd)
{
	struct s5l_pl080_desc *d = to_s5l_desc(vd);
	struct s5l_pl080 *pl = to_s5l_chan(vd->tx.chan)->host;

	if (!pl || !d->lli) {
		kfree(d);
		return;
	}
	if (d->lli_from_pool) {
		s5l_pl080_lli_pool_release(pl, d);
		kfree(d);
		return;
	}
	llist_add(&d->free_node, &pl->free_list);
	schedule_work(&pl->free_work);
}

static struct dma_async_tx_descriptor *
s5l_pl080_prep_slave_sg(struct dma_chan *c, struct scatterlist *sgl,
			unsigned int sg_len,
			enum dma_transfer_direction dir,
			unsigned long flags, void *context)
{
	struct s5l_pl080_chan *ch = to_s5l_chan(c);
	struct s5l_pl080_desc *d;
	struct scatterlist *sg;
	struct pl080_lli *lli;
	dma_addr_t lli_phys, dev_addr;
	unsigned int i, nlli = 0, total = 0;
	u32 cfg;

	for_each_sg(sgl, sg, sg_len, i)
		total += sg_dma_len(sg);
	if (!total)
		return NULL;
	if (sg_len > 1)
		pl080_vinfo(ch->host->dev, "prep_slave_sg sg_len=%u (LLI chain)\n",
			 sg_len);

	/* One LLI node per <= PL080_MAX_XFER_WORDS transfer units */
	nlli = DIV_ROUND_UP(total, PL080_MAX_XFER_WORDS * s5l_pl080_unit());
	if (nlli == 0)
		return NULL;

	d = kzalloc(sizeof(*d), GFP_NOWAIT);
	if (!d)
		return NULL;

	lli = s5l_pl080_lli_alloc(ch->host, nlli, &lli_phys, &d->lli_off,
				  &d->lli_from_pool);
	if (!lli) {
		kfree(d);
		return NULL;
	}

	cfg = 0;
	dev_addr = ch->fifo_addr;
	if (force_flow >= 0) {
		/* Retail music Flow=1 → CFG 0x8a81 (ITC only; Active RO adds 0x20000). */
		cfg |= ((force_flow & 7) << CFG_FLOW_SHIFT) | CFG_ITC;
		if (force_flow != 1 && force_flow != 5)
			cfg |= CFG_IE;
		if (dir == DMA_MEM_TO_DEV)
			cfg |= (ch->peri & 0x1f) << CFG_DST_PERI_SHIFT;
		else
			cfg |= (ch->peri & 0x1f) << CFG_SRC_PERI_SHIFT;
	} else if (force_mem) {
		cfg |= CFG_IE | CFG_ITC;
	} else if (dir == DMA_MEM_TO_DEV) {
		cfg |= (FLOW_M2P << CFG_FLOW_SHIFT) |
		       ((ch->peri & 0x1f) << CFG_DST_PERI_SHIFT) |
		       CFG_IE | CFG_ITC;
	} else {
		cfg |= (FLOW_P2M << CFG_FLOW_SHIFT) |
		       ((ch->peri & 0x1f) << CFG_SRC_PERI_SHIFT) |
		       CFG_IE | CFG_ITC;
	}

	{
		unsigned int idx = 0;
		size_t remaining = total;

		for_each_sg(sgl, sg, sg_len, i) {
			dma_addr_t addr = sg_dma_address(sg);
			size_t sg_left = sg_dma_len(sg);
			size_t sg_off = 0;

			while (sg_left && idx < nlli) {
				size_t unit = s5l_pl080_unit();
				size_t chunk = min_t(size_t, sg_left,
						     PL080_MAX_XFER_WORDS * unit);
				u32 words = chunk / unit;

				if (!words)
					words = 1;
				chunk = words * unit;

				if (dir == DMA_MEM_TO_DEV) {
					lli[idx].src = cpu_to_le32(
						lower_32_bits(addr + sg_off));
					lli[idx].dst = cpu_to_le32(
						lower_32_bits(dev_addr));
				} else {
					lli[idx].src = cpu_to_le32(
						lower_32_bits(dev_addr));
					lli[idx].dst = cpu_to_le32(
						lower_32_bits(addr + sg_off));
				}

				lli[idx].ctrl = cpu_to_le32(s5l_pl080_build_ctl(
					ch, words,
					dir == DMA_MEM_TO_DEV,
					dir == DMA_DEV_TO_MEM,
					idx == nlli - 1));
				lli[idx].ctrl2 = cpu_to_le32(words &
							     PL080S_XFER_COUNT_MASK);

				if (idx < nlli - 1)
					lli[idx].lli = cpu_to_le32(PL080_LLI_ALIAS |
						lower_32_bits(s5l_pl080_lli_pa(
							lli_phys, idx + 1)));
				else
					lli[idx].lli = cpu_to_le32(0);

				sg_off += chunk;
				sg_left -= chunk;
				remaining -= chunk;
				idx++;
			}
		}
	}

	d->lli = lli;
	d->lli_phys = lli_phys;
	d->nlli = nlli;
	d->cfg = cfg;
	d->cyclic = false;
	d->buf_addr = sg_dma_address(sgl);
	d->buf_len = total;
	return vchan_tx_prep(&ch->vc, &d->vd, flags);
}

static struct dma_async_tx_descriptor *
s5l_pl080_prep_dma_cyclic(struct dma_chan *c, dma_addr_t buf_addr,
			  size_t buf_len, size_t period_len,
			  enum dma_transfer_direction dir,
			  unsigned long flags)
{
	struct s5l_pl080_chan *ch = to_s5l_chan(c);
	struct s5l_pl080_desc *d;
	struct pl080_lli *lli;
	dma_addr_t lli_phys, dev_addr;
	unsigned int periods, per_period, nlli, p, idx;
	u32 cfg;

	if (!buf_len || !period_len || buf_len % period_len) {
		pl080_vinfo(ch->host->dev,
			 "cyclic reject len=%zu period=%zu\n", buf_len, period_len);
		return NULL;
	}

	periods = buf_len / period_len;
	per_period = DIV_ROUND_UP(period_len, PL080_MAX_XFER_WORDS * s5l_pl080_unit());
	if (!per_period)
		return NULL;
	nlli = periods * per_period;

	d = kzalloc(sizeof(*d), GFP_NOWAIT);
	if (!d)
		return NULL;

	lli = s5l_pl080_lli_alloc(ch->host, nlli, &lli_phys, &d->lli_off,
				  &d->lli_from_pool);
	if (!lli) {
		kfree(d);
		return NULL;
	}

	cfg = 0;
	dev_addr = ch->fifo_addr;
	if (force_flow >= 0) {
		/* Retail music Flow=1 → CFG 0x8a81 (ITC only; Active RO adds 0x20000). */
		cfg |= ((force_flow & 7) << CFG_FLOW_SHIFT) | CFG_ITC;
		if (force_flow != 1 && force_flow != 5)
			cfg |= CFG_IE;
		if (dir == DMA_MEM_TO_DEV)
			cfg |= (ch->peri & 0x1f) << CFG_DST_PERI_SHIFT;
		else
			cfg |= (ch->peri & 0x1f) << CFG_SRC_PERI_SHIFT;
	} else if (force_mem) {
		cfg |= CFG_IE | CFG_ITC;
	} else if (dir == DMA_MEM_TO_DEV) {
		cfg |= (FLOW_M2P << CFG_FLOW_SHIFT) |
		       ((ch->peri & 0x1f) << CFG_DST_PERI_SHIFT) |
		       CFG_IE | CFG_ITC;
	} else {
		cfg |= (FLOW_P2M << CFG_FLOW_SHIFT) |
		       ((ch->peri & 0x1f) << CFG_SRC_PERI_SHIFT) |
		       CFG_IE | CFG_ITC;
	}

	idx = 0;
	for (p = 0; p < periods; p++) {
		size_t sg_off = 0;
		size_t sg_left = period_len;
		unsigned int chunk_i;

		for (chunk_i = 0; chunk_i < per_period && sg_left; chunk_i++) {
			size_t unit = s5l_pl080_unit();
			size_t chunk = min_t(size_t, sg_left,
					     PL080_MAX_XFER_WORDS * unit);
			u32 words = chunk / unit;
			bool period_last;

			if (!words)
				words = 1;
			chunk = words * unit;
			period_last = (chunk_i == per_period - 1) ||
				      (sg_left <= chunk);

			if (dir == DMA_MEM_TO_DEV) {
				lli[idx].src = cpu_to_le32(lower_32_bits(
					buf_addr + p * period_len + sg_off));
				lli[idx].dst = cpu_to_le32(
					lower_32_bits(dev_addr));
			} else {
				lli[idx].src = cpu_to_le32(
					lower_32_bits(dev_addr));
				lli[idx].dst = cpu_to_le32(lower_32_bits(
					buf_addr + p * period_len + sg_off));
			}

			lli[idx].ctrl = cpu_to_le32(s5l_pl080_build_ctl(
				ch, words,
				dir == DMA_MEM_TO_DEV,
				dir == DMA_DEV_TO_MEM,
				period_last));
			lli[idx].ctrl2 = cpu_to_le32(words &
						     PL080S_XFER_COUNT_MASK);

			if (idx + 1 < nlli)
				lli[idx].lli = cpu_to_le32(PL080_LLI_ALIAS |
							   lower_32_bits(
					s5l_pl080_lli_pa(lli_phys, idx + 1)));
			else
				lli[idx].lli = cpu_to_le32(PL080_LLI_ALIAS |
					lower_32_bits(lli_phys));

			sg_off += chunk;
			sg_left -= chunk;
			idx++;
		}
	}
	if (idx)
		lli[idx - 1].lli = cpu_to_le32(PL080_LLI_ALIAS |
					       lower_32_bits(lli_phys));

	d->lli = lli;
	d->lli_phys = lli_phys;
	d->nlli = nlli;
	d->cfg = cfg;
	d->cyclic = true;
	d->buf_addr = buf_addr;
	d->buf_len = buf_len;
	d->period_len = period_len;
	d->periods = periods;
	d->periods_done = 0;
	pl080_vinfo(ch->host->dev,
		 "cyclic ok peri=%u nlli=%u periods=%u period=%zu fifo=0x%x\n",
		 ch->peri, nlli, periods, period_len,
		 (u32)lower_32_bits(dev_addr));
	return vchan_tx_prep(&ch->vc, &d->vd, flags);
}

static int s5l_pl080_config(struct dma_chan *c,
			    struct dma_slave_config *cfg)
{
	struct s5l_pl080_chan *ch = to_s5l_chan(c);

	ch->src_wid = s5l_pl080_width_enc(cfg->src_addr_width);
	ch->dst_wid = s5l_pl080_width_enc(cfg->dst_addr_width);

	if (cfg->direction == DMA_MEM_TO_DEV) {
		ch->fifo_addr = cfg->dst_addr;
		/*
		 * Memory side is read linearly, so if the client only
		 * described the device side, match it rather than leaving
		 * the source at the module default.
		 */
		if (ch->src_wid < 0)
			ch->src_wid = ch->dst_wid;
		/*
		 * ALSA/dma_tone often pass maxburst=1. burst_enc(1)=0, but
		 * RetailOS music CTL 0x84249000 needs SB/DB enc=1. Prefer
		 * module params (oracle) over a 1-beat slave hint.
		 */
		if (cfg->src_maxburst > 1)
			ch->src_burst = s5l_pl080_burst_enc(cfg->src_maxburst);
		else
			ch->src_burst = clamp(m2p_src_burst, 0, 7);
		if (cfg->dst_maxburst > 1)
			ch->dst_burst = s5l_pl080_burst_enc(cfg->dst_maxburst);
		else
			ch->dst_burst = clamp(m2p_dst_burst, 0, 7);
	} else {
		ch->fifo_addr = cfg->src_addr;
		if (ch->dst_wid < 0)
			ch->dst_wid = ch->src_wid;
		ch->src_burst = cfg->src_maxburst ?
			s5l_pl080_burst_enc(cfg->src_maxburst) : 0;
		ch->dst_burst = cfg->dst_maxburst ?
			s5l_pl080_burst_enc(cfg->dst_maxburst) : 1;
	}
	ch->dir = cfg->direction;
	return 0;
}

static int s5l_pl080_terminate(struct dma_chan *c)
{
	struct s5l_pl080_chan *ch = to_s5l_chan(c);
	u8 id = ch->id % PL080_CH_COUNT;
	unsigned long flags;
	struct virt_dma_desc *vd;

	dev_dbg(ch->host->dev,
		"term ch%u en=0x%x src=0x%x dst=0x%x lli=0x%x ctl=0x%x cfg=0x%x rawtc=0x%x rawerr=0x%x\n",
		 ch->id, readl(ch->base + PL080_ENBLD_CHNS),
		 readl(ch->base + PL080_Cx_SRC(id)),
		 readl(ch->base + PL080_Cx_DST(id)),
		 readl(ch->base + PL080_Cx_LLI(id)),
		 readl(ch->base + PL080_Cx_CTL(id)),
		 readl(ch->base + PL080_Cx_CFG(id)),
		 readl(ch->base + PL080_RAW_TC),
		 readl(ch->base + PL080_RAW_ERR));
	s5l_pl080_chan_disable(ch);
	spin_lock_irqsave(&ch->vc.lock, flags);
	/* Before the descriptor is freed -- cyc_cb points into it. */
	s5l_pl080_cyc_drop(ch);
	if (ch->running) {
		s5l_pl080_desc_free(&ch->running->vd);
		ch->running = NULL;
	}
	while (!list_empty(&ch->vc.desc_submitted)) {
		vd = list_first_entry(&ch->vc.desc_submitted,
				      struct virt_dma_desc, node);
		list_del(&vd->node);
		s5l_pl080_desc_free(vd);
	}
	spin_unlock_irqrestore(&ch->vc.lock, flags);
	return 0;
}

/*
 * The DMA interrupt.
 *
 * Two rules here decide whether a fault stops the audio or hangs the
 * device.
 *
 * An error must disable the channel and end its transfer, and a channel
 * that keeps erroring must be shut down for good after a bounded number of
 * tries. Clearing the error latch alone leaves the channel enabled, so
 * whatever raised the error raises it again immediately -- an interrupt
 * storm on a single core with the watchdog disarmed, indistinguishable
 * from a lockup from outside.
 *
 * And the handler must claim only what it actually serviced. Returning
 * IRQ_HANDLED unconditionally, including when neither engine had anything
 * pending, tells the kernel every interrupt on the line was handled, which
 * disables the spurious-interrupt protection that would otherwise notice a
 * line stuck active and mask it. Claiming accurately lets that protection
 * work: the audio still dies, but the device stays up and says why.
 */
#define PL080_MAX_CH_ERRS	8

static irqreturn_t s5l_pl080_irq(int irq, void *data)
{
	struct s5l_pl080 *pl = data;
	unsigned int eng, i;
	u32 tc, err;
	bool serviced = false;
	bool pending;

	/*
	 * Drain every pending source before returning.
	 *
	 * The VIC line for this controller is configured edge-triggered, and
	 * a PL080 holds its interrupt asserted until the matching status bit
	 * is cleared. Servicing exactly one terminal count per invocation is
	 * therefore not enough: if another period completes while the handler
	 * is still running, the line never falls, no fresh edge is produced,
	 * and the channel goes silent for the rest of the stream. Servicing
	 * one source per invocation yields one DMA interrupt per playback
	 * where a 6 s file at 1024-frame periods needs a few hundred, so ALSA
	 * receives no period callbacks and every stream runs to XRUN.
	 */
	do {
		pending = false;
		for (eng = 0; eng < 2; eng++) {
			void __iomem *b = pl->base[eng];

			if (!b)
				continue;
			tc = readl(b + PL080_INT_TC_STATUS);
			err = readl(b + PL080_INT_ERR_STATUS);
			if (!tc && !err)
				continue;
			serviced = true;
			pending = true;
			dev_dbg(pl->dev, "irq eng%u tc=0x%x err=0x%x\n",
				eng, tc, err);
			if (tc)
				writel(tc, b + PL080_INT_TC_CLEAR);
			if (err)
				writel(err, b + PL080_INT_ERR_CLEAR);

			for (i = 0; i < PL080_CH_COUNT; i++) {
				struct s5l_pl080_chan *ech =
					&pl->chans[eng * PL080_CH_COUNT + i];
				struct s5l_pl080_desc *ed;
				unsigned long eflags;

				if (!(err & BIT(i)))
					continue;

				/*
				 * Stop the channel before anything else. Leaving it
				 * enabled is what turns one error into a storm.
				 */
				s5l_pl080_chan_disable(ech);

				spin_lock_irqsave(&ech->vc.lock, eflags);
				ed = ech->running;
				ech->running = NULL;
				if (++ech->err_count >= PL080_MAX_CH_ERRS)
					ech->err_stuck = true;
				spin_unlock_irqrestore(&ech->vc.lock, eflags);

				dev_err_ratelimited(pl->dev,
						    "ch%u DMA error (%u so far)%s -- channel stopped\n",
						    ech->id, ech->err_count,
						    ech->err_stuck ? ", giving up on it" : "");

				if (ed) {
					spin_lock_irqsave(&ech->vc.lock, eflags);
					vchan_cookie_complete(&ed->vd);
					spin_unlock_irqrestore(&ech->vc.lock, eflags);
				}
			}

			for (i = 0; i < PL080_CH_COUNT; i++) {
				if (!(tc & BIT(i)))
					continue;
				{
					struct s5l_pl080_chan *ch =
						&pl->chans[eng * PL080_CH_COUNT + i];
					struct s5l_pl080_desc *d;
					unsigned long flags;

					spin_lock_irqsave(&ch->vc.lock, flags);
					d = ch->running;
					if (d && d->cyclic) {
						d->periods_done++;
						if (d->periods)
							d->periods_done %= d->periods;
						/*
						 * Advance the self-linked node HERE,
						 * not from the consumer's workqueue.
						 *
						 * The terminal count and the LLI
						 * reload are one hardware event: by
						 * the time this handler runs the
						 * PL080S has already latched lli[0]
						 * and started the next period. The
						 * window to write the period after
						 * that is exactly one period long,
						 * and a process-context consumer
						 * loses that race whenever the box
						 * is busy -- the hardware then
						 * replays the stale source and the
						 * stream stutters with no FIFO
						 * underrun to show for it.
						 *
						 * Two stores to coherent memory is
						 * all it takes, so there is no
						 * reason for it to be anywhere but
						 * in the interrupt.
						 */
						if (d->ring && d->lli) {
							d->ring_off += d->ring_period;
							if (d->ring_off >= d->ring_bytes)
								d->ring_off = 0;
							d->lli[0].src = cpu_to_le32(
								d->ring_base +
								d->ring_off);
						}
						/*
						 * Not vchan_cyclic_callback(): that
						 * runs the callback in a tasklet, and
						 * this one sleeps. See
						 * s5l_pl080_cyc_workfn().
						 */
						if (ch->cyc_active) {
							atomic_inc(&ch->cyc_pending);
							queue_work(pl->cyc_wq,
								   &ch->cyc_work);
						}
						spin_unlock_irqrestore(&ch->vc.lock,
								       flags);
						continue;
					}
					ch->running = NULL;
					if (d)
						vchan_cookie_complete(&d->vd);
					{
						struct virt_dma_desc *vd =
							vchan_next_desc(&ch->vc);
						if (vd) {
							list_del(&vd->node);
							s5l_pl080_start(ch,
									to_s5l_desc(vd));
						}
					}
					spin_unlock_irqrestore(&ch->vc.lock, flags);
				}
			}
		}
	} while (pending);
	return serviced ? IRQ_HANDLED : IRQ_NONE;
}

static struct dma_chan *s5l_pl080_xlate_args(struct s5l_pl080 *pl,
					     struct of_phandle_args *spec)
{
	struct s5l_pl080_chan *ch;
	unsigned int i, peri, eng_lo, eng_hi;

	if (!pl || !spec || spec->args_count < 1)
		return NULL;
	peri = spec->args[0] & 0x1f;
	if (force_peri >= 0)
		peri = force_peri & 0x1f;
	if (force_eng >= 0) {
		eng_lo = force_eng ? PL080_CH_COUNT : 0;
		eng_hi = eng_lo + PL080_CH_COUNT;
	} else {
		eng_lo = 0;
		eng_hi = PL080_CH_COUNT * 2;
	}
	/*
	 * RetailOS music: peri 10 on physical ch2 (EnbldChns=0x4). Prefer it.
	 * (ASoC often already holds ch2 — dma_tone must reuse via lookup_peri,
	 * not allocate a second peri-10 channel on ch3.)
	 */
	if (force_ch >= 0 && force_ch < PL080_CH_COUNT && peri == 10) {
		unsigned int prefer = eng_lo + force_ch;

		if (prefer < eng_hi) {
			ch = &pl->chans[prefer];
			if (ch->base && !ch->vc.chan.client_count) {
				ch->peri = peri;
				ch->src_burst = clamp(m2p_src_burst, 0, 7);
				ch->dst_burst = clamp(m2p_dst_burst, 0, 7);
				pl080_vinfo(pl->dev,
					 "xlate DT peri=%u -> ch%u (forced) peri=%u\n",
					 spec->args[0] & 0x1f, prefer, ch->peri);
				return dma_get_slave_channel(&ch->vc.chan);
			}
		}
	}
	for (i = eng_lo; i < eng_hi; i++) {
		ch = &pl->chans[i];
		if (!ch->base || ch->vc.chan.client_count)
			continue;
		ch->peri = peri;
		ch->src_burst = clamp(m2p_src_burst, 0, 7);
		ch->dst_burst = clamp(m2p_dst_burst, 0, 7);
		pl080_vinfo(pl->dev, "xlate DT peri=%u -> ch%u (eng%u) peri=%u\n",
			 spec->args[0] & 0x1f, i, i / PL080_CH_COUNT, ch->peri);
		return dma_get_slave_channel(&ch->vc.chan);
	}
	return NULL;
}

static struct dma_chan *s5l_pl080_xlate(struct of_phandle_args *spec,
					struct of_dma *ofdma)
{
	return s5l_pl080_xlate_args(ofdma->of_dma_data, spec);
}

/*
 * Request PL080 slave channel from consumer DT dmas[] without creating
 * the consumer-side dma:tx sysfs symlink (avoids sysfs_warn_dup spam).
 */
struct dma_chan *s5l_pl080_request_slave(struct device *consumer,
					 unsigned int idx)
{
	struct of_phandle_args spec;
	struct platform_device *pdev;
	struct s5l_pl080 *pl;
	struct dma_chan *chan;

	if (!consumer || !consumer->of_node)
		return ERR_PTR(-ENODEV);
	if (of_parse_phandle_with_args(consumer->of_node, "dmas", "#dma-cells",
				       idx, &spec))
		return ERR_PTR(-ENODEV);
	pdev = of_find_device_by_node(spec.np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	pl = platform_get_drvdata(pdev);
	if (!pl) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	chan = s5l_pl080_xlate_args(pl, &spec);
	put_device(&pdev->dev);
	if (!chan)
		return ERR_PTR(-EBUSY);
	return chan;
}
EXPORT_SYMBOL_GPL(s5l_pl080_request_slave);

/*
 * Return an already-owned channel for peri (no client_count bump).
 * dma_tone uses this so it rides RetailOS ch2 held by ASoC instead of
 * allocating a second peri-10 channel.
 */
struct dma_chan *s5l_pl080_lookup_peri(unsigned int peri)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct s5l_pl080 *pl;
	unsigned int i;
	struct dma_chan *found = NULL;

	np = of_find_compatible_node(NULL, NULL, "apple,s5l8740-pl080");
	if (!np)
		np = of_find_compatible_node(NULL, NULL, "arm,pl080");
	if (!np)
		return NULL;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return NULL;
	pl = platform_get_drvdata(pdev);
	if (!pl) {
		put_device(&pdev->dev);
		return NULL;
	}
	peri &= 0x1f;
	for (i = 0; i < PL080_CH_COUNT * 2; i++) {
		struct s5l_pl080_chan *ch = &pl->chans[i];

		if (!ch->base || ch->peri != peri || !ch->vc.chan.client_count)
			continue;
		found = &ch->vc.chan;
		dev_dbg(pl->dev, "lookup peri=%u -> ch%u clients=%u\n",
			peri, ch->id, ch->vc.chan.client_count);
		break;
	}
	put_device(&pdev->dev);
	return found;
}
EXPORT_SYMBOL_GPL(s5l_pl080_lookup_peri);

int s5l_pl080_peri_snapshot(unsigned int peri, u32 *src, u32 *dst, u32 *en)
{
	struct dma_chan *chan;
	struct s5l_pl080_chan *ch;
	u8 id;

	chan = s5l_pl080_lookup_peri(peri);
	if (!chan)
		return -ENODEV;
	ch = to_s5l_chan(chan);
	id = ch->id % PL080_CH_COUNT;
	if (src)
		*src = readl(ch->base + PL080_Cx_SRC(id));
	if (dst)
		*dst = readl(ch->base + PL080_Cx_DST(id));
	if (en)
		*en = readl(ch->base + PL080_ENBLD_CHNS);
	return 0;
}
EXPORT_SYMBOL_GPL(s5l_pl080_peri_snapshot);

/*
 * Rewrite the source address of a running single-node cyclic transfer.
 *
 * This exists to reproduce how RetailOS actually drives audio playback. Stock
 * builds ONE 20-byte descriptor whose next-pointer points at ITSELF, so the
 * transfer never terminates: after each terminal count the PL080S reloads
 * SrcAddr, DstAddr, LLI, Control and Control2 from that descriptor, and
 * because LLI reloads with its own address the cycle repeats forever. The
 * channel's Enable bit stays set for the entire life of the stream and no
 * software touches a channel register on the data path at all -- the producer
 * only rewrites the descriptor in memory and then waits for the TC interrupt.
 *
 * Re-arming with dmaengine_prep_slave_single() per period cannot match
 * that: it terminates the channel at every period boundary and reprograms
 * it from a workqueue, and the IIS TX FIFO drains in about a millisecond,
 * so every period boundary is an underrun window.
 *
 * prep_dma_cyclic already builds stock's topology when buf_len equals
 * period_len: nlli == 1 and lli[0].lli == lli_phys, a node pointing at
 * itself. This function supplies the one missing piece, a way to move the
 * source between terminal counts.
 *
 * Deliberately touches NO channel register -- not Cx_SrcAddr, Cx_Control,
 * Cx_Control2, Cx_LLI or Cx_Config -- and never disables or restarts the
 * channel. Writing any of those is what stock does exactly once, at start.
 * The LLI block is dma_alloc_coherent, so a plain store is enough and no
 * cache maintenance is needed.
 */
int s5l_pl080_rearm_set_src(struct dma_chan *c, dma_addr_t addr, size_t bytes)
{
	struct s5l_pl080_chan *ch;
	struct s5l_pl080_desc *d;
	unsigned long flags;
	unsigned int words;
	int ret = 0;

	if (!c)
		return -ENODEV;
	ch = to_s5l_chan(c);

	spin_lock_irqsave(&ch->vc.lock, flags);
	d = ch->running;
	if (!d || !d->lli) {
		ret = -ENXIO;
		goto out;
	}
	/*
	 * Only the stock shape is accepted. A multi-node chain is a different
	 * model and silently rewriting node 0 of one would corrupt it.
	 */
	if (!d->cyclic || d->nlli != 1) {
		ret = -EINVAL;
		goto out;
	}

	d->lli[0].src = cpu_to_le32(lower_32_bits(addr));

	words = (unsigned int)(bytes / s5l_pl080_unit());
	if ((le32_to_cpu(d->lli[0].ctrl2) & PL080S_XFER_COUNT_MASK) != words)
		d->lli[0].ctrl2 = cpu_to_le32(words & PL080S_XFER_COUNT_MASK);
out:
	spin_unlock_irqrestore(&ch->vc.lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(s5l_pl080_rearm_set_src);

/*
 * Hand the whole ring to the controller once, and let the terminal-count
 * interrupt walk it.
 *
 * s5l_pl080_rearm_set_src() above requires a consumer to post every period
 * from process context, inside a one-period window. That works and it is
 * what stock does -- sub_C4960 writes the descriptor and blocks on the
 * per-channel semaphore -- but stock's producer is a dedicated task on a
 * system with nothing else running. This driver runs a workqueue item behind
 * snd_pcm_period_elapsed() on a single core that is also servicing USB,
 * and it misses the window often enough to be audible: the hardware
 * reloads the stale source, replays the period it just finished, and
 * nothing anywhere reports an error because the FIFO never ran dry.
 *
 * The source walk is entirely deterministic for a cyclic ALSA buffer --
 * base, wrap at base+bytes, step by one period -- so there is no reason
 * for software scheduling to be in the loop at all. This installs the
 * three numbers and the interrupt does the arithmetic.
 *
 * ring_off starts at one period, not zero: s5l_pl080_start() has already
 * loaded period zero into Cx_SrcAddr, so what the FIRST terminal count
 * reloads has to be period one.
 */
size_t s5l_pl080_max_seg_bytes(void)
{
	return (size_t)PL080_MAX_XFER_WORDS * s5l_pl080_unit();
}
EXPORT_SYMBOL_GPL(s5l_pl080_max_seg_bytes);

int s5l_pl080_rearm_set_ring(struct dma_chan *c, dma_addr_t base, size_t bytes,
			     size_t period)
{
	struct s5l_pl080_chan *ch;
	struct s5l_pl080_desc *d;
	unsigned long flags;
	unsigned int words;
	int ret = 0;

	if (!c || !bytes || !period || bytes % period)
		return -EINVAL;
	ch = to_s5l_chan(c);

	spin_lock_irqsave(&ch->vc.lock, flags);
	d = ch->running;
	if (!d || !d->lli) {
		ret = -ENXIO;
		goto out;
	}
	/* Same restriction as set_src: only the single self-linked node. */
	if (!d->cyclic || d->nlli != 1) {
		ret = -EINVAL;
		goto out;
	}

	d->ring_base = lower_32_bits(base);
	d->ring_bytes = (u32)bytes;
	d->ring_period = (u32)period;
	d->ring_off = (u32)period % (u32)bytes;
	d->lli[0].src = cpu_to_le32(d->ring_base + d->ring_off);

	words = (unsigned int)(period / s5l_pl080_unit());
	if ((le32_to_cpu(d->lli[0].ctrl2) & PL080S_XFER_COUNT_MASK) != words)
		d->lli[0].ctrl2 = cpu_to_le32(words & PL080S_XFER_COUNT_MASK);

	/* Last, so the interrupt never sees a half-installed ring. */
	d->ring = true;
out:
	spin_unlock_irqrestore(&ch->vc.lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(s5l_pl080_rearm_set_ring);

static ssize_t chregs_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct s5l_pl080 *pl = dev_get_drvdata(dev);
	int n = 0, eng, i;

	if (!pl)
		return -ENODEV;
	for (eng = 0; eng < 2; eng++) {
		void __iomem *b = pl->base[eng];

		if (!b)
			continue;
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "eng%u en=0x%x rawtc=0x%x rawerr=0x%x cfg=0x%x\n",
			       eng, readl(b + PL080_ENBLD_CHNS),
			       readl(b + PL080_RAW_TC),
			       readl(b + PL080_RAW_ERR),
			       readl(b + PL080_CONFIG));
		for (i = 0; i < PL080_CH_COUNT; i++)
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       " e%u ch%u src=%08x dst=%08x lli=%08x ctl=%08x cfg=%08x c2=%08x\n",
				       eng, i,
				       readl(b + PL080_Cx_SRC(i)),
				       readl(b + PL080_Cx_DST(i)),
				       readl(b + PL080_Cx_LLI(i)),
				       readl(b + PL080_Cx_CTL(i)),
				       readl(b + PL080_Cx_CFG(i)),
				       readl(b + PL080S_Cx_CONTROL2(i)));
	}
	return n;
}
static DEVICE_ATTR_RO(chregs);

static int s5l_pl080_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l_pl080 *pl;
	struct resource *res;
	int irq, i, ret;
	u32 id0;

	pl = devm_kzalloc(dev, sizeof(*pl), GFP_KERNEL);
	if (!pl)
		return -ENOMEM;
	pl->dev = dev;
	spin_lock_init(&pl->lock);
	init_llist_head(&pl->free_list);
	INIT_WORK(&pl->free_work, s5l_pl080_free_work);

	for (i = 0; i < 2; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			continue;
		pl->base[i] = devm_ioremap_resource(dev, res);
		if (IS_ERR(pl->base[i]))
			return PTR_ERR(pl->base[i]);
	}
	if (!pl->base[0])
		return -EINVAL;

	pl->clk[0] = devm_clk_get_optional(dev, "dmac0");
	pl->clk[1] = devm_clk_get_optional(dev, "dmac1");
	if (!IS_ERR_OR_NULL(pl->clk[0]))
		clk_prepare_enable(pl->clk[0]);
	if (!IS_ERR_OR_NULL(pl->clk[1]))
		clk_prepare_enable(pl->clk[1]);

	id0 = readl(pl->base[0] + 0xfe0) & 0xff;
	writel(PL080_CONFIG_EN, pl->base[0] + PL080_CONFIG);
	writel(sync_mask, pl->base[0] + PL080_SYNC);
	if (pl->base[1]) {
		writel(PL080_CONFIG_EN, pl->base[1] + PL080_CONFIG);
		writel(sync_mask, pl->base[1] + PL080_SYNC);
	}
	/*
	 * Nothing else is seeded here, and two things that briefly were have
	 * been removed because stock does not write them.
	 *
	 * SoftBReq: a working RetailOS reads 0x00000008 while playing, but
	 * the register is never written anywhere in the image -- a search of
	 * the whole decomp for 0x38200020 / 0x38700020 finds zero hits. That
	 * value is a reset default or hardware-set, not configuration.
	 *
	 * Idle-channel Control: RetailOS holds 0x80249000 in ch0/ch1 while
	 * playing, and it is tempting to seed it. But sub_A4F94 writes
	 * Cx_Control only for the channel it is ALLOCATING, and from a
	 * per-peripheral table at 0x0891DB94:
	 *
	 *	v20 = 32 * chan + 0x3820010C;   // Cx_CTL(chan)
	 *	*(u32 *)0x38200030 = 1;         // DMACConfiguration
	 *	*v20 = *(u32 *)(4 * peri + 0x891DB94);
	 *
	 * so the value in an idle channel is residue from its last
	 * allocation, not state stock maintains. Cx_Control is already written
	 * at channel start, which is the equivalent.
	 *
	 * Reading a register on a working device says what it holds. It does
	 * not say the software put it there.
	 */
	dev_info(dev, "DMACSync=0x%08x (0 = sync enabled, as stock)\n",
		 sync_mask);

	/* DDI0196 M2M: try AHB1/AHB2 × 16/32-bit. dst0==pattern means the engine copies. */
	{
		dma_addr_t sa, da;
		u32 *s, *d;
		int eng, as, ad, wid;

		s = dmam_alloc_coherent(dev, 64, &sa, GFP_KERNEL);
		d = dmam_alloc_coherent(dev, 64, &da, GFP_KERNEL);
		if (s && d) {
			s[0] = 0xa5a5a5a5;
			s[1] = 0x5a5a5a5a;
			for (eng = 0; eng < 2; eng++) {
				void __iomem *b = pl->base[eng];

				if (!b)
					continue;
				pl080_vinfo(dev, "selftest eng%u id=%02x\n",
					 eng, readl(b + 0xfe0) & 0xff);
				for (wid = 1; wid <= 2; wid++) {
					for (as = 0; as <= 1; as++) {
						for (ad = 0; ad <= 1; ad++) {
							u32 ctl, words = (wid == 2) ? 4 : 8;

							d[0] = 0;
							writel(0, b + PL080_Cx_CFG(0));
							writel(lower_32_bits(sa),
							       b + PL080_Cx_SRC(0));
							writel(lower_32_bits(da),
							       b + PL080_Cx_DST(0));
							writel(0, b + PL080_Cx_LLI(0));
							ctl = words |
							      (wid << CTL_WIDTH_SHIFT) |
							      (wid << (CTL_WIDTH_SHIFT + 3)) |
							      (2 << CTL_SBSIZE_SHIFT) |
							      (2 << CTL_DBSIZE_SHIFT) |
							      CTL_PROT_PRIV |
							      CTL_PROT_BUFF |
							      CTL_PROT_CACHE |
							      CTL_SRC_AI | CTL_DST_AI |
							      CTL_TC_IRQ;
							if (as)
								ctl |= BIT(24);
							if (ad)
								ctl |= BIT(25);
							writel(ctl, b + PL080_Cx_CTL(0));
							writel(words, b + PL080S_Cx_CONTROL2(0));
							writel(CFG_ENABLE | CFG_IE | CFG_ITC,
							       b + PL080_Cx_CFG(0));
							writel(BIT(0), b + PL080_SOFT_BREQ);
							writel(BIT(0), b + PL080_SOFT_SREQ);
							udelay(50);
							pl080_vinfo(dev,
								 "selftest e%u w%u s%d d%d tc=%x err=%x dst=%08x\n",
								 eng, wid, as, ad,
								 readl(b + PL080_RAW_TC),
								 readl(b + PL080_RAW_ERR),
								 d[0]);
							writel(0, b + PL080_Cx_CFG(0));
							writel(~0u, b + PL080_INT_TC_CLEAR);
							writel(~0u, b + PL080_INT_ERR_CLEAR);
						}
					}
				}
			}
		}
	}

	dma_cap_zero(pl->ddev.cap_mask);
	dma_cap_set(DMA_SLAVE, pl->ddev.cap_mask);
	dma_cap_set(DMA_CYCLIC, pl->ddev.cap_mask);
	/*
	 * WQ_HIGHPRI because a missed period is an audible dropout, and
	 * WQ_MEM_RECLAIM because playback must keep draining under memory
	 * pressure rather than deadlock against it.
	 */
	pl->cyc_wq = alloc_workqueue("pl080-cyc",
				     WQ_HIGHPRI | WQ_MEM_RECLAIM, 0);
	if (!pl->cyc_wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, s5l_pl080_destroy_wq, pl->cyc_wq);
	if (ret)
		return ret;

	dma_cap_set(DMA_PRIVATE, pl->ddev.cap_mask);
	pl->ddev.dev = dev;
	pl->ddev.device_alloc_chan_resources = s5l_pl080_alloc;
	pl->ddev.device_free_chan_resources = s5l_pl080_free;
	pl->ddev.device_tx_status = s5l_pl080_tx_status;
	pl->ddev.device_issue_pending = s5l_pl080_issue;
	pl->ddev.device_prep_slave_sg = s5l_pl080_prep_slave_sg;
	pl->ddev.device_prep_dma_cyclic = s5l_pl080_prep_dma_cyclic;
	pl->ddev.device_config = s5l_pl080_config;
	pl->ddev.device_terminate_all = s5l_pl080_terminate;
	pl->ddev.device_synchronize = s5l_pl080_synchronize;
	pl->ddev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	pl->ddev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	pl->ddev.directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	/*
	 * The position now comes from the channel's live transfer pointer
	 * (see s5l_pl080_tx_status), so residue is accurate to a burst rather
	 * than to a whole descriptor. Declaring DESCRIPTOR made ALSA treat the
	 * stream as batched and mis-size its own timing.
	 */
	pl->ddev.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	INIT_LIST_HEAD(&pl->ddev.channels);

	for (i = 0; i < PL080_CH_COUNT * 2; i++) {
		struct s5l_pl080_chan *ch = &pl->chans[i];

		ch->host = pl;
		ch->id = i;
		ch->base = pl->base[i / PL080_CH_COUNT];
		if (!ch->base)
			continue;
		ch->vc.desc_free = s5l_pl080_desc_free;
		INIT_WORK(&ch->cyc_work, s5l_pl080_cyc_workfn);
		atomic_set(&ch->cyc_pending, 0);
		vchan_init(&ch->vc, &pl->ddev);
	}

	for (i = 0; i < 2; i++) {
		irq = platform_get_irq_optional(pdev, i);
		if (irq <= 0)
			continue;
		ret = devm_request_irq(dev, irq, s5l_pl080_irq, 0,
				       "s5l8740-pl080", pl);
		if (ret)
			dev_warn(dev, "IRQ %d: %d (poll mode)\n", irq, ret);
	}

	ret = dma_async_device_register(&pl->ddev);
	if (ret)
		return ret;

	ret = of_dma_controller_register(dev->of_node, s5l_pl080_xlate, pl);
	if (ret)
		dev_warn(dev, "of_dma_controller_register: %d\n", ret);

	pl->dummy_cpu = dmam_alloc_coherent(dev, 4096, &pl->dummy_dma,
					    GFP_KERNEL);
	if (!pl->dummy_cpu)
		dev_warn(dev, "dummy DMA sink alloc failed\n");

	pl->lli_pool = dmam_alloc_coherent(dev,
					   s5l_pl080_lli_size(PL080_LLI_POOL_NODES),
					   &pl->lli_pool_phys, GFP_KERNEL);
	if (!pl->lli_pool)
		dev_warn(dev, "LLI pool alloc failed — GFP_NOWAIT fallback only\n");
	else
		pl080_vinfo(dev, "LLI pool %u nodes pa=%pad\n",
			 PL080_LLI_POOL_NODES, &pl->lli_pool_phys);

	pl->pump = kthread_run(s5l_pl080_pump, pl, "n31-pl080-pump");
	if (IS_ERR(pl->pump)) {
		dev_warn(dev, "soft-req pump: %ld\n", PTR_ERR(pl->pump));
		pl->pump = NULL;
	}

	platform_set_drvdata(pdev, pl);
	ret = device_create_file(dev, &dev_attr_chregs);
	if (ret)
		dev_warn(dev, "chregs sysfs: %d\n", ret);
	pl080_vinfo(dev,
		 "PL080 dmaengine @%pR id=%02x peri IIS0=10/11 IIS2 RX=13 (RetailOS)\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0), id0);
	return 0;
}

static void s5l_pl080_remove(struct platform_device *pdev)
{
	struct s5l_pl080 *pl = platform_get_drvdata(pdev);

	if (pl->pump)
		kthread_stop(pl->pump);
	device_remove_file(&pdev->dev, &dev_attr_chregs);
	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&pl->ddev);
	/* Descriptors parked by s5l_pl080_desc_free() must not outlive the channel. */
	cancel_work_sync(&pl->free_work);
	s5l_pl080_free_work(&pl->free_work);
}

/*
 * kexec hands the machine to a new kernel with the old one's memory map
 * already forgotten. A channel still running writes into whatever now
 * occupies its destination, so the controller has to be stopped before
 * the jump -- and remove() does not do it, because unregistering a
 * dma_device says nothing to the hardware.
 *
 * Clearing CONFIG_EN halts both controllers outright rather than
 * unwinding channel by channel, which is what is wanted here: nothing
 * after this point needs the engine, and a per-channel teardown has
 * more ways to get stuck than to succeed.
 */
static void s5l_pl080_shutdown(struct platform_device *pdev)
{
	struct s5l_pl080 *pl = platform_get_drvdata(pdev);
	unsigned int i;

	if (!pl)
		return;
	for (i = 0; i < ARRAY_SIZE(pl->base); i++) {
		if (!pl->base[i])
			continue;
		writel(readl(pl->base[i] + PL080_CONFIG) & ~PL080_CONFIG_EN,
		       pl->base[i] + PL080_CONFIG);
	}
	dev_info(&pdev->dev, "PL080 halted for shutdown/kexec\n");
}

static const struct of_device_id s5l_pl080_of_match[] = {
	{ .compatible = "apple,s5l8740-pl080" },
	{ .compatible = "arm,pl080" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l_pl080_of_match);

static struct platform_driver s5l_pl080_driver = {
	.probe = s5l_pl080_probe,
	.remove = s5l_pl080_remove,
	.shutdown = s5l_pl080_shutdown,
	.driver = {
		.name = "s5l8740-pl080",
		.of_match_table = s5l_pl080_of_match,
	},
};
module_platform_driver(s5l_pl080_driver);

MODULE_DESCRIPTION("S5L8740 PL080 dmaengine (N31)");
MODULE_LICENSE("GPL");
