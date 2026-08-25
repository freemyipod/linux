// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8740 dual PL080 DMA (N31)
 *
 * Bases: 0x38200000 / 0x38700000 (OSOS pair). Not 0x384 (DWC OTG).
 *
 * DT #dma-cells = <2>: <peri_id ccr_flags>
 * Quirks (PL080 + I²S on S5L8740/N31):
 *   Burst: M2P dest=1 beat (fixed IIS FIFO @+0x10); src≈4 beats (half FIFO).
 *   Peri: glass IIS0 TX/RX = 10/11 (Rockbox 0xA); OSOS table 12/13 never TCs.
 *   Cache: PL080 not coherent — dma_sync in start(); no CTL_PROT_CACHE on slave.
 *   LLI: dma_alloc_coherent, 16-byte aligned chain; misaligned LLI hangs engine.
 *   terminate_all: CFG disable + bounded ENBLD poll — never spin on BUSY (amba-pl08x).
 *   SG: multi-element builds LLI chain; contiguous buffers preferred (CMA).
 *   AHB: M2P src=mem on AHB2 (ahb_s=1), dst=FIFO on AHB1 (ahb_d=0).
 *   FIFO: S3C64xx-style ~64 deep — src burst 8 (m2p_src_burst=2), dst=1.
 *   PL080S: CONTROL2 @+0x114 holds count (OSOS B424C), not CTL low bits.
 *   Cache: ARM1176 32-byte lines — LLI/buffer 32-byte aligned; sync in start().
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
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
static int force_peri = 10;
module_param(force_peri, int, 0644);
MODULE_PARM_DESC(force_peri, "override DT DMA peri id (-1 = use DT)");
static int force_mem;
module_param(force_mem, int, 0644);
MODULE_PARM_DESC(force_mem, "1 = M2M flow + soft req, dest still FIFO");
static int force_flow = -1;
module_param(force_flow, int, 0644);
MODULE_PARM_DESC(force_flow, "PL080 FlowCntrl -1=auto M2P, 0=M2M+soft, 1=M2P, 5=M2P-peri");
/* DDI0196 CxControl bits 24/25: 0=AHB1, 1=AHB2. Kitra memcpy uses AHB1. */
/* M2P: AHB2→memory, AHB1→APB FIFO (Samsung PL080S topology). */
static int ahb_s = 1;
module_param(ahb_s, int, 0644);
MODULE_PARM_DESC(ahb_s, "source AHB master (0=AHB1/periph-side, 1=AHB2/mem)");
static int ahb_d;
module_param(ahb_d, int, 0644);
MODULE_PARM_DESC(ahb_d, "dest AHB master (0=AHB1/periph, 1=AHB2/mem)");
/* 1=16-bit S16 LE (Rockbox pcm / OSOS BCB60 16-bit). 2=32-bit packed LR. */
static int xfer_width = 1;
module_param(xfer_width, int, 0644);
MODULE_PARM_DESC(xfer_width, "PL080 src/dst width 0=8 1=16 2=32");
/* M2P: dest burst 1; src 8 beats (~half 64-entry IIS FIFO). */
static int m2p_src_burst = 2; /* enc: 2=8 beats */
module_param(m2p_src_burst, int, 0644);
MODULE_PARM_DESC(m2p_src_burst, "M2P SBSIZE enc (default 2=8 beats)");
static int m2p_dst_burst; /* 0=1 beat — do not burst into IIS TX FIFO */
module_param(m2p_dst_burst, int, 0644);
MODULE_PARM_DESC(m2p_dst_burst, "M2P DBSIZE enc (default 0=1 beat)");
static int force_eng = -1;
module_param(force_eng, int, 0644);
MODULE_PARM_DESC(force_eng, "PL080 engine 0/1 for xlate (-1 = either)");

#define PL080_LLI_ALIGN		32
#define PL080_TERM_POLL_US	10
#define PL080_TERM_POLL_MAX	10

struct pl080_lli {
	__le32 src;
	__le32 dst;
	__le32 lli;
	__le32 ctrl;
} __aligned(PL080_LLI_ALIGN);

static size_t s5l_pl080_lli_size(unsigned int nlli)
{
	return ALIGN(nlli * sizeof(struct pl080_lli), PL080_LLI_ALIGN);
}

static dma_addr_t s5l_pl080_lli_pa(dma_addr_t base, unsigned int idx)
{
	return base + idx * sizeof(struct pl080_lli);
}

static struct pl080_lli *s5l_pl080_lli_alloc(struct device *dev,
					     unsigned int nlli,
					     dma_addr_t *phys)
{
	size_t bytes = s5l_pl080_lli_size(nlli);
	struct pl080_lli *lli;

	lli = dma_alloc_coherent(dev, bytes, phys, GFP_NOWAIT);
	if (!lli)
		return NULL;
	if (*phys & (PL080_LLI_ALIGN - 1))
		dev_warn(dev, "LLI phys misaligned pa=%pad (need %u)\n",
			 &*phys, PL080_LLI_ALIGN);
	return lli;
}
struct s5l_pl080_chan {
	struct virt_dma_chan	vc;
	struct s5l_pl080	*host;
	void __iomem		*base;
	u8			id;
	u8			peri;
	u8			src_burst;
	u8			dst_burst;
	enum dma_transfer_direction dir;
	dma_addr_t		fifo_addr;
	struct s5l_pl080_desc	*running;
};

struct s5l_pl080_desc {
	struct virt_dma_desc	vd;
	struct pl080_lli	*lli;
	dma_addr_t		lli_phys;
	unsigned int		nlli;
	u32			cfg;
	bool			cyclic;
	dma_addr_t		buf_addr;
	size_t			buf_len;
};

struct s5l_pl080;

struct dma_chan *s5l_pl080_request_slave(struct device *consumer,
					 unsigned int idx);

struct s5l_pl080 {
	struct device		*dev;
	void __iomem		*base[2];
	struct clk		*clk[2];
	struct dma_device	ddev;
	struct s5l_pl080_chan	chans[PL080_CH_COUNT * 2];
	spinlock_t		lock;
	void			*dummy_cpu;
	dma_addr_t		dummy_dma;
	struct task_struct	*pump;
};

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

static unsigned int s5l_pl080_unit(void)
{
	unsigned int w = xfer_width & 7;

	if (w > 2)
		w = 1;
	return 1u << w;
}

static u32 s5l_pl080_build_ctl(struct s5l_pl080_chan *ch, u32 words,
			      bool src_inc, bool dst_inc, bool irq)
{
	unsigned int w = xfer_width & 7;
	unsigned int sb, db;
	u32 ctl;

	if (w > 2)
		w = 1;
	if (ch && (ch->dir == DMA_MEM_TO_DEV || ch->dir == DMA_DEV_TO_MEM)) {
		sb = ch->src_burst;
		db = ch->dst_burst;
		ctl = CTL_PROT_PRIV | CTL_PROT_BUFF;
	} else {
		/* M2M selftest / memcpy: Rockbox pcm-s5l8702 8/4 */
		sb = 2;
		db = 1;
		ctl = CTL_PROT_PRIV | CTL_PROT_BUFF | CTL_PROT_CACHE;
	}
	ctl |= words | (w << CTL_WIDTH_SHIFT) | (w << (CTL_WIDTH_SHIFT + 3)) |
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

static void s5l_pl080_sync_buffer(struct s5l_pl080_chan *ch,
				  struct s5l_pl080_desc *d)
{
	struct device *dev = ch->host->dev;

	if (!d->buf_len || !dev)
		return;
	if (ch->dir == DMA_MEM_TO_DEV)
		dma_sync_single_for_device(dev, d->buf_addr, d->buf_len,
					   DMA_TO_DEVICE);
	else if (ch->dir == DMA_DEV_TO_MEM)
		dma_sync_single_for_device(dev, d->buf_addr, d->buf_len,
					   DMA_FROM_DEVICE);
}

static void s5l_pl080_start(struct s5l_pl080_chan *ch, struct s5l_pl080_desc *d)
{
	void __iomem *b = ch->base;
	u8 id = ch->id % PL080_CH_COUNT;
	struct pl080_lli *first = d->lli;

	s5l_pl080_sync_buffer(ch, d);
	s5l_pl080_chan_disable(ch);
	writel(le32_to_cpu(first->src), b + PL080_Cx_SRC(id));
	writel(le32_to_cpu(first->dst), b + PL080_Cx_DST(id));
	/* Next LLI, not the first (already loaded into SRC/DST/CTL). */
	writel(le32_to_cpu(first->lli), b + PL080_Cx_LLI(id));
	writel(le32_to_cpu(first->ctrl), b + PL080_Cx_CTL(id));
	/* B424C: CONTROL2 = transfer count (v27 & 0x1FFFFFFF), not CTL. */
	writel(le32_to_cpu(first->ctrl) & 0x1fffffffu,
	       b + PL080S_Cx_CONTROL2(id));
	writel(d->cfg | CFG_ENABLE, b + PL080_Cx_CFG(id));
	/* M2M / force_flow 0|4: software request. M2P peri waits for IIS DRQ. */
	if (s5l_pl080_need_soft()) {
		writel(BIT(id), b + PL080_SOFT_BREQ);
		if (force_mem || force_flow == 0)
			writel(BIT(id), b + PL080_SOFT_SREQ);
	}
	ch->running = d;
	dev_info(ch->host->dev,
		 "ch%u start peri=%u cfg=0x%x nlli=%u src=0x%x dst=0x%x ctl=0x%x\n",
		 ch->id, ch->peri, (u32)(d->cfg | CFG_ENABLE), d->nlli,
		 le32_to_cpu(first->src), le32_to_cpu(first->dst),
		 le32_to_cpu(first->ctrl));
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

		cur = readl(ch->base + ((ch->dir == DMA_DEV_TO_MEM) ?
					PL080_Cx_DST(id) : PL080_Cx_SRC(id)));
		start = lower_32_bits(d->buf_addr);
		end = start + d->buf_len;
		if (cur >= start && cur < end)
			state->residue = end - cur;
		else
			state->residue = d->buf_len;
	}
	spin_unlock_irqrestore(&ch->vc.lock, flags);
	return st;
}

static int s5l_pl080_alloc(struct dma_chan *c)
{
	return 0;
}

static void s5l_pl080_free(struct dma_chan *c)
{
	vchan_free_chan_resources(&to_s5l_chan(c)->vc);
}

static void s5l_pl080_desc_free(struct virt_dma_desc *vd)
{
	struct s5l_pl080_desc *d = to_s5l_desc(vd);
	struct s5l_pl080_chan *ch = to_s5l_chan(vd->tx.chan);

	if (d->lli && !irqs_disabled() && !in_atomic())
		dma_free_coherent(ch->host->dev, s5l_pl080_lli_size(d->nlli),
				  d->lli, d->lli_phys);
	kfree(d);
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
		dev_info(ch->host->dev, "prep_slave_sg sg_len=%u (LLI chain)\n",
			 sg_len);

	/* One LLI node per <= PL080_MAX_XFER_WORDS transfer units */
	nlli = DIV_ROUND_UP(total, PL080_MAX_XFER_WORDS * s5l_pl080_unit());
	if (nlli == 0)
		return NULL;

	d = kzalloc(sizeof(*d), GFP_NOWAIT);
	if (!d)
		return NULL;

	lli = s5l_pl080_lli_alloc(ch->host->dev, nlli, &lli_phys);
	if (!lli) {
		kfree(d);
		return NULL;
	}

	cfg = 0;
	dev_addr = ch->fifo_addr;
	if (force_flow >= 0) {
		cfg |= ((force_flow & 7) << CFG_FLOW_SHIFT) | CFG_IE | CFG_ITC;
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

				if (idx < nlli - 1)
					lli[idx].lli = cpu_to_le32(
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
		dev_info(ch->host->dev,
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

	lli = s5l_pl080_lli_alloc(ch->host->dev, nlli, &lli_phys);
	if (!lli) {
		kfree(d);
		return NULL;
	}

	cfg = 0;
	dev_addr = ch->fifo_addr;
	if (force_flow >= 0) {
		cfg |= ((force_flow & 7) << CFG_FLOW_SHIFT) | CFG_IE | CFG_ITC;
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

			if (idx + 1 < nlli)
				lli[idx].lli = cpu_to_le32(lower_32_bits(
					s5l_pl080_lli_pa(lli_phys, idx + 1)));
			else
				lli[idx].lli = cpu_to_le32(
					lower_32_bits(lli_phys));

			sg_off += chunk;
			sg_left -= chunk;
			idx++;
		}
	}
	if (idx)
		lli[idx - 1].lli = cpu_to_le32(lower_32_bits(lli_phys));

	d->lli = lli;
	d->lli_phys = lli_phys;
	d->nlli = nlli;
	d->cfg = cfg;
	d->cyclic = true;
	d->buf_addr = buf_addr;
	d->buf_len = buf_len;
	dev_info(ch->host->dev,
		 "cyclic ok peri=%u nlli=%u periods=%u period=%zu fifo=0x%x\n",
		 ch->peri, nlli, periods, period_len,
		 (u32)lower_32_bits(dev_addr));
	return vchan_tx_prep(&ch->vc, &d->vd, flags);
}

static int s5l_pl080_config(struct dma_chan *c,
			    struct dma_slave_config *cfg)
{
	struct s5l_pl080_chan *ch = to_s5l_chan(c);

	if (cfg->direction == DMA_MEM_TO_DEV) {
		ch->fifo_addr = cfg->dst_addr;
		ch->src_burst = cfg->src_maxburst ?
			s5l_pl080_burst_enc(cfg->src_maxburst) :
			clamp(m2p_src_burst, 0, 7);
		ch->dst_burst = cfg->dst_maxburst ?
			s5l_pl080_burst_enc(cfg->dst_maxburst) :
			clamp(m2p_dst_burst, 0, 7);
	} else {
		ch->fifo_addr = cfg->src_addr;
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

	dev_info(ch->host->dev,
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

static irqreturn_t s5l_pl080_irq(int irq, void *data)
{
	struct s5l_pl080 *pl = data;
	unsigned eng, i;
	u32 tc, err;

	for (eng = 0; eng < 2; eng++) {
		void __iomem *b = pl->base[eng];

		if (!b)
			continue;
		tc = readl(b + PL080_INT_TC_STATUS);
		err = readl(b + PL080_INT_ERR_STATUS);
		if (tc || err)
			dev_info_ratelimited(pl->dev,
					     "irq eng%u tc=0x%x err=0x%x\n",
					     eng, tc, err);
		if (tc)
			writel(tc, b + PL080_INT_TC_CLEAR);
		if (err)
			writel(err, b + PL080_INT_ERR_CLEAR);
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
					vchan_cyclic_callback(&d->vd);
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
	return IRQ_HANDLED;
}

static struct dma_chan *s5l_pl080_xlate_args(struct s5l_pl080 *pl,
					     struct of_phandle_args *spec)
{
	struct s5l_pl080_chan *ch;
	unsigned i, peri, eng_lo, eng_hi;

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
	for (i = eng_lo; i < eng_hi; i++) {
		ch = &pl->chans[i];
		if (!ch->base || ch->vc.chan.client_count)
			continue;
		ch->peri = peri;
		ch->src_burst = clamp(m2p_src_burst, 0, 7);
		ch->dst_burst = clamp(m2p_dst_burst, 0, 7);
		dev_info(pl->dev, "xlate DT peri=%u -> ch%u (eng%u) peri=%u\n",
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
	writel(~0u, pl->base[0] + PL080_SYNC);
	if (pl->base[1]) {
		writel(PL080_CONFIG_EN, pl->base[1] + PL080_CONFIG);
		writel(~0u, pl->base[1] + PL080_SYNC);
	}

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
				dev_info(dev, "selftest eng%u id=%02x\n",
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
							dev_info(dev,
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
	pl->ddev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	pl->ddev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	pl->ddev.directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	pl->ddev.residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	INIT_LIST_HEAD(&pl->ddev.channels);

	for (i = 0; i < PL080_CH_COUNT * 2; i++) {
		struct s5l_pl080_chan *ch = &pl->chans[i];

		ch->host = pl;
		ch->id = i;
		ch->base = pl->base[i / PL080_CH_COUNT];
		if (!ch->base)
			continue;
		ch->vc.desc_free = s5l_pl080_desc_free;
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

	pl->pump = kthread_run(s5l_pl080_pump, pl, "n31-pl080-pump");
	if (IS_ERR(pl->pump)) {
		dev_warn(dev, "soft-req pump: %ld\n", PTR_ERR(pl->pump));
		pl->pump = NULL;
	}

	platform_set_drvdata(pdev, pl);
	ret = device_create_file(dev, &dev_attr_chregs);
	if (ret)
		dev_warn(dev, "chregs sysfs: %d\n", ret);
	dev_info(dev,
		 "PL080 dmaengine @%pR id=%02x peri IIS0=10/11 (glass) OSOS=12/13\n",
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
	.driver = {
		.name = "s5l8740-pl080",
		.of_match_table = s5l_pl080_of_match,
	},
};
module_platform_driver(s5l_pl080_driver);

MODULE_DESCRIPTION("S5L8740 PL080 dmaengine (N31)");
MODULE_LICENSE("GPL");
