// SPDX-License-Identifier: GPL-2.0-only

#include <linux/apple-n31.h>
#include <linux/completion.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <linux/delay.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_rect.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_client_event.h>

#include "s5l8740-comp-lut.h"

#define S5L8740_LCD_CON			0x00 /* Control register. */
#define S5L8740_LCD_WCMD		0x04 /* Write command register. */
#define S5L8740_LCD_RCMD		0x0C /* Read command register. */
#define S5L8740_LCD_RDATA		0x10 /* Read data register. */
#define S5L8740_LCD_DBUFF		0x14 /* Read Data buffer */
#define S5L8740_LCD_INTCON		0x18 /* Interrupt control register */
#define S5L8740_LCD_STATUS		0x1C /* LCD Interface status 0106 */
#define S5L8740_LCD_PHTIME		0x20 /* Phase time register 0060 */
#define S5L8740_LCD_RST_TIME	0x24 /* Reset active period 07FF */
#define S5L8740_LCD_DRV_RST		0x28 /* Reset drive signal */
#define S5L8740_LCD_WDATA		0x40 /* Write data register (0x40...0x5C) FIXME */

#define S5L8740_LCD_STATUS_BUSY	0x10

/* GATE0: 1us was too short (stride/FIFO); 100ms wait, pitch-aware blit */
#define S5L8740_LCD_TIMEOUT_US 100000

/*
 * Transfer framing.
 *
 * A transfer is not just a run of pixel writes. Stock brackets it, and
 * sub_A25C0 is the whole shape of it:
 *
 *   if (MEMORY[0x3830008C] << 30)  return -1;   busy: bits 0-1 set
 *   MEMORY[0x38300080] = 1;                     open
 *   sub_AEFAC(0, 0, w, h);                      window, then pixels
 *   do sub_345D68(); while (!v3);               settle
 *   MEMORY[0x38300080] = 0;                     close
 *
 * and sub_A96E4() is the bare wait, "while (MEMORY[0x3830008C] << 30);"
 * -- a left shift of 30 tests bits 0 and 1, so nonzero means busy.
 *
 * Without the bracket the panel is handed a stream that neither begins
 * nor ends where it expects, so its write pointer does not come back to
 * the origin and the next frame starts from wherever the last one
 * stopped. Rows stay intact and the image walks up and down the screen,
 * which is exactly what this driver did while every DSI window command
 * was reporting success.
 */
#define S5L8740_LCD_XFER		0x80	/* 1 = transfer open */
/*
 * +0x8c, bits 1:0, the pair sub_A25C0 tests with `<< 30`.
 *
 * Bit 0 mirrors the gate. Measured 2026-09-07 on the live device: writing
 * +0x80 = 1 makes +0x8c read 0x0FA00301 on every sample, writing 0 makes
 * it read 0x0FA00300 on every sample. It is not "busy"; it is "open".
 * Bit 1 is the one that shows up on top of it in three of the eight
 * oracle snapshots (0x0FA00303), taken mid-refresh, and is the only bit
 * here that can mean a transfer is in flight. That reading of bit 1 is
 * inferred from the capture, not measured directly.
 *
 * The consequence for anyone tempted to wait for these bits to clear
 * before closing the gate: bit 0 cannot clear until the gate is closed.
 * That wait ends only by its own timeout, on every commit.
 */
#define S5L8740_LCD_XSTAT		0x8c
#define S5L8740_LCD_XSTAT_OPEN	BIT(0)
#define S5L8740_LCD_XSTAT_ACTIVE	BIT(1)
#define S5L8740_LCD_XSTAT_BUSY	(S5L8740_LCD_XSTAT_OPEN | S5L8740_LCD_XSTAT_ACTIVE)

/*
 * The layer compositor at 0x38900000 -- how stock actually draws.
 *
 * Captured 2026-09-07 from a running 1.1.2 RetailOS with the UI up
 * (artifacts/retailos-compositor-oracle/), and named from the 1.0.2
 * decomp's sub_825C4 / sub_45690. One layer, L0, 240x432, 32 bpp with a
 * constant 0xFF top byte, stride 960, pointed at one of two heap buffers
 * 414,848 bytes apart. The layer config never changed between samples;
 * a frame is the LCDIF gate cycle (sub_A25C0) with the layer address
 * already pointing at the new buffer.
 *
 * Every value below is what the device held. Where the decomp shows the
 * formula it is noted; where it does not, the captured word is written
 * as captured.
 */
#define S5L8740_COMP_CTRL	0x000	/* bit 0 enable (sub_4AA80), bit 7 (sub_53AC8) */
#define S5L8740_COMP_CTRL_EN	BIT(0)
#define S5L8740_COMP_CTRL_STOCK	0x00000085
#define S5L8740_COMP_LAYERS	0x004	/* 0x10 L0, 0x20 L1, 0x08 L2 (sub_825C4); 0x100 (sub_52DF8) */
#define S5L8740_COMP_LAYERS_L0	BIT(4)
#define S5L8740_COMP_LAYERS_STOCK 0x00000210
#define S5L8740_COMP_CTRL8	0x008	/* 0x700, sub_56E58 / sub_2549DC */
#define S5L8740_COMP_CTRL8_STOCK 0x00000700
#define S5L8740_COMP_OUT10	0x010	/* 0x321 */
#define S5L8740_COMP_OUT10_STOCK 0x00000321
#define S5L8740_COMP_L0_CFG	0x020	/* 0x002107ff: (v<<22)|(byte<<8)|(flag<<21)|(byte<<16) */
#define S5L8740_COMP_L0_CFG_STOCK 0x002107ff
#define S5L8740_COMP_L0_ADDR	0x024	/* getAddress() & 0x7FFFFFFF */
#define S5L8740_COMP_L0_W	0x028	/* 240 */
#define S5L8740_COMP_L0_POS	0x02c	/* y | (x << 16) */
#define S5L8740_COMP_L0_SIZE	0x030	/* 0x00f001b0 = 240 << 16 | 432, LCDIF SIZE packing */
#define S5L8740_COMP_L0_34	0x034
#define S5L8740_COMP_L0_38	0x038
#define S5L8740_COMP_L0_3C	0x03c	/* low 12 bits = 96, sub_45690 */
#define S5L8740_COMP_L1_5C	0x05c	/* low 12 bits = 96, sub_45690 */
#define S5L8740_COMP_114	0x114	/* 0x00100010, sub_45690 */
#define S5L8740_COMP_118	0x118	/* low 3 bits = 3, sub_45690 */
#define S5L8740_COMP_2E0	0x2e0	/* 0x50 */
#define S5L8740_COMP_2E4	0x2e4	/* 0x50 */
#define S5L8740_COMP_1B10	0x1b10	/* |= 0x10000, sub_45690 */
#define S5L8740_COMP_1B10_BIT	BIT(16)
#define S5L8740_COMP_1B10_EN	BIT(0)	/* sub_53E64 */
#define S5L8740_COMP_CTRL_BIT7	BIT(7)	/* sub_53AC8 */
#define S5L8740_COMP_100	0x100	/* read once at bring-up, sub_45690 */
/*
 * Output control, sub_540BC(0, 0, 0x10101, 1) -> 0x81110001: bit 31 set,
 * bit 29 clear, bit 28 clear, bits 25:24 = 01, bits 21:20 = 01, bits
 * 17:16 = 00, bit 0 = 1, everything else read-modify-write.
 */
#define S5L8740_COMP_OUT300	0x300
#define S5L8740_COMP_OUT304	0x304	/* sub_549E4(8, 8, 8) -> 0x00080808 */
/*
 * +0x400 is the register sub_427788 reaches through a veneer the decomp
 * shows as a bare no-arg call. Disassembly: r0 selects the case, r1 the
 * bit value. (4, 1) sets bit 0 at bring-up; (0, 1) and (1, 1) set bits 2
 * and 1 when the CABC block starts. The capture holds 0x00000001.
 */
#define S5L8740_COMP_400	0x400
/* LUT loader, sub_52BAC: index word then data word, five tables. */
#define S5L8740_COMP_LUT_IDX	0x408
#define S5L8740_COMP_LUT_DATA	0x40c
/* Output window, sub_4CEE0: 0x1b24 = (h - 1) | (w - 1) << 16. */
#define S5L8740_COMP_1B14	0x1b14
#define S5L8740_COMP_1B18	0x1b18
#define S5L8740_COMP_1B1C	0x1b1c
#define S5L8740_COMP_1B20	0x1b20
#define S5L8740_COMP_1B24	0x1b24
/*
 * The content-adaptive block at +0x1b30.., programmed by sub_4BB118 ->
 * sub_A49BC once the UI is up. +0x1b34 is read back by a periodic task
 * (sub_779D4) and mapped to a backlight level; the table words below are
 * sub_B5F78 / sub_B5FF0 / sub_B5F08 applied to the firmware's tables at
 * 0x8775CB4 / 0x8775CE0 / 0x8775D0C, which reproduce the captured words
 * exactly (checked 2026-09-07).
 */
#define S5L8740_COMP_CABC_CTRL	0x1b30	/* bit 0 start (sub_A4958), bit 1 (sub_4BB074), bit 5 (sub_AEAE0) */
#define S5L8740_COMP_CABC_38	0x1b38	/* bit 8 (sub_B609C), low byte (sub_A49A8) */
#define S5L8740_COMP_CABC_50	0x1b50
#define S5L8740_COMP_CABC_54	0x1b54
#define S5L8740_COMP_CABC_58	0x1b58
#define S5L8740_COMP_CABC_5C	0x1b5c
#define S5L8740_COMP_CABC_60	0x1b60
#define S5L8740_COMP_CABC_70	0x1b70	/* sub_847E8 */
#define S5L8740_COMP_CABC_74	0x1b74
#define S5L8740_COMP_CABC_78	0x1b78	/* sub_847B8 */
#define S5L8740_COMP_CABC_7C	0x1b7c
#define S5L8740_COMP_CABC_88	0x1b88	/* sub_B5F78, six words */
#define S5L8740_COMP_CABC_A0	0x1ba0	/* sub_B5FF0, six words */
#define S5L8740_COMP_CABC_B8	0x1bb8	/* sub_B5F08, 11 x 3 words */
#define S5L8740_COMP_FRAME_BYTES (WIDTH * HEIGHT * 4)
/* Stock's two buffers are 414,848 apart: a frame plus 128 bytes. */
#define S5L8740_COMP_BUF_BYTES	(S5L8740_COMP_FRAME_BYTES + 128)

#define WIDTH 240
#define HEIGHT 432

/*
 * Frame DMA.
 *
 * Stock never moves a pixel with the CPU. The pixel FIFO is registered in
 * the OSOS DMA peripheral tables as a peripheral in its own right, and
 * whole frames are pushed into it by the PL080:
 *
 *   0x0891DC14[0]  = 0x38300040            peripheral FIFO address table
 *   0x0891DC98[0]  = 0x01                  serviceable by DMAC0 only
 *   0x0891DCB8[0*8 + 1*4] = 0x00000003     DMAC0 request line 3
 *
 * The last table is the one that matters for the device tree, and it is
 * the reason this node asks for line 3 rather than 0. sub_B424C at
 * 0x080B424C builds CxConfig as
 *
 *   0x080B437E  r1 = *(u32 *)(0x0891DCBC + dst_id*8 + ctrl*4 - 4)
 *   0x080B43D6  cfg = flow | ((r1 & 0xF) << 6) | ((r0 & 0xF) << 1) | 0x8001
 *
 * so the byte in the software peripheral tables (id 0 for the LCDIF) is an
 * index, not the hardware request line. Cross-checked on the two lines this
 * board already uses: IIS0 TX is table id 12 and resolves to line 0x0A,
 * IIS2 RX is table id 17 and resolves to line 0x0D -- exactly the 10 and 13
 * the device tree names for i2s0 and i2s2.
 *
 * With src id 32 (memory) and dst id 0, sub_BAAE0 returns 0x800, so stock's
 * channel runs at CFG = 0x800 | (3 << 6) | 0x8001 = 0x000088C1.
 *
 * Transfer parameters come from the tables at 0x0891DB78 with the argument
 * quartet sub_47B0 passes at 0x080047EC (SWidth 2, DWidth 2, SBSize 3,
 * DBSize 3):
 *
 *   SWidth[2] @0x0891DB9C = 0x00080000     32-bit source  (2 << 18)
 *   DWidth[2] @0x0891DBA8 = 0x00400000     32-bit dest    (2 << 21)
 *   SBSize[3] @0x0891DBB8 = 0x00001000     burst code 1 = 4 beats
 *   DBSize[3] @0x0891DBEC = 0x00008000     burst code 1 = 4 beats
 *
 * and sub_A4F94 ORs them with 0x80000000 into CxControl at 0x080A5060.
 */
#define S5L8740_LCD_WDATA_PA	0x38300040u

/*
 * Bytes per descriptor. sub_47B0 at 0x080047BE loads r8 = 16380 and clamps
 * every chunk to it; the count register takes 0x3FFC >> DWidth = 4095
 * transfers, which is the PL080's 12-bit limit. A 240x432 XRGB8888 frame is
 * 414720 bytes, so 26 descriptors -- 25 full ones and a 5220-byte tail.
 */
#define S5L8740_DMA_SEG_BYTES	0x3ffcu

/* sub_47B0 waits 100 ms per chunk at 0x0800481A (sub_B65F4(ctrl, ch, 100)). */
#define S5L8740_DMA_TIMEOUT_MS	100

/*
 * Simple Framebuffer device
 */

 struct s5l8740_device {
	struct drm_device dev;

	/* simplefb settings */
	struct drm_display_mode mode;
    const struct drm_format_info *format;

    /* memory management */
	void __iomem *lcdif;
	void __iomem *dsi;	/* MIPI DSI host; NULL = no windowing */
	void __iomem *clkcon;	/* gates cycled across an LCDIF reset */

	/* display power */
	struct mutex power_lock;
	bool powered;
	bool handoff_armed;	/* CON/+0x70 raised to stock's on a handoff */
	/* The layer compositor at 0x38900000; see S5L8740_COMP_*. */
	void __iomem *comp;
	void *cbuf[2];
	dma_addr_t cbuf_dma[2];
	unsigned int cbuf_order;
	unsigned int cbuf_back;	/* the buffer the next frame is staged into */
	bool comp_ready;
	bool comp_failed;
	unsigned int comp_kicks;
	unsigned int comp_skips;	/* gate was busy at kick time */
	unsigned int comp_waits;	/* waited for a transfer to finish before staging */
	/*
	 * Serialises the frame path against the retry. Both run in process
	 * context; the kick polls the DSI host for up to 2 ms, so this is a
	 * mutex and not a spinlock.
	 */
	struct mutex comp_lock;
	struct delayed_work comp_retry;
	bool comp_pending;	/* +0x24 points at a frame no kick has shown yet */
	unsigned int comp_retries;
	bool dma_gave_up;	/* request never asserted; stop paying the timeout */
	unsigned int dma_fails;
	bool rail_held;

	/*
	 * Frame DMA. The channel is taken lazily on the first update, not in
	 * probe: this driver is built in and dma-s5l8740-pl080 is a module,
	 * so at probe time the controller has not registered yet and
	 * dma_request_chan() would only ever defer. Deferring would hold the
	 * console off the panel until the rootfs is up, which is a worse
	 * failure than a few PIO frames at boot.
	 */
	struct mutex dma_lock;
	struct dma_chan *dma;
	struct device *dma_dev;		/* the device the buffer is mapped for */
	bool dma_ready;
	bool dma_failed;
	void *fbuf;			/* staging frame, physically contiguous */
	dma_addr_t fbuf_dma;
	size_t fbuf_size;
	unsigned int fbuf_order;
	struct scatterlist *sgl;
	unsigned int nsegs;
	struct completion dma_done;

	/* modesetting */
    uint32_t formats[8];
    size_t nformats;
    struct drm_plane primary_plane;
    struct drm_crtc crtc;
    struct drm_encoder encoder;
    struct drm_connector connector;
};

/*
 * MIPI DSI host, a separate block from the LCDIF.
 *
 * The panel is a DCS command-mode panel with full windowed addressing.
 * Before each transfer stock sends set_column_address, set_page_address and
 * write_memory_start through this host, and only then streams pixels into
 * LCDIF. RetailOS never writes LCD_WDATA without doing that first.
 *
 * Verified in the decomp. The short and long paths genuinely wait on the
 * status register differently, which is not a transcription slip:
 *
 *   short: MEMORY[0x3D800034] = dt & 0x3F | (p0 << 8) | (p1 << 16);
 *          wait until (status & 0x400000) == 0            bit 22 clears
 *   long:  MEMORY[0x3D800038] = payload words, LSB first;
 *          MEMORY[0x3D800034] = dt & 0x3F | (len << 8);
 *          wait until (status & 0x500000) == 0x500000     bits 20+22 set
 */
#define S5L8740_DSI_CTL			0x10
#define S5L8740_DSI_CTL_LONG_GATE	BIT(28)
#define S5L8740_DSI_HDR			0x34
#define S5L8740_DSI_PAYLOAD		0x38
#define S5L8740_DSI_STATUS		0x44
#define S5L8740_DSI_ST_SHORT		0x400000u
#define S5L8740_DSI_ST_LONG		0x500000u
#define S5L8740_DSI_TIMEOUT_US		2000

#define S5L8740_DSI_DT_DCS_SHORT_0P	0x05
#define S5L8740_DSI_DT_GEN_LONG		0x29

#define S5L8740_DCS_SET_COLUMN		0x2a
#define S5L8740_DCS_SET_PAGE		0x2b
#define S5L8740_DCS_WRITE_START		0x2c

static inline void s5l8740_lcd_writel(struct s5l8740_device *lcd_dev,
					  u32 reg, u32 val)
{
	writel(val, lcd_dev->lcdif + reg);
}

static struct s5l8740_device *s5l8740_device_of_dev(struct drm_device *dev)
{
	return container_of(dev, struct s5l8740_device, dev);
}

static int s5l8740_primary_plane_helper_atomic_check(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc *new_crtc = new_plane_state->crtc;
	struct drm_crtc_state *new_crtc_state = NULL;

	if (new_crtc)
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_crtc);

	return drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   false, false);
}

/*
 * The stock sequence waits between enabling the rail and touching the
 * LCDIF. That wait is a thunk into ROM, so its length is not recoverable;
 * this is a conservative stand-in, tunable if the panel proves fussy.
 */
/*
 * Send the whole panel on every update, the way stock does, rather than
 * only the rectangles DRM reports as damaged.
 *
 * Off is faster and was the default while the drift was being chased. It
 * is kept because it is the one knob that reproduces the fault, and a
 * regression here is otherwise hard to tell from a slow frame.
 */
static bool lcd_partial;
module_param(lcd_partial, bool, 0644);
MODULE_PARM_DESC(lcd_partial,
		 "Send only damaged rectangles instead of the whole panel");

/*
 * Whether to cycle the LCDIF transfer gate (+0x80) around each update.
 *
 * Off, because that is what stock's pixel-push path does. Every function
 * in the image that writes the gate is sub_A25C0, and sub_A25C0 is not on
 * the path that pushes pixels through the LCDIF: sub_2FEC -- window via
 * sub_75D44, then DMA to 0x38300040 via sub_47B0 -- never touches it. The
 * gate belongs to the runtime path instead: sub_76BBC programs the layer
 * compositor at 0x38900000 through sub_825C4 (three layers, each with a
 * DRAM buffer address, position, size and stride, enabled in +0x04) and
 * then calls sub_828F8 -> sub_A25C0 to open the gate, send the window,
 * wait two microseconds and close it. That cycle is the frame kick for a
 * compositor that scans DRAM by itself.
 *
 * This driver uses the pixel-push path and has never programmed a layer.
 * Cycling the gate before its own pixels arrive therefore kicks a frame
 * from layers that hold nothing, which the panel shows as black until the
 * DMA lands over it. Reported 2026-09-07 as a black flash between
 * refreshes; consistent, not yet measured on the panel.
 *
 * On restores the cycle for comparison.
 */
static bool lcd_frame_xfer;
module_param(lcd_frame_xfer, bool, 0644);
MODULE_PARM_DESC(lcd_frame_xfer,
		 "Cycle the LCDIF transfer gate around each update (default N; stock's push path does not)");

/*
 * Draw through the layer compositor at 0x38900000, the way stock draws at
 * runtime: stage the frame into one of two DRAM buffers, point layer 0 at
 * it, and cycle the LCDIF gate once as the frame kick. No PL080, no 26
 * segments. Off falls back to the pixel-push path above, which is stock's
 * boot-logo mechanism and still works.
 */
static bool lcd_compositor = true;
module_param(lcd_compositor, bool, 0644);
MODULE_PARM_DESC(lcd_compositor,
		 "Draw via the 0x38900000 layer compositor as stock does (default Y; N = PL080 pixel push)");

/*
 * Program the content-adaptive block at +0x1b30.. after bring-up, as
 * sub_4BB118 does once the UI is up.
 *
 * Off by default, and that is a divergence from stock. Stock's start
 * (sub_A4958 -> sub_9BEA0) also switches the PMIC's white-LED driver to
 * take dimming from the single-wire block at 0x3DE00000; this kernel does
 * neither, gpio-d1830 owns the PMIC bus. Without that consumer the engine
 * never completes: +0x1b30 bit 0 and +0x400 bits 2:1 stay set where the
 * capture has them clear, and the output is transformed -- measured with
 * colour bars 2026-09-07: red -> green, black -> yellow, grey -> purple.
 * Restoring the block to its pre-programming words (lcd_comp_reinit=4)
 * gave correct bars at once.
 */
static bool lcd_cabc;
module_param(lcd_cabc, bool, 0644);
MODULE_PARM_DESC(lcd_cabc,
		 "Program the compositor's +0x1b30 content-adaptive block as stock does (default N: without the PMIC/single-wire consumer it transforms the output)");

/*
 * Where the transfer gate closes.
 *
 * Stock closes it before the pixels. sub_A25C0 at 0x080A25C0 is the whole
 * of it -- busy check on 0x3830008C, 0x38300080 = 1, the DCS window, a
 * settle loop, 0x38300080 = 0 -- and it sends no pixel data at all. The
 * path that does send pixels, sub_3034 at 0x08003034, calls the window
 * setter and then sub_47B0 without writing 0x38300080 anywhere.
 *
 * This driver used to hold the gate open across the entire pixel stream.
 * That was its own invention. It is kept behind this parameter because it
 * is the behaviour every frame on the glass so far was drawn with, so a
 * regression can be attributed rather than guessed at.
 */
static bool lcd_xfer_hold;
module_param(lcd_xfer_hold, bool, 0644);
MODULE_PARM_DESC(lcd_xfer_hold,
		 "Hold the LCDIF transfer gate across the pixel stream (pre-DMA behaviour; stock closes it after the window)");

/*
 * Push frames with the PL080, the way stock does.
 *
 * N falls back to the per-pixel programmed-I/O loop this driver shipped
 * with: 240 x 432 poll-then-write pairs, 207360 device round trips per
 * frame with the CPU spinning inside the atomic commit. It is kept so the
 * two paths can be compared on hardware without a rebuild.
 *
 * DEFAULT OFF as of the 2026-09-04 hardware run, which is not a retreat from
 * the finding -- stock really does DMA frames and the addressing is right --
 * but the LCDIF does not assert its DMA request for us, so every frame times
 * out and falls back. Leaving it on costs a 100 ms stall per frame before the
 * PIO path draws anyway, which is strictly worse than PIO alone.
 *
 * What the hardware said: the channel binds (dma0chan1 -> 38300000.lcdif:tx),
 * the geometry is right (26 segments of 0x3FFC to 0x38300040), and then
 * exactly 16 bytes move -- residue 414704 of 414720. Sixteen bytes is the
 * PL080's FIFO prefetch and nothing more, the same signature this tree's own
 * DTS comment records for a stalled request line ("src advanced only the
 * 16-byte prefetch").
 *
 * Ruled out, so nobody re-runs them: a longer timeout (16 bytes is not
 * "nearly finished"), the transfer gate (tested live via lcd_xfer_hold, no
 * change, and sub_3034 never writes 0x38300080 anyway), an SRAM-resident
 * source buffer (a frame is 405 KiB against 192 KiB of SRAM, so stock cannot
 * be doing it either), and a missing LCDIF init register
 * (s5l8740_lcdif_program() writes all ten of sub_2C64's).
 *
 * Still unknown: what arms the LCDIF's DMA request. sub_429B4 sets CON bit 10
 * together with 0x38300070 = 1 and is the best remaining candidate, but both
 * its callers pass a stored setting applied at display power-on rather than a
 * per-transfer arm, so it is a lead and not an answer. Set lcd_dma=Y to
 * experiment; the PIO fallback means a failure costs latency, not pixels.
 */
/*
 * Back on as of 2026-09-06, because the reason it was switched off has been
 * addressed: sub_2C64(0)'s power-domain call is now made (see
 * s5l8740_lcdif_program()), and a domain that was never brought up is a
 * credible reason for a peripheral to answer register writes while never
 * asserting its DMA request.
 *
 * If the request still does not assert, the failure is the same as before
 * and it is not silent: every frame logs, stalls for lcd_dma_timeout_ms and
 * then draws by PIO, so the panel keeps working and the log says why. Set
 * lcd_dma=N to go straight back to PIO.
 */
/*
 * OFF again as of 2026-09-07, and this time the reason is the software
 * burst request rather than the absence of one.
 *
 * s5l_pl080_soft_req() gave the channel a request the LCDIF never
 * asserts, and the transfer stopped timing out. That removed the safety
 * this parameter was left on for: the fallback above only runs when
 * s5l8740_dma_frame() returns an error, so a DMA that half works draws a
 * corrupt frame and reports success, where a DMA that fails outright drew
 * a correct one by PIO.
 *
 * Measured on the panel: the top of the frame is noise and the bottom is
 * correct, the boundary moves with CPU load, and throttling the pump to
 * one outstanding request made the corrupt band grow rather than shrink.
 * So the bursts are not paced by anything the LCDIF agrees with, and
 * until they are, this path costs pixels instead of latency.
 *
 * The DMA path is unchanged and still selectable with lcd_dma=Y. What is
 * not yet known is what paces the LCDIF's consumption; the free-running
 * kthread in s5l_pl080_pump() is a placeholder for that answer, not the
 * answer.
 */
static bool lcd_dma = true;

/*
 * Arm the LCDIF request generator the way sub_429B4(1) does -- CON
 * BIT(10) plus +0x70 = 1 -- before a DMA frame. Measured on a live
 * RetailOS with a working panel; see s5l8740_lcdif_program().
 */
static bool lcd_request_arm = true;
module_param(lcd_request_arm, bool, 0644);
MODULE_PARM_DESC(lcd_request_arm,
		 "Set CON BIT(10) and +0x70=1 on the DMA path, as sub_429B4 does (default Y)");
module_param(lcd_dma, bool, 0644);
MODULE_PARM_DESC(lcd_dma,
		 "Stream frames through the PL080 like stock (default Y); N = per-pixel PIO");

/*
 * How long to wait for a whole frame.
 *
 * S5L8740_DMA_TIMEOUT_MS is stock's number, but stock spends it *per chunk*:
 * sub_47B0 acquires a channel, moves one 0x3FFC-byte piece, and waits
 * sub_B65F4(ctrl, ch, 100) at 0x0800481A before releasing and starting the
 * next. We submit all 26 segments as one LLI chain and wait once, so the
 * same 100 ms covers what stock would allow 2600 ms for. Adjustable rather
 * than simply raised, because a frame that never starts and a frame that
 * runs out of budget want opposite fixes -- see the residue in the timeout
 * warning.
 */
static unsigned int lcd_dma_timeout_ms = S5L8740_DMA_TIMEOUT_MS;
module_param(lcd_dma_timeout_ms, uint, 0644);
MODULE_PARM_DESC(lcd_dma_timeout_ms,
		 "Milliseconds to wait for one DMA frame (default 100, stock's per-chunk budget)");

/* Wait for the interface to leave a transfer. */
/*
 * Stock's test before it opens the gate, sub_A25C0 at 0x0A25DE:
 *
 *	if (MEMORY[0x3830008C] << 30) return -1;
 *
 * A check, not a wait. It refuses and lets the caller come back later.
 *
 * This used to be a readl_poll_timeout_atomic() on the same bits with a
 * 100 ms budget, and it was called from the close path too. Sampled on
 * 2026-09-07 at 50 ms for five seconds, the damage worker was inside it
 * on 85 of 100 samples with +0x8c reading 0x0FA00301 on all 100 -- bit 0
 * set because the gate was open, which is what the close path was waiting
 * for it to stop being. Every commit burned the full budget. That was the
 * kworker at 92% and the panel that stopped refreshing once frames started
 * arriving quickly enough to queue.
 */
static int s5l8740_xfer_idle(struct s5l8740_device *sdev)
{
	if (readl(sdev->lcdif + S5L8740_LCD_XSTAT) & S5L8740_LCD_XSTAT_BUSY)
		return -EBUSY;
	return 0;
}

static int s5l8740_xfer_open(struct s5l8740_device *sdev)
{
	int ret;

	if (!lcd_frame_xfer)
		return 0;
	/*
	 * Opening on top of a transfer still open or active is what stock
	 * refuses outright, returning -1 rather than queueing behind it. The
	 * refusal is reported every time it happens, not once: a frame that
	 * is dropped here is a frame the user does not see, and the rate at
	 * which that happens is a number this driver needs to be able to
	 * read back.
	 */
	ret = s5l8740_xfer_idle(sdev);
	if (ret) {
		dev_warn_ratelimited(sdev->dev.dev,
				     "[drm] LCDIF gate refused: XSTAT=%08x\n",
				     readl(sdev->lcdif + S5L8740_LCD_XSTAT));
		return ret;
	}
	writel(1, sdev->lcdif + S5L8740_LCD_XFER);
	return 0;
}

static void s5l8740_xfer_close(struct s5l8740_device *sdev)
{
	if (!lcd_frame_xfer)
		return;
	/*
	 * sub_A25C0, 0x0A2604..0x0A2626: after the window command it takes a
	 * timestamp, spins until two ticks have elapsed, and writes the gate
	 * to 0. A tick is one microsecond (see the 0x8983AFA derivation in
	 * dma-s5l8740-pl080.c). No status is consulted.
	 *
	 * The wait that used to be here polled +0x8c for bits 1:0 to clear
	 * first, on the reading that they meant "draining". Bit 0 is the gate
	 * itself, so that wait could only end by timing out, and it did, on
	 * every commit, for 100 ms each.
	 */
	udelay(2);
	writel(0, sdev->lcdif + S5L8740_LCD_XFER);
}

/* Short DCS write, no parameters. */
static int s5l8740_dsi_short(struct s5l8740_device *sdev, u8 dt, u8 cmd)
{
	u32 st;

	if (!sdev->dsi)
		return -ENODEV;
	writel((dt & 0x3f) | (cmd << 8), sdev->dsi + S5L8740_DSI_HDR);
	/*
	 * Flush the header write and let the host react before polling.
	 *
	 * The done bits are set while the host is idle -- measured
	 * 0x0155541D with nothing in flight -- so they only mean anything
	 * after issuing a command has cleared them. Polling straight after a
	 * posted write reads the stale idle value, returns success
	 * immediately, and the next command then pushes its payload over one
	 * still in flight.
	 *
	 * That is what put the panel window wrong in one axis only: the
	 * column write survived because setting up the page write gave it
	 * time, and the page write was clobbered by write_memory_start
	 * arriving on its heels. Horizontal placement correct, vertical
	 * placement wherever the previous command happened to leave it.
	 */
	readl(sdev->dsi + S5L8740_DSI_STATUS);
	udelay(1);
	/*
	 * Wait for the bit to SET, not clear.
	 *
	 * The two waits stock uses are opposites and it is easy to read one
	 * as the other:
	 *
	 *   sub_3D11B0(r, m, v)  while ((*r & m) == v)  -- until NOT equal
	 *   sub_714C  (r, m, v)  while ((*r & m) != v)  -- until equal
	 *
	 * The short path calls sub_3D11B0(status, 0x400000, 0), so it waits
	 * until bit 22 is no longer zero. Waiting for it to clear instead
	 * hangs forever: the bit reads 1 at idle -- measured 0x0155541D on
	 * the glass -- and only drops while a command is in flight.
	 */
	return readl_poll_timeout_atomic(sdev->dsi + S5L8740_DSI_STATUS, st,
					 st & S5L8740_DSI_ST_SHORT, 0,
					 S5L8740_DSI_TIMEOUT_US);
}

/* Long write: payload first, LSB-first into the FIFO, then the header. */
static int s5l8740_dsi_long(struct s5l8740_device *sdev, u8 dt,
			    const u8 *buf, unsigned int len)
{
	unsigned int i;
	u32 w = 0, st;

	if (!sdev->dsi)
		return -ENODEV;
	writel(readl(sdev->dsi + S5L8740_DSI_CTL) & ~S5L8740_DSI_CTL_LONG_GATE,
		       sdev->dsi + S5L8740_DSI_CTL);

	for (i = 0; i < len; i++) {
		w |= (u32)buf[i] << (8 * (i & 3));
		if ((i & 3) == 3) {
			writel(w, sdev->dsi + S5L8740_DSI_PAYLOAD);
			w = 0;
		}
	}
	if (len & 3)
		writel(w, sdev->dsi + S5L8740_DSI_PAYLOAD);

	writel((dt & 0x3f) | (len << 8), sdev->dsi + S5L8740_DSI_HDR);
	/*
	 * Flush the header write and let the host react before polling.
	 *
	 * The done bits are set while the host is idle -- measured
	 * 0x0155541D with nothing in flight -- so they only mean anything
	 * after issuing a command has cleared them. Polling straight after a
	 * posted write reads the stale idle value, returns success
	 * immediately, and the next command then pushes its payload over one
	 * still in flight.
	 *
	 * That is what put the panel window wrong in one axis only: the
	 * column write survived because setting up the page write gave it
	 * time, and the page write was clobbered by write_memory_start
	 * arriving on its heels. Horizontal placement correct, vertical
	 * placement wherever the previous command happened to leave it.
	 */
	readl(sdev->dsi + S5L8740_DSI_STATUS);
	udelay(1);
	return readl_poll_timeout_atomic(sdev->dsi + S5L8740_DSI_STATUS, st,
					 (st & S5L8740_DSI_ST_LONG) ==
					 S5L8740_DSI_ST_LONG, 0,
					 S5L8740_DSI_TIMEOUT_US);
}

/* Point the panel at one rectangle and rewind its write pointer. */
static int s5l8740_dsi_window(struct s5l8740_device *sdev,
			      u32 x, u32 y, u32 w, u32 h)
{
	u8 col[5] = { S5L8740_DCS_SET_COLUMN, (x >> 8) & 0xff, x & 0xff,
		      ((x + w - 1) >> 8) & 0xff, (x + w - 1) & 0xff };
	u8 row[5] = { S5L8740_DCS_SET_PAGE, (y >> 8) & 0xff, y & 0xff,
		      ((y + h - 1) >> 8) & 0xff, (y + h - 1) & 0xff };
	int ret;

	ret = s5l8740_dsi_long(sdev, S5L8740_DSI_DT_GEN_LONG, col, sizeof(col));
	if (ret)
		return ret;
	ret = s5l8740_dsi_long(sdev, S5L8740_DSI_DT_GEN_LONG, row, sizeof(row));
	if (ret)
		return ret;

	return s5l8740_dsi_short(sdev, S5L8740_DSI_DT_DCS_SHORT_0P,
				 S5L8740_DCS_WRITE_START);
}

/*
 * Set the window the way sub_A25C0 does at 0x080A25C0:
 *
 *   if (MEMORY[0x3830008C] << 30)  return -1;   busy: bits 0-1 set
 *   MEMORY[0x38300080] = 1;                     open
 *   sub_AEFAC(0, 0, w, h);                      DCS 2A / 2B / 2C
 *   do sub_345D68(); while (!v3);               settle
 *   MEMORY[0x38300080] = 0;                     close
 *
 * The gate is shut again before any pixel moves. Stock's pixel path,
 * sub_3034 at 0x08003034, does not touch 0x38300080 at all.
 *
 * Returns nonzero only when the interface would not let the window be set
 * at all. A window command that times out is not fatal and falls through to
 * the pixels: the panel keeps whatever window it had, and one crooked frame
 * beats a frozen display.
 */
static int s5l8740_window(struct s5l8740_device *sdev,
			  const struct drm_rect *clip)
{
	int ret;

	ret = s5l8740_xfer_open(sdev);
	if (ret)
		return ret;

	if (sdev->dsi &&
	    s5l8740_dsi_window(sdev, clip->x1, clip->y1,
			       drm_rect_width(clip), drm_rect_height(clip)))
		drm_warn_once(&sdev->dev, "DSI window timeout\n");

	if (!lcd_xfer_hold)
		s5l8740_xfer_close(sdev);
	return 0;
}

/*
 * Point the panel at one rectangle and stream it with the CPU.
 *
 * This is not what stock does -- see s5l8740_dma_frame() -- and it is
 * reachable only with lcd_dma=N. Returns nonzero when the interface stopped
 * answering, which costs the rest of the frame.
 */
static int s5l8740_blit(struct s5l8740_device *sdev, const u32 *src,
			unsigned int pitch_px, const struct drm_rect *clip)
{
	unsigned int x, y;

	if (s5l8740_window(sdev, clip))
		return -EBUSY;

	for (y = clip->y1; y < (unsigned int)clip->y2; y++) {
		const u32 *row = src + y * pitch_px;

		for (x = clip->x1; x < (unsigned int)clip->x2; x++) {
			u32 status;

			if (readl_poll_timeout_atomic(sdev->lcdif +
						      S5L8740_LCD_STATUS,
						      status,
						      !(status & S5L8740_LCD_STATUS_BUSY),
						      0, S5L8740_LCD_TIMEOUT_US)) {
				drm_warn_once(&sdev->dev,
					      "LCDIF stayed busy; frame dropped\n");
				return -ETIMEDOUT;
			}

			s5l8740_lcd_writel(sdev, S5L8740_LCD_WDATA, row[x]);
		}
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Frame DMA                                                            */
/* ------------------------------------------------------------------ */

static void s5l8740_dma_release_chan(void *data)
{
	struct dma_chan *chan = data;

	dmaengine_terminate_sync(chan);
	dma_release_channel(chan);
}

static void s5l8740_dma_unmap(void *data)
{
	struct s5l8740_device *sdev = data;

	dma_unmap_single(sdev->dma_dev, sdev->fbuf_dma, sdev->fbuf_size,
			 DMA_TO_DEVICE);
}

static void s5l8740_dma_free_pages(void *data)
{
	struct s5l8740_device *sdev = data;

	free_pages((unsigned long)sdev->fbuf, sdev->fbuf_order);
}

/*
 * Take the channel, the staging buffer and the descriptor list.
 *
 * The staging buffer exists because the plane is drm_gem_shmem: its pages
 * are scattered, and the PL080 driver's own cache maintenance
 * (s5l_pl080_sync_buffer) syncs one range starting at the first
 * scatterlist entry, so a scattered source would leave most of the frame
 * unflushed. A physically contiguous buffer makes that sync exactly right.
 *
 * It is also what stock does. sub_3034 at 0x08003034 allocates 0x19500
 * words, fills them, and hands the buffer to sub_47B0 at 0x080047B0; the
 * LCDIF has no framebuffer-base or stride register anywhere in the image,
 * so there is no in-place path to have instead.
 */
static int s5l8740_dma_setup(struct s5l8740_device *sdev)
{
	struct device *dev = sdev->dev.dev;
	struct dma_slave_config cfg = {};
	struct dma_chan *chan;
	unsigned int i;
	size_t off;
	int ret;

	chan = dma_request_chan(dev, "tx");
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = devm_add_action_or_reset(dev, s5l8740_dma_release_chan, chan);
	if (ret)
		return ret;

	/*
	 * Exactly the parameters stock's tables at 0x0891DB78 produce for the
	 * argument quartet sub_47B0 passes at 0x080047EC: 32-bit on both
	 * sides, burst code 1 (4 beats) on both sides. The PL080 driver
	 * encodes maxburst 4 as code 1 (s5l_pl080_burst_enc), so the control
	 * word it builds is 0x84489000 -- stock's 0x80489000 from sub_A4F94
	 * plus the source-increment bit sub_B424C ORs in at 0x080B4394.
	 */
	cfg.direction = DMA_MEM_TO_DEV;
	cfg.dst_addr = S5L8740_LCD_WDATA_PA;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.dst_maxburst = 4;
	cfg.src_maxburst = 4;
	ret = dmaengine_slave_config(chan, &cfg);
	if (ret)
		return ret;

	/*
	 * Ask the DMAC to raise this channel's bursts itself.
	 *
	 * The question this file has carried for days is what arms the
	 * LCDIF's DMA request. A capture of a RetailOS that is drawing says
	 * the answer is nothing: DMAC0's SoftBReq holds 0x8, a pending burst
	 * request on line 3, which is this channel's, against a channel 3
	 * whose CONFIG has En=0. Stock puts the request there by hand.
	 *
	 * That fits every measurement taken here. The channel binds, the
	 * geometry is right, and its CONTROL word (0x84489000) and CONFIG
	 * (0x88C0, plus the enable bit) match that capture register for
	 * register, and then exactly the PL080's FIFO prefetch moves and it
	 * stops -- because nothing ever asks for the next burst.
	 */
	/*
	 * No software burst pump.
	 *
	 * The reasoning that put one here was that a capture of a drawing
	 * RetailOS shows DMAC0's SoftBReq holding 0x8 against a channel with
	 * En=0, so stock must place the request by hand. The arithmetic
	 * refutes it. This channel's CONTROL word is 0x84489000, whose DBSize
	 * and SBSize fields are both 0b001 -- four transfers. One SoftBReq
	 * write therefore moves sixteen bytes, so a 4095-transfer segment
	 * needs 1024 of them and a frame needs 26624. Nothing paces that from
	 * a kthread, and the panel said so: the corrupt band at the top of
	 * the frame moved with CPU load, and throttling the pump to one
	 * outstanding request made it grow rather than shrink.
	 *
	 * A single pending bit in a capture says what was outstanding at that
	 * instant. It does not say the rate stock sustains, and it never
	 * showed that stock sustains one at all.
	 *
	 * What actually left the request unasserted was this driver never
	 * arming the generator on an inherited panel; see
	 * s5l8740_lcdif_arm_handoff(). Armed, the LCDIF raises DMACBREQ
	 * itself, which is the only thing that can keep pace with its own
	 * FIFO.
	 *
	 * If it still does not assert, the failure is loud and cheap instead
	 * of silent and corrupt: the segment times out, s5l8740_dma_frame()
	 * returns an error, and the caller draws that frame by PIO. A frame
	 * that half arrives is worse than one that does not arrive at all,
	 * because only the second one falls back.
	 */

	sdev->fbuf_size = (size_t)WIDTH * HEIGHT * 4;
	sdev->fbuf_order = get_order(sdev->fbuf_size);
	sdev->fbuf = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
					      sdev->fbuf_order);
	if (!sdev->fbuf)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, s5l8740_dma_free_pages, sdev);
	if (ret)
		return ret;

	/*
	 * Map for the controller, not for this device: the PL080 is the one
	 * that reads the buffer, and it is the device its driver syncs
	 * against in s5l_pl080_start().
	 */
	sdev->dma_dev = dmaengine_get_dma_device(chan);
	sdev->fbuf_dma = dma_map_single(sdev->dma_dev, sdev->fbuf,
					sdev->fbuf_size, DMA_TO_DEVICE);
	if (dma_mapping_error(sdev->dma_dev, sdev->fbuf_dma))
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, s5l8740_dma_unmap, sdev);
	if (ret)
		return ret;

	/* 26 descriptors: 25 of 0x3FFC bytes and a 5220-byte tail. */
	sdev->nsegs = DIV_ROUND_UP(sdev->fbuf_size, S5L8740_DMA_SEG_BYTES);
	sdev->sgl = devm_kcalloc(dev, sdev->nsegs, sizeof(*sdev->sgl),
				 GFP_KERNEL);
	if (!sdev->sgl)
		return -ENOMEM;

	sg_init_table(sdev->sgl, sdev->nsegs);
	for (i = 0, off = 0; i < sdev->nsegs; i++) {
		size_t len = min_t(size_t, S5L8740_DMA_SEG_BYTES,
				   sdev->fbuf_size - off);

		/*
		 * Pre-mapped: the buffer was mapped once above and is only
		 * re-synced per frame, so the scatterlist carries the device
		 * addresses directly rather than going through dma_map_sg().
		 */
		sg_set_buf(&sdev->sgl[i], sdev->fbuf + off, len);
		sg_dma_address(&sdev->sgl[i]) = sdev->fbuf_dma + off;
		sg_dma_len(&sdev->sgl[i]) = len;
		off += len;
	}

	drm_info(&sdev->dev,
		 "frame DMA on %s: %u segments of up to %u bytes to %#x\n",
		 dma_chan_name(chan), sdev->nsegs, S5L8740_DMA_SEG_BYTES,
		 S5L8740_LCD_WDATA_PA);

	sdev->dma = chan;
	sdev->dma_ready = true;
	return 0;
}

/* True once the channel and staging buffer are usable. */
static bool s5l8740_dma_available(struct s5l8740_device *sdev)
{
	int ret;

	if (!lcd_dma)
		return false;
	if (READ_ONCE(sdev->dma_ready))
		return true;
	if (READ_ONCE(sdev->dma_failed))
		return false;

	mutex_lock(&sdev->dma_lock);
	if (!sdev->dma_ready && !sdev->dma_failed) {
		ret = s5l8740_dma_setup(sdev);
		if (ret) {
			/*
			 * dma-s5l8740-pl080 is a module and this driver is
			 * built in, so -EPROBE_DEFER simply means the console
			 * came up first. Stay on PIO and try again next frame.
			 */
			if (ret != -EPROBE_DEFER) {
				sdev->dma_failed = true;
				drm_warn(&sdev->dev,
					 "frame DMA unavailable (%d); using PIO\n",
					 ret);
			}
			sdev->dma = NULL;
		}
	}
	mutex_unlock(&sdev->dma_lock);

	return READ_ONCE(sdev->dma_ready);
}

static void s5l8740_dma_complete(void *arg)
{
	struct s5l8740_device *sdev = arg;

	complete(&sdev->dma_done);
}

/* Copy the frame into the staging buffer, honouring the source pitch. */
/* Copy a WIDTH-stride XRGB8888 frame into @dst; the pixel format is shared
 * by the PL080 push buffer and the compositor's layer buffers. */
static void s5l8740_stage_into(void *dst, const u32 *src,
			       unsigned int pitch_px, unsigned int w,
			       unsigned int h)
{
	unsigned int y;

	if (w > WIDTH)
		w = WIDTH;
	if (h > HEIGHT)
		h = HEIGHT;

	if (pitch_px == WIDTH && w == WIDTH) {
		memcpy(dst, src, (size_t)w * h * 4);
		return;
	}

	for (y = 0; y < h; y++)
		memcpy((u32 *)dst + (size_t)y * WIDTH,
		       src + (size_t)y * pitch_px, (size_t)w * 4);
}

static void s5l8740_stage_frame(struct s5l8740_device *sdev, const u32 *src,
				unsigned int pitch_px, unsigned int w,
				unsigned int h)
{
	s5l8740_stage_into(sdev->fbuf, src, pitch_px, w, h);
}

/*
 * Push the staged frame, the way sub_47B0 at 0x080047B0 does.
 *
 * Stock loops: clamp to 0x3FFC bytes, request a channel (0x080A4F94),
 * start (0x0800BAD8), wait 100 ms (0x080B65F4), release (0x080BCB24), and
 * go round again -- 26 single transfers for a full frame.
 *
 * This used to issue those 26 as one LLI chain, on the reasoning that the
 * bus traffic is identical and only the channel churn differs. The bus
 * traffic is not the point. sub_BAD8 passes a4 = 0 to sub_B424C, and that
 * argument is the linked-list flag:
 *
 *	if (a4) { ...build an LLI, *v23 = lli; }
 *	else      *v23 = 0;
 *
 * so every one of stock's 26 transfers runs with the channel's LLI
 * register at zero -- a single shot, armed by writing CONFIG last, and
 * fully torn down before the next. A chain is a different thing to ask a
 * peripheral for: the channel reloads CONTROL and CONFIG from memory at
 * each boundary, and on a request line that only ever produced a 16-byte
 * prefetch here, "the peripheral never asserted" and "the peripheral was
 * never asked the way stock asks" look identical from the residue.
 *
 * So this is stock's shape now: one segment at a time, its own descriptor,
 * its own completion, its own 100 ms budget, nothing chained.
 */
static void s5l8740_lcdif_arm_handoff(struct s5l8740_device *sdev);

/*
 * The frame kick, sub_A25C0(0) exactly: refuse if the gate is open or a
 * transfer is active, open, send the full-panel window, wait two
 * microseconds, close. Independent of lcd_frame_xfer, which governs the
 * push path; here the cycle IS the frame.
 */
static int s5l8740_comp_kick(struct s5l8740_device *sdev)
{
	void __iomem *b = sdev->lcdif;

	if (readl(b + S5L8740_LCD_XSTAT) & S5L8740_LCD_XSTAT_BUSY) {
		sdev->comp_skips++;
		return -EBUSY;
	}
	writel(1, b + S5L8740_LCD_XFER);
	if (sdev->dsi)
		s5l8740_dsi_window(sdev, 0, 0, WIDTH, HEIGHT);
	udelay(2);
	writel(0, b + S5L8740_LCD_XFER);
	sdev->comp_kicks++;
	return 0;
}

/*
 * sub_52BAC: load the five lookup tables. The index word selects the
 * table in bits 16:12 and the entry in bits 7:0; the second loop writes
 * the 0x11000 index and then the 0x13000 index before the one data word,
 * which is what the firmware does.
 */
static void s5l8740_comp_load_lut(void __iomem *c)
{
	u32 base = readl(c + S5L8740_COMP_LUT_IDX) & 0xfffe0f00;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(s5l8740_comp_lut_17); i++) {
		writel(base | i | 0x17000, c + S5L8740_COMP_LUT_IDX);
		writel(s5l8740_comp_lut_17[i], c + S5L8740_COMP_LUT_DATA);
	}
	for (i = 0; i < ARRAY_SIZE(s5l8740_comp_lut_11_13); i++) {
		writel(base | i | 0x11000, c + S5L8740_COMP_LUT_IDX);
		writel(base | i | 0x13000, c + S5L8740_COMP_LUT_IDX);
		writel(s5l8740_comp_lut_11_13[i], c + S5L8740_COMP_LUT_DATA);
	}
	for (i = 0; i < ARRAY_SIZE(s5l8740_comp_lut_14); i++) {
		writel(base | i | 0x14000, c + S5L8740_COMP_LUT_IDX);
		writel(s5l8740_comp_lut_14[i], c + S5L8740_COMP_LUT_DATA);
		writel(base | i | 0x15000, c + S5L8740_COMP_LUT_IDX);
		writel(s5l8740_comp_lut_15[i], c + S5L8740_COMP_LUT_DATA);
		writel(base | i | 0x16000, c + S5L8740_COMP_LUT_IDX);
		writel(s5l8740_comp_lut_16[i], c + S5L8740_COMP_LUT_DATA);
	}
}

/*
 * sub_B5F08 applied to the 11-entry table at 0x8775D0C: each source byte
 * b becomes 13 * b / 10, or 255 once 13 * b reaches 0xA00. These are the
 * results, and they are the words the capture holds at +0x1bb8..+0x1c38.
 */
static const u32 s5l8740_comp_cabc_tab[33] = {
	0x000a070f, 0x0f050a05, 0x13130e00, 0x0f00070f, 0x0f050a05, 0x13130f00,
	0x0f0f000f, 0x13070f07, 0x13130700, 0x0a0a0500, 0x0f050a05, 0x13130f00,
	0x05050505, 0x00030503, 0x13130f00, 0x0f0f0f0f, 0x13000f0d, 0x13130200,
	0x0f0f070f, 0x0f050005, 0x13130f00, 0x0f0f0f0f, 0x130d0f00, 0x13130200,
	0x03030303, 0x05030303, 0x000f0f00, 0x03030303, 0x03030303, 0x07000f00,
	0x13131313, 0x13131313, 0x13130000,
};

/*
 * sub_4BB118 -> sub_A49BC and the periodic task's first tick after it,
 * in the order disassembled at 0x4BB118 and 0xA49BC.
 *
 * Not implemented, and listed so nobody has to rediscover them:
 * sub_AEFB0(9116) writes 0x3DE00024 in the block the reference docs call
 * SWI, which this kernel does not map; sub_9BEA0(0) and sub_9BEA0(1),
 * called from sub_A4958 either side of the start bit, drive the backlight
 * device object at 0x8920BC8 and sub_BAF0 in that same SWI block.
 */
static void s5l8740_comp_cabc(struct s5l8740_device *sdev)
{
	void __iomem *c = sdev->comp;
	unsigned int i;

	s5l8740_comp_load_lut(c);				/* sub_52BAC */

	/* sub_AE9D8 -> sub_B5F78: from the table at 0x8775CB4. */
	writel(0x00b00080, c + S5L8740_COMP_CABC_88);
	writel(0x00d20070, c + S5L8740_COMP_CABC_88 + 0x4);
	writel(0x00a00100, c + S5L8740_COMP_CABC_88 + 0x8);
	writel(0x00a00100, c + S5L8740_COMP_CABC_88 + 0xc);
	writel(0x00c000c0, c + S5L8740_COMP_CABC_88 + 0x10);
	writel(0x00000000, c + S5L8740_COMP_CABC_88 + 0x14);
	/* -> sub_B5FF0: 16-bit results of the 0x8775CE0 table; high halves are hardware's. */
	writel(0x0bcc, c + S5L8740_COMP_CABC_A0);
	writel(0x0c26, c + S5L8740_COMP_CABC_A0 + 0x4);
	writel(0x08f3, c + S5L8740_COMP_CABC_A0 + 0x8);
	writel(0x08f3, c + S5L8740_COMP_CABC_A0 + 0xc);
	writel(0x19b4, c + S5L8740_COMP_CABC_A0 + 0x10);
	writel(0x08890000, c + S5L8740_COMP_CABC_A0 + 0x14);
	/* sub_AE804 -> sub_B5F08 x 11 */
	for (i = 0; i < 11; i++) {
		writel(s5l8740_comp_cabc_tab[3 * i],
		       c + S5L8740_COMP_CABC_B8 + 12 * i);
		writel(s5l8740_comp_cabc_tab[3 * i + 1],
		       c + S5L8740_COMP_CABC_B8 + 12 * i + 4);
		writel(s5l8740_comp_cabc_tab[3 * i + 2],
		       c + S5L8740_COMP_CABC_B8 + 12 * i + 8);
	}
	/* sub_AEAE0 */
	writel(readl(c + S5L8740_COMP_CABC_CTRL) | BIT(5),
	       c + S5L8740_COMP_CABC_CTRL);
	/* sub_AEFB0(9116): 0x3DE00024, not mapped -- see above. */
	/* sub_AEAF0 -> sub_B604C / sub_B6074 / sub_B60B0 */
	writel(0x07ff07ff, c + S5L8740_COMP_CABC_50);
	writel(0x07ff0000, c + S5L8740_COMP_CABC_54);
	writel(0, c + S5L8740_COMP_CABC_58);
	writel(0, c + S5L8740_COMP_CABC_5C);
	writel(0, c + S5L8740_COMP_CABC_60);
	/* sub_AEAC8 -> sub_B609C(0), sub_A49A8(0x80) */
	writel(readl(c + S5L8740_COMP_CABC_38) & ~BIT(8),
	       c + S5L8740_COMP_CABC_38);
	writel((readl(c + S5L8740_COMP_CABC_38) & ~0xffu) | 0x80,
	       c + S5L8740_COMP_CABC_38);
	/* sub_A4958(obj, 0); sub_9BEA0(0) not implemented */
	writel(readl(c + S5L8740_COMP_CABC_CTRL) & ~BIT(0),
	       c + S5L8740_COMP_CABC_CTRL);
	/* back in sub_4BB118: sub_4BB074(obj, 0) */
	writel(readl(c + S5L8740_COMP_CABC_CTRL) & ~BIT(1),
	       c + S5L8740_COMP_CABC_CTRL);
	/* sub_427788(0, 1); sub_427788(1, 1) */
	writel((readl(c + S5L8740_COMP_400) & ~BIT(2)) | BIT(2),
	       c + S5L8740_COMP_400);
	writel((readl(c + S5L8740_COMP_400) & ~BIT(1)) | BIT(1),
	       c + S5L8740_COMP_400);
	/* sub_A4958(obj, 1); sub_9BEA0(1) not implemented */
	writel(readl(c + S5L8740_COMP_CABC_CTRL) | BIT(0),
	       c + S5L8740_COMP_CABC_CTRL);

	/*
	 * sub_779D4's next tick: sub_847B8 and sub_847E8 push the window
	 * sub_AE9A0 / sub_A4970 stored, {w - 1, h - 1, 0, 0}, into +0x1b78/7c
	 * and +0x1b70/74.
	 */
	writel(0, c + S5L8740_COMP_CABC_78);
	writel((HEIGHT - 1) | ((WIDTH - 1) << 16), c + S5L8740_COMP_CABC_7C);
	writel(0, c + S5L8740_COMP_CABC_70);
	writel((HEIGHT - 1) | ((WIDTH - 1) << 16), c + S5L8740_COMP_CABC_74);
}

/*
 * The words +0x1b30..+0x1c38 held on this device before the CABC block was
 * ever programmed (devmem dump, 2026-09-07, before the bring-up fix).
 * Written back by the re-init knob so the block can be taken out of the
 * picture without a reboot. +0x1b34 is the engine's result and is skipped.
 */
static const u32 s5l8740_comp_cabc_reset_words[67] = {
	0x00000000, 0x000a0889, 0x0000008d, 0x0000000a, 0x00008000, 0x00000000,
	0x00000000, 0x00000000, 0x0fff0fff, 0x0fff0000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x01df010f,
	0x00000000, 0x01df010f, 0x00008000, 0x00008000, 0x01000100, 0x01000100,
	0x01000100, 0x01000100, 0x02000300, 0x00000000, 0x0da70f10, 0x0a6710cd,
	0x117908f3, 0x0da708f3, 0x19992222, 0x08890000, 0x00040404, 0x04040404,
	0x04040400, 0x04000404, 0x04040404, 0x04040400, 0x04040004, 0x04040404,
	0x04040400, 0x04040400, 0x04040404, 0x04040400, 0x04040404, 0x00040404,
	0x04040400, 0x04040404, 0x04000404, 0x04040400, 0x04040404, 0x04040004,
	0x04040400, 0x04040404, 0x04040400, 0x04040400, 0x04040404, 0x04040404,
	0x00040400, 0x04040404, 0x04040404, 0x04000400, 0x04040404, 0x04040404,
	0x04040000,
};

static void s5l8740_comp_cabc_reset(struct s5l8740_device *sdev)
{
	void __iomem *c = sdev->comp;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(s5l8740_comp_cabc_reset_words); i++) {
		if (S5L8740_COMP_CABC_CTRL + 4 * i == 0x1b34)
			continue;
		writel(s5l8740_comp_cabc_reset_words[i],
		       c + S5L8740_COMP_CABC_CTRL + 4 * i);
	}
	/* +0x400 bits 2 and 1, which sub_427788(0, 0) / (1, 0) clear. */
	writel(readl(c + S5L8740_COMP_400) & ~(BIT(2) | BIT(1)),
	       c + S5L8740_COMP_400);
}

/* Which parts of the bring-up a (re-)program runs. */
#define S5L8740_COMP_REINIT_LUT		BIT(0)	/* sub_52BAC */
#define S5L8740_COMP_REINIT_CABC	BIT(1)	/* sub_4BB118 */
#define S5L8740_COMP_REINIT_CABC_RESET	BIT(2)	/* this device's pre-CABC words */

/*
 * Bring the compositor to the state a drawing RetailOS holds it in, in
 * the order the disassembly at 0x45690 and 0x4CEE0 runs it. The decomp
 * of sub_4CEE0 drops the veneer arguments of sub_427788 and the trailing
 * sub_56E58 call; both are recovered by hand (2026-09-07) and noted where
 * they land. Layer 0 is programmed last, where stock's render task
 * (sub_825C4) first touches it, after the enable.
 */
static void s5l8740_comp_program(struct s5l8740_device *sdev, bool first,
				 unsigned int flags)
{
	void __iomem *c = sdev->comp;
	u32 before, v;

	before = readl(c + S5L8740_COMP_CTRL);

	/* sub_45690(obj, first ? 0 : 1) */
	writel((readl(c + S5L8740_COMP_L0_3C) & 0xfffff000) + 96,
	       c + S5L8740_COMP_L0_3C);
	writel((readl(c + S5L8740_COMP_L1_5C) & 0xfffff000) + 96,
	       c + S5L8740_COMP_L1_5C);
	/* sub_4D058(obj, 1): a helper in SRAM at 0x22000350. Not implemented. */

	/* sub_4CEE0 */
	writel(readl(c + S5L8740_COMP_1B10) & ~S5L8740_COMP_1B10_EN,
	       c + S5L8740_COMP_1B10);				/* sub_53E64(0) */
	writel(before & ~S5L8740_COMP_CTRL_EN, c + S5L8740_COMP_CTRL); /* sub_54158 */
	if (flags & S5L8740_COMP_REINIT_LUT)
		s5l8740_comp_load_lut(c);			/* sub_52BAC */
	writel((readl(c + S5L8740_COMP_400) & ~BIT(0)) | BIT(0),
	       c + S5L8740_COMP_400);				/* sub_427788(4, 1) */
	/*
	 * Vtable slot +64 (obj, 801) through the display object, which the
	 * decomp does not resolve; the captured +0x10 word is 801.
	 */
	writel(S5L8740_COMP_OUT10_STOCK, c + S5L8740_COMP_OUT10);
	/* sub_540BC(0, 0, 0x10101, 1) */
	v = readl(c + S5L8740_COMP_OUT300);
	v = (v & 0x5fffffff) | 0x80000000;
	v &= 0xefffffff;
	v = (v & 0xfcffffff) | 0x01000000;
	v = (v & 0xffcfffff) | 0x00100000;
	v = (v & 0xfffcffff) | 0x00010000;
	v = (v & ~1u) | 1;
	writel(v, c + S5L8740_COMP_OUT300);
	writel((8 << 16) | (8 << 8) | 8, c + S5L8740_COMP_OUT304); /* sub_549E4(8, 8, 8) */
	writel(readl(c + S5L8740_COMP_CTRL) | S5L8740_COMP_CTRL_BIT7,
	       c + S5L8740_COMP_CTRL);				/* sub_53AC8(1) */
	/*
	 * Vtable slot +28 (obj, n, 0) for n = 0, 1, 2: unresolved, see the
	 * display doc. The layer-enable word is written below.
	 */
	writel(readl(c + S5L8740_COMP_LAYERS) & ~BIT(1),
	       c + S5L8740_COMP_LAYERS);			/* sub_5413C(0) */
	writel(readl(c + S5L8740_COMP_LAYERS) & ~BIT(8),
	       c + S5L8740_COMP_LAYERS);			/* sub_52DF8(0) */
	writel(2, c + S5L8740_COMP_1B14);
	writel(0x10109, c + S5L8740_COMP_1B1C);
	writel(0x1011b, c + S5L8740_COMP_1B20);
	writel((HEIGHT - 1) | ((WIDTH - 1) << 16), c + S5L8740_COMP_1B24);
	writel(0, c + S5L8740_COMP_1B18);
	/* Vtable slot +56 (obj, 0): its return value feeds a buffer task only. */
	writel(S5L8740_COMP_CTRL8_STOCK, c + S5L8740_COMP_CTRL8);	/* sub_56E58 */

	/* sub_45690 again; +0x114/+0x118 only on the first bring-up (!a2) */
	if (first) {
		writel(((readl(c + S5L8740_COMP_114) & 0xf000f000) | 0x100000) + 16,
		       c + S5L8740_COMP_114);
		writel((readl(c + S5L8740_COMP_118) & ~BIT(2)) | 3,
		       c + S5L8740_COMP_118);
	}
	(void)readl(c + S5L8740_COMP_100);
	/* sub_4F6AC(0, 0, 0, w, h) -> sub_75D44: the panel window. */
	if (sdev->dsi)
		s5l8740_dsi_window(sdev, 0, 0, WIDTH, HEIGHT);
	writel(readl(c + S5L8740_COMP_CTRL) | S5L8740_COMP_CTRL_EN,
	       c + S5L8740_COMP_CTRL);				/* sub_4AA80 */
	s5l8740_lcdif_arm_handoff(sdev);	/* sub_4F680(1) -> sub_429B4(1) */
	writel(readl(c + S5L8740_COMP_1B10) | S5L8740_COMP_1B10_BIT,
	       c + S5L8740_COMP_1B10);

	/* Layer 0, as sub_825C4 leaves it. */
	writel(S5L8740_COMP_L0_CFG_STOCK, c + S5L8740_COMP_L0_CFG);
	writel(WIDTH, c + S5L8740_COMP_L0_W);
	writel(0, c + S5L8740_COMP_L0_POS);
	writel((WIDTH << 16) | HEIGHT, c + S5L8740_COMP_L0_SIZE);
	writel(0, c + S5L8740_COMP_L0_34);
	writel(0, c + S5L8740_COMP_L0_38);
	writel(lower_32_bits(sdev->cbuf_dma[0]) & 0x7fffffff,
	       c + S5L8740_COMP_L0_ADDR);
	writel(S5L8740_COMP_LAYERS_STOCK, c + S5L8740_COMP_LAYERS);
	writel(0x50, c + S5L8740_COMP_2E0);
	writel(0x50, c + S5L8740_COMP_2E4);

	if (flags & S5L8740_COMP_REINIT_CABC_RESET)
		s5l8740_comp_cabc_reset(sdev);
	if (flags & S5L8740_COMP_REINIT_CABC)
		s5l8740_comp_cabc(sdev);

	if (first)
		sdev->cbuf_back = 1;
	drm_info(&sdev->dev,
		 "compositor%s flags=%#x: CTRL %08x -> %08x LAYERS %08x OUT300 %08x WIN %08x +0x400 %08x L0 @%pad size %08x (stock 00000085/00000210/81110001/00ef01af/00000001)\n",
		 first ? "" : " re-init", flags,
		 before, readl(c + S5L8740_COMP_CTRL),
		 readl(c + S5L8740_COMP_LAYERS),
		 readl(c + S5L8740_COMP_OUT300),
		 readl(c + S5L8740_COMP_1B24),
		 readl(c + S5L8740_COMP_400),
		 &sdev->cbuf_dma[0],
		 readl(c + S5L8740_COMP_L0_SIZE));
}

static int s5l8740_comp_setup(struct s5l8740_device *sdev)
{
	struct device *dev = sdev->dev.dev;
	unsigned int i;

	if (!sdev->comp)
		return -ENODEV;

	sdev->cbuf_order = get_order(S5L8740_COMP_BUF_BYTES);
	for (i = 0; i < 2; i++) {
		sdev->cbuf[i] = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
							 sdev->cbuf_order);
		if (!sdev->cbuf[i])
			goto err;
		sdev->cbuf_dma[i] = dma_map_single(dev, sdev->cbuf[i],
						   S5L8740_COMP_BUF_BYTES,
						   DMA_TO_DEVICE);
		if (dma_mapping_error(dev, sdev->cbuf_dma[i])) {
			free_pages((unsigned long)sdev->cbuf[i], sdev->cbuf_order);
			sdev->cbuf[i] = NULL;
			goto err;
		}
	}

	s5l8740_comp_program(sdev, true, S5L8740_COMP_REINIT_LUT |
			     (lcd_cabc ? S5L8740_COMP_REINIT_CABC : 0));
	return 0;

err:
	for (i = 0; i < 2; i++) {
		if (!sdev->cbuf[i])
			continue;
		dma_unmap_single(dev, sdev->cbuf_dma[i],
				 S5L8740_COMP_BUF_BYTES, DMA_TO_DEVICE);
		free_pages((unsigned long)sdev->cbuf[i], sdev->cbuf_order);
		sdev->cbuf[i] = NULL;
	}
	return -ENOMEM;
}

static bool s5l8740_comp_available(struct s5l8740_device *sdev)
{
	int ret;

	if (!lcd_compositor || !sdev->comp)
		return false;
	if (READ_ONCE(sdev->comp_ready))
		return true;
	if (READ_ONCE(sdev->comp_failed))
		return false;

	ret = s5l8740_comp_setup(sdev);
	if (ret) {
		sdev->comp_failed = true;
		drm_warn(&sdev->dev, "compositor unavailable (%d); using the push path\n",
			 ret);
		return false;
	}
	sdev->comp_ready = true;
	return true;
}

/*
 * Wait, briefly, for the transfer the last kick started to finish.
 *
 * After a kick the compositor is still reading the front buffer for as
 * long as the LCDIF is pushing it to the panel -- +0x8c bit 1. Two things
 * must not happen while that is true: staging into that buffer, which
 * tears the frame in flight, and kicking again, which sub_A25C0 refuses.
 * The transfer takes a few milliseconds; the UI's own bursts arrive faster
 * than that, which is where the first boot's 51 refused kicks came from.
 * Process context, so this sleeps rather than spins.
 */
static bool s5l8740_comp_wait_idle(struct s5l8740_device *sdev)
{
	unsigned int waited_us = 0;

	while (readl(sdev->lcdif + S5L8740_LCD_XSTAT) & S5L8740_LCD_XSTAT_BUSY) {
		if (waited_us >= 20000)
			return false;
		usleep_range(200, 400);
		waited_us += 300;
		sdev->comp_waits++;
	}
	return true;
}

/*
 * A kick that was refused leaves +0x24 pointing at a frame nobody has
 * shown. Stock's caller simply tries again on its next tick; the DRM
 * damage path has no tick, so without this the last frame of a burst --
 * the one the user is looking for -- stays staged forever.
 */
static void s5l8740_comp_retry_fn(struct work_struct *work)
{
	struct s5l8740_device *sdev =
		container_of(to_delayed_work(work), struct s5l8740_device,
			     comp_retry);

	mutex_lock(&sdev->comp_lock);
	if (sdev->comp_pending) {
		if (!s5l8740_comp_kick(sdev)) {
			sdev->comp_pending = false;
			sdev->cbuf_back ^= 1;
		} else if (++sdev->comp_retries < 50) {
			schedule_delayed_work(&sdev->comp_retry,
					      usecs_to_jiffies(4000));
		} else {
			sdev->comp_pending = false;
			drm_warn(&sdev->dev,
				 "compositor: gate stayed busy through 50 retries; frame dropped\n");
		}
	}
	mutex_unlock(&sdev->comp_lock);
}

/*
 * One frame the way stock does it: let the previous transfer finish,
 * stage into the back buffer, point the layer at it, kick, swap.
 */
static int s5l8740_comp_frame(struct s5l8740_device *sdev, const u32 *src,
			      unsigned int pitch_px, unsigned int w,
			      unsigned int h)
{
	unsigned int i;
	int ret;

	mutex_lock(&sdev->comp_lock);
	cancel_delayed_work(&sdev->comp_retry);
	sdev->comp_pending = false;
	sdev->comp_retries = 0;

	s5l8740_comp_wait_idle(sdev);

	i = sdev->cbuf_back;
	s5l8740_stage_into(sdev->cbuf[i], src, pitch_px, w, h);
	dma_sync_single_for_device(sdev->dev.dev, sdev->cbuf_dma[i],
				   S5L8740_COMP_FRAME_BYTES, DMA_TO_DEVICE);
	writel(lower_32_bits(sdev->cbuf_dma[i]) & 0x7fffffff,
	       sdev->comp + S5L8740_COMP_L0_ADDR);
	ret = s5l8740_comp_kick(sdev);
	if (ret) {
		sdev->comp_pending = true;
		schedule_delayed_work(&sdev->comp_retry, usecs_to_jiffies(4000));
	} else {
		sdev->cbuf_back = i ^ 1;
	}
	mutex_unlock(&sdev->comp_lock);
	return ret;
}

static int s5l8740_dma_frame(struct s5l8740_device *sdev)
{
	unsigned int timeout = lcd_dma_timeout_ms ? lcd_dma_timeout_ms :
						    S5L8740_DMA_TIMEOUT_MS;
	unsigned int i;

	if (lcd_request_arm)
		s5l8740_lcdif_arm_handoff(sdev);

	dma_sync_single_for_device(sdev->dma_dev, sdev->fbuf_dma,
				   sdev->fbuf_size, DMA_TO_DEVICE);

	for (i = 0; i < sdev->nsegs; i++) {
		struct dma_async_tx_descriptor *tx;
		dma_cookie_t cookie;

		tx = dmaengine_prep_slave_sg(sdev->dma, &sdev->sgl[i], 1,
					     DMA_MEM_TO_DEV,
					     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!tx)
			return -EIO;

		reinit_completion(&sdev->dma_done);
		tx->callback = s5l8740_dma_complete;
		tx->callback_param = sdev;

		cookie = dmaengine_submit(tx);
		if (dma_submit_error(cookie))
			return -EIO;
		dma_async_issue_pending(sdev->dma);

		if (!wait_for_completion_timeout(&sdev->dma_done,
						 msecs_to_jiffies(timeout))) {
			struct dma_tx_state state = {};
			unsigned int len = sg_dma_len(&sdev->sgl[i]);

			/*
			 * Report the segment, not the frame. The residue of a
			 * single 0x3FFC transfer separates the two reasons it
			 * can fail, and they need different fixes:
			 *
			 *   residue == len  the channel never moved a byte,
			 *                   so the request never asserted; a
			 *                   longer timeout will not help.
			 *   residue <  len  it ran and did not finish in the
			 *                   budget; raise lcd_dma_timeout_ms.
			 */
			dmaengine_tx_status(sdev->dma, cookie, &state);
			dmaengine_terminate_sync(sdev->dma);
			drm_warn(&sdev->dev,
				 "frame DMA timeout after %ums on segment %u/%u: residue %u of %u bytes (%s)\n",
				 timeout, i + 1, sdev->nsegs, state.residue,
				 len,
				 state.residue >= len ? "never started" :
							"started, ran out of time");
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static void s5l8740_primary_plane_helper_atomic_update(struct drm_plane *plane,
						       struct drm_atomic_state *state)
{
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old_state = drm_atomic_get_old_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow = to_drm_shadow_plane_state(plane_state);
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_device *dev = plane->dev;
	struct s5l8740_device *sdev = s5l8740_device_of_dev(dev);
	unsigned int pitch_px;
	const u32 *src;
	int idx;

	if (!fb || drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE))
		return;

	/*
	 * Never push pixels at a stopped interface. Each write waits on the
	 * status register, so a full frame against a powered-down LCDIF would
	 * stall for the timeout on every one of them. The panel is repainted
	 * by n31_lcd_power() once it is running again.
	 */
	if (!READ_ONCE(sdev->powered))
		goto out_end_cpu_access;

	if (!drm_dev_enter(dev, &idx))
		goto out_end_cpu_access;

	src = shadow->data[0].vaddr;
	pitch_px = fb->pitches[0] / 4;
	if (!pitch_px)
		pitch_px = fb->width;

	/*
	 * Stock refreshes the panel exactly one way. From sub_A25C0:
	 *
	 *   if (MEMORY[0x3830008C] << 30)  return -1;   busy
	 *   MEMORY[0x38300080] = 1;                     open
	 *   sub_AEFAC(0, 0, w, h);                      window
	 *   do sub_345D68(); while (!v3);               settle
	 *   MEMORY[0x38300080] = 0;                     close
	 *
	 * and sub_AEFAC is one instruction, "b.w sub_75D44" -- a tail jump
	 * into the DCS window setter. So the window is (0, 0, w, h), the whole
	 * panel, on every refresh. The firmware has no partial-window path at
	 * all.
	 *
	 * That is why damage-clipped updates walked up and down the screen.
	 * The LCDIF is programmed once at init with the panel geometry,
	 * 0x38300074 = h | (w << 16), and never again; it frames what it sends
	 * by that register, so a narrower rectangle is still framed to the
	 * full panel width and each row lands further from where it belongs.
	 * The error grows with (panel width - rect width), which is why it
	 * looked as though it depended on what was being drawn. Full-screen
	 * repaints have no discrepancy and landed correctly -- exactly the
	 * split that showed on the glass, fresh screens right and menu deltas
	 * adrift.
	 *
	 * That init is not ours to change, so send full frames like stock.
	 *
	 * The damage rectangles are therefore coalesced to the full panel
	 * rather than clipped to. lcd_partial=Y still clips, and still only
	 * on the PIO path -- a partial rectangle cannot be expressed as a
	 * contiguous run into the pixel FIFO at all, because the LCDIF frames
	 * whatever it is given by 0x38300074.
	 */
	{
		struct drm_rect full = {
			.x1 = 0,
			.y1 = 0,
			.x2 = fb->width,
			.y2 = fb->height,
		};

		if (s5l8740_comp_available(sdev)) {
			s5l8740_comp_frame(sdev, src, pitch_px, fb->width,
					   fb->height);
		} else if (s5l8740_dma_available(sdev) && !sdev->dma_gave_up) {
			int ret;

			s5l8740_stage_frame(sdev, src, pitch_px, fb->width,
					    fb->height);
			if (s5l8740_window(sdev, &full)) {
				drm_warn_once(dev, "LCDIF busy; update skipped\n");
			} else {
				ret = s5l8740_dma_frame(sdev);
				if (ret) {
					/*
					 * Paint it the slow way rather than
					 * dropping the frame. Without this a
					 * DMA that fails leaves the panel
					 * showing whatever was there before,
					 * with nothing else attempting the
					 * update -- which looks like a frozen
					 * display rather than a DMA fault, and
					 * hides the very failure being
					 * diagnosed.
					 *
					 * Rate-limited rather than _once so a
					 * fault that recurs is visible as
					 * recurring.
					 */
					/*
					 * Latch off after three failures.
					 *
					 * The fallback costs a full
					 * lcd_dma_timeout_ms before one pixel
					 * is drawn, and it was being paid on
					 * every frame: measured 2026-09-07,
					 * every segment reported "residue
					 * 16380 of 16380 (never started)", so
					 * each frame stalled 100 ms and then
					 * blitted anyway. That is what made
					 * the panel stop refreshing and
					 * pinned a kworker at 81% -- not the
					 * DMA itself, but retrying it forever.
					 *
					 * A request line that has not
					 * asserted in three consecutive
					 * frames will not assert on the
					 * fourth. Stop asking and give PIO
					 * the whole frame budget; reload the
					 * module to try again.
					 */
					if (++sdev->dma_fails >= 3 &&
					    !sdev->dma_gave_up) {
						sdev->dma_gave_up = true;
						drm_warn(dev,
							 "frame DMA failed %u times; PIO for the rest of this boot\n",
							 sdev->dma_fails);
					}
					drm_warn(dev,
						 "frame DMA failed (%d); falling back to PIO for this frame\n",
						 ret);
					s5l8740_blit(sdev, src, pitch_px,
						     &full);
				}
			}
		} else if (lcd_partial) {
			struct drm_atomic_helper_damage_iter iter;
			struct drm_rect clip;

			drm_atomic_helper_damage_iter_init(&iter, old_state,
							   plane_state);
			drm_atomic_for_each_plane_damage(&iter, &clip)
				if (s5l8740_blit(sdev, src, pitch_px, &clip))
					break;
		} else {
			s5l8740_blit(sdev, src, pitch_px, &full);
		}
	}

	/*
	 * Only meaningful with lcd_xfer_hold=Y; otherwise s5l8740_window()
	 * already shut the gate before the pixels, as stock does.
	 */
	if (lcd_xfer_hold)
		s5l8740_xfer_close(sdev);

	drm_dev_exit(idx);
out_end_cpu_access:
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
}

static const struct drm_plane_helper_funcs s5l8740_primary_plane_helper_funcs = {
	.atomic_check = s5l8740_primary_plane_helper_atomic_check,
	.atomic_update = s5l8740_primary_plane_helper_atomic_update,
    DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
};

static const struct drm_plane_funcs s5l8740_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static int s5l8740_connector_helper_get_modes(struct drm_connector *connector)
{
	struct s5l8740_device *sdev = s5l8740_device_of_dev(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, &sdev->mode);
}

static const struct drm_connector_helper_funcs s5l8740_connector_helper_funcs = {
	.get_modes = s5l8740_connector_helper_get_modes,
};

static const struct drm_connector_funcs s5l8740_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};


/* ------------------------------------------------------------------ */
/* Display power sequence                                               */
/*                                                                      */
/* Reconstructed from the stock firmware, which does the whole thing in */
/* three steps and never sends the panel a command:                     */
/*                                                                      */
/*   on:   enable the display rail, reset the LCDIF, reprogram it, run  */
/*   off:  stop the LCDIF, drop the rail                                */
/*                                                                      */
/* The panel needs no init sequence of its own — the LCDIF is the whole */
/* story — which is what makes a real off/on cycle possible here.       */
/* ------------------------------------------------------------------ */

#define S5L8740_LCD_RESET	0x30	/* reset command / ack */
#define S5L8740_LCD_UNK2C	0x2c
#define S5L8740_LCD_UNK68	0x68
#define S5L8740_LCD_UNK70	0x70
#define S5L8740_LCD_SIZE	0x74	/* height | (width << 16) */
#define S5L8740_LCD_UNK78	0x78
#define S5L8740_LCD_UNK7C	0x7c
#define S5L8740_LCD_UNK84	0x84
#define S5L8740_LCD_UNKA4	0xa4

/*
 * BIT(10) is NOT a hold-in-reset bit. A live RetailOS with a working
 * panel reads CON = 0x00100EB0, i.e. CON_BASE | BIT(10), and sub_429B4
 * sets it together with +0x70 = 1 from the display power-on path. This
 * file used to clear it on the way in and call that housekeeping.
 * Keeping the old name as an alias so the existing power sequence still
 * reads sensibly, but the meaning is "request generation enabled".
 */
#define S5L8740_CON_REQ_EN	BIT(10)	/* 0x400: with +0x70=1, see sub_429B4 */
#define S5L8740_CON_HOLD	S5L8740_CON_REQ_EN
#define S5L8740_CON_RUN		BIT(11)	/* 0x800: interface running */
#define S5L8740_CON_BASE	0x00100ab0
#define S5L8740_CON_MODE_MASK	0xc0000000
#define S5L8740_CON_FMT_MASK	0x00000007
#define S5L8740_STATUS_RESETTING 0x1000

/* CLKCON gates that have to be dropped across an LCDIF reset. */
#define S5L8740_CLKCON_08	0x08
#define S5L8740_CLKCON_18	0x18
#define S5L8740_CLKCON_08_MASK	0x7fff7fff
#define S5L8740_CLKCON_18_MASK	0xffff3fff

#define S5L8740_LCD_RESET_TRIES	5

static unsigned int lcd_power_tries = S5L8740_LCD_RESET_TRIES;
module_param(lcd_power_tries, uint, 0644);
MODULE_PARM_DESC(lcd_power_tries, "Attempts to bring the display up (default 5)");

static unsigned int lcd_rail_settle_us = 3000;
module_param(lcd_rail_settle_us, uint, 0644);
MODULE_PARM_DESC(lcd_rail_settle_us,
		 "Settle time after enabling the display rail (us)");

static bool lcd_manage_rail = true;
module_param(lcd_manage_rail, bool, 0644);
MODULE_PARM_DESC(lcd_manage_rail,
		 "Hold the PMU display rail while the panel is on (default Y)");

static struct s5l8740_device *s5l8740_lcd_dev;

/*
 * The panel's own reset line, pad 15.
 *
 * sub_1C20 does not just take the display rail. It drives pad 15 high
 * first -- sub_2FE0(0) is sub_43D38C(15, 1, a1 ^ 1) -- then enables
 * sub_439B00(4, 1), which is register 16 bit 6, the rail this file
 * already holds. The teardown is the mirror: rail down, then sub_2FE0(1)
 * drives the pad low.
 *
 * Nothing in this kernel has ever driven it. It happens to read high on a
 * unit that has been up for a while, so the omission has been invisible,
 * but "the level we inherited is the level we want" is not a sequence and
 * does not survive a suspend or a cold boot that lands the other way.
 *
 * GPIOCMD rather than gpiod: the pad is not in any DT node, and this
 * matches how sub_43D38C latches it -- DIR set, then command 15 for high
 * or 14 for low.
 */
#define S5L8740_PANEL_RESET_PAD		15
#define S5L8740_GPIO_PHYS		0x3cf00000UL
#define S5L8740_GPIOCMD_OFF		0x1e0
#define S5L8740_GPIO_DIR_OFF		0x14

static void s5l8740_panel_reset_pad(struct s5l8740_device *sdev, bool high)
{
	unsigned int pad = S5L8740_PANEL_RESET_PAD;
	unsigned int bank = pad >> 3, pin = pad & 7;
	void __iomem *gpio;

	gpio = ioremap(S5L8740_GPIO_PHYS, 0x400);
	if (!gpio) {
		drm_warn(&sdev->dev, "panel reset pad: ioremap failed\n");
		return;
	}
	writel(readl(gpio + 32 * bank + S5L8740_GPIO_DIR_OFF) | BIT(pin),
	       gpio + 32 * bank + S5L8740_GPIO_DIR_OFF);
	writel((bank << 16) | (pin << 8) | (high ? 15 : 14),
	       gpio + S5L8740_GPIOCMD_OFF);
	iounmap(gpio);
}

static void s5l8740_lcd_rail(struct s5l8740_device *sdev, bool on)
{
	int (*get)(unsigned int);
	void (*put)(unsigned int);

	if (!lcd_manage_rail)
		return;
	if (on) {
		/* sub_1C20: pad 15 high, then the rail. */
		s5l8740_panel_reset_pad(sdev, true);
		get = (int (*)(unsigned int))__symbol_get("n31_pmu_rail_get");
		if (!get)
			return;
		if (get(N31_PMU_RAIL_DISPLAY))
			drm_warn(&sdev->dev, "display rail enable failed\n");
		__symbol_put("n31_pmu_rail_get");
	} else {
		put = (void (*)(unsigned int))__symbol_get("n31_pmu_rail_put");
		if (!put)
			return;
		put(N31_PMU_RAIL_DISPLAY);
		__symbol_put("n31_pmu_rail_put");
		/* sub_1D04's mirror: rail down, then pad 15 low. */
		s5l8740_panel_reset_pad(sdev, false);
	}
}

/*
 * LCDIF reset. `light` skips the parts that wait on hardware, which the
 * stock code uses when it only needs the clocks cycled.
 */
static int s5l8740_lcdif_reset(struct s5l8740_device *sdev, bool light)
{
	void __iomem *b = sdev->lcdif;
	u32 clk08 = 0, clk18 = 0, val;
	int ret = 0;

	if (sdev->clkcon) {
		clk08 = readl(sdev->clkcon + S5L8740_CLKCON_08);
		clk18 = readl(sdev->clkcon + S5L8740_CLKCON_18);
		writel(clk08 & S5L8740_CLKCON_08_MASK,
		       sdev->clkcon + S5L8740_CLKCON_08);
		writel(clk18 & S5L8740_CLKCON_18_MASK,
		       sdev->clkcon + S5L8740_CLKCON_18);
	}

	writel(readl(b + S5L8740_LCD_CON) & ~S5L8740_CON_HOLD,
	       b + S5L8740_LCD_CON);

	if (!light &&
	    readl_poll_timeout(b + S5L8740_LCD_STATUS, val,
			       !(val & S5L8740_STATUS_RESETTING), 100,
			       500 * USEC_PER_MSEC))
		drm_warn(&sdev->dev, "LCDIF busy before reset (status %08x)\n",
			 readl(b + S5L8740_LCD_STATUS));

	writel(1, b + S5L8740_LCD_RESET);
	if (readl_poll_timeout(b + S5L8740_LCD_RESET, val, !val, 100,
			       500 * USEC_PER_MSEC)) {
		drm_warn(&sdev->dev, "LCDIF reset did not ack\n");
		ret = -ETIMEDOUT;
	}

	if (!light) {
		/*
		 * Poke the hold bit until the interface admits it is held;
		 * the stock code loops on this for up to half a second.
		 */
		ktime_t end = ktime_add_ms(ktime_get(), 500);

		while (!(readl(b + S5L8740_LCD_CON) & S5L8740_CON_HOLD)) {
			writel(readl(b + S5L8740_LCD_CON) | S5L8740_CON_HOLD,
			       b + S5L8740_LCD_CON);
			if (ktime_after(ktime_get(), end)) {
				drm_warn(&sdev->dev, "LCDIF hold ack timeout\n");
				ret = -ETIMEDOUT;
				break;
			}
			usleep_range(100, 200);
		}
	}

	writel(readl(b + S5L8740_LCD_CON) & ~S5L8740_CON_HOLD,
	       b + S5L8740_LCD_CON);

	if (sdev->clkcon) {
		writel(clk08, sdev->clkcon + S5L8740_CLKCON_08);
		writel(clk18, sdev->clkcon + S5L8740_CLKCON_18);
	}
	return ret;
}

/* Reprogram the LCDIF for the current mode. Leaves it stopped. */
/*
 * The part of sub_2C64(0) that is not a register write.
 *
 * Its ten LCDIF writes are reproduced below and always have been. What was
 * never reproduced is the four calls it makes first, because the Hex-Rays
 * export renders three of them as bare no-argument stubs. Hand-disassembled
 * at 0x2CBE they are:
 *
 *	sub_345D28(8, 2, 11)	CLKCON+0x08: domain 8, source 2, divider 11
 *	sub_41CBD8(8, 1)	domain 8 enable
 *	sub_345D70(12, 0, 1)	clock gate id 12 -- CLKCON+0x48 bit 1,
 *				+0x6C bit 16, cleared to enable
 *	sub_439C4C(5)		power domain: table 0x08776BA0[5] = (2, 1),
 *				and sub_1234(2) if domain 2 is down
 *
 * Three of the four are already satisfied here and were checked rather than
 * assumed. CLKCON+0x08 is inherited from the bootloader as 0x2009200A, whose
 * low half is source 2 and divider 11 exactly; the id-12 gate bits both read
 * clear; and clk-s5l8702 models sub_41CBD8. The fourth had no owner at all,
 * so it is called here, at sub_2C64's position, ahead of the writes.
 *
 * Domain 2 is the display domain, which is not an assumption either: its
 * record at 0x0891D660 contains the LCDIF's own gate bits as a subset.
 */
static void s5l8740_lcdif_program(struct s5l8740_device *sdev)
{
	void __iomem *b = sdev->lcdif;
	u32 con = readl(b + S5L8740_LCD_CON);
	u32 keep = con & (S5L8740_CON_MODE_MASK | S5L8740_CON_FMT_MASK);
	int (*domain_up)(unsigned int);

	domain_up = (int (*)(unsigned int))__symbol_get("s5l8740_eic_domain_up");
	if (domain_up) {
		int dret = domain_up(2);

		if (dret)
			drm_warn(&sdev->dev,
				 "display power domain 2 did not come up: %d\n",
				 dret);
		__symbol_put("s5l8740_eic_domain_up");
	}

	writel(0x000a000a, b + S5L8740_LCD_UNK78);
	writel(keep | S5L8740_CON_BASE, b + S5L8740_LCD_CON);
	writel(1, b + S5L8740_LCD_UNK2C);
	writel(0, b + S5L8740_LCD_UNK68);
	writel(0, b + S5L8740_LCD_UNK70);
	writel(sdev->mode.vdisplay | (sdev->mode.hdisplay << 16),
	       b + S5L8740_LCD_SIZE);
	writel(0, b + S5L8740_LCD_PHTIME);
	writel(770, b + S5L8740_LCD_UNK7C);
	writel(100, b + S5L8740_LCD_UNK84);
	writel(1, b + S5L8740_LCD_UNKA4);

	/*
	 * sub_429B4(1): CON |= BIT(10) together with +0x70 = 1.
	 *
	 * sub_2C64 leaves +0x70 at 0, which is what the writes above
	 * reproduce -- but stock does not stop there. sub_429B4 is called from
	 * the display power-on path at 0x23FA and sets both, and a live
	 * RetailOS with a working panel reads back exactly that:
	 *
	 *     CON   0x00100EB0   = S5L8740_CON_BASE | BIT(10)
	 *     +0x70 0x00000001
	 *
	 * where we had CON 0x001006B0 and +0x70 = 0.
	 *
	 * So BIT(10) is not "interface held in reset" as this file called it,
	 * and clearing it was not harmless housekeeping. Whatever it gates, it
	 * is gated together with the +0x70 register, and the pairing is the
	 * best explanation available for the DMA symptom: the PL080 side is
	 * demonstrably fine -- the channel binds, arms, and prefetches its
	 * 16-byte FIFO -- and then the LCDIF never requests another byte,
	 * which is what an unarmed request generator looks like.
	 *
	 * Applied only on the DMA path for now. The PIO path works with these
	 * clear, because the CPU pushes rather than waiting to be asked, and
	 * there is no reason to disturb a working path to test a theory about
	 * a broken one. lcd_request_arm=N disables it outright.
	 */
	if (lcd_dma && lcd_request_arm) {
		writel(1, b + S5L8740_LCD_UNK70);
		writel(readl(b + S5L8740_LCD_CON) | S5L8740_CON_REQ_EN,
		       b + S5L8740_LCD_CON);
	}
}

/*
 * Put an inherited panel into the state a drawing RetailOS holds it in.
 *
 * probe() sets sdev->powered because the boot loader hands the panel over
 * already running, which makes s5l8740_lcd_power_on_locked() return at its
 * first line for the rest of the boot. s5l8740_lcdif_program() and
 * s5l8740_lcdif_run() are only reached from that function, so on a handoff
 * neither ever runs and the LCDIF keeps whatever the boot loader left.
 *
 * Measured on the live device against a working RetailOS:
 *
 *                 ours        RetailOS
 *      CON     0x001006B0    0x00100EB0
 *      +0x70   0x00000000    0x00000001
 *
 * 0x6B0 has bit 10 and not bit 11. Stock reaches 0xEB0 in two steps:
 * sub_2C64 writes the base 0x100AB0, which carries bit 11, and ORs 0x800
 * again on its enable path; sub_429B4(1) then sets bit 10 and only after
 * that writes +0x70 = 1, in that order.
 *
 * The CPU does not care -- it pushes pixels and never waits to be asked --
 * which is why PIO has always worked on top of the handoff state. A DMA
 * channel does care: with the request generator unarmed the LCDIF never
 * raises DMACBREQ, so the channel moved one burst of DBSize (4 transfers,
 * 16 bytes) and stopped. That is the "residue 414704 of 414720" this file
 * recorded and read as a FIFO prefetch; it was one burst, not a prefetch.
 *
 * Called from the DMA path rather than from probe. Arming at probe hung
 * the machine on 2026-09-07: CON was rewritten with no reset, no program
 * and no busy check while fbcon was pushing a frame, and the box never
 * reached userspace. s5l8740_lcdif_run() guards its own CON write with the
 * same poll used here, and the reason is this one.
 *
 * The panel is live, so the reset and the full re-program are deliberately
 * not run: this raises the inherited state to stock's, it does not restart
 * an interface with a picture on it.
 */
static void s5l8740_lcdif_arm_handoff(struct s5l8740_device *sdev)
{
	void __iomem *b = sdev->lcdif;
	u32 before, val;

	if (!b || sdev->handoff_armed)
		return;

	/*
	 * Gate on LCD_STATUS_BUSY, not STATUS_RESETTING.
	 *
	 * This device holds STATUS = 0x00001026 permanently -- bit 12, the
	 * bit named RESETTING here and polled by sub_2C64 -- while the panel
	 * draws correctly through the PIO path. Whatever bit 12 reports on
	 * this build, it is not "interface unusable", and waiting for it to
	 * clear meant this function timed out and armed nothing. Bit 4 is
	 * clear in that same reading and is the one this file already calls
	 * BUSY.
	 */
	if (readl_poll_timeout(b + S5L8740_LCD_STATUS, val,
			       !(val & S5L8740_LCD_STATUS_BUSY), 100,
			       100 * USEC_PER_MSEC)) {
		drm_warn(&sdev->dev,
			 "LCDIF busy (STATUS=%08x); handoff left unarmed\n",
			 readl(b + S5L8740_LCD_STATUS));
		return;
	}
	sdev->handoff_armed = true;

	before = readl(b + S5L8740_LCD_CON);

	/* sub_2C64(1): CON |= 0x800. */
	writel(before | S5L8740_CON_RUN, b + S5L8740_LCD_CON);

	/* sub_429B4(1): CON |= 0x400 first, then +0x70 = 1. */
	writel(readl(b + S5L8740_LCD_CON) | S5L8740_CON_REQ_EN,
	       b + S5L8740_LCD_CON);
	writel(1, b + S5L8740_LCD_UNK70);

	drm_info(&sdev->dev,
		 "LCDIF handoff armed: CON %08x -> %08x +0x70=%u (stock holds 00100eb0/1)\n",
		 before, readl(b + S5L8740_LCD_CON),
		 readl(b + S5L8740_LCD_UNK70));
}

/* Start the interface, matching the stock enable path. */
static int s5l8740_lcdif_run(struct s5l8740_device *sdev)
{
	void __iomem *b = sdev->lcdif;
	u32 con = readl(b + S5L8740_LCD_CON);
	u32 val;

	if ((con & (S5L8740_CON_MODE_MASK | S5L8740_CON_FMT_MASK)) !=
	    (S5L8740_CON_BASE & (S5L8740_CON_MODE_MASK | S5L8740_CON_FMT_MASK))) {
		if (readl_poll_timeout(b + S5L8740_LCD_STATUS, val,
				       !(val & S5L8740_STATUS_RESETTING), 100,
				       500 * USEC_PER_MSEC))
			drm_warn(&sdev->dev, "LCDIF busy before run\n");
		writel((con & 0x3ffffff8) | S5L8740_CON_BASE,
		       b + S5L8740_LCD_CON);
	}
	writel(readl(b + S5L8740_LCD_CON) | S5L8740_CON_RUN,
	       b + S5L8740_LCD_CON);
	return 0;
}

static int s5l8740_lcd_power_on_locked(struct s5l8740_device *sdev)
{
	unsigned int try;
	int ret = -ETIMEDOUT;

	if (sdev->powered)
		return 0;

	s5l8740_lcd_rail(sdev, true);
	if (lcd_manage_rail && lcd_rail_settle_us)
		usleep_range(lcd_rail_settle_us, lcd_rail_settle_us + 500);

	for (try = 0; try < (lcd_power_tries ? lcd_power_tries : 1); try++) {
		ret = s5l8740_lcdif_reset(sdev, try == 0);
		if (ret && try == 0)
			ret = s5l8740_lcdif_reset(sdev, false);

		s5l8740_lcdif_program(sdev);
		s5l8740_lcdif_run(sdev);

		if (readl(sdev->lcdif + S5L8740_LCD_CON) & S5L8740_CON_RUN) {
			ret = 0;
			break;
		}
		drm_warn(&sdev->dev, "display did not start (attempt %u)\n",
			 try + 1);
		ret = -EIO;
		msleep(2);
	}

	if (ret) {
		drm_err(&sdev->dev, "failed to turn on display after %u tries\n",
			lcd_power_tries);
		s5l8740_lcd_rail(sdev, false);
		return ret;
	}

	sdev->powered = true;
	drm_info(&sdev->dev, "display on (CON=%08x STATUS=%08x)\n",
		 readl(sdev->lcdif + S5L8740_LCD_CON),
		 readl(sdev->lcdif + S5L8740_LCD_STATUS));
	return 0;
}

static void s5l8740_lcd_power_off_locked(struct s5l8740_device *sdev)
{
	if (!sdev->powered)
		return;

	writel(readl(sdev->lcdif + S5L8740_LCD_CON) & ~S5L8740_CON_RUN,
	       sdev->lcdif + S5L8740_LCD_CON);
	s5l8740_lcd_rail(sdev, false);
	sdev->powered = false;
	drm_info(&sdev->dev, "display off (CON=%08x)\n",
		 readl(sdev->lcdif + S5L8740_LCD_CON));
}

/*
 * Entry points for the screen-sleep policy. Powering the panel down is the
 * point of the exercise: with only the backlight off the panel keeps drawing.
 */
int n31_lcd_power(bool on)
{
	struct s5l8740_device *sdev = s5l8740_lcd_dev;
	int ret;

	if (!sdev)
		return -ENODEV;

	/*
	 * Suspend the in-kernel clients before the panel goes down so fbcon
	 * stops drawing into a stopped interface; its damage would otherwise
	 * be dropped and the cursor would keep queueing work for nothing.
	 */
	if (!on)
		drm_client_dev_suspend(&sdev->dev, false);

	mutex_lock(&sdev->power_lock);
	if (on) {
		ret = s5l8740_lcd_power_on_locked(sdev);
	} else {
		s5l8740_lcd_power_off_locked(sdev);
		ret = 0;
	}
	mutex_unlock(&sdev->power_lock);

	if (on) {
		if (ret) {
			/* Left suspended on failure; nothing to draw to. */
			return ret;
		}
		/* Buffer survived, interface state did not: resume and repaint. */
		drm_client_dev_resume(&sdev->dev, false);
		drm_client_dev_restore(&sdev->dev);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(n31_lcd_power);

bool n31_lcd_is_on(void)
{
	return s5l8740_lcd_dev && s5l8740_lcd_dev->powered;
}
EXPORT_SYMBOL_GPL(n31_lcd_is_on);

static const struct drm_crtc_helper_funcs s5l8740_crtc_helper_funcs = {
	.atomic_check = drm_crtc_helper_atomic_check,
};

static const struct drm_crtc_funcs s5l8740_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs s5l8740_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_mode_config_funcs s5l8740_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_display_mode s5l8740_mode = {
	DRM_SIMPLE_MODE(WIDTH, HEIGHT, 30, 56),
};

/*
 * DRM driver
 */

DEFINE_DRM_GEM_FOPS(s5l8740_fops);

static struct drm_driver s5l8740_driver = {
    DRM_GEM_SHMEM_DRIVER_OPS,
    DRM_FBDEV_SHMEM_DRIVER_OPS,
    .name			= "s5l8740",
    .desc			= "s5l8740 lcdif",
    .major			= 0,
    .minor			= 1,
    .driver_features	= DRIVER_ATOMIC | DRIVER_GEM | DRIVER_MODESET,
    .fops			= &s5l8740_fops,
};

/*
 * Platform driver
 */

/*
 * Power control from userspace. The panel has no command interface, so
 * an off/on cycle here is the whole recovery path after anything glitches
 * the display rail: rail on, settle, LCDIF reset, reprogram, run, repaint.
 *
 *   cat lcd_power    1 while the interface is running
 *   echo 0 > ...     stop the interface and drop the rail
 *   echo 1 > ...     full bring-up
 */
static ssize_t lcd_power_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct s5l8740_device *sdev = s5l8740_lcd_dev;

	if (!sdev)
		return -ENODEV;
	return sysfs_emit(buf, "%d\n", sdev->powered ? 1 : 0);
}

static ssize_t lcd_power_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int on;
	int ret;

	if (kstrtouint(buf, 0, &on))
		return -EINVAL;
	ret = n31_lcd_power(on != 0);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(lcd_power);

static ssize_t lcd_state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct s5l8740_device *sdev = s5l8740_lcd_dev;

	if (!sdev || !sdev->lcdif)
		return -ENODEV;
	return sysfs_emit(buf,
			  "powered=%d rail_held=%d\n"
			  "CON=%08x STATUS=%08x SIZE=%08x\n"
			  "clkcon_window=%s\n"
			  "dma=%s fails=%u armed=%d\n"
			  "comp=%s kicks=%u skips=%u waits=%u retries=%u pending=%d ctrl=%08x layers=%08x\n"
			  "out300=%08x out304=%08x r400=%08x r404=%08x win=%08x cabc30=%08x cabc38=%08x\n",
			  sdev->powered, sdev->rail_held,
			  readl(sdev->lcdif + S5L8740_LCD_CON),
			  readl(sdev->lcdif + S5L8740_LCD_STATUS),
			  readl(sdev->lcdif + S5L8740_LCD_SIZE),
			  sdev->clkcon ? "mapped" : "absent",
			  sdev->dma_gave_up ? "latched-off" :
			  (sdev->dma ? "on" : "unavailable"),
			  sdev->dma_fails, sdev->handoff_armed,
			  !sdev->comp ? "unmapped" :
			  (sdev->comp_ready ? "on" :
			   (sdev->comp_failed ? "failed" : "idle")),
			  sdev->comp_kicks, sdev->comp_skips,
			  sdev->comp_waits, sdev->comp_retries, sdev->comp_pending,
			  sdev->comp ? readl(sdev->comp + S5L8740_COMP_CTRL) : 0,
			  sdev->comp ? readl(sdev->comp + S5L8740_COMP_LAYERS) : 0,
			  sdev->comp ? readl(sdev->comp + S5L8740_COMP_OUT300) : 0,
			  sdev->comp ? readl(sdev->comp + S5L8740_COMP_OUT304) : 0,
			  sdev->comp ? readl(sdev->comp + S5L8740_COMP_400) : 0,
			  sdev->comp ? readl(sdev->comp + S5L8740_COMP_400 + 4) : 0,
			  sdev->comp ? readl(sdev->comp + S5L8740_COMP_1B24) : 0,
			  sdev->comp ? readl(sdev->comp + S5L8740_COMP_CABC_CTRL) : 0,
			  sdev->comp ? readl(sdev->comp + S5L8740_COMP_CABC_38) : 0);
}
static DEVICE_ATTR_RO(lcd_state);

/*
 * Write 1 to try DMA again after the three-failure latch.
 *
 * The latch exists so a request line that never asserts does not cost
 * lcd_dma_timeout_ms on every frame. It also meant that once it fired,
 * nothing short of a reboot could test a fix -- on 2026-09-07 the actual
 * cause (DMACConfiguration read 0) was found and corrected live, and could
 * not be exercised because the display had already given up for the boot.
 */
static ssize_t lcd_dma_retry_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct s5l8740_device *sdev = s5l8740_lcd_dev;
	unsigned int v;

	if (!sdev)
		return -ENODEV;
	if (kstrtouint(buf, 0, &v) || v != 1)
		return -EINVAL;
	sdev->dma_gave_up = false;
	sdev->dma_fails = 0;
	drm_info(&sdev->dev, "frame DMA re-armed from sysfs\n");
	return count;
}
static DEVICE_ATTR_WO(lcd_dma_retry);

/*
 * Re-run the compositor bring-up as stock's wake path does
 * (sub_2549DC -> sub_254FD4 -> sub_45690(obj, 1)), with the parts that
 * are lookups selectable, so a colour fault can be bisected without a
 * reboot. Write a mask: 1 = reload the LUT tables, 2 = program the CABC
 * block, 4 = first restore the CABC block to this device's pre-CABC words.
 * 0 re-runs the plain register sequence alone.
 */
static ssize_t lcd_comp_reinit_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct s5l8740_device *sdev = s5l8740_lcd_dev;
	unsigned int flags;

	if (!sdev || !sdev->comp || !sdev->comp_ready)
		return -ENODEV;
	if (kstrtouint(buf, 0, &flags) || flags > 7)
		return -EINVAL;
	mutex_lock(&sdev->comp_lock);
	s5l8740_comp_wait_idle(sdev);
	s5l8740_comp_program(sdev, false, flags);
	mutex_unlock(&sdev->comp_lock);
	return count;
}
static DEVICE_ATTR_WO(lcd_comp_reinit);

static int s5l8740_probe(struct platform_device *pdev)
{
    struct s5l8740_device *sdev;
    struct drm_device *dev;
    struct resource *res;
    const struct drm_format_info *format;
	struct drm_plane *primary_plane;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	size_t nformats;
    int ret;

	sdev = devm_drm_dev_alloc(&pdev->dev, &s5l8740_driver, struct s5l8740_device, dev);
    if (IS_ERR(sdev))
        return PTR_ERR(sdev);

    sdev->mode = s5l8740_mode;
    format = drm_format_info(DRM_FORMAT_XRGB8888);
    sdev->format = format;

    dev = &sdev->dev;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
        return -EINVAL;

    drm_dbg(dev, "using I/O memory framebuffer at %pr\n", res);

    sdev->lcdif = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(sdev->lcdif))
        return PTR_ERR(sdev->lcdif);

    mutex_init(&sdev->power_lock);
    mutex_init(&sdev->dma_lock);
    init_completion(&sdev->dma_done);

    /*
     * Third window is the MIPI DSI host. Optional: without it the panel
     * window cannot be set and every update repaints the whole frame.
     */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
    if (res) {
        sdev->dsi = devm_ioremap_resource(&pdev->dev, res);
        if (IS_ERR(sdev->dsi))
            sdev->dsi = NULL;
    }

    /*
     * Fourth window is the layer compositor at 0x38900000, the engine
     * stock draws with at runtime. Optional: without it every update goes
     * through the PL080 pixel push, which is stock's boot-logo path.
     */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
    if (res) {
        sdev->comp = devm_ioremap_resource(&pdev->dev, res);
        if (IS_ERR(sdev->comp))
            sdev->comp = NULL;
    }
    if (!sdev->comp)
        drm_info(dev, "no compositor window; frames go through the pixel push\n");

    /*
     * Second window is the clock controller. Only two gates are touched,
     * and only to cycle them across an LCDIF reset. Optional: without it
     * the reset still works, it just does not gate the clocks first.
     */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if (res) {
    	/*
    	 * Map without claiming. This window is the clock controller, which
    	 * the clock-controller node already owns and the IIS driver also
    	 * maps, so an exclusive devm_ioremap_resource() always lost the
    	 * race and returned -EBUSY. The driver then carried on with
    	 * clkcon = NULL and quietly stopped gating the clocks across an
    	 * LCDIF reset -- a real behaviour change reported only as an error
    	 * line nobody acted on. CLKCON is a shared block; sharing it is
    	 * correct, claiming it is not.
    	 */
    	sdev->clkcon = devm_ioremap(&pdev->dev, res->start,
    				    resource_size(res));
    }
    if (!sdev->clkcon)
    	drm_info(dev,
    		 "no clkcon window; LCDIF reset will not gate clocks\n");

    /* The panel is already running from the boot loader handoff. */
    sdev->powered = true;
    /* Compositor retry state; the block itself is programmed on first use. */
    mutex_init(&sdev->comp_lock);
    INIT_DELAYED_WORK(&sdev->comp_retry, s5l8740_comp_retry_fn);
    s5l8740_lcd_dev = sdev;
    /*
     * remove() and shutdown() both dereference this; without it the
     * former was a NULL dereference on unbind.
     */
    platform_set_drvdata(pdev, sdev);

    /*
     * Take the display rail for the panel we inherited. Without this
     * nobody holds LDO_4, so the PMU's global rail repair -- which
     * clears bits 6 and 7 by design -- has nothing to preserve and
     * switches the panel off underneath us. That is the white screen:
     * the first audio bring-up after boot replays that sequence.
     */
    if (lcd_manage_rail) {
	int (*get)(unsigned int) =
		(int (*)(unsigned int))__symbol_get("n31_pmu_rail_get");

	if (get) {
		if (get(N31_PMU_RAIL_DISPLAY))
			drm_warn(dev, "display rail claim failed\n");
		else
			sdev->rail_held = true;
		__symbol_put("n31_pmu_rail_get");
	} else {
		drm_info(dev,
			 "PMIC absent; display rail unprotected\n");
	}
    }

    if (device_create_file(&pdev->dev, &dev_attr_lcd_power))
	drm_warn(dev, "lcd_power sysfs\n");
    if (device_create_file(&pdev->dev, &dev_attr_lcd_dma_retry))
        dev_warn(&pdev->dev, "lcd_dma_retry attribute not created\n");
    if (device_create_file(&pdev->dev, &dev_attr_lcd_comp_reinit))
        dev_warn(&pdev->dev, "lcd_comp_reinit attribute not created\n");
    if (device_create_file(&pdev->dev, &dev_attr_lcd_state))
	drm_warn(dev, "lcd_state sysfs\n");

    /* GATE0: log WTF handoff, never rewrite CON/PHTIME */
    drm_info(dev, "LCDIF handoff CON=%08x PHTIME=%08x (untouched)\n",
	     readl(sdev->lcdif + S5L8740_LCD_CON),
	     readl(sdev->lcdif + S5L8740_LCD_PHTIME));

    /* CON first (stage0). Print so glass shows whether WDT is still live. */
    {
	void __iomem *wdt = ioremap(0x3c800000, 8);

	if (wdt) {
		writel(0, wdt);
		writel(0, wdt + 4);
		writel(0, wdt);
		writel(0, wdt + 4);
		drm_info(dev, "WDT CON=%08x CNT=%08x (disarmed)\n",
			 readl(wdt), readl(wdt + 4));
		iounmap(wdt);
	}
    }

    /*
	 * Modesetting
	 */

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ret;

	dev->mode_config.min_width = WIDTH;
	dev->mode_config.max_width = WIDTH;
	dev->mode_config.min_height = HEIGHT;
	dev->mode_config.max_height = HEIGHT;
	dev->mode_config.preferred_depth = 32;
	dev->mode_config.funcs = &s5l8740_mode_config_funcs;

    /* Primary plane */

	nformats = drm_fb_build_fourcc_list(dev, &format->format, 1,
        sdev->formats, ARRAY_SIZE(sdev->formats));

    primary_plane = &sdev->primary_plane;
    ret = drm_universal_plane_init(dev, primary_plane, 0, &s5l8740_primary_plane_funcs,
           sdev->formats, nformats,
           NULL,
           DRM_PLANE_TYPE_PRIMARY, NULL);
    if (ret)
        return ret;
    drm_plane_helper_add(primary_plane, &s5l8740_primary_plane_helper_funcs);
    drm_plane_enable_fb_damage_clips(primary_plane);

    /* CRTC */

    crtc = &sdev->crtc;
    ret = drm_crtc_init_with_planes(dev, crtc, primary_plane, NULL,
        &s5l8740_crtc_funcs, NULL);
    if (ret)
        return ret;
    drm_crtc_helper_add(crtc, &s5l8740_crtc_helper_funcs);

    /* Encoder */

    encoder = &sdev->encoder;
    ret = drm_encoder_init(dev, encoder, &s5l8740_encoder_funcs,
       DRM_MODE_ENCODER_NONE, NULL);
    if (ret)
        return ret;
    encoder->possible_crtcs = drm_crtc_mask(crtc);

    /* Connector */

    connector = &sdev->connector;
    ret = drm_connector_init(dev, connector, &s5l8740_connector_funcs,
     DRM_MODE_CONNECTOR_Unknown);
    if (ret)
        return ret;
    drm_connector_helper_add(connector, &s5l8740_connector_helper_funcs);

    ret = drm_connector_attach_encoder(connector, encoder);
    if (ret)
        return ret;

    drm_mode_config_reset(dev);
 
    ret = drm_dev_register(dev, 0);
    if (ret)
         return ret;

    drm_client_setup(dev, sdev->format);

    return 0;
 }
 
/*
 * The LCDIF has no framebuffer-base or stride register -- it is a
 * FIFO-fed command-mode interface, and nothing in the stock image ever
 * hands it an address. What does read memory is the PL080 channel this
 * driver pushes frames through, and across a kexec the staging buffer
 * belongs to the next kernel. So stop the channel, then power the panel
 * down; n31_lcd_power suspends the DRM clients on the way, so nothing is
 * left drawing into a stopped interface.
 */
static void s5l8740_shutdown(struct platform_device *pdev)
{
	struct s5l8740_device *sdev = platform_get_drvdata(pdev);

	if (sdev && READ_ONCE(sdev->dma_ready))
		dmaengine_terminate_sync(sdev->dma);
	if (sdev)
		cancel_delayed_work_sync(&sdev->comp_retry);
	n31_lcd_power(false);
}

static void s5l8740_remove(struct platform_device *pdev)
{
    struct s5l8740_device *sdev = platform_get_drvdata(pdev);
    struct drm_device *dev;

    if (!sdev)
	return;
    dev = &sdev->dev;

    cancel_delayed_work_sync(&sdev->comp_retry);
    drm_dev_unplug(dev);
}
 
static const struct of_device_id s5l8740_of_match_table[] = {
    { .compatible = "samsung,s5l8740-lcdif", },
    { },
};
MODULE_DEVICE_TABLE(of, s5l8740_of_match_table);
 
static struct platform_driver s5l8740_platform_driver = {
    .driver = {
        .name = "s5l8740-lcdif",
        .of_match_table = s5l8740_of_match_table,
    },
    .probe = s5l8740_probe,
    .remove = s5l8740_remove,
    .shutdown = s5l8740_shutdown,
};
 
module_platform_driver(s5l8740_platform_driver);
 
MODULE_DESCRIPTION("s5l8740 tiny drm");
MODULE_LICENSE("GPL v2");
