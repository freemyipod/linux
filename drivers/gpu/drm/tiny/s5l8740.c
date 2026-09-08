// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5L8740 display controller (Apple iPod nano 7th generation).
 *
 * Four register windows make up the display path: the LCDIF that carries
 * pixels to a DCS command-mode panel, the MIPI DSI host the panel window
 * is set through, the layer compositor that draws each frame from DRAM,
 * and the clock controller in which an LCDIF reset cycles two gates. The
 * frame path is the one the stock firmware uses at runtime: one XRGB8888
 * layer pointed at a double-buffered frame, kicked by cycling the LCDIF
 * transfer gate. The DSI host is exposed as a MIPI DSI host and the panel
 * is a drm_panel bound to its child node; power on and off run the
 * firmware's own order through the panel's prepare, enable, disable and
 * unprepare around the host and the LCDIF.
 *
 * Binding: Documentation/devicetree/bindings/display/samsung/samsung,s5l8740-lcdif.yaml
 */

#include <linux/apple-n31.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
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
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <video/mipi_display.h>
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
 * Simple Framebuffer device
 */

 struct s5l8740_device {
	struct drm_device dev;

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
	/*
	 * The MIPI DSI host over the +0x3D800000 window, and the panel
	 * bound to its child node. The DRM device is built once the panel
	 * has attached; until then there is nothing to give a connector.
	 */
	struct mipi_dsi_host dsi_host;
	struct drm_panel *panel;
	struct mutex dsi_lock;		/* one packet at a time */
	bool dsi_ready;			/* host initialised (bootloader or us) */
	bool dsi_words_valid;
	u32 dsi_words[7];		/* PLL CTL RES T28 BAND T54 T58, as found */
	bool drm_ready;

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

/*
 * The rest of the host, from its bring-up sub_2AFC(0) with sub_4FC8 and
 * sub_3ED8, its shutdown sub_4640, and the read path sub_4870. The words
 * the bring-up writes to PLL, CTL, RES, +0x28, BAND, +0x54 and +0x58 come
 * from a panel description the firmware fills at runtime; this driver
 * captures them from the bootloader-initialised host at probe and writes
 * them back on a re-initialisation.
 */
#define S5L8740_DSI_STAT0		0x00	/* lane ready bits, bit 8, bit 20 */
#define S5L8740_DSI_STAT0_UP		BIT(20)	/* set once out of reset */
#define S5L8740_DSI_STAT0_READY		BIT(8)	/* with one bit per lane below */
#define S5L8740_DSI_RESET		0x04
#define S5L8740_DSI_PLL			0x08
#define S5L8740_DSI_PLL_HS		BIT(31)	/* sub_3ED8: 1 = high speed */
#define S5L8740_DSI_CTL2		0x14
#define S5L8740_DSI_CTL2_LP		0xc0	/* sub_3ED8: set = low power */
#define S5L8740_DSI_CTL2_PULSE		BIT(20)
#define S5L8740_DSI_RES			0x18	/* hres | vres << 16 */
#define S5L8740_DSI_RES_EN		BIT(31)
#define S5L8740_DSI_T28			0x28
#define S5L8740_DSI_RXSTAT		0x2c
#define S5L8740_DSI_RXSTAT_ANY		0x00250003u
#define S5L8740_DSI_RXSTAT_ERR		0x00000003u
#define S5L8740_DSI_RXSTAT_TIMEOUT	0x00210000u
#define S5L8740_DSI_INTCLR		0x30
#define S5L8740_DSI_RXDATA		0x3c
#define S5L8740_DSI_T40			0x40
#define S5L8740_DSI_T40_VAL		511
#define S5L8740_DSI_T44_VAL		29
#define S5L8740_DSI_ST_RXEMPTY		BIT(24)	/* receive FIFO empty */
#define S5L8740_DSI_BAND		0x4c	/* PLL band index << 24 */
#define S5L8740_DSI_T54			0x54
#define S5L8740_DSI_T58			0x58
#define S5L8740_DSI_INIT_TIMEOUT_US	20000

enum { DSI_W_PLL, DSI_W_CTL, DSI_W_RES, DSI_W_T28, DSI_W_BAND, DSI_W_T54,
       DSI_W_T58 };

/* Clock gates the display path opens and closes, one bit each (sub_41CBD8). */
#define S5L8740_CLKCON_08_LCDIF		BIT(15)	/* id 8 */
#define S5L8740_CLKCON_14		0x14
#define S5L8740_CLKCON_14_DSI		BIT(15)	/* id 13 */

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
 * How long fbdev deferred I/O waits after an mmap write before it hands
 * the dirty pages to DRM as damage. drm_fbdev_shmem sets HZ/20 = 50 ms,
 * which caps an mmap client (LVGL) at 20 frames a second regardless of
 * how fast the compositor is: measured 2026-09-07, 150 repaints at 30 ms
 * spacing became 75 kicks, with or without FBIOPUT_VSCREENINFO. write()
 * and ioctl clients are not affected; they damage directly.
 */
static unsigned int lcd_fbdefio_ms = 15;
module_param(lcd_fbdefio_ms, uint, 0644);
MODULE_PARM_DESC(lcd_fbdefio_ms,
		 "fbdev deferred-io coalescing delay for mmap writers, ms (default 15; DRM's own is 50)");

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
 * Restoring the block to its pre-programming words gave correct bars at
 * once.
 */
static bool lcd_cabc;
module_param(lcd_cabc, bool, 0644);
MODULE_PARM_DESC(lcd_cabc,
		 "Program the compositor's +0x1b30 content-adaptive block as stock does (default N: without the PMIC/single-wire consumer it transforms the output)");

/* Short packet: the two parameter bytes ride in the header (sub_6D80). */
static int s5l8740_dsi_short(struct s5l8740_device *sdev, u8 dt, u8 p0, u8 p1)
{
	u32 st;

	if (!sdev->dsi)
		return -ENODEV;
	writel((dt & 0x3f) | (p0 << 8) | (p1 << 16), sdev->dsi + S5L8740_DSI_HDR);
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
				 S5L8740_DCS_WRITE_START, 0);
}

/* Single-bit gate RMW on the shared clock controller; nothing else moves. */
static void s5l8740_clkcon_gate(struct s5l8740_device *sdev, unsigned int reg,
				u32 bit, bool open)
{
	u32 v;

	if (!sdev->clkcon)
		return;
	v = readl(sdev->clkcon + reg);
	v = open ? (v & ~bit) : (v | bit);
	writel(v, sdev->clkcon + reg);
}

/* sub_439C4C(id): power domain 2 carries the whole display path. */
static void s5l8740_domain_up(struct s5l8740_device *sdev)
{
	int (*domain_up)(unsigned int);

	domain_up = (int (*)(unsigned int))__symbol_get("s5l8740_eic_domain_up");
	if (!domain_up)
		return;
	if (domain_up(2))
		drm_warn(&sdev->dev, "display power domain 2 did not come up\n");
	__symbol_put("s5l8740_eic_domain_up");
}

/*
 * Record the host's words as the bootloader left them. A host that is not
 * showing the panel's own resolution was not initialised for it, and its
 * words are not written back.
 */
static void s5l8740_dsi_capture(struct s5l8740_device *sdev)
{
	void __iomem *d = sdev->dsi;
	u32 res;

	sdev->dsi_words[DSI_W_PLL] = readl(d + S5L8740_DSI_PLL);
	sdev->dsi_words[DSI_W_CTL] = readl(d + S5L8740_DSI_CTL);
	sdev->dsi_words[DSI_W_RES] = readl(d + S5L8740_DSI_RES);
	sdev->dsi_words[DSI_W_T28] = readl(d + S5L8740_DSI_T28);
	sdev->dsi_words[DSI_W_BAND] = readl(d + S5L8740_DSI_BAND);
	sdev->dsi_words[DSI_W_T54] = readl(d + S5L8740_DSI_T54);
	sdev->dsi_words[DSI_W_T58] = readl(d + S5L8740_DSI_T58);
	res = sdev->dsi_words[DSI_W_RES] & ~S5L8740_DSI_RES_EN;
	sdev->dsi_words_valid = res == (WIDTH | (HEIGHT << 16));
	sdev->dsi_ready = sdev->dsi_words_valid;
	drm_info(&sdev->dev,
		 "DSI host as found: PLL %08x CTL %08x RES %08x +28 %08x BAND %08x +54 %08x +58 %08x STAT0 %08x%s\n",
		 sdev->dsi_words[DSI_W_PLL], sdev->dsi_words[DSI_W_CTL],
		 sdev->dsi_words[DSI_W_RES], sdev->dsi_words[DSI_W_T28],
		 sdev->dsi_words[DSI_W_BAND], sdev->dsi_words[DSI_W_T54],
		 sdev->dsi_words[DSI_W_T58], readl(d + S5L8740_DSI_STAT0),
		 sdev->dsi_words_valid ? "" : " (not this panel's; no re-init possible)");
}

/*
 * sub_2AFC(0) followed by sub_4235A(3), in order. The two delays are
 * sub_345D48 thunks whose argument the export drops; 1 ms stands in.
 */
static int s5l8740_dsi_host_init(struct s5l8740_device *sdev)
{
	void __iomem *d = sdev->dsi;
	u32 lanes, v;
	int ret;

	if (!sdev->dsi_words_valid)
		return -ENODEV;
	lanes = ((sdev->dsi_words[DSI_W_CTL] >> 1) & 0x1f) | S5L8740_DSI_STAT0_READY;

	s5l8740_domain_up(sdev);				/* sub_439C4C(7) */
	writel(~0u, d + S5L8740_DSI_INTCLR);
	/* sub_4FC8 */
	writel(sdev->dsi_words[DSI_W_BAND], d + S5L8740_DSI_BAND);
	s5l8740_clkcon_gate(sdev, S5L8740_CLKCON_14, S5L8740_CLKCON_14_DSI, true);
	writel(sdev->dsi_words[DSI_W_PLL] & ~S5L8740_DSI_PLL_HS, d + S5L8740_DSI_PLL);

	writel(1, d + S5L8740_DSI_RESET);
	usleep_range(1000, 1500);
	writel(0, d + S5L8740_DSI_RESET);
	/*
	 * sub_3D11B0(+0x00, bit 20, 0): loop while the bit is clear, so this
	 * waits for it to SET. Waiting for it to clear instead timed out on
	 * every re-initialisation 2026-09-08 (STAT0 001001c3, bit 20 set at
	 * rest), which left the panel white after its first power-off.
	 */
	ret = readl_poll_timeout(d + S5L8740_DSI_STAT0, v,
				 v & S5L8740_DSI_STAT0_UP, 10,
				 S5L8740_DSI_INIT_TIMEOUT_US);
	if (ret)
		goto fail;
	writel(~0u, d + S5L8740_DSI_INTCLR);
	writel(sdev->dsi_words[DSI_W_RES] & ~S5L8740_DSI_RES_EN, d + S5L8740_DSI_RES);
	writel(sdev->dsi_words[DSI_W_T54], d + S5L8740_DSI_T54);
	writel(sdev->dsi_words[DSI_W_T58], d + S5L8740_DSI_T58);
	writel(sdev->dsi_words[DSI_W_T28], d + S5L8740_DSI_T28);
	writel(S5L8740_DSI_T40_VAL, d + S5L8740_DSI_T40);
	writel(S5L8740_DSI_T44_VAL, d + S5L8740_DSI_STATUS);
	writel(sdev->dsi_words[DSI_W_CTL], d + S5L8740_DSI_CTL);
	writel(S5L8740_DSI_CTL2_PULSE, d + S5L8740_DSI_CTL2);
	usleep_range(1000, 1500);
	writel(readl(d + S5L8740_DSI_CTL2) & ~S5L8740_DSI_CTL2_PULSE, d + S5L8740_DSI_CTL2);
	ret = readl_poll_timeout(d + S5L8740_DSI_STAT0, v, (v & lanes) == lanes,
				 10, S5L8740_DSI_INIT_TIMEOUT_US);
	if (ret)
		goto fail;
	writel(readl(d + S5L8740_DSI_CTL2) | S5L8740_DSI_CTL2_LP, d + S5L8740_DSI_CTL2);
	writel(readl(d + S5L8740_DSI_RES) | S5L8740_DSI_RES_EN, d + S5L8740_DSI_RES);

	/* sub_4235A(3) -> sub_3ED8(1): out of low power, high speed on. */
	writel(readl(d + S5L8740_DSI_CTL2) & ~S5L8740_DSI_CTL2_LP, d + S5L8740_DSI_CTL2);
	ret = readl_poll_timeout(d + S5L8740_DSI_STAT0, v, (v & lanes) == lanes,
				 10, S5L8740_DSI_INIT_TIMEOUT_US);
	if (ret)
		goto fail;
	writel(readl(d + S5L8740_DSI_PLL) | S5L8740_DSI_PLL_HS, d + S5L8740_DSI_PLL);
	sdev->dsi_ready = true;
	drm_info(&sdev->dev, "DSI host up (STAT0 %08x)\n", readl(d + S5L8740_DSI_STAT0));
	return 0;

fail:
	drm_err(&sdev->dev, "DSI host bring-up timed out (STAT0 %08x)\n",
		readl(d + S5L8740_DSI_STAT0));
	return ret;
}

/* sub_4235A(2) -> sub_3ED8(0), then sub_4640: low power, reset, gate closed. */
static void s5l8740_dsi_host_off(struct s5l8740_device *sdev)
{
	void __iomem *d = sdev->dsi;

	writel(readl(d + S5L8740_DSI_CTL2) | S5L8740_DSI_CTL2_LP, d + S5L8740_DSI_CTL2);
	writel(readl(d + S5L8740_DSI_PLL) & ~S5L8740_DSI_PLL_HS, d + S5L8740_DSI_PLL);
	writel(1, d + S5L8740_DSI_RESET);
	s5l8740_clkcon_gate(sdev, S5L8740_CLKCON_14, S5L8740_CLKCON_14_DSI, false);
	sdev->dsi_ready = false;
}

/* The host comes up under the first packet after a power cycle: stock's
 * order is reset released, rail on, host, packets, and the panel's prepare
 * owns the first two. */
static int s5l8740_dsi_ensure(struct s5l8740_device *sdev)
{
	if (sdev->dsi_ready)
		return 0;
	return s5l8740_dsi_host_init(sdev);
}

/*
 * sub_4870: a read. Clear the receive status, send the short packet, wait
 * for the status to say anything, reject errors and timeouts, wait for the
 * receive FIFO, then decode the response header in +0x3c: type in the low
 * six bits, a long response's length above it, a short response's bytes
 * above it. The long path re-reads the FIFO word by word as stock does.
 */
static int s5l8740_dsi_read(struct s5l8740_device *sdev, const u8 *hdr,
			    u8 *buf, size_t len)
{
	void __iomem *d = sdev->dsi;
	u32 st, w, n, i;
	int ret;

	writel(~0u, d + S5L8740_DSI_RXSTAT);
	ret = s5l8740_dsi_short(sdev, hdr[0], hdr[1], hdr[2]);
	if (ret)
		return ret;
	ret = readl_poll_timeout(d + S5L8740_DSI_RXSTAT, st,
				 st & S5L8740_DSI_RXSTAT_ANY, 10,
				 S5L8740_DSI_INIT_TIMEOUT_US);
	if (ret)
		return ret;
	if (st & S5L8740_DSI_RXSTAT_ERR)
		return -EIO;
	if (st & S5L8740_DSI_RXSTAT_TIMEOUT)
		return -ETIMEDOUT;
	/* sub_3D11B0(+0x44, bit 24, bit 24): loop while the FIFO is empty. */
	ret = readl_poll_timeout(d + S5L8740_DSI_STATUS, st,
				 !(st & S5L8740_DSI_ST_RXEMPTY), 10,
				 S5L8740_DSI_INIT_TIMEOUT_US);
	if (ret)
		return ret;
	w = readl(d + S5L8740_DSI_RXDATA);
	switch (w & 0x3f) {
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		n = (w >> 8) & 0xffff;
		for (i = 0; i < n; i++) {
			if ((i & 3) == 0) {
				ret = readl_poll_timeout(d + S5L8740_DSI_STATUS, st,
							 !(st & S5L8740_DSI_ST_RXEMPTY),
							 10, S5L8740_DSI_INIT_TIMEOUT_US);
				if (ret)
					return ret;
				w = readl(d + S5L8740_DSI_RXDATA);
			}
			if (i < len)
				buf[i] = w & 0xff;
			w >>= 8;
		}
		return min_t(size_t, n, len);
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		if (len > 1)
			buf[1] = (w >> 16) & 0xff;
		fallthrough;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		if (len > 0)
			buf[0] = (w >> 8) & 0xff;
		return min_t(size_t, (w & 0x3f) & 0x02 ? 2 : 1, len);
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		drm_dbg(&sdev->dev, "DSI ack/error %04x\n", (w >> 8) & 0xffff);
		return -EIO;
	default:
		drm_dbg(&sdev->dev, "DSI unexpected rx %08x\n", w);
		return -EPROTO;
	}
}

static ssize_t s5l8740_dsi_transfer(struct mipi_dsi_host *host,
				    const struct mipi_dsi_msg *msg)
{
	struct s5l8740_device *sdev = container_of(host, struct s5l8740_device, dsi_host);
	struct mipi_dsi_packet packet;
	ssize_t ret;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		return ret;
	mutex_lock(&sdev->dsi_lock);
	ret = s5l8740_dsi_ensure(sdev);
	if (ret)
		goto out;
	if (msg->rx_buf && msg->rx_len) {
		ret = s5l8740_dsi_read(sdev, packet.header, msg->rx_buf, msg->rx_len);
	} else if (packet.payload_length) {
		ret = s5l8740_dsi_long(sdev, packet.header[0], packet.payload,
				       packet.payload_length);
		if (!ret)
			ret = msg->tx_len;
	} else {
		ret = s5l8740_dsi_short(sdev, packet.header[0], packet.header[1],
					packet.header[2]);
		if (!ret)
			ret = msg->tx_len;
	}
out:
	mutex_unlock(&sdev->dsi_lock);
	return ret;
}

static int s5l8740_drm_setup(struct s5l8740_device *sdev);

static int s5l8740_dsi_attach(struct mipi_dsi_host *host, struct mipi_dsi_device *dsi)
{
	struct s5l8740_device *sdev = container_of(host, struct s5l8740_device, dsi_host);
	struct drm_panel *panel;

	panel = of_drm_find_panel(dsi->dev.of_node);
	if (IS_ERR(panel))
		return PTR_ERR(panel);
	sdev->panel = panel;
	/*
	 * The bootloader left the panel lit. Tell the panel framework so,
	 * or the first power-off is skipped as "already unprepared".
	 */
	if (sdev->powered) {
		panel->prepared = true;
		panel->enabled = true;
	}
	drm_info(&sdev->dev, "panel %s attached: %u lanes, format %u\n",
		 dev_name(&dsi->dev), dsi->lanes, dsi->format);
	return s5l8740_drm_setup(sdev);
}

static int s5l8740_dsi_detach(struct mipi_dsi_host *host, struct mipi_dsi_device *dsi)
{
	struct s5l8740_device *sdev = container_of(host, struct s5l8740_device, dsi_host);

	sdev->panel = NULL;
	return 0;
}

static const struct mipi_dsi_host_ops s5l8740_dsi_host_ops = {
	.attach = s5l8740_dsi_attach,
	.detach = s5l8740_dsi_detach,
	.transfer = s5l8740_dsi_transfer,
};

/* ------------------------------------------------------------------ */
/* Frame staging                                                        */
/* ------------------------------------------------------------------ */

/* Copy a frame into a WIDTH-stride XRGB8888 layer buffer, honouring the source pitch. */
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

static void s5l8740_lcdif_arm_handoff(struct s5l8740_device *sdev);

/*
 * The frame kick, sub_A25C0(0) exactly: refuse if the gate is open or a
 * transfer is active, open, send the full-panel window, wait two
 * microseconds, close. The cycle is the frame.
 */
static int s5l8740_comp_kick(struct s5l8740_device *sdev)
{
	void __iomem *b = sdev->lcdif;

	if (readl(b + S5L8740_LCD_XSTAT) & S5L8740_LCD_XSTAT_BUSY) {
		sdev->comp_skips++;
		return -EBUSY;
	}
	writel(1, b + S5L8740_LCD_XFER);
	if (sdev->dsi) {
		mutex_lock(&sdev->dsi_lock);
		s5l8740_dsi_window(sdev, 0, 0, WIDTH, HEIGHT);
		mutex_unlock(&sdev->dsi_lock);
	}
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

/* Which parts of the bring-up a (re-)program runs. */
#define S5L8740_COMP_REINIT_LUT		BIT(0)	/* sub_52BAC */
#define S5L8740_COMP_REINIT_CABC	BIT(1)	/* sub_4BB118 */

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

	if (READ_ONCE(sdev->comp_ready))
		return true;
	if (READ_ONCE(sdev->comp_failed))
		return false;

	ret = s5l8740_comp_setup(sdev);
	if (ret) {
		sdev->comp_failed = true;
		drm_err(&sdev->dev,
			"compositor bring-up failed (%d); nothing will be drawn\n",
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
	struct drm_fb_helper *helper = sdev->dev.fb_helper;
	unsigned int i;
	int ret;

	/*
	 * fb_deferred_io reads its delay every time it schedules, so the
	 * helper's 50 ms default can be replaced here, once the fbdev exists,
	 * without touching the helper's setup path. See lcd_fbdefio_ms.
	 */
	if (helper && lcd_fbdefio_ms) {
		unsigned long want = max_t(unsigned long, 1,
					   msecs_to_jiffies(lcd_fbdefio_ms));

		if (helper->fbdefio.delay != want)
			helper->fbdefio.delay = want;
	}

	mutex_lock(&sdev->comp_lock);
	cancel_delayed_work(&sdev->comp_retry);
	sdev->comp_pending = false;
	sdev->comp_retries = 0;

	s5l8740_comp_wait_idle(sdev);

	i = sdev->cbuf_back;
	if (src)
		s5l8740_stage_into(sdev->cbuf[i], src, pitch_px, w, h);
	else
		memset(sdev->cbuf[i], 0, S5L8740_COMP_FRAME_BYTES);
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

static void s5l8740_primary_plane_helper_atomic_update(struct drm_plane *plane,
						       struct drm_atomic_state *state)
{
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
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
	 * Never draw at a stopped interface. The panel is repainted by
	 * n31_lcd_power() once it is running again.
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
	 * Stock refreshes the panel exactly one way at runtime: the layer
	 * compositor draws the frame from DRAM and sub_A25C0 cycles the
	 * LCDIF transfer gate as the kick, with the window (0, 0, w, h) set
	 * through the DSI host on every refresh. The firmware has no
	 * partial-window path, so damage clips are not used here either.
	 */
	if (s5l8740_comp_available(sdev))
		s5l8740_comp_frame(sdev, src, pitch_px, fb->width, fb->height);

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

	if (!sdev->panel)
		return 0;
	return drm_panel_get_modes(sdev->panel, connector);
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

static struct s5l8740_device *s5l8740_lcd_dev;

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
	/* sub_41CBD8(8, 1): the LCDIF gate, closed again by the off path. */
	s5l8740_clkcon_gate(sdev, S5L8740_CLKCON_08, S5L8740_CLKCON_08_LCDIF, true);

	writel(0x000a000a, b + S5L8740_LCD_UNK78);
	writel(keep | S5L8740_CON_BASE, b + S5L8740_LCD_CON);
	writel(1, b + S5L8740_LCD_UNK2C);
	writel(0, b + S5L8740_LCD_UNK68);
	writel(0, b + S5L8740_LCD_UNK70);
	writel(HEIGHT | (WIDTH << 16), b + S5L8740_LCD_SIZE);
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
	 * is gated together with the +0x70 register: the pair arms the
	 * LCDIF's request generator, without which the interface takes one
	 * burst and never asks for another.
	 *
	 * sub_429B4(1) writes both, and the compositor kick depends on the
	 * request generator being armed, so they are written on every program.
	 */
	writel(1, b + S5L8740_LCD_UNK70);
	writel(readl(b + S5L8740_LCD_CON) | S5L8740_CON_REQ_EN,
	       b + S5L8740_LCD_CON);
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
 * With the request generator unarmed the LCDIF never raises DMACBREQ:
 * a channel feeding it moves one burst of DBSize (4 transfers, 16 bytes)
 * and stops. Measured here as "residue 414704 of 414720".
 *
 * Called from the compositor bring-up rather than from probe. Arming at
 * probe hung the machine on 2026-09-07: CON was rewritten with no reset,
 * no program and no busy check while fbcon was drawing a frame, and the box never
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
	 * draws correctly. Whatever bit 12 reports on
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
	if (!sdev->panel)
		return -ENODEV;

	/*
	 * sub_1C20: the panel's prepare releases its reset line, raises the
	 * rail and sends sleep-out; the DSI host comes up under that first
	 * packet (sub_2AFC(0) and sub_4235A(3)). Then the LCDIF (sub_2C64(0)),
	 * one blank frame (the script's third opcode), and the panel's enable
	 * for display-on and its enable line.
	 */
	ret = drm_panel_prepare(sdev->panel);
	if (ret)
		return ret;

	for (try = 0; try < S5L8740_LCD_RESET_TRIES; try++) {
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
			S5L8740_LCD_RESET_TRIES);
		drm_panel_unprepare(sdev->panel);
		return ret;
	}

	sdev->powered = true;
	if (s5l8740_comp_available(sdev))
		s5l8740_comp_frame(sdev, NULL, 0, WIDTH, HEIGHT);
	ret = drm_panel_enable(sdev->panel);
	if (ret)
		drm_warn(&sdev->dev, "panel enable failed: %d\n", ret);
	drm_info(&sdev->dev, "display on (CON=%08x STATUS=%08x)\n",
		 readl(sdev->lcdif + S5L8740_LCD_CON),
		 readl(sdev->lcdif + S5L8740_LCD_STATUS));
	return 0;
}

static void s5l8740_lcd_power_off_locked(struct s5l8740_device *sdev)
{
	void __iomem *b = sdev->lcdif;

	if (!sdev->powered)
		return;

	cancel_delayed_work_sync(&sdev->comp_retry);
	/*
	 * sub_1D04: the panel's disable drops its enable line and runs the
	 * off scripts; then sub_46DC stops the LCDIF (request enable off, a
	 * light reset, the gate closed), sub_4640 puts the host to sleep, and
	 * the panel's unprepare drops the rail and asserts reset.
	 */
	if (sdev->panel)
		drm_panel_disable(sdev->panel);
	writel(readl(b + S5L8740_LCD_CON) & ~(S5L8740_CON_RUN | S5L8740_CON_REQ_EN),
	       b + S5L8740_LCD_CON);
	s5l8740_lcdif_reset(sdev, true);
	s5l8740_clkcon_gate(sdev, S5L8740_CLKCON_08, S5L8740_CLKCON_08_LCDIF, false);
	sdev->handoff_armed = false;
	s5l8740_dsi_host_off(sdev);
	if (sdev->panel)
		drm_panel_unprepare(sdev->panel);
	sdev->powered = false;
	drm_info(&sdev->dev, "display off (CON=%08x)\n",
		 readl(b + S5L8740_LCD_CON));
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

static void s5l8740_crtc_atomic_enable(struct drm_crtc *crtc,
				       struct drm_atomic_state *state)
{
	struct s5l8740_device *sdev = s5l8740_device_of_dev(crtc->dev);

	mutex_lock(&sdev->power_lock);
	s5l8740_lcd_power_on_locked(sdev);
	mutex_unlock(&sdev->power_lock);
}

static void s5l8740_crtc_atomic_disable(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct s5l8740_device *sdev = s5l8740_device_of_dev(crtc->dev);

	mutex_lock(&sdev->power_lock);
	s5l8740_lcd_power_off_locked(sdev);
	mutex_unlock(&sdev->power_lock);
}

static const struct drm_crtc_helper_funcs s5l8740_crtc_helper_funcs = {
	.atomic_check = drm_crtc_helper_atomic_check,
	.atomic_enable = s5l8740_crtc_atomic_enable,
	.atomic_disable = s5l8740_crtc_atomic_disable,
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

/*
 * DRM_SIMPLE_MODE() carries a 1 kHz clock, which makes the mode report a
 * refresh of 0 to clients that pace on it. The compositor path delivers
 * about 30 frames a second, so say so: clock = 240 * 432 * 30 / 1000 kHz
 * with no blanking, which drm_mode_vrefresh() reads back as 30.
 */
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
 * Everything DRM, built once the panel has attached to the host: mode
 * config, the one plane, CRTC, encoder and the connector the panel gives
 * its mode to.
 */
static int s5l8740_drm_setup(struct s5l8740_device *sdev)
{
	struct drm_device *dev = &sdev->dev;
	const struct drm_format_info *format = sdev->format;
	struct drm_plane *primary_plane;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	size_t nformats;
	int ret;

	if (sdev->drm_ready)
		return 0;

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
     DRM_MODE_CONNECTOR_DSI);
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
    sdev->drm_ready = true;
    return 0;
}

static int s5l8740_probe(struct platform_device *pdev)
{
    struct s5l8740_device *sdev;
    struct drm_device *dev;
    struct resource *res;
    const struct drm_format_info *format;
    int ret;

	sdev = devm_drm_dev_alloc(&pdev->dev, &s5l8740_driver, struct s5l8740_device, dev);
    if (IS_ERR(sdev))
        return PTR_ERR(sdev);

    format = drm_format_info(DRM_FORMAT_XRGB8888);
    sdev->format = format;

    dev = &sdev->dev;

    sdev->lcdif = devm_platform_ioremap_resource_byname(pdev, "lcdif");
    if (IS_ERR(sdev->lcdif))
        return PTR_ERR(sdev->lcdif);
    mutex_init(&sdev->power_lock);
    mutex_init(&sdev->dsi_lock);

    /* The MIPI DSI host: the panel window is set through it on every frame. */
    sdev->dsi = devm_platform_ioremap_resource_byname(pdev, "dsi");
    if (IS_ERR(sdev->dsi))
        return dev_err_probe(&pdev->dev, PTR_ERR(sdev->dsi), "dsi window\n");

    /* The layer compositor, the engine every frame is drawn with. */
    sdev->comp = devm_platform_ioremap_resource_byname(pdev, "compositor");
    if (IS_ERR(sdev->comp))
        return dev_err_probe(&pdev->dev, PTR_ERR(sdev->comp),
                             "compositor window\n");

    /*
     * The clock controller. Two gates in it are cycled across an LCDIF
     * reset, and nothing else is touched.
     */
    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "clkcon");
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
     * The DSI host. The panel driver binds to the child node, attaches,
     * and that attach builds the DRM device: there is no display to
     * register before there is a panel to show it on.
     */
    s5l8740_dsi_capture(sdev);
    sdev->dsi_host.dev = &pdev->dev;
    sdev->dsi_host.ops = &s5l8740_dsi_host_ops;
    ret = mipi_dsi_host_register(&sdev->dsi_host);
    if (ret)
        return dev_err_probe(&pdev->dev, ret, "dsi host\n");
    if (!sdev->drm_ready)
        drm_info(dev, "waiting for the panel driver\n");
    return 0;
}
 
/*
 * The LCDIF has no framebuffer-base or stride register -- it is a
 * FIFO-fed command-mode interface. What reads memory is the compositor's
 * layer, and across a kexec its buffers belong to the next kernel. So
 * stop the kick retry, then power the panel down; n31_lcd_power suspends
 * the DRM clients on the way, so nothing is left drawing into a stopped
 * interface.
 */
static void s5l8740_shutdown(struct platform_device *pdev)
{
	struct s5l8740_device *sdev = platform_get_drvdata(pdev);

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
    if (sdev->drm_ready)
        drm_dev_unplug(dev);
    mipi_dsi_host_unregister(&sdev->dsi_host);
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
