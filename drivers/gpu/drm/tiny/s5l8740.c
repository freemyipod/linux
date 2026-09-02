// SPDX-License-Identifier: GPL-2.0-only

#include <linux/apple-n31.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

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
#define S5L8740_LCD_XSTAT		0x8c	/* bits 0-1 busy */
#define S5L8740_LCD_XSTAT_BUSY	0x3
#define S5L8740_LCD_XFER_TIMEOUT_US	100000

#define WIDTH 240
#define HEIGHT 432

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
	bool rail_held;

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
 * Bracket each update with the LCDIF transfer gate. Off reproduces the
 * unframed streaming this driver did before, for comparison.
 */
static bool lcd_frame_xfer = true;
module_param(lcd_frame_xfer, bool, 0644);
MODULE_PARM_DESC(lcd_frame_xfer,
		 "Open and close the LCDIF transfer around each update (default Y)");

/* Wait for the interface to leave a transfer. */
static int s5l8740_xfer_idle(struct s5l8740_device *sdev)
{
	u32 st;

	return readl_poll_timeout_atomic(sdev->lcdif + S5L8740_LCD_XSTAT, st,
					 !(st & S5L8740_LCD_XSTAT_BUSY), 0,
					 S5L8740_LCD_XFER_TIMEOUT_US);
}

static int s5l8740_xfer_open(struct s5l8740_device *sdev)
{
	int ret;

	if (!lcd_frame_xfer)
		return 0;
	/*
	 * Opening on top of a transfer still running is what stock refuses
	 * outright, returning -1 rather than queueing behind it.
	 */
	ret = s5l8740_xfer_idle(sdev);
	if (ret)
		return ret;
	writel(1, sdev->lcdif + S5L8740_LCD_XFER);
	return 0;
}

static void s5l8740_xfer_close(struct s5l8740_device *sdev)
{
	if (!lcd_frame_xfer)
		return;
	/*
	 * Let the interface drain before dropping the gate. Closing under a
	 * transfer truncates it, which leaves the panel mid-row and is the
	 * failure this framing exists to prevent.
	 */
	s5l8740_xfer_idle(sdev);
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
 * Point the panel at one rectangle and stream it.
 *
 * Returns nonzero only when the interface stopped answering, which costs
 * the rest of the frame; a window command that will not take is not fatal
 * and falls through to the pixels, since the panel keeps whatever window
 * it had and one crooked frame beats a frozen display.
 */
static int s5l8740_blit(struct s5l8740_device *sdev, const u32 *src,
			unsigned int pitch_px, const struct drm_rect *clip)
{
	unsigned int x, y;

	if (sdev->dsi &&
	    s5l8740_dsi_window(sdev, clip->x1, clip->y1,
			       drm_rect_width(clip), drm_rect_height(clip)))
		drm_warn_once(&sdev->dev, "DSI window timeout\n");

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
	 * Open the transfer before any command or pixel goes out, and close it
	 * once, after the last rectangle. The bracket belongs around the whole
	 * update rather than around each rectangle: stock opens once, blits,
	 * and closes.
	 */
	if (s5l8740_xfer_open(sdev)) {
		drm_warn_once(dev, "LCDIF busy; update skipped\n");
		goto out_dev_exit;
	}

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
	 */
	if (lcd_partial) {
		struct drm_atomic_helper_damage_iter iter;
		struct drm_rect clip;

		drm_atomic_helper_damage_iter_init(&iter, old_state, plane_state);
		drm_atomic_for_each_plane_damage(&iter, &clip)
			if (s5l8740_blit(sdev, src, pitch_px, &clip))
				break;
	} else {
		struct drm_rect full = {
			.x1 = 0,
			.y1 = 0,
			.x2 = fb->width,
			.y2 = fb->height,
		};

		s5l8740_blit(sdev, src, pitch_px, &full);
	}

	s5l8740_xfer_close(sdev);
out_dev_exit:
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

#define S5L8740_CON_HOLD	BIT(10)	/* 0x400: interface held in reset */
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

static void s5l8740_lcd_rail(struct s5l8740_device *sdev, bool on)
{
	int (*get)(unsigned int);
	void (*put)(unsigned int);

	if (!lcd_manage_rail)
		return;
	if (on) {
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
static void s5l8740_lcdif_program(struct s5l8740_device *sdev)
{
	void __iomem *b = sdev->lcdif;
	u32 con = readl(b + S5L8740_LCD_CON);
	u32 keep = con & (S5L8740_CON_MODE_MASK | S5L8740_CON_FMT_MASK);

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
			  "clkcon_window=%s\n",
			  sdev->powered, sdev->rail_held,
			  readl(sdev->lcdif + S5L8740_LCD_CON),
			  readl(sdev->lcdif + S5L8740_LCD_STATUS),
			  readl(sdev->lcdif + S5L8740_LCD_SIZE),
			  sdev->clkcon ? "mapped" : "absent");
}
static DEVICE_ATTR_RO(lcd_state);

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
    s5l8740_lcd_dev = sdev;

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
 * The LCDIF scans out of the framebuffer by DMA. Across a kexec that
 * memory belongs to the next kernel, so the controller would keep
 * fetching whatever landed there and the panel would show it. Powering
 * down stops the fetch; n31_lcd_power suspends the DRM clients on the
 * way, so nothing is left drawing into a stopped interface.
 */
static void s5l8740_shutdown(struct platform_device *pdev)
{
	n31_lcd_power(false);
}

static void s5l8740_remove(struct platform_device *pdev)
{
    struct s5l8740_device *sdev = platform_get_drvdata(pdev);
    struct drm_device *dev = &sdev->dev;

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
