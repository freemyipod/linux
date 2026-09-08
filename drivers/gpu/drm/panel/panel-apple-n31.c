// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple iPod nano 7th generation (N31) display panel.
 *
 * A 240x432 DCS command-mode panel on a one-lane MIPI DSI link, driven by
 * the S5L8740 display controller's DSI host. Two panel variants exist and
 * are told apart by the first byte of manufacturer register 0xA1: 0x11 is
 * a Samsung-style controller that needs its level-2 command key and a
 * display-control block before display-on; every other id takes the bare
 * DCS sequence.
 *
 * The sequences are the firmware's own, from its display power-on and
 * power-off command scripts (sub_1C20, sub_1D04 and the script interpreter
 * sub_2FEC), split where the firmware pushes one blank frame between
 * sleep-out and display-on: that step is the DRM panel contract's boundary
 * between prepare() and enable().
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define N31_PANEL_ID_REG	0xa1
#define N31_PANEL_ID_SAMSUNG	0x11
#define N31_PANEL_ID_TRIES	5

struct n31_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;	/* SoC pad 15, active low */
	struct gpio_desc *enable_gpio;	/* SoC pad 14, active high */
	u32 id;
	bool id_valid;
};

static inline struct n31_panel *to_n31_panel(struct drm_panel *panel)
{
	return container_of(panel, struct n31_panel, panel);
}

/*
 * 240x432 at the 30 Hz the controller refreshes at. There is no timing on
 * a command-mode link; the blanking is zero because the numbers are only
 * there to give the mode a clock.
 */
static const struct drm_display_mode n31_panel_mode = {
	.clock = 240 * 432 * 30 / 1000,
	.hdisplay = 240,
	.hsync_start = 240,
	.hsync_end = 240,
	.htotal = 240,
	.vdisplay = 432,
	.vsync_start = 432,
	.vsync_end = 432,
	.vtotal = 432,
	.width_mm = 30,
	.height_mm = 56,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

/*
 * sub_2F68: set the maximum return size to four bytes, read register 0xA1,
 * keep the first byte as the variant. The firmware tries five times and
 * gives up on the panel if none succeeds; here a failed read leaves the
 * variant unknown and the bare DCS sequence is used, which lights every
 * panel this hardware has been seen with.
 */
static void n31_panel_read_id(struct n31_panel *p)
{
	struct mipi_dsi_device *dsi = p->dsi;
	u8 buf[4];
	unsigned int i;
	int ret;

	if (p->id_valid)
		return;
	for (i = 0; i < N31_PANEL_ID_TRIES; i++) {
		ret = mipi_dsi_set_maximum_return_packet_size(dsi, sizeof(buf));
		if (!ret)
			ret = mipi_dsi_dcs_read(dsi, N31_PANEL_ID_REG, buf,
						sizeof(buf));
		if (ret == sizeof(buf)) {
			p->id = ((u32)buf[0] << 24) | ((u32)buf[1] << 16) |
				((u32)buf[2] << 8) | buf[3];
			p->id_valid = true;
			dev_info(&dsi->dev, "panel id %08x (%s sequence)\n",
				 p->id,
				 (p->id >> 24) == N31_PANEL_ID_SAMSUNG ?
				 "samsung" : "dcs");
			return;
		}
		usleep_range(1000, 2000);
	}
	dev_warn(&dsi->dev, "panel id read failed (%d); using the dcs sequence\n",
		 ret);
}

/*
 * sub_1C20 up to the blank frame: reset released, rail on, then the
 * script's "nop, sleep out, 160 ms". The wait between the rail and the
 * first packet is a firmware delay whose length is a ROM thunk; 3 ms is
 * the stand-in used throughout this port.
 */
static int n31_panel_prepare(struct drm_panel *panel)
{
	struct n31_panel *p = to_n31_panel(panel);
	struct mipi_dsi_device *dsi = p->dsi;
	int ret;

	gpiod_set_value_cansleep(p->reset_gpio, 0);
	ret = regulator_enable(p->supply);
	if (ret) {
		dev_err(&dsi->dev, "rail: %d\n", ret);
		return ret;
	}
	usleep_range(3000, 3500);

	n31_panel_read_id(p);

	ret = mipi_dsi_dcs_nop(dsi);
	if (!ret)
		ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret) {
		dev_err(&dsi->dev, "sleep out: %d\n", ret);
		regulator_disable(p->supply);
		return ret;
	}
	msleep(160);
	return 0;
}

/*
 * The rest of the on script, after the controller has pushed a frame:
 * the Samsung variant's key and display-control block, display on, 34 ms,
 * then the enable line (sub_38DC on the pad the firmware keeps at
 * 0x891DF30, pad 14).
 */
static int n31_panel_enable(struct drm_panel *panel)
{
	struct n31_panel *p = to_n31_panel(panel);
	struct mipi_dsi_device *dsi = p->dsi;
	int ret;

	if (p->id_valid && (p->id >> 24) == N31_PANEL_ID_SAMSUNG) {
		static const u8 key_on[] = { 0xf1, 0x5a, 0x5a };
		static const u8 disp_ctl[] = {
			0xf2, 0x00, 0x77, 0x03, 0x0a, 0x76, 0x03, 0x01, 0x01,
			0x82, 0x01, 0xba, 0x57, 0x00, 0x00, 0x77, 0x0a, 0x76
		};
		static const u8 key_off[] = { 0xf1, 0xa5, 0xa5 };
		static const u8 madctl[] = { 0x36, 0x10, 0x00 };

		ret = mipi_dsi_dcs_write_buffer(dsi, key_on, sizeof(key_on));
		if (ret >= 0)
			ret = mipi_dsi_dcs_write_buffer(dsi, disp_ctl,
							sizeof(disp_ctl));
		if (ret >= 0)
			ret = mipi_dsi_dcs_write_buffer(dsi, key_off,
							sizeof(key_off));
		/* Generic long write, as the script has it: not a DCS packet. */
		if (ret >= 0)
			ret = mipi_dsi_generic_write(dsi, madctl, sizeof(madctl));
		if (ret < 0) {
			dev_err(&dsi->dev, "samsung block: %d\n", ret);
			return ret;
		}
	}

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret) {
		dev_err(&dsi->dev, "display on: %d\n", ret);
		return ret;
	}
	msleep(34);
	gpiod_set_value_cansleep(p->enable_gpio, 1);
	return 0;
}

/*
 * sub_1D04's first half: the enable line low, then the two off scripts --
 * display off with 5 ms, sleep in with 120 ms.
 */
static int n31_panel_disable(struct drm_panel *panel)
{
	struct n31_panel *p = to_n31_panel(panel);
	struct mipi_dsi_device *dsi = p->dsi;
	int ret;

	gpiod_set_value_cansleep(p->enable_gpio, 0);
	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		dev_warn(&dsi->dev, "display off: %d\n", ret);
	msleep(5);
	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		dev_warn(&dsi->dev, "sleep in: %d\n", ret);
	msleep(120);
	return 0;
}

/* sub_1D04's tail: rail down, then the reset line asserted. */
static int n31_panel_unprepare(struct drm_panel *panel)
{
	struct n31_panel *p = to_n31_panel(panel);

	regulator_disable(p->supply);
	gpiod_set_value_cansleep(p->reset_gpio, 1);
	return 0;
}

static int n31_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &n31_panel_mode);
	if (!mode)
		return -ENOMEM;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bpc = 8;
	return 1;
}

static const struct drm_panel_funcs n31_panel_funcs = {
	.prepare = n31_panel_prepare,
	.enable = n31_panel_enable,
	.disable = n31_panel_disable,
	.unprepare = n31_panel_unprepare,
	.get_modes = n31_panel_get_modes,
};

static int n31_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct n31_panel *p;
	int ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dsi = dsi;

	p->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(p->supply))
		return dev_err_probe(dev, PTR_ERR(p->supply), "power supply\n");
	/*
	 * A panel the bootloader lit is running on this rail already. Take
	 * the reference now, or the regulator core's late cleanup counts the
	 * rail as unused and switches it off 30 s into boot -- measured
	 * 2026-09-08 as "PMU_LDO_7: disabling" at 33.8 s, followed by an
	 * "unbalanced disables" warning at the first power-off. The
	 * controller marks the panel prepared and enabled for the same
	 * handoff, so unprepare() balances this.
	 */
	if (regulator_is_enabled(p->supply) > 0) {
		ret = regulator_enable(p->supply);
		if (ret)
			return dev_err_probe(dev, ret, "claiming the lit panel's rail\n");
	}

	/*
	 * Both lines are taken as they are: the bootloader has already lit
	 * the panel and these hold the levels of a running display. They
	 * are driven only by the sequences above.
	 */
	p->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(p->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(p->reset_gpio), "reset gpio\n");
	p->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_ASIS);
	if (IS_ERR(p->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(p->enable_gpio), "enable gpio\n");

	/*
	 * One data lane: the host's CTL word as the bootloader leaves it is
	 * 0x00707003, whose lane field (2 * (1 << n) - 2, sub_2AFC) is 2, so
	 * n = 1. The format field there is 7, which is the firmware's own
	 * code for its mode selector 0xE; RGB888 is what the compositor feeds.
	 */
	dsi->lanes = 1;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM;

	drm_panel_init(&p->panel, dev, &n31_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	/*
	 * The backlight rides on the panel: the panel framework switches it
	 * off before disable() and on after enable(), which is the order the
	 * firmware's power-off takes too (sub_4D08(0) is the first thing
	 * sub_1D04 does). One DPMS off from a DRM master then darkens the
	 * glass and puts the panel to sleep in one step.
	 */
	ret = drm_panel_of_backlight(&p->panel);
	if (ret)
		return dev_err_probe(dev, ret, "backlight\n");
	drm_panel_add(&p->panel);

	mipi_dsi_set_drvdata(dsi, p);
	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&p->panel);
		return dev_err_probe(dev, ret, "dsi attach\n");
	}
	return 0;
}

static void n31_panel_remove(struct mipi_dsi_device *dsi)
{
	struct n31_panel *p = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&p->panel);
}

static const struct of_device_id n31_panel_of_match[] = {
	{ .compatible = "apple,n31-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, n31_panel_of_match);

static struct mipi_dsi_driver n31_panel_driver = {
	.probe = n31_panel_probe,
	.remove = n31_panel_remove,
	.driver = {
		.name = "panel-apple-n31",
		.of_match_table = n31_panel_of_match,
	},
};
module_mipi_dsi_driver(n31_panel_driver);

MODULE_DESCRIPTION("Apple iPod nano 7G display panel");
MODULE_LICENSE("GPL");
