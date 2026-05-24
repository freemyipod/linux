// SPDX-License-Identifier: GPL-2.0
/*
 * Framebuffer driver for the Apple/Samsung S5L8702 LCD controller
 * (iPod nano 3rd generation / N46), 320x240 RGB565.
 *
 * The S5L8702 has a custom 8080-series parallel MPU interface block.
 * The attached panel is one of several ILI93xx-family controllers
 * auto-detected by reading the display ID (command 0x04).  Init
 * sequences are derived from the Rockbox port (firmware/target/arm/
 * s5l8702/ipodnano3g/lcd-nano3g.c, lcd-s5l8702.c).
 *
 */

#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#define DPY_W		320
#define DPY_H		240
#define BPP_BYTES	2
#define FB_SIZE		(DPY_W * DPY_H * BPP_BYTES)

/* LCD controller registers (offsets from block base) */
#define LCD_CONFIG	0x00
#define LCD_WCMD	0x04
#define LCD_RCMD	0x0c
#define LCD_RDATA	0x10
#define LCD_DBUFF	0x14
#define LCD_INTCON	0x18
#define LCD_STATUS	0x1c
#define LCD_PHTIME	0x20
#define LCD_WDATA	0x40

/* LCD_STATUS bits */
#define LCD_STATUS_RXRDY	BIT(0)	/* read-data valid */
#define LCD_STATUS_TXIDLE	BIT(1)	/* tx FIFO idle */
#define LCD_STATUS_TXBUSY	BIT(4)	/* tx FIFO busy */

/* LCD_CONFIG values - opaque magic from Rockbox / firmware RE */
#define LCD_MODE_P8	0x80000c20	/* 8-bit parallel - commands */
#define LCD_MODE_P9	0x81100db8	/* 9-bit parallel - frame data */

/* PWRCON0 clock-gate (PWRCON0 bit 1 = LCD) */
#define S5L8702_PWRCON0_PHYS	0x3c500048UL
#define PWRCON0_LCD_BIT		BIT(1)

/* Dialog D1671 PMU (I2C) - backlight control.  No upstream backlight
 * driver exists for this PMU yet; we just poke it directly here to
 * get pixels visible.  Brightness is the low 5 bits; bit 7 enables.
 */
#define D1671_I2C_ADDR		0x73
#define D1671_REG_LEDCTL	0x20
#define D1671_LEDCTL_ENABLE	BIT(7)
#define D1671_LEDCTL_BRIGHT_MAX	0x1f

/* Panel commands (8-bit set) */
#define R_COLUMN_ADDR_SET	0x2a
#define R_ROW_ADDR_SET		0x2b
#define R_MEMORY_WRITE		0x2c
#define R_READ_DISPLAY_ID	0x04

/* Init-sequence opcodes */
enum {
	SEQ_CMD = 0,	/* CMD, opcode, len, <len bytes> */
	SEQ_SLEEP,	/* SLEEP, ms/10 */
	SEQ_END = 0xff,
};

/* Panel types, identified by display-ID bytes [1..2] */
enum {
	LCD_TYPE_38B3 = 0,
	LCD_TYPE_38C4,
	LCD_TYPE_38D5,
	LCD_TYPE_38E6,
	LCD_TYPE_58XX,
	N_LCD_TYPES,
};

/*
 * Init sequences ported verbatim from Rockbox.  The action opcode is
 * one byte followed by either:
 *   CMD: panel command byte, payload length, then payload bytes
 *   SLEEP: a delay in centiseconds (Rockbox tick = 10 ms)
 */

/* 0xb3 */
static const u8 lcd_init_seq_b3[] = {
	SEQ_CMD,   0xef,  1, 0x80,
	SEQ_CMD,   0xc0,  1, 0x06,
	SEQ_CMD,   0xc1,  1, 0x03,
	SEQ_CMD,   0xc2,  2, 0x12, 0x00,
	SEQ_CMD,   0xc3,  2, 0x12, 0x00,
	SEQ_CMD,   0xc4,  2, 0x12, 0x00,
	SEQ_CMD,   0xc5,  2, 0x40, 0x38,
	SEQ_CMD,   0xb1,  2, 0x5f, 0x3f,
	SEQ_CMD,   0xb2,  2, 0x5f, 0x3f,
	SEQ_CMD,   0xb3,  2, 0x5f, 0x3f,
	SEQ_CMD,   0xb4,  1, 0x02,
	SEQ_CMD,   0xb6,  2, 0x12, 0x02,
	SEQ_CMD,   0x35,  1, 0x00,
	SEQ_CMD,   0x26,  1, 0x10,
	SEQ_CMD,   0xfe,  1, 0x00,
	SEQ_CMD,   0xe0, 11, 0x0f, 0x70, 0x47, 0x03, 0x02, 0x02, 0xa0, 0x94, 0x05, 0x00, 0x0e,
	SEQ_CMD,   0xe1, 11, 0x02, 0x43, 0x77, 0x00, 0x0f, 0x05, 0x49, 0x0a, 0x02, 0x0e, 0x00,
	SEQ_CMD,   0xe2, 11, 0x2f, 0x63, 0x20, 0x50, 0x00, 0x07, 0xd1, 0x13, 0x00, 0x00, 0x0e,
	SEQ_CMD,   0xe3, 11, 0x50, 0x20, 0x60, 0x23, 0x0f, 0x00, 0x31, 0x1d, 0x07, 0x0e, 0x00,
	SEQ_CMD,   0xe4, 11, 0x5e, 0x50, 0x65, 0x27, 0x00, 0x0b, 0xdf, 0xf1, 0x01, 0x00, 0x0e,
	SEQ_CMD,   0xe5, 11, 0x20, 0x67, 0x55, 0x50, 0x0e, 0x01, 0x1f, 0xfd, 0x0b, 0x0e, 0x00,
	SEQ_CMD,   0x3a,  1, 0x06,
	SEQ_CMD,   0x36,  1, 0x60,
	SEQ_CMD,   0x13,  0,
	SEQ_CMD,   0x11,  0,
	SEQ_SLEEP, 12,
	SEQ_CMD,   0x29,  0,
	SEQ_SLEEP, 1,
	SEQ_END,
};

/* 0xc4 */
static const u8 lcd_init_seq_c4[] = {
	SEQ_CMD,   0x01,  0,
	SEQ_SLEEP, 1,
	SEQ_CMD,   0xc0,  1, 0x01,
	SEQ_CMD,   0xc1,  1, 0x03,
	SEQ_CMD,   0xc2,  2, 0x74, 0x00,
	SEQ_CMD,   0xc3,  2, 0x72, 0x03,
	SEQ_CMD,   0xc4,  2, 0x73, 0x03,
	SEQ_CMD,   0xc5,  2, 0x3c, 0x3c,
	SEQ_CMD,   0xb1,  2, 0x6a, 0x15,
	SEQ_CMD,   0xb2,  2, 0x6a, 0x15,
	SEQ_CMD,   0xb3,  2, 0x6a, 0x15,
	SEQ_CMD,   0xb4,  1, 0x02,
	SEQ_CMD,   0xb6,  2, 0x12, 0x02,
	SEQ_CMD,   0x35,  1, 0x00,
	SEQ_CMD,   0x26,  1, 0x10,
	SEQ_CMD,   0xe0, 11, 0x1e, 0x22, 0x44, 0x00, 0x09, 0x01, 0x47, 0xc1, 0x05, 0x02, 0x09,
	SEQ_CMD,   0xe1, 11, 0x0f, 0x32, 0x35, 0x00, 0x03, 0x05, 0x5e, 0x78, 0x03, 0x00, 0x03,
	SEQ_CMD,   0xe2, 11, 0x0d, 0x74, 0x47, 0x41, 0x07, 0x01, 0x74, 0x41, 0x09, 0x03, 0x07,
	SEQ_CMD,   0xe3, 11, 0x5f, 0x41, 0x27, 0x02, 0x00, 0x03, 0x43, 0x55, 0x02, 0x00, 0x03,
	SEQ_CMD,   0xe4, 11, 0x1b, 0x53, 0x44, 0x51, 0x0b, 0x01, 0x64, 0x20, 0x05, 0x02, 0x09,
	SEQ_CMD,   0xe5, 11, 0x7f, 0x41, 0x26, 0x02, 0x04, 0x00, 0x33, 0x35, 0x01, 0x00, 0x02,
	SEQ_CMD,   0x3a,  1, 0x66,
	SEQ_CMD,   0x36,  1, 0x60,
	SEQ_CMD,   0x11,  0,
	SEQ_SLEEP, 12,
	SEQ_CMD,   0x29,  0,
	SEQ_SLEEP, 1,
	SEQ_END,
};

/* 0xd5 */
static const u8 lcd_init_seq_d5[] = {
	SEQ_CMD,   0xfe,  1, 0x00,
	SEQ_CMD,   0xc0,  1, 0x00,
	SEQ_CMD,   0xc1,  1, 0x03,
	SEQ_CMD,   0xc2,  2, 0x73, 0x03,
	SEQ_CMD,   0xc3,  2, 0x73, 0x03,
	SEQ_CMD,   0xc4,  2, 0x73, 0x03,
	SEQ_CMD,   0xc5,  2, 0x64, 0x37,
	SEQ_CMD,   0xb1,  2, 0x69, 0x13,
	SEQ_CMD,   0xb2,  2, 0x69, 0x13,
	SEQ_CMD,   0xb3,  2, 0x69, 0x13,
	SEQ_CMD,   0xb4,  1, 0x02,
	SEQ_CMD,   0xb6,  2, 0x03, 0x12,
	SEQ_CMD,   0x35,  1, 0x00,
	SEQ_CMD,   0x26,  1, 0x10,
	SEQ_CMD,   0xe0, 11, 0x08, 0x00, 0x10, 0x00, 0x03, 0x0e, 0xc8, 0x65, 0x05, 0x00, 0x00,
	SEQ_CMD,   0xe1, 11, 0x06, 0x20, 0x00, 0x00, 0x00, 0x07, 0x4d, 0x0b, 0x08, 0x00, 0x00,
	SEQ_CMD,   0xe2, 11, 0x08, 0x77, 0x27, 0x63, 0x0f, 0x16, 0xcf, 0x25, 0x03, 0x00, 0x00,
	SEQ_CMD,   0xe3, 11, 0x5f, 0x53, 0x77, 0x06, 0x00, 0x02, 0x4b, 0x7b, 0x0f, 0x00, 0x00,
	SEQ_CMD,   0xe4, 11, 0x08, 0x46, 0x57, 0x52, 0x0f, 0x16, 0xcf, 0x25, 0x04, 0x00, 0x00,
	SEQ_CMD,   0xe5, 11, 0x6f, 0x42, 0x57, 0x06, 0x00, 0x04, 0x43, 0x7b, 0x0f, 0x00, 0x00,
	SEQ_CMD,   0x3a,  1, 0x66,
	SEQ_CMD,   0x36,  1, 0x60,
	SEQ_CMD,   0x11,  0,
	SEQ_SLEEP, 12,
	SEQ_CMD,   0x29,  0,
	SEQ_SLEEP, 1,
	SEQ_END,
};

/* 0xe6 */
static const u8 lcd_init_seq_e6[] = {
	SEQ_CMD,   0xef,  1, 0x80,
	SEQ_CMD,   0xc0,  1, 0x0a,
	SEQ_CMD,   0xc1,  1, 0x03,
	SEQ_CMD,   0xc2,  2, 0x12, 0x00,
	SEQ_CMD,   0xc3,  2, 0x12, 0x00,
	SEQ_CMD,   0xc4,  2, 0x12, 0x00,
	SEQ_CMD,   0xc5,  2, 0x38, 0x38,
	SEQ_CMD,   0xb1,  2, 0x5f, 0x3f,
	SEQ_CMD,   0xb2,  2, 0x5f, 0x3f,
	SEQ_CMD,   0xb3,  2, 0x5f, 0x3f,
	SEQ_CMD,   0xb4,  1, 0x02,
	SEQ_CMD,   0xb6,  2, 0x12, 0x02,
	SEQ_CMD,   0x35,  1, 0x00,
	SEQ_CMD,   0x26,  1, 0x10,
	SEQ_CMD,   0xfe,  1, 0x00,
	SEQ_CMD,   0xe0, 11, 0x0f, 0x70, 0x47, 0x03, 0x02, 0x02, 0xa0, 0x94, 0x05, 0x00, 0x0e,
	SEQ_CMD,   0xe1, 11, 0x02, 0x43, 0x77, 0x00, 0x0f, 0x05, 0x49, 0x0a, 0x02, 0x0e, 0x00,
	SEQ_CMD,   0xe2, 11, 0x2f, 0x63, 0x20, 0x50, 0x00, 0x07, 0xd1, 0x13, 0x00, 0x00, 0x0e,
	SEQ_CMD,   0xe3, 11, 0x50, 0x20, 0x60, 0x23, 0x0f, 0x00, 0x31, 0x1d, 0x07, 0x0e, 0x00,
	SEQ_CMD,   0xe4, 11, 0x5e, 0x50, 0x65, 0x27, 0x00, 0x0b, 0xdf, 0xf1, 0x01, 0x00, 0x0e,
	SEQ_CMD,   0xe5, 11, 0x20, 0x67, 0x55, 0x50, 0x0e, 0x01, 0x1f, 0xfd, 0x0b, 0x0e, 0x00,
	SEQ_CMD,   0x3a,  1, 0x06,
	SEQ_CMD,   0x36,  1, 0x60,
	SEQ_CMD,   0x13,  0,
	SEQ_CMD,   0x11,  0,
	SEQ_SLEEP, 12,
	SEQ_CMD,   0x29,  0,
	SEQ_SLEEP, 1,
	SEQ_END,
};

/* 0x58 */
static const u8 lcd_init_seq_58[] = {
	SEQ_CMD,   0xe1,  3, 0x0f, 0x31, 0x04,
	SEQ_CMD,   0xe2,  5, 0x02, 0xa2, 0x08, 0x11, 0x01,
	SEQ_CMD,   0xe3,  2, 0x10, 0x88,
	SEQ_CMD,   0xe4,  2, 0x10, 0x88,
	SEQ_CMD,   0xe5,  2, 0x00, 0x88,
	SEQ_CMD,   0xe7, 13, 0x33, 0x0d, 0x57, 0x0e, 0x57, 0x05, 0x57, 0x02, 0x04, 0x10, 0x0b, 0x02, 0x02,
	SEQ_CMD,   0xe8,  5, 0x30, 0x0d, 0x84, 0x8c, 0x21,
	SEQ_CMD,   0xe9,  5, 0x4b, 0x1a, 0xba, 0x60, 0x11,
	SEQ_CMD,   0xea,  5, 0x4b, 0xba, 0xba, 0x10, 0x11,
	SEQ_CMD,   0xeb,  5, 0x0b, 0x3a, 0xd9, 0x60, 0x11,
	SEQ_CMD,   0xef,  3, 0x00, 0x00, 0x00,
	SEQ_CMD,   0xf0, 18, 0x12, 0x01, 0x21, 0xa5, 0x6c, 0x23, 0x02, 0x01, 0x08, 0x0d, 0x1e, 0xde, 0x5a, 0x93, 0xdc, 0x0d, 0x1e, 0x17,
	SEQ_CMD,   0xf1, 18, 0x12, 0x05, 0x21, 0xb5, 0x8d, 0x24, 0x02, 0x01, 0x08, 0x0d, 0x1a, 0xde, 0x4a, 0x72, 0xdb, 0x0d, 0x1e, 0x17,
	SEQ_CMD,   0xf2, 18, 0x0e, 0x00, 0x30, 0xd6, 0x8f, 0x34, 0x03, 0x07, 0x08, 0x11, 0x1f, 0xcf, 0x29, 0x70, 0xcb, 0x0c, 0x18, 0x17,
	SEQ_CMD,   0xfa,  3, 0x02, 0x00, 0x02,
	SEQ_CMD,   0xf7,  2, 0x00, 0xc0,
	SEQ_CMD,   0x35,  1, 0x00,
	SEQ_CMD,   0x36,  1, 0x20,
	SEQ_CMD,   0x11,  0,
	SEQ_SLEEP, 12,
	SEQ_CMD,   0x29,  0,
	SEQ_SLEEP, 1,
	SEQ_END,
};

static const u8 * const init_seq_by_type[N_LCD_TYPES] = {
	[LCD_TYPE_38B3] = lcd_init_seq_b3,
	[LCD_TYPE_38C4] = lcd_init_seq_c4,
	[LCD_TYPE_38D5] = lcd_init_seq_d5,
	[LCD_TYPE_38E6] = lcd_init_seq_e6,
	[LCD_TYPE_58XX] = lcd_init_seq_58,
};

struct s5l_lcd_par {
	struct fb_info *info;
	void __iomem *regs;
	void __iomem *pwrcon0;
	struct mutex io_lock;
	int lcd_type;
	u8 lcd_id[4];
	u32 pseudo_palette[16];
};

/* Low-level register helpers - all assume io_lock is held by caller. */

static int s5l_wait_tx_idle(struct s5l_lcd_par *par)
{
	u32 v;
	return readl_poll_timeout_atomic(par->regs + LCD_STATUS, v,
					 !(v & LCD_STATUS_TXBUSY),
					 1, 100000);
}

static int s5l_wait_tx_ready(struct s5l_lcd_par *par)
{
	u32 v;
	return readl_poll_timeout_atomic(par->regs + LCD_STATUS, v,
					 (v & LCD_STATUS_TXIDLE),
					 1, 100000);
}

static void s5l_lcd_write_config(struct s5l_lcd_par *par, u32 cfg)
{
	s5l_wait_tx_ready(par);
	udelay(1);
	writel(cfg, par->regs + LCD_CONFIG);
}

static void s5l_lcd_write_cmd(struct s5l_lcd_par *par, u16 cmd)
{
	s5l_wait_tx_idle(par);
	writel(cmd, par->regs + LCD_WCMD);
}

static void s5l_lcd_write_data(struct s5l_lcd_par *par, u16 data)
{
	s5l_wait_tx_idle(par);
	writel(data, par->regs + LCD_WDATA);
}

static void s5l_lcd_send_cmd8(struct s5l_lcd_par *par, u8 cmd,
			      int len, const u8 *data)
{
	s5l_lcd_write_cmd(par, cmd);
	while (len--)
		s5l_lcd_write_data(par, *data++);
}

static void s5l_lcd_read_id(struct s5l_lcd_par *par, u8 cmd,
			    int len, u8 *buf)
{
	u32 v;

	s5l_lcd_write_cmd(par, cmd);
	while (len--) {
		readl_poll_timeout_atomic(par->regs + LCD_STATUS, v,
					  (v & LCD_STATUS_TXIDLE),
					  1, 100000);
		writel(0, par->regs + LCD_RDATA);
		readl_poll_timeout_atomic(par->regs + LCD_STATUS, v,
					  (v & LCD_STATUS_RXRDY),
					  1, 100000);
		*buf++ = readl(par->regs + LCD_DBUFF) >> 1;
	}
}

static void s5l_lcd_run_seq(struct s5l_lcd_par *par, const u8 *seq)
{
	for (;;) {
		u8 op = *seq++;

		switch (op) {
		case SEQ_CMD: {
			u8 cmd = *seq++;
			u8 len = *seq++;
			s5l_lcd_send_cmd8(par, cmd, len, seq);
			seq += len;
			break;
		}
		case SEQ_SLEEP:
			msleep((*seq++) * 10);
			break;
		case SEQ_END:
		default:
			return;
		}
	}
}

/* Look up the PMU i2c_adapter once, so probe can defer cleanly. */
static struct i2c_adapter *s5l_lcd_get_pmu(struct device *dev)
{
	struct device_node *i2c_np;
	struct i2c_adapter *adap;

	i2c_np = of_parse_phandle(dev->of_node, "pmu-i2c", 0);
	if (!i2c_np)
		return ERR_PTR(-ENODEV);

	adap = of_find_i2c_adapter_by_node(i2c_np);
	of_node_put(i2c_np);
	if (!adap)
		return ERR_PTR(-EPROBE_DEFER);

	return adap;
}

static void s5l_lcd_backlight_on(struct device *dev, struct i2c_adapter *adap)
{
	u8 buf[2] = { D1671_REG_LEDCTL,
		      D1671_LEDCTL_ENABLE | D1671_LEDCTL_BRIGHT_MAX };
	struct i2c_msg msg = {
		.addr	= D1671_I2C_ADDR,
		.flags	= 0,
		.len	= sizeof(buf),
		.buf	= buf,
	};

	if (i2c_transfer(adap, &msg, 1) != 1)
		dev_warn(dev, "backlight enable I2C write failed\n");
}

static int s5l_lcd_detect_panel(struct s5l_lcd_par *par)
{
	int retry;

	for (retry = 0; retry < 3; retry++) {
		s5l_lcd_write_config(par, LCD_MODE_P8);
		s5l_lcd_read_id(par, R_READ_DISPLAY_ID, 4, par->lcd_id);

		if (par->lcd_id[1] == 0x58)
			return LCD_TYPE_58XX;
		if (par->lcd_id[1] == 0x38) {
			switch (par->lcd_id[2]) {
			case 0xb3: return LCD_TYPE_38B3;
			case 0xc4: return LCD_TYPE_38C4;
			case 0xd5: return LCD_TYPE_38D5;
			case 0xe6: return LCD_TYPE_38E6;
			}
		}
	}
	return -ENODEV;
}

/* Blit the framebuffer to the panel.  Called from deferred-IO context;
 * a sleeping mutex around the whole transfer is fine.
 */
static void s5l_lcd_blit(struct s5l_lcd_par *par)
{
	const u16 *src = (const u16 *)par->info->screen_buffer;
	int pixels = DPY_W * DPY_H;
	u8 col[] = { 0, 0, (DPY_W - 1) >> 8, (DPY_W - 1) & 0xff };
	u8 row[] = { 0, 0, (DPY_H - 1) >> 8, (DPY_H - 1) & 0xff };
	int i;

	mutex_lock(&par->io_lock);

	s5l_lcd_write_config(par, LCD_MODE_P8);
	s5l_lcd_send_cmd8(par, R_COLUMN_ADDR_SET, sizeof(col), col);
	s5l_lcd_send_cmd8(par, R_ROW_ADDR_SET, sizeof(row), row);
	s5l_lcd_write_cmd(par, R_MEMORY_WRITE);

	s5l_lcd_write_config(par, LCD_MODE_P9);
	for (i = 0; i < pixels; i++)
		s5l_lcd_write_data(par, src[i]);

	mutex_unlock(&par->io_lock);
}

static void s5l_lcd_deferred_io(struct fb_info *info,
				struct list_head *pagereflist)
{
	s5l_lcd_blit(info->par);
}

static void s5l_lcd_damage_range(struct fb_info *info, off_t off, size_t len)
{
	s5l_lcd_blit(info->par);
}

static void s5l_lcd_damage_area(struct fb_info *info, u32 x, u32 y,
				u32 width, u32 height)
{
	s5l_lcd_blit(info->par);
}

FB_GEN_DEFAULT_DEFERRED_SYSMEM_OPS(s5l_lcd,
				   s5l_lcd_damage_range,
				   s5l_lcd_damage_area)

static int s5l_lcd_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;

	if (regno >= 16)
		return -EINVAL;

	/* truecolor: fbcon only uses regs 0..15 to look up fg/bg pixel
	 * values; build an RGB565 from the 16-bit channel intensities
	 * the framework hands us.
	 */
	pal[regno] = ((red   & 0xf800)) |
		     ((green & 0xfc00) >> 5) |
		     ((blue  & 0xf800) >> 11);
	return 0;
}

static const struct fb_ops s5l_lcd_fbops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= s5l_lcd_setcolreg,
	FB_DEFAULT_DEFERRED_OPS(s5l_lcd),
};

static const struct fb_fix_screeninfo s5l_lcd_fix = {
	.id		= "s5l8702-lcd",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.line_length	= DPY_W * BPP_BYTES,
	.accel		= FB_ACCEL_NONE,
};

static const struct fb_var_screeninfo s5l_lcd_var = {
	.xres		= DPY_W,
	.yres		= DPY_H,
	.xres_virtual	= DPY_W,
	.yres_virtual	= DPY_H,
	.bits_per_pixel	= 16,
	.red		= { .offset = 11, .length = 5, .msb_right = 0 },
	.green		= { .offset = 5,  .length = 6, .msb_right = 0 },
	.blue		= { .offset = 0,  .length = 5, .msb_right = 0 },
	.activate	= FB_ACTIVATE_NOW,
	.vmode		= FB_VMODE_NONINTERLACED,
	.width		= -1,
	.height		= -1,
};

static struct fb_deferred_io s5l_lcd_defio = {
	.delay		= HZ / 30,
	.deferred_io	= s5l_lcd_deferred_io,
};

static int s5l_lcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l_lcd_par *par;
	struct i2c_adapter *pmu;
	struct fb_info *info;
	void *vmem;
	u32 pwr;
	int ret;

	pmu = s5l_lcd_get_pmu(dev);
	if (IS_ERR(pmu))
		return PTR_ERR(pmu);

	info = framebuffer_alloc(sizeof(*par), dev);
	if (!info) {
		i2c_put_adapter(pmu);
		return -ENOMEM;
	}

	par = info->par;
	par->info = info;
	mutex_init(&par->io_lock);

	par->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(par->regs)) {
		ret = PTR_ERR(par->regs);
		goto err_release_info;
	}

	/* PWRCON0 LCD clock-gate.  Modelled standalone for now (mirrors
	 * what s5l8702_nand.c does for its own gates); once a proper
	 * clock controller driver exists this should consume a clock
	 * phandle instead.
	 */
	par->pwrcon0 = devm_ioremap(dev, S5L8702_PWRCON0_PHYS, 4);
	if (!par->pwrcon0) {
		ret = -ENOMEM;
		goto err_release_info;
	}
	pwr = readl(par->pwrcon0);
	writel(pwr & ~PWRCON0_LCD_BIT, par->pwrcon0);

	writel(0x33, par->regs + LCD_PHTIME);

	par->lcd_type = s5l_lcd_detect_panel(par);
	if (par->lcd_type < 0) {
		dev_err(dev, "no panel detected (id %02x %02x %02x %02x)\n",
			par->lcd_id[0], par->lcd_id[1],
			par->lcd_id[2], par->lcd_id[3]);
		ret = par->lcd_type;
		goto err_gate_off;
	}
	dev_info(dev, "panel id %02x %02x %02x %02x (type %d)\n",
		 par->lcd_id[0], par->lcd_id[1],
		 par->lcd_id[2], par->lcd_id[3], par->lcd_type);

	s5l_lcd_write_config(par, LCD_MODE_P8);
	s5l_lcd_run_seq(par, init_seq_by_type[par->lcd_type]);

	vmem = vzalloc(FB_SIZE);
	if (!vmem) {
		ret = -ENOMEM;
		goto err_gate_off;
	}

	info->screen_buffer = vmem;
	info->fbops = &s5l_lcd_fbops;
	info->var = s5l_lcd_var;
	info->fix = s5l_lcd_fix;
	info->fix.smem_len = FB_SIZE;
	info->flags = FBINFO_VIRTFB;
	info->pseudo_palette = par->pseudo_palette;
	info->fbdefio = &s5l_lcd_defio;

	fb_deferred_io_init(info);

	ret = register_framebuffer(info);
	if (ret < 0)
		goto err_defio_cleanup;

	platform_set_drvdata(pdev, info);

	s5l_lcd_blit(par);

	s5l_lcd_backlight_on(dev, pmu);
	i2c_put_adapter(pmu);

	dev_info(dev, "registered fb%d (%dx%d RGB565)\n",
		 info->node, DPY_W, DPY_H);
	return 0;

err_defio_cleanup:
	fb_deferred_io_cleanup(info);
	vfree(vmem);
err_gate_off:
	writel(readl(par->pwrcon0) | PWRCON0_LCD_BIT, par->pwrcon0);
err_release_info:
	framebuffer_release(info);
	i2c_put_adapter(pmu);
	return ret;
}

static void s5l_lcd_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct s5l_lcd_par *par = info->par;
	void *vmem = info->screen_buffer;

	unregister_framebuffer(info);
	fb_deferred_io_cleanup(info);
	writel(readl(par->pwrcon0) | PWRCON0_LCD_BIT, par->pwrcon0);
	vfree(vmem);
	framebuffer_release(info);
}

static const struct of_device_id s5l_lcd_of_match[] = {
	{ .compatible = "apple,s5l8702-lcd" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l_lcd_of_match);

static struct platform_driver s5l_lcd_driver = {
	.probe		= s5l_lcd_probe,
	.remove		= s5l_lcd_remove,
	.driver		= {
		.name		= "s5l8702-lcd",
		.of_match_table	= s5l_lcd_of_match,
	},
};
module_platform_driver(s5l_lcd_driver);

MODULE_DESCRIPTION("Apple S5L8702 LCD framebuffer (iPod nano 3G)");
MODULE_AUTHOR("Tucker Osman");
MODULE_LICENSE("GPL");
