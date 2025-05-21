// SPDX-License-Identifier: GPL-2.0-only

// Based on other drivers in drivers/gpu/drm/tiny
// as well as rockbox and freemyipod

#include <linux/clk.h>
#include <linux/of_clk.h>
#include <linux/minmax.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_framebuffer.h>


// TODO: Put these to dtb
#define LCD_WIDTH 176
#define LCD_HEIGHT 132

#define LCD_BASE (void*)0x38600000
#define LCDCON	(*((uint32_t volatile*)(0x38600000)))
#define LCDWCMD   (*((uint32_t volatile*)(0x38600004)))
#define LCDPHTIME (*((uint32_t volatile*)(0x38600010)))
#define LCDSTATUS (*((uint32_t volatile*)(0x3860001c)))
#define LCDWDATA  (*((uint32_t volatile*)(0x38600040)))
#define LCDCON_INITVALUE 0xd01

static int lcd_type = 0;

#define PCON13	   (*((uint32_t volatile*)(0x3CF000D0)))
#define PCON14	   (*((uint32_t volatile*)(0x3CF000E0)))
#define PDAT13	   (*((uint32_t volatile*)(0x3CF000D4)))
#define PDAT14	   (*((uint32_t volatile*)(0x3CF000E4)))

// Based on rockbox an freemyipod
static void lcd_init(void)
{
	PCON13 &= ~0xf;    /* Set pin 0 to input */
	PCON14 &= ~0xf0;   /* Set pin 1 to input */

	if (((PDAT13 & 1) == 0) && ((PDAT14 & 2) == 2)) {
	    lcd_type   = 0;     /* Similar to ILI9320 - aka "type 2" */
	    LCDCON   |= 0x180; /* use 16 bit bus width, big endian */
	} else {
	    lcd_type   = 1;     /* Similar to LDS176  - aka "type 7" */
	    LCDCON   |= 0x100; /* use 16 bit bus width, little endian */
	}

//	LCDCON = LCDCON_INITVALUE;
	//LCDPHTIME = 0; // this breaks the display???
}

static void lcd_send_cmd(uint32_t cmd)
{
	while (LCDSTATUS & 0x10);
	LCDWCMD = cmd;
}

static void lcd_send_data(uint32_t data)
{
	while (LCDSTATUS & 0x10);
	LCDWDATA = data;
}

static inline void lcd_send_cmd_data(int cmd, int data)
{
	while (LCDSTATUS & 0x10);
	LCDWCMD = cmd;

	while (LCDSTATUS & 0x10);
	LCDWDATA = data;
}

/* LCD type 0 register defines */

#define R_ENTRY_MODE	          0x03
#define R_DISPLAY_CONTROL_1	   0x07
#define R_POWER_CONTROL_1	     0x10
#define R_POWER_CONTROL_2	     0x12
#define R_POWER_CONTROL_3	     0x13
#define R_HORIZ_GRAM_ADDR_SET	 0x20
#define R_VERT_GRAM_ADDR_SET	  0x21
#define R_WRITE_DATA_TO_GRAM	  0x22
#define R_HORIZ_ADDR_START_POS	0x50
#define R_HORIZ_ADDR_END_POS	  0x51
#define R_VERT_ADDR_START_POS	 0x52
#define R_VERT_ADDR_END_POS	   0x53


/* LCD type 1 register defines */

#define R_SLEEP_IN	            0x10
#define R_DISPLAY_OFF	         0x28
#define R_COLUMN_ADDR_SET	     0x2a
#define R_ROW_ADDR_SET	        0x2b
#define R_MEMORY_WRITE	        0x2c

static void lcd_setup_drawing_region(int x, int y, int width, int height)
{
	int y0, x0, y1, x1;

	x0 = x;                         /* start horiz */
	y0 = y;                         /* start vert */
	x1 = (x + width) - 1;           /* max horiz */
	y1 = (y + height) - 1;          /* max vert */

	if (lcd_type==0) {
	    lcd_send_cmd_data(R_HORIZ_ADDR_START_POS, x0);
	    lcd_send_cmd_data(R_HORIZ_ADDR_END_POS,   x1);
	    lcd_send_cmd_data(R_VERT_ADDR_START_POS,  y0);
	    lcd_send_cmd_data(R_VERT_ADDR_END_POS,    y1);

	    lcd_send_cmd_data(R_HORIZ_GRAM_ADDR_SET,  (x1 << 8) | x0);
	    lcd_send_cmd_data(R_VERT_GRAM_ADDR_SET,   (y1 << 8) | y0);

	    lcd_send_cmd(0);
	    lcd_send_cmd(R_WRITE_DATA_TO_GRAM);
	} else {
	    lcd_send_cmd(R_COLUMN_ADDR_SET);
	    lcd_send_data(x0);            /* Start column */
	    lcd_send_data(x1);            /* End column */

	    lcd_send_cmd(R_ROW_ADDR_SET);
	    lcd_send_data(y0);            /* Start row */
	    lcd_send_data(y1);            /* End row */

	    lcd_send_cmd(R_MEMORY_WRITE);
	}
}

// TODO
void noinline lcd_write_line(void *data, int length, void *base) __attribute__((naked));
void lcd_write_line(void *data, int length, void *base)
{
	__asm__ volatile(";\
lcd_write_line_:	                   /* r2 = LCD_BASE */;\
	stmfd   sp!, {r4-r6, lr}          /* save non-scratch registers */;\
	add     r12, r2, #0x40            /* LCD_WDATA = LCD data port */;\
;\
.loop:\
	ldmia r0!, {r3, r5}               /* read 4 pixels (=8 byte) */;\
;\
	/* wait for FIFO half full */;\
.fifo_wait:\
	ldr     lr, [r2, #0x1C]           /* while (LCD_STATUS & 0x08); */;\
	tst     lr, #0x8;\
	bgt     .fifo_wait;\
;\
	mov     r4, r3, asr #16           /* r3 = 1st pixel, r4 = 2nd pixel */;\
	mov     r6, r5, asr #16           /* r5 = 3rd pixel, r6 = 4th pixel */;\
	stmia   r12, {r3-r6}              /* write pixels (lowest 16 bit used) */;\
;\
	subs    r1, r1, #4;\
	bgt     .loop;\
;\
	ldmfd   sp!, {r4-r6, pc};\
	");
}

void lcd_update_rect(int x, int y, int width, int height, void *data, int data_stride)
{
	/* Both x and width need to be preprocessed due to asm optimizations */
	x     = x & ~1;                 /* ensure x is even */
	width = (width + 3) & ~3;       /* ensure width is a multiple of 4 */

	data += y * data_stride + x;

	lcd_setup_drawing_region(x, y, width, height);

	/* Copy display bitmap to hardware */
	if (LCD_WIDTH == width) {
	    /* Write all lines at once */
	    lcd_write_line(data, height*LCD_WIDTH, LCD_BASE);
	} else {
	    do {
	        /* Write a single line */
	        lcd_write_line(data, width, LCD_BASE);
	        data += data_stride;
	    } while (--height > 0 );
	}
}

#define DRIVER_NAME	"ili9320drm"
#define DRIVER_DESC	"DRM driver for ili9320-framebuffer platform devices"
#define DRIVER_DATE	"20200625"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

/*
 * Simple Framebuffer device
 */

struct ili9320drm_device {
	struct drm_device dev;

	/* clocks */
#if defined CONFIG_OF && defined CONFIG_COMMON_CLK
	unsigned int clk_count;
	struct clk **clks;
#endif
	/* regulators */
#if defined CONFIG_OF && defined CONFIG_REGULATOR
	unsigned int regulator_count;
	struct regulator **regulators;
#endif

	/* ili9320fb settings */
	struct drm_display_mode mode;
	const struct drm_format_info *format;
	unsigned int pitch;

	/* modesetting */
	uint32_t formats[8];
	struct drm_connector	conn;
	struct drm_simple_display_pipe   pipe;
};

static struct ili9320drm_device *ili9320drm_device_of_dev(struct drm_device *dev)
{
	return container_of(dev, struct ili9320drm_device, dev);
}

/*
 * Hardware
 */

#if defined CONFIG_OF && defined CONFIG_COMMON_CLK
/*
 * Clock handling code.
 *
 * Here we handle the clocks property of our "ili9320-framebuffer" dt node.
 * This is necessary so that we can make sure that any clocks needed by
 * the display engine that the bootloader set up for us (and for which it
 * provided a ili9320fb dt node), stay up, for the life of the ili9320fb
 * driver.
 *
 * When the driver unloads, we cleanly disable, and then release the clocks.
 *
 * We only complain about errors here, no action is taken as the most likely
 * error can only happen due to a mismatch between the bootloader which set
 * up ili9320fb, and the clock definitions in the device tree. Chances are
 * that there are no adverse effects, and if there are, a clean teardown of
 * the fb probe will not help us much either. So just complain and carry on,
 * and hope that the user actually gets a working fb at the end of things.
 */

static void ili9320drm_device_release_clocks(void *res)
{
	struct ili9320drm_device *sdev = ili9320drm_device_of_dev(res);
	unsigned int i;

	for (i = 0; i < sdev->clk_count; ++i) {
		if (sdev->clks[i]) {
			clk_disable_unprepare(sdev->clks[i]);
			clk_put(sdev->clks[i]);
		}
	}
}

static int ili9320drm_device_init_clocks(struct ili9320drm_device *sdev)
{
	struct drm_device *dev = &sdev->dev;
	struct platform_device *pdev = to_platform_device(dev->dev);
	struct device_node *of_node = pdev->dev.of_node;
	struct clk *clock;
	unsigned int i;
	int ret;

	if (dev_get_platdata(&pdev->dev) || !of_node)
		return 0;

	sdev->clk_count = of_clk_get_parent_count(of_node);
	if (!sdev->clk_count)
		return 0;

	sdev->clks = drmm_kzalloc(dev, sdev->clk_count * sizeof(sdev->clks[0]),
				  GFP_KERNEL);
	if (!sdev->clks)
		return -ENOMEM;

	for (i = 0; i < sdev->clk_count; ++i) {
		clock = of_clk_get(of_node, i);
		if (IS_ERR(clock)) {
			ret = PTR_ERR(clock);
			if (ret == -EPROBE_DEFER)
				goto err;
			drm_err(dev, "clock %u not found: %d\n", i, ret);
			continue;
		}
		ret = clk_prepare_enable(clock);
		if (ret) {
			drm_err(dev, "failed to enable clock %u: %d\n",
				i, ret);
			clk_put(clock);
			continue;
		}
		sdev->clks[i] = clock;
	}

	return devm_add_action_or_reset(&pdev->dev,
					ili9320drm_device_release_clocks,
					sdev);

err:
	while (i) {
		--i;
		if (sdev->clks[i]) {
			clk_disable_unprepare(sdev->clks[i]);
			clk_put(sdev->clks[i]);
		}
	}
	return ret;
}
#else
static int ili9320drm_device_init_clocks(struct ili9320drm_device *sdev)
{
	return 0;
}
#endif

#if defined CONFIG_OF && defined CONFIG_REGULATOR

#define SUPPLY_SUFFIX "-supply"

/*
 * Regulator handling code.
 *
 * Here we handle the num-supplies and vin*-supply properties of our
 * "ili9320-framebuffer" dt node. This is necessary so that we can make sure
 * that any regulators needed by the display hardware that the bootloader
 * set up for us (and for which it provided a ili9320fb dt node), stay up,
 * for the life of the ili9320fb driver.
 *
 * When the driver unloads, we cleanly disable, and then release the
 * regulators.
 *
 * We only complain about errors here, no action is taken as the most likely
 * error can only happen due to a mismatch between the bootloader which set
 * up ili9320fb, and the regulator definitions in the device tree. Chances are
 * that there are no adverse effects, and if there are, a clean teardown of
 * the fb probe will not help us much either. So just complain and carry on,
 * and hope that the user actually gets a working fb at the end of things.
 */

static void ili9320drm_device_release_regulators(void *res)
{
	struct ili9320drm_device *sdev = ili9320drm_device_of_dev(res);
	unsigned int i;

	for (i = 0; i < sdev->regulator_count; ++i) {
		if (sdev->regulators[i]) {
			regulator_disable(sdev->regulators[i]);
			regulator_put(sdev->regulators[i]);
		}
	}
}

static int ili9320drm_device_init_regulators(struct ili9320drm_device *sdev)
{
	struct drm_device *dev = &sdev->dev;
	struct platform_device *pdev = to_platform_device(dev->dev);
	struct device_node *of_node = pdev->dev.of_node;
	struct property *prop;
	struct regulator *regulator;
	const char *p;
	unsigned int count = 0, i = 0;
	int ret;

	if (dev_get_platdata(&pdev->dev) || !of_node)
		return 0;

	/* Count the number of regulator supplies */
	for_each_property_of_node(of_node, prop) {
		p = strstr(prop->name, SUPPLY_SUFFIX);
		if (p && p != prop->name)
			++count;
	}

	if (!count)
		return 0;

	sdev->regulators = drmm_kzalloc(dev,
					count * sizeof(sdev->regulators[0]),
					GFP_KERNEL);
	if (!sdev->regulators)
		return -ENOMEM;

	for_each_property_of_node(of_node, prop) {
		char name[32]; /* 32 is max size of property name */
		size_t len;

		p = strstr(prop->name, SUPPLY_SUFFIX);
		if (!p || p == prop->name)
			continue;
		len = strlen(prop->name) - strlen(SUPPLY_SUFFIX) + 1;
		strscpy(name, prop->name, min(sizeof(name), len));

		regulator = regulator_get_optional(&pdev->dev, name);
		if (IS_ERR(regulator)) {
			ret = PTR_ERR(regulator);
			if (ret == -EPROBE_DEFER)
				goto err;
			drm_err(dev, "regulator %s not found: %d\n",
				name, ret);
			continue;
		}

		ret = regulator_enable(regulator);
		if (ret) {
			drm_err(dev, "failed to enable regulator %u: %d\n",
				i, ret);
			regulator_put(regulator);
			continue;
		}

		sdev->regulators[i++] = regulator;
	}
	sdev->regulator_count = i;

	return devm_add_action_or_reset(&pdev->dev,
					ili9320drm_device_release_regulators,
					sdev);

err:
	while (i) {
		--i;
		if (sdev->regulators[i]) {
			regulator_disable(sdev->regulators[i]);
			regulator_put(sdev->regulators[i]);
		}
	}
	return ret;
}
#else
static int ili9320drm_device_init_regulators(struct ili9320drm_device *sdev)
{
	return 0;
}
#endif

static const uint32_t ili9320drm_pipe_formats[] = {
	DRM_FORMAT_RGB565,
};

static const uint64_t ili9320drm_pipe_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static void ili9320drm_pipe_enable(struct drm_simple_display_pipe *pipe,
				 struct drm_crtc_state *crtc_state,
				 struct drm_plane_state *plane_state)
{
}

static void ili9320drm_pipe_disable(struct drm_simple_display_pipe *pipe)
{
}

static void ili9320drm_pipe_update(struct drm_simple_display_pipe *pipe,
				 struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;

	struct drm_rect rect;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect)) {
	    if (state->fb->format->format == DRM_FORMAT_RGB565) {
	        struct drm_gem_dma_object *dma_obj = drm_fb_dma_get_gem_obj(state->fb, 0);
	        lcd_update_rect(0, 0, state->fb->width, state->fb->height, dma_obj->vaddr, state->fb->pitches[0]);
	    }
	}
}

static const struct drm_simple_display_pipe_funcs ili9320drm_pipe_funcs = {
	.enable		= ili9320drm_pipe_enable,
	.disable	= ili9320drm_pipe_disable,
	.update		= ili9320drm_pipe_update,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static int ili9320drm_connector_helper_get_modes(struct drm_connector *connector)
{
	struct ili9320drm_device *sdev = ili9320drm_device_of_dev(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, &sdev->mode);
}

static const struct drm_connector_helper_funcs ili9320drm_connector_helper_funcs = {
	.get_modes = ili9320drm_connector_helper_get_modes,
};

static const struct drm_connector_funcs ili9320drm_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_framebuffer*
ili9320_fb_create(struct drm_device *dev, struct drm_file *file_priv,
	     const struct drm_mode_fb_cmd2 *mode_cmd)
{
	return drm_gem_fb_create_with_dirty(dev, file_priv, mode_cmd);
}

static const struct drm_mode_config_funcs ili9320drm_mode_config_funcs = {
	.fb_create = ili9320_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/*
 * Init / Cleanup
 */

static struct drm_display_mode ili9320drm_mode(unsigned int width,
						  unsigned int height)
{
	// TODO
	const struct drm_display_mode mode = {
		DRM_MODE_INIT(60, width, height,
				  DRM_MODE_RES_MM(width, 40ul),
				  DRM_MODE_RES_MM(height, 40ul))
	};

	return mode;
}

static struct ili9320drm_device *ili9320drm_device_create(struct drm_driver *drv,
							struct platform_device *pdev)
{
	struct ili9320drm_device *sdev;
	struct drm_device *dev;
	int width, height, stride;
	const struct drm_format_info *format;
	struct resource *res, *mem;
	unsigned long max_width, max_height;
	int ret;

	sdev = devm_drm_dev_alloc(&pdev->dev, drv, struct ili9320drm_device, dev);
	if (IS_ERR(sdev))
		return ERR_CAST(sdev);
	dev = &sdev->dev;
	platform_set_drvdata(pdev, sdev);

	/*
	 * Hardware settings
	 */

	ret = ili9320drm_device_init_clocks(sdev);
	if (ret)
		return ERR_PTR(ret);
	ret = ili9320drm_device_init_regulators(sdev);
	if (ret)
		return ERR_PTR(ret);

	width = LCD_WIDTH;
	height = LCD_HEIGHT;
	stride = LCD_WIDTH;
	format = drm_format_info(DRM_FORMAT_RGB565);

	sdev->mode = ili9320drm_mode(width, height);
	sdev->format = format;
	sdev->pitch = stride;

	drm_dbg(dev, "display mode={" DRM_MODE_FMT "}\n", DRM_MODE_ARG(&sdev->mode));
	drm_dbg(dev, "framebuffer format=%p4cc, size=%dx%d, stride=%d byte\n",
		&format->format, width, height, stride);

	/*
	 * Memory management
	 */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return ERR_PTR(-EINVAL);

	ret = devm_aperture_acquire_from_firmware(dev, res->start, resource_size(res));
	if (ret) {
		drm_err(dev, "could not acquire memory range %pr: error %d\n", res, ret);
		return ERR_PTR(ret);
	}

	mem = devm_request_mem_region(&pdev->dev, res->start, resource_size(res), drv->name);
	if (!mem) {
		/*
		 * We cannot make this fatal. Sometimes this comes from magic
		 * spaces our resource handlers simply don't know about. Use
		 * the I/O-memory resource as-is and try to map that instead.
		 */
		drm_warn(dev, "could not acquire memory region %pr\n", res);
		mem = res;
	}

	/*
	 * Modesetting
	 */

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ERR_PTR(ret);

	max_width = max_t(unsigned long, width, DRM_SHADOW_PLANE_MAX_WIDTH);
	max_height = max_t(unsigned long, height, DRM_SHADOW_PLANE_MAX_HEIGHT);

	dev->mode_config.min_width = width;
	dev->mode_config.max_width = max_width;
	dev->mode_config.min_height = height;
	dev->mode_config.max_height = max_height;
	dev->mode_config.preferred_depth = 16;
	dev->mode_config.prefer_shadow_fbdev = 1;
	dev->mode_config.funcs = &ili9320drm_mode_config_funcs;

	drm_connector_helper_add(&sdev->conn, &ili9320drm_connector_helper_funcs);
	ret = drm_connector_init(&sdev->dev, &sdev->conn,
				  &ili9320drm_connector_funcs, DRM_MODE_CONNECTOR_Unknown);
	if (ret)
		return ERR_PTR(ret);

	ret = drm_simple_display_pipe_init(&sdev->dev,
	                   &sdev->pipe,
	                   &ili9320drm_pipe_funcs,
	                   ili9320drm_pipe_formats,
	                   ARRAY_SIZE(ili9320drm_pipe_formats),
	                   ili9320drm_pipe_format_modifiers,
	                   &sdev->conn);

	drm_mode_config_reset(dev);

	return sdev;
}

// These are based on drivers/gpu/drm/drm_gem_dma_helper.c
static const struct drm_gem_object_funcs drm_gem_dma_default_funcs = {
	.free = drm_gem_dma_object_free,
	.print_info = drm_gem_dma_object_print_info,
	.get_sg_table = drm_gem_dma_object_get_sg_table,
	.vmap = drm_gem_dma_object_vmap,
	.mmap = drm_gem_dma_object_mmap,
	.vm_ops = &drm_gem_dma_vm_ops,
};

static struct drm_gem_dma_object *
ili9320__drm_gem_dma_create(struct drm_device *drm, size_t size, bool private)
{
	struct drm_gem_dma_object *dma_obj;
	struct drm_gem_object *gem_obj;
	int ret = 0;

	if (drm->driver->gem_create_object) {
		gem_obj = drm->driver->gem_create_object(drm, size);
		if (IS_ERR(gem_obj))
			return ERR_CAST(gem_obj);
		dma_obj = to_drm_gem_dma_obj(gem_obj);
	} else {
		dma_obj = kzalloc(sizeof(*dma_obj), GFP_KERNEL);
		if (!dma_obj)
			return ERR_PTR(-ENOMEM);
		gem_obj = &dma_obj->base;
	}

	if (!gem_obj->funcs)
		gem_obj->funcs = &drm_gem_dma_default_funcs;

	if (private) {
		drm_gem_private_object_init(drm, gem_obj, size);

		/* Always use writecombine for dma-buf mappings */
		dma_obj->map_noncoherent = false;
	} else {
		ret = drm_gem_object_init(drm, gem_obj, size);
	}
	if (ret)
		goto error;

	ret = drm_gem_create_mmap_offset(gem_obj);
	if (ret) {
		drm_gem_object_release(gem_obj);
		goto error;
	}

	return dma_obj;

error:
	kfree(dma_obj);
	return ERR_PTR(ret);
}
struct drm_gem_dma_object *ili9320_drm_gem_dma_create(struct drm_device *drm,
						  size_t size)
{
	struct drm_gem_dma_object *dma_obj;
	int ret;

	size = round_up(size, PAGE_SIZE);

	dma_obj = ili9320__drm_gem_dma_create(drm, size, false);
	if (IS_ERR(dma_obj))
		return dma_obj;

	// s5l8700: no dma support yet (TODO)
	if (dma_obj->map_noncoherent) {
		dma_obj->vaddr = kzalloc(size, GFP_KERNEL);
#if 0
		dma_obj->vaddr = dma_alloc_noncoherent(drm->dev, size,
							   &dma_obj->dma_addr,
							   DMA_TO_DEVICE,
							   GFP_KERNEL | __GFP_NOWARN);
#endif
	} else {
		dma_obj->vaddr = kzalloc(size, GFP_KERNEL);
#if 0
		dma_obj->vaddr = dma_alloc_wc(drm->dev, size,
						  &dma_obj->dma_addr,
						  GFP_KERNEL | __GFP_NOWARN);
#endif
	}
	if (!dma_obj->vaddr) {
		drm_dbg(drm, "failed to allocate buffer with size %zu\n",
			 size);
		ret = -ENOMEM;
		goto error;
	}

	return dma_obj;

error:
	drm_gem_object_put(&dma_obj->base);
	return ERR_PTR(ret);
}

static struct drm_gem_dma_object *
ili9320_drm_gem_dma_create_with_handle(struct drm_file *file_priv,
				   struct drm_device *drm, size_t size,
				   uint32_t *handle)
{
	struct drm_gem_dma_object *dma_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	dma_obj = ili9320_drm_gem_dma_create(drm, size);
	if (IS_ERR(dma_obj))
		return dma_obj;

	gem_obj = &dma_obj->base;

	/*
	 * allocate a id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file_priv, gem_obj, handle);
	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_put(gem_obj);
	if (ret)
		return ERR_PTR(ret);

	return dma_obj;
}

int ili9320_drm_gem_dma_dumb_create(struct drm_file *file_priv,
				struct drm_device *drm,
				struct drm_mode_create_dumb *args)
{
	struct drm_gem_dma_object *dma_obj;

	args->pitch = DIV_ROUND_UP(args->width * args->bpp, 8);
	args->size = args->pitch * args->height;

	dma_obj = ili9320_drm_gem_dma_create_with_handle(file_priv, drm, args->size,
						 &args->handle);
	return PTR_ERR_OR_ZERO(dma_obj);
}

/*
 * DRM driver
 */
DEFINE_DRM_GEM_DMA_FOPS(ili9320drm_fops);

static struct drm_driver ili9320drm_driver = {
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(ili9320_drm_gem_dma_dumb_create),
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
	.driver_features	= DRIVER_ATOMIC | DRIVER_GEM | DRIVER_MODESET,
	.fops			= &ili9320drm_fops,
};

/*
 * Platform driver
 */

static int ili9320drm_probe(struct platform_device *pdev)
{
	struct ili9320drm_device *sdev;
	struct drm_device *dev;
	int ret;

	sdev = ili9320drm_device_create(&ili9320drm_driver, pdev);
	if (IS_ERR(sdev))
		return PTR_ERR(sdev);
	dev = &sdev->dev;

	ret = drm_dev_register(dev, 0);
	if (ret)
		return ret;

	lcd_init();
	drm_fbdev_generic_setup(dev, 16);

	return 0;
}

static int ili9320drm_remove(struct platform_device *pdev)
{
	struct ili9320drm_device *sdev = platform_get_drvdata(pdev);
	struct drm_device *dev = &sdev->dev;

	drm_dev_unplug(dev);

	return 0;
}

static const struct of_device_id ili9320drm_of_match_table[] = {
	{ .compatible = "samsung,s5l8730-lcdcon" },
	{ },
};
MODULE_DEVICE_TABLE(of, ili9320drm_of_match_table);

static struct platform_driver ili9320drm_platform_driver = {
	.driver = {
		.name = "samsung,s5l8730-lcdcon", /* connect to sysfb */
		.of_match_table = ili9320drm_of_match_table,
	},
	.probe = ili9320drm_probe,
	.remove = ili9320drm_remove,
};

module_platform_driver(ili9320drm_platform_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
