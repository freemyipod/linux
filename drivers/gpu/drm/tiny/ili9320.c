// SPDX-License-Identifier: GPL-2.0-only

// Based on other drivers in drivers/gpu/drm/tiny
// as well as rockbox and freemyipod

#include <linux/clk.h>
#include <linux/of_clk.h>
#include <linux/minmax.h>
#include <linux/platform_device.h>

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
#include <drm/drm_client.h>
#include <drm/drm_fb_helper.h>
#include <linux/delay.h>

#define CLK_BASE 0x3C500000
#define PWRCONEXT               (*(volatile uint32_t*)(CLK_BASE + 0x40))

#define DMABASE8     (*((void* volatile*)(0x38400100)))
#define DMACON8      (*((volatile uint32_t*)(0x38400104)))
#define DMATCNT8     (*((volatile uint32_t*)(0x38400108)))
#define DMACADDR8    (*((void* volatile*)(0x3840010C)))
#define DMACTCNT8    (*((volatile uint32_t*)(0x38400110)))
#define DMACOM8      (*((volatile uint32_t*)(0x38400114)))

#define DMAALLST     (*((volatile uint32_t*)(0x38400180)))
#define DMAALLST2    (*((volatile uint32_t*)(0x38400184)))


// TODO: Put these to dtb
#define LCD_WIDTH 176
#define LCD_HEIGHT 132

#define LCD_BASE (void*)0x38600000
#define LCD_RST_TIME            (*(volatile uint32_t*)(LCD_BASE + 0x24))  /* Reset active period 07FF */
#define LCD_DRV_RST             (*(volatile uint32_t*)(LCD_BASE + 0x28))  /* Reset drive signal */
#define LCDCON	(*((volatile uint32_t*)(0x38600000)))
#define LCDWCMD   (*((volatile uint32_t*)(0x38600004)))
#define LCDPHTIME (*((volatile uint32_t*)(0x38600010)))
#define LCDSTATUS (*((volatile uint32_t*)(0x3860001c)))
#define LCDWDATA  (*((volatile uint32_t*)(0x38600040)))
#define LCDCON_CMDMODEVALUE 0xd01
#define LCDCON_SETUPVALUE 0x2d50
#define LCDCON_FRAMEMODEVALUE 0xe10

#define PCON13	   (*((volatile uint32_t*)(0x3CF000D0)))
#define PCON14	   (*((volatile uint32_t*)(0x3CF000E0)))
#define PDAT13	   (*((volatile uint32_t*)(0x3CF000D4)))
#define PDAT14	   (*((volatile uint32_t*)(0x3CF000E4)))

/* LCD type 0 register defines */

#define R_ENTRY_MODE              0x03
#define R_DISPLAY_CONTROL_1       0x07
#define R_POWER_CONTROL_1         0x10
#define R_POWER_CONTROL_2         0x12
#define R_POWER_CONTROL_3         0x13
#define R_HORIZ_GRAM_ADDR_SET     0x20
#define R_VERT_GRAM_ADDR_SET      0x21
#define R_WRITE_DATA_TO_GRAM      0x22
#define R_HORIZ_ADDR_START_POS    0x50
#define R_HORIZ_ADDR_END_POS      0x51
#define R_VERT_ADDR_START_POS     0x52
#define R_VERT_ADDR_END_POS       0x53


/* LCD type 1 register defines */

#define R_SLEEP_IN                0x10
#define R_DISPLAY_OFF             0x28
#define R_COLUMN_ADDR_SET         0x2a
#define R_ROW_ADDR_SET            0x2b
#define R_MEMORY_WRITE            0x2c


static bool lcd_dma_busy = false;

#define SLEEP   0
#define CMD16   1
#define DATA16  2

unsigned short lcd_init_sequence_0[] = {
    CMD16,  0x00a4, DATA16, 0x0001,
    SLEEP,  0x0000,
    CMD16,  0x0001, DATA16, 0x0100,
    CMD16,  0x0002, DATA16, 0x0300,
    CMD16,  0x0003, DATA16, 0x1230,
    CMD16,  0x0008, DATA16, 0x0404,
    CMD16,  0x0008, DATA16, 0x0404,
    CMD16,  0x000e, DATA16, 0x0010,
    CMD16,  0x0070, DATA16, 0x1000,
    CMD16,  0x0071, DATA16, 0x0001,
    CMD16,  0x0030, DATA16, 0x0002,
    CMD16,  0x0031, DATA16, 0x0400,
    CMD16,  0x0032, DATA16, 0x0007,
    CMD16,  0x0033, DATA16, 0x0500,
    CMD16,  0x0034, DATA16, 0x0007,
    CMD16,  0x0035, DATA16, 0x0703,
    CMD16,  0x0036, DATA16, 0x0507,
    CMD16,  0x0037, DATA16, 0x0005,
    CMD16,  0x0038, DATA16, 0x0407,
    CMD16,  0x0039, DATA16, 0x000e,
    CMD16,  0x0040, DATA16, 0x0202,
    CMD16,  0x0041, DATA16, 0x0003,
    CMD16,  0x0042, DATA16, 0x0000,
    CMD16,  0x0043, DATA16, 0x0200,
    CMD16,  0x0044, DATA16, 0x0707,
    CMD16,  0x0045, DATA16, 0x0407,
    CMD16,  0x0046, DATA16, 0x0505,
    CMD16,  0x0047, DATA16, 0x0002,
    CMD16,  0x0048, DATA16, 0x0004,
    CMD16,  0x0049, DATA16, 0x0004,
    CMD16,  0x0060, DATA16, 0x0202,
    CMD16,  0x0061, DATA16, 0x0003,
    CMD16,  0x0062, DATA16, 0x0000,
    CMD16,  0x0063, DATA16, 0x0200,
    CMD16,  0x0064, DATA16, 0x0707,
    CMD16,  0x0065, DATA16, 0x0407,
    CMD16,  0x0066, DATA16, 0x0505,
    CMD16,  0x0068, DATA16, 0x0004,
    CMD16,  0x0069, DATA16, 0x0004,
    CMD16,  0x0007, DATA16, 0x0001,
    CMD16,  0x0018, DATA16, 0x0001,
    CMD16,  0x0010, DATA16, 0x1690,
    CMD16,  0x0011, DATA16, 0x0100,
    CMD16,  0x0012, DATA16, 0x0117,
    CMD16,  0x0013, DATA16, 0x0f80,
    CMD16,  0x0012, DATA16, 0x0137,
    CMD16,  0x0020, DATA16, 0x0000,
    CMD16,  0x0021, DATA16, 0x0000,
    CMD16,  0x0050, DATA16, 0x0000,
    CMD16,  0x0051, DATA16, 0x00af,
    CMD16,  0x0052, DATA16, 0x0000,
    CMD16,  0x0053, DATA16, 0x0083,
    CMD16,  0x0090, DATA16, 0x0003,
    CMD16,  0x0091, DATA16, 0x0000,
    CMD16,  0x0092, DATA16, 0x0101,
    CMD16,  0x0098, DATA16, 0x0400,
    CMD16,  0x0099, DATA16, 0x1302,
    CMD16,  0x009a, DATA16, 0x0202,
    CMD16,  0x009b, DATA16, 0x0200,
    SLEEP,  0x0000,
    CMD16,  0x0007, DATA16, 0x0021,
    CMD16,  0x0012, DATA16, 0x0137,
    SLEEP,  0x0000,
    CMD16,  0x0007, DATA16, 0x0021,
    CMD16,  0x0012, DATA16, 0x1137,
    SLEEP,  0x0000,
    CMD16,  0x0007, DATA16, 0x0233,
};

unsigned short lcd_init_sequence_1[] = {
    CMD16,  0x0011, DATA16, 0x0000,
    CMD16,  0x0029, DATA16, 0x0000,
    SLEEP,  0x0000,
};

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

static void lcd_send_cmd_data(uint32_t cmd, uint32_t data)
{
    while (LCDSTATUS & 0x10);
    LCDWCMD = cmd;

    while (LCDSTATUS & 0x10);
    LCDWDATA = data;
}

static int lcd_type;


// Based on rockbox an freemyipod
static void lcd_init(void)
{
    unsigned short *lcd_init_sequence;
    unsigned int lcd_init_sequence_length;

	PCON13 &= ~0xf;    /* Set pin 0 to input */
	PCON14 &= ~0xf0;   /* Set pin 1 to input */

    if (((PDAT13 & 1) == 0) && ((PDAT14 & 2) == 2)) {
        lcd_type   = 0;     /* Similar to ILI9320 - aka "type 2" */
        //LCDCON   |= 0x180; /* use 16 bit bus width, big endian */
    } else {
        lcd_type   = 1;     /* Similar to LDS176  - aka "type 7" */
        //LCDCON   |= 0x100; /* use 16 bit bus width, little endian */
    }

//    DMACON8 = 0x20000000 | 0x180000 | (1 << 16);
//    DMACON8 = 0x30890003; for 888
//    DMACON8 = 0x30190006;//for 888
//    DMACON8 = 0x33c90004; // for 8888
//    DMACON8 = 0x30590008; // for 8888
    DMACON8 = 0x20190000;

    if (lcd_type == 0) {
        LCDCON = LCDCON_SETUPVALUE | 0x80;
    } else {
        LCDCON = LCDCON_SETUPVALUE;
    }
    DMATCNT8 = (LCD_WIDTH * LCD_HEIGHT / 2) - 1;

    LCDPHTIME = 0x0;
    lcd_dma_busy = false;
}

static uint32_t lcd_detect(void)
{
    return (PDAT13 & 1) | (PDAT14 & 2);
}

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

void displaylcd_setup(unsigned int startx, unsigned int endx,
                      unsigned int starty, unsigned int endy, bool safe)
{
    while (DMAALLST2 & 0x40000);
    while (!(LCDSTATUS & 0x2));
    LCDCON = LCDCON_CMDMODEVALUE | ((lcd_type == 0) ? 0x80 : 0);
    lcd_setup_drawing_region(startx, starty, endx + 1, endy + 1);
    while (!(LCDSTATUS & 0x2));
    LCDCON = LCDCON_FRAMEMODEVALUE | ((lcd_type == 0) ? 0x80 : 0);
}

void noinline clean_dcache(void) __attribute__((naked));
void clean_dcache(void)
{
    asm volatile(
        "MOV R0, #0                \n\t"
        "clean_dcache_loop2:        \n\t"
        "MCR p15, 0, R0,c7,c10,2   \n\t"
        "ADD R1, R0, #0x10         \n\t"
        "MCR p15, 0, R1,c7,c10,2   \n\t"
        "ADD R1, R1, #0x10         \n\t"
        "MCR p15, 0, R1,c7,c10,2   \n\t"
        "ADD R1, R1, #0x10         \n\t"
        "MCR p15, 0, R1,c7,c10,2   \n\t"
        "ADDS R0, R0, #0x04000000  \n\t"
        "BNE clean_dcache_loop2     \n\t"
        "MCR p15, 0, R0,c7,c10,4   \n\t"
        "MOV PC, LR                \n\t"
    );
}

static void displaylcd_dma(void* data, int pixels)
{
    uint16_t* in = (uint16_t*)data;
    while (LCDSTATUS & 8);
    if (!pixels) return;
    lcd_dma_busy = true;
    DMABASE8 = in;
    clean_dcache();
    DMACOM8 = 4;
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

	/* ili9320fb settings */
	struct drm_display_mode mode;
	const struct drm_format_info *format;
	unsigned int pitch;

	/* modesetting */
	uint32_t formats[8];
	struct drm_connector	conn;
	struct drm_simple_display_pipe   pipe;
    int irq;
    void *ptr;
};

static struct ili9320drm_device *ili9320drm_device_of_dev(struct drm_device *dev)
{
	return container_of(dev, struct ili9320drm_device, dev);
}

/*
 * Hardware
 */

#define FORMAT DRM_FORMAT_RGB565
#define FORMAT_BPP 2
#define FORMAT_DEPTH 16

static const uint32_t ili9320drm_pipe_formats[] = {
	FORMAT,
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
	struct ili9320drm_device *sdev = ili9320drm_device_of_dev(pipe->crtc.dev);

	struct drm_rect rect;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect)) {
	    if (state->fb->format->format == FORMAT) {
	        struct drm_gem_dma_object *dma_obj = drm_fb_dma_get_gem_obj(state->fb, 0);
	        //lcd_update_rect(0, 0, state->fb->width, state->fb->height, dma_obj->vaddr, state->fb->pitches[0]);
//			local_irq_disable();
            if (!sdev->ptr) {
		        drm_warn(&sdev->dev, "Initializing lcd, pitch: %d\n", state->fb->pitches[0]);
                lcd_init();
                displaylcd_setup(0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1, true);
                displaylcd_dma(dma_obj->vaddr, LCD_WIDTH * LCD_HEIGHT);
            } else if (sdev->ptr != dma_obj->vaddr) {
		        drm_warn(&sdev->dev, "unequal...\n");
//                DMACON8 = 0x20000000 | 0x180000 | (1 << 16);

                sdev->ptr = 0;
                while (lcd_dma_busy) {
		            drm_warn(&sdev->dev, "Waiting for dma...\n");
                    mdelay(1);
                }
                //DMACOM8 = 7;
                displaylcd_setup(0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1, true);
                displaylcd_dma(dma_obj->vaddr, LCD_WIDTH * LCD_HEIGHT);
                sdev->ptr = dma_obj->vaddr;
            }
            sdev->ptr = dma_obj->vaddr;
//			local_irq_enable();
	    } else {
		    drm_warn(&sdev->dev, "unknown format %x\n", state->fb->format->format);
        }
	} else {
        drm_warn(&sdev->dev, "no damage?\n");
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
	return drm_gem_fb_create(dev, file_priv, mode_cmd);
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
				  DRM_MODE_RES_MM(width, 400ul),
				  DRM_MODE_RES_MM(height, 400ul))
	};

	return mode;
}

static irqreturn_t dma_irq(int irq, void *pw)
{
    uint32_t dmaallst = DMAALLST;
    uint32_t dmaallst2 = DMAALLST2;

	struct ili9320drm_device *sdev = pw;

    if (dmaallst2 & DMACON8 & 0x30000) {
        DMACOM8 = 7;
        lcd_dma_busy = false;
        if (sdev->ptr) {
            displaylcd_setup(0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1, true);
            displaylcd_dma(sdev->ptr, LCD_WIDTH * LCD_HEIGHT);
        }
    }
    return IRQ_HANDLED;
}

static struct ili9320drm_device *ili9320drm_device_create(struct drm_driver *drv,
							struct platform_device *pdev)
{
	struct ili9320drm_device *sdev;
	struct drm_device *dev;
	int width, height, stride;
	const struct drm_format_info *format;
	unsigned long max_width, max_height;
	int ret;

	sdev = devm_drm_dev_alloc(&pdev->dev, drv, struct ili9320drm_device, dev);
	if (IS_ERR(sdev))
		return ERR_CAST(sdev);
	dev = &sdev->dev;
	platform_set_drvdata(pdev, sdev);

    sdev->ptr = 0;

	width = LCD_WIDTH;
	height = LCD_HEIGHT;
	stride = LCD_WIDTH * FORMAT_BPP;
	format = drm_format_info(FORMAT);

	sdev->mode = ili9320drm_mode(width, height);
	sdev->format = format;
	sdev->pitch = stride;

	drm_dbg(dev, "display mode={" DRM_MODE_FMT "}\n", DRM_MODE_ARG(&sdev->mode));
	drm_dbg(dev, "framebuffer format=%p4cc, size=%dx%d, stride=%d byte\n",
		&format->format, width, height, stride);

	/*
	 * Memory management
	 */

	sdev->irq = platform_get_irq(pdev, 0);
	if (sdev->irq < 0)
		return ERR_PTR(sdev->irq);

	ret = devm_request_irq(&pdev->dev, sdev->irq, dma_irq,
			       IRQF_SHARED, dev_name(&pdev->dev), sdev);
	if (ret < 0) {
		drm_err(dev, "cannot claim IRQ for lcd dma\n");
		return ERR_PTR(ret);
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
	dev->mode_config.preferred_depth = FORMAT_DEPTH;
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
    drm_plane_enable_fb_damage_clips(&sdev->pipe.plane);

	drm_mode_config_reset(dev);

	return sdev;
}

// These are based on drivers/gpu/drm/drm_gem_dma_helper.c
int ili9320_drm_gem_dma_mmap(struct drm_gem_dma_object *dma_obj, struct vm_area_struct *vma)
{
    struct drm_gem_object *obj = &dma_obj->base;
    int ret;

    /*
     * Clear the VM_PFNMAP flag that was set by drm_gem_mmap(), and set the
     * vm_pgoff (used as a fake buffer offset by DRM) to 0 as we want to map
     * the whole buffer.
     */
    vma->vm_pgoff -= drm_vma_node_start(&obj->vma_node);
    vma->vm_flags &= ~VM_PFNMAP;
    vma->vm_flags |= VM_DONTEXPAND;

    if (dma_obj->map_noncoherent) {
        vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
    }
    ret = vm_iomap_memory(vma, virt_to_phys(dma_obj->vaddr), vma->vm_end - vma->vm_start);
    if (ret)
        drm_gem_vm_close(vma);

    return ret;
}

static inline int ili9320_drm_gem_dma_object_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct drm_gem_dma_object *dma_obj = to_drm_gem_dma_obj(obj);

	return ili9320_drm_gem_dma_mmap(dma_obj, vma);
}

int ili9320_drm_gem_dma_vmap(struct drm_gem_dma_object *dma_obj,
             struct iosys_map *map)
{
    // using iomem here to get screen_base initialized for get_fb_unmapped_area
    // TODO: Check again if still needed.
    iosys_map_set_vaddr_iomem(map, dma_obj->vaddr);

    return 0;
}

static inline int ili9320_drm_gem_dma_object_vmap(struct drm_gem_object *obj,
					  struct iosys_map *map)
{
	struct drm_gem_dma_object *dma_obj = to_drm_gem_dma_obj(obj);

	return drm_gem_dma_vmap(dma_obj, map);
}

static const struct drm_gem_object_funcs drm_gem_dma_default_funcs = {
	.free = drm_gem_dma_object_free,
	.print_info = drm_gem_dma_object_print_info,
	.get_sg_table = drm_gem_dma_object_get_sg_table,
	.vmap = ili9320_drm_gem_dma_object_vmap,
	.mmap = ili9320_drm_gem_dma_object_mmap,
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
		dma_obj->dma_addr = (unsigned int)kzalloc(size, GFP_KERNEL);
		dma_obj->vaddr = (void*)dma_obj->dma_addr;
#if 0
		dma_obj->vaddr = dma_alloc_noncoherent(drm->dev, size,
							   &dma_obj->dma_addr,
							   DMA_TO_DEVICE,
							   GFP_KERNEL | __GFP_NOWARN);
#endif
	} else {
		dma_obj->dma_addr = (unsigned int)kzalloc(size, GFP_KERNEL);
		dma_obj->vaddr = (void*)dma_obj->dma_addr;
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
	.gem_prime_mmap		= drm_gem_prime_mmap,
	.fops			= &ili9320drm_fops,
};

static void ili9320_drm_fb_helper_fill_pixel_fmt(struct fb_var_screeninfo *var,
					 const struct drm_format_info *format)
{
	u8 depth = format->depth;

	if (format->is_color_indexed) {
		var->red.offset = 0;
		var->green.offset = 0;
		var->blue.offset = 0;
		var->red.length = depth;
		var->green.length = depth;
		var->blue.length = depth;
		var->transp.offset = 0;
		var->transp.length = 0;
		return;
	}

	switch (depth) {
	case 15:
		var->red.offset = 10;
		var->green.offset = 5;
		var->blue.offset = 0;
		var->red.length = 5;
		var->green.length = 5;
		var->blue.length = 5;
		var->transp.offset = 15;
		var->transp.length = 1;
		break;
	case 16:
    // ili9320
		var->red.offset = 6;
		var->green.offset = 0;
		var->blue.offset = 11;
		var->red.length = 5;
		var->green.length = 6;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 24:
    // ili9320
		var->red.offset = 8;
		var->green.offset = 0;
		var->blue.offset = 16;
		var->red.length = 8;
		var->green.length = 8;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 32:
		var->red.offset = 8;
		var->green.offset = 0;
		var->blue.offset = 16;
		var->red.length = 8;
		var->green.length = 8;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		break;
	default:
		break;
	}
}


static void ili9320_drm_fb_helper_fill_var(struct fb_info *info,
				   struct drm_fb_helper *fb_helper,
				   uint32_t fb_width, uint32_t fb_height)
{
	struct drm_framebuffer *fb = fb_helper->fb;
	const struct drm_format_info *format = fb->format;

	switch (format->format) {
	case DRM_FORMAT_C1:
	case DRM_FORMAT_C2:
	case DRM_FORMAT_C4:
		/* supported format with sub-byte pixels */
		break;

	default:
		WARN_ON((drm_format_info_block_width(format, 0) > 1) ||
			(drm_format_info_block_height(format, 0) > 1));
		break;
	}

	info->pseudo_palette = fb_helper->pseudo_palette;
	info->var.xres_virtual = fb->width;
	info->var.yres_virtual = fb->height;
	info->var.bits_per_pixel = drm_format_info_bpp(format, 0);
	info->var.accel_flags = FB_ACCELF_TEXT;
	info->var.xoffset = 0;
	info->var.yoffset = 0;
	info->var.activate = FB_ACTIVATE_NOW;

	ili9320_drm_fb_helper_fill_pixel_fmt(&info->var, format);

	info->var.xres = fb_width;
	info->var.yres = fb_height;
}

static void drm_fb_helper_fill_fix(struct fb_info *info, uint32_t pitch,
				   bool is_color_indexed)
{
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = is_color_indexed ? FB_VISUAL_PSEUDOCOLOR
					    : FB_VISUAL_TRUECOLOR;
	info->fix.mmio_start = 0;
	info->fix.mmio_len = 0;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 1; /* doing it in hw */
	info->fix.ypanstep = 1; /* doing it in hw */
	info->fix.ywrapstep = 0;
	info->fix.accel = FB_ACCEL_NONE;

	info->fix.line_length = pitch;
}

void ili9320_drm_fb_helper_fill_info(struct fb_info *info,
			     struct drm_fb_helper *fb_helper,
			     struct drm_fb_helper_surface_size *sizes)
{
	struct drm_framebuffer *fb = fb_helper->fb;

	drm_fb_helper_fill_fix(info, fb->pitches[0],
			       fb->format->is_color_indexed);
	ili9320_drm_fb_helper_fill_var(info, fb_helper,
			       sizes->fb_width, sizes->fb_height);

	info->par = fb_helper;
	/*
	 * The DRM drivers fbdev emulation device name can be confusing if the
	 * driver name also has a "drm" suffix on it. Leading to names such as
	 * "simpledrmdrmfb" in /proc/fb. Unfortunately, it's an uAPI and can't
	 * be changed due user-space tools (e.g: pm-utils) matching against it.
	 */
	snprintf(info->fix.id, sizeof(info->fix.id), "%sdrmfb",
		 fb_helper->dev->driver->name);

}

static bool drm_fbdev_use_shadow_fb(struct drm_fb_helper *fb_helper)
{
	struct drm_device *dev = fb_helper->dev;
	struct drm_framebuffer *fb = fb_helper->fb;

	return dev->mode_config.prefer_shadow_fbdev ||
	       dev->mode_config.prefer_shadow ||
	       fb->funcs->dirty;
}

/* @user: 1=userspace, 0=fbcon */
static int drm_fbdev_fb_open(struct fb_info *info, int user)
{
	struct drm_fb_helper *fb_helper = info->par;

	/* No need to take a ref for fbcon because it unbinds on unregister */
	if (user && !try_module_get(fb_helper->dev->driver->fops->owner))
		return -ENODEV;

	return 0;
}

static int drm_fbdev_fb_release(struct fb_info *info, int user)
{
	struct drm_fb_helper *fb_helper = info->par;

	if (user)
		module_put(fb_helper->dev->driver->fops->owner);

	return 0;
}

static void drm_fbdev_cleanup(struct drm_fb_helper *fb_helper)
{
	struct fb_info *fbi = fb_helper->info;
	void *shadow = NULL;

	if (!fb_helper->dev)
		return;

	if (fbi) {
		if (fbi->fbdefio)
			fb_deferred_io_cleanup(fbi);
		if (drm_fbdev_use_shadow_fb(fb_helper))
			shadow = fbi->screen_buffer;
	}

	drm_fb_helper_fini(fb_helper);

	if (shadow)
		vfree(shadow);
	else if (fb_helper->buffer)
		drm_client_buffer_vunmap(fb_helper->buffer);

	drm_client_framebuffer_delete(fb_helper->buffer);
}

static void drm_fbdev_release(struct drm_fb_helper *fb_helper)
{
	drm_fbdev_cleanup(fb_helper);
	drm_client_release(&fb_helper->client);
	kfree(fb_helper);
}

/*
 * fb_ops.fb_destroy is called by the last put_fb_info() call at the end of
 * unregister_framebuffer() or fb_release().
 */
static void drm_fbdev_fb_destroy(struct fb_info *info)
{
	drm_fbdev_release(info->par);
}

static int drm_fbdev_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct drm_fb_helper *fb_helper = info->par;

	if (drm_fbdev_use_shadow_fb(fb_helper))
		return fb_deferred_io_mmap(info, vma);
	else if (fb_helper->dev->driver->gem_prime_mmap)
		return fb_helper->dev->driver->gem_prime_mmap(fb_helper->buffer->gem, vma);
	else
		return -ENODEV;
}

static bool drm_fbdev_use_iomem(struct fb_info *info)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct drm_client_buffer *buffer = fb_helper->buffer;

	return !drm_fbdev_use_shadow_fb(fb_helper) && buffer->map.is_iomem;
}

static ssize_t drm_fbdev_fb_read(struct fb_info *info, char __user *buf,
				 size_t count, loff_t *ppos)
{
	ssize_t ret;

	if (drm_fbdev_use_iomem(info))
		ret = drm_fb_helper_cfb_read(info, buf, count, ppos);
	else
		ret = drm_fb_helper_sys_read(info, buf, count, ppos);

	return ret;
}

static ssize_t drm_fbdev_fb_write(struct fb_info *info, const char __user *buf,
				  size_t count, loff_t *ppos)
{
	ssize_t ret;

	if (drm_fbdev_use_iomem(info))
		ret = drm_fb_helper_cfb_write(info, buf, count, ppos);
	else
		ret = drm_fb_helper_sys_write(info, buf, count, ppos);

	return ret;
}

static void drm_fbdev_fb_fillrect(struct fb_info *info,
				  const struct fb_fillrect *rect)
{
	if (drm_fbdev_use_iomem(info))
		drm_fb_helper_cfb_fillrect(info, rect);
	else
		drm_fb_helper_sys_fillrect(info, rect);
}

static void drm_fbdev_fb_copyarea(struct fb_info *info,
				  const struct fb_copyarea *area)
{
	if (drm_fbdev_use_iomem(info))
		drm_fb_helper_cfb_copyarea(info, area);
	else
		drm_fb_helper_sys_copyarea(info, area);
}

static void drm_fbdev_fb_imageblit(struct fb_info *info,
				   const struct fb_image *image)
{
	if (drm_fbdev_use_iomem(info))
		drm_fb_helper_cfb_imageblit(info, image);
	else
		drm_fb_helper_sys_imageblit(info, image);
}

static const struct fb_ops drm_fbdev_fb_ops = {
	.owner		= THIS_MODULE,
	DRM_FB_HELPER_DEFAULT_OPS,
	.fb_open	= drm_fbdev_fb_open,
	.fb_release	= drm_fbdev_fb_release,
	.fb_destroy	= drm_fbdev_fb_destroy,
	.fb_mmap	= drm_fbdev_fb_mmap,
	.fb_read	= drm_fbdev_fb_read,
	.fb_write	= drm_fbdev_fb_write,
	.fb_fillrect	= drm_fbdev_fb_fillrect,
	.fb_copyarea	= drm_fbdev_fb_copyarea,
	.fb_imageblit	= drm_fbdev_fb_imageblit,
};

static struct fb_deferred_io drm_fbdev_defio = {
	.delay		= HZ / 20,
	.deferred_io	= drm_fb_helper_deferred_io,
};

static int ili9320_drm_fbdev_fb_probe(struct drm_fb_helper *fb_helper,
			      struct drm_fb_helper_surface_size *sizes)
{
	struct drm_client_dev *client = &fb_helper->client;
	struct drm_device *dev = fb_helper->dev;
	struct drm_client_buffer *buffer;
	struct drm_framebuffer *fb;
	struct fb_info *fbi;
	u32 format;
	struct iosys_map map;
	int ret;

	drm_dbg_kms(dev, "surface width(%d), height(%d) and bpp(%d)\n",
		    sizes->surface_width, sizes->surface_height,
		    sizes->surface_bpp);

	format = drm_mode_legacy_fb_format(sizes->surface_bpp, sizes->surface_depth);
	buffer = drm_client_framebuffer_create(client, sizes->surface_width,
					       sizes->surface_height, format);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	fb_helper->buffer = buffer;
	fb_helper->fb = buffer->fb;
	fb = buffer->fb;

	fbi = drm_fb_helper_alloc_info(fb_helper);
	if (IS_ERR(fbi))
		return PTR_ERR(fbi);

	fbi->fbops = &drm_fbdev_fb_ops;
	fbi->screen_size = sizes->surface_height * fb->pitches[0];
	fbi->fix.smem_len = fbi->screen_size;
	fbi->flags = FBINFO_DEFAULT;

	ili9320_drm_fb_helper_fill_info(fbi, fb_helper, sizes);

	if (drm_fbdev_use_shadow_fb(fb_helper)) {
		fbi->screen_buffer = vzalloc(fbi->screen_size);
		if (!fbi->screen_buffer)
			return -ENOMEM;
		fbi->flags |= FBINFO_VIRTFB | FBINFO_READS_FAST;

		fbi->fbdefio = &drm_fbdev_defio;
		fb_deferred_io_init(fbi);
	} else {
		/* buffer is mapped for HW framebuffer */
		ret = drm_client_buffer_vmap(fb_helper->buffer, &map);
		if (ret)
			return ret;
		if (map.is_iomem) {
			fbi->screen_base = map.vaddr_iomem;
		} else {
			fbi->screen_buffer = map.vaddr;
			fbi->flags |= FBINFO_VIRTFB;
		}

		/*
		 * Shamelessly leak the physical address to user-space. As
		 * page_to_phys() is undefined for I/O memory, warn in this
		 * case.
		 */
#if IS_ENABLED(CONFIG_DRM_FBDEV_LEAK_PHYS_SMEM)
		if (fb_helper->hint_leak_smem_start && fbi->fix.smem_start == 0 &&
		    !drm_WARN_ON_ONCE(dev, map.is_iomem))
			fbi->fix.smem_start =
				page_to_phys(virt_to_page(fbi->screen_buffer));
#endif
	}

	return 0;
}

static void ili9320_drm_fbdev_damage_blit_real(struct drm_fb_helper *fb_helper,
				       struct drm_clip_rect *clip,
				       struct iosys_map *dst)
{
	struct drm_framebuffer *fb = fb_helper->fb;
	size_t offset = clip->y1 * fb->pitches[0];
	size_t len = clip->x2 - clip->x1;
	unsigned int y;
	void *src;

	switch (drm_format_info_bpp(fb->format, 0)) {
	case 1:
		offset += clip->x1 / 8;
		len = DIV_ROUND_UP(len + clip->x1 % 8, 8);
		break;
	case 2:
		offset += clip->x1 / 4;
		len = DIV_ROUND_UP(len + clip->x1 % 4, 4);
		break;
	case 4:
		offset += clip->x1 / 2;
		len = DIV_ROUND_UP(len + clip->x1 % 2, 2);
		break;
	default:
		offset += clip->x1 * fb->format->cpp[0];
		len *= fb->format->cpp[0];
		break;
	}

	src = fb_helper->info->screen_buffer + offset;
	iosys_map_incr(dst, offset); /* go to first pixel within clip rect */

	for (y = clip->y1; y < clip->y2; y++) {
		iosys_map_memcpy_to(dst, 0, src, len);
		iosys_map_incr(dst, fb->pitches[0]);
		src += fb->pitches[0];
	}
}

static int ili9320_drm_fbdev_damage_blit(struct drm_fb_helper *fb_helper,
				 struct drm_clip_rect *clip)
{
	struct drm_client_buffer *buffer = fb_helper->buffer;
	struct iosys_map map, dst;
	int ret;

	/*
	 * We have to pin the client buffer to its current location while
	 * flushing the shadow buffer. In the general case, concurrent
	 * modesetting operations could try to move the buffer and would
	 * fail. The modeset has to be serialized by acquiring the reservation
	 * object of the underlying BO here.
	 *
	 * For fbdev emulation, we only have to protect against fbdev modeset
	 * operations. Nothing else will involve the client buffer's BO. So it
	 * is sufficient to acquire struct drm_fb_helper.lock here.
	 */
	mutex_lock(&fb_helper->lock);

	ret = drm_client_buffer_vmap(buffer, &map);
	if (ret)
		goto out;

	dst = map;
	ili9320_drm_fbdev_damage_blit_real(fb_helper, clip, &dst);

	drm_client_buffer_vunmap(buffer);

out:
	mutex_unlock(&fb_helper->lock);

	return ret;
}

static int ili9320_drm_fbdev_fb_dirty(struct drm_fb_helper *helper, struct drm_clip_rect *clip)
{
	struct drm_device *dev = helper->dev;
	int ret;

	if (!drm_fbdev_use_shadow_fb(helper))
		return 0;

	/* Call damage handlers only if necessary */
	if (!(clip->x1 < clip->x2 && clip->y1 < clip->y2))
		return 0;

	if (helper->buffer) {
		ret = ili9320_drm_fbdev_damage_blit(helper, clip);
		if (drm_WARN_ONCE(dev, ret, "Damage blitter failed: ret=%d\n", ret))
			return ret;
	}

	if (helper->fb->funcs->dirty) {
		ret = helper->fb->funcs->dirty(helper->fb, NULL, 0, 0, clip, 1);
		if (drm_WARN_ONCE(dev, ret, "Dirty helper failed: ret=%d\n", ret))
			return ret;
	}

	return 0;
}

static const struct drm_fb_helper_funcs drm_fb_helper_ili9320_funcs = {
	.fb_probe = ili9320_drm_fbdev_fb_probe,
	.fb_dirty = ili9320_drm_fbdev_fb_dirty,
};

static int ili9320_drm_fbdev_client_hotplug(struct drm_client_dev *client)
{
	struct drm_fb_helper *fb_helper = drm_fb_helper_from_client(client);
	struct drm_device *dev = client->dev;
	int ret;

	/* Setup is not retried if it has failed */
	if (!fb_helper->dev && fb_helper->funcs)
		return 0;

	if (dev->fb_helper)
		return drm_fb_helper_hotplug_event(dev->fb_helper);

	if (!dev->mode_config.num_connector) {
		drm_dbg_kms(dev, "No connectors found, will not create framebuffer!\n");
		return 0;
	}

	drm_fb_helper_prepare(dev, fb_helper, &drm_fb_helper_ili9320_funcs);

	ret = drm_fb_helper_init(dev, fb_helper);
	if (ret)
		goto err;

	if (!drm_drv_uses_atomic_modeset(dev))
		drm_helper_disable_unused_functions(dev);

	ret = drm_fb_helper_initial_config(fb_helper, fb_helper->preferred_bpp);
	if (ret)
		goto err_cleanup;

	return 0;

err_cleanup:
	drm_fbdev_cleanup(fb_helper);
err:
	fb_helper->dev = NULL;
	fb_helper->info = NULL;

	drm_err(dev, "fbdev: Failed to setup generic emulation (ret=%d)\n", ret);

	return ret;
}

static void drm_fbdev_client_unregister(struct drm_client_dev *client)
{
	struct drm_fb_helper *fb_helper = drm_fb_helper_from_client(client);

	if (fb_helper->info)
		/* drm_fbdev_fb_destroy() takes care of cleanup */
		drm_fb_helper_unregister_info(fb_helper);
	else
		drm_fbdev_release(fb_helper);
}

static int drm_fbdev_client_restore(struct drm_client_dev *client)
{
	drm_fb_helper_lastclose(client->dev);

	return 0;
}

static const struct drm_client_funcs drm_fbdev_client_funcs = {
	.owner		= THIS_MODULE,
	.unregister	= drm_fbdev_client_unregister,
	.restore	= drm_fbdev_client_restore,
	.hotplug	= ili9320_drm_fbdev_client_hotplug,
};

void ili9320_drm_fbdev_setup(struct drm_device *dev,
                 unsigned int preferred_bpp)
{
    struct drm_fb_helper *fb_helper;
    int ret;

    drm_WARN(dev, !dev->registered, "Device has not been registered.\n");
    drm_WARN(dev, dev->fb_helper, "fb_helper is already set!\n");

    fb_helper = kzalloc(sizeof(*fb_helper), GFP_KERNEL);
    if (!fb_helper)
        return;

    ret = drm_client_init(dev, &fb_helper->client, "fbdev", &drm_fbdev_client_funcs);
    if (ret) {
        kfree(fb_helper);
        drm_err(dev, "Failed to register client: %d\n", ret);
        return;
    }

    /*
     * FIXME: This mixes up depth with bpp, which results in a glorious
     * mess, resulting in some drivers picking wrong fbdev defaults and
     * others wrong preferred_depth defaults.
     */
    if (!preferred_bpp)
        preferred_bpp = dev->mode_config.preferred_depth;
    if (!preferred_bpp)
        preferred_bpp = 32;
    fb_helper->preferred_bpp = preferred_bpp;

    ret = ili9320_drm_fbdev_client_hotplug(&fb_helper->client);
    if (ret)
        drm_dbg_kms(dev, "client hotplug ret=%d\n", ret);

    drm_client_register(&fb_helper->client);
}

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

	//lcd_init();
	ili9320_drm_fbdev_setup(dev, FORMAT_DEPTH);

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
