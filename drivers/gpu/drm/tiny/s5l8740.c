// SPDX-License-Identifier: GPL-2.0-only

#define DEBUG
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/clients/drm_client_setup.h>


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

	/* modesetting */
    uint32_t formats[8];
    size_t nformats;
    struct drm_plane primary_plane;
    struct drm_crtc crtc;
    struct drm_encoder encoder;
    struct drm_connector connector;
};

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

static void s5l8740_primary_plane_helper_atomic_update(struct drm_plane *plane,
    struct drm_atomic_state *state)
{
    struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
    struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);
    struct drm_framebuffer *fb = plane_state->fb;
    struct drm_device *dev = plane->dev;
    struct s5l8740_device *sdev = s5l8740_device_of_dev(dev);
    int idx;

    if (!fb || drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE))
        return;

    if (!drm_dev_enter(dev, &idx))
        goto out_drm_gem_fb_end_cpu_access;

    unsigned int count = fb->width * fb->height;
    int *src = shadow_plane_state->data[0].vaddr;
    void *dst = sdev->lcdif;

    for (int i=0; i < count;i++){
        while ( (readl(dst + 0x1C) & 0x10) == 0x10 );
        writel(src[i],dst + 0x40);
    }

    drm_dev_exit(idx);
out_drm_gem_fb_end_cpu_access:
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
 };
 
 module_platform_driver(s5l8740_platform_driver);
 
 MODULE_DESCRIPTION("s5l8740 tiny drm");
 MODULE_LICENSE("GPL v2");
