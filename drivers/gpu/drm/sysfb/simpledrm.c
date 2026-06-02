// SPDX-License-Identifier: GPL-2.0-only

#include <linux/aperture.h>
#include <linux/minmax.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_data/simplefb.h>
#include <linux/platform_device.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/simplefb_resources.h>

#include "drm_sysfb_helper.h"

#define DRIVER_NAME	"simpledrm"
#define DRIVER_DESC	"DRM driver for simple-framebuffer platform devices"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

/*
 * Helpers for simplefb
 */

static int
simplefb_get_validated_int(struct drm_device *dev, const char *name,
			   uint32_t value)
{
	return drm_sysfb_get_validated_int(dev, name, value, INT_MAX);
}

static int
simplefb_get_validated_int0(struct drm_device *dev, const char *name,
			    uint32_t value)
{
	return drm_sysfb_get_validated_int0(dev, name, value, INT_MAX);
}

static const struct drm_format_info *
simplefb_get_validated_format(struct drm_device *dev, const char *format_name)
{
	static const struct simplefb_format formats[] = SIMPLEFB_FORMATS;
	const struct simplefb_format *fmt = formats;
	const struct simplefb_format *end = fmt + ARRAY_SIZE(formats);
	const struct drm_format_info *info;

	if (!format_name) {
		drm_err(dev, "simplefb: missing framebuffer format\n");
		return ERR_PTR(-EINVAL);
	}

	while (fmt < end) {
		if (!strcmp(format_name, fmt->name)) {
			info = drm_format_info(fmt->fourcc);
			if (!info)
				return ERR_PTR(-EINVAL);
			return info;
		}
		++fmt;
	}

	drm_err(dev, "simplefb: unknown framebuffer format %s\n",
		format_name);

	return ERR_PTR(-EINVAL);
}

static int
simplefb_get_width_pd(struct drm_device *dev,
		      const struct simplefb_platform_data *pd)
{
	return simplefb_get_validated_int0(dev, "width", pd->width);
}

static int
simplefb_get_height_pd(struct drm_device *dev,
		       const struct simplefb_platform_data *pd)
{
	return simplefb_get_validated_int0(dev, "height", pd->height);
}

static int
simplefb_get_stride_pd(struct drm_device *dev,
		       const struct simplefb_platform_data *pd)
{
	return simplefb_get_validated_int(dev, "stride", pd->stride);
}

static const struct drm_format_info *
simplefb_get_format_pd(struct drm_device *dev,
		       const struct simplefb_platform_data *pd)
{
	return simplefb_get_validated_format(dev, pd->format);
}

static int
simplefb_read_u32_of(struct drm_device *dev, struct device_node *of_node,
		     const char *name, u32 *value)
{
	int ret = of_property_read_u32(of_node, name, value);

	if (ret)
		drm_err(dev, "simplefb: cannot parse framebuffer %s: error %d\n",
			name, ret);
	return ret;
}

static int
simplefb_read_string_of(struct drm_device *dev, struct device_node *of_node,
			const char *name, const char **value)
{
	int ret = of_property_read_string(of_node, name, value);

	if (ret)
		drm_err(dev, "simplefb: cannot parse framebuffer %s: error %d\n",
			name, ret);
	return ret;
}

static int
simplefb_get_width_of(struct drm_device *dev, struct device_node *of_node)
{
	u32 width;
	int ret = simplefb_read_u32_of(dev, of_node, "width", &width);

	if (ret)
		return ret;
	return simplefb_get_validated_int0(dev, "width", width);
}

static int
simplefb_get_height_of(struct drm_device *dev, struct device_node *of_node)
{
	u32 height;
	int ret = simplefb_read_u32_of(dev, of_node, "height", &height);

	if (ret)
		return ret;
	return simplefb_get_validated_int0(dev, "height", height);
}

static int
simplefb_get_stride_of(struct drm_device *dev, struct device_node *of_node)
{
	u32 stride;
	int ret = simplefb_read_u32_of(dev, of_node, "stride", &stride);

	if (ret)
		return ret;
	return simplefb_get_validated_int(dev, "stride", stride);
}

static const struct drm_format_info *
simplefb_get_format_of(struct drm_device *dev, struct device_node *of_node)
{
	const char *format;
	int ret = simplefb_read_string_of(dev, of_node, "format", &format);

	if (ret)
		return ERR_PTR(ret);
	return simplefb_get_validated_format(dev, format);
}

static struct resource *
simplefb_get_memory_of(struct drm_device *dev, struct device_node *of_node)
{
	struct resource r, *res;
	int err;

	err = of_reserved_mem_region_to_resource(of_node, 0, &r);
	if (err)
		return NULL;

	res = devm_kmemdup(dev->dev, &r, sizeof(r), GFP_KERNEL);
	if (!res)
		return ERR_PTR(-ENOMEM);

	if (of_property_present(of_node, "reg"))
		drm_warn(dev, "preferring \"memory-region\" over \"reg\" property\n");

	return res;
}

/*
 * Simple Framebuffer device
 */

struct simpledrm_device {
	struct drm_sysfb_device sysfb;
	struct simplefb_resources resources;

	/* modesetting */
	u32 formats[DRM_SYSFB_PLANE_NFORMATS(1)];
	struct drm_plane primary_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
};

static void simpledrm_device_release_resources(void *res)
{
	simplefb_release_resources(res);
}

/*
 * Modesetting
 */

static const u64 simpledrm_primary_plane_format_modifiers[] = {
	DRM_SYSFB_PLANE_FORMAT_MODIFIERS,
};

static const struct drm_plane_helper_funcs simpledrm_primary_plane_helper_funcs = {
	DRM_SYSFB_PLANE_HELPER_FUNCS,
};

static const struct drm_plane_funcs simpledrm_primary_plane_funcs = {
	DRM_SYSFB_PLANE_FUNCS,
	.destroy = drm_plane_cleanup,
};

static const struct drm_crtc_helper_funcs simpledrm_crtc_helper_funcs = {
	DRM_SYSFB_CRTC_HELPER_FUNCS,
};

static const struct drm_crtc_funcs simpledrm_crtc_funcs = {
	DRM_SYSFB_CRTC_FUNCS,
	.destroy = drm_crtc_cleanup,
};

static const struct drm_encoder_funcs simpledrm_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_connector_helper_funcs simpledrm_connector_helper_funcs = {
	DRM_SYSFB_CONNECTOR_HELPER_FUNCS,
};

static const struct drm_connector_funcs simpledrm_connector_funcs = {
	DRM_SYSFB_CONNECTOR_FUNCS,
	.destroy = drm_connector_cleanup,
};

static const struct drm_mode_config_funcs simpledrm_mode_config_funcs = {
	DRM_SYSFB_MODE_CONFIG_FUNCS,
};

/*
 * Init / Cleanup
 */

static struct simpledrm_device *simpledrm_device_create(struct drm_driver *drv,
							struct platform_device *pdev)
{
	const struct simplefb_platform_data *pd = dev_get_platdata(&pdev->dev);
	struct device_node *of_node = pdev->dev.of_node;
	struct simpledrm_device *sdev;
	struct drm_sysfb_device *sysfb;
	struct drm_device *dev;
	int width, height, stride;
	int width_mm = 0, height_mm = 0;
	struct device_node *panel_node;
	const struct drm_format_info *format;
	struct resource *res, *mem = NULL;
	struct drm_plane *primary_plane;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	unsigned long max_width, max_height;
	size_t nformats;
	int ret;

	sdev = devm_drm_dev_alloc(&pdev->dev, drv, struct simpledrm_device, sysfb.dev);
	if (IS_ERR(sdev))
		return ERR_CAST(sdev);
	sysfb = &sdev->sysfb;
	dev = &sysfb->dev;
	platform_set_drvdata(pdev, sdev);

	ret = simplefb_acquire_resources(&pdev->dev, &sdev->resources);
	if (ret)
		return ERR_PTR(ret);

	ret = devm_add_action_or_reset(&pdev->dev, simpledrm_device_release_resources,
				       &sdev->resources);
	if (ret)
		return ERR_PTR(ret);

	if (pd) {
		width = simplefb_get_width_pd(dev, pd);
		if (width < 0)
			return ERR_PTR(width);
		height = simplefb_get_height_pd(dev, pd);
		if (height < 0)
			return ERR_PTR(height);
		stride = simplefb_get_stride_pd(dev, pd);
		if (stride < 0)
			return ERR_PTR(stride);
		format = simplefb_get_format_pd(dev, pd);
		if (IS_ERR(format))
			return ERR_CAST(format);
	} else if (of_node) {
		width = simplefb_get_width_of(dev, of_node);
		if (width < 0)
			return ERR_PTR(width);
		height = simplefb_get_height_of(dev, of_node);
		if (height < 0)
			return ERR_PTR(height);
		stride = simplefb_get_stride_of(dev, of_node);
		if (stride < 0)
			return ERR_PTR(stride);
		format = simplefb_get_format_of(dev, of_node);
		if (IS_ERR(format))
			return ERR_CAST(format);
		mem = simplefb_get_memory_of(dev, of_node);
		if (IS_ERR(mem))
			return ERR_CAST(mem);
		panel_node = of_parse_phandle(of_node, "panel", 0);
		if (panel_node) {
			simplefb_read_u32_of(dev, panel_node, "width-mm", &width_mm);
			simplefb_read_u32_of(dev, panel_node, "height-mm", &height_mm);
			of_node_put(panel_node);
		}
	} else {
		drm_err(dev, "no simplefb configuration found\n");
		return ERR_PTR(-ENODEV);
	}
	if (!stride) {
		stride = drm_format_info_min_pitch(format, 0, width);
		if (drm_WARN_ON(dev, !stride))
			return ERR_PTR(-EINVAL);
	}

	sysfb->fb_mode = drm_sysfb_mode(width, height, width_mm, height_mm);
	sysfb->fb_format = format;
	sysfb->fb_pitch = stride;

	drm_dbg(dev, "display mode={" DRM_MODE_FMT "}\n", DRM_MODE_ARG(&sysfb->fb_mode));
	drm_dbg(dev, "framebuffer format=%p4cc, size=%dx%d, stride=%d byte\n",
		&format->format, width, height, stride);

	/*
	 * Memory management
	 */

	if (mem) {
		void *screen_base;

		ret = devm_aperture_acquire_for_platform_device(pdev, mem->start,
								resource_size(mem),
								NULL);
		if (ret) {
			drm_err(dev, "could not acquire memory range %pr: %d\n", mem, ret);
			return ERR_PTR(ret);
		}

		drm_dbg(dev, "using system memory framebuffer at %pr\n", mem);

		screen_base = devm_memremap(dev->dev, mem->start, resource_size(mem), MEMREMAP_WC);
		if (IS_ERR(screen_base))
			return screen_base;

		iosys_map_set_vaddr(&sysfb->fb_addr, screen_base);
	} else {
		void __iomem *screen_base;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res)
			return ERR_PTR(-EINVAL);

		ret = devm_aperture_acquire_for_platform_device(pdev, res->start,
								resource_size(res),
								NULL);
		if (ret) {
			drm_err(dev, "could not acquire memory range %pr: %d\n", res, ret);
			return ERR_PTR(ret);
		}

		drm_dbg(dev, "using I/O memory framebuffer at %pr\n", res);

		mem = devm_request_mem_region(&pdev->dev, res->start, resource_size(res),
					      drv->name);
		if (!mem) {
			/*
			 * We cannot make this fatal. Sometimes this comes from magic
			 * spaces our resource handlers simply don't know about. Use
			 * the I/O-memory resource as-is and try to map that instead.
			 */
			drm_warn(dev, "could not acquire memory region %pr\n", res);
			mem = res;
		}

		screen_base = devm_ioremap_wc(&pdev->dev, mem->start, resource_size(mem));
		if (!screen_base)
			return ERR_PTR(-ENOMEM);

		iosys_map_set_vaddr_iomem(&sysfb->fb_addr, screen_base);
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
	dev->mode_config.preferred_depth = format->depth;
	dev->mode_config.funcs = &simpledrm_mode_config_funcs;

	/* Primary plane */

	nformats = drm_sysfb_build_fourcc_list(dev, &format->format, 1,
					       sdev->formats, ARRAY_SIZE(sdev->formats));

	primary_plane = &sdev->primary_plane;
	ret = drm_universal_plane_init(dev, primary_plane, 0, &simpledrm_primary_plane_funcs,
				       sdev->formats, nformats,
				       simpledrm_primary_plane_format_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ERR_PTR(ret);
	drm_plane_helper_add(primary_plane, &simpledrm_primary_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(primary_plane);

	/* CRTC */

	crtc = &sdev->crtc;
	ret = drm_crtc_init_with_planes(dev, crtc, primary_plane, NULL,
					&simpledrm_crtc_funcs, NULL);
	if (ret)
		return ERR_PTR(ret);
	drm_crtc_helper_add(crtc, &simpledrm_crtc_helper_funcs);

	/* Encoder */

	encoder = &sdev->encoder;
	ret = drm_encoder_init(dev, encoder, &simpledrm_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ERR_PTR(ret);
	encoder->possible_crtcs = drm_crtc_mask(crtc);

	/* Connector */

	connector = &sdev->connector;
	ret = drm_connector_init(dev, connector, &simpledrm_connector_funcs,
				 DRM_MODE_CONNECTOR_Unknown);
	if (ret)
		return ERR_PTR(ret);
	drm_connector_helper_add(connector, &simpledrm_connector_helper_funcs);
	drm_connector_set_panel_orientation_with_quirk(connector,
						       DRM_MODE_PANEL_ORIENTATION_UNKNOWN,
						       width, height);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		return ERR_PTR(ret);

	drm_mode_config_reset(dev);

	return sdev;
}

/*
 * DRM driver
 */

DEFINE_DRM_GEM_FOPS(simpledrm_fops);

static struct drm_driver simpledrm_driver = {
	DRM_GEM_SHMEM_DRIVER_OPS,
	DRM_FBDEV_SHMEM_DRIVER_OPS,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
	.driver_features	= DRIVER_ATOMIC | DRIVER_GEM | DRIVER_MODESET,
	.fops			= &simpledrm_fops,
};

/*
 * Platform driver
 */

static int simpledrm_probe(struct platform_device *pdev)
{
	struct simpledrm_device *sdev;
	struct drm_sysfb_device *sysfb;
	struct drm_device *dev;
	int ret;

	sdev = simpledrm_device_create(&simpledrm_driver, pdev);
	if (IS_ERR(sdev))
		return PTR_ERR(sdev);
	sysfb = &sdev->sysfb;
	dev = &sysfb->dev;

	ret = drm_dev_register(dev, 0);
	if (ret)
		return ret;

	drm_client_setup(dev, sdev->sysfb.fb_format);

	return 0;
}

static void simpledrm_remove(struct platform_device *pdev)
{
	struct simpledrm_device *sdev = platform_get_drvdata(pdev);
	struct drm_device *dev = &sdev->sysfb.dev;

	drm_dev_unplug(dev);
}

static const struct of_device_id simpledrm_of_match_table[] = {
	{ .compatible = "simple-framebuffer", },
	{ },
};
MODULE_DEVICE_TABLE(of, simpledrm_of_match_table);

static struct platform_driver simpledrm_platform_driver = {
	.driver = {
		.name = "simple-framebuffer", /* connect to sysfb */
		.of_match_table = simpledrm_of_match_table,
	},
	.probe = simpledrm_probe,
	.remove = simpledrm_remove,
};

module_platform_driver(simpledrm_platform_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
