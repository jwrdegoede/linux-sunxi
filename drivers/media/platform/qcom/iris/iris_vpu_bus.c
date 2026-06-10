// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/device.h>
#include <linux/iris_vpu_bus.h>
#include <linux/of_device.h>
#include <linux/slab.h>

static int iris_vpu_bus_dma_configure(struct device *dev)
{
	const u32 *iommu_fid = (const u32 *)dev_get_platdata(dev);

	return of_dma_configure_id(dev, dev->parent->of_node, true, iommu_fid);
}

const struct bus_type iris_vpu_bus_type = {
	.name = "iris-vpu-bus",
	.dma_configure = iris_vpu_bus_dma_configure,
};
EXPORT_SYMBOL_GPL(iris_vpu_bus_type);

static void iris_vpu_bus_release_device(struct device *dev)
{
	kfree(dev);
}

struct device *iris_vpu_bus_create_device(struct device *parent_device, const char *name,
					  u64 dma_mask, const u32 *iommu_fid)
{
	struct device *dev;
	int ret;

	dev = kzalloc_obj(*dev);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	dev->release = iris_vpu_bus_release_device;
	dev->bus = &iris_vpu_bus_type;
	dev->parent = parent_device;
	dev->coherent_dma_mask = dma_mask;
	dev->dma_mask = &dev->coherent_dma_mask;
	dev->platform_data = (void *)iommu_fid;
	dev_set_name(dev, "%s", name);

	ret = device_register(dev);
	if (ret) {
		put_device(dev);
		return ERR_PTR(ret);
	}

	return dev;
}
EXPORT_SYMBOL_GPL(iris_vpu_bus_create_device);

static int __init iris_vpu_bus_init(void)
{
	return bus_register(&iris_vpu_bus_type);
}
postcore_initcall(iris_vpu_bus_init);
