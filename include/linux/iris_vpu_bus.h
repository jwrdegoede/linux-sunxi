/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _LINUX_IRIS_VPU_BUS_H
#define _LINUX_IRIS_VPU_BUS_H

#include <linux/device.h>

#ifdef CONFIG_QCOM_IRIS_VPU_BUS
extern const struct bus_type iris_vpu_bus_type;

struct device *iris_vpu_bus_create_device(struct device *parent_device, const char *name,
					  u64 dma_mask, const u32 *iommu_fid);
#else
static inline struct device *iris_vpu_bus_create_device(struct device *parent_device,
							const char *name, u64 dma_mask,
							const u32 *iommu_fid)
{
	return ERR_PTR(-ENODEV);
}
#endif

#endif /* _LINUX_IRIS_VPU_BUS_H */
