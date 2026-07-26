// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/firmware.h>
#include <linux/firmware/qcom/qcom_pas.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/iommu.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/soc/qcom/mdt_loader.h>

#include "iris_core.h"
#include "iris_firmware.h"

#define IRIS_PAS_ID				9

#define MAX_FIRMWARE_NAME_SIZE	128
#define IRIS_FW_START_ADDR	0

static int iris_load_fw_to_memory(struct iris_core *core, const char *fw_name)
{
	struct device *fw_dev = core->fw_dev ? core->fw_dev : core->dev;
	const struct firmware *firmware = NULL;
	struct qcom_pas_context	*ctx;
	struct iommu_domain *domain;
	struct resource res;
	phys_addr_t mem_phys;
	size_t res_size;
	ssize_t fw_size;
	void *mem_virt;
	int ret;

	if (strlen(fw_name) >= MAX_FIRMWARE_NAME_SIZE - 4)
		return -EINVAL;

	ret = of_reserved_mem_region_to_resource(core->dev->of_node, 0, &res);
	if (ret)
		return ret;

	mem_phys = res.start;
	res_size = resource_size(&res);

	if (!core->pas_ctx) {
		ctx = devm_qcom_pas_context_alloc(core->dev, IRIS_PAS_ID, mem_phys, res_size);
		if (IS_ERR(ctx))
			return PTR_ERR(ctx);
		core->pas_ctx = ctx;
	}

	ret = request_firmware(&firmware, fw_name, fw_dev);
	if (ret)
		return ret;

	fw_size = qcom_mdt_get_size(firmware);
	if (fw_size < 0 || res_size < (size_t)fw_size) {
		ret = -EINVAL;
		goto err_release_fw;
	}

	mem_virt = memremap(mem_phys, res_size, MEMREMAP_WC);
	if (!mem_virt) {
		ret = -ENOMEM;
		goto err_release_fw;
	}

	core->pas_ctx->use_tzmem = !!core->fw_dev;
	ret = qcom_mdt_pas_load(core->pas_ctx, firmware, fw_name, mem_virt, NULL);
	if (ret)
		goto err_mem_unmap;

	if (core->pas_ctx->use_tzmem) {
		domain = iommu_get_domain_for_dev(fw_dev);
		if (!domain) {
			ret = -ENODEV;
			goto err_mem_unmap;
		}

		ret = iommu_map(domain, IRIS_FW_START_ADDR, mem_phys, res_size,
				IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV, GFP_KERNEL);
	}

err_mem_unmap:
	memunmap(mem_virt);
err_release_fw:
	release_firmware(firmware);

	return ret;
}

static void iris_fw_iommu_unmap(struct iris_core *core)
{
	struct iommu_domain *domain;

	if (!core->fw_dev)
		return;

	domain = iommu_get_domain_for_dev(core->fw_dev);
	if (domain)
		iommu_unmap(domain, IRIS_FW_START_ADDR, core->pas_ctx->mem_size);
}

int iris_fw_load(struct iris_core *core)
{
	const struct tz_cp_config *cp_config;
	const char *fwpath = NULL;
	int i, ret;

	ret = of_property_read_string_index(core->dev->of_node, "firmware-name", 0,
					    &fwpath);
	if (ret)
		fwpath = core->iris_firmware_desc->fwname;

	ret = iris_load_fw_to_memory(core, fwpath);
	if (ret) {
		dev_err(core->dev, "firmware download failed\n");
		return ret;
	}

	ret = qcom_pas_prepare_and_auth_reset(core->pas_ctx);
	if (ret)  {
		dev_err(core->dev, "auth and reset failed: %d\n", ret);
		goto err_unmap;
	}

	for (i = 0; i < core->iris_platform_data->tz_cp_config_data_size; i++) {
		cp_config = &core->iris_platform_data->tz_cp_config_data[i];
		ret = qcom_scm_mem_protect_video_var(cp_config->cp_start,
						     cp_config->cp_size,
						     cp_config->cp_nonpixel_start,
						     cp_config->cp_nonpixel_size);
		if (ret) {
			dev_err(core->dev, "qcom_scm_mem_protect_video_var failed: %d\n", ret);
			goto err_pas_shutdown;
		}
	}

	return 0;

err_pas_shutdown:
	qcom_pas_shutdown(IRIS_PAS_ID);
err_unmap:
	iris_fw_iommu_unmap(core);

	return ret;
}

int iris_fw_unload(struct iris_core *core)
{
	int ret;

	ret = qcom_pas_shutdown(IRIS_PAS_ID);
	iris_fw_iommu_unmap(core);

	return ret;
}

int iris_set_hw_state(struct iris_core *core, bool resume)
{
	return qcom_pas_set_remote_state(resume, 0);
}
