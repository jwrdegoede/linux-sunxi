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

/* Detect Gen2 firmware by scanning the blob for:
 *   QC_IMAGE_VERSION_STRING=<version>
 * and then checking:
 *   - version starts with "vfw", OR
 *   - version matches "video-firmware.N.M" with N >= 2
 */

static bool iris_detect_gen2_from_fwdata(const u8 *data, size_t size)
{
	static const char *marker = "QC_IMAGE_VERSION_STRING=";
	const size_t mlen = strlen(marker);
	static const char *vfw = "vfw";
	const size_t vfwlen = strlen(vfw);
	static const char *vf = "video-firmware.";
	const size_t vflen = strlen(vf);

	for (size_t i = 0; i + mlen < size; i++) {
		const char *found;

		if (memcmp(data + i, marker, mlen))
			continue;

		found = data + i + mlen;
		size -= i + mlen;

		/* vfw => Gen2 */
		if (size > vfwlen && !memcmp(found, vfw, vfwlen))
			return true;

		if (size < vflen ||
		    memcmp(found, vf, vflen))
			return false;

		found += vflen;
		size -= vflen;

		/*
		 * video-firmware.1.x is Gen1.
		 * video-firmware.2.x and video-firmware.10.x are Gen2.
		 */
		return size >= 2 &&
			(*found >= '2' || (*found == '1' && found[1] != '.'));
	}

	return false;
}

static const struct firmware *iris_detect_firmware(struct iris_core *core,
						   const char **fw_name)
{
	const struct firmware *firmware;
	bool has_both_gens;
	int ret;

	*fw_name = NULL;
	if (core->iris_platform_data->firmware_desc_gen2)
		core->iris_firmware_desc = core->iris_platform_data->firmware_desc_gen2;
	else if (core->iris_platform_data->firmware_desc_gen1)
		core->iris_firmware_desc = core->iris_platform_data->firmware_desc_gen1;
	else
		return ERR_PTR(-EINVAL);

	has_both_gens = core->iris_platform_data->firmware_desc_gen2 &&
		core->iris_platform_data->firmware_desc_gen1;

	ret = of_property_read_string_index(dev_of_node(core->dev), "firmware-name", 0, fw_name);
	if (ret) {
		*fw_name = core->iris_firmware_desc->fwname;
		ret = request_firmware(&firmware, *fw_name, core->dev);
		if (ret && has_both_gens) {
			core->iris_firmware_desc = core->iris_platform_data->firmware_desc_gen1;
			*fw_name = core->iris_firmware_desc->fwname;
			ret = request_firmware(&firmware, *fw_name, core->dev);
		}

		return ret ? ERR_PTR(ret) : firmware;
	}

	ret = request_firmware(&firmware, *fw_name, core->dev);
	if (ret)
		return ERR_PTR(ret);

	if (has_both_gens &&
	    !iris_detect_gen2_from_fwdata((const u8 *)firmware->data, firmware->size)) {
		dev_info(core->dev, "Gen1 FW detected in %s\n", *fw_name);
		core->iris_firmware_desc = core->iris_platform_data->firmware_desc_gen1;
	}

	return firmware;
}

static int iris_load_fw_to_memory(struct iris_core *core)
{
	struct device *fw_dev = core->fw_dev ? core->fw_dev : core->dev;
	const struct firmware *firmware = NULL;
	struct qcom_pas_context	*ctx;
	struct iommu_domain *domain;
	struct resource res;
	phys_addr_t mem_phys;
	const char *fw_name;
	size_t res_size;
	ssize_t fw_size;
	void *mem_virt;
	int ret;

	ret = of_reserved_mem_region_to_resource(core->dev->of_node, 0, &res);
	if (ret)
		return ret;

	mem_phys = res.start;
	res_size = resource_size(&res);

	firmware = iris_detect_firmware(core, &fw_name);
	if (IS_ERR(firmware))
		return PTR_ERR(firmware);

	core->iris_firmware_data = core->iris_firmware_desc->firmware_data;

	if (!core->pas_ctx) {
		ctx = devm_qcom_pas_context_alloc(core->dev, IRIS_PAS_ID, mem_phys, res_size);
		if (IS_ERR(ctx))
			return PTR_ERR(ctx);
		core->pas_ctx = ctx;
	}

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
	int i, ret;

	ret = iris_load_fw_to_memory(core);
	if (ret) {
		dev_err(core->dev, "firmware download failed %d\n", ret);
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
