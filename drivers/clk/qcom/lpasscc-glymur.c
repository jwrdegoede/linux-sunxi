// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/qcom,glymur-lpasscc.h>

#include "common.h"
#include "reset.h"

static const struct qcom_reset_map lpass_audiocc_glymur_resets[] = {
	[LPASS_AUDIO_SWR_RX_CGCR] = { 0xa0, 1 },
	[LPASS_AUDIO_SWR_WSA1_CGCR] = { 0xb0, 1 },
	[LPASS_AUDIO_SWR_WSA2_CGCR] = { 0xd8, 1 },
	[LPASS_AUDIO_SWR_WSA3_CGCR] = { 0x300c, 1 },
	[LPASS_AUDIO_SWR_WSA4_CGCR] = { 0x301c, 1 },
};

static const struct regmap_config lpass_audiocc_glymur_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.name = "lpass-audio-csr",
	.max_register = 0x3020,
};

static const struct qcom_cc_desc lpass_audiocc_glymur_reset_desc = {
	.config = &lpass_audiocc_glymur_regmap_config,
	.resets = lpass_audiocc_glymur_resets,
	.num_resets = ARRAY_SIZE(lpass_audiocc_glymur_resets),
};

static const struct of_device_id lpasscc_glymur_match_table[] = {
	{
		.compatible = "qcom,glymur-lpassaudiocc",
		.data = &lpass_audiocc_glymur_reset_desc,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, lpasscc_glymur_match_table);

static int lpasscc_glymur_probe(struct platform_device *pdev)
{
	const struct qcom_cc_desc *desc = device_get_match_data(&pdev->dev);

	return qcom_cc_probe_by_index(pdev, 0, desc);
}

static struct platform_driver lpasscc_glymur_driver = {
	.probe = lpasscc_glymur_probe,
	.driver = {
		.name = "lpasscc-glymur",
		.of_match_table = lpasscc_glymur_match_table,
	},
};

module_platform_driver(lpasscc_glymur_driver);

MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@oss.qualcomm.com>");
MODULE_DESCRIPTION("QTI LPASSCC Glymur Driver");
MODULE_LICENSE("GPL");
