// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Qualcomm Technologies, Inc. and/or its subsidiaries
 */

#include <linux/auxiliary_bus.h>
#include <linux/device.h>
#include <linux/device-id/of.h>
#include <linux/err.h>
#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#define QCE_DEFAULT_MEM_BANDWIDTH 393600

static const char *const qce_core_aux_devs[] = { "crypto", NULL };

static int qce_core_probe(struct platform_device *pdev)
{
	struct auxiliary_device *auxdev;
	struct device *dev = &pdev->dev;
	struct icc_path *mem_path;
	const char *const *subdev;
	int ret;

	mem_path = devm_of_icc_get(&pdev->dev, "memory");
	if (IS_ERR(mem_path))
		return PTR_ERR(mem_path);

	dev_set_drvdata(dev, mem_path);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	PM_RUNTIME_ACQUIRE(dev, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	for (subdev = qce_core_aux_devs; *subdev; subdev++) {
		auxdev = __devm_auxiliary_device_create(dev, "qce", *subdev, NULL, 0);
		if (IS_ERR(auxdev))
			return PTR_ERR(auxdev);
	}

	return 0;
}

static int qce_core_runtime_suspend(struct device *dev)
{
	struct icc_path *mem_path = dev_get_drvdata(dev);

	return icc_set_bw(mem_path, 0, 0);
}

static int qce_core_runtime_resume(struct device *dev)
{
	struct icc_path *mem_path = dev_get_drvdata(dev);

	return icc_set_bw(mem_path, QCE_DEFAULT_MEM_BANDWIDTH,
			  QCE_DEFAULT_MEM_BANDWIDTH);
}

static const struct dev_pm_ops qce_core_pm_ops = {
	RUNTIME_PM_OPS(qce_core_runtime_suspend, qce_core_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static const struct of_device_id qce_core_of_match[] = {
	{ .compatible = "qcom,crypto-v5.1" },
	{ .compatible = "qcom,crypto-v5.4" },
	{ .compatible = "qcom,qce" },
	{ }
};
MODULE_DEVICE_TABLE(of, qce_core_of_match);

static struct platform_driver qce_core_driver = {
	.probe = qce_core_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = qce_core_of_match,
		.pm = pm_ptr(&qce_core_pm_ops),
	},
};
module_platform_driver(qce_core_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm crypto engine core driver");
MODULE_ALIAS("platform:" KBUILD_MODNAME);
MODULE_AUTHOR("Bartosz Golaszewski <bartosz.golaszewski@oss.qualcomm.com>");
