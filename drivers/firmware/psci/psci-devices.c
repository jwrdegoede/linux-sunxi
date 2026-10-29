// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/array_size.h>
#include <linux/auxiliary_bus.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct arm_psci_device_info {
	const char *name;
	const char *of_node_name;
};

static const struct arm_psci_device_info arm_psci_devices[] = {
	{
		.name = "psci-cpuidle-domain",
	},
};

static void arm_psci_auxiliary_device_release(struct device *dev)
{
	struct auxiliary_device *auxdev = to_auxiliary_dev(dev);

	of_node_put(dev->of_node);
	kfree(auxdev);
}

static int arm_psci_auxiliary_device_create(struct device *dev,
					    const struct arm_psci_device_info *info)
{
	struct auxiliary_device *auxdev;
	struct device_node *np;
	int ret;

	auxdev = kzalloc_obj(*auxdev);
	if (!auxdev)
		return -ENOMEM;

	if (info->of_node_name)
		np = of_get_child_by_name(dev->of_node, info->of_node_name);
	else
		np = dev->of_node;

	auxdev->id = 0;
	auxdev->name = info->name;
	auxdev->dev.parent = dev;
	auxdev->dev.release = arm_psci_auxiliary_device_release;

	if (np)
		device_set_node(&auxdev->dev, of_fwnode_handle(of_node_get(np)));

	ret = auxiliary_device_init(auxdev);
	if (ret) {
		of_node_put(auxdev->dev.of_node);
		kfree(auxdev);
		goto out_put_node;
	}

	ret = __auxiliary_device_add(auxdev, "arm-psci");
	if (ret) {
		auxiliary_device_uninit(auxdev);
		goto out_put_node;
	}

	ret = devm_add_action_or_reset(dev, auxiliary_device_destroy, auxdev);

out_put_node:
	if (info->of_node_name)
		of_node_put(np);

	return ret;
}

static int arm_psci_probe(struct platform_device *pdev)
{
	const struct arm_psci_device_info *info;
	int auxdev_count = 0;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(arm_psci_devices); i++) {
		info = &arm_psci_devices[i];
		ret = arm_psci_auxiliary_device_create(&pdev->dev, info);
		if (ret)
			dev_warn(&pdev->dev, "failed to create %s aux device: %d\n",
				 info->name, ret);
		else
			auxdev_count++;
	}

	return auxdev_count ? 0 : -ENODEV;
}

static const struct of_device_id arm_psci_of_match[] = {
	{ .compatible = "arm,psci-1.0" },
	{ }
};

static struct platform_driver arm_psci_driver = {
	.probe = arm_psci_probe,
	.driver = {
		.name = "arm-psci",
		.of_match_table = arm_psci_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init arm_psci_init(void)
{
	return platform_driver_register(&arm_psci_driver);
}
arch_initcall(arm_psci_init);
