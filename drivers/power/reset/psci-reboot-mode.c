// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/array_size.h>
#include <linux/auxiliary_bus.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/psci.h>
#include <linux/reboot-mode.h>
#include <linux/slab.h>
#include <linux/types.h>

/*
 * Predefined modes use two arguments. arg1/magic[0] is always zero and
 * arg2/magic[1] is one of the values defined by enum psci_standard_resets.
 */
static const struct reboot_mode_entry psci_resets[] = {
	{
		.name  = "psci-system-reset",
		.magic = { 0, PSCI_SYSTEM_RESET_COLD_RESET },
		.count = 2,
	},
	{
		.name  = "psci-system-reset2-arch-warm-reset",
		.magic = { 0, PSCI_SYSTEM_RESET2_ARCH_WARM_RESET },
		.count = 2,
	},
};

static u64 psci_reboot_mode_get_cookie(const u32 *magic, int count)
{
	u64 cookie = 0;
	int i;

	/*
	 * For a 3-cell magic, arg2 is the high 32 bits and arg3 is the low 32 bits.
	 * For a 2-cell magic, arg2 is the low 32 bits.
	 */
	for (i = 1; i < count; i++)
		cookie = (cookie << 32) | magic[i];

	return cookie;
}

static int psci_reboot_mode_write(struct reboot_mode_driver *reboot,
				  const u32 *magic, u32 count)
{
	if (count < 2 || count > 3)
		return -EINVAL;

	/* Unexpected value at magic[1]/arg2 on a 32-bit system. */
	if (!IS_ENABLED(CONFIG_64BIT) && count == 3 && magic[1])
		return -EINVAL;

	return psci_set_reset_cmd(magic[0],
				  psci_reboot_mode_get_cookie(magic, count));
}

static int psci_reboot_mode_probe(struct auxiliary_device *auxdev,
				  const struct auxiliary_device_id *id)
{
	struct reboot_mode_driver *reboot;
	struct device *dev = &auxdev->dev;
	size_t count;
	int ret;

	reboot = devm_kzalloc(dev, sizeof(*reboot), GFP_KERNEL);
	if (!reboot)
		return -ENOMEM;

	/* Skip PSCI SYSTEM_RESET2 modes if unsupported. */
	count = psci_has_system_reset2_support() ? ARRAY_SIZE(psci_resets) : 1;

	ret = reboot_mode_driver_init(reboot, dev, NULL,
				      psci_reboot_mode_write, psci_resets,
				      count);
	if (ret)
		return ret;

	return devm_reboot_mode_register(dev, reboot);
}

static const struct auxiliary_device_id psci_reboot_mode_id_table[] = {
	{ .name = "arm-psci.psci-reboot-mode" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, psci_reboot_mode_id_table);

static struct auxiliary_driver psci_reboot_mode_driver = {
	.probe = psci_reboot_mode_probe,
	.id_table = psci_reboot_mode_id_table,
};

static int __init psci_reboot_mode_init(void)
{
	return __auxiliary_driver_register(&psci_reboot_mode_driver, THIS_MODULE,
					   "psci-reboot-mode");
}
subsys_initcall(psci_reboot_mode_init);
