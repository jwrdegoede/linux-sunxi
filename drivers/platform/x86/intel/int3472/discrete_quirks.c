// SPDX-License-Identifier: GPL-2.0
/* Author: Hans de Goede <hansg@kernel.org> */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/i2c.h>

#include "common.h"

/*
 * The Terra Pad 1262 v2's \_SB_.PC00.LNK0 DSDT entry has the wrong I2C bus and
 * address, instantiate the i2c-client manually at the right bus and address.
 */
#define TERRAPAD1262V2_LNK0_ACPI_DEVNAME			"INT3474:01"

static int terra_pad_1262_v2_post(struct int3472_discrete_device *int3472)
{
	struct i2c_board_info info = { .addr = 0x21 };
	struct i2c_adapter *adap;	
	struct acpi_device *adev;
	acpi_handle handle;
	acpi_status status;

	status = acpi_get_handle(NULL, "\\_SB_.PC00.LNK0", &handle);
	if (ACPI_FAILURE(status)) {
		dev_err(int3472->dev, "Error could not get _SB_.PC00.LNK0 ACPI handle\n");
		return -EIO;
	}

	adev = acpi_fetch_acpi_dev(handle);
	if (strcmp(TERRAPAD1262V2_LNK0_ACPI_DEVNAME, dev_name(&adev->dev))) {
		dev_err(int3472->dev, "Unexpected LNK0 dev-name: %s\n", dev_name(&adev->dev));
		return -EIO;		
	}

	acpi_set_modalias(adev, dev_name(&adev->dev), info.type, sizeof(info.type));
	info.fwnode = acpi_fwnode_handle(adev);

	status = acpi_get_handle(NULL, "\\_SB_.PC00.I2C1", &handle);
	if (ACPI_FAILURE(status)) {
		dev_err(int3472->dev, "Error could not get _SB_.PC00.I2C1 ACPI handle\n");
		return -EIO;
	}

	adap = i2c_acpi_find_adapter_by_handle(handle);
	if (!adap) {
		dev_err(int3472->dev, "Error could not get I2C1 adapter\n");
		return -EIO;
	}

	adev->power.flags.ignore_parent = true;
	/* Avoid i2c-core-acpi.c enumerating the adev after the INT3472 _DEP is cleared */
	acpi_device_set_enumerated(adev);

	if (IS_ERR(i2c_new_client_device(adap, &info))) {
		adev->power.flags.ignore_parent = false;
		dev_err(int3472->dev, "Error registering i2c-client\n");
		return -EIO;
	}

	dev_info(int3472->dev, "Terra Pad 1262 v2 LNK0 sensor registered\n");
	return 0;
}

static const struct int3472_discrete_quirks lenovo_miix_510_quirks = {
	.regulator_second_sensor = "i2c-OVTI2680:00",
};

static const struct int3472_discrete_quirks terra_pad_1262_v2_quirks = {
	/* LNK1's clk and GPIOs provided by INT3472 UID0 are shared by LNK0 */
	.clk_second_sensor = "i2c-" TERRAPAD1262V2_LNK0_ACPI_DEVNAME,
	.regulator_second_sensor = "i2c-" TERRAPAD1262V2_LNK0_ACPI_DEVNAME,
	.reset_supply_map = "DVDD",
	.powerdown_supply_map = "AVDD",
	/* Skip LNK0's INT3472 UID 1 device it contains bogus info */
	.skip_uid1 = true,
	/* Manually instantiate LNK0 i2c-client to workaround broken I2c bus and address */
	.post = terra_pad_1262_v2_post,
};

const struct dmi_system_id skl_int3472_discrete_quirks[] = {
	{
		/* Lenovo Miix 510-12IKB */
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "LENOVO"),
			DMI_MATCH(DMI_PRODUCT_VERSION, "MIIX 510-12IKB"),
		},
		.driver_data = (void *)&lenovo_miix_510_quirks,
	},
	{
		/* Terra Pad 1262 v2 */
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Wortmann_AG"),
			DMI_MATCH(DMI_PRODUCT_FAMILY, "NB-TERRA;PAD;12,3"),
			DMI_MATCH(DMI_PRODUCT_SKU, "4039407079445"),
		},
		.driver_data = (void *)&terra_pad_1262_v2_quirks,
	},
	{ }
};
