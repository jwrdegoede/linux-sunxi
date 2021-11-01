// SPDX-License-Identifier: GPL-2.0+
/*
 * On the Xiaomi Mi Pad 2 X86 tablet a bunch of devices are hidden when
 * the EFI if the tablet does thinks the OS it is booting is Windows
 * (OSID in the DSDT is set to 1); and the EFI code thinks this as soon
 * as the EFI bootloader is not Xiaomi's own signed Android loader :|
 *
 * This "board-file" takes care of instantiating the hidden devices manually.
 *
 * Copyright (C) 2021 Hans de Goede <hdegoede@redhat.com>
 */

#define pr_fmt(fmt) "xiaomi-mipad2: " fmt

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>

/********** BQ27520 fuel-gauge info **********/
#define BQ27520_ADAPTER "\\_SB_.PCI0.I2C1"

static const char * const bq27520_suppliers[] = { "bq25890-charger" };

static const struct property_entry bq27520_props[] = {
	PROPERTY_ENTRY_STRING_ARRAY("supplied-from", bq27520_suppliers),
	{ }
};

static const struct software_node bq27520_node = {
	.properties = bq27520_props,
};

static const struct i2c_board_info bq27520_board_info = {
	.type = "bq27520",
	.addr = 0x55,
	.dev_name = "bq27520",
	.swnode = &bq27520_node,
};

static struct i2c_client *bq27520_client;

/********** KTD2026 RGB notification LED controller **********/
#define KTD2026_ADAPTER "\\_SB_.PCI0.I2C3"

static const struct i2c_board_info ktd2026_board_info = {
	.type = "ktd2026",
	.addr = 0x30,
	.dev_name = "ktd2026",
};

static struct i2c_client *ktd2026_client;

/********** DMI-match, probe(), etc. **********/
static const struct dmi_system_id xiaomi_mipad2_ids[] __initconst = {
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Xiaomi Inc"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Mipad2"),
		},
	},
	{} /* Terminating entry */
};
MODULE_DEVICE_TABLE(dmi, xiaomi_mipad2_ids);

static __init struct i2c_client *xiaomi_instantiate_i2c_client(
				char *adapter_path,
				const struct i2c_board_info *board_info)
{
	struct i2c_client *client;
	struct i2c_adapter *adap;
	acpi_handle handle;
	acpi_status status;

	status = acpi_get_handle(NULL, adapter_path, &handle);
	if (ACPI_FAILURE(status)) {
		pr_err("Error could not get %s handle\n", adapter_path);
		return ERR_PTR(-ENODEV);
	}

	adap = i2c_acpi_find_adapter_by_handle(handle);
	if (!adap) {
		pr_err("Error could not get %s adapter\n", adapter_path);
		return ERR_PTR(-ENODEV);
	}

	client = i2c_new_client_device(adap, board_info);

	put_device(&adap->dev);

	return client;
}

static __init int xiaomi_mipad2_init(void)
{
	if (!dmi_first_match(xiaomi_mipad2_ids))
		return -ENODEV;

	bq27520_client = xiaomi_instantiate_i2c_client(BQ27520_ADAPTER,
						       &bq27520_board_info);
	if (IS_ERR(bq27520_client))
		return PTR_ERR(bq27520_client);

	ktd2026_client = xiaomi_instantiate_i2c_client(KTD2026_ADAPTER,
						       &ktd2026_board_info);
	if (IS_ERR(ktd2026_client)) {
		i2c_unregister_device(bq27520_client);
		return PTR_ERR(ktd2026_client);
	}

	return 0;
}

static __exit void xiaomi_mipad2_cleanup(void)
{
	i2c_unregister_device(ktd2026_client);
	i2c_unregister_device(bq27520_client);
}

module_init(xiaomi_mipad2_init);
module_exit(xiaomi_mipad2_cleanup);

MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com");
MODULE_DESCRIPTION("Xiaomi Mi Pad 2 board-file");
MODULE_LICENSE("GPL");
