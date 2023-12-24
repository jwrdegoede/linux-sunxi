// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  dell-smo8800.c - Dell Latitude ACPI SMO88XX freefall sensor driver
 *
 *  Copyright (C) 2012 Sonal Santan <sonal.santan@gmail.com>
 *  Copyright (C) 2014 Pali Rohár <pali@kernel.org>
 *  Copyright (C) 2023 Hans de Goede <hansg@kernel.org>
 *
 *  This is loosely based on lis3lv02d driver.
 */

#define DRIVER_NAME "smo8800"

#include <linux/device/bus.h>
#include <linux/dmi.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

struct smo8800_device {
	u32 irq;                     /* acpi device irq */
	atomic_t counter;            /* count after last read */
	struct miscdevice miscdev;   /* for /dev/freefall */
	unsigned long misc_opened;   /* whether the device is open */
	wait_queue_head_t misc_wait; /* Wait queue for the misc dev */
	struct device *dev;          /* acpi device */
	struct i2c_client *i2c_dev;  /* i2c_client for lis3lv02d */
};

static irqreturn_t smo8800_interrupt_quick(int irq, void *data)
{
	struct smo8800_device *smo8800 = data;

	atomic_inc(&smo8800->counter);
	wake_up_interruptible(&smo8800->misc_wait);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t smo8800_interrupt_thread(int irq, void *data)
{
	struct smo8800_device *smo8800 = data;

	dev_info(smo8800->dev, "detected free fall\n");
	return IRQ_HANDLED;
}

static ssize_t smo8800_misc_read(struct file *file, char __user *buf,
				 size_t count, loff_t *pos)
{
	struct smo8800_device *smo8800 = container_of(file->private_data,
					 struct smo8800_device, miscdev);

	u32 data = 0;
	unsigned char byte_data;
	ssize_t retval = 1;

	if (count < 1)
		return -EINVAL;

	atomic_set(&smo8800->counter, 0);
	retval = wait_event_interruptible(smo8800->misc_wait,
				(data = atomic_xchg(&smo8800->counter, 0)));

	if (retval)
		return retval;

	retval = 1;

	byte_data = min_t(u32, data, 255);

	if (put_user(byte_data, buf))
		retval = -EFAULT;

	return retval;
}

static int smo8800_misc_open(struct inode *inode, struct file *file)
{
	struct smo8800_device *smo8800 = container_of(file->private_data,
					 struct smo8800_device, miscdev);

	if (test_and_set_bit(0, &smo8800->misc_opened))
		return -EBUSY; /* already open */

	atomic_set(&smo8800->counter, 0);
	return 0;
}

static int smo8800_misc_release(struct inode *inode, struct file *file)
{
	struct smo8800_device *smo8800 = container_of(file->private_data,
					 struct smo8800_device, miscdev);

	clear_bit(0, &smo8800->misc_opened); /* release the device */
	return 0;
}

static const struct file_operations smo8800_misc_fops = {
	.owner = THIS_MODULE,
	.read = smo8800_misc_read,
	.open = smo8800_misc_open,
	.release = smo8800_misc_release,
};

/*
 * On 2 older PCH generations, Patsburg (for Sandy Bridge and Ivybridge) and
 * Wellsburg (for Haswell and Broadwell), the PCH has 3 extra i2c-i801
 * compatible SMBusses called 'Integrated Device Function' busses. These have
 * FEATURE_IDF set in the i801_ids[] table in i2c-i801.c.
 * The ST microelectronics accelerometer is connected to the main SMBus
 * so the IDF controllers should be skipped.
 */
static const struct pci_device_id i801_idf_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x1d70) }, /* Patsburg IFD0 */
	{ PCI_VDEVICE(INTEL, 0x1d71) }, /* Patsburg IFD1 */
	{ PCI_VDEVICE(INTEL, 0x1d72) }, /* Patsburg IFD2 */
	{ PCI_VDEVICE(INTEL, 0x8d7d) }, /* Wellsburg MS0 */
	{ PCI_VDEVICE(INTEL, 0x8d7e) }, /* Wellsburg MS1 */
	{ PCI_VDEVICE(INTEL, 0x8d7f) }, /* Wellsburg MS2 */
	{}
};

static int smo8800_find_i801(struct device *dev, void *data)
{
	struct i2c_adapter *adap, **adap_ret = data;

	adap = i2c_verify_adapter(dev);
	if (!adap)
		return 0;

	if (!strstarts(adap->name, "SMBus I801 adapter"))
		return 0;

	if (pci_match_id(i801_idf_ids, to_pci_dev(adap->dev.parent)))
		return 0; /* Only register client on main SMBus channel */

	*adap_ret = i2c_get_adapter(adap->nr);
	return 1;
}

/*
 * Accelerometer's I2C address is not specified in DMI nor ACPI,
 * so it is needed to define mapping table based on DMI product names.
 */
static const struct {
	const char *dmi_product_name;
	unsigned short i2c_addr;
} dell_lis3lv02d_devices[] = {
	/*
	 * Dell platform team told us that these Latitude devices have
	 * ST microelectronics accelerometer at I2C address 0x29.
	 */
	{ "Latitude E5250",     0x29 },
	{ "Latitude E5450",     0x29 },
	{ "Latitude E5550",     0x29 },
	{ "Latitude E6440",     0x29 },
	{ "Latitude E6440 ATG", 0x29 },
	{ "Latitude E6540",     0x29 },
	/*
	 * Additional individual entries were added after verification.
	 */
	{ "Latitude 5480",      0x29 },
	{ "Precision 3540",     0x29 },
	{ "Vostro V131",        0x1d },
	{ "Vostro 5568",        0x29 },
	{ "XPS 15 7590",        0x29 },
};

static void smo8800_instantiate_i2c_client(struct smo8800_device *smo8800)
{
	struct i2c_board_info info = { };
	struct i2c_adapter *adap = NULL;
	const char *dmi_product_name;
	u8 addr = 0;
	int i;

	bus_for_each_dev(&i2c_bus_type, NULL, &adap, smo8800_find_i801);
	if (!adap)
		return;

	dmi_product_name = dmi_get_system_info(DMI_PRODUCT_NAME);
	for (i = 0; i < ARRAY_SIZE(dell_lis3lv02d_devices); ++i) {
		if (strcmp(dmi_product_name,
			   dell_lis3lv02d_devices[i].dmi_product_name) == 0) {
			addr = dell_lis3lv02d_devices[i].i2c_addr;
			break;
		}
	}

	if (!addr) {
		dev_warn(smo8800->dev,
			 "Accelerometer lis3lv02d is present on SMBus but its address is unknown, skipping registration\n");
		goto put_adapter;
	}

	info.addr = addr;
	info.irq = smo8800->irq;
	strscpy(info.type, "lis3lv02d", I2C_NAME_SIZE);

	smo8800->i2c_dev = i2c_new_client_device(adap, &info);
	if (IS_ERR(smo8800->i2c_dev)) {
		dev_err_probe(smo8800->dev, PTR_ERR(smo8800->i2c_dev),
			      "registering accel i2c_client\n");
		smo8800->i2c_dev = NULL;
	} else {
		dev_info(smo8800->dev, "Registered %s accelerometer on address 0x%02x\n",
			 info.type, info.addr);
	}
put_adapter:
	i2c_put_adapter(adap);
}

static int smo8800_probe(struct platform_device *device)
{
	int err;
	struct smo8800_device *smo8800;

	smo8800 = devm_kzalloc(&device->dev, sizeof(*smo8800), GFP_KERNEL);
	if (!smo8800) {
		dev_err(&device->dev, "failed to allocate device data\n");
		return -ENOMEM;
	}

	smo8800->dev = &device->dev;
	smo8800->miscdev.minor = MISC_DYNAMIC_MINOR;
	smo8800->miscdev.name = "freefall";
	smo8800->miscdev.fops = &smo8800_misc_fops;

	init_waitqueue_head(&smo8800->misc_wait);

	err = platform_get_irq(device, 0);
	if (err < 0)
		return err;
	smo8800->irq = err;

	smo8800_instantiate_i2c_client(smo8800);

	/* smo8800->irq is passed to the i2c_client and its driver will take care of this */
	if (!smo8800->i2c_dev) {
		err = misc_register(&smo8800->miscdev);
		if (err) {
			dev_err(&device->dev, "failed to register misc dev: %d\n", err);
			goto error_unregister_i2c_client;
		}

		err = request_threaded_irq(smo8800->irq, smo8800_interrupt_quick,
					   smo8800_interrupt_thread,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   DRIVER_NAME, smo8800);
		if (err) {
			dev_err(&device->dev,
				"failed to request thread for IRQ %d: %d\n",
				smo8800->irq, err);
			goto error;
		}
	}

	dev_dbg(&device->dev, "device /dev/freefall registered with IRQ %d\n",
		 smo8800->irq);
	platform_set_drvdata(device, smo8800);
	return 0;

error:
	misc_deregister(&smo8800->miscdev);
error_unregister_i2c_client:
	i2c_unregister_device(smo8800->i2c_dev);
	return err;
}

static void smo8800_remove(struct platform_device *device)
{
	struct smo8800_device *smo8800 = platform_get_drvdata(device);

	if (!smo8800->i2c_dev) {
		free_irq(smo8800->irq, smo8800);
		misc_deregister(&smo8800->miscdev);
		dev_dbg(&device->dev, "device /dev/freefall unregistered\n");
	}
	i2c_unregister_device(smo8800->i2c_dev);
}

static const struct acpi_device_id smo8800_ids[] = {
	{ "SMO8800", 0 },
	{ "SMO8801", 0 },
	{ "SMO8810", 0 },
	{ "SMO8811", 0 },
	{ "SMO8820", 0 },
	{ "SMO8821", 0 },
	{ "SMO8830", 0 },
	{ "SMO8831", 0 },
	{ "", 0 },
};
MODULE_DEVICE_TABLE(acpi, smo8800_ids);

static struct platform_driver smo8800_driver = {
	.probe = smo8800_probe,
	.remove_new = smo8800_remove,
	.driver = {
		.name = DRIVER_NAME,
		.acpi_match_table = smo8800_ids,
	},
};
module_platform_driver(smo8800_driver);

MODULE_DESCRIPTION("Dell Latitude freefall driver (ACPI SMO88XX)");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sonal Santan, Pali Rohár");
/* Ensure the i2c-801 driver is loaded for i2c_client instantiation */
MODULE_SOFTDEP("pre: i2c-i801");
