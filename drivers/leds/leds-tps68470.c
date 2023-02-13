// SPDX-License-Identifier: GPL-2.0
/*
 * LED driver for TPS68470 PMIC
 *
 * Copyright (C) 2023 Red Hat
 *
 * Authors:
 *	Kate Hsuan <hpa@redhat.com>
 */

#include <linux/gpio/driver.h>
#include <linux/mfd/tps68470.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/leds.h>

struct tps68470_led_data {
	struct regmap *tps68470_regmap;
	unsigned int brightness_a;
	unsigned int brightness_b;
	struct led_classdev leda_cdev;
	struct led_classdev ledb_cdev;
};

enum ctrlb_current {
	CTRLB_2MA	= 0,
	CTRLB_4MA	= 1,
	CTRLB_8MA	= 2,
	CTRLB_16MA	= 3,
};

static int set_ledb_current(struct regmap *regmap,
			    unsigned int *data_brightness,
			    enum led_brightness brightness)
{
	unsigned int ledb_current;

	switch (brightness) {
	case LED_HALF:
		ledb_current = CTRLB_8MA;
		break;
	case LED_FULL:
		ledb_current = CTRLB_16MA;
		break;
	case LED_ON:
		ledb_current = CTRLB_4MA;
		break;
	case LED_OFF:
		ledb_current = CTRLB_2MA;
		break;
	default:
		return -EINVAL;
	}

	*data_brightness = brightness;
	return regmap_update_bits(regmap, TPS68470_REG_ILEDCTL,
				  TPS68470_ILEDCTL_CTRLB, ledb_current);
}

static int tps68470_brightness_set(struct led_classdev *led_cdev,
				   enum led_brightness brightness)
{
	struct tps68470_led_data *data;
	struct regmap *regmap;
	unsigned int mask;
	unsigned int value;
	int ret;

	if (!strncmp(led_cdev->name, "tps68470-ileda", 14)) {
		data = container_of(led_cdev, struct tps68470_led_data, leda_cdev);
		regmap = data->tps68470_regmap;
		data->brightness_a = brightness ? TPS68470_ILEDCTL_ENA : 0;
		mask = TPS68470_ILEDCTL_ENA;
		value = data->brightness_a;
	} else if (!strncmp(led_cdev->name, "tps68470-iledb", 14)) {
		data = container_of(led_cdev, struct tps68470_led_data, ledb_cdev);
		regmap = data->tps68470_regmap;
		mask = TPS68470_ILEDCTL_ENB;
		value = brightness ? TPS68470_ILEDCTL_ENB : 0;
		/* Set current state for ledb */
		ret = set_ledb_current(regmap, &data->brightness_b, brightness);
		if (ret)
			goto err_exit;
	} else
		return -EINVAL;

	ret = regmap_update_bits(regmap, TPS68470_REG_ILEDCTL, mask, value);

err_exit:
	return ret;
}

static enum led_brightness tps68470_brightness_get(struct led_classdev *led_cdev)
{
	struct tps68470_led_data *data = container_of(led_cdev,
						      struct tps68470_led_data,
						      ledb_cdev);

	if (!strncmp(led_cdev->name, "tps68470-ileda", 14))
		return data->brightness_a;
	else if (!strncmp(led_cdev->name, "tps68470-iledb", 14))
		return data->brightness_b;

	return -EINVAL;
}

static int tps68470_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct tps68470_led_data *tps68470_led;

	tps68470_led = devm_kzalloc(&pdev->dev, sizeof(struct tps68470_led_data),
				    GFP_KERNEL);
	if (!tps68470_led)
		return -ENOMEM;

	tps68470_led->tps68470_regmap = dev_get_drvdata(pdev->dev.parent);
	tps68470_led->leda_cdev.name = "tps68470-ileda";
	tps68470_led->leda_cdev.max_brightness = 1;
	tps68470_led->leda_cdev.brightness_set_blocking = tps68470_brightness_set;
	tps68470_led->leda_cdev.brightness_get = tps68470_brightness_get;
	tps68470_led->leda_cdev.dev = &pdev->dev;
	tps68470_led->brightness_a = 0;
	ret = led_classdev_register(&pdev->dev, &tps68470_led->leda_cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register LEDA: %d\n", ret);
		return ret;
	}

	tps68470_led->tps68470_regmap = dev_get_drvdata(pdev->dev.parent);
	tps68470_led->ledb_cdev.name = "tps68470-iledb";
	tps68470_led->ledb_cdev.max_brightness = 255;
	tps68470_led->ledb_cdev.brightness_set_blocking = tps68470_brightness_set;
	tps68470_led->ledb_cdev.brightness_get = tps68470_brightness_get;
	tps68470_led->ledb_cdev.dev = &pdev->dev;
	tps68470_led->brightness_b = 0;
	ret = led_classdev_register(&pdev->dev, &tps68470_led->ledb_cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register LEDB: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, tps68470_led);

	return ret;
}

static int tps68470_led_remove(struct platform_device *pdev)
{
	struct tps68470_led_data *data = platform_get_drvdata(pdev);

	led_classdev_unregister(&data->leda_cdev);
	led_classdev_unregister(&data->ledb_cdev);

	return 0;
}

static struct platform_driver tps68470_led_driver = {
	.driver = {
		   .name = "tps68470-led",
	},
	.probe = tps68470_led_probe,
	.remove = tps68470_led_remove,
};
module_platform_driver(tps68470_led_driver);

MODULE_ALIAS("platform:tps68470-led");
MODULE_DESCRIPTION("LED driver for TPS68470 PMIC");
MODULE_LICENSE("GPL v2");
