// SPDX-License-Identifier: GPL-2.0
/* WMI driver for Lenovo Yoga Book YB1-X91* tablets */

#include <linux/acpi.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/wmi.h>
#include <linux/pm_runtime.h>
#include <linux/leds.h>

#include <uapi/linux/input-event-codes.h>

#define YB_MBTN_EVENT_GUID	"243FEC1D-1963-41C1-8100-06A9D82A94B4"
#define YB_MBTN_METHOD_GUID	"742B0CA1-0B20-404B-9CAA-AEFCABF30CE0"

#define YB_PAD_ENABLE	1
#define YB_PAD_DISABLE	2
#define YB_LIGHTUP_BTN	3

#define YB_KBD_BL_DEFAULT 128

struct yogabook_wmi {
	struct wmi_device *wdev;
	struct input_dev *input_dev;
	struct acpi_device *tp_adev;
	struct acpi_device *dig_adev;
	struct device_link *pwm_tp_link;
	struct led_classdev kbd_bl_led;
	bool digitizer_mode;
	bool suspended;
	uint8_t brightness;
};

static int yogabook_wmi_do_action(struct wmi_device *wdev, int action)
{
	struct acpi_buffer output = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_buffer input;
	acpi_status status;
	u32 dummy_arg = 0;

	dev_dbg(&wdev->dev, "Do action: %d\n", action);

	input.pointer = &dummy_arg;
	input.length = sizeof(dummy_arg);

	status = wmi_evaluate_method(YB_MBTN_METHOD_GUID, 0, action, &input,
				     &output);
	if (ACPI_FAILURE(status)) {
		dev_err(&wdev->dev, "Calling WMI method failure: 0x%x\n",
			status);
		return status;
	}

	kfree(output.pointer);

	return 0;
}

/**
 * To control keyboard backlight, call the method KBLC() of the TCS1 ACPI
 * device (Goodix touchpad acts as virtual sensor keyboard).
 * To get this method working independently of touchpad driver and device state,
 * power on the device first.
 */
static int yogabook_wmi_set_kbd_backlight(struct wmi_device *wdev,
					  uint8_t level)
{
	struct yogabook_wmi *data = dev_get_drvdata(&wdev->dev);
	union acpi_object param;
	struct acpi_object_list input;
	struct acpi_buffer output = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_status status;
	int r;

	dev_dbg(&wdev->dev, "Set KBLC level to %u. TP adev power state = %d\n",
		level, data->tp_adev->power.state);

	r = acpi_device_set_power(data->tp_adev, ACPI_STATE_D0);
	if (r)
		dev_err(&wdev->dev,
			  "Failed to power on the touchpad: %d\n", r);

	input.count = 1;
	input.pointer = &param;

	param.type = ACPI_TYPE_INTEGER;
	param.integer.value = 255 - level;

	status = acpi_evaluate_object(acpi_device_handle(data->tp_adev), "KBLC",
				      &input, &output);
	if (ACPI_FAILURE(status)) {
		dev_err(&wdev->dev, "Failed to call KBLC method: 0x%x\n",
			status);
		return status;
	}

	kfree(output.pointer);

	return 0;
}

static void yogabook_wmi_enable_keyboard(struct wmi_device *wdev)
{
	struct yogabook_wmi *data = dev_get_drvdata(&wdev->dev);

	yogabook_wmi_set_kbd_backlight(wdev, data->brightness);
	yogabook_wmi_do_action(wdev, YB_PAD_DISABLE);
}

static void yogabook_wmi_disable_keyboard(struct wmi_device *wdev)
{
	yogabook_wmi_set_kbd_backlight(wdev, 0);
	yogabook_wmi_do_action(wdev, YB_PAD_ENABLE);
}


static void yogabook_wmi_notify(struct wmi_device *wdev, union acpi_object *dummy)
{
	struct yogabook_wmi *data;
	unsigned int key;

	if (wdev == NULL)
		return;

	data = dev_get_drvdata(&wdev->dev);
	if (data == NULL)
		return;

	if (data->suspended)
		return;

	data->digitizer_mode = !data->digitizer_mode;

	if (data->digitizer_mode)
		yogabook_wmi_disable_keyboard(wdev);
	else
		yogabook_wmi_enable_keyboard(wdev);

	key = data->digitizer_mode ? KEY_TOUCHPAD_ON : KEY_TOUCHPAD_OFF;

	input_report_key(data->input_dev, key, 1);
	input_sync(data->input_dev);
	input_report_key(data->input_dev, key, 0);
	input_sync(data->input_dev);
}

static enum led_brightness kbd_brightness_get(struct led_classdev *cdev)
{
	struct yogabook_wmi *data =
		container_of(cdev, struct yogabook_wmi, kbd_bl_led);

	return data->brightness;
}

static int kbd_brightness_set(struct led_classdev *cdev,
				  enum led_brightness value)
{
	struct yogabook_wmi *data =
		container_of(cdev, struct yogabook_wmi, kbd_bl_led);
	struct wmi_device *wdev = data->wdev;

	if ((value < 0) || (value > 255))
		return -EINVAL;

	data->brightness = value;

	if (!data->digitizer_mode)
		yogabook_wmi_set_kbd_backlight(wdev, data->brightness);

	return 0;
}

static int yogabook_wmi_probe(struct wmi_device *wdev, const void *context)
{
	struct yogabook_wmi *data;
	int r;

	data = devm_kzalloc(&wdev->dev, sizeof(struct yogabook_wmi), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	dev_set_drvdata(&wdev->dev, data);

	data->wdev = wdev;
	data->digitizer_mode = false;
	data->suspended = false;
	data->brightness = YB_KBD_BL_DEFAULT;

	data->tp_adev = acpi_dev_get_first_match_dev("GDIX1001", NULL, -1);
	if (!data->tp_adev) {
		dev_err(&wdev->dev,
			"Cannot find the touchpad device in ACPI tables\n");
		return -ENODEV;
	}

	data->dig_adev = acpi_dev_get_first_match_dev("WCOM0019", NULL, -1);
	if (!data->dig_adev) {
		dev_err(&wdev->dev,
			"Cannot find the digitizer device in ACPI tables\n");
		r = -ENODEV;
		goto error_get_dig_adev;
	}
/*
	data->pwm_tp_link = device_link_add(&wdev->dev, &data->tp_adev->dev, DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!data->pwm_tp_link) {
		dev_err(&wdev->dev,
			"Failed to add device link PWM2->TCS1: \n");
		r = -EINVAL;
		goto error_device_link_add;
	}

	dev_dbg(&wdev->dev, "device link status: %d\n", data->pwm_tp_link->status);
*/

	yogabook_wmi_enable_keyboard(wdev);

	data->input_dev = devm_input_allocate_device(&wdev->dev);
	if (data->input_dev == NULL) {
		acpi_dev_put(data->dig_adev);
		acpi_dev_put(data->tp_adev);
		return -ENOMEM;
	}

	data->input_dev->name = "Yoga Book WMI keys";
	data->input_dev->phys = "wmi/input0";

	set_bit(EV_KEY, data->input_dev->evbit);
	set_bit(KEY_TOUCHPAD_ON, data->input_dev->keybit);
	set_bit(KEY_TOUCHPAD_OFF, data->input_dev->keybit);

	r = input_register_device(data->input_dev);
	if (r)
		goto error_register_inputdev;

	data->kbd_bl_led.name = "ybwmi::kbd_backlight";
	data->kbd_bl_led.brightness_set_blocking = kbd_brightness_set;
	data->kbd_bl_led.brightness_get = kbd_brightness_get;
	data->kbd_bl_led.max_brightness = 255;

	r = devm_led_classdev_register(&wdev->dev, &data->kbd_bl_led);
	if (r < 0) {
		dev_err(&wdev->dev,
			"Cannot register LED device for backlight: %d\n", r);
		goto error_register_inputdev;
	}

	return 0;

error_register_inputdev:
//	device_link_del(data->pwm_tp_link);
//error_device_link_add:
	acpi_dev_put(data->dig_adev);
error_get_dig_adev:
	acpi_dev_put(data->tp_adev);

	return r;
}

static void yogabook_wmi_remove(struct wmi_device *wdev)
{
	struct yogabook_wmi *data = dev_get_drvdata(&wdev->dev);

//	device_link_del(data->pwm_tp_link);
	acpi_dev_put(data->dig_adev);
	acpi_dev_put(data->tp_adev);
}

#ifdef CONFIG_PM
int yogabook_wmi_suspend(struct device *dev)
{
	struct wmi_device *wdev = container_of(dev, struct wmi_device, dev);
	struct yogabook_wmi *data = dev_get_drvdata(dev);

	dev_dbg(dev, "SUSPEND");

	data->suspended = true;

	/* Turn off the pen button at sleep */
	if (data->digitizer_mode)
		yogabook_wmi_do_action(wdev, YB_PAD_DISABLE);

	return 0;
}

int yogabook_wmi_resume(struct device *dev)
{
	struct wmi_device *wdev = container_of(dev, struct wmi_device, dev);
	struct yogabook_wmi *data = dev_get_drvdata(dev);

	dev_dbg(dev, "RESUME, enable touchpad: %d", data->digitizer_mode);
	data->suspended = false;

	if (data->digitizer_mode)
		yogabook_wmi_disable_keyboard(wdev);
	else
		yogabook_wmi_enable_keyboard(wdev);

	return 0;
}

#endif

static const struct wmi_device_id yogabook_wmi_id_table[] = {
	{
		.guid_string = YB_MBTN_EVENT_GUID,
	},
	{ } /* Terminating entry */
};

static SIMPLE_DEV_PM_OPS(yogabook_wmi_pm_ops,
			 yogabook_wmi_suspend, yogabook_wmi_resume);

static struct wmi_driver yogabook_wmi_driver = {
	.driver = {
		.name = "yogabook-wmi",
		.pm = &yogabook_wmi_pm_ops,
	},
	.no_notify_data = true,
	.id_table = yogabook_wmi_id_table,
	.probe = yogabook_wmi_probe,
	.remove = yogabook_wmi_remove,
	.notify = yogabook_wmi_notify,
};
module_wmi_driver(yogabook_wmi_driver);

MODULE_DEVICE_TABLE(wmi, yogabook_wmi_id_table);
MODULE_AUTHOR("Yauhen Kharuzhy");
MODULE_DESCRIPTION("Lenovo Yoga Book WMI driver");
MODULE_LICENSE("GPL v2");
