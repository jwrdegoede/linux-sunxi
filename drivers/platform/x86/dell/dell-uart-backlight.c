/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Dell AIO Serial Backlight Driver
 *
 * Copyright (C) 2024 Hans de Goede <hansg@kernel.org>
 * Copyright (C) 2017 AceLan Kao <acelan.kao@canonical.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/acpi.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/serdev.h>
#include <linux/wait.h>
#include "../serdev_helpers.h"

/* The backlight controller must respond within 1 second */
#define DELL_BL_TIMEOUT		msecs_to_jiffies(1000)
#define DELL_BL_MIN_RESP_SIZE	3

struct dell_uart_backlight {
	struct mutex mutex;
	wait_queue_head_t wait_queue;
	struct device *dev;
	struct backlight_device *bl;
	u8 *resp;
	u8 resp_idx;
	u8 resp_len;
	u8 resp_max_len;
	u8 pending_cmd;
	int status;
	int power;
};

/* Checksum: SUM(Length and Cmd and Data) xor 0xFF */
static u8 dell_uart_checksum(u8 *buf, int len)
{
	u8 val = 0;

	while (len-- > 0)
		val += buf[len];

	return val ^ 0xff;
}

static int dell_uart_bl_command(struct dell_uart_backlight *dell_bl,
				const u8 *cmd, int cmd_len,
				u8 *resp, int resp_max_len)
{
	int ret;

	ret = mutex_lock_killable(&dell_bl->mutex);
	if (ret)
		return ret;

	dell_bl->status = 0;
	dell_bl->resp = resp;
	dell_bl->resp_idx = 0;
	dell_bl->resp_max_len = resp_max_len;
	dell_bl->pending_cmd = cmd[1];

	/* The TTY buffer should be big enough to take the entire cmd in one go */
	ret = serdev_device_write_buf(to_serdev_device(dell_bl->dev), cmd, cmd_len);
	if (ret != cmd_len) {
		dev_err(dell_bl->dev, "Error writing command: %d\n", ret);
		ret = (ret < 0) ? ret : -EIO;
		goto out;
	}

	ret = wait_event_timeout(dell_bl->wait_queue, dell_bl->status, DELL_BL_TIMEOUT);
	if (ret == 0) {
		dev_err(dell_bl->dev, "Timed out waiting for response.\n");
		dell_bl->status = -ETIMEDOUT;
	}

	if (dell_bl->status == 1)
		ret = 0;
	else
		ret = dell_bl->status;

out:
	mutex_unlock(&dell_bl->mutex);
	return ret;
}

static int dell_uart_set_brightness(struct dell_uart_backlight *dell_bl, int brightness)
{
	/*
	 * Set Brightness level: Application uses this command to set brightness.
	 * Command: 0x8A 0x0B <brightness-level> Checksum (Length:4 Type:0x0A Cmd:0x0B)
	 * 	    <brightness-level> ranges from 0~100.
	 * Return data: 0x03 0x0B 0xF1 (Length:3 Cmd:0x0B Checksum:0xF1)
	 */
	u8 set_brightness[] = { 0x8A, 0x0B, 0x00, 0x00 };
	u8 resp[3];

	set_brightness[2] = brightness;
	set_brightness[3] = dell_uart_checksum(set_brightness, 3);

	return dell_uart_bl_command(dell_bl, set_brightness, ARRAY_SIZE(set_brightness),
				    resp, ARRAY_SIZE(resp));
}

static int dell_uart_get_brightness(struct dell_uart_backlight *dell_bl)
{
	/*
	 * Get Brightness level: Application uses this command to get brightness.
	 * Command: 0x6A 0x0C 0x89 (Length:3 Type:0x0A Cmd:0x0C Checksum:0x89)
	 * Return data: 0x04 0x0C Data Checksum
	 *              (Length:4 Cmd:0x0C Data:<brightness level>
	 *               Checksum: SUM(Length and Cmd and Data) xor 0xFF)
	 *              <brightness level> ranges from 0~100.
	 */
	const u8 get_brightness[] = { 0x6A, 0x0C, 0x89 };
	u8 resp[4];
	int ret;

	ret = dell_uart_bl_command(dell_bl, get_brightness, ARRAY_SIZE(get_brightness),
				   resp, ARRAY_SIZE(resp));
	if (ret)
		return ret;

	if (resp[0] != 4) {
		dev_err(dell_bl->dev, "Unexpected get brightness response length: %d\n", resp[0]);
		return -EIO;
	}

	if (resp[2] > 100) {
		dev_err(dell_bl->dev, "Unexpected get brightness response: %d\n", resp[2]);
		return -EIO;
	}

	return resp[2];
}

static int dell_uart_set_bl_power(struct dell_uart_backlight *dell_bl, int power)
{
	/*
	 * Screen ON/OFF Control: Application uses this command to control screen ON or OFF.
	 * Command: 0x8A 0x0E Data Checksum (Length:4 Type:0x0A Cmd:0x0E) where
	 * 	    Data=0 to turn OFF the screen.
	 * 	    Data=1 to turn ON the screen.
	 * 	    Other value of Data is reserved and invalid.
	 * Return data: 0x03 0x0E 0xEE (Length:3 Cmd:0x0E Checksum:0xEE)
	 */
	u8 set_power[] = { 0x8A, 0x0E, 0x00, 0x00 };
	u8 resp[3];
	int ret;

	set_power[2] = (power == FB_BLANK_UNBLANK) ? 1 : 0;
	set_power[3] = dell_uart_checksum(set_power, 3);

	ret = dell_uart_bl_command(dell_bl, set_power, ARRAY_SIZE(set_power),
				   resp, ARRAY_SIZE(resp));
	if (ret)
		return ret;

	dell_bl->power = power;
	return 0;
}

/*
 * There is no command to get backlight power status,
 * so we set the backlight power to "on" while initializing,
 * and then track and report its status by power variable
 */
static int dell_uart_get_bl_power(struct dell_uart_backlight *dell_bl)
{
	return dell_bl->power;
}

static int dell_uart_update_status(struct backlight_device *bd)
{
	struct dell_uart_backlight *dell_bl = bl_get_data(bd);
	int ret;

	ret = dell_uart_set_brightness(dell_bl, bd->props.brightness);
	if (ret)
		return ret;

	if (bd->props.power != dell_uart_get_bl_power(dell_bl))
		ret = dell_uart_set_bl_power(dell_bl, bd->props.power);

	return ret;
}

static int dell_uart_get_brightness_op(struct backlight_device *bd)
{
	return dell_uart_get_brightness(bl_get_data(bd));
}

static const struct backlight_ops dell_uart_backlight_ops = {
	.update_status = dell_uart_update_status,
	.get_brightness = dell_uart_get_brightness_op,
};

static ssize_t dell_uart_bl_receive(struct serdev_device *serdev, const u8 *data, size_t len)
{
	struct dell_uart_backlight *dell_bl = serdev_device_get_drvdata(serdev);
	size_t i;
	u8 csum;

	dev_dbg(dell_bl->dev, "Recv: %*ph\n", (int)len, data);

	/* Throw away unexpected bytes / remainder of response after an error */
	if (dell_bl->status) {
		dev_warn(dell_bl->dev, "Bytes received out of band, dropping them.\n");
		return len;
	}

	for (i = 0; i < len; i++) {
		dell_bl->resp[dell_bl->resp_idx] = data[i];

		switch (dell_bl->resp_idx) {
		case 0: /* Length byte */
			dell_bl->resp_len = dell_bl->resp[0];
			if (dell_bl->resp_len < DELL_BL_MIN_RESP_SIZE) {
				dev_err(dell_bl->dev, "Response length too small %d < %d\n",
					dell_bl->resp_len, DELL_BL_MIN_RESP_SIZE);
				dell_bl->status = -EIO;
				goto wakeup;
			} else if (dell_bl->resp_len > dell_bl->resp_max_len) {
				dev_err(dell_bl->dev, "Response length too big %d > %d\n",
					dell_bl->resp_len, dell_bl->resp_max_len);
				dell_bl->status = -EIO;
				goto wakeup;
			}
			break;
		case 1: /* CMD byte */
			if (dell_bl->resp[1] != dell_bl->pending_cmd) {
				dev_err(dell_bl->dev, "Response cmd 0x%02x != pending 0x%02x\n",
					dell_bl->resp[1], dell_bl->pending_cmd);
				dell_bl->status = -EIO;
				goto wakeup;
			}
			break;
		}

		dell_bl->resp_idx++;
		if (dell_bl->resp_idx < dell_bl->resp_len)
			continue;

		csum = dell_uart_checksum(dell_bl->resp, dell_bl->resp_len - 1);
		if (dell_bl->resp[dell_bl->resp_len - 1] != csum) {
			dev_err(dell_bl->dev, "Checksum mismatch got 0x%02x expected 0x%02x\n",
				dell_bl->resp[dell_bl->resp_len - 1], csum);
			dell_bl->status = -EIO;
			goto wakeup;
		}

		dell_bl->status = 1; /* Success */
		goto wakeup;
	}

	return len;

wakeup:
	wake_up(&dell_bl->wait_queue);
	return i + 1;
}

static const struct serdev_device_ops dell_uart_bl_serdev_ops = {
	.receive_buf = dell_uart_bl_receive,
	.write_wakeup = serdev_device_write_wakeup,
};

static int dell_uart_bl_serdev_probe(struct serdev_device *serdev)
{
	/*
	 * Get Firmware Version: Tool uses this command to get firmware version.
	 * Command: 0x6A 0x06 0x8F (Length:3 Type:0x0A Cmd:6 Checksum:0x8F)
	 * Return data: 0x0D 0x06 Data Checksum (Length:13 Cmd:0x06
	 * 	        Data:F/W version(APRILIA=APR27-Vxxx/PHINE=PHI23-Vxxx)
	 * 	        Checksum:SUM(Length and Cmd and Data) xor 0xFF)
	 */
	const u8 get_firmware_ver[] = { 0x6A, 0x06, 0x8F };
	struct dell_uart_backlight *dell_bl;
	struct backlight_properties props;
	struct device *dev = &serdev->dev;
	u8 get_firmware_ver_resp[80];
	int ret;

	dell_bl = devm_kzalloc(dev, sizeof(*dell_bl), GFP_KERNEL);
	if (!dell_bl)
		return -ENOMEM;

	mutex_init(&dell_bl->mutex);
	init_waitqueue_head(&dell_bl->wait_queue);
	dell_bl->dev = dev;

	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return dev_err_probe(dev, ret, "opening UART device\n");

	/* 9600 bps, no flow control, these are the default but set them to be sure */
	serdev_device_set_baudrate(serdev, 9600);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_drvdata(serdev, dell_bl);
	serdev_device_set_client_ops(serdev, &dell_uart_bl_serdev_ops);

	ret = dell_uart_bl_command(dell_bl, get_firmware_ver, ARRAY_SIZE(get_firmware_ver),
				   get_firmware_ver_resp, ARRAY_SIZE(get_firmware_ver_resp));
	if (ret)
		return dev_err_probe(dev, ret, "getting firmware version\n");

	dev_dbg(dev, "Firmware version: %.*s\n", get_firmware_ver_resp[0] - 3,
		get_firmware_ver_resp + 2);

	/* Initialize bl_power to a known value */
	ret = dell_uart_set_bl_power(dell_bl, FB_BLANK_UNBLANK);
	if (ret)
		return ret;

	ret = dell_uart_get_brightness(dell_bl);
	if (ret < 0)
		return ret;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.brightness = ret;
	props.max_brightness = 100;
	props.power = dell_bl->power;

	dell_bl->bl = devm_backlight_device_register(dev, "dell_uart_backlight",
						     dev, dell_bl,
						     &dell_uart_backlight_ops,
						     &props);
	if (IS_ERR(dell_bl->bl))
		return PTR_ERR(dell_bl->bl);

	return 0;
}

struct serdev_device_driver dell_uart_bl_serdev_driver = {
	.probe = dell_uart_bl_serdev_probe,
	.driver = {
		.name = KBUILD_MODNAME,
	},
};

static int dell_uart_bl_pdev_probe(struct platform_device *pdev)
{
	struct serdev_device *serdev;
	struct device *ctrl_dev;
	int ret;

	ctrl_dev = get_serdev_controller("DELL0501", "0", 0, "serial0");
	if (IS_ERR(ctrl_dev))
		return PTR_ERR(ctrl_dev);

	serdev = serdev_device_alloc(to_serdev_controller(ctrl_dev));
	put_device(ctrl_dev);
	if (!serdev)
		return -ENOMEM;

	ret = serdev_device_add(serdev);
	if (ret) {
		dev_err(&pdev->dev, "error %d adding serdev\n", ret);
		serdev_device_put(serdev);
		return ret;
	}

	ret = serdev_device_driver_register(&dell_uart_bl_serdev_driver);
	if (ret) {
		serdev_device_remove(serdev);
		return ret;
	}

	/*
	 * serdev device <-> driver matching relies on OF or ACPI matches and
	 * neither is available here, manually bind the driver.
	 */
	ret = device_driver_attach(&dell_uart_bl_serdev_driver.driver, &serdev->dev);
	if (ret) {
		serdev_device_driver_unregister(&dell_uart_bl_serdev_driver);
		serdev_device_remove(serdev);
		return ret;
	}

	/* So that dell_uart_bl_pdev_remove() can remove the serdev */
	platform_set_drvdata(pdev, serdev);
	return 0;
}

static void dell_uart_bl_pdev_remove(struct platform_device *pdev)
{
	struct serdev_device *serdev = platform_get_drvdata(pdev);

	serdev_device_driver_unregister(&dell_uart_bl_serdev_driver);
	serdev_device_remove(serdev);
}

static struct platform_driver dell_uart_bl_pdev_driver = {
	.probe = dell_uart_bl_pdev_probe,
	.remove_new = dell_uart_bl_pdev_remove,
	.driver = {
		.name = "dell-uart-backlight",
	},
};
module_platform_driver(dell_uart_bl_pdev_driver);

MODULE_ALIAS("platform:dell-uart-backlight");
MODULE_DESCRIPTION("Dell AIO Serial Backlight driver");
MODULE_AUTHOR("Hans de Goede <hansg@kernel.org>");
MODULE_LICENSE("GPL");
