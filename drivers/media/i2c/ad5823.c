/*
 * Analog Devices AD5823 VCM driver
 * Copyright 2023 Hans de Goede <hansg@kernel.org>
 */

#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define AD5823_MAX_FOCUS_POS		1023

#define AD5823_RESET			CCI_REG8(1)
#define AD5823_RESET_RESET		BIT(0)

#define AD5823_MODE			CCI_REG8(2)
#define AD5823_ARC_RES1			0x01

#define AD5823_VCM_MOVE_TIME		CCI_REG8(3)
#define AD5823_VCM_MOVE_TIME_DEFAULT	0x80
#define AD5823_RESONANCE_PERIOD		100000	/* in 0.1 us units */
#define AD5823_RESONANCE_COEF		512	/* in 0.1 us units */

#define AD5823_RESONANCE_OFFSET		0x80	/* for reg 0x02 bit 5 == 0 */

#define AD5823_VCM_CODE			CCI_REG16(4)
#define AD5823_VCM_CODE_RING_CTRL	BIT(10)

#define AD5823_VCM_THRESHOLD		CCI_REG16(6)
#define AD5823_VCM_THRESHOLD_DEFAULT	0x10

#define to_ad5823_device(x) container_of(x, struct ad5823_device, sd)

struct ad5823_device {
	struct v4l2_subdev sd;
	struct device *dev;
	struct regmap *regmap;
	struct regulator *regulator;
	u32 arc_mode;
	u32 resonance_period;	/* in 0.1 us units */

	struct ad5823_v4l2_ctrls {
		struct v4l2_ctrl_handler handler;
		struct v4l2_ctrl *focus;
	} ctrls;
};

static int ad5823_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ad5823_device *ad5823 = container_of(ctrl->handler,
						    struct ad5823_device,
						    ctrls.handler);
	int ret;

	/* See comment in ad5823_open() */
	ret = pm_runtime_resume_and_get(ad5823->dev);
	if (ret)
		return ret;

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		ret = cci_write(ad5823->regmap, AD5823_VCM_CODE,
				AD5823_VCM_CODE_RING_CTRL | ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(ad5823->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ad5823_ctrl_ops = {
	.s_ctrl = ad5823_set_ctrl,
};

static int ad5823_power_down(struct ad5823_device *ad5823)
{
	return regulator_disable(ad5823->regulator);
}

static int ad5823_power_up(struct ad5823_device *ad5823, bool detect)
{
	u64 vcm_move_time, vcm_threshold;
	int ret;

	ret = regulator_enable(ad5823->regulator);
	if (ret)
		return ret;

	cci_write(ad5823->regmap, AD5823_RESET, BIT(0), &ret);

	if (detect) {
		/* There is no id register, check for default reg values. */
		cci_read(ad5823->regmap, AD5823_VCM_MOVE_TIME, &vcm_move_time, &ret);
		cci_read(ad5823->regmap, AD5823_VCM_THRESHOLD, &vcm_threshold, &ret);

		if (!ret && (vcm_move_time != AD5823_VCM_MOVE_TIME_DEFAULT ||
			     vcm_threshold != AD5823_VCM_THRESHOLD_DEFAULT)) {
			dev_err(ad5823->dev, "Failed to detect AD5823 got move-time 0x%02llx vcm-threshold 0x%02llx\n",
				vcm_move_time, vcm_threshold);
			ret = -ENXIO;
		}
	}

	vcm_move_time = ad5823->resonance_period / AD5823_RESONANCE_COEF -
			AD5823_RESONANCE_OFFSET;

	cci_write(ad5823->regmap, AD5823_MODE, ad5823->arc_mode, &ret);
	cci_write(ad5823->regmap, AD5823_VCM_MOVE_TIME, vcm_move_time, &ret);
	if (ret)
		ad5823_power_down(ad5823);

	return ret;
}

static int ad5823_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ad5823_device *ad5823 = to_ad5823_device(sd);

	return ad5823_power_down(ad5823);
}

static int ad5823_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ad5823_device *ad5823 = to_ad5823_device(sd);

	return ad5823_power_up(ad5823, false);
}

static int ad5823_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	/*
	 * The VCM should not be powered up on every open, instead it is
	 * powered up on the first s_ctrl call and then kept on until
	 * the last fd for the subdev is closed.
	 * Do a runtime_pm_get_noresume() here to balance the runtime_pm_put()
	 * on close (which turns the VCM off on the last close), without
	 * powering up the VCM on every open.
	 *
	 * Note not powering up on open is not only to avoid doing unnecessary
	 * work. Powering up may also power-up the sensor through a device-link
	 * and on some models such as e.g. the Acer Iconia w4-820 with OV5693
	 * sensor this changes the stream-on timing in such a way that
	 * the sensor fails to stream.
	 */
	pm_runtime_get_noresume(sd->dev);
	return 0;
}

static int ad5823_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);
	return 0;
}

static const struct v4l2_subdev_internal_ops ad5823_internal_ops = {
	.open = ad5823_open,
	.close = ad5823_close,
};

static int ad5823_init_controls(struct ad5823_device *ad5823)
{
	const struct v4l2_ctrl_ops *ops = &ad5823_ctrl_ops;
	int ret;

	v4l2_ctrl_handler_init(&ad5823->ctrls.handler, 1);

	ad5823->ctrls.focus = v4l2_ctrl_new_std(&ad5823->ctrls.handler, ops,
						V4L2_CID_FOCUS_ABSOLUTE, 0,
						AD5823_MAX_FOCUS_POS, 1, 0);

	if (ad5823->ctrls.handler.error) {
		dev_err(ad5823->dev, "Error initialising v4l2 ctrls\n");
		ret = ad5823->ctrls.handler.error;
		goto err_free_handler;
	}

	ad5823->sd.ctrl_handler = &ad5823->ctrls.handler;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(&ad5823->ctrls.handler);
	return ret;
}

static const struct v4l2_subdev_ops ad5823_ops = { };

static int ad5823_probe(struct i2c_client *client)
{
	struct ad5823_device *ad5823;
	int ret;

	ad5823 = devm_kzalloc(&client->dev, sizeof(*ad5823), GFP_KERNEL);
	if (!ad5823)
		return -ENOMEM;

	ad5823->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(ad5823->regmap))
		return PTR_ERR(ad5823->regmap);

	ad5823->dev = &client->dev;
	ad5823->arc_mode = AD5823_ARC_RES1;
	ad5823->resonance_period = AD5823_RESONANCE_PERIOD;

	/* Optional indication of ARC mode select */
	device_property_read_u32(&client->dev, "adi,arc-mode",
				 &ad5823->arc_mode);

	/* Optional indication of VCM resonance period */
	device_property_read_u32(&client->dev, "adi,resonance-period",
				 &ad5823->resonance_period);

	ad5823->regulator = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(ad5823->regulator))
		return dev_err_probe(&client->dev, PTR_ERR(ad5823->regulator),
				     "getting regulator\n");

	v4l2_i2c_subdev_init(&ad5823->sd, client, &ad5823_ops);
	ad5823->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ad5823->sd.internal_ops = &ad5823_internal_ops;

	ret = ad5823_init_controls(ad5823);
	if (ret)
		return ret;

	ret = media_entity_pads_init(&ad5823->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_free_ctrl_handler;

	ad5823->sd.entity.function = MEDIA_ENT_F_LENS;

	/*
	 * We need the driver to work in the event that pm runtime is disable in
	 * the kernel, so power up and verify the chip now. In the event that
	 * runtime pm is disabled this will leave the chip on, so that the lens
	 * will work.
	 */

	ret = ad5823_power_up(ad5823, true);
	if (ret)
		goto err_cleanup_media;

	pm_runtime_set_active(&client->dev);
	pm_runtime_get_noresume(&client->dev);
	pm_runtime_enable(&client->dev);

	ret = v4l2_async_register_subdev(&ad5823->sd);
	if (ret < 0)
		goto err_pm_runtime;

	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;

err_pm_runtime:
	pm_runtime_disable(&client->dev);
	pm_runtime_put_noidle(&client->dev);
	ad5823_power_down(ad5823);
err_cleanup_media:
	media_entity_cleanup(&ad5823->sd.entity);
err_free_ctrl_handler:
	v4l2_ctrl_handler_free(&ad5823->ctrls.handler);

	return ret;
}

static void ad5823_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ad5823_device *ad5823 =
		container_of(sd, struct ad5823_device, sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&ad5823->ctrls.handler);
	media_entity_cleanup(&ad5823->sd.entity);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ad5823_power_down(ad5823);
	pm_runtime_set_suspended(&client->dev);
}

static const struct i2c_device_id ad5823_id_table[] = {
	{ "ad5823" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad5823_id_table);

static DEFINE_RUNTIME_DEV_PM_OPS(ad5823_pm_ops, ad5823_suspend, ad5823_resume,
				 NULL);

static struct i2c_driver ad5823_i2c_driver = {
	.driver = {
		.name = "ad5823",
		.pm = pm_sleep_ptr(&ad5823_pm_ops),
	},
	.probe = ad5823_probe,
	.remove = ad5823_remove,
	.id_table = ad5823_id_table,
};
module_i2c_driver(ad5823_i2c_driver);

MODULE_AUTHOR("Hans de Goede <hansg@kernel.org>");
MODULE_DESCRIPTION("AD5823 VCM Driver");
MODULE_LICENSE("GPL");
