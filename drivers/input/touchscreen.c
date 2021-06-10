// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Generic helper functions for touchscreens and other two-dimensional
 *  pointing devices
 *
 *  Copyright (c) 2014 Sebastian Reichel <sre@kernel.org>
 */

#include <linux/property.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>

static int touchscreen_get_prop_from_settings_string(const char *settings,
						     const char *propname,
						     bool is_boolean,
						     u32 *val_ret)
{
	char *begin, *end;
	u32 val;

	if (!settings)
		return -ENOENT;

	begin = strstr(settings, propname);
	if (!begin)
		return -ENOENT;

	/* begin must be either the begin of settings, or be preceded by a ';' */
	if (begin != settings && begin[-1] != ';')
		return -EINVAL;

	end = begin + strlen(propname);
	if (*end != '=') {
		if (is_boolean && (*end == '\0' || *end == ';')) {
			*val_ret = true;
			return 0;
		}
		return -EINVAL;
	}

	val = simple_strtoul(end + 1, &end, 0);
	if (*end != '\0' && *end != ';')
		return -EINVAL;

	*val_ret = val;
	return 0;
}

int touchscreen_property_read_u32(struct device *dev, const char *propname,
				  const char *settings, u32 *val)
{
	int error;

	error = device_property_read_u32(dev, propname, val);

	if (touchscreen_get_prop_from_settings_string(settings, propname,
						      false, val) == 0)
		error = 0;

	return error;
}
EXPORT_SYMBOL(touchscreen_property_read_u32);

bool touchscreen_property_read_bool(struct device *dev, const char *propname,
				    const char *settings)
{
	u32 val;

	val = device_property_read_bool(dev, propname);

	touchscreen_get_prop_from_settings_string(settings, propname, true, &val);

	return val;
}
EXPORT_SYMBOL(touchscreen_property_read_bool);

static bool touchscreen_get_prop_u32(struct device *dev,
				     const char *property,
				     const char *settings,
				     unsigned int default_value,
				     unsigned int *value)
{
	u32 val;
	int error;

	error = touchscreen_property_read_u32(dev, property, settings, &val);
	if (error) {
		*value = default_value;
		return false;
	}

	*value = val;
	return true;
}

static void touchscreen_set_params(struct input_dev *dev,
				   unsigned long axis,
				   int min, int max, int fuzz)
{
	struct input_absinfo *absinfo;

	if (!test_bit(axis, dev->absbit)) {
		dev_warn(&dev->dev,
			 "Parameters are specified but the axis %lu is not set up\n",
			 axis);
		return;
	}

	absinfo = &dev->absinfo[axis];
	absinfo->minimum = min;
	absinfo->maximum = max;
	absinfo->fuzz = fuzz;
}

/**
 * touchscreen_parse_properties_with_settings - parse common touchscreen properties
 * @input: input device that should be parsed
 * @multitouch: specifies whether parsed properties should be applied to
 *	single-touch or multi-touch axes
 * @prop: pointer to a struct touchscreen_properties into which to store
 *	axis swap and invert info for use with touchscreen_report_x_y();
 *	or %NULL
 * @settings: string with ; separated name=value pairs overriding
 *	the device-properties or %NULL.
 *
 * This function parses common properties for touchscreens and sets up the
 * input device accordingly. The function keeps previously set up default
 * values if no value is specified.
 *
 * Callers can optional specify a settings string overriding the
 * device-properties, this can be used to implement a module option which
 * allows users to easily specify alternative settings for testing.
 */
void touchscreen_parse_properties_with_settings(struct input_dev *input,
						bool multitouch,
						struct touchscreen_properties *prop,
						const char *settings)
{
	struct device *dev = input->dev.parent;
	struct input_absinfo *absinfo;
	unsigned int axis, axis_x, axis_y;
	unsigned int minimum, maximum, fuzz;
	bool data_present;

	input_alloc_absinfo(input);
	if (!input->absinfo)
		return;

	axis_x = multitouch ? ABS_MT_POSITION_X : ABS_X;
	axis_y = multitouch ? ABS_MT_POSITION_Y : ABS_Y;

	data_present = touchscreen_get_prop_u32(dev, "touchscreen-min-x",
						settings,
						input_abs_get_min(input, axis_x),
						&minimum);
	data_present |= touchscreen_get_prop_u32(dev, "touchscreen-size-x",
						 settings,
						 input_abs_get_max(input,
								   axis_x) + 1,
						 &maximum);
	data_present |= touchscreen_get_prop_u32(dev, "touchscreen-fuzz-x",
						 settings,
						 input_abs_get_fuzz(input, axis_x),
						 &fuzz);
	if (data_present)
		touchscreen_set_params(input, axis_x, minimum, maximum - 1, fuzz);

	data_present = touchscreen_get_prop_u32(dev, "touchscreen-min-y",
						settings,
						input_abs_get_min(input, axis_y),
						&minimum);
	data_present |= touchscreen_get_prop_u32(dev, "touchscreen-size-y",
						 settings,
						 input_abs_get_max(input,
								   axis_y) + 1,
						 &maximum);
	data_present |= touchscreen_get_prop_u32(dev, "touchscreen-fuzz-y",
						 settings,
						 input_abs_get_fuzz(input, axis_y),
						 &fuzz);
	if (data_present)
		touchscreen_set_params(input, axis_y, minimum, maximum - 1, fuzz);

	axis = multitouch ? ABS_MT_PRESSURE : ABS_PRESSURE;
	data_present = touchscreen_get_prop_u32(dev,
						"touchscreen-max-pressure",
						settings,
						input_abs_get_max(input, axis),
						&maximum);
	data_present |= touchscreen_get_prop_u32(dev,
						 "touchscreen-fuzz-pressure",
						 settings,
						 input_abs_get_fuzz(input, axis),
						 &fuzz);
	if (data_present)
		touchscreen_set_params(input, axis, 0, maximum, fuzz);

	if (!prop)
		return;

	prop->max_x = input_abs_get_max(input, axis_x);
	prop->max_y = input_abs_get_max(input, axis_y);

	prop->invert_x = touchscreen_property_read_bool(dev, "touchscreen-inverted-x",
							settings);
	if (prop->invert_x) {
		absinfo = &input->absinfo[axis_x];
		absinfo->maximum -= absinfo->minimum;
		absinfo->minimum = 0;
	}

	prop->invert_y = touchscreen_property_read_bool(dev, "touchscreen-inverted-y",
							settings);
	if (prop->invert_y) {
		absinfo = &input->absinfo[axis_y];
		absinfo->maximum -= absinfo->minimum;
		absinfo->minimum = 0;
	}

	prop->swap_x_y = touchscreen_property_read_bool(dev, "touchscreen-swapped-x-y",
							settings);
	if (prop->swap_x_y)
		swap(input->absinfo[axis_x], input->absinfo[axis_y]);
}
EXPORT_SYMBOL(touchscreen_parse_properties_with_settings);

static void
touchscreen_apply_prop_to_x_y(const struct touchscreen_properties *prop,
			      unsigned int *x, unsigned int *y)
{
	if (prop->invert_x)
		*x = prop->max_x - *x;

	if (prop->invert_y)
		*y = prop->max_y - *y;

	if (prop->swap_x_y)
		swap(*x, *y);
}

/**
 * touchscreen_set_mt_pos - Set input_mt_pos coordinates
 * @pos: input_mt_pos to set coordinates of
 * @prop: pointer to a struct touchscreen_properties
 * @x: X coordinate to store in pos
 * @y: Y coordinate to store in pos
 *
 * Adjust the passed in x and y values applying any axis inversion and
 * swapping requested in the passed in touchscreen_properties and store
 * the result in a struct input_mt_pos.
 */
void touchscreen_set_mt_pos(struct input_mt_pos *pos,
			    const struct touchscreen_properties *prop,
			    unsigned int x, unsigned int y)
{
	touchscreen_apply_prop_to_x_y(prop, &x, &y);
	pos->x = x;
	pos->y = y;
}
EXPORT_SYMBOL(touchscreen_set_mt_pos);

/**
 * touchscreen_report_pos - Report touchscreen coordinates
 * @input: input_device to report coordinates for
 * @prop: pointer to a struct touchscreen_properties
 * @x: X coordinate to report
 * @y: Y coordinate to report
 * @multitouch: Report coordinates on single-touch or multi-touch axes
 *
 * Adjust the passed in x and y values applying any axis inversion and
 * swapping requested in the passed in touchscreen_properties and then
 * report the resulting coordinates on the input_dev's x and y axis.
 */
void touchscreen_report_pos(struct input_dev *input,
			    const struct touchscreen_properties *prop,
			    unsigned int x, unsigned int y,
			    bool multitouch)
{
	touchscreen_apply_prop_to_x_y(prop, &x, &y);
	input_report_abs(input, multitouch ? ABS_MT_POSITION_X : ABS_X, x);
	input_report_abs(input, multitouch ? ABS_MT_POSITION_Y : ABS_Y, y);
}
EXPORT_SYMBOL(touchscreen_report_pos);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Helper functions for touchscreens and other devices");
