/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2014 Sebastian Reichel <sre@kernel.org>
 */

#ifndef _TOUCHSCREEN_H
#define _TOUCHSCREEN_H

struct input_dev;
struct input_mt_pos;

struct touchscreen_properties {
	unsigned int max_x;
	unsigned int max_y;
	bool invert_x;
	bool invert_y;
	bool swap_x_y;
};

void touchscreen_parse_properties_with_settings(struct input_dev *input,
						bool multitouch,
						struct touchscreen_properties *prop,
						const char *settings);

static inline void touchscreen_parse_properties(struct input_dev *input,
						bool multitouch,
						struct touchscreen_properties *prop)
{
	touchscreen_parse_properties_with_settings(input, multitouch, prop, NULL);
}

int touchscreen_property_read_u32(struct device *dev, const char *propname,
				  const char *settings, u32 *val);

bool touchscreen_property_read_bool(struct device *dev, const char *propname,
				    const char *settings);

void touchscreen_set_mt_pos(struct input_mt_pos *pos,
			    const struct touchscreen_properties *prop,
			    unsigned int x, unsigned int y);

void touchscreen_report_pos(struct input_dev *input,
			    const struct touchscreen_properties *prop,
			    unsigned int x, unsigned int y,
			    bool multitouch);

#endif
