// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Sysfs interface for the universal power supply monitor class
 *
 *  Copyright © 2007  David Woodhouse <dwmw2@infradead.org>
 *  Copyright © 2007  Anton Vorontsov <cbou@mail.ru>
 *  Copyright © 2004  Szabolcs Gyurko
 *  Copyright © 2003  Ian Molton <spyro@f2s.com>
 *
 *  Modified: 2004, Oct     Szabolcs Gyurko
 */

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string_helpers.h>

#include "power_supply.h"

#define MAX_PROP_NAME_LEN 30

struct power_supply_attr {
	const char *prop_name;
	char attr_name[MAX_PROP_NAME_LEN + 1];
	struct device_attribute dev_attr;
	const char * const *text_values;
	int text_values_len;
};

#define _POWER_SUPPLY_ATTR(_name, _text, _len)	\
[POWER_SUPPLY_PROP_ ## _name] =			\
{						\
	.prop_name = #_name,			\
	.attr_name = #_name,			\
	.text_values = _text,			\
	.text_values_len = _len,		\
}

#define POWER_SUPPLY_ATTR(_name) _POWER_SUPPLY_ATTR(_name, NULL, 0)
#define _POWER_SUPPLY_ENUM_ATTR(_name, _text)	\
	_POWER_SUPPLY_ATTR(_name, _text, ARRAY_SIZE(_text))
#define POWER_SUPPLY_ENUM_ATTR(_name)	\
	_POWER_SUPPLY_ENUM_ATTR(_name, POWER_SUPPLY_ ## _name ## _TEXT)

static const char * const POWER_SUPPLY_TYPE_TEXT[] = {
	[POWER_SUPPLY_TYPE_UNKNOWN]		= "Unknown",
	[POWER_SUPPLY_TYPE_BATTERY]		= "Battery",
	[POWER_SUPPLY_TYPE_UPS]			= "UPS",
	[POWER_SUPPLY_TYPE_MAINS]		= "Mains",
	[POWER_SUPPLY_TYPE_USB]			= "USB",
	[POWER_SUPPLY_TYPE_USB_DCP]		= "USB_DCP",
	[POWER_SUPPLY_TYPE_USB_CDP]		= "USB_CDP",
	[POWER_SUPPLY_TYPE_USB_ACA]		= "USB_ACA",
	[POWER_SUPPLY_TYPE_USB_TYPE_C]		= "USB_C",
	[POWER_SUPPLY_TYPE_USB_PD]		= "USB_PD",
	[POWER_SUPPLY_TYPE_USB_PD_DRP]		= "USB_PD_DRP",
	[POWER_SUPPLY_TYPE_APPLE_BRICK_ID]	= "BrickID",
	[POWER_SUPPLY_TYPE_WIRELESS]		= "Wireless",
};

static const char * const POWER_SUPPLY_USB_TYPE_TEXT[] = {
	[POWER_SUPPLY_USB_TYPE_UNKNOWN]		= "Unknown",
	[POWER_SUPPLY_USB_TYPE_SDP]		= "SDP",
	[POWER_SUPPLY_USB_TYPE_DCP]		= "DCP",
	[POWER_SUPPLY_USB_TYPE_CDP]		= "CDP",
	[POWER_SUPPLY_USB_TYPE_ACA]		= "ACA",
	[POWER_SUPPLY_USB_TYPE_C]		= "C",
	[POWER_SUPPLY_USB_TYPE_PD]		= "PD",
	[POWER_SUPPLY_USB_TYPE_PD_DRP]		= "PD_DRP",
	[POWER_SUPPLY_USB_TYPE_PD_PPS]		= "PD_PPS",
	[POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID]	= "BrickID",
};

static const char * const POWER_SUPPLY_STATUS_TEXT[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN]		= "Unknown",
	[POWER_SUPPLY_STATUS_CHARGING]		= "Charging",
	[POWER_SUPPLY_STATUS_DISCHARGING]	= "Discharging",
	[POWER_SUPPLY_STATUS_NOT_CHARGING]	= "Not charging",
	[POWER_SUPPLY_STATUS_FULL]		= "Full",
};

static const char * const POWER_SUPPLY_CHARGE_TYPE_TEXT[] = {
	[POWER_SUPPLY_CHARGE_TYPE_UNKNOWN]	= "Unknown",
	[POWER_SUPPLY_CHARGE_TYPE_NONE]		= "N/A",
	[POWER_SUPPLY_CHARGE_TYPE_TRICKLE]	= "Trickle",
	[POWER_SUPPLY_CHARGE_TYPE_FAST]		= "Fast",
	[POWER_SUPPLY_CHARGE_TYPE_STANDARD]	= "Standard",
	[POWER_SUPPLY_CHARGE_TYPE_ADAPTIVE]	= "Adaptive",
	[POWER_SUPPLY_CHARGE_TYPE_CUSTOM]	= "Custom",
	[POWER_SUPPLY_CHARGE_TYPE_LONGLIFE]	= "Long Life",
	[POWER_SUPPLY_CHARGE_TYPE_BYPASS]	= "Bypass",
};

static const char * const POWER_SUPPLY_HEALTH_TEXT[] = {
	[POWER_SUPPLY_HEALTH_UNKNOWN]		    = "Unknown",
	[POWER_SUPPLY_HEALTH_GOOD]		    = "Good",
	[POWER_SUPPLY_HEALTH_OVERHEAT]		    = "Overheat",
	[POWER_SUPPLY_HEALTH_DEAD]		    = "Dead",
	[POWER_SUPPLY_HEALTH_OVERVOLTAGE]	    = "Over voltage",
	[POWER_SUPPLY_HEALTH_UNDERVOLTAGE]	    = "Under voltage",
	[POWER_SUPPLY_HEALTH_UNSPEC_FAILURE]	    = "Unspecified failure",
	[POWER_SUPPLY_HEALTH_COLD]		    = "Cold",
	[POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE] = "Watchdog timer expire",
	[POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE]   = "Safety timer expire",
	[POWER_SUPPLY_HEALTH_OVERCURRENT]	    = "Over current",
	[POWER_SUPPLY_HEALTH_CALIBRATION_REQUIRED]  = "Calibration required",
	[POWER_SUPPLY_HEALTH_WARM]		    = "Warm",
	[POWER_SUPPLY_HEALTH_COOL]		    = "Cool",
	[POWER_SUPPLY_HEALTH_HOT]		    = "Hot",
	[POWER_SUPPLY_HEALTH_NO_BATTERY]	    = "No battery",
	[POWER_SUPPLY_HEALTH_BLOWN_FUSE]	    = "Blown fuse",
	[POWER_SUPPLY_HEALTH_CELL_IMBALANCE]	    = "Cell imbalance",
};

static const char * const POWER_SUPPLY_TECHNOLOGY_TEXT[] = {
	[POWER_SUPPLY_TECHNOLOGY_UNKNOWN]	= "Unknown",
	[POWER_SUPPLY_TECHNOLOGY_NiMH]		= "NiMH",
	[POWER_SUPPLY_TECHNOLOGY_LION]		= "Li-ion",
	[POWER_SUPPLY_TECHNOLOGY_LIPO]		= "Li-poly",
	[POWER_SUPPLY_TECHNOLOGY_LiFe]		= "LiFe",
	[POWER_SUPPLY_TECHNOLOGY_NiCd]		= "NiCd",
	[POWER_SUPPLY_TECHNOLOGY_LiMn]		= "LiMn",
};

static const char * const POWER_SUPPLY_CAPACITY_LEVEL_TEXT[] = {
	[POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN]	= "Unknown",
	[POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL]	= "Critical",
	[POWER_SUPPLY_CAPACITY_LEVEL_LOW]	= "Low",
	[POWER_SUPPLY_CAPACITY_LEVEL_NORMAL]	= "Normal",
	[POWER_SUPPLY_CAPACITY_LEVEL_HIGH]	= "High",
	[POWER_SUPPLY_CAPACITY_LEVEL_FULL]	= "Full",
};

static const char * const POWER_SUPPLY_SCOPE_TEXT[] = {
	[POWER_SUPPLY_SCOPE_UNKNOWN]	= "Unknown",
	[POWER_SUPPLY_SCOPE_SYSTEM]	= "System",
	[POWER_SUPPLY_SCOPE_DEVICE]	= "Device",
};

static const char * const POWER_SUPPLY_CHARGE_BEHAVIOUR_TEXT[] = {
	[POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO]			= "auto",
	[POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE]		= "inhibit-charge",
	[POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE_AWAKE]	= "inhibit-charge-awake",
	[POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE]		= "force-discharge",
};

static struct power_supply_attr power_supply_attrs[] __ro_after_init = {
	/* Properties of type `int' */
	POWER_SUPPLY_ENUM_ATTR(STATUS),
	POWER_SUPPLY_ENUM_ATTR(CHARGE_TYPE),
	POWER_SUPPLY_ENUM_ATTR(HEALTH),
	POWER_SUPPLY_ATTR(PRESENT),
	POWER_SUPPLY_ATTR(ONLINE),
	POWER_SUPPLY_ATTR(AUTHENTIC),
	POWER_SUPPLY_ENUM_ATTR(TECHNOLOGY),
	POWER_SUPPLY_ATTR(CYCLE_COUNT),
	POWER_SUPPLY_ATTR(VOLTAGE_MAX),
	POWER_SUPPLY_ATTR(VOLTAGE_MIN),
	POWER_SUPPLY_ATTR(VOLTAGE_MAX_DESIGN),
	POWER_SUPPLY_ATTR(VOLTAGE_MIN_DESIGN),
	POWER_SUPPLY_ATTR(VOLTAGE_NOW),
	POWER_SUPPLY_ATTR(VOLTAGE_AVG),
	POWER_SUPPLY_ATTR(VOLTAGE_OCV),
	POWER_SUPPLY_ATTR(VOLTAGE_BOOT),
	POWER_SUPPLY_ATTR(CURRENT_MAX),
	POWER_SUPPLY_ATTR(CURRENT_NOW),
	POWER_SUPPLY_ATTR(CURRENT_AVG),
	POWER_SUPPLY_ATTR(CURRENT_BOOT),
	POWER_SUPPLY_ATTR(POWER_NOW),
	POWER_SUPPLY_ATTR(POWER_AVG),
	POWER_SUPPLY_ATTR(CHARGE_FULL_DESIGN),
	POWER_SUPPLY_ATTR(CHARGE_EMPTY_DESIGN),
	POWER_SUPPLY_ATTR(CHARGE_FULL),
	POWER_SUPPLY_ATTR(CHARGE_EMPTY),
	POWER_SUPPLY_ATTR(CHARGE_NOW),
	POWER_SUPPLY_ATTR(CHARGE_AVG),
	POWER_SUPPLY_ATTR(CHARGE_COUNTER),
	POWER_SUPPLY_ATTR(CONSTANT_CHARGE_CURRENT),
	POWER_SUPPLY_ATTR(CONSTANT_CHARGE_CURRENT_MAX),
	POWER_SUPPLY_ATTR(CONSTANT_CHARGE_VOLTAGE),
	POWER_SUPPLY_ATTR(CONSTANT_CHARGE_VOLTAGE_MAX),
	POWER_SUPPLY_ATTR(CHARGE_CONTROL_LIMIT),
	POWER_SUPPLY_ATTR(CHARGE_CONTROL_LIMIT_MAX),
	POWER_SUPPLY_ATTR(CHARGE_CONTROL_START_THRESHOLD),
	POWER_SUPPLY_ATTR(CHARGE_CONTROL_END_THRESHOLD),
	POWER_SUPPLY_ENUM_ATTR(CHARGE_BEHAVIOUR),
	/* Same enum value texts as "charge_type" without the 's' at the end */
	_POWER_SUPPLY_ENUM_ATTR(CHARGE_TYPES, POWER_SUPPLY_CHARGE_TYPE_TEXT),
	POWER_SUPPLY_ATTR(INPUT_CURRENT_LIMIT),
	POWER_SUPPLY_ATTR(INPUT_VOLTAGE_LIMIT),
	POWER_SUPPLY_ATTR(INPUT_POWER_LIMIT),
	POWER_SUPPLY_ATTR(ENERGY_FULL_DESIGN),
	POWER_SUPPLY_ATTR(ENERGY_EMPTY_DESIGN),
	POWER_SUPPLY_ATTR(ENERGY_FULL),
	POWER_SUPPLY_ATTR(ENERGY_EMPTY),
	POWER_SUPPLY_ATTR(ENERGY_NOW),
	POWER_SUPPLY_ATTR(ENERGY_AVG),
	POWER_SUPPLY_ATTR(CAPACITY),
	POWER_SUPPLY_ATTR(CAPACITY_ALERT_MIN),
	POWER_SUPPLY_ATTR(CAPACITY_ALERT_MAX),
	POWER_SUPPLY_ATTR(CAPACITY_ERROR_MARGIN),
	POWER_SUPPLY_ENUM_ATTR(CAPACITY_LEVEL),
	POWER_SUPPLY_ATTR(TEMP),
	POWER_SUPPLY_ATTR(TEMP_MAX),
	POWER_SUPPLY_ATTR(TEMP_MIN),
	POWER_SUPPLY_ATTR(TEMP_ALERT_MIN),
	POWER_SUPPLY_ATTR(TEMP_ALERT_MAX),
	POWER_SUPPLY_ATTR(TEMP_AMBIENT),
	POWER_SUPPLY_ATTR(TEMP_AMBIENT_ALERT_MIN),
	POWER_SUPPLY_ATTR(TEMP_AMBIENT_ALERT_MAX),
	POWER_SUPPLY_ATTR(TIME_TO_EMPTY_NOW),
	POWER_SUPPLY_ATTR(TIME_TO_EMPTY_AVG),
	POWER_SUPPLY_ATTR(TIME_TO_FULL_NOW),
	POWER_SUPPLY_ATTR(TIME_TO_FULL_AVG),
	POWER_SUPPLY_ENUM_ATTR(TYPE),
	POWER_SUPPLY_ENUM_ATTR(USB_TYPE),
	POWER_SUPPLY_ENUM_ATTR(SCOPE),
	POWER_SUPPLY_ATTR(PRECHARGE_CURRENT),
	POWER_SUPPLY_ATTR(CHARGE_TERM_CURRENT),
	POWER_SUPPLY_ATTR(CALIBRATE),
	POWER_SUPPLY_ATTR(MANUFACTURE_YEAR),
	POWER_SUPPLY_ATTR(MANUFACTURE_MONTH),
	POWER_SUPPLY_ATTR(MANUFACTURE_DAY),
	/* Properties of type `const char *' */
	POWER_SUPPLY_ATTR(MODEL_NAME),
	POWER_SUPPLY_ATTR(MANUFACTURER),
	POWER_SUPPLY_ATTR(SERIAL_NUMBER),
};
#define POWER_SUPPLY_ATTR_CNT ARRAY_SIZE(power_supply_attrs)

static struct attribute *
__power_supply_attrs[POWER_SUPPLY_ATTR_CNT + 1] __ro_after_init;

static const struct power_supply_attr *to_ps_attr(struct device_attribute *attr)
{
	return container_of(attr, struct power_supply_attr, dev_attr);
}

static enum power_supply_property dev_attr_psp(struct device_attribute *attr)
{
	return  to_ps_attr(attr) - power_supply_attrs;
}

static void power_supply_escape_spaces(const char *str, char *buf, size_t bufsize)
{
	strscpy(buf, str, bufsize);
	strreplace(buf, ' ', '_');
}

static int power_supply_match_string(const char * const *array, size_t n, const char *s)
{
	int ret;

	/* First try an exact match */
	ret = __sysfs_match_string(array, n, s);
	if (ret >= 0)
		return ret;

	/* Second round, try matching with spaces replaced by '_' */
	for (size_t i = 0; i < n; i++) {
		char buf[32];

		power_supply_escape_spaces(array[i], buf, sizeof(buf));
		if (sysfs_streq(buf, s))
			return i;
	}

	return -EINVAL;
}

static ssize_t power_supply_show_enum_with_available(
			struct device *dev, const char * const labels[], int label_count,
			unsigned int available_values, int value, char *buf)
{
	bool match = false, available, active;
	char escaped_label[32];
	ssize_t count = 0;
	int i;

	for (i = 0; i < label_count; i++) {
		available = available_values & BIT(i);
		active = i == value;
		power_supply_escape_spaces(labels[i], escaped_label, sizeof(escaped_label));

		if (available && active) {
			count += sysfs_emit_at(buf, count, "[%s] ", escaped_label);
			match = true;
		} else if (available) {
			count += sysfs_emit_at(buf, count, "%s ", escaped_label);
		}
	}

	if (!match) {
		dev_warn(dev, "driver reporting unavailable enum value %d\n", value);
		return -EINVAL;
	}

	if (count)
		buf[count - 1] = '\n';

	return count;
}

static ssize_t power_supply_show_charge_behaviour(struct device *dev,
						  struct power_supply *psy,
						  union power_supply_propval *value,
						  char *buf)
{
	struct power_supply_ext_registration *reg;

	scoped_guard(rwsem_read, &psy->extensions_sem) {
		power_supply_for_each_extension(reg, psy) {
			if (power_supply_ext_has_property(reg->ext,
							  POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR))
				return power_supply_charge_behaviour_show(dev,
						reg->ext->charge_behaviours,
						value->intval, buf);
		}
	}

	return power_supply_charge_behaviour_show(dev, psy->desc->charge_behaviours,
						  value->intval, buf);
}

static ssize_t power_supply_format_property(struct device *dev,
					    bool uevent,
					    struct device_attribute *attr,
					    char *buf)
{
	ssize_t ret;
	struct power_supply *psy = dev_to_psy(dev);
	const struct power_supply_attr *ps_attr = to_ps_attr(attr);
	enum power_supply_property psp = dev_attr_psp(attr);
	union power_supply_propval value;

	if (psp == POWER_SUPPLY_PROP_TYPE) {
		value.intval = psy->desc->type;
	} else {
		ret = power_supply_get_property(psy, psp, &value);

		if (ret < 0) {
			if (ret == -ENODATA)
				dev_dbg_ratelimited(dev,
					"driver has no data for `%s' property\n",
					attr->attr.name);
			else if (ret != -ENODEV && ret != -EAGAIN && ret != -EINVAL)
				dev_err_ratelimited(dev,
					"driver failed to report `%s' property: %zd\n",
					attr->attr.name, ret);
			return ret;
		}
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_USB_TYPE:
		ret = power_supply_show_enum_with_available(
				dev, POWER_SUPPLY_USB_TYPE_TEXT,
				ARRAY_SIZE(POWER_SUPPLY_USB_TYPE_TEXT),
				psy->desc->usb_types, value.intval, buf);
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (uevent) /* no possible values in uevents */
			goto default_format;
		ret = power_supply_show_charge_behaviour(dev, psy, &value, buf);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPES:
		if (uevent) /* no possible values in uevents */
			goto default_format;
		ret = power_supply_charge_types_show(dev, psy->desc->charge_types,
						     value.intval, buf);
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME ... POWER_SUPPLY_PROP_SERIAL_NUMBER:
		ret = sysfs_emit(buf, "%s\n", value.strval);
		break;
	default:
default_format:
		if (ps_attr->text_values_len > 0 &&
				value.intval < ps_attr->text_values_len && value.intval >= 0) {
			ret = sysfs_emit(buf, "%s\n", ps_attr->text_values[value.intval]);
		} else {
			ret = sysfs_emit(buf, "%d\n", value.intval);
		}
	}

	return ret;
}

static ssize_t power_supply_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	return power_supply_format_property(dev, false, attr, buf);
}

static ssize_t power_supply_store_property(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count) {
	ssize_t ret;
	struct power_supply *psy = dev_to_psy(dev);
	const struct power_supply_attr *ps_attr = to_ps_attr(attr);
	enum power_supply_property psp = dev_attr_psp(attr);
	union power_supply_propval value;

	ret = -EINVAL;
	if (ps_attr->text_values_len > 0) {
		ret = power_supply_match_string(ps_attr->text_values,
						ps_attr->text_values_len, buf);
	}

	/*
	 * If no match was found, then check to see if it is an integer.
	 * Integer values are valid for enums in addition to the text value.
	 */
	if (ret < 0) {
		long long_val;

		ret = kstrtol(buf, 10, &long_val);
		if (ret < 0)
			return ret;

		ret = long_val;
	}

	value.intval = ret;

	ret = power_supply_set_property(psy, psp, &value);
	if (ret < 0)
		return ret;

	return count;
}

static umode_t power_supply_attr_is_visible(struct kobject *kobj,
					   struct attribute *attr,
					   int attrno)
{
	struct device *dev = kobj_to_dev(kobj);
	struct power_supply *psy = dev_to_psy(dev);
	umode_t mode = S_IRUSR | S_IRGRP | S_IROTH;

	if (!power_supply_attrs[attrno].prop_name)
		return 0;

	if (attrno == POWER_SUPPLY_PROP_TYPE)
		return mode;

	guard(rwsem_read)(&psy->extensions_sem);

	if (power_supply_has_property(psy, attrno)) {
		if (power_supply_property_is_writeable(psy, attrno) > 0)
			mode |= S_IWUSR;
		return mode;
	}

	return 0;
}

static const struct attribute_group power_supply_attr_group = {
	.attrs = __power_supply_attrs,
	.is_visible = power_supply_attr_is_visible,
};

static struct attribute *power_supply_extension_attrs[] = {
	NULL
};

static const struct attribute_group power_supply_extension_group = {
	.name = "extensions",
	.attrs = power_supply_extension_attrs,
};

const struct attribute_group *power_supply_attr_groups[] = {
	&power_supply_attr_group,
	&power_supply_extension_group,
	NULL
};

void __init power_supply_init_attrs(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(power_supply_attrs); i++) {
		struct device_attribute *attr;

		if (!power_supply_attrs[i].prop_name) {
			pr_warn("%s: Property %d skipped because it is missing from power_supply_attrs\n",
				__func__, i);
			sprintf(power_supply_attrs[i].attr_name, "_err_%d", i);
		} else {
			string_lower(power_supply_attrs[i].attr_name,
				     power_supply_attrs[i].attr_name);
		}

		attr = &power_supply_attrs[i].dev_attr;

		attr->attr.name = power_supply_attrs[i].attr_name;
		attr->show = power_supply_show_property;
		attr->store = power_supply_store_property;
		__power_supply_attrs[i] = &attr->attr;
	}
}

static int add_prop_uevent(const struct device *dev, struct kobj_uevent_env *env,
			   enum power_supply_property prop, char *prop_buf)
{
	int ret = 0;
	struct power_supply_attr *pwr_attr;
	struct device_attribute *dev_attr;
	char *line;

	pwr_attr = &power_supply_attrs[prop];
	dev_attr = &pwr_attr->dev_attr;

	ret = power_supply_format_property((struct device *)dev, true, dev_attr, prop_buf);
	if (ret == -ENODEV || ret == -ENODATA || ret == -EINVAL) {
		/*
		 * When a battery is absent, we expect -ENODEV. Don't abort;
		 * send the uevent with at least the PRESENT=0 property
		 */
		return 0;
	}

	if (ret < 0)
		return ret;

	line = strchr(prop_buf, '\n');
	if (line)
		*line = 0;

	return add_uevent_var(env, "POWER_SUPPLY_%s=%s",
			      pwr_attr->prop_name, prop_buf);
}

int power_supply_uevent(const struct device *dev, struct kobj_uevent_env *env)
{
	const struct power_supply *psy = dev_to_psy(dev);
	int ret = 0, j;
	char *prop_buf;

	if (!psy || !psy->desc) {
		dev_dbg(dev, "No power supply yet\n");
		return ret;
	}

	ret = add_uevent_var(env, "POWER_SUPPLY_NAME=%s", psy->desc->name);
	if (ret)
		return ret;

	/*
	 * Kernel generates KOBJ_REMOVE uevent in device removal path, after
	 * resources have been freed. Exit early to avoid use-after-free.
	 */
	if (psy->removing)
		return 0;

	prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!prop_buf)
		return -ENOMEM;

	ret = add_prop_uevent(dev, env, POWER_SUPPLY_PROP_TYPE, prop_buf);
	if (ret)
		goto out;

	for (j = 0; j < POWER_SUPPLY_ATTR_CNT; j++) {
		ret = add_prop_uevent(dev, env, j, prop_buf);
		if (ret)
			goto out;
	}

out:
	free_page((unsigned long)prop_buf);

	return ret;
}

ssize_t power_supply_charge_behaviour_show(struct device *dev,
					   unsigned int available_behaviours,
					   enum power_supply_charge_behaviour current_behaviour,
					   char *buf)
{
	return power_supply_show_enum_with_available(
				dev, POWER_SUPPLY_CHARGE_BEHAVIOUR_TEXT,
				ARRAY_SIZE(POWER_SUPPLY_CHARGE_BEHAVIOUR_TEXT),
				available_behaviours, current_behaviour, buf);
}
EXPORT_SYMBOL_GPL(power_supply_charge_behaviour_show);

int power_supply_charge_behaviour_parse(unsigned int available_behaviours, const char *buf)
{
	int i = sysfs_match_string(POWER_SUPPLY_CHARGE_BEHAVIOUR_TEXT, buf);

	if (i < 0)
		return i;

	if (available_behaviours & BIT(i))
		return i;

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(power_supply_charge_behaviour_parse);

ssize_t power_supply_charge_types_show(struct device *dev,
				       unsigned int available_types,
				       enum power_supply_charge_type current_type,
				       char *buf)
{
	return power_supply_show_enum_with_available(
				dev, POWER_SUPPLY_CHARGE_TYPE_TEXT,
				ARRAY_SIZE(POWER_SUPPLY_CHARGE_TYPE_TEXT),
				available_types, current_type, buf);
}
EXPORT_SYMBOL_GPL(power_supply_charge_types_show);

int power_supply_charge_types_parse(unsigned int available_types, const char *buf)
{
	int i = power_supply_match_string(POWER_SUPPLY_CHARGE_TYPE_TEXT,
					  ARRAY_SIZE(POWER_SUPPLY_CHARGE_TYPE_TEXT),
					  buf);

	if (i < 0)
		return i;

	if (available_types & BIT(i))
		return i;

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(power_supply_charge_types_parse);

int power_supply_sysfs_add_extension(struct power_supply *psy, const struct power_supply_ext *ext,
				     struct device *dev)
{
	return sysfs_add_link_to_group(&psy->dev.kobj, power_supply_extension_group.name,
				       &dev->kobj, ext->name);
}

void power_supply_sysfs_remove_extension(struct power_supply *psy,
					 const struct power_supply_ext *ext)
{
	sysfs_remove_link_from_group(&psy->dev.kobj, power_supply_extension_group.name, ext->name);
}
