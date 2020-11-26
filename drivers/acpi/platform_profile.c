// SPDX-License-Identifier: GPL-2.0-or-later

/* Platform profile sysfs interface */

#include <linux/acpi.h>
#include <linux/bits.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_profile.h>
#include <linux/sysfs.h>

static const struct platform_profile_handler *cur_profile;
static DEFINE_MUTEX(profile_lock);

static const char * const profile_names[] = {
	[platform_profile_low] = "low-power",
	[platform_profile_cool] = "cool",
	[platform_profile_quiet] = "quiet",
	[platform_profile_balance] = "balance",
	[platform_profile_perform] = "performance",
};
static_assert(ARRAY_SIZE(profile_names) == platform_profile_perform+1);

static ssize_t platform_profile_choices_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int len = 0;
	int err, i;

	err = mutex_lock_interruptible(&profile_lock);
	if (err)
		return err;

	if (!cur_profile) {
		mutex_unlock(&profile_lock);
		return -ENODEV;
	}

	if (!cur_profile->choices) {
		mutex_unlock(&profile_lock);
		return sysfs_emit(buf, "\n");
	}

	for (i = 0; i < ARRAY_SIZE(profile_names); i++) {
		if (cur_profile->choices & BIT(i)) {
			if (len == 0)
				len += sysfs_emit_at(buf, len, "%s", profile_names[i]);
			else
				len += sysfs_emit_at(buf, len, " %s", profile_names[i]);
		}
	}
	len += sysfs_emit_at(buf, len, "\n");
	mutex_unlock(&profile_lock);
	return len;
}

static ssize_t platform_profile_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	enum platform_profile_option profile = platform_profile_balance;
	int err;

	err = mutex_lock_interruptible(&profile_lock);
	if (err)
		return err;

	if (!cur_profile) {
		mutex_unlock(&profile_lock);
		return -ENODEV;
	}

	if (!cur_profile->profile_get) {
		mutex_unlock(&profile_lock);
		return -EOPNOTSUPP;
	}

	err = cur_profile->profile_get(&profile);
	mutex_unlock(&profile_lock);
	if (err < 0)
		return err;

	/* Check that profile is valid index */
	if ((profile < 0) || (profile >= ARRAY_SIZE(profile_names)))
		return sysfs_emit(buf, "\n");

	return sysfs_emit(buf, "%s\n", profile_names[profile]);
}

static ssize_t platform_profile_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int err, i;

	err = mutex_lock_interruptible(&profile_lock);
	if (err)
		return err;

	if (!cur_profile) {
		mutex_unlock(&profile_lock);
		return -ENODEV;
	}

	if (!cur_profile->profile_set) {
		mutex_unlock(&profile_lock);
		return -EOPNOTSUPP;
	}

	/* Scan for a matching profile */
	i = sysfs_match_string(profile_names, buf);
	if (i < 0) {
		mutex_unlock(&profile_lock);
		return -EINVAL;
	}

	/* Check that platform supports this profile choice */
	if (!(cur_profile->choices & BIT(i))) {
		mutex_unlock(&profile_lock);
		return -EOPNOTSUPP;
	}

	err = cur_profile->profile_set(i);
	mutex_unlock(&profile_lock);
	if (err)
		return err;
	return count;
}

static DEVICE_ATTR_RO(platform_profile_choices);
static DEVICE_ATTR_RW(platform_profile);

static struct attribute *platform_profile_attrs[] = {
	&dev_attr_platform_profile_choices.attr,
	&dev_attr_platform_profile.attr,
	NULL
};

static const struct attribute_group platform_profile_group = {
	.attrs = platform_profile_attrs
};

void platform_profile_notify(void)
{
	if (!cur_profile)
		return;
	sysfs_notify(acpi_kobj, NULL, "platform_profile");
}
EXPORT_SYMBOL_GPL(platform_profile_notify);

int platform_profile_register(const struct platform_profile_handler *pprof)
{
	int err;

	err = mutex_lock_interruptible(&profile_lock);
	if (err)
		return err;

	/* We can only have one active profile */
	if (cur_profile) {
		mutex_unlock(&profile_lock);
		return -EEXIST;
	}

	err = sysfs_create_group(acpi_kobj, &platform_profile_group);
	if (err) {
		mutex_unlock(&profile_lock);
		return err;
	}

	cur_profile = pprof;
	mutex_unlock(&profile_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(platform_profile_register);

int platform_profile_unregister(void)
{
	int err;

	err = mutex_lock_interruptible(&profile_lock);
	if (err)
		return err;

	if (!cur_profile) {
		mutex_unlock(&profile_lock);
		return -ENODEV;
	}

	sysfs_remove_group(acpi_kobj, &platform_profile_group);
	cur_profile = NULL;
	mutex_unlock(&profile_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(platform_profile_unregister);

static int __init platform_profile_init(void)
{
	return 0;
}
module_init(platform_profile_init);

static void __exit platform_profile_exit(void)
{
	/* Check if we have a registered profile, and clean up */
	if (cur_profile) {
		sysfs_remove_group(acpi_kobj, &platform_profile_group);
		cur_profile = NULL;
	}
}
module_exit(platform_profile_exit);

MODULE_AUTHOR("Mark Pearson <markpearson@lenovo.com>");
MODULE_LICENSE("GPL");
