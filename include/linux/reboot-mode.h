/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __REBOOT_MODE_H__
#define __REBOOT_MODE_H__

#include <linux/types.h>

struct reboot_mode_entry {
	const char *name;
	u32 magic[3];
	u32 count;
};

struct reboot_mode_driver {
	struct device *dev;
	struct list_head head;
	/*
	 * Optional predefined reboot-mode table.
	 *
	 * Must be initialized before reboot_mode_register() and remain valid
	 * until reboot_mode_unregister().
	 *
	 * If a predefined mode name duplicates a DT-defined mode name,
	 * the DT-defined mode takes precedence.
	 */
	const struct reboot_mode_entry *predefined_modes;
	size_t predefined_mode_count;
	int (*write_array)(struct reboot_mode_driver *reboot, const u32 *magic, u32 count);
	int (*write)(struct reboot_mode_driver *reboot, unsigned int magic);
	struct notifier_block reboot_notifier;
};

int reboot_mode_driver_init(struct reboot_mode_driver *reboot,
			    struct device *dev,
			    int (*write)(struct reboot_mode_driver *reboot, unsigned int magic),
			    int (*write_array)(struct reboot_mode_driver *reboot,
					       const u32 *magic, u32 count),
			    const struct reboot_mode_entry *predefined_modes,
			    size_t predefined_mode_count);
int reboot_mode_register(struct reboot_mode_driver *reboot);
int reboot_mode_unregister(struct reboot_mode_driver *reboot);
int devm_reboot_mode_register(struct device *dev,
			      struct reboot_mode_driver *reboot);
void devm_reboot_mode_unregister(struct device *dev,
				 struct reboot_mode_driver *reboot);

#endif
