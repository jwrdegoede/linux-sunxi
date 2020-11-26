/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Platform profile sysfs interface
 *
 * See Documentation/ABI/testing/sysfs-platform_profile.rst for more
 * information.
 */

#ifndef _PLATFORM_PROFILE_H_
#define _PLATFORM_PROFILE_H_

/*
 * If more options are added please update profile_names
 * array in platform-profile.c and sysfs-platform-profile.rst
 * documentation.
 */

enum platform_profile_option {
	platform_profile_low,
	platform_profile_cool,
	platform_profile_quiet,
	platform_profile_balance,
	platform_profile_perform,
};

struct platform_profile_handler {
	unsigned int choices; /* Bitmap of available choices */
	int (*profile_get)(enum platform_profile_option *profile);
	int (*profile_set)(enum platform_profile_option profile);
};

int platform_profile_register(const struct platform_profile_handler *pprof);
int platform_profile_unregister(void);
void platform_profile_notify(void);

#endif  /*_PLATFORM_PROFILE_H_*/
