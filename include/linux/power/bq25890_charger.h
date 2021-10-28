/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Platform data for the TI bq25890 battery charger driver.
 */

#ifndef _BQ25890_CHARGER_H_
#define _BQ25890_CHARGER_H_

#include <linux/regulator/machine.h>

struct bq25890_platform_data {
	const struct regulator_init_data *regulator_init_data;
};

#endif
