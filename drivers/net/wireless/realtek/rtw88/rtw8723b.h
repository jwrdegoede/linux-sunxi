/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */
/* Copyright(c) 2018-2019  Realtek Corporation
 */

#ifndef __RTW8723B_H__
#define __RTW8723B_H__

#include "rtw8723x.h"

extern const struct rtw_chip_info rtw8723b_hw_spec;

/* shared with rtw8703b.c; could move to rtw8723x.h */
#define REG_TXIQK_MATRIXB_LSB2_11N 0x0c9c
#define REG_BB_PWR_SAV5_11N 0x0818

#endif
