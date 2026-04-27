/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Qualcomm CAMSS ISP parameters UAPI
 *
 * Uses the generic V4L2 extensible ISP parameters buffer format defined in
 * <uapi/linux/media/v4l2-isp.h>.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef _UAPI_LINUX_CAMSS_CONFIG_H
#define _UAPI_LINUX_CAMSS_CONFIG_H

#include <linux/types.h>
#include <linux/media/v4l2-isp.h>

#include <linux/videodev2.h>

/**
 * enum camss_params_block_type - CAMSS ISP parameter block identifiers
 *
 * Each value identifies one ISP processing block.  The value is placed in
 * the @type field of &struct v4l2_isp_params_block_header.
 */
enum camss_params_block_type {
	CAMSS_PARAMS_WB_GAIN = 1,
	CAMSS_PARAMS_CHROMA_ENHAN = 2,
	CAMSS_PARAMS_COLOR_CORRECT = 3,
	CAMSS_PARAMS_MAX,
};

/**
 * struct camss_params_wb_gain - White Balance gains
 *
 * @header:   generic block header; @header.type = CAMSS_PARAMS_WB_GAIN
 * @g_gain:   green channel gain (15uQ10)
 * @b_gain:   blue channel gain (15uQ10)
 * @r_gain:   red channel gain (15uQ10)
 */
struct camss_params_wb_gain {
	struct v4l2_isp_params_block_header header;
	__u16 g_gain;
	__u16 b_gain;
	__u16 r_gain;
	__u16 _pad;
} __attribute__((aligned(8)));

/**
 * struct camss_params_chroma_enhan - RGB to YUV colour matrix
 *
 * Implements the CLC_CHROMA_ENHAN pipeline module. All coefficients are
 * signed 12-bit fixed-point Q3.8 (range roughly -8.0 to +7.996).
 *
 * Luma (Y) row of the matrix:
 * @luma_v0:  R-to-Y coefficient (12sQ8, e.g. 0x04d = 0.299)
 * @luma_v1:  G-to-Y coefficient (12sQ8, e.g. 0x096 = 0.587)
 * @luma_v2:  B-to-Y coefficient (12sQ8, e.g. 0x01d = 0.114)
 * @luma_k:   Y output offset    (9s,  0 = no offset)
 *
 * Chroma (Cb) row:
 * @coeff_ap: Cb positive coefficient (12sQ8, e.g. 0x0e6 =  0.886)
 * @coeff_am: Cb negative coefficient (12sQ8, e.g. 0xfb3 = -0.338)
 * @kcb:      Cb output offset        (11s,   128 = +128)
 *
 * Chroma (Cr) row:
 * @coeff_cp: Cr positive coefficient (12sQ8, e.g. 0x0b3 =  0.701)
 * @coeff_cm: Cr negative coefficient (12sQ8, e.g. 0xfe3 = -0.114)
 * @coeff_dp: Cr positive coefficient (12sQ8)
 * @coeff_dm: Cr negative coefficient (12sQ8)
 * @kcr:      Cr output offset        (11s,   128 = +128)
 *
 * @header: generic block header; @header.type = CAMSS_PARAMS_CHROMA_ENHAN
 */
struct camss_params_chroma_enhan {
	struct v4l2_isp_params_block_header header;
	__u16 luma_v0;
	__u16 luma_v1;
	__u16 luma_v2;
	__u16 luma_k;
	__u16 coeff_ap;
	__u16 coeff_am;
	__u16 coeff_cp;
	__u16 coeff_cm;
	__u16 coeff_dp;
	__u16 coeff_dm;
	__u16 kcb;
	__u16 kcr;
} __attribute__((aligned(8)));

/**
 * struct camss_params_color_correct - colour correction matrix
 *
 * signed 12-bit fixed-point (Qm)
 *
 * Out_ch0 (g) = A0*G+B0*B+C0*R + K0
 * Out_ch1 (b) = A1*G+B1*B+C1*R + K1
 * Out_ch2 (r) = A2*G+B2*B+C2*R + K2
 *
 * m = 0x0 - Coefficients are signed 12sQ7 numbers
 * m = 0x1 - Coefficients are signed 12sQ8 numbers
 * m = 0x2 - Coefficients are signed 12sQ9 numbers
 * m = 0x3 - Coefficients are signed 12sQ10 numbers
 */
struct camss_params_color_correct {
	struct v4l2_isp_params_block_header header;
	__u16 a[3];
	__u16 b[3];
	__u16 c[3];
	__u16 k[3];
	__u16 m;
} __attribute__((aligned(8)));

#define CAMSS_PARAMS_MAX_PAYLOAD		\
	(sizeof(struct camss_params_wb_gain)	+\
	 sizeof(struct camss_params_chroma_enhan)	+\
	 sizeof(struct camss_params_color_correct))

#endif /* _UAPI_LINUX_CAMSS_CONFIG_H */
