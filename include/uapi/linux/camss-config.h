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
 * Implements the CLC_WB pipeline module.  The pipeline applies three
 * sequential operations per channel:
 *   1. Subtract sub-offset (black-level substraction)
 *   2. Multiply by gain    (colour balance)
 *   3. Add add-offset      (output pedestal)
 *
 * Gains are 15uQ10 (15-bit unsigned, 10 fractional bits).
 * Offsets are signed and clamped to 12, 13 or 15 bits
 * depending on hardware variant.
 *
 * @header:   block header; @header.type = CAMSS_PARAMS_WB_GAIN
 * @g_gain:   green channel gain (15uQ10, 1024 = 1.0)
 * @b_gain:   blue  channel gain (15uQ10, 1024 = 1.0)
 * @r_gain:   red   channel gain (15uQ10, 1024 = 1.0)
 * @g_sub:    green sub-offset, subtracted before gain
 * @b_sub:    blue  sub-offset, subtracted before gain
 * @r_sub:    red   sub-offset, subtracted before gain
 * @g_add:    green add-offset, added after gain
 * @b_add:    blue  add-offset, added after gain
 * @r_add:    red   add-offset, added after gain
 */
struct camss_params_wb_gain {
	struct v4l2_isp_params_block_header header;
	__u16 g_gain;
	__u16 b_gain;
	__u16 r_gain;
	__s16 g_sub;
	__s16 b_sub;
	__s16 r_sub;
	__s16 g_add;
	__s16 b_add;
	__s16 r_add;
	__u16 _pad[3];
} __aligned(8);

/**
 * struct camss_params_chroma_enhan - RGB to YUV colour transfer matrix
 *
 * Implements the CLC_CHROMA_ENHAN pipeline module. All coefficients are
 * signed 12-bit fixed-point Q3.8 (range roughly -8.0 to +7.996).
 *
 * RGB2Y - Luma (Y) ceofficients
 * Y = v0 * R + v1 * G + v2 * B
 *
 * @luma_v0:  R-to-Y coefficient (12sQ8)
 * @luma_v1:  G-to-Y coefficient (12sQ8)
 * @luma_v2:  B-to-Y coefficient (12sQ8)
 * @luma_k:   Y output offset    (9s,  0 = no offset)
 *
 * RGB2Cb - Chroma (Cb) coefficients
 * Cb = a x ((B - G) + b(R - G)) + KCb
 * with:
 *   a = ap, when (B-G) + b(R-G) > 0; a = am, when (B-G) + b(R-G) ≤ 0;
 *   b = bp when (R-G) > 0; b = bm when (R-G) ≤ 0
 *
 * @coeff_ap: Cb positive coefficient (12sQ8)
 * @coeff_am: Cb negative coefficient (12sQ8)
 * @coeff_bp: Cb positive coefficient (12sQ8)
 * @coeff_bm: Cb negative coefficient (12sQ8)
 * @kcb:      Cb output offset        (11s)
 *
 * RGB2Cr - Chroma (Cr) coefficients:
 * Cr = c x ((R - G) + d(B - G)) + KCr
 * with:
 *   c = cp, when (R-G) + d(B-G) > 0; c = cm, when (R-G) + d(B-G) ≤ 0
 *   d = dp when (B-G) > 0; d = dm when (B-G) ≤ 0
 *
 * @coeff_cp: Cr positive coefficient (12sQ8)
 * @coeff_cm: Cr negative coefficient (12sQ8)
 * @coeff_dp: Cr positive coefficient (12sQ8)
 * @coeff_dm: Cr negative coefficient (12sQ8)
 * @kcr:      Cr output offset        (11s)
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
	__u16 coeff_bp;
	__u16 coeff_bm;
	__u16 coeff_cp;
	__u16 coeff_cm;
	__u16 coeff_dp;
	__u16 coeff_dm;
	__u16 kcb;
	__u16 kcr;
	__u16 _pad[2];
} __aligned(8);

/**
 * struct camss_params_color_correct - colour correction matrix
 *
 * Implements the CLC_CC pipeline module.  The matrix computes:
 *   Out_ch0 (G) = a0*G + b0*B + c0*R + k0
 *   Out_ch1 (B) = a1*G + b1*B + c1*R + k1
 *   Out_ch2 (R) = a2*G + b2*B + c2*R + k2
 *
 * @header:  block header; @header.type = CAMSS_PARAMS_COLOR_CORRECT
 * @a:       G-input coefficients per output channel (12s;
 *           a[0]=Out_G, a[1]=Out_B, a[2]=Out_R)
 * @b:       B-input coefficients (12s)
 * @c:       R-input coefficients (12s)
 * @k:       per-output-channel offsets (typically 9s effective)
 * @qfactor: Q-format selector (2u):
 *           0 = 12sQ7  (range ~-256.0 .. +255.992)
 *           1 = 12sQ8  (range ~-128.0 .. +127.996)
 *           2 = 12sQ9  (range ~-64.0  .. +63.998)
 *           3 = 12sQ10 (range ~-32.0  .. +31.999)
 */
struct camss_params_color_correct {
	struct v4l2_isp_params_block_header header;
	__u16 a[3];
	__u16 b[3];
	__u16 c[3];
	__u16 k[3];
	__u16 qfactor;
	__u16 _pad[3];
} __aligned(8);

#define CAMSS_PARAMS_MAX_PAYLOAD		\
	(sizeof(struct camss_params_wb_gain)	+\
	 sizeof(struct camss_params_chroma_enhan)	+\
	 sizeof(struct camss_params_color_correct))

#endif /* _UAPI_LINUX_CAMSS_CONFIG_H */
