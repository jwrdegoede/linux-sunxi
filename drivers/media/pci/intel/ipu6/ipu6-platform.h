/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2013 - 2023 Intel Corporation */

#ifndef IPU6_PLATFORM_H
#define IPU6_PLATFORM_H

#include "ipu6-fw-isys.h"

#define IPU6_NAME			"intel-ipu6"

#define IPU6SE_FIRMWARE_NAME		"intel/ipu6se_fw.bin"
#define IPU6EP_FIRMWARE_NAME		"intel/ipu6ep_fw.bin"
#define IPU6_FIRMWARE_NAME		"intel/ipu6_fw.bin"
#define IPU6EPMTL_FIRMWARE_NAME		"intel/ipu6epmtl_fw.bin"

/*
 * The following definitions are encoded to the media_device's model field so
 * that the software components which uses IPU6 driver can get the hw stepping
 * information.
 */
#define IPU6_MEDIA_DEV_MODEL_NAME		"ipu6"

#define IPU6SE_ISYS_NUM_STREAMS          IPU6SE_NONSECURE_STREAM_ID_MAX
#define IPU6_ISYS_NUM_STREAMS            IPU6_NONSECURE_STREAM_ID_MAX

extern struct ipu6_isys_internal_pdata isys_ipdata;
extern struct ipu6_psys_internal_pdata psys_ipdata;
extern const struct ipu6_buttress_ctrl isys_buttress_ctrl;
extern const struct ipu6_buttress_ctrl psys_buttress_ctrl;

#endif
