/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SCMI Message Protocol driver QCOM extension header
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef _LINUX_SCMI_QCOM_PROTOCOL_H
#define _LINUX_SCMI_QCOM_PROTOCOL_H

#include <linux/types.h>

#define SCMI_PROTOCOL_QCOM_GENERIC    0x80

struct scmi_protocol_handle;

/**
 * struct qcom_generic_ext_ops - represents the various operations provided
 *				 by QCOM Generic Vendor Protocol
 *
 * @set_param: set parameter specified by param_id and algo_str pair.
 * @get_param: retrieve parameter specified by param_id and algo_str pair.
 * @start_activity: initiate a specific activity defined by algo_str.
 * @stop_activity: halt previously initiated activity defined by algo_str.
 */
struct qcom_generic_ext_ops {
	int (*set_param)(const struct scmi_protocol_handle *ph, void *buf, size_t buf_len,
			 u64 algo_str, u32 param_id);
	int (*get_param)(const struct scmi_protocol_handle *ph, void *buf, size_t buf_len,
			 u64 algo_str, u32 param_id, size_t rx_size);
	int (*start_activity)(const struct scmi_protocol_handle *ph, void *buf, size_t buf_len,
			      u64 algo_str, u32 param_id);
	int (*stop_activity)(const struct scmi_protocol_handle *ph, void *buf, size_t buf_len,
			     u64 algo_str, u32 param_id);
};

#endif /* _LINUX_SCMI_QCOM_PROTOCOL_H */
