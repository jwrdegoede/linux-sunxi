// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/module.h>
#include <linux/scmi_qcom_protocol.h>
#include <linux/string.h>
#include <linux/stringify.h>
#include <linux/types.h>

#include "../../common.h"

/*
 * This protocol is intended as a generic way of exposing a number of Qualcomm
 * SoC specific features through a mixture of pre-determined algorithm string
 * and param_id pairs hosted on the SCMI controller.
 *
 * The QCOM SCMI Vendor Protocol has the protocol id as 0x80 and vendor id set
 * to Qualcomm and the supported version is set to 0x10000. The PROTOCOL_VERSION
 * command returns version 1.0.
 */

/**
 * enum qcom_generic_ext_protocol_cmd - vendor specific commands supported by SCMI Qualcomm
 *                                      generic vendor protocol.
 *
 * @QCOM_SCMI_SET_PARAM: is used to set the parameter of a specific algo_str hosted on
 *			 QCOM SCMI Vendor Protocol. The tx len depends on the algo_str used.
 * @QCOM_SCMI_GET_PARAM: is used to get parameter information of a specific algo_str
 *			 hosted on QCOM SCMI Vendor Protocol. The tx and rx len depends
 *			 on the algo_str used.
 * @QCOM_SCMI_START_ACTIVITY: is used to start the activity performed by the algo_str.
 * @QCOM_SCMI_STOP_ACTIVITY: is used to stop a pre-existing activity performed by the algo_str.
 */
enum qcom_generic_ext_protocol_cmd {
	QCOM_SCMI_SET_PARAM = 0x10,
	QCOM_SCMI_GET_PARAM = 0x11,
	QCOM_SCMI_START_ACTIVITY = 0x12,
	QCOM_SCMI_STOP_ACTIVITY = 0x13,
};

/**
 * struct qcom_scmi_msg - represents the various parameters to be populated
 *                        for using the QCOM SCMI Vendor Protocol
 *
 * @ext_id: reserved, must be zero
 * @algo_low: lower 32 bits of the algo_str
 * @algo_high: upper 32 bits of the algo_str
 * @param_id: serves as token message id to the specific algo_str
 * @buf: serves as the payload to the specified param_id and algo_str pair
 */
struct qcom_scmi_msg {
	__le32 ext_id;
	__le32 algo_low;
	__le32 algo_high;
	__le32 param_id;
	__le32 buf[];
};

/*
 * The firmware only implements a handful of algorithm strings. Keep an
 * allowlist so a client cannot push an unsupported (or garbage) string to
 * the firmware; extend it as new algorithms are added.
 */
static const u64 qcom_scmi_algo_str[] = {
	0x4d454d4c4154ULL, /* "MEMLAT" */
};

static bool qcom_scmi_algo_str_valid(u64 algo_str)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(qcom_scmi_algo_str); i++)
		if (algo_str == qcom_scmi_algo_str[i])
			return true;

	return false;
}

static int qcom_scmi_common_xfer(const struct scmi_protocol_handle *ph,
				 enum qcom_generic_ext_protocol_cmd cmd_id, void *buf,
				 size_t buf_len, u64 algo_str, u32 param_id, size_t rx_size)
{
	struct scmi_xfer *t;
	struct qcom_scmi_msg *msg;
	int ret;

	if (!qcom_scmi_algo_str_valid(algo_str))
		return -EINVAL;

	/* Reject calls where rx_size exceeds buf_len. */
	if (rx_size > buf_len)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, cmd_id, buf_len + sizeof(*msg), rx_size, &t);
	if (ret)
		return ret;

	msg = t->tx.buf;
	msg->ext_id = 0;
	msg->algo_low = cpu_to_le32(lower_32_bits(algo_str));
	msg->algo_high = cpu_to_le32(upper_32_bits(algo_str));
	msg->param_id = cpu_to_le32(param_id);
	if (buf_len)
		memcpy(msg->buf, buf, buf_len);

	ret = ph->xops->do_xfer(ph, t);
	if (!ret && rx_size) {
		/*
		 * The response is returned into the caller's @buf, replacing
		 * the tx payload. Bound the copy to the caller's buffer since
		 * t->rx.len is reported by the firmware, and require at least
		 * the expected rx_size so the caller never reads stale tx data.
		 */
		if (t->rx.len < rx_size || t->rx.len > buf_len)
			ret = -EPROTO;
		else
			memcpy(buf, t->rx.buf, rx_size);
	}
	ph->xops->xfer_put(ph, t);

	return ret;
}

static int qcom_scmi_set_param(const struct scmi_protocol_handle *ph, void *buf, size_t buf_len,
			       u64 algo_str, u32 param_id)
{
	return qcom_scmi_common_xfer(ph, QCOM_SCMI_SET_PARAM, buf, buf_len, algo_str,
				     param_id, 0);
}

static int qcom_scmi_get_param(const struct scmi_protocol_handle *ph, void *buf, size_t buf_len,
			       u64 algo_str, u32 param_id, size_t rx_size)
{
	return qcom_scmi_common_xfer(ph, QCOM_SCMI_GET_PARAM, buf, buf_len, algo_str,
				     param_id, rx_size);
}

static int qcom_scmi_start_activity(const struct scmi_protocol_handle *ph, void *buf,
				    size_t buf_len, u64 algo_str, u32 param_id)
{
	return qcom_scmi_common_xfer(ph, QCOM_SCMI_START_ACTIVITY, buf, buf_len, algo_str,
				     param_id, 0);
}

static int qcom_scmi_stop_activity(const struct scmi_protocol_handle *ph, void *buf,
				   size_t buf_len, u64 algo_str, u32 param_id)
{
	return qcom_scmi_common_xfer(ph, QCOM_SCMI_STOP_ACTIVITY, buf, buf_len, algo_str,
				     param_id, 0);
}

static const struct qcom_generic_ext_ops qcom_proto_ops = {
	.set_param = qcom_scmi_set_param,
	.get_param = qcom_scmi_get_param,
	.start_activity = qcom_scmi_start_activity,
	.stop_activity = qcom_scmi_stop_activity,
};

static int qcom_generic_ext_protocol_init(const struct scmi_protocol_handle *ph)
{
	dev_dbg(ph->dev, "QCOM Generic Vendor Version %d.%d\n",
		PROTOCOL_REV_MAJOR(ph->version), PROTOCOL_REV_MINOR(ph->version));

	return 0;
}

static const struct scmi_protocol qcom_generic_ext = {
	.id = SCMI_PROTOCOL_QCOM_GENERIC,
	.owner = THIS_MODULE,
	.instance_init = &qcom_generic_ext_protocol_init,
	.ops = &qcom_proto_ops,
	.vendor_id = "Qualcomm",
	.supported_version = 0x10000,
};
module_scmi_protocol(qcom_generic_ext);

MODULE_ALIAS("scmi-protocol-" __stringify(SCMI_PROTOCOL_QCOM_GENERIC) "-Qualcomm");
MODULE_AUTHOR("Pragnesh Papaniya <pragnesh.papaniya@oss.qualcomm.com>");
MODULE_AUTHOR("Sibi Sankar <sibi.sankar@oss.qualcomm.com>");
MODULE_DESCRIPTION("QCOM SCMI Generic Vendor Protocol");
MODULE_LICENSE("GPL");
