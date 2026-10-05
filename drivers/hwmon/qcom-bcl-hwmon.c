// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm pmic BCL driver for battery overcurrent and
 * battery or system under voltage monitor
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/bitfield.h>
#include <linux/devm-helpers.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* BCL common regmap offset */
#define REVISION1			0x0
#define REVISION2			0x1
#define STATUS				0x8
#define INT_RT_STS			0x10
#define EN_CTL1				0x46

/* BCL GEN1 regmap offsets */
#define VADC_L0_THR			0x48
#define VCMP_L1_THR			0x49
#define VCMP_L2_THR			0x4a
#define IADC_H0_THR			0x4b
#define IADC_H1_THR			0x4c
#define VADC_DATA1			0x76
#define IADC_DATA1			0x86

/* BCL GEN3 regmap offsets */
#define VCMP_L0_THR			0x47
#define IADC_H1_THR_GEN3		0x4d

#define BCL_ALARM_POLLING_MS		50

/**
 * enum bcl_limit_alarm - BCL alarm threshold levels
 * @BCL_LIMIT_ALARM_LVL0: Level 0 alarm threshold
 * @BCL_LIMIT_ALARM_LVL1: Level 1 alarm threshold
 * @BCL_LIMIT_ALARM_LVL2: Level 2 alarm threshold
 * @BCL_LIMIT_ALARM_MAX: sentinel value
 *
 * Defines the threshold levels for BCL monitoring. Each level corresponds
 * to different severity of in or curr conditions.
 */
enum bcl_limit_alarm {
	BCL_LIMIT_ALARM_LVL0,
	BCL_LIMIT_ALARM_LVL1,
	BCL_LIMIT_ALARM_LVL2,

	BCL_LIMIT_ALARM_MAX,
};

/**
 * enum bcl_channel - BCL supported sensor channel type
 * @CHANNEL_IN: in (voltage) channel
 * @CHANNEL_CURR: curr (current) channel
 * @CHANNEL_MAX: sentinel value
 *
 * Defines the supported channel types for bcl.
 */
enum bcl_channel {
	CHANNEL_IN,
	CHANNEL_CURR,

	CHANNEL_MAX,
};

/**
 * enum bcl_thresh_type - voltage or current threshold representation type
 * @THRESH_TYPE_ADC: Raw ADC value representation
 * @THRESH_TYPE_INDEX: Index-based voltage or current representation
 *
 * Specifies how voltage or current thresholds are stored and interpreted in
 * registers. Some PMICs use raw ADC values while others use indexed values.
 */
enum bcl_thresh_type {
	THRESH_TYPE_ADC,
	THRESH_TYPE_INDEX,
};

/**
 * enum bcl_battery_config - Battery configuration types
 * @BCL_BATT_1S: Single cell battery
 * @BCL_BATT_2S: Two cells in series
 * @BCL_BATT_3S: Three cells in series
 */
enum bcl_battery_config {
	BCL_BATT_1S,
	BCL_BATT_2S,
	BCL_BATT_3S,
};

/**
 * enum bcl_fields - BCL register field identifiers
 * @F_V_MAJOR: Major revision info field
 * @F_V_MINOR: Minor revision info field
 * @F_CTL_EN: Monitor enable control field
 * @F_LVL0_ALARM: Level 0 alarm status field
 * @F_LVL1_ALARM: Level 1 alarm status field
 * @F_LVL2_ALARM: Level 2 alarm status field
 * @F_IN_L0_THR: voltage level 0 threshold field
 * @F_IN_L1_THR: voltage level 1 threshold field
 * @F_IN_L2_THR: voltage level 2 threshold field
 * @F_IN_INPUT: voltage input data field (LSB for 16-bit data)
 * @F_CURR_H0_THR: current level 0 threshold field
 * @F_CURR_H1_THR: current level 1 threshold field
 * @F_CURR_INPUT: current input data field (LSB for 16-bit data)
 * @F_MAX_FIELDS: sentinel value
 *
 * Enumeration of all register fields used by the BCL driver for accessing
 * registers through regmap fields.
 */
enum bcl_fields {
	/* Common fields - present in all BCL variants */
	F_V_MAJOR,
	F_V_MINOR,
	F_CTL_EN,
	F_LVL0_ALARM,
	F_LVL1_ALARM,
	F_LVL2_ALARM,

	/* Voltage monitoring fields */
	F_IN_L0_THR,
	F_IN_L1_THR,
	F_IN_L2_THR,
	F_IN_INPUT,

	/* Current monitoring fields */
	F_CURR_H0_THR,
	F_CURR_H1_THR,
	F_CURR_INPUT,

	F_MAX_FIELDS
};

/**
 * struct bcl_channel_cfg - BCL channel related configuration
 * @default_scale_nu:  Default scaling factor in nano unit
 * @base_mu: Base threshold value in milli unit
 * @max_mu: Maximum threshold value in milli unit
 * @step_mu: step increment value between two indexed threshold value in milli unit
 * @thresh_type: Array specifying threshold representation type for each alarm level
 *
 * Contains hardware-specific configuration and scaling parameters for different
 * channels (voltage and current).
 */
struct bcl_channel_cfg {
	u32 default_scale_nu;
	u32 base_mu;
	u32 max_mu;
	u32 step_mu;
	u8 thresh_type[BCL_LIMIT_ALARM_MAX];
};

/**
 * struct bcl_desc - BCL device descriptor
 * @reg_fields: Array of register field definitions for this device variant
 * @channel_cfg: Array of channel configurations indexed by battery config
 *               PMICs without battery detection: only [BCL_BATT_1S] is defined
 *               PMICs with battery detection: [BCL_BATT_2S], [BCL_BATT_3S]
 * @battery_config_field: Register field for battery configuration detection
 *                        NOTE: This is an ABSOLUTE address, not relative to BCL base
 *                        Set to REG_FIELD(0, 0, 0) if not used
 * @num_reg_fields: Number of register field definitions for this device variant
 * @data_field_bits_size: data read register bit size
 * @thresh_field_bits_size: lsb bit size those are not included in threshold register
 * @in_mon_enabled: Voltage monitoring enabled at boot
 * @in_input_enabled: Voltage input reading enabled at boot
 * @curr_mon_enabled: Current monitoring enabled at boot
 *
 * Contains hardware-specific configuration and scaling parameters for different
 * BCL variants. Each PMIC model may have different register layouts and
 * conversion factors.
 */
struct bcl_desc {
	const struct reg_field *reg_fields;
	struct bcl_channel_cfg channel_cfg[BCL_BATT_3S + 1][CHANNEL_MAX];
	const struct reg_field battery_config_field;
	u8 num_reg_fields;
	u8 data_field_bits_size;
	u8 thresh_field_bits_size;
	bool in_mon_enabled;
	bool in_input_enabled;
	bool curr_mon_enabled;
};

/**
 * struct bcl_alarm_data - BCL alarm interrupt data
 * @irq: IRQ number assigned to this alarm
 * @irq_enabled: Flag indicating if IRQ is enabled
 * @irq_wake_enabled: Flag indicating if IRQ wake is enabled
 * @shutting_down: Flag preventing work from re-enabling IRQ during teardown
 * @type: Alarm level type (LVL0, or LVL1)
 * @device: Pointer to parent BCL device structure
 * @a_lock: Mutex for protecting alarm state
 * @alarm_poll_work: delayed_work to poll alarm status
 *
 * Stores interrupt-related information for each alarm threshold level.
 * Used by the IRQ handler to identify which alarm triggered.
 */
struct bcl_alarm_data {
	int			irq;
	bool			irq_enabled;
	bool			irq_wake_enabled;
	bool			shutting_down;
	enum bcl_limit_alarm	type;
	void			*device;
	/* Protects alarm IRQ enable/disable state */
	struct mutex		a_lock;
	struct delayed_work	alarm_poll_work;
};

/**
 * struct bcl_device - Main BCL device structure
 * @dev: Pointer to device structure
 * @regmap: Regmap for accessing PMIC registers
 * @base: the BCL regbase offset from regmap
 * @fields: Array of regmap fields for register access
 * @bcl_alarms: Array of alarm data structures for each threshold level
 * @lock: Mutex for protecting concurrent hardware access
 * @last_in_input: Last valid voltage input reading in millivolts
 * @last_curr_input: Last valid current input reading in milliamps
 * @last_in_updated: Timestamp of last voltage input update
 * @last_curr_updated: Timestamp of last current input update
 * @desc: Pointer to device descriptor with hardware-specific parameters
 * @hwmon_dev: Pointer to registered hwmon device
 * @hwmon_name: Sanitized name for hwmon device
 * @batt_config: Detected battery configuration (only for SMB2360/2370)
 * @batt_config_regfield: Regmap field for battery configuration register
 *
 * Main driver structure containing all state and configuration for a BCL
 * monitoring instance. Manages voltage and current monitoring, thresholds,
 * and alarm handling.
 */
struct bcl_device {
	struct device		*dev;
	struct regmap		*regmap;
	u16			base;
	struct regmap_field	*fields[F_MAX_FIELDS];
	struct bcl_alarm_data	bcl_alarms[BCL_LIMIT_ALARM_MAX];
	/* Protects hardware register access and device state */
	struct mutex		lock;
	u32			last_in_input;
	s32			last_curr_input;
	unsigned long		last_in_updated;
	unsigned long		last_curr_updated;
	const struct bcl_desc	*desc;
	struct device		*hwmon_dev;
	char			*hwmon_name;
	enum bcl_battery_config	batt_config;
	struct regmap_field	*batt_config_regfield;
};

static const u8 in_attr_to_lvl_map[] = {
	[hwmon_in_min] = BCL_LIMIT_ALARM_LVL0,
	[hwmon_in_lcrit] = BCL_LIMIT_ALARM_LVL1,
	[hwmon_in_lemergency] = BCL_LIMIT_ALARM_LVL2,
	[hwmon_in_min_alarm] = BCL_LIMIT_ALARM_LVL0,
	[hwmon_in_lcrit_alarm] = BCL_LIMIT_ALARM_LVL1,
	[hwmon_in_lemergency_alarm] = BCL_LIMIT_ALARM_LVL2,
};

static const u8 in_lvl_to_attr_map[BCL_LIMIT_ALARM_MAX] = {
	[BCL_LIMIT_ALARM_LVL0] = hwmon_in_min_alarm,
	[BCL_LIMIT_ALARM_LVL1] = hwmon_in_lcrit_alarm,
	[BCL_LIMIT_ALARM_LVL2] = hwmon_in_lemergency_alarm,
};

static const u8 curr_attr_to_lvl_map[] = {
	[hwmon_curr_max] = BCL_LIMIT_ALARM_LVL0,
	[hwmon_curr_crit] = BCL_LIMIT_ALARM_LVL1,
	[hwmon_curr_max_alarm] = BCL_LIMIT_ALARM_LVL0,
	[hwmon_curr_crit_alarm] = BCL_LIMIT_ALARM_LVL1,
	[hwmon_curr_emergency_alarm] = BCL_LIMIT_ALARM_LVL2,
};

static const u8 curr_lvl_to_attr_map[BCL_LIMIT_ALARM_MAX] = {
	[BCL_LIMIT_ALARM_LVL0] = hwmon_curr_max_alarm,
	[BCL_LIMIT_ALARM_LVL1] = hwmon_curr_crit_alarm,
	[BCL_LIMIT_ALARM_LVL2] = hwmon_curr_emergency_alarm,
};

/* Interrupt names for each alarm level */
static const char * const bcl_int_names[BCL_LIMIT_ALARM_MAX] = {
	[BCL_LIMIT_ALARM_LVL0] = "max-min",
	[BCL_LIMIT_ALARM_LVL1] = "critical",
	[BCL_LIMIT_ALARM_LVL2] = "emergency",
};

static const struct reg_field bcl_pm7250b_reg_fields[] = {
	[F_V_MAJOR]	= REG_FIELD(REVISION2, 0, 7),
	[F_V_MINOR]	= REG_FIELD(REVISION1, 0, 7),
	[F_CTL_EN]	= REG_FIELD(EN_CTL1, 7, 7),
	[F_LVL0_ALARM]	= REG_FIELD(STATUS, 0, 0),
	[F_LVL1_ALARM]	= REG_FIELD(STATUS, 1, 1),
	[F_LVL2_ALARM]	= REG_FIELD(STATUS, 2, 2),
	[F_IN_L0_THR]	= REG_FIELD(VADC_L0_THR, 0, 7),
	[F_IN_L1_THR]	= REG_FIELD(VCMP_L1_THR, 0, 5),
	[F_IN_L2_THR]	= REG_FIELD(VCMP_L2_THR, 0, 5),
	[F_IN_INPUT]	= REG_FIELD(VADC_DATA1, 0, 7),
	[F_CURR_H0_THR]	= REG_FIELD(IADC_H0_THR, 0, 7),
	[F_CURR_H1_THR]	= REG_FIELD(IADC_H1_THR, 0, 7),
	[F_CURR_INPUT]	= REG_FIELD(IADC_DATA1, 0, 7),
};

static const struct reg_field bcl_pm7550ba_reg_fields[] = {
	[F_V_MAJOR]	= REG_FIELD(REVISION2, 0, 7),
	[F_V_MINOR]	= REG_FIELD(REVISION1, 0, 7),
	[F_CTL_EN]	= REG_FIELD(EN_CTL1, 7, 7),
	[F_LVL0_ALARM]	= REG_FIELD(STATUS, 0, 0),
	[F_LVL1_ALARM]	= REG_FIELD(STATUS, 1, 1),
	[F_LVL2_ALARM]	= REG_FIELD(STATUS, 2, 2),
	[F_IN_L0_THR]	= REG_FIELD(VADC_L0_THR, 0, 7),
	[F_IN_L1_THR]	= REG_FIELD(VCMP_L1_THR, 0, 5),
	[F_IN_L2_THR]	= REG_FIELD(VCMP_L2_THR, 0, 5),
	[F_IN_INPUT]	= REG_FIELD(VADC_DATA1, 0, 7),
	[F_CURR_H0_THR]	= REG_FIELD(IADC_H0_THR, 0, 7),
	[F_CURR_H1_THR]	= REG_FIELD(IADC_H1_THR_GEN3, 0, 7),
	[F_CURR_INPUT]	= REG_FIELD(IADC_DATA1, 0, 7),
};

static const struct reg_field bcl_pm8350c_reg_fields[] = {
	[F_V_MAJOR]	= REG_FIELD(REVISION2, 0, 7),
	[F_V_MINOR]	= REG_FIELD(REVISION1, 0, 7),
	[F_CTL_EN]	= REG_FIELD(EN_CTL1, 7, 7),
	[F_LVL0_ALARM]	= REG_FIELD(STATUS, 0, 0),
	[F_LVL1_ALARM]	= REG_FIELD(STATUS, 1, 1),
	[F_LVL2_ALARM]	= REG_FIELD(STATUS, 2, 2),
	[F_IN_L0_THR]	= REG_FIELD(VADC_L0_THR, 0, 7),
	[F_IN_L1_THR]	= REG_FIELD(VCMP_L1_THR, 0, 5),
	[F_IN_L2_THR]	= REG_FIELD(VCMP_L2_THR, 0, 5),
};

static const struct reg_field bcl_pm8550_reg_fields[] = {
	[F_V_MAJOR]	= REG_FIELD(REVISION2, 0, 7),
	[F_V_MINOR]	= REG_FIELD(REVISION1, 0, 7),
	[F_CTL_EN]	= REG_FIELD(EN_CTL1, 7, 7),
	[F_LVL0_ALARM]	= REG_FIELD(STATUS, 0, 0),
	[F_LVL1_ALARM]	= REG_FIELD(STATUS, 1, 1),
	[F_LVL2_ALARM]	= REG_FIELD(STATUS, 2, 2),
	[F_IN_L0_THR]	= REG_FIELD(VCMP_L0_THR, 0, 5),
	[F_IN_L1_THR]	= REG_FIELD(VCMP_L1_THR, 0, 5),
	[F_IN_L2_THR]	= REG_FIELD(VCMP_L2_THR, 0, 5),
};

static const struct reg_field bcl_pmh0101_reg_fields[] = {
	[F_V_MAJOR]	= REG_FIELD(REVISION2, 0, 7),
	[F_V_MINOR]	= REG_FIELD(REVISION1, 0, 7),
	[F_CTL_EN]	= REG_FIELD(EN_CTL1, 7, 7),
	[F_LVL0_ALARM]	= REG_FIELD(STATUS, 0, 0),
	[F_LVL1_ALARM]	= REG_FIELD(STATUS, 1, 1),
	[F_LVL2_ALARM]	= REG_FIELD(STATUS, 2, 2),
	[F_IN_L0_THR]	= REG_FIELD(VCMP_L0_THR, 0, 6),
	[F_IN_L1_THR]	= REG_FIELD(VCMP_L1_THR, 0, 6),
	[F_IN_L2_THR]	= REG_FIELD(VCMP_L2_THR, 0, 6),
};

static const struct reg_field bcl_pmih0108_reg_fields[] = {
	[F_V_MAJOR]	= REG_FIELD(REVISION2, 0, 7),
	[F_V_MINOR]	= REG_FIELD(REVISION1, 0, 7),
	[F_CTL_EN]	= REG_FIELD(EN_CTL1, 7, 7),
	[F_LVL0_ALARM]	= REG_FIELD(STATUS, 0, 0),
	[F_LVL1_ALARM]	= REG_FIELD(STATUS, 1, 1),
	[F_LVL2_ALARM]	= REG_FIELD(STATUS, 2, 2),
	[F_IN_L0_THR]	= REG_FIELD(VADC_L0_THR, 0, 7),
	[F_IN_L1_THR]	= REG_FIELD(VCMP_L1_THR, 0, 5),
	[F_IN_L2_THR]	= REG_FIELD(VCMP_L2_THR, 0, 5),
	[F_IN_INPUT]	= REG_FIELD(VADC_DATA1, 0, 7),
	[F_CURR_H0_THR]	= REG_FIELD(IADC_H0_THR, 0, 7),
	[F_CURR_H1_THR]	= REG_FIELD(IADC_H1_THR_GEN3, 0, 7),
	[F_CURR_INPUT]	= REG_FIELD(IADC_DATA1, 0, 7),
};

static const struct reg_field bcl_smb2360_reg_fields[] = {
	[F_V_MAJOR]	= REG_FIELD(REVISION2, 0, 7),
	[F_V_MINOR]	= REG_FIELD(REVISION1, 0, 7),
	[F_CTL_EN]	= REG_FIELD(EN_CTL1, 7, 7),
	[F_LVL0_ALARM]	= REG_FIELD(STATUS, 0, 0),
	[F_LVL1_ALARM]	= REG_FIELD(STATUS, 1, 1),
	[F_LVL2_ALARM]	= REG_FIELD(STATUS, 2, 2),
	[F_IN_L0_THR]	= REG_FIELD(VADC_L0_THR, 0, 7),
	[F_IN_L1_THR]	= REG_FIELD(VCMP_L1_THR, 0, 5),
	[F_IN_L2_THR]	= REG_FIELD(VCMP_L2_THR, 0, 5),
	[F_IN_INPUT]	= REG_FIELD(VADC_DATA1, 0, 7),
	[F_CURR_H0_THR]	= REG_FIELD(IADC_H0_THR, 0, 7),
	[F_CURR_H1_THR]	= REG_FIELD(IADC_H1_THR_GEN3, 0, 7),
	[F_CURR_INPUT]	= REG_FIELD(IADC_DATA1, 0, 7),
};

static const struct reg_field bcl_smb2370_reg_fields[] = {
	[F_V_MAJOR]	= REG_FIELD(REVISION2, 0, 7),
	[F_V_MINOR]	= REG_FIELD(REVISION1, 0, 7),
	[F_CTL_EN]	= REG_FIELD(EN_CTL1, 7, 7),
	[F_LVL0_ALARM]	= REG_FIELD(STATUS, 0, 0),
	[F_LVL1_ALARM]	= REG_FIELD(STATUS, 1, 1),
	[F_LVL2_ALARM]	= REG_FIELD(STATUS, 2, 2),
	[F_IN_L0_THR]	= REG_FIELD(VADC_L0_THR, 0, 7),
	[F_IN_L1_THR]	= REG_FIELD(VCMP_L1_THR, 0, 5),
	[F_IN_L2_THR]	= REG_FIELD(VCMP_L2_THR, 0, 5),
	[F_IN_INPUT]	= REG_FIELD(VADC_DATA1, 0, 7),
	[F_CURR_H0_THR]	= REG_FIELD(IADC_H0_THR, 0, 7),
	[F_CURR_H1_THR]	= REG_FIELD(IADC_H1_THR_GEN3, 0, 7),
	[F_CURR_INPUT]	= REG_FIELD(IADC_DATA1, 0, 7),
};

static const struct bcl_desc pm7250b_data = {
	.reg_fields = bcl_pm7250b_reg_fields,
	.num_reg_fields = F_CURR_INPUT + 1,
	.data_field_bits_size = 8,
	.thresh_field_bits_size = 7,
	.battery_config_field = REG_FIELD(0, 0, 0),
	.in_mon_enabled = true,
	.in_input_enabled = true,
	.curr_mon_enabled = true,
	.channel_cfg[BCL_BATT_1S][CHANNEL_IN] = {
		.base_mu = 2250,
		.max_mu = 3600,
		.step_mu = 25,
		.default_scale_nu = 194637,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
	.channel_cfg[BCL_BATT_1S][CHANNEL_CURR] = {
		.max_mu = 9918,
		.default_scale_nu = 305180,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_ADC},
	},
};

static const struct bcl_desc pm8350c_data = {
	.reg_fields = bcl_pm8350c_reg_fields,
	.num_reg_fields = F_IN_L2_THR + 1,
	.data_field_bits_size = 0,
	.thresh_field_bits_size = 8,
	.battery_config_field = REG_FIELD(0, 0, 0),
	.in_mon_enabled = true,
	.in_input_enabled = false,
	.curr_mon_enabled = false,
	.channel_cfg[BCL_BATT_1S][CHANNEL_IN] = {
		.base_mu = 2250,
		.max_mu = 3600,
		.step_mu = 25,
		.default_scale_nu = 194637,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
};

static const struct bcl_desc pm7550ba_data = {
	.reg_fields = bcl_pm7550ba_reg_fields,
	.num_reg_fields = F_CURR_INPUT + 1,
	.data_field_bits_size = 8,
	.thresh_field_bits_size = 8,
	.battery_config_field = REG_FIELD(0, 0, 0),
	.in_mon_enabled = true,
	.in_input_enabled = true,
	.curr_mon_enabled = true,
	.channel_cfg[BCL_BATT_1S][CHANNEL_IN] = {
		.base_mu = 2250,
		.max_mu = 3600,
		.step_mu = 25,
		.default_scale_nu = 194637,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
	.channel_cfg[BCL_BATT_1S][CHANNEL_CURR] = {
		.max_mu = 11901,
		.default_scale_nu = 366220,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_ADC},
	},
};

static const struct bcl_desc pm8550_data = {
	.reg_fields = bcl_pm8550_reg_fields,
	.num_reg_fields = F_IN_L2_THR + 1,
	.data_field_bits_size = 0,
	.thresh_field_bits_size = 6,
	.battery_config_field = REG_FIELD(0, 0, 0),
	.in_mon_enabled = true,
	.in_input_enabled = false,
	.curr_mon_enabled = false,
	.channel_cfg[BCL_BATT_1S][CHANNEL_IN] = {
		.base_mu = 2250,
		.max_mu = 3600,
		.step_mu = 25,
		.thresh_type = {THRESH_TYPE_INDEX, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
};

static const struct bcl_desc pmih0108_data = {
	.reg_fields = bcl_pmih0108_reg_fields,
	.num_reg_fields = F_CURR_INPUT + 1,
	.data_field_bits_size = 16,
	.thresh_field_bits_size = 8,
	.battery_config_field = REG_FIELD(0, 0, 0),
	.in_mon_enabled = true,
	.in_input_enabled = true,
	.curr_mon_enabled = true,
	.channel_cfg[BCL_BATT_1S][CHANNEL_IN] = {
		.base_mu = 2250,
		.max_mu = 3600,
		.step_mu = 25,
		.default_scale_nu = 194637,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
	.channel_cfg[BCL_BATT_1S][CHANNEL_CURR] = {
		.max_mu = 19836,
		.default_scale_nu = 610370,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_ADC},
	},
};

static const struct bcl_desc pmh0101_data = {
	.reg_fields = bcl_pmh0101_reg_fields,
	.num_reg_fields = F_IN_L2_THR + 1,
	.thresh_field_bits_size = 7,
	.battery_config_field = REG_FIELD(0, 0, 0),
	.in_mon_enabled = true,
	.in_input_enabled = false,
	.curr_mon_enabled = false,
	.channel_cfg[BCL_BATT_1S][CHANNEL_IN] = {
		.base_mu = 1500,
		.max_mu = 4000,
		.step_mu = 25,
		.thresh_type = {THRESH_TYPE_INDEX, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
};

/* Register 0x2a50 mapping: 0 -> 2S, 1 -> 3S */
static const struct bcl_desc smb2360_data = {
	.reg_fields = bcl_smb2360_reg_fields,
	.num_reg_fields = F_CURR_INPUT + 1,
	.data_field_bits_size = 8,
	.thresh_field_bits_size = 8,
	.battery_config_field = REG_FIELD(0x2a50, 0, 1),
	.in_mon_enabled = true,
	.in_input_enabled = true,
	.curr_mon_enabled = true,
	.channel_cfg[BCL_BATT_2S][CHANNEL_IN] = {
		.base_mu = 4500,
		.max_mu = 8400,
		.step_mu = 50,
		.default_scale_nu = 432918,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
	.channel_cfg[BCL_BATT_2S][CHANNEL_CURR] = {
		.max_mu = 17575,
		.default_scale_nu = 540679,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_ADC},
	},
	.channel_cfg[BCL_BATT_3S][CHANNEL_IN] = {
		.base_mu = 6750,
		.max_mu = 12600,
		.step_mu = 75,
		.default_scale_nu = 648790,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
	.channel_cfg[BCL_BATT_3S][CHANNEL_CURR] = {
		.max_mu = 17575,
		.default_scale_nu = 540679,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_ADC},
	},
};

static const struct bcl_desc smb2370_data = {
	.reg_fields = bcl_smb2370_reg_fields,
	.num_reg_fields = F_CURR_INPUT + 1,
	.data_field_bits_size = 16,
	.thresh_field_bits_size = 8,
	.battery_config_field = REG_FIELD(0x2a50, 0, 1),
	.in_mon_enabled = true,
	.in_input_enabled = true,
	.curr_mon_enabled = true,
	.channel_cfg[BCL_BATT_2S][CHANNEL_IN] = {
		.base_mu = 4500,
		.max_mu = 8400,
		.step_mu = 50,
		.default_scale_nu = 432918,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
	.channel_cfg[BCL_BATT_2S][CHANNEL_CURR] = {
		.max_mu = 46852,
		.default_scale_nu = 1441603,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_ADC},
	},
	.channel_cfg[BCL_BATT_3S][CHANNEL_IN] = {
		.base_mu = 6750,
		.max_mu = 12600,
		.step_mu = 75,
		.default_scale_nu = 648790,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_INDEX, THRESH_TYPE_INDEX},
	},
	.channel_cfg[BCL_BATT_3S][CHANNEL_CURR] = {
		.max_mu = 46852,
		.default_scale_nu = 1441603,
		.thresh_type = {THRESH_TYPE_ADC, THRESH_TYPE_ADC},
	},
};

/**
 * bcl_convert_raw_to_milliunit - Convert raw value to milli unit
 * @bcl: BCL device structure
 * @raw_val: Raw ADC value from hardware (signed for current, unsigned for voltage)
 * @type: type of the channel, in or curr
 * @field_width: bits size for data or threshold field
 *
 * Return: value in milli unit
 */
static int bcl_convert_raw_to_milliunit(const struct bcl_device *bcl,
					s32 raw_val,
					enum bcl_channel type,
					u8 field_width)
{
	const struct bcl_desc *desc = bcl->desc;
	u32 def_scale = desc->channel_cfg[bcl->batt_config][type].default_scale_nu;
	u32 scaling_factor = (field_width > 8) ? def_scale : (def_scale << field_width);

	return DIV_S64_ROUND_CLOSEST((s64)raw_val * scaling_factor, 1000000);
}

/**
 * bcl_convert_milliunit_to_raw - Convert milli unit to raw value
 * @bcl: BCL device structure
 * @mval: threshold value in milli unit
 * @type: type of the channel, in or curr
 * @field_width: bits size for data or threshold field
 *
 * Return: Raw ADC value for hardware
 */
static unsigned int bcl_convert_milliunit_to_raw(const struct bcl_device *bcl,
						 int mval,
						 enum bcl_channel type,
						 u8 field_width)
{
	const struct bcl_desc *desc = bcl->desc;
	u32 def_scale = desc->channel_cfg[bcl->batt_config][type].default_scale_nu;
	u32 scaling_factor = (field_width > 8) ? def_scale : (def_scale << field_width);

	return DIV_ROUND_CLOSEST_ULL((u64)mval * 1000000, scaling_factor);
}

/**
 * bcl_convert_milliunit_to_index - Convert milliunit to in or curr index
 * @bcl: BCL device structure
 * @val: in or curr value in milli unit
 * @type: type of the channel, in or curr
 *
 * Converts a value in milli unit to an index for BCL that use indexed thresholds.
 *
 * Return: Index value
 */
static unsigned int bcl_convert_milliunit_to_index(const struct bcl_device *bcl,
						   int val,
						   enum bcl_channel type)
{
	const struct bcl_desc *desc = bcl->desc;
	const struct bcl_channel_cfg *cfg = &desc->channel_cfg[bcl->batt_config][type];
	u64 diff = (u64)val - cfg->base_mu;

	return DIV_ROUND_CLOSEST_ULL(diff, cfg->step_mu);
}

/**
 * bcl_convert_index_to_milliunit - Convert in or curr index to milli unit
 * @bcl: BCL device structure
 * @val: index value
 * @type: type of the channel, in or curr
 *
 * Converts an index value to milli unit for BCL that use indexed thresholds.
 *
 * Return: Value in millivolts or milliamps
 */
static unsigned int bcl_convert_index_to_milliunit(const struct bcl_device *bcl,
						   int val,
						   enum bcl_channel type)
{
	const struct bcl_desc *desc = bcl->desc;
	const struct bcl_channel_cfg *cfg = &desc->channel_cfg[bcl->batt_config][type];

	return cfg->base_mu + val * cfg->step_mu;
}

static int bcl_validate_threshold_fits(const struct bcl_device *bcl,
				       const struct reg_field *reg_field,
				       u32 raw_val,
				       long thresh,
				       const char *unit)
{
	unsigned int field_width;
	unsigned int field_mask;

	if (WARN_ON(reg_field->msb < reg_field->lsb))
		return -EINVAL;

	field_width = reg_field->msb - reg_field->lsb + 1;
	field_mask = GENMASK(field_width - 1, 0);

	if (raw_val > field_mask) {
		dev_err(bcl->dev,
			"Threshold value %ld %s (raw: %u) exceeds field capacity (max: %u)\n",
			thresh, unit, raw_val, field_mask);
		return -EINVAL;
	}

	return 0;
}

static int bcl_in_thresh_write(struct bcl_device *bcl, long value, enum bcl_limit_alarm lvl)
{
	const struct bcl_desc *desc = bcl->desc;
	struct regmap_field *field = bcl->fields[F_IN_L0_THR + lvl];
	u32 raw_val;
	int ret;

	int thresh = clamp_val(value, desc->channel_cfg[bcl->batt_config][CHANNEL_IN].base_mu,
			       desc->channel_cfg[bcl->batt_config][CHANNEL_IN].max_mu);

	if (desc->channel_cfg[bcl->batt_config][CHANNEL_IN].thresh_type[lvl] == THRESH_TYPE_ADC)
		raw_val = bcl_convert_milliunit_to_raw(bcl, thresh, CHANNEL_IN,
						       desc->thresh_field_bits_size);
	else
		raw_val = bcl_convert_milliunit_to_index(bcl, thresh, CHANNEL_IN);

	ret = bcl_validate_threshold_fits(bcl, &desc->reg_fields[F_IN_L0_THR + lvl],
					  raw_val, thresh, "mV");
	if (ret)
		return ret;

	return regmap_field_write(field, raw_val);
}

static int bcl_curr_thresh_write(struct bcl_device *bcl, long value, enum bcl_limit_alarm lvl)
{
	const struct bcl_desc *desc = bcl->desc;
	struct regmap_field *field = bcl->fields[F_CURR_H0_THR + lvl];
	u32 raw_val;
	int ret;

	int thresh = clamp_val(value, 0, desc->channel_cfg[bcl->batt_config][CHANNEL_CURR].max_mu);

	if (desc->channel_cfg[bcl->batt_config][CHANNEL_CURR].thresh_type[lvl] == THRESH_TYPE_ADC)
		raw_val = bcl_convert_milliunit_to_raw(bcl, thresh, CHANNEL_CURR,
						       desc->thresh_field_bits_size);
	else
		raw_val = bcl_convert_milliunit_to_index(bcl, thresh, CHANNEL_CURR);

	ret = bcl_validate_threshold_fits(bcl, &desc->reg_fields[F_CURR_H0_THR + lvl],
					  raw_val, thresh, "mA");
	if (ret)
		return ret;

	return regmap_field_write(field, raw_val);
}

static int bcl_in_thresh_read(struct bcl_device *bcl, enum bcl_limit_alarm lvl, long *out)
{
	int ret, thresh;
	u32 raw_val = 0;
	const struct bcl_desc *desc = bcl->desc;

	ret = regmap_field_read(bcl->fields[F_IN_L0_THR + lvl], &raw_val);
	if (ret)
		return ret;

	if (desc->channel_cfg[bcl->batt_config][CHANNEL_IN].thresh_type[lvl] == THRESH_TYPE_ADC)
		thresh = bcl_convert_raw_to_milliunit(bcl, raw_val, CHANNEL_IN,
						      desc->thresh_field_bits_size);
	else
		thresh = bcl_convert_index_to_milliunit(bcl, raw_val, CHANNEL_IN);

	*out = thresh;

	return 0;
}

static int bcl_curr_thresh_read(struct bcl_device *bcl, enum bcl_limit_alarm lvl, long *out)
{
	int ret, thresh;
	u32 raw_val = 0;
	const struct bcl_desc *desc = bcl->desc;

	ret = regmap_field_read(bcl->fields[F_CURR_H0_THR + lvl], &raw_val);
	if (ret)
		return ret;

	if (desc->channel_cfg[bcl->batt_config][CHANNEL_CURR].thresh_type[lvl] == THRESH_TYPE_ADC)
		thresh = bcl_convert_raw_to_milliunit(bcl, raw_val, CHANNEL_CURR,
						      desc->thresh_field_bits_size);
	else
		thresh = bcl_convert_index_to_milliunit(bcl, raw_val, CHANNEL_CURR);

	*out = thresh;

	return 0;
}

static int bcl_curr_input_read(struct bcl_device *bcl, long *out)
{
	int ret;
	u32 raw_val = 0;
	s32 signed_val;
	const struct bcl_desc *desc = bcl->desc;

	/* Return cached value if read too soon after last update */
	if (bcl->last_curr_updated &&
	    time_before(jiffies, bcl->last_curr_updated + HZ)) {
		*out = bcl->last_curr_input;
		return 0;
	}

	if (desc->data_field_bits_size == 16) {
		u8 buf[2];
		unsigned int reg = bcl->base + desc->reg_fields[F_CURR_INPUT].reg;

		/*
		 * Use a single bulk read for the LSB/MSB pair so the
		 * hardware returns both bytes from one bus transaction,
		 * avoiding a torn read if the ADC updates in between.
		 */
		ret = regmap_bulk_read(bcl->regmap, reg, buf, sizeof(buf));
		if (ret)
			return ret;
		raw_val = buf[0] | FIELD_PREP(GENMASK(15, 8), buf[1]);
	} else {
		ret = regmap_field_read(bcl->fields[F_CURR_INPUT], &raw_val);
		if (ret)
			return ret;
	}

	/*
	 * Current ADC reading is in 2's complement form.
	 * Sign extend the value based on data field bit size.
	 */
	signed_val = sign_extend32(raw_val, desc->data_field_bits_size - 1);

	bcl->last_curr_input =
		bcl_convert_raw_to_milliunit(bcl, signed_val, CHANNEL_CURR,
					     desc->data_field_bits_size);
	bcl->last_curr_updated = jiffies;

	*out = bcl->last_curr_input;

	return 0;
}

static int bcl_in_input_read(struct bcl_device *bcl, long *out)
{
	int ret;
	u32 raw_val = 0;
	const struct bcl_desc *desc = bcl->desc;

	/* Return cached value if read too soon after last update */
	if (bcl->last_in_updated &&
	    time_before(jiffies, bcl->last_in_updated + HZ)) {
		*out = bcl->last_in_input;
		return 0;
	}

	if (desc->data_field_bits_size == 16) {
		u8 buf[2];
		unsigned int reg = bcl->base + desc->reg_fields[F_IN_INPUT].reg;

		/*
		 * Use a single bulk read for the LSB/MSB pair so the
		 * hardware returns both bytes from one bus transaction,
		 * avoiding a torn read if the ADC updates in between.
		 */
		ret = regmap_bulk_read(bcl->regmap, reg, buf, sizeof(buf));
		if (ret)
			return ret;
		raw_val = buf[0] | FIELD_PREP(GENMASK(15, 8), buf[1]);
	} else {
		ret = regmap_field_read(bcl->fields[F_IN_INPUT], &raw_val);
		if (ret)
			return ret;
	}

	bcl->last_in_input =
		bcl_convert_raw_to_milliunit(bcl, raw_val, CHANNEL_IN,
					     desc->data_field_bits_size);
	bcl->last_in_updated = jiffies;

	*out = bcl->last_in_input;

	return 0;
}

static int bcl_read_alarm_status(struct bcl_device *bcl,
				 enum bcl_limit_alarm lvl, long *status)
{
	int ret;
	u32 raw_val = 0;

	ret = regmap_field_read(bcl->fields[F_LVL0_ALARM + lvl], &raw_val);
	if (ret)
		return ret;

	*status = raw_val;

	return 0;
}

static unsigned int bcl_get_version_major(const struct bcl_device *bcl)
{
	u32 raw_val = 0;

	regmap_field_read(bcl->fields[F_V_MAJOR], &raw_val);

	return raw_val;
}

static unsigned int bcl_get_version_minor(const struct bcl_device *bcl)
{
	u32 raw_val = 0;

	regmap_field_read(bcl->fields[F_V_MINOR], &raw_val);

	return raw_val;
}

static void bcl_hwmon_notify_event(struct bcl_device *bcl, enum bcl_limit_alarm alarm)
{
	if (bcl->desc->in_mon_enabled)
		hwmon_notify_event(bcl->hwmon_dev, hwmon_in,
				   in_lvl_to_attr_map[alarm], 0);

	if (bcl->desc->curr_mon_enabled)
		hwmon_notify_event(bcl->hwmon_dev, hwmon_curr,
				   curr_lvl_to_attr_map[alarm], 0);
}

static void bcl_alarm_enable_poll(struct work_struct *work)
{
	struct bcl_alarm_data *alarm = container_of(work, struct bcl_alarm_data,
							 alarm_poll_work.work);
	struct bcl_device *bcl = (struct bcl_device *)alarm->device;
	long status;
	int ret;

	scoped_guard(mutex, &bcl->lock)
		ret = bcl_read_alarm_status(bcl, alarm->type, &status);

	/* If read failed or shutting down, reschedule */
	if (ret || READ_ONCE(alarm->shutting_down)) {
		if (!READ_ONCE(alarm->shutting_down))
			schedule_delayed_work(&alarm->alarm_poll_work,
					      msecs_to_jiffies(BCL_ALARM_POLLING_MS));
		return;
	}

	/* Check if alarm cleared and IRQ needs re-enabling */
	scoped_guard(mutex, &alarm->a_lock) {
		if (alarm->shutting_down)
			return;

		if (!status && !alarm->irq_enabled) {
			alarm->irq_enabled = true;
			enable_irq(alarm->irq);

			if (!alarm->irq_wake_enabled && !enable_irq_wake(alarm->irq))
				alarm->irq_wake_enabled = true;

			bcl_hwmon_notify_event(bcl, alarm->type);
			return;
		}
	}

	/* Alarm still active, reschedule polling */
	if (!READ_ONCE(alarm->shutting_down))
		schedule_delayed_work(&alarm->alarm_poll_work,
				      msecs_to_jiffies(BCL_ALARM_POLLING_MS));
}

static irqreturn_t bcl_handle_alarm(int irq, void *data)
{
	struct bcl_alarm_data *alarm = data;
	struct bcl_device *bcl = (struct bcl_device *)alarm->device;
	long status;

	if (READ_ONCE(alarm->shutting_down))
		return IRQ_HANDLED;

	guard(mutex)(&bcl->lock);

	if (bcl_read_alarm_status(bcl, alarm->type, &status) || !status)
		return IRQ_HANDLED;

	bcl_hwmon_notify_event(bcl, alarm->type);

	guard(mutex)(&alarm->a_lock);
	if (alarm->shutting_down)
		return IRQ_HANDLED;

	if (alarm->irq_enabled) {
		alarm->irq_enabled = false;
		disable_irq_nosync(alarm->irq);
	}

	if (alarm->irq_wake_enabled) {
		disable_irq_wake(alarm->irq);
		alarm->irq_wake_enabled = false;
	}

	schedule_delayed_work(&alarm->alarm_poll_work,
			      msecs_to_jiffies(BCL_ALARM_POLLING_MS));

	return IRQ_HANDLED;
}

static umode_t bcl_hwmon_is_visible(const void *data,
				    enum hwmon_sensor_types type,
				    u32 attr, int channel)
{
	const struct bcl_device *bcl = data;

	switch (type) {
	case hwmon_in:
		if (!bcl->desc->in_mon_enabled)
			return 0;

		switch (attr) {
		case hwmon_in_input:
			return bcl->desc->in_input_enabled ? 0444 : 0;
		case hwmon_in_min_alarm:
		case hwmon_in_lcrit_alarm:
		case hwmon_in_lemergency_alarm:
			return 0444;
		case hwmon_in_min:
		case hwmon_in_lcrit:
		case hwmon_in_lemergency:
			return 0644;
		default:
			return 0;
		}
	case hwmon_curr:
		if (!bcl->desc->curr_mon_enabled)
			return 0;

		switch (attr) {
		case hwmon_curr_input:
		case hwmon_curr_max_alarm:
		case hwmon_curr_crit_alarm:
		case hwmon_curr_emergency_alarm:
			return 0444;
		case hwmon_curr_max:
		case hwmon_curr_crit:
			return 0644;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static int bcl_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long val)
{
	struct bcl_device *bcl = dev_get_drvdata(dev);

	guard(mutex)(&bcl->lock);

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_min:
		case hwmon_in_lcrit:
		case hwmon_in_lemergency:
			return bcl_in_thresh_write(bcl, val, in_attr_to_lvl_map[attr]);
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_max:
		case hwmon_curr_crit:
			return bcl_curr_thresh_write(bcl, val, curr_attr_to_lvl_map[attr]);
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
}

static int bcl_in_read(struct bcl_device *bcl, u32 attr, long *value)
{
	guard(mutex)(&bcl->lock);

	switch (attr) {
	case hwmon_in_input:
		return bcl_in_input_read(bcl, value);
	case hwmon_in_min:
	case hwmon_in_lcrit:
	case hwmon_in_lemergency:
		return bcl_in_thresh_read(bcl, in_attr_to_lvl_map[attr], value);
	case hwmon_in_min_alarm:
	case hwmon_in_lcrit_alarm:
	case hwmon_in_lemergency_alarm:
		return bcl_read_alarm_status(bcl, in_attr_to_lvl_map[attr], value);
	default:
		return -EOPNOTSUPP;
	}
}

static int bcl_curr_read(struct bcl_device *bcl, u32 attr, long *value)
{
	guard(mutex)(&bcl->lock);

	switch (attr) {
	case hwmon_curr_input:
		return bcl_curr_input_read(bcl, value);
	case hwmon_curr_max:
	case hwmon_curr_crit:
		return bcl_curr_thresh_read(bcl, curr_attr_to_lvl_map[attr], value);
	case hwmon_curr_max_alarm:
	case hwmon_curr_crit_alarm:
	case hwmon_curr_emergency_alarm:
		return bcl_read_alarm_status(bcl, curr_attr_to_lvl_map[attr], value);
	default:
		return -EOPNOTSUPP;
	}
}

static int bcl_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long *value)
{
	struct bcl_device *bcl = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_in:
		return bcl_in_read(bcl, attr, value);
	case hwmon_curr:
		return bcl_curr_read(bcl, attr, value);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_ops bcl_hwmon_ops = {
	.is_visible	= bcl_hwmon_is_visible,
	.read		= bcl_hwmon_read,
	.write		= bcl_hwmon_write,
};

static const struct hwmon_channel_info * const bcl_hwmon_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_MIN | HWMON_I_LCRIT |
			   HWMON_I_LEMERGENCY |
			   HWMON_I_MIN_ALARM | HWMON_I_LCRIT_ALARM |
			   HWMON_I_LEMERGENCY_ALARM),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_MAX | HWMON_C_CRIT |
			   HWMON_C_MAX_ALARM | HWMON_C_CRIT_ALARM |
			   HWMON_C_EMERGENCY_ALARM),
	NULL
};

static const struct hwmon_chip_info bcl_chip_info = {
	.ops = &bcl_hwmon_ops,
	.info = bcl_hwmon_info,
};

static int bcl_detect_battery_config(struct bcl_device *bcl)
{
	u32 reg_val = 0;
	int ret;

	ret = regmap_field_read(bcl->batt_config_regfield, &reg_val);
	if (ret) {
		dev_err(bcl->dev, "Failed to read battery config register: %d\n", ret);
		return ret;
	}

	/*
	 * Map register value to battery configuration.
	 * As per hardware documentation:
	 * 0 -> 2S, 1 -> 3S, 2 -> 4S (unsupported)
	 */
	switch (reg_val) {
	case 0:
		bcl->batt_config = BCL_BATT_2S;
		return 0;
	case 1:
		bcl->batt_config = BCL_BATT_3S;
		return 0;
	default:
		dev_err(bcl->dev, "Unsupported battery configuration: 0x%x\n",
			reg_val);
		return -EINVAL;
	}
}

static int bcl_update_scaling_factors(struct bcl_device *bcl)
{
	const struct bcl_desc *desc = bcl->desc;
	int ret;

	/* Check if battery detection is supported */
	if (desc->battery_config_field.reg == 0) {
		/* No battery detection - use BCL_BATT_1S */
		bcl->batt_config = BCL_BATT_1S;
		dev_dbg(bcl->dev, "Using default 1S battery configuration\n");
		return 0;
	}

	bcl->batt_config_regfield = devm_regmap_field_alloc(bcl->dev,
							    bcl->regmap,
							    desc->battery_config_field);
	if (IS_ERR(bcl->batt_config_regfield)) {
		dev_err(bcl->dev, "Failed to allocate battery config regmap field\n");
		return PTR_ERR(bcl->batt_config_regfield);
	}

	ret = bcl_detect_battery_config(bcl);
	if (ret < 0)
		return ret;

	dev_dbg(bcl->dev,
		"%dS battery: BASE=%u mV,MAX=%u mV,STEP=%u mV,V_SCALE=%u nV, I_SCALE=%u nA\n",
		bcl->batt_config + 1,
		desc->channel_cfg[bcl->batt_config][CHANNEL_IN].base_mu,
		desc->channel_cfg[bcl->batt_config][CHANNEL_IN].max_mu,
		desc->channel_cfg[bcl->batt_config][CHANNEL_IN].step_mu,
		desc->channel_cfg[bcl->batt_config][CHANNEL_IN].default_scale_nu,
		desc->channel_cfg[bcl->batt_config][CHANNEL_CURR].default_scale_nu);

	return 0;
}

static void bcl_alarm_work_cleanup_action(void *data)
{
	struct bcl_alarm_data *alarm = data;

	/* Set shutting_down flag to prevent work from being rescheduled */
	scoped_guard(mutex, &alarm->a_lock)
		WRITE_ONCE(alarm->shutting_down, true);

	cancel_delayed_work_sync(&alarm->alarm_poll_work);
}

static void bcl_alarm_wake_cleanup_action(void *data)
{
	struct bcl_alarm_data *alarm = data;

	guard(mutex)(&alarm->a_lock);
	if (alarm->irq_wake_enabled) {
		disable_irq_wake(alarm->irq);
		alarm->irq_wake_enabled = false;
	}
}

static int bcl_alarm_irq_init(struct platform_device *pdev,
			      struct bcl_device *bcl)
{
	int ret, irq_num, i;
	struct bcl_alarm_data *alarm;

	for (i = 0; i < ARRAY_SIZE(bcl->bcl_alarms); i++) {
		alarm = &bcl->bcl_alarms[i];
		alarm->type = i;
		alarm->device = bcl;
		ret = devm_mutex_init(bcl->dev, &alarm->a_lock);
		if (ret)
			return ret;

		/* Initialize work before IRQ request */
		INIT_DELAYED_WORK(&alarm->alarm_poll_work, bcl_alarm_enable_poll);

		irq_num = platform_get_irq_byname(pdev, bcl_int_names[i]);
		if (irq_num < 0)
			return irq_num;

		alarm->irq = irq_num;
		alarm->irq_enabled = true;

		ret = devm_request_threaded_irq(&pdev->dev, irq_num, NULL,
						bcl_handle_alarm, IRQF_ONESHOT,
						bcl_int_names[i], alarm);
		if (ret)
			return ret;

		/*
		 * Register the work cleanup action right after the IRQ is
		 * requested, before any other fallible step in this loop, so
		 * a delayed work scheduled by the IRQ handler is guaranteed to
		 * be canceled on teardown even if the wake cleanup registration
		 * below fails. Keeping it after the IRQ request also preserves
		 * safe devm LIFO teardown order.
		 */
		ret = devm_add_action_or_reset(&pdev->dev, bcl_alarm_work_cleanup_action,
					       alarm);
		if (ret)
			return ret;

		if (!enable_irq_wake(irq_num))
			alarm->irq_wake_enabled = true;

		ret = devm_add_action_or_reset(&pdev->dev, bcl_alarm_wake_cleanup_action,
					       alarm);
		if (ret)
			return ret;
	}

	return 0;
}

static int bcl_regmap_field_init(struct device *dev, struct bcl_device *bcl,
				 const struct bcl_desc *data)
{
	struct reg_field fields[F_MAX_FIELDS];
	int i;

	/*
	 * Note: We use a mutable local copy of struct reg_field (not const)
	 * because we need to modify the .reg field to add the BCL base offset.
	 */
	for (i = 0; i < data->num_reg_fields; i++) {
		fields[i] = data->reg_fields[i];
		fields[i].reg += bcl->base;
	}

	return devm_regmap_field_bulk_alloc(dev, bcl->regmap, bcl->fields,
					    fields, data->num_reg_fields);
}

static int bcl_probe(struct platform_device *pdev)
{
	struct bcl_device *bcl;
	int ret;
	u32 reg;

	bcl = devm_kzalloc(&pdev->dev, sizeof(*bcl), GFP_KERNEL);
	if (!bcl)
		return -ENOMEM;

	bcl->dev = &pdev->dev;
	bcl->desc = device_get_match_data(&pdev->dev);
	if (!bcl->desc)
		return dev_err_probe(&pdev->dev, -EINVAL, "Failed to get device match data\n");

	ret = devm_mutex_init(bcl->dev, &bcl->lock);
	if (ret)
		return ret;

	bcl->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!bcl->regmap)
		return dev_err_probe(&pdev->dev, -EINVAL, "Couldn't get parent's regmap\n");

	ret = device_property_read_u32(&pdev->dev, "reg", &reg);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "Failed to read 'reg' property\n");

	bcl->base = reg;

	ret = bcl_regmap_field_init(bcl->dev, bcl, bcl->desc);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "Unable to allocate regmap fields\n");

	ret = regmap_field_read(bcl->fields[F_CTL_EN], &reg);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "Failed to read BCL enable status\n");

	if (!reg)
		return dev_err_probe(&pdev->dev, -ENODEV, "BCL is not enabled by bootloader\n");

	ret = bcl_update_scaling_factors(bcl);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "Failed to update scaling factors\n");

	dev_set_drvdata(&pdev->dev, bcl);

	bcl->hwmon_name = devm_hwmon_sanitize_name(&pdev->dev,
						   dev_name(bcl->dev));
	if (IS_ERR(bcl->hwmon_name))
		return PTR_ERR(bcl->hwmon_name);

	bcl->hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev,
							      bcl->hwmon_name,
							      bcl,
							      &bcl_chip_info,
							      NULL);
	if (IS_ERR(bcl->hwmon_dev))
		return dev_err_probe(&pdev->dev, PTR_ERR(bcl->hwmon_dev),
				     "Failed to register hwmon device\n");

	ret = bcl_alarm_irq_init(pdev, bcl);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "Failed to initialize alarm IRQs\n");

	dev_dbg(&pdev->dev, "BCL hwmon device with version: %u.%u registered\n",
		bcl_get_version_major(bcl), bcl_get_version_minor(bcl));

	return 0;
}

static const struct of_device_id bcl_match[] = {
	{
		.compatible = "qcom,pm7250b-bcl",
		.data = &pm7250b_data,
	}, {
		.compatible = "qcom,pm7550-bcl",
		.data = &pm8550_data,
	}, {
		.compatible = "qcom,pm7550ba-bcl",
		.data = &pm7550ba_data,
	}, {
		.compatible = "qcom,pm8350c-bcl",
		.data = &pm8350c_data,
	}, {
		.compatible = "qcom,pm8550-bcl",
		.data = &pm8550_data,
	}, {
		.compatible = "qcom,pmh0101-bcl",
		.data = &pmh0101_data,
	}, {
		.compatible = "qcom,pmih0108-bcl",
		.data = &pmih0108_data,
	}, {
		.compatible = "qcom,smb2360-bcl",
		.data = &smb2360_data,
	}, {
		.compatible = "qcom,smb2370-bcl",
		.data = &smb2370_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, bcl_match);

static struct platform_driver bcl_driver = {
	.probe	= bcl_probe,
	.driver	= {
		.name		= "qcom-bcl-hwmon",
		.of_match_table	= bcl_match,
	},
};

module_platform_driver(bcl_driver);

MODULE_DESCRIPTION("Qualcomm SPMI BCL HWMON driver");
MODULE_LICENSE("GPL");
