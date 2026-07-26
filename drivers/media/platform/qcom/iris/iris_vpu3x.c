// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2025 Linaro Ltd
 */

#include <linux/iopoll.h>
#include <linux/reset.h>

#include "iris_instance.h"
#include "iris_vpu_common.h"
#include "iris_vpu_register_defines.h"

#define AON_WRAPPER_MVP_NOC_CORE_SW_RESET	(AON_BASE_OFFS + 0x18)
#define SW_RESET				BIT(0)
#define AON_WRAPPER_MVP_NOC_CORE_CLK_CONTROL	(AON_BASE_OFFS + 0x20)
#define NOC_HALT				BIT(0)
#define AON_WRAPPER_SPARE			(AON_BASE_OFFS + 0x28)

static bool iris_vpu3x_hw_power_collapsed(struct iris_core *core, u32 pwr_status_bit)
{
	u32 value, pwr_status;

	value = readl(core->reg_base + WRAPPER_CORE_POWER_STATUS);
	pwr_status = value & pwr_status_bit;

	return !pwr_status;
}

static void iris_vpu3_power_off_hardware(struct iris_core *core)
{
	u32 reg_val = 0, value, i;
	int ret;

	if (iris_vpu3x_hw_power_collapsed(core, VCODEC0_POWER_STATUS))
		goto disable_power;

	dev_err(core->dev, "video hw is power on\n");

	value = readl(core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);
	if (value)
		writel(CORE_CLK_RUN, core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);

	for (i = 0; i < core->iris_platform_data->num_vpp_pipe; i++) {
		ret = readl_poll_timeout(core->reg_base + VCODEC_SS_IDLE_STATUSN + 4 * i,
					 reg_val, reg_val & 0x400000, 2000, 20000);
		if (ret)
			goto disable_power;
	}

	writel(VIDEO_NOC_RESET_REQ, core->reg_base + AON_WRAPPER_MVP_NOC_RESET_REQ);

	ret = readl_poll_timeout(core->reg_base + AON_WRAPPER_MVP_NOC_RESET_ACK,
				 reg_val, reg_val & 0x3, 200, 2000);
	if (ret)
		goto disable_power;

	writel(0x0, core->reg_base + AON_WRAPPER_MVP_NOC_RESET_REQ);

	ret = readl_poll_timeout(core->reg_base + AON_WRAPPER_MVP_NOC_RESET_ACK,
				 reg_val, !(reg_val & 0x3), 200, 2000);
	if (ret)
		goto disable_power;

	writel(CORE_BRIDGE_SW_RESET | CORE_BRIDGE_HW_RESET_DISABLE,
	       core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);
	writel(CORE_BRIDGE_HW_RESET_DISABLE, core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);
	writel(0x0, core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);

disable_power:
	iris_vpu_power_off_hw(core);
}

static void iris_vpu33_power_off_hardware(struct iris_core *core)
{
	bool handshake_done = false, handshake_busy = false;
	u32 reg_val = 0, value, i;
	u32 count = 0;
	int ret;

	if (iris_vpu3x_hw_power_collapsed(core, VCODEC0_POWER_STATUS))
		goto disable_power;

	dev_err(core->dev, "video hw is power on\n");

	value = readl(core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);
	if (value)
		writel(CORE_CLK_RUN, core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);

	for (i = 0; i < core->iris_platform_data->num_vpp_pipe; i++) {
		ret = readl_poll_timeout(core->reg_base + VCODEC_SS_IDLE_STATUSN + 4 * i,
					 reg_val, reg_val & 0x400000, 2000, 20000);
		if (ret)
			goto disable_power;
	}

	/* Retry up to 1000 times as recommended by hardware documentation */
	do {
		/* set MNoC to low power */
		writel(REQ_POWER_DOWN_PREP, core->reg_base + AON_WRAPPER_MVP_NOC_LPI_CONTROL);

		udelay(15);

		value = readl(core->reg_base + AON_WRAPPER_MVP_NOC_LPI_STATUS);

		handshake_done = value & NOC_LPI_STATUS_DONE;
		handshake_busy = value & (NOC_LPI_STATUS_DENY | NOC_LPI_STATUS_ACTIVE);

		if (handshake_done || !handshake_busy)
			break;

		writel(0, core->reg_base + AON_WRAPPER_MVP_NOC_LPI_CONTROL);

		udelay(15);

	} while (++count < 1000);

	if (!handshake_done && handshake_busy)
		dev_err(core->dev, "LPI handshake timeout\n");

	ret = readl_poll_timeout(core->reg_base + AON_WRAPPER_MVP_NOC_LPI_STATUS,
				 reg_val, reg_val & BIT(0), 200, 2000);
	if (ret)
		goto disable_power;

	writel(0, core->reg_base + AON_WRAPPER_MVP_NOC_LPI_CONTROL);

	writel(CORE_BRIDGE_SW_RESET | CORE_BRIDGE_HW_RESET_DISABLE,
	       core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);
	writel(CORE_BRIDGE_HW_RESET_DISABLE, core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);
	writel(0x0, core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);

disable_power:
	iris_vpu_power_off_hw(core);
}

static int iris_vpu33_power_off_controller(struct iris_core *core)
{
	u32 xo_rst_tbl_size = core->iris_platform_data->controller_rst_tbl_size;
	u32 clk_rst_tbl_size = core->iris_platform_data->clk_rst_tbl_size;
	u32 val = 0;
	int ret;

	writel(MSK_SIGNAL_FROM_TENSILICA | MSK_CORE_POWER_ON, core->reg_base + CPU_CS_X2RPMH);

	writel(REQ_POWER_DOWN_PREP, core->reg_base + WRAPPER_IRIS_CPU_NOC_LPI_CONTROL);

	ret = readl_poll_timeout(core->reg_base + WRAPPER_IRIS_CPU_NOC_LPI_STATUS,
				 val, val & BIT(0), 200, 2000);
	if (ret)
		goto disable_power;

	writel(0x0, core->reg_base + WRAPPER_DEBUG_BRIDGE_LPI_CONTROL);

	ret = readl_poll_timeout(core->reg_base + WRAPPER_DEBUG_BRIDGE_LPI_STATUS,
				 val, val == 0, 200, 2000);
	if (ret)
		goto disable_power;

	writel(CTL_AXI_CLK_HALT | CTL_CLK_HALT,
	       core->reg_base + WRAPPER_TZ_CTL_AXI_CLOCK_CONFIG);
	writel(RESET_HIGH, core->reg_base + WRAPPER_TZ_QNS4PDXFIFO_RESET);
	writel(0x0, core->reg_base + WRAPPER_TZ_QNS4PDXFIFO_RESET);
	writel(0x0, core->reg_base + WRAPPER_TZ_CTL_AXI_CLOCK_CONFIG);

	reset_control_bulk_reset(clk_rst_tbl_size, core->resets);

	/* Disable MVP NoC clock */
	val = readl(core->reg_base + AON_WRAPPER_MVP_NOC_CORE_CLK_CONTROL);
	val |= NOC_HALT;
	writel(val, core->reg_base + AON_WRAPPER_MVP_NOC_CORE_CLK_CONTROL);

	/* enable MVP NoC reset */
	val = readl(core->reg_base + AON_WRAPPER_MVP_NOC_CORE_SW_RESET);
	val |= SW_RESET;
	writel(val, core->reg_base + AON_WRAPPER_MVP_NOC_CORE_SW_RESET);

	/* poll AON spare register bit0 to become zero with 50ms timeout */
	ret = readl_poll_timeout(core->reg_base + AON_WRAPPER_SPARE,
				 val, (val & BIT(0)) == 0, 1000, 50000);
	if (ret)
		goto disable_power;

	/* enable bit(1) to avoid cvp noc xo reset */
	val = readl(core->reg_base + AON_WRAPPER_SPARE);
	val |= BIT(1);
	writel(val, core->reg_base + AON_WRAPPER_SPARE);

	reset_control_bulk_assert(xo_rst_tbl_size, core->controller_resets);

	/* De-assert MVP NoC reset */
	val = readl(core->reg_base + AON_WRAPPER_MVP_NOC_CORE_SW_RESET);
	val &= ~SW_RESET;
	writel(val, core->reg_base + AON_WRAPPER_MVP_NOC_CORE_SW_RESET);

	usleep_range(80, 100);

	reset_control_bulk_deassert(xo_rst_tbl_size, core->controller_resets);

	/* reset AON spare register */
	writel(0, core->reg_base + AON_WRAPPER_SPARE);

	/* Enable MVP NoC clock */
	val = readl(core->reg_base + AON_WRAPPER_MVP_NOC_CORE_CLK_CONTROL);
	val &= ~NOC_HALT;
	writel(val, core->reg_base + AON_WRAPPER_MVP_NOC_CORE_CLK_CONTROL);

disable_power:
	iris_disable_power_domain_and_clocks(core, core->ctrl);

	return 0;
}

static int iris_vpu35_power_on_hw(struct iris_core *core)
{
	return iris_enable_power_domain_and_clocks(core, core->vcodec);
}

static void iris_vpu35_power_off_hw(struct iris_core *core)
{
	iris_vpu33_power_off_hardware(core);
}

static void iris_vpu36_power_off_vcodec(struct iris_core *core, u32 core_id)
{
	u32 bridge_hw_reset[] = {CORE_BRIDGE_HW_RESET_DISABLE, VCODEC1_BRIDGE_HW_RESET_DISABLE};
	u32 lpi_status_active[] = {NOC_LPI_STATUS_ACTIVE, NOC_LPI_VCODEC1_STATUS_ACTIVE};
	u32 power_down_prep[] = {REQ_POWER_DOWN_PREP, REQ_VCODEC1_POWER_DOWN_PREP};
	u32 lpi_status_done[] = {NOC_LPI_STATUS_DONE, NOC_LPI_VCODEC1_STATUS_DONE};
	u32 lpi_status_deny[] = {NOC_LPI_STATUS_DENY, NOC_LPI_VCODEC1_STATUS_DENY};
	u32 bridge_sw_reset[] = {CORE_BRIDGE_SW_RESET, VCODEC1_BRIDGE_SW_RESET};
	u32 idle_status[] = {VCODEC_SS_IDLE_STATUSN, VCODEC1_SS_IDLE_STATUSN};
	u32 power_status[] = {VCODEC0_POWER_STATUS, VCODEC1_POWER_STATUS};
	bool handshake_done, handshake_busy;
	u32 value, i, count = 0;
	int ret;

	if (iris_vpu3x_hw_power_collapsed(core, power_status[core_id]))
		goto disable_power;

	value = readl(core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);
	if (value)
		writel(CORE_CLK_RUN, core->reg_base + WRAPPER_CORE_CLOCK_CONFIG);

	for (i = 0; i < core->iris_platform_data->num_vpp_pipe; i++) {
		ret = readl_poll_timeout(core->reg_base + idle_status[core_id] + 4 * i,
					 value, value & DMA_NOC_IDLE, 2000, 20000);
		if (ret)
			goto disable_power;
	}

	do {
		writel(power_down_prep[core_id], core->reg_base + AON_WRAPPER_MVP_NOC_LPI_CONTROL);
		usleep_range(15, 20);
		value = readl(core->reg_base + AON_WRAPPER_MVP_NOC_LPI_STATUS);

		handshake_done = value & lpi_status_done[core_id];
		handshake_busy = value & (lpi_status_deny[core_id] | lpi_status_active[core_id]);

		if (handshake_done || !handshake_busy)
			break;

		writel(0, core->reg_base + AON_WRAPPER_MVP_NOC_LPI_CONTROL);
		usleep_range(15, 20);
	} while (++count < 1000);

	if (!handshake_done && handshake_busy)
		goto disable_power;

	writel(0, core->reg_base + AON_WRAPPER_MVP_NOC_LPI_CONTROL);

	writel(bridge_sw_reset[core_id] | bridge_hw_reset[core_id],
	       core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);
	writel(bridge_hw_reset[core_id], core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);
	writel(0x0, core->reg_base + CPU_CS_AHB_BRIDGE_SYNC_RESET);

disable_power:
	iris_genpd_set_hwmode(&core->vcodec[core_id], false);
	iris_disable_power_domain_and_clocks(core, &core->vcodec[core_id]);
}

static void iris_vpu36_power_off_hw(struct iris_core *core)
{
	u32 num_cores = core->iris_platform_data->num_cores;
	int i;

	for (i = 0; i < num_cores; i++)
		iris_vpu36_power_off_vcodec(core, i);
}

static int iris_vpu36_power_on_hw(struct iris_core *core)
{
	u32 num_cores = core->iris_platform_data->num_cores;
	int i, ret;

	for (i = 0; i < num_cores; i++) {
		ret = iris_enable_power_domain_and_clocks(core, &core->vcodec[i]);
		if (ret)
			goto error;
	}

	return 0;

error:
	while (--i >= 0)
		iris_vpu36_power_off_vcodec(core, i);

	return ret;
}

static int iris_vpu36_set_hwmode(struct iris_core *core)
{
	u32 num_cores = core->iris_platform_data->num_cores;
	int i, ret;

	for (i = 0; i < num_cores; i++) {
		ret = iris_genpd_set_hwmode(&core->vcodec[i], true);
		if (ret)
			goto error;
	}

	return 0;

error:
	while (--i >= 0)
		iris_genpd_set_hwmode(&core->vcodec[i], false);

	return ret;
}

static void iris_vpu36_program_bootup_registers(struct iris_core *core)
{
	writel(0x0, core->reg_base + CPU_CS_SCIACMDARG3);
}

static int iris_vpu36_check_core_load(struct iris_inst *inst, bool mbpf)
{
	const struct iris_platform_data *platform_data = inst->core->iris_platform_data;
	u32 max_load = mbpf ? platform_data->max_core_mbpf : platform_data->max_core_mbps;
	u32 max_session_cnt = platform_data->max_session_count;
	u32 core0_session_cnt = 0, core1_session_cnt = 0;
	u32 core0_load = 0, core1_load = 0;
	bool select_core0, select_core1;
	struct iris_inst *instance;
	u32 load, new_load;

	list_for_each_entry(instance, &inst->core->instances, list) {
		load = mbpf ? iris_get_mbpf(instance) : iris_get_mbps(instance);

		if (instance->core_id == IRIS_VCODEC0) {
			core0_load += load;
			core0_session_cnt++;
		} else if (instance->core_id == IRIS_VCODEC1) {
			core1_load += load;
			core1_session_cnt++;
		}
	}

	if (inst->core_id == IRIS_VCODEC0)
		return core0_load <= max_load ? 0 : -ENOMEM;
	else if (inst->core_id == IRIS_VCODEC1)
		return core1_load <= max_load ? 0 : -ENOMEM;

	new_load = mbpf ? iris_get_mbpf(inst) : iris_get_mbps(inst);

	select_core0 = core0_load + new_load <= max_load && core0_session_cnt < max_session_cnt;
	select_core1 = core1_load + new_load <= max_load && core1_session_cnt < max_session_cnt;

	if (select_core0 && select_core1)
		inst->core_id = (core0_load <= core1_load) ? IRIS_VCODEC0 : IRIS_VCODEC1;
	else if (select_core0)
		inst->core_id = IRIS_VCODEC0;
	else if (select_core1)
		inst->core_id = IRIS_VCODEC1;
	else
		return -ENOMEM;

	return 0;
}

static u64 iris_vpu36_get_required_freq(struct iris_inst *inst)
{
	u64 vcodec0_freq = 0, vcodec1_freq = 0;
	struct iris_core *core = inst->core;
	struct iris_inst *instance;

	list_for_each_entry(instance, &core->instances, list) {
		if (!instance->max_input_data_size)
			continue;

		if (instance->core_id == IRIS_VCODEC0)
			vcodec0_freq += instance->power.min_freq;
		else if (instance->core_id == IRIS_VCODEC1)
			vcodec1_freq += instance->power.min_freq;
	}

	return max(vcodec0_freq, vcodec1_freq);
}

const struct vpu_ops iris_vpu3_ops = {
	.power_off_hw = iris_vpu3_power_off_hardware,
	.power_on_hw = iris_vpu_power_on_hw,
	.power_off_controller = iris_vpu_power_off_controller,
	.power_on_controller = iris_vpu_power_on_controller,
	.calc_freq = iris_vpu3x_vpu4x_calculate_frequency,
	.set_hwmode = iris_vpu_set_hwmode,
};

const struct vpu_ops iris_vpu33_ops = {
	.power_off_hw = iris_vpu33_power_off_hardware,
	.power_on_hw = iris_vpu_power_on_hw,
	.power_off_controller = iris_vpu33_power_off_controller,
	.power_on_controller = iris_vpu_power_on_controller,
	.calc_freq = iris_vpu3x_vpu4x_calculate_frequency,
	.set_hwmode = iris_vpu_set_hwmode,
};

const struct vpu_ops iris_vpu35_ops = {
	.power_off_hw = iris_vpu35_power_off_hw,
	.power_on_hw = iris_vpu35_power_on_hw,
	.power_off_controller = iris_vpu35_vpu4x_power_off_controller,
	.power_on_controller = iris_vpu35_vpu4x_power_on_controller,
	.program_bootup_registers = iris_vpu35_vpu4x_program_bootup_registers,
	.calc_freq = iris_vpu3x_vpu4x_calculate_frequency,
	.set_hwmode = iris_vpu_set_hwmode,
};

const struct vpu_ops iris_vpu36_ops = {
	.power_off_hw = iris_vpu36_power_off_hw,
	.power_on_hw = iris_vpu36_power_on_hw,
	.power_off_controller = iris_vpu35_vpu4x_power_off_controller,
	.power_on_controller = iris_vpu35_vpu4x_power_on_controller,
	.program_bootup_registers = iris_vpu36_program_bootup_registers,
	.calc_freq = iris_vpu3x_vpu4x_calculate_frequency,
	.set_hwmode = iris_vpu36_set_hwmode,
	.check_core_load = iris_vpu36_check_core_load,
	.get_required_freq = iris_vpu36_get_required_freq,
};
