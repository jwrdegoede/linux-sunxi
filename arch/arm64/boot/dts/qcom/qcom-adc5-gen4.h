/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __DTS_ARM64_QCOM_ADC5_GEN4_H__
#define __DTS_ARM64_QCOM_ADC5_GEN4_H__

/* ADC channels for PMIC5 Gen4 */

#define ADC5_GEN4_VIRT_CHAN(bus_id, sid, chan)	((bus_id) << 13 | (sid) << 8 | (chan))

#define ADC5_GEN4_OFFSET_REF(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x00)
#define ADC5_GEN4_1P25VREF(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x01)
#define ADC5_GEN4_VREF_VADC(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x02)
#define ADC5_GEN4_DIE_TEMP(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x03)

#define ADC5_GEN4_AMUX1_THM(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x04)
#define ADC5_GEN4_AMUX2_THM(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x05)
#define ADC5_GEN4_AMUX3_THM(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x06)
#define ADC5_GEN4_AMUX4_THM(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x07)
#define ADC5_GEN4_AMUX5_THM(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x08)
#define ADC5_GEN4_AMUX6_THM(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x09)
#define ADC5_GEN4_AMUX1_GPIO(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x0a)
#define ADC5_GEN4_AMUX2_GPIO(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x0b)
#define ADC5_GEN4_AMUX3_GPIO(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x0c)
#define ADC5_GEN4_AMUX4_GPIO(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x0d)
#define ADC5_GEN4_AMUX5_GPIO(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x1e)

#define ADC5_GEN4_CHG_TEMP(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x10)
#define ADC5_GEN4_USB_SNS_DIV20(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x11)
#define ADC5_GEN4_VIN_DIV20_MUX(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x12)
#define ADC5_GEN4_USBC_MUX(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x13)
#define ADC5_GEN4_VREF_BAT_THERM(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x15)
#define ADC5_GEN4_IIN(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x17)

#define ADC5_GEN4_VREF_BAT2_THERM(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x1a)
#define ADC5_GEN4_ATEST1(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x1b)
#define ADC5_GEN4_ATEST2(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x1c)
#define ADC5_GEN4_VBAT_2S_MID_CHGR(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x1d)

/* 10k pull-up1 */
#define ADC5_GEN4_AMUX1_THM_10K_PU(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x24)
#define ADC5_GEN4_AMUX2_THM_10K_PU(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x25)
#define ADC5_GEN4_AMUX3_THM_10K_PU(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x26)
#define ADC5_GEN4_AMUX4_THM_10K_PU(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x27)
#define ADC5_GEN4_AMUX5_THM_10K_PU(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x28)
#define ADC5_GEN4_AMUX6_THM_10K_PU(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x29)
#define ADC5_GEN4_AMUX1_GPIO_10K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x2a)
#define ADC5_GEN4_AMUX2_GPIO_10K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x2b)
#define ADC5_GEN4_AMUX3_GPIO_10K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x2c)
#define ADC5_GEN4_AMUX4_GPIO_10K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x2d)
#define ADC5_GEN4_USBC_MUX_10K_PU(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x33)
#define ADC5_GEN4_AMUX5_GPIO_10K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x3e)

/* 100k pull-up2 */
#define ADC5_GEN4_AMUX1_THM_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x44)
#define ADC5_GEN4_AMUX2_THM_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x45)
#define ADC5_GEN4_AMUX3_THM_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x46)
#define ADC5_GEN4_AMUX4_THM_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x47)
#define ADC5_GEN4_AMUX5_THM_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x48)
#define ADC5_GEN4_AMUX6_THM_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x49)
#define ADC5_GEN4_AMUX1_GPIO_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x4a)
#define ADC5_GEN4_AMUX2_GPIO_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x4b)
#define ADC5_GEN4_AMUX3_GPIO_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x4c)
#define ADC5_GEN4_AMUX4_GPIO_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x4d)
#define ADC5_GEN4_USBC_MUX_100K_PU(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x53)
#define ADC5_GEN4_AMUX5_GPIO_100K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x5e)

/* 400k pull-up3 */
#define ADC5_GEN4_AMUX1_THM_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x64)
#define ADC5_GEN4_AMUX2_THM_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x65)
#define ADC5_GEN4_AMUX3_THM_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x66)
#define ADC5_GEN4_AMUX4_THM_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x67)
#define ADC5_GEN4_AMUX5_THM_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x68)
#define ADC5_GEN4_AMUX6_THM_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x69)
#define ADC5_GEN4_AMUX1_GPIO_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x6a)
#define ADC5_GEN4_AMUX2_GPIO_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x6b)
#define ADC5_GEN4_AMUX3_GPIO_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x6c)
#define ADC5_GEN4_AMUX4_GPIO_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x6d)
#define ADC5_GEN4_USBC_MUX_400K_PU(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x73)
#define ADC5_GEN4_AMUX5_GPIO_400K_PU(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x7e)

/* 1/3 Divider */
#define ADC5_GEN4_AMUX1_GPIO_DIV3(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x8a)
#define ADC5_GEN4_VPH_PWR(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x8e)
#define ADC5_GEN4_VBAT_SNS_QBG(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x8f)
#define ADC5_GEN4_VBAT_SNS_CHG(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x94)
#define ADC5_GEN4_VBAT_2S_MID_QBG(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x96)
#define ADC5_GEN4_VPH2_PWR(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x99)
#define ADC5_GEN4_VBAT_2S_MID_CHGR_DIV3(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x9d)
#define ADC5_GEN4_VBAT_2S_MID2(bus_id, sid)	ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0x9f)

#define ADC5_GEN4_ICHG_FB(bus_id, sid)		ADC5_GEN4_VIRT_CHAN(bus_id, sid, 0xa1)

#endif /* __DTS_ARM64_QCOM_ADC5_GEN4_H__ */
