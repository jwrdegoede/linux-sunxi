// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Dollar Cove TI PMIC GPADC Driver
 *
 * Copyright (C) 2014 Intel Corporation (Ramakrishna Pallala <ramakrishna.pallala@intel.com>)
 * Copyright (C) 2024 Hans de Goede <hansg@kernel.org>
 */

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/wait.h>

#include <linux/iio/driver.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>

#define DC_TI_ADC_CNTL_REG			0x50
#define DC_TI_ADC_START				BIT(0)
#define DC_TI_ADC_CH_SEL			GENMASK(2, 1)
#define DC_TI_ADC_EN				BIT(5)
#define DC_TI_ADC_EN_EXT_BPTH_BIAS		BIT(6)

#define DC_TI_ADC_CH0_DATAH_REG			0x54
#define DC_TI_ADC_CH0_DATAL_REG			0x55
#define DC_TI_ADC_CH1_DATAH_REG			0x56
#define DC_TI_ADC_CH1_DATAL_REG			0x57
#define DC_TI_ADC_CH2_DATAH_REG			0x58
#define DC_TI_ADC_CH2_DATAL_REG			0x59
#define DC_TI_ADC_CH3_DATAH_REG			0x5A
#define DC_TI_ADC_CH3_DATAL_REG			0x5B

#define DEV_NAME				"chtdc_ti_adc"

enum dc_ti_adc_id {
	DC_TI_ADC_VBAT,
	DC_TI_ADC_PMICTEMP,
	DC_TI_ADC_BATTEMP,
	DC_TI_ADC_SYSTEMP0,
};

struct dc_ti_adc_info {
	struct mutex lock;
	wait_queue_head_t wait;
	struct device *dev;
	struct regmap *regmap;
	bool conversion_done;
};

static const struct iio_chan_spec dc_ti_adc_channels[] = {
	{
		.indexed = 1,
		.type = IIO_VOLTAGE,
		.channel = DC_TI_ADC_VBAT,
		.address = DC_TI_ADC_CH0_DATAH_REG,
		.datasheet_name = "CH0",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}, {
		.indexed = 1,
		.type = IIO_TEMP,
		.channel = DC_TI_ADC_PMICTEMP,
		.address = DC_TI_ADC_CH1_DATAH_REG,
		.datasheet_name = "CH1",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}, {
		.indexed = 1,
		.type = IIO_TEMP,
		.channel = DC_TI_ADC_BATTEMP,
		.address = DC_TI_ADC_CH2_DATAH_REG,
		.datasheet_name = "CH2",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}, {
		.indexed = 1,
		.type = IIO_TEMP,
		.channel = DC_TI_ADC_SYSTEMP0,
		.address = DC_TI_ADC_CH3_DATAH_REG,
		.datasheet_name = "CH3",
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static struct iio_map dc_ti_adc_default_maps[] = {
	IIO_MAP("CH0", "chtdc_ti_battery", "VBAT"),
	IIO_MAP("CH1", "chtdc_ti_battery", "PMICTEMP"),
	IIO_MAP("CH2", "chtdc_ti_battery", "BATTEMP"),
	IIO_MAP("CH3", "chtdc_ti_battery", "SYSTEMP0"),
	{}
};

static irqreturn_t dc_ti_adc_isr(int irq, void *data)
{
	struct dc_ti_adc_info *info = data;

	info->conversion_done = true;
	wake_up(&info->wait);
	return IRQ_HANDLED;
}

static int dc_ti_adc_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct dc_ti_adc_info *info = iio_priv(indio_dev);
	int ret, ch = chan->channel;
	unsigned int lsb, msb;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	guard(mutex)(&info->lock);

	info->conversion_done = false;

	/*
	 * If channel BPTHERM has been selected, first enable the BPTHERM BIAS
	 * which provides the VREFT Voltage reference to convert BPTHERM Input
	 * voltage to temperature.
	 * As per PMIC Vendor specifications, BPTHERM BIAS should be enabled
	 * 35 ms before ADC_EN command.
	 */
	if (ch == DC_TI_ADC_BATTEMP) {
		ret = regmap_update_bits(info->regmap, DC_TI_ADC_CNTL_REG,
					 DC_TI_ADC_EN_EXT_BPTH_BIAS,
					 DC_TI_ADC_EN_EXT_BPTH_BIAS);
		if (ret < 0)
			return ret;
		msleep(35);
	}

	/*
	 * As per TI (PMIC Vendor), the ADC enable and ADC start commands should
	 * not be sent together. Hence send the commands separately
	 */
	ret = regmap_update_bits(info->regmap, DC_TI_ADC_CNTL_REG,
				 DC_TI_ADC_EN, DC_TI_ADC_EN);
	if (ret < 0)
		goto disable_adc;

	ret = regmap_update_bits(info->regmap, DC_TI_ADC_CNTL_REG,
				 DC_TI_ADC_CH_SEL, FIELD_PREP(DC_TI_ADC_CH_SEL, ch));
	if (ret < 0)
		goto disable_adc;

	/*
	 * As per PMIC Vendor, a minimum of 50 micro seconds delay is required
	 * between ADC Enable and ADC START commands. This is also recommended
	 * by Intel Hardware team after the timing analysis of GPADC signals.
	 * Since the I2C Write transaction to set the channel number also
	 * imparts 25 micro seconds of delay, so we need to wait for another
	 * 25 micro seconds before issuing ADC START command.
	 */
	usleep_range(25, 40);

	ret = regmap_update_bits(info->regmap, DC_TI_ADC_CNTL_REG,
				 DC_TI_ADC_START, DC_TI_ADC_START);
	if (ret < 0)
		goto disable_adc;

	/* TI (PMIC Vendor) recommends 5 sec timeout for conversion */
	ret = wait_event_timeout(info->wait, info->conversion_done, 5 * HZ);
	if (ret == 0) {
		dev_err(info->dev, "Error sample timeout\n");
		ret = -ETIMEDOUT;
		goto disable_adc;
	}

	ret = regmap_read(info->regmap, chan->address, &msb);
	if (ret)
		goto disable_adc;

	ret = regmap_read(info->regmap, chan->address + 1, &lsb);
	if (ret)
		goto disable_adc;

	*val = ((msb << 8) | lsb) & 0x3ff;
	ret = IIO_VAL_INT;

disable_adc:
	regmap_update_bits(info->regmap, DC_TI_ADC_CNTL_REG,
			   DC_TI_ADC_START | DC_TI_ADC_EN, 0);

	if (ch == DC_TI_ADC_BATTEMP)
		regmap_update_bits(info->regmap, DC_TI_ADC_CNTL_REG,
				   DC_TI_ADC_EN_EXT_BPTH_BIAS, 0);

	return ret;
}

static const struct iio_info dc_ti_adc_iio_info = {
	.read_raw = dc_ti_adc_read_raw,
};

static int dc_ti_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct intel_soc_pmic *pmic = dev_get_drvdata(dev->parent);
	struct dc_ti_adc_info *info;
	struct iio_dev *indio_dev;
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);

	ret = devm_mutex_init(dev, &info->lock);
	if (ret)
		return ret;

	init_waitqueue_head(&info->wait);

	info->dev = dev;
	info->regmap = pmic->regmap;

	indio_dev->name = pdev->name;
	indio_dev->channels = dc_ti_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(dc_ti_adc_channels);
	indio_dev->info = &dc_ti_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_map_array_register(dev, indio_dev, dc_ti_adc_default_maps);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, dc_ti_adc_isr,
					IRQF_ONESHOT, DEV_NAME, info);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static struct platform_driver dc_ti_adc_driver = {
	.probe = dc_ti_adc_probe,
	.driver = {
		.name = DEV_NAME,
	},
};

module_platform_driver(dc_ti_adc_driver);

MODULE_ALIAS("platform:" DEV_NAME);
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Intel Dollar Cove (TI) GPADC Driver");
MODULE_LICENSE("GPL");
