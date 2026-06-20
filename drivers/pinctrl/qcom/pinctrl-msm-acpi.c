// SPDX-License-Identifier: GPL-2.0-only
/*
 * ACPI GPIO lookup handling for WoA (Windows on ARM) laptop ACPI tables.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/gpio/driver.h>
#include <linux/list.h>
#include <linux/math.h>
#include "pinctrl-msm.h"

#define MSM_GPIO_WOA_ACPI_GPIOS_PER_BANK	64
#define MSM_GPIO_WOA_ACPI_IRQ_OFFSET		32
#define MSM_GPIO_WOA_ACPI_INVALID_GPIO		~0U
#define MSM_GPIO_WOA_ACPI_MAX_PDC_RANGES	16

#define PDC_RANGE_PIN_BASE			0
#define PDC_RANGE_GIC_BASE			1
#define PDC_RANGE_COUNT				2
#define PDC_RANGE_ELEMENTS			3

/**
 * struct msm_gpio_woa_acpi_parse_data - Data for parsing WoA ACPI GPIO ctl resources
 * @chip:		gpiochip handle
 * @data:		Data for mapping virtual WoA ACPI PDC IRQ GPIOs
 * @soc_data:		Reference to soc_data of platform specific data
 * @pdc_range:		PDC GIC to PDC map ranges
 * @pdc_range_count:	PDC GIC to PDC map range-count
 */
struct msm_gpio_woa_acpi_parse_data {
	struct gpio_chip *chip;
	struct msm_gpio_woa_acpi_data *data;
	const struct msm_pinctrl_soc_data *soc_data;
	u32 pdc_range[MSM_GPIO_WOA_ACPI_MAX_PDC_RANGES][PDC_RANGE_ELEMENTS];
	unsigned int pdc_range_count;
};

/*
 * Mapping does not need translating the acpi_resource in to a regular resoure
 * and adding it to the resource list. Always return 1 to disable this.
 */
static int msm_gpio_woa_acpi_resource(struct acpi_resource *ares, void *_parse)
{
	struct msm_gpio_woa_acpi_parse_data *parse = _parse;
	const struct msm_pinctrl_soc_data *soc_data = parse->soc_data;
	struct msm_gpio_woa_acpi_data *data = parse->data;
	struct gpio_chip *chip = parse->chip;
	u32 gic_irq, pdc_pin;

	if (ares->type != ACPI_RESOURCE_TYPE_EXTENDED_IRQ ||
	    ares->data.extended_irq.interrupt_count != 1)
		return 1;

	if (data->nmap == MSM_GPIO_WOA_ACPI_MAX_VIRT_GPIOS) {
		dev_err(chip->parent, "ACPI resources contain more than %d IRQs\n",
			MSM_GPIO_WOA_ACPI_MAX_VIRT_GPIOS);
		return 1;
	}

	/*
	 * Windows ACPI tables divide GPIOs into banks of 64 pins with one IRQ
	 * per bank. The resources start with listing the real TLMM IRQ for
	 * as many banks as are necessary to cover the real GPIOs. The Windows
	 * virtual GPIO indexes skip these banks, mark them as unavailable.
	 */
	if (data->nmap < DIV_ROUND_UP(chip->ngpio, MSM_GPIO_WOA_ACPI_GPIOS_PER_BANK)) {
		data->map[data->nmap++] = MSM_GPIO_WOA_ACPI_INVALID_GPIO;
		return 1;
	}

	/*
	 * Use the "pdc-ranges" property on the PDC to translate the GIC IRQ
	 * from the acpi_resource to a PDC pin.
	 */
	gic_irq = ares->data.extended_irq.interrupts[0] - MSM_GPIO_WOA_ACPI_IRQ_OFFSET;
	pdc_pin = MSM_GPIO_WOA_ACPI_INVALID_GPIO;
	for (unsigned int i = 0; i < parse->pdc_range_count; i++) {
		u32 gic_base = parse->pdc_range[i][PDC_RANGE_GIC_BASE];
		u32 count = parse->pdc_range[i][PDC_RANGE_COUNT];
		if (gic_irq >= gic_base && gic_irq < (gic_base + count)) {
			pdc_pin = parse->pdc_range[i][PDC_RANGE_PIN_BASE] +
				  gic_irq - gic_base;
			break;
		}
	}
	if (pdc_pin == MSM_GPIO_WOA_ACPI_INVALID_GPIO)
		goto no_map;

	/* Use wakeirq-map to map PDC pin to TLMM pin */
	for (unsigned int i = 0; i < soc_data->nwakeirq_map; i++) {
		if (soc_data->wakeirq_map[i].wakeirq == pdc_pin) {
			data->map[data->nmap++] = soc_data->wakeirq_map[i].gpio;
			return 1;
		}
	}

no_map:
	dev_warn(chip->parent, "Cannot map GIC IRQ %u to TLMM pin\n", gic_irq);
	data->map[data->nmap++] = MSM_GPIO_WOA_ACPI_INVALID_GPIO;
	return 1;
}

int msm_gpio_woa_acpi_init(struct gpio_chip *chip, struct msm_gpio_woa_acpi_data *data,
			   const struct msm_pinctrl_soc_data *soc_data)
{
	struct msm_gpio_woa_acpi_parse_data parse;
	struct fwnode_handle *fwnode;
	struct device_node *pdc_np;
	LIST_HEAD(resources);
	unsigned int ngpio;
	int ret;

	/* WoA ACPI tables are only used in DT-ACPI hybrid mode */
	fwnode = chip->parent->fwnode;
	if (!is_of_node(fwnode) || !is_acpi_device_node(fwnode->secondary))
		return 0;

	parse.chip = chip;
	parse.data = data;
	parse.soc_data = soc_data;

	/* Get PDC ranges, the PDC is the TLMM's wakeup-parent. */
	pdc_np = of_parse_phandle(chip->parent->of_node, "wakeup-parent", 0);
	if (!pdc_np)
		return 0;

	ret = of_property_count_elems_of_size(pdc_np, "qcom,pdc-ranges", sizeof(u32));
	if (ret <= 0 || ret % PDC_RANGE_ELEMENTS ||
	    ret > (MSM_GPIO_WOA_ACPI_MAX_PDC_RANGES * PDC_RANGE_ELEMENTS))
		goto err_pdc_ranges;

	parse.pdc_range_count = ret / PDC_RANGE_ELEMENTS;
	ret = of_property_read_u32_array(pdc_np, "qcom,pdc-ranges",
					 &parse.pdc_range[0][0], ret);
	if (ret)
		goto err_pdc_ranges;

	ret = acpi_dev_get_resources(to_acpi_device_node(fwnode->secondary), &resources,
				     msm_gpio_woa_acpi_resource, &parse);
	if (ret < 0)
		return ret;

	acpi_dev_free_resource_list(&resources);

	ngpio = data->nmap * MSM_GPIO_WOA_ACPI_GPIOS_PER_BANK + 1;
	chip->ngpio = max(chip->ngpio, ngpio);

	for (unsigned int i = 0; i < data->nmap; i++) {
		if (data->map[i] != MSM_GPIO_WOA_ACPI_INVALID_GPIO) {
			/* TODO lower log level to dev_dbg() */
			dev_info(chip->parent, "mapped GPIO 0x%03x to TLMM pin %u\n",
				 i * MSM_GPIO_WOA_ACPI_GPIOS_PER_BANK, data->map[i]);
		}
	}

	return 0;

err_pdc_ranges:
	dev_err(chip->parent, "Error invalid pdc-ranges\n");
	return 0; /* Continue without mapping */
}

void msm_gpio_woa_acpi_valid_mask(struct gpio_chip *chip,
				  struct msm_gpio_woa_acpi_data *data,
				  unsigned long *valid_mask,
				  unsigned int soc_ngpio)
{
	/* First mark all virtual ACPI PDC GPIOs (if any) as invalid */
	bitmap_clear(valid_mask, soc_ngpio, chip->ngpio - soc_ngpio);

	/*
	 * WoA ACPI tables list 1 Interrupt resource per PDC pin and use
	 * a 1 interrupt per bank model. So each PDC pin gets its own bank,
	 * with only pin 0 of that bank used for that PDC pin.
	 */
	for (unsigned int i = 0; i < data->nmap; i++) {
		if (data->map[i] != MSM_GPIO_WOA_ACPI_INVALID_GPIO)
			set_bit(i * MSM_GPIO_WOA_ACPI_GPIOS_PER_BANK, valid_mask);
	}
}

unsigned int msm_gpio_woa_acpi_map(struct msm_gpio_woa_acpi_data *data, unsigned int offset)
{
	unsigned int bank = offset / MSM_GPIO_WOA_ACPI_GPIOS_PER_BANK;

	/* msm_gpio_woa_acpi_valid_mask() should ensure this never happens */
	if (WARN_ON(offset % MSM_GPIO_WOA_ACPI_GPIOS_PER_BANK || bank >= data->nmap ||
	    data->map[bank] == MSM_GPIO_WOA_ACPI_INVALID_GPIO))
		return 0;

	return data->map[bank];
}
