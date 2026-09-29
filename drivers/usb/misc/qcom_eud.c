// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/usb/role.h>
#include <linux/firmware/qcom/qcom_scm.h>

#define EUD_REG_INT1_EN_MASK	0x0024
#define EUD_REG_INT_STATUS_1	0x0044
#define EUD_REG_CTL_OUT_1	0x0074
#define EUD_REG_VBUS_INT_CLR	0x0080
#define EUD_REG_CSR_EUD_EN	0x1014
#define EUD_REG_SW_ATTACH_DET	0x1018
#define EUD_REG_PORT_SEL	0x1028
#define EUD_REG_EUD_EN2		0x0000

#define EUD_MAX_PORTS		2

#define EUD_ENABLE		BIT(0)
#define EUD_INT_PET_EUD		BIT(0)
#define EUD_INT_VBUS		BIT(2)
#define EUD_INT_SAFE_MODE	BIT(4)
#define EUD_INT_ALL		(EUD_INT_VBUS | EUD_INT_SAFE_MODE)
#define EUD_PORT_SEL_MASK	BIT(0)

static const char * const eud_port_names[] = {
	"primary",
	"secondary",
};

struct eud_path {
	struct eud_chip		*chip;
	struct phy		*phy;
	struct usb_role_switch	*controller_sw;
	struct usb_role_switch	*eud_sw;
	struct device_node	*np;
	enum usb_role		curr_role;
	char			name[16];
	u8			num;
};

struct eud_chip {
	struct device			*dev;
	void __iomem			*base;
	struct eud_path			*paths[EUD_MAX_PORTS];
	phys_addr_t			mode_mgr;
	unsigned int			int_status;
	int				irq;
	bool				enabled;
	bool				usb_attached;
	bool				phy_enabled;
	u8				port_idx;
};

static int eud_phy_enable(struct eud_chip *chip)
{
	struct phy *phy;
	int ret;

	if (chip->phy_enabled)
		return 0;

	phy = chip->paths[chip->port_idx]->phy;

	ret = phy_init(phy);
	if (ret) {
		dev_err(chip->dev, "Failed to initialize USB2 PHY for port %u: %d\n",
			chip->port_idx, ret);
		return ret;
	}

	ret = phy_power_on(phy);
	if (ret) {
		dev_err(chip->dev, "Failed to power on USB2 PHY for port %u: %d\n",
			chip->port_idx, ret);
		phy_exit(phy);
		return ret;
	}

	chip->phy_enabled = true;

	return 0;
}

static void eud_phy_disable(struct eud_chip *chip)
{
	struct phy *phy;

	if (!chip->phy_enabled)
		return;

	phy = chip->paths[chip->port_idx]->phy;

	phy_power_off(phy);
	phy_exit(phy);
	chip->phy_enabled = false;
}

static int enable_eud(struct eud_chip *priv)
{
	int ret;

	ret = eud_phy_enable(priv);
	if (ret)
		return ret;

	ret = qcom_scm_io_writel(priv->mode_mgr + EUD_REG_EUD_EN2, 1);
	if (ret) {
		eud_phy_disable(priv);
		return ret;
	}

	writel(EUD_ENABLE, priv->base + EUD_REG_CSR_EUD_EN);
	writel(EUD_INT_VBUS | EUD_INT_SAFE_MODE,
			priv->base + EUD_REG_INT1_EN_MASK);

	return 0;
}

static int disable_eud(struct eud_chip *priv)
{
	int ret;

	ret = qcom_scm_io_writel(priv->mode_mgr + EUD_REG_EUD_EN2, 0);
	if (ret)
		return ret;

	writel(0, priv->base + EUD_REG_CSR_EUD_EN);
	eud_phy_disable(priv);

	return 0;
}

static ssize_t enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct eud_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", chip->enabled);
}

static ssize_t enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct eud_chip *chip = dev_get_drvdata(dev);
	bool enable;
	int ret;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	if (enable) {
		ret = enable_eud(chip);
		if (!ret)
			chip->enabled = enable;
		else
			disable_eud(chip);

	} else {
		ret = disable_eud(chip);
	}

	return ret < 0 ? ret : count;
}

static DEVICE_ATTR_RW(enable);

static ssize_t port_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct eud_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", eud_port_names[chip->port_idx]);
}

static ssize_t port_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct eud_chip *chip = dev_get_drvdata(dev);
	int port;

	port = sysfs_match_string(eud_port_names, buf);
	if (port < 0)
		return port;

	/* Check if the corresponding path is available */
	if (!chip->paths[port])
		return -EOPNOTSUPP;

	/* Port selection must be done before enabling EUD */
	if (chip->enabled)
		return -EBUSY;

	writel(port, chip->base + EUD_REG_PORT_SEL);
	chip->port_idx = port;

	return count;
}

static DEVICE_ATTR_RW(port);

static struct attribute *eud_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_port.attr,
	NULL,
};
ATTRIBUTE_GROUPS(eud);

static void usb_attach_detach(struct eud_chip *chip)
{
	u32 reg;

	/* read ctl_out_1[4] to find USB attach or detach event */
	reg = readl(chip->base + EUD_REG_CTL_OUT_1);
	chip->usb_attached = reg & EUD_INT_SAFE_MODE;
}

static void pet_eud(struct eud_chip *chip)
{
	u32 reg;
	int ret;

	/* When the EUD_INT_PET_EUD in SW_ATTACH_DET is set, the cable has been
	 * disconnected and we need to detach the pet to check if EUD is in safe
	 * mode before attaching again.
	 */
	reg = readl(chip->base + EUD_REG_SW_ATTACH_DET);
	if (reg & EUD_INT_PET_EUD) {
		/* Detach & Attach pet for EUD */
		writel(0, chip->base + EUD_REG_SW_ATTACH_DET);
		/* Delay to make sure detach pet is done before attach pet */
		ret = readl_poll_timeout(chip->base + EUD_REG_SW_ATTACH_DET,
					reg, (reg == 0), 1, 100);
		if (ret) {
			dev_err(chip->dev, "Detach pet failed\n");
			return;
		}
	}
	/* Attach pet for EUD */
	writel(EUD_INT_PET_EUD, chip->base + EUD_REG_SW_ATTACH_DET);
}

static irqreturn_t handle_eud_irq(int irq, void *data)
{
	struct eud_chip *chip = data;
	u32 reg;

	reg = readl(chip->base + EUD_REG_INT_STATUS_1);
	switch (reg & EUD_INT_ALL) {
	case EUD_INT_VBUS:
		usb_attach_detach(chip);
		return IRQ_WAKE_THREAD;
	case EUD_INT_SAFE_MODE:
		pet_eud(chip);
		return IRQ_HANDLED;
	default:
		return IRQ_NONE;
	}
}

static irqreturn_t handle_eud_irq_thread(int irq, void *data)
{
	struct eud_chip *chip = data;
	struct eud_path *path;
	int ret;

	path = chip->paths[chip->port_idx];

	if (chip->usb_attached)
		ret = usb_role_switch_set_role(path->controller_sw, USB_ROLE_DEVICE);
	else
		ret = usb_role_switch_set_role(path->controller_sw, USB_ROLE_HOST);
	if (ret)
		dev_err(chip->dev, "failed to set role switch\n");

	/* set and clear vbus_int_clr[0] to clear interrupt */
	writel(BIT(0), chip->base + EUD_REG_VBUS_INT_CLR);
	writel(0, chip->base + EUD_REG_VBUS_INT_CLR);

	return IRQ_HANDLED;
}

static int eud_role_switch_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct eud_path *path = usb_role_switch_get_drvdata(sw);
	int ret;

	/* curr_role tracks the role from EUD's point of view */
	path->curr_role = role;

	/* Forward the role request to the USB controller */
	ret = usb_role_switch_set_role(path->controller_sw, role);
	if (ret) {
		dev_err(path->chip->dev, "failed to set role %s for port %u: %d\n",
			usb_role_string(role), path->num, ret);
		return ret;
	}

	return 0;
}

static void eud_path_role_switch_release(void *data)
{
	struct eud_path *path = data;

	usb_role_switch_unregister(path->eud_sw);
	usb_role_switch_put(path->controller_sw);
	of_node_put(path->np);
}

static int eud_init_path(struct eud_chip *chip, struct device_node *np)
{
	struct usb_role_switch_desc role_sw_desc = {NULL};
	struct usb_role_switch *sw;
	struct device_node *controller_node;
	struct eud_path *path;
	const char *dr_mode = "";
	u32 path_num;
	int ret;

	ret = of_property_read_u32(np, "reg", &path_num);
	if (ret) {
		/* Legacy DT uses 'ports' node without 'reg' property; treat as path 0 */
		if (of_node_name_eq(np, "ports"))
			path_num = 0;
		else
			return dev_err_probe(chip->dev, ret, "unexpected child node '%s'\n",
					     np->name);
	}

	if (path_num >= EUD_MAX_PORTS)
		return dev_err_probe(chip->dev, -EINVAL, "invalid path number: %u (max %d)\n",
				     path_num, EUD_MAX_PORTS - 1);

	path = devm_kzalloc(chip->dev, sizeof(*path), GFP_KERNEL);
	if (!path)
		return -ENOMEM;

	path->chip = chip;
	path->num = path_num;

	controller_node = of_graph_get_remote_node(np, 0, -1);
	if (!controller_node)
		return dev_err_probe(chip->dev, -ENODEV,
				     "failed to get controller node for path %u\n", path_num);

	path->phy = devm_of_phy_get_by_index(chip->dev, controller_node, 0);
	if (IS_ERR(path->phy)) {
		of_node_put(controller_node);
		return dev_err_probe(chip->dev, PTR_ERR(path->phy),
				     "failed to get PHY for path %d\n", path_num);
	}

	chip->paths[path_num] = path;

	of_property_read_string(controller_node, "dr_mode", &dr_mode);

	/* No role switch needed for fixed role controllers */
	if (!strcmp(dr_mode, "host")) {
		path->curr_role = USB_ROLE_HOST;
		of_node_put(controller_node);
		return 0;
	}

	if (!strcmp(dr_mode, "peripheral")) {
		path->curr_role = USB_ROLE_DEVICE;
		of_node_put(controller_node);
		return 0;
	}

	path->curr_role = USB_ROLE_NONE;

	sw = usb_role_switch_find_by_fwnode(of_fwnode_handle(controller_node));
	if (!sw) {
		of_node_put(controller_node);
		return dev_err_probe(chip->dev, -EPROBE_DEFER,
				     "Failed to get controller role switch for path %d\n",
				     path_num);
	}
	path->controller_sw = sw;

	of_node_put(controller_node);

	path->np = of_node_get(np);
	role_sw_desc.fwnode = of_fwnode_handle(path->np);
	role_sw_desc.set = eud_role_switch_set;
	role_sw_desc.driver_data = path;
	snprintf(path->name, sizeof(path->name), "eud-path%u", path_num);
	role_sw_desc.name = path->name;

	path->eud_sw = usb_role_switch_register(chip->dev, &role_sw_desc);
	if (IS_ERR(path->eud_sw)) {
		usb_role_switch_put(path->controller_sw);
		of_node_put(path->np);
		return dev_err_probe(chip->dev, PTR_ERR(path->eud_sw),
				     "Failed to register EUD role switch for path %d\n", path_num);
	}

	ret = devm_add_action_or_reset(chip->dev, eud_path_role_switch_release, path);
	if (ret)
		return ret;

	return 0;
}

static int eud_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct eud_chip *chip;
	struct resource *res;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	chip->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(chip->base))
		return PTR_ERR(chip->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -ENODEV;
	chip->mode_mgr = res->start;

	for_each_child_of_node_scoped(np, child) {
		ret = eud_init_path(chip, child);
		if (ret)
			return ret;
	}

	/* Primary path is mandatory. Secondary is optional */
	if (!chip->paths[0])
		return dev_err_probe(chip->dev, -ENODEV, "primary path not found\n");

	chip->port_idx = readl(chip->base + EUD_REG_PORT_SEL) & EUD_PORT_SEL_MASK;
	if (!chip->paths[chip->port_idx]) {
		dev_info(chip->dev, "secondary path not available, falling back to primary\n");
		chip->port_idx = 0;
		writel(chip->port_idx, chip->base + EUD_REG_PORT_SEL);
	}

	chip->irq = platform_get_irq(pdev, 0);
	if (chip->irq < 0)
		return chip->irq;

	ret = devm_request_threaded_irq(&pdev->dev, chip->irq, handle_eud_irq,
			handle_eud_irq_thread, IRQF_ONESHOT, NULL, chip);
	if (ret)
		return ret;

	enable_irq_wake(chip->irq);

	platform_set_drvdata(pdev, chip);

	return 0;
}

static void eud_remove(struct platform_device *pdev)
{
	struct eud_chip *chip = platform_get_drvdata(pdev);

	if (chip->enabled)
		disable_eud(chip);

	device_init_wakeup(&pdev->dev, false);
	disable_irq_wake(chip->irq);
}

static const struct of_device_id eud_dt_match[] = {
	{ .compatible = "qcom,eud" },
	{ }
};
MODULE_DEVICE_TABLE(of, eud_dt_match);

static struct platform_driver eud_driver = {
	.probe	= eud_probe,
	.remove = eud_remove,
	.driver	= {
		.name = "qcom_eud",
		.dev_groups = eud_groups,
		.of_match_table = eud_dt_match,
	},
};
module_platform_driver(eud_driver);

MODULE_DESCRIPTION("QTI EUD driver");
MODULE_LICENSE("GPL v2");
