// SPDX-License-Identifier: GPL-2.0-only
/*
 * Broadcom BCM4716 core wrapper reset controller
 *
 * Every core on the AXI backplane of BCM4716/BCM4717/BCM4718 SoCs sits
 * behind an agent wrapper that gates its clock and holds it in reset. This
 * driver implements the bcma core enable and disable sequences on one
 * wrapper, so that a core can be driven without the bcma bus driver.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>

#define WRAPPER_IOCTL		0x408
#define  WRAPPER_IOCTL_CLK	BIT(0)
#define  WRAPPER_IOCTL_FGC	BIT(1)
#define WRAPPER_RESET_CTL	0x800
#define  WRAPPER_RESET_CTL_RESET BIT(0)
#define WRAPPER_RESET_ST	0x804

struct bcm4716_wrapper_reset {
	struct reset_controller_dev rcdev;
	void __iomem *base;
};

static struct bcm4716_wrapper_reset *
to_bcm4716_wrapper_reset(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct bcm4716_wrapper_reset, rcdev);
}

static void bcm4716_wrapper_write(struct bcm4716_wrapper_reset *priv,
				  unsigned int reg, u32 val)
{
	writel(val, priv->base + reg);
	/* Read back to make sure the write has reached the wrapper */
	readl(priv->base + reg);
}

static int bcm4716_wrapper_assert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	struct bcm4716_wrapper_reset *priv = to_bcm4716_wrapper_reset(rcdev);
	u32 val;
	int ret;

	if (readl(priv->base + WRAPPER_RESET_CTL) & WRAPPER_RESET_CTL_RESET)
		return 0;

	/* Wait for outstanding transactions of the core to finish */
	ret = readl_poll_timeout_atomic(priv->base + WRAPPER_RESET_ST, val,
					!val, 10, 300);
	if (ret)
		dev_warn(rcdev->dev, "core still busy, resetting anyway\n");

	bcm4716_wrapper_write(priv, WRAPPER_RESET_CTL,
			      WRAPPER_RESET_CTL_RESET);
	udelay(1);

	bcm4716_wrapper_write(priv, WRAPPER_IOCTL, 0);
	usleep_range(10, 20);

	return 0;
}

static int bcm4716_wrapper_deassert(struct reset_controller_dev *rcdev,
				    unsigned long id)
{
	struct bcm4716_wrapper_reset *priv = to_bcm4716_wrapper_reset(rcdev);

	/* Leave reset with the clock forced on, then let it gate normally */
	bcm4716_wrapper_write(priv, WRAPPER_IOCTL,
			      WRAPPER_IOCTL_CLK | WRAPPER_IOCTL_FGC);

	bcm4716_wrapper_write(priv, WRAPPER_RESET_CTL, 0);
	udelay(1);

	bcm4716_wrapper_write(priv, WRAPPER_IOCTL, WRAPPER_IOCTL_CLK);
	udelay(1);

	return 0;
}

static int bcm4716_wrapper_reset(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	bcm4716_wrapper_assert(rcdev, id);

	return bcm4716_wrapper_deassert(rcdev, id);
}

static int bcm4716_wrapper_status(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	struct bcm4716_wrapper_reset *priv = to_bcm4716_wrapper_reset(rcdev);

	return !!(readl(priv->base + WRAPPER_RESET_CTL) &
		  WRAPPER_RESET_CTL_RESET);
}

static int bcm4716_wrapper_xlate(struct reset_controller_dev *rcdev,
				 const struct fwnode_reference_args *reset_spec)
{
	/* One wrapper, one reset line */
	return 0;
}

static const struct reset_control_ops bcm4716_wrapper_reset_ops = {
	.assert = bcm4716_wrapper_assert,
	.deassert = bcm4716_wrapper_deassert,
	.reset = bcm4716_wrapper_reset,
	.status = bcm4716_wrapper_status,
};

static int bcm4716_wrapper_reset_probe(struct platform_device *pdev)
{
	struct bcm4716_wrapper_reset *priv;

	priv = devm_kzalloc_obj(&pdev->dev, *priv);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->rcdev.ops = &bcm4716_wrapper_reset_ops;
	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.dev = &pdev->dev;
	priv->rcdev.fwnode_xlate = bcm4716_wrapper_xlate;
	priv->rcdev.fwnode_reset_n_cells = 0;
	priv->rcdev.nr_resets = 1;

	return devm_reset_controller_register(&pdev->dev, &priv->rcdev);
}

static const struct of_device_id bcm4716_wrapper_reset_of_match[] = {
	{ .compatible = "brcm,bcm4716-wrapper-reset" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm4716_wrapper_reset_of_match);

static struct platform_driver bcm4716_wrapper_reset_driver = {
	.probe = bcm4716_wrapper_reset_probe,
	.driver = {
		.name = "bcm4716-wrapper-reset",
		.of_match_table = bcm4716_wrapper_reset_of_match,
	},
};
module_platform_driver(bcm4716_wrapper_reset_driver);

MODULE_DESCRIPTION("Broadcom BCM4716 core wrapper reset driver");
MODULE_LICENSE("GPL");
