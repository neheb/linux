// SPDX-License-Identifier: GPL-2.0-only
/*
 * Broadcom BCM4716 USB 2.0 PHY
 *
 * The USB 2.0 host core of BCM4716/BCM4717/BCM4718 SoCs has its EHCI
 * registers at the start of the core and its OHCI registers in a second
 * window. The host control register and the shim MDIO interface to the
 * PHY follow the EHCI registers. The core must come out of reset once for
 * both controllers, so that is done here as well.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

/* Offsets from 0x200 into the core */
#define USB_HOSTCTL		0x000
#define  USB_HOSTCTL_INIT	0x7ff
#define USB_SHIM_MDIO_CTL	0x324
#define USB_SHIM_MDIO_DATA	0x328
#define  USB_SHIM_MDIO_DONE	BIT(31)

struct bcm4716_usb_phy {
	void __iomem *base;
	struct reset_control *reset;
	struct clk *cpu_clk;
};

/*
 * At high CPU clocks the host stops acknowledging, unless the clock and
 * data recovery of the PHY is set to match.
 */
static void bcm4716_usb_phy_set_cdr(struct bcm4716_usb_phy *priv)
{
	unsigned long rate = clk_get_rate(priv->cpu_clk);
	u32 cdr;

	if (rate >= 480000000)
		cdr = 0x1846b;		/* fast */
	else if (rate == 453000000)
		cdr = 0x1046b;		/* slow */
	else
		return;

	writel(0x1, priv->base + USB_SHIM_MDIO_CTL);
	usleep_range(500, 1000);
	writel(cdr, priv->base + USB_SHIM_MDIO_CTL);
	usleep_range(500, 1000);
	writel(0x4ab, priv->base + USB_SHIM_MDIO_CTL);
	usleep_range(500, 1000);
	readl(priv->base + USB_SHIM_MDIO_DATA);
	writel(USB_SHIM_MDIO_DONE, priv->base + USB_SHIM_MDIO_DATA);
}

static int bcm4716_usb_phy_init(struct phy *phy)
{
	struct bcm4716_usb_phy *priv = phy_get_drvdata(phy);
	int ret;

	ret = reset_control_reset(priv->reset);
	if (ret)
		return ret;
	usleep_range(10000, 20000);

	/* Take the host controller and the PHY out of reset */
	writel(USB_HOSTCTL_INIT, priv->base + USB_HOSTCTL);
	udelay(1);

	bcm4716_usb_phy_set_cdr(priv);

	return 0;
}

static int bcm4716_usb_phy_exit(struct phy *phy)
{
	struct bcm4716_usb_phy *priv = phy_get_drvdata(phy);

	return reset_control_assert(priv->reset);
}

static const struct phy_ops bcm4716_usb_phy_ops = {
	.init = bcm4716_usb_phy_init,
	.exit = bcm4716_usb_phy_exit,
	.owner = THIS_MODULE,
};

static int bcm4716_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct bcm4716_usb_phy *priv;
	struct phy *phy;

	priv = devm_kzalloc_obj(dev, *priv);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->reset))
		return dev_err_probe(dev, PTR_ERR(priv->reset),
				     "failed to get the core reset\n");

	priv->cpu_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->cpu_clk))
		return dev_err_probe(dev, PTR_ERR(priv->cpu_clk),
				     "failed to get the CPU clock\n");

	phy = devm_phy_create(dev, NULL, &bcm4716_usb_phy_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy), "failed to create PHY\n");

	phy_set_drvdata(phy, priv);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id bcm4716_usb_phy_of_match[] = {
	{ .compatible = "brcm,bcm4716-usb-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm4716_usb_phy_of_match);

static struct platform_driver bcm4716_usb_phy_driver = {
	.probe = bcm4716_usb_phy_probe,
	.driver = {
		.name = "bcm4716-usb-phy",
		.of_match_table = bcm4716_usb_phy_of_match,
	},
};
module_platform_driver(bcm4716_usb_phy_driver);

MODULE_DESCRIPTION("Broadcom BCM4716 USB 2.0 PHY driver");
MODULE_LICENSE("GPL");
