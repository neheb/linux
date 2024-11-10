/*
 * Driver for the MPC5200 Fast Ethernet Controller - MDIO bus driver
 *
 * Copyright (C) 2007  Domen Puncer, Telargo, Inc.
 * Copyright (C) 2008  Wolfram Sang, Pengutronix
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/mpc52xx.h>
#include "fec_mpc52xx.h"

struct mpc52xx_fec_mdio_priv {
	struct mpc52xx_fec __iomem *regs;
};

static int mpc52xx_fec_mdio_transfer(struct mii_bus *bus, int phy_id,
		int reg, u32 value)
{
	struct mpc52xx_fec_mdio_priv *priv = bus->priv;
	struct mpc52xx_fec __iomem *fec = priv->regs;
	int tries = 3;

	value |= (phy_id << FEC_MII_DATA_PA_SHIFT) & FEC_MII_DATA_PA_MSK;
	value |= (reg << FEC_MII_DATA_RA_SHIFT) & FEC_MII_DATA_RA_MSK;

	out_be32(&fec->ievent, FEC_IEVENT_MII);
	out_be32(&fec->mii_data, value);

	/* wait for it to finish, this takes about 23 us on lite5200b */
	while (!(in_be32(&fec->ievent) & FEC_IEVENT_MII) && --tries)
		msleep(1);

	if (!tries)
		return -ETIMEDOUT;

	return value & FEC_MII_DATA_OP_RD ?
		in_be32(&fec->mii_data) & FEC_MII_DATA_DATAMSK : 0;
}

static int mpc52xx_fec_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	return mpc52xx_fec_mdio_transfer(bus, phy_id, reg, FEC_MII_READ_FRAME);
}

static int mpc52xx_fec_mdio_write(struct mii_bus *bus, int phy_id, int reg,
		u16 data)
{
	return mpc52xx_fec_mdio_transfer(bus, phy_id, reg,
		data | FEC_MII_WRITE_FRAME);
}

static int mpc52xx_fec_mdio_probe(struct platform_device *of)
{
	struct mpc52xx_fec_mdio_priv *priv;
	struct device *dev = &of->dev;
	struct resource *res;
	struct mii_bus *bus;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	bus = devm_mdiobus_alloc(dev);
	if (!bus)
		return -ENOMEM;

	bus->name = "mpc52xx MII bus";
	bus->read = mpc52xx_fec_mdio_read;
	bus->write = mpc52xx_fec_mdio_write;

	/* setup registers */
	priv->regs = devm_platform_get_and_ioremap_resource(of, 0, &res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	snprintf(bus->id, MII_BUS_ID_SIZE, "%pa", &res->start);
	bus->priv = priv;

	bus->parent = dev;

	/* set MII speed */
	out_be32(&priv->regs->mii_speed, ((mpc5xxx_get_bus_frequency(dev) >> 20) / 5) << 1);

	return devm_mdiobus_register(dev, bus);
}

static const struct of_device_id mpc52xx_fec_mdio_match[] = {
	{ .compatible = "fsl,mpc5200b-mdio", },
	{ .compatible = "fsl,mpc5200-mdio", },
	{ .compatible = "mpc5200b-fec-phy", },
	{}
};
MODULE_DEVICE_TABLE(of, mpc52xx_fec_mdio_match);

struct platform_driver mpc52xx_fec_mdio_driver = {
	.driver = {
		.name = "mpc5200b-fec-phy",
		.owner = THIS_MODULE,
		.of_match_table = mpc52xx_fec_mdio_match,
	},
	.probe = mpc52xx_fec_mdio_probe,
};

/* let fec driver call it, since this has to be registered before it */
EXPORT_SYMBOL_GPL(mpc52xx_fec_mdio_driver);

MODULE_LICENSE("Dual BSD/GPL");
