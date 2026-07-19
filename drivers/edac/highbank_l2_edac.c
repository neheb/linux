// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2011-2012 Calxeda, Inc.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/edac.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "edac_module.h"

#define SR_CLR_SB_ECC_INTR	0x0
#define SR_CLR_DB_ECC_INTR	0x4

struct hb_l2_drvdata {
	void __iomem *base;
	int sb_irq;
	int db_irq;
};

static irqreturn_t highbank_l2_err_handler(int irq, void *dev_id)
{
	struct edac_device_ctl_info *dci = dev_id;
	struct hb_l2_drvdata *drvdata = dci->pvt_info;

	if (irq == drvdata->sb_irq) {
		writel(1, drvdata->base + SR_CLR_SB_ECC_INTR);
		edac_device_handle_ce(dci, 0, 0, dci->ctl_name);
	}
	if (irq == drvdata->db_irq) {
		writel(1, drvdata->base + SR_CLR_DB_ECC_INTR);
		edac_device_handle_ue(dci, 0, 0, dci->ctl_name);
	}

	return IRQ_HANDLED;
}

static const struct of_device_id hb_l2_err_of_match[] = {
	{ .compatible = "calxeda,hb-sregs-l2-ecc", },
	{},
};
MODULE_DEVICE_TABLE(of, hb_l2_err_of_match);

static void highbank_l2_edac_free(void *data)
{
	struct edac_device_ctl_info *dci = data;

	edac_device_free_ctl_info(dci);
}

static int highbank_l2_err_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct edac_device_ctl_info *dci;
	struct hb_l2_drvdata *drvdata;
	void __iomem *base;
	int db_irq;
	int sb_irq;
	int res;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	db_irq = platform_get_irq(pdev, 0);
	if (db_irq < 0)
		return db_irq;

	sb_irq = platform_get_irq(pdev, 1);
	if (sb_irq < 0)
		return sb_irq;

	dci = edac_device_alloc_ctl_info(sizeof(*drvdata), "cpu",
					 1, "L", 1, 2, 0);
	if (!dci)
		return -ENOMEM;

	if (devm_add_action_or_reset(&pdev->dev, highbank_l2_edac_free, dci))
		return -ENOMEM;

	drvdata = dci->pvt_info;
	dci->dev = &pdev->dev;
	platform_set_drvdata(pdev, dci);

	id = of_match_device(hb_l2_err_of_match, &pdev->dev);
	dci->mod_name = pdev->dev.driver->name;
	dci->ctl_name = id ? id->compatible : "unknown";
	dci->dev_name = dev_name(&pdev->dev);

	if (edac_device_add_device(dci))
		return 0;

	drvdata->base = base;
	drvdata->db_irq = db_irq;
	drvdata->sb_irq = sb_irq;

	res = devm_request_irq(&pdev->dev, drvdata->db_irq,
			       highbank_l2_err_handler,
			       0, dev_name(&pdev->dev), dci);
	if (res < 0)
		goto err2;

	res = devm_request_irq(&pdev->dev, drvdata->sb_irq,
			       highbank_l2_err_handler,
			       0, dev_name(&pdev->dev), dci);
	if (res < 0)
		goto err2;

	return 0;
err2:
	edac_device_del_device(&pdev->dev);
	return res;
}

static void highbank_l2_err_remove(struct platform_device *pdev)
{
	edac_device_del_device(&pdev->dev);
}

static struct platform_driver highbank_l2_edac_driver = {
	.probe = highbank_l2_err_probe,
	.remove = highbank_l2_err_remove,
	.driver = {
		.name = "hb_l2_edac",
		.of_match_table = hb_l2_err_of_match,
	},
};

module_platform_driver(highbank_l2_edac_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Calxeda, Inc.");
MODULE_DESCRIPTION("EDAC Driver for Calxeda Highbank L2 Cache");
