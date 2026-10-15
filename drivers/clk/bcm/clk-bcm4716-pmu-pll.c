// SPDX-License-Identifier: GPL-2.0-only
/*
 * Broadcom BCM4716 PMU main PLL
 *
 * The ChipCommon PMU of BCM4716/BCM4717/BCM4718 runs the main PLL off the
 * 20 MHz crystal. The PLL has one feedback divider and four post dividers.
 * Channels 1 to 3 clock the CPU, the memory controller and the backplane.
 * The boot loader programs the PLL, so this driver only reads it.
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/brcm,bcm4716-pmu-pll.h>

#define PMU_PLLCTL_ADDR		0x660
#define PMU_PLLCTL_DATA		0x664

/* Control words of the main PLL, accessed through PMU_PLLCTL_ADDR/DATA */
#define MAINPLL_PLL0		12
#define MAINPLL_P1P2		(MAINPLL_PLL0 + 0)
#define  MAINPLL_P1		GENMASK(27, 24)
#define  MAINPLL_P2		GENMASK(23, 20)
#define MAINPLL_M14		(MAINPLL_PLL0 + 1)
#define  MAINPLL_MDIV_WIDTH	8
#define  MAINPLL_MDIV		GENMASK(7, 0)
#define MAINPLL_NM5		(MAINPLL_PLL0 + 2)
#define  MAINPLL_NDIV		GENMASK(31, 20)

#define BCM4716_PMU_PLL_NUM	(BCM4716_PMU_PLL_BACKPLANE + 1)

struct bcm4716_pmu_pll {
	struct clk_hw hw;
	struct regmap *regmap;
	unsigned int channel;
};

static int bcm4716_pmu_pll_read(struct regmap *regmap, unsigned int reg,
				unsigned int *val)
{
	int ret;

	ret = regmap_write(regmap, PMU_PLLCTL_ADDR, reg);
	if (ret)
		return ret;

	/* Read back to flush the address write before reading the data */
	ret = regmap_read(regmap, PMU_PLLCTL_ADDR, val);
	if (ret)
		return ret;

	return regmap_read(regmap, PMU_PLLCTL_DATA, val);
}

static unsigned long bcm4716_pmu_pll_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct bcm4716_pmu_pll *pll = container_of(hw, struct bcm4716_pmu_pll, hw);
	unsigned int p1p2, m14, nm5;
	u32 p1, p2, mdiv, ndiv;

	if (bcm4716_pmu_pll_read(pll->regmap, MAINPLL_P1P2, &p1p2) ||
	    bcm4716_pmu_pll_read(pll->regmap, MAINPLL_M14, &m14) ||
	    bcm4716_pmu_pll_read(pll->regmap, MAINPLL_NM5, &nm5))
		return 0;

	p1 = FIELD_GET(MAINPLL_P1, p1p2);
	p2 = FIELD_GET(MAINPLL_P2, p1p2);
	mdiv = (m14 >> ((pll->channel - 1) * MAINPLL_MDIV_WIDTH)) &
	       MAINPLL_MDIV;
	ndiv = FIELD_GET(MAINPLL_NDIV, nm5);

	if (!p2 || !mdiv)
		return 0;

	return div_u64((u64)parent_rate * p1 * ndiv, p2 * mdiv);
}

static const struct clk_ops bcm4716_pmu_pll_ops = {
	.recalc_rate = bcm4716_pmu_pll_recalc_rate,
};

static const char * const bcm4716_pmu_pll_names[BCM4716_PMU_PLL_NUM] = {
	[BCM4716_PMU_PLL_CPU]		= "cpu",
	[BCM4716_PMU_PLL_MEM]		= "mem",
	[BCM4716_PMU_PLL_BACKPLANE]	= "backplane",
};

static int bcm4716_pmu_pll_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct clk_parent_data parent = { .index = 0 };
	struct regmap *regmap;
	int i, ret;

	regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to get the ChipCommon regmap\n");

	data = devm_kzalloc_flex(dev, *data, hws, BCM4716_PMU_PLL_NUM);
	if (!data)
		return -ENOMEM;
	data->num = BCM4716_PMU_PLL_NUM;

	for (i = 0; i < BCM4716_PMU_PLL_NUM; i++) {
		struct clk_init_data init = {
			.name = devm_kasprintf(dev, GFP_KERNEL, "%pOFn-%s",
					       dev->of_node,
					       bcm4716_pmu_pll_names[i]),
			.ops = &bcm4716_pmu_pll_ops,
			.parent_data = &parent,
			.num_parents = 1,
		};
		struct bcm4716_pmu_pll *pll;

		if (!init.name)
			return -ENOMEM;

		pll = devm_kzalloc_obj(dev, *pll);
		if (!pll)
			return -ENOMEM;

		pll->regmap = regmap;
		pll->channel = i + 1;
		pll->hw.init = &init;

		ret = devm_clk_hw_register(dev, &pll->hw);
		if (ret)
			return ret;

		data->hws[i] = &pll->hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id bcm4716_pmu_pll_of_match[] = {
	{ .compatible = "brcm,bcm4716-pmu-pll" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm4716_pmu_pll_of_match);

static struct platform_driver bcm4716_pmu_pll_driver = {
	.probe = bcm4716_pmu_pll_probe,
	.driver = {
		.name = "bcm4716-pmu-pll",
		.of_match_table = bcm4716_pmu_pll_of_match,
	},
};
module_platform_driver(bcm4716_pmu_pll_driver);

MODULE_DESCRIPTION("Broadcom BCM4716 PMU main PLL driver");
MODULE_LICENSE("GPL");
