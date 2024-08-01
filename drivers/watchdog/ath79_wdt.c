// SPDX-License-Identifier: GPL-2.0-only
/*
 * Atheros AR71XX/AR724X/AR913X built-in hardware watchdog timer.
 *
 * Copyright (C) 2008-2011 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2008 Imre Kaloz <kaloz@openwrt.org>
 *
 * This driver was based on: drivers/watchdog/ixp4xx_wdt.c
 *	Author: Deepak Saxena <dsaxena@plexity.net>
 *	Copyright 2004 (c) MontaVista, Software, Inc.
 *
 * which again was based on sa1100 driver,
 *	Copyright (C) 2000 Oleg Drokin <green@crimea.edu>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#define DRIVER_NAME	"ath79-wdt"

#define WDT_TIMEOUT	15	/* seconds */

#define WDOG_REG_CTRL		0x00
#define WDOG_REG_TIMER		0x04

#define WDOG_CTRL_LAST_RESET	BIT(31)
#define WDOG_CTRL_ACTION_MASK	3
#define WDOG_CTRL_ACTION_NONE	0	/* no action */
#define WDOG_CTRL_ACTION_GPI	1	/* general purpose interrupt */
#define WDOG_CTRL_ACTION_NMI	2	/* NMI */
#define WDOG_CTRL_ACTION_FCR	3	/* full chip reset */

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
			   "(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int timeout = WDT_TIMEOUT;
module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds "
			  "(default=" __MODULE_STRING(WDT_TIMEOUT) "s)");

static unsigned long wdt_flags;

#define WDT_FLAGS_BUSY		0
#define WDT_FLAGS_EXPECT_CLOSE	1

static struct clk *wdt_clk;
static unsigned long wdt_freq;
static int boot_status;
static int max_timeout;
static void __iomem *wdt_base;

static inline void ath79_wdt_wr(unsigned reg, u32 val)
{
	iowrite32(val, wdt_base + reg);
}

static inline u32 ath79_wdt_rr(unsigned reg)
{
	return ioread32(wdt_base + reg);
}

static int ath79_wdt_ping(struct watchdog_device *wdd)
{
	ath79_wdt_wr(WDOG_REG_TIMER, wdt_freq * timeout);
	/* flush write */
	ath79_wdt_rr(WDOG_REG_TIMER);

	return 0;
}

static int ath79_wdt_set_timeout(struct watchdog_device *wdd, unsigned int val)
{
	timeout = val;
	ath79_wdt_ping();

	return 0;
}

static void ath79_wdt_enable(void)
{
	ath79_wdt_ping();

	/*
	 * Updating the TIMER register requires a few microseconds
	 * on the AR934x SoCs at least. Use a small delay to ensure
	 * that the TIMER register is updated within the hardware
	 * before enabling the watchdog.
	 */
	udelay(2);

	ath79_wdt_wr(WDOG_REG_CTRL, WDOG_CTRL_ACTION_FCR);
	/* flush write */
	ath79_wdt_rr(WDOG_REG_CTRL);
}

static int ath79_wdt_start(struct watchdog_device *wdd)
{
	if (test_and_set_bit(WDT_FLAGS_BUSY, &wdt_flags))
		return -EBUSY;

	clear_bit(WDT_FLAGS_EXPECT_CLOSE, &wdt_flags);
	ath79_wdt_enable();

	return 0;
}

static void ath79_wdt_disable(void)
{
	ath79_wdt_wr(WDOG_REG_CTRL, WDOG_CTRL_ACTION_NONE);
	/* flush write */
	ath79_wdt_rr(WDOG_REG_CTRL);
}

static unsigned int ath79_wdt_get_timeleft(struct watchdog_device *wdd)
{
	return ath79_wdt_rr(WDOG_REG_TIMER);
}

static int ath79_wdt_stop(struct watchdog_device *wdd)
{
	if (test_bit(WDT_FLAGS_EXPECT_CLOSE, &wdt_flags))
		ath79_wdt_disable();
	else {
		pr_crit("device closed unexpectedly, watchdog timer will not stop!\n");
		ath79_wdt_ping();
	}

	clear_bit(WDT_FLAGS_BUSY, &wdt_flags);
	clear_bit(WDT_FLAGS_EXPECT_CLOSE, &wdt_flags);

	return 0;
}

static int ath79_wdt_bootcause(void)
{
	ctrl = ath79_wdt_rr(WDOG_REG_CTRL);
	if (ath79_wdt_rr(ctrl & WDOG_CTRL_LAST_RESET)
		return WDIOF_CARDRESET;

	return 0;
}

static const struct watchdog_info ath79_wdt_info = {
	.options		= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING |
				  WDIOF_MAGICCLOSE | WDIOF_CARDRESET,
	.firmware_version	= 0,
	.identity		= "ATH79 watchdog",
};

static const struct watchdog_ops ath79_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ath79_wdt_start,
	.stop		= ath79_wdt_stop,
	.ping		= ath79_wdt_ping,
	.set_timeout	= ath79_wdt_set_timeout,
	.get_timeleft	= ath79_wdt_get_timeleft,
};

static struct watchdog_device ath79_wdt_dev = {
	.info		= &ath79_wdt_info,
	.ops		= &ath79_wdt_ops,
	.min_timeout	= 1,
	.max_timeout	= max_timeout,
	.timeout	= timeout,
};

static int ath79_wdt_probe(struct platform_device *pdev)
{
	wdt_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt_base))
		return PTR_ERR(wdt_base);

	wdt_clk = devm_clk_get_enabled(&pdev->dev, "wdt");
	if (IS_ERR(wdt_clk))
		return PTR_ERR(wdt_clk);

	wdt_freq = clk_get_rate(wdt_clk);
	if (!wdt_freq)
		return -EINVAL;

	ath79_wdt_dev.bootstatus = ath79_wdt_bootcause();

	watchdog_init_timeout(&ath79_wdt_dev, ath79_wdt_dev.max_timeout,
					      &pdev->dev);
	boot_status = ath79_wdt_bootcause();

	return devm_watchdog_register(&pdev->dev, &ath79_wdt_dev);
}

static void ath79_wdt_shutdown(struct platform_device *pdev)
{
	ath79_wdt_stop();
}

#ifdef CONFIG_OF
static const struct of_device_id ath79_wdt_match[] = {
	{ .compatible = "qca,ar7130-wdt" },
	{},
};
MODULE_DEVICE_TABLE(of, ath79_wdt_match);
#endif

static struct platform_driver ath79_wdt_driver = {
	.probe		= ath79_wdt_probe,
	.shutdown	= ath79_wdt_shutdown,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(ath79_wdt_match),
	},
};

module_platform_driver(ath79_wdt_driver);

MODULE_DESCRIPTION("Atheros AR71XX/AR724X/AR913X hardware watchdog driver");
MODULE_AUTHOR("Gabor Juhos <juhosg@openwrt.org");
MODULE_AUTHOR("Imre Kaloz <kaloz@openwrt.org");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
