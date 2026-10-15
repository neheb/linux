// SPDX-License-Identifier: GPL-2.0+
/*
 *  Watchdog driver for Broadcom BCM47XX
 *
 *  Copyright (C) 2008 Aleksandar Radovanovic <biblbroks@sezampro.rs>
 *  Copyright (C) 2009 Matthieu CASTET <castet.matthieu@free.fr>
 *  Copyright (C) 2012-2013 Hauke Mehrtens <hauke@hauke-m.de>
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bcm47xx_wdt.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/stringify.h>

#define DRV_NAME		"bcm47xx_wdt"

#define WDT_DEFAULT_TIME	30	/* seconds */
#define WDT_SOFTTIMER_MAX	255	/* seconds */
#define WDT_SOFTTIMER_THRESHOLD	60	/* seconds */

/* ChipCommon PMU watchdog of BCM4716/BCM4717/BCM4718 */
#define BCM4716_PMU_WATCHDOG	0x634
#define BCM4716_WDT_MAX_TICKS	GENMASK(23, 0)
#define BCM4716_WDT_TICKS_PER_MS 32	/* 32 kHz ILP clock */

static int timeout = WDT_DEFAULT_TIME;
static bool nowayout = WATCHDOG_NOWAYOUT;

module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, "Watchdog time in seconds. (default="
				__stringify(WDT_DEFAULT_TIME) ")");

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		"Watchdog cannot be stopped once started (default="
				__stringify(WATCHDOG_NOWAYOUT) ")");

static inline struct bcm47xx_wdt *bcm47xx_wdt_get(struct watchdog_device *wdd)
{
	return container_of(wdd, struct bcm47xx_wdt, wdd);
}

static int bcm47xx_wdt_hard_keepalive(struct watchdog_device *wdd)
{
	struct bcm47xx_wdt *wdt = bcm47xx_wdt_get(wdd);

	wdt->timer_set_ms(wdt, wdd->timeout * 1000);

	return 0;
}

static int bcm47xx_wdt_hard_start(struct watchdog_device *wdd)
{
	return 0;
}

static int bcm47xx_wdt_hard_stop(struct watchdog_device *wdd)
{
	struct bcm47xx_wdt *wdt = bcm47xx_wdt_get(wdd);

	wdt->timer_set(wdt, 0);

	return 0;
}

static int bcm47xx_wdt_hard_set_timeout(struct watchdog_device *wdd,
					unsigned int new_time)
{
	struct bcm47xx_wdt *wdt = bcm47xx_wdt_get(wdd);
	u32 max_timer = wdt->max_timer_ms;

	if (new_time < 1 || new_time > max_timer / 1000) {
		pr_warn("timeout value must be 1<=x<=%d, using %d\n",
			max_timer / 1000, new_time);
		return -EINVAL;
	}

	wdd->timeout = new_time;
	return 0;
}

static int bcm47xx_wdt_restart(struct watchdog_device *wdd,
			       unsigned long action, void *data)
{
	struct bcm47xx_wdt *wdt = bcm47xx_wdt_get(wdd);

	wdt->timer_set(wdt, 1);

	return 0;
}

static const struct watchdog_ops bcm47xx_wdt_hard_ops = {
	.owner		= THIS_MODULE,
	.start		= bcm47xx_wdt_hard_start,
	.stop		= bcm47xx_wdt_hard_stop,
	.ping		= bcm47xx_wdt_hard_keepalive,
	.set_timeout	= bcm47xx_wdt_hard_set_timeout,
	.restart        = bcm47xx_wdt_restart,
};

static void bcm47xx_wdt_soft_timer_tick(struct timer_list *t)
{
	struct bcm47xx_wdt *wdt = timer_container_of(wdt, t, soft_timer);
	u32 next_tick = min(wdt->wdd.timeout * 1000, wdt->max_timer_ms);

	if (!atomic_dec_and_test(&wdt->soft_ticks)) {
		wdt->timer_set_ms(wdt, next_tick);
		mod_timer(&wdt->soft_timer, jiffies + HZ);
	} else {
		pr_crit("Watchdog will fire soon!!!\n");
	}
}

static int bcm47xx_wdt_soft_keepalive(struct watchdog_device *wdd)
{
	struct bcm47xx_wdt *wdt = bcm47xx_wdt_get(wdd);

	atomic_set(&wdt->soft_ticks, wdd->timeout);

	return 0;
}

static int bcm47xx_wdt_soft_start(struct watchdog_device *wdd)
{
	struct bcm47xx_wdt *wdt = bcm47xx_wdt_get(wdd);

	bcm47xx_wdt_soft_keepalive(wdd);
	bcm47xx_wdt_soft_timer_tick(&wdt->soft_timer);

	return 0;
}

static int bcm47xx_wdt_soft_stop(struct watchdog_device *wdd)
{
	struct bcm47xx_wdt *wdt = bcm47xx_wdt_get(wdd);

	timer_delete_sync(&wdt->soft_timer);
	wdt->timer_set(wdt, 0);

	return 0;
}

static int bcm47xx_wdt_soft_set_timeout(struct watchdog_device *wdd,
					unsigned int new_time)
{
	if (new_time < 1 || new_time > WDT_SOFTTIMER_MAX) {
		pr_warn("timeout value must be 1<=x<=%d, using %d\n",
			WDT_SOFTTIMER_MAX, new_time);
		return -EINVAL;
	}

	wdd->timeout = new_time;
	return 0;
}

static const struct watchdog_info bcm47xx_wdt_info = {
	.identity	= DRV_NAME,
	.options	= WDIOF_SETTIMEOUT |
				WDIOF_KEEPALIVEPING |
				WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops bcm47xx_wdt_soft_ops = {
	.owner		= THIS_MODULE,
	.start		= bcm47xx_wdt_soft_start,
	.stop		= bcm47xx_wdt_soft_stop,
	.ping		= bcm47xx_wdt_soft_keepalive,
	.set_timeout	= bcm47xx_wdt_soft_set_timeout,
	.restart        = bcm47xx_wdt_restart,
};

static u32 bcm4716_wdt_timer_set(struct bcm47xx_wdt *wdt, u32 ticks)
{
	struct regmap *regmap = bcm47xx_wdt_get_drvdata(wdt);

	/* A count of 1 doesn't fire, so use the shortest count that does */
	if (ticks == 1)
		ticks = 2;
	else if (ticks > BCM4716_WDT_MAX_TICKS)
		ticks = BCM4716_WDT_MAX_TICKS;

	regmap_write(regmap, BCM4716_PMU_WATCHDOG, ticks);

	return ticks;
}

static u32 bcm4716_wdt_timer_set_ms(struct bcm47xx_wdt *wdt, u32 ms)
{
	u32 ticks;

	ticks = bcm4716_wdt_timer_set(wdt, ms * BCM4716_WDT_TICKS_PER_MS);

	return ticks / BCM4716_WDT_TICKS_PER_MS;
}

/* Set up a watchdog described in device tree, in the ChipCommon syscon */
static struct bcm47xx_wdt *bcm47xx_wdt_of_init(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm47xx_wdt *wdt;
	struct regmap *regmap;

	regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(regmap))
		return ERR_CAST(regmap);

	wdt = devm_kzalloc_obj(dev, *wdt);
	if (!wdt)
		return ERR_PTR(-ENOMEM);

	wdt->driver_data = regmap;
	wdt->timer_set = bcm4716_wdt_timer_set;
	wdt->timer_set_ms = bcm4716_wdt_timer_set_ms;
	wdt->max_timer_ms = BCM4716_WDT_MAX_TICKS / BCM4716_WDT_TICKS_PER_MS;

	return wdt;
}

static int bcm47xx_wdt_probe(struct platform_device *pdev)
{
	int ret;
	bool soft;
	struct bcm47xx_wdt *wdt = dev_get_platdata(&pdev->dev);

	if (!wdt && pdev->dev.of_node) {
		wdt = bcm47xx_wdt_of_init(pdev);
		if (IS_ERR(wdt))
			return dev_err_probe(&pdev->dev, PTR_ERR(wdt),
					     "failed to set up the watchdog\n");
	}

	if (!wdt)
		return -ENXIO;

	soft = wdt->max_timer_ms < WDT_SOFTTIMER_THRESHOLD * 1000;

	if (soft) {
		wdt->wdd.ops = &bcm47xx_wdt_soft_ops;
		timer_setup(&wdt->soft_timer, bcm47xx_wdt_soft_timer_tick, 0);
	} else {
		wdt->wdd.ops = &bcm47xx_wdt_hard_ops;
	}

	wdt->wdd.info = &bcm47xx_wdt_info;
	wdt->wdd.timeout = WDT_DEFAULT_TIME;
	wdt->wdd.parent = &pdev->dev;
	ret = wdt->wdd.ops->set_timeout(&wdt->wdd, timeout);
	if (ret)
		goto err_timer;
	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_set_restart_priority(&wdt->wdd, 64);
	watchdog_stop_on_reboot(&wdt->wdd);

	ret = devm_watchdog_register_device(&pdev->dev, &wdt->wdd);
	if (ret)
		goto err_timer;

	dev_info(&pdev->dev, "BCM47xx Watchdog Timer enabled (%d seconds%s%s)\n",
		timeout, nowayout ? ", nowayout" : "",
		soft ? ", Software Timer" : "");
	return 0;

err_timer:
	if (soft)
		timer_delete_sync(&wdt->soft_timer);

	return ret;
}

static const struct of_device_id bcm47xx_wdt_of_match[] = {
	{ .compatible = "brcm,bcm4716-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm47xx_wdt_of_match);

static struct platform_driver bcm47xx_wdt_driver = {
	.driver		= {
		.name	= "bcm47xx-wdt",
		.of_match_table = bcm47xx_wdt_of_match,
	},
	.probe		= bcm47xx_wdt_probe,
};

module_platform_driver(bcm47xx_wdt_driver);

MODULE_AUTHOR("Aleksandar Radovanovic");
MODULE_AUTHOR("Hauke Mehrtens <hauke@hauke-m.de>");
MODULE_DESCRIPTION("Watchdog driver for Broadcom BCM47xx");
MODULE_LICENSE("GPL");
