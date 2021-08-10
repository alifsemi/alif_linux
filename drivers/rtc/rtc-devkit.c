// SPDX-License-Identifier: GPL-2.0+
/*
 * The hardware IP in the Alif SoC is really a Synopsys DW-APB timer.
 * This code is based on APM X-Gene SoC Real Time Clock Driver. The X-gene
 * soc also is likely using the same IP. The Linux kernel did have an RTC
 * driver for this IP but at some point that code was deprecated, and it was
 * classified as a clockevent/clocksource (drivers/clocksource/dw_apb_timer.c)
 * since the IP does not maintain date and time like other RTC devices and is
 * just a counter in hardware. The date/time maintenance is done in software
 * for this driver.
 *
 * Copyright (c) 2021, Alif Semicondutor
 * Author: Harith George <harith.g@alifsemi.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>

/* RTC CSR Registers */
#define RTC_CCVR			0x00
#define RTC_CMR				0x04
#define RTC_CLR				0x08
#define RTC_CCR				0x0C
#define RTC_CCR_IE			BIT(0)
#define RTC_CCR_MASK			BIT(1)
#define RTC_CCR_EN			BIT(2)
#define RTC_CCR_WEN			BIT(3)
#define RTC_CCR_PSCLREN			BIT(4)
#define RTC_STAT			0x10
#define RTC_STAT_BIT			BIT(0)
#define RTC_RSTAT			0x14
#define RTC_EOI				0x18
#define RTC_VER				0x1C
#define RTC_CPCR			0x20

struct bolt_rtc_dev {
	struct rtc_device *rtc;
	struct device *dev;
	void __iomem *csr_base;
	struct clk *clk;
	unsigned int irq_wake;
	unsigned int irq_enabled;
};

static int bolt_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct bolt_rtc_dev *pdata = dev_get_drvdata(dev);

	rtc_time64_to_tm(readl(pdata->csr_base + RTC_CCVR), tm);
	return 0;
}

static int bolt_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct bolt_rtc_dev *pdata = dev_get_drvdata(dev);

	/*
	 * NOTE: After the following write, the RTC_CCVR is only reflected
	 *       after the update cycle of 1 seconds.
	 */
	writel((u32)rtc_tm_to_time64(tm), pdata->csr_base + RTC_CLR);
	readl(pdata->csr_base + RTC_CLR); /* Force a barrier */

	return 0;
}

static int bolt_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct bolt_rtc_dev *pdata = dev_get_drvdata(dev);

	/* If possible, CMR should be read here */
	rtc_time64_to_tm(0, &alrm->time);
	alrm->enabled = readl(pdata->csr_base + RTC_CCR) & RTC_CCR_IE;

	return 0;
}

static int bolt_rtc_alarm_irq_enable(struct device *dev, u32 enabled)
{
	struct bolt_rtc_dev *pdata = dev_get_drvdata(dev);
	u32 ccr;

	ccr = readl(pdata->csr_base + RTC_CCR);
	if (enabled) {
		ccr &= ~RTC_CCR_MASK;
		ccr |= RTC_CCR_IE;
	} else {
		ccr &= ~RTC_CCR_IE;
		ccr |= RTC_CCR_MASK;
	}
	writel(ccr, pdata->csr_base + RTC_CCR);

	return 0;
}

static int bolt_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct bolt_rtc_dev *pdata = dev_get_drvdata(dev);

	writel((u32)rtc_tm_to_time64(&alrm->time), pdata->csr_base + RTC_CMR);

	bolt_rtc_alarm_irq_enable(dev, alrm->enabled);

	return 0;
}

static const struct rtc_class_ops bolt_rtc_ops = {
	.read_time	= bolt_rtc_read_time,
	.set_time	= bolt_rtc_set_time,
	.read_alarm	= bolt_rtc_read_alarm,
	.set_alarm	= bolt_rtc_set_alarm,
	.alarm_irq_enable = bolt_rtc_alarm_irq_enable,
};

static irqreturn_t bolt_rtc_interrupt(int irq, void *id)
{
	struct bolt_rtc_dev *pdata = id;

	/* Check if interrupt asserted */
	if (!(readl(pdata->csr_base + RTC_STAT) & RTC_STAT_BIT))
		return IRQ_NONE;

	/* Clear interrupt */
	readl(pdata->csr_base + RTC_EOI);

	rtc_update_irq(pdata->rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static int bolt_rtc_probe(struct platform_device *pdev)
{
	struct bolt_rtc_dev *pdata;
	struct resource *res;
	int ret;
	int irq;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	platform_set_drvdata(pdev, pdata);
	pdata->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pdata->csr_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pdata->csr_base))
		return PTR_ERR(pdata->csr_base);

	pdata->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(pdata->rtc))
		return PTR_ERR(pdata->rtc);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(&pdev->dev, irq, bolt_rtc_interrupt, 0,
			       dev_name(&pdev->dev), pdata);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		return ret;
	}

	pdata->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pdata->clk)) {
		dev_err(&pdev->dev, "Couldn't get the clock for RTC\n");
		return -ENODEV;
	}
	ret = clk_prepare_enable(pdata->clk);
	if (ret)
		return ret;

	/* The rtc clock should be at 32.768KHz */
	/* Set the prescaler to 32768 so that the counter is
	 * incremented every second. And turn on the clock */
	writel(32768, pdata->csr_base + RTC_CPCR);
	writel(RTC_CCR_PSCLREN | RTC_CCR_EN, pdata->csr_base + RTC_CCR);

	ret = device_init_wakeup(&pdev->dev, 1);
	if (ret) {
		clk_disable_unprepare(pdata->clk);
		return ret;
	}

	/* HW does not support update faster than 1 seconds */
	pdata->rtc->uie_unsupported = 1;
	pdata->rtc->ops = &bolt_rtc_ops;
	pdata->rtc->range_max = U32_MAX;

	ret = rtc_register_device(pdata->rtc);
	if (ret) {
		clk_disable_unprepare(pdata->clk);
		return ret;
	}

	return 0;
}

static int bolt_rtc_remove(struct platform_device *pdev)
{
	struct bolt_rtc_dev *pdata = platform_get_drvdata(pdev);

	bolt_rtc_alarm_irq_enable(&pdev->dev, 0);
	device_init_wakeup(&pdev->dev, 0);
	clk_disable_unprepare(pdata->clk);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bolt_rtc_of_match[] = {
	{.compatible = "alif,devkit-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, bolt_rtc_of_match);
#endif

static struct platform_driver bolt_rtc_driver = {
	.probe		= bolt_rtc_probe,
	.remove		= bolt_rtc_remove,
	.driver		= {
		.name	= "devkit-rtc",
		.of_match_table	= of_match_ptr(bolt_rtc_of_match),
	},
};

module_platform_driver(bolt_rtc_driver);

MODULE_DESCRIPTION("Alif Devkit SoC RTC driver");
MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_LICENSE("GPL");
