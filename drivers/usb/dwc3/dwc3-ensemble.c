// SPDX-License-Identifier: GPL-2.0
/**
 * dwc3-alif.c - ALIF BOLT DWC3 Specific Glue layer
 *
 * Copyright (c) 2021 ALIF India Pvt Ltd.
 *
 * Author: Nishit Sharma <nishit.sharma@alifsemi.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#define BOLT_MAX_CLOCKS	3

struct dwc3_bolt_driverdata {
	const char		*clk_names[BOLT_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;
};

struct dwc3_bolt {
	struct device		*dev;

	const char		**clk_names;
	struct clk		*clks[BOLT_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;

	struct regulator	*vdd33;
	struct regulator	*vdd18;
};

static int dwc3_bolt_remove_child(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static int dwc3_bolt_probe(struct platform_device *pdev)
{
	struct dwc3_bolt	*bolt;
	struct device		*dev = &pdev->dev;
	struct device_node	*node = dev->of_node;
	const struct dwc3_bolt_driverdata *driver_data;
	int	i, ret;

	bolt = devm_kzalloc(dev, sizeof(*bolt), GFP_KERNEL);
	if (!bolt)
		return -ENOMEM;

	driver_data = of_device_get_match_data(dev);
	bolt->dev = dev;
	bolt->num_clks = driver_data->num_clks;
	bolt->clk_names = (const char **)driver_data->clk_names;
	bolt->suspend_clk_idx = driver_data->suspend_clk_idx;

	platform_set_drvdata(pdev, bolt);

	for (i = 0; i < bolt->num_clks; i++) {
		bolt->clks[i] = devm_clk_get(dev, bolt->clk_names[i]);
		if (IS_ERR(bolt->clks[i])) {
			dev_err(dev, "failed to get clock: %s\n",
				bolt->clk_names[i]);
			return PTR_ERR(bolt->clks[i]);
		}
	}

	for (i = 0; i < bolt->num_clks; i++) {
		ret = clk_prepare_enable(bolt->clks[i]);
		if (ret) {
			while (i-- > 0)
				clk_disable_unprepare(bolt->clks[i]);
			return ret;
		}
	}

	if (bolt->suspend_clk_idx >= 0)
		clk_prepare_enable(bolt->clks[bolt->suspend_clk_idx]);

	bolt->vdd33 = devm_regulator_get(dev, "vdd33");
	if (IS_ERR(bolt->vdd33)) {
		ret = PTR_ERR(bolt->vdd33);
		goto vdd33_err;
	}
	ret = regulator_enable(bolt->vdd33);
	if (ret) {
		dev_err(dev, "Failed to enable VDD33 supply\n");
		goto vdd33_err;
	}

	bolt->vdd18 = devm_regulator_get(dev, "vdd18");
	if (IS_ERR(bolt->vdd18)) {
		ret = PTR_ERR(bolt->vdd18);
		goto vdd18_err;
	}
	ret = regulator_enable(bolt->vdd18);
	if (ret) {
		dev_err(dev, "Failed to enable VDD18 supply\n");
		goto vdd18_err;
	}

	if (node) {
		ret = of_platform_populate(node, NULL, NULL, dev);
		if (ret) {
			dev_err(dev, "failed to add dwc3 core\n");
			goto populate_err;
		}
	} else {
		dev_err(dev, "no device node, failed to add dwc3 core\n");
		ret = -ENODEV;
		goto populate_err;
	}
	return 0;

populate_err:
	regulator_disable(bolt->vdd18);
vdd18_err:
	regulator_disable(bolt->vdd33);
vdd33_err:
	for (i = bolt->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(bolt->clks[i]);

	if (bolt->suspend_clk_idx >= 0)
		clk_disable_unprepare(bolt->clks[bolt->suspend_clk_idx]);

	return ret;
}

static int dwc3_bolt_remove(struct platform_device *pdev)
{
	struct dwc3_bolt	*bolt = platform_get_drvdata(pdev);
	int i;

	device_for_each_child(&pdev->dev, NULL, dwc3_bolt_remove_child);

	for (i = bolt->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(bolt->clks[i]);

	if (bolt->suspend_clk_idx >= 0)
		clk_disable_unprepare(bolt->clks[bolt->suspend_clk_idx]);

	regulator_disable(bolt->vdd33);
	regulator_disable(bolt->vdd18);

	return 0;
}

static const struct dwc3_bolt_driverdata dwc3_bolt_drvdata = {
	.clk_names = {""},
	.num_clks = 0,
	.suspend_clk_idx = -1,
};

static const struct of_device_id dwc3_bolt_match[] = {
	{
		.compatible = "alif,ensemble-usb3",
		.data = &dwc3_bolt_drvdata,
	},
};
MODULE_DEVICE_TABLE(of, dwc3_bolt_match);

#ifdef CONFIG_PM_SLEEP
static int dwc3_bolt_suspend(struct device *dev)
{
	struct dwc3_bolt *bolt = dev_get_drvdata(dev);
	int i;

	for (i = bolt->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(bolt->clks[i]);

	regulator_disable(bolt->vdd33);
	regulator_disable(bolt->vdd18);

	return 0;
}

static int dwc3_bolt_resume(struct device *dev)
{
	struct dwc3_bolt *bolt = dev_get_drvdata(dev);
	int i, ret;

	ret = regulator_enable(bolt->vdd33);
	if (ret) {
		dev_err(dev, "Failed to enable VDD33 supply\n");
		return ret;
	}
	ret = regulator_enable(bolt->vdd18);
	if (ret) {
		dev_err(dev, "Failed to enable VDD18 supply\n");
		return ret;
	}

	for (i = 0; i < bolt->num_clks; i++) {
		ret = clk_prepare_enable(bolt->clks[i]);
		if (ret) {
			while (i-- > 0)
				clk_disable_unprepare(bolt->clks[i]);
			return ret;
		}
	}

	return 0;
}

static const struct dev_pm_ops bolt_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_bolt_suspend, dwc3_bolt_resume)
};

#define DEV_PM_OPS	(&bolt_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver bolt_driver = {
	.probe		= dwc3_bolt_probe,
	.remove		= dwc3_bolt_remove,
	.driver		= {
		.name	= "alif-bolt",
		.of_match_table = dwc3_bolt_match,
		.pm	= DEV_PM_OPS,
	},
};

module_platform_driver(bolt_driver);

MODULE_AUTHOR("Nishit Sharma <nishit.sharma@alifsemi.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 BOLT  Glue Layer");
