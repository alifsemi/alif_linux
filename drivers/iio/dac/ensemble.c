// SPDX-License-Identifier: GPL-2.0-only /*
/*
 * IIO DAC driver for Alif Ensemble DAC
 * Code based on LPC18XX DAC
 *
 * Copyright (C) 2021 Harith George <harith.g@alifsemi.com>
 *
 */
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

/* Ensemble DAC registers and bits */
#define ENSEMBLE_DAC_REG1		0x000
#define ENSEMBLE_DAC_IN			0x004

struct ensemble_dac {
	void __iomem *base;
	struct clk *clk;
};

static const struct iio_chan_spec ensemble_dac_iio_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.output = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static int ensemble_dac_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct ensemble_dac *dac = iio_priv(indio_dev);

	*val = readl(dac->base + ENSEMBLE_DAC_IN);
	return IIO_VAL_INT;
}

static int ensemble_dac_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct ensemble_dac *dac = iio_priv(indio_dev);

	/* Ensemble has a 12 bit DAC */
	val &= 0xFFF;

	/* A0 revision has a HW bug that requires the same value to be
	 * written thrice for it to reflect on the output.
	 */
	writel(val, dac->base + ENSEMBLE_DAC_IN);
	writel(val, dac->base + ENSEMBLE_DAC_IN);
	writel(val, dac->base + ENSEMBLE_DAC_IN);
	return 0;
}

static const struct iio_info ensemble_dac_info = {
	.read_raw = ensemble_dac_read_raw,
	.write_raw = ensemble_dac_write_raw,
};

static int ensemble_dac_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct ensemble_dac *dac;
	void __iomem *tmp;
	struct resource *res;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*dac));
	if (!indio_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, indio_dev);
	dac = iio_priv(indio_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dac->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dac->base))
		return PTR_ERR(dac->base);

	dac->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dac->clk)) {
		dev_err(&pdev->dev, "error getting clock\n");
		return PTR_ERR(dac->clk);
	}
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &ensemble_dac_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ensemble_dac_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(ensemble_dac_iio_channels);

	ret = clk_prepare_enable(dac->clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable clock\n");
		goto exit;
	}

	/* Enable Analog Peripheral */
	printk("Enable analog peripheral.\n");
	tmp = ioremap(0x70040000, 0x40);
	writel(0x388C4230, tmp + 0x1C);
	iounmap(tmp);
	/* Enable DAC reference voltage */
	printk("Enable DAC reference voltage.\n");
	tmp = ioremap(0x49023000, 0x10);
	writel(0x10200000, tmp + 0x4);
	iounmap(tmp);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "unable to register device\n");
		goto dis_clk;
	}

	/*TODO - A0 revision of the SoC has a HW quirk that enable/reset
	 * (bit 27) for both DAC instances (0x49028000 and 0x49029000) are
	 * in 0x49028000 register
         */
	if(res->start == 0x49028000){
		printk("DAC setting addr 0x%x value 0x0E31C7FD\n",res->start);
		writel(0x0E31C7FD, dac->base + ENSEMBLE_DAC_REG1);
	} else {
		/* TODO Since the DAC at 0x49029000 can be enabled only by
		 * reseting bit 27 of 0x49028000, we use this DAC
		 * with the other DAC also enabled in the dtb.. All this is
		 * just temporary at the moment. Hopefully this will be
		 * fixed in HW soon.
		 */
		printk("DAC setting addr 0x%x value 0x0631C7FD\n",res->start);
		writel(0x0631C7FD, dac->base + ENSEMBLE_DAC_REG1);
	}

	printk("DAC set initial value to 0x0\n");
	writel(0x0, dac->base + ENSEMBLE_DAC_IN);

	return 0;

dis_clk:
	clk_disable_unprepare(dac->clk);
exit:
	return ret;
}

static int ensemble_dac_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct ensemble_dac *dac = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	/*TODO Disable DAC first */
	clk_disable_unprepare(dac->clk);

	return 0;
}

static const struct of_device_id ensemble_dac_match[] = {
	{ .compatible = "alif,ensemble-dac" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ensemble_dac_match);

static struct platform_driver ensemble_dac_driver = {
	.probe	= ensemble_dac_probe,
	.remove	= ensemble_dac_remove,
	.driver	= {
		.name = "ensemble-dac",
		.of_match_table = ensemble_dac_match,
	},
};
module_platform_driver(ensemble_dac_driver);

MODULE_DESCRIPTION("Ensemble DAC driver");
MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_LICENSE("GPL v2");
