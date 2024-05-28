// SPDX-License-Identifier: GPL-2.0-only /*
/*
 * IIO DAC driver for Alif Ensemble DAC
 * Code based on LPC18XX DAC
 *
 * Copyright (C) 2021-2024 Harith George <harith.g@alifsemi.com>
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

#define DAC_CTRL_DAC0_CKEN                  (1U  <<  0U)
#define DAC_CTRL_DAC1_CKEN                  (1U  <<  4U)

/* Ensemble DAC registers and bits */
#define ENSEMBLE_DAC_REG1		0x000
#define ENSEMBLE_DAC_IN			0x004

/* DAC  Control register */
#define DAC_EN                   (1U << 0)   /* Enable DAC */
#define DAC_RESET_B              (1U << 27)  /* 0=Reset,this will reset the DAC */
#define DAC_HP_MODE_EN           (1U << 18)  /* To enable the dac output buffer */
#define DAC_MAX_INPUT            (0xFFFU)    /* Maximum input for the DAC is 4095(DAC 12 bit resolution) */
#define DAC_IN_BYP_MUX           (1U << 1U)  /* Select the DAC input data source */
#define DAC_MAX_BYP_VAL_Msk      (0x3FFCU)   /* DAC input data in bypass mode */
#define DAC_TWOSCOMP_Pos          22U        /* Converts two's complement to unsigned binary data */
#define DAC_INPUT_BYP_MUX_Pos     1U         /* Set DAC input source in bypass mode */
#define DAC_BYP_VAL_Pos           2U         /* DAC input data bypass mode   */
#define DAC_IBIAS_VAL_Pos         23U        /* DAC buffer output current    */
#define DAC_CAP_VAL_Pos           14U        /* DAC capacitance compensation */

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
	unsigned int reg;

	/* Ensemble has a 12 bit DAC */
	val &= 0xFFF;
	writel(val, dac->base + ENSEMBLE_DAC_IN);

	reg = readl(dac->base + ENSEMBLE_DAC_REG1);
	reg |= DAC_EN; /* DAC Enable */
	writel(reg, dac->base + ENSEMBLE_DAC_REG1);

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
	unsigned int reg;

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

	tmp = ioremap(0x4902F000, 0x40);
	reg = readl(tmp + 0x38);
	reg |= (1 << 0);
	writel(reg, tmp + 0x38);

	reg = readl(tmp + 0x34);
	if (res->start == 0x49028000)
		reg |= DAC_CTRL_DAC0_CKEN;
	else
		reg |= DAC_CTRL_DAC1_CKEN;

	writel(reg, tmp + 0x34);
	iounmap(tmp);

	/* Enable Analog Peripheral */
	tmp = ioremap(0x1A60A000, 0x40);
	reg = readl(tmp + 0x3C);
	reg |= ((1 << 22) | (1 << 23));
	writel(reg, tmp + 0x3C);
	iounmap(tmp);

	/* Enable DAC reference voltage */
	tmp = ioremap(0x49023000, 0x10);
	writel((0x1 << 27) | (0x20 << 21) | (0x1 << 20) |
		(0x4 << 17) | (0x0 << 16) | (0x1 << 15) |
		(0x10 << 10) | (0xA << 6) |  (0xA << 1), tmp + 0x4);
	iounmap(tmp);

	reg = readl(dac->base + ENSEMBLE_DAC_REG1);
	reg &= ~DAC_EN; /* DAC Disable */
	writel(reg, dac->base + ENSEMBLE_DAC_REG1);
	reg |= DAC_RESET_B;
	writel(reg, dac->base + ENSEMBLE_DAC_REG1);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "unable to register device\n");
		goto dis_clk;
	}

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
