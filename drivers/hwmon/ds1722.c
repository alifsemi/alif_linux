// SPDX-License-Identifier: GPL-2.0-only
/*
 * ds1722 - hwmon driver for Dallas Integrated ds1722 SPI
 * digital temperature sensor.
 *
 * Copyright (c) 2021, Alif Semiconductors.
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/device.h>

/* GPIO2 PortA registers*/
#define  GPIO2_DR                       0x49001000
#define  GPIO2_DDR                      0x49001004

/* Software control of GPIO line by making CS line High and low */
#define	 ENABLE_HIGH_OUPUT		0x01000000
#define	 DISABLE_HIGH_OUPUT		0x00000000

/* DS1722 Read registers */
#define DS1722_REG_CFG				0x00
#define DS1722_REG_TEMP_LSB			0x01
#define DS1722_REG_TEMP_MSB			0x02

/* DS1722 write register */
#define DS1722_REG_WRITE			0x80

/* DS1722 configuration  */
#define DS1722_MODE_CONTINUOUS			0x00
#define DS1722_MODE_STANDBY			0x01
#define DS1722_MODE_MASK			0xE0
#define DS1722_RESOLUTION_12BIT			0x08

void __iomem *gpio2_dr;
void __iomem *gpio2_ddr;

struct ds1722_data {
	struct device *hwmon_dev;
	struct spi_device *spi_device;
	u8 mode;
};

static int ds1722_set_mode(struct ds1722_data *data, u8 mode)
{
	int ret;
	struct spi_device *spi = data->spi_device;

	/* Configuration Register Setting */
	u8 buf[2] = {
		 DS1722_REG_WRITE,
		(data->mode | DS1722_MODE_MASK) | mode
	};

	/* CE high  */
	writel(ENABLE_HIGH_OUPUT, gpio2_dr);
	writel(ENABLE_HIGH_OUPUT, gpio2_ddr);
	udelay(1);

	/* Configuration Register Programing and Writing Data */
	ret = spi_write(spi, &buf[0], 1);
	ret = spi_write(spi, &buf[1], 1);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to set sensor mode.\n");
		return ret;
	}
	data->mode = (data->mode & DS1722_MODE_MASK) | mode;
	/* CE Low  */
	writel(DISABLE_HIGH_OUPUT, gpio2_dr);
	writel(DISABLE_HIGH_OUPUT, gpio2_ddr);
	udelay(1);
	return 0;
}

static ssize_t ds1722_temp_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ds1722_data *data = dev_get_drvdata(dev);
	u32 dec_val;
	u16 res;
	u8 temp;

	/* CE high  */
	writel(ENABLE_HIGH_OUPUT, gpio2_dr);
	writel(ENABLE_HIGH_OUPUT, gpio2_ddr);
	udelay(1);


	/* First READ MSB Register(0x2) to get Temperature reading */
	res = (temp = spi_w8r8(data->spi_device, DS1722_REG_TEMP_MSB)) << 8;
	/* CE Low  */
	writel(DISABLE_HIGH_OUPUT, gpio2_dr);
        writel(DISABLE_HIGH_OUPUT, gpio2_ddr);
	udelay(1);
	/* CE High  */
	writel(ENABLE_HIGH_OUPUT, gpio2_dr);
	writel(ENABLE_HIGH_OUPUT, gpio2_ddr);

	if (temp < 0) {
	/* READ LSB Register(0x1) to get decimal Temperature reading  */
		temp = (16 - (spi_w8r8(data->spi_device, DS1722_REG_TEMP_LSB) >> 4)) & 0x0f;
	} else {
	/* READ LSB Register(0x1) to get decimal Temperature reading  */
		temp = (spi_w8r8(data->spi_device, DS1722_REG_TEMP_LSB) >> 4);
	}
	udelay(1);

	/* After Reanding temperature value from LSB register(0x1),
	 * each bit represents 0.0625 degrees Celsius. So If temp=2
	 * than 0.0625 will multiply with 2 which is quivalent=.1250
	 */
	switch (temp) {
	case 0:
		/* .0000 */
		dec_val=0000;
		break;
	case 1:
		/* .0625 */
		dec_val=0625;
		break;
	case 2:
		/* .1250 */
		dec_val=1250;
		break;
	case 3:
		/* .1875 */
		dec_val=1875;
		break;
	case 4:
		/* .2500 */
		dec_val=2500;
		break;
	case 5:
		/* .3125 */
		dec_val=3125;
		break;
	case 6:
		/* .3750 */
		dec_val=3750;
		break;
	case 7:
		/* .4375 */
		dec_val=4375;
		break;
	case 8:
		/* .5000 */
		dec_val=5000;
		break;
	case 9:
		/* .5625 */
		dec_val=5625;
		break;
	case 10:
		/* .6250 */
		dec_val=6250;
		break;
	case 11:
		/* .6875 */
		dec_val=6875;
		break;
	case 12:
		/* .7500 */
		dec_val=7500;
		break;
	case 13:
		/* .8125 */
		dec_val=8125;
		break;
	case 14:
		/* .8750 */
		dec_val=8750;
		break;
	case 15:
		/* .9375 */
		dec_val=9375;
		break;
	}

	/* CE Low  */
	writel(DISABLE_HIGH_OUPUT, gpio2_dr);
	writel(DISABLE_HIGH_OUPUT, gpio2_ddr);
	udelay(1);

	return pr_info("\n %d.%d Degree Celsius\n\n", (char)(res >> 8), dec_val);
}

static SENSOR_DEVICE_ATTR_RO(temp1_input, ds1722_temp, 0);

static struct attribute *ds1722_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(ds1722);

#ifdef CONFIG_OF
static const struct of_device_id ds1722_of_ids[] = {
        {
                .compatible = "ti,ds1722",
                .data = 0,
        },
        {},
};
MODULE_DEVICE_TABLE(of, ds1722_of_ids);
#endif


static int ds1722_probe(struct spi_device *spi)
{
	int ret;
	struct ds1722_data *data;

	gpio2_dr = ioremap(GPIO2_DR, 4);
	gpio2_ddr = ioremap(GPIO2_DDR, 4);

	data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spi_set_drvdata(spi, data);
	data->spi_device = spi;
	/*
	 * Set SD bit to 0 so we can have continuous measurements.
	 * Set resolution to 12 bits for maximum precision.
	 */
	data->mode = DS1722_MODE_CONTINUOUS | DS1722_RESOLUTION_12BIT;
	ret = ds1722_set_mode(data, DS1722_MODE_CONTINUOUS);
	if (ret < 0)
		return ret;

	data->hwmon_dev = hwmon_device_register_with_groups(&spi->dev,
							    spi->modalias,
							    data,
							    ds1722_groups);
	if (IS_ERR(data->hwmon_dev)) {
		ds1722_set_mode(data, DS1722_MODE_STANDBY);
		return PTR_ERR(data->hwmon_dev);
	}

	dev_info(&(data->spi_device->dev),"DS1722 Temperature sensor enabled\n");
	return 0;
}

static int ds1722_remove(struct spi_device *spi)
{
	struct ds1722_data *data = spi_get_drvdata(spi);

	hwmon_device_unregister(data->hwmon_dev);

	return ds1722_set_mode(data, DS1722_MODE_STANDBY);
}

static int __maybe_unused ds1722_suspend(struct device *dev)
{
	struct spi_device *spi_device = to_spi_device(dev);
	struct ds1722_data *data = spi_get_drvdata(spi_device);

	return ds1722_set_mode(data, DS1722_MODE_STANDBY);
}

static int __maybe_unused ds1722_resume(struct device *dev)
{
	struct spi_device *spi_device = to_spi_device(dev);
	struct ds1722_data *data = spi_get_drvdata(spi_device);

	return ds1722_set_mode(data, DS1722_MODE_CONTINUOUS);
}

static SIMPLE_DEV_PM_OPS(ds1722_pm_ops, ds1722_suspend, ds1722_resume);


static const struct spi_device_id  ds1722_id[] = {
	{"ds1722", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, ds1722_id);

static struct spi_driver ds1722_driver = {
	.driver = {
		.name = "ds1722",
		.pm = &ds1722_pm_ops,
		.of_match_table = of_match_ptr(ds1722_of_ids),
	},
	.id_table = ds1722_id,
	.probe = ds1722_probe,
	.remove =   ds1722_remove,
};

module_spi_driver(ds1722_driver);

MODULE_AUTHOR("Pankaj Pandey <ppandey@alifsemi.com>");
MODULE_DESCRIPTION("ds1722 sensor driver");
MODULE_LICENSE("GPL v2");
