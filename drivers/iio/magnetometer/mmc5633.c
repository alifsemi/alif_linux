// SPDX-License-Identifier: GPL-2.0-only
/*
 * MMC5633 - MEMSIC 3-axis Magnetic Sensor
 *
 * Copyright (c) 2022, Alif Semiconductor
 *
 * I3C IIO driver for MMC5633
 *
 * Author: Harith George <harith.g@alifsemi.com>
 * Acknowledgement: This code is based on the MMC35240 I2C driver.
 * The magnetometer x/y/z data readout code has not been extensively tested.
 * But the temperature register readout seemed working okay on an Alif board.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/pm.h>

#include <linux/kernel.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define MMC5633_DRV_NAME		"mmc5633"
#define MMC5633_REGMAP_NAME		"mmc5633_regmap"

#define MMC5633_REG_XOUT0		0x00
#define MMC5633_REG_XOUT1		0x01
#define MMC5633_REG_YOUT0		0x02
#define MMC5633_REG_YOUT1		0x03
#define MMC5633_REG_ZOUT0		0x04
#define MMC5633_REG_ZOUT1		0x05
#define MMC5633_REG_XOUT2		0x06
#define MMC5633_REG_YOUT2		0x07
#define MMC5633_REG_ZOUT2		0x08
#define MMC5633_REG_TOUT		0x09
#define MMC5633_REG_TPH0		0x0A
#define MMC5633_REG_TPH1		0x0B
#define MMC5633_REG_TU			0x0C
#define MMC5633_REG_STATUS1		0x18
#define MMC5633_REG_STATUS0		0x19
#define MMC5633_REG_ODR			0x1A
#define MMC5633_REG_CTRL0		0x1B
#define MMC5633_REG_CTRL1		0x1C
#define MMC5633_REG_CTRL2		0x1D
#define MMC5633_REG_ST_X_TH		0x1E
#define MMC5633_REG_ST_Y_TH		0x1F
#define MMC5633_REG_ST_Z_TH		0x20
#define MMC5633_REG_ST_X		0x27
#define MMC5633_REG_ST_Y		0x28
#define MMC5633_REG_ST_Z		0x29
#define MMC5633_REG_PROD_ID		0x39

#define MMC5633_STATUS1_MEAS_M_DONE	BIT(6)
#define MMC5633_STATUS1_MEAS_T_DONE	BIT(7)

#define MMC5633_CTRL0_CMM_BIT		BIT(1)

/* Take Measure of Magnetic field*/
#define MMC5633_CTRL0_TM_M		BIT(0)

/* Take Measure of Temperature*/
#define MMC5633_CTRL0_TM_T		BIT(1)

#define MMC5633_CTRL0_SET		BIT(3)
#define MMC5633_CTRL0_RESET		BIT(4)
#define MMC5633_CTRL0_AUTO_SR		BIT(5)

/* output resolution bits */
#define MMC5633_CTRL1_BW0_BIT		BIT(0)
#define MMC5633_CTRL1_BW1_BIT		BIT(1)

#define MMC5633_CTRL1_BW_MASK		(MMC5633_CTRL1_BW0_BIT | \
					 MMC5633_CTRL1_BW1_BIT)
#define MMC5633_CTRL1_BW_SHIFT		0

#define MMC5633_WAIT_START		5000	/* us */
#define MMC5633_WAIT_SET_RESET		1000	/* us */

#define MMC5633_OTP_START_ADDR		0x1B

enum mmc5633_resolution {
	MMC5633_BW_00 = 0, /* 6.6 ms */
	MMC5633_BW_01,     /* 3.5 ms */
	MMC5633_BW_10,     /* 2.0 ms */
	MMC5633_BW_11,     /* 1.2 ms */
};

enum mmc5633_addr {
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z,
	TEMP,
};

static const struct {
	int sens[3]; /* sensitivity per X, Y, Z axis */
	int nfo; /* null field output */
} mmc5633_props_table[] = {
	/* 20 bits */
	{
		{16384, 16384, 16384},
		524288,
	},
	/* 18 bits */
	{
		{4096, 4096, 4096},
		131072,
	},
	/* 16 bits */
	{
		{1024, 1024, 1024},
		32768,
	},
};

struct mmc5633_data {
	struct i3c_device *client;
	struct mutex mutex;
	struct regmap *regmap;
	enum mmc5633_resolution res;
};

static const struct {
	int val;
	int val2;
} mmc5633_samp_freq[] = { {75, 0},
			   {150, 0},
			   {255, 0},
			   {1000, 0} };

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("75 150 255 1000");

#define MMC5633_CHANNEL(_axis) { \
	.type = IIO_MAGN, \
	.modified = 1, \
	.channel2 = IIO_MOD_ ## _axis, \
	.address = AXIS_ ## _axis, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
			BIT(IIO_CHAN_INFO_SCALE), \
}

static const struct iio_chan_spec mmc5633_channels[] = {
	MMC5633_CHANNEL(X),
	MMC5633_CHANNEL(Y),
	MMC5633_CHANNEL(Z),
	{ 	.type = IIO_TEMP,
		.modified = 1,
		.channel2 = IIO_MOD_TEMP_AMBIENT,
		.address = TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					BIT(IIO_CHAN_INFO_SCALE),
	}
};

static struct attribute *mmc5633_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group mmc5633_attribute_group = {
	.attrs = mmc5633_attributes,
};

static int mmc5633_get_samp_freq_index(struct mmc5633_data *data,
					int val, int val2)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mmc5633_samp_freq); i++)
		if (mmc5633_samp_freq[i].val == val &&
		    mmc5633_samp_freq[i].val2 == val2)
			return i;
	return -EINVAL;
}

static int mmc5633_hw_set(struct mmc5633_data *data, bool set)
{
	u8 coil_bit;

	if (set)
		coil_bit = MMC5633_CTRL0_SET;
	else
		coil_bit = MMC5633_CTRL0_RESET;

	return regmap_update_bits(data->regmap, MMC5633_REG_CTRL0,
				  coil_bit, coil_bit);

}

static int mmc5633_init(struct mmc5633_data *data)
{
	int ret;
	unsigned int reg_id;

	/* Most likely below delay is not needed as we would have
	 * already spent more than 5ms after Vdd has been applied.
	 */
	usleep_range(MMC5633_WAIT_START, MMC5633_WAIT_START + 1);

	ret = regmap_read(data->regmap, MMC5633_REG_PROD_ID, &reg_id);
	if (ret < 0) {
		dev_err(&data->client->dev, "Error reading product id\n");
		return ret;
	}

	dev_dbg(&data->client->dev, "MMC5633 chip id 0x%x\n", reg_id);

	/*
	 * make sure we restore sensor characteristics, by doing
	 * a SET/RESET sequence, the axis polarity being naturally
	 * aligned after RESET
	 */
	ret = mmc5633_hw_set(data, true);
	if (ret < 0)
		return ret;
	usleep_range(MMC5633_WAIT_SET_RESET, MMC5633_WAIT_SET_RESET + 1);

	ret = mmc5633_hw_set(data, false);
	if (ret < 0)
		return ret;
	usleep_range(MMC5633_WAIT_SET_RESET, MMC5633_WAIT_SET_RESET + 1);

	/* Set Auto SR */
	ret = regmap_update_bits(data->regmap, MMC5633_REG_CTRL0,
			  MMC5633_CTRL0_AUTO_SR, MMC5633_CTRL0_AUTO_SR);
	if (ret < 0)
		return ret;

	/* Set default sampling frequency */
	ret = regmap_update_bits(data->regmap, MMC5633_REG_CTRL1,
				 MMC5633_CTRL1_BW_MASK,
				 data->res << MMC5633_CTRL1_BW_SHIFT);
	if (ret < 0)
		return ret;

	return 0;
}

/**
 * mmc5633_raw_to_mgauss - convert raw readings to milli gauss. Also apply
			    compensation for output value.
 *
 * @data: device private data
 * @index: axis index for which we want the conversion
 * @buf: raw data to be converted, 2 bytes in little endian format
 * @val: compensated output reading (unit is milli gauss)
 *
 * Returns: 0 in case of success, -EINVAL when @index is not valid
 */
static int mmc5633_raw_to_mgauss(struct mmc5633_data *data, int index,
				  unsigned char buf[], int *val)
{
	int raw[3];
	int sens[3];
	int nfo;

	raw[AXIS_X] = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	raw[AXIS_Y] = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
	raw[AXIS_Z] = (buf[6] << 12) | (buf[7] << 4) | (buf[8] >> 4);

	sens[AXIS_X] = mmc5633_props_table[data->res].sens[AXIS_X];
	sens[AXIS_Y] = mmc5633_props_table[data->res].sens[AXIS_Y];
	sens[AXIS_Z] = mmc5633_props_table[data->res].sens[AXIS_Z];

	nfo = mmc5633_props_table[data->res].nfo;

	switch (index) {
	case AXIS_X:
		*val = (raw[AXIS_X] - nfo) * 1000 / sens[AXIS_X];
		break;
	case AXIS_Y:
		*val = (raw[AXIS_Y] - nfo) * 1000 / sens[AXIS_Y] -
			(raw[AXIS_Z] - nfo)  * 1000 / sens[AXIS_Z];
		break;
	case AXIS_Z:
		*val = (raw[AXIS_Y] - nfo) * 1000 / sens[AXIS_Y] +
			(raw[AXIS_Z] - nfo) * 1000 / sens[AXIS_Z];
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mmc5633_read_measurement(struct mmc5633_data *data, int index,
				 unsigned char *buf, int *val, int *val2)
{
	int ret, tries = 10;
	unsigned int reg_status;
	unsigned char take_meas_bit, status_done;
	unsigned int temp;

	if(index == TEMP){
		take_meas_bit = MMC5633_CTRL0_TM_T;
		status_done = MMC5633_STATUS1_MEAS_T_DONE;
	}
	else{
		take_meas_bit = MMC5633_CTRL0_TM_M;
		status_done = MMC5633_STATUS1_MEAS_M_DONE;
	}

	ret = regmap_write(data->regmap, MMC5633_REG_CTRL0,
			   take_meas_bit);
	if (ret < 0)
		return ret;

	while (tries-- > 0) {
		ret = regmap_read(data->regmap, MMC5633_REG_STATUS1,
				  &reg_status);
		if (ret < 0)
			return ret;
		if (reg_status & status_done)
			break;
		/* max wait time to complete measurement is ~7 ms */
		usleep_range(7000, 8000);
	}

	if (tries < 0) {
		dev_err(&data->client->dev, "data not ready\n");
		return -EIO;
	}

	if(index == TEMP){
		ret = regmap_read(data->regmap, MMC5633_REG_TOUT, &temp);
		if (ret < 0)
			return ret;
		buf[9] = temp;
		/* The 8 bit range corresponds to -75°C to +125°C
		 0 corresponds to -75°C */

		/* Temp readout needs to be scaled to 200 / 256 of the readout
		 * as full range of 256 bits cover 200 degrees celcius*/
		/* Multiply by another factor of 100 to get the fractional part later */
		temp = 200 * 100 * temp;
		temp = temp >> 8; /* Divide by 256 */
		temp -= 7500;	  /* Temp measured b/w -75C to +125C*/
		*val = temp / 100;

		/* Multiply by 10^4 as val2 is in micro units*/
		*val2 = (temp % 100) * 10000;
		return IIO_VAL_INT_PLUS_MICRO;
	}

	/* Read 9 bytes out */
	ret = regmap_bulk_read(data->regmap, MMC5633_REG_XOUT0,
			(u8 *)&buf[0],	9 * sizeof(char));
	if (ret < 0)
		return ret;
	ret = mmc5633_raw_to_mgauss(data, index, buf, val);
	if (ret < 0)
		return ret;
	return IIO_VAL_INT;
}

static int mmc5633_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	struct mmc5633_data *data = iio_priv(indio_dev);
	int ret, i;
	unsigned int reg;
	/* 10 bytes ->  xout0 xout1, xout2,
			yout0, yout1, yout2,
			zout0, zout1, zout2,
			temp */
	char buf[10];

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->mutex);
		ret = mmc5633_read_measurement(data, chan->address, buf, val, val2);
		mutex_unlock(&data->mutex);
		return ret;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 1000;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&data->mutex);
		ret = regmap_read(data->regmap, MMC5633_REG_CTRL1, &reg);
		mutex_unlock(&data->mutex);
		if (ret < 0)
			return ret;

		i = (reg & MMC5633_CTRL1_BW_MASK) >> MMC5633_CTRL1_BW_SHIFT;
		if (i < 0 || i >= ARRAY_SIZE(mmc5633_samp_freq))
			return -EINVAL;

		*val = mmc5633_samp_freq[i].val;
		*val2 = mmc5633_samp_freq[i].val2;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int mmc5633_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int val,
			      int val2, long mask)
{
	struct mmc5633_data *data = iio_priv(indio_dev);
	int i, ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		/*TODO: The ODR register actually controls the frequency
		 * of continuous mode measurements. But below code is
		 * setting the BW bits which is not really correct */
		i = mmc5633_get_samp_freq_index(data, val, val2);
		if (i < 0)
			return -EINVAL;
		mutex_lock(&data->mutex);
		ret = regmap_update_bits(data->regmap, MMC5633_REG_CTRL1,
					 MMC5633_CTRL1_BW_MASK,
					 i << MMC5633_CTRL1_BW_SHIFT);
		mutex_unlock(&data->mutex);
		return ret;
	default:
		return -EINVAL;
	}
}

static const struct iio_info mmc5633_info = {
	.read_raw	= mmc5633_read_raw,
	.write_raw	= mmc5633_write_raw,
	.attrs		= &mmc5633_attribute_group,
};

static bool mmc5633_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MMC5633_REG_TPH0:
	case MMC5633_REG_TPH1:
	case MMC5633_REG_TU:
	case MMC5633_REG_ODR:
	case MMC5633_REG_CTRL0:
	case MMC5633_REG_CTRL1:
	case MMC5633_REG_CTRL2:
		return true;
	default:
		return false;
	}
}

static bool mmc5633_is_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MMC5633_REG_XOUT0:
	case MMC5633_REG_XOUT1:
	case MMC5633_REG_YOUT0:
	case MMC5633_REG_YOUT1:
	case MMC5633_REG_ZOUT0:
	case MMC5633_REG_ZOUT1:
	case MMC5633_REG_XOUT2:
	case MMC5633_REG_YOUT2:
	case MMC5633_REG_ZOUT2:
	case MMC5633_REG_TOUT:
	case MMC5633_REG_STATUS0:
	case MMC5633_REG_STATUS1:
	case MMC5633_REG_PROD_ID:
		return true;
	default:
		return false;
	}
}

static bool mmc5633_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MMC5633_REG_ODR:
	case MMC5633_REG_CTRL0:
	case MMC5633_REG_CTRL1:
	case MMC5633_REG_CTRL2:
		return false;
	default:
		return true;
	}
}

static struct reg_default mmc5633_reg_defaults[] = {
	{ MMC5633_REG_CTRL0,  0x00 },
	{ MMC5633_REG_CTRL1,  0x00 },
};

static const struct regmap_config mmc5633_regmap_config = {
	.name = MMC5633_REGMAP_NAME,

	.reg_bits = 8,
	.val_bits = 8,

	.max_register = MMC5633_REG_PROD_ID,
	.cache_type = REGCACHE_FLAT,

	.writeable_reg = mmc5633_is_writeable_reg,
	.readable_reg = mmc5633_is_readable_reg,
	.volatile_reg = mmc5633_is_volatile_reg,

	.reg_defaults = mmc5633_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(mmc5633_reg_defaults),
};

static int mmc5633_probe(struct i3c_device *client)
{
	struct mmc5633_data *data;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_i3c(client, &mmc5633_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "regmap initialization failed\n");
		return PTR_ERR(regmap);
	}

	data = iio_priv(indio_dev);
	dev_set_drvdata(&client->dev, indio_dev);

	data->client = client;
	data->regmap = regmap;
	data->res = MMC5633_BW_00;

	mutex_init(&data->mutex);

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &mmc5633_info;
	indio_dev->name = MMC5633_DRV_NAME;
	indio_dev->channels = mmc5633_channels;
	indio_dev->num_channels = ARRAY_SIZE(mmc5633_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = mmc5633_init(data);
	if (ret < 0) {
		dev_err(&client->dev, "mmc5633 chip init failed\n");
		return ret;
	}
	return devm_iio_device_register(&client->dev, indio_dev);
}

enum hwid{
	MMC5633_ID = 0x10,
};

static const struct i3c_device_id mmc5633_id[] = {
	I3C_DEVICE(0x0251, 0x00, (void *)MMC5633_ID),
	{},
};
MODULE_DEVICE_TABLE(i3c, mmc5633_id);

static struct i3c_driver mmc5633_driver = {
	.driver = {
		.name = MMC5633_DRV_NAME,
	},
	.probe		= mmc5633_probe,
	.id_table	= mmc5633_id,
};

module_i3c_driver(mmc5633_driver);

MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_DESCRIPTION("MEMSIC MMC5633 magnetic sensor driver");
MODULE_LICENSE("GPL v2");
