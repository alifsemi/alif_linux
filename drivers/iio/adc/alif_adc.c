// SPDX-License-Identifier: GPL-2.0+
/*
 *  alif_adc.c - Support for ADC12/ADC24
 *
 *  8-channels, 12-bit ADC and 4 differential channels, 24-bit ADC
 *
 *  Copyright (C) 2024 Pankaj Pandey <pankaj.pandey@alifsemi.com>
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/iio/sysfs.h>
#include "alif_temp.h"

extern int tempData[][2];
uint32_t channel_sel;

/* ADC_INTERRUPT bits */
#define AVG_SAMPLE_RDY	  (1 << 0)
#define ALL_SMAPLES_TAKEN (1 << 1)
#define AVG_CALC_THRESH_0 (1 << 2)
#define AVG_CALC_THRESH_1 (1 << 3)
#define ADC_INTERRUPT_FLAGS ((1<<0) | (1 << 1) | (1 << 2) | (1 << 3))

/* Read value setting  */
#define ADC_24BIT_READ (0xFFFFFF)
#define ADC_16BIT_READ (0xFFFF0)
#define ADC_12BIT_READ (0x3FF)
#define ADC_8BIT_READ (0xFF)

/* LSB is used to calculate analog voltage value
 * from the number of ADC sample value
 * Analog value = adc sample * vref / (2 ^ 24 - 1);
 * Analog value = adc sample * vref / (2 ^ 12 - 1);
 */
#define ADC_24BIT_LSB_DIV	(1 << 24)
#define ADC_12BIT_LSB_DIV	(1 << 12)

/*
 * ALIF registers definitions
 */
#define ALIFAD_SELECT(x)	((x) + 0x04)
#define ALIFAD_CTRL(x)	((x) + 0x08)
#define ALIFAD_VALUE(x)	((x) + 0x48)

/* Bit definitions for ALIFAD_SELECT: */
/* constant, always write this value! */
#define ALIFAD_REFm         0x00000200
/* constant, always write this value! */
#define ALIFAD_REFp		0x00000080
/* multiple of this is the channel number: 0, 1, 2 */
#define ALIFAD_IN		0x00000010
/* constant, always write this value! */
#define ALIFAD_INTERNAL	0x00000004

/* Bit definitions for ALIFAD_CTRL: */
#define ALIFAD_STROBE	0x00000002
#define ALIFAD_PDN_CTRL	0x00000004

/* Bit definitions for ALIFAD_VALUE: */
#define ALIFAD_VALUE_MASK	0x000003FF

#define ALIF_NAME "adc-dev"

#define CGU_BASE        0x1A602000
#define CGU_CLK_ENA     0x14

#define AON_BASE        0x1A604000

/* PMU_PERIPH offset */
#define PMU_PERIPH_OFFSET (0X40)

/* ANA Register */
#define ANA_BASE            0x1A60A000
#define ANA_VBAT_REG2       0x3C

/* Expansion Slave registers. */
#define EXPSLV_BASE     0x4902F000
#define EXPSLV_ADC_CTRL 0x30
#define EXPSLV_CMP_CTRL 0x38

/* CMP Register */
#define CMP0_BASE 0x49023000
#define CMP_COMP_REG2 0x4

/* Adc Vref setting */
#define ADC_VREF_BUF_RDIV_EN (0x0U << 16)
#define ADC_VREF_BUF_EN      (0x1U << 15)
#define ADC_VREF_CONT        (0x10U << 10)
#define ANA_PERIPH_LDO_CONT  (0xAU << 6)
#define ANA_PERIPH_BG_CONT   (0xAU << 1)


/* ADC register offsets */
#define ADC_START_SRC      (0x00)
#define ADC_COMP_THRESH_A  (0x04)
#define ADC_COMP_THRESH_B  (0x08)
#define ADC_CLK_DIVISOR    (0x0C)
#define ADC_INTERRUPT      (0x10)
#define ADC_INTERRUPT_MASK (0x14)
#define ADC_SAMPLE_WIDTH   (0x18)
#define ADC_AVG_NUM        (0x20)
#define ADC_SHIFT_CONTROL  (0x24)
#define ADC_CONTROL        (0x30)
#define ADC_SEQUENCER_CTRL (0x34)
#define ADC_REG1           (0x38)
#define ADC_SEL            (0x3C)
#define ADC_SAMPLE_REG_0   (0x50)

/****ADC Register macros****/
#define ADC_START_CONTINUOUS_CONV          (1U << 6)
#define ADC_START_ENABLE                   (1U << 7)
#define ADC_START_SINGLE_SHOT_CONV         (1U << 0)

/* Sample width */
#define ADC12_SAMPLE_WIDTH_Msk (0XFFFF)

#define SHIFT_DIR       (0 << 16)
#define SHIFT_CONTROL   (1 << 8)
#define COMP_THRESHOLD_A (1 << 0)
#define COMP_THRESHOLD_B (1 << 0)
#define COMP_EN (0 << 16)

/* Channel Numbers */
#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_1 1
#define ADC_CHANNEL_2 2
#define ADC_CHANNEL_3 3
#define ADC_CHANNEL_4 4
#define ADC_CHANNEL_5 5
#define ADC_CHANNEL_6 6

/****ADC MASK CHANNEL****/
#define ADC_UNMASK_CHANNEL_0                    (1 << 0)
#define ADC_UNMASK_CHANNEL_1                    (1 << 1)
#define ADC_UNMASK_CHANNEL_2                    (1 << 2)
#define ADC_UNMASK_CHANNEL_3                    (1 << 3)
#define ADC_UNMASK_CHANNEL_4                    (1 << 4)
#define ADC_UNMASK_CHANNEL_5                    (1 << 5)
#define ADC_UNMASK_CHANNEL_6                    (1 << 6)
#define ADC_UNMASK_CHANNEL_7                    (1 << 7)
#define ADC_UNMASK_CHANNEL_8                    (1 << 8)

#define ADC_COMPARATOR_THRESHOLD_ABOVE_A        (1 << 0)
#define ADC_COMPARATOR_THRESHOLD_ABOVE_B        (1 << 1)
#define ADC_COMPARATOR_THRESHOLD_BELOW_A        (1 << 2)
#define ADC_COMPARATOR_THRESHOLD_BELOW_B        (1 << 3)
#define ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B    (1 << 4)
#define ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B    (1 << 5)

#define TEMPERATURE_SENSOR                      ADC_CHANNEL_6
#define MAX_NUM_THRESHOLD                       (6)

/****Sequencer Macros****/
#define ADC_SEQUENCER_MSK_BIT (0x01)
#define ADC_MAX_INIT_CHANNEL  (0X100)
#define ADC_MSK_INIT_CHANNEL  (0X0F)
#define ADC_MSK_ALL_CHANNELS  (0X1FF)

/********Interrupt mask macro*******/
#define ADC_INTR_CMPA_POS            (2)
#define ADC_INTR_CMPA_MSK            (1 << ADC_INTR_CMPA_POS)
#define ADC_INTR_CMPB_POS            (3)
#define ADC_INTR_CMPB_MSK            (1 << ADC_INTR_CMPB_POS)
#define ADC_THRSHLD_CMP_MASK_BIT_POS (16)
#define ADC_THRSHLD_CMP_MASK_BIT     (0x03 << ADC_THRSHLD_CMP_MASK_BIT_POS)

/****Shift bit macro****/
#define ADC_SHIFT_BIT          (16)
#define ADC_SEQUENCER_INIT_Pos (12)

/****Comparator Macros****/
#define ADC_CMP_THRHLD_ABOVE_A     (0)
#define ADC_CMP_THRHLD_BELOW_A     (1)
#define ADC_CMP_THRHLD_BETWEEN_A_B (2)

#define ADC_CMP_THRHLD_ABOVE_B     (0)
#define ADC_CMP_THRHLD_BELOW_B     (1)
#define ADC_CMP_THRHLD_OUTSIDE_A_B (2)

/* ADC reg1 position macro */
#define ADC120_DIFFERENTIAL_EN_Pos (1)
#define ADC120_COMPARATOR_EN_Pos   (2)
#define ADC120_COMPARATOR_BIAS_Pos (3)
#define ADC120_VCM_DIV_Pos         (5)

#define ADC121_DIFFERENTIAL_EN_Pos (7)
#define ADC121_COMPARATOR_EN_Pos   (8)
#define ADC121_COMPARATOR_BIAS_Pos (9)
#define ADC121_VCM_DIV_Pos         (11)

#define ADC122_DIFFERENTIAL_EN_Pos (13)
#define ADC122_COMPARATOR_EN_Pos   (14)
#define ADC122_COMPARATOR_BIAS_Pos (15)
#define ADC122_VCM_DIV_Pos         (17)

/* PMU_PERIPH field definitions */
#define PMU_PERIPH_ADC1_PGA_EN           (1U << 0)
#define PMU_PERIPH_ADC1_PGA_GAIN_Pos     (1)
#define PMU_PERIPH_ADC1_PGA_GAIN_Msk     (0x7)
#define PMU_PERIPH_ADC2_PGA_EN           (1U << 4)
#define PMU_PERIPH_ADC2_PGA_GAIN_Pos     (5)
#define PMU_PERIPH_ADC2_PGA_GAIN_Msk     (0xE0)
#define PMU_PERIPH_ADC3_PGA_EN           (1U << 8)
#define PMU_PERIPH_ADC3_PGA_GAIN_Pos     (9)
#define PMU_PERIPH_ADC3_PGA_GAIN_Msk     (0xE00)
#define PMU_PERIPH_ADC24_EN              (1U << 12)
#define PMU_PERIPH_ADC24_OUTPUT_RATE_Pos (13)
#define PMU_PERIPH_ADC24_OUTPUT_RATE_Msk (0xE000)
#define PMU_PERIPH_ADC24_PGA_EN          (1U << 16)
#define PMU_PERIPH_ADC24_PGA_GAIN_Pos    (17)
#define PMU_PERIPH_ADC24_PGA_GAIN_Msk    (0xE0000)
#define PMU_PERIPH_ADC24_BIAS_Pos        (20)
#define PMU_PERIPH_ADC24_BIAS_Msk        (0x700000)

//1.76v change to 176mV
#define VREF_VOLT 176

//Program clk divisor from 2 to 16 on ADC_CLK_DIVISOR register
#define CLK_DIVISOR(x) (x < 16 ? x : 16)

//Program Sample number from 2 to 256 to ADC_AVG_NUM register
#define AVG_NUM(x) (x < 256 ? x : 256)

//Program Sample width from 2 to 32 to ADC_SAMPLE_WIDTH register
#define WIDTH_SAMPLE(x) (x < 32 ? x : 32)

static uint32_t val_diff, data_diff;

enum ADC_SCAN_MODE {
	ADC_SCAN_MODE_MULTI_CH,
	ADC_SCAN_MODE_SINGLE_CH
};

enum COMPARATOR_BIAS {
	COMPARATOR_BIAS_0_5,   //0.5 MS/s
	COMPARATOR_BIAS_1_0,   //1 MS/s
	COMPARATOR_BIAS_2_5,   //2.5 MS/s
	COMPARATOR_BIAS_5_0    //5 MS/s
};

enum ADC24_BIAS {
	ADC24_BIAS_5_0,   //5uA
	ADC24_BIAS_6_2,   //6.2uA
	ADC24_BIAS_7_5,   //7.5uA
	ADC24_BIAS_8_75   //8.75uA
};

enum ADC_PGA_GAIN {
	ADC_PGA_GAIN_0_DB,
	ADC_PGA_GAIN_6_DB,
	ADC_PGA_GAIN_12_DB,
	ADC_PGA_GAIN_18_DB,
	ADC_PGA_GAIN_24_DB,
	ADC_PGA_GAIN_30_DB,
	ADC_PGA_GAIN_36_DB,
	ADC_PGA_GAIN_42_DB,
};

enum ADC24_OUTPUT_RATE {
	ADC_OUTPUT_RATE_1K, //1KS/s
	ADC_OUTPUT_RATE_2K, //2KS/s
	ADC_OUTPUT_RATE_3K, //3KS/s
	ADC_OUTPUT_RATE_4K, //4KS/s
	ADC_OUTPUT_RATE_16K, //16KS/s
};

/**
 * enum ADC_CONV_STAT.
 * Status of an ongoing ADC conversion.
 */
enum ADC_CONV_STAT {
	ADC_CONV_STAT_CMP_THLD_ABOVE_A = (1U << 0),
	ADC_CONV_STAT_CMP_THLD_ABOVE_B = (1U << 1),
	ADC_CONV_STAT_CMP_THLD_BELOW_A = (1U << 2),
	ADC_CONV_STAT_CMP_THLD_BELOW_B = (1U << 3),
	ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B = (1U << 4),
	ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B = (1U << 5),
};

void __iomem *analog_base_adc120;

struct alif_adc_state {
	void __iomem *adc_base;
	void __iomem *aon_ctrl;
	struct clk *clk;
	struct completion completion;
	struct device dev;
	const char *name;
	u32 *buffer;
	u32 sample_width;
	u32 comparator;
	u32 instance;
	u32 value;
	u32 vref;
	u32 clk_div;
	u32 avg_sample;
	u32 width_sample;
	u8 differential;
	u8 comparator_en;
	u8 comparator_bias;
	u8 adc24_bias;
	u8 adc24_output_rate;
	u8 default_set;
};

/* ADC12/ADC24 instances */
enum ADC_INSTANCE {
	ADC_INSTANCE_ADC12_0,
	ADC_INSTANCE_ADC12_1,
	ADC_INSTANCE_ADC12_2,
	ADC_INSTANCE_ADC24_0,
};



#define ALIF_ADC_CHANNEL_BASE(_index, _chan_type)		\
		.type = (_chan_type),				\
		.indexed = 1,					\
		.channel = _index,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.address = ALIFAD_IN * _index,		\
		.scan_index = _index,

#define ALIF_ADC_CHANNEL(_index, _chan_type) {		\
		ALIF_ADC_CHANNEL_BASE(_index, _chan_type)	\
}

#define ALIF_ADC_TEMPERATURE_CHAN(_idx, _chan_type) {  \
		.type = (_chan_type),   \
		.channel = (_idx),              \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
		.scan_index = (_idx),                                   \
		.scan_type = {                                          \
				.sign = 'u',                            \
				.realbits = 12,                         \
				.storagebits = 16,                      \
},                                                      \
}

#define ALIF_ADC_CHANNEL_DIFF(chan1, chan2, si)             \
		{                                               \
	.type = IIO_VOLTAGE,                                    \
	.indexed = 1,                                           \
	.channel = (chan1),                                     \
	.channel2 = (chan2),                                    \
	.differential = 1,                                      \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),           \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),   \
	.scan_index = si,                                       \
	.scan_type = {                                          \
			.sign = 'u',                            \
			.realbits = 12,                         \
			.storagebits = 16,                      \
		},                                              \
		}

static const struct iio_chan_spec alif_adc_iio_channels[] = {
ALIF_ADC_CHANNEL(0, IIO_VOLTAGE),
ALIF_ADC_CHANNEL(1, IIO_VOLTAGE),
ALIF_ADC_CHANNEL(2, IIO_VOLTAGE),
ALIF_ADC_CHANNEL(3, IIO_VOLTAGE),
ALIF_ADC_CHANNEL(4, IIO_VOLTAGE),
ALIF_ADC_CHANNEL(5, IIO_VOLTAGE),
ALIF_ADC_TEMPERATURE_CHAN(6, IIO_TEMP),
ALIF_ADC_CHANNEL(7, IIO_VOLTAGE), };

static const struct iio_chan_spec adc24_channels[] = {
ALIF_ADC_CHANNEL_DIFF(0, 4, 8),
ALIF_ADC_CHANNEL_DIFF(1, 5, 9),
ALIF_ADC_CHANNEL_DIFF(2, 6, 10),
ALIF_ADC_CHANNEL_DIFF(3, 7, 11), };

static inline void adc_disable_single_shot_conv(struct alif_adc_state *st)
{
	uint32_t data;

	data = readl(st->adc_base + ADC_START_SRC);
	data &= ~(ADC_START_ENABLE);
	writel(data, st->adc_base + ADC_START_SRC);

	data = readl(st->adc_base + ADC_CONTROL);
	data &= ~ADC_START_SINGLE_SHOT_CONV;
	writel(data, st->adc_base + ADC_CONTROL);
}

static inline void adc_sequencer_msk_ch_control(struct alif_adc_state *st,
		int channel) {
	uint32_t val;

	val = readl(st->adc_base + ADC_SEQUENCER_CTRL);
	val &= ~(ADC_MSK_ALL_CHANNELS);
	val |= ((~(1 << channel)) & ADC_MSK_ALL_CHANNELS);
	writel(val, st->adc_base + ADC_SEQUENCER_CTRL);
}

static inline void adc_enable_single_shot_conv(struct alif_adc_state *st)
{
	uint32_t val;

	val = readl(st->adc_base + ADC_START_SRC);
	val |= ADC_START_ENABLE;
	writel(val, st->adc_base + ADC_START_SRC);
	val = readl(st->adc_base + ADC_START_SRC);

	val = readl(st->adc_base + ADC_CONTROL);
	val |= ADC_START_SINGLE_SHOT_CONV;
	writel(val, st->adc_base + ADC_CONTROL);
	val = readl(st->adc_base + ADC_CONTROL);
}

static inline void adc_set_ch_scan_mode(struct alif_adc_state *st,
		int channel_scan_mode, int channel) {
	uint32_t val;

	val = readl(st->adc_base + ADC_SEQUENCER_CTRL);
	val = (channel_scan_mode << 0) | (channel << ADC_SEQUENCER_INIT_Pos);
	writel(val, st->adc_base + ADC_SEQUENCER_CTRL);
	val = readl(st->adc_base + ADC_SEQUENCER_CTRL);
}

static inline void disable_adc(struct alif_adc_state *st)
{
	uint32_t data;

	data = readl(st->adc_base + ADC_START_SRC);
	data &= ~(ADC_START_ENABLE);
	writel(data, st->adc_base + ADC_START_SRC);

	data = readl(st->adc_base + ADC_CONTROL);
	data &= ~ADC_START_SINGLE_SHOT_CONV;
	writel(data, st->adc_base + ADC_CONTROL);
}

static inline void adc_set_diff_and_comp(struct alif_adc_state *st,
		u32 inst, u8 differential, u8 comparator_en,
		u8 comparator_bias)
{

	switch (inst) {
	case ADC_INSTANCE_ADC12_0:
		val_diff |= ((differential << ADC120_DIFFERENTIAL_EN_Pos));
		val_diff |= ((1 << ADC120_VCM_DIV_Pos)
			    | (comparator_en << ADC120_COMPARATOR_EN_Pos)
			    | (comparator_bias << ADC120_COMPARATOR_BIAS_Pos));
		break;

	case ADC_INSTANCE_ADC12_1:
		val_diff |= ((differential << ADC121_DIFFERENTIAL_EN_Pos));
		val_diff |= ((1 << ADC121_VCM_DIV_Pos)
			    | (comparator_en << ADC121_COMPARATOR_EN_Pos)
			    | (comparator_bias << ADC121_COMPARATOR_BIAS_Pos));
		break;

	case ADC_INSTANCE_ADC12_2:
		val_diff |= ((differential << ADC122_DIFFERENTIAL_EN_Pos));
		val_diff |= ((1 << ADC122_VCM_DIV_Pos)
			    | (comparator_en << ADC122_COMPARATOR_EN_Pos)
			    | (comparator_bias << ADC122_COMPARATOR_BIAS_Pos));
		break;

	default:
		pr_err("ADC instances not found for %s\n", __func__);
	}

	data_diff = readl(analog_base_adc120);
	writel(val_diff, analog_base_adc120);
}

int get_temp(int *adc_value)
{
	uint32_t i;

	/* check for temperature operating range */
	if ((*adc_value < tempData[0][0])
			|| (*adc_value > tempData[MAX_TEMP_RANGE][0])) {
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(tempData); i++) {
		/* check if value matches with tempdata */
		if (*adc_value == tempData[i][0])
			break;
	}

	return (tempData[i][1]);
}

static int alif_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct alif_adc_state *st = iio_priv(indio_dev);
	int ret, temp, buff;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&indio_dev->mlock);
		ret = clk_prepare_enable(st->clk);
		if (ret) {
			mutex_unlock(&indio_dev->mlock);
			return ret;
		}

		adc_set_ch_scan_mode(st, ADC_SCAN_MODE_SINGLE_CH,
					chan->channel);
		adc_sequencer_msk_ch_control(st, chan->channel);
		channel_sel = ((readl(st->adc_base + ADC_SEQUENCER_CTRL)
				& 0xF000) >> 12);
		/* check channel number passed is enabled or not */
		if (((readl(st->adc_base + ADC_SEQUENCER_CTRL))
			& (1 << channel_sel))) {
			pr_err(" Error: channel is masked\n");
			return -EINVAL;
		}
		adc_enable_single_shot_conv(st);
		wait_for_completion(&st->completion); /* set by ISR */
		clk_disable_unprepare(st->clk);

		switch (chan->type) {
		case IIO_VOLTAGE:
			if (st->instance == ADC_INSTANCE_ADC24_0)
				*val = (st->value & ADC_24BIT_READ);
			else
				*val = (st->value & ADC_16BIT_READ) >> 8;

			break;

		case IIO_TEMP:
			*val = (st->value & ADC_16BIT_READ) >> 8;
			break;

		default:
			mutex_unlock(&indio_dev->mlock);
			return -EINVAL;
		}
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
	/* we are using below mentioned formula to calculate voltage
	 * fractional value in volt:-
	 * Analog Voltage =  (ADC Sample Value(24bit or 12bit) * Vref) /
	 *		     (2^12 or 2^24  - 1)
	 *
	 * --> Reference Voltage (Vref):This is the maximum input voltage
	 * that the ADC can read.
	 * For example, if Vref is 1.76V, the analog input range is 0
	 * to 1.76V.
	 * --> 2^12 or 2^24: Powerof 2 it is decided based upon according
	 * to 12-bit or 24-bit ADC.
	 */
		switch (chan->type) {
		case IIO_VOLTAGE:
			if (st->instance == ADC_INSTANCE_ADC24_0) {
				*val = (st->value & ADC_24BIT_READ);
				*val = (*val * st->vref) / 100;
				*val2 = (ADC_24BIT_LSB_DIV - 1);
			} else {
				*val = (st->value & ADC_16BIT_READ) >> 8;
				*val = (*val * st->vref) / 100;
				*val2 = (ADC_12BIT_LSB_DIV - 1);
			}

			return IIO_VAL_FRACTIONAL;

		case IIO_TEMP:
			*val = (st->value & ADC_16BIT_READ) >> 8;
			buff = *val;
			temp = get_temp(&buff);
			pr_info("Temperature: %d.%d°C\n", temp / 10,
				abs(temp % 10));
			if (temp < 0)
				return (~temp + 1);
			else
				return temp;

		default:
			return -EINVAL;
		}
	}
	return -EINVAL;
}

static ssize_t adc_differential_var_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct alif_adc_state *st = iio_priv(dev_to_iio_dev(dev));
	int ret;

	ret = sscanf(buf, "%du", (int *)&st->differential);
	if (ret != 1)
		return -EINVAL;

	if (st->differential == 1) {
		adc_set_diff_and_comp(st, st->instance, st->differential,
					st->comparator_en, st->comparator_bias);
	}
	return count;
}

static ssize_t adc_differential_var_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct alif_adc_state *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->differential);
}

static IIO_DEVICE_ATTR(differential_var, 0644,
			adc_differential_var_show,
			adc_differential_var_store, 0);

static struct attribute *alif_attributes[] = {
		&iio_dev_attr_differential_var.dev_attr.attr, NULL, };

static const struct attribute_group alif_attribute_group = { .attrs =
		alif_attributes, };

static const struct iio_info alif_adc_iio_info = { .read_raw =
		&alif_read_raw, .attrs = &alif_attribute_group, };

static inline void read_adc_data(struct alif_adc_state *st)
{

	uint32_t value;
	void __iomem *channel_sample_reg;
	void __iomem *sample_reg = (st->adc_base + ADC_SAMPLE_REG_0);
	uint32_t channel_num = readl(st->adc_base + ADC_SEL);

	channel_sample_reg = (sample_reg + sizeof(uint32_t) * channel_num);
	st->value = (readl((u32 *)channel_sample_reg));
	value = readl(st->adc_base + ADC_CONTROL);
	value &= ADC_THRSHLD_CMP_MASK_BIT;
	value >>= ADC_SHIFT_BIT;

	switch (value) {
	case ADC_CMP_THRHLD_ABOVE_A:
		st->comparator = ADC_CONV_STAT_CMP_THLD_ABOVE_A;
		break;
	case ADC_CMP_THRHLD_BELOW_A:
		st->comparator = ADC_CONV_STAT_CMP_THLD_BELOW_A;
		break;
	case ADC_CMP_THRHLD_BETWEEN_A_B:
		st->comparator = ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B;
		break;
	default:
		pr_err("ADC CMP Theshold instances not found\n");
		return;
	}
}

static irqreturn_t alif_adc_isr(int irq, void *dev_id)
{
	uint32_t status;
	struct alif_adc_state *st = dev_id;

	status = readl(st->adc_base + ADC_INTERRUPT);

	if ((status & ADC_INTERRUPT_FLAGS) == ((AVG_SAMPLE_RDY) |
		(ALL_SMAPLES_TAKEN) | (AVG_CALC_THRESH_0) |
		(AVG_CALC_THRESH_1))) {
		iowrite32(status, st->adc_base + ADC_INTERRUPT);
		read_adc_data(st);
	} else {
		return IRQ_NONE;
	}

	complete(&st->completion);
	disable_adc(st);

	return IRQ_HANDLED;
}

static inline void adc_unmask_interrupt(struct alif_adc_state *st)
{
	uint32_t val;

	val = readl(st->adc_base + ADC_INTERRUPT_MASK);
	val = 0;
	writel(val, st->adc_base + ADC_INTERRUPT_MASK);
}

static inline void adc_set_comparator_ctrl_bit(struct alif_adc_state *st)
{
	uint32_t val;

	val = readl(st->adc_base + ADC_CONTROL);
	val = COMP_EN;
	writel(val, st->adc_base + ADC_CONTROL);
}
static inline void adc_set_comparator_b(struct alif_adc_state *st)
{
	uint32_t val;

	val = readl(st->adc_base + ADC_COMP_THRESH_B);
	val = COMP_THRESHOLD_B;
	writel(val, st->adc_base + ADC_COMP_THRESH_B);
}

static inline void adc_set_comparator_a(struct alif_adc_state *st)
{
	uint32_t val;

	val = readl(st->adc_base + ADC_COMP_THRESH_A);
	val = COMP_THRESHOLD_A;
	writel(val, st->adc_base + ADC_COMP_THRESH_A);
}

static inline void adc_set_n_shift_bit(struct alif_adc_state *st)
{
	uint32_t val;

	val = readl(st->adc_base + ADC_SHIFT_CONTROL);
	val = (SHIFT_DIR) | (SHIFT_CONTROL);
	writel(val, st->adc_base + ADC_SHIFT_CONTROL);
}

static inline void adc_set_clock_divsor(struct alif_adc_state *st)
{
	writel(st->clk_div, st->adc_base + ADC_CLK_DIVISOR);
}

static inline void adc_set_avg_sample(struct alif_adc_state *st)
{
	/* set adc avg. num 256 for ADC12/ADC24 */
	writel(st->avg_sample, st->adc_base + ADC_AVG_NUM);
}

static inline void adc_set_sample_width(struct alif_adc_state *st)
{
	/* set Sample width value 16 for ADC12/ADC24 */
	uint32_t val;

	val = readl(st->adc_base + ADC_SAMPLE_WIDTH);
	val = (val & ~ADC12_SAMPLE_WIDTH_Msk) | st->width_sample;
	writel(val, st->adc_base + ADC_SAMPLE_WIDTH);
}

/* CGU Enable 160 Mhz clock */
static inline void adc_clk_config(void)
{
	uint32_t val;
	void __iomem *va_base;

	va_base = ioremap(CGU_BASE, SZ_64);
	if (!va_base)
		pr_err("Failed to map CGU_BASE\n");

	val = ioread32(va_base + CGU_CLK_ENA);
	val |= BIT(20);
	writel(val, va_base + CGU_CLK_ENA);
	iounmap(va_base);

	/* Enable LDO and BG for the ANALOG */
	va_base = ioremap(ANA_BASE, SZ_64);
	if (!va_base)
		pr_err("Failed to map CGU_BASE\n");

	val = ioread32(va_base + ANA_VBAT_REG2);
	val |= (BIT(22) | BIT(23));
	writel(val, va_base + ANA_VBAT_REG2);
	iounmap(va_base);

	/* Enable ADC Clock Control */
	va_base = ioremap(EXPSLV_BASE, SZ_64);
	if (!va_base)
		pr_err("Failed to map CGU_BASE\n");

	val = ioread32(va_base + EXPSLV_ADC_CTRL);
	val |= (BIT(0) | BIT(4) | BIT(8) | BIT(12));
	writel(val, va_base + EXPSLV_ADC_CTRL);

	/* Enable COMP Clock Control */
	val = ioread32(va_base + EXPSLV_CMP_CTRL);
	val |= (BIT(0) | BIT(4));
	writel(val, va_base + EXPSLV_CMP_CTRL);
	iounmap(va_base);

}

/* Vref setting */
static inline void adc_analog_config(void)
{
	uint32_t val;
	uint32_t cmp_reg2_val;
	void __iomem *va_base;
	/* Adc Vref setting */
	va_base = ioremap(CMP0_BASE, SZ_16);
	if (!va_base)
		pr_err("Failed to map CMP0_BASE\n");

	val = ioread32(va_base + CMP_COMP_REG2);
	cmp_reg2_val = (ADC_VREF_BUF_RDIV_EN | ADC_VREF_BUF_EN | ADC_VREF_CONT |
	ANA_PERIPH_LDO_CONT | ANA_PERIPH_BG_CONT);
	val |= cmp_reg2_val;
	writel(val, va_base + CMP_COMP_REG2);
	iounmap(va_base);
}

static inline void enable_adc24(struct alif_adc_state *st)
{
	uint32_t data = 0;

	data = readl(st->aon_ctrl + PMU_PERIPH_OFFSET);
	data |= PMU_PERIPH_ADC24_EN;
	writel(data, (st->aon_ctrl + PMU_PERIPH_OFFSET));
}

static inline void set_adc24_bias(struct alif_adc_state *st, uint32_t bias)
{
	uint32_t data;

	data = readl(st->aon_ctrl + PMU_PERIPH_OFFSET);
	data |= ((bias << PMU_PERIPH_ADC24_BIAS_Pos) &
		PMU_PERIPH_ADC24_BIAS_Msk);
	writel(data, st->aon_ctrl + PMU_PERIPH_OFFSET);
}

static inline void set_adc24_output_rate(struct alif_adc_state *st,
		uint32_t rate)
{
	uint32_t data;

	data = readl(st->aon_ctrl + PMU_PERIPH_OFFSET);
	data |= ((rate << PMU_PERIPH_ADC24_OUTPUT_RATE_Pos)
			& PMU_PERIPH_ADC24_OUTPUT_RATE_Msk);
	writel(data, st->aon_ctrl + PMU_PERIPH_OFFSET);
}

static int alif_adc_probe(struct platform_device *pdev)
{
	struct alif_adc_state *st = NULL;
	struct resource *res;
	int retval = -ENODEV;
	struct iio_dev *iodev = NULL;
	int irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform I/O memory\n");
		return -ENXIO;
	}

	iodev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!iodev)
		return -ENOMEM;

	st = iio_priv(iodev);

	st->adc_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!st->adc_base) {
		dev_err(&pdev->dev, "failed mapping memory\n");
		return -EBUSY;
	}

	st->aon_ctrl = ioremap(AON_BASE, SZ_128);

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENXIO;


	platform_set_drvdata(pdev, iodev);

	init_completion(&st->completion);

	st->differential = 0;
	iodev->name = dev_name(&pdev->dev);
	iodev->dev.parent = &pdev->dev;
	iodev->info = &alif_adc_iio_info;
	iodev->modes = INDIO_DIRECT_MODE;
	iodev->channels = alif_adc_iio_channels;
	iodev->num_channels = ARRAY_SIZE(alif_adc_iio_channels);
	st->name = iodev->name;
	st->vref = VREF_VOLT;
	st->clk_div = CLK_DIVISOR(2);
	st->avg_sample = AVG_NUM(256);
	st->width_sample = WIDTH_SAMPLE(16);

	/* CGU Enable 160 Mhz clock */
	adc_clk_config();

	/* Vref Setting */
	adc_analog_config();

	/* set sample width value ADC12/24 */
	adc_set_sample_width(st);

	/* set the clock divisor */
	adc_set_avg_sample(st);

	/* set clock divisor for ADC121 */
	adc_set_clock_divsor(st);

	/* set adc set and shift bit ADC120 */
	adc_set_n_shift_bit(st);

	/* set comparator_a threshold */
	adc_set_comparator_a(st);

	/* set comparator_b threshold */
	adc_set_comparator_b(st);

	/* set comparator ctrl bit for ADC120 */
	adc_set_comparator_ctrl_bit(st);

	/* disabling the ADC_INTERRUPT_MASK ADC12/ADC24 */
	adc_unmask_interrupt(st);

	/* Checking instances ADC120, ADC121 and ADC122 */
	if (!(strcmp(st->name, "49020000.adc12_0"))) {
		st->instance = ADC_INSTANCE_ADC12_0;
		st->comparator_en = 1;
		st->comparator_bias = COMPARATOR_BIAS_2_5;
		st->default_set = 1;
		analog_base_adc120 = (st->adc_base) + ADC_REG1;
	} else if (!(strcmp(st->name, "49021000.adc12_1"))) {
		st->instance = ADC_INSTANCE_ADC12_1;
		st->comparator_en = 1;
		st->comparator_bias = COMPARATOR_BIAS_2_5;
		st->default_set = 1;
	} else if (!(strcmp(st->name, "49022000.adc12_2"))) {
		st->instance = ADC_INSTANCE_ADC12_2;
		st->comparator_en = 1;
		st->comparator_bias = COMPARATOR_BIAS_2_5;
		st->default_set = 1;
	} else if (!(strcmp(st->name, "49027000.adc24"))) {
		st->comparator_en = 1;
		st->adc24_bias = ADC24_BIAS_8_75;
		st->default_set = 1;
		st->adc24_output_rate = ADC_OUTPUT_RATE_1K;
		/* enable adc24 from control register */
		enable_adc24(st);
		/* set output rate from control register */
		set_adc24_output_rate(st, st->adc24_output_rate);
		/* Set adc24 bias from control register */
		set_adc24_bias(st, st->adc24_bias);
		iodev->channels = adc24_channels;
		iodev->num_channels = ARRAY_SIZE(adc24_channels);
		st->instance = ADC_INSTANCE_ADC24_0;
	}
	/* set differential control for ADC12 */
	if (st->instance != ADC_INSTANCE_ADC24_0) {
		adc_set_diff_and_comp(st, st->instance, st->differential,
					st->comparator_en, st->comparator_bias);
	}

	retval = devm_request_irq(&pdev->dev, irq, alif_adc_isr, 0,
			ALIF_NAME, st);

	if (retval < 0) {
		dev_err(&pdev->dev, "failed requesting interrupt\n");
		return retval;
	}

	retval = devm_iio_device_register(&pdev->dev, iodev);
	if (retval)
		return retval;

	dev_info(&pdev->dev, "ALIF ADC driver loaded, IRQ %d\n", irq);

	return 0;
}

static int alif_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct alif_adc_state *adc_priv = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	adc_disable_single_shot_conv(adc_priv);
	clk_disable_unprepare(adc_priv->clk);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id alif_adc_match[] = {
		{ .compatible = "alif,adc-dev" },
		{},
};
MODULE_DEVICE_TABLE(of, alif_adc_match);
#endif

static struct platform_driver alif_adc_driver = {
		.probe  = alif_adc_probe,
		.remove = alif_adc_remove,
		.driver = {
			.name = ALIF_NAME,
			.of_match_table = of_match_ptr(alif_adc_match),
		},
};

module_platform_driver(alif_adc_driver);

MODULE_AUTHOR("Pankaj Pandey <pankaj.pandey@alifsemi.com>");
MODULE_DESCRIPTION("ALIF ADC12/ADC24 driver");
MODULE_LICENSE("GPL");
