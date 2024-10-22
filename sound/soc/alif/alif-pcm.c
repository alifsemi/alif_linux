// SPDX-License-Identifier: GPL-2.0
/*
 * PDM Driver for Alif PDM module
 * Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/lcm.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-component.h>
#include <sound/simple_card.h>
#include <sound/pcm.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/err.h>
#include <sound/pcm_params.h>
#include <linux/jiffies.h>
#include "fir_coefficient_defines.h"

#define MODE_FREQ_MODE_0 1
#define MODE_FREQ_8K 8000
#define MODE_FREQ_16K 16000
#define MODE_FREQ_32K 32000
#define MODE_FREQ_48K 48000
#define MODE_FREQ_96K 96000
#define MODE_FREQ_192K 192000
#define OWN_TIME 100

#define PDM_CTL0_REG				0x0000
#define PDM_CTL1_REG				0x0004
#define PDM_FIFO_WATERMARK_H_REG		0x0008
#define PDM_FIFO_STAT_REG			0x000C
#define PDM_IRQ_ENABLE_REG			0x001C
#define PDM_WARNING_IRQ_REG			0x0014
#define PDM_ERROR_IRQ_REG			0x0010
#define PDM_AUDIO_DTCT_IRQ_REG			0x0018
#define PDM_AUDIOOUT_CH0_CH1_REG		0x0020
#define PDM_AUDIOOUT_CH2_CH3_REG		0x0024
#define PDM_AUDIOOUT_CH4_CH5_REG		0x0028
#define PDM_AUDIOOUT_CH6_CH7_REG		0x002C
#define FIFO_FULL_IRQ_EN			0x1
#define FIFO_OVERFLOW_IRQ_EN			0x2
#define ALL_CH_AUDIO_DETECT_IRQ_EN		0xFF00
#define BITS_PER_SAMPLE				(sizeof(unsigned char) * 2 * 8)
#define ODD_CHANNEL_SAMPLE_MASK			(0x0000FFFF)
#define EVEN_CHANNEL_SAMPLE_MASK			(0xFFFF0000)

#define BYPASS_IIR_FILTER       2
#define CONFIG_BITS             16

#define ALIF_PCM_RATES          SNDRV_PCM_RATE_8000_192000
#define ALIF_PCM_FORMATS        (SNDRV_PCM_FMTBIT_S16_LE)

#define MAX_CHANNELS            8

#define MIN_PERIODS             (4)
#define MAX_PERIODS             (MAX_BUFFER_BYTES / MIN_PERIOD_BYTES)
#define MIN_PERIOD_BYTES	(6400)
#define MAX_BUFFER_BYTES	(2 * MIN_PERIOD_BYTES * MIN_PERIODS * 10)
#define MAX_PERIOD_BYTES	(MAX_BUFFER_BYTES / MIN_PERIODS)

static ssize_t  modefreq_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t  modefreq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t  channelsel_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t  channelsel_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR_RW(modefreq);
static DEVICE_ATTR_RW(channelsel);

struct alif_pcm_dev {
	struct device	*dev;
	void __iomem	*mem;
	struct clk      *pclk;
	unsigned int pdm_mode;
	unsigned char channel;

	/*counts frame*/
	snd_pcm_uframes_t pdm_buffer_ptr;

	/* used to iterate through each channel data*/
	snd_pcm_uframes_t pdm_buffer_index;

	struct snd_pcm_substream __rcu *pdm_substream;
};

static void pcm_setup(struct alif_pcm_dev *dev);
static int pcm_dai_probe(struct snd_soc_dai *dai);
static int alif_pcm_open(struct snd_pcm_substream *ss);

static irqreturn_t alif_pcm_interrupt(int irq, void *dev_id)
{
	struct alif_pcm_dev *dev = dev_id;
	struct snd_pcm_substream *substreamp;
	unsigned int fifo_count;
	unsigned int pcm_warning;
	unsigned int result1;
	unsigned int result2;
	unsigned int channel_config, audio_ch01,
			audio_ch23, audio_ch45, audio_ch67;
	snd_pcm_uframes_t period_size = 4;
	unsigned int period_pos;
	snd_pcm_uframes_t rt_buffer_size_bytes = 0;
	snd_pcm_uframes_t pcm_buffer_tail = dev->pdm_buffer_ptr;
	unsigned short *circular_buffer = NULL;

	pcm_warning = readl_relaxed(dev->mem + PDM_WARNING_IRQ_REG);
	fifo_count = readl_relaxed(dev->mem + PDM_FIFO_STAT_REG);
	channel_config = readl_relaxed(dev->mem + PDM_CTL0_REG);
	rcu_read_lock();
	substreamp = rcu_dereference(dev->pdm_substream);
	period_size = substreamp->runtime->period_size;
	rt_buffer_size_bytes = substreamp->runtime->buffer_size *
				substreamp->runtime->channels *
				(BITS_PER_SAMPLE / 8);
	circular_buffer = (unsigned short *)(substreamp->runtime->dma_area);

	if (pcm_warning & BIT(0)) {
		while (fifo_count) {
			audio_ch01 = readl_relaxed(dev->mem +
					PDM_AUDIOOUT_CH0_CH1_REG);
			audio_ch23 = readl_relaxed(dev->mem +
					PDM_AUDIOOUT_CH2_CH3_REG);
			audio_ch45 = readl_relaxed(dev->mem +
					PDM_AUDIOOUT_CH4_CH5_REG);
			audio_ch67 = readl_relaxed(dev->mem +
					PDM_AUDIOOUT_CH6_CH7_REG);

			if (channel_config & (1 << 0)) {
				result1 =  (audio_ch01 &
						ODD_CHANNEL_SAMPLE_MASK);
				circular_buffer[dev->pdm_buffer_index++] =
					(uint16_t)(result1);
				dev->pdm_buffer_index =
					(dev->pdm_buffer_index <=
					 (rt_buffer_size_bytes / 2)) ?
					dev->pdm_buffer_index : 0;
			}

			if (channel_config & (1 << 1)) {
				result2 =
					((audio_ch01 &
					  EVEN_CHANNEL_SAMPLE_MASK) >> 16);
				circular_buffer[dev->pdm_buffer_index++] =
					(uint16_t)(result2);
				dev->pdm_buffer_index =
					(dev->pdm_buffer_index <=
					 (rt_buffer_size_bytes / 2)) ?
					dev->pdm_buffer_index : 0;
			}

			if (channel_config & (1 << 2)) {
				result1 =  (audio_ch23 &
						ODD_CHANNEL_SAMPLE_MASK);
				circular_buffer[dev->pdm_buffer_index++] =
					(uint16_t)(result1);
				dev->pdm_buffer_index =
					(dev->pdm_buffer_index <=
					 (rt_buffer_size_bytes / 2)) ?
					dev->pdm_buffer_index : 0;
			}

			if (channel_config & (1 << 3)) {
				result2 =
					((audio_ch23 &
					  EVEN_CHANNEL_SAMPLE_MASK) >> 16);
				circular_buffer[dev->pdm_buffer_index++] =
					(uint16_t)(result2);
				dev->pdm_buffer_index =
					(dev->pdm_buffer_index <=
					 (rt_buffer_size_bytes / 2)) ?
					dev->pdm_buffer_index : 0;
			}

			if (channel_config & (1 << 4)) {
				result1 =  (audio_ch45 &
						ODD_CHANNEL_SAMPLE_MASK);
				circular_buffer[dev->pdm_buffer_index++] =
					(uint16_t)(result1);
				dev->pdm_buffer_index =
					(dev->pdm_buffer_index <=
					 (rt_buffer_size_bytes / 2)) ?
					dev->pdm_buffer_index : 0;
			}

			if (channel_config & (1 << 5)) {
				result2 =
					((audio_ch45 &
					  EVEN_CHANNEL_SAMPLE_MASK) >> 16);
				circular_buffer[dev->pdm_buffer_index++] =
					(uint16_t)(result2);
				dev->pdm_buffer_index =
					(dev->pdm_buffer_index <=
					 (rt_buffer_size_bytes / 2)) ?
					dev->pdm_buffer_index : 0;
			}

			if (channel_config & (1 << 6)) {
				result1 =  (audio_ch67 &
						ODD_CHANNEL_SAMPLE_MASK);
				circular_buffer[dev->pdm_buffer_index++] =
					(uint16_t)(result1);
				dev->pdm_buffer_index =
					(dev->pdm_buffer_index <=
					 (rt_buffer_size_bytes / 2)) ?
					dev->pdm_buffer_index : 0;
			}

			if (channel_config & (1 << 7)) {
				result2 =
					((audio_ch67 &
					  EVEN_CHANNEL_SAMPLE_MASK) >> 16);
				circular_buffer[dev->pdm_buffer_index++] =
					(uint16_t)(result2);
				dev->pdm_buffer_index =
					(dev->pdm_buffer_index <=
					 (rt_buffer_size_bytes / 2)) ?
					dev->pdm_buffer_index : 0;
			}

			fifo_count--;

		}
		pcm_buffer_tail = bytes_to_frames(substreamp->runtime,
						dev->pdm_buffer_index * 2);
		if (pcm_buffer_tail > bytes_to_frames(substreamp->runtime,
						rt_buffer_size_bytes)) {
			pcm_buffer_tail = pcm_buffer_tail %
				bytes_to_frames(substreamp->runtime,
						rt_buffer_size_bytes);
		}
		dev->pdm_buffer_ptr = pcm_buffer_tail;
		period_pos = pcm_buffer_tail % period_size;
		period_pos += MAX_CHANNELS;
		if (period_pos >= period_size)
			snd_pcm_period_elapsed(substreamp);
		(void)readl_relaxed(dev->mem + PDM_WARNING_IRQ_REG);
		(void)readl_relaxed(dev->mem + PDM_ERROR_IRQ_REG);
	}
	rcu_read_unlock();
	return IRQ_HANDLED;
}

static ssize_t modefreq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct alif_pcm_dev *alif_dev = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", alif_dev->pdm_mode);
}

static ssize_t modefreq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct alif_pcm_dev *alif_dev = dev_get_drvdata(dev);

	ret = sscanf(buf, "%d\n", &(alif_dev->pdm_mode));

	if (ret != 1) {
		pr_info("Failed ! Expected an integer pdm_mode\n");
		return -EINVAL;
	}
	return count;
}


static ssize_t channelsel_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct alif_pcm_dev *alif_dev = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", alif_dev->channel);

}

static ssize_t channelsel_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct alif_pcm_dev *alif_dev = dev_get_drvdata(dev);
	unsigned int channel_map = 0;

	ret = sscanf(buf, "%d\n", &(channel_map));
	if (ret != 1) {
		pr_info("Failed! Expected one values\n");
		return -EINVAL;

	}
	alif_dev->channel = channel_map & 0xff;

	return count;
}

static int alif_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct alif_pcm_dev *dev = dev_get_drvdata(dai->dev);
	unsigned int rate;
	int ret = 0;
	unsigned int config_pdm;

	dev->pdm_buffer_index = 0;
	rate = params_rate(params);

	writel_relaxed(0x1 << BYPASS_IIR_FILTER, dev->mem + PDM_CTL1_REG);
	writel_relaxed(0x1, dev->mem + PDM_FIFO_WATERMARK_H_REG);

	switch (rate) {
	case MODE_FREQ_MODE_0:
	{
		writel_relaxed(0x0 << CONFIG_BITS,
				(dev->mem + PDM_CTL0_REG));
		config_pdm = readl_relaxed(dev->mem +
				PDM_CTL0_REG);
		break;
	}
	case MODE_FREQ_8K:
	{

		writel_relaxed(0x1 << CONFIG_BITS,
				(dev->mem + PDM_CTL0_REG));
		config_pdm = readl_relaxed(dev->mem +
				PDM_CTL0_REG);
		dev->pdm_mode = 1;

		break;
	}
	case MODE_FREQ_16K:
	{
		switch (dev->pdm_mode) {
		case 2:
		{
			writel_relaxed(0x2 << CONFIG_BITS,
					dev->mem + PDM_CTL0_REG);
			config_pdm = readl_relaxed(dev->mem +
					PDM_CTL0_REG);
			break;
		}
		case 3:
		{
			writel_relaxed(0x3 << CONFIG_BITS,
					dev->mem + PDM_CTL0_REG);
			config_pdm = readl_relaxed(dev->mem +
					PDM_CTL0_REG);
			break;
		}
		case 4:
		{
			writel_relaxed(0x4 << CONFIG_BITS,
					dev->mem + PDM_CTL0_REG);
			config_pdm = readl_relaxed(dev->mem +
					PDM_CTL0_REG);
			break;
		}
		default:
		{
			writel_relaxed(0x2 << CONFIG_BITS,
					dev->mem + PDM_CTL0_REG);
			config_pdm = readl_relaxed(dev->mem +
					PDM_CTL0_REG);
			dev->pdm_mode = 2;
			break;
		}
		}
		break;
	}
	case MODE_FREQ_32K:
	{

		writel_relaxed(0x5 << CONFIG_BITS,
				dev->mem + PDM_CTL0_REG);
		config_pdm = readl_relaxed(dev->mem +
				PDM_CTL0_REG);
		dev->pdm_mode = 5;
		break;
	}
	case MODE_FREQ_48K:
	{
		switch (dev->pdm_mode) {
		case 6:
		{
			writel_relaxed(0x6 << CONFIG_BITS,
					dev->mem + PDM_CTL0_REG);
			config_pdm = readl_relaxed(dev->mem +
					PDM_CTL0_REG);
			break;
		}
		case 7:
		{
			writel_relaxed(0x7 << CONFIG_BITS,
					dev->mem + PDM_CTL0_REG);
			config_pdm = readl_relaxed(dev->mem +
					PDM_CTL0_REG);
			break;
		}
		default:
		{
			writel_relaxed(0x6 << CONFIG_BITS,
					dev->mem + PDM_CTL0_REG);
			config_pdm = readl_relaxed(dev->mem +
					PDM_CTL0_REG);
			dev->pdm_mode = 6;
			break;
		}
		}
		break;
	}
	case MODE_FREQ_96K:
	{
		writel_relaxed(0x8 << CONFIG_BITS,
				dev->mem + PDM_CTL0_REG);
		config_pdm = readl_relaxed(dev->mem +
				PDM_CTL0_REG);
		dev->pdm_mode = 8;
		break;
	}
	case MODE_FREQ_192K:
	{
		writel_relaxed(0x9 << CONFIG_BITS,
				dev->mem + PDM_CTL0_REG);
		config_pdm = readl_relaxed(dev->mem +
				PDM_CTL0_REG);
		dev->pdm_mode = 9;
		break;
	}
	default:
	{
		pr_info("Invalid frequency rate\n");
		return -EINVAL;
	}
	}

	pr_info("selected channels : 0x%2x\n", dev->channel);
	config_pdm = config_pdm | dev->channel;
	writel_relaxed(config_pdm, dev->mem + PDM_CTL0_REG);
	pcm_setup(dev);
	ret = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(params));
	if (ret < 0)
		return ret;
	else
		return 0;
}

static void pcm_setup(struct alif_pcm_dev *dev)
{
	writel_relaxed(0x00000000, (dev->mem + PDM_CH0_FIR_COEF_0));
	writel_relaxed(0x000007FF, (dev->mem + PDM_CH0_FIR_COEF_1));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH0_FIR_COEF_2));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH0_FIR_COEF_3));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH0_FIR_COEF_4));
	writel_relaxed(0x000007FC, (dev->mem + PDM_CH0_FIR_COEF_5));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH0_FIR_COEF_6));
	writel_relaxed(0x000007FB, (dev->mem + PDM_CH0_FIR_COEF_7));
	writel_relaxed(0x000007E4, (dev->mem + PDM_CH0_FIR_COEF_8));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH0_FIR_COEF_9));
	writel_relaxed(0x0000002B, (dev->mem + PDM_CH0_FIR_COEF_10));
	writel_relaxed(0x00000009, (dev->mem + PDM_CH0_FIR_COEF_11));
	writel_relaxed(0x00000016, (dev->mem + PDM_CH0_FIR_COEF_12));
	writel_relaxed(0x00000049, (dev->mem + PDM_CH0_FIR_COEF_13));
	writel_relaxed(0x00000793, (dev->mem + PDM_CH0_FIR_COEF_14));
	writel_relaxed(0x000006F8, (dev->mem + PDM_CH0_FIR_COEF_15));
	writel_relaxed(0x00000045, (dev->mem + PDM_CH0_FIR_COEF_16));
	writel_relaxed(0x00000178, (dev->mem + PDM_CH0_FIR_COEF_17));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH0_IIR_COEF_SEL));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH0_PHASE));
	writel_relaxed(0x00000013, (dev->mem + PDM_CH0_GAIN));
	writel_relaxed(0x00060002, (dev->mem + PDM_CH0_PKDET_TH));
	writel_relaxed(0x00020027, (dev->mem + PDM_CH0_PKDET_ITV));

	writel_relaxed(0x00000001, (dev->mem + PDM_CH1_FIR_COEF_0));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH1_FIR_COEF_1));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH1_FIR_COEF_2));
	writel_relaxed(0x000007F4, (dev->mem + PDM_CH1_FIR_COEF_3));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH1_FIR_COEF_4));
	writel_relaxed(0x000007ED, (dev->mem + PDM_CH1_FIR_COEF_5));
	writel_relaxed(0x000007F5, (dev->mem + PDM_CH1_FIR_COEF_6));
	writel_relaxed(0x000007F4, (dev->mem + PDM_CH1_FIR_COEF_7));
	writel_relaxed(0x000007D3, (dev->mem + PDM_CH1_FIR_COEF_8));
	writel_relaxed(0x000007FE, (dev->mem + PDM_CH1_FIR_COEF_9));
	writel_relaxed(0x000007BC, (dev->mem + PDM_CH1_FIR_COEF_10));
	writel_relaxed(0x000007E5, (dev->mem + PDM_CH1_FIR_COEF_11));
	writel_relaxed(0x000007D9, (dev->mem + PDM_CH1_FIR_COEF_12));
	writel_relaxed(0x00000793, (dev->mem + PDM_CH1_FIR_COEF_13));
	writel_relaxed(0x00000029, (dev->mem + PDM_CH1_FIR_COEF_14));
	writel_relaxed(0x0000072C, (dev->mem + PDM_CH1_FIR_COEF_15));
	writel_relaxed(0x00000072, (dev->mem + PDM_CH1_FIR_COEF_16));
	writel_relaxed(0x000002FD, (dev->mem + PDM_CH1_FIR_COEF_17));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH1_IIR_COEF_SEL));
	writel_relaxed(0x0000001F, (dev->mem + PDM_CH1_PHASE));
	writel_relaxed(0x0000000D, (dev->mem + PDM_CH1_GAIN));
	writel_relaxed(0x00060002, (dev->mem + PDM_CH1_PKDET_TH));
	writel_relaxed(0x0004002D, (dev->mem + PDM_CH1_PKDET_ITV));

	writel_relaxed(0x00000000, (dev->mem + PDM_CH2_FIR_COEF_0));
	writel_relaxed(0x000007FF, (dev->mem + PDM_CH2_FIR_COEF_1));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH2_FIR_COEF_2));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH2_FIR_COEF_3));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH2_FIR_COEF_4));
	writel_relaxed(0x000007FC, (dev->mem + PDM_CH2_FIR_COEF_5));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH2_FIR_COEF_6));
	writel_relaxed(0x000007FB, (dev->mem + PDM_CH2_FIR_COEF_7));
	writel_relaxed(0x000007E4, (dev->mem + PDM_CH2_FIR_COEF_8));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH2_FIR_COEF_9));
	writel_relaxed(0x0000002B, (dev->mem + PDM_CH2_FIR_COEF_10));
	writel_relaxed(0x00000009, (dev->mem + PDM_CH2_FIR_COEF_11));
	writel_relaxed(0x00000016, (dev->mem + PDM_CH2_FIR_COEF_12));
	writel_relaxed(0x00000049, (dev->mem + PDM_CH2_FIR_COEF_13));
	writel_relaxed(0x00000793, (dev->mem + PDM_CH2_FIR_COEF_14));
	writel_relaxed(0x000006F8, (dev->mem + PDM_CH2_FIR_COEF_15));
	writel_relaxed(0x00000045, (dev->mem + PDM_CH2_FIR_COEF_16));
	writel_relaxed(0x00000178, (dev->mem + PDM_CH2_FIR_COEF_17));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH2_IIR_COEF_SEL));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH2_PHASE));
	writel_relaxed(0x00000013, (dev->mem + PDM_CH2_GAIN));
	writel_relaxed(0x00060002, (dev->mem + PDM_CH2_PKDET_TH));
	writel_relaxed(0x00020027, (dev->mem + PDM_CH2_PKDET_ITV));

	writel_relaxed(0x00000001, (dev->mem + PDM_CH3_FIR_COEF_0));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH3_FIR_COEF_1));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH3_FIR_COEF_2));
	writel_relaxed(0x000007F4, (dev->mem + PDM_CH3_FIR_COEF_3));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH3_FIR_COEF_4));
	writel_relaxed(0x000007ED, (dev->mem + PDM_CH3_FIR_COEF_5));
	writel_relaxed(0x000007F5, (dev->mem + PDM_CH3_FIR_COEF_6));
	writel_relaxed(0x000007F4, (dev->mem + PDM_CH3_FIR_COEF_7));
	writel_relaxed(0x000007D3, (dev->mem + PDM_CH3_FIR_COEF_8));
	writel_relaxed(0x000007FE, (dev->mem + PDM_CH3_FIR_COEF_9));
	writel_relaxed(0x000007BC, (dev->mem + PDM_CH3_FIR_COEF_10));
	writel_relaxed(0x000007E5, (dev->mem + PDM_CH3_FIR_COEF_11));
	writel_relaxed(0x000007D9, (dev->mem + PDM_CH3_FIR_COEF_12));
	writel_relaxed(0x00000793, (dev->mem + PDM_CH3_FIR_COEF_13));
	writel_relaxed(0x00000029, (dev->mem + PDM_CH3_FIR_COEF_14));
	writel_relaxed(0x0000072C, (dev->mem + PDM_CH3_FIR_COEF_15));
	writel_relaxed(0x00000072, (dev->mem + PDM_CH3_FIR_COEF_16));
	writel_relaxed(0x000002FD, (dev->mem + PDM_CH3_FIR_COEF_17));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH3_IIR_COEF_SEL));
	writel_relaxed(0x0000001F, (dev->mem + PDM_CH3_PHASE));
	writel_relaxed(0x0000000D, (dev->mem + PDM_CH3_GAIN));
	writel_relaxed(0x00060002, (dev->mem + PDM_CH3_PKDET_TH));
	writel_relaxed(0x0004002D, (dev->mem + PDM_CH3_PKDET_ITV));


	writel_relaxed(0x00000001, (dev->mem + PDM_CH4_FIR_COEF_0));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH4_FIR_COEF_1));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH4_FIR_COEF_2));
	writel_relaxed(0x000007F4, (dev->mem + PDM_CH4_FIR_COEF_3));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH4_FIR_COEF_4));
	writel_relaxed(0x000007ED, (dev->mem + PDM_CH4_FIR_COEF_5));
	writel_relaxed(0x000007F5, (dev->mem + PDM_CH4_FIR_COEF_6));
	writel_relaxed(0x000007F4, (dev->mem + PDM_CH4_FIR_COEF_7));
	writel_relaxed(0x000007D3, (dev->mem + PDM_CH4_FIR_COEF_8));
	writel_relaxed(0x000007FE, (dev->mem + PDM_CH4_FIR_COEF_9));
	writel_relaxed(0x000007BC, (dev->mem + PDM_CH4_FIR_COEF_10));
	writel_relaxed(0x000007E5, (dev->mem + PDM_CH4_FIR_COEF_11));
	writel_relaxed(0x000007D9, (dev->mem + PDM_CH4_FIR_COEF_12));
	writel_relaxed(0x00000793, (dev->mem + PDM_CH4_FIR_COEF_13));
	writel_relaxed(0x00000029, (dev->mem + PDM_CH4_FIR_COEF_14));
	writel_relaxed(0x0000072C, (dev->mem + PDM_CH4_FIR_COEF_15));
	writel_relaxed(0x00000072, (dev->mem + PDM_CH4_FIR_COEF_16));
	writel_relaxed(0x000002FD, (dev->mem + PDM_CH4_FIR_COEF_17));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH4_IIR_COEF_SEL));
	writel_relaxed(0x0000001F, (dev->mem + PDM_CH4_PHASE));
	writel_relaxed(0x0000000D, (dev->mem + PDM_CH4_GAIN));
	writel_relaxed(0x00060002, (dev->mem + PDM_CH4_PKDET_TH));
	writel_relaxed(0x0004002D, (dev->mem + PDM_CH4_PKDET_ITV));

	writel_relaxed(0x00000000, (dev->mem + PDM_CH5_FIR_COEF_0));
	writel_relaxed(0x000007FF, (dev->mem + PDM_CH5_FIR_COEF_1));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH5_FIR_COEF_2));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH5_FIR_COEF_3));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH5_FIR_COEF_4));
	writel_relaxed(0x000007FC, (dev->mem + PDM_CH5_FIR_COEF_5));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH5_FIR_COEF_6));
	writel_relaxed(0x000007FB, (dev->mem + PDM_CH5_FIR_COEF_7));
	writel_relaxed(0x000007E4, (dev->mem + PDM_CH5_FIR_COEF_8));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH5_FIR_COEF_9));
	writel_relaxed(0x0000002B, (dev->mem + PDM_CH5_FIR_COEF_10));
	writel_relaxed(0x00000009, (dev->mem + PDM_CH5_FIR_COEF_11));
	writel_relaxed(0x00000016, (dev->mem + PDM_CH5_FIR_COEF_12));
	writel_relaxed(0x00000049, (dev->mem + PDM_CH5_FIR_COEF_13));
	writel_relaxed(0x00000793, (dev->mem + PDM_CH5_FIR_COEF_14));
	writel_relaxed(0x000006F8, (dev->mem + PDM_CH5_FIR_COEF_15));
	writel_relaxed(0x00000045, (dev->mem + PDM_CH5_FIR_COEF_16));
	writel_relaxed(0x00000178, (dev->mem + PDM_CH5_FIR_COEF_17));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH5_IIR_COEF_SEL));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH5_PHASE));
	writel_relaxed(0x00000013, (dev->mem + PDM_CH5_GAIN));
	writel_relaxed(0x00060002, (dev->mem + PDM_CH5_PKDET_TH));
	writel_relaxed(0x00020027, (dev->mem + PDM_CH5_PKDET_ITV));

	writel_relaxed(0x00000001, (dev->mem + PDM_CH6_FIR_COEF_0));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH6_FIR_COEF_1));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH6_FIR_COEF_2));
	writel_relaxed(0x000007F4, (dev->mem + PDM_CH6_FIR_COEF_3));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH6_FIR_COEF_4));
	writel_relaxed(0x000007ED, (dev->mem + PDM_CH6_FIR_COEF_5));
	writel_relaxed(0x000007F5, (dev->mem + PDM_CH6_FIR_COEF_6));
	writel_relaxed(0x000007F4, (dev->mem + PDM_CH6_FIR_COEF_7));
	writel_relaxed(0x000007D3, (dev->mem + PDM_CH6_FIR_COEF_8));
	writel_relaxed(0x000007FE, (dev->mem + PDM_CH6_FIR_COEF_9));
	writel_relaxed(0x000007BC, (dev->mem + PDM_CH6_FIR_COEF_10));
	writel_relaxed(0x000007E5, (dev->mem + PDM_CH6_FIR_COEF_11));
	writel_relaxed(0x000007D9, (dev->mem + PDM_CH6_FIR_COEF_12));
	writel_relaxed(0x00000793, (dev->mem + PDM_CH6_FIR_COEF_13));
	writel_relaxed(0x00000029, (dev->mem + PDM_CH6_FIR_COEF_14));
	writel_relaxed(0x0000072C, (dev->mem + PDM_CH6_FIR_COEF_15));
	writel_relaxed(0x00000072, (dev->mem + PDM_CH6_FIR_COEF_16));
	writel_relaxed(0x000002FD, (dev->mem + PDM_CH6_FIR_COEF_17));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH6_IIR_COEF_SEL));
	writel_relaxed(0x0000001F, (dev->mem + PDM_CH6_PHASE));
	writel_relaxed(0x0000000D, (dev->mem + PDM_CH6_GAIN));
	writel_relaxed(0x00060002, (dev->mem + PDM_CH6_PKDET_TH));
	writel_relaxed(0x0004002D, (dev->mem + PDM_CH6_PKDET_ITV));


	writel_relaxed(0x00000000, (dev->mem + PDM_CH7_FIR_COEF_0));
	writel_relaxed(0x000007FF, (dev->mem + PDM_CH7_FIR_COEF_1));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH7_FIR_COEF_2));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH7_FIR_COEF_3));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH7_FIR_COEF_4));
	writel_relaxed(0x000007FC, (dev->mem + PDM_CH7_FIR_COEF_5));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH7_FIR_COEF_6));
	writel_relaxed(0x000007FB, (dev->mem + PDM_CH7_FIR_COEF_7));
	writel_relaxed(0x000007E4, (dev->mem + PDM_CH7_FIR_COEF_8));
	writel_relaxed(0x00000000, (dev->mem + PDM_CH7_FIR_COEF_9));
	writel_relaxed(0x0000002B, (dev->mem + PDM_CH7_FIR_COEF_10));
	writel_relaxed(0x00000009, (dev->mem + PDM_CH7_FIR_COEF_11));
	writel_relaxed(0x00000016, (dev->mem + PDM_CH7_FIR_COEF_12));
	writel_relaxed(0x00000049, (dev->mem + PDM_CH7_FIR_COEF_13));
	writel_relaxed(0x00000793, (dev->mem + PDM_CH7_FIR_COEF_14));
	writel_relaxed(0x000006F8, (dev->mem + PDM_CH7_FIR_COEF_15));
	writel_relaxed(0x00000045, (dev->mem + PDM_CH7_FIR_COEF_16));
	writel_relaxed(0x00000178, (dev->mem + PDM_CH7_FIR_COEF_17));
	writel_relaxed(0x00000004, (dev->mem + PDM_CH7_IIR_COEF_SEL));
	writel_relaxed(0x00000003, (dev->mem + PDM_CH7_PHASE));
	writel_relaxed(0x00000013, (dev->mem + PDM_CH7_GAIN));
	writel_relaxed(0x00060002, (dev->mem + PDM_CH7_PKDET_TH));
	writel_relaxed(0x00020027, (dev->mem + PDM_CH7_PKDET_ITV));
}

static void disable_interrupts(struct alif_pcm_dev *dev)
{
	writel_relaxed(0x0, dev->mem + PDM_IRQ_ENABLE_REG);
	return;
}


static void enable_interrupts(struct alif_pcm_dev *dev)
{
	unsigned int irq = 0;

	irq = FIFO_FULL_IRQ_EN |
		FIFO_OVERFLOW_IRQ_EN |
		ALL_CH_AUDIO_DETECT_IRQ_EN;
	writel_relaxed(irq, dev->mem + PDM_IRQ_ENABLE_REG);
	return;
}

static int alif_pcm_trigger(struct snd_pcm_substream *substream, int cmd,
						struct snd_soc_dai *dai)
{
	struct alif_pcm_dev *dev = snd_soc_dai_get_drvdata(dai);

	switch(cmd) {
	case SNDRV_PCM_TRIGGER_START:
		WRITE_ONCE(dev->pdm_buffer_ptr, 0);
		WRITE_ONCE(dev->pdm_buffer_index, 0);
		rcu_assign_pointer(dev->pdm_substream, substream);
		enable_interrupts(dev);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		dev->pdm_buffer_index = 0;
		disable_interrupts(dev);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int alif_pcm_startup(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	pr_info("alif pcm startup\n");

	return 0;
}

static const struct snd_soc_dai_ops alif_pcm_dai_ops = {
	.startup	= alif_pcm_startup,
	.hw_params	= alif_pcm_hw_params,
	.trigger	= alif_pcm_trigger,
};

static struct snd_soc_dai_driver alif_pcm_dai = {
	.probe = pcm_dai_probe,
	.capture = {
			.stream_name = "alif-pcm",
			.channels_min = 1,
			.channels_max = MAX_CHANNELS,
			.rates = ALIF_PCM_RATES,
			.formats = ALIF_PCM_FORMATS,
	},
	.ops = &alif_pcm_dai_ops,
};

static struct snd_pcm_hardware params_capture = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_NONINTERLEAVED,
	.buffer_bytes_max = MAX_BUFFER_BYTES,
	.period_bytes_min = MIN_PERIOD_BYTES,
	.period_bytes_max = MAX_PERIOD_BYTES,
	.channels_min = 1,
	.channels_max = MAX_CHANNELS,
	.periods_min = MIN_PERIODS,
	.periods_max = MAX_PERIODS,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
};

static int pcm_dai_probe(struct snd_soc_dai *dai)
{
	return 0;
}

static snd_pcm_uframes_t component_get_pointer(struct snd_pcm_substream *ss)
{
	struct alif_pcm_dev *dev = ss->runtime->private_data;

	return READ_ONCE(dev->pdm_buffer_ptr);
}

static int alif_pcm_open(struct snd_pcm_substream *ss)
{
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct snd_soc_pcm_runtime *rtd = ss->private_data;
	struct alif_pcm_dev *dev = snd_soc_dai_get_drvdata(rtd->cpu_dai);

	ss->f_flags = 0;
	snd_soc_set_runtime_hwparams(ss, &params_capture);
	runtime->private_data = dev;

	return 0;
}

static int alif_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	size_t size = params_capture.buffer_bytes_max;

	snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL), size, size);
	return 0;
}

static void alif_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}


static const struct snd_pcm_ops alif_pcm_ops = {
		.open =  alif_pcm_open,
		.pointer = component_get_pointer,
};

static const struct snd_soc_component_driver alif_pcm_component = {
		.name = "alif-pcm",
		.pcm_new = alif_pcm_new,
		.pcm_free = alif_pcm_free,
		.ops = &alif_pcm_ops,
};

static int alif_pcm_probe(struct platform_device *pdev)
{
	struct alif_pcm_dev *dev;
	struct resource *mem;
	void __iomem *base;
	int irq;
	int err;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		dev_err(&pdev->dev, "kzalloc error: %d\n", err);
		return err;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(base)) {
		err = PTR_ERR(base);
		dev_err(&pdev->dev, "devm_ioremap_resource error: %d\n", err);
		return err;
	}

	dev->mem = base;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev,
			"alif_pcm: platform_get_irq error: %d\n", irq);
		return irq;
	}
	dev_set_name(&pdev->dev, "%s", "alifpcm");

	err = devm_request_irq(&pdev->dev, irq, alif_pcm_interrupt, 0,
			dev_name(&pdev->dev), dev);
	if (err) {
		dev_err(&pdev->dev,
			"alif_pcm: request irq returned: %d\n", err);
		return err;
	}

	dev->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(dev->pclk)) {
		err = PTR_ERR(dev->pclk);
		dev_err(&pdev->dev,
		"failed to get the peripheral clock: %d\n", err);
		return err;
	}

	/*Enabling all channels by default */
	dev->channel = 0xff;
	dev->dev = &pdev->dev;

	platform_set_drvdata(pdev, dev);

	err = clk_prepare_enable(dev->pclk);
	if (err) {
		dev_err(&pdev->dev,
		"failed to enable the peripheral clock: %d\n", err);
		return err;
	}

	err = devm_snd_soc_register_component(&pdev->dev,
			&alif_pcm_component,
			&alif_pcm_dai, 1);
	if (err) {
		dev_err(&pdev->dev, "failed to register DAI: %d\n", err);
		clk_disable_unprepare(dev->pclk);
		return err;
	}

	err = device_create_file(&pdev->dev, &dev_attr_modefreq);
	if (err) {
		dev_err(&pdev->dev, "Failed to create sysfs file......\n");

		return err;
	}

	err = device_create_file(&pdev->dev, &dev_attr_channelsel);
	if (err) {
		dev_err(&pdev->dev, "Failed to create sysfs file......\n");

		return err;
	}

	return 0;
}

static int alif_pcm_remove(struct platform_device *pdev)
{
	struct alif_pcm_dev *dev = platform_get_drvdata(pdev);
	clk_disable_unprepare(dev->pclk);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id alif_ids[] = {
		{
				.compatible = "alif,alif-pcm",
		},
};
MODULE_DEVICE_TABLE(of, alif_ids);
#endif

static struct platform_driver alif_pcm_driver = {
		.probe		= alif_pcm_probe,
		.remove		= alif_pcm_remove,
		.driver		= {
				.name	= "alif_pcm",
				.of_match_table = alif_ids,
		},
};
module_platform_driver(alif_pcm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Alif PCM Driver");
