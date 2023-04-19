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
#include <sound/soc-card.h>
#include <sound/soc-link.h>
#include <sound/simple_card.h>


#define ALIF_PCM_CONFIG_REGISTER     0x0000
#define ALIF_PCM_CTL_REGISTER        0x0004
#define ALIF_PCM_THRESHOLD_REGISTER  0x0008
#define ALIF_PCM_STATUS_REGISTER     0x000C
#define ALIF_PCM_INTERRUPT_REGISTER  0x001C
#define ALIF_PCM_DATA_REGISTER       0x0020
#define BUFFER_SIZE                  (4000)
//for debugging purposes
#define TEMP_BUFFER_SIZE             (100000)

#define ALIF_PCM_RATES          SNDRV_PCM_RATE_8000_192000
#define ALIF_PCM_FORMATS        (SNDRV_PCM_FMTBIT_S16_LE)

#define PINMUX_ADDR                  0X1A603000
#define CFGMASTER_ADDR               0x4902F000

#define PINMUX_CLOCK0_OFFSET    0x64
#define PINMUX_CLOCK1_OFFSET    0x6C
#define PINMUX_CLOCK2_OFFSET    0x170
#define PINMUX_CLOCK3_OFFSET    0x174

#define PINMUX_DATA0_OFFSET     0x60
#define PINMUX_DATA1_OFFSET     0x68
#define PINMUX_DATA2_OFFSET     0xb0
#define PINMUX_DATA3_OFFSET     0xb4

#define MAX_CHANNELS            8

#define MIN_PERIODS             (16)
#define MAX_PERIODS             (32)
#define MIN_PERIOD_BYTES        (8 * sizeof(unsigned short))
#define MAX_PERIOD_BYTES        (1000 * sizeof(unsigned short))
#define MAX_BUFFER_BYTES        (BUFFER_SIZE * MAX_CHANNELS * sizeof(unsigned short))

unsigned short *circular_buffer = NULL;
unsigned short *temp_buffer = NULL;
unsigned int pcm_buffer_tail;
unsigned int pcm_buffer_head;
unsigned int pcm_buffer_num_sets;
spinlock_t buffer_lock;

struct alif_pcm_dev {
	struct device	*dev;
	void __iomem	*mem;
	struct clk      *pclk;
	int             channel;
};

int pcm_intr_status;

int record_data = 0;

struct snd_pcm_substream *substreamp;

static int alif_pcm_dai_probe(struct snd_soc_dai *dai);
static int alif_pcm_dai_open(struct snd_soc_component *component, struct snd_pcm_substream *ss);

static irqreturn_t alif_pcm_interrupt(int irq, void *dev_id)
{
	struct alif_pcm_dev *dev = dev_id;
	irqreturn_t ret = IRQ_HANDLED;
	unsigned int n_items;
	unsigned int i, k;
	unsigned int tmp;
	unsigned int index;
	u32 result;

	pcm_intr_status = readl_relaxed(dev->mem + ALIF_PCM_STATUS_REGISTER + 8);

	n_items = readl_relaxed(dev->mem + ALIF_PCM_STATUS_REGISTER);

	if(record_data) {

		spin_lock(&buffer_lock);

		for(i=0; i<n_items; i++) {

			/* read all four data registers
			 * They make one item.
			 */

			for(k=0; k<MAX_CHANNELS/2; k++) {
				index = (k * 2 * BUFFER_SIZE) + pcm_buffer_tail + i;
				result = readl_relaxed(dev->mem + ALIF_PCM_DATA_REGISTER + (k*sizeof(u32)));

				circular_buffer[index] = result & 0x0000FFFF;
				circular_buffer[index + BUFFER_SIZE] = (result & 0xFFFF0000) >> 16;
			}

		}

		pcm_buffer_tail = (pcm_buffer_tail+n_items) % BUFFER_SIZE;
		pcm_buffer_num_sets += n_items;
		spin_unlock(&buffer_lock);

	} else {
		/* clear the fifo */
		for(i=0; i<n_items; i++) {
			for(k=0; k<4; k++) {
				tmp = readl_relaxed(dev->mem + ALIF_PCM_DATA_REGISTER + (k*sizeof(u32)));
			}
		}
	}

	if(substreamp)
		snd_pcm_period_elapsed(substreamp);

	return ret;
}

int get_pcm_data_copy(int channel, char __user *buf, int count, unsigned int pos)
{
	unsigned int first_n;
	unsigned int remaining;
	int rc = 0;
	unsigned int index;

	spin_lock(&buffer_lock);

	if(count > pcm_buffer_num_sets) {
		count = pcm_buffer_num_sets;
	}

	if(count < 0) {
		printk("get_pcm_data_copy: count %d\n", count);
		rc = -EINVAL;
		goto out;
	}

	if(pos >= BUFFER_SIZE) {
		printk("get_pcm_data_copy: pos greater than buffer size: %d %d\n", pos, BUFFER_SIZE);
		rc = -EINVAL;
		goto out;
	}

	index = (channel * BUFFER_SIZE);

	if(pos + count <= BUFFER_SIZE) {

		rc = copy_to_user(buf, circular_buffer + index + pos, count * sizeof(unsigned short));
		if(rc) {
			printk("copy_to_user error\n");
			goto out;
		}

	} else {
		first_n = BUFFER_SIZE - pos;
		remaining = count - first_n;

		rc = copy_to_user(buf, circular_buffer + index + pos, first_n * sizeof(unsigned short));

		if (rc == 0) {
			rc = copy_to_user(buf + first_n * sizeof(unsigned short), circular_buffer + index, remaining * sizeof(unsigned short));
		}

		if(rc) {
			printk("get_pcm_data_copy: copy_to_user error\n");
			goto out;
		}
	}

	pcm_buffer_head = (pcm_buffer_head + count) % BUFFER_SIZE;
	pcm_buffer_num_sets -= count;
out:
	spin_unlock(&buffer_lock);

	if(rc < 0)
		return rc;
	else
		return (count * sizeof(unsigned short));
}

static snd_pcm_uframes_t get_pcm_data_pointer(void)
{
	snd_pcm_uframes_t pointer;

	spin_lock(&buffer_lock);

	pointer = pcm_buffer_tail;

	spin_unlock(&buffer_lock);

	return pointer;
}

static void pcm_setup(struct alif_pcm_dev *dev)
{

	/*
	 * Set the values for each channel basaed on the simulation testing
	 */

	/* Channel 0 */
	writel_relaxed(0x00000000, dev->mem+0x040);
	writel_relaxed(0x000007FF, dev->mem+0x044);
	writel_relaxed(0x00000000,  dev->mem+0x048);
	writel_relaxed(0x00000004, dev->mem+0x04c);
	writel_relaxed(0x00000004, dev->mem+0x050);
	writel_relaxed(0x000007FC, dev->mem+0x054);
	writel_relaxed(0x00000000, dev->mem+0x058);
	writel_relaxed(0x000007FB, dev->mem+0x05c);
	writel_relaxed(0x000007E4, dev->mem+0x060);
	writel_relaxed(0x00000000, dev->mem+0x064);
	writel_relaxed(0x0000002B, dev->mem+0x068);
	writel_relaxed(0x00000009, dev->mem+0x06c);
	writel_relaxed(0x00000016, dev->mem+0x070);
	writel_relaxed(0x00000049, dev->mem+0x074);
	writel_relaxed(0x00000793, dev->mem+0x078);
	writel_relaxed(0x000006F8, dev->mem+0x07c);
	writel_relaxed(0x00000045, dev->mem+0x080);
	writel_relaxed(0x00000178, dev->mem+0x084);
	writel_relaxed(0x00000004, dev->mem+0x0c0);
	writel_relaxed(0x00000003, dev->mem+0x0c4);
	writel_relaxed(0x00000013, dev->mem+0x0c8);
	writel_relaxed(0x00060002, dev->mem+0x0cc);
	writel_relaxed(0x00020027, dev->mem+0x0d0);

	/* Channel 1*/
	writel_relaxed(0x00000001, (dev->mem+0x140));
	writel_relaxed(0x00000003, (dev->mem+0x144));
	writel_relaxed(0x00000003, (dev->mem+0x148));
	writel_relaxed(0x000007F4, (dev->mem+0x14c));
	writel_relaxed(0x00000004, (dev->mem+0x150));
	writel_relaxed(0x000007ED, (dev->mem+0x154));
	writel_relaxed(0x000007F5, (dev->mem+0x158));
	writel_relaxed(0x000007F4, (dev->mem+0x15c));
	writel_relaxed(0x000007D3, (dev->mem+0x160));
	writel_relaxed(0x000007FE, (dev->mem+0x164));
	writel_relaxed(0x000007BC, (dev->mem+0x168));
	writel_relaxed(0x000007E5, (dev->mem+0x16c));
	writel_relaxed(0x000007D9, (dev->mem+0x170));
	writel_relaxed(0x00000793, (dev->mem+0x174));
	writel_relaxed(0x00000029, (dev->mem+0x178));
	writel_relaxed(0x0000072C, (dev->mem+0x17c));
	writel_relaxed(0x00000072, (dev->mem+0x180));
	writel_relaxed(0x000002FD, (dev->mem+0x184));
	writel_relaxed(0x00000004, (dev->mem+0x1c0));
	writel_relaxed(0x0000001F, (dev->mem+0x1c4));
	writel_relaxed(0x0000000D, (dev->mem+0x1c8));
	writel_relaxed(0x00060002, (dev->mem+0x1cc));
	writel_relaxed(0x0004002D, (dev->mem+0x1d0));

	/* Channel 2 */
	writel_relaxed(0x00000000, (dev->mem+0x240));
	writel_relaxed(0x000007FF, (dev->mem+0x244));
	writel_relaxed(0x00000000, (dev->mem+0x248));
	writel_relaxed(0x00000004, (dev->mem+0x24c));
	writel_relaxed(0x00000004, (dev->mem+0x250));
	writel_relaxed(0x000007FC, (dev->mem+0x254));
	writel_relaxed(0x00000000, (dev->mem+0x258));
	writel_relaxed(0x000007FB, (dev->mem+0x25c));
	writel_relaxed(0x000007E4, (dev->mem+0x260));
	writel_relaxed(0x00000000, (dev->mem+0x264));
	writel_relaxed(0x0000002B, (dev->mem+0x268));
	writel_relaxed(0x00000009, (dev->mem+0x26c));
	writel_relaxed(0x00000016, (dev->mem+0x270));
	writel_relaxed(0x00000049, (dev->mem+0x274));
	writel_relaxed(0x00000793, (dev->mem+0x278));
	writel_relaxed(0x000006F8, (dev->mem+0x27c));
	writel_relaxed(0x00000045, (dev->mem+0x280));
	writel_relaxed(0x00000178, (dev->mem+0x284));
	writel_relaxed(0x00000004, (dev->mem+0x2c0));
	writel_relaxed(0x00000003, (dev->mem+0x2c4));
	writel_relaxed(0x00000013, (dev->mem+0x2c8));
	writel_relaxed(0x00060002, (dev->mem+0x2cc));
	writel_relaxed(0x00020027, (dev->mem+0x2d0));

	/* Channel 3 */
	writel_relaxed(0x00000001, (dev->mem+0x340));
	writel_relaxed(0x00000003, (dev->mem+0x344));
	writel_relaxed(0x00000003, (dev->mem+0x348));
	writel_relaxed(0x000007F4, (dev->mem+0x34c));
	writel_relaxed(0x00000004, (dev->mem+0x350));
	writel_relaxed(0x000007ED, (dev->mem+0x354));
	writel_relaxed(0x000007F5, (dev->mem+0x358));
	writel_relaxed(0x000007F4, (dev->mem+0x35c));
	writel_relaxed(0x000007D3, (dev->mem+0x360));
	writel_relaxed(0x000007FE, (dev->mem+0x364));
	writel_relaxed(0x000007BC, (dev->mem+0x368));
	writel_relaxed(0x000007E5, (dev->mem+0x36c));
	writel_relaxed(0x000007D9, (dev->mem+0x370));
	writel_relaxed(0x00000793, (dev->mem+0x374));
	writel_relaxed(0x00000029, (dev->mem+0x378));
	writel_relaxed(0x0000072C, (dev->mem+0x37c));
	writel_relaxed(0x00000072, (dev->mem+0x380));
	writel_relaxed(0x000002FD, (dev->mem+0x384));
	writel_relaxed(0x00000004, (dev->mem+0x3c0));
	writel_relaxed(0x0000001F, (dev->mem+0x3c4));
	writel_relaxed(0x0000000D, (dev->mem+0x3c8));
	writel_relaxed(0x00060002, (dev->mem+0x3cc));
	writel_relaxed(0x0004002D, (dev->mem+0x3d0));


	/* Channel 4 */
	writel_relaxed(0x00000001, (dev->mem+0x440));
	writel_relaxed(0x00000003, (dev->mem+0x444));
	writel_relaxed(0x00000003, (dev->mem+0x448));
	writel_relaxed(0x000007F4, (dev->mem+0x44c));
	writel_relaxed(0x00000004, (dev->mem+0x450));
	writel_relaxed(0x000007ED, (dev->mem+0x454));
	writel_relaxed(0x000007F5, (dev->mem+0x458));
	writel_relaxed(0x000007F4, (dev->mem+0x45c));
	writel_relaxed(0x000007D3, (dev->mem+0x460));
	writel_relaxed(0x000007FE, (dev->mem+0x464));
	writel_relaxed(0x000007BC, (dev->mem+0x468));
	writel_relaxed(0x000007E5, (dev->mem+0x46c));
	writel_relaxed(0x000007D9, (dev->mem+0x470));
	writel_relaxed(0x00000793, (dev->mem+0x474));
	writel_relaxed(0x00000029, (dev->mem+0x478));
	writel_relaxed(0x0000072C, (dev->mem+0x47c));
	writel_relaxed(0x00000072, (dev->mem+0x480));
	writel_relaxed(0x000002FD, (dev->mem+0x484));
	writel_relaxed(0x00000004, (dev->mem+0x4c0));
	writel_relaxed(0x0000001F, (dev->mem+0x4c4));
	writel_relaxed(0x0000000D, (dev->mem+0x4c8));
	writel_relaxed(0x00060002, (dev->mem+0x4cc));
	writel_relaxed(0x0004002D, (dev->mem+0x4d0));

	/* Channel 5 */
	writel_relaxed(0x00000000, (dev->mem+0x540));
	writel_relaxed(0x000007FF, (dev->mem+0x544));
	writel_relaxed(0x00000000, (dev->mem+0x548));
	writel_relaxed(0x00000004, (dev->mem+0x54c));
	writel_relaxed(0x00000004, (dev->mem+0x550));
	writel_relaxed(0x000007FC, (dev->mem+0x554));
	writel_relaxed(0x00000000, (dev->mem+0x558));
	writel_relaxed(0x000007FB, (dev->mem+0x55c));
	writel_relaxed(0x000007E4, (dev->mem+0x560));
	writel_relaxed(0x00000000, (dev->mem+0x564));
	writel_relaxed(0x0000002B, (dev->mem+0x568));
	writel_relaxed(0x00000009, (dev->mem+0x56c));
	writel_relaxed(0x00000016, (dev->mem+0x570));
	writel_relaxed(0x00000049, (dev->mem+0x574));
	writel_relaxed(0x00000793, (dev->mem+0x578));
	writel_relaxed(0x000006F8, (dev->mem+0x57c));
	writel_relaxed(0x00000045, (dev->mem+0x580));
	writel_relaxed(0x00000178, (dev->mem+0x584));
	writel_relaxed(0x00000004, (dev->mem+0x5c0));
	writel_relaxed(0x00000003, (dev->mem+0x5c4));
	writel_relaxed(0x00000013, (dev->mem+0x5c8));
	writel_relaxed(0x00060002, (dev->mem+0x5cc));
	writel_relaxed(0x00020027, (dev->mem+0x5d0));

	/* Channel 6 */
	writel_relaxed(0x00000001, (dev->mem+0x640));
	writel_relaxed(0x00000003, (dev->mem+0x644));
	writel_relaxed(0x00000003, (dev->mem+0x648));
	writel_relaxed(0x000007F4, (dev->mem+0x64c));
	writel_relaxed(0x00000004, (dev->mem+0x650));
	writel_relaxed(0x000007ED, (dev->mem+0x654));
	writel_relaxed(0x000007F5, (dev->mem+0x658));
	writel_relaxed(0x000007F4, (dev->mem+0x65c));
	writel_relaxed(0x000007D3, (dev->mem+0x660));
	writel_relaxed(0x000007FE, (dev->mem+0x664));
	writel_relaxed(0x000007BC, (dev->mem+0x668));
	writel_relaxed(0x000007E5, (dev->mem+0x66c));
	writel_relaxed(0x000007D9, (dev->mem+0x670));
	writel_relaxed(0x00000793, (dev->mem+0x674));
	writel_relaxed(0x00000029, (dev->mem+0x678));
	writel_relaxed(0x0000072C, (dev->mem+0x67c));
	writel_relaxed(0x00000072, (dev->mem+0x680));
	writel_relaxed(0x000002FD, (dev->mem+0x684));
	writel_relaxed(0x00000004, (dev->mem+0x6c0));
	writel_relaxed(0x0000001F, (dev->mem+0x6c4));
	writel_relaxed(0x0000000D, (dev->mem+0x6c8));
	writel_relaxed(0x00060002, (dev->mem+0x6cc));
	writel_relaxed(0x0004002D, (dev->mem+0x6d0));


	/* Channel 7 */
	writel_relaxed(0x00000000, (dev->mem+0x740));
	writel_relaxed(0x000007FF, (dev->mem+0x744));
	writel_relaxed(0x00000000, (dev->mem+0x748));
	writel_relaxed(0x00000004, (dev->mem+0x74c));
	writel_relaxed(0x00000004, (dev->mem+0x750));
	writel_relaxed(0x000007FC, (dev->mem+0x754));
	writel_relaxed(0x00000000, (dev->mem+0x758));
	writel_relaxed(0x000007FB, (dev->mem+0x75c));
	writel_relaxed(0x000007E4, (dev->mem+0x760));
	writel_relaxed(0x00000000, (dev->mem+0x764));
	writel_relaxed(0x0000002B, (dev->mem+0x768));
	writel_relaxed(0x00000009, (dev->mem+0x76c));
	writel_relaxed(0x00000016, (dev->mem+0x770));
	writel_relaxed(0x00000049, (dev->mem+0x774));
	writel_relaxed(0x00000793, (dev->mem+0x778));
	writel_relaxed(0x000006F8, (dev->mem+0x77c));
	writel_relaxed(0x00000045, (dev->mem+0x780));
	writel_relaxed(0x00000178, (dev->mem+0x784));
	writel_relaxed(0x00000004, (dev->mem+0x7c0));
	writel_relaxed(0x00000003, (dev->mem+0x7c4));
	writel_relaxed(0x00000013, (dev->mem+0x7c8));
	writel_relaxed(0x00060002, (dev->mem+0x7cc));
	writel_relaxed(0x00020027, (dev->mem+0x7d0));

}

static void setup_config(struct alif_pcm_dev *dev)
{
	/* Configuration register setting */
	writel_relaxed(0x500ff, dev->mem + ALIF_PCM_CONFIG_REGISTER);

	/* Control register setting */
	writel_relaxed(0x4, dev->mem + ALIF_PCM_CTL_REGISTER);

	/* set the threshold for interrupt */
	writel_relaxed(7, dev->mem + ALIF_PCM_THRESHOLD_REGISTER);

	pcm_setup(dev);
}

int setup_buffer(struct alif_pcm_dev *dev)
{
	circular_buffer = devm_kmalloc(dev->dev, BUFFER_SIZE * MAX_CHANNELS * sizeof(unsigned short), GFP_KERNEL);

	if(!circular_buffer) {
		return -ENOMEM;
	}

	/* this buffer is used for debugging purposes
	 * To copy the alsa file to a contiguous buffer
	 */
	temp_buffer = devm_kmalloc(dev->dev, TEMP_BUFFER_SIZE, GFP_KERNEL);

	if(!temp_buffer) {
		return -ENOMEM;
	}

	printk(KERN_DEBUG "temp buffer: %ls\n", temp_buffer);

	pcm_buffer_tail = 0;

	pcm_buffer_head = 0;

	pcm_buffer_num_sets = 0;

	spin_lock_init(&buffer_lock);

	return 0;
}

static void disable_interrupts(struct alif_pcm_dev *dev)
{
	u32 value = 0x0;

	writel_relaxed(value, dev->mem + ALIF_PCM_INTERRUPT_REGISTER);

	return;
}

static void enable_interrupts(struct alif_pcm_dev *dev)
{
	u32 value = 0xff03;

	writel_relaxed(value, dev->mem + ALIF_PCM_INTERRUPT_REGISTER);

	return;
}

static int alif_pcm_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct alif_pcm_dev *dev = snd_soc_dai_get_drvdata(dai);

	switch(cmd) {

	case SNDRV_PCM_TRIGGER_START:
		/* Enable channels */
		pcm_buffer_tail = 0;
		pcm_buffer_head = 0;
		pcm_buffer_num_sets = 0;
		record_data = 1;

		/* Enable interrupts */
		enable_interrupts(dev);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		record_data = 0;

		disable_interrupts(dev);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int alif_pcm_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	printk("alif_pcm_startup\n");

	return 0;
}

static const struct snd_soc_dai_ops alif_pcm_dai_ops = {
	.startup = alif_pcm_startup,
	.trigger = alif_pcm_trigger,
};

static struct snd_soc_dai_driver alif_pcm_dai = {
	.probe = alif_pcm_dai_probe,
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
	.info = SNDRV_PCM_INFO_NONINTERLEAVED,
	.buffer_bytes_max = MAX_BUFFER_BYTES,
	.period_bytes_min = MIN_PERIOD_BYTES,
	.period_bytes_max = MAX_PERIOD_BYTES,
	.channels_min = 1,
	.channels_max = MAX_CHANNELS,
	.periods_min = MIN_PERIODS,
	.periods_max = MAX_PERIODS,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
};

static int alif_pcm_dai_probe(struct snd_soc_dai *dai)
{
	return 0;
}

static int alif_component_copy_user(struct snd_soc_component *component,
				    struct snd_pcm_substream *ss,
				    int channel,
				    unsigned long pos,
				    void __user *buf,
				    unsigned long bytes)
{
	int count;
	unsigned int elem;

	count = bytes/ sizeof(unsigned short);
	elem = pos / sizeof(unsigned short);

	return get_pcm_data_copy(channel, buf, count, elem);
}

static snd_pcm_uframes_t alif_component_pointer(struct snd_soc_component * component, struct snd_pcm_substream *ss)
{
	unsigned int p;

	p = get_pcm_data_pointer();

	return (snd_pcm_uframes_t) (p);
}

static int alif_pcm_dai_open(struct snd_soc_component *component, struct snd_pcm_substream *ss)
{
	struct snd_pcm_runtime *runtime = ss->runtime;

	runtime->hw = params_capture;
	substreamp = ss;

	return 0;
}

static const struct snd_soc_component_driver alif_pcm_component = {
	.name = "alif-pcm",
	.open = alif_pcm_dai_open,
	.copy_user = alif_component_copy_user,
	.pointer = alif_component_pointer,
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
		dev_err(&pdev->dev, "alif_pcm: platform_get_irq error: %d\n", irq);
		return irq;
	}

	err = devm_request_irq(&pdev->dev, irq, alif_pcm_interrupt, 0,
			       pdev->name, dev);
	if (err) {
		dev_err(&pdev->dev, "alif_pcm: request irq returned: %d\n", err);
		return err;
	}

	dev->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(dev->pclk)) {
		err = PTR_ERR(dev->pclk);
		dev_err(&pdev->dev,
			"failed to get the peripheral clock: %d\n", err);
		return err;
	}

	dev->dev = &pdev->dev;

	platform_set_drvdata(pdev, dev);

	err = clk_prepare_enable(dev->pclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable the peripheral clock: %d\n", err);
		return err;
	}

	dev_set_name(&pdev->dev, "%s", "alifpcm");

	err = devm_snd_soc_register_component(&pdev->dev,
					      &alif_pcm_component,
					      &alif_pcm_dai, 1);

	if (err) {
		dev_err(&pdev->dev, "failed to register DAI: %d\n", err);
		clk_disable_unprepare(dev->pclk);
		return err;
	}

	setup_config(dev);
	setup_buffer(dev);

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
