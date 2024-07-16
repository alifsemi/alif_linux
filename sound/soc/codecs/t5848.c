// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * I2S MEMS microphone driver for InvenSense T5848
 *
 * - Non configurable.
 * - I2S interface, 64 BCLs per frame, 32 bits per channel, 24 bit data
 * - Based on InvenSense ICS-43432 driver
 *
 * Copyright (c) 2024 Alif Semiconductors
 * Author: Aravind Krishnan M <aravind.krishnan@alifsemi.com>
 *
 * Licensed under GPL v2.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#define T5848_RATE_MIN 16000 /* Hz, from data sheet */
#define T5848_RATE_MAX 52800 /* Hz, from data sheet */

#define T5848_FORMATS (SNDRV_PCM_FMTBIT_S32_LE |	\
			SNDRV_PCM_FMTBIT_S24_LE |	\
			SNDRV_PCM_FMTBIT_S16_LE)

static struct snd_soc_dai_driver t5848_dai = {
	.name = "t5848-hifi",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rate_min = T5848_RATE_MIN,
		.rate_max = T5848_RATE_MAX,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.formats = T5848_FORMATS,
	},
};

static const struct snd_soc_component_driver t5848_component_driver = {
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static int t5848_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
			&t5848_component_driver,
			&t5848_dai, 1);
}

static const struct of_device_id t5848_ids[] = {
	{ .compatible = "invensense,t5848", },
	{ /* sentinel*/ }
};
MODULE_DEVICE_TABLE(of, t5848_ids);

static struct platform_driver t5848_driver = {
	.driver = {
		.name = "t5848-codec",
		.of_match_table = of_match_ptr(t5848_ids),
	},
	.probe = t5848_probe,
};

module_platform_driver(t5848_driver);

MODULE_DESCRIPTION("ASoC T5848 driver");
MODULE_AUTHOR("Aravind Krishnan M <aravind.krishnan@alifsemi.com>");
MODULE_LICENSE("GPL v2");
