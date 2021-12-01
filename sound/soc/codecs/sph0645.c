/*
 * I2S MEMS microphone driver for Knowles SPH0645
 *
 * - Non configurable.
 * - I2S interface, 64 BCLs per frame, 32 bits per channel, 24 bit data
 * - Based on InvenSense ICS-43432 driver
 *
 * Copyright (c) 2021 Alif Semiconductors
 * Author: Harith George <harith.g@alifsemi.com>
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

#define SPH0645_RATE_MIN 16000 /* Hz, from data sheet */
#define SPH0645_RATE_MAX 52800 /* Hz, from data sheet */

#define SPH0645_FORMATS (SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S16_LE)

static struct snd_soc_dai_driver sph0645_dai = {
	.name = "sph0645-hifi",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rate_min = SPH0645_RATE_MIN,
		.rate_max = SPH0645_RATE_MAX,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.formats = SPH0645_FORMATS,
	},
};

static struct snd_soc_component_driver sph0645_component_driver = {
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static int sph0645_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
			&sph0645_component_driver,
			&sph0645_dai, 1);
}

static const struct of_device_id sph0645_ids[] = {
	{ .compatible = "knowles,sph0645", },
	{ /* sentinel*/ }
};
MODULE_DEVICE_TABLE(of, sph0645_ids);

static struct platform_driver sph0645_driver = {
	.driver = {
		.name = "sph0645-codec",
		.of_match_table = of_match_ptr(sph0645_ids),
	},
	.probe = sph0645_probe,
};

module_platform_driver(sph0645_driver);

MODULE_DESCRIPTION("ASoC SPH0645 driver");
MODULE_AUTHOR("Harith George <harith.g@alifsemi.com>");
MODULE_LICENSE("GPL v2");
