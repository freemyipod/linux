// SPDX-License-Identifier: GPL-2.0-only
/*
 * N31 ASoC machine — IIS0+CS42 playback + IIS2 FM capture (local only).
 *
 * Playback: IIS0 CPU DAI + CS42L81 SPI codec (peri 10 TX).
 * Capture:  IIS2 CPU DAI + snd-soc-dummy (peri 13 RX). Userspace loops
 * arecord/tinycap → aplay/tinyplay. No FM→BT / A2DP path.
 *
 * CS42 path has no snd_soc_dapm_route table — analog routing uses explicit
 * register writes: cs42_codec_prepare() + cs42_retailos_play_start/stop().
 * (SoC master, codec slave, 16-bit S16_LE).
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

SND_SOC_DAILINK_DEFS(playback,
	DAILINK_COMP_ARRAY(COMP_CPU("s5l8740-i2s")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "cs42l81-hifi")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

SND_SOC_DAILINK_DEFS(fm_capture,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2078-pcm")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("snd-soc-dummy")));

static struct snd_soc_dai_link nano7_dais[] = {
	{
		.name = "CS42L81",
		.stream_name = "Playback",
		SND_SOC_DAILINK_REG(playback),
		.playback_only = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
	},
	{
		.name = "BCM2078-PCM",
		.stream_name = "BCM2078 PCM Capture",
		SND_SOC_DAILINK_REG(fm_capture),
		.capture_only = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
	},
};

static struct snd_soc_card nano7_card = {
	.name = "nano7g-audio",
	.owner = THIS_MODULE,
	.dai_link = nano7_dais,
	.num_links = ARRAY_SIZE(nano7_dais),
};

static int nano7_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *cpu_np, *codec_np, *fm_np;
	int ret;

	cpu_np = of_parse_phandle(dev->of_node, "apple,cpu", 0);
	if (!cpu_np) {
		dev_err(dev, "missing apple,cpu phandle (IIS0)\n");
		return -EINVAL;
	}
	nano7_dais[0].cpus->of_node = cpu_np;
	nano7_dais[0].cpus->dai_name = NULL;
	/* dmaengine PCM lives on the IIS platform device */
	nano7_dais[0].platforms->of_node = cpu_np;
	nano7_dais[0].platforms->name = NULL;

	codec_np = of_parse_phandle(dev->of_node, "apple,codec", 0);
	if (!codec_np)
		codec_np = of_find_compatible_node(NULL, NULL, "cirrus,cs42l81");
	if (!codec_np) {
		dev_err(dev, "missing apple,codec / cirrus,cs42l81\n");
		of_node_put(cpu_np);
		return -EINVAL;
	}
	nano7_dais[0].codecs->of_node = codec_np;
	nano7_dais[0].codecs->name = NULL;
	nano7_dais[0].codecs->dai_name = "cs42l81-hifi";

	fm_np = of_parse_phandle(dev->of_node, "apple,fm-cpu", 0);
	if (!fm_np)
		fm_np = of_find_compatible_node(NULL, NULL, "apple,s5l8740-iis2");
	if (fm_np) {
		nano7_dais[1].cpus->of_node = fm_np;
		nano7_dais[1].cpus->dai_name = NULL;
		nano7_dais[1].platforms->of_node = fm_np;
		nano7_dais[1].platforms->name = NULL;
		nano7_card.num_links = 2;
	} else {
		dev_warn(dev, "no IIS2 fm-cpu — playback-only card\n");
		nano7_card.num_links = 1;
	}

	nano7_card.dev = dev;
	ret = devm_snd_soc_register_card(dev, &nano7_card);
	if (ret) {
		of_node_put(cpu_np);
		of_node_put(codec_np);
		of_node_put(fm_np);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_err(dev, "snd_soc_register_card failed: %d\n", ret);
		return ret;
	}

	dev_info(dev, "nano7g-audio: IIS0+CS42 play%s (no FM→BT)\n",
		 nano7_card.num_links > 1 ? " + BCM2078 PCM capture (IIS2 RX)" : "");
	return 0;
}

static const struct of_device_id nano7_audio_of_match[] = {
	{ .compatible = "apple,n31-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, nano7_audio_of_match);

static struct platform_driver nano7_audio_driver = {
	.probe = nano7_audio_probe,
	.driver = {
		.name = "nano7-audio",
		.of_match_table = nano7_audio_of_match,
		.pm = &snd_soc_pm_ops,
	},
};
module_platform_driver(nano7_audio_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("iPod nano 7G ASoC machine (play + FM capture)");
MODULE_SOFTDEP("pre: cs42l81_spi s5l8740_i2s s5l8740_iis2");
