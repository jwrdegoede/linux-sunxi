// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018-2023, Linaro Limited.
// Copyright (c) 2018, The Linux Foundation. All rights reserved.

#include <dt-bindings/sound/qcom,lpass.h>
#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <sound/soc.h>
#include "sdw.h"

/*
 * Inline direction check that doesn't require snd_soc_dai_stream_valid
 * to be exported for module use.
 */
static bool sdw_dai_has_dir(struct snd_soc_dai *dai, int dir)
{
	return snd_soc_dai_pcm_stream_get_i(dai, dir)->channels_min != 0;
}

static bool qcom_snd_is_sdw_dai(int id)
{
	switch (id) {
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_TX_0:
	case WSA_CODEC_DMA_RX_1:
	case WSA_CODEC_DMA_TX_1:
	case WSA_CODEC_DMA_TX_2:
	case RX_CODEC_DMA_RX_0:
	case TX_CODEC_DMA_TX_0:
	case RX_CODEC_DMA_RX_1:
	case TX_CODEC_DMA_TX_1:
	case RX_CODEC_DMA_RX_2:
	case TX_CODEC_DMA_TX_2:
	case RX_CODEC_DMA_RX_3:
	case TX_CODEC_DMA_TX_3:
	case RX_CODEC_DMA_RX_4:
	case TX_CODEC_DMA_TX_4:
	case RX_CODEC_DMA_RX_5:
	case TX_CODEC_DMA_TX_5:
	case RX_CODEC_DMA_RX_6:
	case RX_CODEC_DMA_RX_7:
	case SLIMBUS_0_RX...SLIMBUS_6_TX:
		return true;
	default:
		break;
	}

	/* DSP Bypass usecase, cpu dai index overlaps with DSP dai ids,
	 * DO NOT MERGE into top switch case */
	switch (id) {
	case LPASS_CDC_DMA_TX3:
	case LPASS_CDC_DMA_RX0:
		return true;
	default:
		break;
	}

	return false;
}

/*
 * Allocate a stream runtime for one direction and set it on the codec DAIs
 * of that direction.
 */
static int qcom_snd_sdw_setup_dir(struct snd_soc_pcm_runtime *rtd,
				  int dir, const char *cpu_name)
{
	struct sdw_stream_runtime *sruntime;
	struct snd_soc_dai *codec_dai;
	u32 rx_ch[SDW_MAX_PORTS], tx_ch[SDW_MAX_PORTS];
	u32 rx_ch_cnt = 0, tx_ch_cnt = 0;
	int ret, i, j;
	bool used = false;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		if (sdw_dai_has_dir(codec_dai, dir)) {
			used = true;
			break;
		}
	}
	if (!used)
		return 0;

	sruntime = sdw_alloc_stream(cpu_name, SDW_STREAM_PCM);
	if (!sruntime)
		return -ENOMEM;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		if (!sdw_dai_has_dir(codec_dai, dir))
			continue;

		ret = snd_soc_dai_set_stream(codec_dai, sruntime, dir);
		if (ret < 0 && ret != -ENOTSUPP) {
			dev_err(rtd->dev, "Failed to set sdw stream on %s\n", codec_dai->name);
			goto err;
		} else if (ret == -ENOTSUPP) {
			continue;
		}

		ret = snd_soc_dai_get_channel_map(codec_dai, &tx_ch_cnt, tx_ch,
						  &rx_ch_cnt, rx_ch);
		if (ret != 0 && ret != -ENOTSUPP) {
			dev_err(rtd->dev, "Failed to get codec chan map %s\n", codec_dai->name);
			goto err;
		}
	}

	switch (snd_soc_rtd_to_cpu(rtd, 0)->id) {
	case RX_CODEC_DMA_RX_0:
	case TX_CODEC_DMA_TX_3:
		if (tx_ch_cnt || rx_ch_cnt) {
			for_each_rtd_codec_dais(rtd, j, codec_dai) {
				if (!sdw_dai_has_dir(codec_dai, dir))
					continue;
				ret = snd_soc_dai_set_channel_map(codec_dai,
								  tx_ch_cnt, tx_ch,
								  rx_ch_cnt, rx_ch);
				if (ret != 0 && ret != -ENOTSUPP)
					goto err;
			}
		}
	}

	return 0;
err:
	sdw_release_stream(sruntime);
	return ret;
}

/**
 * qcom_snd_sdw_startup() - Helper to start Soundwire stream for SoC audio card
 * @substream: The PCM substream from audio, as passed to snd_soc_ops->startup()
 *
 * Helper for the SoC audio card (snd_soc_ops->startup()) to allocate and set
 * Soundwire stream runtime to each codec DAI.
 *
 * The shutdown() callback should call sdw_release_stream() on the same
 * sdw_stream_runtime.
 *
 * Return: 0 or errno
 */
int qcom_snd_sdw_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	if (!qcom_snd_is_sdw_dai(cpu_dai->id))
		return 0;

	ret = qcom_snd_sdw_setup_dir(rtd, substream->stream, cpu_dai->name);
	if (ret)
		return ret;

	/* Also set up the stream of the other direction on such links */
	if (rtd->dai_link->multi_direction_codecs) {
		ret = qcom_snd_sdw_setup_dir(rtd, substream->stream ^ 1,
					     cpu_dai->name);
		if (ret) {
			struct sdw_stream_runtime *sr;
			struct snd_soc_dai *cd;
			int i;

			for_each_rtd_codec_dais(rtd, i, cd) {
				if (!sdw_dai_has_dir(cd, substream->stream))
					continue;
				sr = snd_soc_dai_get_stream(cd, substream->stream);
				if (!IS_ERR_OR_NULL(sr)) {
					sdw_release_stream(sr);
					break;
				}
			}
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_startup);

/* Prepare+enable the SDW stream for one direction. */
static int qcom_snd_sdw_prepare_dir(struct snd_soc_pcm_runtime *rtd, int dir)
{
	struct sdw_stream_runtime *sruntime = NULL;
	struct snd_soc_dai *codec_dai;
	int ret, i;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		if (!sdw_dai_has_dir(codec_dai, dir))
			continue;
		sruntime = snd_soc_dai_get_stream(codec_dai, dir);
		if (!IS_ERR(sruntime) && sruntime)
			break;
	}
	if (!sruntime || IS_ERR(sruntime))
		return 0;

	ret = sdw_prepare_stream(sruntime);
	if (ret)
		return ret;

	ret = sdw_enable_stream(sruntime);
	if (ret) {
		sdw_deprepare_stream(sruntime);
		return ret;
	}
	return 0;
}

int qcom_snd_sdw_prepare(struct snd_pcm_substream *substream,
			 bool *stream_prepared)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	if (!qcom_snd_is_sdw_dai(cpu_dai->id))
		return 0;

	if (*stream_prepared)
		return 0;

	/**
	 * NOTE: there is a strict hw requirement about the ordering of port
	 * enables and actual WSA881x PA enable. PA enable should only happen
	 * after soundwire ports are enabled if not DC on the line is
	 * accumulated resulting in Click/Pop Noise
	 * PA enable/mute are handled as part of codec DAPM and digital mute.
	 */
	ret = qcom_snd_sdw_prepare_dir(rtd, substream->stream);
	if (ret)
		return ret;

	if (rtd->dai_link->multi_direction_codecs) {
		ret = qcom_snd_sdw_prepare_dir(rtd, substream->stream ^ 1);
		if (ret) {
			struct sdw_stream_runtime *sr = NULL;
			struct snd_soc_dai *cd;
			int i;

			for_each_rtd_codec_dais(rtd, i, cd) {
				if (!sdw_dai_has_dir(cd, substream->stream))
					continue;
				sr = snd_soc_dai_get_stream(cd, substream->stream);
				if (!IS_ERR(sr) && sr)
					break;
			}
			if (sr && !IS_ERR(sr)) {
				sdw_disable_stream(sr);
				sdw_deprepare_stream(sr);
			}
			return ret;
		}
	}

	*stream_prepared = true;
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_prepare);

struct sdw_stream_runtime *qcom_snd_sdw_get_stream(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sdw_stream_runtime *sruntime;
	int i;

	if (!qcom_snd_is_sdw_dai(cpu_dai->id))
		return NULL;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		sruntime = snd_soc_dai_get_stream(codec_dai, substream->stream);
		if (sruntime != ERR_PTR(-ENOTSUPP))
			return sruntime;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_get_stream);

static struct sdw_stream_runtime *
sdw_find_dir_stream(struct snd_soc_pcm_runtime *rtd, int dir)
{
	struct snd_soc_dai *codec_dai;
	struct sdw_stream_runtime *sr;
	int i;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		if (!sdw_dai_has_dir(codec_dai, dir))
			continue;
		sr = snd_soc_dai_get_stream(codec_dai, dir);
		if (!IS_ERR(sr) && sr)
			return sr;
	}
	return NULL;
}

void qcom_snd_sdw_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sdw_stream_runtime *sr;

	sr = sdw_find_dir_stream(rtd, substream->stream);
	if (sr)
		sdw_release_stream(sr);

	if (rtd->dai_link->multi_direction_codecs) {
		sr = sdw_find_dir_stream(rtd, substream->stream ^ 1);
		if (sr)
			sdw_release_stream(sr);
	}
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_shutdown);

int qcom_snd_sdw_hw_free(struct snd_pcm_substream *substream, bool *stream_prepared)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sdw_stream_runtime *sr;

	if (!qcom_snd_is_sdw_dai(cpu_dai->id))
		return 0;

	if (!*stream_prepared)
		return 0;

	sr = sdw_find_dir_stream(rtd, substream->stream);
	if (sr) {
		sdw_disable_stream(sr);
		sdw_deprepare_stream(sr);
	}

	if (rtd->dai_link->multi_direction_codecs) {
		sr = sdw_find_dir_stream(rtd, substream->stream ^ 1);
		if (sr) {
			sdw_disable_stream(sr);
			sdw_deprepare_stream(sr);
		}
	}

	*stream_prepared = false;
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_sdw_hw_free);
MODULE_DESCRIPTION("Qualcomm ASoC SoundWire helper functions");
MODULE_LICENSE("GPL");
