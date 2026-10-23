.. SPDX-License-Identifier: GPL-2.0

===========================
Multi-direction codec DAIs
===========================

Overview
========

An ASoC backend (BE) link that bundles **codec DAIs of both directions**
on a **single-direction CPU DAI** can set
``snd_soc_dai_link.multi_direction_codecs`` to tell the ASoC core to
call ``.hw_params`` / ``.hw_free`` on opposite-direction codec DAIs with
their own direction, instead of skipping them.

Typical use case: a smart-amp speaker protection link where the CPU
side is a playback-only DMA (e.g. ``WSA_CODEC_DMA_RX_0`` on Qualcomm
AudioReach), but the codec side exposes both a playback SPKR DAI and a
companion capture V/I-sense DAI on the same SoundWire bus.  Opening the
playback FE should bring up both the amplifier output chain *and* its
V/I feedback capture chain atomically, without userspace having to open
a secondary PCM.

Without the flag, the second codec DAI's ``.hw_params`` is skipped
because its native direction does not match ``substream->stream``;
``snd_soc_dai_stream_valid()`` filters it out.  With the flag set, the
ASoC core:

1. Still iterates every codec DAI on the link.
2. For a codec DAI whose native direction is **opposite** the
   substream's, calls its ``.hw_params`` / ``.hw_free`` with its own
   direction (by briefly flipping the substream direction under
   ``pcm_mutex``, encapsulated in ``soc_pcm_dai_hw_params_at_stream``).
3. Optionally calls ``.multi_direction_params_fixup`` so the machine
   driver can rewrite rate/format/channels on a per-codec copy of the
   params (an 8 kHz / S32_LE V/I-sense codec DAI will not accept a
   48 kHz / S16_LE playback stream's params verbatim).
4. Sends start and stop stream events for the opposite direction in
   prepare and hw_free, so the stream widgets of those codec DAIs power
   up and down with the link.

Links that do not set the flag retain the pre-existing behaviour
exactly.

When to use this
================

Prefer ``multi_direction_codecs`` when:

* the two directions are physically **inseparable** on the codec side
  (same chip, same wire); and
* a single open semantically covers both (speaker protection, playback
  with feedback for echo-reference, haptic/boost feedback); and
* you do not want userspace to see or manage two PCMs.

Prefer **DPCM FE/BE pairs** when:

* the two directions can be exercised independently; or
* userspace legitimately needs to control each direction (classic
  playback + microphone capture).

Prefer a **DAPM supply widget** when:

* only the DAPM power graph (clock, LDO, bias) needs linking, and
* no ``.hw_params`` plumbing is required on the secondary direction.

Interaction with DAPM
=====================

DAPM edges are directional and the flag does not change the graph.  The
opposite-direction codec DAIs' stream widgets are powered by the extra
start and stop stream events the core sends for that direction in
prepare and hw_free.

Only ``.hw_params`` and ``.hw_free`` are called on the opposite-direction
codec DAIs.  Their ``.startup``, ``.prepare``, ``.trigger``,
``.shutdown`` and mute callbacks are not, so a codec DAI used this way
must not depend on them.

Machine-driver example
======================

A machine driver that bundles V/I-sense capture codec DAIs onto a
speaker-playback BE link::

    static int wsa_vi_params_fixup(struct snd_soc_pcm_runtime *rtd,
                                   struct snd_pcm_hw_params *params,
                                   int stream)
    {
        if (stream != SNDRV_PCM_STREAM_CAPTURE)
            return 0;
        /* VI capture chain runs at 8kHz / S32_LE / 4ch */
        hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE)->min      = 8000;
        hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE)->max      = 8000;
        hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS)->min  = 4;
        hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS)->max  = 4;
        snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
        snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
                     SNDRV_PCM_FORMAT_S32_LE);
        return 0;
    }

    SND_SOC_DAILINK_DEFS(wsa_sp, ...);
    SND_SOC_DAILINK_REG(wsa_sp);

    static struct snd_soc_dai_link board_dai_links[] = {
        {
            .name                           = "WSA Playback",
            .multi_direction_codecs         = 1,
            .multi_direction_params_fixup   = wsa_vi_params_fixup,
            SND_SOC_DAILINK_REG(wsa_sp),
            /*
             * codec DAIs include both the playback SPKR DAIs and
             * their companion capture V/I-sense DAIs on the same
             * SoundWire bus.
             */
        },
    };

Notes
=====

* ``multi_direction_params_fixup`` is called with the **flipped**
  direction so the fixup logic can key off ``stream`` to decide
  whether the params need rewriting.
* ``snd_soc_dai_symmetric_set_params()`` runs after the flipped
  ``.hw_params``.  The symmetric-params tracking is per-DAI, so each
  codec DAI records the format it was called with in its own
  direction; nothing cross-contaminates.
* The transient ``substream->stream`` flip is bounded to a single
  codec DAI's ``.hw_params`` / ``.hw_free`` call and its DAPM update,
  under ``pcm_mutex``.  The codec DAI callbacks see the flipped value,
  which is the purpose of the flip.  ALSA core paths that do not take
  ``pcm_mutex``, such as ``snd_pcm_info()``, can observe it during that
  window.
