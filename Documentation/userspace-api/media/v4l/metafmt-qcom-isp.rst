.. SPDX-License-Identifier: GPL-2.0
.. c:namespace:: V4L

.. _v4l2-meta-fmt-qcom-isp-params:

**************************************
V4L2_META_FMT_QCOM_ISP_PARAMS ('QCIP')
**************************************

Configuration Parameters
========================

The ``V4L2_META_FMT_QCOM_ISP_PARAMS`` format carries image processing
configuration for the ISP engines found in the Qualcomm Camera Subsystem
(CAMSS). It is passed to a metadata output video node using the
:c:type:`v4l2_meta_format` interface.

Rather than a single struct containing sub-structs for each configurable area
of the ISP, parameters use the :ref:`v4l2-isp` parameters system, through which
groups of parameters are defined as distinct structs or "blocks" which may be
added to the data member of :c:type:`v4l2_isp_params_buffer`. Userspace is
responsible for populating the data member with the blocks that need to be
configured by the driver. Each block-specific struct embeds
:c:type:`v4l2_isp_params_block_header` as its first member and userspace must
populate the type member with a value from :c:type:`camss_params_block_type`.
Populated blocks must be placed consecutively in the data member, and the
combined size of all populated blocks must be set in the data_size member of
:c:type:`v4l2_isp_params_buffer`.

The set of supported blocks depends on the CAMSS engine consuming the buffer.
The block definitions themselves are shared across CAMSS ISPs; currently the
Offline Processing Engine (OPE) is the only engine consuming them, through its
``ope_params`` metadata output video node. Additional engines and blocks may be
added to this format in the future.

A block carrying ``V4L2_ISP_PARAMS_FL_BLOCK_DISABLE`` bypasses the
corresponding hardware module. Any other block updates the module's
configuration and leaves it enabled. Blocks omitted from a buffer keep their
previously programmed configuration and enable state; until userspace submits
a block for a given module, the driver's default configuration applies.

OPE processing pipeline
=======================

The OPE is a memory-to-memory engine that reads a Bayer RAW frame from its
``ope_input`` node and writes a YUV frame to its ``ope_disp_output`` node. The
parameter blocks configure fixed-function hardware modules that the frame data
passes through, in the following order:

.. code-block:: none

	 Bayer RAW in            RGB domain                      YUV domain      YUV out
	 (8/10-bit)   +------+   +------+   +------+   +------+   +------+   +------+   (NV12/NV16/
	  ----------> |  WB  |-->| DEMO |-->|  CC  |-->| GLUT |-->|CHROMA|-->| DSC  |----------->
	              +------+   +------+   +------+   +------+   |ENHAN |   +------+    NV24/GREY)
	                                                         +------+

The stages, and the parameter block that configures each one, are:

.. flat-table:: OPE pipeline stages
    :header-rows: 1
    :stub-columns: 0

    * - Stage
      - Module
      - Parameter block
      - Function
    * - WB
      - CLC_WB
      - :c:type:`camss_params_wb_gain`
      - Per-channel white balance gains and black-level / pedestal offsets,
        applied to the raw Bayer data.
    * - DEMO
      - CLC_DEMO
      - *(none)*
      - Demosaic: reconstructs a full-resolution RGB image from the Bayer
        mosaic. Always enabled; not configurable through this format.
    * - CC
      - CLC_CC
      - :c:type:`camss_params_color_correct`
      - Color correction matrix applied in the RGB domain.
    * - GLUT
      - CLC_GLUT
      - :c:type:`camss_params_gamma`
      - Per-channel gamma correction curves (256-entry LUTs).
    * - CHROMA_ENHAN
      - CLC_CHROMA_ENHAN
      - :c:type:`camss_params_chroma_enhan`
      - RGB to YUV color transfer matrix.
    * - DSC
      - Downscaler
      - *(none)*
      - Chroma (and, when required, luma) downscaling to produce the requested
        YUV subsampling, e.g. YUV 4:4:4 to 4:2:2 (NV16) or 4:2:0 (NV12).

The input node accepts 8-bit and 10-bit packed Bayer RAW formats (for example
``V4L2_PIX_FMT_SRGGB8`` and ``V4L2_PIX_FMT_SRGGB10P``). The output node
produces semi-planar YUV (``V4L2_PIX_FMT_NV12`` / ``NV21`` / ``NV16`` /
``NV61`` / ``NV24`` / ``NV42``) or luma-only ``V4L2_PIX_FMT_GREY``. A
configurable stage is bypassed while its block carries
``V4L2_ISP_PARAMS_FL_BLOCK_DISABLE``.

The following example populates an OPE parameters buffer with a white balance
and a gamma correction block:

.. code-block:: c

	struct v4l2_isp_params_buffer *params =
		(struct v4l2_isp_params_buffer *)buffer;

	params->version = V4L2_ISP_PARAMS_VERSION_V1;
	params->data_size = sizeof(struct camss_params_wb_gain) +
			    sizeof(struct camss_params_gamma);

	__u8 *data = params->data;

	struct camss_params_wb_gain *wb =
		(struct camss_params_wb_gain *)data;

	memset(wb, 0, sizeof(*wb));
	wb->header.type = CAMSS_PARAMS_WB_GAIN;
	wb->header.flags = V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	wb->header.size = sizeof(struct camss_params_wb_gain);

	/* Unity gain on all three channels (15uQ10, 1024 = 1.0) */
	wb->g_gain = 1024;
	wb->b_gain = 1024;
	wb->r_gain = 1024;

	data += sizeof(struct camss_params_wb_gain);

	struct camss_params_gamma *gamma =
		(struct camss_params_gamma *)data;

	memset(gamma, 0, sizeof(*gamma));
	gamma->header.type = CAMSS_PARAMS_GAMMA;
	gamma->header.flags = V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	gamma->header.size = sizeof(struct camss_params_gamma);

	/* Identity curve (pass-through, gamma 1.0) */
	for (unsigned int i = 0; i < CAMSS_GAMMA_LUT_SIZE; i++)
		gamma->glut[i] = gamma->blut[i] = gamma->rlut[i] = i;

Qualcomm CAMSS ISP uAPI data types
==================================

.. kernel-doc:: include/uapi/linux/qcom-camss-config.h
