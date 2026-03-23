// SPDX-License-Identifier: GPL-2.0
/*
 * camss-ope.c
 *
 * Qualcomm MSM Camera Subsystem - Offline Processing Engine
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * This driver provides a minimal implementation for the Qualcomm Offline
 * Processing Engine (OPE). OPE is a memory-to-memory hardware block
 * designed for image processing on a source frame. Typically, the input
 * frame originates from the SoC CSI capture path, though not limited to.
 *
 * The hardware architecture consists of Fetch Engines and Write Engines,
 * connected through intermediate pipeline modules:
 *   [FETCH ENGINES] => [Pipeline Modules] => [WRITE ENGINES]
 *
 * Current Configuration:
 *     Fetch Engine: One fetch engine is used for Bayer frame input.
 *     Write Engines: Two display write engines for Y and UV planes output.
 *
 * Only a subset of the pipeline modules are enabled:
 *   CLC_WB: White balance for channel gain configuration
 *   CLC_DEMO: Demosaic for Bayer to RGB conversion
 *   CLC_CHROMA_ENHAN: for RGB to YUV conversion
 *   CLC_DOWNSCALE*: Downscaling for UV (YUV444 -> YUV422/YUV420) and YUV planes
 *
 * Default configuration values are based on public standards such as BT.601.
 *
 * Processing Model:
 * OPE processes frames in stripes of up to 336 pixels. Therefore, frames must
 * be split into stripes for processing. Each stripe is configured after the
 * previous one has been acquired (double buffered registers). To minimize
 * inter-stripe latency, the stripe configurations are generated ahead of time.
 *
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_clock.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define MEM2MEM_NAME		"qcom-camss-ope"

/* TOP registers */
#define OPE_TOP_HW_VERSION						0x000
#define		OPE_TOP_HW_VERSION_STEP		GENMASK(15, 0)
#define		OPE_TOP_HW_VERSION_REV		GENMASK(27, 16)
#define		OPE_TOP_HW_VERSION_GEN		GENMASK(31, 28)
#define OPE_TOP_RESET_CMD						0x004
#define		OPE_TOP_RESET_CMD_HW		BIT(0)
#define		OPE_TOP_RESET_CMD_SW		BIT(1)
#define		OPE_TOP_CORE_CFG					0x010
#define OPE_TOP_IRQ_STATUS						0x014
#define OPE_TOP_IRQ_MASK						0x018
#define		OPE_TOP_IRQ_STATUS_RST_DONE	BIT(0)
#define		OPE_TOP_IRQ_STATUS_WE		BIT(1)
#define		OPE_TOP_IRQ_STATUS_FE		BIT(2)
#define		OPE_TOP_IRQ_STATUS_VIOL		BIT(3)
#define		OPE_TOP_IRQ_STATUS_IDLE		BIT(4)
#define OPE_TOP_IRQ_CLEAR						0x01c
#define OPE_TOP_IRQ_SET							0x020
#define OPE_TOP_IRQ_CMD							0x024
#define		OPE_TOP_IRQ_CMD_CLEAR		BIT(0)
#define		OPE_TOP_IRQ_CMD_SET		BIT(4)
#define OPE_TOP_VIOLATION_STATUS					0x028
#define OPE_TOP_DEBUG(i)						(0x0a0 + (i) * 4)
#define OPE_TOP_DEBUG_CFG						0x0dc

/* Fetch engines */
#define OPE_BUS_RD_INPUT_IF_IRQ_MASK					0x00c
#define OPE_BUS_RD_INPUT_IF_IRQ_CLEAR					0x010
#define OPE_BUS_RD_INPUT_IF_IRQ_CMD					0x014
#define		OPE_BUS_RD_INPUT_IF_IRQ_CMD_CLEAR	BIT(0)
#define		OPE_BUS_RD_INPUT_IF_IRQ_CMD_SET		BIT(4)
#define OPE_BUS_RD_INPUT_IF_IRQ_STATUS					0x018
#define		OPE_BUS_RD_INPUT_IF_IRQ_STATUS_RST_DONE	BIT(0)
#define		OPE_BUS_RD_INPUT_IF_IRQ_STATUS_RUP_DONE	BIT(1)
#define		OPE_BUS_RD_INPUT_IF_IRQ_STATUS_BUF_DONE	BIT(2)
#define OPE_BUS_RD_INPUT_IF_CMD						0x01c
#define		OPE_BUS_RD_INPUT_IF_CMD_GO_CMD		BIT(0)
#define OPE_BUS_RD_CLIENT_0_CORE_CFG					0x050
#define		OPE_BUS_RD_CLIENT_0_CORE_CFG_EN		BIT(0)
#define OPE_BUS_RD_CLIENT_0_CCIF_META_DATA				0x054
#define		OPE_BUS_RD_CLIENT_0_CCIF_MD_PIX_PATTERN	GENMASK(7, 2)
#define OPE_BUS_RD_CLIENT_0_ADDR_IMAGE						0x058
#define OPE_BUS_RD_CLIENT_0_RD_BUFFER_SIZE				0x05c
#define OPE_BUS_RD_CLIENT_0_RD_STRIDE					0x060
#define OPE_BUS_RD_CLIENT_0_UNPACK_CFG_0				0x064

/* Write engines */
#define OPE_BUS_WR_INPUT_IF_IRQ_MASK_0					0x018
#define OPE_BUS_WR_INPUT_IF_IRQ_MASK_1					0x01c
#define OPE_BUS_WR_INPUT_IF_IRQ_CLEAR_0					0x020
#define OPE_BUS_WR_INPUT_IF_IRQ_CLEAR_1					0x024
#define OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0				0x028
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_RUP_DONE	BIT(0)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_BUF_DONE	BIT(8)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_CONS_VIOL	BIT(28)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_VIOL		BIT(30)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_IMG_SZ_VIOL	BIT(31)
#define OPE_BUS_WR_INPUT_IF_IRQ_STATUS_1				0x02c
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_1_CLIENT_DONE(c)	BIT(0 + (c))
#define OPE_BUS_WR_INPUT_IF_IRQ_CMD					0x030
#define		OPE_BUS_WR_INPUT_IF_IRQ_CMD_CLEAR		BIT(0)
#define		OPE_BUS_WR_INPUT_IF_IRQ_CMD_SET			BIT(1)
#define OPE_BUS_WR_VIOLATION_STATUS					0x064
#define OPE_BUS_WR_IMAGE_SIZE_VIOLATION_STATUS				0x070
#define OPE_BUS_WR_CLIENT_CFG(c)					(0x200 + (c) * 0x100)
#define		OPE_BUS_WR_CLIENT_CFG_EN			BIT(0)
#define		OPE_BUS_WR_CLIENT_CFG_AUTORECOVER		BIT(4)
#define OPE_BUS_WR_CLIENT_ADDR_IMAGE(c)					(0x204 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_IMAGE_CFG_0(c)				(0x20c + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_IMAGE_CFG_1(c)				(0x210 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_IMAGE_CFG_2(c)				(0x214 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_PACKER_CFG(c)					(0x218 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_ADDR_FRAME_HEADER(c)				(0x220 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_MAX	8

/* Pipeline modules */
#define OPE_PP_CLC_WB_GAIN_MODULE_CFG					(0x200 + 0x60)
#define		OPE_PP_CLC_WB_GAIN_MODULE_CFG_EN	BIT(0)
#define OPE_PP_CLC_WB_GAIN_WB_CFG(ch)					(0x200 + 0x68 + 4 * (ch))

#define OPE_PP_CLC_DOWNSCALE_MN_DS_C_PRE_BASE				0x1c00
#define OPE_PP_CLC_DOWNSCALE_MN_DS_Y_DISP_BASE				0x3000
#define OPE_PP_CLC_DOWNSCALE_MN_DS_C_DISP_BASE				0x3200
#define OPE_PP_CLC_DOWNSCALE_MN_CFG(ds)					((ds) + 0x60)
#define		OPE_PP_CLC_DOWNSCALE_MN_CFG_EN			BIT(0)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_CFG(ds)				((ds) + 0x64)
#define		OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_H_SCALE_EN	BIT(9)
#define		OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_V_SCALE_EN	BIT(10)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_IMAGE_SIZE_CFG(ds)			((ds) + 0x68)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_MN_H_CFG(ds)				((ds) + 0x6c)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_MN_V_CFG(ds)				((ds) + 0x74)

#define OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG				(0x1200 + 0x60)
#define		OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG_EN		BIT(0)
#define OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0				(0x1200 + 0x68)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V0		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V1		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1				(0x1200 + 0x6c)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1_K		GENMASK(31, 23)
#define OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2				(0x1200 + 0x70)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2_V2		GENMASK(11, 0)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG				(0x1200 + 0x74)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AP		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AM		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG				(0x1200 + 0x78)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BP		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BM		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG				(0x1200 + 0x7C)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CP		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CM		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG				(0x1200 + 0x80)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DP		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DM		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0				(0x1200 + 0x84)
#define		OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0_KCB	GENMASK(31, 21)
#define OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1				(0x1200 + 0x88)
#define		OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1_KCR	GENMASK(31, 21)

#define OPE_STRIPE_MAX_W	336
#define OPE_STRIPE_MAX_H	8192
#define OPE_STRIPE_MIN_W	16
#define OPE_STRIPE_MIN_H	OPE_STRIPE_MIN_W
#define OPE_MAX_STRIPE		16
#define OPE_ALIGN_H		1
#define OPE_ALIGN_W		1
#define OPE_MIN_W		24
#define OPE_MIN_H		16
#define OPE_MAX_W		(OPE_STRIPE_MAX_W * OPE_MAX_STRIPE)
#define OPE_MAX_H		OPE_STRIPE_MAX_H

#define MEM2MEM_CAPTURE		BIT(0)
#define MEM2MEM_OUTPUT		BIT(1)

#define OPE_RESET_TIMEOUT_MS	100

/* Expected framerate for power scaling */
#define DEFAULT_FRAMERATE 60

/* Downscaler helpers */
#define Q21(v) (((uint64_t)(v)) << 21)
#define DS_Q21(n, d) ((uint32_t)(((uint64_t)(n) << 21) / (d)))
#define DS_RESOLUTION(input, output) \
	(((output) * 128 <= (input)) ? 0x0 : \
	((output) * 16 <= (input))  ? 0x1 : \
	((output) * 8  <= (input))  ? 0x2 : 0x3)
#define DS_OUTPUT_PIX(input, phase_init, phase_step) \
		((Q21(input) - (phase_init)) / (phase_step))

enum ope_downscaler {
	OPE_DS_C_PRE,
	OPE_DS_C_DISP,
	OPE_DS_Y_DISP,
	OPE_DS_MAX,
};

static const u32 ope_ds_base[OPE_DS_MAX] = { OPE_PP_CLC_DOWNSCALE_MN_DS_C_PRE_BASE,
					     OPE_PP_CLC_DOWNSCALE_MN_DS_C_DISP_BASE,
					     OPE_PP_CLC_DOWNSCALE_MN_DS_Y_DISP_BASE };

enum ope_wr_client {
	OPE_WR_CLIENT_VID_Y,
	OPE_WR_CLIENT_VID_C,
	OPE_WR_CLIENT_DISP_Y,
	OPE_WR_CLIENT_DISP_C,
	OPE_WR_CLIENT_ARGB,
	OPE_WR_CLIENT_MAX
};

enum ope_pixel_pattern {
	OPE_PIXEL_PATTERN_RGRGRG,
	OPE_PIXEL_PATTERN_GRGRGR,
	OPE_PIXEL_PATTERN_BGBGBG,
	OPE_PIXEL_PATTERN_GBGBGB,
	OPE_PIXEL_PATTERN_YCBYCR,
	OPE_PIXEL_PATTERN_YCRYCB,
	OPE_PIXEL_PATTERN_CBYCRY,
	OPE_PIXEL_PATTERN_CRYCBY
};

enum ope_stripe_location {
	OPE_STRIPE_LOCATION_FULL,
	OPE_STRIPE_LOCATION_LEFT,
	OPE_STRIPE_LOCATION_RIGHT,
	OPE_STRIPE_LOCATION_MIDDLE
};

enum ope_unpacker_format {
	OPE_UNPACKER_FMT_PLAIN_128,
	OPE_UNPACKER_FMT_PLAIN_8,
	OPE_UNPACKER_FMT_PLAIN_16_10BPP,
	OPE_UNPACKER_FMT_PLAIN_16_12BPP,
	OPE_UNPACKER_FMT_PLAIN_16_14BPP,
	OPE_UNPACKER_FMT_PLAIN_32_20BPP,
	OPE_UNPACKER_FMT_ARGB_16_10BPP,
	OPE_UNPACKER_FMT_ARGB_16_12BPP,
	OPE_UNPACKER_FMT_ARGB_16_14BPP,
	OPE_UNPACKER_FMT_PLAIN_32,
	OPE_UNPACKER_FMT_PLAIN_64,
	OPE_UNPACKER_FMT_TP_10,
	OPE_UNPACKER_FMT_MIPI_8,
	OPE_UNPACKER_FMT_MIPI_10,
	OPE_UNPACKER_FMT_MIPI_12,
	OPE_UNPACKER_FMT_MIPI_14,
	OPE_UNPACKER_FMT_PLAIN_16_16BPP,
	OPE_UNPACKER_FMT_PLAIN_128_ODD_EVEN,
	OPE_UNPACKER_FMT_PLAIN_8_ODD_EVEN
};

enum ope_packer_format {
	OPE_PACKER_FMT_PLAIN_128,
	OPE_PACKER_FMT_PLAIN_8,
	OPE_PACKER_FMT_PLAIN_8_ODD_EVEN,
	OPE_PACKER_FMT_PLAIN_8_10BPP,
	OPE_PACKER_FMT_PLAIN_8_10BPP_ODD_EVEN,
	OPE_PACKER_FMT_PLAIN_16_10BPP,
	OPE_PACKER_FMT_PLAIN_16_12BPP,
	OPE_PACKER_FMT_PLAIN_16_14BPP,
	OPE_PACKER_FMT_PLAIN_16_16BPP,
	OPE_PACKER_FMT_PLAIN_32,
	OPE_PACKER_FMT_PLAIN_64,
	OPE_PACKER_FMT_TP_10,
	OPE_PACKER_FMT_MIPI_10,
	OPE_PACKER_FMT_MIPI_12
};

struct ope_fmt {
	u32 fourcc;
	unsigned int types;
	enum ope_pixel_pattern pattern;
	enum ope_unpacker_format unpacker_format;
	enum ope_packer_format packer_format;
	unsigned int depth;
	unsigned int align; /* pix alignment = 2^align */
};

static const struct ope_fmt formats[] = { /* TODO: add multi-planes formats */
	/* Output - Bayer MIPI 10 */
	{ V4L2_PIX_FMT_SBGGR10P, MEM2MEM_OUTPUT, OPE_PIXEL_PATTERN_BGBGBG,
	  OPE_UNPACKER_FMT_MIPI_10, OPE_PACKER_FMT_MIPI_10, 10, 2 },
	{ V4L2_PIX_FMT_SGBRG10P, MEM2MEM_OUTPUT, OPE_PIXEL_PATTERN_GBGBGB,
	  OPE_UNPACKER_FMT_MIPI_10, OPE_PACKER_FMT_MIPI_10, 10, 2 },
	{ V4L2_PIX_FMT_SGRBG10P, MEM2MEM_OUTPUT, OPE_PIXEL_PATTERN_GRGRGR,
	 OPE_UNPACKER_FMT_MIPI_10, OPE_PACKER_FMT_MIPI_10, 10, 2 },
	{ V4L2_PIX_FMT_SRGGB10P, MEM2MEM_OUTPUT, OPE_PIXEL_PATTERN_RGRGRG,
	  OPE_UNPACKER_FMT_MIPI_10, OPE_PACKER_FMT_MIPI_10, 10, 2 },
	/* Output - Bayer MIPI/Plain 8 */
	{ V4L2_PIX_FMT_SRGGB8, MEM2MEM_OUTPUT, OPE_PIXEL_PATTERN_RGRGRG,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8, 8, 0 },
	{ V4L2_PIX_FMT_SBGGR8, MEM2MEM_OUTPUT, OPE_PIXEL_PATTERN_BGBGBG,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8, 8, 0 },
	{ V4L2_PIX_FMT_SGBRG8, MEM2MEM_OUTPUT, OPE_PIXEL_PATTERN_GBGBGB,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8, 8, 0 },
	{ V4L2_PIX_FMT_SGRBG8, MEM2MEM_OUTPUT, OPE_PIXEL_PATTERN_GRGRGR,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8, 8, 0 },
	/* Capture - YUV 8-bit per component */
	{ V4L2_PIX_FMT_NV24, MEM2MEM_CAPTURE, OPE_PIXEL_PATTERN_YCBYCR,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8, 24, 0 },
	{ V4L2_PIX_FMT_NV42, MEM2MEM_CAPTURE, OPE_PIXEL_PATTERN_YCRYCB,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8_ODD_EVEN, 24, 0 },
	{ V4L2_PIX_FMT_NV16, MEM2MEM_CAPTURE, OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8, 16, 1 },
	{ V4L2_PIX_FMT_NV61, MEM2MEM_CAPTURE, OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8_ODD_EVEN, 16, 1 },
	{ V4L2_PIX_FMT_NV12, MEM2MEM_CAPTURE, OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8, 12, 1 },
	{ V4L2_PIX_FMT_NV21, MEM2MEM_CAPTURE, OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8_ODD_EVEN, 12, 1 },
	/* Capture - Greyscale 8-bit */
	{ V4L2_PIX_FMT_GREY, MEM2MEM_CAPTURE, OPE_PIXEL_PATTERN_RGRGRG,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8, 8, 0 },
};

#define OPE_NUM_FORMATS ARRAY_SIZE(formats)

#define OPE_WB(n, d) (((n) << 10) / (d))

/* Per-queue, driver-specific private data */
struct ope_q_data {
	unsigned int width;
	unsigned int height;
	unsigned int bytesperline;
	unsigned int sizeimage;
	const struct ope_fmt *fmt;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quant;
	unsigned int sequence;
};

struct ope_dev {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device vfd;
	struct media_device mdev;
	struct v4l2_m2m_dev *m2m_dev;

	void __iomem *base;
	void __iomem *base_rd;
	void __iomem *base_wr;
	void __iomem *base_pp;

	struct completion reset_complete;

	struct icc_path *icc_data;
	struct icc_path *icc_config;

	struct mutex mutex;
	struct list_head ctx_list;
	void *context;
};

struct ope_dsc_config {
	u32 input_width;
	u32 input_height;
	u32 output_width;
	u32 output_height;
	u32 phase_step_h;
	u32 phase_step_v;
};

struct ope_stripe {
	struct {
		dma_addr_t addr;
		u32 width;
		u32 height;
		u32 stride;
		enum ope_stripe_location location;
		enum ope_pixel_pattern pattern;
		enum ope_unpacker_format format;
	} src;
	struct {
		dma_addr_t addr;
		u32 width;
		u32 height;
		u32 stride;
		u32 x_init;
		enum ope_packer_format format;
		bool enabled;
	} dst[OPE_WR_CLIENT_MAX];
	struct ope_dsc_config dsc[OPE_DS_MAX];
};

struct ope_ctx {
	struct v4l2_fh fh;
	struct ope_dev *ope;

	/* Processing mode */
	int			mode;
	u8			alpha_component;
	u8			rotation;
	unsigned int framerate;

	enum v4l2_colorspace	colorspace;
	enum v4l2_xfer_func	xfer_func;

	/* Source and destination queue data */
	struct ope_q_data   q_data_src;
	struct ope_q_data   q_data_dst;

	u8 current_stripe;
	struct ope_stripe stripe[OPE_MAX_STRIPE];

	bool started;

	struct list_head list;
};

struct ope_freq_tbl {
	unsigned int load;
	unsigned long freq;
};

static inline char *print_fourcc(u32 fmt)
{
	static char code[5];

	code[0] = (unsigned char)(fmt & 0xff);
	code[1] = (unsigned char)((fmt >> 8) & 0xff);
	code[2] = (unsigned char)((fmt >> 16) & 0xff);
	code[3] = (unsigned char)((fmt >> 24) & 0xff);
	code[4] = '\0';

	return code;
}

static inline enum ope_stripe_location ope_stripe_location(unsigned int index,
							   unsigned int count)
{
	if (count == 1)
		return OPE_STRIPE_LOCATION_FULL;
	if (index == 0)
		return OPE_STRIPE_LOCATION_LEFT;
	if (index == (count - 1))
		return OPE_STRIPE_LOCATION_RIGHT;

	return OPE_STRIPE_LOCATION_MIDDLE;
}

static inline bool ope_stripe_is_last(struct ope_stripe *stripe)
{
	if (!stripe)
		return false;

	if (stripe->src.location == OPE_STRIPE_LOCATION_RIGHT ||
	    stripe->src.location == OPE_STRIPE_LOCATION_FULL)
		return true;

	return false;
}

static inline struct ope_stripe *ope_get_stripe(struct ope_ctx *ctx, unsigned int index)
{
	return &ctx->stripe[index];
}

static inline struct ope_stripe *ope_current_stripe(struct ope_ctx *ctx)
{
	if (!ctx)
		return NULL;

	if (ctx->current_stripe >= OPE_MAX_STRIPE)
		return NULL;

	return ope_get_stripe(ctx, ctx->current_stripe);
}

static inline unsigned int ope_stripe_index(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	return stripe - &ctx->stripe[0];
}

static inline struct ope_stripe *ope_prev_stripe(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	unsigned int index = ope_stripe_index(ctx, stripe);

	return index ? ope_get_stripe(ctx, index - 1) : NULL;
}

static inline struct ope_q_data *get_q_data(struct ope_ctx *ctx, enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->q_data_src;
	else
		return &ctx->q_data_dst;
}

static inline unsigned long __q_data_pixclk(struct ope_q_data *q, unsigned int fps)
{
	return (unsigned long)q->width * q->height * fps;
}

static inline unsigned int __q_data_load_avg(struct ope_q_data *q, unsigned int fps)
{
	/* Data load in kBps, calculated from pixel clock and bits per pixel */
	return mult_frac(__q_data_pixclk(q, fps), q->fmt->depth, 1000) / 8;
}

static inline unsigned int __q_data_load_peak(struct ope_q_data *q, unsigned int fps)
{
	return __q_data_load_avg(q, fps) * 2;
}

static inline unsigned int __q_data_load_config(struct ope_q_data *q, unsigned int fps)
{
	unsigned int stripe_count = q->width / OPE_STRIPE_MAX_W + 1;
	unsigned int stripe_load = 50 * 4 * fps; /* about 50 x 32-bit registers to configure */

	/* Return config load in kBps */
	return mult_frac(stripe_count, stripe_load, 1000);
}

static inline struct ope_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct ope_ctx, fh);
}

static inline u32 ope_read(struct ope_dev *ope, u32 reg)
{
	return readl(ope->base + reg);
}

static inline void ope_write(struct ope_dev *ope, u32 reg, u32 value)
{
	writel(value, ope->base + reg);
}

static inline u32 ope_read_wr(struct ope_dev *ope, u32 reg)
{
	return readl_relaxed(ope->base_wr + reg);
}

static inline void ope_write_wr(struct ope_dev *ope, u32 reg, u32 value)
{
	writel_relaxed(value, ope->base_wr + reg);
}

static inline u32 ope_read_rd(struct ope_dev *ope, u32 reg)
{
	return readl_relaxed(ope->base_rd + reg);
}

static inline void ope_write_rd(struct ope_dev *ope, u32 reg, u32 value)
{
	writel_relaxed(value, ope->base_rd + reg);
}

static inline u32 ope_read_pp(struct ope_dev *ope, u32 reg)
{
	return readl_relaxed(ope->base_pp + reg);
}

static inline void ope_write_pp(struct ope_dev *ope, u32 reg, u32 value)
{
	writel_relaxed(value, ope->base_pp + reg);
}

static inline void ope_start(struct ope_dev *ope)
{
	wmb(); /* Ensure the next write occurs only after all prior normal memory accesses */
	ope_write_rd(ope, OPE_BUS_RD_INPUT_IF_CMD, OPE_BUS_RD_INPUT_IF_CMD_GO_CMD);
}

static bool ope_pix_fmt_is_yuv(u32 fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV24:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV42:
	case V4L2_PIX_FMT_GREY:
		return true;
	default:
		return false;
	}
}

static const struct ope_fmt *find_format(unsigned int pixelformat)
{
	const struct ope_fmt *fmt;
	unsigned int i;

	for (i = 0; i < OPE_NUM_FORMATS; i++) {
		fmt = &formats[i];
		if (fmt->fourcc == pixelformat)
			break;
	}

	if (i == OPE_NUM_FORMATS)
		return NULL;

	return &formats[i];
}

static inline void ope_dbg_print_stripe(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	struct ope_dev *ope = ctx->ope;
	int i;

	dev_dbg(ope->dev, "S%u/FE0: addr=%pad;W=%ub;H=%u;stride=%u;loc=%u;pattern=%u;fmt=%u\n",
		ope_stripe_index(ctx, stripe), &stripe->src.addr, stripe->src.width,
		stripe->src.height, stripe->src.stride, stripe->src.location, stripe->src.pattern,
		stripe->src.format);

	for (i = 0; i < OPE_DS_MAX; i++) {
		struct ope_dsc_config *c = &stripe->dsc[i];

		dev_dbg(ope->dev, "S%u/DSC%d: %ux%u => %ux%u\n",
			ope_stripe_index(ctx, stripe), i, c->input_width, c->input_height,
			c->output_width, c->output_height);
	}

	for (i = 0; i < OPE_WR_CLIENT_MAX; i++) {
		if (!stripe->dst[i].enabled)
			continue;

		dev_dbg(ope->dev,
			"S%u/WE%d: addr=%pad;X=%u;W=%upx;H=%u;stride=%u;fmt=%u\n",
			ope_stripe_index(ctx, stripe), i, &stripe->dst[i].addr,
			stripe->dst[i].x_init, stripe->dst[i].width, stripe->dst[i].height,
			stripe->dst[i].stride, stripe->dst[i].format);
	}
}

static void ope_gen_stripe_argb_dst(struct ope_ctx *ctx, struct ope_stripe *stripe, dma_addr_t dst)
{
	unsigned int index = ope_stripe_index(ctx, stripe);
	unsigned int img_width = ctx->q_data_dst.width;
	unsigned int width, height;
	dma_addr_t addr;

	/* This is GBRA64 format (le16)G (le16)B (le16)R (le16)A */

	stripe->dst[OPE_WR_CLIENT_ARGB].enabled = true;

	width = stripe->src.width;
	height = stripe->src.height;

	if (!index) {
		addr = dst;
	} else {
		struct ope_stripe *prev = ope_get_stripe(ctx, index - 1);

		addr = prev->dst[OPE_WR_CLIENT_ARGB].addr + prev->dst[OPE_WR_CLIENT_ARGB].width * 8;
	}

	stripe->dst[OPE_WR_CLIENT_ARGB].addr = addr;
	stripe->dst[OPE_WR_CLIENT_ARGB].x_init = 0;
	stripe->dst[OPE_WR_CLIENT_ARGB].width = width;
	stripe->dst[OPE_WR_CLIENT_ARGB].height = height;
	stripe->dst[OPE_WR_CLIENT_ARGB].stride = img_width * 8;
	stripe->dst[OPE_WR_CLIENT_ARGB].format = OPE_PACKER_FMT_PLAIN_64;
}

static void ope_gen_stripe_yuv_dst(struct ope_ctx *ctx, struct ope_stripe *stripe, dma_addr_t dst)
{
	struct ope_stripe *prev = ope_prev_stripe(ctx, stripe);
	unsigned int img_width = ctx->q_data_dst.width;
	unsigned int img_height = ctx->q_data_dst.height;
	unsigned int width, height;
	u32 x_init = 0;

	stripe->dst[OPE_WR_CLIENT_DISP_Y].enabled = true;
	stripe->dst[OPE_WR_CLIENT_DISP_C].enabled = true;

	/* Y */
	width = stripe->dsc[OPE_DS_Y_DISP].output_width;
	height = stripe->dsc[OPE_DS_Y_DISP].output_height;

	if (prev)
		x_init = prev->dst[OPE_WR_CLIENT_DISP_Y].x_init +
			 prev->dst[OPE_WR_CLIENT_DISP_Y].width;

	stripe->dst[OPE_WR_CLIENT_DISP_Y].addr = dst;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].x_init = x_init;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].width = width;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].height = height;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].stride = img_width;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].format = OPE_PACKER_FMT_PLAIN_8;

	/* UV */
	width = stripe->dsc[OPE_DS_C_DISP].output_width;
	height = stripe->dsc[OPE_DS_C_DISP].output_height;

	if (prev)
		x_init = prev->dst[OPE_WR_CLIENT_DISP_C].x_init +
			 prev->dst[OPE_WR_CLIENT_DISP_C].width;

	stripe->dst[OPE_WR_CLIENT_DISP_C].addr = dst + img_width * img_height;
	stripe->dst[OPE_WR_CLIENT_DISP_C].x_init = x_init;
	stripe->dst[OPE_WR_CLIENT_DISP_C].format = ctx->q_data_dst.fmt->packer_format;
	stripe->dst[OPE_WR_CLIENT_DISP_C].width = width * 2;
	stripe->dst[OPE_WR_CLIENT_DISP_C].height = height;

	switch (ctx->q_data_dst.fmt->fourcc) {
	case V4L2_PIX_FMT_NV42:
	case V4L2_PIX_FMT_NV24: /* YUV 4:4:4 */
		stripe->dst[OPE_WR_CLIENT_DISP_C].stride = img_width * 2;
		break;
	case V4L2_PIX_FMT_GREY: /* No UV */
		stripe->dst[OPE_WR_CLIENT_DISP_C].enabled = false;
		break;
	default:
		stripe->dst[OPE_WR_CLIENT_DISP_C].stride = img_width;
	}
}

static void ope_gen_stripe_dsc(struct ope_ctx *ctx, struct ope_stripe *stripe,
			       unsigned int h_scale, unsigned int v_scale)
{
	struct ope_dsc_config *dsc_c, *dsc_y;

	dsc_c = &stripe->dsc[OPE_DS_C_DISP];
	dsc_y = &stripe->dsc[OPE_DS_Y_DISP];

	dsc_c->phase_step_h = dsc_y->phase_step_h = h_scale;
	dsc_c->phase_step_v = dsc_y->phase_step_v = v_scale;

	dsc_c->input_width = stripe->dsc[OPE_DS_C_PRE].output_width;
	dsc_c->input_height = stripe->dsc[OPE_DS_C_PRE].output_height;

	dsc_y->input_width = stripe->src.width;
	dsc_y->input_height = stripe->src.height;

	dsc_c->output_width = DS_OUTPUT_PIX(dsc_c->input_width, 0, h_scale);
	dsc_c->output_height = DS_OUTPUT_PIX(dsc_c->input_height, 0, v_scale);

	dsc_y->output_width = DS_OUTPUT_PIX(dsc_y->input_width, 0, h_scale);
	dsc_y->output_height = DS_OUTPUT_PIX(dsc_y->input_height, 0, v_scale);

	/* Adjust initial phase ? */
}

static void ope_gen_stripe_chroma_dsc(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	struct ope_dsc_config *dsc;

	dsc = &stripe->dsc[OPE_DS_C_PRE];

	dsc->input_width = stripe->src.width;
	dsc->input_height = stripe->src.height;

	switch (ctx->q_data_dst.fmt->fourcc) {
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_NV16:
		dsc->output_width = dsc->input_width / 2;
		dsc->output_height = dsc->input_height;
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		dsc->output_width = dsc->input_width / 2;
		dsc->output_height = dsc->input_height / 2;
		break;
	default:
		dsc->output_width = dsc->input_width;
		dsc->output_height = dsc->input_height;
	}

	dsc->phase_step_h = DS_Q21(dsc->input_width, dsc->output_width);
	dsc->phase_step_v = DS_Q21(dsc->input_height, dsc->output_height);
}

static void ope_gen_stripes(struct ope_ctx *ctx, dma_addr_t src, dma_addr_t dst)
{
	const struct ope_fmt *src_fmt = ctx->q_data_src.fmt;
	const struct ope_fmt *dst_fmt = ctx->q_data_dst.fmt;
	unsigned int num_stripes, width, i;
	unsigned int h_scale, v_scale;

	width = ctx->q_data_src.width;
	num_stripes = DIV_ROUND_UP(ctx->q_data_src.width, OPE_STRIPE_MAX_W);
	h_scale = DS_Q21(ctx->q_data_src.width, ctx->q_data_dst.width);
	v_scale = DS_Q21(ctx->q_data_src.height, ctx->q_data_dst.height);

	for (i = 0; i < num_stripes; i++) {
		struct ope_stripe *stripe = &ctx->stripe[i];

		/* Clear config */
		memset(stripe, 0, sizeof(*stripe));

		/* Fetch Engine */
		stripe->src.addr = src;
		stripe->src.width = width;
		stripe->src.height = ctx->q_data_src.height;
		stripe->src.stride = ctx->q_data_src.bytesperline;
		stripe->src.location = ope_stripe_location(i, num_stripes);
		stripe->src.pattern = src_fmt->pattern;
		stripe->src.format = src_fmt->unpacker_format;

		/* Ensure the last stripe will be large enough */
		if (width > OPE_STRIPE_MAX_W && width < (OPE_STRIPE_MAX_W + OPE_STRIPE_MIN_W))
			stripe->src.width -= OPE_STRIPE_MIN_W * 2;

		v4l_bound_align_image(&stripe->src.width, src_fmt->align,
				      OPE_STRIPE_MAX_W, src_fmt->align,
				      &stripe->src.height, OPE_STRIPE_MIN_H, OPE_STRIPE_MAX_H,
				      OPE_ALIGN_H, 0);

		width -= stripe->src.width;
		src += stripe->src.width * src_fmt->depth / 8;

		if (ope_pix_fmt_is_yuv(dst_fmt->fourcc)) {
			/* YUV Chroma downscaling */
			ope_gen_stripe_chroma_dsc(ctx, stripe);

			/* YUV downscaling */
			ope_gen_stripe_dsc(ctx, stripe, h_scale, v_scale);

			/* Write Engines */
			ope_gen_stripe_yuv_dst(ctx, stripe, dst);
		} else {
			ope_gen_stripe_argb_dst(ctx, stripe, dst);
		}

		/* Source width is in byte unit, not pixel */
		stripe->src.width = stripe->src.width * src_fmt->depth / 8;

		ope_dbg_print_stripe(ctx, stripe);
	}
}

static void ope_prog_rgb2yuv(struct ope_dev *ope)
{
	/* Default RGB to YUV - no special effect - CF BT.601 */
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V0, 0x04d) | /* R to Y  0.299  12sQ8 */
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V1, 0x096)); /* G to Y  0.587 12sQ8 */
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2_V2, 0x01d)); /* B to Y  0.114 12sQ8 */
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1_K, 0)); /* Y offset 0 9sQ0 */
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AP, 0x0e6) |
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AM, 0x0e6)); /* 0.886 12sQ8 (Cb) */
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BP, 0xfb3) |
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BM, 0xfb3)); /* -0.338 12sQ8 (Cb) */
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CP, 0xb3) |
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CM, 0xb3)); /* 0.701 12sQ8 (Cr) */
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DP, 0xfe3) |
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DM, 0xfe3)); /* -0.114 12sQ8 (Cr) */
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1_KCR, 128)); /* KCR 128 11s */
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0_KCB, 128)); /* KCB 128 11s */

	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG,
		     OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG_EN);
}

static void ope_prog_bayer2rgb(struct ope_dev *ope)
{
	/* Fixed Settings */
	ope_write_pp(ope, 0x860, 0x4001);
	ope_write_pp(ope, 0x868, 128);
	ope_write_pp(ope, 0x86c, 128 << 20);
	ope_write_pp(ope, 0x870, 102);
}

static void ope_prog_wb(struct ope_dev *ope)
{
	/* Default white balance config */
	u32 g_gain = OPE_WB(1, 1);
	u32 b_gain = OPE_WB(3, 2);
	u32 r_gain = OPE_WB(3, 2);

	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_WB_CFG(0), g_gain);
	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_WB_CFG(1), b_gain);
	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_WB_CFG(2), r_gain);

	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_MODULE_CFG, OPE_PP_CLC_WB_GAIN_MODULE_CFG_EN);
}

static void ope_prog_stripe(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	struct ope_dev *ope = ctx->ope;
	int i;

	dev_dbg(ope->dev, "Context %p - Programming S%u\n", ctx, ope_stripe_index(ctx, stripe));

	/* Fetch Engine */
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_UNPACK_CFG_0, stripe->src.format);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_RD_BUFFER_SIZE,
		     (stripe->src.width << 16) + stripe->src.height);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_ADDR_IMAGE, stripe->src.addr);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_RD_STRIDE, stripe->src.stride);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_CCIF_META_DATA,
		     FIELD_PREP(OPE_BUS_RD_CLIENT_0_CCIF_MD_PIX_PATTERN, stripe->src.pattern));
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_CORE_CFG, OPE_BUS_RD_CLIENT_0_CORE_CFG_EN);

	/* Write Engines */
	for (i = 0; i < OPE_WR_CLIENT_MAX; i++) {
		if (!stripe->dst[i].enabled) {
			ope_write_wr(ope, OPE_BUS_WR_CLIENT_CFG(i), 0);
			continue;
		}

		ope_write_wr(ope, OPE_BUS_WR_CLIENT_ADDR_IMAGE(i), stripe->dst[i].addr);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_IMAGE_CFG_0(i),
			     (stripe->dst[i].height << 16) + stripe->dst[i].width);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_IMAGE_CFG_1(i), stripe->dst[i].x_init);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_IMAGE_CFG_2(i), stripe->dst[i].stride);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_PACKER_CFG(i), stripe->dst[i].format);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_CFG(i),
			     OPE_BUS_WR_CLIENT_CFG_EN + OPE_BUS_WR_CLIENT_CFG_AUTORECOVER);
	}

	/* Downscalers */
	for (i = 0; i < OPE_DS_MAX; i++) {
		struct ope_dsc_config *dsc = &stripe->dsc[i];
		u32 base = ope_ds_base[i];
		u32 cfg = 0;

		if (dsc->input_width != dsc->output_width) {
			dsc->phase_step_h |= DS_RESOLUTION(dsc->input_width,
							   dsc->output_width) << 30;
			cfg |= OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_H_SCALE_EN;
		}

		if (dsc->input_height != dsc->output_height) {
			dsc->phase_step_v |= DS_RESOLUTION(dsc->input_height,
							   dsc->output_height) << 30;
			cfg |= OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_V_SCALE_EN;
		}

		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_CFG(base), cfg);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_IMAGE_SIZE_CFG(base),
			     ((dsc->input_width - 1) << 16) + dsc->input_height - 1);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_MN_H_CFG(base), dsc->phase_step_h);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_MN_V_CFG(base), dsc->phase_step_v);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_CFG(base),
			     cfg ? OPE_PP_CLC_DOWNSCALE_MN_CFG_EN : 0);
	}
}

/*
 * mem2mem callbacks
 */
static void ope_device_run(void *priv)
{
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	struct ope_ctx *ctx = priv;
	struct ope_dev *ope = ctx->ope;
	dma_addr_t src, dst;

	dev_dbg(ope->dev, "Start context %p", ctx);

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	if (!src_buf)
		return;

	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (!dst_buf)
		return;

	src = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	dst = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);

	/* Generate stripes from full frame */
	ope_gen_stripes(ctx, src, dst);

	if (priv != ope->context) {
		/* If context changed, reprogram the submodules */
		ope_prog_wb(ope);
		ope_prog_bayer2rgb(ope);
		ope_prog_rgb2yuv(ope);
		ope->context = priv;
	}

	/* Program the first stripe */
	ope_prog_stripe(ctx, &ctx->stripe[0]);

	/* Go! */
	ope_start(ope);
}

static void ope_job_done(struct ope_ctx *ctx, enum vb2_buffer_state vbstate)
{
	struct vb2_v4l2_buffer *src, *dst;

	if (!ctx)
		return;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

	if (dst && src)
		dst->vb2_buf.timestamp = src->vb2_buf.timestamp;

	if (src)
		v4l2_m2m_buf_done(src, vbstate);
	if (dst)
		v4l2_m2m_buf_done(dst, vbstate);

	v4l2_m2m_job_finish(ctx->ope->m2m_dev, ctx->fh.m2m_ctx);
}

static void ope_buf_done(struct ope_ctx *ctx)
{
	struct ope_stripe *stripe = ope_current_stripe(ctx);

	if (!ctx)
		return;

	dev_dbg(ctx->ope->dev, "Context %p Stripe %u done\n",
		ctx,  ope_stripe_index(ctx, stripe));

	if (ope_stripe_is_last(stripe)) {
		ctx->current_stripe = 0;
		ope_job_done(ctx, VB2_BUF_STATE_DONE);
	} else {
		ctx->current_stripe++;
		ope_start(ctx->ope);
	}
}

static void ope_job_abort(void *priv)
{
	struct ope_ctx *ctx = priv;

	/* reset to abort */
	ope_write(ctx->ope, OPE_TOP_RESET_CMD, OPE_TOP_RESET_CMD_SW);
}

static void ope_rup_done(struct ope_ctx *ctx)
{
	struct ope_stripe *stripe = ope_current_stripe(ctx);

	/* We can program next stripe (double buffered registers) */
	if (!ope_stripe_is_last(stripe))
		ope_prog_stripe(ctx, ++stripe);
}

/*
 * interrupt handler
 */
static void ope_fe_irq(struct ope_dev *ope)
{
	u32 status = ope_read_rd(ope, OPE_BUS_RD_INPUT_IF_IRQ_STATUS);

	ope_write_rd(ope, OPE_BUS_RD_INPUT_IF_IRQ_CLEAR, status);
	ope_write_rd(ope, OPE_BUS_RD_INPUT_IF_IRQ_CMD, OPE_BUS_RD_INPUT_IF_IRQ_CMD_CLEAR);

	/* Nothing to do */
}

static void ope_we_irq(struct ope_ctx *ctx)
{
	struct ope_dev *ope;
	u32 status0;

	if (!ctx) {
		pr_err("Instance released before the end of transaction\n");
		return;
	}

	ope = ctx->ope;

	status0 = ope_read_wr(ope, OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0);
	ope_write_wr(ope, OPE_BUS_WR_INPUT_IF_IRQ_CLEAR_0, status0);
	ope_write_wr(ope, OPE_BUS_WR_INPUT_IF_IRQ_CMD, OPE_BUS_WR_INPUT_IF_IRQ_CMD_CLEAR);

	if (status0 & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_CONS_VIOL) {
		dev_err_ratelimited(ope->dev, "Write Engine configuration violates constrains\n");
		ope_job_abort(ctx);
	}

	if (status0 & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_IMG_SZ_VIOL) {
		u32 status = ope_read_wr(ope, OPE_BUS_WR_IMAGE_SIZE_VIOLATION_STATUS);
		int i;

		for (i = 0; i < OPE_WR_CLIENT_MAX; i++) {
			if (BIT(i) & status)
				dev_err_ratelimited(ope->dev,
					 "Write Engine (WE%d) image size violation\n", i);
		}

		ope_job_abort(ctx);
	}

	if (status0 & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_VIOL) {
		dev_err_ratelimited(ope->dev, "Write Engine fatal violation\n");
		ope_job_abort(ctx);
	}

	if (status0 & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_RUP_DONE)
		ope_rup_done(ctx);
}

static irqreturn_t ope_irq(int irq, void *dev_id)
{
	struct ope_dev *ope = dev_id;
	struct ope_ctx *ctx = ope->m2m_dev ? v4l2_m2m_get_curr_priv(ope->m2m_dev) : NULL;
	u32 status;

	status = ope_read(ope, OPE_TOP_IRQ_STATUS);
	ope_write(ope, OPE_TOP_IRQ_CLEAR, status);
	ope_write(ope, OPE_TOP_IRQ_CMD, OPE_TOP_IRQ_CMD_CLEAR);

	if (status & OPE_TOP_IRQ_STATUS_RST_DONE) {
		ope_job_done(ctx, VB2_BUF_STATE_ERROR);
		complete(&ope->reset_complete);
	}

	if (status & OPE_TOP_IRQ_STATUS_VIOL) {
		u32 violation = ope_read(ope, OPE_TOP_VIOLATION_STATUS);

		dev_warn(ope->dev, "OPE Violation: %u", violation);
	}

	if (status & OPE_TOP_IRQ_STATUS_FE)
		ope_fe_irq(ope);

	if (status & OPE_TOP_IRQ_STATUS_WE)
		ope_we_irq(ctx);

	if (status & OPE_TOP_IRQ_STATUS_IDLE)
		ope_buf_done(ctx);

	return IRQ_HANDLED;
}

static void ope_irq_init(struct ope_dev *ope)
{
	ope_write(ope, OPE_TOP_IRQ_MASK,
		  OPE_TOP_IRQ_STATUS_RST_DONE | OPE_TOP_IRQ_STATUS_WE |
		  OPE_TOP_IRQ_STATUS_VIOL | OPE_TOP_IRQ_STATUS_IDLE);

	ope_write_wr(ope, OPE_BUS_WR_INPUT_IF_IRQ_MASK_0,
		     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_RUP_DONE |
		     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_CONS_VIOL |
		     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_VIOL |
		     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_IMG_SZ_VIOL);
}

/*
 * video ioctls
 */
static int ope_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, MEM2MEM_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm Offline Processing Engine", sizeof(cap->card));
	return 0;
}

static int ope_enum_fmt(struct v4l2_fmtdesc *f, u32 type)
{
	const struct ope_fmt *fmt;
	int i, num = 0;

	for (i = 0; i < OPE_NUM_FORMATS; ++i) {
		if (formats[i].types & type) {
			if (num == f->index)
				break;
			++num;
		}
	}

	if (i < OPE_NUM_FORMATS) {
		fmt = &formats[i];
		f->pixelformat = fmt->fourcc;
		return 0;
	}

	return -EINVAL;
}

static int ope_enum_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	return ope_enum_fmt(f, MEM2MEM_CAPTURE);
}

static int ope_enum_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

	return ope_enum_fmt(f, MEM2MEM_OUTPUT);
}

static int ope_g_fmt(struct ope_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct ope_q_data *q_data;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);

	pix_mp->width = q_data->width;
	pix_mp->height = q_data->height;
	pix_mp->pixelformat = q_data->fmt->fourcc;
	pix_mp->num_planes = 1;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->colorspace = ctx->colorspace;
	pix_mp->xfer_func = ctx->xfer_func;
	pix_mp->ycbcr_enc = q_data->ycbcr_enc;
	pix_mp->quantization = q_data->quant;
	pix_mp->plane_fmt[0].bytesperline = q_data->bytesperline;
	pix_mp->plane_fmt[0].sizeimage = q_data->sizeimage;

	return 0;
}

static int ope_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return ope_g_fmt(file2ctx(file), f);
}

static int ope_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return ope_g_fmt(file2ctx(file), f);
}

static int ope_try_fmt(struct v4l2_format *f, const struct ope_fmt *fmt)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	unsigned int stride = pix_mp->plane_fmt[0].bytesperline;
	unsigned int size;

	pix_mp->num_planes = 1;
	pix_mp->field = V4L2_FIELD_NONE;

	v4l_bound_align_image(&pix_mp->width, OPE_MIN_W, OPE_MAX_W, fmt->align,
			      &pix_mp->height, OPE_MIN_H, OPE_MAX_H, OPE_ALIGN_H, 0);

	if (ope_pix_fmt_is_yuv(pix_mp->pixelformat)) {
		stride = MAX(pix_mp->width, stride);
		size = fmt->depth * pix_mp->width / 8 * pix_mp->height;
	} else {
		stride = MAX(pix_mp->width * fmt->depth / 8, stride);
		size = stride * pix_mp->height;
	}

	pix_mp->plane_fmt[0].bytesperline = stride;
	pix_mp->plane_fmt[0].sizeimage = size;

	return 0;
}

static int ope_try_fmt_vid_cap(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct ope_ctx *ctx = file2ctx(file);
	const struct ope_fmt *fmt;
	int ret;

	dev_dbg(ctx->ope->dev, "Try capture format: %ux%u-%s (planes:%u bpl:%u size:%u)\n",
		pix_mp->width, pix_mp->height, print_fourcc(pix_mp->pixelformat),
		pix_mp->num_planes, pix_mp->plane_fmt[0].bytesperline,
		pix_mp->plane_fmt[0].sizeimage);

	fmt = find_format(pix_mp->pixelformat);
	if (!fmt) {
		pix_mp->pixelformat = ctx->q_data_dst.fmt->fourcc;
		fmt = ctx->q_data_dst.fmt;
	}

	if (!(fmt->types & MEM2MEM_CAPTURE) && (fmt != ctx->q_data_src.fmt))
		return -EINVAL;

	if (pix_mp->width > ctx->q_data_src.width ||
	    pix_mp->height > ctx->q_data_src.height) {
		pix_mp->width = ctx->q_data_src.width;
		pix_mp->height = ctx->q_data_src.height;
	}

	pix_mp->colorspace = ope_pix_fmt_is_yuv(pix_mp->pixelformat) ?
		ctx->colorspace : V4L2_COLORSPACE_RAW;
	pix_mp->xfer_func = ctx->xfer_func;
	pix_mp->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix_mp->colorspace);
	pix_mp->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
					pix_mp->colorspace, pix_mp->ycbcr_enc);

	ret = ope_try_fmt(f, fmt);

	dev_dbg(ctx->ope->dev, "Fixed capture format: %ux%u-%s (planes:%u bpl:%u size:%u)\n",
		pix_mp->width, pix_mp->height, print_fourcc(pix_mp->pixelformat),
		pix_mp->num_planes, pix_mp->plane_fmt[0].bytesperline,
		pix_mp->plane_fmt[0].sizeimage);

	return ret;
}

static int ope_try_fmt_vid_out(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct ope_fmt *fmt;
	struct ope_ctx *ctx = file2ctx(file);
	int ret;

	dev_dbg(ctx->ope->dev, "Try output format: %ux%u-%s (planes:%u bpl:%u size:%u)\n",
		pix_mp->width, pix_mp->height, print_fourcc(pix_mp->pixelformat),
		pix_mp->num_planes, pix_mp->plane_fmt[0].bytesperline,
		pix_mp->plane_fmt[0].sizeimage);

	fmt = find_format(pix_mp->pixelformat);
	if (!fmt) {
		pix_mp->pixelformat = ctx->q_data_src.fmt->fourcc;
		fmt = ctx->q_data_src.fmt;
	}
	if (!(fmt->types & MEM2MEM_OUTPUT))
		return -EINVAL;

	if (!pix_mp->colorspace)
		pix_mp->colorspace = V4L2_COLORSPACE_SRGB;

	pix_mp->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix_mp->colorspace);
	pix_mp->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
					pix_mp->colorspace, pix_mp->ycbcr_enc);

	ret = ope_try_fmt(f, fmt);

	dev_dbg(ctx->ope->dev, "Fixed output format: %ux%u-%s (planes:%u bpl:%u size:%u)\n",
		pix_mp->width, pix_mp->height, print_fourcc(pix_mp->pixelformat),
		pix_mp->num_planes, pix_mp->plane_fmt[0].bytesperline,
		pix_mp->plane_fmt[0].sizeimage);

	return ret;
}

static int ope_s_fmt(struct ope_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct ope_q_data *q_data;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&ctx->ope->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	q_data->fmt = find_format(pix_mp->pixelformat);
	if (!q_data->fmt)
		return -EINVAL;
	q_data->width = pix_mp->width;
	q_data->height = pix_mp->height;
	q_data->bytesperline = pix_mp->plane_fmt[0].bytesperline;
	q_data->sizeimage = pix_mp->plane_fmt[0].sizeimage;

	dev_dbg(ctx->ope->dev, "Set %s format: %ux%u %s (%u bytes)\n",
		V4L2_TYPE_IS_OUTPUT(f->type) ? "output" : "capture",
		q_data->width, q_data->height, print_fourcc(q_data->fmt->fourcc),
		q_data->sizeimage);

	return 0;
}

static int ope_s_fmt_vid_cap(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct ope_ctx *ctx = file2ctx(file);
	int ret;

	ret = ope_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	ret = ope_s_fmt(file2ctx(file), f);
	if (ret)
		return ret;

	ctx->q_data_dst.ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	ctx->q_data_dst.quant = f->fmt.pix_mp.quantization;

	return 0;
}

static int ope_s_fmt_vid_out(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct ope_ctx *ctx = file2ctx(file);
	int ret;

	ret = ope_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	ret = ope_s_fmt(file2ctx(file), f);
	if (ret)
		return ret;

	ctx->colorspace = f->fmt.pix_mp.colorspace;
	ctx->xfer_func = f->fmt.pix_mp.xfer_func;
	ctx->q_data_src.ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	ctx->q_data_src.quant = f->fmt.pix_mp.quantization;

	return 0;
}

static int ope_enum_framesizes(struct file *file, void *fh,
			       struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > 0)
		return -EINVAL;

	if (!find_format(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = OPE_MIN_W;
	fsize->stepwise.max_width = OPE_MAX_W;
	fsize->stepwise.step_width = 1 << OPE_ALIGN_W;
	fsize->stepwise.min_height = OPE_MIN_H;
	fsize->stepwise.max_height = OPE_MAX_H;
	fsize->stepwise.step_height = 1 << OPE_ALIGN_H;

	return 0;
}

static int ope_enum_frameintervals(struct file *file, void *fh,
				   struct v4l2_frmivalenum *fival)
{
	fival->type = V4L2_FRMIVAL_TYPE_STEPWISE;
	fival->stepwise.min.numerator = 1;
	fival->stepwise.min.denominator = 120;
	fival->stepwise.max.numerator = 1;
	fival->stepwise.max.denominator = 1;
	fival->stepwise.step.numerator = 1;
	fival->stepwise.step.denominator = 1;

	return 0;
}

static int ope_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return -EINVAL;
}

static const struct v4l2_ctrl_ops ope_ctrl_ops = {
	.s_ctrl = ope_s_ctrl,
};

static const struct v4l2_ioctl_ops ope_ioctl_ops = {
	.vidioc_querycap	= ope_querycap,

	.vidioc_enum_fmt_vid_cap = ope_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap_mplane	= ope_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap_mplane	= ope_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane	= ope_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out = ope_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out_mplane	= ope_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out_mplane	= ope_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out_mplane	= ope_s_fmt_vid_out,

	.vidioc_enum_framesizes	= ope_enum_framesizes,
	.vidioc_enum_frameintervals = ope_enum_frameintervals,

	.vidioc_reqbufs		= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf	= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf		= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf		= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf	= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs	= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf		= v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon	= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff	= v4l2_m2m_ioctl_streamoff,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/*
 * Queue operations
 */
static int ope_queue_setup(struct vb2_queue *vq,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], struct device *alloc_devs[])
{
	struct ope_ctx *ctx = vb2_get_drv_priv(vq);
	struct ope_q_data *q_data = get_q_data(ctx, vq->type);
	unsigned int size = q_data->sizeimage;

	if (*nplanes) {
		if (*nplanes != 1)
			return -EINVAL;
	} else {
		*nplanes = 1;
	}

	if (sizes[0]) {
		if (sizes[0] < size)
			return -EINVAL;
	} else {
		sizes[0] = size;
	}

	dev_dbg(ctx->ope->dev, "get %d buffer(s) of size %d each.\n", *nbuffers, size);

	return 0;
}

static int ope_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct ope_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ope_dev *ope = ctx->ope;
	struct ope_q_data *q_data;

	q_data = get_q_data(ctx, vb->vb2_queue->type);
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vbuf->field == V4L2_FIELD_ANY)
			vbuf->field = V4L2_FIELD_NONE;
		if (vbuf->field != V4L2_FIELD_NONE) {
			v4l2_err(&ope->v4l2_dev, "Field isn't supported\n");
			return -EINVAL;
		}
	}

	if (vb2_plane_size(vb, 0) < q_data->sizeimage) {
		v4l2_err(&ope->v4l2_dev, "Data will not fit into plane (%lu < %lu)\n",
			 vb2_plane_size(vb, 0), (long)q_data->sizeimage);
		return -EINVAL;
	}

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0, q_data->sizeimage);

	vbuf->sequence = q_data->sequence++;

	return 0;
}

static void ope_adjust_power(struct ope_dev *ope)
{
	int ret;
	unsigned long pixclk = 0;
	unsigned int loadavg = 0;
	unsigned int loadpeak = 0;
	unsigned int loadconfig = 0;
	struct ope_ctx *ctx;

	lockdep_assert_held(&ope->mutex);

	list_for_each_entry(ctx, &ope->ctx_list, list) {
		if (!ctx->started)
			continue;

		if (!ctx->framerate)
			ctx->framerate = DEFAULT_FRAMERATE;

		pixclk += __q_data_pixclk(&ctx->q_data_src, ctx->framerate);
		loadavg += __q_data_load_avg(&ctx->q_data_src, ctx->framerate);
		loadavg += __q_data_load_avg(&ctx->q_data_dst, ctx->framerate);
		loadpeak += __q_data_load_peak(&ctx->q_data_src, ctx->framerate);
		loadpeak += __q_data_load_peak(&ctx->q_data_dst, ctx->framerate);
		loadconfig += __q_data_load_config(&ctx->q_data_src, ctx->framerate);
	}

	/* 30% margin for overhead */
	pixclk = mult_frac(pixclk, 13, 10);

	dev_dbg(ope->dev, "Adjusting clock:%luHz avg:%uKBps peak:%uKBps config:%uKBps\n",
		pixclk, loadavg, loadpeak, loadconfig);

	ret = dev_pm_opp_set_rate(ope->dev, pixclk);
	if (ret)
		dev_warn(ope->dev, "Failed to adjust OPP rate: %d\n", ret);

	ret = icc_set_bw(ope->icc_data, loadavg, loadpeak);
	if (ret)
		dev_warn(ope->dev, "Failed to set data path bandwidth: %d\n", ret);

	ret = icc_set_bw(ope->icc_config, loadconfig, loadconfig * 5);
	if (ret)
		dev_warn(ope->dev, "Failed to set config path bandwidth: %d\n", ret);
}

static void ope_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct ope_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int ope_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ope_ctx *ctx = vb2_get_drv_priv(q);
	struct ope_dev *ope = ctx->ope;
	struct ope_q_data *q_data;
	int ret;

	dev_dbg(ope->dev, "Start streaming (ctx %p/%u)\n", ctx, q->type);

	lockdep_assert_held(&ope->mutex);

	q_data = get_q_data(ctx, q->type);
	q_data->sequence = 0;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->started = true;
		ope_adjust_power(ctx->ope);
	}

	ret = pm_runtime_resume_and_get(ctx->ope->dev);
	if (ret) {
		dev_err(ope->dev, "Could not resume\n");
		return ret;
	}

	ope_irq_init(ope);

	return 0;
}

static void ope_stop_streaming(struct vb2_queue *q)
{
	struct ope_ctx *ctx = vb2_get_drv_priv(q);
	struct ope_dev *ope = ctx->ope;
	struct vb2_v4l2_buffer *vbuf;

	dev_dbg(ctx->ope->dev, "Stop streaming (ctx %p/%u)\n", ctx, q->type);

	lockdep_assert_held(&ope->mutex);

	if (ope->context == ctx)
		ope->context = NULL;

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->started = false;
		ope_adjust_power(ctx->ope);
	}

	pm_runtime_put(ctx->ope->dev);

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (vbuf == NULL)
			return;

		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}
}

static const struct vb2_ops ope_qops = {
	.queue_setup	 = ope_queue_setup,
	.buf_prepare	 = ope_buf_prepare,
	.buf_queue	 = ope_buf_queue,
	.start_streaming = ope_start_streaming,
	.stop_streaming  = ope_stop_streaming,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct ope_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &ope_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->ope->mutex;
	src_vq->dev = ctx->ope->v4l2_dev.dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &ope_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->ope->mutex;
	dst_vq->dev = ctx->ope->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

/*
 * File operations
 */
static int ope_open(struct file *file)
{
	struct ope_dev *ope = video_drvdata(file);
	struct ope_ctx *ctx = NULL;
	int rc = 0;

	if (mutex_lock_interruptible(&ope->mutex))
		return -ERESTARTSYS;

	ctx = kvzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		rc = -ENOMEM;
		goto open_unlock;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->ope = ope;
	ctx->colorspace = V4L2_COLORSPACE_RAW;

	ctx->q_data_src.fmt = find_format(V4L2_PIX_FMT_SRGGB8);
	ctx->q_data_src.width = 640;
	ctx->q_data_src.height = 480;
	ctx->q_data_src.bytesperline = 640;
	ctx->q_data_src.sizeimage = 640 * 480;

	ctx->q_data_dst.fmt = find_format(V4L2_PIX_FMT_NV12);
	ctx->q_data_dst.width = 640;
	ctx->q_data_dst.height = 480;
	ctx->q_data_dst.bytesperline = 640;
	ctx->q_data_dst.sizeimage = 640 * 480 * 3 / 2;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(ope->m2m_dev, ctx, &queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		rc = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		kvfree(ctx);
		goto open_unlock;
	}

	v4l2_fh_add(&ctx->fh, file);

	list_add(&ctx->list, &ope->ctx_list);

	dev_dbg(ope->dev, "Created ctx %p\n", ctx);

open_unlock:
	mutex_unlock(&ope->mutex);
	return rc;
}

static int ope_release(struct file *file)
{
	struct ope_dev *ope = video_drvdata(file);
	struct ope_ctx *ctx = file2ctx(file);

	dev_dbg(ope->dev, "Releasing ctx %p\n", ctx);

	guard(mutex)(&ope->mutex);

	if (ope->context == ctx)
		ope->context = NULL;

	list_del(&ctx->list);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kvfree(ctx);

	return 0;
}

static const struct v4l2_file_operations ope_fops = {
	.owner		= THIS_MODULE,
	.open		= ope_open,
	.release	= ope_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static const struct video_device ope_videodev = {
	.name		= MEM2MEM_NAME,
	.vfl_dir	= VFL_DIR_M2M,
	.fops		= &ope_fops,
	.device_caps	= V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE,
	.ioctl_ops	= &ope_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release_empty,
};

static const struct v4l2_m2m_ops m2m_ops = {
	.device_run	= ope_device_run,
	.job_abort	= ope_job_abort,
};

static int ope_soft_reset(struct ope_dev *ope)
{
	u32 version;
	int ret = 0;

	ret = pm_runtime_resume_and_get(ope->dev);
	if (ret) {
		dev_err(ope->dev, "Could not resume\n");
		return ret;
	}

	version = ope_read(ope, OPE_TOP_HW_VERSION);

	dev_dbg(ope->dev, "HW Version = %u.%u.%u\n",
		(u32)FIELD_GET(OPE_TOP_HW_VERSION_GEN, version),
		(u32)FIELD_GET(OPE_TOP_HW_VERSION_REV, version),
		(u32)FIELD_GET(OPE_TOP_HW_VERSION_STEP, version));

	reinit_completion(&ope->reset_complete);

	ope_write(ope, OPE_TOP_RESET_CMD, OPE_TOP_RESET_CMD_SW);

	if (!wait_for_completion_timeout(&ope->reset_complete,
					 msecs_to_jiffies(OPE_RESET_TIMEOUT_MS))) {
		dev_err(ope->dev, "Reset timeout\n");
		ret = -ETIMEDOUT;
	}

	pm_runtime_put(ope->dev);

	return ret;
}

static int ope_init_power(struct ope_dev *ope)
{
	struct dev_pm_domain_list *pmdomains;
	struct device *dev = ope->dev;
	int ret;

	ope->icc_data = devm_of_icc_get(dev, "data");
	if (IS_ERR(ope->icc_data))
		return dev_err_probe(dev, PTR_ERR(ope->icc_data),
				     "failed to get interconnect data path\n");

	ope->icc_config = devm_of_icc_get(dev, "config");
	if (IS_ERR(ope->icc_config))
		return dev_err_probe(dev, PTR_ERR(ope->icc_config),
				     "failed to get interconnect config path\n");

	/*  Devices with multiple PM domains must be attached separately */
	devm_pm_domain_attach_list(dev, NULL, &pmdomains);

	/* core clock is scaled as part of operating points */
	ret = devm_pm_opp_set_clkname(dev, "core");
	if (ret)
		return ret;

	ret = devm_pm_opp_of_add_table(dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "invalid OPP table\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = devm_pm_clk_create(dev);
	if (ret)
		return ret;

	ret = of_pm_clk_add_clks(dev);
	if (ret < 0)
		return ret;

	return 0;
}

static int ope_init_mmio(struct ope_dev *ope)
{
	struct platform_device *pdev = to_platform_device(ope->dev);

	ope->base = devm_platform_ioremap_resource_byname(pdev, "top");
	if (IS_ERR(ope->base))
		return PTR_ERR(ope->base);

	ope->base_rd = devm_platform_ioremap_resource_byname(pdev, "bus_read");
	if (IS_ERR(ope->base_rd))
		return PTR_ERR(ope->base_rd);

	ope->base_wr = devm_platform_ioremap_resource_byname(pdev, "bus_write");
	if (IS_ERR(ope->base_wr))
		return PTR_ERR(ope->base_wr);

	ope->base_pp = devm_platform_ioremap_resource_byname(pdev, "pipeline");
	if (IS_ERR(ope->base_pp))
		return PTR_ERR(ope->base_pp);

	return 0;
}

static int ope_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vfd;
	struct ope_dev *ope;
	int ret, irq;

	ope = devm_kzalloc(&pdev->dev, sizeof(*ope), GFP_KERNEL);
	if (!ope)
		return -ENOMEM;

	ope->dev = dev;
	init_completion(&ope->reset_complete);

	ret = ope_init_power(ope);
	if (ret)
		return dev_err_probe(dev, ret, "Power init failed\n");

	ret = ope_init_mmio(ope);
	if (ret)
		return dev_err_probe(dev, ret, "MMIO init failed\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "Unable to get IRQ\n");

	ret = devm_request_irq(dev, irq, ope_irq, IRQF_TRIGGER_RISING, "ope", ope);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Requesting IRQ failed\n");

	ret = ope_soft_reset(ope);
	if (ret < 0)
		return ret;

	ret = v4l2_device_register(&pdev->dev, &ope->v4l2_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Registering V4L2 device failed\n");

	mutex_init(&ope->mutex);
	INIT_LIST_HEAD(&ope->ctx_list);

	ope->vfd = ope_videodev;
	vfd = &ope->vfd;
	vfd->lock = &ope->mutex;
	vfd->v4l2_dev = &ope->v4l2_dev;
	video_set_drvdata(vfd, ope);
	snprintf(vfd->name, sizeof(vfd->name), "%s", ope_videodev.name);

	platform_set_drvdata(pdev, ope);

	ope->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(ope->m2m_dev)) {
		ret = dev_err_probe(dev, PTR_ERR(ope->m2m_dev), "Failed to init mem2mem device\n");
		goto err_unregister_v4l2;
	}

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
	if (ret) {
		dev_err(dev, "Failed to refgister video device\n");
		goto err_release_m2m;
	}

	/* TODO: Add stat device and link it to media */
	ope->mdev.dev = dev;
	strscpy(ope->mdev.model, MEM2MEM_NAME, sizeof(ope->mdev.model));
	media_device_init(&ope->mdev);
	ope->v4l2_dev.mdev = &ope->mdev;

	ret = v4l2_m2m_register_media_controller(ope->m2m_dev, vfd,
						 MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register m2m media controller\n");
		goto err_unregister_video;
	}

	ret = media_device_register(&ope->mdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register media device\n");
		goto err_unregister_m2m_mc;
	}

	return 0;

err_unregister_m2m_mc:
	v4l2_m2m_unregister_media_controller(ope->m2m_dev);
err_unregister_video:
	video_unregister_device(&ope->vfd);
err_release_m2m:
	v4l2_m2m_release(ope->m2m_dev);
err_unregister_v4l2:
	v4l2_device_unregister(&ope->v4l2_dev);

	return ret;
}

static void ope_remove(struct platform_device *pdev)
{
	struct ope_dev *ope = platform_get_drvdata(pdev);

	media_device_unregister(&ope->mdev);
	v4l2_m2m_unregister_media_controller(ope->m2m_dev);
	video_unregister_device(&ope->vfd);
	v4l2_m2m_release(ope->m2m_dev);
	v4l2_device_unregister(&ope->v4l2_dev);
}

static const struct of_device_id ope_dt_ids[] = {
	{ .compatible = "qcom,qcm2290-camss-ope"},
	{ },
};
MODULE_DEVICE_TABLE(of, ope_dt_ids);

static const struct dev_pm_ops ope_pm_ops = {
	SET_RUNTIME_PM_OPS(pm_clk_suspend, pm_clk_resume, NULL)
};

static struct platform_driver ope_driver = {
	.probe		= ope_probe,
	.remove		= ope_remove,
	.driver		= {
		.name	= MEM2MEM_NAME,
		.of_match_table = ope_dt_ids,
		.pm = &ope_pm_ops,
	},
};

module_platform_driver(ope_driver);

MODULE_DESCRIPTION("CAMSS Offline Processing Engine");
MODULE_AUTHOR("Loic Poulain <loic.poulain@oss.qualcomm.com>");
MODULE_LICENSE("GPL");
