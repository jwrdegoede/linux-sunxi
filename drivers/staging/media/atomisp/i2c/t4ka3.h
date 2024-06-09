/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Support for T4KA3 8M camera sensor.
 *
 * Copyright (C) 2014 Intel Corporation. All Rights Reserved.
 * Copyright (C) 2016 XiaoMi, Inc.
 * Copyright (C) 2024 Hans de Goede <hansg@kernel.org>
 */

#ifndef __T4KA3_H__
#define __T4KA3_H__
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include "../include/linux/atomisp_gmin_platform.h"

#define T4KA3_NAME	"t4ka3"

/* Defines for register writes and register array processing */
#define T4KA3_BYTE_MAX	30
#define T4KA3_SHORT_MAX	16
#define I2C_MSG_LENGTH		0x2
#define MAX_FMTS		1

#define T4KA3_RES_WIDTH_MAX	3280
#define T4KA3_RES_HEIGHT_MAX	2464

#define T4K3A_PIXELS_PER_LINE	3440
#define T4K3A_LINES_PER_FRAME	2492

#define T4K3A_FPS		30

#define T4K3A_PIXEL_RATE	(T4K3A_PIXELS_PER_LINE * T4K3A_LINES_PER_FRAME * T4K3A_FPS)

/*
 * TODO this really should be derived from the 19.2 MHz xvclk combined
 * with the PLL settings. But without a datasheet this is the closest
 * approximation possible.
 *
 * link-freq = pixel_rate * bpp / (lanes * 2)
 * (lanes * 2) because CSI lanes use double-data-rate (DDR) signalling.
 * bpp = 10 and lanes = 4
 */
#define T4K3A_LINK_FREQ		((s64)T4K3A_PIXEL_RATE * 10 / 8)

#define T4KA3_REG_PRODUCT_ID			0x0000
#define T4KA3_PRODUCT_ID				0x1490

#define T4KA3_REG_IMG_ORIENTATION		0x0101
#define T4KA3_HFLIP_BIT					0x1
#define T4KA3_VFLIP_BIT					0x2
#define T4KA3_VFLIP_OFFSET				1

#define T4KA3_REG_COARSE_INTEGRATION_TIME	0x0202

#define T4KA3_COARSE_INTEGRATION_TIME_MIN		1
#define T4KA3_COARSE_INTEGRATION_TIME_MARGIN		6

#define T4KA3_MAX_EXPOSURE_SUPPORTED		\
	(0xffff - T4KA3_COARSE_INTEGRATION_TIME_MARGIN)

#define T4KA3_REG_DIGGAIN_GREEN_R		0x020e
#define T4KA3_REG_DIGGAIN_RED			0x0210
#define T4KA3_REG_DIGGAIN_BLUE			0x0212
#define T4KA3_REG_DIGGAIN_GREEN_B		0x0214

#define T4KA3_REG_GLOBAL_GAIN			0x0234
#define T4KA3_MAX_GLOBAL_GAIN_SUPPORTED			0x07ff
#define T4KA3_MIN_GLOBAL_GAIN_SUPPORTED			0x0080

#define T4KA3_REG_VT_PIX_CLK_DIV		0x0300
#define T4KA3_REG_VT_SYS_CLK_DIV		0x0302
#define T4KA3_REG_PRE_PLL_CLK_DIV		0x0304
#define T4KA3_REG_PLL_MULTIPLIER		0x0306
#define T4KA3_REG_FRAME_LENGTH_LINES		0x0340
#define T4KA3_REG_LINE_LENGTH_PCK		0x0342

#define T4KA3_REG_HORIZONTAL_START		0x0344
#define T4KA3_REG_VERTICAL_START		0x0346
#define T4KA3_REG_HORIZONTAL_END		0x0348
#define T4KA3_REG_VERTICAL_END			0x034a
#define T4KA3_REG_HORIZONTAL_OUTPUT_SIZE	0x034c
/* Per resolution register lists set this to vertical (resolution - 2) ? */
#define T4KA3_REG_VERTICAL_OUTPUT_SIZE		0x034e

/* Window width/height reg guessed based on per resolution register lists */
#define T4KA3_REG_WINDOW_WIDTH			0x040c
#define T4KA3_REG_WINDOW_HEIGHT			0x040e

#define T4KA3_REG_TEST_PATTERN_MODE		0x0601

enum t4ka3_tok_type {
	T4KA3_8BIT  = 0x0001,
	T4KA3_16BIT = 0x0002,
	T4KA3_RMW   = 0x0010,
	T4KA3_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	T4KA3_TOK_DELAY  = 0xfe00, /* delay token for reg list */
	T4KA3_TOK_MASK = 0xfff0
};

struct t4ka3_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *link_freq;
};

struct t4ka3_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct camera_sensor_platform_data *platform_data;
	struct mutex input_lock; /* serialize sensor's ioctl */
	struct t4ka3_ctrls ctrls;
	s64 link_freq[1];
	int fmt_idx;
	int streaming;
	int power;
	u16 coarse_itg;
	u16 gain;
	u16 digital_gain;
	u16 flip;
};

/**
 * struct t4ka3_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct t4ka3_reg {
	enum t4ka3_tok_type type;
	u16 sreg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

#define to_t4ka3_sensor(x) container_of(x, struct t4ka3_device, sd)

#define T4KA3_MAX_WRITE_BUF_SIZE	30
struct t4ka3_write_buffer {
	u16 addr;
	u8 data[T4KA3_MAX_WRITE_BUF_SIZE];
};

struct t4ka3_write_ctrl {
	int index;
	struct t4ka3_write_buffer buffer;
};

struct t4ka3_format_struct {
	u8 *desc;
	struct regval_list *regs;
	u32 pixelformat;
};

struct t4ka3_resolution {
	u8 *desc;
	const struct t4ka3_reg *regs;
	int res;
	int width;
	int height;
	u32 skip_frames;
	u32 code;
};

/* init settings */
static struct t4ka3_reg const t4ka3_init_config[] = {
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x3094, 0x01},
	{T4KA3_8BIT, 0x0233, 0x01},
	{T4KA3_8BIT, 0x4B06, 0x01},
	{T4KA3_8BIT, 0x4B07, 0x01},
	{T4KA3_8BIT, 0x3028, 0x01},
	{T4KA3_8BIT, 0x3032, 0x14},
	{T4KA3_8BIT, 0x305C, 0x0C},
	{T4KA3_8BIT, 0x306D, 0x0A},
	{T4KA3_8BIT, 0x3071, 0xFA},
	{T4KA3_8BIT, 0x307E, 0x0A},
	{T4KA3_8BIT, 0x307F, 0xFC},
	{T4KA3_8BIT, 0x3091, 0x04},
	{T4KA3_8BIT, 0x3092, 0x60},
	{T4KA3_8BIT, 0x3096, 0xC0},
	{T4KA3_8BIT, 0x3100, 0x07},
	{T4KA3_8BIT, 0x3101, 0x4C},
	{T4KA3_8BIT, 0x3118, 0xCC},
	{T4KA3_8BIT, 0x3139, 0x06},
	{T4KA3_8BIT, 0x313A, 0x06},
	{T4KA3_8BIT, 0x313B, 0x04},
	{T4KA3_8BIT, 0x3143, 0x02},
	{T4KA3_8BIT, 0x314F, 0x0E},
	{T4KA3_8BIT, 0x3169, 0x99},
	{T4KA3_8BIT, 0x316A, 0x99},
	{T4KA3_8BIT, 0x3171, 0x05},
	{T4KA3_8BIT, 0x31A1, 0xA7},
	{T4KA3_8BIT, 0x31A2, 0x9C},
	{T4KA3_8BIT, 0x31A3, 0x8F},
	{T4KA3_8BIT, 0x31A4, 0x75},
	{T4KA3_8BIT, 0x31A5, 0xEE},
	{T4KA3_8BIT, 0x31A6, 0xEA},
	{T4KA3_8BIT, 0x31A7, 0xE4},
	{T4KA3_8BIT, 0x31A8, 0xE4},
	{T4KA3_8BIT, 0x31DF, 0x05},
	{T4KA3_8BIT, 0x31EC, 0x1B},
	{T4KA3_8BIT, 0x31ED, 0x1B},
	{T4KA3_8BIT, 0x31EE, 0x1B},
	{T4KA3_8BIT, 0x31F0, 0x1B},
	{T4KA3_8BIT, 0x31F1, 0x1B},
	{T4KA3_8BIT, 0x31F2, 0x1B},
	{T4KA3_8BIT, 0x3204, 0x3F},
	{T4KA3_8BIT, 0x3205, 0x03},
	{T4KA3_8BIT, 0x3210, 0x01},
	{T4KA3_8BIT, 0x3216, 0x68},
	{T4KA3_8BIT, 0x3217, 0x58},
	{T4KA3_8BIT, 0x3218, 0x58},
	{T4KA3_8BIT, 0x321A, 0x68},
	{T4KA3_8BIT, 0x321B, 0x60},
	{T4KA3_8BIT, 0x3238, 0x03},
	{T4KA3_8BIT, 0x3239, 0x03},
	{T4KA3_8BIT, 0x323A, 0x05},
	{T4KA3_8BIT, 0x323B, 0x06},
	{T4KA3_8BIT, 0x3243, 0x03},
	{T4KA3_8BIT, 0x3244, 0x08},
	{T4KA3_8BIT, 0x3245, 0x01},
	{T4KA3_8BIT, 0x3307, 0x19},
	{T4KA3_8BIT, 0x3308, 0x19},
	{T4KA3_8BIT, 0x3320, 0x01},
	{T4KA3_8BIT, 0x3326, 0x15},
	{T4KA3_8BIT, 0x3327, 0x0D},
	{T4KA3_8BIT, 0x3328, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x339E, 0x07},
	{T4KA3_8BIT, 0x3424, 0x00},
	{T4KA3_8BIT, 0x343C, 0x01},
	{T4KA3_8BIT, 0x3398, 0x04},
	{T4KA3_8BIT, 0x343A, 0x10},
	{T4KA3_8BIT, 0x339A, 0x22},
	{T4KA3_8BIT, 0x33B4, 0x00},
	{T4KA3_8BIT, 0x3393, 0x01},
	{T4KA3_8BIT, 0x33B3, 0x6E},
	{T4KA3_8BIT, 0x3433, 0x06},
	{T4KA3_8BIT, 0x3433, 0x00},
	{T4KA3_8BIT, 0x33B3, 0x00},
	{T4KA3_8BIT, 0x3393, 0x03},
	{T4KA3_8BIT, 0x33B4, 0x03},
	{T4KA3_8BIT, 0x343A, 0x00},
	{T4KA3_8BIT, 0x339A, 0x00},
	{T4KA3_8BIT, 0x3398, 0x00},
	{T4KA3_TOK_TERM, 0, 0, }
};

/* Stream mode */
static struct t4ka3_reg const t4ka3_suspend[] = {
	{T4KA3_8BIT, 0x0100, 0x0 },
	{T4KA3_TOK_TERM, 0, 0 },
};

static struct t4ka3_reg const t4ka3_streaming[] = {
	{T4KA3_8BIT, 0x0100, 0x01},
	{T4KA3_TOK_TERM, 0, 0 },
};

/* GROUPED_PARAMETER_HOLD */
static struct t4ka3_reg const t4ka3_param_hold[] = {
	{ T4KA3_8BIT,  0x0104, 0x1 },
	{ T4KA3_TOK_TERM, 0, 0 }
};
static struct t4ka3_reg const t4ka3_param_update[] = {
	{ T4KA3_8BIT,  0x0104, 0x0 },
	{ T4KA3_TOK_TERM, 0, 0 }
};

/* Settings */

static struct t4ka3_reg const t4ka3_736x496_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x02},
	{T4KA3_8BIT, 0x034D, 0xE0},
	{T4KA3_8BIT, 0x034E, 0x01},
	{T4KA3_8BIT, 0x034F, 0xEE},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x01},
	{T4KA3_8BIT, 0x0409, 0x74},
	{T4KA3_8BIT, 0x040A, 0x00},
	{T4KA3_8BIT, 0x040B, 0xFA},
	{T4KA3_8BIT, 0x040C, 0x02},
	{T4KA3_8BIT, 0x040D, 0xE0},
	{T4KA3_8BIT, 0x040E, 0x01},
	{T4KA3_8BIT, 0x040F, 0xF0},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x22},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};

static struct t4ka3_reg const t4ka3_896x736_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x03},
	{T4KA3_8BIT, 0x034D, 0x80},
	{T4KA3_8BIT, 0x034E, 0x02},
	{T4KA3_8BIT, 0x034F, 0xDE},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x01},
	{T4KA3_8BIT, 0x0409, 0x74},
	{T4KA3_8BIT, 0x040A, 0x00},
	{T4KA3_8BIT, 0x040B, 0xFA},
	{T4KA3_8BIT, 0x040C, 0x03},
	{T4KA3_8BIT, 0x040D, 0x80},
	{T4KA3_8BIT, 0x040E, 0x02},
	{T4KA3_8BIT, 0x040F, 0xE0},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x22},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_1296x736_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x05},
	{T4KA3_8BIT, 0x034D, 0x10},
	{T4KA3_8BIT, 0x034E, 0x02},
	{T4KA3_8BIT, 0x034F, 0xDE},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x00},
	{T4KA3_8BIT, 0x0409, 0xAC},
	{T4KA3_8BIT, 0x040A, 0x00},
	{T4KA3_8BIT, 0x040B, 0xFA},
	{T4KA3_8BIT, 0x040C, 0x05},
	{T4KA3_8BIT, 0x040D, 0x10},
	{T4KA3_8BIT, 0x040E, 0x02},
	{T4KA3_8BIT, 0x040F, 0xE0},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x22},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_1632x1224_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x06},
	{T4KA3_8BIT, 0x034D, 0x60},
	{T4KA3_8BIT, 0x034E, 0x04},
	{T4KA3_8BIT, 0x034F, 0xC6},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x00},
	{T4KA3_8BIT, 0x0409, 0x04},
	{T4KA3_8BIT, 0x040A, 0x00},
	{T4KA3_8BIT, 0x040B, 0x06},
	{T4KA3_8BIT, 0x040C, 0x06},
	{T4KA3_8BIT, 0x040D, 0x60},
	{T4KA3_8BIT, 0x040E, 0x04},
	{T4KA3_8BIT, 0x040F, 0xC8},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x22},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

static struct t4ka3_reg const t4ka3_1936x1096_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x07},
	{T4KA3_8BIT, 0x034D, 0x90},
	{T4KA3_8BIT, 0x034E, 0x04},
	{T4KA3_8BIT, 0x034F, 0x46},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0c},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x02},
	{T4KA3_8BIT, 0x0409, 0xA0},
	{T4KA3_8BIT, 0x040A, 0x02},
	{T4KA3_8BIT, 0x040B, 0xAE},
	{T4KA3_8BIT, 0x040C, 0x07},
	{T4KA3_8BIT, 0x040D, 0x90},
	{T4KA3_8BIT, 0x040E, 0x04},
	{T4KA3_8BIT, 0x040F, 0x4B}, /* Should be 0x48 ? */
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x11},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_2064x1552_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x08},
	{T4KA3_8BIT, 0x034D, 0x10},
	{T4KA3_8BIT, 0x034E, 0x06},
	{T4KA3_8BIT, 0x034F, 0x0E},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x02},
	{T4KA3_8BIT, 0x0409, 0x60},
	{T4KA3_8BIT, 0x040A, 0x01},
	{T4KA3_8BIT, 0x040B, 0xCA},
	{T4KA3_8BIT, 0x040C, 0x08},
	{T4KA3_8BIT, 0x040D, 0x10},
	{T4KA3_8BIT, 0x040E, 0x06},
	{T4KA3_8BIT, 0x040F, 0x10},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x11},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_2576x1936_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x0A},
	{T4KA3_8BIT, 0x034D, 0x10},
	{T4KA3_8BIT, 0x034E, 0x07},
	{T4KA3_8BIT, 0x034F, 0x8E},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x01},
	{T4KA3_8BIT, 0x0409, 0x60},
	{T4KA3_8BIT, 0x040A, 0x01},
	{T4KA3_8BIT, 0x040B, 0x0A},
	{T4KA3_8BIT, 0x040C, 0x0A},
	{T4KA3_8BIT, 0x040D, 0x10},
	{T4KA3_8BIT, 0x040E, 0x07},
	{T4KA3_8BIT, 0x040F, 0x90},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x11},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

#if 0 /* unused */
static struct t4ka3_reg const t4ka3_3280x1852_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x0C},
	{T4KA3_8BIT, 0x034D, 0xD0},
	{T4KA3_8BIT, 0x034E, 0x07},
	{T4KA3_8BIT, 0x034F, 0x3A},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9f},
	{T4KA3_8BIT, 0x0408, 0x00},
	{T4KA3_8BIT, 0x0409, 0x00},
	{T4KA3_8BIT, 0x040A, 0x01},
	{T4KA3_8BIT, 0x040B, 0x34},
	{T4KA3_8BIT, 0x040C, 0x0C},
	{T4KA3_8BIT, 0x040D, 0xD0},
	{T4KA3_8BIT, 0x040E, 0x07},
	{T4KA3_8BIT, 0x040F, 0x3C},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x11},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};
#endif

static struct t4ka3_reg const t4ka3_3280x2464_30fps[] = {
	{T4KA3_8BIT, 0x0112, 0x0A},
	{T4KA3_8BIT, 0x0113, 0x0A},
	{T4KA3_8BIT, 0x0114, 0x03},
	{T4KA3_8BIT, 0x4136, 0x13},
	{T4KA3_8BIT, 0x4137, 0x33},
	{T4KA3_8BIT, 0x0820, 0x0A},
	{T4KA3_8BIT, 0x0821, 0x0D},
	{T4KA3_8BIT, 0x0822, 0x00},
	{T4KA3_8BIT, 0x0823, 0x00},
	{T4KA3_8BIT, 0x0301, 0x0A},
	{T4KA3_8BIT, 0x0303, 0x01},
	{T4KA3_8BIT, 0x0305, 0x04},
	{T4KA3_8BIT, 0x0306, 0x02},
	{T4KA3_8BIT, 0x0307, 0x18},
	{T4KA3_8BIT, 0x030B, 0x01},
	{T4KA3_8BIT, 0x034C, 0x0C},
	{T4KA3_8BIT, 0x034D, 0xD0},
	{T4KA3_8BIT, 0x034E, 0x09},
	{T4KA3_8BIT, 0x034F, 0x9E},
	{T4KA3_8BIT, 0x0340, 0x09},
	{T4KA3_8BIT, 0x0341, 0xBC},
	{T4KA3_8BIT, 0x0342, 0x0D},
	{T4KA3_8BIT, 0x0343, 0x70},
	{T4KA3_8BIT, 0x0344, 0x00},
	{T4KA3_8BIT, 0x0345, 0x00},
	{T4KA3_8BIT, 0x0346, 0x00},
	{T4KA3_8BIT, 0x0347, 0x00},
	{T4KA3_8BIT, 0x0348, 0x0C},
	{T4KA3_8BIT, 0x0349, 0xCF},
	{T4KA3_8BIT, 0x034A, 0x09},
	{T4KA3_8BIT, 0x034B, 0x9F},
	{T4KA3_8BIT, 0x0408, 0x00},
	{T4KA3_8BIT, 0x0409, 0x00},
	{T4KA3_8BIT, 0x040A, 0x00},
	{T4KA3_8BIT, 0x040B, 0x02},
	{T4KA3_8BIT, 0x040C, 0x0C},
	{T4KA3_8BIT, 0x040D, 0xD0},
	{T4KA3_8BIT, 0x040E, 0x09},
	{T4KA3_8BIT, 0x040F, 0xA0},
	{T4KA3_8BIT, 0x0900, 0x01},
	{T4KA3_8BIT, 0x0901, 0x11},
	{T4KA3_8BIT, 0x0902, 0x00},
	{T4KA3_8BIT, 0x4220, 0x00},
	{T4KA3_8BIT, 0x4222, 0x01},
	{T4KA3_8BIT, 0x3380, 0x01},
	{T4KA3_8BIT, 0x3090, 0x88},
	{T4KA3_8BIT, 0x3394, 0x20},
	{T4KA3_8BIT, 0x3090, 0x08},
	{T4KA3_8BIT, 0x3394, 0x10},
	{T4KA3_TOK_TERM, 0, 0 }
};

struct t4ka3_resolution t4ka3_res_preview[] = {

	{
		.desc = "t4ka3_736x496_30fps",
		.regs = t4ka3_736x496_30fps,
		.width = 736,
		.height = 496,
		.skip_frames = 2,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	},
	{
		.desc = "t4ka3_896x736_30fps",
		.regs = t4ka3_896x736_30fps,
		.width = 896,
		.height = 736,
		.skip_frames = 2,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	},
	{
		.desc = "t4ka3_1936x1096_30fps",
		.regs = t4ka3_1936x1096_30fps,
		.width = 1936,
		.height = 1096,
		.skip_frames = 2,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	},
	{
		.desc = "t4ka3_3280x2464_30fps",
		.regs = t4ka3_3280x2464_30fps,
		.width = 3280,
		.height = 2464,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	},
};
#define N_RES_PREVIEW (ARRAY_SIZE(t4ka3_res_preview))

struct t4ka3_resolution *t4ka3_res = t4ka3_res_preview;
static int N_RES = N_RES_PREVIEW;

#endif
