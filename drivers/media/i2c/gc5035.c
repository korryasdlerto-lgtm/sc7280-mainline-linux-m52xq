// SPDX-License-Identifier: GPL-2.0
/*
 * GalaxyCore GC5035 5 Mpixel (2592x1944) 2-lane MIPI CSI-2 sensor driver.
 *
 * Register sequence, exposure/gain logic and chip identification are taken
 * from the Rockchip BSP driver (drivers/media/i2c/gc5035.c, Rockchip
 * Electronics, GPL-2.0), restructured as a mainline-style V4L2 subdev for the
 * Galaxy M52 5G (SM-M526B) macro camera.  OTP calibration is not used.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#define GC5035_CHIP_ID			0x5035
#define GC5035_REG_CHIP_ID		CCI_REG16(0xf0)
#define GC5035_REG_PAGE			CCI_REG8(0xfe)
#define GC5035_REG_EXPOSURE		CCI_REG16(0x03)
#define GC5035_REG_VTS			CCI_REG16(0x41)
#define GC5035_REG_STREAM		CCI_REG8(0x3e)
#define GC5035_STREAM_ON		0x91
#define GC5035_STREAM_OFF		0x01
#define GC5035_REG_AGAIN		CCI_REG8(0xb6)
#define GC5035_REG_DGAIN_INT		CCI_REG8(0xb1)
#define GC5035_REG_DGAIN_FRAC		CCI_REG8(0xb2)
#define GC5035_REG_TEST_PATTERN		CCI_REG8(0x8c)	/* page 1 */

#define GC5035_XCLK_FREQ		(24 * HZ_PER_MHZ)
#define GC5035_LINK_FREQ		438000000LL
#define GC5035_LANES			2
#define GC5035_PIXEL_RATE		(GC5035_LINK_FREQ * 2 * GC5035_LANES / 10)

#define GC5035_WIDTH			2592
#define GC5035_HEIGHT			1944
#define GC5035_HTS			0x0b68		/* 4 x register 0x05/0x06 (0x02da) */
#define GC5035_VTS_DEF			0x07d0
#define GC5035_VTS_MAX			0x1fff
#define GC5035_VBLANK_MIN		16
#define GC5035_EXPOSURE_MIN		4
#define GC5035_EXPOSURE_STEP		1
#define GC5035_EXPOSURE_DEF		0x07c0

#define GC5035_GAIN_MIN			64
#define GC5035_GAIN_MAX			1024
#define GC5035_GAIN_STEP		1
#define GC5035_GAIN_DEF			64

static const char * const gc5035_supply_names[] = {
	"avdd",		/* analog 2.8 V */
	"dovdd",	/* digital I/O 1.8 V */
	"dvdd",		/* digital core 1.2 V */
};
#define GC5035_NUM_SUPPLIES ARRAY_SIZE(gc5035_supply_names)

/* analogue gain (x64) -> analogue gain register index */
static const u16 gc5035_agc[][2] = {
	{  64,  0 }, {  76,  1 }, {  90,  2 }, { 106,  3 }, { 126,  8 }, { 150,  9 },
	{ 179, 10 }, { 211, 11 }, { 250, 12 }, { 301, 13 }, { 358, 14 }, { 427, 15 },
	{ 499, 16 }, { 589, 17 }, { 704, 18 }, { 830, 19 }, { 998, 20 },
};

static const struct cci_reg_sequence gc5035_init_regs[] = {
	/* SYSTEM */
	{CCI_REG8(0xfc), 0x01},
	{CCI_REG8(0xf4), 0x40},
	{CCI_REG8(0xf5), 0xe9},
	{CCI_REG8(0xf6), 0x14},
	{CCI_REG8(0xf8), 0x49},
	{CCI_REG8(0xf9), 0x82},
	{CCI_REG8(0xfa), 0x00},
	{CCI_REG8(0xfc), 0x81},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0x36), 0x01},
	{CCI_REG8(0xd3), 0x87},
	{CCI_REG8(0x36), 0x00},
	{CCI_REG8(0x33), 0x00},
	{CCI_REG8(0xfe), 0x03},
	{CCI_REG8(0x01), 0xe7},
	{CCI_REG8(0xf7), 0x01},
	{CCI_REG8(0xfc), 0x8f},
	{CCI_REG8(0xfc), 0x8f},
	{CCI_REG8(0xfc), 0x8e},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0xee), 0x30},
	{CCI_REG8(0x87), 0x18},
	{CCI_REG8(0xfe), 0x01},
	{CCI_REG8(0x8c), 0x90},
	{CCI_REG8(0xfe), 0x00},
	/* Analog & CISCTL */
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0x05), 0x02},
	{CCI_REG8(0x06), 0xda},
	{CCI_REG8(0x9d), 0x0c},
	{CCI_REG8(0x09), 0x00},
	{CCI_REG8(0x0a), 0x04},
	{CCI_REG8(0x0b), 0x00},
	{CCI_REG8(0x0c), 0x03},
	{CCI_REG8(0x0d), 0x07},
	{CCI_REG8(0x0e), 0xa8},
	{CCI_REG8(0x0f), 0x0a},
	{CCI_REG8(0x10), 0x30},
	{CCI_REG8(0x11), 0x02},
	{CCI_REG8(0x17), 0x80},
	{CCI_REG8(0x19), 0x05},
	{CCI_REG8(0xfe), 0x02},
	{CCI_REG8(0x30), 0x03},
	{CCI_REG8(0x31), 0x03},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0xd9), 0xc0},
	{CCI_REG8(0x1b), 0x20},
	{CCI_REG8(0x21), 0x48},
	{CCI_REG8(0x28), 0x22},
	{CCI_REG8(0x29), 0x58},
	{CCI_REG8(0x44), 0x20},
	{CCI_REG8(0x4b), 0x10},
	{CCI_REG8(0x4e), 0x1a},
	{CCI_REG8(0x50), 0x11},
	{CCI_REG8(0x52), 0x33},
	{CCI_REG8(0x53), 0x44},
	{CCI_REG8(0x55), 0x10},
	{CCI_REG8(0x5b), 0x11},
	{CCI_REG8(0xc5), 0x02},
	{CCI_REG8(0x8c), 0x1a},
	{CCI_REG8(0xfe), 0x02},
	{CCI_REG8(0x33), 0x05},
	{CCI_REG8(0x32), 0x38},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0x91), 0x80},
	{CCI_REG8(0x92), 0x28},
	{CCI_REG8(0x93), 0x20},
	{CCI_REG8(0x95), 0xa0},
	{CCI_REG8(0x96), 0xe0},
	{CCI_REG8(0xd5), 0xfc},
	{CCI_REG8(0x97), 0x28},
	{CCI_REG8(0x16), 0x0c},
	{CCI_REG8(0x1a), 0x1a},
	{CCI_REG8(0x1f), 0x11},
	{CCI_REG8(0x20), 0x10},
	{CCI_REG8(0x46), 0xe3},
	{CCI_REG8(0x4a), 0x04},
	{CCI_REG8(0x54), 0x02},
	{CCI_REG8(0x62), 0x00},
	{CCI_REG8(0x72), 0xcf},
	{CCI_REG8(0x73), 0xc9},
	{CCI_REG8(0x7a), 0x05},
	{CCI_REG8(0x7d), 0xcc},
	{CCI_REG8(0x90), 0x00},
	{CCI_REG8(0xce), 0x98},
	{CCI_REG8(0xd0), 0xb2},
	{CCI_REG8(0xd2), 0x40},
	{CCI_REG8(0xe6), 0xe0},
	{CCI_REG8(0xfe), 0x02},
	{CCI_REG8(0x12), 0x01},
	{CCI_REG8(0x13), 0x01},
	{CCI_REG8(0x14), 0x01},
	{CCI_REG8(0x15), 0x02},
	{CCI_REG8(0x22), 0x7c},
	{CCI_REG8(0x91), 0x00},
	{CCI_REG8(0x92), 0x00},
	{CCI_REG8(0x93), 0x00},
	{CCI_REG8(0x94), 0x00},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0xfc), 0x88},
	{CCI_REG8(0xfe), 0x10},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0xfc), 0x8e},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0xfc), 0x88},
	{CCI_REG8(0xfe), 0x10},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0xfc), 0x8e},
	/* Gain */
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0xb0), 0x6e},
	{CCI_REG8(0xb1), 0x01},
	{CCI_REG8(0xb2), 0x00},
	{CCI_REG8(0xb3), 0x00},
	{CCI_REG8(0xb4), 0x00},
	{CCI_REG8(0xb6), 0x00},
	/* ISP */
	{CCI_REG8(0xfe), 0x01},
	{CCI_REG8(0x53), 0x00},
	{CCI_REG8(0x89), 0x03},
	{CCI_REG8(0x60), 0x40},
	{CCI_REG8(0x87), 0x50},
	/* BLK */
	{CCI_REG8(0xfe), 0x01},
	{CCI_REG8(0x42), 0x21},
	{CCI_REG8(0x49), 0x03},
	{CCI_REG8(0x4a), 0xff},
	{CCI_REG8(0x4b), 0xc0},
	{CCI_REG8(0x55), 0x00},
	/* Anti_blooming */
	{CCI_REG8(0xfe), 0x01},
	{CCI_REG8(0x41), 0x28},
	{CCI_REG8(0x4c), 0x00},
	{CCI_REG8(0x4d), 0x00},
	{CCI_REG8(0x4e), 0x3c},
	{CCI_REG8(0x44), 0x08},
	{CCI_REG8(0x48), 0x01},
	/* Crop */
	{CCI_REG8(0xfe), 0x01},
	{CCI_REG8(0x91), 0x00},
	{CCI_REG8(0x92), 0x08},
	{CCI_REG8(0x93), 0x00},
	{CCI_REG8(0x94), 0x07},
	{CCI_REG8(0x95), 0x07},
	{CCI_REG8(0x96), 0x98},
	{CCI_REG8(0x97), 0x0a},
	{CCI_REG8(0x98), 0x20},
	{CCI_REG8(0x99), 0x00},
	/* MIPI */
	{CCI_REG8(0xfe), 0x03},
	{CCI_REG8(0x02), 0x57},
	{CCI_REG8(0x03), 0xb7},
	{CCI_REG8(0x15), 0x14},
	{CCI_REG8(0x18), 0x0f},
	{CCI_REG8(0x21), 0x22},
	{CCI_REG8(0x22), 0x06},
	{CCI_REG8(0x23), 0x48},
	{CCI_REG8(0x24), 0x12},
	{CCI_REG8(0x25), 0x28},
	{CCI_REG8(0x26), 0x08},
	{CCI_REG8(0x29), 0x06},
	{CCI_REG8(0x2a), 0x58},
	{CCI_REG8(0x2b), 0x08},
	{CCI_REG8(0xfe), 0x01},
	{CCI_REG8(0x8c), 0x10},
	{CCI_REG8(0xfe), 0x00},
	{CCI_REG8(0x3e), 0x01},
};

static const char * const gc5035_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4",
};

static const s64 gc5035_link_freq_menu[] = { GC5035_LINK_FREQ };

struct gc5035 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[GC5035_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *vblank;

	u32 dgain_ratio;	/* exposure is set in even steps; the rest goes into digital gain */
};

static inline struct gc5035 *to_gc5035(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc5035, sd);
}

static int gc5035_set_exposure(struct gc5035 *gc, u32 exposure)
{
	u32 even = exposure & ~1U;
	int ret = 0;

	if (!even)
		even = 2;
	gc->dgain_ratio = 64 * exposure / even;

	cci_write(gc->regmap, GC5035_REG_PAGE, 0, &ret);
	cci_write(gc->regmap, GC5035_REG_EXPOSURE, even & 0x3fff, &ret);
	return ret;
}

static int gc5035_set_gain(struct gc5035 *gc, u32 gain)
{
	u32 dgain;
	int i, ret = 0;

	gain = clamp(gain, (u32)GC5035_GAIN_MIN, (u32)GC5035_GAIN_MAX);
	for (i = ARRAY_SIZE(gc5035_agc) - 1; i > 0; i--)
		if (gain >= gc5035_agc[i][0])
			break;

	dgain = gain * (gc->dgain_ratio ? gc->dgain_ratio : 64) / gc5035_agc[i][0];

	cci_write(gc->regmap, GC5035_REG_PAGE, 0, &ret);
	cci_write(gc->regmap, GC5035_REG_AGAIN, gc5035_agc[i][1], &ret);
	cci_write(gc->regmap, GC5035_REG_DGAIN_INT, dgain >> 6, &ret);
	cci_write(gc->regmap, GC5035_REG_DGAIN_FRAC, (dgain << 2) & 0xfc, &ret);
	return ret;
}

static int gc5035_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc5035 *gc = container_of(ctrl->handler, struct gc5035, ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK)
		__v4l2_ctrl_modify_range(gc->exposure, GC5035_EXPOSURE_MIN,
					 GC5035_HEIGHT + ctrl->val - 4,
					 GC5035_EXPOSURE_STEP, GC5035_EXPOSURE_DEF);

	if (!pm_runtime_get_if_in_use(gc->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = gc5035_set_exposure(gc, ctrl->val);
		if (!ret)
			ret = gc5035_set_gain(gc, gc->again->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc5035_set_gain(gc, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		cci_write(gc->regmap, GC5035_REG_PAGE, 0, &ret);
		cci_write(gc->regmap, GC5035_REG_VTS,
			  (GC5035_HEIGHT + ctrl->val) & 0x1fff, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(gc->regmap, GC5035_REG_PAGE, 1, &ret);
		cci_write(gc->regmap, GC5035_REG_TEST_PATTERN, ctrl->val, &ret);
		cci_write(gc->regmap, GC5035_REG_PAGE, 0, &ret);
		break;
	default:
		ret = -EINVAL;
	}

	pm_runtime_put(gc->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gc5035_ctrl_ops = {
	.s_ctrl = gc5035_s_ctrl,
};

static int gc5035_init_controls(struct gc5035 *gc)
{
	struct v4l2_ctrl_handler *hdl = &gc->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 10);

	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, GC5035_PIXEL_RATE,
				 GC5035_PIXEL_RATE, 1, GC5035_PIXEL_RATE);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      gc5035_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, GC5035_HTS - GC5035_WIDTH,
				 GC5035_HTS - GC5035_WIDTH, 1, GC5035_HTS - GC5035_WIDTH);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gc->vblank = v4l2_ctrl_new_std(hdl, &gc5035_ctrl_ops, V4L2_CID_VBLANK,
				       GC5035_VBLANK_MIN, GC5035_VTS_MAX - GC5035_HEIGHT, 1,
				       GC5035_VTS_DEF - GC5035_HEIGHT);

	/* exposure must precede gain: the digital gain depends on the exposure rounding */
	gc->exposure = v4l2_ctrl_new_std(hdl, &gc5035_ctrl_ops, V4L2_CID_EXPOSURE,
					 GC5035_EXPOSURE_MIN,
					 GC5035_VTS_DEF - 4, GC5035_EXPOSURE_STEP,
					 GC5035_EXPOSURE_DEF);

	gc->again = v4l2_ctrl_new_std(hdl, &gc5035_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
				      GC5035_GAIN_MIN, GC5035_GAIN_MAX,
				      GC5035_GAIN_STEP, GC5035_GAIN_DEF);

	v4l2_ctrl_new_std_menu_items(hdl, &gc5035_ctrl_ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(gc5035_test_pattern_menu) - 1, 0, 0,
				     gc5035_test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		goto err;
	}

	ret = v4l2_fwnode_device_parse(gc->dev, &props);
	if (ret)
		goto err;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &gc5035_ctrl_ops, &props);
	if (ret)
		goto err;

	gc->sd.ctrl_handler = hdl;
	return 0;
err:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static void gc5035_fill_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->width = GC5035_WIDTH;
	fmt->height = GC5035_HEIGHT;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int gc5035_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	gc5035_fill_format(v4l2_subdev_state_get_format(state, 0));
	*v4l2_subdev_state_get_crop(state, 0) = (struct v4l2_rect){
		.width = GC5035_WIDTH, .height = GC5035_HEIGHT,
	};
	return 0;
}

static int gc5035_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	/* single mode */
	gc5035_fill_format(&fmt->format);
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	return 0;
}

static int gc5035_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	return 0;
}

static int gc5035_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;
	fse->min_width = fse->max_width = GC5035_WIDTH;
	fse->min_height = fse->max_height = GC5035_HEIGHT;
	return 0;
}

static int gc5035_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = (struct v4l2_rect){ .width = GC5035_WIDTH, .height = GC5035_HEIGHT };
		return 0;
	}
	return -EINVAL;
}

static int gc5035_enable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	struct gc5035 *gc = to_gc5035(sd);
	int ret;

	ret = pm_runtime_resume_and_get(gc->dev);
	if (ret < 0)
		return ret;

	ret = cci_multi_reg_write(gc->regmap, gc5035_init_regs,
				  ARRAY_SIZE(gc5035_init_regs), NULL);
	if (ret) {
		dev_err(gc->dev, "failed to write the init sequence (%d)\n", ret);
		goto err_put;
	}

	ret = __v4l2_ctrl_handler_setup(&gc->ctrls);
	if (ret)
		goto err_put;

	cci_write(gc->regmap, GC5035_REG_PAGE, 0, &ret);
	cci_write(gc->regmap, GC5035_REG_STREAM, GC5035_STREAM_ON, &ret);
	if (ret)
		goto err_put;
	return 0;

err_put:
	pm_runtime_put_autosuspend(gc->dev);
	return ret;
}

static int gc5035_disable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct gc5035 *gc = to_gc5035(sd);
	int ret = 0;

	cci_write(gc->regmap, GC5035_REG_PAGE, 0, &ret);
	cci_write(gc->regmap, GC5035_REG_STREAM, GC5035_STREAM_OFF, &ret);
	pm_runtime_put_autosuspend(gc->dev);
	return ret;
}

static const struct v4l2_subdev_video_ops gc5035_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gc5035_pad_ops = {
	.enum_mbus_code = gc5035_enum_mbus_code,
	.enum_frame_size = gc5035_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc5035_set_fmt,
	.get_selection = gc5035_get_selection,
	.enable_streams = gc5035_enable_streams,
	.disable_streams = gc5035_disable_streams,
};

static const struct v4l2_subdev_ops gc5035_subdev_ops = {
	.video = &gc5035_video_ops,
	.pad = &gc5035_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc5035_internal_ops = {
	.init_state = gc5035_init_state,
};

static int gc5035_power_on(struct device *dev)
{
	struct gc5035 *gc = to_gc5035(dev_get_drvdata(dev));
	int ret;

	gpiod_set_value_cansleep(gc->reset_gpio, 1);

	ret = regulator_bulk_enable(GC5035_NUM_SUPPLIES, gc->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}

	ret = clk_prepare_enable(gc->xclk);
	if (ret) {
		dev_err(dev, "failed to enable the clock\n");
		regulator_bulk_disable(GC5035_NUM_SUPPLIES, gc->supplies);
		return ret;
	}

	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(gc->reset_gpio, 0);

	/* 8192 clock cycles before the first register access (~0.35 ms), plus margin */
	usleep_range(5000, 6000);
	return 0;
}

static int gc5035_power_off(struct device *dev)
{
	struct gc5035 *gc = to_gc5035(dev_get_drvdata(dev));

	gpiod_set_value_cansleep(gc->reset_gpio, 1);
	clk_disable_unprepare(gc->xclk);
	regulator_bulk_disable(GC5035_NUM_SUPPLIES, gc->supplies);
	return 0;
}

static int gc5035_check_hwcfg(struct device *dev)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *endpoint;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	if (ep.bus.mipi_csi2.num_data_lanes != GC5035_LANES) {
		dev_err(dev, "only %d data lanes are supported\n", GC5035_LANES);
		ret = -EINVAL;
	} else if (ep.nr_of_link_frequencies != 1 ||
		   ep.link_frequencies[0] != GC5035_LINK_FREQ) {
		dev_err(dev, "link-frequencies must be %lld\n", GC5035_LINK_FREQ);
		ret = -EINVAL;
	}

	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

/* DEBUG: with the sensor powered, list every 7-bit address that ACKs on this bus */
static void gc5035_scan_bus(struct i2c_client *client)
{
	u8 buf;
	int a;

	for (a = 0x08; a < 0x78; a++) {
		struct i2c_msg m = { .addr = a, .flags = I2C_M_RD, .len = 1, .buf = &buf };

		if (a == client->addr)
			continue;
		if (i2c_transfer(client->adapter, &m, 1) == 1)
			dev_info(&client->dev, "scan: ACK at 0x%02x\n", a);
	}
}

static int gc5035_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc5035 *gc;
	u64 id;
	int ret, i;

	gc = devm_kzalloc(dev, sizeof(*gc), GFP_KERNEL);
	if (!gc)
		return -ENOMEM;
	gc->dev = dev;

	ret = gc5035_check_hwcfg(dev);
	if (ret)
		return ret;

	gc->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(gc->regmap))
		return dev_err_probe(dev, PTR_ERR(gc->regmap), "cci regmap\n");

	gc->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gc->xclk))
		return dev_err_probe(dev, PTR_ERR(gc->xclk), "failed to get the clock\n");
	if (clk_get_rate(gc->xclk) != GC5035_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "unsupported clock rate %lu\n",
				     clk_get_rate(gc->xclk));

	for (i = 0; i < GC5035_NUM_SUPPLIES; i++)
		gc->supplies[i].supply = gc5035_supply_names[i];
	ret = devm_regulator_bulk_get(dev, GC5035_NUM_SUPPLIES, gc->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the supplies\n");

	gc->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gc->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc->reset_gpio), "reset gpio\n");

	v4l2_i2c_subdev_init(&gc->sd, client, &gc5035_subdev_ops);
	gc->sd.internal_ops = &gc5035_internal_ops;

	ret = gc5035_power_on(dev);
	if (ret)
		return ret;

	ret = cci_read(gc->regmap, GC5035_REG_CHIP_ID, &id, NULL);
	if (ret) {
		dev_err(dev, "failed to read the chip id (%d)\n", ret);
		gc5035_scan_bus(client);
		goto err_power_off;
	}
	if (id != GC5035_CHIP_ID) {
		dev_err(dev, "unexpected chip id 0x%04llx\n", id);
		ret = -ENODEV;
		goto err_power_off;
	}
	dev_info(dev, "detected GC%04llx\n", id);

	ret = gc5035_init_controls(gc);
	if (ret)
		goto err_power_off;

	gc->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&gc->sd.entity, 1, &gc->pad);
	if (ret)
		goto err_ctrls;

	gc->sd.state_lock = gc->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&gc->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&gc->sd);
	if (ret) {
		dev_err(dev, "failed to register the subdev (%d)\n", ret);
		goto err_cleanup;
	}
	return 0;

err_cleanup:
	v4l2_subdev_cleanup(&gc->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_entity:
	media_entity_cleanup(&gc->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&gc->ctrls);
err_power_off:
	gc5035_power_off(dev);
	return ret;
}

static void gc5035_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc5035 *gc = to_gc5035(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc->ctrls);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		gc5035_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(gc5035_pm_ops, gc5035_power_off, gc5035_power_on, NULL);

static const struct of_device_id gc5035_of_match[] = {
	{ .compatible = "galaxycore,gc5035" },
	{ }
};
MODULE_DEVICE_TABLE(of, gc5035_of_match);

static struct i2c_driver gc5035_driver = {
	.driver = {
		.name = "gc5035",
		.of_match_table = gc5035_of_match,
		.pm = pm_ptr(&gc5035_pm_ops),
	},
	.probe = gc5035_probe,
	.remove = gc5035_remove,
};
module_i2c_driver(gc5035_driver);

MODULE_DESCRIPTION("GalaxyCore GC5035 sensor driver");
MODULE_LICENSE("GPL");
