// SPDX-License-Identifier: GPL-2.0
/*
 * Fitipower FP5529 voice-coil motor (autofocus actuator) driver.
 *
 * Register map and power-up sequence follow the Samsung Exynos is-actuator-fp5529
 * driver.  Used on the rear main camera (S5KGW3) of the Galaxy M52 5G.
 * Exposes V4L2_CID_FOCUS_ABSOLUTE (10 bit).
 *
 * m52xq: the coil holds the lens with a current that grows with the position code. The driver used
 * to keep it at the last position for ever (also through suspend: Megapixels left it at 345 of
 * 1023), and on a reboot the lens fell from there onto its stop with an audible click. Now the lens
 * is walked down to 0 and the driver put into power-down (PD) whenever no one has the sub-device
 * open, before system suspend and at shutdown/reboot ("soft landing", as the stock camera does);
 * opening the sub-device powers it up again and returns the lens to its last position.
 * The control handler lock is taken directly: v4l2_ctrl_find() takes it itself, so
 * v4l2_ctrl_unlock(v4l2_ctrl_find(...)) while holding it deadlocked the boot (v222).
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#define FP5529_REG_IC_INFO	0x00	/* [7:4] manufacturer, [3:0] model, default 0xe1 */
#define FP5529_REG_CONTROL	0x02	/* [1] RING, [0] PD */
#define FP5529_REG_VCM_MSB	0x03	/* [1:0] pos[9:8], followed by LSB at 0x04 */
#define FP5529_REG_ACC_MODE	0x06
#define FP5529_REG_ACC_TIME	0x07

#define FP5529_MAX_FOCUS_POS	1023

struct fp5529 {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrls;
	struct regulator_bulk_data supplies[2];
	u32 initial_pos;
	u32 pos;
	u32 saved_pos;		/* m52xq: position to return to after a power-down */
	int open_count;
	bool parked;		/* lens at 0 and the driver in power-down */
};

static inline struct fp5529 *sd_to_fp5529(struct v4l2_subdev *sd)
{
	return container_of(sd, struct fp5529, sd);
}

static int fp5529_write(struct i2c_client *client, u8 reg, const u8 *val, int len)
{
	u8 buf[3] = { reg };
	int ret;

	memcpy(&buf[1], val, len);
	ret = i2c_master_send(client, buf, len + 1);
	return ret == len + 1 ? 0 : (ret < 0 ? ret : -EIO);
}

static int fp5529_write8(struct i2c_client *client, u8 reg, u8 val)
{
	return fp5529_write(client, reg, &val, 1);
}

static int fp5529_read8(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = 1, .buf = &reg },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = 1, .buf = val },
	};

	return i2c_transfer(client->adapter, msgs, 2) == 2 ? 0 : -EIO;
}

/*
 * The coil moved in one jump (up to the whole 1023 code range) is clearly audible and rings; stock walks the lens along a
 * soft landing curve. Walk it in slew_step codes every slew_delay_us microseconds (slew_step = 0 jumps like before). Both
 * can be changed at run time: /sys/module/fp5529/parameters/.
 */
static unsigned int slew_step = 12;
module_param(slew_step, uint, 0644);
MODULE_PARM_DESC(slew_step, "VCM codes per step while moving (0 = jump)");
static unsigned int slew_delay_us = 3000;
module_param(slew_delay_us, uint, 0644);
MODULE_PARM_DESC(slew_delay_us, "delay between steps, microseconds");

static int fp5529_set_position(struct i2c_client *client, u32 pos)
{
	u8 v[2] = { (pos >> 8) & 0x03, pos & 0xff };

	return fp5529_write(client, FP5529_REG_VCM_MSB, v, 2);
}

static int fp5529_hw_init(struct i2c_client *client);

/* walk the lens from vcm->pos to target in slew_step codes (at least 8, so that the landing is never a jump) */
static int fp5529_walk(struct fp5529 *vcm, struct i2c_client *client, u32 target, u32 step, u32 delay)
{
	int ret;

	while (step && vcm->pos != target) {
		u32 pos = vcm->pos;
		u32 diff = pos > target ? pos - target : target - pos;

		if (diff <= step)
			break;
		pos = pos > target ? pos - step : pos + step;
		ret = fp5529_set_position(client, pos);
		if (ret)
			return ret;
		vcm->pos = pos;
		usleep_range(delay, delay + 500);
	}
	ret = fp5529_set_position(client, target);
	if (!ret)
		vcm->pos = target;
	return ret;
}

/* m52xq: soft landing + power-down; called with the control handler lock held */
static void fp5529_park(struct fp5529 *vcm)
{
	struct i2c_client *client = v4l2_get_subdevdata(&vcm->sd);

	if (vcm->parked)
		return;
	vcm->saved_pos = vcm->pos;
	fp5529_walk(vcm, client, 0, max(READ_ONCE(slew_step), 8U), max(READ_ONCE(slew_delay_us), 2000U));
	usleep_range(5000, 6000);	/* let the lens settle at the stop before the current goes */
	fp5529_write8(client, FP5529_REG_CONTROL, 0x01);
	vcm->parked = true;
}

static int fp5529_unpark(struct fp5529 *vcm)
{
	struct i2c_client *client = v4l2_get_subdevdata(&vcm->sd);
	int ret;

	if (!vcm->parked)
		return 0;
	ret = fp5529_hw_init(client);
	if (ret)
		return ret;
	vcm->pos = 0;
	ret = fp5529_set_position(client, 0);
	if (ret)
		return ret;
	vcm->parked = false;
	return fp5529_walk(vcm, client, vcm->saved_pos, READ_ONCE(slew_step), READ_ONCE(slew_delay_us));
}

static int fp5529_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct fp5529 *vcm = container_of(ctrl->handler, struct fp5529, ctrls);
	struct i2c_client *client = v4l2_get_subdevdata(&vcm->sd);
	u32 target = ctrl->val;
	u32 step = READ_ONCE(slew_step);
	u32 delay = READ_ONCE(slew_delay_us);
	int ret;

	if (ctrl->id != V4L2_CID_FOCUS_ABSOLUTE)
		return -EINVAL;

	if (vcm->parked) {
		/* powered down: remember it, it is applied when the sub-device is opened again */
		vcm->saved_pos = target;
		return 0;
	}

	while (step && vcm->pos != target) {
		u32 pos = vcm->pos;
		u32 diff = pos > target ? pos - target : target - pos;

		if (diff <= step)
			break;
		pos = pos > target ? pos - step : pos + step;
		ret = fp5529_set_position(client, pos);
		if (ret)
			return ret;
		vcm->pos = pos;
		usleep_range(delay, delay + 500);
	}

	ret = fp5529_set_position(client, target);
	if (!ret)
		vcm->pos = target;
	return ret;
}

static const struct v4l2_ctrl_ops fp5529_ctrl_ops = {
	.s_ctrl = fp5529_s_ctrl,
};

static const struct v4l2_subdev_core_ops fp5529_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops fp5529_ops = {
	.core = &fp5529_core_ops,
};

static int fp5529_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct fp5529 *vcm = sd_to_fp5529(sd);
	int ret = 0;

	mutex_lock(vcm->ctrls.lock);
	if (vcm->open_count++ == 0)
		ret = fp5529_unpark(vcm);
	if (ret)
		vcm->open_count--;
	mutex_unlock(vcm->ctrls.lock);
	return ret;
}

static int fp5529_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct fp5529 *vcm = sd_to_fp5529(sd);

	mutex_lock(vcm->ctrls.lock);
	if (vcm->open_count > 0 && --vcm->open_count == 0)
		fp5529_park(vcm);
	mutex_unlock(vcm->ctrls.lock);
	return 0;
}

static const struct v4l2_subdev_internal_ops fp5529_internal_ops = {
	.open = fp5529_open,
	.close = fp5529_close,
};

static int fp5529_hw_init(struct i2c_client *client)
{
	int ret;

	/* PD on, then off (normal operation), wait 5 ms, ring mode, acceleration mode/time */
	ret = fp5529_write8(client, FP5529_REG_CONTROL, 0x01);
	if (ret)
		return ret;
	ret = fp5529_write8(client, FP5529_REG_CONTROL, 0x00);
	if (ret)
		return ret;
	usleep_range(5000, 6000);
	ret = fp5529_write8(client, FP5529_REG_CONTROL, 0x02);
	if (ret)
		return ret;
	ret = fp5529_write8(client, FP5529_REG_ACC_MODE, 0x01);
	if (ret)
		return ret;
	return fp5529_write8(client, FP5529_REG_ACC_TIME, 0x36);
}

static int fp5529_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fp5529 *vcm;
	u8 id = 0;
	int ret;

	vcm = devm_kzalloc(dev, sizeof(*vcm), GFP_KERNEL);
	if (!vcm)
		return -ENOMEM;

	vcm->supplies[0].supply = "vio";
	vcm->supplies[1].supply = "vcc";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(vcm->supplies), vcm->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	device_property_read_u32(dev, "fitipower,initial-position", &vcm->initial_pos);
	if (vcm->initial_pos > FP5529_MAX_FOCUS_POS)
		vcm->initial_pos = FP5529_MAX_FOCUS_POS;

	ret = regulator_bulk_enable(ARRAY_SIZE(vcm->supplies), vcm->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");
	usleep_range(5000, 6000);

	ret = fp5529_read8(client, FP5529_REG_IC_INFO, &id);
	if (ret) {
		dev_err(dev, "no response from the VCM\n");
		goto err_power;
	}
	dev_info(dev, "FP5529 IC info 0x%02x\n", id);

	ret = fp5529_hw_init(client);
	if (ret)
		goto err_power;
	ret = fp5529_set_position(client, vcm->initial_pos);
	if (ret)
		goto err_power;
	vcm->pos = vcm->initial_pos;

	v4l2_i2c_subdev_init(&vcm->sd, client, &fp5529_ops);
	vcm->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	vcm->sd.internal_ops = &fp5529_internal_ops;

	v4l2_ctrl_handler_init(&vcm->ctrls, 1);
	v4l2_ctrl_new_std(&vcm->ctrls, &fp5529_ctrl_ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, FP5529_MAX_FOCUS_POS, 1, vcm->initial_pos);
	if (vcm->ctrls.error) {
		ret = vcm->ctrls.error;
		goto err_ctrls;
	}
	vcm->sd.ctrl_handler = &vcm->ctrls;

	ret = media_entity_pads_init(&vcm->sd.entity, 0, NULL);
	if (ret)
		goto err_ctrls;
	vcm->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = v4l2_async_register_subdev(&vcm->sd);
	if (ret)
		goto err_entity;
	/* m52xq: nobody uses the lens yet: rest at 0 in power-down */
	mutex_lock(vcm->ctrls.lock);
	fp5529_park(vcm);
	mutex_unlock(vcm->ctrls.lock);
	return 0;

err_entity:
	media_entity_cleanup(&vcm->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&vcm->ctrls);
err_power:
	regulator_bulk_disable(ARRAY_SIZE(vcm->supplies), vcm->supplies);
	return ret;
}

static void fp5529_remove(struct i2c_client *client)
{
	struct fp5529 *vcm = sd_to_fp5529(i2c_get_clientdata(client));

	v4l2_async_unregister_subdev(&vcm->sd);
	v4l2_ctrl_handler_free(&vcm->ctrls);
	media_entity_cleanup(&vcm->sd.entity);
	fp5529_write8(client, FP5529_REG_CONTROL, 0x01);
	regulator_bulk_disable(ARRAY_SIZE(vcm->supplies), vcm->supplies);
}

static void fp5529_shutdown(struct i2c_client *client)
{
	struct fp5529 *vcm = sd_to_fp5529(i2c_get_clientdata(client));

	/* m52xq: soft landing before the rails go down on reboot / power-off (no click) */
	mutex_lock(vcm->ctrls.lock);
	fp5529_park(vcm);
	mutex_unlock(vcm->ctrls.lock);
}

static int fp5529_suspend(struct device *dev)
{
	struct fp5529 *vcm = sd_to_fp5529(dev_get_drvdata(dev));

	mutex_lock(vcm->ctrls.lock);
	fp5529_park(vcm);
	mutex_unlock(vcm->ctrls.lock);
	return 0;
}

static int fp5529_resume(struct device *dev)
{
	struct fp5529 *vcm = sd_to_fp5529(dev_get_drvdata(dev));
	int ret = 0;

	mutex_lock(vcm->ctrls.lock);
	if (vcm->open_count > 0)
		ret = fp5529_unpark(vcm);
	mutex_unlock(vcm->ctrls.lock);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fp5529_pm_ops, fp5529_suspend, fp5529_resume);

static const struct of_device_id fp5529_of_table[] = {
	{ .compatible = "fitipower,fp5529" },
	{ }
};
MODULE_DEVICE_TABLE(of, fp5529_of_table);

static struct i2c_driver fp5529_driver = {
	.driver = {
		.name = "fp5529",
		.of_match_table = fp5529_of_table,
		.pm = pm_sleep_ptr(&fp5529_pm_ops),
	},
	.probe = fp5529_probe,
	.remove = fp5529_remove,
	.shutdown = fp5529_shutdown,
};
module_i2c_driver(fp5529_driver);

MODULE_DESCRIPTION("Fitipower FP5529 VCM driver");
MODULE_LICENSE("GPL");
