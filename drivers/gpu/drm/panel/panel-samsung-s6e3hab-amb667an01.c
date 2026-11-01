// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 M52XQ mainline port project
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree
// (Samsung Galaxy M52 5G / SM-M526B "yupik", stock dtbo.img fragment@83),
// then hand-fixed (mode_flags command-mode bug, compatible string).

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct s6e3hab_amb667an01 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	bool dark;	/* M52XQ v229: display switched off by brightness 0 (panel still powered) */
	bool enabled;	/* M52XQ v230: between enable() and disable() */
	bool resume_dark;	/* M52XQ v230: dark because of the dark resume (not the in-call blanking) */
	bool asleep;		/* M52XQ v234: dark resume, the panel was not woken up at all (no light) */
};

/*
 * M52XQ v230: "dark resume" after a Wi-Fi wake-up. The system-sleep hook writes the number of seconds to
 * /sys/module/panel_samsung_s6e3hab_amb667an01/parameters/dark_resume before user space is thawed; until then the panel
 * keeps DCS display off although Plasma enables the output and restores the brightness after the resume (that was a
 * visible flash before m52xq-wifi-dark could switch it off again). Writing 0 (a notification or a key: the phone stays
 * awake) switches the display on at once; the time limit makes sure a crashed helper can never leave the screen dark.
 */
static struct s6e3hab_amb667an01 *m52xq_panel;
static bool dark_resume_on;
static unsigned long dark_resume_until;
static DEFINE_MUTEX(dark_resume_lock);

static bool dark_resume_active(void)
{
	return dark_resume_on && time_before(jiffies, dark_resume_until);
}

static int s6e3hab_amb667an01_set_dark(struct mipi_dsi_device *dsi, bool dark);
static void s6e3hab_amb667an01_wake(struct s6e3hab_amb667an01 *ctx);

/* called with dark_resume_lock held */
static void dark_resume_end(void)
{
	struct s6e3hab_amb667an01 *ctx = m52xq_panel;

	dark_resume_on = false;
	if (ctx && ctx->enabled && ctx->asleep) {
		dev_dbg(&ctx->dsi->dev, "M52XQ: dark resume ended, panel woken up\n");
		s6e3hab_amb667an01_wake(ctx);
	} else if (ctx && ctx->enabled && ctx->resume_dark) {
		dev_dbg(&ctx->dsi->dev, "M52XQ: dark resume ended, display on\n");
		s6e3hab_amb667an01_set_dark(ctx->dsi, false);
	}
}

static void dark_resume_expire(struct work_struct *work)
{
	mutex_lock(&dark_resume_lock);
	if (dark_resume_on && !time_before(jiffies, dark_resume_until))
		dark_resume_end();
	mutex_unlock(&dark_resume_lock);
}
static DECLARE_DELAYED_WORK(dark_resume_work, dark_resume_expire);

/*
 * M52XQ v232: the power and volume keys end a dark resume at once. In v230 only m52xq-wifi-dark could end it, and
 * it did not see the key presses: the phone looked dead (KWin enabled the output on every press, the panel stayed
 * off) until the 30 s window was over.
 */
static void dark_resume_key(struct work_struct *work)
{
	mutex_lock(&dark_resume_lock);
	if (dark_resume_on) {
		pr_debug("panel-s6e3hab: M52XQ: key pressed, dark resume ended\n");
		dark_resume_end();
	}
	mutex_unlock(&dark_resume_lock);
}
static DECLARE_WORK(dark_resume_key_work, dark_resume_key);

static void dark_resume_keys_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	if (type == EV_KEY && value == 1 && READ_ONCE(dark_resume_on) &&
	    (code == KEY_POWER || code == KEY_VOLUMEUP || code == KEY_VOLUMEDOWN))
		schedule_work(&dark_resume_key_work);
}

static int dark_resume_keys_connect(struct input_handler *handler, struct input_dev *dev,
				    const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "m52xq-dark-resume";
	ret = input_register_handle(handle);
	if (ret)
		goto err_free;
	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return ret;
}

static void dark_resume_keys_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dark_resume_keys_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(KEY_VOLUMEDOWN)] = BIT_MASK(KEY_VOLUMEDOWN) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(KEY_VOLUMEUP)] = BIT_MASK(KEY_VOLUMEUP) },
	},
	{ },
};

static struct input_handler dark_resume_keys_handler = {
	.event		= dark_resume_keys_event,
	.connect	= dark_resume_keys_connect,
	.disconnect	= dark_resume_keys_disconnect,
	.name		= "m52xq-dark-resume",
	.id_table	= dark_resume_keys_ids,
};
static bool dark_resume_keys_registered;

static inline
struct s6e3hab_amb667an01 *to_s6e3hab_amb667an01(struct drm_panel *panel)
{
	return container_of_const(panel, struct s6e3hab_amb667an01, panel);
}

static void s6e3hab_amb667an01_reset(struct s6e3hab_amb667an01 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
}

static int s6e3hab_amb667an01_on(struct s6e3hab_amb667an01 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	struct drm_dsc_picture_parameter_set pps;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	/*
	 * Match the panel's downstream on-command sequence: program PPS
	 * with the 9f/f0/fc test keys unlocked, before exiting sleep mode.
	 * The DSI host has populated ctx->dsc before panel prepare runs.
	 */
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0xa9, 0x96);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x96,
					 0x04, 0xfd, 0x20, 0x00, 0x00, 0x50,
					 0x55);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x97, 0x0a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x28, 0x98);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x98,
					 0x00, 0x51, 0x00, 0x61, 0x55);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x73, 0x91);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x91, 0x01, 0x7f);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x0b, 0xf2);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf2, 0x00, 0x10);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_msleep(&dsi_ctx, 55);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x0b, 0xf2);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf2, 0x09, 0x90);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x0c, 0xef);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xef, 0x46);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xed, 0x06, 0x4c);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb9,
					 0x00, 0xc0, 0x81, 0x09, 0x00, 0x00,
					 0x00, 0x11, 0x03);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x3c, 0x91);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x91, 0x02);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x0b, 0xb1);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x08);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x53, 0xb5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb5, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x54, 0xb5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb5, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x57, 0xb5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb5, 0x00);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x60, 0x20);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x06, 0xf3);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf3, 0xc0);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x18, 0xf6);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf6, 0x01);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x53, 0x20);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x02, 0xb5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb5, 0x56);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x0b, 0xb1);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb1, 0x08);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb0, 0x0b, 0x92);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x92, 0x6d);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xb5, 0x19);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x51, 0x07, 0xff);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf7, 0x0f);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0x5a, 0x5a);

	return dsi_ctx.accum_err;
}

static int s6e3hab_amb667an01_off(struct s6e3hab_amb667an01 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0xa5, 0xa5);
	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0x5a, 0x5a);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0xa5, 0xa5);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0x5a, 0x5a);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int s6e3hab_amb667an01_prepare(struct drm_panel *panel)
{
	struct s6e3hab_amb667an01 *ctx = to_s6e3hab_amb667an01(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	/*
	 * M52XQ v234: during a dark resume the panel is not woken up at all: its reset + sleep-out + init sequence
	 * (with the brightness set to the maximum) lit the OLED for a fraction of a second although the display
	 * itself stayed off (seen on 23.10). It stays in sleep/reset; dark_resume_end() wakes it if the phone
	 * really wakes up (key, notification).
	 */
	mutex_lock(&dark_resume_lock);
	if (dark_resume_active()) {
		ctx->asleep = true;
		mutex_unlock(&dark_resume_lock);
		dev_dbg(dev, "M52XQ: dark resume, panel kept asleep\n");
		return 0;
	}
	mutex_unlock(&dark_resume_lock);

	s6e3hab_amb667an01_reset(ctx);

	ret = s6e3hab_amb667an01_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	msleep(28); /* TODO: Is this panel-dependent? */

	return 0;
}

/*
 * M52XQ: the panel never actually turns its display output on without
 * this -- downstream sends it as its own dedicated "display_on_tx_cmds"
 * sequence, separate from the main on-command init sequence in prepare().
 * Without it, DSI/DPU commits complete successfully (no kernel-side
 * timeout) but nothing ever appears on the physical panel -- confirmed
 * live: no picture, no backlight. Matches every sibling S6E3-family
 * mainline panel driver (s6e3ha8, s6e3ha2, s6e3fc2x01), which all call
 * mipi_dsi_dcs_set_display_on() from a dedicated .enable, not .prepare --
 * DRM's panel-backlight follower ties backlight power state to
 * enable/disable, not prepare/unprepare, so backlight_get_brightness()
 * would report 0 regardless of the configured brightness without this.
 */
static int s6e3hab_amb667an01_enable(struct drm_panel *panel)
{
	struct s6e3hab_amb667an01 *ctx = to_s6e3hab_amb667an01(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/*
	 * M52XQ: downstream's display_on_tx_cmds_revA wraps the DCS
	 * SET_DISPLAY_ON in the same 0x9f test-key unlock/lock pair used
	 * everywhere else in this driver (see _off(), which already models
	 * this correctly for SET_DISPLAY_OFF). Sending a bare DCS 0x29 with
	 * no surrounding unlock is ACKed by the panel but silently ignored --
	 * confirmed live via get_power_mode() reporting Display OFF (bit 2 of
	 * 0x99) despite a successful, error-free DCS write.
	 */
	mutex_lock(&dark_resume_lock);
	ctx->enabled = true;
	if (ctx->asleep || dark_resume_active()) {
		ctx->dark = true;
		ctx->resume_dark = true;
		mutex_unlock(&dark_resume_lock);
		dev_dbg(&ctx->dsi->dev, "M52XQ: dark resume, display kept off\n");
		return 0;
	}
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0xa5, 0xa5);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0x5a, 0x5a);
	ctx->dark = false;
	ctx->resume_dark = false;
	mutex_unlock(&dark_resume_lock);

	if (dsi_ctx.accum_err < 0)
		dev_err(&ctx->dsi->dev, "failed to set display on: %d\n",
			dsi_ctx.accum_err);

	return dsi_ctx.accum_err;
}

static int s6e3hab_amb667an01_disable(struct drm_panel *panel)
{
	struct s6e3hab_amb667an01 *ctx = to_s6e3hab_amb667an01(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	mutex_lock(&dark_resume_lock);
	ctx->enabled = false;
	if (ctx->asleep) {
		mutex_unlock(&dark_resume_lock);
		return 0;	/* v234: the panel sleeps, nothing to switch off */
	}
	mutex_unlock(&dark_resume_lock);

	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0)
		dev_err(dev, "failed to set display off: %d\n", ret);

	return ret;
}

static int s6e3hab_amb667an01_unprepare(struct drm_panel *panel)
{
	struct s6e3hab_amb667an01 *ctx = to_s6e3hab_amb667an01(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	mutex_lock(&dark_resume_lock);
	if (ctx->asleep) {
		ctx->asleep = false;	/* v234: it never woke up, it is still in sleep and reset */
		mutex_unlock(&dark_resume_lock);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return 0;
	}
	mutex_unlock(&dark_resume_lock);

	ret = s6e3hab_amb667an01_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const u16 s6e3hab_amb667an01_dbv[256];

/* v234: the panel kept asleep during a dark resume is really woken up (called with dark_resume_lock held) */
static void s6e3hab_amb667an01_wake(struct s6e3hab_amb667an01 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	struct backlight_device *bl = ctx->panel.backlight;
	u16 level = bl ? min_t(u16, backlight_get_brightness(bl), 255) : 255;

	s6e3hab_amb667an01_reset(ctx);
	if (s6e3hab_amb667an01_on(ctx) < 0) {
		dev_err(&dsi->dev, "M52XQ: waking the panel up failed\n");
		return;
	}
	msleep(28);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0xa5, 0xa5);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0x5a, 0x5a);
	ctx->asleep = false;
	ctx->dark = false;
	ctx->resume_dark = false;
	/* the init sequence sets the maximum brightness: back to the user's level */
	if (level)
		mipi_dsi_dcs_set_display_brightness_large(dsi, s6e3hab_amb667an01_dbv[level]);
}

static const struct drm_display_mode s6e3hab_amb667an01_mode = {
	.clock = (1080 + 60 + 4 + 60) * (2400 + 40 + 4 + 20) * 120 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 60,
	.hsync_end = 1080 + 60 + 4,
	.htotal = 1080 + 60 + 4 + 60,
	.vdisplay = 2400,
	.vsync_start = 2400 + 40,
	.vsync_end = 2400 + 40 + 4,
	.vtotal = 2400 + 40 + 4 + 20,
	.width_mm = 69,
	.height_mm = 154,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int s6e3hab_amb667an01_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &s6e3hab_amb667an01_mode);
}

static const struct drm_panel_funcs s6e3hab_amb667an01_panel_funcs = {
	.prepare = s6e3hab_amb667an01_prepare,
	.unprepare = s6e3hab_amb667an01_unprepare,
	.enable = s6e3hab_amb667an01_enable,
	.disable = s6e3hab_amb667an01_disable,
	.get_modes = s6e3hab_amb667an01_get_modes,
};

/*
 * Samsung maps the 0..255 user brightness level to the 11-bit DBV (0x51)
 * value through samsung,candela_map_table_revA of the vendor panel node
 * (level 255 = DBV 0x7ff = 420 nit). Levels above 255 are HBM and need
 * separate commands, so they are not exposed here.
 */
static const u16 s6e3hab_amb667an01_dbv[256] = {
	10, 12, 14, 16, 19, 22, 26, 29, 33, 36, 40, 44,
	48, 53, 57, 61, 66, 70, 75, 80, 84, 89, 94, 99,
	104, 110, 115, 120, 125, 131, 136, 142, 147, 153, 158, 164,
	170, 176, 182, 187, 193, 199, 205, 211, 218, 224, 230, 236,
	242, 249, 255, 261, 268, 274, 281, 287, 294, 301, 307, 314,
	321, 327, 334, 341, 348, 355, 362, 369, 376, 383, 390, 397,
	404, 411, 418, 426, 433, 441, 448, 455, 463, 470, 478, 486,
	493, 501, 509, 516, 524, 532, 539, 547, 555, 563, 571, 579,
	587, 595, 602, 610, 618, 627, 635, 643, 651, 659, 667, 675,
	683, 692, 700, 708, 717, 725, 733, 742, 750, 758, 767, 775,
	784, 792, 801, 809, 818, 826, 835, 844, 852, 861, 870, 878,
	887, 896, 904, 913, 921, 930, 939, 947, 956, 964, 973, 981,
	990, 999, 1007, 1016, 1025, 1034, 1042, 1051, 1060, 1069, 1078, 1086,
	1095, 1104, 1113, 1122, 1131, 1140, 1149, 1158, 1167, 1176, 1185, 1194,
	1203, 1212, 1221, 1230, 1239, 1248, 1258, 1267, 1276, 1285, 1294, 1304,
	1313, 1322, 1331, 1341, 1350, 1359, 1369, 1378, 1387, 1397, 1406, 1416,
	1425, 1435, 1444, 1453, 1463, 1472, 1482, 1492, 1501, 1511, 1520, 1530,
	1539, 1549, 1559, 1568, 1578, 1588, 1597, 1607, 1617, 1626, 1636, 1646,
	1656, 1666, 1675, 1685, 1695, 1705, 1715, 1725, 1734, 1744, 1754, 1764,
	1774, 1784, 1794, 1804, 1814, 1824, 1834, 1844, 1854, 1864, 1874, 1884,
	1894, 1904, 1914, 1925, 1935, 1945, 1955, 1965, 1975, 1986, 1996, 2006,
	2016, 2026, 2037, 2047,
};

/*
 * Send the brightness command in LP mode by default: switching to HS for
 * it while the command-mode panel is being refreshed produced DSI FIFO
 * errors and visible artifacts while dragging the slider.
 */
static bool bl_hs;
module_param(bl_hs, bool, 0644);
MODULE_PARM_DESC(bl_hs, "Send brightness DCS in HS mode (default: LP)");

static int dark_resume_set(const char *val, const struct kernel_param *kp)
{
	unsigned int sec;
	int ret;

	ret = kstrtouint(val, 0, &sec);
	if (ret)
		return ret;
	mutex_lock(&dark_resume_lock);
	if (sec) {
		dark_resume_on = true;
		dark_resume_until = jiffies + msecs_to_jiffies(min(sec, 120U) * 1000);
		mod_delayed_work(system_wq, &dark_resume_work, dark_resume_until - jiffies + 1);
	} else {
		dark_resume_end();
	}
	mutex_unlock(&dark_resume_lock);
	return 0;
}

static int dark_resume_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%u\n", dark_resume_active() ?
			  jiffies_to_msecs(dark_resume_until - jiffies) / 1000 + 1 : 0);
}

static const struct kernel_param_ops dark_resume_ops = {
	.set = dark_resume_set,
	.get = dark_resume_get,
};
module_param_cb(dark_resume, &dark_resume_ops, NULL, 0644);
MODULE_PARM_DESC(dark_resume, "Keep the display off for N s (max 120) after a dark resume; 0 = display on now");

/*
 * M52XQ v229: brightness 0 set by user space (the in-call proximity service, at the ear) switches the display off with
 * DCS SET_DISPLAY_OFF while the panel stays powered, like the stock phone; brightness > 0 switches it on again. DBV level
 * 0 alone is the dimmest level (the picture stayed visible), and a real DPMS off (panel powered down) silences the
 * Samsung proximity sensor, so the call could never see "far" again. A blanked backlight (DPMS, panel disable) keeps the
 * old behaviour.
 */
static int s6e3hab_amb667an01_set_dark(struct mipi_dsi_device *dsi, bool dark)
{
	struct s6e3hab_amb667an01 *ctx = mipi_dsi_get_drvdata(dsi);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	if (!ctx || ctx->dark == dark)
		return 0;
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0xa5, 0xa5);
	if (dark)
		mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	else
		mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_generic_write_seq_multi(&dsi_ctx, 0x9f, 0x5a, 0x5a);
	if (dsi_ctx.accum_err < 0) {
		dev_err(&dsi->dev, "failed to set display %s: %d\n", dark ? "off" : "on", dsi_ctx.accum_err);
		return dsi_ctx.accum_err;
	}
	ctx->dark = dark;
	ctx->resume_dark = false;
	dev_dbg(&dsi->dev, "M52XQ: display %s by brightness\n", dark ? "off" : "on");
	return 0;
}

static int s6e3hab_amb667an01_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct s6e3hab_amb667an01 *ctx = mipi_dsi_get_drvdata(dsi);
	u16 level = min_t(u16, backlight_get_brightness(bl), 255);
	int ret;

	/* v234: the panel is asleep (dark resume): the level is applied when it is woken up */
	if (ctx && READ_ONCE(ctx->asleep))
		return 0;

	if (!backlight_is_blank(bl)) {
		if (bl->props.brightness == 0)
			return s6e3hab_amb667an01_set_dark(dsi, true);
		mutex_lock(&dark_resume_lock);
		ret = dark_resume_active() ? 0 : s6e3hab_amb667an01_set_dark(dsi, false);
		mutex_unlock(&dark_resume_lock);
		if (ret < 0)
			return ret;
	}

	if (bl_hs)
		dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi,
				s6e3hab_amb667an01_dbv[level]);

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret < 0 ? ret : 0;
}

static const struct backlight_ops s6e3hab_amb667an01_bl_ops = {
	.update_status = s6e3hab_amb667an01_bl_update_status,
};

static struct backlight_device *
s6e3hab_amb667an01_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 255,
		.max_brightness = 255,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &s6e3hab_amb667an01_bl_ops, &props);
}

static int s6e3hab_amb667an01_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e3hab_amb667an01 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e3hab_amb667an01, panel,
				   &s6e3hab_amb667an01_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);
	m52xq_panel = ctx;
	if (!dark_resume_keys_registered && !input_register_handler(&dark_resume_keys_handler))
		dark_resume_keys_registered = true;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* Command-mode panel (qcom,mdss-dsi-panel-type = "dsi_cmd_mode" in
	 * the downstream devicetree) -- MIPI_DSI_MODE_VIDEO_SYNC_PULSE from
	 * the generator's traffic-mode fallback does not apply here. */
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = s6e3hab_amb667an01_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	/* TODO: Pass slice_per_pkt = 2 */
	ctx->dsc.slice_height = 40;
	ctx->dsc.slice_width = 540;
	/*
	 * TODO: hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	WARN_ON(1080 % ctx->dsc.slice_width);
	ctx->dsc.slice_count = 1080 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void s6e3hab_amb667an01_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3hab_amb667an01 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	if (dark_resume_keys_registered) {
		input_unregister_handler(&dark_resume_keys_handler);
		dark_resume_keys_registered = false;
	}
	mutex_lock(&dark_resume_lock);
	m52xq_panel = NULL;
	mutex_unlock(&dark_resume_lock);
	cancel_delayed_work_sync(&dark_resume_work);
	cancel_work_sync(&dark_resume_key_work);
	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id s6e3hab_amb667an01_of_match[] = {
	{ .compatible = "samsung,s6e3hab-amb667an01" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s6e3hab_amb667an01_of_match);

static struct mipi_dsi_driver s6e3hab_amb667an01_driver = {
	.probe = s6e3hab_amb667an01_probe,
	.remove = s6e3hab_amb667an01_remove,
	.driver = {
		.name = "panel-s6e3hab-amb667an01",
		.of_match_table = s6e3hab_amb667an01_of_match,
	},
};
module_mipi_dsi_driver(s6e3hab_amb667an01_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for ss_dsi_panel_S6E3HAB_AMB667AN01_FHD");
MODULE_LICENSE("GPL");
