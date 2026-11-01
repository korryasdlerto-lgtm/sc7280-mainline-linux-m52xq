// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Driver for Synaptics TCM Oncell Touchscreens
 *
 *  Copyright (c) 2024 Frieder Hannenheim <frieder.hannenheim@proton.me>
 *  Copyright (c) 2024 Caleb Connolly <caleb@postmarketos.org>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

/*
 * The TCM oncell interface uses a command byte, which may be followed by additional
 * data. The packet format is defined in the tcm_cmd struct.
 *
 * The following list only defines commands that are used in this driver (and their
 * counterparts for context). Vendor reference implementations can be found at
 * https://github.com/LineageOS/android_kernel_oneplus_sm8250/tree/ee0a7ee1939ffd53000e42051caf8f0800defb27/drivers/input/touchscreen/synaptics_tcm
 */

/*
 * Request information about the chip. We don't send this command explicitly as
 * the controller automatically sends this information when starting up.
 */
#define TCM_IDENTIFY				0x02

/* Enable/disable reporting touch inputs */
#define TCM_ENABLE_REPORT			0x05
#define TCM_DISABLE_REPORT			0x06

/*
 * After powering on, we send this to exit the bootloader mode and run the main
 * firmware.
 */
#define TCM_RUN_APPLICATION_FIRMWARE		0x14

/*
 * Reports information about the vendor provided application firmware. This is
 * also used to determine when the firmware has finished booting.
 */
#define TCM_GET_APPLICATION_INFO		0x20

/* Debug: retrieve the currently active touch report config from the fw */
#define TCM_GET_TOUCH_REPORT_CONFIG		0x25
#define TCM_SET_TOUCH_REPORT_CONFIG		0x26
/* Recalibrate the capacitive baseline; some fw needs this before it will
 * start reporting real touches at all. */
#define TCM_REZERO				0x27

#define MODE_APPLICATION			0x01

#define APP_STATUS_OK				0x00
#define APP_STATUS_BOOTING			0x01
#define APP_STATUS_UPDATING			0x02

/* status codes */
#define REPORT_IDLE				0x00
#define REPORT_OK				0x01
#define REPORT_BUSY				0x02
#define REPORT_CONTINUED_READ			0x03
#define REPORT_RECEIVE_BUFFER_OVERFLOW		0x0c
#define REPORT_PREVIOUS_COMMAND_PENDING		0x0d
#define REPORT_NOT_IMPLEMENTED			0x0e
#define REPORT_ERROR				0x0f

/* report types */
#define REPORT_IDENTIFY				0x10
#define REPORT_TOUCH				0x11
#define REPORT_DELTA				0x12
#define REPORT_RAW				0x13
#define REPORT_DEBUG				0x14
#define REPORT_LOG				0x1d
#define REPORT_TOUCH_HOLD			0x20
#define REPORT_INVALID				0xff

/*
 * M52XQ: this Samsung S3908 firmware reports touch under Samsung's own
 * "sec_input" framework report codes, not the generic TouchComm
 * REPORT_TOUCH -- confirmed 2026-09-11 night by fetching the real
 * production driver source (crdroidandroid/android_kernel_samsung_sm7325,
 * drivers/input/sec_input/synaptics/{synaptics_reg.h,synaptics_interrupt.c}).
 * REPORT_SEC_COORDINATE_EVENT (0xc1) is the real per-finger coordinate
 * push; 0xc0/0xc2 are gesture/status events (not handled here). Earlier
 * theory that 0xc1 == DC_PS_STATUS (a dynamic-config-id coincidentally
 * sharing the same number in the UNRELATED generic OnePlus/Oppo reference
 * driver family) was wrong for this chip/firmware.
 */
#define REPORT_SEC_COORDINATE_EVENT		0xc1

/* Bit layout confirmed against the real driver's struct sec_touch_event_data. */
struct sec_touch_event {
	u8 eid_tid_tchsta;	/* eid:2, tid:4, tchsta:2 (0=none 1=press 2=move 3=release) */
	u8 x_11_4;
	u8 y_11_4;
	u8 xy_3_0;		/* y_3_0:4, x_3_0:4 */
	u8 major;
	u8 minor;
	u8 z_ttype32;		/* z:6, ttype_3_2:2 */
	u8 left_ttype10;	/* left_event:5, max_energy_flag:1, ttype_1_0:2 */
	u8 noise_level;
	u8 max_strength;
	u8 hover_etc;		/* hover_id_num:4, noise_status:2, reserved:2 */
	u8 reserved[5];
} __packed; /* 16 bytes, matches the observed 0xc1 payload length exactly */

#define SEC_COORD_ACTION_NONE		0
#define SEC_COORD_ACTION_PRESS		1
#define SEC_COORD_ACTION_MOVE		2
#define SEC_COORD_ACTION_RELEASE	3

#define SEC_TOUCHTYPE_NORMAL		0
#define SEC_TOUCHTYPE_GLOVE		3
#define SEC_TOUCHTYPE_PALM		5
#define SEC_TOUCHTYPE_WET		6

struct tcm_message_header {
	u8 marker;
	u8 code;
	__le16 length;
} __packed;

struct tcm_cmd {
	u8 cmd;
	__le16 length;
	u8 data[];
} __packed;

struct tcm_identification {
	struct tcm_message_header header;
	u8 version;
	u8 mode;
	char part_number[16];
	u8 build_id[4];
	u8 max_write_size[2];
} __packed;

struct tcm_app_info {
	struct tcm_message_header header;
	u8 version[2];
	__le16 status;
	u8 static_config_size[2];
	u8 dynamic_config_size[2];
	u8 app_config_start_write_block[2];
	u8 app_config_size[2];
	u8 max_touch_report_config_size[2];
	u8 max_touch_report_payload_size[2];
	char customer_config_id[16];
	__le16 max_x;
	__le16 max_y;
	u8 max_objects[2];
	u8 num_of_buttons[2];
	u8 num_of_image_rows[2];
	u8 num_of_image_cols[2];
	u8 has_hybrid_data[2];
} __packed;

struct tcm_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct completion response;
	struct touchscreen_properties props;
	struct regulator_bulk_data supplies[2];
	bool powered;	/* vdd/vcc enabled; close() powers the controller off like the Samsung driver's stop_device() */

	/* annoying state */
	u16 buf_size;
	char buf[256];
};

/*
 * M52XQ: the wire format for a command is [cmd_byte][data...] -- struct
 * tcm_cmd's "length" field is host-side bookkeeping only and must NOT be
 * transmitted. Build the actual i2c buffer explicitly instead of casting
 * the struct's own memory (which has "length" sitting between cmd and
 * data) directly to the wire -- that silently truncated/misaligned any
 * command that actually carried a payload (never exercised before, since
 * every command used so far was zero-argument).
 */
static int tcm_send_cmd(struct tcm_data *tcm, u8 cmd_byte,
			const void *data, u16 length)
{
	struct i2c_client *client = tcm->client;
	struct i2c_msg msg;
	u8 *buf;
	int ret;

	dev_dbg(&client->dev, "sending command %#x (%u bytes)\n", cmd_byte,
		length);

	buf = kmalloc(1 + length, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = cmd_byte;
	if (length)
		memcpy(&buf[1], data, length);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 1 + length;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	kfree(buf);
	if (ret == 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int tcm_send_cmd_noargs(struct tcm_data *tcm, u8 cmd)
{
	return tcm_send_cmd(tcm, cmd, NULL, 0);
}

static int tcm_recv_report(struct tcm_data *tcm,
			   void *buf, size_t length)
{
	struct i2c_client *client = tcm->client;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret == 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int tcm_read_message(struct tcm_data *tcm, u8 cmd, void *buf,
			    size_t length)
{
	int ret;

	reinit_completion(&tcm->response);
	ret = tcm_send_cmd_noargs(tcm, cmd);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&tcm->response,
					  msecs_to_jiffies(1000));
	if (ret == 0)
		return -ETIMEDOUT;

	if (buf) {
		if (length > tcm->buf_size) {
			dev_warn(&tcm->client->dev,
				 "expected %zu bytes, got %u\n",
				 length, tcm->buf_size);
		}
		length = min(tcm->buf_size, length);
		memcpy(buf, tcm->buf, length);
	}

	return 0;
}

/* Debug: send a command with a data payload and wait for its reply. */
static int tcm_write_message(struct tcm_data *tcm, u8 cmd_byte,
			     const void *data, size_t len)
{
	int ret;

	reinit_completion(&tcm->response);
	ret = tcm_send_cmd(tcm, cmd_byte, data, len);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&tcm->response,
					  msecs_to_jiffies(5000));
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

static int tcm_hw_init(struct tcm_data *tcm, u16 *max_x, u16 *max_y);
static int tcm_power_on(struct tcm_data *tcm);

static void tcm_power_off(void *data)
{
	struct tcm_data *tcm = data;

	disable_irq(tcm->client->irq);
	if (tcm->powered) {
		regulator_bulk_disable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
		tcm->powered = false;
	}
}

/*
 * M52XQ: the bare TCM_DISABLE_REPORT byte left the controller powered and scanning (measured: ~140 mA more during the
 * system sleep with the input device not inhibited, and +35..55 mA awake with it inhibited). The Samsung driver
 * (synaptics_ts_stop_device) powers the controller off when the screen goes off: do the same, keep the reset line low
 * while it is off, and bring it back (power, reset, hw init) in open().
 */
static int tcm_input_open(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);
	u16 max_x, max_y;
	int ret, attempt;

	if (tcm->powered)
		return i2c_smbus_write_byte(tcm->client, TCM_ENABLE_REPORT);

	for (attempt = 0; attempt < 5; attempt++) {
		ret = tcm_power_on(tcm);
		if (ret)
			return ret;
		tcm->powered = true;
		if (attempt == 0)
			enable_irq(tcm->client->irq);
		ret = tcm_hw_init(tcm, &max_x, &max_y);
		if (!ret)
			break;
		dev_warn(&tcm->client->dev, "hw_init after power-on failed (%d), power-cycling (%d/5)\n", ret, attempt + 1);
		regulator_bulk_disable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
		tcm->powered = false;
		msleep(50);
	}
	if (ret) {
		dev_err(&tcm->client->dev, "failed to bring the controller back: %d\n", ret);
		return ret;
	}

	return i2c_smbus_write_byte(tcm->client, TCM_ENABLE_REPORT);
}

static void tcm_input_close(struct input_dev *dev)
{
	struct tcm_data *tcm = input_get_drvdata(dev);

	if (!tcm->powered)
		return;

	disable_irq(tcm->client->irq);
	if (tcm->reset_gpio)
		gpiod_set_value_cansleep(tcm->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(tcm->supplies), tcm->supplies);
	tcm->powered = false;
}

/*
 * The default report config looks like this:
 *
 * a5 01 80 00 11 08 1e 08 0f 01 04 01 06 04 07 04
 * 08 0c 09 0c 0a 08 0b 08 0c 08 0d 10 0e 10 03 00
 * 00 00
 *
 * a5 01 80 00 - HEADER + length
 *
 * 11 08 - TOUCH_FRAME_RATE (8 bits)
 * 30 08 - UNKNOWN (8 bits)
 * 0f 01 - TOUCH_0D_BUTTONS_STATE (1 bit)
 * 04 01 - TOUCH_PAD_TO_NEXT_BYTE (7 bits - padding)
 * 06 04 - TOUCH_OBJECT_N_INDEX (4 bits)
 * 07 04 - TOUCH_OBJECT_N_CLASSIFICATION (4 bits)
 * 08 0c - TOUCH_OBJECT_N_X_POSITION (12 bits)
 * 09 0c - TOUCH_OBJECT_N_Y_POSITION (12 bits)
 * 0a 08 - TOUCH_OBJECT_N_Z (8 bits)
 * 0b 08 - TOUCH_OBJECT_N_X_WIDTH (8 bits)
 * 0c 08 - TOUCH_OBJECT_N_Y_WIDTH (8 bits)
 * 0d 10 - TOUCH_OBJECT_N_TX_POSITION_TIXELS (16 bits) ??
 * 0e 10 - TOUCH_OBJECT_N_RX_POSITION_TIXELS (16 bits) ??
 * 03 00 - TOUCH_FOREACH_END (0 bits)
 * 00 00 - TOUCH_END (0 bits)
 *
 * Since we only support this report config, we just hardcode the format below.
 * To support additional report configs, we would need to parse the config and
 * use it to parse the reports dynamically.
 */

struct tcm_report_point {
	u8 unknown;
	u8 buttons;
	__le32 point; /* idx : 4, class : 4, x : 12, y : 12 */
	u8 z;
	u8 width_x;
	u8 width_y;
	u8 tx;
	u8 rx;
} __packed;

static int tcm_handle_touch_report(struct tcm_data *tcm, const char *buf,
				   size_t len)
{
	const struct tcm_report_point *point;
	/* If the input device hasn't registered yet then we can't do anything */
	if (!tcm->input)
		return 0;

	buf += sizeof(struct tcm_message_header);
	len -= sizeof(struct tcm_message_header);

	dev_dbg(&tcm->client->dev, "touch report len %zu\n", len);
	if ((len - 3) % sizeof(*point))
		dev_err(&tcm->client->dev, "invalid touch report length\n");

	buf++; /* Skip the FPS report */

	/* We don't need to report releases because we have INPUT_MT_DROP_UNUSED */
	for (int i = 0; i < (len - 1) / sizeof(*point); i++) {
		u8 major_width, minor_width;
		u16 idx, x, y;
		u32 _point;

		point = (struct tcm_report_point *)buf;
		_point = le32_to_cpu(point->point);

		minor_width = point->width_x;
		major_width = point->width_y;

		if (minor_width > major_width)
			swap(major_width, minor_width);

		idx = _point & 0xf;
		x = (_point >> 8) & 0xfff;
		y = (_point >> 20) & 0xfff;

		dev_dbg(&tcm->client->dev, "touch report: idx %u x %u y %u\n",
			idx, x, y);

		input_mt_slot(tcm->input, idx);
		input_mt_report_slot_state(tcm->input, MT_TOOL_FINGER, true);

		touchscreen_report_pos(tcm->input, &tcm->props, x, y, true);

		input_report_abs(tcm->input, ABS_MT_TOUCH_MAJOR, major_width);
		input_report_abs(tcm->input, ABS_MT_TOUCH_MINOR, minor_width);
		input_report_abs(tcm->input, ABS_MT_PRESSURE, point->z);

		buf += sizeof(*point);
	}

	input_mt_sync_frame(tcm->input);
	input_sync(tcm->input);

	return 0;
}

static int tcm_handle_sec_coordinate_event(struct tcm_data *tcm, const char *buf,
					   size_t len)
{
	const struct sec_touch_event *ev;
	size_t count, i;

	if (!tcm->input)
		return 0;

	buf += sizeof(struct tcm_message_header);
	len -= sizeof(struct tcm_message_header);

	/*
	 * M52XQ: one 0xc1 report carries ONE 16-byte event PER FINGER
	 * (Samsung's synaptics_ts_irq_thread loops over data_length / 16 events and
	 * then syncs once). Reading only the first event made two-finger gestures
	 * impossible: the second finger never reached the input layer.
	 */
	if (len < sizeof(*ev)) {
		dev_err(&tcm->client->dev, "sec coordinate event too short\n");
		return -EINVAL;
	}
	count = len / sizeof(*ev);

	for (i = 0; i < count; i++) {
		u8 tid, action, ttype;
		u16 x, y;
		u8 z, major, minor;

		ev = (const struct sec_touch_event *)(buf + i * sizeof(*ev));

		tid = (ev->eid_tid_tchsta >> 2) & 0xf;
		action = (ev->eid_tid_tchsta >> 6) & 0x3;
		x = ((u16)ev->x_11_4 << 4) | (ev->xy_3_0 >> 4);
		y = ((u16)ev->y_11_4 << 4) | (ev->xy_3_0 & 0xf);
		major = ev->major;
		minor = ev->minor;
		z = ev->z_ttype32 & 0x3f;
		ttype = ((ev->z_ttype32 >> 6) & 0x3) << 2 | ((ev->left_ttype10 >> 6) & 0x3);

		if (tid >= 10)
			continue;

		dev_dbg(&tcm->client->dev,
			"sec coord: tid %u action %u x %u y %u z %u ttype %u\n",
			tid, action, x, y, z, ttype);

		input_mt_slot(tcm->input, tid);

		if (action == SEC_COORD_ACTION_RELEASE) {
			input_mt_report_slot_state(tcm->input, MT_TOOL_FINGER, false);
		} else if (ttype == SEC_TOUCHTYPE_NORMAL || ttype == SEC_TOUCHTYPE_GLOVE ||
			   ttype == SEC_TOUCHTYPE_PALM || ttype == SEC_TOUCHTYPE_WET) {
			input_mt_report_slot_state(tcm->input, MT_TOOL_FINGER, true);
			input_report_abs(tcm->input, ABS_MT_POSITION_X, x);
			input_report_abs(tcm->input, ABS_MT_POSITION_Y, y);
			input_report_abs(tcm->input, ABS_MT_TOUCH_MAJOR, major);
			input_report_abs(tcm->input, ABS_MT_TOUCH_MINOR, minor);
			input_report_abs(tcm->input, ABS_MT_PRESSURE, z ?: 1);
		}
	}

	input_mt_sync_frame(tcm->input);
	input_sync(tcm->input);

	return 0;
}

static irqreturn_t tcm_report_irq(int irq, void *data)
{
	struct tcm_data *tcm = data;
	struct tcm_message_header *header;
	char buf[256];
	u16 len;
	int ret;

	header = (struct tcm_message_header *)buf;
	ret = tcm_recv_report(tcm, buf, sizeof(buf));
	if (ret) {
		dev_err(&tcm->client->dev, "failed to read report: %d\n", ret);
		return IRQ_HANDLED;
	}

	len = le16_to_cpu(header->length);

	dev_dbg(&tcm->client->dev, "report %#x len %u\n", header->code, len);
	print_hex_dump_bytes("report: ", DUMP_PREFIX_OFFSET, buf,
			     min(sizeof(buf), len + sizeof(*header)));

	if (len > sizeof(buf) - sizeof(*header)) {
		dev_err(&tcm->client->dev, "report too long\n");
		return IRQ_HANDLED;
	}

	/*
	 * M52XQ: the real Samsung driver for this exact chip/platform family
	 * (sm7325, PixelExperience-Devices/kernel_samsung_sm7325) uses a much
	 * simpler and correct classification than our previous
	 * completion_done()-based heuristic: any code >= REPORT_IDENTIFY
	 * (0x10) is UNCONDITIONALLY an unsolicited report/indication; any
	 * code < 0x10 is UNCONDITIONALLY the response to whatever command is
	 * currently pending. This matches the status_code (0x00-0x0f) vs
	 * report_type (0x10+) enum split exactly. Our old heuristic could
	 * misroute a genuine REPORT_TOUCH into the "response" path depending
	 * on unrelated completion state.
	 */
	if (header->code == REPORT_IDENTIFY) {
		/*
		 * Falls through to the response path below: tcm_hw_init()
		 * sends TCM_RUN_APPLICATION_FIRMWARE and treats the resulting
		 * IDENTIFY as its response, matching the real Samsung
		 * driver's explicit special-case for this exact situation
		 * (command_status == CMD_BUSY with command in {RESET,
		 * RUN_BOOTLOADER_FIRMWARE, RUN_APPLICATION_FIRMWARE,
		 * ENTER_PRODUCTION_TEST_MODE}).
		 */
	} else if (header->code >= REPORT_IDENTIFY) {
		if (header->code == REPORT_TOUCH)
			tcm_handle_touch_report(tcm, buf, len + sizeof(*header));
		else if (header->code == REPORT_SEC_COORDINATE_EVENT)
			tcm_handle_sec_coordinate_event(tcm, buf, len + sizeof(*header));
		else
			dev_dbg(&tcm->client->dev, "ignoring report %#x\n",
				header->code);
		return IRQ_HANDLED;
	}

	tcm->buf_size = len + sizeof(*header);
	memcpy(tcm->buf, buf, len + sizeof(*header));
	complete(&tcm->response);

	return IRQ_HANDLED;
}

static int tcm_hw_init(struct tcm_data *tcm, u16 *max_x, u16 *max_y)
{
	int ret;
	struct tcm_identification id = { 0 };
	struct tcm_app_info app_info = { 0 };
	u16 status;

	/*
	 * Tell the firmware to start up. After starting it sends an IDENTIFY
	 * report, which we treat like a response to this message even though
	 * it's technically a new report.
	 */
	ret = tcm_read_message(tcm, TCM_RUN_APPLICATION_FIRMWARE, &id,
			       sizeof(id));
	if (ret) {
		dev_err(&tcm->client->dev, "failed to identify device: %d\n",
			ret);
		return ret;
	}

	dev_dbg(&tcm->client->dev, "Synaptics TCM %s v%d mode %d\n",
		id.part_number, id.version, id.mode);
	if (id.mode != MODE_APPLICATION) {
		/* We don't support firmware updates or anything else */
		dev_err(&tcm->client->dev, "Device is not in application mode\n");
		return -ENODEV;
	}

	do {
		msleep(20);
		ret = tcm_read_message(tcm, TCM_GET_APPLICATION_INFO,
				       &app_info, sizeof(app_info));
		if (ret) {
			dev_err(&tcm->client->dev,
				"failed to get application info: %d\n", ret);
			return ret;
		}
		status = le16_to_cpu(app_info.status);
	} while (status == APP_STATUS_BOOTING || status == APP_STATUS_UPDATING);

	dev_dbg(&tcm->client->dev,
		"Application firmware v%d.%d (customer '%s') status %d\n",
		app_info.version[0], app_info.version[1],
		app_info.customer_config_id, status);

	*max_x = le16_to_cpu(app_info.max_x);
	*max_y = le16_to_cpu(app_info.max_y);

	/* Debug: dump the currently active touch report config from the fw */
	{
		u8 report_config[64];
		int cfg_ret;
		u16 max_cfg_size = get_unaligned_le16(app_info.max_touch_report_config_size);

		memset(report_config, 0, sizeof(report_config));
		cfg_ret = tcm_read_message(tcm, TCM_GET_TOUCH_REPORT_CONFIG,
					   report_config, sizeof(report_config));
		dev_dbg(&tcm->client->dev,
			"GET_TOUCH_REPORT_CONFIG ret=%d max_cfg_size=%u\n",
			 cfg_ret, max_cfg_size);
		print_hex_dump_debug("tcm report cfg: ", DUMP_PREFIX_OFFSET,
				     16, 1, report_config, sizeof(report_config), false);
	}

	/*
	 * M52XQ: attempting to force the standard reference report config
	 * via CMD_SET_TOUCH_REPORT_CONFIG (0x26) hangs the bus for the full
	 * wait timeout with zero IRQ activity, every time, regardless of
	 * wire-format fixes or longer timeouts -- abandoned. The 0xc1
	 * "custom" report turned out to be DC_PS_STATUS (power/charger
	 * telemetry), not touch -- REPORT_TOUCH (0x11) has never once been
	 * observed despite reporting being enabled. Try an explicit
	 * capacitive-baseline recalibration in case the fw needs it before
	 * it will start reporting real touches at all.
	 */
	{
		int rezero_ret = tcm_read_message(tcm, TCM_REZERO, NULL, 0);
		dev_dbg(&tcm->client->dev, "REZERO ret=%d\n", rezero_ret);
	}

	return 0;
}

static int tcm_power_on(struct tcm_data *tcm)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(tcm->supplies),
				    tcm->supplies);
	if (ret)
		return ret;

	/*
	 * M52XQ: reset IS wired -- gpio105, confirmed via our own decompiled
	 * entry36's "pmx_ts_reset_suspend" pinctrl group naming, and matched
	 * independently by sibling sm7325 devices (Motorola Dubai, Xiaomi
	 * Taoyao) which both use tlmm 105 GPIO_ACTIVE_LOW for touch reset on
	 * the same reference platform. Earlier "no reset line" conclusion
	 * was wrong. reset_gpio stays optional for portability to other
	 * boards that genuinely lack it.
	 */
	if (tcm->reset_gpio) {
		gpiod_set_value_cansleep(tcm->reset_gpio, 1);
		usleep_range(10000, 11000);
		gpiod_set_value_cansleep(tcm->reset_gpio, 0);
	}
	usleep_range(80000, 81000);

	return 0;
}

static int tcm_probe(struct i2c_client *client)
{
	struct tcm_data *tcm;
	u16 max_x, max_y;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C |
						I2C_FUNC_SMBUS_BYTE_DATA |
						I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;

	tcm = devm_kzalloc(&client->dev, sizeof(struct tcm_data), GFP_KERNEL);
	if (!tcm)
		return -ENOMEM;

	i2c_set_clientdata(client, tcm);
	tcm->client = client;

	init_completion(&tcm->response);

	tcm->supplies[0].supply = "vdd";
	tcm->supplies[1].supply = "vcc";
	ret = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(tcm->supplies),
				      tcm->supplies);
	if (ret)
		return ret;

	tcm->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(tcm->reset_gpio))
		return PTR_ERR(tcm->reset_gpio);

	ret = devm_add_action_or_reset(&client->dev, tcm_power_off,
				       tcm);
	if (ret)
		return ret;

	ret = tcm_power_on(tcm);
	if (ret)
		return ret;
	tcm->powered = true;

	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					tcm_report_irq,
					IRQF_ONESHOT,
					"synaptics_tcm_report", tcm);
	if (ret < 0)
		return ret;

	ret = tcm_hw_init(tcm, &max_x, &max_y);
	if (ret) {
		/*
		 * M52XQ: the first i2c transaction right after power-on
		 * sometimes gets NAKed (-ENXIO) even though power/pinmux
		 * are already correct -- the chip just isn't ready yet.
		 * A short retry loop reliably recovers where a single
		 * attempt doesn't.
		 */
		int attempt;

		/*
		 * A bare retry against power that's been left on the whole
		 * time never recovers (tested: 100 attempts / ~11s, 0
		 * successes) -- the chip appears to latch a bad state from
		 * the very first power-on and needs an actual voltage
		 * power-cycle (real OFF, not just a resent command) to
		 * reset. Requires the vdd/vcc regulators to NOT be
		 * regulator-always-on in the DT.
		 */
		for (attempt = 0; attempt < 20 && ret; attempt++) {
			dev_warn(&client->dev,
				 "hw_init failed (%d), power-cycling and retrying (%d/20)\n",
				 ret, attempt + 1);
			regulator_bulk_disable(ARRAY_SIZE(tcm->supplies),
					       tcm->supplies);
			msleep(50);
			ret = tcm_power_on(tcm);
			if (ret)
				continue;
			ret = tcm_hw_init(tcm, &max_x, &max_y);
		}

		if (ret) {
			dev_err(&client->dev, "failed to initialize hardware\n");
			return ret;
		}
	}

	tcm->input = devm_input_allocate_device(&client->dev);
	if (!tcm->input)
		return -ENOMEM;

	tcm->input->name = "Synaptics TCM Oncell Touchscreen";
	tcm->input->id.bustype = BUS_I2C;
	tcm->input->open = tcm_input_open;
	tcm->input->close = tcm_input_close;

	/*
	 * M52XQ: REPORT_SEC_COORDINATE_EVENT packs X/Y as 12-bit values
	 * (0-4095), independent of GET_APPLICATION_INFO's max_x/max_y
	 * (confirmed against the real Samsung driver -- sibling sm7325
	 * devices hardcode sec,max_coords = <4096 4096> in their DT rather
	 * than trusting app_info for this). Widen the registered range so
	 * input_report_abs() never clips real SEC coordinates.
	 */
	if (max_x < 4095)
		max_x = 4095;
	if (max_y < 4095)
		max_y = 4095;
	input_set_abs_params(tcm->input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(tcm->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	touchscreen_parse_properties(tcm->input, true, &tcm->props);

	/*
	 * M52XQ: no INPUT_MT_DROP_UNUSED. The 0xc1 (sec coordinate) report carries ONE
	 * finger event, so input_mt_sync_frame() with DROP_UNUSED released every other
	 * finger on each report and two-finger gestures (pinch zoom) could not work.
	 * Releases come explicitly from the RELEASE action of the finger's own event.
	 */
	ret = input_mt_init_slots(tcm->input, 10, INPUT_MT_DIRECT);
	if (ret)
		return ret;

	input_set_drvdata(tcm->input, tcm);

	ret = input_register_device(tcm->input);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id syna_driver_ids[] = {
	{
		.compatible = "syna,s3908",
	},
	{
		/*
		 * M52XQ: the downstream DTBO overlay forces this node's
		 * "compatible" to Samsung's sec_input value no matter what
		 * the base entry36 DT says -- match it here too so this
		 * driver still binds.
		 */
		.compatible = "synaptics,synaptics_ts",
	},
	{}
};
MODULE_DEVICE_TABLE(of, syna_driver_ids);

static const struct i2c_device_id syna_i2c_ids[] = {
	{ "synaptics-tcm", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, syna_i2c_ids);

static struct i2c_driver syna_i2c_driver = {
	.probe		= tcm_probe,
	.id_table	= syna_i2c_ids,
	.driver		= {
		.name		= "synaptics-tcm",
		.of_match_table	= syna_driver_ids,
	},
};

module_i2c_driver(syna_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Frieder Hannenheim <frieder.hannenheim@proton.me>");
MODULE_AUTHOR("Caleb Connolly <caleb@postmarketos.org>");
MODULE_DESCRIPTION("A driver for Synaptics TCM Oncell Touchpanels");
