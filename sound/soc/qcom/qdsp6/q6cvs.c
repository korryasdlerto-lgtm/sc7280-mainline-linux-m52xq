// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
// Copyright (c) 2020, Stephan Gerhold

#include <linux/module.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/soc/qcom/apr.h>
#include "q6cvs.h"
#include "q6voice-common.h"

/*
 * m52xq (SM7325): like downstream voice_create_mvm_cvs_session(), also open the
 * passive CVS session of the modem's VoiceMMode1 voice and un-mute its Tx
 * (voice_send_stream_mute_cmd); without it the uplink stays silent.
 */
#define VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION	0x00011140
#define VSS_IVOLUME_CMD_MUTE_V2				0x0001138B

struct vss_istream_cmd_create_passive_control_session_cmd {
	struct apr_hdr hdr;
	char name[20];
} __packed;

struct vss_ivolume_cmd_mute_v2_cmd {
	struct apr_hdr hdr;
	u16 direction;
	u16 mute_flag;
	u16 ramp_duration_ms;
} __packed;

struct q6voice_session *q6cvs_session_create(enum q6voice_path_type path)
{
	struct vss_istream_cmd_create_passive_control_session_cmd cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION;
	/* same name as the passive MVM session (q6mvm_session_name()) */
	if (path == Q6VOICE_PATH_VOICE)
		strscpy(cmd.name, "11C05000", sizeof(cmd.name));

	return q6voice_session_create(Q6VOICE_SERVICE_CVS, path, &cmd.hdr);
}
EXPORT_SYMBOL_GPL(q6cvs_session_create);

/* direction: 0 = Tx, 1 = Rx */
int q6cvs_set_mute(struct q6voice_session *cvs, u16 direction, bool mute)
{
	struct vss_ivolume_cmd_mute_v2_cmd cmd;

	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = VSS_IVOLUME_CMD_MUTE_V2;
	cmd.direction = direction;
	cmd.mute_flag = mute;
	cmd.ramp_duration_ms = 20;

	return q6voice_common_send(cvs, &cmd.hdr);
}
EXPORT_SYMBOL_GPL(q6cvs_set_mute);

#define VSS_IRECORD_CMD_START				0x000112BE
#define VSS_IRECORD_CMD_STOP				0x00011237
#define VSS_IRECORD_PORT_ID_DEFAULT			0xFFFF
#define VSS_IRECORD_TAP_POINT_NONE			0x00010F78
#define VSS_IRECORD_TAP_POINT_STREAM_END		0x00010F79

struct vss_irecord_cmd_start_cmd {
	struct apr_hdr hdr;
	u32 rx_tap_point;
	u32 tx_tap_point;
	u16 port_id;
	u32 mode;
} __packed;

/*
 * m52xq debug: tap the Tx (uplink) / Rx (downlink) stream of the voice call at the end of the stream and send it to the
 * default AFE pseudo ports (VOICE_RECORD_TX 0x8004 / VOICE_RECORD_RX 0x8003); it can then be recorded with a normal
 * capture FE (MultiMedia Mixer VOICE_RECORD_TX).  This shows what the DSP really sends towards the modem.
 */
int q6cvs_record(struct q6voice_session *cvs, bool start)
{
	if (start) {
		struct vss_irecord_cmd_start_cmd cmd;

		memset(&cmd, 0, sizeof(cmd));
		cmd.hdr.pkt_size = sizeof(cmd);
		cmd.hdr.opcode = VSS_IRECORD_CMD_START;
		cmd.rx_tap_point = VSS_IRECORD_TAP_POINT_STREAM_END;
		cmd.tx_tap_point = VSS_IRECORD_TAP_POINT_STREAM_END;
		cmd.port_id = VSS_IRECORD_PORT_ID_DEFAULT;
		return q6voice_common_send(cvs, &cmd.hdr);
	} else {
		struct apr_hdr hdr;

		memset(&hdr, 0, sizeof(hdr));
		hdr.pkt_size = sizeof(hdr);
		hdr.opcode = VSS_IRECORD_CMD_STOP;
		return q6voice_common_send(cvs, &hdr);
	}
}
EXPORT_SYMBOL_GPL(q6cvs_record);

#define VSS_IPLAYBACK_CMD_START				0x000112BD
#define VSS_IPLAYBACK_CMD_STOP				0x00011239
#define VSS_IPLAYBACK_PORT_ID_VOICE			0x8005	/* AFE_PORT_ID_VOICE_PLAYBACK_TX */

struct vss_iplayback_cmd_start_cmd {
	struct apr_hdr hdr;
	u16 port_id;
} __packed;

/*
 * m52xq: deliver audio from the AFE pseudo port VOICE_PLAYBACK_TX (0x8005) into the Tx (uplink) voice stream, mixed with
 * the microphone ("in-call music").  Used to inject the audio of a USB headset microphone into the call.
 */
int q6cvs_playback(struct q6voice_session *cvs, bool start)
{
	if (start) {
		struct vss_iplayback_cmd_start_cmd cmd;

		memset(&cmd, 0, sizeof(cmd));
		cmd.hdr.pkt_size = sizeof(cmd);
		cmd.hdr.opcode = VSS_IPLAYBACK_CMD_START;
		cmd.port_id = VSS_IPLAYBACK_PORT_ID_VOICE;
		return q6voice_common_send(cvs, &cmd.hdr);
	} else {
		struct apr_hdr hdr;

		memset(&hdr, 0, sizeof(hdr));
		hdr.pkt_size = sizeof(hdr);
		hdr.opcode = VSS_IPLAYBACK_CMD_STOP;
		return q6voice_common_send(cvs, &hdr);
	}
}
EXPORT_SYMBOL_GPL(q6cvs_playback);

static int q6cvs_probe(struct apr_device *adev)
{
	return q6voice_common_probe(adev, Q6VOICE_SERVICE_CVS);
}

static const struct of_device_id q6cvs_device_id[]  = {
	{ .compatible = "qcom,q6cvs" },
	{},
};
MODULE_DEVICE_TABLE(of, q6cvs_device_id);

static struct apr_driver qcom_q6cvs_driver = {
	.probe = q6cvs_probe,
	.remove = q6voice_common_remove,
	.callback = q6voice_common_callback,
	.driver = {
		.name = "qcom-q6cvs",
		.of_match_table = of_match_ptr(q6cvs_device_id),
	},
};

module_apr_driver(qcom_q6cvs_driver);

MODULE_AUTHOR("Stephan Gerhold <stephan@gerhold.net>");
MODULE_DESCRIPTION("Q6 Core Voice Stream");
MODULE_LICENSE("GPL v2");
