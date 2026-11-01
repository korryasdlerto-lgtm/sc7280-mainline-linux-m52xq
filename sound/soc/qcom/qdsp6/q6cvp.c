// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
// Copyright (c) 2020, Stephan Gerhold

#include <linux/firmware.h>
#include <linux/unaligned.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/soc/qcom/apr.h>
#include "q6cvp.h"
#include "q6core.h"
#include "q6voice-common.h"

#define VSS_IVOCPROC_DIRECTION_RX	0
#define VSS_IVOCPROC_DIRECTION_TX	1
#define VSS_IVOCPROC_DIRECTION_RX_TX	2

#define VSS_IVOCPROC_PORT_ID_NONE	0xFFFF

#define VSS_IVOCPROC_TOPOLOGY_ID_NONE			0x00010F70
#define VSS_IVOCPROC_TOPOLOGY_ID_TX_SM_ECNS		0x00010F71
#define VSS_IVOCPROC_TOPOLOGY_ID_TX_DM_FLUENCE		0x00010F72

#define VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT		0x00010F77

#define VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING		0x00010F7C
#define VSS_IVOCPROC_VOCPROC_MODE_EC_EXT_MIXING		0x00010F7D

#define VSS_ICOMMON_CAL_NETWORK_ID_NONE			0x0001135E

#define VSS_IVOCPROC_CMD_ENABLE				0x000100C6
#define VSS_IVOCPROC_CMD_DISABLE			0x000110E1

#define VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V2	0x000112BF
#define VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V3	0x00013169

struct vss_ivocproc_cmd_create_full_control_session_v2_cmd {
	struct apr_hdr hdr;

	/*
	 * Vocproc direction. The supported values:
	 * VSS_IVOCPROC_DIRECTION_RX
	 * VSS_IVOCPROC_DIRECTION_TX
	 * VSS_IVOCPROC_DIRECTION_RX_TX
	 */
	u16 direction;

	/*
	 * Tx device port ID to which the vocproc connects. If a port ID is
	 * not being supplied, set this to #VSS_IVOCPROC_PORT_ID_NONE.
	 */
	u16 tx_port_id;

	/*
	 * Tx path topology ID. If a topology ID is not being supplied, set
	 * this to #VSS_IVOCPROC_TOPOLOGY_ID_NONE.
	 */
	u32 tx_topology_id;

	/*
	 * Rx device port ID to which the vocproc connects. If a port ID is
	 * not being supplied, set this to #VSS_IVOCPROC_PORT_ID_NONE.
	 */
	u16 rx_port_id;

	/*
	 * Rx path topology ID. If a topology ID is not being supplied, set
	 * this to #VSS_IVOCPROC_TOPOLOGY_ID_NONE.
	 */
	u32 rx_topology_id;

	/* Voice calibration profile ID. */
	u32 profile_id;

	/*
	 * Vocproc mode. The supported values:
	 * VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING
	 * VSS_IVOCPROC_VOCPROC_MODE_EC_EXT_MIXING
	 */
	u32 vocproc_mode;

	/*
	 * Port ID to which the vocproc connects for receiving echo
	 * cancellation reference signal. If a port ID is not being supplied,
	 * set this to #VSS_IVOCPROC_PORT_ID_NONE. This parameter value is
	 * ignored when the vocproc_mode parameter is set to
	 * VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING.
	 */
	u16 ec_ref_port_id;

	/*
	 * Session name string used to identify a session that can be shared
	 * with passive controllers (optional).
	 */
	char name[20];
} __packed;

/* m52xq voice debugging knobs, read when a voice path starts (change them, then cycle the voice session) */
static unsigned int tx_topology = VSS_IVOCPROC_TOPOLOGY_ID_NONE;
module_param(tx_topology, uint, 0644);
MODULE_PARM_DESC(tx_topology, "vocproc Tx topology id (0x10F70 NONE, 0x10F71 SM_ECNS, 0x10F72 DM_FLUENCE)");
static unsigned int rx_topology = VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT;
module_param(rx_topology, uint, 0644);
MODULE_PARM_DESC(rx_topology, "vocproc Rx topology id (0x10F77 RX_DEFAULT; m52xq ACDB handset 0x10001000)");
static unsigned int vocproc_mode = VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING;
module_param(vocproc_mode, uint, 0644);
MODULE_PARM_DESC(vocproc_mode, "vocproc mode (0x10F7C EC int mixing, 0x10F7D EC ext mixing)");
/*
 * m52xq (SM7325): the vocproc Tx media format must be MONO even though the LPASS TX codec DMA backend runs 2 channels:
 * with 2 channels the stream (VOICE_RECORD_TX tap) stays all zeros, with 1 channel the microphone voice comes through
 * (measured with the uplink tap during a live call, 2026-10-04).
 */
static unsigned int tx_channels = 1;
module_param(tx_channels, uint, 0644);
MODULE_PARM_DESC(tx_channels, "number of Tx device channels in the vocproc media format (0 = same as the backend, 2)");
static unsigned int tx_rate;
module_param(tx_rate, uint, 0644);
MODULE_PARM_DESC(tx_rate, "override the Tx device sample rate in the vocproc media format (0 = driver default)");
static unsigned int rx_channels;
module_param(rx_channels, uint, 0644);
MODULE_PARM_DESC(rx_channels, "override the number of Rx device channels (0 = automatic: 1 for QUINARY MI2S, else 2)");
static unsigned int rx_bits;
module_param(rx_bits, uint, 0644);
MODULE_PARM_DESC(rx_bits, "override the Rx device bits per sample (0 = automatic: 24 for QUINARY MI2S, else 16)");

static bool dev_unmute = true;
module_param(dev_unmute, bool, 0644);
MODULE_PARM_DESC(dev_unmute, "send explicit vocproc Tx+Rx un-mute after the voice start");

bool q6cvp_dev_unmute_enabled(void)
{
	return dev_unmute;
}
EXPORT_SYMBOL_GPL(q6cvp_dev_unmute_enabled);

struct q6voice_session *q6cvp_session_create(enum q6voice_path_type path,
					     u16 tx_port, u16 rx_port)
{
	struct vss_ivocproc_cmd_create_full_control_session_v2_cmd cmd;

	/* m52xq: custom vendor topologies (e.g. 0x1000110a) must be known to the DSP first */
	q6core_register_acdb_topologies();

	cmd.hdr.pkt_size = sizeof(cmd);
	/*
	 * m52xq (SM7325, CVD >= 2.2): downstream uses V3 here (same payload as
	 * V2); with V2 the later VSS_IVOCPROC_CMD_TOPOLOGY_COMMIT fails (status 1).
	 */
	cmd.hdr.opcode = VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V3;

	/* TODO: Implement calibration */
	/*
	 * m52xq: without ACDB calibration downstream voice_get_topology()
	 * falls back to TOPOLOGY_ID_NONE for Tx; SM_ECNS without its
	 * calibration passes no audio to the modem (uplink silent).
	 */
	cmd.tx_topology_id = tx_topology;
	cmd.rx_topology_id = rx_topology;

	cmd.direction = VSS_IVOCPROC_DIRECTION_RX_TX;
	cmd.tx_port_id = tx_port;
	cmd.rx_port_id = rx_port;
	cmd.profile_id = VSS_ICOMMON_CAL_NETWORK_ID_NONE;
	cmd.vocproc_mode = vocproc_mode;
	cmd.ec_ref_port_id = VSS_IVOCPROC_PORT_ID_NONE;

	return q6voice_session_create(Q6VOICE_SERVICE_CVP, path, &cmd.hdr);
}
EXPORT_SYMBOL_GPL(q6cvp_session_create);

/*
 * m52xq (SM7325, CVD 2.x): newer ADSP voice firmware needs the vocproc device
 * media format and a topology commit after the session is created, and a
 * channel mixer + MFC when the Rx device port is stereo.  Same sequence as
 * downstream techpack/audio/dsp/q6voice.c voice_setup_vocproc() (no calibration).
 */
#define VSS_ICOMMON_CMD_SET_PARAM_V2			0x0001133D
#define VSS_ICOMMON_CMD_SET_PARAM_V3			0x00013245
#define VSS_IVOCPROC_CMD_TOPOLOGY_COMMIT		0x00013198
#define VSS_MODULE_CVD_GENERIC				0x0001316E
#define VSS_PARAM_VOCPROC_TX_CHANNEL_INFO		0x0001328E
#define VSS_PARAM_VOCPROC_RX_CHANNEL_INFO		0x0001328F
#define VSS_PARAM_VOCPROC_EC_REF_CHANNEL_INFO		0x00013290
#define VSS_PARAM_TX_PORT_ENDPOINT_MEDIA_INFO		0x00013253
#define VSS_PARAM_RX_PORT_ENDPOINT_MEDIA_INFO		0x00013254
#define AUDPROC_MODULE_ID_MFC				0x00010912
#define AUDPROC_PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT	0x00010913
#define AUDPROC_CHMIXER_PARAM_ID_COEFF			0x00010342
#define VSS_NUM_CHANNELS_MAX				32
#define PCM_CHANNEL_FL					1
#define PCM_CHANNEL_FR					2

struct vss_icommon_set_param_v2_hdr {
	struct apr_hdr hdr;
	u32 mem_handle;
	u64 mem_address;
	u32 mem_size;
	u32 module_id;
	u32 param_id;
	u16 param_size;
	u16 reserved;
} __packed;

struct vss_param_vocproc_dev_channel_info {
	u32 num_channels;
	u32 bits_per_sample;
	u8 channel_mapping[VSS_NUM_CHANNELS_MAX];
} __packed;

struct vss_param_endpoint_media_format_info {
	u32 port_id;
	u16 num_channels;
	u16 bits_per_sample;
	u32 sample_rate;
	u8 channel_mapping[VSS_NUM_CHANNELS_MAX];
} __packed;

struct vss_param_channel_mixer_info {
	u32 index;
	u16 num_output_channels;
	u16 num_input_channels;
	u16 out_channel_map[2];
	u16 in_channel_map[1];
	u16 channel_weight_coeff[2][1];
	u16 reserved;
} __packed;

struct vss_param_mfc_config_info {
	u32 sample_rate;
	u16 bits_per_sample;
	u16 num_channels;
	u16 channel_type[VSS_NUM_CHANNELS_MAX];
} __packed;

/*
 * m52xq: static vocproc calibration of the stock Samsung Handset_cal.acdb (VSTI tables) for the
 * handset device pair (tx 34 CALL_HANDSET_MIC, rx 7 HANDSET), prepared on the PC as a flat list of
 * param_hdr_v3 + data records (stock-audio/acdb-parse/m52xq_voice_cal.py). Sent parameter by
 * parameter with in-band VSS_ICOMMON_CMD_SET_PARAM_V3 after the topology commit; a parameter the
 * ADSP rejects is logged and skipped, it never stops the call.
 */
static bool cal_enable;
module_param(cal_enable, bool, 0644);
MODULE_PARM_DESC(cal_enable, "load the Samsung voice calibration after the topology commit");
static char *cal_dir = "qcom/sm7325/samsung/m52xq";
module_param(cal_dir, charp, 0644);
MODULE_PARM_DESC(cal_dir, "firmware directory of m52xq_voice_cal_<dev>_<rate>k.bin / m52xq_voice_vol_<dev>_<rate>k_<level>.bin");
static char *cal_device = "";
module_param(cal_device, charp, 0644);
MODULE_PARM_DESC(cal_device, "calibration device (handset, speaker, headset, bt); empty = by Rx port");
static unsigned int vol_index = 4;
module_param(vol_index, uint, 0644);
MODULE_PARM_DESC(vol_index, "volume level 0..7 of the volume-dependent (VDYI) calibration");

struct vss_icommon_set_param_v3_hdr {
	struct apr_hdr hdr;
	u32 mem_handle;
	u32 mem_address_lsw;
	u32 mem_address_msw;
	u32 payload_size;
} __packed;

#define M52XQ_CAL_REC_HDR	16	/* u32 module, u16 instance, u16 reserved, u32 param, u32 size */

/* m52xq: send one calibration file (param_hdr_v3 + data records), parameter by parameter */
static int q6cvp_send_cal_file(struct q6voice_session *cvp, const char *name)
{
	const struct firmware *fw;
	struct vss_icommon_set_param_v3_hdr *cmd;
	size_t off = 0;
	int ok = 0, bad = 0;

	if (request_firmware(&fw, name, cvp->dev)) {
		dev_info(cvp->dev, "m52xq: voice calibration %s not found\n", name);
		return -ENOENT;
	}
	while (off + M52XQ_CAL_REC_HDR <= fw->size) {
		const u8 *rec = fw->data + off;
		u32 module_id = get_unaligned_le32(rec);
		u16 instance_id = get_unaligned_le16(rec + 4);
		u32 param_id = get_unaligned_le32(rec + 8);
		u32 size = get_unaligned_le32(rec + 12);
		size_t len = M52XQ_CAL_REC_HDR + ALIGN(size, 4);
		int ret;

		if (off + M52XQ_CAL_REC_HDR + size > fw->size || size > 4096) {
			dev_err(cvp->dev, "m52xq: voice calibration %s corrupt at %zu\n", name, off);
			break;
		}
		cmd = kzalloc(sizeof(*cmd) + len, GFP_KERNEL);
		if (!cmd)
			break;
		cmd->hdr.pkt_size = sizeof(*cmd) + len;
		cmd->hdr.opcode = VSS_ICOMMON_CMD_SET_PARAM_V3;
		cmd->payload_size = len;
		memcpy(cmd + 1, rec, M52XQ_CAL_REC_HDR + size);
		put_unaligned_le32(ALIGN(size, 4), (u8 *)(cmd + 1) + 12);
		ret = q6voice_common_send(cvp, &cmd->hdr);
		kfree(cmd);
		if (ret) {
			if (bad++ < 8)
				dev_warn(cvp->dev, "m52xq: cal %#x/%#x/%#x (%u bytes): %d\n",
					 module_id, instance_id, param_id, size, ret);
		} else {
			ok++;
		}
		off += len;
	}
	dev_info(cvp->dev, "m52xq: voice calibration %s: %d params set, %d rejected\n", name, ok, bad);
	release_firmware(fw);
	return 0;
}

/*
 * m52xq: stock Samsung voice calibration (Handset/Speaker/Headset/Bluetooth_cal.acdb of the phone),
 * prepared on the PC (stock-audio/acdb-parse/m52xq_voice_cal2.py): static VSTI parameters plus the
 * volume-dependent VDYI ones of one volume level. Sent after the topology commit; a parameter the
 * ADSP rejects is logged and skipped, it never stops the call.
 */
static void q6cvp_load_cal(struct q6voice_session *cvp, u16 rx_port, unsigned int rate)
{
	const char *dev = (cal_device && cal_device[0]) ? cal_device :
			  rx_port == 0x1016 ? "speaker" : "handset";	/* QUINARY MI2S Rx = loud speaker */
	char name[160];

	if (!cal_enable)
		return;
	if (!rate || rate % 1000) {
		dev_info(cvp->dev, "m52xq: no voice calibration for %u Hz\n", rate);
		return;
	}
	snprintf(name, sizeof(name), "%s/m52xq_voice_cal_%s_%uk.bin", cal_dir, dev, rate / 1000);
	if (q6cvp_send_cal_file(cvp, name))
		return;
	snprintf(name, sizeof(name), "%s/m52xq_voice_vol_%s_%uk_%u.bin", cal_dir, dev, rate / 1000,
		 min(vol_index, 7u));
	q6cvp_send_cal_file(cvp, name);
}

static int q6cvp_set_param(struct q6voice_session *cvp, u32 module_id,
			   u32 param_id, const void *data, u16 size)
{
	struct vss_icommon_set_param_v2_hdr *cmd;
	int ret;

	cmd = kzalloc(sizeof(*cmd) + size, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd->hdr.pkt_size = sizeof(*cmd) + size;
	cmd->hdr.opcode = VSS_ICOMMON_CMD_SET_PARAM_V2;
	cmd->mem_size = sizeof(*cmd) - offsetof(struct vss_icommon_set_param_v2_hdr, module_id) + size;
	cmd->module_id = module_id;
	cmd->param_id = param_id;
	cmd->param_size = size;
	memcpy(cmd + 1, data, size);

	ret = q6voice_common_send(cvp, &cmd->hdr);
	if (ret)
		pr_err("q6cvp: set param %#x/%#x failed: %d\n", module_id, param_id, ret);
	kfree(cmd);
	return ret;
}

static void q6cvp_fill_map(u8 *map, unsigned int channels)
{
	map[0] = PCM_CHANNEL_FL;
	if (channels > 1)
		map[1] = PCM_CHANNEL_FR;
}

int q6cvp_set_media(struct q6voice_session *cvp, u16 rx_port, unsigned int rx_ch,
		    u16 tx_port, unsigned int tx_ch, unsigned int rate)
{
	unsigned int rx_bps = 16;
	struct vss_param_vocproc_dev_channel_info ci;
	struct vss_param_endpoint_media_format_info mf;
	struct apr_pkt commit;
	int ret;

	if (tx_channels)
		tx_ch = tx_channels;
	/*
	 * m52xq: the loud speaker (TFA9878 on QUINARY MI2S) backend is mono, 24 bit in a 32 bit slot
	 * (sm8250_be_hw_params_fixup); the earpiece (RX codec DMA) is stereo 16 bit.
	 */
	if (rx_port == 0x1016) {		/* AFE_PORT_ID_QUINARY_MI2S_RX */
		rx_ch = 1;
		rx_bps = 24;
	}
	if (rx_channels)
		rx_ch = rx_channels;
	if (rx_bits)
		rx_bps = rx_bits;
	memset(&ci, 0, sizeof(ci));
	ci.num_channels = rx_ch;
	ci.bits_per_sample = rx_bps;
	q6cvp_fill_map(ci.channel_mapping, rx_ch);
	ret = q6cvp_set_param(cvp, VSS_MODULE_CVD_GENERIC,
			      VSS_PARAM_VOCPROC_RX_CHANNEL_INFO, &ci, sizeof(ci));
	if (ret)
		return ret;
	ret = q6cvp_set_param(cvp, VSS_MODULE_CVD_GENERIC,
			      VSS_PARAM_VOCPROC_EC_REF_CHANNEL_INFO, &ci, sizeof(ci));
	if (ret)
		return ret;

	memset(&ci, 0, sizeof(ci));
	ci.num_channels = tx_ch;
	ci.bits_per_sample = 16;
	q6cvp_fill_map(ci.channel_mapping, tx_ch);
	ret = q6cvp_set_param(cvp, VSS_MODULE_CVD_GENERIC,
			      VSS_PARAM_VOCPROC_TX_CHANNEL_INFO, &ci, sizeof(ci));
	if (ret)
		return ret;

	memset(&mf, 0, sizeof(mf));
	mf.port_id = rx_port;
	mf.num_channels = rx_ch;
	mf.bits_per_sample = rx_bps;
	mf.sample_rate = rate;
	q6cvp_fill_map(mf.channel_mapping, rx_ch);
	ret = q6cvp_set_param(cvp, VSS_MODULE_CVD_GENERIC,
			      VSS_PARAM_RX_PORT_ENDPOINT_MEDIA_INFO, &mf, sizeof(mf));
	if (ret)
		return ret;

	memset(&mf, 0, sizeof(mf));
	mf.port_id = tx_port;
	mf.num_channels = tx_ch;
	mf.bits_per_sample = 16;
	mf.sample_rate = tx_rate ? tx_rate : rate;
	q6cvp_fill_map(mf.channel_mapping, tx_ch);
	ret = q6cvp_set_param(cvp, VSS_MODULE_CVD_GENERIC,
			      VSS_PARAM_TX_PORT_ENDPOINT_MEDIA_INFO, &mf, sizeof(mf));
	if (ret)
		return ret;

	commit.hdr.pkt_size = APR_HDR_SIZE;
	commit.hdr.opcode = VSS_IVOCPROC_CMD_TOPOLOGY_COMMIT;
	ret = q6voice_common_send(cvp, &commit.hdr);
	if (ret) {
		pr_err("q6cvp: topology commit failed: %d\n", ret);
		return ret;
	}

	q6cvp_load_cal(cvp, rx_port, tx_rate ? tx_rate : rate);

	if (rx_ch > 1) {
		/* vocproc Rx output is mono: duplicate it to both channels */
		struct vss_param_channel_mixer_info cm = {
			.index = 0,
			.num_output_channels = rx_ch,
			.num_input_channels = 1,
			.out_channel_map = { PCM_CHANNEL_FL, PCM_CHANNEL_FR },
			.in_channel_map = { PCM_CHANNEL_FL },
			.channel_weight_coeff = { { 1 << 14 }, { 1 << 14 } },
		};
		struct vss_param_mfc_config_info mfc;

		ret = q6cvp_set_param(cvp, AUDPROC_MODULE_ID_MFC,
				      AUDPROC_CHMIXER_PARAM_ID_COEFF, &cm, sizeof(cm));
		if (ret)
			return ret;

		memset(&mfc, 0, sizeof(mfc));
		mfc.sample_rate = rate;
		mfc.bits_per_sample = 16;
		mfc.num_channels = rx_ch;
		mfc.channel_type[0] = PCM_CHANNEL_FL;
		mfc.channel_type[1] = PCM_CHANNEL_FR;
		ret = q6cvp_set_param(cvp, AUDPROC_MODULE_ID_MFC,
				      AUDPROC_PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT, &mfc, sizeof(mfc));
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(q6cvp_set_media);

int q6cvp_enable(struct q6voice_session *cvp, bool state)
{
	struct apr_pkt cmd;

	cmd.hdr.pkt_size = APR_HDR_SIZE;
	cmd.hdr.opcode = state ? VSS_IVOCPROC_CMD_ENABLE : VSS_IVOCPROC_CMD_DISABLE;

	return q6voice_common_send(cvp, &cmd.hdr);
}
EXPORT_SYMBOL_GPL(q6cvp_enable);

static int q6cvp_probe(struct apr_device *adev)
{
	return q6voice_common_probe(adev, Q6VOICE_SERVICE_CVP);
}

static const struct of_device_id q6cvp_device_id[]  = {
	{ .compatible = "qcom,q6cvp" },
	{},
};
MODULE_DEVICE_TABLE(of, q6cvp_device_id);

static struct apr_driver qcom_q6cvp_driver = {
	.probe = q6cvp_probe,
	.remove = q6voice_common_remove,
	.callback = q6voice_common_callback,
	.driver = {
		.name = "qcom-q6cvp",
		.of_match_table = of_match_ptr(q6cvp_device_id),
	},
};

module_apr_driver(qcom_q6cvp_driver);

MODULE_AUTHOR("Stephan Gerhold <stephan@gerhold.net>");
MODULE_DESCRIPTION("Q6 Core Voice Processor");
MODULE_LICENSE("GPL v2");

#define VSS_IVOLUME_CMD_MUTE_V2_CVP			0x0001138B

struct vss_ivolume_cvp_mute_cmd {
	struct apr_hdr hdr;
	u16 direction;
	u16 mute_flag;
	u16 ramp_duration_ms;
} __packed;

/* m52xq debug: vocproc (device) mute, direction: 0 = Tx, 1 = Rx */
int q6cvp_set_mute(struct q6voice_session *cvp, u16 direction, bool mute)
{
	struct vss_ivolume_cvp_mute_cmd cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = VSS_IVOLUME_CMD_MUTE_V2_CVP;
	cmd.direction = direction;
	cmd.mute_flag = mute;
	cmd.ramp_duration_ms = 20;

	return q6voice_common_send(cvp, &cmd.hdr);
}
EXPORT_SYMBOL_GPL(q6cvp_set_mute);
