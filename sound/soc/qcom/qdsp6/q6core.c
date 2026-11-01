// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2011-2017, The Linux Foundation. All rights reserved.
// Copyright (c) 2018, Linaro Limited

#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/jiffies.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/soc/qcom/apr.h>
#include "q6core.h"
#include "q6dsp-errno.h"

#define ADSP_STATE_READY_TIMEOUT_MS    3000
#define Q6_READY_TIMEOUT_MS 100
#define AVCS_CMD_ADSP_EVENT_GET_STATE		0x0001290C
#define AVCS_CMDRSP_ADSP_EVENT_GET_STATE	0x0001290D
#define AVCS_GET_VERSIONS       0x00012905
#define AVCS_GET_VERSIONS_RSP   0x00012906
#define AVCS_CMD_GET_FWK_VERSION	0x001292c
/* m52xq: custom topologies from the vendor ACDB (after sc7280-mainline f41bf85, Richard Acayan) */
#define AVCS_CMD_REGISTER_TOPOLOGIES	0x00012923
#define AVCS_CMD_SHARED_MEM_MAP_REGIONS	0x00012924
#define AVCS_CMDRSP_SHARED_MEM_MAP_REGIONS	0x00012925
#define AVCS_MEM_POOL_SHMEM8_4K		3
#define ACDB_SECTION_GPROPLUT	0x54554c504f525047ULL	/* "GPROPLUT" */
#define ACDB_SECTION_DATAPOOL	0x4c4f4f5041544144ULL	/* "DATAPOOL" */
#define ACDB_TOPOLOGIES_BLOB	0x000131a7
#define AVCS_CMDRSP_GET_FWK_VERSION	0x001292d

struct avcs_svc_info {
	uint32_t service_id;
	uint32_t version;
} __packed;

struct avcs_cmdrsp_get_version {
	uint32_t build_id;
	uint32_t num_services;
	struct avcs_svc_info svc_api_info[];
} __packed;

/* for ADSP2.8 and above */
struct avcs_svc_api_info {
	uint32_t service_id;
	uint32_t api_version;
	uint32_t api_branch_version;
} __packed;

struct avcs_cmdrsp_get_fwk_version {
	uint32_t build_major_version;
	uint32_t build_minor_version;
	uint32_t build_branch_version;
	uint32_t build_subbranch_version;
	uint32_t num_services;
	struct avcs_svc_api_info svc_api_info[];
} __packed;

struct q6core {
	struct apr_device *adev;
	wait_queue_head_t wait;
	uint32_t avcs_state;
	struct mutex lock;
	bool resp_received;
	uint32_t num_services;
	struct avcs_cmdrsp_get_fwk_version *fwk_version;
	struct avcs_cmdrsp_get_version *svc_version;
	bool fwk_version_supported;
	bool get_state_supported;
	bool get_version_supported;
	bool is_version_requested;
	bool topo_registered;
	bool topo_resp;
	u32 topo_status;
	bool map_resp;
	u32 map_handle;
	u32 map_status;
	void *topo_buf;
	dma_addr_t topo_iova;
	size_t topo_buf_size;
};

struct avcs_cmd_shared_mem_map_regions {
	u16 mem_pool_id;
	u16 num_regions;
	u32 property_flag;
	u32 shm_addr_lsw;
	u32 shm_addr_msw;
	u32 mem_size_bytes;
} __packed;

struct avcs_cmd_register_topologies {
	uint32_t payload_addr_lsw;
	uint32_t payload_addr_msw;
	uint32_t mem_map_handle;
	uint32_t payload_size;
} __packed;

/* m52xq: 0x131a7 = old AVCS custom topology info (ADSP_EBADPARAM in-band), 0x132ac = _V3 (with instance ids) */
static unsigned int acdb_topo_prop = 0x132ac;
module_param(acdb_topo_prop, uint, 0644);
MODULE_PARM_DESC(acdb_topo_prop, "Global_cal.acdb GPROPLUT property with the AVCS custom topologies");
/* out-of-band through q6asm-dais memory hung the SoC once (watchdog bite, v198): off unless asked */
static unsigned int acdb_shmem_stage;
module_param(acdb_shmem_stage, uint, 0644);
MODULE_PARM_DESC(acdb_shmem_stage, "0 = in-band, 1 = only map the AVCS shared memory (test), 2 = map + register out-of-band");

/*
 * m52xq: downstream msm_audio_ion ORs the SMMU SID (iommus 0x1801 & qcom,smmu-sid-mask 0xf = 1) into bits 32+
 * of every address it hands to the DSP; with msw = 0 the DSP read of the topology buffer hung the SoC (v200).
 */
static unsigned int acdb_msw = 1;
module_param(acdb_msw, uint, 0644);
MODULE_PARM_DESC(acdb_msw, "value ORed into the upper 32 address bits given to the DSP (SMMU SID, 1)");

static char *acdb_name = "qcom/sm7325/samsung/m52xq/Global_cal.acdb";
module_param(acdb_name, charp, 0644);
MODULE_PARM_DESC(acdb_name, "vendor Global_cal.acdb with the custom topologies (empty = none)");

static struct q6core *g_core;

static int q6core_callback(struct apr_device *adev, const struct apr_resp_pkt *data)
{
	struct q6core *core = dev_get_drvdata(&adev->dev);
	const struct aprv2_ibasic_rsp_result_t *result;
	const struct apr_hdr *hdr = &data->hdr;

	result = data->payload;
	switch (hdr->opcode) {
	case APR_BASIC_RSP_RESULT:{
		result = data->payload;
		switch (result->opcode) {
		case AVCS_GET_VERSIONS:
			if (result->status == ADSP_EUNSUPPORTED)
				core->get_version_supported = false;
			core->resp_received = true;
			break;
		case AVCS_CMD_GET_FWK_VERSION:
			if (result->status == ADSP_EUNSUPPORTED)
				core->fwk_version_supported = false;
			core->resp_received = true;
			break;
		case AVCS_CMD_ADSP_EVENT_GET_STATE:
			if (result->status == ADSP_EUNSUPPORTED)
				core->get_state_supported = false;
			core->resp_received = true;
			break;
		case AVCS_CMD_REGISTER_TOPOLOGIES:
			core->topo_status = result->status;
			core->topo_resp = true;
			wake_up(&core->wait);
			break;
		case AVCS_CMD_SHARED_MEM_MAP_REGIONS:
			core->map_status = result->status;
			core->map_resp = true;
			wake_up(&core->wait);
			break;
		}
		break;
	}
	case AVCS_CMDRSP_GET_FWK_VERSION: {
		struct avcs_cmdrsp_get_fwk_version *fwk;

		fwk = data->payload;

		core->fwk_version = kmemdup(data->payload,
					    struct_size(fwk, svc_api_info,
							fwk->num_services),
					    GFP_ATOMIC);
		if (!core->fwk_version)
			return -ENOMEM;

		core->fwk_version_supported = true;
		core->resp_received = true;

		break;
	}
	case AVCS_GET_VERSIONS_RSP: {
		struct avcs_cmdrsp_get_version *v;

		v = data->payload;

		core->svc_version = kmemdup(data->payload,
					    struct_size(v, svc_api_info,
							v->num_services),
					    GFP_ATOMIC);
		if (!core->svc_version)
			return -ENOMEM;

		core->get_version_supported = true;
		core->resp_received = true;

		break;
	}
	case AVCS_CMDRSP_SHARED_MEM_MAP_REGIONS:
		core->map_handle = *(const u32 *)data->payload;
		core->map_status = 0;
		core->map_resp = true;
		wake_up(&core->wait);
		break;
	case AVCS_CMDRSP_ADSP_EVENT_GET_STATE:
		core->get_state_supported = true;
		core->avcs_state = result->opcode;

		core->resp_received = true;
		break;
	default:
		dev_err(&adev->dev, "Message id from adsp core svc: 0x%x\n",
			hdr->opcode);
		break;
	}

	if (core->resp_received)
		wake_up(&core->wait);

	return 0;
}

static int q6core_get_fwk_versions(struct q6core *core)
{
	struct apr_device *adev = core->adev;
	struct apr_pkt pkt;
	int rc;

	pkt.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				      APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	pkt.hdr.pkt_size = APR_HDR_SIZE;
	pkt.hdr.opcode = AVCS_CMD_GET_FWK_VERSION;

	rc = apr_send_pkt(adev, &pkt);
	if (rc < 0)
		return rc;

	rc = wait_event_timeout(core->wait, (core->resp_received),
				msecs_to_jiffies(Q6_READY_TIMEOUT_MS));
	if (rc > 0 && core->resp_received) {
		core->resp_received = false;

		if (!core->fwk_version_supported)
			return -ENOTSUPP;
		else
			return 0;
	}


	return rc;
}

static int q6core_get_svc_versions(struct q6core *core)
{
	struct apr_device *adev = core->adev;
	struct apr_pkt pkt;
	int rc;

	pkt.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				      APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	pkt.hdr.pkt_size = APR_HDR_SIZE;
	pkt.hdr.opcode = AVCS_GET_VERSIONS;

	rc = apr_send_pkt(adev, &pkt);
	if (rc < 0)
		return rc;

	rc = wait_event_timeout(core->wait, (core->resp_received),
				msecs_to_jiffies(Q6_READY_TIMEOUT_MS));
	if (rc > 0 && core->resp_received) {
		core->resp_received = false;
		return 0;
	}

	return rc;
}

static bool __q6core_is_adsp_ready(struct q6core *core)
{
	struct apr_device *adev = core->adev;
	struct apr_pkt pkt;
	int rc;

	core->get_state_supported = false;

	pkt.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				      APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	pkt.hdr.pkt_size = APR_HDR_SIZE;
	pkt.hdr.opcode = AVCS_CMD_ADSP_EVENT_GET_STATE;

	rc = apr_send_pkt(adev, &pkt);
	if (rc < 0)
		return false;

	rc = wait_event_timeout(core->wait, (core->resp_received),
				msecs_to_jiffies(Q6_READY_TIMEOUT_MS));
	if (rc > 0 && core->resp_received) {
		core->resp_received = false;

		if (core->avcs_state)
			return true;
	}

	/* assume that the adsp is up if we not support this command */
	if (!core->get_state_supported)
		return true;

	return false;
}

/**
 * q6core_get_svc_api_info() - Get version number of a service.
 *
 * @svc_id: service id of the service.
 * @ainfo: Valid struct pointer to fill svc api information.
 *
 * Return: zero on success and error code on failure or unsupported
 */
int q6core_get_svc_api_info(int svc_id, struct q6core_svc_api_info *ainfo)
{
	int i;
	int ret = -ENOTSUPP;

	if (!g_core || !ainfo)
		return 0;

	mutex_lock(&g_core->lock);
	if (!g_core->is_version_requested) {
		if (q6core_get_fwk_versions(g_core) == -ENOTSUPP)
			q6core_get_svc_versions(g_core);
		g_core->is_version_requested = true;
	}

	if (g_core->fwk_version_supported) {
		for (i = 0; i < g_core->fwk_version->num_services; i++) {
			struct avcs_svc_api_info *info;

			info = &g_core->fwk_version->svc_api_info[i];
			if (svc_id != info->service_id)
				continue;

			ainfo->api_version = info->api_version;
			ainfo->api_branch_version = info->api_branch_version;
			ret = 0;
			break;
		}
	} else if (g_core->get_version_supported) {
		for (i = 0; i < g_core->svc_version->num_services; i++) {
			struct avcs_svc_info *info;

			info = &g_core->svc_version->svc_api_info[i];
			if (svc_id != info->service_id)
				continue;

			ainfo->api_version = info->version;
			ainfo->api_branch_version = 0;
			ret = 0;
			break;
		}
	}

	mutex_unlock(&g_core->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(q6core_get_svc_api_info);

/**
 * q6core_is_adsp_ready() - Get status of adsp
 *
 * Return: Will be an true if adsp is ready and false if not.
 */
bool q6core_is_adsp_ready(void)
{
	unsigned long  timeout;
	bool ret = false;

	if (!g_core)
		return false;

	mutex_lock(&g_core->lock);
	timeout = jiffies + msecs_to_jiffies(ADSP_STATE_READY_TIMEOUT_MS);
	for (;;) {
		if (__q6core_is_adsp_ready(g_core)) {
			ret = true;
			break;
		}

		if (!time_after(timeout, jiffies)) {
			ret = false;
			break;
		}
	}

	mutex_unlock(&g_core->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(q6core_is_adsp_ready);

static int acdb_section_offset(const struct firmware *fw, u64 section, u32 *off_out)
{
	u32 off = 32, sect_size;
	u64 sect_id;

	while (off + 12 <= fw->size) {
		memcpy(&sect_id, fw->data + off, 8);
		memcpy(&sect_size, fw->data + off + 8, 4);
		if (off + 12 + sect_size > fw->size)
			return -EINVAL;
		if (sect_id == section) {
			*off_out = off;
			return 0;
		}
		off += 12 + sect_size;
	}

	return -ENOENT;
}

static int acdb_get_global_property(const struct firmware *fw, u32 prop_id, u32 *offset)
{
	u32 gproplut_off, datapool_off, lut_size, lut_nelts, i, id, poff;
	int ret;

	ret = acdb_section_offset(fw, ACDB_SECTION_GPROPLUT, &gproplut_off);
	if (ret)
		return ret;
	ret = acdb_section_offset(fw, ACDB_SECTION_DATAPOOL, &datapool_off);
	if (ret)
		return ret;

	memcpy(&lut_size, fw->data + gproplut_off + 8, 4);
	if (lut_size < 4)
		return -EINVAL;
	memcpy(&lut_nelts, fw->data + gproplut_off + 12, 4);
	if (lut_size < 4 + 8 * lut_nelts)
		return -EINVAL;

	for (i = 0; i < lut_nelts; i++) {
		memcpy(&id, fw->data + gproplut_off + 16 + i * 8, 4);
		memcpy(&poff, fw->data + gproplut_off + 20 + i * 8, 4);
		if (datapool_off + 12 + poff + 4 > fw->size)
			return -EINVAL;
		if (id == prop_id) {
			*offset = datapool_off + 12 + poff;
			return 0;
		}
	}

	return -ENOENT;
}

/**
 * q6core_register_acdb_topologies() - register the vendor custom topologies once
 *
 * m52xq: called before the first voice session (the ACDB lives on the rootfs, which is not mounted
 * yet when q6core probes).  Sends the GPROPLUT 0x131a7 blob of Global_cal.acdb in-band with
 * AVCS_CMD_REGISTER_TOPOLOGIES.
 */
int q6core_register_acdb_topologies(void)
{
	struct avcs_cmd_register_topologies *cmd;
	const struct firmware *fw;
	struct apr_pkt *pkt;
	u32 topo_off, size;
	struct device *dev;
	int ret, pkt_size;
	void *p;

	if (!g_core || !acdb_name || !acdb_name[0])
		return 0;

	dev = &g_core->adev->dev;
	mutex_lock(&g_core->lock);
	if (g_core->topo_registered) {
		mutex_unlock(&g_core->lock);
		return 0;
	}

	ret = request_firmware(&fw, acdb_name, dev);
	if (ret) {
		dev_err(dev, "m52xq: no ACDB %s: %d\n", acdb_name, ret);
		goto unlock;
	}

	ret = acdb_get_global_property(fw, acdb_topo_prop, &topo_off);
	if (ret) {
		dev_err(dev, "m52xq: ACDB parse error: %d\n", ret);
		goto release;
	}
	memcpy(&size, fw->data + topo_off, 4);
	if (topo_off + 4 + size > fw->size) {
		ret = -EINVAL;
		goto release;
	}
	{
		u32 w[4] = {};

		memcpy(w, fw->data + topo_off + 4, min_t(u32, size, sizeof(w)));
		dev_info(dev, "m52xq: ACDB prop 0x%x, %u bytes (%*ph), %s\n", acdb_topo_prop, size,
			 16, w, acdb_shmem_stage ? "shared memory" : "in-band");
	}

	if (!acdb_shmem_stage) {
		pkt_size = APR_HDR_SIZE + sizeof(*cmd) + size;
		p = kzalloc(pkt_size, GFP_KERNEL);
		if (!p) {
			ret = -ENOMEM;
			goto release;
		}
		pkt = p;
		cmd = p + APR_HDR_SIZE;
		pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		pkt->hdr.pkt_size = pkt_size;
		pkt->hdr.opcode = AVCS_CMD_REGISTER_TOPOLOGIES;
		cmd->payload_size = size;
		memcpy(p + APR_HDR_SIZE + sizeof(*cmd), fw->data + topo_off + 4, size);
		g_core->topo_resp = false;
		ret = apr_send_pkt(g_core->adev, pkt);
		kfree(p);
		if (ret < 0) {
			dev_err(dev, "m52xq: in-band AVCS_CMD_REGISTER_TOPOLOGIES send failed: %d\n", ret);
			goto release;
		}
		goto wait_resp;
	}

	/*
	 * Like downstream q6core_send_custom_topologies(): the blob goes through shared memory
	 * (in-band payload -> ADSP_EBADPARAM).  The buffer is allocated against the q6asm-dais device,
	 * whose iommus (SID 0x1801) is the ADSP's audio SMMU context, so the IOVA is valid for the DSP.
	 */
	if (!g_core->topo_buf) {
		struct device_node *np = of_find_compatible_node(NULL, NULL, "qcom,q6asm-dais");
		struct platform_device *pdev = np ? of_find_device_by_node(np) : NULL;

		of_node_put(np);
		if (!pdev) {
			dev_err(dev, "m52xq: no q6asm-dais device for the ACDB buffer\n");
			ret = -ENODEV;
			goto release;
		}
		g_core->topo_buf_size = ALIGN(size, 4096);
		g_core->topo_buf = dma_alloc_coherent(&pdev->dev, g_core->topo_buf_size,
						      &g_core->topo_iova, GFP_KERNEL);
		if (!g_core->topo_buf) {
			ret = -ENOMEM;
			goto release;
		}
	}
	dev_info(dev, "m52xq: ACDB buffer iova %pad size %zu, mapping (stage %u)\n",
		 &g_core->topo_iova, g_core->topo_buf_size, acdb_shmem_stage);
	if (size > g_core->topo_buf_size) {
		ret = -E2BIG;
		goto release;
	}
	memset(g_core->topo_buf, 0, g_core->topo_buf_size);
	memcpy(g_core->topo_buf, fw->data + topo_off + 4, size);

	if (!g_core->map_handle) {
		struct avcs_cmd_shared_mem_map_regions *map;

		pkt_size = APR_HDR_SIZE + sizeof(*map);
		p = kzalloc(pkt_size, GFP_KERNEL);
		if (!p) {
			ret = -ENOMEM;
			goto release;
		}
		pkt = p;
		map = p + APR_HDR_SIZE;
		pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		pkt->hdr.pkt_size = pkt_size;
		pkt->hdr.opcode = AVCS_CMD_SHARED_MEM_MAP_REGIONS;
		map->mem_pool_id = AVCS_MEM_POOL_SHMEM8_4K;
		map->num_regions = 1;
		map->shm_addr_lsw = lower_32_bits(g_core->topo_iova);
		map->shm_addr_msw = upper_32_bits(g_core->topo_iova) | acdb_msw;
		map->mem_size_bytes = g_core->topo_buf_size;
		g_core->map_resp = false;
		g_core->map_handle = 0;
		ret = apr_send_pkt(g_core->adev, pkt);
		kfree(p);
		if (ret < 0)
			goto release;
		if (!wait_event_timeout(g_core->wait, g_core->map_resp, msecs_to_jiffies(1000))) {
			dev_err(dev, "m52xq: AVCS shared mem map timeout\n");
			ret = -ETIMEDOUT;
			goto release;
		}
		if (!g_core->map_handle) {
			dev_err(dev, "m52xq: AVCS shared mem map failed, status 0x%x\n", g_core->map_status);
			ret = -EIO;
			goto release;
		}
		dev_info(dev, "m52xq: ACDB buffer iova %pad (msw 0x%x) mapped, handle 0x%x\n",
			 &g_core->topo_iova, upper_32_bits(g_core->topo_iova) | acdb_msw, g_core->map_handle);
		if (acdb_shmem_stage == 1) {
			dev_info(dev, "m52xq: stage 1 done (map only, no registration)\n");
			ret = 0;
			goto release;
		}
	}

	pkt_size = APR_HDR_SIZE + sizeof(*cmd);
	p = kzalloc(pkt_size, GFP_KERNEL);
	if (!p) {
		ret = -ENOMEM;
		goto release;
	}
	pkt = p;
	cmd = p + APR_HDR_SIZE;
	pkt->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	pkt->hdr.pkt_size = pkt_size;
	pkt->hdr.opcode = AVCS_CMD_REGISTER_TOPOLOGIES;
	cmd->payload_addr_lsw = lower_32_bits(g_core->topo_iova);
	cmd->payload_addr_msw = upper_32_bits(g_core->topo_iova) | acdb_msw;
	cmd->mem_map_handle = g_core->map_handle;
	cmd->payload_size = size;

	dev_info(dev, "m52xq: AVCS_CMD_REGISTER_TOPOLOGIES addr 0x%x_%08x handle 0x%x size %u\n",
		 cmd->payload_addr_msw, cmd->payload_addr_lsw, cmd->mem_map_handle, size);
	g_core->topo_resp = false;
	ret = apr_send_pkt(g_core->adev, pkt);
	kfree(p);
	if (ret < 0) {
		dev_err(dev, "m52xq: AVCS_CMD_REGISTER_TOPOLOGIES send failed: %d\n", ret);
		goto release;
	}
wait_resp:
	if (!wait_event_timeout(g_core->wait, g_core->topo_resp, msecs_to_jiffies(1000))) {
		dev_err(dev, "m52xq: AVCS_CMD_REGISTER_TOPOLOGIES timeout\n");
		ret = -ETIMEDOUT;
		goto release;
	}
	dev_info(dev, "m52xq: registered %u bytes of ACDB topologies from %s, status 0x%x\n",
		 size, acdb_name, g_core->topo_status);
	ret = g_core->topo_status ? -EIO : 0;
	if (!ret)
		g_core->topo_registered = true;
release:
	release_firmware(fw);
unlock:
	mutex_unlock(&g_core->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(q6core_register_acdb_topologies);

/* m52xq test hook: echo 1 > /sys/module/q6core/parameters/acdb_register (re)registers the topologies */
static int acdb_register_set(const char *val, const struct kernel_param *kp)
{
	if (g_core)
		g_core->topo_registered = false;
	return q6core_register_acdb_topologies();
}
static const struct kernel_param_ops acdb_register_ops = { .set = acdb_register_set };
module_param_cb(acdb_register, &acdb_register_ops, NULL, 0200);

static int q6core_probe(struct apr_device *adev)
{
	g_core = kzalloc_obj(*g_core);
	if (!g_core)
		return -ENOMEM;

	dev_set_drvdata(&adev->dev, g_core);

	mutex_init(&g_core->lock);
	g_core->adev = adev;
	init_waitqueue_head(&g_core->wait);
	return 0;
}

static void q6core_exit(struct apr_device *adev)
{
	struct q6core *core = dev_get_drvdata(&adev->dev);

	if (core->fwk_version_supported)
		kfree(core->fwk_version);
	if (core->get_version_supported)
		kfree(core->svc_version);

	g_core = NULL;
	kfree(core);
}

#ifdef CONFIG_OF
static const struct of_device_id q6core_device_id[]  = {
	{ .compatible = "qcom,q6core" },
	{},
};
MODULE_DEVICE_TABLE(of, q6core_device_id);
#endif

static struct apr_driver qcom_q6core_driver = {
	.probe = q6core_probe,
	.remove = q6core_exit,
	.callback = q6core_callback,
	.driver = {
		.name = "qcom-q6core",
		.of_match_table = of_match_ptr(q6core_device_id),
	},
};

module_apr_driver(qcom_q6core_driver);
MODULE_DESCRIPTION("q6 core");
MODULE_LICENSE("GPL v2");
