// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/hex.h>
#include <linux/delay.h>

#include "mac.h"

#include <net/mac80211.h>
#include "core.h"
#include "hif.h"
#include "debug.h"
#include "wmi.h"
#include "wow.h"
#include "dp_rx.h"

/* M52XQ v227: run-time knobs (/sys/module/ath11k/parameters), defaults = the stock Samsung WCN6750 setup */
static int m52xq_hw_filter = WMI_HW_DATA_FILTER_DROP_NON_ARP_BC; /* gHwFilterMode=1; 0 = no filter, 3 = upstream */
module_param(m52xq_hw_filter, int, 0644);
MODULE_PARM_DESC(m52xq_hw_filter, "M52XQ: hw data filter bitmap during WoW (1 stock, 3 upstream, 0 off)");
static unsigned int m52xq_wow_keepalive = 15; /* gStaKeepAlivePeriod=15 */
module_param(m52xq_wow_keepalive, uint, 0644);
MODULE_PARM_DESC(m52xq_wow_keepalive, "M52XQ: keepalive interval in s during WoW");
static unsigned int m52xq_wow_inactivity = 50; /* g_wow_data_inactivity_timeout default; 0 = not sent */
module_param(m52xq_wow_inactivity, uint, 0644);
MODULE_PARM_DESC(m52xq_wow_inactivity, "M52XQ: STA PS inactivity time in ms during WoW (0 = unchanged)");

/*
 * M52XQ v228: the stock driver (qcacld, LineageOS log) is woken by the firmware with WOW reason BPF_ALLOW: Android loads
 * an APF program and the firmware wakes the host for every packet the program passes; without a program the WCN6750
 * firmware drops the packets during WoW (patterns and RA match never woke us). 0 = no program, 1 = pass everything,
 * 2 = pass frames to our own (unicast) address only: LDB R0,[0]; JSET R0 & 1 -> DROP; PASS.
 * 3 (v243, default) = unicast IPv4 TCP and ICMP only. Mode 2 passed every unicast frame, so the IPv6 neighbour probes of the
 * router, the replies to NTP / DHCP / DNS / mDNS requests the phone sent just before sleeping, ARP etc. woke the system
 * again 1-2 s after it fell asleep (a wake-up storm: 109 wake-ups in 42 min, 5 % asleep, ~360 mA, 30.10.2026).
 */
static int m52xq_wow_apf = 2;
module_param(m52xq_wow_apf, int, 0644);
MODULE_PARM_DESC(m52xq_wow_apf, "M52XQ: APF program during WoW (0 off, 1 pass all, 2 pass unicast, 3 unicast IPv4 TCP/ICMP (Ethernet frame), 9 bytes from m52xq_wow_apf_hex; v246: 4 pass unicast and 5 bytes from m52xq_wow_apf_hex through the stock path (APF enable + work memory))");

static int m52xq_wow_apf_ver = 4;
module_param(m52xq_wow_apf_ver, int, 0644);
MODULE_PARM_DESC(m52xq_wow_apf_ver, "M52XQ: bpf_version for the APF work memory write (modes 4 and 5)");

/*
 * v244: mode 3 above made the phone deaf to the network during sleep (ping and ssh never woke it), so the frame the firmware
 * hands to the APF program is probably not the Ethernet frame assumed (mode 2 only looked at byte 0). Mode 9 loads the program
 * from this string (hex, spaces and ':' ignored), set at run time: echo 9 > m52xq_wow_apf; echo "08 9a 01 01" > m52xq_wow_apf_hex
 * (takes effect at the next suspend), so the frame layout can be probed without rebuilding the kernel.
 */
static char *m52xq_wow_apf_hex = "";
module_param(m52xq_wow_apf_hex, charp, 0644);
MODULE_PARM_DESC(m52xq_wow_apf_hex, "M52XQ: APF program bytes in hex for m52xq_wow_apf=9");

static int m52xq_wow_apf_parse(const char *str, u8 *out, int max)
{
	int n = 0, nib = -1, c;

	if (!str)
		return 0;
	for (; *str; str++) {
		c = hex_to_bin(*str);
		if (c < 0)
			continue;	/* spaces, ':' ... */
		if (nib < 0) {
			nib = c;
		} else {
			if (n >= max)
				return -E2BIG;
			out[n++] = (nib << 4) | c;
			nib = -1;
		}
	}
	return nib < 0 ? n : -EINVAL;
}

static void ath11k_wow_apf(struct ath11k *ar, bool enable)
{
	static const u8 pass_all[] = { 0x68 };			/* LI R0, 0 -> end of program = PASS */
	static const u8 pass_ucast[] = { 0x08, 0x9a, 0x01, 0x01 };	/* LDB R0,[0]; JSET R0,1 -> +1 (DROP) */
	/*
	 * Ethernet frame; the jump target one past the end of the program (24 + 1) is DROP, the end (24) is PASS:
	 *  0: LDB R0,[0]            08
	 *  1: JSET R0,#1 -> DROP    9a 15 01   (not unicast)
	 *  4: LDB R0,[12]           0a 0c      EtherType high byte
	 *  6: JNE R0,#0x08 -> DROP  82 10 08
	 *  9: LDB R0,[13]           0a 0d      EtherType low byte
	 * 11: JNE R0,#0x00 -> DROP  82 0b 00   (not IPv4)
	 * 14: LDB R0,[23]           0a 17      IPv4 protocol
	 * 16: JEQ R0,#6 -> PASS     7a 05 06   (TCP)
	 * 19: JEQ R0,#1 -> PASS     7a 02 01   (ICMP)
	 * 22: JMP -> DROP           72 01
	 */
	static const u8 pass_ip_tcp_icmp[] = {
		0x08, 0x9a, 0x15, 0x01,
		0x0a, 0x0c, 0x82, 0x10, 0x08,
		0x0a, 0x0d, 0x82, 0x0b, 0x00,
		0x0a, 0x17, 0x7a, 0x05, 0x06,
		0x7a, 0x02, 0x01, 0x72, 0x01,
	};
	struct ath11k_vif *arvif;
	u8 custom[256];
	int custom_len = 0;

	if (m52xq_wow_apf <= 0)
		return;

	if (enable && (m52xq_wow_apf == 9 || m52xq_wow_apf == 5)) {
		custom_len = m52xq_wow_apf_parse(m52xq_wow_apf_hex, custom, sizeof(custom));
		if (custom_len <= 0) {
			ath11k_warn(ar->ab, "apf: bad m52xq_wow_apf_hex (%d), no program\n", custom_len);
			return;
		}
	}

	list_for_each_entry(arvif, &ar->arvifs, list) {
		if (arvif->vdev_type != WMI_VDEV_TYPE_STA || !arvif->is_up)
			continue;
		if (m52xq_wow_apf == 4 || m52xq_wow_apf == 5) {
			/* stock path: APF off, program into the work memory, APF on (resume: APF off) */
			ath11k_wmi_apf_enable(ar, arvif->vdev_id, false);
			if (enable) {
				if (m52xq_wow_apf == 4)
					ath11k_wmi_apf_write_memory(ar, arvif->vdev_id, m52xq_wow_apf_ver, pass_ucast, sizeof(pass_ucast));
				else
					ath11k_wmi_apf_write_memory(ar, arvif->vdev_id, m52xq_wow_apf_ver, custom, custom_len);
				ath11k_wmi_apf_enable(ar, arvif->vdev_id, true);
			}
			continue;
		}
		if (!enable)
			ath11k_wmi_apf_del_instructions(ar, arvif->vdev_id);
		else if (m52xq_wow_apf == 1)
			ath11k_wmi_apf_set_instructions(ar, arvif->vdev_id, pass_all, sizeof(pass_all));
		else if (m52xq_wow_apf == 9)
			ath11k_wmi_apf_set_instructions(ar, arvif->vdev_id, custom, custom_len);
		else if (m52xq_wow_apf >= 3)
			ath11k_wmi_apf_set_instructions(ar, arvif->vdev_id, pass_ip_tcp_icmp, sizeof(pass_ip_tcp_icmp));
		else
			ath11k_wmi_apf_set_instructions(ar, arvif->vdev_id, pass_ucast, sizeof(pass_ucast));
	}
}

static const struct wiphy_wowlan_support ath11k_wowlan_support = {
	.flags = WIPHY_WOWLAN_DISCONNECT |
		 WIPHY_WOWLAN_MAGIC_PKT |
		 WIPHY_WOWLAN_SUPPORTS_GTK_REKEY |
		 WIPHY_WOWLAN_GTK_REKEY_FAILURE,
	.pattern_min_len = WOW_MIN_PATTERN_SIZE,
	.pattern_max_len = WOW_MAX_PATTERN_SIZE,
	.max_pkt_offset = WOW_MAX_PKT_OFFSET,
};

int ath11k_wow_enable(struct ath11k_base *ab)
{
	struct ath11k *ar = ath11k_ab_to_ar(ab, 0);
	int i, ret;

	clear_bit(ATH11K_FLAG_HTC_SUSPEND_COMPLETE, &ab->dev_flags);

	for (i = 0; i < ATH11K_WOW_RETRY_NUM; i++) {
		reinit_completion(&ab->htc_suspend);

		ret = ath11k_wmi_wow_enable(ar);
		if (ret) {
			ath11k_warn(ab, "failed to issue wow enable: %d\n", ret);
			return ret;
		}

		ret = wait_for_completion_timeout(&ab->htc_suspend, 3 * HZ);
		if (ret == 0) {
			ath11k_warn(ab,
				    "timed out while waiting for htc suspend completion\n");
			return -ETIMEDOUT;
		}

		if (test_bit(ATH11K_FLAG_HTC_SUSPEND_COMPLETE, &ab->dev_flags))
			/* success, suspend complete received */
			return 0;

		ath11k_warn(ab, "htc suspend not complete, retrying (try %d)\n",
			    i);
		msleep(ATH11K_WOW_RETRY_WAIT_MS);
	}

	ath11k_warn(ab, "htc suspend not complete, failing after %d tries\n", i);

	return -ETIMEDOUT;
}

int ath11k_wow_wakeup(struct ath11k_base *ab)
{
	struct ath11k *ar = ath11k_ab_to_ar(ab, 0);
	int ret;

	/* In the case of WCN6750, WoW wakeup is done
	 * by sending SMP2P power save exit message
	 * to the target processor.
	 */
	if (ab->hw_params.smp2p_wow_exit)
		return 0;

	reinit_completion(&ab->wow.wakeup_completed);

	ret = ath11k_wmi_wow_host_wakeup_ind(ar);
	if (ret) {
		ath11k_warn(ab, "failed to send wow wakeup indication: %d\n",
			    ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&ab->wow.wakeup_completed, 3 * HZ);
	if (ret == 0) {
		ath11k_warn(ab, "timed out while waiting for wow wakeup completion\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int ath11k_wow_vif_cleanup(struct ath11k_vif *arvif)
{
	struct ath11k *ar = arvif->ar;
	int i, ret;

	for (i = 0; i < WOW_EVENT_MAX; i++) {
		ret = ath11k_wmi_wow_add_wakeup_event(ar, arvif->vdev_id, i, 0);
		if (ret) {
			ath11k_warn(ar->ab, "failed to issue wow wakeup for event %s on vdev %i: %d\n",
				    wow_wakeup_event(i), arvif->vdev_id, ret);
			return ret;
		}
	}

	for (i = 0; i < ar->wow.max_num_patterns; i++) {
		ret = ath11k_wmi_wow_del_pattern(ar, arvif->vdev_id, i);
		if (ret) {
			ath11k_warn(ar->ab, "failed to delete wow pattern %d for vdev %i: %d\n",
				    i, arvif->vdev_id, ret);
			return ret;
		}
	}

	return 0;
}

static int ath11k_wow_cleanup(struct ath11k *ar)
{
	struct ath11k_vif *arvif;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	list_for_each_entry(arvif, &ar->arvifs, list) {
		ret = ath11k_wow_vif_cleanup(arvif);
		if (ret) {
			ath11k_warn(ar->ab, "failed to clean wow wakeups on vdev %i: %d\n",
				    arvif->vdev_id, ret);
			return ret;
		}
	}

	return 0;
}

/* Convert a 802.3 format to a 802.11 format.
 *         +------------+-----------+--------+----------------+
 * 802.3:  |dest mac(6B)|src mac(6B)|type(2B)|     body...    |
 *         +------------+-----------+--------+----------------+
 *                |__         |_______    |____________  |________
 *                   |                |                |          |
 *         +--+------------+----+-----------+---------------+-----------+
 * 802.11: |4B|dest mac(6B)| 6B |src mac(6B)|  8B  |type(2B)|  body...  |
 *         +--+------------+----+-----------+---------------+-----------+
 */
/* clang stack usage explodes if this is inlined */
static noinline_for_stack
void ath11k_wow_convert_8023_to_80211(struct cfg80211_pkt_pattern *new,
				      const struct cfg80211_pkt_pattern *old)
{
	u8 hdr_8023_pattern[ETH_HLEN] = {};
	u8 hdr_8023_bit_mask[ETH_HLEN] = {};
	u8 hdr_80211_pattern[WOW_HDR_LEN] = {};
	u8 hdr_80211_bit_mask[WOW_HDR_LEN] = {};
	u8 bytemask[WOW_MAX_PATTERN_SIZE] = {};

	int total_len = old->pkt_offset + old->pattern_len;
	int hdr_80211_end_offset;

	struct ieee80211_hdr_3addr *new_hdr_pattern =
		(struct ieee80211_hdr_3addr *)hdr_80211_pattern;
	struct ieee80211_hdr_3addr *new_hdr_mask =
		(struct ieee80211_hdr_3addr *)hdr_80211_bit_mask;
	struct ethhdr *old_hdr_pattern = (struct ethhdr *)hdr_8023_pattern;
	struct ethhdr *old_hdr_mask = (struct ethhdr *)hdr_8023_bit_mask;
	int hdr_len = sizeof(*new_hdr_pattern);

	struct rfc1042_hdr *new_rfc_pattern =
		(struct rfc1042_hdr *)(hdr_80211_pattern + hdr_len);
	struct rfc1042_hdr *new_rfc_mask =
		(struct rfc1042_hdr *)(hdr_80211_bit_mask + hdr_len);
	int rfc_len = sizeof(*new_rfc_pattern);
	int i;

	/* convert bitmask to bytemask */
	for (i = 0; i < old->pattern_len; i++)
		if (old->mask[i / 8] & BIT(i % 8))
			bytemask[i] = 0xff;

	memcpy(hdr_8023_pattern + old->pkt_offset,
	       old->pattern, ETH_HLEN - old->pkt_offset);
	memcpy(hdr_8023_bit_mask + old->pkt_offset,
	       bytemask, ETH_HLEN - old->pkt_offset);

	/* Copy destination address */
	memcpy(new_hdr_pattern->addr1, old_hdr_pattern->h_dest, ETH_ALEN);
	memcpy(new_hdr_mask->addr1, old_hdr_mask->h_dest, ETH_ALEN);

	/* Copy source address */
	memcpy(new_hdr_pattern->addr3, old_hdr_pattern->h_source, ETH_ALEN);
	memcpy(new_hdr_mask->addr3, old_hdr_mask->h_source, ETH_ALEN);

	/* Copy logic link type */
	memcpy(&new_rfc_pattern->snap_type,
	       &old_hdr_pattern->h_proto,
	       sizeof(old_hdr_pattern->h_proto));
	memcpy(&new_rfc_mask->snap_type,
	       &old_hdr_mask->h_proto,
	       sizeof(old_hdr_mask->h_proto));

	/* Compute new pkt_offset */
	if (old->pkt_offset < ETH_ALEN)
		new->pkt_offset = old->pkt_offset +
			offsetof(struct ieee80211_hdr_3addr, addr1);
	else if (old->pkt_offset < offsetof(struct ethhdr, h_proto))
		new->pkt_offset = old->pkt_offset +
			offsetof(struct ieee80211_hdr_3addr, addr3) -
			offsetof(struct ethhdr, h_source);
	else
		new->pkt_offset = old->pkt_offset + hdr_len + rfc_len - ETH_HLEN;

	/* Compute new hdr end offset */
	if (total_len > ETH_HLEN)
		hdr_80211_end_offset = hdr_len + rfc_len;
	else if (total_len > offsetof(struct ethhdr, h_proto))
		hdr_80211_end_offset = hdr_len + rfc_len + total_len - ETH_HLEN;
	else if (total_len > ETH_ALEN)
		hdr_80211_end_offset = total_len - ETH_ALEN +
			offsetof(struct ieee80211_hdr_3addr, addr3);
	else
		hdr_80211_end_offset = total_len +
			offsetof(struct ieee80211_hdr_3addr, addr1);

	new->pattern_len = hdr_80211_end_offset - new->pkt_offset;

	memcpy((u8 *)new->pattern,
	       hdr_80211_pattern + new->pkt_offset,
	       new->pattern_len);
	memcpy((u8 *)new->mask,
	       hdr_80211_bit_mask + new->pkt_offset,
	       new->pattern_len);

	if (total_len > ETH_HLEN) {
		/* Copy frame body */
		memcpy((u8 *)new->pattern + new->pattern_len,
		       (void *)old->pattern + ETH_HLEN - old->pkt_offset,
		       total_len - ETH_HLEN);
		memcpy((u8 *)new->mask + new->pattern_len,
		       bytemask + ETH_HLEN - old->pkt_offset,
		       total_len - ETH_HLEN);

		new->pattern_len += total_len - ETH_HLEN;
	}
}

static int ath11k_wmi_pno_check_and_convert(struct ath11k *ar, u32 vdev_id,
					    struct cfg80211_sched_scan_request *nd_config,
					    struct wmi_pno_scan_req *pno)
{
	int i, j;
	u8 ssid_len;

	pno->enable = 1;
	pno->vdev_id = vdev_id;
	pno->uc_networks_count = nd_config->n_match_sets;

	if (!pno->uc_networks_count ||
	    pno->uc_networks_count > WMI_PNO_MAX_SUPP_NETWORKS)
		return -EINVAL;

	if (nd_config->n_channels > WMI_PNO_MAX_NETW_CHANNELS_EX)
		return -EINVAL;

	/* Filling per profile params */
	for (i = 0; i < pno->uc_networks_count; i++) {
		ssid_len = nd_config->match_sets[i].ssid.ssid_len;

		if (ssid_len == 0 || ssid_len > 32)
			return -EINVAL;

		pno->a_networks[i].ssid.ssid_len = ssid_len;

		memcpy(pno->a_networks[i].ssid.ssid,
		       nd_config->match_sets[i].ssid.ssid,
		       nd_config->match_sets[i].ssid.ssid_len);
		pno->a_networks[i].authentication = 0;
		pno->a_networks[i].encryption     = 0;
		pno->a_networks[i].bcast_nw_type  = 0;

		/* Copying list of valid channel into request */
		pno->a_networks[i].channel_count = nd_config->n_channels;
		pno->a_networks[i].rssi_threshold = nd_config->match_sets[i].rssi_thold;

		for (j = 0; j < nd_config->n_channels; j++) {
			pno->a_networks[i].channels[j] =
					nd_config->channels[j]->center_freq;
		}
	}

	/* set scan to passive if no SSIDs are specified in the request */
	if (nd_config->n_ssids == 0)
		pno->do_passive_scan = true;
	else
		pno->do_passive_scan = false;

	for (i = 0; i < nd_config->n_ssids; i++) {
		j = 0;
		while (j < pno->uc_networks_count) {
			if (pno->a_networks[j].ssid.ssid_len ==
				nd_config->ssids[i].ssid_len &&
			(memcmp(pno->a_networks[j].ssid.ssid,
				nd_config->ssids[i].ssid,
				pno->a_networks[j].ssid.ssid_len) == 0)) {
				pno->a_networks[j].bcast_nw_type = BCAST_HIDDEN;
				break;
			}
			j++;
		}
	}

	if (nd_config->n_scan_plans == 2) {
		pno->fast_scan_period = nd_config->scan_plans[0].interval * MSEC_PER_SEC;
		pno->fast_scan_max_cycles = nd_config->scan_plans[0].iterations;
		pno->slow_scan_period =
			nd_config->scan_plans[1].interval * MSEC_PER_SEC;
	} else if (nd_config->n_scan_plans == 1) {
		pno->fast_scan_period = nd_config->scan_plans[0].interval * MSEC_PER_SEC;
		pno->fast_scan_max_cycles = 1;
		pno->slow_scan_period = nd_config->scan_plans[0].interval * MSEC_PER_SEC;
	} else {
		ath11k_warn(ar->ab, "Invalid number of scan plans %d !!",
			    nd_config->n_scan_plans);
	}

	if (nd_config->flags & NL80211_SCAN_FLAG_RANDOM_ADDR) {
		/* enable mac randomization */
		pno->enable_pno_scan_randomization = 1;
		memcpy(pno->mac_addr, nd_config->mac_addr, ETH_ALEN);
		memcpy(pno->mac_addr_mask, nd_config->mac_addr_mask, ETH_ALEN);
	}

	pno->delay_start_time = nd_config->delay;

	/* Current FW does not support min-max range for dwell time */
	pno->active_max_time = WMI_ACTIVE_MAX_CHANNEL_TIME;
	pno->passive_max_time = WMI_PASSIVE_MAX_CHANNEL_TIME;

	return 0;
}

static int ath11k_vif_wow_set_wakeups(struct ath11k_vif *arvif,
				      struct cfg80211_wowlan *wowlan)
{
	int ret, i;
	unsigned long wow_mask = 0;
	struct ath11k *ar = arvif->ar;
	const struct cfg80211_pkt_pattern *patterns = wowlan->patterns;
	int pattern_id = 0;

	/* Setup requested WOW features */
	switch (arvif->vdev_type) {
	case WMI_VDEV_TYPE_IBSS:
		__set_bit(WOW_BEACON_EVENT, &wow_mask);
		fallthrough;
	case WMI_VDEV_TYPE_AP:
		__set_bit(WOW_DEAUTH_RECVD_EVENT, &wow_mask);
		__set_bit(WOW_DISASSOC_RECVD_EVENT, &wow_mask);
		__set_bit(WOW_PROBE_REQ_WPS_IE_EVENT, &wow_mask);
		__set_bit(WOW_AUTH_REQ_EVENT, &wow_mask);
		__set_bit(WOW_ASSOC_REQ_EVENT, &wow_mask);
		__set_bit(WOW_HTT_EVENT, &wow_mask);
		__set_bit(WOW_RA_MATCH_EVENT, &wow_mask);
		break;
	case WMI_VDEV_TYPE_STA:
		if (wowlan->disconnect) {
			__set_bit(WOW_DEAUTH_RECVD_EVENT, &wow_mask);
			__set_bit(WOW_DISASSOC_RECVD_EVENT, &wow_mask);
			__set_bit(WOW_BMISS_EVENT, &wow_mask);
			__set_bit(WOW_CSA_IE_EVENT, &wow_mask);
		}

		if (wowlan->magic_pkt)
			__set_bit(WOW_MAGIC_PKT_RECVD_EVENT, &wow_mask);

		/*
		 * M52XQ: the vendor driver (qcacld pmo_set_sta_wow_bitmask) also enables WOW_HTT_EVENT and WOW_RA_MATCH_EVENT
		 * (wake on a unicast frame addressed to us) for a STA; with only PATTERN_MATCH the WCN6750 firmware never
		 * woke the host for a ping, while a beacon-loss wake (WOW_BMISS_EVENT) reached the host (IRQ 210).
		 */
		if (wowlan->n_patterns) {
			__set_bit(WOW_HTT_EVENT, &wow_mask);
			__set_bit(WOW_RA_MATCH_EVENT, &wow_mask);
		}

		if (wowlan->nd_config) {
			struct wmi_pno_scan_req *pno;
			int ret;

			pno = kzalloc_obj(*pno);
			if (!pno)
				return -ENOMEM;

			ar->nlo_enabled = true;

			ret = ath11k_wmi_pno_check_and_convert(ar, arvif->vdev_id,
							       wowlan->nd_config, pno);
			if (!ret) {
				ath11k_wmi_wow_config_pno(ar, arvif->vdev_id, pno);
				__set_bit(WOW_NLO_DETECTED_EVENT, &wow_mask);
			}

			kfree(pno);
		}
		break;
	default:
		break;
	}

	for (i = 0; i < wowlan->n_patterns; i++) {
		u8 ath_pattern[WOW_MAX_PATTERN_SIZE] = {};
		u8 ath_bitmask[WOW_MAX_PATTERN_SIZE] = {};
		struct cfg80211_pkt_pattern new_pattern = {};

		new_pattern.pattern = ath_pattern;
		new_pattern.mask = ath_bitmask;
		if (patterns[i].pattern_len > WOW_MAX_PATTERN_SIZE)
			continue;

		if (ar->wmi->wmi_ab->wlan_resource_config.rx_decap_mode ==
		    ATH11K_HW_TXRX_NATIVE_WIFI) {
			if (patterns[i].pkt_offset < ETH_HLEN) {
				ath11k_wow_convert_8023_to_80211(&new_pattern,
								 &patterns[i]);
			} else {
				int j;

				new_pattern = patterns[i];
				new_pattern.mask = ath_bitmask;

				/* convert bitmask to bytemask */
				for (j = 0; j < patterns[i].pattern_len; j++)
					if (patterns[i].mask[j / 8] & BIT(j % 8))
						ath_bitmask[j] = 0xff;

				new_pattern.pkt_offset += WOW_HDR_LEN - ETH_HLEN;
			}
		}

		if (WARN_ON(new_pattern.pattern_len > WOW_MAX_PATTERN_SIZE))
			return -EINVAL;

		ret = ath11k_wmi_wow_add_pattern(ar, arvif->vdev_id,
						 pattern_id,
						 new_pattern.pattern,
						 new_pattern.mask,
						 new_pattern.pattern_len,
						 new_pattern.pkt_offset);
		if (ret) {
			ath11k_warn(ar->ab, "failed to add pattern %i to vdev %i: %d\n",
				    pattern_id,
				    arvif->vdev_id, ret);
			return ret;
		}

		pattern_id++;
		__set_bit(WOW_PATTERN_MATCH_EVENT, &wow_mask);

		/*
		 * M52XQ: the vendor driver (qcacld pmo_configure_wow_sta) gives the WCN6750 firmware its patterns in the
		 * 802.3 layout (unicast pattern = own MAC at offset 0); add the user pattern in that layout as well, next to
		 * the converted 802.11 one, so whichever layout the firmware matches against is covered.
		 */
		if (ar->wmi->wmi_ab->wlan_resource_config.rx_decap_mode == ATH11K_HW_TXRX_NATIVE_WIFI &&
		    patterns[i].pkt_offset < ETH_HLEN) {
			u8 raw_mask[WOW_MAX_PATTERN_SIZE] = {};
			int j;

			for (j = 0; j < patterns[i].pattern_len; j++)
				if (patterns[i].mask[j / 8] & BIT(j % 8))
					raw_mask[j] = 0xff;

			ret = ath11k_wmi_wow_add_pattern(ar, arvif->vdev_id, pattern_id,
							 patterns[i].pattern, raw_mask,
							 patterns[i].pattern_len,
							 patterns[i].pkt_offset);
			if (ret)
				ath11k_warn(ar->ab, "failed to add 802.3 pattern %i to vdev %i: %d\n",
					    pattern_id, arvif->vdev_id, ret);
			else
				pattern_id++;
		}
	}

	/*
	 * M52XQ v227: qcacld sends the whole event bitmap at once in the 4-word layout of the firmware; for a station
	 * it also enables BETTER_AP, GTK_ERR, NLO, CLIENT_KICKOUT, EXTSCAN (25), OEM_RESPONSE (30), TDLS_CONN_TRACKER (32)
	 * and 11D_SCAN (35) (pmo_set_sta_wow_bitmask)
	 */
	if (m52xq_wow_evt_array) {
		u32 bitmaps[4] = { (u32)wow_mask, 0, 0, 0 };

		if (arvif->vdev_type == WMI_VDEV_TYPE_STA) {
			bitmaps[0] |= BIT(WOW_BETTER_AP_EVENT) | BIT(WOW_GTK_ERR_EVENT) |
				      BIT(WOW_NLO_DETECTED_EVENT) | BIT(WOW_CLIENT_KICKOUT_EVENT) | BIT(25) | BIT(30);
			bitmaps[1] |= BIT(32 - 32) | BIT(35 - 32);
		}
		return ath11k_wmi_wow_add_wakeup_events(ar, arvif->vdev_id, bitmaps);
	}

	for (i = 0; i < WOW_EVENT_MAX; i++) {
		if (!test_bit(i, &wow_mask))
			continue;
		ret = ath11k_wmi_wow_add_wakeup_event(ar, arvif->vdev_id, i, 1);
		if (ret) {
			ath11k_warn(ar->ab, "failed to enable wakeup event %s on vdev %i: %d\n",
				    wow_wakeup_event(i), arvif->vdev_id, ret);
			return ret;
		}
	}

	return 0;
}

static int ath11k_wow_set_wakeups(struct ath11k *ar,
				  struct cfg80211_wowlan *wowlan)
{
	struct ath11k_vif *arvif;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	list_for_each_entry(arvif, &ar->arvifs, list) {
		ret = ath11k_vif_wow_set_wakeups(arvif, wowlan);
		if (ret) {
			ath11k_warn(ar->ab, "failed to set wow wakeups on vdev %i: %d\n",
				    arvif->vdev_id, ret);
			return ret;
		}
	}

	return 0;
}

static int ath11k_vif_wow_clean_nlo(struct ath11k_vif *arvif)
{
	int ret = 0;
	struct ath11k *ar = arvif->ar;

	switch (arvif->vdev_type) {
	case WMI_VDEV_TYPE_STA:
		if (ar->nlo_enabled) {
			struct wmi_pno_scan_req *pno;

			pno = kzalloc_obj(*pno);
			if (!pno)
				return -ENOMEM;

			pno->enable = 0;
			ar->nlo_enabled = false;
			ret = ath11k_wmi_wow_config_pno(ar, arvif->vdev_id, pno);
			kfree(pno);
		}
		break;
	default:
		break;
	}
	return ret;
}

static int ath11k_wow_nlo_cleanup(struct ath11k *ar)
{
	struct ath11k_vif *arvif;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	list_for_each_entry(arvif, &ar->arvifs, list) {
		ret = ath11k_vif_wow_clean_nlo(arvif);
		if (ret) {
			ath11k_warn(ar->ab, "failed to clean nlo settings on vdev %i: %d\n",
				    arvif->vdev_id, ret);
			return ret;
		}
	}

	return 0;
}

static int ath11k_wow_set_hw_filter(struct ath11k *ar)
{
	struct ath11k_vif *arvif;
	u32 bitmap;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	if (m52xq_hw_filter <= 0) /* v227: no hw data filter during WoW */
		return 0;

	list_for_each_entry(arvif, &ar->arvifs, list) {
		bitmap = m52xq_hw_filter; /* v227: upstream DROP_NON_ICMPV6_MC | DROP_NON_ARP_BC, stock only the latter */
		ret = ath11k_wmi_hw_data_filter_cmd(ar, arvif->vdev_id,
						    bitmap,
						    true);
		if (ret) {
			ath11k_warn(ar->ab, "failed to set hw data filter on vdev %i: %d\n",
				    arvif->vdev_id, ret);
			return ret;
		}
	}

	return 0;
}

static int ath11k_wow_clear_hw_filter(struct ath11k *ar)
{
	struct ath11k_vif *arvif;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	list_for_each_entry(arvif, &ar->arvifs, list) {
		ret = ath11k_wmi_hw_data_filter_cmd(ar, arvif->vdev_id, 0, false);

		if (ret) {
			ath11k_warn(ar->ab, "failed to clear hw data filter on vdev %i: %d\n",
				    arvif->vdev_id, ret);
			return ret;
		}
	}

	return 0;
}

static int ath11k_wow_arp_ns_offload(struct ath11k *ar, bool enable)
{
	struct ath11k_vif *arvif;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	list_for_each_entry(arvif, &ar->arvifs, list) {
		if (arvif->vdev_type != WMI_VDEV_TYPE_STA)
			continue;

		ret = ath11k_wmi_arp_ns_offload(ar, arvif, enable);

		if (ret) {
			ath11k_warn(ar->ab, "failed to set arp ns offload vdev %i: enable %d, ret %d\n",
				    arvif->vdev_id, enable, ret);
			return ret;
		}
	}

	return 0;
}

static int ath11k_gtk_rekey_offload(struct ath11k *ar, bool enable)
{
	struct ath11k_vif *arvif;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	list_for_each_entry(arvif, &ar->arvifs, list) {
		if (arvif->vdev_type != WMI_VDEV_TYPE_STA ||
		    !arvif->is_up ||
		    !arvif->rekey_data.enable_offload)
			continue;

		/* get rekey info before disable rekey offload */
		if (!enable) {
			ret = ath11k_wmi_gtk_rekey_getinfo(ar, arvif);
			if (ret) {
				ath11k_warn(ar->ab, "failed to request rekey info vdev %i, ret %d\n",
					    arvif->vdev_id, ret);
				return ret;
			}
		}

		ret = ath11k_wmi_gtk_rekey_offload(ar, arvif, enable);

		if (ret) {
			ath11k_warn(ar->ab, "failed to offload gtk reky vdev %i: enable %d, ret %d\n",
				    arvif->vdev_id, enable, ret);
			return ret;
		}
	}

	return 0;
}

static int ath11k_wow_protocol_offload(struct ath11k *ar, bool enable)
{
	int ret;

	ret = ath11k_wow_arp_ns_offload(ar, enable);
	if (ret) {
		ath11k_warn(ar->ab, "failed to offload ARP and NS %d %d\n",
			    enable, ret);
		return ret;
	}

	ret = ath11k_gtk_rekey_offload(ar, enable);
	if (ret) {
		ath11k_warn(ar->ab, "failed to offload gtk rekey %d %d\n",
			    enable, ret);
		return ret;
	}

	return 0;
}

static int ath11k_wow_set_keepalive(struct ath11k *ar,
				    enum wmi_sta_keepalive_method method,
				    u32 interval)
{
	struct ath11k_vif *arvif;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	list_for_each_entry(arvif, &ar->arvifs, list) {
		ret = ath11k_mac_vif_set_keepalive(arvif, method, interval);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * M52XQ v245: the firmware sometimes does not acknowledge the WoW enable ("htc suspend not complete" three times,
 * "failed to start wow: -110", 31.10.2026 01:24). op_suspend then returned 1, mac80211 disconnected the station
 * (deauth by local choice) and the system slept for minutes with a dead Wi-Fi: no message or call could wake it. Undo
 * everything that was set for the WoW and let the suspend fail instead (the system stays awake, the next try follows).
 */
static void ath11k_wow_abort_suspend(struct ath11k *ar)
{
	struct ath11k_vif *arvif;

	ath11k_wow_clear_hw_filter(ar);
	ath11k_wow_apf(ar, false);
	if (m52xq_wow_inactivity) {
		list_for_each_entry(arvif, &ar->arvifs, list)
			if (arvif->vdev_type == WMI_VDEV_TYPE_STA && arvif->is_up)
				ath11k_wmi_set_sta_ps_param(ar, arvif->vdev_id, WMI_STA_PS_PARAM_INACTIVITY_TIME, 200);
	}
	ath11k_wow_protocol_offload(ar, false);
	ath11k_wow_set_keepalive(ar, WMI_STA_KEEPALIVE_METHOD_NULL_FRAME, WMI_STA_KEEPALIVE_INTERVAL_DISABLE);
	ath11k_dp_rx_pktlog_start(ar->ab);
}

int ath11k_wow_op_suspend(struct ieee80211_hw *hw,
			  struct cfg80211_wowlan *wowlan)
{
	struct ath11k *ar = hw->priv;
	bool abort_suspend = false;
	int ret;

	ret = ath11k_mac_wait_tx_complete(ar);
	if (ret) {
		ath11k_warn(ar->ab, "failed to wait tx complete: %d\n", ret);
		return ret;
	}

	mutex_lock(&ar->conf_mutex);

	ret = ath11k_dp_rx_pktlog_stop(ar->ab, true);
	if (ret) {
		ath11k_warn(ar->ab,
			    "failed to stop dp rx (and timer) pktlog during wow suspend: %d\n",
			    ret);
		goto exit;
	}

	ret =  ath11k_wow_cleanup(ar);
	if (ret) {
		ath11k_warn(ar->ab, "failed to clear wow wakeup events: %d\n",
			    ret);
		goto exit;
	}

	ret = ath11k_wow_set_wakeups(ar, wowlan);
	if (ret) {
		ath11k_warn(ar->ab, "failed to set wow wakeup events: %d\n",
			    ret);
		goto cleanup;
	}

	ret = ath11k_wow_protocol_offload(ar, true);
	if (ret) {
		ath11k_warn(ar->ab, "failed to set wow protocol offload events: %d\n",
			    ret);
		goto cleanup;
	}

	ret = ath11k_wow_set_hw_filter(ar);
	if (ret) {
		ath11k_warn(ar->ab, "failed to set hw filter: %d\n",
			    ret);
		goto cleanup;
	}

	ret = ath11k_wow_set_keepalive(ar,
				       WMI_STA_KEEPALIVE_METHOD_NULL_FRAME,
				       m52xq_wow_keepalive ? m52xq_wow_keepalive : WMI_STA_KEEPALIVE_INTERVAL_DEFAULT);
	if (ret) {
		ath11k_warn(ar->ab, "failed to enable wow keepalive: %d\n", ret);
		goto cleanup;
	}

	ath11k_wow_apf(ar, true); /* v228 */

	/* M52XQ v227: like qcacld pmo_configure_vdev_suspend_params: the STA PS inactivity time for the WoW time */
	if (m52xq_wow_inactivity) {
		struct ath11k_vif *arvif;

		list_for_each_entry(arvif, &ar->arvifs, list)
			if (arvif->vdev_type == WMI_VDEV_TYPE_STA && arvif->is_up)
				ath11k_wmi_set_sta_ps_param(ar, arvif->vdev_id, WMI_STA_PS_PARAM_INACTIVITY_TIME,
							    m52xq_wow_inactivity);
	}

	ret = ath11k_wow_enable(ar->ab);
	if (ret) {
		ath11k_warn(ar->ab, "failed to start wow: %d\n", ret);
		abort_suspend = true;	/* v245 */
		goto cleanup;
	}

	ret = ath11k_dp_rx_pktlog_stop(ar->ab, false);
	if (ret) {
		ath11k_warn(ar->ab,
			    "failed to stop dp rx pktlog during wow suspend: %d\n",
			    ret);
		goto cleanup;
	}

	ath11k_ce_stop_shadow_timers(ar->ab);
	ath11k_dp_stop_shadow_timers(ar->ab);

	ath11k_hif_irq_disable(ar->ab);
	ath11k_hif_ce_irq_disable(ar->ab);

	ret = ath11k_hif_suspend(ar->ab);
	if (ret) {
		ath11k_warn(ar->ab, "failed to suspend hif: %d\n", ret);
		goto wakeup;
	}

	goto exit;

wakeup:
	ath11k_wow_wakeup(ar->ab);

cleanup:
	ath11k_wow_cleanup(ar);
	if (abort_suspend)
		ath11k_wow_abort_suspend(ar);

exit:
	mutex_unlock(&ar->conf_mutex);
	if (abort_suspend)
		return -EBUSY;	/* mac80211: err < 0 = the suspend is cancelled, the station stays connected */
	return ret ? 1 : 0;
}

void ath11k_wow_op_set_wakeup(struct ieee80211_hw *hw, bool enabled)
{
	struct ath11k *ar = hw->priv;

	mutex_lock(&ar->conf_mutex);
	device_set_wakeup_enable(ar->ab->dev, enabled);
	mutex_unlock(&ar->conf_mutex);
}

int ath11k_wow_op_resume(struct ieee80211_hw *hw)
{
	struct ath11k *ar = hw->priv;
	int ret;

	mutex_lock(&ar->conf_mutex);

	ret = ath11k_hif_resume(ar->ab);
	if (ret) {
		ath11k_warn(ar->ab, "failed to resume hif: %d\n", ret);
		goto exit;
	}

	ath11k_hif_ce_irq_enable(ar->ab);
	ath11k_hif_irq_enable(ar->ab);

	ret = ath11k_dp_rx_pktlog_start(ar->ab);
	if (ret) {
		ath11k_warn(ar->ab, "failed to start rx pktlog from wow: %d\n", ret);
		goto exit;
	}

	ret = ath11k_wow_wakeup(ar->ab);
	if (ret) {
		ath11k_warn(ar->ab, "failed to wakeup from wow: %d\n", ret);
		goto exit;
	}

	ret = ath11k_wow_nlo_cleanup(ar);
	if (ret) {
		ath11k_warn(ar->ab, "failed to cleanup nlo: %d\n", ret);
		goto exit;
	}

	ret = ath11k_wow_clear_hw_filter(ar);
	if (ret) {
		ath11k_warn(ar->ab, "failed to clear hw filter: %d\n", ret);
		goto exit;
	}

	ath11k_wow_apf(ar, false); /* v228 */

	/* M52XQ v227: the inactivity time of the awake station again (qcacld ps_data_inactivity_timeout default 200) */
	if (m52xq_wow_inactivity) {
		struct ath11k_vif *arvif;

		list_for_each_entry(arvif, &ar->arvifs, list)
			if (arvif->vdev_type == WMI_VDEV_TYPE_STA && arvif->is_up)
				ath11k_wmi_set_sta_ps_param(ar, arvif->vdev_id, WMI_STA_PS_PARAM_INACTIVITY_TIME, 200);
	}

	ret = ath11k_wow_protocol_offload(ar, false);
	if (ret) {
		ath11k_warn(ar->ab, "failed to clear wow protocol offload events: %d\n",
			    ret);
		goto exit;
	}

	ret = ath11k_wow_set_keepalive(ar,
				       WMI_STA_KEEPALIVE_METHOD_NULL_FRAME,
				       WMI_STA_KEEPALIVE_INTERVAL_DISABLE);
	if (ret) {
		ath11k_warn(ar->ab, "failed to disable wow keepalive: %d\n", ret);
		goto exit;
	}

exit:
	if (ret) {
		switch (ar->state) {
		case ATH11K_STATE_ON:
			ar->state = ATH11K_STATE_RESTARTING;
			ret = 1;
			break;
		case ATH11K_STATE_OFF:
		case ATH11K_STATE_RESTARTING:
		case ATH11K_STATE_RESTARTED:
		case ATH11K_STATE_WEDGED:
		case ATH11K_STATE_FTM:
			ath11k_warn(ar->ab, "encountered unexpected device state %d on resume, cannot recover\n",
				    ar->state);
			ret = -EIO;
			break;
		}
	}

	mutex_unlock(&ar->conf_mutex);
	return ret;
}

int ath11k_wow_init(struct ath11k *ar)
{
	if (!test_bit(WMI_TLV_SERVICE_WOW, ar->wmi->wmi_ab->svc_map))
		return 0;

	ar->wow.wowlan_support = ath11k_wowlan_support;

	if (ar->wmi->wmi_ab->wlan_resource_config.rx_decap_mode ==
	    ATH11K_HW_TXRX_NATIVE_WIFI) {
		ar->wow.wowlan_support.pattern_max_len -= WOW_MAX_REDUCE;
		ar->wow.wowlan_support.max_pkt_offset -= WOW_MAX_REDUCE;
	}

	if (test_bit(WMI_TLV_SERVICE_NLO, ar->wmi->wmi_ab->svc_map)) {
		ar->wow.wowlan_support.flags |= WIPHY_WOWLAN_NET_DETECT;
		ar->wow.wowlan_support.max_nd_match_sets = WMI_PNO_MAX_SUPP_NETWORKS;
	}

	ar->wow.max_num_patterns = ATH11K_WOW_PATTERNS;
	ar->wow.wowlan_support.n_patterns = ar->wow.max_num_patterns;
	ar->hw->wiphy->wowlan = &ar->wow.wowlan_support;

	device_set_wakeup_capable(ar->ab->dev, true);

	return 0;
}
