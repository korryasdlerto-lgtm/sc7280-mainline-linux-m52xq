/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __Q6CORE_H__
#define __Q6CORE_H__

struct q6core_svc_api_info {
	uint32_t service_id;
	uint32_t api_version;
	uint32_t api_branch_version;
};

bool q6core_is_adsp_ready(void);
int q6core_get_svc_api_info(int svc_id, struct q6core_svc_api_info *ainfo);
int q6core_register_acdb_topologies(void);

#endif /* __Q6CORE_H__ */
