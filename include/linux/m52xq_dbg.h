/* SPDX-License-Identifier: GPL-2.0 */
/* M52XQ-GPU-DBG: rate-limit every debug call site to its first few hits */
#ifndef _LINUX_M52XQ_DBG_H
#define _LINUX_M52XQ_DBG_H
#include <linux/printk.h>
#define M52XQ_DBG(fmt, ...) ({					\
	static int __m52xq_cnt;					\
	if (__m52xq_cnt < 4) {					\
		__m52xq_cnt++;					\
		printk(KERN_ERR fmt, ##__VA_ARGS__);		\
	}							\
})
#endif
