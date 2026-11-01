/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _Q6_CVP_H
#define _Q6_CVP_H

#include "q6voice.h"

struct q6voice_session;

struct q6voice_session *q6cvp_session_create(enum q6voice_path_type path,
					     u16 tx_port, u16 rx_port);
int q6cvp_enable(struct q6voice_session *cvp, bool enable);
int q6cvp_set_media(struct q6voice_session *cvp, u16 rx_port, unsigned int rx_ch,
		    u16 tx_port, unsigned int tx_ch, unsigned int rate);

int q6cvp_set_mute(struct q6voice_session *cvp, u16 direction, bool mute);
bool q6cvp_dev_unmute_enabled(void);

#endif /*_Q6_CVP_H */
