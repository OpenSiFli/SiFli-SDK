/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WIFI_IF_H_
#define _WIFI_IF_H_

#include "wifi_api.h"

#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

extern aic_wifi_event_cb g_aic_wifi_event_cb;

int aicwf_is_5g_enable(void);
int aic_wifi_init_mac(void);
int aic_wifi_open(int mode, void *param);
int aic_wifi_close(int mode);
void aic_wifi_close_drvier(void);
int aic_wifi_set_mode(int mode);
#endif /* _WIFI_IF_H_ */

