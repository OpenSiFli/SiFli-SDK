/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi shared type definitions (no ops dispatch).
 */

#ifndef _WIFI_PORT_H_
#define _WIFI_PORT_H_

#include "wifi_def.h"
#include "aic_porting_types.h"
#include "rtos_port.h"

struct aic_wifi_dev
{
    rtos_semaphore   wifi_task_sem;
    rtos_task_handle wifi_task_init_hdl;

    uint32_t         wifi_drive_op;
    uint32_t         wifi_config_st;
    uint32_t         wifi_init_status;
    AIC_WIFI_MODE    wifi_mode;
    AIC_WIFI_AP_MODE wifi_ap_mode;
    int              g_wifi_init;
};

extern struct aic_wifi_dev aicwf_wifi_dev;

#endif /* _WIFI_PORT_H_ */
