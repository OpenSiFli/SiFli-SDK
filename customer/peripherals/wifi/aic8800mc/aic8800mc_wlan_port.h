/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AIC8800MC_WLAN_PORT_H__
#define __AIC8800MC_WLAN_PORT_H__

#include <rtthread.h>
#ifdef RT_USING_WIFI
    #include <wlan_mgnt.h>
#endif

int aic8800mc_wlan_init(void);
void aic8800mc_wlan_report_connect(void);
void aic8800mc_wlan_report_disconnect(void);
void aic8800mc_wlan_report_connect_fail(void);
void aic8800mc_wlan_report_scan_done(void);
void aic8800mc_wlan_report_scan_result(struct rt_wlan_info *info);
void aic8800mc_wlan_rx_data(void *buff, int len);

#endif
