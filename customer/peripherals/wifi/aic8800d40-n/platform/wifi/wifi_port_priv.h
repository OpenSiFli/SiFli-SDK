/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal shared declarations for wifi_port split files.
 * NOT for external consumption - use wifi_port.h or uapapi.h instead.
 */

#ifndef _WIFI_PORT_PRIV_H_
#define _WIFI_PORT_PRIV_H_

#include "wifi_port.h"
#include "plat_port.h"
#include "uapapi.h"

#include <string.h>
#ifndef os_strcmp
    #define os_strcmp(s1, s2) strcmp((s1), (s2))
#endif
#ifndef os_strcat
    #define os_strcat(s1, s2) strcat(s1, s2)
#endif
#ifndef os_strlen
    #define os_strlen(s) strlen(s)
#endif

/* Common macros */
#define AIC_DRIVER_REBOOT_OPEN      0x01
#define AIC_DRIVER_REBOOT_PU        0x06
#define AIC_DRIVER_REBOOT_ADP       0x04
#define AIC_DIRVER_REBOOT_PD        0x05
#define AIC_DRIVER_REBOOT_CLOSE     0x02
#define AIC_DRIVER_REBOOT_RESET     0x03
#define AIC_DRIVER__TEMP_TEST       0x08

#ifndef UAP_CFG_NOT_SEPARATE
    #define UAP_CFG_NOT_SEPARATE 1
#endif

#ifndef UAP_CFG_NOT_SEPARATE
    #define UAP_CFG_NOT_SEPARATE 1
#endif

/* ============================================================
 * Shared global variables
 * ============================================================ */

/* Core (wifi_port.c) */
extern int g_athandle;

/* AP (wifi_port_ap.c) */
extern int wifi_ap_isolate;
extern unsigned char wifi_bss_st;
extern struct aic_ap_cfg wifi_ap_cfg;
#if !UAP_CFG_NOT_SEPARATE
    extern struct aic_ap_cfg wifi_ap_cfg_2g4;
    extern struct aic_ap_cfg wifi_ap_cfg_5g;
#endif
extern struct aic_ap_cfg *user_ap_cfg;

/* WPS types (moved from wifi_port.c) */
typedef enum
{
    WPS_REG_INVALIDE = 0x0,
    WPS_REG_WAIT_FOR_NEW_SESSION,
    WPS_REG_ONGOING,
    WPS_REG_SUCC,
    WPS_REG_FAIL,
    WPS_REG_OVERLAP,
} WPS_RET_STATE;

enum wps_event
{
    WPS_EV_M2D,
    WPS_EV_FAIL,
    WPS_EV_SUCCESS,
    WPS_EV_PWD_AUTH_FAIL,
    WPS_EV_PBC_OVERLAP,
    WPS_EV_PBC_TIMEOUT,
    WPS_EV_PBC_ACTIVE,
    WPS_EV_PBC_DISABLE,
    WPS_EV_ER_AP_ADD,
    WPS_EV_ER_AP_REMOVE,
    WPS_EV_ER_ENROLLEE_ADD,
    WPS_EV_ER_ENROLLEE_REMOVE,
    WPS_EV_ER_AP_SETTINGS,
    WPS_EV_ER_SET_SELECTED_REGISTRAR,
    WPS_EV_AP_PIN_SUCCESS,
    WPS_EV_AP_PIN_TIMEOUT,
    WPS_EV_SELECTED_TIMEOUT
};

typedef void (*aic_wps_status_notify)(int event);
typedef void (*WSPNOTIFYFUNC)(unsigned char state);

struct plat_wps_dev
{
    int           wpsmethod;
    WPS_RET_STATE wps_ret;
};

/* P2P config (wifi_port_p2p.c) */
#ifdef CONFIG_P2P_SUPPORT
    extern struct aic_p2p_cfg aic_wifi_p2p_cfg;
#endif

/* ============================================================
 * AP functions (wifi_port_ap.c)
 * ============================================================ */
unsigned char app_wlan_status(void);
t_s32 app_wlan_bss_config(t_u32 start_stop);
t_s32 app_wlan_sta_list(sta_list *list);
int32_t app_wlan_sta_deauth(uap_802_11_mac_addr *mac);
t_s32 app_wlan_scan_channels_config(uap_config_scan_chan *param);
int32_t app_wlan_sta_conn_notify_reg(void (*notify_cb)(uint8_t, uint8_t *));
int32_t app_wlan_get_current_channel(void);
void app_wlan_get_channel_by_country(int *first, int *last);
void app_wlan_intra_bss_unicast_deny(int enable);
int app_wlan_params_set_filter_only(void *filter, uap_config_param *uap_cfg);
void app_wlan_configure_wifimac(uint8_t *mac);
int app_wlan_get_sta_num(void);

/* ============================================================
 * STA functions (wifi_port_sta.c)
 * ============================================================ */
uint32_t app_wlan_sta_get_ip(void);
uint8_t app_wlan_sta_show_scan_res(void);
int app_wlan_sta_start(uint8_t *ssid, uint8_t *pw, int timeout_ms);
int app_wlan_sta_stop(void);
int app_wlan_scan_start(void);

/* ============================================================
 * WPS functions (wifi_port_wps.c)
 * ============================================================ */
int app_wlan_wps_notify_reg(WSPNOTIFYFUNC NotifyFunc);
int app_wlan_wps_get_reg_status(void);
void app_wlan_wps_use_pbc(void);
void app_wlan_wps_launch(void);
void app_wlan_wps_disable(void);
void app_wlan_wps_enable(void);
void app_wlan_wps_use_cancel(void);
void app_wlan_wps_use_pin(char *pin);
int app_wlan_validate_checksum(unsigned long int PIN);

/* ============================================================
 * P2P functions (wifi_port_p2p.c)
 * ============================================================ */
#ifdef CONFIG_P2P_SUPPORT
    int app_wlan_p2p_start(const char *dev_name, const char *ssid, const char *passwd);
    int app_wlan_p2p_stop(void);
#endif

/* ============================================================
 * Core utility functions (wifi_port.c)
 * ============================================================ */
void SetWifiConfigFlag(uint8_t value);
unsigned char GetWifiConfigFlag(void);
uint8_t GetWifiStatus(void);
int GetMACAddr(uint8_t *macbuf);
void wifi_set_mode(AIC_WIFI_MODE mode);
int  wifi_switch_mode(AIC_WIFI_MODE mode);
void wifi_driver_action(unsigned int act);

#endif /* _WIFI_PORT_PRIV_H_ */
