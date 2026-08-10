/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _WIFI_DEF_H_
#define _WIFI_DEF_H_

#include "aic_plat_types.h"

#define AIC_MAC_ADDR_LEN    6
#define AIC_MAX_SSID_LEN    32
#define AIC_MAX_PASSWD_LEN  64
#define AIC_MIN_KEY_LEN     8
#define AIC_MAX_KEY_LEN     64
#define AIC_MAX_AP_COUNT    20
#define AIC_MAX_STA_COUNT   4

typedef enum
{
    WIFI_MODE_STA = 0,
    WIFI_MODE_AP,
    WIFI_MODE_P2P,
    WIFI_MODE_RFTEST,
    WIFI_MODE_UNKNOWN
} AIC_WIFI_MODE;

typedef enum
{
    AP_PS_CLK_1 = 1,  // 3:7 -> active 3, doze 7
    AP_PS_CLK_2,      // 5:5 -> active 5, doze 5
    AP_PS_CLK_3,      // 7:3 -> active 7, doze 3
    AP_PS_CLK_4,      // 8:2 -> active 8, doze 2
    AP_PS_CLK_5,      // 9:1 -> active 9, doze 1
} AIC_WIFI_AP_PS_CLK_MODE;

typedef enum
{
    KEY_NONE = 0,
    KEY_WEP,
    KEY_WPA,
    KEY_WPA_WPA2,
    KEY_WPA2,
    KEY_WPA2_WPA3,
    KEY_WPA3,
    KEY_MAX_VALUE,
} AIC_WIFI_SECURITY_TYPE;

typedef enum
{
    SCAN_RESULT_EVENT = 0,
    SCAN_DONE_EVENT,
    JOIN_SUCCESS_EVENT,
    JOIN_FAIL_EVENT,
    LEAVE_RESULT_EVENT,
    ASSOC_IND_EVENT,
    DISASSOC_STA_IND_EVENT,
    DISASSOC_P2P_IND_EVENT,
    EAPOL_STA_FIN_EVENT,
    EAPOL_P2P_FIN_EVENT,
    PRO_DISC_REQ_EVENT,
    STA_DISCONNECT_EVENT,
    UNKNOWN_EVENT
} AIC_WIFI_EVENT;

struct ap_ssid
{
    /// Actual length of the SSID.
    unsigned char length;
    /// Array containing the SSID name.
    unsigned char array[AIC_MAX_SSID_LEN];
};

struct ap_passwd
{
    unsigned char length;
    unsigned char array[AIC_MAX_PASSWD_LEN];
};

struct aic_ap_cfg
{
    struct ap_ssid aic_ap_ssid;
    /**
     * Password: if OPEN, set length to 0
     */
    struct ap_passwd aic_ap_passwd;
    /**
     * Band : 0 -> 2.4G, 1 -> 5G
     */
    unsigned char band;
    /**
     * Type : 0 -> 20M, 1 -> 40M
     */
    unsigned char type;
    /**
     * Channel Number : 2.4G (1 ~ 13), 5G (36/40/44/48/149/153/157/161/165)
     */
    unsigned char channel;
    /**
     * Hidden ssid : 0 -> no hidden, 1 -> zero length, 2 -> zero contents
     */
    unsigned char hidden_ssid;
    /**
     * Value for maximum station inactivity, seconds
     */
    unsigned int max_inactivity;
    /**
     * Enable wifi6
     */
    unsigned char enable_he;
    /**
     * Enable ACS
     */
    unsigned char enable_acs;
    /**
     * Beacon interval
     */
    unsigned char bcn_interval;

    AIC_WIFI_SECURITY_TYPE sercurity_type;
    /**
     * STA number: max(NX_REMOTE_STA_MAX)
     */
    unsigned char sta_num;

	unsigned char dtim;
};

struct aic_sta_cfg
{
    struct ap_ssid aic_ap_ssid;
    /**
     * Password: if OPEN, set length to 0
     */
    struct ap_passwd aic_ap_passwd;
};

struct aic_p2p_cfg
{
    struct ap_ssid aic_p2p_ssid;
    /**
     * Password: if OPEN, set length to 0
     */
    struct ap_passwd aic_ap_passwd;
    /**
     * Band : 0 -> 2.4G, 1 -> 5G
     */
    unsigned char band;
    /**
     * Type : 0 -> 20M, 1 -> 40M
     */
    unsigned char type;
    /**
     * Channel Number : 2.4G (1 ~ 13), 5G (36/40/44/48/149/153/157/161/165)
     */
    unsigned char channel;
    /**
     * Enable wifi6
     */
    unsigned char enable_he;
    /**
     * Enable ACS
     */
    unsigned char enable_acs;
};

typedef struct _aic_wifi_scan_result_data
{
    unsigned int  reserved[16];
}  aic_wifi_scan_result_data;

typedef struct _aic_wifi_scan_done_data
{
    unsigned int  reserved[1];
}  aic_wifi_scan_done_data;

typedef struct _aic_wifi_join_data
{
    unsigned int  reserved[1];
} aic_wifi_join_data;

typedef struct _aic_wifi_leave_data
{
    unsigned int  reserved[1];
} aic_wifi_leave_data;

typedef struct _aic_wifi_auth_deauth_data
{
    unsigned int  reserved[6];
} aic_wifi_auth_deauth_data;

typedef struct _aic_wifi_event_data
{
    union
    {
        struct _aic_wifi_scan_result_data  scan_result_data;
        struct _aic_wifi_scan_done_data  scan_done_data;
        struct _aic_wifi_join_data  join_data;
        struct _aic_wifi_leave_data  leave_data;
        struct _aic_wifi_auth_deauth_data  auth_deauth_data;
    }data;
    #ifdef CONFIG_P2P
    unsigned int p2p_enabled;
    unsigned int p2p_dev_port_num;
    #endif
} aic_wifi_event_data;

typedef int (* wifi_event_handle)(void *enData);
typedef void (* aic_wifi_event_cb)(AIC_WIFI_EVENT enEvent, aic_wifi_event_data *enData);

#endif /* _WIFI_DEF_H_ */