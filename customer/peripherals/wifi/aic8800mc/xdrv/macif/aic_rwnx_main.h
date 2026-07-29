/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _AIC_RWNX_MAIN_H_
#define _AIC_RWNX_MAIN_H_

#define AP_SSID_BUF_MAX             33
#define AP_PSWD_BUF_MAX             64
#define HTTP_URI_BUF_MAX            256

// aic work mode status
enum AIC_MODE_STATUS
{
    AIC_MODE_IDLE,          // AIC doesn't connect AP and start AP
    AIC_MODE_WLAN,          // AIC connect to AP
    AIC_MODE_AP_CONFIG,     // AIC start ap to config network
    AIC_MODE_AP_DIRECT,     // AIC start ap to transfer data between host and AIC
};

// wlan connect status about mcu
enum WLAN_STATUS
{
    WLAN_DISCONNECT,        // AIC-STA hasn't connected AP
    WLAN_CONNECTED,         // AIC-STA has connected AP
    WLAN_SCANNING,
    WLAN_CONNECTING,
};

// wlan connect failure result
enum WLAN_CONN_FAIL_RESULT
{
    WLAN_CONN_FAIL,         // AIC-STA failed to connect AP
    WLAN_PSWD_WRONG,        // AIC-STA failed to connect AP since passward wrong
};

// fhostif custom message connect req
struct fhcustmsg_connect_req
{
    uint8_t ssid[AP_SSID_BUF_MAX];
    uint8_t pw[AP_PSWD_BUF_MAX];
};

struct fhcustmsg_connect_result_cfm
{
    int8_t status;
    uint8_t ussid[AP_SSID_BUF_MAX];
};

struct fhcustmsg_connect_ind
{
    uint8_t ussid[AP_SSID_BUF_MAX];
    int8_t  rssi;
    uint32_t ip;
    uint32_t mk;
    uint32_t gw;
    uint32_t dns1;
    uint32_t dns2;
    uint8_t  bssid[6];
    uint8_t  pw[AP_PSWD_BUF_MAX];
    uint32_t akm;
};

struct fhcustmsg_disconnect_ind
{
    int8_t status;
    uint8_t ussid[AP_SSID_BUF_MAX];
};

struct fhcustmsg_set_mac_req
{
    uint8_t mac_addr[6];
};

struct fhcustmsg_mac_addr_cfm
{
    uint8_t mac_addr[6];
};

struct fhcustmsg_wlan_status_cfm
{
    uint8_t status;
    int8_t  rssi;
    uint8_t ussid[AP_SSID_BUF_MAX];
    uint32_t ip;
    uint32_t mk;
    uint32_t gw;
    uint32_t dns1;
    uint32_t dns2;

    uint16_t freq;
    uint8_t width;
    uint8_t band;
    // 0:bg, 1:n/ac, 2:ax
    uint8_t format;
    // unit: bps
    uint32_t max_rate;

    uint8_t  bssid[6];
    uint8_t  pw[AP_PSWD_BUF_MAX];
    uint32_t akm;
};

struct fhcustmsg_scan_wifi_result_cfm
{
    //uint8_t  scan_num;
    int8_t   rssi;
    uint8_t  bssid[6];
    uint8_t  ssid[AP_SSID_BUF_MAX];
    uint32_t akm;
    int      channal;
} __packed;

enum STA_CFG_REQ_OP
{
    STA_CFG_REQ_OP_GET_LIST,
    STA_CFG_REQ_OP_SET,
    STA_CFG_REQ_OP_DELETE,
    STA_CFG_REQ_OP_CLEAR,
};

struct fhcustmsg_sta_cfg_req
{
    uint32_t op;
    uint8_t  enable;
    uint8_t  bssid[6];
    uint8_t  ssid[AP_SSID_BUF_MAX];
};

struct sta_info
{
    uint8_t enable;
    uint8_t ssid[AP_SSID_BUF_MAX];
    uint8_t pw[AP_PSWD_BUF_MAX];
    uint32_t akm;
};

struct fhcustmsg_sta_cfg_cfm
{
    int8_t status;
    int8_t count;
    struct sta_info sta_info[8];
};

#endif /* _AIC_RWNX_MAIN_H_ */