/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _WIFI_IF_H_
#define _WIFI_IF_H_

#include "wifi_al.h"
#include "fhost_api.h"

#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

/// WIFI_API index message
enum wifi_api_msg_index {
    WIFI_API_SCAN_START,
    WIFI_API_SCAN_CANCEL,
    WIFI_API_STA_CONNECT,
    WIFI_API_STA_DISCONNECT,
    WIFI_API_AP_START,
    WIFI_API_AP_STOP,
    WIFI_API_AP_DEAUTH,
    WIFI_API_P2P_GO_START,
    WIFI_API_P2P_GO_STOP,
    WIFI_API_P2P_GO_DEAUTH,
    WIFI_API_WPS_PBC_SET,
    WIFI_API_MONITOR_START,
    WIFI_API_MONITOR_STOP,
};

/// WIFI_API handler description
struct wifi_api_handler {
    /// message index
    int index;
    /// handler function
    void (*func) (void *msg);
};

/// WIFI_API message header
struct wifi_api_msg_hdr {
    /// Length, in bytes, of the message (including this header)
    uint16_t len;
    /// ID of the message.
    uint16_t id;
};

struct wifi_api_set_wps_pbc_tag {
    struct wifi_api_msg_hdr hdr;
    /// Vif idx
    int fhost_vif_idx;
};

struct wifi_api_monitor_start_tag {
    struct wifi_api_msg_hdr hdr;
    /// Vif idx
    int fhost_vif_idx;
    /// Interface configuration
    struct fhost_vif_monitor_cfg cfg;
};

struct wifi_api_monitor_stop_tag {
    struct wifi_api_msg_hdr hdr;
    /// Vif idx
    int fhost_vif_idx;
};

extern aic_wifi_event_cb g_aic_wifi_event_cb;

int wifi_api_msg_send(struct wifi_api_msg_hdr *cmd);
int aicwf_is_5g_enable(void);
int aic_wifi_init_mac(void);
int aic_wifi_open(int mode, void *param, u16 chip_id);
int aic_wifi_close(int mode);
void aic_wifi_close_drvier(void);

#endif /* _WIFI_IF_H_ */