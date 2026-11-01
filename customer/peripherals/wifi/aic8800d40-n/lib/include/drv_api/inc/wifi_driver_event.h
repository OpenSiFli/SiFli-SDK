/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Shim header replacing platform/wifi/wifi_driver_event.h
 * Contains enum/struct definitions only (no platform code).
 */

#ifndef __WIFI_DRIVER_EVENT_H__
#define __WIFI_DRIVER_EVENT_H__

#ifndef ETH_ALEN
    #define ETH_ALEN 6
#endif

enum wifi_dev_driver_status
{
    WIFI_DEVICE_DRIVER_UNKNOWN = 0,
    WIFI_DEVICE_DRIVER_LOADED,
    WIFI_DEVICE_DRIVER_UNLOAD,
};

struct wifi_dev_event
{
    enum wifi_dev_driver_status drv_status;
    unsigned char local_mac_addr[ETH_ALEN];
};

enum wifi_p2p_event_type
{
    WIFI_P2P_EVENT_UNKNOWN = 0,
    WIFI_P2P_EVENT_GOT_PRO_DISC_REQ_AFTER_GONEGO_OK,
    WIFI_P2P_EVENT_ON_ASSOC_REQ,
    WIFI_P2P_EVENT_ON_DISASSOC,
};

struct wifi_p2p_event
{
    enum wifi_p2p_event_type event_type;
    unsigned char peer_dev_mac_addr[ETH_ALEN];
    unsigned int peer_dev_port;
};

enum wifi_ap_event_type
{
    WIFI_AP_EVENT_UNKNOWN = 0,
    WIFI_AP_EVENT_ON_DISASSOC,
    WIFI_AP_EVENT_ON_ASSOC,
};

struct wifi_ap_event
{
    enum wifi_ap_event_type event_type;
    unsigned char peer_dev_mac_addr[ETH_ALEN];
};

enum wifi_sta_event_type
{
    WIFI_STA_EVENT_UNKNOWN = 0,
    WIFI_STA_EVENT_ON_DISASSOC,
    WIFI_STA_EVENT_ON_ASSOC_FAIL,
    WIFI_STA_EVENT_ON_ASSOC,
};

struct wifi_sta_event
{
    enum wifi_sta_event_type event_type;
    unsigned char peer_dev_mac_addr[ETH_ALEN];
};

enum wifi_drv_event_type
{
    WIFI_DRV_EVENT_DEFAULT = 0,
    WIFI_DRV_EVENT_NET_DEVICE,
    WIFI_DRV_EVENT_AP,
    WIFI_DRV_EVENT_P2P,
    WIFI_DRV_EVENT_STA,
};

union wifi_drv_event_node
{
    struct wifi_dev_event     dev_event;
    struct wifi_p2p_event     p2p_event;
    struct wifi_ap_event      ap_event;
    struct wifi_sta_event     sta_event;
};

typedef struct
{
    enum wifi_drv_event_type type;
    union wifi_drv_event_node node;
} wifi_drv_event;

typedef int (*wifi_drv_event_cbk)(const wifi_drv_event *drv_event);
typedef void (*wifi_priv_event_cb)(unsigned char *MAC_addr, unsigned char mode);

int wifi_drv_event_set_cbk(wifi_drv_event_cbk cbk);
void wifi_drv_event_reset_cbk(void);
int wifi_drv_event_initialize(void);
int wifi_drv_event_finalize(void);
void wifi_drv_event_upper_netdev_event(struct wifi_dev_event *upper_event);
void wifi_drv_event_upper_p2p_event(struct wifi_p2p_event *upper_event);
void wifi_drv_event_upper_ap_event(struct wifi_ap_event *upper_event);

#endif /* __WIFI_DRIVER_EVENT_H__ */
