/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file    aic8800_wlan_dev.c
 * @brief   AIC8800 -> RT-Thread Wi-Fi management framework glue layer (rt_wlan_dev_ops)
 *
 * Build condition: CONFIG_AIC8800_USE_RT_WLAN=y (Kconfig: aic8800d40-n/Kconfig)
 *           This option selects RT_USING_WIFI, bringing up the rt_wlan framework in one step.
 *
 * Architecture (option B, dual mode):
 *   Enabled -> netif ownership is handed to the rt_wlan framework (eth_device/netdev), the driver only fills in ops;
 *              two devices are registered: wlan0 (RT_WLAN_FLAG_STA_ONLY) + wlan1 (RT_WLAN_FLAG_AP_ONLY),
 *              sharing the same ops table (the framework dispatches by flags, consistent with the aic8800d40 driver).
 *              RX from netif_port.c goes through aic8800_wlan_get_device()/aic8800_wlan_get_ap_device()
 *              and is injected via rt_wlan_dev_report_data,
 *              TX goes through ops->wlan_send->aicwf_tx_eth_data_process straight to the hardware.
 *   Disabled (default) -> route 1 is kept (driver builds its own netif, full STA+AP+P2P concurrency), this file is not built.
 *
 * Event model:
 *   AIC8800 host lib async event -> wifi_port.c:aic_wifi_event_callback -> m_ext_event_cb
 *   (registered by this file at init via aic_wifi_ext_event_register) -> translated into rt_wlan events.
 *   Deterministic results triggered by an op (scan/join/disconnect/softap/ap_stop) are reported synchronously;
 *   passive disconnects (LEAVE/STA_DISCONNECT) go through the async bridge.
 *
 * Framework API (rtos/rtthread/components/drivers/wlan/wlan_dev.h):
 *   Register:   rt_wlan_dev_register(wlan, "wlan0", ops, flag, user_data)
 *   RX deliver: rt_wlan_dev_report_data(device, buff, len)   [called by netif_port.c]
 *   Events:     rt_wlan_dev_indicate_event_handle(device, RT_WLAN_DEV_EVT_xxx, buff)
 *   TX exit:    framework linkoutput -> ops->wlan_send(wlan, buff, len)
 */

#include <rtthread.h>
#include <string.h>

/* RT-Thread Wi-Fi device framework (CPPPATH injected by SConscript from components/drivers/wlan) */
#include "wlan_dev.h"

/* AIC8800 host lib control plane API + types */
#include "wifi_api.h"     /* aic_wifi_scan / start_sta / start_ap / stop_sta / rssi / mode_set / set_ps_lvl */
#include "wifi_def.h"     /* AIC_WIFI_MODE / AIC_WIFI_EVENT / aic_ap_cfg / aic_wifi_event_cb */
#include "netif_api.h"    /* aic_get_mac_address / aic_set_mac_address */

#define DBG_TAG   "aic.wlan"
#define DBG_LVL   DBG_INFO
#include <rtdbg.h>

/* ============================================================================
 * External symbols (implemented on the platform side; those without a public header declaration are extern'ed here)
 * ============================================================================ */
/* wifi_port.c: [AIC8800-PORT] generic event extension callback registration (see m_ext_event_cb in that file) */
extern void aic_wifi_ext_event_register(aic_wifi_event_cb cb);
/* netif_port.c: Ethernet frame TX direct-to-hardware entry */
extern int aicwf_tx_eth_data_process(uint8_t *data, uint32_t len, void *msg);

/* ============================================================================
 * Module state
 * ============================================================================ */
static struct rt_wlan_device aic8800_wlan_dev;      /* STA device instance (wlan0) */
static struct rt_wlan_device aic8800_wlan_dev_ap;   /* AP  device instance (wlan1) */
static int g_ps_level  = 0;   /* power save level cache (AIC has no ps_lvl getter) */
static int g_channel   = 0;   /* set_channel cache, takes effect on softap (STA channel follows AP) */

/* ============================================================================
 * Helpers: scan result translation + reporting
 * ============================================================================ */

/**
 * @brief Get the AIC scan results, translate them into rt_wlan_info, report SCAN_REPORT one by one, then SCAN_DONE
 *
 * Called in the SCAN_DONE_EVENT async bridge (AIC wifi task context).
 * Note: wifi_ap_info_t has no security field -> SECURITY_UNKNOWN is filled in uniformly (degraded in M4, can be improved in M5).
 */
static void aic8800_report_scan_results(struct rt_wlan_device *wlan)
{
    static wifi_ap_list_t ap_list;   /* static: ~3KB, avoids stack pressure in the event context (AIC wifi task) */
    struct rt_wlan_info info;
    struct rt_wlan_buff buff;
    int i, cnt;

    rt_memset(&ap_list, 0, sizeof(ap_list));
    aic_wifi_getscan(&ap_list);

    buff.data = &info;
    buff.len  = sizeof(info);

    cnt = (int)ap_list.ap_count;
    if (cnt > MAX_AP_COUNT) cnt = MAX_AP_COUNT;

    for (i = 0; i < cnt; i++)
    {
        wifi_ap_info_t *ap = &ap_list.ap_info[i];
        int slen = 0;
        while (slen < (int)sizeof(ap->ssid) && ap->ssid[slen] != 0) slen++;

        rt_memset(&info, 0, sizeof(info));
        info.security = SECURITY_UNKNOWN;                              /* AIC scan results carry no security */
        info.band     = (ap->channel > 14) ? RT_802_11_BAND_5GHZ       /* 5GHZ=0 */
                        : RT_802_11_BAND_2_4GHZ;    /* 2.4GHZ=1 */
        info.datarate = 0;
        info.channel  = (rt_int16_t)ap->channel;
        info.rssi     = (rt_int16_t)(int)ap->rssi;                     /* the unsigned value stores two's complement, (int) restores the negative value */
        info.hidden   = 0;
        info.ssid.len = (rt_uint8_t)slen;
        if (slen > 0)
            rt_memcpy(info.ssid.val, ap->ssid, slen);
        rt_memcpy(info.bssid, ap->bssid, sizeof(info.bssid));

        rt_wlan_dev_indicate_event_handle(wlan, RT_WLAN_DEV_EVT_SCAN_REPORT, &buff);
    }

    rt_wlan_dev_indicate_event_handle(wlan, RT_WLAN_DEV_EVT_SCAN_DONE, RT_NULL);
    LOG_I("scan done: %d AP reported", cnt);
}

/**
 * @brief AIC event -> rt_wlan event bridge (registered as m_ext_event_cb in wifi_port.c)
 *
 * Handles asynchronous/passive events only; deterministic results triggered by an op are
 * reported synchronously by each op to avoid duplication.
 */
static void aic8800_wlan_event_bridge(AIC_WIFI_EVENT event, aic_wifi_event_data *data)
{
    struct rt_wlan_device *wlan = &aic8800_wlan_dev;
    (void)data;

    switch (event)
    {
    /* Note: SCAN_DONE_EVENT is not handled here. The host library's aic_wifi_scan() is blocking
     * (the scan has finished when it returns, and the event is not guaranteed to be reported back),
     * so the scan results are reported directly before the wlan_scan op returns; otherwise the
     * framework's rt_wlan_scan_with_info() would time out waiting for SCAN_DONE. */
    case LEAVE_RESULT_EVENT:
    case STA_DISCONNECT_EVENT:
        LOG_I("event: disconnect (async)");
        rt_wlan_dev_indicate_event_handle(wlan, RT_WLAN_DEV_EVT_DISCONNECT, RT_NULL);
        break;
    default:
        break;
    }
}

/* ============================================================================
 * rt_wlan_dev_ops implementation
 * ============================================================================ */

/** wlan_init — called before the framework's first use; registers the event bridge (idempotent). Hardware init is done by the route 1 init path */
static rt_err_t aic8800_wlan_init(struct rt_wlan_device *wlan)
{
    (void)wlan;
    aic_wifi_ext_event_register(aic8800_wlan_event_bridge);
    LOG_I("wlan_init: event bridge registered");
    return RT_EOK;
}

/** wlan_mode — STA/AP mode setting; STA_AP concurrency is not supported by the standard framework (returns ENOSYS, use route 1) */
static rt_err_t aic8800_wlan_mode(struct rt_wlan_device *wlan, rt_wlan_mode_t mode)
{
    (void)wlan;
    switch (mode)
    {
    case RT_WLAN_STATION:
        aic_wifi_mode_set(WIFI_MODE_STA);
        return RT_EOK;  /* 0 */
    case RT_WLAN_AP:
        aic_wifi_mode_set(WIFI_MODE_AP);
        return RT_EOK;  /* 1 */
    default:
        LOG_W("mode %d unsupported (STA_AP 并发请用路线1)", (int)mode);
        return -RT_ENOSYS;
    }
}

/** wlan_scan — trigger a scan; aic_wifi_scan() blocks until the scan finishes, and after it returns
 *  SCAN_REPORT (per AP) + SCAN_DONE are reported synchronously (the framework's rt_wlan_scan_with_info
 *  has already set up complete before calling this op, so events reported inside the op are not lost). */
static rt_err_t aic8800_wlan_scan(struct rt_wlan_device *wlan, struct rt_scan_info *scan_info)
{
    int ret;
    (void)scan_info;

    ret = aic_wifi_scan();
    LOG_I("wlan_scan trigger ret=%d", ret);
    if (ret != 0)
        return -RT_ERROR;

    aic8800_report_scan_results(wlan);
    return RT_EOK;
}

/** wlan_join — connect STA (aic_wifi_start_sta blocks until connected/timeout) + report CONNECT/CONNECT_FAIL synchronously */
static rt_err_t aic8800_wlan_join(struct rt_wlan_device *wlan, struct rt_sta_info *sta_info)
{
    char ssid[RT_WLAN_SSID_MAX_LENGTH + 1] = {0};
    char pw[RT_WLAN_PASSWORD_MAX_LENGTH + 1] = {0};
    int slen, klen, ret;

    slen = sta_info->ssid.len;
    if (slen > RT_WLAN_SSID_MAX_LENGTH) slen = RT_WLAN_SSID_MAX_LENGTH;
    rt_memcpy(ssid, sta_info->ssid.val, slen);
    klen = sta_info->key.len;
    if (klen > RT_WLAN_PASSWORD_MAX_LENGTH) klen = RT_WLAN_PASSWORD_MAX_LENGTH;
    if (klen > 0) rt_memcpy(pw, sta_info->key.val, klen);

    LOG_I("wlan_join ssid=%s", ssid);
    ret = aic_wifi_start_sta((uint8_t *)ssid, (uint8_t *)pw, 30000);   /* 30s timeout */
    if (ret == 0)
    {
        rt_wlan_dev_indicate_event_handle(wlan, RT_WLAN_DEV_EVT_CONNECT, RT_NULL);
        LOG_I("wlan_join success");
        return RT_EOK;
    }
    rt_wlan_dev_indicate_event_handle(wlan, RT_WLAN_DEV_EVT_CONNECT_FAIL, RT_NULL);
    LOG_E("wlan_join failed: %d", ret);
    return -RT_ERROR;
}

/** wlan_softap — build aic_ap_cfg, start SoftAP + report AP_START */
static rt_err_t aic8800_wlan_softap(struct rt_wlan_device *wlan, struct rt_ap_info *ap_info)
{
    struct aic_ap_cfg cfg;
    int slen, klen, ret;

    rt_memset(&cfg, 0, sizeof(cfg));

    slen = ap_info->ssid.len;
    if (slen > AIC_MAX_SSID_LEN) slen = AIC_MAX_SSID_LEN;
    cfg.aic_ap_ssid.length = (unsigned char)slen;
    rt_memcpy(cfg.aic_ap_ssid.array, ap_info->ssid.val, slen);

    klen = ap_info->key.len;
    if (klen > AIC_MAX_PASSWD_LEN) klen = AIC_MAX_PASSWD_LEN;
    /* OPEN network: set the password length to 0 */
    if (ap_info->security != SECURITY_OPEN && klen > 0)
    {
        cfg.aic_ap_passwd.length = (unsigned char)klen;
        rt_memcpy(cfg.aic_ap_passwd.array, ap_info->key.val, klen);
        cfg.sercurity_type = KEY_WPA2;                 /* encryption fixed to WPA2 (covers the most common case; can be refined in M5) */
    }
    else
    {
        cfg.aic_ap_passwd.length = 0;
        cfg.sercurity_type = KEY_NONE;
    }

    cfg.channel     = (unsigned char)ap_info->channel;
    cfg.band        = (ap_info->channel > 14) ? 1 : 0; /* AIC: 0=2.4G, 1=5G */
    cfg.type        = 0;                               /* 20M */
    cfg.hidden_ssid = ap_info->hidden ? 1 : 0;
    cfg.bcn_interval = 100;                            /* 100ms default beacon */
    cfg.sta_num      = 4;                              /* default max 4 STAs */
    cfg.dtim         = 1;

    LOG_I("wlan_softap ssid=%.*s ch=%d", slen, (char *)cfg.aic_ap_ssid.array, cfg.channel);
    ret = aic_wifi_start_ap(&cfg);
    if (ret == 0)
    {
        g_channel = cfg.channel;
        rt_wlan_dev_indicate_event_handle(wlan, RT_WLAN_DEV_EVT_AP_START, RT_NULL);
        return RT_EOK;
    }
    LOG_E("wlan_softap failed: %d", ret);
    return -RT_ERROR;
}

/** wlan_disconnect — disconnect STA + report DISCONNECT synchronously */
static rt_err_t aic8800_wlan_disconnect(struct rt_wlan_device *wlan)
{
    LOG_I("wlan_disconnect");
    aic_wifi_stop_sta();
    rt_wlan_dev_indicate_event_handle(wlan, RT_WLAN_DEV_EVT_DISCONNECT, RT_NULL);
    return RT_EOK;
}

/** wlan_ap_stop — stop SoftAP + report AP_STOP */
static rt_err_t aic8800_wlan_ap_stop(struct rt_wlan_device *wlan)
{
    LOG_I("wlan_ap_stop");
    aic_wifi_stop_ap();
    rt_wlan_dev_indicate_event_handle(wlan, RT_WLAN_DEV_EVT_AP_STOP, RT_NULL);
    return RT_EOK;
}

/** wlan_ap_deauth — kick out a STA; AIC has no clean unicast deauth API -> degraded */
static rt_err_t aic8800_wlan_ap_deauth(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    (void)wlan;
    (void)mac;
    LOG_W("wlan_ap_deauth unsupported");
    return -RT_ENOSYS;
}

/** wlan_scan_stop — AIC has no separate scan-stop API; the scan ends naturally, return EOK */
static rt_err_t aic8800_wlan_scan_stop(struct rt_wlan_device *wlan)
{
    (void)wlan;
    return RT_EOK;
}

/** wlan_get_rssi — RSSI of the current connection (dBm) */
static int aic8800_wlan_get_rssi(struct rt_wlan_device *wlan)
{
    (void)wlan;
    return (int)aic_wifi_ap_rssi_get();
}

/** wlan_set_powersave — maps to aic_wifi_set_ps_lvl; the level is cached for get queries */
static rt_err_t aic8800_wlan_set_powersave(struct rt_wlan_device *wlan, int level)
{
    (void)wlan;
    g_ps_level = level;
    aic_wifi_set_ps_lvl(level);
    LOG_D("wlan_set_powersave level=%d", level);
    return RT_EOK;
}

/** wlan_get_powersave — returns the cached level (AIC has no ps_lvl getter) */
static int aic8800_wlan_get_powersave(struct rt_wlan_device *wlan)
{
    (void)wlan;
    return g_ps_level;
}

/** wlan_cfg_promisc — promiscuous mode; AIC8800 has no corresponding API -> degraded */
static rt_err_t aic8800_wlan_cfg_promisc(struct rt_wlan_device *wlan, rt_bool_t start)
{
    (void)wlan;
    (void)start;
    LOG_W("wlan_cfg_promisc unsupported");
    return -RT_ENOSYS;
}

/** wlan_cfg_filter — frame filtering; AIC8800 has no corresponding API -> degraded */
static rt_err_t aic8800_wlan_cfg_filter(struct rt_wlan_device *wlan, struct rt_wlan_filter *filter)
{
    (void)wlan;
    (void)filter;
    return -RT_ENOSYS;
}

/** wlan_set_channel — the STA channel follows the AP; cached here to take effect on softap */
static rt_err_t aic8800_wlan_set_channel(struct rt_wlan_device *wlan, int channel)
{
    (void)wlan;
    g_channel = channel;
    LOG_D("wlan_set_channel=%d (softap 生效)", channel);
    return RT_EOK;
}

/** wlan_get_channel — if connected, convert from the freq in connected_ap_info; otherwise return the cache */
static int aic8800_wlan_get_channel(struct rt_wlan_device *wlan)
{
    struct aic_connected_ap_info_t *info;
    (void)wlan;
    info = aic_wifi_get_sta_connected_ap_info();
    if (info && info->freq)
        return aic_wifi_phy_freq_to_channel(info->band, info->freq);
    return g_channel;
}

/** wlan_set_country — country code; AIC8800 has no corresponding API -> degraded */
static rt_err_t aic8800_wlan_set_country(struct rt_wlan_device *wlan, rt_country_code_t country_code)
{
    (void)wlan;
    (void)country_code;
    return -RT_ENOSYS;
}

/** wlan_get_country — degraded, returns the first enum value */
static rt_country_code_t aic8800_wlan_get_country(struct rt_wlan_device *wlan)
{
    (void)wlan;
    return (rt_country_code_t)0;
}

/** wlan_set_mac — write the local MAC (aic_set_mac_address) */
static rt_err_t aic8800_wlan_set_mac(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    (void)wlan;
    aic_set_mac_address((uint8_t *)mac);
    return RT_EOK;
}

/** wlan_get_mac — read the local MAC (aic_get_mac_address returns the base MAC) */
static rt_err_t aic8800_wlan_get_mac(struct rt_wlan_device *wlan, rt_uint8_t mac[])
{
    uint8_t *m;
    (void)wlan;
    m = aic_get_mac_address();
    if (m) rt_memcpy(mac, m, 6);
    else   rt_memset(mac, 0, 6);
    return RT_EOK;
}

/** wlan_recv — pull mode (not used by this driver); RX is push-based: netif_port.c -> rt_wlan_dev_report_data */
static int aic8800_wlan_recv(struct rt_wlan_device *wlan, void *buff, int len)
{
    (void)wlan;
    (void)buff;
    (void)len;
    return 0;
}

/** wlan_send — framework TX exit (lwIP linkoutput -> this function); Ethernet frames go straight to AIC hardware */
static int aic8800_wlan_send(struct rt_wlan_device *wlan, void *buff, int len)
{
    (void)wlan;
    return aicwf_tx_eth_data_process((uint8_t *)buff, (uint32_t)len, RT_NULL);
}

/* ============================================================================
 * ops structure
 * ============================================================================ */
static const struct rt_wlan_dev_ops aic8800_wlan_ops =
{
    .wlan_init          = aic8800_wlan_init,
    .wlan_mode          = aic8800_wlan_mode,
    .wlan_scan          = aic8800_wlan_scan,
    .wlan_join          = aic8800_wlan_join,
    .wlan_softap        = aic8800_wlan_softap,
    .wlan_disconnect    = aic8800_wlan_disconnect,
    .wlan_ap_stop       = aic8800_wlan_ap_stop,
    .wlan_ap_deauth     = aic8800_wlan_ap_deauth,
    .wlan_scan_stop     = aic8800_wlan_scan_stop,
    .wlan_get_rssi      = aic8800_wlan_get_rssi,
    .wlan_set_powersave = aic8800_wlan_set_powersave,
    .wlan_get_powersave = aic8800_wlan_get_powersave,
    .wlan_cfg_promisc   = aic8800_wlan_cfg_promisc,
    .wlan_cfg_filter    = aic8800_wlan_cfg_filter,
    .wlan_set_channel   = aic8800_wlan_set_channel,
    .wlan_get_channel   = aic8800_wlan_get_channel,
    .wlan_set_country   = aic8800_wlan_set_country,
    .wlan_get_country   = aic8800_wlan_get_country,
    .wlan_set_mac       = aic8800_wlan_set_mac,
    .wlan_get_mac       = aic8800_wlan_get_mac,
    .wlan_recv          = aic8800_wlan_recv,
    .wlan_send          = aic8800_wlan_send,
};

/* ============================================================================
 * Public interface: called by the netif_port.c RX bridge
 * ============================================================================ */
struct rt_wlan_device *aic8800_wlan_get_device(void)
{
    return &aic8800_wlan_dev;
}

/** AP device instance (used by the netif_port.c RX bridge after determining the vif) */
struct rt_wlan_device *aic8800_wlan_get_ap_device(void)
{
    return &aic8800_wlan_dev_ap;
}

/* ============================================================================
 * Init: register rt_wlan_device + event bridge
 *
 * Timing: INIT_DEVICE_EXPORT — the RT-Thread device init stage.
 * Hardware init is still done by aicwf_wifi_init_task of route 1; only the framework devices and
 * event callbacks are registered here, without touching the hardware, so it is boot-safe.
 * The framework calls wlan_init on the App's first rt_wlan_connect/scan.
 * ============================================================================ */
static int aic8800_wlan_dev_init(void)
{
    rt_err_t err;

    /* STA device: scan/connect/disconnect go through wlan0 */
    err = rt_wlan_dev_register(&aic8800_wlan_dev,
                               RT_WLAN_DEVICE_STA_NAME,
                               &aic8800_wlan_ops,
                               RT_WLAN_FLAG_STA_ONLY,
                               RT_NULL);
    if (err != RT_EOK)
    {
        LOG_E("register %s failed: %d", RT_WLAN_DEVICE_STA_NAME, (int)err);
        return -1;
    }

    /* AP device: rt_wlan_start_ap / wifi ap go through wlan1 (the framework requires a separate device for AP mode) */
    err = rt_wlan_dev_register(&aic8800_wlan_dev_ap,
                               RT_WLAN_DEVICE_AP_NAME,
                               &aic8800_wlan_ops,
                               RT_WLAN_FLAG_AP_ONLY,
                               RT_NULL);
    if (err != RT_EOK)
    {
        LOG_E("register %s failed: %d", RT_WLAN_DEVICE_AP_NAME, (int)err);
        return -1;
    }

    /* Register the event bridge (SCAN_DONE / LEAVE / STA_DISCONNECT -> rt_wlan events) */
    aic_wifi_ext_event_register(aic8800_wlan_event_bridge);

    LOG_I("AIC8800 registered: %s(STA) + %s(AP)",
          RT_WLAN_DEVICE_STA_NAME, RT_WLAN_DEVICE_AP_NAME);
    return 0;
}
INIT_DEVICE_EXPORT(aic8800_wlan_dev_init);
