/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi AP mode platform functions.
 */

#include "wifi_port_priv.h"
#include "wifi_api.h"
#include "netif_api.h"
#include "fmac_port.h"
#include "aic_log.h"
#include <string.h>

#define AP_SSID_DFT  "BTD-AP"
#define AP_PASS_DFT  "12345678"
#define AP_SSID_2G4  "BTD-2G4"
#define AP_PASS_2G4  "12345678"
#define AP_SSID_5G   "BTDs-5G"
#define AP_PASS_5G   "12345678"

unsigned int aic_feature_config = 0;

int wifi_ap_isolate = 0;
unsigned char wifi_bss_st = 0x0;

#if UAP_CFG_NOT_SEPARATE
struct aic_ap_cfg wifi_ap_cfg =
{
    .aic_ap_ssid = { sizeof(AP_SSID_DFT) - 1, AP_SSID_DFT },
    .aic_ap_passwd = { sizeof(AP_PASS_DFT) - 1, AP_PASS_DFT },
    .band = 1,
    .type = 1,
    .channel = 6,
    .hidden_ssid = 0,
    .max_inactivity = 60,
    .enable_he = 1,
    .enable_acs = 1,
    .bcn_interval = 100,
    .sta_num = 10,
    .sercurity_type = KEY_WPA2_WPA3,
};
#else
struct aic_ap_cfg wifi_ap_cfg_2g4 =
{
    .aic_ap_ssid = { sizeof(AP_SSID_2G4) - 1, AP_SSID_2G4 },
    .aic_ap_passwd = { sizeof(AP_PASS_2G4) - 1, AP_PASS_2G4 },
    .band = 0,
    .type = PHY_CHNL_BW_20,
    .channel = 6,
    .hidden_ssid = 0,
    .max_inactivity = 60,
    .enable_he = 1,
    .enable_acs = 0,
    .bcn_interval = 100,
    .sta_num = 10,
};

struct aic_ap_cfg wifi_ap_cfg_5g =
{
    .aic_ap_ssid = { sizeof(AP_SSID_5G) - 1, AP_SSID_5G },
    .aic_ap_passwd = { sizeof(AP_PASS_5G) - 1, AP_PASS_5G },
    .band = 1,
    .type = PHY_CHNL_BW_20,
    .channel = 6,
    .hidden_ssid = 0,
    .max_inactivity = 60,
    .enable_he = 1,
    .enable_acs = 0,
    .bcn_interval = 100,
    .sta_num = 10,
};
#endif

#if UAP_CFG_NOT_SEPARATE
    struct aic_ap_cfg *user_ap_cfg = &wifi_ap_cfg;
#else
    struct aic_ap_cfg *user_ap_cfg = NULL;
#endif

/**
 * @brief Apply WLAN MAC address filter rules to the AP configuration
 *
 * Clears the existing filter list and re-applies the MAC address filter
 * according to the filter mode in uap_cfg (blacklist/whitelist/disabled).
 *
 * @param uap_cfg AP configuration pointer; if NULL, only clear the filter list
 */
static void aicwf_app_wlan_mac_filter(uap_config_param *uap_cfg)
{
    unsigned char i;
    /* [BUGFIX] (common: APSTA concurrency) SoftAP runs on vif2 (non-zero), so use the actual AP vif idx;
     * otherwise the DENY_ACL/ACCEPT_ACL command goes to the STA's wpa_s (ap_iface=NULL) and causes a Data Abort */
    int ap_vif = aic_wifi_get_ap_vif_idx();
    if (ap_vif < 0)
        ap_vif = 0;

    {
        aic_wifi_clear_blacklist(ap_vif);
        aic_wifi_clear_whitelist(ap_vif);
    }

    if (uap_cfg == NULL)
        return;
    if (uap_cfg->filter.filter_mode == UAP_MAC_FILTER_MODE_BLACK_LIST)
    {
        for (i = 0; i < uap_cfg->filter.mac_count; i++)
        {
            aic_wifi_add_blacklist(ap_vif, (struct macaddr *)uap_cfg->filter.mac_list[i]);
        }
    }
    else if (uap_cfg->filter.filter_mode == UAP_MAC_FILTER_MODE_WHITE_LIST)
    {
        for (i = 0; i < uap_cfg->filter.mac_count; i++)
        {
            aic_wifi_add_whitelist(ap_vif, (struct macaddr *)uap_cfg->filter.mac_list[i]);
        }
        aic_ap_macaddr_acl(ap_vif, 1);
    }
    else
    {
        aic_ap_macaddr_acl(ap_vif, 0);
    }
}

/**
 * @brief Get the WLAN BSS running state
 * @return UAP_BSS_STOP=stopped, UAP_BSS_START=started, UAP_BSS_RESET=resetting
 */
unsigned char app_wlan_status(void)
{
    unsigned char ret = 0xff;
    switch (wifi_bss_st)
    {
    case 0:
        ret = UAP_BSS_STOP;
        break;
    case 1:
        ret = UAP_BSS_START;
        break;
    case 0xff:
        ret = UAP_BSS_RESET;
        break;
    }
    return ret;
}

#if UAP_CFG_NOT_SEPARATE
/**
 * @brief Start/stop the AP BSS configuration (non-separate mode, single AP configuration)
 * @param start_stop 1=start AP, 0=stop AP
 * @return >=0 on success, <0 on failure
 */
t_s32 app_wlan_bss_config(t_u32 start_stop)
{
    int ret = 0;
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;

    if (!GetWiFiInitStat())
    {
        DBG_APP_INF("%s: Wifi is not start\n", __func__);
        return -1;
    }

    if (1)
    {
        if (wifi_ap_cfg.sercurity_type == KEY_NONE)
        {
            wifi_ap_cfg.aic_ap_passwd.length = 0;
        }

        wifi_bss_st = 0xff;

        if (start_stop)
        {
            /* Mode exclusivity: rejected while in another mode; must stop back to UNKNOWN first */
            if (wifi_switch_mode(WIFI_MODE_AP) < 0)
            {
                DBG_APP_ERR("%s: need exit current mode first\n", __func__);
                return -1;
            }
            ret = aic_wifi_start_ap(&wifi_ap_cfg);
            if (ret >= 0)
            {
                aicwf_app_wlan_mac_filter(NULL);
                wifi_bss_st = 1;
                /* switch_mode may already have set APSTA/APP2P; do not overwrite the combined value */
                if (wifidev->wifi_mode != WIFI_MODE_APSTA &&
                        wifidev->wifi_mode != WIFI_MODE_APP2P)
                    wifidev->wifi_mode = WIFI_MODE_AP;
            }
            else
            {
                wifi_bss_st = 0;
                /* start failure rollback: restore the remaining role (STA/P2P concurrency); single mode back to UNKNOWN */
                if (wifidev->wifi_mode == WIFI_MODE_APSTA)
                    wifidev->wifi_mode = WIFI_MODE_STA;
                else if (wifidev->wifi_mode == WIFI_MODE_APP2P)
                    wifidev->wifi_mode = WIFI_MODE_P2P;
                else
                    wifidev->wifi_mode = WIFI_MODE_UNKNOWN;
                aic_wifi_mode_set((int)wifidev->wifi_mode);
            }
        }
        else
        {
            /* Idempotent: nothing to stop when not in an AP-related mode */
            if (wifidev->wifi_mode != WIFI_MODE_AP &&
                    wifidev->wifi_mode != WIFI_MODE_APSTA &&
                    wifidev->wifi_mode != WIFI_MODE_APP2P)
            {
                ret = 0;
            }
            else
            {
                ret = aic_wifi_stop_ap();
                if (ret == 0)
                {
                    /* stop succeeded, restore the remaining peer mode (concurrent exit of one side) */
                    if (wifidev->wifi_mode == WIFI_MODE_APSTA)
                        wifidev->wifi_mode = WIFI_MODE_STA;
                    else if (wifidev->wifi_mode == WIFI_MODE_APP2P)
                        wifidev->wifi_mode = WIFI_MODE_P2P;
                    else
                        wifidev->wifi_mode = WIFI_MODE_UNKNOWN;
                    aic_wifi_mode_set((int)wifidev->wifi_mode);
                }
                else
                {
                    DBG_APP_ERR("%s: stop ap fail %d\n", __func__, ret);
                }
            }
            wifi_bss_st = 0;
        }
    }

    DBG_APP_INF("%s: set=%d, mode=%d, ret=%d", __func__, start_stop, wifidev->wifi_mode, ret);
    return ret;
}
#else
/**
 * @brief Start/stop the AP BSS configuration (separate mode, supports 2.4G/5G dual band)
 *
 * Selects the 2.4G or 5G AP configuration according to wifi_ap_mode to start or stop the AP.
 *
 * @param start_stop 1=start AP, 0=stop AP
 * @return >=0 on success, <0 on failure
 */
t_s32 app_wlan_bss_config(t_u32 start_stop)
{
    int ret = 0;
    uap_config_param *uap_cfg = NULL;
    struct aic_ap_cfg *ap_cfg = NULL;
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;

    if (!GetWiFiInitStat())
    {
        DBG_APP_INF("%s: Wifi is not start\n", __func__);
        return -1;
    }

    if (1)
    {
        DBG_APP_INF("%s: mode=%d\n", __func__, wifidev->wifi_ap_mode);
        if (wifidev->wifi_ap_mode == WIFI_AP_MODE_2G4)
        {
            ap_cfg = &wifi_ap_cfg_2g4;
        }
        else if (wifidev->wifi_ap_mode == WIFI_AP_MODE_5G)
        {
            ap_cfg = &wifi_ap_cfg_5g;
        }

        user_ap_cfg = ap_cfg;

        if (ap_cfg->sercurity_type == KEY_NONE)
        {
            ap_cfg->aic_ap_passwd.length = 0;
        }

        wifi_bss_st = 0xff;

        if (start_stop)
        {
            /* Mode exclusivity: rejected while in another mode; must stop back to UNKNOWN first */
            if (wifi_switch_mode(WIFI_MODE_AP) < 0)
            {
                DBG_APP_ERR("%s: need exit current mode first\n", __func__);
                return -1;
            }
            ret = aic_wifi_start_ap(ap_cfg);
            if (ret >= 0)
            {
                aicwf_app_wlan_mac_filter(uap_cfg);
                wifi_bss_st = 1;
                /* switch_mode may already have set APSTA/APP2P; do not overwrite the combined value */
                if (wifidev->wifi_mode != WIFI_MODE_APSTA &&
                        wifidev->wifi_mode != WIFI_MODE_APP2P)
                    wifidev->wifi_mode = WIFI_MODE_AP;
            }
            else
            {
                wifi_bss_st = 0;
                /* start failure rollback: restore the remaining role (STA/P2P concurrency); single mode back to UNKNOWN */
                if (wifidev->wifi_mode == WIFI_MODE_APSTA)
                    wifidev->wifi_mode = WIFI_MODE_STA;
                else if (wifidev->wifi_mode == WIFI_MODE_APP2P)
                    wifidev->wifi_mode = WIFI_MODE_P2P;
                else
                    wifidev->wifi_mode = WIFI_MODE_UNKNOWN;
                aic_wifi_mode_set((int)wifidev->wifi_mode);
            }
        }
        else
        {
            /* Idempotent: nothing to stop when not in an AP-related mode */
            if (wifidev->wifi_mode != WIFI_MODE_AP &&
                    wifidev->wifi_mode != WIFI_MODE_APSTA &&
                    wifidev->wifi_mode != WIFI_MODE_APP2P)
            {
                ret = 0;
            }
            else
            {
                ret = aic_wifi_stop_ap();
                if (ret == 0)
                {
                    /* stop succeeded, restore the remaining peer mode (concurrent exit of one side) */
                    if (wifidev->wifi_mode == WIFI_MODE_APSTA)
                        wifidev->wifi_mode = WIFI_MODE_STA;
                    else if (wifidev->wifi_mode == WIFI_MODE_APP2P)
                        wifidev->wifi_mode = WIFI_MODE_P2P;
                    else
                        wifidev->wifi_mode = WIFI_MODE_UNKNOWN;
                    aic_wifi_mode_set((int)wifidev->wifi_mode);
                }
                else
                {
                    DBG_APP_ERR("%s: stop ap fail %d\n", __func__, ret);
                }
            }
            wifi_bss_st = 0;
        }
    }
    DBG_APP_INF("%s: set=%d, mode=%d, ret=%d", __func__, start_stop, wifidev->wifi_mode, ret);
    return ret;
}
#endif

/**
 * @brief Get the list of associated STAs
 * @param list Output pointer to the STA list structure
 * @return UAP_SUCCESS on success, -1=BSS not started
 */
t_s32 app_wlan_sta_list(sta_list *list)
{
    DBG_APP_INF("%s st %d ", __func__, wifi_bss_st);

    if (wifi_bss_st != 1)
        return -1;

    uint8_t index, cnt = aic_ap_get_associated_sta_cnt();
    void *sta_list = aic_ap_get_associated_sta_list();
    struct co_list_hdr *list_hdr = co_list_pick(sta_list);
    index = 0;

    list->sta_count = cnt;

    while (list_hdr != NULL)
    {
        struct aic_sta_info *sta = (struct aic_sta_info *)list_hdr;
        list_hdr = co_list_next(list_hdr);
        list->info[index].rssi = aic_ap_get_associated_sta_rssi((uint8_t *)sta->mac_addr.array);
        list->info[index].power_mfg_status = 0;
        memcpy(list->info[index].mac_address, sta->mac_addr.array, 6);
        DBG_APP_VRB("STA[%d] = %x:%x:%02x rssi:%d\n", index, sta->mac_addr.array[0], sta->mac_addr.array[1], sta->mac_addr.array[2], list->info[index].rssi);

        index ++;
    }

    return UAP_SUCCESS;
}

/**
 * @brief De-authenticate the specified STA
 * @param mac MAC address of the STA to de-authenticate
 * @return UAP_SUCCESS on success, UAP_FAILURE_INVALID_PARAM=invalid parameter
 */
int32_t app_wlan_sta_deauth(uap_802_11_mac_addr *mac)
{
    if (!mac)
    {
        DBG_APP_INF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
        return UAP_FAILURE_INVALID_PARAM;
    }
    return UAP_SUCCESS;
}

/**
 * @brief AP internal packet forwarding control (get/set)
 *
 * Controls the AP internal unicast and broadcast packet forwarding isolation policy.
 *
 * @param param Packet forwarding control parameters (action=UAP_ACTION_GET to get, UAP_ACTION_SET to set)
 * @return UAP_SUCCESS
 */
static int32_t aicwf_app_wlan_pkt_fwd_ctl(uap_pkt_fwd_ctl *param)
{
    if (param->action == UAP_ACTION_GET)
    {
        param->pkt_fwd_ctl = 0;
        if (wifi_ap_isolate & UAP_PKT_FWD_INTRA_BSS_UCAST)
        {
            param->pkt_fwd_ctl |= UAP_PKT_FWD_INTRA_BSS_UCAST;
        }
        if (wifi_ap_isolate & UAP_PKT_FWD_INTRA_BSS_BCAST)
        {
            param->pkt_fwd_ctl |= UAP_PKT_FWD_INTRA_BSS_BCAST;
        }
    }
    else if (param->action == UAP_ACTION_SET)
    {
        wifi_ap_isolate = 0;
        if (param->pkt_fwd_ctl & UAP_PKT_FWD_INTRA_BSS_UCAST)
        {
            wifi_ap_isolate |= UAP_PKT_FWD_INTRA_BSS_UCAST;
        }
        if (param->pkt_fwd_ctl & UAP_PKT_FWD_INTRA_BSS_BCAST)
        {
            wifi_ap_isolate |= UAP_PKT_FWD_INTRA_BSS_BCAST;
        }
    }
    DBG_APP_VRB("%s:0x%x\r\n", __func__, wifi_ap_isolate);
    return UAP_SUCCESS;
}

/**
 * @brief Configure the WLAN scan channels
 *
 * Sets the scan channel list according to the current WiFi mode (AP/STA).
 * In STA mode, full 2.4GHz channel scanning (1-13) is configured automatically.
 *
 * @param param Scan channel configuration parameters
 * @return UAP_SUCCESS on success, UAP_FAILURE=WiFi not started
 */
t_s32 app_wlan_scan_channels_config(uap_config_scan_chan *param)
{
    DBG_APP_INF("%s\n", __func__);
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;
    uap_config_scan_chan *scan_chan = NULL;

    if (!GetWiFiInitStat())
    {
        DBG_APP_INF("%s: Wifi is not start\n", __func__);
        return UAP_FAILURE;
    }

    if ((aicwf_get_wifi_fw_type() & 0x2) == 0x2)
    {
        DBG_APP_INF("fw is rftest mode, return\n");
        return 0;
    }

    if (wifidev->wifi_mode == WIFI_MODE_AP)
    {
        if (param->band)
            param->band -= 1;
        aic_channel_set((aic_config_scan_chan *)param);
    }
    else if (wifidev->wifi_mode == WIFI_MODE_STA)
    {
        scan_chan = (uap_config_scan_chan *)aicwf_rtos_malloc(sizeof(uap_config_scan_chan));
        memset(scan_chan, 0, sizeof(uap_config_scan_chan));
        scan_chan->action = ACTION_SET;
        scan_chan->band = UAP_RF_24_G - 1;
        scan_chan->total_chan = 13;
        scan_chan->chan_num[0] = 1;
        scan_chan->chan_num[1] = 2;
        scan_chan->chan_num[2] = 3;
        scan_chan->chan_num[3] = 4;
        scan_chan->chan_num[4] = 5;
        scan_chan->chan_num[5] = 6;
        scan_chan->chan_num[6] = 7;
        scan_chan->chan_num[7] = 8;
        scan_chan->chan_num[8] = 9;
        scan_chan->chan_num[9] = 10;
        scan_chan->chan_num[10] = 11;
        scan_chan->chan_num[11] = 12;
        scan_chan->chan_num[12] = 13;
        aic_channel_set((aic_config_scan_chan *)scan_chan);
        aicwf_rtos_free(scan_chan);
    }
    else
    {
        DBG_APP_INF("%s: wifi_mode = %d\n", __func__, wifidev->wifi_mode);
    }

    return UAP_SUCCESS;
}

/**
 * @brief Register the STA connection notification callback
 * @param notify_cb Callback function pointer
 * @return UAP_SUCCESS
 */
int32_t app_wlan_sta_conn_notify_reg(void (*notify_cb)(uint8_t, uint8_t *))
{
    return UAP_SUCCESS;
}

/**
 * @brief Get the current working channel
 * @return Current channel number (currently always returns 0)
 */
int32_t app_wlan_get_current_channel(void)
{
    uint8_t channel = 0;
    return (int32_t)channel;
}

/**
 * @brief Get the available channel range for the country code
 * @param first Output first available channel number
 * @param last  Output last available channel number
 */
void app_wlan_get_channel_by_country(int *first, int *last)
{
    char *ccode;
    int found = 1;

    *first = 1;
    ccode = NULL;

    if (ccode == NULL)
    {
        found = 0;
        goto cfound;
    }

    DBG_APP_INF("app_wlan_get_channel_by_country: country code is %s", ccode);

    if ((ccode[0] == 'J') && (ccode[1] == 'P'))
    {
        *last  = 14;
    }
    else if (((ccode[0] == 'U') && (ccode[1] == 'S')) ||
             ((ccode[0] == 'A') && (ccode[1] == 'S')) ||
             ((ccode[0] == 'C') && (ccode[1] == 'A')) ||
             ((ccode[0] == 'F') && (ccode[1] == 'M')) ||
             ((ccode[0] == 'G') && (ccode[1] == 'U')) ||
             ((ccode[0] == 'K') && (ccode[1] == 'Y')) ||
             ((ccode[0] == 'P') && (ccode[1] == 'R')) ||
             ((ccode[0] == 'T') && (ccode[1] == 'W')) ||
             ((ccode[0] == 'U') && (ccode[1] == 'M')) ||
             ((ccode[0] == 'V') && (ccode[1] == 'I')))
    {
        *last  = 11;
    }
    else if (((ccode[0] == 'I') && (ccode[1] == 'R')) ||
             ((ccode[0] == 'S') && (ccode[1] == 'Y')))
    {
        found = 0;
    }
    else
    {
        *last  = 13;
    }

cfound:
    if (found == 0)
    {
        *last = 11;
    }

    DBG_APP_INF("app_wlan_get_channel_by_country: first channel is %d, last channel is %d", *first, *last);
    return;
}

/**
 * @brief Control AP intra-BSS unicast isolation
 * @param enable 1=enable isolation, 0=disable isolation
 */
void app_wlan_intra_bss_unicast_deny(int enable)
{
    uap_pkt_fwd_ctl fwctl;
    fwctl.action = 0;
    aicwf_app_wlan_pkt_fwd_ctl(&fwctl);
    fwctl.action = 1;
    fwctl.pkt_fwd_ctl = 1;

    if (enable)
    {
        fwctl.pkt_fwd_ctl |= 0x4;
    }
    else
    {
        fwctl.pkt_fwd_ctl &= (~0x4);
    }

    aicwf_app_wlan_pkt_fwd_ctl(&fwctl);
}

/**
 * @brief Set MAC address filter rules only
 *
 * Updates the MAC filter list alone without modifying other AP parameters.
 *
 * @param filter  MAC filter configuration
 * @param uap_cfg AP configuration (optional); if non-NULL, the filter rules are also applied to it
 * @return 0=success, -1=BSS not started, cannot set
 */
int app_wlan_params_set_filter_only(void *filter, uap_config_param *uap_cfg)
{
    unsigned char i;
    uap_mac_filter *uap_filter = (uap_mac_filter *)filter;
    /* [BUGFIX] (common: APSTA concurrency) SoftAP runs on vif2 (non-zero), so use the actual AP vif idx */
    int ap_vif = aic_wifi_get_ap_vif_idx();
    if (ap_vif < 0)
        ap_vif = 0;

    if (wifi_bss_st != 1)
    {
        DBG_APP_INF("app_wlan_params_set_filter_only:can'sset[st:%d]", wifi_bss_st);
        return -1;
    }

    DBG_APP_INF("app_wlan_params_set_filter_only:set:%d\n", uap_filter->filter_mode);

    if (uap_cfg)
        memcpy(&(uap_cfg->filter), uap_filter, sizeof(uap_mac_filter));

    {
        aic_wifi_clear_blacklist(ap_vif);
        aic_wifi_clear_whitelist(ap_vif);
    }
    if (uap_filter->filter_mode == UAP_MAC_FILTER_MODE_BLACK_LIST)
    {
        for (i = 0; i < uap_filter->mac_count; i++)
        {
            aic_wifi_add_blacklist(ap_vif, (struct macaddr *)uap_filter->mac_list[i]);
        }
    }
    else if (uap_filter->filter_mode == UAP_MAC_FILTER_MODE_WHITE_LIST)
    {
        aic_ap_macaddr_acl(ap_vif, 1);
        for (i = 0; i < uap_filter->mac_count; i++)
        {
            aic_wifi_add_whitelist(ap_vif, (struct macaddr *)uap_filter->mac_list[i]);
        }
    }
    else
    {
        aic_ap_macaddr_acl(ap_vif, 0);
    }

    return 0;
}

/**
 * @brief Configure the WiFi MAC address
 *
 * Updates the chip MAC address only when the given MAC address is not all zeros.
 *
 * @param mac 6-byte MAC address
 */
void app_wlan_configure_wifimac(uint8_t *mac)
{
    DBG_APP_INF("%s  %x:%x:%x:%x:%x:%x", __func__, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (!(mac[0] == 0 && mac[1] == 0 && mac[2] == 0 &&
            mac[3] == 0 && mac[4] == 0 && mac[5] == 0))
        aic_set_mac_address(mac);
}

/**
 * @brief Get the number of associated STAs
 * @return STA count, -1=WiFi not started
 */
int app_wlan_get_sta_num(void)
{
    if (GetWifiStatus() != 1)
        return -1;

    uint8_t num = aic_ap_get_associated_sta_cnt();
    DBG_APP_INF("%s %d", __func__, num);
    return num;
}
