/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi P2P platform functions.
 */

#ifdef CONFIG_P2P_SUPPORT

#include "wifi_port_priv.h"
#include "wifi_api.h"
#include "aic_log.h"
#include <string.h>

#define AP_SSID_DFT  "BTD-AP"
#define AP_PASS_DFT  "12345678"

struct aic_p2p_cfg aic_wifi_p2p_cfg =
{
    .enable_acs = 1,
    .band = 0,
    .channel = 6,
    .type = WIFI_AP_MODE_2G4,
    .enable_acs = 1,
    .enable_he = 1,
    .aic_p2p_ssid = {
        .array = AP_SSID_DFT,
        .length = sizeof(AP_SSID_DFT) - 1,
    },
    .aic_ap_passwd = {
        .array = AP_PASS_DFT,
        .length = sizeof(AP_PASS_DFT) - 1,
    },
    .p2p_dev_name = AP_PASS_DFT,
};

/**
 * @brief Start the P2P (WiFi Direct) function
 *
 * Starts the P2P GO (Group Owner) after configuring the P2P device name, SSID and password.
 * Empty parameters keep the existing configuration unchanged.
 *
 * @param dev_name P2P device name (optional; NULL keeps the current value)
 * @param ssid     P2P group SSID (optional; NULL keeps the current value)
 * @param passwd   P2P group password (optional; NULL keeps the current value)
 * @return >=0 on success, <0 on failure
 */
int app_wlan_p2p_start(const char *dev_name, const char *ssid, const char *passwd)
{
    int ret;

    if (dev_name)
    {
        memset(aic_wifi_p2p_cfg.p2p_dev_name, 0, sizeof(aic_wifi_p2p_cfg.p2p_dev_name));
        strncpy((char *)aic_wifi_p2p_cfg.p2p_dev_name, dev_name, sizeof(aic_wifi_p2p_cfg.p2p_dev_name) - 1);
    }
    if (ssid)
    {
        memset(aic_wifi_p2p_cfg.aic_p2p_ssid.array, 0, sizeof(aic_wifi_p2p_cfg.aic_p2p_ssid.array));
        strncpy((char *)aic_wifi_p2p_cfg.aic_p2p_ssid.array, ssid, sizeof(aic_wifi_p2p_cfg.aic_p2p_ssid.array) - 1);
        aic_wifi_p2p_cfg.aic_p2p_ssid.length = strlen((char *)aic_wifi_p2p_cfg.aic_p2p_ssid.array);
    }
    if (passwd)
    {
        memset(aic_wifi_p2p_cfg.aic_ap_passwd.array, 0, sizeof(aic_wifi_p2p_cfg.aic_ap_passwd.array));
        strncpy((char *)aic_wifi_p2p_cfg.aic_ap_passwd.array, passwd, sizeof(aic_wifi_p2p_cfg.aic_ap_passwd.array) - 1);
        aic_wifi_p2p_cfg.aic_ap_passwd.length = strlen((char *)aic_wifi_p2p_cfg.aic_ap_passwd.array);
    }

    if (wifi_switch_mode(WIFI_MODE_P2P) < 0)
    {
        DBG_APP_ERR("%s: need exit current mode first\n", __func__);
        return -1;
    }

    ret = aic_wifi_start_p2p(&aic_wifi_p2p_cfg);

    if (ret >= 0)
    {
        /* switch_mode may already have set APP2P; do not overwrite the combined value */
        if (aicwf_wifi_dev.wifi_mode != WIFI_MODE_APP2P)
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_P2P;
    }
    else
    {
        /* start failure: first clean up the half-started state (when wlan_start_p2p succeeds but GROUP_ADD
         * fails, p2p_started=1 and wpa iface/netif/dhcp must be reclaimed via stop), then roll back the mode */
        aic_wifi_stop_p2p();
        if (aicwf_wifi_dev.wifi_mode == WIFI_MODE_APP2P)
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_AP;
        else
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_UNKNOWN;
        aic_wifi_mode_set((int)aicwf_wifi_dev.wifi_mode);
    }

    return ret;
}

/**
 * @brief Stop the P2P function
 * @return Return value of aic_wifi_stop_p2p()
 */
int app_wlan_p2p_stop(void)
{
    int ret = 0;

    /* Idempotent: nothing to stop when not in a P2P-related mode */
    if (aicwf_wifi_dev.wifi_mode != WIFI_MODE_P2P &&
            aicwf_wifi_dev.wifi_mode != WIFI_MODE_APP2P)
    {
        return 0;
    }

    ret = aic_wifi_stop_p2p();
    if (ret == 0)
    {
        /* stop succeeded, restore the remaining peer mode (P2P stopped in APP2P -> AP) */
        if (aicwf_wifi_dev.wifi_mode == WIFI_MODE_APP2P)
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_AP;
        else
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_UNKNOWN;
        aic_wifi_mode_set((int)aicwf_wifi_dev.wifi_mode);
    }

    return ret;
}

#endif /* CONFIG_P2P_SUPPORT */
