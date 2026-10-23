/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi STA mode platform functions.
 */

#include "wifi_port_priv.h"
#include "wifi_api.h"
#include "netif_api.h"
#include "aic_log.h"
#include <string.h>
#include "netif_port.h"
#define STA_DHCP_POLL_INTERVAL_MS 10
#define STA_DHCP_TIMEOUT_MS (10000/STA_DHCP_POLL_INTERVAL_MS)

/**
 * @brief Get the IP address obtained via DHCP in STA mode
 * @return 32-bit IP address (network byte order)
 */
uint32_t app_wlan_sta_get_ip(void)
{
    uint32_t ip = 0;
    aicwf_net_if_get_ip(NULL, &ip, NULL, NULL);
    DBG_APP_INF("%s:%d:%d:%d:%d", __func__,
                ((uint8_t *)&ip)[3], ((uint8_t *)&ip)[2], ((uint8_t *)&ip)[1], ((uint8_t *)&ip)[0]);

    return ip;
}

/**
 * @brief Authentication mode enum -> AT output string
 */
static const char *app_wlan_auth_mode_str(aic_wifi_auth_mode_e mode)
{
    switch (mode)
    {
    case AIC_WIFI_AUTH_NONE:
        return "NONE";
    case AIC_WIFI_AUTH_WEP:
        return "WEP";
    case AIC_WIFI_AUTH_WPA_PSK:
        return "WPA-PSK";
    case AIC_WIFI_AUTH_WPA2_PSK:
        return "WPA2-PSK";
    case AIC_WIFI_AUTH_WPA3_SAE:
        return "WPA3-SAE";
    case AIC_WIFI_AUTH_WPA2_WPA3:
        return "WPA2/WPA3";
    default:
        return "OTHER";
    }
}

/**
 * @brief Display the WiFi scan results
 *
 * Gets the scan results, sorts them by RSSI in descending order and outputs them via the AT interface.
 *
 * @return Number of APs found
 */
uint8_t app_wlan_sta_show_scan_res(void)
{
    wifi_ap_list_t aplist;
    uint16_t apcnt;
    char rsp[128] = {0};
    uint8_t index[MAX_AP_COUNT];
    int i, j;
    uint8_t rs1, rs2, tmp;

    aic_wifi_getscan(&aplist);

    if (aplist.ap_count == 0)
    {
        aicwf_ATRESP(g_athandle, 1, 0, "+NONE");
        return 0;
    }

    for (i = 0; i < aplist.ap_count; i++)
    {
        index[i] = i;
    }

    for (i = 0; i < aplist.ap_count - 1; i++)
    {
        for (j = 0; j < (aplist.ap_count - 1 - i); j++)
        {
            rs1 = aplist.ap_info[index[j]].rssi;
            rs2 = aplist.ap_info[index[j + 1]].rssi;

            if (rs1 < rs2)
            {
                tmp = index[j];
                index[j] = index[j + 1];
                index[j + 1] = tmp;
            }
        }
    }

    for (apcnt = 0; apcnt < aplist.ap_count; apcnt++)
    {
        memset(rsp, 0, 128);
        /* SSID last: SSID may contain spaces, keeping LEN/AUTH before it does not break parsing;
         * SSID_LEN=0 means a hidden AP (the SSID field is a placeholder) */
        sprintf(rsp, "[%d]:rssi=%3d dBm CH=%3d BSSID=%02x:%02x:%02x:%02x:%02x:%02x AUTH=%s SSID_LEN=%d SSID=%s",
                apcnt + 1, (int8_t)(aplist.ap_info[index[apcnt]].rssi), aic_wifi_phy_channel_to_freq(0, aplist.ap_info[index[apcnt]].channel),
                aplist.ap_info[index[apcnt]].bssid[0], aplist.ap_info[index[apcnt]].bssid[1],
                aplist.ap_info[index[apcnt]].bssid[2], aplist.ap_info[index[apcnt]].bssid[3],
                aplist.ap_info[index[apcnt]].bssid[4], aplist.ap_info[index[apcnt]].bssid[5],
                app_wlan_auth_mode_str((aic_wifi_auth_mode_e)aplist.ap_info[index[apcnt]].auth_mode),
                aplist.ap_info[index[apcnt]].ssid_len,
                (char *)aplist.ap_info[index[apcnt]].ssid);
        aicwf_ATRESP(g_athandle, 1, 0, rsp);
    }

    return aplist.ap_count;
}

/**
 * @brief STA connection state callback
 *
 * Prints the corresponding log message according to the connection state.
 *
 * @param st Current STA state (connected/disconnected/other)
 */
static void aicwf_app_wlan_sta_status_cb(aic_wifi_mac_status_e st)
{
    if (WIFI_STA_STATUS_DISCONNECTED == st)
    {
        /* Sync the tracked mode value on an FW-initiated disconnect, matching the reset semantics
         * of the successful sta stop path (APSTA->AP, STA->UNKNOWN). Otherwise the mode keeps a
         * zombie STA: on ap stop, "restore the remaining peer" would downgrade APSTA to STA
         * (link already down), and a later p2p start would be rejected since STA->P2P has no
         * allowed combination. Idempotent with an explicit sta stop */
        AIC_WIFI_MODE *mode = &aicwf_wifi_dev.wifi_mode;
        AIC_WIFI_MODE new_mode = *mode;

        if (*mode == WIFI_MODE_APSTA)
            new_mode = WIFI_MODE_AP;
        else if (*mode == WIFI_MODE_STA)
            new_mode = WIFI_MODE_UNKNOWN;

        if (new_mode != *mode)
        {
            DBG_APP_INF("AIC-STA: disconn, mode sync %d -> %d\n",
                        *mode, new_mode);
            *mode = new_mode;
            aic_wifi_mode_set((int)new_mode);
        }
        else
        {
            DBG_APP_INF("AIC-STA: disconn\n");
        }
    }
    else if (WIFI_STA_STATUS_CONNECTED == st)
    {
        DBG_APP_INF("AIC-STA: conn\n");
    }
    else
    {
        DBG_APP_INF("AIC-STA: other\n");
    }
}

/**
 * @brief Start the WiFi connection in STA mode
 *
 * Initiates a STA connection and starts DHCP to obtain an IP address after connecting.
 * Waits for DHCP completion by polling, with a timeout of about 10 seconds.
 * Returns success if DHCP completes before the timeout, otherwise returns failure.
 *
 * @param ssid       SSID of the AP
 * @param pw         Password of the AP
 * @param timeout_ms Connection timeout (milliseconds)
 * @return >=0 success (DHCP done), <0 failure
 */
int app_wlan_sta_start(uint8_t *ssid, uint8_t *pw, int timeout_ms)
{
    int ret  = 0;
    int dhcp_timeout = STA_DHCP_TIMEOUT_MS;

    SetWifiConfigFlag(1);
    aic_wifi_sta_status_cb_set(aicwf_app_wlan_sta_status_cb);

    /* Mode exclusivity: rejected while in another mode; must stop back to UNKNOWN first */
    if (wifi_switch_mode(WIFI_MODE_STA) < 0)
    {
        DBG_APP_ERR("%s: need exit current mode first\n", __func__);
        return -1;
    }

    ret = aic_wifi_start_sta(ssid, pw, timeout_ms);
    if (ret >= 0)
    {
        /* switch_mode may already have set APSTA; do not overwrite the combined value */
        if (aicwf_wifi_dev.wifi_mode != WIFI_MODE_APSTA)
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_STA;
        aicwf_netif_mode_switch(1);

        DBG_APP_INF("%s: start dhcp\n", __func__);
        ret = -10;
        while (dhcp_timeout--)
        {
            if (aicwf_net_if_dhcp_done(0))
            {
                ret = 0;
                app_wlan_sta_get_ip();
                break;
            }
            aicwf_rtos_msleep(STA_DHCP_POLL_INTERVAL_MS);
        }

        if (dhcp_timeout)
        {
            DBG_APP_INF("%s: dhcp finish\n", __func__);
        }
        else
        {
            DBG_APP_WRN("%s: dhcp timeout\n", __func__);
        }
    }
    else
    {
        DBG_APP_ERR("%s: start sta fail %d\n", __func__, ret);
    }

    if (ret < 0)
    {
        aic_wifi_stop_sta();
        /* start failure rollback: restore the peer role in concurrent scenarios (AP); single mode back to UNKNOWN,
         * avoiding an inconsistent state where the SoftAP stays alive but the mode remains APSTA */
        if (aicwf_wifi_dev.wifi_mode == WIFI_MODE_APSTA)
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_AP;
        else
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_UNKNOWN;
        aic_wifi_mode_set((int)aicwf_wifi_dev.wifi_mode);
    }

    return ret;
}

/**
 * @brief Stop the WiFi connection in STA mode
 * @return 0=success, non-zero=failure
 */
int app_wlan_sta_stop(void)
{
    int ret = 0;

    /* Idempotent: nothing to stop when not in a STA-related mode */
    if (aicwf_wifi_dev.wifi_mode != WIFI_MODE_STA &&
            aicwf_wifi_dev.wifi_mode != WIFI_MODE_APSTA)
    {
        return 0;
    }

    ret = aic_wifi_stop_sta();
    if (ret == 0)
    {
        /* stop succeeded, restore the remaining peer mode (STA stopped in APSTA -> AP) */
        if (aicwf_wifi_dev.wifi_mode == WIFI_MODE_APSTA)
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_AP;
        else
            aicwf_wifi_dev.wifi_mode = WIFI_MODE_UNKNOWN;
        aic_wifi_mode_set((int)aicwf_wifi_dev.wifi_mode);
    }
    else
    {
        DBG_APP_ERR("%s: fail, %d\n", __func__, ret);
    }

    return ret;
}

/**
 * @brief Start the WiFi scan
 * @return >=0 on success, -1=WiFi not initialized
 */
int app_wlan_scan_start(void)
{
    int ret = -1;
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;
    if (wifidev->g_wifi_init)
    {
        ret = aic_wifi_scan();
    }
    return ret;
}
