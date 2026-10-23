/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi core lifecycle, event handling, and platform configuration.
 */

#include "wifi_port_priv.h"
#include "wifi_api.h"
#include "cli_api.h"
#include "netif_api.h"
#include "sdio_api.h"
#include "sys_api.h"
#include "fmac_port.h"
#include "sdio_port.h"
#include "netif_port.h"
#include "aic_log.h"
#include <string.h>

/* ============================================================
 * Global state
 * ============================================================ */

int g_athandle = 0;

struct aic_wifi_dev aicwf_wifi_dev =
{
    .wifi_mode = WIFI_MODE_UNKNOWN,
};

int g_vendor_specific_apple = 0;
static wifi_event_handle m_scan_result_handle = NULL;
static wifi_event_handle m_scan_done_handle = NULL;
static wifi_event_handle m_join_success_handle = NULL;
static wifi_event_handle m_join_fail_handle = NULL;
static wifi_event_handle m_leave_handle = NULL;
wifi_priv_event_cb priv_aic_wifi_event_cb = NULL;

/* [AIC8800-PORT] Generic event extension callback (for the rt_wlan bridge layer etc. to consume all events).
 * Distinct from priv_aic_wifi_event_cb (signature (MAC_addr,mode), for MAC/mode notification):
 * this callback receives (AIC_WIFI_EVENT, aic_wifi_event_data). The original m_xxx_handle handlers are static
 * with no external registration entry and stay NULL; this hook is added by the platform side, NULL-guarded,
 * zero overhead when unregistered, and has no impact on route 1 (OFF). */
static aic_wifi_event_cb m_ext_event_cb = NULL;
static uint8_t wifi_auto_start = 0;

void aic_wifi_cmd(char *atcmd, unsigned int athandle, unsigned char chan_num);
void wifi_driver_action(unsigned int act);

/**
 * @brief Set the WiFi configuration flag
 * @param value Configuration state; 1 means configured
 */
void SetWifiConfigFlag(uint8_t value)
{
    DBG_APP_INF("%s %d\n", __func__, value);
    aicwf_wifi_dev.wifi_config_st = value;
}

/**
 * @brief Get the WiFi configuration flag
 * @return Current configuration state
 */
unsigned char GetWifiConfigFlag(void)
{
    return aicwf_wifi_dev.wifi_config_st;
}

/**
 * @brief Get the current WiFi working mode
 * @return WiFi mode, see the AIC_WIFI_MODE enumeration
 */
int aic_wifi_get_mode(void)
{
    return aicwf_wifi_dev.wifi_mode;
}

/**
 * @brief Get the WiFi BSS state
 *
 * Returns the WiFi state based on the configuration flag and the actual AP state:
 * - Configured and AP started: return 1
 * - Configured but AP not started: return 0
 * - Not configured: return 0xff
 *
 * @return 0=AP stopped, 1=AP started, 0xff=not configured
 */
uint8_t GetWifiStatus(void)
{
    if (aicwf_wifi_dev.wifi_config_st == 1)
    {
        if (app_wlan_status() == UAP_BSS_START)
            return 1;
        else
            return 0;
    }
    else
    {
        return 0xff;
    }
}

/**
 * @brief Get the WiFi MAC address (internal implementation)
 * @param wifi_mac Output buffer for the 6-byte MAC address
 * @return Always returns 0
 */
static int aicwf_get_wifi_mac(uint8_t *wifi_mac)
{
    uint8_t *mac;
    mac = aic_get_mac_address();
    memcpy(wifi_mac, mac, 6);
    DBG_APP_INF("%s :%x:%x:%x:%x:%x:%x\n", __func__,
                wifi_mac[0], wifi_mac[1], wifi_mac[2], wifi_mac[3], wifi_mac[4], wifi_mac[5]);
    return 0;
}

/**
 * @brief Get the WiFi MAC address
 * @param macbuf Output buffer for the 6-byte MAC address
 * @return Always returns 0
 */
int GetMACAddr(uint8_t *macbuf)
{
    DBG_APP_INF("%s", __func__);
    aicwf_get_wifi_mac(macbuf);
    return 0;
}

/**
 * @brief Case-insensitive string comparison
 * @param s1 First string
 * @param s2 Second string
 * @param n  Maximum number of characters to compare
 * @return 0=equal, positive=s1 greater than s2, negative=s1 less than s2
 */
int strncasecmp(const char *s1, const char *s2, size_t n)
{
    int c1, c2;
    while (n > 0)
    {
        c1 = *((unsigned char *)(s1++));
        if (c1 >= 'A' && c1 <= 'Z')
            c1 = c1 + ('a' - 'A');
        c2 = *((unsigned char *)(s2++));
        if (c2 >= 'A' && c2 <= 'Z')
            c2 = c2 + ('a' - 'A');
        if (c1 != c2)
        {
            return (c1 - c2);
        }
        if (c1 == '\0')
        {
            return 0;
        }
        --n;
    }
    return 0;
}

/**
 * @brief Get the WiFi initialization state
 * @return WiFi initialization state (set by the driver init task)
 */
uint8_t GetWiFiInitStat(void)
{
    DBG_APP_INF("%s->>%d\n", __func__, aicwf_wifi_dev.wifi_init_status);
    return aicwf_wifi_dev.wifi_init_status;
}

/**
 * @brief Get the WPS configuration method string
 * @return Configuration method string ("push_button keypad")
 */
const char *aic_get_wps_configmethod(void)
{
    const char *cm = "push_button keypad";
    return cm;
}

/**
 * @brief Get the WPS manufacturer name
 * @return Manufacturer name ("AIC")
 */
const char *aic_get_wps_Manufac(void)
{
    const char *cm = "AIC";
    return cm;
}

#if defined(CONFIG_AIC8800DW)
/**
 * @brief Get the WPS model name (DW series)
 * @return Model name ("AIC-8800-DW")
 */
const char *aic_get_wps_ModelName(void)
{
    const char *cm = "AIC-8800-DW";
    return cm;
}
#elif defined(CONFIG_AIC8800D80)
/**
 * @brief Get the WPS model name (D80 series)
 * @return Model name ("AIC-8800-D80")
 */
const char *aic_get_wps_ModelName(void)
{
    const char *cm = "AIC-8800-D80";
    return cm;
}
#elif defined(CONFIG_AIC8800D80N)
/**
 * @brief Get the WPS model name (D80N series)
 * @return Model name ("AIC-8800-D80N")
 */
const char *aic_get_wps_ModelName(void)
{
    const char *cm = "AIC-8800-D80N";
    return cm;
}
#endif

/**
 * @brief Get the WPS serial number
 * @return Serial number string ("12345")
 */
const char *aic_get_wps_SerialNum(void)
{
    const char *cm = "12345";
    return cm;
}

/**
 * @brief Get the WPS model number
 * @return Model number string ("123")
 */
const char *aic_get_wps_ModelNum(void)
{
    const char *cm = "123";
    return cm;
}

/**
 * @brief Get the WPS device name
 * @return Device name string ("E5576-821")
 */
const char *aic_get_wps_DeviceName(void)
{
    const char *cm = "E5576-821";
    return cm;
}

/**
 * @brief Get the WPA configuration extra flag
 * @return Configuration extra flag (0x0)
 */
unsigned char aic_get_wpas_config_extra(void)
{
    return 0x0;
}

/**
 * @brief Get the ACS (automatic channel selection) noise parameters
 * @param fix  Output fixed flag
 * @param n24g Output 2.4GHz noise value (dBm)
 * @param n5g  Output 5GHz noise value (dBm)
 */
void aic_get_acs_noise(unsigned char *fix, signed char *n24g, signed char *n5g)
{
    *fix = 1;
    *n24g = -89;
    *n5g = -92;
}

/**
 * @brief Get the ACS channel weight factors
 * @return Channel weight factor string, e.g. "1:0.98 6:0.98 11:0.98"
 */
const char *aic_get_acs_chfactor(void)
{
    const char *fc =  "1:0.98 6:0.98 11:0.98";
    return fc;
}

/**
 * @brief Get the ACS scan count
 * @return Scan count (5)
 */
unsigned char aic_get_acs_scan_num(void)
{
    return 5;
}

/**
 * @brief Get the power save level
 * @return Power save level (5)
 */
unsigned char aic_get_ps_level(void)
{
    return 5;
}

/**
 * @brief Get the saved MAMT configuration value
 * @return Configuration value (20)
 */
unsigned int aic_config_mamt_saved(void)
{
    return 20;
}

/**
 * @brief Set the WiFi working mode
 *
 * Determines the WiFi working mode from the firmware type and the requested mode.
 * RF test mode can be entered automatically via the firmware type flag or production mode.
 *
 * @param mode Target WiFi mode
 */
void wifi_set_default_mode(void)
{
    unsigned char fwtype = aicwf_get_wifi_fw_type();
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;

    if (fwtype & 0x02)
        wifidev->wifi_mode = WIFI_MODE_RFTEST;
    else if (aicwf_hal_IsInProduction_Mode() && (fwtype & 0x04))
        wifidev->wifi_mode = WIFI_MODE_RFTEST;

    if ((fwtype & 0x05) == 0x05)
    {
        wifidev->wifi_mode = WIFI_MODE_UNKNOWN;
    }

    /* No AT/factory fwtype: keep the combo fmac path (the mailbox HCI needs
       the WiFi firmware to be started). */
    if (wifidev->wifi_mode == WIFI_MODE_UNKNOWN)
        wifidev->wifi_mode = WIFI_MODE_STA;
}

/**
 * @brief Force the WiFi working mode (without checking the current state)
 *
 * Sets both levels of the mode: platform (wifi_mode) and driver (rwnx_hw->mode).
 * Typically used by external triggers such as AT commands, together with
 * wifi_driver_action(0x03) to restart the driver for the change to take effect.
 *
 * @param mode Target WiFi mode
 */
void wifi_set_mode(AIC_WIFI_MODE mode)
{
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;

    /* Sync the platform/driver two-level mode so the reset path can run deinit/init for the target mode */
    wifidev->wifi_mode = mode;
    aic_wifi_mode_set((int)mode);
}

/**
 * @brief Switch the WiFi working mode
 *
 * If the target mode differs from the current mode and the current mode is not
 * WIFI_MODE_UNKNOWN, returns -1, meaning the current mode must be exited first.
 * When the mode is allowed, switches to the target mode via aic_wifi_mode_set.
 *
 * @param mode Target WiFi mode
 * @return 0=success, -1=must exit the current mode first
 */
int wifi_switch_mode(AIC_WIFI_MODE mode)
{
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;
    AIC_WIFI_MODE cur = wifidev->wifi_mode;

    if (cur == mode || cur == WIFI_MODE_UNKNOWN)
    {
        /* Keep platform role in sync — netif TX/RX gates on aicwf_wifi_dev.wifi_mode */
        wifidev->wifi_mode = mode;
        aic_wifi_mode_set((int)mode);
        return 0;
    }

    /* STA + SoftAP concurrent: allow STA<->AP without tearing the other down */
    if ((cur == WIFI_MODE_STA && mode == WIFI_MODE_AP) ||
            (cur == WIFI_MODE_AP && mode == WIFI_MODE_STA) ||
            (cur == WIFI_MODE_APSTA &&
             (mode == WIFI_MODE_STA || mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA)))
    {
        DBG_APP_INF("%s: concurrent cur=%d + %d -> APSTA\n", __func__, cur, mode);
        wifidev->wifi_mode = WIFI_MODE_APSTA;
        aic_wifi_mode_set((int)WIFI_MODE_APSTA);
        return 0;
    }

#ifdef CONFIG_P2P_SUPPORT
    /* APP2P concurrency has a fixed start order: P2P first, then AP (matching STA/AP: STA first, then AP).
     * If AP starts first, the P2P GO can only follow the AP channel (including the one actually picked by ACS),
     * and a phone cannot discover the GO on a non-discovery 5G channel
     * (e.g. ch44); so reject starting P2P while AP is running,
     * the user must ap stop first and then p2p start */
    if ((cur == WIFI_MODE_P2P && mode == WIFI_MODE_AP) ||
            (cur == WIFI_MODE_APP2P &&
             (mode == WIFI_MODE_AP || mode == WIFI_MODE_P2P || mode == WIFI_MODE_APP2P)))
    {
        DBG_APP_INF("%s: concurrent cur=%d + %d -> APP2P\n", __func__, cur, mode);
        wifidev->wifi_mode = WIFI_MODE_APP2P;
        aic_wifi_mode_set((int)WIFI_MODE_APP2P);
        return 0;
    }
#endif

#ifdef CONFIG_P2P_SUPPORT
    if (cur == WIFI_MODE_AP && mode == WIFI_MODE_P2P)
    {
        DBG_APP_ERR("%s: APP2P order fixed (P2P first, AP second): "
                    "stop AP before starting P2P\n", __func__);
        return -1;
    }
#endif

    DBG_APP_INF("%s: cur=%d target=%d, need exit first\n",
                __func__, cur, mode);
    return -1;
}

/**
 * @brief Register the WiFi private event callback
 * @param cb Event callback function pointer
 */
void aic_wifi_priv_event_register(wifi_priv_event_cb cb)
{
    priv_aic_wifi_event_cb = cb;
}

/* [AIC8800-PORT] Register the generic event extension callback (see the m_ext_event_cb description).
 * The rt_wlan bridge layer (aic8800_wlan_dev.c) registers it in wlan_init to
 * capture asynchronous events such as SCAN_DONE / LEAVE / STA_DISCONNECT and convert them into rt_wlan events. */
void aic_wifi_ext_event_register(aic_wifi_event_cb cb)
{
    m_ext_event_cb = cb;
}

/**
 * @brief WiFi event callback dispatcher
 *
 * Dispatches events to the registered handlers according to the event type.
 * Supported events include: scan result, scan done, join success/failure,
 * leave, probe request, EAPOL done, assoc/disassoc indication, etc.
 *
 * @param enEvent Event type
 * @param enData  Event data
 */
void aic_wifi_event_callback(AIC_WIFI_EVENT enEvent, aic_wifi_event_data *enData)
{
    switch (enEvent)
    {
    case SCAN_RESULT_EVENT:
    {
        if (m_scan_result_handle)
        {
            m_scan_result_handle(enData);
        }
        break;
    }
    case SCAN_DONE_EVENT:
    {
        if (m_scan_done_handle)
        {
            m_scan_done_handle(enData);
        }
        break;
    }
    case JOIN_SUCCESS_EVENT:
    {
        if (m_join_success_handle)
        {
            m_join_success_handle(enData);
        }
        break;
    }
    case JOIN_FAIL_EVENT:
    {
        if (m_join_fail_handle)
        {
            m_join_fail_handle(enData);
        }
        break;
    }
    case LEAVE_RESULT_EVENT:
    {
        if (m_leave_handle)
        {
            m_leave_handle(enData);
        }
        break;
    }
    case PRO_DISC_REQ_EVENT:
    {
        uint32_t *mac_addr = (uint32_t *)enData->data.auth_deauth_data.reserved;
        DBG_APP_VRB("PRO_DISC_REQ_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                    , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    }
    case EAPOL_STA_FIN_EVENT:
    {
        uint32_t *mac_addr = (uint32_t *)enData->data.auth_deauth_data.reserved;
        DBG_APP_VRB("EAPOL_STA_FIN_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                    , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    }
    case EAPOL_P2P_FIN_EVENT:
    {
        uint32_t *mac_addr = (uint32_t *)enData->data.auth_deauth_data.reserved;
        DBG_APP_VRB("EAPOL_P2P_FIN_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                    , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    }
    case ASSOC_IND_EVENT:
    {
        uint32_t *mac_addr = (uint32_t *)enData->data.auth_deauth_data.reserved;
        DBG_APP_VRB("ASSOC_IND_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                    , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    }
    case STA_DISCONNECT_EVENT:
    {
        DBG_APP_INF("STA_DISCONNECT_EVENT\r\n");
        break;
    }
    case DISASSOC_STA_IND_EVENT:
    {
        uint32_t *mac_addr = (uint32_t *)enData->data.auth_deauth_data.reserved;
        DBG_APP_VRB("DISASSOC_STA_IND_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                    , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    }
    case DISASSOC_P2P_IND_EVENT:
    {
        uint32_t *mac_addr = (uint32_t *)enData->data.auth_deauth_data.reserved;
        DBG_APP_VRB("DISASSOC_P2P_IND_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                    , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    }
    default:
    {
        break;
    }
    }

    /* [AIC8800-PORT] Dispatch to the extension event callback (rt_wlan bridge layer); NULL-guarded, zero overhead when unregistered */
    if (m_ext_event_cb)
    {
        m_ext_event_cb(enEvent, enData);
    }
}

/**
 * @brief CLI command response callback, returns the result through the AT interface
 * @param rsp Command response string
 */
static void aicwf_CliCmdCb(char *rsp)
{
    aicwf_ATRESP(0, 1, 0, rsp);
}

/**
 * @brief Entry of the WiFi driver init task
 *
 * The task manages the WiFi driver lifecycle through a semaphore, supporting:
 * - AIC_DRIVER_REBOOT_PU/OPEN: power-up reset and WiFi initialization
 * - AIC_DIRVER_REBOOT_PD/CLOSE: power-down and WiFi de-initialization
 * - AIC_DRIVER_REBOOT_ADP/RESET: full reset flow in adapter test mode
 *
 * @param argv Task argument (unused)
 */
static void aicwf_wifi_init_task(void *argv)
{
    unsigned char fwtype = aicwf_get_wifi_fw_type();
    unsigned char need_start = 0;
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;

    wifi_set_default_mode();

    DBG_APP_INF("wifi_mode: %d\n", wifidev->wifi_mode);

    aic_cli_set_cb(aicwf_CliCmdCb);
    while (1)
    {
        aicwf_rtos_semaphore_wait(wifidev->wifi_task_sem, AIC_RTOS_WAIT_FOREVEVR);
        DBG_APP_INF("wifi_drive_op: %d\n", wifidev->wifi_drive_op);

        switch (wifidev->wifi_drive_op)
        {
        case AIC_DRIVER_REBOOT_PU:
        {
            aicwf_sdio_reset_wifi();
            if (aicwf_sdio_reset())
                break;
            need_start = wifi_auto_start;
        }
        case AIC_DRIVER_REBOOT_OPEN:
            aic_wifi_init(wifidev->wifi_mode, NULL);
            if ((need_start) && (wifidev->g_wifi_init))
            {
                wifidev->wifi_init_status = wifidev->g_wifi_init;
                wifi_bss_st = 0;
                if (wifidev->wifi_mode == WIFI_MODE_AP)
                {
                    app_wlan_bss_config(1);
                }
                need_start = 0;
            }
            else if (!wifidev->g_wifi_init)
            {
                aicwf_driver_unexcepted(0x08);
            }
            break;

        case AIC_DIRVER_REBOOT_PD:
            aic_sdio_host_predeinit();
            aicwf_sdio_pdn_wifi();
        case AIC_DRIVER_REBOOT_CLOSE:
            aic_wifi_deinit(wifidev->wifi_mode);
            /* deinit stops everything, so reset the mode to UNKNOWN */
            wifidev->wifi_mode = WIFI_MODE_UNKNOWN;
            need_start = 0;
            break;

        case AIC_DRIVER_REBOOT_ADP:
            aic_set_adap_test(1);
        case AIC_DRIVER_REBOOT_RESET:
        {
            /* Record the mode before reset: deinit uses it for cleanup, init uses it to pick the firmware (RFTEST distinction) */
            AIC_WIFI_MODE restore_mode = wifidev->wifi_mode;
            aic_sdio_host_predeinit();
            aicwf_sdio_pdn_wifi();
            aic_wifi_deinit(restore_mode);
            aicwf_sdio_reset_wifi();
            if (!aicwf_sdio_reset())
            {
                if (aic_wifi_init(restore_mode, NULL) >= 0)
                {
                    wifidev->wifi_init_status = wifidev->g_wifi_init;
                    wifi_bss_st = 0;
                    /* Restore AP only automatically (same mode is allowed by wifi_switch_mode);
                       STA restore only reconfigures the scan channels, the upper layer re-initiates the connection */
                    if (restore_mode == WIFI_MODE_AP)
                    {
                        app_wlan_bss_config(1);
                    }
                    else if (restore_mode == WIFI_MODE_STA)
                    {
                        app_wlan_scan_channels_config(NULL);
                    }
                }
                else if (!wifidev->g_wifi_init)
                {
                    aicwf_driver_unexcepted(0x08);
                }
            }
        }
        break;

        default:
            break;
        }

        wifidev->wifi_drive_op = 0;
        aic_set_adap_test(0);
        wifidev->wifi_init_status = wifidev->g_wifi_init;

        if (!wifidev->g_wifi_init)
            wifi_bss_st = 0xff;
    }
}

/**
 * @brief Initialize the WiFi driver
 *
 * Creates the WiFi init semaphore and the driver management task.
 * After the task is created, wifi_driver_action() triggers the start operation.
 */
void wifi_driver_init(void)
{
    int ret = 0;
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;

    DBG_APP_INF("%s", __func__);


    ret = aicwf_rtos_semaphore_create_only(&wifidev->wifi_task_sem, "wifi_init_sem", 1, 0);
    if (ret)
    {
        DBG_APP_ERR("wifi task sem create fail\n");
    }

    ret = aicwf_rtos_task_create_only(aicwf_wifi_init_task, "aicwf_wifi_init_task", 0,
                                      rwnx_wifi_stack_size, NULL, rwnx_wifi_priority, &wifidev->wifi_task_init_hdl);

    if (ret || (wifidev->wifi_task_init_hdl == NULL))
    {
        DBG_APP_ERR("wifi task init create fail\n");
    }

    wifi_driver_action(0x01);
    return;
}

/**
 * @brief Trigger a WiFi driver operation
 *
 * Signals the WiFi init task to trigger the specified driver operation.
 * Operation codes:
 * - 0x01: power-up reset and open
 * - 0x02: power-down and close
 * - 0x03: full reset
 * - other: custom operation
 *
 * @param act Driver operation code
 */
void wifi_driver_action(unsigned int act)
{
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;
    if (aicwf_rtos_semaphore_get_count(wifidev->wifi_task_sem) != 0)
    {
        DBG_APP_INF("r u creazy?");
        return;
    }

    wifidev->wifi_drive_op = act;
    aicwf_rtos_semaphore_signal(wifidev->wifi_task_sem, 0);
}
