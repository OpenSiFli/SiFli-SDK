/** @file wifi_main.c
 *  @brief WiFi Main Source File
 *  Copyright 2020 NXP
 *  SPDX-License-Identifier: BSD-3-Clause
 */

#include "wlan_wifi.h"
#include "wifi.h"
#include "wm_net.h"
#include <osa.h>
#include "wifi_main.h"
#include "skw_type_msg.h"

#if CONFIG_WPA_SUPP
    #include "supp_api.h"
#endif

/*******************************************************************************
 * Definitions & Globals
 ******************************************************************************/
rt_timer_t scan_timer_handler = NULL;
static int wifi_start = WIFI_CLOSE;
struct wlan_scan_result scan_res[CONFIG_MAX_AP_ENTRIES];
struct wifi_scan_t scan_msg;
static void skw_wifi_set_start(int start);
/*******************************************************************************
 * Utility Functions
 ******************************************************************************/
static void printSeparator(void)
{
    PRINTF("========================================\r\n");
}

/*******************************************************************************
 * WiFi Event Callback
 ******************************************************************************/
int wlan_event_callback(enum wlan_event_reason reason, void *data)
{
    int ret;
    struct wlan_ip_config addr;
    char ssid[IEEEtypes_SSID_SIZE + 1] = {0};
    char ip[16];
    static int auth_fail = 0;

    switch (reason)
    {
    case WLAN_REASON_INITIALIZED:
        PRINTF("app_cb: WLAN initialized\r\n");
        break;
    case WLAN_REASON_INITIALIZATION_FAILED:
        PRINTF("app_cb: WLAN: initialization failed\r\n");
        break;
    case WLAN_REASON_AUTH_SUCCESS:
        PRINTF("app_cb: WLAN: authenticated to network\r\n");
        break;
    case WLAN_REASON_SUCCESS:
        PRINTF("app_cb: WLAN: connected to network\r\n");
        ret = wlan_get_address(&addr);
        if (ret != WM_SUCCESS)
        {
            PRINTF("failed to get IP address\r\n");
            return 0;
        }
        net_inet_ntoa(addr.ipv4.address, ip);
        ret = wlan_get_current_network_ssid(ssid);
        if (ret != WM_SUCCESS)
        {
            PRINTF("Failed to get External AP network\r\n");
            return 0;
        }
        PRINTF("Connected to following BSS:\r\n");
        PRINTF("SSID = [%s]\r\n", ssid);
        if (addr.ipv4.address != 0U)
        {
            PRINTF("IPv4 Address: [%s]\r\n", ip);
            skw_wifi_set_ipaddr(ip);
            skw_wifi_set_start(WIFI_CONNECTED);
        }
        auth_fail = 0;
        break;
    case WLAN_REASON_CONNECT_FAILED:
        PRINTF("app_cb: WLAN: connect failed\r\n");
        break;
    case WLAN_REASON_NETWORK_NOT_FOUND:
        PRINTF("app_cb: WLAN: network not found\r\n");
        break;
    case WLAN_REASON_NETWORK_AUTH_FAILED:
        PRINTF("app_cb: WLAN: network authentication failed\r\n");
        auth_fail++;
        if (auth_fail >= 3)
        {
            PRINTF("Authentication Failed. Disconnecting ... \r\n");
            wlan_disconnect();
            auth_fail = 0;
        }
        break;
    case WLAN_REASON_ADDRESS_SUCCESS:
        PRINTF("network mgr: DHCP new lease\r\n");
        break;
    case WLAN_REASON_ADDRESS_FAILED:
        PRINTF("app_cb: failed to obtain an IP address\r\n");
        break;
    case WLAN_REASON_USER_DISCONNECT:
        PRINTF("app_cb: disconnected\r\n");
        auth_fail = 0;
        break;
    case WLAN_REASON_LINK_LOST:
        PRINTF("app_cb: WLAN: link lost\r\n");
        break;
    case WLAN_REASON_CHAN_SWITCH:
        PRINTF("app_cb: WLAN: channel switch\r\n");
        break;

    case WLAN_REASON_PS_ENTER:
        break;
    case WLAN_REASON_PS_EXIT:
        break;
#if CONFIG_SUBSCRIBE_EVENT_SUPPORT
    case WLAN_REASON_RSSI_HIGH:
    case WLAN_REASON_SNR_LOW:
    case WLAN_REASON_SNR_HIGH:
    case WLAN_REASON_MAX_FAIL:
    case WLAN_REASON_BEACON_MISSED:
    case WLAN_REASON_DATA_RSSI_LOW:
    case WLAN_REASON_DATA_RSSI_HIGH:
    case WLAN_REASON_DATA_SNR_LOW:
    case WLAN_REASON_DATA_SNR_HIGH:
    case WLAN_REASON_LINK_QUALITY:
    case WLAN_REASON_PRE_BEACON_LOST:
        break;
#endif
#if CONFIG_WIFI_IND_DNLD
    case WLAN_REASON_FW_HANG:
    case WLAN_REASON_FW_RESET:
        break;
#endif
    default:
        PRINTF("app_cb: WLAN: Unknown Event: %d\r\n", reason);
    }
    return 0;
}

/*******************************************************************************
 * WiFi Driver Control Functions
 ******************************************************************************/
int wlan_driver_init(void)
{
    int result = wlan_init(NULL, 0);
    RT_ASSERT(0 == result);
    result = wlan_start(wlan_event_callback);
    RT_ASSERT(0 == result);
    skw_wifi_set_start(WIFI_OPEN);
    return result;
}
MSH_CMD_EXPORT(wlan_driver_init, wlan_driver_init);

int wlan_driver_deinit(void)
{
    int result = wlan_stop();
    RT_ASSERT(0 == result);
    wlan_deinit(0);
    return result;
}

int wlan_driver_reset(void)
{
    int result = wlan_driver_deinit();
    RT_ASSERT(0 == result);
    result = wlan_driver_init();
    RT_ASSERT(0 == result);
    return result;
}
MSH_CMD_EXPORT(wlan_driver_reset, wlan_driver_reset);

/*******************************************************************************
 * WiFi Scan & Connect
 ******************************************************************************/
static int __scan_cb(unsigned int count)
{
    struct wlan_scan_result res;
    unsigned int i;
    int err;

    if (count == 0U)
    {
        PRINTF("no networks found\r\n");
        return 0;
    }

    PRINTF("%d network%s found:\r\n", count, count == 1U ? "" : "s");
    for (i = 0; i < count; i++)
    {
        err = wlan_get_scan_result(i, &res);
        if (err != 0)
        {
            PRINTF("Error: can't get scan res %d\r\n", i);
            continue;
        }
        print_mac(res.bssid);
        if (res.ssid[0] != '\0')
            PRINTF(" \"%s\"\r\n", res.ssid);
        else
        {
            //(void)PRINTF(" (hidden) %s\r\n", print_role(res.role));
        }
        (void)PRINTF("\tmode: ");
#if CONFIG_11AC
#if CONFIG_11AX
        if (res.dot11ax != 0U)
        {
            (void)PRINTF("802.11AX ");
        }
        else
#endif
            if (res.dot11ac != 0U)
            {
                (void)PRINTF("802.11AC ");
            }
            else
#endif
                if (res.dot11n != 0U)
                {
                    (void)PRINTF("802.11N ");
                }
                else
                {
                    (void)PRINTF("802.11BG ");
                }
        (void)PRINTF("\r\n");

        (void)PRINTF("\tchannel: %d\r\n", res.channel);
        (void)PRINTF("\trssi: -%d dBm\r\n", res.rssi);
        (void)PRINTF("\tsecurity: ");
        if (res.wep != 0U)
        {
            (void)PRINTF("WEP ");
        }
        if ((res.wpa != 0U) && (res.wpa2 != 0U))
        {
            (void)PRINTF("WPA/WPA2 Mixed ");
        }
        else if ((res.wpa2 != 0U) && (res.wpa3_sae != 0U))
        {
            (void)PRINTF("WPA2/WPA3 SAE Mixed ");
        }
        else
        {
            if (res.wpa != 0U)
            {
                (void)PRINTF("WPA ");
            }
            if (res.wpa2 != 0U)
            {
                (void)PRINTF("WPA2 ");
            }
            if (res.wpa2_sha256 != 0U)
            {
                (void)PRINTF("WPA2-SHA256");
            }
            if (res.wpa3_sae != 0U)
            {
                (void)PRINTF("WPA3-SAE ");
            }
#if CONFIG_DRIVER_OWE
            if (res.owe != 0U)
            {
                (void)PRINTF("OWE Only");
            }
#endif
            if (res.wpa2_entp != 0U)
            {
                (void)PRINTF("WPA2 Enterprise ");
            }
            if (res.wpa2_entp_sha256 != 0U)
            {
                (void)PRINTF("WPA2-SHA256 Enterprise ");
            }
            if (res.wpa3_1x_sha256 != 0U)
            {
                (void)PRINTF("WPA3-SHA256 Enterprise ");
            }
            if (res.wpa3_1x_sha384 != 0U)
            {
                (void)PRINTF("WPA3-SHA384 Enterprise ");
            }
        }
#if (CONFIG_11R)
        if (res.ft_1x != 0U)
        {
            (void)PRINTF("with FT_802.1x");
        }
        if (res.ft_psk != 0U)
        {
            (void)PRINTF("with FT_PSK");
        }
        if (res.ft_sae != 0U)
        {
            (void)PRINTF("with FT_SAE");
        }
        if (res.ft_1x_sha384 != 0U)
        {
            (void)PRINTF("with FT_802.1x SHA384");
        }
#endif
        if (!((res.wep != 0U) || (res.wpa != 0U) || (res.wpa2 != 0U) || (res.wpa3_sae != 0U) || (res.wpa2_entp != 0U) ||
                (res.wpa2_sha256 != 0U) ||
#if CONFIG_DRIVER_OWE
                (res.owe != 0U) ||
#endif
                (res.wpa2_entp_sha256 != 0U) || (res.wpa3_1x_sha256 != 0U) || (res.wpa3_1x_sha384 != 0U)))
        {
            (void)PRINTF("OPEN ");
        }
        (void)PRINTF("\r\n");

        (void)PRINTF("\tWMM: %s\r\n", (res.wmm != 0U) ? "YES" : "NO");

#if CONFIG_11K
        if (res.neighbor_report_supported == true)
        {
            (void)PRINTF("\t802.11K: YES\r\n");
        }
#endif
#if CONFIG_11V
        if (res.bss_transition_supported == true)
        {
            (void)PRINTF("\t802.11V: YES\r\n");
        }
#endif
        if ((res.ap_mfpc == true) && (res.ap_mfpr == true))
        {
            (void)PRINTF("\t802.11W: Capable, Required\r\n");
        }
        if ((res.ap_mfpc == true) && (res.ap_mfpr == false))
        {
            (void)PRINTF("\t802.11W: Capable\r\n");
        }
        if ((res.ap_mfpc == false) && (res.ap_mfpr == false))
        {
            (void)PRINTF("\t802.11W: NA\r\n");
        }
#if CONFIG_WPA_SUPP_WPS
        if (res.wps)
        {
            if (res.wps_session == WPS_SESSION_PBC)
                (void)PRINTF("\tWPS: %s, Session: %s\r\n", "YES", "Push Button");
            else if (res.wps_session == WPS_SESSION_PIN)
                (void)PRINTF("\tWPS: %s, Session: %s\r\n", "YES", "PIN");
            else
                (void)PRINTF("\tWPS: %s, Session: %s\r\n", "YES", "Not active");
        }
        else
            (void)PRINTF("\tWPS: %s \r\n", "NO");
#endif
#if CONFIG_DRIVER_OWE
        if (res.trans_ssid_len != 0U)
        {
            (void)PRINTF("\tOWE BSSID: ");
            print_mac(res.trans_bssid);
            (void)PRINTF("\r\n\tOWE SSID:");
            if (res.trans_ssid_len != 0U)
            {
                (void)PRINTF(" \"%s\"\r\n", res.trans_ssid);
            }
        }
#endif
    }
    return 0;
}

static void skw_wlan_scan(void)
{
    if (wlan_scan(__scan_cb) != 0)
        PRINTF("Error: scan request failed\r\n");
    else
    {
        PRINTF("Scan scheduled...\r\n");
        if (scan_timer_handler) rt_timer_start(scan_timer_handler);
        skw_wifi_set_start(WIFI_SCANING);
    }
}
static int test_wlan_scan(int argc, char **argv)
{
    skw_wlan_scan();
    return 0;
}
MSH_CMD_EXPORT(test_wlan_scan, test_wlan_scan);

static int skw_wlan_connect(char *ssid, char *password, char *key_mode)
{
    int ret = RT_EOK;
    struct wlan_network sta_network;
    memset(&sta_network, 0, sizeof(struct wlan_network));
    memcpy(sta_network.name, ssid, strlen(ssid));
    memcpy(sta_network.ssid, ssid, strlen(ssid));
    sta_network.ip.ipv4.addr_type = ADDR_TYPE_DHCP;
    sta_network.ssid_specific = 1;

    if (password)
    {
        if (rt_strcmp(key_mode, "wpa3") == 0)
        {
            sta_network.security.type = WLAN_SECURITY_WPA3_SAE;
            sta_network.security.password_len = strlen(password);
            strncpy(sta_network.security.password, password, strlen(password));
            sta_network.security.password[sta_network.security.password_len] = '\0';
        }
        else
        {
            sta_network.security.psk_len = strlen(password);
            sta_network.security.type = WLAN_SECURITY_WPA;
            strncpy(sta_network.security.psk, password, strlen(password));
            sta_network.security.psk[sta_network.security.psk_len] = '\0';
        }
        rt_kprintf("Connecting to %s with password %s key_mode=%s\r\n", sta_network.ssid, sta_network.security.psk, key_mode);
    }
    else
    {
        sta_network.security.psk_len = 0;
        sta_network.security.type = WLAN_SECURITY_NONE;
        PRINTF("No password provided, connecting to open network %s\r\n", sta_network.ssid);
    }

    ret = wlan_add_network(&sta_network);
    ret = wlan_connect(ssid);
    if (ret != WM_SUCCESS)
    {
        PRINTF("Error: connect request failed\r\n");
        return RT_ERROR;
    }
    skw_wifi_set_start(WIFI_CONNECTING);
    return ret;
}

static void test_wlan_connect(int argc, char **argv)
{
    skw_wlan_connect(argv[1], argv[2], argv[3]);
}
MSH_CMD_EXPORT(test_wlan_connect, test_wlan_connect);

/*******************************************************************************
 * WiFi State & Control
 ******************************************************************************/
static void skw_wifi_set_start(int start)
{
    wifi_start = start;
}
static int skw_wifi_get_start(void)
{
    return wifi_start;
}
static rt_err_t skw_wifi_open(void)
{
    return wlan_driver_init();
}
static rt_err_t skw_wifi_scan_start(void)
{
    skw_wlan_scan();
    return RT_EOK;
}
static rt_err_t skw_wifi_scan_stop(void)
{
    skw_wifi_stop_scan();
    return RT_EOK;
}
static void *skw_wifi_get_scan_result(void)
{
    return &scan_msg;
}
static rt_err_t skw_wifi_connect(char *ssid, char *password, char *key_mode)
{
    skw_wlan_connect(ssid, password, key_mode);
    return RT_EOK;
}
static rt_err_t skw_wifi_disconnect(void)
{
    skw_disconnected();
    skw_wifi_set_start(WIFI_DISCONNECT);
    return RT_EOK;
}
static int skw_wifi_get_state(void)
{
    return skw_wifi_get_start();
}
static rt_err_t skw_wifi_set_mac(char *mac)
{
    wifi_setup_mac_address(mac);
    return RT_EOK;
}
static rt_err_t skw_wifi_control(rt_device_t dev, int cmd, void *args)
{
    if (args)
    {
        struct wifi_device_msg *msg = (struct wifi_device_msg *)args;
        rt_kprintf("%s %d flag=%d\n", __func__, __LINE__, msg->cmd_flag);
        switch (msg->cmd_flag)
        {
        case WIFI_CMD_OPEN:
            skw_wifi_open();
            break;
        case WIFI_CMD_SCAN_STATR:
            skw_wifi_scan_start();
            break;
        case WIFI_CMD_SCAN_STOP:
            skw_wifi_scan_stop();
            break;
        case WIFI_CMD_CONNECT:
            skw_wifi_connect(msg->ssid, msg->passwd, msg->key_mode);
            break;
        case WIFI_CMD_DISCONNECT:
            skw_wifi_disconnect();
            break;
        case WIFI_CMD_GET_STATE:
            skw_wifi_get_state();
            break;
        case WIFI_CMD_GET_SCAN_RESUIT:
            skw_wifi_get_scan_result();
            break;
        case WIFI_CMD_SET_MAC:
            skw_wifi_set_mac(msg->mac);
            break;
        default:
            break;
        }
    }
    switch (cmd)
    {
    case RT_DEVICE_CTRL_RESUME:
        break;
    case RT_DEVICE_CTRL_SUSPEND:
        break;
    default:
        break;
    }
    return 0;
}

/*******************************************************************************
 * Device Registration & Scan Timer
 ******************************************************************************/
static struct rt_device wifi_device;
#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops wifi_ops =
{
    RT_NULL,
    skw_wifi_open,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    skw_wifi_control
};
#endif

static void skw_wifi_timerout(void *param)
{
    if (scan_timer_handler) rt_timer_stop(scan_timer_handler);
    skw_wifi_set_start(WIFI_SCANEND);
    unsigned int count = 0;
    if (wifi_get_scan_result_count(&count) != WM_SUCCESS)
        count = 0;
    if (count)
    {
        rt_kprintf("%s %d count=%d\n", __func__, __LINE__, count);
        for (int i = 0; i < count; i++)
            wlan_get_scan_result(count, &scan_res[i]);
        scan_msg.scant_num = count;
    }
}

int skw_wifi_device(void)
{
    rt_err_t rt_err = RT_EOK;
    wifi_device.type = RT_Device_Class_Block;
#ifdef RT_USING_DEVICE_OPS
    wifi_device.ops = &wifi_ops;
#else
    wifi_device.control = skw_wifi_control;
#endif
    wifi_device.user_data = &wifi_device;
    rt_err = rt_device_register(&wifi_device, "wifi_swt6621",
                                RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);
    RT_ASSERT(RT_EOK == rt_err);
    if (!scan_timer_handler)
    {
        scan_timer_handler = rt_timer_create("scan_timerout", skw_wifi_timerout, 0,
                                             rt_tick_from_millisecond(2000), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
        RT_ASSERT(scan_timer_handler);
    }
    return 0;
}
INIT_PRE_APP_EXPORT(skw_wifi_device);