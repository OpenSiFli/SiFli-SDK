/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rtthread.h>
#include "wifi_cli.h"
#include "wifi_if.h"
#include "osal_debug.h"
#include "osal_service.h"
#include <stdio.h>
#include <string.h>
#include "rwnx_msg_tx.h"
#include "aic_ota.h"
#include "lwip/netif.h"
#include "netdev.h"
#include "netal_service.h"

typedef struct _cmd_entry
{
    char *command;
    void (*function)(int, char **);
} cmd_entry;

extern struct aic_sdio_dev *g_sdio_dev;
#define CLI_RWNX_HW     g_sdio_dev->rwnx_hw

static void cmd_help(int argc, char **argv);

#define WIFI_NOT_ON_EXIT()  \
do { \
    if (g_sdio_dev == NULL) {   \
        DBG_APP_ERR("wifi is off\n");   \
        return; \
    }   \
} while(0)

static void cmd_exit(int argc, char **argv)
{
    DBG_APP_INF("delete wifi cli task\n");
    rtos_task_delete(NULL);
}

static void cmd_wifi_on(int argc, char **argv)
{
    wifi_open();

    DBG_APP_INF("Enter wifi on\n");
}

static void cmd_wifi_off(int argc, char **argv)
{
    WIFI_NOT_ON_EXIT();

    wifi_close();

    DBG_APP_INF("Exit wifi off\n");
}

static void cmd_wlan_status_get(int argc, char **argv)
{
    int ret;
    struct fhcustmsg_wlan_status_cfm cfm;

    WIFI_NOT_ON_EXIT();

    ret = rwnx_send_fhcustmsg_wlan_status_req(CLI_RWNX_HW, &cfm);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_wlan_status_get fail: %d\n", ret);
        return;
    }
    if (cfm.status == WLAN_DISCONNECT)
    {
        DBG_APP_INF("wlan_status: disconnect\n");
    }
    else if (cfm.status == WLAN_CONNECTED)
    {
        DBG_APP_INF("wlan_status: connected\n");
        DBG_APP_INF("ssid: \'%s\' (%3d dBm) akm=%d\n", cfm.ussid, cfm.rssi, cfm.akm);
        DBG_MACIF_INF("bssid: %02x:%02x:%02x:%02x:%02x:%02x\n", cfm.bssid[0], cfm.bssid[1],
                      cfm.bssid[2], cfm.bssid[3], cfm.bssid[4], cfm.bssid[5]);
        DBG_APP_INF("ip: %d.%d.%d.%d,  gw: %d.%d.%d.%d,  mk: %d.%d.%d.%d\n",
                    (cfm.ip >> 0) & 0xff, (cfm.ip >> 8) & 0xff, (cfm.ip >> 16) & 0xff, (cfm.ip >> 24) & 0xff,
                    (cfm.gw >> 0) & 0xff, (cfm.gw >> 8) & 0xff, (cfm.gw >> 16) & 0xff, (cfm.gw >> 24) & 0xff,
                    (cfm.mk >> 0) & 0xff, (cfm.mk >> 8) & 0xff, (cfm.mk >> 16) & 0xff, (cfm.mk >> 24) & 0xff);
        DBG_APP_INF("dns1: %d.%d.%d.%d,  dns2: %d.%d.%d.%d\n",
                    (cfm.dns1 >> 0) & 0xff, (cfm.dns1 >> 8) & 0xff, (cfm.dns1 >> 16) & 0xff, (cfm.dns1 >> 24) & 0xff,
                    (cfm.dns2 >> 0) & 0xff, (cfm.dns2 >> 8) & 0xff, (cfm.dns2 >> 16) & 0xff, (cfm.dns2 >> 24) & 0xff);
        DBG_APP_INF("freq: %d, width %d, format %d, rate %d\n", cfm.freq, cfm.width, cfm.format, cfm.max_rate);

#if 0
//#ifdef RT_USING_NETDEV
#ifdef RT_USING_WIFI
        const char *netdev_name = "w0";
#else
        const char *netdev_name = "e0";
#endif
        struct netdev *netdev = netdev_get_by_name(netdev_name);
        if (netdev && !(netdev->flags & NETDEV_FLAG_LINK_UP))
        {
#if defined(CONFIG_VNET_MODE)
            ret = netal_eth_protocol_linkchange(1);
            if (ret)
            {
                DBG_MACIF_ERR("eth linkchange err: %d\n", ret);
                return;
            }
            ret = netal_eth_protocol_addrinfoset(&cfm.ip, &cfm.mk, &cfm.gw);
            if (ret)
            {
                DBG_MACIF_ERR("eth addrinfoset err: %d\n", ret);
                return;
            }
#endif

            netdev_set_dns_server(netdev, 0, (ip_addr_t *)&cfm.dns1);
            netdev_set_dns_server(netdev, 1, (ip_addr_t *)&cfm.dns2);
        }
#endif
    }
    else if (cfm.status == WLAN_SCANNING)
    {
        DBG_APP_INF("wlan_status: scanning\n");
    }
    else if (cfm.status == WLAN_CONNECTING)
    {
        DBG_APP_INF("wlan_status: connecting\n");
        DBG_APP_INF("ssid: \'%s\' (%3d dBm) akm=%d\n", cfm.ussid, cfm.rssi, cfm.akm);

    }
    else
    {
        DBG_APP_ERR("wlan_status: invalid status\n");
    }
}

static void cmd_sta_connect(int argc, char **argv)
{
    int ret;
    char *ssid, *passwd = NULL;

    WIFI_NOT_ON_EXIT();

    if ((argc < 2) || (argc > 3))
    {
        DBG_APP_ERR("Usage: connect SSID <PASSWORD>\n");
        return;
    }
    ssid = argv[1];
    if (argc > 2)
    {
        passwd = argv[2];
    }
    DBG_APP_INF("connect ssid=%s\n", ssid);
    ret = rwnx_send_fhcustmsg_connect_req(CLI_RWNX_HW, ssid, passwd);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_connect_req fail: %d\n", ret);
        return;
    }
    DBG_APP_INF("connecting...\n");
}

static void cmd_wifi_scan(int argc, char **argv)
{
    int ret;

    WIFI_NOT_ON_EXIT();

    ret = rwnx_send_fhcustmsg_scan_req(CLI_RWNX_HW);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_scan_req fail: %d\n", ret);
#ifdef WK_WIFI_SUPPORT
        hl_if_data_t *rt_msg = service_fill_msg(WIFI_APP, WIFI_AP_LIST_IND, NULL, 0);
        rt_err_t err = send_msg_to_preprocess_thread(rt_msg, sizeof(rt_msg));
#endif
        return;
    }
    DBG_APP_INF("scanning...\n");
}

static void cmd_sta_disconnect(int argc, char **argv)
{
    int ret;

    WIFI_NOT_ON_EXIT();

    ret = rwnx_send_fhcustmsg_disconnect_req(CLI_RWNX_HW);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_disconnect_req fail: %d\n", ret);
        return;
    }
    DBG_APP_INF("disconnecting...\n");
}

static void cmd_enter_sleep(int argc, char **argv)
{
    int ret;
    uint8_t lvl = PM_LEVEL_LIGHT_SLEEP;
    struct fhcustmsg_sleep_cfm cfm;

    WIFI_NOT_ON_EXIT();

    ret = rwnx_send_fhcustmsg_enter_sleep_req(CLI_RWNX_HW, lvl, &cfm);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_enter_sleep_req fail: %d\n", ret);
        return;
    }
    if (cfm.status == 0)
    {
        DBG_APP_INF("Enter sleep success\n");
    }
    else
    {
        DBG_APP_INF("Enter sleep fail: %d\n", cfm.status);
    }
}

static void cmd_exit_sleep(int argc, char **argv)
{
    int ret;

    WIFI_NOT_ON_EXIT();

    ret = rwnx_send_fhcustmsg_exit_sleep_req(CLI_RWNX_HW);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_exit_sleep_req fail: %d\n", ret);
        return;
    }
    DBG_APP_INF("Exit sleep\n");
}

static void cmd_suspend(int argc, char **argv)
{
    int ret;
    uint8_t lvl = PM_LEVEL_DEEP_SLEEP;
    struct fhcustmsg_sleep_cfm cfm;

    WIFI_NOT_ON_EXIT();

    ret = rwnx_send_fhcustmsg_enter_sleep_req(CLI_RWNX_HW, lvl, &cfm);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_enter_sleep_req fail: %d\n", ret);
        return;
    }

    if (cfm.status == 0)
    {
        wifi_if_sdio_suspend();
        DBG_APP_INF("Enter suspend success\n");
    }
    else
    {
        DBG_APP_INF("Enter suspend fail: %d\n", cfm.status);
    }
}

static void cmd_resume(int argc, char **argv)
{
    int ret;

    wifi_if_sdio_resume();

    DBG_APP_INF("Enter resume\n");
}

#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"

static void cmd_sta_get_mac(int argc, char **argv)
{
    int ret;
    struct fhcustmsg_mac_addr_cfm cfm;

    WIFI_NOT_ON_EXIT();

    ret = rwnx_send_fhcustmsg_get_mac_addr_req(CLI_RWNX_HW, &cfm);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_get_mac_addr_req fail: %d\n", ret);
        return;
    }
    DBG_APP_INF("mac: "MAC_FMT"\n",
                cfm.mac_addr[0], cfm.mac_addr[1], cfm.mac_addr[2], cfm.mac_addr[3], cfm.mac_addr[4], cfm.mac_addr[5]);
}

static void cmd_sta_set_mac(int argc, char **argv)
{
    int ret;
    uint32_t mac_int[6];
    uint8_t mac_addr[6];

    WIFI_NOT_ON_EXIT();

    if (argc != 2)
    {
        DBG_APP_ERR("Usage: setmac xx:xx:xx:xx:xx:xx\n");
        return;
    }

    ret = sscanf(argv[1], MAC_FMT, &mac_int[0], &mac_int[1], &mac_int[2], \
                 &mac_int[3], &mac_int[4], &mac_int[5]);
    if (6 != ret)
    {
        DBG_APP_ERR("mac argument error\n");
        return;
    }
    for (int i = 0; i < 6; i++)
        mac_addr[i] = mac_int[i];

    ret = rwnx_send_fhcustmsg_set_mac_addr_req(CLI_RWNX_HW, mac_addr);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_set_mac_addr_req fail: %d\n", ret);
        return;
    }
    DBG_APP_INF("set mac "MAC_FMT" success\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
}


#include "ota_test_bin.h"

static void cmd_wifi_ota(int argc, char **argv)
{
    int ret;

    WIFI_NOT_ON_EXIT();

    ret = aic_host_ota(CLI_RWNX_HW, (uint8_t *)host_wb_bin, host_wb_bin_size, host_wb_bin_FW_VERSION);
    if (ret)
    {
        DBG_APP_ERR("host ota fail: %d\n", ret);
        return;
    }

    DBG_APP_INF("ota success, rebooting...\n");

    // OTA成功后直接重启系统，避免复杂的资源清理问题
    extern void rt_hw_cpu_reset(void);
    rt_hw_cpu_reset();
}

static void cmd_wifi_get_fw_version(int argc, char **argv)
{
    int ret;
    struct fhcustmsg_get_fw_version_cfm cfm;

    WIFI_NOT_ON_EXIT();

    ret = rwnx_send_fhcustmsg_get_fw_version_req(CLI_RWNX_HW, &cfm);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_disconnect_req fail: %d\n", ret);
        return;
    }
    DBG_APP_INF("fw version [%s]\n", cfm.version);
}

static void cmd_wifi_reset(int argc, char **argv)
{
    int ret;

    WIFI_NOT_ON_EXIT();

    ret = rwnx_send_fhcustmsg_reset_req(CLI_RWNX_HW);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_reset_req fail: %d\n", ret);
        return;
    }
    DBG_APP_INF("reseting...\n");
}

static void cmd_wifi_sta_cfg(int argc, char **argv)
{
    struct fhcustmsg_sta_cfg_cfm cfm;
    int op = STA_CFG_REQ_OP_GET_LIST;
    int enable = 0;
    char *ssid = NULL;
    int ret;

    WIFI_NOT_ON_EXIT();

    if (argc == 2 && !strcmp(argv[1], "clear"))
    {
        op = STA_CFG_REQ_OP_CLEAR;
    }

    if (argc == 3 && !strcmp(argv[1], "enable"))
    {
        op = STA_CFG_REQ_OP_SET;
        enable = 1;
        ssid = argv[2];
    }

    if (argc == 3 && !strcmp(argv[1], "disable"))
    {
        op = STA_CFG_REQ_OP_SET;
        enable = 0;
        ssid = argv[2];
    }

    if (argc == 3 && !strcmp(argv[1], "delete"))
    {
        op = STA_CFG_REQ_OP_DELETE;
        enable = 0;
        ssid = argv[2];
    }

    ret = rwnx_send_fhcustmsg_sta_cfg_req(CLI_RWNX_HW, op, ssid, enable, &cfm);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_sta_cfg_req fail: %d\n", ret);
        return;
    }

    DBG_APP_INF("fhcustmsg_sta_cfg_req result %d\n", cfm.status);

    if (cfm.status == 0)
    {
        if (op == STA_CFG_REQ_OP_GET_LIST)
        {
            for (int i = 0; i < cfm.count; i++)
                DBG_APP_INF("[%d] %s AKM=%d SSID=%s PWD=%s\n", i, cfm.sta_info[i].enable ? "enable" : "disable",
                            cfm.sta_info[i].akm, cfm.sta_info[i].ssid, cfm.sta_info[i].pw);
        }
    }
}

#if 0
static void cmd_http_request(int argc, char **argv)
{
    static char uri_get_wallpaper[256];
    int ret;

    WIFI_NOT_ON_EXIT();

    if (argc != 2)
    {
        DBG_APP_ERR("Usage: http_request pagenum\n");
        return;
    }
    rt_snprintf(uri_get_wallpaper, sizeof(uri_get_wallpaper),
                "http://test.iwhopro.com"
                "/howeartest/wifi/wallpaper/wallpaperMarket/selectWifiWallpaperPageByType?"
                "bluetoothName=%s&projectCode=3859"
                "&pageNum=%s&pageSize=1&styleId=5", "HK9%20Ultra%20Max", argv[1]);

    ret = rwnx_send_fhcustmsg_http_req(CLI_RWNX_HW, uri_get_wallpaper);
    if (ret)
    {
        DBG_APP_ERR("fhcustmsg_http_request_req fail: %d\n", ret);
        return;
    }
    DBG_APP_INF("http request...\n");
}
#endif

static const cmd_entry wifi_cli_cmd_list[] =
{
    {"wifi_on", cmd_wifi_on},
    {"wifi_off", cmd_wifi_off},
    {"status",  cmd_wlan_status_get},
    {"connect", cmd_sta_connect},
    {"scan", cmd_wifi_scan},
    {"disconnect", cmd_sta_disconnect},
    {"enter_sleep", cmd_enter_sleep},
    {"exit_sleep", cmd_exit_sleep},
    {"getmac", cmd_sta_get_mac},
    {"setmac", cmd_sta_set_mac},
    {"ota_test", cmd_wifi_ota},
    {"ver", cmd_wifi_get_fw_version},
    {"reset", cmd_wifi_reset},
    {"sta_cfg", cmd_wifi_sta_cfg},
//    {"http_request", cmd_http_request},
#if CONFIG_WLAN
    {"wifi_connect_bssid", cmd_wifi_connect_bssid},
    {"wifi_info", cmd_wifi_info},
    {"wifi_ap", cmd_wifi_ap},
    {"wifi_reoder_scan", cmd_wifi_reorder_scan},
#if SCAN_WITH_SSID
    {"wifi_scan_with_ssid", cmd_wifi_scan_with_ssid},
    {"wifi_scan_with_multissid", cmd_wifi_scan_with_multiple_ssid},
#endif
    //{"iwpriv", cmd_wifi_iwpriv},
    //{"wifi_promisc", cmd_promisc},
#if (CONFIG_INCLUDE_SIMPLE_CONFIG)
    {"wifi_simple_config", cmd_simple_config},
#endif
#if CONFIG_WPS
#if CONFIG_ENABLE_WPS
    {"wifi_wps", cmd_wps},
#endif
#if defined(CONFIG_ENABLE_WPS_AP) && CONFIG_ENABLE_WPS_AP
    {"wifi_ap_wps", cmd_ap_wps},
#endif
#if CONFIG_ENABLE_P2P
    {"wifi_p2p_start", cmd_wifi_p2p_start},
    {"wifi_p2p_stop", cmd_wifi_p2p_stop},
    {"wifi_p2p_auto_go", cmd_wifi_p2p_auto_go_start},
    {"p2p_listen", cmd_p2p_listen},
    {"p2p_find", cmd_p2p_find},
    {"p2p_peers", cmd_p2p_peers},
    {"p2p_info", cmd_p2p_info},
    {"p2p_disconnect", cmd_p2p_disconnect},
    {"p2p_connect", cmd_p2p_connect},
#endif
#endif
#if CONFIG_CONCURRENT_MODE
    {"wifi_sta_ap", cmd_wifi_sta_and_ap},
#endif

    {"wifi_debug", cmd_debug},
#endif
#if CONFIG_LWIP_LAYER
    //{"app", cmd_app},
#if CONFIG_BSD_TCP
    {"tcp", cmd_tcp},
    {"udp", cmd_udp},
#endif
    {"ping", cmd_ping},
#endif
    {"wifi_suspend", cmd_suspend},
    {"wifi_resume", cmd_resume},
#if 0  /*it does't work normal and is for specific needs*/ //CONFIG_CUSTOMER_EE_REQUEST
    {"wifi_stop_ap", cmd_stop_ap},
    {"wifi_resume_ap", cmd_resume_ap},
#endif
#if CONFIG_SET_PRIORITY
    {"set_pri", cmd_set_priority},
#endif
#if CONFIG_CUSTOM_IE
    {"add_ie", cmd_add_ie},
    {"update_ie", cmd_update_ie},
    {"del_ie", cmd_del_ie},
#endif
    {"exit", cmd_exit},
    {"help", cmd_help}
};

static void cmd_help(int argc, char **argv)
{
    int i;

    DBG_APP_INF("cmds:\n");

    for (i = 0; i < sizeof(wifi_cli_cmd_list) / sizeof(wifi_cli_cmd_list[0]); i++)
    {
        DBG_APP_INF("    %s", wifi_cli_cmd_list[i].command);
    }
    DBG_APP_INF("\n");
}

#define MAX_ARGC    10

static int parse_cmd(char *buf, char **argv)
{
    int argc = 0;

    rtos_memset(argv, 0, sizeof(argv)*MAX_ARGC);
    while ((argc < MAX_ARGC) && (*buf != '\0'))
    {
        argv[argc] = buf;
        argc ++;
        buf ++;

        while ((*buf != ' ') && (*buf != '\0'))
            buf ++;

        while (*buf == ' ')
        {
            *buf = '\0';
            buf ++;
        }
        // Don't replace space
//        if(argc == 1){
//            if(strcmp(argv[0], "iwpriv") == 0){
//                if(*buf != '\0'){
//                    argv[1] = buf;
//                    argc ++;
//                }
//                break;
//            }
//        }
    }

    return argc;
}

char cmd_rx_buf[128];
rtos_semaphore wifi_cli_sema;

void wifi_cli_task(void *param)
{
    int i, argc, ret;
    char *cli_argv[MAX_ARGC];
    char temp_uart_buf[128];
    rtos_semaphore_create(&wifi_cli_sema, "cli_sema", 1, 0);
    DBG_APP_INF("start wifi_cli task\r\n");

    while (1)
    {
        ret = rtos_semaphore_wait(wifi_cli_sema, -1);
        if (!ret)
        {
            rtos_memcpy(temp_uart_buf, cmd_rx_buf, 100);
            if ((argc = parse_cmd(temp_uart_buf, cli_argv)) > 0)
            {
                int found = 0;

                for (i = 0; i < sizeof(wifi_cli_cmd_list) / sizeof(wifi_cli_cmd_list[0]); i ++)
                {
                    if (strcmp((const char *)cli_argv[0], (const char *)(wifi_cli_cmd_list[i].command)) == 0)
                    {
                        wifi_cli_cmd_list[i].function(argc, cli_argv);
                        found = 1;
                        break;
                    }
                }
                if (!found)
                    DBG_APP_ERR("unknown command '%s'", cli_argv[0]);
            }

            DBG_APP_INF("\r\n\n# ");
            cmd_rx_buf[0] = '\0';
            temp_uart_buf[0] = '\0';

        }
        else
        {
            DBG_APP_ERR("wifi cli sema wait err %d", ret);
        }
    }
}

int wifi_cli_init(void)
{
    rtos_task_handle task_handle;
    int ret;
    ret = rtos_task_create(wifi_cli_task, "wifi_cli_task", APP_WIFI_CLI_TASK,
                           app_wifi_cli_stack_size, NULL, app_wifi_cli_priority,
                           &task_handle);
    if (ret)
    {
        DBG_APP_ERR("wifi cli task create fail,%d\n", ret);
    }
    return ret;
}

int wifi_cli_handler(int argc, char **argv)
{
    int len = 0, cur_len = 0;
    rtos_memset(cmd_rx_buf, 0, 100);
    for (uint8_t i = 1; i < argc; i ++)
    {
        cur_len = strlen(argv[i]);
        rtos_memcpy(&cmd_rx_buf[len], argv[i], cur_len);
        cmd_rx_buf[len + cur_len] = ' ';
        len += cur_len + 1;
    }
    ASSERT_ERR(len < sizeof(cmd_rx_buf));
    rtos_semaphore_signal(wifi_cli_sema, false);
    return 0;
}

int wifi_cli_handler2(char *cmd)
{
    int len = 0, cur_len = 0;
    rtos_memset(cmd_rx_buf, 0, 100);
    cur_len = strlen(cmd);
    rtos_memcpy(&cmd_rx_buf[len], cmd, cur_len);
    cmd_rx_buf[len + cur_len] = ' ';
    len += cur_len + 1;
    ASSERT_ERR(len < sizeof(cmd_rx_buf));
    rtos_semaphore_signal(wifi_cli_sema, false);
    return 0;
}
#include "wifi_device.h"
rt_err_t rt_wifi_open(rt_device_t dev, rt_uint16_t oflag)
{
    wifi_open();
    return RT_EOK;
}
rt_err_t rt_wifi_close(rt_device_t dev)
{

    return RT_EOK;
}

rt_err_t rt_wifi_control(rt_device_t dev, int cmd, void *args)
{
    int ret;
    switch (cmd)
    {
    case WIFI_CONTROL_CONNECT:
    {
        wifi_connect_t *con = (wifi_connect_t *)args;
        rt_kprintf("connect ssid=%s pwd=%s\n", con->ssid, con->pwd);
        rwnx_send_fhcustmsg_connect_req(CLI_RWNX_HW, con->ssid, con->pwd);
    }
    break;
    case WIFI_CONTROL_DISCONNECT:
    {
        rwnx_send_fhcustmsg_disconnect_req(CLI_RWNX_HW);
    }
    break;
    case WIFI_CONTROL_GET_STATUS:
    {
        struct fhcustmsg_wlan_status_cfm cfm;
        uint8_t *flag = (uint8_t *)args;
        ret = rwnx_send_fhcustmsg_wlan_status_req(CLI_RWNX_HW, &cfm);
        *flag = cfm.status;
        rt_kprintf("WIFI_CONTROL_GET_STATUS %d\n", *flag);
    }
    break;
    case WIFI_CONTROL_CLEAR_CFG:
    {
        struct fhcustmsg_sta_cfg_cfm cfm_1;
        ret = rwnx_send_fhcustmsg_sta_cfg_req(CLI_RWNX_HW, STA_CFG_REQ_OP_CLEAR, NULL, 0, &cfm_1);
    }
#ifdef RT_USING_PM
    case RT_DEVICE_CTRL_RESUME:
    {

    }
    break;
    case RT_DEVICE_CTRL_SUSPEND:
    {

    }
    break;
#endif
    }
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops wifi_ops =
{
    RT_NULL,
    rt_wifi_open,
    rt_wifi_close,
    RT_NULL,
    RT_NULL,
    rt_wifi_control,
};
#endif

void wifi_device_create(void)
{
    rt_device_t  wifi_dev = (rt_device_t)rt_malloc(sizeof(struct rt_device));
    if (wifi_dev)
    {
        wifi_dev->type = RT_Device_Class_Miscellaneous;
        wifi_dev->rx_indicate = RT_NULL;
        wifi_dev->tx_complete = RT_NULL;
#ifdef RT_USING_DEVICE_OPS
        wifi_dev->ops        = &wifi_ops;
#else
        wifi_dev->init       = RT_NULL;
        wifi_dev->open       = rt_wifi_open;
        wifi_dev->close      = rt_wifi_close;
        wifi_dev->read       = RT_NULL;
        wifi_dev->write      = RT_NULL;
        wifi_dev->control    = rt_wifi_control;
#endif
        wifi_dev->user_data = wifi_dev;

        rt_device_register(wifi_dev, "wifi_dev",
                           RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);
    }
}
int rt_wifi_init(void)
{
    wifi_device_create();
    return 0;
}
INIT_APP_EXPORT(rt_wifi_init);

rt_err_t wifi_test_t(int argc, char **argv)
{
    rt_device_t dev = rt_device_find("wifi_dev");
    if (strcmp(argv[1], "open") == 0)
    {
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    }
    if (strcmp(argv[1], "status") == 0)
    {
        uint8_t status;
        rt_device_control(dev, WIFI_CONTROL_GET_STATUS, &status);
        rt_kprintf("%s %d %d\n", __func__, __LINE__, status);
    }
    if (strcmp(argv[1], "connect") == 0)
    {
        wifi_connect_t con;
        con.ssid = argv[2];
        con.pwd = argv[3];
        rt_device_control(dev, WIFI_CONTROL_CONNECT, &con);
    }
    if (strcmp(argv[1], "disconnect") == 0)
    {
        rt_device_control(dev, WIFI_CONTROL_DISCONNECT, NULL);
    }
    if (strcmp(argv[1], "cfg_clear") == 0)
    {
        rt_device_control(dev, WIFI_CONTROL_CLEAR_CFG, NULL);
    }
    return 0;
}
MSH_CMD_EXPORT(wifi_test_t, wifi stats);


