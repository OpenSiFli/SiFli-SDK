/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * WiFi AT/CLI command dispatcher.
 */

#include "wifi_port_priv.h"
#include "wifi_api.h"
#include "bt_api.h"
#ifdef AIC_HCI_OVER_UART
    #include "aic_bt_uart.h"
#endif
#include "cli_api.h"
#include "netif_api.h"
#include "fmac_port.h"
#include "sdio_port.h"
#include "netif_port.h"
#include "aic_log.h"
#ifdef CONFIG_IPERF
    #include "iperf.h"
#endif
#include <string.h>
#include <stdlib.h>

/*
 * Number of leading placeholder tokens before the command (declares the cmd start position).
 *
 * Some platform Shell/AT frameworks prepend a placeholder token to the real command
 * (e.g. "wifi iperf ..."); in that case the placeholder must be skipped, so set it to 1;
 * if the platform command itself is the first token of the line (e.g. "iperf ..."), set it to 0.
 *
 * Defaults to 1; the actual value depends on the porting platform (see docs/aic8800-porting/SKILL.md).
 */
#ifndef AICWF_CMD_SKIP_PREFIX
    #define AICWF_CMD_SKIP_PREFIX  1
#endif

/**
 * @brief CLI command response callback, returns the result through the AT interface
 * @param rsp Command response string
 */
static void aicwf_app_wlan_cli_cmd_cb(char *rsp)
{
    aicwf_ATRESP(0, 1, 0, rsp);
}

/**
 * @brief Parse the AT command argument string
 *
 * Parses a space/tab separated command string into an argv array,
 * modifying the string in place (separating each argument with '\0').
 *
 * @param cmd  Input command string (will be modified)
 * @param argv Output array of argument pointers
 * @return Number of arguments (argc)
 */
static int aicwf_app_wlan_cmd_parse_argv(char *cmd, char *argv[])
{
    int last, argc;
    char *start, *end;

    argc = 0;
    start = cmd;

    while (start != '\0')
    {
        while (*start == ' ' || *start == '\t')
            start++;
        if (*start == '\0')
            break;
        end = start;
        while (*end != ' ' && *end != '\t' && *end != '\0')
            end++;
        last = *end == '\0';
        *end = '\0';
        argv[argc++] = start;
        if (last)
            break;
        start = end + 1;
    }

    return argc;
}

/**
 * @brief WiFi AT command dispatcher
 *
 * Parses the AT command string and dispatches it to the sub-command handlers.
 * Supported commands include:
 * - aicrftest: RF test
 * - switch: mode switch
 * - stainfo/stanum/stalist: STA information query
 * - uapst: state query
 * - mac: MAC address read/write
 * - mem: memory information/resource management
 * - info: version and chip information
 * - wps: WPS operations
 * - ps/pp: power save/power control
 * - temp: temperature read
 * - dpd: DPD (digital pre-distortion)
 * - shut: shutdown/restart control
 * - fw: firmware version
 * - filter: MAC filter
 * - sta: STA connect/scan
 * - rssi: RSSI read
 * - ap: AP start/stop
 * - agg: aggregation configuration
 * - cca: CCA threshold setting
 * - 5g: 5GHz configuration
 * - fixrate: fixed rate
 * - btstart: Bluetooth start
 * - p2pstart: P2P start
 * - iperf: iperf performance test
 *
 * @param atcmd     AT command string
 * @param athandle  AT interface handle
 * @param chan_num  Channel number
 */
void aic_wifi_cmd(char *atcmd, unsigned int athandle, unsigned char chan_num)
{
    int argc, ret = 0, i;
    char *argv[20];
    char rev_c[64];
    char rsp[128] = {0};
    struct aic_wifi_dev *wifidev = &aicwf_wifi_dev;

    DBG_APP_INF("%s %s\n", __func__, atcmd);
    if (os_strlen(atcmd) > 64)
    {
        DBG_APP_INF("%s too long\n", __func__);
        goto end;
    }

    memset(rev_c, 0, 64);
    memcpy(rev_c, atcmd, os_strlen(atcmd));

    argc = aicwf_app_wlan_cmd_parse_argv(atcmd, &argv[0]);
    if (argc == 0)
    {
        DBG_APP_ERR("cmd error\n");
        goto end;
    }
    /* Skip the leading placeholder token(s) so argv[0] points to the real command name */
    for (i = 0; i < argc - AICWF_CMD_SKIP_PREFIX; i++)
    {
        argv[i] = argv[i + AICWF_CMD_SKIP_PREFIX];
    }
    argc -= AICWF_CMD_SKIP_PREFIX;

    //rf test
    if (os_strcmp(argv[0], "aicrftest") == 0)
    {
        if (wifidev->wifi_mode == WIFI_MODE_RFTEST)
        {
            if (argc > 1)
            {
                ret = aic_cli_run_cmd(rev_c + os_strlen("aicrftest") + 1);
            }
        }
        else
        {
            aicwf_ATRESP(athandle, 1, 0, "rf bin is not exist or not in productmode!!!");
        }
    }
    else if (os_strcmp(argv[0], "switch") == 0)
    {
        AIC_WIFI_MODE mode = WIFI_MODE_UNKNOWN;
        if (os_strcmp(argv[1], "ap") == 0)
            mode =  WIFI_MODE_AP;
        else if (os_strcmp(argv[1], "test") == 0)
            mode =  WIFI_MODE_RFTEST;
        else if (os_strcmp(argv[1], "sta") == 0)
            mode =  WIFI_MODE_STA;

        if ((mode == WIFI_MODE_RFTEST) && !aicwf_rftest_fw_available())
        {
            aicwf_ATRESP(athandle, 1, 0, "rf bin is not exist!!!");
        }
        else
        {
            sprintf(rsp, "set OK wait 2~3s switch %d to %d ", wifidev->wifi_mode, mode);
            wifi_set_mode(mode);
            wifi_driver_action(0x03);
            aicwf_ATRESP(athandle, 1, 0, rsp);
        }
    }
    else if (os_strcmp(argv[0], "stainfo") == 0)
    {
        sta_list stal;
        uint8_t n;

        int ret = app_wlan_sta_list(&stal);

        if (ret == 0)
        {
            for (n = 0; n < stal.sta_count; n++)
            {
                sprintf(rsp, "addr:%x:%x:%x", stal.info[n].mac_address[0], stal.info[n].mac_address[1], stal.info[n].mac_address[2]);
                aicwf_ATRESP(athandle, 1, 0, rsp);
            }

            sprintf(rsp, "sta:n:%d", stal.sta_count);
            aicwf_ATRESP(athandle, 1, 0, rsp);
        }
        else
        {
            aicwf_ATRESP(athandle, 1, 0, "read list fail");
        }
    }
    else if (os_strcmp(argv[0], "uapst") == 0)
    {
        uint8_t st = GetWifiStatus();
        uint8_t ist = GetWiFiInitStat();

        sprintf(rsp, "wifi st:%d init:%d\n", st, ist);
        aicwf_ATRESP(athandle, 1, 0, rsp);
        return;
    }
    else if (os_strcmp(argv[0], "mac") == 0)
    {
        if (GetWiFiInitStat())
        {
            char mac[6];
            char rsp[32] = {0};
            int type = strtoul(argv[1], NULL, 10);
            if (type == 0)
            {
                GetMACAddr((uint8_t *)mac);
                if (argc == 8)
                {
                    mac[0] = (char)strtoul(argv[2], NULL, 16);
                    mac[1] = (char)strtoul(argv[3], NULL, 16);
                    mac[2] = (char)strtoul(argv[4], NULL, 16);
                    mac[3] = (char)strtoul(argv[5], NULL, 16);
                    mac[4] = (char)strtoul(argv[6], NULL, 16);
                    mac[5] = (char)strtoul(argv[7], NULL, 16);
                    aic_set_mac_address((uint8_t *)mac);
                    sprintf(rsp, "mac:%x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                    aicwf_ATRESP(athandle, 1, 0, rsp);
                }
                else
                {
                    aicwf_ATRESP(athandle, 1, 0, "param err");
                }
            }
            else if (type == 1)
            {
                memcpy(mac, aic_get_mac_address(), 6);
                sprintf(rsp, "mac:%x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                aicwf_ATRESP(athandle, 1, 0, rsp);
            }
            else
            {
                aicwf_ATRESP(athandle, 1, 0, "param err");
            }
        }
        else
            aicwf_ATRESP(athandle, 1, 0, "wifi is not init");
    }
    else if (os_strcmp(argv[0], "stanum") == 0)
    {
        int n;
        n = app_wlan_get_sta_num();
        sprintf(rsp, "n:%d\n", n);
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
    else if (os_strcmp(argv[0], "mem") == 0)
    {
#ifdef CONFIG_RTOS_RESLIST
        uint32_t mcnt, fcnt = 0, rcnt;
        static void *test_p = NULL;
        mcnt = strtoul(argv[1], NULL, 10);
        if (mcnt == 0)
        {
            aicwf_rtos_mem_cur_cnt(&mcnt, &fcnt, &rcnt);
            sprintf(rsp, "mem u:%d  m:%d f:%d\n", mcnt - fcnt, mcnt, fcnt);
            if (test_p)
                aicwf_rtos_free(test_p);
            aicwf_ATRESP(athandle, 1, 0, rsp);
        }
        else if (mcnt == 1)
        {
            aicwf_rtos_res_list_info_show();
            aicwf_ATRESP(athandle, 1, 0, "ok see log");
            return;
        }
        else if (mcnt == 2)
        {
            aicwf_rtos_remove_all();
            aicwf_ATRESP(athandle, 1, 0, "ok see log");
            return;
        }
        else
        {
            test_p = aicwf_rtos_malloc(mcnt);
        }

        if (mcnt > fcnt)
            sprintf(rsp, "mem u:%d  m:%d f:%d %p\n", mcnt - fcnt, mcnt, fcnt, test_p);
        else
            sprintf(rsp, "mem u:%d  m:%d f:%d %p\n", mcnt + (0xFFFFFFFF - fcnt), mcnt, fcnt, test_p);
        aicwf_ATRESP(athandle, 1, 0, rsp);
#else
        aicwf_ATRESP(athandle, 1, 0, rsp);
#endif
    }
    else if (os_strcmp(argv[0], "info") == 0)
    {
        uint8_t info[3] = {0};
        char ver[64] = {0};
        aic_wifi_sw_version(ver);
        aic_get_chip_info(info);
        sprintf(rsp, "ver:%s cid:0x%x(%s) sid:%d fmat:%d mode:%d\n", ver, info[0], (info[0] & 0xc0) ? "H" : "T", info[1], info[2], wifidev->wifi_mode);
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
#ifdef CONFIG_STATION
    else if (os_strcmp(argv[0], "apinfo") == 0)
    {
        struct aic_connected_ap_info_t *apinfo = aic_wifi_get_sta_connected_ap_info();
        if (apinfo)
        {
            sprintf(rsp, "ssid:%.32s bssid:%02x:%02x:%02x:%02x:%02x:%02x freq:%u band:%u fmt:%u rate:%u",
                    apinfo->ssid,
                    apinfo->bssid[0], apinfo->bssid[1], apinfo->bssid[2],
                    apinfo->bssid[3], apinfo->bssid[4], apinfo->bssid[5],
                    apinfo->freq, apinfo->band, apinfo->format, apinfo->max_rate);
            aicwf_ATRESP(athandle, 1, 0, rsp);
        }
        else
        {
            aicwf_ATRESP(athandle, 1, 0, "Conn AP info unavailable");
        }
    }
#endif
    else if (os_strcmp(argv[0], "wps") == 0)
    {
        if (os_strcmp(argv[1], "pbc") == 0)
        {
            app_wlan_wps_use_pbc();
        }
        else if (os_strcmp(argv[1], "pin") == 0)
        {
            app_wlan_wps_use_pin(argv[2]);
        }
        else if (os_strcmp(argv[1], "cancel") == 0)
        {
            app_wlan_wps_use_cancel();
        }
        else if (os_strcmp(argv[1], "disable") == 0)
        {
            app_wlan_wps_disable();
        }
        else if (os_strcmp(argv[1], "cms") == 0)
        {
            aic_ap_wps_config_methods(argv[2]);
        }
    }
    else if (os_strcmp(argv[0], "ps") == 0)
    {
        int ret = 0, lvl = 0;
        lvl = strtoul(argv[1], NULL, 10);
        ret = aic_wifi_set_ps_lvl(lvl);
        if (!ret)
        {
            sprintf(rsp, "ap_ps lvl:%d, ret:%d\n", lvl, ret);
            aicwf_ATRESP(athandle, 1, 0, rsp);
        }
        else
        {
            aicwf_ATRESP(athandle, 1, 0, "+ERR\n");
        }
    }
    else if (os_strcmp(argv[0], "pp") == 0)
    {
        int lv = strtoul(argv[1], NULL, 10);
        if (lv <= 8)
        {
            aicwf_hal_switchPP(lv);
            sprintf(rsp, "pp set: %d,%d\n", lv, ret);
        }
        else
        {
            sprintf(rsp, "pp err: %d,%d\n", lv, ret);
        }
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
    else if (os_strcmp(argv[0], "temp") == 0)
    {
        int32_t temp = 0;
        ret = aic_wifi_get_temp(&temp);
        if (!ret)
            sprintf(rsp, "get chip temp: %d\n", temp);
        else
            sprintf(rsp, "get chip temp err: %d\n", ret);
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
#if (defined(CONFIG_AIC8800DC) || defined(CONFIG_AIC8800DW))
    else if (os_strcmp(argv[0], "dpd") == 0)
    {
        uint32_t result = aic_get_dpd_result();
#ifdef CONFIG_DPD
        sprintf(rsp, "dpd result: %x\n", result);
        aicwf_ATRESP(athandle, 1, 0, rsp);
#else
        aicwf_ATRESP(athandle, 1, 0, "+ERR");
#endif
    }
#endif
    else if (os_strcmp(argv[0], "shut") == 0)
    {
        uint32_t op;
        op = strtoul(argv[1], NULL, 10);
        if (op == 222)
        {
            sdio_unexcept_test = 1;
        }
        else if (op == 666)
        {
            sdio_unexcept_test = 2;
        }
        else if (op == 999)
        {
            sdio_unexcept_test = 0;
        }
        else
        {
            wifi_driver_action(op);
        }

        sprintf(rsp, "+OK: %d %d\n", op, sdio_unexcept_test);
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
    else if (os_strcmp(argv[0], "fw") == 0)
    {
        sprintf(rsp, "fw ver: %s\n", aic_get_fw_ver());
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
    else if (os_strcmp(argv[0], "filter") == 0)
    {
        uap_mac_filter filter;
        int fmode = strtoul(argv[1], NULL, 10);

        filter.filter_mode = fmode;
        filter.mac_count = 1;
        filter.mac_list[0][0] = strtoul(argv[2], NULL, 16);
        filter.mac_list[0][1] = strtoul(argv[3], NULL, 16);
        filter.mac_list[0][2] = strtoul(argv[4], NULL, 16);
        filter.mac_list[0][3] = strtoul(argv[5], NULL, 16);
        filter.mac_list[0][4] = strtoul(argv[6], NULL, 16);
        filter.mac_list[0][5] = strtoul(argv[7], NULL, 16);

        app_wlan_params_set_filter_only(&filter, NULL);
    }
    else if (os_strcmp(argv[0], "sta") == 0)
    {
        uint32_t ip;
        uint8_t ipb[4];

        if (os_strcmp(argv[1], "conn") == 0)
        {
            uint8_t *ssid = (uint8_t *)argv[2];
            uint8_t *pw = (uint8_t *)argv[3];
            uint16_t to = strtoul(argv[4], NULL, 10);
            ret = app_wlan_sta_start(ssid, pw, to);

            if (ret >= 0)
            {
                ip = app_wlan_sta_get_ip();
                memcpy(ipb, &ip, 4);
                sprintf(rsp, "+OK ip: %u:%u:%u:%u\n", ipb[0], ipb[1], ipb[2], ipb[3]);
                aicwf_ATRESP(athandle, 1, 0, rsp);
            }
            else
            {
                aicwf_ATRESP(athandle, 1, 0, "+ERR");
            }
            return;
        }
        else if (os_strcmp(argv[1], "scan") == 0)
        {
            ret = app_wlan_scan_start();
            if (ret >= 0)
            {
                app_wlan_sta_show_scan_res();
                aicwf_ATRESP(athandle, 1, 0, "+OK");
            }
            else
            {
                aicwf_ATRESP(athandle, 1, 0, "+ERR");
            }
            return;

        }
        else if (os_strcmp(argv[1], "stop") == 0)
        {
            app_wlan_sta_stop();
        }
    }
    else if (os_strcmp(argv[0], "rssi") == 0)
    {
        int rssi = aic_wifi_ap_rssi_get();
        sprintf(rsp, "Conn AP rssi: %d\n", rssi);
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
    else if (os_strcmp(argv[0], "ap") == 0)
    {
        if (os_strcmp(argv[1], "start") == 0)
        {
            wifi_ap_cfg.band = strtoul(argv[2], NULL, 10);
            wifi_ap_cfg.type = strtoul(argv[3], NULL, 10);
            wifi_ap_cfg.enable_he = strtoul(argv[4], NULL, 10);
            wifi_ap_cfg.enable_acs = strtoul(argv[5], NULL, 10);
            wifi_ap_cfg.channel =  strtoul(argv[6], NULL, 10);
            wifi_ap_cfg.sercurity_type = strtoul(argv[7], NULL, 10);
            /* On success app_wlan_bss_config sets wifi_mode=AP internally; on failure the mode state is left untouched */
            app_wlan_bss_config(1);
        }
        else if (os_strcmp(argv[1], "stop") == 0)
        {
            app_wlan_bss_config(0);
        }
        else
        {
            aicwf_ATRESP(athandle, 1, 0, "+ERR");
        }
    }
    else if (os_strcmp(argv[0], "stalist") == 0)
    {
        aic_wifi_get_stalist2str(rsp);
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
    else if (os_strcmp(argv[0], "agg") == 0)
    {
        int en, id;
        en = strtoul(argv[1], NULL, 10);
        id = strtoul(argv[2], NULL, 10);
        aic_wifi_set_agg(en, id);
    }
    else if (os_strcmp(argv[0], "bare_bypass") == 0)
    {
        /* [backport EC] Runtime switch for the pure TCP ACK reorder bypass (rwnx_utils.c) */
        extern uint8_t aicwifi_bare_bypass;
        int value = strtoul(argv[1], NULL, 10);
        if (value <= 1)
        {
            aicwifi_bare_bypass = value;
            sprintf(rsp, "+OK bare_bypass:%d\n", aicwifi_bare_bypass);
        }
        else
        {
            sprintf(rsp, "+ERR bare_bypass:input err:%d\n", value);
        }
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
    else if (os_strcmp(argv[0], "cca") == 0)
    {
        extern int aic_wifi_set_cca_threshold_param(struct aic_mm_set_cca_threshold_req * req);
        struct aic_mm_set_cca_threshold_req reqt;
        int r, f;
        r = strtol(argv[1], NULL, 10);
        f = strtol(argv[2], NULL, 10);
        reqt.auto_cca_en = 0;
        reqt.hwconfig_id = 0;
        reqt.cca20p_rise_th = r;
        reqt.cca20s_fall_th = r;
        reqt.cca20p_fall_th = f;
        reqt.cca20s_fall_th = f;

        aic_wifi_set_cca_threshold_param(&reqt);

        sprintf(rsp, "+OK r:%d f:%d\n", r, f);
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
    else if (os_strcmp(argv[0], "5g") == 0)
    {
        uap_config_param *uap_cfg = (uap_config_param *)aicwf_rtos_malloc(sizeof(uap_config_param));
        uap_config_scan_chan *scan_param = (uap_config_scan_chan *)aicwf_rtos_malloc(sizeof(uap_config_scan_chan));

        memset(uap_cfg, 0, sizeof(uap_config_param));
        memset(scan_param, 0, sizeof(uap_config_scan_chan));

#define AP_SSID_5G  "BTDs-5G"
#define AP_PASS_5G  "12345678"

        memcpy(uap_cfg->ssid, AP_SSID_5G, strlen(AP_SSID_5G));
        uap_cfg->ssid_len = strlen(AP_SSID_5G);
        memcpy(uap_cfg->wpa_cfg.passphrase, AP_PASS_5G, strlen(AP_PASS_5G));
        uap_cfg->wpa_cfg.length = strlen(AP_PASS_5G);
        uap_cfg->bcast_ssid_ctrl = 1;
        uap_cfg->action = UAP_ACTION_SET;
        uap_cfg->mode = UAP_BAND_AX;
        uap_cfg->acs_mode = 0;
        uap_cfg->channel = 40;
        uap_cfg->rf_band = UAP_RF_50_G;
        uap_cfg->channel_bandwidth = UAP_CHAN_WIDTH_40MHZ;
        uap_cfg->sec_chan = UAP_SEC_CHAN_ABOVE;
        uap_cfg->max_sta_count = 12;
        uap_cfg->protocol = UAP_PROTOCOL_WPA;
        uap_cfg->wpa_cfg.group_cipher = 1;

        scan_param->action = ACTION_SET;
        scan_param->band = UAP_RF_50_G;
        scan_param->total_chan = 13;
        scan_param->chan_num[0] = 36;
        scan_param->chan_num[1] = 40;
        scan_param->chan_num[2] = 44;
        scan_param->chan_num[3] = 48;
        scan_param->chan_num[4] = 52;
        scan_param->chan_num[5] = 56;
        scan_param->chan_num[6] = 60;
        scan_param->chan_num[7] = 64;
        scan_param->chan_num[8] = 149;
        scan_param->chan_num[9] = 153;
        scan_param->chan_num[10] = 157;
        scan_param->chan_num[11] = 161;
        scan_param->chan_num[12] = 165;

        app_wlan_scan_channels_config(scan_param);
        aicwf_rtos_free(scan_param);
        aicwf_rtos_free(uap_cfg);
    }
    else if (os_strcmp(argv[0], "fixrate") == 0)
    {
        extern int aic_wifi_fixrate(int argc, int argv[]);
        int param[4];
        param[0] = strtoul(argv[1], NULL, 10);
        param[1] = strtoul(argv[2], NULL, 10);
        param[2] = strtoul(argv[3], NULL, 10);
        param[3] = strtoul(argv[4], NULL, 10);

        aic_wifi_fixrate(4, param);

        sprintf(rsp, "+OK fixrate:%d %d %d %d\n", param[0], param[1], param[2], param[3]);
        aicwf_ATRESP(athandle, 1, 0, rsp);
    }
#ifdef AIC_FEATURE_BT
    else if (os_strcmp(argv[0], "btstart") == 0)
    {
#ifdef AIC_HCI_OVER_UART
        /* UART HCI backend entry (aic_hci.c maps hci_tr_* at compile time): open UART + start
         * the H4 reassembly task, idempotent; on the NimBLE path aic_hci_init also reaches here via hci_tr_start */
        (void)aic_bt_uart_hci_start();
#endif
#if defined(AIC_FEATURE_NIMBLE)
        /* On the SDIO/UART mailbox path the BLE stack is started automatically by wifi_port_plt during WiFi bring-up;
         * this also covers a manual restart after btstop (nimble_stack_start is idempotent) */
        extern int nimble_stack_start(void);
        (void)nimble_stack_start();
#endif
        aicwf_ATRESP(athandle, 1, 0, "+OK");
    }
    else if (os_strcmp(argv[0], "btstop") == 0)
    {
#if defined(AIC_FEATURE_NIMBLE)
        /* Stop BLE: stop advertising -> exit host task -> stop HCI RX, and reset the idempotent flag so it can restart.
         * BLE only, no impact on WiFi (the shared fhost_rx_task is protected on the wlan_if.c side). */
        extern int nimble_stack_stop(void);
        aicwf_ATRESP(athandle, 1, 0, (nimble_stack_stop() == 0) ? "+OK" : "+ERR");
#else
        aicwf_ATRESP(athandle, 1, 0, "+ERR");
#endif
    }
#endif
#ifdef CONFIG_P2P_SUPPORT
    else if (os_strcmp(argv[0], "p2pstart") == 0)
    {
        char *name = argv[1];
        char *passphrase = argv[2];
        char *ssid = argv[3];

        if (argc > 3)
        {
            int ret = app_wlan_p2p_start(name, ssid, passphrase);
            if (ret == 0)
            {
                aicwf_ATRESP(athandle, 1, 0, "+OK");
            }
            else
            {
                aicwf_ATRESP(athandle, 1, 0, "+ERR");
            }
        }
        else
        {
            app_wlan_p2p_stop();
        }
    }
#endif
#ifdef CONFIG_IPERF
    else if (os_strcmp(argv[0], "iperf") == 0)
    {
        if (os_strcmp(argv[1], "start") == 0)
        {
            int handle;
            handle = iperf_parse_argv(argc - 1, &argv[1]);
            net_if_t *netif = aicwf_net_if_get(0);
            if (handle < 0)
            {
                aicwf_ATRESP(athandle, 1, 0, "+ERR");
            }
            else
            {
                iperf_start((struct netif *)netif, handle);
            }
        }
        else if (os_strcmp(argv[1], "stop") == 0)
        {
            iperf_stop("a");
        }
    }
#endif

end:
    aicwf_ATRESP(athandle, 1, ret, "aic cmd finish");
}
