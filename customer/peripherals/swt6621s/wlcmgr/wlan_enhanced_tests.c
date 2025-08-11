/** @file wlan_enhanced_tests.c
 *
 *  @brief  This file provides WLAN ENHANCED Test API
 *
 *  Copyright 2008-2020, 2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <wlan_wifi.h>
#include <cli.h>
#include <cli_utils.h>
#include <string.h>
#include <wm_net.h> /* for net_inet_aton */
#include <wifi.h>
#include <wlan_tests.h>

#ifdef WIFI_BT_TX_PWR_LIMITS
    #include WIFI_BT_TX_PWR_LIMITS
#else
    //#error "Region tx power config not defined"
#endif

/*
 * NXP Test Framework (MTF) functions
 */

static void dump_wlan_uap_get_pmfcfg_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-uap-get-pmfcfg \r\n");
}

static void wlan_uap_pmfcfg_get(int argc, char *argv[])
{
    int ret;
    uint8_t mfpc = 0U;
    uint8_t mfpr = 0U;

    if (argc != 1)
    {
        dump_wlan_uap_get_pmfcfg_usage();
        return;
    }

    ret = wlan_uap_get_pmfcfg(&mfpc, &mfpr);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Uap Management Frame Protection Capability: %s\r\n", mfpc == 1 ? "Yes" : "No");
        if (mfpc != 0U)
            (void)PRINTF("Uap Management Frame Protection: %s\r\n", mfpr == 1 ? "Required" : "Optional");
    }
    else
    {
        (void)PRINTF("Uap PMF configuration read failed\r\n");
        dump_wlan_uap_get_pmfcfg_usage();
    }
}

static void dump_wlan_get_pmfcfg_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-pmfcfg \r\n");
}

static void wlan_pmfcfg_get(int argc, char *argv[])
{
    int ret;
    uint8_t mfpc, mfpr;

    if (argc != 1)
    {
        dump_wlan_get_pmfcfg_usage();
        return;
    }

    ret = wlan_get_pmfcfg(&mfpc, &mfpr);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Management Frame Protection Capability: %s\r\n", mfpc == 1U ? "Yes" : "No");
        if (mfpc != 0U)
        {
            (void)PRINTF("Management Frame Protection: %s\r\n", mfpr == 1U ? "Required" : "Optional");
        }
    }
    else
    {
        (void)PRINTF("PMF configuration read failed\r\n");
        dump_wlan_get_pmfcfg_usage();
    }
}

static void dump_wlan_set_ed_mac_mode_usage(void)
{
    (void)PRINTF("Usage:\r\n");
#if CONFIG_5GHz_SUPPORT
    (void)PRINTF("wlan-set-ed-mac-mode <interface> <ed_ctrl_2g> <ed_offset_2g> <ed_ctrl_5g> <ed_offset_5g>\r\n");
#else
    (void)PRINTF("wlan-set-ed-mac-mode <interface> <ed_ctrl_2g> <ed_offset_2g>\r\n");
#endif
    (void)PRINTF("\r\n");
    (void)PRINTF("\tinterface \r\n");
    (void)PRINTF("\t    # 0       - for STA\r\n");
    (void)PRINTF("\t    # 1       - for uAP\r\n");
    (void)PRINTF("\ted_ctrl_2g \r\n");
    (void)PRINTF("\t    # 0       - disable EU adaptivity for 2.4GHz band\r\n");
    (void)PRINTF("\t    # 1       - enable EU adaptivity for 2.4GHz band\r\n");
    (void)PRINTF("\ted_offset_2g \r\n");
    (void)PRINTF("\t    # 0       - Default Energy Detect threshold\r\n");
    (void)PRINTF("\t    # ed_threshold = ed_base - ed_offset_2g\r\n");
    (void)PRINTF("\t    # e.g., if ed_base default is -62dBm, ed_offset_2g is 0x8, then ed_threshold is -70dBm\r\n");
#if CONFIG_5GHz_SUPPORT
    (void)PRINTF("\ted_ctrl_5g \r\n");
    (void)PRINTF("\t    # 0       - disable EU adaptivity for 5GHz band\r\n");
    (void)PRINTF("\t    # 1       - enable EU adaptivity for 5GHz band\r\n");
    (void)PRINTF("\ted_offset_5g \r\n");
    (void)PRINTF("\t    # 0       - Default Energy Detect threshold\r\n");
    (void)PRINTF("\t    # ed_threshold = ed_base - ed_offset_5g\r\n");
    (void)PRINTF("\t    # e.g., if ed_base default is -62dBm, ed_offset_5g is 0x8, then ed_threshold is -70dBm\r\n");
#endif
}

static void wlan_ed_mac_mode_set(int argc, char *argv[])
{
    int ret;
    wlan_ed_mac_ctrl_t wlan_ed_mac_ctrl;
    t_u8 interface;

#if CONFIG_5GHz_SUPPORT
    if (argc != 6)
#else
    if (argc != 4)
#endif
    {
        dump_wlan_set_ed_mac_mode_usage();
        return;
    }

    errno     = 0;
    interface = (t_u8)strtol(argv[1], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
    errno                       = 0;
    wlan_ed_mac_ctrl.ed_ctrl_2g = (t_u16)strtol(argv[2], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
    errno                         = 0;
    wlan_ed_mac_ctrl.ed_offset_2g = (t_s16)strtol(argv[3], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
#if CONFIG_5GHz_SUPPORT
    errno                       = 0;
    wlan_ed_mac_ctrl.ed_ctrl_5g = (t_u16)strtol(argv[4], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
    errno                         = 0;
    wlan_ed_mac_ctrl.ed_offset_5g = (t_s16)strtol(argv[5], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
#endif

    if (wlan_ed_mac_ctrl.ed_ctrl_2g != 0U && wlan_ed_mac_ctrl.ed_ctrl_2g != 1U)
    {
        dump_wlan_set_ed_mac_mode_usage();
        return;
    }
#if CONFIG_5GHz_SUPPORT
    if (wlan_ed_mac_ctrl.ed_ctrl_5g != 0U && wlan_ed_mac_ctrl.ed_ctrl_5g != 1U)
    {
        dump_wlan_set_ed_mac_mode_usage();
        return;
    }
#endif

    if (interface == MLAN_BSS_TYPE_STA)
    {
        ret = wlan_set_ed_mac_mode(wlan_ed_mac_ctrl);
    }
    else
    {
        ret = wlan_set_uap_ed_mac_mode(wlan_ed_mac_ctrl);
    }
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("ED MAC MODE settings configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("ED MAC MODE settings configuration failed\r\n");
        dump_wlan_set_ed_mac_mode_usage();
    }
}

static void dump_wlan_get_ed_mac_mode_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-ed-mac-mode <interface>\r\n");
    (void)PRINTF("\r\n");
    (void)PRINTF("\tinterface \r\n");
    (void)PRINTF("\t    # 0       - for STA\r\n");
    (void)PRINTF("\t    # 1       - for uAP\r\n");
}

static void wlan_ed_mac_mode_get(int argc, char *argv[])
{
    int ret;
    wlan_ed_mac_ctrl_t wlan_ed_mac_ctrl;
    int interface;

    if (argc != 2)
    {
        dump_wlan_get_ed_mac_mode_usage();
        return;
    }
    errno     = 0;
    interface = (t_u8)strtol(argv[1], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }

    if (interface == MLAN_BSS_TYPE_STA)
    {
        ret = wlan_get_ed_mac_mode(&wlan_ed_mac_ctrl);
    }
    else
    {
        ret = wlan_get_uap_ed_mac_mode(&wlan_ed_mac_ctrl);
    }
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("EU adaptivity for 2.4GHz band : %s\r\n",
                     wlan_ed_mac_ctrl.ed_ctrl_2g == 1U ? "Enabled" : "Disabled");
        if (wlan_ed_mac_ctrl.ed_ctrl_2g != 0U)
        {
            (void)PRINTF("Energy Detect threshold offset : 0X%x\r\n", wlan_ed_mac_ctrl.ed_offset_2g);
        }
#if CONFIG_5GHz_SUPPORT
        (void)PRINTF("EU adaptivity for 5GHz band : %s\r\n",
                     wlan_ed_mac_ctrl.ed_ctrl_5g == 1U ? "Enabled" : "Disabled");
        if (wlan_ed_mac_ctrl.ed_ctrl_5g != 0U)
        {
            (void)PRINTF("Energy Detect threshold offset : 0X%x\r\n", wlan_ed_mac_ctrl.ed_offset_5g);
        }
#endif
    }
    else
    {
        (void)PRINTF("ED MAC MODE read failed\r\n");
        dump_wlan_get_ed_mac_mode_usage();
    }
}
#if 0
static int wlan_memrdwr_getset(int argc, char *argv[])
{
    uint8_t action;
    uint32_t value;
    int ret;

    if (argc != 3 && argc != 4)
    {
        return -WM_FAIL;
    }

    if (argc == 3)
    {
        action = ACTION_GET;
        value  = 0;
    }
    else
    {
        action = ACTION_SET;
        value  = a2hex_or_atoi(argv[3]);
    }

    ret = wifi_mem_access(action, a2hex_or_atoi(argv[2]), &value);

    if (ret == WM_SUCCESS)
    {
        if (action == ACTION_GET)
        {
            (void)PRINTF("At Memory 0x%x: 0x%x\r\n", a2hex_or_atoi(argv[2]), value);
        }
        else
        {
            (void)PRINTF("Set the Memory successfully\r\n");
        }
    }
    else
    {
        wlcm_e("Read/write Mem failed");
        return -WM_FAIL;
    }
    return WM_SUCCESS;
}
#endif

static char *bw[]           = {"20 MHz", "40 MHz", "80 MHz", "160 MHz"};
static char *rate_format[4] = {"LG", "HT", "VHT", "HE"};
static char *lg_rate[]      = {"1 Mbps",  "2 Mbps",  "5.5 Mbps", "11 Mbps", "6 Mbps",  "9 Mbps",
                               "12 Mbps", "18 Mbps", "24 Mbps",  "36 Mbps", "48 Mbps", "54 Mbps"
                              };

static void print_ds_rate(wlan_ds_rate ds_rate)
{
    if (ds_rate.sub_command == WIFI_DS_RATE_CFG)
    {
        (void)PRINTF("Tx Rate Configuration: \r\n");
        /* format */
        if (ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_AUTO)
        {
            (void)PRINTF("    Type:       0xFF (Auto)\r\n");
        }
        else if ((unsigned int)(ds_rate.param.rate_cfg.rate_format) <= 3U)
        {
            (void)PRINTF("    Type:         %d (%s)\r\n", ds_rate.param.rate_cfg.rate_format,
                         rate_format[ds_rate.param.rate_cfg.rate_format]);
            if (ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_LG)
            {
                (void)PRINTF("    Rate Index: %d (%s)\r\n", ds_rate.param.rate_cfg.rate_index,
                             lg_rate[ds_rate.param.rate_cfg.rate_index]);
            }
            else if (ds_rate.param.rate_cfg.rate_format >= MLAN_RATE_FORMAT_HT)
            {
                (void)PRINTF("    MCS Index:  %d\r\n", (int)ds_rate.param.rate_cfg.rate_index);
            }
            else
            {
                /* Do Nothing */
            }
#if CONFIG_11AC
            if ((ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_VHT)
#if CONFIG_11AX
                    || (ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_HE)
#endif
               )
            {
                (void)PRINTF("    NSS:        %d\r\n", (int)ds_rate.param.rate_cfg.nss);
            }
#endif
#if CONFIG_11AX
            if (ds_rate.param.rate_cfg.rate_setting == 0xffff)
                (void)PRINTF("    Rate setting: Preamble type/BW/GI/STBC/.. : auto \r\n");
            else
            {
                (void)PRINTF("    HE Rate setting:   0x%x\r\n", ds_rate.param.rate_cfg.rate_setting);
                (void)PRINTF("        Preamble type: %x\r\n", (ds_rate.param.rate_cfg.rate_setting & 0x0003));
                (void)PRINTF("        BW:            %x\r\n", (ds_rate.param.rate_cfg.rate_setting & 0x001C) >> 2);
                (void)PRINTF("        LTF + GI size: %x\r\n", (ds_rate.param.rate_cfg.rate_setting & 0x0060) >> 5);
                (void)PRINTF("        STBC:          %x\r\n", (ds_rate.param.rate_cfg.rate_setting & 0x0080) >> 7);
                (void)PRINTF("        DCM:           %x\r\n", (ds_rate.param.rate_cfg.rate_setting & 0x0100) >> 8);
                (void)PRINTF("        Coding:        %x\r\n", (ds_rate.param.rate_cfg.rate_setting & 0x0200) >> 9);
                (void)PRINTF("        maxPE:         %x\r\n", (ds_rate.param.rate_cfg.rate_setting & 0x3000) >> 12);
            }
#endif
        }
        else
        {
            (void)PRINTF("    Unknown rate format.\r\n");
        }
    }
    else if (ds_rate.sub_command == WIFI_DS_GET_DATA_RATE)
    {
        wifi_data_rate_t *datarate = (wifi_data_rate_t *) & (ds_rate.param.data_rate);
        (void)PRINTF("Data Rate:\r\n");
#ifdef SD8801
        (void)PRINTF("  TX: \r\n");
        if (datarate->tx_data_rate < 12)
        {
            (void)PRINTF("    Type: %s\r\n", rate_format[0]);
            /* LG */
            (void)PRINTF("    Rate: %s\r\n", lg_rate[datarate->tx_data_rate]);
        }
        else
        {
            /* HT*/
            (void)PRINTF("    Type: %s\r\n", rate_format[1]);
            if (datarate->tx_bw <= 2)
                (void)PRINTF("    BW:   %s\r\n", bw[datarate->tx_bw]);
            if (datarate->tx_gi == 0)
                (void)PRINTF("    GI:   Long\r\n");
            else
                (void)PRINTF("    GI:   Short\r\n");
            (void)PRINTF("    MCS:  MCS %d\r\n", (int)(datarate->tx_data_rate - 12));
        }

        (void)PRINTF("  RX: \n");
        if (datarate->rx_data_rate < 12)
        {
            (void)PRINTF("    Type: %s\r\n", rate_format[0]);
            /* LG */
            (void)PRINTF("    Rate: %s\r\n", lg_rate[datarate->rx_data_rate]);
        }
        else
        {
            /* HT*/
            (void)PRINTF("    Type: %s\r\n", rate_format[1]);
            if (datarate->rx_bw <= 2)
            {
                (void)PRINTF("    BW:   %s\r\n", bw[datarate->rx_bw]);
            }
            if (datarate->rx_gi == 0)
            {
                (void)PRINTF("    GI:   Long\r\n");
            }
            else
            {
                (void)PRINTF("    GI:   Short\r\n");
            }
            (void)PRINTF("    MCS:  MCS %d\r\n", (int)(datarate->rx_data_rate - 12));
        }
#else
        (void)PRINTF("  TX: \r\n");
        if ((unsigned int)(datarate->tx_rate_format) <= 3U)
        {
            (void)PRINTF("    Type: %s\r\n", rate_format[datarate->tx_rate_format]);
            if ((datarate->tx_rate_format == MLAN_RATE_FORMAT_LG) && datarate->tx_data_rate <= 11U)
            {
                /* LG */
                (void)PRINTF("    Rate: %s\r\n", lg_rate[datarate->tx_data_rate]);
            }
            else
            {
                /* HT, VHT, HE*/
                if (datarate->tx_bw <= 3)
                    (void)PRINTF("    BW:   %s\r\n", bw[datarate->tx_bw]);
                if (datarate->tx_rate_format < 3)
                {
                    if (datarate->tx_gi == 0)
                        (void)PRINTF("    GI:   Long\r\n");
                    else
                        (void)PRINTF("    GI:   Short\r\n");
                }
#if CONFIG_11AX
                else if (datarate->tx_rate_format == 3)
                {
                    switch (datarate->tx_gi)
                    {
                    case 0:
                        (void)PRINTF("    GI:   1xHELTF + GI 0.8us\r\n");
                        break;
                    case 1:
                        (void)PRINTF("    GI:   2xHELTF + GI 0.8us\r\n");
                        break;
                    case 2:
                        (void)PRINTF("    GI:   2xHELTF + GI 1.6us\r\n");
                        break;
                    case 3:
                        (void)PRINTF(
                            "    GI:   4xHELTF + GI 0.8us DCM=0 and STBC=0 or\r\n"
                            "          4xHELTF + GI 3.2us Otherwise  \r\n");
                        break;
                    }
                }
#endif
#if (CONFIG_11AC) || (CONFIG_11AX)
                if (datarate->tx_rate_format >= 2)
                    (void)PRINTF("    NSS:  %d\r\n", datarate->tx_nss + 1);
#endif
                if (datarate->tx_mcs_index != 0xFFU)
                {
                    (void)PRINTF("    MCS:  MCS %d\r\n", (int)datarate->tx_mcs_index);
                }
                else
                {
                    (void)PRINTF("    MCS:  Auto\r\n");
                }
                (void)PRINTF("    Rate: %.2f Mbps\r\n", (double)datarate->tx_data_rate / 2);
            }
        }

        (void)PRINTF("  RX: \r\n");
        if ((unsigned int)(datarate->rx_rate_format) <= 3U)
        {
            (void)PRINTF("    Type: %s\r\n", rate_format[datarate->rx_rate_format]);
            if ((datarate->rx_rate_format == MLAN_RATE_FORMAT_LG) && datarate->rx_data_rate <= 11U)
            {
                /* LG */
                (void)PRINTF("    Rate: %s\r\n", lg_rate[datarate->rx_data_rate]);
            }
            else
            {
                /* HT, VHT, HE*/
                if (datarate->rx_bw <= 3)
                    (void)PRINTF("    BW:   %s\r\n", bw[datarate->rx_bw]);
                if (datarate->rx_rate_format < 3)
                {
                    if (datarate->rx_gi == 0)
                        (void)PRINTF("    GI:   Long\r\n");
                    else
                        (void)PRINTF("    GI:   Short\r\n");
                }
#if CONFIG_11AX
                else if (datarate->rx_rate_format == 3)
                {
                    switch (datarate->rx_gi)
                    {
                    case 0:
                        (void)PRINTF("    GI:   1xHELTF + GI 0.8us\r\n");
                        break;
                    case 1:
                        (void)PRINTF("    GI:   2xHELTF + GI 0.8us\r\n");
                        break;
                    case 2:
                        (void)PRINTF("    GI:   2xHELTF + GI 1.6us\r\n");
                        break;
                    case 3:
                        (void)PRINTF(
                            "    GI:   4xHELTF + GI 0.8us DCM=0 and STBC=0 or\r\n"
                            "          4xHELTF + GI 3.2us Otherwise  \r\n");
                        break;
                    }
                }
#endif
#if (CONFIG_11AC) || (CONFIG_11AX)
                if (datarate->rx_rate_format >= 2)
                    (void)PRINTF("    NSS:  %d\r\n", datarate->rx_nss + 1);
#endif
                if (datarate->rx_mcs_index != 0xFFU)
                {
                    (void)PRINTF("    MCS:  MCS %d\r\n", (int)datarate->rx_mcs_index);
                }
                else
                {
                    (void)PRINTF("    MCS:  Auto\n");
                }
                (void)PRINTF("    Rate: %.2f Mbps\r\n", (double)datarate->rx_data_rate / 2);
            }
        }
#endif
    }
    else
    {
        /* Do Nothing */
    }
}

static void dump_wlan_set_txratecfg_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-txratecfg <sta/uap> <format> <index> ");
#if (CONFIG_11AC) || (CONFIG_11AX)
    (void)PRINTF("<nss> ");
    (void)PRINTF("<rate_setting> ");
#endif
#if CONFIG_AUTO_NULL_TX
    (void)PRINTF("<autoTx_set>\r\n");
#endif
    (void)PRINTF("\r\n");

    (void)PRINTF("\tWhere\r\n");
    (void)PRINTF("\t<format> - This parameter specifies the data rate format used in this command\r\n");
    (void)PRINTF("\t        0:    LG\r\n");
    (void)PRINTF("\t        1:    HT\r\n");
#if CONFIG_11AC
    (void)PRINTF("\t        2:    VHT\r\n");
#endif
#if CONFIG_11AX
    (void)PRINTF("\t        3:    HE\r\n");
#endif
    (void)PRINTF("\t        0xff: Auto\r\n");
    (void)PRINTF("\t<index> - This parameter specifies the rate or MCS index\r\n");
    (void)PRINTF("\tIf <format> is 0 (LG),\r\n");
    (void)PRINTF("\t        0       1 Mbps\r\n");
    (void)PRINTF("\t        1       2 Mbps\r\n");
    (void)PRINTF("\t        2       5.5 Mbps\r\n");
    (void)PRINTF("\t        3       11 Mbps\r\n");
    (void)PRINTF("\t        4       6 Mbps\r\n");
    (void)PRINTF("\t        5       9 Mbps\r\n");
    (void)PRINTF("\t        6       12 Mbps\r\n");
    (void)PRINTF("\t        7       18 Mbps\r\n");
    (void)PRINTF("\t        8       24 Mbps\r\n");
    (void)PRINTF("\t        9       36 Mbps\r\n");
#ifndef RW610
    (void)PRINTF("\t        10      48 Mbps\r\n");
    (void)PRINTF("\t        11      54 Mbps\r\n");
#endif
    (void)PRINTF("\tIf <format> is 1 (HT),\r\n");
    (void)PRINTF("\t        0       MCS0\r\n");
    (void)PRINTF("\t        1       MCS1\r\n");
    (void)PRINTF("\t        2       MCS2\r\n");
    (void)PRINTF("\t        3       MCS3\r\n");
    (void)PRINTF("\t        4       MCS4\r\n");
    (void)PRINTF("\t        5       MCS5\r\n");
    (void)PRINTF("\t        6       MCS6\r\n");
    (void)PRINTF("\t        7       MCS7\r\n");
#if CONFIG_11AC
    (void)PRINTF("\tIf <format> is 2 (VHT),\r\n");
    (void)PRINTF("\t        0       MCS0\r\n");
    (void)PRINTF("\t        1       MCS1\r\n");
    (void)PRINTF("\t        2       MCS2\r\n");
    (void)PRINTF("\t        3       MCS3\r\n");
    (void)PRINTF("\t        4       MCS4\r\n");
    (void)PRINTF("\t        5       MCS5\r\n");
    (void)PRINTF("\t        6       MCS6\r\n");
    (void)PRINTF("\t        7       MCS7\r\n");
    (void)PRINTF("\t        8       MCS8\r\n");
#ifndef RW610
    (void)PRINTF("\t        9       MCS9\r\n");
#endif
#endif
#if CONFIG_11AX
    (void)PRINTF("\tIf <format> is 3 (HE),\r\n");
    (void)PRINTF("\t        0       MCS0\r\n");
    (void)PRINTF("\t        1       MCS1\r\n");
    (void)PRINTF("\t        2       MCS2\r\n");
    (void)PRINTF("\t        3       MCS3\r\n");
    (void)PRINTF("\t        4       MCS4\r\n");
    (void)PRINTF("\t        5       MCS5\r\n");
    (void)PRINTF("\t        6       MCS6\r\n");
    (void)PRINTF("\t        7       MCS7\r\n");
    (void)PRINTF("\t        8       MCS8\r\n");
    (void)PRINTF("\t        9       MCS9\r\n");
#ifndef RW610
    (void)PRINTF("\t        10      MCS10\r\n");
    (void)PRINTF("\t        11      MCS11\r\n");
#endif
#endif
#if (CONFIG_11AX) || (CONFIG_11AC)
    (void)PRINTF("\t<nss> - This parameter specifies the NSS. It is valid only for VHT and HE\r\n");
    (void)PRINTF("\tIf <format> is 2 (VHT) or 3 (HE),\r\n");
    (void)PRINTF("\t        1       NSS1\r\n");
#ifndef RW610
    (void)PRINTF("\t        2       NSS2\r\n");
#endif
#endif
#if CONFIG_11AC
    (void)PRINTF("\t<rate_setting> - This parameter can only specifies the GI types now.\r\n");
    (void)PRINTF("\tIf <format> is 1 (HT),\r\n");
    (void)PRINTF("\t        0x0000  Long GI\r\n");
    (void)PRINTF("\t        0x0020  Short GI\r\n");
    (void)PRINTF("\tIf <format> is 2 (VHT),\r\n");
    (void)PRINTF("\t        0x0000  Long GI\r\n");
    (void)PRINTF("\t        0x0020  Short GI\r\n");
    (void)PRINTF("\t        0x0060  Short GI and Nsym mod 10=9\r\n");
#endif
#if CONFIG_11AX
    (void)PRINTF("\tIf <format> is 3 (HE),\r\n");
    (void)PRINTF("\t        0x0000  1xHELTF + GI0.8us\r\n");
    (void)PRINTF("\t        0x0020  2xHELTF + GI0.8us\r\n");
    (void)PRINTF("\t        0x0040  2xHELTF + GI1.6us\r\n");
    (void)PRINTF("\t        0x0060  4xHELTF + GI0.8us if DCM = 1 and STBC = 1\r\n");
    (void)PRINTF("\t                4xHELTF + GI3.2us, otherwise\r\n");
#endif
#if CONFIG_AUTO_NULL_TX
    (void)PRINTF(
        "\t<autoTx_set> - This parameter specifies whether only fix auto tx data rate, this parameter is optional "
        "\r\n");
    (void)PRINTF("\t        0:    not fix auto tx data rate\r\n");
    (void)PRINTF("\t        1:    only fix auto tx data rate\r\n");
#endif
}

static void test_wlan_set_txratecfg(int argc, char **argv)
{
    mlan_bss_type bss_type = (mlan_bss_type)0;
    wlan_ds_rate ds_rate;
#if CONFIG_11AX
    wlan_txrate_setting *rate_setting = NULL;
#endif
    int rv = WM_SUCCESS;

    if (argc < 3 ||
#if (CONFIG_11AC) || (CONFIG_11AX)
            argc > 7)
    {
#else
            argc > 5)
    {
#endif
        (void)PRINTF("Invalid arguments\r\n");
        goto done;
    }

    if (string_equal("sta", argv[1]))
        bss_type = MLAN_BSS_TYPE_STA;
    else if (string_equal("uap", argv[1]))
        bss_type = MLAN_BSS_TYPE_UAP;
    else
    {
        (void)PRINTF("Invalid bss type selection\r\n");
        goto done;
    }

    (void)memset(&ds_rate, 0, sizeof(wlan_ds_rate));
#if CONFIG_AUTO_NULL_TX
    ds_rate.auto_null_fixrate_enable = 0xff;
#endif

    ds_rate.sub_command = WIFI_DS_RATE_CFG;

    errno                              = 0;
    ds_rate.param.rate_cfg.rate_format = (mlan_rate_format)(strtol(argv[2], NULL, 0));
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
    errno                             = 0;
    ds_rate.param.rate_cfg.rate_index = (t_u32)strtol(argv[3], NULL, 0);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
#if (CONFIG_11AC) || (CONFIG_11AX)
    if (argc >= 5)
    {
        errno                      = 0;
        ds_rate.param.rate_cfg.nss = strtol(argv[4], NULL, 0);
        if (errno != 0)
        {
            (void)PRINTF("Error during strtoul errno:%d", errno);
        }
        else
        {
            /*Do Nothing*/
        }
    }

    if (argc >= 6)
    {
        errno                               = 0;
        ds_rate.param.rate_cfg.rate_setting = strtol(argv[5], NULL, 0);
        if (errno != 0)
            (void)PRINTF("Error during strtoul errno:%d", errno);
    }
    else
    {
        errno                               = 0;
        ds_rate.param.rate_cfg.rate_setting = 0xffff;
        if (errno != 0)
            (void)PRINTF("Error during strtoul errno:%d", errno);
    }
#else
    ds_rate.param.rate_cfg.rate_setting = 0xffff;
#endif
#if CONFIG_AUTO_NULL_TX
#if (CONFIG_11AC) || (CONFIG_11AX)
    if (argc == 7)
#else
    if (argc == 5)
#endif
    {
        errno                            = 0;
        ds_rate.auto_null_fixrate_enable = strtol(argv[argc - 1], NULL, 0);
        ;
        if (ds_rate.auto_null_fixrate_enable > 1)
        {
            ds_rate.auto_null_fixrate_enable = 0xff;
            (void)PRINTF("Invalid autoTx_only selection\r\n");
            goto done;
        }
    }
#endif

    if ((ds_rate.param.rate_cfg.rate_format != MLAN_RATE_FORMAT_AUTO)
#if (CONFIG_11AX)
            && (ds_rate.param.rate_cfg.rate_format > MLAN_RATE_FORMAT_HE)
#elif (CONFIG_11AC)
            && (ds_rate.param.rate_cfg.rate_format > MLAN_RATE_FORMAT_VHT)
#else
            && (ds_rate.param.rate_cfg.rate_format > MLAN_RATE_FORMAT_HT)
#endif
       )
    {
        (void)PRINTF("Invalid format selection\r\n");
        goto done;
    }

    if (ds_rate.param.rate_cfg.rate_format != MLAN_RATE_FORMAT_AUTO)
    {
        if (((ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_LG) &&
                (ds_rate.param.rate_cfg.rate_index > MLAN_RATE_INDEX_OFDM7))
                || ((ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_HT) &&
                    (ds_rate.param.rate_cfg.rate_index != 32U) &&
                    (ds_rate.param.rate_cfg.rate_index > 7U)
                   )
#if CONFIG_11AC
                || ((ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_VHT) &&
#ifndef RW610
                    (ds_rate.param.rate_cfg.rate_index > MLAN_RATE_INDEX_MCS9))
#else
                    (ds_rate.param.rate_cfg.rate_index > MLAN_RATE_INDEX_MCS8))
#endif
#endif
#if CONFIG_11AX
                || ((ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_HE) &&
#ifndef RW610
                    (ds_rate.param.rate_cfg.rate_index > MLAN_RATE_INDEX_MCS11))
#else
                    (ds_rate.param.rate_cfg.rate_index > MLAN_RATE_INDEX_MCS9))
#endif
#endif
           )
        {
            (void)PRINTF("Invalid index selection\r\n");
            goto done;
        }
#if (CONFIG_11AC) || (CONFIG_11AX)
#ifndef RW610
        /* NSS is supported up to 2 */
        if ((ds_rate.param.rate_cfg.nss <= 0) || (ds_rate.param.rate_cfg.nss >= 3))
#else
        /* NSS is supported up to 1 */
        if ((ds_rate.param.rate_cfg.nss <= 0) || (ds_rate.param.rate_cfg.nss >= 2))
#endif
        {
            (void)PRINTF("Invalid nss selection\r\n");
            goto done;
        }
#endif

        if (argc >= 6)
        {
#if CONFIG_11AX
            /* HE Preamble type */
// #define HE_SU_PREAMBLE 0
#define HE_ER_PREAMBLE 1

            /* HE ER SU Type */
#define HE_ER_SU_BANDWIDTH_TONE242 0
#define HE_ER_SU_BANDWIDTH_TONE106 1

            rate_setting = (wlan_txrate_setting *)&ds_rate.param.rate_cfg.rate_setting;

#ifdef RW610
            if (ds_rate.param.rate_cfg.rate_setting != 0xffff)
            {
                if (rate_setting->stbc != 0)
                {
                    (void)PRINTF("Invalid STBC setting\r\n");
                    (void)PRINTF("This chip does not support STBC\r\n");
                    goto done;
                }
                if (rate_setting->adv_coding != 0)
                {
                    (void)PRINTF("Invalid coding setting\r\n");
                    (void)PRINTF("This chip does not support LDPC\r\n");
                    goto done;
                }
                if (ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_HE && rate_setting->preamble == HE_ER_PREAMBLE)
                {
                    if (rate_setting->bandwidth > HE_ER_SU_BANDWIDTH_TONE106)
                    {
                        (void)PRINTF("Invalid BW setting for this extended rate\r\n");
                        (void)PRINTF("This is 20MHz only chip\r\n");
                        goto done;
                    }
                }
                else
                {
                    if (rate_setting->bandwidth != 0)
                    {
                        (void)PRINTF("Invalid BW setting\r\n");
                        (void)PRINTF("This is 20MHz only chip\r\n");
                        goto done;
                    }
                }
            }
#endif
            if (ds_rate.param.rate_cfg.rate_format == MLAN_RATE_FORMAT_HE)
            {
                if (rate_setting->preamble == HE_ER_PREAMBLE)
                {
                    if (rate_setting->bandwidth == HE_ER_SU_BANDWIDTH_TONE242)
                    {
                        if ((ds_rate.param.rate_cfg.rate_index > MLAN_RATE_INDEX_MCS2) ||
                                (ds_rate.param.rate_cfg.nss > MLAN_RATE_NSS1))
                        {
                            (void)PRINTF("Invalid rate and MCS or NSS configuration for 242 tone\r\n");
                            goto done;
                        }
                    }
                    else if (rate_setting->bandwidth == HE_ER_SU_BANDWIDTH_TONE106)
                    {
                        if ((ds_rate.param.rate_cfg.rate_index != MLAN_RATE_INDEX_MCS0) ||
                                (ds_rate.param.rate_cfg.nss != MLAN_RATE_NSS1))
                        {
                            (void)PRINTF("Invalid rate and MCS or NSS configuration for 106 tone\r\n");
                            goto done;
                        }
                    }
                    else
                    {
                        (void)PRINTF("Invalid Bandwidth for HE ER Preamble\r\n");
                        goto done;
                    }
                }
                if ((rate_setting->dcm) && (rate_setting->stbc == 0))
                {
                    if ((ds_rate.param.rate_cfg.rate_index == MLAN_RATE_INDEX_MCS2) ||
                            (ds_rate.param.rate_cfg.rate_index > MLAN_RATE_INDEX_MCS4))
                    {
                        (void)PRINTF("Invalid MCS configuration if DCM is supported\r\n");
                        goto done;
                    }
                }
            }
#endif
        }
    }

    rv = wlan_set_txratecfg(ds_rate, bss_type);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set txratecfg\r\n");
        goto done;
    }
    (void)PRINTF("Configured txratecfg as below:\r\n");
    print_ds_rate(ds_rate);
    return;

done:
    dump_wlan_set_txratecfg_usage();
}

static void test_wlan_get_txratecfg(int argc, char **argv)
{
    mlan_bss_type bss_type = (mlan_bss_type)0;
    wlan_ds_rate ds_rate;

    if (argc != 2)
    {
        (void)PRINTF("Invalid arguments\r\n");
        (void)PRINTF("Usage: wlan-get-txratecfg <sta/uap>\r\n");
        return;
    }

    if (string_equal("sta", argv[1]))
        bss_type = MLAN_BSS_TYPE_STA;
    else if (string_equal("uap", argv[1]))
        bss_type = MLAN_BSS_TYPE_UAP;
    else
    {
        (void)PRINTF("Invalid bss type selection\r\n");
        return;
    }

    (void)memset(&ds_rate, 0, sizeof(wlan_ds_rate));

    ds_rate.sub_command = WIFI_DS_RATE_CFG;

    int rv = wlan_get_txratecfg(&ds_rate, bss_type);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get tx rate cfg\r\n");
        return;
    }

    print_ds_rate(ds_rate);
}

static void test_wlan_get_data_rate(int argc, char **argv)
{
    mlan_bss_type bss_type = (mlan_bss_type)0;
    wlan_ds_rate ds_rate;

    if (argc != 2)
    {
        (void)PRINTF("Invalid arguments\r\n");
        (void)PRINTF("Usage: wlan-get-data-rate <sta/uap>\r\n");
        return;
    }

    if (string_equal("sta", argv[1]))
        bss_type = MLAN_BSS_TYPE_STA;
    else if (string_equal("uap", argv[1]))
        bss_type = MLAN_BSS_TYPE_UAP;
    else
    {
        (void)PRINTF("Invalid bss type selection\r\n");
        return;
    }

    (void)memset(&ds_rate, 0, sizeof(wlan_ds_rate));

    ds_rate.sub_command = WIFI_DS_GET_DATA_RATE;

    int rv = wlan_get_data_rate(&ds_rate, bss_type);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get tx rate cfg\r\n");
        return;
    }

    print_ds_rate(ds_rate);
}

void print_txpwrlimit(wlan_txpwrlimit_t *txpwrlimit)
{
    int i, j;

    (void)PRINTF("--------------------------------------------------------------------------------\r\n");
    (void)PRINTF("Get txpwrlimit: sub_band=%x \r\n", txpwrlimit->subband);
    for (i = 0; i < txpwrlimit->num_chans; i++)
    {
        (void)PRINTF("StartFreq: %d\r\n", txpwrlimit->txpwrlimit_config[i].chan_desc.start_freq);
        (void)PRINTF("ChanWidth: %d\r\n", txpwrlimit->txpwrlimit_config[i].chan_desc.chan_width);
        (void)PRINTF("ChanNum:   %d\r\n", txpwrlimit->txpwrlimit_config[i].chan_desc.chan_num);
        (void)PRINTF("Pwr:");
        for (j = 0; j < txpwrlimit->txpwrlimit_config[i].num_mod_grps; j++)
        {
            if (j == (txpwrlimit->txpwrlimit_config[i].num_mod_grps - 1))
                (void)PRINTF("%d,%d", txpwrlimit->txpwrlimit_config[i].txpwrlimit_entry[j].mod_group,
                             txpwrlimit->txpwrlimit_config[i].txpwrlimit_entry[j].tx_power);
            else
                (void)PRINTF("%d,%d,", txpwrlimit->txpwrlimit_config[i].txpwrlimit_entry[j].mod_group,
                             txpwrlimit->txpwrlimit_config[i].txpwrlimit_entry[j].tx_power);
        }
        (void)PRINTF("\r\n");
    }
    (void)PRINTF("\r\n");
}

static void print_chanlist(wlan_chanlist_t chanlist)
{
    unsigned char i;

    (void)PRINTF("--------------------------------------------------------------------------------\r\n");
    (void)PRINTF("Number of channels configured: %d\r\n", chanlist.num_chans);
    (void)PRINTF("\r\n");
    for (i = 0; i < chanlist.num_chans; i++)
    {
        (void)PRINTF("ChanNum: %d\t", chanlist.chan_info[i].chan_num);
        (void)PRINTF("ChanFreq: %d\t", chanlist.chan_info[i].chan_freq);
        (void)PRINTF("%s", chanlist.chan_info[i].passive_scan_or_radar_detect ? "Passive" : "Active");
        (void)PRINTF("\r\n");
    }
}

static void dump_wlan_get_txpwrlimit_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-txpwrlimit <subband> \r\n");
    (void)PRINTF("\r\n");
    (void)PRINTF("\t Where subband is: \r\n");
    (void)PRINTF("\t       0x00 2G subband  (2.4G: channel 1-14)\r\n");
#if CONFIG_5GHz_SUPPORT
    (void)PRINTF("\t       0x10 5G subband0 (5G: channel 36,40,44,48,\r\n");
    (void)PRINTF("\t                                     52,56,60,64)\r\n");
    (void)PRINTF("\t       0x11 5G subband1 (5G: channel 100,104,108,112,\r\n");
    (void)PRINTF("\t                                     116,120,124,128,\r\n");
    (void)PRINTF("\t                                     132,136,140,144)\r\n");
    (void)PRINTF("\t       0x12 5G subband2 (5G: channel 149,153,157,161,165,172)\r\n");
    (void)PRINTF("\t       0x13 5G subband3 (5G: channel 183,184,185,187,188,\r\n");
    (void)PRINTF("\t                                     189, 192,196;\r\n");
    (void)PRINTF("\t                         5G: channel 7,8,11,12,16,34)\r\n");
#endif
}

static void test_wlan_get_txpwrlimit(int argc, char **argv)
{
    wifi_SubBand_t subband;
    wlan_txpwrlimit_t *txpwrlimit = NULL;

    ARG_UNUSED(chanlist_2g_cfg);
#if CONFIG_5GHz_SUPPORT
    ARG_UNUSED(chanlist_5g_cfg);
#endif

    if (argc != 2)
    {
        dump_wlan_get_txpwrlimit_usage();
        return;
    }

    errno   = 0;
    subband = (wifi_SubBand_t)strtol(argv[1], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }

    if (subband != SubBand_2_4_GHz
#if CONFIG_5GHz_SUPPORT
            && subband != SubBand_5_GHz_0 && subband != SubBand_5_GHz_1 && subband != SubBand_5_GHz_2 &&
            subband != SubBand_5_GHz_3
#endif
       )
    {
        dump_wlan_get_txpwrlimit_usage();
        return;
    }
#if !CONFIG_MEM_POOLS
    txpwrlimit = OSA_MemoryAllocate(sizeof(wlan_txpwrlimit_t));
#else
    txpwrlimit = (wlan_txpwrlimit_t *)OSA_MemoryPoolAllocate(buf_2048_MemoryPool);
#endif
    if (txpwrlimit == NULL)
    {
        (void)PRINTF("Cannot allocate memory\r\n");
        return;
    }

    int rv = wlan_get_txpwrlimit(subband, txpwrlimit);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get TX PWR Limit configuration\r\n");
    }
    else
    {
        print_txpwrlimit(txpwrlimit);
    }
#if !CONFIG_MEM_POOLS
    OSA_MemoryFree(txpwrlimit);
#else
    OSA_MemoryPoolFree(buf_2048_MemoryPool, txpwrlimit);
#endif
}

#if !CONFIG_COMPRESS_TX_PWTBL

static void test_wlan_set_txpwrlimit(int argc, char **argv)
{
    wlan_txpwrlimit_t *txpwrlimit = NULL;

#if !CONFIG_MEM_POOLS
    txpwrlimit = OSA_MemoryAllocate(sizeof(wlan_txpwrlimit_t));
#else
    txpwrlimit = (wlan_txpwrlimit_t *)OSA_MemoryPoolAllocate(buf_2048_MemoryPool);
#endif

    if (txpwrlimit == NULL)
    {
        (void)PRINTF("Cannot allocate memory\r\n");
        return;
    }

    int rv = wlan_set_txpwrlimit(&tx_pwrlimit_2g_cfg);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set 2G TX PWR Limit configuration\r\n");
    }
    else
    {
#if CONFIG_5GHz_SUPPORT
        rv = wlan_set_txpwrlimit(&tx_pwrlimit_5g_cfg);
        if (rv != WM_SUCCESS)
        {
            (void)PRINTF("Unable to set 5G TX PWR Limit configuration\r\n");
        }
        else
        {
#endif
            txpwrlimit->subband = SubBand_2_4_GHz;
            rv                  = wlan_get_txpwrlimit(txpwrlimit->subband, txpwrlimit);
            if (rv != WM_SUCCESS)
            {
                (void)PRINTF("Unable to get 2G TX PWR Limit configuration\r\n");
            }
            else
            {
                print_txpwrlimit(txpwrlimit);
            }
#if CONFIG_5GHz_SUPPORT
            txpwrlimit->subband = SubBand_5_GHz_0;
            rv                  = wlan_get_txpwrlimit(txpwrlimit->subband, txpwrlimit);
            if (rv != WM_SUCCESS)
            {
                (void)PRINTF("Unable to get 5G SubBand0 TX PWR Limit configuration\r\n");
            }
            else
            {
                print_txpwrlimit(txpwrlimit);
            }
            txpwrlimit->subband = SubBand_5_GHz_1;
            rv                  = wlan_get_txpwrlimit(txpwrlimit->subband, txpwrlimit);
            if (rv != WM_SUCCESS)
            {
                (void)PRINTF("Unable to get 5G SubBand1 TX PWR Limit configuration\r\n");
            }
            else
            {
                print_txpwrlimit(txpwrlimit);
            }
            txpwrlimit->subband = SubBand_5_GHz_2;
            rv                  = wlan_get_txpwrlimit(txpwrlimit->subband, txpwrlimit);
            if (rv != WM_SUCCESS)
            {
                (void)PRINTF("Unable to get 5G SubBand2 TX PWR Limit configuration\r\n");
            }
            else
            {
                print_txpwrlimit(txpwrlimit);
            }
        }
#endif
    }
#if !CONFIG_MEM_POOLS
    OSA_MemoryFree(txpwrlimit);
#else
    OSA_MemoryPoolFree(buf_2048_MemoryPool, txpwrlimit);
#endif
}

static void test_wlan_set_chanlist_and_txpwrlimit(int argc, char **argv)
{
    wlan_txpwrlimit_t *txpwrlimit = NULL;

#if !CONFIG_MEM_POOLS
    txpwrlimit = OSA_MemoryAllocate(sizeof(wlan_txpwrlimit_t));
#else
    txpwrlimit = (wlan_txpwrlimit_t *)OSA_MemoryPoolAllocate(buf_2048_MemoryPool);
#endif
    if (txpwrlimit == NULL)
    {
        (void)PRINTF("Cannot allocate memory\r\n");
        return;
    }

    int rv = wlan_set_chanlist_and_txpwrlimit(&chanlist_2g_cfg, &tx_pwrlimit_2g_cfg);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set 2G TX PWR Limit configuration\r\n");
    }
    else
    {
#if CONFIG_5GHz_SUPPORT
        rv = wlan_set_chanlist_and_txpwrlimit(&chanlist_5g_cfg, &tx_pwrlimit_5g_cfg);
        if (rv != WM_SUCCESS)
        {
            (void)PRINTF("Unable to set 5G TX PWR Limit configuration\r\n");
        }
        else
        {
#endif
            txpwrlimit->subband = SubBand_2_4_GHz;
            rv                  = wlan_get_txpwrlimit(txpwrlimit->subband, txpwrlimit);
            if (rv != WM_SUCCESS)
            {
                (void)PRINTF("Unable to get 2G TX PWR Limit configuration\r\n");
            }
            else
            {
                print_txpwrlimit(txpwrlimit);
            }
#if CONFIG_5GHz_SUPPORT
            txpwrlimit->subband = SubBand_5_GHz_0;
            rv                  = wlan_get_txpwrlimit(txpwrlimit->subband, txpwrlimit);
            if (rv != WM_SUCCESS)
            {
                (void)PRINTF("Unable to get 5G SubBand0 TX PWR Limit configuration\r\n");
            }
            else
            {
                print_txpwrlimit(txpwrlimit);
            }
            txpwrlimit->subband = SubBand_5_GHz_1;
            rv                  = wlan_get_txpwrlimit(txpwrlimit->subband, txpwrlimit);
            if (rv != WM_SUCCESS)
            {
                (void)PRINTF("Unable to get 5G SubBand1 TX PWR Limit configuration\r\n");
            }
            else
            {
                print_txpwrlimit(txpwrlimit);
            }
            txpwrlimit->subband = SubBand_5_GHz_2;
            rv                  = wlan_get_txpwrlimit(txpwrlimit->subband, txpwrlimit);
            if (rv != WM_SUCCESS)
            {
                (void)PRINTF("Unable to get 5G SubBand2 TX PWR Limit configuration\r\n");
            }
            else
            {
                print_txpwrlimit(txpwrlimit);
            }
        }
#endif
        wlan_chanlist_t chanlist;

        (void)memset(&chanlist, 0x00, sizeof(wlan_chanlist_t));
        rv = wlan_get_chanlist(&chanlist);
        if (rv != WM_SUCCESS)
        {
            (void)PRINTF("Unable to get channel list configuration\r\n");
        }
        else
        {
            print_chanlist(chanlist);
        }
    }
#if !CONFIG_MEM_POOLS
    OSA_MemoryFree(txpwrlimit);
#else
    OSA_MemoryPoolFree(buf_2048_MemoryPool, txpwrlimit);
#endif
}
#endif

static void test_wlan_set_chanlist(int argc, char **argv)
{
    wlan_chanlist_t chanlist;

#if (CONFIG_COMPRESS_TX_PWTBL) && !defined(RW610)
    ARG_UNUSED(rg_table_fc);
    ARG_UNUSED(rg_table_fc_len);
#endif

#if (CONFIG_COMPRESS_TX_PWTBL) && defined(RW610)
    ARG_UNUSED(tx_pwrlimit_2g_cfg);
    ARG_UNUSED(chanlist_2g_cfg);
#if CONFIG_5GHz_SUPPORT
    ARG_UNUSED(tx_pwrlimit_5g_cfg);
    ARG_UNUSED(chanlist_5g_cfg);
#endif
#endif

    (void)memset(&chanlist, 0x00, sizeof(wlan_chanlist_t));

    /* Get channel list of current region */
    int ret = wlan_get_chanlist(&chanlist);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get channel list of current region\r\n");
        return;
    }
    ret = wlan_set_chanlist(&chanlist);
    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Failed to set channel list!\r\n");
        return;
    }
    print_chanlist(chanlist);
}

static void test_wlan_get_chanlist(int argc, char **argv)
{
    wlan_chanlist_t chanlist;

    (void)memset(&chanlist, 0x00, sizeof(wlan_chanlist_t));
    int rv = wlan_get_chanlist(&chanlist);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get channel list configuration\r\n");
    }
    else
    {
        print_chanlist(chanlist);
    }
}

#if CONFIG_11AX
static void dump_wlan_set_txomi_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-tx-omi <interface> <tx-omi> <tx-option> <num_data_pkts>\r\n");
    (void)PRINTF("where, interface = uap or sta\r\n");
    (void)PRINTF("where, tx-omi =\r\n");
    (void)PRINTF("\t Bit 0-2: Rx NSS\r\n");
    (void)PRINTF("\t Bit 3-4: Channel Width. 0: 20MHz  1: 40MHz  2: 80MHz\r\n");
    (void)PRINTF("\t Bit 5  : UL MU Disable\r\n");
    (void)PRINTF("\t Bit 6-8: Tx NSTS (applies to client mode only)\r\n");
    (void)PRINTF("\t Bit 9  : ER SU Disable\r\n");
    (void)PRINTF("\t Bit 10 : DL MU-MIMO Resound Recommendation\r\n");
    (void)PRINTF("\t Bit 11 : DL MU Data Disable\r\n");
    (void)PRINTF("\t Example : For 1x1 SoC, to set bandwidth,\r\n");
    (void)PRINTF("20M, tx-omi = 0x00\r\n");
    (void)PRINTF("40M, tx-omi = 0x08\r\n");
    (void)PRINTF("80M, tx-omi = 0x10\r\n");
    (void)PRINTF("where, tx-option =\r\n");
    (void)PRINTF("\t 0: send OMI in NULL Data\r\n");
    (void)PRINTF("\t 1: send OMI in QoS Data\r\n");
    (void)PRINTF("\t 0xff: send OMI in either QoS Data or NULL Data\r\n");
    (void)PRINTF("where, num_data_pkts =\r\n");
    (void)PRINTF("\t Set this value only when tx-option is 1 or 0xff.\r\n");
    (void)PRINTF("\t Minimum value is 1\r\n");
    (void)PRINTF("\t Maximum value is 16\r\n");
    (void)PRINTF("\t num_data_pkts is applied only if OMI is sent in QoS data frame\r\n");
    (void)PRINTF("\t It specifies the number of consecutive data frames containing the OMI\r\n");
}


static void test_wlan_set_rutxpwrlimit(int argc, char **argv)
{
    int rv;
#if (CONFIG_COMPRESS_RU_TX_PWTBL) && (CONFIG_11AX)
#ifdef RW610
    if (argc != 2)
    {
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("wlan-set-rutxpwrlimit <region-code>\r\n");
        return;
    }

    t_u16 region_code = (t_u16)strtol(argv[1], NULL, 0);
    rv = wlan_set_ru_power_cfg(region_code);
#else
    rv = wlan_set_11ax_rutxpowerlimit(rutxpowerlimit_cfg_set, sizeof(rutxpowerlimit_cfg_set));
#endif

    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set RU TX PWR Limit configuration\r\n");
    }
#else
    rv = wlan_set_11ax_rutxpowerlimit_legacy(&rutxpowerlimit_2g_cfg_set);
    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set 2G RU TX PWR Limit configuration\r\n");
    }
#if CONFIG_5GHz_SUPPORT
    else
    {
        rv = wlan_set_11ax_rutxpowerlimit_legacy(&rutxpowerlimit_5g_cfg_set);
        if (rv != WM_SUCCESS)
        {
            (void)PRINTF("Unable to set 5G RU TX PWR Limit configuration\r\n");
        }
    }
#endif /* CONFIG_5GHz_SUPPORT */
#endif /* (CONFIG_COMPRESS_RU_TX_PWTBL) && (CONFIG_11AX) */
}


static void test_wlan_set_tx_omi(int argc, char **argv)
{
    int ret;
    t_u8 interface = 0;
    uint16_t tx_omi;
    uint8_t tx_option     = 0;
    uint8_t num_data_pkts = 0;

    if (argc != 4 && argc != 5)
    {
        dump_wlan_set_txomi_usage();
        return;
    }

    errno = 0;

    tx_omi    = (uint16_t)strtol(argv[2], NULL, 0);
    tx_option = (uint8_t)strtol(argv[3], NULL, 0);

    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
        errno = 0;
    }

    if (argc == 4)
    {
        if (tx_option != 0)
        {
            dump_wlan_set_txomi_usage();
            return;
        }
        num_data_pkts = 1;
    }
    else
    {
        if (tx_option == 0)
        {
            dump_wlan_set_txomi_usage();
            return;
        }
        num_data_pkts = (uint8_t)strtol(argv[4], NULL, 0);

        if (errno != 0)
            (void)PRINTF("Error during strtoul errno:%d", errno);

        if (num_data_pkts < 1 || num_data_pkts > 16)
        {
            dump_wlan_set_txomi_usage();
            return;
        }
    }

    if (string_equal("sta", argv[1]))
    {
        interface = (t_u8)WLAN_BSS_TYPE_STA;
    }
    else if (string_equal("uap", argv[1]))
    {
        interface = (t_u8)WLAN_BSS_TYPE_UAP;
    }

    ret = wlan_set_11ax_tx_omi(interface, tx_omi, tx_option, num_data_pkts);

    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("TX OMI: 0x%x set\r\n", tx_omi);
        (void)PRINTF("TX OPTION: 0x%x set\r\n", tx_option);
        (void)PRINTF("TX NUM_DATA_PKTS: 0x%x set\r\n", num_data_pkts);
    }
    else
    {
        (void)PRINTF("Unable to set TX OMI: 0x%x\r\n", tx_omi);
        (void)PRINTF("Unable to set TX OPTION: 0x%x\r\n", tx_option);
        (void)PRINTF("Unable to set TX NUM_DATA_PKTS: 0x%x\r\n", num_data_pkts);
    }
}

static void dump_wlan_set_tol_time_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("set OBSS Narrow Bandwidth RU Tolerance Time\r\n");
    (void)PRINTF("Pls set toltime when sta is in disconnect state.\r\n");
    (void)PRINTF("wlan-set-toltime value\r\n");
    (void)PRINTF("value:\r\n");
    (void)PRINTF("Valid range[1..3600]\r\n");
}

static void test_wlan_set_toltime(int argc, char **argv)
{
    unsigned int value;
    int ret;
    if (argc != 2)
    {
        (void)PRINTF("Error: invalid number of arguments\r\n");
        dump_wlan_set_tol_time_usage();
        return;
    }

    if (get_uint(argv[1], &value, strlen(argv[1])))
    {
        (void)PRINTF("Error: invalid option argument\r\n");
        dump_wlan_set_tol_time_usage();
        return;
    }

    ret = wlan_set_11ax_tol_time(value);

    if (ret != WM_SUCCESS)
    {
        (void)PRINTF("Error: Failed to set Tolerance Time.\r\n");
        dump_wlan_set_tol_time_usage();
        return;
    }
}

static wlan_11ax_config_t ax_conf;
#if CONFIG_11AX_TWT
    static wlan_twt_setup_config_t twt_setup_conf;
    static wlan_twt_teardown_config_t teardown_conf;
    static wlan_btwt_config_t btwt_config;
#endif /* CONFIG_11AX_TWT */

/* cfg tables for 11axcfg and twt commands to FW */
static uint8_t g_11ax_cfg[31] = {0};

const static test_cfg_param_t g_11ax_cfg_param[] =
{
    /* name                 offset  len     notes */
    {"band", 0, 1, NULL},
    {"cap_id", 1, 2, NULL},
    {"cap_len", 3, 2, NULL},
    {"he_cap_id", 5, 1, NULL},
    {"he_mac_cap_info", 6, 6, NULL},
    {"he_phy_cap_info", 12, 11, NULL},
    {"he_mcs_nss_support", 23, 4, NULL},
    {"pe", 27, 2, NULL},
};

#if CONFIG_11AX_TWT
static uint8_t g_btwt_cfg[12] = {0};

const static test_cfg_param_t g_btwt_cfg_param[] =
{
    /* name             offset  len   notes */
    {"action", 0, 2, "only support 1: Set"},
    {"sub_id", 2, 2, "Broadcast TWT AP config"},
    {"nominal_wake", 4, 1, "range 64-255"},
    {"max_sta_support", 5, 1, "Max STA Support"},
    {"twt_mantissa", 6, 2, NULL},
    {"twt_offset", 8, 2, NULL},
    {"twt_exponent", 10, 1, NULL},
    {"sp_gap", 11, 1, NULL},
};

static uint8_t g_twt_setup_cfg[12] = {0};

static test_cfg_param_t g_twt_setup_cfg_param[] =
{
    /* name                 offset  len  notes */
    {"implicit", 0, 1, "0: TWT session is explicit, 1: Session is implicit"},
    {"announced", 1, 1, "0: Unannounced, 1: Announced TWT"},
    {"trigger_enabled", 2, 1, "0: Non-Trigger enabled, 1: Trigger enabled TWT"},
    {"twt_info_disabled", 3, 1, "0: TWT info enabled, 1: TWT info disabled"},
    {"negotiation_type", 4, 1, "0: Individual TWT, 3: Broadcast TWT"},
    {"twt_wakeup_duration", 5, 1, "time after which the TWT requesting STA can transition to doze state"},
    {"flow_identifier", 6, 1, "Range: [0-7]"},
    {
        "hard_constraint", 7, 1,
        "0: FW can tweak the TWT setup parameters if it is rejected by AP, 1: FW should not tweak any parameters"
    },
    {"twt_exponent", 8, 1, "Range: [0-63]"},
    {"twt_mantissa", 9, 2, "Range: [0-sizeof(UINT16)]"},
    {"twt_request", 11, 1, "Type, 0: REQUEST_TWT, 1: SUGGEST_TWT"},
};

static uint8_t g_twt_teardown_cfg[3] = {0};

static test_cfg_param_t g_twt_teardown_cfg_param[] =
{
    /* name             offset  len  notes */
    {"FlowIdentifier", 0, 1, "Range: [0-7]"},
    {"NegotiationType", 1, 1, "0: Future Individual TWT SP start time, 1: Next Wake TBTT tim"},
    {"TearDownAllTWT", 2, 1, "1: To teardown all TWT, 0 otherwise"},
};
#endif /* CONFIG_11AX_TWT */

static void test_wlan_11ax_cfg(int argc, char **argv)
{
    test_wlan_cfg_process(TEST_WLAN_11AX_CFG, argc, argv);
}

#if CONFIG_11AX_TWT
static void test_wlan_bcast_twt(int argc, char **argv)
{
    test_wlan_cfg_process(TEST_WLAN_BCAST_TWT, argc, argv);
}

static void test_wlan_twt_setup(int argc, char **argv)
{
    test_wlan_cfg_process(TEST_WLAN_TWT_SETUP, argc, argv);
}

static void test_wlan_twt_teardown(int argc, char **argv)
{
    test_wlan_cfg_process(TEST_WLAN_TWT_TEARDOWN, argc, argv);
}

static void test_wlan_twt_report(int argc, char **argv)
{
    int i;
    int j;
    int num;
    wlan_twt_report_t info;

    memset(&info, 0x00, sizeof(info));
    wlan_get_twt_report(&info);

    num = info.length / WLAN_BTWT_REPORT_LEN;
    num = num <= WLAN_BTWT_REPORT_MAX_NUM ? num : WLAN_BTWT_REPORT_MAX_NUM;

    (void)PRINTF("twt_report len %hu, num %d, info:\r\n", info.length, num);
    for (i = 0; i < num; i++)
    {
        (void)PRINTF("id[%d]:\r\n", i);
        for (j = 0; j < WLAN_BTWT_REPORT_LEN; j++)
        {
            (void)PRINTF(" 0x%02x", info.data[i * WLAN_BTWT_REPORT_LEN + j]);
        }
        (void)PRINTF("\r\n");
    }
}

#endif /* CONFIG_11AX_TWT */

static void wlan_init_g_test_cfg_arrays()
{
    memcpy(g_11ax_cfg, wlan_get_11ax_cfg(), 31);
#if CONFIG_11AX_TWT
    memcpy(g_btwt_cfg, wlan_get_btwt_cfg(), 12);
    memcpy(g_twt_setup_cfg, wlan_get_twt_setup_cfg(), 12);
    memcpy(g_twt_teardown_cfg, wlan_get_twt_teardown_cfg(), 3);
#endif /* CONFIG_11AX_TWT */
}

/*
 *  Cfg table for mutiple params commands in freeRTOS.
 *  name:          cfg name
 *  data:          cfg data stored and prepared to send
 *  total_len:     len of cfg data
 *  param_list:    param list of cfg data
 *  param_num:     number of cfg param list
 */
static test_cfg_table_t g_test_cfg_table_list[] =   /*  name         data           total_len    param_list param_num*/
{
    {"11axcfg", g_11ax_cfg, 29, g_11ax_cfg_param, 8},
#if CONFIG_11AX_TWT
    {"twt_bcast", g_btwt_cfg, 12, g_btwt_cfg_param, 8},
    {"twt_setup", g_twt_setup_cfg, 12, g_twt_setup_cfg_param, 11},
    {
        "twt_teardown", g_twt_teardown_cfg, 3, g_twt_teardown_cfg_param,
        3
    },
#endif /* CONFIG_11AX_TWT */
    {NULL}
};

static void dump_cfg_data_param(int param_id, uint8_t *data, const test_cfg_param_t *param_cfg)
{
    int i;

    (void)PRINTF("%s ", param_cfg->name);
    if (param_cfg->notes != NULL)
        (void)PRINTF("#### %s\r\n", param_cfg->notes);
    else
        (void)PRINTF("\r\n");

    (void)PRINTF("[%d]: ", param_id);
    for (i = 0; i < param_cfg->len; i++)
    {
        (void)PRINTF("0x%02x ", data[param_cfg->offset + i]);
    }
    (void)PRINTF("\r\n");
}

static void set_cfg_data_param(uint8_t *data, const test_cfg_param_t *param_cfg, char **argv)
{
    int i;

    for (i = 0; i < param_cfg->len; i++)
    {
        data[param_cfg->offset + i] = a2hex(argv[3 + i]);
    }
}

static void dump_cfg_data(test_cfg_table_t *cfg)
{
    int i;
    uint8_t *data = cfg->data;

    (void)PRINTF("cfg[%s] len[%d] param_num[%d]: \r\n", cfg->name, cfg->len, cfg->param_num);
    for (i = 0; i < cfg->param_num; i++)
    {
        dump_cfg_data_param(i, data, &cfg->param_list[i]);
    }
}

static void dump_cfg_help(test_cfg_table_t *cfg)
{
    dump_cfg_data(cfg);
}

/*
 *  match param name and set data by input
 *  argv[0] "wlan-xxxx"
 *  argv[1] "set"
 *  argv[2] param_id
 *  argv[3] param_data_set
 */
static void set_cfg_data(test_cfg_table_t *cfg, int argc, char **argv)
{
    uint8_t *data                     = cfg->data;
    const test_cfg_param_t *param_cfg = NULL;
    int param_id                      = atoi(argv[2]);
    /* input data starts from argv[3] */
    int input_data_num = argc - 3;

    if (param_id < 0 || param_id >= cfg->param_num)
    {
        (void)PRINTF("invalid param index %d\r\n", param_id);
        return;
    }

    param_cfg = &cfg->param_list[param_id];
    if (param_cfg->len != input_data_num)
    {
        (void)PRINTF("invalid input number %d, param has %d u8 arguments\r\n", input_data_num, param_cfg->len);
        return;
    }

    set_cfg_data_param(data, param_cfg, argv);
    dump_cfg_data_param(param_id, data, param_cfg);
}

static void send_cfg_msg(test_cfg_table_t *cfg, uint32_t index)
{
    int ret;

    switch (index)
    {
    case TEST_WLAN_11AX_CFG:
        (void)memcpy((void *)&ax_conf, (void *)cfg->data, sizeof(ax_conf));
        ret = wlan_set_11ax_cfg(&ax_conf);
        break;
#if CONFIG_11AX_TWT
    case TEST_WLAN_BCAST_TWT:
        (void)memcpy((void *)&btwt_config, (void *)cfg->data, sizeof(btwt_config));
        ret = wlan_set_btwt_cfg(&btwt_config);
        break;
    case TEST_WLAN_TWT_SETUP:
        (void)memcpy((void *)&twt_setup_conf, (void *)cfg->data, sizeof(twt_setup_conf));
        ret = wlan_set_twt_setup_cfg(&twt_setup_conf);
        break;
    case TEST_WLAN_TWT_TEARDOWN:
        (void)memcpy((void *)&teardown_conf, (void *)cfg->data, sizeof(teardown_conf));
        ret = wlan_set_twt_teardown_cfg(&teardown_conf);
        break;
#endif /* CONFIG_11AX_TWT */
    default:
        ret = -1;
        break;
    }

    (void)PRINTF("send config [%s] ret %d\r\n", cfg->name, ret);
}

void test_wlan_cfg_process(uint32_t index, int argc, char **argv)
{
    test_cfg_table_t *cfg = NULL;

    /* last cfg table is invalid */
    if (index >= (sizeof(g_test_cfg_table_list) / sizeof(test_cfg_table_t) - 1))
    {
        (void)PRINTF("cfg table too large index %u\r\n", index);
        return;
    }

    cfg = &g_test_cfg_table_list[index];

    if (argc < 2)
    {
        dump_cfg_help(cfg);
        return;
    }

    if (string_equal("help", argv[1]))
        dump_cfg_help(cfg);
    else if (string_equal("dump", argv[1]))
        dump_cfg_data(cfg);
    else if (string_equal("set", argv[1]))
        set_cfg_data(cfg, argc, argv);
    else if (string_equal("done", argv[1]))
        send_cfg_msg(cfg, index);
    else
        (void)PRINTF("unknown argument\r\n");
}

#endif /* CONFIG_11AX */

#if CONFIG_WIFI_CLOCKSYNC
static void dump_wlan_get_tsf_info_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-tsfinfo <tsf_format>\r\n");
    (void)PRINTF("where, tsf_format =\r\n");
    (void)PRINTF("0:    Report GPIO assert TSF\r\n");
    (void)PRINTF("1:    Report Beacon TSF and Offset (valid if CONFIG Mode 2)\r\n");
}

static void test_get_tsf_info(int argc, char **argv)
{
    wlan_tsf_info_t tsf_info;
    (void)memset(&tsf_info, 0, sizeof(wlan_tsf_info_t));
    if (argc != 2)
    {
        dump_wlan_get_tsf_info_usage();
        return;
    }

    errno               = 0;
    tsf_info.tsf_format = (uint16_t)strtol(argv[1], NULL, 0);

    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }

    int rv = wlan_get_tsf_info(&tsf_info);

    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to get TSF info\r\n");
    }
    else
    {
        (void)PRINTF("tsf format:              %d\n\r", tsf_info.tsf_format);
        (void)PRINTF("tsf info:                %d\n\r", tsf_info.tsf_info);
        (void)PRINTF("tsf:                     %llu\n\r", tsf_info.tsf);
        (void)PRINTF("tsf offset:              %d\n\r", tsf_info.tsf_offset);
    }
}

static void dump_wlan_set_clocksync_cfg_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-clocksync <mode> <role> <gpio_pin> <gpio_level> <pulse width>\r\n");
    (void)PRINTF("Set WIFI TSF based clock sync setting. \r\nWhere, \r\n");
    (void)PRINTF("<mode> is use to configure GPIO TSF latch mode\r\n");
    (void)PRINTF("\t\t0:    GPIO level\r\n");
    (void)PRINTF("\t\t1:    GPIO toggle\r\n");
    (void)PRINTF("\t\t2:    GPIO toggle on Next Beacon\r\n");
    (void)PRINTF("<role> \r\n");
    (void)PRINTF("\t\t0: when mode set to 0 or 1\r\n");
    (void)PRINTF("\t\t1:  AP\r\n");
    (void)PRINTF("\t\t2: STA\r\n");
    (void)PRINTF("<gpio pin number>\r\n");
    (void)PRINTF("<GPIO Level/Toggle>\r\n");
    (void)PRINTF("\t\tmode = 0\r\n");
    (void)PRINTF("\t\t0: low    1: high\r\n");
    (void)PRINTF("\t\tmode = 1 or 2\r\n");
    (void)PRINTF("\t\t0: low to high\r\n");
    (void)PRINTF("\t\t1: high to low\r\n");
    (void)PRINTF("GPIO pulse width\r\n");
    (void)PRINTF("\t\tmode = 0,  reserved, set to 0\r\n");
    (void)PRINTF("\t\tmode 1 or 2\r\n");
    (void)PRINTF("\t\t0: GPIO remain on toggle level (high or low)\r\n");
    (void)PRINTF("\t\tNon-0: GPIO pulse width in microseconds (min 1 us)\r\n");
}

static void test_set_clocksync_cfg(int argc, char **argv)
{
    wlan_clock_sync_gpio_tsf_t tsf_latch;

    if (argc != 6)
    {
        dump_wlan_set_clocksync_cfg_usage();
        return;
    }

    errno                     = 0;
    tsf_latch.clock_sync_mode = (uint8_t)strtol(argv[1], NULL, 0);

    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }

    errno                     = 0;
    tsf_latch.clock_sync_Role = (uint8_t)strtol(argv[2], NULL, 0);

    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }

    errno                                = 0;
    tsf_latch.clock_sync_gpio_pin_number = (uint8_t)strtol(argv[3], NULL, 0);

    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }

    errno                                  = 0;
    tsf_latch.clock_sync_gpio_level_toggle = (uint8_t)strtol(argv[4], NULL, 0);

    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }

    errno                                 = 0;
    tsf_latch.clock_sync_gpio_pulse_width = (uint16_t)strtol(argv[5], NULL, 0);

    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }

    int rv = wlan_set_clocksync_cfg(&tsf_latch);

    if (rv != WM_SUCCESS)
    {
        (void)PRINTF("Unable to set clocksync config\r\n");
    }
    else
    {
        (void)PRINTF("Clock Sync config set as:\r\n");
        (void)PRINTF("Mode                 :%d\n\r", tsf_latch.clock_sync_mode);
        (void)PRINTF("Role                 :%d\n\r", tsf_latch.clock_sync_Role);
        (void)PRINTF("GPIO Pin Number      :%d\n\r", tsf_latch.clock_sync_gpio_pin_number);
        (void)PRINTF("GPIO Level or Toggle :%d\n\r", tsf_latch.clock_sync_gpio_level_toggle);
        (void)PRINTF("GPIO Pulse Width     :%d\n\r", tsf_latch.clock_sync_gpio_pulse_width);
    }
}
#endif /* CONFIG_WIFI_CLOCKSYNC */


static struct cli_command wlan_enhanced_commands[] =
{
    {"wlan-get-txpwrlimit", "<subband>", test_wlan_get_txpwrlimit},
#if !CONFIG_COMPRESS_TX_PWTBL
    {"wlan-set-txpwrlimit", NULL, test_wlan_set_txpwrlimit},
    {"wlan-set-chanlist-and-txpwrlimit", NULL, test_wlan_set_chanlist_and_txpwrlimit},
#endif
    {"wlan-set-chanlist", NULL, test_wlan_set_chanlist},
    {"wlan-get-chanlist", NULL, test_wlan_get_chanlist},
#if CONFIG_11AC
    {"wlan-set-txratecfg", "<sta/uap> <format> <index> <nss> <rate_setting> <autoTx_set>", test_wlan_set_txratecfg},
#else
    {"wlan-set-txratecfg", "<sta/uap> <format> <index> <autoTx_set>", test_wlan_set_txratecfg},
#endif
    {"wlan-get-txratecfg", "<sta/uap>", test_wlan_get_txratecfg},
    {"wlan-get-data-rate", "<sta/uap>", test_wlan_get_data_rate},
    {"wlan-get-pmfcfg", NULL, wlan_pmfcfg_get},
    {"wlan-uap-get-pmfcfg", NULL, wlan_uap_pmfcfg_get},
#if CONFIG_5GHz_SUPPORT
    {
        "wlan-set-ed-mac-mode", "<interface> <ed_ctrl_2g> <ed_offset_2g> <ed_ctrl_5g> <ed_offset_5g>",
        wlan_ed_mac_mode_set
    },
#else
    {"wlan-set-ed-mac-mode", "<interface> <ed_ctrl_2g> <ed_offset_2g>", wlan_ed_mac_mode_set},
#endif
    {"wlan-get-ed-mac-mode", "<interface>", wlan_ed_mac_mode_get},
#if CONFIG_11AX
    {"wlan-set-tx-omi", "<interface> <tx-omi> <tx-option> <num_data_pkts>", test_wlan_set_tx_omi},
    {"wlan-set-toltime", "<value>", test_wlan_set_toltime},
    {"wlan-set-rutxpwrlimit", NULL, test_wlan_set_rutxpwrlimit},
    {"wlan-11ax-cfg", "<11ax_cfg>", test_wlan_11ax_cfg},
#if CONFIG_11AX_TWT
    {"wlan-11ax-bcast-twt", "<bcast_twt_cfg>", test_wlan_bcast_twt},
    {"wlan-11ax-twt-setup", "<twt_cfg>", test_wlan_twt_setup},
    {"wlan-11ax-twt-teardown", "<twt_cfg>", test_wlan_twt_teardown},
    {"wlan-11ax-twt-report", "<twt_report_get>", test_wlan_twt_report},
#endif /* CONFIG_11AX_TWT */
#endif /* CONFIG_11AX */
#if CONFIG_WIFI_CLOCKSYNC
    {"wlan-get-tsfinfo", "<format-type>", test_get_tsf_info},
    {"wlan-set-clocksync", "<mode> <role> <gpio_pin> <gpio_level> <pulse width>", test_set_clocksync_cfg},
#endif /* CONFIG_WIFI_CLOCKSYNC */
};

int wlan_enhanced_cli_init(void)
{
#if CONFIG_11AX
    wlan_init_g_test_cfg_arrays();
#endif

    if (cli_register_commands(wlan_enhanced_commands,
                              (int)(sizeof(wlan_enhanced_commands) / sizeof(struct cli_command))) != 0)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

int wlan_enhanced_cli_deinit(void)
{
    if (cli_unregister_commands(wlan_enhanced_commands,
                                (int)(sizeof(wlan_enhanced_commands) / sizeof(struct cli_command))) != 0)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}
