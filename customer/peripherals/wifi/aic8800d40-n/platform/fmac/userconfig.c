/*
 * Copyright (C) 2018-2020 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "userconfig.h"

//8800D userconfig drop
aic_nvram_info_t aic_nvram_info =
{
    .txpwr_idx = {
        .enable           = 1,
        .dsss             = 9,
        .ofdmlowrate_2g4  = 10,
        .ofdm64qam_2g4    = 10,
        .ofdm256qam_2g4   = 9,
        .ofdm1024qam_2g4  = 8,
        .ofdmlowrate_5g   = 11,
        .ofdm64qam_5g     = 9,
        .ofdm256qam_5g    = 9,
        .ofdm1024qam_5g   = 9
    },
    .txpwr_ofst = {
        .enable       = 1,
        .chan_1_4     = 0,
        .chan_5_9     = 0,
        .chan_10_13   = 0,
        .chan_36_64   = 3,
        .chan_100_120 = -15,
        .chan_122_140 = -7,
        .chan_142_165 = 3,
    },
    .xtal_cap = {
        .enable        = 0,
        .xtal_cap      = 24,
        .xtal_cap_fine = 31,
    },
};

aic_userconfig_info_t aic_userconfig_info =
{
    //8800D userconfig
    .txpwr_lvl = {
        .enable           = 1,
        .dsss             = 17,
        .ofdmlowrate_2g4  = 15,
        .ofdm64qam_2g4    = 14,
        .ofdm256qam_2g4   = 13,
        .ofdm1024qam_2g4  = 13,
        .ofdmlowrate_5g   = 15,
        .ofdm64qam_5g     = 14,
        .ofdm256qam_5g    = 13,
        .ofdm1024qam_5g   = 13
    },
    //8800DCDW userconfig
    .txpwr_lvl_v2 = {
        .enable             = 1,
        .pwrlvl_11b_11ag_2g4 =
        //1M,   2M,   5M5,  11M,  6M,   9M,   12M,  18M,  24M,  36M,  48M,  54M
        { 18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   16,   16},
        .pwrlvl_11n_11ac_2g4 =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9
        { 18,   18,   18,   18,   18,   18,   16,   16,   16,   16},
        .pwrlvl_11ax_2g4 =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9, MCS10,MCS11
        { 18,   18,   18,   18,   18,   18,   16,   16,   16,   16,   15,   15},
    },
    //8800D40D80 userconfig
#if defined(CONFIG_AIC8800D80N)
    .txpwr_lvl_v3 = {  // D80N: matches Linux aic_userconfig_8800d80n.txt
        .enable             = 1,
        .pwrlvl_11b_11ag_2g4 =
        //1M,   2M,   5M5,  11M,  6M,   9M,   12M,  18M,  24M,  36M,  48M,  54M
        { 20,   20,   20,   20,   20,   20,   20,   20,   19,   19,   18,   18},
        .pwrlvl_11n_11ac_2g4 =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9
        { 18,   18,   18,   18,   16,   16,   16,   16,   15,   15},
        .pwrlvl_11ax_2g4 =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9, MCS10,MCS11
        { 18,   18,   18,   17,   17,   17,   17,   16,   16,   16,   15,   14},
        .pwrlvl_11a_5g =
        //NA,   NA,   NA,   NA,   6M,   9M,   12M,  18M,  24M,  36M,  48M,  54M
        { 0x80, 0x80, 0x80, 0x80, 18,   17,   17,   17,   17,   17,   17,   16},
        .pwrlvl_11n_11ac_5g =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9
        { 18,   18,   18,   17,   17,   17,   16,   15,   14,   14},
        .pwrlvl_11ax_5g =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9, MCS10,MCS11
        { 18,   18,   18,   17,   17,   17,   16,   15,   13,   13,   13,   13},
    },
#else
    .txpwr_lvl_v3 = {
        .enable             = 1,
        .pwrlvl_11b_11ag_2g4 =
        //1M,   2M,   5M5,  11M,  6M,   9M,   12M,  18M,  24M,  36M,  48M,  54M
        { 18,   18,   18,   18,   18,   18,   18,   18,   16,   16,   15,   15},
        .pwrlvl_11n_11ac_2g4 =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9
        { 18,   18,   18,   18,   16,   16,   15,   15,   14,   14},
        .pwrlvl_11ax_2g4 =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9, MCS10,MCS11
        { 18,   18,   18,   18,   16,   16,   15,   15,   14,   14,   13,   13},
        .pwrlvl_11a_5g =
        //NA,   NA,   NA,   NA,   6M,   9M,   12M,  18M,  24M,  36M,  48M,  54M
        { 0x80, 0x80, 0x80, 0x80, 18,   18,   18,   18,   16,   16,   15,   15},
        .pwrlvl_11n_11ac_5g =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9
        { 18,   18,   18,   18,   16,   16,   15,   15,   14,   14},
        .pwrlvl_11ax_5g =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9, MCS10,MCS11
        { 18,   18,   18,   18,   16,   16,   14,   14,   13,   13,   12,   12},
    },
#endif
#if defined(CONFIG_AIC8800D80X2)
    .txpwr_lvl_v4 = {  // D80X2: matches Linux aic_userconfig_8800d80x2.txt
        .enable             = 1,
        .pwrlvl_11b_11ag_2g4 =
        //1M,   2M,   5M5,  11M,  6M,   9M,   12M,  18M,  24M,  36M,  48M,  54M
        { 18,   18,   18,   18,   18,   18,   18,   18,   16,   16,   15,   15},
        .pwrlvl_11n_11ac_2g4 =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9
        { 18,   18,   18,   18,   16,   16,   15,   15,   14,   14},
        .pwrlvl_11ax_2g4 =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9, MCS10,MCS11
        { 18,   18,   18,   18,   16,   16,   15,   15,   14,   14,   13,   13},
        .pwrlvl_11a_5g =
        //6M,   9M,   12M,  18M,  24M,  36M,  48M,  54M
        { 18,   18,   18,   18,   16,   16,   15,   15},
        .pwrlvl_11n_11ac_5g =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9
        { 18,   18,   18,   18,   16,   16,   15,   15,   14,   14},
        .pwrlvl_11ax_5g =
        //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9, MCS10,MCS11
        { 18,   18,   18,   18,   16,   16,   14,   14,   13,   13,   12,   12},
        .pwrlvl_11a_6g = { 0, 0, 0, 0, 0, 0, 0, 0 },
        .pwrlvl_11n_11ac_6g = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        .pwrlvl_11ax_6g = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    },
#endif
    //8800DCDW
    .txpwr_ofst = {
        .enable       = 0,
        .chan_1_4     = 0,
        .chan_5_9     = 0,
        .chan_10_13   = 0,
        .chan_36_64   = 0,
        .chan_100_120 = 0,
        .chan_122_140 = 0,
        .chan_142_165 = 0,
    },
    //8800D40D80
    .txpwr_ofst2x = {
        .enable       = 0,
        .pwrofst2x_tbl_2g4 =
        {
            // ch1-4, ch5-9, ch10-13
            {   0,    0,    0   }, // 11b
            {   0,    0,    0   }, // ofdm_highrate
            {   0,    0,    0   }, // ofdm_lowrate
        },
        .pwrofst2x_tbl_5g =
        {
            // ch42,  ch58, ch106,ch122,ch138,ch155
            {   0,    0,    0,    0,    0,    0   }, // ofdm_lowrate
            {   0,    0,    0,    0,    0,    0   }, // ofdm_highrate
            {   0,    0,    0,    0,    0,    0   }, // ofdm_midrate
        },
    },
    //8800D80X2 (matches Linux aic_userconfig_8800d80x2.txt: ofst_enable=0)
    .txpwr_ofst2x_v2 = {
        .enable       = 0,
        .pwrofst_flags = 0,
        .pwrofst2x_tbl_2g4_ant0 = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },
        .pwrofst2x_tbl_2g4_ant1 = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } },
        .pwrofst2x_tbl_5g_ant0 =
        {
            // ch42,  ch58, ch106,ch122,ch138,ch155
            {   0,    0,    0   },
            {   0,    0,    0   },
            {   0,    0,    0   },
            {   0,    0,    0   },
            {   0,    0,    0   },
            {   0,    0,    0   },
        },
        .pwrofst2x_tbl_5g_ant1 =
        {
            // ch42,  ch58, ch106,ch122,ch138,ch155
            {   0,    0,    0   },
            {   0,    0,    0   },
            {   0,    0,    0   },
            {   0,    0,    0   },
            {   0,    0,    0   },
            {   0,    0,    0   },
        },
        .pwrofst2x_tbl_6g_ant0 = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        .pwrofst2x_tbl_6g_ant1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    },
    //8800DCDWD40D80
    .xtal_cap = {
        .enable        = 0,
        .xtal_cap      = 24,
        .xtal_cap_fine = 31,
    },
};

//bt param config
/* BT antenna mode — matches the Linux SDIO baseline (aic8800_bsp/aic_bsp_driver.c + aic_bsp_driver.h):
 *   D80 / D80N / D80X2 : COANT (AICBT_BTMODE_DEFAULT_8800d80* = BT_ONLY_COANT)
 *   DC / DW            : COMBO (fixed BT_WIFI_COMBO in the driver)
 * If misconfigured, the stack still starts but nothing is received over the air
 * (phones cannot find BLE advertisements / BT does not work). */
#if defined(CONFIG_AIC8800D80) || defined(CONFIG_AIC8800D80N) || \
    defined(CONFIG_AIC8800D80X2)
    #define AICBTU_BTMODE_DEFAULT   AICBTU_BTMODE_BT_ONLY_COANT
#else
    #define AICBTU_BTMODE_DEFAULT   AICBTU_BTMODE_BT_WIFI_COMBO
#endif

aicbt_user_info_t aicbt_user_info =
{
#if defined(AIC_HCI_OVER_SDIO)
    /* Mailbox HCI: BT only usable together with the WiFi firmware. */
    .btmode        = AICBTU_BTMODE_DEFAULT,
    .btport        = AICBTU_BTPORT_MB,
#else
    .btmode        = AICBTU_BTMODE_DEFAULT,
    .btport        = AICBTU_BTPORT_UART,
#endif
    .uart_baud     = AICBTU_UART_BAUD_921600,
    .uart_flowctrl = AICBTU_UART_FLOWCTRL_DISABLE,
    .lpm_enable    = 0,
    .txpwr_lvl     = 0x00006f2f,
    .enable        = 1
};





