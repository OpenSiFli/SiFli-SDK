/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Shim header replacing platform/fmac/userconfig.h
 * Contains struct definitions only (no platform code).
 */

#ifndef _USERCONFIG_H_
#define _USERCONFIG_H_

#include "aic_porting_types.h"

typedef struct
{
    uint8_t enable;
    uint8_t dsss;
    uint8_t ofdmlowrate_2g4;
    uint8_t ofdm64qam_2g4;
    uint8_t ofdm256qam_2g4;
    uint8_t ofdm1024qam_2g4;
    uint8_t ofdmlowrate_5g;
    uint8_t ofdm64qam_5g;
    uint8_t ofdm256qam_5g;
    uint8_t ofdm1024qam_5g;
} aic_txpwr_lvl_conf_t;

typedef struct
{
    uint8_t enable;
    int8_t pwrlvl_11b_11ag_2g4[12];
    int8_t pwrlvl_11n_11ac_2g4[10];
    int8_t pwrlvl_11ax_2g4[12];
} aic_txpwr_lvl_conf_v2_t;

typedef struct
{
    uint8_t enable;
    int8_t pwrlvl_11b_11ag_2g4[12];
    int8_t pwrlvl_11n_11ac_2g4[10];
    int8_t pwrlvl_11ax_2g4[12];
    int8_t pwrlvl_11a_5g[12];
    int8_t pwrlvl_11n_11ac_5g[10];
    int8_t pwrlvl_11ax_5g[12];
} aic_txpwr_lvl_conf_v3_t;

typedef struct
{
    uint8_t enable;
    int8_t pwrlvl_11b_11ag_2g4[12];
    int8_t pwrlvl_11n_11ac_2g4[10];
    int8_t pwrlvl_11ax_2g4[12];
    int8_t pwrlvl_11a_5g[8];
    int8_t pwrlvl_11n_11ac_5g[10];
    int8_t pwrlvl_11ax_5g[12];
    int8_t pwrlvl_11a_6g[8];
    int8_t pwrlvl_11n_11ac_6g[10];
    int8_t pwrlvl_11ax_6g[12];
} aic_txpwr_lvl_conf_v4_t;

typedef struct
{
    uint8_t enable;
    uint8_t dsss;
    uint8_t ofdmlowrate_2g4;
    uint8_t ofdm64qam_2g4;
    uint8_t ofdm256qam_2g4;
    uint8_t ofdm1024qam_2g4;
    uint8_t ofdmlowrate_5g;
    uint8_t ofdm64qam_5g;
    uint8_t ofdm256qam_5g;
    uint8_t ofdm1024qam_5g;
} aic_txpwr_idx_conf_t;

typedef struct
{
    uint8_t enable;
    int8_t chan_1_4;
    int8_t chan_5_9;
    int8_t chan_10_13;
    int8_t chan_36_64;
    int8_t chan_100_120;
    int8_t chan_122_140;
    int8_t chan_142_165;
} aic_txpwr_ofst_conf_t;

typedef struct
{
    int8_t enable;
    int8_t pwrofst2x_tbl_2g4[3][3];
    int8_t pwrofst2x_tbl_5g[3][6];
} aic_txpwr_ofst2x_conf_t;

typedef struct
{
    uint8_t enable;
    uint8_t pwrofst_flags;
    int8_t pwrofst2x_tbl_2g4_ant0[3][3];
    int8_t pwrofst2x_tbl_2g4_ant1[3][3];
    int8_t pwrofst2x_tbl_5g_ant0[6][3];
    int8_t pwrofst2x_tbl_5g_ant1[6][3];
    int8_t pwrofst2x_tbl_6g_ant0[15];
    int8_t pwrofst2x_tbl_6g_ant1[15];
} aic_txpwr_ofst2x_conf_v2_t;

typedef struct
{
    uint8_t enable;
    uint8_t xtal_cap;
    uint8_t xtal_cap_fine;
} aic_xtal_cap_conf_t;

typedef struct
{
    aic_txpwr_lvl_conf_t txpwr_lvl;
    aic_txpwr_lvl_conf_v2_t txpwr_lvl_v2;
    aic_txpwr_lvl_conf_v3_t txpwr_lvl_v3;
    aic_txpwr_lvl_conf_v4_t txpwr_lvl_v4;
    aic_txpwr_ofst_conf_t txpwr_ofst;
    aic_txpwr_ofst2x_conf_t txpwr_ofst2x;
    aic_txpwr_ofst2x_conf_v2_t txpwr_ofst2x_v2;
    aic_xtal_cap_conf_t xtal_cap;
} aic_userconfig_info_t;

typedef struct
{
    aic_txpwr_idx_conf_t txpwr_idx;
    aic_txpwr_ofst_conf_t txpwr_ofst;
    aic_xtal_cap_conf_t xtal_cap;
} aic_nvram_info_t;

extern aic_nvram_info_t aic_nvram_info;
extern aic_userconfig_info_t aic_userconfig_info;

enum aicbtu_btport_type
{
    AICBTU_BTPORT_NULL,
    AICBTU_BTPORT_MB,
    AICBTU_BTPORT_UART,
};

enum aicbtu_btmode_type
{
    AICBTU_BTMODE_BT_ONLY_SW = 0x0,
    AICBTU_BTMODE_BT_WIFI_COMBO,
    AICBTU_BTMODE_BT_ONLY,
    AICBTU_BTMODE_BT_ONLY_TEST,
    AICBTU_BTMODE_BT_WIFI_COMBO_TEST,
    AICBTU_BTMODE_BT_ONLY_COANT,
    AICBTU_MODE_NULL = 0xFF,
};

enum aicbtu_uart_baud_type
{
    AICBTU_UART_BAUD_115200     = 115200,
    AICBTU_UART_BAUD_921600     = 921600,
    AICBTU_UART_BAUD_1_5M       = 1500000,
    AICBTU_UART_BAUD_3_25M      = 3250000,
};

enum aicbtu_user_uart_flowctrl_type
{
    AICBTU_UART_FLOWCTRL_DISABLE = 0x0,
    AICBTU_UART_FLOWCTRL_ENABLE,
};

typedef struct
{
    uint32_t btmode;
    uint32_t btport;
    uint32_t uart_baud;
    uint32_t uart_flowctrl;
    uint32_t lpm_enable;
    uint32_t txpwr_lvl;
    uint8_t enable;
} aicbt_user_info_t;

extern aicbt_user_info_t aicbt_user_info;

enum
{
    VRF_LDO_MODE,
    VRF_DCDC_MODE,
};

#endif /* _USERCONFIG_H_ */
