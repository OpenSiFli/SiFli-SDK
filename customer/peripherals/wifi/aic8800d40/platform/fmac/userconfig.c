/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (C) 2018-2020 AICSemi Ltd.
 *
 * All Rights Reserved
 */

#include "userconfig.h"

//8800D userconfig
aic_nvram_info_t aic_nvram_info = {
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
        .enable       = 0,
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

//8800DCDW & 8800D40/D80 userconfig
aic_userconfig_info_t aic_userconfig_info = {
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
    // txpwr_lvl_v2 for 8800DCDW
    .txpwr_lvl_v2 = {
        .enable             = 1,
        .pwrlvl_11b_11ag_2g4 =
            //1M,   2M,   5M5,  11M,  6M,   9M,   12M,  18M,  24M,  36M,  48M,  54M
            { 18,   18,   18,   18,   20,   20,   20,   20,   18,   18,   16,   16},
        .pwrlvl_11n_11ac_2g4 =
            //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9
            { 20,   20,   20,   20,   18,   18,   16,   16,   16,   16},
        .pwrlvl_11ax_2g4 =
            //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9, MCS10,MCS11
            { 20,   20,   20,   20,   18,   18,   16,   16,   16,   16,   15,   15},
    },
    // txpwr_lvl_v3 for 8800D40/D80
#if 0
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
#else
     .txpwr_lvl_v3 = {
        .enable             = 1,
        .pwrlvl_11b_11ag_2g4 =
            //1M,   2M,   5M5,  11M,  6M,   9M,   12M,  18M,  24M,  36M,  48M,  54M
            { 21,   21,   21,   21,   21,   21,   21,   21,   19,   19,   17,   17},
        .pwrlvl_11n_11ac_2g4 =
            //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9
            { 21,   21,   21,   21,   17,   17,   16,   16,   15,   15},
        .pwrlvl_11ax_2g4 =
            //MCS0, MCS1, MCS2, MCS3, MCS4, MCS5, MCS6, MCS7, MCS8, MCS9, MCS10,MCS11
            { 21,   21,   21,   21,   17,   17,   16,   16,   15,   15,   14,   14},
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
    // txpwr_ofst for 8800DCDW
#if 0
    .txpwr_ofst = {
        .enable       = 1,
        .chan_1_4     = 0,
        .chan_5_9     = 0,
        .chan_10_13   = 0,
        .chan_36_64   = -4,
        .chan_100_120 = -4,
        .chan_122_140 = 4,
        .chan_142_165 = 4,
    },
#else
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
#endif
    // txpwr_ofst2x for 8800D40/D80
    .txpwr_ofst2x = {
        .enable       = 0,
        .pwrofst2x_tbl_2g4 =
        { // ch1-4, ch5-9, ch10-13
            {   0,    0,    0   }, // 11b
            {   0,    0,    0   }, // ofdm_highrate
            {   0,    0,    0   }, // ofdm_lowrate
        },
        .pwrofst2x_tbl_5g =
        { // ch42,  ch58, ch106,ch122,ch138,ch155
            {   0,    0,    0,    0,    0,    0   }, // ofdm_lowrate
            {   0,    0,    0,    0,    0,    0   }, // ofdm_highrate
            {   0,    0,    0,    0,    0,    0   }, // ofdm_midrate
        },
    },
    .xtal_cap = {
        .enable        = 0,
        .xtal_cap      = 24,
        .xtal_cap_fine = 31,
    },
};

