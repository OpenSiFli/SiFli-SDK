/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _USERCONFIG_H_
#define _USERCONFIG_H_

#include "aic_plat_types.h"

typedef unsigned char u8_l;
typedef signed char   s8_l;
typedef signed char   int8_t;
typedef unsigned char uint8_t;

typedef struct
{
    u8_l enable;
    u8_l dsss;
    u8_l ofdmlowrate_2g4;
    u8_l ofdm64qam_2g4;
    u8_l ofdm256qam_2g4;
    u8_l ofdm1024qam_2g4;
    u8_l ofdmlowrate_5g;
    u8_l ofdm64qam_5g;
    u8_l ofdm256qam_5g;
    u8_l ofdm1024qam_5g;
} aic_txpwr_lvl_conf_t;

typedef struct
{
    u8_l enable;
    s8_l pwrlvl_11b_11ag_2g4[12];
    s8_l pwrlvl_11n_11ac_2g4[10];
    s8_l pwrlvl_11ax_2g4[12];
} aic_txpwr_lvl_conf_v2_t;

typedef struct
{
    u8_l enable;
    s8_l pwrlvl_11b_11ag_2g4[12];
    s8_l pwrlvl_11n_11ac_2g4[10];
    s8_l pwrlvl_11ax_2g4[12];
    s8_l pwrlvl_11a_5g[12];
    s8_l pwrlvl_11n_11ac_5g[10];
    s8_l pwrlvl_11ax_5g[12];
} aic_txpwr_lvl_conf_v3_t;

typedef struct {
	u8_l enable;
	u8_l dsss;
	u8_l ofdmlowrate_2g4;
	u8_l ofdm64qam_2g4;
	u8_l ofdm256qam_2g4;
	u8_l ofdm1024qam_2g4;
	u8_l ofdmlowrate_5g;
	u8_l ofdm64qam_5g;
	u8_l ofdm256qam_5g;
	u8_l ofdm1024qam_5g;
} aic_txpwr_idx_conf_t;

typedef struct {
	u8_l enable;
	s8_l chan_1_4;
	s8_l chan_5_9;
	s8_l chan_10_13;
	s8_l chan_36_64;
	s8_l chan_100_120;
	s8_l chan_122_140;
	s8_l chan_142_165;
} aic_txpwr_ofst_conf_t;

typedef struct
{
    s8_l enable;
    s8_l pwrofst2x_tbl_2g4[3][3];
    s8_l pwrofst2x_tbl_5g[3][6];
} aic_txpwr_ofst2x_conf_t;

typedef struct
{
    u8_l enable;
    u8_l xtal_cap;
    u8_l xtal_cap_fine;
} aic_xtal_cap_conf_t;

//8800DCDW userconfig
typedef struct
{
    aic_txpwr_lvl_conf_t txpwr_lvl;
    aic_txpwr_lvl_conf_v2_t txpwr_lvl_v2;
    aic_txpwr_lvl_conf_v3_t txpwr_lvl_v3;
    aic_txpwr_ofst_conf_t txpwr_ofst;
    aic_txpwr_ofst2x_conf_t txpwr_ofst2x;
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

#endif /* _USERCONFIG_H_ */

