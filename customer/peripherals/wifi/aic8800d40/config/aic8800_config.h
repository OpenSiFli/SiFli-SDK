/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AIC8800_CONFIG_H_
#define _AIC8800_CONFIG_H_

#include "plat_port.h"

#define     CONFIG_AIC8800DW_SUPPORT
#define     CONFIG_AIC8800D80_SUPPORT
#define     CONFIG_SDIO_SUPPORT
//#define     CONFIG_OOB

//// firmware bin file
#define     CONFIG_DOWNLOAD_FW
//#define     CONFIG_LOAD_FW_FROM_FLASH
#define     CONFIG_WIFI_MODE_RFTEST

//// wifi setting
#define     CONFIG_RWNX_FULLMAC
//#define     CONFIG_OFFCHANNEL
#define     USE_5G
#define     CONFIG_USE_5G

//// hardware settings
#define     CONFIG_PMIC_SETTING
#define     CONFIG_LOAD_USERCONFIG
#ifdef CONFIG_AIC8800_VRF_DCDC_MODE
    #define     CONFIG_VRF_DCDC_MODE
#endif
#define     CONFIG_DPD
#define     CONFIG_FORCE_DPD_CALIB

//// SDIO and AP power save
//#define     CONFIG_SDIO_BUS_PWRCTRL
#define     CONFIG_SDIO_BUS_PWRCTRL_DYN

//// software settings
#define     CONFIG_LWIP
#define     CONFIG_REORD_FORWARD_LIST
//#define     CONFIG_RX_NOCOPY
#define     CONFIG_RX_WAIT_LWIP
//#define     CONFIG_APP_IPERF

/// WPA3 Need
#define     CONFIG_MBEDTLS

// flow ctr delay ms
#define     SDIO_FC_NARB

//// debug log
#define     CONFIG_RWNX_DBG
#define     AIC_LOG_DEBUG_ON

////rm driver one key ,just reset env
//#define     CONFIG_DRIVER_ORM

//os platform
//#define     CONFIG_PLAT_THREADX
#define     CONFIG_PLAT_RTTHREAD

//bt support
#ifdef CONFIG_AIC8800_BT_SUPPORT
    #define CONFIG_BT_SUPPORT
#endif

#define CONFIG_MONITOR_RADIO_HEADER

//#define CONFIG_USE_LOCAL_MAC_ADDR

#endif /* _AIC8800_CONFIG_H_ */
