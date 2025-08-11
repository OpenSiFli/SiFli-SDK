/*
 *  Copyright 2020-2022 NXP
 *  All rights reserved.
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _WIFI_CONFIG_H_
#define _WIFI_CONFIG_H_

//#include "app_config.h"
#ifndef RW610
    //#include "wifi_bt_module_config.h"
#endif
#define WIFI_IW416_BOARD_MURATA_1XK_USD

//#define CONFIG_IPV6 0
//#define CONFIG_MAX_IPV6_ADDRESSES 0

#if defined(SD9177)
    #define CONFIG_WMM 1
    #define CONFIG_SDIO_MULTI_PORT_TX_AGGR 1
    #define CONFIG_WIFI_FEATURES 1
    #define CONFIG_OFFLOAD       1
#endif

#if defined(RW610)
    #define CONFIG_MAX_RESCAN_LIMIT 30
    #define PRINTF_FLOAT_ENABLE 1
    #define CONFIG_HOST_SLEEP 0
    #define CONFIG_POWER_MANAGER 0
    #define CONFIG_MEF_CFG 1
    /** If define CONFIG_TX_RX_ZERO_COPY 1, please make sure
    *  #define PBUF_POOL_BUFSIZE 1752
    *  in lwipopts.h
    */
    #define CONFIG_TX_RX_ZERO_COPY 1
    #define CONFIG_ANT_DETECT 1
#endif

#if defined(SD8978) || defined(SD8987) || defined(SD8801) || defined(SD9177)
    #define CONFIG_WIFI_AUTO_POWER_SAVE 1
    #define CONFIG_WIFI_CAPA        1
    #define CONFIG_ROAMING          1
    #define CONFIG_CLOUD_KEEP_ALIVE 1
    #define CONFIG_TURBO_MODE       1
    #define CONFIG_AUTO_RECONNECT   1
    #define CONFIG_DRIVER_OWE       1
    #define CONFIG_EXT_SCAN_SUPPORT 1
    #define CONFIG_WIFI_EU_CRYPTO   1
    #define CONFIG_11R              1
    #define CONFIG_11K              1
    #define CONFIG_11V              1
#endif

/*
 * Config options for wpa supplicant
 */
#define CONFIG_WPA_SUPP 1

#if CONFIG_WPA_SUPP
    #ifdef RW610
        #define CONFIG_WPA_SUPP_WPS               1
        #define CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE 1
        #define CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE 1
        #define CONFIG_WIFI_USB_FILE_ACCESS       0

        #define CONFIG_WPA_SUPP_DPP 1

        #if (CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE || CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE)
            #define CONFIG_EAP_TLS 1
            #define CONFIG_EAP_PEAP 1
            #define CONFIG_EAP_TTLS 1
            #define CONFIG_EAP_FAST 1
            #define CONFIG_EAP_SIM 1
            #define CONFIG_EAP_AKA 1
            #define CONFIG_EAP_AKA_PRIME 1

            #if (CONFIG_EAP_PEAP || CONFIG_EAP_TTLS || CONFIG_EAP_FAST)
                #define CONFIG_EAP_MSCHAPV2 1
                #define CONFIG_EAP_GTC 1
            #endif
        #endif

    #else
        #define CONFIG_WPA_SUPP_WPS               1
        #define CONFIG_WPA_SUPP_WPA3              1
        #define CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE 0 //企业模式（WPA-Enterprise）​：需 RADIUS 服务器认证，支持多用户独立密钥，安全性更高

        #if defined(SD9177)
            #define CONFIG_WPA_SUPP_DPP 0

            #if CONFIG_WPA_SUPP_DPP
                #define CONFIG_WPA_SUPP_DPP2 1
                #define CONFIG_WPA_SUPP_DPP3 1
                #define CONFIG_RX_CHAN_INFO  1
                #define CONFIG_TXPD_RXPD_V3  1
            #endif
        #endif

        #if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE

            #define CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE 1

            #define CONFIG_EAP_TLS 1
            #define CONFIG_EAP_PEAP 1
            #define CONFIG_EAP_TTLS 1
            #define CONFIG_EAP_FAST 1
            #define CONFIG_EAP_SIM 1
            #define CONFIG_EAP_AKA 1
            #define CONFIG_EAP_AKA_PRIME 1
            #define CONFIG_EAP_MSCHAPV2 1
            #define CONFIG_EAP_GTC 1

        #endif
    #endif
#endif

/* WLCMGR debug */
#define CONFIG_WLCMGR_DEBUG 0

/*
 * Wifi extra debug options
 */
#define CONFIG_WIFI_EXTRA_DEBUG 0
#define CONFIG_WIFI_EVENTS_DEBUG 0
#define CONFIG_WIFI_CMD_RESP_DEBUG 0
#define CONFIG_WIFI_PKT_DEBUG 0
#define CONFIG_WIFI_SCAN_DEBUG 0
#define CONFIG_WIFI_IO_INFO_DUMP 0
#define CONFIG_WIFI_IO_DEBUG 0
#define CONFIG_WIFI_IO_DUMP 0
#define CONFIG_WIFI_MEM_DEBUG 0
#define CONFIG_WIFI_AMPDU_DEBUG 0
#define CONFIG_WIFI_TIMER_DEBUG 0
#define CONFIG_WIFI_SDIO_DEBUG 0
#define CONFIG_WIFI_FW_DEBUG 0
#define CONFIG_WIFI_UAP_DEBUG 0
#define CONFIG_WPS_DEBUG 0
#define CONFIG_FW_VDLL_DEBUG 0
#define CONFIG_DHCP_SERVER_DEBUG 0
#define CONFIG_FWDNLD_IO_DEBUG 0
#ifdef RW610
    #define CONFIG_WIFI_PS_DEBUG 0
#endif

/*
 * Heap debug options
 */
#define CONFIG_HEAP_DEBUG 0
#define CONFIG_HEAP_STAT 0

/*
 * wpa supplicant debug options
 */
#define CONFIG_WPA_SUPP_DEBUG_LEVEL 6

#define CONFIG_SUPP_DEBUG 0

#endif /* _WIFI_CONFIG_H_ */
