/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AIC_CFG_H_
#define _AIC_CFG_H_

/* ============================================================================
 * AIC8800 unified configuration header — shared by Host and Platform layers
 *
 * Usage:
 *   1. Feature trimming: uncomment the AIC_DISABLE_* macros below to disable them
 *   2. Other macros: driver defaults, usually no need to modify
 *   3. Source trimming: use together with the Make switches in aic_cfg.mk
 *
 * Note: AP and STA are basic features and cannot be trimmed
 * ============================================================================
 */

/* ============================================================================
 * Feature trimming — RTOS space-constrained scenarios
 * Controlled in aic_cfg.mk via AIC_FEATURE_P2P / AIC_FEATURE_BT,
 * the Makefile maps them to -D compile macros passed in here
 * ============================================================================ */


/* ============================================================================
 * Chip model
 *
 * Controlled by aic_cfg.mk via -D flags; the value here is only the fallback default.
 * AIC_CHIP_MODEL is set in aic_cfg.mk: D80 / D80N / D80N_U02C / DW / DC / DL (DC variant)
 * ============================================================================ */
#ifndef CONFIG_AIC8800D80
    #ifndef CONFIG_AIC8800DW
        #ifndef CONFIG_AIC8800DC
            #ifndef CONFIG_AIC8800D80N
                #ifndef CONFIG_AIC8800D80X2
                    // if not specified by the Makefile, enable D80 by default
                    #define CONFIG_AIC8800D80
                #endif
            #endif
        #endif
    #endif
#endif


/* ============================================================================
 * Hardware configuration
 * ============================================================================ */
// USE_5G is only supported by CONFIG_AIC8800D80 / CONFIG_AIC8800D80N / CONFIG_AIC8800D80X2, set by aic_cfg.mk via -DUSE_5G
#if (defined(CONFIG_AIC8800D80) || defined(CONFIG_AIC8800D80N) || defined(CONFIG_AIC8800D80X2)) && !defined(USE_5G)
    #define USE_5G
#endif
/* ---- Bus type selection (Plan B) --------------------------------------- *
 * AIC_BUS_TYPE selects the active transport bus:
 *   SDIO (default) → CONFIG_SDIO_SUPPORT
 *   USB            → CONFIG_USB_SUPPORT + CONFIG_AICWF_BUS_USB
 * Set via aic_cfg.mk (AIC_BUS_TYPE := SDIO|USB) or -DAIC_BUS_USB / -DAIC_BUS_SDIO.
 * When USB is selected, do NOT auto-define CONFIG_SDIO_SUPPORT.
 */
#if !defined(CONFIG_SDIO_SUPPORT) && !defined(CONFIG_USB_SUPPORT)
    #if defined(AIC_BUS_USB)
        #define CONFIG_USB_SUPPORT
        #ifndef CONFIG_AICWF_BUS_USB
            #define CONFIG_AICWF_BUS_USB
        #endif
    #else
        #define CONFIG_SDIO_SUPPORT
    #endif
#endif
#ifndef CONFIG_PMIC_SETTING
    #define CONFIG_PMIC_SETTING
#endif
#ifndef CONFIG_LOAD_USERCONFIG
    #define CONFIG_LOAD_USERCONFIG
#endif
#ifndef CONFIG_DPD
    #define CONFIG_DPD
#endif
#ifndef CONFIG_FORCE_DPD_CALIB
    #define CONFIG_FORCE_DPD_CALIB
#endif
/* SDIO pwrctrl compile macros are enabled by default in SDIO mode only:
 * CONFIG_SDIO_BUS_PWRCTRL must not be defined in USB mode, otherwise
 * 1) the USB part of hal_co.c uses #ifndef CONFIG_SDIO_BUS_PWRCTRL to decide
 *    whether g_aic_pwrctl_enable is defined, and would skip it once defined;
 * 2) the call sites in fhost_tx/fhost_rx/wifi_api are gated by that macro,
 *    so a USB build would wrongly enable the SDIO pwrctl call path */
#if !defined(CONFIG_USB_SUPPORT)
    #ifndef CONFIG_SDIO_BUS_PWRCTRL
        #define CONFIG_SDIO_BUS_PWRCTRL
    #endif
    #ifndef CONFIG_SDIO_BUS_PWRCTRL_DYN
        #define CONFIG_SDIO_BUS_PWRCTRL_DYN
    #endif
#endif /* !CONFIG_USB_SUPPORT */


/* ============================================================================
 * Firmware loading
 *
 * CONFIG_LOAD_FW:
 *   0 = filemode   load from file system
 *   1 = flashmode  load from Flash
 *   2 = innermode  embedded firmware array
 *
 * Note: CONFIG_LOAD_FW is a platform-layer firmware source decision, passed by
 * platform_config.mk via -DCONFIG_LOAD_FW=n; the host layer does not depend on
 * it and provides no default here.
 * ============================================================================ */
#ifndef CONFIG_DOWNLOAD_FW
    #define CONFIG_DOWNLOAD_FW
#endif


/* ============================================================================
 * Protocol stack / software configuration
 * ============================================================================ */
#ifndef CONFIG_RWNX_FULLMAC
    #define CONFIG_RWNX_FULLMAC
#endif
#ifndef CONFIG_LWIP
    #define CONFIG_LWIP
#endif
#ifndef CONFIG_REORD_FORWARD_LIST
    #define CONFIG_REORD_FORWARD_LIST
#endif
#ifndef CONFIG_MBEDTLS
    #define CONFIG_MBEDTLS
#endif
#ifndef CONFIG_MBEDTLS_V3
    #define CONFIG_MBEDTLS_V3
#endif
#ifndef CONFIG_DRIVER_EDCA
    #define CONFIG_DRIVER_EDCA
#endif
#ifndef CONFIG_SOCK_SENDTO
    #define CONFIG_SOCK_SENDTO
#endif
#ifndef CONFIG_SOCK_RTOS_AL
    #define CONFIG_SOCK_RTOS_AL
#endif
#ifndef CONFIG_RTOS_RESLIST
    #define CONFIG_RTOS_RESLIST
#endif
#ifndef SDIO_FC_NARB
    #define SDIO_FC_NARB
#endif
//#define CONFIG_FILTER_TCP_ACK


/* ============================================================================
 * Debug
 * ============================================================================ */
#ifndef AIC_LOG_DEBUG_ON
    #define AIC_LOG_DEBUG_ON
#endif

/* Compile-time minimum log severity (default: all levels).
 * Reduce ROM by setting a higher threshold in CFLAGS:
 *   -DCONFIG_LOG_LEVEL=3   → strip INF+VRB format strings & code
 *   -DCONFIG_LOG_LEVEL=2   → strip WRN+INF+VRB
 * Values: 1=CRT 2=ERR 3=WRN 4=INF 5=VRB
 *
 * Compile-time module filter (default: all modules).
 * Lop off unused modules in CFLAGS:
 *   -DCONFIG_LOG_MODULES=0x00000C00   → FHOST+FMAC only (bits 10,11) */
#ifndef CONFIG_DRIVER_ORM
    #define CONFIG_DRIVER_ORM
#endif
#ifndef CONFIG_RTOS_TRACE
    #define CONFIG_RTOS_TRACE        /* not referenced by current code, reserved */
#endif


/* ============================================================================
 * Platform adaptation
 * ============================================================================ */
#ifndef PLATFORM_NO_FILE
    #define PLATFORM_NO_FILE         /* no stdio FILE in RTOS/bare-metal environments */
#endif


/* ============================================================================
 * Basic features (AP / STA — cannot be trimmed)
 * ============================================================================ */
#ifndef CONFIG_STATION
    #define CONFIG_STATION
#endif
#ifndef CONFIG_SOFTAP
    #define CONFIG_SOFTAP
#endif
#ifndef CONFIG_OFFCHANNEL
    #define CONFIG_OFFCHANNEL        /* low-level MAC feature, may be used by both AP and STA */
#endif


/* ============================================================================
 * Optional features (P2P / BT — controlled by AIC_FEATURE_P2P / AIC_FEATURE_BT in aic_cfg.mk)
 * ============================================================================ */
/* P2P */
#ifdef AIC_FEATURE_P2P
    #ifndef CONFIG_P2P_SUPPORT
        #define CONFIG_P2P_SUPPORT
    #endif
    /*
    * Note: after detecting CONFIG_P2P_SUPPORT, build_config.h chains in:
    *   CONFIG_P2P            → wpa_supplicant P2P core
    *   CONFIG_WIFI_DISPLAY   → Wi-Fi Display
    *   CONFIG_GAS            → Generic Advertisement Service
    * and rwnx_config.h defines, after detecting CONFIG_P2P_SUPPORT:
    *   NX_P2P               → LMAC P2P support
    *   CFG_P2P              → FMAC P2P support
    */
#endif /* AIC_FEATURE_P2P */

/* Bluetooth */
#ifdef AIC_FEATURE_BT
    #ifndef CONFIG_BT_SUPPORT
        #define CONFIG_BT_SUPPORT
    #endif
#endif /* AIC_FEATURE_BT */


#endif /* _AIC_CFG_H_ */
