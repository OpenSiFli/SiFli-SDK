/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * AIC module-level logging interface.
 * Provides DBG_MODULE_SEVERITY macros with compile-time + runtime filtering.
 *
 * Compile-time gates (preprocessor #if — format strings eliminated when disabled):
 *   CONFIG_LOG_LEVEL   = 1..5  minimum severity (default 5=VRB all)
 *   CONFIG_LOG_MODULES  = bitmask of enabled modules (default 0xFFFFFFFF all)
 *
 * Runtime gate:
 *   platform_test_module_severity()  per-module+per-severity filter
 *
 * Output gate:
 *   AIC_LOG_DEBUG_ON  final printf dispatch
 */

#ifndef _AIC_LOG_H_
#define _AIC_LOG_H_

#include "plat_port.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * Compile-time severity thresholds (for #if comparison)
 * ================================================================ */
#define AIC_SEV_CRT  1
#define AIC_SEV_ERR  2
#define AIC_SEV_WRN  3
#define AIC_SEV_INF  4
#define AIC_SEV_VRB  5

#ifndef CONFIG_LOG_LEVEL
#define CONFIG_LOG_LEVEL  AIC_SEV_VRB
#endif

/* ================================================================
 * Compile-time module bit constants (for #if comparison)
 * ================================================================ */
#define AIC_MOD_CMN     (1 << 0)
#define AIC_MOD_OSAL    (1 << 1)
#define AIC_MOD_NETAL   (1 << 2)
#define AIC_MOD_XHCI    (1 << 3)
#define AIC_MOD_SDIO    (1 << 4)
#define AIC_MOD_USB     (1 << 5)
#define AIC_MOD_MACIF   (1 << 6)
#define AIC_MOD_RDATA   (1 << 7)
#define AIC_MOD_VNET    (1 << 8)
#define AIC_MOD_FMAC    (1 << 9)
#define AIC_MOD_FHOST   (1 << 10)
#define AIC_MOD_APP     (1 << 11)
#define AIC_MOD_WPA     (1 << 12)
#define AIC_MOD_NIM     (1 << 13)
#define AIC_MOD_BT      (1 << 14)

#ifndef CONFIG_LOG_MODULES
#define CONFIG_LOG_MODULES  0xFFFFFFFF
#endif

/* ================================================================
 * Module index enum
 * ================================================================ */
enum dbg_mod_tag
{
    DBG_MOD_IDX_CMN = 0,
    DBG_MOD_IDX_OSAL,
    DBG_MOD_IDX_NETAL,
    DBG_MOD_IDX_XHCI,
    DBG_MOD_IDX_SDIO,
    DBG_MOD_IDX_USB,
    DBG_MOD_IDX_MACIF,
    DBG_MOD_IDX_RDATA,
    DBG_MOD_IDX_VNET,
    DBG_MOD_IDX_FMAC,
    DBG_MOD_IDX_FHOST,
    DBG_MOD_IDX_APP,
    DBG_MOD_IDX_WPA,
    DBG_MOD_IDX_NIM,
    DBG_MOD_IDX_BT,
    DBG_MOD_IDX_MAX,
};

#define DBG_MOD_MIN     0x80
#define DBG_MOD_MAX     (DBG_MOD_MIN + DBG_MOD_IDX_MAX)
#define DBG_MOD_ALL     0xFFFFFFFF

/* ================================================================
 * Severity index enum
 * ================================================================ */
enum dbg_sev_tag
{
    DBG_SEV_IDX_NONE = 0,
    DBG_SEV_IDX_CRT  = 1,
    DBG_SEV_IDX_ERR  = 2,
    DBG_SEV_IDX_WRN  = 3,
    DBG_SEV_IDX_INF  = 4,
    DBG_SEV_IDX_VRB  = 5,
    DBG_SEV_IDX_MAX,
    DBG_SEV_ALL
};

#define DBG_SEV_MIN     0x9A
#define DBG_SEV_MAX     0xA0

/* ================================================================
 * Debug environment (runtime filter state)
 * ================================================================ */
struct debug_env_tag
{
    uint32_t    filter_module;
    uint32_t    filter_severity;
};

/* Runtime filter — weak stub in host, strong impl in platform */
extern int platform_test_module_severity(unsigned int mod_idx, unsigned int sev_idx);

/* ================================================================
 * Output backend
 * ================================================================ */
extern int aicwf_log_printf(const char *fmt, ...);

#ifdef AIC_LOG_DEBUG_ON
#define AIC_LOG_PRINTF(fmt, ...)    { aicwf_log_printf(fmt, ##__VA_ARGS__); }
#else
#define AIC_LOG_PRINTF(...)
#endif

#define OSAL_DBG_PRINTF             AIC_LOG_PRINTF

/* ================================================================
 * Inner template — runtime checks only
 * (compile-time gating already done by the outer #if blocks)
 * ================================================================ */
#define __DBG_MOD_SEV_TEMPLATE(mod, sev, fmt, ...)                          \
do {                                                                        \
    if (platform_test_module_severity(DBG_MOD_IDX_##mod, DBG_SEV_IDX_##sev)) {   \
        OSAL_DBG_PRINTF("[AIC]["#mod"]"fmt, ## __VA_ARGS__);                        \
    }                                                                       \
} while (0)

/* ================================================================
 * Per-module + per-severity macros
 *
 * Each macro is guarded by #if (CONFIG_LOG_LEVEL >= sev) &&
 * (CONFIG_LOG_MODULES & module_bit).  When either condition fails
 * the macro expands to nothing — format strings are consumed by the
 * preprocessor and never reach the compiler.
 * ================================================================ */

/* ---- CMN ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_CMN)
#define DBG_CMN_CRT(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(CMN, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_CMN_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_CMN)
#define DBG_CMN_ERR(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(CMN, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_CMN_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_CMN)
#define DBG_CMN_WRN(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(CMN, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_CMN_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_CMN)
#define DBG_CMN_INF(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(CMN, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_CMN_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_CMN)
#define DBG_CMN_VRB(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(CMN, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_CMN_VRB(fmt, ...)
#endif

/* ---- OSAL ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_OSAL)
#define DBG_OSAL_CRT(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(OSAL, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_OSAL_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_OSAL)
#define DBG_OSAL_ERR(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(OSAL, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_OSAL_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_OSAL)
#define DBG_OSAL_WRN(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(OSAL, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_OSAL_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_OSAL)
#define DBG_OSAL_INF(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(OSAL, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_OSAL_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_OSAL)
#define DBG_OSAL_VRB(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(OSAL, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_OSAL_VRB(fmt, ...)
#endif

/* ---- NETAL ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_NETAL)
#define DBG_NETAL_CRT(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(NETAL, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_NETAL_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_NETAL)
#define DBG_NETAL_ERR(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(NETAL, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_NETAL_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_NETAL)
#define DBG_NETAL_WRN(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(NETAL, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_NETAL_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_NETAL)
#define DBG_NETAL_INF(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(NETAL, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_NETAL_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_NETAL)
#define DBG_NETAL_VRB(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(NETAL, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_NETAL_VRB(fmt, ...)
#endif

/* ---- XHCI ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_XHCI)
#define DBG_XHCI_CRT(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(XHCI, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_XHCI_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_XHCI)
#define DBG_XHCI_ERR(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(XHCI, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_XHCI_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_XHCI)
#define DBG_XHCI_WRN(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(XHCI, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_XHCI_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_XHCI)
#define DBG_XHCI_INF(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(XHCI, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_XHCI_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_XHCI)
#define DBG_XHCI_VRB(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(XHCI, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_XHCI_VRB(fmt, ...)
#endif

/* ---- SDIO ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_SDIO)
#define DBG_SDIO_CRT(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(SDIO, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_SDIO_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_SDIO)
#define DBG_SDIO_ERR(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(SDIO, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_SDIO_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_SDIO)
#define DBG_SDIO_WRN(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(SDIO, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_SDIO_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_SDIO)
#define DBG_SDIO_INF(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(SDIO, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_SDIO_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_SDIO)
#define DBG_SDIO_VRB(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(SDIO, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_SDIO_VRB(fmt, ...)
#endif

/* ---- USB ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_USB)
#define DBG_USB_CRT(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(USB, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_USB_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_USB)
#define DBG_USB_ERR(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(USB, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_USB_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_USB)
#define DBG_USB_WRN(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(USB, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_USB_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_USB)
#define DBG_USB_INF(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(USB, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_USB_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_USB)
#define DBG_USB_VRB(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(USB, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_USB_VRB(fmt, ...)
#endif

/* ---- MACIF ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_MACIF)
#define DBG_MACIF_CRT(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(MACIF, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_MACIF_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_MACIF)
#define DBG_MACIF_ERR(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(MACIF, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_MACIF_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_MACIF)
#define DBG_MACIF_WRN(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(MACIF, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_MACIF_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_MACIF)
#define DBG_MACIF_INF(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(MACIF, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_MACIF_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_MACIF)
#define DBG_MACIF_VRB(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(MACIF, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_MACIF_VRB(fmt, ...)
#endif

/* ---- RDATA ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_RDATA)
#define DBG_RDATA_CRT(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(RDATA, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_RDATA_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_RDATA)
#define DBG_RDATA_ERR(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(RDATA, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_RDATA_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_RDATA)
#define DBG_RDATA_WRN(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(RDATA, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_RDATA_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_RDATA)
#define DBG_RDATA_INF(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(RDATA, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_RDATA_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_RDATA)
#define DBG_RDATA_VRB(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(RDATA, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_RDATA_VRB(fmt, ...)
#endif

/* ---- VNET ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_VNET)
#define DBG_VNET_CRT(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(VNET, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_VNET_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_VNET)
#define DBG_VNET_ERR(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(VNET, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_VNET_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_VNET)
#define DBG_VNET_WRN(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(VNET, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_VNET_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_VNET)
#define DBG_VNET_INF(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(VNET, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_VNET_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_VNET)
#define DBG_VNET_VRB(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(VNET, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_VNET_VRB(fmt, ...)
#endif

/* ---- FMAC ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_FMAC)
#define DBG_FMAC_CRT(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(FMAC, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_FMAC_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_FMAC)
#define DBG_FMAC_ERR(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(FMAC, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_FMAC_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_FMAC)
#define DBG_FMAC_WRN(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(FMAC, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_FMAC_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_FMAC)
#define DBG_FMAC_INF(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(FMAC, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_FMAC_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_FMAC)
#define DBG_FMAC_VRB(fmt, ...)   __DBG_MOD_SEV_TEMPLATE(FMAC, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_FMAC_VRB(fmt, ...)
#endif

/* ---- FHOST ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_FHOST)
#define DBG_FHOST_CRT(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(FHOST, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_FHOST_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_FHOST)
#define DBG_FHOST_ERR(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(FHOST, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_FHOST_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_FHOST)
#define DBG_FHOST_WRN(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(FHOST, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_FHOST_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_FHOST)
#define DBG_FHOST_INF(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(FHOST, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_FHOST_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_FHOST)
#define DBG_FHOST_VRB(fmt, ...)  __DBG_MOD_SEV_TEMPLATE(FHOST, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_FHOST_VRB(fmt, ...)
#endif

/* ---- APP ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_APP)
#define DBG_APP_CRT(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(APP, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_APP_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_APP)
#define DBG_APP_ERR(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(APP, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_APP_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_APP)
#define DBG_APP_WRN(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(APP, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_APP_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_APP)
#define DBG_APP_INF(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(APP, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_APP_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_APP)
#define DBG_APP_VRB(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(APP, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_APP_VRB(fmt, ...)
#endif

/* ---- WPA ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_WPA)
#define DBG_WPA_CRT(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(WPA, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_WPA_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_WPA)
#define DBG_WPA_ERR(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(WPA, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_WPA_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_WPA)
#define DBG_WPA_WRN(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(WPA, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_WPA_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_WPA)
#define DBG_WPA_INF(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(WPA, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_WPA_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_WPA)
#define DBG_WPA_VRB(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(WPA, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_WPA_VRB(fmt, ...)
#endif

/* ---- NIM (NimBLE host stack, STACK/nimble adapt) ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_NIM)
#define DBG_NIM_CRT(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(NIM, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_NIM_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_NIM)
#define DBG_NIM_ERR(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(NIM, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_NIM_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_NIM)
#define DBG_NIM_WRN(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(NIM, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_NIM_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_NIM)
#define DBG_NIM_INF(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(NIM, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_NIM_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_NIM)
#define DBG_NIM_VRB(fmt, ...)    __DBG_MOD_SEV_TEMPLATE(NIM, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_NIM_VRB(fmt, ...)
#endif

/* ---- BT (BT driver / HCI transport: host/driver/drv_bt, platform/bt) ---- */
#if (CONFIG_LOG_LEVEL >= AIC_SEV_CRT) && (CONFIG_LOG_MODULES & AIC_MOD_BT)
#define DBG_BT_CRT(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(BT, CRT, fmt, ## __VA_ARGS__)
#else
#define DBG_BT_CRT(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_ERR) && (CONFIG_LOG_MODULES & AIC_MOD_BT)
#define DBG_BT_ERR(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(BT, ERR, fmt, ## __VA_ARGS__)
#else
#define DBG_BT_ERR(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_WRN) && (CONFIG_LOG_MODULES & AIC_MOD_BT)
#define DBG_BT_WRN(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(BT, WRN, fmt, ## __VA_ARGS__)
#else
#define DBG_BT_WRN(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_INF) && (CONFIG_LOG_MODULES & AIC_MOD_BT)
#define DBG_BT_INF(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(BT, INF, fmt, ## __VA_ARGS__)
#else
#define DBG_BT_INF(fmt, ...)
#endif

#if (CONFIG_LOG_LEVEL >= AIC_SEV_VRB) && (CONFIG_LOG_MODULES & AIC_MOD_BT)
#define DBG_BT_VRB(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(BT, VRB, fmt, ## __VA_ARGS__)
#else
#define DBG_BT_VRB(fmt, ...)
#endif

/* ================================================================
 * Assertion macro
 * ================================================================ */
#define ASSERT_ERR(cond)                                                            \
    do {                                                                            \
        if (!(cond)) {                                                              \
            DBG_CMN_CRT("ASSERT_ERR(" #cond ") %s:%d\n", __FILE__, __LINE__);       \
        }                                                                           \
    } while(0)

#ifdef __cplusplus
}
#endif

#endif /* _AIC_LOG_H_ */
