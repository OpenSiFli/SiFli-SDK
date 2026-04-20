/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _OSAL_DEBUG_H_
#define _OSAL_DEBUG_H_

#include "osal_porting.h"

/** @addtogroup RTOS
 *  @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DEFINITIONS
 ****************************************************************************************
 */

/**
 * @name Debug Print definitions
 * @{
 ****************************************************************************************
 */

// Prefix used for module filtering
// If you modify any value, modify also DBG_MOD_ macros below
#define D_CMN       "\x80"   ///< Prefix for CMN
#define D_OSAL      "\x81"   ///< Prefix for OSAL
#define D_NETAL     "\x82"   ///< Prefix for NETAL
#define D_XHCI      "\x83"   ///< Prefix for XHCI
#define D_SDIO      "\x84"   ///< Prefix for SDIO
#define D_USB       "\x85"   ///< Prefix for USB
#define D_MACIF     "\x86"   ///< Prefix for MACIF
#define D_RDATA     "\x87"   ///< Prefix for RDATA
#define D_VNET      "\x88"   ///< Prefix for VNET
#define D_FMAC      "\x89"   ///< Prefix for FMAC
#define D_FHOST     "\x8A"   ///< Prefix for FHOST
#define D_APP       "\x8B"   ///< Prefix for APP
#define D_XC        "\x8C"   ///< Prefix unused
#define D_XD        "\x8D"   ///< Prefix unused
#define D_XE        "\x8E"   ///< Prefix unused
#define D_XF        "\x8F"   ///< Prefix unused

// Prefix used for severity filtering
// If you modify any value, modify also DBG_SEV_ macros below
#define D_CRT       "\x9A"   ///< Prefix for critical
#define D_ERR       "\x9B"   ///< Prefix for error
#define D_WRN       "\x9C"   ///< Prefix for warning
#define D_INF       "\x9D"   ///< Prefix for info
#define D_VRB       "\x9E"   ///< Prefix for verbose debug

/// Module filtering macros, used only by debug module
enum dbg_mod_tag
{
    DBG_MOD_IDX_CMN = 0,  ///< Bit index for CMN
    DBG_MOD_IDX_OSAL,     ///< Bit index for OSAL
    DBG_MOD_IDX_NETAL,    ///< Bit index for NETAL
    DBG_MOD_IDX_XHCI,     ///< Bit index for XHCI
    DBG_MOD_IDX_SDIO,     ///< Bit index for SDIO
    DBG_MOD_IDX_USB,      ///< Bit index for USB
    DBG_MOD_IDX_MACIF,    ///< Bit index for MACIF
    DBG_MOD_IDX_RDATA,    ///< Bit index for RDATA
    DBG_MOD_IDX_VNET,     ///< Bit index for VNET
    DBG_MOD_IDX_FMAC,     ///< Bit index for FMAC
    DBG_MOD_IDX_FHOST,    ///< Bit index for FHOST
    DBG_MOD_IDX_APP,      ///< Bit index for APP
    DBG_MOD_IDX_MAX,      ///< Number of modules
};

#define DBG_MOD_MIN     0x80
#define DBG_MOD_MAX     (DBG_MOD_MIN + DBG_MOD_IDX_MAX)

#define DBG_MOD_ALL         0xFFFFFFFF


/// Severity filtering macros, used only by debug module
enum dbg_sev_tag
{
    DBG_SEV_IDX_NONE = 0,   ///< No print allowed
    DBG_SEV_IDX_CRT  = 1,   ///< Critical and unspecified allowed only
    DBG_SEV_IDX_ERR  = 2,   ///< Error allowed and above
    DBG_SEV_IDX_WRN  = 3,   ///< Warning allowed and above
    DBG_SEV_IDX_INF  = 4,   ///< Info allowed and above
    DBG_SEV_IDX_VRB  = 5,   ///< All allowed
    DBG_SEV_IDX_MAX,        ///< Number of severity levels
    DBG_SEV_ALL             ///< Convenient macro
};

#define DBG_SEV_MIN     0x9A
#define DBG_SEV_MAX     0xA0

#define OSAL_DBG_PRINTF(fmt, ...)                               \
do {                                                            \
    if (osal_porting_ops.osal_dbg_printf) {                     \
        osal_porting_ops.osal_dbg_printf(fmt, ## __VA_ARGS__);  \
    }                                                           \
} while (0)

/**
 *****************************************************************************************
 * @brief Declaration of DEBUG environment.
 * Any module or task will retrieve the DEBUG environment on its own, it is
 * not passed anymore as a parameter to the function handlers
 * of the Debug task.  If an other module wants to make use of the
 * environment of debug, it simply needs to include this file and use
 * the extern global variable.
 *****************************************************************************************
 */
struct debug_env_tag
{
    /// User trace filter (bit set means traces enabled)
    uint32_t    filter_module;
    /// Severity of filter
    uint32_t    filter_severity;
};

/*
 * FUNCTIONS
 ****************************************************************************************
 */

int dbg_test_module_severity(unsigned int mod_idx, unsigned int sev_idx);


#define __DBG_MOD_SEV_TEMPLATE(mod, sev, fmt, ...)                          \
do {                                                                        \
    if (dbg_test_module_severity(DBG_MOD_IDX_##mod, DBG_SEV_IDX_##sev)) {   \
        OSAL_DBG_PRINTF(fmt, ## __VA_ARGS__);                               \
    }                                                                       \
} while (0)


#define DBG_CMN_CRT(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(CMN, CRT, fmt, ## __VA_ARGS__)
#define DBG_CMN_ERR(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(CMN, ERR, fmt, ## __VA_ARGS__)
#define DBG_CMN_WRN(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(CMN, WRN, fmt, ## __VA_ARGS__)
#define DBG_CMN_INF(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(CMN, INF, fmt, ## __VA_ARGS__)
#define DBG_CMN_VRB(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(CMN, VRB, fmt, ## __VA_ARGS__)

#define DBG_OSAL_CRT(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(OSAL, CRT, fmt, ## __VA_ARGS__)
#define DBG_OSAL_ERR(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(OSAL, ERR, fmt, ## __VA_ARGS__)
#define DBG_OSAL_WRN(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(OSAL, WRN, fmt, ## __VA_ARGS__)
#define DBG_OSAL_INF(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(OSAL, INF, fmt, ## __VA_ARGS__)
#define DBG_OSAL_VRB(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(OSAL, VRB, fmt, ## __VA_ARGS__)

#define DBG_NETAL_CRT(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(NETAL, CRT, fmt, ## __VA_ARGS__)
#define DBG_NETAL_ERR(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(NETAL, ERR, fmt, ## __VA_ARGS__)
#define DBG_NETAL_WRN(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(NETAL, WRN, fmt, ## __VA_ARGS__)
#define DBG_NETAL_INF(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(NETAL, INF, fmt, ## __VA_ARGS__)
#define DBG_NETAL_VRB(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(NETAL, VRB, fmt, ## __VA_ARGS__)

#define DBG_XHCI_CRT(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(XHCI, CRT, fmt, ## __VA_ARGS__)
#define DBG_XHCI_ERR(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(XHCI, ERR, fmt, ## __VA_ARGS__)
#define DBG_XHCI_WRN(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(XHCI, WRN, fmt, ## __VA_ARGS__)
#define DBG_XHCI_INF(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(XHCI, INF, fmt, ## __VA_ARGS__)
#define DBG_XHCI_VRB(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(XHCI, VRB, fmt, ## __VA_ARGS__)

#define DBG_SDIO_CRT(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(SDIO, CRT, fmt, ## __VA_ARGS__)
#define DBG_SDIO_ERR(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(SDIO, ERR, fmt, ## __VA_ARGS__)
#define DBG_SDIO_WRN(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(SDIO, WRN, fmt, ## __VA_ARGS__)
#define DBG_SDIO_INF(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(SDIO, INF, fmt, ## __VA_ARGS__)
#define DBG_SDIO_VRB(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(SDIO, VRB, fmt, ## __VA_ARGS__)

#define DBG_MACIF_CRT(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(MACIF, CRT, fmt, ## __VA_ARGS__)
#define DBG_MACIF_ERR(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(MACIF, ERR, fmt, ## __VA_ARGS__)
#define DBG_MACIF_WRN(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(MACIF, WRN, fmt, ## __VA_ARGS__)
#define DBG_MACIF_INF(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(MACIF, INF, fmt, ## __VA_ARGS__)
#define DBG_MACIF_VRB(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(MACIF, VRB, fmt, ## __VA_ARGS__)

#define DBG_RDATA_CRT(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(RDATA, CRT, fmt, ## __VA_ARGS__)
#define DBG_RDATA_ERR(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(RDATA, ERR, fmt, ## __VA_ARGS__)
#define DBG_RDATA_WRN(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(RDATA, WRN, fmt, ## __VA_ARGS__)
#define DBG_RDATA_INF(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(RDATA, INF, fmt, ## __VA_ARGS__)
#define DBG_RDATA_VRB(fmt, ...)     __DBG_MOD_SEV_TEMPLATE(RDATA, VRB, fmt, ## __VA_ARGS__)

#define DBG_VNET_CRT(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(VNET, CRT, fmt, ## __VA_ARGS__)
#define DBG_VNET_ERR(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(VNET, ERR, fmt, ## __VA_ARGS__)
#define DBG_VNET_WRN(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(VNET, WRN, fmt, ## __VA_ARGS__)
#define DBG_VNET_INF(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(VNET, INF, fmt, ## __VA_ARGS__)
#define DBG_VNET_VRB(fmt, ...)      __DBG_MOD_SEV_TEMPLATE(VNET, VRB, fmt, ## __VA_ARGS__)

#define DBG_APP_CRT(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(APP, CRT, fmt, ## __VA_ARGS__)
#define DBG_APP_ERR(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(APP, ERR, fmt, ## __VA_ARGS__)
#define DBG_APP_WRN(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(APP, WRN, fmt, ## __VA_ARGS__)
#define DBG_APP_INF(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(APP, INF, fmt, ## __VA_ARGS__)
#define DBG_APP_VRB(fmt, ...)       __DBG_MOD_SEV_TEMPLATE(APP, VRB, fmt, ## __VA_ARGS__)

/// Assertions showing a critical error that could require a full system reset
#define ASSERT_ERR(cond)                                                                \
    do {                                                                                \
        if (!(cond)) {                                                                  \
            DBG_CMN_CRT("ASSERT_ERR("  #cond ") %s:%d\n", __BASE_FILE__, __LINE__);     \
        }                                                                               \
    } while(0)

#ifdef __cplusplus
}
#endif

/*\@}*/

#endif /* _OSAL_DEBUG_H_ */
