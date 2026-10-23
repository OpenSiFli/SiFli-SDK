/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform-independent type definitions for AIC8800 driver.
 * Replaces platform/inc/aic_types.h for standalone builds.
 */

#ifndef _AIC_PORTING_TYPES_H_
#define _AIC_PORTING_TYPES_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- Utility macros ---- */

#ifndef BIT
    #define BIT(x)      (1ul << (x))
#endif

#ifndef NULL
    #define NULL ((void *)0)
#endif

/* TRUE/FALSE macros — kept for wpa_supplicant third-party compatibility */
#define TRUE                1
#define FALSE               0

#ifndef pdTRUE
    #define pdTRUE              1
#endif
#ifndef pdFALSE
    #define pdFALSE             0
#endif

#ifndef ETH_ALEN
    #define ETH_ALEN 6
#endif

/* ---- WPA supplicant / legacy type aliases ---- */
/* wpa_supplicant third-party code uses u8/u16/u32/s8/s16/s32.
 * Define here so they're available before common/sae.h etc. */
#ifndef WPA_TYPES_DEFINED
    #define WPA_TYPES_DEFINED
    typedef uint8_t             u8;
    typedef uint16_t            u16;
    typedef uint32_t            u32;
    typedef uint64_t            u64;
    typedef int8_t              s8;
    typedef int16_t             s16;
    typedef int32_t             s32;
    typedef int64_t             s64;
#endif

/* ---- Linux kernel-style types (used by nl80211/linux_wext/netif_tcp) ---- */

typedef uint8_t             __u8;
typedef uint16_t            __u16;
typedef uint32_t            __u32;
typedef __u16               __be16;
typedef __u16               __sum16;
typedef __u32               __be32;

/* ---- AIC-specific types ---- */

typedef uint32_t            aic_gid_t;

/* AIC error type — platform-independent error code */
typedef int aic_err_t;
#define AIC_ERR_OK          0
#define AIC_ERR_MEM        -1
#define AIC_ERR_BUF        -2
#define AIC_ERR_TIMEOUT    -3
#define AIC_ERR_RTE        -4
#define AIC_ERR_INPROGRESS -5
#define AIC_ERR_VAL        -6
#define AIC_ERR_WOULDBLOCK -7
#define AIC_ERR_USE        -8
#define AIC_ERR_ALREADY    -9
#define AIC_ERR_ISCONN     -10
#define AIC_ERR_CONN       -11
#define AIC_ERR_IF         -12
#define AIC_ERR_ABRT       -13
#define AIC_ERR_RST        -14
#define AIC_ERR_CLSD       -15
#define AIC_ERR_ARG        -16

#include "aic_cfg.h"
#endif /* _AIC_PORTING_TYPES_H_ */
