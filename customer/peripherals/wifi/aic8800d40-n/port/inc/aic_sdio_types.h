/*
 * Copyright (C) 2018-2025 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-owned SDIO type definitions.
 * Provides SDIO constants without platform dependencies.
 */

#ifndef _AIC_SDIO_TYPES_H_
#define _AIC_SDIO_TYPES_H_

#include <stdint.h>

/* SDIO function numbers */
#define FUNC_0              0
#define FUNC_1              1
#define FUNC_2              2
#define SDIOM_MAX_FUNCS     3

/* TX slist max entries */
#define SDIO_TX_SLIST_MAX   136

/* SDIO scatter-gather list (ADMA mode)
 *
 * The field names must match the host-side usage: layer_common/src/sdio_co.c
 * writes sdiodev->sg_list[i].buf / .len when building frames
 * (an older version used addr here, so the host did not build with
 * CONFIG_SDIO_ADMA enabled).
 * The host drv_api/inc/netif_def.h also fills data segments by `buf` semantics.
 *
 * If a platform already has its own sg type (HAL provides a type with the same
 * name and layout), it may define AICWF_SDIO_SG_LIST_EXTERNAL in the platform
 * port header and include the platform HAL header first, skipping this
 * definition and avoiding a duplicate declaration (#256 invalid redeclaration).
 * Example (ASR/064 platform):
 *   #define AICWF_SDIO_SG_LIST_EXTERNAL
 *   #include "sdio.h"   // typedef struct { unsigned char *buf; unsigned int len; } sdio_sg_list;
 * Constraint: the platform type must match the host semantics -- the first
 * member is the segment start address, the second is the segment length.
 */
#ifdef CONFIG_SDIO_ADMA
#ifndef AICWF_SDIO_SG_LIST_EXTERNAL
typedef struct
{
    void     *buf;   /* segment start address (host: sdiodev->sg_list[i].buf) */
    uint32_t  len;   /* segment length (bytes) */
} sdio_sg_list;
#endif /* AICWF_SDIO_SG_LIST_EXTERNAL */
#endif /* CONFIG_SDIO_ADMA */

#endif /* _AIC_SDIO_TYPES_H_ */
