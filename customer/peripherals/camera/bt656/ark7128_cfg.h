/**
 * @file ark7128_cfg.h
 * @brief ARK7128 per-format configuration data structures (driver-internal).
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ARK7128_CFG_H__
#define __ARK7128_CFG_H__

#include <rtthread.h>
#include "ark7128.h"

/**
 * @brief One register operation.
 *
 * @details When the page changes, 0xE8 is written first, then the register
 *          operation is performed.
 */
struct ark7128_reg_op
{
    rt_uint8_t page;    /**< Register page number */
    rt_uint8_t reg;     /**< Register address inside the page */
    rt_uint8_t val;     /**< Value to write (ignored by RMW operations) */
    rt_uint8_t op;      /**< Operation type, see ARK7128_OP_xxx */
};

/** @brief Write val directly */
#define ARK7128_OP_WRITE        0U
/** @brief Read-modify-write: (v & 0xCF) | 0x20 */
#define ARK7128_OP_RMW_E4       1U

/**
 * @brief Complete description of one video input format.
 *
 * @details Geometry + decoder register table + extra operations + DCMI sync
 *          codes.
 */
struct ark7128_format
{
    struct bt656_format info;       /**< Generic geometry, must stay first */

    ark7128_mode_t mode;            /**< Chip format enumeration */

    const rt_uint8_t (*reg_tbl)[2]; /**< Main register table: [reg, val] pairs */
    rt_uint16_t reg_pairs;          /**< Number of pairs in the main table */

    const struct ark7128_reg_op *extra; /**< Format-specific extra operations */
    rt_uint16_t extra_cnt;              /**< Number of extra operations */
};

/**
 * @brief Get the format selected by menuconfig (ARK7128 -> Video input format).
 *
 * @return Format description pointer; never NULL.
 */
const struct ark7128_format *ark7128_get_format(void);

/**
 * @brief Look up a format by mode.
 *
 * @param[in] mode Video format.
 *
 * @return Format description pointer; NULL when not found.
 */
const struct ark7128_format *ark7128_format_by_mode(ark7128_mode_t mode);

#endif /* __ARK7128_CFG_H__ */
