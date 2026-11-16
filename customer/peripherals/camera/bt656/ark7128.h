/**
 * @file ark7128.h
 * @brief ARK7128 chip adaptation layer internal interface.
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @details
 * This file is for driver-internal use only. Applications include bt656.h, not
 * this file.
 *
 * Layers:
 *   - bt656.c        Generic BT.656 capture layer (DCMI/DMA/buffer pool)
 *   - ark7128.c      Chip adaptation: I2C, register table, format, pins, and
 *                    registration with bt656
 *   - ark7128_cfg.c  Register tables and geometry for each input format
 *   - ark7128_i2c.c  Paged register access and lock detection
 *
 * @defgroup ARK7128 ARK7128 video decoder chip adaptation
 * @{
 */

#ifndef __ARK7128_H__
#define __ARK7128_H__

#include <rtthread.h>
#include "bt656.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Slave address: 38pin package tied to GND */
#define ARK7128_SLAVE_ADDR  (0xB0 >> 1)
/** @brief Expected chip ID */
#define ARK7128_CHIP_ID     (0x7128)

/**
 * @brief Video input format.
 *
 * Selected by menuconfig under @c ARK7128 @c -> @c Video\ input\ format.
 */
typedef enum
{
    ARK7128_MODE_UNKNOWN = 0,   /**< Unconfigured or unknown format */
    ARK7128_MODE_CVBS_PAL,      /**< CVBS PAL, 720x288 single field */
    ARK7128_MODE_CVBS_NTSC,     /**< CVBS NTSC, 720x240 single field */
    ARK7128_MODE_AHD_720P25,    /**< AHD 720P 25fps, 1280x720 full frame */
    ARK7128_MODE_AHD_720P30,    /**< AHD 720P 30fps, 1280x720 full frame */
} ark7128_mode_t;

/**
 * @brief Register ARK7128 with the generic BT.656 layer.
 *
 * @details Called automatically by INIT_BOARD_EXPORT; applications do not call
 *          it manually.
 *
 * @return Error code.
 * @retval RT_EOK Registered successfully.
 * @retval -RT_EBUSY Another chip is already registered.
 */
int ark7128_register(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* __ARK7128_H__ */
