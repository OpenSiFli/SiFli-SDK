/**
 * @file bt656_decoder.h
 * @brief BT.656 decoder chip adaptation interface (driver-internal).
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @details
 * Only chip adaptation layers (for example ark7128.c) and bt656.c include this
 * file; applications must not.
 *
 * To add a BT.656 decoder chip, implement one bt656_decoder_ops and call
 * bt656_register_decoder() during INIT_BOARD_EXPORT. Neither the application nor
 * bt656.c needs to change. Only one chip is supported at a time.
 *
 * @defgroup BT656_DECODER BT.656 decoder chip adaptation
 * @{
 */

#ifndef __BT656_DECODER_H__
#define __BT656_DECODER_H__

#include "bt656.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Decoder chip operation table.
 *
 * @details Every callback may be NULL; bt656.c skips missing optional
 *          operations.
 */
struct bt656_decoder_ops
{
    const char *name;   /**< Chip name, used for logging only */

    /**
     * @brief Initialize the chip (I2C, pins, read ID, first register table load).
     * @return Error code.
     */
    int (*init)(void);

    /**
     * @brief Shut down the chip (release I2C and so on).
     * @return Error code.
     */
    int (*deinit)(void);

    /**
     * @brief Reconfigure the chip for a format (soft reset + reload register table).
     * @param[in] fmt Target format.
     * @return Error code.
     */
    int (*configure)(const struct bt656_format *fmt);

    /**
     * @brief Wait for the decoder to lock stably.
     * @param[in] timeout_ms Timeout in milliseconds; <= 0 uses the chip default.
     * @return Error code.
     */
    int (*wait_ready)(rt_int32_t timeout_ms);

    /**
     * @brief Select the analog input channel.
     * @param[in] source Channel number.
     * @return Error code.
     */
    int (*set_source)(rt_uint8_t source);

    /**
     * @brief Read the chip ID.
     * @return Chip ID; 0 on failure.
     */
    rt_uint16_t (*detect)(void);

    /**
     * @brief Read the signal lock state.
     * @return 1 when locked, 0 when unlocked.
     */
    rt_uint8_t (*signal_locked)(void);

    /**
     * @brief Get the format selected at compile time.
     * @return Format pointer; must not be NULL.
     */
    const struct bt656_format *(*format)(void);

    void       *buffer;         /**< Buffer pool base, at least buffer_count * buffer_stride bytes */
    rt_uint32_t buffer_count;   /**< Number of blocks, 2 ~ 32 */
    rt_uint32_t buffer_stride;  /**< Bytes actually occupied per block (>= frame_bytes) */
};

/**
 * @brief Register the single BT.656 decoder chip implementation.
 *
 * @param[in] ops Chip operation table, must stay valid for the lifetime.
 *
 * @return Error code.
 * @retval RT_EOK Registered successfully.
 * @retval -RT_EINVAL Invalid argument (ops is NULL, format is NULL, buffers too small).
 * @retval -RT_EBUSY Another chip is already registered.
 */
int bt656_register_decoder(const struct bt656_decoder_ops *ops);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* __BT656_DECODER_H__ */
