/**
 * @file ark7128_i2c.h
 * @brief ARK7128 I2C control layer interface (driver-internal).
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @details
 * Board wiring (I2C/DCMI pins are configured by the board pinmux, see
 * customer/boards/sf32lb57-dpi-hdk_base/bsp_pinmux.c):
 *   - I2C bus name from Kconfig ARK7128_I2C_BUS_NAME (default "i2c1");
 *   - slave address 0x58;
 *   - camera analog input is VIN2, control register page 0x10 / reg 0x0B = 0x11;
 *   - the module has its own RC power-on reset and no MCU reset pin.
 */

#ifndef __ARK7128_I2C_H__
#define __ARK7128_I2C_H__

#include "ark7128.h"

/** @brief I2C bus device name, overridden by Kconfig ARK7128_I2C_BUS_NAME */
#ifndef ARK7128_I2C_BUS_NAME
    #define ARK7128_I2C_BUS_NAME        "i2c1"
#endif
/** @brief Default input channel: 0=VIN1 1=VIN2 2=VIN3 3=VIN4 */
#define ARK7128_INPUT_SEL           1U

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure the I2C1 pins and open the bus.
 *
 * @return Error code.
 * @retval RT_EOK Success, or already initialized.
 * @retval -RT_EIO I2C bus device not found.
 */
int  ark7128_i2c_init(void);

/**
 * @brief Shut down the I2C control layer.
 *
 * @note Only releases the internal handle; the pin mux is left in place.
 */
void ark7128_i2c_deinit(void);

/**
 * @brief Write one register.
 *
 * @param[in] reg Register address.
 * @param[in] val Value to write.
 *
 * @return Error code.
 * @retval RT_EOK Success.
 * @retval -RT_EIO Bus not initialized, or still failing after 5 retries.
 *
 * @note Not thread-safe on its own: the page register (0xE8) is shared, so a
 *       caller must hold the driver transaction lock. Prefer the higher-level
 *       ark7128_i2c_write_page() / ark7128_i2c_select_source().
 */
int  ark7128_i2c_write(uint8_t reg, uint8_t val);

/**
 * @brief Read one register.
 *
 * @param[in]  reg Register address.
 * @param[out] val Value read back.
 *
 * @return Error code.
 * @retval RT_EOK Success.
 * @retval -RT_EIO Bus not initialized, NULL argument, or still failing after
 *                 5 retries.
 *
 * @note Not thread-safe on its own: see ark7128_i2c_write().
 */
int  ark7128_i2c_read(uint8_t reg, uint8_t *val);

/**
 * @brief Download a run of [reg, val] pairs.
 *
 * @param[in] tbl   Register table, two bytes per entry.
 * @param[in] pairs Number of entries.
 *
 * @return Error code.
 * @retval RT_EOK Success.
 * @retval -RT_EINVAL NULL argument.
 * @retval -RT_EIO Any register write failed.
 *
 * @note Not thread-safe on its own: see ark7128_i2c_write().
 */
int  ark7128_i2c_write_pairs(const uint8_t *tbl, rt_uint32_t pairs);

/**
 * @brief Write 0xE8 to switch page, then write the target register.
 *
 * @param[in] page Page number.
 * @param[in] reg  Register address inside the page.
 * @param[in] val  Value to write.
 *
 * @return Error code.
 * @retval RT_EOK Success.
 * @retval -RT_EIO Write failed.
 */
int  ark7128_i2c_write_page(uint8_t page, uint8_t reg, uint8_t val);

/**
 * @brief Select the input channel.
 *
 * @param[in] source Channel number 0~3 (VIN1~VIN4).
 *
 * @return Error code.
 * @retval RT_EOK Success.
 * @retval -RT_EINVAL source is out of range.
 * @retval -RT_EIO Register write failed.
 */
int  ark7128_i2c_select_source(uint8_t source);

/**
 * @brief Soft-reset the decoder.
 *
 * @details Pulses bit1 of page0 0xA1 and then waits for fixed delays.
 */
void ark7128_i2c_decoder_reset(void);

/**
 * @brief Download the register table of the current format.
 *
 * @details Verifies the chip ID first, then writes the main register table, the
 *          extra registers and the default input channel.
 *
 * @return Error code.
 * @retval RT_EOK Success.
 * @retval -RT_EINVAL Format not configured.
 * @retval -RT_EIO Chip ID mismatch, or any register write failed.
 */
int  ark7128_i2c_config(void);

/**
 * @brief Read the chip ID.
 *
 * @return Chip ID; 0 when the read fails (never reported as 0x7128).
 */
uint16_t ark7128_i2c_detect_chip(void);

/**
 * @brief Read the raw value of page0 0x26.
 *
 * @param[out] raw Raw register value read back.
 *
 * @return Error code.
 * @retval RT_EOK Success.
 * @retval -RT_EINVAL NULL argument.
 * @retval -RT_EIO I2C read failed.
 */
int  ark7128_i2c_signal_raw(uint8_t *raw);

/**
 * @brief Read the signal lock state.
 *
 * @return Lock state.
 * @retval 1 Locked.
 * @retval 0 Unlocked, or the I2C read failed.
 */
uint8_t ark7128_i2c_signal_check(void);

/**
 * @brief Wait for a "stable lock".
 *
 * @details Five consecutive locked reads are required (a single read may only
 *          be a temporary lock).
 *
 * @param[in] timeout_ms Timeout in milliseconds; <= 0 uses 800 ms.
 *
 * @return Error code.
 * @retval RT_EOK Stably locked.
 * @retval -RT_ETIMEOUT Timeout.
 */
int  ark7128_i2c_wait_lock(rt_int32_t timeout_ms);

/**
 * @brief Get the video format selected at compile time.
 *
 * @return Current format; ARK7128_MODE_UNKNOWN when unconfigured.
 */
ark7128_mode_t ark7128_i2c_get_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARK7128_I2C_H__ */
