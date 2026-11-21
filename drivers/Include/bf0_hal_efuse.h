/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_EFUSE_H
#define __BF0_HAL_EFUSE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup EFUSE
 * @ingroup BF0_HAL_Driver
 * @{
 */

/** EFUSE bank size in bytes */
#define HAL_EFUSE_BANK_SIZE         32
/** EFUSE bank size in words, must be 2^n */
#define HAL_EFUSE_BANK_WORD_SIZE    8
/** EFUSE bank size in bits, must be 2^n */
#define HAL_EFUSE_BANK_BIT_SIZE     256
/** EFUSE bank number */
#ifdef SF32LB57X
#define HAL_EFUSE_BANK_NUM          (3)
#else
#define HAL_EFUSE_BANK_NUM          (4)
#endif

/**
 * @brief  Init Efuse controller
 * @retval void
 */
HAL_StatusTypeDef HAL_EFUSE_Init(void);

/**
 * @brief  Configure bypass
 * @param enabled true: enable bypass, false: disable bypass
 * @retval void
 */
void HAL_EFUSE_ConfigBypass(bool enabled);

/**
 * @brief  Write data to efuse starting from bit_offset
 * @param bit_offset bit_offset in efuse, must be 32bits aligned, bank0: 0~255, bank1: 256~511
 * @param data point to the data to be written
 * @param size data size in byte, must be multiple of 4bytes and written data cannot cross bank boundary
 * @retval size successfully written
 */
int32_t HAL_EFUSE_Write(uint16_t bit_offset, uint8_t *data, int32_t size);


/**
 * @brief  Read data to efuse starting from bit_offset
 * @param bit_offset bit_offset in efuse, must be 32bits aligned, bank0: 0~255, bank1: 256~511
 * @param data point to buffer to save read data
 * @param size data size in byte, must be multiple of 4bytes and read data cannot cross bank boundary
 * @retval size successfully read
 */
int32_t HAL_EFUSE_Read(uint16_t bit_offset, uint8_t *data, int size);

/**
 * @brief  Read data from efuse starting from bit_offset, no alignment requirement for bit_offset
 *
 * @param bit_offset bit_offset in efuse, bank0: 0~255, bank1: 256~511
 * @param data point to buffer to save read data
 * @param bit_size data size in bit to be read, read data cannot cross bank boundary
 * @return bit size successfully read
 */
int32_t HAL_EFUSE_Read2(uint16_t bit_offset, uint8_t *data, int bit_size);

/**
 * @brief  Extract data from provided bank data starting from bit_offset, no alignment requirement for bit_offset
 *
 * @param bank_data pointer to the whole bank data array, HAL_EFUSE_BANK_WORD_SIZE words are expected
 * @param bit_offset_in_bank bit_offset in one bank, range: 0~255
 * @param data point to buffer to save read data
 * @param bit_size data size in bit to be read, read data cannot cross bank boundary
 * @return bit size successfully read
 */
int32_t HAL_EFUSE_Extract(const uint32_t *bank_data, uint16_t bit_offset_in_bank, uint8_t *data, int bit_size);

/**
 * @brief  Write data to efuse starting from bit_offset, no alignment requirement for bit_offset
 *
 * @param bit_offset bit_offset in efuse, bank0: 0~255, bank1: 256~511
 * @param data point to the data to be written
 * @param bit_size data size in bit to be written, written data cannot cross bank boundary
 * @return bit size successfully written
 */
int32_t HAL_EFUSE_Write2(uint16_t bit_offset, uint8_t *data, int32_t bit_size);

#ifdef __cplusplus
}
#endif

///@}   EFUSE

#endif /* __BF0_HAL_EFUSE_H */