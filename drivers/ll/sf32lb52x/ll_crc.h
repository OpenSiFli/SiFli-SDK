/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_CRC_H
#define __LL_CRC_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_crc.h
 * @brief Header-only low-level CRC APIs for SF32LB52x.
 */

/** @defgroup LL_CRC_DATASIZE LL CRC Input Data Size */
/** @{ */
#define LL_CRC_DATASIZE_8B  (0x0UL << CRC_CR_DATASIZE_Pos)
#define LL_CRC_DATASIZE_16B (0x1UL << CRC_CR_DATASIZE_Pos)
#define LL_CRC_DATASIZE_24B (0x2UL << CRC_CR_DATASIZE_Pos)
#define LL_CRC_DATASIZE_32B (0x3UL << CRC_CR_DATASIZE_Pos)
/** @} */

/** @defgroup LL_CRC_POLYSIZE LL CRC Polynomial Size */
/** @{ */
#define LL_CRC_POLYSIZE_32B (0x0UL << CRC_CR_POLYSIZE_Pos)
#define LL_CRC_POLYSIZE_16B (0x1UL << CRC_CR_POLYSIZE_Pos)
#define LL_CRC_POLYSIZE_8B  (0x2UL << CRC_CR_POLYSIZE_Pos)
#define LL_CRC_POLYSIZE_7B  (0x3UL << CRC_CR_POLYSIZE_Pos)
/** @} */

/** @defgroup LL_CRC_REVIN LL CRC Input Bit Reversal */
/** @{ */
#define LL_CRC_REV_IN_NONE        (0x0UL << CRC_CR_REV_IN_Pos)
#define LL_CRC_REV_IN_BY_BYTE     (0x1UL << CRC_CR_REV_IN_Pos)
#define LL_CRC_REV_IN_BY_HALFWORD (0x2UL << CRC_CR_REV_IN_Pos)
#define LL_CRC_REV_IN_BY_WORD     (0x3UL << CRC_CR_REV_IN_Pos)
/** @} */

/** @defgroup LL_CRC_REVOUT LL CRC Output Bit Reversal */
/** @{ */
#define LL_CRC_REV_OUT_DISABLE (0x0UL << CRC_CR_REV_OUT_Pos)
#define LL_CRC_REV_OUT_ENABLE  (0x1UL << CRC_CR_REV_OUT_Pos)
/** @} */

/**
 * @brief CRC control register configuration.
 */
typedef struct
{
    uint32_t data_size; /**< Input data width, use @ref LL_CRC_DATASIZE_8B to @ref
                           LL_CRC_DATASIZE_32B. */
    uint32_t poly_size; /**< Polynomial width, use @ref LL_CRC_POLYSIZE_32B to @ref
                           LL_CRC_POLYSIZE_7B. */
    uint32_t rev_in;    /**< Input bit reversal, use @ref LL_CRC_REV_IN_NONE to @ref
                           LL_CRC_REV_IN_BY_WORD. */
    uint32_t rev_out;   /**< Output bit reversal, use @ref LL_CRC_REV_OUT_DISABLE or
                           @ref LL_CRC_REV_OUT_ENABLE. */
} ll_crc_ctrl_config_t;

/**
 * @brief Set CRC initial value register.
 * @param[in] CRCx CRC instance pointer.
 * @param[in] init Initial value.
 */
static inline void ll_crc_set_init(CRC_TypeDef *CRCx, uint32_t init)
{
    WRITE_REG(CRCx->INIT, init);
}

/**
 * @brief Set CRC polynomial register.
 * @param[in] CRCx CRC instance pointer.
 * @param[in] poly Polynomial value.
 */
static inline void ll_crc_set_poly(CRC_TypeDef *CRCx, uint32_t poly)
{
    WRITE_REG(CRCx->POL, poly);
}

/**
 * @brief Configure CRC control fields DATASIZE/POLYSIZE/REV_IN/REV_OUT.
 * @param[in] CRCx CRC instance pointer.
 * @param[in] cfg Pointer to control configuration.
 */
static inline void ll_crc_config_ctrl(CRC_TypeDef *CRCx,
                                      const ll_crc_ctrl_config_t *cfg)
{
    MODIFY_REG(CRCx->CR,
               CRC_CR_DATASIZE | CRC_CR_POLYSIZE | CRC_CR_REV_IN | CRC_CR_REV_OUT,
               cfg->data_size | cfg->poly_size | cfg->rev_in | cfg->rev_out);
}

/**
 * @brief Reset CRC calculation unit to INIT state.
 * @param[in] CRCx CRC instance pointer.
 */
static inline void ll_crc_reset(CRC_TypeDef *CRCx)
{
    SET_BIT(CRCx->CR, CRC_CR_RESET);
}

/**
 * @brief Push one 8-bit data item.
 * @param[in] CRCx CRC instance pointer.
 * @param[in] data 8-bit payload.
 */
static inline void ll_crc_push_data8(CRC_TypeDef *CRCx, uint8_t data)
{
    MODIFY_REG(CRCx->CR, CRC_CR_DATASIZE, LL_CRC_DATASIZE_8B);
    WRITE_REG(CRCx->DR, (uint32_t)data);
}

/**
 * @brief Push one 16-bit data item.
 * @param[in] CRCx CRC instance pointer.
 * @param[in] data 16-bit payload.
 */
static inline void ll_crc_push_data16(CRC_TypeDef *CRCx, uint16_t data)
{
    MODIFY_REG(CRCx->CR, CRC_CR_DATASIZE, LL_CRC_DATASIZE_16B);
    WRITE_REG(CRCx->DR, (uint32_t)data);
}

/**
 * @brief Push one 24-bit data item.
 * @param[in] CRCx CRC instance pointer.
 * @param[in] data24 24-bit payload in bits [23:0].
 */
static inline void ll_crc_push_data24(CRC_TypeDef *CRCx, uint32_t data24)
{
    MODIFY_REG(CRCx->CR, CRC_CR_DATASIZE, LL_CRC_DATASIZE_24B);
    WRITE_REG(CRCx->DR, (data24 & 0x00FFFFFFUL));
}

/**
 * @brief Push one 32-bit data item.
 * @param[in] CRCx CRC instance pointer.
 * @param[in] data 32-bit payload.
 */
static inline void ll_crc_push_data32(CRC_TypeDef *CRCx, uint32_t data)
{
    MODIFY_REG(CRCx->CR, CRC_CR_DATASIZE, LL_CRC_DATASIZE_32B);
    WRITE_REG(CRCx->DR, data);
}

/**
 * @brief Read current CRC result.
 * @param[in] CRCx CRC instance pointer.
 * @return Current CRC result register value.
 */
static inline uint32_t ll_crc_read_result(CRC_TypeDef *CRCx)
{
    return READ_REG(CRCx->DR);
}

/**
 * @brief Check DONE flag.
 * @param[in] CRCx CRC instance pointer.
 * @return Non-zero when DONE is set.
 */
static inline uint32_t ll_crc_is_active_flag_done(CRC_TypeDef *CRCx)
{
    return READ_BIT(CRCx->SR, CRC_SR_DONE);
}

/**
 * @brief Check OVERFLOW flag.
 * @param[in] CRCx CRC instance pointer.
 * @return Non-zero when OVERFLOW is set.
 */
static inline uint32_t ll_crc_is_active_flag_overflow(CRC_TypeDef *CRCx)
{
    return READ_BIT(CRCx->SR, CRC_SR_OVERFLOW);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_CRC_H */
