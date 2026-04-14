/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_EXTDMA_H
#define __LL_EXTDMA_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_extdma.h
 * @brief Header-only low-level ExtDMA APIs for SF32LB52x.
 */

/** @defgroup LL_EXTDMA_SRCBURST LL ExtDMA Source Burst */
/** @{ */
#define LL_EXTDMA_SRCBURST_SINGLE (0x0UL << EXTDMA_CCR_SRCBURST_Pos)
#define LL_EXTDMA_SRCBURST_INCR4 (0x1UL << EXTDMA_CCR_SRCBURST_Pos)
#define LL_EXTDMA_SRCBURST_INCR8 (0x2UL << EXTDMA_CCR_SRCBURST_Pos)
#define LL_EXTDMA_SRCBURST_INCR16 (0x3UL << EXTDMA_CCR_SRCBURST_Pos)
/** @} */

/** @defgroup LL_EXTDMA_DSTBURST LL ExtDMA Destination Burst */
/** @{ */
#define LL_EXTDMA_DSTBURST_SINGLE (0x0UL << EXTDMA_CCR_DSTBURST_Pos)
#define LL_EXTDMA_DSTBURST_INCR4 (0x1UL << EXTDMA_CCR_DSTBURST_Pos)
#define LL_EXTDMA_DSTBURST_INCR8 (0x2UL << EXTDMA_CCR_DSTBURST_Pos)
#define LL_EXTDMA_DSTBURST_INCR16 (0x3UL << EXTDMA_CCR_DSTBURST_Pos)
/** @} */

/** @defgroup LL_EXTDMA_SIZE LL ExtDMA Data Width (UM fixed to 32-bit) */
/** @{ */
#define LL_EXTDMA_SRCSIZE_WORD (0x2UL << EXTDMA_CCR_SRCSIZE_Pos)
#define LL_EXTDMA_DSTSIZE_WORD (0x2UL << EXTDMA_CCR_DSTSIZE_Pos)
/** @} */

/** @defgroup LL_EXTDMA_FLAG LL ExtDMA ISR Flags */
/** @{ */
#define LL_EXTDMA_FLAG_GIF EXTDMA_ISR_GIF
#define LL_EXTDMA_FLAG_TCIF EXTDMA_ISR_TCIF
#define LL_EXTDMA_FLAG_HTIF EXTDMA_ISR_HTIF
#define LL_EXTDMA_FLAG_TEIF EXTDMA_ISR_TEIF
/** @} */

/** @defgroup LL_EXTDMA_CLEAR LL ExtDMA IFCR Clear Bits */
/** @{ */
#define LL_EXTDMA_CLEAR_GIF EXTDMA_IFCR_CGIF
#define LL_EXTDMA_CLEAR_TCIF EXTDMA_IFCR_CTCIF
#define LL_EXTDMA_CLEAR_HTIF EXTDMA_IFCR_CHTIF
#define LL_EXTDMA_CLEAR_TEIF EXTDMA_IFCR_CTEIF
/** @} */

/**
 * @brief ExtDMA control register configuration.
 */
typedef struct
{
    uint32_t src_burst; /**< Source burst setting, use @ref
                           LL_EXTDMA_SRCBURST_SINGLE to @ref
                           LL_EXTDMA_SRCBURST_INCR16. */
    uint32_t dst_burst; /**< Destination burst setting, use @ref
                           LL_EXTDMA_DSTBURST_SINGLE to @ref
                           LL_EXTDMA_DSTBURST_INCR16. */
    uint32_t src_inc;   /**< Source increment control, 0 or
                           EXTDMA_CCR_SRCINC. */
    uint32_t dst_inc;   /**< Destination increment control, 0 or
                           EXTDMA_CCR_DSTINC. */
    uint32_t it_tc;     /**< Transfer complete interrupt, 0 or
                           EXTDMA_CCR_TCIE. */
    uint32_t it_ht;     /**< Half transfer interrupt, 0 or
                           EXTDMA_CCR_HTIE. */
    uint32_t it_te;     /**< Transfer error interrupt, 0 or
                           EXTDMA_CCR_TEIE. */
} ll_extdma_ctrl_config_t;

/**
 * @brief Set ExtDMA source address register.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @param[in] src_addr Source address (word aligned).
 */
static inline void ll_extdma_set_src_addr(EXTDMA_TypeDef *EXTDMAx,
                                          uint32_t src_addr)
{
    WRITE_REG(EXTDMAx->SRCAR, src_addr);
}

/**
 * @brief Set ExtDMA destination address register.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @param[in] dst_addr Destination address (word aligned).
 */
static inline void ll_extdma_set_dst_addr(EXTDMA_TypeDef *EXTDMAx,
                                          uint32_t dst_addr)
{
    WRITE_REG(EXTDMAx->DSTAR, dst_addr);
}

/**
 * @brief Set ExtDMA transfer count in words.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @param[in] ndt_words Number of 32-bit words.
 */
static inline void ll_extdma_set_ndt_words(EXTDMA_TypeDef *EXTDMAx,
                                           uint32_t ndt_words)
{
    MODIFY_REG(EXTDMAx->CNDTR, EXTDMA_CNDTR_NDT,
               (ndt_words << EXTDMA_CNDTR_NDT_Pos) & EXTDMA_CNDTR_NDT_Msk);
}

/**
 * @brief Configure ExtDMA control fields (burst/inc/interrupt/size).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @param[in] cfg Pointer to control configuration.
 * @note SRCSIZE and DSTSIZE are always forced to 32-bit word.
 */
static inline void ll_extdma_config_ctrl(EXTDMA_TypeDef *EXTDMAx,
                                         const ll_extdma_ctrl_config_t *cfg)
{
    MODIFY_REG(EXTDMAx->CCR,
               EXTDMA_CCR_SRCBURST | EXTDMA_CCR_DSTBURST | EXTDMA_CCR_SRCSIZE |
                   EXTDMA_CCR_DSTSIZE | EXTDMA_CCR_SRCINC | EXTDMA_CCR_DSTINC |
                   EXTDMA_CCR_TCIE | EXTDMA_CCR_HTIE | EXTDMA_CCR_TEIE,
               cfg->src_burst | cfg->dst_burst | LL_EXTDMA_SRCSIZE_WORD |
                   LL_EXTDMA_DSTSIZE_WORD | cfg->src_inc | cfg->dst_inc |
                   cfg->it_tc | cfg->it_ht | cfg->it_te);
}

/**
 * @brief Enable ExtDMA transfer.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_enable(EXTDMA_TypeDef *EXTDMAx)
{
    SET_BIT(EXTDMAx->CCR, EXTDMA_CCR_EN);
}

/**
 * @brief Disable ExtDMA transfer.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_disable(EXTDMA_TypeDef *EXTDMAx)
{
    CLEAR_BIT(EXTDMAx->CCR, EXTDMA_CCR_EN);
}

/**
 * @brief Trigger ExtDMA software reset.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @note RESET bit is auto-cleared by hardware.
 */
static inline void ll_extdma_software_reset(EXTDMA_TypeDef *EXTDMAx)
{
    SET_BIT(EXTDMAx->CCR, EXTDMA_CCR_RESET);
}

/**
 * @brief Read ExtDMA interrupt status register.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return ISR register value.
 */
static inline uint32_t ll_extdma_get_isr(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_REG(EXTDMAx->ISR);
}

/**
 * @brief Check GIF flag.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Non-zero when GIF is set.
 */
static inline uint32_t ll_extdma_is_active_flag_gif(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_BIT(EXTDMAx->ISR, EXTDMA_ISR_GIF);
}

/**
 * @brief Check TCIF flag.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Non-zero when TCIF is set.
 */
static inline uint32_t ll_extdma_is_active_flag_tcif(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_BIT(EXTDMAx->ISR, EXTDMA_ISR_TCIF);
}

/**
 * @brief Check HTIF flag.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Non-zero when HTIF is set.
 */
static inline uint32_t ll_extdma_is_active_flag_htif(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_BIT(EXTDMAx->ISR, EXTDMA_ISR_HTIF);
}

/**
 * @brief Check TEIF flag.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Non-zero when TEIF is set.
 */
static inline uint32_t ll_extdma_is_active_flag_teif(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_BIT(EXTDMAx->ISR, EXTDMA_ISR_TEIF);
}

/**
 * @brief Clear GIF flag by writing IFCR.CGIF.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_clear_flag_gif(EXTDMA_TypeDef *EXTDMAx)
{
    WRITE_REG(EXTDMAx->IFCR, EXTDMA_IFCR_CGIF);
}

/**
 * @brief Clear TCIF flag by writing IFCR.CTCIF.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_clear_flag_tcif(EXTDMA_TypeDef *EXTDMAx)
{
    WRITE_REG(EXTDMAx->IFCR, EXTDMA_IFCR_CTCIF);
}

/**
 * @brief Clear HTIF flag by writing IFCR.CHTIF.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_clear_flag_htif(EXTDMA_TypeDef *EXTDMAx)
{
    WRITE_REG(EXTDMAx->IFCR, EXTDMA_IFCR_CHTIF);
}

/**
 * @brief Clear TEIF flag by writing IFCR.CTEIF.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_clear_flag_teif(EXTDMA_TypeDef *EXTDMAx)
{
    WRITE_REG(EXTDMAx->IFCR, EXTDMA_IFCR_CTEIF);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_EXTDMA_H */
