/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_DMAC_H
#define __LL_DMAC_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_dmac.h
 * @brief Header-only low-level DMAC APIs for SF32LB52x.
 */

/** @defgroup LL_DMAC_PRIORITY LL DMAC Channel Priority */
/** @{ */
#define LL_DMAC_PRIORITY_LOW      (0x0UL << DMAC_CCR1_PL_Pos)
#define LL_DMAC_PRIORITY_MEDIUM   (0x1UL << DMAC_CCR1_PL_Pos)
#define LL_DMAC_PRIORITY_HIGH     (0x2UL << DMAC_CCR1_PL_Pos)
#define LL_DMAC_PRIORITY_VERYHIGH (0x3UL << DMAC_CCR1_PL_Pos)
/** @} */

/** @defgroup LL_DMAC_MSIZE LL DMAC Memory Data Width */
/** @{ */
#define LL_DMAC_MSIZE_8BIT  (0x0UL << DMAC_CCR1_MSIZE_Pos)
#define LL_DMAC_MSIZE_16BIT (0x1UL << DMAC_CCR1_MSIZE_Pos)
#define LL_DMAC_MSIZE_32BIT (0x2UL << DMAC_CCR1_MSIZE_Pos)
/** @} */

/** @defgroup LL_DMAC_PSIZE LL DMAC Peripheral Data Width */
/** @{ */
#define LL_DMAC_PSIZE_8BIT  (0x0UL << DMAC_CCR1_PSIZE_Pos)
#define LL_DMAC_PSIZE_16BIT (0x1UL << DMAC_CCR1_PSIZE_Pos)
#define LL_DMAC_PSIZE_32BIT (0x2UL << DMAC_CCR1_PSIZE_Pos)
/** @} */

/** @defgroup LL_DMAC_DIR LL DMAC Transfer Direction */
/** @{ */
#define LL_DMAC_DIR_PERIPH_TO_MEM (0x0UL << DMAC_CCR1_DIR_Pos)
#define LL_DMAC_DIR_MEM_TO_PERIPH (0x1UL << DMAC_CCR1_DIR_Pos)
/** @} */

/** @defgroup LL_DMAC_SWITCH LL DMAC Binary Switch */
/** @{ */
#define LL_DMAC_MEM2MEM_DISABLE (0x0UL << DMAC_CCR1_MEM2MEM_Pos)
#define LL_DMAC_MEM2MEM_ENABLE  (0x1UL << DMAC_CCR1_MEM2MEM_Pos)
#define LL_DMAC_CIRC_DISABLE    (0x0UL << DMAC_CCR1_CIRC_Pos)
#define LL_DMAC_CIRC_ENABLE     (0x1UL << DMAC_CCR1_CIRC_Pos)
#define LL_DMAC_PINC_DISABLE    (0x0UL << DMAC_CCR1_PINC_Pos)
#define LL_DMAC_PINC_ENABLE     (0x1UL << DMAC_CCR1_PINC_Pos)
#define LL_DMAC_MINC_DISABLE    (0x0UL << DMAC_CCR1_MINC_Pos)
#define LL_DMAC_MINC_ENABLE     (0x1UL << DMAC_CCR1_MINC_Pos)
#define LL_DMAC_IT_TC_DISABLE   (0x0UL << DMAC_CCR1_TCIE_Pos)
#define LL_DMAC_IT_TC_ENABLE    (0x1UL << DMAC_CCR1_TCIE_Pos)
#define LL_DMAC_IT_HT_DISABLE   (0x0UL << DMAC_CCR1_HTIE_Pos)
#define LL_DMAC_IT_HT_ENABLE    (0x1UL << DMAC_CCR1_HTIE_Pos)
#define LL_DMAC_IT_TE_DISABLE   (0x0UL << DMAC_CCR1_TEIE_Pos)
#define LL_DMAC_IT_TE_ENABLE    (0x1UL << DMAC_CCR1_TEIE_Pos)
/** @} */

/** @defgroup LL_DMAC_LIMITS LL DMAC Common Limits */
/** @{ */
#define LL_DMAC_CHANNEL_MIN      1U
#define LL_DMAC_CHANNEL_MAX      8U
#define LL_DMAC_REQUEST_ID_MASK  0x3FUL
/** @} */

/** @defgroup LL_DMAC_FLAG LL DMAC ISR Flag Template (channel 1 based) */
/** @{ */
#define LL_DMAC_FLAG_GI DMAC_ISR_GIF1
#define LL_DMAC_FLAG_TC DMAC_ISR_TCIF1
#define LL_DMAC_FLAG_HT DMAC_ISR_HTIF1
#define LL_DMAC_FLAG_TE DMAC_ISR_TEIF1
/** @} */

/** @defgroup LL_DMAC_CLEAR LL DMAC IFCR Clear Template (channel 1 based) */
/** @{ */
#define LL_DMAC_CLEAR_GI DMAC_IFCR_CGIF1
#define LL_DMAC_CLEAR_TC DMAC_IFCR_CTCIF1
#define LL_DMAC_CLEAR_HT DMAC_IFCR_CHTIF1
#define LL_DMAC_CLEAR_TE DMAC_IFCR_CTEIF1
/** @} */

/**
 * @brief DMAC channel register layout.
 */
typedef DMA_Channel_TypeDef ll_dmac_channel_t;

/**
 * @brief DMAC channel control register configuration.
 */
typedef struct
{
    uint32_t mem2mem;  /**< Memory-to-memory mode, use @ref LL_DMAC_MEM2MEM_DISABLE
                          or @ref LL_DMAC_MEM2MEM_ENABLE. */
    uint32_t priority; /**< Channel priority, use @ref LL_DMAC_PRIORITY_LOW to @ref
                          LL_DMAC_PRIORITY_VERYHIGH. */
    uint32_t msize;    /**< Memory transfer width, use @ref LL_DMAC_MSIZE_8BIT to @ref
                          LL_DMAC_MSIZE_32BIT. */
    uint32_t psize;    /**< Peripheral transfer width, use @ref LL_DMAC_PSIZE_8BIT to
                          @ref LL_DMAC_PSIZE_32BIT. */
    uint32_t minc;     /**< Memory increment control, use @ref LL_DMAC_MINC_DISABLE or
                          @ref LL_DMAC_MINC_ENABLE. */
    uint32_t pinc;     /**< Peripheral increment control, use @ref LL_DMAC_PINC_DISABLE
                          or @ref LL_DMAC_PINC_ENABLE. */
    uint32_t circ;     /**< Circular mode control, use @ref LL_DMAC_CIRC_DISABLE or @ref
                          LL_DMAC_CIRC_ENABLE. */
    uint32_t dir;      /**< Transfer direction, use @ref LL_DMAC_DIR_PERIPH_TO_MEM or
                          @ref LL_DMAC_DIR_MEM_TO_PERIPH. */
    uint32_t it_tc;    /**< Transfer-complete interrupt control, use @ref
                          LL_DMAC_IT_TC_DISABLE or @ref LL_DMAC_IT_TC_ENABLE. */
    uint32_t it_ht;    /**< Half-transfer interrupt control, use @ref
                          LL_DMAC_IT_HT_DISABLE or @ref LL_DMAC_IT_HT_ENABLE. */
    uint32_t it_te;    /**< Transfer-error interrupt control, use @ref
                          LL_DMAC_IT_TE_DISABLE or @ref LL_DMAC_IT_TE_ENABLE. */
} ll_dmac_channel_config_t;

/**
 * @brief CCR configuration mask excluding EN bit.
 */
#define LL_DMAC_CCR_CONFIG_MASK                                                 \
    (DMAC_CCR1_TCIE | DMAC_CCR1_HTIE | DMAC_CCR1_TEIE | DMAC_CCR1_DIR |        \
     DMAC_CCR1_CIRC | DMAC_CCR1_PINC | DMAC_CCR1_MINC | DMAC_CCR1_PSIZE |      \
     DMAC_CCR1_MSIZE | DMAC_CCR1_PL | DMAC_CCR1_MEM2MEM)

/**
 * @brief Compute ISR/IFCR bit shift from channel index.
 * @param[in] ch Channel index in range 1..8.
 * @return Bit shift amount for channel flags.
 */
static inline uint32_t ll_dmac_channel_flag_shift(uint32_t ch)
{
    return ((ch - 1U) * 4U);
}

/**
 * @brief Get DMAC channel register pointer.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 * @return Channel register pointer.
 */
static inline ll_dmac_channel_t *ll_dmac_get_channel(DMAC_TypeDef *DMACx,
                                                     uint32_t ch)
{
    return ((ll_dmac_channel_t *)&DMACx->CCR1) + (ch - 1U);
}

/**
 * @brief Configure channel CCR mode fields with one register update.
 * @param[in] chx DMAC channel pointer.
 * @param[in] cfg Pointer to channel configuration.
 */
static inline void ll_dmac_config_channel(ll_dmac_channel_t *chx,
                                          const ll_dmac_channel_config_t *cfg)
{
    MODIFY_REG(chx->CCR, LL_DMAC_CCR_CONFIG_MASK,
               cfg->mem2mem | cfg->priority | cfg->msize | cfg->psize |
                   cfg->minc | cfg->pinc | cfg->circ | cfg->dir | cfg->it_tc |
                   cfg->it_ht | cfg->it_te);
}

/**
 * @brief Set CPAR register value.
 * @param[in] chx DMAC channel pointer.
 * @param[in] addr Address value written to CPAR.
 */
static inline void ll_dmac_set_cpar(ll_dmac_channel_t *chx, uint32_t addr)
{
    WRITE_REG(chx->CPAR, addr);
}

/**
 * @brief Set CM0AR register value.
 * @param[in] chx DMAC channel pointer.
 * @param[in] addr Address value written to CM0AR.
 */
static inline void ll_dmac_set_cm0ar(ll_dmac_channel_t *chx, uint32_t addr)
{
    WRITE_REG(chx->CM0AR, addr);
}

/**
 * @brief Set transfer unit count in CNDTR.NDT.
 * @param[in] chx DMAC channel pointer.
 * @param[in] ndt Transfer unit count in range 0..65535.
 */
static inline void ll_dmac_set_ndt(ll_dmac_channel_t *chx, uint32_t ndt)
{
    MODIFY_REG(chx->CNDTR, DMAC_CNDTR1_NDT, (ndt & DMAC_CNDTR1_NDT));
}

/**
 * @brief Set burst size in CBSR.BS.
 * @param[in] chx DMAC channel pointer.
 * @param[in] bs Burst size field value.
 */
static inline void ll_dmac_set_burst_size(ll_dmac_channel_t *chx, uint32_t bs)
{
    MODIFY_REG(chx->CBSR, DMAC_CBSR1_BS,
               ((bs << DMAC_CBSR1_BS_Pos) & DMAC_CBSR1_BS_Msk));
}

/**
 * @brief Enable DMAC channel.
 * @param[in] chx DMAC channel pointer.
 */
static inline void ll_dmac_enable_channel(ll_dmac_channel_t *chx)
{
    SET_BIT(chx->CCR, DMAC_CCR1_EN);
}

/**
 * @brief Disable DMAC channel.
 * @param[in] chx DMAC channel pointer.
 */
static inline void ll_dmac_disable_channel(ll_dmac_channel_t *chx)
{
    CLEAR_BIT(chx->CCR, DMAC_CCR1_EN);
}

/**
 * @brief Check whether DMAC channel is enabled.
 * @param[in] chx DMAC channel pointer.
 * @return Non-zero when channel EN bit is set.
 */
static inline uint32_t ll_dmac_is_enabled_channel(ll_dmac_channel_t *chx)
{
    return READ_BIT(chx->CCR, DMAC_CCR1_EN);
}

/**
 * @brief Select request source id for a channel.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 * @param[in] request_id Request source id in range 0..63.
 */
static inline void ll_dmac_set_channel_request(DMAC_TypeDef *DMACx, uint32_t ch,
                                               uint32_t request_id)
{
    uint32_t shift;
    uint32_t req;

    req = (request_id & LL_DMAC_REQUEST_ID_MASK);
    if (ch <= 4U)
    {
        shift = (ch - 1U) * 8U;
        MODIFY_REG(DMACx->CSELR1, (DMAC_CSELR1_C1S << shift), (req << shift));
    }
    else
    {
        shift = (ch - 5U) * 8U;
        MODIFY_REG(DMACx->CSELR2, (DMAC_CSELR2_C5S << shift), (req << shift));
    }
}

/**
 * @brief Read DMAC global ISR register.
 * @param[in] DMACx DMAC instance pointer.
 * @return ISR register value.
 */
static inline uint32_t ll_dmac_get_isr(DMAC_TypeDef *DMACx)
{
    return READ_REG(DMACx->ISR);
}

/**
 * @brief Check TCIF flag of selected channel.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 * @return Non-zero when TCIF is set.
 */
static inline uint32_t ll_dmac_is_active_flag_tc(DMAC_TypeDef *DMACx,
                                                  uint32_t ch)
{
    return READ_BIT(DMACx->ISR, (LL_DMAC_FLAG_TC << ll_dmac_channel_flag_shift(ch)));
}

/**
 * @brief Check HTIF flag of selected channel.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 * @return Non-zero when HTIF is set.
 */
static inline uint32_t ll_dmac_is_active_flag_ht(DMAC_TypeDef *DMACx,
                                                  uint32_t ch)
{
    return READ_BIT(DMACx->ISR, (LL_DMAC_FLAG_HT << ll_dmac_channel_flag_shift(ch)));
}

/**
 * @brief Check TEIF flag of selected channel.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 * @return Non-zero when TEIF is set.
 */
static inline uint32_t ll_dmac_is_active_flag_te(DMAC_TypeDef *DMACx,
                                                  uint32_t ch)
{
    return READ_BIT(DMACx->ISR, (LL_DMAC_FLAG_TE << ll_dmac_channel_flag_shift(ch)));
}

/**
 * @brief Check GIF flag of selected channel.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 * @return Non-zero when GIF is set.
 */
static inline uint32_t ll_dmac_is_active_flag_gi(DMAC_TypeDef *DMACx,
                                                  uint32_t ch)
{
    return READ_BIT(DMACx->ISR, (LL_DMAC_FLAG_GI << ll_dmac_channel_flag_shift(ch)));
}

/**
 * @brief Clear TCIF flag of selected channel.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 */
static inline void ll_dmac_clear_flag_tc(DMAC_TypeDef *DMACx, uint32_t ch)
{
    WRITE_REG(DMACx->IFCR,
              (LL_DMAC_CLEAR_TC << ll_dmac_channel_flag_shift(ch)));
}

/**
 * @brief Clear HTIF flag of selected channel.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 */
static inline void ll_dmac_clear_flag_ht(DMAC_TypeDef *DMACx, uint32_t ch)
{
    WRITE_REG(DMACx->IFCR,
              (LL_DMAC_CLEAR_HT << ll_dmac_channel_flag_shift(ch)));
}

/**
 * @brief Clear TEIF flag of selected channel.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 */
static inline void ll_dmac_clear_flag_te(DMAC_TypeDef *DMACx, uint32_t ch)
{
    WRITE_REG(DMACx->IFCR,
              (LL_DMAC_CLEAR_TE << ll_dmac_channel_flag_shift(ch)));
}

/**
 * @brief Clear GIF flag of selected channel.
 * @param[in] DMACx DMAC instance pointer.
 * @param[in] ch Channel index in range 1..8.
 */
static inline void ll_dmac_clear_flag_gi(DMAC_TypeDef *DMACx, uint32_t ch)
{
    WRITE_REG(DMACx->IFCR,
              (LL_DMAC_CLEAR_GI << ll_dmac_channel_flag_shift(ch)));
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_DMAC_H */
