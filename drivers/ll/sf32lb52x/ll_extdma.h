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
 * @brief Read ExtDMA interrupt status register.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return ISR register value.
 */
static inline uint32_t ll_extdma_get_isr(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_REG(EXTDMAx->ISR);
}

/**
 * @brief Check OFIF flag (compression overflow).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Non-zero when OFIF is set.
 */
static inline uint32_t ll_extdma_is_active_flag_ofif(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_BIT(EXTDMAx->ISR, EXTDMA_ISR_OFIF);
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
 * @brief Check HTIF flag.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Non-zero when HTIF is set.
 */
static inline uint32_t ll_extdma_is_active_flag_htif(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_BIT(EXTDMAx->ISR, EXTDMA_ISR_HTIF);
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
 * @brief Check GIF flag.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Non-zero when GIF is set.
 */
static inline uint32_t ll_extdma_is_active_flag_gif(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_BIT(EXTDMAx->ISR, EXTDMA_ISR_GIF);
}

/**
 * @brief Clear OFIF flag by writing IFCR.COFIF.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_clear_flag_ofif(EXTDMA_TypeDef *EXTDMAx)
{
    WRITE_REG(EXTDMAx->IFCR, EXTDMA_IFCR_COFIF);
}

/**
 * @brief Clear TEIF flag by writing IFCR.CTEIF.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_clear_flag_teif(EXTDMA_TypeDef *EXTDMAx)
{
    WRITE_REG(EXTDMAx->IFCR, EXTDMA_IFCR_CTEIF);
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
 * @brief Clear TCIF flag by writing IFCR.CTCIF.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_clear_flag_tcif(EXTDMA_TypeDef *EXTDMAx)
{
    WRITE_REG(EXTDMAx->IFCR, EXTDMA_IFCR_CTCIF);
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
 * @brief Trigger ExtDMA software reset.
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @note RESET bit is auto-cleared by hardware.
 */
static inline void ll_extdma_software_reset(EXTDMA_TypeDef *EXTDMAx)
{
    SET_BIT(EXTDMAx->CCR, EXTDMA_CCR_RESET);
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
 * @brief Enable compression overflow interrupt (CCR.OFIE).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_enable_it_ofif(EXTDMA_TypeDef *EXTDMAx)
{
    SET_BIT(EXTDMAx->CCR, EXTDMA_CCR_OFIE);
}

/**
 * @brief Disable compression overflow interrupt (CCR.OFIE = 0).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_disable_it_ofif(EXTDMA_TypeDef *EXTDMAx)
{
    CLEAR_BIT(EXTDMAx->CCR, EXTDMA_CCR_OFIE);
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
 * @brief Set the starting byte position in the first source word (CMPRCR.SRCPOS).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @param[in] pos     Starting byte position (2 bits).
 */
static inline void ll_extdma_cmpr_set_src_pos(EXTDMA_TypeDef *EXTDMAx, uint32_t pos)
{
    MODIFY_REG(EXTDMAx->CMPRCR, EXTDMA_CMPRCR_SRCPOS,
               MAKE_REG_VAL(pos, EXTDMA_CMPRCR_SRCPOS_Msk, EXTDMA_CMPRCR_SRCPOS_Pos));
}

/**
 * @brief Set the source frame format (CMPRCR.SRCFMT).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @param[in] fmt     Source format, one of @ref EXTDMA_CMPRCR_SRCFMT_RGB565,
 *                    @ref EXTDMA_CMPRCR_SRCFMT_RGB888 or
 *                    @ref EXTDMA_CMPRCR_SRCFMT_ARGB8888.
 */
static inline void ll_extdma_cmpr_set_src_fmt(EXTDMA_TypeDef *EXTDMAx, uint32_t fmt)
{
    MODIFY_REG(EXTDMAx->CMPRCR, EXTDMA_CMPRCR_SRCFMT, fmt & EXTDMA_CMPRCR_SRCFMT_Msk);
}

/*==============================================================================
 * Compression (CMPRCR / CMPRSR / CMPRNDTR / CMPRCFG0 / CMPRCFG1 / CMPRQR / CMPRDR)
 *============================================================================*/

/**
 * @brief Enable compression (CMPRCR.CMPREN).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_cmpr_enable(EXTDMA_TypeDef *EXTDMAx)
{
    SET_BIT(EXTDMAx->CMPRCR, EXTDMA_CMPRCR_CMPREN);
}

/**
 * @brief Disable compression (CMPRCR.CMPREN = 0).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 */
static inline void ll_extdma_cmpr_disable(EXTDMA_TypeDef *EXTDMAx)
{
    CLEAR_BIT(EXTDMAx->CMPRCR, EXTDMA_CMPRCR_CMPREN);
}

/**
 * @brief Set the per-line output target size (CMPRSR.TGTSIZE).
 * @param[in] EXTDMAx   ExtDMA instance pointer.
 * @param[in] tgt_size  Output target size of each line (12 bits).
 * @note Output data size of each line is tgt_size * 3 * 2 bytes.
 */
static inline void ll_extdma_cmpr_set_tgt_size(EXTDMA_TypeDef *EXTDMAx, uint32_t tgt_size)
{
    MODIFY_REG(EXTDMAx->CMPRSR, EXTDMA_CMPRSR_TGTSIZE,
               MAKE_REG_VAL(tgt_size, EXTDMA_CMPRSR_TGTSIZE_Msk,
                            EXTDMA_CMPRSR_TGTSIZE_Pos));
}

/**
 * @brief Get the per-line output target size (CMPRSR.TGTSIZE).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Target size value.
 */
static inline uint32_t ll_extdma_cmpr_get_tgt_size(EXTDMA_TypeDef *EXTDMAx)
{
    return GET_REG_VAL2(EXTDMAx->CMPRSR, EXTDMA_CMPRSR_TGTSIZE);
}

/**
 * @brief Set the per-line input pixel count (CMPRSR.LINESIZE).
 * @param[in] EXTDMAx   ExtDMA instance pointer.
 * @param[in] line_size Column (pixel) count of each line (12 bits).
 */
static inline void ll_extdma_cmpr_set_line_size(EXTDMA_TypeDef *EXTDMAx, uint32_t line_size)
{
    MODIFY_REG(EXTDMAx->CMPRSR, EXTDMA_CMPRSR_LINESIZE,
               MAKE_REG_VAL(line_size, EXTDMA_CMPRSR_LINESIZE_Msk,
                            EXTDMA_CMPRSR_LINESIZE_Pos));
}

/**
 * @brief Get the per-line input pixel count (CMPRSR.LINESIZE).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Line size in pixels.
 */
static inline uint32_t ll_extdma_cmpr_get_line_size(EXTDMA_TypeDef *EXTDMAx)
{
    return GET_REG_VAL2(EXTDMAx->CMPRSR, EXTDMA_CMPRSR_LINESIZE);
}

/**
 * @brief Set the post-compression transfer count (CMPRNDTR.CMPRNDT).
 * @param[in] EXTDMAx  ExtDMA instance pointer.
 * @param[in] cmpr_ndt Number of data to transfer after compression (20 bits).
 * @note Must be written by software before compression. The value should be
 *       TGTSIZE * 6 * line_number / 4 in compression mode.
 */
static inline void ll_extdma_cmpr_set_ndt(EXTDMA_TypeDef *EXTDMAx, uint32_t cmpr_ndt)
{
    MODIFY_REG(EXTDMAx->CMPRNDTR, EXTDMA_CMPRNDTR_CMPRNDT,
               MAKE_REG_VAL(cmpr_ndt, EXTDMA_CMPRNDTR_CMPRNDT_Msk,
                            EXTDMA_CMPRNDTR_CMPRNDT_Pos));
}

/**
 * @brief Get the compressed transfer count (CMPRNDTR.CMPRNDT).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Number of data written after compression.
 */
static inline uint32_t ll_extdma_cmpr_get_ndt(EXTDMA_TypeDef *EXTDMAx)
{
    return GET_REG_VAL2(EXTDMAx->CMPRNDTR, EXTDMA_CMPRNDTR_CMPRNDT);
}

/**
 * @brief Set the compression configuration 0 (CMPRCFG0).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @param[in] cfg     Raw CMPRCFG0 value.
 */
static inline void ll_extdma_cmpr_set_cfg0(EXTDMA_TypeDef *EXTDMAx, uint32_t cfg)
{
    WRITE_REG(EXTDMAx->CMPRCFG0, cfg);
}

/**
 * @brief Get the compression configuration 0 (CMPRCFG0).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return CMPRCFG0 value.
 */
static inline uint32_t ll_extdma_cmpr_get_cfg0(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_REG(EXTDMAx->CMPRCFG0);
}

/**
 * @brief Set the compression configuration 1 (CMPRCFG1).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @param[in] cfg     Raw CMPRCFG1 value.
 */
static inline void ll_extdma_cmpr_set_cfg1(EXTDMA_TypeDef *EXTDMAx, uint32_t cfg)
{
    WRITE_REG(EXTDMAx->CMPRCFG1, cfg);
}

/**
 * @brief Get the compression configuration 1 (CMPRCFG1).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return CMPRCFG1 value.
 */
static inline uint32_t ll_extdma_cmpr_get_cfg1(EXTDMA_TypeDef *EXTDMAx)
{
    return READ_REG(EXTDMAx->CMPRCFG1);
}

/**
 * @brief Get the line least dummy word count in one frame (CMPRQR.DUMMY).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Dummy word count.
 */
static inline uint32_t ll_extdma_cmpr_get_dummy(EXTDMA_TypeDef *EXTDMAx)
{
    return GET_REG_VAL2(EXTDMAx->CMPRQR, EXTDMA_CMPRQR_DUMMY);
}

/**
 * @brief Get the low-quality block number (CMPRQR.LQB).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Low-quality block count.
 */
static inline uint32_t ll_extdma_cmpr_get_lqb(EXTDMA_TypeDef *EXTDMAx)
{
    return GET_REG_VAL2(EXTDMAx->CMPRQR, EXTDMA_CMPRQR_LQB);
}

/**
 * @brief Get the quality sum to low-quality block ratio (CMPRQR.LQR).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return LQR value.
 */
static inline uint32_t ll_extdma_cmpr_get_lqr(EXTDMA_TypeDef *EXTDMAx)
{
    return GET_REG_VAL2(EXTDMAx->CMPRQR, EXTDMA_CMPRQR_LQR);
}

/**
 * @brief Get the max used output buffer during compression (CMPRDR.MAXBUF).
 * @param[in] EXTDMAx ExtDMA instance pointer.
 * @return Max buffer usage (7 bits).
 */
static inline uint32_t ll_extdma_cmpr_get_maxbuf(EXTDMA_TypeDef *EXTDMAx)
{
    return GET_REG_VAL2(EXTDMAx->CMPRDR, EXTDMA_CMPRDR_MAXBUF);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_EXTDMA_H */
