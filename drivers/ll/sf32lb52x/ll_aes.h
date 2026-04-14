/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_AES_H
#define __LL_AES_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_aes.h
 * @brief Header-only low-level AES APIs for SF32LB52x.
 */

/** @defgroup LL_AES_MODE LL AES Block Mode */
/** @{ */
#define LL_AES_MODE_ECB (0x0UL << AES_ACC_AES_SETTING_AES_MODE_Pos)
#define LL_AES_MODE_CTR (0x1UL << AES_ACC_AES_SETTING_AES_MODE_Pos)
#define LL_AES_MODE_CBC (0x2UL << AES_ACC_AES_SETTING_AES_MODE_Pos)
/** @} */

/** @defgroup LL_AES_KEYLEN LL AES Key Length */
/** @{ */
#define LL_AES_KEYLEN_128 (0x0UL << AES_ACC_AES_SETTING_AES_LENGTH_Pos)
#define LL_AES_KEYLEN_192 (0x1UL << AES_ACC_AES_SETTING_AES_LENGTH_Pos)
#define LL_AES_KEYLEN_256 (0x2UL << AES_ACC_AES_SETTING_AES_LENGTH_Pos)
/** @} */

/** @defgroup LL_AES_KEYSEL LL AES Key Source */
/** @{ */
#define LL_AES_KEYSEL_EXT  (0x0UL << AES_ACC_AES_SETTING_KEY_SEL_Pos)
#define LL_AES_KEYSEL_ROOT (0x1UL << AES_ACC_AES_SETTING_KEY_SEL_Pos)
/** @} */

/** @defgroup LL_AES_ALGO LL AES Algorithm Standard */
/** @{ */
#define LL_AES_ALGO_AES (0x0UL << AES_ACC_AES_SETTING_ALGO_STANDARD_Pos)
#define LL_AES_ALGO_SM4 (0x1UL << AES_ACC_AES_SETTING_ALGO_STANDARD_Pos)
/** @} */

/** @defgroup LL_AES_OP LL AES Encrypt/Decrypt */
/** @{ */
#define LL_AES_OP_DECRYPT (0x0UL << AES_ACC_AES_SETTING_AES_OP_MODE_Pos)
#define LL_AES_OP_ENCRYPT (0x1UL << AES_ACC_AES_SETTING_AES_OP_MODE_Pos)
/** @} */

/** @defgroup LL_AES_BYPASS LL AES Bypass */
/** @{ */
#define LL_AES_BYPASS_DISABLE (0x0UL << AES_ACC_AES_SETTING_AES_BYPASS_Pos)
#define LL_AES_BYPASS_ENABLE  (0x1UL << AES_ACC_AES_SETTING_AES_BYPASS_Pos)
/** @} */

/** @defgroup LL_AES_IRQ LL AES IRQ Status Bits */
/** @{ */
#define LL_AES_IRQ_DONE      AES_ACC_IRQ_DONE_STAT
#define LL_AES_IRQ_BUS_ERR   AES_ACC_IRQ_BUS_ERR_STAT
#define LL_AES_IRQ_SETUP_ERR AES_ACC_IRQ_SETUP_ERR_STAT
/** @} */

/**
 * @brief AES core control register configuration.
 */
typedef struct
{
    uint32_t mode;    /**< AES mode, use @ref LL_AES_MODE_ECB to @ref LL_AES_MODE_CBC. */
    uint32_t key_len; /**< Key length, use @ref LL_AES_KEYLEN_128 to @ref LL_AES_KEYLEN_256. */
    uint32_t key_sel; /**< Key source, use @ref LL_AES_KEYSEL_EXT or @ref LL_AES_KEYSEL_ROOT. */
    uint32_t algo;    /**< Algorithm, use @ref LL_AES_ALGO_AES or @ref LL_AES_ALGO_SM4. */
    uint32_t op_mode; /**< Encrypt/decrypt, use @ref LL_AES_OP_DECRYPT or @ref LL_AES_OP_ENCRYPT. */
    uint32_t bypass;  /**< Bypass control, use @ref LL_AES_BYPASS_DISABLE or @ref LL_AES_BYPASS_ENABLE. */
} ll_aes_core_config_t;

/**
 * @brief Configure AES core fields in AES_SETTING with one MODIFY_REG transaction.
 * @param[in] AESx AES instance pointer.
 * @param[in] cfg Pointer to AES core configuration.
 */
static inline void ll_aes_config_core(AES_ACC_TypeDef *AESx,
                                      const ll_aes_core_config_t *cfg)
{
    MODIFY_REG(AESx->AES_SETTING,
               AES_ACC_AES_SETTING_AES_MODE | AES_ACC_AES_SETTING_AES_LENGTH |
                   AES_ACC_AES_SETTING_KEY_SEL |
                   AES_ACC_AES_SETTING_ALGO_STANDARD |
                   AES_ACC_AES_SETTING_AES_OP_MODE |
                   AES_ACC_AES_SETTING_AES_BYPASS,
               cfg->mode | cfg->key_len | cfg->key_sel | cfg->algo |
                   cfg->op_mode | cfg->bypass);
}

/**
 * @brief Set IV words to IV_W0..IV_W3.
 * @param[in] AESx AES instance pointer.
 * @param[in] iv_words Pointer to 4-word IV buffer.
 */
static inline void ll_aes_set_iv_words(AES_ACC_TypeDef *AESx,
                                       const uint32_t iv_words[4])
{
    WRITE_REG(AESx->IV_W0, iv_words[0]);
    WRITE_REG(AESx->IV_W1, iv_words[1]);
    WRITE_REG(AESx->IV_W2, iv_words[2]);
    WRITE_REG(AESx->IV_W3, iv_words[3]);
}

/**
 * @brief Set external key words to EXT_KEY_W0..EXT_KEY_W7.
 * @param[in] AESx AES instance pointer.
 * @param[in] key_words Pointer to key words.
 * @param[in] key_word_count Number of key words to write.
 */
static inline void ll_aes_set_key_words(AES_ACC_TypeDef *AESx,
                                        const uint32_t *key_words,
                                        uint32_t key_word_count)
{
    volatile uint32_t *key_reg = &AESx->EXT_KEY_W0;
    uint32_t i;

    for (i = 0U; i < key_word_count; i++)
    {
        WRITE_REG(key_reg[i], key_words[i]);
    }
}

/**
 * @brief Set AES DMA input address register.
 * @param[in] AESx AES instance pointer.
 * @param[in] in_addr Input address.
 */
static inline void ll_aes_set_dma_in(AES_ACC_TypeDef *AESx, uint32_t in_addr)
{
    WRITE_REG(AESx->DMA_IN, in_addr);
}

/**
 * @brief Set AES DMA output address register.
 * @param[in] AESx AES instance pointer.
 * @param[in] out_addr Output address.
 */
static inline void ll_aes_set_dma_out(AES_ACC_TypeDef *AESx, uint32_t out_addr)
{
    WRITE_REG(AESx->DMA_OUT, out_addr);
}

/**
 * @brief Set AES DMA transfer size in blocks (16 bytes per block).
 * @param[in] AESx AES instance pointer.
 * @param[in] blocks Transfer block count.
 */
static inline void ll_aes_set_dma_blocks(AES_ACC_TypeDef *AESx, uint32_t blocks)
{
    MODIFY_REG(AESx->DMA_DATA, AES_ACC_DMA_DATA_SIZE,
               ((blocks << AES_ACC_DMA_DATA_SIZE_Pos) & AES_ACC_DMA_DATA_SIZE_Msk));
}

/**
 * @brief Start AES transfer.
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_start(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->COMMAND, AES_ACC_COMMAND_START);
}

/**
 * @brief Reset AES accelerator logic.
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_reset(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->COMMAND, AES_ACC_COMMAND_AES_ACC_RESET);
}

/**
 * @brief Check BUSY flag.
 * @param[in] AESx AES instance pointer.
 * @return Non-zero when AES is busy.
 */
static inline uint32_t ll_aes_is_active_flag_busy(AES_ACC_TypeDef *AESx)
{
    return READ_BIT(AESx->STATUS, AES_ACC_STATUS_BUSY);
}

/**
 * @brief Read AES IRQ status register.
 * @param[in] AESx AES instance pointer.
 * @return IRQ register value.
 */
static inline uint32_t ll_aes_get_irq_status(AES_ACC_TypeDef *AESx)
{
    return READ_REG(AESx->IRQ);
}

/**
 * @brief Clear AES IRQ bits with rw1c semantics.
 * @param[in] AESx AES instance pointer.
 * @param[in] irq_mask IRQ bits to clear by writing 1.
 */
static inline void ll_aes_clear_irq(AES_ACC_TypeDef *AESx, uint32_t irq_mask)
{
    WRITE_REG(AESx->IRQ, irq_mask);
}

/**
 * @brief Enable DONE interrupt mask.
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_enable_it_done(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->SETTING, AES_ACC_SETTING_DONE_IRQ_MASK);
}

/**
 * @brief Disable DONE interrupt mask.
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_disable_it_done(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->SETTING, AES_ACC_SETTING_DONE_IRQ_MASK);
}

/**
 * @brief Enable BUS_ERR interrupt mask.
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_enable_it_bus_err(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->SETTING, AES_ACC_SETTING_BUS_ERR_IRQ_MASK);
}

/**
 * @brief Disable BUS_ERR interrupt mask.
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_disable_it_bus_err(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->SETTING, AES_ACC_SETTING_BUS_ERR_IRQ_MASK);
}

/**
 * @brief Enable SETUP_ERR interrupt mask.
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_enable_it_setup_err(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->SETTING, AES_ACC_SETTING_SETUP_ERR_IRQ_MASK);
}

/**
 * @brief Disable SETUP_ERR interrupt mask.
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_disable_it_setup_err(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->SETTING, AES_ACC_SETTING_SETUP_ERR_IRQ_MASK);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_AES_H */
