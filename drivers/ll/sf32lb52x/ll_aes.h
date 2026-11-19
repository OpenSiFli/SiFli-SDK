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
#define LL_AES_IRQ_HASH_DONE     AES_ACC_IRQ_HASH_DONE_STAT
#define LL_AES_IRQ_HASH_BUS_ERR  AES_ACC_IRQ_HASH_BUS_ERR_STAT
#define LL_AES_IRQ_HASH_PAD_ERR  AES_ACC_IRQ_HASH_PAD_ERR_STAT
#define LL_AES_IRQ_DONE_RAW      AES_ACC_IRQ_DONE_RAW_STAT
#define LL_AES_IRQ_BUS_ERR_RAW   AES_ACC_IRQ_BUS_ERR_RAW_STAT
#define LL_AES_IRQ_SETUP_ERR_RAW AES_ACC_IRQ_SETUP_ERR_RAW_STAT
#define LL_AES_IRQ_HASH_DONE_RAW     AES_ACC_IRQ_HASH_DONE_RAW_STAT
#define LL_AES_IRQ_HASH_BUS_ERR_RAW  AES_ACC_IRQ_HASH_BUS_ERR_RAW_STAT
#define LL_AES_IRQ_HASH_PAD_ERR_RAW  AES_ACC_IRQ_HASH_PAD_ERR_RAW_STAT
/** @} */

/** @defgroup LL_AES_HASH_MODE LL AES HASH Mode */
/** @{ */
#define LL_AES_HASH_MODE_SHA1   (0x0UL << AES_ACC_HASH_SETTING_HASH_MODE_Pos)
#define LL_AES_HASH_MODE_SHA224 (0x1UL << AES_ACC_HASH_SETTING_HASH_MODE_Pos)
#define LL_AES_HASH_MODE_SHA256 (0x2UL << AES_ACC_HASH_SETTING_HASH_MODE_Pos)
#define LL_AES_HASH_MODE_SM3    (0x3UL << AES_ACC_HASH_SETTING_HASH_MODE_Pos)
/** @} */

/** @defgroup LL_AES_HASH_ENDIAN LL AES HASH Result Endian */
/** @{ */
#define LL_AES_HASH_ENDIAN_LITTLE (0x0UL << AES_ACC_HASH_SETTING_RESULT_ENDIAN_Pos)
#define LL_AES_HASH_ENDIAN_BIG    (0x1UL << AES_ACC_HASH_SETTING_RESULT_ENDIAN_Pos)
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
 * @brief Enable HASH auto clock gating (COMMAND.AUTO_GATE).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_auto_gate_enable(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->COMMAND, AES_ACC_COMMAND_AUTO_GATE);
}

/**
 * @brief Disable HASH auto clock gating (COMMAND.AUTO_GATE = 0).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_auto_gate_disable(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->COMMAND, AES_ACC_COMMAND_AUTO_GATE);
}

/**
 * @brief Reset HASH accelerator logic (COMMAND.HASH_RESET).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_reset(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->COMMAND, AES_ACC_COMMAND_HASH_RESET);
}

/**
 * @brief Deassert the HASH accelerator reset (COMMAND.HASH_RESET = 0).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_release_reset(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->COMMAND, AES_ACC_COMMAND_HASH_RESET);
}

/*==============================================================================
 * HASH control
 *============================================================================*/

/**
 * @brief Start HASH accelerator (COMMAND.HASH_START, write 1 to trigger).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_start(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->COMMAND, AES_ACC_COMMAND_HASH_START);
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
 * @brief Deassert the AES accelerator reset (COMMAND.AES_ACC_RESET = 0).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_release_reset(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->COMMAND, AES_ACC_COMMAND_AES_ACC_RESET);
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
 * @brief Check HASH busy flag (STATUS.HASH_BUSY).
 * @param[in] AESx AES instance pointer.
 * @return Non-zero when HASH is busy.
 */
static inline uint32_t ll_aes_is_hash_busy(AES_ACC_TypeDef *AESx)
{
    return READ_BIT(AESx->STATUS, AES_ACC_STATUS_HASH_BUSY);
}

/**
 * @brief Check flash key valid flag (STATUS.FLASH_KEY_VALID).
 * @param[in] AESx AES instance pointer.
 * @return Non-zero when the flash key is valid.
 */
static inline uint32_t ll_aes_is_flash_key_valid(AES_ACC_TypeDef *AESx)
{
    return READ_BIT(AESx->STATUS, AES_ACC_STATUS_FLASH_KEY_VALID);
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
 * @brief Enable HASH PAD_ERR interrupt (SETTING.HASH_PAD_ERR_MASK).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_enable_it_pad_err(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->SETTING, AES_ACC_SETTING_HASH_PAD_ERR_MASK);
}

/**
 * @brief Disable HASH PAD_ERR interrupt (SETTING.HASH_PAD_ERR_MASK = 0).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_disable_it_pad_err(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->SETTING, AES_ACC_SETTING_HASH_PAD_ERR_MASK);
}

/**
 * @brief Enable HASH BUS_ERR interrupt (SETTING.HASH_BUS_ERR_MASK).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_enable_it_bus_err(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->SETTING, AES_ACC_SETTING_HASH_BUS_ERR_MASK);
}

/**
 * @brief Disable HASH BUS_ERR interrupt (SETTING.HASH_BUS_ERR_MASK = 0).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_disable_it_bus_err(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->SETTING, AES_ACC_SETTING_HASH_BUS_ERR_MASK);
}

/*==============================================================================
 * HASH interrupt control (SETTING.HASH_*_MASK, 0=mask, 1=unmask)
 *============================================================================*/

/**
 * @brief Enable HASH DONE interrupt (SETTING.HASH_DONE_MASK).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_enable_it_done(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->SETTING, AES_ACC_SETTING_HASH_DONE_MASK);
}

/**
 * @brief Disable HASH DONE interrupt (SETTING.HASH_DONE_MASK = 0).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_disable_it_done(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->SETTING, AES_ACC_SETTING_HASH_DONE_MASK);
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
 * @brief Set the AES operation mode (encrypt/decrypt) in AES_SETTING.
 * @param[in] AESx AES instance pointer.
 * @param[in] enc  Non-zero for encrypt, zero for decrypt.
 */
static inline void ll_aes_set_op_mode(AES_ACC_TypeDef *AESx, uint32_t enc)
{
    MODIFY_REG(AESx->AES_SETTING, AES_ACC_AES_SETTING_AES_OP_MODE,
               enc ? AES_ACC_AES_SETTING_AES_OP_MODE : 0UL);
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
 * @brief Set HASH DMA input data byte size (HASH_DMA_DATA.SIZE).
 * @param[in] AESx AES instance pointer.
 * @param[in] size Input data byte size.
 */
static inline void ll_aes_hash_set_dma_data_size(AES_ACC_TypeDef *AESx, uint32_t size)
{
    WRITE_REG(AESx->HASH_DMA_DATA, size);
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
 * @brief Load HASH length from HASH_LEN_L/H registers (HASH_SETTING.HASH_LEN_LOAD, w1t).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_len_load(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->HASH_SETTING, AES_ACC_HASH_SETTING_HASH_LEN_LOAD);
}

/**
 * @brief Load HASH IV from HASH_IV_H* registers (HASH_SETTING.HASH_IV_LOAD, w1t).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_iv_load(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->HASH_SETTING, AES_ACC_HASH_SETTING_HASH_IV_LOAD);
}

/**
 * @brief Set HASH result endian (HASH_SETTING.RESULT_ENDIAN).
 * @param[in] AESx    AES instance pointer.
 * @param[in] endian  Endian, use @ref LL_AES_HASH_ENDIAN_LITTLE or @ref LL_AES_HASH_ENDIAN_BIG.
 */
static inline void ll_aes_hash_set_result_endian(AES_ACC_TypeDef *AESx, uint32_t endian)
{
    MODIFY_REG(AESx->HASH_SETTING, AES_ACC_HASH_SETTING_RESULT_ENDIAN,
               MAKE_REG_VAL(endian, AES_ACC_HASH_SETTING_RESULT_ENDIAN_Msk,
                            AES_ACC_HASH_SETTING_RESULT_ENDIAN_Pos));
}

/**
 * @brief Select HASH default IV source (HASH_SETTING.DFT_IV_SEL).
 * @param[in] AESx AES instance pointer.
 * @param[in] sel  IV source: 0 = default per hash mode, 1 = HASH_IV_H* registers.
 */
static inline void ll_aes_hash_set_dft_iv_sel(AES_ACC_TypeDef *AESx, uint32_t sel)
{
    MODIFY_REG(AESx->HASH_SETTING, AES_ACC_HASH_SETTING_DFT_IV_SEL,
               MAKE_REG_VAL(sel, AES_ACC_HASH_SETTING_DFT_IV_SEL_Msk,
                            AES_ACC_HASH_SETTING_DFT_IV_SEL_Pos));
}

/**
 * @brief Enable HASH byte swap (HASH_SETTING.BYTE_SWAP).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_byte_swap_enable(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->HASH_SETTING, AES_ACC_HASH_SETTING_BYTE_SWAP);
}

/**
 * @brief Disable HASH byte swap (HASH_SETTING.BYTE_SWAP = 0).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_byte_swap_disable(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->HASH_SETTING, AES_ACC_HASH_SETTING_BYTE_SWAP);
}

/**
 * @brief Enable HASH padding (HASH_SETTING.DO_PADDING).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_padding_enable(AES_ACC_TypeDef *AESx)
{
    SET_BIT(AESx->HASH_SETTING, AES_ACC_HASH_SETTING_DO_PADDING);
}

/**
 * @brief Disable HASH padding (HASH_SETTING.DO_PADDING = 0).
 * @param[in] AESx AES instance pointer.
 */
static inline void ll_aes_hash_padding_disable(AES_ACC_TypeDef *AESx)
{
    CLEAR_BIT(AESx->HASH_SETTING, AES_ACC_HASH_SETTING_DO_PADDING);
}

/*==============================================================================
 * HASH configuration
 *============================================================================*/

/**
 * @brief Set HASH mode (HASH_SETTING.HASH_MODE).
 * @param[in] AESx  AES instance pointer.
 * @param[in] mode  Hash mode, use @ref LL_AES_HASH_MODE_SHA1 to @ref LL_AES_HASH_MODE_SM3.
 */
static inline void ll_aes_hash_set_mode(AES_ACC_TypeDef *AESx, uint32_t mode)
{
    MODIFY_REG(AESx->HASH_SETTING, AES_ACC_HASH_SETTING_HASH_MODE,
               MAKE_REG_VAL(mode, AES_ACC_HASH_SETTING_HASH_MODE_Msk,
                            AES_ACC_HASH_SETTING_HASH_MODE_Pos));
}

/*==============================================================================
 * HASH data path
 *============================================================================*/

/**
 * @brief Set HASH DMA input address (HASH_DMA_IN.ADDR).
 * @param[in] AESx    AES instance pointer.
 * @param[in] in_addr Input address.
 */
static inline void ll_aes_hash_set_dma_in(AES_ACC_TypeDef *AESx, uint32_t in_addr)
{
    WRITE_REG(AESx->HASH_DMA_IN, in_addr);
}

/**
 * @brief Set HASH IV words to HASH_IV_H0..HASH_IV_H7.
 * @param[in] AESx AES instance pointer.
 * @param[in] iv_words Pointer to 8-word IV buffer.
 */
static inline void ll_aes_hash_set_iv_words(AES_ACC_TypeDef *AESx,
        const uint32_t iv_words[8])
{
    volatile uint32_t *iv_reg = &AESx->HASH_IV_H0;
    uint32_t i;

    for (i = 0U; i < 8U; i++)
    {
        WRITE_REG(iv_reg[i], iv_words[i]);
    }
}

/**
 * @brief Get one HASH IV word (HASH_IV_Hx).
 * @param[in] AESx AES instance pointer.
 * @param[in] idx  IV word index (0..7).
 * @return IV word value.
 */
static inline uint32_t ll_aes_hash_get_iv_word(AES_ACC_TypeDef *AESx, uint32_t idx)
{
    volatile uint32_t *iv_reg = &AESx->HASH_IV_H0;

    return (idx < 8U) ? READ_REG(iv_reg[idx]) : 0U;
}

/**
 * @brief Read HASH result words from HASH_RESULT_H0..HASH_RESULT_H7.
 * @param[in]  AESx      AES instance pointer.
 * @param[out] res_words Pointer to 8-word result buffer.
 */
static inline void ll_aes_hash_get_result_words(AES_ACC_TypeDef *AESx,
        uint32_t res_words[8])
{
    volatile uint32_t *res_reg = &AESx->HASH_RESULT_H0;
    uint32_t i;

    for (i = 0U; i < 8U; i++)
    {
        res_words[i] = READ_REG(res_reg[i]);
    }
}

/**
 * @brief Get one HASH result word (HASH_RESULT_Hx).
 * @param[in] AESx AES instance pointer.
 * @param[in] idx  Result word index (0..7).
 * @return Result word value.
 */
static inline uint32_t ll_aes_hash_get_result_word(AES_ACC_TypeDef *AESx, uint32_t idx)
{
    volatile uint32_t *res_reg = &AESx->HASH_RESULT_H0;

    return (idx < 8U) ? READ_REG(res_reg[idx]) : 0U;
}

/**
 * @brief Set HASH input length (HASH_LEN_L/H).
 * @param[in] AESx AES instance pointer.
 * @param[in] len_l Length low 32 bits.
 * @param[in] len_h Length high 29 bits.
 */
static inline void ll_aes_hash_set_length(AES_ACC_TypeDef *AESx,
        uint32_t len_l, uint32_t len_h)
{
    WRITE_REG(AESx->HASH_LEN_L, len_l);
    MODIFY_REG(AESx->HASH_LEN_H, AES_ACC_HASH_LEN_H_DATA,
               MAKE_REG_VAL(len_h, AES_ACC_HASH_LEN_H_DATA_Msk,
                            AES_ACC_HASH_LEN_H_DATA_Pos));
}

/**
 * @brief Get HASH result length (HASH_RESULT_LEN_L/H).
 * @param[in]  AESx       AES instance pointer.
 * @param[out] len_l      Result length low 32 bits.
 * @param[out] len_h      Result length high 29 bits.
 */
static inline void ll_aes_hash_get_result_length(AES_ACC_TypeDef *AESx,
        uint32_t *len_l, uint32_t *len_h)
{
    *len_l = READ_REG(AESx->HASH_RESULT_LEN_L);
    *len_h = READ_REG(AESx->HASH_RESULT_LEN_H) & AES_ACC_HASH_RESULT_LEN_H_DATA_Msk;
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_AES_H */
