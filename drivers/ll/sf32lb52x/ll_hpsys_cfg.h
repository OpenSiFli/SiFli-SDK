/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_HPSYS_CFG_H
#define __LL_HPSYS_CFG_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_hpsys_cfg.h
 * @brief Header-only low-level HPSYS_CFG shared definitions for SF32LB52x.
 */

/** @defgroup LL_HPSYS_CFG_PINR LL HPSYS_CFG PINR Field Values */
/** @{ */
/**
 * @brief Floating/disconnected value for HPSYS_CFG *_PINR fields.
 */
#define LL_CFG_PINR_FLOAT 0x3FU
/** @} */

/**
 * @brief USARTx PINR routing configuration.
 */
typedef struct
{
    uint32_t txd_pa; /**< TXD PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t rxd_pa; /**< RXD PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t rts_pa; /**< RTS PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t cts_pa; /**< CTS PA index or @ref LL_CFG_PINR_FLOAT. */
} ll_cfg_usart_pinr_config_t;

/**
 * @brief I2Cx PINR routing configuration.
 */
typedef struct
{
    uint32_t scl_pa; /**< SCL PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t sda_pa; /**< SDA PA index or @ref LL_CFG_PINR_FLOAT. */
} ll_cfg_i2c_pinr_config_t;

/**
 * @brief GPTIMx + ETR PINR routing configuration.
 */
typedef struct
{
    uint32_t ch1_pa; /**< CH1 PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t ch2_pa; /**< CH2 PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t ch3_pa; /**< CH3 PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t ch4_pa; /**< CH4 PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t etr_pa; /**< ETR PA index or @ref LL_CFG_PINR_FLOAT. */
} ll_cfg_gptim_pinr_config_t;

/**
 * @brief LPTIMx PINR routing configuration.
 */
typedef struct
{
    uint32_t in_pa;  /**< IN PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t out_pa; /**< OUT PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t etr_pa; /**< ETR PA index or @ref LL_CFG_PINR_FLOAT. */
} ll_cfg_lptim_pinr_config_t;

/**
 * @brief ATIM1 PINR1 routing configuration.
 */
typedef struct
{
    uint32_t ch1_pa; /**< CH1 PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t ch2_pa; /**< CH2 PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t ch3_pa; /**< CH3 PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t ch4_pa; /**< CH4 PA index or @ref LL_CFG_PINR_FLOAT. */
} ll_cfg_atim1_pinr1_config_t;

/**
 * @brief ATIM1 PINR2 routing configuration.
 */
typedef struct
{
    uint32_t ch1n_pa; /**< CH1N PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t ch2n_pa; /**< CH2N PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t ch3n_pa; /**< CH3N PA index or @ref LL_CFG_PINR_FLOAT. */
} ll_cfg_atim1_pinr2_config_t;

/**
 * @brief ATIM1 PINR3 routing configuration.
 */
typedef struct
{
    uint32_t bk_pa;  /**< BK PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t bk2_pa; /**< BK2 PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t etr_pa; /**< ETR PA index or @ref LL_CFG_PINR_FLOAT. */
} ll_cfg_atim1_pinr3_config_t;

/**
 * @brief PTA PINR routing configuration.
 */
typedef struct
{
    uint32_t bt_active_pa;    /**< BT_ACTIVE PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t bt_collision_pa; /**< BT_COLLISION PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t bt_priority_pa;  /**< BT_PRIORITY PA index or @ref LL_CFG_PINR_FLOAT. */
    uint32_t wlan_active_pa;  /**< WLAN_ACTIVE PA index or @ref LL_CFG_PINR_FLOAT. */
} ll_cfg_pta_pinr_config_t;


/**
 * @brief Configure USART1 PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to USART1 PINR configuration.
 */
static inline void
ll_cfg_config_usart1_pinr(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_usart_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->txd_pa, HPSYS_CFG_USART1_PINR_TXD_PIN_Msk,
                         HPSYS_CFG_USART1_PINR_TXD_PIN_Pos) |
            MAKE_REG_VAL(cfg->rxd_pa, HPSYS_CFG_USART1_PINR_RXD_PIN_Msk,
                         HPSYS_CFG_USART1_PINR_RXD_PIN_Pos) |
            MAKE_REG_VAL(cfg->rts_pa, HPSYS_CFG_USART1_PINR_RTS_PIN_Msk,
                         HPSYS_CFG_USART1_PINR_RTS_PIN_Pos) |
            MAKE_REG_VAL(cfg->cts_pa, HPSYS_CFG_USART1_PINR_CTS_PIN_Msk,
                         HPSYS_CFG_USART1_PINR_CTS_PIN_Pos);

    MODIFY_REG(CFGx->USART1_PINR,
               HPSYS_CFG_USART1_PINR_TXD_PIN_Msk |
                   HPSYS_CFG_USART1_PINR_RXD_PIN_Msk |
                   HPSYS_CFG_USART1_PINR_RTS_PIN_Msk |
                   HPSYS_CFG_USART1_PINR_CTS_PIN_Msk,
               value);
}

/**
 * @brief Configure USART2 PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to USART2 PINR configuration.
 */
static inline void
ll_cfg_config_usart2_pinr(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_usart_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->txd_pa, HPSYS_CFG_USART2_PINR_TXD_PIN_Msk,
                         HPSYS_CFG_USART2_PINR_TXD_PIN_Pos) |
            MAKE_REG_VAL(cfg->rxd_pa, HPSYS_CFG_USART2_PINR_RXD_PIN_Msk,
                         HPSYS_CFG_USART2_PINR_RXD_PIN_Pos) |
            MAKE_REG_VAL(cfg->rts_pa, HPSYS_CFG_USART2_PINR_RTS_PIN_Msk,
                         HPSYS_CFG_USART2_PINR_RTS_PIN_Pos) |
            MAKE_REG_VAL(cfg->cts_pa, HPSYS_CFG_USART2_PINR_CTS_PIN_Msk,
                         HPSYS_CFG_USART2_PINR_CTS_PIN_Pos);

    MODIFY_REG(CFGx->USART2_PINR,
               HPSYS_CFG_USART2_PINR_TXD_PIN_Msk |
                   HPSYS_CFG_USART2_PINR_RXD_PIN_Msk |
                   HPSYS_CFG_USART2_PINR_RTS_PIN_Msk |
                   HPSYS_CFG_USART2_PINR_CTS_PIN_Msk,
               value);
}

/**
 * @brief Configure USART3 PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to USART3 PINR configuration.
 */
static inline void
ll_cfg_config_usart3_pinr(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_usart_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->txd_pa, HPSYS_CFG_USART3_PINR_TXD_PIN_Msk,
                         HPSYS_CFG_USART3_PINR_TXD_PIN_Pos) |
            MAKE_REG_VAL(cfg->rxd_pa, HPSYS_CFG_USART3_PINR_RXD_PIN_Msk,
                         HPSYS_CFG_USART3_PINR_RXD_PIN_Pos) |
            MAKE_REG_VAL(cfg->rts_pa, HPSYS_CFG_USART3_PINR_RTS_PIN_Msk,
                         HPSYS_CFG_USART3_PINR_RTS_PIN_Pos) |
            MAKE_REG_VAL(cfg->cts_pa, HPSYS_CFG_USART3_PINR_CTS_PIN_Msk,
                         HPSYS_CFG_USART3_PINR_CTS_PIN_Pos);

    MODIFY_REG(CFGx->USART3_PINR,
               HPSYS_CFG_USART3_PINR_TXD_PIN_Msk |
                   HPSYS_CFG_USART3_PINR_RXD_PIN_Msk |
                   HPSYS_CFG_USART3_PINR_RTS_PIN_Msk |
                   HPSYS_CFG_USART3_PINR_CTS_PIN_Msk,
               value);
}

/**
 * @brief Configure I2C1 PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to I2C1 PINR configuration.
 */
static inline void ll_cfg_config_i2c1_pinr(HPSYS_CFG_TypeDef *CFGx,
                                           const ll_cfg_i2c_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->scl_pa, HPSYS_CFG_I2C1_PINR_SCL_PIN_Msk,
                         HPSYS_CFG_I2C1_PINR_SCL_PIN_Pos) |
            MAKE_REG_VAL(cfg->sda_pa, HPSYS_CFG_I2C1_PINR_SDA_PIN_Msk,
                         HPSYS_CFG_I2C1_PINR_SDA_PIN_Pos);

    MODIFY_REG(CFGx->I2C1_PINR,
               HPSYS_CFG_I2C1_PINR_SCL_PIN_Msk |
                   HPSYS_CFG_I2C1_PINR_SDA_PIN_Msk,
               value);
}

/**
 * @brief Configure I2C2 PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to I2C2 PINR configuration.
 */
static inline void ll_cfg_config_i2c2_pinr(HPSYS_CFG_TypeDef *CFGx,
                                           const ll_cfg_i2c_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->scl_pa, HPSYS_CFG_I2C2_PINR_SCL_PIN_Msk,
                         HPSYS_CFG_I2C2_PINR_SCL_PIN_Pos) |
            MAKE_REG_VAL(cfg->sda_pa, HPSYS_CFG_I2C2_PINR_SDA_PIN_Msk,
                         HPSYS_CFG_I2C2_PINR_SDA_PIN_Pos);

    MODIFY_REG(CFGx->I2C2_PINR,
               HPSYS_CFG_I2C2_PINR_SCL_PIN_Msk |
                   HPSYS_CFG_I2C2_PINR_SDA_PIN_Msk,
               value);
}

/**
 * @brief Configure I2C3 PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to I2C3 PINR configuration.
 */
static inline void ll_cfg_config_i2c3_pinr(HPSYS_CFG_TypeDef *CFGx,
                                           const ll_cfg_i2c_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->scl_pa, HPSYS_CFG_I2C3_PINR_SCL_PIN_Msk,
                         HPSYS_CFG_I2C3_PINR_SCL_PIN_Pos) |
            MAKE_REG_VAL(cfg->sda_pa, HPSYS_CFG_I2C3_PINR_SDA_PIN_Msk,
                         HPSYS_CFG_I2C3_PINR_SDA_PIN_Pos);

    MODIFY_REG(CFGx->I2C3_PINR,
               HPSYS_CFG_I2C3_PINR_SCL_PIN_Msk |
                   HPSYS_CFG_I2C3_PINR_SDA_PIN_Msk,
               value);
}

/**
 * @brief Configure I2C4 PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to I2C4 PINR configuration.
 */
static inline void ll_cfg_config_i2c4_pinr(HPSYS_CFG_TypeDef *CFGx,
                                           const ll_cfg_i2c_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->scl_pa, HPSYS_CFG_I2C4_PINR_SCL_PIN_Msk,
                         HPSYS_CFG_I2C4_PINR_SCL_PIN_Pos) |
            MAKE_REG_VAL(cfg->sda_pa, HPSYS_CFG_I2C4_PINR_SDA_PIN_Msk,
                         HPSYS_CFG_I2C4_PINR_SDA_PIN_Pos);

    MODIFY_REG(CFGx->I2C4_PINR,
               HPSYS_CFG_I2C4_PINR_SCL_PIN_Msk |
                   HPSYS_CFG_I2C4_PINR_SDA_PIN_Msk,
               value);
}

/**
 * @brief Configure GPTIM1 PINR register and ETR1 field.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to GPTIM1 PINR configuration.
 */
static inline void
ll_cfg_config_gptim1_pinr(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_gptim_pinr_config_t *cfg)
{
    uint32_t gptim_value;
    uint32_t etr_value;

    gptim_value = MAKE_REG_VAL(cfg->ch1_pa, HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Msk,
                               HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Pos) |
                  MAKE_REG_VAL(cfg->ch2_pa, HPSYS_CFG_GPTIM1_PINR_CH2_PIN_Msk,
                               HPSYS_CFG_GPTIM1_PINR_CH2_PIN_Pos) |
                  MAKE_REG_VAL(cfg->ch3_pa, HPSYS_CFG_GPTIM1_PINR_CH3_PIN_Msk,
                               HPSYS_CFG_GPTIM1_PINR_CH3_PIN_Pos) |
                  MAKE_REG_VAL(cfg->ch4_pa, HPSYS_CFG_GPTIM1_PINR_CH4_PIN_Msk,
                               HPSYS_CFG_GPTIM1_PINR_CH4_PIN_Pos);
    etr_value = MAKE_REG_VAL(cfg->etr_pa, HPSYS_CFG_ETR_PINR_ETR1_PIN_Msk,
                             HPSYS_CFG_ETR_PINR_ETR1_PIN_Pos);

    MODIFY_REG(CFGx->GPTIM1_PINR,
               HPSYS_CFG_GPTIM1_PINR_CH1_PIN_Msk |
                   HPSYS_CFG_GPTIM1_PINR_CH2_PIN_Msk |
                   HPSYS_CFG_GPTIM1_PINR_CH3_PIN_Msk |
                   HPSYS_CFG_GPTIM1_PINR_CH4_PIN_Msk,
               gptim_value);
    MODIFY_REG(CFGx->ETR_PINR, HPSYS_CFG_ETR_PINR_ETR1_PIN_Msk, etr_value);
}

/**
 * @brief Configure GPTIM2 PINR register and ETR2 field.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to GPTIM2 PINR configuration.
 */
static inline void
ll_cfg_config_gptim2_pinr(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_gptim_pinr_config_t *cfg)
{
    uint32_t gptim_value;
    uint32_t etr_value;

    gptim_value = MAKE_REG_VAL(cfg->ch1_pa, HPSYS_CFG_GPTIM2_PINR_CH1_PIN_Msk,
                               HPSYS_CFG_GPTIM2_PINR_CH1_PIN_Pos) |
                  MAKE_REG_VAL(cfg->ch2_pa, HPSYS_CFG_GPTIM2_PINR_CH2_PIN_Msk,
                               HPSYS_CFG_GPTIM2_PINR_CH2_PIN_Pos) |
                  MAKE_REG_VAL(cfg->ch3_pa, HPSYS_CFG_GPTIM2_PINR_CH3_PIN_Msk,
                               HPSYS_CFG_GPTIM2_PINR_CH3_PIN_Pos) |
                  MAKE_REG_VAL(cfg->ch4_pa, HPSYS_CFG_GPTIM2_PINR_CH4_PIN_Msk,
                               HPSYS_CFG_GPTIM2_PINR_CH4_PIN_Pos);
    etr_value = MAKE_REG_VAL(cfg->etr_pa, HPSYS_CFG_ETR_PINR_ETR2_PIN_Msk,
                             HPSYS_CFG_ETR_PINR_ETR2_PIN_Pos);

    MODIFY_REG(CFGx->GPTIM2_PINR,
               HPSYS_CFG_GPTIM2_PINR_CH1_PIN_Msk |
                   HPSYS_CFG_GPTIM2_PINR_CH2_PIN_Msk |
                   HPSYS_CFG_GPTIM2_PINR_CH3_PIN_Msk |
                   HPSYS_CFG_GPTIM2_PINR_CH4_PIN_Msk,
               gptim_value);
    MODIFY_REG(CFGx->ETR_PINR, HPSYS_CFG_ETR_PINR_ETR2_PIN_Msk, etr_value);
}

/**
 * @brief Configure LPTIM1 PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to LPTIM1 PINR configuration.
 */
static inline void
ll_cfg_config_lptim1_pinr(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_lptim_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->in_pa, HPSYS_CFG_LPTIM1_PINR_IN_PIN_Msk,
                         HPSYS_CFG_LPTIM1_PINR_IN_PIN_Pos) |
            MAKE_REG_VAL(cfg->out_pa, HPSYS_CFG_LPTIM1_PINR_OUT_PIN_Msk,
                         HPSYS_CFG_LPTIM1_PINR_OUT_PIN_Pos) |
            MAKE_REG_VAL(cfg->etr_pa, HPSYS_CFG_LPTIM1_PINR_ETR_PIN_Msk,
                         HPSYS_CFG_LPTIM1_PINR_ETR_PIN_Pos);

    MODIFY_REG(CFGx->LPTIM1_PINR,
               HPSYS_CFG_LPTIM1_PINR_IN_PIN_Msk |
                   HPSYS_CFG_LPTIM1_PINR_OUT_PIN_Msk |
                   HPSYS_CFG_LPTIM1_PINR_ETR_PIN_Msk,
               value);
}

/**
 * @brief Configure LPTIM2 PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to LPTIM2 PINR configuration.
 */
static inline void
ll_cfg_config_lptim2_pinr(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_lptim_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->in_pa, HPSYS_CFG_LPTIM2_PINR_IN_PIN_Msk,
                         HPSYS_CFG_LPTIM2_PINR_IN_PIN_Pos) |
            MAKE_REG_VAL(cfg->out_pa, HPSYS_CFG_LPTIM2_PINR_OUT_PIN_Msk,
                         HPSYS_CFG_LPTIM2_PINR_OUT_PIN_Pos) |
            MAKE_REG_VAL(cfg->etr_pa, HPSYS_CFG_LPTIM2_PINR_ETR_PIN_Msk,
                         HPSYS_CFG_LPTIM2_PINR_ETR_PIN_Pos);

    MODIFY_REG(CFGx->LPTIM2_PINR,
               HPSYS_CFG_LPTIM2_PINR_IN_PIN_Msk |
                   HPSYS_CFG_LPTIM2_PINR_OUT_PIN_Msk |
                   HPSYS_CFG_LPTIM2_PINR_ETR_PIN_Msk,
               value);
}

/**
 * @brief Configure ATIM1 PINR1 register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to ATIM1 PINR1 configuration.
 */
static inline void
ll_cfg_config_atim1_pinr1(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_atim1_pinr1_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->ch1_pa, HPSYS_CFG_ATIM1_PINR1_CH1_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR1_CH1_PIN_Pos) |
            MAKE_REG_VAL(cfg->ch2_pa, HPSYS_CFG_ATIM1_PINR1_CH2_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR1_CH2_PIN_Pos) |
            MAKE_REG_VAL(cfg->ch3_pa, HPSYS_CFG_ATIM1_PINR1_CH3_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR1_CH3_PIN_Pos) |
            MAKE_REG_VAL(cfg->ch4_pa, HPSYS_CFG_ATIM1_PINR1_CH4_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR1_CH4_PIN_Pos);

    MODIFY_REG(CFGx->ATIM1_PINR1,
               HPSYS_CFG_ATIM1_PINR1_CH1_PIN_Msk |
                   HPSYS_CFG_ATIM1_PINR1_CH2_PIN_Msk |
                   HPSYS_CFG_ATIM1_PINR1_CH3_PIN_Msk |
                   HPSYS_CFG_ATIM1_PINR1_CH4_PIN_Msk,
               value);
}

/**
 * @brief Configure ATIM1 PINR2 register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to ATIM1 PINR2 configuration.
 */
static inline void
ll_cfg_config_atim1_pinr2(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_atim1_pinr2_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->ch1n_pa, HPSYS_CFG_ATIM1_PINR2_CH1N_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR2_CH1N_PIN_Pos) |
            MAKE_REG_VAL(cfg->ch2n_pa, HPSYS_CFG_ATIM1_PINR2_CH2N_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR2_CH2N_PIN_Pos) |
            MAKE_REG_VAL(cfg->ch3n_pa, HPSYS_CFG_ATIM1_PINR2_CH3N_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR2_CH3N_PIN_Pos);

    MODIFY_REG(CFGx->ATIM1_PINR2,
               HPSYS_CFG_ATIM1_PINR2_CH1N_PIN_Msk |
                   HPSYS_CFG_ATIM1_PINR2_CH2N_PIN_Msk |
                   HPSYS_CFG_ATIM1_PINR2_CH3N_PIN_Msk,
               value);
}

/**
 * @brief Configure ATIM1 PINR3 register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to ATIM1 PINR3 configuration.
 */
static inline void
ll_cfg_config_atim1_pinr3(HPSYS_CFG_TypeDef *CFGx,
                          const ll_cfg_atim1_pinr3_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->bk_pa, HPSYS_CFG_ATIM1_PINR3_BK_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR3_BK_PIN_Pos) |
            MAKE_REG_VAL(cfg->bk2_pa, HPSYS_CFG_ATIM1_PINR3_BK2_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR3_BK2_PIN_Pos) |
            MAKE_REG_VAL(cfg->etr_pa, HPSYS_CFG_ATIM1_PINR3_ETR_PIN_Msk,
                         HPSYS_CFG_ATIM1_PINR3_ETR_PIN_Pos);

    MODIFY_REG(CFGx->ATIM1_PINR3,
               HPSYS_CFG_ATIM1_PINR3_BK_PIN_Msk |
                   HPSYS_CFG_ATIM1_PINR3_BK2_PIN_Msk |
                   HPSYS_CFG_ATIM1_PINR3_ETR_PIN_Msk,
               value);
}

/**
 * @brief Configure PTA PINR register.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] cfg Pointer to PTA PINR configuration.
 */
static inline void ll_cfg_config_pta_pinr(HPSYS_CFG_TypeDef *CFGx,
                                          const ll_cfg_pta_pinr_config_t *cfg)
{
    uint32_t value;

    value = MAKE_REG_VAL(cfg->bt_active_pa, HPSYS_CFG_PTA_PINR_BT_ACTIVE_Msk,
                         HPSYS_CFG_PTA_PINR_BT_ACTIVE_Pos) |
            MAKE_REG_VAL(cfg->bt_collision_pa,
                         HPSYS_CFG_PTA_PINR_BT_COLLISION_Msk,
                         HPSYS_CFG_PTA_PINR_BT_COLLISION_Pos) |
            MAKE_REG_VAL(cfg->bt_priority_pa,
                         HPSYS_CFG_PTA_PINR_BT_PRIORITY_Msk,
                         HPSYS_CFG_PTA_PINR_BT_PRIORITY_Pos) |
            MAKE_REG_VAL(cfg->wlan_active_pa,
                         HPSYS_CFG_PTA_PINR_WLAN_ACTIVE_Msk,
                         HPSYS_CFG_PTA_PINR_WLAN_ACTIVE_Pos);

    MODIFY_REG(CFGx->PTA_PINR,
               HPSYS_CFG_PTA_PINR_BT_ACTIVE_Msk |
                   HPSYS_CFG_PTA_PINR_BT_COLLISION_Msk |
                   HPSYS_CFG_PTA_PINR_BT_PRIORITY_Msk |
                   HPSYS_CFG_PTA_PINR_WLAN_ACTIVE_Msk,
               value);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_HPSYS_CFG_H */
