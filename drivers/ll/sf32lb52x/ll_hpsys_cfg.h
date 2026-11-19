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
 * @brief Get the boot mode (BMR.BOOT_MODE).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return 0 = normal mode, 1 = download mode.
 */
static inline uint32_t ll_cfg_get_boot_mode(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_BIT(CFGx->BMR, HPSYS_CFG_BMR_BOOT_MODE) ? 1UL : 0UL;
}

/**
 * @brief Get the chip ID register (IDR).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Raw IDR value.
 */
static inline uint32_t ll_cfg_get_id(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_REG(CFGx->IDR);
}

/**
 * @brief Get the series ID (IDR.SID[31:24]).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Series ID.
 */
static inline uint32_t ll_cfg_get_series_id(HPSYS_CFG_TypeDef *CFGx)
{
    return GET_REG_VAL2(CFGx->IDR, HPSYS_CFG_IDR_SID);
}

/**
 * @brief Get the chip ID (IDR.CID[23:16]).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Chip ID.
 */
static inline uint32_t ll_cfg_get_chip_id(HPSYS_CFG_TypeDef *CFGx)
{
    return GET_REG_VAL2(CFGx->IDR, HPSYS_CFG_IDR_CID);
}

/**
 * @brief Get the package ID (IDR.PID[15:8]).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Package ID.
 */
static inline uint32_t ll_cfg_get_package_id(HPSYS_CFG_TypeDef *CFGx)
{
    return GET_REG_VAL2(CFGx->IDR, HPSYS_CFG_IDR_PID);
}

/**
 * @brief Get the ECO revision ID (IDR.REVID[7:0]).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return ECO revision ID.
 */
static inline uint32_t ll_cfg_get_revision_id(HPSYS_CFG_TypeDef *CFGx)
{
    return GET_REG_VAL2(CFGx->IDR, HPSYS_CFG_IDR_REVID);
}

/**
 * @brief Select the host of the debug SWD interface (SWCR.SWSEL).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] sel  0 = SW connected to HCPU, 1 = SW connected to LCPU.
 */
static inline void ll_cfg_set_sw_selection(HPSYS_CFG_TypeDef *CFGx, uint32_t sel)
{
    MODIFY_REG(CFGx->SWCR, HPSYS_CFG_SWCR_SWSEL,
               MAKE_REG_VAL(sel, HPSYS_CFG_SWCR_SWSEL_Msk, HPSYS_CFG_SWCR_SWSEL_Pos));
}

/**
 * @brief Get the SWD host selection (SWCR.SWSEL).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return 0 = HCPU, 1 = LCPU.
 */
static inline uint32_t ll_cfg_get_sw_selection(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_BIT(CFGx->SWCR, HPSYS_CFG_SWCR_SWSEL) ? 1UL : 0UL;
}

/**
 * @brief Set the root key encryption mode (SCR.FKEY_MODE).
 * @param[in] CFGx  HPSYS_CFG instance pointer.
 * @param[in] mode  FKEY mode: 1 = AES uses dedicated algorithm for root key.
 */
static inline void ll_cfg_set_fkey_mode(HPSYS_CFG_TypeDef *CFGx, uint32_t mode)
{
    MODIFY_REG(CFGx->SCR, HPSYS_CFG_SCR_FKEY_MODE,
               MAKE_REG_VAL(mode, HPSYS_CFG_SCR_FKEY_MODE_Msk, HPSYS_CFG_SCR_FKEY_MODE_Pos));
}

/*==============================================================================
 * System Reset Control (SYSCR)
 *============================================================================*/

/**
 * @brief Route WDT1 timeout to SoC reset (SYSCR.WDT1_REBOOT).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_wdt1_reboot_set(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->SYSCR, HPSYS_CFG_SYSCR_WDT1_REBOOT);
}

/**
 * @brief Disable WDT1 SoC reset routing (SYSCR.WDT1_REBOOT).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_wdt1_reboot_clear(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->SYSCR, HPSYS_CFG_SYSCR_WDT1_REBOOT);
}

/**
 * @brief Get the RTC time shadow register (RTC_TR).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Raw RTC time value.
 */
static inline uint32_t ll_cfg_get_rtc_time(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_REG(CFGx->RTC_TR) &
           (HPSYS_CFG_RTC_TR_PM | HPSYS_CFG_RTC_TR_HT |
            HPSYS_CFG_RTC_TR_HU | HPSYS_CFG_RTC_TR_MNT |
            HPSYS_CFG_RTC_TR_MNU | HPSYS_CFG_RTC_TR_ST |
            HPSYS_CFG_RTC_TR_SU | HPSYS_CFG_RTC_TR_SS);
}

/**
 * @brief Get the RTC date shadow register (RTC_DR).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Raw RTC date value.
 */
static inline uint32_t ll_cfg_get_rtc_date(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_REG(CFGx->RTC_DR) &
           (HPSYS_CFG_RTC_DR_ERR | HPSYS_CFG_RTC_DR_CB |
            HPSYS_CFG_RTC_DR_YT | HPSYS_CFG_RTC_DR_YU |
            HPSYS_CFG_RTC_DR_WD | HPSYS_CFG_RTC_DR_MT |
            HPSYS_CFG_RTC_DR_MU | HPSYS_CFG_RTC_DR_DT |
            HPSYS_CFG_RTC_DR_DU);
}

/**
 * @brief Set the ULP SRAM retention mode (ULPMCR.RAM_RM).
 * @param[in] CFGx   HPSYS_CFG instance pointer.
 * @param[in] rm     SRAM retention mode (2 bits).
 */
static inline void ll_cfg_set_ulp_ram_rm(HPSYS_CFG_TypeDef *CFGx, uint32_t rm)
{
    MODIFY_REG(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_RAM_RM,
               MAKE_REG_VAL(rm, HPSYS_CFG_ULPMCR_RAM_RM_Msk, HPSYS_CFG_ULPMCR_RAM_RM_Pos));
}

static inline uint32_t ll_cfg_get_sys_reserved(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_REG(CFGx->SYS_RSVD) &
           (HPSYS_CFG_SYS_RSVD_RESERVE0 | HPSYS_CFG_SYS_RSVD_RESERVE1 |
            HPSYS_CFG_SYS_RSVD_RESERVE2 | HPSYS_CFG_SYS_RSVD_RESERVE3);
}

static inline void ll_cfg_set_sys_reserved(HPSYS_CFG_TypeDef *CFGx, uint32_t value)
{
    uint32_t mask = HPSYS_CFG_SYS_RSVD_RESERVE0 |
                    HPSYS_CFG_SYS_RSVD_RESERVE1 |
                    HPSYS_CFG_SYS_RSVD_RESERVE2;

    MODIFY_REG(CFGx->SYS_RSVD, mask, value & mask);
}

/**
 * @brief Configure one per-field PINR value by register offset and field index.
 * @param[in] CFGx        HPSYS_CFG instance pointer.
 * @param[in] pinr_offset PINR register byte offset from the HPSYS_CFG base
 *                        (e.g. 0x48 for I2C1_PINR).
 * @param[in] pinr_field  PINR field index (0..3), one 8-bit slot per field.
 * @param[in] pad         PAD index (6 bits) to route into the field.
 * @note Used by the pinctrl driver for dynamic per-field PINR updates.
 */
static inline void ll_cfg_set_pinr_field(HPSYS_CFG_TypeDef *CFGx,
        uint32_t pinr_offset,
        uint32_t pinr_field,
        uint32_t pad)
{
    __IO uint32_t *pinr_reg;
    uint32_t pinr_msk;

    pinr_reg = (__IO uint32_t *)((uint8_t *)CFGx + pinr_offset);
    pinr_msk = 0xFFU << (8U * (pinr_field & 0x3U));
    MODIFY_REG(*pinr_reg, pinr_msk, (pad & 0x3FU) << (8U * (pinr_field & 0x3U)));
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

/*==============================================================================
 * ANAU Bandgap (ANAU_CR)
 *============================================================================*/

/**
 * @brief Enable the ANAU bandgap (ANAU_CR.EN_BG).
 * @note Required before TSEN/GPADC analog measurement (see reference manual
 *       8.2.3.3); shared with GPADC, recommended to keep enabled.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_anau_bandgap_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->ANAU_CR, HPSYS_CFG_ANAU_CR_EN_BG);
}

/**
 * @brief Disable the ANAU bandgap (ANAU_CR.EN_BG).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_anau_bandgap_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->ANAU_CR, HPSYS_CFG_ANAU_CR_EN_BG);
}

/*==============================================================================
 * CAU2 HPBG (High-Performance Bandgap)
 *============================================================================*/

/**
 * @brief Enable the CAU2 HPBG rail (CAU2_CR.HPBG_EN + CAU2_CR.HPBG_VDDPSW_EN).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_hpbg_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->CAU2_CR,
            HPSYS_CFG_CAU2_CR_HPBG_EN | HPSYS_CFG_CAU2_CR_HPBG_VDDPSW_EN);
}

/**
 * @brief Disable the CAU2 HPBG rail (CAU2_CR.HPBG_EN + CAU2_CR.HPBG_VDDPSW_EN).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_hpbg_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->CAU2_CR,
              HPSYS_CFG_CAU2_CR_HPBG_EN | HPSYS_CFG_CAU2_CR_HPBG_VDDPSW_EN);
}

/* Missing bit definitions (not in regs/hpsys_cfg.h) defined here as LL-local;
 * removed once the regs header is updated upstream. */
#define HPSYS_CFG_ULPMCR_FORCE_ON_Pos  (31U)
#define HPSYS_CFG_ULPMCR_FORCE_ON_Msk  (0x1UL << HPSYS_CFG_ULPMCR_FORCE_ON_Pos)
#define HPSYS_CFG_ULPMCR_FORCE_ON      HPSYS_CFG_ULPMCR_FORCE_ON_Msk

#define HPSYS_CFG_DBGR_SEL_L_Pos       (0U)
#define HPSYS_CFG_DBGR_SEL_L_Msk       (0x0FUL << HPSYS_CFG_DBGR_SEL_L_Pos)
#define HPSYS_CFG_DBGR_SEL_L           HPSYS_CFG_DBGR_SEL_L_Msk
#define HPSYS_CFG_DBGR_SEL_H_Pos       (4U)
#define HPSYS_CFG_DBGR_SEL_H_Msk       (0x0FUL << HPSYS_CFG_DBGR_SEL_H_Pos)
#define HPSYS_CFG_DBGR_SEL_H           HPSYS_CFG_DBGR_SEL_H_Msk
#define HPSYS_CFG_DBGR_BITEN_L_Pos     (8U)
#define HPSYS_CFG_DBGR_BITEN_L_Msk     (0xFFUL << HPSYS_CFG_DBGR_BITEN_L_Pos)
#define HPSYS_CFG_DBGR_BITEN_L         HPSYS_CFG_DBGR_BITEN_L_Msk
#define HPSYS_CFG_DBGR_BITEN_H_Pos     (16U)
#define HPSYS_CFG_DBGR_BITEN_H_Msk     (0xFFUL << HPSYS_CFG_DBGR_BITEN_H_Pos)
#define HPSYS_CFG_DBGR_BITEN_H         HPSYS_CFG_DBGR_BITEN_H_Msk
#define HPSYS_CFG_DBGR_CLK_SEL_Pos     (24U)
#define HPSYS_CFG_DBGR_CLK_SEL_Msk     (0x7UL << HPSYS_CFG_DBGR_CLK_SEL_Pos)
#define HPSYS_CFG_DBGR_CLK_SEL         HPSYS_CFG_DBGR_CLK_SEL_Msk
#define HPSYS_CFG_DBGR_CLK_EN_Pos      (27U)
#define HPSYS_CFG_DBGR_CLK_EN_Msk      (0x1UL << HPSYS_CFG_DBGR_CLK_EN_Pos)
#define HPSYS_CFG_DBGR_CLK_EN          HPSYS_CFG_DBGR_CLK_EN_Msk

/**
 * @brief Set the LDO voltage select shortcut (SYSCR.LDO_VSEL).
 * @param[in] CFGx  HPSYS_CFG instance pointer.
 * @param[in] vsel  LDO voltage select (1 bit).
 */
static inline void ll_cfg_set_ldo_vsel(HPSYS_CFG_TypeDef *CFGx, uint32_t vsel)
{
    MODIFY_REG(CFGx->SYSCR, HPSYS_CFG_SYSCR_LDO_VSEL,
               MAKE_REG_VAL(vsel, HPSYS_CFG_SYSCR_LDO_VSEL_Msk, HPSYS_CFG_SYSCR_LDO_VSEL_Pos));
}

/**
 * @brief Route AHB space to SDMMC1 (SYSCR.SDNAND = 1).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_sdnand_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->SYSCR, HPSYS_CFG_SYSCR_SDNAND);
}

/**
 * @brief Route AHB space to MPI2 (SYSCR.SDNAND = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_sdnand_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->SYSCR, HPSYS_CFG_SYSCR_SDNAND);
}

/**
 * @brief Force the ULP memory on (ULPMCR.FORCE_ON).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_ulp_force_on(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_FORCE_ON);
}

/**
 * @brief Release the ULP memory force-on (ULPMCR.FORCE_ON = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_ulp_force_off(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_FORCE_ON);
}

/**
 * @brief Disable the ULP ROM (ULPMCR.ROM_DIS).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_ulp_rom_disable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_ROM_DIS);
}

/**
 * @brief Enable the ULP ROM (ULPMCR.ROM_DIS = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_ulp_rom_enable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_ROM_DIS);
}

/**
 * @brief Enable the ULP ROM retention mode (ULPMCR.ROM_RME).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_ulp_rom_rm_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_ROM_RME);
}

/**
 * @brief Disable the ULP ROM retention mode (ULPMCR.ROM_RME = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_ulp_rom_rm_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_ROM_RME);
}

/**
 * @brief Set the ULP ROM retention mode (ULPMCR.ROM_RM).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] rm   ROM retention mode (2 bits).
 */
static inline void ll_cfg_set_ulp_rom_rm(HPSYS_CFG_TypeDef *CFGx, uint32_t rm)
{
    MODIFY_REG(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_ROM_RM,
               MAKE_REG_VAL(rm, HPSYS_CFG_ULPMCR_ROM_RM_Msk, HPSYS_CFG_ULPMCR_ROM_RM_Pos));
}

/**
 * @brief Set the ULP SRAM write pulse width (ULPMCR.RAM_WPULSE).
 * @param[in] CFGx   HPSYS_CFG instance pointer.
 * @param[in] wpulse Write pulse width (3 bits).
 */
static inline void ll_cfg_set_ulp_ram_wpulse(HPSYS_CFG_TypeDef *CFGx, uint32_t wpulse)
{
    MODIFY_REG(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_RAM_WPULSE,
               MAKE_REG_VAL(wpulse, HPSYS_CFG_ULPMCR_RAM_WPULSE_Msk, HPSYS_CFG_ULPMCR_RAM_WPULSE_Pos));
}

/**
 * @brief Set the ULP SRAM wake address (ULPMCR.RAM_WA).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] wa   Wake address (3 bits).
 */
static inline void ll_cfg_set_ulp_ram_wa(HPSYS_CFG_TypeDef *CFGx, uint32_t wa)
{
    MODIFY_REG(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_RAM_WA,
               MAKE_REG_VAL(wa, HPSYS_CFG_ULPMCR_RAM_WA_Msk, HPSYS_CFG_ULPMCR_RAM_WA_Pos));
}

/**
 * @brief Set the ULP SRAM retention address (ULPMCR.RAM_RA).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] ra   Retention address (3 bits).
 */
static inline void ll_cfg_set_ulp_ram_ra(HPSYS_CFG_TypeDef *CFGx, uint32_t ra)
{
    MODIFY_REG(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_RAM_RA,
               MAKE_REG_VAL(ra, HPSYS_CFG_ULPMCR_RAM_RA_Msk, HPSYS_CFG_ULPMCR_RAM_RA_Pos));
}

/**
 * @brief Enable the ULP SRAM retention mode (ULPMCR.RAM_RME).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_ulp_ram_rm_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_RAM_RME);
}

/**
 * @brief Disable the ULP SRAM retention mode (ULPMCR.RAM_RME = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_ulp_ram_rm_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->ULPMCR, HPSYS_CFG_ULPMCR_RAM_RME);
}

/**
 * @brief Swap the upper/lower byte of debug data (DBGR.SWAP).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] swap 1 = swap, 0 = no swap.
 */
static inline void ll_cfg_set_dbg_swap(HPSYS_CFG_TypeDef *CFGx, uint32_t swap)
{
    MODIFY_REG(CFGx->DBGR, HPSYS_CFG_DBGR_SWAP,
               MAKE_REG_VAL(swap, HPSYS_CFG_DBGR_SWAP_Msk, HPSYS_CFG_DBGR_SWAP_Pos));
}

/**
 * @brief Get the LP2HP NMI interrupt flag (DBGR.LP2HP_NMIF).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Non-zero when the LP2HP NMI flag is set.
 */
static inline uint32_t ll_cfg_get_dbg_lp2hp_nmi_flag(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_BIT(CFGx->DBGR, HPSYS_CFG_DBGR_LP2HP_NMIF) ? 1UL : 0UL;
}

/**
 * @brief Enable the LP2HP NMI interrupt (DBGR.LP2HP_NMIE).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_dbg_lp2hp_nmi_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->DBGR, HPSYS_CFG_DBGR_LP2HP_NMIE);
}

/**
 * @brief Disable the LP2HP NMI interrupt (DBGR.LP2HP_NMIE = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_dbg_lp2hp_nmi_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->DBGR, HPSYS_CFG_DBGR_LP2HP_NMIE);
}

/**
 * @brief Send an NMI to the LCPU (DBGR.HP2LP_NMI).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_dbg_hp2lp_nmi(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->DBGR, HPSYS_CFG_DBGR_HP2LP_NMI);
}

/**
 * @brief Clear the HP2LP NMI request (DBGR.HP2LP_NMI = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_dbg_hp2lp_nmi_clear(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->DBGR, HPSYS_CFG_DBGR_HP2LP_NMI);
}

/**
 * @brief Enable the debug clock (DBGR.CLK_EN).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_dbg_clk_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->DBGR, HPSYS_CFG_DBGR_CLK_EN);
}

/**
 * @brief Disable the debug clock (DBGR.CLK_EN = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_dbg_clk_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->DBGR, HPSYS_CFG_DBGR_CLK_EN);
}

/**
 * @brief Set the debug clock select (DBGR.CLK_SEL).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] sel  Debug clock select (3 bits).
 */
static inline void ll_cfg_set_dbg_clk_sel(HPSYS_CFG_TypeDef *CFGx, uint32_t sel)
{
    MODIFY_REG(CFGx->DBGR, HPSYS_CFG_DBGR_CLK_SEL,
               MAKE_REG_VAL(sel, HPSYS_CFG_DBGR_CLK_SEL_Msk, HPSYS_CFG_DBGR_CLK_SEL_Pos));
}

/**
 * @brief Set the debug data higher byte bit enable (DBGR.BITEN_H).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] mask Bit enable mask for the higher byte (8 bits).
 */
static inline void ll_cfg_set_dbg_bitten_h(HPSYS_CFG_TypeDef *CFGx, uint32_t mask)
{
    MODIFY_REG(CFGx->DBGR, HPSYS_CFG_DBGR_BITEN_H,
               MAKE_REG_VAL(mask, HPSYS_CFG_DBGR_BITEN_H_Msk, HPSYS_CFG_DBGR_BITEN_H_Pos));
}

/**
 * @brief Set the debug data lower byte bit enable (DBGR.BITEN_L).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] mask Bit enable mask for the lower byte (8 bits).
 */
static inline void ll_cfg_set_dbg_bitten_l(HPSYS_CFG_TypeDef *CFGx, uint32_t mask)
{
    MODIFY_REG(CFGx->DBGR, HPSYS_CFG_DBGR_BITEN_L,
               MAKE_REG_VAL(mask, HPSYS_CFG_DBGR_BITEN_L_Msk, HPSYS_CFG_DBGR_BITEN_L_Pos));
}

/**
 * @brief Set the debug data higher byte select (DBGR.SEL_H).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] sel  Higher byte select (4 bits).
 */
static inline void ll_cfg_set_dbg_sel_h(HPSYS_CFG_TypeDef *CFGx, uint32_t sel)
{
    MODIFY_REG(CFGx->DBGR, HPSYS_CFG_DBGR_SEL_H,
               MAKE_REG_VAL(sel, HPSYS_CFG_DBGR_SEL_H_Msk, HPSYS_CFG_DBGR_SEL_H_Pos));
}

/**
 * @brief Set the debug data lower byte select (DBGR.SEL_L).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] sel  Lower byte select (4 bits).
 */
static inline void ll_cfg_set_dbg_sel_l(HPSYS_CFG_TypeDef *CFGx, uint32_t sel)
{
    MODIFY_REG(CFGx->DBGR, HPSYS_CFG_DBGR_SEL_L,
               MAKE_REG_VAL(sel, HPSYS_CFG_DBGR_SEL_L_Msk, HPSYS_CFG_DBGR_SEL_L_Pos));
}

/**
 * @brief Power down the ROM during deep sleep (MDBGR.PD_ROM).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_pd_rom_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_PD_ROM);
}

/**
 * @brief Keep the ROM powered during deep sleep (MDBGR.PD_ROM = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_pd_rom_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_PD_ROM);
}

/**
 * @brief Force ROM into light sleep during active (MDBGR.LS_ROM).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_ls_rom_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_LS_ROM);
}

/**
 * @brief Release ROM light sleep force (MDBGR.LS_ROM = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_ls_rom_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_LS_ROM);
}

/**
 * @brief Force RAM2 into light sleep during active (MDBGR.LS_RAM2).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_ls_ram2_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_LS_RAM2);
}

/**
 * @brief Release RAM2 light sleep force (MDBGR.LS_RAM2 = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_ls_ram2_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_LS_RAM2);
}

/**
 * @brief Force RAM1 into light sleep during active (MDBGR.LS_RAM1).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_ls_ram1_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_LS_RAM1);
}

/**
 * @brief Release RAM1 light sleep force (MDBGR.LS_RAM1 = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_ls_ram1_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_LS_RAM1);
}

/**
 * @brief Force RAM0 into light sleep during active (MDBGR.LS_RAM0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_ls_ram0_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_LS_RAM0);
}

/**
 * @brief Release RAM0 light sleep force (MDBGR.LS_RAM0 = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_mdbg_ls_ram0_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->MDBGR, HPSYS_CFG_MDBGR_LS_RAM0);
}

/**
 * @brief Check whether the all-in-one BIST failed (BISTCR.BIST_FAIL).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Non-zero when BIST failed.
 */
static inline uint32_t ll_cfg_is_bist_failed(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_BIT(CFGx->BISTCR, HPSYS_CFG_BISTCR_BIST_FAIL) ? 1UL : 0UL;
}

/**
 * @brief Check whether the all-in-one BIST is done (BISTCR.BIST_DONE).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Non-zero when BIST is done.
 */
static inline uint32_t ll_cfg_is_bist_done(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_BIT(CFGx->BISTCR, HPSYS_CFG_BISTCR_BIST_DONE) ? 1UL : 0UL;
}

/**
 * @brief Enable the BIST mode (BISTCR.BIST_MODE).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_bist_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->BISTCR, HPSYS_CFG_BISTCR_BIST_MODE);
}

/**
 * @brief Disable the BIST mode (BISTCR.BIST_MODE = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_bist_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->BISTCR, HPSYS_CFG_BISTCR_BIST_MODE);
}

/**
 * @brief Get the BIST result (BISTR).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @return Raw BISTR value.
 */
static inline uint32_t ll_cfg_get_bist_result(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_REG(CFGx->BISTR) &
           (HPSYS_CFG_BISTR_BIST_FAIL_ROM | HPSYS_CFG_BISTR_BIST_FAIL_RAM |
            HPSYS_CFG_BISTR_BIST_FAIL_HCPU | HPSYS_CFG_BISTR_BIST_FAIL_EPIC |
            HPSYS_CFG_BISTR_BIST_FAIL_EZIP | HPSYS_CFG_BISTR_BIST_FAIL_SDMMC1 |
            HPSYS_CFG_BISTR_BIST_FAIL_USBC | HPSYS_CFG_BISTR_BIST_FAIL_AUD);
}

/**
 * @brief Set a ROM compare register by index (ROMCR0..ROMCR2).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] idx  ROM index, 0..2.
 * @param[in] val  Compare value.
 */
static inline void ll_cfg_set_rom_compare(HPSYS_CFG_TypeDef *CFGx, uint32_t idx, uint32_t val)
{
    if (idx < 3U)
    {
        (&CFGx->ROMCR0)[idx] = val;
    }
}

/**
 * @brief Get a ROM compare register by index (ROMCR0..ROMCR2).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] idx  ROM index, 0..2.
 * @return Compare value, or 0 if idx is out of range.
 */
static inline uint32_t ll_cfg_get_rom_compare(HPSYS_CFG_TypeDef *CFGx, uint32_t idx)
{
    return (idx < 3U) ? (&CFGx->ROMCR0)[idx] : 0UL;
}

/**
 * @brief Get the LCPU interrupt status flag (LPIRQ.IF0/IF1).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] idx  Interrupt index, 0 or 1.
 * @return Non-zero when the flag is set.
 */
static inline uint32_t ll_cfg_get_lpirq_flag(HPSYS_CFG_TypeDef *CFGx, uint32_t idx)
{
    return READ_BIT(CFGx->LPIRQ,
                    (idx == 0U) ? HPSYS_CFG_LPIRQ_IF0 : HPSYS_CFG_LPIRQ_IF1) ? 1UL : 0UL;
}

/**
 * @brief Clear the LCPU interrupt status flag (LPIRQ.IF0/IF1, write 1 to clear).
 * @note Both SEL0 and SEL1 (rw source-select fields) are preserved; only the
 *       target rw1c IF bit is written.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] idx  Interrupt index, 0 or 1.
 */
static inline void ll_cfg_clear_lpirq_flag(HPSYS_CFG_TypeDef *CFGx, uint32_t idx)
{
    const uint32_t sel_mask = HPSYS_CFG_LPIRQ_SEL0 | HPSYS_CFG_LPIRQ_SEL1;
    const uint32_t if_bit = (idx == 0U) ? HPSYS_CFG_LPIRQ_IF0 : HPSYS_CFG_LPIRQ_IF1;

    WRITE_REG(CFGx->LPIRQ, (READ_REG(CFGx->LPIRQ) & sel_mask) | if_bit);
}

/**
 * @brief Set the LCPU interrupt source selection (LPIRQ.SEL0/SEL1).
 * @note IF0/IF1 are rw1c (write 1 clears a pending flag). A plain read-modify-
 *       write would write a pending IF bit back as 1 and clear it, so both IF
 *       bits are forced to 0 here (a no-op for W1C) while the target rw SEL
 *       field is updated. The other SEL field is preserved.
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] idx  Interrupt index, 0 or 1.
 * @param[in] sel  Source select (6 bits).
 */
static inline void ll_cfg_set_lpirq_select(HPSYS_CFG_TypeDef *CFGx, uint32_t idx, uint32_t sel)
{
    /* Force W1C IF bits to 0; keep all other bits (both SEL fields + RSVD). */
    uint32_t reg = READ_REG(CFGx->LPIRQ) & ~(HPSYS_CFG_LPIRQ_IF0 | HPSYS_CFG_LPIRQ_IF1);

    if (idx == 0U)
    {
        reg = (reg & ~HPSYS_CFG_LPIRQ_SEL0) |
              MAKE_REG_VAL(sel, HPSYS_CFG_LPIRQ_SEL0_Msk, HPSYS_CFG_LPIRQ_SEL0_Pos);
    }
    else
    {
        reg = (reg & ~HPSYS_CFG_LPIRQ_SEL1) |
              MAKE_REG_VAL(sel, HPSYS_CFG_LPIRQ_SEL1_Msk, HPSYS_CFG_LPIRQ_SEL1_Pos);
    }
    WRITE_REG(CFGx->LPIRQ, reg);
}

/**
 * @brief Set the USB DC test point and macro select (USBCR.DC_TR/DC_TE).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] tr   DC test point select (3 bits).
 * @param[in] te   DC test macro select (1 bit).
 */
static inline void ll_cfg_usb_set_dc_test(HPSYS_CFG_TypeDef *CFGx, uint32_t tr, uint32_t te)
{
    MODIFY_REG(CFGx->USBCR, HPSYS_CFG_USBCR_DC_TR | HPSYS_CFG_USBCR_DC_TE,
               MAKE_REG_VAL(tr, HPSYS_CFG_USBCR_DC_TR_Msk, HPSYS_CFG_USBCR_DC_TR_Pos) |
               MAKE_REG_VAL(te, HPSYS_CFG_USBCR_DC_TE_Msk, HPSYS_CFG_USBCR_DC_TE_Pos));
}

/**
 * @brief Set the USB TX output impedance tuning (USBCR.TX_RTUNE).
 * @param[in] CFGx  HPSYS_CFG instance pointer.
 * @param[in] rtune TX impedance tuning (3 bits).
 */
static inline void ll_cfg_usb_set_tx_rtune(HPSYS_CFG_TypeDef *CFGx, uint32_t rtune)
{
    MODIFY_REG(CFGx->USBCR, HPSYS_CFG_USBCR_TX_RTUNE,
               MAKE_REG_VAL(rtune, HPSYS_CFG_USBCR_TX_RTUNE_Msk, HPSYS_CFG_USBCR_TX_RTUNE_Pos));
}

/**
 * @brief Enable the USB DP pull up/down (USBCR.DP_EN).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_usb_dp_en_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->USBCR, HPSYS_CFG_USBCR_DP_EN);
}

/**
 * @brief Disable the USB DP pull up/down (USBCR.DP_EN = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_usb_dp_en_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->USBCR, HPSYS_CFG_USBCR_DP_EN);
}

/**
 * @brief Enable the USB DM 15k pull-down (USBCR.DM_PD).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_usb_dm_pd_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->USBCR, HPSYS_CFG_USBCR_DM_PD);
}

/**
 * @brief Disable the USB DM 15k pull-down (USBCR.DM_PD = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_usb_dm_pd_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->USBCR, HPSYS_CFG_USBCR_DM_PD);
}

/**
 * @brief Enable the USB LDO low power mode (USBCR.LDO_LP_EN).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_usb_ldo_lp_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->USBCR, HPSYS_CFG_USBCR_LDO_LP_EN);
}

/**
 * @brief Disable the USB LDO low power mode (USBCR.LDO_LP_EN = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_usb_ldo_lp_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->USBCR, HPSYS_CFG_USBCR_LDO_LP_EN);
}

/**
 * @brief Set the USB 2.5V LDO output voltage (USBCR.LDO_VSEL).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] vsel LDO voltage select (3 bits).
 */
static inline void ll_cfg_usb_set_ldo_vsel(HPSYS_CFG_TypeDef *CFGx, uint32_t vsel)
{
    MODIFY_REG(CFGx->USBCR, HPSYS_CFG_USBCR_LDO_VSEL,
               MAKE_REG_VAL(vsel, HPSYS_CFG_USBCR_LDO_VSEL_Msk, HPSYS_CFG_USBCR_LDO_VSEL_Pos));
}

/**
 * @brief Enable the USB PHY (USBCR.USB_EN).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_usb_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->USBCR, HPSYS_CFG_USBCR_USB_EN);
}

/**
 * @brief Disable the USB PHY (USBCR.USB_EN = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_usb_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->USBCR, HPSYS_CFG_USBCR_USB_EN);
}

/**
 * @brief Set the ANAU DC test mode (ANAU_CR.DC_MR).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] mr   DC test mode (3 bits).
 */
static inline void ll_cfg_set_anau_dc_mr(HPSYS_CFG_TypeDef *CFGx, uint32_t mr)
{
    MODIFY_REG(CFGx->ANAU_CR, HPSYS_CFG_ANAU_CR_DC_MR,
               MAKE_REG_VAL(mr, HPSYS_CFG_ANAU_CR_DC_MR_Msk, HPSYS_CFG_ANAU_CR_DC_MR_Pos));
}

/**
 * @brief Set the eFuse VDD pull-down (ANAU_CR.EFUSE_VDD_PD).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] pd   Pull-down enable (1 bit).
 */
static inline void ll_cfg_set_efuse_vdd_pd(HPSYS_CFG_TypeDef *CFGx, uint32_t pd)
{
    MODIFY_REG(CFGx->ANAU_CR, HPSYS_CFG_ANAU_CR_EFUSE_VDD_PD,
               MAKE_REG_VAL(pd, HPSYS_CFG_ANAU_CR_EFUSE_VDD_PD_Msk, HPSYS_CFG_ANAU_CR_EFUSE_VDD_PD_Pos));
}

/**
 * @brief Enable the eFuse VDD (ANAU_CR.EFUSE_VDD_EN).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_efuse_vdd_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->ANAU_CR, HPSYS_CFG_ANAU_CR_EFUSE_VDD_EN);
}

/**
 * @brief Disable the eFuse VDD (ANAU_CR.EFUSE_VDD_EN = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_efuse_vdd_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->ANAU_CR, HPSYS_CFG_ANAU_CR_EFUSE_VDD_EN);
}

/**
 * @brief Enable the VBAT monitor (ANAU_CR.EN_VBAT_MON).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_vbat_mon_enable(HPSYS_CFG_TypeDef *CFGx)
{
    SET_BIT(CFGx->ANAU_CR, HPSYS_CFG_ANAU_CR_EN_VBAT_MON);
}

/**
 * @brief Disable the VBAT monitor (ANAU_CR.EN_VBAT_MON = 0).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 */
static inline void ll_cfg_vbat_mon_disable(HPSYS_CFG_TypeDef *CFGx)
{
    CLEAR_BIT(CFGx->ANAU_CR, HPSYS_CFG_ANAU_CR_EN_VBAT_MON);
}

static inline uint32_t ll_cfg_get_anau_reserved(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_REG(CFGx->ANAU_RSVD) &
           (HPSYS_CFG_ANAU_RSVD_RESERVE0 | HPSYS_CFG_ANAU_RSVD_RESERVE1 |
            HPSYS_CFG_ANAU_RSVD_RESERVE2 | HPSYS_CFG_ANAU_RSVD_RESERVE3);
}

static inline void ll_cfg_set_anau_reserved(HPSYS_CFG_TypeDef *CFGx, uint32_t value)
{
    uint32_t mask = HPSYS_CFG_ANAU_RSVD_RESERVE0 |
                    HPSYS_CFG_ANAU_RSVD_RESERVE1 |
                    HPSYS_CFG_ANAU_RSVD_RESERVE2;

    MODIFY_REG(CFGx->ANAU_RSVD, mask, value & mask);
}

/**
 * @brief Set the analog test point (ANATR).
 * @param[in] CFGx     HPSYS_CFG instance pointer.
 * @param[in] te0      ATEST0 DC test enable.
 * @param[in] ur0      ATEST0 unit resistor select (3 bits).
 * @param[in] te1      ATEST1 DC test enable.
 * @param[in] ur1      ATEST1 unit resistor select (3 bits).
 */
static inline void ll_cfg_set_analog_test(HPSYS_CFG_TypeDef *CFGx, uint32_t te0, uint32_t ur0,
        uint32_t te1, uint32_t ur1)
{
    MODIFY_REG(CFGx->ANATR,
               HPSYS_CFG_ANATR_DC_TE_ATEST0 | HPSYS_CFG_ANATR_DC_UR_ATEST0 |
               HPSYS_CFG_ANATR_DC_TE_ATEST1 | HPSYS_CFG_ANATR_DC_UR_ATEST1,
               MAKE_REG_VAL(te0, HPSYS_CFG_ANATR_DC_TE_ATEST0_Msk, HPSYS_CFG_ANATR_DC_TE_ATEST0_Pos) |
               MAKE_REG_VAL(ur0, HPSYS_CFG_ANATR_DC_UR_ATEST0_Msk, HPSYS_CFG_ANATR_DC_UR_ATEST0_Pos) |
               MAKE_REG_VAL(te1, HPSYS_CFG_ANATR_DC_TE_ATEST1_Msk, HPSYS_CFG_ANATR_DC_TE_ATEST1_Pos) |
               MAKE_REG_VAL(ur1, HPSYS_CFG_ANATR_DC_UR_ATEST1_Msk, HPSYS_CFG_ANATR_DC_UR_ATEST1_Pos));
}

/**
 * @brief Set the CAU2 DC test mode (CAU2_CR.DC_MR).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] mr   DC test mode (3 bits).
 */
static inline void ll_cfg_set_cau2_dc_mr(HPSYS_CFG_TypeDef *CFGx, uint32_t mr)
{
    MODIFY_REG(CFGx->CAU2_CR, HPSYS_CFG_CAU2_CR_DC_MR,
               MAKE_REG_VAL(mr, HPSYS_CFG_CAU2_CR_DC_MR_Msk, HPSYS_CFG_CAU2_CR_DC_MR_Pos));
}

/**
 * @brief Set the CAU2 DC bias mode (CAU2_CR.DC_BR).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] br   DC bias mode (3 bits).
 */
static inline void ll_cfg_set_cau2_dc_br(HPSYS_CFG_TypeDef *CFGx, uint32_t br)
{
    MODIFY_REG(CFGx->CAU2_CR, HPSYS_CFG_CAU2_CR_DC_BR,
               MAKE_REG_VAL(br, HPSYS_CFG_CAU2_CR_DC_BR_Msk, HPSYS_CFG_CAU2_CR_DC_BR_Pos));
}

/**
 * @brief Set the CAU2 DC trim (CAU2_CR.DC_TR).
 * @param[in] CFGx HPSYS_CFG instance pointer.
 * @param[in] tr   DC trim (3 bits).
 */
static inline void ll_cfg_set_cau2_dc_tr(HPSYS_CFG_TypeDef *CFGx, uint32_t tr)
{
    MODIFY_REG(CFGx->CAU2_CR, HPSYS_CFG_CAU2_CR_DC_TR,
               MAKE_REG_VAL(tr, HPSYS_CFG_CAU2_CR_DC_TR_Msk, HPSYS_CFG_CAU2_CR_DC_TR_Pos));
}

static inline uint32_t ll_cfg_get_cau2_reserved(HPSYS_CFG_TypeDef *CFGx)
{
    return READ_REG(CFGx->CAU2_RSVD) &
           (HPSYS_CFG_CAU2_RSVD_RESERVE0 | HPSYS_CFG_CAU2_RSVD_RESERVE1 |
            HPSYS_CFG_CAU2_RSVD_RESERVE2);
}

static inline void ll_cfg_set_cau2_reserved(HPSYS_CFG_TypeDef *CFGx, uint32_t value)
{
    MODIFY_REG(CFGx->CAU2_RSVD, HPSYS_CFG_CAU2_RSVD_RESERVE0,
               value & HPSYS_CFG_CAU2_RSVD_RESERVE0);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_HPSYS_CFG_H */
