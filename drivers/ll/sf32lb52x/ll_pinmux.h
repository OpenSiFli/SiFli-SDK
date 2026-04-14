/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_PINMUX_H
#define __LL_PINMUX_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_pinmux.h
 * @brief Header-only low-level PINMUX PAD APIs for SF32LB52x.
 */

/** @defgroup LL_PINMUX_PULL LL PINMUX Pull Configuration */
/** @{ */
#define LL_PINMUX_PULL_NONE 0x00000000U
#define LL_PINMUX_PULL_DOWN HPSYS_PINMUX_PAD_SA00_PE
#define LL_PINMUX_PULL_UP (HPSYS_PINMUX_PAD_SA00_PE | HPSYS_PINMUX_PAD_SA00_PS)
/** @} */

/** @defgroup LL_PINMUX_INPUT_EN LL PINMUX Input Enable */
/** @{ */
#define LL_PINMUX_INPUT_DISABLE 0x00000000U
#define LL_PINMUX_INPUT_ENABLE HPSYS_PINMUX_PAD_SA00_IE
/** @} */

/** @defgroup LL_PINMUX_INPUT_TYPE LL PINMUX Input Type */
/** @{ */
#define LL_PINMUX_INPUT_CMOS 0x00000000U
#define LL_PINMUX_INPUT_SCHMITT HPSYS_PINMUX_PAD_SA00_IS
/** @} */

/** @defgroup LL_PINMUX_SLEW LL PINMUX Slew Rate */
/** @{ */
#define LL_PINMUX_SLEW_FAST 0x00000000U
#define LL_PINMUX_SLEW_SLOW HPSYS_PINMUX_PAD_SA00_SR
/** @} */

/** @defgroup LL_PINMUX_DRIVE LL PINMUX Drive Strength */
/** @{ */
#define LL_PINMUX_DRIVE_0 0x00000000U
#define LL_PINMUX_DRIVE_1 HPSYS_PINMUX_PAD_SA00_DS0
#define LL_PINMUX_DRIVE_2 HPSYS_PINMUX_PAD_SA00_DS1
#define LL_PINMUX_DRIVE_3                                                      \
    (HPSYS_PINMUX_PAD_SA00_DS0 | HPSYS_PINMUX_PAD_SA00_DS1)
/** @} */

/**
 * @brief PINMUX PAD configuration.
 */
typedef struct
{
    uint32_t fsel;       /**< Raw FSEL field value (0..15). */
    uint32_t pull;       /**< Pull config from @ref LL_PINMUX_PULL. */
    uint32_t input_en;   /**< Input enable from @ref LL_PINMUX_INPUT_EN. */
    uint32_t input_type; /**< Input type from @ref LL_PINMUX_INPUT_TYPE. */
    uint32_t slew;       /**< Slew rate from @ref LL_PINMUX_SLEW. */
    uint32_t drive;      /**< Drive strength from @ref LL_PINMUX_DRIVE. */
} ll_pinmux_pad_config_t;

/**
 * @brief Check whether PAD index is valid for HPSYS PINMUX.
 * @param[in] pad_index PAD index.
 * @return Non-zero when valid.
 */
static inline uint32_t ll_pinmux_is_valid_pad_index(uint32_t pad_index)
{
    return (pad_index < (uint32_t)HPSYS_PAD_NUM);
}

/**
 * @brief Get PAD register pointer by PAD index.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 * @return PAD register pointer.
 */
static inline __IO uint32_t *
ll_pinmux_get_pad_reg(HPSYS_PINMUX_TypeDef *PINMUXx, uint32_t pad_index)
{
    return &((&PINMUXx->PAD_SA00)[pad_index]);
}

/**
 * @brief Configure one PAD atomically (FSEL/IE/PE/PS/IS/SR/DS).
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 * @param[in] cfg Pointer to PAD configuration.
 * @note POE is reserved by UM and is intentionally not exposed/modified.
 */
static inline void ll_pinmux_config_pad(HPSYS_PINMUX_TypeDef *PINMUXx,
                                        uint32_t pad_index,
                                        const ll_pinmux_pad_config_t *cfg)
{
    __IO uint32_t *pad_reg;
    uint32_t mask;
    uint32_t value;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    mask = HPSYS_PINMUX_PAD_SA00_FSEL_Msk | HPSYS_PINMUX_PAD_SA00_PE_Msk |
           HPSYS_PINMUX_PAD_SA00_PS_Msk | HPSYS_PINMUX_PAD_SA00_IE_Msk |
           HPSYS_PINMUX_PAD_SA00_IS_Msk | HPSYS_PINMUX_PAD_SA00_SR_Msk |
           HPSYS_PINMUX_PAD_SA00_DS0_Msk | HPSYS_PINMUX_PAD_SA00_DS1_Msk;
    value = ((cfg->fsel << HPSYS_PINMUX_PAD_SA00_FSEL_Pos) &
             HPSYS_PINMUX_PAD_SA00_FSEL_Msk) |
            cfg->input_en | cfg->pull | cfg->input_type | cfg->slew |
            cfg->drive;

    MODIFY_REG(*pad_reg, mask, value);
}

/**
 * @brief Set PAD FSEL field.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 * @param[in] fsel Raw FSEL field value (0..15).
 */
static inline void ll_pinmux_set_fsel(HPSYS_PINMUX_TypeDef *PINMUXx,
                                      uint32_t pad_index, uint32_t fsel)
{
    __IO uint32_t *pad_reg;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    MODIFY_REG(*pad_reg, HPSYS_PINMUX_PAD_SA00_FSEL_Msk,
               (fsel << HPSYS_PINMUX_PAD_SA00_FSEL_Pos) &
                   HPSYS_PINMUX_PAD_SA00_FSEL_Msk);
}

/**
 * @brief Get PAD FSEL field.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 * @return Raw FSEL field value.
 */
static inline uint32_t ll_pinmux_get_fsel(HPSYS_PINMUX_TypeDef *PINMUXx,
                                          uint32_t pad_index)
{
    __IO uint32_t *pad_reg;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return 0U;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    return (*pad_reg & HPSYS_PINMUX_PAD_SA00_FSEL_Msk) >>
           HPSYS_PINMUX_PAD_SA00_FSEL_Pos;
}

/**
 * @brief Enable PAD input buffer.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 */
static inline void ll_pinmux_enable_input(HPSYS_PINMUX_TypeDef *PINMUXx,
                                          uint32_t pad_index)
{
    __IO uint32_t *pad_reg;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    SET_BIT(*pad_reg, HPSYS_PINMUX_PAD_SA00_IE_Msk);
}

/**
 * @brief Disable PAD input buffer.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 */
static inline void ll_pinmux_disable_input(HPSYS_PINMUX_TypeDef *PINMUXx,
                                           uint32_t pad_index)
{
    __IO uint32_t *pad_reg;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    CLEAR_BIT(*pad_reg, HPSYS_PINMUX_PAD_SA00_IE_Msk);
}

/**
 * @brief Query PAD input buffer enable state.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 * @return Non-zero when input is enabled.
 */
static inline uint32_t ll_pinmux_is_input_enabled(HPSYS_PINMUX_TypeDef *PINMUXx,
                                                  uint32_t pad_index)
{
    __IO uint32_t *pad_reg;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return 0U;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    return READ_BIT(*pad_reg, HPSYS_PINMUX_PAD_SA00_IE_Msk);
}

/**
 * @brief Configure PAD pull-up/down state.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 * @param[in] pull Pull value from @ref LL_PINMUX_PULL.
 */
static inline void ll_pinmux_config_pull(HPSYS_PINMUX_TypeDef *PINMUXx,
                                         uint32_t pad_index, uint32_t pull)
{
    __IO uint32_t *pad_reg;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    MODIFY_REG(*pad_reg,
               HPSYS_PINMUX_PAD_SA00_PE_Msk | HPSYS_PINMUX_PAD_SA00_PS_Msk,
               pull);
}

/**
 * @brief Configure PAD drive strength.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 * @param[in] drive Drive value from @ref LL_PINMUX_DRIVE.
 */
static inline void ll_pinmux_config_drive(HPSYS_PINMUX_TypeDef *PINMUXx,
                                          uint32_t pad_index, uint32_t drive)
{
    __IO uint32_t *pad_reg;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    MODIFY_REG(*pad_reg,
               HPSYS_PINMUX_PAD_SA00_DS0_Msk | HPSYS_PINMUX_PAD_SA00_DS1_Msk,
               drive);
}

/**
 * @brief Configure PAD input type.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 * @param[in] input_type Input type from @ref LL_PINMUX_INPUT_TYPE.
 */
static inline void ll_pinmux_set_input_type(HPSYS_PINMUX_TypeDef *PINMUXx,
                                            uint32_t pad_index,
                                            uint32_t input_type)
{
    __IO uint32_t *pad_reg;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    MODIFY_REG(*pad_reg, HPSYS_PINMUX_PAD_SA00_IS_Msk, input_type);
}

/**
 * @brief Configure PAD slew rate.
 * @param[in] PINMUXx PINMUX instance pointer.
 * @param[in] pad_index PAD index (0..HPSYS_PAD_NUM-1).
 * @param[in] slew Slew rate from @ref LL_PINMUX_SLEW.
 */
static inline void ll_pinmux_set_slew_rate(HPSYS_PINMUX_TypeDef *PINMUXx,
                                           uint32_t pad_index, uint32_t slew)
{
    __IO uint32_t *pad_reg;

    if (ll_pinmux_is_valid_pad_index(pad_index) == 0U)
    {
        return;
    }

    pad_reg = ll_pinmux_get_pad_reg(PINMUXx, pad_index);
    MODIFY_REG(*pad_reg, HPSYS_PINMUX_PAD_SA00_SR_Msk, slew);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_PINMUX_H */
