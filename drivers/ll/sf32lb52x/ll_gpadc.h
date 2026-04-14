/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_GPADC_H
#define __LL_GPADC_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_gpadc.h
 * @brief Header-only low-level GPADC APIs for SF32LB52x.
 */

/** @defgroup LL_GPADC_OP_MODE LL GPADC Operation Mode */
/** @{ */
#define LL_GPADC_OP_MODE_SINGLE 0x00000000U
#define LL_GPADC_OP_MODE_CONTINUOUS GPADC_ADC_CTRL_REG_ADC_OP_MODE
/** @} */

/** @defgroup LL_GPADC_INPUT_MODE LL GPADC Input Mode */
/** @{ */
#define LL_GPADC_INPUT_MODE_DIFF 0x00000000U
#define LL_GPADC_INPUT_MODE_SINGLE_ENDED GPADC_ADC_CFG_REG1_ANAU_GPADC_SE
/** @} */

/** @defgroup LL_GPADC_DMA_DATA LL GPADC DMA Data Type */
/** @{ */
#define LL_GPADC_DMA_DATA_COMBINED 0x00000000U
#define LL_GPADC_DMA_DATA_RAW GPADC_ADC_CTRL_REG_DMA_DATA_SEL
/** @} */

/** @defgroup LL_GPADC_TIMER_TRIG_TYPE LL GPADC Timer Trigger Type */
/** @{ */
#define LL_GPADC_TIMER_TRIG_PULSE 0x00000000U
#define LL_GPADC_TIMER_TRIG_LEVEL GPADC_ADC_CTRL_REG_TIMER_TRIG_TYP
/** @} */

/** @defgroup LL_GPADC_TRIGSRC LL GPADC Trigger Source */
/** @{ */
#define LL_GPADC_TRIGSRC_GPTIM1_TRGO                                           \
    (0x0UL << GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL_Pos)
#define LL_GPADC_TRIGSRC_GPTIM2_TRGO                                           \
    (0x1UL << GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL_Pos)
#define LL_GPADC_TRIGSRC_APTIM1_TRGO                                           \
    (0x2UL << GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL_Pos)
#define LL_GPADC_TRIGSRC_BTIM1_TRGO                                            \
    (0x3UL << GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL_Pos)
#define LL_GPADC_TRIGSRC_BTIM2_TRGO                                            \
    (0x4UL << GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL_Pos)
#define LL_GPADC_TRIGSRC_GPTIM1_CH0                                            \
    (0x5UL << GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL_Pos)
#define LL_GPADC_TRIGSRC_GPTIM1_CH1                                            \
    (0x6UL << GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL_Pos)
#define LL_GPADC_TRIGSRC_GPTIM1_CH2                                            \
    (0x7UL << GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL_Pos)
/** @} */

/**
 * @brief GPADC analog-front-end configuration.
 */
typedef struct
{
    uint32_t cmref_fast_en; /**< CMREF fast enable, 0 or
                               GPADC_ADC_CFG_REG1_ANAU_GPADC_CMREF_FAST_EN. */
    uint32_t p_int_en;      /**< P-side internal path enable, 0 or
                               GPADC_ADC_CFG_REG1_ANAU_GPADC_P_INT_EN. */
    uint32_t cl_dly; /**< CL delay field value for ANAU_GPADC_CL_DLY[5:3]. */
    uint32_t en_v18; /**< 1.8V analog enable, 0 or
                        GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_V18. */
    uint32_t input_mode; /**< Input mode, use @ref LL_GPADC_INPUT_MODE_DIFF or
                            @ref LL_GPADC_INPUT_MODE_SINGLE_ENDED. */
    uint32_t mute;       /**< Analog mute control, 0 or
                            GPADC_ADC_CFG_REG1_ANAU_GPADC_MUTE. */
    uint32_t force_p_channel; /**< Force positive channel value for
                                 ANAU_GPADC_SEL_PCH[14:12]. */
    uint32_t force_n_channel; /**< Force negative channel value for
                                 ANAU_GPADC_SEL_NCH[11:9]. */
    uint32_t ldovref_sel;     /**< LDO VREF selection field value for
                                 ANAU_GPADC_LDOVREF_SEL[18:15]. */
    uint32_t vsp;             /**< VSP field value for ANAU_GPADC_VSP[21:20]. */
    uint32_t cmpcl; /**< CMPCL field value for ANAU_GPADC_CMPCL[24:22]. */
    uint32_t cmm;   /**< CMM field value for ANAU_GPADC_CMM[29:25]. */
} ll_gpadc_analog_config_t;

/**
 * @brief GPADC conversion clock configuration.
 */
typedef struct
{
    uint32_t data_samp_dly; /**< DATA_SAMP_DLY field value for
                               ADC_CTRL_REG[20:17]. */
    uint32_t
        conv_width; /**< CONV_WIDTH field value for ADC_CTRL_REG2[31:24]. */
    uint32_t samp_width; /**< SAMP_WIDTH field value for ADC_CTRL_REG2[23:0]. */
} ll_gpadc_clock_config_t;

/**
 * @brief GPADC trigger path configuration.
 */
typedef struct
{
    uint32_t timer_enable; /**< Non-zero to enable timer trigger path
                              (ADC_CTRL_REG.TIMER_TRIG_EN). */
    uint32_t timer_source; /**< Timer trigger source, use @ref
                              LL_GPADC_TRIGSRC_GPTIM1_TRGO to @ref
                              LL_GPADC_TRIGSRC_GPTIM1_CH2. */
    uint32_t
        timer_type; /**< Timer trigger type, use @ref LL_GPADC_TIMER_TRIG_PULSE
                       or @ref LL_GPADC_TIMER_TRIG_LEVEL. */
} ll_gpadc_trigger_config_t;

/**
 * @brief GPADC operation mode configuration.
 */
typedef struct
{
    uint32_t op_mode;   /**< Operation mode, use @ref LL_GPADC_OP_MODE_SINGLE or
                           @ref LL_GPADC_OP_MODE_CONTINUOUS. */
    uint32_t init_time; /**< INIT_TIME field value for ADC_CTRL_REG[6:3]. */
} ll_gpadc_mode_config_t;

/**
 * @brief GPADC slot configuration.
 */
typedef struct
{
    uint32_t slot_enable; /**< Non-zero to enable slot conversion. */
    uint32_t
        p_channel; /**< Positive channel field value for PCHNL_SEL[10:8]. */
    uint32_t
        n_channel; /**< Negative channel field value for NCHNL_SEL[13:11]. */
} ll_gpadc_slot_config_t;

/**
 * @brief Configure analog-front-end fields in ADC_CFG_REG1.
 * @param[in] GPADCx GPADC instance pointer.
 * @param[in] cfg Pointer to analog configuration.
 */
static inline void ll_gpadc_config_analog(GPADC_TypeDef *GPADCx,
                                          const ll_gpadc_analog_config_t *cfg)
{
    MODIFY_REG(GPADCx->ADC_CFG_REG1,
               GPADC_ADC_CFG_REG1_ANAU_GPADC_CMM |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_CMPCL |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_VSP |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOVREF_SEL |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_SEL_PCH |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_SEL_NCH |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_MUTE |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_SE |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_V18 |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_CL_DLY |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_P_INT_EN |
                   GPADC_ADC_CFG_REG1_ANAU_GPADC_CMREF_FAST_EN,
               ((cfg->cmm << GPADC_ADC_CFG_REG1_ANAU_GPADC_CMM_Pos) &
                GPADC_ADC_CFG_REG1_ANAU_GPADC_CMM) |
                   ((cfg->cmpcl << GPADC_ADC_CFG_REG1_ANAU_GPADC_CMPCL_Pos) &
                    GPADC_ADC_CFG_REG1_ANAU_GPADC_CMPCL) |
                   ((cfg->vsp << GPADC_ADC_CFG_REG1_ANAU_GPADC_VSP_Pos) &
                    GPADC_ADC_CFG_REG1_ANAU_GPADC_VSP) |
                   ((cfg->ldovref_sel
                     << GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOVREF_SEL_Pos) &
                    GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOVREF_SEL) |
                   ((cfg->force_p_channel
                     << GPADC_ADC_CFG_REG1_ANAU_GPADC_SEL_PCH_Pos) &
                    GPADC_ADC_CFG_REG1_ANAU_GPADC_SEL_PCH) |
                   ((cfg->force_n_channel
                     << GPADC_ADC_CFG_REG1_ANAU_GPADC_SEL_NCH_Pos) &
                    GPADC_ADC_CFG_REG1_ANAU_GPADC_SEL_NCH) |
                   cfg->mute | cfg->input_mode | cfg->en_v18 |
                   ((cfg->cl_dly << GPADC_ADC_CFG_REG1_ANAU_GPADC_CL_DLY_Pos) &
                    GPADC_ADC_CFG_REG1_ANAU_GPADC_CL_DLY) |
                   cfg->p_int_en | cfg->cmref_fast_en);
}

/**
 * @brief Configure conversion clock related fields.
 * @param[in] GPADCx GPADC instance pointer.
 * @param[in] cfg Pointer to clock configuration.
 */
static inline void ll_gpadc_config_clock(GPADC_TypeDef *GPADCx,
                                         const ll_gpadc_clock_config_t *cfg)
{
    MODIFY_REG(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_DATA_SAMP_DLY,
               ((cfg->data_samp_dly << GPADC_ADC_CTRL_REG_DATA_SAMP_DLY_Pos) &
                GPADC_ADC_CTRL_REG_DATA_SAMP_DLY));

    MODIFY_REG(GPADCx->ADC_CTRL_REG2,
               GPADC_ADC_CTRL_REG2_CONV_WIDTH | GPADC_ADC_CTRL_REG2_SAMP_WIDTH,
               ((cfg->conv_width << GPADC_ADC_CTRL_REG2_CONV_WIDTH_Pos) &
                GPADC_ADC_CTRL_REG2_CONV_WIDTH) |
                   ((cfg->samp_width << GPADC_ADC_CTRL_REG2_SAMP_WIDTH_Pos) &
                    GPADC_ADC_CTRL_REG2_SAMP_WIDTH));
}

/**
 * @brief Configure trigger path fields.
 * @param[in] GPADCx GPADC instance pointer.
 * @param[in] cfg Pointer to trigger configuration.
 */
static inline void ll_gpadc_config_trigger(GPADC_TypeDef *GPADCx,
                                           const ll_gpadc_trigger_config_t *cfg)
{
    uint32_t val;

    val = ((cfg->timer_enable != 0U) ? GPADC_ADC_CTRL_REG_TIMER_TRIG_EN : 0U) |
          cfg->timer_source | cfg->timer_type;

    MODIFY_REG(GPADCx->ADC_CTRL_REG,
               GPADC_ADC_CTRL_REG_TIMER_TRIG_EN |
                   GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL |
                   GPADC_ADC_CTRL_REG_TIMER_TRIG_TYP,
               val);
}

/**
 * @brief Configure operation mode fields.
 * @param[in] GPADCx GPADC instance pointer.
 * @param[in] cfg Pointer to mode configuration.
 */
static inline void ll_gpadc_config_mode(GPADC_TypeDef *GPADCx,
                                        const ll_gpadc_mode_config_t *cfg)
{
    MODIFY_REG(GPADCx->ADC_CTRL_REG,
               GPADC_ADC_CTRL_REG_ADC_OP_MODE | GPADC_ADC_CTRL_REG_INIT_TIME,
               cfg->op_mode |
                   ((cfg->init_time << GPADC_ADC_CTRL_REG_INIT_TIME_Pos) &
                    GPADC_ADC_CTRL_REG_INIT_TIME));
}

/**
 * @brief Configure one slot register.
 * @param[in] GPADCx GPADC instance pointer.
 * @param[in] slot_index Slot index, valid range 0..7.
 * @param[in] cfg Pointer to slot configuration.
 */
static inline void ll_gpadc_config_slot(GPADC_TypeDef *GPADCx,
                                        uint32_t slot_index,
                                        const ll_gpadc_slot_config_t *cfg)
{
    volatile uint32_t *slot_reg;

    slot_reg = (&GPADCx->ADC_SLOT0_REG) + slot_index;
    MODIFY_REG(*slot_reg, 0x00003F01UL,
               (((cfg->n_channel << 11U) & 0x00003800UL) |
                ((cfg->p_channel << 8U) & 0x00000700UL) |
                ((cfg->slot_enable != 0U) ? 0x00000001UL : 0U)));
}

/**
 * @brief Enable GPADC core.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_enable_core(GPADC_TypeDef *GPADCx)
{
    SET_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_FRC_EN_ADC);
}

/**
 * @brief Disable GPADC core.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_disable_core(GPADC_TypeDef *GPADCx)
{
    CLEAR_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_FRC_EN_ADC);
}

/**
 * @brief Check whether GPADC core is enabled.
 * @param[in] GPADCx GPADC instance pointer.
 * @return Non-zero when core is enabled.
 */
static inline uint32_t ll_gpadc_is_enabled_core(GPADC_TypeDef *GPADCx)
{
    return READ_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_FRC_EN_ADC);
}

/**
 * @brief Enable GPADC LDO reference.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_enable_ldoref(GPADC_TypeDef *GPADCx)
{
    SET_BIT(GPADCx->ADC_CFG_REG1, GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
}

/**
 * @brief Disable GPADC LDO reference.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_disable_ldoref(GPADC_TypeDef *GPADCx)
{
    CLEAR_BIT(GPADCx->ADC_CFG_REG1, GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
}

/**
 * @brief Check whether GPADC LDO reference is enabled.
 * @param[in] GPADCx GPADC instance pointer.
 * @return Non-zero when LDO reference is enabled.
 */
static inline uint32_t ll_gpadc_is_enabled_ldoref(GPADC_TypeDef *GPADCx)
{
    return READ_BIT(GPADCx->ADC_CFG_REG1,
                    GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
}

/**
 * @brief Enable force channel selection from ADC_CFG_REG1.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_enable_force_channel_select(GPADC_TypeDef *GPADCx)
{
    SET_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_CHNL_SEL_FRC_EN);
}

/**
 * @brief Disable force channel selection from ADC_CFG_REG1.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_disable_force_channel_select(GPADC_TypeDef *GPADCx)
{
    CLEAR_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_CHNL_SEL_FRC_EN);
}

/**
 * @brief Read ADC_RDATA0 register.
 * @param[in] GPADCx GPADC instance pointer.
 * @return ADC_RDATA0 raw register value.
 */
static inline uint32_t ll_gpadc_read_rdata0(GPADC_TypeDef *GPADCx)
{
    return READ_REG(GPADCx->ADC_RDATA0);
}

/**
 * @brief Read ADC_RDATA1 register.
 * @param[in] GPADCx GPADC instance pointer.
 * @return ADC_RDATA1 raw register value.
 */
static inline uint32_t ll_gpadc_read_rdata1(GPADC_TypeDef *GPADCx)
{
    return READ_REG(GPADCx->ADC_RDATA1);
}

/**
 * @brief Read ADC_RDATA2 register.
 * @param[in] GPADCx GPADC instance pointer.
 * @return ADC_RDATA2 raw register value.
 */
static inline uint32_t ll_gpadc_read_rdata2(GPADC_TypeDef *GPADCx)
{
    return READ_REG(GPADCx->ADC_RDATA2);
}

/**
 * @brief Read ADC_RDATA3 register.
 * @param[in] GPADCx GPADC instance pointer.
 * @return ADC_RDATA3 raw register value.
 */
static inline uint32_t ll_gpadc_read_rdata3(GPADC_TypeDef *GPADCx)
{
    return READ_REG(GPADCx->ADC_RDATA3);
}

/**
 * @brief Read one slot conversion sample.
 * @param[in] GPADCx GPADC instance pointer.
 * @param[in] slot_index Slot index, valid range 0..7.
 * @return 12-bit slot sample data.
 */
static inline uint32_t ll_gpadc_get_slot_data(GPADC_TypeDef *GPADCx,
                                              uint32_t slot_index)
{
    uint32_t rdata;

    rdata = *((&GPADCx->ADC_RDATA0) + (slot_index >> 1U));
    return ((slot_index & 1U) == 0U) ? (rdata & 0x00000FFFUL)
                                     : ((rdata >> 16U) & 0x00000FFFUL);
}

/**
 * @brief Read DMA combined/raw data field from ADC_DMA_RDATA.
 * @param[in] GPADCx GPADC instance pointer.
 * @return DMA_RDATA[12:0] field value.
 */
static inline uint32_t ll_gpadc_read_dma_data(GPADC_TypeDef *GPADCx)
{
    return (READ_REG(GPADCx->ADC_DMA_RDATA) & 0x00001FFFUL);
}

/**
 * @brief Read DMA raw data field from ADC_DMA_RDATA.
 * @param[in] GPADCx GPADC instance pointer.
 * @return DMA_RDATA_RAW[28:16] field value.
 */
static inline uint32_t ll_gpadc_read_dma_raw_data(GPADC_TypeDef *GPADCx)
{
    return ((READ_REG(GPADCx->ADC_DMA_RDATA) >> 16U) & 0x00001FFFUL);
}

/**
 * @brief Check ADC_DONE status flag.
 * @param[in] GPADCx GPADC instance pointer.
 * @return Non-zero when ADC_DONE is set.
 */
static inline uint32_t ll_gpadc_is_active_flag_adc_done(GPADC_TypeDef *GPADCx)
{
    return READ_BIT(GPADCx->GPADC_STATUS, GPADC_GPADC_STATUS_ADC_DONE);
}

/**
 * @brief Get SLOT_DONE bitmap.
 * @param[in] GPADCx GPADC instance pointer.
 * @return SLOT_DONE bits in position [8:1].
 */
static inline uint32_t ll_gpadc_get_slot_done_mask(GPADC_TypeDef *GPADCx)
{
    return READ_BIT(GPADCx->GPADC_STATUS, GPADC_GPADC_STATUS_SLOT_DONE);
}

/**
 * @brief Get currently converting slot index.
 * @param[in] GPADCx GPADC instance pointer.
 * @return CUR_SLOT field value in range 0..7.
 */
static inline uint32_t ll_gpadc_get_current_slot(GPADC_TypeDef *GPADCx)
{
    return ((READ_REG(GPADCx->GPADC_STATUS) & GPADC_GPADC_STATUS_CUR_SLOT) >>
            GPADC_GPADC_STATUS_CUR_SLOT_Pos);
}

/**
 * @brief Check raw GPADC IRQ flag.
 * @param[in] GPADCx GPADC instance pointer.
 * @return Non-zero when raw IRQ flag is active.
 */
static inline uint32_t ll_gpadc_is_active_flag_irq_raw(GPADC_TypeDef *GPADCx)
{
    return READ_BIT(GPADCx->GPADC_IRQ, GPADC_GPADC_IRQ_GPADC_IRSR);
}

/**
 * @brief Check masked GPADC IRQ flag.
 * @param[in] GPADCx GPADC instance pointer.
 * @return Non-zero when masked IRQ flag is active.
 */
static inline uint32_t ll_gpadc_is_active_flag_irq_masked(GPADC_TypeDef *GPADCx)
{
    return READ_BIT(GPADCx->GPADC_IRQ, GPADC_GPADC_IRQ_GPADC_ISR);
}

/**
 * @brief Clear GPADC IRQ flag.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_clear_flag_irq(GPADC_TypeDef *GPADCx)
{
    SET_BIT(GPADCx->GPADC_IRQ, GPADC_GPADC_IRQ_GPADC_ICR);
}

/**
 * @brief Enable end-of-conversion interrupt.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_enable_it_eoc(GPADC_TypeDef *GPADCx)
{
    CLEAR_BIT(GPADCx->GPADC_IRQ, GPADC_GPADC_IRQ_GPADC_IMR);
}

/**
 * @brief Disable end-of-conversion interrupt.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_disable_it_eoc(GPADC_TypeDef *GPADCx)
{
    SET_BIT(GPADCx->GPADC_IRQ, GPADC_GPADC_IRQ_GPADC_IMR);
}

/**
 * @brief Check whether end-of-conversion interrupt is enabled.
 * @param[in] GPADCx GPADC instance pointer.
 * @return Non-zero when interrupt is enabled.
 */
static inline uint32_t ll_gpadc_is_enabled_it_eoc(GPADC_TypeDef *GPADCx)
{
    return (READ_BIT(GPADCx->GPADC_IRQ, GPADC_GPADC_IRQ_GPADC_IMR) == 0U);
}

/**
 * @brief Enable GPADC DMA interface.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_enable_dma(GPADC_TypeDef *GPADCx)
{
    SET_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_DMA_EN);
}

/**
 * @brief Disable GPADC DMA interface.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_disable_dma(GPADC_TypeDef *GPADCx)
{
    CLEAR_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_DMA_EN);
}

/**
 * @brief Check whether GPADC DMA interface is enabled.
 * @param[in] GPADCx GPADC instance pointer.
 * @return Non-zero when DMA is enabled.
 */
static inline uint32_t ll_gpadc_is_enabled_dma(GPADC_TypeDef *GPADCx)
{
    return READ_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_DMA_EN);
}

/**
 * @brief Select DMA combined data output.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_select_dma_combined_data(GPADC_TypeDef *GPADCx)
{
    CLEAR_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_DMA_DATA_SEL);
}

/**
 * @brief Select DMA raw data output.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_select_dma_raw_data(GPADC_TypeDef *GPADCx)
{
    SET_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_DMA_DATA_SEL);
}

/**
 * @brief Request ADC start conversion.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_request_start(GPADC_TypeDef *GPADCx)
{
    SET_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_ADC_START);
}

/**
 * @brief Request ADC stop in continuous mode.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_request_stop(GPADC_TypeDef *GPADCx)
{
    SET_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_ADC_STOP);
}

/**
 * @brief Clear ADC stop request.
 * @param[in] GPADCx GPADC instance pointer.
 */
static inline void ll_gpadc_clear_stop_request(GPADC_TypeDef *GPADCx)
{
    CLEAR_BIT(GPADCx->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_ADC_STOP);
}

#ifdef __cplusplus
}
#endif

#endif /* __LL_GPADC_H */
