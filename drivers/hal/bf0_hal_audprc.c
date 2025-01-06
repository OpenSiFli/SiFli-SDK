/**
  ******************************************************************************
  * @file   bf0_hal_audprc.c
  * @author Sifli software development team
  * @brief   AUDPRC HAL module driver.
  *          This file provides firmware functions to manage the following
  ******************************************************************************
*/
/**
 *
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"

#if defined(HAL_AUDPRC_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)
//#define DBG_LEVEL                DBG_LOG
//#define LOG_TAG              "hal.audprc"
//#include "drv_log.h"


static void HAL_AUDPRC_DMA_Init(DMA_HandleTypeDef          *hdma, uint8_t from_mem);

static void AUDPRC_DMATxCplt(DMA_HandleTypeDef *hdma);
static void AUDPRC_DMATxHalfCplt(DMA_HandleTypeDef *hdma);
static void AUDPRC_DMARxCplt(DMA_HandleTypeDef *hdma);
static void AUDPRC_DMAHalfRxCplt(DMA_HandleTypeDef *hdma);
static void AUDPRC_DMAError(DMA_HandleTypeDef *hdma);

/**
  * @brief Initializes the AUDPRC according to the specified parameters
  *         in the AUDPRC_InitTypeDef and create the associated handle.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Init(AUDPRC_HandleTypeDef *haprc)
{
    int i = 0;
    /* Check the AUDPRC handle allocation */
    if (haprc == NULL)
    {
        return HAL_ERROR;
    }

    if (haprc->State[0] == HAL_AUDPRC_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        haprc->Lock = HAL_UNLOCKED;

        /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        HAL_AUDPRC_MspInit(haprc);
    }

    for (i = 0; i < HAL_AUDPRC_INSTANC_CNT; i++)
    {
        haprc->State[i] = HAL_AUDPRC_STATE_BUSY;
    }

    // Set clock divider
    MODIFY_REG(haprc->Instance->CFG, AUDPRC_CFG_AUDCLK_DIV_Msk,
               MAKE_REG_VAL(haprc->Init.clk_div, AUDPRC_CFG_AUDCLK_DIV_Msk, AUDPRC_CFG_AUDCLK_DIV_Pos));

    __HAL_AUDPRC_AUTO_GATE_EN(haprc);

    // Set adc/dac clock divider
    uint32_t val = ((uint32_t)(haprc->Init.adc_div) << AUDPRC_STB_ADC_DIV_Pos) | ((uint32_t)(haprc->Init.dac_div) << AUDPRC_STB_DAC_DIV_Pos);
    haprc->Instance->STB = val;

    /* Disable the AUDPRC  */
    __HAL_AUDPRC_DISABLE(haprc);


    for (i = 0; i < HAL_AUDPRC_INSTANC_CNT; i++)
        if (haprc->hdma[i] != NULL)
            haprc->hdma[i]->Parent = (void *)haprc;

    for (i = HAL_AUDPRC_TX_CH0; i < HAL_AUDPRC_RX_CH0; i++)
    {
        if (haprc->hdma[i] != NULL)
            HAL_AUDPRC_DMA_Init(haprc->hdma[i], 1);
    }
    for (i = HAL_AUDPRC_RX_CH0; i < HAL_AUDPRC_TX_OUT_CH0; i++)
    {
        if (haprc->hdma[i] != NULL)
            HAL_AUDPRC_DMA_Init(haprc->hdma[i], 0);
    }
    for (i = HAL_AUDPRC_TX_OUT_CH0; i < HAL_AUDPRC_INSTANC_CNT; i++)
    {
        if (haprc->hdma[i] != NULL)
            HAL_AUDPRC_DMA_Init(haprc->hdma[i], 0);
    }
    HAL_AUDPRC_Config_DACPath(haprc, &(haprc->Init.dac_cfg));
    HAL_AUDPRC_Config_ADCPath(haprc, &(haprc->Init.adc_cfg));
    //LOG_I("init ADC_PATH_CFG0 0x%x\n",  haprc->Instance->ADC_PATH_CFG0);
#if 0
    // configure TX DMA, instance and request should be initialized by caller
    if (haprc->hdmatx0 != NULL)
    {
        haprc->hdmatx0->Parent = (void *)haprc;
        HAL_AUDPRC_DMA_Init(haprc->hdmatx0, 1);
    }
    if (haprc->hdmatx1 != NULL)
    {
        haprc->hdmatx1->Parent = (void *)haprc;
        HAL_AUDPRC_DMA_Init(haprc->hdmatx1, 1);
    }
    if (haprc->hdmatx2 != NULL)
    {
        haprc->hdmatx2->Parent = (void *)haprc;
        HAL_AUDPRC_DMA_Init(haprc->hdmatx2, 1);
    }
    if (haprc->hdmatx3 != NULL)
    {
        haprc->hdmatx3->Parent = (void *)haprc;
        HAL_AUDPRC_DMA_Init(haprc->hdmatx3, 1);
    }

    // configure RX DMA, instance and request should be initialized by caller
    if (haprc->hdmarx0 != NULL)
    {
        haprc->hdmarx0->Parent = (void *)haprc;
        HAL_AUDPRC_DMA_Init(haprc->hdmarx0, 0);
    }
    if (haprc->hdmarx1 != NULL)
    {
        haprc->hdmarx1->Parent = (void *)haprc;
        HAL_AUDPRC_DMA_Init(haprc->hdmarx1, 0);
    }

    // configure TX OUT DMA
    if (haprc->hdmatoc0 != NULL)
    {
        haprc->hdmatoc0->Parent = (void *)haprc;
        HAL_AUDPRC_DMA_Init(haprc->hdmatoc0, 0);
    }
    if (haprc->hdmatoc1 != NULL)
    {
        haprc->hdmatoc1->Parent = (void *)haprc;
        HAL_AUDPRC_DMA_Init(haprc->hdmatoc1, 0);
    }
#endif
    haprc->ErrorCode = HAL_AUDPRC_ERROR_NONE;
    for (i = 0; i < HAL_AUDPRC_INSTANC_CNT; i++)
    {
        haprc->State[i] = HAL_AUDPRC_STATE_READY;
    }

    return HAL_OK;
}

/**
  * @brief DeInitializes the AUDPRC peripheral
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_DeInit(AUDPRC_HandleTypeDef *haprc)
{
    int i;

    /* Check the AUDPRC handle allocation */
    if (haprc == NULL)
    {
        return HAL_ERROR;
    }


#if 1
    for (i = 0; i < HAL_AUDPRC_INSTANC_CNT; i++)
    {
        haprc->State[i] |= HAL_AUDPRC_STATE_BUSY;
        if (haprc->hdma[i] != NULL)
            HAL_DMA_DeInit(haprc->hdma[i]);
    }
#else
    if (haprc->hdmatx0 != NULL)
    {
        HAL_DMA_DeInit(haprc->hdmatx0);
    }
    if (haprc->hdmatx1 != NULL)
    {
        HAL_DMA_DeInit(haprc->hdmatx1);
    }
    if (haprc->hdmatx2 != NULL)
    {
        HAL_DMA_DeInit(haprc->hdmatx2);
    }
    if (haprc->hdmatx3 != NULL)
    {
        HAL_DMA_DeInit(haprc->hdmatx3);
    }
    if (haprc->hdmarx0 != NULL)
    {
        HAL_DMA_DeInit(haprc->hdmarx0);
    }
    if (haprc->hdmarx1 != NULL)
    {
        HAL_DMA_DeInit(haprc->hdmarx1);
    }
    if (haprc->hdmatoc0 != NULL)
    {
        HAL_DMA_DeInit(haprc->hdmatoc0);
    }
    if (haprc->hdmatoc1 != NULL)
    {
        HAL_DMA_DeInit(haprc->hdmatoc1);
    }
#endif
    /* Disable the AUDPRC Peripheral Clock */
    __HAL_AUDPRC_DISABLE(haprc);

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    HAL_AUDPRC_MspDeInit(haprc);

    haprc->ErrorCode = HAL_AUDPRC_ERROR_NONE;
    for (i = 0; i < HAL_AUDPRC_INSTANC_CNT; i++)
    {
        haprc->State[i] |= HAL_AUDPRC_STATE_RESET;
    }

    /* Release Lock */
    __HAL_UNLOCK(haprc);

    return HAL_OK;
}

/**
  * @brief Initializes the AUDPRC pin setting.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @retval None
  */
__HAL_ROM_USED void HAL_AUDPRC_MspInit(AUDPRC_HandleTypeDef *haprc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(haprc);
}

/**
  * @brief deunitializes the AUDPRC pin setting.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @retval None
  */
__HAL_ROM_USED void HAL_AUDPRC_MspDeInit(AUDPRC_HandleTypeDef *haprc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(haprc);
}

/**
  * @brief Set AUDPRC transmit channel configures.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param cfg: configure parameters
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Config_TChanel(AUDPRC_HandleTypeDef *haprc, int channel, AUDPRC_ChnlCfgTypeDef *cfg)
{
    uint32_t value;

    /* Check the AUDPRC handle allocation */
    if (haprc == NULL)
    {
        return HAL_ERROR;
    }

    switch (channel)
    {
    case 0:
        if (cfg->format == 1) // for 24 bit mode, only support mono --- mode = 0
            value = (cfg->dma_mask << AUDPRC_TX_CH0_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_TX_CH0_CFG_FORMAT_Pos)
                    | (cfg->en << AUDPRC_TX_CH0_CFG_ENABLE_Pos);
        else
            value = (cfg->dma_mask << AUDPRC_TX_CH0_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_TX_CH0_CFG_FORMAT_Pos)
                    | (cfg->mode << AUDPRC_TX_CH0_CFG_MODE_Pos) | (cfg->en << AUDPRC_TX_CH0_CFG_ENABLE_Pos);
        haprc->Instance->TX_CH0_CFG = value;
        break;
    case 1:
        value = (cfg->dma_mask << AUDPRC_TX_CH1_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_TX_CH1_CFG_FORMAT_Pos)
                | (cfg->en << AUDPRC_TX_CH1_CFG_ENABLE_Pos);
        haprc->Instance->TX_CH1_CFG = value;
        break;
    case 2:
        if (cfg->format == 1) // for 24 bit mode, only support mono --- mode = 0
            value = (cfg->dma_mask << AUDPRC_TX_CH2_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_TX_CH2_CFG_FORMAT_Pos)
                    | (cfg->en << AUDPRC_TX_CH2_CFG_ENABLE_Pos);
        else
            value = (cfg->dma_mask << AUDPRC_TX_CH2_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_TX_CH2_CFG_FORMAT_Pos)
                    | (cfg->mode << AUDPRC_TX_CH2_CFG_MODE_Pos) | (cfg->en << AUDPRC_TX_CH2_CFG_ENABLE_Pos);
        haprc->Instance->TX_CH2_CFG = value;
        break;
    case 3:
        value = (cfg->dma_mask << AUDPRC_TX_CH3_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_TX_CH3_CFG_FORMAT_Pos)
                | (cfg->en << AUDPRC_TX_CH3_CFG_ENABLE_Pos);
        haprc->Instance->TX_CH3_CFG = value;
        break;
    default:
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief Set AUDPRC Receive configures.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param cfg: configure parameters
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Config_RChanel(AUDPRC_HandleTypeDef *haprc, int channel, AUDPRC_ChnlCfgTypeDef *cfg)
{
    uint32_t value;

    /* Check the AUDPRC handle allocation */
    if (haprc == NULL)
    {
        return HAL_ERROR;
    }

    switch (channel)
    {
    case 0:
        if (cfg->format == 1) // for 24 bit mode, only support mono --- mode = 0
            value = (cfg->dma_mask << AUDPRC_RX_CH0_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_RX_CH0_CFG_FORMAT_Pos)
                    | (cfg->en << AUDPRC_RX_CH0_CFG_ENABLE_Pos);
        else
            value = (cfg->dma_mask << AUDPRC_RX_CH0_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_RX_CH0_CFG_FORMAT_Pos)
                    | (cfg->mode << AUDPRC_RX_CH0_CFG_MODE_Pos) | (cfg->en << AUDPRC_RX_CH0_CFG_ENABLE_Pos);
        haprc->Instance->RX_CH0_CFG = value;
        break;
    case 1:
        value = (cfg->dma_mask << AUDPRC_RX_CH1_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_RX_CH1_CFG_FORMAT_Pos)
                | (cfg->en << AUDPRC_RX_CH1_CFG_ENABLE_Pos);
        haprc->Instance->RX_CH1_CFG = value;
        break;
    default:
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief Set AUDPRC transmit output channel configures.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param cfg: configure parameters
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Config_OutChanel(AUDPRC_HandleTypeDef *haprc, int channel, AUDPRC_ChnlCfgTypeDef *cfg)
{
    uint32_t value;

    /* Check the AUDPRC handle allocation */
    if (haprc == NULL)
    {
        return HAL_ERROR;
    }

    switch (channel)
    {
    case 0:
        if (cfg->format == 1) // for 24 bit mode, only support mono --- mode = 0
            value = (cfg->dma_mask << AUDPRC_TX_OUT_CH0_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_TX_OUT_CH0_CFG_FORMAT_Pos)
                    | (cfg->en << AUDPRC_TX_OUT_CH0_CFG_ENABLE_Pos);
        else
            value = (cfg->dma_mask << AUDPRC_TX_OUT_CH0_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_TX_OUT_CH0_CFG_FORMAT_Pos)
                    | (cfg->mode << AUDPRC_TX_OUT_CH0_CFG_MODE_Pos) | (cfg->en << AUDPRC_TX_OUT_CH0_CFG_ENABLE_Pos);
        haprc->Instance->TX_OUT_CH0_CFG = value;
        break;
    case 1:
        value = (cfg->dma_mask << AUDPRC_TX_OUT_CH1_CFG_DMA_MSK_Pos) | (cfg->format << AUDPRC_TX_OUT_CH1_CFG_FORMAT_Pos)
                | (cfg->en << AUDPRC_TX_OUT_CH1_CFG_ENABLE_Pos);
        haprc->Instance->TX_OUT_CH1_CFG = value;
        break;
    default:
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief  all channel disable
 * @param haprc -pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
 * @return HAL status
 */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Clear_All_Channel(AUDPRC_HandleTypeDef *haprc)
{
    haprc->Instance->TX_CH0_CFG = 0;
    haprc->Instance->TX_CH1_CFG = 0;
    haprc->Instance->TX_CH2_CFG = 0;
    haprc->Instance->TX_CH3_CFG = 0;
    haprc->Instance->RX_CH0_CFG = 0;
    haprc->Instance->RX_CH1_CFG = 0;
    haprc->Instance->TX_OUT_CH0_CFG = 0;
    haprc->Instance->TX_OUT_CH1_CFG = 0;
    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Clear_Dac_Channel(AUDPRC_HandleTypeDef *haprc)
{
    haprc->Instance->TX_CH0_CFG = 0;
    haprc->Instance->TX_CH1_CFG = 0;
    haprc->Instance->TX_CH2_CFG = 0;
    haprc->Instance->TX_CH3_CFG = 0;
    haprc->Instance->TX_OUT_CH0_CFG = 0;
    haprc->Instance->TX_OUT_CH1_CFG = 0;
    return HAL_OK;
}
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Clear_Adc_Channel(AUDPRC_HandleTypeDef *haprc)
{
    haprc->Instance->RX_CH0_CFG = 0;
    haprc->Instance->RX_CH1_CFG = 0;
    return HAL_OK;
}


/**
  * @brief  Configure DAC path volume.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param channel: channel 0 or 1
  * @param volume: configure volume -36dB~60dB
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Config_DACPath_Volume(AUDPRC_HandleTypeDef *haprc, int channel, int volume)
{
    uint32_t rough_vol, fine_vol;

    /* Check the AUDPRC handle allocation */
    if (haprc == NULL)
    {
        return HAL_ERROR;
    }

    if ((channel < 0) || (channel > 1))
    {
        return HAL_ERROR;
    }
#ifdef SF32LB52X

    if ((volume < -36) || (volume > 60))
    {
        return HAL_ERROR;
    }

    rough_vol = (volume + 36) / 6;
    fine_vol  = ((volume + 36) % 6) << 1;

    if (channel == 0)
    {
        MODIFY_REG(haprc->Instance->DAC_PATH_CFG0, AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_L_Msk | AUDPRC_DAC_PATH_CFG0_FINE_VOL_L_Msk, \
                   MAKE_REG_VAL(rough_vol, AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_L_Msk, AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_L_Pos) |
                   MAKE_REG_VAL(fine_vol, AUDPRC_DAC_PATH_CFG0_FINE_VOL_L_Msk, AUDPRC_DAC_PATH_CFG0_FINE_VOL_L_Pos));
    }
    else
    {
        MODIFY_REG(haprc->Instance->DAC_PATH_CFG0, AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_R_Msk | AUDPRC_DAC_PATH_CFG0_FINE_VOL_R_Msk, \
                   MAKE_REG_VAL(rough_vol, AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_R_Msk, AUDPRC_DAC_PATH_CFG0_ROUGH_VOL_R_Pos) |
                   MAKE_REG_VAL(fine_vol, AUDPRC_DAC_PATH_CFG0_FINE_VOL_R_Msk, AUDPRC_DAC_PATH_CFG0_FINE_VOL_R_Pos));
    }
#else

    if ((volume < -18) || (volume > 13))
    {
        return HAL_ERROR;
    }

    uint32_t value = (volume + 18) << 1;

    if (channel == 0)
    {
        MODIFY_REG(haprc->Instance->DAC_PATH_CFG0, AUDPRC_DAC_PATH_CFG0_VOL_L_Msk, \
                   MAKE_REG_VAL(value, AUDPRC_DAC_PATH_CFG0_VOL_L_Msk, AUDPRC_DAC_PATH_CFG0_VOL_L_Pos));
    }
    else
    {
        MODIFY_REG(haprc->Instance->DAC_PATH_CFG0, AUDPRC_DAC_PATH_CFG0_VOL_R_Msk, \
                   MAKE_REG_VAL(value, AUDPRC_DAC_PATH_CFG0_VOL_R_Msk, AUDPRC_DAC_PATH_CFG0_VOL_R_Pos));
    }
#endif

    //LOG_E("set volume rough:%d, fine:%d, cfg0:0x%x", rough_vol, fine_vol, haprc->Instance_hp->DAC_CH0_CFG);

    return HAL_OK;
}

/**
  * @brief  Configure DAC path.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param cfg: configure parameters
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Config_DACPath(AUDPRC_HandleTypeDef *haprc, AUDPRC_DACCfgTypeDef *cfg)
{
    uint32_t value;

    /* Check the AUDPRC handle allocation */
    if (haprc == NULL)
    {
        return HAL_ERROR;
    }
    value = haprc->Instance->DAC_PATH_CFG0;
    value &= ~(AUDPRC_DAC_PATH_CFG0_MIXLSRC0_Msk | AUDPRC_DAC_PATH_CFG0_MIXLSRC1_Msk | AUDPRC_DAC_PATH_CFG0_MIXRSRC0_Msk | AUDPRC_DAC_PATH_CFG0_MIXRSRC1_Msk);
    value |= MAKE_REG_VAL(cfg->mixlsrc0, AUDPRC_DAC_PATH_CFG0_MIXLSRC0_Msk, AUDPRC_DAC_PATH_CFG0_MIXLSRC0_Pos)
             | MAKE_REG_VAL(cfg->mixlsrc1, AUDPRC_DAC_PATH_CFG0_MIXLSRC1_Msk, AUDPRC_DAC_PATH_CFG0_MIXLSRC1_Pos)
             | MAKE_REG_VAL(cfg->mixrsrc0, AUDPRC_DAC_PATH_CFG0_MIXRSRC0_Msk, AUDPRC_DAC_PATH_CFG0_MIXRSRC0_Pos)
             | MAKE_REG_VAL(cfg->mixrsrc1, AUDPRC_DAC_PATH_CFG0_MIXRSRC1_Msk, AUDPRC_DAC_PATH_CFG0_MIXRSRC1_Pos);
    //| MAKE_REG_VAL(cfg->dst_sel, AUDPRC_DAC_PATH_CFG0_DST_SEL_Msk, AUDPRC_DAC_PATH_CFG0_DST_SEL_Pos);
    haprc->Instance->DAC_PATH_CFG0 = value;

    HAL_AUDPRC_Config_DACPath_Volume(haprc, 0, cfg->vol_l);
    HAL_AUDPRC_Config_DACPath_Volume(haprc, 1, cfg->vol_r);

    value = MAKE_REG_VAL(cfg->muxlsrc0, AUDPRC_DAC_PATH_CFG1_MUXLSRC0_Msk, AUDPRC_DAC_PATH_CFG1_MUXLSRC0_Pos)
            | MAKE_REG_VAL(cfg->muxlsrc1, AUDPRC_DAC_PATH_CFG1_MUXLSRC1_Msk, AUDPRC_DAC_PATH_CFG1_MUXLSRC1_Pos)
            | MAKE_REG_VAL(cfg->muxrsrc0, AUDPRC_DAC_PATH_CFG1_MUXRSRC0_Msk, AUDPRC_DAC_PATH_CFG1_MUXRSRC0_Pos)
            | MAKE_REG_VAL(cfg->muxrsrc1, AUDPRC_DAC_PATH_CFG1_MUXRSRC1_Msk, AUDPRC_DAC_PATH_CFG1_MUXRSRC1_Pos)
            //| MAKE_REG_VAL(cfg->eq_ch_en, AUDPRC_DAC_PATH_CFG1_EQ_CH_EN_Msk, AUDPRC_DAC_PATH_CFG1_EQ_CH_EN_Pos)
            //| MAKE_REG_VAL(cfg->eq_stage, AUDPRC_DAC_PATH_CFG1_EQ_STAGE_Msk, AUDPRC_DAC_PATH_CFG1_EQ_STAGE_Pos)
            //| MAKE_REG_VAL(cfg->eq_clr, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Msk, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Pos)
            | MAKE_REG_VAL(cfg->src_ch_en, AUDPRC_DAC_PATH_CFG1_SRC_CH_EN_Msk, AUDPRC_DAC_PATH_CFG1_SRC_CH_EN_Pos)
            | MAKE_REG_VAL(cfg->src_hbf1_en, AUDPRC_DAC_PATH_CFG1_SRC_HBF1_EN_Msk, AUDPRC_DAC_PATH_CFG1_SRC_HBF1_EN_Pos)
            | MAKE_REG_VAL(cfg->src_hbf1_mode, AUDPRC_DAC_PATH_CFG1_SRC_HBF1_MODE_Msk, AUDPRC_DAC_PATH_CFG1_SRC_HBF1_MODE_Pos)
            | MAKE_REG_VAL(cfg->src_hbf2_en, AUDPRC_DAC_PATH_CFG1_SRC_HBF2_EN_Msk, AUDPRC_DAC_PATH_CFG1_SRC_HBF2_EN_Pos)
            | MAKE_REG_VAL(cfg->src_hbf2_mode, AUDPRC_DAC_PATH_CFG1_SRC_HBF2_MODE_Msk, AUDPRC_DAC_PATH_CFG1_SRC_HBF2_MODE_Pos)
            | MAKE_REG_VAL(cfg->src_hbf3_en, AUDPRC_DAC_PATH_CFG1_SRC_HBF3_EN_Msk, AUDPRC_DAC_PATH_CFG1_SRC_HBF3_EN_Pos)
            | MAKE_REG_VAL(cfg->src_hbf3_mode, AUDPRC_DAC_PATH_CFG1_SRC_HBF3_MODE_Msk, AUDPRC_DAC_PATH_CFG1_SRC_HBF3_MODE_Pos);
#ifndef SF32LB58X
    value |= (3 << AUDPRC_DAC_PATH_CFG1_SRC_CH_CLR_Pos);
#endif
    haprc->Instance->DAC_PATH_CFG1 = value;

#ifndef SF32LB58X
    while ((haprc->Instance->DAC_PATH_CFG1 & AUDPRC_DAC_PATH_CFG1_SRC_CH_CLR_DONE) == 0);
    value &= (~AUDPRC_DAC_PATH_CFG1_SRC_CH_CLR);
    haprc->Instance->DAC_PATH_CFG1 = value;
#endif

    value = MAKE_REG_VAL(cfg->sinc_ratio, AUDPRC_DAC_PATH_CFG2_SINC_RATIO_Msk, AUDPRC_DAC_PATH_CFG2_SINC_RATIO_Pos)
            | MAKE_REG_VAL(cfg->src_sinc_en, AUDPRC_DAC_PATH_CFG2_SRC_SINC_EN_Msk, AUDPRC_DAC_PATH_CFG2_SRC_SINC_EN_Pos);
    haprc->Instance->DAC_PATH_CFG2 = value;

    return HAL_OK;
}


/**
  * @brief  Configure ADC path volume.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param channel: channel 0 or 1
  * @param volume: configure volume -36dB~60dB
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Config_ADCPath_Volume(AUDPRC_HandleTypeDef *haprc, int channel, int volume)
{
    uint32_t rough_vol, fine_vol;

    /* Check the AUDPRC handle allocation */
    if (haprc == NULL)
    {
        return HAL_ERROR;
    }

    if ((channel < 0) || (channel > 1))
    {
        return HAL_ERROR;
    }
#ifdef SF32LB52X

    if ((volume < -36) || (volume > 60))
    {
        return HAL_ERROR;
    }

    rough_vol = (volume + 36) / 6;
    fine_vol  = ((volume + 36) % 6) << 1;

    //fine_vol = 0xF;//MUTE

    if (channel == 0)
    {
        MODIFY_REG(haprc->Instance->ADC_PATH_CFG0, AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_L_Msk | AUDPRC_ADC_PATH_CFG0_FINE_VOL_L_Msk, \
                   MAKE_REG_VAL(rough_vol, AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_L_Msk, AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_L_Pos) |
                   MAKE_REG_VAL(fine_vol, AUDPRC_ADC_PATH_CFG0_FINE_VOL_L_Msk, AUDPRC_ADC_PATH_CFG0_FINE_VOL_L_Pos));
    }
    else
    {
        MODIFY_REG(haprc->Instance->ADC_PATH_CFG0, AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_R_Msk | AUDPRC_ADC_PATH_CFG0_FINE_VOL_R_Msk, \
                   MAKE_REG_VAL(rough_vol, AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_R_Msk, AUDPRC_ADC_PATH_CFG0_ROUGH_VOL_R_Pos) |
                   MAKE_REG_VAL(fine_vol, AUDPRC_ADC_PATH_CFG0_FINE_VOL_R_Msk, AUDPRC_ADC_PATH_CFG0_FINE_VOL_R_Pos));
    }
#else

    if ((volume < -18) || (volume > 13))
    {
        return HAL_ERROR;
    }

    uint32_t value = (volume + 18) << 1;

    if (channel == 0)
    {
        MODIFY_REG(haprc->Instance->ADC_PATH_CFG0, AUDPRC_ADC_PATH_CFG0_VOL_L_Msk, \
                   MAKE_REG_VAL(value, AUDPRC_ADC_PATH_CFG0_VOL_L_Msk, AUDPRC_ADC_PATH_CFG0_VOL_L_Pos));
    }
    else
    {
        MODIFY_REG(haprc->Instance->ADC_PATH_CFG0, AUDPRC_ADC_PATH_CFG0_VOL_R_Msk, \
                   MAKE_REG_VAL(value, AUDPRC_ADC_PATH_CFG0_VOL_R_Msk, AUDPRC_ADC_PATH_CFG0_VOL_R_Pos));
    }
#endif


    //LOG_E("set volume rough:%d, fine:%d, cfg0:0x%x", rough_vol, fine_vol, haprc->Instance_hp->DAC_CH0_CFG);

    return HAL_OK;
}

/**
  * @brief  Configure ADC path.
  * @param  haprc: pointer to a AUDPRC_HandleTypeDef structure that contains
  *         the configuration information for AUDPRC module
  * @param cfg: configure parameters
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Config_ADCPath(AUDPRC_HandleTypeDef *haprc, AUDPRC_ADCCfgTypeDef *cfg)
{
    uint32_t value;

    /* Check the AUDPRC handle allocation */
    if (haprc == NULL)
    {
        return HAL_ERROR;
    }
    value = haprc->Instance->ADC_PATH_CFG0;
    value &= (AUDPRC_ADC_PATH_CFG0_SRC_SEL_Msk | AUDPRC_ADC_PATH_CFG0_DATA_SWAP_Msk
#ifdef SF32LB58X
              | AUDPRC_ADC_PATH_CFG0_SRC_CH_EN_Msk | AUDPRC_ADC_PATH_CFG0_SRC_HBF1_EN_Msk
              | AUDPRC_ADC_PATH_CFG0_SRC_HBF1_MODE_Msk | AUDPRC_ADC_PATH_CFG0_SRC_HBF2_EN_Msk | AUDPRC_ADC_PATH_CFG0_SRC_HBF2_MODE_Msk
              | AUDPRC_ADC_PATH_CFG0_SRC_HBF3_EN_Msk | AUDPRC_ADC_PATH_CFG0_SRC_HBF3_MODE_Msk
#endif
              | AUDPRC_ADC_PATH_CFG0_RX2TX_LOOPBACK_Msk);
    value |= MAKE_REG_VAL(cfg->data_swap, AUDPRC_ADC_PATH_CFG0_DATA_SWAP_Msk, AUDPRC_ADC_PATH_CFG0_DATA_SWAP_Pos)
#ifdef SF32LB58X
             | MAKE_REG_VAL(cfg->src_ch_en, AUDPRC_ADC_PATH_CFG0_SRC_CH_EN_Msk, AUDPRC_ADC_PATH_CFG0_SRC_CH_EN_Pos)
             | MAKE_REG_VAL(cfg->src_hbf1_en, AUDPRC_ADC_PATH_CFG0_SRC_HBF1_EN_Msk, AUDPRC_ADC_PATH_CFG0_SRC_HBF1_EN_Pos)
             | MAKE_REG_VAL(cfg->src_hbf1_mode, AUDPRC_ADC_PATH_CFG0_SRC_HBF1_MODE_Msk, AUDPRC_ADC_PATH_CFG0_SRC_HBF1_MODE_Pos)
             | MAKE_REG_VAL(cfg->src_hbf2_en, AUDPRC_ADC_PATH_CFG0_SRC_HBF2_EN_Msk, AUDPRC_ADC_PATH_CFG0_SRC_HBF2_EN_Pos)
             | MAKE_REG_VAL(cfg->src_hbf2_mode, AUDPRC_ADC_PATH_CFG0_SRC_HBF2_MODE_Msk, AUDPRC_ADC_PATH_CFG0_SRC_HBF2_MODE_Pos)
             | MAKE_REG_VAL(cfg->src_hbf3_en, AUDPRC_ADC_PATH_CFG0_SRC_HBF3_EN_Msk, AUDPRC_ADC_PATH_CFG0_SRC_HBF3_EN_Pos)
             | MAKE_REG_VAL(cfg->src_hbf3_mode, AUDPRC_ADC_PATH_CFG0_SRC_HBF3_MODE_Msk, AUDPRC_ADC_PATH_CFG0_SRC_HBF3_MODE_Pos)
#endif
             | MAKE_REG_VAL(cfg->rx2tx_loopback, AUDPRC_ADC_PATH_CFG0_RX2TX_LOOPBACK_Msk, AUDPRC_ADC_PATH_CFG0_RX2TX_LOOPBACK_Pos);
    haprc->Instance->ADC_PATH_CFG0 = value;

    HAL_AUDPRC_Config_ADCPath_Volume(haprc, 0, cfg->vol_l);
    HAL_AUDPRC_Config_ADCPath_Volume(haprc, 1, cfg->vol_r);

#ifdef SF32LB58X
    value = MAKE_REG_VAL(cfg->sinc_ratio, AUDPRC_ADC_PATH_CFG1_SINC_RATIO_Msk, AUDPRC_ADC_PATH_CFG1_SINC_RATIO_Pos)
            | MAKE_REG_VAL(cfg->src_sinc_en, AUDPRC_ADC_PATH_CFG1_SRC_SINC_EN_Msk, AUDPRC_ADC_PATH_CFG1_SRC_SINC_EN_Pos);
    haprc->Instance->ADC_PATH_CFG1 = value;
#endif
    //LOG_E("ADC_PATH_CFG0 0x%x, 0x%x\n", value, &haprc->Instance->ADC_PATH_CFG0);
    //rt_kprintf("aa ADC_PATH_CFG0 0x%x, 0x%x\n", value, &haprc->Instance->ADC_PATH_CFG0);
    return HAL_OK;
}

static void HAL_AUDPRC_DMA_Init(DMA_HandleTypeDef          *hdma, uint8_t from_mem)
{
    // configure TX DMA, instance and request should be initialized by caller
    if (hdma != NULL)
    {
        if (from_mem)
            hdma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
        else
            hdma->Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma->Init.MemInc              = DMA_MINC_ENABLE;
        hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        /*Mode will be changed, based on destination select*/
        hdma->Init.Mode                = DMA_CIRCULAR; //DMA_CIRCULAR; //DMA_CIRCULAR; //DMA_NORMAL;
        hdma->Init.Priority            = DMA_PRIORITY_HIGH;
        hdma->Init.BurstSize = 0;
        //hdma->Parent = (void *)haprc;

        HAL_DMA_Init(hdma);
    }

}

HAL_StatusTypeDef HAL_AUDPRC_Transmit_DMA(AUDPRC_HandleTypeDef *haprc, uint8_t *pData, uint32_t Size, uint32_t did)
{
    uint32_t txentry;
    uint32_t *dmamask;

    if ((haprc == NULL) || (haprc->hdma[did] == NULL) || (pData == NULL) || (Size == 0U) || (did > HAL_AUDPRC_TX_CH3))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(haprc);


    txentry = (uint32_t)(&haprc->Instance->TX_CH0_ENTRY) + 8 * did;
    dmamask = (uint32_t *)((uint32_t)&haprc->Instance->TX_CH0_CFG + 8 * did);

    if (!(haprc->State[did] & HAL_AUDPRC_STATE_BUSY_TX))
    {
        haprc->buf[did] = pData;
        haprc->State[did] |= HAL_AUDPRC_STATE_BUSY_TX;
        haprc->ErrorCode = HAL_AUDPRC_ERROR_NONE;

        __HAL_AUDPRC_DACPATH_ENABLE(haprc);

        // DMA transmit as 32 bits
        haprc->dataSize[did] = Size >> 2;

        /* Set the AUDPRC Tx DMA Half transfert complete callback */
        if (AUDPRC_TX_TO_MEM == haprc->dest_sel)
        {
            haprc->hdma[did]->XferHalfCpltCallback = NULL;
            __HAL_DMA_CLEAR_CIRCLUAR_MODE(haprc->hdma[did], DMA_CIRCULAR);
        }
        else
        {
            haprc->hdma[did]->XferHalfCpltCallback = AUDPRC_DMATxHalfCplt;
            __HAL_DMA_SET_CIRCLUAR_MODE(haprc->hdma[did], DMA_CIRCULAR);
        }

        /* Set the AUDPRC Tx DMA transfert complete callback */
        haprc->hdma[did]->XferCpltCallback = AUDPRC_DMATxCplt;

        /* Set the DMA error callback */
        haprc->hdma[did]->XferErrorCallback = AUDPRC_DMAError;

        /* Enable the Tx DMA Channel */
        HAL_DMA_Start_IT(haprc->hdma[did], (uint32_t)haprc->buf[did], txentry, haprc->dataSize[did]);

        /* Check if the AUDPRC Tx request is already enabled */
        *dmamask &= ~AUDPRC_TX_CH0_CFG_DMA_MSK;
        *dmamask |= AUDPRC_TX_CH0_CFG_ENABLE;
        /* Check if the AUDPRC is already enabled */
        //__HAL_AUDPRC_ENABLE(haprc);

        /* Process Unlocked */
        __HAL_UNLOCK(haprc);

        return HAL_OK;
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(haprc);
        return HAL_BUSY;
    }
}

HAL_StatusTypeDef HAL_AUDPRC_Receive_DMA(AUDPRC_HandleTypeDef *haprc, uint8_t *pData, uint32_t Size, uint32_t did)
{
    uint32_t rxentry;
    uint32_t *dmamask;

    /* check dma handle */
    if ((haprc == NULL) || (haprc->hdma[did] == NULL) || (pData == NULL) || (Size == 0U) || (did > HAL_AUDPRC_INSTANC_CNT) || (did < HAL_AUDPRC_RX_CH0))
    {
        return  HAL_ERROR;
    }

    if (haprc->State[did] & HAL_AUDPRC_STATE_BUSY_RX)
    {
        return HAL_BUSY;
    }

    /* Process Locked */
    __HAL_LOCK(haprc);
    rxentry = (uint32_t)(&haprc->Instance->RX_CH0_ENTRY) + 8 * (did - HAL_AUDPRC_RX_CH0);
    dmamask = (uint32_t *)((uint32_t)&haprc->Instance->RX_CH0_CFG + 8 * (did - HAL_AUDPRC_RX_CH0));

    /* Set the transaction information */
    haprc->State[did]  = HAL_AUDPRC_STATE_BUSY_RX;
    haprc->ErrorCode   = HAL_AUDPRC_ERROR_NONE;

    if ((did == HAL_AUDPRC_RX_CH0) || (did == HAL_AUDPRC_RX_CH1))
    {
        __HAL_AUDPRC_ADCPATH_ENABLE(haprc);
    }
    haprc->buf[did]    = pData;
    haprc->dataSize[did] = Size >> 2;

    /* Set the AUDPRC RxDMA Half transfer complete callback */
    if (AUDPRC_TX_TO_MEM == haprc->dest_sel)
    {
        haprc->hdma[did]->XferHalfCpltCallback = NULL;
        __HAL_DMA_CLEAR_CIRCLUAR_MODE(haprc->hdma[did], DMA_CIRCULAR);
    }
    else
    {
        haprc->hdma[did]->XferHalfCpltCallback = AUDPRC_DMAHalfRxCplt;
        __HAL_DMA_SET_CIRCLUAR_MODE(haprc->hdma[did], DMA_CIRCULAR);
    }

    /* Set the AUDPRC Rx DMA transfer complete callback */
    haprc->hdma[did]->XferCpltCallback = AUDPRC_DMARxCplt;

    /* Set the AUDPRC error callback */
    haprc->hdma[did]->XferErrorCallback = AUDPRC_DMAError;

    /* Set the AUDPRC AbortCpltCallback */
    haprc->hdma[did]->XferAbortCallback = NULL;


    /* Enable the Rx DMA Stream/Channel  */
    HAL_DMA_Start_IT(haprc->hdma[did], (uint32_t)(rxentry), (uint32_t)haprc->buf[did], haprc->dataSize[did]);

    /* Check if the AUDPRC Rx request is already enabled */
    *dmamask &= ~AUDPRC_RX_CH0_CFG_DMA_MSK;
    *dmamask |= AUDPRC_RX_CH0_CFG_ENABLE;
    /* Check if the AUDPRC is already enabled */
    //__HAL_AUDPRC_ENABLE(haprc);

    /* Process Unlocked */
    __HAL_UNLOCK(haprc);
    return HAL_OK;
}


HAL_StatusTypeDef HAL_AUDPRC_DMAStop(AUDPRC_HandleTypeDef *haprc, uint32_t did)
{
    /* Abort the DMA rx/tx Stream/Channel  */
    if (haprc->hdma[did] != NULL)
    {
        //__HAL_DMA_DISABLE(haprc->hdma[did]);
        HAL_DMA_Abort(haprc->hdma[did]);
    }
    if (did <= HAL_AUDPRC_TX_CH3)
    {
        __HAL_AUDPRC_DACPATH_DISABLE(haprc);
    }
    if ((did == HAL_AUDPRC_RX_CH0) || (did == HAL_AUDPRC_RX_CH1))
    {
        __HAL_AUDPRC_ADCPATH_DISABLE(haprc);
    }
    //__HAL_AUDPRC_DISABLE(haprc);

    //haprc->State = HAL_AUDPRC_STATE_READY;
    return HAL_OK;
}

__weak void HAL_AUDPRC_TxCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(haprc);
    UNUSED(cid);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_AUDPRC_TxHalfCpltCallback should be implemented in the user file
     */

}

__weak void HAL_AUDPRC_TxHalfCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(haprc);
    UNUSED(cid);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_AUDPRC_TxHalfCpltCallback should be implemented in the user file
     */

}

__weak void HAL_AUDPRC_RxHalfCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(haprc);
    UNUSED(cid);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_AUDPRC_RxHalfCpltCallback should be implemented in the user file
     */
}

__weak void HAL_AUDPRC_RxCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(haprc);
    UNUSED(cid);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_AUDPRC_RxCpltCallback should be implemented in the user file
     */
}

__weak void HAL_AUDPRC_ErrorCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(haprc);
    UNUSED(cid);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_PDM_ErrorCallback should be implemented in the user file
     */
}

__weak void HAL_AUDPRC_AbortCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(haprc);
    UNUSED(cid);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_PDM_AbortCpltCallback should be implemented in the user file
     */
}


static void AUDPRC_DMATxCplt(DMA_HandleTypeDef *hdma)
{
    AUDPRC_HandleTypeDef *aprc = (AUDPRC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    int i;
    for (i = 0; i < HAL_AUDPRC_RX_CH0; i++)
        if (aprc->hdma[i] == hdma)
            break;
    if (i >= HAL_AUDPRC_RX_CH0) // something wrong
        return;
#if 0
    if (HAL_IS_BIT_CLR(hdma->Instance->CCR, DMA_CCR_CIRC))
    {
        uint32_t *maskreg;
        if (i < HAL_AUDPRC_RX_CH0)
        {
            maskreg = (uint32_t *)((uint32_t)&aprc->Instance->TX_CH0_CFG + 8 * i);
            /* Disable Tx DMA Request */
            SET_BIT(*maskreg, AUDPRC_TX_CH0_CFG_DMA_MSK);
        }

        aprc->State &= ~HAL_AUDPRC_STATE_BUSY_TX;
    }
#endif
    HAL_AUDPRC_TxCpltCallback(aprc, i);
}

/**
  * @brief DMA I2S transmit process half complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void AUDPRC_DMATxHalfCplt(DMA_HandleTypeDef *hdma)
{
    AUDPRC_HandleTypeDef *aprc = (AUDPRC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    int i;
    for (i = 0; i < HAL_AUDPRC_RX_CH0; i++)
        if (aprc->hdma[i] == hdma)
            break;
    if (i >= HAL_AUDPRC_RX_CH0) // something wrong
        return;

    HAL_AUDPRC_TxHalfCpltCallback(aprc, i);
}

static void AUDPRC_DMARxCplt(DMA_HandleTypeDef *hdma)
{
    AUDPRC_HandleTypeDef *aprc = (AUDPRC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    int i;
    for (i = HAL_AUDPRC_RX_CH0; i < HAL_AUDPRC_INSTANC_CNT; i++)
        if (aprc->hdma[i] == hdma)
            break;
    if (i >= HAL_AUDPRC_INSTANC_CNT)
        return;

    /* DMA Normal Mode */
#if 0
    if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
    {
        uint32_t *maskreg;

        if (i < HAL_AUDPRC_INSTANC_CNT)
        {
            maskreg = (uint32_t *)((uint32_t)&aprc->Instance->TX_CH0_CFG + 8 * i);
            /* Disable Tx DMA Request */
            SET_BIT(*maskreg, AUDPRC_TX_CH0_CFG_DMA_MSK);
        }
    }
#endif
    HAL_AUDPRC_RxCpltCallback(aprc, i);

}

/**
  * @brief  DMA PDM half receive process complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void AUDPRC_DMAHalfRxCplt(DMA_HandleTypeDef *hdma)
{
    AUDPRC_HandleTypeDef *aprc = (AUDPRC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    int i;
    for (i = HAL_AUDPRC_RX_CH0; i < HAL_AUDPRC_INSTANC_CNT; i++)
        if (aprc->hdma[i] == hdma)
            break;
    if (i >= HAL_AUDPRC_INSTANC_CNT)
        return;
    HAL_AUDPRC_RxHalfCpltCallback(aprc, i);
}


/**
  * @brief  DMA PDM communication error callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void AUDPRC_DMAError(DMA_HandleTypeDef *hdma)
{
    AUDPRC_HandleTypeDef *aprc = (AUDPRC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    int i;
    for (i = 0; i < HAL_AUDPRC_INSTANC_CNT; i++)
        if (aprc->hdma[i] == hdma)
            break;
    if (i >= HAL_AUDPRC_INSTANC_CNT)
        return;

    aprc->State[i] = HAL_AUDPRC_STATE_READY;
    HAL_AUDPRC_ErrorCallback(aprc, i);
}

#endif /* HAL_AUDPRC_MODULE_ENABLED */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/


