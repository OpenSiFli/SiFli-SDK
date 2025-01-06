/**
  ******************************************************************************
  * @file   bf0_hal_sdadc.c
  * @author Sifli software development team
  * @brief   This file provides firmware functions to manage the following
  *          functionalities of the Analog to Digital Convertor (ADC)
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

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup ADC Analog Digital Converter
  * @brief ADC HAL module driver
  * @{
  */

#ifdef HAL_SDADC_MODULE_ENABLED



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup ADC_Private_Constants ADC Private Constants
  * @{
  */


/**
    * @}
    */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup ADC_Private_Functions ADC Private Functions
  * @{
  */

static void SDADC_DMAConvCplt(DMA_HandleTypeDef *hdma);
static void SDADC_DMAError(DMA_HandleTypeDef *hdma);

/**
    * @}
    */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup SDADC_Exported_Functions SDADC Exported Functions
  * @{
  */

/** @defgroup ADC_Exported_Functions_Group1 Initialization/de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the SDADC.
      (+) De-initialize the SDADC
@endverbatim
  * @{
  */

/**
  * @brief  Initializes the SDADC peripheral and regular group according to
  *         parameters specified in structure "ADC_InitTypeDef".
  * @note   As prerequisite, ADC clock must be configured at RCC top level
  *         depending on both possible clock sources: APB clock of HSI clock.
  * @note   Possibility to update parameters on the fly:
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_InitTypeDef".
  * @note   This function configures the SDADC within 2 scopes: scope of entire
  *         ADC and scope of regular group. For parameters details, see comments
  *         of structure "SDADC_InitTypeDef".
  * @param  hadc ADC handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_Init(SDADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;
    uint32_t i, tmp_val;

    /* Check ADC handle */
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }

    /* Actions performed only if SDADC is coming from state reset:                */
    /* - SDADC voltage regulator enable                                           */
    if (hadc->State == HAL_SDADC_STATE_RESET)
    {
        /* Initialize ADC error code */
        SDADC_CLEAR_ERRORCODE(hadc);

        /* Allocate lock resource and initialize it */
        hadc->Lock = HAL_UNLOCKED;
    }

    /* Configuration of ADC parameters if previous preliminary actions are      */
    /* correctly completed.                                                     */
    /* and if there is no conversion on going on regular group (ADC can be      */
    /* enabled anyway, in case of call of this function to update a parameter   */
    /* on the fly).                                                             */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_SDADC_STATE_ERROR_INTERNAL) &&
            (tmp_hal_status == HAL_OK))
    {
        /* Set ADC state */
        SDADC_STATE_CLR_SET(hadc->State,
                            HAL_SDADC_STATE_REG_BUSY,
                            HAL_SDADC_STATE_BUSY_INTERNAL);

        // enable SDADCK CLK
        hwp_pmuc->HXT_CR2 |= PMUC_HXT_CR2_SDADC_CLKIN_EN;

        /* Init CFG0 ~ CFG3, set some bits to fix value after analog confirm */
        //hadc->Instance->CFG0 = 0;
        tmp_val = hadc->Instance->CFG0;
        tmp_val &= (~SDADC_CFG0_FCK_SEL_LV); // set FCK SEL TO 0
        hadc->Instance->CFG0 = tmp_val;
        //hadc->Instance->CFG1 = 0;
        tmp_val = hadc->Instance->CFG1;
        tmp_val &= (~SDADC_CFG1_FCHOP_SIG_SEL_LV);
        tmp_val |= (2 << SDADC_CFG1_FCHOP_SIG_SEL_LV_Pos);
        hadc->Instance->CFG1 = tmp_val;
        //hadc->Instance->CFG2 = 0;
        tmp_val = hadc->Instance->CFG2;
        tmp_val &= (~SDADC_CFG2_FCHOP_REF_SEL_LV);
        tmp_val |= (2 << SDADC_CFG2_FCHOP_REF_SEL_LV_Pos);
        hadc->Instance->CFG2 = tmp_val;
        //hadc->Instance->CFG3 = 0;
        tmp_val = hadc->Instance->CFG3;
        tmp_val &= (~(SDADC_CFG3_REFBUF_CHOP_MX_LV_Msk | SDADC_CFG3_AMP1_BM_LV_Msk
                      | SDADC_CFG3_REFBUF_BM_LV_Msk | SDADC_CFG3_AMP2_BM_LV_Msk));
        tmp_val |= ((0 << SDADC_CFG3_REFBUF_CHOP_MX_LV_Pos) | (5 << SDADC_CFG3_AMP1_BM_LV_Pos)
                    | (5 << SDADC_CFG3_REFBUF_BM_LV_Pos) | (5 << SDADC_CFG3_AMP2_BM_LV_Pos));
        hadc->Instance->CFG3 = tmp_val;

        tmp_val = hadc->Instance->RSVD;
        tmp_val &= (~SDADC_RSVD_RESERVE1_Msk);
        tmp_val |= (1 << SDADC_RSVD_RESERVE1_Pos);
        hadc->Instance->RSVD = tmp_val;

        /* Set continue mode, dsample, run and pu */
        // PU for start, PPU FOR power, need wait 2 s after PPU set to make sure reference stable
        tmp_val = hadc->Instance->CFG0;
        tmp_val &= ~(SDADC_CFG0_CONTIN_LV | SDADC_CFG0_DSAMPLE_MODE | SDADC_CFG0_FCK_SEL_LV); // set FCK SEL TO 0
        tmp_val |= ((hadc->Init.conti_mode << SDADC_CFG0_CONTIN_LV_Pos) | (hadc->Init.dsample_mode << SDADC_CFG0_DSAMPLE_MODE_Pos)
                    | SDADC_CFG0_RUN_LV | SDADC_CFG0_PPU_LV);    // move SDADC_CFG0_PU_LV to start
        hadc->Instance->CFG0 = tmp_val;

        /* Set diff or single */
        tmp_val = hadc->Instance->CFG2;
        tmp_val &= ~(SDADC_CFG2_SE_DIFF_SEL_LV);
        tmp_val |= (hadc->Init.diff_sel << SDADC_CFG2_SE_DIFF_SEL_LV_Pos);
        hadc->Instance->CFG2 = tmp_val;

        /* Set vref sel */
        tmp_val = hadc->Instance->CFG3;
        tmp_val &= ~(SDADC_CFG3_VREF_SEL_LV);
        tmp_val |= (hadc->Init.vref_sel << SDADC_CFG3_VREF_SEL_LV_Pos);
        hadc->Instance->CFG3 = tmp_val;

        /* Set default gain*/
        tmp_val = hadc->Instance->CFG2;
        tmp_val &= ~(SDADC_CFG2_GAIN_NUME_LV | SDADC_CFG2_GAIN_DENO_LV);
        tmp_val |= (4 << SDADC_CFG2_GAIN_DENO_LV_Pos) | (4 << SDADC_CFG2_GAIN_NUME_LV_Pos);
        hadc->Instance->CFG2 = tmp_val;

        /* Set default accurate */
        tmp_val = hadc->Instance->CFG1;
        tmp_val &= ~(SDADC_CFG1_CHOP1_NUM_LV | SDADC_CFG1_CHOP2_NUM_LV | SDADC_CFG1_CHOP3_NUM_LV);
        tmp_val |= (0x9c << SDADC_CFG1_CHOP1_NUM_LV_Pos)
                   | (0xc9 << SDADC_CFG1_CHOP2_NUM_LV_Pos)
                   | (0x1ff << SDADC_CFG1_CHOP3_NUM_LV_Pos);
        hadc->Instance->CFG1 = tmp_val;

        tmp_val = hadc->Instance->CFG2;
        tmp_val &= ~(SDADC_CFG2_SAMPLE_NUM_LV | SDADC_CFG2_CHOP_REF_NUM_LV);
        tmp_val |= (0x9c << SDADC_CFG2_CHOP_REF_NUM_LV_Pos)
                   | (0xe0 << SDADC_CFG2_SAMPLE_NUM_LV_Pos);
        hadc->Instance->CFG2 = tmp_val;

        SDADC_Enable(hadc);

        /* Disable all slot at initial */
        for (i = 0; i < 5; i++)
            HAL_SDADC_EnableSlot(hadc, i, 0);
    }
    else
    {
        /* Update ADC state machine to error */
        SET_BIT(hadc->State, HAL_SDADC_STATE_ERROR_INTERNAL);

        tmp_hal_status = HAL_ERROR;
    }

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Deinitialize the SDADC peripheral registers to their default reset
  *         values, with deinitialization of the ADC MSP.
  * @note   For devices with several ADCs: reset of ADC common registers is done
  *         only if all ADCs sharing the same common group are disabled.
  *         If this is not the case, reset of these common parameters reset is
  *         bypassed without error reporting: it can be the intended behaviour in
  *         case of reset of a single ADC while the other ADCs sharing the same
  *         common group is still running.
  * @param  hadc SDADC handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_DeInit(SDADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Check ADC handle */
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }

    /* Set ADC state */
    SET_BIT(hadc->State, HAL_SDADC_STATE_BUSY_INTERNAL);

    /* Disable ADC peripheral if conversions are effectively stopped */
    if (tmp_hal_status == HAL_OK)
    {
        /* Disable the ADC peripheral */
        tmp_hal_status = SDADC_Disable(hadc);

        /* Check if ADC is effectively disabled */
        if (tmp_hal_status != HAL_ERROR)
        {
            /* Change ADC state */
            hadc->State = HAL_SDADC_STATE_READY;
        }
    }


    /* Configuration of ADC parameters if previous preliminary actions are      */
    /* correctly completed.                                                     */
    if (tmp_hal_status != HAL_ERROR)
    {
        /* Set ADC error code to none */
        SDADC_CLEAR_ERRORCODE(hadc);

        /* Set ADC state */
        hadc->State = HAL_SDADC_STATE_RESET;
    }

    // Disable SDADC CLK
    hwp_pmuc->HXT_CR2 &= (~PMUC_HXT_CR2_SDADC_CLKIN_EN);

    /* Process unlocked */
    __HAL_UNLOCK(hadc);

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Initialize the SDMADC MSP.
  * @param  hcomp  SDMADC handle
  * @retval hadc
  */
__weak void HAL_SDADC_MspInit(SDADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_SDADC_MspInit could be implemented in the user file
     */
    // set adc input pin, it's an independent pad for analog to avoid digital interference

    // set sdmadc pin, they are analog pin, no function in list, set to GPIO and set mode to input
    HAL_PIN_Set(PAD_PB23, GPIO_B23, PIN_NOPULL, 0);  //##SDADC_CH0
    HAL_PIN_Set(PAD_PB24, GPIO_B24, PIN_NOPULL, 0);  //##SDADC_CH1
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_NOPULL, 0);  //##SDADC_CH2
    HAL_PIN_Set(PAD_PB26, GPIO_B26, PIN_NOPULL, 0);  //##SDADC_CH3

#if 1
    GPIO_InitStruct.Pin = 23;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init((GPIO_TypeDef *)hwp_gpio2, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = 24;
    HAL_GPIO_Init((GPIO_TypeDef *)hwp_gpio2, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = 25;
    HAL_GPIO_Init((GPIO_TypeDef *)hwp_gpio2, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = 26;
    HAL_GPIO_Init((GPIO_TypeDef *)hwp_gpio2, &GPIO_InitStruct);

#endif
}

/**
  * @brief  DeInitialize the SDMADC MSP.
  * @param  hadc  SDMADC handle
  * @retval None
  */
__weak void HAL_SDADC_MspDeInit(SDADC_HandleTypeDef *hadc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hadc);

}


/**
  * @}
  */

/** @defgroup SDADC_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Start conversion of regular group.
      (+) Stop conversion of regular group.
      (+) Poll for conversion complete on regular group.
      (+) Poll for conversion event.
      (+) Get result of regular channel conversion.
      (+) Start conversion of regular group and enable interruptions.
      (+) Stop conversion of regular group and disable interruptions.
      (+) Handle SDADC interrupt request
      (+) Start conversion of regular group and enable DMA transfer.
      (+) Stop conversion of regular group and disable ADC DMA transfer.
@endverbatim
  * @{
  */

/**
  * @brief  Enables SDADC, starts conversion of regular group.
  *         Interruptions enabled in this function: None.
  * @param  hadc SDADC handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_Start(SDADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Perform ADC enable and conversion start if no conversion is on going */
    if (1)
    {
        // TODO, how to make sure power for analog opened
        //hwp_tsen->ANAU_ANA_TP |= TSEN_ANAU_ANA_TP_ANAU_IARY_EN;
        /* Process locked */
        __HAL_LOCK(hadc);

        /* Start conversion if ADC is effectively enabled */
        if (tmp_hal_status == HAL_OK)
        {
            /* Set ADC state                                                        */
            /* - Clear state bitfield related to regular group conversion results   */
            /* - Set state bitfield related to regular operation                    */
            SDADC_STATE_CLR_SET(hadc->State,
                                HAL_SDADC_STATE_READY | HAL_SDADC_STATE_REG_EOC | HAL_SDADC_STATE_REG_OVR,
                                HAL_SDADC_STATE_REG_BUSY);

            /* Reset ADC all error code fields */
            SDADC_CLEAR_ERRORCODE(hadc);

            /* Process unlocked */
            /* Unlock before starting ADC conversions: in case of potential         */
            /* interruption, to let the process to ADC IRQ Handler.                 */
            __HAL_UNLOCK(hadc);

            /* Enable analog module */
            SDADC_Enable(hadc);

            /* Enable INT */
            //hadc->Instance->INT_MSK &= ~(SDADC_INT_MSK_OVERFLOW|SDADC_INT_MSK_DSAMPLE);
            __HAL_SDADC_ENABLE_IRQ(hadc, SDADC_INT_MSK_OVERFLOW | SDADC_INT_MSK_DSAMPLE);

            // TODO, how to config timer/gpio/software triger?   */
            //hadc->Instance->TRIG |= SDADC_TRIG_ADC_START;
            switch (hadc->Init.src_sel)
            {
            case HAL_SDADC_SRC_SW:
                hadc->Instance->TRIG |= SDADC_TRIG_ADC_START;
                break;
            case HAL_SDADC_SRC_GPIO:
                hadc->Instance->TRIG |= SDADC_TRIG_GPIO_TRIG_EN;
                break;
            case HAL_SDADC_SRC_TIMER:
                hadc->Instance->TRIG |= SDADC_TRIG_TIMER_TRIG_EN;
                break;
            default:
                hadc->Instance->TRIG |= SDADC_TRIG_ADC_START;
                break;
            }
        }
    }
    else
    {
        tmp_hal_status = HAL_BUSY;
    }

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Stop ADC conversion of regular group, disable ADC peripheral.
  * @param  hadc ADC handle
  * @retval HAL status.
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_Stop(SDADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hadc);

    hadc->Instance->TRIG &= ~(SDADC_TRIG_ADC_START
                              | SDADC_TRIG_GPIO_TRIG_EN | SDADC_TRIG_TIMER_TRIG_EN);

    //if (hadc->Init.conti_mode != 0)
    //    SDADC_Disable(hadc);

    /* Set ADC state */
    SDADC_STATE_CLR_SET(hadc->State,
                        HAL_SDADC_STATE_REG_BUSY,
                        HAL_SDADC_STATE_READY);
    /* Process unlocked */
    __HAL_UNLOCK(hadc);

    // close power for analog
    //hwp_tsen->ANAU_ANA_TP &= (~TSEN_ANAU_ANA_TP_ANAU_IARY_EN);

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Wait for regular group conversion to be completed.
  * @param  hadc SDADC handle
  * @param  Timeout Timeout value in millisecond.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_PollForConversion(SDADC_HandleTypeDef *hadc, uint32_t Timeout)
{
    uint32_t tickstart;

    /* Check SDADC handle */
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }

    /* Enable analog module */
    //SDADC_Enable(hadc);

    /* Enable INT */
    //hadc->Instance->INT_MSK &= ~(SDADC_INT_MSK_DSAMPLE);
    __HAL_SDADC_ENABLE_IRQ(hadc, SDADC_INT_MSK_DSAMPLE);

    // open power for analog
    //hwp_tsen->ANAU_ANA_TP |= TSEN_ANAU_ANA_TP_ANAU_IARY_EN;

    hadc->Instance->TRIG |= SDADC_TRIG_ADC_START;

    /* Get tick count */
    tickstart = HAL_GetTick();

    /* Wait until End of Conversion flag is raised */
    //while (HAL_IS_BIT_CLR(hadc->Instance->INT_ST, SDADC_INT_ST_DSAMPLE))
    while (HAL_IS_BIT_CLR(hadc->Instance->CFG0, SDADC_CFG0_SDADC_DATA_RDY))
    {
        /* Check if timeout is disabled (set to infinite wait) */
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0) || ((HAL_GetTick() - tickstart) > Timeout))
            {
                /* Update ADC state machine to timeout */
                SET_BIT(hadc->State, HAL_SDADC_STATE_TIMEOUT);

                /* Process unlocked */
                __HAL_UNLOCK(hadc);

                return HAL_TIMEOUT;
            }
        }
    }

    //hadc->Instance->TRIG &= ~SDADC_TRIG_ADC_START;

    /* Clear ISR */
    //__HAL_SDADC_CLEAR_FLAG(hadc, SDADC_INT_ST_DSAMPLE);

    /* Update ADC state machine */
    SET_BIT(hadc->State, HAL_SDADC_STATE_REG_EOC);

    // close power for analog
    //hwp_tsen->ANAU_ANA_TP &= (~TSEN_ANAU_ANA_TP_ANAU_IARY_EN);
    //SDADC_Disable(hadc);

    /* Return ADC state */
    return HAL_OK;
}

/**
  * @brief  Enables SDADC, starts conversion of regular group with interruption.
  *         Interruptions enabled in this function:
  *          - EOC (end of conversion of regular group) or EOS (end of
  *            sequence of regular group) depending on ADC initialization
  *            parameter "EOCSelection"
  *          - overrun (if available)
  *         Each of these interruptions has its dedicated callback function.
  * @param  hadc SDADC handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_Start_IT(SDADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Check ADC handle */
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }
    // open power for analog
    //hwp_tsen->ANAU_ANA_TP |= TSEN_ANAU_ANA_TP_ANAU_IARY_EN;
    SDADC_Enable(hadc);

    __HAL_SDADC_ENABLE_IRQ(hadc, SDADC_INT_ST_DSAMPLE);

    if (hadc->Init.src_sel == HAL_SDADC_SRC_SW)
        __HAL_SDADC_START_SW_CONV(hadc);
    else // GPIO NOT SUPPORT, SO ONLY SW AND TIMER
        __HAL_SDADC_START_TIMER_CONV(hadc);

    /* Return function status */
    return tmp_hal_status;
}


/**
  * @brief  Stop SDADC conversion of regular group, disable interruption of
  *         end-of-conversion, disable ADC peripheral.
  * @param  hadc SDADC handle
  * @retval HAL status.
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_Stop_IT(SDADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hadc);

    //if (hadc->Init.conti_mode != 0)
    //    __HAL_ADC_STOP_CONV(hadc);

    __HAL_SDADC_DISABLE_IRQ(hadc, SDADC_INT_MSK_OVERFLOW | SDADC_INT_MSK_OVERFLOW);

    hadc->Instance->TRIG &= ~(SDADC_TRIG_ADC_START
                              | SDADC_TRIG_GPIO_TRIG_EN | SDADC_TRIG_TIMER_TRIG_EN);
    SDADC_Disable(hadc);

    /* Process unlocked */
    __HAL_UNLOCK(hadc);
    // close power for analog
    //hwp_tsen->ANAU_ANA_TP &= (~TSEN_ANAU_ANA_TP_ANAU_IARY_EN);

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Enables SDADC, starts conversion of regular group and transfers result
  *         through DMA.
  *         Interruptions enabled in this function:
  *          - DMA transfer complete
  *          - DMA half transfer
  *          - overrun
  *         Each of these interruptions has its dedicated callback function.
  * @param  hadc SDADC handle
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from ADC peripheral to memory.
  * @retval None
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_Start_DMA(SDADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* CheckSD ADC handle */
    if (hadc == NULL || hadc->DMA_Handle == NULL)
    {
        return HAL_ERROR;
    }
    if (hadc->Init.conti_mode == 0) // only continue mode use dma
        return HAL_ERROR;

    // open power for analog
    //hwp_tsen->ANAU_ANA_TP |= TSEN_ANAU_ANA_TP_ANAU_IARY_EN;
#if 0   // DMA Handle init should set in driver but not HAL
    /* Init DMA configure*/
    hadc->DMA_Handle->Instance                 = SDADC_DMA_INSTANCE;
    hadc->DMA_Handle->Init.Request             = SDADC_DMA_REQUEST;
#endif

    hadc->DMA_Handle->Init.Direction =  DMA_PERIPH_TO_MEMORY;
    hadc->DMA_Handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hadc->DMA_Handle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hadc->DMA_Handle->Init.PeriphInc           = DMA_PINC_DISABLE;
    hadc->DMA_Handle->Init.MemInc              = DMA_MINC_ENABLE;
    hadc->DMA_Handle->Init.Mode                = DMA_NORMAL;
    hadc->DMA_Handle->Init.Priority            = DMA_PRIORITY_MEDIUM;
    hadc->DMA_Handle->XferCpltCallback = SDADC_DMAConvCplt;
    hadc->DMA_Handle->XferErrorCallback = SDADC_DMAError;


    tmp_hal_status = HAL_DMA_Init(hadc->DMA_Handle);
    if (tmp_hal_status != HAL_OK)
        return tmp_hal_status;

    SDADC_DMA_ENABLE(hadc);
    SDADC_Enable(hadc);
    tmp_hal_status = HAL_DMA_Start(hadc->DMA_Handle, (uint32_t)(&hadc->Instance->ADC_DATA), (uint32_t)pData, Length);

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Stop SDADC conversion of regular group, disable ADC DMA transfer, disable
  *         ADC peripheral.
  *         Each of these interruptions has its dedicated callback function.
  * @param  hadc SDADC handle
  * @retval HAL status.
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_Stop_DMA(SDADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    if (hadc->Init.conti_mode == 0) // only continue mode use dma
        return HAL_ERROR;

    /* Process locked */
    __HAL_LOCK(hadc);

    SDADC_DMA_DISABLE(hadc);

    tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

    SDADC_Disable(hadc);

    /* Process unlocked */
    __HAL_UNLOCK(hadc);
    // close power for analog
    //hwp_tsen->ANAU_ANA_TP &= (~TSEN_ANAU_ANA_TP_ANAU_IARY_EN);

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Get SDADC regular group conversion result.
  * @note   Reading register DR automatically clears ADC flag EOC
  *         (ADC group regular end of unitary conversion).
  * @note   This function does not clear ADC flag EOS
  *         (ADC group regular end of sequence conversion).
  *         Occurrence of flag EOS rising:
  *          - If sequencer is composed of 1 rank, flag EOS is equivalent
  *            to flag EOC.
  *          - If sequencer is composed of several ranks, during the scan
  *            sequence flag EOC only is raised, at the end of the scan sequence
  *            both flags EOC and EOS are raised.
  *         To clear this flag, either use function:
  *         in programming model IT: @ref HAL_ADC_IRQHandler(), in programming
  *         model polling: @ref HAL_ADC_PollForConversion()
  *         or @ref __HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_EOS).
  * @param  hadc SDADC handle
  * @retval ADC group regular conversion data
  */
__HAL_ROM_USED uint32_t HAL_SDADC_GetValue(SDADC_HandleTypeDef *hadc, uint32_t slot)
{
    uint32_t value;
    __IO uint32_t *p = &(hadc->Instance->CH0_DOUT);

    /* Check SDADC handle */
    if (hadc == NULL || slot >= 5)
    {
        return HAL_ERROR;
    }

    /* For continue mode, it use DMA, can not get data here */
    if (hadc->Init.conti_mode == 1)
    {
        hadc->ErrorCode = HAL_SDADC_ERROR_INTERNAL;
        return 0;
    }
    if (hadc->Init.dsample_mode == 0) // use single mode, single channel
    {
        p =  &(hadc->Instance->SINGLE_DOUT);
    }
    else
    {
        p += slot;
    }

    value = *p & 0XFFFFFF;

    /* Return ADC converted value */
    return value;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_SetSource(SDADC_HandleTypeDef *hadc, HAL_SDADC_SRC__T src)
{
    if (hadc == NULL || src > HAL_SDADC_SRC_TIMER)
    {
        return HAL_ERROR;
    }

    // it used for start triger mode
    hadc->Init.src_sel = src;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_SetTimer(SDADC_HandleTypeDef *hadc, HAL_SDADC_SRC_TIME_T src)
{
    if (hadc == NULL || src >= HAL_SDADC_SRC_TIMER_CNT)
    {
        return HAL_ERROR;
    }

    hadc->Instance->TRIG |= (src << SDADC_TRIG_TIMER_TRIG_SRC_SEL_Pos);

    return HAL_OK;
}


/**
  * @brief  Handles SDADC interrupt request.
  * @param  hadc SDADC handle
  * @retval None
  */
__HAL_ROM_USED void HAL_SDADC_IRQHandler(SDADC_HandleTypeDef *hadc)
{
    /* Clear ISR */
    __HAL_SDADC_CLEAR_FLAG(hadc, SDADC_INT_CLR_INT_CLR);

    /* Set status */
    SET_BIT(hadc->State, HAL_SDADC_START_IRQ_DONE);
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  hadc ADC handle
  * @retval None
  */
__weak void HAL_SDADC_ConvCpltCallback(SDADC_HandleTypeDef *hadc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hadc);

    /* NOTE : This function should not be modified. When the callback is needed,
              function HAL_ADC_ConvCpltCallback must be implemented in the user file.
     */
}


/**
  * @}
  */

/** @defgroup ADC_Exported_Functions_Group3 Peripheral Control functions
 *  @brief    Peripheral Control functions
 *
@verbatim
 ===============================================================================
             ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure channels on regular group
      (+) Configure the analog watchdog

@endverbatim
  * @{
  */

/**
  * @brief  Configures the the selected channel to be linked to the regular
  *         group.
  * @note   In case of usage of internal measurement channels:
  *         VrefInt/Vbat/TempSensor.
  *         Sampling time constraints must be respected (sampling time can be
  *         adjusted in function of ADC clock frequency and sampling time
  *         setting).
  *         Refer to device datasheet for timings values, parameters TS_vrefint,
  *         TS_vbat, TS_temp (values rough order: 5us to 17us).
  *         These internal paths can be be disabled using function
  *         HAL_SDADC_DeInit().
  * @note   Possibility to update parameters on the fly:
  *         This function initializes channel into regular group, following
  *         calls to this function can be used to reconfigure some parameters
  *         of structure "SDADC_ChannelConfTypeDef" on the fly, without reseting
  *         the SDADC.
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_ChannelConfTypeDef".
  * @param  hadc ADC handle
  * @param  sConfig Structure of ADC channel for regular group.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_ConfigChannel(SDADC_HandleTypeDef *hadc, SDADC_ChannelConfTypeDef *sConfig)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;
    uint32_t value = 0;
    __IO uint32_t *p = &(hadc->Instance->CH0_CFG);

    /* Check ADC handle */
    if (hadc == NULL || sConfig == NULL || sConfig->Channel >= 5)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hadc);

    if (hadc->Init.dsample_mode == 0) // single mode and continue mode, use ana register
    {
        value = hadc->Instance->CFG3;
        value &= ~(SDADC_CFG3_SEL_PCH_LV | SDADC_CFG3_SEL_NCH_LV);
        value |= ((sConfig->nchnl_sel << SDADC_CFG3_SEL_NCH_LV_Pos) | (sConfig->pchnl_sel << SDADC_CFG3_SEL_PCH_LV_Pos));
        hadc->Instance->CFG3 = value;
    }
    else
    {
        p += sConfig->Channel;

        value = ((sConfig->shift_num << SDADC_CH0_CFG_SHIFT_NUM_Pos) & SDADC_CH0_CFG_SHIFT_NUM)
                | ((sConfig->nchnl_sel << SDADC_CH0_CFG_SEL_NCH_Pos) & SDADC_CH0_CFG_SEL_NCH)
                | ((sConfig->pchnl_sel << SDADC_CH0_CFG_SEL_PCH_Pos) & SDADC_CH0_CFG_SEL_PCH)
                | ((sConfig->slot_en << SDADC_CH0_CFG_EN_Pos) & SDADC_CH0_CFG_EN);

        *p = value;
        if (sConfig->slot_en)
            hadc->Init.en_slot |= (1 << sConfig->Channel);
        else
            hadc->Init.en_slot &= ~(1 << sConfig->Channel);
    }
    /* Process unlocked */
    __HAL_UNLOCK(hadc);

    /* Return function status */
    return tmp_hal_status;
}

__HAL_ROM_USED HAL_StatusTypeDef       HAL_SDADC_ConfigAccu(SDADC_HandleTypeDef *hadc, SDADC_AccurateConfTypeDef *sConfig)
{
    uint32_t value = 0;

    /* Check ADC handle */
    if (hadc == NULL || sConfig == NULL)
    {
        return HAL_ERROR;
    }

    value = hadc->Instance->CFG1;
    value &= ~(SDADC_CFG1_CHOP1_NUM_LV | SDADC_CFG1_CHOP2_NUM_LV | SDADC_CFG1_CHOP3_NUM_LV);
    value |= ((sConfig->chop1_num & 0x1ff) << SDADC_CFG1_CHOP1_NUM_LV_Pos)
             | ((sConfig->chop2_num & 0x1ff) << SDADC_CFG1_CHOP2_NUM_LV_Pos)
             | ((sConfig->chop3_num & 0x1ff) << SDADC_CFG1_CHOP3_NUM_LV_Pos);
    hadc->Instance->CFG1 = value;

    value = hadc->Instance->CFG2;
    value &= ~(SDADC_CFG2_SAMPLE_NUM_LV | SDADC_CFG2_CHOP_REF_NUM_LV);
    value |= ((sConfig->chop_ref_num & 0x1ff) << SDADC_CFG2_CHOP_REF_NUM_LV_Pos)
             | ((sConfig->sample_num & 0x1ff) << SDADC_CFG2_SAMPLE_NUM_LV_Pos);
    hadc->Instance->CFG2 = value;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef       HAL_SDADC_ConfigGain(SDADC_HandleTypeDef *hadc, SDADC_GainConfTypeDef *sConfig)
{
    uint32_t value = 0;

    /* Check ADC handle */
    if (hadc == NULL || sConfig == NULL)
    {
        return HAL_ERROR;
    }

    value = hadc->Instance->CFG2;
    value &= ~(SDADC_CFG2_GAIN_NUME_LV | SDADC_CFG2_GAIN_DENO_LV);
    value |= ((sConfig->gain_deno & 7) << SDADC_CFG2_GAIN_DENO_LV_Pos) | ((sConfig->gain_nume & 7) << SDADC_CFG2_GAIN_NUME_LV_Pos);
    hadc->Instance->CFG2 = value;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_SDADC_EnableSlot(SDADC_HandleTypeDef *hadc, uint32_t slot, uint8_t en)
{
    __IO uint32_t *p = &(hadc->Instance->CH0_CFG);

    /* Check SDADC handle */
    if (hadc == NULL || slot >= 5)
    {
        return HAL_ERROR;
    }

    if (hadc->Init.dsample_mode == 0)
        return HAL_OK;

    p += slot;

    // Clear these bit, disable
    *p &= (~SDADC_CH0_CFG_EN);

    if (en != 0)
    {
        *p |= SDADC_CH0_CFG_EN;
        hadc->Init.en_slot |= (1 << slot);
    }
    else
    {
        hadc->Init.en_slot &= ~(1 << slot);
    }

    return HAL_OK;
}

/**
  * @}
  */


/** @defgroupSD ADC_Exported_Functions_Group4 Peripheral State functions
 *  @brief    Peripheral State functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions to get in run-time the status of the
    peripheral.
      (+) Check the SDADC state
      (+) Check the SDADC error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the SDADC state
  * @note   ADC state machine is managed by bitfields, ADC status must be
  *         compared with states bits.
  *         For example:
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_REG_BUSY)) "
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_AWD1)    ) "
  * @param  hadc ADC handle
  * @retval HAL state
  */
__HAL_ROM_USED uint32_t HAL_SDADC_GetState(SDADC_HandleTypeDef *hadc)
{
    /* Return ADC state */
    return hadc->State;
}

/**
  * @brief  Return the ADC error code
  * @param  hadc ADC handle
  * @retval ADC Error Code
  */
__HAL_ROM_USED uint32_t HAL_SDADC_GetError(SDADC_HandleTypeDef *hadc)
{
    return hadc->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup ADC_Private_Functions ADC Private Functions
  * @{
  */


/**
  * @brief  DMA transfer complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void SDADC_DMAConvCplt(DMA_HandleTypeDef *hdma)
{
    /* Retrieve ADC handle corresponding to current DMA handle */
    SDADC_HandleTypeDef *hadc = (SDADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Update state machine on conversion status if not in error state */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_SDADC_STATE_ERROR_INTERNAL | HAL_SDADC_STATE_ERROR_DMA))
    {
        /* Set ADC state */
        SET_BIT(hadc->State, HAL_SDADC_STATE_REG_EOC);

        /* Conversion complete callback */
        HAL_SDADC_ConvCpltCallback(hadc);
    }
    else
    {
        /* Call DMA error callback */
        hadc->DMA_Handle->XferErrorCallback(hdma);
    }

}

/**
  * @brief  DMA error callback
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void SDADC_DMAError(DMA_HandleTypeDef *hdma)
{
    /* Retrieve ADC handle corresponding to current DMA handle */
    SDADC_HandleTypeDef *hadc = (SDADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Set ADC state */
    SET_BIT(hadc->State, HAL_SDADC_STATE_ERROR_DMA);

    /* Set ADC error code to DMA error */
    SET_BIT(hadc->ErrorCode, HAL_SDADC_ERROR_DMA);

}

/**
  * @}
  */

#endif /* HAL_SDADC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
