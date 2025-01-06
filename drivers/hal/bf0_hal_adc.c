/**
  ******************************************************************************
  * @file   bf0_hal_adc.c
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

#ifdef HAL_ADC_MODULE_ENABLED



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


static void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma);
static void ADC_DMAError(DMA_HandleTypeDef *hdma);

/**
    * @}
    */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup ADC_Exported_Functions ADC Exported Functions
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
      (+) Initialize and configure the ADC.
      (+) De-initialize the ADC
@endverbatim
  * @{
  */

/**
  * @brief  Initializes the ADC peripheral and regular group according to
  *         parameters specified in structure "ADC_InitTypeDef".
  * @note   As prerequisite, ADC clock must be configured at RCC top level
  *         depending on both possible clock sources: APB clock of HSI clock.
  * @note   Possibility to update parameters on the fly:
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_InitTypeDef".
  * @note   This function configures the ADC within 2 scopes: scope of entire
  *         ADC and scope of regular group. For parameters details, see comments
  *         of structure "ADC_InitTypeDef".
  * @param  hadc ADC handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;
    uint32_t i;

    /* Check ADC handle */
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }

    /* Actions performed only if ADC is coming from state reset:                */
    /* - ADC voltage regulator enable                                           */
    if (hadc->State == HAL_ADC_STATE_RESET)
    {
        /* Initialize ADC error code */
        ADC_CLEAR_ERRORCODE(hadc);

        /* Allocate lock resource and initialize it */
        hadc->Lock = HAL_UNLOCKED;
    }

    /* Initial ADC Msp */
    HAL_ADC_MspInit(hadc);

    /* Configuration of ADC parameters if previous preliminary actions are      */
    /* correctly completed.                                                     */
    /* and if there is no conversion on going on regular group (ADC can be      */
    /* enabled anyway, in case of call of this function to update a parameter   */
    /* on the fly).                                                             */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL) &&
            (tmp_hal_status == HAL_OK))
    {
        /* Set ADC state */
        ADC_STATE_CLR_SET(hadc->State,
                          HAL_ADC_STATE_REG_BUSY,
                          HAL_ADC_STATE_BUSY_INTERNAL);

        /* TODO , Init ADC configure register for analog */
        //hadc->Instance->ADC_CFG_REG1 = ;

        /* Set single channel */
        hadc->Instance->ADC_CFG_REG1 |= GPADC_ADC_CFG_REG1_ANAU_GPADC_SE;

        i = hadc->Instance->ADC_CTRL_REG;
        // disable all triger mode by default
        i &= (~(GPADC_ADC_CTRL_REG_GPIO_TRIG_EN | GPADC_ADC_CTRL_REG_TIMER_TRIG_EN | GPADC_ADC_CTRL_REG_INIT_TIME));
        // set init time to max
#if (GPADC_CALIB_FLOW_VERSION == 1)
        i |= GPADC_ADC_CTRL_REG_INIT_TIME;
#else
        i |= (8 << GPADC_ADC_CTRL_REG_INIT_TIME_Pos);
#endif
        hadc->Instance->ADC_CTRL_REG = i;
#if (GPADC_CALIB_FLOW_VERSION != 1)
        __HAL_ADC_SET_SAMPLE_WIDTH(hadc, hadc->Init.sample_width);
        __HAL_ADC_SET_CONV_WIDTH(hadc, hadc->Init.conv_width);
        __HAL_ADC_SET_DATA_DELAY(hadc, hadc->Init.data_samp_delay);
        // TODO: Add for Range
#else
        /* Set ADC clock, clk = 12/(div+1) */
        __HAL_ADC_SET_CLOCK_DIV(hadc, hadc->Init.clk_div);
        // set range
        if (hadc->Init.atten3)  //enable 3X mode
            hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_ATTN3X);
        else //disable 3X mode
            hadc->Instance->ADC_CFG_REG1 &= (~GPADC_ADC_CFG_REG1_ANAU_GPADC_ATTN3X);
#endif


        /* Disable all slot at initial */
        for (i = 0; i < 8; i++)
            HAL_ADC_EnableSlot(hadc, i, 0);

        // set LDOCORE and LDOREF enable
#if (GPADC_CALIB_FLOW_VERSION != 1)
        // TODO:
        uint32_t value;
        value = hadc->Instance->ADC_CFG_REG1;
        // register default setting from hardware sample code
#if (GPADC_CALIB_FLOW_VERSION == 3)
        value &= ~(GPADC_ADC_CFG_REG1_ANAU_GPADC_VSP | GPADC_ADC_CFG_REG1_ANAU_GPADC_CMM | GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_V18);

        if (hadc->Init.avdd_v18_en)
            value |= (0x8 << GPADC_ADC_CFG_REG1_ANAU_GPADC_CMM_Pos) | GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_V18;
        else
#else
        value &= ~(GPADC_ADC_CFG_REG1_ANAU_GPADC_VSP | GPADC_ADC_CFG_REG1_ANAU_GPADC_CMM);
#endif
            value |= ((2 << GPADC_ADC_CFG_REG1_ANAU_GPADC_VSP_Pos) | (0x10 << GPADC_ADC_CFG_REG1_ANAU_GPADC_CMM_Pos));
        hadc->Instance->ADC_CFG_REG1 = value;

#if (GPADC_CALIB_FLOW_VERSION == 3)
        hwp_hpsys_cfg->ANAU_CR |= (HPSYS_CFG_ANAU_CR_EN_BG);
#endif /* (GPADC_CALIB_FLOW_VERSION == 3) */

#else   // (GPADC_CALIB_FLOW_VERSION == 1)
        hwp_tsen->BGR |= TSEN_BGR_EN;
        hwp_tsen->ANAU_ANA_TP |= TSEN_ANAU_ANA_TP_ANAU_IARY_EN;
#endif

    }
    else
    {
        /* Update ADC state machine to error */
        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL);

        tmp_hal_status = HAL_ERROR;
    }

    /* Return function status */
    return tmp_hal_status;
}


/**
  * @brief  Deinitialize the ADC peripheral registers to their default reset
  *         values, with deinitialization of the ADC MSP.
  * @note   For devices with several ADCs: reset of ADC common registers is done
  *         only if all ADCs sharing the same common group are disabled.
  *         If this is not the case, reset of these common parameters reset is
  *         bypassed without error reporting: it can be the intended behaviour in
  *         case of reset of a single ADC while the other ADCs sharing the same
  *         common group is still running.
  * @param  hadc ADC handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Check ADC handle */
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }

    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_BUSY_INTERNAL);

    /* Disable ADC peripheral if conversions are effectively stopped */
    if (tmp_hal_status == HAL_OK)
    {
        // reset LDOCORE and LDOREF enable
        //hadc->Instance->ADC_CFG_REG1 &= (~(GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOCORE_EN|GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN));

        /* Check if ADC is effectively disabled */
        if (tmp_hal_status != HAL_ERROR)
        {
            /* Change ADC state */
            hadc->State = HAL_ADC_STATE_READY;
        }
    }


    /* Configuration of ADC parameters if previous preliminary actions are      */
    /* correctly completed.                                                     */
    if (tmp_hal_status != HAL_ERROR)
    {
        /* Set ADC error code to none */
        ADC_CLEAR_ERRORCODE(hadc);

        /* Set ADC state */
        hadc->State = HAL_ADC_STATE_RESET;
    }

    /* Deinitial ADC Msp */
    HAL_ADC_MspDeInit(hadc);

    /* Process unlocked */
    __HAL_UNLOCK(hadc);

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @}
  */

/** @defgroup ADC_Exported_Functions_Group2 IO operation functions
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
      (+) Handle ADC interrupt request
      (+) Start conversion of regular group and enable DMA transfer.
      (+) Stop conversion of regular group and disable ADC DMA transfer.
@endverbatim
  * @{
  */

/**
  * @brief  Prepare ADC setting before start triger.
  * @param  hadc ADC handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Prepare(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL)
        return HAL_ERROR;

#if (GPADC_CALIB_FLOW_VERSION != 1)
#if (GPADC_CALIB_FLOW_VERSION == 3)
    // enable analog
    hwp_hpsys_cfg->ANAU_CR |= (HPSYS_CFG_ANAU_CR_EN_VBAT_MON);
    hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#else   /* GPADC_CALIB_FLOW_VERSION == 2 */
    hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_BG | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif /* GPADC_CALIB_FLOW_VERSION == 3 */
    ADC_SET_UNMUTE(hadc);

#else   /* (GPADC_CALIB_FLOW_VERSION == 1) */
    // set LDOCORE and LDOREF enable
    hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOCORE_EN | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif

    HAL_Delay_us(200);
    hadc->Instance->ADC_CTRL_REG |= (GPADC_ADC_CTRL_REG_FRC_EN_ADC);

    // TODO: force set to single end, if use differential mode later, add new interface or paramter
    __HAL_ADC_SINGLE_END(hadc);

    /* Process locked */
    __HAL_LOCK(hadc);

    /* Start conversion if ADC is effectively enabled */

    /* Set ADC state                                                        */
    /* - Clear state bitfield related to regular group conversion results   */
    /* - Set state bitfield related to regular operation                    */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR,
                      HAL_ADC_STATE_REG_BUSY);

    /* Reset ADC all error code fields */
    ADC_CLEAR_ERRORCODE(hadc);
#if (GPADC_CALIB_FLOW_VERSION != 1)
    HAL_Delay_us(200);
#else
    HAL_Delay_us(50);
#endif
    /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential         */
    /* interruption, to let the process to ADC IRQ Handler.                 */
    __HAL_UNLOCK(hadc);

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Enables ADC, starts conversion of regular group.
  *         Interruptions enabled in this function: None.
  * @param  hadc ADC handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef res = HAL_OK;

    /* Perform ADC enable and conversion start if no conversion is on going */
    res = HAL_ADC_Prepare(hadc);
    if (res == HAL_OK)
        hadc->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_ADC_START;

    return res;
}

/**
  * @brief  Stop ADC conversion of regular group, disable ADC peripheral.
  * @param  hadc ADC handle
  * @retval HAL status.
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hadc);

    if (hadc->Init.op_mode != 0)
        hadc->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_ADC_STOP;

    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_REG_BUSY,
                      HAL_ADC_STATE_READY);

    // reset LDOCORE and LDOREF enable
#if (GPADC_CALIB_FLOW_VERSION != 1)
#if (GPADC_CALIB_FLOW_VERSION == 3)
    hadc->Instance->ADC_CFG_REG1 &= ~(GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
    hwp_hpsys_cfg->ANAU_CR &= (~HPSYS_CFG_ANAU_CR_EN_VBAT_MON);
#else   /* (GPADC_CALIB_FLOW_VERSION == 2) */
    // TODO
    hadc->Instance->ADC_CFG_REG1 &= ~(GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_BG | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif /* (GPADC_CALIB_FLOW_VERSION == 3) */
    ADC_SET_MUTE(hadc);

#else   /* (GPADC_CALIB_FLOW_VERSION == 1) */
    hadc->Instance->ADC_CFG_REG1 &= (~(GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOCORE_EN | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN));
    hadc->Instance->ADC_CTRL_REG &= (~GPADC_ADC_CTRL_REG_FRC_EN_ADC);
#endif

    /* Process unlocked */
    __HAL_UNLOCK(hadc);

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Wait for regular group conversion to be completed.
  * @param  hadc ADC handle
  * @param  Timeout Timeout value in millisecond.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout)
{
    uint32_t tickstart;

    /* Check ADC handle */
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }

    /* Get tick count */
    tickstart = HAL_GetTick();

    /* Wait until End of Conversion flag is raised */
    while (HAL_IS_BIT_CLR(hadc->Instance->GPADC_IRQ, GPADC_GPADC_IRQ_GPADC_IRSR))
    {
        /* Check if timeout is disabled (set to infinite wait) */
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0) || ((HAL_GetTick() - tickstart) > Timeout))
            {
                /* Update ADC state machine to timeout */
                SET_BIT(hadc->State, HAL_ADC_STATE_TIMEOUT);

                /* Process unlocked */
                __HAL_UNLOCK(hadc);

                return HAL_TIMEOUT;
            }
        }
    }

    /* Clear ISR */
    __HAL_ADC_CLEAR_FLAG(hadc, GPADC_GPADC_IRQ_GPADC_ICR);

    /* Update ADC state machine */
    SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

    return HAL_OK;
}

/**
  * @brief  Enables ADC, starts conversion of regular group with interruption.
  *         Interruptions enabled in this function:
  *          - EOC (end of conversion of regular group) or EOS (end of
  *            sequence of regular group) depending on ADC initialization
  *            parameter "EOCSelection"
  *          - overrun (if available)
  *         Each of these interruptions has its dedicated callback function.
  * @param  hadc ADC handle
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Check ADC handle */
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }

    __HAL_ADC_ENABLE_IRQ(hadc, GPADC_GPADC_IRQ_GPADC_IMR);

    __HAL_ADC_START_CONV(hadc);
    /* Return function status */
    return tmp_hal_status;
}


/**
  * @brief  Stop ADC conversion of regular group, disable interruption of
  *         end-of-conversion, disable ADC peripheral.
  * @param  hadc ADC handle
  * @retval HAL status.
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hadc);

    if (hadc->Init.op_mode != 0)
        __HAL_ADC_STOP_CONV(hadc);

    __HAL_ADC_DISABLE_IRQ(hadc, GPADC_GPADC_IRQ_GPADC_IMR);

    /* Process unlocked */
    __HAL_UNLOCK(hadc);

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Enables ADC, starts conversion of regular group and transfers result
  *         through DMA.
  *         Interruptions enabled in this function:
  *          - DMA transfer complete
  *          - DMA half transfer
  *          - overrun
  *         Each of these interruptions has its dedicated callback function.
  * @param  hadc ADC handle
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from ADC peripheral to memory.
  * @retval None
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_DMA_PREPARE(ADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Perform ADC enable and conversion start if no conversion is on going */
#if 1
    {
#if (GPADC_CALIB_FLOW_VERSION != 1)
#if (GPADC_CALIB_FLOW_VERSION == 3)
        // TODO
        hwp_hpsys_cfg->ANAU_CR |= (HPSYS_CFG_ANAU_CR_EN_VBAT_MON);
        //hwp_hpsys_cfg->ANAU_CR |= (HPSYS_CFG_ANAU_CR_EN_BG);
        hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#else
        hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_BG | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif

#else   //(GPADC_CALIB_FLOW_VERSION == 1)
        // set LDOCORE and LDOREF enable
        hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOCORE_EN | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif

        hadc->Instance->ADC_CTRL_REG |= (GPADC_ADC_CTRL_REG_FRC_EN_ADC);
        /* Process locked */
        __HAL_LOCK(hadc);

        /* Start conversion if ADC is effectively enabled */
        if (tmp_hal_status == HAL_OK)
        {
            /* Set ADC state                                                        */
            /* - Clear state bitfield related to regular group conversion results   */
            /* - Set state bitfield related to regular operation                    */
            ADC_STATE_CLR_SET(hadc->State,
                              HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR,
                              HAL_ADC_STATE_REG_BUSY);

            /* Reset ADC all error code fields */
            ADC_CLEAR_ERRORCODE(hadc);
#if (GPADC_CALIB_FLOW_VERSION != 1)
            HAL_Delay_us(200);
#else
            HAL_Delay_us(50);
#endif
            /* Process unlocked */
            /* Unlock before starting ADC conversions: in case of potential         */
            /* interruption, to let the process to ADC IRQ Handler.                 */
            __HAL_UNLOCK(hadc);

        }
    }
#endif

    /* Check ADC handle */
    if (hadc == NULL || hadc->DMA_Handle == NULL)
    {
        return HAL_ERROR;
    }

    /* Init DMA configure*/
    //hadc->DMA_Handle->Instance                 = GPADC_DMA_INSTANCE;
    //hadc->DMA_Handle->Init.Request             = GPADC_DMA_REQUEST;
    //hadc->DMA_Handle->Init.Direction = DMA_PERIPH_TO_MEMORY;
    //hadc->DMA_Handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    //hadc->DMA_Handle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    //hadc->DMA_Handle->Init.PeriphInc           = DMA_PINC_DISABLE;
    //hadc->DMA_Handle->Init.MemInc              = DMA_MINC_ENABLE;
    //hadc->DMA_Handle->Init.Mode                = DMA_NORMAL;
    //hadc->DMA_Handle->Init.Priority            = DMA_PRIORITY_MEDIUM;
    hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;
    hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;

    tmp_hal_status = HAL_DMA_Init(hadc->DMA_Handle);
    if (tmp_hal_status != HAL_OK)
        return tmp_hal_status;

    return tmp_hal_status;
}
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Perform ADC enable and conversion start if no conversion is on going */
#if 0
    {
#ifndef  SF32LB55X
#ifdef SF32LB52X
        // TODO
        hwp_hpsys_cfg->ANAU_CR |= (HPSYS_CFG_ANAU_CR_EN_VBAT_MON);
        //hwp_hpsys_cfg->ANAU_CR |= (HPSYS_CFG_ANAU_CR_EN_BG);
        hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#else
        hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_BG | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif /* SF32LB52X */

#else
        // set LDOCORE and LDOREF enable
        hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOCORE_EN | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif

        hadc->Instance->ADC_CTRL_REG |= (GPADC_ADC_CTRL_REG_FRC_EN_ADC);
        /* Process locked */
        __HAL_LOCK(hadc);

        /* Start conversion if ADC is effectively enabled */
        if (tmp_hal_status == HAL_OK)
        {
            /* Set ADC state                                                        */
            /* - Clear state bitfield related to regular group conversion results   */
            /* - Set state bitfield related to regular operation                    */
            ADC_STATE_CLR_SET(hadc->State,
                              HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR,
                              HAL_ADC_STATE_REG_BUSY);

            /* Reset ADC all error code fields */
            ADC_CLEAR_ERRORCODE(hadc);
#ifndef  SF32LB55X
            HAL_Delay_us(200);
#else
            HAL_Delay_us(50);
#endif
            /* Process unlocked */
            /* Unlock before starting ADC conversions: in case of potential         */
            /* interruption, to let the process to ADC IRQ Handler.                 */
            __HAL_UNLOCK(hadc);
            * /

        }
    }
#endif

    /* Check ADC handle */
    if (hadc == NULL || hadc->DMA_Handle == NULL)
    {
        return HAL_ERROR;
    }
#if 0
    /* Init DMA configure*/
    hadc->DMA_Handle->Instance                 = GPADC_DMA_INSTANCE;
    hadc->DMA_Handle->Init.Request             = GPADC_DMA_REQUEST;
    hadc->DMA_Handle->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hadc->DMA_Handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hadc->DMA_Handle->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hadc->DMA_Handle->Init.PeriphInc           = DMA_PINC_DISABLE;
    hadc->DMA_Handle->Init.MemInc              = DMA_MINC_ENABLE;
    hadc->DMA_Handle->Init.Mode                = DMA_NORMAL;
    hadc->DMA_Handle->Init.Priority            = DMA_PRIORITY_MEDIUM;
    hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;
    hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;


    tmp_hal_status = HAL_DMA_Init(hadc->DMA_Handle);
    if (tmp_hal_status != HAL_OK)
        return tmp_hal_status;
#endif
    ADC_DMA_ENABLE(hadc);
    tmp_hal_status = HAL_DMA_Start(hadc->DMA_Handle, (uint32_t)(&hadc->Instance->ADC_DMA_RDATA), (uint32_t)pData, Length);
    hadc->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_ADC_START;

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief  Stop ADC conversion of regular group, disable ADC DMA transfer, disable
  *         ADC peripheral.
  *         Each of these interruptions has its dedicated callback function.
  * @param  hadc ADC handle
  * @retval HAL status.
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hadc);

    ADC_DMA_DISABLE(hadc);

    tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

    if (hadc->Init.op_mode != 0)
        hadc->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_ADC_STOP;

    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_REG_BUSY,
                      HAL_ADC_STATE_READY);

    // reset LDOCORE and LDOREF enable
#if (GPADC_CALIB_FLOW_VERSION != 1)
#if (GPADC_CALIB_FLOW_VERSION == 3)
    hadc->Instance->ADC_CFG_REG1 &= ~(GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
    //hwp_hpsys_cfg->ANAU_CR &= (~HPSYS_CFG_ANAU_CR_EN_BG);
    hwp_hpsys_cfg->ANAU_CR &= (~HPSYS_CFG_ANAU_CR_EN_VBAT_MON);
#else
    // TODO
    hadc->Instance->ADC_CFG_REG1 &= ~(GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_BG | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif
#else
    hadc->Instance->ADC_CFG_REG1 &= (~(GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOCORE_EN | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN));
#endif
    hadc->Instance->ADC_CTRL_REG &= (~GPADC_ADC_CTRL_REG_FRC_EN_ADC);


    /* Process unlocked */
    __HAL_UNLOCK(hadc);

    /* Return function status */
    return tmp_hal_status;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_DMA_WAIT_DONE(ADC_HandleTypeDef *hadc, uint32_t timeout)
{
    HAL_StatusTypeDef res = HAL_OK;

    if ((hadc == NULL) || (hadc->DMA_Handle == NULL))
        return HAL_ERROR;

    res = HAL_DMA_PollForTransfer(hadc->DMA_Handle, HAL_DMA_FULL_TRANSFER, timeout);

    ADC_DMA_DISABLE(hadc);

    return res;
}


/**
  * @brief  Get ADC regular group conversion result.
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
  * @param  hadc ADC handle
  * @retval ADC group regular conversion data
  */
__HAL_ROM_USED uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc, uint32_t slot)
{
    uint32_t value, odd;
    __IO uint32_t *p = &(hadc->Instance->ADC_RDATA0);

    /* Check ADC handle */
    if (hadc == NULL || slot >= 8)
    {
        return HAL_ERROR;
    }
    odd = 0;

    if (hadc->Init.en_slot == 1)
    {
        p += slot / 2;
        odd = slot % 2;
    }
    value = *p;

    if (odd)
        value = (value  & GPADC_ADC_RDATA0_SLOT1_RDATA_Msk) >> GPADC_ADC_RDATA0_SLOT1_RDATA_Pos;
    else
        value = value & GPADC_ADC_RDATA0_SLOT0_RDATA_Msk;

    /* Return ADC converted value */
    return value;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Get_All(ADC_HandleTypeDef *hadc, uint32_t *buf)
{
    int i;
    __IO uint32_t *p = &(hadc->Instance->ADC_RDATA0);
    uint32_t *data = (uint32_t *)buf;

    /* Check ADC handle */
    if (hadc == NULL || buf == NULL)
        return HAL_ERROR;

    for (i = 0; i < 8; i++)
    {
        if (i & 1)
        {
            *data = (*p  & GPADC_ADC_RDATA0_SLOT1_RDATA_Msk) >> GPADC_ADC_RDATA0_SLOT1_RDATA_Pos;
            p++;
        }
        else
            *data = *p & GPADC_ADC_RDATA0_SLOT0_RDATA_Msk;
        data++;
    }

    return HAL_OK;
}


__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_SetSource(ADC_HandleTypeDef *hadc, HAL_ADC_SRC__T src)
{
    if (hadc == NULL || src > HAL_ADC_SRC_TIMER)
    {
        return HAL_ERROR;
    }

    hadc->Instance->ADC_CTRL_REG |= (src << GPADC_ADC_CTRL_REG_GPIO_TRIG_EN_Pos);

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_SetTimer(ADC_HandleTypeDef *hadc, HAL_ADC_SRC_TIME_T src)
{
    if (hadc == NULL || src >= HAL_ADC_SRC_TIMER_CNT)
    {
        return HAL_ERROR;
    }

    hadc->Instance->ADC_CTRL_REG |= (src << GPADC_ADC_CTRL_REG_TIMER_TRIG_SRC_SEL_Pos);

    return HAL_OK;
}


/**
  * @brief  Handles ADC interrupt request.
  * @param  hadc ADC handle
  * @retval None
  */
__HAL_ROM_USED void HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc)
{
    /* Clear ISR */
    __HAL_ADC_CLEAR_FLAG(hadc, GPADC_GPADC_IRQ_GPADC_ICR);

    /* Set status */
    SET_BIT(hadc->State, HAL_ADC_START_IRQ_DONE);
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  hadc ADC handle
  * @retval None
  */
__weak void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
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
  *         HAL_ADC_DeInit().
  * @note   Possibility to update parameters on the fly:
  *         This function initializes channel into regular group, following
  *         calls to this function can be used to reconfigure some parameters
  *         of structure "ADC_ChannelConfTypeDef" on the fly, without reseting
  *         the ADC.
  *         The setting of these parameters is conditioned to ADC state.
  *         For parameters constraints, see comments of structure
  *         "ADC_ChannelConfTypeDef".
  * @param  hadc ADC handle
  * @param  sConfig Structure of ADC channel for regular group.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig)
{
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;
    uint32_t value = 0;
    __IO uint32_t *p = &(hadc->Instance->ADC_SLOT0_REG);

    /* Check ADC handle */
    if (hadc == NULL || sConfig == NULL || sConfig->Channel >= 8)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hadc);

    if (hadc->Init.en_slot == 1)
        p += sConfig->Channel;

    value = *p;

    value &= ~(GPADC_ADC_SLOT0_REG_NCHNL_SEL | GPADC_ADC_SLOT0_REG_PCHNL_SEL | GPADC_ADC_SLOT0_REG_ACC_NUM);
    value |= ((sConfig->nchnl_sel << GPADC_ADC_SLOT0_REG_NCHNL_SEL_Pos) & GPADC_ADC_SLOT0_REG_NCHNL_SEL_Msk)
             | ((sConfig->pchnl_sel << GPADC_ADC_SLOT0_REG_PCHNL_SEL_Pos) & GPADC_ADC_SLOT0_REG_PCHNL_SEL_Msk)
             | ((sConfig->acc_num << GPADC_ADC_SLOT0_REG_ACC_NUM_Pos) & GPADC_ADC_SLOT0_REG_ACC_NUM_Msk);


    if (sConfig->slot_en)
    {
        value |= GPADC_ADC_SLOT0_REG_SLOT_EN;
    }
    else
    {
        value &= ~GPADC_ADC_SLOT0_REG_SLOT_EN;
    }

    *p = value;

    /* Process unlocked */
    __HAL_UNLOCK(hadc);

    /* Return function status */
    return tmp_hal_status;
}

/**
  * @brief Set ADC sampling frequency.
  * @param hadc ADC handle.
  * @param freq frequence want to be set.
  * @retval actual work frequency, 0 if fail.
  */
__HAL_ROM_USED uint32_t HAL_ADC_SetFreq(ADC_HandleTypeDef *hadc, uint32_t freq)
{
    uint32_t fpclk; // = HAL_RCC_GetPCLKFreq(CORE_ID_LCPU, 1);
    uint32_t div; // = fpclk / freq;

    if (freq == 0)
        return 0;

    //extern void rt_kprintf(const char *fmt, ...);
#if (GPADC_CALIB_FLOW_VERSION != 3)
    fpclk = HAL_RCC_GetPCLKFreq(CORE_ID_LCPU, 1);
    div = fpclk / freq;
#else
    fpclk = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);
    div = fpclk / freq;
#endif
    //for single triger mode, init time will effect adc frequency, init time only work once after strt
    //int init_time = (hadc->Instance->ADC_CTRL_REG & GPADC_ADC_CTRL_REG_INIT_TIME_Msk)>>GPADC_ADC_CTRL_REG_INIT_TIME_Pos;
    //div = fpclk / (freq * (1+init_time));
#if (GPADC_CALIB_FLOW_VERSION != 1)
    uint32_t min_conv, min_samp, data_dly;
    uint32_t max_conv, max_samp;

    max_conv = GPADC_ADC_CTRL_REG2_CONV_WIDTH >> GPADC_ADC_CTRL_REG2_CONV_WIDTH_Pos;
    max_samp = GPADC_ADC_CTRL_REG2_SAMP_WIDTH >> GPADC_ADC_CTRL_REG2_SAMP_WIDTH_Pos;
#if (GPADC_CALIB_FLOW_VERSION == 3)
    uint32_t std_conv, std_samp, std_dly, std_freq;
    std_conv = 76;
    std_samp = 74;
    std_dly = 2;
    std_freq = 36000000; // same to ATE
    std_freq /= 1000;   // to avoid over range for u32
    fpclk /= 1000;     // to avoid over range for u32
    min_conv = (std_conv * fpclk + std_freq / 2) / std_freq - 1;
    data_dly = (std_dly * fpclk + std_freq / 2) / std_freq;
    min_samp = (std_samp * fpclk + std_freq / 2) / std_freq - data_dly - 1;
    if ((min_conv > max_conv) || (min_samp > max_samp)) // over rage
        HAL_ASSERT(0);
#else
    min_conv = 24;
    min_samp = 22;
    data_dly = 2;

    //cnt = min_conv + min_samp + data_dly + 2;
    if ((div <= max_conv * 2) && (div <= max_samp * 2)) // have enough clock
    {
        min_conv = div / 2;
        data_dly = 2;
        min_samp = min_conv - data_dly - 2;
    }
    else // not support
        return 0;
#endif

    //sampel rate too high, not support, set to max freq
    hadc->Init.data_samp_delay = data_dly;
    hadc->Init.conv_width = min_conv;
    hadc->Init.sample_width = min_samp;

    __HAL_ADC_SET_SAMPLE_WIDTH(hadc, hadc->Init.sample_width);
    __HAL_ADC_SET_CONV_WIDTH(hadc, hadc->Init.conv_width);
    __HAL_ADC_SET_DATA_DELAY(hadc, hadc->Init.data_samp_delay);
    //rt_kprintf("src freq %d, div %d, data_samp_delay %d, conv_width %d, sample_width %d\n", fpclk, div, data_dly, min_conv, min_samp);
#else
    uint32_t maxdiv;
    maxdiv = GPADC_ADC_CTRL_REG_ADC_CLK_DIV >> GPADC_ADC_CTRL_REG_ADC_CLK_DIV_Pos;
    maxdiv = maxdiv + 1; // divider + 1

    if (div > maxdiv)
        div = maxdiv;
    if (div < 1)
        div = 1;
    hadc->Init.clk_div = div - 1;
    __HAL_ADC_SET_CLOCK_DIV(hadc, hadc->Init.clk_div);
#endif
    return fpclk / div;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_EnableSlot(ADC_HandleTypeDef *hadc, uint32_t slot, uint8_t en)
{
    __IO uint32_t *p = &(hadc->Instance->ADC_SLOT0_REG);

    /* Check ADC handle */
    if (hadc == NULL || slot >= 8)
    {
        return HAL_ERROR;
    }
    if (hadc->Init.en_slot == 1)
        p += slot;

    // Clear these bit, disable
    //*p &= (~GPADC_ADC_SLOT0_REG_SLOT_EN);
    //*p &= (~GPADC_ADC_SLOT0_REG_ACC_NUM);

    if (en != 0)
    {
        *p |= GPADC_ADC_SLOT0_REG_SLOT_EN;
        //*p |= (3 << GPADC_ADC_SLOT0_REG_ACC_NUM_Pos);
    }
    else
    {
        *p &= (~GPADC_ADC_SLOT0_REG_SLOT_EN);
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_ADC_Set_MultiMode(ADC_HandleTypeDef *hadc, uint8_t multi_mode)
{
    if (hadc == NULL)
        return HAL_ERROR;

    if (multi_mode)
        hadc->Init.en_slot = 1;
    else
        hadc->Init.en_slot = 0;

    return HAL_OK;
}


__HAL_ROM_USED int HAL_ADC_Get_Offset(ADC_HandleTypeDef *hadc)
{
    int offset;

#if (GPADC_CALIB_FLOW_VERSION != 1)
#if (GPADC_CALIB_FLOW_VERSION == 3)
    hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#else
    // TODO
    hadc->Instance->ADC_CFG_REG1 |= (GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_BG | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif
#else

    hadc->Instance->ADC_CFG_REG1 |= GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN;

    hadc->Instance->ADC_CFG_REG1 |= GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOCORE_EN;
#endif
    //hadc->Instance->ADC_CFG_REG1 |= GPADC_ADC_CFG_REG1_ANAU_GPADC_SE;
    hadc->Instance->ADC_CFG_REG1 |= GPADC_ADC_CFG_REG1_ANAU_GPADC_MUTE;

    hadc->Instance->ADC_SLOT0_REG &= (~GPADC_ADC_SLOT0_REG_ACC_NUM);
    hadc->Instance->ADC_SLOT0_REG |= (3 << GPADC_ADC_SLOT0_REG_ACC_NUM_Pos);

    HAL_Delay_us(50);
    hadc->Instance->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_ADC_START;
    while (!(hadc->Instance->GPADC_IRQ & GPADC_GPADC_IRQ_GPADC_IRSR_Msk));

    offset = hadc->Instance->ADC_RDATA0 & GPADC_ADC_RDATA0_SLOT0_RDATA_Msk;

    hadc->Instance->GPADC_IRQ |= GPADC_GPADC_IRQ_GPADC_ICR;


#if (GPADC_CALIB_FLOW_VERSION != 1)
#if (GPADC_CALIB_FLOW_VERSION == 3)
    hadc->Instance->ADC_CFG_REG1 &= ~(GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#else
    // TODO
    hadc->Instance->ADC_CFG_REG1 &= ~(GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_BG | GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN);
#endif
#else
    hadc->Instance->ADC_CFG_REG1 &= ~GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN;
    hadc->Instance->ADC_CFG_REG1 &= ~GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOCORE_EN;
#endif
    hadc->Instance->ADC_CFG_REG1 &= ~GPADC_ADC_CFG_REG1_ANAU_GPADC_MUTE;

    return offset - 512;
}

/**
  * @}
  */


/** @defgroup ADC_Exported_Functions_Group4 Peripheral State functions
 *  @brief    Peripheral State functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions to get in run-time the status of the
    peripheral.
      (+) Check the ADC state
      (+) Check the ADC error code

@endverbatim
  * @{
  */

/**
  * @brief  Return the ADC state
  * @note   ADC state machine is managed by bitfields, ADC status must be
  *         compared with states bits.
  *         For example:
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_REG_BUSY)) "
  *           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_AWD1)    ) "
  * @param  hadc ADC handle
  * @retval HAL state
  */
__HAL_ROM_USED uint32_t HAL_ADC_GetState(ADC_HandleTypeDef *hadc)
{
    /* Return ADC state */
    return hadc->State;
}

/**
  * @brief  Return the ADC error code
  * @param  hadc ADC handle
  * @retval ADC Error Code
  */
__HAL_ROM_USED uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc)
{
    return hadc->ErrorCode;
}

/**
  * @brief  Initialize the ADC MSP.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *               the configuration information for ADC module.
  * @retval None
  */
__weak void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hadc);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_ADC_MspInit should be implemented in the user file
     */
}

/**
  * @brief  De-Initialize the ADC MSP.
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *               the configuration information for ADC module.
  * @retval None
  */
__weak void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hadc);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_ADC_MspDeInit should be implemented in the user file
     */
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
static void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma)
{
    /* Retrieve ADC handle corresponding to current DMA handle */
    ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Update state machine on conversion status if not in error state */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA))
    {
        /* Set ADC state */
        SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

        /* Conversion complete callback */
        HAL_ADC_ConvCpltCallback(hadc);
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
static void ADC_DMAError(DMA_HandleTypeDef *hdma)
{
    /* Retrieve ADC handle corresponding to current DMA handle */
    ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_DMA);

    /* Set ADC error code to DMA error */
    SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_DMA);

}

/**
  * @}
  */

#endif /* HAL_ADC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
