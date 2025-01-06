/**
  ******************************************************************************
  * @file   bf0_hal_rtc.c
  * @author Sifli software development team
  * @brief   RTC HAL module driver.
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

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup RTC Real timer clock
  * @brief RTC HAL module driver
  * @{
  */

#if defined(HAL_RTC_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup RTC_Exported_Functions RTC Exported Functions
  * @{
  */

/** @defgroup RTC_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
   [..] This section provides functions allowing to initialize and configure the
         RTC Prescaler (Synchronous and Asynchronous), RTC Hour format, disable
         RTC registers Write protection, enter and exit the RTC initialization mode,
         RTC registers synchronization check and reference clock detection enable.
         (#) The RTC Prescaler is programmed to generate the RTC 1Hz time base.
             It is split into 2 programmable prescalers to minimize power consumption.
             (++) A 7-bit asynchronous prescaler and a 13-bit synchronous prescaler.
             (++) When both prescalers are used, it is recommended to configure the
                 asynchronous prescaler to a high value to minimize power consumption.
         (#) All RTC registers are Write protected. Writing to the RTC registers
             is enabled by writing a key into the Write Protection register, RTC_WPR.
         (#) To configure the RTC Calendar, user application should enter
             initialization mode. In this mode, the calendar counter is stopped
             and its value can be updated. When the initialization sequence is
             complete, the calendar restarts counting after 4 RTCCLK cycles.
         (#) To read the calendar through the shadow registers after Calendar
             initialization, calendar update or after wake-up from low power modes
             the software must first clear the RSF flag. The software must then
             wait until it is set again before reading the calendar, which means
             that the calendar registers have been correctly copied into the
             RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
             implements the above software sequence (RSF clear and RSF check).

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the RTC peripheral
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  wakesrc wakeup source, see RTC_INIT_XXX
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc, uint32_t wakesrc)
{
    HAL_StatusTypeDef r = HAL_ERROR;

    if (hrtc)
    {
        if (wakesrc == RTC_INIT_SKIP)
        {
            hrtc->State = HAL_RTC_STATE_READY;
            if (SystemPowerOnModeGet() == PM_COLD_BOOT) /*Cold boot, remove wakeup timer and Alarm*/
            {
                hrtc->Instance->CR &=
                    ((uint32_t)~(RTC_CR_ALRME | RTC_CR_ALRMIE | RTC_CR_WUTE | RTC_CR_WUTIE));
                hrtc->Instance->ISR &= ~(RTC_ISR_ALRMF | RTC_ISR_WUTF);
            }
            r = HAL_OK;
        }
        else
        {
            if (hrtc->State == HAL_RTC_STATE_RESET)
            {
                hrtc->Lock = HAL_UNLOCKED;      /* Allocate lock resource and initialize it */
                HAL_RTC_MspInit(hrtc);          /* Initialize RTC MSP */
            }
            hrtc->State = HAL_RTC_STATE_BUSY;           /* Set RTC state */
            hrtc->callback = NULL;
            if (RTC_EnterInitMode(hrtc) != HAL_OK)      /* Set Initialization mode */
                hrtc->State = HAL_RTC_STATE_ERROR;      /* Set RTC state */
            else
            {
                hrtc->Instance->CR &=                   /* Clear RTC_CR FMT, OSEL and POL Bits */
                    ((uint32_t)~(RTC_CR_FMT | RTC_CR_OSEL | RTC_CR_POL));
                if (wakesrc == RTC_INIT_NORMAL)         /* Clear wakeup timer and alarm */
                {
                    hrtc->Instance->CR &=
                        ((uint32_t)~(RTC_CR_ALRME | RTC_CR_ALRMIE | RTC_CR_WUTE | RTC_CR_WUTIE));
                    hrtc->Instance->ISR &= ~(RTC_ISR_ALRMF | RTC_ISR_WUTF);
                }
                hrtc->Instance->CR |=                   /* Set RTC_CR register */
                    (uint32_t)(hrtc->Init.HourFormat);
                hrtc->Instance->ISR &= (uint32_t)~RTC_ISR_INIT; /* Exit Initialization mode */

                {
                    uint32_t psclr = (hrtc->Init.DivAInt << RTC_PSCLR_DIVA_INT_Pos) & RTC_PSCLR_DIVA_INT;
                    psclr |= (hrtc->Init.DivAFrac << RTC_PSCLR_DIVA_FRAC_Pos) & RTC_PSCLR_DIVA_FRAC;
                    psclr |= (hrtc->Init.DivB << RTC_PSCLR_DIVB_Pos) & RTC_PSCLR_DIVB;
                    hrtc->Instance->PSCLR = psclr;
                }
                /* If  CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
                if ((hrtc->Instance->CR & RTC_CR_BYPSHAD) == RESET &&
                        (HAL_RTC_WaitForSynchro(hrtc) != HAL_OK))
                    hrtc->State = HAL_RTC_STATE_ERROR;
                else
                {
                    /* Set RTC state */
                    hrtc->State = HAL_RTC_STATE_READY;
                    r = HAL_OK;
                }
            }
        }
    }
    return r;
}

/**
  * @brief  DeInitializes the RTC peripheral
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @note   This function doesn't reset the RTC Backup Data registers.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc)
{
    uint32_t tickstart = 0U;
    HAL_StatusTypeDef r = HAL_ERROR;

    hrtc->State = HAL_RTC_STATE_BUSY;           /* Set RTC state */
    if (RTC_EnterInitMode(hrtc) != HAL_OK)      /* Set Initialization mode */
        hrtc->State = HAL_RTC_STATE_ERROR;      /* Set RTC state */
    else
    {
        hrtc->Instance->TR = 0x00000000U;       /* Reset TR, DR and CR registers */
        hrtc->Instance->DR = 0x00002101U;
        hrtc->Instance->CR &= RTC_CR_WUCKSEL;      /* Reset All CR bits except CR[0] */
        tickstart = HAL_GetTick();              /* Get tick */
        /* Wait till WUTWF flag is set and if Time out is reached exit */
        while (((hrtc->Instance->ISR) & RTC_ISR_WUTWF) == (uint32_t)RESET)
            if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
            {
                /* Set RTC state */
                hrtc->State = HAL_RTC_STATE_TIMEOUT;
                r = HAL_TIMEOUT;
            }
        if (r != HAL_TIMEOUT)
        {
            hrtc->Instance->CR &= 0x00000000U;      /* Reset all RTC CR register bits */
            hrtc->Instance->WUTR = 0x0000FFFFU;
            hrtc->Instance->SHIFTR = 0x00000000U;
            hrtc->Instance->ISR = 0x00000000U;      /* Reset ISR register and exit initialization mode */

            /* If  RTC_CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
            if ((hrtc->Instance->CR & RTC_CR_BYPSHAD) == RESET && HAL_RTC_WaitForSynchro(hrtc) != HAL_OK)
                hrtc->State = HAL_RTC_STATE_ERROR;
            else
            {
                HAL_RTC_MspDeInit(hrtc);          /* De-Initialize RTC MSP */
                hrtc->State = HAL_RTC_STATE_RESET;
                r = HAL_OK;
            }
        }
    }

    __HAL_UNLOCK(hrtc);           /* Release Lock */
    return HAL_OK;
}

/**
  * @brief  Initializes the RTC MSP.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrtc);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_RTC_MspInit could be implemented in the user file
     */
}

/**
  * @brief  DeInitializes the RTC MSP.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hrtc);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_RTC_MspDeInit could be implemented in the user file
     */
}

static uint32_t time_2_reg(RTC_TimeTypeDef *sTime, uint32_t Format)
{
    uint32_t tmpreg = 0U;

    if (Format == RTC_FORMAT_BIN)
    {
        tmpreg = (uint32_t)(((uint32_t)RTC_ByteToBcd2(sTime->Hours) << RTC_TR_HU_Pos) | \
                            ((uint32_t)RTC_ByteToBcd2(sTime->Minutes) << RTC_TR_MNU_Pos) | \
                            ((uint32_t)RTC_ByteToBcd2(sTime->Seconds) << RTC_TR_SU_Pos) | \
                            (((uint32_t)sTime->TimeFormat) << RTC_TR_PM_Pos) | \
                            sTime->SubSeconds \
                           );
    }
    else
    {
        tmpreg = (((uint32_t)(sTime->Hours) << RTC_TR_HU_Pos) | \
                  ((uint32_t)(sTime->Minutes) << RTC_TR_MNU_Pos) | \
                  ((uint32_t)(sTime->Seconds) << RTC_TR_SU_Pos) | \
                  ((uint32_t)(sTime->TimeFormat) << RTC_TR_PM_Pos) | \
                  sTime->SubSeconds \
                 );
    }
    return tmpreg;
}

/**
  * @} RTC_Exported_Functions_Group1
  */

/** @defgroup RTC_Exported_Functions_Group2 RTC Time and Date functions
 *  @brief   RTC Time and Date functions
 *
@verbatim
 ===============================================================================
                 ##### RTC Time and Date functions #####
 ===============================================================================

 [..] This section provides functions allowing to configure Time and Date features

@endverbatim
  * @{
  */

/**
  * @brief  Sets RTC current time.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTime Pointer to Time structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
    uint32_t tmpreg = 0U;
    HAL_StatusTypeDef r = HAL_ERROR;

    /* Process Locked */
    __HAL_LOCK(hrtc);
    hrtc->State = HAL_RTC_STATE_BUSY;

    if ((hrtc->Instance->CR & RTC_CR_FMT) == (uint32_t)RESET)
        sTime->TimeFormat = 0x00U;
    tmpreg = time_2_reg(sTime, Format);
    if (RTC_EnterInitMode(hrtc) != HAL_OK) /* Set Initialization mode */
        hrtc->State = HAL_RTC_STATE_ERROR;  /* Set RTC state */
    else
    {
        hrtc->Instance->TR = (uint32_t)(tmpreg); /* Set the RTC_TR register */
#if 0
        hrtc->Instance->CR &= (uint32_t)~RTC_CR_BCK;                    /* Clear the bits to be configured */
#endif
        hrtc->Instance->CR |=                                           /* Configure the RTC_CR register */
            (uint32_t)(sTime->DayLightSaving | sTime->StoreOperation);
        hrtc->Instance->ISR &= (uint32_t)~RTC_ISR_INIT;                 /* Exit Initialization mode */

        /* If  CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
        if ((hrtc->Instance->CR & RTC_CR_BYPSHAD) == RESET
                && HAL_RTC_WaitForSynchro(hrtc) != HAL_OK)
            hrtc->State = HAL_RTC_STATE_ERROR;

        else
        {
            hrtc->State = HAL_RTC_STATE_READY;
            r = HAL_OK;
        }
    }
    __HAL_UNLOCK(hrtc);
    return r;
}

__HAL_ROM_USED void reg_2_time(uint32_t reg, RTC_TimeTypeDef *sTime, uint32_t Format)
{
    /* Fill the structure fields with the read parameters */
    sTime->Hours = (uint8_t)((reg & (RTC_TR_HT | RTC_TR_HU)) >> RTC_TR_HU_Pos);
    sTime->Minutes = (uint8_t)((reg & (RTC_TR_MNT | RTC_TR_MNU)) >> RTC_TR_MNU_Pos);
    sTime->Seconds = (uint8_t)((reg & (RTC_TR_ST | RTC_TR_SU)) >> RTC_TR_SU_Pos);
    sTime->TimeFormat = (uint8_t)((reg & (RTC_TR_PM)) >> RTC_TR_PM_Pos);

    /* Check the input parameters format */
    if (Format == RTC_FORMAT_BIN)
    {
        /* Convert the time structure parameters to Binary format */
        sTime->Hours = (uint8_t)RTC_Bcd2ToByte(sTime->Hours);
        sTime->Minutes = (uint8_t)RTC_Bcd2ToByte(sTime->Minutes);
        sTime->Seconds = (uint8_t)RTC_Bcd2ToByte(sTime->Seconds);
    }
}
/**
  * @brief  Gets RTC current time.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sTime Pointer to Time structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @note  You can use SubSeconds and SecondFraction (sTime structure fields returned) to convert SubSeconds
  *        value in second fraction ratio with time unit following generic formula:
  *        Second fraction ratio * time_unit= [(SecondFraction-SubSeconds)/(SecondFraction+1)] * time_unit
  *        This conversion can be performed only if no shift operation is pending (ie. SHFP=0) when PREDIV_S >= SS
  * @note You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock the values
  *        in the higher-order calendar shadow registers to ensure consistency between the time and date values.
  *        Reading RTC current time locks the values in calendar shadow registers until current date is read.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
    uint32_t tmpreg = 0U;

    /* Get the TR register */

#ifdef SOC_BF0_HCPU  // Use LPSYS_CFG register to get time immediately.
    tmpreg = hwp_hpsys_cfg->RTC_TR;
#else
    tmpreg = hwp_lpsys_cfg->RTC_TR;
#endif
    reg_2_time(tmpreg, sTime, Format);
    sTime->SubSeconds = tmpreg & RTC_TR_SS;
    sTime->SecondFractionInt = (hrtc->Instance->PSCLR & RTC_PSCLR_DIVA_INT) >> RTC_PSCLR_DIVA_INT_Pos;
    sTime->SecondFractionFrac = (hrtc->Instance->PSCLR & RTC_PSCLR_DIVA_FRAC) >> RTC_PSCLR_DIVA_FRAC_Pos;

    return HAL_OK;
}

static uint32_t date_2_reg(RTC_DateTypeDef *sDate, uint32_t Format)
{
    uint32_t datetmpreg = 0U;

    if ((Format == RTC_FORMAT_BIN) && ((sDate->Month & 0x10U) == 0x10U))
        sDate->Month = (uint8_t)((sDate->Month & (uint8_t)~(0x10U)) + (uint8_t)0x0AU);

    if (sDate->Year < 100)
        datetmpreg |= RTC_DR_CB;
    else
        sDate->Year -= 100;

    if (Format == RTC_FORMAT_BIN)
        datetmpreg |= (((uint32_t)RTC_ByteToBcd2(sDate->Year) << RTC_DR_YU_Pos) | \
                       ((uint32_t)RTC_ByteToBcd2(sDate->Month) << RTC_DR_MU_Pos) | \
                       ((uint32_t)RTC_ByteToBcd2(sDate->Date)) | \
                       ((uint32_t)sDate->WeekDay << RTC_DR_WD_Pos));
    else
        datetmpreg |= ((((uint32_t)sDate->Year) << RTC_DR_YU_Pos) | \
                       (((uint32_t)sDate->Month) << RTC_DR_MU_Pos) | \
                       ((uint32_t)sDate->Date) | \
                       (((uint32_t)sDate->WeekDay) << RTC_DR_WD_Pos));
    return datetmpreg;
}

/**
  * @brief  Sets RTC current date.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sDate Pointer to date structure
  * @param  Format specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format)
{
    uint32_t datetmpreg = 0U;
    HAL_StatusTypeDef r = HAL_ERROR;

    /* Process Locked */
    __HAL_LOCK(hrtc);
    hrtc->State = HAL_RTC_STATE_BUSY;

    datetmpreg = date_2_reg(sDate, Format);

    if (RTC_EnterInitMode(hrtc) != HAL_OK)     /* Set Initialization mode */
        hrtc->State = HAL_RTC_STATE_ERROR;      /* Set RTC state*/
    else
    {
        hrtc->Instance->DR = (uint32_t)(datetmpreg & RTC_DR_RESERVED_MASK); /* Set the RTC_DR register */
        hrtc->Instance->ISR &= (uint32_t)~RTC_ISR_INIT;  /* Exit Initialization mode */

        /* If  CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
        if ((hrtc->Instance->CR & RTC_CR_BYPSHAD) == RESET && HAL_RTC_WaitForSynchro(hrtc) != HAL_OK)
            hrtc->State = HAL_RTC_STATE_ERROR;
        else
        {
            hrtc->State = HAL_RTC_STATE_READY ;
            r = HAL_OK;
        }
    }
    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);
    return r;
}

__HAL_ROM_USED void reg_2_date(uint32_t reg, RTC_DateTypeDef *sDate, uint32_t Format)
{
    /* Fill the structure fields with the read parameters */
    sDate->Year = (uint8_t)((reg & (RTC_DR_YT | RTC_DR_YU)) >> RTC_DR_YU_Pos);
    sDate->Month = (uint8_t)((reg & (RTC_DR_MT | RTC_DR_MU)) >> RTC_DR_MU_Pos);
    sDate->Date = (uint8_t)(reg & (RTC_DR_DT | RTC_DR_DU));
    sDate->WeekDay = (uint8_t)((reg & (RTC_DR_WD)) >> RTC_DR_WD_Pos);

    /* Check the input parameters format */
    if (Format == RTC_FORMAT_BIN)
    {
        /* Convert the date structure parameters to Binary format */
        sDate->Year = (uint8_t)RTC_Bcd2ToByte(sDate->Year);
        sDate->Month = (uint8_t)RTC_Bcd2ToByte(sDate->Month);
        sDate->Date = (uint8_t)RTC_Bcd2ToByte(sDate->Date);
    }
}

/**
  * @brief  Gets RTC current date.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sDate Pointer to Date structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN:  Binary data format
  *            @arg RTC_FORMAT_BCD:  BCD data format
  * @note You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock the values
  * in the higher-order calendar shadow registers to ensure consistency between the time and date values.
  * Reading RTC current time locks the values in calendar shadow registers until Current date is read.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format)
{
    uint32_t datetmpreg = 0U;

    /* Get the DR register */
#ifdef SOC_BF0_HCPU
    datetmpreg = (uint32_t)(hwp_hpsys_cfg->RTC_DR & RTC_DR_RESERVED_MASK);
    if (hwp_hpsys_cfg->RTC_DR & RTC_DR_ERR)
        return HAL_ERROR;
#else
    datetmpreg = (uint32_t)(hwp_lpsys_cfg->RTC_DR & RTC_DR_RESERVED_MASK);
    if (hwp_lpsys_cfg->RTC_DR & RTC_DR_ERR)
        return HAL_ERROR;
#endif

    /* Fill the structure fields with the read parameters */
    reg_2_date(datetmpreg, sDate, Format);
    if (datetmpreg & RTC_DR_CB)
    {
        if (sDate->Year < 70)
            hrtc->Instance->DR &= ~(RTC_DR_CB);
        else
            sDate->Year |= RTC_CENTURY_BIT;
    }
    return HAL_OK;
}

/**
  * @brief  Increase/Decrease RTC by 1 second..
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  increase >0: increase, <0:decrease, 0 no change
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_IncOneSecond(RTC_HandleTypeDef *hrtc, int32_t increase)
{
    HAL_StatusTypeDef r = HAL_OK;

#ifndef SF32LB55X
    if (increase)
    {
        if (hrtc->Instance->ISR & RTC_ISR_SHPF)
            r = HAL_BUSY;
        else
        {
            if (increase > 0)
                hrtc->Instance->SHIFTR = RTC_SHIFTR_ADD1S;
            else
                hrtc->Instance->SHIFTR = (hrtc->Instance->PSCLR & RTC_PSCLR_DIVB_Msk);
        }
    }
#else
    r = HAL_ERROR;
#endif
    return r;
}


/**
  * @} RTC_Exported_Functions_Group2
  */


/** @addtogroup RTC_Exported_Functions_Group3
 *  @brief   RTC Alarm functions
 *
@verbatim
 ===============================================================================
                 ##### RTC Alarm functions #####
 ===============================================================================

 [..] This section provides functions allowing to configure Alarm feature

@endverbatim
  * @{
  */

/**
  * @brief  Sets the specified RTC Alarm.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sAlarm Pointer to Alarm structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *             @arg RTC_FORMAT_BIN: Binary data format
  *             @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format)
{


    /* Process Locked */
    __HAL_LOCK(hrtc);

    hrtc->State = HAL_RTC_STATE_BUSY;

    while (!(__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_ISR_ALRMWF)))
    {
        __HAL_RTC_ALARM_DISABLE(hrtc);
    }

    if ((hrtc->Instance->CR & RTC_CR_FMT) == (uint32_t)RESET)
        sAlarm->AlarmTime.TimeFormat = 0x00U;
    hrtc->Instance->ALRMTR = time_2_reg(&(sAlarm->AlarmTime), Format) ;
    /*Ignore Year in Alarm*/
    hrtc->Instance->ALRMDR = date_2_reg(&(sAlarm->AlarmDate), Format) & 0xFFFF;
    hrtc->Instance->ALRMDR |= (sAlarm->AlarmMask | sAlarm->AlarmSubSecondMask);

    NVIC_EnableIRQ(RTC_IRQn);
    __HAL_RTC_ALARM_ENABLE_IT(hrtc);
    __HAL_RTC_ALARM_ENABLE(hrtc);

    /* Change RTC state */
    hrtc->State = HAL_RTC_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);

    return HAL_OK;
}

/**
  * @brief  Deactivate the specified RTC Alarm
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc)
{
    uint32_t tickstart = 0U;

    /* Process Locked */
    __HAL_LOCK(hrtc);
    hrtc->State = HAL_RTC_STATE_BUSY;

    __HAL_RTC_ALARM_DISABLE(hrtc);
    /* In case of interrupt mode is used, the interrupt source must disabled */
    __HAL_RTC_ALARM_DISABLE_IT(hrtc);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait till RTC ALRxWF flag is set and if Time out is reached exit */
    while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_ISR_ALRMWF) == RESET)
    {
        if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
        {
            hrtc->State = HAL_RTC_STATE_TIMEOUT;
            /* Process Unlocked */
            __HAL_UNLOCK(hrtc);

            return HAL_TIMEOUT;
        }
    }

    hrtc->State = HAL_RTC_STATE_READY;
    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);

    return HAL_OK;
}

/**
  * @brief  Gets the RTC Alarm value and masks.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  sAlarm Pointer to Date structure
  * @param  Format Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *             @arg RTC_FORMAT_BIN: Binary data format
  *             @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format)
{
    uint32_t tmpreg = 0U;

    tmpreg = (uint32_t)(hrtc->Instance->ALRMTR);
    reg_2_time(tmpreg, &(sAlarm->AlarmTime), Format);
    sAlarm->AlarmTime.SubSeconds = tmpreg & RTC_ALRMTR_SS;

    return HAL_OK;
}

/**
  * @brief  This function handles Alarm interrupt request.
  * @retval None
  */
__HAL_ROM_USED void HAL_RTC_IRQHandler(RTC_HandleTypeDef *hrtc)
{
    if (hrtc == NULL)
        return;
    if (hrtc->callback)
    {
        int reason;

        if (__HAL_RTC_ALARM_GET_IT(hrtc))
        {
            reason = RTC_CBK_ALARM;
            __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_ISR_ALRMF);
        }
        else if (__HAL_RTC_WAKEUPTIMER_GET_IT(hrtc))
        {
            reason = RTC_CBK_WAKEUP;
            __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_ISR_WUTF);
        }
        else
        {
            reason = -1;
        }
        (*(hrtc->callback))(reason);
    }
    else if (__HAL_RTC_ALARM_GET_IT(hrtc))
    {
        HAL_RTC_AlarmAEventCallback(hrtc);
        /* Clear the Alarm interrupt pending bit */
        __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_ISR_ALRMF);
    }
    else if (__HAL_RTC_WAKEUPTIMER_GET_IT(hrtc))
    {
        HAL_RTC_WakeupTimerEventCallback(hrtc);
        /* Clear the Wakeuptimer pending bit */
        __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_ISR_WUTF);
    }
    /* Change RTC state */
    hrtc->State = HAL_RTC_STATE_READY;
}

/**
  * @brief  Alarm A callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    /* Prevent unused argument(s) compilation warning */

    UNUSED(hrtc);
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_RTC_AlarmAEventCallback could be implemented in the user file
    */
}

/**
  * @brief  This function handles AlarmA Polling request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout)
{
    uint32_t tickstart = 0U;

    /* Get tick */
    tickstart = HAL_GetTick();

    while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_ISR_ALRMF) == RESET)
    {
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0U) || ((HAL_GetTick() - tickstart) > Timeout))
            {
                hrtc->State = HAL_RTC_STATE_TIMEOUT;
                return HAL_TIMEOUT;
            }
        }
    }

    /* Clear the Alarm interrupt pending bit */
    __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_ISR_ALRMF);

    /* Change RTC state */
    hrtc->State = HAL_RTC_STATE_READY;

    return HAL_OK;
}

/**
  * @} RTC_Exported_Functions_Group3
  */

/** @defgroup RTC_Exported_Functions_Group4 Peripheral Control functions
 *  @brief   Peripheral Control functions
 *
@verbatim
 ===============================================================================
                     ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Wait for RTC Time and Date Synchronization

@endverbatim
  * @{
  */

/**
  * @brief  Waits until the RTC Time and Date registers (RTC_TR and RTC_DR) are
  *         synchronized with RTC APB clock.
  * @note   To read the calendar through the shadow registers after Calendar
  *         initialization, calendar update or after wake-up from low power modes
  *         the software must first clear the RSF flag.
  *         The software must then wait until it is set again before reading
  *         the calendar, which means that the calendar registers have been
  *         correctly copied into the RTC_TR and RTC_DR shadow registers.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_WaitForSynchro(RTC_HandleTypeDef *hrtc)
{
    uint32_t tickstart = 0U;
    HAL_StatusTypeDef r = HAL_OK;

    hrtc->Instance->ISR &= (uint32_t)RTC_RSF_MASK;   /* Clear RSF flag */
    tickstart = HAL_GetTick();                       /* Get tick */
    while ((hrtc->Instance->ISR & RTC_ISR_RSF) == (uint32_t)RESET)   /* Wait the registers to be synchronised */
        if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
        {
            r = HAL_TIMEOUT;
            break;
        }
    return r;
}

/**
  * @} RTC_Exported_Functions_Group4
  */

/** @defgroup RTC_Exported_Functions_Group5 Peripheral State functions
 *  @brief   Peripheral State functions
 *
@verbatim
 ===============================================================================
                     ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Get RTC state

@endverbatim
  * @{
  */
/**
  * @brief  Returns the RTC state.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL state
  */
__HAL_ROM_USED HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc)
{
    return hrtc->State;
}


/**
  * @brief  Set the RTC backup register.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  idx index of backup register
  * @param  value value to set into backup register
  * @retval None
  */
__HAL_ROM_USED void HAL_RTC_set_backup(RTC_HandleTypeDef *hrtc, uint8_t idx, uint32_t value)
{
    volatile uint32_t *p = &hrtc->Instance->BKP0R;
    *(p + idx) = value;
}

/**
  * @brief  get the RTC backup register value.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  idx index of backup register
  * @retval value saved in the backup register
  */
__HAL_ROM_USED uint32_t HAL_RTC_get_backup(RTC_HandleTypeDef *hrtc, uint8_t idx)
{
    volatile uint32_t *p = &hrtc->Instance->BKP0R;
    return *(p + idx);
}


#ifndef SF32LB55X
HAL_RAM_RET_CODE_SECT(HAL_PBR_ConfigMode,  __HAL_ROM_USED HAL_StatusTypeDef HAL_PBR_ConfigMode(uint8_t pin, bool output_en))
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    __IO uint32_t *pbr;

    if (pin > HAL_PBR_MAX)
    {
        goto __EXIT;
    }

    pbr = &hwp_rtc->PBR0R;
    if (output_en)
    {
        MODIFY_REG(pbr[pin], RTC_PBR0R_OE_Msk | RTC_PBR0R_IE_Msk, RTC_PBR0R_OE_Msk | RTC_PBR0R_IE_Msk);
    }
    else
    {
        MODIFY_REG(pbr[pin], RTC_PBR0R_OE_Msk | RTC_PBR0R_IE_Msk, RTC_PBR0R_IE_Msk);
    }

    ret = HAL_OK;

__EXIT:
    return ret;
}

HAL_RAM_RET_CODE_SECT(HAL_PBR_ReadPin,  __HAL_ROM_USED int8_t HAL_PBR_ReadPin(uint8_t pin))
{
    int8_t val = -1;
    __IO uint32_t *pbr;

    if (pin > HAL_PBR_MAX)
    {
        goto __EXIT;
    }

    pbr = &hwp_rtc->PBR0R;

    val = GET_REG_VAL(pbr[pin], RTC_PBR0R_IN_Msk, RTC_PBR0R_IN_Pos);

__EXIT:
    return val;
}

HAL_RAM_RET_CODE_SECT(HAL_PBR_WritePin, __HAL_ROM_USED HAL_StatusTypeDef HAL_PBR_WritePin(uint8_t pin, uint8_t state))
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    __IO uint32_t *pbr;

    if (pin > HAL_PBR_MAX)
    {
        goto __EXIT;
    }

    pbr = &hwp_rtc->PBR0R;
    if (RTC_PBR0R_OE != (pbr[pin] & RTC_PBR0R_OE_Msk))
    {
        goto __EXIT;
    }
    MODIFY_REG(pbr[pin], RTC_PBR0R_OUT_Msk, MAKE_REG_VAL(state, RTC_PBR0R_OUT_Msk, RTC_PBR0R_OUT_Pos));

    ret = HAL_OK;

__EXIT:
    return ret;
}

HAL_RAM_RET_CODE_SECT(HAL_PBR_GetMode,  __HAL_ROM_USED HAL_StatusTypeDef HAL_PBR_GetMode(uint8_t pin, bool *output_en))
{
    HAL_StatusTypeDef ret = HAL_ERROR;
    __IO uint32_t *pbr;

    if (!output_en)
    {
        goto __EXIT;
    }

    if (pin > HAL_PBR_MAX)
    {
        goto __EXIT;
    }

    pbr = &hwp_rtc->PBR0R;
    if (pbr[pin] & RTC_PBR0R_OE)
    {
        *output_en = true;
    }
    else
    {
        *output_en = false;
    }

    ret = HAL_OK;

__EXIT:
    return ret;
}

#endif /* SF32LB55X */


/**
  * @} RTC_Exported_Functions_Group5
  */

/**
  * @} RTC_Exported_Functions_Group
  */


/** @addtogroup RTC_Private_Functions
  * @{
  */

/**
  * @brief  Enters the RTC Initialization mode.
  * @note   The RTC Initialization mode is write protected, use the
  *         __HAL_RTC_WRITEPROTECTION_DISABLE() before calling this function.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef *hrtc)
{
    uint32_t tickstart = 0U;
    HAL_StatusTypeDef r = HAL_OK;

    hrtc->Instance->ISR |= RTC_ISR_INIT;          // set to enter init mode
    tickstart = HAL_GetTick();

    /* Wait till RTC is in INIT state and if Time out is reached exit */
    while ((hrtc->Instance->ISR & RTC_ISR_INITF) == (uint32_t)RESET)
        if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
        {
            r = HAL_TIMEOUT;
            break;
        }

    return r;
}


/**
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value Byte to be converted
  * @retval Converted byte
  */
__HAL_ROM_USED uint8_t RTC_ByteToBcd2(uint8_t Value)
{
    uint32_t bcdhigh = 0U;

    while (Value >= 10U)
    {
        bcdhigh++;
        Value -= 10U;
    }

    return ((uint8_t)(bcdhigh << 4U) | Value);
}

/**
  * @brief  Converts from 2 digit BCD to Binary.
  * @param  Value BCD value to be converted
  * @retval Converted word
  */
__HAL_ROM_USED uint8_t RTC_Bcd2ToByte(uint8_t Value)
{
    uint32_t tmp = 0U;
    tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
    return (tmp + (Value & (uint8_t)0x0F));
}


/**
  * @brief  Sets wake up timer with interrupt
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  WakeUpCounter Wake up counter
  * @param  WakeUpClock Wake up clock
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_RTC_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock)
{
    uint32_t tickstart = 0U;

    /* Process Locked */
    __HAL_LOCK(hrtc);

    hrtc->State = HAL_RTC_STATE_BUSY;

    /*Check RTC WUTWF flag is reset only when wake up timer enabled*/
    if ((hrtc->Instance->CR & RTC_CR_WUTE) != RESET)
    {
        tickstart = HAL_GetTick();

        /* Wait till RTC WUTWF flag is reset and if Time out is reached exit */
        while (__HAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTWF) == SET)
        {
            if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
            {
                hrtc->State = HAL_RTC_STATE_TIMEOUT;

                /* Process Unlocked */
                __HAL_UNLOCK(hrtc);

                return HAL_TIMEOUT;
            }
        }
    }

    __HAL_RTC_WAKEUPTIMER_DISABLE(hrtc);

    tickstart = HAL_GetTick();

    /* Wait till RTC WUTWF flag is set and if Time out is reached exit */
    while (__HAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTWF) == RESET)
    {
        if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
        {
            hrtc->State = HAL_RTC_STATE_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(hrtc);

            return HAL_TIMEOUT;
        }
    }

    HAL_ASSERT(WakeUpClock == RTC_WAKEUP_SEC || WakeUpClock == RTC_WAKEUP_SUBSEC);
    MODIFY_REG(hrtc->Instance->CR, RTC_CR_WUCKSEL_Msk, MAKE_REG_VAL(WakeUpClock, RTC_CR_WUCKSEL_Msk, RTC_CR_WUCKSEL_Pos));

    /* Configure the Wake-up Timer counter */
    hrtc->Instance->WUTR = (uint32_t)(WakeUpCounter & RTC_WUTR_WUT);

    /* Enable the Wake-up Timer */
    NVIC_EnableIRQ(RTC_IRQn);
    __HAL_RTC_WAKEUPTIMER_ENABLE_IT(hrtc);
    __HAL_RTC_WAKEUPTIMER_ENABLE(hrtc);


    hrtc->State = HAL_RTC_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);

    return HAL_OK;
}

/**
  * @brief  Deactivates wake up timer counter.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
__HAL_ROM_USED uint32_t HAL_RTC_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc)
{

    uint32_t tickstart = 0U;

    /* Process Locked */
    __HAL_LOCK(hrtc);

    hrtc->State = HAL_RTC_STATE_BUSY;

    /* Disable the Wake-up Timer */
    __HAL_RTC_WAKEUPTIMER_DISABLE(hrtc);

    /* In case of interrupt mode is used, the interrupt source must disabled */
    __HAL_RTC_WAKEUPTIMER_DISABLE_IT(hrtc);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait till RTC WUTWF flag is set and if Time out is reached exit */
    while (__HAL_RTC_WAKEUPTIMER_GET_FLAG(hrtc, RTC_FLAG_WUTWF) == RESET)
    {
        if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE)
        {
            hrtc->State = HAL_RTC_STATE_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(hrtc);

            return HAL_TIMEOUT;
        }
    }

    hrtc->State = HAL_RTC_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);

    return HAL_OK;


}


/**
  * @brief  Gets wake up timer counter.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval Counter value
  */
__HAL_ROM_USED uint32_t HAL_RTC_GetWakeUpTimer(RTC_HandleTypeDef *hrtc)
{
    /* Get the counter value */
    return ((uint32_t)(hrtc->Instance->WUTR & RTC_WUTR_WUT));
}



/**
  * @brief  Wakeup Timer callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
__weak void HAL_RTC_WakeupTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    /* Prevent unused argument(s) compilation warning */

    UNUSED(hrtc);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_RTC_WakeUpTimerEventCallback could be implemented in the user file
     */

}


/**
  * @brief  Register call back functions for RTC module.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  cbk callback function to be registered.
  * @retval None
  */
__HAL_ROM_USED void HAL_RTC_RegCallback(RTC_HandleTypeDef *hrtc, RTC_cb cbk)
{
    if (hrtc)
        hrtc->callback = cbk;
}


/**
  * @} RTC_Private_Functions
  */

#endif /* HAL_RTC_MODULE_ENABLED */
/**
  * @} RTC
  */

/**
  * @} BF0_HAL_Driver
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
