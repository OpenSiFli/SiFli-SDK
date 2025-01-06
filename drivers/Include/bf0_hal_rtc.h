/**
  ******************************************************************************
  * @file   bf0_hal_rtc.h
  * @author Sifli software development team
  * @brief  HAL State structures definition
  *
  ******************************************************************************
*/
/*
 * @attention
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

#ifndef __BF0_HAL_RTC_H
#define __BF0_HAL_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */
#ifndef RC10K_FREQ
#define RC10K_FREQ 9000
#endif

#ifndef RC32K_FREQ
#define RC32K_FREQ 26665
#endif

#ifndef LXT_FREQ
#define LXT_FREQ 32768
#endif

#define RC10K_SUB_SEC_DIVB 256   /*!< RTC DIVB default setting */

/** @addtogroup RTC Real timer clock
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup RTC_Exported_Types RTC Exported Types
  * @{
  */

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
    HAL_RTC_STATE_RESET             = 0x00U,  /*!< RTC not yet initialized or disabled */
    HAL_RTC_STATE_READY             = 0x01U,  /*!< RTC initialized and ready for use   */
    HAL_RTC_STATE_BUSY              = 0x02U,  /*!< RTC process is ongoing              */
    HAL_RTC_STATE_TIMEOUT           = 0x03U,  /*!< RTC timeout state                   */
    HAL_RTC_STATE_ERROR             = 0x04U   /*!< RTC error state                     */
} HAL_RTCStateTypeDef;

/**
  * @brief  RTC Configuration Structure definition
  */
typedef struct
{
    uint32_t HourFormat;      /*!< Specifies the RTC Hour Format.
                                 This parameter can be a value of @ref RTC_Hour_Formats */

    uint32_t DivAInt;    /*!< Specifies the RTC Divider A integrate value.
                                  This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x80 */

    uint32_t DivAFrac;   /*!< Specifies the RTC Divider A fraction value. */

    uint32_t DivB;     /*!< Specifies the RTC Divider B value.
                                  This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x100U */
} RTC_InitTypeDef;

/**
  * @brief  RTC Time structure definition
  */
typedef struct
{
    uint8_t Hours;            /*!< Specifies the RTC Time Hour.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 12 if the RTC_HourFormat_12 is selected.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 23 if the RTC_HourFormat_24 is selected  */

    uint8_t Minutes;          /*!< Specifies the RTC Time Minutes.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

    uint8_t Seconds;          /*!< Specifies the RTC Time Seconds.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

    uint8_t TimeFormat;       /*!< Specifies the RTC AM/PM Time.
                                 This parameter can be a value of @ref RTC_AM_PM_Definitions */

    uint32_t SubSeconds;     /*!< Specifies the RTC_SSR RTC Sub Second register content.
                                 This parameter corresponds to a time unit range between [0-1] Second
                                 with [1 Sec / SecondFraction] granularity */

    uint32_t SecondFractionInt;  /*!< Specifies the range or granularity of Sub Second register content
                                 corresponding to Synchronous DivA integrade factor value
                                 This parameter corresponds to SecondFraction integrade part. SecondFraction is
                                 calcualted by [Fclk << 2^14 / (SecondFractionInt << 2^14) + SecondFractionFrac].
                                 This field will be used only by HAL_RTC_GetTime function */
    uint32_t SecondFractionFrac;  /*!< Specifies the range or granularity of Sub Second register content
                                 corresponding to Synchronous DivA fraction factor value
                                 This parameter corresponds to SecondFraction fraction part. SecondFraction is
                                 calcualted by [Fclk << 2^14 / (SecondFractionInt << 2^14) + SecondFractionFrac].
                                 This field will be used only by HAL_RTC_GetTime function */
    uint32_t DayLightSaving;  /*!< Specifies DayLight Save Operation.
                                 This parameter can be a value of @ref RTC_DayLightSaving_Definitions */

    uint32_t StoreOperation;  /*!< Specifies RTC_StoreOperation value to be written in the BCK bit
                                 in CR register to store the operation.
                                 This parameter can be a value of @ref RTC_StoreOperation_Definitions */
} RTC_TimeTypeDef;

/**
  * @brief  RTC Date structure definition
  */
typedef struct
{
    uint8_t WeekDay;  /*!< Specifies the RTC Date WeekDay.
                         This parameter can be a value of @ref RTC_WeekDay_Definitions */

    uint8_t Month;    /*!< Specifies the RTC Date Month (in BCD format).
                         This parameter can be a value of @ref RTC_Month_Date_Definitions */

    uint8_t Date;     /*!< Specifies the RTC Date.
                         This parameter must be a number between Min_Data = 1 and Max_Data = 31 */
#define RTC_CENTURY_BIT 0x80
    uint8_t Year;     /*!< Specifies the RTC Date Year as offset of 1900.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 199
                         if Year<70, The year will be 2000+Year, RTC_CENTURY_BIT will be set
                         Other wise, The year will be 1900+Year, RTC_CENTURY_BIT will be clear
                    */

} RTC_DateTypeDef;

/**
  * @brief  RTC Alarm structure definition
  */
typedef struct
{
    RTC_TimeTypeDef AlarmTime;     /*!< Specifies the RTC Alarm Time members */
    RTC_DateTypeDef AlarmDate;     /*!< Specifies the RTC Alarm date members */

    uint32_t AlarmMask;            /*!< Specifies the RTC Alarm Masks.
                                      This parameter can be a value of @ref RTC_AlarmMask_Definitions */

    uint32_t AlarmSubSecondMask;   /*!< Specifies the RTC Alarm SubSeconds Masks.
                                      This parameter can be a value of @ref RTC_Alarm_Sub_Seconds_Masks_Definitions */

    uint32_t AlarmDateWeekDaySel;  /*!< Specifies the RTC Alarm is on Date or WeekDay.
                                     This parameter can be a value of @ref RTC_AlarmDateWeekDay_Definitions */

    uint8_t AlarmDateWeekDay;      /*!< Specifies the RTC Alarm Date/WeekDay.
                                      If the Alarm Date is selected, this parameter must be set to a value in the 1-31 range.
                                      If the Alarm WeekDay is selected, this parameter can be a value of @ref RTC_WeekDay_Definitions */

} RTC_AlarmTypeDef;

#define RTC_CBK_ALARM           0
#define RTC_CBK_WAKEUP          1
/**
 ****************************************************************************************
 * @brief Call back definition of RTC module
 *
 * @param[in] reason       Callback reason, should be RTC_CBK_XXX, -1 if unknown reason.
 ****************************************************************************************
 */
typedef void (*RTC_cb)(int reason);

/**
  * @brief  RTC Handle Structure definition
  */

typedef struct
{
    RTC_TypeDef                 *Instance;  /*!< Register base address    */

    RTC_InitTypeDef             Init;       /*!< RTC required parameters  */

    HAL_LockTypeDef             Lock;       /*!< RTC locking object       */

    __IO HAL_RTCStateTypeDef    State;      /*!< Time communication state */

    RTC_cb                      callback;   /*!< RTC callback*/

} RTC_HandleTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RTC_Exported_Constants RTC Exported Constants
  * @{
  */

/** @defgroup RTC_Hour_Formats RTC Hour Formats
  * @{
  */
#define RTC_HOURFORMAT_24              0x00000000U
#define RTC_HOURFORMAT_12              0x00000040U
/**
  * @}
  */

/** @defgroup RTC_AM_PM_Definitions RTC AM PM Definitions
  * @{
  */
#define RTC_HOURFORMAT12_AM            ((uint8_t)0x00)
#define RTC_HOURFORMAT12_PM            ((uint8_t)0x40)
/**
  * @}
  */

/** @defgroup RTC_DayLightSaving_Definitions RTC DayLight Saving Definitions
  * @{
  */
#define RTC_DAYLIGHTSAVING_SUB1H       0x00020000U
#define RTC_DAYLIGHTSAVING_ADD1H       0x00010000U
#define RTC_DAYLIGHTSAVING_NONE        0x00000000U
/**
  * @}
  */

/** @defgroup RTC_StoreOperation_Definitions RTC Store Operation Definitions
  * @{
  */
#define RTC_STOREOPERATION_RESET        0x00000000U
#define RTC_STOREOPERATION_SET          0x00040000U
/**
  * @}
  */

/** @defgroup RTC_Input_parameter_format_definitions RTC Input Parameter Format Definitions
  * @{
  */
#define RTC_FORMAT_BIN                      0x00000000U
#define RTC_FORMAT_BCD                      0x00000001U
/**
  * @}
  */

/** @defgroup RTC_Month_Date_Definitions RTC Month Date Definitions
  * @{
  */
/* Coded in BCD format */
#define RTC_MONTH_JANUARY              ((uint8_t)0x01)
#define RTC_MONTH_FEBRUARY             ((uint8_t)0x02)
#define RTC_MONTH_MARCH                ((uint8_t)0x03)
#define RTC_MONTH_APRIL                ((uint8_t)0x04)
#define RTC_MONTH_MAY                  ((uint8_t)0x05)
#define RTC_MONTH_JUNE                 ((uint8_t)0x06)
#define RTC_MONTH_JULY                 ((uint8_t)0x07)
#define RTC_MONTH_AUGUST               ((uint8_t)0x08)
#define RTC_MONTH_SEPTEMBER            ((uint8_t)0x09)
#define RTC_MONTH_OCTOBER              ((uint8_t)0x10)
#define RTC_MONTH_NOVEMBER             ((uint8_t)0x11)
#define RTC_MONTH_DECEMBER             ((uint8_t)0x12)
/**
  * @}
  */

/** @defgroup RTC_WeekDay_Definitions RTC WeekDay Definitions
  * @{
  */
#define RTC_WEEKDAY_MONDAY             ((uint8_t)0x01)
#define RTC_WEEKDAY_TUESDAY            ((uint8_t)0x02)
#define RTC_WEEKDAY_WEDNESDAY          ((uint8_t)0x03)
#define RTC_WEEKDAY_THURSDAY           ((uint8_t)0x04)
#define RTC_WEEKDAY_FRIDAY             ((uint8_t)0x05)
#define RTC_WEEKDAY_SATURDAY           ((uint8_t)0x06)
#define RTC_WEEKDAY_SUNDAY             ((uint8_t)0x07)
/**
  * @}
  */

/** @defgroup RTC_AlarmDateWeekDay_Definitions RTC Alarm Date WeekDay Definitions
  * @{
  */
#define RTC_ALARMDATEWEEKDAYSEL_DATE      0x00000000U
#define RTC_ALARMDATEWEEKDAYSEL_WEEKDAY   0x40000000U
/**
  * @}
  */

/** @defgroup RTC_AlarmMask_Definitions RTC Alarm Mask Definitions
  * @{
  */
#define RTC_ALARMMASK_NONE                0x00000000U
#define RTC_ALARMMASK_DATEWEEKDAY         RTC_ALRMDR_MSKWD
#define RTC_ALARMMASK_MONTH               RTC_ALRMDR_MSKM
#define RTC_ALARMMASK_DAY                 RTC_ALRMDR_MSKD
#define RTC_ALARMMASK_SECONDS             RTC_ALRMDR_MSKS
#define RTC_ALARMMASK_HOURS               RTC_ALRMDR_MSKH
#define RTC_ALARMMASK_MINUTES             RTC_ALRMDR_MSKMN
#define RTC_ALARMMASK_ALL                 0x3FF00000U
/**
  * @}
  */

/** @defgroup RTC_Alarm_Sub_Seconds_Masks_Definitions RTC Alarm Sub Seconds Masks Definitions
  * @{
  */
#define RTC_ALARMSUBSECONDMASK_ALL         0x00000000U  /*!< All Alarm SS fields are masked. 
                                                                        There is no comparison on sub seconds 
                                                                        for Alarm */
#define RTC_ALARMSUBSECONDMASK_SS14_1      0x01000000U  /*!< SS[14:1] are don't care in Alarm 
                                                                        comparison. Only SS[0] is compared.    */
#define RTC_ALARMSUBSECONDMASK_SS14_2      0x02000000U  /*!< SS[14:2] are don't care in Alarm 
                                                                        comparison. Only SS[1:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_3      0x03000000U  /*!< SS[14:3] are don't care in Alarm 
                                                                        comparison. Only SS[2:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_4      0x04000000U  /*!< SS[14:4] are don't care in Alarm 
                                                                        comparison. Only SS[3:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_5      0x05000000U  /*!< SS[14:5] are don't care in Alarm 
                                                                        comparison. Only SS[4:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_6      0x06000000U  /*!< SS[14:6] are don't care in Alarm 
                                                                        comparison. Only SS[5:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_7      0x07000000U  /*!< SS[14:7] are don't care in Alarm 
                                                                        comparison. Only SS[6:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_8      0x08000000U  /*!< SS[14:8] are don't care in Alarm 
                                                                        comparison. Only SS[7:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_9      0x09000000U  /*!< SS[14:9] are don't care in Alarm 
                                                                        comparison. Only SS[8:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_10     0x0A000000U  /*!< SS[14:10] are don't care in Alarm 
                                                                        comparison. Only SS[9:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_11     0x0B000000U  /*!< SS[14:11] are don't care in Alarm 
                                                                        comparison. Only SS[10:0] are compared */
#define RTC_ALARMSUBSECONDMASK_SS14_12     0x0C000000U  /*!< SS[14:12] are don't care in Alarm 
                                                                        comparison.Only SS[11:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_SS14_13     0x0D000000U  /*!< SS[14:13] are don't care in Alarm 
                                                                        comparison. Only SS[12:0] are compared */
#define RTC_ALARMSUBSECONDMASK_SS14        0x0E000000U  /*!< SS[14] is don't care in Alarm 
                                                                        comparison.Only SS[13:0] are compared  */
#define RTC_ALARMSUBSECONDMASK_NONE        0x0F000000U  /*!< SS[14:0] are compared and must match 
                                                                        to activate alarm. */
/**
  * @}
  */

/** @defgroup RTC_Interrupts_Definitions RTC Interrupts Definitions
  * @{
  */
#define RTC_IT_TS                         0x00008000U
#define RTC_IT_WUT                        0x00004000U
#define RTC_IT_ALRB                       0x00002000U
#define RTC_IT_ALRA                       0x00001000U
#define RTC_IT_TAMP                       0x00000004U /* Used only to Enable the Tamper Interrupt */
#define RTC_IT_TAMP1                      0x00020000U
#define RTC_IT_TAMP2                      0x00040000U
/**
  * @}
  */

/** @defgroup RTC_Flags_Definitions RTC Flags Definitions
  * @{
  */
#define RTC_FLAG_RECALPF                  0x00010000U
#define RTC_FLAG_TAMP2F                   0x00004000U
#define RTC_FLAG_TAMP1F                   0x00002000U
#define RTC_FLAG_TSOVF                    0x00001000U
#define RTC_FLAG_TSF                      0x00000800U
#define RTC_FLAG_WUTF                     0x00000400U
#define RTC_FLAG_ALRBF                    0x00000200U
#define RTC_FLAG_ALRAF                    0x00000100U
#define RTC_FLAG_INITF                    0x00000040U
#define RTC_FLAG_RSF                      0x00000020U
#define RTC_FLAG_INITS                    0x00000010U
#define RTC_FLAG_SHPF                     0x00000008U
#define RTC_FLAG_WUTWF                    0x00000004U
#define RTC_FLAG_ALRBWF                   0x00000002U
#define RTC_FLAG_ALRAWF                   0x00000001U
/**
  * @}
  */

/** @defgroup RTC_Wakeup_Clock RTC wakeup clock selection
  * @{
  */
#define RTC_WAKEUP_SEC                    0x00000000U           // From Prescalar B
#define RTC_WAKEUP_SUBSEC                 0x00000001U           // From Prescalar A
/**
  * @}
  */


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RTC_Exported_Macros RTC Exported Macros
  * @{
  */

/** @brief Reset RTC handle state
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_RTC_STATE_RESET)

/**
  * @brief  Enable the RTC ALARM peripheral.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_ALARM_ENABLE(__HANDLE__)                           ((__HANDLE__)->Instance->CR |= (RTC_CR_ALRME))

/**
  * @brief  Disable the RTC ALARM peripheral.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_ALARM_DISABLE(__HANDLE__)                          ((__HANDLE__)->Instance->CR &= ~(RTC_CR_ALRME))

/**
  * @brief  Enable the RTC Alarm interrupt.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_ALARM_ENABLE_IT(__HANDLE__)          ((__HANDLE__)->Instance->CR |= (RTC_CR_ALRMIE))

/**
  * @brief  Disable the RTC Alarm interrupt.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_ALARM_DISABLE_IT(__HANDLE__)         ((__HANDLE__)->Instance->CR &= ~(RTC_CR_ALRMIE))

/**
  * @brief  Check whether the specified RTC Alarm interrupt has occurred or not.
  * @param  \__HANDLE__ specifies the RTC handle.

  * @retval None
  */
#define __HAL_RTC_ALARM_GET_IT(__HANDLE__)           (((((__HANDLE__)->Instance->ISR)& RTC_ISR_ALRMF) != RESET)? SET : RESET)

/**
  * @brief  Get the selected RTC Alarm's flag status.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @param  \__FLAG__ specifies the RTC Alarm Flag to check.
  *         This parameter can be:
  *            @arg RTC_ISR_ALRMF
  *            @arg RTC_ISR_ALRMWF
  * @retval None
  */
#define __HAL_RTC_ALARM_GET_FLAG(__HANDLE__, __FLAG__)                (((((__HANDLE__)->Instance->ISR) & (__FLAG__)) != RESET)? SET : RESET)

/**
  * @brief  Clear the RTC Alarm's pending flags.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @param  \__FLAG__ specifies the RTC Alarm Flag sources to be enabled or disabled.
  *          This parameter can be:
  *             @arg RTC_FLAG_ALRAF
  *             @arg RTC_FLAG_ALRBF
  * @retval None
  */
#define __HAL_RTC_ALARM_CLEAR_FLAG(__HANDLE__, __FLAG__)                  ((__HANDLE__)->Instance->ISR) = (~((__FLAG__) | RTC_ISR_INIT)|((__HANDLE__)->Instance->ISR & RTC_ISR_INIT))


/**
  * @brief  Check whether the specified RTC Alarm interrupt has been enabled or not.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @param  \__INTERRUPT__ specifies the RTC Alarm interrupt sources to check.
  *         This parameter can be:
  *            @arg RTC_IT_ALRA: Alarm A interrupt
  *            @arg RTC_IT_ALRB: Alarm B interrupt
  * @retval None
  */
#define __HAL_RTC_ALARM_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)     (((((__HANDLE__)->Instance->CR) & (__INTERRUPT__)) != RESET) ? SET : RESET)

/**
  * @brief  Enable interrupt on the RTC Alarm associated Exti line.
  * @retval None
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_IT()            (EXTI->IMR |= RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief  Disable interrupt on the RTC Alarm associated Exti line.
  * @retval None
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_IT()           (EXTI->IMR &= ~(RTC_EXTI_LINE_ALARM_EVENT))

/**
  * @brief  Enable event on the RTC Alarm associated Exti line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_EVENT()         (EXTI->EMR |= RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief  Disable event on the RTC Alarm associated Exti line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_EVENT()         (EXTI->EMR &= ~(RTC_EXTI_LINE_ALARM_EVENT))

/**
  * @brief  Enable falling edge trigger on the RTC Alarm associated Exti line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_FALLING_EDGE()   (EXTI->FTSR |= RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief  Disable falling edge trigger on the RTC Alarm associated Exti line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_FALLING_EDGE()  (EXTI->FTSR &= ~(RTC_EXTI_LINE_ALARM_EVENT))

/**
  * @brief  Enable rising edge trigger on the RTC Alarm associated Exti line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE()    (EXTI->RTSR |= RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief  Disable rising edge trigger on the RTC Alarm associated Exti line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_RISING_EDGE()   (EXTI->RTSR &= ~(RTC_EXTI_LINE_ALARM_EVENT))

/**
  * @brief  Enable rising & falling edge trigger on the RTC Alarm associated Exti line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_ENABLE_RISING_FALLING_EDGE() do { __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE(); \
                                                               __HAL_RTC_ALARM_EXTI_ENABLE_FALLING_EDGE();\
                                                             } while(0U)

/**
  * @brief  Disable rising & falling edge trigger on the RTC Alarm associated Exti line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_DISABLE_RISING_FALLING_EDGE() do { __HAL_RTC_ALARM_EXTI_DISABLE_RISING_EDGE();\
                                                                __HAL_RTC_ALARM_EXTI_DISABLE_FALLING_EDGE();\
                                                              } while(0U)

/**
  * @brief Check whether the RTC Alarm associated Exti line interrupt flag is set or not.
  * @retval Line Status.
  */
#define __HAL_RTC_ALARM_EXTI_GET_FLAG()              (EXTI->PR & RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief Clear the RTC Alarm associated Exti line flag.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_CLEAR_FLAG()            (EXTI->PR = RTC_EXTI_LINE_ALARM_EVENT)

/**
  * @brief Generate a Software interrupt on RTC Alarm associated Exti line.
  * @retval None.
  */
#define __HAL_RTC_ALARM_EXTI_GENERATE_SWIT()         (EXTI->SWIER |= RTC_EXTI_LINE_ALARM_EVENT)

/**
* @brief  Get the selected RTC WakeUpTimer's flag status.
* @param  \__HANDLE__ specifies the RTC handle.
* @param  \__FLAG__ specifies the RTC WakeUpTimer Flag to check.
*          This parameter can be:
*             @arg RTC_FLAG_WUTF
*             @arg RTC_FLAG_WUTWF
* @retval None
*/
#define __HAL_RTC_WAKEUPTIMER_GET_FLAG(__HANDLE__, __FLAG__)                (((((__HANDLE__)->Instance->ISR) & (__FLAG__)) != RESET)? SET : RESET)

/**
  * @brief  Enable the RTC WakeUp Timer peripheral.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_WAKEUPTIMER_ENABLE(__HANDLE__)                          ((__HANDLE__)->Instance->CR |= (RTC_CR_WUTE))

/**
  * @brief  Disable the RTC Wake-up Timer peripheral.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_WAKEUPTIMER_DISABLE(__HANDLE__)                          ((__HANDLE__)->Instance->CR &= ~(RTC_CR_WUTE))


/**
  * @brief  Enable the RTC WakeUpTimer interrupt.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */

#define __HAL_RTC_WAKEUPTIMER_ENABLE_IT(__HANDLE__)          ((__HANDLE__)->Instance->CR |= (RTC_CR_WUTIE))

/**
  * @brief  Disable the RTC Alarm interrupt.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_WAKEUPTIMER_DISABLE_IT(__HANDLE__)         ((__HANDLE__)->Instance->CR &= ~(RTC_CR_WUTIE))

/**
  * @brief  Check whether the specified RTC WakeUpTimer interrupt has occurred or not.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @retval None
  */
#define __HAL_RTC_WAKEUPTIMER_GET_IT(__HANDLE__)           (((((__HANDLE__)->Instance->ISR)& RTC_ISR_WUTF) != RESET)? SET : RESET)

/**
  * @brief  Clear the RTC Wake Up timer's pending flags.
  * @param  \__HANDLE__ specifies the RTC handle.
  * @param  \__FLAG__ specifies the RTC Tamper Flag sources to be enabled or disabled.
  *         This parameter can be:
  *            @arg RTC_FLAG_WUTF
  * @retval None
  */
#define __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(__HANDLE__, __FLAG__)    ((__HANDLE__)->Instance->ISR) = (~((__FLAG__) | RTC_ISR_INIT)|((__HANDLE__)->Instance->ISR & RTC_ISR_INIT))


#ifdef hwp_pbr
#ifdef SF32LB58X
#define HAL_PBR_MAX   (5)
#elif defined SF32LB56X
#define HAL_PBR_MAX   (3)
#else
#define HAL_PBR_MAX   (3)
#endif
#endif /* hwp_pbr */

#ifdef SF32LB52X
#define HAL_RTC_LXT_ENABLED()          (hwp_rtc->CR&RTC_CR_LPCKSEL)
#define HAL_RTC_ENABLE_LXT()           hwp_rtc->CR |= RTC_CR_LPCKSEL
#else
#define HAL_RTC_LXT_ENABLED()          HAL_PMU_LXT_ENABLED()
#define HAL_RTC_ENABLE_LXT()
#endif /* SF32LB52X */

/**
  * @}
  */

/* Include RTC HAL Extension module */
#include "bf0_hal_rtc_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RTC_Exported_Functions
  * @{
  */

/** @addtogroup RTC_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions  ****************************/

#define  RTC_INIT_NORMAL    0       // Normal Init
#define  RTC_INIT_SKIP      1       // Skip RTC hardware init
#define  RTC_INIT_REINIT    2       // RTC Reinit, should not clear Alarm and wakeup timer

/**
  * @brief  Initializes the RTC peripheral
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  wakesrc wakeup source,  see RTC_INIT_XXX
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc, uint32_t wakesrc);
/**
  * @brief  DeInitializes the RTC peripheral
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @note   This function doesn't reset the RTC Backup Data registers.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
/**
  * @brief  Initializes the RTC MSP.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void       HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
/**
  * @brief  DeInitializes the RTC MSP.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void       HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group2
  * @{
  */
/* RTC Time and Date functions ************************************************/
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
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);

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
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
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
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
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
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);

/**
  * @brief  Increase/Decrease RTC by 1 second..
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  increase >0: increase, <0:decrease, 0 no change
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_IncOneSecond(RTC_HandleTypeDef *hrtc, int32_t increase);

/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group3
  * @{
  */
/* RTC Alarm functions ********************************************************/
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
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);

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
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
/**
  * @brief  Deactivate the specified RTC Alarm
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc);
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
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
void                HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);

/**
  * @brief  This function handles AlarmA Polling request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef   HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
/**
  * @brief  Alarm A callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void         HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
/**
  * @brief  Sets wake up timer with interrupt
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  WakeUpCounter Wake up counter
  * @param  WakeUpClock Wake up clock, see @ref RTC_Wakeup_Clock
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
/**
  * @brief  Deactivates wake up timer counter.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL status
  */
uint32_t HAL_RTC_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc);
/**
  * @brief  Gets wake up timer counter.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval Counter value
  */
uint32_t HAL_RTC_GetWakeUpTimer(RTC_HandleTypeDef *hrtc);
/**
  * @brief  Wakeup Timer callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void HAL_RTC_WakeupTimerEventCallback(RTC_HandleTypeDef *hrtc);
/**
  * @brief  This function handles Alarm interrupt request.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void HAL_RTC_IRQHandler(RTC_HandleTypeDef *hrtc);



/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group4
  * @{
  */
/* Peripheral Control functions ***********************************************/
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
HAL_StatusTypeDef   HAL_RTC_WaitForSynchro(RTC_HandleTypeDef *hrtc);
/**
  * @}
  */

/** @addtogroup RTC_Exported_Functions_Group5
  * @{
  */
/* Peripheral State functions *************************************************/
/**
  * @brief  Returns the RTC state.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval HAL state
  */
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);

/**
  * @brief  Set the RTC backup register.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  idx index of backup register
  * @param  value value to set into backup register
  * @retval None
  */
void HAL_RTC_set_backup(RTC_HandleTypeDef *hrtc, uint8_t idx, uint32_t value);

/**
  * @brief  get the RTC backup register value.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  idx index of backup register
  * @retval value saved in the backup register
  */
uint32_t HAL_RTC_get_backup(RTC_HandleTypeDef *hrtc, uint8_t idx);


/**
  * @brief  Register call back functions for RTC module.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @param  cbk callback function to be registered.
  * @retval None
  */
void HAL_RTC_RegCallback(RTC_HandleTypeDef *hrtc, RTC_cb cbk);

/**
  * @brief  config PBR pin mode
  * @param[in] pin        PBR pin index, starting from 0
  * @param[in] output_en  true: output mode, false: input mode
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PBR_ConfigMode(uint8_t pin, bool output_en);

/**
  * @brief  Read PBR pin state
  * @param[in] pin PBR pin index, starting from 0
   *
  * @retval 0 and 1 are valid value, if less than 0, some error happens, e.g. pin is invalid
  */
int8_t HAL_PBR_ReadPin(uint8_t pin);

/**
  * @brief  Set PBR pin state
  * @param[in] pin PBR pin index, starting from 0
  * @param[in] state pin state, 0 and 1 are valid vallue
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PBR_WritePin(uint8_t pin, uint8_t state);

/**
  * @brief  Get PBR pin mode
  * @param[in] pin PBR pin index, starting from 0
  * @param[in,out] output_en pointer to pin mode, true: output mode, false: input  mode
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PBR_GetMode(uint8_t pin, bool *output_en);


#define HAL_PBR0_FORCE1_ENABLE()   (hwp_rtc->PBR0R |= RTC_PBR0R_FORCE1)
#define HAL_PBR0_FORCE1_DISABLE()  (hwp_rtc->PBR0R &= ~RTC_PBR0R_FORCE1)

/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup RTC_Private_Constants RTC Private Constants
  * @{
  */
/* Masks Definition */
#define RTC_TR_RESERVED_MASK    0x007F7F7FU
#define RTC_DR_RESERVED_MASK    0x01FFFF3FU
#define RTC_INIT_MASK           0xFFFFFFFFU
#define RTC_RSF_MASK            0xFFFFFF5FU
#define RTC_FLAGS_MASK          ((uint32_t)(RTC_FLAG_TSOVF | RTC_FLAG_TSF | RTC_FLAG_WUTF | \
                                            RTC_FLAG_ALRBF | RTC_FLAG_ALRAF | RTC_FLAG_INITF | \
                                            RTC_FLAG_RSF | RTC_FLAG_INITS | RTC_FLAG_WUTWF | \
                                            RTC_FLAG_ALRBWF | RTC_FLAG_ALRAWF | RTC_FLAG_TAMP1F | \
                                            RTC_FLAG_RECALPF | RTC_FLAG_SHPF))

#define RTC_TIMEOUT_VALUE       1000

#define RTC_EXTI_LINE_ALARM_EVENT             ((uint32_t)EXTI_IMR_MR17)  /*!< External interrupt line 17 Connected to the RTC Alarm event */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup RTC_Private_Macros RTC Private Macros
  * @{
  */

/** @defgroup RTC_IS_RTC_Definitions RTC Private macros to check input parameters
  * @{
  */
#define IS_RTC_HOUR_FORMAT(FORMAT)     (((FORMAT) == RTC_HOURFORMAT_12) || \
                                        ((FORMAT) == RTC_HOURFORMAT_24))
#define IS_RTC_OUTPUT(OUTPUT) (((OUTPUT) == RTC_OUTPUT_DISABLE) || \
                               ((OUTPUT) == RTC_OUTPUT_ALARMA)  || \
                               ((OUTPUT) == RTC_OUTPUT_ALARMB)  || \
                               ((OUTPUT) == RTC_OUTPUT_WAKEUP))
#define IS_RTC_OUTPUT_POL(POL) (((POL) == RTC_OUTPUT_POLARITY_HIGH) || \
                                ((POL) == RTC_OUTPUT_POLARITY_LOW))
#define IS_RTC_OUTPUT_TYPE(TYPE) (((TYPE) == RTC_OUTPUT_TYPE_OPENDRAIN) || \
                                  ((TYPE) == RTC_OUTPUT_TYPE_PUSHPULL))
#define IS_RTC_HOUR12(HOUR)            (((HOUR) > 0U) && ((HOUR) <= 12U))
#define IS_RTC_HOUR24(HOUR)            ((HOUR) <= 23U)
#define IS_RTC_ASYNCH_PREDIV(PREDIV)   ((PREDIV) <= 0x7FU)
#define IS_RTC_SYNCH_PREDIV(PREDIV)    ((PREDIV) <= 0x7FFFU)
#define IS_RTC_MINUTES(MINUTES)        ((MINUTES) <= 59U)
#define IS_RTC_SECONDS(SECONDS)        ((SECONDS) <= 59U)

#define IS_RTC_HOURFORMAT12(PM)  (((PM) == RTC_HOURFORMAT12_AM) || ((PM) == RTC_HOURFORMAT12_PM))
#define IS_RTC_DAYLIGHT_SAVING(SAVE) (((SAVE) == RTC_DAYLIGHTSAVING_SUB1H) || \
                                      ((SAVE) == RTC_DAYLIGHTSAVING_ADD1H) || \
                                      ((SAVE) == RTC_DAYLIGHTSAVING_NONE))
#define IS_RTC_STORE_OPERATION(OPERATION) (((OPERATION) == RTC_STOREOPERATION_RESET) || \
                                           ((OPERATION) == RTC_STOREOPERATION_SET))
#define IS_RTC_FORMAT(FORMAT) (((FORMAT) == RTC_FORMAT_BIN) || ((FORMAT) == RTC_FORMAT_BCD))
#define IS_RTC_YEAR(YEAR)              ((YEAR) <= 99U)
#define IS_RTC_MONTH(MONTH)            (((MONTH) >= 1U) && ((MONTH) <= 12U))
#define IS_RTC_DATE(DATE)              (((DATE) >= 1U) && ((DATE) <= 31U))
#define IS_RTC_WEEKDAY(WEEKDAY) (((WEEKDAY) == RTC_WEEKDAY_MONDAY)    || \
                                 ((WEEKDAY) == RTC_WEEKDAY_TUESDAY)   || \
                                 ((WEEKDAY) == RTC_WEEKDAY_WEDNESDAY) || \
                                 ((WEEKDAY) == RTC_WEEKDAY_THURSDAY)  || \
                                 ((WEEKDAY) == RTC_WEEKDAY_FRIDAY)    || \
                                 ((WEEKDAY) == RTC_WEEKDAY_SATURDAY)  || \
                                 ((WEEKDAY) == RTC_WEEKDAY_SUNDAY))
#define IS_RTC_ALARM_DATE_WEEKDAY_DATE(DATE) (((DATE) > 0U) && ((DATE) <= 31U))
#define IS_RTC_ALARM_DATE_WEEKDAY_WEEKDAY(WEEKDAY) (((WEEKDAY) == RTC_WEEKDAY_MONDAY)    || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_TUESDAY)   || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_WEDNESDAY) || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_THURSDAY)  || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_FRIDAY)    || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_SATURDAY)  || \
                                                    ((WEEKDAY) == RTC_WEEKDAY_SUNDAY))
#define IS_RTC_ALARM_DATE_WEEKDAY_SEL(SEL) (((SEL) == RTC_ALARMDATEWEEKDAYSEL_DATE) || \
                                            ((SEL) == RTC_ALARMDATEWEEKDAYSEL_WEEKDAY))
#define IS_RTC_ALARM_MASK(MASK)  (((MASK) & 0x7F7F7F7FU) == (uint32_t)RESET)
#define IS_RTC_ALARM(ALARM)      (((ALARM) == RTC_ALARM_A) || ((ALARM) == RTC_ALARM_B))
#define IS_RTC_ALARM_SUB_SECOND_VALUE(VALUE) ((VALUE) <= 0x00007FFFU)

#define IS_RTC_ALARM_SUB_SECOND_MASK(MASK)   (((MASK) == RTC_ALARMSUBSECONDMASK_ALL) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_1) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_2) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_3) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_4) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_5) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_6) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_7) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_8) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_9) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_10) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_11) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_12) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14_13) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_SS14) || \
                                              ((MASK) == RTC_ALARMSUBSECONDMASK_NONE))
/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup RTC_Private_Functions RTC Private Functions
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
HAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef *hrtc);

/**
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value Byte to be converted
  * @retval Converted byte
  */
uint8_t            RTC_ByteToBcd2(uint8_t Value);

/**
  * @brief  Converts from 2 digit BCD to Binary.
  * @param  Value BCD value to be converted
  * @retval Converted word
  */
uint8_t            RTC_Bcd2ToByte(uint8_t Value);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_RTC_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
