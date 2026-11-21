/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BF0_HAL_WDT_H
#define __BF0_HAL_WDT_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAL_WDT_MODULE_ENABLED
#define WDT_TypeDef void
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @addtogroup WDT
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/** @defgroup WDT_Exported_Types WDT Exported Types
 * @{
 */

/**
 * @brief  WDT Init structure definition
 */
typedef struct
{

    uint32_t Reload;     /*!< Specifies the WDT down-counter reload value for WDT counter 1.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 0x0FFFFFF */
    uint32_t Reload2;     /*!< Specifies the WDT down-counter reload value for WDT counter 2.
                              This parameter must be a number between Min_Data = 0 and Max_Data = 0x0FFFFFF */

} WDT_InitTypeDef;

/**
 * @brief  WDT Handle Structure definition
 */
typedef struct
{
    WDT_TypeDef                 *Instance;  /*!< Register base address    */

    WDT_InitTypeDef             Init;       /*!< WDTrequired parameters */

} WDT_HandleTypeDef;

/**
 * @}
 */

/* Exported constants --------------------------------------------------------*/
/** @defgroup WDT_Exported_Constants WDT Exported Constants
 * @{
 */
#define WDT_CMD_START                 0x00000076U  /*!< WDT start       */
#define WDT_CMD_STOP                  0x00000034U  /*!< WDT stop      */

/* Status register need 5 RC LSI divided by prescaler clock to be updated. With
   higher prescaler (256), and according to HSI variation, we need to wait at
   least 6 cycles so 48 ms. */
#define HAL_WDT_DEFAULT_TIMEOUT       48000U

/**
 * @}
 */


/* Exported macros -----------------------------------------------------------*/
/** @defgroup WDT_Exported_Macros WDT Exported Macros
 * @{
 */

/**
 * @brief  Start the WDT count down.
 * @param  \__HANDLE__  WDT handle
 * @retval None
 */
#define __HAL_WDT_START(__HANDLE__)    \
{ \
    unsigned int cnt=0U;\
    WRITE_REG((__HANDLE__)->Instance->WDT_CCR, WDT_CMD_START);\
    while (((__HANDLE__)->Instance->WDT_SR & WDT_WDT_SR_WDT_ACTIVE) == 0 && cnt<HAL_WDT_DEFAULT_TIMEOUT) {\
        cnt++; \
        HAL_Delay_us_(1);\
    }\
}

/**
 * @brief  Stop the WDT count down.
 * @param  \__HANDLE__  WDT handle
 * @retval None
 */
#define __HAL_WDT_STOP(__HANDLE__)     \
{\
    unsigned int cnt=0U;\
    HAL_Delay(1);\
    WRITE_REG((__HANDLE__)->Instance->WDT_CCR, WDT_CMD_STOP);\
    while (((__HANDLE__)->Instance->WDT_SR & WDT_WDT_SR_WDT_ACTIVE) == WDT_WDT_SR_WDT_ACTIVE && cnt<HAL_WDT_DEFAULT_TIMEOUT) {\
        cnt++; \
        HAL_Delay_us_(1);\
    }\
    HAL_Delay(1);\
}

/**
 * @brief  Clear interrupt
 * @param  \__HANDLE__  WDT handle
 * @retval None
 */
#define __HAL_WDT_CLEAR(__HANDLE__)                MODIFY_REG((__HANDLE__)->Instance->WDT_ICR,0,WDT_WDT_ICR_INT_CLR)

/**
 * @brief  WDT generate interrupt or reset only
 * @param  \__HANDLE__  WDT handle
 * @param  INT  1: Generate interrupt and reset   0:Reset only
 * @retval None
 */
#define __HAL_WDT_INT(__HANDLE__,INT)              MODIFY_REG((__HANDLE__)->Instance->WDT_CR,WDT_WDT_CR_RESPONSE_MODE,INT?WDT_WDT_CR_RESPONSE_MODE:0)

/**
 * @brief  Disable WDT.
 * @retval None
 */
#ifdef SOC_BF0_HCPU
#define __HAL_WDT_DISABLE()                       WRITE_REG(hwp_wdt1->WDT_CCR, WDT_CMD_STOP)
#else
#define __HAL_WDT_DISABLE()                       WRITE_REG(hwp_wdt2->WDT_CCR, WDT_CMD_STOP)
#endif
#define __HAL_IWDT_DISABLE()                       WRITE_REG(hwp_iwdt->WDT_CCR, WDT_CMD_STOP)

#ifdef SF32LB55X
#define WDT_IRQHandler NMI_Handler
#endif

/**
 * @brief  Reload WDT counter 1 with value defined in the reload register
 * @param  \__HANDLE__  WDT handle
 * @retval None
 */
#define __HAL_WDT_RELOAD_COUNTER(__HANDLE__)       WRITE_REG((__HANDLE__)->Instance->WDT_CVR0,(__HANDLE__)->Init.Reload)

/**
 * @brief  Reload WDT counter 2 with value defined in the reload register
 * @param  \__HANDLE__  WDT handle
 * @retval None
 */
#define __HAL_WDT_RELOAD_COUNTER2(__HANDLE__)       WRITE_REG((__HANDLE__)->Instance->WDT_CVR1,(__HANDLE__)->Init.Reload2)


#define WDT_PROTECT_MAGIC 0x58ab99fc
#define WDT_RELEASE_MAGIC 0x51ff8621
/**
 * @brief  Protect WDT write
 * @param  \__HANDLE__  WDT handle
 * @param  PROTECT 1: Protect ,  0: Release
 * @retval None
 */
#define __HAL_WDT_PROTECT(__HANDLE__,PROTECT)       MODIFY_REG((__HANDLE__)->Instance->WDT_WP,WDT_WDT_WP_WRPT_Msk,PROTECT?WDT_PROTECT_MAGIC:WDT_RELEASE_MAGIC)


/**
 * @brief  Check whether WDT write is protected
 * @param  \__HANDLE__  WDT handle
 * @retval Non-Zer0 is protected, 0 is not protected
 */
#define __HAL_WDT_IS_PROTECTED(__HANDLE__) (((__HANDLE__)->Instance->WDT_WP&WDT_WDT_WP_WRPT_ST_Msk)==WDT_WDT_WP_WRPT_ST)

/**
 * @}
 */

/* Exported functions --------------------------------------------------------*/
/** @defgroup WDT_Exported_Functions  WDT Exported Functions
 * @{
 */

/** @defgroup WDT_Exported_Functions_Group1 Initialization and Start functions
 * @{
 */

/**
 * @brief  Initialize Watchdog timer.
 * @param  wdt  WDT handle
 * @retval Status of initialization, 0 OK, non-Zer0 error.
 */
HAL_StatusTypeDef HAL_WDT_Init(WDT_HandleTypeDef *wdt);
/**
 * @}
 */

/** @defgroup WDT_Exported_Functions_Group2 IO operation functions
 * @{
 */
/**
 * @brief  Kick Watchdog timer.
 * @param  wdt  WDT handle
 * @retval Status of kick, 0 OK, non-Zer0 error.
 */
HAL_StatusTypeDef HAL_WDT_Refresh(WDT_HandleTypeDef *wdt);
/**
 * @}
 */

/**
 * @}
 */

/* Private constants ---------------------------------------------------------*/
/** @defgroup WDT_Private_Constants WDT Private Constants
 * @{
 */

/**
 * @brief  WDT Command Register value
 */

/**
 * @}
 */

/* Private macros ------------------------------------------------------------*/
/** @defgroup WDT_Private_Macros WDT Private Macros
 * @{
 */
#ifdef hwp_wdt1
#define IS_WDT_ALL_INSTANCE(wdt)    ((wdt)==hwp_wdt1||(wdt)==hwp_wdt2||(wdt)==hwp_iwdt)
#else
#define IS_WDT_ALL_INSTANCE(wdt)    ((wdt)==hwp_wdt2||(wdt)==hwp_iwdt)
#endif /* hwp_wdt1 */
#define IS_WDT_RELOAD(value)        (((value)&0xFF000000)==0)


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

#endif /* BF0_HAL_Driver */