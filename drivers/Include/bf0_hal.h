/**
  ******************************************************************************
  * @file   bf0_hal.h
  * @author Sifli software development team
  * @brief   This file contains all the functions prototypes for the HAL
  *          module driver.
  * @{
  ******************************************************************************
*/
/*
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

#ifndef __BF0_CP_HAL_H
#define __BF0_CP_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAKE_REG_VAL(val, mask, offset)        ((((uint32_t)(val)) << (offset)) & (mask))
#define GET_REG_VAL(reg, mask, offset)        ((((uint32_t)(reg)) & (mask)) >> (offset))

//#define HAL_DEBUG_ENABLED
//#define TARMAC

/* Includes ------------------------------------------------------------------*/
#ifdef SOC_BF0_HCPU
#include "bf0_hal_conf_hcpu.h"
#elif defined(SOC_BF0_LCPU)
#include "bf0_hal_conf_lcpu.h"
#elif defined(WIN32)
#define __IO volatile
#define __STATIC_INLINE                        static __inline
#define __get_PRIMASK() 0
#define __set_PRIMASK(mask) 0
#else
#error "Core not defined."
#endif
#include "bf0_hal_hlp.h"

/** @addtogroup BF0_HAL_Driver Hardware Abstract Layer
  * @brief Hardware Abstract Layer driver interface
  * @{
  */

/** @addtogroup HAL_COMMON Hal Common functions
 *  @{
 *
*/

/** @addtogroup HAL_SYS_CFG System Configuration
  * @ingroup HAL_COMMON
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @addtogroup SYSCFG_Exported_Constants SYSCFG Exported Constants
  * @{
  */

/** @addtogroup SYSCFG_BootMode Boot Mode
  * @{
  */
#define SYSCFG_BOOT_NORMAL         0U       /*!< Normal boot */
#define SYSCFG_BOOT_UART           1U       /*!< Boot with UART loop for DFU */
/**
  * @} SYSCFG_BootMode
  */

/** @defgroup SYSCFG_SWD SWD connect interface
    @ingroup SYSCFG_Exported_Constants
  * @{
  */
#define SYSCFG_SWD_HCPU             0       /*!< SWD interface connect to HCPU */
#define SYSCFG_SWD_LCPU             1       /*!< SWD interface connect to LCPU */
#define SYSCFG_SWD_BCPU             2       /*!< SWD interface connect to BCPU */

/**
  * @} SYSCFG_SWD
  */

/**
  * @} SYSCFG_Exported_Constants
  */

/** @defgroup SYSCFG_Exported_Macros SYSCFG Exported Macros
    @ingroup HAL_SYS_CFG
  * @{
  */

/** @brief  Get Current boot mode.
  * @retval The boot mode as configured by user
  *           @arg @ref SYSCFG_BOOT_NORMAL
  *           @arg @ref SYSCFG_BOOT_UART
  */
#define __HAL_SYSCFG_GET_BOOT_MODE()        (hwp_hpsys_cfg->BMR&HPSYS_CFG_BMR_BOOT_MODE_Msk)

/** @brief  Set SWD interface.
  * @param  COREID core id for SWD
  *           @arg @ref SYSCFG_SWD_HCPU
  *           @arg @ref SYSCFG_SWD_LCPU
  *           @arg @ref SYSCFG_SWD_BCPU
  */
#define __HAL_SYSCFG_SET_SWD(COREID)        (hwp_lpsys_rcc->SWCR=COREID)

/** @brief  Set Security Key control.
  */
#define __HAL_SYSCFG_SET_SECURITY()      (hwp_hpsys_cfg->SCR|=HPSYS_CFG_SCR_FKEY_MODE)

/** @brief  Clear Security Key control.
  */

#define __HAL_SYSCFG_CLEAR_SECURITY()      (hwp_hpsys_cfg->SCR&=~HPSYS_CFG_SCR_FKEY_MODE)

#define __HAL_SYSCFG_GET_SID()            (hwp_hpsys_cfg->IDR>>HPSYS_CFG_IDR_SID_Pos)           /*!< Get serial ID*/
#define __HAL_SYSCFG_GET_CID()            ((hwp_hpsys_cfg->IDR>>HPSYS_CFG_IDR_CID_Pos)&0xff)    /*!< Get Chip ID*/
#define __HAL_SYSCFG_GET_PID()            ((hwp_hpsys_cfg->IDR>>HPSYS_CFG_IDR_PID_Pos)&0xff)    /*!< Get Package ID*/
#ifndef SOC_SF32LB55X
#define __HAL_SYSCFG_GET_REVID()          (hwp_hpsys_cfg->IDR&0xff)                             /*!< Get Revision ID*/
#else
#define __HAL_SYSCFG_GET_REVID()          ((hwp_hpsys_cfg->USBCR>>HPSYS_CFG_USBCR_RSVD1_Pos)&0xff) /*!< Get Revision ID*/
#endif

#define __HAL_SYSCFG_Enable_USB()         (hwp_hpsys_cfg->USBCR|=HPSYS_CFG_USBCR_USB_EN)        /*!< Enable USB module*/
#define __HAL_SYSCFG_USB_DM_PD()         (hwp_hpsys_cfg->USBCR|=HPSYS_CFG_USBCR_DM_PD)        /*!< Pull Down USB DM pin, host only*/


#ifdef SF32LB55X
#define __HAL_SYSCFG_Enable_WDT_REBOOT(enable)
#else
#ifdef SOC_BF0_HCPU
#define __HAL_SYSCFG_Enable_WDT_REBOOT(enable)      \
    {\
        if (enable) \
            (hwp_hpsys_cfg->SYSCR|=HPSYS_CFG_SYSCR_WDT1_REBOOT);       /*!< Enable Watchdog to reboot whole system*/ \
        else  \
            (hwp_hpsys_cfg->SYSCR&=~HPSYS_CFG_SYSCR_WDT1_REBOOT);      /*!< Watchdog only reboot HCPU subsystem*/  \
    }
#else
#define __HAL_SYSCFG_Enable_WDT_REBOOT(enable)      \
    {\
        if (enable) \
            (hwp_lpsys_cfg->SYSCR|=LPSYS_CFG_SYSCR_WDT2_REBOOT);       /*!< Enable Watchdog to reboot whole system*/ \
        else  \
            (hwp_lpsys_cfg->SYSCR&=~LPSYS_CFG_SYSCR_WDT2_REBOOT);      /*!< Watchdog only reboot LCPU subsystem*/  \
    }
#endif
#endif

#ifndef SF32LB55X
#ifdef SOC_BF0_HCPU
#define __HAL_SYSCFG_Enable_Assert_Trigger(enable)    \
        { \
            if (enable) \
                (hwp_hpsys_cfg->DBGR |= HPSYS_CFG_DBGR_LP2HP_NMIE);   /*!< Enable to receive LCPU trigger assert notify*/ \
            else \
                (hwp_hpsys_cfg->DBGR &= ~HPSYS_CFG_DBGR_LP2HP_NMIE);  /*!< Not receive LCPU trigger assert notify*/  \
        }

#define __HAL_SYSCFG_Trigger_Assert()               (hwp_hpsys_cfg->DBGR |= HPSYS_CFG_DBGR_HP2LP_NMI)
#define __HAL_SYSCFG_Get_Trigger_Assert_Flag()      ((hwp_hpsys_cfg->DBGR & HPSYS_CFG_DBGR_LP2HP_NMIF) >> HPSYS_CFG_DBGR_LP2HP_NMIF_Pos)
#else // SOC_BF0_HCPU
#define __HAL_SYSCFG_Enable_Assert_Trigger(enable)    \
        { \
            if (enable) \
                (hwp_lpsys_cfg->DBGR |= LPSYS_CFG_DBGR_HP2LP_NMIE);   /*!< Enable to receive HCPU trigger assert notify*/ \
            else \
                (hwp_lpsys_cfg->DBGR &= ~LPSYS_CFG_DBGR_HP2LP_NMIE);  /*!< Not receive HCPU trigger assert notify*/  \
        }

#define __HAL_SYSCFG_Trigger_Assert()               (hwp_lpsys_cfg->DBGR |= LPSYS_CFG_DBGR_LP2HP_NMI)
#define __HAL_SYSCFG_Get_Trigger_Assert_Flag()      ((hwp_lpsys_cfg->DBGR & LPSYS_CFG_DBGR_HP2LP_NMIF) >> LPSYS_CFG_DBGR_HP2LP_NMIF_Pos)
#endif // SOC_BF0_HCPU
#endif // SF32LB55X

#ifndef  SF32LB55X
#define __HAL_SYSCFG_HPBG_EN()                (hwp_hpsys_cfg->CAU2_CR |= HPSYS_CFG_CAU2_CR_HPBG_EN)
#define __HAL_SYSCFG_HPBG_VDDPSW_EN()        (hwp_hpsys_cfg->CAU2_CR |= HPSYS_CFG_CAU2_CR_HPBG_VDDPSW_EN)
#endif  //#ifdef SF32LB55X

#define SystemFixClock  48000000

#ifdef ROM_ENABLED
#define __HAL_ROM_USED __weak
#else
#define __HAL_ROM_USED
#endif


#ifndef HAL_ALIGN
#define HAL_ALIGN(size, align)           (((size) + (align) - 1) & ~((align) - 1))
#endif /* HAL_ALIGN */
#ifndef HAL_ALIGN_DOWN
#define HAL_ALIGN_DOWN(size, align)      ((size) & ~((align) - 1))
#endif /* HAL_ALIGN_DOWN */

#ifndef HAL_MIN
#define HAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#endif /* HAL_MIN */

#ifndef HAL_MAX
#define HAL_MAX(a, b) ((a) > (b) ? (a) : (b))
#endif /* HAL_MAX */

#ifndef HAL_ABS
#define HAL_ABS(x) ((x) > 0 ? (x) : (-(x)))
#endif /* HAL_ABS */

/** @brief  CHIP ID for A3.
  */
#ifdef SOC_SF32LB55X
#define HAL_CHIP_REV_ID_A3 0x80
#else
#define HAL_CHIP_REV_ID_A3 0xFF // Not defined
#endif

#ifdef SF32LB52X
#define HAL_CHIP_REV_ID_A4 0x07
#else
#define HAL_CHIP_REV_ID_A4 0xFF
#endif


#ifdef SF32LB52X
#ifdef SOC_BF0_HCPU
#define HAL_LXT_ENABLED() HAL_RTC_LXT_ENABLED()
#define HAL_LXT_DISABLED() (!HAL_RTC_LXT_ENABLED())
#else
#define HAL_LXT_ENABLED() HAL_GetLXTEnabled()
#define HAL_LXT_DISABLED() (!HAL_GetLXTEnabled())
#endif
#else
#define HAL_LXT_ENABLED() HAL_PMU_LXT_ENABLED()
#define HAL_LXT_DISABLED() HAL_PMU_LXT_DISABLED()
#endif


/**
  * @} SYSCFG_Exported_Macros
  */

/**
  * @} HAL_SYS_CFG
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup HAL_Exported_Functions HAL Exported Functions
  * @ingroup HAL_COMMON
  * @{
  */

/** @addtogroup HAL_Exported_Functions_Group1
  * @ingroup HAL_Exported_Functions
  * @{
  */

/* Initialization and de-initialization functions  ******************************/
/**
  * @brief  Configure the Flash prefetch, the Instruction and Data caches,
  *         the time base source, NVIC and any required global low level hardware
  *         by calling the HAL_PreInit() callback function to be optionally defined in user file
  *
  * @note   HAL_Init() function is called at the beginning of program after reset and before
  *         the clock configuration.
  *
  * @note   In the default implementation the System Timer (Systick) is used as source of time base.
  *         The Systick configuration is based on MSI clock, as MSI is the clock
  *         used after a system Reset and the NVIC configuration is set to Priority group 4.
  *         Once done, time base tick starts incrementing: the tick variable counter is incremented
  *         each 1ms in the SysTick_Handler() interrupt handler.
  *
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_Init(void);

/**
  * @brief  Before HAL_Init is called, board specific initialization.
  * @retval None
  */
void HAL_PreInit(void);

/**
  * @brief  It's called after HAL_PreInit in HAL_Init
  * @retval None
  */
void HAL_PostMspInit(void);


/**
  * @brief De-initialize common part of the HAL and stop the source of time base.
  * @note This function is optional.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DeInit(void);

/**
  * @brief  DeInitialize the MSP.
  * @retval None
  */
void HAL_MspDeInit(void);

/**
  * @brief This function configures the source of the time base:
  *        The time source is configured to have 1ms time base with a dedicated
  *        Tick interrupt priority.
  * @note This function is called  automatically at the beginning of program after
  *       reset by HAL_Init() or at any time when clock is reconfigured  by HAL_RCC_ClockConfig().
  * @note In the default implementation, SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals.
  *       Care must be taken if HAL_Delay() is called from a peripheral ISR process,
  *       The SysTick interrupt must have higher priority (numerically lower)
  *       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
  *       The function is declared as __weak  to be overwritten  in case of other
  *       implementation  in user file.
  * @param TickPriority  Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

/**
  * @} HAL_Exported_Functions_Group1
  */

/** @addtogroup HAL_Exported_Functions_Group2 HAL Control functions
  * @ingroup HAL_Exported_Functions
  * @{
  */

/* Peripheral Control functions  ************************************************/
/**
  * @brief This function is called to increment a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in SysTick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
  *      implementations in user file.
  * @retval None
  */
void HAL_IncTick(void);

/**
  * @brief This function provides minimum delay (in milliseconds) based
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay  specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(uint32_t Delay);
void HAL_Delay_us(uint32_t us);
void HAL_Delay_us_(__IO uint32_t us);

/**
  * @brief Provide a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval tick value
  */
uint32_t HAL_GetTick(void);

/**
  * @brief Suspend Tick increment.
  * @note In the default implementation , SysTick timer is the source of time base. It is
  *       used to generate interrupts at regular time intervals. Once HAL_SuspendTick()
  *       is called, the SysTick interrupt will be disabled and so Tick increment
  *       is suspended.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval None
  */
void HAL_SuspendTick(void);

/**
  * @brief Resume Tick increment.
  * @note In the default implementation , SysTick timer is the source of time base. It is
  *       used to generate interrupts at regular time intervals. Once HAL_ResumeTick()
  *       is called, the SysTick interrupt will be enabled and so Tick increment
  *       is resumed.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval None
  */
void HAL_ResumeTick(void);

/**
  * @} HAL_Exported_Functions_Group2
  */

/**
  * @} HAL_Exported_Functions
  */

/** @defgroup HAL_PORT Hal porting functions
 *  @ingroup HAL_COMMON
 *  @{
 *
*/

#if defined(USE_FULL_ASSERT)||defined(_SIFLI_DOXYGEN_)
#define HAL_ASSERT(expr) ((expr) ? (void)0U : HAL_AssertFailed((char *)__FILE__, __LINE__))

/* Exported functions ------------------------------------------------------- */
/**
  * @brief  The assert failure printing.
    If BSP_USING_FULL_ASSERT is defined, the asser_failed function need to be implemented.
  * @param  file file name of source code for the failed assert
  * @param  line code line number for the failed assert
  * @retval None
  */
void HAL_AssertFailed(char *file, uint32_t line);
#elif defined(USE_LOOP_ASSERT)
#define HAL_ASSERT(expr) if ((expr)==0) while (1) {;}
#else
#define HAL_ASSERT(expr) if ((expr)==0) while (1) {;}
#endif /* USE_FULL_ASSERT */

/**
  * @brief  Debug function to stop current thread execution.
  * @retval None
  */
#if defined(__CLANG_ARM) || defined(__GNUC__)
__STATIC_INLINE void HAL_sw_breakpoint(void)
{
    __ASM
    (
        "B . \n"
    );
}
#else
__STATIC_INLINE void HAL_sw_breakpoint(void)
{
    volatile uint32_t dummy = 0;
    while (0 == dummy);
}
#endif

__STATIC_INLINE uint32_t HAL_DisableInterrupt(void)
{
    uint32_t mask;

    mask = __get_PRIMASK();
    __set_PRIMASK(1);
    return mask;
}

__STATIC_INLINE void HAL_EnableInterrupt(uint32_t mask)
{
    __set_PRIMASK(mask);
}

uint32_t HAL_GetLXTEnabled(void);


__STATIC_INLINE uint32_t HAL_GetElapsedTick(uint32_t prev_tick, uint32_t curr_tick)
{
    uint32_t elapsed_tick;

    if (curr_tick >= prev_tick)
    {
        elapsed_tick = curr_tick - prev_tick;
    }
    else
    {
        elapsed_tick = (UINT32_MAX - prev_tick) + 1 + curr_tick;
    }

    return elapsed_tick;
}


/**
  * @} HAL_PORT
  */

/**
  * @} HAL_COMMON
  */

/**
  * @} BF0_HAL_Driver
  */

#ifdef BSP_USING_NO_OS
#if defined(__CC_ARM) || defined(__CLANG_ARM)
#elif defined(__ICCARM__)
#define main __low_level_init
#elif defined(__GNUC__)
#define main entry
#endif
#else
#ifdef WIN32
#define main app_main
#endif
#endif

/// @} file

#ifdef __cplusplus
}
#endif

#endif /* __BF0_CP_HAL_H */



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
