/**
  ******************************************************************************
  * @file   bf0_hal.c
  * @author Sifli software development team
  * @brief   HAL module driver.
  *          This is the common part of the HAL initialization
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
#include "bf0_hal_pinmux.h"
/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup HAL_COMMON Hal Common functions
  * @brief HAL module driver
  * @{
  */

#if defined(HAL_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#if defined(VREFBUF)
    #define VREFBUF_TIMEOUT_VALUE     10U   /* 10 ms (to be confirmed) */
#endif /* VREFBUF */

/* ------------ SYSCFG registers bit address in the alias region ------------ */
#define SYSCFG_OFFSET             (SYSCFG_BASE - PERIPH_BASE)
/* ---  MEMRMP Register ---*/
/* Alias word address of FB_MODE bit */
#define MEMRMP_OFFSET             SYSCFG_OFFSET
#define FB_MODE_BitNumber         8U
#define FB_MODE_BB                (PERIPH_BB_BASE + (MEMRMP_OFFSET * 32U) + (FB_MODE_BitNumber * 4U))

/* --- SCSR Register ---*/
/* Alias word address of SRAM2ER bit */
#define SCSR_OFFSET               (SYSCFG_OFFSET + 0x18U)
#define BRER_BitNumber            0U
#define SCSR_SRAM2ER_BB           (PERIPH_BB_BASE + (SCSR_OFFSET * 32U) + (BRER_BitNumber * 4U))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

#if defined(__CC_ARM) || defined(__CLANG_ARM)
#elif defined(__GNUC__)
    /* workaround large bin size produced by arm-none-eabi-gcc xpack version
    * see https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/discussions/23
    */
    char __aeabi_unwind_cpp_pr0[0];
#endif

/** @defgroup HAL_Exported_Variables HAL Exported Variables
  * @{
  */
__IO uint32_t uwTick;
/**
  * @} HAL_Exported_Variables
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup HAL_Exported_Functions HAL Exported Functions
  * @{
  */

/** @defgroup HAL_Exported_Functions_Group1 Initialization and de-initialization Functions
 *  @brief    Initialization and de-initialization functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the Flash interface the NVIC allocation and initial time base
          clock configuration.
      (+) De-initialize common part of the HAL.
      (+) Configure the time base source to have 1ms time base with a dedicated
          Tick interrupt priority.
        (++) SysTick timer is used by default as source of time base, but user
             can eventually implement his proper time base source (a general purpose
             timer for example or other time source), keeping in mind that Time base
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
             handled in milliseconds basis.
        (++) Time base configuration function (HAL_InitTick ()) is called automatically
             at the beginning of the program after reset by HAL_Init() or at any time
             when clock is configured, by HAL_RCC_ClockConfig().
        (++) Source of time base is configured  to generate interrupts at regular
             time intervals. Care must be taken if HAL_Delay() is called from a
             peripheral ISR process, the Tick interrupt line must have higher priority
            (numerically lower) than the peripheral interrupt. Otherwise the caller
            ISR process will be blocked.
       (++) functions affecting time base configurations are declared as __weak
             to make  override possible  in case of other  implementations in user file.
@endverbatim
  * @{
  */

/**
  * @brief  Configure the Flash prefetch, the Instruction and Data caches,
  *         the time base source, NVIC and any required global low level hardware
  *         by calling the HAL_PreInit() weak function to be optionally defined in user file
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
HAL_StatusTypeDef HAL_Init(void)
{
    HAL_StatusTypeDef  status = HAL_OK;

    /* Configure Flash prefetch, Instruction cache, Data cache */
    /* Default configuration at reset is:                      */
    /* - Prefetch disabled                                     */
    /* - Instruction cache enabled                             */
    /* - Data cache enabled                                    */
#if (INSTRUCTION_CACHE_ENABLE == 0)
//   __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
#endif /* INSTRUCTION_CACHE_ENABLE */

#if (DATA_CACHE_ENABLE == 0)
//   __HAL_FLASH_DATA_CACHE_DISABLE();
#endif /* DATA_CACHE_ENABLE */

#if (PREFETCH_ENABLE != 0)
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif /* PREFETCH_ENABLE */

#if defined(HAL_V2D_GPU_MODULE_ENABLED)
    HAL_RCC_ResetModule(RCC_MOD_GPU);
#endif

    HAL_PreInit();

#ifdef SOC_BF0_HCPU
    if (PM_STANDBY_BOOT != SystemPowerOnModeGet())
    {
//TODO: 52x also needs RC48
//#ifndef SF32LB52X
        // Except Standby mode, all other boot mode need to re-calibrate RC48
        status = HAL_RCC_CalibrateRC48();
#ifndef TARMAC
        HAL_ASSERT(HAL_OK == status);
#endif
//#endif /* SF32LB52X */
    }
#endif /* SOC_BF0_HCPU */

    HAL_PostMspInit();
    HAL_RCC_Init();

#ifdef SOC_BF0_HCPU
    if (PM_STANDBY_BOOT != SystemPowerOnModeGet())
    {
        HAL_PMU_Init();
    }
#endif /* SOC_BF0_HCPU */

    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* Use SysTick as time base source and configure 1ms tick (default clock after Reset is MSI) */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
    {
        status = HAL_ERROR;
    }
    else

    {
#ifdef SOC_BF0_HCPU
        /* init AES_ACC as normal mode */
        __HAL_SYSCFG_CLEAR_SECURITY();
        HAL_EFUSE_Init();
#endif
    }

    /* Return function status */
    return status;
}

/**
  * @brief De-initialize common part of the HAL and stop the source of time base.
  * @note This function is optional.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DeInit(void)
{

#if 0
    /* Reset of all peripherals */
    __HAL_RCC_APB1_FORCE_RESET();
    __HAL_RCC_APB1_RELEASE_RESET();

    __HAL_RCC_APB2_FORCE_RESET();
    __HAL_RCC_APB2_RELEASE_RESET();

    __HAL_RCC_AHB1_FORCE_RESET();
    __HAL_RCC_AHB1_RELEASE_RESET();

    __HAL_RCC_AHB2_FORCE_RESET();
    __HAL_RCC_AHB2_RELEASE_RESET();

    __HAL_RCC_AHB3_FORCE_RESET();
    __HAL_RCC_AHB3_RELEASE_RESET();
#endif
    /* De-Init the low level hardware */
    HAL_MspDeInit();

    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Before HAL_Init is called, board specific initialization.
  * @retval None
  */
__weak void HAL_PreInit(void)
{
    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_PreInit could be implemented in the board specific file
     */
}

/**
  * @brief  It's called after HAL_PreInit.
  * @retval None
  */

__weak void HAL_PostMspInit(void)
{

}


/**
  * @brief  DeInitialize the MSP.
  * @retval None
  */
__weak void HAL_MspDeInit(void)
{
    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_MspDeInit could be implemented in the user file
     */
}

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
__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    HAL_StatusTypeDef  status = HAL_OK;

    /*Configure the SysTick to have interrupt in 1ms time basis*/
    if (HAL_SYSTICK_Config(SystemCoreClock / 1000UL) != 0U)
    {
        status = HAL_ERROR;
    }
    else
    {
        /*Configure the SysTick IRQ priority */
        HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0);
    }

    /* Return function status */
    return status;
}

/**
  * @} HAL_Exported_Functions_Group1
  */

/** @defgroup HAL_Exported_Functions_Group2 HAL Control functions
 *  @brief    HAL Control functions
 *
@verbatim
 ===============================================================================
                      ##### HAL Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Provide a tick value in millisecond
      (+) Provide a blocking delay in millisecond
      (+) Suspend the time base source interrupt
      (+) Resume the time base source interrupt
      (+) Get the HAL API driver version
      (+) Get the device identifier
      (+) Get the device revision identifier

@endverbatim
  * @{
  */


/**
  * @brief This function is called to increment a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in SysTick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
  *      implementations in user file.
  * @retval None
  */
__weak void HAL_IncTick(void)
{
    uwTick++;
}

/**
  * @brief Provide a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval tick value
  */
__weak uint32_t HAL_GetTick(void)
{
    return uwTick;
}

// For CM33, need 5 instructions(12 cycles), Please note that XIP might take more time in flash read.

/*
    LDR     r0,[sp,#0]  3 cycles
    SUBS    r1,r0,#1    1
    CMP     r0,#0       1
    STR     R1,[SP,0]       1
    BNE     6 cycles
*/
#define WAIT_US_LOOP_CYCLE 12
__weak void HAL_Delay_us_(__IO uint32_t us)
{
    //TODO: replaced by SystemCoreClock?
    static uint32_t sysclk_m;

    if (us == 0 || sysclk_m == 0)
    {
        sysclk_m = HAL_RCC_GetHCLKFreq(CORE_ID_DEFAULT) / 1000000;
        if (us == 0)
            return;
    }

    if (us > (1 << 24)) // MAX 16s to avoid cnt over range --- suppose clk max to 256M
        HAL_ASSERT(0);

    //if (sysclk_m > 48)   // DLL has extra cost, Reason to be determined.
    //    us -= 1;
    if (us > 0)          // Extra 1 us for cost of function
    {
#ifdef DWT

        if (0 == HAL_DBG_DWT_IsInit())
        {
            HAL_DBG_DWT_Init();
        }
        uint32_t cnt = sysclk_m * us;
        uint32_t start = HAL_DBG_DWT_GetCycles();
        while ((HAL_DBG_DWT_GetCycles() - start) < cnt)
        {
        }

#else
        volatile uint32_t i = sysclk_m * (us - 1) / WAIT_US_LOOP_CYCLE;
        while (i-- > 0); //need 5 instructions at ARMCC
#endif
    }
}

// Use systick to get more accurate us level delay.
__weak void HAL_Delay_us2_(__IO uint32_t us)
{
    uint32_t reload = SysTick->LOAD;
    uint32_t told, tnow, tcnt = 0;
    us = us * reload / (1000000 / HAL_TICK_PER_SECOND);
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= us)
            {
                break;
            }
        }
    }

}

#define MAX_US_DELAY    10000
__weak void HAL_Delay_us(uint32_t us)
{
    uint32_t ticks;

    if (0 == us)
    {
        /* sysclk_m needs to be initialized as it's a random value in keil driver */
        HAL_Delay_us_(0);
    }

    while (us > 0)
    {
        if (us > MAX_US_DELAY)
        {
            ticks = MAX_US_DELAY;
            us -= MAX_US_DELAY;
        }
        else
        {
            ticks = us;
            us = 0;
        }
        if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0) // Systick not enabled yet, use loop
            HAL_Delay_us_(ticks);
        else
            HAL_Delay_us2_(ticks);
    }
}


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
__weak void HAL_Delay(__IO uint32_t Delay)
{
#if 1
    while (Delay > 0)
    {
        HAL_Delay_us(1000);
        Delay--;
    }
#else
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;

    /* Add a period to guaranty minimum wait */
    if (wait < HAL_MAX_DELAY)
    {
        wait++;
    }

    while ((HAL_GetTick() - tickstart) < wait)
    {
    }
#endif
}

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
__weak void HAL_SuspendTick(void)
{
    /* Disable SysTick Interrupt */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

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
__weak void HAL_ResumeTick(void)
{
    /* Enable SysTick Interrupt */
    SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
}

__weak void HAL_AssertFailed(char *file, uint32_t line)
{
    volatile uint32_t dummy = 0;
    while (0 == dummy);
}

#ifndef SF32LB55X
#ifdef HAL_WDT_MODULE_ENABLED
__weak void WDT_IRQHandler(void)
{

}
#endif

__weak void DBG_Trigger_IRQHandler(void)
{

}

//SF32LB52X in security mode used
#if defined(SF32LB52X) && defined(SOC_BF0_HCPU)
void PendSv_DBG_Trigger(void)
{
    DBG_Trigger_IRQHandler();
}
#endif /* SF32LB52X && SOC_BF0_HCPU */

#ifdef HAL_NMI_HANLDER_OVERRIDED
    __weak void NMI_Handler(void)
#else
    void NMI_Handler(void)
#endif /* HAL_NMI_HANLDER_OVERRIDED */
{
    if (__HAL_SYSCFG_Get_Trigger_Assert_Flag())
        DBG_Trigger_IRQHandler();
#ifdef HAL_WDT_MODULE_ENABLED
    else
        WDT_IRQHandler();
#endif
}

#if defined(SF32LB52X) && defined(SOC_BF0_LCPU)
__HAL_ROM_USED uint32_t HAL_GetLXTEnabled(void)
{
    uint8_t is_lxt_enabled;
    uint16_t len = 1;
    HAL_StatusTypeDef ret = HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_XTAL_ENABLED, &is_lxt_enabled, &len);

    //default is 32K
    if (ret != HAL_OK)
        is_lxt_enabled = 1;

    return (uint32_t)is_lxt_enabled;
}
#endif


#endif // SF32LB55X

/**
 * @} HAL_Exported_Functions_Group2
 */

/**
 * @} HAL_Exported_Functions
 */

#endif /* HAL_MODULE_ENABLED */

/**
 * @} HAL_COMMON
 */

/**
 * @} BF0_HAL_Driver
 */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
