/**
  ******************************************************************************
  * @file   system_bf0_cp.c
  * @author Sifli software development team
  * @brief   CMSIS Cortex-M0 Device Peripheral Access Layer System Source File.
  *
  ******************************************************************************
*/
/**
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

#include "bf0_hal.h"
#include "register.h"

/**
  * @}
  */

/** @addtogroup BF0_CP_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup BF0_CP_System_Private_Defines
  * @{
  */
#if !defined  (HSE_VALUE)
    #define HSE_VALUE    ((uint32_t)8000000) /*!< Default value of the External oscillator in Hz.
    This value can be provided and adapted by the user application. */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
    #define HSI_VALUE    ((uint32_t)8000000) /*!< Default value of the Internal oscillator in Hz.
    This value can be provided and adapted by the user application. */
#endif /* HSI_VALUE */

#if !defined (HSI48_VALUE)
    #define HSI48_VALUE    ((uint32_t)48000000) /*!< Default value of the HSI48 Internal oscillator in Hz.
    This value can be provided and adapted by the user application. */
#endif /* HSI48_VALUE */
/**
  * @}
  */

/** @addtogroup BF0_CP_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup BF0_CP_System_Private_Variables
  * @{
  */
/* This variable is updated in three ways:
    1) by calling CMSIS function SystemCoreClockUpdate()
    2) by calling HAL API function HAL_RCC_GetHCLKFreq()
    3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
       Note: If you use this function to configure the system clock there is no need to
             call the 2 first functions listed above, since SystemCoreClock variable is
             updated automatically.
*/
uint32_t SystemCoreClock = 24000000;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * @}
  */

/** @addtogroup BF0_CP_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup BF0_CP_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system.
  *         Initialize the default HSI clock source, vector table location and the PLL configuration is reset.
  * @param  None
  * @retval None
  */
#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
    extern uint32_t __Vectors;
#endif

void SystemInit(void)
{
#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
    SCB->VTOR = (uint32_t)&__Vectors;
#endif
}

#ifndef __COREMARK_ENABLE__
typedef struct
{
    uint32_t load_region;
    uint32_t exe_region;
    uint32_t length;
    uint32_t callback;
} region_init_t;

extern int Load$$RW_IRAM1$$RW$$Base;
extern int Load$$RW_IRAM1$$ZI$$Base;
extern int Image$$RW_IRAM1$$RW$$Base;
extern int Image$$RW_IRAM1$$ZI$$Base;
extern int Image$$RW_IRAM1$$RW$$Length;
extern int Image$$RW_IRAM1$$ZI$$Length;
//extern void __scatterload_copy();
extern void __scatterload_copy();
extern void __scatterload_zeroinit();


region_init_t g_region_origin[2];
void System_wakeup_section_init()
{
    g_region_origin[0].load_region = (uint32_t)&Load$$RW_IRAM1$$RW$$Base;
    g_region_origin[0].exe_region = (uint32_t)&Image$$RW_IRAM1$$RW$$Base;
    g_region_origin[0].length = (uint32_t)&Image$$RW_IRAM1$$RW$$Length;
    // Convert to ARM.
    g_region_origin[0].callback = (uint32_t)(__scatterload_copy) ^ 0x1;
    g_region_origin[1].load_region = (uint32_t)&Load$$RW_IRAM1$$ZI$$Base;
    g_region_origin[1].exe_region = (uint32_t)&Image$$RW_IRAM1$$ZI$$Base;
    g_region_origin[1].length = (uint32_t)&Image$$RW_IRAM1$$ZI$$Length;
    // Convert to ARM.
    g_region_origin[1].callback = (uint32_t)(__scatterload_zeroinit) ^ 0x1;
}
#endif

/**
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**)
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) HSI_VALUE is a constant defined in BF0_CP_hal.h file (default value
  *             8 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSE_VALUE is a constant defined in BF0_CP_hal.h file (default value
  *              8 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{

}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

